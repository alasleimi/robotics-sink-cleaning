"""Fit five motion classes using PGLfD KMP and start/end-frame TP-KMP.

Cross-validation holds out whole recordings. Final models use all accepted takes.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from pglfd.model.tp import TaskParameterizedModel
from pglfd.geom.se3 import multiply_poses
from sinkcleaning.demonstrations import load, in_frame, save_prediction
from sinkcleaning.lab import path_phase
from train_kmp import fit, means

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT/"data/lab/2026-11-11"


def fit_models(training, phase, folder=None):
    # The tool points down, near the SO(3) identity chart's 180-degree cut.
    # PGLfD's global log unwrapping can otherwise introduce large path spikes
    # through position/orientation cross-covariance. Use one fixed reference
    # orientation, estimated ONLY from the training set, to stay off that cut.
    anchor = np.r_[np.zeros(3), Rotation.from_quat(np.concatenate(training)[:, 3:]).mean().as_quat()]
    models = [fit([in_frame(d, anchor) for d in training], phase, folder/"kmp.npz" if folder else None)]
    for name, index in (("start", 0), ("end", -1)):
        # Task axes follow the fixed sink/chart orientation, not how a human
        # happened to roll the sponge at an endpoint. Tool roll is an output.
        local = [in_frame(d, np.r_[d[index, :3], anchor[3:]]) for d in training]
        models.append(fit(local, phase, folder/f"{name}.npz" if folder else None))
    if folder:
        (folder/"anchor.json").write_text(json.dumps(anchor.tolist()))
    return models, anchor


def predict(models, phase, frames):
    regressors, anchor = models
    return {"kmp": multiply_poses(anchor, means(regressors[0], phase)),
            "tpkmp": means(TaskParameterizedModel(list(zip(("start", "end"), frames)), regressors[1:]), phase)}


def errors(prediction, truth):
    distance = np.linalg.norm(prediction[:, :3]-truth[:, :3], axis=1)*1000
    angle = (Rotation.from_quat(prediction[:, 3:])*Rotation.from_quat(truth[:, 3:]).inv()).magnitude()
    return {"position_rmse_mm": float(np.sqrt(np.mean(distance**2))),
            "position_max_mm": float(distance.max()),
            "orientation_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(angle**2))))}


def train(recordings, output):
    report = json.loads((LAB/"analysis/recordings.json").read_text())
    groups = defaultdict(list)
    for entry in report:
        if not entry["train"]:
            continue
        rows, meta = load(recordings/entry["id"])
        start, end = entry["used_interval_s"]
        rows = rows[(rows[:, 0] >= start-1e-7) & (rows[:, 0] <= end+1e-7)]
        phase, poses = path_phase(rows[:, 1:8])
        groups[entry["motion"]].append((entry["id"], in_frame(poses, meta["sink_pose"]), meta, end-start))
    output.mkdir(parents=True, exist_ok=False)
    summaries = {}
    for name, demos in sorted(groups.items()):
        folder = output/name; folder.mkdir()
        paths = [d[1] for d in demos]
        folds = []
        for index, (ident, heldout, _, _) in enumerate(demos):
            training = paths[:index]+paths[index+1:]
            fitted = fit_models(training, phase)
            frames = [np.r_[heldout[i, :3], fitted[1][3:]] for i in (0, -1)]
            predictions = predict(fitted, phase, frames)
            folds.append({"held_out": ident, "training": [d[0] for i, d in enumerate(demos) if i != index],
                          "metrics": {k: errors(v, heldout) for k, v in predictions.items()}})
        models = fit_models(paths, phase, folder)
        frames = [np.r_[np.mean([d[i, :3] for d in paths], axis=0), models[1][3:]] for i in (0, -1)]
        predictions = predict(models, phase, frames)
        meta = demos[0][2]
        duration = float(np.median([d[3] for d in demos]))
        for label, poses in predictions.items():
            base = multiply_poses(np.array(meta["sink_pose"]), poses)
            save_prediction(folder/label, base, meta, label, duration=duration)
        # A real parameter-change check: rigidly move both task frames 30 mm.
        shifted = [f.copy() for f in frames]
        for f in shifted:
            f[:3] += [.03, -.02, .01]
        moved = predict(models, phase, shifted)["tpkmp"]
        equivariance = float(np.max(np.linalg.norm(moved[:, :3]-predictions["tpkmp"][:, :3]-[.03, -.02, .01], axis=1)))
        if equivariance > 1e-5:
            raise AssertionError("TP-KMP did not respond consistently to translated task frames")
        average = {kind: {metric: float(np.mean([f["metrics"][kind][metric] for f in folds]))
                          for metric in folds[0]["metrics"][kind]} for kind in predictions}
        result = {"motion": name, "training_recordings": [d[0] for d in demos], "training_count": len(demos),
                  "phase": "Smoothed TCP arc length; pauses/speed are not learned", "output_manifold": "R3*SO3",
                  "kmp_chart": "Fixed training-mean orientation in sink coordinates avoids the SO3 180-degree log cut; not a per-test task frame",
                  "task_frames_in_sink": {"start": frames[0].tolist(), "end": frames[1].tolist()},
                  "sink_pose": meta["sink_pose"], "duration_s": duration,
                  "task_frame_definition": "Origins at cleaning start/end positions; both axes fixed relative to sink by training reference orientation. Tool orientation is learned, not supplied from held-out endpoints.",
                  "evaluation": "Leave-one-recording-out; KMP gets sink frame, TP-KMP also gets held-out start/end positions. Different conditioning, not a fair algorithm ranking.",
                  "limitations": "Same estimated sink pose/session; no evidence of transfer to a different sink placement. Two-example classes have only one training take per fold; covariance is regularized, not well estimated.",
                  "folds": folds, "mean_fold_metrics": average,
                  "task_frame_translation_check_max_error_m": equivariance,
                  "final_models": "Refit using ALL listed recordings; final paths are not held-out predictions"}
        (folder/"report.json").write_text(json.dumps(result, indent=2))
        plot(paths, predictions, name, folder/"paths.png")
        summaries[name] = {"recordings": len(demos), "mean_fold_metrics": average}
        print(name, json.dumps(summaries[name]), flush=True)
    (output/"summary.json").write_text(json.dumps(summaries, indent=2))


def plot(paths, predictions, name, output):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for demo in paths:
        axes[0].plot(demo[:, 0]*1000, demo[:, 1]*1000, color=".75", lw=1)
        axes[1].plot(np.linspace(0, 1, len(demo)), demo[:, 2]*1000, color=".75", lw=1)
    for label, p in predictions.items():
        axes[0].plot(p[:, 0]*1000, p[:, 1]*1000, label=label.upper())
        axes[1].plot(np.linspace(0, 1, len(p)), p[:, 2]*1000, label=label.upper())
    axes[0].axis("equal"); axes[0].legend(); axes[1].legend()
    axes[0].set(xlabel="Sink X (mm)", ylabel="Sink Y (mm)", title=name.replace("_", " "))
    axes[1].set(xlabel="Path progress", ylabel="Sink Z (mm)", title="Height; grey = demonstrations")
    fig.tight_layout(); fig.savefig(output, dpi=150); plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recordings", type=Path, default=ROOT/"data/recordings/lab-2026-11-11")
    p.add_argument("--output", type=Path, default=LAB/"models")
    args = p.parse_args(); train(args.recordings, args.output)
