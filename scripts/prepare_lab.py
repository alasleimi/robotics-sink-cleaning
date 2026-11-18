"""Inspect/import the November lab session and estimate the NAURU sink placement."""
import argparse
import hashlib
import io
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sinkcleaning.lab import read_trajectory, reconstruct, measured_poses, trim_interval
from sinkcleaning.demonstrations import HEADER

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT/"data/lab/2026-11-11"
sys.path.insert(0, str(ROOT/"simulation/adapter"))


def fit_sink(records):
    rim = json.loads((ROOT/"simulation/assets/nauru/rim.json").read_text())
    trees = {k: cKDTree(np.array(rim[k])[np.array(rim[k])[:, 1] > -.06, :2])
             for k in ("inner", "outer")}
    points = []
    for entry, t, q, poses, doc in records:
        if not entry["train"] or not entry["motion"].startswith("rim_"):
            continue
        ids = trim_interval(t, poses, entry)
        p = poses[ids, :3]
        # Central front arc is least ambiguous; avoid approach and side endpoints.
        p = p[(p[:, 0] < .72) & (p[:, 1] > -.25) & (p[:, 1] < .18) & (p[:, 2] < .205)]
        # Equal weight per recording, independent of sampling rate/operator pauses.
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
        p = np.column_stack([np.interp(np.linspace(0, d[-1], 60), d, p[:, i]) for i in range(3)])
        points.append((entry["motion"].split("_")[1], p))

    def residual(x, selected):
        rotation = Rotation.from_euler("z", x[2]).as_matrix()[:2, :2]
        return np.concatenate([trees[k].query((p[:, :2]-x[:2]) @ rotation)[0] for k, p in selected])

    def solve(selected):
        return least_squares(residual, [.71, -.045, np.pi/2], args=(selected,),
                             loss="soft_l1", f_scale=.015,
                             bounds=([.5, -.2, 1], [.9, .1, 2.2])).x
    x = solve(points)
    rim_z = float(np.median(np.concatenate([p[:, 2] for _, p in points])))
    pose = [*x[:2].tolist(), rim_z-.17, *Rotation.from_euler("z", x[2]).as_quat().tolist()]
    loo = np.array([solve(points[:i]+points[i+1:]) for i in range(len(points))])
    result = {
        "sink_pose": pose, "pose_convention": "metres, XYZW; original STEP bottom-origin in robot base",
        "product": "User-supplied NAURU 575x425 v4 photo-constrained STEP",
        "status": "Trajectory-derived estimate, NOT measured calibration",
        "rim_z_robot_base_m": rim_z, "yaw_deg": float(np.rad2deg(x[2])),
        "fit_residual_mm_median_p90_max": (np.percentile(residual(x, points), [50, 90, 100])*1000).tolist(),
        "leave_one_recording_out_xy_range_mm": (np.ptp(loo[:, :2], axis=0)*1000).tolist(),
        "leave_one_recording_out_yaw_range_deg": float(np.rad2deg(np.ptp(loo[:, 2]))),
        "method": "Robust XY fit of two observed arc bands to CAD inner/outer front rim; median central-arc TCP height",
        "assumptions": ["Sink upright and stationary for later recordings", "Two arc bands correspond approximately to inner/outer rim",
                        "Recorded course TCP approximates contact point; sponge size/compression and true TCP not calibrated"],
        "uncertainty": "Centimetre-scale systematic offsets remain; fit scatter/LOO spread are not calibration confidence intervals.",
        "floor": "Illustrative 850 mm rim height only; absolute lab floor/robot pedestal height unknown",
    }
    return result, rim


def prepare(output):
    from model import Model
    manifest = json.loads((LAB/"classification.json").read_text())
    model = Model()
    records, reports = [], []
    try:
        for entry in manifest["recordings"]:
            path = LAB/"raw"/f"traj_2026-11-11_{entry['id']}.json"
            document, t, q = read_trajectory(path)
            poses = reconstruct(model, q)
            actual = measured_poses(document)
            dt = np.diff(t)
            report = {**entry, "source": str(path.relative_to(ROOT)).replace("\\", "/"),
                      "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "samples": len(t),
                      "duration_s": float(t[-1]-t[0]), "dt_ms_min_median_max": (np.percentile(dt, [0, 50, 100])*1000).tolist(),
                      "tcp_extent_mm": (np.ptp(poses[:, :3], axis=0)*1000).tolist(),
                      "max_joint_limit_excess_deg": float(np.rad2deg(max(0, np.max(q-model.limits[:, 1]), np.max(model.limits[:, 0]-q)))),
                      "missing_optional_channels": [k for k in ("tcp", "torque", "extTorque", "extForce")
                                                    if not all(k in s for s in document["samples"])]}
            if entry.get("setup") == "different_robot_test":
                report["kinematics_note"] = "Actual robot/tool unknown. TCP geometry and limit comparisons assume the sink iiwa 7 R800 with 202 mm TCP; illustrative only."
            if actual is not None:
                delta = np.linalg.norm(poses[:, :3]-actual[:, :3], axis=1)*1000
                angle = (Rotation.from_quat(poses[:, 3:])*Rotation.from_quat(actual[:, 3:]).inv()).magnitude()
                report["fk_vs_measured_tcp_mm_median_p95_max"] = np.percentile(delta, [50, 95, 100]).tolist()
                report["fk_vs_measured_orientation_deg_p95"] = float(np.rad2deg(np.percentile(angle, 95)))
            if all("extForce" in s for s in document["samples"]):
                wrench = np.array([s["extForce"] for s in document["samples"]])
                if wrench.shape == (len(t), 6) and np.isfinite(wrench).all():
                    report["raw_external_force_N_median_p95_max"] = np.percentile(np.linalg.norm(wrench[:, 3:], axis=1), [50, 95, 100]).tolist()
                    report["raw_external_moment_Nm_median_p95_max"] = np.percentile(np.linalg.norm(wrench[:, :3], axis=1), [50, 95, 100]).tolist()
                    report["wrench_note"] = "Raw Task 11 sensor readings; include guiding hand loads, not isolated sponge contact. Frame/reference unspecified. Not used for contact labels or force learning."
            ids = trim_interval(t, poses, entry) if entry["train"] else None
            if ids is not None:
                report["used_interval_s"] = t[ids[[0, -1]]].tolist()
                report["training_samples"] = len(ids)
            records.append((entry, t, q, poses, document)); reports.append(report)
        scene, rim = fit_sink(records)
        (ROOT/"simulation/scenes/nauru.json").write_text(json.dumps(scene, indent=2))
        output.mkdir(parents=True, exist_ok=True)
        for (entry, t, q, poses, document), report in zip(records, reports):
            folder = output/entry["id"]
            folder.mkdir(exist_ok=True)
            rows = np.full((len(t), 22), np.nan)
            rows[:, 0] = t; rows[:, 1:8] = poses; rows[:, 8:15] = q; rows[:, 21] = 0
            # extForce frame/reference is not explicit in Task 11. Retain raw
            # values in the JSON; do not mislabel them as base-frame TCP wrench.
            text = io.StringIO()
            np.savetxt(text, rows, delimiter=",", header=HEADER, comments="", fmt="%.9g")
            # Java Double.parseDouble accepts NaN, not NumPy's lowercase nan.
            (folder/"samples.csv").write_text(text.getvalue().replace("nan", "NaN"))
            meta = {"schema": 2, "complete": True, "source": "robot_task9_or_task11", "pose_frame": "robot_base",
                    "pose_source": "joint FK; 202 mm course TCP; measured Task 11 poses independently checked",
                    "units": {"position": "m", "orientation": "quaternion_xyzw", "time": "s", "joints": "rad"},
                    "tcp": [0, 0, .202, 0, 0, 0, 1], "tool_template": "EXERCISE_TOO (lab source)",
                    "sink_pose": scene["sink_pose"], "sink_pose_status": scene["status"],
                    "joint_positions_valid": True, "wrench_valid": False, "read_seconds_valid": False,
                    "read_seconds_note": "0 placeholder; not measured by the lab recorder",
                    "raw_source": report["source"], "sha256": report["sha256"],
                    "motion": entry["motion"], "cleaning_interval_s": report.get("used_interval_s")}
            if entry.get("setup") == "different_robot_test":
                meta.update(setup=entry["setup"], pose_source=report["kinematics_note"],
                            tool_template="Unknown; 202 mm TCP assumed for visualization",
                            sink_pose=None, sink_pose_status="Different robot test; sink setup does not apply")
            (folder/"metadata.json").write_text(json.dumps(meta, indent=2))
        results = LAB/"analysis"; results.mkdir(exist_ok=True)
        (results/"recordings.json").write_text(json.dumps(reports, indent=2))
        (results/"sink-placement.json").write_text(json.dumps(scene, indent=2))
        plot_overview(records, scene, rim, results/"trajectories.png")
        print(f"Imported {len(records)} recordings to {output}")
        print(f"Training: {sum(r['train'] for r in reports)} recordings. Reports: {results}")
        print("Sink estimate:", json.dumps(scene["sink_pose"]))
    finally:
        model.env.close()


def plot_overview(records, scene, rim, output):
    fig, axes = plt.subplots(6, 4, figsize=(16, 20))
    rotation = Rotation.from_quat(scene["sink_pose"][3:]).as_matrix()[:2, :2]
    for ax, (entry, t, q, poses, _) in zip(axes.flat, records):
        other_robot = entry.get("setup") == "different_robot_test"
        for kind in (() if other_robot else ("inner", "outer")):
            outline = np.array(rim[kind])[:, :2] @ rotation.T + scene["sink_pose"][:2]
            ax.plot(*outline.T, color=".8", lw=.5)
        ax.plot(*poses[:, :2].T, color=".65", lw=.7)
        ids = trim_interval(t, poses, entry) if entry["train"] else np.arange(len(t))
        ids = ids[::max(1, len(ids)//150)]
        ax.scatter(*poses[ids, :2].T, c=t[ids], cmap="viridis", s=3)
        label = "other robot test (assumed FK)" if other_robot else entry["motion"].replace("_", " ")
        ax.set_title(entry["id"]+"\n"+label, fontsize=9)
        ax.set(xlim=(.25, 1), ylim=(-.44, .34), xlabel="Robot X (m)", ylabel="Robot Y (m)")
        ax.set_aspect("equal"); ax.grid(alpha=.15)
    fig.suptitle("24 lab recordings | purple → yellow = time; grey = approach / CAD rim\n"
                 "Motion labels and sink placement are geometric interpretations, not measured calibration", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .965)); fig.savefig(output, dpi=135); plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT/"data/recordings/lab-2026-11-11")
    prepare(p.parse_args().output)
