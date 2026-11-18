"""Audit lab labels and sink placement independently of the learning models.

Uses recorded Task 11 TCP where available; otherwise joint FK. Does not change
classification, training intervals, models or the simulator scene.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.distance import squareform
from scipy.spatial.transform import Rotation
from sinkcleaning.lab import read_trajectory, reconstruct, measured_poses, path_phase, trim_interval
from sinkcleaning.mesh_distance import MeshSurface

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT/"data/lab/2026-11-11"
sys.path.insert(0, str(ROOT/"simulation/adapter"))


def percentiles(values, levels=(50, 90, 100), scale=1000):
    finite = np.asarray(values)[np.isfinite(values)]
    return (np.percentile(finite, levels)*scale).tolist() if len(finite) else None


def load_records():
    from model import Model
    model = Model()
    records = {}
    try:
        for entry in json.loads((LAB/"classification.json").read_text())["recordings"]:
            if entry.get("setup") == "different_robot_test":
                continue
            doc, t, q = read_trajectory(LAB/"raw"/f"traj_2026-11-11_{entry['id']}.json")
            fk, actual = reconstruct(model, q), measured_poses(doc)
            poses = actual if actual is not None else fk
            ids = trim_interval(t, poses, entry) if entry["train"] else np.arange(len(t))
            records[entry["id"]] = dict(entry=entry, t=t, q=q, poses=poses, ids=ids,
                                       measured=actual is not None, fk=fk,
                                       path=path_phase(poses[ids], 120)[1])
    finally:
        model.close()
    return records


def band(record):
    return record["entry"]["motion"].split("_")[1]


def cluster_bands(rims, lower=-.23, upper=.17):
    """Compare X/Z at shared robot Y, without a sink model or fitted transform."""
    y = np.linspace(lower, upper, 80)
    curves, names = [], list(rims)
    for r in rims.values():
        p = r["poses"][r["ids"], :3]
        p = p[np.argsort(p[:, 1])]
        if p[0, 1] > lower or p[-1, 1] < upper:
            raise ValueError("Common Y interval is not covered by every sweep")
        curves.append(np.column_stack([np.interp(y, p[:, 1], p[:, j]) for j in (0, 2)]))
    curves = np.asarray(curves)
    distance = np.sqrt(np.mean(np.sum((curves[:, None]-curves[None, :])**2, axis=3), axis=2))
    clusters = fcluster(linkage(squareform(distance), method="average"), 2, criterion="maxclust")
    rows = []
    for n, ident in enumerate(names):
        own = [j for j, other in enumerate(names) if j != n and band(rims[other]) == band(rims[ident])]
        other = [j for j, other in enumerate(names) if band(rims[other]) != band(rims[ident])]
        a, b = float(np.mean(distance[n, own])), float(np.mean(distance[n, other]))
        nn = int(np.argsort(distance[n])[1])
        rows.append(dict(id=ident, original_band=band(rims[ident]), cluster=int(clusters[n]),
                         nearest=names[nn], nearest_band=band(rims[names[nn]]),
                         same_band_mean_distance_mm=a*1000, other_band_mean_distance_mm=b*1000,
                         silhouette=(b-a)/max(a, b), leave_one_out_band_matches=a < b))
    return rows, curves, y


class RimFit:
    def __init__(self, rims, rim):
        self.rims = rims
        self.paths = {i: r["path"].copy() for i, r in rims.items()}
        self.trees = {k: cKDTree(np.asarray(rim[k])[:, :2]) for k in ("inner", "outer")}
        self.trees["either"] = cKDTree(np.concatenate([np.asarray(rim[k])[:, :2] for k in ("inner", "outer")]))

    def points(self, ident, full=False):
        p = self.paths[ident][:, :3]
        return p if full else p[(p[:, 1] > -.24) & (p[:, 1] < .18)]

    def residual(self, x, names, mode="labels", full=False):
        result = []
        rotation = Rotation.from_euler("z", x[2]).as_matrix()[:2, :2]
        for ident in names:
            p = self.points(ident, full)
            label = band(self.rims[ident])
            key = label if mode == "labels" else ("outer" if label == "inner" else "inner") if mode == "swap" else mode
            result.append(self.trees[key].query((p[:, :2]-x[:2])@rotation)[0])
        return np.concatenate(result)

    def solve(self, names, mode="labels", seed=(.71, -.04, np.pi/2)):
        return least_squares(lambda x: self.residual(x, names, mode), seed,
                             loss="soft_l1", f_scale=.01, max_nfev=160).x

    def pose(self, x, names):
        height = float(np.median(np.concatenate([self.points(i)[:, 2] for i in names])))-.17
        return np.r_[x[:2], height, Rotation.from_euler("z", x[2]).as_quat()]


def surface_report(surface, poses, placement):
    local = (poses[:, :3]-placement[:3])@Rotation.from_quat(placement[3:]).as_matrix()
    distance, height = surface.query(local)
    return dict(distance_mm_median_p90_max=percentiles(distance),
                vertical_gap_mm_p10_median_p90=percentiles(local[:, 2]-height, (10, 50, 90)),
                outside_vertical_footprint_fraction=float(np.isnan(height).mean()))


def audit(output):
    output.mkdir(parents=True, exist_ok=True)
    records = load_records()
    rims = {i: r for i, r in records.items() if r["entry"]["train"] and r["entry"]["motion"].startswith("rim_")}
    loops = {i: r for i, r in records.items() if r["entry"]["motion"] == "bowl_loop"}
    rim = json.loads((ROOT/"simulation/assets/nauru/rim.json").read_text())
    scene = json.loads((ROOT/"simulation/scenes/nauru.json").read_text())
    original = np.asarray(scene["sink_pose"])
    surface = MeshSurface.from_obj(ROOT/"simulation/assets/nauru/nauru.obj")
    clustering, curves, y = cluster_bands(rims)
    names = list(rims)
    fit = RimFit(rims, rim)
    checks = []
    groups = {"all": names, "earlier_joint_only": [i for i in names if not rims[i]["measured"]],
              "later_measured_tcp": [i for i in names if rims[i]["measured"]]}
    for name, selected in groups.items():
        x = fit.solve(selected)
        held = [i for i in names if i not in selected]
        checks.append(dict(group=name, fitted=selected, held_out=held, pose=fit.pose(x, selected).tolist(),
                           yaw_deg=float(np.rad2deg(x[2])),
                           fit_xy_mm_median_p90_max=percentiles(fit.residual(x, selected)),
                           heldout_xy_mm_median_p90_max=percentiles(fit.residual(x, held)) if held else None,
                           full_path_xy_mm_median_p90_max=percentiles(fit.residual(x, selected, full=True))))
    alternatives = []
    for mode in ("either", "labels", "swap", "inner", "outer"):
        # No yaw bounds: deliberately challenge the original front-facing assumption.
        for deg in (range(0, 360, 30) if mode in ("either", "labels", "swap") else (90,)):
            x = fit.solve(names, mode, (.71, -.04, np.deg2rad(deg)))
            pose = fit.pose(x, names)
            residual = fit.residual(x, names, mode)
            alternatives.append(dict(mode=mode, start_yaw_deg=deg, pose=pose.tolist(),
                                     yaw_deg=float(np.rad2deg(x[2]) % 360),
                                     xy_rmse_mm=float(np.sqrt(np.mean(residual**2))*1000),
                                     xy_mm_median_p90_max=percentiles(residual),
                                     independent_bowl={i: surface_report(surface, r["path"], pose) for i, r in loops.items()}))
    per_take = {}
    for ident, r in records.items():
        path = r["path"]
        row = dict(motion=r["entry"]["motion"], train=r["entry"]["train"],
                   pose_source="measured Task 11 TCP" if r["measured"] else "joint FK; 202 mm TCP",
                   z_mm_min_median_max=percentiles(path[:, 2], (0, 50, 100)),
                   xyz_range_mm=(np.ptp(path[:, :3], axis=0)*1000).tolist(),
                   interval_s=r["t"][r["ids"][[0, -1]]].tolist())
        if r["measured"]:
            row["fk_vs_measured_mm_median_p95_max"] = percentiles(np.linalg.norm(r["poses"][:, :3]-r["fk"][:, :3], axis=1), (50, 95, 100))
        if r["entry"]["train"]:
            row["original_scene_surface"] = surface_report(surface, path, original)
        if ident in rims:
            p = r["poses"][r["ids"]]
            middle = p[(p[:, 1] > -.20) & (p[:, 1] < .14), 2]
            dz = p[:, 2]-np.median(middle)
            a, b = 0, len(p)-1
            while a < b and dz[a] > .025: a += 1
            while b > a and dz[b] > .025: b -= 1
            row["endpoint_height_above_central_median_mm"] = (dz[[0, -1]]*1000).tolist()
            row["candidate_interval_25mm_height_gate_s"] = r["t"][r["ids"][[a, b]]].tolist()
            row["interval_note"] = "Diagnostic only; height gate does not establish contact and has not changed training intervals."
            row["direction_matches"] = bool((p[-1, 1]-p[0, 1] > 0) == r["entry"]["motion"].endswith("plus"))
            x = fit.solve([ident])
            row["individual_fit_xy_mm_yaw_deg"] = [*(x[:2]*1000).tolist(), float(np.rad2deg(x[2]))]
            row["individual_fit_note"] = "Path-specific effective fit, NOT evidence that the physical sink moved."
        if ident in loops:
            xy = path[:, :2]
            row["signed_closed_area_m2"] = float(np.sum(xy[:, 0]*np.roll(xy[:, 1], -1)-xy[:, 1]*np.roll(xy[:, 0], -1))/2)
        per_take[ident] = row
    offsets = []
    saved = {i: p.copy() for i, p in fit.paths.items()}
    for offset in (-.02, 0, .02):
        for i, p in saved.items():
            fit.paths[i] = p.copy()
            fit.paths[i][:, :3] += Rotation.from_quat(p[:, 3:]).apply([0, 0, offset])
        x = fit.solve(names); pose = fit.pose(x, names)
        bowl = {}
        for i, r in loops.items():
            p = r["path"].copy(); p[:, :3] += Rotation.from_quat(p[:, 3:]).apply([0, 0, offset])
            bowl[i] = surface_report(surface, p, pose)
        offsets.append(dict(assumed_tool_z_offset_mm=offset*1000, fitted_pose=pose.tolist(),
                            rim_xy_mm_median_p90_max=percentiles(fit.residual(x, names)), independent_bowl=bowl))
    fit.paths = saved
    report = dict(
        purpose="Retrospective consistency audit; not independent ground truth or hardware calibration",
        setup_status="Ala is unsure whether sink/base/sponge changed during later takes; no fixed-setup assumption confirmed.",
        measured_pose_policy="Use recorded Task 11 TCP directly where available; earlier records use independently checked iiwa 7 FK.",
        limitations=["Only one session, unknown sponge contact point/compression and sink movement",
                     "CAD itself is photo-constrained, not a measured scan of the lab basin",
                     "Labels and intervals were originally selected from these paths; clustering agreement is corroboration, not classification accuracy",
                     "Temporal split is a retrospective robustness check, not a prospectively untouched test set",
                     "Mesh distances are to recorded TCP points; they are not sponge penetration/contact errors"],
        classification=dict(band_checks=clustering, mean_silhouette=float(np.mean([r["silhouette"] for r in clustering])),
                            common_y_m=[float(y[0]), float(y[-1])],
                            interval_sensitivity={f"{lo},{hi}": cluster_bands(rims, lo, hi)[0] for lo, hi in ((-.20, .14), (-.26, .20))}),
        original_scene_pose=original.tolist(), temporal_split=checks, alternative_fits=alternatives,
        contact_offset_sensitivity=offsets, recordings=per_take)
    (output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    plot(records, rims, curves, y, clustering, rim, original, report, output)
    print("Band agreement:", sum(r["leave_one_out_band_matches"] for r in clustering), "/", len(clustering))
    for row in checks: print(row["group"], "held-out XY mm:", row["heldout_xy_mm_median_p90_max"])
    print("Saved", output)


def plot(records, rims, curves, y, clustering, rim, original, report, output):
    colors = {"inner": "#2274a5", "outer": "#e18b20", "bowl": "#399963"}
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    rotation = Rotation.from_quat(original[3:]).as_matrix()
    for key in ("inner", "outer"):
        p = np.asarray(rim[key])@rotation.T+original[:3]
        axes[0, 0].plot(p[:, 0]*1000, p[:, 1]*1000, color=".65", lw=.8)
    for ident, r in records.items():
        if not r["entry"]["train"]: continue
        key = band(r) if ident in rims else "bowl"
        p = r["path"]
        axes[0, 0].plot(p[:, 0]*1000, p[:, 1]*1000, color=colors[key], alpha=.65)
        axes[0, 1].plot(p[:, 1]*1000, p[:, 2]*1000, color=colors[key], alpha=.65)
    axes[0, 0].set(title="Blue: inner band; orange: outer; green: bowl\nGrey: original fitted CAD", xlabel="Robot X (mm)", ylabel="Robot Y (mm)")
    axes[0, 0].set_aspect("equal")
    axes[0, 1].axhline((original[2]+.17)*1000, color=".45", ls="--", label="Estimated rim height")
    axes[0, 1].set(title="Height reveals approaches and lower bowl loops", xlabel="Robot Y (mm)", ylabel="Robot Z (mm)")
    axes[0, 1].legend()
    labels = [r["id"] for r in clustering]; indices = np.arange(len(labels))
    axes[1, 0].barh(indices-.18, [r["same_band_mean_distance_mm"] for r in clustering], .36, label="Other takes in same band", color="#2274a5")
    axes[1, 0].barh(indices+.18, [r["other_band_mean_distance_mm"] for r in clustering], .36, label="Takes in opposite band", color="#e18b20")
    axes[1, 0].set(yticks=indices, yticklabels=labels, xlabel="Mean path distance at common Y (mm)", title="All 12 agree with their band without fitting a sink")
    axes[1, 0].legend(fontsize=8)
    selected = [(i, r) for i, r in report["recordings"].items() if r["train"]]
    for n, (ident, row) in enumerate(selected):
        median, p90, maximum = row["original_scene_surface"]["distance_mm_median_p90_max"]
        color = colors["bowl" if row["motion"] == "bowl_loop" else row["motion"].split("_")[1]]
        axes[1, 1].plot([median, p90], [n, n], color=color, lw=3)
        axes[1, 1].plot(median, n, "o", color=color)
    axes[1, 1].set(yticks=np.arange(len(selected)), yticklabels=[i for i, _ in selected], xlabel="TCP to CAD mesh (mm): dot median, line to p90", title="3D surface consistency; not measured contact error")
    for ax in axes.flat: ax.grid(alpha=.18)
    fig.suptitle("Lab audit: motion groups supported; exact sink/contact calibration unresolved", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .96)); fig.savefig(output/"overview.png", dpi=145); plt.close(fig)
    # Show the raw selected interval and explicit threshold candidates for all sweeps.
    fig, axes = plt.subplots(3, 4, figsize=(15, 10))
    for ax, (ident, r) in zip(axes.flat, rims.items()):
        t, p, ids = r["t"], r["poses"], r["ids"]
        ax.plot(t, p[:, 2]*1000, color=".7", lw=1)
        ax.plot(t[ids], p[ids, 2]*1000, color=colors[band(r)], lw=2)
        a, b = report["recordings"][ident]["candidate_interval_25mm_height_gate_s"]
        ax.axvline(a, color=".3", ls=":"); ax.axvline(b, color=".3", ls=":")
        ax.set(title=ident, xlabel="Seconds", ylabel="TCP height (mm)", ylim=(130, 280)); ax.grid(alpha=.2)
    fig.suptitle("Selected intervals (colour) include some descending approach / raised endpoints\nDotted: diagnostic 25 mm height gate, not verified contact boundaries", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(output/"intervals.png", dpi=130); plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=LAB/"analysis/audit")
    audit(parser.parse_args().output)
