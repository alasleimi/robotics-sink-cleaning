"""Read Task 9 / Task 11 recordings without inventing missing telemetry."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.ndimage import gaussian_filter1d


def read_trajectory(path):
    document = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if document.get("unit") != "rad":
        raise ValueError("Expected joint angles in radians")
    samples = document.get("samples", [])
    t = np.array([s["t"] for s in samples], dtype=float)
    q = np.array([s["q"] for s in samples], dtype=float)
    if len(t) < 2 or q.shape != (len(t), 7):
        raise ValueError("Need at least two samples with seven joints")
    if not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError("Nonfinite required joint/time sample")
    if t[0] < 0 or np.any(np.diff(t) <= 0):
        raise ValueError("Timestamps must increase")
    return document, t, q


def reconstruct(model, q):
    """Kinematic observation only: retain even out-of-limit measured samples."""
    import mujoco
    data = model.ik  # Already isolated from the live simulation state.
    poses = []
    for joints in q:
        data.qpos[model.qpos] = joints
        mujoco.mj_kinematics(model.m, data)
        poses.append(model.pose("tcp", data))
    return np.array(poses)


def measured_poses(document):
    rows = [s.get("tcp") for s in document["samples"]]
    if any(row is None for row in rows):
        return None
    tcp = np.asarray(rows, dtype=float)
    if tcp.shape != (len(rows), 6) or not np.isfinite(tcp).all():
        return None
    # Task 11 explicitly writes Sunrise A/B/C in degrees (intrinsic Z/Y/X).
    return np.c_[tcp[:, :3]/1000, Rotation.from_euler("ZYX", tcp[:, 3:], degrees=True).as_quat()]


def path_phase(poses, n=60):
    """Align cleaning geometry by smoothed TCP arc length, not operator pauses.

    Preserve the unsmoothed positions/orientations for interpolation. This is a
    path model; the recording's speed is available separately in the raw file.
    """
    smooth = gaussian_filter1d(poses[:, :3], 2, axis=0, mode="nearest")
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(smooth, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-9]
    distance, poses = distance[keep], poses[keep]
    if len(distance) < 2 or distance[-1] < .02:
        raise ValueError("Less than 2 cm of useful cleaning motion")
    progress = distance/distance[-1]
    phase = np.linspace(0, 1, n)
    position = np.column_stack([np.interp(phase, progress, poses[:, i]) for i in range(3)])
    orientation = Slerp(progress, Rotation.from_quat(poses[:, 3:]))(phase).as_quat()
    return phase, np.c_[position, orientation]


def trim_interval(t, poses, entry):
    if "interval_s" in entry:
        start, end = entry["interval_s"]
    elif entry.get("motion", "").startswith("rim_"):
        # Reviewed full sweeps traverse between the two Y extrema. No reversal
        # is performed: the two directions remain separate learning classes.
        smooth = gaussian_filter1d(poses[:, 1], 5)
        start, end = sorted(t[[np.argmin(smooth), np.argmax(smooth)]])
    else:
        return None
    ids = np.flatnonzero((t >= start) & (t <= end))
    if len(ids) < 3:
        raise ValueError("Cleaning interval contains too few samples")
    return ids
