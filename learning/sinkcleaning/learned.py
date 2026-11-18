"""Reload the saved PGLfD parameters; no pickle or retraining required."""
import json
from pathlib import Path
import numpy as np
from pglfd.geom.se3 import multiply_poses
from pglfd.manif.product import ProductManifold
from pglfd.model.kernel import MaternKernel
from pglfd.model.kmp import KMP
from pglfd.model.tp import TaskParameterizedModel


def load_kmp(path):
    with np.load(path, allow_pickle=False) as d:
        input_manifold = ProductManifold.from_string("R1")
        return KMP(input_manifold, ProductManifold.from_string("R3*SO3"), d["phase"], d["means"],
                   MaternKernel(input_manifold, float(d["length_scale"])), d["covariances"],
                   lambda1=float(d["lambda1"]), lambda2=float(d["lambda2"]))


def predict_saved(folder, *, kind="tpkmp", frames=None, n=60):
    """Return sink-frame poses; task frames are XYZ + XYZW in that same frame."""
    folder = Path(folder)
    report = json.loads((folder/"report.json").read_text())
    if kind == "kmp":
        model = load_kmp(folder/"kmp.npz")
    elif kind == "tpkmp":
        frames = report["task_frames_in_sink"] if frames is None else frames
        origins = []
        for name in ("start", "end"):
            pose = np.asarray(frames[name], dtype=float)
            if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-6):
                raise ValueError(f"{name} task frame must be XYZ plus unit XYZW quaternion")
            origins.append((name, pose))
        model = TaskParameterizedModel(origins, [load_kmp(folder/f"{k}.npz") for k in ("start", "end")])
    else:
        raise ValueError("Model must be kmp or tpkmp")
    poses = np.array([g.mu for g in model(np.linspace(0, 1, n)[:, None])])
    if kind == "kmp":
        poses = multiply_poses(np.array(json.loads((folder/"anchor.json").read_text())), poses)
    if not np.isfinite(poses).all():
        raise ValueError("Nonfinite learned prediction")
    return poses, report
