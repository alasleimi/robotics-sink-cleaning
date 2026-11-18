"""Play lab JSON joints or a learned CSV path in the NAURU simulator.

This is kinematic inspection, preserving recorded timestamps/joints. It does
not send robot commands or claim to reproduce contact forces/controller motion.
Space pauses, R restarts, arrow keys scrub by one second. Ends paused, stays open.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import mujoco
from sinkcleaning.lab import read_trajectory, reconstruct
from sinkcleaning.demonstrations import load

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT/"data/lab/2026-11-11"
sys.path.insert(0, str(ROOT/"simulation/adapter"))


def load_playback(model, source, cleaning_only=False):
    source = Path(source)
    if not source.exists():
        source = LAB/"raw"/f"traj_2026-11-11_{source}.json"
    if source.suffix == ".json":
        _, t, q = read_trajectory(source)
        poses = reconstruct(model, q)
        report = json.loads((LAB/"analysis/recordings.json").read_text())
        entry = next((r for r in report if Path(r["source"]).name == source.name), None)
        other_robot = entry is not None and entry.get("setup") == "different_robot_test"
        if cleaning_only:
            if entry is None or "used_interval_s" not in entry:
                raise ValueError("No accepted cleaning interval for this recording; omit --cleaning-only to inspect all samples")
            a, b = entry["used_interval_s"]
            keep = (t >= a) & (t <= b)
            t, q, poses = t[keep], q[keep], poses[keep]
        kind = "recorded joints and timing"
    else:
        rows, meta = load(source)
        other_robot = meta.get("setup") == "different_robot_test"
        t, poses = rows[:, 0], rows[:, 1:8]
        if meta.get("joint_positions_valid", True) and np.isfinite(rows[:, 8:15]).all():
            q = rows[:, 8:15]
            kind = "imported recorded joints and timing"
        else:
            # Continuous IK seeds preserve the simulated elbow where possible.
            # These are new simulated joints, never claimed to be measured joints.
            q = []
            seed = model.d.qpos[model.qpos].copy()
            for pose in poses:
                seed = np.asarray(model.inverse(pose, seed, "tcp"))
                q.append(seed)
            q = np.asarray(q)
            if np.max(np.abs(np.diff(q, axis=0))) > .4:
                raise ValueError("Learned path has an IK joint jump; inspect the path before playback")
            kind = "learned pose path with simulated IK; timing is illustrative"
    if other_robot:
        kind += "; DIFFERENT ROBOT TEST: robot/tool geometry assumed, displayed sink does not apply"
    excess = max(0, np.max(q-model.limits[:, 1]), np.max(model.limits[:, 0]-q))
    if excess:
        print(f"Recorded samples exceed nominal model limits by {np.rad2deg(excess):.2f} deg; displayed unchanged, not executable motion.")
    return t-t[0], q, poses, kind


def at_time(t, q, seconds):
    return np.array([np.interp(seconds, t, q[:, i]) for i in range(7)])


def camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [.46, -.03, .23]
    cam.distance = 1.75; cam.azimuth = 135; cam.elevation = -35
    return cam


def draw_path(scene, poses):
    points = poses[np.unique(np.linspace(0, len(poses)-1, min(180, len(poses))).astype(int)), :3]
    for a, b in zip(points, points[1:]):
        if scene.ngeom >= scene.maxgeom or np.linalg.norm(a-b) < 1e-8:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                           np.array([.1, .6, .85, .8], dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, .0018, a, b)
        scene.ngeom += 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", nargs="?", default="18-40-01", help="HH-MM-SS, raw JSON file, or learned recording folder")
    p.add_argument("--cleaning-only", action="store_true")
    p.add_argument("--speed", type=float, default=1.)
    p.add_argument("--headless", action="store_true", help="Validate/load without opening a window")
    p.add_argument("--snapshot", type=Path, help="Save an offscreen simulator image")
    args = p.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        p.error("--speed must be positive and finite")
    if sys.platform == "darwin" and not args.headless and not os.environ.get("LAB_MJPYTHON"):
        os.environ["LAB_MJPYTHON"] = "1"
        os.execv(str(Path(sys.executable).with_name("mjpython")), ["mjpython", str(Path(__file__).resolve()), *sys.argv[1:]])
    from model import Model
    model = Model(scene="nauru")
    try:
        t, q, poses, kind = load_playback(model, args.source, args.cleaning_only)
        model.m.geom_rgba[model.m.geom("TeachTarget_visual").id, 3] = 0
        print(f"Loaded {len(t)} samples, {t[-1]:.2f}s: {kind}.", flush=True)
        model.d.qpos[model.qpos] = q[len(q)//2]
        mujoco.mj_forward(model.m, model.d)
        if args.snapshot:
            from PIL import Image
            model.m.vis.global_.offwidth = 1100; model.m.vis.global_.offheight = 800
            options = mujoco.MjvOption(); options.geomgroup[0] = 0; options.sitegroup[:] = 0
            with mujoco.Renderer(model.m, height=800, width=1100) as renderer:
                renderer.update_scene(model.d, camera=camera(), scene_option=options)
                draw_path(renderer.scene, poses)
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(renderer.render()).save(args.snapshot)
        if args.headless:
            return
        from mujoco import viewer
        state = {"time": 0., "playing": True}

        def key(code):
            if code == 32: state["playing"] = not state["playing"]
            elif code in (82, 114): state.update(time=0., playing=True)
            elif code in (262, 263): state["time"] = float(np.clip(state["time"]+(1 if code == 262 else -1), 0, t[-1]))

        model.viewer = viewer.launch_passive(model.m, model.d, key_callback=key)
        with model.viewer.lock():
            model.viewer.opt.geomgroup[0] = 0; model.viewer.opt.sitegroup[:] = 0
            cam = camera()
            model.viewer.cam.lookat[:] = cam.lookat
            for name in ("distance", "azimuth", "elevation"): setattr(model.viewer.cam, name, getattr(cam, name))
            draw_path(model.viewer.user_scn, poses)
        print("SPACE pause/resume | R restart | left/right scrub 1s | close viewer to exit", flush=True)
        previous = time.perf_counter()
        while model.viewer.is_running():
            now = time.perf_counter()
            if state["playing"]:
                state["time"] = min(t[-1], state["time"]+(now-previous)*args.speed)
                if state["time"] >= t[-1]: state["playing"] = False
            previous = now
            with model.viewer.lock():
                model.d.qpos[model.qpos] = at_time(t, q, state["time"])
                model.d.qvel[:] = 0
                model.d.time = state["time"]
                mujoco.mj_forward(model.m, model.d)
            model.viewer.sync()
            time.sleep(.01)
    finally:
        model.close()


if __name__ == "__main__":
    main()
