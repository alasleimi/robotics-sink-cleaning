"""Regress real lab formats, robot FK, learned path continuity and CAD cavity."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import pytest
import mujoco
from pglfd.geom.se3 import multiply_poses
from sinkcleaning.lab import read_trajectory, reconstruct, measured_poses, path_phase
from sinkcleaning.learned import predict_saved
from sinkcleaning.demonstrations import load

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"simulation/adapter"))
sys.path.insert(0, str(ROOT/"scripts"))
from model import Model
from play_lab import at_time, load_playback
LAB = ROOT/"data/lab/2026-11-11"


@pytest.fixture(scope="module")
def robot():
    model = Model(scene="nauru")
    yield model
    model.close()


def test_all_raw_files_preserved_and_task11_agrees_with_simulator(robot):
    before = robot.d.qpos.copy()
    report = json.loads((LAB/"analysis/recordings.json").read_text())
    assert len(report) == 24
    for entry in report:
        path = ROOT/entry["source"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
        doc, t, q = read_trajectory(path)
        np.testing.assert_array_equal(at_time(t, q, t[len(t)//2]), q[len(t)//2])
        actual = measured_poses(doc)
        if actual is not None:
            fk = reconstruct(robot, q[::10])
            error = np.linalg.norm(fk[:, :3]-actual[::10, :3], axis=1)
            assert np.percentile(error, 95) < .0005  # Independent measured TCP evidence.
    np.testing.assert_array_equal(before, robot.d.qpos)


def test_bad_time_rejected_and_missing_telemetry_allowed(tmp_path):
    doc = {"unit": "rad", "samples": [{"t": 0, "q": [0]*7}, {"t": .02, "q": [.1]*7}]}
    path = tmp_path/"sample.json"; path.write_text(json.dumps(doc))
    assert read_trajectory(path)[2].shape == (2, 7)
    assert measured_poses(doc) is None
    doc["samples"][1]["t"] = 0; path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="Timestamps"): read_trajectory(path)


def test_cad_collision_keeps_bowl_open():
    assets = ROOT/"simulation/assets/nauru"
    geometry = json.loads((assets/"geometry.json").read_text())
    size = " ".join(map(str, geometry["hfield_size"]))
    m = mujoco.MjModel.from_xml_string(f'''<mujoco><asset>
      <hfield name="sink" file="{(assets/'nauru.hfield').as_posix()}" size="{size}"/>
      </asset><worldbody><geom type="hfield" hfield="sink"/></worldbody></mujoco>''')
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    def height(x, y):
        hit = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(m, d, np.array([x, y, .4]), np.array([0., 0., -1.]), None, 1, -1, hit)
        assert distance > 0 and hit[0] == 0
        return .4-distance
    assert .055 < height(0, .075) < .11
    assert .165 < height(.27, 0) <= .171


def test_final_learned_paths_reachable_continuous_and_reloadable(robot):
    for folder in sorted((LAB/"models").iterdir()):
        if not folder.is_dir(): continue
        for kind in ("kmp", "tpkmp"):
            rows, metadata = load(folder/kind)
            assert len(metadata["tcp"]) == 7
            assert metadata["joint_positions_valid"] is False
            poses, report = predict_saved(folder, kind=kind)
            base = multiply_poses(np.array(report["sink_pose"]), poses)
            np.testing.assert_allclose(base, rows[:, 1:8], atol=1e-7)
            # Regress spurious spikes from the SO3 log cut in the initial fit.
            assert np.max(np.linalg.norm(np.diff(base[:, :3], axis=0), axis=1)) < .045
            t, q, _, _ = load_playback(robot, folder/kind)
            for joints in q: robot.validate_joints(joints)
            fk = reconstruct(robot, q)
            assert np.max(np.linalg.norm(fk[:, :3]-base[:, :3], axis=1)) < .00016


def test_tpkmp_task_frames_actually_move_the_path():
    folder = LAB/"models/rim_inner_y_minus"
    original, report = predict_saved(folder)
    frames = report["task_frames_in_sink"]
    offset = np.array([.02, -.03, .01])
    for name in frames:
        frames[name] = np.r_[np.array(frames[name][:3])+offset, frames[name][3:]].tolist()
    moved, _ = predict_saved(folder, frames=frames)
    np.testing.assert_allclose(moved[:, :3], original[:, :3]+offset, atol=1e-5)


def test_path_alignment_removes_pause_without_reversing_motion():
    poses = np.zeros((101, 7)); poses[:, 6] = 1
    poses[:, 0] = np.r_[np.linspace(0, .1, 40), np.full(21, .1), np.linspace(.1, .2, 40)]
    _, path = path_phase(poses)
    np.testing.assert_allclose(path[[0, -1], 0], [0, .2])
    assert np.all(np.diff(path[:, 0]) >= 0)
    assert np.count_nonzero(np.diff(path[:, 0]) == 0) <= 2
