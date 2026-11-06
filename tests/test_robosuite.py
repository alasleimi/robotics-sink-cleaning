"""Physics and kinematics regressions for the actual robosuite model."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation/adapter"))
from model import Model
from server import interpolate_pose


@pytest.fixture
def robot():
    model = Model()
    try:
        yield model
    finally:
        model.close()


def test_saved_pose_seed_and_physical_jacobian(robot):
    q = np.array(robot.state()["q"])
    pose = robot.pose("tcp")
    np.testing.assert_allclose(robot.inverse(pose, q), q, atol=1e-10)
    jac = robot.jacobian("tcp")
    for i in range(7):
        changed = q.copy()
        changed[i] += 1e-6
        p = robot.call("forward", changed, "tcp")
        derivative = np.r_[(np.array(p[:3])-pose[:3])/1e-6,
            (Rotation.from_quat(p[3:])*Rotation.from_quat(pose[3:]).inv()).as_rotvec()/1e-6]
        np.testing.assert_allclose(jac[:,i], derivative, atol=2e-6)
    np.testing.assert_allclose(robot.state()["q"], q)
    assert np.linalg.norm(np.array(pose[:3])-robot.pose("flange")[:3]) == pytest.approx(.202)


def test_joint_limits_and_unreachable_target(robot):
    with pytest.raises(ValueError, match="joint limits"):
        robot.validate_joints([4,0,0,0,0,0,0])
    with pytest.raises(ValueError, match="Unreachable"):
        robot.inverse([3,0,3,0,0,0,1], robot.state()["q"])


def test_exercise_sweep_preserves_pose_near_joint_limits(robot):
    # Tiny changes at the starting singular orientation can select either wrist
    # branch. Both must traverse Exercise 6's line without sticking at +/-175 deg.
    rng = np.random.default_rng(1)
    for _ in range(3):
        q = np.array(robot.state()["q"]) + rng.normal(0, 1e-4, 7)
        start = robot.call("forward", q, "tcp")
        for t in np.linspace(0, 1, 100):
            q = robot.inverse(interpolate_pose(start, [.4,-.3,.35,0,1,0,0], t), q)
        for y in np.r_[np.linspace(-.3,.3,100), np.linspace(.3,-.3,100)]:
            pose = [.4,float(y),.35,0,1,0,0]
            q = robot.inverse(pose, q)
            actual = robot.call("forward", q, "tcp")
            assert np.linalg.norm(np.array(actual[:3])-pose[:3]) < .0002
            assert (Rotation.from_quat(actual[3:])*Rotation.from_quat(pose[3:]).inv()).magnitude() < .002


def test_measured_force_and_compliant_response(robot):
    for _ in range(50):
        robot.step()
    np.testing.assert_allclose(robot.state()["force"], 0, atol=.05)
    before = np.array(robot.state()["q"])
    robot.call("setForce", [0,25,0])
    for _ in range(150):
        robot.step()
    np.testing.assert_allclose(robot.state()["force"], [0,25,0], atol=.15)
    stiff_displacement = np.linalg.norm(np.array(robot.state()["q"])-before)
    robot.command({"kind":"joint", "q":before, "stiffness":[100]*7, "damping":[.8]*7})
    for _ in range(200):
        robot.step()
    assert np.linalg.norm(np.array(robot.state()["q"])-before) > stiff_displacement*3


def test_cartesian_impedance_and_teaching_target(robot):
    q, pose = robot.state()["q"], robot.pose("tcp")
    robot.command({"kind":"cartesian", "q":q, "pose":pose,
                   "stiffness":[1500,1500,500,200,200,200], "damping":[.8]*6,
                   "nullStiffness":5, "nullDamping":.5})
    robot.call("setForce", [0,10,0])
    for _ in range(200):
        robot.step()
    assert robot.pose("tcp")[1]-pose[1] == pytest.approx(10/1500, abs=.002)
    target = robot.pose("tcp")
    target[0] -= .01
    robot.set_target(target)
    np.testing.assert_allclose(robot.state()["target"], target)
    target_id = robot.m.body("TeachTarget").mocapid[0]
    np.testing.assert_allclose(robot.d.mocap_pos[target_id], target[:3])


def test_robosuite_reset_and_step_interface(robot):
    env = robot.env
    observation = env.reset()
    assert "robot0_joint_pos" in observation
    target = robot.d.qpos[robot.qpos].copy()
    target[0] += .1
    for _ in range(60):
        observation, reward, done, info = env.step(target)
    assert abs(robot.d.qpos[robot.qpos][0]-target[0]) < .005
    assert reward == 0 and not done
