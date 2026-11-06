"""Robosuite/MuJoCo backend for Sunrise. SI units; quaternions XYZW."""
from __future__ import annotations
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from robosuite.controllers.parts.generic.joint_pos import JointPositionController
from robosuite.utils.control_utils import opspace_matrices
from environment import CourseEnvironment, ROOT, tool_parameters


class SunriseImpedance(JointPositionController):
    """Robosuite dynamics with physical-unit KUKA-style spring gains.

    Stock robosuite gains multiply acceleration error by inertia. Sunrise gains
    denote N/m or Nm/rad, so we explicitly map physical springs and use model
    inertia for damping. This is not KUKA's private controller.
    """
    def torque(self, command, jac, pose, ref_rotation):
        self.update()
        q, dq, mass = self.joint_pos, self.joint_vel, self.mass_matrix
        k, ratio = np.array(command["stiffness"]), np.array(command["damping"])
        if command["kind"] == "cartesian":
            goal = command["pose"]
            error = np.r_[np.array(goal[:3])-pose[:3],
                          (Rotation.from_quat(goal[3:])*Rotation.from_quat(pose[3:]).inv()).as_rotvec()]
            basis = np.zeros((6, 6))
            basis[:3, :3] = basis[3:, 3:] = ref_rotation
            j_local = basis.T @ jac
            inertia, _, _, null = opspace_matrices(mass, j_local, j_local[:3], j_local[3:])
            damping = 2*ratio*np.sqrt(k*np.maximum(np.diag(inertia), .001))
            wrench = k*(basis.T @ error) - damping*(j_local @ dq)
            if "maxForce" in command:
                wrench = np.clip(wrench, -np.array(command["maxForce"]), command["maxForce"])
            kn, dn = command.get("nullStiffness", 0), command.get("nullDamping", .3)
            posture = kn*(np.array(command["q"])-q) - 2*dn*np.sqrt(kn*np.diag(mass))*dq
            torque = j_local.T @ wrench + null.T @ posture
        else:
            damping = np.where(k == 0, 2*ratio, 2*ratio*np.sqrt(k*np.diag(mass)))
            torque = k*(np.array(command["q"])-q) - damping*dq
        return self.clip_torques(torque + self.torque_compensation)


class Model:
    def __init__(self, *, headless=True):
        self.env = CourseEnvironment()
        self.env.reset()
        self.sim = self.env.sim
        self.m, self.d = self.sim.model._model, self.sim.data._data
        self.robot = self.env.robots[0]
        self.joints = [self.sim.model.joint_name2id(n) for n in self.robot.robot_model.joints]
        self.qpos = self.m.jnt_qposadr[self.joints]
        self.dofs = self.m.jnt_dofadr[self.joints]
        self.actuators = self.robot._ref_actuators_indexes_dict["right"]
        self.target_mocap = int(self.m.body("TeachTarget").mocapid[0])
        for name in ("left_eef_target", "right_eef_target"):
            body = self.m.body(name).id
            self.m.geom_rgba[self.m.geom_bodyid == body, 3] = 0
        self.limits = self.m.jnt_range[self.joints].copy()
        self.tcp_offset, self.tool_mass, self.tool_cog = tool_parameters()
        gripper = self.robot.gripper["right"]
        self.sites = {"tcp": self.sim.model.site_name2id(gripper.important_sites["grip_site"]),
                      "flange": self.sim.model.site_name2id(gripper.naming_prefix + "ft_frame")}
        self.tool = int(self.m.site_bodyid[self.sites["flange"]])
        self.sensor_force = self.sim.model.sensor_name2id(gripper.naming_prefix + "force_ee")
        self.sensor_torque = self.sim.model.sensor_name2id(gripper.naming_prefix + "torque_ee")
        self.ik = mujoco.MjData(self.m)
        self.ik.qpos[:] = self.d.qpos
        indexes = {"joints": self.joints, "qpos": self.qpos, "qvel": self.dofs}
        self.controller = SunriseImpedance(self.sim, joint_indexes=indexes,
            actuator_range=(self.m.actuator_ctrlrange[self.actuators, 0], self.m.actuator_ctrlrange[self.actuators, 1]),
            ref_name=gripper.important_sites["grip_site"], input_type="absolute", lite_physics=False)
        self.current = {"kind": "joint", "q": self.d.qpos[self.qpos].copy(),
                        "stiffness": [3000,3000,2000,2000,1000,800,500], "damping": [.8]*7}
        self.force = np.zeros(3)
        self.viewer = None
        self.set_target(self.pose("tcp"))
        if not headless:
            from mujoco import viewer
            self.viewer = viewer.launch_passive(self.m, self.d)
            with self.viewer.lock():
                self.viewer.opt.geomgroup[0] = 0  # Hide overlapping collision meshes.
                self.viewer.opt.sitegroup[:] = 0
                self.viewer.cam.lookat[:] = [0, 0, .6]
                self.viewer.cam.distance = 2.2
                self.viewer.cam.azimuth = 135
                self.viewer.cam.elevation = -20

    def pose(self, which, data=None):
        data = self.d if data is None else data
        site = self.sites[which]
        return np.r_[data.site_xpos[site], Rotation.from_matrix(data.site_xmat[site].reshape(3,3)).as_quat()].tolist()

    def jacobian(self, which, data=None):
        jp, jr = np.zeros((3,self.m.nv)), np.zeros((3,self.m.nv))
        mujoco.mj_jacSite(self.m, self.d if data is None else data, jp, jr, self.sites[which])
        return np.vstack((jp[:,self.dofs], jr[:,self.dofs]))

    def state(self):
        tcp, flange = self.pose("tcp"), self.pose("flange")
        rot = Rotation.from_quat(flange[3:]).as_matrix()
        def sensor(ident):
            adr = self.m.sensor_adr[ident]
            return self.d.sensordata[adr:adr+3].copy()
        # Sensor is parent-on-child; negate to report environment-on-tool.
        gravity = self.tool_mass*self.m.opt.gravity
        force = -rot @ sensor(self.sensor_force) - gravity
        moment = -rot @ sensor(self.sensor_torque) - np.cross(rot @ self.tool_cog, gravity)
        moment -= np.cross(np.array(tcp[:3])-flange[:3], force)
        external = self.jacobian("tcp").T @ np.r_[force, moment]
        return {"q": self.d.qpos[self.qpos].tolist(), "dq": self.d.qvel[self.dofs].tolist(),
                "tcp": tcp, "flange": flange, "force": force.tolist(), "torque": moment.tolist(),
                "measuredTorque": self.d.qfrc_actuator[self.dofs].tolist(),
                "externalTorque": external.tolist(), "time": float(self.d.time),
                "target": np.r_[self.d.mocap_pos[self.target_mocap], self.d.mocap_quat[self.target_mocap][[1,2,3,0]]].tolist(),
                "backend": "robosuite"}

    def set_target(self, pose):
        self.d.mocap_pos[self.target_mocap] = pose[:3]
        self.d.mocap_quat[self.target_mocap] = np.array(pose[3:])[[3,0,1,2]]

    def call(self, name, *args):
        if name == "forward":
            q, which = args
            self.validate_joints(q)
            self.ik.qpos[self.qpos] = q
            mujoco.mj_forward(self.m, self.ik)
            return self.pose(which, self.ik)
        if name == "setForce":
            self.force = np.array(args[0], dtype=float)
            return
        raise ValueError(f"Unknown model operation {name}")

    def inverse(self, pose, seed, which="tcp", retry=False):
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 1e-8:
            raise ValueError("Expected finite XYZ + XYZW pose")
        candidates = [seed]
        if retry:
            candidates += [[0,.52,0,-1.05,0,.78,0], [.5,.7,.5,-1.2,.3,1,0], [-.5,.7,-.5,-1.2,-.3,1,0]]
        target_rotation = Rotation.from_quat(pose[3:])
        for candidate in candidates:
            q = np.clip(np.array(candidate, dtype=float), self.limits[:,0], self.limits[:,1])
            for _ in range(160):
                self.ik.qpos[self.qpos] = q
                mujoco.mj_forward(self.m, self.ik)
                actual = self.pose(which, self.ik)
                error = np.r_[pose[:3]-actual[:3],
                    (target_rotation*Rotation.from_quat(actual[3:]).inv()).as_rotvec()]
                if np.linalg.norm(error[:3]) < .00015 and np.linalg.norm(error[3:]) < .0015:
                    return q.tolist()
                jac = self.jacobian(which, self.ik)
                pinv = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6)*1e-5, np.eye(6))
                # Move away from joint limits in the redundant direction before
                # clamping stalls the primary pose solve. No posture change when
                # the supplied seed already achieves the requested pose.
                distance_low = np.maximum(q-self.limits[:,0], .02)
                distance_high = np.maximum(self.limits[:,1]-q, .02)
                avoid = np.clip(.005*(1/distance_low**2-1/distance_high**2), -5, 5)
                # The exact one-dimensional null vector avoids leaking the
                # posture correction into the primary pose near singularities.
                null = np.linalg.svd(jac, full_matrices=True)[2][-1]
                delta = pinv @ error + null * (null @ avoid) * min(1., np.linalg.norm(error)/.05)
                q = np.clip(q + delta*min(1., .15/max(np.max(np.abs(delta)), 1e-9)), self.limits[:,0], self.limits[:,1])
            # Clamping a pseudoinverse step can stall at a joint limit. A bounded
            # solve redistributes motion across redundant joints without changing
            # the seed to an unrelated elbow branch during a LIN trajectory.
            def residual(angles):
                self.ik.qpos[self.qpos] = angles
                mujoco.mj_forward(self.m, self.ik)
                actual = self.pose(which, self.ik)
                return np.r_[pose[:3]-actual[:3],
                    (target_rotation*Rotation.from_quat(actual[3:]).inv()).as_rotvec()]
            fit = least_squares(residual, q, bounds=(self.limits[:,0], self.limits[:,1]),
                                max_nfev=1500, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            error = residual(fit.x)
            if np.linalg.norm(error[:3]) < .00015 and np.linalg.norm(error[3:]) < .0015:
                return fit.x.tolist()
        print("IK diagnostic: " + json.dumps({"pose": pose.tolist(), "seed": list(seed), "fit": fit.x.tolist()}), flush=True)
        raise ValueError(f"Unreachable {which} target: position error {np.linalg.norm(error[:3]):.4f} m, angle error {np.linalg.norm(error[3:]):.4f} rad")

    def validate_joints(self, q):
        q = np.asarray(q)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("Expected seven finite joint angles in radians")
        if np.any(q < self.limits[:,0]-1e-6) or np.any(q > self.limits[:,1]+1e-6):
            raise ValueError("Joint target exceeds iiwa7 joint limits")

    def command(self, command):
        self.current = command

    def step(self):
        # Sync consumes mouse perturbations and mocap edits. Keep injected forces
        # separate from the viewer's applied body wrenches.
        if self.viewer:
            self.viewer.sync()
        perturbation = self.d.xfrc_applied.copy()
        for _ in range(round(.02/self.m.opt.timestep)):
            self.d.xfrc_applied[:] = perturbation
            self.d.xfrc_applied[self.tool, :3] += self.force
            self.d.xfrc_applied[self.tool, 3:] += np.cross(self.d.site_xpos[self.sites["tcp"]]-self.d.xipos[self.tool], self.force)
            self.controller.update(force=True)
            which = self.current.get("which", "tcp")
            rotation = np.eye(3)
            if "rotationWhich" in self.current:
                rotation = Rotation.from_quat(self.pose(self.current["rotationWhich"])[3:]).as_matrix()
            elif "rotation" in self.current:
                rotation = np.array(self.current["rotation"]).reshape(3,3)
            self.d.ctrl[self.actuators] = self.controller.torque(self.current, self.jacobian(which), self.pose(which), rotation)
            mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        self.d.xfrc_applied[:] = perturbation

    def is_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer:
            self.viewer.close()
        self.env.close()
