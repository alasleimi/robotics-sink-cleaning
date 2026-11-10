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
        if "guideJointSpeed" in command:
            # KUKA describes opposing damping above the guiding velocity limit.
            # Approximate that response; this is not certified safety monitoring.
            torque -= 100*np.sign(dq)*np.maximum(np.abs(dq)-command["guideJointSpeed"],0)
            velocity=jac[:3]@dq
            speed=np.linalg.norm(velocity)
            if speed>command["guideCartSpeed"]:
                torque -= jac[:3].T@(500*(speed-command["guideCartSpeed"])*velocity/max(speed,1e-9))
        return self.clip_torques(torque + self.torque_compensation)


class Model:
    def __init__(self, *, headless=True, scene="empty"):
        self.env = CourseEnvironment(scene=scene)
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
                if scene == "sink":
                    self.viewer.cam.lookat[:] = [.4,0,.2]
                    self.viewer.cam.distance = 1.8
                    self.viewer.cam.azimuth = -45
                    self.viewer.cam.elevation = -30

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
        external = self.external_joint_torques()
        return {"q": self.d.qpos[self.qpos].tolist(), "dq": self.d.qvel[self.dofs].tolist(),
                "tcp": tcp, "flange": flange, "force": force.tolist(), "torque": moment.tolist(),
                "measuredTorque": self.d.qfrc_actuator[self.dofs].tolist(),
                "externalTorque": external.tolist(), "time": float(self.d.time),
                "target": np.r_[self.d.mocap_pos[self.target_mocap], self.d.mocap_quat[self.target_mocap][[1,2,3,0]]].tolist(),
                "redundancy": self.redundancy(), "contacts": int(self.d.ncon), "backend": "robosuite"}

    def external_joint_torques(self):
        """Project actual contact and applied body loads onto every robot joint.

        A flange-only wrench misses a hand pushing an upper-arm link. Exclude
        gravity, actuators and joint-limit constraints from external contact.
        """
        result = self.d.qfrc_applied.copy()
        for body in range(1, self.m.nbody):
            wrench = self.d.xfrc_applied[body]
            if np.any(wrench):
                mujoco.mj_applyFT(self.m, self.d, wrench[:3], wrench[3:], self.d.xipos[body], body, result)
        mujoco.mj_applyFT(self.m, self.d, self.force, np.zeros(3),
                         self.d.site_xpos[self.sites["tcp"]], self.tool, result)
        for index, contact in enumerate(self.d.contact):
            local = np.zeros(6)
            mujoco.mj_contactForce(self.m, self.d, index, local)
            basis = contact.frame.reshape(3,3).T
            for geom, sign in ((contact.geom1, -1), (contact.geom2, 1)):
                body = int(self.m.geom_bodyid[geom])
                if body:
                    mujoco.mj_applyFT(self.m, self.d, sign*(basis @ local[:3]), sign*(basis @ local[3:]),
                                     contact.pos, body, result)
        return result[self.dofs]

    def redundancy(self, q=None):
        """Sunrise E1/status/turn for this iiwa (manual section 14.10).

        Find the wrist-axis intersection from the model, avoiding assumptions
        about offsets between the MJCF joint origins and the wrist centre.
        """
        data = self.d
        if q is not None:
            self.validate_joints(q)
            data = self.ik
            data.qpos[self.qpos] = q
            mujoco.mj_forward(self.m, data)
        angles = data.qpos[self.qpos]
        ids = self.joints[4:]
        projectors = [np.eye(3)-np.outer(a, a) for a in data.xaxis[ids]]
        wrist = np.linalg.solve(sum(projectors),
            sum(p @ x for p, x in zip(projectors, data.xanchor[ids])))
        a1 = self.m.jnt_bodyid[self.joints[0]]
        x = (data.xmat[a1].reshape(3,3).T @ (wrist-data.xanchor[self.joints[0]]))[0]
        status = int(x < 0) | (int(angles[3] < 0) << 1) | (int(angles[5] <= 0) << 2)
        turn = sum(int(angle < 0) << i for i, angle in enumerate(angles))
        return {"e1": float(angles[2]), "status": status, "turn": turn}

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

    def teaching_step(self, pose):
        """One bounded differential IK step for the mouse, from the actual arm.

        A dragged marker can be unreachable. Never run the full pose solver or
        queue up old mouse positions here: physics and input must keep updating.
        PTP/LIN still use inverse() with its full accuracy/redundancy checks.
        """
        actual = self.pose("tcp")
        error = np.r_[np.asarray(pose[:3])-actual[:3],
            (Rotation.from_quat(pose[3:])*Rotation.from_quat(actual[3:]).inv()).as_rotvec()]
        error /= max(1., np.linalg.norm(error[:3])/.02, np.linalg.norm(error[3:])/.1)
        jac = self.jacobian("tcp")
        delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6)*1e-4, error)
        delta *= min(1., .05/max(np.max(np.abs(delta)), 1e-9))
        return np.clip(self.d.qpos[self.qpos]+delta, *self.limits.T).tolist()

    def inverse(self, pose, seed, which="tcp", retry=False, *, e1=None, status=None):
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 1e-8:
            raise ValueError("Expected finite XYZ + XYZW pose")
        self.validate_joints(seed)
        if e1 is not None and (not np.isfinite(e1) or not self.limits[2,0] <= e1 <= self.limits[2,1]):
            raise ValueError("E1 must be finite and within joint 3 limits")
        if status is not None and (not isinstance(status, (int, np.integer)) or not 0 <= status <= 7):
            raise ValueError("Status must be an integer from 0 to 7")
        lower, upper = self.limits.T.copy()
        if status is not None:
            for joint, bit in ((3, 1), (5, 2)):
                if status & (1 << bit):
                    upper[joint] = 0
                else:
                    lower[joint] = 0
        free = np.array([i for i in range(7) if e1 is None or i != 2])
        candidates = [seed]
        if retry:
            candidates += [[0,.52,0,-1.05,0,.78,0], [.5,.7,.5,-1.2,.3,1,0], [-.5,.7,-.5,-1.2,-.3,1,0]]
            if status is not None:
                # Give the numerical solver both shoulder configurations. Axis
                # sign bounds select the requested elbow and wrist branches.
                candidates += [[a,.8,seed[2],(-1 if status & 2 else 1)*1.2,
                                b,(-1 if status & 4 else 1)*.9,0]
                               for a in (-2., 0., 2.) for b in (-2., 0., 2.)]
        target_rotation = Rotation.from_quat(pose[3:])

        def residual(angles):
            self.ik.qpos[self.qpos] = angles
            mujoco.mj_forward(self.m, self.ik)
            actual = self.pose(which, self.ik)
            return np.r_[pose[:3]-actual[:3],
                (target_rotation*Rotation.from_quat(actual[3:]).inv()).as_rotvec()]

        def acceptable(angles, error):
            if np.linalg.norm(error[:3]) >= .00015 or np.linalg.norm(error[3:]) >= .0015:
                return False
            if status is None:
                return True
            actual = self.redundancy(angles)["status"]
            # The two wrist/elbow branches coincide at zero. Permit the shared
            # boundary, while still checking the overhead/basic configuration.
            mask = 1 | (2 if abs(angles[3]) > 1e-6 else 0) | (4 if abs(angles[5]) > 1e-6 else 0)
            return (actual & mask) == (status & mask)

        for candidate in candidates:
            q = np.clip(np.array(candidate, dtype=float), lower, upper)
            if e1 is not None:
                q[2] = e1  # Eliminated from the solve: a hard constraint, not a preference.
            for _ in range(160):
                error = residual(q)
                if acceptable(q, error):
                    return q.tolist()
                jac = self.jacobian(which, self.ik)[:, free]
                pinv = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6)*1e-5, np.eye(6))
                # Move away from joint limits in the redundant direction before
                # clamping stalls the primary pose solve. No posture change when
                # the supplied seed already achieves the requested pose.
                distance_low = np.maximum(q-lower, .02)
                distance_high = np.maximum(upper-q, .02)
                avoid = np.clip(.005*(1/distance_low**2-1/distance_high**2), -5, 5)
                # The exact one-dimensional null vector avoids leaking the
                # posture correction into the primary pose near singularities.
                delta = pinv @ error
                if e1 is None:
                    null = np.linalg.svd(jac, full_matrices=True)[2][-1]
                    delta += null * (null @ avoid) * min(1., np.linalg.norm(error)/.05)
                q[free] = np.clip(q[free] + delta*min(1., .15/max(np.max(np.abs(delta)), 1e-9)), lower[free], upper[free])
            # Clamping a pseudoinverse step can stall at a joint limit. A bounded
            # solve redistributes motion across redundant joints without changing
            # the seed to an unrelated elbow branch during a LIN trajectory.
            def constrained_residual(values):
                angles = q.copy()
                angles[free] = values
                return residual(angles)
            fit = least_squares(constrained_residual, q[free], bounds=(lower[free], upper[free]),
                                max_nfev=1500, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            q[free] = fit.x
            error = residual(q)
            if acceptable(q, error):
                return q.tolist()
        print("IK diagnostic: " + json.dumps({"pose": pose.tolist(), "seed": list(seed), "fit": q.tolist(), "e1": e1, "status": status}), flush=True)
        raise ValueError(f"Unreachable {which} target with E1={e1}, status={status}: position error {np.linalg.norm(error[:3]):.4f} m, angle error {np.linalg.norm(error[3:]):.4f} rad")

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
