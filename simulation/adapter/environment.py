"""Robosuite iiwa7 task, course tool and mouse-movable teaching target."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import EmptyArena
from robosuite.models.grippers import NullGripper, register_gripper
from robosuite.models.tasks import ManipulationTask

ROOT = Path(__file__).resolve().parents[2]


def tool_parameters():
    template = ET.parse(ROOT / "src/main/resources/RoboticsAPI.data.xml").find(".//toolTemplate[@name='EXERCISE_TOOL']")
    transform = template.find("./frames/frame[@name='TCP']/transformation").attrib
    if any(float(transform[k]) != 0 for k in ("a", "b", "c")):
        raise ValueError("Rotated tool TCP requires explicit adapter support")
    load = template.find("loadData").attrib
    return ([float(transform[k])/1000 for k in ("x", "y", "z")], float(load["mass"]),
            [float(load[k])/1000 for k in ("cogX", "cogY", "cogZ")])


@register_gripper
class CourseTool(NullGripper):
    def __init__(self, idn=0):
        super().__init__(idn)
        tcp, mass, cog = tool_parameters()
        body = self.worldbody[0]
        body.find("inertial").set("mass", str(mass))
        body.find("inertial").set("pos", " ".join(map(str, cog)))
        # The course specifies no inertia tensor; this sphere approximation is explicit.
        body.find("inertial").set("diaginertia", " ".join([str(.4*mass*.025**2)]*3))
        eef = body.find("body")
        eef.set("pos", " ".join(map(str, tcp)))
        eef.set("quat", "1 0 0 0")
        ET.SubElement(body, "geom", name=self.naming_prefix + "tool_contact", type="sphere",
                      pos=" ".join(map(str, tcp)), size=".025", rgba=".15 .45 .65 1",
                      friction=".6 .005 .0001", group="1")
        ET.SubElement(body, "geom", name=self.naming_prefix + "tool_stem", type="capsule",
                      fromto="0 0 0 " + " ".join(map(str, tcp)), size=".008", mass="0",
                      contype="0", conaffinity="0", rgba=".4 .4 .4 1", group="1")
        eef.find("site[@name='" + self.naming_prefix + "grip_site_cylinder']").set("rgba", "0 0 0 0")
        self._contact_geoms.append("tool_contact")
        self._visual_geoms.append("tool_stem")


class CourseEnvironment(ManipulationEnv):
    """Real robosuite environment; env.step accepts seven absolute joint targets."""
    def __init__(self, **kwargs):
        super().__init__(robots="IIWA", base_types="NullMount", gripper_types="CourseTool",
            controller_configs={"type": "BASIC", "body_parts": {"right": {
                "type": "JOINT_POSITION", "input_type": "absolute", "kp": 150,
                "damping_ratio": 1, "interpolation": None, "gripper": {"type": "GRIP"}}}},
            initialization_noise=None, use_camera_obs=False, has_renderer=False,
            has_offscreen_renderer=False, control_freq=50, hard_reset=False,
            ignore_done=True, **kwargs)

    def _load_model(self):
        super()._load_model()
        self.robots[0].robot_model.set_base_xpos(np.zeros(3))
        self.robots[0].init_qpos = np.array([0, np.pi/6, 0, -np.pi/3, 0, np.pi/4, 0])
        arena = EmptyArena()
        target = ET.SubElement(arena.worldbody, "body", name="TeachTarget", mocap="true", pos="0 0 1.5")
        ET.SubElement(target, "geom", name="TeachTarget_visual", type="sphere", size=".035",
                      contype="0", conaffinity="0", rgba=".1 .9 .3 .35", group="1")
        self.model = ManipulationTask(mujoco_arena=arena, mujoco_robots=[self.robots[0].robot_model])
        self.model.root.find("option").set("timestep", ".002")
        self.model.root.find("option").set("integrator", "implicitfast")

    def reward(self, action=None):
        return 0.0  # Exercise sandbox; no invented sink-cleaning reward.

    def _check_success(self):
        return False
