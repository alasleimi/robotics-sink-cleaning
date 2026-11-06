"""Prepare an owned simulator scene. All control is confined to loopback CoppeliaSim."""
from __future__ import annotations

from pathlib import Path
import math
import time
import xml.etree.ElementTree as ET

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

ROOT = Path(__file__).resolve().parents[2]


class Model:
    def __init__(self, port=23000):
        self.client = RemoteAPIClient(host="127.0.0.1", port=port)
        self.sim = s = self.client.require("sim")
        if s.getSimulationState() != s.simulation_stopped:
            raise RuntimeError("Adapter requires its own stopped simulator instance.")
        s.loadScene(str(ROOT / "simulation/scenes/iiwa7.ttt"))
        self.robot = s.getObject("/LBRiiwa7R800")
        objects = s.getObjectsInTree(self.robot, s.handle_all, 0)
        # The distributed model contains an automatic demonstration script.
        scripts = [h for h in objects if s.getObjectType(h) == s.sceneobject_script]
        if scripts:
            s.removeObjects(scripts)
        self.joints = s.getObjectsInTree(self.robot, s.sceneobject_joint, 0)
        if len(self.joints) != 7:
            raise RuntimeError("Expected the seven-axis iiwa 7 R800 model")
        self.flange = s.getObject("/LBRiiwa7R800/connection")
        tree = ET.parse(ROOT / "src/main/resources/RoboticsAPI.data.xml")
        template = tree.find(".//toolTemplate[@name='EXERCISE_TOOL']")
        transform = template.find("./frames/frame[@name='TCP']/transformation")
        self.tcp_offset = [float(transform.attrib[k]) / 1000 for k in ("x", "y", "z")]
        if any(float(transform.attrib[k]) != 0 for k in ("a", "b", "c")):
            raise RuntimeError("Non-identity tool rotation needs explicit adapter support")
        mass = float(template.find("loadData").attrib["mass"])
        self.tcp = s.createDummy(.015)
        s.setObjectAlias(self.tcp, "ExerciseTCP")
        s.setObjectParent(self.tcp, self.flange, False)
        s.setObjectPose(self.tcp, [*self.tcp_offset, 0, 0, 0, 1], self.flange)
        # A simple contact body stands in for the exercise tool. It is not calibrated CAD.
        self.tool = s.createPrimitiveShape(s.primitiveshape_spheroid, [.05, .05, .05], 0)
        s.setObjectAlias(self.tool, "ExerciseToolContact")
        s.setObjectParent(self.tool, self.flange, False)
        s.setObjectPose(self.tool, [*self.tcp_offset, 0, 0, 0, 1], self.flange)
        s.setShapeMass(self.tool, mass)
        s.setObjectInt32Param(self.tool, s.shapeintparam_static, 0)
        s.setObjectInt32Param(self.tool, s.shapeintparam_respondable, 1)
        self.targets = {}
        for name, tip in (("tcp", self.tcp), ("flange", self.flange)):
            target = s.createDummy(.035)
            s.setObjectAlias(target, "TeachTarget" if name == "tcp" else "FlangeTarget")
            s.setObjectPose(target, s.getObjectPose(tip, s.handle_world), s.handle_world)
            self.targets[name] = target
        self.limits = []
        for joint in self.joints:
            cyclic, interval = s.getJointInterval(joint)
            if cyclic:
                raise RuntimeError("Unexpected cyclic iiwa joint")
            self.limits.append([interval[0], interval[0] + interval[1]])
            s.setJointMode(joint, s.jointmode_dynamic, 0)
            s.setObjectInt32Param(joint, s.jointintparam_dynctrlmode, s.jointdynctrl_callback)
        for shape in s.getObjectsInTree(self.robot, s.sceneobject_shape, 0):
            s.setFloatProperty(shape, "mujoco.gravcomp", 1.0)
        s.setFloatProperty(self.tool, "mujoco.gravcomp", 1.0)
        s.setInt32Param(s.intparam_dynamic_engine, s.physics_mujoco)
        s.setFloatParam(s.floatparam_simulation_time_step, .02)
        self.script = s.createScript(s.scripttype_simulation, (Path(__file__).with_name("control.lua")).read_text(), 0, "lua")
        s.setObjectAlias(self.script, "SunriseAdapterControl")
        s.setObjectParent(self.script, self.joints[0], False)
        config = {"joints": self.joints, "tcp": self.tcp, "flange": self.flange,
                  "tcpTarget": self.targets["tcp"], "flangeTarget": self.targets["flange"],
                  "tool": self.tool, "toolMass": mass, "limits": self.limits, "maxTorque": [176, 176, 110, 110, 110, 40, 40]}
        s.writeCustomDataBlock(self.joints[0], "adapter-config", s.packTable(config))
        s.setStepping(True)
        s.startSimulation()
        s.step()
        self.initial = self.state()

    def call(self, name, *args):
        return self.sim.callScriptFunction(name, self.script, *args)

    def state(self):
        return self.call("state")

    def command(self, command):
        self.call("setCommand", command)

    def step(self):
        self.sim.step()

    def inverse(self, pose, seed, which="tcp", retry=False):
        candidates = [seed]
        if retry:
            candidates += [[0, .52, 0, -1.05, 0, .78, 0], [.5, .7, .5, -1.2, .3, 1, 0], [-.5, .7, -.5, -1.2, -.3, 1, 0]]
        best = None
        for candidate in candidates:
            result = self.call("inverse", pose, candidate, which)
            best = result
            if result["positionError"] < .001 and result["angleError"] < .01:
                self.validate_joints(result["q"])
                return result["q"]
        raise ValueError(f"Unreachable {which} target: position error {best['positionError']:.4f} m, angle error {best['angleError']:.4f} rad")

    def validate_joints(self, q):
        if len(q) != 7 or any(not math.isfinite(x) for x in q):
            raise ValueError("Expected seven finite joint angles in radians")
        for i, (value, (low, high)) in enumerate(zip(q, self.limits)):
            if value < low - 1e-6 or value > high + 1e-6:
                raise ValueError(f"Joint {i+1} target {value:.3f} outside [{low:.3f}, {high:.3f}]")

    def close(self):
        self.sim.stopSimulation()
        deadline = time.monotonic() + 5
        while self.sim.getSimulationState() != self.sim.simulation_stopped and time.monotonic() < deadline:
            time.sleep(.02)
