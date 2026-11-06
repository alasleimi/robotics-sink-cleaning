"""Authenticated loopback bridge. One thread owns ZeroMQ and the simulation clock."""
from __future__ import annotations

import argparse
from collections import deque
import copy
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import secrets
import time
import traceback

import numpy as np

from model import Model, ROOT

POSITION = {"kind": "joint", "stiffness": [3000, 3000, 2000, 2000, 1000, 800, 500], "damping": [.8] * 7}


def slerp(a, b, u):
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = a + u * (b - a)
        return (q / np.linalg.norm(q)).tolist()
    angle = math.acos(np.clip(dot, -1, 1))
    return ((math.sin((1-u)*angle)*a + math.sin(u*angle)*b) / math.sin(angle)).tolist()


def interpolate_pose(a, b, u):
    return [(1-u)*a[i] + u*b[i] for i in range(3)] + slerp(a[3:], b[3:], u)


def quaternion_product(a, b):
    av, bv = np.array(a[:3]), np.array(b[:3])
    return [*(a[3]*bv + b[3]*av + np.cross(av, bv)), a[3]*b[3]-av@bv]


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                     [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                     [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])


def smooth(u):
    u = np.clip(u, 0., 1.)
    return 10*u**3 - 15*u**4 + 6*u**5


def inverse_smooth(value):
    low, high = 0., 1.
    for _ in range(30):
        mid = (low+high)/2
        if smooth(mid) < value:
            low = mid
        else:
            high = mid
    return (low+high)/2


class Engine:
    def __init__(self, model):
        self.model = model
        self.state = model.state()
        self.queue = deque()
        self.motions = {}
        self.active = None
        self.paused = False
        self.counter = 0
        self.last_command = {**POSITION, "q": self.state["q"]}
        self.manual = None
        self.follow_target = False
        self.trace = deque(maxlen=10000)
        self.time = 0.

    def submit(self, request):
        items = request.get("motions") if request["type"] == "batch" else [request]
        if not items:
            raise ValueError("Empty motion batch")
        self.counter += 1
        entry = {"id": self.counter, "state": "queued", "error": None, "fired": [], "items": copy.deepcopy(items), "part": 0}
        self.motions[self.counter] = entry
        self.queue.append(entry)
        return {"id": self.counter}

    def hold(self):
        self.last_command = {**POSITION, "q": list(self.state["q"])}
        self.model.command(self.last_command)

    def cancel(self, ident):
        entry = self.motions[int(ident)]
        if entry["state"] in ("finished", "cancelled", "error"):
            return
        entry["state"] = "cancelled"
        if self.active and self.active is not entry and self.active.get("blend", {}).get("owner") is entry:
            self.cancel(self.active["id"])
        if self.active is entry:
            self.active = None
            self.manual = None
            self.follow_target = False
            self.hold()

    def public(self, entry):
        return {k: entry[k] for k in ("id", "state", "error", "fired")}

    def plan(self, entry, start=None, record=True):
        spec = entry["items"][entry["part"]]
        kind = spec["type"]
        which = spec.get("which", "tcp")
        state = self.state if start is None else start
        q0 = np.array(state["q"])
        pose0 = state[which]
        target = spec.get("pose")
        if kind == "relative":
            offset = spec["offset"]
            refq = (self.state[spec["referenceWhich"]] if "referenceWhich" in spec else spec.get("reference", pose0))[3:]
            translation = np.array(pose0[:3]) + rotation(refq) @ offset[:3]
            world_delta = quaternion_product(quaternion_product(refq, offset[3:]), [-refq[0], -refq[1], -refq[2], refq[3]])
            target = [*translation, *quaternion_product(world_delta, pose0[3:])]
            kind = "lin"
        if kind == "hold":
            q1 = q0.copy()
            target = pose0
            duration = spec.get("seconds", -1)
            if duration < 0:
                duration = float("inf")
        else:
            if "q" in spec:
                q1 = np.array(spec["q"])
                self.model.validate_joints(q1.tolist())
                target = self.model.call("forward", q1.tolist(), which)
            elif target is not None:
                seed = spec.get("seed", q0.tolist())
                q1 = np.array(self.model.inverse(target, seed, which, retry=kind != "lin"))
            else:
                raise ValueError("Motion has no target")
            velocity = spec.get("jointVelocity", .3)
            acceleration = spec.get("jointAcceleration", .3)
            if not 0 < velocity <= 1 or not 0 < acceleration <= 1:
                raise ValueError("Relative joint velocity/acceleration must be in (0, 1]")
            delta = float(np.max(np.abs(q1-q0)))
            duration = max(.12, 1.875*delta/(1.7*velocity), math.sqrt(5.8*delta/(3*acceleration)))
            if kind == "lin":
                cart_velocity = spec.get("cartVelocity", .15)
                if cart_velocity <= 0:
                    raise ValueError("Cartesian velocity must be positive")
                distance = np.linalg.norm(np.array(target[:3])-pose0[:3])
                duration = max(duration, 1.875*distance/cart_velocity)
                if spec.get("cartAcceleration"):
                    duration = max(duration, math.sqrt(5.8*distance/spec["cartAcceleration"]))
        entry.update({"spec": spec, "kind": kind, "q0": q0, "q1": q1, "pose0": pose0,
                      "target": target, "duration": duration, "elapsed": 0., "settling": 0., "lastQ": q0.tolist()})
        entry["state"] = "running"
        if record:
            self.trace.append({"event": "start", "id": entry["id"], "part": entry["part"], "type": kind, "time": self.time})

    def lookahead(self, entry):
        """Overlap the ends of two trajectories with a C2 crossfade inside the blend region.

        This is a simulator approximation, not KUKA's proprietary path planner.
        Async successors must already be queued before the overlap begins.
        """
        spec = entry["spec"]
        if entry.get("blend") or entry["elapsed"] > entry["duration"]*.5:
            return
        if not (spec.get("blendRel", 0) > 0 or spec.get("blendCart", 0) > 0):
            return
        if entry["kind"] == "hold" or spec.get("conditions"):
            return
        if entry["part"]+1 < len(entry["items"]):
            owner, part = entry, entry["part"]+1
        else:
            candidates = [e for e in self.queue if e["state"] == "queued"]
            if not candidates:
                return
            owner, part = candidates[0], 0
        next_spec = owner["items"][part]
        if next_spec.get("which", "tcp") != spec.get("which", "tcp") or next_spec.get("control") != spec.get("control") or next_spec.get("conditions"):
            return
        successor = {"id": owner["id"], "items": owner["items"], "part": part, "fired": [], "error": None}
        which = spec.get("which", "tcp")
        self.plan(successor, {"q": entry["q1"], which: entry["target"]}, record=False)
        if successor["kind"] == "hold":
            return
        r1 = r2 = min(.3, spec.get("blendRel", .3))
        if spec.get("blendCart", 0) > 0:
            radius = spec["blendCart"]
            d1 = np.linalg.norm(np.array(entry["target"][:3])-entry["pose0"][:3])
            d2 = np.linalg.norm(np.array(successor["target"][:3])-successor["pose0"][:3])
            r1, r2 = min(.3, radius/max(d1, 1e-9)), min(.3, radius/max(d2, 1e-9))
        overlap = min(entry["duration"]*(1-inverse_smooth(1-r1)), successor["duration"]*inverse_smooth(r2))
        if overlap > .04:
            entry["blend"] = {"next": successor, "owner": owner, "overlap": overlap, "started": False}

    def sample(self, entry, elapsed):
        fraction = smooth(elapsed/max(entry["duration"], .001))
        pose = interpolate_pose(entry["pose0"], entry["target"], fraction)
        spec = entry["spec"]
        if entry["kind"] == "lin":
            q = self.model.inverse(pose, entry["lastQ"], spec.get("which", "tcp"))
        else:
            q = (entry["q0"] + fraction*(entry["q1"]-entry["q0"])).tolist()
            if spec.get("control", {}).get("kind") == "cartesian":
                pose = self.model.call("forward", q, spec.get("which", "tcp"))
        entry["lastQ"] = q
        return q, pose

    def blend_sample(self, entry, q, pose):
        blend = entry.get("blend")
        if not blend:
            return q, pose
        elapsed = entry["elapsed"] - entry["duration"] + blend["overlap"]
        if elapsed < 0:
            return q, pose
        successor = blend["next"]
        if not blend["started"]:
            blend["started"] = True
            self.trace.append({"event": "blend", "id": entry["id"], "next": successor["id"], "time": self.time})
        q2, pose2 = self.sample(successor, elapsed)
        weight = smooth(elapsed/blend["overlap"])
        if entry["kind"] == successor["kind"] == "ptp":
            q = ((1-weight)*np.array(q)+weight*np.array(q2)).tolist()
            pose = self.model.call("forward", q, entry["spec"].get("which", "tcp"))
        else:
            if entry["kind"] == "ptp":
                pose = self.model.call("forward", q, entry["spec"].get("which", "tcp"))
            if successor["kind"] == "ptp":
                pose2 = self.model.call("forward", q2, successor["spec"].get("which", "tcp"))
            pose = interpolate_pose(pose, pose2, weight)
            q = self.model.inverse(pose, entry.get("blendQ", q), entry["spec"].get("which", "tcp"))
        entry["blendQ"] = q
        successor["elapsed"] = elapsed + .02
        successor["lastQ"] = q
        return q, pose

    def condition(self, cond):
        kind = cond["type"]
        if kind in ("or", "and"):
            results = [self.condition(c) for c in cond["children"]]
            return any(results) if kind == "or" else all(results)
        if kind == "force":
            f = np.array(self.state["force"])
            if "rotationWhich" in cond:
                f = rotation(self.state[cond["rotationWhich"]][3:]).T @ f
            elif "rotation" in cond:
                f = np.array(cond["rotation"]).reshape(3, 3).T @ f
            return np.linalg.norm(f[cond["axes"]]) > cond["threshold"] + cond.get("tolerance", 0)
        if kind == "jointTorque":
            value = self.state["externalTorque"][cond["joint"]]
            return value < cond["min"] or value > cond["max"]
        raise ValueError(f"Unsupported condition {kind}")

    def finish(self, entry):
        if entry.get("blend", {}).get("started") and not entry["fired"]:
            blend = entry.pop("blend")
            successor, owner = blend["next"], blend["owner"]
            self.trace.append({"event": "blendHandoff", "id": entry["id"], "next": owner["id"], "time": self.time,
                               "speed": float(np.linalg.norm(self.state["dq"]))})
            if owner is not entry:
                entry["state"] = "finished"
                self.queue.remove(owner)
            owner.update(successor)
            self.active = owner
            return
        entry["part"] += 1
        if entry["part"] < len(entry["items"]) and not entry["fired"]:
            self.plan(entry)
        else:
            entry["state"] = "finished"
            self.trace.append({"event": "finish", "id": entry["id"], "fired": entry["fired"], "time": self.time})
            self.active = None

    def tick(self):
        self.state = self.model.state()
        self.time = self.state["time"]
        if not self.paused:
            if self.active is None:
                while self.queue and self.active is None:
                    candidate = self.queue.popleft()
                    if candidate["state"] == "queued":
                        self.active = candidate
                        try:
                            self.plan(candidate)
                        except Exception as exc:
                            candidate.update(state="error", error=str(exc))
                            self.active = None
            if self.active:
                entry = self.active
                try:
                    spec = entry["spec"]
                    fired = [c.get("id", i) for i, c in enumerate(spec.get("conditions", [])) if self.condition(c)]
                    if fired:
                        entry["fired"] = fired
                        self.hold()
                        self.finish(entry)
                    else:
                        self.lookahead(entry)
                        u = min(1., entry["elapsed"] / max(entry["duration"], .001))
                        q, pose = self.sample(entry, entry["elapsed"])
                        q, pose = self.blend_sample(entry, q, pose)
                        control = copy.deepcopy(spec.get("control", POSITION))
                        control.update(q=q, pose=pose, which=spec.get("which", "tcp"))
                        self.last_command = control
                        self.model.command(control)
                        entry["elapsed"] += .02
                        if u >= 1:
                            # Impedance does not require reaching the target while externally displaced.
                            compliant = "control" in spec
                            joint_error = np.max(np.abs(np.array(self.state["q"])-entry["q1"]))
                            if entry.get("blend", {}).get("started") or compliant or joint_error < .001:
                                self.finish(entry)
                            else:
                                entry["settling"] += .02
                                if entry["settling"] > 5:
                                    raise ValueError(f"Motion did not settle (joint error {joint_error:.3f} rad)")
                except Exception as exc:
                    entry.update(state="error", error=str(exc))
                    self.trace.append({"event": "error", "id": entry["id"], "error": str(exc), "time": self.time})
                    self.active = None
                    self.hold()
        if self.follow_target and not self.paused:
            try:
                self.manual = self.model.inverse(self.state["target"], self.state["q"])
            except ValueError:
                pass  # Keep the last reachable teaching target when the mouse goes out of reach.
        if self.manual and not self.paused:
            # Mouse/slider teaching overrides zero-stiffness motion targets only.
            self.model.command({**POSITION, "q": self.manual})
        self.model.step()

    def request(self, command):
        action = command["action"]
        if action == "state":
            return {**self.state, "tcpOffset": self.model.tcp_offset, "paused": self.paused, "activeCount": sum(e["state"] in ("queued", "running") for e in self.motions.values())}
        if action == "submit":
            return self.submit(command["motion"])
        if action == "motion":
            return self.public(self.motions[int(command["id"])])
        if action == "cancel":
            self.cancel(command["id"])
        elif action == "pause":
            self.paused = True
            self.hold()
        elif action == "resume":
            self.paused = False
        elif action == "abort":
            for ident in list(self.motions):
                self.cancel(ident)
            self.queue.clear()
            self.manual = None
            self.follow_target = False
            self.paused = True
            self.hold()
        elif action == "force":
            force = command["force"]
            if len(force) != 3 or any(not math.isfinite(x) or abs(x) > 100 for x in force):
                raise ValueError("Use three finite forces, each within ±100 N")
            self.model.call("setForce", force)
        elif action == "teach":
            self.model.validate_joints(command["q"])
            if self.active and not all(x == 0 for x in self.active["spec"].get("control", {}).get("stiffness", [1])):
                raise ValueError("Use teaching controls in Exercise 1 / zero-stiffness mode, or abort active motion first")
            self.manual = command["q"]
            self.follow_target = False
        elif action == "teachTarget":
            if self.active and not all(x == 0 for x in self.active["spec"].get("control", {}).get("stiffness", [1])):
                raise ValueError("Use scene teaching in Exercise 1 or abort active motion first")
            self.model.sim.setObjectPose(self.model.targets["tcp"], self.state["tcp"], self.model.sim.handle_world)
            self.follow_target = True
        elif action == "releaseTeach":
            self.manual = None
            self.follow_target = False
        elif action == "inverse":
            return {"q": self.model.inverse(command["pose"], command.get("seed", self.state["q"]), command.get("which", "tcp"), True)}
        elif action == "forward":
            return {"pose": self.model.call("forward", command["q"], command.get("which", "tcp"))}
        elif action == "trace":
            return {"events": list(self.trace)}
        else:
            raise ValueError(f"Unknown adapter action: {action}")
        return {"ok": True}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--sim-port", type=int, default=23000)
    parser.add_argument("--session", type=Path, default=ROOT / ".tools/adapter-session.json")
    parser.add_argument("--fast", action="store_true")
    args = parser.parse_args()
    engine = Engine(Model(args.sim_port))
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def handle(self):
            try:
                super().handle()
            except (ConnectionResetError, BrokenPipeError, TimeoutError):
                pass  # Closing an exercise must not stop physics or spam a traceback.

        def log_message(self, *args):
            pass

        def do_POST(self):
            try:
                if self.headers.get("Authorization") != "Bearer " + token:
                    self.send_error(401)
                    return
                size = int(self.headers.get("Content-Length", 0))
                if size <= 0 or size > 1_000_000:
                    raise ValueError("Invalid request size")
                result = engine.request(json.loads(self.rfile.read(size)))
                payload = json.dumps(result, allow_nan=False).encode()
                self.send_response(200)
            except Exception as exc:
                payload = json.dumps({"error": str(exc)}).encode()
                self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Connection", "close")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    server = HTTPServer(("127.0.0.1", args.port), Handler)
    server.timeout = .001
    args.session.parent.mkdir(parents=True, exist_ok=True)
    args.session.write_text(json.dumps({"url": f"http://127.0.0.1:{server.server_port}", "token": token}))
    print(f"Adapter ready on loopback port {server.server_port}", flush=True)
    try:
        while True:
            start = time.monotonic()
            # HTTP handlers and physics share a thread: the ZeroMQ socket never migrates.
            for _ in range(4):
                server.handle_request()
            engine.tick()
            if not args.fast:
                time.sleep(max(0., .02 - (time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        engine.model.close()
        server.server_close()
        args.session.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
