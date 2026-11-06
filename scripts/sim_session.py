"""Own a simulator, loopback bridge and Java exercise process as one session."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import urllib.request

from common import ROOT, load_config, local_path, gradle


class Session:
    def __init__(self, *, headless=False, fast=False, build=True, scene="empty"):
        self.config = load_config()
        if build:
            gradle(self.config, "simClasspath")
        self.processes = []
        self.logs = []
        self.folder = Path(tempfile.mkdtemp(prefix="sim-", dir=ROOT / ".tools"))
        self.session_file = self.folder / "session.json"
        try:
            # macOS requires MuJoCo's launcher for a passive GUI viewer.
            python = Path(sys.executable)
            if sys.platform == "darwin" and not headless:
                python = python.with_name("mjpython")
                if not python.is_file():
                    raise RuntimeError("MuJoCo viewer launcher missing; run setup again.")
            bridge_args = [str(python), str(ROOT / "simulation/adapter/server.py"), "--session", str(self.session_file), "--scene", scene]
            if fast:
                bridge_args.append("--fast")
            if headless:
                bridge_args.append("--headless")
            self.bridge = self.start(bridge_args, "bridge")
            deadline = time.monotonic() + 120
            while not self.session_file.exists():
                if any(p.poll() is not None for p in self.processes):
                    raise RuntimeError(f"Simulator startup failed (exit codes {[p.poll() for p in self.processes]}). Logs: {self.folder}")
                if time.monotonic() > deadline:
                    raise TimeoutError(f"Simulator did not become ready. Logs: {self.folder}")
                time.sleep(.1)
            self.auth = json.loads(self.session_file.read_text())
        except BaseException:
            self.close()
            raise

    def start(self, args, name, **kwargs):
        hidden = kwargs.pop("hidden", True)
        interactive = kwargs.pop("interactive", False)
        log = (self.folder / f"{name}.log").open("w", encoding="utf-8")
        self.logs.append(log)
        p = subprocess.Popen(args, cwd=kwargs.pop("cwd", ROOT), stdin=None if interactive else subprocess.PIPE,
                             stdout=None if interactive else log, stderr=subprocess.STDOUT,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" and hidden else 0, **kwargs)
        self.processes.append(p)
        return p

    def call(self, action, **params):
        data = json.dumps({"action": action, **params}).encode()
        req = urllib.request.Request(self.auth["url"], data=data,
            headers={"Authorization": "Bearer " + self.auth["token"], "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as response:
            return json.load(response)

    def java(self, main, *args, data=None, headless=True, label="exercise", interactive=False):
        java = local_path(self.config, "java_home") / "bin" / ("java.exe" if os.name == "nt" else "java")
        classpath = (ROOT / ".tools/simulator-classpath.txt").read_text()
        return self.start([str(java), f"-Djava.awt.headless={str(headless).lower()}",
            f"-Drobot.dataDir={data or ROOT / 'data/simulation/robosuite-sunrise'}", f"-Dsim.session={self.session_file}",
            "-cp", classpath, main, *args], label, hidden=headless, interactive=interactive)

    def close(self):
        for p in reversed(self.processes):
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=5)
        for log in self.logs:
            log.close()
        for p in self.processes:
            if p.stdin:
                p.stdin.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def run_exercise(exercise, *, headless=False, fast=False, data=None, scene="empty"):
    with Session(headless=headless, fast=fast, scene=scene) as session:
        print(f"Simulation ready. Logs: {session.folder}", flush=True)
        process = session.java("simproxy.Launcher", exercise, data=data, headless=headless)
        try:
            with (session.folder / "exercise.log").open(encoding="utf-8", errors="replace") as output:
                while process.poll() is None:
                    if session.bridge.poll() is not None:
                        process.terminate()
                        break
                    text = output.read()
                    if text:
                        print(text, end="", flush=True)
                    time.sleep(.1)
                print(output.read(), end="", flush=True)
            result = process.returncode
            if session.bridge.poll() == 0:
                result = 0  # Closing the viewer is an ordinary session exit.
        except KeyboardInterrupt:
            result = 130
        if result not in (0, 130):
            raise RuntimeError(f"Exercise exited with status {result}")
