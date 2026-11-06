"""Own a simulator, loopback bridge and Java exercise process as one session."""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

from common import ROOT, load_config, local_path, gradle


class Session:
    def __init__(self, *, headless=False, fast=False, build=True):
        self.config = load_config()
        if build:
            gradle(self.config, "simClasspath")
        self.processes = []
        self.logs = []
        self.folder = Path(tempfile.mkdtemp(prefix="sim-", dir=ROOT / ".tools"))
        self.session_file = self.folder / "session.json"
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        env = dict(os.environ, COPPELIASIM_USER_SETTINGS_FOLDER_SUFFIX="-sink-adapter")
        sim_args = [str(local_path(self.config, "simulator")), f"-GzmqRemoteApi.rpcPort={port}", "-GpreferredSandboxLang=lua", f"-Gpython={sys.executable}", "-vscriptinfos"]
        if headless:
            sim_args.append("-h")
        try:
            self.start(sim_args, "coppelia", cwd=local_path(self.config, "simulator_resources"), env=env, hidden=headless)
            bridge_args = [sys.executable, str(ROOT / "simulation/adapter/server.py"), "--sim-port", str(port), "--session", str(self.session_file)]
            if fast:
                bridge_args.append("--fast")
            self.start(bridge_args, "bridge")
            deadline = time.monotonic() + 60
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
        log = (self.folder / f"{name}.log").open("w", encoding="utf-8")
        self.logs.append(log)
        p = subprocess.Popen(args, cwd=kwargs.pop("cwd", ROOT), stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" and hidden else 0, **kwargs)
        self.processes.append(p)
        return p

    def call(self, action, **params):
        data = json.dumps({"action": action, **params}).encode()
        req = urllib.request.Request(self.auth["url"], data=data,
            headers={"Authorization": "Bearer " + self.auth["token"], "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as response:
            return json.load(response)

    def java(self, main, *args, data=None, headless=True, label="exercise"):
        java = local_path(self.config, "java_home") / "bin" / ("java.exe" if os.name == "nt" else "java")
        classpath = (ROOT / ".tools/simulator-classpath.txt").read_text()
        return self.start([str(java), f"-Djava.awt.headless={str(headless).lower()}",
            f"-Drobot.dataDir={data or ROOT / 'data/simulation'}", f"-Dsim.session={self.session_file}",
            "-cp", classpath, main, *args], label, hidden=headless)

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


def run_exercise(exercise, *, headless=False, fast=False, data=None):
    with Session(headless=headless, fast=fast) as session:
        print(f"Simulation ready. Logs: {session.folder}", flush=True)
        process = session.java("simproxy.Launcher", exercise, data=data, headless=headless)
        try:
            with (session.folder / "exercise.log").open(encoding="utf-8", errors="replace") as output:
                while process.poll() is None:
                    text = output.read()
                    if text:
                        print(text, end="", flush=True)
                    time.sleep(.1)
                print(output.read(), end="", flush=True)
            result = process.returncode
        except KeyboardInterrupt:
            result = 130
        if result not in (0, 130):
            raise RuntimeError(f"Exercise exited with status {result}")
