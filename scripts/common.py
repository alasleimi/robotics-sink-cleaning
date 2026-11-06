"""Shared local-tool paths and platform detection. No robot connection code."""

from __future__ import annotations

import json
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / ".tools"
CONFIG = TOOLS / "local.json"


def platform_key(system: str | None = None, machine: str | None = None) -> str:
    system = system or platform.system()
    machine = (machine or platform.machine()).lower()
    if system == "Windows" and machine in ("amd64", "x86_64"):
        return "windows-x64"
    if system == "Darwin" and machine in ("arm64", "aarch64"):
        return "mac-arm64"
    if system == "Darwin" and machine in ("x86_64", "amd64"):
        return "mac-x64"
    raise RuntimeError(f"Unsupported platform: {system}/{machine}. Supported: Windows x64, Intel Mac, Apple Silicon Mac.")


def load_config() -> dict:
    if not CONFIG.is_file():
        raise RuntimeError("Run uv run --locked scripts/setup.py first.")
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    if config["platform"] != platform_key():
        raise RuntimeError("Local configuration belongs to another platform. Run setup again.")
    return config


def local_path(config: dict, name: str) -> Path:
    return ROOT / config[name]


def java_env(config: dict) -> dict[str, str]:
    env = dict(os.environ)
    env["JAVA_HOME"] = str(local_path(config, "java_home"))
    env["PATH"] = str(local_path(config, "java_home") / "bin") + os.pathsep + env.get("PATH", "")
    return env


def gradle(config: dict, *args: str) -> None:
    wrapper = ROOT / ("gradlew.bat" if os.name == "nt" else "gradlew")
    subprocess.run([str(wrapper), "--no-daemon", *args], cwd=ROOT, env=java_env(config), check=True)
