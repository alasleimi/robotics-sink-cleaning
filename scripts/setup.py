"""Install pinned local tools and check the project on Windows/macOS.

Run from any directory: uv run --locked scripts/setup.py
Downloads stay under .tools; no administrator rights or global PATH edits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import urllib.request
import zipfile

from common import CONFIG, ROOT, TOOLS, gradle, platform_key


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def download(asset: dict) -> Path:
    if not asset["url"].startswith("https://") or len(asset["sha256"]) != 64:
        raise ValueError("Every download needs HTTPS and a pinned SHA-256.")
    cache = TOOLS / "downloads" / asset["filename"]
    cache.parent.mkdir(parents=True, exist_ok=True)
    if cache.is_file() and digest(cache) == asset["sha256"]:
        print(f"Using cached {cache.name}", flush=True)
        return cache
    print(f"Downloading {cache.name} ...", flush=True)
    partial = cache.with_suffix(cache.suffix + ".part")
    request = urllib.request.Request(asset["url"], headers={"User-Agent": "robotics-sink-cleaning-setup/0.1"})
    try:
        with urllib.request.urlopen(request, timeout=90) as response, partial.open("wb") as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
        if digest(partial) != asset["sha256"]:
            raise RuntimeError(f"Hash mismatch for {cache.name}; installation stopped.")
        partial.replace(cache)
    finally:
        partial.unlink(missing_ok=True)
    return cache


def validate_zip_member(destination: Path, name: str) -> None:
    # Reject absolute names and traversal before handing trusted archives to ditto.
    member = Path(name.replace("\\", "/"))
    if member.is_absolute() or not (destination / member).resolve().is_relative_to(destination.resolve()):
        raise ValueError(f"Unsafe archive path: {name}")


def unpack(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    if archive.name.endswith(".zip"):
        with zipfile.ZipFile(archive) as zipped:
            for member in zipped.infolist():
                validate_zip_member(destination, member.filename)
            if sys.platform == "darwin":
                # Preserve macOS app symlinks, permissions, and signatures.
                subprocess.run(["/usr/bin/ditto", "-x", "-k", str(archive), str(destination)], check=True)
            else:
                zipped.extractall(destination)
    else:
        with tarfile.open(archive) as tar:
            tar.extractall(destination, filter="data")


def install(asset: dict, directory: str, marker: str) -> Path:
    destination = TOOLS / directory / asset["sha256"][:12]
    receipt = destination / ".installed-sha256"
    matches = sorted(p for p in destination.rglob(marker) if p.is_file()) if destination.is_dir() else []
    if receipt.is_file() and receipt.read_text().strip() == asset["sha256"] and matches:
        return matches[0]
    unpack(download(asset), destination)
    matches = sorted(p for p in destination.rglob(marker) if p.is_file())
    if not matches:
        raise RuntimeError(f"Installed archive does not contain {marker}")
    receipt.write_text(asset["sha256"], encoding="utf-8")
    return matches[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-checks", action="store_true", help="Install tools without compiling/running local checks")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "tool-versions.json").read_text(encoding="utf-8"))
    key = platform_key()
    target = manifest["platforms"][key]
    print(f"Preparing {key}: Python, Java, Gradle, robosuite and MuJoCo.", flush=True)
    java = install(target["jdk"], "jdk", "java.exe" if os.name == "nt" else "java")
    java_home = java.parent.parent
    config = {
        "platform": key,
        "java_home": java_home.relative_to(ROOT).as_posix(),
        "simulator_backend": "robosuite",
        "robosuite_version": manifest["robosuite"],
        "mujoco_version": manifest["mujoco"],
    }
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    CONFIG.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    if not args.skip_checks:
        gradle(config, "classes", "run")
        subprocess.run([sys.executable, str(ROOT / "scripts" / "doctor.py")], cwd=ROOT, check=True)
    print("Setup complete. Open the simulator with: uv run --locked scripts/run.py sim", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(f"Setup failed: {error}")
