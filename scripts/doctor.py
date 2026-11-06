"""Check pinned local tools and optionally run a short isolated physics probe."""
import argparse
import importlib
from importlib.metadata import version
from pathlib import Path
import subprocess
import sys
from common import load_config, local_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="Create and close an isolated headless robosuite scene")
    args = parser.parse_args()
    config = load_config()
    java = local_path(config, "java_home") / "bin" / ("java.exe" if sys.platform == "win32" else "java")
    subprocess.run([str(java), "-version"], check=True)
    for package in ["numpy", "matplotlib", "pglfd.geom.so3", "pglfd.model.kmp", "sinkcleaning", "robosuite", "mujoco"]:
        importlib.import_module(package)
        print(f"Import OK: {package}")
    for name in ("robosuite", "mujoco"):
        actual = version(name)
        if actual != config[name + "_version"]:
            raise RuntimeError(f"{name} version mismatch; rerun uv run --locked scripts/setup.py")
        print(f"{name} {actual}")
    if args.sim:
        subprocess.run([sys.executable, str(Path(__file__).with_name("sim_probe.py"))], timeout=90, check=True)
    print("Local checks passed.")


if __name__ == "__main__":
    main()
