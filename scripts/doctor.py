"""Local installation checks; optionally verify an already-running simulator."""

import argparse
import importlib
from pathlib import Path
import subprocess
import sys

from common import load_config, local_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="Also check a simulator on localhost:23000 (read only)")
    args = parser.parse_args()
    config = load_config()
    java = local_path(config, "java_home") / "bin" / ("java.exe" if sys.platform == "win32" else "java")
    subprocess.run([str(java), "-version"], check=True)
    for package in ["numpy", "matplotlib", "pglfd.geom.so3", "pglfd.model.kmp", "sinkcleaning", "coppeliasim_zmqremoteapi_client"]:
        importlib.import_module(package)
        print(f"Import OK: {package}")
    for key in ["simulator", "simulator_resources"]:
        if not local_path(config, key).exists():
            raise RuntimeError(f"Missing {key}; rerun setup.")
    model = local_path(config, "simulator_resources") / "models/robots/non-mobile/KUKA LBR iiwa 7 R800.ttm"
    if not model.is_file():
        raise RuntimeError("The installed simulator does not contain the expected iiwa 7 R800 model.")
    print(f"CoppeliaSim {config['coppeliasim_version']} installed at {local_path(config, 'simulator')}")
    if args.sim:
        # Run the client in a child so a disconnected server cannot hang this check.
        subprocess.run([sys.executable, str(Path(__file__).with_name("sim_probe.py"))], timeout=25, check=True)
    print("Local checks passed. Hardware connection and full Sunrise simulation are separate, untested steps.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError, ImportError) as error:
        sys.exit(f"Check failed: {error}")
