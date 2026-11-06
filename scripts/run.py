"""Convenience commands using the locally installed Java and simulator."""

import argparse
import subprocess
import sys

from common import ROOT, gradle, load_config, local_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["sim", "build", "doctor"])
    args = parser.parse_args()
    config = load_config()
    if args.command == "build":
        gradle(config, "classes", "run")
    elif args.command == "doctor":
        subprocess.run([sys.executable, str(ROOT / "scripts" / "doctor.py")], check=True, cwd=ROOT)
    else:
        executable = local_path(config, "simulator")
        resources = local_path(config, "simulator_resources")
        scene = ROOT / "simulation" / "scenes" / "iiwa7.ttt"
        if not scene.is_file():
            raise RuntimeError(f"Starter scene missing: {scene}")
        # This launches only the simulator. The original course applications are untouched.
        subprocess.Popen([str(executable), "-f" + str(scene)], cwd=resources)
        print("Opened CoppeliaSim with the iiwa 7 R800 starter scene.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        sys.exit(str(error))
