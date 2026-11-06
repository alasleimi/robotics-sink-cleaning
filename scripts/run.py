"""Convenience commands using the locally installed Java and simulator."""

import argparse
import subprocess
import sys

from common import ROOT, gradle, load_config, local_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["sim", "exercise", "build", "doctor"])
    parser.add_argument("exercise", nargs="?", choices=["01", "02", "03", "04", "05a", "05b", "06", "07", "08"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fast", action="store_true", help="Run simulation without wall-clock pacing")
    parser.add_argument("--data", help="Separate teaching directory (default data/simulation/robosuite-sunrise)")
    parser.add_argument("--scene", choices=["empty", "wire", "sink"], default="empty", help="Optional practice scene")
    args = parser.parse_args()
    config = load_config()
    if args.command == "exercise":
        if not args.exercise:
            parser.error("Choose an exercise, e.g. exercise 01")
        from sim_session import run_exercise
        run_exercise(args.exercise, headless=args.headless, fast=args.fast, data=args.data, scene=args.scene)
    elif args.command == "build":
        gradle(config, "classes", "run")
    elif args.command == "doctor":
        subprocess.run([sys.executable, str(ROOT / "scripts" / "doctor.py")], check=True, cwd=ROOT)
    else:
        from sim_session import Session
        import time
        with Session(headless=args.headless, fast=args.fast, build=False, scene=args.scene) as session:
            print(f"Robosuite iiwa7 ready. Logs: {session.folder}", flush=True)
            try:
                while session.bridge.poll() is None:
                    time.sleep(.1)
            except KeyboardInterrupt:
                pass
            if session.bridge.returncode not in (None, 0):
                raise RuntimeError(f"Simulator stopped unexpectedly. Logs: {session.folder}")



if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        sys.exit(str(error))
