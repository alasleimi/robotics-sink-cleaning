"""Shared launch plumbing for the two small record/replay commands."""
import argparse
import json
from pathlib import Path
import os
import subprocess
import time

from common import ROOT, gradle, load_config, local_path
from sim_session import Session


def run(command):
    parser=argparse.ArgumentParser(description=f"{command.capitalize()} a demonstration; simulation unless --robot is specified.")
    parser.add_argument("folder",type=Path,help="Recording directory (must be new when recording)")
    parser.add_argument("--robot",action="store_true",help="Use the actual configured Sunrise controller")
    parser.add_argument("--headless",action="store_true",help="Hide simulator windows")
    parser.add_argument("--scene",choices=["empty","sink","wire"],default="sink")
    if command=="record":
        parser.add_argument("--seconds",type=float,default=0,help="Otherwise Enter stops recording")
        parser.add_argument("--sink",type=Path,help="JSON containing calibrated sink_pose in robot base (metres, XYZW)")
    else:
        parser.add_argument("--approach",action="store_true",help="Allow a slow PTP move to the first pose")
    args=parser.parse_args()
    folder=args.folder.resolve()
    java_args=[str(folder)]
    if command=="record":
        calibration=args.sink or (ROOT/"simulation/scenes/sink.json" if not args.robot and args.scene=="sink" else None)
        java_args += [str(args.seconds),str(calibration.resolve()) if calibration else "-"]
    elif args.approach:
        java_args.append("--approach")
    if args.robot:
        config=load_config()
        gradle(config,"robotClasspath")
        java=local_path(config,"java_home")/"bin"/("java.exe" if os.name=="nt" else "java")
        subprocess.run([str(java),"-Djava.awt.headless=true","-cp",(ROOT/".tools/robot-classpath.txt").read_text(),
                        "sinkcleaning."+command.capitalize(),*java_args],cwd=ROOT,check=True)
    else:
        with Session(headless=args.headless,scene=args.scene) as session:
            process=session.java("simproxy.DemoLauncher",command,*java_args,headless=args.headless,interactive=True)
            while process.poll() is None and session.bridge.poll() is None:
                time.sleep(.1)
            if process.poll() is None:
                # Let an active recorder flush its partial file if the viewer
                # was closed, then close the remaining Java window.
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.terminate(); process.wait(timeout=5)
                metadata=folder/"metadata.json"
                saved=command=="record" and metadata.exists() and json.loads(metadata.read_text()).get("complete",False)
                result=0 if saved else 1
            else:
                result=process.returncode
            if result: raise SystemExit(result)
