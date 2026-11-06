"""Create labelled, scripted simulator demonstrations through the real recorder.

These are synthetic teaching inputs, not recordings made by a human or real robot.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from common import ROOT
from sim_session import Session


def wait_motion(session,motion):
    ident=session.call("submit",motion=motion)["id"]
    deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        state=session.call("motion",id=ident)
        if state["state"]=="error": raise RuntimeError(state["error"])
        if state["state"]=="finished": return
        time.sleep(.05)
    raise TimeoutError("Motion timed out")


def make_demos(output,count=5,headless=True):
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=False)
    with Session(headless=headless,scene="sink") as s:
        print("Simulator logs:",s.folder,flush=True)
        report={"sessions":[str(s.folder)],"demonstrations":[]}
        for index in range(count):
            folder=output/f"wipe-{index+1:02}"
            # A small back-and-forth floor wipe, with varied start/end positions.
            phase=np.linspace(0,1,161)
            offset=(index-(count-1)/2)*.004
            poses=np.column_stack((.49+.10*phase, offset+.045*np.sin(2*np.pi*phase),
                np.full(len(phase),.286),np.zeros(len(phase)),np.ones(len(phase)),
                np.zeros(len(phase)),np.zeros(len(phase))))
            s.call("handguide",enabled=False)
            # Lift before repositioning so the sponge does not cut through a rim.
            current=s.call("state")["tcp"]; above=list(current); above[2]=.5
            wait_motion(s,{"type":"lin","pose":above,"cartVelocity":.08})
            above=poses[0].copy(); above[2]=.5
            wait_motion(s,{"type":"ptp","pose":above.tolist(),"jointVelocity":.2})
            hover=poses[0].copy(); hover[2]=.292
            wait_motion(s,{"type":"lin","pose":hover.tolist(),"cartVelocity":.04})
            # Solve all teaching targets first; the recording samples actual physics.
            seed=s.call("state")["q"]; targets=[]
            for pose in poses:
                seed=s.call("inverse",pose=pose.tolist(),seed=seed,e1=0,status=2)["q"]
                targets.append(seed)
            p=s.java("simproxy.DemoLauncher","record",str(folder),"0",str(ROOT/"simulation/scenes/sink.json"),label=f"record-{index}")
            deadline=time.monotonic()+30
            while "RECORDING" not in (s.folder/f"record-{index}.log").read_text(errors="replace"):
                if p.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError((s.folder/f"record-{index}.log").read_text())
                time.sleep(.05)
            s.call("handguide",enabled=True)
            start=time.monotonic(); contacts=0
            for k,q in enumerate(targets):
                s.call("teach",q=q)
                contacts=max(contacts,s.call("state")["contacts"])
                time.sleep(max(0,start+k*.05-time.monotonic()))
            time.sleep(.5)
            s.call("handguide",enabled=False)
            if p.wait(timeout=30): raise RuntimeError((s.folder/f"record-{index}.log").read_text())
            meta=json.loads((folder/"metadata.json").read_text())
            meta["demonstrator"]="scripted simulator teaching target"
            meta["max_scene_contacts"]=contacts
            (folder/"metadata.json").write_text(json.dumps(meta,indent=2))
            report["demonstrations"].append({"folder":str(folder),"samples":meta["samples"],"contacts":contacts})
            print("Recorded",folder.name,meta["samples"],"samples",flush=True)
        (output/"generation.json").write_text(json.dumps(report,indent=2))
    return report


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path); p.add_argument("--count",type=int,default=5)
    p.add_argument("--show",action="store_true")
    args=p.parse_args()
    if args.count<2: p.error("Use at least two demonstrations")
    make_demos(args.output,args.count,not args.show)
