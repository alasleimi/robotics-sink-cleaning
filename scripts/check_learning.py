"""Train both PGLfD models and replay the raw/learned paths in the sink scene."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from common import ROOT
from sim_session import Session
from make_demos import wait_motion
from train_kmp import train
from sinkcleaning.demonstrations import load


def main():
    with Session(headless=True,fast=True,scene='sink') as s:
        print('Learning integration:',s.folder,flush=True)
        models=s.folder/'models'
        report={'training':train(ROOT/'data/examples/sink/demos',models),'replay':{}}
        folders=[('recorded',ROOT/'data/examples/sink/demos/wipe-01'),
                 ('kmp',models/'kmp'),('tpkmp',models/'tpkmp')]
        for name,folder in folders:
            rows,_=load(folder)
            # Explicit clearance approach for this fixed fixture, not a planner.
            above=s.call('state')['tcp']; above[2]=.5
            wait_motion(s,{'type':'lin','pose':above,'cartVelocity':.08})
            above=rows[0,1:8].copy(); above[2]=.5
            wait_motion(s,{'type':'ptp','pose':above.tolist(),'jointVelocity':.2})
            wait_motion(s,{'type':'lin','pose':rows[0,1:8].tolist(),'cartVelocity':.04})
            p=s.java('simproxy.DemoLauncher','replay',str(folder),label=name)
            assert p.wait(timeout=180)==0,(s.folder/(name+'.log')).read_text()
            pose=s.call('state')['tcp']
            distance=float(np.linalg.norm(np.array(pose[:3])-rows[-1,1:4])*1000)
            angle=float((Rotation.from_quat(pose[3:])*Rotation.from_quat(rows[-1,4:8]).inv()).magnitude())
            assert distance<2 and angle<.02,(name,distance,angle)
            report['replay'][name]={'final_position_error_mm':distance,'final_orientation_error_rad':angle}
            print('PASS',name,'replay',flush=True)
        report['hardware']='not connected'
        (s.folder/'learning-report.json').write_text(json.dumps(report,indent=2))
        print('Learning checks passed:',s.folder/'learning-report.json',flush=True)


if __name__=='__main__':main()
