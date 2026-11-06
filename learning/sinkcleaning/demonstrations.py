"""Read the robot/simulator CSV, express poses in task frames and resample."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from pglfd.geom.se3 import invert_transformation, multiply_poses

HEADER="t,x,y,z,qx,qy,qz,qw,j1,j2,j3,j4,j5,j6,j7,fx,fy,fz,mx,my,mz,read_seconds"


def load(folder):
    folder=Path(folder)
    meta=json.loads((folder/"metadata.json").read_text(encoding="utf-8"))
    if meta.get("schema")!=1 or not meta.get("complete"):
        raise ValueError(f"Incomplete or unsupported recording: {folder}")
    if meta.get("pose_frame")!="robot_base":
        raise ValueError("Expected robot-base poses")
    with (folder/"samples.csv").open() as f:
        if f.readline().strip()!=HEADER: raise ValueError("Unexpected CSV columns")
    rows=np.loadtxt(folder/"samples.csv",delimiter=",",skiprows=1,ndmin=2)
    if rows.shape[0]<2 or rows.shape[1]!=22 or not np.all(np.isfinite(rows)):
        raise ValueError("Need at least two finite, complete samples")
    if np.any(np.diff(rows[:,0])<=0) or rows[0,0]<0: raise ValueError("Timestamps must increase")
    if not np.allclose(np.linalg.norm(rows[:,4:8],axis=1),1,atol=.005): raise ValueError("Invalid quaternion")
    return rows,meta


def in_frame(poses,frame):
    frame=np.asarray(frame,dtype=float)
    if frame.shape!=(7,) or not np.all(np.isfinite(frame)) or not np.isclose(np.linalg.norm(frame[3:]),1,atol=.005):
        raise ValueError("Frame must contain finite XYZ and a unit XYZW quaternion")
    return multiply_poses(invert_transformation(frame),np.asarray(poses))


def resample(rows,n=60):
    # Trim only stationary ends; keep intentional pauses inside a demonstration.
    travel=np.linalg.norm(np.diff(rows[:,1:4],axis=0),axis=1)
    rotation=(Rotation.from_quat(rows[1:,4:8])*Rotation.from_quat(rows[:-1,4:8]).inv()).magnitude()
    moving=np.flatnonzero((travel>0.0001)|(rotation>.001))
    if len(moving)<2: raise ValueError("Demonstration has no useful motion")
    rows=rows[moving[0]:moving[-1]+2]
    t=(rows[:,0]-rows[0,0])/(rows[-1,0]-rows[0,0])
    phase=np.linspace(0,1,n)
    pos=np.column_stack([np.interp(phase,t,rows[:,i]) for i in range(1,4)])
    ori=Slerp(t,Rotation.from_quat(rows[:,4:8]))(phase).as_quat()
    return phase,np.c_[pos,ori]


def save_prediction(folder,poses,meta,model,duration=8.):
    folder=Path(folder)
    folder.mkdir(parents=True,exist_ok=False)
    rows=np.zeros((len(poses),22)); rows[:,0]=np.linspace(0,duration,len(poses)); rows[:,1:8]=poses
    np.savetxt(folder/"samples.csv",rows,delimiter=",",header=HEADER,comments="")
    result={key:meta[key] for key in ('schema','units','pose_frame','tcp','tool_template','sink_pose','calibration_file') if key in meta}
    result.update(complete=True,source="learned",learned_model=model,
                  joint_positions_valid=False,wrench_valid=False,samples=len(rows),duration_s=duration)
    (folder/"metadata.json").write_text(json.dumps(result,indent=2))
