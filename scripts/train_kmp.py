"""Fit ordinary KMP and task-parameterized KMP with the supplied PGLfD.

The last demonstration is held out. TP-KMP receives its start/end frames;
ordinary KMP is a fixed sink-frame baseline. These are different conditioning
assumptions, not a claim that one algorithm is universally better.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.kernel import MaternKernel
from pglfd.model.kmp import KMP
from pglfd.model.tp import TaskParameterizedModel
from pglfd.geom.se3 import multiply_poses
from sinkcleaning.demonstrations import load,in_frame,resample,save_prediction


def fit(trajectories,phase,path):
    manifold=ProductManifold.from_string("R3*SO3")
    # Aligned repetitions directly give a mean/covariance at each phase.
    # This avoids adding a GMM stage just to estimate the same reference data.
    reference=[Gaussian.mle_from_data(manifold,np.array([demo[i] for demo in trajectories]),reg=1e-6)
               for i in range(len(phase))]
    means=np.array([g.mu for g in reference]); cov=np.array([g.sigma for g in reference])
    np.savez(path,phase=phase,means=means,covariances=cov,length_scale=.15)
    man_in=ProductManifold.from_string("R1")
    return KMP(man_in,manifold,phase,means,MaternKernel(man_in,.15),cov,lambda1=1,lambda2=1)


def means(model,phase):
    result=model(phase.reshape(-1,1))
    poses=np.array([g.mu for g in result])
    if not np.all(np.isfinite(poses)): raise ValueError("Model produced nonfinite poses")
    return poses


def train(root,output):
    folders=sorted(p.parent for p in Path(root).glob("*/samples.csv"))
    if len(folders)<3: raise ValueError("Use at least three takes: two training, one held out")
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    trajectories=[]; metadata=[]
    for folder in folders:
        rows,meta=load(folder)
        if meta.get("sink_pose") is None: raise ValueError(f"Missing sink calibration: {folder}")
        phase,poses=resample(rows,60)
        trajectories.append(in_frame(poses,meta["sink_pose"])); metadata.append(meta)
    training,heldout=trajectories[:-1],trajectories[-1]
    kmp=fit(training,phase,output/"kmp.npz")
    local=[]
    for name,index in [("start",0),("end",-1)]:
        local.append(fit([in_frame(demo,demo[index]) for demo in training],phase,output/(name+".npz")))
    tp=TaskParameterizedModel([("start",heldout[0]),("end",heldout[-1])],local)
    report={"training":[str(f) for f in folders[:-1]],"held_out":str(folders[-1]),
            "reference":"Aligned empirical manifold means/covariances; no GMM",
            "comparison":"KMP uses sink frame; TP-KMP additionally receives held-out start/end frames",
            "demonstrators":[m.get("demonstrator",m["source"]) for m in metadata],"metrics":{}}
    fig=plt.figure(figsize=(12,4)); xy=fig.add_subplot(131); height=fig.add_subplot(132); error=fig.add_subplot(133)
    for demo in training: xy.plot(demo[:,0]*1000,demo[:,1]*1000,color=".8",linewidth=1)
    xy.plot(heldout[:,0]*1000,heldout[:,1]*1000,"k--",label="Held out")
    for label,model in [("kmp",kmp),("tpkmp",tp)]:
        prediction=means(model,phase)
        delta=np.linalg.norm(prediction[:,:3]-heldout[:,:3],axis=1)*1000
        angular=(Rotation.from_quat(prediction[:,3:])*Rotation.from_quat(heldout[:,3:]).inv()).magnitude()
        report["metrics"][label]={"position_rmse_mm":float(np.sqrt(np.mean(delta**2))),
            "orientation_rmse_deg":float(np.rad2deg(np.sqrt(np.mean(angular**2)))),"max_position_error_mm":float(delta.max())}
        base=multiply_poses(np.array(metadata[-1]["sink_pose"]),prediction)
        save_prediction(output/label,base,metadata[-1],label)
        xy.plot(prediction[:,0]*1000,prediction[:,1]*1000,label=label.upper())
        height.plot(phase,prediction[:,2]*1000,label=label.upper()); error.plot(phase,delta,label=label.upper())
    xy.set(xlabel="Sink x (mm)",ylabel="Sink y (mm)",title="Wiping path"); xy.axis("equal"); xy.legend()
    height.plot(phase,heldout[:,2]*1000,"k--"); height.set(xlabel="Progress",ylabel="Sink z (mm)",title="Height")
    error.set(xlabel="Progress",ylabel="Position error (mm)",title="Held-out error"); error.legend()
    fig.tight_layout(); fig.savefig(output/"comparison.png",dpi=160); plt.close(fig)
    (output/"report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report["metrics"],indent=2))
    print("Saved comparison and replayable predictions in",output)
    return report


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("recordings"); p.add_argument("output")
    args=p.parse_args(); train(args.recordings,args.output)
