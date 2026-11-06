"""Plot a saved demonstration and report its actual sample timing."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sinkcleaning.demonstrations import load

if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder"); p.add_argument("--output",default=".tools/demonstration.png")
    a=p.parse_args(); rows,meta=load(a.folder)
    dt=np.diff(rows[:,0])
    print(json.dumps({"source":meta["source"],"samples":len(rows),"median_hz":float(1/np.median(dt)),
                     "max_gap_s":float(dt.max()),"duration_s":float(rows[-1,0]-rows[0,0])},indent=2))
    fig=plt.figure(figsize=(10,4)); ax=fig.add_subplot(121,projection="3d")
    ax.plot(*rows[:,1:4].T); ax.set(xlabel="Base x (m)",ylabel="Base y (m)",zlabel="Base z (m)")
    ax=fig.add_subplot(122); ax.plot(rows[:-1,0],dt*1000); ax.set(xlabel="Time (s)",ylabel="Sample interval (ms)")
    Path(a.output).parent.mkdir(parents=True,exist_ok=True)
    fig.tight_layout(); fig.savefig(a.output,dpi=150); plt.close(fig)
