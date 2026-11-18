"""Generate a new path from saved KMP/TP-KMP parameters and optional task frames."""
import argparse
import json
from pathlib import Path
import numpy as np
from pglfd.geom.se3 import multiply_poses
from sinkcleaning.learned import predict_saved
from sinkcleaning.demonstrations import save_prediction


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("model", type=Path, help="Motion model directory containing report.json")
    p.add_argument("output", type=Path, help="New directory for the prediction")
    p.add_argument("--kind", choices=("kmp", "tpkmp"), default="tpkmp")
    p.add_argument("--frames", type=Path, help='JSON with "start" and "end" poses in sink frame')
    args = p.parse_args()
    if args.frames and args.kind != "tpkmp": p.error("--frames applies to tpkmp")
    frames = json.loads(args.frames.read_text()) if args.frames else None
    poses, report = predict_saved(args.model, kind=args.kind, frames=frames)
    metadata = json.loads((args.model/args.kind/"metadata.json").read_text())
    save_prediction(args.output, multiply_poses(np.array(report["sink_pose"]), poses), metadata,
                    args.kind, duration=report["duration_s"])
    print("Saved", args.output)
