# /// script
# requires-python = ">=3.12"
# dependencies = ["cadquery==2.6.1", "numpy>=2,<3"]
# ///
"""One-time CAD conversion: uv run scripts/convert_nauru.py path/to/sink.step.

Playback uses the committed OBJ/height field and does not install CadQuery.
"""
import argparse
import hashlib
import json
from pathlib import Path
import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def convert(source, output):
    output.mkdir(parents=True, exist_ok=True)
    shape = cq.importers.importStep(str(source)).val()  # STEP units -> mm
    vertices, faces = shape.tessellate(1., .15)
    vertices = np.array([v.toTuple() for v in vertices]) / 1000
    faces = np.array(faces)
    with (output / "nauru.obj").open("w") as f:
        f.write("# NAURU STEP tessellation; coordinates in metres\n")
        np.savetxt(f, vertices, fmt="v %.8f %.8f %.8f")
        np.savetxt(f, faces + 1, fmt="f %d %d %d")
    # Upper envelope of the CAD triangles. Unlike a single mesh convex hull,
    # this retains the basin cavity. Undercuts/overflow tunnel are not represented.
    extent = np.max(np.abs(vertices[:, :2]), axis=0)
    x = np.linspace(-extent[0], extent[0], 59)
    y = np.linspace(-extent[1], extent[1], 45)
    xx, yy = np.meshgrid(x, y)
    points = np.c_[xx.ravel(), yy.ravel()]
    height = np.zeros(len(points))
    for triangle in vertices[faces]:
        a, b, c = triangle
        basis = np.column_stack((b[:2]-a[:2], c[:2]-a[:2]))
        if abs(np.linalg.det(basis)) < 1e-12:
            continue
        inside_box = np.all((points >= triangle[:, :2].min(0)-1e-9) &
                            (points <= triangle[:, :2].max(0)+1e-9), axis=1)
        ids = np.flatnonzero(inside_box)
        uv = (points[ids]-a[:2]) @ np.linalg.inv(basis).T
        keep = np.all(uv >= -1e-8, axis=1) & (uv.sum(1) <= 1+1e-8)
        ids, uv = ids[keep], uv[keep]
        height[ids] = np.maximum(height[ids], a[2]+uv @ [b[2]-a[2], c[2]-a[2]])
    with (output / "nauru.hfield").open("wb") as f:
        f.write(np.array([len(y), len(x)], dtype="<i4").tobytes())
        f.write(height.astype("<f4").tobytes())
    # STEP face 2 is the horizontal top: outer boundary, tap hole, bowl boundary.
    rim = {}
    for label, wire in zip(("outer", "tap", "inner"), shape.Faces()[2].Wires()):
        points = []
        for edge in wire.Edges():
            samples, _ = edge.sample(max(3, int(edge.Length()/2)))
            points.extend(p.toTuple() for p in samples)
        rim[label] = (np.array(points)/1000).tolist()
    (output / "rim.json").write_text(json.dumps(rim))
    metadata = {
        "source": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "bounds_m": [vertices.min(0).tolist(), vertices.max(0).tolist()],
        "hfield_size": [*extent.tolist(), float(height.max()), .01],
        "hfield_grid": [len(y), len(x)], "vertices": len(vertices), "triangles": len(faces),
        "visual": "STEP tessellation, 1 mm linear / 0.15 rad angular tolerance",
        "collision": "~10 mm upper-envelope height field; cavity retained; undercuts omitted",
        "cadquery_docs": "https://cadquery.readthedocs.io/en/latest/importexport.html",
        "mujoco_docs": "https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#asset-hfield",
    }
    (output / "geometry.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("step", type=Path)
    p.add_argument("--output", type=Path, default=ROOT/"simulation/assets/nauru")
    args = p.parse_args()
    convert(args.step, args.output)
