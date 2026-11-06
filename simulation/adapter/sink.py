"""A white O.novo-sized basin. Product envelope is sourced; bowl contours approximate."""
import xml.etree.ElementTree as ET
import json
from pathlib import Path
import numpy as np

SINK = json.loads((Path(__file__).resolve().parents[1]/"scenes/sink.json").read_text())


def add_sink(arena):
    # Keep robot-base coordinates unchanged; the floor is 45 cm below its mount.
    arena.worldbody.find("geom[@name='floor']").set("pos", "0 0 -.45")
    ET.SubElement(arena.worldbody,"geom",name="robot_pedestal",type="box",pos="0 0 -.225",
                  size=".13 .13 .225",rgba=".25 .28 .3 1",group="1")
    pose=SINK["sink_pose"]
    body=ET.SubElement(arena.worldbody,"body",name="sink",pos=" ".join(map(str,pose[:3])),
                       quat=" ".join(map(str,[pose[6],*pose[3:6]])))
    rings=[(.115,.17,-.125),(.15,.22,-.075),(.18,.255,-.02),(.23,.30,0)]
    asset=arena.asset
    def solid(name, vertices, color=".94 .94 .91 1"):
        ET.SubElement(asset,"mesh",name=name,vertex=" ".join(map(str,np.array(vertices).ravel())))
        ET.SubElement(body,"geom",name=name,type="mesh",mesh=name,rgba=color,group="1",
                      friction=".5 .005 .0001",solref=".02 1")
    n=32
    # Independent convex wedges preserve the open concavity for contact physics.
    for band,(a,b) in enumerate(zip(rings,rings[1:])):
        for i in range(n):
            angles=[2*np.pi*i/n,2*np.pi*(i+1)/n]
            top=[[rx*np.cos(t),ry*np.sin(t),z] for rx,ry,z in (a,b) for t in angles]
            solid(f"basin_{band}_{i}",top+[[x,y,z-.016] for x,y,z in top])
    floor=[[.115*np.cos(t),.17*np.sin(t),z] for z in (-.125,-.175) for t in np.linspace(0,2*np.pi,n,endpoint=False)]
    solid("basin_floor",floor)
    ET.SubElement(body,"geom",name="drain",type="cylinder",pos=".055 0 -.124",size=".02 .001",
                  rgba=".3 .32 .34 1",contype="0",conaffinity="0",group="1")
    for name,ends in [("tap_stem",[.205,0,0,.205,0,.14]),("tap_spout",[.205,0,.14,.11,0,.14])]:
        ET.SubElement(body,"geom",name=name,type="capsule",fromto=" ".join(map(str,ends)),size=".013",
                      rgba=".55 .58 .6 1",group="1")
    ET.SubElement(body,"geom",name="sink_wall",type="box",pos=".26 0 -.25",size=".025 .6 .6",
                  rgba=".78 .84 .87 1",group="1")
