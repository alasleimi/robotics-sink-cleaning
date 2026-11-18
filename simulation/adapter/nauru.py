"""User-supplied STEP visuals plus a concave, approximate collision surface."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

SIMULATION = Path(__file__).resolve().parents[1]


def add_nauru(arena):
    assets = SIMULATION/"assets/nauru"
    geometry = json.loads((assets/"geometry.json").read_text())
    scene = json.loads((SIMULATION/"scenes/nauru.json").read_text())
    pose = scene["sink_pose"]
    floor = scene["rim_z_robot_base_m"]-.85
    arena.worldbody.find("geom[@name='floor']").set("pos", f"0 0 {floor}")
    ET.SubElement(arena.worldbody, "geom", name="robot_pedestal", type="box",
                  pos=f"0 0 {floor/2}", size=f".13 .13 {-floor/2}", rgba=".25 .28 .3 1", group="1")
    body = ET.SubElement(arena.worldbody, "body", name="sink", pos=" ".join(map(str, pose[:3])),
                         quat=" ".join(map(str, [pose[6], *pose[3:6]])))
    ET.SubElement(arena.asset, "mesh", name="nauru_visual", file=str(assets/"nauru.obj"))
    ET.SubElement(arena.asset, "hfield", name="nauru_surface", file=str(assets/"nauru.hfield"),
                  size=" ".join(map(str, geometry["hfield_size"])))
    ET.SubElement(body, "geom", name="nauru_visual", type="mesh", mesh="nauru_visual",
                  rgba=".96 .96 .94 1", contype="0", conaffinity="0", group="1")
    ET.SubElement(body, "geom", name="nauru_contact", type="hfield", hfield="nauru_surface",
                  rgba=".3 .6 .9 .3", group="0", friction=".5 .005 .0001", solref=".02 1")
