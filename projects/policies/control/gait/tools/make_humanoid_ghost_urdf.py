# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Strip <inertial> and <collision> from a humanoid URDF, leaving <visual> +
<joint>, so the OmniSim importer emits a fully KINEMATIC, visual-only ghost
(no Physics on any link -> Newton skips it, it cannot fall, joints driven by
motor.setPosition()). The ghost URDF is written next to the source so its
relative mesh paths still resolve.

Usage:  python projects/policies/control/gait/tools/make_humanoid_ghost_urdf.py h1
        python projects/policies/control/gait/tools/make_humanoid_ghost_urdf.py h1
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]

SRC = {
    "h1": "projects/robots/unitree/h1/urdf/h1.urdf",
}


def _urdf_path(robot, rel):
    """Resolve a registry URDF path, failing with the reason when that robot
    package is not in this tree -- rather than a bare FileNotFoundError out of
    ET.parse(). projects/robots/nasa/ is held from the public OmniSim snapshot."""
    p = ROOT / rel
    if not p.exists():
        why = "that robot package is not present in this tree"
        raise SystemExit(f"error: no URDF for '{robot}' at {rel} -- {why}.")
    return p


def make(robot):
    src = _urdf_path(robot, SRC[robot])
    dst = src.with_name(f"{robot}_ghost.urdf")
    tree = ET.parse(src)
    root = tree.getroot()
    n_inert = n_coll = 0
    for link in root.findall("link"):
        for tag in ("inertial", "collision"):
            for el in link.findall(tag):
                link.remove(el)
                if tag == "inertial":
                    n_inert += 1
                else:
                    n_coll += 1
    root.set("name", root.get("name", robot) + "_ghost")
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    nlink = len(root.findall("link"))
    njoint = len(root.findall("joint"))
    nvis = sum(len(l.findall("visual")) for l in root.findall("link"))
    print(f"wrote {dst.name}: {nlink} links, {njoint} joints, {nvis} visuals; "
          f"stripped {n_inert} inertial + {n_coll} collision blocks")


if __name__ == "__main__":
    make(sys.argv[1] if len(sys.argv) > 1 else "h1")
