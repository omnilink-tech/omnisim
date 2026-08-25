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

"""Generate b2_ghost.urdf: the B2 URDF with every <inertial> and
<collision> stripped, leaving only <visual> + <joint>. The OmniSim URDF
importer then emits NO Physics on any link, so the robot imports as a fully
KINEMATIC, visual-only articulation -- Newton skips it (no-Physics Solids
are not registered), it cannot fall, and its joints are driven directly by
motor.setPosition(). This is the trot-gait-model GHOST body (same recipe as
projects/robots/omnisim/omniquad/urdf/make_ghost_urdf.py).

Run after make_b2_urdf.py (it reads the generated b2.urdf).
"""
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "b2.urdf"
DST = HERE / "b2_ghost.urdf"

tree = ET.parse(SRC)
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
root.set("name", root.get("name", "b2") + "_ghost")
tree.write(DST, encoding="utf-8", xml_declaration=True)
nlink = len(root.findall("link"))
njoint = len(root.findall("joint"))
nvis = sum(len(l.findall("visual")) for l in root.findall("link"))
print(f"wrote {DST.name}: {nlink} links, {njoint} joints, {nvis} visuals; "
      f"stripped {n_inert} inertial + {n_coll} collision blocks")
