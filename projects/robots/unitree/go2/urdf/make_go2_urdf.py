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

"""Generate the in-repo OmniSim Go2 URDF from the upstream unitree_ros
go2_description URDF.

Two transforms applied to the upstream file:
  1. Mesh refs  package://go2_description/dae/<f>  ->  package://go2/meshes/<f>
     so the OmniSim importer resolves them against projects/robots/unitree/go2.
  2. A <rest>VALUE</rest> child is injected into each leg revolute joint so the
     robot SPAWNS in the standing crouch (hip 0, thigh 0.789, calf -1.579 ==
     go2_trot_gait.standing_pose) instead of all-zeros -- at q=0 the calf is
     outside its [-2.72,-0.84] limit, which makes the load pose explode.

Run from anywhere:  python make_go2_urdf.py [path/to/upstream/go2_description.urdf]
Default upstream path is the sparse clone under $TEMP/unitree_ros. The output
projects/robots/unitree/go2/urdf/go2.urdf is committed, so this script only
needs re-running to refresh from upstream.
"""
from __future__ import annotations
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "go2.urdf"

# standing crouch == go2_trot_gait.standing_pose(GaitParams())
REST = {"hip": 0.0, "thigh": 0.789, "calf": -1.579}


def find_upstream() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    tmp = os.environ.get("TEMP") or os.environ.get("TMP") or "/tmp"
    return Path(tmp) / "unitree_ros/robots/go2_description/urdf/go2_description.urdf"


def main() -> None:
    up = find_upstream()
    if not up.exists():
        raise SystemExit(f"upstream URDF not found: {up}\n"
                         f"sparse-clone unitree_ros/robots/go2_description first.")
    txt = up.read_text(encoding="utf-8")

    # 1. mesh paths -> repo layout
    txt = txt.replace("package://go2_description/dae/", "package://go2/meshes/")
    # robot name -> go2
    txt = txt.replace('<robot name="go2_description">', '<robot name="go2">')

    # 1b. FOOT CONTACT: the upstream foot collision lives on a separate
    # <link name="*_foot"> joined by a dont_collapse fixed joint, which the
    # OmniSim importer turns into an inert NULL-physics child Solid -- so the
    # only active collider per leg is the calf shin capsule, ~9 cm short of
    # the foot tip. Mirror Spot (whose foot sphere sits on the lower-leg link):
    # inject the foot collision sphere directly onto each calf link, where the
    # importer folds it into the calf body's boundingObject. -0.213 == calf
    # length L2; r 0.022 == upstream foot sphere radius.
    foot_coll = ('\n    <collision>\n'
                 '      <origin xyz="0 0 -0.213" rpy="0 0 0"/>\n'
                 '      <geometry>\n'
                 '        <sphere radius="0.022"/>\n'
                 '      </geometry>\n'
                 '    </collision>')
    txt = re.sub(r'(<link name="(?:FL|FR|RL|RR)_calf">)',
                 lambda m: m.group(1) + foot_coll, txt)

    # 2. inject <rest> into each leg revolute joint, right before </joint>.
    def inject(m: re.Match) -> str:
        block = m.group(0)
        jname = m.group(1)
        kind = ("hip" if jname.endswith("_hip_joint") else
                "thigh" if jname.endswith("_thigh_joint") else
                "calf" if jname.endswith("_calf_joint") else None)
        if kind is None:
            return block
        rest = f'    <rest>{REST[kind]}</rest>\n  '
        return block[:-len("</joint>")] + rest + "</joint>"

    # match each <joint name="..." type="revolute"> ... </joint>
    txt = re.sub(
        r'<joint name="([^"]+)" type="revolute">.*?</joint>',
        inject, txt, flags=re.DOTALL)

    OUT.write_text(txt, encoding="utf-8")
    n_rest = txt.count("<rest>")
    print(f"wrote {OUT}  ({n_rest} <rest> tags, expect 12)")
    assert n_rest == 12, f"expected 12 rest tags, got {n_rest}"


if __name__ == "__main__":
    main()
