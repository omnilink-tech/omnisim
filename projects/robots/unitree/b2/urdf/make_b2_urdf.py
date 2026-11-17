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

"""Generate the in-repo OmniSim B2 URDF from the upstream unitree_ros
b2_description URDF. Same transforms as make_go2_urdf.py:
  1. Mesh refs package://b2_description/meshes/<f> -> package://b2/meshes/<f>.
  2. A <rest>VALUE</rest> child injected into each leg revolute joint so the
     robot SPAWNS in the standing crouch (hip 0, thigh 0.775, calf -1.550 ==
     b2_trot_gait.standing_pose) instead of all-zeros (at q=0 the calf is
     outside its [-2.82,-0.43] limit, which makes the load pose explode).
  3. A foot collision SPHERE (r 0.032 at 0 0 -0.35) injected onto each calf
     link -- the upstream *_foot collider is a dont_collapse fixed child the
     importer makes inert, so only the calf would otherwise collide.

Run:  python make_b2_urdf.py [path/to/upstream/b2_description.urdf]
"""
from __future__ import annotations
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "b2.urdf"

# standing crouch == b2_trot_gait.standing_pose(GaitParams())
REST = {"hip": 0.0, "thigh": 0.775, "calf": -1.550}


def find_upstream() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    tmp = os.environ.get("TEMP") or os.environ.get("TMP") or "/tmp"
    return Path(tmp) / "unitree_ros/robots/b2_description/urdf/b2_description.urdf"


def main() -> None:
    up = find_upstream()
    if not up.exists():
        raise SystemExit(f"upstream URDF not found: {up}\n"
                         f"sparse-clone unitree_ros/robots/b2_description first.")
    txt = up.read_text(encoding="utf-8")

    txt = txt.replace("package://b2_description/meshes/", "package://b2/meshes/")
    txt = re.sub(r'<robot name="[^"]*">', '<robot name="b2">', txt, count=1)

    # foot contact sphere on each calf link (r 0.032 == upstream foot sphere)
    foot_coll = ('\n    <collision>\n'
                 '      <origin xyz="0 0 -0.35" rpy="0 0 0"/>\n'
                 '      <geometry>\n'
                 '        <sphere radius="0.032"/>\n'
                 '      </geometry>\n'
                 '    </collision>')
    txt = re.sub(r'(<link name="(?:FL|FR|RL|RR)_calf">)',
                 lambda m: m.group(1) + foot_coll, txt)

    # inject <rest> into each leg revolute joint
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

    txt = re.sub(
        r'<joint name="([^"]+)" type="revolute">.*?</joint>',
        inject, txt, flags=re.DOTALL)

    OUT.write_text(txt, encoding="utf-8")
    n_rest = txt.count("<rest>")
    # count the spheres injected onto the calf links (the opening <link
    # name="*_calf"> immediately followed by our <collision>); the upstream
    # *_foot links also carry r=0.032 spheres, so don't count those.
    n_foot = len(re.findall(r'_calf">\n    <collision>\n      <origin xyz="0 0 -0.35"', txt))
    print(f"wrote {OUT}  ({n_rest} <rest> tags, {n_foot} calf foot spheres; expect 12, 4)")
    assert n_rest == 12 and n_foot == 4, "unexpected injection count"


if __name__ == "__main__":
    main()
