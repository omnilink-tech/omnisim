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

"""Patch the Spot URDF so it's suitable for RL training.

Changes:
  1. Replace each lower_leg's <collision><mesh.../></collision> with a
     small sphere at the foot tip (z=-0.32 in lower_leg frame, radius
     0.04 m). The original mesh-collision treats the whole shin as a
     contact surface, which is why pure-physics walking has the body
     dragging on the floor whenever a leg lays flat. A ball at the toe
     is the standard URDF pattern for quadrupeds (ANYmal, Mini Cheetah,
     Spot in MuJoCo).
  2. Add <dynamics damping="0.5" friction="0.05"/> to every revolute
     joint. Critical for RL: without joint damping the policy learns to
     oscillate joints to extract free energy from the simulator.

Run from repo root:
    python projects/policies/research/tools/patch_spot_urdf_for_rl.py

The script is idempotent (already-patched joints are skipped) and edits
the file in place.
"""

from __future__ import annotations

import pathlib
import re
import sys


URDF_PATH = pathlib.Path("projects/robots/boston_dynamics/spot/urdf/spot.urdf")
LEGS = ("front_left", "front_right", "rear_left", "rear_right")


def patch_lower_leg_collision(content: str) -> str:
    """Replace mesh collision in each lower_leg link with a foot sphere."""
    new_collision = (
        '<collision>\n'
        '            <origin xyz="0 0 -0.32" rpy="0 0 0"/>\n'
        '            <geometry>\n'
        '                <sphere radius="0.04"/>\n'
        '            </geometry>\n'
        '        </collision>'
    )
    for leg in LEGS:
        # The lower-leg block uniquely contains the mesh filename pattern.
        old = (
            r'<collision>\s*'
            r'<geometry>\s*'
            r'<mesh filename="package://spot/meshes/'
            + re.escape(f"{leg}_lower_leg_collision.stl")
            + r'" />\s*'
            r'</geometry>\s*'
            r'</collision>'
        )
        n_before = len(content)
        content = re.sub(old, new_collision, content, count=1)
        if len(content) == n_before:
            # idempotency: maybe already patched (no mesh collision left)
            if f"{leg}_lower_leg" in content and "<sphere radius=" not in _block_around(
                content, f'<link name="{leg}_lower_leg">'
            ):
                raise SystemExit(f"failed to patch {leg}_lower_leg collision")
    return content


def _block_around(content: str, anchor: str) -> str:
    """Return the substring from anchor to the closing </link>."""
    i = content.find(anchor)
    if i < 0:
        return ""
    j = content.find("</link>", i)
    return content[i:j]


def add_joint_damping(content: str) -> str:
    """Add <dynamics damping=... friction=.../> to every revolute joint
    that doesn't already have one."""
    damping = '<dynamics damping="0.5" friction="0.05"/>'
    # Match a revolute joint block; if it doesn't contain <dynamics, insert
    # one just before </joint>.
    pat = re.compile(
        r'(<joint name="[^"]+" type="revolute">)(.*?)(</joint>)',
        re.DOTALL,
    )

    def repl(m: re.Match) -> str:
        head, body, tail = m.group(1), m.group(2), m.group(3)
        if "<dynamics" in body:
            return m.group(0)
        return head + body + "        " + damping + "\n    " + tail

    new = pat.sub(repl, content)
    return new


def main() -> int:
    if not URDF_PATH.exists():
        print(f"URDF not found: {URDF_PATH}", file=sys.stderr)
        return 1
    src = URDF_PATH.read_text()
    out = patch_lower_leg_collision(src)
    out = add_joint_damping(out)
    if out == src:
        print("no changes (already patched)")
        return 0
    URDF_PATH.write_text(out)
    n_spheres = out.count('<sphere radius="0.04"/>')
    n_dampings = out.count('<dynamics damping="0.5"')
    print(f"patched: {n_spheres} foot spheres, {n_dampings} joint <dynamics>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
