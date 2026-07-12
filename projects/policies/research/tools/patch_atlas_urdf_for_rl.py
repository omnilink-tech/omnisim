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

"""Patch the Atlas URDF so it's suitable for RL training.

Changes (all idempotent — script can be re-run safely):

  1. **Foot contact rewrite.** Drake's chull foot mesh is small (only
     the foot's upper plate makes it into the chull) and oriented such
     that its bounding box doesn't cover the actual sole area. MuJoCo
     mesh-vs-plane contact then becomes flaky and the robot can't
     balance even at the nominal pose.

     Drake's own URDF marks the right contact polygon already: 4
     contact-point spheres per foot at (xy = heel-back × ±0.066,
     toe-front × ±0.066), but they ship with radius="0.0" to disable
     them in favour of an external contact solver. We:
       a. Replace each `<sphere radius="0.0"/>` with `radius="0.015"`,
          turning the disabled markers into 1.5 cm contact balls.
       b. Remove the foot's `<mesh ... l_foot_chull.obj.../>` and
          `<mesh ... r_foot_chull.obj.../>` collisions entirely, so
          contact only happens through those 4 corner spheres.
     This gives a clean 14 × 26 cm contact polygon that both MJX and
     Webots/ODE handle reliably.

  2. **Joint dynamics bump.** Drake ships damping=0.1 friction=0 on
     all revolute joints. Too soft for a 175 kg biped under position
     control — the policy learns to pump joints at solver frequency
     to extract free energy. Raise to damping=2.0 friction=0.1.

Run from repo root:
    python projects/policies/research/tools/patch_atlas_urdf_for_rl.py
"""
from __future__ import annotations

import pathlib
import re
import sys


URDF_PATH = pathlib.Path(
    "projects/robots/boston_dynamics/atlas/urdf/atlas_convex_hull.urdf"
)

DAMPING = 2.0
FRICTION = 0.1
FOOT_SPHERE_RADIUS = 0.015


def enable_corner_contact_spheres(text: str) -> tuple[str, int]:
    pat = re.compile(r'<sphere\s+radius="0\.0"\s*/>')
    new = f'<sphere radius="{FOOT_SPHERE_RADIUS}"/>'
    out, n = pat.subn(new, text)
    return out, n


def strip_foot_mesh_collisions(text: str) -> tuple[str, int]:
    """Remove `<collision ... <mesh ... [lr]_foot_chull.obj /> </collision>`
    blocks so foot contact only goes through the corner spheres."""
    pat = re.compile(
        r'\s*<collision\b[^>]*>\s*'
        r'<geometry>\s*'
        r'<mesh\s+filename="meshes/[lr]_foot_chull\.obj"\s*/>\s*'
        r'</geometry>\s*'
        r'</collision>',
        re.DOTALL,
    )
    out, n = pat.subn("", text)
    return out, n


def bump_joint_damping(text: str) -> tuple[str, int]:
    """Replace `<dynamics damping="0.1" friction="0"/>` (Drake default)
    with the RL-tuned values. Idempotent: only matches the defaults."""
    pat = re.compile(r'<dynamics\s+damping="0\.1"\s+friction="0"\s*/>')
    new = f'<dynamics damping="{DAMPING}" friction="{FRICTION}"/>'
    return pat.subn(new, text)


def main() -> int:
    if not URDF_PATH.exists():
        print(f"URDF not found: {URDF_PATH}", file=sys.stderr)
        return 1
    src = URDF_PATH.read_text()
    out, n_spheres = enable_corner_contact_spheres(src)
    out, n_foot_mesh = strip_foot_mesh_collisions(out)
    out, n_damp = bump_joint_damping(out)
    if out == src:
        print("no changes (already patched)")
        return 0
    URDF_PATH.write_text(out)
    print(f"patched: enabled {n_spheres} corner-contact spheres "
          f"(r={FOOT_SPHERE_RADIUS}), stripped {n_foot_mesh} foot mesh "
          f"collisions, bumped {n_damp} joint <dynamics> blocks "
          f"(damping={DAMPING} friction={FRICTION})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
