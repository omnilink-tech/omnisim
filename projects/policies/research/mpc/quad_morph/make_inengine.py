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

"""In-engine A/B assets for the quad foot-redesign: a CONTACT-PATCH (box) foot copy
of the Spot deploy URDF + a terrain demo world driven by the deterministic
in-engine locomotion MPC (OMNISIM_INENGINE_MPC_LOCO).

Writes ADDITIVE, clearly-named copies (the originals are NEVER modified):
  - projects/robots/boston_dynamics/spot/urdf/spot_bigfoot.urdf   (4 foot collision
        spheres -> a flat box sole; lives in the urdf dir so the relative meshes/
        paths still resolve)
  - projects/policies/worlds/spot_terrain_mpc_bigfoot.wbt          (the rough-track
        terrain world, URDFRobot url repointed at the bigfoot urdf)

Foot box (URDF size = FULL extents) = 2x the proven offline `boxwide` half-extents
(0.045, 0.075, 0.013) -> (0.09, 0.15, 0.026), centre z -0.342 so the box BOTTOM sits
at -0.355 = the original sphere's bottom (pos -0.32 - r 0.035). The leg IK targets
the kinematic tip independent of the geom, so the standing settle / gait are
unchanged -- only the contact patch changes. Same clean foot-isolation as the
G1/H1 experiment; offline A/B: ~+85% forward speed vs the point foot.
"""
from __future__ import annotations
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())

URDF = REPO / "projects/robots/boston_dynamics/spot/urdf/spot.urdf"
URDF_OUT = REPO / "projects/robots/boston_dynamics/spot/urdf/spot_bigfoot.urdf"
WORLD = REPO / "projects/policies/research/worlds/spot_rough_track.wbt"
# Keep the demo world in research/worlds/ so the `spot_walk_deploy` controller
# (research/controllers/) resolves -- OmniSim searches <world-project>/controllers/,
# and policies/worlds/ has no controllers/ sibling (=> falls back to <generic>).
WORLD_OUT = REPO / "projects/policies/research/worlds/spot_terrain_mpc_bigfoot.wbt"


def repl(txt, old, new, n):
    c = txt.count(old)
    assert c == n, f"expected {n}x {old!r}, found {c}"
    return txt.replace(old, new)


u = URDF.read_text(encoding="utf-8")
u = repl(u, '<origin xyz="0 0 -0.32" rpy="0 0 0"/>', '<origin xyz="0 0 -0.342" rpy="0 0 0"/>', 4)
u = repl(u, '<sphere radius="0.035"/>', '<box size="0.09 0.15 0.026"/>', 4)
URDF_OUT.write_text(u, encoding="utf-8")
print(f"wrote {URDF_OUT.relative_to(REPO)}")

w = WORLD.read_text(encoding="utf-8")
w = repl(w, '"../../../robots/boston_dynamics/spot/urdf/spot.urdf"',
         '"../../../robots/boston_dynamics/spot/urdf/spot_bigfoot.urdf"', 1)
w = repl(w, 'title "Spot walk deploy"',
         'title "Spot BIGFOOT terrain -- deterministic MPC walk (foot-redesign A/B)"', 1)
WORLD_OUT.write_text(w, encoding="utf-8")
print(f"wrote {WORLD_OUT.relative_to(REPO)}")
print("done. originals untouched.")
