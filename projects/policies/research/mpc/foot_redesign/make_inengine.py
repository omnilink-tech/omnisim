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

"""In-engine A/B assets: a BIGFOOT copy of the G1 deploy URDF + cube-throw world.

Writes two ADDITIVE, clearly-named copies (the originals are never modified):
  - projects/robots/unitree/g1/urdf/g1_23dof_omnisim_bigfoot.urdf   (foot box enlarged;
        lives in the urdf dir so the relative meshes/ paths still resolve)
  - projects/policies/worlds/g1_hstand_cubethrow_bigfoot.wbt        (same world, URDFRobot
        url repointed at the bigfoot urdf)

Foot box (URDF size = FULL extents): orig 0.17x0.06x0.012 @ x=0.035  ->
bigfoot 0.26x0.09x0.016 @ x=0.06 (= 2x the bigfoot MJCF half-extents 0.13/0.045/0.008 @ 0.06),
i.e. toe reach 0.035+0.085=0.120 m -> 0.06+0.13=0.190 m (matches H1), width 0.06 -> 0.09 m.
"""
from __future__ import annotations
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())

URDF = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf"
URDF_OUT = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_bigfoot.urdf"
WORLD = REPO / "projects/policies/worlds/g1_hstand_cubethrow.wbt"
WORLD_OUT = REPO / "projects/policies/worlds/g1_hstand_cubethrow_bigfoot.wbt"


def repl(txt, old, new, n):
    c = txt.count(old)
    assert c == n, f"expected {n}x {old!r}, found {c}"
    return txt.replace(old, new)


u = URDF.read_text(encoding="utf-8")
u = repl(u, '<box size="0.17 0.06 0.012"/>', '<box size="0.26 0.09 0.016"/>', 2)
u = repl(u, '<origin xyz="0.035 0 -0.030" rpy="0 0 0"/>', '<origin xyz="0.06 0 -0.030" rpy="0 0 0"/>', 2)
URDF_OUT.write_text(u, encoding="utf-8")
print(f"wrote {URDF_OUT.relative_to(REPO)}")

w = WORLD.read_text(encoding="utf-8")
w = repl(w, "g1_23dof_omnisim.urdf", "g1_23dof_omnisim_bigfoot.urdf", 1)
w = repl(w, 'title "Unitree G1 stand -- cubes thrown from the sides"',
         'title "Unitree G1 BIGFOOT stand -- cubes thrown (foot-redesign A/B)"', 1)
WORLD_OUT.write_text(w, encoding="utf-8")
print(f"wrote {WORLD_OUT.relative_to(REPO)}")
print("done. originals untouched.")
