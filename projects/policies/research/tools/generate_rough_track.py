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

"""Generate the quad rough-terrain benchmark worlds (<robot>_rough_track.wbt).

Grafts a graduated difficult-terrain course onto each quadruped's flat
walk-deploy world. The robot spawns at x=0 and walks +x. Terrain is static
Box Solids (Newton-collidable when the deploy is run with
OMNISIM_NEWTON_STATICS=1) — the same representation the hill worlds use.

Course along +x:
  x 0..5    flat approach (gait ramps in, reaches speed)
  x 6..16   six full-width BUMP BARS of increasing height (3..18 cm)  -> clearance limit
  x 18..24  RUBBLE field (scattered small boxes)                      -> uneven footing
  x 24+     flat run-out

Run:  python projects/policies/research/tools/generate_rough_track.py
Drive: scripts/dev/run_quad_rough_track.ps1 -Robot {omniquad,go2,b2}
"""
import os
import random
from pathlib import Path

_REPO = next(p for p in Path(__file__).resolve().parents
             if (p / "AGENTS.md").exists() or (p / ".git").exists())
WDIR = _REPO / "projects" / "policies" / "research" / "worlds"
ROBOTS = ["omniquad", "go2", "b2"]

# (x_center, protruding height m) — brackets omniquad/go2/b2 foot clearance (~0.05-0.08)
BUMPS = [(6, 0.03), (8, 0.05), (10, 0.07), (12, 0.10), (14, 0.14), (16, 0.18)]
BUMP_THICK = 0.20      # x-extent (step-over bar)
LANE_W = 6.0           # full arena width (y)


def _solid(x, y, z, sx, sy, sz, name, rgb):
    r, g, b = rgb
    return (f'Solid {{ translation {x:.4f} {y:.4f} {z:.4f} '
            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} '
            f'roughness 0.95 metalness 0 }} geometry Box {{ size {sx:.4f} {sy:.4f} {sz:.4f} }} }} ] '
            f'name "{name}" boundingObject Box {{ size {sx:.4f} {sy:.4f} {sz:.4f} }} }}')


def build_terrain():
    out = ["# ===== DIFFICULT TRACK (static boxes; needs OMNISIM_NEWTON_STATICS=1) ====="]
    for i, (x, h) in enumerate(BUMPS):
        shade = max(0.2, 0.7 - h)                       # taller -> darker
        out.append(_solid(x, 0.0, h / 2.0, BUMP_THICK, LANE_W, h,
                          f"bump_{i}_{int(h*100)}cm",
                          (round(shade, 2), round(shade * 0.6, 2), 0.18)))
    rng = random.Random(42)                             # deterministic rubble
    n = 0
    for x in [18 + 0.5 * k for k in range(13)]:         # x 18..24
        for _ in range(4):
            rx = x + rng.uniform(-0.2, 0.2)
            ry = rng.uniform(-1.2, 1.2)
            h = rng.uniform(0.03, 0.06)
            sx = rng.uniform(0.12, 0.22)
            sy = rng.uniform(0.12, 0.22)
            out.append(_solid(rx, ry, h / 2.0, sx, sy, h, f"rock_{n}", (0.45, 0.43, 0.40)))
            n += 1
    out.append(f"# bumps={len(BUMPS)} rocks={n}")
    return "\n".join(out) + "\n"


def main():
    terrain = build_terrain()
    for r in ROBOTS:
        src = WDIR / f"{r}_walk_deploy.wbt"
        dst = WDIR / f"{r}_rough_track.wbt"
        s = src.read_text(encoding="utf-8")
        assert "URDFRobot {" in s, f"no URDFRobot in {src}"
        s = s.replace("URDFRobot {", terrain + "\nURDFRobot {", 1)
        dst.write_text(s, encoding="utf-8", newline="")
        print("wrote", dst.name)
    print("terrain:", terrain.count("Solid {"), "static boxes")


if __name__ == "__main__":
    main()
