#!/usr/bin/env python3
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

"""gen_blockworld -- a Minecraft-style block world for agents, as an .omniworld.

    python projects/samples/demos/worlds/environments/gen_blockworld.py            # rewrites blockworld.omniworld
    python projects/samples/demos/worlds/environments/gen_blockworld.py --seed 7 --size 24 --out my_blocks.omniworld

Same (seed, size, params) -> byte-identical output, like the omniworld generator.

WHAT IT MAKES. A `size` x `size` field of one-metre cubes: a rolling heightfield of
1..`max_height` blocks per column from seeded value noise, grass on top, dirt beneath,
stone below that, a few ore blocks in the stone, a handful of trees (log + leaf cubes),
and one dynamic probe ball resting on the surface. Every block is its own static Solid
named `b_<x>_<y>_<z>` (DEF `B_<x>_<y>_<z>`) with a Box boundingObject, so an agent can
address any block by grid coordinate over the harness: `GET /scene/node/B_3_4_2`,
`POST /scene/delete {"def": "B_3_4_2", "physics": "rebuild"}`,
`POST /scene/spawn {"vrml": "<a block>", "def": "B_3_4_3", "physics": "rebuild"}`.

WHY EVERY BLOCK CAN BE A SOLID. Since 2026-09-07 a plain static collider is not a MuJoCo
body of its own -- its shapes sit on Newton's world body -- so thousands of blocks load
and finalise in seconds (docs/developer/agents-hard-won-rules.md#world-body-statics).
Before that, 2000 statics overflowed MuJoCo's broadphase arena and the world had no
physics at all. Buried blocks are still colliders on purpose: a rebuild after an edit
re-registers the scene as it is, so a freshly exposed block is solid without any
bookkeeping on the agent's side.

The guide with the agent loop and measured costs: docs/guide/blockworld-agent-environment.md
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "blockworld.omniworld"

# Block palette: name -> (baseColor r g b). Flat colours, roughness 1, no metal --
# the point is legibility in a screenshot, not realism.
PALETTE = {
    "grass": (0.36, 0.62, 0.24),
    "dirt": (0.45, 0.31, 0.18),
    "stone": (0.50, 0.50, 0.52),
    "ore": (0.82, 0.68, 0.22),
    "log": (0.40, 0.26, 0.13),
    "leaves": (0.20, 0.48, 0.18),
    "sand": (0.83, 0.78, 0.55),
}


def value_noise(size: int, seed: int, cell: int = 6) -> list[list[float]]:
    """Seeded bilinear value noise in [0, 1], `cell` blocks per lattice step."""
    rng = random.Random(seed)
    lattice = size // cell + 2
    grid = [[rng.random() for _ in range(lattice)] for _ in range(lattice)]

    def smooth(t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    out = []
    for x in range(size):
        row = []
        for y in range(size):
            gx, gy = x / cell, y / cell
            ix, iy = int(gx), int(gy)
            fx, fy = smooth(gx - ix), smooth(gy - iy)
            a = grid[ix][iy] * (1 - fx) + grid[ix + 1][iy] * fx
            b = grid[ix][iy + 1] * (1 - fx) + grid[ix + 1][iy + 1] * fx
            row.append(a * (1 - fy) + b * fy)
        out.append(row)
    return out


def look_at_axis_angle(pos, target, up=(0.0, 0.0, 1.0)):
    """Viewpoint rotation (axis-angle) for a camera at `pos` looking at `target`.

    The wgpu main view's camera looks along its local +X with +Z up (the
    Z-up convention every flagship world was re-authored to in 2026-08;
    docs/developer/agents-hard-won-rules.md). Columns of the rotation are
    (forward, left, up)."""
    fx, fy, fz = (target[i] - pos[i] for i in range(3))
    n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    f = (fx / n, fy / n, fz / n)
    # left = up x forward
    l = (up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0])
    ln = math.sqrt(sum(c * c for c in l)) or 1.0
    l = tuple(c / ln for c in l)
    # true up = forward x left
    u = (f[1] * l[2] - f[2] * l[1], f[2] * l[0] - f[0] * l[2], f[0] * l[1] - f[1] * l[0])
    # rotation matrix with columns f, l, u
    m = [[f[0], l[0], u[0]], [f[1], l[1], u[1]], [f[2], l[2], u[2]]]
    trace = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    s = 2.0 * math.sin(angle)
    if abs(s) < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    axis = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    return (axis[0], axis[1], axis[2], angle)


def block_line(x: int, y: int, z: int, kind: str) -> str:
    r, g, b = PALETTE[kind]
    # Block (x, y, z) occupies [x, x+1) x [y, y+1) x [z, z+1): centre at +0.5.
    return (
        'DEF B_%d_%d_%d Solid { name "b_%d_%d_%d" translation %.1f %.1f %.1f '
        "children [ Shape { appearance PBRAppearance { baseColor %.2f %.2f %.2f roughness 1 metalness 0 } "
        "geometry Box { size 1 1 1 } } ] boundingObject Box { size 1 1 1 } }"
        % (x, y, z, x, y, z, x + 0.5, y + 0.5, z + 0.5, r, g, b)
    )


def generate(seed: int, size: int, max_height: int, trees: int, ore_fraction: float) -> tuple[str, dict]:
    rng = random.Random(seed * 7919 + size)
    noise = value_noise(size, seed)
    heights = [[1 + int(round(noise[x][y] * (max_height - 1))) for y in range(size)] for x in range(size)]

    lines = []
    counts = {k: 0 for k in PALETTE}
    for x in range(size):
        for y in range(size):
            h = heights[x][y]
            for z in range(h):
                depth = h - 1 - z
                if depth == 0:
                    kind = "sand" if h == 1 and rng.random() < 0.5 else "grass"
                elif depth <= 2:
                    kind = "dirt"
                else:
                    kind = "ore" if rng.random() < ore_fraction else "stone"
                counts[kind] += 1
                lines.append(block_line(x, y, z, kind))

    # Trees: a 4-log trunk on a grass column, a 3x3x2 leaf crown plus a cap.
    placed = 0
    attempts = 0
    tree_cells = set()
    while placed < trees and attempts < 200:
        attempts += 1
        x, y = rng.randrange(2, size - 2), rng.randrange(2, size - 2)
        if any(abs(x - tx) < 4 and abs(y - ty) < 4 for tx, ty in tree_cells):
            continue
        base = heights[x][y]
        for z in range(base, base + 4):
            lines.append(block_line(x, y, z, "log"))
            counts["log"] += 1
        top = base + 4
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    lines.append(block_line(x + dx, y + dy, top - 1 + dz, "leaves"))
                    counts["leaves"] += 1
        lines.append(block_line(x, y, top + 1, "leaves"))
        counts["leaves"] += 1
        tree_cells.add((x, y))
        placed += 1

    # The probe ball: a dynamic body dropped onto the middle column.
    cx, cy = size // 2, size // 2
    ball_z = heights[cx][cy] + 2.0
    probe = (
        'DEF PROBE Solid { name "probe" translation %.1f %.1f %.1f '
        "children [ Shape { appearance PBRAppearance { baseColor 0.85 0.15 0.12 roughness 0.5 metalness 0 } "
        "geometry Sphere { radius 0.3 } } ] boundingObject Sphere { radius 0.3 } physics Physics { density -1 mass 1 } }"
        % (cx + 0.5, cy + 0.5, ball_z)
    )

    cam_pos = (-0.55 * size, -0.55 * size, 0.9 * size)
    cam_target = (size / 2.0, size / 2.0, 2.0)
    ax, ay, az, ang = look_at_axis_angle(cam_pos, cam_target)

    total = sum(counts.values())
    header = [
        "#OMNISIM R2025a utf8",
        "",
        "# blockworld -- a Minecraft-style block world for agents.",
        "# GENERATED by gen_blockworld.py (seed %d, size %d, max_height %d, trees %d); edit the",
        "# generator, not this file. %d one-metre blocks, each a static Solid named b_<x>_<y>_<z>",
        "# (DEF B_<x>_<y>_<z>) with a Box boundingObject, plus one dynamic probe ball. Every block",
        "# is addressable over the harness by grid coordinate; place and break with /scene/spawn",
        "# and /scene/delete + {\"physics\": \"rebuild\"}. Guide: docs/guide/blockworld-agent-environment.md",
        "",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        "",
        "WorldInfo {",
        '  title "blockworld"',
        "  basicTimeStep 16",
        "}",
        "Viewpoint {",
        "  position %.2f %.2f %.2f" % cam_pos,
        "  orientation %.5f %.5f %.5f %.5f" % (ax, ay, az, ang),
        "}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF GROUND Solid { name "ground" translation %.1f %.1f -0.05 children [ Shape { appearance PBRAppearance { baseColor 0.30 0.30 0.32 roughness 1 metalness 0 } geometry Box { size %d %d 0.1 } } ] boundingObject Box { size %d %d 0.1 } }'
        % (size / 2.0, size / 2.0, size + 8, size + 8, size + 8, size + 8),
    ]
    header[3] = header[3] % (seed, size, max_height, trees)
    header[4] = header[4] % total
    # The probe goes BEFORE the blocks: run-headless --fail-on-runaway enumerates at
    # most 256 root children when it picks the dynamic bodies to track, and a ball
    # hidden behind 2,600 blocks would leave it with no evidence.
    text = "\n".join(header + [probe] + lines) + "\n"
    stats = {"blocks": total, **counts, "trees": placed, "probe_drop_z": ball_z,
             "size": size, "max_height": max_height, "seed": seed}
    return text, stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--size", type=int, default=32, help="blocks per side (default 32)")
    ap.add_argument("--max-height", type=int, default=4, help="tallest column in blocks (default 4)")
    ap.add_argument("--trees", type=int, default=6)
    ap.add_argument("--ore-fraction", type=float, default=0.08)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args(argv)
    text, stats = generate(args.seed, args.size, args.max_height, args.trees, args.ore_fraction)
    args.out.write_text(text, encoding="utf-8", newline="\n")
    print("wrote %s: %s" % (args.out, ", ".join("%s=%s" % kv for kv in stats.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
