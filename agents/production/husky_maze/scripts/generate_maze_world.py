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



"""Generate a husky_maze-style .wbt with a different seed.



Usage:

    python agents/production/husky_maze/scripts/generate_maze_world.py \

        --seed 19 \

        --title "Husky Maze (Unknown)" \

        --out projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt



The output file is byte-for-byte the same shape as

projects/samples/demos/worlds/flagship/husky_maze.wbt — perimeter walls, internal

H_*/V_* walls in the same coordinate convention, the goal marker, and

the URDFRobot husky wired to the husky_omnilink_bridge controller. Only

the maze layout (and the title, if provided) differ.



The bridge will refuse to expose the maze graph for any world whose

title contains "unknown" (case-insensitive), so passing a title with

"Unknown" in it is what makes the bridge serve `try_get_known_map ->

{available: false}`. That, in turn, forces the agent to fall back to

lidar — which is the entire point of the demo.

"""



from __future__ import annotations



import argparse

import random

from pathlib import Path

from typing import List, Tuple



CELLS = 11

CELL_SIZE = 2.0

ORIGIN_X = -10.0

ORIGIN_Y = -10.0





def generate_perfect_maze(seed: int) -> Tuple[List[Tuple[int, int, int]], List[Tuple[int, int, int]]]:

    """Recursive-backtracker over an 11x11 grid. Returns (h_walls, v_walls)

    where h_walls[i] = (col, row_lo, row_hi) means the wall blocks cell

    (col, row_lo) <-> (col, row_lo+1), and v_walls[i] = (col_lo, col_hi, row)

    means the wall blocks (col_lo, row) <-> (col_lo+1, row).



    Implementation: start with all internal walls present; carve passages

    in DFS order so exactly one path exists between any two cells."""

    rng = random.Random(seed)

    visited = [[False] * CELLS for _ in range(CELLS)]

    # Carved-passages set: (cell_a, cell_b) sorted, both in cell coords.

    carved = set()



    def neighbours(c, r):

        out = []

        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):

            nc, nr = c + dc, r + dr

            if 0 <= nc < CELLS and 0 <= nr < CELLS:

                out.append((nc, nr))

        return out



    def dfs(c, r):

        visited[c][r] = True

        nbs = neighbours(c, r)

        rng.shuffle(nbs)

        for nc, nr in nbs:

            if visited[nc][nr]:

                continue

            a, b = sorted([(c, r), (nc, nr)])

            carved.add((a, b))

            dfs(nc, nr)



    # Start somewhere that gives a nice trace; corner is fine.

    dfs(0, 0)



    # Now enumerate every internal edge. If it's NOT carved, it's a wall.

    h_walls: List[Tuple[int, int, int]] = []

    v_walls: List[Tuple[int, int, int]] = []

    for col in range(CELLS):

        for row in range(CELLS - 1):

            edge = tuple(sorted([(col, row), (col, row + 1)]))

            if edge not in carved:

                h_walls.append((col, row, row + 1))

    for col in range(CELLS - 1):

        for row in range(CELLS):

            edge = tuple(sorted([(col, row), (col + 1, row)]))

            if edge not in carved:

                v_walls.append((col, col + 1, row))

    return h_walls, v_walls





def render_world(

    seed: int,

    title: str,

    h_walls,

    v_walls,

    info_lines: List[str],

    markers: List[dict] = None,

    with_camera: bool = False,

    topdown_viewport: bool = False,

) -> str:

    """Render the .wbt text. Mirrors husky_maze.wbt header and order so a

    diff between the two worlds tells you exactly which walls changed.



    `info_lines` go into WorldInfo.info as the mission brief — the agent

    reads them via the bridge's /mission endpoint."""

    lines: List[str] = []

    lines.append("#VRML_SIM R2025a utf8")

    lines.append("")

    lines.append(f"# Husky maze - {title}")

    lines.append(f"# Recursive-backtracker maze, seed={seed}. Same coordinate")

    lines.append("# convention as husky_maze.wbt: 11x11 cells, 2 m each,")

    lines.append("# cell (col, row) center at (-10 + 2*col, -10 + 2*row).")

    lines.append("# Start cell (0, 10) at (-10, +10).")

    lines.append("")

    lines.append('EXTERNPROTO "omnisim://projects/objects/floors/protos/Floor.proto"')

    lines.append('EXTERNPROTO "omnisim://projects/objects/apartment_structure/protos/Wall.proto"')

    lines.append('EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/NightSky.proto"')

    lines.append('EXTERNPROTO "omnisim://projects/appearances/protos/Roughcast.proto"')

    lines.append('EXTERNPROTO "omnisim://projects/appearances/protos/Asphalt.proto"')

    lines.append("")

    lines.append("WorldInfo {")

    lines.append("  basicTimeStep 16")

    lines.append(f'  title "{title}"')

    if info_lines:

        # MFString — quote each line. Escape internal quotes and backslashes.

        info_quoted = []

        for line in info_lines:

            esc = line.replace("\\", "\\\\").replace('"', '\\"')

            info_quoted.append(f'    "{esc}"')

        lines.append("  info [")

        lines.extend(info_quoted)

        lines.append("  ]")

    lines.append("}")

    if topdown_viewport:

        # Strict top-down view — exactly what OmniSim's "View > Top" (key 7)

        # sets via WbViewpoint::topView() in src/omnisim/nodes/WbViewpoint.cpp.

        # The orientation is a 120° rotation about the (-1, 1, 1) diagonal

        # axis: it leaves +Z aligned with world +Z and points the camera's

        # forward axis straight down at the world origin. z=40 frames the

        # full 22×22 m maze with comfortable margin at the default 0.785

        # rad FoV (visible area at z=0 ≈ 33 m vs the 22 m maze).

        lines.append("Viewpoint {")

        lines.append("  orientation -0.5773 0.5773 0.5773 2.0944")

        lines.append("  position 0 0 40")

        lines.append("}")

    else:

        lines.append("Viewpoint {")

        lines.append("  orientation -0.30 -0.30 0.91 1.75")

        lines.append("  position -4 -36 34")

        lines.append("}")

    lines.append("NightSky {")

    lines.append("  luminosity 1.2")

    lines.append("}")

    lines.append("DirectionalLight {")

    lines.append("  direction 0.4 -0.3 -1")

    lines.append("  color 0.95 0.96 1.0")

    lines.append("  intensity 3.0")

    lines.append("  ambientIntensity 0.55")

    lines.append("  castShadows TRUE")

    lines.append("}")

    lines.append("DirectionalLight {")

    lines.append("  direction -0.4 0.3 -0.6")

    lines.append("  color 0.85 0.88 1.0")

    lines.append("  intensity 1.4")

    lines.append("  ambientIntensity 0.0")

    lines.append("  castShadows FALSE")

    lines.append("}")

    lines.append("")

    lines.append("Floor {")

    lines.append("  size 26 26")

    lines.append("  tileSize 26 26")

    lines.append("  appearance Asphalt {")

    lines.append("    colorOverride 0.55 0.57 0.60")

    lines.append("  }")

    lines.append("}")

    lines.append("")

    lines.append("# Outer perimeter walls (22 m x 22 m enclosure, centred on origin).")

    lines.append('Wall { name "wall_perim_N" translation 0 11 0 size 22.2 0.2 1 }')

    lines.append('Wall { name "wall_perim_S" translation 0 -11 0 size 22.2 0.2 1 }')

    lines.append('Wall { name "wall_perim_E" translation 11 0 0 size 0.2 22.2 1 }')

    lines.append('Wall { name "wall_perim_W" translation -11 0 0 size 0.2 22.2 1 }')

    lines.append("")

    lines.append("# Internal maze walls.")

    for i, (col, r_lo, r_hi) in enumerate(h_walls):

        cx = ORIGIN_X + col * CELL_SIZE

        cy = ORIGIN_Y + r_lo * CELL_SIZE + 1.0

        lines.append(

            f'Wall {{ name "H_{i:03d}" translation {cx:>3.0f} {cy:>3.0f} 0 size 2 0.2 1 }}'

        )

    lines.append("")

    for i, (c_lo, c_hi, row) in enumerate(v_walls):

        cx = ORIGIN_X + c_lo * CELL_SIZE + 1.0

        cy = ORIGIN_Y + row * CELL_SIZE

        lines.append(

            f'Wall {{ name "V_{i:03d}" translation {cx:>3.0f} {cy:>3.0f} 0 size 0.2 2 1 }}'

        )

    lines.append("")

    lines.append("# Goal marker - red post + yellow finial at the SE goal cell.")

    lines.append("Solid {")

    lines.append("  translation 10 -10 0.25")

    lines.append('  name "goal_marker"')

    lines.append("  children [")

    lines.append("    Shape {")

    lines.append("      appearance PBRAppearance {")

    lines.append("        baseColor 0.95 0.1 0.1")

    lines.append("        roughness 0.4")

    lines.append("        metalness 0.0")

    lines.append("        emissiveColor 0.4 0.0 0.0")

    lines.append("      }")

    lines.append("      geometry Cylinder {")

    lines.append("        height 0.5")

    lines.append("        radius 0.15")

    lines.append("      }")

    lines.append("    }")

    lines.append("    Pose {")

    lines.append("      translation 0 0 0.42")

    lines.append("      children [")

    lines.append("        Shape {")

    lines.append("          appearance PBRAppearance {")

    lines.append("            baseColor 1.0 0.85 0.0")

    lines.append("            roughness 0.4")

    lines.append("            metalness 0.0")

    lines.append("            emissiveColor 0.5 0.4 0.0")

    lines.append("          }")

    lines.append("          geometry Sphere {")

    lines.append("            radius 0.22")

    lines.append("            subdivision 2")

    lines.append("          }")

    lines.append("        }")

    lines.append("      ]")

    lines.append("    }")

    lines.append("  ]")

    lines.append("}")

    lines.append("")



    # Optional extra visual markers (used by visual / vision-only worlds).

    # Each entry: {col, row, name, color: (r,g,b), height_m?}.

    for m in (markers or []):

        cx = ORIGIN_X + m["col"] * CELL_SIZE

        cy = ORIGIN_Y + m["row"] * CELL_SIZE

        r, g, b = m["color"]

        h = m.get("height_m", 0.6)

        lines.append(f"# Visual marker {m['name']!r} at cell ({m['col']},{m['row']}).")

        lines.append("Solid {")

        lines.append(f"  translation {cx:.1f} {cy:.1f} {h/2:.2f}")

        lines.append(f'  name "{m["name"]}"')

        lines.append("  children [")

        lines.append("    Shape {")

        lines.append("      appearance PBRAppearance {")

        lines.append(f"        baseColor {r:.2f} {g:.2f} {b:.2f}")

        lines.append("        roughness 0.4")

        lines.append("        metalness 0.0")

        lines.append(f"        emissiveColor {r*0.4:.2f} {g*0.4:.2f} {b*0.4:.2f}")

        lines.append("      }")

        lines.append("      geometry Cylinder {")

        lines.append(f"        height {h:.2f}")

        lines.append("        radius 0.18")

        lines.append("      }")

        lines.append("    }")

        lines.append("  ]")

        lines.append("}")

        lines.append("")



    lines.append("# Husky robot, spawned at the NW start cell facing east.")

    lines.append("DEF HUSKY URDFRobot {")

    lines.append('  url "../../../../projects/robots/clearpath/husky_description/urdf/husky.urdf"')

    lines.append("  translation -10 10 0.1")

    lines.append('  name "husky"')

    lines.append("  supervisor TRUE")

    lines.append('  controller "husky_omnilink_bridge"')

    lines.append("}")

    lines.append("")



    if with_camera:

        # Vision sidecar: a separate Robot that owns a real OmniSim Camera.

        # Its controller (`husky_eye`) tracks the husky's base_link pose

        # each tick and exposes /image on port 6071. The main bridge

        # proxies its /camera endpoint to this. See controllers/husky_eye/.

        lines.append("# Vision sidecar — see controllers/husky_eye/.")

        lines.append("Robot {")

        lines.append('  name "husky_eye"')

        lines.append("  translation -10 10 1")

        lines.append("  supervisor TRUE")

        lines.append('  controller "husky_eye"')

        lines.append("  children [")

        lines.append("    Camera {")

        lines.append('      name "front_camera"')

        lines.append("      width 320")

        lines.append("      height 240")

        lines.append("      fieldOfView 1.4")

        # Tilt the camera down a bit so the agent sees the ground markers.

        lines.append("      rotation 0 1 0 0.35")

        lines.append("    }")

        lines.append("    Shape {")

        lines.append("      appearance PBRAppearance {")

        lines.append("        baseColor 0.2 0.6 1.0")

        lines.append("        emissiveColor 0.05 0.15 0.3")

        lines.append("        roughness 0.4")

        lines.append("      }")

        lines.append("      geometry Sphere { radius 0.05 }")

        lines.append("    }")

        lines.append("  ]")

    # NOTE: We DO NOT attach a Camera as a URDFRobot child. URDFRobot

    # treats an explicit `children` field as a replacement for its URDF

    # expansion, so adding a Camera here would strip out the wheels and

    # the husky would have no physics. Vision is provided via the bridge's

    # /camera endpoint backed by Supervisor.exportImage of the OmniSim

    # viewport instead — see husky_omnilink_bridge.py.

    _ = with_camera  # accepted for backward compat, ignored.

    lines.append("}")

    return "\n".join(lines) + "\n"





def main() -> None:

    p = argparse.ArgumentParser()

    p.add_argument("--seed", type=int, default=19)

    p.add_argument("--title", type=str, default="Husky Maze (Unknown)")

    p.add_argument("--out", type=Path, required=True)

    p.add_argument(

        "--info",

        type=str,

        action="append",

        default=[],

        help="Mission brief line; pass multiple --info for multi-line. "

             "Read by the bridge from WorldInfo.info and exposed at /mission.",

    )

    p.add_argument(

        "--marker",

        type=str,

        action="append",

        default=[],

        help="Add a coloured cylinder marker. Format: "

             "'name:col,row:r,g,b'. Example: 'red_cylinder:5,3:0.95,0.1,0.1'.",

    )

    p.add_argument(

        "--with-camera",

        action="store_true",

        help="Accepted for backward compat; the bridge's vision uses "

             "Supervisor.exportImage instead of a URDFRobot Camera. "

             "Pair with --topdown-viewport to give the agent a useful view.",

    )

    p.add_argument(

        "--topdown-viewport",

        action="store_true",

        help="Set Viewpoint to a straight-down overhead view so exportImage "

             "captures the maze + colour markers from above. Use for "

             "vision-only worlds.",

    )

    args = p.parse_args()



    markers = []

    for spec in args.marker:

        try:

            name_part, cell_part, color_part = spec.split(":")

            col, row = (int(x) for x in cell_part.split(","))

            r, g, b = (float(x) for x in color_part.split(","))

            markers.append({"name": name_part, "col": col, "row": row, "color": (r, g, b)})

        except Exception as exc:

            raise SystemExit(f"bad --marker {spec!r}: {exc}") from exc



    h_walls, v_walls = generate_perfect_maze(args.seed)

    text = render_world(

        args.seed, args.title, h_walls, v_walls, args.info,

        markers=markers, with_camera=args.with_camera,

        topdown_viewport=args.topdown_viewport,

    )

    args.out.parent.mkdir(parents=True, exist_ok=True)

    args.out.write_text(text, encoding="utf-8")

    print(f"wrote {args.out}: {len(h_walls)} H + {len(v_walls)} V walls, "

          f"{len(markers)} marker(s), camera={args.with_camera} "

          f"(seed={args.seed}, title={args.title!r}, "

          f"info_lines={len(args.info)})")





if __name__ == "__main__":

    main()

