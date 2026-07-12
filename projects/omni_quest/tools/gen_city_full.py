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

"""Compose the FULL-CITY navigator: the Husky navigates the whole road grid.

Where gen_city_nav.py decks one street and hand-codes a 5-node graph, this derives
the ENTIRE walkable network from the grid spec (city_grid.json) via
citygraph.build_city_graph: four sidewalk-corner nodes per intersection, walk edges
over every block, cross edges at every intersection. A mesh of invisible collision
decks (one strip per sidewalk centreline, both axes) gives the Husky a continuous
surface to drive the whole grid, and the A* planner routes any corner to any corner,
rerouting around blockages. This is the general outdoor-navigation stack at city
scale, not a single hand-tuned corridor.

Writes worlds/city_husky_full.wbt + controllers/omni_quest_nav/course_full.py.
NOTE: run tools/setup_city_nav.sh once so the city_traffic controller resolves.

    python projects/omni_quest/tools/gen_city_full.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJ / "controllers" / "omni_quest_nav"))
import citygraph  # noqa: E402

_OMNILINK_LICENCE_HEADER = """\
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
"""

CITY_SRC = (PROJ.parents[0] / "samples" / "demos" / "worlds" / "showcase"
            / "city_traffic.wbt")
WORLD_OUT = PROJ / "worlds" / "city_husky_full.wbt"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_full.py"
GRID_JSON = PROJ / "controllers" / "city_traffic" / "city_grid.json"

REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0
OFF = 6.5            # sidewalk centreline offset from a road centreline (matches decks)
DECK_W = 4.5         # deck strip width (m)
SPAWN_Z = 0.35       # on top of the 0.15 m deck
START_NODE = "i1j1WS"   # SW corner of the central-SW intersection
DEST_NODE = "i2j2EN"    # NE corner of the central-NE intersection (cross-city diagonal)


def deck_mesh(xs, ys):
    """One invisible collision strip per sidewalk centreline on each axis. The strips
    span the whole grid and overlap at corners, so the mesh is one continuous walkable
    surface covering every sidewalk AND every crossing (the strips bridge the roads)."""
    lo = min(min(xs), min(ys)) - OFF - 2.0
    hi = max(max(xs), max(ys)) + OFF + 2.0
    span, ctr = hi - lo, (lo + hi) / 2.0
    sw_y = sorted({round(y + s * OFF, 3) for y in ys for s in (-1, 1)})
    sw_x = sorted({round(x + s * OFF, 3) for x in xs for s in (-1, 1)})
    lines = []
    for k, y in enumerate(sw_y):
        lines.append(f'  Solid {{ name "deckh{k}" translation {ctr} {y} 0.075 '
                     f'boundingObject Box {{ size {span:.1f} {DECK_W} 0.15 }} }}')
    for k, x in enumerate(sw_x):
        lines.append(f'  Solid {{ name "deckv{k}" translation {x} {ctr} 0.075 '
                     f'boundingObject Box {{ size {DECK_W} {span:.1f} 0.15 }} }}')
    return "\n".join(lines)


def husky_block(nodes, decks):
    sx, sy = nodes[START_NODE]
    return f"""
# ============================================================================
# OmniQuest FULL-CITY navigator (tools/gen_city_full.py). The Husky walks the whole
# sidewalk grid on the collision-deck mesh below, routing any corner to any corner
# with the A* planner and waiting for traffic at each crossing. Camera + GPS only.
# ============================================================================
{decks}
DEF CITY_HUSKY URDFRobot {{
  url "../../robots/clearpath/husky_description/urdf/husky.urdf"
  translation {sx} {sy} {SPAWN_Z}
  rotation 0 0 1 0
  name "husky"
  supervisor TRUE
  controller "omni_quest_nav"
  controllerArgs ["--course" "full" "--vmax" "0.7"]
}}
DEF CITY_HUSKY_EYE Robot {{
  translation {sx} {sy} {SPAWN_Z}
  rotation 0 0 1 0
  name "husky_cam"
  supervisor TRUE
  controller "omni_quest_eye"
  controllerArgs ["--target" "CITY_HUSKY" "--road"]
  physics NULL
  children [
    SpotLight {{ location 0.15 0 0.55 direction 1 0 -0.4 color 1 1 0.96 intensity 1.8 cutOffAngle 1.15 beamWidth 0.9 radius 20 castShadows FALSE }}
    Camera {{ name "stereo_left"  translation 0.30  0.05 0.32 rotation 0 1 0 0.30 width 256 height 192 fieldOfView 1.6 }}
    Camera {{ name "stereo_right" translation 0.30 -0.05 0.32 rotation 0 1 0 0.30 width 256 height 192 fieldOfView 1.6 }}
    Pose {{ translation 0 0 7.5 children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.12 0.70 emissiveColor 0.90 0.05 0.50 metalness 0 roughness 1 }} geometry Cylinder {{ height 13 radius 0.09 }} }} ] }}
    Pose {{ translation 0 0 14.2 children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.12 0.70 emissiveColor 1 0.10 0.60 metalness 0 roughness 1 }} geometry Sphere {{ radius 0.55 }} }} ] }}
  ]
}}
"""


def write_course(nodes, edges):
    sx, sy = nodes[START_NODE]
    dx, dy = nodes[DEST_NODE]
    L = [_OMNILINK_LICENCE_HEADER,
         "# Generated by tools/gen_city_full.py - do not hand-edit.",
         '"""Full-city course: the whole sidewalk graph (any corner to any corner)."""',
         "", f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}",
         f"START = ({sx}, {sy})", "",
         f"ROUTE_ENU = [({dx}, {dy}, 'G')]   # route-mode uses the graph; this is a stub",
         "OBSTACLES = []", "CROSSINGS = []", "",
         f"GRAPH_NODES = {nodes!r}",
         f"GRAPH_EDGES = {edges!r}",
         f"START_NODE = {START_NODE!r}", f"DEST_NODE = {DEST_NODE!r}"]
    COURSE_OUT.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    if not CITY_SRC.exists():
        raise SystemExit(f"city source not found: {CITY_SRC}")
    grid = json.loads(GRID_JSON.read_text(encoding="utf-8"))
    xs, ys = grid["xs"], grid["ys"]
    nodes, edges = citygraph.build_city_graph(xs, ys, OFF)
    g = citygraph.CityGraph(nodes, edges)
    route = g.plan(START_NODE, DEST_NODE)
    if not route:
        raise SystemExit(f"no route {START_NODE}->{DEST_NODE}")
    city = CITY_SRC.read_text(encoding="utf-8", errors="replace")
    WORLD_OUT.write_text(city.rstrip() + "\n" + husky_block(nodes, deck_mesh(xs, ys)),
                         encoding="utf-8")
    write_course(nodes, edges)
    print(f"[gen] {WORLD_OUT.relative_to(PROJ)}: full city, {len(nodes)} nodes, "
          f"{len(edges)} edges, {2 * (len(xs) + len(ys))} deck strips")
    print(f"[gen] route {START_NODE}->{DEST_NODE}: {len(route)} legs, "
          f"{g.route_length(route):.0f} m; wrote {COURSE_OUT.name}")
    print("[gen] NOTE: tools/setup_city_nav.sh must have copied the city_traffic ctrl")


if __name__ == "__main__":
    main()
