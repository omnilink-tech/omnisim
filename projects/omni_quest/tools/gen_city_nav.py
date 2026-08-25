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

"""Compose the city-with-traffic world + an OmniQuest PEDESTRIAN navigator.

Reads the city_traffic showcase (4x4 road grid + 48 cars driven by the
city_traffic supervisor) and appends a Husky that walks the SIDEWALKS like a
pedestrian: it rides on top of the raised pavement (a collision deck we add,
since the city sidewalks have no collision), stays off the road / away from cars,
and waits for a gap in traffic before crossing a street.

Sidewalk geometry (from city_grid.json + the Road proto): road centrelines at
x,y in {-93,-31,31,93}, road width 10 m, raised sidewalks 3 m wide at 0.15 m,
centred at road-edge+3 (e.g. the north sidewalk of the y=-31 road is at y=-23).

Writes worlds/city_husky_nav.wbt + controllers/omni_quest_nav/course_city.py.
NOTE: run tools/setup_city_nav.sh once so the city_traffic controller resolves.

    python projects/omni_quest/tools/gen_city_nav.py
"""

from __future__ import annotations

from pathlib import Path

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

PROJ = Path(__file__).resolve().parents[1]
CITY_SRC = (PROJ.parents[0] / "samples" / "demos" / "worlds" / "showcase"
            / "city_traffic.omniworld")
WORLD_OUT = PROJ / "worlds" / "city_husky_nav.wbt"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_city.py"

REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0

# Pedestrian route: walk EAST along the north sidewalk of the y=-31 street
# (sidewalk centreline y=-23), crossing the x=31 road at the crossing (31,-23)
# only when traffic is clear. The whole route is a straight line at y=-23, so a
# single raised collision deck carries it (sidewalks + the crossing).
# Sidewalk spans road-edge(-26) to building-edge(-23); its CENTRE is y=-24.5
# (1.5 m clear of the road to the south and the buildings to the north).
SIDEWALK_Y = -24.5
START = (5.0, SIDEWALK_Y)
ROUTE = [
    (14.0, SIDEWALK_Y, "sidewalk"),
    (22.0, SIDEWALK_Y, "approach"),   # just west of the kerb hold-zone (x 23.5..26)
    (33.0, SIDEWALK_Y, "crossed"),    # far side of the x=31 road
    (40.0, SIDEWALK_Y, "east_end"),
]
# (cross_y, road_x, road_half, stop_lo, stop_hi, danger_y): HOLD at the kerb
# (robot x in [stop_lo, stop_hi], just west of the x=31 road edge at x=26) until
# no car is on the x=31 road (|x-31|<road_half) near the crossing line (|y-cross_y|
# <danger_y), then cross.
CROSSINGS = [(SIDEWALK_Y, 31.0, 6.0, 23.5, 26.0, 6.0)]
SPAWN_Z = 0.35   # on top of the 0.15 m deck (settles to ~0.30)

# GLOBAL ROUTE GRAPH for deploy-with-reroute: nodes are sidewalk corners of the y=-31
# street; "walk" edges run a sidewalk (axis/lane), "cross" edges step across the road
# at an intersection. The Husky spawns on AN->BN (heads east toward BN). If the north
# walk AN-BN is blocked (bus stop) the global planner reroutes AN->AS->BS->BN->G.
GRAPH_NODES = {
    "AN": (-31.0, -24.5), "BN": (31.0, -24.5),    # north sidewalk corners
    "AS": (-31.0, -37.5), "BS": (31.0, -37.5),    # south sidewalk corners
    "G":  (40.0, -24.5),                          # destination, east of the x=31 cross
}
GRAPH_EDGES = [
    ("AN", "BN", "walk",  {"axis": "x", "lane": -24.5}),  # north walk (has the bus stop)
    ("AS", "BS", "walk",  {"axis": "x", "lane": -37.5}),  # south walk (clear detour)
    ("AN", "AS", "cross", {"axis": "y", "lane": -31.0, "road": -31.0}),
    ("BN", "BS", "cross", {"axis": "y", "lane": 31.0,  "road": -31.0}),
    ("BN", "G",  "walk",  {"axis": "x", "lane": -24.5}),  # destination leg
]
START_NODE = "AN"   # the Husky spawns on the AN->BN leg, heading east toward BN
DEST_NODE = "G"

HUSKY = f"""
# ============================================================================
# OmniQuest PEDESTRIAN navigator added to the city (tools/gen_city_nav.py).
# Walks the sidewalk (y=-23) on top of the collision deck below; --road camera so
# grey pavement reads as drivable; waits for a gap before crossing the x=31 road.
# ============================================================================
# Invisible collision decks (0.15 m) giving the Husky a surface ON TOP of the raised
# city pavement (the visible sidewalks are Road borders with no collision). The global
# planner can DETOUR onto the south sidewalk when the north one is blocked, so we deck
# BOTH sidewalks of the y=-31 street plus the two crossings — all at the same 0.15 m
# height so the detour is one continuous, kerb-free surface the robot can drive.
DEF DECK_N Solid {{ translation 16 -24.25 0.075 boundingObject Box {{ size 102 4.5 0.15 }} }}
DEF DECK_S Solid {{ translation 0 -37.5 0.075 boundingObject Box {{ size 72 4.5 0.15 }} }}
DEF DECK_XW Solid {{ translation -31 -31 0.075 boundingObject Box {{ size 5 16 0.15 }} }}
DEF DECK_XE Solid {{ translation 31 -31 0.075 boundingObject Box {{ size 5 16 0.15 }} }}
DEF CITY_HUSKY URDFRobot {{
  url "../../robots/clearpath/husky_description/urdf/husky.urdf"
  translation {START[0]} {START[1]} {SPAWN_Z}
  rotation 0 0 1 0
  name "husky"
  supervisor TRUE
  controller "omni_quest_nav"
  controllerArgs ["--course" "city" "--vmax" "0.7"]
}}
DEF CITY_HUSKY_EYE Robot {{
  translation {START[0]} {START[1]} {SPAWN_Z}
  rotation 0 0 1 0
  name "husky_cam"
  supervisor TRUE
  controller "omni_quest_eye"
  controllerArgs ["--target" "CITY_HUSKY" "--road"]
  physics NULL
  children [
    # Headlight — the city sidewalk is shadowed by the buildings, so a forward-down
    # spotlight lights the pavement ahead and lets the camera segment free space
    # reliably (a real robot's headlamp; still camera-only perception).
    SpotLight {{ location 0.15 0 0.55 direction 1 0 -0.4 color 1 1 0.96 intensity 1.8 cutOffAngle 1.15 beamWidth 0.9 radius 20 castShadows FALSE }}
    # STEREO camera pair (0.10 m baseline, pitched down): the eye block-matches the
    # two views into a depth map and a per-column nearest-obstacle DEPTH profile. Depth
    # separates flat drivable ground from things that stand up (bus shelters, buildings,
    # pedestrians) regardless of colour — solving the grey-pavement-vs-grey-wall problem
    # a single colour camera cannot. Camera + GPS only, no lidar.
    Camera {{ name "stereo_left"  translation 0.30  0.05 0.32 rotation 0 1 0 0.30 width 256 height 192 fieldOfView 1.6 }}
    Camera {{ name "stereo_right" translation 0.30 -0.05 0.32 rotation 0 1 0 0.30 width 256 height 192 fieldOfView 1.6 }}
    # Locator beacon — the sidecar tracks the robot, so this glowing pole + ball
    # floats above the Husky (find it from across the city).
    Pose {{ translation 0 0 7.5 children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.12 0.70 emissiveColor 0.90 0.05 0.50 metalness 0 roughness 1 }} geometry Cylinder {{ height 13 radius 0.09 }} }} ] }}
    Pose {{ translation 0 0 14.2 children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.12 0.70 emissiveColor 1 0.10 0.60 metalness 0 roughness 1 }} geometry Sphere {{ radius 0.55 }} }} ] }}
  ]
}}
"""


def write_course():
    L = [_OMNILINK_LICENCE_HEADER,
         "# Generated by tools/gen_city_nav.py — do not hand-edit.",
         '"""City pedestrian course: sidewalk waypoints + a traffic crossing."""', "",
         f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}",
         f"START = ({START[0]}, {START[1]})", "", "ROUTE_ENU = ["]
    for e, n, name in ROUTE:
        L.append(f"    ({e}, {n}, {name!r}),")
    L.append("]")
    L.append("OBSTACLES = []  # buildings/cars are camera-detected; none scored statically")
    L.append("")
    L.append("# (px, py, safe_gap, approach, commit) — wait for a gap before crossing")
    L.append(f"CROSSINGS = {CROSSINGS!r}")
    L.append("")
    L.append("# Global route graph (deploy-with-reroute).")
    L.append(f"GRAPH_NODES = {GRAPH_NODES!r}")
    L.append(f"GRAPH_EDGES = {GRAPH_EDGES!r}")
    L.append(f"START_NODE = {START_NODE!r}")
    L.append(f"DEST_NODE = {DEST_NODE!r}")
    COURSE_OUT.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    if not CITY_SRC.exists():
        raise SystemExit(f"city source not found: {CITY_SRC}")
    city = CITY_SRC.read_text(encoding="utf-8", errors="replace")
    WORLD_OUT.write_text(city.rstrip() + "\n" + HUSKY, encoding="utf-8")
    write_course()
    print(f"[gen] composed {WORLD_OUT.relative_to(PROJ)} (pedestrian on the y=-23 "
          f"sidewalk + collision deck)")
    print(f"[gen] route {len(ROUTE)} waypoints, {len(CROSSINGS)} crossing; "
          f"wrote {COURSE_OUT.name}")
    print("[gen] NOTE: tools/setup_city_nav.sh must have copied the city_traffic ctrl")


if __name__ == "__main__":
    main()
