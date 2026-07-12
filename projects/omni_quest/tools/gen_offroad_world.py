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

"""Generate the harder Omni Quest course: rough terrain + a tree obstacle field.

Writes two files from one seeded source so the world and the controller's
obstacle map can never drift:

  * worlds/omni_quest_offroad.wbt              — the world
  * controllers/omni_quest_nav/course_offroad.py — ROUTE_ENU + OBSTACLES the
                                                   controller imports (passed
                                                   --course offroad)

Obstacles are seeded *near the route segments* on purpose, so the straight-line
path between waypoints is blocked and the robot has to detour around them.

    python projects/omni_quest/tools/gen_offroad_world.py
"""

from __future__ import annotations

import math
import random
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
WORLD_OUT = PROJ / "worlds" / "omni_quest_offroad.wbt"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_offroad.py"

SEED = 7
REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0

# Husky start (ENU m) + a winding route across the field.
START = (-25.0, -25.0)
ROUTE = [
    (-8.0, -18.0, "south_ridge"),
    (18.0, -20.0, "south_post"),
    (24.0, 8.0, "east_bend"),
    (-4.0, 20.0, "north_glade"),
]

# Terrain: rougher rolling (relief = size.z + finer octaves) -> more wheel slip.
TERRAIN_SIZE = (70.0, 70.0, 0.7)
TERRAIN_GRID = 60
TERRAIN_OCTAVES = 5
SPAWN_Z = 0.85                     # just above max relief; husky settles onto it

OBS_RADIUS = 0.7                   # sensor disc per tree (trunk + margin)
WP_CLEAR = 3.0                     # keep obstacles this far from any waypoint/start
OBS_SPACING = 2.5                  # tighter gaps between obstacles (harder to thread)
SCATTER = 8                        # extra random trees for realism + density


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def far_enough(p, pts, d):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= d for q in pts)


def generate():
    rng = random.Random(SEED)
    waypts = [(e, n) for e, n, _ in ROUTE]
    keepout = [START] + waypts
    obstacles = []  # (x, y)

    # 1) Seed obstacles near each route segment so the path is blocked.
    nodes = [START] + waypts
    for a, b in zip(nodes, nodes[1:]):
        for t in (0.25, 0.4, 0.55, 0.7, 0.85):
            base = lerp(a, b, t)
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy) or 1.0
            px, py = -dy / L, dx / L                 # unit perpendicular
            off = rng.uniform(0.6, 2.2) * rng.choice((-1, 1))
            cand = (base[0] + px * off, base[1] + py * off)
            if (far_enough(cand, keepout, WP_CLEAR)
                    and far_enough(cand, obstacles, OBS_SPACING)):
                obstacles.append(cand)

    # 2) A few scattered trees for realism (still off the waypoints).
    tries = 0
    target = len(obstacles) + SCATTER
    while len(obstacles) < target and tries < 500:
        tries += 1
        cand = (rng.uniform(-28, 28), rng.uniform(-28, 28))
        if (far_enough(cand, keepout, WP_CLEAR)
                and far_enough(cand, obstacles, OBS_SPACING)):
            obstacles.append(cand)

    return obstacles, rng


def spawn_yaw():
    dx = ROUTE[0][0] - START[0]
    dy = ROUTE[0][1] - START[1]
    return math.atan2(dy, dx)


def post(e, n, name, color):
    return f"""DEF {name.upper()} Solid {{
  translation {e} {n} 0.6
  name "post_{name}"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 0.4 metalness 0 }}
      geometry Cylinder {{ height 1.6 radius 0.09 }}
    }}
  ]
}}
"""


def write_world(obstacles, rng):
    yaw = spawn_yaw()
    colors = ["0.98 0.65 0.10", "0.10 0.85 0.20", "0.10 0.30 0.95", "0.95 0.95 0.95"]
    sx, sy, sz = TERRAIN_SIZE
    parts = []
    parts.append('#VRML_SIM R2025a utf8')
    parts.append('# Omni Quest — OFF-ROAD course (generated by tools/gen_offroad_world.py).')
    parts.append('# Rough rolling terrain + a tree obstacle field seeded across the route.')
    parts.append('# Controller (omni_quest_nav, --course offroad) navigates GPS waypoints')
    parts.append('# AND avoids obstacles via a modelled forward range sensor. Do not hand-edit;')
    parts.append('# regenerate with: python projects/omni_quest/tools/gen_offroad_world.py')
    for p in (
        'omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto',
        'omnisim://projects/objects/lights/protos/OmniSimSun.proto',
        'omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto',
        'omnisim://projects/appearances/protos/Grass.proto',
        'omnisim://projects/objects/floors/protos/UnevenTerrain.proto',
        'omnisim://projects/objects/trees/protos/Pine.proto',
        'omnisim://projects/objects/trees/protos/Oak.proto',
    ):
        parts.append(f'EXTERNPROTO "{p}"')
    parts.append(f"""
WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "WGS84"
  gpsReference {REF_LAT} {REF_LON} {REF_ALT}
  contactProperties [
    ContactProperties {{ material1 "default" material2 "default" maxContactJoints 50 }}
  ]
}}
Viewpoint {{ orientation -0.32 0.33 0.89 1.66 position 6 -52 46 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}
DirectionalLight {{ direction -0.5 0.4 -0.6 color 0.85 0.88 1.0 intensity 1.4 ambientIntensity 0.0 castShadows FALSE }}
UnevenTerrain {{
  name "terrain"
  size {sx} {sy} {sz}
  xDimension {TERRAIN_GRID} yDimension {TERRAIN_GRID}
  perlinNOctaves {TERRAIN_OCTAVES}
  randomSeed {SEED}
  appearance Grass {{ textureTransform TextureTransform {{ scale 40 40 }} }}
}}
DEF HUSKY URDFRobot {{
  url "../../robots/clearpath/husky_description/urdf/husky.urdf"
  translation {START[0]} {START[1]} {SPAWN_Z}
  rotation 0 0 1 {yaw:.4f}
  name "husky"
  supervisor TRUE
  controller "omni_quest_nav"
  controllerArgs ["--course" "offroad"]
}}
# Forward-camera sidecar. Device children mis-bind on an imported URDFRobot, so
# the camera rides a separate Robot that snaps onto the Husky each tick (the
# proven husky_eye pattern). Its controller (omni_quest_eye) runs the vision and
# writes _perception.json; the nav controller reads it. No physics = static.
DEF HUSKY_CAM Robot {{
  translation {START[0]} {START[1]} {SPAWN_Z}
  rotation 0 0 1 {yaw:.4f}
  name "husky_cam"
  supervisor TRUE
  controller "omni_quest_eye"
  physics NULL
  children [
    Camera {{
      name "front_camera"
      translation 0.32 0 0.35
      rotation 0 1 0 0.22
      width 128
      height 48
      fieldOfView 1.2
    }}
  ]
}}""")
    # Waypoint posts.
    for i, (e, n, name) in enumerate(ROUTE):
        parts.append(post(e, n, name, colors[i % len(colors)]))
    # Obstacle trees (alternate Pine / Oak), planted (z = -0.5) so the rolling
    # terrain never leaves a trunk floating.
    for i, (x, y) in enumerate(obstacles):
        kind = "Pine" if i % 2 == 0 else "Oak"
        a = rng.uniform(0, 6.28)
        parts.append(f'{kind} {{ name "obs_{i}" translation {x:.2f} {y:.2f} -0.5 '
                     f'rotation 0 0 1 {a:.2f} }}')
    WORLD_OUT.write_text("\n".join(parts) + "\n", encoding="utf-8")


def write_course(obstacles):
    lines = [
        _OMNILINK_LICENCE_HEADER,
        "# Generated by tools/gen_offroad_world.py — do not hand-edit.",
        '"""Off-road course data shared by the controller and the plot tools."""',
        "",
        f"REF_LAT = {REF_LAT}",
        f"REF_LON = {REF_LON}",
        f"REF_ALT = {REF_ALT}",
        f"START = ({START[0]}, {START[1]})",
        "",
        "# (east_m, north_m, name)",
        "ROUTE_ENU = [",
    ]
    for e, n, name in ROUTE:
        lines.append(f"    ({e}, {n}, {name!r}),")
    lines.append("]")
    lines.append("")
    lines.append("# (x_m, y_m, radius_m) discs for the modelled range sensor")
    lines.append("OBSTACLES = [")
    for x, y in obstacles:
        lines.append(f"    ({x:.2f}, {y:.2f}, {OBS_RADIUS}),")
    lines.append("]")
    COURSE_OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    obstacles, rng = generate()
    write_world(obstacles, rng)
    write_course(obstacles)
    print(f"[gen] {len(obstacles)} obstacles, {len(ROUTE)} waypoints")
    print(f"[gen] wrote {WORLD_OUT.relative_to(PROJ)}")
    print(f"[gen] wrote {COURSE_OUT.relative_to(PROJ)}")


if __name__ == "__main__":
    main()
