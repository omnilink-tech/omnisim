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

"""Generate the populated "living" Omni Quest world.

A flat grass field (so the many props sit correctly and physics stays stable),
densely populated:
  * a GPS route threaded through a MIXED obstacle field — trees, rocks, barrels,
    crates, cones (all with collision; the camera segments them as non-grass),
  * backdrop structures (barn, silo, shipping containers, fences),
  * a pasture with animals + flowers,
  * THREE roaming background robots (Husky + Jackal + TurtleBot) that wander.

Writes worlds/omni_quest_world.wbt + controllers/omni_quest_nav/course_world.py.
    python projects/omni_quest/tools/gen_world.py
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
WORLD_OUT = PROJ / "worlds" / "omni_quest_world.wbt"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_world.py"

SEED = 11
REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0

START = (-26.0, -22.0)
ROUTE = [
    (-8.0, -16.0, "south_field"),
    (20.0, -18.0, "east_marker"),
    (22.0, 8.0, "ridge_bend"),
    (0.0, 16.0, "pasture_gate"),
    (-22.0, 10.0, "west_stand"),
]
PASTURE = (0.0, 29.0, 12.0)        # (cx, cy, radius) — animals + roaming robots
WP_CLEAR = 3.2
OBS_SPACING = 2.8

# obstacle kind -> sensor/scoring radius (m)
KINDS = [("Pine", 0.7), ("Oak", 0.8), ("Cypress", 0.6),
         ("rock", 0.8), ("barrel", 0.4), ("crate", 0.45), ("cone", 0.35)]

EXTERNPROTOS = [
    "objects/backgrounds/protos/OmniSimSky.proto",
    "objects/lights/protos/OmniSimSun.proto",
    "objects/lights/protos/OmniSimSunMarker.proto",
    "appearances/protos/Grass.proto",
    "objects/trees/protos/Pine.proto",
    "objects/trees/protos/Oak.proto",
    "objects/trees/protos/Cypress.proto",
    "objects/rocks/protos/Rock.proto",
    "objects/obstacles/protos/OilBarrel.proto",
    "objects/factory/containers/protos/WoodenBox.proto",
    "objects/traffic/protos/TrafficCone.proto",
    "objects/buildings/protos/Barn.proto",
    "objects/buildings/protos/Silo.proto",
    "objects/freight/protos/IntermodalContainer.proto",
    "objects/street_furniture/protos/Fence.proto",
    "objects/animals/protos/Cow.proto",
    "objects/animals/protos/Sheep.proto",
    "objects/animals/protos/Horse.proto",
    "objects/animals/protos/Deer.proto",
    "objects/animals/protos/Dog.proto",
    "objects/plants/protos/BunchOfSunFlowers.proto",
    "objects/plants/protos/PottedTree.proto",
]


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def far(p, pts, d):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= d for q in pts)


def in_pasture(p, margin=2.0):
    return math.hypot(p[0] - PASTURE[0], p[1] - PASTURE[1]) < PASTURE[2] + margin


def spawn_yaw():
    return math.atan2(ROUTE[0][1] - START[1], ROUTE[0][0] - START[0])


def generate(rng):
    waypts = [(e, n) for e, n, _ in ROUTE]
    keepout = [START] + waypts
    obstacles = []  # (x, y, kind, radius)
    nodes = [START] + waypts
    for a, b in zip(nodes, nodes[1:]):
        for t in (0.28, 0.44, 0.6, 0.76, 0.9):
            base = lerp(a, b, t)
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy) or 1.0
            px, py = -dy / L, dx / L
            off = rng.uniform(0.7, 2.4) * rng.choice((-1, 1))
            cand = (base[0] + px * off, base[1] + py * off)
            if in_pasture(cand):
                continue
            if (far(cand, keepout, WP_CLEAR)
                    and far(cand, [(x, y) for x, y, _, _ in obstacles], OBS_SPACING)):
                kind, radius = rng.choice(KINDS)
                obstacles.append((cand[0], cand[1], kind, radius))
    # a few scattered for density (south of the pasture)
    target = len(obstacles) + 6
    tries = 0
    while len(obstacles) < target and tries < 400:
        tries += 1
        cand = (rng.uniform(-30, 30), rng.uniform(-32, 8))
        if in_pasture(cand):
            continue
        if (far(cand, keepout, WP_CLEAR)
                and far(cand, [(x, y) for x, y, _, _ in obstacles], OBS_SPACING)):
            kind, radius = rng.choice(KINDS)
            obstacles.append((cand[0], cand[1], kind, radius))
    return obstacles


def emit_obstacle(kind, x, y, i, rng):
    a = round(rng.uniform(0, 6.28), 2)
    if kind in ("Pine", "Oak", "Cypress"):
        return f'{kind} {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 rotation 0 0 1 {a} }}'
    if kind == "rock":
        s = round(rng.uniform(1.1, 1.9), 2)
        return f'Rock {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale {s} }}'
    if kind == "barrel":
        return f'OilBarrel {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.44 }}'
    if kind == "crate":
        return f'WoodenBox {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.3 }}'
    if kind == "cone":
        return f'TrafficCone {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale 1.3 }}'
    return ""


def post(e, n, name, color):
    return (f'DEF {name.upper()} Solid {{ translation {e} {n} 0.8 name "post_{name}" '
            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {color} '
            f'roughness 0.4 metalness 0 }} geometry Cylinder {{ height 1.6 radius 0.09 }} }} ] }}')


def roamer(kind, defname, ctrl_name, url, z, speed):
    cx, cy, r = PASTURE
    args = f'["--cx" "{cx}" "--cy" "{cy}" "--radius" "{r - 1.5:.1f}" "--speed" "{speed}"]'
    px = cx + (2.0 if defname == "ROAM_HUSKY" else (-3.0 if defname == "ROAM_JACKAL" else 4.0))
    py = cy + (-2.0 if defname == "ROAM_HUSKY" else (3.0 if defname == "ROAM_JACKAL" else 1.0))
    return (f'DEF {defname} URDFRobot {{\n'
            f'  url "{url}"\n'
            f'  translation {px:.1f} {py:.1f} {z}\n'
            f'  name "{ctrl_name}"\n  supervisor TRUE\n'
            f'  controller "omni_quest_wander"\n  controllerArgs {args}\n}}')


def write_world(obstacles, rng):
    yaw = spawn_yaw()
    colors = ["0.98 0.65 0.10", "0.10 0.85 0.20", "0.10 0.30 0.95",
              "0.95 0.95 0.95", "0.90 0.20 0.85"]
    P = []
    P.append("#VRML_SIM R2025a utf8")
    P.append("# Omni Quest — POPULATED 'living' world (generated by tools/gen_world.py).")
    P.append("# Flat grass field, a GPS route through a mixed obstacle field (trees/rocks/")
    P.append("# barrels/crates/cones), backdrop structures, a pasture of animals + flowers,")
    P.append("# and three roaming robots. Do not hand-edit; regenerate with gen_world.py.")
    for p in EXTERNPROTOS:
        P.append(f'EXTERNPROTO "omnisim://projects/{p}"')
    P.append(f"""
WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "WGS84"
  gpsReference {REF_LAT} {REF_LON} {REF_ALT}
  contactProperties [ ContactProperties {{ material1 "default" material2 "default" maxContactJoints 30 }} ]
}}
Viewpoint {{ orientation -0.31 0.33 0.89 1.66 position 4 -58 50 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}
DirectionalLight {{ direction -0.5 0.4 -0.6 color 0.9 0.9 1.0 intensity 1.3 ambientIntensity 0.0 castShadows FALSE }}
DEF GROUND Solid {{
  name "ground"
  children [ Shape {{ appearance Grass {{ textureTransform TextureTransform {{ scale 50 50 }} }} geometry DEF G Plane {{ size 220 220 }} }} ]
  boundingObject USE G
  locked TRUE
}}
# --- Main Husky + forward-camera sidecar (camera-based obstacle avoidance) ---
DEF HUSKY URDFRobot {{
  url "../../robots/clearpath/husky_description/urdf/husky.urdf"
  translation {START[0]} {START[1]} 0.15
  rotation 0 0 1 {yaw:.4f}
  name "husky"
  supervisor TRUE
  controller "omni_quest_nav"
  controllerArgs ["--course" "world"]
}}
DEF HUSKY_CAM Robot {{
  translation {START[0]} {START[1]} 0.15
  rotation 0 0 1 {yaw:.4f}
  name "husky_cam"
  supervisor TRUE
  controller "omni_quest_eye"
  physics NULL
  children [ Camera {{ name "front_camera" translation 0.32 0 0.35 rotation 0 1 0 0.22 width 128 height 48 fieldOfView 1.2 }} ]
}}""")
    # Waypoint posts
    for i, (e, n, name) in enumerate(ROUTE):
        P.append(post(e, n, name, colors[i % len(colors)]))
    # Mixed obstacle field
    P.append("# --- mixed obstacle field (trees / rocks / barrels / crates / cones) ---")
    for i, (x, y, kind, _r) in enumerate(obstacles):
        P.append(emit_obstacle(kind, x, y, i, rng))
    # Backdrop structures (edges, off-route)
    P.append("# --- backdrop structures ---")
    P.append('Barn { translation -34 30 0 rotation 0 0 1 -1.2 name "barn" }')
    P.append('Silo { translation 33 26 0 name "silo" }')
    P.append('IntermodalContainer { translation 30 -30 0 rotation 0 0 1 1.2 name "container_a" }')
    P.append('IntermodalContainer { translation 26 -30 0 rotation 0 0 1 1.2 name "container_b" }')
    P.append('Fence { translation -30 -30 0 rotation 0 0 1 0 name "fence_a" }')
    P.append('Fence { translation -22 -31 0 rotation 0 0 1 0 name "fence_b" }')
    # Pasture: animals + flowers (animals are decor — no collision)
    P.append("# --- pasture life: animals + flowers ---")
    cx, cy, _ = PASTURE
    animals = [("Cow", -4, 30), ("Sheep", 3, 33), ("Sheep", 6, 30),
               ("Horse", -7, 26), ("Deer", 8, 35), ("Dog", 1, 24)]
    for j, (a, ax, ay) in enumerate(animals):
        ang = round(rng.uniform(0, 6.28), 2)
        P.append(f'{a} {{ translation {ax} {ay} 0 rotation 0 0 1 {ang} name "{a.lower()}_{j}" }}')
    for j, (fx, fy) in enumerate([(-10, 33), (10, 31), (-2, 37)]):
        P.append(f'BunchOfSunFlowers {{ translation {fx} {fy} 0 name "flowers_{j}" }}')
    P.append('PottedTree { translation 12 22 0 name "potted_0" }')
    P.append('PottedTree { translation -12 20 0 name "potted_1" }')
    # Roaming robots
    P.append("# --- roaming background robots (wander the pasture) ---")
    P.append(roamer("husky", "ROAM_HUSKY", "roam_husky",
                    "../../robots/clearpath/husky_description/urdf/husky.urdf", "0.15", "3.0"))
    P.append(roamer("jackal", "ROAM_JACKAL", "roam_jackal",
                    "../../robots/clearpath/jackal_description/urdf/jackal.urdf", "0.1", "2.5"))
    P.append(roamer("jackal2", "ROAM_JACKAL2", "roam_jackal2",
                    "../../robots/clearpath/jackal_description/urdf/jackal.urdf", "0.1", "3.0"))
    WORLD_OUT.write_text("\n".join(P) + "\n", encoding="utf-8")


def write_course(obstacles):
    lines = [
        _OMNILINK_LICENCE_HEADER,
        "# Generated by tools/gen_world.py — do not hand-edit.",
        '"""Populated-world course data (route + scored obstacle discs)."""',
        "",
        f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}",
        f"START = ({START[0]}, {START[1]})",
        "", "ROUTE_ENU = [",
    ]
    for e, n, name in ROUTE:
        lines.append(f"    ({e}, {n}, {name!r}),")
    lines.append("]")
    lines.append("")
    lines.append("# (x, y, radius) collision discs for scoring (analyze_run.py)")
    lines.append("OBSTACLES = [")
    for x, y, _kind, r in obstacles:
        lines.append(f"    ({x:.2f}, {y:.2f}, {r}),")
    lines.append("]")
    COURSE_OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    rng = random.Random(SEED)
    obstacles = generate(rng)
    write_world(obstacles, rng)
    write_course(obstacles)
    print(f"[gen] {len(obstacles)} obstacles, {len(ROUTE)} waypoints, 3 roaming robots")
    print(f"[gen] wrote {WORLD_OUT.relative_to(PROJ)} + {COURSE_OUT.name}")


if __name__ == "__main__":
    main()
