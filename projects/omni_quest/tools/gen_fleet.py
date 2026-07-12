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

"""Generate the cross-platform fleet world: a Husky AND a Jackal navigating.

Two different robot platforms run the *same* GPS + camera navigation algorithm
(omni_quest_nav) at once, each with its own camera sidecar, on parallel lanes
through a shared mixed obstacle field. Proves the algorithm is platform-agnostic.

Writes worlds/omni_quest_fleet.wbt + controllers/omni_quest_nav/course_fleet.py.
    python projects/omni_quest/tools/gen_fleet.py
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
WORLD_OUT = PROJ / "worlds" / "omni_quest_fleet.wbt"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_fleet.py"

SEED = 5
REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0

START_HUSKY = (-28.0, -14.0)
ROUTE_HUSKY = [(-10.0, -14.0, "h1"), (6.0, -16.0, "h2"),
               (20.0, -12.0, "h3"), (28.0, -15.0, "h4")]
START_JACKAL = (-28.0, 14.0)
ROUTE_JACKAL = [(-10.0, 14.0, "j1"), (6.0, 16.0, "j2"),
                (20.0, 12.0, "j3"), (28.0, 15.0, "j4")]

WP_CLEAR = 3.0
OBS_SPACING = 2.8
KINDS = [("Pine", 0.7), ("Oak", 0.8), ("rock", 0.8), ("barrel", 0.4),
         ("crate", 0.45), ("cone", 0.35)]

EXTERNPROTOS = [
    "objects/backgrounds/protos/OmniSimSky.proto",
    "objects/lights/protos/OmniSimSun.proto",
    "objects/lights/protos/OmniSimSunMarker.proto",
    "appearances/protos/Grass.proto",
    "objects/trees/protos/Pine.proto",
    "objects/trees/protos/Oak.proto",
    "objects/rocks/protos/Rock.proto",
    "objects/obstacles/protos/OilBarrel.proto",
    "objects/factory/containers/protos/WoodenBox.proto",
    "objects/traffic/protos/TrafficCone.proto",
]


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def far(p, pts, d):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= d for q in pts)


def yaw_to(start, wp0):
    return math.atan2(wp0[1] - start[1], wp0[0] - start[0])


def seed_lane(rng, start, route, keepout, obstacles):
    nodes = [start] + [(e, n) for e, n, _ in route]
    for a, b in zip(nodes, nodes[1:]):
        for t in (0.35, 0.55, 0.75):
            base = lerp(a, b, t)
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy) or 1.0
            px, py = -dy / L, dx / L
            off = rng.uniform(0.7, 2.2) * rng.choice((-1, 1))
            cand = (base[0] + px * off, base[1] + py * off)
            if (far(cand, keepout, WP_CLEAR)
                    and far(cand, [(x, y) for x, y, _, _ in obstacles], OBS_SPACING)):
                kind, radius = rng.choice(KINDS)
                obstacles.append((cand[0], cand[1], kind, radius))


def generate(rng):
    keepout = [START_HUSKY, START_JACKAL]
    keepout += [(e, n) for e, n, _ in ROUTE_HUSKY + ROUTE_JACKAL]
    obstacles = []
    seed_lane(rng, START_HUSKY, ROUTE_HUSKY, keepout, obstacles)
    seed_lane(rng, START_JACKAL, ROUTE_JACKAL, keepout, obstacles)
    return obstacles


def emit_obstacle(kind, x, y, i, rng):
    a = round(rng.uniform(0, 6.28), 2)
    if kind in ("Pine", "Oak"):
        return f'{kind} {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 rotation 0 0 1 {a} }}'
    if kind == "rock":
        return f'Rock {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale {round(rng.uniform(1.1, 1.8), 2)} }}'
    if kind == "barrel":
        return f'OilBarrel {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.44 }}'
    if kind == "crate":
        return f'WoodenBox {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.3 }}'
    return f'TrafficCone {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale 1.3 }}'


def post(e, n, name, color):
    return (f'Solid {{ translation {e} {n} 0.8 name "post_{name}" children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {color} roughness 0.4 metalness 0 }} '
            f'geometry Cylinder {{ height 1.6 radius 0.08 }} }} ] }}')


def navigator(defname, url, start, platform, rid, route_attr, cam_z, cam_off):
    yaw = yaw_to(start, (ROUTE_HUSKY if rid == "husky" else ROUTE_JACKAL)[0])
    z = 0.15 if platform == "husky" else 0.1
    eye_def = defname.replace("_NAV", "_EYE")
    return (
        f'DEF {defname} URDFRobot {{\n'
        f'  url "{url}"\n  translation {start[0]} {start[1]} {z}\n'
        f'  rotation 0 0 1 {yaw:.4f}\n  name "{rid}_nav"\n  supervisor TRUE\n'
        f'  controller "omni_quest_nav"\n'
        f'  controllerArgs ["--course" "fleet" "--route" "{route_attr}" '
        f'"--platform" "{platform}" "--id" "{rid}"]\n}}\n'
        f'DEF {eye_def} Robot {{\n'
        f'  translation {start[0]} {start[1]} {z}\n  rotation 0 0 1 {yaw:.4f}\n'
        f'  name "{rid}_cam"\n  supervisor TRUE\n  controller "omni_quest_eye"\n'
        f'  controllerArgs ["--target" "{defname}" "--id" "{rid}"]\n  physics NULL\n'
        f'  children [ Camera {{ name "front_camera" translation {cam_off} 0 {cam_z} '
        f'rotation 0 1 0 0.22 width 128 height 48 fieldOfView 1.2 }} ] }}')


def write_world(obstacles, rng):
    P = ["#VRML_SIM R2025a utf8",
         "# Omni Quest — CROSS-PLATFORM FLEET (generated by tools/gen_fleet.py).",
         "# A Husky and a Jackal run the SAME GPS+camera nav algorithm at once, on",
         "# parallel lanes through a shared obstacle field. Regenerate; don't edit."]
    for p in EXTERNPROTOS:
        P.append(f'EXTERNPROTO "omnisim://projects/{p}"')
    P.append(f"""
WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "WGS84"
  gpsReference {REF_LAT} {REF_LON} {REF_ALT}
  contactProperties [ ContactProperties {{ material1 "default" material2 "default" maxContactJoints 30 }} ]
}}
Viewpoint {{ orientation -0.30 0.32 0.90 1.66 position 2 -52 46 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}
DirectionalLight {{ direction -0.5 0.4 -0.6 color 0.9 0.9 1.0 intensity 1.3 ambientIntensity 0.0 castShadows FALSE }}
DEF GROUND Solid {{
  name "ground"
  children [ Shape {{ appearance Grass {{ textureTransform TextureTransform {{ scale 50 50 }} }} geometry DEF G Plane {{ size 200 200 }} }} ]
  boundingObject USE G
  locked TRUE
}}""")
    P.append(navigator("HUSKY_NAV",
                       "../../robots/clearpath/husky_description/urdf/husky.urdf",
                       START_HUSKY, "husky", "husky", "ROUTE_HUSKY", "0.35", "0.32"))
    P.append(navigator("JACKAL_NAV",
                       "../../robots/clearpath/jackal_description/urdf/jackal.urdf",
                       START_JACKAL, "jackal", "jackal", "ROUTE_JACKAL", "0.28", "0.24"))
    for i, (e, n, name) in enumerate(ROUTE_HUSKY):
        P.append(post(e, n, name, "0.98 0.65 0.10"))
    for i, (e, n, name) in enumerate(ROUTE_JACKAL):
        P.append(post(e, n, name, "0.10 0.55 0.95"))
    P.append("# --- shared mixed obstacle field ---")
    for i, (x, y, kind, _r) in enumerate(obstacles):
        P.append(emit_obstacle(kind, x, y, i, rng))
    WORLD_OUT.write_text("\n".join(P) + "\n", encoding="utf-8")


def write_course(obstacles):
    L = [_OMNILINK_LICENCE_HEADER,
         "# Generated by tools/gen_fleet.py — do not hand-edit.",
         '"""Fleet course: shared ref + obstacles, one route per platform."""', "",
         f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}", ""]
    for nm, route in (("ROUTE_HUSKY", ROUTE_HUSKY), ("ROUTE_JACKAL", ROUTE_JACKAL)):
        L.append(f"{nm} = [")
        for e, n, name in route:
            L.append(f"    ({e}, {n}, {name!r}),")
        L.append("]")
    L.append("ROUTE_ENU = ROUTE_HUSKY  # default")
    L.append("")
    L.append("OBSTACLES = [")
    for x, y, _k, r in obstacles:
        L.append(f"    ({x:.2f}, {y:.2f}, {r}),")
    L.append("]")
    COURSE_OUT.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    rng = random.Random(SEED)
    obstacles = generate(rng)
    write_world(obstacles, rng)
    write_course(obstacles)
    print(f"[gen] fleet: Husky + Jackal, {len(obstacles)} obstacles, "
          f"{len(ROUTE_HUSKY)}+{len(ROUTE_JACKAL)} waypoints")
    print(f"[gen] wrote {WORLD_OUT.relative_to(PROJ)} + {COURSE_OUT.name}")


if __name__ == "__main__":
    main()
