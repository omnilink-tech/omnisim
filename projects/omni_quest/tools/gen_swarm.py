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

"""Generate the interacting swarm world: every robot is a NAVIGATOR, paths cross.

Four robots (2 Husky + 2 Jackal) all run the same omni_quest_nav GPS+camera
algorithm, on routes that cross each other mid-field — so each must see and avoid
the *other moving robots* with its camera (a robot reads as a non-grass obstacle
to the traversability segmentation), on top of a few static obstacles.

Writes worlds/omni_quest_swarm.omniworld + controllers/omni_quest_nav/course_swarm.py.
    python projects/omni_quest/tools/gen_swarm.py
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
WORLD_OUT = PROJ / "worlds" / "omni_quest_swarm.omniworld"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_swarm.py"

SEED = 9
REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0

HUSKY_URL = "../../robots/clearpath/husky_description/urdf/husky.urdf"
JACKAL_URL = "../../robots/clearpath/jackal_description/urdf/jackal.urdf"

# Two shared lanes with TWO-WAY traffic, so each pair meets HEAD-ON mid-lane —
# the case a forward camera can actually solve (both robots see each other dead
# ahead and bear off to pass, like traffic). A 4-way perpendicular crossing
# can't be solved by forward cameras alone; that needs right-of-way coordination.
#   lane y=-6:  n1 (Husky)  West->East   meets   n2 (Jackal) East->West
#   lane y=+6:  n3 (Jackal) West->East   meets   n4 (Husky)  East->West
NAVS = [
    # rid, platform, url, start, route, cam_z, cam_off
    ("n1", "husky", HUSKY_URL, (-32.0, -6.0), [(32.0, -6.0, "a1")], 0.35, 0.32),
    ("n2", "jackal", JACKAL_URL, (32.0, -6.0), [(-32.0, -6.0, "b1")], 0.28, 0.24),
    ("n3", "jackal", JACKAL_URL, (-32.0, 6.0), [(32.0, 6.0, "c1")], 0.28, 0.24),
    ("n4", "husky", HUSKY_URL, (32.0, 6.0), [(-32.0, 6.0, "d1")], 0.35, 0.32),
]

# Static obstacles near the lane ENDS only, so the central zone (x in [-18,18])
# is clear for the head-on robot-robot pass while each robot still weaves around
# static obstacles near its start/finish.
STATIC = [(-22.0, -4.5, "Pine", 0.7), (22.0, -7.5, "rock", 0.8),
          (-22.0, 7.5, "Oak", 0.8), (22.0, 4.5, "barrel", 0.4),
          (-26.0, -7.2, "cone", 0.35), (26.0, 6.2, "cone", 0.35)]

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


def yaw_to(start, wp0):
    return math.atan2(wp0[1] - start[1], wp0[0] - start[0])


def emit_obstacle(kind, x, y, i, rng):
    a = round(rng.uniform(0, 6.28), 2)
    if kind in ("Pine", "Oak"):
        return f'{kind} {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 rotation 0 0 1 {a} }}'
    if kind == "rock":
        return f'Rock {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale {round(rng.uniform(1.1, 1.7), 2)} }}'
    if kind == "barrel":
        return f'OilBarrel {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.44 }}'
    if kind == "crate":
        return f'WoodenBox {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0.3 }}'
    return f'TrafficCone {{ name "obs_{i}" translation {x:.2f} {y:.2f} 0 scale 1.3 }}'


def post(e, n, name, color):
    return (f'Solid {{ translation {e} {n} 0.8 name "post_{name}" children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {color} roughness 0.4 metalness 0 }} '
            f'geometry Cylinder {{ height 1.6 radius 0.08 }} }} ] }}')


def navigator(rid, platform, url, start, route, cam_z, cam_off):
    defname = f"{rid.upper()}_NAV"
    eye_def = f"{rid.upper()}_EYE"
    z = 0.15 if platform == "husky" else 0.1
    yaw = yaw_to(start, route[0])
    return (
        f'DEF {defname} URDFRobot {{\n'
        f'  url "{url}"\n  translation {start[0]} {start[1]} {z}\n'
        f'  rotation 0 0 1 {yaw:.4f}\n  name "{rid}_nav"\n  supervisor TRUE\n'
        f'  controller "omni_quest_nav"\n'
        f'  controllerArgs ["--course" "swarm" "--route" "ROUTE_{rid.upper()}" '
        f'"--platform" "{platform}" "--id" "{rid}" "--vmax" "0.6" "--coordinate"]\n}}\n'
        f'DEF {eye_def} Robot {{\n'
        f'  translation {start[0]} {start[1]} {z}\n  rotation 0 0 1 {yaw:.4f}\n'
        f'  name "{rid}_cam"\n  supervisor TRUE\n  controller "omni_quest_eye"\n'
        f'  controllerArgs ["--target" "{defname}" "--id" "{rid}"]\n  physics NULL\n'
        f'  children [ Camera {{ name "front_camera" translation {cam_off} 0 {cam_z} '
        f'rotation 0 1 0 0.22 width 160 height 48 fieldOfView 1.9 }} ] }}')


def write_world(rng):
    colors = ["0.98 0.65 0.10", "0.10 0.55 0.95", "0.20 0.85 0.30", "0.90 0.20 0.85"]
    P = ["#VRML_SIM R2025a utf8",
         "# Omni Quest — INTERACTING SWARM (generated by tools/gen_swarm.py).",
         "# Four robots (2 Husky + 2 Jackal) all run omni_quest_nav, on crossing",
         "# lanes, so each avoids the OTHER moving robots by camera. Regenerate."]
    for p in EXTERNPROTOS:
        P.append(f'EXTERNPROTO "omnisim://projects/{p}"')
    P.append(f"""
WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "WGS84"
  gpsReference {REF_LAT} {REF_LON} {REF_ALT}
  contactProperties [ ContactProperties {{ material1 "default" material2 "default" maxContactJoints 30 }} ]
}}
Viewpoint {{ orientation -0.31 0.32 0.89 1.66 position 0 -56 50 }}
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
    for rid, platform, url, start, route, cz, co in NAVS:
        P.append(navigator(rid, platform, url, start, route, cz, co))
    for i, (rid, _p, _u, _s, route, _cz, _co) in enumerate(NAVS):
        for e, n, name in route:
            P.append(post(e, n, name, colors[i % len(colors)]))
    P.append("# --- static obstacles (clear of the crossing square) ---")
    for i, (x, y, kind, _r) in enumerate(STATIC):
        P.append(emit_obstacle(kind, x, y, i, rng))
    WORLD_OUT.write_text("\n".join(P) + "\n", encoding="utf-8")


def write_course():
    L = [_OMNILINK_LICENCE_HEADER,
         "# Generated by tools/gen_swarm.py — do not hand-edit.",
         '"""Swarm course: shared ref + static obstacles, one route per robot."""', "",
         f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}", ""]
    for rid, _p, _u, _s, route, _cz, _co in NAVS:
        L.append(f"ROUTE_{rid.upper()} = [")
        for e, n, name in route:
            L.append(f"    ({e}, {n}, {name!r}),")
        L.append("]")
    L.append("ROUTE_ENU = ROUTE_N1  # default")
    L.append("")
    L.append("OBSTACLES = [")
    for x, y, _k, r in STATIC:
        L.append(f"    ({x:.2f}, {y:.2f}, {r}),")
    L.append("]")
    COURSE_OUT.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    rng = random.Random(SEED)
    write_world(rng)
    write_course()
    print(f"[gen] swarm: {len(NAVS)} navigators (crossing lanes), "
          f"{len(STATIC)} static obstacles")
    print(f"[gen] wrote {WORLD_OUT.relative_to(PROJ)} + {COURSE_OUT.name}")


if __name__ == "__main__":
    main()
