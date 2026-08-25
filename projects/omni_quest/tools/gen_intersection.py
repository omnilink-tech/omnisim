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

"""Generate the 4-way intersection world, solved with SIDE CAMERAS.

Four robots (2 Husky + 2 Jackal) all run omni_quest_nav and drive straight
across a shared field on PERPENDICULAR lanes that cross at four points. Each
robot has a front camera (steering) AND a forward-right camera, so it can see
crossing traffic before it is dead ahead and apply a camera-only "yield to the
traffic on your right" rule — which breaks the crossing symmetry with no V2V
(the other robot sees that same traffic on its left and proceeds).

Writes worlds/omni_quest_intersection.omniworld + course_intersection.py.
    python projects/omni_quest/tools/gen_intersection.py
"""

from __future__ import annotations

import math
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
WORLD_OUT = PROJ / "worlds" / "omni_quest_intersection.omniworld"
COURSE_OUT = PROJ / "controllers" / "omni_quest_nav" / "course_intersection.py"

REF_LAT, REF_LON, REF_ALT = 40.67, -73.94, 0.0
HUSKY_URL = "../../robots/clearpath/husky_description/urdf/husky.urdf"
JACKAL_URL = "../../robots/clearpath/jackal_description/urdf/jackal.urdf"

# A single perpendicular crossing at the origin (kept to 2 robots so the camera
# render load stays light enough that perception stays fresh):
#   n1 (Husky)  W->E @ y=0   meets   n2 (Jackal) S->N @ x=0   at (0,0)
# n2 is on n1's RIGHT, so n1 yields; n1 is on n2's LEFT, so n2 proceeds.
NAVS = [
    ("n1", "husky", HUSKY_URL, (-28.0, 0.0), [(28.0, 0.0, "a1")], 0.35, 0.32),
    ("n2", "jackal", JACKAL_URL, (0.0, -28.0), [(0.0, 28.0, "b1")], 0.28, 0.24),
]

EXTERNPROTOS = [
    "objects/backgrounds/protos/OmniSimSky.proto",
    "objects/lights/protos/OmniSimSun.proto",
    "objects/lights/protos/OmniSimSunMarker.proto",
    "appearances/protos/Grass.proto",
]


def yaw_to(start, wp0):
    return math.atan2(wp0[1] - start[1], wp0[0] - start[0])


def post(e, n, name, color):
    return (f'Solid {{ translation {e} {n} 0.8 name "post_{name}" children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {color} roughness 0.4 metalness 0 }} '
            f'geometry Cylinder {{ height 1.6 radius 0.08 }} }} ] }}')


def navigator(rid, platform, url, start, route, cz, co):
    defname = f"{rid.upper()}_NAV"
    eye_def = f"{rid.upper()}_EYE"
    z = 0.15 if platform == "husky" else 0.1
    yaw = yaw_to(start, route[0])
    # front camera (steering) + forward-RIGHT camera (yaw -1.0 rad via a Pose, so
    # it sees crossing traffic approaching from the right before it's dead ahead).
    cams = (
        f'Camera {{ name "front_camera" translation {co} 0 {cz} rotation 0 1 0 0.22 '
        f'width 128 height 48 fieldOfView 1.2 }} '
        f'Pose {{ rotation 0 0 1 -1.0 children [ Camera {{ name "right_camera" '
        f'translation {co} 0 {cz} rotation 0 1 0 0.30 width 80 height 32 '
        f'fieldOfView 1.5 }} ] }}')
    return (
        f'DEF {defname} URDFRobot {{\n'
        f'  url "{url}"\n  translation {start[0]} {start[1]} {z}\n'
        f'  rotation 0 0 1 {yaw:.4f}\n  name "{rid}_nav"\n  supervisor TRUE\n'
        f'  controller "omni_quest_nav"\n'
        f'  controllerArgs ["--course" "intersection" "--route" "ROUTE_{rid.upper()}" '
        f'"--platform" "{platform}" "--id" "{rid}" "--vmax" "0.6"]\n}}\n'
        f'DEF {eye_def} Robot {{\n'
        f'  translation {start[0]} {start[1]} {z}\n  rotation 0 0 1 {yaw:.4f}\n'
        f'  name "{rid}_cam"\n  supervisor TRUE\n  controller "omni_quest_eye"\n'
        f'  controllerArgs ["--target" "{defname}" "--id" "{rid}"]\n  physics NULL\n'
        f'  children [ {cams} ] }}')


def write_world():
    colors = ["0.98 0.65 0.10", "0.10 0.55 0.95", "0.20 0.85 0.30", "0.90 0.20 0.85"]
    P = ["#VRML_SIM R2025a utf8",
         "# Omni Quest — 4-WAY INTERSECTION solved with side cameras + yield-to-right",
         "# (generated by tools/gen_intersection.py). Regenerate; don't hand-edit."]
    for p in EXTERNPROTOS:
        P.append(f'EXTERNPROTO "omnisim://projects/{p}"')
    P.append(f"""
WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "WGS84"
  gpsReference {REF_LAT} {REF_LON} {REF_ALT}
  contactProperties [ ContactProperties {{ material1 "default" material2 "default" maxContactJoints 30 }} ]
}}
Viewpoint {{ orientation -0.31 0.32 0.89 1.66 position 0 -54 48 }}
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
    WORLD_OUT.write_text("\n".join(P) + "\n", encoding="utf-8")


def write_course():
    L = [_OMNILINK_LICENCE_HEADER,
         "# Generated by tools/gen_intersection.py — do not hand-edit.",
         '"""Intersection course: one straight route per robot, crossing lanes."""', "",
         f"REF_LAT = {REF_LAT}", f"REF_LON = {REF_LON}", f"REF_ALT = {REF_ALT}", ""]
    for rid, _p, _u, _s, route, _cz, _co in NAVS:
        L.append(f"ROUTE_{rid.upper()} = [")
        for e, n, name in route:
            L.append(f"    ({e}, {n}, {name!r}),")
        L.append("]")
    L.append("ROUTE_ENU = ROUTE_N1  # default")
    L.append("OBSTACLES = []  # no static obstacles — this world is about the crossing")
    COURSE_OUT.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    write_world()
    write_course()
    print(f"[gen] intersection: {len(NAVS)} navigators on perpendicular crossing "
          f"lanes, front+right cameras, yield-to-right")
    print(f"[gen] wrote {WORLD_OUT.relative_to(PROJ)} + {COURSE_OUT.name}")


if __name__ == "__main__":
    main()
