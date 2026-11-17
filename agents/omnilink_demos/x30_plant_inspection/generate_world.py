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

"""Generate showcase/x30_plant_inspection.omniworld + its sign textures.

One table (NODES / EDGES / PLACES) drives both the scene geometry and the
robot's customData, so a place and the thing it names cannot drift apart.
Edit the table (or the scene below it), run this, and commit the world and
textures it writes -- the world file is the artefact, this is how it is made.

    python agents/omnilink_demos/x30_plant_inspection/generate_world.py

Deterministic: with no edits it reproduces the committed world byte for byte.
Needs Pillow. The camera Viewpoint was baked once with
scripts/dev/set_viewpoint.py (--center -9.0 -4.6 0.5 --radius 3.2) and is
copied into the header below.
"""
import json
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
WORLD_DIR = os.path.join(REPO, "projects", "samples", "demos", "worlds", "showcase")
TEX_REL = "textures/x30_plant"
TEX_DIR = os.path.join(WORLD_DIR, *TEX_REL.split("/"))
OUT = os.path.join(WORLD_DIR, "x30_plant_inspection.omniworld")
PORT = int(os.environ.get("X30_PORT", "8796"))

# ── Site plan (metres; x east, y north) ─────────────────────────────────
BOUNDS = (-12.0, 12.0, -9.0, 11.0)          # the fenced yard
WALK_W = 1.6                                # walkway width
NODES = {
    "SW": (-8.0, -5.0), "S_CR": (-4.5, -5.0), "S_MID": (0.0, -5.0),
    "S_P101": (2.5, -5.0), "SE": (8.0, -5.0), "E_E3": (8.0, 0.0),
    "NE": (8.0, 4.0), "N_T2": (3.5, 4.0), "N_MID": (0.0, 4.0),
    "N_T1": (-3.5, 4.0), "NW": (-8.0, 4.0), "W_V7": (-8.0, 1.0),
}
EDGES = [("SW", "S_CR"), ("S_CR", "S_MID"), ("S_MID", "S_P101"), ("S_P101", "SE"),
         ("SE", "E_E3"), ("E_E3", "NE"), ("NE", "N_T2"), ("N_T2", "N_MID"),
         ("N_MID", "N_T1"), ("N_T1", "NW"), ("NW", "W_V7"), ("W_V7", "SW"),
         ("S_MID", "N_MID")]
# name: (x, y, yaw_deg, description, walkway node)
PLACES = {
    "charging dock": (-9.9, -5.0, 0, "the robot's charging dock at the west end of the south walkway", "SW"),
    "control room": (-4.5, -5.9, -90, "the door of the operators' control-room container", "S_CR"),
    "pump P-101": (2.5, -5.9, -90, "feed pump P-101 and its discharge pressure gauge", "S_P101"),
    "tank T-1": (-3.5, 4.9, 90, "storage tank T-1 and its level gauge", "N_T1"),
    "tank T-2": (3.5, 4.9, 90, "storage tank T-2 and its level gauge", "N_T2"),
    "electrical cabinet E-3": (8.9, 0.0, 0, "motor-control cabinet E-3", "E_E3"),
    "valve manifold V-7": (-8.9, 1.0, 180, "valve manifold V-7 under the pipe rack", "W_V7"),
}
DOCK = PLACES["charging dock"]

out = []
w = out.append


def fmt(*v):
    return " ".join(("%.4g" % x) for x in v)


def shape(geom, app, t=(0, 0, 0), r=None, name=None):
    rot = "" if r is None else " rotation %s" % fmt(*r)
    return "Pose { translation %s%s children [ Shape { appearance %s geometry %s } ] }" % (
        fmt(*t), rot, app, geom)


def solid(name, children, t=(0, 0, 0), r=None, bo=None):
    rot = "" if r is None else "\n  rotation %s" % fmt(*r)
    bo_s = "" if bo is None else "\n  boundingObject %s" % bo
    w("Solid {\n  translation %s%s\n  children [\n    %s\n  ]\n  name \"%s\"%s\n}" % (
        fmt(*t), rot, "\n    ".join(children), name, bo_s))


_VIS = [0]


def visual(children, t=(0, 0, 0), r=None):
    """Scenery with no collider. A collider-less static Solid, not a bare
    Pose: root-level Pose scenery did not draw in the harness screenshots."""
    _VIS[0] += 1
    solid("scenery %d" % _VIS[0], children, t=t, r=r)


def pbr(c, rough=0.6, metal=0.0, tex=None, emissive=None):
    s = "PBRAppearance { baseColor %s roughness %g metalness %g" % (fmt(*c), rough, metal)
    if tex:
        s += " baseColorMap ImageTexture { url [ \"%s/%s\" ] filtering 4 }" % (TEX_REL, tex)
    if emissive:
        s += " emissiveColor %s emissiveIntensity 1" % fmt(*emissive)
    return s + " }"


YELLOW = pbr((0.93, 0.72, 0.05), 0.55)
WALK_GREEN = pbr((0.19, 0.34, 0.24), 0.75)
STEEL = "GalvanizedMetal { }"
PIPE_GREY = pbr((0.55, 0.57, 0.58), 0.45, 0.6)
TANK_WHITE = pbr((0.86, 0.87, 0.85), 0.35, 0.15)
DARK = pbr((0.12, 0.12, 0.13), 0.6)
RED = pbr((0.72, 0.08, 0.06), 0.4)

# ── Textures ────────────────────────────────────────────────────────────
os.makedirs(TEX_DIR, exist_ok=True)


def font(px):
    return ImageFont.load_default(size=px)


def sign(fname, title, sub="", bg=(250, 250, 245), fg=(20, 25, 30), band=(0, 90, 60)):
    img = Image.new("RGB", (512, 256), bg)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, 511, 34), fill=band)
    d.rectangle((6, 6, 505, 249), outline=fg, width=4)
    tw = d.textlength(title, font=font(104))
    d.text(((512 - tw) / 2, 58), title, font=font(104), fill=fg)
    if sub:
        sw = d.textlength(sub, font=font(34))
        d.text(((512 - sw) / 2, 186), sub, font=font(34), fill=fg)
    img.save(os.path.join(TEX_DIR, fname))


def gauge(fname, frac, label):
    s = 256
    img = Image.new("RGB", (s, s), (40, 40, 42))
    d = ImageDraw.Draw(img)
    d.ellipse((8, 8, s - 8, s - 8), fill=(246, 246, 240), outline=(30, 30, 30), width=6)
    cx = cy = s / 2
    a0, a1 = math.radians(225), math.radians(-45)
    # green / amber / red bands
    for lo, hi, col in ((0.0, 0.65, (40, 150, 60)), (0.65, 0.85, (230, 160, 20)), (0.85, 1.0, (200, 30, 20))):
        d.arc((34, 34, s - 34, s - 34), start=-math.degrees(a0 + (a1 - a0) * hi),
              end=-math.degrees(a0 + (a1 - a0) * lo), fill=col, width=10)
    for i in range(11):
        a = a0 + (a1 - a0) * i / 10
        r0, r1 = (s / 2 - 30, s / 2 - 16) if i % 5 == 0 else (s / 2 - 26, s / 2 - 16)
        d.line((cx + r0 * math.cos(a), cy - r0 * math.sin(a), cx + r1 * math.cos(a), cy - r1 * math.sin(a)),
               fill=(20, 20, 20), width=4)
        if i % 2 == 0:
            txt = str(i)
            tw = d.textlength(txt, font=font(20))
            d.text((cx + (s / 2 - 50) * math.cos(a) - tw / 2, cy - (s / 2 - 50) * math.sin(a) - 11), txt,
                   font=font(20), fill=(20, 20, 20))
    lw = d.textlength(label, font=font(20))
    d.text((cx - lw / 2, cy + 40), label, font=font(20), fill=(20, 20, 20))
    a = a0 + (a1 - a0) * frac
    d.line((cx, cy, cx + (s / 2 - 36) * math.cos(a), cy - (s / 2 - 36) * math.sin(a)), fill=(190, 20, 20), width=6)
    d.ellipse((cx - 10, cy - 10, cx + 10, cy + 10), fill=(20, 20, 20))
    img.save(os.path.join(TEX_DIR, fname))


def hazard(fname):
    img = Image.new("RGB", (256, 64), (235, 180, 10))
    d = ImageDraw.Draw(img)
    for x in range(-64, 320, 48):
        d.polygon([(x, 64), (x + 24, 64), (x + 88, 0), (x + 64, 0)], fill=(25, 25, 25))
    img.save(os.path.join(TEX_DIR, fname))


sign("sign_t1.png", "T-1", "CRUDE FEED  120 m3")
sign("sign_t2.png", "T-2", "CRUDE FEED  120 m3")
sign("sign_p101.png", "P-101", "FEED PUMP")
sign("sign_e3.png", "E-3", "MOTOR CONTROL  400 V", band=(170, 30, 20))
sign("sign_v7.png", "V-7", "MANIFOLD")
sign("sign_dock.png", "DOCK", "ROBOT CHARGING", band=(20, 90, 170))
sign("sign_cr.png", "CONTROL", "OPERATORS ONLY", band=(20, 90, 170))
gauge("gauge_p101.png", 0.48, "bar")
gauge("gauge_level.png", 0.62, "level x10%")
hazard("hazard.png")

# ── Header ──────────────────────────────────────────────────────────────
custom = {
    "places": {n: [p[0], p[1], p[2], p[3], p[4]] for n, p in PLACES.items()},
    "walkway": {"nodes": {n: list(v) for n, v in NODES.items()}, "edges": [list(e) for e in EDGES]},
    "bounds": [BOUNDS[0] + 0.8, BOUNDS[1] - 0.8, BOUNDS[2] + 0.8, BOUNDS[3] - 0.8],
    "home": "charging dock",
}
custom_s = json.dumps(custom, separators=(",", ":")).replace('"', '\\"')

w("""#OMNISIM R2025a utf8
# Deep Robotics X30 on an inspection round of a process-plant yard, driven by
# OmniLink. The legs are REAL contact physics: the bridge's --locomotion trot
# (omnilink_quadruped_bridge/_crawl_motion.py) plants feet and the body moves
# because they push -- no supervisor pin, no policy. Navigation uses the
# robot's pose from the Supervisor (standing in for leg odometry + IMU) and the
# walkway graph declared in customData; nothing senses obstacles. Scripted
# gait, flat ground only: say so wherever this world's footage is shown.
#
# Talk to it: right-click the robot -> Show Robot Window (needs an OmniKey),
# or POST /prompt {"text": "go to pump P-101"} to the bridge on local port %d.
# The scene and the places in customData were generated from ONE site table
# and share coordinates: move a piece of equipment, move its place with it.
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/RoughConcrete.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/FormedConcrete.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/GalvanizedMetal.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/MetalPipePaint.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/CorrugatedMetal.proto"
EXTERNPROTO "omnisim://projects/objects/obstacles/protos/OilBarrel.proto"
EXTERNPROTO "omnisim://projects/objects/freight/protos/IntermodalContainer.proto"
EXTERNPROTO "omnisim://projects/objects/freight/protos/IntermodalOfficeContainer.proto"
EXTERNPROTO "omnisim://projects/objects/factory/forklift/protos/Forklift.proto"
EXTERNPROTO "omnisim://projects/objects/factory/pallet/protos/WoodenPalletStack.proto"
EXTERNPROTO "omnisim://projects/objects/street_furniture/protos/ElectricalCabinet.proto"
EXTERNPROTO "omnisim://projects/objects/street_furniture/protos/Fence.proto"
EXTERNPROTO "omnisim://projects/objects/street_furniture/protos/FireHydrant.proto"
WorldInfo {
  basicTimeStep 8
  title "OmniLink -- X30 plant inspection"
  info [
    "Deep Robotics X30 inspection round, driven through OmniLink. Real contact-physics legs (scripted trot), flat ground."
  ]
  # The stiff-PD quadruped stance recipe (projects/robots/deep_robotics/PROVENANCE.md).
  newtonGroundMu 2
  newtonSubsteps 8
  newtonCompoundColliders TRUE
}
Viewpoint {
  orientation -0.278945 0.120681 0.952694 2.359689
  position 1.570 -15.852 10.388
  exposure 1.5
  follow "x30"
  followType "Tracking Shot"
  followSmoothness 0.4
}
OmniSimSky { }
DEF SUN OmniSimSun { intensity 3.2 }
# Low late-afternoon sun from the south-west, far behind the usual camera:
# long shadows, warm light, and the marker orb stays out of frame.
DEF SUN_MARKER OmniSimSunMarker { translation -30 -38 40 }
""" % PORT)

# ── Ground ──────────────────────────────────────────────────────────────
gx, gy = (BOUNDS[1] - BOUNDS[0]) + 6, (BOUNDS[3] - BOUNDS[2]) + 6
cx, cy = (BOUNDS[0] + BOUNDS[1]) / 2, (BOUNDS[2] + BOUNDS[3]) / 2
solid("yard slab", [
    shape("Box { size %s }" % fmt(gx, gy, 0.4),
          "RoughConcrete { textureTransform TextureTransform { scale %s } }" % fmt(gx / 1.2, gy / 1.2),
          (0, 0, 0))], t=(cx, cy, -0.2), bo="Box { size %s }" % fmt(gx, gy, 0.4))
# a darker asphalt apron outside the fence
visual([shape("Box { size %s }" % fmt(80, 80, 0.02), pbr((0.13, 0.13, 0.14), 0.95), (0, 0, -0.012))], t=(cx, cy, 0))

# ── Walkway: green epoxy + yellow edge lines (paint, no collider) ───────
# The walkway is axis-aligned: every edge runs E-W or N-S. Each node is a
# WALK_W square; each edge fills the gap between its two squares. Edge lines
# run along the edges only, and a node square gets a line on each side that
# has no walkway leaving it -- so the outline is the union's outline, with
# no line crossing a junction.
paint = []
H = WALK_W / 2
LW = 0.1                                    # line width
Z_FILL, Z_LINE, T = 0.002, 0.0045, 0.004    # paint thickness 4 mm, lines 5 mm
links = {n: set() for n in NODES}
for a, b in EDGES:
    (x0, y0), (x1, y1) = NODES[a], NODES[b]
    if abs(y1 - y0) < 1e-6:                 # E-W
        links[a].add("E" if x1 > x0 else "W"); links[b].add("W" if x1 > x0 else "E")
        lo, hi = sorted((x0, x1))
        L = hi - lo - WALK_W
        mx = (lo + hi) / 2
        paint.append(shape("Box { size %s }" % fmt(L + 0.002, WALK_W, T), WALK_GREEN, (mx, y0, Z_FILL)))
        for side in (-1, 1):
            paint.append(shape("Box { size %s }" % fmt(L + 0.002, LW, T + 0.001), YELLOW, (mx, y0 + side * (H - LW / 2), Z_LINE)))
    else:                                   # N-S
        links[a].add("N" if y1 > y0 else "S"); links[b].add("S" if y1 > y0 else "N")
        lo, hi = sorted((y0, y1))
        L = hi - lo - WALK_W
        my = (lo + hi) / 2
        paint.append(shape("Box { size %s }" % fmt(WALK_W, L + 0.002, T), WALK_GREEN, (x0, my, Z_FILL)))
        for side in (-1, 1):
            paint.append(shape("Box { size %s }" % fmt(LW, L + 0.002, T + 0.001), YELLOW, (x0 + side * (H - LW / 2), my, Z_LINE)))
for n, (x, y) in NODES.items():
    paint.append(shape("Box { size %s }" % fmt(WALK_W, WALK_W, T), WALK_GREEN, (x, y, Z_FILL)))
    for d, (ox, oy, sx, sy) in {"E": (H - LW / 2, 0, LW, WALK_W), "W": (-H + LW / 2, 0, LW, WALK_W),
                                "N": (0, H - LW / 2, WALK_W, LW), "S": (0, -H + LW / 2, WALK_W, LW)}.items():
        if d not in links[n]:
            paint.append(shape("Box { size %s }" % fmt(sx, sy, T + 0.001), YELLOW, (x + ox, y + oy, Z_LINE)))
visual(paint)

# ── Charging dock ───────────────────────────────────────────────────────
dx, dy = DOCK[0], DOCK[1]
solid("charging dock", [
    shape("Box { size 1.5 1.2 0.008 }", pbr((0.20, 0.21, 0.23), 0.5, 0.3), (0, 0, 0.004)),
    shape("Box { size 1.5 0.08 0.009 }", YELLOW, (0, 0.56, 0.0045)),
    shape("Box { size 1.5 0.08 0.009 }", YELLOW, (0, -0.56, 0.0045)),
    shape("Box { size 0.25 0.9 1.1 }", pbr((0.85, 0.86, 0.88), 0.35, 0.2), (-0.95, 0, 0.55)),
    shape("Box { size 0.02 0.6 0.04 }", pbr((0.05, 0.9, 0.4), 0.3, 0, emissive=(0.05, 0.9, 0.4)), (-0.82, 0, 0.9)),
    shape("Box { size 0.01 0.8 0.4 }", pbr((1, 1, 1), 0.6, tex="sign_dock.png"), (-0.815, 0, 0.55), (0, 0, 1, 0)),
], t=(dx - 0.1, dy, 0), bo="Pose { translation -0.95 0 0.55 children [ Box { size 0.25 0.9 1.1 } ] }")

# ── Tank farm: bund wall, two tanks, gauges, ladders, signs ─────────────
bx0, bx1, by0, by1 = -6.2, 6.2, 5.9, 10.4
bund = []
for (x, y, sx, sy) in ((0, by0, bx1 - bx0, 0.3), (0, by1, bx1 - bx0, 0.3),
                       (bx0, (by0 + by1) / 2, 0.3, by1 - by0), (bx1, (by0 + by1) / 2, 0.3, by1 - by0)):
    bund.append(("Box { size %s }" % fmt(sx, sy, 0.45), (x, y, 0.225)))
solid("tank bund", [shape(g, "FormedConcrete { }", t) for g, t in bund] + [
    shape("Box { size %s }" % fmt(bx1 - bx0, 0.31, 0.08),
          pbr((1, 1, 1), 0.6, tex="hazard.png"), (0, by0, 0.43))],
      bo="Group { children [ %s ] }" % " ".join(
          "Pose { translation %s children [ %s ] }" % (fmt(*t), g) for g, t in bund))

for tid, tx in (("t1", -3.5), ("t2", 3.5)):
    ty, R, H = 8.3, 1.6, 5.2
    kids = [
        shape("Cylinder { radius %g height %g subdivision 48 }" % (R, H), TANK_WHITE, (0, 0, H / 2)),
        shape("Cone { bottomRadius %g height 0.5 subdivision 48 }" % (R + 0.02), TANK_WHITE, (0, 0, H + 0.25)),
        shape("Cylinder { radius %g height 0.12 subdivision 48 }" % (R + 0.03), pbr((0.5, 0.52, 0.54), 0.5, 0.5), (0, 0, 0.06)),
        # sign plate on the tank face (toward -y, the walkway)
        shape("Box { size 1.0 0.02 0.5 }", pbr((1, 1, 1), 0.6, tex="sign_%s.png" % tid), (0, -R - 0.01, 2.2)),
        # level gauge: dial on a bracket at eye height for the robot's cameras
        shape("Box { size 0.08 0.25 0.08 }", STEEL, (0.9, -R - 0.1, 0.95)),
        shape("Cylinder { radius 0.16 height 0.06 subdivision 32 }", pbr((0.25, 0.25, 0.27), 0.4, 0.6), (0.9, -R - 0.24, 0.95), (1, 0, 0, 1.5708)),
        shape("Box { size 0.3 0.005 0.3 }", pbr((1, 1, 1), 0.4, tex="gauge_level.png"), (0.9, -R - 0.275, 0.95)),
    ]
    # caged ladder up the side
    for k in range(18):
        kids.append(shape("Box { size 0.45 0.03 0.03 }", STEEL, (-0.9, -R - 0.12, 0.3 + k * 0.28)))
    for sx in (-1.12, -0.68):
        kids.append(shape("Box { size 0.04 0.04 %g }" % (H + 0.4), STEEL, (sx, -R - 0.12, (H + 0.4) / 2)))
    solid("tank %s" % tid.upper().replace("T", "T-"), kids, t=(tx, ty, 0),
          bo="Pose { translation 0 0 %g children [ Box { size %g %g %g } ] }" % (H / 2, 2 * R, 2 * R, H))

# pipe from each tank bottom west along the bund to the rack (inside the bund)
visual([shape("Cylinder { radius 0.11 height 10.6 subdivision 20 }", PIPE_GREY, (-5.3, 9.9, 0.55), (0, 1, 0, 1.5708))])

# ── Pipe rack along the west fence, with the V-7 manifold ───────────────
rx, rz = -11.0, 3.2
rack = []
for yy in (-3.0, 0.5, 4.0, 7.5):
    for ox in (-0.55, 0.55):
        rack.append(shape("Box { size 0.2 0.2 %g }" % rz, "MetalPipePaint { }", (ox, yy, rz / 2)))
    rack.append(shape("Box { size 1.4 0.2 0.2 }", "MetalPipePaint { }", (0, yy, rz)))
for i, (ox, r, col) in enumerate(((-0.35, 0.14, (0.55, 0.57, 0.58)), (0.0, 0.1, (0.85, 0.7, 0.1)), (0.33, 0.12, (0.25, 0.45, 0.3)))):
    rack.append(shape("Cylinder { radius %g height 11.6 subdivision 20 }" % r, pbr(col, 0.45, 0.5), (ox, 2.25, rz + 0.1 + r), (1, 0, 0, 1.5708)))
# drop to the manifold at y = 1.0 and the header with three valves
my = 1.0
rack += [
    shape("Cylinder { radius 0.1 height 2.3 subdivision 20 }", PIPE_GREY, (0.35, my, 2.2)),
    shape("Cylinder { radius 0.12 height 1.8 subdivision 20 }", PIPE_GREY, (0.35, my, 1.05), (1, 0, 0, 1.5708)),
]
for k, dy_ in enumerate((-0.6, 0.0, 0.6)):
    rack += [
        shape("Box { size 0.26 0.2 0.26 }", RED, (0.35, my + dy_, 1.05)),
        shape("Cylinder { radius 0.03 height 0.35 subdivision 12 }", STEEL, (0.62, my + dy_, 1.05), (0, 1, 0, 1.5708)),
        shape("Cylinder { radius 0.16 height 0.03 subdivision 24 }", pbr((0.85, 0.15, 0.1), 0.4), (0.8, my + dy_, 1.05), (0, 1, 0, 1.5708)),
    ]
rack.append(shape("Box { size 0.02 0.8 0.4 }", pbr((1, 1, 1), 0.6, tex="sign_v7.png"), (0.66, my, 1.75)))
solid("pipe rack", rack, t=(rx, 0, 0),
      bo="Group { children [ %s ] }" % (" ".join(
          "Pose { translation %s children [ Box { size 0.2 0.2 %g } ] }" % (fmt(ox, yy, rz / 2), rz)
          for yy in (-3.0, 0.5, 4.0, 7.5) for ox in (-0.55, 0.55))
          + " Pose { translation 0.35 1 1.05 children [ Box { size 0.3 1.8 0.5 } ] }"))

# pipe bridge over the north walkway feeding the tanks (2.9 m clear)
visual([
    shape("Cylinder { radius 0.11 height 4.2 subdivision 20 }", PIPE_GREY, (-9.0, 9.9, 3.3), (0, 1, 0, 1.5708)),
    shape("Cylinder { radius 0.11 height 2.75 subdivision 20 }", PIPE_GREY, (-6.9, 9.9, 1.93)),
])

# ── Pump P-101 skid ─────────────────────────────────────────────────────
px, py = 2.5, -7.4
pump = [
    shape("Box { size 2.4 1.1 0.18 }", pbr((0.93, 0.72, 0.05), 0.5, 0.2), (0, 0, 0.09)),
    shape("Cylinder { radius 0.28 height 0.8 subdivision 32 }", pbr((0.1, 0.25, 0.55), 0.35, 0.3), (-0.55, 0, 0.5), (0, 1, 0, 1.5708)),
    shape("Box { size 0.5 0.45 0.12 }", pbr((0.1, 0.25, 0.55), 0.35, 0.3), (-0.55, 0, 0.24)),
    shape("Cylinder { radius 0.07 height 0.35 subdivision 16 }", STEEL, (-0.05, 0, 0.5), (0, 1, 0, 1.5708)),
    shape("Cylinder { radius 0.32 height 0.3 subdivision 32 }", pbr((0.45, 0.47, 0.5), 0.4, 0.6), (0.4, 0, 0.5), (0, 1, 0, 1.5708)),
    shape("Cylinder { radius 0.11 height 0.7 subdivision 20 }", PIPE_GREY, (0.95, 0, 0.5), (0, 1, 0, 1.5708)),
    # discharge riser toward the walkway with the gauge
    shape("Cylinder { radius 0.09 height 2.2 subdivision 20 }", PIPE_GREY, (0.4, 0.45, 1.2)),
    shape("Cylinder { radius 0.09 height 0.5 subdivision 20 }", PIPE_GREY, (0.4, 0.2, 0.62), (1, 0, 0, 1.5708)),
    shape("Box { size 0.2 0.18 0.2 }", RED, (0.4, 0.45, 0.95)),
    shape("Cylinder { radius 0.02 height 0.18 subdivision 10 }", STEEL, (0.4, 0.58, 1.1), (1, 0, 0, 1.5708)),
    shape("Cylinder { radius 0.13 height 0.06 subdivision 32 }", pbr((0.25, 0.25, 0.27), 0.4, 0.6), (0.4, 0.68, 1.1), (1, 0, 0, 1.5708)),
    shape("Box { size 0.24 0.005 0.24 }", pbr((1, 1, 1), 0.4, tex="gauge_p101.png"), (0.4, 0.712, 1.1), (0, 0, 1, math.pi)),
    shape("Box { size 0.9 0.02 0.45 }", pbr((1, 1, 1), 0.6, tex="sign_p101.png"), (-0.55, 0.56, 0.95), (0, 0, 1, math.pi)),
    shape("Box { size 0.05 0.05 0.9 }", STEEL, (-0.95, 0.56, 0.45)),
    shape("Box { size 0.05 0.05 0.9 }", STEEL, (-0.15, 0.56, 0.45)),
]
solid("pump P-101", pump, t=(px, py, 0),
      bo="Group { children [ Pose { translation 0 0 0.45 children [ Box { size 2.4 1.1 0.9 } ] } ] }")

# ── Motor-control cabinets E-1..E-4 on the east side ─────────────────────
for k, yy in enumerate((-2.1, -0.7, 0.7, 2.1)):
    w("ElectricalCabinet { translation %s rotation 0 0 1 3.1416 name \"cabinet E-%d\" }" % (fmt(10.3, yy, 0), k + 1))
visual([
    shape("Box { size 0.8 5.6 0.25 }", "FormedConcrete { }", (10.4, 0, -0.09)),
    shape("Box { size 1.4 6.0 0.08 }", "CorrugatedMetal { }", (10.35, 0, 2.6)),
    shape("Box { size 0.08 0.08 2.6 }", STEEL, (9.75, -2.9, 1.3)),
    shape("Box { size 0.08 0.08 2.6 }", STEEL, (9.75, 2.9, 1.3)),
    shape("Box { size 0.02 0.8 0.4 }", pbr((1, 1, 1), 0.6, tex="sign_e3.png"), (9.92, 0.0, 1.55), (0, 0, 1, 0)),
])

# ── Control room, containers, forklift, pallets, barrels ─────────────────
w("IntermodalOfficeContainer { translation %s rotation 0 0 1 0 name \"control room\" primaryColor 0.86 0.86 0.82 secondaryColor 0.3 0.32 0.35 }" % fmt(-4.5, -7.6, 0))
visual([shape("Box { size 0.02 1.0 0.5 }", pbr((1, 1, 1), 0.6, tex="sign_cr.png"), (0, 0, 0), (0, 0, 1, -1.5708))], t=(-4.5, -6.37, 2.1))
w("IntermodalContainer { translation %s rotation 0 0 1 1.5708 name \"container A\" color 0.08 0.24 0.55 }" % fmt(10.4, -6.2, 0))
w("IntermodalContainer { translation %s rotation 0 0 1 1.5708 name \"container B\" color 0.8 0.35 0.07 }" % fmt(10.4, -6.2, 2.6))
w("Forklift { translation %s rotation 0 0 1 2.6 name \"forklift\" enablePhysics FALSE }" % fmt(6.2, -7.7, 0.81))
for k, (x, y) in enumerate(((-1.2, -7.6), (-0.1, -7.9))):
    w("WoodenPalletStack { translation %s rotation 0 0 1 %g name \"pallets %d\" palletNumber %d }" % (fmt(x, y, 0), 0.1 * k, k, 6 + 3 * k))
for k, (x, y) in enumerate(((5.2, 6.7), (5.9, 6.8), (5.5, 7.4), (-5.4, 7.0), (-5.6, 7.7))):
    w("OilBarrel { translation %s name \"barrel %d\" }" % (fmt(x, y, 0.44), k))
w("FireHydrant { translation %s name \"hydrant\" }" % fmt(-2.2, -6.4, 0))

# ── Perimeter fence and light masts ─────────────────────────────────────
x0, x1, y0, y1 = BOUNDS
w("Fence { translation 0 0 0 name \"perimeter\" height 2.2 poleGap 2.5 horizontalBarsNumber 3 path [ %s ] }" % ", ".join(
    fmt(*p) for p in ((x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0), (x0, y0, 0))))
for (x, y) in ((-11.5, -8.5), (11.5, 10.5), (11.5, -8.5), (-11.5, 10.5)):
    visual([
        shape("Cylinder { radius 0.09 height 7 subdivision 16 }", STEEL, (0, 0, 3.5)),
        shape("Box { size 0.7 0.35 0.2 }", pbr((0.2, 0.2, 0.22), 0.5, 0.4), (0, 0, 7.05)),
    ], t=(x, y, 0))

# ── The robot ───────────────────────────────────────────────────────────
w("""DEF X30 URDFRobot {
  url "../../../../robots/deep_robotics/x30/urdf/x30.urdf"
  translation %s
  rotation 0 0 1 %g
  name "x30"
  supervisor TRUE
  controller "omnilink_quadruped_bridge"
  controllerArgs [ "--robot" "x30" "--port" "%d" "--locomotion" "trot" ]
  window "omnilink_chat"
  customData "%s"
}""" % (fmt(DOCK[0], DOCK[1], 0.43), math.radians(DOCK[2]), PORT, custom_s))

open(OUT, "w", encoding="utf-8").write("\n".join(out) + "\n")
print("wrote", OUT, "and", sum(f.endswith(".png") for f in os.listdir(TEX_DIR)), "textures")
