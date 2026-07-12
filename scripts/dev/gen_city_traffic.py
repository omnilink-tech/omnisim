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

"""Generate the living-city world showcase/city_traffic.wbt for an NxN street grid,
plus controllers/city_traffic/city_grid.json (read by the city_traffic supervisor so
the world and controller can never drift apart).

The city is wall-to-wall mixed-use: each block is a perimeter of building masses
lining the streets edge-to-edge, with ground-floor shopfronts. ~1/4 of the units are
RESTAURANTS with outdoor cafe seating (umbrella + table + chairs + a diner) on the
sidewalk; the rest are SHOPS (lit display window + sign board). Every junction has
zebra crossings + traffic signals + a street-name sign; speed-limit signs line the
roads. Cars route the grid and pedestrians walk the block-perimeter sidewalks (both
driven by the city_traffic supervisor).

Edit the GRID PARAMS below and re-run:  python scripts/dev/gen_city_traffic.py
"""
import json
import math
import os
import random

# ---------------------------------------------------------------------------
# GRID PARAMS
# ---------------------------------------------------------------------------
XS = [-93.0, -31.0, 31.0, 93.0]     # N-S road x-positions (cols)
YS = [-93.0, -31.0, 31.0, 93.0]     # E-W road y-positions (rows)
STUB = 40.0
ROADW = 10.0
XR = 6.0          # crossroad (junction) half-extent
LANE = 2.6
H = 8.0           # stop-line distance from junction centre
CRUISE = 7.0
CARS_PER_DIR = 3
GROUND = 320.0
SIDEWALK = 8.0    # building line = road centreline +/- SIDEWALK
SWALK_C = 6.5     # sidewalk centreline = road centreline +/- SWALK_C (pedestrian path)
UNIT_W = 24.0
PEDS_PER_LOOP = 4
SEED = 7

MATS = {
    "brick":  "PBRAppearance { baseColor 0.46 0.22 0.17 roughness 0.92 metalness 0 }",
    "brick2": "PBRAppearance { baseColor 0.55 0.34 0.27 roughness 0.9 metalness 0 }",
    "conc":   "PBRAppearance { baseColor 0.62 0.62 0.60 roughness 0.9 metalness 0 }",
    "tan":    "PBRAppearance { baseColor 0.80 0.72 0.55 roughness 0.85 metalness 0 }",
    "stone":  "PBRAppearance { baseColor 0.80 0.80 0.78 roughness 0.8 metalness 0 }",
    "glass":  "PBRAppearance { baseColor 0.34 0.46 0.55 roughness 0.12 metalness 0.75 }",
    "glass2": "PBRAppearance { baseColor 0.30 0.40 0.38 roughness 0.14 metalness 0.7 }",
    "dark":   "PBRAppearance { baseColor 0.26 0.28 0.33 roughness 0.4 metalness 0.5 }",
    "shop":   "PBRAppearance { baseColor 0.55 0.68 0.74 roughness 0.08 metalness 0.5 emissiveColor 0.22 0.27 0.30 }",
    "win":    "PBRAppearance { baseColor 0.20 0.28 0.34 roughness 0.15 metalness 0.6 emissiveColor 0.02 0.03 0.04 }",
    "roof":   "PBRAppearance { baseColor 0.18 0.18 0.20 roughness 0.8 metalness 0.2 }",
    "spand":  "PBRAppearance { baseColor 0.22 0.24 0.27 roughness 0.5 metalness 0.3 }",
    "awn_r":  "PBRAppearance { baseColor 0.72 0.12 0.12 roughness 0.6 emissiveColor 0.15 0.02 0.02 }",
    "awn_g":  "PBRAppearance { baseColor 0.10 0.45 0.22 roughness 0.6 emissiveColor 0.02 0.10 0.05 }",
    "awn_b":  "PBRAppearance { baseColor 0.12 0.30 0.62 roughness 0.6 emissiveColor 0.02 0.05 0.13 }",
    "awn_o":  "PBRAppearance { baseColor 0.88 0.46 0.10 roughness 0.6 emissiveColor 0.18 0.09 0.02 }",
    "awn_t":  "PBRAppearance { baseColor 0.10 0.50 0.50 roughness 0.6 emissiveColor 0.02 0.10 0.10 }",
    "sign_s": "PBRAppearance { baseColor 0.95 0.82 0.25 roughness 0.5 emissiveColor 0.40 0.32 0.06 }",  # shop sign
    "sign_r": "PBRAppearance { baseColor 0.85 0.16 0.16 roughness 0.5 emissiveColor 0.34 0.04 0.04 }",  # restaurant sign
    "metal":  "PBRAppearance { baseColor 0.72 0.74 0.76 roughness 0.4 metalness 0.7 }",
    "tabletp":"PBRAppearance { baseColor 0.88 0.86 0.82 roughness 0.5 metalness 0.1 }",
    "umb_a":  "PBRAppearance { baseColor 0.82 0.20 0.18 roughness 0.7 }",
    "umb_b":  "PBRAppearance { baseColor 0.16 0.42 0.30 roughness 0.7 }",
    "umb_c":  "PBRAppearance { baseColor 0.92 0.88 0.78 roughness 0.7 }",
    "umb_d":  "PBRAppearance { baseColor 0.20 0.36 0.62 roughness 0.7 }",
    "skin":   "PBRAppearance { baseColor 0.80 0.62 0.50 roughness 0.7 }",
    "shirt":  "PBRAppearance { baseColor 0.30 0.42 0.62 roughness 0.85 }",
    "shirt2": "PBRAppearance { baseColor 0.55 0.20 0.20 roughness 0.85 }",
    "stsign": "PBRAppearance { baseColor 0.10 0.45 0.20 roughness 0.6 emissiveColor 0.02 0.10 0.05 }",  # street name plate
    "ped_skin": "PBRAppearance { baseColor 0.80 0.62 0.50 roughness 0.7 metalness 0 }",
    "ped_shoe": "PBRAppearance { baseColor 0.12 0.12 0.14 roughness 0.7 metalness 0 }",
    "bus_body": "PBRAppearance { baseColor 0.74 0.16 0.13 roughness 0.4 metalness 0.3 }",
    "bus_glass": "PBRAppearance { baseColor 0.10 0.13 0.16 roughness 0.1 metalness 0.6 }",
    "bus_trim": "PBRAppearance { baseColor 0.86 0.86 0.87 roughness 0.5 metalness 0.2 }",
    "bus_sign": "PBRAppearance { baseColor 0.95 0.85 0.30 roughness 0.4 emissiveColor 0.5 0.42 0.10 }",
    "tire": "PBRAppearance { baseColor 0.05 0.05 0.06 roughness 0.9 metalness 0 }",
}
WALLS_LOW = ["brick", "brick2", "conc", "tan", "stone"]
WALLS_TOWER = ["glass", "glass2", "dark"]
SHOP_AWN = ["awn_b", "awn_g", "awn_t"]
REST_AWN = ["awn_r", "awn_o"]
UMBRELLAS = ["umb_a", "umb_b", "umb_c", "umb_d"]
PED_SHIRTS = [(0.30, 0.42, 0.62), (0.62, 0.20, 0.20), (0.20, 0.45, 0.28), (0.80, 0.80, 0.82),
              (0.85, 0.72, 0.20), (0.16, 0.40, 0.46), (0.55, 0.35, 0.62)]
PED_PANTS = [(0.18, 0.20, 0.28), (0.30, 0.30, 0.32), (0.45, 0.38, 0.26), (0.10, 0.10, 0.12)]
PED_HAIR = [(0.10, 0.08, 0.06), (0.28, 0.18, 0.10), (0.58, 0.44, 0.26), (0.08, 0.08, 0.09), (0.60, 0.60, 0.62)]

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD = os.path.join(REPO, "projects/samples/demos/worlds/showcase/city_traffic.wbt")
SPEC = os.path.join(REPO, "projects/samples/demos/controllers/city_traffic/city_grid.json")


def look_at(eye, target, up=(0, 0, 1)):
    f = [target[i] - eye[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in f)); f = [c / n for c in f]
    d = sum(f[i] * up[i] for i in range(3)); u = [up[i] - d * f[i] for i in range(3)]
    un = math.sqrt(sum(c * c for c in u)); u = [c / un for c in u]
    y = [u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2], u[0] * f[1] - u[1] * f[0]]
    R = [[f[0], y[0], u[0]], [f[1], y[1], u[1]], [f[2], y[2], u[2]]]
    tr = R[0][0] + R[1][1] + R[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    rx, ry, rz = R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]
    s = 2 * math.sin(ang)
    return [rx / s, ry / s, rz / s, ang]


def car_pose(axis, fixed, dr, u):
    if axis == "EW":
        lat = fixed - LANE if dr > 0 else fixed + LANE
        return (u, lat), (0.0 if dr > 0 else 3.14159)
    lat = fixed + LANE if dr > 0 else fixed - LANE
    return (lat, u), (1.5708 if dr > 0 else -1.5708)


def main():
    rng = random.Random(SEED)
    lo_x, hi_x = XS[0] - STUB, XS[-1] + STUB
    lo_y, hi_y = YS[0] - STUB, YS[-1] + STUB
    out = []
    w = out.append
    defined = set()

    def mat(name):
        if name in defined:
            return "USE MAT_%s" % name
        defined.add(name)
        return "DEF MAT_%s %s" % (name, MATS[name])

    # Perf: only the big building masses cast shadows (cast=True). The hundreds of
    # small/flat boxes (overlays, signs, furniture, people) render into the sun's
    # shadow map for no visible gain and dominated the shadow pass.
    def box(cx, cy, cz, sx, sy, sz, m, cast=False):
        w("    Pose { translation %.2f %.2f %.2f children [ Shape { %sappearance %s geometry Box { size %.2f %.2f %.2f } } ] }\n"
          % (cx, cy, cz, "" if cast else "castShadows FALSE ", mat(m), sx, sy, sz))

    def cyl(cx, cy, cz, r, hgt, m):
        w("    Pose { translation %.2f %.2f %.2f rotation 1 0 0 1.5708 children [ Shape { castShadows FALSE appearance %s geometry Cylinder { radius %.2f height %.2f subdivision 10 } } ] }\n"
          % (cx, cy, cz, mat(m), r, hgt))

    def emit_ped(i, x, y, sh, pa, ha, rot=0.0):
        """A low-poly person with DEF'd articulated limbs the supervisor swings to
        walk: DEF PEDi (body) + PEDi_LEGF/LEGB/ARML/ARMR (limbs, rotated each step).
        Peds not listed in the spec's "peds" simply stand here (e.g. bus-stop waiters)."""
        shoe = lambda: mat("ped_shoe")
        skin = lambda: mat("ped_skin")
        w('DEF PED%d Solid { translation %.1f %.1f 0 rotation 0 0 1 %.4f name "ped%d" children [\n' % (i, x, y, rot, i))
        w('  DEF PED%d_LEGF Pose { translation 0 0.085 0.82 children [\n' % i)
        w('    Pose { translation 0 0 -0.34 children [ Shape { castShadows FALSE appearance DEF P%d_PA PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.9 } geometry Capsule { radius 0.072 height 0.5 subdivision 8 } } ] }\n' % (i, pa[0], pa[1], pa[2]))
        w('    Pose { translation 0.05 0 -0.70 children [ Shape { castShadows FALSE appearance %s geometry Box { size 0.23 0.10 0.07 } } ] } ] }\n' % shoe())
        w('  DEF PED%d_LEGB Pose { translation 0 -0.085 0.82 children [\n' % i)
        w('    Pose { translation 0 0 -0.34 children [ Shape { castShadows FALSE appearance USE P%d_PA geometry Capsule { radius 0.072 height 0.5 subdivision 8 } } ] }\n' % i)
        w('    Pose { translation 0.05 0 -0.70 children [ Shape { castShadows FALSE appearance %s geometry Box { size 0.23 0.10 0.07 } } ] } ] }\n' % shoe())
        w('  Pose { translation 0 0 0.86 children [ Shape { castShadows FALSE appearance USE P%d_PA geometry Box { size 0.30 0.21 0.18 } } ] }\n' % i)
        w('  Pose { translation 0 0 1.14 children [ Shape { castShadows FALSE appearance DEF P%d_SH PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.85 } geometry Box { size 0.33 0.20 0.5 } } ] }\n' % (i, sh[0], sh[1], sh[2]))
        w('  Pose { translation 0 0 1.38 rotation 1 0 0 1.5708 children [ Shape { castShadows FALSE appearance USE P%d_SH geometry Capsule { radius 0.085 height 0.26 subdivision 8 } } ] }\n' % i)
        w('  DEF PED%d_ARML Pose { translation 0 0.2 1.37 children [\n' % i)
        w('    Pose { translation 0 0 -0.22 children [ Shape { castShadows FALSE appearance USE P%d_SH geometry Capsule { radius 0.046 height 0.34 subdivision 8 } } ] }\n' % i)
        w('    Pose { translation 0 0 -0.45 children [ Shape { castShadows FALSE appearance %s geometry Sphere { radius 0.05 subdivision 2 } } ] } ] }\n' % skin())
        w('  DEF PED%d_ARMR Pose { translation 0 -0.2 1.37 children [\n' % i)
        w('    Pose { translation 0 0 -0.22 children [ Shape { castShadows FALSE appearance USE P%d_SH geometry Capsule { radius 0.046 height 0.34 subdivision 8 } } ] }\n' % i)
        w('    Pose { translation 0 0 -0.45 children [ Shape { castShadows FALSE appearance %s geometry Sphere { radius 0.05 subdivision 2 } } ] } ] }\n' % skin())
        w('  Pose { translation 0 0 1.48 children [ Shape { castShadows FALSE appearance %s geometry Cylinder { radius 0.046 height 0.08 } } ] }\n' % skin())
        w('  Pose { translation 0.01 0 1.60 children [ Shape { castShadows FALSE appearance %s geometry Sphere { radius 0.112 subdivision 3 } } ] }\n' % skin())
        w('  Pose { translation -0.03 0 1.66 children [ Shape { castShadows FALSE appearance DEF P%d_HA PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.85 } geometry Sphere { radius 0.105 subdivision 2 } } ] }\n' % (i, ha[0], ha[1], ha[2]))
        w('] }\n')

    def cafe(fx, fy, ax, ay, ox, oy):
        """Outdoor cafe seating on the sidewalk in front of a restaurant unit:
        umbrella + table + two chairs + a seated diner. (ox,oy)=outward (toward road)."""
        bx, by = fx + ox * 1.2, fy + oy * 1.2          # cafe centre, ~1.2 m out on the sidewalk
        cyl(bx, by, 1.15, 0.04, 2.3, "metal")          # umbrella pole
        box(bx, by, 2.35, 1.9, 1.9, 0.08, rng.choice(UMBRELLAS))  # umbrella canopy
        cyl(bx, by, 0.36, 0.05, 0.72, "metal")         # table leg
        cyl(bx, by, 0.73, 0.42, 0.05, "tabletp")       # table top
        for s in (-1, 1):                              # two chairs flanking along the frontage
            chx, chy = bx + ax * 0.7 * s, by + ay * 0.7 * s
            box(chx, chy, 0.45, 0.40, 0.40, 0.06, "metal")
            box(chx - ox * 0.18, chy - oy * 0.18, 0.70, 0.40, 0.06, 0.45, "metal")  # chair back
        # one seated diner at a chair
        dx, dy = bx + ax * 0.7, by + ay * 0.7
        box(dx, dy, 0.92, 0.30, 0.32, 0.46, rng.choice(["shirt", "shirt2"]))
        w("    Pose { translation %.2f %.2f 1.22 children [ Shape { castShadows FALSE appearance %s geometry Sphere { radius 0.12 subdivision 2 } } ] }\n"
          % (dx, dy, mat("skin")))

    def seated(i, x, y, rot, sh, pa, ha):
        """A person sitting on a bench, facing +X (local). Static."""
        w('Solid { translation %.1f %.1f 0 rotation 0 0 1 %.4f name "sitter_%d" children [\n' % (x, y, rot, i))
        w('  Pose { translation 0.06 0 0.47 children [ Shape { castShadows FALSE appearance DEF S%d_PA PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.9 } geometry Box { size 0.42 0.34 0.14 } } ] }\n' % (i, pa[0], pa[1], pa[2]))
        w('  Pose { translation 0.24 0 0.24 children [ Shape { castShadows FALSE appearance USE S%d_PA geometry Box { size 0.14 0.34 0.46 } } ] }\n' % i)
        w('  Pose { translation -0.08 0 0.78 children [ Shape { castShadows FALSE appearance DEF S%d_SH PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.85 } geometry Box { size 0.30 0.34 0.5 } } ] }\n' % (i, sh[0], sh[1], sh[2]))
        w('  Pose { translation -0.06 0 1.07 children [ Shape { castShadows FALSE appearance %s geometry Sphere { radius 0.115 subdivision 3 } } ] }\n' % mat("ped_skin"))
        w('  Pose { translation -0.09 0 1.12 children [ Shape { castShadows FALSE appearance DEF S%d_HA PBRAppearance { baseColor %.2f %.2f %.2f roughness 0.85 } geometry Sphere { radius 0.1 subdivision 2 } } ] }\n] }\n' % (i, ha[0], ha[1], ha[2]))

    def unit(cx, cy, ax, ay, ox, oy, uw, depth, h):
        along = uw * 0.97
        sx = abs(ax) * along + abs(ox) * depth
        sy = abs(ay) * along + abs(oy) * depth
        tower = h > 33.0
        wall = rng.choice(WALLS_TOWER if tower else WALLS_LOW)
        box(cx, cy, h / 2.0, sx, sy, h, wall, cast=True)
        box(cx, cy, h + 0.12, sx + 0.5, sy + 0.5, 0.3, "roof")
        fx, fy = cx + ox * depth / 2.0, cy + oy * depth / 2.0
        ssx = abs(ax) * uw * 0.9 + abs(ox) * 0.4
        ssy = abs(ay) * uw * 0.9 + abs(oy) * 0.4
        box(fx + ox * 0.2, fy + oy * 0.2, 1.7, ssx, ssy, 3.0, "shop")          # ground-floor display window
        if tower:
            for fl in range(1, 4):
                box(cx, cy, h * fl / 4.0, sx + 0.3, sy + 0.3, 0.5, "spand")
            return
        restaurant = rng.random() < 0.28
        # sign board over the shopfront
        sgx = abs(ax) * uw * 0.6 + abs(ox) * 0.3
        sgy = abs(ay) * uw * 0.6 + abs(oy) * 0.3
        box(fx + ox * 0.45, fy + oy * 0.45, 3.95, sgx, sgy, 0.7, "sign_r" if restaurant else "sign_s")
        # awning
        asx = abs(ax) * uw * 0.88 + abs(ox) * 1.5
        asy = abs(ay) * uw * 0.88 + abs(oy) * 1.5
        box(fx + ox * 0.85, fy + oy * 0.85, 3.4, asx, asy, 0.16, rng.choice(REST_AWN if restaurant else SHOP_AWN))
        if h > 9.0:
            for s in (-1, 1):
                off = s * uw * 0.26
                wsx = abs(ax) * 1.7 + abs(ox) * 0.2
                wsy = abs(ay) * 1.7 + abs(oy) * 0.2
                box(fx + ax * off + ox * 0.06, fy + ay * off + oy * 0.06, (3.8 + h) / 2.0, wsx, wsy, h - 4.2, "win")
        if restaurant:
            cafe(fx, fy, ax, ay, ox, oy)

    def side(x0, y0, x1, y1, ox, oy):
        seglen = math.hypot(x1 - x0, y1 - y0)
        if seglen < 7.0:
            return
        n = max(1, int(round(seglen / UNIT_W)))
        uw = seglen / n
        ax, ay = (x1 - x0) / seglen, (y1 - y0) / seglen
        for k in range(n):
            t = (k + 0.5) * uw
            fxw, fyw = x0 + ax * t, y0 + ay * t
            depth = rng.uniform(9.0, 12.0)
            x = rng.random()
            h = rng.uniform(7, 13) if x < 0.55 else (rng.uniform(15, 28) if x < 0.85 else rng.uniform(34, 56))
            unit(fxw - ox * depth / 2.0, fyw - oy * depth / 2.0, ax, ay, ox, oy, uw, depth, h)

    # ---- header / protos ----
    w("#VRML_SIM R2025a utf8\n")
    w("# GENERATED by scripts/dev/gen_city_traffic.py - do not hand-edit; re-run the generator.\n#\n")
    w("# City - Traffic: a living {0}x{1} mixed-use city. Wall-to-wall blocks of shops &\n".format(len(XS), len(YS)))
    w("# restaurants (outdoor cafe seating) + glass towers; cars route the grid and people\n")
    w("# walk the sidewalks; {0} signalised junctions with zebra crossings & road signs.\n".format(len(XS) * len(YS)))
    w("# All driven by the city_traffic supervisor, which reads the grid from city_grid.json.\n\n")
    protos = ["objects/floors/protos/Floor", "appearances/protos/RoughConcrete",
              "objects/road/protos/Road", "objects/road/protos/Crossroad",
              "objects/traffic/protos/PedestrianCrossing", "objects/traffic/protos/SpeedLimitSign",
              "objects/street_furniture/protos/BusStop", "objects/street_furniture/protos/Bench",
              "objects/street_furniture/protos/StoneFountain", "appearances/protos/Grass",
              "objects/buildings/protos/Church", "objects/buildings/protos/Museum",
              "objects/buildings/protos/Auditorium", "objects/buildings/protos/TheThreeTowers",
              "objects/animals/protos/Dog",
              "objects/trees/protos/Oak", "samples/demos/protos/Car",
              "objects/backgrounds/protos/OmniSimSky", "objects/lights/protos/OmniSimSun",
              "objects/lights/protos/OmniSimSunMarker"]
    for p in protos:
        w('EXTERNPROTO "omnisim://projects/{0}.proto"\n'.format(p))

    # Perf: 32 ms steps (kinematic world; the supervisor scales all motion by dt)
    # + 30 FPS render cap = half the per-second supervisor/IPC and render cost.
    w('\nWorldInfo {\n  basicTimeStep 32\n  FPS 30\n  title "City - Traffic"\n  info [\n')
    w('    "A living {0}x{1} mixed-use city: wall-to-wall shops & restaurants with cafe seating,"\n'.format(len(XS), len(YS)))
    w('    "walking pedestrians, and cars routing the grid under signalised, zebra-crossed junctions."\n  ]\n}\n\n')

    span = max(hi_x - lo_x, hi_y - lo_y)
    eye = (span * 0.62, -span * 0.66, span * 0.58)
    o = look_at(eye, (0, 0, 2))
    w("# High aerial sized to the whole grid, looking NW.\n")
    w("Viewpoint {{\n  orientation {0:.6f} {1:.6f} {2:.6f} {3:.6f}\n".format(*o))
    w("  position {0:.1f} {1:.1f} {2:.1f}\n  fieldOfView 0.9\n  near 0.3\n  ambientOcclusionRadius 2\n  bloomThreshold 6\n}}\n\n".format(*eye))

    w('DEF SKY OmniSimSky { skyColor [ 0.38 0.50 0.66 ] luminosity 1.15 }\n')
    w('DEF SUN OmniSimSun { color 1.0 0.97 0.92 direction -0.32 0.22 -0.92 intensity 4.3 ambientIntensity 0 }\n')
    w('DEF NIGHTFILL DirectionalLight { direction -0.2 -0.2 -1 color 0.40 0.46 0.62 intensity 0 ambientIntensity 0 castShadows FALSE }\n')
    w('DEF SUN_MARKER OmniSimSunMarker { translation 40 -28 110 radius 1.4 baseColor 1.0 0.95 0.85 emissiveColor 1.0 0.90 0.70 emissiveIntensity 150 }\n')
    w('DirectionalLight { direction 0.30 -0.20 -0.55 color 0.55 0.65 0.82 intensity 0.7 ambientIntensity 0 castShadows FALSE }\n')
    w('Fog { color 0.66 0.72 0.80 fogType "EXPONENTIAL" visibilityRange 420 }\n\n')
    w('DEF GROUND Floor {{ translation 0 0 -0.05 name "city_ground" size {0} {0} tileSize 6 6 appearance RoughConcrete {{ }} }}\n\n'.format(GROUND))

    # ---- roads ----
    w("# === Roads ({0} E-W + {1} N-S), segmented at the junctions ===\n".format(len(YS), len(XS)))
    for j, y in enumerate(YS):
        cuts = [lo_x] + [c for xi in XS for c in (xi - XR, xi + XR)] + [hi_x]
        for k in range(0, len(cuts), 2):
            w('Road {{ translation 0 0 0 name "ew{0}_{1}" width {2} numberOfLanes 2 roadBorderWidth [ 3 ] wayPoints [ {3} {4} 0, {5} {4} 0 ] }}\n'.format(j, k // 2, ROADW, cuts[k], y, cuts[k + 1]))
    for i, x in enumerate(XS):
        cuts = [lo_y] + [c for yi in YS for c in (yi - XR, yi + XR)] + [hi_y]
        for k in range(0, len(cuts), 2):
            w('Road {{ translation 0 0 0 name "ns{0}_{1}" width {2} numberOfLanes 2 roadBorderWidth [ 3 ] wayPoints [ {3} {4} 0, {3} {5} 0 ] }}\n'.format(i, k // 2, ROADW, x, cuts[k], cuts[k + 1]))
    w("\n# === Junctions ===\n")
    for i, x in enumerate(XS):
        for j, y in enumerate(YS):
            w('Crossroad {{ translation {0} {1} 0 name "xr_{2}_{3}" shape [ {4} {4} 0, {5} {4} 0, {5} {5} 0, {4} {5} 0 ] }}\n'.format(x, y, i, j, -XR, XR))

    # ---- zebra crossings + road signs ----
    w("\n# === Zebra crossings + road signs at every junction ===\n")
    for i, x in enumerate(XS):
        for j, y in enumerate(YS):
            for nm, px, py, sa, sb in (("n", x, y + 9, ROADW, 3.6), ("s", x, y - 9, ROADW, 3.6),
                                       ("e", x + 9, y, 3.6, ROADW), ("we", x - 9, y, 3.6, ROADW)):
                w('PedestrianCrossing {{ translation {0:.1f} {1:.1f} 0.02 name "zebra_{2}_{3}_{4}" size {5} {6} enableBoundingObject FALSE }}\n'.format(px, py, i, j, nm, sa, sb))
            # street-name sign at the SW corner (pole + two perpendicular plates)
            sx, sy = x - 7.4, y - 7.4
            w('DEF STSIGN_{0}_{1} Solid {{ translation {2:.1f} {3:.1f} 0 name "stsign_{0}_{1}" children [\n'.format(i, j, sx, sy))
            w('  Pose { translation 0 0 1.4 children [ Shape { castShadows FALSE appearance %s geometry Cylinder { radius 0.06 height 2.8 } } ] }\n' % mat("metal"))
            w('  Pose { translation 0.35 0 2.7 children [ Shape { castShadows FALSE appearance %s geometry Box { size 1.0 0.06 0.26 } } ] }\n' % mat("stsign"))
            w('  Pose { translation 0 0.35 2.55 children [ Shape { castShadows FALSE appearance %s geometry Box { size 0.06 1.0 0.26 } } ] }\n] }\n' % mat("stsign"))
    # speed-limit signs mid-block on each road (roadside)
    w("# speed-limit signs mid-block\n")
    for j, y in enumerate(YS):
        for i in range(len(XS) - 1):
            mx = (XS[i] + XS[i + 1]) / 2.0
            w('SpeedLimitSign {{ translation {0:.1f} {1:.1f} 0 rotation 0 0 1 1.5708 name "spd_ew{2}_{3}" }}\n'.format(mx, y - 6.6, j, i))
    for i, x in enumerate(XS):
        for j in range(len(YS) - 1):
            my = (YS[j] + YS[j + 1]) / 2.0
            w('SpeedLimitSign {{ translation {0:.1f} {1:.1f} 0 name "spd_ns{2}_{3}" }}\n'.format(x + 6.6, my, i, j))

    # ---- signals ----
    w("\n# === Signals (supervisor recolours SIG_NS_APP/SIG_EW_APP) ===\n")
    first = True
    for i, x in enumerate(XS):
        for j, y in enumerate(YS):
            if first:
                pole = 'DEF SIG_POLE PBRAppearance { baseColor 0.12 0.12 0.13 roughness 0.6 metalness 0.6 }'
                ns = 'DEF SIG_NS_APP PBRAppearance { baseColor 0.06 0.06 0.06 roughness 0.4 emissiveColor 0 0 0 emissiveIntensity 6 }'
                ew = 'DEF SIG_EW_APP PBRAppearance { baseColor 0.06 0.06 0.06 roughness 0.4 emissiveColor 0 0 0 emissiveIntensity 6 }'
                first = False
            else:
                pole, ns, ew = 'USE SIG_POLE', 'USE SIG_NS_APP', 'USE SIG_EW_APP'
            w('DEF SIG_{0}_{1} Solid {{ translation {2} {3} 0 name "signal_{0}_{1}" children [\n'.format(i, j, x + XR + 1, y + XR + 1))
            w('  Pose {{ translation 0 0 3 children [ Shape {{ appearance {0} geometry Cylinder {{ radius 0.14 height 6 }} }} ] }}\n'.format(pole))
            w('  Pose {{ translation 0 0 6 children [ Shape {{ appearance {0} geometry Sphere {{ radius 0.45 subdivision 2 }} }} ] }}\n'.format(ns))
            w('  Pose {{ translation 0 0 5 children [ Shape {{ appearance {0} geometry Sphere {{ radius 0.45 subdivision 2 }} }} ] }}\n] }}\n'.format(ew))

    # ---- buildings: wall-to-wall mixed-use ring per block ----
    w("\n# === Buildings (wall-to-wall mixed-use; shops/restaurants/offices/towers) ===\n")
    xb = [lo_x] + XS + [hi_x]
    yb = [lo_y] + YS + [hi_y]

    def inset(idx, n):
        return SIDEWALK if 1 <= idx <= n else 3.0

    cc = (len(XS) // 2, len(YS) // 2)        # centre cell -> park
    LANDMARKS = {(cc[0], cc[1] + 1): "Church", (cc[0] + 1, cc[1]): "Museum",
                 (cc[0] - 1, cc[1]): "Auditorium", (cc[0], cc[1] - 1): "TheThreeTowers"}
    nblk = 0
    sidx = 0
    for ci in range(len(xb) - 1):
        for cj in range(len(yb) - 1):
            bx0 = xb[ci] + inset(ci, len(XS)); bx1 = xb[ci + 1] - inset(ci + 1, len(XS))
            by0 = yb[cj] + inset(cj, len(YS)); by1 = yb[cj + 1] - inset(cj + 1, len(YS))
            if bx1 - bx0 < 12 or by1 - by0 < 12:
                continue
            ccx, ccy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
            if (ci, cj) == cc:                                   # central park
                w('# central park\n')
                w('DEF PARK Floor {{ translation {0:.1f} {1:.1f} -0.04 name "central_park" size {2:.0f} {3:.0f} tileSize 4 4 appearance Grass {{ }} }}\n'.format(
                    ccx, ccy, bx1 - bx0, by1 - by0))
                w('StoneFountain {{ translation {0:.1f} {1:.1f} -0.04 name "park_fountain" }}\n'.format(ccx, ccy))
                # paths (light concrete cross), benches, trees
                w('Pose {{ translation {0:.1f} {1:.1f} 0.0 children [ Shape {{ appearance {2} geometry Box {{ size {3:.0f} 3 0.04 }} }} ] }}\n'.format(ccx, ccy, mat("conc"), bx1 - bx0))
                w('Pose {{ translation {0:.1f} {1:.1f} 0.0 children [ Shape {{ appearance {2} geometry Box {{ size 3 {3:.0f} 0.04 }} }} ] }}\n'.format(ccx, ccy, mat("conc"), by1 - by0))
                for k, (ox, oy) in enumerate([(-7, -7), (7, -7), (-7, 7), (7, 7)]):
                    brot = math.atan2(-oy, -ox)
                    w('Bench {{ translation {0:.1f} {1:.1f} -0.04 rotation 0 0 1 {2:.4f} woodColor 0.45 0.3 0.16 name "park_bench_{3}" }}\n'.format(
                        ccx + ox, ccy + oy, brot, k))
                    if k in (0, 3):
                        seated(sidx, ccx + ox, ccy + oy, brot, rng.choice(PED_SHIRTS), rng.choice(PED_PANTS), rng.choice(PED_HAIR)); sidx += 1
                for k, (ox, oy) in enumerate([(-15, -15), (15, -15), (-15, 15), (15, 15), (-16, 2), (16, -2)]):
                    w('Oak {{ translation {0:.1f} {1:.1f} -0.04 rotation 0 0 1 {2:.2f} name "park_oak_{3}" }}\n'.format(
                        ccx + ox, ccy + oy, 0.3 + 0.5 * k, k))
                nblk += 1
                continue
            if (ci, cj) in LANDMARKS:                            # landmark building, facing the park
                proto = LANDMARKS[(ci, cj)]
                ang = math.atan2(-ccy, -ccx)
                w('{0} {{ translation {1:.1f} {2:.1f} -0.05 rotation 0 0 1 {3:.4f} name "landmark_{4}" }}\n'.format(
                    proto, ccx, ccy, ang, nblk))
                nblk += 1
                continue
            w('DEF BLK_{0} Solid {{ name "block_{0}" children [\n'.format(nblk))
            d = 11.0
            side(bx0, by0, bx1, by0, 0, -1)
            side(bx0, by1, bx1, by1, 0, 1)
            side(bx0, by0 + d, bx0, by1 - d, -1, 0)
            side(bx1, by0 + d, bx1, by1 - d, 1, 0)
            w('] }\n')
            nblk += 1

    # ---- a few street trees ----
    w("\n# === Street trees ===\n")
    ix = [(xb[k] + xb[k + 1]) / 2 for k in range(len(xb) - 1)]
    iy = [(yb[k] + yb[k + 1]) / 2 for k in range(len(yb) - 1)]
    t = 0
    for bx in ix:
        for by in iy:
            w('Oak {{ translation {0:.1f} {1:.1f} 0 rotation 0 0 1 {2:.2f} name "oak_{3}" }}\n'.format(bx, by, 0.3 + 0.4 * (t % 5), t))
            t += 1

    # ---- pedestrians: walk the interior block-perimeter sidewalks ----
    w("\n# === Pedestrians (walk block-perimeter sidewalks; supervisor-driven) ===\n")
    peds = []
    pidx = 0
    for ci in range(1, len(XS)):                       # interior cells only (4 road-bounded sides)
        for cj in range(1, len(YS)):
            lx0, lx1 = XS[ci - 1] + SWALK_C, XS[ci] - SWALK_C
            ly0, ly1 = YS[cj - 1] + SWALK_C, YS[cj] - SWALK_C
            if lx1 - lx0 < 8 or ly1 - ly0 < 8:
                continue
            perim = 2 * ((lx1 - lx0) + (ly1 - ly0))
            for k in range(PEDS_PER_LOOP):
                s = (k + 0.5) / PEDS_PER_LOOP * perim
                emit_ped(pidx, lx0, ly0, rng.choice(PED_SHIRTS), rng.choice(PED_PANTS), rng.choice(PED_HAIR))
                peds.append({"loop": [round(lx0, 1), round(ly0, 1), round(lx1, 1), round(ly1, 1)], "s": round(s, 2)})
                pidx += 1

    # ---- bus stops + people waiting (waiters are NOT in the spec, so they just stand) ----
    w("\n# === Bus stops + waiting people ===\n")
    BUS_STOPS = [(15.0, -24.5, -1.5708), (-15.0, 24.5, 1.5708), (-24.5, 15.0, 3.1416),
                 (24.5, -15.0, 0.0), (60.0, 24.5, 1.5708), (-60.0, -24.5, -1.5708)]
    for bi, (bx, by, ang) in enumerate(BUS_STOPS):
        w('BusStop {{ translation {0:.1f} {1:.1f} 0 rotation 0 0 1 {2:.4f} name "busstop_{3}" }}\n'.format(bx, by, ang, bi))
        cf, sf = math.cos(ang), math.sin(ang)
        for sgn in (-1, 1):
            wx = bx + cf * 1.4 - sf * 0.8 * sgn
            wy = by + sf * 1.4 + cf * 0.8 * sgn
            emit_ped(pidx, wx, wy, rng.choice(PED_SHIRTS), rng.choice(PED_PANTS), rng.choice(PED_HAIR), rot=ang)
            pidx += 1

    # ---- street benches with people sitting on them (facing the central park) ----
    w("\n# === Street benches (with people) ===\n")
    pcl = XS[2] - SWALK_C    # park-perimeter sidewalk coord (= 24.5 for the 4x4 grid)
    for (bxx, byy, brot) in [(10.0, -pcl + 1, 1.5708), (-10.0, pcl - 1, -1.5708),
                             (pcl - 1, -8.0, 3.1416), (-pcl + 1, 8.0, 0.0)]:
        w('Bench {{ translation {0:.1f} {1:.1f} 0 rotation 0 0 1 {2:.4f} woodColor 0.45 0.3 0.16 name "sbench_{3}" }}\n'.format(bxx, byy, brot, sidx))
        seated(sidx, bxx, byy, brot, rng.choice(PED_SHIRTS), rng.choice(PED_PANTS), rng.choice(PED_HAIR)); sidx += 1

    # ---- a dog being walked (the controller trails it behind a chosen pedestrian) ----
    w("\n# === Dog walker (dog trails a pedestrian) ===\n")
    w('DEF DOG Dog { translation -84 -86 0 rotation 0 0 1 1.5708 scale 0.85 color 0.46 0.34 0.20 name "dog" }\n')

    # ---- a city bus that loops the central block, pulling up to the 4 central stops ----
    w("\n# === City bus (city_traffic supervisor drives it around the central block) ===\n")
    blx, bhx = XS[1] + LANE, XS[2] - LANE       # inner-lane x (left, right kerb lanes)
    bly, bhy = YS[1] + LANE, YS[2] - LANE       # inner-lane y (bottom, top kerb lanes)
    bus_wp = [[blx, bhy, 0], [-15.0, bhy, 4], [bhx, bhy, 0], [bhx, -15.0, 4],
              [bhx, bly, 0], [15.0, bly, 4], [blx, bly, 0], [blx, 15.0, 4]]
    w('DEF BUS Solid { translation %.1f %.1f 0 rotation 0 0 1 0 name "city_bus" children [\n' % (blx, bhy))
    box(0, 0, 1.55, 11.0, 2.5, 2.35, "bus_body", cast=True)
    box(0, 0, 0.55, 11.0, 2.52, 0.7, "tire")                       # lower skirt (dark)
    box(0.2, 1.28, 1.95, 8.6, 0.06, 0.95, "bus_glass")             # window band, right
    box(0.2, -1.28, 1.95, 8.6, 0.06, 0.95, "bus_glass")            # window band, left
    box(5.55, 0, 1.85, 0.08, 2.3, 1.3, "bus_glass")                # windscreen
    box(-5.55, 0, 1.85, 0.08, 2.3, 1.1, "bus_glass")               # rear glass
    box(0, 0, 2.78, 10.6, 2.46, 0.18, "bus_trim")                  # roof
    box(5.45, 0, 2.45, 0.16, 1.6, 0.4, "bus_sign")                 # destination sign
    box(2.4, 1.29, 1.1, 1.0, 0.05, 1.6, "bus_glass")               # front door
    box(-2.2, 1.29, 1.1, 1.0, 0.05, 1.6, "bus_glass")              # rear door
    for wx in (3.7, -2.6, -3.9):
        cyl(wx, 1.25, 0.52, 0.52, 0.36, "tire")
        cyl(wx, -1.25, 0.52, 0.52, 0.36, "tire")
    w('] }\n')

    # ---- pedestrians who cross at the zebra crossings (wait for the light) ----
    w("\n# === Crossing pedestrians (cross when their road's cars are stopped) ===\n")
    # (ax, ay, bx, by, axis-of-road-being-crossed): walk between kerbs A and B over a zebra.
    CROSSERS = [(40.0, -36.5, 40.0, -25.5, "EW"), (-40.0, 25.5, -40.0, 36.5, "EW"),
                (-40.0, -36.5, -40.0, -25.5, "EW"), (25.5, 22.0, 36.5, 22.0, "NS"),
                (-36.5, -22.0, -25.5, -22.0, "NS"), (25.5, -22.0, 36.5, -22.0, "NS")]
    crossers = []
    for (ax, ay, bx, by, axis) in CROSSERS:
        rot = math.atan2(by - ay, bx - ax)
        emit_ped(pidx, ax, ay, rng.choice(PED_SHIRTS), rng.choice(PED_PANTS), rng.choice(PED_HAIR), rot=rot)
        crossers.append({"i": pidx, "a": [ax, ay], "b": [bx, by], "axis": axis})
        pidx += 1

    # ---- moving fleet ----
    w("\n# === Moving fleet (DEF CAR0..; city_traffic supervisor drives them) ===\n")
    cars = []
    fracs = [(i + 0.5) / CARS_PER_DIR for i in range(CARS_PER_DIR)]
    roads = [("EW", j, y) for j, y in enumerate(YS)] + [("NS", i, x) for i, x in enumerate(XS)]
    pcol = [(0.85, 0.85, 0.86), (0.60, 0.07, 0.07), (0.10, 0.18, 0.45), (0.05, 0.05, 0.06),
            (0.62, 0.64, 0.66), (0.08, 0.22, 0.12), (0.72, 0.60, 0.30), (0.55, 0.06, 0.06)]
    idx = 0
    for axis, ridx, fixed in roads:
        lo, hi = (lo_x, hi_x) if axis == "EW" else (lo_y, hi_y)
        for dr in (+1, -1):
            for fr in fracs:
                u = lo + (hi - lo) * fr
                (px, py), ang = car_pose(axis, fixed, dr, u)
                col = (0.96, 0.78, 0.10) if idx % 5 == 0 else pcol[idx % len(pcol)]  # ~1/5 are taxis
                w('DEF CAR{0} Car {{ translation {1:.1f} {2:.1f} 0 rotation 0 0 1 {3} name "car{0}" color {4} {5} {6} }}\n'.format(
                    idx, px, py, ang, *col))
                cars.append({"road": "%s%d" % (axis, ridx), "dir": dr, "u": round(u, 2)})
                idx += 1

    w('\nDEF CITY_TRAFFIC Robot { name "city_traffic" controller "city_traffic" supervisor TRUE children [ ] }\n')

    with open(WORLD, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("".join(out))
    spec = {"xs": XS, "ys": YS, "stub": STUB, "lane": LANE, "H": H, "cruise": CRUISE,
            "cars": cars, "peds": peds, "crossers": crossers,
            "bus": {"wp": bus_wp, "speed": 6.0}, "dog": {"ped": 0}}
    with open(SPEC, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(spec, fh, indent=1)

    nshapes = sum(1 for ln in out if "geometry Box" in ln)
    print("wrote %s (%d blocks, ~%d boxes, %d cars, %d peds, %d junctions)" % (
        os.path.relpath(WORLD, REPO), nblk, nshapes, len(cars), len(peds), len(XS) * len(YS)))
    print("wrote %s" % os.path.relpath(SPEC, REPO))


if __name__ == "__main__":
    main()
