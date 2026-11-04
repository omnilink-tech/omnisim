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

"""The clothes rack's assembly, step by step: a world and the plan its director plays.

Every part starts in the kit laid out on a mat in front of the rack and moves, in the
build order of the assembly guide, to where design.py puts it. The connector halves are
drawn from a mesh: design.py's CAD, or with --zmap the half the CNC twin actually milled.

    python projects/manufacturing/products/clothes_rack/assembly.py [--zmap connector.zmap]

writes meshes/connector_half.stl, assembly_plan.json and the world
projects/samples/demos/worlds/manufacturing/clothes_rack_assembly.omniworld.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design  # noqa: E402

ROOT = design.ROOT
WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/clothes_rack_assembly.omniworld"
PLAN = HERE / "assembly_plan.json"
MESH = HERE / "meshes" / "connector_half.stl"

MAT_Z = 0.004                    # the kit mat's top
ALU = "PBRAppearance { baseColor .8 .82 .85 metalness .3 roughness .35 }"
MILLED = "PBRAppearance { baseColor .86 .88 .91 metalness .3 roughness .25 }"
STEEL = "PBRAppearance { baseColor .68 .7 .73 metalness .3 roughness .35 }"
BLACK = "PBRAppearance { baseColor .05 .05 .05 metalness 0 roughness .6 }"
RUBBER = "PBRAppearance { baseColor .15 .15 .16 metalness 0 roughness .9 }"
WOOD = "PBRAppearance { baseColor .55 .38 .22 metalness 0 roughness .7 }"
GARMENTS = [(".85 .78 .65", "sweatshirt"), (".75 .76 .78", "shirt"), (".2 .33 .5", "denim jacket"),
            (".22 .3 .42", "jacket"), (".12 .12 .13", "jacket"), (".35 .42 .25", "overshirt")]


def _f(v: float) -> str:
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


# ---- rotations ------------------------------------------------------------------------
def quat(R: np.ndarray) -> List[float]:
    """Unit quaternion (w, x, y, z) of a rotation matrix whose columns are the local axes."""
    m = np.asarray(R, dtype=float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return list(q / np.linalg.norm(q))


def axis_angle(q: Sequence[float]) -> List[float]:
    w, x, y, z = q
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    return [x / s, y / s, z / s, 2 * math.acos(max(-1.0, min(1.0, w)))]


def z_to(axis) -> np.ndarray:
    """A rotation that turns local +z onto `axis` (a unit vector along a world axis, +-)."""
    a = np.asarray(axis, dtype=float)
    helper = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(helper, a)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(a, x), a])


FLIP_Y = np.diag([-1.0, 1.0, -1.0])     # half B: half A turned 180 degrees about the upright


# ---- geometry, in each part's own frame (metres) ---------------------------------------
def _at(z: float, shape: str, x: float = 0.0, y: float = 0.0, rot: str = "") -> str:
    r = f" rotation {rot}" if rot else ""
    return f"Pose {{ translation {_f(x)} {_f(y)} {_f(z)}{r} children [ {shape} ] }}"


def _cyl(r: float, h: float, app: str, sub: int = 24) -> str:
    return f"Shape {{ appearance {app} geometry Cylinder {{ radius {_f(r)} height {_f(h)} subdivision {sub} }} }}"


def _box(sx: float, sy: float, sz: float, app: str) -> str:
    return f"Shape {{ appearance {app} geometry Box {{ size {_f(sx)} {_f(sy)} {_f(sz)} }} }}"


def tube_shape(length_m: float) -> str:
    return _cyl(0.015, length_m, ALU, 40)


def bolt_shape() -> str:
    """M6 x 55 socket cap with its washer; origin on the washer's face, shank along +z."""
    return " ".join([_at(-0.0008, _cyl(0.006, 0.0016, STEEL)), _at(-0.0046, _cyl(0.005, 0.006, STEEL)),
                     _at(-0.0016 + 0.0275, _cyl(0.003, 0.055, STEEL, 12))])


def nut_shape() -> str:
    """Washer and M6 nyloc nut; origin on the washer's face, nut along +z."""
    return " ".join([_at(0.0008, _cyl(0.006, 0.0016, STEEL)), _at(0.0042, _cyl(0.0058, 0.0052, STEEL, 6))])


def foot_shape() -> str:
    """M8 levelling glide; origin on the floor, stem up into the tube."""
    return " ".join([_at(0.004, _cyl(0.015, 0.008, RUBBER)), _at(0.0105, _cyl(0.007, 0.005, STEEL, 6)),
                     _at(0.02, _cyl(0.004, 0.024, STEEL, 12))])


def cap_shape() -> str:
    """Tube end cap; origin on the tube's end face, +z outward."""
    return " ".join([_at(0.0015, _cyl(0.0152, 0.003, BLACK)), _at(-0.006, _cyl(0.013, 0.012, BLACK))])


def garment_shape(colour: str) -> str:
    """A hanger and a garment; origin where the hook rests on top of the bar, x along the bar."""
    cloth = f"PBRAppearance {{ baseColor {colour} metalness 0 roughness .95 }}"
    tilt = math.atan2(0.035, 0.2)
    parts = [
        # The hook: up from the hanger, round the back of the bar, over its top, down the front.
        _at(-0.056, _box(0.004, 0.004, 0.048, STEEL)),
        _at(-0.016, _box(0.004, 0.004, 0.032, STEEL), y=0.019),
        _at(0.002, _box(0.004, 0.038, 0.004, STEEL)),
        _at(-0.004, _box(0.004, 0.004, 0.012, STEEL), y=-0.017),
        # Hanger shoulders.
        _at(-0.098, _box(0.012, 0.205, 0.02, WOOD), y=0.1, rot=f"1 0 0 {_f(-tilt)}"),
        _at(-0.098, _box(0.012, 0.205, 0.02, WOOD), y=-0.1, rot=f"1 0 0 {_f(tilt)}"),
        # The garment: body and sleeves.
        _at(-0.42, _box(0.03, 0.36, 0.62, cloth)),
        _at(-0.37, _box(0.028, 0.085, 0.52, cloth), y=0.215, rot=f"1 0 0 {_f(-0.14)}"),
        _at(-0.37, _box(0.028, 0.085, 0.52, cloth), y=-0.215, rot=f"1 0 0 {_f(0.14)}"),
    ]
    return " ".join(parts)


# ---- the plan -------------------------------------------------------------------------
class Track:
    """Keyframes of one part: (t, position, quaternion, lift into this key, spin turns at it)."""

    def __init__(self, pos, R):
        self.keys = [[0.0, list(map(float, pos)), quat(R), 0.0, 0.0]]

    @property
    def last(self):
        return self.keys[-1]

    def hold(self, t: float):
        if t > self.last[0] + 1e-9:
            k = list(self.last)
            k[0], k[3] = t, 0.0
            self.keys.append(k)

    def move(self, t0: float, t1: float, pos, R=None, lift: float = 0.0, turns: float = 0.0):
        self.hold(t0)
        q = quat(R) if R is not None else self.last[2]
        self.keys.append([t1, list(map(float, pos)), q, lift, self.last[4] + turns])

    def json(self):
        return [[round(k[0], 3)] + [round(v, 6) for v in k[1]] + [round(v, 7) for v in k[2]]
                + [round(k[3], 4), round(k[4], 3)] for k in self.keys]


def plan(rack: design.Rack) -> dict:
    mm = 0.001
    parts: Dict[str, dict] = {}

    def add(name, shape, kit_pos, kit_R):
        parts[name] = {"shape": shape, "track": Track(kit_pos, kit_R)}
        return parts[name]["track"]

    conns = {c["name"]: c for c in design.connectors(rack)}
    tubes = {t["name"]: t for t in design.tubes(rack)}
    lying = z_to([1, 0, 0])

    # ---- the kit, laid out on the mat ----
    tube_rows = {"bar": (0.0, -0.70), "upright_left": (0.0, -0.78), "upright_right": (0.0, -0.86),
                 "base_left": (-0.3, -0.94), "base_right": (0.3, -0.94)}
    for name, t in tubes.items():
        kx, ky = tube_rows[name]
        add(name, tube_shape(t["length"] * mm), [kx, ky, MAT_Z + 0.015], lying)
    halves = []
    for i, (cname, side) in enumerate((c, s) for c in ("foot_left", "foot_right", "top_left", "top_right")
                                      for s in ("a", "b")):
        halves.append(f"{cname}_{side}")
        add(f"{cname}_{side}", 'Shape { appearance %s geometry Mesh { url "MESH" } }' % MILLED,
            [-0.455 + i * 0.13, -1.10, MAT_Z + (rack.block_t_mm + rack.relief_mm) * mm], np.eye(3))
    holes = [(hx * mm, (hy - rack.through_y_mm) * mm) for hx, hy in rack.holes_xy]
    fasteners = []
    for i, (cname, (hx, hy)) in enumerate((c, h) for c in ("foot_left", "foot_right", "top_left", "top_right")
                                          for h in holes):
        fasteners.append((cname, i, hx, hy))
        add(f"bolt_{i}", bolt_shape(), [-0.42 + i * 0.035, -1.25, MAT_Z + 0.006], lying)
        add(f"nut_{i}", nut_shape(), [-0.42 + i * 0.035, -1.33, MAT_Z], np.eye(3))
    feet = [(sx * rack.upright_x_mm * mm, sy * rack.foot_y_mm * mm) for sx in (-1, 1) for sy in (-1, 1)]
    for i in range(4):
        add(f"foot_{i}", foot_shape(), [-0.12 + i * 0.06, -1.42, MAT_Z], np.eye(3))
    caps = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            caps.append(([sx * rack.upright_x_mm * mm, sy * rack.base_mm / 2 * mm, rack.base_z * mm], [0, sy, 0]))
        caps.append(([sx * rack.bar_mm / 2 * mm, 0.0, rack.bar_z * mm], [sx, 0, 0]))
    for i in range(6):
        add(f"cap_{i}", cap_shape(), [0.2 + i * 0.05, -1.42, MAT_Z + 0.012], np.eye(3))
    garment_x = list(design.GARMENT_X_M)
    for i, (colour, _) in enumerate(GARMENTS):
        add(f"garment_{i}", garment_shape(colour), [garment_x[i], 0.0, -2.0], np.eye(3))

    def T(name):
        return parts[name]["track"]

    def conn_pose(cname, side):
        c = conns[cname]
        R = c["R"] if side == "a" else c["R"] @ FLIP_Y
        return np.asarray(c["origin"]) * mm, R

    def in_conn(cname, p_mm):
        c = conns[cname]
        return np.asarray(c["origin"]) * mm + c["R"] @ np.asarray(p_mm) * mm

    steps, camera = [], []
    t = 5.0

    def caption(t0, t1, text):
        steps.append([round(t0, 2), round(t1, 2), text])

    # 1. Base tubes and feet.
    caption(t, t + 8, f"1. Rivet an M8 nut into each base tube, {rack.foot_y_mm:.0f} mm each side of its centre, "
                      "and screw in a levelling foot")
    for k, side in enumerate(("left", "right")):
        tb = tubes[f"base_{side}"]
        T(f"base_{side}").move(t + 0.4 * k, t + 3 + 0.4 * k, np.asarray(tb["centre"]) * mm, z_to(tb["axis"]), lift=0.25)
    for i, (fx, fy) in enumerate(feet):
        tr = T(f"foot_{i}")
        a = t + 3.5 + 0.3 * i
        tr.move(a, a + 2.0, [fx, fy + math.copysign(0.12, fy), 0.0], lift=0.15)
        tr.move(a + 2.0, a + 3.5, [fx, fy, 0.0], turns=3)
    t += 8.0
    # 2-4. Foot connectors around base tube and upright.
    caption(t, t + 5, "2. Seat each base tube in a connector half, on the outside")
    for k, cname in enumerate(("foot_left", "foot_right")):
        pos, R = conn_pose(cname, "a")
        tr = T(f"{cname}_a")
        tr.move(t + 0.5 * k, t + 2.5 + 0.5 * k, in_conn(cname, [0, 0, -120]), R, lift=0.3)
        tr.move(t + 2.5 + 0.5 * k, t + 4.0 + 0.5 * k, pos)
    t += 5.0
    caption(t, t + 7, "3. Stand an upright in each connector's vertical bore, down onto the base tube")
    for k, side in enumerate(("left", "right")):
        tb = tubes[f"upright_{side}"]
        end = np.asarray(tb["centre"]) * mm
        T(f"upright_{side}").move(t + 0.6 * k, t + 3.5 + 0.6 * k, end + [0, 0, 0.35], z_to(tb["axis"]), lift=0.4)
        T(f"upright_{side}").move(t + 3.5 + 0.6 * k, t + 5.5 + 0.6 * k, end)
    t += 7.0
    caption(t, t + 5, "4. Close each foot connector with a second half: both halves are the same CNC part")
    for k, cname in enumerate(("foot_left", "foot_right")):
        pos, R = conn_pose(cname, "b")
        tr = T(f"{cname}_b")
        tr.move(t + 0.5 * k, t + 2.5 + 0.5 * k, in_conn(cname, [0, 0, 120]), R, lift=0.3)
        tr.move(t + 2.5 + 0.5 * k, t + 4.0 + 0.5 * k, pos)
    t += 5.0

    def bolt_up(cnames, t):
        for j, (cname, i, hx, hy) in enumerate(f for f in fasteners if f[0] in cnames):
            R = conns[cname]["R"]
            a = t + 0.18 * j
            fin = in_conn(cname, [hx / mm, hy / mm, -(rack.block_t_mm + rack.relief_mm)])
            T(f"bolt_{i}").move(a, a + 2.0, fin + R @ [0, 0, -0.08], R, lift=0.2)
            T(f"bolt_{i}").move(a + 2.0, a + 3.5, fin, turns=4)
            fin = in_conn(cname, [hx / mm, hy / mm, rack.block_t_mm + rack.relief_mm])
            T(f"nut_{i}").move(a + 1.5, a + 3.5, fin + R @ [0, 0, 0.07], R, lift=0.2)
            T(f"nut_{i}").move(a + 3.5, a + 5.0, fin, turns=-5)
        return t + 0.18 * 7 + 5.5

    caption(t, t + 7, "5. Four M6 x 55 bolts per connector, washers both sides, nyloc nuts: finger-tight for now")
    t = bolt_up(("foot_left", "foot_right"), t) + 0.5
    caption(t, t + 5, "6. Hold the back half of a top connector on each upright's top")
    for k, cname in enumerate(("top_left", "top_right")):
        pos, R = conn_pose(cname, "a")
        tr = T(f"{cname}_a")
        tr.move(t + 0.5 * k, t + 2.5 + 0.5 * k, in_conn(cname, [0, 0, -120]), R, lift=0.3)
        tr.move(t + 2.5 + 0.5 * k, t + 4.0 + 0.5 * k, pos)
    t += 5.0
    caption(t, t + 6, "7. Lay the top bar in both connectors, its ends 10 mm out of each")
    bar = tubes["bar"]
    T("bar").move(t, t + 3.5, np.asarray(bar["centre"]) * mm + [0, -0.25, 0.3], z_to(bar["axis"]))
    T("bar").move(t + 3.5, t + 5.5, np.asarray(bar["centre"]) * mm)
    t += 6.0
    caption(t, t + 5, "8. Close the top connectors and bolt them like the feet")
    for k, cname in enumerate(("top_left", "top_right")):
        pos, R = conn_pose(cname, "b")
        tr = T(f"{cname}_b")
        tr.move(t + 0.5 * k, t + 2.5 + 0.5 * k, in_conn(cname, [0, 0, 120]), R, lift=0.3)
        tr.move(t + 2.5 + 0.5 * k, t + 4.0 + 0.5 * k, pos)
    t += 4.5
    t = bolt_up(("top_left", "top_right"), t) + 0.5
    caption(t, t + 5, "9. Check it is square (equal diagonals), then torque all 16 bolts to 5 N m, crosswise")
    for j, (cname, i, hx, hy) in enumerate(fasteners):
        a = t + 0.5 + 0.2 * (j % 4)
        T(f"bolt_{i}").move(a, a + 1.5, T(f"bolt_{i}").last[1], turns=1)
    t += 5.0
    caption(t, t + 5, "10. Tap in the six end caps; set the feet so the rack stands level")
    for i, (pos, axis) in enumerate(caps):
        p = np.asarray(pos)             # on the tube's end face
        tr = T(f"cap_{i}")
        a = t + 0.3 * i
        tr.move(a, a + 2.0, p + np.asarray(axis) * 0.06, z_to(axis), lift=0.25)
        tr.move(a + 2.0, a + 3.0, p)
    t += 5.0
    checks = design.checks(rack)
    caption(t, t + 1e4, f"Done: {rack.bar_mm:.0f} x {rack.base_mm:.0f} x {rack.bar_top_mm:.0f} mm, "
                        f"{checks['rack_kg']:.1f} kg, made for {checks['load_kg']:.0f} kg of clothes on the bar")
    for i in range(len(GARMENTS)):
        tr = T(f"garment_{i}")
        a = t + 0.6 * i
        fin = [garment_x[i], 0.0, rack.bar_top_mm * mm]
        tr.move(a, a + 0.01, [fin[0], fin[1], fin[2] + 0.5])
        tr.move(a + 0.01, a + 1.6, fin)
    t_done = t + 0.6 * len(GARMENTS) + 2.0

    # Camera: (t, position, target); "orbit" after the last key.
    over = ([2.3, -3.3, 1.9], [0.0, -0.5, 0.5])
    foot = ([1.15, -0.85, 0.5], [0.5, -0.02, 0.1])
    top = ([1.15, -1.0, 1.85], [0.5, 0.0, 1.45])
    high = ([2.6, -3.2, 2.2], [0.0, 0.0, 0.85])
    front = ([0.9, -4.1, 1.15], [0.0, 0.0, 0.75])
    for a, b in zip(steps, steps[1:]):
        a[1] = b[0]                    # each caption stays up until the next step starts
    st = [x[0] for x in steps]
    camera = [[0.0, *over], [st[1] - 1.0, *over], [st[1], *foot], [st[5] - 1.0, *foot], [st[5], *top],
              [st[6] - 0.5, *top], [st[6] + 0.5, *high], [st[7] - 0.5, *high], [st[7] + 0.5, *top],
              [st[9] - 0.5, *top], [st[9] + 0.5, *over], [st[10] - 0.5, *over], [st[10] + 0.5, *front],
              [t_done, *front]]
    return {"title": "OmniSim: a clothes rack, from the CNC twin to the finished product",
            "kit": "Kit: 5 aluminium tubes sawn to length, 8 CNC-milled connector halves, 16 M6 bolts, 4 feet, 6 caps",
            "duration": round(t_done, 2), "hold_s": 25.0, "orbit": {"centre": [0, 0, 0.75], "radius": 4.2, "height": 1.15,
                                                                   "period_s": 40.0},
            "captions": steps, "camera": [[round(c[0], 2)] + c[1] + c[2] for c in camera],
            "parts": {k: v["track"].json() for k, v in parts.items()},
            "_shapes": {k: v["shape"] for k, v in parts.items()}}


# ---- the world ------------------------------------------------------------------------
def look(pos, target) -> List[float]:
    """Viewpoint orientation looking from `pos` at `target` (the camera looks along +x, +z up)."""
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    left = np.cross([0.0, 0.0, 1.0], f)
    left /= np.linalg.norm(left)
    return axis_angle(quat(np.column_stack([f, left, np.cross(f, left)])))


def world(p: dict, mesh_url: str, plan_ref: str) -> str:
    out = ["#OMNISIM R2025a utf8",
           'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
           'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
           'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
           "# Generated by projects/manufacturing/products/clothes_rack/assembly.py. Regenerate; do not hand-edit.",
           'WorldInfo { basicTimeStep 16 title "Clothes rack - assembly" FPS 30 }',
           f"DEF VIEW Viewpoint {{ position {' '.join(_f(v) for v in p['camera'][0][1:4])} "
           f"orientation {' '.join(_f(v) for v in look(p['camera'][0][1:4], p['camera'][0][4:7]))} fieldOfView .75 }}",
           # The sun front-right, behind the cameras, so the faces they see are lit.
           "OmniSimSky { }", "DEF SUN OmniSimSun { }", "DEF SUN_MARKER OmniSimSunMarker { translation 5 -7 9 }",
           'DEF FLOOR Solid { name "floor" translation 0 0 -.05 children [ Shape { appearance PBRAppearance '
           '{ baseColor .3 .31 .32 roughness .9 metalness 0 } geometry Box { size 8 8 .1 } } ] boundingObject Box '
           '{ size 8 8 .1 } }',
           f"DEF MAT Pose {{ translation 0 -1.06 {_f(MAT_Z / 2)} children [ Shape {{ appearance PBRAppearance "
           f"{{ baseColor .16 .2 .26 roughness .95 metalness 0 }} geometry Box {{ size 1.3 .86 {_f(MAT_Z)} }} }} ] }}",
           f'DEF DIRECTOR Robot {{ name "assembly-director" controller "clothes_rack_assembly" '
           f'controllerArgs [ "--plan" "{plan_ref}" ] supervisor TRUE }}']
    for name, track in p["parts"].items():
        k = track[0]
        rot = axis_angle(k[4:8])
        shape = p["_shapes"][name].replace("MESH", mesh_url)
        out.append(f"DEF P_{name.upper()} Pose {{ translation {_f(k[1])} {_f(k[2])} {_f(k[3])} "
                   f"rotation {' '.join(_f(v) for v in rot)} children [ {shape} ] }}")
    return "\n".join(out) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zmap", type=Path, help="draw the connector half the twin milled (OMNISIM_MILLING_STOCK_DUMP)")
    ap.add_argument("--res", type=float, default=1.0, help="connector mesh spacing, mm")
    args = ap.parse_args(argv)
    rack = design.Rack()
    zmap = None
    if args.zmap:
        sys.path.insert(0, str(design.SERVICE))
        import inspect_part
        zmap = inspect_part.load_zmap(args.zmap)
    MESH.parent.mkdir(parents=True, exist_ok=True)
    n = design.write_half_stl(rack, MESH, res=args.res, metres=True, zmap=zmap, frame="connector")
    p = plan(rack)
    p["connector_mesh"] = "as milled by the CNC twin" if zmap else "CAD (design.py)"
    shapes = p.pop("_shapes")
    PLAN.write_text(json.dumps(p, indent=1), encoding="utf-8", newline="\n")
    p["_shapes"] = shapes
    import os
    mesh_url = Path(os.path.relpath(MESH, WORLD.parent)).as_posix()
    WORLD.write_text(world(p, mesh_url, PLAN.relative_to(ROOT).as_posix()), encoding="utf-8", newline="\n")
    print(f"wrote {MESH.relative_to(ROOT)} ({n} triangles, {p['connector_mesh']}), {PLAN.relative_to(ROOT)} "
          f"({len(p['parts'])} parts, {p['duration']} s), {WORLD.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
