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

"""A two-wheeled desk robot, designed for the three machines OmniSim Manufacturing twins.

Each process makes what it is best at, and what nobody should make is bought:

- LASER (Sienci LaserBeam on the LongMill, 3 mm plywood): the two deck plates, every
  hole and slot in them, and the name engraved on the top one.
- PRINTER (Sovol SV08, PLA): the two motor clamps, the two wheel rims (with the tyre
  groove) and the electronics tray. Shapes a sheet or a 3-axis mill cannot make.
- CNC (LongMill, 6061 aluminium): the two wheel hubs. A D-shaft bore has to be stiff and
  exact, or the wheel slips on the shaft; PLA creeps and splits there, aluminium does not.
- BOUGHT (`BOUGHT`, each with its source): Pololu micro metal gearmotors and ball caster,
  O-ring tyres, a Seeed XIAO ESP32-S3, a Pololu DRV8833 driver, a 4 x AA holder, M2/M3
  screws and M3 standoffs.

The fit comes from the bought parts' drawings. The 40 mm wheel puts the axle 19.7 mm up;
the motor's shaft sits 5 mm above the face it rests on, so the bottom plate's top is at
14.7 mm and its underside at 11.7 mm. The caster with its 1/16" spacer is 10.1 + 1.6 =
11.7 mm tall: the robot stands level to 0.03 mm.

Frames: the ROBOT frame has x forward, y to the left, z up, the floor at z = 0 and the
plates centred on x = y = 0. A part's own frame is its MAKING frame: the sheet's top
centre for the plates (work coordinates), the bed for printed parts (z up from the bed),
the stock's top centre for the hubs. Units: mm and g unless a name says otherwise.

    python projects/manufacturing/products/desk_robot/design.py --out .tmp/desk_robot
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

PRODUCT = Path(__file__).resolve().parent
MANUFACTURING = PRODUCT.parents[1]
ROOT = MANUFACTURING.parents[1]
# The engraved name: the brand wordmark's outlines, and the cap height (91 units) its SVGs set it at.
WORDMARK_SVG = ROOT / "resources" / "branding" / "omnisim" / "svg" / "wordmark_mono_dark.svg"
WORDMARK_CAP_UNITS = 91.0
DEJAVU_BOLD_CAP = 0.729                  # DejaVu Sans Bold cap height per em, the face the name used to be in
for _p in (MANUFACTURING / "cnc", MANUFACTURING / "laser"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import job as laser_job  # noqa: E402
import toolpath  # noqa: E402

POLOLU = "https://www.pololu.com"
# Every bought part, with the numbers the design uses and where they come from.
BOUGHT: Dict[str, dict] = {
    "gearmotor": {
        "spec": "Pololu #3065, 100:1 Micro Metal Gearmotor HPCB 6V", "qty": 2, "g": 9.5,
        "body_mm": [12.0, 10.0, 25.6], "gearbox_mm": 9.0, "shaft_d_mm": 3.0, "shaft_flat_mm": 1.0,
        "shaft_len_mm": 10.0, "bushing_proud_mm": 0.7, "no_load_rpm": 330, "stall_kgcm": 1.6, "stall_a": 1.5,
        "source": f"{POLOLU}/product/3065/specs; drawing {POLOLU}/file/0J949/micro-metal-gearmotors-dimensions.pdf",
        "note": "the shaft is taken as centred on the 10 mm height (the drawing shows it so; not dimensioned)",
    },
    "caster": {
        "spec": "Pololu #951 ball caster, 3/8 in metal ball, with its 1/16 in spacer", "qty": 1, "g": 4.0,
        "height_mm": 10.07, "spacer_mm": 1.5875, "hole_spacing_mm": 13.5, "hole_d_mm": 2.3, "flange_mm": 1.52,
        "footprint_mm": [19.1, 12.1],
        "source": f"{POLOLU}/product/951; drawing {POLOLU}/file/0J1636/pololu-ball-caster-with-0-375in-ball.pdf",
    },
    "tyre": {
        "spec": "O-ring 34 x 2 mm NBR 70 (DIN 3771 / ISO 3601-1), stretched onto the rim", "qty": 2, "g": 0.4,
        "id_mm": 34.0, "cs_mm": 2.0,
        "source": "https://tameson.com/products/s24cr-x100-nbr-o-ring-34-x-2mm-od-38mm-70-shore-a-100-pieces; "
                  "stretch <= 5 %: Parker O-Ring Handbook ORD 5700 s3.3",
    },
    "controller": {
        "spec": "Seeed Studio XIAO ESP32-S3", "qty": 1, "g": 3.0, "board_mm": [21.0, 17.8],
        "source": "https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/",
        "note": "no mounting holes: it sits in the printed tray; mass estimated",
    },
    "driver": {
        "spec": "Pololu #2130 DRV8833 dual motor driver carrier", "qty": 1, "g": 1.0, "board_mm": [20.3, 12.7],
        "source": f"{POLOLU}/product/2130/specs",
    },
    "battery_holder": {
        "spec": "Keystone 2478, 4 x AA side by side, wire leads", "qty": 1, "g": 15.0, "outline_mm": [63.0, 58.0, 16.0],
        "source": "https://www.keyelco.com/userAssets/file/M65p28.pdf",
        "note": "held by a hook-and-loop strap through two slots (its hole positions are not dimensioned); mass estimated",
    },
    "cells": {"spec": "AA alkaline cell", "qty": 4, "g": 23.0,
              "source": "Energizer E91 datasheet, typical weight 23 g"},
    "standoff": {"spec": "Wurth 970250321, M3 female/female hex standoff, 25 mm", "qty": 4, "g": 2.6, "len_mm": 25.0,
                 "source": "https://www.we-online.com/components/products/datasheet/970250321.pdf",
                 "note": "mass estimated (5.5 mm hex, 25 mm, steel)"},
    # Fasteners: lengths from bought_parts.joints() (each stack's grip + nut + one pitch, or
    # 1.5 d into a standoff); masses of steel ISO 4762 screws + ISO 4032 nuts, from their volumes.
    "screw_m3": {"spec": "M3 x 8 socket head cap screw, ISO 4762 (into the standoffs, 5 mm of thread)", "qty": 8,
                 "g": 0.9, "source": "https://www.fasteners.eu/standards/ISO/4762/"},
    "screw_m2_clamp": {"spec": "M2 x 8 socket head cap screw + M2 nut ISO 4032 (motor clamps)", "qty": 4, "g": 0.5,
                       "source": "https://www.fasteners.eu/standards/ISO/4762/"},
    "screw_m16_caster": {"spec": "M1.6 x 8 socket head cap screw + M1.6 nut (caster, heads in its housing's notches)",
                         "qty": 2, "g": 0.28, "source": "https://www.fasteners.eu/standards/ISO/4762/",
                         "note": "an M2 cap head (3.8 mm) does not fit the 3.0 mm notches in Pololu's housing (its STEP)"},
    "screw_m2_hub": {"spec": "M2 x 8 socket head cap screw (hub into rim, self-tapping in PLA)", "qty": 6, "g": 0.35,
                     "source": "https://www.fasteners.eu/standards/ISO/4762/"},
    "screw_m2_tray": {"spec": "M2 x 8 socket head cap screw + M2 nut ISO 4032 (tray)", "qty": 2, "g": 0.5,
                      "source": "https://www.fasteners.eu/standards/ISO/4762/"},
    "strap": {"spec": "hook-and-loop strap, 20 mm wide, 200 mm", "qty": 1, "g": 3.0, "source": "any"},
}
CLEARANCE = {"M1.6": 1.8, "M2": 2.4, "M3": 3.4}      # ISO 273 normal fit (Optimas / Albany County Fasteners charts)
M2_TAP_PLA_MM = 1.7                       # an M2 screw cuts its own thread in a printed 1.7 mm hole
PLYWOOD_G_CM3 = 0.68                      # birch plywood, typical
PLA_G_CM3 = 1.24
ALUMINIUM_G_CM3 = 2.70
# The laser's kerf, measured on the twin's test tag (laser/README.md): 0.26 mm at the top of
# a 3 mm cut, about 0.1 at the bottom. The plates are cut offset by half the mean.
LASER_KERF_MM = 0.18


@dataclass(frozen=True)
class Robot:
    plate_x: float = 90.0
    plate_y: float = 80.0
    plate_t: float = 3.0
    corner_r: float = 5.0
    axle_x: float = 15.0                 # the wheels' axis, ahead of the plates' centre
    faceplate_y: float = 40.5            # |y| of each gearbox's output face (just outside the plate)
    clamp_depth: float = 8.5             # along the motor, over the gearbox
    clamp_wall: float = 1.6
    clamp_clear: float = 0.05            # per side, around the 12 mm of the gearbox
    clamp_squeeze: float = 0.1           # the roof sits 0.1 under the gearbox's 10 mm: the tabs stand
                                         # 0.1 proud until the screws pull them down and the PLA grips it
    clamp_tab_t: float = 2.0
    clamp_span: float = 28.0             # tab to tab
    clamp_hole_spacing: float = 21.0     # an M2 nut (4 mm AF, 2.31 corner) clears the 7.65 mm wall by 0.2
    groove_root_d: float = 35.4          # 4.1 % stretch on the 34 mm O-ring (Parker: <= 5 %)
    rim_width: float = 7.0
    rim_flange_d: float = 37.4
    rim_web: float = 2.5
    rim_boss_r: float = 8.0
    rim_recess_r: float = 16.0
    rim_bore_d: float = 3.4              # the shaft passes through, clear
    hub_d: float = 16.0
    hub_t: float = 4.0
    hub_bore_d: float = 3.10             # the D-bore: 0.1 over the shaft
    hub_flat: float = 1.06               # its flat, from the bore's centre (the shaft's is 1.0)
    hub_screw_pcd: float = 11.0
    hub_gap: float = 2.4                 # from the gearbox face to the hub: the hub's M2 cap heads (2.0 mm,
                                         # toward the gearbox) clear it by 0.4. At 1.0 they hit it: found by
                                         # the physical assembly on Newton (assemble.py, step 6)
    caster_x: float = -38.0              # its screw heads clear the tray's end (x -33.5) by 2.6 mm
    standoff_xy: Tuple[float, float] = (39.0, 34.0)
    standoff_len: float = 25.0
    tray_x: float = -10.0
    name: str = "OmniSim"                # engraved as the brand wordmark (name_engraving)

    # ---- what follows from the parts ------------------------------------------------
    @property
    def motor(self) -> dict:
        return BOUGHT["gearmotor"]

    @property
    def tyre_cs_mm(self) -> float:
        """The O-ring's section, thinned by its stretch (volume kept): d / sqrt(1 + s)."""
        t = BOUGHT["tyre"]
        stretch = self.groove_root_d / t["id_mm"] - 1.0
        return t["cs_mm"] / math.sqrt(1.0 + stretch)

    @property
    def tyre_stretch(self) -> float:
        return self.groove_root_d / BOUGHT["tyre"]["id_mm"] - 1.0

    @property
    def wheel_d(self) -> float:
        return self.groove_root_d + 2.0 * self.tyre_cs_mm

    @property
    def axle_z(self) -> float:
        return self.wheel_d / 2.0

    @property
    def plate_top_z(self) -> float:
        return self.axle_z - self.motor["body_mm"][1] / 2.0

    @property
    def caster_z(self) -> float:
        c = BOUGHT["caster"]
        return c["height_mm"] + c["spacer_mm"]

    @property
    def clamp_y(self) -> float:
        """|y| of each clamp's centre: over the gearbox, 0.5 mm in from its face."""
        return self.faceplate_y - 0.5 - self.clamp_depth / 2.0

    @property
    def wheel_y(self) -> float:
        """|y| of each tyre's centre plane."""
        return self.faceplate_y + self.hub_gap + self.hub_t + self.rim_width / 2.0

    @property
    def track(self) -> float:
        return 2.0 * self.wheel_y


# ---- 2D shapes -------------------------------------------------------------------------

def rounded_rect(w: float, h: float, r: float, cx: float = 0.0, cy: float = 0.0) -> Polygon:
    return box(cx - w / 2 + r, cy - h / 2 + r, cx + w / 2 - r, cy + h / 2 - r).buffer(r, quad_segs=16)


def hole(x: float, y: float, d: float) -> Polygon:
    return Point(x, y).buffer(d / 2.0, quad_segs=32)


def slot(x: float, y: float, length: float, width: float, along_y: bool = True) -> Polygon:
    half = (length - width) / 2.0
    a, b = ((x, y - half), (x, y + half)) if along_y else ((x - half, y), (x + half, y))
    return shapely.LineString([a, b]).buffer(width / 2.0, quad_segs=16)


def _standoffs(r: Robot) -> List[Polygon]:
    sx, sy = r.standoff_xy
    return [hole(i * sx, j * sy, CLEARANCE["M3"]) for i in (-1, 1) for j in (-1, 1)]


def bottom_plate(r: Robot) -> Polygon:
    holes = _standoffs(r)
    holes += [hole(r.axle_x + i * r.clamp_hole_spacing / 2, j * r.clamp_y, CLEARANCE["M2"]) for i in (-1, 1) for j in (-1, 1)]
    s = BOUGHT["caster"]["hole_spacing_mm"] / 2
    holes += [hole(r.caster_x, j * s, CLEARANCE["M1.6"]) for j in (-1, 1)]   # see bought_parts.joints: M1.6
    holes += [hole(r.tray_x, j * TRAY["hole_y"], CLEARANCE["M2"]) for j in (-1, 1)]
    holes.append(slot(r.axle_x, 0.0, 12.0, 6.0))          # the motor leads, up to the tray
    return rounded_rect(r.plate_x, r.plate_y, r.corner_r).difference(unary_union(holes))


def top_plate(r: Robot) -> Polygon:
    holes = _standoffs(r)
    holes += [slot(i * 33.5, 0.0, 22.0, 3.0) for i in (-1, 1)]   # the battery strap
    holes.append(hole(0.0, 34.0, 8.0))                            # the battery leads, down
    return rounded_rect(r.plate_x, r.plate_y, r.corner_r).difference(unary_union(holes))


def _wordmark_shape(cap_h: float) -> Polygon:
    """The OmniSim wordmark (Sora SemiBold, kerned) from the brand SVG, y up, in mm.

    Sora's contours overlap (the i's stem and its top bar), so they are filled by winding, not
    by parity: outer contours are unioned and the opposite-turning ones (the counters) cut out."""
    outer, inner = [], []
    for poly in _wordmark_polygons(cap_h):
        p = Polygon(poly)
        (outer if p.exterior.is_ccw else inner).append(p if p.is_valid else p.buffer(0))
    if sum(p.area for p in inner) > sum(p.area for p in outer):
        outer, inner = inner, outer
    return unary_union(outer).difference(unary_union(inner))


def _wordmark_polygons(cap_h: float) -> List[np.ndarray]:
    """The OmniSim wordmark's contours, sampled, y up, in mm."""
    from svgelements import SVG, Close, Move
    from svgelements import Path as SvgPath
    scale = cap_h / WORDMARK_CAP_UNITS
    polys: List[np.ndarray] = []
    for el in SVG.parse(str(WORDMARK_SVG), reify=True).elements():
        if not isinstance(el, SvgPath):
            continue
        pts: List[Tuple[float, float]] = []
        for seg in el:
            if isinstance(seg, (Move, Close)):
                if len(pts) > 2:
                    polys.append(np.array(pts) * (scale, -scale))
                pts = [(seg.end.x, seg.end.y)] if isinstance(seg, Move) else []
            elif seg.start is not None:
                n = 1 if type(seg).__name__ == "Line" else 4
                pts += [(seg.point(i / n).x, seg.point(i / n).y) for i in range(1, n + 1)]
        if len(pts) > 2:
            polys.append(np.array(pts) * (scale, -scale))
    return polys


def name_engraving(r: Robot, height: float = 5.0) -> Polygon:
    """The name, as outlines, reading across the front of the top plate. "OmniSim" is the brand
    wordmark (resources/branding/omnisim/BRAND.md), with the cap height DejaVu Sans Bold has at
    this size; any other name is set in DejaVu Sans Bold."""
    if r.name == "OmniSim":
        shape = _wordmark_shape(cap_h=DEJAVU_BOLD_CAP * height)
    else:
        from matplotlib.font_manager import FontProperties
        from matplotlib.textpath import TextPath
        path = TextPath((0, 0), r.name, size=height, prop=FontProperties(family="DejaVu Sans", weight="bold"))
        shape = Polygon()
        for poly in path.to_polygons():
            p = Polygon(poly)
            if not p.is_valid:
                p = p.buffer(0)
            shape = shape.symmetric_difference(p)      # counters (the hole in O) cancel
    x0, y0, x1, y1 = shape.bounds
    shape = affinity.translate(shape, -(x0 + x1) / 2, -(y0 + y1) / 2)
    # Reading from the front: the text runs along -y, its top toward -x... turned to face a
    # viewer standing ahead of the robot.
    shape = affinity.rotate(shape, 90, origin=(0, 0))
    return affinity.translate(shape, r.plate_x / 2 - 5.0, 0.0)


def hub_outline(r: Robot, cx: float = 0.0, cy: float = 0.0) -> Polygon:
    bore = hole(0, 0, r.hub_bore_d).intersection(box(-5, -5, 5, r.hub_flat))
    screws = [hole(r.hub_screw_pcd / 2 * math.cos(a), r.hub_screw_pcd / 2 * math.sin(a), CLEARANCE["M2"])
              for a in (math.radians(30), math.radians(150), math.radians(270))]
    part = hole(0, 0, r.hub_d).difference(unary_union([bore] + screws))
    return affinity.translate(part, cx, cy)


def shaft_section(r: Robot) -> Polygon:
    m = r.motor
    return hole(0, 0, m["shaft_d_mm"]).intersection(box(-5, -5, 5, m["shaft_flat_mm"]))


# ---- printed parts as stacked bands -----------------------------------------------------
# A printed part is a stack of (z0, z1, footprint) bands, each an extrusion of a shapely
# polygon: enough for clamps, trays and a rim whose groove is stepped finer than a layer.

TRAY = {"x": 47.0, "y": 21.0, "base": 2.0, "wall_h": 3.0, "wall": 1.5, "tab_x": 8.0, "hole_y": 13.5,
        "pockets": [(-11.65, 21.4, 18.2), (11.9, 20.7, 13.1)],   # (x, size x, size y): XIAO, DRV8833
        "usb_notch_y": 10.0}     # the XIAO's USB-C overhangs its board by 1.53 mm (Seeed's STEP), 9 mm wide


def motor_clamp(r: Robot) -> List[Tuple[float, float, Polygon]]:
    """Over the gearbox, open at the bottom: x across the motor, y along it, as Pololu's
    #989 bracket, printed the way it is used (the 12 mm bridge prints across)."""
    m = r.motor
    cw, ch = m["body_mm"][0] + 2 * r.clamp_clear, m["body_mm"][1] - r.clamp_squeeze
    ow = cw + 2 * r.clamp_wall
    d = r.clamp_depth
    channel = box(-cw / 2, -d, cw / 2, d)
    holes = unary_union([hole(i * r.clamp_hole_spacing / 2, 0, CLEARANCE["M2"]) for i in (-1, 1)])
    tabs = box(-r.clamp_span / 2, -d / 2, r.clamp_span / 2, d / 2).difference(channel).difference(holes)
    walls = box(-ow / 2, -d / 2, ow / 2, d / 2).difference(channel)
    return [(0.0, r.clamp_tab_t, tabs), (r.clamp_tab_t, ch, walls), (ch, ch + r.clamp_wall, box(-ow / 2, -d / 2, ow / 2, d / 2))]


def rim(r: Robot, step: float = 0.1) -> List[Tuple[float, float, Polygon]]:
    """Printed on its inner face (the hub's side, z = 0). A web, a boss for the hub's three
    M2 screws, the outer ring with the tyre groove: 45-degree flanks, so it prints without
    support, around a 1 mm root at the groove's diameter."""
    screws = [hole(r.hub_screw_pcd / 2 * math.cos(a), r.hub_screw_pcd / 2 * math.sin(a), M2_TAP_PLA_MM)
              for a in (math.radians(30), math.radians(150), math.radians(270))]
    bore = unary_union([hole(0, 0, r.rim_bore_d)] + screws)
    R = r.rim_flange_d / 2
    root = r.groove_root_d / 2
    depth = R - root
    mid = r.rim_width / 2

    def outer(z: float) -> float:
        dz = abs(z - mid)
        flank = 0.5 + depth               # root 1 mm wide, then 45 degrees out to the flange
        if dz <= 0.5:
            return root
        if dz <= flank:
            return root + (dz - 0.5)
        return R

    bands = []
    z = 0.0
    while z < r.rim_width - 1e-9:
        z1 = min(r.rim_width, z + step)
        disc = hole(0, 0, 2 * outer(0.5 * (z + z1)))
        if z >= r.rim_web:
            disc = disc.difference(hole(0, 0, 2 * r.rim_recess_r).difference(hole(0, 0, 2 * r.rim_boss_r)))
        bands.append((z, z1, disc.difference(bore)))
        z = z1
    return _merge(bands)


def tray(r: Robot) -> List[Tuple[float, float, Polygon]]:
    t = TRAY
    body = box(-t["x"] / 2, -t["y"] / 2, t["x"] / 2, t["y"] / 2)
    tabs = box(-t["tab_x"] / 2, -t["hole_y"] - 3.0, t["tab_x"] / 2, t["hole_y"] + 3.0)
    holes = unary_union([hole(0, j * t["hole_y"], CLEARANCE["M2"]) for j in (-1, 1)])
    pockets = unary_union([box(x - sx / 2, -sy / 2, x + sx / 2, sy / 2) for x, sx, sy in t["pockets"]])
    (xx, sx, _) = t["pockets"][0]
    pockets = pockets.union(box(-t["x"] / 2 - 1.0, -t["usb_notch_y"] / 2, xx - sx / 2 + 0.5, t["usb_notch_y"] / 2))
    return [(0.0, t["base"], body.union(tabs).difference(holes)),
            (t["base"], t["base"] + t["wall_h"], body.difference(pockets))]


def _merge(bands):
    """Join consecutive bands with the same footprint."""
    out = []
    for z0, z1, p in bands:
        if out and out[-1][2].equals(p):
            out[-1] = (out[-1][0], z1, p)
        else:
            out.append((z0, z1, p))
    return out


def band_volume_mm3(bands) -> float:
    return sum((z1 - z0) * p.area for z0, z1, p in bands)


# ---- meshes ------------------------------------------------------------------------------

def _prism(poly: Polygon, z0: float, z1: float) -> np.ndarray:
    """Triangles (n x 3 x 3) of one closed prism over a polygon (holes allowed)."""
    tris = []
    for t in shapely.constrained_delaunay_triangles(poly).geoms:
        a, b, c = list(t.exterior.coords)[:3]
        ccw = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0
        if not ccw:
            b, c = c, b
        tris.append([(a[0], a[1], z1), (b[0], b[1], z1), (c[0], c[1], z1)])    # top, facing +z
        tris.append([(a[0], a[1], z0), (c[0], c[1], z0), (b[0], b[1], z0)])    # bottom, facing -z
    oriented = shapely.geometry.polygon.orient(poly, sign=1.0)
    for ring in [oriented.exterior, *oriented.interiors]:
        pts = list(ring.coords)
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            # Exterior counter-clockwise, holes clockwise (orient): the material is to the left.
            tris.append([(x0, y0, z0), (x1, y1, z0), (x1, y1, z1)])
            tris.append([(x0, y0, z0), (x1, y1, z1), (x0, y0, z1)])
    return np.asarray(tris, dtype=np.float64)


def part_triangles(bands, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> np.ndarray:
    out = []
    for z0, z1, p in bands:
        polys = list(p.geoms) if isinstance(p, MultiPolygon) else [p]
        for q in polys:
            if not q.is_empty:
                out.append(_prism(q, z0, z1))
    tris = np.concatenate(out)
    tris[:, :, 0] += dx
    tris[:, :, 1] += dy
    tris[:, :, 2] += dz
    return tris


def write_stl(path: Path, tris: np.ndarray, name: str = "omnisim") -> None:
    """Binary STL (mm)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with path.open("wb") as f:
        f.write(name.encode()[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


# The print plate: every printed part, as laid out on the SV08's bed (mm from the plate's
# centre; cura_slice.py centres the plate's footprint on the bed).
PLATE = [("motor_clamp", motor_clamp, (-30.0, -25.0)), ("motor_clamp", motor_clamp, (0.0, -25.0)),
         ("rim", rim, (-22.0, 22.0)), ("rim", rim, (22.0, 22.0)), ("tray", tray, (32.0, -25.0))]


def print_plate(r: Robot) -> np.ndarray:
    return np.concatenate([part_triangles(fn(r), dx, dy) for _, fn, (dx, dy) in PLATE])


# ---- jobs ---------------------------------------------------------------------------------

def laser_jobs(r: Robot, kerf_mm: float = LASER_KERF_MM) -> Dict[str, str]:
    """One job per plate (each is cut from its own sheet; a sheet at the twin's 0.05 mm is
    at most 99 mm across). Sienci's 3 mm plywood settings: 100 %, 2.2 mm/s, 3 passes,
    Z down 1 mm a pass; engraving 100 % at 25.4 mm/s."""
    return {
        "bottom_plate": laser_job.program([bottom_plate(r)], None, kerf_mm=kerf_mm,
                                          title="desk robot bottom plate, 3 mm plywood"),
        "top_plate": laser_job.program([top_plate(r)], name_engraving(r), kerf_mm=kerf_mm,
                                       title="desk robot top plate, 3 mm plywood, engraved"),
    }


HUB_STOCK_MM = (48.0, 26.0, 4.0)        # 6061 flat bar, 4 mm, sawn to 48 mm, taped down
HUB_CENTRES = ((-13.0, 0.0), (13.0, 0.0))
THROUGH_MM = 0.2                         # into the tape and spoilboard under the blank


class _FloorWriter(toolpath.GcodeWriter):
    """Plunges in rapid to just above the floor the previous depth pass left, then feeds:
    toolpath.clear() cuts a feature pass by pass over its whole area, so everything above
    (this pass's depth + one step) is already air. Feeding the tool down through that air
    from the safe height made the 1 mm tool's bores take 1.8 hours."""

    step = 0.0

    def plunge_to(self, z: float) -> None:
        above = z + self.step + 0.1
        if above < 0.0:
            self.emit(f"G0 Z{above:.3f}")
        super().plunge_to(z)


def hub_program(r: Robot, tip_above_top_mm: float) -> str:
    """Both hubs from one blank, in work coordinates on the blank's top centre.

    T3 (1 mm) cuts each D-bore and the three M2 holes while the hubs are still held by the
    blank; T1 (1/4 in) then cuts each hub free along its outline. The blank is taped down
    (blue tape and CA glue, the hobby-CNC way), so the hubs stay put when they come free.
    Feeds are conservative for a trim router: 1 mm tool 200 mm/min, 0.2 mm a pass; 1/4 in
    single flute 900 mm/min, 0.5 mm a pass. Retracts go 1 mm above the blank (nothing else
    stands on the spoilboard) and plunges rapid to just above the last pass's floor."""
    t = r.hub_t + THROUGH_MM
    lines = ["(desk robot wheel hubs: two from one 48 x 26 x 4 mm 6061 blank)", "G21 G90 G17 G94",
             f"G10 L20 P1 X0 Y0 Z{tip_above_top_mm:.3f}", "G54"]
    w = _FloorWriter(1.0, 200.0, 40.0)
    w.step = 0.2
    w.emit("G0 Z5")
    w.emit("T3 M6")
    w.emit("M3 S24000")
    w.emit("G4 P1")
    for cx, cy in HUB_CENTRES:
        outline = hub_outline(r, cx, cy)
        for interior in outline.interiors:
            toolpath.clear(w, Polygon(interior), 1.0, t, 0.4, w.step)
    w.emit("M5")
    w.feed, w.plunge, w.step = 900.0, 200.0, 0.5
    w.emit("G0 Z5")
    w.emit("T1 M6")
    w.emit("M3 S18000")
    w.emit("G4 P1")
    ring = unary_union([hole(cx, cy, r.hub_d + 2 * 6.85).difference(hole(cx, cy, r.hub_d)) for cx, cy in HUB_CENTRES])
    toolpath.clear(w, ring, 6.35, t, 3.0, w.step)
    w.emit("M5")
    w.emit("G0 Z20")
    w.emit("M2")
    return "\n".join(lines + w.lines) + "\n"


# ---- the bill of materials and the mass -------------------------------------------------

def made_parts(r: Robot) -> List[dict]:
    bottom, top = bottom_plate(r), top_plate(r)
    hub = hub_outline(r)
    return [
        {"part": "bottom_plate", "process": "laser", "material": "3 mm birch plywood", "qty": 1,
         "g": bottom.area * r.plate_t / 1000 * PLYWOOD_G_CM3},
        {"part": "top_plate", "process": "laser", "material": "3 mm birch plywood", "qty": 1,
         "g": top.area * r.plate_t / 1000 * PLYWOOD_G_CM3},
        {"part": "motor_clamp", "process": "printing", "material": "PLA", "qty": 2,
         "g": band_volume_mm3(motor_clamp(r)) / 1000 * PLA_G_CM3},
        {"part": "rim", "process": "printing", "material": "PLA", "qty": 2,
         "g": band_volume_mm3(rim(r)) / 1000 * PLA_G_CM3},
        {"part": "tray", "process": "printing", "material": "PLA", "qty": 1,
         "g": band_volume_mm3(tray(r)) / 1000 * PLA_G_CM3},
        {"part": "hub", "process": "cnc", "material": "6061 aluminium", "qty": 2,
         "g": hub.area * r.hub_t / 1000 * ALUMINIUM_G_CM3},
    ]


def bom(r: Robot) -> dict:
    import json
    import electrical
    import wiring
    made = made_parts(r)
    bought = [{"part": k, **v} for k, v in BOUGHT.items()]
    lengths = None
    if wiring.ROUTED.exists():
        lengths = {w["id"]: w["cut_mm"] for w in json.loads(wiring.ROUTED.read_text(encoding="utf-8")) if "cut_mm" in w}
    em = electrical.electrical_mass_g(r, lengths)
    bought.append({"part": "electrical", "qty": 1, "g": round(em["total"], 2),
                   "spec": f"switch, regulator, fuse, diode, capacitors, resistors, JST SM pairs ({em['parts']:.2f} g) "
                           f"and the routed wires ({em['wire']:.2f} g)", "source": "electrical.py; release/electrical/"})
    total = sum(p["g"] * p["qty"] for p in made) + sum(b["g"] * b["qty"] for b in bought)
    return {"made": made, "bought": bought, "total_g": round(total, 1),
            "note": "printed masses are solid-body volumes x PLA density: an upper bound (the slicer's infill makes them lighter)"}


def checks(r: Robot) -> dict:
    """The fits the design depends on, each as a number."""
    m = r.motor
    shaft = shaft_section(r)
    bore = hole(0, 0, r.hub_bore_d).intersection(box(-5, -5, 5, r.hub_flat))
    # What a 1 mm end mill can make of the D-bore: the bore opened and closed by its radius.
    milled = bore.buffer(-0.5, quad_segs=32).buffer(0.5, quad_segs=32)
    shaft_in_hub = shaft.difference(milled).area
    return {
        "tyre_stretch_pct": round(100 * r.tyre_stretch, 2),
        "wheel_d_mm": round(r.wheel_d, 3),
        "axle_z_mm": round(r.axle_z, 3),
        "plate_underside_mm": round(r.plate_top_z - r.plate_t, 3),
        "caster_mm": round(r.caster_z, 3),
        "level_error_mm": round(r.plate_top_z - r.plate_t - r.caster_z, 3),
        "track_mm": round(r.track, 2),
        "shaft_overlap_with_milled_bore_mm2": round(shaft_in_hub, 5),
        "shaft_clear_of_rim_mm": round((r.rim_bore_d - m["shaft_d_mm"]) / 2, 3),
        "shaft_end_inside_rim_mm": round(r.hub_gap + r.hub_t + r.rim_width - m["shaft_len_mm"], 3),
        "clamp_holes_from_plate_edge_mm": round(r.plate_y / 2 - r.clamp_y - CLEARANCE["M2"] / 2, 3),
        "speed_no_load_m_s": round(m["no_load_rpm"] / 60 * math.pi * r.wheel_d / 1000, 3),
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the desk robot's parts, jobs, BOM and checks.")
    p.add_argument("--out", required=True)
    p.add_argument("--hub-tip-above", type=float, default=10.0,
                   help="the CNC twin's tool tip above the hub blank's top at power-up (mm)")
    a = p.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r = Robot()
    for name, text in laser_jobs(r).items():
        (out / f"{name}.gcode").write_text(text, encoding="utf-8", newline="\n")
    write_stl(out / "print_plate.stl", print_plate(r), "desk robot print plate")
    for name, fn in (("motor_clamp", motor_clamp), ("rim", rim), ("tray", tray)):
        write_stl(out / f"{name}.stl", part_triangles(fn(r)), name)
    (out / "hubs.nc").write_text(hub_program(r, a.hub_tip_above), encoding="utf-8", newline="\n")
    (out / "bom.json").write_text(json.dumps(bom(r), indent=1), encoding="utf-8")
    report = {"robot": asdict(r), "checks": checks(r)}
    (out / "design.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    print(json.dumps(report["checks"], indent=1))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
