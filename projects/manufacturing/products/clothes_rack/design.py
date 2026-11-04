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

"""A clothes rack, designed for the machine we have: a LongMill MK2 router.

The concept is a steel or aluminium tube frame: a top bar on two uprights, each upright
standing on a base tube with two levelling feet. A router cannot make tube, and the
concept's joints are welded or cast. So:

- The TUBES are bought: 30 x 2 mm aluminium round tube, sawn to the cut list.
- The four JOINTS are the machined part: one connector design used four times. It is a
  two-piece clamp milled from 100 x 20 mm aluminium flat bar sawn to 100 mm squares. Each
  half carries a half bore for the through tube (the top bar, or a base tube) and a half
  bore for the upright, at right angles, plus four bolt holes. Both halves are the same
  part, so one program makes all eight.
- The bores are 30.2 mm across and the split faces stand 0.5 mm back from the bore
  centre, so a closed pair leaves 1 mm between the halves: four M6 bolts clamp any tube
  from 29.2 to 30.2 mm. No welding, no threads in the aluminium, no second setup.

Units: mm and kg unless a name says otherwise. A connector half is described in its
MILLING frame (work coordinates, Z0 = stock top, the part at Z <= 0, X/Y centred on the
blank) and in its CONNECTOR frame: the origin where the two tube axes cross, x along the
through tube, y along the upright (the upright leaves toward -y), z across the split
(half A at z < 0, half B, the same part turned 180 degrees about y, at z > 0).

    python projects/manufacturing/products/clothes_rack/design.py --out .tmp/clothes_rack
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

MANUFACTURING = Path(__file__).resolve().parents[2]
SERVICE = MANUFACTURING / "cnc"           # the CNC twin that mills the connectors
ROOT = MANUFACTURING.parents[1]
if str(SERVICE) not in sys.path:
    sys.path.insert(0, str(SERVICE))
import toolpath3d  # noqa: E402
import tool_table as tools_mod  # noqa: E402

TOOLS = MANUFACTURING / "machines" / "longmill_mk2_30x30" / "longmill_mk2_tools_aluminium.json"

# ---- materials ------------------------------------------------------------------------
ALUMINIUM_KG_M3 = 2700.0         # 6061 / 6063
ALUMINIUM_E_MPA = 69000.0
TUBE_YIELD_MPA = 170.0           # 6063-T6, EN 755-2 minimum Rp0.2 (T5 is 110)
STEEL_KG_M3 = 7850.0


@dataclass(frozen=True)
class Rack:
    bar_mm: float = 1200.0           # top bar length
    bar_top_mm: float = 1500.0       # floor to the top of the bar
    base_mm: float = 600.0           # base tube length (the rack's depth); the concept's 500 tipped at ~9 N empty
    upright_x_mm: float = 540.0      # uprights at +-this from the centre
    foot_y_mm: float = 270.0         # levelling feet at +-this along each base tube
    foot_h_mm: float = 20.0          # floor to the base tube's underside, feet screwed in
    tube_d_mm: float = 30.0
    tube_wall_mm: float = 2.0
    end_gap_mm: float = 0.5          # an upright stops this short of the tube it meets

    # The connector half, milled from a BLOCK_MM square of BLOCK_T_MM flat bar.
    block_mm: float = 100.0
    block_t_mm: float = 20.0
    bore_r_mm: float = 15.1
    relief_mm: float = 0.5           # bore centre above the split face
    through_y_mm: float = 15.0       # the through tube's axis, milling frame
    hole_d_mm: float = 7.0           # M6 clearance, ISO 273 coarse
    holes_xy: Tuple[Tuple[float, float], ...] = ((-32.0, 40.0), (32.0, 40.0), (-32.0, -25.0), (32.0, -25.0))
    hole_bottom_mm: float = -20.3    # through, 0.3 mm into the spoilboard

    # ---- derived ----
    @property
    def tube_r(self) -> float:
        return self.tube_d_mm / 2.0

    @property
    def bar_z(self) -> float:
        """Height of the top bar's axis."""
        return self.bar_top_mm - self.tube_r

    @property
    def base_z(self) -> float:
        """Height of the base tubes' axes."""
        return self.foot_h_mm + self.tube_r

    @property
    def upright_mm(self) -> float:
        """Upright length: from the base tube's top to the bar's underside, less the gaps."""
        return (self.bar_z - self.tube_r) - (self.base_z + self.tube_r) - 2 * self.end_gap_mm

    @property
    def tube_kg_m(self) -> float:
        ri = self.tube_r - self.tube_wall_mm
        return ALUMINIUM_KG_M3 * math.pi * (self.tube_r ** 2 - ri ** 2) * 1e-6


# ---- the connector half ---------------------------------------------------------------
def connector_surface(rack: Rack, x: np.ndarray, y: np.ndarray, holes: bool = True) -> np.ndarray:
    """Work Z of the half's milled face at (x, y), milling frame. Defined beyond the blank
    too (the bores run on), so a cutter can finish a bore out past the blank's edge."""
    x, y = np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)
    r, c = rack.bore_r_mm, rack.relief_mm
    z = np.zeros(np.broadcast(x, y).shape)
    dy = y - rack.through_y_mm
    z = np.minimum(z, np.where(np.abs(dy) < r, c - np.sqrt(np.maximum(r * r - dy * dy, 0.0)), 0.0))
    z = np.minimum(z, np.where((y <= rack.through_y_mm) & (np.abs(x) < r),
                               c - np.sqrt(np.maximum(r * r - x * x, 0.0)), 0.0))
    if holes:
        for hx, hy in rack.holes_xy:
            z = np.where(np.hypot(x - hx, y - hy) < rack.hole_d_mm / 2.0, -rack.block_t_mm, z)
    return np.maximum(np.minimum(z, 0.0), -rack.block_t_mm)


def half_grid(rack: Rack, res: float = 0.25, holes: bool = True, margin: float = 0.0):
    """The half's face sampled on the blank (plus `margin` all round): (z, x0, y0)."""
    h = rack.block_mm / 2.0 + margin
    xs = np.arange(-h, h + res / 2, res)
    X, Y = np.meshgrid(xs, xs)
    return connector_surface(rack, X, Y, holes), float(xs[0]), float(xs[0])


def half_volume_mm3(rack: Rack, res: float = 0.1) -> float:
    z, _, _ = half_grid(rack, res)
    # Cell-centred: drop the last row/column of the node grid.
    return float((z[:-1, :-1] + rack.block_t_mm).sum() * res * res)


def write_half_stl(rack: Rack, path: Path, res: float = 0.25, metres: bool = False, zmap: Optional[dict] = None,
                   frame: str = "milling") -> int:
    """The half as a closed mesh. `zmap` (inspect_part.load_zmap): the half as the twin milled
    it, not as drawn. frame "connector" puts it in the connector frame (half A)."""
    if zmap is None:
        z, x0, y0 = half_grid(rack, res)
    else:
        step = max(1, int(round(res / zmap["res_mm"])))
        z = -zmap["depth_mm"][::step, ::step]
        res = zmap["res_mm"] * step
        x0 = -(zmap["nx"] - 1) * zmap["res_mm"] / 2.0
        y0 = -(zmap["ny"] - 1) * zmap["res_mm"] / 2.0
    tris = toolpath3d.heightfield_solid(z, x0, y0, res, -rack.block_t_mm)
    if frame == "connector":
        tris = tris - np.array([0.0, rack.through_y_mm, rack.relief_mm])
    if metres:
        tris = tris / 1000.0
    toolpath3d.write_stl(path, tris, b"OmniSim clothes rack connector half")
    return len(tris)


# ---- the program ----------------------------------------------------------------------
# Aluminium on a LongMill: Sienci cuts it with a 1/4" single-flute end mill. These are a
# STARTING POINT from chip-load arithmetic, not a published LongMill table: run the first
# half as a test cut, with an air blast or mist, and listen.
#   rough: 18,000 rpm x 1 flute x 0.05 mm/tooth = 900 mm/min; 0.5 mm deep, 2.5 mm (40 %) over
#   ball:  18,000 rpm x 2 flutes x 0.022 mm/tooth = 800 mm/min; 0.4 mm stepover
FEEDS = {"rpm": 18000.0, "rough_feed": 900.0, "plunge": 150.0, "step_down": 0.5, "rough_stepover": 2.5,
         "allowance": 0.3, "finish_feed": 800.0, "finish_plunge": 300.0, "finish_stepover": 0.4,
         "hole_feed": 300.0}


def program(rack: Rack, table: Optional[Dict[int, tools_mod.Tool]] = None, res: float = 0.1) -> Tuple[str, dict]:
    """The G-code for one half: rough, bore the four holes, change to the ball, finish."""
    table = table or tools_mod.load(TOOLS)
    margin = table[1].radius_mm + 1.0
    z, x0, y0 = half_grid(rack, res, holes=False, margin=margin)
    surface = toolpath3d.Grid(z, x0, y0, res)
    holes = [(hx, hy, rack.hole_d_mm, rack.hole_bottom_mm) for hx, hy in rack.holes_xy]
    return toolpath3d.job(surface, table[1], table[2], holes=holes, safe_z=3.0,
                          title="clothes rack connector half, 6061 aluminium, 100 x 100 x 20 blank", **FEEDS)


# ---- where everything goes ------------------------------------------------------------
def _frame(x_axis, y_axis) -> np.ndarray:
    x, y = np.asarray(x_axis, float), np.asarray(y_axis, float)
    return np.column_stack([x, y, np.cross(x, y)])


def connectors(rack: Rack) -> List[dict]:
    """The four connectors, assembled: origin (world mm) and rotation (connector -> world).
    Half A sits outboard on the feet and at the back on the top bar."""
    out = []
    for side, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * rack.upright_x_mm
        # Foot: the base tube runs along world Y, the upright leaves upward (connector -y = +Z).
        out.append({"name": f"foot_{side}", "origin": [x, 0.0, rack.base_z],
                    "R": _frame([0, sx, 0], [0, 0, -1])})
        # Top: the bar runs along world X, the upright leaves downward (connector -y = -Z).
        out.append({"name": f"top_{side}", "origin": [x, 0.0, rack.bar_z], "R": _frame([1, 0, 0], [0, 0, 1])})
    return out


def tubes(rack: Rack) -> List[dict]:
    """Every tube, assembled: centre, axis, length."""
    out = [{"name": "bar", "centre": [0.0, 0.0, rack.bar_z], "axis": [1, 0, 0], "length": rack.bar_mm}]
    mid = 0.5 * ((rack.base_z + rack.tube_r) + (rack.bar_z - rack.tube_r))
    for side, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * rack.upright_x_mm
        out.append({"name": f"upright_{side}", "centre": [x, 0.0, mid], "axis": [0, 0, 1], "length": rack.upright_mm})
        out.append({"name": f"base_{side}", "centre": [x, 0.0, rack.base_z], "axis": [0, 1, 0],
                    "length": rack.base_mm})
    return out


# ---- bill of materials ----------------------------------------------------------------
BOLT = {"spec": "M6 x 55 socket head cap screw, A2 stainless", "kg": 0.0155}
NUT = {"spec": "M6 nyloc nut, A2", "kg": 0.0025}
WASHER = {"spec": "M6 flat washer (DIN 125), A2", "kg": 0.0011}
FOOT = {"spec": "M8 levelling glide, 30 mm base, 20 mm set height", "kg": 0.030}
RIVNUT = {"spec": "M8 rivet nut for 2 mm wall (drill 11 mm)", "kg": 0.004}
CAP = {"spec": "30 mm round tube insert end cap, 1.5-2 mm wall", "kg": 0.004}


def bom(rack: Rack) -> List[dict]:
    half_kg = half_volume_mm3(rack, 0.25) * 1e-9 * ALUMINIUM_KG_M3
    kgm = rack.tube_kg_m
    rows = [
        {"item": "Top bar", "qty": 1, "spec": f"30 x 2 mm aluminium round tube 6063-T6, {rack.bar_mm:.0f} mm",
         "make": "saw", "kg_each": kgm * rack.bar_mm / 1000},
        {"item": "Upright", "qty": 2, "spec": f"30 x 2 mm aluminium round tube 6063-T6, {rack.upright_mm:.0f} mm",
         "make": "saw", "kg_each": kgm * rack.upright_mm / 1000},
        {"item": "Base tube", "qty": 2, "spec": f"30 x 2 mm aluminium round tube 6063-T6, {rack.base_mm:.0f} mm",
         "make": "saw", "kg_each": kgm * rack.base_mm / 1000},
        {"item": "Connector half", "qty": 8,
         "spec": f"6061-T6 aluminium, {rack.block_mm:.0f} x {rack.block_mm:.0f} x {rack.block_t_mm:.0f} mm blank "
                 f"(100 x 20 flat bar sawn to {rack.block_mm:.0f} mm)", "make": "CNC", "kg_each": half_kg},
        {"item": "Bolt", "qty": 16, "spec": BOLT["spec"], "make": "buy", "kg_each": BOLT["kg"]},
        {"item": "Nut", "qty": 16, "spec": NUT["spec"], "make": "buy", "kg_each": NUT["kg"]},
        {"item": "Washer", "qty": 32, "spec": WASHER["spec"], "make": "buy", "kg_each": WASHER["kg"]},
        {"item": "Levelling foot", "qty": 4, "spec": FOOT["spec"], "make": "buy", "kg_each": FOOT["kg"]},
        {"item": "Rivet nut", "qty": 4, "spec": RIVNUT["spec"], "make": "buy", "kg_each": RIVNUT["kg"]},
        {"item": "End cap", "qty": 6, "spec": CAP["spec"], "make": "buy", "kg_each": CAP["kg"]},
    ]
    for r in rows:
        r["kg_each"] = round(r["kg_each"], 4)
        r["kg"] = round(r["kg_each"] * r["qty"], 3)
    return rows


# ---- checks ---------------------------------------------------------------------------
def checks(rack: Rack, load_kg: float = 20.0, bolt_torque_nm: float = 5.0) -> dict:
    """Hand calculations for the assembled rack carrying `load_kg` of clothes on the bar."""
    g = 9.81
    rows = bom(rack)
    rack_kg = sum(r["kg"] for r in rows)
    ro, ri = rack.tube_r, rack.tube_r - rack.tube_wall_mm
    I = math.pi / 4.0 * (ro ** 4 - ri ** 4)                     # mm^4
    span = 2 * rack.upright_x_mm
    w = load_kg * g / span                                      # N/mm, spread along the span
    deflect = 5 * w * span ** 4 / (384 * ALUMINIUM_E_MPA * I)   # simply supported: the clamps only help
    stress = (w * span ** 2 / 8) * ro / I
    total_n = (rack_kg + load_kg) * g
    # Tipping forward or back about one base tube's feet line, pushed at the bar.
    tip_n = total_n * rack.foot_y_mm / rack.bar_z
    tip_empty_n = rack_kg * g * rack.foot_y_mm / rack.bar_z
    # Bolt preload from torque, F = T / (K d), K = 0.2 (dry steel, the common rule).
    preload = bolt_torque_nm * 1000 / (0.2 * 6.0)
    # A tube slips in its clamp when the pull beats friction on both halves (mu 0.3, Al on Al).
    grip_n = 0.3 * 2 * 2 * preload                               # two bolts on each tube
    return {
        "rack_kg": round(rack_kg, 2), "load_kg": load_kg,
        "bar_deflection_mm": round(deflect, 2), "bar_stress_mpa": round(stress, 1),
        "bar_safety_factor": round(TUBE_YIELD_MPA / stress, 1) if stress > 0 else None,
        "tip_force_at_bar_n": round(tip_n, 1), "tip_force_empty_n": round(tip_empty_n, 1),
        "slide_force_n": {"mu_0.5": round(0.5 * total_n, 1)},
        "bolt_preload_n": round(preload), "tube_grip_n": round(grip_n),
        "upright_load_n": round(total_n / 2, 1),
        "assumptions": ["bar simply supported over the upright span (clamped ends deflect less)",
                        "clothes hang at the bar's centre line; the rack's own mass centred",
                        "bolt preload F = T/(0.2 d); friction 0.3 aluminium on aluminium"],
    }


GARMENT_X_M = (-0.30, -0.21, -0.12, -0.03, 0.06, 0.15)   # where the six hangers hang on the bar
GARMENT_DROP_M = 0.4                                      # a garment's centre of mass below the bar top


def mass_properties(rack: Rack, load_kg: float = 20.0) -> dict:
    """Mass, centre of mass and inertia (about the centre of mass, world axes; kg, m, kg m^2)
    of the assembled rack with `load_kg` of clothes shared over the six hangers."""
    mm = 1e-3
    rows = {r["item"]: r for r in bom(rack)}
    items = []                                     # (mass, centre, inertia about its own centre)
    ro, ri = rack.tube_r * mm, (rack.tube_r - rack.tube_wall_mm) * mm
    for t in tubes(rack):
        m = rack.tube_kg_m * t["length"] * mm
        L = t["length"] * mm
        along, across = 0.5 * m * (ro ** 2 + ri ** 2), m * (3 * (ro ** 2 + ri ** 2) + L ** 2) / 12
        a = np.abs(np.asarray(t["axis"], float))
        items.append((m, np.asarray(t["centre"]) * mm, np.diag(across + (along - across) * a)))
    # A connector with its 4 bolts, nuts and 8 washers: a 100 x 100 x 41 mm block.
    per = 2 * rows["Connector half"]["kg_each"] + 4 * (rows["Bolt"]["kg_each"] + rows["Nut"]["kg_each"]
                                                      + 2 * rows["Washer"]["kg_each"])
    for c in connectors(rack):
        s = np.array([0.1, 0.1, 0.041])
        local = per / 12 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2, s[0] ** 2 + s[1] ** 2])
        items.append((per, np.asarray(c["origin"]) * mm, c["R"] @ local @ c["R"].T))
    small = rows["Levelling foot"]["kg_each"] + rows["Rivet nut"]["kg_each"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            items.append((small, np.array([sx * rack.upright_x_mm, sy * rack.foot_y_mm, 15.0]) * mm, np.zeros((3, 3))))
            items.append((rows["End cap"]["kg_each"], np.array([sx * rack.upright_x_mm, sy * rack.base_mm / 2,
                                                                 rack.base_z]) * mm, np.zeros((3, 3))))
        items.append((rows["End cap"]["kg_each"], np.array([sx * rack.bar_mm / 2, 0.0, rack.bar_z]) * mm, np.zeros((3, 3))))
    for x in GARMENT_X_M:                          # each garment: a 0.36 x 0.62 m panel, 30 mm thick
        m = load_kg / len(GARMENT_X_M)
        items.append((m, np.array([x, 0.0, rack.bar_top_mm * mm - GARMENT_DROP_M]),
                      m / 12 * np.diag([0.36 ** 2 + 0.62 ** 2, 0.03 ** 2 + 0.62 ** 2, 0.03 ** 2 + 0.36 ** 2])))
    M = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / M
    I = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - com
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return {"mass_kg": float(M), "com_m": com.tolist(), "inertia_kg_m2": I.tolist()}


# ---- inspection -----------------------------------------------------------------------
def _width_at(u: np.ndarray, z: np.ndarray, level: float) -> float:
    """Width of the cut where a section z(u) crosses `level`, interpolated between samples."""
    inside = np.flatnonzero(z < level)
    if inside.size == 0:
        return 0.0
    a, b = inside[0], inside[-1]

    def cross(i0, i1):
        return u[i0] + (level - z[i0]) * (u[i1] - u[i0]) / (z[i1] - z[i0])
    return float(cross(b, b + 1) - cross(a - 1, a))


def inspect_half(rack: Rack, zmap: dict, ideal: Optional[np.ndarray] = None) -> dict:
    """A milled half (inspect_part.load_zmap, work X0 Y0 at the blank's centre) measured the
    way a clamp is judged: each bore's depth and width across it, the holes' diameter and
    position, the split face, and the bore surfaces' deviation from the drawing along
    their normal."""
    res = zmap["res_mm"]
    part = -zmap["depth_mm"]
    n = part.shape[0]
    xs = -(n - 1) * res / 2 + np.arange(n) * res
    X, Y = np.meshgrid(xs, xs)
    target = connector_surface(rack, X, Y)
    t = rack.block_t_mm

    r, c = rack.bore_r_mm, rack.relief_mm
    levels = (-1.0, -7.0)                      # just under the split face, and half way down

    def sections(axis: str, at) -> dict:
        """Cross-sections of a bore: its depth on the centre line and its width at `levels`."""
        rows = []
        for a in at:
            if axis == "bar":                 # across the bar bore at x = a: z over y
                i = int(round((a - xs[0]) / res))
                u, z = xs - rack.through_y_mm, part[:, i]
            else:                             # across the upright bore at y = a: z over x
                j = int(round((a - xs[0]) / res))
                u, z = xs, part[j, :]
            near = np.abs(u) < r + 2.0
            u, z = u[near], z[near]
            rows.append([float(z[np.argmin(np.abs(u))])] + [_width_at(u, z, lv) for lv in levels])
        rows = np.array(rows)
        drawn = [c - r] + [2 * math.sqrt(r * r - (lv - c) ** 2) for lv in levels]
        out = {"sections": len(rows)}
        for k, name in enumerate(["depth_mm"] + [f"width_at_{-lv:g}mm" for lv in levels]):
            v = rows[:, k]
            out[name] = {"drawn": round(drawn[k], 3), "mean": round(float(v.mean()), 4),
                         "min": round(float(v.min()), 4), "max": round(float(v.max()), 4)}
        return out

    holes = []
    through = part <= -t + 1e-6
    for hx, hy in rack.holes_xy:
        m = through & (np.hypot(X - hx, Y - hy) < rack.hole_d_mm)
        area = float(m.sum()) * res * res
        holes.append({"at": [hx, hy], "diameter_mm": round(2 * math.sqrt(area / math.pi), 3),
                      "centre_error_mm": round(float(math.hypot(X[m].mean() - hx, Y[m].mean() - hy)), 4)})
    # Along the normal of the drawing, on the bores only (clear of the edges and the holes).
    gy, gx = np.gradient(target, res)
    k = 1.0 / np.sqrt(1.0 + gx * gx + gy * gy)
    bore = (target < -1.0) & (target > -t + 1e-6) & (np.abs(X) < rack.block_mm / 2 - 1) & (np.abs(Y) < rack.block_mm / 2 - 1)
    dev = ((part - target) * k)[bore]
    from scipy import ndimage
    # The split face: drawn flat for 0.5 mm all round (clear of every edge it meets).
    face = (ndimage.minimum_filter(target, size=2 * int(round(0.5 / res)) + 1) == 0.0)
    face &= (np.abs(X) < rack.block_mm / 2 - 1) & (np.abs(Y) < rack.block_mm / 2 - 1)
    out = {
        # The bar bore clear of the upright's (|x| > 16), the upright's clear of the bar's.
        "bar_bore": sections("bar", [-45, -40, -35, -30, -25, -20, 20, 25, 30, 35, 40, 45]),
        "upright_bore": sections("upright", [-45, -40, -35, -30, -25, -20]),
        "holes": holes,
        "bore_vs_drawing_normal_mm": {"mean": round(float(dev.mean()), 4), "p01": round(float(np.percentile(dev, 1)), 4),
                                      "p99": round(float(np.percentile(dev, 99)), 4), "min": round(float(dev.min()), 4),
                                      "max": round(float(dev.max()), 4), "points": int(dev.size)},
        "face_untouched_mm": round(float(np.abs(part[face]).max()), 4),
        "removed_mm3": round(zmap["removed_mm3"], 1),
    }
    if ideal is not None:          # the twin's own error: what it cut against a perfect machine
        d = ((part - np.maximum(ideal, -t)) * k)[bore]
        out["twin_vs_ideal_normal_mm"] = {"p01": round(float(np.percentile(d, 1)), 4),
                                          "p99": round(float(np.percentile(d, 99)), 4),
                                          "min": round(float(d.min()), 4), "max": round(float(d.max()), 4)}
    return out


def cut_list(rack: Rack) -> List[dict]:
    return [{"part": r["item"], "qty": r["qty"], "length_mm": float(r["spec"].rsplit(",", 1)[1].split()[0])}
            for r in bom(rack) if r["make"] == "saw"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--load-kg", type=float, default=20.0)
    ap.add_argument("--no-program", action="store_true", help="skip the G-code (the slow part)")
    args = ap.parse_args(argv)
    rack = Rack()
    args.out.mkdir(parents=True, exist_ok=True)
    n = write_half_stl(rack, args.out / "connector_half.stl")
    report = {"rack": {k: v for k, v in asdict(rack).items()}, "upright_mm": round(rack.upright_mm, 1),
              "bom": bom(rack), "cut_list": cut_list(rack), "checks": checks(rack, args.load_kg),
              "feeds": FEEDS, "tools": str(TOOLS.relative_to(ROOT).as_posix())}
    if not args.no_program:
        text, info = program(rack)
        (args.out / "connector_half.nc").write_text(text, encoding="utf-8", newline="\n")
        report["program"] = info
    (args.out / "rack.json").write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    print(f"wrote {args.out}: connector_half.stl ({n} triangles), rack.json"
          + ("" if args.no_program else f", connector_half.nc ({report['program']['lines']} lines)"))
    print(json.dumps(report["checks"], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
