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

"""SparkRover Mini: the chassis and the drive, designed against skeleton.py.

What it is, bottom up (robot frame of skeleton.py: mm, origin on the floor under the
wheels' centre, +x forward, +y left, +z up):

- DRIVE: four Pololu #4844 gearmotors (25D, 34:1, HP 12 V, 48 CPR encoder), one per
  wheel, direct drive. Sized in `drive()`: a 2 kg skid-steer turning on the spot needs
  1.05 kg.cm per wheel against the tyres' lateral scrub at mu 1.0; the motor stalls at
  5.5 kg.cm at the battery's 6.0 V floor (5.2x), and a spin-turn draws 24 % of the stall
  current (Pololu's rule: 25 %). Two 66.5 mm motors (21 + 30.8 + 14.7, Pololu's drawing)
  face each other on each axle with 1 mm between their encoders; the gearbox face is at
  |y| 67, the skeleton's base / wheel-well boundary.
- MOTOR MOUNTS: Pololu #2676 stamped aluminium L-brackets, bought, hung upside down
  from the deck on a 3 mm plywood spacer. The skeleton's DECISIONS name CNC aluminium
  for motor mounts "where a part takes load": the load is 4.9 N and 0.10 N.m per wheel,
  which a 2 mm aluminium L carries with a large margin, and a 3-axis mill makes an L
  only in two set-ups. So the bought bracket.
- WHEELS: DFRobot FIT0199 D65 x 26 mm rubber wheels (a standard 12 mm RC hex hub, 4 mm
  centre bore) on Pololu #2684 12 mm hex adapters for 4 mm shafts.
- BASE DECK: two layers of 3 mm birch plywood, laser-cut, glued: the lower one with
  clearance holes, the upper one with hex nut traps for the bracket screws (a nut in a
  trap, so a 12 mm screw reaches it and its tip stays under the base zone's top).
  Underside at z 50 (the lower SHELL_MOUNTS' plane), top at 56.
- SPINE FOOT: a CNC-milled 4 mm 6061 plate on the deck, z 56..60, under the spine. The
  spine is the part that takes the arms' and head's loads into the deck, and it fills
  the 4 mm between the deck's top and the spine zone's floor (z 60).
- SPINE: a glued box of 6 mm laser-cut birch plywood: two side plates at |y| 49..55
  (the SHELL_MOUNTS' upper screws and the SHOULDER_BOLTS pass through them), a rear
  board plate (the electrical bay's wall, a Raspberry Pi 5 hole pattern), a front board
  plate (the head zone's), and two rungs. Bolted down through the deck and the foot by
  four M3 screws into nuts captured in T-slots in the side plates.
- BATTERY: the electrical agent's 2S2P pack (four 18650s flat, 75 x 72 x 20 mm); here, a
  reserved volume under the deck, centred between the two rows of hung motors with 1.5 mm
  round the pack, held by a hook-and-loop strap through two deck slots just outboard of it
  (`BATTERY_HOLDER`).

Everything bought is in `BOUGHT` with its numbers and the page they come from; every
dimension of a made part is derived from those numbers or from the skeleton. The
vendors' drawings are 2D; our solids are drawn from them. No vendor STEP is fetched:
the gearmotor's product page lists none, and the drawings carry every number the fit
needs.

    python projects/manufacturing/products/spark_rover/chassis.py --out .tmp/spark_rover_chassis
"""
from __future__ import annotations

import argparse
import functools
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely import affinity
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
MANUFACTURING = HERE.parents[1]
DESK = MANUFACTURING / "products" / "desk_robot"
for _p in (HERE, DESK, MANUFACTURING / "cnc", MANUFACTURING / "laser"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import skeleton as sk  # noqa: E402
from skeleton import Box, Mount  # noqa: E402
import design as desk  # noqa: E402  (desk_robot/design.py, read-only: prisms, laser kerf, CNC writer)
import bought_parts as desk_bought  # noqa: E402  (screw_length(), ISO 4762 / 4032 tables)
import job as laser_job  # noqa: E402
import toolpath  # noqa: E402

G = 9.81
KGCM = 0.0980665                         # N.m per kg.cm
POLOLU = "https://www.pololu.com"

# ---- bought parts ---------------------------------------------------------------------------
BOUGHT: Dict[str, dict] = {
    "gearmotor": {
        "spec": "Pololu #4844, 34:1 Metal Gearmotor 25Dx67L mm HP 12V with 48 CPR Encoder", "qty": 4, "g": 101.0,
        "ratio": 34.014, "counts_per_rev": 1632.67, "rated_v": 12.0,
        "no_load_rpm": 300.0, "no_load_a": 0.30, "stall_a": 5.0, "stall_kgcm": 11.0,
        "at_6v": {"no_load_rpm": 150.0, "stall_a": 2.5, "stall_kgcm": 5.5},
        "max_eff": {"rpm": 260.0, "kgcm": 1.6, "a": 0.82, "pct": 44.0},
        "current_rule": 0.25,            # Pololu: "25% or less of the stall current" for continuous use
        "continuous_kgcm": 4.0,          # Pololu #4844: recommended continuous 4 kg.cm, intermittent 8
        "intermittent_kgcm": 8.0,
        # Pololu drawing 0J1634, page 2 (25D with 48 CPR encoder), 34:1 row:
        "gearbox_d": 25.0, "gearbox_l": 21.0, "can_d": 24.4, "can_l": 30.8, "encoder_d": 25.0, "encoder_l": 14.7,
        "drawn_d": 24.9,                 # the product page: "diameter just under 25 mm"; drawn so, the foot
                                         # does not bear on it (the flange locates the motor)
        "shaft_d": 4.0, "shaft_flat": 3.5, "shaft_l": 12.5, "bushing_d": 7.0, "bushing_l": 2.5,
        "face_hole_r": 8.5, "face_hole_depth": 6.0, "face_tap_drill": 2.5,
        "source": f"{POLOLU}/product/4844/specs; drawing {POLOLU}/file/0J1634/25d-metal-gearmotor-dimension-diagram.pdf",
        "note": "speed, stall torque and stall current scale with voltage (Pololu's 6 V and 12 V rows agree); the "
                "no-load current is taken at its 12 V value (0.30 A) at every voltage, which over-states it at 7.2 V",
    },
    "bracket": {
        "spec": "Pololu #2676 25D mm Metal Gearmotor Bracket (stamped aluminium L)", "qty": 4, "g": 8.5, "t": 2.0,
        "flange_w": 25.0, "flange_h": 27.0, "axis_from_foot": 14.5, "centre_hole_d": 7.5, "face_holes_d": 3.3,
        "face_holes_pcd": 17.0, "foot_l": 52.0, "foot_w": 21.7, "foot_holes_d": 3.3, "foot_hole0": 6.4,
        "pitch": 6.35, "notch_r": 1.7, "notch0": 9.5, "notch_inset": 1.7,
        "source": f"{POLOLU}/product/2676; drawing {POLOLU}/file/0J825/2676-bracket-dimensions.pdf",
        "note": "notch_inset (the U-notch's centre 1.7 mm in from the foot's edge) is read off the drawing's picture, "
                "not dimensioned; Pololu sells them as a pair (#2676 = 2 brackets): buy two pairs",
    },
    "hex_adapter": {
        "spec": "Pololu #2684 12 mm Hex Wheel Adapter for 4 mm Shaft (2 set screws M3, 1 M4 x 8 screw each)", "qty": 4,
        "hex_af": 12.0, "length": 17.0, "bore_d": 4.0, "bore_depth": 10.0, "m4_depth": 7.0,
        "set_screws_from_motor_end": (2.75, 7.25),
        "source": f"{POLOLU}/product/2684; drawing {POLOLU}/file/0J850/2684-dimensions.pdf",
        "note": "mass not listed: computed from the drawing's volume, 6061 at 2.70 g/cm3; a 2-pack: buy two",
    },
    "wheel": {
        "spec": "DFRobot FIT0199 D65mm Rubber Wheel (without shaft): 65 mm OD, 26 mm tyre, 12 mm hex hub, 4 mm bore",
        "qty": 4, "g": 30.0, "od": 65.0, "width": 26.0, "rim_d": 45.0, "hub_hex": 12.0, "bore_d": 4.0,
        "source": "https://www.dfrobot.com/product-652.html; drawing "
                  "https://dfimg.dfrobot.com/enshop/image/data/FIT0199/FIT0199-B/FIT0199Dimensions.png",
        "note": "DFRobot gives OD, width, rim 45 mm, hex 12 mm and bore 4 mm only. ASSUMED, measure on the wheel: "
                "the hex socket's web 16.5 mm in from the tyre's inner edge (hub_seat), 2 mm thick, and the mass "
                "(30 g estimated for a rubber tyre on a plastic hub this size). Sold as a pair: buy two",
    },
    "spine_foot_stock": {"spec": "6061 aluminium plate, 4 mm, cut to 40 x 120 mm", "qty": 1, "source": "any"},
    "strap": {"spec": "hook-and-loop strap, 20 mm wide, 250 mm", "qty": 1, "g": 4.0, "source": "any"},
}
# Fasteners. ISO 7380-1 button heads where a head must be low (fasteners.eu tables), ISO 4762
# cap screws elsewhere (desk_robot/bought_parts.SHCS), ISO 4032 nuts, ISO 7089 washers.
BUTTON = {"M3": {"head_d": 5.7, "head_k": 1.65, "pitch": 0.5, "d": 3.0}}
SHCS = dict(desk_bought.SHCS, M4={"head_d": 7.0, "head_k": 4.0, "pitch": 0.7, "d": 4.0},
            **{"M2.5": {"head_d": 4.5, "head_k": 2.5, "pitch": 0.45, "d": 2.5}})
NUT = desk_bought.NUT
WASHER = {"M3": {"d1": 3.2, "d2": 7.0, "h": 0.5}}
FASTENER_SOURCES = {"ISO 7380": "https://www.fasteners.eu/standards/ISO/7380/",
                    "ISO 4762": "https://www.fasteners.eu/standards/ISO/4762/",
                    "ISO 4032": "https://www.fasteners.eu/standards/ISO/4032/",
                    "ISO 7089": "https://www.fasteners.eu/standards/ISO/7089/"}
CLEARANCE = {"M2.5": 2.9, "M3": 3.4, "M4": 4.5}     # ISO 273 normal fit
NUT_TRAP_AF = 5.6                                    # an M3 nut (5.5 AF) in a laser-cut hex trap
PLYWOOD_G_CM3 = desk.PLYWOOD_G_CM3
ALUMINIUM_G_CM3 = desk.ALUMINIUM_G_CM3
STEEL_G_CM3 = 7.85
LASER_KERF_MM = desk.LASER_KERF_MM                   # measured on 3 mm (laser/README.md); assumed for 6 mm
# Sienci's material guide, as the twin reproduces it (laser/README.md "Calibration"): 3 mm
# plywood 100 %, 2.2 mm/s, 3 passes, Z down 1 mm a pass (the fitted row); 6 mm plywood 100 %,
# 2.2 mm/s, Z down 1.2 mm a pass. Sienci's row says 5 passes, but on the twin 5 left every
# 6 mm part joined to the sheet by a 0.16-0.20 mm web (2026-11-03, release/make/MAKE.md); 6
# passes cut the spine rear free with 9/9 holes through, +3.7 min a plate. The ~14 passes in
# the README are the 1/4 in row at 5 mm/s, not this one.
LASER = {3.0: {"passes": 3, "z_step": 1.0, "mm_s": 2.2}, 6.0: {"passes": 6, "z_step": 1.2, "mm_s": 2.2}}

# The battery's volume, reserved for the electrical agent: its 2S2P pack (electrical.py's part
# and mass) against the deck's underside, centred, between the front and rear motor pairs, with
# BATTERY_MARGIN round it on every side but the deck's. Its leads leave from its -y end.
BATTERY_PACK = {"spec": "2S2P pack, 4 x Molicel P28A 18650 flat side by side, protection board, XT30 "
                        "(electrical.py; outline to be confirmed by it)", "size_mm": (75.0, 72.0, 20.0), "g": 200.0}
BATTERY_MARGIN = 1.5


def _battery_holder() -> dict:
    (px, py, pz), m = BATTERY_PACK["size_mm"], BATTERY_MARGIN
    top = Chassis.deck_z
    b = Box(-px / 2 - m, px / 2 + m, -py / 2 - m, py / 2 + m, top - pz - m, top)
    strap_x = b.x1 + STRAP_SLOT_W / 2                         # the strap passes just outside the volume
    return {"size_mm": (b.x1 - b.x0, b.y1 - b.y0, b.z1 - b.z0), "box": b,
            "pack_box": Box(-px / 2, px / 2, -py / 2, py / 2, top - pz, top),
            "centre_mm": (0.0, 0.0, top - pz / 2), "strap_x": strap_x,
            "hold": f"hook-and-loop strap, 20 mm, through the deck's two slots at x = +-{strap_x:g}, under the pack",
            "lead_hole": BATTERY_LEAD_HOLE,
            "leads": f"from the pack's -y end up through the deck's {BATTERY_LEAD_HOLE[2]:g} mm hole at "
                     f"{BATTERY_LEAD_HOLE[:2]} into the electrical bay (mirror it in y if the pack's leads leave +y)",
            "pack": BATTERY_PACK}


STRAP_SLOT_W = 2.5             # a 20 mm hook-and-loop strap, ~1.5 mm thick, on edge through a 2.5 x 25 slot
R_SPINE_X = 15.0
BATTERY_LEAD_HOLE = (-28.0, -42.5, 10.0)   # beyond the pack's -y end (y -36), clear of the spine foot (x >= -15)


@dataclass(frozen=True)
class Chassis:
    face_y: float = 67.0           # |y| of each gearbox's output face (= the base zone's edge): 1 mm between encoders
    deck_z: float = 50.0           # the deck's underside = the lower SHELL_MOUNTS' plane
    layer_t: float = 3.0           # each of the deck's two plywood layers
    deck_x: float = 170.0          # = the base zone's length
    deck_y: float = 140.0          # = the base zone's width
    deck_r: float = 10.0
    spacer_t: float = 3.0          # plywood, between each bracket's foot and the deck
    foot_t: float = 4.0            # the spine foot, 6061: deck top to the spine zone's floor
    spine_t: float = 6.0           # plywood
    spine_x: float = 15.0          # the spine's half length (x), = its zone
    spine_y: float = 55.0          # the spine's side faces (|y|), = the skeleton's
    spine_top: float = 262.0
    rear_top: float = 130.0        # the rear (bay) plate, z 60..130
    front_z: Tuple[float, float] = (180.0, 256.0)   # the front (head) plate
    rung_t: float = 6.0
    hub_seat: float = 16.5         # ASSUMED (BOUGHT["wheel"]): the wheel's hex web, in from the tyre's inner edge
    hub_web: float = 2.0           # ASSUMED: the web's thickness under the M4 screw's head
    hub_socket: float = 8.0        # ASSUMED: the hex socket's depth, in front of the web
    tee_x: Tuple[float, ...] = (-8.0, 8.0)   # the spine's T-slot screws, under each side plate
    tee_nut_z: float = 3.0         # the T-slot nut's pocket, above the side plate's bottom edge
    mu_floor: Tuple[float, float] = (0.8, 1.0)   # tyre on a classroom floor (vinyl, wood, lino)
    crr: float = 0.02              # rolling resistance, rubber on a hard floor
    mass_kg: float = sk.MASS_TARGET_G[1] / 1000.0   # the skeleton's mass ceiling, for the drive's sizing
    battery_v: Tuple[float, float, float] = (6.0, 7.2, 8.4)   # 2S Li-ion: empty, nominal (2 x 3.6 V, Molicel P28A), full

    @property
    def axle_z(self) -> float:
        return sk.AXLE_Z

    @property
    def deck_top(self) -> float:
        return self.deck_z + 2 * self.layer_t

    @property
    def spine_z0(self) -> float:
        return self.deck_top + self.foot_t

    @property
    def flange_out(self) -> float:
        """|y| of each bracket flange's outer face (the wheel side)."""
        return self.face_y + BOUGHT["bracket"]["t"]

    @property
    def motor_l(self) -> float:
        m = BOUGHT["gearmotor"]
        return m["gearbox_l"] + m["can_l"] + m["encoder_l"]

    @property
    def wheel_in(self) -> float:
        """|y| of each tyre's inner edge."""
        return sk.TRACK / 2 - sk.WHEEL_W / 2

    @property
    def adapter_y(self) -> float:
        """|y| of the hex adapter's motor end: 0.35 mm past the face screws' button heads (on
        their washers), which sit at the same radius as the adapter's corners."""
        return self.flange_out + WASHER["M3"]["h"] + BUTTON["M3"]["head_k"] + 0.35

    @property
    def foot_outer_z(self) -> float:
        """z of each (upside-down) bracket foot's outer face: the axle plus the bracket's 14.5."""
        return self.axle_z + BOUGHT["bracket"]["axis_from_foot"]


BATTERY_HOLDER = _battery_holder()

def _r(r) -> Chassis:
    return r if isinstance(r, Chassis) else Chassis()


# ---- the drive --------------------------------------------------------------------------------

def motor_at(v: float) -> dict:
    """The motor's line at v volts (speed, stall torque and stall current proportional to v;
    Pololu's 6 V and 12 V rows agree with that)."""
    m = BOUGHT["gearmotor"]
    k = v / m["rated_v"]
    return {"v": v, "no_load_rpm": m["no_load_rpm"] * k, "stall_kgcm": m["stall_kgcm"] * k,
            "stall_a": m["stall_a"] * k, "no_load_a": m["no_load_a"]}


def operating(v: float, load_kgcm: float) -> dict:
    """Speed and current at one wheel load on the motor's linear torque line."""
    m = motor_at(v)
    f = load_kgcm / m["stall_kgcm"]
    rpm = m["no_load_rpm"] * (1.0 - f)
    amps = m["no_load_a"] + (m["stall_a"] - m["no_load_a"]) * f
    return {"v": v, "load_kgcm": round(load_kgcm, 4), "torque_frac": round(f, 4), "rpm": round(rpm, 1),
            "m_s": round(rpm / 60.0 * math.pi * sk.WHEEL_D / 1000.0, 3), "a": round(amps, 3),
            "a_frac_of_stall": round(amps / m["stall_a"], 4)}


def drive(r=None) -> dict:
    """The sizing. Per wheel, with W = m g on four wheels, wheel radius rw, track T, wheelbase B:

    - SPIN on the spot: each tyre slides sideways at its contact (the patch moves on a circle
      round the centre, the wheel rolls only along x), so the floor pushes back on each with
      mu W/4 across the robot, at a lever B/2 from the centre: a resisting moment mu W B/2.
      The four drive forces F act at T/2: 4 F T/2. So F = mu W B / (4 T), the torque F rw.
      (Valid while F < mu W/4, i.e. B < T: 110 < 170. A wheel that cannot reach F spins.)
    - RAMP of 10 degrees: F = W (sin a + Crr cos a) / 4.
    - CRUISE on the flat: F = Crr W / 4.
    - The floor can load the gearbox at most mu W/4 rw (then the tyre slips)."""
    r = _r(r)
    W = r.mass_kg * G
    rw = sk.WHEEL_D / 2 / 1000.0
    B, T = sk.WHEELBASE, sk.TRACK
    mu_lo, mu_hi = r.mu_floor
    a = math.radians(10.0)
    loads = {
        "spin_mu_hi": mu_hi * W * B / (4 * T) * rw / KGCM,
        "spin_mu_lo": mu_lo * W * B / (4 * T) * rw / KGCM,
        "ramp_10deg": W * (math.sin(a) + r.crr * math.cos(a)) / 4 * rw / KGCM,
        "cruise": r.crr * W / 4 * rw / KGCM,
        "slip_mu_hi": mu_hi * W / 4 * rw / KGCM,
    }
    v_lo, v_nom, v_hi = r.battery_v
    ops = {name: {f"{v:g}V": operating(v, loads[name]) for v in r.battery_v} for name in
           ("spin_mu_hi", "spin_mu_lo", "ramp_10deg", "cruise")}
    m = BOUGHT["gearmotor"]
    stall_lo = motor_at(v_lo)["stall_kgcm"]
    mm_per_count = math.pi * sk.WHEEL_D / m["counts_per_rev"]
    return {
        "weight_n": round(W, 2), "wheel_radius_m": rw,
        "loads_kgcm_per_wheel": {k: round(v, 4) for k, v in loads.items()},
        "loads_nm_per_wheel": {k: round(v * KGCM, 4) for k, v in loads.items()},
        "operating": ops,
        "margins": {
            "spin_stall_over_load_at_6V": round(stall_lo / loads["spin_mu_hi"], 2),
            "ramp_stall_over_load_at_6V": round(stall_lo / loads["ramp_10deg"], 2),
            "ramp_traction_tan_over_mu": round(math.tan(a) / mu_lo, 3),
            "slip_torque_over_continuous": round(loads["slip_mu_hi"] / m["continuous_kgcm"], 3),
            "no_load_m_s": {f"{v:g}V": round(motor_at(v)["no_load_rpm"] / 60 * math.pi * sk.WHEEL_D / 1000, 3)
                            for v in r.battery_v},
            "pwm_for_0.4_m_s_at_7.2V": round(0.4 / ops["cruise"][f"{v_nom:g}V"]["m_s"], 3),
        },
        "currents_per_motor_a": {
            "cruise": ops["cruise"][f"{v_nom:g}V"]["a"],
            "ramp_10deg": ops["ramp_10deg"][f"{v_nom:g}V"]["a"],
            "spin_turn": ops["spin_mu_hi"][f"{v_nom:g}V"]["a"],
            "stall": {f"{v:g}V": round(motor_at(v)["stall_a"], 3) for v in r.battery_v},
            "note": "per motor, four motors; spin-turn at mu 1.0; stall is what a driver channel must survive",
        },
        "encoder_mm_per_count": round(mm_per_count, 4),
    }


# ---- 2D shapes ----------------------------------------------------------------------------------

def _c(x: float, y: float, d: float) -> Polygon:
    return Point(x, y).buffer(d / 2.0, quad_segs=16)


def _hex(af: float, cx: float = 0.0, cy: float = 0.0) -> Polygon:
    return affinity.translate(desk_bought._hexagon(af), cx, cy)


def _rrect(x0, y0, x1, y1, rad) -> Polygon:
    return box(x0 + rad, y0 + rad, x1 - rad, y1 - rad).buffer(rad, quad_segs=8)


def _slot(x0, y0, x1, y1) -> Polygon:
    """A rounded slot filling the rectangle."""
    rad = min(x1 - x0, y1 - y0) / 2.0
    return _rrect(x0, y0, x1, y1, rad)


def _d_shaft(cx: float, cz: float) -> Polygon:
    m = BOUGHT["gearmotor"]
    flat = m["shaft_flat"] - m["shaft_d"] / 2
    return _c(cx, cz, m["shaft_d"]).intersection(box(cx - 5, cz - 5, cx + 5, cz + flat))


def motor_xs() -> Tuple[float, float]:
    return (sk.WHEELBASE / 2, -sk.WHEELBASE / 2)


def foot_screw_points(r: Chassis, xm: float, sy: int) -> List[Tuple[float, float]]:
    """The bracket screws: in the foot's edge notches over the motor CAN (k = 3 and 5: the
    gearbox, wider than the can, leaves no room for a head under the foot)."""
    b = BOUGHT["bracket"]
    dx = b["foot_w"] / 2 - b["notch_inset"]
    return [(xm + s * dx, sy * (r.flange_out - (b["notch0"] + k * b["pitch"]))) for k in (3, 5) for s in (-1, 1)]


def bracket_foot(r: Chassis, xm: float, sy: int) -> Polygon:
    b = BOUGHT["bracket"]
    y_out = sy * r.flange_out
    y_in = sy * (r.flange_out - b["foot_l"])
    hw = b["foot_w"] / 2
    p = box(xm - hw, min(y_in, y_out), xm + hw, max(y_in, y_out))
    cut = [_c(xm, sy * (r.flange_out - (b["foot_hole0"] + k * b["pitch"])), b["foot_holes_d"]) for k in range(7)]
    for k in range(6):
        y = sy * (r.flange_out - (b["notch0"] + k * b["pitch"]))
        for s in (-1, 1):
            cx = xm + s * (hw - b["notch_inset"])
            cut.append(_c(cx, y, 2 * b["notch_r"]))
            cut.append(box(min(cx, xm + s * (hw + 0.5)), y - b["notch_r"], max(cx, xm + s * (hw + 0.5)), y + b["notch_r"]))
    return p.difference(unary_union(cut))


def bracket_flange(r: Chassis, xm: float) -> Polygon:
    """In (x, z), upside down: the foot on top, the round end round the axle."""
    b = BOUGHT["bracket"]
    z = r.axle_z
    rad = b["flange_w"] / 2
    top = r.foot_outer_z - b["t"]
    p = box(xm - rad, z, xm + rad, top).union(_c(xm, z, b["flange_w"]))
    holes = [_c(xm, z, b["centre_hole_d"])]
    q = b["face_holes_pcd"] / 2
    holes += [_c(xm + dx, z + dz, b["face_holes_d"]) for dx, dz in ((q, 0), (-q, 0), (0, q), (0, -q))]
    return p.difference(unary_union(holes))


def spacer(r: Chassis, xm: float, sy: int) -> Polygon:
    b = BOUGHT["bracket"]
    y_out, y_in = sy * r.flange_out, sy * (r.flange_out - b["foot_l"])
    p = box(xm - b["foot_w"] / 2, min(y_in, y_out), xm + b["foot_w"] / 2, max(y_in, y_out))
    cut = []
    for x, y in foot_screw_points(r, xm, sy):           # U-notches, as the foot's (a hole would cut its edge)
        s = 1.0 if x > xm else -1.0
        cut.append(_c(x, y, CLEARANCE["M3"]))
        cut.append(box(min(x, xm + s * (b["foot_w"] / 2 + 0.5)), y - CLEARANCE["M3"] / 2,
                       max(x, xm + s * (b["foot_w"] / 2 + 0.5)), y + CLEARANCE["M3"] / 2))
    return p.difference(unary_union(cut))


def deck_layer(r: Chassis, upper: bool) -> Polygon:
    hx, hy = r.deck_x / 2, r.deck_y / 2
    p = _rrect(-hx, -hy, hx, hy, r.deck_r)
    cut = [_c(m.at[0], m.at[1], CLEARANCE["M3"]) for m in sk.SHELL_MOUNTS if m.name.startswith("lower")]
    for xm in motor_xs():
        for sy in (1, -1):
            for x, y in foot_screw_points(r, xm, sy):
                cut.append(_hex(NUT_TRAP_AF, x, y) if upper else _c(x, y, CLEARANCE["M3"]))
        cut.append(_slot(xm - 5.0, -13.0, xm + 5.0, 13.0))                       # the motor leads, up
    for x in r.tee_x:
        for sy in (1, -1):
            cut.append(_c(x, sy * (r.spine_y - r.spine_t / 2), CLEARANCE["M3"]))  # the spine's screws
    for s in (-1, 1):
        sx = BATTERY_HOLDER["strap_x"]
        cut.append(_slot(s * sx - STRAP_SLOT_W / 2, -12.5, s * sx + STRAP_SLOT_W / 2, 12.5))   # the battery strap
    cut.append(_c(*BATTERY_HOLDER["lead_hole"]))                                  # the battery leads
    return p.difference(unary_union(cut))


def spine_foot(r: Chassis) -> Polygon:
    p = _rrect(-r.spine_x, -r.spine_y, r.spine_x, r.spine_y, 3.5)
    return p.difference(unary_union([_c(x, sy * (r.spine_y - r.spine_t / 2), CLEARANCE["M3"])
                                     for x in r.tee_x for sy in (1, -1)]))


# The spine's plates. Side plates in (x, z); rear and front plates in (y, z); rungs in (x, y).
REAR_TABS = ((70.0, 90.0), (100.0, 120.0))
FRONT_TABS = ((190.0, 210.0), (226.0, 246.0))
MID_TAB_X = 8.0
FIT = 0.1                      # per side, every tab in its slot (a glued fit)
TOP_TAB_X = 10.0
PI5 = {"holes": (58.0, 49.0), "board": (85.0, 56.0), "centre_yz": (0.0, 98.0), "d": CLEARANCE["M2.5"],
       "source": "https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf"}
REAR_ROW = {"y": (-40.0, -20.0, 0.0, 20.0, 40.0), "z": (64.0,), "d": CLEARANCE["M3"]}
FRONT_GRID = {"y": (-40.0, -20.0, 0.0, 20.0, 40.0), "z": (190.0, 210.0, 230.0, 250.0), "d": CLEARANCE["M3"]}


def side_plate(r: Chassis) -> Polygon:
    x, t, z0 = r.spine_x, r.spine_t, r.spine_z0
    p = box(-x, z0, x, r.spine_top)
    cut = []
    for z_a, z_b in REAR_TABS:
        cut.append(box(-x - 1, z_a - FIT, -x + t + FIT, z_b + FIT))
    for z_a, z_b in FRONT_TABS:
        cut.append(box(x - t - FIT, z_a - FIT, x + 1, z_b + FIT))
    cut.append(box(-MID_TAB_X - FIT, r.rear_top - FIT, MID_TAB_X + FIT, r.rear_top + r.rung_t + FIT))
    cut.append(box(-TOP_TAB_X - FIT, r.spine_top - r.rung_t - FIT, TOP_TAB_X + FIT, r.spine_top + 1))
    for xt in r.tee_x:                                          # the T-slots: shank slot and nut pocket
        cut.append(box(xt - CLEARANCE["M3"] / 2, z0 - 1, xt + CLEARANCE["M3"] / 2, z0 + r.tee_nut_z + 3.5))
        cut.append(box(xt - 3.2, z0 + r.tee_nut_z, xt + 3.2, z0 + r.tee_nut_z + NUT["M3"]["m"] + 0.1))
    for m in side_mounts():
        if m.at[1] > 0:
            cut.append(_c(m.at[0], m.at[2], CLEARANCE["M3"]))   # a hole outside the plate cuts nothing
    return p.difference(unary_union(cut))


SPINE_FACE_Y = 55.0
HEX_KEY = {"spec": "ISO 2936 2.5 mm hex key (for M3 ISO 4762)", "long_mm": 56.0, "short_mm": 18.0}


def side_mounts() -> List[Mount]:
    """Every skeleton interface whose screw passes through a spine side plate (|y| 55)."""
    return [m for m in sk.SHELL_MOUNTS + sk.SHOULDER_BOLTS
            if m.part == "chassis" and abs(abs(m.at[1]) - SPINE_FACE_Y) < 1e-6]


def rear_plate(r: Chassis) -> Polygon:
    w = r.spine_y - r.spine_t
    p = box(-w, r.spine_z0, w, r.rear_top)
    for z_a, z_b in REAR_TABS:
        p = p.union(box(-r.spine_y, z_a, -w, z_b)).union(box(w, z_a, r.spine_y, z_b))
    sx, sz = PI5["holes"]
    cy, cz = PI5["centre_yz"]
    cut = [_c(cy + i * sx / 2, cz + j * sz / 2, PI5["d"]) for i in (-1, 1) for j in (-1, 1)]
    cut += [_c(y, z, REAR_ROW["d"]) for y in REAR_ROW["y"] for z in REAR_ROW["z"]]
    return p.difference(unary_union(cut))


def front_plate(r: Chassis) -> Polygon:
    w = r.spine_y - r.spine_t
    z0, z1 = r.front_z
    p = box(-w, z0, w, z1)
    for z_a, z_b in FRONT_TABS:
        p = p.union(box(-r.spine_y, z_a, -w, z_b)).union(box(w, z_a, r.spine_y, z_b))
    return p.difference(unary_union([_c(y, z, FRONT_GRID["d"]) for y in FRONT_GRID["y"] for z in FRONT_GRID["z"]]))


def rung(r: Chassis, tab_x: float, hole: Polygon) -> Polygon:
    w = r.spine_y - r.spine_t
    p = box(-r.spine_x, -w, r.spine_x, w)
    p = p.union(box(-tab_x, w, tab_x, r.spine_y)).union(box(-tab_x, -r.spine_y, tab_x, -w))
    return p.difference(hole)


# The mid rung's harness slot (x, y): the front motors' leads and their connectors pass it on the
# way up to the bay. 10 x 24 passes a 1 x 6 2.54 mm header housing (15.2 x 2.5) and a JST XH
# 6-pin housing (17.4 x 5.75, JST's XH drawing: 2.5 x 5 + 4.9) one at a time.
MID_SLOT = (-3.0, -12.0, 7.0, 12.0)


def mid_rung(r: Chassis) -> Polygon:
    return rung(r, MID_TAB_X, _slot(*MID_SLOT))


def top_rung(r: Chassis) -> Polygon:
    return rung(r, TOP_TAB_X, _c(0.0, 0.0, 10.0))            # the accessory slot's harness


# ---- 3D: prisms in any plane, mirrored --------------------------------------------------------------

def _flip(t: np.ndarray) -> np.ndarray:
    return t[:, [0, 2, 1], :]


def p_xy(poly, z0, z1) -> np.ndarray:
    return desk_bought._prism(poly, z0, z1)


def p_xz(poly, y0, y1) -> np.ndarray:
    """A polygon in (x, z), extruded along y from y0 to y1."""
    t = desk_bought._prism(poly, min(y0, y1), max(y0, y1))[:, :, [0, 2, 1]]
    return _flip(t)


def p_yz(poly, x0, x1) -> np.ndarray:
    t = desk_bought._prism(poly, min(x0, x1), max(x0, x1))
    return t[:, :, [2, 0, 1]]


def mirror_y(t: np.ndarray) -> np.ndarray:
    out = t.copy()
    out[:, :, 1] *= -1.0
    return _flip(out)


def _cat(*ts) -> np.ndarray:
    return np.concatenate([t for t in ts if len(t)])


WHEEL_LABEL = {"front_left": "fl", "front_right": "fr", "rear_left": "rl", "rear_right": "rr"}


def _left(sy: int, t: np.ndarray) -> np.ndarray:
    """Solids are built for the left (+y) side; the right side is their mirror."""
    return t if sy > 0 else mirror_y(t)


def motor_solids(r: Chassis, xm: float) -> Tuple[np.ndarray, np.ndarray]:
    """(body, shaft) of a left motor at x = xm. The body runs from the encoder's end to the
    hex adapter's motor end; the shaft inside the adapter is a solid of its own. Face holes:
    the tap drill (drawn through the gearbox; the real ones are 6.0 mm deep)."""
    m = BOUGHT["gearmotor"]
    z, F = r.axle_z, r.face_y
    gb = _c(xm, z, m["drawn_d"])
    holes = unary_union([_c(xm, z + s * m["face_hole_r"], m["face_tap_drill"]) for s in (-1, 1)])
    y_gb = F - m["gearbox_l"]
    y_can = y_gb - m["can_l"]
    y_enc = y_can - m["encoder_l"]
    body = _cat(
        p_xz(gb.difference(holes), y_gb, F),          # the tapped holes drawn through the gearbox (6.0 deep)
        p_xz(_c(xm, z, m["can_d"]), y_can, y_gb),
        p_xz(_c(xm, z, m["drawn_d"]), y_enc, y_can),
        p_xy(box(xm - 4.0, y_enc + 10.0, xm + 4.0, y_enc + 14.0), z + m["encoder_d"] / 2, z + m["encoder_d"] / 2 + 4.0),
        p_xz(_c(xm, z, m["bushing_d"]), F, F + m["bushing_l"]),
        p_xz(_d_shaft(xm, z), F + m["bushing_l"], r.adapter_y),
    )
    shaft = p_xz(_d_shaft(xm, z), r.adapter_y, F + m["shaft_l"])
    return body, shaft


def bracket_solid(r: Chassis, xm: float) -> np.ndarray:
    b = BOUGHT["bracket"]
    top = r.foot_outer_z
    return _cat(p_xz(bracket_flange(r, xm), r.face_y, r.flange_out),
                p_xy(bracket_foot(r, xm, 1), top - b["t"], top))


def adapter_solid(r: Chassis, xm: float) -> np.ndarray:
    a = BOUGHT["hex_adapter"]
    z, y0 = r.axle_z, r.adapter_y
    hexa = _hex(a["hex_af"], xm, z)
    # One prism: the 4 mm bore drawn through (the far 7 mm is the M4 thread, drawn at its 4 mm).
    return p_xz(hexa.difference(_c(xm, z, a["bore_d"] + 0.05)), y0, y0 + a["length"])


def wheel_solid(r: Chassis, xm: float) -> np.ndarray:
    w = BOUGHT["wheel"]
    z, y0 = r.axle_z, r.wheel_in
    web0 = y0 + r.hub_seat
    tyre = _c(xm, z, w["od"]).difference(_c(xm, z, w["rim_d"]))
    rim = _c(xm, z, w["rim_d"]).difference(_c(xm, z, w["rim_d"] - 4.0))
    web = _c(xm, z, w["rim_d"] - 4.0).difference(_c(xm, z, w["bore_d"] + 0.2))
    boss = _c(xm, z, 18.0).difference(_hex(w["hub_hex"] + 0.1, xm, z))
    return _cat(p_xz(tyre, y0, y0 + w["width"]), p_xz(rim, y0 + 1.0, y0 + w["width"] - 1.0),
                p_xz(web, web0, web0 + r.hub_web), p_xz(boss, web0 - r.hub_socket, web0))


def screw_solid(size: str, length: float, head_at, axis, button: bool = False) -> np.ndarray:
    """A screw: head bearing face at `head_at`, shank along unit `axis` (one of +-x, +-y, +-z)."""
    s = (BUTTON if button else SHCS)[size]
    base = _cat(p_xy(_c(0, 0, s["head_d"]), -s["head_k"], 0.0), p_xy(_c(0, 0, s["d"]), 0.0, length))
    return _orient(base, axis) + np.asarray(head_at, float)


def nut_solid(size: str, at, axis) -> np.ndarray:
    n = NUT[size]
    d = SHCS[size]["d"]
    base = p_xy(desk_bought._hexagon(n["s"]).difference(Point(0, 0).buffer(d / 2 * 0.98, quad_segs=8)), 0.0, n["m"])
    return _orient(base, axis) + np.asarray(at, float)


def washer_solid(size: str, at, axis) -> np.ndarray:
    w = WASHER[size]
    base = p_xy(_c(0, 0, w["d2"]).difference(_c(0, 0, w["d1"])), 0.0, w["h"])
    return _orient(base, axis) + np.asarray(at, float)


def _orient(t: np.ndarray, axis) -> np.ndarray:
    """Turn a part drawn along +z so +z points along `axis` (proper rotations only)."""
    ax = tuple(float(a) for a in axis)
    R = {(0, 0, 1): np.eye(3), (0, 0, -1): np.diag([1.0, -1.0, -1.0]),
         (0, 1, 0): np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),
         (0, -1, 0): np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),
         (1, 0, 0): np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),
         (-1, 0, 0): np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)}[tuple(int(round(a)) for a in ax)]
    # R maps local +z to `axis`: its third column.
    return t @ R.T


# ---- joints: every screw, from screw_length() -------------------------------------------------------

def _face_screw(r: Chassis) -> dict:
    """Into the gearbox face's 6.0 mm tapped holes through the 2 mm flange and an ISO 7089
    washer: 1.5 d (4.5 mm) is the rule and the hole's depth less 0.5 the limit."""
    m = BOUGHT["gearmotor"]
    grip = BOUGHT["bracket"]["t"] + WASHER["M3"]["h"]
    s = desk_bought.screw_length("M3", grip, nut=False, thread_in_mm=1.5 * 3.0)
    return dict(s, hole_depth_mm=m["face_hole_depth"], ok=1.5 * 3.0 - 1e-9 <= s["thread_in_mm"] <= m["face_hole_depth"] - 0.5)


def joints(r=None) -> List[dict]:
    """Every screwed joint of the chassis: label, the screw (screw_length's answer), the head's
    bearing point, the shank's direction, the nut or thread it ends in."""
    r = _r(r)
    out = []
    b = BOUGHT["bracket"]
    m = BOUGHT["gearmotor"]
    for wname, (wx, wy, _) in sk.WHEELS.items():
        sy = 1 if wy > 0 else -1
        tag = WHEEL_LABEL[wname]
        fs = _face_screw(r)
        for k, s in enumerate((1, -1)):
            at = (wx, sy * (r.flange_out + WASHER["M3"]["h"]), r.axle_z + s * m["face_hole_r"])
            out.append({"joint": "gearbox_face", "label": f"screw_face_{tag}_{k}", "step": 2, "head": "ISO 7380",
                        "at": at, "axis": (0.0, -float(sy), 0.0), "into": "the gearbox face's M3 thread",
                        "washer": f"washer_face_{tag}_{k}", **fs})
        grip = b["t"] + r.spacer_t + r.layer_t
        for k, (x, y) in enumerate(foot_screw_points(r, wx, sy)):
            out.append({"joint": "bracket_foot", "label": f"screw_foot_{tag}_{k}", "step": 3, "head": "ISO 7380",
                        "at": (x, y, r.foot_outer_z - b["t"]), "axis": (0.0, 0.0, 1.0),
                        "into": "an ISO 4032 nut in the upper deck layer's trap", "nut_label": f"nut_foot_{tag}_{k}",
                        **desk_bought.screw_length("M3", grip)})
        a = BOUGHT["hex_adapter"]
        grip = r.hub_web
        s = desk_bought.screw_length("M4", grip, nut=False, thread_in_mm=1.5 * 4.0) if "M4" in desk_bought.SHCS else \
            _m4_length(grip)
        out.append({"joint": "wheel_hub", "label": f"screw_wheel_{tag}", "step": 4, "head": "ISO 4762",
                    "at": (wx, sy * (r.wheel_in + r.hub_seat + r.hub_web), r.axle_z), "axis": (0.0, -float(sy), 0.0),
                    "into": "the hex adapter's M4 thread (7 mm deep)", "supplied": "with Pololu #2684 (M4 x 8)",
                    **s, "ok": s["length"] == 8 and s["thread_in_mm"] <= a["m4_depth"]})
    for x in r.tee_x:
        for sy in (1, -1):
            grip = 2 * r.layer_t + r.foot_t + r.tee_nut_z
            tag = f"{'l' if sy > 0 else 'r'}{'f' if x > 0 else 'b'}"
            out.append({"joint": "spine_tslot", "label": f"screw_spine_{tag}", "step": 6, "head": "ISO 4762",
                        "at": (x, sy * (r.spine_y - r.spine_t / 2), r.deck_z), "axis": (0.0, 0.0, 1.0),
                        "into": "an ISO 4032 nut captured in the side plate's T-slot", "nut_label": f"nut_spine_{tag}",
                        **desk_bought.screw_length("M3", grip)})
    return out


def _m4_length(grip: float) -> dict:
    need = grip + 1.5 * 4.0
    length = next(L for L in (6, 8, 10, 12, 16, 20) if L >= need - 1e-9)
    return {"size": "M4", "length": length, "grip_mm": grip, "needs_mm": need, "nut": None, "proud_mm": None,
            "thread_in_mm": round(length - grip, 3)}


def interface_joints(r=None) -> List[dict]:
    """The screws through the chassis into another subsystem's thread (skeleton.SHELL_MOUNTS and
    SHOULDER_BOLTS, and the Pi 5's standoffs): their lengths are ours to name, their threads the
    holder's to provide."""
    r = _r(r)
    out = []
    for mt in sk.SHELL_MOUNTS + present_side(r, sk.SHOULDER_BOLTS):
        ax = np.asarray(mt.axis, float)
        plate = 2 * r.layer_t if mt.name.startswith("lower") else r.spine_t
        head = np.asarray(mt.at, float) - ax * plate
        grip = plate
        s = desk_bought.screw_length("M3", grip, nut=False, thread_in_mm=1.5 * 3.0)
        out.append({"joint": "interface", "label": f"screw_interface_{mt.name}", "mount": mt.name, "holder": mt.holder,
                    "mass_counted_by": "chassis",
                    "step": 7, "head": "ISO 4762", "at": tuple(float(v) for v in head.round(3)),
                    "axis": tuple(float(v) for v in ax),
                    "into": f"the {mt.holder}'s M3 thread: it must take {s['thread_in_mm']:g} mm (+0.5 clear)", **s})
    out += pi5_joints(r)
    return out


# The standoff: 12 mm, not the 8 mm proposed. With 9 mm of grip, ISO 4762 has M2.5 x 12 (3.0 mm
# of thread, under 1.5 d) and M2.5 x 16 (7.0 mm) and nothing between; x 16 plus the Pi's own
# M2.5 x 6 (4.4 mm through its 1.6 mm board) plus 0.5 between them needs 11.9 mm.
PI5_STANDOFF = {"spec": "M2.5 F-F standoff, 12 mm (proposed to electrical.py)", "len_mm": 12.0}
PI5_BOARD_T = 1.6              # the Pi 5's PCB, under its own screws' heads
PI5_BAY_PLATE_T = 3.0          # electrical.py's bay plate, between the spine's rear plate and the standoffs
ISO4762_LENGTHS = (3, 4, 5, 6, 8, 10, 12, 16, 20, 25)


def screw_len(size: str, grip: float, thread_in: float) -> int:
    """screw_length()'s rule for sizes its table lacks: the shortest ISO 4762 length that gives
    `thread_in` of thread beyond `grip`."""
    return next(L for L in ISO4762_LENGTHS if L >= grip + thread_in - 1e-9)


def pi5_joints(r=None, standoff_mm: Optional[float] = None) -> List[dict]:
    """The Raspberry Pi 5's four M2.5 screws: from inside the spine (head on the rear plate's
    inner face, x -9), through the rear plate (6) and the electrical's 3 mm bay plate, at least
    1.5 d into an F-F standoff; the Pi's own screw (1.5 d through its board) takes the other end,
    and the two must leave 0.5 mm between them inside it."""
    r = _r(r)
    L_so = PI5_STANDOFF["len_mm"] if standoff_mm is None else standoff_mm
    grip = r.spine_t + PI5_BAY_PLATE_T
    length = screw_len("M2.5", grip, 1.5 * 2.5)
    t = length - grip
    pi_len = screw_len("M2.5", PI5_BOARD_T, 1.5 * 2.5)
    pi_t = pi_len - PI5_BOARD_T
    fits = t + pi_t + 0.5 <= L_so + 1e-9
    out = []
    for mt in electrical_mounts(r):
        if not mt.name.startswith("bay_pi5"):
            continue
        ax = np.asarray(mt.axis, float)
        head = np.asarray(mt.at, float) - ax * r.spine_t
        out.append({"joint": "interface", "label": f"screw_interface_{mt.name}", "mount": mt.name, "holder": mt.holder,
                    "mass_counted_by": "chassis", "step": 7, "head": "ISO 4762",
                    "at": tuple(float(v) for v in head.round(3)), "axis": tuple(float(v) for v in ax),
                    "into": f"an M2.5 F-F standoff, {L_so:g} mm: {t:g} mm of its thread; the Pi's M2.5 x {pi_len} "
                            f"takes {pi_t:g} from the other end",
                    "standoff_mm": L_so, "pi_screw": f"M2.5 x {pi_len} ISO 4762", "pi_thread_mm": round(pi_t, 3),
                    "size": "M2.5", "length": length, "grip_mm": grip, "needs_mm": grip + 1.5 * 2.5, "nut": None,
                    "proud_mm": None, "thread_in_mm": round(t, 3),
                    "ok": t >= 1.5 * 2.5 - 1e-9 and fits})
    return out


# ---- mounts --------------------------------------------------------------------------------------

def present_shoulder_bolts(r: Chassis) -> List[Mount]:
    return present_side(r, sk.SHOULDER_BOLTS)


def present_side(r: Chassis, mounts_: Sequence[Mount]) -> List[Mount]:
    """Those of `mounts_` the spine's side plates carry: a clearance hole inside the plate."""
    side = side_plate(r)
    out = []
    for m in mounts_:
        disc = _c(m.at[0], m.at[2], CLEARANCE["M3"] - 0.02)
        outline = Polygon(side.exterior)
        if outline.contains(disc) and side.intersection(disc).area < 1e-6:
            out.append(m)
    return out


def electrical_mounts(r=None) -> List[Mount]:
    """The board mounting points the chassis provides to the electrical subsystem: clearance
    holes in the spine's rear plate (its face at x = -15, the bay's wall) and front plate (its
    face at x = +15, the head zone's wall). A screw from inside the spine into a standoff in the
    electrical zone: the standoff holds the thread."""
    r = _r(r)
    out = []
    sx, sz = PI5["holes"]
    cy, cz = PI5["centre_yz"]
    for i, (dy, dz) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        out.append(Mount(f"bay_pi5_{i}", (-r.spine_x, cy + dy * sx / 2, cz + dz * sz / 2), (-1.0, 0.0, 0.0), "M2.5",
                         "electrical", "chassis"))
    for i, (y, z) in enumerate((y, z) for z in REAR_ROW["z"] for y in REAR_ROW["y"]):
        out.append(Mount(f"bay_row_{i}", (-r.spine_x, y, z), (-1.0, 0.0, 0.0), "M3", "electrical", "chassis"))
    for i, (y, z) in enumerate((y, z) for z in FRONT_GRID["z"] for y in FRONT_GRID["y"]):
        out.append(Mount(f"head_grid_{i}", (r.spine_x, y, z), (1.0, 0.0, 0.0), "M3", "electrical", "chassis"))
    return out


def mounts(r=None) -> List[Mount]:
    """The skeleton interfaces the chassis provides (the clearance holes and faces), and its own
    electrical board points. Shoulder bolts: those the side plates carry (all eight)."""
    r = _r(r)
    return list(sk.SHELL_MOUNTS) + present_shoulder_bolts(r) + electrical_mounts(r)


# ---- solids --------------------------------------------------------------------------------------

@functools.lru_cache(maxsize=4)
def _solids(r: Chassis) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}
    for wname, (wx, wy, _) in sk.WHEELS.items():
        sy = 1 if wy > 0 else -1
        tag = WHEEL_LABEL[wname]
        body, shaft = motor_solids(r, wx)
        out[f"motor_{tag}"] = _left(sy, body)
        out[f"motor_{tag}_shaft"] = _left(sy, shaft)
        out[f"bracket_{tag}"] = _left(sy, bracket_solid(r, wx))
        out[f"spacer_{tag}"] = _left(sy, p_xy(spacer(r, wx, 1), r.foot_outer_z, r.deck_z))
        out[f"hex_adapter_{tag}"] = _left(sy, adapter_solid(r, wx))
        out[f"wheel_{tag}"] = _left(sy, wheel_solid(r, wx))
    out["deck_lower"] = p_xy(deck_layer(r, False), r.deck_z, r.deck_z + r.layer_t)
    out["deck_upper"] = p_xy(deck_layer(r, True), r.deck_z + r.layer_t, r.deck_top)
    out["spine_foot"] = p_xy(spine_foot(r), r.deck_top, r.spine_z0)
    side = side_plate(r)
    out["spine_side_l"] = p_xz(side, r.spine_y - r.spine_t, r.spine_y)
    out["spine_side_r"] = mirror_y(out["spine_side_l"])
    out["spine_rear"] = p_yz(rear_plate(r), -r.spine_x, -r.spine_x + r.spine_t)
    out["spine_front"] = p_yz(front_plate(r), r.spine_x - r.spine_t, r.spine_x)
    out["spine_rung_mid"] = p_xy(mid_rung(r), r.rear_top, r.rear_top + r.rung_t)
    out["spine_rung_top"] = p_xy(top_rung(r), r.spine_top - r.rung_t, r.spine_top)
    for jt in joints(r):
        button = jt["head"] == "ISO 7380"
        out[jt["label"]] = screw_solid(jt["size"], jt["length"], jt["at"], jt["axis"], button)
        ax = np.asarray(jt["axis"], float)
        if jt.get("nut_label"):
            out[jt["nut_label"]] = nut_solid(jt["size"], np.asarray(jt["at"]) + ax * jt["grip_mm"], jt["axis"])
        if jt.get("washer"):
            out[jt["washer"]] = washer_solid("M3", jt["at"], jt["axis"])      # under the head, on the flange
    return out


def solids(r=None) -> Dict[str, np.ndarray]:
    """Every chassis part, assembled, robot frame, mm: closed prisms (each a solid or a few
    closed shells that touch). The interface screws (into the shell's inserts and the arms'
    brackets) are in interface_solids(): they reach into the holder's zone by design."""
    return {k: v.copy() for k, v in _solids(_r(r)).items()}


def interface_solids(r=None) -> Dict[str, np.ndarray]:
    return {jt["label"]: screw_solid(jt["size"], jt["length"], jt["at"], jt["axis"]) for jt in interface_joints(_r(r))}


# ---- masses ----------------------------------------------------------------------------------------

def mass_props(tris: np.ndarray) -> Tuple[float, np.ndarray]:
    """(volume mm3, centroid mm) of a closed, outward-wound triangle set (several shells allowed)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    cen = (v6[:, None] * (a + b + c)).sum(0) / (24.0 * vol)
    return float(vol), cen


MADE = {"deck_lower": ("laser", "3 mm birch plywood", PLYWOOD_G_CM3, 1),
        "deck_upper": ("laser", "3 mm birch plywood", PLYWOOD_G_CM3, 1),
        "spacer": ("laser", "3 mm birch plywood", PLYWOOD_G_CM3, 3),
        "spine_foot": ("cnc", "6061 aluminium, 4 mm", ALUMINIUM_G_CM3, 5),
        "spine_side": ("laser", "6 mm birch plywood", PLYWOOD_G_CM3, 6),
        "spine_rear": ("laser", "6 mm birch plywood", PLYWOOD_G_CM3, 6),
        "spine_front": ("laser", "6 mm birch plywood", PLYWOOD_G_CM3, 6),
        "spine_rung": ("laser", "6 mm birch plywood", PLYWOOD_G_CM3, 6)}


def _made_key(label: str) -> Optional[str]:
    return next((k for k in MADE if label.startswith(k)), None)


def parts(r=None) -> List[dict]:
    """Every part with its mass and centre (robot frame). Made parts: their solids' volume x
    density; bought parts: the maker's mass (or the drawing's volume where none is listed);
    fasteners: their drawn steel volume."""
    r = _r(r)
    sol = _solids(r)
    out = []
    jt_by = {j["label"]: j for j in joints(r)}
    for label, tris in sol.items():
        vol, cen = mass_props(tris)
        key = _made_key(label)
        if key:
            proc, mat, rho, step = MADE[key]
            out.append({"label": label, "step": step, "kind": "made", "process": proc, "material": mat,
                        "mass_g": round(vol / 1000 * rho, 2), "centre_mm": tuple(float(v) for v in cen.round(2)), "source": "chassis.py"})
            continue
        if label.startswith("motor_") and label.endswith("_shaft"):
            out.append({"label": label, "step": 2, "kind": "bought", "process": "bought", "material": "steel",
                        "mass_g": 0.0, "centre_mm": tuple(float(v) for v in cen.round(2)), "source": BOUGHT["gearmotor"]["source"],
                        "note": "the output shaft beyond |y| 70, drawn apart for the zones; its mass is in the motor's"})
            continue
        for prefix, key, step, mat in (("motor_", "gearmotor", 2, "mixed"), ("bracket_", "bracket", 2, "aluminium"),
                                       ("hex_adapter_", "hex_adapter", 4, "aluminium"), ("wheel_", "wheel", 4, "rubber, plastic")):
            if label.startswith(prefix):
                b = BOUGHT[key]
                g = b.get("g", round(vol / 1000 * ALUMINIUM_G_CM3, 2))
                out.append({"label": label, "step": step, "kind": "bought", "process": "bought", "material": mat,
                            "mass_g": g, "centre_mm": tuple(float(v) for v in cen.round(2)), "source": b["source"], "spec": b["spec"]})
                break
        else:
            std = jt_by[label]["head"] if label in jt_by else ("ISO 4032" if label.startswith("nut") else "ISO 7089")
            out.append({"label": label, "step": _fastener_step(label, jt_by), "kind": "bought", "process": "bought",
                        "material": "steel", "mass_g": round(vol / 1000 * STEEL_G_CM3, 3), "centre_mm": tuple(float(v) for v in cen.round(2)),
                        "source": FASTENER_SOURCES.get(std, ""), "spec": _fastener_spec(label, jt_by)})
    isol = interface_solids(r)
    for jt in interface_joints(r):
        if jt["mass_counted_by"] != "chassis":
            continue                     # a screw another subsystem's parts() counts
        vol, cen = mass_props(isol[jt["label"]])
        out.append({"label": jt["label"], "step": 7, "kind": "bought", "process": "bought", "material": "steel",
                    "mass_g": round(vol / 1000 * STEEL_G_CM3, 3), "centre_mm": tuple(float(v) for v in cen.round(2)),
                    "source": FASTENER_SOURCES["ISO 4762"],
                    "spec": f"{jt['size']} x {jt['length']} ISO 4762 (interface screw, see interface_joints)"})
    b = BOUGHT["strap"]
    out.append({"label": "battery_strap", "step": 8, "kind": "bought", "process": "bought", "material": "nylon",
                "mass_g": b["g"], "centre_mm": BATTERY_HOLDER["centre_mm"], "source": b["source"], "spec": b["spec"]})
    return out


def _fastener_step(label: str, jt_by: dict) -> int:
    """A screw goes in at its joint's step; its nut or washer with it (nut_foot_fl_0 is
    screw_foot_fl_0's)."""
    for k in (label, "screw_" + label.split("_", 1)[-1]):
        if k in jt_by:
            return jt_by[k]["step"]
    raise KeyError(f"{label}: no joint gives its step")


def _fastener_spec(label: str, jt_by: dict) -> str:
    if label in jt_by:
        j = jt_by[label]
        return f"{j['size']} x {j['length']} {j['head']}"
    if label.startswith("nut"):
        return "M3 nut ISO 4032"
    return "M3 washer ISO 7089"


def mass_summary(r=None) -> dict:
    ps = parts(r)
    total = sum(p["mass_g"] for p in ps)
    com = sum(np.asarray(p["centre_mm"]) * p["mass_g"] for p in ps) / total
    by = {}
    for p in ps:
        k = p["label"].split("_")[0]
        by[k] = round(by.get(k, 0.0) + p["mass_g"], 1)
    return {"total_g": round(total, 1), "com_mm": tuple(float(v) for v in np.round(com, 1)), "by_kind_g": by}


# ---- jobs -----------------------------------------------------------------------------------------

def _centred(poly) -> Polygon:
    x0, y0, x1, y1 = poly.bounds
    return affinity.translate(poly, -(x0 + x1) / 2, -(y0 + y1) / 2)


def laser_jobs(r=None, kerf_mm: float = LASER_KERF_MM) -> Dict[str, str]:
    """One job per sheet; work zero on the sheet's top centre, focused (laser/job.py). 3 mm
    parts at Sienci's 3 mm row, 6 mm parts at its 6 mm row (LASER)."""
    r = _r(r)

    def prog(cuts, t, title):
        s = LASER[t]
        return laser_job.program(cuts, None, passes=s["passes"], z_step=s["z_step"], cut_mm_s=s["mm_s"],
                                 kerf_mm=kerf_mm, title=f"SparkRover chassis: {title}, {t:g} mm birch plywood")
    xm = motor_xs()[0]
    sp = _centred(spacer(r, xm, 1))
    spacers = [affinity.translate(sp, dx, 0.0) for dx in (-36.0, -12.0, 12.0, 36.0)]
    return {
        "deck_lower": prog([deck_layer(r, False)], 3.0, "deck, lower layer"),
        "deck_upper": prog([deck_layer(r, True)], 3.0, "deck, upper layer (nut traps)"),
        "spacers": prog(spacers, 3.0, "four motor-bracket spacers"),
        "spine_side_l": prog([_centred(side_plate(r))], 6.0, "spine side plate, left"),
        "spine_side_r": prog([_centred(side_plate(r))], 6.0, "spine side plate, right (the same part)"),
        "spine_rear": prog([_centred(rear_plate(r))], 6.0, "spine rear (bay) plate"),
        "spine_front": prog([_centred(front_plate(r))], 6.0, "spine front (head) plate"),
        "spine_rungs": prog([affinity.translate(_centred(mid_rung(r)), -20.0, 0.0),
                             affinity.translate(_centred(top_rung(r)), 20.0, 0.0)], 6.0, "spine rungs, mid and top"),
    }


SPINE_FOOT_STOCK_MM = (40.0, 120.0, 4.0)     # 6061 plate, taped down (the desk robot's hubs' way)
THROUGH_MM = 0.2


def spine_foot_program(r=None, tip_above_top_mm: float = 10.0) -> str:
    """The spine foot from a 40 x 120 x 4 mm 6061 blank, work zero on its top centre. T3 (1 mm
    carbide, the fine aluminium set) cuts the four 3.4 mm holes while the blank holds the
    part; T1 (1/4 in single flute) then cuts the outline free. Feeds as the desk robot's hubs:
    1 mm tool 200 mm/min at 0.2 mm a pass, 1/4 in 900 mm/min at 0.5 mm a pass."""
    r = _r(r)
    t = r.foot_t + THROUGH_MM
    part = spine_foot(r)
    lines = [f"(SparkRover spine foot: 6061, {SPINE_FOOT_STOCK_MM[0]:g} x {SPINE_FOOT_STOCK_MM[1]:g} x "
             f"{SPINE_FOOT_STOCK_MM[2]:g} mm blank; tools: longmill_mk2_tools_aluminium_fine.json)",
             "G21 G90 G17 G94", f"G10 L20 P1 X0 Y0 Z{tip_above_top_mm:.3f}", "G54"]
    w = desk._FloorWriter(1.0, 200.0, 40.0)
    w.step = 0.2
    w.emit("G0 Z5")
    w.emit("T3 M6")
    w.emit("M3 S24000")
    w.emit("G4 P1")
    for interior in part.interiors:
        toolpath.clear(w, Polygon(interior), 1.0, t, 0.4, w.step)
    w.emit("M5")
    w.feed, w.plunge, w.step = 900.0, 200.0, 0.5
    w.emit("G0 Z5")
    w.emit("T1 M6")
    w.emit("M3 S18000")
    w.emit("G4 P1")
    outline = Polygon(part.exterior)
    ring = outline.buffer(6.85, quad_segs=16).difference(outline)
    toolpath.clear(w, ring, 6.35, t, 3.0, w.step)
    w.emit("M5")
    w.emit("G0 Z20")
    w.emit("M2")
    return "\n".join(lines + w.lines) + "\n"


def jobs(r=None) -> Dict[str, str]:
    """{part label: machine file text} for every made part: laser G-code per sheet and the
    spine foot's CNC program."""
    out = laser_jobs(r)
    out["spine_foot"] = spine_foot_program(r)
    return out


# ---- checks ---------------------------------------------------------------------------------------

def _bbox(t: np.ndarray) -> Box:
    v = t.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return Box(*(float(x) for x in (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])))


def _grown(b: Box, tol: float) -> Box:
    return Box(b.x0 - tol, b.x1 + tol, b.y0 - tol, b.y1 + tol, b.z0 - tol, b.z1 + tol)


def _vol(b: Box) -> float:
    return max(0.0, b.x1 - b.x0) * max(0.0, b.y1 - b.y0) * max(0.0, b.z1 - b.z0)


def _isect(a: Box, b: Box) -> Box:
    return Box(max(a.x0, b.x0), min(a.x1, b.x1), max(a.y0, b.y0), min(a.y1, b.y1), max(a.z0, b.z0), min(a.z1, b.z1))


def zone_report(r=None, tol: float = 0.05) -> dict:
    """Each solid's bounding box against the chassis zones: in one zone, in their union only
    (it crosses a boundary between two of our own zones: allowed, integrate.zones tests the
    union), or outside them (not allowed)."""
    zs = [_grown(b, tol) for b in sk.zone_of("chassis")]
    one, union, out = [], [], []
    for label, t in _solids(_r(r)).items():
        bb = _bbox(t)
        if any(z.contains(bb) for z in zs):
            one.append(label)
            continue
        cover = sum(_vol(_isect(bb, z)) for z in sk.zone_of("chassis"))
        (union if cover >= _vol(bb) * (1 - 1e-6) else out).append(label)
    return {"one_zone": one, "union_only": union, "outside": out}


def _hole_present(poly: Polygon, u: float, v: float, d: float) -> bool:
    disc = _c(u, v, d - 0.02)
    return Polygon(poly.exterior).contains(disc) and poly.intersection(disc).area < 1e-6


def _spine_obstacles(r: Chassis) -> Polygon:
    """What stands inside the spine, in its (x, z) section, that a hex key's long leg cannot pass."""
    x = r.spine_x
    return unary_union([box(-x, r.deck_top, x, r.spine_z0),                                  # the foot
                        box(-x, r.spine_z0, -x + r.spine_t, r.rear_top),                     # rear plate
                        box(-x, r.rear_top, x, r.rear_top + r.rung_t),                       # mid rung
                        box(x - r.spine_t, r.front_z[0], x, r.front_z[1]),                   # front plate
                        box(-x, r.spine_top - r.rung_t, x, r.spine_top)])                    # top rung


def key_access(r=None, need_deg: float = 60.0) -> dict:
    """Can an L-key drive every screw head on a side plate's inner face? The short leg in the
    socket along y, the long leg in the spine's (x, z) section, leaving through the open front
    (x = +15) or rear (x = -15) between the rungs and plates; it must swing `need_deg` (one hex
    flat) without striking them. Inside the spine only: what stands outside it (boards in the
    bay or the head) can shorten the swing."""
    from shapely.geometry import LineString
    r = _r(r)
    obst = _spine_obstacles(r)
    out = {}
    for m in side_mounts():
        if m.at[1] < 0:
            continue
        xh, zh = m.at[0], m.at[2]
        best = (0.0, None)
        for side, xf in (("front", r.spine_x), ("rear", -r.spine_x)):
            run, longest = 0.0, 0.0
            dx = abs(xf - xh)
            for k in range(-178, 179):                       # the leg's angle above the horizontal, 0.5 deg steps
                th = math.radians(k * 0.5)
                ok = dx / math.cos(th) <= HEX_KEY["long_mm"] and not LineString(
                    [(xh, zh), (xf, zh + dx * math.tan(th))]).intersects(obst)
                run = run + 0.5 if ok else 0.0
                longest = max(longest, run)
            if longest > best[0]:
                best = (longest, side)
        out[m.name] = {"through": best[1], "swing_deg": best[0]}
    ok = all(v["swing_deg"] >= need_deg for v in out.values())
    detail = "; ".join(f"{k} {v['through']} {v['swing_deg']:g} deg" for k, v in out.items())
    return {"ok": ok, "detail": f"{HEX_KEY['spec']}, >= {need_deg:g} deg each: {detail}", "heads": out}


def harness_pass(r=None, need_w: float = 40.0) -> dict:
    """The spine is open for the front-to-bay harness between the mid rung and the front plate
    (z 136..180) at both its front (x +15) and rear (x -15) faces: nothing of ours stands in
    that slab but the screw heads on the side plates' inner faces (|y| 46..49). `need_w`:
    two FPC ribbons (DSI and camera) side by side, 15-20 mm each."""
    r = _r(r)
    z0, z1 = r.rear_top + r.rung_t, r.front_z[0]
    window = box(-r.spine_x, z0, r.spine_x, z1)
    hit = _spine_obstacles(r).intersection(window).area
    wide = 2 * (r.spine_y - r.spine_t - SHCS["M3"]["head_k"])
    ok = hit < 1e-9 and wide >= need_w and z1 - z0 >= 20.0
    return {"ok": ok, "z": (z0, z1), "clear_y": wide,
            "detail": f"open front and rear, z {z0:g}..{z1:g} ({z1 - z0:g} mm), {wide:g} mm across between the side "
                      f"plates' screw heads (two ribbons need about {need_w:g}); nothing else of ours inside"}


# The front motors' leads, robot frame (left side; the right mirrors y): out of the encoder's
# top, up through the deck's slot, along the deck's top and over the spine foot into the spine's
# open front (below the mid rung), up inside it, through the mid rung's slot, and out of the
# spine's open rear into the bay.
FRONT_LEAD_ROUTE = [(55.0, 11.0, 49.0), (55.0, 6.0, 58.0), (14.0, 6.0, 62.0), (2.0, 4.0, 128.0),
                    (2.0, 4.0, 140.0), (-16.0, 10.0, 150.0)]
MOTOR_LEAD_MM = 200.0          # Pololu: "approximately 200 mm", 1 x 6 female header, 2.54 mm
LEAD_SLACK_MM = 20.0           # strain relief at the encoder and a little play at the joint
LEAD_NEEDED_MM = 300.0         # electrical.py: encoder to its driver in the bay, front motors


def lead_routes(r=None) -> dict:
    """The front motors' lead route, its length, and where the 200 mm lead ends (the joint to
    the electrical's extension). Checked: the route passes the deck through its slot and the
    mid rung through its slot, and touches no spine plate, rung or the foot on the way."""
    r = _r(r)
    pts = np.asarray(FRONT_LEAD_ROUTE, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    to_bay = float(seg.sum())
    reach = MOTOR_LEAD_MM - LEAD_SLACK_MM
    d = pts[-1] - pts[-2]
    joint = pts[-1] + d / np.linalg.norm(d) * (reach - to_bay)      # carried on into the bay
    # Sample the route and test it against our parts' boxes (the slots excepted).
    x, t = r.spine_x, r.spine_t
    boxes = {"spine_foot": Box(-x, x, -r.spine_y, r.spine_y, r.deck_top, r.spine_z0),
             "spine_side": Box(-x, x, r.spine_y - t, r.spine_y, r.spine_z0, r.spine_top),
             "spine_rear": Box(-x, -x + t, -r.spine_y, r.spine_y, r.spine_z0, r.rear_top),
             "spine_front": Box(x - t, x, -r.spine_y, r.spine_y, r.front_z[0], r.front_z[1]),
             "spine_rung_mid": Box(-x, x, -r.spine_y, r.spine_y, r.rear_top, r.rear_top + r.rung_t),
             "deck": Box(-r.deck_x / 2, r.deck_x / 2, -r.deck_y / 2, r.deck_y / 2, r.deck_z, r.deck_top)}
    xm = motor_xs()[0]
    slots = {"deck": _slot(xm - 5.0, -13.0, xm + 5.0, 13.0), "spine_rung_mid": _slot(*MID_SLOT)}
    hits = set()
    for a, b in zip(pts[:-1], pts[1:]):
        for f in np.linspace(0.0, 1.0, 200):
            q = a + (b - a) * f
            for k, bx in boxes.items():
                if bx.x0 < q[0] < bx.x1 and bx.y0 < q[1] < bx.y1 and bx.z0 < q[2] < bx.z1:
                    if k in slots and slots[k].buffer(-1.0).contains(Point(q[0], q[1])):
                        continue                                    # through the slot, 1 mm off its edge
                    hits.add(k)
    ext = LEAD_NEEDED_MM - reach
    ok = not hits and reach > to_bay
    detail = (f"encoder to the spine's rear opening {to_bay:.0f} mm; the {MOTOR_LEAD_MM:g} mm lead less "
              f"{LEAD_SLACK_MM:g} slack ends {reach - to_bay:.0f} mm into the bay, at about "
              f"{tuple(round(float(v)) for v in joint)} (left; right mirrored): the joint (1 x 6 header to a JST XH "
              f"6-pin extension of about {ext:.0f} mm, for the electrical's {LEAD_NEEDED_MM:g}); route hits "
              f"{sorted(hits) or 'nothing'}")
    return {"ok": ok, "to_bay_mm": round(to_bay, 1), "joint_mm": tuple(float(v) for v in joint.round(1)),
            "extension_mm": round(ext, 1), "detail": detail}


def checks(r=None) -> Dict[str, Tuple[bool, str]]:
    r = _r(r)
    dr = drive(r)
    sol = _solids(r)
    m = BOUGHT["gearmotor"]
    b = BOUGHT["bracket"]
    a = BOUGHT["hex_adapter"]
    out: Dict[str, Tuple[bool, str]] = {}
    mg = dr["margins"]
    L = dr["loads_kgcm_per_wheel"]
    out["drive_spin_torque"] = (mg["spin_stall_over_load_at_6V"] >= 2.0,
                                f"spin on the spot at mu 1.0 needs {L['spin_mu_hi']:.3f} kg.cm a wheel; the motor stalls "
                                f"at {motor_at(6.0)['stall_kgcm']:.2f} at 6.0 V: x{mg['spin_stall_over_load_at_6V']}")
    out["drive_ramp_10deg"] = (mg["ramp_stall_over_load_at_6V"] >= 2.0 and mg["ramp_traction_tan_over_mu"] < 1.0,
                               f"{L['ramp_10deg']:.3f} kg.cm a wheel, x{mg['ramp_stall_over_load_at_6V']} at 6.0 V; "
                               f"tan 10 deg / mu 0.8 = {mg['ramp_traction_tan_over_mu']}")
    cr = dr["operating"]["cruise"]
    out["drive_speed"] = (cr["6V"]["m_s"] >= 0.5 and cr["8.4V"]["m_s"] < 1.5,
                          f"cruise at full duty: {cr['6V']['m_s']} m/s at 6.0 V, {cr['7.2V']['m_s']} at 7.2, "
                          f"{cr['8.4V']['m_s']} at 8.4: 0.3-0.5 m/s is reached at every charge (PWM "
                          f"{mg['pwm_for_0.4_m_s_at_7.2V']:.0%} for 0.4 m/s at 7.2 V)")
    cont = [dr["operating"][k][f"{v:g}V"]["a_frac_of_stall"] for k in ("cruise", "ramp_10deg") for v in r.battery_v]
    spin = dr["operating"]["spin_mu_hi"]
    out["drive_continuous_current"] = (max(cont) <= m["current_rule"],
                                       f"cruise and 10 deg ramp at most {max(cont):.0%} of stall current (Pololu: 25 %); "
                                       f"spin-turns, intermittent: {spin['7.2V']['a']} A = "
                                       f"{spin['7.2V']['a_frac_of_stall']:.0%} at 7.2 V, mu 1.0 "
                                       f"({dr['operating']['spin_mu_lo']['7.2V']['a_frac_of_stall']:.0%} at mu 0.8)")
    out["gearbox_load"] = (mg["slip_torque_over_continuous"] < 1.0,
                           f"the tyre slips at {L['slip_mu_hi']:.2f} kg.cm (mu 1.0): "
                           f"{mg['slip_torque_over_continuous']:.0%} of the 4 kg.cm continuous limit")
    # The drive line along y (left side; the right is its mirror).
    enc_gap = 2 * (r.face_y - r.motor_l)
    head_top = r.flange_out + WASHER["M3"]["h"] + BUTTON["M3"]["head_k"]
    engage = r.face_y + m["shaft_l"] - r.adapter_y
    tip = r.face_y + m["shaft_l"]
    set_ok = all(s < engage for s in a["set_screws_from_motor_end"])
    seat = r.adapter_y + a["length"] - (r.wheel_in + r.hub_seat)
    out["drive_line_fit"] = (enc_gap >= 1.0 - 1e-9 and head_top <= r.adapter_y and set_ok and r.flange_out <= r.wheel_in - 2.0
                             and r.face_y + m["bushing_l"] <= r.adapter_y and abs(seat) < 1e-9,
                             f"encoders {enc_gap:.2f} mm apart; face screw heads to |y| {head_top:.2f} (the adapter "
                             f"starts at {r.adapter_y:g}, its far end on the wheel's web); shaft {engage:.2f} mm into the adapter (set screws at "
                             f"{a['set_screws_from_motor_end']}); shaft tip at {tip:.1f}; flange {r.wheel_in - r.flange_out:.1f} "
                             f"mm from the tyre")
    # The bracket screws' heads, under the foot, over the motor can.
    dx_in = b["foot_w"] / 2 - b["notch_inset"] - BUTTON["M3"]["head_d"] / 2
    room = (b["axis_from_foot"] - b["t"]) - math.sqrt((m["can_d"] / 2) ** 2 - dx_in ** 2)
    out["foot_screw_heads_clear_motor"] = (room - BUTTON["M3"]["head_k"] >= 0.2,
                                           f"{room:.2f} mm between the foot and the can at the head's inner edge, "
                                           f"the head is {BUTTON['M3']['head_k']}: {room - BUTTON['M3']['head_k']:.2f} clear")
    out["axle_height"] = (abs(r.foot_outer_z + r.spacer_t - r.deck_z) < 1e-9,
                          f"axle {r.axle_z} + bracket {b['axis_from_foot']} + spacer {r.spacer_t} = deck underside {r.deck_z}")
    # Zones.
    zr = zone_report(r)
    out["zones_union"] = (not zr["outside"], f"outside every chassis zone: {zr['outside'] or 'none'}")
    # Wheels.
    bad = []
    for wname, (wx, wy, wz) in sk.WHEELS.items():
        bb = _bbox(sol[f"wheel_{WHEEL_LABEL[wname]}"])
        c = ((bb.x0 + bb.x1) / 2, (bb.y0 + bb.y1) / 2, (bb.z0 + bb.z1) / 2)
        if (max(abs(c[0] - wx), abs(c[1] - wy), abs(c[2] - wz)) > 0.01 or abs(bb.z0) > 0.01
                or abs((bb.x1 - bb.x0) - sk.WHEEL_D) > 0.05 or abs((bb.y1 - bb.y0) - sk.WHEEL_W) > 1e-6):
            bad.append(wname)
    out["wheels_at_positions"] = (not bad, f"tyres centred on skeleton.WHEELS, OD {sk.WHEEL_D}, {sk.WHEEL_W} wide, "
                                           f"on z = 0: {'all four' if not bad else 'NOT ' + str(bad)}")
    low = min(_bbox(t).z0 for k, t in sol.items() if not k.startswith("wheel_"))
    out["ground_clearance"] = (low >= sk.GROUND_CLEARANCE, f"lowest non-wheel point z {low:.2f} (>= {sk.GROUND_CLEARANCE})")
    # Mount holes.
    deck_l, deck_u, side = deck_layer(r, False), deck_layer(r, True), side_plate(r)
    miss = [mt.name for mt in sk.SHELL_MOUNTS
            if not (_hole_present(deck_l, mt.at[0], mt.at[1], CLEARANCE["M3"])
                    and _hole_present(deck_u, mt.at[0], mt.at[1], CLEARANCE["M3"]))
            and mt.name.startswith("lower")]
    miss += [mt.name for mt in sk.SHELL_MOUNTS if mt.name.startswith("upper")
             and not _hole_present(side, mt.at[0], mt.at[2], CLEARANCE["M3"])]
    out["shell_mount_holes"] = (not miss, f"8 SHELL_MOUNTS clearance holes: missing {miss or 'none'}")
    have = {m_.name for m_ in present_shoulder_bolts(r)}
    lack = [mt.name for mt in sk.SHOULDER_BOLTS if mt.name not in have]
    out["shoulder_bolt_holes"] = (not lack, f"{len(have)} of {len(sk.SHOULDER_BOLTS)} SHOULDER_BOLTS clearance holes in "
                                            f"the side plates; missing {lack or 'none'}")
    ka = key_access(r)
    out["key_access"] = (ka["ok"], ka["detail"])
    ex = []
    for mt in electrical_mounts(r):
        plate = rear_plate(r) if mt.at[0] < 0 else front_plate(r)
        if not _hole_present(plate, mt.at[1], mt.at[2], CLEARANCE[mt.size]):
            ex.append(mt.name)
    out["electrical_mount_holes"] = (not ex, f"{len(electrical_mounts(r))} board points: missing {ex or 'none'}")
    # Fasteners.
    fb = []
    for jt in joints(r) + interface_joints(r):
        if jt.get("nut"):
            ok = jt["proud_mm"] >= SHCS[jt["size"]]["pitch"] - 1e-9
        elif "ok" in jt:
            ok = jt["ok"]
        else:
            ok = jt["thread_in_mm"] >= 1.5 * SHCS[jt["size"]]["d"] - 1e-9
        if not ok:
            fb.append(jt["label"])
    n = len(joints(r)) + len(interface_joints(r))
    out["fasteners"] = (not fb, f"{n} screws: every one a nut plus a pitch or 1.5 d of thread (the gearbox face: also "
                                f"<= its 6.0 mm hole less 0.5; the Pi 5's M2.5: also room for the Pi's own screw in "
                                f"the standoff); short: {fb or 'none'}")
    hb = BATTERY_HOLDER["box"]
    hit = [k for k, t in sol.items() if hb.overlaps(_bbox(t))]
    sz = BATTERY_HOLDER["size_mm"]
    near = min(min(abs(_bbox(t).x0), abs(_bbox(t).x1)) for k, t in sol.items()
               if _bbox(t).z0 < hb.z1 and _bbox(t).z1 > hb.z0 and _bbox(t).y0 < hb.y1 and _bbox(t).y1 > hb.y0
               and not (_bbox(t).x0 < 0 < _bbox(t).x1))
    pb = BATTERY_HOLDER["pack_box"]
    out["battery_volume_clear"] = (not hit and near - pb.x1 >= BATTERY_MARGIN and hb.z0 >= sk.GROUND_CLEARANCE,
                                   f"{sz[0]:g} x {sz[1]:g} x {sz[2]:g} volume {hb}: crossed by {hit or 'nothing'}; "
                                   f"the nearest part beside the pack at |x| {near:.2f} ({near - pb.x1:.2f} mm from it); "
                                   f"its underside z {hb.z0:g} (ground clearance {sk.GROUND_CLEARANCE:g})")
    pk = BATTERY_PACK["size_mm"]
    room = BATTERY_HOLDER["size_mm"]
    lx, ly, ld = BATTERY_HOLDER["lead_hole"]
    lead_ok = abs(ly) - ld / 2 >= pk[1] / 2 and lx + ld / 2 < -R_SPINE_X
    out["battery_pack_fits"] = (all(a_ <= b_ for a_, b_ in zip(pk, room)) and lead_ok,
                                f"{BATTERY_PACK['spec']}, {pk} mm in the {tuple(round(v, 2) for v in room)} mm volume: "
                                f"{tuple(round(b_ - a_, 1) for a_, b_ in zip(pk, room))} mm to spare; strap slots at "
                                f"x +-{BATTERY_HOLDER['strap_x']:g}; lead hole at ({lx:g}, {ly:g}) beyond its -y end")
    hp = harness_pass(r)
    out["harness_pass"] = (hp["ok"], hp["detail"])
    lr = lead_routes(r)
    out["front_lead_routes"] = (lr["ok"], lr["detail"])
    ms = mass_summary(r)
    out["chassis_mass"] = (ms["total_g"] < sk.MASS_TARGET_G[1] * 0.6,
                           f"{ms['total_g']} g at {ms['com_mm']} (under 60 % of the robot's "
                           f"{sk.MASS_TARGET_G[1] / 1000:g} kg ceiling, leaving the rest "
                           f"for battery, electronics, shell and arms)")
    return out


def interference(r=None) -> dict:
    """The chassis' solids against each other with FCL, the integrator's method."""
    import integrate
    return integrate.interference({"chassis": solids(r)})


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the SparkRover chassis' machine files, BOM and checks.")
    p.add_argument("--out", required=True)
    p.add_argument("--fcl", action="store_true", help="also run the FCL interference check")
    a = p.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r = Chassis()
    for name, text in jobs(r).items():
        ext = "nc" if name == "spine_foot" else "gcode"
        (out / f"{name}.{ext}").write_text(text, encoding="utf-8", newline="\n")
    rep = {"chassis": asdict(r), "drive": drive(r), "mass": mass_summary(r),
           "checks": {k: {"ok": v[0], "detail": v[1]} for k, v in checks(r).items()},
           "joints": joints(r), "interface_joints": interface_joints(r)}
    if a.fcl:
        rep["interference"] = interference(r)
    (out / "chassis.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    (out / "parts.json").write_text(json.dumps(parts(r), indent=1, default=str), encoding="utf-8")
    print(json.dumps(rep["checks"], indent=1))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
