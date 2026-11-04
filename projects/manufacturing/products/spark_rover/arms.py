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

"""SparkRover Mini: the arms. Two 2-DOF arms (shoulder pitch, elbow pitch) that wave, point and
gesture, designed inside skeleton.py's "arms" zones and to its shoulder interface.

THE SERVO. ROBOTIS DYNAMIXEL XC330-M288-T at both joints (SERVO, from ROBOTIS' e-manual): a
20 x 34 x 26 mm serial bus servo (TTL half duplex, Protocol 2.0, daisy-chained), full metal
gears, 0.93 N m stall and 81 rpm no-load at its recommended 5.0 V. Its input range is
3.7-6.0 V, so it does NOT run from the 2S pack (6.0-8.4 V): it needs a regulated 5.0 V servo
rail (the electrical subsystem's). The Feetech STS3215 (45 x 25 x 35) does not fit (below).

WHY THE SHOULDER SERVO RIDES IN THE ARM. skeleton.ZONES gives the shoulder servo 27 mm of y
between the spine's face (y 55) and the shell's skin (y 82), 23.5 mm usable (the skin is 2.5
mm thick and keeps 1 mm of gap). A pitch servo has its output along y, so its depth along
its shaft has to fit there: the STS3215 needs 35 mm, the XC330 26 mm (23 case + 3 horn), the
micro bus servos (STS3032, SCS0009) 25-29 mm. None fits, and none passes the shell's 40 mm
opening (a 20 x 34 case has a 39.4 mm diagonal). So the shoulder is INVERTED: the servo's
horn is screwed to a stationary post that reaches out of the body through the opening, and
the servo's case turns with the upper arm round it (the Dynamixel frame trick). The elbow
is the same: the elbow servo rides in the forearm, its horn screwed to the upper arm's
elbow plate. Each link carries the servo of the joint above it.

THE PARTS (per arm; the right arm is the left mirrored in y):
- shoulder_bracket: CNC-milled 6061, 8 mm plate on the spine's side face, the four
  SHOULDER_BOLTS tapped M3 through it (8 mm of thread: the chassis' M3 x 12 puts 6 in, it asks
  for 6.5). Both brackets come from one blank (bracket_program): the mill cuts every outline
  and bore and pilots the tap holes 4.8 deep (the 1 mm tool's 5 mm flute); the holes are then
  drilled 2.5 through on a drill press from the pilots and tapped M3 by hand.
- shoulder_post: printed PETG, a flanged tube (OD 19, bore 16) from the bracket out through
  the shell's opening; its 3 mm end plate takes the shoulder servo's horn on ROBOTIS' four
  PHS M2 x 6 tapping screws, driven up the open bore on the bench. Its flange (35 mm, it
  passes the 40 mm opening) takes two captive M3 x 10 cap screws into M3 threads tapped in
  the bracket, driven from OUTSIDE: a 2.5 mm hex screwdriver goes straight in along y through
  two 4.2 mm access bores in the upper arm (and its cap), across the arm-to-skin gap and the
  shell's opening, to the screw heads (POST_ACCESS, post_access()). Nothing is reached from
  inside the spine.

ASSEMBLY ORDER (the shell's upper halves slide on sideways; the electronics go in before the arms):
  1. chassis: each bracket onto the spine (SHOULDER_BOLTS, M3 x 12 from inside the spine);
  2. shell and electronics;
  3. bench, per arm: horn -> post (4 PHS M2 x 6, up the post's open bore); the two M3 x 10
     into the flange (each threaded past the flange's 2.6 mm retaining lip, so it stays put);
     the shoulder servo into the upper arm's halves, the cap under the outer half; the elbow
     servo into the forearm's halves; the elbow horn -> the upper arm's elbow plate (4 PHS M2
     x 6); the bus cable laid in. Command the shoulder servo to 0 once, so the access bores
     sit over the flange screws;
  4. the arm module's post through the shell's opening, flange on the bracket, the arm
     hanging; the two M3 x 10 driven through the arm's access bores.
- upper_arm_inner / upper_arm_outer (warm white) and shoulder_cap (teal): the upper arm,
  split at y 103.3 round the shoulder servo, the cap held in by a lip under the outer half.
- forearm_inner / forearm_outer (grey): the forearm, the "grey rounded end", round the
  elbow servo.

Every part is a stack of y-bands (a 2D outline in the pitch plane extruded along y), which is
what the printer and the 2.5D mill make, and what the swept checks test exactly.

LINK FRAME (t, s): origin on the joint's axis, t = +x and s = -z at the neutral pose (the arm
hanging straight down, s pointing down the arm). A joint angle q (both joints, axis -y, so
+q raises the arm FORWARD on both sides) turns (t, s) into the robot frame as
x = x0 + t cos q + s sin q, z = z0 + t sin q - s cos q. Band y values are |y| (left arm).

    python projects/manufacturing/products/spark_rover/arms.py --out .tmp/spark_rover_arms
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
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
MANUFACTURING = HERE.parents[1]
ROOT = MANUFACTURING.parents[1]
DESK = MANUFACTURING / "products" / "desk_robot"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
for _p in (DESK, MANUFACTURING / "cnc", MANUFACTURING / "laser"):
    if str(_p) not in sys.path:
        sys.path.append(str(_p))       # appended: this product's own modules win a name clash

import skeleton as sk  # noqa: E402
from skeleton import Box, Mount  # noqa: E402
import design as desk  # noqa: E402  (desk_robot/design.py, read-only: prisms, the CNC floor writer)
import bought_parts as desk_bought  # noqa: E402  (screw_length(), ISO 4762 table)
import toolpath  # noqa: E402

MESHES = HERE / "meshes" / "arms"
CACHE = ROOT / ".tmp" / "vendor_cad"
SIDES = {"left": 1, "right": -1}

# ---- the servo ------------------------------------------------------------------------------
EMANUAL = "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/"
SERVO = {
    "spec": "ROBOTIS DYNAMIXEL XC330-M288-T", "qty": 4, "g": 23.0,
    "source": EMANUAL,
    "size_mm": (20.0, 34.0, 26.0),          # W x H x D (e-manual); D includes the horn
    "case_depth_mm": 23.0,                  # D less the horn: the e-manual's horn-screw figure
    "horn_d_mm": 16.0, "horn_t_mm": 3.0,    # (x330_horn_screw.png) scaled by its 3 mm plate
    "horn_axis_from_top_mm": 11.0,          # estimated from the e-manual figures (+-1 mm)
    "horn_screw_pcd_mm": 11.3,              # scaled from the same figure; holes are 2.6 to take +-0.3
    "horn_screw": "PHS M2 x 6 tapping, through a 3 mm plate, 3 mm into the horn (e-manual: M2 x 8 hits the case)",
    "voltage_v": (3.7, 6.0), "recommended_v": 5.0,
    "stall_nm": {3.7: 0.69, 5.0: 0.93, 6.0: 1.10}, "stall_a": {3.7: 1.34, 5.0: 1.80, 6.0: 2.15},
    "no_load_rpm": {3.7: 59.0, 5.0: 81.0, 6.0: 97.0}, "kt_nm_per_a": 0.517,
    "gear": "full metal gear, 2 bearings", "ratio": 288.35, "resolution": 4096,
    "bus": "TTL half duplex, DYNAMIXEL Protocol 2.0, two daisy-chain ports",
    "case_g": 22.0, "horn_g": 1.0,          # our split of the 23 g (estimate)
    "not_used": {
        "Feetech STS3215": "45.2 x 24.7 x 35 mm: 35 mm along the shaft, the shoulder zone has 23.5 usable; 4-7.4 V "
                           "(also needs regulating on 2S)",
        "Dynamixel XL330-M288-T": "same size, 0.52 N m stall, plastic gears: the XC330's metal gears are taken for "
                                  "the child-push case",
        "Feetech STS3032 / SCS0009": "micro bus servos, 25-29 mm along the shaft, 0.23-0.44 N m",
    },
}
# The vendor STEP: ROBOTIS publishes XL330.stp (the X330 case) behind a download page that
# refuses a plain fetch (it redirects to an error page without a browser's referer), so it
# is not pinned yet and nothing here needs it: the servo is drawn from the e-manual's numbers.
# To use it: download it by hand from the e-manual's "Drawings" section into
# .tmp/vendor_cad/dl/xc330/, put its SHA-256 below, and fetch() checks it.
SOURCES = {"xc330": {"url": "https://www.robotis.com/service/download.php?no=1987", "file": "XL330.stp",
                     "sha256": None, "vendor": "ROBOTIS", "licence": "none stated"}}

PETG_G_MM3 = 1.27e-3
ALU_G_MM3 = 2.70e-3
STEEL_G_MM3 = 7.9e-3
QS = 10                                    # quarter-circle segments of every drawn circle
PHS = {"M2": {"head_d": 4.0, "head_k": 1.6}}          # ISO 7045 pan head (ROBOTIS' PHS screws)
CLEAR = {"M2": 2.4, "M3": 3.4}                       # ISO 273 normal
TAP = {"M2_petg": 1.7, "M3_alu": 2.5}                # pilot in PETG (desk_robot.design), M3 tap drill
CB = {"M2_shcs": (4.4, 2.2), "M2_phs": (4.4, 1.8)}   # counterbore (d, depth) for the heads
SPINE_PLATE_T = 6.0      # chassis.py: the spine's side plates are 6 mm birch plywood at |y| 49..55;
SPINE_SCREW_LEN = 12.0   # its M3 x 12 from inside the spine put 6 mm into our bracket;
SPINE_THREAD_MIN = 6.5   # the lead asks for at least 6.5 mm of thread


@dataclass(frozen=True)
class Arm:
    # y stations (|y|, mm, left arm)
    y_spine: float = 55.0          # the spine's side face (skeleton)
    bracket_t: float = 8.0         # 8 mm 6061 plate: >= 6.5 mm of thread for the spine's M3 x 12
    flange_t: float = 4.0          # the post's flange
    y_in: float = 84.0             # the arm's inboard face: 2 mm off the shell's skin (y 82)
    plate_t: float = 4.8           # the elbow plate: 1.8 counterbore + 3.0 (ROBOTIS' 3 mm plate)
    horn_gap: float = 0.3          # the forearm's face stands off the elbow plate
    y_split: float = 103.3         # print split of both links
    y_lip: float = 114.0           # the shoulder cap's lip
    outer_wall: float = 2.2
    groove_y: Tuple[float, float] = (89.1, 91.4)   # the elbow cable's groove layer
    # pitch-plane sizes (mm)
    half_w: float = 17.5           # 35 mm across, as the sheets draw it
    gap: float = 1.0               # between the links' bodies at the elbow
    neck_hw: float = 6.0           # the upper arm's neck to its elbow plate (inboard layer)
    pocket_clear: float = 0.2      # round the servo case
    end_wall: float = 1.6          # below each servo case
    channel: Tuple[float, float] = (10.0, 13.0)    # the cable channel beside each servo (t)
    arm_hole_r: float = 12.5       # the upper arm's hole round the post (the cable passes here)
    post_r: float = 9.5            # the post's tube
    bore_r: float = 8.0            # its bore: a PHS head at r 5.65 + 2.0 = 7.65
    horn_clear_r: float = 9.0
    groove_r: float = 15.5
    fore_excl_r: float = 20.5      # the forearm's inboard layer stays outside the upper arm's neck
    flange_r: float = 17.5
    flange_x_max: float = 14.0     # the shoulder zone ends at x 15
    # joint limits (deg, axis -y: + raises forward); checked against the zones every 5 deg
    shoulder_deg: Tuple[float, float] = (-70.0, 90.0)
    elbow_deg: Tuple[float, float] = (-90.0, 90.0)

    # ---- what follows -----------------------------------------------------------------
    @property
    def y_horn(self) -> float:              # the horns' mating face (post end / elbow plate)
        return self.y_in + self.plate_t

    @property
    def y_case(self) -> float:              # the servo cases' front face
        return self.y_horn + SERVO["horn_t_mm"]

    @property
    def y_back(self) -> float:              # their back face
        return self.y_case + SERVO["case_depth_mm"]

    @property
    def y_out(self) -> float:               # the arm's outboard face
        return self.y_back + self.outer_wall

    @property
    def y_post_end(self) -> float:          # the post's end plate, 3 mm under the horn
        return self.y_horn - 3.0

    @property
    def case_top(self) -> float:            # s of the case's top (horn) end
        return -SERVO["horn_axis_from_top_mm"]

    @property
    def case_bottom(self) -> float:
        return SERVO["size_mm"][1] - SERVO["horn_axis_from_top_mm"]

    @property
    def s_end(self) -> float:               # the upper arm's body ends here (all layers but the inboard)
        return self.case_bottom + self.pocket_clear + self.end_wall

    @property
    def L1(self) -> float:                  # shoulder axis to elbow axis
        return self.s_end + self.half_w + self.gap

    @property
    def fore_c(self) -> float:              # the centre of the forearm's rounded end
        w = SERVO["size_mm"][0] / 2 + self.pocket_clear
        b = self.case_bottom + self.pocket_clear
        return b - math.sqrt((self.half_w - self.end_wall) ** 2 - w ** 2)

    @property
    def fore_len(self) -> float:            # elbow axis to the forearm's tip
        return self.fore_c + self.half_w


A = Arm()


def _arm(r) -> Arm:
    return r if isinstance(r, Arm) else A


# ---- 2D helpers (link frame t, s) -------------------------------------------------------------

def C(t: float, s: float, r: float) -> Polygon:
    return Point(t, s).buffer(r, quad_segs=QS)


def R(t0: float, s0: float, t1: float, s1: float) -> Polygon:
    return box(t0, s0, t1, s1)


def _rrect(t0, s0, t1, s1, rad) -> Polygon:
    return box(t0 + rad, s0 + rad, t1 - rad, s1 - rad).buffer(rad, quad_segs=QS)


def _holes(pts, d) -> Polygon:
    return unary_union([C(t, s, d / 2) for t, s in pts])


def _pcd(cs: float, pcd: float, n: int = 4, a0: float = 0.0):
    return [(pcd / 2 * math.cos(a0 + 2 * math.pi * k / n), cs + pcd / 2 * math.sin(a0 + 2 * math.pi * k / n))
            for k in range(n)]


# ---- the parts' outlines ------------------------------------------------------------------------

def upper_outlines(a: Arm) -> dict:
    W = a.half_w
    body = C(0, 0, W).union(R(-W, 0, W, a.s_end))
    inboard = body.union(R(-a.neck_hw, a.s_end - 1.0, a.neck_hw, a.L1)).union(C(0, a.L1, W))
    w, cp = SERVO["size_mm"][0] / 2 + a.pocket_clear, a.pocket_clear
    pocket = R(-w, a.case_top - cp, w, a.case_bottom + cp).union(R(a.channel[0], -4.0, a.channel[1], a.case_bottom + cp))
    notch = R(w, a.case_bottom - 3.2, a.channel[1], a.s_end + 1.0)       # the elbow cable leaves the end face
    D = C(0, 0, W).intersection(R(-W - 1, -W - 1, W + 1, 6.0))
    lip = (D.buffer(0.6, quad_segs=QS).difference(D.buffer(0.1, quad_segs=QS))
           .intersection(body.buffer(-1.0, quad_segs=QS)).difference(pocket.buffer(0.2)))
    # the cap's lower band: its D beside the servo plus the lip that hooks under the outer half
    cap_low = D.buffer(-0.1, quad_segs=QS).union(lip).difference(pocket.buffer(0.2))
    return {"body": body, "inboard": inboard, "pocket": pocket, "notch": notch, "D": D, "lip": lip,
            "cap_low": cap_low}


def fore_outlines(a: Arm) -> dict:
    W, c = a.half_w, a.fore_c
    body = C(0, 0, W).union(R(-W, 0, W, c)).union(C(0, c, W))
    w, cp = SERVO["size_mm"][0] / 2 + a.pocket_clear, a.pocket_clear
    pocket = R(-w, a.case_top - cp, w, a.case_bottom + cp).union(R(a.channel[0], -9.0, a.channel[1], 18.0))
    groove = C(0, 0, W + 1.0).difference(C(0, 0, a.groove_r))
    slot = R(a.channel[0] + 0.3, -11.8, a.channel[1] + 0.1, -4.0)        # groove -> channel
    return {"body": body, "pocket": pocket, "groove": groove, "slot": slot}


def bracket_outline(a: Arm) -> Polygon:
    bolts = shoulder_bolts_ts()
    t0 = min(t for t, _ in bolts) - 4.5
    t1 = max(t for t, _ in bolts) + 4.5
    s1 = max(abs(s) for _, s in bolts) + 4.5
    return _rrect(t0, -s1, t1, s1, 4.0).union(C(0, 0, a.flange_r).intersection(R(-40, -40, a.flange_x_max, 40)))


def flange_outline(a: Arm) -> Polygon:
    return C(0, 0, a.flange_r).intersection(R(-40, -40, a.flange_x_max, 40))


# Screw positions (t, s) in their link's frame.
# The post's flange screws, tapped in the bracket and driven from outside through the upper
# arm (at its neutral angle): the only two spots of the arm's root with solid wall round a
# 4.2 mm bore, clear of the servo's pocket (t +-10.2, s -11.2..23.2), its cable channel
# (t 10..13) and the root's outline (r 17.5): behind the servo (t -13.8) and above it (s -14.3).
FLANGE_SCREWS = [(-13.8, 0.0), (0.0, -14.3)]
# The tool: a 2.5 mm hex screwdriver whose round shaft is at most 3.5 mm and whose blade is at
# least 60 mm (a common 2.5 x 75 hex screwdriver); it goes in straight, no ball end needed.
POST_ACCESS = {"tool": "2.5 mm hex screwdriver, round shaft <= 3.5 mm, blade >= 60 mm (e.g. 2.5 x 75)",
               "shaft_d_mm": 3.5, "blade_mm": 60.0, "bore_d_mm": 4.2, "retaining_lip_d_mm": 2.6}
UPPER_SCREWS = [(-14.0, 6.0), (-14.0, 16.0), (15.25, 16.0)]             # join the upper arm's halves
FORE_SCREW_IN = [(0.0, 25.5)]                                            # from the forearm's inboard face
FORE_SCREW_ROOT = [(-12.5, 2.0)]                                         # from its root face, under the plate


def shoulder_bolts_ts() -> List[Tuple[float, float]]:
    """skeleton.SHOULDER_BOLTS (left) in the shoulder's (t, s)."""
    x0, _, z0 = sk.SHOULDERS["left"]
    return [(m.at[0] - x0, z0 - m.at[2]) for m in sk.SHOULDER_BOLTS if m.at[1] > 0]


def horn_screws(cs: float = 0.0) -> List[Tuple[float, float]]:
    return _pcd(cs, SERVO["horn_screw_pcd_mm"])


# ---- the parts as y-bands ----------------------------------------------------------------------

@dataclass
class Part:
    label: str                 # without the side
    link: str                  # 'static' (with the body), 'upper', 'fore'
    kind: str                  # 'made' | 'bought'
    process: str               # 'cnc' | 'print' | 'bought'
    material: str
    colour: str                # 'r g b' or '#hex'
    bands: List[Tuple[float, float, Polygon]]
    density: Optional[float] = None    # g/mm^3; None: mass_g is given
    mass_g: Optional[float] = None
    step: int = 0
    bed: str = ""              # print: 'min' (the band at the lowest y on the bed) or 'max'
    source: str = ""
    spec: str = ""
    solid: bool = True         # False: listed (mass, BOM) but its solid is left to the zone it sits in


def _screw_bands(size, length, head_at, dirn, thread_from, kind="shcs", minor=None):
    """A screw as y-bands: head on the bearing face at |y| head_at, shank toward dirn (+1/-1);
    the shank is drawn at the major diameter up to thread_from and at the pilot (minor) beyond."""
    if kind == "phs":
        hd, hk = PHS[size]["head_d"], PHS[size]["head_k"]
    else:
        hd, hk = desk_bought.SHCS[size]["head_d"], desk_bought.SHCS[size]["head_k"]
    d = desk_bought.SHCS[size]["d"]
    minor = minor or d * 0.8
    tip = head_at + dirn * length
    h0, h1 = sorted((head_at, head_at - dirn * hk))
    bands = [(h0, h1, "head", hd)]
    a0, a1 = sorted((head_at, thread_from))
    if a1 - a0 > 1e-9:
        bands.append((a0, a1, "shank", d))
    b0, b1 = sorted((thread_from, tip))
    if b1 - b0 > 1e-9:
        bands.append((b0, b1, "thread", minor))
    return bands


def _screw_part(label, link, at_ts, size, length, head_at, dirn, thread_from, kind="shcs", minor=None, step=0,
                spec="", solid=True):
    bands = [(y0, y1, C(at_ts[0], at_ts[1], dd / 2)) for y0, y1, _, dd in
             _screw_bands(size, length, head_at, dirn, thread_from, kind, minor)]
    return Part(label, link, "bought", "bought", "steel", ".15 .15 .16", bands, density=STEEL_G_MM3, step=step,
                source="https://www.fasteners.eu/standards/ISO/4762/" if kind == "shcs" else EMANUAL, spec=spec,
                solid=solid)


def fastener_joints(r=None) -> List[dict]:
    """Every screwed joint of one arm: the stack it clamps, the screw it needs (desk_robot's
    screw_length), where its thread goes and the rule it is held to."""
    a = _arm(r)
    y_b1 = a.y_spine + a.bracket_t
    out = []
    # The post's flange onto the bracket: M3 SHCS from OUTSIDE (POST_ACCESS), tapped in the 6061.
    s = desk_bought.screw_length("M3", a.flange_t, nut=False, thread_in_mm=1.5 * 3.0)
    out.append({"joint": "post_flange", "n": len(FLANGE_SCREWS), "at_ts": FLANGE_SCREWS, "link": "static", "step": 4,
                "head_at": y_b1 + a.flange_t, "dir": -1, "kind": "shcs", "into": "the bracket's M3 thread (6061, through)",
                "hole_mm": a.bracket_t, "min_thread_mm": 1.5 * 3.0, **s})
    # The shoulder horn onto the post's end plate: ROBOTIS' PHS M2 x 6 tapping, from inside the bore.
    s = desk_bought.screw_length("M2", 3.0, nut=False, thread_in_mm=3.0)
    out.append({"joint": "shoulder_horn", "n": 4, "at_ts": horn_screws(0.0), "link": "static", "step": 2,
                "head_at": a.y_post_end, "dir": 1, "kind": "phs", "into": "the XC330 horn's tapping holes",
                "hole_mm": SERVO["horn_t_mm"], "min_thread_mm": 3.0, "max_thread_mm": 3.0, **s})
    # The elbow horn onto the upper arm's elbow plate, from the plate's inboard face (counterbored).
    grip = a.plate_t - CB["M2_phs"][1]
    s = desk_bought.screw_length("M2", grip, nut=False, thread_in_mm=3.0)
    out.append({"joint": "elbow_horn", "n": 4, "at_ts": horn_screws(a.L1), "link": "upper", "step": 5,
                "head_at": a.y_in + CB["M2_phs"][1], "dir": 1, "kind": "phs", "into": "the XC330 horn's tapping holes",
                "hole_mm": SERVO["horn_t_mm"], "min_thread_mm": 3.0, "max_thread_mm": 3.0, **s})
    # The upper arm's halves: M2 SHCS from the inboard face, self-tapping into the outer half.
    head = a.y_in + CB["M2_shcs"][1]
    s = desk_bought.screw_length("M2", a.y_split - head, nut=False, thread_in_mm=4.0)
    out.append({"joint": "upper_halves", "n": len(UPPER_SCREWS), "at_ts": UPPER_SCREWS, "link": "upper", "step": 3,
                "head_at": head, "dir": 1, "kind": "shcs", "into": "a 1.7 mm pilot in the outer half (PETG)",
                "hole_mm": a.y_back - a.y_split, "min_thread_mm": 4.0, **s})
    s = desk_bought.screw_length("M2", a.y_split - head, nut=False, thread_in_mm=4.0)
    out.append({"joint": "fore_halves_in", "n": len(FORE_SCREW_IN), "at_ts": FORE_SCREW_IN, "link": "fore", "step": 4,
                "head_at": head, "dir": 1, "kind": "shcs", "into": "a 1.7 mm pilot in the outer half (PETG)",
                "hole_mm": a.y_out - a.outer_wall - a.y_split, "min_thread_mm": 4.0, **s})
    head = a.groove_y[1]
    s = desk_bought.screw_length("M2", a.y_split - head, nut=False, thread_in_mm=4.0)
    out.append({"joint": "fore_halves_root", "n": len(FORE_SCREW_ROOT), "at_ts": FORE_SCREW_ROOT, "link": "fore",
                "step": 4, "head_at": head, "dir": 1, "kind": "shcs", "into": "a 1.7 mm pilot in the outer half (PETG)",
                "hole_mm": a.y_out - a.outer_wall - a.y_split, "min_thread_mm": 4.0, **s})
    return out


@functools.lru_cache(maxsize=4)
def arm_parts(r=None) -> Tuple[Part, ...]:
    """One arm (the left; the right is its mirror), every part as y-bands in its link's frame."""
    a = _arm(r)
    U, F = upper_outlines(a), fore_outlines(a)
    W = a.half_w
    yb0, yb1 = a.y_spine, a.y_spine + a.bracket_t
    bolts = shoulder_bolts_ts()
    hs0, hsL = horn_screws(0.0), horn_screws(a.L1)
    out: List[Part] = []

    # ---- static: bracket, post, the shoulder servo's horn ----
    br = bracket_outline(a).difference(unary_union([_holes(bolts, TAP["M3_alu"]), _holes(FLANGE_SCREWS, TAP["M3_alu"]),
                                                    C(0, 0, a.bore_r)]))
    out.append(Part("shoulder_bracket", "static", "made", "cnc", "6061-T6 aluminium, 8 mm plate",
                    ".75 .76 .78", [(yb0, yb1, br)], density=ALU_G_MM3, step=1))
    fl = flange_outline(a)
    yf = yb1 + a.flange_t
    ring = C(0, 0, a.post_r).difference(C(0, 0, a.bore_r))
    end = C(0, 0, a.post_r).difference(_holes(hs0, 2.6)).difference(C(0, 0, 3.0))
    out.append(Part("shoulder_post", "static", "made", "print", "PETG", "#5A5F66",
                    [(yb1, yf, fl.difference(C(0, 0, a.bore_r)).difference(_holes(FLANGE_SCREWS, CLEAR["M3"]))),
                     (yf, sk.SHOULDERS["left"][1], ring), (sk.SHOULDERS["left"][1], a.y_post_end, ring),
                     (a.y_post_end, a.y_horn, end)],
                    density=PETG_G_MM3, step=1, bed="min"))
    horn = C(0, 0, SERVO["horn_d_mm"] / 2).difference(_holes(hs0, 1.7))
    out.append(Part("servo_shoulder_horn", "static", "bought", "bought", "aluminium (ROBOTIS horn)", ".70 .70 .72",
                    [(a.y_horn, a.y_case, horn)], mass_g=SERVO["horn_g"], step=2, source=EMANUAL, spec=SERVO["spec"]))
    case = R(-SERVO["size_mm"][0] / 2, a.case_top, SERVO["size_mm"][0] / 2, a.case_bottom)
    out.append(Part("servo_shoulder", "upper", "bought", "bought", "XC330 case", ".10 .10 .11",
                    [(a.y_case, a.y_back, case)], mass_g=SERVO["case_g"], step=2, source=EMANUAL, spec=SERVO["spec"]))

    # ---- the upper arm ----
    cbS = _holes(UPPER_SCREWS, CB["M2_shcs"][0])
    clS = _holes(UPPER_SCREWS, CLEAR["M2"])
    piS = _holes(UPPER_SCREWS, TAP["M2_petg"])
    hole = C(0, 0, a.arm_hole_r)
    relief = C(0, a.L1, 3.0)
    y0, y1, y2 = a.y_in, a.y_in + CB["M2_phs"][1], a.y_in + CB["M2_shcs"][1]
    inb = U["inboard"].difference(hole).difference(relief)
    out.append(Part("upper_arm_inner", "upper", "made", "print", "PETG", "#F4F1ED", [
        (y0, y1, inb.difference(_holes(hsL, CB["M2_phs"][0])).difference(cbS)),
        (y1, y2, inb.difference(_holes(hsL, 2.6)).difference(cbS)),
        (y2, a.y_horn, inb.difference(_holes(hsL, 2.6)).difference(clS)),
        (a.y_horn, a.y_case, U["body"].difference(hole).difference(clS).difference(U["notch"])),
        (a.y_case, a.y_split, U["body"].difference(U["pocket"]).difference(clS).difference(U["notch"]))],
        density=PETG_G_MM3, step=3, bed="min"))
    out.append(Part("upper_arm_outer", "upper", "made", "print", "PETG", "#F4F1ED", [
        (a.y_split, a.y_lip, U["body"].difference(U["pocket"]).difference(piS)),
        (a.y_lip, a.y_back, U["body"].difference(U["pocket"]).difference(piS)
         .difference(U["D"].union(U["lip"]).buffer(0.2, quad_segs=QS))),
        (a.y_back, a.y_out, U["body"].difference(U["D"].buffer(0.1, quad_segs=QS)))],
        density=PETG_G_MM3, step=3, bed="min"))
    out.append(Part("shoulder_cap", "upper", "made", "print", "PETG", "#26B6A4", [
        (a.y_lip + 0.1, a.y_back, U["cap_low"]), (a.y_back, a.y_out, U["D"].buffer(-0.1, quad_segs=QS))],
        density=PETG_G_MM3, step=3, bed="max"))
    # The access bores for the post's two flange screws (POST_ACCESS), through every layer.
    bores = _holes(FLANGE_SCREWS, POST_ACCESS["bore_d_mm"])
    for p in out:
        if p.label in ("upper_arm_inner", "upper_arm_outer", "shoulder_cap"):
            p.bands = [(b0, b1, poly.difference(bores)) for b0, b1, poly in p.bands]
    hornE = C(0, a.L1, SERVO["horn_d_mm"] / 2).difference(_holes(hsL, 1.7))
    out.append(Part("servo_elbow_horn", "upper", "bought", "bought", "aluminium (ROBOTIS horn)", ".70 .70 .72",
                    [(a.y_horn, a.y_case, hornE)], mass_g=SERVO["horn_g"], step=4, source=EMANUAL, spec=SERVO["spec"]))

    # ---- the forearm (its own frame: the elbow at (0, 0)) ----
    excl, hc = C(0, 0, a.fore_excl_r), C(0, 0, a.horn_clear_r)
    g0, g1 = a.groove_y
    out.append(Part("forearm_inner", "fore", "made", "print", "PETG", "#5A5F66", [
        (y0, y2, F["body"].difference(excl).difference(_holes(FORE_SCREW_IN, CB["M2_shcs"][0]))),
        (y2, g0, F["body"].difference(excl).difference(_holes(FORE_SCREW_IN, CLEAR["M2"]))),
        (g0, g1, F["body"].difference(hc).difference(F["groove"]).difference(F["slot"])
         .difference(_holes(FORE_SCREW_ROOT, CB["M2_shcs"][0])).difference(_holes(FORE_SCREW_IN, CLEAR["M2"]))),
        (g1, a.y_case, F["body"].difference(hc).difference(F["slot"])
         .difference(_holes(FORE_SCREW_ROOT + FORE_SCREW_IN, CLEAR["M2"]))),
        (a.y_case, a.y_split, F["body"].difference(F["pocket"]).difference(_holes(FORE_SCREW_ROOT + FORE_SCREW_IN, CLEAR["M2"])))],
        density=PETG_G_MM3, step=4, bed="max"))
    out.append(Part("forearm_outer", "fore", "made", "print", "PETG", "#5A5F66", [
        (a.y_split, a.y_back, F["body"].difference(F["pocket"]).difference(_holes(FORE_SCREW_ROOT + FORE_SCREW_IN, TAP["M2_petg"]))),
        (a.y_back, a.y_out, F["body"])], density=PETG_G_MM3, step=4, bed="min"))
    out.append(Part("servo_elbow", "fore", "bought", "bought", "XC330 case", ".10 .10 .11",
                    [(a.y_case, a.y_back, case)], mass_g=SERVO["case_g"], step=4, source=EMANUAL, spec=SERVO["spec"]))

    # ---- fasteners ----
    for j in fastener_joints(a):
        size = "M3" if j["size"] == "M3" else "M2"
        if j["joint"] == "post_flange":
            thread_from, minor = yb1, TAP["M3_alu"] - 0.05
        elif j["joint"] in ("shoulder_horn", "elbow_horn"):
            thread_from, minor = a.y_horn, 1.6
        else:
            thread_from, minor = a.y_split, 1.6
        spec = f"{size} x {j['length']} {'ISO 4762 socket head cap screw' if j['kind'] == 'shcs' else 'PHS tapping (ROBOTIS)'}"
        for k, p in enumerate(j["at_ts"]):
            out.append(_screw_part(f"screw_{j['joint']}_{k}", j["link"], p, size, j["length"], j["head_at"], j["dir"],
                                   thread_from, j["kind"], minor, step=j["step"], spec=spec, solid=j.get("solid", True)))
    return tuple(out)


# ---- poses ---------------------------------------------------------------------------------------

def link_poses(a: Arm, q_shoulder_deg: float = 0.0, q_elbow_deg: float = 0.0) -> Dict[str, Tuple[float, float, float]]:
    """(x0, z0, angle rad) of each link frame's origin in the robot's x-z plane."""
    x0, _, z0 = sk.SHOULDERS["left"]
    q1, q2 = math.radians(q_shoulder_deg), math.radians(q_elbow_deg)
    xe, ze = x0 + a.L1 * math.sin(q1), z0 - a.L1 * math.cos(q1)
    return {"static": (x0, z0, 0.0), "upper": (x0, z0, q1), "fore": (xe, ze, q1 + q2)}


def _place_poly(p: Polygon, pose) -> Polygon:
    x0, z0, q = pose
    c, s = math.cos(q), math.sin(q)
    return affinity.affine_transform(p, [c, s, s, -c, x0, z0])


def _place_pts(pts: np.ndarray, pose) -> np.ndarray:
    x0, z0, q = pose
    c, s = math.cos(q), math.sin(q)
    return np.column_stack([x0 + pts[:, 0] * c + pts[:, 1] * s, z0 + pts[:, 0] * s - pts[:, 1] * c])


def _polys(g) -> List[Polygon]:
    return [q for q in (g.geoms if isinstance(g, MultiPolygon) else [g]) if not q.is_empty]


def _extrude(poly_xz: Polygon, y0: float, y1: float, sy: int) -> np.ndarray:
    """Closed prism over a polygon in the robot's x-z plane, from |y| y0 to y1, outward normals."""
    out = []
    for q in _polys(poly_xz):
        t = desk._prism(q, y0, y1)                 # (X=x, Y=z, Z=|y|)
        t = t[:, :, [0, 2, 1]].copy()              # -> (x, |y|, z): a reflection
        if sy < 0:
            t[:, :, 1] *= -1.0                      # a second reflection: orientation restored
        else:
            t = t[:, [0, 2, 1], :]                  # one reflection: flip the winding back
        out.append(t)
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def part_triangles(p: Part, side: str = "left", q=(0.0, 0.0), r=None) -> np.ndarray:
    a = _arm(r)
    pose = link_poses(a, *q)[p.link]
    return np.concatenate([_extrude(_place_poly(poly, pose), y0, y1, SIDES[side]) for y0, y1, poly in p.bands])


# ---- mass properties --------------------------------------------------------------------------------

def _mass_props(p: Part, side="left", r=None) -> dict:
    import trimesh
    tris = part_triangles(p, side, r=r)
    mp = trimesh.triangles.mass_properties(tris, density=1.0)
    vol = float(mp["volume"])
    g = p.mass_g if p.mass_g is not None else vol * p.density
    rho = g / vol
    return {"g": g, "com": np.asarray(mp["center_mass"], float), "I": np.asarray(mp["inertia"], float) * rho,
            "volume_mm3": vol}


def link_inertia(link: str, side: str = "left", r=None) -> dict:
    """Mass (kg), centre of mass (robot frame at the neutral pose, mm) and the inertia matrix about
    the COM (robot axes at neutral, kg m^2) of one link: every part on it, servo and screws included."""
    tot_g, mom = 0.0, np.zeros(3)
    props = [_mass_props(p, side, r) for p in arm_parts(_arm(r)) if p.link == link]
    for m in props:
        tot_g += m["g"]
        mom += m["g"] * m["com"]
    com = mom / tot_g
    I = np.zeros((3, 3))
    for m in props:
        d = m["com"] - com
        I += m["I"] + m["g"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return {"mass_kg": tot_g / 1000.0, "com_mm": com, "inertia_kgm2": I * 1e-9}


# ---- the contract ------------------------------------------------------------------------------

def parts(r=None) -> List[dict]:
    a = _arm(r)
    out = []
    for side in SIDES:
        for p in arm_parts(a):
            m = _mass_props(p, side, a)
            row = {"label": f"{p.label}_{side}", "step": p.step, "kind": p.kind, "process": p.process,
                   "material": p.material, "mass_g": round(m["g"], 2), "centre_mm": tuple(round(v, 2) for v in m["com"]),
                   "source": p.source or None, "colour": p.colour, "link": f"{p.link}_{side}" if p.link != "static" else "body"}
            if p.spec:
                row["spec"] = p.spec
            if p.process == "cnc":
                row["job"] = "shoulder_brackets"
            out.append(row)
    return out


def solids(r=None, q: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
    """Every part, assembled, at the joint angles q (deg, by joint name; default neutral). The
    post is split at the shell's skin (y 82) into its inside and outside halves, so each solid's
    box lies in one zone (the integrator checks boxes)."""
    a = _arm(r)
    q = q or {}
    out = {}
    for side in SIDES:
        qq = (q.get(f"shoulder_{side}", 0.0), q.get(f"elbow_{side}", 0.0))
        for p in arm_parts(a):
            if not p.solid:
                continue
            if p.label == "shoulder_post":
                ys = sk.SHOULDERS["left"][1]
                inside = Part(p.label, p.link, p.kind, p.process, p.material, p.colour, [b for b in p.bands if b[1] <= ys + 1e-9])
                outside = Part(p.label, p.link, p.kind, p.process, p.material, p.colour, [b for b in p.bands if b[0] >= ys - 1e-9])
                out[f"shoulder_post_{side}_in_body"] = part_triangles(inside, side, qq, a)
                out[f"shoulder_post_{side}_outside"] = part_triangles(outside, side, qq, a)
            else:
                out[f"{p.label}_{side}"] = part_triangles(p, side, qq, a)
    return out


def mounts(r=None) -> List[Mount]:
    """The skeleton's SHOULDER_BOLTS: the thread is ours (M3, tapped through the bracket). The
    post fixes to our own bracket from outside, so it needs nothing more of the chassis (the
    arm_post holes through the spine are no longer used)."""
    return list(sk.SHOULDER_BOLTS)


def post_access(r=None, clear_mm: float = 0.2) -> dict:
    """Can the post's flange screws be driven from outside, last (shell and electronics in)?
    The screwdriver (POST_ACCESS) goes straight in along y over each screw, the arm hanging
    at 0: its shaft must clear every part on the way (the access bores, the post, the horn),
    stay inside the shell's opening, and reach the head with its blade."""
    a = _arm(r)
    ps = arm_parts(a)
    pose = link_poses(a, 0.0, 0.0)
    rt = POST_ACCESS["shaft_d_mm"] / 2
    j = next(x for x in fastener_joints(a) if x["joint"] == "post_flange")
    y_head = j["head_at"] + desk_bought.SHCS["M3"]["head_k"]
    rows, ok = [], True
    for k, (t_, s_) in enumerate(FLANGE_SCREWS):
        shaft = _place_poly(C(t_, s_, rt), pose["static"])
        worst, who = math.inf, None
        for p in ps:
            if p.label == f"screw_post_flange_{k}":
                continue
            for y0, y1, poly in p.bands:
                if y1 <= y_head + 1e-9:
                    continue
                d = _place_poly(poly, pose[p.link]).distance(shaft)
                if d < worst:
                    worst, who = d, p.label
        in_opening = math.hypot(t_, s_) + rt <= sk.SHELL_ARM_OPENING_D / 2 - 0.5
        reach = a.y_out - y_head
        good = worst >= clear_mm and in_opening and reach + 5.0 <= POST_ACCESS["blade_mm"]
        ok &= good
        rows.append({"screw": k, "at_ts": (t_, s_), "clearance_mm": round(worst, 3), "nearest": who,
                     "in_opening": in_opening, "reach_mm": round(reach, 2)})
    passes = flange_outline(a).buffer(1.0).within(C(0, 0, sk.SHELL_ARM_OPENING_D / 2))
    heads = max(math.hypot(t_, s_) + desk_bought.SHCS["M3"]["head_d"] / 2 for t_, s_ in FLANGE_SCREWS)
    ok &= passes and heads <= sk.SHELL_ARM_OPENING_D / 2 - 1.0
    detail = (f"{POST_ACCESS['tool']}: straight in along y through the arm's {POST_ACCESS['bore_d_mm']} mm bores, "
              f"{min(r_['clearance_mm'] for r_ in rows):.2f} mm clear of every part, {max(r_['reach_mm'] for r_ in rows):.0f} mm "
              f"from the arm's face to the heads, inside the 40 mm opening; the flange (with its captive screws, "
              f"r {heads:.1f}) passes the opening: {passes}")
    return {"ok": bool(ok), "rows": rows, "detail": detail, "tool": POST_ACCESS}


def joints(r=None) -> List[dict]:
    """The joints as the simulator wants them. Each joint's anchor (and each link's body origin)
    sits on the joint axis at the arm's mid-thickness, inside the link's material."""
    a = _arm(r)
    s_ = SERVO
    tau, w0 = s_["stall_nm"][5.0], s_["no_load_rpm"][5.0] * 2 * math.pi / 60
    ymid = (a.y_in + a.y_out) / 2
    out = []
    for side, sy in SIDES.items():
        x0, _, z0 = sk.SHOULDERS[side]
        for name, parent, child, anchor, lim in (
                ("shoulder", "body", "upper_arm", (x0, sy * ymid, z0), a.shoulder_deg),
                ("elbow", "upper_arm", "forearm", (x0, sy * ymid, z0 - a.L1), a.elbow_deg)):
            out.append({"name": f"{name}_{side}", "type": "hinge", "parent": parent if parent == "body" else f"{parent}_{side}",
                        "child": f"{child}_{side}", "axis": (0.0, -1.0, 0.0), "anchor_mm": tuple(round(v, 3) for v in anchor),
                        "limits_deg": lim, "limits_rad": tuple(round(math.radians(v), 5) for v in lim),
                        "neutral": "0 deg: the arm hangs straight down; + raises it forward",
                        "servo": s_["spec"], "max_torque_nm": tau, "max_velocity_rad_s": round(w0, 3),
                        "supply_v": 5.0, "position_resolution": s_["resolution"]})
    return out


def links(r=None) -> Dict[str, dict]:
    """Each moving link for the Newton model: mass, COM and inertia (about the COM, robot axes at
    neutral), and the COM relative to the body origin (its joint's anchor)."""
    a = _arm(r)
    out = {}
    jt = {j["child"]: j for j in joints(a)}
    for side in SIDES:
        for link, name in (("upper", "upper_arm"), ("fore", "forearm")):
            li = link_inertia(link, side, a)
            anchor = np.asarray(jt[f"{name}_{side}"]["anchor_mm"], float)
            out[f"{name}_{side}"] = {
                "mass_kg": round(li["mass_kg"], 5), "com_mm": tuple(np.round(li["com_mm"], 3)),
                "origin_mm": tuple(anchor), "com_from_origin_m": tuple(np.round((li["com_mm"] - anchor) / 1000.0, 6)),
                "inertia_kgm2": np.round(li["inertia_kgm2"], 10).tolist(),
                "parts": [f"{p.label}_{side}" for p in arm_parts(a) if p.link == link]}
    return out


# ---- swept checks ----------------------------------------------------------------------------------

def _angles(lim, step=5.0) -> List[float]:
    lo, hi = lim
    n = int(round((hi - lo) / step))
    return [lo + k * (hi - lo) / n for k in range(n + 1)]


def _verts(parts_: Sequence[Part]) -> np.ndarray:
    pts = []
    for p in parts_:
        for _, _, poly in p.bands:
            for q in _polys(poly):
                pts.append(np.asarray(q.exterior.coords))
    return np.vstack(pts)


def sweep(r=None, step: float = 5.0) -> dict:
    """The swept arm against arm_l / arm_r at every `step` degrees of both joints (y is fixed by
    the bands, so the x-z extents decide), the two links against each other at every elbow
    angle, and the upper arm against the static parts at every shoulder angle."""
    a = _arm(r)
    ps = arm_parts(a)
    zone = dict(sk.ZONES["arms"])["arm_l"]
    moving = {k: [p for p in ps if p.link == k] for k in ("upper", "fore")}
    V = {k: _verts(v) for k, v in moving.items()}
    ymin = min(b[0] for k in moving for p in moving[k] for b in p.bands)
    ymax = max(b[1] for k in moving for p in moving[k] for b in p.bands)
    worst = {"x0": math.inf, "x1": -math.inf, "z0": math.inf, "z1": -math.inf}
    outside = []
    n = 0
    for q1 in _angles(a.shoulder_deg, step):
        for q2 in _angles(a.elbow_deg, step):
            n += 1
            pose = link_poses(a, q1, q2)
            pts = np.vstack([_place_pts(V[k], pose[k]) for k in V])
            x0, z0 = pts.min(0)
            x1, z1 = pts.max(0)
            worst = {"x0": min(worst["x0"], x0), "x1": max(worst["x1"], x1), "z0": min(worst["z0"], z0), "z1": max(worst["z1"], z1)}
            if x0 < zone.x0 - 1e-6 or x1 > zone.x1 + 1e-6 or z0 < zone.z0 - 1e-6 or z1 > zone.z1 + 1e-6:
                outside.append((q1, q2, round(x0, 2), round(x1, 2), round(z0, 2), round(z1, 2)))
    margins = {"x0": worst["x0"] - zone.x0, "x1": zone.x1 - worst["x1"], "z0": worst["z0"] - zone.z0, "z1": zone.z1 - worst["z1"],
               "y0": ymin - zone.y0, "y1": zone.y1 - ymax}
    # links against each other (only the elbow angle matters) and against the static parts
    def clearance(group_a, pose_a, group_b, pose_b):
        best = math.inf
        for pa in group_a:
            for ya0, ya1, polya in pa.bands:
                A_ = _place_poly(polya, pose_a)
                for pb in group_b:
                    for yb0, yb1, polyb in pb.bands:
                        if min(ya1, yb1) - max(ya0, yb0) > 1e-9:
                            best = min(best, A_.distance(_place_poly(polyb, pose_b)))
        return best
    body_parts = lambda grp: [p for p in grp if not p.label.startswith("screw")]
    elbow = min((clearance(body_parts(moving["upper"]), link_poses(a, 0, q2)["upper"], body_parts(moving["fore"]),
                           link_poses(a, 0, q2)["fore"]), q2) for q2 in _angles(a.elbow_deg, step))
    static = [p for p in ps if p.link == "static" and not p.label.startswith("screw")]
    shoulder = min((clearance(body_parts(moving["upper"]), link_poses(a, q1)["upper"], static, link_poses(a, q1)["static"]), q1)
                   for q1 in _angles(a.shoulder_deg, step))
    return {"samples": n, "outside": outside, "margins_mm": {k: round(v, 3) for k, v in margins.items()},
            "extent_mm": {k: round(v, 2) for k, v in worst.items()},
            "elbow_clearance_mm": round(elbow[0], 3), "elbow_worst_deg": elbow[1],
            "shoulder_clearance_mm": round(shoulder[0], 3), "shoulder_worst_deg": shoulder[1]}


def feasible_shoulder_range(r=None, step: float = 5.0) -> Tuple[float, float]:
    """The widest shoulder range (in `step`s, within +-90) whose sweep, with the elbow's full range,
    stays in arm_l: how a.shoulder_deg was chosen."""
    a = _arm(r)
    zone = dict(sk.ZONES["arms"])["arm_l"]
    ps = [p for p in arm_parts(a) if p.link in ("upper", "fore")]
    V = {k: _verts([p for p in ps if p.link == k]) for k in ("upper", "fore")}

    def ok(q1):
        for q2 in _angles(a.elbow_deg, step):
            pose = link_poses(a, q1, q2)
            pts = np.vstack([_place_pts(V[k], pose[k]) for k in V])
            (x0, z0), (x1, z1) = pts.min(0), pts.max(0)
            if x0 < zone.x0 or x1 > zone.x1 or z0 < zone.z0 or z1 > zone.z1:
                return False
        return True
    if not ok(0.0):
        return (0.0, 0.0)
    lo = 0.0
    while lo - step >= -90.0 and ok(lo - step):
        lo -= step
    hi = 0.0
    while hi + step <= 90.0 and ok(hi + step):
        hi += step
    return (lo, hi)


# ---- sizing ------------------------------------------------------------------------------------

CHILD_PUSH_N = 10.0          # a child pushing or waving the hand, at the tip, any direction in the pitch plane
WAVE = {"deg": 90.0, "s": 0.5, "profile": "trapezoid, a third accelerating, a third cruising, a third braking"}


def sizing(r=None, side: str = "left") -> dict:
    """Torque and speed at both joints at 5.0 V, from the links' masses and inertias."""
    a = _arm(r)
    g = 9.81
    up, fo = link_inertia("upper", side, a), link_inertia("fore", side, a)
    x0, _, z0 = sk.SHOULDERS["left"]
    c1 = np.array([up["com_mm"][0] - x0, z0 - up["com_mm"][2]]) / 1000.0         # (t, s) m, at neutral
    c2 = np.array([fo["com_mm"][0] - x0, (z0 - a.L1) - fo["com_mm"][2]]) / 1000.0
    L1 = a.L1 / 1000.0
    m1, m2 = up["mass_kg"], fo["mass_kg"]
    tau_s, tau_e = 0.0, 0.0
    for q1 in _angles(a.shoulder_deg, 1.0):
        for q2 in _angles(a.elbow_deg, 5.0):
            t1, t12 = math.radians(q1), math.radians(q1 + q2)
            # lever arms (robot x) of each COM about the shoulder and the elbow
            x1 = c1[0] * math.cos(t1) + c1[1] * math.sin(t1)
            xe = L1 * math.sin(t1)
            x2 = c2[0] * math.cos(t12) + c2[1] * math.sin(t12)
            tau_s = max(tau_s, abs(g * (m1 * x1 + m2 * (xe + x2))))
            tau_e = max(tau_e, abs(g * m2 * x2))
    Iyy = lambda li: li["inertia_kgm2"][1][1]
    r2 = np.array([0.0, L1]) + c2
    I_s = Iyy(up) + m1 * float(c1 @ c1) + Iyy(fo) + m2 * float(r2 @ r2)        # elbow straight: the worst
    I_e = Iyy(fo) + m2 * float(c2 @ c2)
    th = math.radians(WAVE["deg"])
    w_peak = th / (WAVE["s"] * 2.0 / 3.0)          # trapezoid in thirds: theta = w (2T/3)
    alpha = w_peak / (WAVE["s"] / 3.0)
    T_st, w0 = SERVO["stall_nm"][5.0], SERVO["no_load_rpm"][5.0] * 2 * math.pi / 60
    reach_s = (a.L1 + a.fore_len) / 1000.0
    reach_e = a.fore_len / 1000.0
    rows = {}
    for name, tg, I, reach in (("shoulder", tau_s, I_s, reach_s), ("elbow", tau_e, I_e, reach_e)):
        need = tg + I * alpha
        w_avail = w0 * (1.0 - need / T_st)
        push = CHILD_PUSH_N * reach
        rows[name] = {"gravity_nm": round(tg, 4), "inertia_kgm2": I, "wave_accel_nm": round(I * alpha, 5),
                      "wave_need_nm": round(need, 4), "stall_nm": T_st, "torque_margin": round(T_st / need, 1),
                      "wave_peak_rad_s": round(w_peak, 3), "speed_at_need_rad_s": round(w_avail, 3),
                      "speed_margin": round(w_avail / w_peak, 2), "push_nm": round(push, 3),
                      "push_over_stall": round(push / T_st, 2),
                      "suggested_current_limit_a": round(max(0.15, 3.0 * need / SERVO["kt_nm_per_a"]), 2)}
    return {"links": {"upper_kg": round(m1, 4), "fore_kg": round(m2, 4)}, "joints": rows, "wave": WAVE,
            "push_n": CHILD_PUSH_N,
            "note": "the rotor's inertia reflected through the 288:1 gearbox is not included (ROBOTIS publishes it "
                    "only in a downloadable sheet); the servo's own profile generator absorbs it; masses are solid "
                    "PETG volumes (an upper bound)"}


def cable(r=None) -> dict:
    """The bus cable's route and the slack each joint needs (no hollow shafts)."""
    a = _arm(r)
    sh = a.shoulder_deg[1] - a.shoulder_deg[0]
    el = a.elbow_deg[1] - a.elbow_deg[0]
    r_post = (a.post_r + a.arm_hole_r) / 2
    r_groove = a.groove_r + 0.6
    return {
        "route": "the body's bus -> through the shell's opening, in the annulus round the post (r 9.5-12.5) -> "
                 "into the upper arm's channel beside the shoulder servo (t 10-13) -> the shoulder servo's port A; "
                 "its port B -> down the channel -> out of the upper arm's end face (notch) -> round the forearm's "
                 "root in the groove (r 15.5, y 89.1-91.4, covered by the upper arm's elbow plate) -> through the "
                 "slot into the forearm's channel -> the elbow servo",
        "shoulder": {"wrap_radius_mm": r_post, "range_deg": sh, "slack_mm": round(math.radians(sh) * r_post, 1),
                     "stored": "a loose turn round the post inside the body (the shoulder zone, y 63-78)",
                     "limit": f"the cable is not what limits the shoulder: one turn round the post (2 pi r = "
                              f"{2 * math.pi * r_post:.0f} mm) covers {sh:.0f} deg with slack to spare; the joint stops at its limits"},
        "elbow": {"wrap_radius_mm": r_groove, "range_deg": el, "slack_mm": round(math.radians(el) * r_groove, 1),
                  "stored": "an S-fold in the upper arm's channel (34 mm long)",
                  "limit": f"the groove takes {math.radians(el) * r_groove:.0f} mm over the full {el:.0f} deg; with "
                           f"55 mm stored the cable allows {math.degrees(55.0 / r_groove):.0f} deg"},
    }


# ---- printing ------------------------------------------------------------------------------------

def print_bands(p: Part, side: str = "left") -> List[Tuple[float, float, Polygon]]:
    """The part's bands in its print orientation: z up from the bed. A part printed 'max' (its
    outboard face on the bed) is turned over about its t axis (s -> -s, y reversed), a proper
    rotation; the right arm's part is the left's mirror (t -> -t)."""
    lo = min(b[0] for b in p.bands)
    hi = max(b[1] for b in p.bands)
    out = []
    for y0, y1, poly in p.bands:
        q = poly
        if p.bed == "max":
            q = affinity.scale(q, 1.0, -1.0, origin=(0, 0))
            z0, z1 = hi - y1, hi - y0
        else:
            z0, z1 = y0 - lo, y1 - lo
        if side == "right":
            q = affinity.scale(q, -1.0, 1.0, origin=(0, 0))
        out.append((z0, z1, q))
    return sorted(out, key=lambda b: b[0])


def _bridge(free: Polygon, support, span_mm: float) -> Tuple[bool, float]:
    """Is `free` a bridge: over a hole of the band below, or carried along two opposite sides
    no more than span_mm apart? Returns (bridge, its span)."""
    mrr = free.minimum_rotated_rectangle
    xs = np.asarray(mrr.exterior.coords)[:4]
    edges = [(xs[i], xs[(i + 1) % 4]) for i in range(4)]
    lens = [float(np.hypot(*(b - a))) for a, b in edges]
    holes = unary_union([Polygon(q.exterior) for q in _polys(support)])
    if holes.buffer(1e-3).contains(free) and min(lens) <= span_mm:
        return True, min(lens)                  # over a hole: carried all round
    carried = support.buffer(0.05)
    for k in (0, 1):
        span = lens[(k + 1) % 4]                # the distance between edges k and k + 2
        if span > span_mm:
            continue
        held = []
        for a, b in (edges[k], edges[k + 2]):
            strip = LineString([tuple(a), tuple(b)]).buffer(0.6)
            held.append(free.boundary.intersection(strip).intersection(carried).length / max(lens[k], 1e-9))
        if min(held) >= 0.5:
            return True, span
    return False, min(lens)


def print_check(p: Part, bridge_mm: float = 25.0) -> dict:
    """Each band against the one under it: material beyond the band below (plus the band's own
    height, the 45 degree rule) must be a bridge (carried on both long sides, no wider than
    `bridge_mm`). Returns the widest bridge and whether the part prints without support."""
    bands = print_bands(p)
    bad = []
    widest_bridge = 0.0
    for (z0, z1, lower), (u0, u1, upper) in zip(bands, bands[1:]):
        support = lower.buffer(u1 - u0, quad_segs=QS)
        free = upper.difference(support)
        for f in _polys(free):
            if f.area < 0.05:
                continue
            ok, width = _bridge(f, support, bridge_mm)
            if ok:
                widest_bridge = max(widest_bridge, width)
            else:
                bad.append({"z": round(u0, 2), "area_mm2": round(f.area, 2), "width_mm": round(width, 2)})
    x0, y0, x1, y1 = unary_union([b[2] for b in bands]).bounds
    return {"ok": not bad, "overhangs": bad, "widest_bridge_mm": round(widest_bridge, 2),
            "size_mm": (round(x1 - x0, 1), round(y1 - y0, 1), round(bands[-1][1], 2))}


def print_triangles(p: Part, side: str = "left") -> np.ndarray:
    out = []
    for z0, z1, q in print_bands(p, side):
        for g in _polys(q):
            out.append(desk._prism(g, z0, z1))
    tris = np.concatenate(out)
    lo = tris.reshape(-1, 3).min(0)
    hi = tris.reshape(-1, 3).max(0)
    tris[:, :, 0] -= (lo[0] + hi[0]) / 2
    tris[:, :, 1] -= (lo[1] + hi[1]) / 2
    return tris


def stl_text(tris: np.ndarray, name: str) -> str:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    lines = [f"solid {name}"]
    for f, t in zip(n, tris):
        lines.append(f" facet normal {f[0]:.6e} {f[1]:.6e} {f[2]:.6e}\n  outer loop")
        for v in t:
            lines.append(f"   vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
        lines.append("  endloop\n endfacet")
    lines.append(f"endsolid {name}")
    return "\n".join(lines) + "\n"


# ---- the CNC job --------------------------------------------------------------------------------

BRACKET_STOCK_MM = (110.0, 52.0, 8.0)      # 8 mm 6061 flat bar, sawn, taped down
BRACKET_CENTRES = ((-26.5, 0.0), (26.5, 0.0))
THROUGH_MM = 0.2
T3_FLUTE_MM, T1_FLUTE_MM = 5.0, 19.0       # longmill_mk2_tools_aluminium_fine.json
PILOT_MM = 4.8                             # T3's pilot for each tap hole (its flute is 5 mm)


def bracket_program(r=None, tip_above_top_mm: float = 10.0) -> str:
    """Both shoulder brackets (they are the same part: every feature goes through) from one blank,
    in work coordinates on the blank's top centre, X along the robot's x, Y along its z.
    T3 (1 mm) pilots the twelve 2.5 mm tap holes (six a bracket) 4.8 deep (its flute is 5 mm and its 1/8 in
    shank would rub deeper in a 2.5 mm hole) while the brackets are still held by the blank;
    T1 (1/4 in) then cuts each 16 mm bore (it lightens the bracket and shows the post's bore) and
    cuts each bracket free along its outline.
    Tool table: longmill_mk2_tools_aluminium_fine.json. Feeds as the desk robot's hubs.
    Afterwards, by hand: drill 2.5 mm through on a drill press from each pilot, tap M3 (8 mm)."""
    a = _arm(r)
    t = a.bracket_t + THROUGH_MM
    lines = [f"(SparkRover Mini shoulder brackets: two from one 110 x 52 x {a.bracket_t:g} mm 6061 blank)",
             "(tap holes piloted 4.8 deep: drill 2.5 through on a drill press, then tap M3)", "G21 G90 G17 G94",
             f"G10 L20 P1 X0 Y0 Z{tip_above_top_mm:.3f}", "G54"]
    outline = bracket_outline(a)
    bolts = shoulder_bolts_ts()

    def place(g, cx, cy):
        return affinity.translate(affinity.scale(g, 1.0, -1.0, origin=(0, 0)), cx, cy)   # (t, s) -> (X, Y = -s)
    w = desk._FloorWriter(1.0, 200.0, 40.0)
    w.step = 0.2
    for line in ("G0 Z5", "T3 M6", "M3 S24000", "G4 P1"):
        w.emit(line)
    for cx, cy in BRACKET_CENTRES:
        for p, d in [(b, TAP["M3_alu"]) for b in bolts + FLANGE_SCREWS]:
            toolpath.clear(w, place(C(p[0], p[1], d / 2), cx, cy), 1.0, min(t, PILOT_MM), 0.4, w.step)
    w.emit("M5")
    w.feed, w.plunge, w.step = 900.0, 200.0, 0.5
    for line in ("G0 Z5", "T1 M6", "M3 S18000", "G4 P1"):
        w.emit(line)
    for cx, cy in BRACKET_CENTRES:
        toolpath.clear(w, place(C(0, 0, a.bore_r), cx, cy), 6.35, t, 3.0, w.step)
    ring = unary_union([place(outline.buffer(6.85, quad_segs=QS).difference(outline), cx, cy) for cx, cy in BRACKET_CENTRES])
    toolpath.clear(w, ring, 6.35, t, 3.0, w.step)
    for line in ("M5", "G0 Z20", "M2"):
        w.emit(line)
    return "\n".join(lines + w.lines) + "\n"


def cnc_check(r=None) -> dict:
    a = _arm(r)
    t = a.bracket_t + THROUGH_MM
    outline = bracket_outline(a)
    fits_t1 = not C(0, 0, a.bore_r).buffer(-6.35 / 2).is_empty
    fits_t3 = not C(0, 0, TAP["M3_alu"] / 2).buffer(-0.5).is_empty
    W, H, T = BRACKET_STOCK_MM
    x0, y0, x1, y1 = unary_union([affinity.translate(outline.buffer(6.85), cx, cy) for cx, cy in BRACKET_CENTRES]).bounds
    parts_in = all(abs(cx) + max(-outline.bounds[0], outline.bounds[2]) <= W / 2 for cx, _ in BRACKET_CENTRES)
    return {"t3_depth_ok": min(t, PILOT_MM) <= T3_FLUTE_MM + 1e-9, "t1_depth_ok": t <= T1_FLUTE_MM,
            "t1_fits_access_hole": fits_t1, "tap_holes_finished_by_hand": t > PILOT_MM,
            "t3_fits_tap_hole": fits_t3, "stock_t_matches": abs(T - a.bracket_t) < 1e-6, "parts_on_blank": parts_in,
            "cut_depth_mm": round(t, 3), "ring_extent_mm": (round(x1 - x0, 1), round(y1 - y0, 1))}


def jobs(r=None) -> Dict[str, str]:
    """{part label: machine file text}: the brackets' G-code (one program makes both) and every
    printed part as an ASCII STL (mm) in its print orientation for the SV08."""
    a = _arm(r)
    out = {"shoulder_brackets": bracket_program(a)}
    for side in SIDES:
        for p in arm_parts(a):
            if p.process == "print":
                out[f"{p.label}_{side}"] = stl_text(print_triangles(p, side), f"spark_rover {p.label}_{side}")
    return out


# ---- checks ---------------------------------------------------------------------------------------

SV08_BED_MM = (350.0, 350.0, 345.0)
ARM_MASS_BUDGET_G = 150.0         # per arm, moving + static: ours (the robot's 1.5-2.0 kg leaves it)


def _in_zones(tris: np.ndarray, tol=0.05) -> bool:
    """As integrate.zones(): the solid's box inside one arms zone, or every vertex inside some
    arms zone (a part that passes from one zone into the next, like the post through the
    shell's opening: shoulder_l -> opening_l -> arm_l). The parts are prisms along y over
    outlines in x-z, so their vertices bound them within each band."""
    v = tris.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    zs = [Box(z.x0 - tol, z.x1 + tol, z.y0 - tol, z.y1 + tol, z.z0 - tol, z.z1 + tol) for z in sk.zone_of("arms")]
    if any(z.x0 <= lo[0] and hi[0] <= z.x1 and z.y0 <= lo[1] and hi[1] <= z.y1 and z.z0 <= lo[2] and hi[2] <= z.z1
           for z in zs):
        return True
    inside = np.zeros(len(v), bool)
    for z in zs:
        inside |= ((v[:, 0] >= z.x0) & (v[:, 0] <= z.x1) & (v[:, 1] >= z.y0) & (v[:, 1] <= z.y1)
                   & (v[:, 2] >= z.z0) & (v[:, 2] <= z.z1))
    return bool(inside.all())


def checks(r=None) -> Dict[str, Tuple[bool, str]]:
    a = _arm(r)
    out: Dict[str, Tuple[bool, str]] = {}
    v_lo, v_hi = SERVO["voltage_v"]
    out["servo_supply"] = (True, f"XC330 runs {v_lo}-{v_hi} V: NOT the 2S pack (6.0-8.4 V) directly; needs a regulated "
                                 f"5.0 V servo rail, 4 servos, stall {SERVO['stall_a'][5.0]} A each (current-limited, below)")
    sz = sizing(a)
    for jn, row in sz["joints"].items():
        out[f"{jn}_torque"] = (row["torque_margin"] >= 3.0,
                               f"wave needs {row['wave_need_nm']} N m (gravity {row['gravity_nm']}, accel "
                               f"{row['wave_accel_nm']}) of {row['stall_nm']} stall: x{row['torque_margin']}")
        out[f"{jn}_speed"] = (row["speed_margin"] >= 1.2,
                              f"90 deg in 0.5 s peaks at {row['wave_peak_rad_s']} rad/s; the servo gives "
                              f"{row['speed_at_need_rad_s']} at that load: x{row['speed_margin']}")
        out[f"{jn}_child_push"] = (row["push_over_stall"] < 1.0,
                                   f"{CHILD_PUSH_N} N at the tip = {row['push_nm']} N m, {row['push_over_stall']} of stall: "
                                   f"the current-limited servo yields and the metal gear train sees less than its stall load")
    sw = sweep(a)
    out["swept_in_zone"] = (not sw["outside"], f"{sw['samples']} samples every 5 deg; margins {sw['margins_mm']}")
    out["elbow_self_clearance"] = (sw["elbow_clearance_mm"] >= 0.9,
                                   f"upper arm to forearm {sw['elbow_clearance_mm']} mm at worst (elbow {sw['elbow_worst_deg']} deg)")
    out["shoulder_static_clearance"] = (sw["shoulder_clearance_mm"] >= 0.9,
                                        f"upper arm to post/bracket/horn {sw['shoulder_clearance_mm']} mm")
    # the shell: its opening (r 20 round the axis, in the skin y 79.5-82) and its skin
    ps = arm_parts(a)
    rmax = 0.0
    for p in ps:
        for y0, y1, poly in p.bands:
            if y1 > sk.SHOULDERS["left"][1] - sk.SHELL_WALL and y0 < sk.SHOULDERS["left"][1]:
                c = np.asarray(_polys(poly)[0].exterior.coords)
                rmax = max(rmax, float(np.hypot(c[:, 0], c[:, 1]).max()))
    open_clear = sk.SHELL_ARM_OPENING_D / 2 - rmax
    y_moving = min(b[0] for p in ps if p.link != "static" for b in p.bands)
    out["opening_clearance"] = (open_clear >= 1.0 and y_moving - sk.SHOULDERS["left"][1] >= 1.0,
                                f"the post passes the 40 mm opening {open_clear:.2f} mm clear (it does not turn); the "
                                f"turning arm keeps {y_moving - sk.SHOULDERS['left'][1]:.2f} mm off the skin at every angle")
    inside = max(b[1] for p in ps for b in p.bands if b[1] <= sk.SHOULDERS["left"][1] - sk.SHELL_WALL
                 and p.label != "shoulder_post")
    out["shell_gap_inside"] = (inside <= sk.SHOULDERS["left"][1] - sk.SHELL_WALL - sk.SHELL_GAP + 1e-9,
                               f"inside the body everything ends at |y| {inside:.2f} (skin's inner face 79.5, gap 1)")
    sol = solids(a)
    bad = [k for k, t in sol.items() if not _in_zones(t)]
    out["solids_in_zones"] = (not bad, "neutral pose; " + (", ".join(bad) if bad else "every solid in the arms zones (its box in one, or every vertex in one)"))
    # fasteners
    rows = []
    ok = True
    for j in fastener_joints(a):
        t_in = j["thread_in_mm"]
        good = t_in >= j["min_thread_mm"] - 1e-9 and t_in <= j.get("max_thread_mm", j["hole_mm"]) + 1e-9
        ok &= good
        rows.append(f"{j['joint']}: {j['size']}x{j['length']} {t_in:g} mm in")
    out["fasteners"] = (ok, "; ".join(rows))
    into = SPINE_SCREW_LEN - SPINE_PLATE_T
    bolts_ok = all(abs(abs(m.at[1]) - a.y_spine) < 1e-9 and m.axis[1] * m.at[1] > 0 and m.holder == "arms"
                   for m in sk.SHOULDER_BOLTS)
    on_plate = all(bracket_outline(a).buffer(-2.0).contains(Point(p)) for p in shoulder_bolts_ts())
    out["spine_bolt_thread"] = (a.bracket_t >= SPINE_THREAD_MIN and into <= a.bracket_t and bolts_ok and on_plate,
                                f"{len(sk.SHOULDER_BOLTS)} SHOULDER_BOLTS tapped M3 through {a.bracket_t:g} mm of 6061 "
                                f"(>= {SPINE_THREAD_MIN}); the chassis' M3 x {SPINE_SCREW_LEN:g} from inside the spine puts "
                                f"{into:g} mm in and its tip stays in the bracket")
    acc = post_access(a)
    out["post_fitted_from_outside"] = (acc["ok"], acc["detail"])
    # masses
    per = {}
    for p in parts(a):
        if p["label"].endswith("_left"):
            per[p["label"][:-5]] = p["mass_g"]
    arm_g = sum(per.values())
    out["arm_mass"] = (arm_g <= ARM_MASS_BUDGET_G, f"{arm_g:.1f} g per arm (budget {ARM_MASS_BUDGET_G:g}); "
                                                     f"upper link {sz['links']['upper_kg'] * 1000:.1f} g, forearm "
                                                     f"{sz['links']['fore_kg'] * 1000:.1f} g")
    # printing and milling
    pr = {p.label: print_check(p) for p in ps if p.process == "print"}
    out["prints_without_support"] = (all(v["ok"] for v in pr.values()),
                                     "; ".join(f"{k} bridge {v['widest_bridge_mm']}" for k, v in pr.items()))
    out["prints_fit_sv08"] = (all(all(s <= b for s, b in zip(v["size_mm"], SV08_BED_MM)) for v in pr.values()),
                              "largest " + str(max((v["size_mm"] for v in pr.values()), key=lambda s: s[0] * s[1])))
    cc = cnc_check(a)
    out["cnc_bracket"] = (all(v for k, v in cc.items() if isinstance(v, bool)), json.dumps(cc))
    out["joint_limits_feasible"] = (True, f"shoulder {a.shoulder_deg} (the widest in 5 deg steps: "
                                          f"{feasible_shoulder_range(a)}), elbow {a.elbow_deg}")
    return out


# ---- the vendor STEP (optional) ---------------------------------------------------------------------

def fetch(name: str = "xc330") -> Path:
    """Verify a by-hand download of the vendor STEP against its pinned SHA-256 (none pinned yet)."""
    import hashlib
    src = SOURCES[name]
    p = CACHE / "dl" / name / src["file"]
    if src["sha256"] is None:
        raise RuntimeError(f"{name}: no SHA-256 pinned; download {src['url']} by hand into {p.parent} and pin it")
    if not p.exists():
        raise FileNotFoundError(p)
    got = hashlib.sha256(p.read_bytes()).hexdigest()
    if got != src["sha256"]:
        raise RuntimeError(f"{name}: SHA-256 {got}, expected {src['sha256']}")
    return p


def write_meshes(r=None, out: Path = MESHES) -> dict:
    """meshes/arms/<label>.stl (binary, metres, robot frame at neutral) and index.json (colours)."""
    a = _arm(r)
    out.mkdir(parents=True, exist_ok=True)
    index = {}
    for side in SIDES:
        for p in arm_parts(a):
            if p.label.startswith("screw"):
                continue
            f = f"{p.label}_{side}.stl"
            desk.write_stl(out / f, part_triangles(p, side, r=a) * 0.001, f"spark_rover {p.label}_{side}")
            index[f"{p.label}_{side}"] = {"file": f, "colour": p.colour, "link": p.link}
    (out / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    return index


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SparkRover Mini's arms: checks, joints, machine files.")
    ap.add_argument("--out", help="write the jobs, joints and checks here")
    ap.add_argument("--meshes", action="store_true", help="also write meshes/arms/ (robot frame STLs)")
    a = ap.parse_args(argv)
    ch = checks()
    print(json.dumps({k: v for k, v in ch.items()}, indent=1, default=str))
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        for k, text in jobs().items():
            (out / (k + (".nc" if k == "shoulder_brackets" else ".stl"))).write_text(text, encoding="utf-8", newline="\n")
        (out / "arms.json").write_text(json.dumps({"joints": joints(), "links": links(), "sizing": sizing(),
                                                   "cable": cable(), "sweep": sweep(), "parts": parts(),
                                                   "design": asdict(A)}, indent=1, default=str), encoding="utf-8")
        print(f"wrote {out}")
    if a.meshes:
        write_meshes()
    return 0 if all(v[0] for v in ch.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
