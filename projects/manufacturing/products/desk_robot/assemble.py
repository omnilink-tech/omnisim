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

"""The desk robot, assembled step by step on Newton, each step checked before the next.

For every step of the assembly guide (robot.STEPS) this writes one world: the robot as
built so far as ONE rigid body (every part screwed on so far, welded where physics left
it), standing on a two-rail build stand, and the step's new parts laid out in a kit on
the bench. Over its own harness (headless, its own ports) it then, part by part:

1. picks the part from the kit and puts it APPROACH_MM off its seat, along its fastener
   axis (the direction a screw goes in);
2. moves it to the seat along that axis in INCREMENT_MM steps (`/scene/set_pose`, one
   physics step each), reading `/sim/contacts` after every increment. A contact before
   the seat means something is in the way: the part backs up and comes on in
   REFINE_MM steps, and stops at the first contact;
3. lets it go (gravity, SETTLE_S) so physics rests it on its mating face, except a part
   that is held while it is fixed (the wheels, pressed onto their shafts);
4. measures, from the engine's poses and contacts: where it came to rest against its
   seat, how far each of its holes is off the hole it screws into (perpendicular to the
   screw), how deep it sits in anything else, and which bodies it touches.

A screwed joint is a weld once it is tight: "the screw is in" is represented by the part
reaching its seat within SEAT_TOL_MM with its holes on the mating holes within the
clearance a screw has (design.CLEARANCE), and the part then becomes part of the robot's
rigid body for the next step, AT THE POSE PHYSICS LEFT IT IN. The screws' heads and nuts
join the body with it, so the next step's parts meet them. Then the whole sub-assembly is
watched for another SETTLE_S: nothing may move more than DRIFT_TOL_MM.

Step 6 also spins the wheels (the drive controller, on the stand: wheels in the air);
step 7 also stands the finished robot on the bench (one rigid body: gearmotors do not
back-drive) and drives it. Spin and drive use projects/samples/demos/controllers/desk_robot_drive.

Writes release/assembly/assembly_log.json, physical_step_<k>.png (k = 1..7, the robot after
each step), physical_step_6_spin.png, physical_step_7_standing.png, physical_step_7_drive.png.
Exit status 0 only if every step passes.

    python projects/manufacturing/products/desk_robot/assemble.py [--port 6911] [--until 7]

The parts, their seats, holes, fasteners and mating faces come from ONE function,
`kit(r, mesh_url)`, an adapter over design.py and robot.py: robot.py does not (yet)
expose collision shapes, fastener positions or seats, so the adapter derives them from
design.py's numbers and takes only the visuals from robot.assembly().

Simulation only: a screw is not simulated (no thread, no preload); its head and nut are
solid shapes, its shank is not (it would sit inside the holes the plates do not have in
their collision boxes).
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import MultiPolygon, Point, Polygon

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import design  # noqa: E402
import robot as robot_mod  # noqa: E402
from design import BOUGHT, Robot  # noqa: E402

ROOT = design.ROOT
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"
OUT = HERE / "release/assembly"
TMP = ROOT / ".tmp/desk_robot_assemble"
MM = 1e-3

# ---- tolerances, fixed before any run --------------------------------------------------
# SEAT: the tightest clearance a screw has here (an M3 into a standoff's thread through a
#   3.4 mm hole: 0.2 mm). A part further than that from its seat cannot take its screw.
# PEN: the solver's own resting penetration is the instrument's resolution. MuJoCo's soft
#   contact at its default stiffness (newtonContactKe 2500) rests a body ~0.1-0.2 mm into
#   what holds it, whatever its mass; anything deeper than 0.3 mm is geometry, not softness.
#   (Step 1 measures this floor: a plate on two rails, nothing to interfere.)
# DRIFT: in SETTLE_S after settling, nothing may move more than a quarter of SEAT.
SEAT_TOL_MM = 0.2
PEN_TOL_MM = 0.3
DRIFT_TOL_MM = 0.05
SETTLE_S = 0.5
APPROACH_MM = 5.0
INCREMENT_MM = 0.5
REFINE_MM = 0.05
STAND_H_MM = 15.0          # the build stand's rails: the plate's underside 15 mm up, so the
                           # caster and (from step 6) the wheels hang clear of the bench
DT_MS = 2
# Contact stiffness for the assembly worlds. The engine's default (2500 N/m) is a spring
# that rests a body weight / ke into its support (measured: the 14 g plate 0.056 mm, the
# 110 g battery 0.55 mm), which is foam, not plywood, PLA or steel, and is deeper than the
# clearances being checked. 50 kN/m rests the battery ~0.02 mm in. The spin and drive
# worlds keep the default, as the drive test (tests/test_desk_robot_drive.py) does.
ASSEMBLY_CONTACT_KE = 50000.0
# A resting part's contacts are summed over this many single steps (a stiff contact on a
# cylinder's end flickers between its points from one step to the next).
CONTACT_POLL = 5

NO_LOAD_RAD_S = BOUGHT["gearmotor"]["no_load_rpm"] * 2 * math.pi / 60


# ---- rigid transforms (metres) ------------------------------------------------------------

class Pose:
    """A rigid transform: x_world = R @ x_local + t."""

    def __init__(self, R=None, t=None):
        self.R = np.eye(3) if R is None else np.asarray(R, dtype=float).reshape(3, 3)
        self.t = np.zeros(3) if t is None else np.asarray(t, dtype=float).reshape(3)

    def __matmul__(self, o: "Pose") -> "Pose":
        return Pose(self.R @ o.R, self.R @ o.t + self.t)

    def inv(self) -> "Pose":
        return Pose(self.R.T, -self.R.T @ self.t)

    def apply(self, p) -> np.ndarray:
        return np.asarray(p, dtype=float) @ self.R.T + self.t

    @staticmethod
    def aa(axis, angle, t=(0.0, 0.0, 0.0)) -> "Pose":
        a = np.asarray(axis, dtype=float)
        a = a / np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
        return Pose(R, t)

    def axis_angle(self) -> Tuple[float, float, float, float]:
        R = self.R
        c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
        ang = math.acos(c)
        if ang < 1e-9:
            return (0.0, 0.0, 1.0, 0.0)
        if abs(ang - math.pi) < 1e-6:
            # axis from the symmetric part
            M = (R + np.eye(3)) / 2
            i = int(np.argmax(np.diag(M)))
            v = M[:, i] / math.sqrt(max(M[i, i], 1e-12))
            return (float(v[0]), float(v[1]), float(v[2]), ang)
        v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
        return (float(v[0]), float(v[1]), float(v[2]), ang)


def _frame_z(axis) -> np.ndarray:
    """A rotation whose third column is `axis` (a cylinder's axis is its local z)."""
    z = np.asarray(axis, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# ---- shapes ----------------------------------------------------------------------------------

@dataclass
class Prim:
    """One convex collision shape in its part's frame (metres). Cylinder axis = local z."""
    kind: str                       # "box" | "cyl" | "sphere"
    pose: Pose
    size: Tuple[float, ...]         # box (sx, sy, sz); cyl (radius, height); sphere (radius,)
    friction: Optional[float] = None

    def moved(self, T: Pose) -> "Prim":
        return Prim(self.kind, T @ self.pose, self.size, self.friction)

    def half(self) -> np.ndarray:
        if self.kind == "box":
            return np.asarray(self.size) / 2
        if self.kind == "cyl":
            return np.array([self.size[0], self.size[0], self.size[1] / 2])
        return np.array([self.size[0]] * 3)

    def volume(self) -> float:
        if self.kind == "box":
            return float(np.prod(self.size))
        if self.kind == "cyl":
            return math.pi * self.size[0] ** 2 * self.size[1]
        return 4 / 3 * math.pi * self.size[0] ** 3

    def corners(self) -> np.ndarray:
        h = self.half()
        c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
        return self.pose.apply(c)


def box_mm(centre, size, R=None, friction=None) -> Prim:
    return Prim("box", Pose(R, np.asarray(centre, float) * MM), tuple(float(s) * MM for s in size), friction)


def cyl_mm(centre, axis, radius, height, friction=None) -> Prim:
    return Prim("cyl", Pose(_frame_z(axis), np.asarray(centre, float) * MM), (radius * MM, height * MM), friction)


def sphere_mm(centre, radius, friction=None) -> Prim:
    return Prim("sphere", Pose(None, np.asarray(centre, float) * MM), (radius * MM,), friction)


def ring_mm(centre, axis_y: int, r_in, r_out, y0, y1, n=12) -> List[Prim]:
    """An annulus about a y-parallel axis through `centre` (x, z used), from y0 to y1, as n boxes.

    Each box's inner face is a plane `r_in` from the axis, so the bore is a regular n-gon
    whose inscribed circle has radius r_in: a shaft of radius < r_in passes without touching.
    """
    out = []
    ro = r_out * math.cos(math.pi / n)
    w = 2 * ro * math.tan(math.pi / n)
    cx, _, cz = centre
    for k in range(n):
        a = 2 * math.pi * k / n
        u = np.array([math.cos(a), 0.0, math.sin(a)])
        v = np.array([0.0, 1.0, 0.0])
        R = np.column_stack([u, v, np.cross(u, v)])
        c = np.array([cx, (y0 + y1) / 2, cz]) + u * (r_in + ro) / 2
        out.append(box_mm(c, (ro - r_in, abs(y1 - y0), w), R))
    return out


def _rectilinear(poly: Polygon) -> bool:
    for ring in [poly.exterior, *poly.interiors]:
        pts = list(ring.coords)
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            if abs(x1 - x0) > 1e-6 and abs(y1 - y0) > 1e-6:
                return False
    return True


def rect_boxes(poly, fill_below_mm2: float = 20.0) -> List[Tuple[float, float, float, float]]:
    """A footprint as axis-aligned rectangles (x0, y0, x1, y1), mm.

    Holes smaller than `fill_below_mm2` (screw holes) are filled: a screw's shank stands in
    them, and a convex solver cannot represent them anyway. A footprint that is not
    rectilinear once those are filled falls back to its bounding rectangle.
    """
    polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    out = []
    for p in polys:
        if p.is_empty:
            continue
        p = Polygon(p.exterior, [i for i in p.interiors if Polygon(i).area >= fill_below_mm2])
        if not _rectilinear(p):
            out.append(p.bounds)
            continue
        xs = sorted({round(x, 6) for ring in [p.exterior, *p.interiors] for x, _ in ring.coords})
        ys = sorted({round(y, 6) for ring in [p.exterior, *p.interiors] for _, y in ring.coords})
        cols = []
        for i in range(len(xs) - 1):
            xm = (xs[i] + xs[i + 1]) / 2
            runs, start = [], None
            for j in range(len(ys) - 1):
                inside = p.contains(Point(xm, (ys[j] + ys[j + 1]) / 2))
                if inside and start is None:
                    start = j
                if not inside and start is not None:
                    runs.append((start, j))
                    start = None
            if start is not None:
                runs.append((start, len(ys) - 1))
            cols.append(runs)
        open_: Dict[Tuple[int, int], int] = {}
        for i, runs in enumerate(cols + [[]]):
            for key in list(open_):
                if key not in runs:
                    i0 = open_.pop(key)
                    out.append((xs[i0], ys[key[0]], xs[i], ys[key[1]]))
            for key in runs:
                open_.setdefault(key, i)
    return out


def bands_to_boxes(bands, offset_mm) -> List[Prim]:
    """A printed part's (z0, z1, footprint) bands as boxes, in the robot frame."""
    ox, oy, oz = offset_mm
    out = []
    for z0, z1, poly in bands:
        for x0, y0, x1, y1 in rect_boxes(poly):
            out.append(box_mm(((x0 + x1) / 2 + ox, (y0 + y1) / 2 + oy, (z0 + z1) / 2 + oz), (x1 - x0, y1 - y0, z1 - z0)))
    return out


# ---- signed distances and penetration ------------------------------------------------------

def sdf(prim: Prim, p_local: np.ndarray) -> np.ndarray:
    if prim.kind == "box":
        q = np.abs(p_local) - prim.half()
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    if prim.kind == "cyl":
        d = np.stack([np.hypot(p_local[:, 0], p_local[:, 1]) - prim.size[0], np.abs(p_local[:, 2]) - prim.size[1] / 2], axis=1)
        return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(axis=1), 0)
    return np.linalg.norm(p_local, axis=1) - prim.size[0]


_SAMPLES: Dict[tuple, np.ndarray] = {}


def surface_points(prim: Prim) -> np.ndarray:
    """Points on the shape's surface (local frame): every corner and edge, faces at <= 1 mm."""
    key = (prim.kind, tuple(round(s, 9) for s in prim.size))
    if key in _SAMPLES:
        return _SAMPLES[key]
    sp = min(1.0 * MM, max(min(prim.half()) / 2, 0.1 * MM))
    if prim.kind == "box":
        h = prim.half()
        pts = []
        for ax in range(3):
            o = [i for i in range(3) if i != ax]
            g = [np.linspace(-h[i], h[i], max(2, int(math.ceil(2 * h[i] / sp)) + 1)) for i in o]
            A, B = np.meshgrid(g[0], g[1], indexing="ij")
            for s in (-1, 1):
                P = np.zeros((A.size, 3))
                P[:, o[0]], P[:, o[1]], P[:, ax] = A.ravel(), B.ravel(), s * h[ax]
                pts.append(P)
        out = np.concatenate(pts)
    elif prim.kind == "cyl":
        r, hh = prim.size[0], prim.size[1] / 2
        na = max(12, int(math.ceil(2 * math.pi * r / sp)))
        ang = np.linspace(0, 2 * math.pi, na, endpoint=False)
        zs = np.linspace(-hh, hh, max(2, int(math.ceil(2 * hh / sp)) + 1))
        A, Z = np.meshgrid(ang, zs, indexing="ij")
        side = np.stack([r * np.cos(A.ravel()), r * np.sin(A.ravel()), Z.ravel()], axis=1)
        caps = [np.array([[0, 0, s * hh]]) for s in (-1, 1)]
        for rr in np.linspace(sp, r, max(1, int(math.ceil(r / sp)))):
            n = max(8, int(math.ceil(2 * math.pi * rr / sp)))
            a = np.linspace(0, 2 * math.pi, n, endpoint=False)
            for s in (-1, 1):
                caps.append(np.stack([rr * np.cos(a), rr * np.sin(a), np.full(n, s * hh)], axis=1))
        out = np.concatenate([side] + caps)
    else:
        r = prim.size[0]
        n = max(60, int(4 * math.pi * r * r / sp ** 2))
        i = np.arange(n) + 0.5
        phi = np.arccos(1 - 2 * i / n)
        th = math.pi * (1 + 5 ** 0.5) * i
        out = r * np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)
    _SAMPLES[key] = out
    return out


def _aabb(prim: Prim, T: Pose):
    c = T.apply(prim.corners())
    return c.min(axis=0), c.max(axis=0)


def penetration(prims_a: Sequence[Prim], Ta: Pose, prims_b: Sequence[Prim], Tb: Pose) -> Tuple[float, Optional[list]]:
    """How deep body A's shapes and body B's shapes are in each other (m), and where (world)."""
    best, where = 0.0, None
    boxes_b = [(pb, *_aabb(pb, Tb)) for pb in prims_b]
    for pa in prims_a:
        lo_a, hi_a = _aabb(pa, Ta)
        wa = Ta @ pa.pose
        for pb, lo_b, hi_b in boxes_b:
            if np.any(lo_a > hi_b + 1e-4) or np.any(lo_b > hi_a + 1e-4):
                continue
            wb = Tb @ pb.pose
            for src, dst, wsrc, wdst in ((pa, pb, wa, wb), (pb, pa, wb, wa)):
                pw = wsrc.apply(surface_points(src))
                d = sdf(dst, wdst.inv().apply(pw))
                k = int(np.argmin(d))
                if -d[k] > best:
                    best, where = float(-d[k]), [round(float(v), 5) for v in pw[k]]
    return best, where


# ---- the parts: ONE adapter over design.py, bought_parts.py and robot.py -------------------------

@dataclass
class Fastener:
    """One fastened joint of a part. The screw's axis runs through `point` (robot frame, mm,
    on the head's bearing face) along `axis` (the shank's direction) for `length_mm` (the
    grip: what it clamps). `into` is the part that owns the mating hole or thread."""
    name: str
    size: str                          # "M1.6" | "M2" | "M3" | "D-shaft"
    point: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    tol_mm: float                      # how far the two hole axes may be apart for the screw to pass
    into: str
    length_mm: float = 3.0
    head: Optional[Prim] = None        # head, nut and the shank's tip past the nut join the robot
    nut: Optional[Prim] = None         # with the part (part frame)
    tip: Optional[Prim] = None
    holes: Dict[str, object] = field(default_factory=dict)   # part -> hole diameter mm, "thread" or None
    joint: Optional[dict] = None       # bought_parts.joints() row it came from
    check_hole: bool = True
    note: str = ""


@dataclass
class Part:
    name: str
    step: int                          # 0-based index into robot.STEPS
    mate: str                          # "ASSY", "STAND" or another part of the same step
    approach: Tuple[float, float, float]   # unit, robot frame: from the seat toward where it comes from
    prims: List[Prim]                  # collision, robot frame at the seat (metres)
    mass_g: float
    fasteners: List[Fastener] = field(default_factory=list)
    expect: List[Sequence[str]] = field(default_factory=list)   # each: DEFs of which it must touch one
    free_running: bool = False         # must touch nothing on the robot (a wheel)
    released: bool = True              # let go at the seat (gravity rests it); False = held
    meshes: Tuple[str, ...] = ()       # robot.assembly() mesh names (or prefixes) that belong to it
    visual: List[str] = field(default_factory=list)
    kit_rot: Pose = field(default_factory=Pose)
    wheel_side: Optional[str] = None
    joint_prims: List[Prim] = field(default_factory=list)   # collision once jointed (driving)
    design_checks: List[dict] = field(default_factory=list)
    hole_bands: List[tuple] = field(default_factory=list)   # (z0, z1, footprint, ox, oy) mm: where its holes are
    hole_const: Optional[float] = None  # a bought part's hole diameter, mm
    threaded: bool = False              # its holes are threads (a standoff)

    @property
    def def_(self) -> str:
        return "P_" + self.name.upper()

    def fastener_prims(self) -> List[Prim]:
        return [p for f in self.fasteners for p in (f.head, f.nut, f.tip) if p is not None]

    def com(self) -> np.ndarray:
        v = np.array([p.volume() for p in self.prims])
        c = np.array([p.pose.t for p in self.prims])
        return (c * v[:, None]).sum(axis=0) / v.sum()

    def bbox_corners(self) -> np.ndarray:
        c = np.concatenate([p.corners() for p in self.prims])
        lo, hi = c.min(axis=0), c.max(axis=0)
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])

    def inside(self, pts_mm: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts_mm, float) * MM
        d = np.min([sdf(q, q.pose.inv().apply(pts)) for q in self.prims], axis=0)
        return d < -1e-6

    def hole_at(self, x: float, y: float, z: float):
        """The diameter of this part's hole through (x, y) at height z (mm), 'thread', or None."""
        if self.threaded:
            return "thread"
        if self.hole_const is not None:
            return self.hole_const
        for z0, z1, poly, ox, oy in self.hole_bands:
            if not (z0 - 1e-6 <= z <= z1 + 1e-6):
                continue
            for g in (list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]):
                for ring in g.interiors:
                    hp = Polygon(ring)
                    if hp.contains(Point(x - ox, y - oy)):
                        return round(2 * math.sqrt(hp.area / math.pi), 3)
        return None


# ISO 4762 heads / ISO 4032 nuts, used when bought_parts is not there to say.
_SHCS = {"M1.6": {"head_d": 3.0, "head_k": 1.6, "pitch": 0.35, "d": 1.6},
         "M2": {"head_d": 3.8, "head_k": 2.0, "pitch": 0.4, "d": 2.0},
         "M3": {"head_d": 5.5, "head_k": 3.0, "pitch": 0.5, "d": 3.0}}
_NUT = {"M1.6": {"s": 3.2, "m": 1.3}, "M2": {"s": 4.0, "m": 1.6}, "M3": {"s": 5.5, "m": 2.4}}


def _bought():
    try:
        import bought_parts
        return bought_parts
    except Exception:   # noqa: BLE001
        return None


def _joint_rows(r: Robot) -> Tuple[List[dict], dict, dict, str]:
    """Every screwed joint: bought_parts.joints(r) when it exists, else the first design's
    (screws from above, nuts below, as the original ASSEMBLY.md has them)."""
    bp = _bought()
    if bp is not None and hasattr(bp, "joints"):
        return bp.joints(r), getattr(bp, "SHCS", _SHCS), getattr(bp, "NUT", _NUT), "bought_parts.joints(r)"
    pt, t = r.plate_top_z, r.plate_t
    rows = []

    def row(kind, step, at, head_z, d, size, length, grip, nut):
        rows.append({"joint": kind, "step": step, "at": at, "head_z": head_z, "dir": d, "size": size, "length": length,
                     "grip_mm": grip, "nut": nut, "proud_mm": (length - grip - _NUT[nut]["m"]) if nut else None,
                     "thread_in_mm": None if nut else length - grip})
    sx, sy = r.standoff_xy
    for i in (-1, 1):
        for j in (-1, 1):
            row("standoff_bottom", 4, (i * sx, j * sy), pt - t, 1, "M3", 6, t, None)
            row("standoff_top", 6, (i * sx, j * sy), pt + r.standoff_len + t, -1, "M3", 6, t, None)
            row("motor_clamp", 2, (r.axle_x + i * r.clamp_hole_spacing / 2, j * r.clamp_y), pt + r.clamp_tab_t, -1,
                "M2", 6, r.clamp_tab_t + t, "M2")
    for j in (-1, 1):
        row("caster", 1, (r.caster_x, j * BOUGHT["caster"]["hole_spacing_mm"] / 2), pt, -1, "M2", 6,
            t + BOUGHT["caster"]["spacer_mm"] + BOUGHT["caster"].get("flange_mm", 1.5), "M2")
        row("tray", 3, (r.tray_x, j * design.TRAY["hole_y"]), pt + design.TRAY["base"], -1, "M2", 6,
            design.TRAY["base"] + t, "M2")
    return rows, _SHCS, _NUT, "assemble.py fallback (bought_parts not importable)"


def _cyl_along(centre_xy, z0, z1, radius) -> Prim:
    lo, hi = min(z0, z1), max(z0, z1)
    return cyl_mm((centre_xy[0], centre_xy[1], (lo + hi) / 2), (0, 0, 1), radius, hi - lo)


def _attach_joints(r: Robot, parts: List[Part]) -> str:
    """Each screwed joint to the part it fastens (the part of the joint's step its shank
    passes through), with the part it fastens INTO (the latest earlier part it passes
    through), both parts' holes read off their geometry, and its head, nut and tip."""
    rows, shcs, nuts, source = _joint_rows(r)
    for n, jt in enumerate(rows):
        x, y = jt["at"]
        d = jt["dir"]
        z0 = jt["head_z"]
        zs = z0 + d * np.arange(0.25, jt["length"], 0.25)
        pts = np.stack([np.full(zs.size, x), np.full(zs.size, y), zs], axis=1)
        pierced = [p for p in parts if p.step <= jt["step"] and not p.wheel_side and p.inside(pts).any()]
        owners = [p for p in pierced if p.step == jt["step"]]
        befores = sorted([p for p in pierced if p.step < jt["step"]], key=lambda p: -p.step)
        if not owners or not befores:
            # nothing to fasten, or nothing to fasten it to: a design error, reported on the step
            host = next((p for p in parts if p.step == jt["step"]), parts[0])
            host.design_checks.append({"check": f"joint {jt['joint']} at ({x:g}, {y:g})", "pass": False,
                                       "note": f"its shank passes through {[p.name for p in pierced]}: "
                                               "no part of this step, or nothing earlier, to join"})
            continue
        owner, into = owners[0], befores[0]
        s = shcs[jt["size"]]
        grip_end = z0 + d * jt["grip_mm"]
        head = _cyl_along((x, y), z0, z0 - d * s["head_k"], s["head_d"] / 2)
        nut = tip = None
        if jt.get("nut"):
            m = nuts[jt["nut"]]["m"]
            nut = _cyl_along((x, y), grip_end, grip_end + d * m, nuts[jt["nut"]]["s"] / 2)
            tip_end = z0 + d * jt["length"]
            if (tip_end - (grip_end + d * m)) * d > 1e-6:
                tip = _cyl_along((x, y), grip_end + d * m, tip_end, s["d"] / 2)
        holes, tol, missing = {}, 0.0, []
        for p in (owner, into):
            zin = [float(z) for z, ins in zip(zs, p.inside(pts)) if ins]
            hd = p.hole_at(x, y, zin[len(zin) // 2]) if zin else None
            holes[p.name] = hd
            if hd is None:
                missing.append(p.name)
            elif hd != "thread":
                tol += max(0.0, (float(hd) - s["d"]) / 2)
        name = f"{jt['joint']}_{owner.name}_{n}"
        f = Fastener(name, jt["size"], (x, y, z0), (0, 0, d), tol, into.name, length_mm=jt["grip_mm"],
                     head=head, nut=nut, tip=tip, holes=holes, joint=jt)
        owner.fasteners.append(f)
        if missing:
            owner.design_checks.append({"check": f"{name}: a hole for the screw", "pass": False,
                                        "note": f"no hole at ({x:g}, {y:g}) in {missing}"})
        engage = jt["length"] - jt["grip_mm"]
        if jt.get("nut"):
            need = nuts[jt["nut"]]["m"] + s["pitch"]
            what = "through its nut, a pitch proud"
        else:
            need = 1.5 * s["d"]
            what = "into its thread, at least 1.5 d"
        owner.design_checks.append({"check": f"{name}: {jt['size']} x {jt['length']} {what}", "grip_mm": round(jt["grip_mm"], 3),
                                    "engages_mm": round(engage, 3), "needs_mm": round(need, 3), "pass": engage >= need - 1e-6})
    return source


def kit(r: Robot, mesh_url: str) -> List[Part]:
    """Every part of the robot in assembly order: shapes, seat, holes, fasteners, mates.

    The seat of every part is its mate's frame: each part's frame IS the robot frame when
    it is seated (that is how robot.py places them), so "on its seat" means "its pose
    relative to its mate is the identity". Shapes are envelopes from design.py's numbers
    and BOUGHT's datasheet sizes; screwed joints come from bought_parts.joints(r); the
    visuals from robot.assembly(). robot.py exposes none of the collision data itself.
    """
    m = BOUGHT["gearmotor"]
    pt, t = r.plate_top_z, r.plate_t
    ub = pt - t
    top0 = pt + r.standoff_len
    sx, sy = r.standoff_xy
    up, down = (0, 0, 1), (0, 0, -1)
    made = {p["part"]: p for p in design.made_parts(r)}
    parts: List[Part] = []

    # 1. the bottom plate, flat on the stand. Its frame defines the robot frame.
    parts.append(Part("bottom_plate", 0, "STAND", up, [box_mm((0, 0, pt - t / 2), (r.plate_x, r.plate_y, t))],
                      made["bottom_plate"]["g"], expect=[["RAIL_L"], ["RAIL_R"]], meshes=("bottom_plate",),
                      hole_bands=[(ub, pt, design.bottom_plate(r), 0.0, 0.0)]))

    # 2. the caster, under the plate (the plate is turned over for this step, so the caster
    #    comes down onto it): spacer on the plate, the flange under it, the housing, the ball.
    c = BOUGHT["caster"]
    sp = c["spacer_mm"]
    fl = c.get("flange_mm", 1.5)
    fy, fx = c["footprint_mm"]          # 19.1 along y (its two holes), 12.1 along x
    top_c = ub - sp                     # the flange's mounting face, on the spacer
    ball_r = 9.525 / 2
    ball_z = top_c - c["height_mm"] + ball_r
    housing_r = 4.6                     # Pololu's housing is notched to |y| 4.6 beside each hole
    parts.append(Part("caster", 1, "ASSY", down,
                      [box_mm((r.caster_x, 0, ub - sp / 2), (fx, fy, sp)),
                       box_mm((r.caster_x, 0, top_c - fl / 2), (fx, fy, fl)),
                       _cyl_along((r.caster_x, 0), top_c - fl, ball_z, housing_r),
                       sphere_mm((r.caster_x, 0, ball_z), ball_r, friction=0.0)],
                      c["g"], expect=[["ASSY"]], meshes=("caster",), hole_const=c["hole_d_mm"],
                      visual=[robot_mod._at((r.caster_x * MM, 0, ball_z * MM),
                                            "Shape { appearance PBRAppearance { baseColor .8 .8 .82 metalness 1 roughness .2 } "
                                            f"geometry Sphere {{ radius {robot_mod._f(ball_r * MM)} subdivision 3 }} }}")]))

    # 3. the motors, laid on the plate, then their clamps over the gearboxes.
    for side, j in (("L", 1), ("R", -1)):
        face = j * r.faceplate_y
        parts.append(Part(f"motor_{side}", 2, "ASSY", up,
                          [box_mm((r.axle_x, j * (r.faceplate_y - m["body_mm"][2] / 2), r.axle_z),
                                  (m["body_mm"][0], m["body_mm"][2], m["body_mm"][1])),
                           cyl_mm((r.axle_x, face + j * m["bushing_proud_mm"] / 2, r.axle_z), (0, 1, 0), 2.0, m["bushing_proud_mm"]),
                           cyl_mm((r.axle_x, j * (r.faceplate_y + m["shaft_len_mm"] / 2), r.axle_z), (0, 1, 0),
                                  m["shaft_d_mm"] / 2, m["shaft_len_mm"])],
                          m["g"], expect=[["ASSY"]], meshes=("gearmotor",)))
    for side, j in (("L", 1), ("R", -1)):
        bands = design.motor_clamp(r)
        parts.append(Part(f"clamp_{side}", 2, "ASSY", up, bands_to_boxes(bands, (r.axle_x, j * r.clamp_y, pt)),
                          made["motor_clamp"]["g"], expect=[["ASSY", f"P_MOTOR_{side}"]], meshes=("motor_clamp",),
                          hole_bands=[(pt + z0, pt + z1, poly, r.axle_x, j * r.clamp_y) for z0, z1, poly in bands]))

    # 4. the electronics tray, then the boards into its pockets.
    T = design.TRAY
    bands = design.tray(r)
    parts.append(Part("tray", 3, "ASSY", up, bands_to_boxes(bands, (r.tray_x, 0, pt)), made["tray"]["g"],
                      expect=[["ASSY"]], meshes=("tray",),
                      hole_bands=[(pt + z0, pt + z1, poly, r.tray_x, 0.0) for z0, z1, poly in bands]))
    for name, key, (x, _sx, _sy) in (("xiao", "controller", T["pockets"][0]), ("drv8833", "driver", T["pockets"][1])):
        board = BOUGHT[key]["board_mm"]
        parts.append(Part(name, 3, "tray", up, [box_mm((r.tray_x + x, 0, pt + T["base"] + 0.8), (board[0], board[1], 1.6))],
                          BOUGHT[key]["g"], expect=[["P_TRAY"]], meshes=(name,)))

    # 5. the standoffs.
    hex_r = 5.5 / 2 / math.cos(math.pi / 6)      # 5.5 mm across flats: its corners, the conservative envelope
    for i in (-1, 1):
        for j in (-1, 1):
            nm = f"standoff_{'F' if i > 0 else 'B'}{'L' if j > 0 else 'R'}"
            parts.append(Part(nm, 4, "ASSY", up, [cyl_mm((i * sx, j * sy, pt + r.standoff_len / 2), up, hex_r, r.standoff_len)],
                              BOUGHT["standoff"]["g"], expect=[["ASSY"]], meshes=("standoff",), threaded=True))

    # 6. the wheels: hub + rim + O-ring, assembled beforehand, pressed onto the D-shafts.
    for side, j in (("L", 1), ("R", -1)):
        ax = np.array([r.axle_x, 0.0, r.axle_z])
        hub_in = r.faceplate_y + r.hub_gap              # the hub's face toward the gearbox
        y_hub = (j * hub_in, j * (hub_in + r.hub_t))
        y_rim = (j * (hub_in + r.hub_t), j * (hub_in + r.hub_t + r.rim_width))
        tc = r.tyre_cs_mm
        prims = (ring_mm(ax, j, r.hub_bore_d / 2, r.hub_d / 2, *y_hub)
                 + ring_mm(ax, j, r.rim_bore_d / 2, r.rim_flange_d / 2, *y_rim)
                 + ring_mm(ax, j, r.groove_root_d / 2, r.wheel_d / 2, j * (r.wheel_y - tc / 2), j * (r.wheel_y + tc / 2), n=24))
        # The hub's three M2 screws go in from the hub's free face (its holes are clear, the
        # rim's are 1.7 mm to cut a thread), so their heads face the gearbox.
        s = _SHCS["M2"]
        heads = []
        for a in (30.0, 150.0, 270.0):
            u = math.radians(a)
            # the hub's own (x, y) becomes the robot's (x, -/+z): robot.py turns it about x
            hx, hz = r.axle_x + r.hub_screw_pcd / 2 * math.cos(u), r.axle_z - j * r.hub_screw_pcd / 2 * math.sin(u)
            heads.append(cyl_mm((hx, j * (hub_in - s["head_k"] / 2), hz), (0, 1, 0), s["head_d"] / 2, s["head_k"]))
        hub_len = float(re.search(r"M2 x (\d+)", BOUGHT["screw_m2_hub"]["spec"]).group(1))
        bore = Fastener(f"wheel_{side}_D_bore", "D-shaft", (r.axle_x, j * hub_in, r.axle_z), (0, j, 0),
                        (r.hub_bore_d - m["shaft_d_mm"]) / 2, f"motor_{side}", length_mm=r.hub_t,
                        note="radial clearance of the D-bore on the shaft")
        tyre = Prim("cyl", Pose(_frame_z((0, 1, 0)), np.array([r.axle_x, j * r.wheel_y, r.axle_z]) * MM),
                    (r.wheel_d / 2 * MM, tc * MM))
        checks = [{"check": f"wheel_{side} hub screws: M2 x {hub_len:g} into the rim's 1.7 mm holes, at least 2 d",
                   "grip_mm": r.hub_t, "engages_mm": hub_len - r.hub_t, "needs_mm": 4.0, "pass": hub_len - r.hub_t >= 4.0},
                  {"check": f"wheel_{side} hub screw heads clear of the gearbox face", "head_k_mm": s["head_k"],
                   "hub_gap_mm": r.hub_gap, "pass": s["head_k"] <= r.hub_gap,
                   "note": "heads at PCD 11 on the hub's gearbox side; the press-on run below measures it"}]
        parts.append(Part(f"wheel_{side}", 5, f"motor_{side}", (0, j, 0), prims + heads,
                          robot_mod.wheel_mass_g(r), [bore], expect=[], free_running=True, released=False,
                          kit_rot=Pose.aa((1, 0, 0), math.pi / 2), wheel_side="left" if j > 0 else "right",
                          joint_prims=[tyre], design_checks=checks))

    # 7. the top plate on the standoffs, then the battery holder on it.
    parts.append(Part("top_plate", 6, "ASSY", up, [box_mm((0, 0, top0 + t / 2), (r.plate_x, r.plate_y, t))],
                      made["top_plate"]["g"], expect=[["ASSY"]], meshes=("top_plate", "engraving"),
                      hole_bands=[(top0, top0 + t, design.top_plate(r), 0.0, 0.0)]))
    bh = BOUGHT["battery_holder"]["outline_mm"]
    parts.append(Part("battery", 6, "top_plate", up, [box_mm((0, 0, top0 + t + bh[2] / 2), bh)],
                      BOUGHT["battery_holder"]["g"] + BOUGHT["cells"]["g"] * BOUGHT["cells"]["qty"] + BOUGHT["strap"]["g"],
                      expect=[["P_TOP_PLATE"]], meshes=("battery",)))

    _attach_joints(r, parts)
    # screws and nuts: their mass goes with the part they fasten
    sg = {"M1.6": 0.25, "M2": 0.4, "M3": 0.6}
    for p in parts:
        p.mass_g += sum(sg.get(f.size, 0.0) for f in p.fasteners if f.joint)
    _attach_visuals(r, parts, mesh_url)
    slot_inner = 33.5 - 3.0 / 2
    for p in parts:
        if p.name == "battery":
            p.design_checks.append({"check": "battery holder clear of the strap slots", "holder_half_x_mm": bh[0] / 2,
                                    "slot_inner_edge_x_mm": slot_inner, "clear_mm": round(slot_inner - bh[0] / 2, 3),
                                    "pass": slot_inner - bh[0] / 2 >= 0})
    return parts


def _attach_visuals(r: Robot, parts: List[Part], mesh_url: str) -> None:
    """robot.assembly()'s visuals, each to its part: by mesh name (or a mesh named
    `<model>_<k>`), else the part of that step whose shapes are nearest the visual's position."""
    chassis, wheels = robot_mod.assembly(r, mesh_url)
    for step, vrml in chassis:
        cands = [p for p in parts if p.step == step]
        names = re.findall(r'url "[^"]*?([A-Za-z0-9_.]+)\.(?:stl|obj|dae)"', vrml)
        tr = re.match(r"\s*Pose \{ translation ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)", vrml)
        at = np.array([float(v) for v in tr.groups()]) if tr else None
        named = [p for p in cands if any(nm == k or nm.startswith(k + "_") for nm in names for k in p.meshes)]
        pool = named or cands
        if not pool:
            continue
        if at is None or len(pool) == 1:
            pool[0].visual.append(vrml)
            continue

        def dist(p: Part) -> float:
            if named:
                return float(np.linalg.norm(p.com() - at))
            return float(min(sdf(q, q.pose.inv().apply(at[None]))[0] for q in p.prims + p.fastener_prims()))
        min(pool, key=dist).visual.append(vrml)
    for step, side, vis in wheels:
        for p in parts:
            if p.wheel_side == side:
                wy = (1 if side == "left" else -1) * r.wheel_y
                p.visual.append(robot_mod._at((r.axle_x * MM, wy * MM, r.axle_z * MM), vis))


# ---- worlds -----------------------------------------------------------------------------------

_f, _v = robot_mod._f, robot_mod._v


def _pose_fields(T: Pose) -> str:
    x, y, z, a = T.axis_angle()
    return f"translation {_v(*T.t)} rotation {_v(x, y, z, a)}"


def prim_vrml(p: Prim) -> str:
    if p.kind == "box":
        g = f"Box {{ size {_v(*p.size)} }}"
    elif p.kind == "cyl":
        g = f"Cylinder {{ radius {_f(p.size[0])} height {_f(p.size[1])} subdivision 24 }}"
    else:
        g = f"Sphere {{ radius {_f(p.size[0])} subdivision 2 }}"
    return f"Pose {{ {_pose_fields(p.pose)} children [ {g} ] }}"


def _collision(prims: Sequence[Prim]) -> Tuple[str, List[str]]:
    """boundingObject Group + nested no-physics Solids for shapes with their own friction
    (the caster ball is frictionless, as in robot.py: a ball caster rolls)."""
    plain = [p for p in prims if p.friction is None]
    special = [p for p in prims if p.friction is not None]
    bo = "Group { children [ " + " ".join(prim_vrml(p) for p in plain) + " ] }" if plain else ""
    kids = [f'Solid {{ name "frictionless_{i}" newtonFriction {_f(p.friction)} boundingObject {prim_vrml(p)} }}'
            for i, p in enumerate(special)]
    return bo, kids


# Every body's origin is put at its centre of mass, with its inertia given explicitly: a part
# whose frame is the robot frame has its origin up to 45 mm from its material, and a body so
# built tumbled off a flat seat it should have rested on (the caster, the motors, the
# standoffs: measured in this file's first runs), while the parts whose origin was near their
# material sat still. COM_OFFSET[def] is that centre in the body's own (robot) frame, metres;
# Harness.pose/set_pose convert, so everything else works in part frames.
COM_OFFSET: Dict[str, np.ndarray] = {}


def _prim_inertia(p: Prim, mass: float) -> np.ndarray:
    if p.kind == "box":
        a, b, c = p.size
        I = mass / 12 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
    elif p.kind == "cyl":
        r, h = p.size
        I = np.diag([mass * (3 * r * r + h * h) / 12] * 2 + [mass * r * r / 2])
    else:
        I = np.eye(3) * 0.4 * mass * p.size[0] ** 2
    return p.pose.R @ I @ p.pose.R.T


def _physics(groups: Sequence[Tuple[float, Sequence[Prim]]]) -> Tuple[np.ndarray, str]:
    """(centre of mass, Physics node about it) for [(grams, shapes)]: each group's mass spread
    over its shapes by volume (overlapping shapes count twice: an envelope, not a solid)."""
    items = []
    for g, prims in groups:
        vol = sum(q.volume() for q in prims)
        items += [(g / 1000.0 * q.volume() / vol, q) for q in prims]
    M = sum(m for m, _ in items)
    com = sum(m * q.pose.t for m, q in items) / M
    I = np.zeros((3, 3))
    for m, q in items:
        d = q.pose.t - com
        I += _prim_inertia(q, m) + m * (d @ d * np.eye(3) - np.outer(d, d))
    phys = (f"physics Physics {{ density -1 mass {_f(M)} centerOfMass [ 0 0 0 ] "
            f"inertiaMatrix [ {I[0, 0]:.6g} {I[1, 1]:.6g} {I[2, 2]:.6g} {I[0, 1]:.6g} {I[0, 2]:.6g} {I[1, 2]:.6g} ] }}")
    return com, phys


@dataclass
class Weld:
    part: Part
    W: Pose                     # the part's frame in the robot frame, as physics left it


def _welded_prims(welds: Sequence[Weld], jointed: bool = False) -> List[Prim]:
    out = []
    for w in welds:
        if jointed and w.part.wheel_side:
            continue
        out += [q.moved(w.W) for q in w.part.prims + w.part.fastener_prims()]
    return out


def _vis(T: Pose, visual: Sequence[str]) -> str:
    return f"Pose {{ {_pose_fields(T)} children [ {' '.join(visual)} ] }}"


def assy_node(welds: Sequence[Weld], T: Pose, robot: bool = False, controller: str = "<none>",
              args: Sequence[str] = ()) -> str:
    """The robot as built so far: ONE body (a Solid), or with the wheels on hinges (a Robot)."""
    chassis = [w for w in welds if not (robot and w.part.wheel_side)]
    com, phys = _physics([(w.part.mass_g, [q.moved(w.W) for q in w.part.prims]) for w in chassis])
    COM_OFFSET["ASSY"] = com
    C = Pose(None, -com)
    kids = [_vis(C @ w.W, w.part.visual) for w in chassis]
    bo, special = _collision([q.moved(C) for q in _welded_prims(chassis)])
    kids += special
    T = T @ Pose(None, com)
    if not robot:
        return (f'DEF ASSY Solid {{ name "assembly" {_pose_fields(T)}\n  children [\n    ' + "\n    ".join(kids)
                + f" ]\n  boundingObject {bo}\n  {phys} }}")
    m = BOUGHT["gearmotor"]
    motors = {w.part.name: w for w in welds if w.part.name.startswith("motor_")}
    for w in welds:
        if not w.part.wheel_side:
            continue
        side = w.part.wheel_side
        mw = motors["motor_L" if side == "left" else "motor_R"]
        r = Robot()
        j = 1 if side == "left" else -1
        axle_design = np.array([r.axle_x, j * r.wheel_y, r.axle_z]) * MM
        anchor = (C @ mw.W).apply(axle_design)           # on the shaft, as the motor was welded
        axis = mw.W.R @ np.array([0.0, 1.0, 0.0])
        E = C @ w.W @ Pose(None, axle_design)            # the wheel's centre, as it was pressed on
        jp = w.part.joint_prims[0]
        local = Pose(None, -axle_design)
        kids.append(
            f"HingeJoint {{ jointParameters HingeJointParameters {{ axis {_v(*axis)} anchor {_v(*anchor)} }}\n"
            f'  device [ RotationalMotor {{ name "{side}_motor" maxVelocity {_f(NO_LOAD_RAD_S)} '
            f'maxTorque {_f(m["stall_kgcm"] * 9.81 / 100)} }} PositionSensor {{ name "{side}_sensor" }} ]\n'
            f'  endPoint DEF WHEEL_{side[0].upper()} Solid {{ name "{side}_wheel" {_pose_fields(E)}\n'
            f"    children [ {_vis(local, w.part.visual)} ]\n"
            f"    boundingObject {prim_vrml(jp.moved(local))}\n"
            f"    physics Physics {{ density -1 mass {_f(w.part.mass_g / 1000.0)} }} }} }}")
    arg_list = " ".join(f'"{a}"' for a in args)
    return (f'DEF ASSY Robot {{ name "desk-robot" controller "{controller}" controllerArgs [ {arg_list} ] supervisor TRUE\n'
            f"  {_pose_fields(T)}\n  children [\n    " + "\n    ".join(kids) + " ]\n"
            f"  boundingObject {bo}\n  {phys} }}")


def part_node(p: Part, T: Pose) -> str:
    com, phys = _physics([(p.mass_g, p.prims)])
    COM_OFFSET[p.def_] = com
    C = Pose(None, -com)
    bo, special = _collision([q.moved(C) for q in p.prims])
    kids = [_vis(C, p.visual)] + special
    # A held part (pressed on, not let go) is held against gravity: the builder's hand. Without
    # it a set_pose's settle step (and the 0-4 basic steps the engine overshoots it by) drops
    # a wheel onto its shaft, further than the D-bore's 0.05 mm clearance.
    hold = "" if p.released else " newtonGravityCompensation 1"
    return (f'DEF {p.def_} Solid {{ name "{p.name}"{hold} {_pose_fields(T @ Pose(None, com))}\n  children [ '
            + " ".join(kids) + " ]\n"
            f"  boundingObject {bo}\n  {phys} }}")


RAILS = {"RAIL_L": ((5.0, 23.5), (54.0, 13.0)), "RAIL_R": ((5.0, -23.5), (54.0, 13.0))}
# Two rails under the plate (x -22..32, |y| 17..30): clear of the caster (x <= -27), the
# tray's nuts (|y| <= 15.5), the clamps' nuts and the standoffs' screw heads (|y| >= 31.25).


def rail_prims() -> Dict[str, Tuple[Pose, List[Prim]]]:
    return {k: (Pose(), [box_mm((c[0], c[1], STAND_H_MM / 2), (s[0], s[1], STAND_H_MM))]) for k, (c, s) in RAILS.items()}


def bench_prims() -> Tuple[Pose, List[Prim]]:
    return Pose(), [box_mm((0, 0, -10), (1400, 1000, 20))]


def world_text(title: str, nodes: Sequence[str], rails: bool, eye=(0.17, -0.20, 0.15), target=(0.0, 0.0, 0.025),
               contact_ke: Optional[float] = None) -> str:
    sys.path.insert(0, str(design.MANUFACTURING / "core"))
    from worldkit import look_at
    L = ["#OMNISIM R2025a utf8",
         'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
         'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
         'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
         "# Written by projects/manufacturing/products/desk_robot/assemble.py; deleted after the run.",
         f'WorldInfo {{ basicTimeStep {DT_MS} title "{title}" newtonSolver "mujoco" newtonSubsteps 2 FPS 30'
         + (f" newtonContactKe {_f(contact_ke)}" if contact_ke else "") + " }",
         f"Viewpoint {{ position {_v(*eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
         "OmniSimSky { }", "DEF SUN OmniSimSun { }", "DEF SUN_MARKER OmniSimSunMarker { }",
         'DEF BENCH Solid { name "bench" translation 0 0 -0.01 children [ Shape { appearance PBRAppearance '
         '{ baseColor .9 .9 .88 roughness .8 metalness 0 } geometry Box { size 1.4 1 .02 } } ] '
         'boundingObject Box { size 1.4 1 .02 } }']
    if rails:
        for k, (c, s) in RAILS.items():
            L.append(f'DEF {k} Solid {{ name "{k.lower()}" translation {_v(c[0] * MM, c[1] * MM, STAND_H_MM / 2 * MM)} '
                     f'children [ Shape {{ appearance PBRAppearance {{ baseColor .35 .37 .4 roughness .6 metalness .3 }} '
                     f'geometry DEF {k}_BOX Box {{ size {_v(s[0] * MM, s[1] * MM, STAND_H_MM * MM)} }} }} ] '
                     f'boundingObject USE {k}_BOX }}')
    return "\n".join(L + list(nodes) + [""])


# ---- the harness -------------------------------------------------------------------------------

class Harness:
    """Our own harness on our own ports (headless). Stopped, with its engine tree, by stop()."""

    def __init__(self, port: int, log_path: Path):
        self.port = port
        self.base = f"http://127.0.0.1:{port}"
        self.log_path = log_path
        self.proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w", encoding="utf-8")
        self.proc = subprocess.Popen([sys.executable, "-m", "omnisim", "harness", "--port", str(self.port),
                                      "--supervisor-port", str(self.port + 1)], cwd=ROOT, stdout=self._log,
                                     stderr=subprocess.STDOUT)
        for _ in range(180):
            try:
                self.call("/healthz", timeout=5)
                return
            except Exception:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"the harness exited ({self.proc.returncode}); see {self.log_path}")
                time.sleep(1.0)
        raise RuntimeError("the harness did not come up in 180 s")

    def call(self, path: str, body=None, timeout: float = 300, raw: bool = False):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, headers={"Content-Type": "application/json"},
                                     method="POST" if body is not None else "GET")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                b = resp.read()
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"{path}: HTTP {e.code}: {e.read()[:800]!r}") from None
        return b if raw else json.loads(b or b"{}")

    def stop(self) -> None:
        if self.proc is None:
            return
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(self.proc.pid)], capture_output=True)
        else:
            self.proc.terminate()
        try:
            self.proc.wait(timeout=20)
        except Exception:
            pass
        self._log.close()
        self.proc = None

    # -- the verbs this file uses
    def load(self, world: Path) -> dict:
        d = self.call("/world/load", {"path": world.relative_to(ROOT).as_posix(), "light": True}, timeout=600)
        if d.get("ok") is False:
            raise RuntimeError(f"world load failed: {json.dumps(d)[:2000]}")
        return d

    def pause(self) -> None:
        self.call("/sim/pause", {"lease_ms": 300000})

    def step(self, n: int) -> dict:
        self.pause()
        return self.call("/sim/step", {"steps": int(n)})

    def pose(self, def_: str) -> Pose:
        """The body's FRAME (the robot frame for the assembly, the part frame for a part)."""
        d = self.call(f"/scene/node/{def_}")
        T = Pose(np.asarray(d["orientation"], float).reshape(3, 3), d["position"])
        return T @ Pose(None, -COM_OFFSET.get(def_, np.zeros(3)))

    def set_pose(self, def_: str, T: Pose, settle: int = 1) -> dict:
        T = T @ Pose(None, COM_OFFSET.get(def_, np.zeros(3)))
        x, y, z, a = T.axis_angle()
        return self.call("/scene/set_pose", {"def": def_, "translation": [float(v) for v in T.t],
                                             "rotation": [x, y, z, a], "reset_physics": True, "settle_steps": settle})

    def contacts(self) -> List[dict]:
        return self.call("/sim/contacts").get("contacts", [])

    def screenshot(self, path: Path) -> None:
        from PIL import Image
        png = self.call("/world/screenshot", {}, raw=True)
        im = Image.open(io.BytesIO(png)).convert("RGB")
        im.thumbnail((1200, 1200))
        path.parent.mkdir(parents=True, exist_ok=True)
        im.save(path, optimize=True)


def _touching(cs: Sequence[dict], def_: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for c in cs:
        a, b = c.get("a_def"), c.get("b_def")
        if a == def_ or b == def_:
            o = (b or f"name:{c.get('b_name')}") if a == def_ else (a or f"name:{c.get('a_name')}")
            out[o] = out.get(o, 0) + 1
    return out


def _points_with(cs: Sequence[dict], def_: str) -> List[list]:
    return [[round(v, 5) for v in c.get("point", [])] for c in cs if def_ in (c.get("a_def"), c.get("b_def"))][:6]


def _max_disp_mm(T0: Pose, T1: Pose, corners_m: np.ndarray) -> float:
    return float(np.max(np.linalg.norm(T1.apply(corners_m) - T0.apply(corners_m), axis=1)) / MM)


# ---- one step -------------------------------------------------------------------------------------

class Scene:
    """What is in the current world, with the poses last read from the engine."""

    def __init__(self, h: Harness, welds: Sequence[Weld], parts: Sequence[Part], rails: bool, robot: bool = False):
        self.h, self.welds, self.parts, self.rails, self.robot = h, list(welds), list(parts), rails, robot
        self.assy_prims = _welded_prims(self.welds, jointed=robot)

    def bodies(self, skip: Sequence[str] = ()) -> Dict[str, Tuple[Pose, List[Prim]]]:
        out = {"BENCH": bench_prims()}
        if self.rails:
            out.update(rail_prims())
        if self.welds:
            out["ASSY"] = (self.h.pose("ASSY"), self.assy_prims)
        for p in self.parts:
            out[p.def_] = (self.h.pose(p.def_), p.prims)
        return {k: v for k, v in out.items() if k not in skip}


def _seat_pose(h: Harness, p: Part, stand_seat: Pose, welds: Optional[Dict[str, "Weld"]] = None) -> Pose:
    """The part's seat: its mate's frame. A mate already welded into the robot is found
    where it was welded (a wheel's seat is its shaft, wherever physics left the motor)."""
    if p.mate == "STAND":
        return stand_seat
    if p.mate == "ASSY":
        return h.pose("ASSY")
    if welds and p.mate in welds:
        return h.pose("ASSY") @ welds[p.mate].W
    return h.pose("P_" + p.mate.upper())


def _into_world(h: Harness, f: Fastener, welds: Dict[str, Weld], step_parts: Dict[str, Part]) -> Pose:
    if f.into in step_parts:
        return h.pose(step_parts[f.into].def_)
    return h.pose("ASSY") @ welds[f.into].W


def place_part(sc: Scene, p: Part, stand_seat: Pose, welds: Dict[str, Weld], step_parts: Dict[str, Part]) -> dict:
    h = sc.h
    seat = _seat_pose(h, p, stand_seat, welds)
    a = seat.R @ np.asarray(p.approach, float)

    def at(off_mm: float) -> Dict[str, int]:
        h.set_pose(p.def_, Pose(seat.R, seat.t + a * off_mm * MM))
        return _touching(h.contacts(), p.def_)

    n = int(round(APPROACH_MM / INCREMENT_MM))
    blocked_at, blocker, first_points = None, {}, []
    for i in range(n + 1):
        off = APPROACH_MM - i * INCREMENT_MM
        touch = at(off)
        if off > 1e-9 and touch:
            o = off + INCREMENT_MM
            at(o)
            while True:
                o2 = round(o - REFINE_MM, 6)
                touch = at(o2)
                if touch or o2 <= 1e-9:
                    break
                o = o2
            if touch and o2 > 1e-9:
                blocked_at, blocker = o2, touch
                first_points = _points_with(h.contacts(), p.def_)
            break
    stop_off = blocked_at or 0.0
    # What the part WOULD overlap if it were forced onto its seat (geometry at the measured
    # poses of everything else): the interference, whether or not physics let it get there.
    others = sc.bodies(skip=[p.def_])
    interference, inter_where, inter_with = 0.0, None, None
    for k, (Tb, pb) in others.items():
        d, w = penetration(p.prims, seat, pb, Tb)
        if d > interference:
            interference, inter_where, inter_with = d, w, k
    if p.released:
        h.step(int(SETTLE_S * 1000 / DT_MS))
    T = h.pose(p.def_)
    seat = _seat_pose(h, p, stand_seat, welds)          # the mate may have settled too
    rel = seat.inv() @ T
    corners = p.bbox_corners()
    seat_err = _max_disp_mm(Pose(), rel, corners)
    axial = float((rel.t @ np.asarray(p.approach, float)) / MM)
    holes = []
    for f in p.fasteners:
        if not f.check_hole:
            continue
        Ti = _into_world(h, f, welds, step_parts)
        pa = np.asarray(f.point, float) * MM
        ax = np.asarray(f.axis, float)
        ends = np.array([pa, pa + ax * f.length_mm * MM])
        d = T.apply(ends) - Ti.apply(ends)
        axw = Ti.R @ ax
        lat = d - np.outer(d @ axw, axw)
        mis = float(np.max(np.linalg.norm(lat, axis=1)) / MM)
        holes.append({"fastener": f.name, "into": f.into, "misalignment_mm": round(mis, 4), "allowed_mm": round(f.tol_mm, 3),
                      "pass": mis <= f.tol_mm})
    # penetration as the part rests, and its screws' heads and nuts (they go in now)
    bodies = sc.bodies(skip=[p.def_])
    pen, pen_where, pen_with = 0.0, None, None
    for k, (Tb, pb) in bodies.items():
        d, w = penetration(p.prims, T, pb, Tb)
        if d > pen:
            pen, pen_where, pen_with = d, w, k
    fast_checks = []
    for f in p.fasteners:
        for what, prim in (("head", f.head), ("nut", f.nut), ("tip", f.tip)):
            if prim is None:
                continue
            worst, wwith, wwhere = 0.0, None, None
            for k, (Tb, pb) in bodies.items():
                d, w = penetration([prim], T, pb, Tb)
                if d > worst:
                    worst, wwith, wwhere = d, k, w
            fast_checks.append({"check": f"{f.name} {what} clear", "max_overlap_mm": round(worst / MM, 3),
                                "with": wwith, "at": wwhere, "pass": worst / MM <= PEN_TOL_MM})
    touch: Dict[str, int] = {}
    for i in range(CONTACT_POLL if p.released else 1):
        if i:
            h.step(1)
        for k2, v2 in _touching(h.contacts(), p.def_).items():
            touch[k2] = touch.get(k2, 0) + v2
    reasons = []
    if blocked_at and blocked_at > SEAT_TOL_MM:
        reasons.append(f"stopped {blocked_at:.2f} mm short of its seat: touched {sorted(blocker)} first, at {first_points}")
    if seat_err > SEAT_TOL_MM:
        reasons.append(f"rests {seat_err:.3f} mm from its seat (allowed {SEAT_TOL_MM})")
    for hl in holes:
        if not hl["pass"]:
            reasons.append(f"{hl['fastener']}: holes {hl['misalignment_mm']:.3f} mm apart (allowed {hl['allowed_mm']})")
    if pen / MM > PEN_TOL_MM:
        reasons.append(f"{pen / MM:.3f} mm into {pen_with} (allowed {PEN_TOL_MM})")
    for group in p.expect:
        if not any(e in touch for e in group):
            reasons.append(f"not touching {' or '.join(group)} (touches {sorted(touch)})")
    if p.free_running:
        bad = {k: v for k, v in touch.items() if k == "ASSY" or k.startswith("P_")}
        if bad:
            reasons.append(f"must run free but touches {sorted(bad)}")
    for fc in fast_checks:
        if not fc["pass"]:
            reasons.append(f"{fc['check']}: overlaps {fc['with']} by {fc['max_overlap_mm']} mm")
    for dc in p.design_checks:
        if dc.get("pass") is False:
            reasons.append(f"{dc['check']}: engages {dc.get('engages_mm')} mm, needs {dc.get('needs_mm')}"
                           if "engages_mm" in dc else f"{dc['check']}: {dc.get('note', 'fails')}")
    return {"part": p.name, "def": p.def_, "mate": p.mate, "approach_dir": list(p.approach),
            "approach_mm": APPROACH_MM, "released": p.released,
            "stopped_short_mm": round(stop_off, 3), "blocked_by": sorted(blocker), "first_contact_points": first_points,
            "interference_at_seat_mm": round(interference / MM, 3), "interference_with": inter_with,
            "interference_at": inter_where,
            "seat_error_mm": round(seat_err, 4), "axial_error_mm": round(axial, 4),
            "hole_misalignment_mm": round(max([hl["misalignment_mm"] for hl in holes], default=0.0), 4), "holes": holes,
            "max_penetration_mm": round(pen / MM, 4), "penetration_with": pen_with, "penetration_at": pen_where,
            "contacts": touch, "expected_contacts": p.expect, "fastener_clearance": fast_checks,
            "design_checks": p.design_checks, "pass": not reasons, "reasons": reasons,
            "_rel_to_assy": (h.pose("ASSY").inv() @ T) if sc.welds else None, "_T": T}


def _kit_pose(p: Part, i: int) -> Pose:
    R = p.kit_rot.R
    c = np.concatenate([q.corners() for q in p.prims])
    w = c @ R.T
    centre = (w.min(axis=0) + w.max(axis=0)) / 2
    x = -0.10 + 0.055 * i
    return Pose(R, np.array([x - centre[0], 0.13 - centre[1], 0.001 - w.min(axis=0)[2]]))


def upright(ub_mm: float, x: float = 0.0, lift_mm: float = 0.3) -> Pose:
    return Pose(None, (x, 0.0, (STAND_H_MM - ub_mm + lift_mm) * MM))


def flipped(pt_mm: float, lift_mm: float = 0.3) -> Pose:
    return Pose.aa((1, 0, 0), math.pi, (0.0, 0.0, (STAND_H_MM + pt_mm + lift_mm) * MM))


def _write(world: Path, text: str, written: List[Path]) -> None:
    world.write_text(text, encoding="utf-8", newline="\n")
    written.append(world)


def run_assembly_step(h: Harness, r: Robot, k: int, parts: List[Part], welds: List[Weld], written: List[Path]) -> Tuple[dict, List[Weld]]:
    t0 = time.time()
    title = robot_mod.STEPS[k]
    step_parts = [p for p in parts if p.step == k]
    ub, pt = r.plate_top_z - r.plate_t, r.plate_top_z
    jointed = any(w.part.wheel_side for w in welds)
    assy_T = flipped(pt) if k == 1 else upright(ub)
    nodes = []
    if welds:
        nodes.append(assy_node(welds, assy_T, robot=jointed))
    for i, p in enumerate(step_parts):
        nodes.append(part_node(p, _kit_pose(p, i)))
    world = WORLDS / f".desk_robot_assemble_{k + 1}.omniworld"
    _write(world, world_text(f"desk robot, physical assembly step {k + 1}", nodes, rails=True,
                             contact_ke=ASSEMBLY_CONTACT_KE), written)
    h.load(world)
    h.pause()
    h.step(150)
    sc = Scene(h, welds, step_parts, rails=True, robot=jointed)
    wmap = {w.part.name: w for w in welds}
    smap = {p.name: p for p in step_parts}
    stand_seat = upright(ub, lift_mm=0.0)
    results = []
    for p in step_parts:
        res = place_part(sc, p, stand_seat, wmap, smap)
        results.append(res)
        print(f"  step {k + 1} {p.name}: seat {res['seat_error_mm']:.3f} mm, holes {res['hole_misalignment_mm']:.3f} mm, "
              f"pen {res['max_penetration_mm']:.3f} mm, contacts {res['contacts']}"
              + ("" if res["pass"] else f"  FAIL: {'; '.join(res['reasons'])}"), flush=True)
    # the sub-assembly on the stand, its new parts on it: nothing may move. A held part is in
    # the builder's hand, not on a seat: it is welded where it was measured and not watched.
    held = {p.name: res for p, res in zip(step_parts, results) if not p.released}
    watch = (["ASSY"] if welds else []) + [p.def_ for p in step_parts if p.name not in held]
    corners = {d: (np.concatenate([q.corners() for q in sc.assy_prims]) if d == "ASSY"
                   else next(p for p in step_parts if p.def_ == d).bbox_corners()) for d in watch}
    P0 = {d: h.pose(d) for d in watch}
    h.step(int(SETTLE_S * 1000 / DT_MS))
    P1 = {d: h.pose(d) for d in watch}
    drift = {d: round(_max_disp_mm(P0[d], P1[d], corners[d]), 4) for d in watch}
    TA_now = h.pose("ASSY") if welds else None
    for p in step_parts:
        if p.name in held:
            h.set_pose(p.def_, TA_now @ held[p.name]["_rel_to_assy"])
    pairs = {}
    for c in h.contacts():
        key = " / ".join(sorted([str(c.get("a_def")), str(c.get("b_def"))]))
        pairs[key] = pairs.get(key, 0) + 1
    shot = OUT / f"physical_step_{k + 1}.png"
    h.screenshot(shot)
    # weld: every part joins the robot's body where physics left it
    if welds:
        TA = h.pose("ASSY")
    else:
        TA = h.pose(step_parts[0].def_)          # the bottom plate defines the robot frame
    new = [Weld(p, held[p.name]["_rel_to_assy"] if p.name in held else TA.inv() @ P1[p.def_]) for p in step_parts]
    reasons = [f"{res['part']}: {x}" for res in results for x in res["reasons"]]
    for d, v in drift.items():
        if v > DRIFT_TOL_MM:
            reasons.append(f"{d} moved {v:.4f} mm in {SETTLE_S} s after settling (allowed {DRIFT_TOL_MM})")
    for res in results:
        res.pop("_T", None)
        res.pop("_rel_to_assy", None)
    for w in new:
        for res in results:
            if res["part"] == w.part.name:
                x, y, z, a = w.W.axis_angle()
                res["weld_in_robot_frame"] = {"translation_mm": [round(v / MM, 4) for v in w.W.t],
                                              "rotation_deg": round(math.degrees(a), 4)}
    entry = {"step": k + 1, "title": title, "orientation": "turned over" if k == 1 else "upright on the stand",
             "parts_added": [p.name for p in step_parts], "parts": results, "settle_drift_mm": drift,
             "max_drift_mm": max(drift.values(), default=0.0), "contact_pairs": pairs,
             "screenshot": shot.name, "pass": not reasons, "reasons": reasons}
    entry["wall_s"] = round(time.time() - t0, 1)
    return entry, welds + new


def _wait_json(h: Harness, path: Path, timeout: float, poll_contacts: Sequence[str] = ()) -> Tuple[Optional[dict], Dict[str, int]]:
    t0 = time.time()
    seen: Dict[str, int] = {}
    while time.time() - t0 < timeout:
        if poll_contacts:
            for c in h.contacts():
                for d in poll_contacts:
                    if d in (c.get("a_def"), c.get("b_def")):
                        o = c.get("b_def") if c.get("a_def") == d else c.get("a_def")
                        key = f"{d} / {o}"
                        seen[key] = seen.get(key, 0) + 1
        if path.exists():
            try:
                return json.loads(path.read_text(encoding="utf-8")), seen
            except Exception:
                pass
        time.sleep(0.25)
    return None, seen


def run_spin_check(h: Harness, r: Robot, welds: List[Weld], written: List[Path]) -> dict:
    """The wheels on their hinges, the robot on the stand: the motors run them in the air."""
    t0 = time.time()
    res_path = TMP / "spin.json"
    res_path.unlink(missing_ok=True)
    ub = r.plate_top_z - r.plate_t
    world = WORLDS / ".desk_robot_assemble_6_spin.omniworld"
    _write(world, world_text("desk robot, wheels spun on the stand", [
        assy_node(welds, upright(ub), robot=True, controller="desk_robot_drive", args=[f"--result={res_path.as_posix()}"])],
        rails=True), written)
    h.load(world)
    T0 = h.pose("ASSY")
    d, seen = _wait_json(h, res_path, 180, poll_contacts=("WHEEL_L", "WHEEL_R"))
    T1 = h.pose("ASSY")
    out = {"world": "the robot so far with its wheels on hinges, on the stand; the drive controller runs "
                    "them 1.2 s ahead and 3 s in a spin", "result": d}
    reasons = []
    if d is None:
        reasons.append("the drive controller wrote no result in 180 s")
    else:
        for k in ("straight_wheel_rad_s", "spin_wheel_rad_s"):
            frac = d[k] / NO_LOAD_RAD_S
            out[f"{k}_of_no_load"] = round(frac, 3)
            if frac < 0.9:
                reasons.append(f"{k} {d[k]} is {100 * frac:.0f} % of the motors' no-load speed: a wheel is dragging")
    moved = float(np.linalg.norm(T1.t - T0.t) / MM)
    out["chassis_moved_mm"] = round(moved, 3)
    out["wheel_contacts_while_spinning"] = seen
    if moved > 1.0:
        reasons.append(f"the chassis moved {moved:.2f} mm on the stand while the wheels ran")
    if seen:
        reasons.append(f"a spinning wheel touched {sorted(seen)}")
    out["note"] = ("a wheel and the chassis it hangs from are parent and child of one hinge, and MuJoCo never "
                   "collides those: whether the wheel clears the gearbox is the press-on check above, not this one")
    h.screenshot(OUT / "physical_step_6_spin.png")
    out["screenshot"] = "physical_step_6_spin.png"
    out["pass"], out["reasons"], out["wall_s"] = not reasons, reasons, round(time.time() - t0, 1)
    return out


def run_drive_check(h: Harness, r: Robot, welds: List[Weld], written: List[Path]) -> dict:
    """The finished robot on the bench: it must stand on its wheels and caster, then drive."""
    t0 = time.time()
    res_path = TMP / "drive.json"
    res_path.unlink(missing_ok=True)
    world = WORLDS / ".desk_robot_assemble_7_drive.omniworld"
    start = Pose(None, (-0.45, 0.0, 0.0005))
    _write(world, world_text("desk robot, assembled, driven", [
        assy_node(welds, start, robot=True, controller="desk_robot_drive", args=[f"--result={res_path.as_posix()}"])],
        rails=False, eye=(-0.05, -0.75, 0.35), target=(0.0, 0.0, 0.02)), written)
    h.load(world)
    d, _ = _wait_json(h, res_path, 240)
    out: dict = {"world": "the finished robot on the bench, the drive controller: 1.2 s full ahead, stop, 3 s spin, stop",
                 "result": d}
    reasons = []
    p = robot_mod.predictions(r)
    if d is None:
        reasons.append("the drive controller wrote no result in 240 s")
    else:
        v = d["straight_speed_m_s"]
        out["distance_straight_m"] = round(v * 1.0, 3)
        out["predicted_no_load_speed_m_s"] = p["no_load_speed_m_s"]
        if not (0.8 * p["no_load_speed_m_s"] < v <= 1.01 * p["no_load_speed_m_s"]):
            reasons.append(f"straight speed {v} m/s against {p['no_load_speed_m_s']} predicted (no load)")
        if abs(d["heading_drift_deg_straight"]) > 2.0:
            reasons.append(f"heading drifted {d['heading_drift_deg_straight']} deg driving straight")
        z0, z1 = d["chassis_z_range_mm"]
        out["chassis_z_range_mm"] = [z0, z1]
    # After the drive the controller's "stop" is a velocity command of 0, a damper with no
    # static torque, and the caster is frictionless: what the robot then does is the motor
    # model's, not the assembly's. Recorded, not judged (the standing check judges stability).
    h.step(500)
    TA = h.pose("ASSY")
    corners = np.concatenate([q.corners() for q in _welded_prims(welds, jointed=True)])
    h.step(int(SETTLE_S * 1000 / DT_MS))
    TB = h.pose("ASSY")
    rel = TA.inv() @ TB
    out["after_drive_on_velocity_hold"] = {
        "moved_mm_in_0.5s": round(_max_disp_mm(TA, TB, corners), 4),
        "translation_mm_robot_frame": [round(v / MM, 4) for v in rel.t],
        "tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(TB.R[2, 2]))))), 3)}
    try:
        h.call("/scene/frame", {"def": "ASSY", "margin": 0.6})
        h.step(5)                               # the view redraws on the next steps
    except Exception as e:  # noqa: BLE001
        out["frame_error"] = str(e)[:300]
    h.screenshot(OUT / "physical_step_7_drive.png")
    out["screenshot"] = "physical_step_7_drive.png"
    out["pass"], out["reasons"], out["wall_s"] = not reasons, reasons, round(time.time() - t0, 1)
    return out


def run_standing_check(h: Harness, r: Robot, welds: List[Weld], written: List[Path]) -> dict:
    """The finished robot on the bench, brakes on: its 100:1 gearmotors do not back-drive
    under its own weight, so its wheels are welded. It must come to rest on its wheels and
    caster and then stay put."""
    t0 = time.time()
    world = WORLDS / ".desk_robot_assemble_7_standing.omniworld"
    # At the engine's default contact stiffness, as the drive test runs: at 10 and 50 kN/m the
    # whole robot on its wheels chatters (contacts flicker 5 <-> 9) and walks backwards
    # 0.4-4 mm/s, while at 2.5 kN/m it is still to 0.1 um (measured 2026-11-03, this file's
    # development runs). The parts on the stand's rails do not show it at 50 kN/m.
    _write(world, world_text("desk robot, assembled, standing", [assy_node(welds, Pose(None, (0.0, 0.0, 0.0005)))],
                             rails=False), written)
    h.load(world)
    h.pause()
    h.step(int(1000 / DT_MS))
    corners = np.concatenate([q.corners() for q in _welded_prims(welds)])
    TA = h.pose("ASSY")
    h.step(int(SETTLE_S * 1000 / DT_MS))
    TB = h.pose("ASSY")
    touch: Dict[str, int] = {}
    for i in range(CONTACT_POLL):
        h.step(1)
        for k2, v2 in _touching(h.contacts(), "ASSY").items():
            touch[k2] = touch.get(k2, 0) + v2
    pts = [c.get("point") for c in h.contacts() if "ASSY" in (c.get("a_def"), c.get("b_def"))]
    feet = {"wheel_L": 0, "wheel_R": 0, "caster": 0}
    for p in pts:
        q = TB.inv().apply(np.asarray(p, float)) / MM
        if q[0] < -20:
            feet["caster"] += 1
        else:
            feet["wheel_L" if q[1] > 0 else "wheel_R"] += 1
    drift = _max_disp_mm(TA, TB, corners)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(TB.R[2, 2])))))
    out = {"world": "the finished robot as one rigid body (wheels braked) dropped 0.5 mm onto the bench",
           "contact_ke": "engine default (2500 N/m)",
           "drift_mm": round(drift, 4), "tilt_deg": round(tilt, 3),
           "plate_underside_mm": round(float(TB.t[2] / MM + (r.plate_top_z - r.plate_t)), 3),
           "designed_plate_underside_mm": round(r.plate_top_z - r.plate_t, 3),
           "contacts": touch, "contacts_by_foot": feet}
    reasons = []
    if drift > DRIFT_TOL_MM:
        reasons.append(f"standing, it moved {drift:.4f} mm in {SETTLE_S} s (allowed {DRIFT_TOL_MM})")
    if "BENCH" not in touch:
        reasons.append(f"it does not touch the bench (touches {sorted(touch)})")
    for k, v in feet.items():
        if not v:
            reasons.append(f"it does not stand on its {k.replace('_', ' ')}")
    h.screenshot(OUT / "physical_step_7_standing.png")
    out["screenshot"] = "physical_step_7_standing.png"
    out["pass"], out["reasons"], out["wall_s"] = not reasons, reasons, round(time.time() - t0, 1)
    return out


def _static_clearance(r: Robot) -> dict:
    """bought_parts.clearance(r) on the drawn models (the FCL check of every solid against
    every other in the assembled robot): what this run's physics is compared against."""
    bp = _bought()
    if bp is None or not hasattr(bp, "clearance"):
        return {"available": False}
    try:
        c = bp.clearance(r, prefer_vendor=False)
    except Exception as e:  # noqa: BLE001
        return {"available": False, "error": f"{type(e).__name__}: {e}"}
    out = {"available": True, "models": "drawn (prefer_vendor=False)"}
    for k, v in c.items():
        out[k] = v if k in ("interference", "pressed") else (len(v) if isinstance(v, list) else v)
    return out


def _machine() -> dict:
    try:
        sys.path.insert(0, str(ROOT / "projects/policies/common"))
        import env_fingerprint
        return env_fingerprint._machine_identity(env_fingerprint._nvidia_gpu())
    except Exception as e:  # noqa: BLE001
        return {"error": str(e), "os": platform.platform(terse=True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Assemble the desk robot step by step on Newton, checking each step.")
    ap.add_argument("--port", type=int, default=6911, help="harness port (the supervisor takes port + 1)")
    ap.add_argument("--until", type=int, default=len(robot_mod.STEPS), help="stop after this step (1..7)")
    ap.add_argument("--out", default=None, help="default: release/assembly next to this file")
    a = ap.parse_args(argv)
    global OUT
    if a.out:
        OUT = Path(a.out)
    OUT.mkdir(parents=True, exist_ok=True)
    TMP.mkdir(parents=True, exist_ok=True)
    t_all = time.time()
    r = Robot()
    robot_mod.write_meshes(r)
    url = Path(os.path.relpath(robot_mod.MESHES, WORLDS)).as_posix()
    parts = kit(r, url)
    log = {"what": "the desk robot assembled step by step on Newton (MuJoCo CPU solver), each step checked",
           "simulation_only": True,
           "tolerances": {"seat_mm": SEAT_TOL_MM, "penetration_mm": PEN_TOL_MM, "drift_mm": DRIFT_TOL_MM,
                          "settle_s": SETTLE_S, "approach_mm": APPROACH_MM, "increment_mm": INCREMENT_MM,
                          "refine_mm": REFINE_MM, "holes": "per fastener, from design.CLEARANCE (see each hole's allowed_mm)"},
           "solver": {"basicTimeStep_ms": DT_MS, "newtonSubsteps": 2, "newtonSolver": "mujoco"},
           "method": ("each part from a kit on the bench to APPROACH_MM off its seat along its fastener axis, onto the "
                      "seat in INCREMENT_MM set_pose steps (one physics step each, contacts read after each), stopped "
                      "at the first contact before the seat; released for SETTLE_S under gravity (held if a press-on "
                      "part); measured from the engine's poses and contacts; then welded into the robot's body where "
                      "it came to rest, its screw heads and nuts with it"),
           "machine": _machine(), "joints_from": None, "static_clearance": _static_clearance(r), "steps": []}
    log["joints_from"] = _joint_rows(r)[3]
    h = Harness(a.port, TMP / "harness.log")
    written: List[Path] = []
    welds: List[Weld] = []
    ok = True
    error = None
    try:
        h.start()
        for k in range(min(a.until, len(robot_mod.STEPS))):
            print(f"step {k + 1}: {robot_mod.STEPS[k]}", flush=True)
            entry, welds = run_assembly_step(h, r, k, parts, welds, written)
            if k == 5:
                entry["spin_check"] = run_spin_check(h, r, welds, written)
                if not entry["spin_check"]["pass"]:
                    entry["pass"] = False
                    entry["reasons"] += [f"spin: {x}" for x in entry["spin_check"]["reasons"]]
            if k == 6:
                entry["standing_check"] = run_standing_check(h, r, welds, written)
                if not entry["standing_check"]["pass"]:
                    entry["pass"] = False
                    entry["reasons"] += [f"standing: {x}" for x in entry["standing_check"]["reasons"]]
                entry["drive_check"] = run_drive_check(h, r, welds, written)
                if not entry["drive_check"]["pass"]:
                    entry["pass"] = False
                    entry["reasons"] += [f"drive: {x}" for x in entry["drive_check"]["reasons"]]
            log["steps"].append(entry)
            ok = ok and entry["pass"]
            print(f"step {k + 1}: {'PASS' if entry['pass'] else 'FAIL'} ({entry['wall_s']} s)", flush=True)
    except Exception as e:  # noqa: BLE001
        import traceback
        error = f"{type(e).__name__}: {e}"
        traceback.print_exc()
        ok = False
    finally:
        h.stop()
        for w in written:
            w.unlink(missing_ok=True)
            # the harness writes a working copy beside it, and the GUI a perspective file
            (w.parent / f".harness_{w.name}").unlink(missing_ok=True)
            (w.parent / f"..harness_{w.stem}.omniperspective").unlink(missing_ok=True)
    log["mass_g"] = round(sum(p.mass_g for p in parts), 2)
    log["error"] = error
    log["all_pass"] = ok and error is None and len(log["steps"]) == len(robot_mod.STEPS)
    log["wall_s"] = round(time.time() - t_all, 1)
    (OUT / "assembly_log.json").write_text(json.dumps(log, indent=1), encoding="utf-8")
    print(f"wrote {OUT / 'assembly_log.json'} ({log['wall_s']} s): {'ALL PASS' if log['all_pass'] else 'FAILURES'}")
    return 0 if log["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
