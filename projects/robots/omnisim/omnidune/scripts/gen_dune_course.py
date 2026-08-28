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

"""Generate the OmniDune desert racing course -- a reusable terrain asset.

    python projects/robots/omnisim/omnidune/scripts/gen_dune_course.py
    python projects/robots/omnisim/omnidune/scripts/gen_dune_course.py --stats

Emits ``projects/robots/omnisim/omnidune/worlds/dune_course.omniworld``: a
200 m x 200 m dirt/desert racing terrain expressed ENTIRELY as elevation --
no props, no vehicle, one static ``Solid`` carrying one ``ElevationGrid``.

WHY ElevationGrid AND NOT A MESH
--------------------------------
An ``ElevationGrid`` used as a ``boundingObject`` is the one collider in this
engine that is NOT silently convexified.  ``OmSolid::attachNewtonShapeFrom-
BoundingObject`` (src/omnisim/nodes/OmSolid.cpp:3113) dispatches it to
``OmNewtonBackend::addShapeHeightfield`` (src/omnisim/physics/OmNewtonBackend.cpp:1519)
-> ``World.add_shape_heightfield`` (src/omnisim/physics/omnisim_newton_runtime.py:1166)
-> ``newton.Heightfield`` -> ``mjGEOM_HFIELD``
(newton/_src/solvers/mujoco/solver_mujoco.py:5659).  EVERY other mesh collider
-- ``IndexedFaceSet``, an imported CAD hull, a bowl -- goes through
``spec.add_mesh`` and MuJoCo's convex-hull path, so a concave shape becomes a
solid lump.  A dune basin authored as a mesh would be a featureless dome;
authored as an ElevationGrid it is the terrain the file declares, measured to
0.1 mm by OmniBench lane 4 (``object.elevationgrid_terrain``,
docs/benchmarks/lane4-capability-matrix.md).

THE RUNTIME'S CONSTRAINTS (read, not guessed -- omnisim_newton_runtime.py:1166)
------------------------------------------------------------------------------
* ``xDimension``/``yDimension`` must both be >= 2, else the runtime prints a
  refusal and registers NOTHING.
* ``len(height)`` must equal ``xDimension * yDimension`` EXACTLY.  One value
  short and the runtime refuses the whole heightfield and returns -1 -- leaving
  a world that loads, renders, prints PASS and has no ground.  ``--stats``
  asserts this, and so does ``emit()``.
* OmniSim authors by CELL SPACING + a dimension COUNT; ``newton.Heightfield``
  wants HALF-EXTENTS.  The runtime does the conversion
  (``span = spacing * (dim - 1)``, ``hx = span / 2``) and the VRML
  corner-origin -> newton centre-origin ``+span/2`` shift itself, so this file
  stays in OmniSim's units and the terrain lands where the ``.omniworld`` says.
* A newton heightfield is ALWAYS static / world-attached (``add_shape_
  heightfield`` takes no body argument).  The terrain ``Solid`` therefore has
  NO ``physics`` node, and a heightfield must never be put on something that
  moves.
* ``OmNewtonBackend::addShapeHeightfield`` refuses more than 4,000,000 samples
  with a named WARNING.  257 x 257 = 66,049 is 1.7% of that ceiling.

TWO CONSEQUENCES OF THE HEIGHTFIELD BEING A FINITE PATCH
--------------------------------------------------------
1. There is NO SKIRT and no implicit floor beneath it.  A body that leaves the
   footprint falls for ever -- measured on
   projects/samples/demos/worlds/physics/newton_heightfield_terrain.omniworld,
   where a ball rolled off the edge and reached z = -981.  This course
   therefore walls its own perimeter: the outer band climbs to +14 m.
   Containment is part of the terrain, not a separate prop.
2. MuJoCo gives the hfield ``size_base = 1e-4`` (solver_mujoco.py:6046) -- the
   collider is a SHELL, not a filled block.  Nothing is solid below the
   surface.  The course is shaped for a 15-25 m/s buggy, which at
   ``basicTimeStep 8`` moves 12-20 cm per step, comfortably inside the
   heightfield's collision margin.

THE COURSE
----------
The racing line is a FILLETED CONVEX POLYGON, not a spline: exact straight
segments joined by exact circular arcs.  That matters twice over -- a straight
has curvature EXACTLY zero, so it carries no banking at all, and the start
straight can therefore be made dead flat by construction and used as the
world's analytic datum.

From the start/finish line, running counter-clockwise (s = arclength, metres):

    s 0-26        flat start straight -- no grade, no bank, no micro-texture.
    s 40-100      the jump complex: kicker face -> lip at +4.35 m -> the ground
                  falls away to a -0.50 m gap -> a landing knuckle at +2.60 m
                  whose far side is the descending landing ramp.
    turn 1 (R22)  banked berm, 23.1 deg
    east straight WHOOPS -- 7 m rollers, +/- 0.42 m
    turn 2 (R30)  banked berm, 17.4 deg
    link straight light washboard, 5.5 m, +/- 0.22 m
    turn 3 (R28)  banked berm, 18.6 deg
    north straight long swells, 19 m, +/- 0.95 m
    turn 4 (R20)  the tightest berm, clamped at 24 deg
    west straight rollers, 14 m, +/- 0.62 m
    turn 5 (R22)  banked berm onto the start straight

Banking is derived, not drawn: ``tan(bank) = BANK_FRACTION * kappa * v^2 / g``
at the design speed, clamped, and tapered to zero at both tangent points of
each arc so the transition is smooth and the straights stay flat.  Outside the
racing surface every corner carries a berm lip whose height scales with the
banking, so the fast corners get something to lean on and the straights get a
low symmetric edge.

Everything outside the corridor is dune field: fBm value noise plus a
transverse dune train, lifted toward the rim.

DETERMINISM
-----------
Pure ``math`` + integer hashing; no ``random``, no ``numpy``, no clock, no
filesystem input.  Heights are emitted at 3 decimals (mm), which also absorbs
last-ULP libm differences, so the same ``(seed, span, cells)`` produces a
byte-identical world.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_OUT = Path(__file__).resolve().parents[1] / "worlds" / "dune_course.omniworld"

# --------------------------------------------------------------------------
# grid
# --------------------------------------------------------------------------
DEFAULT_SEED = 20260826
DEFAULT_SPAN = 200.0     # metres, square playable patch
DEFAULT_CELLS = 256      # cells per side -> cells + 1 samples per side

# --------------------------------------------------------------------------
# dune basin
# --------------------------------------------------------------------------
BASIN_NOISE_LEN = 90.0   # m per unit of the broad fBm octave
BASIN_AMP = 3.2          # m, +/- amplitude of the broad rolling landscape
DUNE_WAVELEN = 42.0      # m, crest-to-crest of the transverse dune train
DUNE_BEARING_DEG = 28.0  # dune crests run perpendicular to this bearing
DUNE_MEANDER = 14.0      # m of lateral crest wander
DUNE_AMP = 2.6           # m, dune crest above the trough
DUNE_SHARPNESS = 1.7     # >1 sharpens crests and flattens troughs
RIPPLE_AMP = 0.22        # m, fine wind ripple
BOWL_AMP = 4.0           # m the basin rises toward the rim
BOWL_INNER = 0.45        # normalised radius where that rise starts

# --------------------------------------------------------------------------
# containment rim -- see "TWO CONSEQUENCES" above; this is load bearing
# --------------------------------------------------------------------------
RIM_START = 87.0         # m (Chebyshev radius) where the wall starts climbing
RIM_TOP = 99.5           # m where it reaches full height
RIM_HEIGHT = 14.0        # m

# --------------------------------------------------------------------------
# track corridor
# --------------------------------------------------------------------------
HALF_W = 6.0             # m, half the racing surface (12 m wide)
BERM_W = 4.5             # m, berm / edge band outside the racing surface
SHOULDER = 12.0          # m, blend band from berm crest back to the dunes
DESIGN_SPEED = 16.0      # m/s the banking is computed for
BANK_FRACTION = 0.36     # fraction of the lateral load the berm carries.
                         # Full compensation at R=20, v=16 needs 52 deg, which
                         # is unrideable; a real berm takes a share of it.
MAX_BANK_DEG = 24.0      # hard clamp
BANK_TAPER_M = 8.0       # m of transition at each end of an arc (clothoid-ish)
GRADE_SMOOTH_M = 30.0    # +/- window (m) the road grade follows the landscape
GRADE_FOLLOW = 0.75      # how much of the smoothed landscape the road keeps
MICRO_AMP = 0.045        # m, ruts / washboard on the racing surface
CENTERLINE_STEP = 1.0    # m between centreline samples

# --------------------------------------------------------------------------
# the racing line: a convex polygon, traversed counter-clockwise, with a
# circular fillet at each vertex.  (x, y, fillet radius).
# --------------------------------------------------------------------------
TRACK_POLYGON = (
    (-76.0, -72.0, 22.0),   # V0  turn 5 -> onto the start straight
    (76.0, -72.0, 22.0),    # V1  turn 1
    (74.0, 24.0, 30.0),     # V2  turn 2
    (14.0, 68.0, 28.0),     # V3  turn 3
    (-72.0, 42.0, 20.0),    # V4  turn 4, the tightest
)
# s = 0 is the start of the straight leaving V0's fillet, i.e. straight index 0
# is the bottom straight.  Straight k is segment 2k, arc k is segment 2k+1.

# --------------------------------------------------------------------------
# features
# --------------------------------------------------------------------------
START_FLAT_END = 26.0    # m; the dead-flat start straight is s in [0, this]
START_FLAT_TAPER = 10.0  # m of taper out of (and into) the flat zone

# (arclength m, height offset m) knots of the jump complex, interpolated with
# smoothstep between consecutive knots.  Zero at both ends so it contributes
# nothing outside its span, which must fit inside straight 0 (106.75 m).
JUMP_KNOTS = (
    (40.0, 0.00),    # toe of the kicker
    (46.0, 0.20),
    (62.0, 4.10),
    (65.0, 4.35),    # THE LIP -- takeoff, 20 deg face
    (73.0, -0.50),   # back face of the kicker down into the gap
    (83.0, 2.60),    # landing knuckle
    (91.0, 1.20),    # landing ramp, descending
    (100.0, 0.00),
)

# Rhythm sections, one per straight: (label, straight index, inset m,
# wavelength m, amplitude m).  A section is skipped if its straight is too
# short to hold two full wavelengths, so retuning the polygon cannot silently
# produce half a whoop.
RHYTHM = (
    ("whoops", 1, 6.0, 7.0, 0.42),
    ("washboard", 2, 5.0, 5.5, 0.22),
    ("swells", 3, 6.0, 19.0, 0.95),
    ("rollers", 4, 6.0, 14.0, 0.62),
)

# Viewpoint: frame the jump complex (the hero feature), not the whole patch --
# a 200 m square framed as a single sphere puts the camera 800 m out and
# flattens every feature into texture.
HERO_AT_S = 70.0         # arclength of the shot's centre, just past the lip
HERO_RADIUS = 70.0       # m, framing radius handed to omniworld.viewpoint
HERO_LOOK_UP = 4.0       # m above the road the camera aims at


# ==========================================================================
# deterministic value noise
# ==========================================================================
_M32 = 0xFFFFFFFF


def _hash01(ix: int, iy: int, seed: int) -> float:
    """Repeatable [0,1) hash of an integer lattice corner.

    Pure integer arithmetic masked to 32 bits, so it is bit-identical on every
    platform -- unlike the ``sin(x) * 43758.5453`` idiom UnevenTerrain.proto
    uses, whose result depends on the host's libm.
    """
    h = (ix * 0x9E3779B1 + iy * 0x85EBCA77 + seed * 0xC2B2AE3D) & _M32
    h ^= h >> 15
    h = (h * 0x2545F491) & _M32
    h ^= h >> 13
    h = (h * 0x27D4EB2F) & _M32
    h ^= h >> 16
    return h / 4294967296.0


def _fade(t: float) -> float:
    """Quintic ease: first and second derivatives vanish at both ends, so
    adjacent noise cells meet without the creases a linear blend leaves."""
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _value_noise(x: float, y: float, seed: int) -> float:
    ix = math.floor(x)
    iy = math.floor(y)
    fx = _fade(x - ix)
    fy = _fade(y - iy)
    ix = int(ix)
    iy = int(iy)
    a = _hash01(ix, iy, seed)
    b = _hash01(ix + 1, iy, seed)
    c = _hash01(ix, iy + 1, seed)
    d = _hash01(ix + 1, iy + 1, seed)
    lo = a + (b - a) * fx
    hi = c + (d - c) * fx
    return lo + (hi - lo) * fy


def _fbm(x: float, y: float, seed: int, octaves: int,
         lacunarity: float = 2.0, gain: float = 0.5) -> float:
    """Fractal sum of value noise, normalised to roughly [-1, 1]."""
    total = 0.0
    norm = 0.0
    amp = 1.0
    freq = 1.0
    for o in range(octaves):
        total += amp * (_value_noise(x * freq, y * freq, seed + o * 7919) * 2.0 - 1.0)
        norm += amp
        amp *= gain
        freq *= lacunarity
    return total / norm


def _smoothstep(a: float, b: float, x: float) -> float:
    if b <= a:
        return 0.0 if x < a else 1.0
    t = (x - a) / (b - a)
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _circular_smooth(values, window_m: float, step: float):
    """Moving average over +/- window_m on a closed loop."""
    n = len(values)
    half = max(1, int(round(window_m / step)))
    out = [0.0] * n
    for i in range(n):
        acc = 0.0
        for k in range(-half, half + 1):
            acc += values[(i + k) % n]
        out[i] = acc / (2 * half + 1)
    return out


# ==========================================================================
# centreline: a filleted convex polygon
# ==========================================================================
class Centerline:
    """Arclength-parameterised closed racing line, sampled uniformly.

    Built from ``TRACK_POLYGON`` as alternating straight / circular-arc
    segments, so curvature is EXACT: identically zero on a straight and
    identically ``1/R`` on an arc.  That is what lets the start straight be
    dead flat and unbanked by construction rather than by luck.

    Attributes
    ----------
    pts     [(x, y)]      sample positions, world metres
    tan     [(tx, ty)]    unit tangents (direction of travel)
    kappa   [float]       signed curvature, rad/m; + = turning left (all
                          corners here are left turns -- the loop is CCW)
    s       [float]       arclength at each sample
    seg     [int]         segment index owning each sample
    seg_s   [float]       arclength at the start of each segment
    seg_len [float]       length of each segment
    straight_index        {straight ordinal -> segment index}
    length  float         lap length
    """

    def __init__(self, polygon, step: float):
        n = len(polygon)
        verts = [(p[0], p[1]) for p in polygon]
        radii = [p[2] for p in polygon]

        # --- fillet every vertex -------------------------------------
        # For vertex V with incoming unit direction d_in and outgoing d_out,
        # the fillet of radius R is tangent to both edges at a distance
        # t = R * tan(|theta| / 2) from V.
        fil = []
        for i in range(n):
            vp = verts[(i - 1) % n]
            v = verts[i]
            vn = verts[(i + 1) % n]
            d_in = _unit((v[0] - vp[0], v[1] - vp[1]))
            d_out = _unit((vn[0] - v[0], vn[1] - v[1]))
            theta = math.atan2(d_in[0] * d_out[1] - d_in[1] * d_out[0],
                               d_in[0] * d_out[0] + d_in[1] * d_out[1])
            if abs(theta) < 1e-9:
                raise ValueError("TRACK_POLYGON vertex %d is collinear" % i)
            R = radii[i]
            t = R * math.tan(abs(theta) * 0.5)
            p_in = (v[0] - d_in[0] * t, v[1] - d_in[1] * t)
            p_out = (v[0] + d_out[0] * t, v[1] + d_out[1] * t)
            # arc centre: offset from the entry tangent point along the normal
            # pointing to the inside of the turn.
            sgn = 1.0 if theta > 0.0 else -1.0
            nrm = (-d_in[1] * sgn, d_in[0] * sgn)
            centre = (p_in[0] + nrm[0] * R, p_in[1] + nrm[1] * R)
            a0 = math.atan2(p_in[1] - centre[1], p_in[0] - centre[0])
            edge = math.hypot(vn[0] - v[0], vn[1] - v[1])
            fil.append({"v": v, "R": R, "theta": theta, "trim": t,
                        "p_in": p_in, "p_out": p_out, "centre": centre,
                        "a0": a0, "sgn": sgn, "edge_len": edge})

        # --- segment table: straight k = segment 2k, arc k = segment 2k+1 --
        # Straight k runs from vertex k's fillet exit to vertex k+1's entry.
        segments = []
        self.straight_index = {}
        for k in range(n):
            nxt = fil[(k + 1) % n]
            slen = fil[k]["edge_len"] - fil[k]["trim"] - nxt["trim"]
            if slen <= 0.0:
                raise ValueError(
                    "fillet radii at vertices %d/%d eat the whole %.1f m edge"
                    % (k, (k + 1) % n, fil[k]["edge_len"]))
            self.straight_index[k] = len(segments)
            segments.append({"kind": "S", "len": slen,
                             "p0": fil[k]["p_out"],
                             "dir": _unit((nxt["p_in"][0] - fil[k]["p_out"][0],
                                           nxt["p_in"][1] - fil[k]["p_out"][1]))})
            f = nxt
            segments.append({"kind": "A", "len": f["R"] * abs(f["theta"]),
                             "R": f["R"], "theta": f["theta"],
                             "centre": f["centre"], "a0": f["a0"],
                             "sgn": f["sgn"]})
        self.segments = segments
        self.seg_len = [sg["len"] for sg in segments]
        self.seg_s = []
        acc = 0.0
        for sl in self.seg_len:
            self.seg_s.append(acc)
            acc += sl
        self.length = acc

        # --- uniform arclength sampling ------------------------------
        count = max(16, int(round(self.length / step)))
        self.step = self.length / count
        self.pts = []
        self.tan = []
        self.kappa = []
        self.s = []
        self.seg = []
        self.seg_off = []
        for i in range(count):
            s = i * self.step
            k = self._segment_of(s)
            local = s - self.seg_s[k]
            p, t, kap = self._eval(k, local)
            self.pts.append(p)
            self.tan.append(t)
            self.kappa.append(kap)
            self.s.append(s)
            self.seg.append(k)
            self.seg_off.append(local)

    def _segment_of(self, s: float) -> int:
        s = s % self.length
        lo, hi = 0, len(self.seg_s) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.seg_s[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def _eval(self, k: int, local: float):
        sg = self.segments[k]
        if sg["kind"] == "S":
            d = sg["dir"]
            p = (sg["p0"][0] + d[0] * local, sg["p0"][1] + d[1] * local)
            return p, d, 0.0
        R = sg["R"]
        sgn = sg["sgn"]
        a = sg["a0"] + sgn * (local / R)
        c = sg["centre"]
        p = (c[0] + R * math.cos(a), c[1] + R * math.sin(a))
        t = (-math.sin(a) * sgn, math.cos(a) * sgn)
        return p, t, sgn / R

    def index_at(self, s: float) -> int:
        return int(round((s % self.length) / self.step)) % len(self.pts)

    def point_at(self, s: float):
        k = self._segment_of(s)
        return self._eval(k, (s % self.length) - self.seg_s[k])[0]


def _unit(v):
    n = math.hypot(v[0], v[1]) or 1.0
    return (v[0] / n, v[1] / n)


# ==========================================================================
# the terrain
# ==========================================================================
class Course:
    """The generated terrain: the grid plus everything needed to query it."""

    def __init__(self, seed=DEFAULT_SEED, span=DEFAULT_SPAN, cells=DEFAULT_CELLS):
        if cells < 2:
            raise ValueError("cells must be >= 2")
        self.seed = int(seed)
        self.span = float(span)
        self.cells = int(cells)
        self.dim = self.cells + 1                     # samples per side
        self.spacing = self.span / float(self.cells)  # metres between samples
        # VRML puts the grid's (0,0) CORNER at the geometry origin, so the
        # Solid is translated by -span/2 to centre the patch on the world.
        self.origin = (-self.span * 0.5, -self.span * 0.5)
        self.line = Centerline(TRACK_POLYGON, CENTERLINE_STEP)
        self._build_road()
        self.heights = self._build_grid()

    # -- base landscape ---------------------------------------------------
    def base_height(self, wx: float, wy: float) -> float:
        """The dune basin, with no track carved into it."""
        z = BASIN_AMP * _fbm(wx / BASIN_NOISE_LEN, wy / BASIN_NOISE_LEN,
                             self.seed, 5)
        ang = math.radians(DUNE_BEARING_DEG)
        u = wx * math.cos(ang) + wy * math.sin(ang)
        meander = _fbm(wx / 55.0, wy / 55.0, self.seed + 1013, 3)
        phase = (u + DUNE_MEANDER * meander) * (2.0 * math.pi / DUNE_WAVELEN)
        z += DUNE_AMP * (0.5 - 0.5 * math.cos(phase)) ** DUNE_SHARPNESS
        z += RIPPLE_AMP * _fbm(wx / 6.0, wy / 6.0, self.seed + 2027, 2)
        r = math.hypot(wx, wy) / (self.span * 0.5)
        z += BOWL_AMP * _smoothstep(BOWL_INNER, 1.0, r)
        return z + self.rim_height(wx, wy)

    def rim_height(self, wx: float, wy: float) -> float:
        """Containment wall.  A newton heightfield has no skirt: without this
        a body that leaves the footprint falls for ever."""
        d = max(abs(wx), abs(wy))
        t = _smoothstep(RIM_START, RIM_TOP, d)
        return RIM_HEIGHT * t * t

    # -- longitudinal road profile ---------------------------------------
    def _flat_window(self, s: float) -> float:
        """1.0 inside the dead-flat start straight, tapering to 0 outside.

        The flat zone is the world's analytic datum.  On it the racing surface
        has no grade, no bank (straight 0 has curvature exactly zero) and no
        micro-texture, so a body dropped there must rest at exactly
        ``road0 + half_extent``.  That is what makes the collider provable with
        a number instead of a screenshot.
        """
        L = self.line.length
        s = s % L
        if s <= START_FLAT_END:
            return 1.0
        if s < START_FLAT_END + START_FLAT_TAPER:
            return 1.0 - _smoothstep(START_FLAT_END,
                                     START_FLAT_END + START_FLAT_TAPER, s)
        if s >= L - START_FLAT_TAPER:
            return _smoothstep(L - START_FLAT_TAPER, L, s)
        return 0.0

    @staticmethod
    def _knot_profile(knots, s: float) -> float:
        if s <= knots[0][0] or s >= knots[-1][0]:
            return 0.0
        for i in range(len(knots) - 1):
            s0, h0 = knots[i]
            s1, h1 = knots[i + 1]
            if s0 <= s <= s1:
                return h0 + (h1 - h0) * _smoothstep(s0, s1, s)
        return 0.0

    def _rhythm(self, s: float) -> float:
        """Whoops / washboard / swells / rollers, each windowed onto its own
        straight so it can never spill into a corner."""
        out = 0.0
        for _label, straight_k, inset, wavelen, amp in RHYTHM:
            seg = self.line.straight_index[straight_k]
            s0 = self.line.seg_s[seg] + inset
            s1 = self.line.seg_s[seg] + self.line.seg_len[seg] - inset
            if s1 - s0 < 2.0 * wavelen or s < s0 or s > s1:
                continue
            taper = min(_smoothstep(s0, s0 + wavelen, s),
                        1.0 - _smoothstep(s1 - wavelen, s1, s))
            out += amp * taper * math.sin(2.0 * math.pi * (s - s0) / wavelen)
        return out

    def _build_road(self):
        """Per-sample road height and banking."""
        line = self.line
        n = len(line.pts)
        raw = [self.base_height(p[0], p[1]) for p in line.pts]
        grade = [g * GRADE_FOLLOW
                 for g in _circular_smooth(raw, GRADE_SMOOTH_M, line.step)]
        self.road0 = grade[0]

        self.road = [0.0] * n
        for i in range(n):
            s = line.s[i]
            w = self._flat_window(s)
            g = grade[i] * (1.0 - w) + self.road0 * w
            feature = (self._knot_profile(JUMP_KNOTS, s) + self._rhythm(s)) * (1.0 - w)
            self.road[i] = g + feature

        # tan(bank) = BANK_FRACTION * kappa * v^2 / g, clamped, then tapered to
        # zero at both tangent points of the arc it belongs to.  A straight has
        # kappa exactly 0, so it is exactly unbanked -- no smoothing pass is
        # needed and none is applied, which is what keeps the start straight a
        # usable analytic datum.
        self.tan_max = math.tan(math.radians(MAX_BANK_DEG))
        v2_over_g = DESIGN_SPEED * DESIGN_SPEED / 9.81
        self.bank = [0.0] * n
        for i in range(n):
            k = line.kappa[i]
            if k == 0.0:
                continue
            seg = line.seg[i]
            seg_len = line.seg_len[seg]
            off = line.seg_off[i]
            taper_len = min(BANK_TAPER_M, seg_len * 0.4)
            f = min(_smoothstep(0.0, taper_len, off),
                    _smoothstep(0.0, taper_len, seg_len - off))
            self.bank[i] = f * _clamp(BANK_FRACTION * k * v2_over_g,
                                      -self.tan_max, self.tan_max)

    # -- cross-section ----------------------------------------------------
    def _cross_section(self, i: int, u: float) -> float:
        """Height of the racing surface at signed lateral offset ``u`` from
        centreline sample ``i``.  ``u`` > 0 is left of the direction of travel.

        Inside the racing surface the OUTSIDE of the corner is lifted (the
        inside stays level -- that is the shape of a real berm, not a
        symmetric tilt).  Outside it, an edge lip rises; its height scales
        with the banking, so a fast corner gets something to lean on and a
        straight gets a low symmetric kerb.
        """
        b = self.bank[i]
        o = -u if b >= 0.0 else u          # positive toward the corner outside
        mag = abs(b)
        z = mag * _clamp(o, 0.0, HALF_W)
        d = abs(u)
        if d > HALF_W:
            e = min(d - HALF_W, BERM_W) / BERM_W
            k = _clamp(mag / self.tan_max, 0.0, 1.0)
            lip = (0.8 + 1.6 * k) if o > 0.0 else (0.8 - 0.3 * k)
            z += lip * (e ** 1.5)
        return z

    # -- the grid ---------------------------------------------------------
    def _build_grid(self):
        dim = self.dim
        sp = self.spacing
        ox, oy = self.origin
        line = self.line
        n = len(line.pts)
        influence = HALF_W + BERM_W + SHOULDER
        inf2 = influence * influence

        # 1. base landscape everywhere
        heights = [0.0] * (dim * dim)
        for j in range(dim):
            wy = oy + j * sp
            row = j * dim
            for i in range(dim):
                heights[row + i] = self.base_height(ox + i * sp, wy)

        # 2. nearest centreline sample per affected cell.  SCATTER, not
        #    gather: cost is O(lap length x corridor area) instead of
        #    O(cells x samples), which is 1.9M inner steps instead of 34M.
        best_d2 = [None] * (dim * dim)
        best_k = [-1] * (dim * dim)
        for k in range(n):
            cx, cy = line.pts[k]
            i0 = max(0, int(math.ceil((cx - influence - ox) / sp)))
            i1 = min(dim - 1, int(math.floor((cx + influence - ox) / sp)))
            j0 = max(0, int(math.ceil((cy - influence - oy) / sp)))
            j1 = min(dim - 1, int(math.floor((cy + influence - oy) / sp)))
            for j in range(j0, j1 + 1):
                dy = oy + j * sp - cy
                dy2 = dy * dy
                row = j * dim
                for i in range(i0, i1 + 1):
                    dx = ox + i * sp - cx
                    d2 = dx * dx + dy2
                    if d2 > inf2:
                        continue
                    idx = row + i
                    cur = best_d2[idx]
                    if cur is None or d2 < cur:
                        best_d2[idx] = d2
                        best_k[idx] = k

        # 3. carve the corridor
        corridor_edge = HALF_W + BERM_W
        for idx in range(dim * dim):
            k = best_k[idx]
            if k < 0:
                continue
            i = idx % dim
            j = idx // dim
            px = ox + i * sp
            py = oy + j * sp
            cx, cy = line.pts[k]
            tx, ty = line.tan[k]
            dx = px - cx
            dy = py - cy
            along = dx * tx + dy * ty       # longitudinal refinement
            u = -dx * ty + dy * tx          # signed perpendicular offset
            d = abs(u)
            if d > corridor_edge + SHOULDER:
                continue
            alpha = 1.0 - _smoothstep(corridor_edge, corridor_edge + SHOULDER, d)
            if alpha <= 0.0:
                continue
            s_eff = line.s[k] + along
            z_track = self._road_at(s_eff) + self._cross_section(k, u)
            if MICRO_AMP > 0.0:
                z_track += MICRO_AMP * (1.0 - self._flat_window(s_eff)) * \
                    _fbm(px / 3.0, py / 3.0, self.seed + 404, 2)
            heights[idx] = heights[idx] * (1.0 - alpha) + z_track * alpha

        # QUANTISE HERE, NOT AT EMIT TIME.  The .omniworld carries 3 decimals
        # (mm), so the collider MuJoCo builds is the rounded grid -- rounding
        # in place makes ``height_at`` report exactly what the engine will
        # step, which is what lets a drop probe predict a rest height instead
        # of approximating one.
        return [float(_fmt(v)) for v in heights]

    def _road_at(self, s: float) -> float:
        """Road height at arbitrary arclength, linear between samples."""
        line = self.line
        t = (s % line.length) / line.step
        i0 = int(math.floor(t)) % len(self.road)
        f = t - math.floor(t)
        i1 = (i0 + 1) % len(self.road)
        return self.road[i0] + (self.road[i1] - self.road[i0]) * f

    # -- queries, for whoever builds the vehicle on top of this -----------
    def height_at(self, wx: float, wy: float) -> float:
        """Bilinear sample of the EMITTED grid, in world coordinates.

        The grid is already quantised to the 3 decimals the world file
        carries, so this returns exactly the elevation the engine registers.
        MuJoCo triangulates each hfield cell into two triangles, so this and
        the collider agree exactly on a grid vertex and on a locally planar
        patch, and differ by at most the cell's bilinear twist elsewhere.
        Probe on the flat start straight for an exact prediction.
        """
        ox, oy = self.origin
        fx = (wx - ox) / self.spacing
        fy = (wy - oy) / self.spacing
        i0 = int(math.floor(fx))
        j0 = int(math.floor(fy))
        if i0 < 0 or j0 < 0 or i0 >= self.dim - 1 or j0 >= self.dim - 1:
            raise ValueError("(%.3f, %.3f) is outside the terrain patch"
                             % (wx, wy))
        tx = fx - i0
        ty = fy - j0
        h = self.heights
        d = self.dim
        a = h[j0 * d + i0]
        b = h[j0 * d + i0 + 1]
        c = h[(j0 + 1) * d + i0]
        e = h[(j0 + 1) * d + i0 + 1]
        return (a + (b - a) * tx) * (1.0 - ty) + (c + (e - c) * tx) * ty

    def grid_vertex(self, i: int, j: int):
        """World (x, y, z) of grid sample (i, j) -- an EXACT surface point."""
        return (self.origin[0] + i * self.spacing,
                self.origin[1] + j * self.spacing,
                self.heights[j * self.dim + i])

    def nearest_vertex(self, wx: float, wy: float):
        i = int(round((wx - self.origin[0]) / self.spacing))
        j = int(round((wy - self.origin[1]) / self.spacing))
        return (max(0, min(self.dim - 1, i)), max(0, min(self.dim - 1, j)))


# ==========================================================================
# emit
# ==========================================================================
def _fmt(v: float) -> str:
    s = "%.3f" % v
    return "0.000" if s == "-0.000" else s


def _viewpoint(course: Course) -> str:
    sys.path.insert(0, str(REPO_ROOT / "src" / "python"))
    from omniworld.viewpoint import (  # noqa: E402
        format_orientation, format_position, hero_view,
    )
    cx, cy = course.line.point_at(HERO_AT_S)
    cz = course._road_at(HERO_AT_S) + HERO_LOOK_UP
    eye, orient = hero_view((cx, cy, cz), HERO_RADIUS)
    return ("Viewpoint {\n"
            "  orientation %s\n"
            "  position %s\n"
            "}\n" % (format_orientation(orient), format_position(eye)))


HEADER = '''#OMNISIM R2025a utf8

# OmniDune -- a %(span).0f m x %(span).0f m dirt/desert racing course, expressed ENTIRELY as
# terrain elevation.  No props, no vehicle: one static Solid, one ElevationGrid.
#
# GENERATED -- do not hand-edit.  Regenerate with
#   python projects/robots/omnisim/omnidune/scripts/gen_dune_course.py --seed %(seed)d
# The same (seed, span, cells) gives a byte-identical file.
#
# WHY THE COLLIDER IS AN ElevationGrid.  An ElevationGrid boundingObject is the
# ONE collider in this engine that is not silently convexified: OmSolid dispatches
# it to OmNewtonBackend::addShapeHeightfield -> newton.Heightfield -> mjGEOM_HFIELD.
# Every other mesh collider goes through MuJoCo's convex-hull path, which would
# turn this basin into a featureless dome.  Verified PASS to 0.1 mm by OmniBench
# lane 4 (object.elevationgrid_terrain).
#
# GRID: %(dim)d x %(dim)d samples at %(spacing).5f m cell spacing -> a %(patch).1f x %(patch).1f m patch.
#   height[] carries exactly %(count)d values (xDimension * yDimension).  ONE VALUE
#   SHORT AND THE RUNTIME REGISTERS NO HEIGHTFIELD AT ALL -- silently, with the
#   world still loading and printing PASS (omnisim_newton_runtime.py,
#   World.add_shape_heightfield).  The engine refuses > 4,000,000 samples; this
#   is 1.7%% of that ceiling.
#
# THE RIM IS NOT DECORATION.  A newton heightfield is a finite patch with no skirt
# and no implicit floor beneath it, so a body that leaves the footprint falls for
# ever.  The terrain walls itself instead: beyond a Chebyshev radius of %(rimstart).0f m it
# climbs to +%(rimh).0f m by the patch edge.  Keep vehicles inside |x|, |y| < %(rimstart).0f.
#
# THE TERRAIN IS STATIC BY CONSTRUCTION.  newton's add_shape_heightfield takes no
# body -- a heightfield is always world-attached, zero mass, zero inertia.  Do not
# add a Physics node to DUNE_COURSE and do not parent it to anything that moves.
#
# THE COURSE, counter-clockwise from the start/finish line (s = arclength, m):
#   s 0-%(flatend).0f       flat start straight -- DEAD FLAT BY CONSTRUCTION.  The racing
#                line here is straight segment 0, whose curvature is exactly zero,
#                so it carries no bank; the grade is pinned and the micro-texture
#                masked out.  This is the world's ANALYTIC DATUM: every emitted
#                sample inside it is exactly %(datum).3f m, so a body dropped on the
#                start straight must rest at %(datum).3f + its half-extent.  The flat
#                patch is x in [%(flatx0).2f, %(flatx1).2f] at y = %(flaty).2f, +/- %(flathw).1f m laterally.
#                Start grid: from (%(startx).2f, %(starty).2f) heading %(starthdg).0f deg.
#   s %(jump0).0f-%(jump1).0f     the jump complex -- kicker face to a lip at +%(liph).2f m (s = %(lips).0f),
#                then the ground falls away to a %(gaph).2f m gap and rises to a landing
#                knuckle at +%(knuckh).2f m whose far side is the descending landing ramp.
#   corners      %(ncorner)d banked berms, R = %(radii)s m.  tan(bank) = %(bankfrac).2f * kappa * v^2 / g
#                at v = %(vdes).0f m/s, clamped to %(bankmax).0f deg and tapered to zero at both
#                tangent points, so the straights stay flat.  Measured banking:
#                %(bankreport)s.
#   rhythm       %(rhythmreport)s
#   lap length   %(lap).2f m; racing surface %(width).0f m wide plus a %(berm).1f m berm each side.
#
# HEIGHT RANGE %(zmin).3f .. %(zmax).3f m.  Terrain-only world -- add a vehicle in a
# sibling world, or spawn one here and RELOAD (a runtime-spawned body is not in
# the solver; see AGENTS.md scene/spawn).
#
# LIGHTING is the canonical recipe (docs/WORLD_RECIPE.md): OmniSimSky +
# DEF SUN OmniSimSun + DEF SUN_MARKER OmniSimSunMarker.  No hand-written
# Background / DirectionalLight, no TexturedBackground, no NightSky.

EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/SandyGround.proto"

WorldInfo {
  title "OmniDune desert racing course"
  gravity 9.81
  basicTimeStep 8
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonGroundMu 1.2
  newtonCondim 6
  newtonCone "elliptic"
  newtonImpratio 10
  newtonNjmax 1024
  newtonNconmax 1024
}
'''


def _reports(course: Course):
    line = course.line
    banks = []
    for k, sg in enumerate(line.segments):
        if sg["kind"] != "A":
            continue
        idx = [i for i in range(len(line.pts)) if line.seg[i] == k]
        peak = max(abs(course.bank[i]) for i in idx) if idx else 0.0
        banks.append("R%.0f %.1f deg" % (sg["R"], math.degrees(math.atan(peak))))
    rhythm = []
    for label, straight_k, inset, wavelen, amp in RHYTHM:
        seg = line.straight_index[straight_k]
        usable = line.seg_len[seg] - 2.0 * inset
        if usable < 2.0 * wavelen:
            rhythm.append("%s SKIPPED (straight too short)" % label)
        else:
            rhythm.append("%s %.0f m x +/-%.2f m over %.0f m" %
                          (label, wavelen, amp, usable))
    return "; ".join(banks), "; ".join(rhythm)


def emit(course: Course) -> str:
    n = len(course.heights)
    if n != course.dim * course.dim:
        raise AssertionError("height[] has %d values, xDimension*yDimension "
                             "is %d -- the runtime would register NO "
                             "heightfield" % (n, course.dim * course.dim))
    line = course.line
    start_p = line.point_at(0.0)
    start_t = line.tan[0]
    bankrep, rhythmrep = _reports(course)
    head = HEADER % {
        "span": course.span,
        "patch": course.spacing * (course.dim - 1),
        "seed": course.seed,
        "dim": course.dim,
        "spacing": course.spacing,
        "count": n,
        "rimstart": RIM_START,
        "rimh": RIM_HEIGHT,
        "flatend": START_FLAT_END,
        "datum": course.height_at(start_p[0] + 6.0, start_p[1]),
        "flatx0": start_p[0] + 2.0,
        "flatx1": start_p[0] + START_FLAT_END - 2.0,
        "flaty": start_p[1],
        "flathw": HALF_W - 1.0,
        "startx": start_p[0], "starty": start_p[1],
        "starthdg": math.degrees(math.atan2(start_t[1], start_t[0])),
        "jump0": JUMP_KNOTS[0][0], "jump1": JUMP_KNOTS[-1][0],
        "liph": JUMP_KNOTS[3][1], "lips": JUMP_KNOTS[3][0],
        "gaph": JUMP_KNOTS[4][1], "knuckh": JUMP_KNOTS[5][1],
        "ncorner": len(TRACK_POLYGON),
        "radii": "/".join("%.0f" % v[2] for v in TRACK_POLYGON),
        "bankfrac": BANK_FRACTION,
        "vdes": DESIGN_SPEED,
        "bankmax": MAX_BANK_DEG,
        "bankreport": bankrep,
        "rhythmreport": rhythmrep,
        "lap": line.length,
        "width": HALF_W * 2.0,
        "berm": BERM_W,
        "zmin": min(course.heights),
        "zmax": max(course.heights),
    }

    out = [head, _viewpoint(course),
           "OmniSimSky {\n}\n",
           "DEF SUN OmniSimSun {\n}\n",
           "DEF SUN_MARKER OmniSimSunMarker {\n}\n",
           "DEF DUNE_COURSE Solid {\n"
           "  translation %s %s 0\n"
           '  name "dune course"\n'
           '  model "OmniDune terrain"\n'
           "  children [\n"
           "    DEF DUNE_COURSE_SHAPE Shape {\n"
           "      appearance SandyGround {\n"
           "        colorOverride 1 0.87 0.7\n"
           "        textureTransform TextureTransform {\n"
           "          scale 64 64\n"
           "        }\n"
           "      }\n"
           "      geometry ElevationGrid {\n"
           "        xDimension %d\n"
           "        yDimension %d\n"
           "        xSpacing %s\n"
           "        ySpacing %s\n"
           "        height [\n"
           % (_fmt(course.origin[0]), _fmt(course.origin[1]),
              course.dim, course.dim,
              repr(course.spacing), repr(course.spacing))]

    per_line = 16
    h = course.heights
    for start in range(0, n, per_line):
        out.append("          " +
                   " ".join(_fmt(v) for v in h[start:start + per_line]) + "\n")

    out.append("        ]\n"
               "      }\n"
               "    }\n"
               "  ]\n"
               "  boundingObject USE DUNE_COURSE_SHAPE\n"
               "}\n")
    return "".join(out)


# ==========================================================================
# CLI
# ==========================================================================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument("--span", type=float, default=DEFAULT_SPAN,
                    help="playable patch size, metres (default %(default)s)")
    ap.add_argument("--cells", type=int, default=DEFAULT_CELLS,
                    help="cells per side; samples per side is cells+1 "
                         "(default %(default)s)")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--stats", action="store_true",
                    help="print the grid contract, the course layout and the "
                         "analytic probe datum, then exit without writing")
    args = ap.parse_args(argv)

    course = Course(seed=args.seed, span=args.span, cells=args.cells)
    n = len(course.heights)
    line = course.line

    if args.stats:
        bankrep, rhythmrep = _reports(course)
        print("seed            %d" % course.seed)
        print("dimension       %d x %d samples" % (course.dim, course.dim))
        print("cell spacing    %r m" % course.spacing)
        print("patch span      %.3f m (%.3f x %d cells)"
              % (course.spacing * (course.dim - 1), course.spacing, course.cells))
        print("height[] length %d;  xDimension*yDimension = %d  -> %s"
              % (n, course.dim ** 2, "OK" if n == course.dim ** 2 else "MISMATCH"))
        print("engine ceiling  4000000 samples -> using %.2f%%"
              % (100.0 * n / 4000000.0))
        print("z range         %.3f .. %.3f m"
              % (min(course.heights), max(course.heights)))
        print("lap length      %.3f m" % line.length)
        print("banking         %s" % bankrep)
        print("rhythm          %s" % rhythmrep)
        print("segments:")
        for k, sg in enumerate(line.segments):
            print("  %2d %s len %7.3f  s %8.3f%s"
                  % (k, sg["kind"], sg["len"], line.seg_s[k],
                     "  R=%.0f" % sg["R"] if sg["kind"] == "A" else ""))
        print("road0 (pre-quantisation) %.6f m" % course.road0)
        sp = line.point_at(0.0)
        print("ANALYTIC DATUM (flat start straight), emitted grid heights:")
        for ds in (4.0, 8.0, 12.0, 16.0, 20.0):
            for du in (-4.0, 0.0, 4.0):
                pt = (sp[0] + ds, sp[1] + du)
                print("   s=%4.1f u=%+5.1f -> (%9.4f, %9.4f)  z = %.6f"
                      % (ds, du, pt[0], pt[1], course.height_at(*pt)))
        return 0

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(emit(course), encoding="utf-8", newline="\n")
    print("wrote %s" % out)
    print("  %d x %d samples, %r m spacing, %d heights, z %.3f .. %.3f m, "
          "lap %.1f m" % (course.dim, course.dim, course.spacing, n,
                          min(course.heights), max(course.heights), line.length))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
