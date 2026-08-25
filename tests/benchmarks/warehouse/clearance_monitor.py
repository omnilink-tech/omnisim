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

"""Measure how close the warehouse ever gets to a collision — from OUTSIDE.

Why this exists
---------------
The OMNITUG500 tugs in `warehouse_omnilink.omniworld` are KINEMATIC. They have no
collider, no mass and no Newton body: nothing in the physics engine can stop
one. Collision-free operation is produced *entirely* by the navigation layer
in `omnilink_mobile_bridge.py` — a swept oriented-footprint (SAT) test against
obstacles harvested by walking the live scene at startup, plus peer yielding
and a drawbar-angle clamp. That navigator is good. The problem is that
nothing MEASURED it, so the only available claim was the adjective:
"it avoids things".

This turns the adjective into a number, and derives it independently: the
navigator's own belief about where it is safe is never consulted. This tool
reads published POSE, applies its own geometry to its own obstacle set, and
reports the minimum separation that actually occurred, when, and between what.

    "minimum clearance over the 20-minute run was 8.4 cm, at t=612 s,
     between TUG_A and RACK_UPRIGHT_3, at (4.12, -2.88)"

WHAT THIS MEASURES — AND WHAT IT DOES NOT
-----------------------------------------
**This is a GEOMETRIC measurement of where the navigator put the robot. It is
NOT a physics result.** The tug has no collision body, so a reported clearance
of 0.08 m does not mean "the physics kept them 8 cm apart" — it means "the
navigator drove a footprint that came within 8 cm of a shape". Had it driven
straight through, the physics would not have objected; this tool would simply
have reported a negative number. Every clearance figure here is evidence about
the NAVIGATION LAYER. It is not evidence about contact handling, and it must
never be quoted as "the physics prevented a collision".

That is also precisely why the measurement is worth making: when nothing can
physically stop the vehicle, a geometric audit from outside the control loop is
the only evidence there is.

Two hard rules this file keeps (inherited from `measure_line.py`)
----------------------------------------------------------------
1. **GET only, never POST.** A POST to a bridge calls `note_external_command`
   and pauses that robot's idle loop for ~60 s. A monitor that perturbs what it
   monitors is worthless. Every request here is a GET.
2. **It writes exactly one file: `--out`.** It never opens a `.wbt` for
   writing, never touches a controller, a world or a log.

Usage
-----
    # With the harness running (best obstacle fidelity):
    python tests/benchmarks/warehouse/clearance_monitor.py --duration 1200 \
        --out tests/benchmarks/warehouse/results/clearance.json

    # No harness — parse the world file statically instead (weaker, see below):
    python tests/benchmarks/warehouse/clearance_monitor.py --duration 1200 \
        --source world \
        --world projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld

    # Validate the tool itself. No simulator, no network beyond loopback.
    python tests/benchmarks/warehouse/clearance_monitor.py --selftest

Exit codes
----------
    0  measured cleanly, no intersection
    2  a bridge did not answer preflight (or does not publish yaw)
    3  a bridge stopped answering mid-run (partial JSON still written)
    4  bad arguments
    5  no obstacle source could be resolved
    6  AN INTERSECTION OCCURRED (negative clearance) — the demo gate
    7  --selftest failed
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Session / polling helpers are REUSED, not re-implemented: `get_json` is the
# GET-only verb, `Recorder` + `collect` are the paced multi-bridge poll loop
# (ThreadPoolExecutor, tick resync, consecutive-failure bail-out), `preflight`
# is the "is this actually the warehouse world" probe, and `percentile` is the
# pinned linear-interpolated method the rest of this directory reports with.
from measure_line import (  # noqa: E402
    BridgeError, Recorder, _dig, _num, collect, get_json, percentile,
    preflight,
)

SCHEMA = "omnisim.warehouse.clearance_monitor/1"

EXIT_OK = 0
EXIT_PREFLIGHT = 2
EXIT_BRIDGE_DIED = 3
EXIT_ARGS = 4
EXIT_NO_OBSTACLES = 5
EXIT_INTERSECTION = 6
EXIT_SELFTEST = 7


# ══════════════════════════════════════════════════════════════════════
# MEASURED FOOTPRINTS — the constants block.
#
# These are measurements, not round numbers, and the difference matters: a
# 1.30 x 0.75 m "near enough" tug over-states the beam by 3.4 cm per side,
# which is a third of the warn threshold this tool reports against.
#
# OMNITUG500 (the warehouse tug):
#   0.7162 (X, beam) x 1.2595 (Y, fore-aft) x 0.2963 (Z) m, taken as the union
#   of the vehicle's 8 STL meshes. The mesh is centred on the origin in X and
#   Y, and its bottom face is flush with z = 0. Its local +Y is forward.
#   `omnilink_mobile_bridge.py` carries the same numbers rounded to 3 dp
#   (FOOT_L = 1.259, FOOT_W = 0.716); this file keeps 4 dp.
#
#   HEADING CONVENTION. The bridge publishes `/state.yaw` as the HEADING —
#   the direction of travel — with the mesh's +Y-forward offset already
#   removed (`_read_pose`: `wrap_pi(yaw - self.yaw_offset)`, yaw_offset =
#   -pi/2 for this vehicle). So the LONG (1.2595 m) axis lies along the
#   published yaw, exactly as the bridge's own `_rect(x, y, FOOT_L/2,
#   FOOT_W/2, yaw)` assumes. Get this 90 deg wrong and every number below is
#   wrong; it is asserted in --selftest.
#
# Trolley (the towed cart):
#   0.70 x 0.70 x 0.325 m collider. The bridge's `_DECK_M = 0.70` is the same
#   number.
# ══════════════════════════════════════════════════════════════════════

TUG_LEN_M = 1.2595       # along the published heading
TUG_WID_M = 0.7162       # across the heading
TUG_HGT_M = 0.2963       # bottom flush at z = 0

CART_LEN_M = 0.70
CART_WID_M = 0.70
CART_HGT_M = 0.325

# Obstacles whose TOP is at or below this are driven over, not avoided: floor
# decals, in-floor drag-chain conveyor decks, slots and hatching. Cited from
# `omnilink_mobile_bridge.DRIVE_OVER_Z = 0.06`, which is the OMNITUG500's ground
# clearance. Overridable with --drive-over-z.
DRIVE_OVER_Z = 0.06

# DEF-name prefixes for bodies that MOVE during the run. A scene snapshot of a
# moving body is stale the instant it is taken, so these are never used as
# static obstacles — carts are tracked live from bridge telemetry instead.
DEFAULT_MOBILE_PREFIXES = (
    "TROLLEY", "BOX_", "MAV_", "GRASP_PART_", "OMNIARM6",
)
DEFAULT_MOBILE_SUFFIXES = ("_DOG",)

# Node types a static .wbt parse can walk through. Anything else at the top
# level is a PROTO instance whose body is not in the file.
KNOWN_BASE_TYPES = (
    "Solid", "Group", "Transform", "Pose", "Shape", "Robot", "Supervisor",
)

# Node types whose geometry a static .wbt parse cannot resolve.
UNRESOLVABLE_TYPES = (
    "URDFRobot", "Mesh", "IndexedFaceSet", "ElevationGrid", "PointSet",
    "IndexedLineSet", "TriangleMeshGeometry",
)


# ══════════════════════════════════════════════════════════════════════
# Geometry.  Pure, no I/O — exercised by --selftest.
# ══════════════════════════════════════════════════════════════════════

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def rect(cx: float, cy: float, hx: float, hy: float,
         ang: float) -> List[Tuple[float, float]]:
    """Oriented rectangle -> its 4 corners, CW from (+hx, +hy).

    `hx` is the half-extent ALONG `ang`. Ported verbatim from
    `omnilink_mobile_bridge.MobileBridge._rect` so the monitor and the
    navigator describe the same quadrilateral.
    """
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + sx * c - sy * s, cy + sx * s + sy * c)
            for sx, sy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def obb_clearance(A: Sequence[Tuple[float, float]],
                  B: Sequence[Tuple[float, float]]) -> float:
    """Signed 2D clearance between two convex quads.

        >  0   true minimum distance between the two boundaries
        == 0   touching (returned as -0.0; NOT counted as an intersection)
        <  0   -penetration depth along the shallowest separating axis

    PORTED, NOT REINVENTED. This is `MobileBridge._obb_clearance`
    (`projects/samples/demos/controllers/omnilink_mobile_bridge/
    omnilink_mobile_bridge.py`, ~line 1226), the same routine the bridge uses
    for its towed-cart telemetry. Two implementations of the same test would
    eventually disagree and there would be no way to tell which was right, so
    this is a port with the identical axis set, the identical overlap
    convention, and the identical vertex->edge fallback.

    Why SAT on ORIENTED boxes and not an AABB shortcut: see the incident
    recorded at `MobileBridge._obb_vs_aabb`, which cost a whole measurement
    run. At a 27 deg heading the axis-aligned envelope of the 1.26 x 0.72 m
    tug is far larger than the tug, and the guard reported the vehicle inside
    the fill conveyor's kerb while its true footprint was 0.45 m clear. An
    AABB monitor would report that phantom as an excursion. --selftest pins a
    hand-computed case where the two answers disagree by construction.
    """
    def axes(poly):
        out = []
        for i in range(len(poly)):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % len(poly)]
            ex, ey = x1 - x0, y1 - y0
            L = math.hypot(ex, ey) or 1.0
            out.append((-ey / L, ex / L))
        return out

    def proj(poly, ax):
        ds = [p[0] * ax[0] + p[1] * ax[1] for p in poly]
        return min(ds), max(ds)

    separated = False
    min_overlap = 1e9
    for ax in axes(A) + axes(B):
        a0, a1 = proj(A, ax)
        b0, b1 = proj(B, ax)
        if max(b0 - a1, a0 - b1) > 0:
            separated = True
            break
        min_overlap = min(min_overlap, min(a1 - b0, b1 - a0))
    if not separated:
        return -min_overlap

    def seg_pt(px, py, ax_, ay_, bx_, by_):
        dx, dy = bx_ - ax_, by_ - ay_
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            return math.hypot(px - ax_, py - ay_)
        t = clamp(((px - ax_) * dx + (py - ay_) * dy) / l2, 0.0, 1.0)
        return math.hypot(px - (ax_ + t * dx), py - (ay_ + t * dy))

    d = 1e9
    for P, Q in ((A, B), (B, A)):
        for px, py in P:
            for i in range(len(Q)):
                qx0, qy0 = Q[i]
                qx1, qy1 = Q[(i + 1) % len(Q)]
                d = min(d, seg_pt(px, py, qx0, qy0, qx1, qy1))
    return d


def aabb_clearance(A: Sequence[Tuple[float, float]],
                   B: Sequence[Tuple[float, float]]) -> float:
    """The SHORTCUT this tool deliberately does not use, kept only so
    --selftest can demonstrate that it gives a different (wrong) answer.

    Signed gap between the axis-aligned envelopes of A and B.
    """
    ax0, ax1 = min(p[0] for p in A), max(p[0] for p in A)
    ay0, ay1 = min(p[1] for p in A), max(p[1] for p in A)
    bx0, bx1 = min(p[0] for p in B), max(p[0] for p in B)
    by0, by1 = min(p[1] for p in B), max(p[1] for p in B)
    dx = max(bx0 - ax1, ax0 - bx1)
    dy = max(by0 - ay1, ay0 - by1)
    if dx > 0 and dy > 0:
        return math.hypot(dx, dy)
    if dx > 0 or dy > 0:
        return max(dx, dy)
    return max(dx, dy)          # both <= 0: overlap, depth = the shallower


def circum_radius(hx: float, hy: float) -> float:
    return math.hypot(hx, hy)


# ══════════════════════════════════════════════════════════════════════
# Obstacles
# ══════════════════════════════════════════════════════════════════════

class Obstacle:
    """One world-space, AXIS-ALIGNED rectangle in XY plus its z-span.

    Axis-aligned is not a shortcut here — it is what both sources actually
    provide. The harness returns a world-space AABB per node; the static parse
    computes the world AABB of the transformed geometry. For a rotated
    obstacle that envelope is CONSERVATIVE (larger than the true shape), so
    clearance against it is UNDER-stated, never over-stated. The moving bodies
    are the oriented ones, and that is the side where an axis-aligned shortcut
    was measured to produce phantoms.
    """

    __slots__ = ("label", "sub", "x0", "y0", "x1", "y1", "z0", "z1",
                 "exact", "poly", "cx", "cy", "radius")

    def __init__(self, label, sub, x0, y0, x1, y1, z0, z1, exact=True):
        self.label = label
        self.sub = sub
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.z0, self.z1 = z0, z1
        self.exact = exact
        self.poly = [(x1, y1), (x1, y0), (x0, y0), (x0, y1)]
        self.cx, self.cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self.radius = math.hypot((x1 - x0) / 2.0, (y1 - y0) / 2.0)

    @property
    def name(self) -> str:
        return self.label if self.sub is None else f"{self.label}#{self.sub}"

    def as_json(self) -> dict:
        return {
            "name": self.name,
            "bbox": [round(self.x0, 4), round(self.y0, 4),
                     round(self.x1, 4), round(self.y1, 4)],
            "z": [round(self.z0, 4), round(self.z1, 4)],
            "exact": self.exact,
        }


def _is_mobile(def_name: Optional[str], prefixes: Sequence[str],
               suffixes: Sequence[str]) -> bool:
    if not def_name:
        return False
    up = def_name.upper()
    return (any(up.startswith(p) for p in prefixes)
            or any(up.endswith(s) for s in suffixes))


# ── Source A: the running harness ────────────────────────────────────

def obstacles_from_harness(base_url: str, timeout: float, token: str,
                           prefixes, suffixes) -> Dict[str, Any]:
    """GET /scene/tree?bounds=1 -> obstacle rectangles.

    Trust note, reported alongside every number this produces: the harness
    walks a LIVE scene graph, so it sees mesh bounds, PROTO expansions and
    URDF robot geometry that no text parse of a .wbt can see. That makes it
    the better source. But the harness runs its OWN simulator process: unless
    it is the very process the demo is running in, the poses it reports are
    the world's AUTHORED poses, not the live ones. That is fine for immobile
    structure (authored == live for a wall) and wrong for anything that
    moves, which is why mobile DEFs are excluded here and tracked from bridge
    telemetry instead.
    """
    data = get_json(base_url.rstrip("/") + "/scene/tree?bounds=1", timeout,
                    token)
    if not isinstance(data, dict):
        raise BridgeError(f"{base_url}/scene/tree returned "
                          f"{type(data).__name__}, expected an object")
    nodes = data.get("nodes") or []
    if not data.get("bounds_included"):
        raise BridgeError(
            f"{base_url}/scene/tree answered without bounds "
            f"(bounds_included=false). The supervisor's bounds index failed, "
            f"so there is no geometry to measure against.")

    # DEF -> its nearest DEF'd ancestor, so a node can be attributed to the
    # top-level body it belongs to.
    def_parent: Dict[str, Optional[str]] = {}
    for n in nodes:
        d = n.get("def")
        if d:
            def_parent[d] = n.get("parent_def")

    def owner_of(n: dict) -> Optional[str]:
        d = n.get("def") or n.get("parent_def")
        seen = set()
        while d and def_parent.get(d) and d not in seen:
            seen.add(d)
            d = def_parent[d]
        return d

    groups: Dict[str, Dict[str, Any]] = {}
    for n in nodes:
        own = owner_of(n)
        if own is None:
            continue
        g = groups.setdefault(own, {"self": None, "shapes": []})
        if n.get("def") == own and n.get("parent_def") is None:
            g["self"] = n
        if n.get("type") == "Shape" and n.get("bounds"):
            g["shapes"].append(n)

    obstacles: List[Obstacle] = []
    unchecked: List[Dict[str, str]] = []
    excluded_mobile: List[str] = []
    granularity = {"shape_level": 0, "node_level": 0}

    for own, g in sorted(groups.items()):
        if _is_mobile(own, prefixes, suffixes):
            excluded_mobile.append(own)
            continue
        rows = g["shapes"]
        level = "shape_level"
        if not rows:
            node = g["self"]
            if node is None or not node.get("bounds"):
                unchecked.append({
                    "name": own,
                    "reason": "no bounds returned for this node or any Shape "
                              "beneath it",
                })
                continue
            rows = [node]
            level = "node_level"
        added = 0
        for i, row in enumerate(rows):
            b = row.get("bounds") or {}
            lo, hi = b.get("bbox_min"), b.get("bbox_max")
            if not (isinstance(lo, (list, tuple)) and len(lo) == 3
                    and isinstance(hi, (list, tuple)) and len(hi) == 3):
                continue
            obstacles.append(Obstacle(
                own, i if len(rows) > 1 else None,
                float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]),
                float(lo[2]), float(hi[2]),
                exact=bool(b.get("exact", True))))
            added += 1
        if added:
            granularity[level] += 1
        else:
            unchecked.append({"name": own,
                              "reason": "bounds present but not a usable "
                                        "bbox_min/bbox_max pair"})

    return {
        "source": "harness",
        "detail": f"GET {base_url.rstrip('/')}/scene/tree?bounds=1 "
                  f"({len(nodes)} nodes)",
        "obstacles": obstacles,
        "unchecked": unchecked,
        "excluded_mobile": sorted(excluded_mobile),
        "granularity": granularity,
    }


# ── Source B: a static .wbt parse ────────────────────────────────────
#
# Weaker than the harness, and the report says so: this sees only Box /
# Cylinder / Capsule / Sphere / Plane primitives written literally in the
# file. It cannot expand a PROTO, cannot read a mesh, and cannot see inside a
# URDFRobot. Everything it cannot resolve is listed by name as UNCHECKED
# rather than dropped, because a clearance number that quietly excluded the
# racking is worse than no clearance number at all.

def _tokenize(text: str) -> List[Tuple[str, str]]:
    tokens: List[Tuple[str, str]] = []
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        if ch == "#":
            j = text.find("\n", i)
            i = n if j < 0 else j + 1
            continue
        if ch in " \t\r\n,":
            i += 1
            continue
        if ch == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == "\\" and j + 1 < n:
                    buf.append(text[j + 1])
                    j += 2
                    continue
                buf.append(text[j])
                j += 1
            tokens.append(("str", "".join(buf)))
            i = j + 1
            continue
        if ch in "{}[]":
            tokens.append(("punct", ch))
            i += 1
            continue
        j = i
        while j < n and text[j] not in ' \t\r\n,{}[]"#':
            j += 1
        tokens.append(("word", text[i:j]))
        i = j
    return tokens


def _is_number(w: str) -> bool:
    try:
        float(w)
        return True
    except ValueError:
        return False


class _WbtParser:
    """Minimal VRML-shaped reader: enough for `.wbt` node/field structure."""

    def __init__(self, tokens: List[Tuple[str, str]]) -> None:
        self.tk = tokens
        self.i = 0
        self.defs: Dict[str, dict] = {}

    def _peek(self):
        return self.tk[self.i] if self.i < len(self.tk) else None

    def _next(self):
        t = self._peek()
        if t is not None:
            self.i += 1
        return t

    def parse_roots(self) -> List[dict]:
        out = []
        guard = 0
        while self._peek() is not None and guard < 10_000_000:
            guard += 1
            before = self.i
            node = self.parse_node()
            if node:
                out.append(node)
            if self.i == before:          # made no progress; skip a token
                self.i += 1
        return out

    def parse_node(self) -> Optional[dict]:
        t = self._next()
        if t is None:
            return None
        kind, val = t
        if kind == "punct":
            return None
        if kind == "str":
            return None
        if val == "DEF":
            nxt = self._next()
            if nxt is None:
                return None
            node = self.parse_node()
            if node is not None:
                node["def"] = nxt[1]
                self.defs[nxt[1]] = node
            return node
        if val == "USE":
            nxt = self._next()
            return {"use": nxt[1]} if nxt else None
        nxt = self._peek()
        if nxt is not None and nxt == ("punct", "{"):
            self._next()
            return {"type": val, "fields": self.parse_fields()}
        return None                        # bare keyword (EXTERNPROTO, ...)

    def parse_fields(self) -> Dict[str, Any]:
        fields: Dict[str, Any] = {}
        while True:
            t = self._peek()
            if t is None:
                return fields
            if t == ("punct", "}"):
                self._next()
                return fields
            self._next()
            if t[0] != "word":
                continue
            fields[t[1]] = self.parse_value()

    def parse_value(self) -> Any:
        t = self._peek()
        if t is None:
            return None
        if t == ("punct", "["):
            self._next()
            items = []
            while True:
                nxt = self._peek()
                if nxt is None or nxt == ("punct", "]"):
                    self._next()
                    return items
                before = self.i
                items.append(self.parse_value())
                if self.i == before:
                    self._next()
        if t[0] == "str":
            self._next()
            return t[1]
        if t[0] == "punct":
            self._next()
            return None
        w = t[1]
        if w in ("TRUE", "FALSE"):
            self._next()
            return w == "TRUE"
        if _is_number(w):
            nums = []
            while True:
                p = self._peek()
                if p is None or p[0] != "word" or not _is_number(p[1]):
                    break
                nums.append(float(p[1]))
                self._next()
            return nums[0] if len(nums) == 1 else nums
        if w in ("DEF", "USE"):
            return self.parse_node()
        nxt = self.tk[self.i + 1] if self.i + 1 < len(self.tk) else None
        if nxt == ("punct", "{"):
            return self.parse_node()
        self._next()
        return {"symbol": w}


def _axis_angle_matrix(axis_angle: Sequence[float]):
    """SFRotation (x y z angle) -> a row-major 3x3 tuple."""
    if not axis_angle or len(axis_angle) < 4:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z, a = (float(axis_angle[0]), float(axis_angle[1]),
                  float(axis_angle[2]), float(axis_angle[3]))
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    return (
        (x * x * C + c,     x * y * C - z * s, x * z * C + y * s),
        (y * x * C + z * s, y * y * C + c,     y * z * C - x * s),
        (z * x * C - y * s, z * y * C + x * s, z * z * C + c),
    )


def _mat_mul(A, B):
    return tuple(
        tuple(sum(A[r][k] * B[k][c] for k in range(3)) for c in range(3))
        for r in range(3))


def _apply(R, t, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2] + t[0],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2] + t[1],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2] + t[2])


def _vec3(val, default=(0.0, 0.0, 0.0)):
    if isinstance(val, (list, tuple)) and len(val) >= 3:
        return (float(val[0]), float(val[1]), float(val[2]))
    return default


def _half_extents(typ: str, f: dict) -> Optional[Tuple[float, float, float]]:
    """Local-frame half-extents of a primitive's bounding box.

    Cylinder/Capsule are taken as axis-along-LOCAL-Z, which is what
    `MobileBridge._walk_geometry` assumes and what this world's authored
    wheels confirm (a `Cylinder` under `rotation 1 0 0 1.5708` becomes a
    horizontal axle).
    """
    if typ == "Box":
        s = _vec3(f.get("size"), (0.0, 0.0, 0.0))
        return (abs(s[0]) / 2.0, abs(s[1]) / 2.0, abs(s[2]) / 2.0)
    if typ in ("Cylinder", "Capsule"):
        r = float(f.get("radius") or 0.0)
        h = float(f.get("height") or 0.0)
        half_z = h / 2.0 + (r if typ == "Capsule" else 0.0)
        return (abs(r), abs(r), abs(half_z))
    if typ == "Sphere":
        r = abs(float(f.get("radius") or 0.0))
        return (r, r, r)
    if typ == "Plane":
        s = f.get("size")
        if isinstance(s, (list, tuple)) and len(s) >= 2:
            return (abs(float(s[0])) / 2.0, abs(float(s[1])) / 2.0, 0.0)
        return (0.5, 0.5, 0.0)
    return None


def _walk_static(node, R, t, scale, label, out, unchecked, defs, depth=0):
    if node is None or depth > 12:
        return
    if "use" in node:
        target = defs.get(node["use"])
        if target is None:
            unchecked.append({"name": f"{label}/USE {node['use']}",
                              "reason": "USE reference not resolvable"})
            return
        node = target
    typ = node.get("type")
    f = node.get("fields") or {}
    if typ in UNRESOLVABLE_TYPES:
        unchecked.append({
            "name": f"{label}/{node.get('def') or typ}",
            "reason": f"{typ} geometry cannot be resolved by a static .wbt "
                      f"parse (no PROTO expansion, no mesh reads)",
        })
        return

    tr = _vec3(f.get("translation"))
    rot = f.get("rotation")
    Rl = _axis_angle_matrix(rot if isinstance(rot, (list, tuple)) else None)
    sc = f.get("scale")
    sl = _vec3(sc, (1.0, 1.0, 1.0)) if isinstance(sc, (list, tuple)) else (1.0, 1.0, 1.0)

    tw = _apply(R, t, (tr[0] * scale[0], tr[1] * scale[1], tr[2] * scale[2]))
    Rw = _mat_mul(R, Rl)
    sw = (scale[0] * sl[0], scale[1] * sl[1], scale[2] * sl[2])

    he = _half_extents(typ, f) if typ else None
    if he is not None:
        hx, hy, hz = he[0] * sw[0], he[1] * sw[1], he[2] * sw[2]
        xs, ys, zs = [], [], []
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                for sz in (-hz, hz):
                    p = _apply(Rw, tw, (sx, sy, sz))
                    xs.append(p[0])
                    ys.append(p[1])
                    zs.append(p[2])
        out.append((min(xs), min(ys), max(xs), max(ys), min(zs), max(zs)))
        return

    for fname in ("children", "geometry", "boundingObject", "endPoint"):
        v = f.get(fname)
        if v is None:
            continue
        kids = v if isinstance(v, list) else [v]
        for kid in kids:
            if isinstance(kid, dict):
                _walk_static(kid, Rw, tw, sw, label, out, unchecked, defs,
                             depth + 1)


def obstacles_from_world(path: str, prefixes, suffixes) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    parser = _WbtParser(_tokenize(text))
    roots = parser.parse_roots()

    obstacles: List[Obstacle] = []
    unchecked: List[Dict[str, str]] = []
    excluded_mobile: List[str] = []

    for node in roots:
        if not isinstance(node, dict):
            continue
        label = node.get("def")
        typ = node.get("type")
        if typ in ("WorldInfo", "Viewpoint", "Background", "Fog",
                   "DirectionalLight", "PointLight", "SpotLight",
                   "OmniSimSky", "OmniSimSun", "OmniSimSunMarker"):
            continue
        if label is None and typ is None:
            continue
        # A DEF-less node still has to be IDENTIFIABLE in the unchecked list --
        # eleven rows reading "<Wall>" tell a reader nothing, so tag them with
        # their authored position.
        name = label
        if name is None:
            tr = _vec3((node.get("fields") or {}).get("translation"))
            name = f"<{typ} @ {tr[0]:+.2f},{tr[1]:+.2f}>"
        if _is_mobile(label, prefixes, suffixes):
            excluded_mobile.append(name)
            continue
        # A PROTO instance (Wall, Fence, Floor, CardboardBox, ...) has no
        # expandable body in the file, so a text parse can never see its
        # geometry. THIS IS REPORTED, LOUDLY, AND NOT DROPPED: measured on
        # warehouse_omnilink.omniworld, the building shell is 11 `Wall` PROTOs and
        # 3 `Fence` PROTOs. An earlier revision of this function silently
        # skipped them and produced a confident clearance number computed
        # against a warehouse with no walls in it.
        if typ not in KNOWN_BASE_TYPES:
            unchecked.append({
                "name": name,
                "reason": f"'{typ}' is a PROTO (or unknown) node; a static "
                          f".wbt parse cannot expand it. Use the harness "
                          f"source to measure against this shape.",
            })
            continue
        boxes: List[tuple] = []
        _walk_static(node, ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0.0, 0.0, 0.0),
                     (1.0, 1.0, 1.0), name, boxes, unchecked, parser.defs)
        if not boxes:
            unchecked.append({
                "name": name,
                "reason": f"no primitive geometry (Box/Cylinder/Capsule/"
                          f"Sphere/Plane) resolved under this {typ}; it may "
                          f"be a pose marker, or its shapes may be PROTOs",
            })
            continue
        for i, (x0, y0, x1, y1, z0, z1) in enumerate(boxes):
            obstacles.append(Obstacle(name, i if len(boxes) > 1 else None,
                                      x0, y0, x1, y1, z0, z1, exact=True))

    return {
        "source": "world",
        "detail": f"static parse of {path} ({len(roots)} root nodes)",
        "obstacles": obstacles,
        "unchecked": unchecked,
        "excluded_mobile": sorted(excluded_mobile),
        "granularity": {"shape_level": len(obstacles), "node_level": 0},
    }


def filter_obstacles(obstacles: List[Obstacle], drive_over_z: float,
                     body_top_z: float) -> Tuple[List[Obstacle], Dict[str, List[str]]]:
    """Split into (candidates, dropped-by-rule).

    Dropped is REPORTED, not silent: a floor decal and an overhead roof beam
    are both legitimately un-hittable by a 0.30 m tall vehicle with 6 cm of
    ground clearance, but which nodes were set aside on that basis has to be
    inspectable or the clearance number is unfalsifiable.
    """
    keep: List[Obstacle] = []
    dropped = {"driven_over": [], "overhead": [], "degenerate": []}
    for o in obstacles:
        if not all(math.isfinite(v) for v in (o.x0, o.y0, o.x1, o.y1,
                                              o.z0, o.z1)):
            dropped["degenerate"].append(o.name)
            continue
        if (o.x1 - o.x0) <= 0 or (o.y1 - o.y0) <= 0:
            dropped["degenerate"].append(o.name)
            continue
        if o.z1 <= drive_over_z:
            dropped["driven_over"].append(o.name)
            continue
        if o.z0 >= body_top_z:
            dropped["overhead"].append(o.name)
            continue
        keep.append(o)
    return keep, dropped


# ══════════════════════════════════════════════════════════════════════
# Collection — reuses measure_line's paced poll loop verbatim.
# ══════════════════════════════════════════════════════════════════════

class PoseRecorder(Recorder):
    """A `Recorder` that keeps POSE, not the whole state history.

    `collect()` hands every polled `/state` to `add()`. A 20-minute run at
    10 Hz is 12 000 samples per bridge, and the full state dict is large, so
    only the last one is retained (the progress line reads it) and each sample
    is reduced to the few numbers this tool measures.
    """

    def __init__(self, name: str, url: str) -> None:
        super().__init__(name, url)
        self.frames: List[dict] = []
        self.yaw_missing = 0

    def add(self, t: float, state: dict, latency: float) -> None:
        self.t.append(t)
        self.sim.append(_num(state.get("sim_time")))
        self.states = [state]               # last only; keeps memory O(1)
        self.latencies.append(latency)
        self.consecutive_failures = 0

        x, y = _num(state.get("x")), _num(state.get("y"))
        yaw = _num(state.get("yaw"))
        if yaw is None:
            self.yaw_missing += 1
        towed = state.get("towed") if isinstance(state.get("towed"),
                                                 dict) else None
        cart = None
        if towed is not None:
            cx, cy = _num(towed.get("x")), _num(towed.get("y"))
            cyaw = _num(towed.get("yaw"))
            if cx is not None and cy is not None:
                cart = {"def": towed.get("def") or f"{self.name}_cart",
                        "x": cx, "y": cy, "yaw": cyaw,
                        "artic_deg": _num(towed.get("artic_deg"))}
        self.frames.append({
            "t": t,
            "sim": _num(state.get("sim_time")),
            "x": x, "y": y, "yaw": yaw,
            "leg": _dig(state, "idle_loop", "leg"),
            "cart": cart,
        })


def check_yaw_published(recs: Dict[str, PoseRecorder], timeout: float,
                        token: str) -> List[str]:
    """Hard-fail if a tug does not publish yaw.

    An oriented footprint needs a heading. If a bridge does not give one there
    is no honest fallback: an axis-aligned box would silently over-state the
    beam by up to 80 % at 45 deg and manufacture excursions that never
    happened. So this refuses to run rather than quietly measuring the wrong
    shape.
    """
    problems = []
    for name, rec in recs.items():
        if name == "omniarm6":
            continue
        st = get_json(rec.url + "/state", timeout, token)
        if _num(_dig(st, "yaw")) is None:
            problems.append(
                f"  {name:6s} {rec.url}/state publishes no numeric `yaw`.\n"
                f"         This tool measures ORIENTED footprints; without a "
                f"heading it\n"
                f"         cannot build one, and it will not silently "
                f"substitute an\n"
                f"         axis-aligned box (that shortcut is the documented "
                f"cause of a\n"
                f"         phantom incursion in the navigator's own history)."
            )
    return problems


def merge_frames(recs: Dict[str, PoseRecorder], period: float) -> List[dict]:
    """Bucket per-bridge samples into simultaneous frames.

    `collect()` polls every bridge inside one `pool.map` per tick, so samples
    from the same tick share a wall timestamp to within the poll latency.
    Bucketing on `round(t / period)` therefore groups exactly the readings the
    poll loop intended to be simultaneous, and a bridge that missed a poll
    simply leaves its slot empty in that frame instead of silently shifting
    every later comparison by one tick.
    """
    buckets: Dict[int, Dict[str, dict]] = {}
    for name, rec in recs.items():
        if name == "omniarm6":
            continue
        for fr in rec.frames:
            buckets.setdefault(int(round(fr["t"] / period)), {})[name] = fr
    return [dict(sorted(buckets[k].items())) for k in sorted(buckets)]


# ══════════════════════════════════════════════════════════════════════
# Evaluation
# ══════════════════════════════════════════════════════════════════════

CATEGORIES = ("tug_vs_static", "tug_vs_tug", "tug_vs_cart",
              "cart_vs_static", "tug_vs_own_tow")


class Sample:
    __slots__ = ("t", "sim", "category", "a", "b", "clearance", "ax", "ay",
                 "bx", "by")

    def __init__(self, t, sim, category, a, b, clearance, ax, ay, bx, by):
        self.t = t
        self.sim = sim
        self.category = category
        self.a = a
        self.b = b
        self.clearance = clearance
        self.ax, self.ay = ax, ay
        self.bx, self.by = bx, by

    def as_json(self) -> dict:
        return {
            "t_wall_s": round(self.t, 3),
            "sim_time_s": None if self.sim is None else round(self.sim, 3),
            "category": self.category,
            "a": self.a, "b": self.b,
            "clearance_m": round(self.clearance, 5),
            "a_xy": [round(self.ax, 3), round(self.ay, 3)],
            "b_xy": [round(self.bx, 3), round(self.by, 3)],
        }


def evaluate_frame(frame: Dict[str, dict], obstacles: List[Obstacle],
                   geom: dict, broad_m: float,
                   out: List[Sample], skipped: Dict[str, int]) -> None:
    """One frame -> every measurable pair clearance in it."""
    tug_hx, tug_hy = geom["tug_len"] / 2.0, geom["tug_wid"] / 2.0
    cart_hx, cart_hy = geom["cart_len"] / 2.0, geom["cart_wid"] / 2.0
    tug_r = circum_radius(tug_hx, tug_hy)
    cart_r = circum_radius(cart_hx, cart_hy)

    bodies: Dict[str, dict] = {}
    carts: Dict[str, dict] = {}
    t = None
    sim = None
    for name, fr in frame.items():
        t = fr["t"] if t is None else t
        sim = fr["sim"] if sim is None else sim
        if fr["x"] is None or fr["y"] is None:
            skipped["pose_missing"] = skipped.get("pose_missing", 0) + 1
            continue
        if fr["yaw"] is None:
            skipped["yaw_missing"] = skipped.get("yaw_missing", 0) + 1
            continue
        bodies[name] = {
            "x": fr["x"], "y": fr["y"], "yaw": fr["yaw"],
            "poly": rect(fr["x"], fr["y"], tug_hx, tug_hy, fr["yaw"]),
        }
        c = fr.get("cart")
        if c is not None:
            if c.get("yaw") is None:
                skipped["cart_yaw_missing"] = skipped.get(
                    "cart_yaw_missing", 0) + 1
            else:
                carts[c["def"]] = {
                    "x": c["x"], "y": c["y"], "yaw": c["yaw"],
                    "towed_by": name,
                    "poly": rect(c["x"], c["y"], cart_hx, cart_hy, c["yaw"]),
                }

    def vs_static(body_name, body, radius, category):
        for o in obstacles:
            if math.hypot(o.cx - body["x"], o.cy - body["y"]) > (
                    radius + o.radius + broad_m):
                continue
            d = obb_clearance(body["poly"], o.poly)
            out.append(Sample(t, sim, category, body_name, o.name, d,
                              body["x"], body["y"], o.cx, o.cy))

    for name, body in bodies.items():
        vs_static(name, body, tug_r, "tug_vs_static")
    for cdef, cart in carts.items():
        vs_static(cdef, cart, cart_r, "cart_vs_static")

    names = sorted(bodies)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = bodies[names[i]], bodies[names[j]]
            if math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > (
                    2 * tug_r + broad_m):
                continue
            out.append(Sample(t, sim, "tug_vs_tug", names[i], names[j],
                              obb_clearance(a["poly"], b["poly"]),
                              a["x"], a["y"], b["x"], b["y"]))

    for name, body in bodies.items():
        for cdef, cart in carts.items():
            if math.hypot(body["x"] - cart["x"],
                          body["y"] - cart["y"]) > (tug_r + cart_r + broad_m):
                continue
            own = cart["towed_by"] == name
            out.append(Sample(
                t, sim, "tug_vs_own_tow" if own else "tug_vs_cart",
                name, cdef, obb_clearance(body["poly"], cart["poly"]),
                body["x"], body["y"], cart["x"], cart["y"]))


def category_stats(samples: List[Sample]) -> Dict[str, Any]:
    per: Dict[str, List[Sample]] = {c: [] for c in CATEGORIES}
    for s in samples:
        per[s.category].append(s)
    out: Dict[str, Any] = {}
    for cat, rows in per.items():
        if not rows:
            out[cat] = {"n": 0, "min_m": None, "p1_m": None, "median_m": None,
                        "at": None}
            continue
        vals = [r.clearance for r in rows]
        worst = min(rows, key=lambda r: r.clearance)
        out[cat] = {
            "n": len(rows),
            "min_m": round(min(vals), 5),
            "p1_m": round(percentile(vals, 1.0), 5),
            "median_m": round(statistics.median(vals), 5),
            "at": worst.as_json(),
        }
    return out


def excursion_runs(samples: List[Sample], warn_m: float,
                   period: float) -> List[Dict[str, Any]]:
    """Contiguous stretches where the frame minimum sat below `warn_m`.

    Grouped per category so a long slow pass down an aisle reads as one event
    rather than 300 rows. A gap of more than 3 poll periods closes a run.
    """
    by_cat: Dict[str, Dict[float, Sample]] = {}
    for s in samples:
        if s.clearance >= warn_m:
            continue
        cur = by_cat.setdefault(s.category, {})
        prev = cur.get(s.t)
        if prev is None or s.clearance < prev.clearance:
            cur[s.t] = s

    runs: List[Dict[str, Any]] = []
    for cat, rows in by_cat.items():
        ordered = [rows[k] for k in sorted(rows)]
        cur: List[Sample] = []
        for s in ordered:
            if cur and (s.t - cur[-1].t) > 3.0 * period:
                runs.append(_run_row(cat, cur))
                cur = []
            cur.append(s)
        if cur:
            runs.append(_run_row(cat, cur))
    runs.sort(key=lambda r: r["min_m"])
    return runs


def _run_row(cat: str, rows: List[Sample]) -> Dict[str, Any]:
    worst = min(rows, key=lambda r: r.clearance)
    return {
        "category": cat,
        "t_start_s": round(rows[0].t, 2),
        "t_end_s": round(rows[-1].t, 2),
        "duration_s": round(rows[-1].t - rows[0].t, 2),
        "samples": len(rows),
        "min_m": round(worst.clearance, 5),
        "worst": worst.as_json(),
    }


# ══════════════════════════════════════════════════════════════════════
# Report
# ══════════════════════════════════════════════════════════════════════

HONESTY = [
    "The OMNITUG500 tugs are KINEMATIC: no collider, no mass, no physics body. "
    "These clearances are a GEOMETRIC audit of where the navigation layer put "
    "the vehicle. They are NOT a physics result and must never be quoted as "
    "'the physics prevented a collision' -- nothing here could have stopped "
    "the tug.",
    "Obstacles include VISUAL-ONLY props (most of this world's bodies are "
    "non-collidable) on purpose. The demo's claim is that it never LOOKS like "
    "it clips anything, so a prop with no boundingObject is still a thing the "
    "tug must not drive through on camera.",
    "Obstacle rectangles are world-space AXIS-ALIGNED envelopes; the moving "
    "bodies are ORIENTED (SAT). For a rotated obstacle the envelope is "
    "conservative, so clearance against it is under-stated, never "
    "over-stated.",
    "Static obstacle geometry is captured ONCE, at startup. Bodies that move "
    "during the run are excluded from it by DEF pattern and tracked live from "
    "bridge telemetry instead; see `excluded_mobile` and `not_measured`.",
]


def build_report(args, geom, obs_info, obstacles, dropped, samples,
                 frames_n, recs, period, warnings, run_meta) -> Dict[str, Any]:
    # THE HEADLINE AND THE GATE EXCLUDE `tug_vs_own_tow`.
    #
    # A tug and the cart it is towing are joined by a drawbar: they are close
    # on purpose, always, and on a tight turn the 2D footprints can legally
    # touch. Folding that pair into the headline would replace the real
    # signal with a constant, and folding it into the gate would fail a demo
    # rehearsal for a designed condition. It is still measured, still in the
    # category table, and an actual OVERLAP is still surfaced -- as its own
    # number, under its own name, because it is a different claim.
    gated = [s for s in samples if s.category != "tug_vs_own_tow"]
    intersections = sorted([s for s in gated if s.clearance < 0.0],
                           key=lambda s: s.clearance)
    own_tow_overlaps = sorted(
        [s for s in samples
         if s.category == "tug_vs_own_tow" and s.clearance < 0.0],
        key=lambda s: s.clearance)
    overall = min(gated, key=lambda s: s.clearance) if gated else None
    unchecked = obs_info.get("unchecked", [])

    not_measured = [
        "Carts that are not currently under tow. The bridges publish only the "
        "cart they are towing, so parked, conveyor-borne and staged trolleys "
        "have no live pose and are neither obstacles nor measured bodies.",
        "The OMNIARM6 pick cell. The arm is a URDFRobot whose links move, so a "
        "startup snapshot of it would be a stale obstacle; it is excluded by "
        "the mobile-DEF rule and the bridge publishes no world-space link "
        "poses to replace it. Clearance between a tug and the arm or its "
        "pedestal is therefore NOT measured. (`--mobile-prefix` cannot add it "
        "back; the navigator handles static-base machines with a declared "
        "keep-out box instead, and this tool has no equivalent yet.)",
        "Vertical clearance. Every number is a 2D separation in the XY plane, "
        "gated by a z-overlap test; it is not a 3D distance.",
        "Anything between polls. At {:.0f} Hz a tug at 1 m/s moves {:.0f} mm "
        "between samples, so a transient closer approach can fall between two "
        "readings.".format(args.hz, 1000.0 / max(args.hz, 1e-9)),
    ]
    if unchecked:
        not_measured.insert(0, (
            "{} obstacle node(s) whose geometry could not be resolved -- "
            "listed by name in `obstacles.unchecked`. Clearance against those "
            "shapes is NOT included in any number below."
        ).format(len(unchecked)))

    yaw_missing = {n: r.yaw_missing for n, r in recs.items()
                   if n != "omniarm6" and r.yaw_missing}
    if yaw_missing:
        not_measured.insert(0, (
            "Samples where a bridge published no yaw: {}. An oriented "
            "footprint cannot be built without a heading and no axis-aligned "
            "substitute was used; those samples are excluded."
        ).format(yaw_missing))

    return {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(
            timespec="seconds"),
        "verdict": {
            "intersections": len(intersections),
            "min_clearance_m": None if overall is None else round(
                overall.clearance, 5),
            "min_pair": None if overall is None else overall.as_json(),
            "warn_threshold_m": args.warn_m,
            "excursions_below_warn": len([s for s in gated
                                          if s.clearance < args.warn_m]),
            "own_tow_overlaps": len(own_tow_overlaps),
            "own_tow_overlap_worst": (own_tow_overlaps[0].as_json()
                                      if own_tow_overlaps else None),
            "gate": "intersections counts every category EXCEPT "
                    "tug_vs_own_tow; a tug overlapping the cart it tows is "
                    "reported as own_tow_overlaps and does not fail the gate",
        },
        "what_this_is": HONESTY,
        "not_measured": not_measured,
        "run": run_meta,
        "geometry": {
            "tug_footprint_m": [geom["tug_len"], geom["tug_wid"],
                                geom["tug_hgt"]],
            "cart_footprint_m": [geom["cart_len"], geom["cart_wid"],
                                 geom["cart_hgt"]],
            "footprint_provenance":
                "OMNITUG500: union of its 8 STL meshes, mesh centred on origin in "
                "X/Y, bottom flush at z=0, local +Y forward. Trolley: the "
                "0.70 x 0.70 x 0.325 m collider. Published /state.yaw is the "
                "heading, so the long axis lies along it.",
            "method": "separating-axis test on oriented rectangles, ported "
                      "from omnilink_mobile_bridge._obb_clearance",
            "broad_phase_m": args.broad_m,
            "drive_over_z_m": args.drive_over_z,
            "body_top_z_m": geom["body_top_z"],
        },
        "obstacles": {
            "source": obs_info["source"],
            "source_detail": obs_info["detail"],
            "source_trust": (
                "harness: live scene graph -- sees meshes, PROTO expansions "
                "and URDF geometry. Poses are those of the world THAT harness "
                "has loaded, which is only the running demo if the harness is "
                "driving it; immobile structure is identical either way."
                if obs_info["source"] == "harness" else
                "static .wbt parse -- WEAKER. Sees only Box/Cylinder/Capsule/"
                "Sphere/Plane primitives written literally in the file. No "
                "PROTO expansion, no mesh reads, no URDFRobot interiors, so it "
                "UNDER-counts obstacles and therefore OVER-states clearance. "
                "Everything it could not resolve is in `unchecked`."),
            "counted": len(obstacles),
            "granularity": obs_info.get("granularity"),
            "unchecked": unchecked,
            "excluded_mobile": obs_info.get("excluded_mobile", []),
            "dropped_by_z_rule": {
                k: {"n": len(v), "names": sorted(set(v))}
                for k, v in dropped.items()
            },
            "cross_check": obs_info.get("cross_check"),
        },
        "intersections": [s.as_json() for s in intersections[:50]],
        "own_tow_overlaps": [s.as_json() for s in own_tow_overlaps[:20]],
        "categories": category_stats(samples),
        "excursions": excursion_runs(gated, args.warn_m, period),
        "sampling": {
            "frames": frames_n,
            "pair_measurements": len(samples),
            "hz": args.hz,
            "poll_latency_ms": {
                n: (round(1000.0 * statistics.fmean(r.latencies), 2)
                    if r.latencies else None)
                for n, r in recs.items()
            },
            "failed_polls": {n: len(r.failures) for n, r in recs.items()},
            "skipped": run_meta.get("skipped", {}),
        },
        "warnings": warnings,
    }


def _f(v, spec=".3f", dash="   -  "):
    if v is None:
        return dash
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return dash


def print_summary(rep: Dict[str, Any], out=sys.stdout) -> None:
    # ASCII only, and written through an encoding-safe wrapper. On Windows a
    # piped stdout is cp1252, and a box-drawing character in the banner
    # crashed the whole summary with a UnicodeEncodeError AFTER the JSON had
    # been written -- i.e. the run succeeded and the operator saw a traceback.
    def w(s: str) -> None:
        try:
            out.write(s)
        except UnicodeEncodeError:
            enc = getattr(out, "encoding", None) or "ascii"
            out.write(s.encode(enc, "replace").decode(enc, "replace"))

    v = rep["verdict"]
    w("\n")
    w("=" * 72 + "\n")
    w("  WAREHOUSE CLEARANCE - geometric audit of the navigation layer\n")
    w("=" * 72 + "\n")

    if v["intersections"]:
        w("\n")
        w("!" * 72 + "\n")
        w(f"  INTERSECTION: {v['intersections']} pair-sample(s) had NEGATIVE "
          f"clearance.\n")
        w("  A negative value is overlap, not a near miss. Full detail:\n")
        w("!" * 72 + "\n")
        for row in rep["intersections"][:10]:
            w(f"    {row['clearance_m']:+8.4f} m  {row['category']:<15s} "
              f"{row['a']} <-> {row['b']}\n")
            w(f"              t={row['t_wall_s']:.2f}s "
              f"sim={_f(row['sim_time_s'], '.2f')}s  "
              f"at ({row['a_xy'][0]:+.2f}, {row['a_xy'][1]:+.2f}) "
              f"vs ({row['b_xy'][0]:+.2f}, {row['b_xy'][1]:+.2f})\n")
        if len(rep["intersections"]) > 10:
            w(f"    ... {len(rep['intersections']) - 10} more in the JSON\n")

    w("\n  WHAT THIS IS\n")
    w("    The tugs have NO physics body. This measures where the NAVIGATOR\n")
    w("    put them, geometrically. It is not evidence about contact\n")
    w("    handling and cannot be quoted as 'the physics stopped it'.\n")

    o = rep["obstacles"]
    w(f"\n  OBSTACLE SOURCE: {o['source']}\n")
    w(f"    {o['source_detail']}\n")
    w(f"    counted {o['counted']} rectangle(s); "
      f"{len(o['unchecked'])} node(s) UNCHECKED; "
      f"{len(o['excluded_mobile'])} mobile DEF(s) excluded\n")
    if o["source"] == "world":
        w("    NOTE: a static parse under-counts obstacles, so every "
          "clearance\n          below is an OPTIMISTIC bound.\n")
    if o["unchecked"]:
        w("    UNCHECKED -- NO clearance was computed against these shapes:\n")
        grouped: Dict[str, List[str]] = {}
        for row in o["unchecked"]:
            grouped.setdefault(row["reason"], []).append(row["name"])
        for reason, names in grouped.items():
            w(f"      {len(names)} node(s): {reason}\n")
            shown = ", ".join(names[:8])
            more = "" if len(names) <= 8 else f", ... (+{len(names) - 8})"
            w(f"        {shown}{more}\n")

    w("\n  HEADLINE\n")
    if v["min_pair"] is None:
        w("    no pair was ever measurable — nothing to report\n")
    else:
        p = v["min_pair"]
        w(f"    minimum clearance   {v['min_clearance_m']:.4f} m "
          f"({v['min_clearance_m'] * 100:.1f} cm)\n")
        w(f"    between             {p['a']}  <->  {p['b']}"
          f"   [{p['category']}]\n")
        w(f"    when                t = {p['t_wall_s']:.2f} s wall, "
          f"sim = {_f(p['sim_time_s'], '.2f')} s\n")
        w(f"    where               body at ({p['a_xy'][0]:+.2f}, "
          f"{p['a_xy'][1]:+.2f}), other at ({p['b_xy'][0]:+.2f}, "
          f"{p['b_xy'][1]:+.2f})\n")

    w("\n  PER CATEGORY                      n       min        p1    median\n")
    w("  " + "-" * 62 + "\n")
    dash = "       -"
    for cat in CATEGORIES:
        c = rep["categories"][cat]
        tag = cat + ("  (diagnostic)" if cat == "tug_vs_own_tow" else "")
        w(f"  {tag:<28s} {c['n']:>6d}  {_f(c['min_m'], '8.4f', dash)} "
          f"{_f(c['p1_m'], '8.4f', dash)} {_f(c['median_m'], '8.4f', dash)}\n")
    w("\n    tug_vs_own_tow is the DRAWBAR gap to the cart that tug is towing.\n")
    w("    It is close by design, so it is excluded from the headline and from\n")
    w("    the exit-code gate. Read it as articulation, not as a near miss.\n")
    if v.get("own_tow_overlaps"):
        ot = v["own_tow_overlap_worst"]
        w(f"    NOTE: the tug and its own cart OVERLAPPED in "
          f"{v['own_tow_overlaps']} sample(s), worst "
          f"{ot['clearance_m']:+.4f} m at t={ot['t_wall_s']:.1f}s "
          f"({ot['a']} / {ot['b']}).\n")
        w("          That is a jackknife-guard question, not a collision "
          "one; it does not fail the gate.\n")

    ex = rep["excursions"]
    w(f"\n  EXCURSIONS below {v['warn_threshold_m']:.2f} m: {len(ex)} run(s)\n")
    if ex:
        w("    min_m    dur_s  n     category         pair\n")
        for r in ex[:15]:
            wr = r["worst"]
            w(f"    {r['min_m']:+7.4f} {r['duration_s']:7.2f} "
              f"{r['samples']:>4d}  {r['category']:<15s} "
              f"{wr['a']} <-> {wr['b']}  @t={wr['t_wall_s']:.1f}s\n")
        if len(ex) > 15:
            w(f"    ... {len(ex) - 15} more in the JSON\n")

    w("\n  NOT MEASURED\n")
    for line in rep["not_measured"]:
        w(f"    - {line}\n")

    s = rep["sampling"]
    w(f"\n  SAMPLING  frames={s['frames']} pairs={s['pair_measurements']} "
      f"hz={s['hz']}\n")
    if any(s["failed_polls"].values()):
        w(f"    failed polls: {s['failed_polls']}\n")
    if s.get("skipped"):
        w(f"    skipped: {s['skipped']}\n")
    for line in rep.get("warnings", []):
        w(f"  ! {line}\n")
    w("\n")


# ══════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════

def _footprint(text: str, what: str) -> Tuple[float, float, float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"{what} needs 'length,width,height' in metres, got {text!r}")
    try:
        vals = tuple(float(p) for p in parts)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{what}: non-numeric in {text!r}")
    if any(v <= 0 for v in vals):
        raise argparse.ArgumentTypeError(f"{what}: extents must be positive")
    return vals            # type: ignore[return-value]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="clearance_monitor.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Measure minimum separation in the warehouse demo from "
                    "outside the control loop. GET-only; writes only --out.",
        epilog="The tugs are kinematic: this is a GEOMETRIC audit of the "
               "navigation layer, not a physics result.")
    p.add_argument("--tug-a-url", default="http://127.0.0.1:8766")
    p.add_argument("--tug-b-url", default="http://127.0.0.1:8767")
    p.add_argument("--arm-url", default="http://127.0.0.1:8765")
    p.add_argument("--no-arm", action="store_true",
                   help="do not poll the OMNIARM6 bridge (skips the "
                        "'is this the warehouse world' check)")
    p.add_argument("--duration", type=float, default=600.0)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--token", default=os.environ.get("OMNILINK_TOKEN", ""))
    p.add_argument("--max-consecutive-failures", type=int, default=5)
    p.add_argument("--source", choices=("auto", "harness", "world"),
                   default="auto")
    p.add_argument("--harness", default="http://127.0.0.1:6789")
    p.add_argument("--world", default=None,
                   help="path to a .wbt for the static-parse fallback")
    p.add_argument("--warn-m", type=float, default=0.15)
    p.add_argument("--broad-m", type=float, default=3.0,
                   help="only pairs within this margin of contact are "
                        "recorded (default 3.0 m)")
    p.add_argument("--drive-over-z", type=float, default=DRIVE_OVER_Z)
    p.add_argument("--overhead-z", type=float, default=None,
                   help="treat obstacles whose bottom is at or above this as "
                        "overhead (default: the body's own height)")
    p.add_argument("--tug-footprint", default=None,
                   help=f"L,W,H in metres (default "
                        f"{TUG_LEN_M},{TUG_WID_M},{TUG_HGT_M})")
    p.add_argument("--cart-footprint", default=None,
                   help=f"L,W,H in metres (default "
                        f"{CART_LEN_M},{CART_WID_M},{CART_HGT_M})")
    p.add_argument("--mobile-prefix", action="append", default=[],
                   help="extra DEF prefix to treat as a moving body "
                        "(repeatable)")
    p.add_argument("--out", default=None)
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--selftest", action="store_true",
                   help="validate the geometry and the whole pipeline against "
                        "synthetic data and a fake bridge; no simulator")
    return p


def resolve_obstacles(args, prefixes, suffixes, warnings) -> Dict[str, Any]:
    """Pick a source and say which one was used. They are not equally good."""
    harness_info = None
    harness_err = None
    if args.source in ("auto", "harness"):
        try:
            harness_info = obstacles_from_harness(
                args.harness, max(args.timeout, 5.0), args.token,
                prefixes, suffixes)
        except (BridgeError, OSError, ValueError) as e:
            harness_err = str(e)
            if args.source == "harness":
                raise BridgeError(
                    f"--source harness, but {args.harness} did not answer "
                    f"/scene/tree?bounds=1: {e}\n"
                    f"  Start it with: python -m omnisim harness\n"
                    f"  Or fall back to: --source world --world <path.wbt>")

    world_info = None
    world_err = None
    if args.world and (args.source == "world" or harness_info is None
                       or args.source == "auto"):
        try:
            world_info = obstacles_from_world(args.world, prefixes, suffixes)
        except (OSError, ValueError) as e:
            world_err = str(e)
            if args.source == "world":
                raise BridgeError(f"--world {args.world}: {e}")

    if harness_info is not None:
        if harness_err:
            warnings.append(f"harness: {harness_err}")
        if world_info is not None:
            harness_info["cross_check"] = {
                "static_parse_of": args.world,
                "static_rectangles": len(world_info["obstacles"]),
                "harness_rectangles": len(harness_info["obstacles"]),
                "note": "counts differ because the static parse cannot see "
                        "meshes, PROTO expansions or URDF interiors. The "
                        "harness figure is the one used.",
            }
        return harness_info

    if world_info is not None:
        if harness_err and args.source == "auto":
            warnings.append(
                f"no harness at {args.harness} ({harness_err.splitlines()[0]})"
                f" -- fell back to the static .wbt parse, which UNDER-counts "
                f"obstacles and therefore OVER-states clearance")
        return world_info

    raise BridgeError(
        "no obstacle source. Either start the harness "
        f"({args.harness}) with `python -m omnisim harness`, or pass "
        f"--world <path.wbt>."
        + (f"\n  harness said: {harness_err}" if harness_err else "")
        + (f"\n  world said:   {world_err}" if world_err else ""))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    if args.hz <= 0 or args.duration <= 0:
        print("--hz and --duration must be positive", file=sys.stderr)
        return EXIT_ARGS
    if args.broad_m < 0 or args.warn_m < 0:
        print("--broad-m and --warn-m must be >= 0", file=sys.stderr)
        return EXIT_ARGS

    try:
        tug = (_footprint(args.tug_footprint, "--tug-footprint")
               if args.tug_footprint else (TUG_LEN_M, TUG_WID_M, TUG_HGT_M))
        cart = (_footprint(args.cart_footprint, "--cart-footprint")
                if args.cart_footprint
                else (CART_LEN_M, CART_WID_M, CART_HGT_M))
    except argparse.ArgumentTypeError as e:
        print(str(e), file=sys.stderr)
        return EXIT_ARGS

    body_top_z = (args.overhead_z if args.overhead_z is not None
                  else max(tug[2], cart[2]))
    geom = {"tug_len": tug[0], "tug_wid": tug[1], "tug_hgt": tug[2],
            "cart_len": cart[0], "cart_wid": cart[1], "cart_hgt": cart[2],
            "body_top_z": body_top_z}
    prefixes = tuple(DEFAULT_MOBILE_PREFIXES) + tuple(
        p.upper() for p in args.mobile_prefix)
    warnings: List[str] = []

    # ── obstacles ────────────────────────────────────────────────────
    try:
        obs_info = resolve_obstacles(args, prefixes, DEFAULT_MOBILE_SUFFIXES,
                                     warnings)
    except BridgeError as e:
        print(f"\nno obstacle geometry:\n{e}\n", file=sys.stderr)
        return EXIT_NO_OBSTACLES
    obstacles, dropped = filter_obstacles(obs_info["obstacles"],
                                          args.drive_over_z, body_top_z)
    if not obstacles:
        warnings.append("the obstacle set is EMPTY after the z rules -- "
                        "tug_vs_static and cart_vs_static will be blank")
    if not args.quiet:
        print(f"obstacles: {len(obstacles)} rectangle(s) from "
              f"{obs_info['source']} "
              f"({len(obs_info['unchecked'])} unchecked, "
              f"{len(dropped['driven_over'])} driven over, "
              f"{len(dropped['overhead'])} overhead)", file=sys.stderr)

    # ── bridges ──────────────────────────────────────────────────────
    recs: Dict[str, PoseRecorder] = {
        "tug_a": PoseRecorder("tug_a", args.tug_a_url.rstrip("/")),
        "tug_b": PoseRecorder("tug_b", args.tug_b_url.rstrip("/")),
    }
    if not args.no_arm:
        recs["omniarm6"] = PoseRecorder("omniarm6", args.arm_url.rstrip("/"))
    try:
        warnings.extend(preflight(recs, args.timeout, args.token))
        problems = check_yaw_published(recs, args.timeout, args.token)
        if problems:
            raise BridgeError("bridge(s) do not publish a heading:\n"
                              + "\n".join(problems))
    except BridgeError as e:
        print(f"\npreflight failed:\n{e}\n", file=sys.stderr)
        return EXIT_PREFLIGHT

    if not args.quiet:
        print(f"polling {len(recs)} bridge(s) for {args.duration:.0f}s at "
              f"{args.hz} Hz (GET only)", file=sys.stderr)
    t_start = time.time()
    ok, err = collect(recs, args.duration, args.hz, args.timeout, args.token,
                      args.max_consecutive_failures, args.quiet)
    if not ok:
        warnings.append(f"run truncated: {err}")

    # ── evaluate ─────────────────────────────────────────────────────
    period = 1.0 / args.hz
    frames = merge_frames(recs, period)
    samples: List[Sample] = []
    skipped: Dict[str, int] = {}
    for fr in frames:
        evaluate_frame(fr, obstacles, geom, args.broad_m, samples, skipped)

    run_meta = {
        "started_utc": datetime.fromtimestamp(t_start, timezone.utc)
                               .isoformat(timespec="seconds"),
        "duration_requested_s": args.duration,
        "completed": bool(ok),
        "bridges": {n: r.url for n, r in recs.items()},
        "skipped": skipped,
    }
    rep = build_report(args, geom, obs_info, obstacles, dropped, samples,
                       len(frames), recs, period, warnings, run_meta)

    if args.out:
        d = os.path.dirname(os.path.abspath(args.out))
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, indent=2, sort_keys=False)
            fh.write("\n")
        if not args.quiet:
            print(f"wrote {args.out}", file=sys.stderr)
    if not args.quiet:
        print_summary(rep)

    if rep["verdict"]["intersections"]:
        return EXIT_INTERSECTION
    if not ok:
        return EXIT_BRIDGE_DIED
    return EXIT_OK


# ══════════════════════════════════════════════════════════════════════
# --selftest: geometry unit tests + a full pipeline run against a fake
# bridge.  No simulator, no GPU, no network beyond loopback.
# ══════════════════════════════════════════════════════════════════════

def _close(a, b, tol=1e-6, what=""):
    if a is None or abs(a - b) > tol:
        raise AssertionError(f"{what}: {a!r} != {b!r} (tol {tol})")


def _t_touching():
    """Edge-to-edge contact reads as exactly zero, and is NOT an
    intersection. The boundary matters: the gate fires on < 0."""
    A = rect(0, 0, 0.5, 0.5, 0.0)
    B = rect(1.0, 0, 0.5, 0.5, 0.0)
    d = obb_clearance(A, B)
    _close(d, 0.0, 1e-12, "touching")
    assert not (d < 0.0), "touching must not be reported as an intersection"


def _t_overlapping():
    """Penetration depth is returned as a negative number."""
    A = rect(0, 0, 0.5, 0.5, 0.0)
    B = rect(0.8, 0, 0.5, 0.5, 0.0)
    _close(obb_clearance(A, B), -0.2, 1e-12, "overlap depth")
    assert obb_clearance(A, B) < 0.0


def _t_disjoint_corner():
    """Corner-to-corner disjoint pair -> true Euclidean min distance."""
    A = rect(0, 0, 0.5, 0.5, 0.0)
    B = rect(2, 2, 0.5, 0.5, 0.0)
    _close(obb_clearance(A, B), math.hypot(1.0, 1.0), 1e-12, "corner gap")


def _t_symmetry_and_self():
    A = rect(0.3, -1.2, 0.62975, 0.3581, 0.7)
    B = rect(1.9, 0.4, 0.35, 0.35, -0.2)
    _close(obb_clearance(A, B), obb_clearance(B, A), 1e-12, "symmetry")
    assert obb_clearance(A, A) < 0.0, "a box must intersect itself"


def _t_aabb_vs_obb_divergence():
    """THE CASE THAT JUSTIFIES THE WHOLE APPROACH.

    A OMNITUG500-sized footprint at a 27 deg heading beside a 0.6 m corner post.
    The axis-aligned envelope of the tug overlaps the post; the true oriented
    footprint is 0.10999836 m clear. Both numbers are hand-computed below.
    This is the same failure mode recorded in
    `MobileBridge._obb_vs_aabb` ("it cost a whole measurement run"): an
    AABB monitor reports a phantom incursion the vehicle never had.
    """
    ang = math.radians(27.0)
    hx, hy = TUG_LEN_M / 2.0, TUG_WID_M / 2.0          # 0.62975, 0.3581
    tug = rect(0.0, 0.0, hx, hy, ang)
    post = [(1.15, 1.15), (1.15, 0.55), (0.55, 0.55), (0.55, 1.15)]

    # Hand computation. The nearest feature is the post's (0.55, 0.55) corner
    # against the tug's forward-left edge, whose outward normal is the heading
    # unit vector u = (cos27, sin27). The tug's extent along u is exactly its
    # half-length, so the gap is u.(0.55, 0.55) - hx.
    u = (math.cos(ang), math.sin(ang))
    expected = (0.55 * u[0] + 0.55 * u[1]) - hx
    _close(expected, 0.1099983631603531, 1e-12, "hand-computed expectation")

    d_obb = obb_clearance(tug, post)
    _close(d_obb, expected, 1e-9, "OBB clearance")

    d_aabb = aabb_clearance(tug, post)
    assert d_aabb < 0.0, (
        f"the AABB shortcut was expected to report overlap, got {d_aabb}")
    assert abs(d_obb - d_aabb) > 0.1, (
        f"AABB and OBB must DISAGREE here: obb={d_obb} aabb={d_aabb}")
    # The tug's axis-aligned envelope really does reach into the post.
    ax1 = max(p[0] for p in tug)
    ay1 = max(p[1] for p in tug)
    assert ax1 > 0.55 and ay1 > 0.55, "envelope should overlap the post"


def _t_heading_convention():
    """The LONG axis must lie along the published yaw.

    Get this 90 deg wrong and every clearance in the report is wrong. At
    yaw=0 the footprint must be 1.2595 m in X and 0.7162 m in Y.
    """
    p = rect(0, 0, TUG_LEN_M / 2.0, TUG_WID_M / 2.0, 0.0)
    _close(max(q[0] for q in p) - min(q[0] for q in p), TUG_LEN_M, 1e-12,
           "x extent at yaw=0")
    _close(max(q[1] for q in p) - min(q[1] for q in p), TUG_WID_M, 1e-12,
           "y extent at yaw=0")
    p90 = rect(0, 0, TUG_LEN_M / 2.0, TUG_WID_M / 2.0, math.pi / 2)
    _close(max(q[0] for q in p90) - min(q[0] for q in p90), TUG_WID_M, 1e-9,
           "x extent at yaw=90")


def _t_wbt_parse():
    """The static parser must resolve a transformed primitive exactly."""
    text = """#VRML_SIM R2025a utf8
DEF W Solid {
  translation 2.5 0 0.5
  children [ Shape { geometry Box { size 1 2 1 } } ]
  name "w"
}
DEF SPUN Solid {
  translation 0 0 0.5
  children [
    Transform { rotation 0 0 1 0.7853981633974483
      children [ Shape { geometry Box { size 2 2 1 } } ] }
  ]
}
"""
    import tempfile
    fd, path = tempfile.mkstemp(suffix=".omniworld")
    os.close(fd)
    try:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        info = obstacles_from_world(path, ("MAV_",), ())
        by = {o.label: o for o in info["obstacles"]}
        w = by["W"]
        _close(w.x0, 2.0, 1e-9, "wall x0")
        _close(w.x1, 3.0, 1e-9, "wall x1")
        _close(w.y0, -1.0, 1e-9, "wall y0")
        _close(w.z0, 0.0, 1e-9, "wall z0")
        _close(w.z1, 1.0, 1e-9, "wall z1")
        # A 2x2 box spun 45 deg has an envelope of 2*sqrt(2) on a side.
        s = by["SPUN"]
        _close(s.x1 - s.x0, 2.0 * math.sqrt(2.0), 1e-9, "spun envelope")
    finally:
        os.unlink(path)


def _t_filters():
    obs = [
        Obstacle("DECAL", None, -1, -1, 1, 1, 0.0, 0.004),
        Obstacle("BEAM", None, -1, -1, 1, 1, 2.0, 2.4),
        Obstacle("POST", None, -1, -1, 1, 1, 0.0, 1.2),
        Obstacle("FLAT", None, 0, 0, 0, 0, 0.0, 1.0),
    ]
    keep, dropped = filter_obstacles(obs, 0.06, 0.325)
    assert [o.label for o in keep] == ["POST"], [o.label for o in keep]
    assert dropped["driven_over"] == ["DECAL"]
    assert dropped["overhead"] == ["BEAM"]
    assert dropped["degenerate"] == ["FLAT"]


# ── fake bridge ──────────────────────────────────────────────────────

def _fake_bridge(traj: List[Tuple[float, float, float]], robot_id: str,
                 tow_def: Optional[str] = None,
                 tow_offset: Tuple[float, float] = (-0.9, 0.0)):
    """A loopback HTTP server that answers GET /state from a script.

    Index-advancing, then holding the final pose, so the measured minimum is
    a property of the trajectory and not of how many times it was polled.
    """
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    state = {"i": 0}
    lock = threading.Lock()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):        # silence
            pass

        def do_GET(self):                 # noqa: N802
            if self.path.startswith("/capabilities"):
                body = json.dumps([{"capabilities": ["drive"]}]).encode()
            elif self.path.startswith("/state"):
                with lock:
                    i = min(state["i"], len(traj) - 1)
                    state["i"] += 1
                x, y, yaw = traj[i]
                payload = {
                    "id": robot_id, "model": "OMNITUG500",
                    "x": x, "y": y, "yaw": yaw,
                    "sim_time": 0.1 * i,
                    "idle_loop": {"leg": "selftest"},
                }
                if tow_def:
                    payload["carrying"] = tow_def
                    payload["towed"] = {
                        "def": tow_def,
                        "x": x + tow_offset[0], "y": y + tow_offset[1],
                        "yaw": yaw, "artic_deg": 0.0,
                    }
                body = json.dumps(payload).encode()
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}"


_SELFTEST_WORLD = """#VRML_SIM R2025a utf8
DEF WALL Solid {
  translation 2.5 0 0.5
  children [ Shape { geometry Box { size 1 2 1 } } ]
  name "wall"
}
DEF FLOOR_DECAL Solid {
  translation 0 0 0.002
  children [ Shape { geometry Box { size 8 8 0.004 } } ]
  name "decal"
}
DEF ROOF_BEAM Solid {
  translation 0 0 2.2
  children [ Shape { geometry Box { size 8 0.2 0.4 } } ]
  name "beam"
}
DEF MESH_PROP Solid {
  translation 0 -3 0.5
  children [ Shape { geometry IndexedFaceSet { } } ]
  name "mesh_prop"
}
"""


def _pipeline(traj_a, out_path, world_path, extra=None,
              tow_def=None, tow_offset=(-0.9, 0.0)) -> Tuple[int, dict]:
    srv_a, url_a = _fake_bridge(traj_a, "tug_a", tow_def, tow_offset)
    # tug_b parks at (0, 4): close enough that the --broad-m 3.0 window keeps
    # the tug-vs-tug pair (centre distance <= 2*0.7244 + 3.0 = 4.449 m), which
    # is what makes its clearance assertable at all.
    srv_b, url_b = _fake_bridge([(0.0, 4.0, 0.0)], "tug_b")
    try:
        argv = [
            "--source", "world", "--world", world_path,
            "--tug-a-url", url_a, "--tug-b-url", url_b, "--no-arm",
            "--duration", "2.0", "--hz", "20", "--quiet",
            "--out", out_path,
        ] + list(extra or [])
        code = main(argv)
        with open(out_path, "r", encoding="utf-8") as fh:
            return code, json.load(fh)
    finally:
        srv_a.shutdown()
        srv_b.shutdown()


def _t_pipeline_clear():
    """End-to-end against a scripted trajectory with a hand-computed answer.

    tug_a drives +X along y=0 from x=0 to x=1.0 at yaw=0. Its leading edge is
    at x + 1.2595/2. The wall starts at x=2. So the minimum clearance is
        2.0 - (1.0 + 0.62975) = 0.37025 m
    and it happens at the last waypoint. tug_b sits at (0, 4) with the same
    heading, so the two beams face each other and the tug-vs-tug gap is
        4.0 - 2*(0.7162/2) = 3.2838 m
    for every sample (their X spans overlap throughout).
    """
    import tempfile
    d = tempfile.mkdtemp()
    world = os.path.join(d, "selftest.wbt")
    with open(world, "w", encoding="utf-8") as fh:
        fh.write(_SELFTEST_WORLD)
    out = os.path.join(d, "clearance.json")
    traj = [(0.1 * i, 0.0, 0.0) for i in range(11)]
    code, rep = _pipeline(traj, out, world)

    assert code == EXIT_OK, f"expected clean exit, got {code}"
    assert rep["obstacles"]["source"] == "world", rep["obstacles"]["source"]
    _close(rep["verdict"]["min_clearance_m"], 0.37025, 1e-4,
           "pipeline min clearance")
    p = rep["verdict"]["min_pair"]
    assert p["category"] == "tug_vs_static", p
    assert p["a"] == "tug_a" and p["b"] == "WALL", p
    _close(rep["categories"]["tug_vs_tug"]["min_m"], 3.2838, 1e-3,
           "tug vs tug")
    assert rep["categories"]["tug_vs_cart"]["n"] == 0
    assert rep["verdict"]["intersections"] == 0

    # The z rules must have set aside the decal and the beam, by name.
    dz = rep["obstacles"]["dropped_by_z_rule"]
    assert "FLOOR_DECAL" in dz["driven_over"]["names"], dz
    assert "ROOF_BEAM" in dz["overhead"]["names"], dz
    # The unresolvable mesh must be NAMED, not silently skipped.
    names = " ".join(u["name"] for u in rep["obstacles"]["unchecked"])
    assert "MESH_PROP" in names, rep["obstacles"]["unchecked"]
    assert any("GEOMETRIC" in s for s in rep["what_this_is"])


def _t_pipeline_intersection():
    """Driving into the wall must be reported loudly and gate non-zero."""
    import tempfile
    d = tempfile.mkdtemp()
    world = os.path.join(d, "selftest.wbt")
    with open(world, "w", encoding="utf-8") as fh:
        fh.write(_SELFTEST_WORLD)
    out = os.path.join(d, "hit.json")
    traj = [(0.2 * i, 0.0, 0.0) for i in range(13)]     # runs to x = 2.4
    code, rep = _pipeline(traj, out, world)
    assert code == EXIT_INTERSECTION, f"expected 6, got {code}"
    assert rep["verdict"]["intersections"] > 0
    assert rep["verdict"]["min_clearance_m"] < 0
    worst = rep["intersections"][0]
    assert worst["a"] == "tug_a" and worst["b"] == "WALL", worst
    # Deepest overlap: leading edge at 2.4 + 0.62975 = 3.02975 vs wall
    # x in [2,3] -> the tug is through it; depth is capped by the wall's own
    # 1 m thickness, so assert the sign and the pair, not a fragile depth.
    assert len(rep["excursions"]) > 0


def _fake_harness(payload: dict):
    """Loopback server answering GET /scene/tree?bounds=1 with `payload`."""
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):                 # noqa: N802
            if not self.path.startswith("/scene/tree"):
                self.send_error(404)
                return
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}"


def _t_harness_source():
    """The PRIMARY obstacle source, against a synthetic /scene/tree.

    Pins the three behaviours the report depends on: Shape-level bounds are
    preferred over the owning node's coarser union, a mobile DEF is excluded
    rather than frozen at a stale pose, and a node with no usable bounds is
    reported as UNCHECKED by name instead of vanishing.
    """
    def bounds(x0, y0, z0, x1, y1, z1, exact=True):
        return {"bbox_min": [x0, y0, z0], "bbox_max": [x1, y1, z1],
                "center": [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2],
                "exact": exact}

    payload = {
        "bounds_included": True,
        "nodes": [
            {"type": "Group", "id": 0},
            # Union bounds span the whole rack; the two Shapes are the real
            # uprights, and using them is what keeps the aisle measurable.
            {"def": "RACK", "type": "Solid", "id": 1, "parent_def": None,
             "bounds": bounds(0, 0, 0, 4, 1, 2)},
            {"type": "Shape", "id": 2, "parent_def": "RACK",
             "bounds": bounds(0, 0, 0, 0.1, 1, 2)},
            {"type": "Shape", "id": 3, "parent_def": "RACK",
             "bounds": bounds(3.9, 0, 0, 4, 1, 2, exact=False)},
            {"def": "TROLLEY_B", "type": "Solid", "id": 4, "parent_def": None,
             "bounds": bounds(-1, -1, 0, -0.3, -0.3, 0.3)},
            {"def": "NO_BOUNDS", "type": "Solid", "id": 5,
             "parent_def": None},
        ],
    }
    srv, url = _fake_harness(payload)
    try:
        info = obstacles_from_harness(url, 5.0, "",
                                      DEFAULT_MOBILE_PREFIXES,
                                      DEFAULT_MOBILE_SUFFIXES)
    finally:
        srv.shutdown()

    assert info["source"] == "harness"
    names = sorted(o.name for o in info["obstacles"])
    assert names == ["RACK#0", "RACK#1"], names
    assert info["granularity"]["shape_level"] == 1, info["granularity"]
    # The coarse 4 m union must NOT have been used.
    widths = sorted(round(o.x1 - o.x0, 3) for o in info["obstacles"])
    assert widths == [0.1, 0.1], widths
    assert any(o.exact is False for o in info["obstacles"]), \
        "the `exact` flag must survive from the harness"
    assert info["excluded_mobile"] == ["TROLLEY_B"], info["excluded_mobile"]
    assert [u["name"] for u in info["unchecked"]] == ["NO_BOUNDS"], \
        info["unchecked"]

    # A harness that answers without bounds must be a hard error, not a
    # silently empty obstacle set.
    srv2, url2 = _fake_harness({"bounds_included": False, "nodes": []})
    try:
        obstacles_from_harness(url2, 5.0, "", DEFAULT_MOBILE_PREFIXES,
                               DEFAULT_MOBILE_SUFFIXES)
    except BridgeError:
        pass
    else:
        raise AssertionError("bounds_included=false must raise")
    finally:
        srv2.shutdown()


def _t_pipeline_towed_cart():
    """A towed cart is measured, and the DRAWBAR pair does not fail the gate.

    tug_a tows a cart pinned 0.5 m behind its centre — deliberately close
    enough that the two footprints OVERLAP (tug half-length 0.62975 vs a cart
    centred 0.5 m back with 0.35 m half-extent). That must be reported as
    `own_tow_overlaps`, must NOT be counted as an intersection, and must NOT
    change the exit code, because a tug touching the cart on its own drawbar
    is a jackknife question rather than a collision.

    Meanwhile the cart itself is measured against the world: at the final
    waypoint the tug is at x=1.0, so the cart is at x=0.5, its leading face is
    at 0.85, and the wall starts at x=2 -> cart_vs_static min = 1.15 m.
    """
    import tempfile
    d = tempfile.mkdtemp()
    world = os.path.join(d, "selftest.wbt")
    with open(world, "w", encoding="utf-8") as fh:
        fh.write(_SELFTEST_WORLD)
    out = os.path.join(d, "tow.json")
    traj = [(0.1 * i, 0.0, 0.0) for i in range(11)]
    code, rep = _pipeline(traj, out, world, tow_def="TROLLEY_X",
                          tow_offset=(-0.5, 0.0))

    assert code == EXIT_OK, f"own-tow overlap must not fail the gate: {code}"
    assert rep["verdict"]["intersections"] == 0, rep["verdict"]
    assert rep["verdict"]["own_tow_overlaps"] > 0, rep["verdict"]
    assert rep["categories"]["tug_vs_own_tow"]["min_m"] < 0
    # The headline must be a REAL pair, not the drawbar.
    assert rep["verdict"]["min_pair"]["category"] != "tug_vs_own_tow"
    _close(rep["categories"]["cart_vs_static"]["min_m"], 1.15, 1e-4,
           "cart vs wall")
    # The SAME cart is a CROSS pair for the other tug: tug_b does not tow it,
    # so tug_b <-> TROLLEY_X is an ordinary tug_vs_cart clearance and is NOT
    # given the drawbar exemption. tug_b sits at (0, 4) and the cart runs
    # along y=0, so the gap is 4 - 0.7162/2 - 0.70/2 = 3.2919 m.
    tc = rep["categories"]["tug_vs_cart"]
    assert tc["n"] > 0 and tc["at"]["a"] == "tug_b", tc
    assert tc["at"]["b"] == "TROLLEY_X", tc
    _close(tc["min_m"], 3.2919, 1e-4, "other tug vs the towed cart")
    # And the drawbar penetration depth is exact: (0.62975 + 0.35) - 0.5.
    _close(rep["categories"]["tug_vs_own_tow"]["min_m"], -0.47975, 1e-9,
           "drawbar penetration depth")


def selftest() -> int:
    tests = [
        ("sat_touching", _t_touching),
        ("sat_overlapping", _t_overlapping),
        ("sat_disjoint_corner", _t_disjoint_corner),
        ("sat_symmetry_and_self", _t_symmetry_and_self),
        ("aabb_vs_obb_divergence", _t_aabb_vs_obb_divergence),
        ("heading_convention", _t_heading_convention),
        ("wbt_static_parse", _t_wbt_parse),
        ("z_filters", _t_filters),
        ("harness_obstacle_source", _t_harness_source),
        ("pipeline_clear", _t_pipeline_clear),
        ("pipeline_intersection", _t_pipeline_intersection),
        ("pipeline_towed_cart_gate", _t_pipeline_towed_cart),
    ]
    failed = []
    print("clearance_monitor --selftest (no simulator, no GPU)")
    for name, fn in tests:
        t0 = time.monotonic()
        try:
            fn()
            print(f"  PASS  {name}  ({time.monotonic() - t0:.2f}s)")
        except Exception as e:                          # noqa: BLE001
            failed.append((name, e))
            print(f"  FAIL  {name}: {e}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    if failed:
        for name, e in failed:
            print(f"  {name}: {e!r}")
        return EXIT_SELFTEST
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
