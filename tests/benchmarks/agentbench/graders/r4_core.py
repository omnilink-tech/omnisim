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

"""R4 ``mobile_manipulation`` -- the **sim-neutral core** of the robotics tier.

Drive to a part, pick it up, carry it across the room, put it down. It is the
canonical warehouse job and it is what a mobile manipulator *is*. R1 asks
whether a robot can perceive and navigate; R3 asks whether a grip can hold a
part while an arm swings. R4 asks the question neither of them does: **can a
grip hold while the body holding it is driving** -- accelerating, braking and
yawing for several metres. A contact model that passes a static pinch and lets
go on the first turn fails here and passes R3.

    R4.1  the run is clean            exit 0, no error lines, driver completed
    R4.2  the specified scene is intact
                                      five obstacles matched to the GRADED
                                      layout by geometry, plus a table whose
                                      top and a pad whose top are where the
                                      frozen scene spec puts them
    R4.3  one articulated mobile manipulator
                                      exactly one robot-class body, >= 6 joints,
                                      and >= 3 of its link bodies TRANSLATE in
                                      the base frame (a wheel does not)
    R4.4  the payload started at rest on the table
                                      one body with the payload's t=0 cube
                                      geometry, dynamic, not part of the robot
    R4.5  the base drove the route without striking anything
                                      started at the declared start, reached
                                      the table then the pad in that order,
                                      zero contacts against obstacles or walls
    R4.6  the payload was lifted clear and CARRIED
                                      the grasp proof -- see below
    R4.7  it was held continuously     exactly one carry episode, ending at
                                      the pad: no drop, no re-acquisition
    R4.8  it was delivered and stayed  at rest on the pad for the last HOLD_S
    R4.9  nothing was teleported, nothing was structurally attached

**The grasp proof, and why it is three numbers and not a contact count.**
R4.6 asks for a contiguous window in which all of these hold at once:

* the payload's **lowest face** is ``CARRY_CLEARANCE_M`` above the floor --
  above the pad, and above anything else it could be resting on down there;
* the **distance from the payload to the robot's base origin** stays inside a
  ``CARRY_RADIUS_TOL_M`` band, and so does the **height difference** between
  them;
* the base travels ``TRANSPORT_MIN_M`` and the payload travels
  ``TRANSPORT_MIN_M`` during the window.

Nothing that is not being carried satisfies that conjunction. A payload parked
on top of an obstacle keeps its height and loses the radius the moment the base
drives on. One dragged along the floor keeps the radius and loses the height.
One that is dropped loses both within a handful of samples. And a payload that
was never picked up fails the lift clause, which is measured against the
MEASURED table top rather than the authored one.

The **radius** and the **height difference** are used rather than a vector
offset for a reason that is a property of the evidence and not a preference:
the neutral bundle carries positions and no orientations, and the base yaws
continuously while it drives, so a world-frame offset from a held payload to
its base is not constant even for a perfect grasp. Both quantities used here
are invariant to that yaw.

**What this core deliberately does not claim.** It cannot see which link is
the gripper, it cannot tell differential drive from mecanum, it cannot tell a
motorised joint from a passive one, and it cannot distinguish a runtime weld
between two independent dynamic bodies from a friction grasp. All four are
written down in ``meta.json`` (``not_expressible``, ``known_hole``) rather
than quietly implied to be covered.

Every threshold is READ from ``tasks/R4_mobile_manipulation/meta.json``'s
``constants`` block, every scene dimension from that task's
``benchmark_assets/scene.json``, and every obstacle size from its
``benchmark_assets/obstacles.json``. No number in this task is written down
twice. Everything here is metres and seconds, and nothing in this module knows
what a simulator is.
"""

from __future__ import annotations

import json
import math
import random
from pathlib import Path

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier, Verdict)

TASK = "R4_mobile_manipulation"

TASK_DIR = Path(__file__).resolve().parents[1] / "tasks" / TASK


class LayoutError(Exception):
    """No legal obstacle layout could be drawn. Never repaired, never faked."""


def _meta():
    """The task contract. Read, never duplicated."""
    return json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))


def scene_spec():
    """The frozen scene geometry, from the task's own shipped asset."""
    p = TASK_DIR / "initial" / "benchmark_assets" / "scene.json"
    return json.loads(p.read_text(encoding="utf-8"))


def obstacle_spec():
    """The PUBLISHED obstacle list, from the task's own shipped asset.

    What is contractually published is the **count and the sizes**; the
    positions here are the reference layout the agent authors against, and the
    layout a run is GRADED on is whatever :func:`resolve_graded_layout`
    returns.
    """
    p = TASK_DIR / "initial" / "benchmark_assets" / "obstacles.json"
    return json.loads(p.read_text(encoding="utf-8"))["obstacles"]


def obstacle_sizes(spec=None):
    """``[(name, (sx, sy, sz)), ...]`` -- the half of the layout that is
    published and is never redrawn."""
    return [(o["name"], tuple(float(v) for v in o["size"]))
            for o in (spec if spec is not None else obstacle_spec())]


# --- thresholds: read from meta.json ----------------------------------------

_C = _meta()["constants"]

START_XY = tuple(float(v) for v in _C["START_XY"])
START_YAW_DEG = float(_C["START_YAW_DEG"])
START_TOL_M = float(_C["START_TOL_M"])
TABLE_XY = tuple(float(v) for v in _C["TABLE_XY"])
PAD_XY = tuple(float(v) for v in _C["PAD_XY"])
ARENA_SIZE_M = float(_C["ARENA_SIZE_M"])
ARENA_HALF_M = float(_C["ARENA_HALF_M"])
N_OBSTACLES = int(_C["N_OBSTACLES"])
OBSTACLE_POSE_TOL_M = float(_C["OBSTACLE_POSE_TOL_M"])
SCENE_TOL_M = float(_C["SCENE_TOL_M"])
PAYLOAD_SIZE_TOL_M = float(_C["PAYLOAD_SIZE_TOL_M"])
PAYLOAD_START_TOL_M = float(_C["PAYLOAD_START_TOL_M"])
TABLE_REST_TOL_M = float(_C["TABLE_REST_TOL_M"])
CARRY_CLEARANCE_M = float(_C["CARRY_CLEARANCE_M"])
PICK_OVER_TABLE_M = float(_C["PICK_OVER_TABLE_M"])
CARRY_MIN_S = float(_C["CARRY_MIN_S"])
CARRY_RADIUS_TOL_M = float(_C["CARRY_RADIUS_TOL_M"])
CARRY_DZ_TOL_M = float(_C["CARRY_DZ_TOL_M"])
GRIP_PROXIMITY_M = float(_C["GRIP_PROXIMITY_M"])
TRANSPORT_MIN_M = float(_C["TRANSPORT_MIN_M"])
AIRBORNE_MIN_S = float(_C["AIRBORNE_MIN_S"])
RELEASE_NEAR_PAD_M = float(_C["RELEASE_NEAR_PAD_M"])
DELIVERY_TOL_M = float(_C["DELIVERY_TOL_M"])
PLACE_Z_TOL_M = float(_C["PLACE_Z_TOL_M"])
HOLD_S = float(_C["HOLD_S"])
HOLD_MIN_SAMPLES = int(_C["HOLD_MIN_SAMPLES"])
REST_BAND_M = float(_C["REST_BAND_M"])
MAX_BASE_STEP_M = float(_C["MAX_BASE_STEP_M"])
MAX_PAYLOAD_SPEED_MPS = float(_C["MAX_PAYLOAD_SPEED_MPS"])
MAX_PAYLOAD_STEP_M = float(_C["MAX_PAYLOAD_STEP_M"])
MIN_ROBOT_JOINTS = int(_C["MIN_ROBOT_JOINTS"])
MIN_ARTICULATED_LINKS = int(_C["MIN_ARTICULATED_LINKS"])
ARTICULATION_MIN_M = float(_C["ARTICULATION_MIN_M"])
MIN_MOTION_FOR_CREDIT_M = float(_C["MIN_MOTION_FOR_CREDIT_M"])
PICK_APPROACH_M = float(_C["PICK_APPROACH_M"])
PLACE_APPROACH_M = float(_C["PLACE_APPROACH_M"])
LIDAR_MIN_FOV_DEG = float(_C["LIDAR_MIN_FOV_DEG"])
LIDAR_MIN_SAMPLES = int(_C["LIDAR_MIN_SAMPLES"])
LIDAR_MIN_RANGE_M = float(_C["LIDAR_MIN_RANGE_M"])

PLACEMENT_ARENA_INNER_M = float(_C["PLACEMENT_ARENA_INNER_M"])
PLACEMENT_MIN_GAP_M = float(_C["PLACEMENT_MIN_GAP_M"])
PLACEMENT_START_CLEAR_M = float(_C["PLACEMENT_START_CLEAR_M"])
PLACEMENT_FIXTURE_CLEAR_M = float(_C["PLACEMENT_FIXTURE_CLEAR_M"])
PLACEMENT_ROUTE_INFLATE_M = float(_C["PLACEMENT_ROUTE_INFLATE_M"])
PLACEMENT_ROUTE_CELL_M = float(_C["PLACEMENT_ROUTE_CELL_M"])
PLACEMENT_APPROACH_CELL_M = float(_C["PLACEMENT_APPROACH_CELL_M"])
PLACEMENT_MIN_BLOCK_M = float(_C["PLACEMENT_MIN_BLOCK_M"])
PLACEMENT_MAX_ATTEMPTS = int(_C["PLACEMENT_MAX_ATTEMPTS"])

# --- scene geometry: read from benchmark_assets/scene.json ------------------

_S = scene_spec()

TABLE_NAME = str(_S["table"]["name"])
TABLE_TOP_Z_M = float(_S["table"]["top_z"])
TABLE_AABB = (tuple(float(v) for v in _S["table"]["aabb_min"]),
              tuple(float(v) for v in _S["table"]["aabb_max"]))

PAD_NAME = str(_S["pad"]["name"])
PAD_TOP_Z_M = float(_S["pad"]["top_z"])
PAD_AABB = (tuple(float(v) for v in _S["pad"]["aabb_min"]),
            tuple(float(v) for v in _S["pad"]["aabb_max"]))
PAD_HALF_XY_M = (float(_S["pad"]["size"][0]) / 2.0,
                 float(_S["pad"]["size"][1]) / 2.0)

PAYLOAD_NAME = str(_S["payload"]["name"])
PAYLOAD_SIZE_M = float(_S["payload"]["size"])
PAYLOAD_START_XYZ = tuple(float(v) for v in _S["payload"]["position"])
#: Where a payload of this size sits when it is resting on that table / pad.
PAYLOAD_REST_Z_TABLE_M = TABLE_TOP_Z_M + PAYLOAD_SIZE_M / 2.0
PAYLOAD_REST_Z_PAD_M = PAD_TOP_Z_M + PAYLOAD_SIZE_M / 2.0

#: The centre height at which the payload's lowest face clears the TABLE top
#: by CARRY_CLEARANCE_M. Reported, and used only when the run left no
#: measured table to compute the same thing from.
Z_LIFTED_M = TABLE_TOP_Z_M + PAYLOAD_SIZE_M / 2.0 + CARRY_CLEARANCE_M
#: ...and the same over bare floor. Both are DERIVED conveniences: the grade
#: itself asks :func:`support_height` what is underneath the payload at each
#: sample and compares against THAT, because a fixed height cannot tell "held
#: 0.6 m up" from "sitting on a 0.6 m table".
Z_CARRIED_M = CARRY_CLEARANCE_M + PAYLOAD_SIZE_M / 2.0

#: The whole point-to-point job, for the report: start -> table -> pad.
STRAIGHT_START_TABLE_M = math.hypot(TABLE_XY[0] - START_XY[0],
                                    TABLE_XY[1] - START_XY[1])
STRAIGHT_TABLE_PAD_M = math.hypot(PAD_XY[0] - TABLE_XY[0],
                                  PAD_XY[1] - TABLE_XY[1])

#: The file a placement step writes to declare the layout a run is graded on.
LAYOUT_SIDECAR = "r4_graded_layout.json"
#: ...and the environment variable naming the directory it wrote it in, for a
#: grader running in a subprocess that cannot see the placer's own paths.
LAYOUT_DIR_ENV = "AGENTBENCH_R4_LAYOUT_DIR"
PLACEMENT_MECHANISM = "grade_time_placement/v1"

_ALL = [("R4.1", "the run is clean"),
        ("R4.2", "the specified scene is present and intact"),
        ("R4.3", "there is one articulated mobile manipulator"),
        ("R4.4", "the payload started at rest on the table"),
        ("R4.5", "the base drove the route without striking anything"),
        ("R4.6", "the payload was lifted clear and carried"),
        ("R4.7", "the payload was held continuously, never re-acquired"),
        ("R4.8", "the payload was delivered onto the pad and stayed"),
        ("R4.9", "nothing was teleported and nothing was attached")]

_BASIS = {"R4.1": MIXED, "R4.2": CORE_PHYSICAL, "R4.3": MIXED,
          "R4.4": CORE_PHYSICAL, "R4.5": CORE_PHYSICAL,
          "R4.6": CORE_PHYSICAL, "R4.7": CORE_PHYSICAL,
          "R4.8": CORE_PHYSICAL, "R4.9": MIXED}


# --- small pure helpers ------------------------------------------------------


def _f(v):
    return float(v)


def _dist(a, b):
    return math.hypot(_f(a[0]) - _f(b[0]), _f(a[1]) - _f(b[1]))


def path_length_xy(xy):
    return sum(_dist(xy[i], xy[i - 1]) for i in range(1, len(xy)))


def max_step_xy(xy):
    return max((_dist(xy[i], xy[i - 1]) for i in range(1, len(xy))),
               default=0.0)


def speed_profile(t, xyz):
    """``(max_speed_mps, max_step_m)`` over a 3-D pose series.

    dt-independent by construction, for the reason R3 gives: a per-sample
    distance bound silently changes meaning with the basic timestep, and an
    agent that authors ``basicTimeStep 8`` would be graded against a different
    implied speed than one that authors 32.
    """
    max_speed, max_step = 0.0, 0.0
    for i in range(1, len(xyz)):
        d = math.sqrt(sum((_f(xyz[i][k]) - _f(xyz[i - 1][k])) ** 2
                          for k in range(3)))
        max_step = max(max_step, d)
        dt = _f(t[i]) - _f(t[i - 1])
        if dt > 0:
            max_speed = max(max_speed, d / dt)
    return max_speed, max_step


def maximal_runs(mask, t, min_s=0.0):
    """Every maximal contiguous True run of at least ``min_s`` seconds.

    ``[(i0, i1, seconds), ...]``. Contiguous rather than cumulative on
    purpose: a payload that flickers above a height threshold while bouncing
    accumulates time without ever having been carried. Counting the runs is
    also how R4.7 tells one carry from a drop-and-regrasp.
    """
    out = []
    i, n = 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        span = _f(t[j]) - _f(t[i])
        if span >= min_s:
            out.append((i, j, span))
        i = j + 1
    return out


def spread(values):
    """max - min, or ``float('nan')`` for an empty series."""
    return (max(values) - min(values)) if values else float("nan")


def aabb_error(body, lo, hi):
    """Largest per-corner coordinate error between a body's AABB and a spec.

    ``None`` when the body has no world-space AABB -- an absent measurement,
    never a pass.
    """
    if body is None or not body.has_aabb:
        return None
    blo, bhi = body.aabb
    err = 0.0
    for k in range(3):
        err = max(err, abs(_f(blo[k]) - _f(lo[k])),
                  abs(_f(bhi[k]) - _f(hi[k])))
    return err


# --- finding the three fixture bodies, by GEOMETRY --------------------------
#
# Never by name and never by DEF. R1 shipped a grader keyed on the published
# OBSTACLE_n names, the first real agent called its boxes "crate A".."crate E",
# and a correct world scored zero obstacles. The names in scene.json exist so a
# recorder can be asked for a per-step TRACK of the payload; they are not the
# identity rule, and none of the three functions below consults one.


class _SpecBox:
    """A :class:`~agentbench.graders.evidence.Body`-alike over a layout entry.

    So the geometric helpers read a *layout* with exactly the code that reads a
    *measured scene*; a second implementation is a second set of bugs.
    """

    __slots__ = ("name", "body_id", "_lo", "_hi")

    has_aabb = True
    robot_class = False
    member_of = None

    def __init__(self, o):
        cx, cy = _f(o["position"][0]), _f(o["position"][1])
        sx, sy = _f(o["size"][0]), _f(o["size"][1])
        self.name = o["name"]
        self.body_id = o["name"]
        self._lo = (cx - sx / 2.0, cy - sy / 2.0, 0.0)
        self._hi = (cx + sx / 2.0, cy + sy / 2.0, _f(o["size"][2]))

    @property
    def aabb(self):
        return (self._lo, self._hi)


def spec_bodies(layout):
    return [_SpecBox(o) for o in layout]


def match_spec_obstacles(bodies, spec, tol_m=OBSTACLE_POSE_TOL_M):
    """``(found, missing)`` -- the specified obstacles, matched by geometry.

    A body matches a spec entry when its world-space AABB centre is within
    ``tol_m`` of the specified centre on x and y AND its footprint is within
    ``tol_m`` of the specified extents. Each spec entry matches at most once.
    Extra scenery the agent chose to add is neither required nor penalised --
    it only makes the navigation harder for itself.
    """
    unmatched = [b for b in bodies
                 if not getattr(b, "robot_class", False) and b.has_aabb]
    found, missing = [], []
    for want in spec:
        cx, cy = _f(want["position"][0]), _f(want["position"][1])
        sx, sy = _f(want["size"][0]), _f(want["size"][1])
        hit = None
        for b in unmatched:
            lo, hi = b.aabb
            bcx, bcy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
            bsx, bsy = hi[0] - lo[0], hi[1] - lo[1]
            if (abs(bcx - cx) <= tol_m and abs(bcy - cy) <= tol_m
                    and abs(bsx - sx) <= tol_m and abs(bsy - sy) <= tol_m):
                hit = b
                break
        if hit is not None:
            unmatched.remove(hit)
            found.append(hit)
        else:
            missing.append(want["name"])
    return found, missing


def find_fixture(bodies, aabb, tol_m=SCENE_TOL_M, exclude=()):
    """The non-robot body whose world AABB matches ``aabb``, or ``None``.

    Used for the table and the pad. ``(body, n_candidates)`` -- more than one
    match is reported rather than silently resolved, because "there are two
    tables" is a fact about the world the agent built.
    """
    lo, hi = aabb
    skip = {id(b) for b in exclude}
    hits = []
    for b in bodies:
        if id(b) in skip or getattr(b, "robot_class", False) or not b.has_aabb:
            continue
        # NOT ``(aabb_error(...) or 1e9) <= tol``: a PERFECT match is 0.0,
        # which is falsy, and that spelling rejected exactly the bodies it was
        # meant to accept. Measured on the R4 oracle's first run -- the table
        # and the pad are authored at the spec to the digit, both scored
        # ``fixture_candidates: 0``, and the consequences cascaded (the table
        # fell into R4.5's hit set and "airborne" lost its table reference).
        err = aabb_error(b, lo, hi)
        if err is not None and err <= tol_m:
            hits.append(b)
    if len(hits) == 1:
        return hits[0], 1
    return None, len(hits)


def find_payload(bodies):
    """``(body, how, n_candidates)`` -- the payload, found by GEOMETRY.

    A body is the payload when its frozen t=0 AABB is a ``PAYLOAD_SIZE_M`` cube
    (within ``PAYLOAD_SIZE_TOL_M`` on every axis) whose centre is within
    ``PAYLOAD_START_TOL_M`` of the specified start point. ``how`` records which
    channel answered, because the two are not equally strong: ``"aabb"``
    verified the size, ``"position"`` is the fallback for an adapter that
    reports no world-space AABB at all and verified only the start point. The
    caller marks the size clause vacuous in that case rather than pretending it
    was checked.
    """
    pool = [b for b in bodies if not getattr(b, "robot_class", False)]
    hits = []
    for b in pool:
        if not b.has_aabb:
            continue
        lo, hi = b.aabb
        size = [_f(hi[k]) - _f(lo[k]) for k in range(3)]
        centre = [(_f(hi[k]) + _f(lo[k])) / 2.0 for k in range(3)]
        if any(abs(s - PAYLOAD_SIZE_M) > PAYLOAD_SIZE_TOL_M for s in size):
            continue
        if any(abs(centre[k] - PAYLOAD_START_XYZ[k]) > PAYLOAD_START_TOL_M
               for k in range(3)):
            continue
        hits.append(b)
    if len(hits) == 1:
        return hits[0], "aabb", 1
    if hits:
        return None, "ambiguous", len(hits)
    if not any(b.has_aabb for b in pool):
        near = [b for b in pool
                if b.position is not None
                and all(abs(_f(b.position[k]) - PAYLOAD_START_XYZ[k])
                        <= PAYLOAD_START_TOL_M for k in range(3))]
        if len(near) == 1:
            return near[0], "position", 1
        if near:
            return None, "ambiguous", len(near)
    return None, "none", 0


def same_body(a, b):
    """Do two body records name the same body? Id first, then name."""
    if a is None or b is None:
        return False
    if a.body_id and b.body_id and a.body_id == b.body_id:
        return True
    return bool(a.name and b.name and a.name == b.name)


def trajectory_row(traj, body):
    """``(row, how)`` -- which row of ``traj.xyz`` is this body.

    Three channels, tried in order, because the arms disagree about what
    ``Trajectory.body_ids`` holds and neither is wrong (see R3's note). The
    scene NAME is the last resort and is only reached for a body the FIXTURE
    named, never for one the agent named.
    """
    for key, how in ((body.body_id, "body_id"), (body.name, "name")):
        if key:
            i = traj.index_of(key)
            if i is not None:
                return i, how
    if body.name:
        i = traj.index_of_name(body.name)
        if i is not None:
            return i, "scene name"
    return None, "unresolved"


# --- what counts as a collision ---------------------------------------------


def _covers_xy(body, pt):
    lo, hi = body.aabb
    return lo[0] <= pt[0] <= hi[0] and lo[1] <= pt[1] <= hi[1]


def hittable_bodies(bodies, robot, matched, spare=()):
    """``(names, unbounded)`` -- what a robot touch counts as a COLLISION.

    R4 differs from R1 in one load-bearing way: three bodies in this scene are
    ones the robot is SUPPOSED to touch or reach into -- the payload it grasps,
    the table it reaches over and the pad it places on -- so a hit set built
    from "every non-robot body that is not drive-over-able" would count the
    grasp itself as a collision. ``spare`` is those three, resolved by
    geometry upstream, and they are excluded by identity rather than by name.

    What is left, and none of it is a name convention:

    * a MATCHED obstacle of the graded layout -- the primary channel;
    * a body whose name says *wall*, plus any body whose footprint lies against
      the arena boundary and spans most of it -- a wall is scenery no spec
      entry describes, and an agent may or may not have called it one;
    * any other non-robot body with world bounds that the robot could not have
      driven over.

    A body with no bounds is NOT counted and is reported instead: on upstream
    Webots a ``Mesh`` hull arrives without bounds, and failing an honest agent
    for touching a body we could not measure would be scoring our instrument.
    """
    ground_z = 0.0
    if robot is not None and getattr(robot, "has_aabb", False):
        ground_z = _f(robot.aabb[0][2])

    spare_ids = {id(b) for b in spare if b is not None}
    matched_ids = {id(b) for b in matched}
    names, unbounded = set(), []
    for b in matched:
        for key in (b.name, b.body_id):
            if key:
                names.add(str(key).lower())
    for b in bodies:
        if getattr(b, "robot_class", False) or getattr(b, "member_of", None):
            continue
        if robot is not None and b.body_id == robot.body_id:
            continue
        if id(b) in spare_ids or id(b) in matched_ids:
            continue
        label = (b.name or b.body_id or "")
        if "wall" in label.lower():
            names.add(label.lower())
            continue
        if not b.has_aabb:
            unbounded.append(label)
            continue
        lo, hi = b.aabb
        cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
        if (max(abs(cx), abs(cy)) >= ARENA_HALF_M - 0.5
                and max(hi[0] - lo[0], hi[1] - lo[1]) >= ARENA_SIZE_M * 0.5):
            names.add(label.lower())            # a wall by geometry
            continue
        if b.top_z is not None and _f(b.top_z) <= ground_z + 0.05:
            continue                            # drove over it
        if _covers_xy(b, START_XY) and _covers_xy(b, TABLE_XY):
            continue                            # it IS the ground
        for key in (b.name, b.body_id):
            if key:
                names.add(str(key).lower())
    return names, unbounded


def support_height(pt, bodies, margin_m=0.0):
    """The highest static surface UNDER a point, from measured geometry.

    The floor plane (z = 0) is the floor of this function: a scene always has
    one, and a payload 0.10 m off bare ground is off the ground whether or not
    the agent authored a floor Solid the scan could bound.

    This is what makes "airborne" mean *not resting on anything* rather than
    *above some fixed height*, and the difference is the whole assertion: a
    payload sitting untouched on a 0.6 m table is 0.6 m up, and a threshold
    that called that "carried" would be satisfied at t = 0 by every run
    including the one where the robot never moved. Measured on the R4 fixture:
    the payload's authored rest height is 0.625 m, which clears every
    floor-relative carry threshold this task could sensibly set.
    """
    best = 0.0
    for b in bodies:
        if b is None or not b.has_aabb:
            continue
        lo, hi = b.aabb
        if (lo[0] - margin_m <= _f(pt[0]) <= hi[0] + margin_m
                and lo[1] - margin_m <= _f(pt[1]) <= hi[1] + margin_m):
            best = max(best, _f(hi[2]))
    return best


def airborne_mask(xyz, bodies, clearance_m=CARRY_CLEARANCE_M,
                  half_size_m=None):
    """Per-sample "the payload is resting on nothing", from measured geometry.

    True where the payload's LOWEST FACE is ``clearance_m`` above the highest
    static surface beneath its centre -- the table while it is over the table,
    an obstacle while it is over an obstacle, the pad while it is over the pad,
    and the floor everywhere else.
    """
    half = PAYLOAD_SIZE_M / 2.0 if half_size_m is None else float(half_size_m)
    out = []
    for p in xyz:
        bottom = _f(p[2]) - half
        out.append(bottom >= support_height(p, bodies, margin_m=half)
                   + clearance_m)
    return out


def over_table(pt, table, margin_m=PICK_OVER_TABLE_M):
    """Is this xy above the table's footprint (expanded by ``margin_m``)?

    How "it was picked up FROM the table" is asserted: the carry window has to
    START here. A payload that was already on the floor, or that the robot
    collected from somewhere else, does not.
    """
    if table is None or not table.has_aabb:
        lo = (TABLE_AABB[0][0], TABLE_AABB[0][1])
        hi = (TABLE_AABB[1][0], TABLE_AABB[1][1])
    else:
        blo, bhi = table.aabb
        lo, hi = (blo[0], blo[1]), (bhi[0], bhi[1])
    return (lo[0] - margin_m <= _f(pt[0]) <= hi[0] + margin_m
            and lo[1] - margin_m <= _f(pt[1]) <= hi[1] + margin_m)


def samples_inside(xy, bodies, tol_m=0.05):
    """Which measured obstacles the robot's own track passes THROUGH.

    "It drove around" asserted directly from measured geometry rather than
    inferred from a path length. A robot that grazes a box keeps its origin
    outside it; an origin ``tol_m`` INSIDE one went through it -- which is what
    a world whose obstacles have no collision geometry, or a pose written
    rather than driven, looks like.
    """
    through = []
    for b in bodies:
        if not b.has_aabb:
            continue
        lo, hi = b.aabb
        x0, x1 = lo[0] + tol_m, hi[0] - tol_m
        y0, y1 = lo[1] + tol_m, hi[1] - tol_m
        if x0 > x1 or y0 > y1:
            continue
        for p in xy:
            if x0 <= _f(p[0]) <= x1 and y0 <= _f(p[1]) <= y1:
                through.append(b.name or b.body_id)
                break
    return through


# --- grade-time placement ----------------------------------------------------


def _aabb_xy(o):
    cx, cy = _f(o["position"][0]), _f(o["position"][1])
    sx, sy = _f(o["size"][0]), _f(o["size"][1])
    return (cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0)


def _box_point_gap(o, pt):
    x0, y0, x1, y1 = _aabb_xy(o)
    dx = max(x0 - pt[0], 0.0, pt[0] - x1)
    dy = max(y0 - pt[1], 0.0, pt[1] - y1)
    return math.hypot(dx, dy)


def _box_box_gap_xy(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    dx = max(bx0 - ax1, 0.0, ax0 - bx1)
    dy = max(by0 - ay1, 0.0, ay0 - by1)
    return math.hypot(dx, dy)


def _box_box_gap(a, b):
    return _box_box_gap_xy(_aabb_xy(a), _aabb_xy(b))


_TABLE_XY_BOX = (TABLE_AABB[0][0], TABLE_AABB[0][1],
                 TABLE_AABB[1][0], TABLE_AABB[1][1])
_PAD_XY_BOX = (PAD_AABB[0][0], PAD_AABB[0][1],
               PAD_AABB[1][0], PAD_AABB[1][1])


def segment_blocked_length(bodies, start=START_XY, goal=TABLE_XY,
                           samples=2000):
    """How many metres of the straight ``start -> goal`` line lie inside a body.

    "Blocked" as a boolean is satisfied by a corner clipped by a centimetre,
    which proves nothing about what a blind robot would hit. This is the
    quantity :data:`PLACEMENT_MIN_BLOCK_M` is a floor on, and R4.2 re-derives
    it from the MEASURED scene rather than trusting the asset file.
    """
    span = math.hypot(goal[0] - start[0], goal[1] - start[1])
    boxes = [b.aabb for b in bodies if b.has_aabb]
    inside = 0
    for i in range(samples + 1):
        t = i / float(samples)
        x = start[0] + (goal[0] - start[0]) * t
        y = start[1] + (goal[1] - start[1]) * t
        for lo, hi in boxes:
            if lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]:
                inside += 1
                break
    return span * inside / float(samples + 1)


def segment_blocked_by(bodies, start=START_XY, goal=TABLE_XY, samples=2000):
    """Which bodies the straight ``start -> goal`` segment passes through."""
    hits = []
    for b in bodies:
        if not b.has_aabb:
            continue
        lo, hi = b.aabb
        for i in range(samples + 1):
            t = i / float(samples)
            x = start[0] + (goal[0] - start[0]) * t
            y = start[1] + (goal[1] - start[1]) * t
            if lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]:
                hits.append(b.name or b.body_id)
                break
    return hits


def _grid(layout, inflate_m, cell_m):
    """``(n, blocked)`` -- a square occupancy grid over the arena.

    A cell is blocked when its centre is inside an obstacle, inside the TABLE
    or the PAD, or outside the inflated arena. The two fixtures are in it
    because they are real bodies a robot has to route around; asking "is there
    a route" while pretending the table is not there would accept a layout that
    is only solvable through it.

    Deliberately NOT any navigator's own planner: this asks whether the LAYOUT
    admits a route, not whether one particular robot solves it.
    """
    n = int(round(2.0 * ARENA_HALF_M / cell_m))
    blocked = [bytearray(n) for _ in range(n)]

    def centre(i):
        return (i + 0.5) * cell_m - ARENA_HALF_M

    edge = PLACEMENT_ARENA_INNER_M - inflate_m
    for i in range(n):
        if abs(centre(i)) > edge:
            blocked[i] = bytearray(b"\x01" * n)
    for j in range(n):
        if abs(centre(j)) > edge:
            for i in range(n):
                blocked[i][j] = 1

    boxes = [(_aabb_xy(o)) for o in layout] + [_TABLE_XY_BOX, _PAD_XY_BOX]
    for (bx0, by0, bx1, by1) in boxes:
        x0, x1 = bx0 - inflate_m, bx1 + inflate_m
        y0, y1 = by0 - inflate_m, by1 + inflate_m
        for i in range(n):
            if not (x0 <= centre(i) <= x1):
                continue
            row = blocked[i]
            for j in range(n):
                if y0 <= centre(j) <= y1:
                    row[j] = 1
    return n, blocked


def _cell_of(pt, n, cell_m):
    return (min(max(int((pt[0] + ARENA_HALF_M) / cell_m), 0), n - 1),
            min(max(int((pt[1] + ARENA_HALF_M) / cell_m), 0), n - 1))


def route_reaches_both(layout, inflate_m=PLACEMENT_ROUTE_INFLATE_M,
                       cell_m=PLACEMENT_ROUTE_CELL_M,
                       approach_m=PLACEMENT_APPROACH_CELL_M):
    """Can an inflated robot get from the start to WITHIN REACH of both?

    Not "to the table" -- the table is a solid the robot cannot stand inside.
    The question a mobile manipulator's layout has to answer is whether the
    reachable set from the start contains a parking spot within ``approach_m``
    of the table centre AND one within ``approach_m`` of the pad centre. One
    flood fill answers both.
    """
    n, blocked = _grid(layout, inflate_m, cell_m)
    src = _cell_of(START_XY, n, cell_m)
    if blocked[src[0]][src[1]]:
        return False, False
    seen = [bytearray(n) for _ in range(n)]
    seen[src[0]][src[1]] = 1
    stack = [src]
    near_table = near_pad = False
    while stack:
        i, j = stack.pop()
        cx = (i + 0.5) * cell_m - ARENA_HALF_M
        cy = (j + 0.5) * cell_m - ARENA_HALF_M
        if math.hypot(cx - TABLE_XY[0], cy - TABLE_XY[1]) <= approach_m:
            near_table = True
        if math.hypot(cx - PAD_XY[0], cy - PAD_XY[1]) <= approach_m:
            near_pad = True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < n and not blocked[a][b] \
                    and not seen[a][b]:
                seen[a][b] = 1
                stack.append((a, b))
    return near_table, near_pad


def layout_legality(layout):
    """Every acceptance criterion for a graded layout, evaluated separately.

    Named individually rather than reduced to a boolean, so a rejected draw
    says WHICH rule it broke and a test can assert the rules one at a time.
    """
    out = {"outside_arena": [], "too_close": [], "near_start": [],
           "on_fixture": []}
    for o in layout:
        x0, y0, x1, y1 = _aabb_xy(o)
        if (min(x0, y0) < -PLACEMENT_ARENA_INNER_M
                or max(x1, y1) > PLACEMENT_ARENA_INNER_M):
            out["outside_arena"].append(o["name"])
        if _box_point_gap(o, START_XY) < PLACEMENT_START_CLEAR_M:
            out["near_start"].append(o["name"])
        box = _aabb_xy(o)
        if (_box_box_gap_xy(box, _TABLE_XY_BOX) < PLACEMENT_FIXTURE_CLEAR_M
                or _box_box_gap_xy(box, _PAD_XY_BOX)
                < PLACEMENT_FIXTURE_CLEAR_M):
            out["on_fixture"].append(o["name"])
    for i, a in enumerate(layout):
        for b in layout[i + 1:]:
            if _box_box_gap(a, b) < PLACEMENT_MIN_GAP_M:
                out["too_close"].append("%s/%s" % (a["name"], b["name"]))

    bodies = spec_bodies(layout)
    out["blocking_straight_line"] = segment_blocked_by(bodies)
    out["blocked_length_m"] = round(segment_blocked_length(bodies), 4)
    near_table, near_pad = route_reaches_both(layout)
    out["route_to_table"] = near_table
    out["route_to_pad"] = near_pad
    out["legal"] = (not out["outside_arena"] and not out["too_close"]
                    and not out["near_start"] and not out["on_fixture"]
                    and near_table and near_pad
                    and out["blocked_length_m"] >= PLACEMENT_MIN_BLOCK_M)
    return out


def _cheaply_legal(layout):
    """The constraints that need no grid. A pre-filter for the sampler only:
    :func:`layout_legality` remains the one authority and re-checks all of
    them."""
    for i, o in enumerate(layout):
        x0, y0, x1, y1 = _aabb_xy(o)
        if (min(x0, y0) < -PLACEMENT_ARENA_INNER_M
                or max(x1, y1) > PLACEMENT_ARENA_INNER_M):
            return False
        if _box_point_gap(o, START_XY) < PLACEMENT_START_CLEAR_M:
            return False
        box = (x0, y0, x1, y1)
        if (_box_box_gap_xy(box, _TABLE_XY_BOX) < PLACEMENT_FIXTURE_CLEAR_M
                or _box_box_gap_xy(box, _PAD_XY_BOX)
                < PLACEMENT_FIXTURE_CLEAR_M):
            return False
        for b in layout[i + 1:]:
            if _box_box_gap(o, b) < PLACEMENT_MIN_GAP_M:
                return False
    return True


def sample_layout_with_report(seed, sizes=None,
                              max_attempts=PLACEMENT_MAX_ATTEMPTS):
    """``(layout, report)`` -- the GRADED obstacle layout for one seed.

    PURE: the same seed gives the same layout on any machine, and nothing but
    the seed and the published sizes goes in. The count and the sizes are the
    published contract; only the positions are drawn.

    Rejection sampling, because every constraint is checked on the WHOLE
    layout: an illegal draw is discarded and redrawn, so "a route reaches both
    fixtures" and "the straight line to the table is still blocked" are
    acceptance criteria rather than hopes. There is NO fallback -- exhausting
    the budget raises :class:`LayoutError`, since substituting a layout the
    agent has already seen is exactly the escape hatch this mechanism closes.
    """
    if sizes is None:
        sizes = obstacle_sizes()
    rng = random.Random("%s/placement/%s" % (TASK, seed))
    for attempt in range(1, int(max_attempts) + 1):
        layout = []
        for name, size in sizes:
            hx = PLACEMENT_ARENA_INNER_M - size[0] / 2.0
            hy = PLACEMENT_ARENA_INNER_M - size[1] / 2.0
            layout.append({"name": name,
                           "position": [round(rng.uniform(-hx, hx), 4),
                                        round(rng.uniform(-hy, hy), 4),
                                        round(size[2] / 2.0, 4)],
                           "size": [size[0], size[1], size[2]]})
        if not _cheaply_legal(layout):
            continue
        legal = layout_legality(layout)
        if legal["legal"]:
            return layout, {"seed": seed, "attempts": attempt,
                            "mechanism": PLACEMENT_MECHANISM,
                            "legality": legal}
    raise LayoutError(
        "no legal layout for seed %r in %d attempts; the constraints "
        "(arena %.2f m, gap %.2f m, start %.2f m, fixtures %.2f m, a route to "
        "both, %.2f m of the start->table line blocked) may be unsatisfiable "
        "for these sizes"
        % (seed, max_attempts, PLACEMENT_ARENA_INNER_M, PLACEMENT_MIN_GAP_M,
           PLACEMENT_START_CLEAR_M, PLACEMENT_FIXTURE_CLEAR_M,
           PLACEMENT_MIN_BLOCK_M))


def sample_layout(seed, sizes=None, max_attempts=PLACEMENT_MAX_ATTEMPTS):
    """The GRADED obstacle layout for one benchmark seed."""
    return sample_layout_with_report(seed, sizes=sizes,
                                     max_attempts=max_attempts)[0]


def write_graded_layout(directory, seed, layout=None):
    """Declare, on disk, the layout a run is being graded against."""
    obstacles = list(layout) if layout is not None else sample_layout(seed)
    path = Path(directory) / LAYOUT_SIDECAR
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"task": TASK, "seed": seed,
                                "mechanism": PLACEMENT_MECHANISM,
                                "obstacles": obstacles}, indent=1),
                    encoding="utf-8")
    return path


def read_graded_layout(*directories):
    """``(layout, source)`` from the first directory carrying a sidecar."""
    for d in directories:
        if not d:
            continue
        path = Path(d) / LAYOUT_SIDECAR
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            return None, "%s unreadable: %s" % (LAYOUT_SIDECAR, exc)
        obstacles = data.get("obstacles")
        if obstacles:
            return list(obstacles), "%s (seed %s)" % (LAYOUT_SIDECAR,
                                                      data.get("seed"))
    return None, None


def resolve_graded_layout(layout=None, seed=None, directories=()):
    """``(layout, source)`` -- which layout is this run being scored against?

    Precedence, most explicit first: an argument, then a sidecar written by a
    placement step, then a seed to draw from. ``(None, None)`` means no
    grade-time placement happened, and :func:`grade` says so out loud rather
    than quietly scoring the published layout as if it proved perception.
    """
    if layout:
        return list(layout), "argument"
    found, source = read_graded_layout(*directories)
    if found:
        return found, source
    if seed is not None:
        return sample_layout(seed), "seed %s" % (seed,)
    return None, None


# --- the grade ---------------------------------------------------------------


def grade(bundle, *, self_verified=False, graded_layout=None,
          layout_source=None):
    """Score one run. ``bundle`` is an
    :class:`~agentbench.graders.evidence.EvidenceBundle` and nothing else."""
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)
    L = bundle.lbl

    layout = list(graded_layout) if graded_layout else obstacle_spec()
    placed = bool(graded_layout)
    source = layout_source or ("published (NO grade-time placement: this row "
                               "is not evidence of perception -- a memorised "
                               "path would avoid these boxes too)")
    v.measurements["graded_layout"] = {
        "placed": placed, "source": source,
        "obstacles": [{"name": o["name"], "position": o["position"]}
                      for o in layout]}
    if not placed:
        v.note("R4 was scored against the PUBLISHED obstacle layout: no "
               "grade-time placement sidecar (%s) was found and no seed was "
               "given. R4.5's collision-freedom therefore does not by itself "
               "evidence perception. R4.6-R4.8 are unaffected -- a grasp that "
               "does not hold is not a planning failure." % LAYOUT_SIDECAR)

    if bundle.artifact is None:
        v.progress = NO_ARTIFACT
        for aid, what in _ALL:
            v.add(aid, False, what, basis=_BASIS[aid],
                  detail=L("artifact_missing_detail", "no artifact"))
        return v.finish()
    v.artifacts["world"] = str(bundle.artifact)

    traj = bundle.trajectory
    if traj is None or traj.xyz is None:
        v.progress = ARTIFACT_INVALID
        why = (bundle.motion_error or (traj.error if traj else None)
               or "no samples")
        for aid, what in _ALL:
            v.add(aid, False, what, detail=why, basis=_BASIS[aid])
        v.measurements.update(bundle.adapter_measurements.get("motion") or {})
        return v.finish()

    v.progress = ARTIFACT_RUNS
    v.measurements.update(bundle.adapter_measurements.get("motion") or {})

    attrib = bundle.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    if attrib is None:
        v.outcome = INVALID
        v.note(L("attribution_missing_note",
                 "no physics-engine attribution (SPEC A1.10 rule, applied "
                 "suite-wide): the adapter could not name the engine that "
                 "drove this run from any channel it has."))
        return v

    # --- R4.1 the run is clean --------------------------------------------
    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    v.add("R4.1", bool(proc and proc.clean and complete), "the run is clean",
          measured={L("exit_code", "exit code"): proc.exit_code if proc
                    else None,
                    L("error_lines", "error lines"): len(errs),
                    L("driver_completed", "driver reached a clean quit"):
                        complete,
                    L("timed_out", "timed out"):
                        bool(proc.timed_out) if proc else None},
          threshold={L("exit_code", "exit code"): 0,
                     L("error_lines", "error lines"): 0},
          detail="; ".join(errs[:3]), basis=MIXED,
          attested=([("error stream + exit code: %s"
                      % (proc.log_source or "adapter-supplied"))]
                    if proc else []),
          falsifiers=[Falsifier(
              "clean run",
              "a non-zero exit code, an error-class line, a timeout, or a "
              "driver that never reached its target simulated time",
              "the adapter captured the exit code AND the simulator's own "
              "error stream",
              bool(proc is not None and proc.exit_code is not None
                   and proc.log_available is not False))])

    # --- R4.2 the specified scene is intact --------------------------------
    #
    # MEASURED, not trusted, for the same reason R3.2 measures its bin: every
    # clause from R4.4 on is stated against the table top and the pad top, so
    # an agent that quietly lowered the table or raised the pad would be graded
    # against a scene of its own choosing and "it lifted the payload clear"
    # would stop meaning anything.
    t0 = list(bundle.t0.bodies)
    matched, missing = match_spec_obstacles(t0, layout)
    table, n_tables = find_fixture(t0, TABLE_AABB)
    pad, n_pads = find_fixture(t0, PAD_AABB, exclude=[table] if table else ())
    table_err = aabb_error(table, *TABLE_AABB)
    pad_err = aabb_error(pad, *PAD_AABB)
    table_top = table.top_z if (table is not None and table.has_aabb) else None
    pad_top = pad.top_z if (pad is not None and pad.has_aabb) else None
    blocked_m = segment_blocked_length(matched) if matched else 0.0
    scene_ok = (len(matched) == N_OBSTACLES and table is not None
                and pad is not None and table_err is not None
                and table_err <= SCENE_TOL_M and pad_err is not None
                and pad_err <= SCENE_TOL_M)
    v.add("R4.2", scene_ok, "the specified scene is present and intact",
          measured={L("obstacles_matched", "obstacles matched by geometry"):
                    len(matched),
                    L("obstacles_missing", "specified obstacles not found"):
                        missing,
                    L("table_top_z", "measured table top z (m)"):
                        None if table_top is None else round(_f(table_top), 4),
                    L("table_aabb_error", "table AABB error (m)"):
                        None if table_err is None else round(table_err, 4),
                    L("pad_top_z", "measured pad top z (m)"):
                        None if pad_top is None else round(_f(pad_top), 4),
                    L("pad_aabb_error", "pad AABB error (m)"):
                        None if pad_err is None else round(pad_err, 4),
                    L("fixture_candidates", "bodies matching table / pad"):
                        [n_tables, n_pads],
                    L("blocked_length",
                      "metres of the straight start->table line inside an "
                      "obstacle"): round(blocked_m, 4)},
          threshold={L("obstacles_matched", "obstacles matched"): N_OBSTACLES,
                     L("table_aabb_error", "table AABB error (m)"):
                         "<= %.3f" % SCENE_TOL_M,
                     L("pad_aabb_error", "pad AABB error (m)"):
                         "<= %.3f" % SCENE_TOL_M},
          detail=("obstacles, table and pad are all located by GEOMETRY and "
                  "never by name -- an agent may call them anything. The "
                  "%.2f m of the straight start->table line that lies inside "
                  "an obstacle is the SENSING argument, re-derived here from "
                  "the measured scene rather than trusted from the asset: a "
                  "robot that does not perceive cannot cross it without "
                  "striking something, which R4.5 then fails it for"
                  % blocked_m),
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("obstacles measured",
                        "an obstacle deleted, resized or moved",
                        "a world-space AABB for every candidate body at a "
                        "frozen t=0",
                        any(b.has_aabb for b in t0),
                        detail=bundle.t0.source or "no t=0 geometry"),
              Falsifier("the route is genuinely blocked",
                        "a layout whose straight start->table line is clear, "
                        "which a blind driver could cross",
                        "at least %.2f m of that line inside a MEASURED "
                        "obstacle" % PLACEMENT_MIN_BLOCK_M,
                        blocked_m >= PLACEMENT_MIN_BLOCK_M)])

    # --- R4.3 one articulated mobile manipulator ---------------------------
    robots = [b for b in bundle.roster.bodies if b.robot_class]
    robot = robots[0] if len(robots) == 1 else None
    n_joints = (robot.n_joints if robot is not None else None) or 0
    base_row, base_how = (trajectory_row(traj, robot)
                          if robot is not None else (None, "no robot"))
    art_links, art_detail = _articulated_links(traj, robot, base_row)
    v.add("R4.3", bool(robot is not None and n_joints >= MIN_ROBOT_JOINTS
                       and len(art_links) >= MIN_ARTICULATED_LINKS),
          "there is one articulated mobile manipulator",
          measured={L("robot_bodies", "robot-class bodies"): len(robots),
                    L("robot_joints", "joints on that robot"): n_joints,
                    L("articulated_links",
                      "link bodies that TRANSLATE in the base frame"):
                        len(art_links),
                    L("articulation_detail", "their base-frame travel (m)"):
                        art_detail[:6],
                    L("base_row", "the base's pose row resolved by"): base_how},
          threshold={L("robot_bodies", "robot-class bodies"): 1,
                     L("robot_joints", "joints"): ">= %d" % MIN_ROBOT_JOINTS,
                     L("articulated_links", "links that translate"):
                         ">= %d" % MIN_ARTICULATED_LINKS},
          detail=("the joint count is over the WHOLE robot and cannot separate "
                  "an arm joint from a wheel joint on any arm (the neutral "
                  "evidence carries a count and nothing finer), which is why "
                  "the prompt's 'a base plus an arm with >= 4 joints' is "
                  "asserted as '>= %d joints total' PLUS a behavioural clause "
                  "a wheeled chassis cannot satisfy: a wheel spins, so its "
                  "link origin does not move relative to the base, while an "
                  "arm link's does by >= %.2f m"
                  % (MIN_ROBOT_JOINTS, ARTICULATION_MIN_M)),
          basis=MIXED,
          attested=([("robot identity: %s" % robot.identity_evidence)]
                    if robot is not None and robot.identity_evidence else []),
          falsifiers=[Falsifier(
              "articulation observed",
              "a mobile base with no arm on it -- wheels only",
              "per-link pose tracks for the robot (the recorder's --links "
              "channel) plus a resolved base row to measure them against",
              bool(base_row is not None and traj.rows_of_kind("link")),
              detail=("no link tracks in this recording: the task asks for "
                      "them (phases.standalone.links) and without them this "
                      "clause is unmeasured, which is scored as a failure "
                      "with the reason named -- never as a pass"
                      if not traj.rows_of_kind("link") else ""))])

    # --- R4.4 the payload started at rest on the table ----------------------
    payload, how, n_cand = find_payload(t0)
    pay_row, pay_how = ((None, "no payload") if payload is None
                        else trajectory_row(traj, payload))
    dynamic = bool(payload is not None and payload.dynamic is True)
    detached = bool(payload is not None
                    and not getattr(payload, "member_of", None)
                    and not same_body(payload, robot))
    z0 = (None if pay_row is None else _f(traj.xyz[pay_row][0][2]))
    xy0 = (None if pay_row is None
           else (_f(traj.xyz[pay_row][0][0]), _f(traj.xyz[pay_row][0][1])))
    start_xy_err = None if xy0 is None else _dist(xy0, TABLE_XY)
    rest_ref = (PAYLOAD_REST_Z_TABLE_M if table_top is None
                else _f(table_top) + PAYLOAD_SIZE_M / 2.0)
    start_z_err = None if z0 is None else abs(z0 - rest_ref)
    v.add("R4.4", bool(payload is not None and dynamic and detached
                       and pay_row is not None
                       and start_xy_err is not None
                       and start_xy_err <= PAYLOAD_START_TOL_M
                       and start_z_err is not None
                       and start_z_err <= TABLE_REST_TOL_M),
          "the payload started at rest on the table",
          measured={L("payload_found", "payload matched at t=0 by"): how,
                    L("payload_candidates", "bodies matching the payload spec"):
                        n_cand,
                    L("payload_dynamic", "it still has a mass model"):
                        None if payload is None else payload.dynamic,
                    L("payload_independent",
                      "it is not a part of the robot"): detached,
                    L("start_xy_error", "xy error vs the table centre (m)"):
                        None if start_xy_err is None
                        else round(start_xy_err, 4),
                    L("start_z", "first sample z (m)"):
                        None if z0 is None else round(z0, 4),
                    L("start_z_error",
                      "z error vs MEASURED table top + half the cube (m)"):
                        None if start_z_err is None
                        else round(start_z_err, 4),
                    L("pose_row", "pose row resolved by"): pay_how},
          threshold={L("payload_candidates", "candidates"): 1,
                     L("payload_dynamic", "mass model"): True,
                     L("start_xy_error", "xy error (m)"):
                         "<= %.3f" % PAYLOAD_START_TOL_M,
                     L("start_z_error", "z error (m)"):
                         "<= %.3f" % TABLE_REST_TOL_M},
          detail=("the payload is found by its t=0 geometry -- a %.3f m cube "
                  "within %.2f m of the specified start -- and never by name "
                  "or DEF; expected rest z is %.4f m = the MEASURED table top "
                  "plus half the cube. 'dynamic' and 'independent' are the "
                  "anti-cheat clauses a body nailed in place, or nested into "
                  "the robot, fails here rather than at R4.9"
                  % (PAYLOAD_SIZE_M, PAYLOAD_START_TOL_M, rest_ref)),
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("payload size verified",
                        "a body at the right place that is not the specified "
                        "cube",
                        "a world-space AABB for the candidate body at t=0",
                        how == "aabb",
                        detail=("matched on the t=0 point only: this adapter "
                                "reports no world-space AABB, so the SIZE was "
                                "not checked" if how == "position" else "")),
              Falsifier("dynamics attached",
                        "the agent 'held' the payload by removing its mass "
                        "model",
                        "the adapter answers 'is this body dynamic' either "
                        "way rather than returning None",
                        payload is not None and payload.dynamic is not None)])

    if payload is None or pay_row is None or robot is None or base_row is None \
            or traj.t is None or len(traj.t) != traj.n_samples:
        why = _why_unmeasurable(payload, pay_row, robot, base_row, traj)
        for aid in ("R4.5", "R4.6", "R4.7", "R4.8", "R4.9"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid], detail=why)
        return v.finish()

    t = traj.t
    base = traj.xyz[base_row]
    pay = traj.xyz[pay_row]
    base_xy = [(p[0], p[1]) for p in base]
    pay_xy = [(p[0], p[1]) for p in pay]

    # --- R4.5 the drive ----------------------------------------------------
    travelled = path_length_xy(base_xy)
    start_err = _dist(base_xy[0], START_XY)
    i_table = _first_within(base_xy, TABLE_XY, PICK_APPROACH_M)
    i_pad = (None if i_table is None
             else _first_within(base_xy, PAD_XY, PLACE_APPROACH_M,
                                start=i_table))
    spare = [b for b in (payload, table, pad) if b is not None]
    hit_names, unbounded = hittable_bodies(t0, robot, matched, spare=spare)
    contacts = bundle.contacts
    hits, first_hits = _count_hits(contacts, robot, hit_names)
    channel_live = bool(contacts is not None and contacts.supported
                        and (contacts.total_observed or 0) > 0)
    through = samples_inside(base_xy, matched)
    v.add("R4.5", bool(start_err <= START_TOL_M
                       and travelled >= MIN_MOTION_FOR_CREDIT_M
                       and i_table is not None and i_pad is not None
                       and channel_live and hits == 0 and not through),
          "the base drove the route without striking anything",
          measured={L("start_error", "distance from the declared start (m)"):
                    round(start_err, 4),
                    L("travelled", "distance actually driven (m)"):
                        round(travelled, 4),
                    L("reached_table",
                      "first sample within %.1f m of the table (s)"
                      % PICK_APPROACH_M):
                        None if i_table is None else round(_f(t[i_table]), 3),
                    L("reached_pad",
                      "first later sample within %.1f m of the pad (s)"
                      % PLACE_APPROACH_M):
                        None if i_pad is None else round(_f(t[i_pad]), 3),
                    L("contacts", "robot-obstacle/wall contacts"): hits,
                    L("first_hits", "what it struck"): first_hits[:4],
                    L("hittable", "bodies a touch would count against"):
                        len(hit_names),
                    L("unbounded", "bodies with no bounds to judge"):
                        unbounded[:4],
                    L("through", "obstacles the track passes through"):
                        through,
                    L("contacts_seen", "contacts of any kind observed"):
                        None if contacts is None else contacts.total_observed},
          threshold={L("start_error", "start error (m)"):
                     "<= %.2f" % START_TOL_M,
                     L("travelled", "distance driven (m)"):
                         ">= %.1f" % MIN_MOTION_FOR_CREDIT_M,
                     L("contacts", "collisions"): 0,
                     L("through", "pass-throughs"): []},
          detail=("the ORDER is asserted, not just the visits: the first "
                  "sample near the pad is searched for only AFTER the first "
                  "one near the table, so a robot that drove to the pad first "
                  "and the table second did not do this task. The payload, "
                  "the table and the pad are excluded from the hit set BY "
                  "IDENTITY (they were resolved geometrically at R4.2/R4.4), "
                  "because reaching into them is the job -- everything else "
                  "with bounds is a collision. Yaw is not in the neutral "
                  "evidence, so the start pose is asserted as a POSITION and "
                  "the '%.0f degrees' is unmeasured" % START_YAW_DEG),
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("a collision could have been reported",
                        "the robot striking an obstacle or a wall",
                        "a contact channel that produced at least one contact "
                        "of any kind over the run",
                        channel_live,
                        detail=(contacts.witness_detail if contacts
                                else "no contact observation at all")),
              Falsifier("it earned collision-freedom by moving",
                        "a robot that never moved and therefore hit nothing",
                        "at least %.1f m of measured travel"
                        % MIN_MOTION_FOR_CREDIT_M,
                        travelled >= MIN_MOTION_FOR_CREDIT_M)])

    # --- R4.6 / R4.7 the grasp ---------------------------------------------
    # Static geometry the payload could be RESTING on: the table, the pad,
    # every matched obstacle, and (implicitly, inside support_height) the floor
    # plane. "Airborne" is measured against whatever is under the payload at
    # that instant, not against a fixed height -- a payload sitting untouched
    # on a 0.6 m table is 0.6 m up, and a floor-relative threshold would call
    # that a carry from t = 0 on every run in the suite.
    supports = [b for b in ([table, pad] + list(matched)) if b is not None]
    clear = airborne_mask(pay, supports)
    episodes = maximal_runs(clear, t, AIRBORNE_MIN_S)
    carry = _best_carry(episodes, t, base, pay)
    if carry is None:
        cm = {"airborne_episodes": len(episodes)}
        carried_ok = False
        picked_from_table = False
        grip = {"link": None, "max_distance": None, "spread": None}
    else:
        cm = carry
        picked_from_table = over_table(
            (pay[cm["episode_i0"]][0], pay[cm["episode_i0"]][1]), table)
        grip = _gripper_proximity(traj, art_links, pay, cm["i0"], cm["i1"])
        carried_ok = bool(cm["seconds"] >= CARRY_MIN_S
                          and cm["base_travel"] >= TRANSPORT_MIN_M
                          and cm["payload_travel"] >= TRANSPORT_MIN_M
                          and cm["radius_spread"] <= CARRY_RADIUS_TOL_M
                          and cm["dz_spread"] <= CARRY_DZ_TOL_M
                          and grip["max_distance"] is not None
                          and grip["max_distance"] <= GRIP_PROXIMITY_M)
    v.add("R4.6", bool(picked_from_table and carried_ok),
          "the payload was lifted clear and carried",
          measured={L("picked_from_table",
                      "the carry began with the payload over the table"):
                    picked_from_table,
                    L("max_z", "highest payload z reached (m)"):
                        round(max(_f(p[2]) for p in pay), 4),
                    L("support_at_pick",
                      "surface height under the payload at t=0 (m)"):
                        round(support_height(pay[0], supports,
                                             PAYLOAD_SIZE_M / 2.0), 4),
                    L("carry_s", "the RIGID transport window (s)"):
                        round(cm.get("seconds", 0.0), 3),
                    L("airborne_s", "the whole airborne episode (s)"):
                        round(cm.get("episode_s", 0.0), 3),
                    L("carry_clearance",
                      "lowest face must clear whatever is under it by (m)"):
                        CARRY_CLEARANCE_M,
                    L("grip_link",
                      "the articulated link the payload stayed nearest"):
                        grip["link"],
                    L("grip_distance",
                      "furthest the payload got from it during the carry (m)"):
                        _r4(grip["max_distance"]),
                    L("grip_spread",
                      "spread of that distance (m)"): _r4(grip["spread"]),
                    L("base_travel", "base travel during the carry (m)"):
                        round(cm.get("base_travel", 0.0), 4),
                    L("payload_travel", "payload travel during the carry (m)"):
                        round(cm.get("payload_travel", 0.0), 4),
                    L("radius_spread",
                      "spread of |payload - base| during the carry (m)"):
                        _r4(cm.get("radius_spread")),
                    L("radius_mean", "mean |payload - base| (m)"):
                        _r4(cm.get("radius_mean")),
                    L("dz_spread",
                      "spread of (payload z - base z) during the carry (m)"):
                        _r4(cm.get("dz_spread"))},
          threshold={L("picked_from_table", "carry began over the table"):
                     True,
                     L("carry_s", "carry window (s)"):
                         ">= %.1f" % CARRY_MIN_S,
                     L("base_travel", "base travel (m)"):
                         ">= %.1f" % TRANSPORT_MIN_M,
                     L("payload_travel", "payload travel (m)"):
                         ">= %.1f" % TRANSPORT_MIN_M,
                     L("radius_spread", "radius spread (m)"):
                         "<= %.3f" % CARRY_RADIUS_TOL_M,
                     L("dz_spread", "height-difference spread (m)"):
                         "<= %.3f" % CARRY_DZ_TOL_M,
                     L("grip_distance", "distance to the nearest link (m)"):
                         "<= %.2f" % GRIP_PROXIMITY_M},
          detail=("THE GRASP PROOF, and it is geometric. Over one contiguous "
                  "window the payload's lowest face is %.2f m clear of "
                  "WHATEVER IS UNDER IT -- the table while it is over the "
                  "table, an obstacle while it is over one, the pad, else the "
                  "floor -- while it and the base each travel >= %.1f m and "
                  "the DISTANCE between them holds inside a %.3f m band, as "
                  "does the height difference. Both are invariant to the "
                  "base's yaw, which changes continuously while it drives, so "
                  "a world-frame offset would not be constant even for a "
                  "perfect grasp. Nothing that is not being carried satisfies "
                  "all of it: a payload left on top of an obstacle keeps the "
                  "radius only until the base drives on; one dragged along "
                  "the floor keeps the radius and loses the clearance; one "
                  "never picked up fails 'the carry began over the table'. "
                  "The last clause is 'tracking the GRIPPER' rather than the "
                  "base, and it is what a payload riding on the robot's own "
                  "DECK fails: the payload must stay within %.2f m of an "
                  "ARTICULATED link -- one that moves in the base frame, i.e. "
                  "part of the arm and never the chassis. Measured on an "
                  "oracle run whose grip failed exactly that way: the cube "
                  "fell onto the chassis, rode 5.4 m with a near-constant "
                  "radius to the base, and sat 0.465 m from the wrist"
                  % (CARRY_CLEARANCE_M, TRANSPORT_MIN_M, CARRY_RADIUS_TOL_M,
                     GRIP_PROXIMITY_M)),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "carried through free space",
              "a payload that never left the table, one pushed along it, or "
              "one that fell out of the gripper",
              "a pose series long enough to contain a %.1f s window"
              % CARRY_MIN_S,
              bool((traj.recorded_s or 0.0) > CARRY_MIN_S
                   and traj.n_samples >= 2),
              detail="recorded %s s" % (traj.recorded_s,))])

    # The EPISODE's end, not the rigid window's: "where was it put down" is a
    # question about the moment the payload stopped being clear of everything,
    # which is after the arm has already begun lowering it.
    end_i = None if carry is None else carry["episode_i1"]
    end_xy = (None if end_i is None
              else (_f(pay[end_i][0]), _f(pay[end_i][1])))
    end_near_pad = (None if end_xy is None else _dist(end_xy, PAD_XY))
    descent = (None if end_i is None
               else _f(pay[end_i][2]) - min(_f(p[2]) for p in pay[end_i:]))
    v.add("R4.7", bool(len(episodes) == 1 and carried_ok
                       and end_near_pad is not None
                       and end_near_pad <= RELEASE_NEAR_PAD_M),
          "the payload was held continuously, never re-acquired",
          measured={L("episodes",
                      "separate airborne episodes of >= %.1f s"
                      % AIRBORNE_MIN_S): len(episodes),
                    L("episode_spans", "their durations (s)"):
                        [round(e[2], 3) for e in episodes[:6]],
                    L("end_near_pad",
                      "distance from the pad where the carry ended (m)"):
                        _r4(end_near_pad),
                    L("descent_after_carry",
                      "descent from the end of the carry (m)"): _r4(descent)},
          threshold={L("episodes", "airborne episodes"): 1,
                     L("end_near_pad", "distance from the pad (m)"):
                         "<= %.2f" % RELEASE_NEAR_PAD_M},
          detail=("'a dropped or re-acquired payload is a failure' asserted "
                  "directly: a drop and a re-grasp produce TWO airborne "
                  "episodes and this counts them. The single episode must "
                  "also end over the pad, so a payload put down anywhere else "
                  "and later collected fails here even though each half of it "
                  "looked like a carry. The descent after the carry is "
                  "REPORTED and not gated -- 'release it on the pad' does not "
                  "fix a release height, and gating one would fail an honest "
                  "agent for a choice the prompt does not make"),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "a drop would have been visible",
              "a payload released mid-route and picked up again",
              "a per-step pose series over the whole run, not a first-N-steps "
              "window",
              bool(traj.n_samples > 100
                   and (traj.recorded_s or 0.0) >= CARRY_MIN_S * 2))])

    # --- R4.8 delivered, and stayed ----------------------------------------
    hold = [k for k in range(len(pay)) if _f(t[k]) >= _f(t[-1]) - HOLD_S]
    hold_z = [_f(pay[k][2]) for k in hold]
    hold_span = (_f(t[hold[-1]]) - _f(t[hold[0]])) if hold else 0.0
    rest_ref_pad = (PAYLOAD_REST_Z_PAD_M if pad_top is None
                    else _f(pad_top) + PAYLOAD_SIZE_M / 2.0)
    off_pad = sum(1 for k in hold
                  if _dist((pay[k][0], pay[k][1]), PAD_XY) > DELIVERY_TOL_M)
    wrong_z = sum(1 for z in hold_z if abs(z - rest_ref_pad) > PLACE_Z_TOL_M)
    band = spread(hold_z)
    v.add("R4.8", bool(hold and off_pad == 0 and wrong_z == 0
                       and band == band and band <= REST_BAND_M
                       and len(hold) >= HOLD_MIN_SAMPLES
                       and hold_span >= HOLD_S - 1e-9),
          "the payload was delivered onto the pad and stayed",
          measured={L("hold_samples", "samples in the last %.1f s" % HOLD_S):
                    len(hold),
                    L("hold_span", "the window actually covered (s)"):
                        round(hold_span, 3),
                    L("off_pad", "of those, outside the delivery tolerance"):
                        off_pad,
                    L("wrong_z", "of those, not at the pad rest height"):
                        wrong_z,
                    L("rest_band", "z spread over that window (m)"):
                        _r4(band),
                    L("final_xyz", "final payload xyz (m)"):
                        [round(_f(pay[-1][k]), 4) for k in range(3)],
                    L("expected_rest_z",
                      "MEASURED pad top + half the cube (m)"):
                        round(rest_ref_pad, 4)},
          threshold={L("off_pad", "samples off the pad"): 0,
                     L("wrong_z", "samples at the wrong height"): 0,
                     L("rest_band", "z spread (m)"): "<= %.3f" % REST_BAND_M,
                     L("hold_span", "window covered (s)"): ">= %.1f" % HOLD_S,
                     L("hold_samples", "samples"):
                         ">= %d" % HOLD_MIN_SAMPLES},
          detail=("'on the pad' is xy within %.2f m of the pad centre AND the "
                  "centre height at the MEASURED pad top plus half the cube, "
                  "so a payload balanced on the pad's edge, resting beside it "
                  "on the floor, or still held above it all fail. The sample "
                  "count stops a coarse recording from satisfying a "
                  "%.1f-second claim with two points"
                  % (DELIVERY_TOL_M, HOLD_S)),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "it stayed put",
              "a payload that rolled off, was dragged away, or was never on "
              "the pad",
              ">= %d samples inside the last %.1f s of the recording"
              % (HOLD_MIN_SAMPLES, HOLD_S),
              len(hold) >= HOLD_MIN_SAMPLES)])

    # --- R4.9 no teleport, no structural attachment ------------------------
    base_step = max_step_xy(base_xy)
    pay_speed, pay_step = speed_profile(t, pay)
    pay_kind = traj.kind_of(pay_row)
    pay_parent = traj.parent_of(pay_row)
    structural = bool(pay_kind != "link" and not pay_parent
                      and not getattr(payload, "member_of", None))
    v.add("R4.9", bool(base_step <= MAX_BASE_STEP_M
                       and pay_speed <= MAX_PAYLOAD_SPEED_MPS
                       and pay_step <= MAX_PAYLOAD_STEP_M and structural
                       and dynamic),
          "nothing was teleported and nothing was attached",
          measured={L("base_step", "largest single-sample base step (m)"):
                    round(base_step, 4),
                    L("payload_speed", "fastest the payload ever moved (m/s)"):
                        round(pay_speed, 4),
                    L("payload_step", "largest single-sample payload step (m)"):
                        round(pay_step, 4),
                    L("payload_track_kind", "how the payload is tracked"):
                        pay_kind or "unclassified",
                    L("payload_parent", "the body it belongs to, if any"):
                        pay_parent,
                    L("payload_dynamic", "it still has a mass model"):
                        None if payload is None else payload.dynamic},
          threshold={L("base_step", "base step (m)"):
                     "<= %.2f" % MAX_BASE_STEP_M,
                     L("payload_speed", "payload speed (m/s)"):
                         "<= %.1f" % MAX_PAYLOAD_SPEED_MPS,
                     L("payload_step", "payload step (m)"):
                         "<= %.2f" % MAX_PAYLOAD_STEP_M,
                     L("payload_track_kind", "track kind"): "not a link"},
          detail=("the start->pad jump is %.2f m, which at any plausible "
                  "timestep is far over both rate bounds. The STRUCTURAL "
                  "clause is what catches re-parenting: a payload moved into "
                  "the robot's subtree is recorded as a LINK of it, with a "
                  "parent, and stops being an independent body. A runtime "
                  "WELD between two bodies that both stay independent is NOT "
                  "distinguishable from a friction grasp in this evidence -- "
                  "meta.json says so under known_hole rather than implying "
                  "otherwise, and the prompt forbids it explicitly"
                  % _dist(PAYLOAD_START_XYZ[:2], PAD_XY)),
          basis=MIXED,
          falsifiers=[Falsifier(
              "re-parenting would have been visible",
              "the payload moved into the robot's subtree so the solver "
              "carries it for free",
              "a recorder that classifies each track as robot / link / solid "
              "and names a link's parent",
              bool(traj.kinds))])

    return v.finish(GRADED_PASS)


# --- helpers the grade uses --------------------------------------------------


def _r4(v):
    return None if v is None or v != v else round(float(v), 4)


def _why_unmeasurable(payload, pay_row, robot, base_row, traj):
    if payload is None:
        return ("no body matching the specified payload was found at t=0, so "
                "nothing about the manipulation can be measured")
    if robot is None:
        return ("the roster does not contain exactly one robot-class body, so "
                "there is no base to measure the carry against")
    if pay_row is None or base_row is None:
        return ("%s has no row in the pose series (rows: %s). The task asks "
                "the recorder for a track of the payload by name and gets one "
                "for free for a top-level dynamic body; a payload nested "
                "inside a static group under a different name has neither"
                % ("the payload" if pay_row is None else "the robot",
                   traj.body_ids[:6]))
    return ("the pose series carries no usable time base (%d samples, %s "
            "times), and every threshold from here on is in seconds"
            % (traj.n_samples,
               "none" if traj.t is None else len(traj.t)))


def _gripper_proximity(traj, art_links, pay, i0, i1):
    """``{link, max_distance, spread}`` -- did the payload track the GRIPPER?

    The brief's own words for the grasp proof are "the payload tracks the
    gripper", and this is the only channel in the neutral evidence that can
    say so: among the robot's ARTICULATED links (:func:`_articulated_links` --
    bodies that move in the base frame, so an arm link and never the chassis
    or a wheel), take the one the payload stayed nearest, and report how far
    it ever got from it and how much that distance varied.

    The gripper is identified GEOMETRICALLY and never by name -- nothing in
    the prompt suggests a name for a tool body, and R1's obstacle-name mistake
    is exactly this shape.

    This is the clause that separates a grasp from a DECK RIDE, which is not
    hypothetical: an oracle run whose grip failed dropped the cube onto the
    robot's own chassis, where it kept a near-constant distance to the base
    for 5.4 m -- satisfying every other clause in R4.6 -- while sitting
    0.465 m from the nearest arm link.
    """
    best = {"link": None, "max_distance": None, "spread": None}
    if traj.xyz is None:
        return best
    for i in art_links:
        p = traj.xyz[i]
        if len(p) <= i1:
            continue
        d = [math.sqrt(sum((_f(p[k][j]) - _f(pay[k][j])) ** 2
                           for j in range(3))) for k in range(i0, i1 + 1)]
        if not d:
            continue
        hi = max(d)
        if best["max_distance"] is None or hi < best["max_distance"]:
            best = {"link": traj.name_of(i) or ("row %d" % i),
                    "max_distance": hi, "spread": spread(d)}
    return best


def _first_within(xy, point, radius, start=0):
    for k in range(start, len(xy)):
        if _dist(xy[k], point) <= radius:
            return k
    return None


def _articulated_links(traj, robot, base_row):
    """``(rows, detail)`` -- link bodies that TRANSLATE in the base frame.

    The clause that tells an arm from a set of wheels without a device query.
    A wheel spins about its own axis, so its link ORIGIN keeps a constant
    offset from the base; an arm link's origin sweeps. Measured as the spread
    of the link-to-base distance plus the spread of the height difference --
    the same two yaw-invariant quantities the carry proof uses, and for the
    same reason: the neutral evidence has no orientations.
    """
    if robot is None or base_row is None or traj.xyz is None:
        return [], []
    rows = traj.rows_of_kind("link")
    if not rows:
        return [], []
    parent = robot.body_id
    base = traj.xyz[base_row]
    out, detail = [], []
    for i in rows:
        if traj.parent_of(i) not in (None, "", parent) \
                and traj.parent_of(i) != robot.name:
            continue
        p = traj.xyz[i]
        n = min(len(p), len(base))
        radius = [math.sqrt(sum((_f(p[k][j]) - _f(base[k][j])) ** 2
                                for j in range(3))) for k in range(n)]
        dz = [_f(p[k][2]) - _f(base[k][2]) for k in range(n)]
        travel = max(spread(radius), spread(dz))
        if travel != travel:
            continue
        detail.append([traj.name_of(i) or ("row %d" % i), round(travel, 4)])
        if travel >= ARTICULATION_MIN_M:
            out.append(i)
    detail.sort(key=lambda d: -d[1])
    return out, detail


def rigid_window(t, base, pay, i0, i1, radius_tol, dz_tol):
    """The longest sub-window of ``[i0, i1]`` over which the payload is RIGID
    with respect to the base, and what the pair did during it.

    Why a sub-window and not the whole airborne episode: the episode
    necessarily begins with a LIFT and ends with a SET-DOWN, and during both
    the arm is extending or folding, so the payload-to-base distance changes
    by design. Measured on the R4 oracle's own run -- a complete, successful
    pick / 6 m transport / place -- the whole-episode spreads were 0.295 m of
    radius and 0.570 m of height difference, entirely from those two ends, and
    a constancy test applied to the episode fails a run that did the task
    perfectly. The claim R4.6 wants to make is about the TRANSPORT: while the
    robot was driving, the payload moved with it as one body.

    Two monotonic deques give the maximal window ending at each sample in O(n);
    among all of them the one with the greatest base displacement is returned,
    because the carry is the stretch during which the robot actually went
    somewhere.
    """
    from collections import deque
    radius = [math.sqrt(sum((_f(pay[k][j]) - _f(base[k][j])) ** 2
                            for j in range(3))) for k in range(i0, i1 + 1)]
    dz = [_f(pay[k][2]) - _f(base[k][2]) for k in range(i0, i1 + 1)]
    n = len(radius)
    rmax, rmin, zmax, zmin = deque(), deque(), deque(), deque()
    left = 0
    best = None
    for right in range(n):
        for dq, arr, cmp_ in ((rmax, radius, True), (rmin, radius, False),
                              (zmax, dz, True), (zmin, dz, False)):
            while dq and ((arr[dq[-1]] <= arr[right]) if cmp_
                          else (arr[dq[-1]] >= arr[right])):
                dq.pop()
            dq.append(right)
        while (radius[rmax[0]] - radius[rmin[0]] > radius_tol
               or dz[zmax[0]] - dz[zmin[0]] > dz_tol):
            left += 1
            for dq in (rmax, rmin, zmax, zmin):
                if dq[0] < left:
                    dq.popleft()
        a, b = i0 + left, i0 + right
        travel = _dist((base[a][0], base[a][1]), (base[b][0], base[b][1]))
        if best is None or travel > best["base_travel"]:
            seg = radius[left:right + 1]
            best = {"i0": a, "i1": b,
                    "seconds": _f(t[b]) - _f(t[a]),
                    "t0": round(_f(t[a]), 3), "t1": round(_f(t[b]), 3),
                    "base_travel": travel,
                    "payload_travel": _dist((pay[a][0], pay[a][1]),
                                            (pay[b][0], pay[b][1])),
                    "radius_spread": spread(seg),
                    "radius_mean": sum(seg) / len(seg),
                    "dz_spread": spread(dz[left:right + 1])}
    return best or {"i0": i0, "i1": i0, "seconds": 0.0, "t0": None, "t1": None,
                    "base_travel": 0.0, "payload_travel": 0.0,
                    "radius_spread": float("nan"), "radius_mean": float("nan"),
                    "dz_spread": float("nan")}


def _best_carry(episodes, t, base, pay):
    """The airborne episode that best evidences a carry, with its numbers.

    "Best" is the one whose RIGID sub-window covers the most ground: if a run
    contains several airborne episodes, the carry is the one during which the
    robot actually went somewhere while the payload came with it. R4.7
    separately refuses a run that has more than one episode at all.
    """
    best = None
    for i0, i1, span in episodes:
        rec = rigid_window(t, base, pay, i0, i1,
                           CARRY_RADIUS_TOL_M, CARRY_DZ_TOL_M)
        rec["episode_i0"], rec["episode_i1"] = i0, i1
        rec["episode_s"] = span
        rec["airborne_episodes"] = len(episodes)
        if best is None or rec["base_travel"] > best["base_travel"]:
            best = rec
    return best


def _count_hits(contacts, robot, hit_names):
    """``(n, first_hits)`` -- contacts between the robot and the hit set."""
    if contacts is None or not contacts.supported:
        return 0, []
    rid = (robot.body_id or "").lower() if robot is not None else ""
    rname = (robot.name or "").lower() if robot is not None else ""
    n, first = 0, []
    for p in contacts.pairs:
        a = str(p.a or "").lower()
        b = str(p.b or "").lower()
        for me, other, me_robot in ((a, b, p.a_robot), (b, a, p.b_robot)):
            if not other or other == me:
                continue
            if not (me_robot or me in (rid, rname)):
                continue
            if other in hit_names:
                n += 1
                if other not in first:
                    first.append(other)
                break
    return n, first
