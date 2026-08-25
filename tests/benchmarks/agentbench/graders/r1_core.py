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

"""R1 ``lidar_nav`` -- the **sim-neutral core** of the robotics tier.

Sensor-driven obstacle avoidance to a goal: the canonical mobile-robot
exercise, and the first non-trivial thing anyone is taught with any simulator.
Every comparator on the primary leaderboard documents a LiDAR and a
differential-drive robot, so a simulator that cannot express this is making a
statement about itself rather than about the task.

    R1.1  the run is clean          exit 0, no error lines, driver completed
    R1.2  one drivable robot        exactly one robot-class body, >= 2 joints
    R1.3  the obstacles are intact  five of them, at the GRADED layout, none
                                    moved beyond tolerance, and the straight
                                    line still blocked
    R1.4  the goal is reached       final |xy - GOAL| <= 0.30 m
    R1.5  nothing was hit           no contact naming the robot and anything
                                    it could not have driven over
    R1.6  it drove, and drove AROUND  no sample inside an obstacle it was
                                    graded against, no sample step over
                                    0.25 m, and a path at least as long as a
                                    LOWER BOUND on the shortest legal route

**Where the sensing proof comes from, and why it is not an API counter.**
The graded layout's straight START->GOAL line is BLOCKED -- an acceptance
criterion of the layout itself (below), and re-verified here from the measured
obstacle geometry rather than trusted. So R1.4 + R1.5 + R1.6 together say: it
got there, around the obstacles, touching nothing and passing through nothing.
That is behavioural evidence of perception, it means the same thing in every
simulator, and it cannot be satisfied by a robot that drives blind.

A LiDAR read-count would have been the obvious alternative and it is worse: it
is spelled differently in every simulator, it is trivially satisfiable by
reading the sensor and ignoring it, and it would have made this core
non-neutral. What a read count *does* catch and behaviour alone does not is a
controller that plans a fixed path from published obstacle positions. The
answer to that is **grade-time placement**, not a counter.

--------------------------------------------------------------------------
THE ANTI-HARDCODE MECHANISM, AND WHICH ONE IS AUTHORITATIVE
--------------------------------------------------------------------------

**Authoritative: grade-time placement** (:func:`sample_layout`). The task
publishes the obstacle **count and sizes**; the **positions are drawn at grade
time** from the benchmark seed, under legality constraints checked *at sample
time* -- inside the arena, ``PLACEMENT_MIN_GAP_M`` apart,
``PLACEMENT_ENDPOINT_CLEAR_M`` clear of start and goal, a route exists, and the
straight start->goal line is still blocked over at least
``PLACEMENT_MIN_BLOCK_M``. Illegal draws are **re-sampled**, never repaired and
never swapped for a published fallback, so "the straight line is blocked" is an
acceptance criterion instead of a bail-out. :func:`grade` scores the world
against **that** layout, which is what makes R1.3 and R1.5 decidable on it.

**Superseded: the seeded perturbation** (:func:`seeded_offsets`,
:func:`perturbed_spec`, ``PERTURBATION_MAX_M``). It is kept so the historical
sweep (``adapters/mujoco/r1_perturbation_sweep.py``) still reproduces the
evidence that retired it, and it is **not used by** :func:`grade`. Measured
over 510 graded MuJoCo runs (``adapters/mujoco/BRINGUP.md`` §3.5): it catches
the memoriser 15 % of the time at its declared 0.6 m and 56 % at 1.8 m, but
against an *optimal* fixed path it saturates at ~51 % near 3.0 m and then
declines -- a random displacement is as likely to move a box off the path as
onto it -- while past 1.8 m the layouts themselves become fatally illegal
(25 % at 3.0 m). Grade-time placement catches the same strongest fixed path in
one run far more often (measured; see ``meta.json``). Do not wire the
perturbation into a graded run.

Everything here is metres and seconds. Nothing in this module knows what a
simulator is, and every threshold is READ from
``tasks/R1_lidar_nav/meta.json`` -- there is no second copy to drift.
"""

from __future__ import annotations

import heapq
import json
import math
import random
from pathlib import Path

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Verdict)

TASK = "R1_lidar_nav"

TASK_DIR = Path(__file__).resolve().parents[1] / "tasks" / TASK


def _meta():
    """The task contract. Read, never duplicated."""
    return json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))


# --- thresholds: read from meta.json ----------------------------------------

_C = _meta()["constants"]

START_XY = tuple(float(v) for v in _C["START_XY"])
GOAL_XY = tuple(float(v) for v in _C["GOAL_XY"])
GOAL_TOL_M = float(_C["GOAL_TOL_M"])
ARENA_SIZE_M = float(_C["ARENA_SIZE_M"])
ARENA_HALF_M = float(_C["ARENA_HALF_M"])
N_OBSTACLES = int(_C["N_OBSTACLES"])
OBSTACLE_POSE_TOL_M = float(_C["OBSTACLE_POSE_TOL_M"])
STRAIGHT_LINE_M = float(_C["STRAIGHT_LINE_M"])
MAX_STEP_DISPLACEMENT_M = float(_C["MAX_STEP_DISPLACEMENT_M"])
LIDAR_MIN_FOV_DEG = float(_C["LIDAR_MIN_FOV_DEG"])
LIDAR_MIN_SAMPLES = int(_C["LIDAR_MIN_SAMPLES"])
LIDAR_MIN_RANGE_M = float(_C["LIDAR_MIN_RANGE_M"])
MIN_ROBOT_JOINTS = int(_C["MIN_ROBOT_JOINTS"])
#: Below this, the robot did not meaningfully move, and assertions that
#: only a MOVING robot can earn (R1.5's collision-free) are not credited.
MIN_MOTION_FOR_CREDIT_M = float(_C["MIN_MOTION_FOR_CREDIT_M"])

# -- grade-time placement (the authoritative anti-hardcode mechanism) --------

#: Every obstacle's footprint must lie inside this half-extent -- the inner
#: face of the arena wall, so no box is buried in a wall.
PLACEMENT_ARENA_INNER_M = float(_C["PLACEMENT_ARENA_INNER_M"])
#: Minimum surface-to-surface gap between any two obstacles.
PLACEMENT_MIN_GAP_M = float(_C["PLACEMENT_MIN_GAP_M"])
#: Minimum surface distance from every obstacle to the start and the goal, so
#: both poses are occupiable by a robot with room to turn.
PLACEMENT_ENDPOINT_CLEAR_M = float(_C["PLACEMENT_ENDPOINT_CLEAR_M"])
#: The obstacle inflation used to ask "does a route exist at all" -- the
#: tightest squeeze a plausible robot could physically take.
PLACEMENT_ROUTE_INFLATE_M = float(_C["PLACEMENT_ROUTE_INFLATE_M"])
#: Grid cell for both route questions (existence, and the R1.6 floor).
PLACEMENT_ROUTE_CELL_M = float(_C["PLACEMENT_ROUTE_CELL_M"])
#: How much of the straight start->goal line must lie INSIDE obstacles. A line
#: clipped by a centimetre of corner is "blocked" by the letter of the test and
#: proves nothing; this is the margin that makes a blind straight run collide.
PLACEMENT_MIN_BLOCK_M = float(_C["PLACEMENT_MIN_BLOCK_M"])
#: Re-sample budget. Exhausting it RAISES; it never falls back to a published
#: layout, because that fallback is precisely the defect this mechanism
#: replaces (it handed the memoriser its best case).
PLACEMENT_MAX_ATTEMPTS = int(_C["PLACEMENT_MAX_ATTEMPTS"])

# -- R1.6's derived path floor ----------------------------------------------

#: An 8-connected grid over-states a Euclidean distance by at most
#: sqrt(4 - 2*sqrt(2)) = 1.0824, so dividing by that (multiplying by 0.9239)
#: turns the grid route into a LOWER BOUND on the true shortest route.
PATH_FLOOR_GRID_FACTOR = float(_C["PATH_FLOOR_GRID_FACTOR"])
#: Allowance for discretising the obstacle edges (half a cell) and for the
#: recorder's own path-integral sampling error (~0.7 %, measured).
PATH_FLOOR_SLACK_M = float(_C["PATH_FLOOR_SLACK_M"])
#: How far inside a matched obstacle a recorded sample must be before R1.6
#: calls it "went THROUGH". A robot that grazes a box keeps its origin outside
#: it; this margin is for pose origins that are not the geometric centre.
THROUGH_TOL_M = float(_C["THROUGH_TOL_M"])
#: A body whose top is no higher than the robot's own underside (plus this) is
#: something the robot DROVE OVER -- a floor, a tile, a threshold -- and
#: touching it is not a collision.
DRIVE_OVER_TOL_M = float(_C["DRIVE_OVER_TOL_M"])

# -- superseded: the seeded perturbation ------------------------------------

#: SUPERSEDED by grade-time placement. Kept only so the sweep that retired it
#: still reproduces; :func:`grade` does not read it.
PERTURBATION_MAX_M = float(_C["PERTURBATION_MAX_M"])

#: The name the graded layout is declared under, by whoever placed it.
LAYOUT_SIDECAR = "r1_graded_layout.json"
#: ...and where to look for it when the grader runs in a SUBPROCESS whose run
#: directory the placement step could not know. The OmniSim arm grades through
#: ``run_agentbench.py --agent external``, which builds its own per-cell run
#: dir (and deletes anything already there), so the sidecar cannot be
#: pre-placed beside that run. This variable names the directory it WAS
#: written to. It is a benchmark-side channel only: ``cc_lane`` scrubs every
#: ``AGENTBENCH_*`` name out of the agent's environment, so the value never
#: reaches a session, and it carries a directory rather than a layout so the
#: sidecar on disk stays the single declaration.
LAYOUT_DIR_ENV = "AGENTBENCH_R1_LAYOUT_DIR"
#: What :func:`sample_layout` is, in one string, for the record on a row.
PLACEMENT_MECHANISM = "grade_time_placement/v1"

_ALL = [("R1.1", "the run is clean"),
        ("R1.2", "there is one drivable robot"),
        ("R1.3", "the benchmark obstacles are intact"),
        ("R1.4", "the goal is reached"),
        ("R1.5", "nothing was hit"),
        ("R1.6", "it drove around the obstacles, and did not teleport")]

_BASIS = {"R1.1": MIXED, "R1.2": MIXED, "R1.3": CORE_PHYSICAL,
          "R1.4": CORE_PHYSICAL, "R1.5": CORE_PHYSICAL,
          "R1.6": CORE_PHYSICAL}


class LayoutError(RuntimeError):
    """No legal layout could be drawn for a seed within the sample budget.

    Raised rather than papered over: a placement mechanism that silently
    substitutes a layout the agent has seen is the mechanism this one replaced.
    """


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length_xy(xy):
    """Integrated path length of an (N, 2) array of xy samples, in metres."""
    total = 0.0
    for i in range(1, len(xy)):
        total += math.hypot(float(xy[i][0]) - float(xy[i - 1][0]),
                            float(xy[i][1]) - float(xy[i - 1][1]))
    return total


def max_step_xy(xy):
    """Largest single-sample displacement, in metres. The teleport witness."""
    worst = 0.0
    for i in range(1, len(xy)):
        worst = max(worst, math.hypot(float(xy[i][0]) - float(xy[i - 1][0]),
                                      float(xy[i][1]) - float(xy[i - 1][1])))
    return worst


def obstacle_spec():
    """The PUBLISHED obstacle list, read from the task's own shipped asset.

    Read rather than duplicated: the agent and the grader must be looking at
    the same boxes, and a second copy in code is a second source of truth
    waiting to drift.

    What is contractually published is the **count and the sizes** -- see
    :func:`obstacle_sizes`. The positions in the asset are the layout the agent
    authors against; the layout it is GRADED on is drawn by
    :func:`sample_layout`.
    """
    p = TASK_DIR / "initial" / "benchmark_assets" / "obstacles.json"
    return json.loads(p.read_text(encoding="utf-8"))["obstacles"]


def obstacle_sizes(spec=None):
    """``[(name, (sx, sy, sz)), ...]`` -- the part of the layout that is
    published to the agent and is NOT redrawn at grade time."""
    return [(o["name"], tuple(float(v) for v in o["size"]))
            for o in (spec if spec is not None else obstacle_spec())]


class _SpecBox:
    """A ``graders.evidence.Body``-alike over one spec entry.

    So the geometric helpers read a *layout* with exactly the code that reads a
    *measured scene*; a second implementation is a second set of bugs.
    """

    __slots__ = ("name", "body_id", "_lo", "_hi")

    has_aabb = True

    def __init__(self, o):
        cx, cy = float(o["position"][0]), float(o["position"][1])
        sx, sy = float(o["size"][0]), float(o["size"][1])
        self.name = o["name"]
        self.body_id = o["name"]
        self._lo = (cx - sx / 2.0, cy - sy / 2.0, 0.0)
        self._hi = (cx + sx / 2.0, cy + sy / 2.0, float(o["size"][2]))

    @property
    def aabb(self):
        return (self._lo, self._hi)


def spec_bodies(layout):
    """Body-alikes for a layout, for the name-free geometric helpers."""
    return [_SpecBox(o) for o in layout]


def match_spec_obstacles(bodies, spec, tol_m=OBSTACLE_POSE_TOL_M):
    """Find the SPECIFIED obstacles by GEOMETRY, never by name.

    The first real R1 run made the case for this: the agent built a perfectly
    good scene -- compliant LiDAR, four-wheel rover, avoidance controller --
    and named its obstacles ``crate A``..``crate E``. A grader keyed on the
    published ``OBSTACLE_n`` names would have scored zero obstacles and failed
    a correct world, which is grading OUR CONVENTION instead of the task. The
    same rule the SPEC already states for robots (enumerate by type + measured
    properties, not by DEF) applies here.

    A body matches a spec entry when its world-space AABB centre is within
    ``tol_m`` of the specified centre on x and y AND its footprint is within
    ``tol_m`` of the specified extents. Each spec entry is matched at most
    once. Extra scenery the agent chose to add is neither required nor
    penalised -- it only makes the navigation harder for itself.

    ``spec`` is the layout the run is GRADED on, which since grade-time
    placement is not in general the published one.
    """
    unmatched = list(bodies)
    found, missing = [], []
    for want in spec:
        cx, cy = float(want["position"][0]), float(want["position"][1])
        sx, sy = float(want["size"][0]), float(want["size"][1])
        hit = None
        for b in unmatched:
            if not b.has_aabb:
                continue
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


def segment_blocked_by(bodies, start=START_XY, goal=GOAL_XY, samples=2000):
    """Which obstacle AABBs the straight start->goal segment passes through.

    Re-derived from the MEASURED scene rather than trusted from the asset
    file: if an agent authored the obstacles somewhere else, the sensing
    argument that R1.4-R1.6 rest on would silently stop holding, and a grader
    that assumed it would be scoring a claim instead of a fact.
    """
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


def segment_blocked_length(bodies, start=START_XY, goal=GOAL_XY, samples=2000):
    """How many metres of the straight start->goal line lie inside a body.

    "Blocked" as a boolean is satisfied by a corner clipped by a centimetre,
    which proves nothing about what a blind robot would hit. This is the
    quantity :data:`PLACEMENT_MIN_BLOCK_M` is a floor on.
    """
    span = _dist(start, goal)
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


# --- routes over a layout ----------------------------------------------------

def _grid(layout, inflate_m, cell_m):
    """``(n, blocked)`` -- a square occupancy grid over the arena.

    A cell is blocked when its centre is inside an obstacle or a wall, both
    inflated by ``inflate_m``. Deliberately NOT any navigator's own planner:
    this asks whether the LAYOUT admits a route, not whether one particular
    robot solves it.
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

    for o in layout:
        cx, cy = float(o["position"][0]), float(o["position"][1])
        sx, sy = float(o["size"][0]), float(o["size"][1])
        x0, x1 = cx - sx / 2.0 - inflate_m, cx + sx / 2.0 + inflate_m
        y0, y1 = cy - sy / 2.0 - inflate_m, cy + sy / 2.0 + inflate_m
        i0 = max(0, int(math.floor((x0 + ARENA_HALF_M) / cell_m - 0.5)))
        i1 = min(n - 1, int(math.ceil((x1 + ARENA_HALF_M) / cell_m - 0.5)))
        j0 = max(0, int(math.floor((y0 + ARENA_HALF_M) / cell_m - 0.5)))
        j1 = min(n - 1, int(math.ceil((y1 + ARENA_HALF_M) / cell_m - 0.5)))
        for i in range(i0, i1 + 1):
            if not (x0 <= centre(i) <= x1):
                continue
            row = blocked[i]
            for j in range(j0, j1 + 1):
                if y0 <= centre(j) <= y1:
                    row[j] = 1
    return n, blocked


def _cell_of(pt, n, cell_m):
    return (min(max(int((pt[0] + ARENA_HALF_M) / cell_m), 0), n - 1),
            min(max(int((pt[1] + ARENA_HALF_M) / cell_m), 0), n - 1))


def route_exists(layout, inflate_m=PLACEMENT_ROUTE_INFLATE_M,
                 cell_m=PLACEMENT_ROUTE_CELL_M):
    """Is there ANY collision-free route from start to goal in this layout?"""
    n, blocked = _grid(layout, inflate_m, cell_m)
    src = _cell_of(START_XY, n, cell_m)
    dst = _cell_of(GOAL_XY, n, cell_m)
    if blocked[src[0]][src[1]] or blocked[dst[0]][dst[1]]:
        return False
    seen = [bytearray(n) for _ in range(n)]
    seen[src[0]][src[1]] = 1
    stack = [src]
    while stack:
        i, j = stack.pop()
        if (i, j) == dst:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < n and not blocked[a][b] \
                    and not seen[a][b]:
                seen[a][b] = 1
                stack.append((a, b))
    return False


def shortest_route_m(layout, inflate_m=0.0, cell_m=PLACEMENT_ROUTE_CELL_M):
    """Length of the shortest collision-free route, metres, or ``None``.

    8-connected Dijkstra. ``inflate_m = 0`` is the POINT robot, which is what
    :func:`min_path_length` wants: every route a real robot of any size can
    take is also open to a point, so the point's shortest route is a bound no
    honest navigator can undercut.
    """
    n, blocked = _grid(layout, inflate_m, cell_m)
    src = _cell_of(START_XY, n, cell_m)
    dst = _cell_of(GOAL_XY, n, cell_m)
    if blocked[src[0]][src[1]] or blocked[dst[0]][dst[1]]:
        return None
    diag = math.sqrt(2.0)
    best = {src: 0.0}
    heap = [(0.0, src)]
    while heap:
        d, cur = heapq.heappop(heap)
        if d > best.get(cur, float("inf")):
            continue
        if cur == dst:
            return d * cell_m
        i, j = cur
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                a, b = i + di, j + dj
                if not (0 <= a < n and 0 <= b < n) or blocked[a][b]:
                    continue
                nd = d + (diag if di and dj else 1.0)
                if nd < best.get((a, b), float("inf")):
                    best[(a, b)] = nd
                    heapq.heappush(heap, (nd, (a, b)))
    return None


def min_path_length(layout):
    """R1.6's path floor, in metres, derived from the layout being SCORED.

    The fixed 11.5 m this replaces was arithmetic about ONE layout -- "the
    straight line is 11.31 m and is blocked" -- and it punished honest
    navigators the moment the layout changed: five oracle runs arrived within
    0.10 m of the goal, collision-free, on 11.31-11.49 m paths and FAILED R1.6
    for taking a legitimately shorter legal route (BRINGUP.md §3.5, finding 2).

    So the floor is a **lower bound on the shortest legal route** and nothing
    more: the point-robot grid route, corrected for the 8-connected metric,
    less the discretisation slack and the goal tolerance a run is allowed to
    stop short by. By construction no collision-free arrival can fall under
    it, which is the property defect B lacked. The teleport clause and the
    "went THROUGH a box" clause are what actually discriminate in R1.6; this
    one is a floor, and it is honest about being one.
    """
    grid = shortest_route_m(layout, inflate_m=0.0)
    if grid is None:                       # boxed in; nothing to bound
        grid = STRAIGHT_LINE_M
    return max(0.0, PATH_FLOOR_GRID_FACTOR * grid - PATH_FLOOR_SLACK_M
               - GOAL_TOL_M)


# --- GRADE-TIME PLACEMENT ----------------------------------------------------

def _aabb_xy(o):
    cx, cy = float(o["position"][0]), float(o["position"][1])
    sx, sy = float(o["size"][0]), float(o["size"][1])
    return (cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0)


def _box_point_gap(o, pt):
    x0, y0, x1, y1 = _aabb_xy(o)
    return math.hypot(max(x0 - pt[0], 0.0, pt[0] - x1),
                      max(y0 - pt[1], 0.0, pt[1] - y1))


def _box_box_gap(a, b):
    ax0, ay0, ax1, ay1 = _aabb_xy(a)
    bx0, by0, bx1, by1 = _aabb_xy(b)
    return math.hypot(max(bx0 - ax1, 0.0, ax0 - bx1),
                      max(by0 - ay1, 0.0, ay0 - by1))


def layout_legality(layout):
    """Every acceptance criterion for a graded layout, evaluated separately.

    Named individually rather than reduced to a boolean, so a rejected draw
    says WHICH rule it broke and a test can assert the rules one at a time.
    """
    out = {"outside_arena": [], "too_close": [], "near_start": [],
           "near_goal": []}
    for o in layout:
        x0, y0, x1, y1 = _aabb_xy(o)
        if (min(x0, y0) < -PLACEMENT_ARENA_INNER_M
                or max(x1, y1) > PLACEMENT_ARENA_INNER_M):
            out["outside_arena"].append(o["name"])
        if _box_point_gap(o, START_XY) < PLACEMENT_ENDPOINT_CLEAR_M:
            out["near_start"].append(o["name"])
        if _box_point_gap(o, GOAL_XY) < PLACEMENT_ENDPOINT_CLEAR_M:
            out["near_goal"].append(o["name"])
    for i, a in enumerate(layout):
        for b in layout[i + 1:]:
            if _box_box_gap(a, b) < PLACEMENT_MIN_GAP_M:
                out["too_close"].append("%s/%s" % (a["name"], b["name"]))

    bodies = spec_bodies(layout)
    out["blocking_straight_line"] = segment_blocked_by(bodies)
    out["blocked_length_m"] = round(segment_blocked_length(bodies), 4)
    out["route_exists"] = route_exists(layout)
    out["legal"] = (not out["outside_arena"] and not out["too_close"]
                    and not out["near_start"] and not out["near_goal"]
                    and out["route_exists"]
                    and out["blocked_length_m"] >= PLACEMENT_MIN_BLOCK_M)
    return out


def _cheaply_legal(layout):
    """The constraints that need no grid: arena, endpoint clearance, spacing.

    A pre-filter for the sampler only. :func:`layout_legality` remains the one
    authority on whether a layout is legal, and it re-checks all of these.
    """
    for i, o in enumerate(layout):
        x0, y0, x1, y1 = _aabb_xy(o)
        if (min(x0, y0) < -PLACEMENT_ARENA_INNER_M
                or max(x1, y1) > PLACEMENT_ARENA_INNER_M):
            return False
        if _box_point_gap(o, START_XY) < PLACEMENT_ENDPOINT_CLEAR_M:
            return False
        if _box_point_gap(o, GOAL_XY) < PLACEMENT_ENDPOINT_CLEAR_M:
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
    layout: an illegal draw is discarded and redrawn, so "a route exists" and
    "the straight line is still blocked" are acceptance criteria rather than
    hopes. There is no fallback -- exhausting the budget raises
    :class:`LayoutError`, since substituting a layout the agent has already
    seen is exactly the escape hatch this mechanism was built to close.
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
        # The cheap rules first, so a draw that is obviously illegal costs a
        # few distance computations instead of a flood fill. The verdict is
        # still layout_legality's -- this only decides what is worth asking.
        if not _cheaply_legal(layout):
            continue
        legal = layout_legality(layout)
        if legal["legal"]:
            return layout, {"seed": seed, "attempts": attempt,
                            "mechanism": PLACEMENT_MECHANISM,
                            "legality": legal}
    raise LayoutError(
        "no legal layout for seed %r in %d attempts; the constraints "
        "(arena %.2f m, gap %.2f m, endpoints %.2f m, route, %.2f m of the "
        "straight line blocked) may be unsatisfiable for these sizes"
        % (seed, max_attempts, PLACEMENT_ARENA_INNER_M, PLACEMENT_MIN_GAP_M,
           PLACEMENT_ENDPOINT_CLEAR_M, PLACEMENT_MIN_BLOCK_M))


def sample_layout(seed, sizes=None, max_attempts=PLACEMENT_MAX_ATTEMPTS):
    """The GRADED obstacle layout for one benchmark seed. See
    :func:`sample_layout_with_report`."""
    return sample_layout_with_report(seed, sizes=sizes,
                                     max_attempts=max_attempts)[0]


# --- how a run declares the layout it was graded on -------------------------

def write_graded_layout(directory, seed, layout=None):
    """Declare, on disk, the layout a run is being graded against.

    The placement step's half of the handshake: whoever injects the obstacle
    poses into the agent's world writes this next to the run, and
    :func:`resolve_graded_layout` reads it back. One file, one name, so the
    grader never has to infer the layout from the world it is scoring.
    """
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

    Precedence, most explicit first: an argument, then a sidecar written by
    the placement step, then a seed the layout is drawn from. ``(None, None)``
    means no grade-time placement happened, and :func:`grade` says so out loud
    rather than quietly scoring the published layout as if it proved anything.
    """
    if layout:
        return list(layout), "argument"
    found, source = read_graded_layout(*directories)
    if found:
        return found, source
    if seed is not None:
        return sample_layout(seed), "seed %s" % (seed,)
    return None, None


# --- what counts as a collision ---------------------------------------------

def _covers_xy(body, pt):
    lo, hi = body.aabb
    return lo[0] <= pt[0] <= hi[0] and lo[1] <= pt[1] <= hi[1]


def hittable_bodies(bodies, robot, matched):
    """Which bodies count as a COLLISION when the robot touches them.

    R1.5 used to take this set from the MATCHED spec obstacles alone, and that
    is defect A: move the obstacles and nothing matches, so the hit set is
    empty and a robot-obstacle contact **cannot be counted**. Measured over 30
    seeds at 1.8 m of perturbation, the memorising driver physically struck an
    obstacle in 20 runs and R1.5 reported 0 collisions in every one of them
    (BRINGUP.md §3.5, finding 3). Scoring the graded layout fixes the common
    case; this function fixes the assertion, so that a contact is countable
    even on a run whose obstacles never matched anything.

    Three ways in, none of which is a name convention:

    * a MATCHED obstacle of the graded layout -- the primary channel;
    * a body whose name says *wall* -- kept, because a wall is scenery no spec
      entry describes and its name is the only handle any arm offers;
    * any other non-robot body with world bounds that the robot could not have
      **driven over**: its top is above the robot's own underside and it does
      not span both the start and the goal (a floor does both).

    A body with no bounds is NOT counted from the geometric channel and is
    reported instead: on upstream Webots a ``Mesh`` hull arrives without bounds
    (BRINGUP.md), and failing an honest agent for touching a body we could not
    measure would be scoring our instrument.
    """
    ground_z = 0.0
    if robot is not None and getattr(robot, "has_aabb", False):
        ground_z = float(robot.aabb[0][2])

    names, unbounded = set(), []
    matched_ids = {id(b) for b in matched}
    for b in matched:
        for key in (b.name, b.body_id):
            if key:
                names.add(str(key).lower())
    for b in bodies:
        if getattr(b, "robot_class", False) or getattr(b, "member_of", None):
            continue
        if robot is not None and b.body_id == robot.body_id:
            continue
        label = (b.name or b.body_id or "")
        if "wall" in label.lower():
            names.add(label.lower())
            continue
        if id(b) in matched_ids:
            continue
        if not b.has_aabb:
            unbounded.append(label)
            continue
        if b.top_z is not None and float(b.top_z) <= ground_z + DRIVE_OVER_TOL_M:
            continue                                    # drove over it
        if _covers_xy(b, START_XY) and _covers_xy(b, GOAL_XY):
            continue                                    # it IS the ground
        for key in (b.name, b.body_id):
            if key:
                names.add(str(key).lower())
    return names, unbounded


def samples_inside(xy, bodies, tol_m=THROUGH_TOL_M):
    """Which measured obstacles the robot's own track passes THROUGH.

    "It drove around" asserted directly, from the graded layout's measured
    geometry, instead of inferred from a path length. A robot that grazes a box
    keeps its origin outside it; an origin that ends up ``tol_m`` INSIDE one
    went through it -- which is what a world whose obstacles have no collision
    geometry, or a pose written rather than driven, looks like.
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
            if x0 <= float(p[0]) <= x1 and y0 <= float(p[1]) <= y1:
                through.append(b.name or b.body_id)
                break
    return through


# --- SUPERSEDED: the seeded perturbation ------------------------------------

def seeded_offsets(spec, seed, max_m=PERTURBATION_MAX_M):
    """SUPERSEDED by :func:`sample_layout`. Deterministic per-obstacle offsets.

    Kept so ``adapters/mujoco/r1_perturbation_sweep.py`` still reproduces the
    510 graded runs that retired this mechanism. :func:`grade` does not call
    it. See the module docstring for why it cannot be tuned into working.
    """
    rng = random.Random("R1_lidar_nav/%s" % seed)
    out = {}
    for o in spec:
        # polar sampling keeps the shift inside a disc of radius max_m rather
        # than a square, so no obstacle can move further diagonally than the
        # declared bound
        r = max_m * math.sqrt(rng.random())
        th = rng.uniform(0.0, 2.0 * math.pi)
        out[o["name"]] = (round(r * math.cos(th), 4), round(r * math.sin(th), 4))
    return out


def perturbed_spec(spec, seed, max_m=PERTURBATION_MAX_M):
    """SUPERSEDED by :func:`sample_layout`. ``spec`` with the offsets applied.

    Its bail-out is the reason it is superseded rather than retuned: a layout
    whose straight line is no longer blocked is replaced by the PUBLISHED
    layout, which is the memoriser's best case, and that fallback rate climbs
    with the displacement. Grade-time placement makes the same condition an
    acceptance criterion instead.
    """
    offsets = seeded_offsets(spec, seed, max_m)
    moved = []
    for o in spec:
        dx, dy = offsets[o["name"]]
        q = dict(o)
        q["position"] = [o["position"][0] + dx, o["position"][1] + dy,
                         o["position"][2]]
        moved.append(q)
    if not segment_blocked_by(spec_bodies(moved)):
        return spec, offsets, False
    return moved, offsets, True


def grade(bundle, *, self_verified=False, graded_layout=None,
          layout_source=None):
    """Score one run.

    ``graded_layout`` is the layout the world was PLACED with -- the output of
    :func:`sample_layout` for the run's seed. Every geometric assertion is
    scored against it: R1.3 matches it, R1.5 takes its hit set from what
    matched it, and R1.6 derives its floor and its "went through" check from
    it. Passing the published layout here instead was defect A, which blinded
    R1.5 on any run whose obstacles had moved.

    Omitting it means **no grade-time placement happened**. The run is then
    scored against the published layout -- the only thing left to score it
    against -- and the verdict carries a note saying the run contains no
    evidence against a memorising agent, because it does not.
    """
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)
    L = bundle.lbl

    if graded_layout:
        spec = list(graded_layout)
        placed = True
        source = layout_source or "graded layout (argument)"
    else:
        spec = obstacle_spec()
        placed = False
        source = "PUBLISHED layout -- no grade-time placement"
        v.note("no grade-time obstacle placement was applied to this run: it "
               "was scored against the layout the task publishes, which the "
               "agent has seen. A controller that planned a fixed path from "
               "benchmark_assets/obstacles.json and cast no sensor beam "
               "passes every assertion here (measured). This row is not "
               "evidence of perception.")
    v.measurements["r1_graded_layout"] = {
        "placed": placed, "source": source,
        "obstacles": [{"name": o["name"],
                       "position": [round(float(c), 4)
                                    for c in o["position"]]}
                      for o in spec]}

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
                 "no physics-engine attribution: the adapter could not name "
                 "the engine that drove this run from any channel it has."))
        return v

    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    v.add("R1.1", bool(proc and proc.clean and complete), "the run is clean",
          measured={L("exit_code", "exit code"): proc.exit_code if proc
                    else None,
                    L("error_lines", "error lines"): len(errs),
                    L("driver_completed", "driver reached a clean quit"):
                        complete},
          threshold={L("exit_code", "exit code"): 0,
                     L("error_lines", "error lines"): 0},
          detail="; ".join(errs[:3]), basis=MIXED)

    # --- R1.2 one drivable robot ------------------------------------------
    robots = [b for b in bundle.roster.bodies if b.robot_class]
    drivable = [b for b in robots
                if (b.n_joints or 0) >= MIN_ROBOT_JOINTS]
    v.add("R1.2", len(drivable) == 1, "there is one drivable robot",
          measured={L("robot_bodies", "robot-class bodies"): len(robots),
                    L("with_joints", "of those, with >= %d joints"
                      % MIN_ROBOT_JOINTS): len(drivable),
                    L("names", "names"): [b.name for b in robots][:6]},
          threshold={L("with_joints", "robots with enough joints"): 1},
          basis=MIXED)

    # --- R1.3 the obstacles are intact ------------------------------------
    # Matched by GEOMETRY, not by name: an agent that builds the specified
    # scene and calls the boxes something else has still built the specified
    # scene (measured, first R1 run: "crate A".."crate E"). Matched against the
    # GRADED layout, which is the one the run was placed with.
    non_robots = [b for b in bundle.roster.bodies if not b.robot_class]
    scannable = [b for b in non_robots if b.has_aabb]
    obstacles, missing = match_spec_obstacles(non_robots, spec)
    blocked_by = segment_blocked_by(obstacles)
    # An agent that named its obstacles something of its own leaves the
    # NAME-KEYED bounds scan with nothing to record, so the grader sees no
    # candidates at all. That is an instrument gap, not an agent failure, and
    # saying so is the difference between a measurement and an accusation --
    # measured 2026-08-09, when R1.3 was scored against both arms while it
    # could not pass on either.
    if not scannable:
        v.add("R1.3", False, "the benchmark obstacles are intact",
              measured={L("scannable_bodies",
                          "non-robot bodies with world bounds"): 0},
              threshold={L("scannable_bodies",
                           "non-robot bodies with world bounds"):
                         ">= %d" % N_OBSTACLES},
              detail=("INSTRUMENT GAP, not an agent failure: no non-robot "
                      "body was recorded with world-space bounds, so the "
                      "obstacles could not be measured at all. The bounds "
                      "scan is name-keyed while this assertion matches by "
                      "geometry; a world using its own names is invisible "
                      "to it."),
              basis=CORE_PHYSICAL)
        v.note(L("r1_3_instrument_gap",
                 "R1.3 was not decidable on this run: the obstacles were "
                 "never scanned into the t=0 inventory."))
    else:
        v.add("R1.3", len(obstacles) == N_OBSTACLES and bool(blocked_by),
              "the benchmark obstacles are intact",
              measured={L("obstacles_found", "specified obstacles found"):
                        len(obstacles),
                        L("obstacles_missing", "specified obstacles missing"):
                            missing,
                        L("blocking_straight_line",
                          "of those, blocking the straight line"): blocked_by,
                        L("layout_scored", "layout scored against"): source},
              threshold={L("obstacles_found", "specified obstacles found"):
                         N_OBSTACLES,
                         L("blocking_straight_line",
                           "the straight line must be blocked"): ">= 1"},
              detail=("matched by position and footprint within %.2f m, never "
                      "by name, against the layout this run was GRADED on; "
                      "the sensing argument rests on the direct path being "
                      "blocked, re-derived here from measured geometry"
                      % OBSTACLE_POSE_TOL_M),
              basis=CORE_PHYSICAL)

    if not drivable:
        for aid in ("R1.4", "R1.5", "R1.6"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail="no drivable robot to measure")
        return v.finish()

    # The ROSTER and the TRAJECTORY are different index spaces: the roster
    # holds every body (robot, obstacles, scenery), the trajectory holds only
    # the tracked ones. Indexing xyz with a roster position is a bug that
    # happens to work whenever the two orders coincide and silently measures
    # the WRONG BODY when they do not -- measured 2026-08-09, webots R1:
    # "index 11 is out of bounds for axis 0 with size 2". Map through the
    # body id, which is the invariant the adapter actually guarantees.
    robot = drivable[0]
    idx = traj.index_of(robot.body_id)
    if idx is None and getattr(traj, "names", None):
        idx = traj.index_of_name(robot.name) if robot.name else None
    if idx is None:
        for aid in ("R1.4", "R1.5", "R1.6"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  measured={L("robot_body_id", "robot body id"):
                            robot.body_id,
                            L("tracked_ids", "tracked body ids"):
                                list(traj.body_ids)[:8]},
                  detail=("the drivable robot has no row in the recorded "
                          "trajectory, so its motion was never measured; "
                          "this is an instrument gap, not an agent failure"))
        return v.finish()
    xy = traj.xy(idx)

    # --- R1.4 the goal is reached -----------------------------------------
    final = (float(xy[-1][0]), float(xy[-1][1]))
    d_goal = _dist(final, GOAL_XY)
    v.add("R1.4", d_goal <= GOAL_TOL_M, "the goal is reached",
          measured={L("final_xy", "final xy (m)"): [round(final[0], 4),
                                                    round(final[1], 4)],
                    L("distance_to_goal", "distance to goal (m)"):
                        round(d_goal, 4)},
          threshold={L("distance_to_goal", "distance to goal (m)"):
                     "<= %.2f" % GOAL_TOL_M},
          basis=CORE_PHYSICAL)

    # --- R1.5 nothing was hit ---------------------------------------------
    con = bundle.contacts
    robot_keys = {str(k).lower() for k in (robot.name, robot.body_id) if k}
    hittable, unbounded = hittable_bodies(non_robots, robot, obstacles)

    def _is_hit(p):
        a = (p.a or "").lower()
        b = (p.b or "").lower()
        pair = {a, b}
        if not (pair & robot_keys):
            return False
        other = pair - robot_keys
        return bool(other) and next(iter(other)) in hittable

    hits = [p for p in (con.pairs if con else []) if _is_hit(p)]
    # A simulator that cannot answer the contact query at all must not read as
    # "nothing was hit" -- that is the difference between checking and not
    # being able to check, and only one of them supports a PASS.
    if con is None or not con.supported:
        v.add("R1.5", False, "nothing was hit",
              measured={L("contact_query", "contact query supported"): False},
              threshold={L("contact_query", "contact query supported"): True},
              detail=("the adapter cannot answer contacts on this simulator, "
                      "so collision-free CANNOT be established; unmeasured is "
                      "never a pass"),
              basis=CORE_PHYSICAL)
    else:
        # "Nothing was hit" has to be EARNED. A robot that never moved hits
        # nothing, and crediting that is a free pass handed to exactly the
        # runs that did least -- measured on R1's first graded run, which
        # collected this PASS with a path length of 0.0 m because its
        # controller had not been collected with the world.
        moved = path_length_xy(xy) >= MIN_MOTION_FOR_CREDIT_M
        v.add("R1.5", (not hits) and moved, "nothing was hit",
              measured={L("collisions", "robot-obstacle/wall contacts"):
                        len(hits),
                        L("first_hits", "first hits"):
                            ["%s/%s" % (p.a, p.b) for p in hits[:3]],
                        L("moved_m", "distance actually travelled (m)"):
                            round(path_length_xy(xy), 4),
                        L("hittable", "bodies a touch would count against"):
                            len(hittable),
                        L("unbounded", "bodies with no bounds to judge"):
                            unbounded[:4]},
              threshold={L("collisions", "robot-obstacle/wall contacts"): 0,
                         L("moved_m", "distance actually travelled (m)"):
                             ">= %.2f" % MIN_MOTION_FOR_CREDIT_M},
              detail=("collision-free is only a result if the robot was ever "
                      "at risk; a stationary robot earns nothing here. The "
                      "hit set is built from the GRADED layout's matched "
                      "obstacles plus every non-robot body the robot could "
                      "not have driven over, so a contact is countable even "
                      "when nothing matched"),
              basis=CORE_PHYSICAL)

    # --- R1.6 it drove around, and did not teleport ------------------------
    length = path_length_xy(xy)
    worst_step = max_step_xy(xy)
    start_err = _dist((float(xy[0][0]), float(xy[0][1])), START_XY)
    floor = min_path_length(spec)
    through = samples_inside(xy, obstacles)
    ok6 = (length >= floor and worst_step <= MAX_STEP_DISPLACEMENT_M
           and not through)
    v.add("R1.6", ok6, "it drove around the obstacles, and did not teleport",
          measured={L("path_length", "integrated path length (m)"):
                    round(length, 4),
                    L("max_step", "largest single-sample step (m)"):
                        round(worst_step, 4),
                    L("start_error", "distance from the declared start (m)"):
                        round(start_err, 4),
                    L("through", "obstacles the track passes through"):
                        through},
          threshold={L("path_length", "integrated path length (m)"):
                     ">= %.2f" % floor,
                     L("max_step", "largest single-sample step (m)"):
                         "<= %.2f" % MAX_STEP_DISPLACEMENT_M,
                     L("through", "obstacles the track passes through"): 0},
          detail=("the floor is a LOWER BOUND on the shortest legal route in "
                  "the layout this run was graded on (a fixed floor punished "
                  "honest navigators on any other layout); the step bound is "
                  "the teleport witness, and 'went through' is 'it drove "
                  "AROUND' asserted directly from measured geometry"),
          basis=CORE_PHYSICAL)

    return v.finish()
