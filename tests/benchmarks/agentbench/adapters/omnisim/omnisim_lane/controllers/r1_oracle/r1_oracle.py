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

"""R1 ORACLE driver -- the known-good solution, on the OmniSim arm.

**MEASURED 6/6 on 2026-08-11**, on the unmodified grader, in the fixtures
``test_r1_discriminates_omnisim.py`` uses (60 s window, ``contact_steps 0``):
reached the goal with **0.151 m** to spare of a 0.30 m tolerance, **zero**
robot-obstacle contacts, **14.27 m** of integrated path against a 10.77 m floor
and a 12.25 m point-optimal route, largest single recorded step **0.0084 m**
(the teleport bound is 0.25 m), and **227 occupied cells learned from an empty
grid**. It arrives at t = 33.9 s and holds station for the rest of the window.

⚠ **THE HARD PART WAS NOT THE NAVIGATION, AND ANYONE TOUCHING THE CONTROL
CONSTANTS BELOW NEEDS THE ACTUATION CONTRACT FIRST.** See the block above
``CMD_PERIOD_S``: a wheel command held CONSTANT is tracked exactly, but every
CHANGE kicks the chassis to ~10x the target and the kicks ACCUMULATE, so a
navigator that republishes every tick never leaves the transient. The first
version of this driver did exactly that and flipped the rover onto its back at
t = 15.5 s. The measurements that price it are in this file, not in a report,
because the next person to raise a gain here will re-derive them otherwise.

Paired with ``omnisim_lane/worlds/r1_oracle.wbt``. It exists for one purpose:
SPEC 7.1's gate says an oracle performing the known-good solution must PASS
every task it is run on, and until that has been shown for a given arm, that
arm's pass rate is unfalsifiable -- exactly the defect C2 shipped with, where
a task nobody had proven could FAIL passed 5/5 unfixed for a whole campaign.
The gate is per (task, arm) and R1 had it on neither of ours.

**How it senses, stated exactly.** The engine's own ``Lidar`` device: one
planar scan of ``SCAN_BEAMS`` = 181 beams over ``fieldOfView`` = 180 deg out
to 6 m, read every control tick with ``Lidar.getRangeImage()``. That is the
node the task's prompt asks for (>= 180 deg, >= 180 samples, >= 5 m) and it is
cleared by the DEVICE's own declared fields, not by an assertion about them --
the driver reads ``getHorizontalResolution()`` / ``getFov()`` / ``getMaxRange()``
back off the device and records them.

Two conventions this depends on were MEASURED, not assumed
(``controllers/r1_probe/`` + ``worlds/r1_probe.wbt``, run 2026-08-09):

  * **beam 0 is the LEFT end of the fan.** A post at a known +1.0708 rad
    bearing landed on index 28 of 181; the mirror convention predicts 151.
    So ``theta_i = fov/2 - (i + 0.5) * fov / res`` -- bin CENTRES, decreasing
    with index. Get this backwards and the robot builds a mirror-image map and
    drives confidently into the obstacle it thinks it is avoiding.
  * **``InertialUnit.getRollPitchYaw()[2]`` is world yaw about +z, from +x**,
    i.e. directly comparable with ``atan2(dy, dx)``: the probe's rover was
    authored at ``rotation 0 0 1 0.5`` and the IMU read 0.49999999.

Ranges are RADIAL distances to the surface (the probe's post, a 0.2 m box
centred 3.0 m away, read 2.900 m -- its near face), which is the engine's
documented lidar behaviour and what the occupancy update below assumes.

**What it knows and what it does not.** It is given what the task gives every
agent: the arena is 10 x 10 m, the start is (-4, -4) and the goal is (4, 4).
It is NOT given the obstacles. ``benchmark_assets/obstacles.json`` is never
read here, no obstacle pose appears anywhere in this file, and the occupancy
grid the planner runs on starts **empty** -- every occupied cell in it was put
there by a beam that came back short. The straight line from start to goal is
blocked by three of the five obstacles, so a driver that did not perceive
could not produce this run; and one that had memorised the published layout
would be caught the moment the layout moved, which is what
``test_r1_discriminates_omnisim.py`` does to this file.

**The method** is the textbook one for "navigate a partially known space", and
is deliberately the same one the MuJoCo arm's oracle uses, so a difference
between the arms is a difference between the SIMULATORS:

  1. cast the fan, and SCORE the cell each trusted short return landed in, and
     un-score every cell the beam flew through on the way -- a cell a later
     beam sees past is cleared, so the map corrects itself;
  2. inflate the occupied set by the robot's radius plus a margin, so the
     planner can treat the robot as a point;
  3. A* to the goal over the inflated grid, treating never-seen cells as free
     -- optimism about the unknown is what makes it explore rather than freeze
     -- and replan at 1 Hz, so a newly seen obstacle changes the plan instead
     of being driven into;
  4. follow the plan with pure pursuit, publishing wheel targets at 4 Hz under
     the slew limits the actuation contract requires, and park in the goal.

Two of those four are not the textbook version, and both changed because a
measurement said so. The map is a SCORE map with free-space carving because
the add-only set it replaced ended a run believing 1303 cells were occupied of
which **70 % were phantom** -- returns taken while the chassis was tilted, when
a 0.24 m planar scanner sees the floor -- at which point A* found no route at
all and the driver followed a stale plan into the two obstacles it had
correctly mapped. And the pursuit target is a point at a fixed arc length
rather than the next far-enough vertex, because string-pulled vertices move
when the map does: the vertex form sent the target jumping between opposite
ends of the arena and the rover spent a whole window turning on the spot.

Nothing here writes a pose. Every metre of the path is integrated by the
wheels, so R1.6's teleport bound (<= 0.25 m between samples) is satisfied by
roughly two orders of magnitude, which is what a physical drive looks like.

**It reports through a JSON sidecar, not stdout.** ``omnisim-bin.exe`` is a
GUI-subsystem binary on Windows and a controller's stdout goes nowhere a
caller can read -- measured on this lane's first probe run, whose every print
vanished while the engine log recorded a clean controller exit. The sidecar
path is ``$AGENTBENCH_R1_ORACLE_OUT``, defaulting beside this file.

The module imports nothing from the engine at import time, so a test can
import it for its declared geometry (``ROBOT_RADIUS_M``, ``SCAN_BEAMS``)
without a simulator.
"""

from __future__ import annotations

import heapq
import json
import math
import os

# --- what the task hands every agent ----------------------------------------
ARENA_HALF_M = 5.0
START_XY = (-4.0, -4.0)
GOAL_XY = (4.0, 4.0)
GOAL_STOP_M = 0.15          # half the task's 0.30 m tolerance
#: Once parked, the rover only creeps back if it slips past this. Wider than
#: GOAL_STOP_M so arriving does not immediately re-trigger a correction, and
#: still inside the task's tolerance so a run that ends here has passed.
GOAL_HOLD_M = 0.22
#: The parked creep, in wheel rad/s -- 0.16 m/s, which stops in 0.08 m at the
#: deceleration this rover has, i.e. inside the tolerance it is correcting to.
CREEP_RPS = 2.0

# --- the LiDAR, as declared on the rover (the task's minima are
#     180 deg / 180 samples / 5 m). Read back off the device at run time; these
#     are what the world authored.
SCAN_BEAMS = 181
SCAN_FOV_DEG = 180.0
SCAN_RANGE_M = 6.0

# --- the robot, as built in worlds/r1_oracle.wbt -----------------------------
WHEEL_RADIUS_M = 0.08
#: Geometric track is 0.30 m. A SKID steer scrubs its wheels sideways to turn,
#: so the track that converts a commanded yaw rate into a wheel-speed
#: difference is effectively wider. MEASURED on this rover rather than guessed
#: (turn-in-place ladder, 2026-08-11): a differential wheel command of ``a``
#: rad/s produces a yaw rate of **0.19 a** rad/s, linear from a = 1 to 4
#: (0.139 / 0.348 / 0.532 / 0.756), which is 2 R / 0.19 = 0.84 m of effective
#: track -- twice the 0.42 that was assumed here. Turning in place also drifts
#: **0.000 m**, which is the fact the whole actuation design below rests on.
WHEEL_BASE_EFF_M = 0.84
#: Circumscribed radius of chassis + wheels: the chassis corner is at
#: hypot(0.17, 0.13) = 0.214 m and the outer wheel corner at
#: hypot(0.12 + 0.08, 0.15 + 0.025) = 0.266 m, so 0.27 m bounds the robot.
ROBOT_RADIUS_M = 0.27
#: ⚠ **THE HARD CEILING ON EVERY WHEEL COMMAND.** The motors declare
#: ``maxVelocity 12``; commanding *at or near* that cap does not saturate, it
#: RUNS AWAY (documented in ``test_r1_discriminates_omnisim.py``). The first
#: version of this file computed ``(v +/- w * base/2) / R`` and published it
#: raw: with ``v = 0.8`` and ``w = 2.0`` that is **15.25 rad/s**, over the cap
#: before the arithmetic even starts. Measured consequence, 2026-08-11: 122 of
#: 2166 control ticks commanded >= 12 rad/s, the chassis left the floor
#: (z peaked at 0.677 m on a rover whose origin rides at 0.0), and at t = 15.5 s
#: the rover **flipped onto its back** (roll = pi) and spent the remaining 36 s
#: spinning its wheels in the air 5.80 m from the goal.
WHEEL_CMD_MAX_RPS = 9.0
#: Cruise, 0.480 m/s. A wheel command held CONSTANT tracks its rolling speed
#: exactly -- measured over a ladder from 3 to 10 rad/s, the steady ground
#: speed was ``0.08 * cmd`` to three decimals every time. It is 6.0 rather
#: than the 8.0 the time budget would prefer because the braking distance
#: scales with the square of it and this rover decelerates at 0.16 m/s^2: from
#: 0.640 m/s it needs 1.28 m to stop, which is more clearance than the plan
#: leaves beside an obstacle.
CRUISE_RPS = 6.0
#: The scan plane's height above the floor, and the Lidar's own vertical
#: half-angle -- both read off the ``Lidar`` node in the world. Together they
#: say how far a beam can be trusted at a given chassis tilt: see
#: :func:`trust_range_m`.
LIDAR_HEIGHT_M = 0.24
LIDAR_VFOV_HALF_RAD = 0.025

MOTORS_LEFT = ("left front motor", "left rear motor")
MOTORS_RIGHT = ("right front motor", "right rear motor")

# --- planner ----------------------------------------------------------------
GRID_CELL_M = 0.2
#: Robot radius, plus the grid's own quantisation error (a return is snapped to
#: a 0.2 m cell whose centre can sit 0.14 m INSIDE the surface that produced
#: it), plus margin for pure-pursuit tracking error and for a skid steer's
#: turn-out. An oracle that passes on a 2 cm margin has not shown a task is
#: passable.
INFLATE_M = 0.80
#: Tried in order when the planner cannot find a route at INFLATE_M. Every
#: fallback still exceeds the 0.27 m robot radius; the ladder exists so a
#: layout that closes a corridor degrades the margin instead of stalling the
#: robot, which is what makes this driver work on a MOVED layout too.
INFLATE_FALLBACK_M = (0.80, 0.65)
#: One control tick per this many basic timesteps (8 ms -> 24 ms, ~42 Hz).
#: Perception and planning run at this rate; the WHEELS do not -- see
#: ``CMD_PERIOD_S``.
CONTROL_EVERY_STEPS = 3
#: ~1 Hz. Replanning five times a second on a map that is still filling in
#: made the pursuit target jump between opposite ends of the arena from one
#: second to the next, and the rover spent a whole 60 s window turning on the
#: spot at (-2, -3) chasing it. A plan is worth having only if it is followed
#: long enough to be tested.
REPLAN_EVERY_TICKS = 40
#: Pure pursuit is unstable when the lookahead is shorter than the vehicle's
#: turning radius, and this rover's is 0.84 m at cruise (0.480 m/s over the
#: 0.57 rad/s the differential cap buys). At 1.0 m it cut corners, overshot,
#: and swung +/- 1.5 rad of heading error the whole way across the arena.
LOOKAHEAD_M = 1.8

# --- ⚠ THE ACTUATION CONTRACT, MEASURED ON THIS ROVER ------------------------
# A wheel command held CONSTANT is tracked exactly. CHANGING one costs a
# transient in which the chassis outruns its own wheels by about 10x and then
# decays back over ~0.5 s -- and the transients ACCUMULATE, so a controller
# that republishes every tick never leaves the transient at all. That is not a
# theory: it is a dither ladder run on this world (base 5.0 rad/s, rolling
# speed 0.400 m/s), and it is the whole reason this driver publishes at 4 Hz
# instead of 42:
#
#     change    every 0.10 s   every 0.25 s   every 0.50 s   every 1.0 s
#     0.5 rad/s   0.733 m/s      0.353          0.350          0.337
#     1.0         1.479          0.520          0.403          0.362
#     2.0         1.693          1.129          0.849          0.567
#
# The first version of this driver recomputed both wheels every 24 ms, which is
# the top-left cell of that table: it spent the run at 2-6 m/s on wheels
# commanded at 0.5 m/s, and its 23.5 m of path over 12 m of route is what that
# looks like from the outside.
#
# The DIFFERENTIAL is exempt in ONE respect only, and assuming otherwise cost
# two runs: a pure spin command (wl = -a, wr = +a) drifts **0.000 m**, because
# the two translational kicks are equal and opposite. Its YAW is kicked exactly
# like the common mode's speed -- measured on the same rig, mid 6.0 with the
# differential dithered by 0.5 every 0.25 s, peak yaw rate **12.6 rad/s**
# against a constant-command peak of 0.82. So BOTH channels are rationed; what
# the cancellation buys is that a turn does not also become a lunge.
#: How often a new wheel target may be published at all.
CMD_PERIOD_S = 0.25
#: Largest change to EITHER channel per publish: 0.5 rad/s at 4 Hz is 2 rad/s^2
#: and it is the fastest setting the table prices at the rolling speed (0.353
#: mean against 0.400, peak 0.84 m/s). 1.0 was tried and measured: it doubles
#: the peak to 1.5 m/s and takes the largest single recorded step from 0.0116 m
#: to 0.0274 m. Reaching cruise takes 3.5 s; that is what this rover costs.
MID_SLEW_RPS = 0.5
DIFF_SLEW_RPS = 0.5
#: 0.57 rad/s of yaw at the measured 0.19 rad/s per rad/s of differential.
DIFF_MAX_RPS = 3.0
#: Wheel-differential per radian of heading error, and per rad/s of measured
#: yaw rate. The measured yaw rate is 0.19 rad/s per rad/s of differential, so
#: the proportional term is a heading gain of ~1.1/s; the rate term is what
#: stops a 4 Hz loop on an 0.9 s plant from ringing.
K_DIFF_PER_RAD = 3.0
#: Rate feedback is OFF. Differencing the IMU yaw over one 24 ms tick is far
#: noisier than the signal it is damping, and at any useful gain it saturated
#: the differential on sensor noise alone. The slew limit above is the damping.
K_DIFF_PER_RATE = 0.0
#: Above this heading error the robot turns in place rather than arcing.
TURN_IN_PLACE_RAD = 0.9
#: Speed is ramped down linearly between these two distances to the nearest
#: return dead ahead, reaching zero at the lower one. A hard threshold made
#: ``mid_want`` flip between 0 and cruise every 0.25 s as the reading crossed
#: it, which is the worst cell of the actuation table above -- the rover braked
#: for its own route, at speed, all the way across the arena.
SLOW_FROM_M = 1.2
EMERGENCY_M = 0.5
#: Deceleration available to the common mode: MID_SLEW / CMD_PERIOD in wheel
#: rad/s^2, times the wheel radius. Used to decide how early to brake for the
#: goal, so the rover does not sail through it and have to come back -- which is
#: how the previous version reached the goal at t = 36 s and finished 0.89 m
#: from it, in the wall corner, upside down.
DECEL_MPS2 = MID_SLEW_RPS / CMD_PERIOD_S * WHEEL_RADIUS_M
#: Recovery. A skid steer that has wedged a corner against a box reads as
#: "commanded and not moving"; backing out and re-planning is what a driver
#: does, and without it the run ends where it jammed.
STUCK_WINDOW_S = 2.5
STUCK_MOVE_M = 0.10
REVERSE_S = 1.6
REVERSE_RPS = -2.5
GIVE_UP_S = 56.0

# --- the occupancy map -------------------------------------------------------
#: A SCORE per cell, not a set. The set version could only ever add: one bad
#: return marked a cell for the rest of the run, and measured on the run that
#: produced this rewrite, **70 % of the 1303 cells the driver believed in were
#: phantom** -- enough that A* found no route at all from t = 7 s and the
#: driver spent the rest of the run following a stale plan into OBSTACLE_1 and
#: OBSTACLE_4. A cell that a later beam passes straight through is now cleared,
#: so the map corrects itself instead of silting up.
OCC_HIT = 2                 # one good return is enough to plan around
OCC_MISS = -1               # ...but it takes two clear pass-throughs to undo
OCC_OCCUPIED = 2            # score at or above this is an obstacle
OCC_CLAMP_HI = 6
OCC_CLAMP_LO = -2
#: Free-space carving is the expensive half, so it runs on every Nth beam.
#: Marking a HIT uses every beam -- reacting late to a new obstacle is the one
#: error this driver cannot afford.
CARVE_BEAM_STRIDE = 2
CARVE_STEP_M = 0.1
CARVE_MAX_M = 4.0

_N = int(round(2 * ARENA_HALF_M / GRID_CELL_M))          # 50 x 50 cells

OUT = os.environ.get(
    "AGENTBENCH_R1_ORACLE_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "r1_oracle_telemetry.json"))
#: Per-tick diagnostic trace, written only when this names a path. It is the
#: instrument that found the two defects this driver used to have (commands
#: over the motor cap, and a map polluted by scans taken while the chassis was
#: pitched), and it is kept so the next one can be found the same way instead
#: of guessed at. Off by default: a graded run should write the telemetry
#: sidecar and nothing else.
TRACE = os.environ.get("AGENTBENCH_R1_ORACLE_TRACE", "")


# --- grid helpers ------------------------------------------------------------

def cell(x, y):
    """World metres -> (ix, iy), clamped to the arena."""
    ix = int((x + ARENA_HALF_M) / GRID_CELL_M)
    iy = int((y + ARENA_HALF_M) / GRID_CELL_M)
    return min(max(ix, 0), _N - 1), min(max(iy, 0), _N - 1)


def centre(ix, iy):
    """(ix, iy) -> the world xy of that cell's centre."""
    return (ix * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0,
            iy * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0)


def _disc_offsets(radius_m):
    """The cell offsets within ``radius_m`` -- the inflation structuring set."""
    r = int(math.ceil(radius_m / GRID_CELL_M))
    return [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if math.hypot(dx, dy) * GRID_CELL_M <= radius_m + 1e-9]


def _wall_margin(radius_m):
    """Cells whose centre is within ``radius_m`` of the arena boundary.

    The arena's EXTENT is given by the task ("10 m x 10 m"); its obstacles are
    not. Blocking the border keeps the planner from routing through a wall it
    has not swept yet -- the walls are also seen and inflated like anything
    else the moment a beam returns off one.
    """
    out = set()
    for ix in range(_N):
        for iy in range(_N):
            cx, cy = centre(ix, iy)
            if (ARENA_HALF_M - abs(cx) < radius_m
                    or ARENA_HALF_M - abs(cy) < radius_m):
                out.add((ix, iy))
    return out


_DISCS = {r: _disc_offsets(r) for r in INFLATE_FALLBACK_M}
_WALLS = {r: _wall_margin(r) for r in INFLATE_FALLBACK_M}


def inflate(occ, radius_m=INFLATE_M):
    """Dilate the occupied SET by ``radius_m`` and add the wall margin.

    Sets rather than an array: a controller runs under whatever interpreter
    the engine spawns and must not need numpy to be navigable.
    """
    out = set(_WALLS[radius_m])
    disc = _DISCS[radius_m]
    for (ix, iy) in occ:
        for dx, dy in disc:
            jx, jy = ix + dx, iy + dy
            if 0 <= jx < _N and 0 <= jy < _N:
                out.add((jx, jy))
    return out


_NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, 1.41421356), (1, -1, 1.41421356),
               (-1, 1, 1.41421356), (-1, -1, 1.41421356)]


def astar(blocked, start, goal, partial=False):
    """8-connected A* over the inflated grid. ``None`` when there is no route.

    Never-seen cells are free: optimism about the unknown is what turns a
    planner into an explorer. A cell that turns out to be occupied is
    discovered by the next scan and the next replan routes around it.

    ``partial`` returns the route to the REACHABLE cell that got closest to the
    goal when the goal itself is unreachable. That is the difference between a
    driver that keeps making progress on a partly-mapped world and one that
    freezes -- or, as the first version of this file did, silently keeps
    following the plan it made before the map closed, which is how it drove
    into the two obstacles it had correctly mapped.
    """
    start, goal = tuple(start), tuple(goal)
    if goal in blocked and not partial:     # refuse to aim into an obstacle
        return None

    def h(c):
        return math.hypot(c[0] - goal[0], c[1] - goal[1])

    def unwind(cur):
        path = [cur]
        while cur in came:
            cur = came[cur]
            path.append(cur)
        return [centre(*c) for c in reversed(path)]

    open_q = [(h(start), 0.0, start)]
    came, best = {}, {start: 0.0}
    closest, closest_h = start, h(start)
    while open_q:
        _f, g, cur = heapq.heappop(open_q)
        if cur == goal:
            return unwind(cur)
        if g > best.get(cur, math.inf) + 1e-9:
            continue
        hc = h(cur)
        if hc < closest_h:
            closest, closest_h = cur, hc
        for dx, dy, stepc in _NEIGHBOURS:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < _N and 0 <= nxt[1] < _N):
                continue
            if nxt in blocked:
                continue
            ng = g + stepc
            if ng < best.get(nxt, math.inf) - 1e-9:
                best[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
    if partial and closest != start:
        return unwind(closest)
    return None


def free_line(blocked, a, b):
    """Is the straight segment a->b clear of blocked cells? (string pulling)"""
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (GRID_CELL_M / 2.0)) + 1
    for i in range(n + 1):
        t = i / float(n)
        if cell(a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t) in blocked:
            return False
    return True


def shorten(blocked, path):
    """Drop waypoints the robot can see past.

    Keeps the 8-connected zig-zag from being driven literally, which is what
    makes the tracking error -- and therefore the clearance the inflation has
    to pay for -- small.
    """
    if not path:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not free_line(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def pursuit_target(path, x, y, lookahead_m=LOOKAHEAD_M):
    """The point ``lookahead_m`` along the path from where the robot is now.

    Projects onto the polyline and walks forward by arc length, rather than
    taking "the next vertex further away than the lookahead". The vertex form
    is what the first version used and it is not stable under replanning: a
    string-pulled path's vertices move when the map changes, so the target
    jumped between opposite ends of the arena and the heading loop chased it
    instead of the route. A point at a fixed arc length moves smoothly when the
    path does.
    """
    if not path:
        return (x, y), 0.0
    if len(path) == 1:
        return path[0], math.hypot(path[0][0] - x, path[0][1] - y)

    best_i, best_t, best_d2 = 0, 0.0, None
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        s = 0.0 if seg2 <= 1e-12 else \
            max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
        px, py = ax + s * dx, ay + s * dy
        d2 = (px - x) ** 2 + (py - y) ** 2
        if best_d2 is None or d2 < best_d2:
            best_i, best_t, best_d2 = i, s, d2

    # walk forward from that projection
    remaining = lookahead_m
    i, s = best_i, best_t
    while i < len(path) - 1:
        ax, ay = path[i]
        bx, by = path[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        left = seg * (1.0 - s)
        if left >= remaining:
            f = s + (remaining / seg if seg > 1e-9 else 0.0)
            return (ax + (bx - ax) * f, ay + (by - ay) * f), remaining
        remaining -= left
        i, s = i + 1, 0.0
    return path[-1], lookahead_m - remaining


def beam_angles(res, fov):
    """Per-beam azimuth in the robot frame, in range-image index order.

    Bin CENTRES over the declared field of view, decreasing with index --
    index 0 is the LEFT end. Measured, not assumed: see the module docstring
    and ``controllers/r1_probe/``.
    """
    d = fov / float(res)
    return [fov / 2.0 - (i + 0.5) * d for i in range(res)]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def trust_range_m(cos_theta, sin_theta, roll, pitch, height_m=LIDAR_HEIGHT_M,
                  vhalf_rad=LIDAR_VFOV_HALF_RAD, max_range_m=SCAN_RANGE_M):
    """How far one beam can be believed, given the chassis attitude.

    A planar scanner 0.24 m off the floor with a 0.05 rad vertical spread sees
    the FLOOR as soon as the chassis tips: the beam is aimed down by the body
    tilt resolved along its own azimuth, and where that ray meets z = 0 it
    returns a range like any other surface. Nothing in the range image says
    which returns those are, so a driver that trusts the whole scan writes a
    ring of obstacles that is not there -- which is exactly what filled 70 % of
    the old map.

    So each beam is trusted only inside ``height / tan(tilt)``. At a level
    chassis that is 9.6 m and the sensor's own 6 m range governs; at 3 deg of
    roll the side beams are trusted to 3.2 m and the rest of their reading is
    treated as *no information*, which is what it is. The tilt is the
    conservative sum of the pitch and roll contributions, so the bound never
    over-trusts the beam it is bounding.

    Takes the beam azimuth as its cosine and sine rather than as an angle:
    both are per-beam constants the caller already has, and this is the
    innermost thing in the control loop (181 beams x ~2000 ticks).
    """
    tilt = (abs(pitch * cos_theta) + abs(roll * sin_theta) + vhalf_rad)
    if tilt >= math.pi / 2.0 - 1e-6:
        return 0.0
    if tilt <= 1e-6:
        return max_range_m
    return min(max_range_m, height_m / math.tan(tilt))


def occupied_set(score):
    """The cells the map currently believes are obstacles."""
    return {c for c, s in score.items() if s >= OCC_OCCUPIED}


def _bump(score, c, delta):
    s = score.get(c, 0) + delta
    score[c] = OCC_CLAMP_HI if s > OCC_CLAMP_HI else (
        OCC_CLAMP_LO if s < OCC_CLAMP_LO else s)


# --- the drive ---------------------------------------------------------------

def drive(robot, out_path=OUT, trace_path=TRACE):
    """Navigate to the goal. Returns (and writes) a dict of what happened."""
    dt = int(robot.getBasicTimeStep())
    lidar = robot.getDevice("lidar")
    lidar.enable(dt)
    gps = robot.getDevice("gps")
    gps.enable(dt)
    imu = robot.getDevice("imu")
    imu.enable(dt)
    left = [robot.getDevice(n) for n in MOTORS_LEFT]
    right = [robot.getDevice(n) for n in MOTORS_RIGHT]
    for m in left + right:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    #: The wheel encoders, read only when a trace is being taken. What they
    #: answer is "is the chassis ROLLING on these wheels or SLIDING under
    #: them?", and no other reading on this robot can tell those apart.
    encoders = []
    if trace_path:
        for n in ("left front sensor", "right front sensor"):
            try:
                s = robot.getDevice(n)
                s.enable(dt)
                encoders.append(s)
            except Exception:                             # noqa: BLE001
                pass

    doc = {"driver": "r1_oracle", "arrived": False,
           "reason": "the run ended before the driver did",
           "reads_obstacle_file": False, "occupancy_grid_started_empty": True,
           "scans": 0, "beams_read": 0, "short_returns": 0,
           "cells_marked_occupied": 0, "narrowest_planning_margin_m": None,
           "lidar": None, "final_xy": None, "distance_to_goal_m": None,
           "sim_time_s": None, "control_ticks": 0, "path_length_m": 0.0}

    def emit():
        try:
            with open(out_path, "w", encoding="utf-8") as fh:
                json.dump(doc, fh, indent=2)
        except OSError:
            pass

    emit()          # exists even if the run is killed mid-way

    if robot.step(dt) == -1:
        return doc

    res = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    max_range = lidar.getMaxRange()
    doc["lidar"] = {"horizontal_resolution": res, "fov_rad": fov,
                    "fov_deg": math.degrees(fov), "max_range_m": max_range,
                    "number_of_layers": lidar.getNumberOfLayers()}
    angles = beam_angles(res, fov)
    beam_cos = [math.cos(a) for a in angles]
    beam_sin = [math.sin(a) for a in angles]
    # the emergency window is +/-10 beams = +/-10 deg. Wider than that and
    # a box the plan deliberately passes 0.7 m to the side of reads as
    # something dead ahead, and the rover brakes for its own route.
    mid_lo, mid_hi = res // 2 - 6, res // 2 + 7

    score = {}                        # EMPTY. Only beams write here.
    occ = set()
    blocked = inflate(occ)
    path = []
    narrowest = INFLATE_M
    ticks = 0
    steps = 0
    last_xy = None
    mid_cmd = diff_cmd = 0.0          # the published wheel target, decomposed
    wl = wr = 0.0
    next_publish = 0.0
    last_yaw = None
    yaw_rate = 0.0
    #: The CLOSEST thing seen dead ahead since the last publish. Taking the
    #: minimum rather than the latest reading is what stops the speed cap from
    #: dithering on a single noisy scan.
    near_min = SCAN_RANGE_M
    tick_s = dt * CONTROL_EVERY_STEPS / 1000.0
    recent = []                    # (t, x, y, yaw), the stuck detector
    reverse_until = -1.0
    doc["reversals"] = 0
    doc["partial_plans"] = 0
    doc["publishes"] = 0
    doc["max_wheel_cmd_rps"] = 0.0
    doc["max_tilt_rad"] = 0.0
    doc["holding_since_s"] = None
    trace = [] if trace_path else None

    while True:
        if robot.step(dt) == -1:
            if not doc["arrived"]:
                doc["reason"] = "the simulation ended"
            break
        steps += 1
        if steps % CONTROL_EVERY_STEPS:
            continue
        ticks += 1
        t = robot.getTime()

        x, y, z = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        if last_xy is not None:
            doc["path_length_m"] += math.hypot(x - last_xy[0], y - last_xy[1])
        last_xy = (x, y)
        doc["max_tilt_rad"] = max(doc["max_tilt_rad"],
                                  abs(wrap(roll)), abs(wrap(pitch)))

        ranges = lidar.getRangeImage() or []
        doc["scans"] += 1
        doc["beams_read"] += len(ranges)

        # 1. THE MAP. A return inside the beam's trusted range is an obstacle
        #    where it landed and free space everywhere it flew through; a
        #    return beyond it is no information at all. Nothing else writes
        #    here -- the grid starts empty and every cell in it came off a
        #    beam.
        nearest_ahead = max_range
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        for i, r in enumerate(ranges):
            if r is None or r != r:
                continue
            bc, bs = beam_cos[i], beam_sin[i]
            trust = trust_range_m(bc, bs, roll, pitch, max_range_m=max_range)
            hit = r < min(trust, max_range - 1e-3)
            # the beam direction in the world frame, from the body-frame
            # azimuth and the measured yaw
            ct, st = cy_ * bc - sy_ * bs, sy_ * bc + cy_ * bs
            if hit:
                doc["short_returns"] += 1
                _bump(score, cell(x + r * ct, y + r * st), OCC_HIT)
                if mid_lo <= i < mid_hi and r < nearest_ahead:
                    nearest_ahead = r
            if i % CARVE_BEAM_STRIDE:
                continue
            # carve the free space this beam flew through, but never past what
            # it can be trusted to have seen
            clear = min(r if hit else trust, trust, CARVE_MAX_M) - CARVE_STEP_M
            d = ROBOT_RADIUS_M
            while d < clear:
                _bump(score, cell(x + d * ct, y + d * st), OCC_MISS)
                d += CARVE_STEP_M

        d_goal = math.hypot(GOAL_XY[0] - x, GOAL_XY[1] - y)
        if d_goal <= GOAL_STOP_M and not doc["arrived"]:
            doc["arrived"] = True
            doc["holding_since_s"] = round(t, 3)
            doc["reason"] = "inside the goal tolerance"
        if t >= GIVE_UP_S and not doc["arrived"]:
            doc["reason"] = "the give-up time passed without reaching the goal"

        # 2. replan on the freshly inflated map, at the widest margin that
        #    admits a route (a layout that closes a corridor should cost
        #    clearance, not motion)
        if (ticks - 1) % REPLAN_EVERY_TICKS == 0 or not path:
            occ = occupied_set(score)
            here = cell(x, y)
            goal_cell = cell(*GOAL_XY)
            found, margin, whole = None, INFLATE_M, False
            for radius in INFLATE_FALLBACK_M:
                blocked = inflate(occ, radius)
                blocked.discard(here)     # standing inside our own margin
                found = astar(blocked, here, goal_cell)
                if found:
                    margin, whole = radius, True
                    break
            if found is None:
                # No route to the goal on the map as it stands. Head for the
                # reachable cell that gets CLOSEST to it and re-ask next tick,
                # because the map is partial and the next scan may open the
                # way. The old fallback -- keep following the plan made before
                # the map closed -- is what drove this rover into OBSTACLE_1.
                blocked = inflate(occ, INFLATE_FALLBACK_M[-1])
                blocked.discard(here)
                found = astar(blocked, here, goal_cell, partial=True)
                margin = INFLATE_FALLBACK_M[-1]
                if found:
                    doc["partial_plans"] += 1
            if found:
                path = shorten(blocked, found)
                if whole:
                    # A* lands on the goal CELL; the goal is a POINT. Without
                    # this the robot parks on a cell centre and stops half a
                    # cell short.
                    path[-1] = GOAL_XY
                narrowest = min(narrowest, margin)
            elif not path:
                path = [(x, y), GOAL_XY]
                narrowest = 0.0

        # 3. pure pursuit, at a fixed arc length along the plan
        near_min = min(near_min, nearest_ahead)
        (tx, ty), _ahead = pursuit_target(path, x, y)
        if doc["arrived"]:
            tx, ty = GOAL_XY
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        yaw_rate = 0.0 if last_yaw is None else wrap(yaw - last_yaw) / tick_s
        last_yaw = yaw

        # 4. stuck? A commanded rover that is not moving has wedged something.
        recent.append((t, x, y, yaw))
        while recent and t - recent[0][0] > STUCK_WINDOW_S:
            recent.pop(0)

        # 5. PUBLISH -- at CMD_PERIOD_S, never faster. Everything above runs
        #    every tick; the wheels hear from us four times a second, because
        #    that is what the actuation contract at the top of this file costs.
        if t < next_publish:
            if trace is not None:
                enc = [round(s.getValue(), 4) for s in encoders]
                trace.append([round(t, 3), round(x, 3), round(y, 3),
                              round(z, 3), round(roll, 3), round(pitch, 3),
                              round(yaw, 3), round(tx, 2), round(ty, 2),
                              round(err, 3), round(mid_cmd, 3),
                              round(diff_cmd, 3), round(wl, 2), round(wr, 2),
                              round(nearest_ahead, 2), len(occ)] + enc)
            continue
        next_publish = t + CMD_PERIOD_S
        doc["publishes"] += 1
        near_seen, near_min = near_min, max_range

        if doc["arrived"]:
            # PARKED, and it never navigates again. R1.4 reads the LAST sample
            # of a 60 s window, and the two runs before this one both REACHED
            # the goal and then lost it: one coasted 0.57 m past, and one --
            # arriving at t = 33.9 s -- drifted out of the tolerance, was
            # declared stuck by the recovery logic for standing still, reversed
            # seven times and finished against the north wall 1.72 m away. So
            # once the goal has been reached the only behaviours left are HOLD
            # and CREEP BACK: no planner, no recovery, no reverse.
            if d_goal <= GOAL_HOLD_M:
                mid_want, diff_want = 0.0, 0.0
            else:
                diff_want = max(-DIFF_MAX_RPS,
                                min(DIFF_MAX_RPS, K_DIFF_PER_RAD * err))
                mid_want = (0.0 if abs(err) > TURN_IN_PLACE_RAD
                            else CREEP_RPS)
        elif t < reverse_until:
            mid_want, diff_want = REVERSE_RPS, 0.0
        else:
            # Not moving AND not turning, while the goal is still out there.
            # The previous form also required a non-zero SPEED command, so a
            # rover the emergency ramp had pinned in front of a box could never
            # trigger it -- measured: 10 s parked at (-2.9, 1.5) with mid_cmd
            # 0, spinning on the spot, until the window ran out.
            wedged = (len(recent) > 4
                      and t - recent[0][0] >= STUCK_WINDOW_S * 0.9
                      and math.hypot(x - recent[0][1],
                                     y - recent[0][2]) < STUCK_MOVE_M
                      and abs(wrap(yaw - recent[0][3])) < 0.35)
            if wedged:
                reverse_until = t + REVERSE_S
                doc["reversals"] += 1
                recent = []
                mid_want, diff_want = REVERSE_RPS, 0.0
            else:
                raw = K_DIFF_PER_RAD * err - K_DIFF_PER_RATE * yaw_rate
                diff_want = max(-DIFF_MAX_RPS, min(DIFF_MAX_RPS, raw))
                if abs(err) > TURN_IN_PLACE_RAD:
                    mid_want = 0.0           # spin on the spot: no drift
                else:
                    mid_want = CRUISE_RPS * math.cos(err)
                # brake for the goal early enough to STOP in it: the speed
                # that can still be shed over the distance remaining, with
                # margin, expressed in wheel rad/s
                stop_in = max(0.0, d_goal - GOAL_STOP_M * 0.5)
                mid_want = min(mid_want,
                               math.sqrt(2.0 * DECEL_MPS2 * stop_in) * 0.7
                               / WHEEL_RADIUS_M)
                # ...and for whatever is dead ahead, on a ramp rather than a
                # cliff
                if near_seen < SLOW_FROM_M:
                    f = ((near_seen - EMERGENCY_M)
                         / (SLOW_FROM_M - EMERGENCY_M))
                    mid_want = min(mid_want, CRUISE_RPS * max(0.0, f))

        # 6. slew BOTH channels, and never hand either wheel a target the
        #    motor runs away on. Yaw is paid first: a rover that turns slower
        #    than it wanted still gets there, one that drives its planned arc
        #    at the wrong heading does not.
        mid_cmd += max(-MID_SLEW_RPS, min(MID_SLEW_RPS, mid_want - mid_cmd))
        diff_cmd += max(-DIFF_SLEW_RPS,
                        min(DIFF_SLEW_RPS, diff_want - diff_cmd))
        diff = max(-WHEEL_CMD_MAX_RPS, min(WHEEL_CMD_MAX_RPS, diff_cmd))
        room = WHEEL_CMD_MAX_RPS - abs(diff)
        mid = max(-room, min(room, mid_cmd))
        wl, wr = mid - diff, mid + diff
        doc["max_wheel_cmd_rps"] = max(doc["max_wheel_cmd_rps"],
                                       abs(wl), abs(wr))
        for m in left:
            m.setVelocity(wl)
        for m in right:
            m.setVelocity(wr)

        if trace is not None:
            enc = [round(s.getValue(), 4) for s in encoders]
            trace.append([round(t, 3), round(x, 3), round(y, 3), round(z, 3),
                          round(roll, 3), round(pitch, 3), round(yaw, 3),
                          round(tx, 2), round(ty, 2), round(err, 3),
                          round(mid_cmd, 3), round(diff_cmd, 3), round(wl, 2),
                          round(wr, 2), round(nearest_ahead, 2), len(occ)]
                         + enc)

    # brake, and hold station for whatever is left of the recording window so
    # the FINAL sample -- which is what R1.4 measures -- is where it stopped.
    for m in left + right:
        m.setVelocity(0.0)
    x, y, _z = gps.getValues()
    occ = occupied_set(score)
    doc.update({"control_ticks": ticks, "final_xy": [x, y],
                "sim_time_s": robot.getTime(),
                "distance_to_goal_m": math.hypot(GOAL_XY[0] - x,
                                                 GOAL_XY[1] - y),
                "cells_marked_occupied": len(occ),
                "cells_ever_touched": len(score),
                "narrowest_planning_margin_m": narrowest})
    emit()
    if trace is not None:
        try:
            with open(trace_path, "w", encoding="utf-8") as fh:
                json.dump({"columns": ["t", "x", "y", "z", "roll", "pitch",
                                       "yaw", "tx", "ty", "err", "mid",
                                       "diff", "wl", "wr", "near", "n_occ"]
                                      + ["enc%d" % i
                                         for i in range(len(encoders))],
                           "rows": trace,
                           "occupied_cells": sorted(occ)}, fh)
        except OSError:
            pass
    while robot.step(dt) != -1:
        pass
    return doc


def main():
    from omnisim import Robot
    drive(Robot())


if __name__ == "__main__":
    main()
