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

"""R1 ORACLE driver -- the known-good solution, on the MuJoCo arm.

Paired with ``r1_oracle.xml``. It exists for one purpose: SPEC 7.1's gate says
an oracle that performs the known-good solution must PASS every task it is run
on, and until that has been shown for a given arm, that arm's pass rate is
unfalsifiable -- exactly the defect C2 shipped with, where a task nobody had
proven could FAIL passed 5/5 unfixed for a whole campaign.

**How it senses, stated exactly.** ``mujoco.mj_ray`` -- MuJoCo's own
ray/geom intersection, the same primitive its ``<rangefinder>`` sensor is
built on. Every control tick the driver casts a planar fan of
:data:`SCAN_BEAMS` = 181 beams spanning :data:`SCAN_FOV_DEG` = 180 deg,
centred on the robot's heading, out to :data:`SCAN_RANGE_M` = 6.0 m, from the
``lidar`` site on the rover's mast -- clearing the task's stated LiDAR minima
(>= 180 deg, >= 180 samples, >= 5 m) with the beam count, not with an
assertion about them. ``flg_static=1`` so the static obstacle and wall geoms
are visible to it; ``bodyexclude`` is the rover's own body, and the mast
height puts a horizontal beam above the wheels and the chassis so a self-hit
is geometrically impossible rather than filtered away.

The MJCF alternative -- 181 ``<rangefinder>`` sensors on 181 replicated sites,
so the LiDAR lives in the model -- was considered and not used: it costs 181
ray casts on EVERY one of the 30,000 integration steps rather than on the 20
control ticks a second that consume them, and the point of this file is that
the *controller* perceives.

**What it knows and what it does not.** It is given what the task gives every
agent: the arena is 10 x 10 m, the start is (-4, -4) and the goal is (4, 4).
It is NOT given the obstacles. ``benchmark_assets/obstacles.json`` is never
read here, no obstacle pose appears anywhere in this file, and the occupancy
grid the planner runs on starts **empty** -- every occupied cell in it was put
there by a beam that came back short. That is the difference between an oracle
and a proof of nothing: the straight line from start to goal is blocked by
three of the five obstacles, so a driver that did not perceive could not
produce this run, and one that had memorised the published layout would be
caught the moment the layout moved. It does not need to be caught -- run this
same driver against a perturbed arena and it still arrives
(``test_r1_discriminates_mujoco.py``).

**The method** is the textbook one for "navigate a partially known space":

  1. cast the fan, and mark the cell each short return landed in as occupied
     in a 0.2 m grid;
  2. inflate the occupied set by the robot's radius plus a margin
     (:data:`INFLATE_M`) so the planner can treat the robot as a point;
  3. A* to the goal over the inflated grid, treating never-seen cells as free
     -- optimism about the unknown is what makes it explore rather than
     freeze -- and replan at :data:`REPLAN_HZ`, so a newly seen obstacle
     changes the plan instead of being driven into;
  4. follow the plan with pure pursuit, and stop inside the goal tolerance.

Nothing here writes a pose, and every metre of the path is integrated by the
wheels: R1.6's teleport bound (<= 0.25 m between samples) is satisfied by a
factor of ~100, which is what a physical drive looks like.

Runs standalone for development -- ``python r1_oracle.py r1_oracle.xml`` --
and is otherwise executed by the arm's recorder, which owns termination.
"""

from __future__ import annotations

import heapq
import math
import sys

import mujoco
import numpy as np

# --- what the task hands every agent ----------------------------------------
ARENA_HALF_M = 5.0
START_XY = (-4.0, -4.0)
GOAL_XY = (4.0, 4.0)
GOAL_STOP_M = 0.10          # well inside the task's 0.30 m tolerance

# --- the LiDAR (the task's minima are 180 deg / 180 samples / 5 m) ----------
SCAN_FOV_DEG = 180.0
SCAN_BEAMS = 181
SCAN_RANGE_M = 6.0

# --- the robot, as built in r1_oracle.xml -----------------------------------
WHEEL_RADIUS_M = 0.13
WHEEL_BASE_M = 0.38
#: Circumscribed radius of the chassis + wheels: the chassis corner is at
#: hypot(0.25, 0.16) = 0.297 m and the outer wheel edge at hypot(0.23, 0.23)
#: = 0.264 m, so 0.30 m bounds the robot.
ROBOT_RADIUS_M = 0.30

# --- planner ----------------------------------------------------------------
GRID_CELL_M = 0.2
#: Robot radius, plus the grid's own quantisation error (a return is snapped
#: to a 0.2 m cell whose centre can sit 0.14 m INSIDE the surface that
#: produced it), plus margin for pure-pursuit tracking error. Measured at
#: 0.5: the run cleared OBSTACLE_5's corner by 0.264 m centre-to-surface,
#: i.e. INSIDE the 0.30 m circumscribed radius -- it did not collide, but
#: only because the corner happened to fall where the footprint is narrow.
#: An oracle that passes on that margin has not shown a task is passable.
INFLATE_M = 0.7
#: Tried in order when the planner cannot find a route at INFLATE_M. Every
#: fallback still exceeds the 0.30 m robot radius; the ladder exists so a
#: layout that closes a corridor degrades the margin instead of stalling the
#: robot, which is what makes this driver work on a MOVED layout too.
INFLATE_FALLBACK_M = (0.7, 0.5, 0.4)
CONTROL_HZ = 20.0
REPLAN_HZ = 4.0
LOOKAHEAD_M = 0.7
V_MAX_MPS = 0.9
W_MAX_RPS = 2.2
K_HEADING = 2.6
#: Above this heading error the robot turns in place rather than arcing.
TURN_IN_PLACE_RAD = 0.8
#: A return closer than this dead ahead stops forward motion for a tick.
EMERGENCY_M = 0.45
GIVE_UP_S = 55.0

_N = int(round(2 * ARENA_HALF_M / GRID_CELL_M))          # 50 x 50 cells


# --- grid helpers ------------------------------------------------------------

def _cell(x, y):
    """World metres -> (ix, iy), clamped to the arena."""
    ix = int((x + ARENA_HALF_M) / GRID_CELL_M)
    iy = int((y + ARENA_HALF_M) / GRID_CELL_M)
    return min(max(ix, 0), _N - 1), min(max(iy, 0), _N - 1)


def _centre(ix, iy):
    """(ix, iy) -> the world xy of that cell's centre."""
    return (ix * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0,
            iy * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0)


def _disc_offsets(radius_m):
    """The cell offsets within ``radius_m`` -- the inflation structuring set."""
    r = int(math.ceil(radius_m / GRID_CELL_M))
    return [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if math.hypot(dx, dy) * GRID_CELL_M <= radius_m + 1e-9]


def _wall_margin_mask(radius_m):
    """Cells whose centre is within ``radius_m`` of the arena boundary.

    The arena's EXTENT is given by the task ("10 m x 10 m"); its obstacles are
    not. Blocking the border keeps the planner from routing through a wall it
    has not swept yet -- the walls are also seen and inflated like anything
    else the moment a beam returns off one.
    """
    mask = np.zeros((_N, _N), dtype=bool)
    for ix in range(_N):
        for iy in range(_N):
            cx, cy = _centre(ix, iy)
            if (ARENA_HALF_M - abs(cx) < radius_m
                    or ARENA_HALF_M - abs(cy) < radius_m):
                mask[ix, iy] = True
    return mask


#: Structuring sets and wall margins, built once per inflation radius.
_DISCS = {r: _disc_offsets(r) for r in INFLATE_FALLBACK_M}
_WALLS = {r: _wall_margin_mask(r) for r in INFLATE_FALLBACK_M}


def inflate(occ, radius_m=INFLATE_M):
    """Dilate the occupied set by ``radius_m`` and add the wall margin."""
    out = _WALLS[radius_m].copy()
    ixs, iys = np.nonzero(occ)
    for dx, dy in _DISCS[radius_m]:
        jx, jy = ixs + dx, iys + dy
        keep = (jx >= 0) & (jx < _N) & (jy >= 0) & (jy < _N)
        out[jx[keep], jy[keep]] = True
    return out

_NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, 1.41421356), (1, -1, 1.41421356),
               (-1, 1, 1.41421356), (-1, -1, 1.41421356)]


def astar(blocked, start, goal):
    """8-connected A* over the inflated grid. ``None`` when there is no route.

    Never-seen cells are free: optimism about the unknown is what turns a
    planner into an explorer. A cell that turns out to be occupied is
    discovered by the next scan and the next replan routes around it.
    """
    if blocked[goal]:                       # refuse to aim into an obstacle
        return None
    start = tuple(start)
    goal = tuple(goal)
    h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])   # noqa: E731
    open_q = [(h(start), 0.0, start)]
    came, best = {}, {start: 0.0}
    while open_q:
        _f, g, cur = heapq.heappop(open_q)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return [_centre(*c) for c in reversed(path)]
        if g > best.get(cur, math.inf) + 1e-9:
            continue
        for dx, dy, step in _NEIGHBOURS:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < _N and 0 <= nxt[1] < _N):
                continue
            if blocked[nxt]:
                continue
            ng = g + step
            if ng < best.get(nxt, math.inf) - 1e-9:
                best[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
    return None


def free_line(blocked, a, b):
    """Is the straight segment a->b clear of blocked cells? (string pulling)"""
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (GRID_CELL_M / 2.0)) + 1
    for i in range(n + 1):
        t = i / float(n)
        if blocked[_cell(a[0] + (b[0] - a[0]) * t,
                         a[1] + (b[1] - a[1]) * t)]:
            return False
    return True


def shorten(blocked, path):
    """Drop waypoints the robot can see past. Keeps the 8-connected zig-zag
    from being driven literally, which is what makes the tracking error --
    and therefore the clearance the inflation has to pay for -- small."""
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


# --- sensing -----------------------------------------------------------------

class Lidar:
    """A planar LiDAR built on ``mujoco.mj_ray``.

    One instance owns its beam table and its scratch buffers so a scan costs
    no allocation. ``scan()`` returns the range per beam in metres, with
    ``SCAN_RANGE_M`` for a beam that hit nothing within range -- the same
    convention a real driver reads off a max-range return.
    """

    def __init__(self, model, site_name, body_name):
        self.model = model
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                      site_name)
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                      body_name)
        if self.site < 0 or self.body < 0:
            raise RuntimeError("r1_oracle.xml must declare the %r site on the "
                               "%r body" % (site_name, body_name))
        half = math.radians(SCAN_FOV_DEG) / 2.0
        self.angles = [-half + i * (2 * half / (SCAN_BEAMS - 1))
                       for i in range(SCAN_BEAMS)]
        self._pnt = np.zeros(3)
        self._vec = np.zeros(3)
        self._gid = np.zeros(1, dtype=np.int32)
        self.n_scans = 0
        self.n_beams_cast = 0

    def scan(self, data, yaw):
        """Ranges for every beam, at the site's current pose."""
        self._pnt[:] = data.site_xpos[self.site]
        out = []
        for a in self.angles:
            th = yaw + a
            self._vec[0] = math.cos(th)
            self._vec[1] = math.sin(th)
            self._vec[2] = 0.0
            d = mujoco.mj_ray(self.model, data, self._pnt, self._vec, None,
                              1, self.body, self._gid)
            out.append(SCAN_RANGE_M if (d < 0 or d > SCAN_RANGE_M) else d)
        self.n_scans += 1
        self.n_beams_cast += SCAN_BEAMS
        return out


# --- the drive ---------------------------------------------------------------

def _yaw(data, bid):
    m = data.xmat[bid]
    return math.atan2(m[3], m[0])           # body +x axis, in world


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def drive(model, data, *, verbose=True):
    """Navigate to the goal. Returns a dict of what actually happened."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rover")
    lidar = Lidar(model, "lidar", "rover")
    occ = np.zeros((_N, _N), dtype=bool)
    blocked = inflate(occ)
    path, plan_i = [], 0

    dt = model.opt.timestep
    every = max(1, int(round(1.0 / (CONTROL_HZ * dt))))
    replan_every = max(1, int(round(CONTROL_HZ / REPLAN_HZ)))
    ticks = 0
    step = 0
    arrived = False
    narrowest = INFLATE_M               # the tightest margin any plan used
    reason = "give-up time reached"

    while data.time < GIVE_UP_S:
        mujoco.mj_step(model, data)
        step += 1
        if data.time < 0.05:                # let it settle onto its wheels
            continue
        if step % every:                    # control runs at CONTROL_HZ
            continue
        ticks += 1

        x, y = float(data.xpos[bid][0]), float(data.xpos[bid][1])
        yaw = _yaw(data, bid)
        ranges = lidar.scan(data, yaw)

        # 1. every short return is an obstacle cell. Nothing else writes here.
        for a, r in zip(lidar.angles, ranges):
            if r >= SCAN_RANGE_M - 1e-6:
                continue
            th = yaw + a
            occ[_cell(x + r * math.cos(th), y + r * math.sin(th))] = True

        d_goal = math.hypot(GOAL_XY[0] - x, GOAL_XY[1] - y)
        if d_goal <= GOAL_STOP_M:
            arrived, reason = True, "inside the goal tolerance"
            break

        # 2. replan on the freshly inflated map, widest margin that admits a
        #    route (a layout that closes a corridor should cost clearance,
        #    not motion)
        if (ticks - 1) % replan_every == 0 or plan_i >= len(path):
            here = _cell(x, y)
            goal_cell = _cell(*GOAL_XY)
            found = None
            for radius in INFLATE_FALLBACK_M:
                blocked = inflate(occ, radius)
                if blocked[here]:           # standing inside our own margin
                    blocked[here] = False
                found = astar(blocked, here, goal_cell)
                if found:
                    margin = radius
                    break
            if found:
                path = shorten(blocked, found)
                # A* lands on the goal CELL; the goal is a POINT. Without this
                # the robot parks on the cell centre and stops half a cell
                # short -- measured 4.10, 4.10 against a goal of 4.0, 4.0.
                path[-1] = GOAL_XY
                plan_i = 1
                if margin != INFLATE_M:
                    narrowest = min(narrowest, margin)
            elif not path:
                path, plan_i = [(x, y), GOAL_XY], 1
                narrowest = 0.0

        # 3. pure pursuit: the furthest waypoint within the lookahead
        while (plan_i < len(path) - 1
               and math.hypot(path[plan_i][0] - x,
                              path[plan_i][1] - y) < LOOKAHEAD_M):
            plan_i += 1
        tx, ty = path[min(plan_i, len(path) - 1)]
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)

        w = max(-W_MAX_RPS, min(W_MAX_RPS, K_HEADING * err))
        if abs(err) > TURN_IN_PLACE_RAD:
            v = 0.0
        else:
            v = V_MAX_MPS * math.cos(err)
            v = min(v, 0.9 * d_goal + 0.05)     # ease into the goal
        mid = SCAN_BEAMS // 2
        if min(ranges[mid - 20:mid + 21]) < EMERGENCY_M:
            v = 0.0                              # something is right there

        data.ctrl[0] = (v - w * WHEEL_BASE_M / 2.0) / WHEEL_RADIUS_M
        data.ctrl[1] = (v + w * WHEEL_BASE_M / 2.0) / WHEEL_RADIUS_M

        if verbose and ticks % 40 == 0:
            print("t=%5.1f s  xy=(%6.2f,%6.2f)  goal %5.2f m  "
                  "nearest beam %4.2f m  waypoints %d"
                  % (data.time, x, y, d_goal, min(ranges), len(path)))

    # brake, and let the wheels actually stop before the run ends
    data.ctrl[:] = 0.0
    t_stop = data.time
    while data.time - t_stop < 0.6:
        mujoco.mj_step(model, data)

    x, y = float(data.xpos[bid][0]), float(data.xpos[bid][1])
    out = {"arrived": arrived, "reason": reason, "final_xy": (x, y),
           "distance_to_goal_m": math.hypot(GOAL_XY[0] - x, GOAL_XY[1] - y),
           "sim_time_s": float(data.time), "control_ticks": ticks,
           "scans": lidar.n_scans, "beams_cast": lidar.n_beams_cast,
           "cells_marked_occupied": int(occ.sum()),
           "narrowest_planning_margin_m": narrowest}
    if verbose:
        print("[oracle] %s: final xy=(%.3f, %.3f), %.3f m from the goal after "
              "%.1f s; %d scans / %d beams / %d occupied cells learned"
              % (reason, x, y, out["distance_to_goal_m"], data.time,
                 lidar.n_scans, lidar.n_beams_cast, out["cells_marked_occupied"]))
    return out


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "r1_oracle.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    return drive(model, data)


if __name__ == "__main__":
    main()
