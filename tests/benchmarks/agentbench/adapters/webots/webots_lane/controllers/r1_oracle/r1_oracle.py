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

"""R1 ORACLE driver -- the known-good solution, on the upstream-Webots arm.

Paired with ``webots_lane/worlds/r1_lidar_nav.wbt``. It exists for one purpose:
SPEC 7.1's gate says an oracle performing the known-good solution must PASS
every task it is run on, and until that has been shown for a given (task, arm)
pair, that arm's pass rate on that task is unfalsifiable -- exactly the defect
C2 shipped with, where a world nobody had proven could FAIL passed 5/5 unfixed
for a whole campaign. This arm matters more than most: upstream Webots is the
CONTROL, so a weak control manufactures a win for OmniSim.

**How it senses, stated exactly.** ``wb_lidar_get_range_image`` on a
``SickLms291`` -- upstream's own Sick LMS 291 model, mounted where upstream's
own lidar-navigation sample mounts it. The device reports 181 samples across
180 deg with an 80 m maximum range, so the task's stated LiDAR minima (>= 180
deg, >= 180 samples, >= 5 m) are cleared by the DEVICE, not by an assertion
about it, and the numbers are read back off the device at startup and printed.

Two conventions here were measured by ``controllers/r1_probe_lidar`` against
R2025a rather than remembered, because getting either backwards mirrors the
map, which does not fail loudly -- it drives the robot into the thing it
thinks it avoided:

* **index 0 is the LEFT of the fan**, index ``n-1`` the right, and each bin
  sits at its CENTRE: ``angle_i = fov/2 - (i + 0.5) * fov / n``, positive to
  the left. Measured: a box at world (2.0, 1.5) seen from (0, 0) with yaw 0
  landed at index 51 of 181, i.e. 38.78 deg left, and the device's own point
  cloud put it at (1.6140, 1.2970) in the sensor frame -- atan2 of which is
  38.78 deg. The two agree to the printed digits.
* **the sensor frame is x-forward, y-left**, so the fan is planar in the
  robot's xy and needs no frame conversion beyond the mount offset.

**What it knows and what it does not.** It is given what the task gives every
agent: the arena is 10 x 10 m, the start is (-4, -4) and the goal is (4, 4). It
is NOT given the obstacles. ``benchmark_assets/obstacles.json`` is never read
here, no obstacle pose appears anywhere in this file, and the occupancy grid
the planner runs on starts **empty** -- every occupied cell in it was put there
by a beam that came back short. The straight line from start to goal is blocked
by OBSTACLE_1, so a driver that did not perceive could not produce this run;
one that had memorised the published layout would be caught the moment the
layout moved, and this one is not, which
``test_r1_discriminates_webots.py::test_the_oracle_still_arrives_when_the_obstacles_move``
measures rather than claims.

**Where its own pose comes from.** A ``GPS`` and an ``InertialUnit``, both
upstream device nodes, both declared in the world. That is localisation from
sensors -- the ordinary Webots way, and what every upstream navigation sample
does -- not the Supervisor pose reads the task forbids: this controller is a
plain ``Robot``, has no Supervisor rights, and could not read a scene pose if
it wanted to.

**The method** is the textbook one for "navigate a partially known space":

  1. cast the fan, and mark the cell each short return landed in as occupied in
     a 0.2 m grid;
  2. inflate the occupied set by the robot's radius plus a margin
     (:data:`INFLATE_M`) so the planner can treat the robot as a point;
  3. A* to the goal over the inflated grid, treating never-seen cells as free
     -- optimism about the unknown is what makes it explore rather than freeze
     -- and replan at :data:`REPLAN_HZ`, so a newly seen obstacle changes the
     plan instead of being driven into;
  4. follow the plan with pure pursuit, and stop inside the goal tolerance.

Nothing here writes a pose, and every metre of the path is integrated by the
wheels: R1.6's teleport bound (<= 0.25 m between samples) is met by two orders
of magnitude, which is what a physical drive looks like.
"""

import heapq
import math

from controller import Robot

# --- what the task hands every agent ----------------------------------------
ARENA_HALF_M = 5.0
START_XY = (-4.0, -4.0)
GOAL_XY = (4.0, 4.0)
GOAL_STOP_M = 0.12          # well inside the task's 0.30 m tolerance

# --- the robot, as built in r1_lidar_nav.wbt ---------------------------------
LIDAR_NAME = "Sick LMS 291"
#: The Sick's mount offset along the robot's +x, from r1_lidar_nav.wbt (and
#: from upstream's own pioneer3at.wbt, which chose the same 0.136). The fan's
#: origin is here, not at the robot origin, and a 0.136 m error in the ray
#: origin is 0.7 of a grid cell.
LIDAR_OFFSET_X_M = 0.136
WHEEL_RADIUS_M = 0.11       # Pioneer3at.proto
HALF_TRACK_M = 0.197        # Pioneer3at.proto wheel anchors, |y|
MAX_WHEEL_RAD_S = 6.4       # Pioneer3at.proto RotationalMotor maxVelocity
WHEELS = ("front left wheel", "front right wheel",
          "back left wheel", "back right wheel")
#: Circumscribed radius of the Pioneer 3-AT: the chassis corner is at
#: hypot(0.254, 0.1965) = 0.321 m and the outer wheel corner at
#: hypot(0.1331 + 0.11, 0.197 + 0.0375) = 0.338 m, so 0.34 m bounds it.
ROBOT_RADIUS_M = 0.34

# --- how the fan is consumed -------------------------------------------------
#: Returns beyond this are not marked. The device reaches 80 m and the arena is
#: 10 m across, so this is not a range limit -- it is a QUALITY limit: a bin is
#: ~1 deg wide, so at 6 m one bin spans 0.1 m, about half a grid cell, and
#: marking a 9 m return would smear a wall across cells it does not occupy.
MARK_MAX_M = 6.0
#: A return closer than this dead ahead stops forward motion for a tick.
EMERGENCY_M = 0.55
#: Half-width, in bins, of the "dead ahead" window the emergency stop watches.
EMERGENCY_HALF_BINS = 20

# --- planner ----------------------------------------------------------------
GRID_CELL_M = 0.2
#: Robot radius (0.34), plus the grid's own quantisation error (a return is
#: snapped to a 0.2 m cell whose centre can sit 0.14 m INSIDE the surface that
#: produced it), plus margin for pure-pursuit tracking error.
INFLATE_M = 0.75
#: Tried in order when the planner cannot find a route at INFLATE_M. Every
#: fallback still exceeds the 0.34 m robot radius plus the 0.14 m
#: quantisation; the ladder exists so a layout that closes a corridor degrades
#: the margin instead of stalling the robot, which is what makes this driver
#: work on a MOVED layout too.
INFLATE_FALLBACK_M = (0.75, 0.6, 0.5)
CONTROL_PERIOD_MS = 64      # 4 basic steps at basicTimeStep 16
REPLAN_EVERY_TICKS = 4      # ~3.9 Hz
LOOKAHEAD_M = 0.8
V_MAX_MPS = 0.62            # the wheels cap at 6.4 rad/s * 0.11 m = 0.704 m/s
W_MAX_RPS = 1.4
K_HEADING = 2.2
#: Above this heading error the robot turns in place rather than arcing.
TURN_IN_PLACE_RAD = 0.7
#: Simulated seconds after which the driver stops trying and reports.
GIVE_UP_S = 200.0

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
            cx, cy = _centre(ix, iy)
            if (ARENA_HALF_M - abs(cx) < radius_m
                    or ARENA_HALF_M - abs(cy) < radius_m):
                out.add((ix, iy))
    return out


#: Structuring sets and wall margins, built once per inflation radius. Pure
#: stdlib: upstream's own python3 is what runs a controller, and this file must
#: not need numpy to be installed in it.
_DISCS = {r: _disc_offsets(r) for r in INFLATE_FALLBACK_M}
_WALLS = {r: _wall_margin(r) for r in INFLATE_FALLBACK_M}


def inflate(occ, radius_m=INFLATE_M):
    """The blocked set: ``occ`` dilated by ``radius_m``, plus the wall margin."""
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


def astar(blocked, start, goal):
    """8-connected A* over the inflated grid. ``None`` when there is no route.

    Never-seen cells are free: optimism about the unknown is what turns a
    planner into an explorer. A cell that turns out to be occupied is
    discovered by the next scan and the next replan routes around it.
    """
    if goal in blocked:                     # refuse to aim into an obstacle
        return None
    start, goal = tuple(start), tuple(goal)

    def h(c):
        return math.hypot(c[0] - goal[0], c[1] - goal[1])

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
        if g > best.get(cur, float("inf")) + 1e-9:
            continue
        for dx, dy, step in _NEIGHBOURS:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < _N and 0 <= nxt[1] < _N):
                continue
            if nxt in blocked:
                continue
            ng = g + step
            if ng < best.get(nxt, float("inf")) - 1e-9:
                best[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_q, (ng + h(nxt), ng, nxt))
    return None


def free_line(blocked, a, b):
    """Is the straight segment a->b clear of blocked cells? (string pulling)"""
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (GRID_CELL_M / 2.0)) + 1
    for i in range(n + 1):
        t = i / float(n)
        if _cell(a[0] + (b[0] - a[0]) * t,
                 a[1] + (b[1] - a[1]) * t) in blocked:
            return False
    return True


def shorten(blocked, path):
    """Drop waypoints the robot can see past. Keeps the 8-connected zig-zag
    from being driven literally, which is what makes the tracking error -- and
    therefore the clearance the inflation has to pay for -- small."""
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


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --- the drive ---------------------------------------------------------------

def main():
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    control_every = max(1, int(round(CONTROL_PERIOD_MS / float(dt_ms))))

    lidar = robot.getDevice(LIDAR_NAME)
    lidar.enable(dt_ms * control_every)
    n_beams = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    # Bin CENTRES, left-positive -- measured, see the module docstring.
    angles = [fov / 2.0 - (i + 0.5) * fov / n_beams for i in range(n_beams)]
    print("[oracle] lidar '%s': %d samples over %.2f deg, max range %.1f m"
          % (LIDAR_NAME, n_beams, math.degrees(fov), lidar.getMaxRange()),
          flush=True)

    gps = robot.getDevice("gps")
    gps.enable(dt_ms)
    imu = robot.getDevice("inertial unit")
    imu.enable(dt_ms)

    wheels = [robot.getDevice(w) for w in WHEELS]
    for m in wheels:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    occ = set()
    blocked = inflate(occ)
    path, plan_i = [], 0
    ticks, step = 0, 0
    scans, beams_cast = 0, 0
    arrived = False
    narrowest = INFLATE_M               # the tightest margin any plan used
    reason = "give-up time reached"
    mid = n_beams // 2
    lo_e = max(0, mid - EMERGENCY_HALF_BINS)
    hi_e = min(n_beams, mid + EMERGENCY_HALF_BINS + 1)

    while robot.step(dt_ms) != -1 and robot.getTime() < GIVE_UP_S:
        step += 1
        if step % control_every:        # control runs at CONTROL_PERIOD_MS
            continue
        ticks += 1

        pos = gps.getValues()
        x, y = float(pos[0]), float(pos[1])
        yaw = float(imu.getRollPitchYaw()[2])
        ranges = lidar.getRangeImage()
        if not ranges:
            continue
        scans += 1
        beams_cast += n_beams

        # 1. every short return is an obstacle cell. Nothing else writes here.
        lx = x + LIDAR_OFFSET_X_M * math.cos(yaw)
        ly = y + LIDAR_OFFSET_X_M * math.sin(yaw)
        for a, r in zip(angles, ranges):
            if not (r == r) or r > MARK_MAX_M:      # NaN / inf / too far
                continue
            th = yaw + a
            occ.add(_cell(lx + r * math.cos(th), ly + r * math.sin(th)))

        d_goal = math.hypot(GOAL_XY[0] - x, GOAL_XY[1] - y)
        if d_goal <= GOAL_STOP_M:
            arrived, reason = True, "inside the goal tolerance"
            break

        # 2. replan on the freshly inflated map, widest margin that admits a
        #    route (a layout that closes a corridor should cost clearance, not
        #    motion)
        if (ticks - 1) % REPLAN_EVERY_TICKS == 0 or plan_i >= len(path):
            here = _cell(x, y)
            goal_cell = _cell(*GOAL_XY)
            found, margin = None, INFLATE_M
            for radius in INFLATE_FALLBACK_M:
                blocked = inflate(occ, radius)
                blocked.discard(here)   # standing inside our own margin
                found = astar(blocked, here, goal_cell)
                if found:
                    margin = radius
                    break
            if found:
                path = shorten(blocked, found)
                # A* lands on the goal CELL; the goal is a POINT. Without this
                # the robot parks on the cell centre and stops half a cell
                # short of a 0.30 m tolerance.
                path[-1] = GOAL_XY
                plan_i = 1
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
        ahead = [r for r in ranges[lo_e:hi_e] if r == r]
        if ahead and min(ahead) < EMERGENCY_M:
            v = 0.0                              # something is right there

        wl = (v - w * HALF_TRACK_M) / WHEEL_RADIUS_M
        wr = (v + w * HALF_TRACK_M) / WHEEL_RADIUS_M
        scale = max(1.0, abs(wl) / MAX_WHEEL_RAD_S, abs(wr) / MAX_WHEEL_RAD_S)
        wl, wr = wl / scale, wr / scale
        wheels[0].setVelocity(wl)        # front left
        wheels[2].setVelocity(wl)        # back left
        wheels[1].setVelocity(wr)        # front right
        wheels[3].setVelocity(wr)        # back right

        if ticks % 60 == 0:
            print("[oracle] t=%5.1f s xy=(%6.2f,%6.2f) goal %5.2f m "
                  "nearest beam %4.2f m waypoints %d"
                  % (robot.getTime(), x, y, d_goal,
                     min(r for r in ranges if r == r), len(path)), flush=True)

    # brake, and let the wheels actually stop before the recorder's window ends
    for m in wheels:
        m.setVelocity(0.0)
    pos = gps.getValues()
    x, y = float(pos[0]), float(pos[1])
    print("[oracle] %s: final xy=(%.3f, %.3f), %.3f m from the goal after "
          "%.1f s; %d scans / %d beams / %d occupied cells learned; "
          "narrowest planning margin %.2f m"
          % (reason, x, y, math.hypot(GOAL_XY[0] - x, GOAL_XY[1] - y),
             robot.getTime(), scans, beams_cast, len(occ), narrowest),
          flush=True)
    print("[oracle] arrived=%s" % ("yes" if arrived else "no"), flush=True)

    # Stay alive and stationary: the grader's recorder owns termination
    # (upstream has no --duration), so a driver that exits early would leave
    # the rest of the window unexplained.
    while robot.step(dt_ms) != -1:
        pass


if __name__ == "__main__":
    main()
