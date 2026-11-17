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

"""R4 BLIND control -- the oracle's control law with the LiDAR read DELETED.

The negative half of SPEC 7.1 has two useful shapes, not one. ``r4_null`` shows
the task cannot be passed by doing NOTHING; this shows it cannot be passed by
doing everything EXCEPT perceiving. It is the oracle line for line -- same
robot, same GPS/InertialUnit localisation, same planner, same arm, same grasp,
same speeds -- with exactly two changes:

* the LiDAR is never read, so the occupancy grid stays EMPTY and A* plans
  straight lines through boxes it does not know about;
* the reactive stop that would have saved it is gone with the beams.

The straight route from the start to the table is blocked by 1.014 m of
OBSTACLE_1 (an ACCEPTANCE CRITERION of every graded layout, re-derived by the
grader from the measured scene), so this driver strikes it. That is what makes
R4.5 a real assertion on this arm rather than one that cannot fail: R1 spent a
week with a collision clause that structurally could not go red, and every R1
number recorded in that window was uninformative.

This file is generated from ``r4_oracle.py`` by hand and must stay a two-line
diff from it in spirit: if it ever fails for a reason other than blindness, the
control is not controlling for blindness.
"""

import heapq
import math

from controller import Robot

# --- what the task hands every agent (benchmark_assets/scene.json) ----------
ARENA_HALF_M = 5.0
START_XY = (-4.0, -4.0)
TABLE_XY = (3.0, 3.0)
TABLE_TOP_Z = 0.60
PAD_XY = (-3.0, 3.5)
PAD_TOP_Z = 0.10
PAYLOAD_SIZE = 0.05

# --- the robot, as built in r4_mobile_manipulation.wbt ----------------------
LIDAR_NAME = "lidar"
LIDAR_OFFSET_X_M = 0.27
WHEEL_RADIUS_M = 0.10
HALF_TRACK_M = 0.22
MAX_WHEEL_RAD_S = 14.0
WHEELS = ("left wheel", "right wheel")
#: Circumscribed radius of the chassis: hypot(0.25, 0.18) = 0.308, and the
#: wheels reach |y| = 0.245, so 0.33 m bounds the whole footprint.
ROBOT_RADIUS_M = 0.33

#: The arm, from the world file. MOUNT is the yaw axis in the robot frame;
#: L2/L3 are the two links and TOOL_M is the offset from the wrist origin to
#: the point midway between the fingers.
MOUNT = (0.10, 0.0, 0.45)
L2, L3, TOOL_M = 0.38, 0.34, 0.055
REACH_M = L2 + L3 + TOOL_M

ARM_JOINTS = ("j1", "j2", "j3", "j4")
FINGERS = ("finger left", "finger right")
#: Authored open; closed leaves 1.5 mm of interference per face on a 0.05 m
#: cube, which is what makes the grip a FRICTION grip.
FINGER_OPEN = (0.0, 0.0)
FINGER_CLOSED = (-0.020, 0.020)

# --- how the fan is consumed (identical policy to r1_oracle) ----------------
MARK_MAX_M = 6.0
EMERGENCY_M = 0.55
EMERGENCY_HALF_BINS = 20
#: Inside this distance of the current waypoint the reactive stop and the
#: planner are both switched off and the robot drives straight at it.
#:
#: MEASURED, not tuned by taste: the pick standoff is 0.75 m from the table
#: CENTRE, which puts the lidar (0.27 m forward) 0.08 m from the table's face,
#: so an unconditional 0.55 m emergency stop can never let the robot park
#: there. On the second oracle run it deadlocked at 0.38 m from the standoff
#: and sat there for 120 s. The planner has the mirror-image problem: it
#: inflates the table it can see by >= 0.52 m, which blocks every cell around
#: the parking spot. Both are correct behaviours far from a target and wrong
#: next to one, so the last 1.2 m is driven open-loop-in-geometry towards a
#: point the TASK published -- not towards something the robot guessed.
FINAL_APPROACH_M = 1.2

# --- planner ----------------------------------------------------------------
GRID_CELL_M = 0.2
INFLATE_FALLBACK_M = (0.75, 0.62, 0.52)
CONTROL_PERIOD_MS = 64
REPLAN_EVERY_TICKS = 4
LOOKAHEAD_M = 0.8
V_MAX_MPS = 0.45
W_MAX_RPS = 1.3
K_HEADING = 2.2
TURN_IN_PLACE_RAD = 0.7
ARRIVE_M = 0.12
ALIGN_RAD = 0.02

# --- manipulation -----------------------------------------------------------
#: How far from the fixture CENTRE the base parks. Both were checked against
#: REACH_M and against the chassis footprint -- see the world file's header.
PICK_STANDOFF_M = 0.75
PLACE_STANDOFF_M = 0.70
#: Cartesian interpolation rate for the tool. Slow enough that the payload is
#: never whipped out of the fingers by the arm itself.
TOOL_SPEED_MPS = 0.14
#: The grasp height. The fingers are 0.036 m tall, so gripping at 0.632 puts
#: them on the cube's middle 0.036 m and keeps their lower face 0.014 m clear
#: of the table top.
GRASP_Z = 0.627
#: Lifted clear of the table WHILE STILL OVER IT (r4_core asks for exactly
#: that): the payload's lowest face ends up ~0.755 m up against a 0.60 m table.
LIFT_Z = 0.80
LIFT_BACK_M = 0.10
#: Carried over the base, high enough to clear a 0.5 m obstacle top by the
#: grader's CARRY_CLEARANCE_M even if the payload's xy crosses one.
TUCK_ROBOT_FRAME = (0.25, 0.0, 0.76)
#: Release: the payload's lowest face ends ~0.02 m above the pad top.
RELEASE_Z = 0.165
APPROACH_BACK_M = 0.12

GIVE_UP_S = 300.0

_N = int(round(2 * ARENA_HALF_M / GRID_CELL_M))


# --- grid helpers (verbatim policy from r1_oracle) --------------------------

def _cell(x, y):
    ix = int((x + ARENA_HALF_M) / GRID_CELL_M)
    iy = int((y + ARENA_HALF_M) / GRID_CELL_M)
    return min(max(ix, 0), _N - 1), min(max(iy, 0), _N - 1)


def _centre(ix, iy):
    return (ix * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0,
            iy * GRID_CELL_M - ARENA_HALF_M + GRID_CELL_M / 2.0)


def _disc_offsets(radius_m):
    r = int(math.ceil(radius_m / GRID_CELL_M))
    return [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if math.hypot(dx, dy) * GRID_CELL_M <= radius_m + 1e-9]


def _wall_margin(radius_m):
    out = set()
    for ix in range(_N):
        for iy in range(_N):
            cx, cy = _centre(ix, iy)
            if (ARENA_HALF_M - abs(cx) < radius_m
                    or ARENA_HALF_M - abs(cy) < radius_m):
                out.add((ix, iy))
    return out


_DISCS = {r: _disc_offsets(r) for r in INFLATE_FALLBACK_M}
_WALLS = {r: _wall_margin(r) for r in INFLATE_FALLBACK_M}


def inflate(occ, radius_m):
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
    """8-connected A*. Never-seen cells are free -- optimism about the unknown
    is what turns a planner into an explorer."""
    if goal in blocked:
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
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (GRID_CELL_M / 2.0)) + 1
    for i in range(n + 1):
        t = i / float(n)
        if _cell(a[0] + (b[0] - a[0]) * t,
                 a[1] + (b[1] - a[1]) * t) in blocked:
            return False
    return True


def shorten(blocked, path):
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


# --- arm kinematics ----------------------------------------------------------

def arm_ik(target_robot):
    """Joint angles putting the tool point at ``target_robot`` (robot frame).

    ``None`` when the point is out of reach -- never a clamped near-miss, which
    would silently grasp thin air. The tool axis is held HORIZONTAL, so the
    fingers always approach a body from the side.
    """
    dx = target_robot[0] - MOUNT[0]
    dy = target_robot[1] - MOUNT[1]
    dz = target_robot[2] - MOUNT[2]
    q1 = math.atan2(dy, dx)
    r = math.hypot(dx, dy)
    a, b = r - TOOL_M, dz
    d = math.hypot(a, b)
    if d > L2 + L3 - 1e-4 or d < abs(L2 - L3) + 1e-4:
        return None
    cos_b = (d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    cos_b = max(-1.0, min(1.0, cos_b))
    best = None
    for sign in (1.0, -1.0):
        beta = sign * math.acos(cos_b)
        alpha = (math.atan2(b, a)
                 - math.atan2(L3 * math.sin(beta), L2 + L3 * math.cos(beta)))
        elbow_h = L2 * math.sin(alpha)          # elbow-up is the higher one
        if best is None or elbow_h > best[0]:
            best = (elbow_h, alpha, beta)
    _h, alpha, beta = best
    return (q1, -alpha, -beta, alpha + beta)


def world_to_robot(pt, origin, rpy):
    """A world point in the robot's own BODY frame, using the full attitude.

    Not a planar yaw-only rotation, and that was measured rather than assumed:
    on the first oracle run the base sat ~0.07 rad nose-up while the arm was
    extended (two wheels plus two frictionless caster spheres is a rocking
    support), so a yaw-only transform put the commanded grasp 0.055 m ABOVE
    the cube -- the fingers closed on air, the cube never left the table, and
    R4.6-R4.8 all failed for what looked like a grasp problem and was a frame
    problem. The InertialUnit already reports roll and pitch; using them is
    ordinary closed-loop practice, not extra information.

    ``rpy`` is the InertialUnit's (roll, pitch, yaw). The body->world rotation
    is Rz(yaw) Ry(pitch) Rx(roll); this applies its transpose.
    """
    roll, pitch, yaw = rpy
    d = (pt[0] - origin[0], pt[1] - origin[1], pt[2] - origin[2])
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    # R = Rz Ry Rx, rows of R^T = columns of R
    r00, r01, r02 = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    r10, r11, r12 = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    r20, r21, r22 = -sp, cp * sr, cp * cr
    return (r00 * d[0] + r10 * d[1] + r20 * d[2],
            r01 * d[0] + r11 * d[1] + r21 * d[2],
            r02 * d[0] + r12 * d[1] + r22 * d[2])


# --- the drive ---------------------------------------------------------------

def main():
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    control_every = max(1, int(round(CONTROL_PERIOD_MS / float(dt_ms))))

    lidar = robot.getDevice(LIDAR_NAME)
    lidar.enable(dt_ms * control_every)
    n_beams = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    angles = [fov / 2.0 - (i + 0.5) * fov / n_beams for i in range(n_beams)]
    print("[blind] lidar '%s': %d samples over %.2f deg, max range %.1f m"
          % (LIDAR_NAME, n_beams, math.degrees(fov), lidar.getMaxRange()),
          flush=True)

    camera = robot.getDevice("camera")
    if camera is not None:
        camera.enable(dt_ms * control_every * 4)

    gps = robot.getDevice("gps")
    gps.enable(dt_ms)
    imu = robot.getDevice("imu")
    imu.enable(dt_ms)

    wheels = [robot.getDevice(w) for w in WHEELS]
    for m in wheels:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    arm = [robot.getDevice(j) for j in ARM_JOINTS]
    fingers = [robot.getDevice(f) for f in FINGERS]
    finger_sensors = []
    for nm in FINGERS:
        try:
            ps = robot.getDevice("%s sensor" % nm)
            ps.enable(dt_ms)
            finger_sensors.append(ps)
        except Exception:            # noqa: BLE001
            pass

    occ = set()
    path, plan_i = [], 0
    ticks, step = 0, 0
    scans, beams_cast = 0, 0
    narrowest = INFLATE_FALLBACK_M[0]

    # --- the plan, as a list of stages. Every stage is (name, payload). ----
    stages = [
        ("tool", TUCK_ROBOT_FRAME, "robot", 3.0),   # fold the arm before driving
        ("grip", FINGER_OPEN, 0.4),
        ("nav", _standoff(TABLE_XY, PICK_STANDOFF_M, PAD_XY)),
        ("face", TABLE_XY),
        ("creep", TABLE_XY, PICK_STANDOFF_M),
        ("face", TABLE_XY),
        ("tool", (TABLE_XY[0], TABLE_XY[1], GRASP_Z), "world_back", 0.2),
        ("tool", (TABLE_XY[0], TABLE_XY[1], GRASP_Z), "world", 0.3),
        ("grip", FINGER_CLOSED, 2.2),
        ("tool", (TABLE_XY[0], TABLE_XY[1], LIFT_Z), "world_back", 0.4),
        ("tool", TUCK_ROBOT_FRAME, "robot", 0.6),
        ("nav", _standoff(PAD_XY, PLACE_STANDOFF_M, TABLE_XY)),
        ("face", PAD_XY),
        ("creep", PAD_XY, PLACE_STANDOFF_M),
        ("face", PAD_XY),
        ("tool", (PAD_XY[0], PAD_XY[1], RELEASE_Z + 0.18), "world", 0.3),
        ("tool", (PAD_XY[0], PAD_XY[1], RELEASE_Z), "world", 0.6),
        ("grip", FINGER_OPEN, 1.0),
        ("tool", (PAD_XY[0], PAD_XY[1], RELEASE_Z + 0.22), "world", 0.3),
        ("tool", TUCK_ROBOT_FRAME, "robot", 0.5),
        ("done", None),
    ]
    stage_i = 0
    stage_t0 = None
    tool_from = None
    tool_progress = 0.0
    # The tool point this controller last COMMANDED, in the robot frame. Held
    # here rather than re-derived from the motors: a motor reports the target
    # it was given, which at t=0 is the authored zero pose whose tool point
    # sits exactly on the reach boundary -- so an FK read would open with an
    # unreachable interpolation start. The first stage commands the tuck pose
    # outright and lets the motors slew to it at their own maxVelocity.
    tool_cmd = TUCK_ROBOT_FRAME
    mid = n_beams // 2
    lo_e = max(0, mid - EMERGENCY_HALF_BINS)
    hi_e = min(n_beams, mid + EMERGENCY_HALF_BINS + 1)

    while robot.step(dt_ms) != -1 and robot.getTime() < GIVE_UP_S:
        step += 1
        if step % control_every:
            continue
        ticks += 1
        now = robot.getTime()

        pos = gps.getValues()
        x, y = float(pos[0]), float(pos[1])
        rpy = [float(a) for a in imu.getRollPitchYaw()]
        yaw = rpy[2]
        origin = (x, y, float(pos[2]))
        # THE ONE CHANGE: the fan is never cast, so ``occ`` stays empty and
        # every plan is a straight line through whatever is actually there.
        ranges = None

        if stage_i >= len(stages):
            break
        kind = stages[stage_i][0]
        if stage_t0 is None:
            stage_t0 = now
            tool_from = None
            print("[blind] t=%6.2f stage %d %s xy=(%.2f, %.2f) yaw=%.3f"
                  % (now, stage_i, kind, x, y, yaw), flush=True)

        if kind == "done":
            _stop(wheels)
            break

        if kind == "grip":
            _, target, hold = stages[stage_i]
            for m, p in zip(fingers, target):
                m.setPosition(p)
            _stop(wheels)
            if now - stage_t0 >= hold:
                if finger_sensors:
                    got = [m.getValue() for m in finger_sensors]
                    print("[blind] finger positions %s (commanded %s)"
                          % ([round(g, 5) for g in got], list(target)),
                          flush=True)
                stage_i, stage_t0 = stage_i + 1, None
            continue

        if kind == "tool":
            _, target, frame, hold = stages[stage_i]
            if frame == "robot":
                goal = target
            else:
                goal = world_to_robot(target, origin, rpy)
                if frame == "world_back":
                    n = math.hypot(goal[0], goal[1]) or 1.0
                    goal = (goal[0] - APPROACH_BACK_M * goal[0] / n,
                            goal[1] - APPROACH_BACK_M * goal[1] / n,
                            goal[2])
            if tool_from is None:
                tool_from = tool_cmd
                tool_progress = 0.0
            span = max(1e-6, math.dist(tool_from, goal))
            tool_progress = min(1.0, tool_progress
                                + TOOL_SPEED_MPS * (CONTROL_PERIOD_MS / 1000.0)
                                / span)
            here = tuple(tool_from[k] + (goal[k] - tool_from[k])
                         * tool_progress for k in range(3))
            q = arm_ik(here)
            if q is None:
                print("[blind] UNREACHABLE tool target %r (stage %d)"
                      % (here, stage_i), flush=True)
            else:
                for m, a in zip(arm, q):
                    m.setPosition(a)
                tool_cmd = here
            _stop(wheels)
            if tool_progress >= 1.0 and now - stage_t0 >= hold:
                stage_i, stage_t0 = stage_i + 1, None
            continue

        if kind == "creep":
            # The nav stage stops within ARRIVE_M of the standoff, which on the
            # first run left the base 0.85 m from the table -- past the arm's
            # reach, so the IK ran at the singularity and the grasp missed.
            # This drives the LAST few centimetres closed-loop on the measured
            # distance to the fixture itself, which is the number that decides
            # whether the payload is reachable.
            _, target, want = stages[stage_i]
            d = math.hypot(target[0] - x, target[1] - y)
            err_a = _wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
            if abs(d - want) <= 0.02 or now - stage_t0 > 12.0:
                _stop(wheels)                  # the timeout never stalls a run
                stage_i, stage_t0 = stage_i + 1, None
            else:
                v = max(-0.12, min(0.12, 0.6 * (d - want)))
                _drive(wheels, v, max(-0.4, min(0.4, 1.5 * err_a)))
            continue

        if kind == "face":
            _, target = stages[stage_i]
            err = _wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
            if abs(err) <= ALIGN_RAD:
                _stop(wheels)
                stage_i, stage_t0 = stage_i + 1, None
            else:
                w = max(-0.8, min(0.8, 2.0 * err))
                _drive(wheels, 0.0, w)
            continue

        # --- kind == "nav" ------------------------------------------------
        goal_xy = stages[stage_i][1]
        d_goal = math.hypot(goal_xy[0] - x, goal_xy[1] - y)
        if d_goal <= ARRIVE_M:
            _stop(wheels)
            path, plan_i = [], 0
            stage_i, stage_t0 = stage_i + 1, None
            continue

        if d_goal <= FINAL_APPROACH_M:
            path, plan_i = [(x, y), goal_xy], 1
        elif (ticks - 1) % REPLAN_EVERY_TICKS == 0 or plan_i >= len(path):
            here = _cell(x, y)
            goal_cell = _cell(*goal_xy)
            found, margin, blocked = None, INFLATE_FALLBACK_M[0], set()
            for radius in INFLATE_FALLBACK_M:
                blocked = inflate(occ, radius)
                blocked.discard(here)
                blocked.discard(goal_cell)
                found = astar(blocked, here, goal_cell)
                if found:
                    margin = radius
                    break
            if found:
                path = shorten(blocked, found)
                path[-1] = goal_xy
                plan_i = 1
                narrowest = min(narrowest, margin)
            elif not path:
                path, plan_i = [(x, y), goal_xy], 1
                narrowest = 0.0

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
            v = min(v, 0.9 * d_goal + 0.05)
        # ...and with the beams goes the reactive stop that would have saved
        # it. A blind driver has nothing to stop for.
        pass
        _drive(wheels, v, w)

        if ticks % 80 == 0:
            print("[blind] t=%6.2f xy=(%6.2f,%6.2f) -> (%5.2f,%5.2f) %5.2f m "
                  "waypoints %d cells %d"
                  % (now, x, y, goal_xy[0], goal_xy[1], d_goal, len(path),
                     len(occ)), flush=True)

    _stop(wheels)
    pos = gps.getValues()
    print("[blind] finished stage %d/%d at t=%.2f, xy=(%.3f, %.3f); "
          "%d scans / %d beams / %d occupied cells learned; narrowest "
          "planning margin %.2f m"
          % (stage_i, len(stages) - 1, robot.getTime(), float(pos[0]),
             float(pos[1]), scans, beams_cast, len(occ), narrowest), flush=True)
    print("[blind] delivered=%s"
          % ("yes" if stage_i >= len(stages) - 1 else "no"), flush=True)

    # The grader's recorder owns termination (upstream has no --duration), so
    # a driver that exited early would leave the rest of the window
    # unexplained. Hold still instead.
    while robot.step(dt_ms) != -1:
        pass


def _standoff(target, distance, approach_from):
    """A parking spot ``distance`` from ``target``, on the side we come from."""
    dx = approach_from[0] - target[0]
    dy = approach_from[1] - target[1]
    n = math.hypot(dx, dy) or 1.0
    return (target[0] + distance * dx / n, target[1] + distance * dy / n)


def _drive(wheels, v, w):
    wl = (v - w * HALF_TRACK_M) / WHEEL_RADIUS_M
    wr = (v + w * HALF_TRACK_M) / WHEEL_RADIUS_M
    scale = max(1.0, abs(wl) / MAX_WHEEL_RAD_S, abs(wr) / MAX_WHEEL_RAD_S)
    wheels[0].setVelocity(wl / scale)
    wheels[1].setVelocity(wr / scale)


def _stop(wheels):
    for m in wheels:
        m.setVelocity(0.0)


if __name__ == "__main__":
    main()
