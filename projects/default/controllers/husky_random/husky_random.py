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

"""Husky goal-seeking driver with crater + boundary avoidance.

Picks a random navigable target point on the map, drives toward it,
and lets a potential-field bias bend the path around craters and the
world edge. When it arrives (or gets stuck) it picks a new goal. The
result is purposeful continuous motion: each husky always has
somewhere to go, and the avoidance forces are strong enough close to
a crater rim that the husky stops and turns in place rather than
overshooting into the bowl.

Falls back gracefully when the URDFRobot does not expose supervisor
APIs or when the recipe did not write a customData payload — in that
case the controller drives a simple random walk so the same script
still works in non-Mars worlds.

How a goal is picked:
- Uniform random point inside ``world`` minus ``boundary_margin``.
- Reject if within ``rim + GOAL_CRATER_MARGIN`` of any crater.
- Reject if too close to the husky's current position so it actually
  has to drive somewhere.

How steering combines:
- Goal direction = unit vector toward the current goal.
- Crater repulsion = quadratic ramp inside ``rim + AVOIDANCE_MARGIN_M``,
  squared so the closer the husky is to a rim, the harder it pushes
  back. Wins easily over the linear goal pull near a rim.
- Boundary repulsion = quadratic ramp once the husky is within
  ``boundary_margin`` of any world edge.
- Danger stop = forward speed clamped to 0 while a rim is closer
  than ``DANGER_RIM_DISTANCE_M``. Husky turns in place to align with
  the safe direction before resuming motion.

Stall + replan:
- Motor torque feedback signals near-stall state. After ~0.8 s of
  torque-pegged forward command, the controller picks a fresh goal.
- A no-progress timer also replans every ~6 s if the husky hasn't
  moved at least ~3 m, which catches stalls the torque check missed
  (free-spinning wheels on dust, etc.).
"""

from __future__ import annotations

import json
import math
import random

try:
    # Supervisor is a subclass of Robot — same API plus getSelf() / getFromDef.
    # We instantiate it so position tracking works when the world declares the
    # robot as supervisor TRUE; on a plain robot, getSelf() returns None and
    # the controller transparently falls back to wheel-only stuck detection.
    from omnisim import Supervisor as _RobotImpl
except Exception:
    from omnisim import Robot as _RobotImpl
Robot = _RobotImpl


MAX_WHEEL_SPEED = 6.0

# Goal selection.
GOAL_REACH_RADIUS_M = 2.5
GOAL_CRATER_MARGIN_M = 4.0     # never pick a goal within rim+this
GOAL_MIN_DISTANCE_M = 8.0       # new goal at least this far from current pos
GOAL_PICK_ATTEMPTS = 64

# Steering.
SPIN_GAIN = 4.0                # rad/s of spin per rad of yaw error
ALIGN_FAST_THRESHOLD = 0.35    # rad — below this we drive at full forward
MIN_FORWARD = 0.30             # of MAX_WHEEL_SPEED — even when turning hard

# Crater + boundary avoidance.
AVOIDANCE_MARGIN_M = 6.0       # repulsion ramp begins at rim + this
DANGER_RIM_DISTANCE_M = 1.5    # forward speed = 0 when rim is closer than this
CRATER_REPULSION_STRENGTH = 4.0
BOUNDARY_REPULSION_STRENGTH = 3.0
BOUNDARY_DEFAULT_MARGIN_M = 8.0

# Stall + replan.
STALL_TORQUE_THRESHOLD_NM = 12.0
STALL_DEBOUNCE_MS = 800
NO_PROGRESS_DISTANCE_M = 3.0   # required progress over the window
NO_PROGRESS_WINDOW_MS = 6000

# Random walk fallback (no supervisor / no customData).
RW_RAMP = 0.08
RW_MIN_HOLD_S = 2.5
RW_MAX_HOLD_S = 5.5

# Reactive collision recovery for the random-walk fallback.
# Detects the "stuck" condition by comparing commanded vs. actual wheel speed:
# if we tell the wheels to turn at cruise rate but they're barely spinning,
# something is holding the chassis in place (wall, crate, terrain). Then we
# run BACKUP -> TURN -> CRUISE to escape.
#
# Wheel velocity is the right signal here. Torque feedback is ambiguous
# because Webots' velocity-mode PID + acceleration limits give moderate
# steady-state torque values whether the wheel is loaded or pushing on a wall.
RW_STUCK_COMMANDED_RAD_S = 2.0    # only trip when commanding meaningful motion (>this rad/s)
RW_STUCK_WHEEL_SPEED_RAD_S = 1.2  # wheels delivering less than this rad/s = obstacle
                                  # (cruise normally runs at ~3, so 1.2 = ~60% slowdown.
                                  # Catches partial wedges, not just full stops.)
RW_COLLISION_DEBOUNCE_MS = 150    # sustained-stuck duration before triggering recovery
                                  # (short enough to react to glancing wall-bounces)
RW_WHEEL_SPEED_TAU = 0.12         # low-pass time constant (s) for wheel-speed filtering

# Position-based "no progress" detector. Most reliable stuck signal when a
# supervisor handle is available: if the chassis hasn't physically moved much
# while we're commanding motion, something is holding it in place — even if
# the wheels happen to be spinning at commanded speed (slip, lifted wheel,
# wedged at an angle that lets wheels rotate but body not translate).
RW_NO_PROGRESS_WINDOW_S = 4.0     # rolling window of (position, time) samples
RW_NO_PROGRESS_DISTANCE_M = 0.5   # less than this displacement over the window = stuck
RW_RECOVERY_BACKUP_S = 1.5        # long enough to actually clear the obstacle
RW_RECOVERY_TURN_S = 2.8          # long enough for a real ~180° skid-steer pivot
RW_RECOVERY_BACKUP_FRAC = 0.7     # of MAX_WHEEL_SPEED, applied to both sides
RW_RECOVERY_TURN_FRAC = 1.0       # of MAX_WHEEL_SPEED, applied with opposing signs


def clamp(value, low, high):
    return max(low, min(high, value))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_orientation(orient) -> float:
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def _parse_payload(robot):
    craters: list[tuple[float, float, float]] = []
    world_centre = (40.0, 40.0)
    world_size = 80.0
    boundary_margin = BOUNDARY_DEFAULT_MARGIN_M
    raw = ""
    try:
        raw = robot.getCustomData() or ""
    except Exception:
        return craters, world_centre, world_size, boundary_margin
    if not raw.strip():
        return craters, world_centre, world_size, boundary_margin
    try:
        payload = json.loads(raw)
    except Exception:
        return craters, world_centre, world_size, boundary_margin

    centre_data = payload.get("world_centre")
    if isinstance(centre_data, (list, tuple)) and len(centre_data) >= 2:
        try:
            world_centre = (float(centre_data[0]), float(centre_data[1]))
        except (TypeError, ValueError):
            pass
    if isinstance(payload.get("world_size"), (int, float)):
        world_size = float(payload["world_size"])
    if isinstance(payload.get("boundary_margin"), (int, float)):
        boundary_margin = float(payload["boundary_margin"])

    crater_data = payload.get("craters")
    if isinstance(crater_data, list):
        for item in crater_data:
            if not isinstance(item, (list, tuple)) or len(item) < 3:
                continue
            try:
                cx, cy, cr = float(item[0]), float(item[1]), float(item[2])
            except (TypeError, ValueError):
                continue
            craters.append((cx, cy, cr))
    return craters, world_centre, world_size, boundary_margin


def _get_pose(self_node):
    if self_node is None:
        return None
    try:
        pos = self_node.getPosition()
        orient = self_node.getOrientation()
    except Exception:
        return None
    if pos is None or len(pos) < 2:
        return None
    return float(pos[0]), float(pos[1]), _yaw_from_orientation(orient)


def _select_goal(rng, world_centre, world_size, boundary_margin,
                  craters, current_pos):
    """Pick a random navigable point. Falls back to world centre if
    no candidate passes after GOAL_PICK_ATTEMPTS tries."""
    cx, cy = world_centre
    half = max(world_size / 2.0 - boundary_margin, 5.0)
    cur_x = cur_y = None
    if current_pos is not None:
        cur_x, cur_y = current_pos[0], current_pos[1]
    for _ in range(GOAL_PICK_ATTEMPTS):
        x = cx + rng.uniform(-half, half)
        y = cy + rng.uniform(-half, half)
        bad = False
        for crx, cry, crr in craters:
            if math.hypot(x - crx, y - cry) < crr + GOAL_CRATER_MARGIN_M:
                bad = True
                break
        if bad:
            continue
        if cur_x is not None:
            if math.hypot(x - cur_x, y - cur_y) < GOAL_MIN_DISTANCE_M:
                continue
        return (x, y)
    return (cx, cy)


def _compute_steering(x, y, yaw, goal, craters,
                      world_centre, world_size, boundary_margin):
    """Return ``(target_forward, target_spin, in_danger, min_rim_distance)``.

    All speeds are absolute rad/s on the wheel motors; the caller
    feeds them through tank-steering math.
    """
    gx, gy = goal
    dx, dy = gx - x, gy - y
    dist_goal = math.hypot(dx, dy)
    if dist_goal > 1e-6:
        goal_dir_x = dx / dist_goal
        goal_dir_y = dy / dist_goal
    else:
        goal_dir_x, goal_dir_y = 1.0, 0.0

    # Crater repulsion (quadratic).
    rep_x = 0.0
    rep_y = 0.0
    min_rim = float("inf")
    for crx, cry, crr in craters:
        ex, ey = x - crx, y - cry
        dist = math.hypot(ex, ey)
        rim_dist = dist - crr
        if rim_dist < min_rim:
            min_rim = rim_dist
        threshold = crr + AVOIDANCE_MARGIN_M
        if dist >= threshold or dist < 1e-6:
            continue
        # Linear factor in [0, 1], squared for sharp close-up push.
        linear = (threshold - dist) / max(AVOIDANCE_MARGIN_M, 1e-6)
        intensity = linear * linear * CRATER_REPULSION_STRENGTH
        rep_x += (ex / dist) * intensity
        rep_y += (ey / dist) * intensity

    # Boundary repulsion (quadratic).
    cx_w, cy_w = world_centre
    half_world = world_size / 2.0
    inset = max(half_world - boundary_margin, 1.0)
    rel_x = x - cx_w
    rel_y = y - cy_w

    def _bound_push(rel, axis_inset, margin):
        if rel > axis_inset:
            t = (rel - axis_inset) / max(margin, 1e-6)
            return -t * t * BOUNDARY_REPULSION_STRENGTH
        if rel < -axis_inset:
            t = (-axis_inset - rel) / max(margin, 1e-6)
            return t * t * BOUNDARY_REPULSION_STRENGTH
        return 0.0

    rep_x += _bound_push(rel_x, inset, boundary_margin)
    rep_y += _bound_push(rel_y, inset, boundary_margin)

    # Combine: goal pull + repulsion.
    desired_x = goal_dir_x + rep_x
    desired_y = goal_dir_y + rep_y
    if abs(desired_x) < 1e-6 and abs(desired_y) < 1e-6:
        # Pure repulsion cancellation — just point along the repulsion
        # vector if it exists, otherwise stay aligned with current yaw.
        if abs(rep_x) > 1e-6 or abs(rep_y) > 1e-6:
            desired_x, desired_y = rep_x, rep_y
        else:
            desired_x = math.cos(yaw)
            desired_y = math.sin(yaw)

    desired_yaw = math.atan2(desired_y, desired_x)
    yaw_error = _wrap_pi(desired_yaw - yaw)
    target_spin = clamp(
        yaw_error * SPIN_GAIN,
        -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED,
    )

    # Forward speed: full speed when aligned + safe; crawl when turning;
    # zero when too close to a rim (turn in place).
    if min_rim < DANGER_RIM_DISTANCE_M:
        target_forward = 0.0
        in_danger = True
    else:
        # Cosine-weighted speed: full when yaw_error≈0, drops to
        # MIN_FORWARD when error >= 90°.
        alignment = max(0.0, math.cos(yaw_error))
        target_forward = MAX_WHEEL_SPEED * (
            MIN_FORWARD + (1.0 - MIN_FORWARD) * alignment
        )
        in_danger = False
    return target_forward, target_spin, in_danger, min_rim


_TRACE_INTERVAL_MS = 2000
# Hardcoded trace directory. The controller writes a per-husky log if
# this directory already exists at startup (env-var-based trace
# wiring proved unreliable through Webots' subprocess launcher).
# Create the directory before launching to enable tracing.
_TRACE_DIRS = (
    r"C:\tmp\husky_trace",
    "/tmp/husky_trace",
)


def _open_trace_file(name: str):
    import os
    safe_name = "".join(
        c if c.isalnum() or c in "_-" else "_" for c in name
    )
    for d in _TRACE_DIRS:
        try:
            if os.path.isdir(d):
                return open(
                    os.path.join(d, f"{safe_name}.log"),
                    "w", encoding="utf-8", buffering=1,
                )
            # Try to create the dir, in case tracing was requested
            # but the directory hasn't been pre-created.
            os.makedirs(d, exist_ok=True)
            return open(
                os.path.join(d, f"{safe_name}.log"),
                "w", encoding="utf-8", buffering=1,
            )
        except Exception:
            continue
    return None


def _run_goal_seeker(robot, time_step, motors, left_motors, right_motors,
                     self_node, craters, world_centre, world_size,
                     boundary_margin, rng):
    """Main loop when supervisor + payload are available."""
    name = robot.getName()
    trace_file = _open_trace_file(name)

    target_forward = 0.0
    target_spin = 0.0
    current_forward = 0.0
    current_spin = 0.0
    sim_ms = 0
    stall_started_ms = -1
    progress_window_start_ms = 0
    progress_window_pos = None
    goal = None
    next_trace_ms = 0
    in_danger_now = False
    min_rim_now = float("inf")

    while robot.step(time_step) != -1:
        sim_ms += time_step
        pose = _get_pose(self_node)
        if pose is None:
            # Lost the supervisor mid-run — drive nowhere safely.
            for m in motors:
                m.setVelocity(0.0)
            continue
        x, y, yaw = pose

        if goal is None:
            goal = _select_goal(
                rng, world_centre, world_size, boundary_margin,
                craters, current_pos=(x, y),
            )
            progress_window_start_ms = sim_ms
            progress_window_pos = (x, y)

        # Arrived?
        if math.hypot(goal[0] - x, goal[1] - y) < GOAL_REACH_RADIUS_M:
            goal = _select_goal(
                rng, world_centre, world_size, boundary_margin,
                craters, current_pos=(x, y),
            )
            progress_window_start_ms = sim_ms
            progress_window_pos = (x, y)

        # No-progress replan: if we haven't moved enough in the window,
        # pick a new goal and reset.
        if sim_ms - progress_window_start_ms > NO_PROGRESS_WINDOW_MS:
            if progress_window_pos is not None:
                moved = math.hypot(
                    x - progress_window_pos[0], y - progress_window_pos[1]
                )
                if moved < NO_PROGRESS_DISTANCE_M:
                    goal = _select_goal(
                        rng, world_centre, world_size, boundary_margin,
                        craters, current_pos=(x, y),
                    )
            progress_window_start_ms = sim_ms
            progress_window_pos = (x, y)

        target_forward, target_spin, in_danger_now, min_rim_now = _compute_steering(
            x, y, yaw, goal, craters,
            world_centre, world_size, boundary_margin,
        )

        # Periodic trace to a per-husky file (Webots' --stdout/--stderr
        # forwarding is unreliable in some shell setups).
        if trace_file is not None and sim_ms >= next_trace_ms:
            state = "DANGER" if in_danger_now else "DRIVE"
            trace_file.write(
                f"t_ms={sim_ms} "
                f"pos=({x:.2f},{y:.2f}) yaw={yaw:.2f} "
                f"goal=({goal[0]:.2f},{goal[1]:.2f}) "
                f"min_rim_m={min_rim_now:.2f} state={state}\n"
            )
            next_trace_ms = sim_ms + _TRACE_INTERVAL_MS

        # Stall detection — if torque-pegged while commanding forward,
        # replan to escape.
        try:
            torques = [abs(m.getTorqueFeedback()) for m in motors]
        except Exception:
            torques = [0.0] * len(motors)
        mean_torque = sum(torques) / len(torques) if torques else 0.0
        if (
            mean_torque > STALL_TORQUE_THRESHOLD_NM
            and target_forward > 1.0
            and not math.isnan(mean_torque)
        ):
            if stall_started_ms < 0:
                stall_started_ms = sim_ms
            elif sim_ms - stall_started_ms > STALL_DEBOUNCE_MS:
                goal = _select_goal(
                    rng, world_centre, world_size, boundary_margin,
                    craters, current_pos=(x, y),
                )
                progress_window_start_ms = sim_ms
                progress_window_pos = (x, y)
                stall_started_ms = -1
        else:
            stall_started_ms = -1

        # Ramp current velocities toward targets so motion is smooth.
        ramp_step = 0.12 * MAX_WHEEL_SPEED
        current_forward += clamp(
            target_forward - current_forward, -ramp_step, ramp_step
        )
        current_spin += clamp(
            target_spin - current_spin, -ramp_step, ramp_step
        )
        left_speed = clamp(
            current_forward - current_spin,
            -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED,
        )
        right_speed = clamp(
            current_forward + current_spin,
            -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED,
        )
        for m in left_motors:
            m.setVelocity(left_speed)
        for m in right_motors:
            m.setVelocity(right_speed)


def _run_random_walk(robot, time_step, motors, left_motors, right_motors,
                     sensors, self_node, rng):
    """Random walk with reactive collision-recovery state machine:

        CRUISE  -- stuck detected -->  BACKUP  -- timeout -->  TURN  -- timeout --> CRUISE

    Stuck detection runs two signals in parallel and either one triggers:
      - wheel-speed: commanded > 2 rad/s but actual < 1.2 rad/s for 150 ms
        (catches obstacles the wheels can't push through)
      - chassis no-progress: less than 0.5 m of XY travel over a 4 s window
        while commanding motion (catches slip / wedged-but-wheels-still-turning).
        Only active when self_node (supervisor handle) is provided.

    Recovery: reverse 1.5 s, spin in place 2.8 s in a random direction,
    resume cruise — so the husky never wedges itself permanently.
    """
    name = robot.getName()
    trace_file = _open_trace_file(name)
    next_trace_ms = 0
    target_forward = 0.0
    target_spin = 0.0
    current_forward = 0.0
    current_spin = 0.0
    hold_until_ms = 0
    sim_ms = 0
    cruise_speed = 0.55 * MAX_WHEEL_SPEED  # gentle cruise to avoid hard hits

    state = "CRUISE"
    state_until_ms = 0
    collision_pegged_ms = 0

    # Wheel-speed tracking for stuck detection. We low-pass the per-step
    # delta-angle so PID jitter and physics noise don't fool us.
    prev_angles = None
    wheel_speed_filt = 0.0
    dt_s = time_step / 1000.0
    alpha = dt_s / max(RW_WHEEL_SPEED_TAU, dt_s)  # IIR coefficient

    # Position tracking via supervisor handle (if available). Used for both the
    # no-progress detector and per-step XY logging in the trace, so we can
    # later confirm "the robot kept moving" empirically.
    pos_history: list[tuple[int, float, float]] = []  # (sim_ms, x, y) samples
    initial_xy = None
    cumulative_distance = 0.0
    last_xy = None

    while robot.step(time_step) != -1:
        sim_ms += time_step

        # Sample current chassis position + yaw (None when no supervisor).
        cur_x = cur_y = cur_yaw = None
        if self_node is not None:
            try:
                pos = self_node.getPosition()
                if pos is not None and len(pos) >= 2:
                    cur_x = float(pos[0])
                    cur_y = float(pos[1])
                orient = self_node.getOrientation()
                cur_yaw = _yaw_from_orientation(orient)
            except Exception:
                pass
        if cur_x is not None:
            if initial_xy is None:
                initial_xy = (cur_x, cur_y)
            if last_xy is not None:
                cumulative_distance += math.hypot(
                    cur_x - last_xy[0], cur_y - last_xy[1]
                )
            last_xy = (cur_x, cur_y)
            # Maintain a rolling window of position samples for no-progress.
            pos_history.append((sim_ms, cur_x, cur_y))
            window_cutoff = sim_ms - int(RW_NO_PROGRESS_WINDOW_S * 1000)
            while pos_history and pos_history[0][0] < window_cutoff:
                pos_history.pop(0)

        # Mean abs wheel speed from position-sensor differentiation. This
        # answers the question "are the wheels actually turning?" without
        # the ambiguity of motor torque feedback.
        angles = []
        for s in sensors:
            try:
                angles.append(s.getValue() if s is not None else 0.0)
            except Exception:
                angles.append(0.0)
        if prev_angles is not None and dt_s > 0:
            instant = sum(
                abs(a - p) for a, p in zip(angles, prev_angles)
            ) / max(len(angles), 1) / dt_s
            wheel_speed_filt = (1.0 - alpha) * wheel_speed_filt + alpha * instant
        prev_angles = angles

        # Torque feedback retained for trace-only visibility; not used for
        # decisions any more (see RW_STUCK_* constants for the new signal).
        try:
            torques = [abs(m.getTorqueFeedback()) for m in motors]
        except Exception:
            torques = [0.0] * len(motors)
        peak_torque = max(torques) if torques else 0.0

        if state == "CRUISE":
            if sim_ms >= hold_until_ms:
                target_forward = cruise_speed * rng.uniform(0.85, 1.0)
                target_spin = rng.uniform(-0.35, 0.35) * MAX_WHEEL_SPEED
                hold_until_ms = sim_ms + int(rng.uniform(
                    RW_MIN_HOLD_S, RW_MAX_HOLD_S
                ) * 1000)

            # Stuck detection: two signals OR'd together so either one trips.
            commanded_speed = abs(current_forward) + abs(current_spin)
            wheel_stuck = (
                commanded_speed > RW_STUCK_COMMANDED_RAD_S
                and wheel_speed_filt < RW_STUCK_WHEEL_SPEED_RAD_S
            )
            # Position-based no-progress: window must be full (>= 90% of the
            # configured duration) before evaluating, otherwise we'd false-trip
            # at startup before the first window has elapsed.
            position_stuck = False
            if (cur_x is not None and pos_history and
                    commanded_speed > RW_STUCK_COMMANDED_RAD_S):
                window_age_ms = sim_ms - pos_history[0][0]
                if window_age_ms >= int(0.9 * RW_NO_PROGRESS_WINDOW_S * 1000):
                    x0, y0 = pos_history[0][1], pos_history[0][2]
                    if math.hypot(cur_x - x0, cur_y - y0) < RW_NO_PROGRESS_DISTANCE_M:
                        position_stuck = True
            stuck = wheel_stuck or position_stuck
            if stuck:
                collision_pegged_ms += time_step
                if collision_pegged_ms >= RW_COLLISION_DEBOUNCE_MS:
                    state = "BACKUP"
                    state_until_ms = sim_ms + int(RW_RECOVERY_BACKUP_S * 1000)
                    target_forward = -RW_RECOVERY_BACKUP_FRAC * MAX_WHEEL_SPEED
                    target_spin = 0.0
                    # Skip cruise-ramp: hard-step backwards at full power.
                    current_forward = target_forward
                    current_spin = target_spin
                    collision_pegged_ms = 0
            else:
                collision_pegged_ms = 0

        elif state == "BACKUP":
            if sim_ms >= state_until_ms:
                state = "TURN"
                state_until_ms = sim_ms + int(RW_RECOVERY_TURN_S * 1000)
                target_forward = 0.0
                target_spin = (
                    rng.choice((-1.0, 1.0))
                    * RW_RECOVERY_TURN_FRAC
                    * MAX_WHEEL_SPEED
                )
                # Same trick: jump straight to a pure pivot, instead of arcing
                # backward for 200 ms while spin ramps up against forward bleed-off.
                current_forward = target_forward
                current_spin = target_spin

        elif state == "TURN":
            if sim_ms >= state_until_ms:
                state = "CRUISE"
                hold_until_ms = 0  # force immediate fresh cruise target

        ramp = RW_RAMP * MAX_WHEEL_SPEED
        current_forward += clamp(
            target_forward - current_forward, -ramp, ramp
        )
        current_spin += clamp(target_spin - current_spin, -ramp, ramp)
        left_speed = clamp(
            current_forward - current_spin,
            -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED,
        )
        right_speed = clamp(
            current_forward + current_spin,
            -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED,
        )
        for m in left_motors:
            m.setVelocity(left_speed)
        for m in right_motors:
            m.setVelocity(right_speed)

        if trace_file is not None and sim_ms >= next_trace_ms:
            pos_str = (
                f"pos=({cur_x:.2f},{cur_y:.2f}) yaw={cur_yaw:+.2f} "
                if cur_x is not None
                else ""
            )
            dist_str = (
                f"total_dist={cumulative_distance:.1f} "
                if cur_x is not None
                else ""
            )
            trace_file.write(
                f"t_ms={sim_ms} state={state} "
                f"{pos_str}{dist_str}"
                f"cmd_fwd={current_forward:.2f} cmd_spin={current_spin:.2f} "
                f"wheel_left={left_speed:+.2f} wheel_right={right_speed:+.2f} "
                f"wheel_speed={wheel_speed_filt:.2f} "
                f"torque_peak={peak_torque:.1f}\n"
            )
            next_trace_ms = sim_ms + _TRACE_INTERVAL_MS


def main() -> None:
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    motor_names = (
        "front_left_wheel_motor",
        "rear_left_wheel_motor",
        "front_right_wheel_motor",
        "rear_right_wheel_motor",
    )
    motors = [robot.getDevice(mname) for mname in motor_names]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        try:
            m.enableTorqueFeedback(time_step)
        except Exception:
            pass
    # Position sensors give us actual wheel angular position; differentiating
    # over time is the most reliable "are the wheels actually turning?" signal,
    # and that's what the random-walk fallback uses to detect collisions.
    sensors = []
    for m in motors:
        s = None
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(time_step)
        except Exception:
            s = None
        sensors.append(s)
    left_motors = motors[:2]
    right_motors = motors[2:]

    # RNG seeded by the husky's name so the fleet doesn't move in
    # lockstep but each husky's stream is reproducible.
    seed = abs(hash(robot.getName())) % (1 << 32)
    rng = random.Random(seed)

    # Pull supervisor handle + payload. Either failing means we run
    # the random-walk fallback.
    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None
    craters, world_centre, world_size, boundary_margin = _parse_payload(robot)

    # Route to goal-seeker only when a payload (craters list) is present.
    # The warehouse Husky is a supervisor but has no payload — it should run
    # random-walk with position-based stuck detection, not goal-seeker, since
    # goal-seeker would aim at world (40,40) on a 30x18 m warehouse map.
    if self_node is not None and craters:
        _run_goal_seeker(
            robot, time_step, motors, left_motors, right_motors,
            self_node, craters, world_centre, world_size,
            boundary_margin, rng,
        )
    else:
        _run_random_walk(
            robot, time_step, motors, left_motors, right_motors,
            sensors, self_node, rng,
        )


if __name__ == "__main__":
    main()
