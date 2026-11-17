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

"""TB3 random-walk driver.

Minimal differential-drive random walk for a 2-wheel TurtleBot3
(burger, waffle, or waffle_pi) loaded from URDF. Picks a random
forward + spin command every few seconds and drives the two
wheel motors via skid-steer math. Reactively backs up + spins
when wheels stall against an obstacle.

Companion to husky_random.py but trimmed to 2 motors (no fleet
goal-seeker, no crater payload). All TB3 variants expose the same
wheel-joint names from the URDF importer:
  wheel_left_joint_motor / wheel_right_joint_motor

Writes a per-robot trace to C:\\tmp\\husky_trace\\<name>.log every
2 seconds when that directory exists, matching husky_random's format
(state, position, total_dist, command). Used by the demo harness to
verify the robot actually drove a non-trivial distance.
"""

from __future__ import annotations

import math
import os
import random
import sys

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


MAX_WHEEL_SPEED = 6.0
CRUISE_FRAC = 0.5            # cruise speed = this * MAX_WHEEL_SPEED
HOLD_MIN_S = 2.0             # min time before picking a new (forward, spin)
HOLD_MAX_S = 5.0
RAMP_FRAC = 0.10             # fraction of MAX per step that velocities ramp

# Stuck detection -- wheel-speed signal.
STUCK_CMD_MIN_RAD_S = 1.5    # only trip when commanding meaningful motion
STUCK_WHEEL_MAX_RAD_S = 0.8  # wheels delivering less than this = obstacle
STUCK_DEBOUNCE_MS = 200
WHEEL_FILT_TAU_S = 0.12

# Recovery.
BACKUP_S = 1.2
BACKUP_FRAC = 0.6
TURN_S = 2.4
TURN_FRAC = 1.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _open_trace(name: str):
    safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in name)
    for d in (r"C:\tmp\husky_trace", "/tmp/husky_trace"):
        try:
            os.makedirs(d, exist_ok=True)
            return open(os.path.join(d, f"{safe}.log"), "w", encoding="utf-8", buffering=1)
        except Exception:
            continue
    return None


def main() -> int:
    sys.stderr.write("[tb3_random] starting\n")
    sys.stderr.flush()
    try:
        robot = _Robot()
    except Exception as exc:
        sys.stderr.write(f"[tb3_random] _Robot() failed: {exc}\n")
        from omnisim import Robot
        robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    dt_s = time_step / 1000.0
    sys.stderr.write(f"[tb3_random] {robot.getName()} time_step={time_step}\n")
    sys.stderr.flush()

    left = robot.getDevice("wheel_left_joint_motor")
    right = robot.getDevice("wheel_right_joint_motor")
    if left is None or right is None:
        sys.stderr.write(
            "[tb3_random] missing wheel_left_joint_motor/wheel_right_joint_motor; "
            "is this a TB3 URDFRobot?\n"
        )
        return 1
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    # Position-sensor readback -- the URDF importer pairs each motor with
    # a PositionSensor of the same joint name; use it to differentiate
    # commanded vs. actual wheel speed.
    sensors = []
    for m in (left, right):
        s = None
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(time_step)
        except Exception:
            s = None
        sensors.append(s)

    rng = random.Random(abs(hash(robot.getName())) % (1 << 32))

    # Supervisor handle for chassis position (only when world declares
    # supervisor TRUE; otherwise we still drive, just no position trace).
    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None
    trace = _open_trace(robot.getName())
    next_trace_ms = 0
    initial_xy = None
    last_xy = None
    cumulative_dist = 0.0

    state = "CRUISE"
    state_until_ms = 0
    sim_ms = 0
    target_fwd = 0.0
    target_spin = 0.0
    cur_fwd = 0.0
    cur_spin = 0.0
    hold_until_ms = 0
    prev_angles = None
    wheel_filt = 0.0
    stuck_ms = 0
    alpha = dt_s / max(WHEEL_FILT_TAU_S, dt_s)

    while robot.step(time_step) != -1:
        sim_ms += time_step

        # Sample wheel angles for stuck detection.
        angles = []
        for s in sensors:
            try:
                angles.append(s.getValue() if s is not None else 0.0)
            except Exception:
                angles.append(0.0)
        if prev_angles is not None and dt_s > 0:
            instant = sum(abs(a - p) for a, p in zip(angles, prev_angles)) / 2.0 / dt_s
            wheel_filt = (1 - alpha) * wheel_filt + alpha * instant
        prev_angles = angles

        if state == "CRUISE":
            if sim_ms >= hold_until_ms:
                target_fwd = CRUISE_FRAC * MAX_WHEEL_SPEED * rng.uniform(0.85, 1.0)
                target_spin = rng.uniform(-0.35, 0.35) * MAX_WHEEL_SPEED
                hold_until_ms = sim_ms + int(rng.uniform(HOLD_MIN_S, HOLD_MAX_S) * 1000)
            cmd = abs(cur_fwd) + abs(cur_spin)
            if cmd > STUCK_CMD_MIN_RAD_S and wheel_filt < STUCK_WHEEL_MAX_RAD_S:
                stuck_ms += time_step
                if stuck_ms >= STUCK_DEBOUNCE_MS:
                    state = "BACKUP"
                    state_until_ms = sim_ms + int(BACKUP_S * 1000)
                    target_fwd = -BACKUP_FRAC * MAX_WHEEL_SPEED
                    target_spin = 0.0
                    cur_fwd, cur_spin = target_fwd, target_spin
                    stuck_ms = 0
            else:
                stuck_ms = 0
        elif state == "BACKUP":
            if sim_ms >= state_until_ms:
                state = "TURN"
                state_until_ms = sim_ms + int(TURN_S * 1000)
                target_fwd = 0.0
                target_spin = rng.choice((-1.0, 1.0)) * TURN_FRAC * MAX_WHEEL_SPEED
                cur_fwd, cur_spin = target_fwd, target_spin
        elif state == "TURN":
            if sim_ms >= state_until_ms:
                state = "CRUISE"
                hold_until_ms = 0

        ramp = RAMP_FRAC * MAX_WHEEL_SPEED
        cur_fwd += clamp(target_fwd - cur_fwd, -ramp, ramp)
        cur_spin += clamp(target_spin - cur_spin, -ramp, ramp)
        l_speed = clamp(cur_fwd - cur_spin, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        r_speed = clamp(cur_fwd + cur_spin, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        left.setVelocity(l_speed)
        right.setVelocity(r_speed)

        # Track motion + write trace.
        cur_z = None
        if self_node is not None:
            try:
                pos = self_node.getPosition()
                if pos is not None and len(pos) >= 3:
                    cur_xy = (float(pos[0]), float(pos[1]))
                    cur_z = float(pos[2])
                    if initial_xy is None:
                        initial_xy = cur_xy
                    if last_xy is not None:
                        cumulative_dist += math.hypot(cur_xy[0] - last_xy[0], cur_xy[1] - last_xy[1])
                    last_xy = cur_xy
            except Exception:
                pass
        if trace is not None and sim_ms >= next_trace_ms:
            xy_str = ""
            if last_xy is not None:
                z_str = f"{cur_z:+.3f}" if cur_z is not None else "?"
                xy_str = f"pos=({last_xy[0]:+.2f},{last_xy[1]:+.2f},{z_str}) "
            trace.write(
                f"t_ms={sim_ms} state={state} {xy_str}"
                f"total_dist={cumulative_dist:.2f} "
                f"cmd_fwd={cur_fwd:+.2f} cmd_spin={cur_spin:+.2f} "
                f"wheel_l={l_speed:+.2f} wheel_r={r_speed:+.2f} "
                f"wheel_meas={wheel_filt:.2f}\n"
            )
            next_trace_ms = sim_ms + 2000

    return 0


if __name__ == "__main__":
    sys.exit(main())
