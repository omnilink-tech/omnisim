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

"""queen_walker — random-walk controller for the ORC team queen.

The queen is a non-combatant target: she wanders the arena with the
same CRUISE/BACKUP/TURN state machine used by the Husky random walker
(`projects/default/controllers/husky_random/husky_random.py`), with two
ORC-specific extensions:

  - Arena soft fence. customData declares ``arena_size`` (square side
    in metres); when the queen approaches within ``ARENA_MARGIN_M`` of
    the half-extent, she steers back toward the centre so she doesn't
    drive OOTA and lose the match by accident.

  - Match-over awareness. Reads the DEF DIRECTOR supervisor's
    customData each step. When ``match_over`` flips true or she
    herself is listed under ``eliminated``, the wheels are stopped.

She has the same 4WD layout + wheel motor names as the BattleBot
fighters, so the damage director registers her like any other fighter
(chassis + 4 wheels; no weapon body).
"""

from __future__ import annotations

import json
import math
import random
import sys

try:
    from omnisim import Supervisor as _RobotImpl
except Exception:
    from omnisim import Robot as _RobotImpl


WHEEL_MOTOR_NAMES = (
    "front_left_wheel_motor",
    "rear_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_right_wheel_motor",
)

MAX_WHEEL_SPEED = 6.0
CRUISE_FRAC = 0.55
RAMP = 0.08

RW_MIN_HOLD_S = 2.5
RW_MAX_HOLD_S = 5.5

RW_STUCK_COMMANDED = 2.0
RW_STUCK_WHEEL_SPEED = 1.2
RW_COLLISION_DEBOUNCE_MS = 150
RW_WHEEL_SPEED_TAU = 0.12

RW_NO_PROGRESS_WINDOW_S = 4.0
RW_NO_PROGRESS_DISTANCE_M = 0.5
RW_RECOVERY_BACKUP_S = 1.5
RW_RECOVERY_TURN_S = 2.8
RW_RECOVERY_BACKUP_FRAC = 0.7
RW_RECOVERY_TURN_FRAC = 1.0

# Arena keepout: when within this distance of any arena edge, override
# the random target with a vector pointing back toward the centre so the
# queen doesn't OOTA.
ARENA_MARGIN_M = 3.0

DIRECTOR_NAMES = ("damage_director", "match_director")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _yaw_from_orientation(orient) -> float:
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _scan_root_nodes(robot):
    out = []
    try:
        root = robot.getRoot()
        children = root.getField("children")
        n = children.getCount()
    except Exception:
        return out
    for i in range(n):
        try:
            node = children.getMFNode(i)
            nm_field = node.getField("name")
            nm = nm_field.getSFString() if nm_field is not None else ""
            out.append((nm, node))
        except Exception:
            continue
    return out


def main() -> int:
    robot = _RobotImpl()
    time_step = int(robot.getBasicTimeStep())
    dt_s = time_step / 1000.0

    # Self introspection.
    self_name = "queen"
    try:
        s = robot.getSelf()
        if s is not None:
            nf = s.getField("name")
            if nf is not None:
                self_name = nf.getSFString() or self_name
    except Exception:
        pass

    # Parse arena_size from own customData (set by the world per queen).
    arena_half = 10.0  # default 20 m arena half-extent
    try:
        raw = robot.getCustomData() or ""
        if raw.strip():
            data = json.loads(raw)
            if isinstance(data.get("arena_size"), (int, float)):
                arena_half = float(data["arena_size"]) * 0.5
    except Exception:
        pass

    # Motors + sensors.
    motors = []
    sensors = []
    for name in WHEEL_MOTOR_NAMES:
        try:
            m = robot.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            motors.append(m)
            try:
                s = m.getPositionSensor()
                if s is not None:
                    s.enable(time_step)
                sensors.append(s)
            except Exception:
                sensors.append(None)
        except Exception as exc:
            sys.stderr.write(f"[queen_walker:{self_name}] motor {name!r} setup "
                             f"failed: {exc}\n")
            motors.append(None)
            sensors.append(None)
    left_motors = motors[:2]
    right_motors = motors[2:]

    # Supervisor handle for position + boundary keepout.
    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    # Director handle (resolved lazily — director may load after us).
    director_node = None

    def director_state():
        nonlocal director_node
        if director_node is None:
            for nm, node in _scan_root_nodes(robot):
                if nm in DIRECTOR_NAMES:
                    director_node = node
                    break
        if director_node is None:
            return None
        try:
            raw = director_node.getField("customData").getSFString()
        except Exception:
            return None
        if not raw:
            return None
        try:
            return json.loads(raw)
        except Exception:
            return None

    rng = random.Random(abs(hash(self_name)) % (1 << 32))

    target_forward = 0.0
    target_spin = 0.0
    current_forward = 0.0
    current_spin = 0.0
    hold_until_ms = 0
    sim_ms = 0

    state = "CRUISE"
    state_until_ms = 0
    collision_pegged_ms = 0

    prev_angles = None
    wheel_speed_filt = 0.0
    alpha = dt_s / max(RW_WHEEL_SPEED_TAU, dt_s)

    pos_history: list[tuple[int, float, float]] = []

    cruise_speed = CRUISE_FRAC * MAX_WHEEL_SPEED
    eliminated_logged = False

    sys.stderr.write(
        f"[queen_walker:{self_name}] start arena_half={arena_half:.1f}m "
        f"time_step={time_step}ms\n"
    )

    while robot.step(time_step) != -1:
        sim_ms += time_step

        # Stop driving if the match is over or we're eliminated.
        st = director_state()
        if st is not None:
            if (st.get("match_over") is True
                    or self_name in st.get("eliminated", [])):
                if not eliminated_logged:
                    sys.stderr.write(
                        f"[queen_walker:{self_name}] match_over/eliminated -> "
                        f"halt\n")
                    eliminated_logged = True
                for m in motors:
                    if m is not None:
                        m.setVelocity(0.0)
                continue

        # Sample pose.
        cur_x = cur_y = None
        cur_yaw = 0.0
        if self_node is not None:
            try:
                pos = self_node.getPosition()
                if pos is not None and len(pos) >= 2:
                    cur_x = float(pos[0])
                    cur_y = float(pos[1])
                cur_yaw = _yaw_from_orientation(self_node.getOrientation())
            except Exception:
                pass
        if cur_x is not None:
            pos_history.append((sim_ms, cur_x, cur_y))
            cutoff = sim_ms - int(RW_NO_PROGRESS_WINDOW_S * 1000)
            while pos_history and pos_history[0][0] < cutoff:
                pos_history.pop(0)

        # Wheel-speed estimate via sensor differentiation.
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

        if state == "CRUISE":
            need_new_target = sim_ms >= hold_until_ms
            # Arena keepout: when near the edge, override target with
            # spin toward centre.
            near_edge = False
            if cur_x is not None:
                limit = arena_half - ARENA_MARGIN_M
                if abs(cur_x) > limit or abs(cur_y) > limit:
                    near_edge = True
                    # Vector pointing back to (0,0).
                    desired_yaw = math.atan2(-cur_y, -cur_x)
                    yaw_err = _wrap_pi(desired_yaw - cur_yaw)
                    target_forward = cruise_speed * 0.7
                    target_spin = clamp(
                        yaw_err * 3.0, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED
                    )
                    hold_until_ms = sim_ms + 600  # short hold; re-check
            if need_new_target and not near_edge:
                target_forward = cruise_speed * rng.uniform(0.85, 1.0)
                target_spin = rng.uniform(-0.35, 0.35) * MAX_WHEEL_SPEED
                hold_until_ms = sim_ms + int(
                    rng.uniform(RW_MIN_HOLD_S, RW_MAX_HOLD_S) * 1000
                )

            commanded = abs(current_forward) + abs(current_spin)
            wheel_stuck = (
                commanded > RW_STUCK_COMMANDED
                and wheel_speed_filt < RW_STUCK_WHEEL_SPEED
            )
            position_stuck = False
            if (cur_x is not None and pos_history
                    and commanded > RW_STUCK_COMMANDED):
                window_age_ms = sim_ms - pos_history[0][0]
                if window_age_ms >= int(0.9 * RW_NO_PROGRESS_WINDOW_S * 1000):
                    x0, y0 = pos_history[0][1], pos_history[0][2]
                    if math.hypot(cur_x - x0, cur_y - y0) < RW_NO_PROGRESS_DISTANCE_M:
                        position_stuck = True
            if wheel_stuck or position_stuck:
                collision_pegged_ms += time_step
                if collision_pegged_ms >= RW_COLLISION_DEBOUNCE_MS:
                    state = "BACKUP"
                    state_until_ms = sim_ms + int(RW_RECOVERY_BACKUP_S * 1000)
                    target_forward = -RW_RECOVERY_BACKUP_FRAC * MAX_WHEEL_SPEED
                    target_spin = 0.0
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
                current_forward = target_forward
                current_spin = target_spin

        elif state == "TURN":
            if sim_ms >= state_until_ms:
                state = "CRUISE"
                hold_until_ms = 0

        ramp = RAMP * MAX_WHEEL_SPEED
        current_forward += clamp(
            target_forward - current_forward, -ramp, ramp
        )
        current_spin += clamp(
            target_spin - current_spin, -ramp, ramp
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
            if m is not None:
                m.setVelocity(left_speed)
        for m in right_motors:
            if m is not None:
                m.setVelocity(right_speed)

    return 0


if __name__ == "__main__":
    sys.exit(main())
