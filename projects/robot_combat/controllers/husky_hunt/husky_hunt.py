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

"""husky_hunt — robot combat AI for two Newton-backed Huskies.

State machine:

  SEEK     — opponent out of charge range OR not yet aligned. Drive at
             moderate speed toward target, skid-steer turn toward it.
  CHARGE   — within CHARGE_RANGE_M and aligned. Full forward, ram.
  REVERSE  — impact detected (own Δv > IMPACT_DV_THRESHOLD). Back up
             for REVERSE_S seconds to create space for another run.
  PIVOT    — reverse done. Spin in place until aligned again.
  VICTORY  — opponent hasn't moved meaningfully for VICTORY_STILL_S
             seconds (wheels broken off — can't move). Spin in place;
             the match is over.

controllerArgs:
  [0] target robot's `name` field (matched via root-children walk;
      the URDFRobot nodes in the head-on world don't have DEF names).
      Defaults to "husky_b".

Per-controller debug log goes to husky_hunt_<self_name>.log
so the agent driving the iteration loop can see state transitions
without polling the GUI.
"""

from __future__ import annotations

import json
import math
import sys
import time

try:
    from omnisim import Supervisor as _RobotImpl
except Exception:
    from omnisim import Robot as _RobotImpl


WHEEL_NAMES = (
    "front_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_left_wheel_motor",
    "rear_right_wheel_motor",
)

# ---------------------------------------------------------------------------
# Combat tuning
# ---------------------------------------------------------------------------

# Wheel speed bounds (rad/s). Husky wheel radius ~0.165 m, so 12 rad/s
# ~ 2.0 m/s linear -- intentionally well under the 8 m/s head-on speeds
# that overflowed ODE's contact-joint table and crashed OmniSim.
SEEK_SPEED = 7.0
CHARGE_SPEED = 12.0
REVERSE_SPEED = 9.0
PIVOT_SPEED = 6.0  # one side forward, one side back

# Steering.
SPIN_GAIN = 5.0       # rad/s wheel-diff per rad of yaw error during SEEK
ALIGN_TIGHT = 0.20    # rad — under this, drive full speed
ALIGN_LOOSE = 0.45    # rad — used when leaving PIVOT
MIN_FORWARD_FRAC = 0.35  # never fully stop while seeking

# Range to enter CHARGE.
CHARGE_RANGE_M = 6.0
# Hysteresis: drop back to SEEK if we drift further than this.
DISENGAGE_RANGE_M = 9.0

# Impact detection: hold previous self-velocity, watch xy speed delta.
# 16ms basicTimeStep -> 1.0 m/s Δv per step ~ 6 g, plenty for the
# head-on contacts we get at CHARGE_SPEED. Was 2.0 — too high; the
# robots just stalemated pressing against each other without ever
# triggering REVERSE.
IMPACT_DV_THRESHOLD = 1.0

# Stalemate: if we're close, ordering forward, but own speed has been
# tiny for STALEMATE_S, we're pressed against the target and should
# back up to make a new run.
STALEMATE_RANGE_M = 2.2
STALEMATE_SPEED_M_S = 0.25
STALEMATE_S = 1.0

# Reverse phase.
REVERSE_S = 1.4
# After reverse, also pivot briefly so we don't just charge back into
# the same exact spot.
PIVOT_S_MAX = 1.5

# Victory: opponent position barely changes over this window AND its
# current linear velocity is tiny AND we're far enough away that we're
# not the one holding them in place. Requiring all three avoids
# declaring victory on a robot that's just briefly stuck against a
# wall or paused mid-pivot.
VICTORY_WINDOW_S = 3.0
VICTORY_MAX_DISPLACEMENT_M = 0.3
VICTORY_MAX_SPEED_M_S = 0.15
# When both robots end up immobile and close (both lost wheels, both
# settled where they ground to a halt), we still want to call the
# match. Drop the min-range guard from 3m to 1m: if I'm 1m away and
# target hasn't moved 0.3m in 3s with own speed near zero, target is
# disabled regardless of who's nearby.
VICTORY_MIN_RANGE_M = 1.0

# Grace period before checking victory (give the match a chance to
# develop and avoid declaring victory on a robot that hasn't moved yet
# because its controller's startup race is still resolving).
VICTORY_GRACE_S = 8.0


def _yaw_from_orientation(orient) -> float:
    """Flattened 3x3 row-major. Yaw = atan2(R10, R00) = atan2(o[3], o[0])."""
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _xy_mag(v) -> float:
    return math.hypot(v[0], v[1])


def _find_target_by_name(supervisor, target_name: str):
    """getFromDef requires a DEF; the world's URDFRobot nodes don't
    have one, so scan children by `name` field instead."""
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        n = children.getCount()
    except Exception as exc:
        sys.stderr.write(f"[husky_hunt] root walk failed: {exc}\n")
        return None
    for i in range(n):
        try:
            node = children.getMFNode(i)
            name_field = node.getField("name")
            if name_field is None:
                continue
            if name_field.getSFString() == target_name:
                return node
        except Exception:
            continue
    return None


def main() -> int:
    robot = _RobotImpl()
    time_step = int(robot.getBasicTimeStep())
    dt = time_step / 1000.0

    target_name = sys.argv[1] if len(sys.argv) >= 2 else "husky_b"

    # Try to derive own name for the per-controller log path. Falls
    # back to a generic name if Supervisor.getSelf isn't available.
    self_name = "self"
    try:
        s = robot.getSelf()
        if s is not None:
            nf = s.getField("name")
            if nf is not None:
                self_name = nf.getSFString() or self_name
    except Exception:
        pass

    log_path = f"husky_hunt_{self_name}.log"
    try:
        log_f = open(log_path, "w", buffering=1, encoding="utf-8")
    except OSError:
        log_f = None

    def log(msg: str) -> None:
        line = f"[{self_name}->{target_name}] {msg}"
        sys.stderr.write(line + "\n")
        if log_f is not None:
            log_f.write(line + "\n")

    log(f"start; time_step={time_step}ms")

    motors = {}
    for name in WHEEL_NAMES:
        try:
            m = robot.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            motors[name] = m
        except Exception as exc:
            log(f"motor {name!r} setup failed: {exc}")

    def set_wheels(left: float, right: float) -> None:
        cap = CHARGE_SPEED
        left = max(-cap, min(cap, left))
        right = max(-cap, min(cap, right))
        try:
            motors["front_left_wheel_motor"].setVelocity(left)
            motors["rear_left_wheel_motor"].setVelocity(left)
            motors["front_right_wheel_motor"].setVelocity(right)
            motors["rear_right_wheel_motor"].setVelocity(right)
        except Exception:
            pass

    if not motors:
        log("no motors; idling")
        while robot.step(time_step) != -1:
            pass
        return 0

    # Read own customData ("game_over": true when chassis is broken).
    # If the supervisor has flagged us disabled, we stop driving and
    # let the other robot's victory check fire.
    def own_disabled() -> bool:
        try:
            raw = robot.getCustomData() or ""
        except Exception:
            return False
        if not raw:
            return False
        try:
            return bool(json.loads(raw).get("game_over", False))
        except Exception:
            return False

    target_node = None
    state = "SEEK"
    state_entered_t = 0.0  # sim seconds
    sim_t = 0.0
    disabled_logged = False

    prev_self_xy_speed = 0.0
    # Target-position history for victory detection.
    tgt_history: list[tuple[float, float, float]] = []  # (sim_t, x, y)
    # Time at which we first observed own speed < STALEMATE_SPEED_M_S
    # while in range; resets when speed climbs back up or range opens.
    stalemate_started_t: float | None = None
    start_t = 0.0

    while robot.step(time_step) != -1:
        sim_t += dt

        # Chassis broken = disabled. Halt and let the other side win.
        if own_disabled():
            if not disabled_logged:
                log(f"DISABLED at t={sim_t:.2f}s (game_over=True)")
                disabled_logged = True
            set_wheels(0.0, 0.0)
            continue

        if target_node is None:
            target_node = _find_target_by_name(robot, target_name)
            if target_node is None:
                set_wheels(0.0, 0.0)
                continue
            log(f"locked onto target '{target_name}' at t={sim_t:.2f}s")
            start_t = sim_t

        # Read poses; if anything fails, halt this tick (target may
        # have been removed during a damage detach pass).
        try:
            my_self = robot.getSelf()
            my_pos = my_self.getPosition()
            my_orient = my_self.getOrientation()
            my_vel = my_self.getVelocity()
            tgt_pos = target_node.getPosition()
        except Exception:
            set_wheels(0.0, 0.0)
            continue

        # Maintain target history for victory check.
        tgt_history.append((sim_t, tgt_pos[0], tgt_pos[1]))
        # Drop entries older than the window.
        cutoff = sim_t - VICTORY_WINDOW_S
        while tgt_history and tgt_history[0][0] < cutoff:
            tgt_history.pop(0)

        # Geometry to target.
        dx = tgt_pos[0] - my_pos[0]
        dy = tgt_pos[1] - my_pos[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        yaw = _yaw_from_orientation(my_orient)
        yaw_err = _wrap_pi(bearing - yaw)

        # Impact detection: my own xy speed Δ from last step.
        cur_speed = _xy_mag(my_vel)
        dv = abs(cur_speed - prev_self_xy_speed)
        prev_self_xy_speed = cur_speed

        # Victory check: target's position barely changed over window
        # AND its current linear speed is tiny AND we're far enough
        # away that we're not the one pinning them. Only after grace.
        if (sim_t - start_t) > VICTORY_GRACE_S and state != "VICTORY":
            if dist > VICTORY_MIN_RANGE_M and len(tgt_history) >= 2:
                xs = [p[1] for p in tgt_history]
                ys = [p[2] for p in tgt_history]
                spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                tgt_speed = 0.0
                try:
                    tgt_speed = _xy_mag(target_node.getVelocity())
                except Exception:
                    pass
                window_full = (sim_t - tgt_history[0][0]) >= VICTORY_WINDOW_S * 0.9
                if (window_full
                        and spread < VICTORY_MAX_DISPLACEMENT_M
                        and tgt_speed < VICTORY_MAX_SPEED_M_S):
                    log(f"VICTORY at t={sim_t:.2f}s (target spread {spread:.2f}m, "
                        f"tgt_speed={tgt_speed:.2f}m/s, dist={dist:.2f}m)")
                    state = "VICTORY"
                    state_entered_t = sim_t

        # Stalemate detection: in range + own speed tiny for STALEMATE_S
        # = we're pressed nose-to-nose. Back off to break the deadlock.
        in_close_range = dist < STALEMATE_RANGE_M
        is_slow = cur_speed < STALEMATE_SPEED_M_S
        if in_close_range and is_slow and state in ("SEEK", "CHARGE"):
            if stalemate_started_t is None:
                stalemate_started_t = sim_t
            elif (sim_t - stalemate_started_t) >= STALEMATE_S:
                log(f"STALEMATE at t={sim_t:.2f}s (dist={dist:.2f}m, "
                    f"speed={cur_speed:.2f}m/s) -> REVERSE")
                state = "REVERSE"
                state_entered_t = sim_t
                stalemate_started_t = None
        else:
            stalemate_started_t = None

        # State transitions.
        prev_state = state
        if state == "SEEK":
            if dist < CHARGE_RANGE_M and abs(yaw_err) < ALIGN_TIGHT:
                state = "CHARGE"
                state_entered_t = sim_t
        elif state == "CHARGE":
            if dv > IMPACT_DV_THRESHOLD:
                state = "REVERSE"
                state_entered_t = sim_t
                log(f"IMPACT at t={sim_t:.2f}s (dv={dv:.2f} m/s, "
                    f"dist={dist:.2f}m)")
            elif dist > DISENGAGE_RANGE_M:
                state = "SEEK"
                state_entered_t = sim_t
        elif state == "REVERSE":
            if (sim_t - state_entered_t) >= REVERSE_S:
                state = "PIVOT"
                state_entered_t = sim_t
        elif state == "PIVOT":
            if abs(yaw_err) < ALIGN_LOOSE or (sim_t - state_entered_t) >= PIVOT_S_MAX:
                state = "SEEK"
                state_entered_t = sim_t

        if state != prev_state:
            log(f"state {prev_state} -> {state} at t={sim_t:.2f}s "
                f"(dist={dist:.2f}m, yaw_err={math.degrees(yaw_err):.0f}deg)")

        # Drive.
        if state == "VICTORY":
            # Slow celebratory spin.
            set_wheels(-PIVOT_SPEED * 0.5, PIVOT_SPEED * 0.5)
        elif state == "SEEK":
            spin = SPIN_GAIN * yaw_err
            forward = SEEK_SPEED if abs(yaw_err) < ALIGN_TIGHT else \
                max(MIN_FORWARD_FRAC * SEEK_SPEED,
                    SEEK_SPEED * (1.0 - (abs(yaw_err) - ALIGN_TIGHT) / math.pi))
            set_wheels(forward - spin, forward + spin)
        elif state == "CHARGE":
            spin = SPIN_GAIN * yaw_err
            set_wheels(CHARGE_SPEED - spin, CHARGE_SPEED + spin)
        elif state == "REVERSE":
            set_wheels(-REVERSE_SPEED, -REVERSE_SPEED)
        elif state == "PIVOT":
            # Spin in place toward target.
            sign = 1.0 if yaw_err > 0 else -1.0
            set_wheels(-sign * PIVOT_SPEED, sign * PIVOT_SPEED)

    return 0


if __name__ == "__main__":
    sys.exit(main())
