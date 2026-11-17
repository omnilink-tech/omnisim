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

"""drive_forward — minimal controller that runs all four wheels at a
fixed forward velocity, with per-wheel scaling read from customData.

Used by the Phase 5 driving variant of the damage arena demo. The
damage tracker (in harness_supervisor) writes the per-wheel torque
scale and a game_over flag into this robot's customData when wheel or
chassis state changes; we read it each step and apply.

customData schema (written by damage_tracker._write_behavior_gate):
    {"wheel_torque": {"fl": 1.0, "fr": 1.0, "rl": 1.0, "rr": 1.0},
     "game_over": false}

Missing/invalid customData -> all wheels at full scale (pristine).
game_over=true -> all wheels stop regardless of individual scales.
"""

from __future__ import annotations

import json
import sys

from omnisim import Robot


BASE_VELOCITY_RAD_S = 2.5  # Husky wheel commanded speed when undamaged
                            # ~0.4 m/s linear — slow enough to see the impacts

# URDF joint names for the four wheels — these are the device names
# Webots exposes via getDevice() on the URDFRobot.
WHEEL_NAMES = {
    "fl": "front_left_wheel_motor",
    "fr": "front_right_wheel_motor",
    "rl": "rear_left_wheel_motor",
    "rr": "rear_right_wheel_motor",
}


def parse_gate(raw: str) -> tuple[dict[str, float], bool]:
    """Returns (per-wheel scales, game_over flag) read from the
    supervisor-written customData. base_velocity_rad_s is no longer
    read from here — Webots' VRML SFString parser truncates strings
    containing escaped quotes (`\\"`) at the second character, which
    drops keys placed before the supervisor's wheel_torque write.
    Speed override comes from controllerArgs[0] now (see main()).
    """
    if not raw:
        return ({k: 1.0 for k in WHEEL_NAMES}, False)
    try:
        d = json.loads(raw)
    except json.JSONDecodeError:
        return ({k: 1.0 for k in WHEEL_NAMES}, False)
    wt = d.get("wheel_torque") or {}
    scales = {k: float(wt.get(k, 1.0)) for k in WHEEL_NAMES}
    return (scales, bool(d.get("game_over", False)))


def main() -> int:
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # Speed override: first controllerArgs entry, falls back to the
    # module default. Worlds set this via `controllerArgs ["50"]` on
    # the URDFRobot. Bypasses customData JSON because Webots' SFString
    # parser truncates strings with `\"` escapes at the second char.
    base_velocity_rad_s = BASE_VELOCITY_RAD_S
    if len(sys.argv) >= 2:
        try:
            base_velocity_rad_s = float(sys.argv[1])
        except ValueError:
            sys.stderr.write(
                f"[drive_forward] bad controllerArgs[0]={sys.argv[1]!r}; "
                f"using default {BASE_VELOCITY_RAD_S}\n"
            )

    motors: dict[str, object] = {}
    for short, name in WHEEL_NAMES.items():
        m = robot.getDevice(name)
        if m is None:
            sys.stderr.write(f"[drive_forward] missing motor {name!r}; aborting\n")
            return 1
        try:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[drive_forward] motor {name} setup failed: {exc}\n")
            return 1
        motors[short] = m

    sys.stderr.write(
        f"[drive_forward] driving 4 wheels at {base_velocity_rad_s} rad/s; "
        f"reading torque scale from customData\n"
    )
    sys.stderr.flush()

    # P4 perf: setVelocity once, robot.step in a tight loop after.
    # The original implementation called getCustomData() + setVelocity()
    # on 4 motors *every* step -- that's 5 IPC calls per controller per
    # tick. With 10 controllers in newton_husky_swarm_drive that's 50
    # extra IPC ops/tick and the multi-husky world dropped to ~4 fps
    # because of it (Webots syncs across all controllers per tick).
    #
    # For pristine "drive forward" the velocity never changes, so write
    # it once. Damage-gate (game_over / per-wheel scaling) re-evaluates
    # only when the customData string we cached actually changed -- and
    # we read the customData only every Nth step.
    DAMAGE_POLL_EVERY = 30  # ~0.5 sec at 60 Hz; cheap and good enough
    _last_raw = ""
    _last_scales = {k: 1.0 for k in WHEEL_NAMES}
    _last_game_over = False

    # Set initial velocity (once per motor).
    for short, m in motors.items():
        try:
            m.setVelocity(base_velocity_rad_s * _last_scales.get(short, 1.0))
        except Exception:
            pass

    tick = 0
    while robot.step(time_step) != -1:
        tick += 1
        if tick % DAMAGE_POLL_EVERY != 0:
            continue
        raw = robot.getCustomData() or ""
        if raw == _last_raw:
            continue
        _last_raw = raw
        scales, game_over = parse_gate(raw)
        # Only re-issue setVelocity if the values actually changed --
        # one extra IPC per dirty motor, zero when nothing changed.
        if game_over and not _last_game_over:
            for m in motors.values():
                try:
                    m.setVelocity(0.0)
                except Exception:
                    pass
        elif _last_game_over and not game_over:
            for short, m in motors.items():
                try:
                    m.setVelocity(base_velocity_rad_s * scales.get(short, 1.0))
                except Exception:
                    pass
        else:
            for short, m in list(motors.items()):
                if scales.get(short, 1.0) != _last_scales.get(short, 1.0):
                    try:
                        m.setVelocity(base_velocity_rad_s * scales.get(short, 1.0))
                    except Exception:
                        motors.pop(short, None)
        _last_scales = scales
        _last_game_over = game_over

    return 0


if __name__ == "__main__":
    sys.exit(main())
