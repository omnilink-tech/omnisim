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
"""Drive one Husky toward the target encoded in ``customData``.

The paired agent-fleet demo gives four independent controllers either one
shared target (the failing plan) or four separated targets (the replanned
solution).  The controller deliberately stays small: OmniSim's contact state,
not a hidden coordinator, decides whether the plan is safe.
"""

from __future__ import annotations

import json
import math

from omnisim import Supervisor


MAX_SPEED = 6.0
TURN_GAIN = 3.2
DEFAULT_STOP_RADIUS = 0.28


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_orientation(orientation) -> float:
    return math.atan2(orientation[3], orientation[0])


def main() -> None:
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())
    payload = json.loads(robot.getCustomData() or "{}")
    target = payload.get("target", [0.0, 0.0])
    target_x, target_y = float(target[0]), float(target[1])
    stop_radius = float(payload.get("stop_radius", DEFAULT_STOP_RADIUS))

    motor_names = (
        "front_left_wheel_motor",
        "rear_left_wheel_motor",
        "front_right_wheel_motor",
        "rear_right_wheel_motor",
    )
    motors = [robot.getDevice(name) for name in motor_names]
    left_motors = motors[:2]
    right_motors = motors[2:]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    self_node = robot.getSelf()
    announced = False
    while robot.step(time_step) != -1:
        position = self_node.getPosition()
        orientation = self_node.getOrientation()
        dx = target_x - float(position[0])
        dy = target_y - float(position[1])
        distance = math.hypot(dx, dy)

        if distance <= stop_radius:
            for motor in motors:
                motor.setVelocity(0.0)
            if not announced:
                print(
                    f"[fleet_goal] {robot.getName()} reached "
                    f"({target_x:.2f}, {target_y:.2f})"
                )
                announced = True
            continue

        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(desired_yaw - yaw_from_orientation(orientation))
        alignment = max(0.0, math.cos(yaw_error))
        forward = MAX_SPEED * (0.18 + 0.82 * alignment)
        turn = clamp(TURN_GAIN * yaw_error, -MAX_SPEED, MAX_SPEED)
        left_speed = clamp(forward - turn, -MAX_SPEED, MAX_SPEED)
        right_speed = clamp(forward + turn, -MAX_SPEED, MAX_SPEED)
        for motor in left_motors:
            motor.setVelocity(left_speed)
        for motor in right_motors:
            motor.setVelocity(right_speed)


if __name__ == "__main__":
    main()
