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

"""OmniBot random walk controller.

Picks a new (forward, spin) target every few seconds and ramps towards
it, so the robot drifts around the arena without human input. Used by
the two-robot demo world projects/samples/demos/worlds/starter/two_omnibots.wbt.
"""

import random

from omnisim import Robot

MAX_SPEED = 6.28
RAMP = 0.15
MIN_HOLD_S = 1.5
MAX_HOLD_S = 4.0


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice("left_wheel_joint_motor")
    right_motor = robot.getDevice("right_wheel_joint_motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    rng = random.Random(robot.getName())

    target_forward = 0.0
    target_spin = 0.0
    current_forward = 0.0
    current_spin = 0.0
    hold_until_ms = 0
    sim_ms = 0

    while robot.step(time_step) != -1:
        sim_ms += time_step
        if sim_ms >= hold_until_ms:
            target_forward = rng.uniform(-0.6, 1.0) * MAX_SPEED
            target_spin = rng.uniform(-1.0, 1.0) * MAX_SPEED * 0.5
            hold_until_ms = sim_ms + int(rng.uniform(MIN_HOLD_S, MAX_HOLD_S) * 1000)

        current_forward += clamp(target_forward - current_forward, -RAMP * MAX_SPEED, RAMP * MAX_SPEED)
        current_spin += clamp(target_spin - current_spin, -RAMP * MAX_SPEED, RAMP * MAX_SPEED)

        left_speed = clamp(current_forward - current_spin, -MAX_SPEED, MAX_SPEED)
        right_speed = clamp(current_forward + current_spin, -MAX_SPEED, MAX_SPEED)

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    main()
