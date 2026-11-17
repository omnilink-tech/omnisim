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

"""OmniArm 7 wave demo controller.

Drives the seven revolute joints of the OmniArm 7 cobot with a slow sinusoidal
pattern so the arm has visible motion in the sample world without any
external input. Same recipe as the in-tree OmniArm 6 `omniarm6_wave` controller:
each joint gets its own frequency, phase, and a small amplitude around a
gently-bent "home" pose so the 1.9 m arm never swings itself into the
pedestal or its own joint limits.

When the robot name ends in an integer suffix (e.g. "omniarm7_3" in a fleet
world), that integer becomes a global phase offset so multiple arms scan
across a grid instead of moving in lockstep.
"""

import math
import re

from omnisim import Robot

JOINT_NAMES = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
]

# (amplitude_rad, period_s, phase_offset_rad, center_rad).
# Periods are mutually-prime-ish so the pose never quite repeats. The centre
# pose folds the elbow (joint4) so the tall arm reads as a cobot at work
# rather than a flagpole, and shoulder amplitude stays small: OmniArm 7's upper
# arm + forearm are 0.7 m each, so wide swings would reach the floor.
WAVE = [
    (0.6,  9.0, 0.0,  0.0),    # joint1 base yaw — lateral sway
    (0.12, 7.0, 0.5, -0.25),   # joint2 shoulder pitch — slight forward lean
    (0.4, 13.0, 1.0,  0.0),    # joint3 upper-arm roll
    (0.25, 11.0, 1.5, 0.9),    # joint4 elbow — folded working pose
    (0.4,  6.0, 2.0,  0.0),    # joint5 forearm roll
    (0.3,  5.0, 2.5, -0.5),    # joint6 wrist pitch
    (1.0,  4.0, 3.0,  0.0),    # joint7 wrist yaw
]

FLEET_PHASE_STRIDE = math.pi / 8.0  # 22.5 deg between successive arms


def name_phase_offset(name):
    match = re.search(r"_(\d+)$", name)
    return FLEET_PHASE_STRIDE * int(match.group(1)) if match else 0.0


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    motors = [robot.getDevice(f"{name}_motor") for name in JOINT_NAMES]
    name_offset = name_phase_offset(robot.getName())

    elapsed = 0.0
    dt = time_step / 1000.0
    while robot.step(time_step) != -1:
        elapsed += dt
        for motor, (amp, period, phase, center) in zip(motors, WAVE):
            target = center + amp * math.sin(
                2.0 * math.pi * elapsed / period + phase + name_offset
            )
            motor.setPosition(target)


if __name__ == "__main__":
    main()
