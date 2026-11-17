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

"""OmniArm 6 wave demo controller.

Drives the six revolute joints of the OmniArm 6 cobot with a slow sinusoidal
pattern so the arm has visible motion in the sample world without any
external input. Each joint gets a different frequency and a small
amplitude so the arm doesn't whip itself against its own joint limits.

When the robot name ends in an integer suffix (e.g. "omniarm6_3" in a fleet
world), that integer becomes a global phase offset so every arm in the
fleet sits at a different point in the same wave - it scans across the
grid instead of moving in lockstep. The base name "omniarm6" has no suffix
and behaves identically to the single-arm version.
"""

import math
import re

from omnisim import Robot

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# (amplitude_rad, period_s, phase_offset_rad, center_rad).
# Periods are mutually-prime-ish so the pose never quite repeats. Shoulder
# and elbow amplitudes are deliberately small so the arm stays upright and
# never swings the dense wrist/forearm hulls into the pedestal — that's
# what was producing the 33-contact-point physics spam in the original
# wider wave.
WAVE = [
    (0.6,  9.0, 0.0,  0.0),    # joint1 base yaw — full lateral sway
    (0.15, 7.0, 0.5, -0.05),   # joint2 shoulder — tiny forward lean
    (0.20, 11.0, 1.0, 0.30),   # joint3 elbow — gentle bend
    (0.6, 13.0, 1.5, 0.0),     # joint4 wrist roll
    (0.25, 6.0, 2.0, 0.0),     # joint5 wrist pitch
    (1.0,  4.0, 2.5, 0.0),     # joint6 wrist yaw
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
