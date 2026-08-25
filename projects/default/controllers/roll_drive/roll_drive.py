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

"""roll_drive -- spin every rotational motor forward at one commanded rate.

Half of the roll-check (``scripts/dev/roll_check.py``); the other half is
``roll_probe``, which measures. This one only *drives*, and it is deliberately
the dumbest controller in the tree: velocity control, one rate, no steering, no
closed loop, no knowledge of the world. That is the point -- the roll-check
grades a PHYSICAL property (does wheel spin account for body motion), and any
cleverness here would be a second variable in the measurement.

The sweep swaps EVERY robot's controller to this one in a throwaway sibling
copy of the world, so this also runs on robots that have no wheels at all. It
must therefore be inert when there is nothing to drive: no motors found means
step quietly forever, never exit -- a controller that exits takes its robot's
IPC channel down and the engine logs it as a failure.

RATE. ``OMNISIM_ROLL_OMEGA`` (rad/s, default 6.0) is set by the sweep from the
world's own statically-read wheel radius, so a 2.5 cm e-puck wheel and a 30 cm
battlebot wheel are both driven at a comparable GROUND speed instead of one
crawling and the other launching. Each motor still clamps to its own
``maxVelocity``: a world that authored a slow motor gets its slow motor, which
is part of what is under test.

RAMP. ``OMNISIM_ROLL_RAMP_S`` (default 0.5 s) ramps the command from 0, because
a step command from rest is a torque spike, and a torque spike produces
LEGITIMATE wheelspin that has nothing to do with the defect being hunted. The
probe additionally discards the spin-up window; the ramp is belt and braces.
"""

import os

from omnisim import Robot
from omnisim.motor import Motor

OMEGA = float(os.environ.get("OMNISIM_ROLL_OMEGA", "6.0"))
RAMP_S = float(os.environ.get("OMNISIM_ROLL_RAMP_S", "0.5"))


def main():
    robot = Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = []
    for index in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(index)
        # LinearMotor answers the same Python class, so filter on the device's
        # own reported type -- driving a LinearMotor at 6 "rad/s" would extend
        # a slider at 6 m/s and wreck the scene.
        if not isinstance(device, Motor) or device.getType() != Motor.ROTATIONAL:
            continue
        # Velocity control: an infinite position target is what turns a
        # position servo into a wheel. Without it setVelocity() only caps the
        # servo's speed towards a target it already sits on, i.e. nothing moves.
        device.setPosition(float("inf"))
        motors.append(device)

    if not motors:
        # Inert, but ALIVE. Exiting here would tear down this robot's IPC
        # channel and surface as a controller failure in the engine log, which
        # would make every wheel-less robot in a swept world look broken.
        while robot.step(step_ms) != -1:
            pass
        return

    for motor in motors:
        motor.setVelocity(0.0)

    while robot.step(step_ms) != -1:
        elapsed = robot.getTime()
        scale = 1.0 if RAMP_S <= 0.0 else min(1.0, elapsed / RAMP_S)
        for motor in motors:
            target = OMEGA * scale
            try:
                limit = motor.getMaxVelocity()
            except Exception:  # pragma: no cover - defensive
                limit = None
            if limit:
                target = max(-abs(limit), min(abs(limit), target))
            motor.setVelocity(target)


main()
