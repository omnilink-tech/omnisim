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

"""R1 BLIND driver -- a robot that DRIVES, and does not perceive.

The null proves the assertions can fail when nothing happens. This proves the
one assertion the null cannot exercise: **R1.5, "nothing was hit"**. A parked
robot earns no collision, so a null leaves the collision channel completely
untested -- and an untested collision channel is exactly how a task ends up
crediting a robot that ploughed through the scene. That is not hypothetical on
this arm: on the recorded ``r1_settled_omnisim`` cell an agent's rover finished
at (9.22, 17.36) -- outside a walled 10 x 10 arena -- and R1.5 reported
``robot-obstacle/wall contacts: 0`` and PASSED.

So this driver has the same rover and the same speed as the oracle, and one
difference: **it never enables the Lidar.** It is aimed straight down the
START -> GOAL diagonal by ``worlds/r1_blind.wbt`` (the only difference from
``r1_oracle.wbt`` is the rover's authored yaw and its ``controller`` field) and
it drives forward. That diagonal is blocked by three of the five obstacles, so
it drives into one.

Expected: R1.4 fails (it never arrives) and R1.5 fails **because the contact
was seen** -- which is the property being established. R1.6 is not asserted
either way here: how far a stuck robot's wheels drag it is a fact about
friction, not about the gate.

⚠ **IT DOES NOT STEER, AND ON THIS ENGINE BUILD IT COULD NOT.** Measured
2026-08-09 on this tree's binary: a motor target set AFTER the Newton world is
finalised has no effect -- neither ``setVelocity``, nor ``setPosition`` on a
range-limited motor, nor ``setTorque``. Whatever target is in place at finalize
is what the joint does for the rest of the run. Reproduced on this rover and on
a stock ``URDFRobot`` Husky. The consequence for this file is only cosmetic --
a blind straight-line driver has no steering to lose -- but it is why there is
no ORACLE beside it.
"""

MOTORS_LEFT = ("left front motor", "left rear motor")
MOTORS_RIGHT = ("right front motor", "right rear motor")

#: rad/s on every wheel. Both sides equal: straight ahead, for ever.
DRIVE_RAD_S = 8.0


def main():
    from omnisim import Robot

    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    motors = [robot.getDevice(n) for n in MOTORS_LEFT + MOTORS_RIGHT]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(DRIVE_RAD_S)
    while robot.step(dt) != -1:
        for m in motors:
            m.setVelocity(DRIVE_RAD_S)


if __name__ == "__main__":
    main()
