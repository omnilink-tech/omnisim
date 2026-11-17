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

"""R1 NULL driver -- the negative half of SPEC 7.1's gate, on the OmniSim arm.

It steps the simulation and commands nothing. Same world, same robot, same
recorder, same grader as ``r1_oracle`` -- only the ``controller`` field
differs -- so the verdict difference can only be the program.

What it must NOT do is fail for the wrong reason. R1.1 (the run is clean),
R1.2 (there is one drivable robot) and R1.3 (the obstacles are intact) are all
TRUE of an agent that did nothing in a well-formed world, and a gate that
demanded they fail would be asking the null to be a broken world rather than
an idle agent. The three that must fail are R1.4 (it never arrived), R1.6 (it
never drove) and -- the one a lazy grader hands out for free -- R1.5: a robot
that never moved hits nothing, and ``MIN_MOTION_FOR_CREDIT_M`` is what stops
that from being scored as collision-free navigation.

It holds the wheels at zero velocity rather than leaving them unset, so
"stationary" is a commanded state and not an accident of the motor default.
"""

MOTORS = ("left front motor", "left rear motor",
          "right front motor", "right rear motor")


def main():
    from omnisim import Robot

    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    for name in MOTORS:
        m = robot.getDevice(name)
        if m is not None:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
    while robot.step(dt) != -1:
        pass


if __name__ == "__main__":
    main()
