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

"""R1 NULL driver -- the negative half of the SPEC 7.1 gate, on this arm.

The same world as the oracle, the same robot, the same recorder, the same
window. It connects to the simulator and steps, and it commands nothing: no
motor is touched, no sensor is enabled, no pose is written.

SPEC 1.1 says no task may be passable by doing nothing. This is what "doing
nothing" is, and R1 must FAIL it -- while still passing R1.1 (the run IS
clean), R1.2 (the robot IS drivable) and R1.3 (the obstacles ARE intact).
Demanding that those three fail too would be asking the null to be a broken
world rather than an agent that did nothing, and would prove nothing about the
assertions that matter.

It steps rather than exiting because the recorder owns termination on this arm
(upstream has no ``--duration``): a controller that returned immediately would
be measuring an ABSENT controller, which is a different negative control.
"""

from controller import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

print("[null] connected; commanding nothing for the whole window", flush=True)

while robot.step(dt) != -1:
    pass
