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

"""R4 NULL driver -- the negative half of SPEC 7.1's gate, on this arm.

It connects and commands nothing. The world is otherwise byte-identical to the
oracle's (``test_r4_discriminates_webots.py`` asserts that the only line that
differs is the ``controller`` field), so the verdict difference is the driver
and nothing else.

SPEC 1.1 / 7.1: **no task may be passable by doing nothing.** That rule is not
hypothetical here -- C2 shipped a world whose UNFIXED form passed 5/5 for a
whole campaign because nobody had asserted the task could fail, and every C2
number ever recorded was uninformative as a result.

R4.1 and R4.2 SHOULD pass for this driver: the run is clean and the scene is
intact, both of which are true of an agent that did nothing, and a gate that
demanded they fail would be asking the null to be a broken world rather than an
idle agent.
"""

from controller import Robot


def main():
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    print("[null] connected; commanding nothing", flush=True)
    while robot.step(dt_ms) != -1:
        pass


if __name__ == "__main__":
    main()
