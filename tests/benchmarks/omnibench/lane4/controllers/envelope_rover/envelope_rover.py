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

"""envelope_rover — lane-4b wheeled-robot driver.

Drives all four wheels at a constant velocity so the rover is genuinely
under load for the whole measurement window. The credibility checklist this
suite follows is explicit that actions must never be idle: a parked robot
generates a different (easier) contact problem than a driving one, and a
throughput number taken on parked robots is not a throughput number for
robots.

Set the wheel targets ONCE and then step: re-issuing setVelocity every tick
would add an FFI round trip per wheel per step into the very timing bucket
this lane measures.
"""

import sys

from omnisim import Robot


def main():
    robot = Robot()
    step_ms = int(round(robot.getBasicTimeStep())) or 1
    speed = 4.0
    for a in sys.argv[1:]:
        if a.startswith("--speed="):
            speed = float(a.split("=", 1)[1])
    for name in ("w_fl", "w_fr", "w_rl", "w_rr"):
        m = robot.getDevice(name)
        if m is None:
            print("[envelope_rover] missing motor %r" % name, flush=True)
            continue
        m.setPosition(float("inf"))     # velocity control
        m.setVelocity(speed)
    while robot.step(step_ms) != -1:
        pass


main()
