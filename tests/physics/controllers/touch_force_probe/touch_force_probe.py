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

"""Force-TouchSensor probe for tests/test_newton_touch_force_parity.py.

Runs on the 100 kg STACK robot whose 100 kg force TouchSensor hangs below it
(2 cm air gap -- ALL the robot weight flows through the mount, none through
contact). The stack spawns 5 cm in the air:

    tick 3      value_flight  -> still falling: mount wrench ~ 0
    +2.5 s      value_rest    -> resting: mount carries the ROBOT's 981 N
                                 (not the 1962 N floor-contact sum)

The values ARE the verdict; assertions live in the test.
Output: OMNISIM_TOUCH_PROBE_OUT.
"""
import json
import os
import sys

from omnisim import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
ts = robot.getDevice("ts")
ts.enable(dt)


def advance(ms):
    n = max(1, int(round(ms / dt)))
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
    return True


out = {}
advance(3 * dt)
out["value_flight"] = float(ts.getValue())
advance(2500)
samples = []
for _ in range(10):
    robot.step(dt)
    samples.append(float(ts.getValue()))
out["value_rest"] = sum(samples) / len(samples)
out["rest_samples"] = samples

path = os.environ.get("OMNISIM_TOUCH_PROBE_OUT", "touch_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("touch_force_probe: wrote %s\n" % path)
sys.stdout.flush()
