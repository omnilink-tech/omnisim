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

"""Bumper-TouchSensor probe for tests/test_newton_touch_bumper_parity.py.

Runs on the PROBER robot, whose bumper pad protrudes 10 mm BELOW the chassis,
so the scene answers two questions with one drop:

    tick 3      value_flight  -> still falling, nothing touching  -> 0
    +2.5 s      value_rest    -> resting on the pad               -> 1
                rest_z        -> 0.66 if the PAD is the collider,
                                 0.65 if the pad was dropped by the fold
                                 and the CHASSIS took the contact

REST_Z IS NOT DECORATION. The bumper defect was one mechanism with two
symptoms -- the sensor could not be asked about its contacts AND its
boundingObject was not a collider at all -- so a fix that restored only the
read would leave the scene's collision geometry wrong by the pad's protrusion
and this probe would still look green on value alone.

The values ARE the verdict; assertions live in the test.
Output: OMNISIM_BUMPER_PROBE_OUT.
"""
import json
import os
import sys

from omnisim import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
ts = robot.getDevice("ts")
ts.enable(dt)
me = robot.getSelf()


def advance(ms):
    n = max(1, int(round(ms / dt)))
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
    return True


out = {}
advance(3 * dt)
out["value_flight"] = float(ts.getValue())
out["z_flight"] = float(me.getPosition()[2])
advance(2500)
samples = []
zs = []
for _ in range(10):
    robot.step(dt)
    samples.append(float(ts.getValue()))
    zs.append(float(me.getPosition()[2]))
# A bumper is a per-step latch that resets once read, so a settled contact can
# still miss the odd sample; the DUTY over a resting window is the honest
# statistic, not any single read.
out["value_rest_duty"] = sum(1 for v in samples if v > 0.5) / float(len(samples))
out["rest_samples"] = samples
out["rest_z"] = sum(zs) / len(zs)

path = os.environ.get("OMNISIM_BUMPER_PROBE_OUT", "bumper_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("touch_bumper_probe: wrote %s\n" % path)
sys.stdout.flush()
