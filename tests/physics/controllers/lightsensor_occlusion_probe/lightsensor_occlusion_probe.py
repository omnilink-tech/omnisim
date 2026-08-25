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

"""Read every LightSensor on the robot for a few ticks; write the values.

Instrument for tests/test_newton_lightsensor_occlusion_parity.py: the sensor
readings ARE the verdict, so this controller does nothing else. Output path
comes from OMNISIM_LIGHTSENSOR_PROBE_OUT (written once, after the readings
settle).
"""
import json
import os
import sys

from omnisim import LightSensor, Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

sensors = {}
for i in range(robot.getNumberOfDevices()):
    d = robot.getDeviceByIndex(i)
    if isinstance(d, LightSensor):
        d.enable(dt)
        sensors[d.getName()] = d

# a few ticks so the scene settles and every sensor has a fresh sample
for _ in range(8):
    if robot.step(dt) == -1:
        break

out = {name: {"value": s.getValue()} for name, s in sensors.items()}
path = os.environ.get("OMNISIM_LIGHTSENSOR_PROBE_OUT", "lightsensor_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("lightsensor_occlusion_probe: wrote %d sensors -> %s\n" % (len(out), path))
sys.stdout.flush()
