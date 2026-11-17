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

"""Read the radar target list for a few ticks; write count + distances.

Instrument for tests/test_newton_radar_occlusion_parity.py: the surviving
target set IS the verdict (an occluded target is dropped from the output), so
this controller does nothing else. Output path comes from
OMNISIM_RADAR_PROBE_OUT (written once, after a couple of refreshes so target
speeds have settled).
"""
import json
import os
import sys

from omnisim import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

radar = robot.getDevice("radar")
radar.enable(dt)

# several ticks: the first radar refresh has no previous-translation history,
# later refreshes are steady-state
for _ in range(12):
    if robot.step(dt) == -1:
        break

targets = radar.getTargets()
out = {
    "count": radar.getNumberOfTargets(),
    "distances": sorted(round(t.distance, 4) for t in targets),
}
path = os.environ.get("OMNISIM_RADAR_PROBE_OUT", "radar_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("radar_occlusion_probe: wrote %d targets -> %s\n" % (out["count"], path))
sys.stdout.flush()
