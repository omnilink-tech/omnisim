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
"""Hold a Pose still the way a servo holds a body: 0.2 micrometres of jitter every step (it flips
float bits, it is far below a pixel), for a fixed stretch of WALL time, then quit. The picture
never visibly changes."""
from pathlib import Path
import json
import sys
import time
from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
out = Path(sys.argv[1])
warmup, seconds = float(sys.argv[2]), float(sys.argv[3])
field = robot.getFromDef('HELD').getField('translation')
base = field.getSFVec3f()

try:
    start = time.monotonic()
    k = 0
    while time.monotonic() - start < warmup + seconds:
        if k == 0 and time.monotonic() - start >= warmup:
            (out / 'window_start.json').write_text(json.dumps({'t': time.time()}))
            k = 1
        jitter = 2e-7 if int(robot.getTime() / (step / 1000.0)) % 2 else -2e-7
        field.setSFVec3f([base[0] + jitter, base[1], base[2]])
        if robot.step(step) == -1:
            raise RuntimeError('Stopped during the hold')
    (out / 'done.json').write_text(json.dumps({'t': time.time()}))
    robot.simulationQuit(0)
except Exception as error:
    (out / 'error.txt').write_text(str(error))
    robot.simulationQuit(1)
