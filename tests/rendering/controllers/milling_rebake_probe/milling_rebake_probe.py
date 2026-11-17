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
"""Zig-zag the CUTTER through a MillingStock for a fixed stretch of WALL time, then quit."""
from pathlib import Path
import json
import sys
import time
from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
out = Path(sys.argv[1])
seconds = float(sys.argv[2])
cutter = robot.getFromDef('CUTTER').getField('translation')

try:
    for _ in range(60):  # let the first OmniLight bake land
        if robot.step(step) == -1:
            raise RuntimeError('Stopped during warmup')
    start = time.monotonic()
    k = 0
    while time.monotonic() - start < seconds:
        # ONE serpentine pass paced by WALL time (fast mode would otherwise sweep the block in a
        # moment and then re-cut air): 40 rows 2 mm apart across 80 mm, 5 mm into the 20 mm block,
        # so fresh material is cut for the whole run.
        u = (time.monotonic() - start) / seconds * 40.0
        row, along = min(int(u), 39), u - int(u)
        x = -0.04 + 0.08 * (along if row % 2 == 0 else 1.0 - along)
        y = -0.04 + 0.002 * row
        cutter.setSFVec3f([x, y, 0.015])
        if robot.step(step) == -1:
            raise RuntimeError('Stopped during the cut')
        k += 1
    (out / 'done.json').write_text(json.dumps({'steps': k}))
    robot.simulationQuit(0)
except Exception as error:
    (out / 'error.txt').write_text(str(error))
    robot.simulationQuit(1)
