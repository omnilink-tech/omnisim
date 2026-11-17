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
"""Export the MAIN VIEW twice: as authored, then after a supervisor moved the bare Pose."""
from pathlib import Path
import json
import sys
from omnisim import Supervisor

IEND = bytes.fromhex('0000000049454e44ae426082')

robot = Supervisor()
step = int(robot.getBasicTimeStep())
out = Path(sys.argv[1])


def export(name):
    path = out / name
    robot.exportImage(str(path), 100)
    for _ in range(200):
        if robot.step(step) == -1:
            raise RuntimeError('Stopped during capture')
        try:
            with path.open('rb') as saved:
                saved.seek(-12, 2)
                if saved.read() == IEND:
                    return
        except OSError:
            pass
    raise RuntimeError('Capture of %s did not complete' % name)


try:
    for _ in range(30):
        if robot.step(step) == -1:
            raise RuntimeError('Stopped during warmup')
    export('authored.png')
    robot.getFromDef('LOOSE').getField('translation').setSFVec3f([0, 0.6, 0.3])
    for _ in range(4):
        robot.step(step)
    export('moved.png')
    (out / 'done.json').write_text(json.dumps({'sim_time': robot.getTime()}))
    robot.simulationQuit(0)
except Exception as error:
    (out / 'error.txt').write_text(str(error))
    robot.simulationQuit(1)
