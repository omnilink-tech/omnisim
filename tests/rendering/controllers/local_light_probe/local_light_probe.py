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
"""Four fixed-camera captures: shadow off/on at two positions of the same lamp."""
from pathlib import Path
import sys
from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
lamp = robot.getFromDef('LAMP')
output = Path(sys.argv[1])
try:
    for side, y in enumerate((-1.5, 1.5)):
        lamp.getField('location').setSFVec3f([3, y, 3])
        for shadowed in (False, True):
            lamp.getField('castShadows').setSFBool(shadowed)
            for _ in range(25):
                if robot.step(step) == -1:
                    raise RuntimeError('Simulation ended before capture')
            robot.exportImage(str(output / f'{side}_{int(shadowed)}.png'), 100)
            robot.step(step)
    robot.simulationQuit(0)
except Exception as error:
    (output / 'error.txt').write_text(str(error), encoding='utf-8')
    robot.simulationQuit(1)
