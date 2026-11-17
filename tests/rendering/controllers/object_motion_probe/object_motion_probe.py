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
"""Bounded rigid motion, insertion, geometry and camera edits for temporal rendering."""
from pathlib import Path
import json
import sys
from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
out = Path(sys.argv[1])
block = robot.getFromDef('BLOCK')
view = robot.getFromDef('VIEW')
camera = robot.getDevice('probe')
camera.enable(step)
states = []
try:
    for _ in range(32):
        if robot.step(step) == -1: raise RuntimeError('Stopped during warmup')
    for frame in range(12):
        block.getField('translation').setSFVec3f([3, -.45+.08*frame, 1.0])
        block.getField('rotation').setSFRotation([0,0,1, .08*frame])
        if frame == 4:
            robot.getRoot().getField('children').importMFNodeFromString(-1, '''
                DEF NEW Solid { translation 2.6 -.5 .9 children [ Shape {
                    appearance PBRAppearance { baseColor .7 .15 .1 roughness 1 metalness 0 }
                    geometry Box { size .2 .25 .7 }
                } ] }''')
            if robot.getFromDef('NEW') is None: raise RuntimeError('Insertion failed')
        if frame == 6: robot.getFromDef('NEW').remove()
        if frame == 8: robot.getFromDef('BOX').getField('size').setSFVec3f([.25,1.1,1.2])
        if frame >= 9: view.getField('position').setSFVec3f([0, -.04*(frame-8), 2.3])
        if robot.step(step) == -1: raise RuntimeError('Stopped during motion')
        path = out / f'{frame:02}.png'
        robot.exportImage(str(path), 100)
        for _ in range(100):
            if robot.step(step) == -1: raise RuntimeError('Stopped during capture')
            try:
                with path.open('rb') as saved:
                    saved.seek(-12, 2)
                    if saved.read() == bytes.fromhex('0000000049454e44ae426082'): break
            except OSError: pass
        else: raise RuntimeError('Capture did not complete')
        camera.saveImage(str(out / f'{frame:02}-sensor.png'), 100)
        states.append({'frame':frame, 'position':block.getField('translation').getSFVec3f(),
                       'rotation':block.getField('rotation').getSFRotation()})
    (out/'states.json').write_text(json.dumps(states))
    robot.simulationQuit(0)
except Exception as error:
    (out/'error.txt').write_text(str(error))
    robot.simulationQuit(1)
