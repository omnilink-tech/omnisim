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
"""Edits that must invalidate a shadow atlas, captured after each settles."""
from pathlib import Path
import sys
import json
from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
output = Path(sys.argv[1])
lamp = robot.getFromDef('LAMP')
block = robot.getFromDef('BLOCK')
shape = robot.getFromDef('BLOCK_SHAPE')
material = robot.getFromDef('BLOCK_MAT')
box = robot.getFromDef('BLOCK_BOX')
view = robot.getFromDef('VIEW')
camera = robot.getDevice('probe')
camera.enable(step)

def capture(name):
    for _ in range(20):
        if robot.step(step) == -1:
            raise RuntimeError('Simulation ended before capture')
    image = output / (name + '.png')
    robot.exportImage(str(image), 100)
    # Main-view export is queued to the GUI. Wait for the completed PNG before
    # the next scene edit, otherwise a delayed capture can photograph that edit.
    for _ in range(100):
        if robot.step(step) == -1:
            raise RuntimeError('Simulation ended while waiting for screenshot')
        try:
            with image.open('rb') as saved:
                saved.seek(-12, 2)
                if saved.read() == bytes.fromhex('0000000049454e44ae426082'):
                    break
        except OSError:
            pass
    else:
        raise RuntimeError(f'Screenshot did not finish: {name}')
    camera.saveImage(str(output / (name + '-sensor.png')), 100)
    (output / (name + '.json')).write_text(json.dumps({
        'time': robot.getTime(), 'lamp_shadows': lamp.getField('castShadows').getSFBool(),
        'block_shadows': shape.getField('castShadows').getSFBool(),
        'transparency': material.getField('transparency').getSFFloat(),
        'size': box.getField('size').getSFVec3f()}))
    robot.step(step)

try:
    capture('initial')
    capture('unchanged')
    view.getField('position').setSFVec3f([0, -.4, 3])
    capture('camera')
    lamp.getField('location').setSFVec3f([3, 1.5, 3])
    capture('lamp')
    block.getField('translation').setSFVec3f([4, .3, .5])
    capture('caster')
    box.getField('size').setSFVec3f([1.8, .8, 1])
    capture('geometry')
    shape.getField('castShadows').setSFBool(False)
    capture('no_cast')
    shape.getField('castShadows').setSFBool(True)
    material.getField('transparency').setSFFloat(.5)
    capture('transparent')
    material.getField('transparency').setSFFloat(0.0)
    lamp.getField('castShadows').setSFBool(False)
    capture('disabled')
    lamp.getField('castShadows').setSFBool(True)
    capture('enabled')
    robot.getRoot().getField('children').importMFNodeFromString(-1, '''
      DEF INSERTED Solid { translation 2.5 -1.3 0.7 children [ Shape {
        appearance PBRAppearance { baseColor 0.4 0.2 0.1 roughness 1 metalness 0 }
        geometry Box { size .5 .5 1.4 }
      } ] }''')
    # The import is a supervisor field transaction. Read back its result before
    # scheduling captures, so every comparison arm has the same committed scene.
    inserted = robot.getFromDef('INSERTED')
    if inserted is None or inserted.getField('translation').getSFVec3f() != [2.5, -1.3, 0.7]:
        raise RuntimeError('Inserted node was not acknowledged by the supervisor')
    capture('inserted')
    # Repeated growth exercises recycled native buffer-handle addresses. A
    # pointer-equality cache can otherwise retain an older, smaller GPU buffer.
    additions = []
    for index in range(1, 4):
        robot.getRoot().getField('children').importMFNodeFromString(-1, f'''
          DEF ADDED_{index} Solid {{ translation 2.6 {-1.3 + .6*index} 1.1 children [ Shape {{
            appearance PBRAppearance {{ baseColor .2 .5 .2 roughness 1 metalness 0 }}
            geometry Box {{ size .25 .25 1 }}
          }} ] }}''')
        addition = robot.getFromDef(f'ADDED_{index}')
        if addition is None:
            raise RuntimeError('Additional object was not acknowledged')
        additions.append(addition)
        capture(f'grown{index}')
    for addition in additions:
        addition.remove()
    inserted.remove()
    capture('removed')
    lamp.getField('radius').setSFFloat(4.0)
    capture('radius')
    robot.simulationQuit(0)
except Exception as error:
    (output / 'error.txt').write_text(str(error), encoding='utf-8')
    robot.simulationQuit(1)
