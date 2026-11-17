# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Build the spring-loaded variant without altering the archived rigid-slot world."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from compartment import remote_node, spring_controller_node

ROOT=Path(__file__).resolve().parent


def write_spring_models():
    """Put the estimated pads over the jaw tips, clear of their tapered mesh."""
    for side in ('left','right'):
        model=ET.parse(ROOT/f'aloha_{side}.urdf')
        for link in model.findall('link'):
            if not link.get('name','').endswith('_finger_link'):continue
            y=-.026 if 'left_finger_link' in link.get('name') else .026
            for kind in ('visual','collision'):
                for node in link.findall(kind):
                    if node.find('geometry/box') is not None:
                        node.find('origin').set('xyz',f'.078 {y} 0')
        model.write(ROOT/f'aloha_{side}_spring.urdf',encoding='utf-8',xml_declaration=True)


def battery_collider(sides=48):
    """A convex cylinder with flat ends; the primitive uses capsule contacts."""
    points=[(.0068*math.cos(2*math.pi*i/sides),
             .0068*math.sin(2*math.pi*i/sides),z)
            for z in (-.0245,.0245) for i in range(sides)]
    faces=[list(reversed(range(sides))),list(range(sides,2*sides))]
    faces += [[i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides)]
    coordinates=', '.join(' '.join(f'{x:.9f}' for x in p) for p in points)
    indices=' '.join(' '.join(map(str,face))+' -1' for face in faces)
    return f'IndexedFaceSet {{ coord Coordinate {{ point [ {coordinates} ] }} coordIndex [ {indices} ] }}'


def scene_text():
    scene=(ROOT/'worlds/aloha_battery.omniworld').read_text(encoding='utf-8')
    start=scene.index('DEF REMOTE Solid')
    end=scene.index('DEF BATTERY Solid')
    scene=scene[:start]+remote_node()+scene[end:]+spring_controller_node()
    scene=scene.replace('basicTimeStep 2','basicTimeStep 1')
    scene=scene.replace('boundingObject Pose { children [ Cylinder { radius .0068 height .049 } ] }',
                        'boundingObject '+battery_collider())
    scene=scene.replace('newtonGroundMu 3','newtonGroundMu 3\n  newtonContactKe 160000\n  newtonContactKd 800')
    scene=scene.replace('name "battery"','name "battery"\n newtonFriction .3')
    scene=scene.replace('name "table"','name "table"\n newtonFriction .5')
    scene=scene.replace('translation 0 0 -.025','translation 0 0 -.015')
    scene=scene.replace('translation .085 .013 .007','translation .069 .016 .0168')
    for side in ('left','right'):
        scene=scene.replace(f'../aloha_{side}.urdf',f'../aloha_{side}_spring.urdf')
    return scene.replace('ALOHA - recorded battery insertion','ALOHA - spring compartment development')


if __name__=='__main__':
    write_spring_models()
    path=ROOT/'worlds/aloha_battery_spring.omniworld'
    path.write_text(scene_text(),encoding='utf-8')
    print(path)
