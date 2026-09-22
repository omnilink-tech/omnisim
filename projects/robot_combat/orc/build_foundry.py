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
"""Build the deterministic Foundry world; no engine, GPU or external assets required."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/python"))
sys.path.insert(0, str(HERE / "shared"))
from omniworld.viewpoint import look_at
from foundry_physics import WHEEL, CHASSIS, BAR, DRUM, part_spec
from foundry_config import normalize_match, COLORS


def numbers(values):
    return " ".join(f"{v:.6g}" for v in values)


def physics(spec):
    return (f'Physics {{ density -1 mass {spec.mass:g} centerOfMass [ 0 0 0 ] '
            f'inertiaMatrix [ {numbers(spec.inertia)} 0 0 0 ] }}')


def shape(geometry, material):
    return f"Shape {{ appearance USE {material} geometry {geometry} }}"


def pose(position, children, rotation=(0, 0, 1, 0)):
    return f"Pose {{ translation {numbers(position)} rotation {numbers(rotation)} children [ {children} ] }}"


def box(position, size, material, rotation=(0, 0, 1, 0)):
    return pose(position, shape(f"Box {{ size {numbers(size)} }}", material), rotation)


def cylinder(position, radius, height, material, rotation=(0, 0, 1, 0), sides=24):
    return pose(position, shape(f"Cylinder {{ radius {radius} height {height} subdivision {sides} }}", material), rotation)


def cylinder_collider(radius, height, sides=64):
    """Convex cylinder with exact axial extent; avoids the capsule fallback."""
    points = [(radius*math.cos(i*math.tau/sides),radius*math.sin(i*math.tau/sides),z)
              for z in (-height/2,height/2) for i in range(sides)]
    faces = [list(reversed(range(sides))),list(range(sides,2*sides))]
    faces += [[i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides)]
    return ('IndexedFaceSet { coord Coordinate { point [ '+','.join(numbers(p) for p in points)+
            ' ] } coordIndex [ '+' '.join(' '.join(map(str,f))+' -1' for f in faces)+' ] }')


def chamfered(size, bevel=0.025):
    """Closed convex panel, with clipped corners and bevels catching grazing light."""
    x, y, z = [v/2 for v in size]
    b = min(bevel, z*.45, x*.2, y*.2)
    points = []
    for h, inset in [(-z, b), (-z+b, 0), (z-b, 0), (z, b)]:
        a, c = x-inset, y-inset
        k = max(b, min(x, y)*.14)
        points += [(a-k, -c, h), (a, -c+k, h), (a, c-k, h), (a-k, c, h),
                   (-a+k, c, h), (-a, c-k, h), (-a, -c+k, h), (-a+k, -c, h)]
    faces = [list(reversed(range(8))), list(range(24, 32))]
    faces += [[8*j+i, 8*j+(i+1)%8, 8*(j+1)+(i+1)%8, 8*(j+1)+i]
              for j in range(3) for i in range(8)]
    return ("IndexedFaceSet { coord Coordinate { point [ " +
            ", ".join(numbers(p) for p in points) + " ] } coordIndex [ " +
            " ".join(" ".join(map(str, f))+" -1" for f in faces) + " ] creaseAngle 0.25 }")


def panel(position, size, material, rotation=(0, 0, 1, 0)):
    return pose(position, shape(chamfered(size), material), rotation)


# Line lettering is actual world geometry, legible without a raster-text node.
GLYPHS = {
    "A": [[(0,0),(.5,1),(1,0)],[(.23,.4),(.77,.4)]],
    "C": [[(1,1),(0,1),(0,0),(1,0)]],
    "D": [[(0,0),(0,1),(.7,1),(1,.7),(1,.3),(.7,0),(0,0)]],
    "F": [[(0,0),(0,1),(1,1)],[(0,.5),(.8,.5)]],
    "I": [[(.5,0),(.5,1)]], "L": [[(0,1),(0,0),(1,0)]],
    "N": [[(0,0),(0,1),(1,0),(1,1)]],
    "O": [[(0,0),(0,1),(1,1),(1,0),(0,0)]],
    "R": [[(0,0),(0,1),(1,1),(1,.55),(0,.55)],[(.45,.55),(1,0)]],
    "U": [[(0,1),(0,0),(1,0),(1,1)]], "V": [[(0,1),(.5,0),(1,1)]],
    "Y": [[(0,1),(.5,.5),(1,1)],[(.5,.5),(.5,0)]],
    "Z": [[(0,1),(1,1),(0,0),(1,0)]],
    "0": [[(0,0),(0,1),(1,1),(1,0),(0,0)]],
    "1": [[(.2,.8),(.5,1),(.5,0)]], "7": [[(0,1),(1,1),(.3,0)]],
}
GLYPHS.update({
    'B':[[(0,0),(0,1),(.75,1),(1,.75),(.75,.5),(0,.5),(.75,.5),(1,.25),(.75,0),(0,0)]],
    'E':[[(1,1),(0,1),(0,0),(1,0)],[(0,.5),(.8,.5)]],
    'G':[[(1,.8),(.8,1),(0,1),(0,0),(1,0),(1,.5),(.5,.5)]],
    'H':[[(0,0),(0,1)],[(1,0),(1,1)],[(0,.5),(1,.5)]],
    'J':[[(0,.2),(.2,0),(.8,0),(1,.2),(1,1)]],
    'K':[[(0,0),(0,1)],[(1,1),(0,.5),(1,0)]],
    'M':[[(0,0),(0,1),(.5,.45),(1,1),(1,0)]],
    'P':[[(0,0),(0,1),(1,1),(1,.5),(0,.5)]],
    'Q':[[(0,0),(0,1),(1,1),(1,0),(0,0)],[(.6,.3),(1.1,-.1)]],
    'S':[[(1,1),(0,1),(0,.5),(1,.5),(1,0),(0,0)]],
    'T':[[(0,1),(1,1)],[(.5,1),(.5,0)]],
    'W':[[(0,1),(.2,0),(.5,.5),(.8,0),(1,1)]],
    'X':[[(0,0),(1,1)],[(0,1),(1,0)]],
    '2':[[(0,1),(1,1),(1,.5),(0,0),(1,0)]],
    '3':[[(0,1),(1,1),(1,0),(0,0)],[(.3,.5),(1,.5)]],
    '4':[[(0,1),(0,.5),(1,.5)],[(1,1),(1,0)]],
    '5':[[(1,1),(0,1),(0,.5),(1,.5),(1,0),(0,0)]],
    '6':[[(1,1),(0,1),(0,0),(1,0),(1,.5),(0,.5)]],
    '8':[[(0,0),(0,1),(1,1),(1,0),(0,0)],[(0,.5),(1,.5)]],
    '9':[[(1,0),(1,1),(0,1),(0,.5),(1,.5)]],
    '-':[[(.2,.5),(.8,.5)]],
})


def lettering(text, position, height, material, rotation=(0, 0, 1, 0)):
    children = []
    for index, char in enumerate(text):
        for polyline in GLYPHS.get(char, []):
            for a, b in zip(polyline, polyline[1:]):
                start = ((a[0]*.65+index*.85)*height, a[1]*height)
                end = ((b[0]*.65+index*.85)*height, b[1]*height)
                angle = math.atan2(end[1]-start[1], end[0]-start[0])
                children.append(box(((start[0]+end[0])/2, (start[1]+end[1])/2, 0),
                                    (math.dist(start, end)+.035*height, .055*height, .008),
                                    material, (0,0,1,angle)))
    return pose(position, "\n".join(children), rotation)


def static(name, position, size, material):
    return (f'DEF {name} Solid {{ translation {numbers(position)} name "{name.lower()}" '
            f'children [ {shape(f"Box {{ size {numbers(size)} }}", material)} ] '
            f'boundingObject Box {{ size {numbers(size)} }} }}')


def materials(match=None):
    definitions = {
        "CONCRETE": '''PBRAppearance { baseColor 0.75 0.79 0.82 roughness 0.94 metalness 0
          baseColorMap ImageTexture { url [ "omnisim://projects/appearances/protos/textures/asphalt/asphalt_base_color.jpg" ] }
          normalMap ImageTexture { url [ "omnisim://projects/appearances/protos/textures/asphalt/asphalt_normal.jpg" ] }
          normalMapFactor 0.12 textureTransform TextureTransform { scale 28 28 } }''',
        "WALL": 'RoughConcrete { colorOverride 0.37 0.40 0.42 textureTransform TextureTransform { scale 16 8 } }',
        "STEEL": 'PBRAppearance { baseColor 0.49 0.53 0.57 roughness 0.48 metalness 0.85 }',
        "DARK": 'PBRAppearance { baseColor 0.045 0.055 0.062 metalness 0.7 roughness 0.48 }',
        "RUBBER": 'PBRAppearance { baseColor 0.025 0.029 0.033 roughness 0.92 metalness 0 }',
        "RUST": 'ScratchedPaint { colorOverride 0.30 0.13 0.064 }',
        "ORANGE": 'ScratchedPaint { colorOverride 0.92 0.28 0.07 }',
        "BLUE": 'ScratchedPaint { colorOverride 0.09 0.25 0.30 }',
        "MARKING": 'PBRAppearance { baseColor 0.68 0.62 0.44 roughness 0.95 metalness 0 }',
        "WHITE": 'PBRAppearance { baseColor 0.72 0.75 0.70 roughness 0.65 metalness 0 }',
        "AMBER": 'PBRAppearance { baseColor 1 0.23 0.04 emissiveColor 1 0.24 0.04 emissiveIntensity 4 roughness 0.4 metalness 0 }',
        "CYAN": 'PBRAppearance { baseColor 0.08 0.65 0.74 emissiveColor 0.08 0.65 0.74 emissiveIntensity 3 roughness 0.4 metalness 0 }',
    }
    if match:
        for slot,key in (('ANVIL','TEAM_A'),('RAZOR','TEAM_B')):
            color = COLORS[match['robots'][slot]['color']].lstrip('#')
            rgb = [int(color[i:i+2],16)/255 for i in (0,2,4)]
            definitions[key] = f'ScratchedPaint {{ colorOverride {numbers(rgb)} }}'
    return 'Pose { translation 0 0 -10 children [\n' + '\n'.join(
        f'Shape {{ appearance DEF {key} {value} geometry Box {{ size 0.001 0.001 0.001 }} }}'
        for key, value in definitions.items()) + '\n] }'


def wheel(prefix, short, x, y, settings=None):
    motor_name = {"fl":"front_left", "fr":"front_right", "rl":"rear_left", "rr":"rear_right"}[short]
    bits = [cylinder((0,0,0), .17, .115, "RUBBER"),
            cylinder((0,0,.062), .111, .017, "DARK"),
            cylinder((0,0,-.062), .111, .017, "DARK")]
    for z in (-.073, .073):
        bits.append(cylinder((0,0,z), .06, .016, "STEEL", sides=12))
        for n in range(6):
            t = n*math.tau/6
            bits.append(cylinder((.08*math.cos(t),.08*math.sin(t),z), .012, .018,"STEEL",sides=6))
    for n in range(16):
        t = n*math.tau/16
        bits.append(box((.163*math.cos(t),.163*math.sin(t),0),(.022,.05,.122),"RUBBER",(0,0,1,t)))
    return f'''DEF {prefix}_{short.upper()}_JOINT HingeJoint {{
 jointParameters HingeJointParameters {{ axis 0 1 0 anchor {x} {y} -0.02 }}
 device [ PositionSensor {{ name "{motor_name}_wheel_sensor" }} ]
 endPoint DEF {prefix}_{short.upper()}_WHEEL Solid {{
  translation {x} {y} -0.02 rotation 1 0 0 1.570796327
  name "{short}_wheel" children [ {' '.join(bits)} ]
  boundingObject {cylinder_collider(.17,.122)}
  newtonFriction 0.9 physics {physics(part_spec(prefix,short+'_wheel',{prefix:settings}) if settings else WHEEL)}
 }}
}}'''


def robot(prefix, position, yaw, color, drum=False, settings=None):
    if settings:
        drum = settings['weapon'] == 'drum'
    luminous = "CYAN" if drum else "AMBER"
    chassis_colliders = [chamfered((1.12,.78,.30)),
                        pose((-.03,0,.14),chamfered((1.05,.72,.06))),
                        pose((-.16,0,.191),chamfered((.55,.43,.048)))]
    pieces = [panel((0,0,0), (1.12,.78,.30),"DARK"),
              panel((-.03,0,.14),(1.05,.72,.06),color),
              panel((-.16,0,.191),(.55,.43,.048),"DARK")]
    for y in (-.34, .34):
        pieces += [panel((-.1,y,.183),(.84,.09,.047),"STEEL"),
                   box((.33,y,.198),(.18,.035,.018),luminous)]
    for n in range(6):
        pieces.append(box((-.36+n*.078,0,.219),(.023,.27,.012),"RUBBER"))
    for x in (-.44,-.12,.25,.46):
        for y in (-.26,.26):
            pieces.append(cylinder((x,y,.184),.018,.018,"STEEL",sides=6))
    pieces.append(panel((.36,0,.178),(.28,.49,.025),color))
    if settings:
        pieces.append(lettering(settings['name'],(-.34,-.16,.233),.05*min(1,10/len(settings['name'])),"WHITE"))
    else:
        pieces.append(lettering(prefix,(.24,-.16,.196),.067,"WHITE"))
    for short,x,y in [("fl",.39,.455),("fr",.39,-.455),("rl",-.39,.455),("rr",-.39,-.455)]:
        pieces.append(wheel(prefix,short,x,y,settings))
    if drum:
        visuals = [cylinder((0,0,0),.176,.49,"STEEL")]
        colliders = [cylinder_collider(.176,.49)]
        for n in range(4):
            t = n*math.tau/4
            p=(.17*math.cos(t),.17*math.sin(t),0)
            visuals.append(box(p,(.065,.058,.45),color,(0,0,1,t)))
            colliders.append(pose(p, 'Box { size 0.065 0.058 0.45 }',(0,0,1,t)))
        axis, position_weapon, rotation = "0 1 0", ".79 0 .07", "1 0 0 1.570796327"
    else:
        visuals = [panel((0,0,0),(.99,.15,.075),"STEEL"),
                   cylinder((0,0,0),.13,.10,"DARK"),
                   panel((.45,0,0),(.14,.22,.09),color),
                   panel((-.45,0,0),(.14,.22,.09),color)]
        colliders = ['Box { size 0.99 0.15 0.075 }',
                     pose((.45,0,0),'Box { size 0.14 0.22 0.09 }'),
                     pose((-.45,0,0),'Box { size 0.14 0.22 0.09 }')]
        # The blade must clear its own chassis at EVERY spin angle, including
        # when a removed joint no longer filters parent/child collisions.
        # The tooth corner sweeps a 0.532 m radius. Moving the spindle
        # forward clears the 0.56 m chassis front without lifting the blade
        # above the opponent's armor and wheels. The support is below it.
        pieces += [box((.83,0,-.105),(.60,.14,.06),'DARK'),
                   cylinder((1.12,0,-.07),.055,.10,'STEEL')]
        chassis_colliders += [pose((.83,0,-.105),'Box { size .60 .14 .06 }'),
                              pose((1.12,0,-.07),cylinder_collider(.055,.10))]
        axis, position_weapon, rotation = "0 0 1", "1.12 0 .03", "0 0 1 0"
    pieces.append(f'''DEF {prefix}_WEAPON_JOINT HingeJoint {{
 jointParameters HingeJointParameters {{ axis {axis} anchor {position_weapon} }}
 device [ PositionSensor {{ name "weapon_sensor" }} ]
 endPoint DEF {prefix}_WEAPON Solid {{
  translation {position_weapon} rotation {rotation} name "weapon"
  children [ {' '.join(visuals)} ]
  boundingObject Group {{ children [ {' '.join(colliders)} ] }}
  newtonFriction 0.45 physics {physics(part_spec(prefix,'weapon',{prefix:settings}) if settings else DRUM if drum else BAR)}
 }}
}}''')
    return f'''DEF {prefix} Robot {{
 translation {numbers(position)} rotation 0 0 1 {yaw}
 name "{prefix.lower()}" supervisor TRUE controller "orc_foundry_pilot"
 controllerArgs [ "{prefix}"{' '+json.dumps(json.dumps(settings)) if settings else ''} ]
 children [ {' '.join(pieces)} ]
 boundingObject Group {{ children [ {' '.join(chassis_colliders)} ] }}
 physics {physics(part_spec(prefix,'chassis',{prefix:settings}) if settings else CHASSIS)}
}}'''


def environment():
    scene = [static("YARD_FLOOR",(0,0,-.12),(32,32,.24),"CONCRETE")]
    # The center is open driving space; the perimeter supplies cover and scale.
    for x in (-14.8,14.8):
        scene.append(static(f"SIDE_{'W' if x<0 else 'E'}",(x,0,.65),(.45,30,1.3),"WALL"))
    scene.append(static("SOUTH_WALL",(0,-14.8,.65),(29.6,.45,1.3),"WALL"))
    scene.append(static("FACTORY",(0,17,4.3),(30,5.5,8.6),"WALL"))
    scene.append(box((0,14.21,5.8),(29.8,.12,3.9),"DARK"))
    for x in range(-14,15,2):
        scene.append(box((x,14.08,4.45),(.13,.23,8.7),"RUST"))
    for x in (-9,0,9):
        scene.append(box((x,14.09,2.05),(5.2,.16,4.0),"STEEL"))
        for z in range(1,14):
            scene.append(box((x,13.98,z*.29), (5.15,.07,.032),"DARK"))
        scene.append(box((x,13.8,4.3),(4.9,.16,.07),"AMBER"))
    scene.append(box((0,13.85,6.75),(9,.28,1.9),"DARK"))
    scene.append(lettering("ORC",(-3.7,13.66,6.3),.88,"WHITE",(1,0,0,math.pi/2)))
    scene.append(lettering("FOUNDRY",(-.85,13.65,6.38),.55,"AMBER",(1,0,0,math.pi/2)))
    for x in (-11.5,11.5):
        for y in (-7.5,2.5,9.5):
            tag = f"BARRIER_{str(x).replace('-','N').replace('.','_')}_{str(y).replace('-','N').replace('.','_')}"
            scene.append(static(tag,(x,y,.48),(2.6,.65,.96),"WALL"))
            scene.append(box((x,y-.333,.68),(2.5,.012,.16),"MARKING"))
            for offset in (-1,-.5,0,.5,1):
                scene.append(box((x+offset,y-.344,.68),(.1,.013,.17),"DARK",(0,1,0,-.3)))
    # Drainage, expansion seams, painted stopping bays and skid marks.
    for n in range(-3,4):
        scene.append(box((n*4,0,.001),( .018,29,.002),"DARK"))
        scene.append(box((0,n*4,.001),(29,.018,.002),"DARK"))
    for y in (-10,10):
        scene.append(box((0,y,.003),(18,.34,.004),"DARK"))
        for x in range(-9,10):
            scene.append(box((x,y,.006),(.034,.33,.009),"STEEL"))
    for x in (-7,7):
        for y in (-6,-2,2,6):
            scene.append(box((x,y,.004),(.07,1.2,.008),"MARKING"))
    for x in (-3.1,3.1):
        for y in (-.65,.65):
            scene.append(box((x,y,.005),(2.1,.06,.009),"MARKING"))
    for x,y in [(-8,7),(8,-6),(8,8)]:
        tag=f"CRATE_{x}_{y}".replace('-','N')
        scene.append(static(tag,(x,y,.62),(1.5,1.2,1.24),"BLUE"))
        for dx in (-.69,.69):
            scene.append(box((x+dx,y,.65),(.09,1.26,1.28),"STEEL"))
    # Gantry and pipes frame the encounter without blocking the approach.
    for x in (-10,10):
        scene.append(static(f"GANTRY_{'W' if x<0 else 'E'}",(x,7,2.6),(.4,.6,5.2),"RUST"))
    scene.append(box((0,7,5.2),(20.5,.7,.45),"RUST"))
    scene.append(box((0,6.9,4.6),(5.6,.22,.9),"DARK"))
    scene.append(lettering("FOUNDRY",(-2.1,6.75,4.33),.58,"WHITE",(1,0,0,math.pi/2)))
    for x in (-9,9):
        scene.append(cylinder((x,18,10),.7,7,"RUST"))
        for z in (7,10,13):
            scene.append(cylinder((x,18,z),.77,.12,"STEEL"))
    for x in (-13,13):
        scene.append(cylinder((x,5,3.5),.12,7,"DARK"))
        scene.append(box((x,4.7,7),(1,.75,.12),"STEEL"))
        scene.append(box((x,4.7,6.925),(.8,.55,.03),"AMBER"))
    colliders = [node for node in scene if node.startswith('DEF ')]
    visuals = [node for node in scene if not node.startswith('DEF ')]
    # The native draw collector traverses the world's Solids. A bare root
    # Pose can parse successfully yet leave its decoration outside that list.
    # One visual-only Solid makes all dressing visible without adding colliders.
    return '\n'.join(colliders) + '\nDEF YARD_DETAILS Solid { name "yard_details" children [\n' + '\n'.join(visuals) + '\n] }'


def build(match=None, garage=False):
    match = normalize_match(match) if match is not None else None
    director = {'mode':'manual','duration_s':180}
    if match:
        director.update(mode='demo' if garage else 'manual',duration_s=match['duration_s'],match=match,garage=garage)
    eye = (-2.5,-8,2.0)
    imports = ["objects/backgrounds/protos/OmniSimSky", "objects/lights/protos/OmniSimSun",
               "objects/lights/protos/OmniSimSunMarker", "appearances/protos/BrushedSteel",
               "appearances/protos/ScratchedPaint", "appearances/protos/RoughConcrete"]
    return '''#OMNISIM R2025a utf8
# Copyright 2026 OmniLink. SPDX-License-Identifier: Apache-2.0
# Generated by projects/robot_combat/orc/build_foundry.py. Edit the generator.
# Foundry uses the canonical sky/sun/marker with low, warm afternoon sun.
# 30 FPS is a conservative preview cap, not a measured performance result.
''' + '\n'.join(('IMPORTABLE ' if p.endswith('ScratchedPaint') else '') +
                f'EXTERNPROTO "omnisim://projects/{p}.proto"' for p in imports) + f'''
WorldInfo {{
 title "ORC / FOUNDRY" basicTimeStep 8 FPS 30
 newtonSolver "mujoco" newtonSubsteps 8
 newtonStatics TRUE newtonRobotColliders TRUE newtonCompoundColliders TRUE
 newtonGroundMu 0.45 newtonNjmax 1024 newtonNconmax 512
}}
DEF GAME_CAMERA Viewpoint {{ position {numbers(eye)}
 orientation {numbers(look_at(eye,(-.6,0,.35)))} fieldOfView 1.0
 near 0.06 far 180 exposure 1.0 ambientOcclusionRadius 1.2 bloomThreshold 4
}}
OmniSimSky {{ luminosity 1.4 }}
DEF SUN OmniSimSun {{ color 1 0.83 0.64 intensity 2.5 direction -0.65 0.4 -0.5 }}
DEF SUN_MARKER OmniSimSunMarker {{ translation 65 -40 50 radius 0.1 }}
{materials(match)}
{environment()}
{robot('ANVIL',(-3.1,0,.20),0,'TEAM_A' if match else 'ORANGE',settings=match['robots']['ANVIL'] if match else None)}
{robot('RAZOR',(3.1,0,.20),math.pi,'TEAM_B' if match else 'BLUE',True,settings=match['robots']['RAZOR'] if match else None)}
DEF DIRECTOR Robot {{ name "foundry_director" supervisor TRUE
 controller "orc_foundry_director"
 customData {json.dumps(json.dumps(director))}
}}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify generated world is current without writing")
    args = parser.parse_args()
    path = HERE / 'worlds/orc_foundry.omniworld'
    content = build()
    if args.check:
        if not path.exists() or path.read_text(encoding='utf-8') != content:
            raise SystemExit('Foundry world is out of date; run build_foundry.py')
        print('Foundry world matches its generator.')
    else:
        path.write_text(content, encoding='utf-8')
        print(f'Wrote {path} ({len(content):,} bytes).')


if __name__ == '__main__':
    main()
