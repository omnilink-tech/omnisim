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
# limitations under the License.# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
"""Author the explicitly estimated battery/remote scene in OmniSim format."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[3]/'src/python'))
from omniworld.viewpoint import look_at

def box(pos,size,color,collision=False):
    xyz=lambda v:' '.join(map(str,v))
    geom=f'Box {{ size {xyz(size)} }}'
    child=geom if collision else f'Shape {{ appearance PBRAppearance {{ baseColor {color} roughness .65 metalness 0 }} geometry {geom} }}'
    return f'Pose {{ translation {xyz(pos)} children [ {child} ] }}'

def build():
    # Reconstructed from reference frames and forward kinematics, not measured CAD.
    parts=[((0,0,.003),(.134,.041,.006)),((-.0375,0,.0095),(.052,.041,.019)),((0,.019,.011),(.134,.003,.016)),
           ((0,-.019,.011),(.134,.003,.016)),((-.065,0,.011),(.004,.038,.016)),
           ((.065,0,.011),(.004,.038,.016)),((0,0,.011),(.134,.002,.012)),
           ((-.011,0,.010),(.003,.038,.012)),((.046,0,.010),(.003,.038,.012))]
    # Slot centre is body x=.0175, y=-.0095, with 54 mm clear length.
    visuals='\n'.join(box(p,s,'.92 .92 .87') for p,s in parts)
    colliders='\n'.join(box(p,s,'',True) for p,s in parts)
    scene='''#OMNISIM R2025a utf8
# ALOHA battery episode. Scene dimensions and dynamics are estimates.
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
WorldInfo {
  title "ALOHA - recorded battery insertion"
  basicTimeStep 2
  newtonSolver "mujoco"
  newtonCompoundColliders TRUE
  newtonGroundMu 3
  newtonCone "elliptic"
  newtonImpratio 10
  newtonNoslipIterations 5
  newtonCondim 6
}
Viewpoint { position .01 -.12 1.08 orientation CAMERA_ORIENTATION fieldOfView 1.15 exposure .85 }
OmniSimSky { }
DEF SUN OmniSimSun { }
DEF SUN_MARKER OmniSimSunMarker { }
DEF TABLE Solid {
 translation 0 0 -.025
 name "table"
 children [ Shape { appearance PBRAppearance { baseColor .08 .09 .10 roughness .72 metalness 0 } geometry Box { size 1.5 1 .05 } } ]
 boundingObject Box { size 1.5 1 .05 }
}
DEF REMOTE Solid {
 translation -.058 .043 0
 name "remote"
 children [
 REMOTE_VISUALS
 # The already-installed second battery is visual geometry fixed in the remote.
 Pose { translation .0175 .0095 .0128 rotation 0 1 0 1.57079632679 children [ Shape { appearance PBRAppearance { baseColor .1 .1 .1 roughness .4 metalness .4 } geometry Cylinder { radius .0065 height .05 } } ] }
 ]
 boundingObject Group { children [ REMOTE_COLLIDERS ] }
 physics Physics { density -1 mass .055 }
}
DEF BATTERY Solid {
 translation .085 .013 .007
 rotation 0 1 0 1.57079632679
 name "battery"
 newtonFrictionRolling .0001
 children [
 Shape { appearance PBRAppearance { baseColor .06 .065 .07 roughness .35 metalness .3 } geometry Cylinder { radius .0068 height .049 } }
 Pose { translation 0 0 .0195 children [ Shape { appearance PBRAppearance { baseColor .68 .44 .16 roughness .3 metalness .7 } geometry Cylinder { radius .0069 height .010 } } ] }
 ]
 boundingObject Pose { children [ Cylinder { radius .0068 height .049 } ] }
 physics Physics {
   density -1 mass .023 centerOfMass [ 0 0 0 ]
   inertiaMatrix [ .000004867 .000004867 .00000053176 0 0 0 ]
 }
}
DEF ALOHA_LEFT URDFRobot {
 url "../aloha_left.urdf"
 translation -.469 0 0
 name "aloha_left"
 staticBase TRUE
 supervisor TRUE
 controller "aloha_replay"
}
DEF ALOHA_RIGHT URDFRobot {
 url "../aloha_right.urdf"
 translation .469 0 0
 rotation 0 0 1 3.14159265359
 name "aloha_right"
 staticBase TRUE
 supervisor TRUE
 controller "aloha_replay"
}
'''.replace('REMOTE_VISUALS',visuals).replace('REMOTE_COLLIDERS',colliders).replace('CAMERA_ORIENTATION',' '.join(map(str,look_at((.01,-.12,1.08),(.01,0,.07)))))
    (ROOT/'worlds').mkdir(exist_ok=True)
    (ROOT/'worlds/aloha_battery.omniworld').write_text(scene)

if __name__=='__main__': build()
