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
# limitations under the License."""Estimated, mechanically active battery compartment; dimensions in metres."""
import math

# These are declared reconstruction parameters, not measurements of the filmed
# remote. The contact topology follows a negative spring / positive button pair.
PARAMETERS = {
    'stiffness_n_m': 100.0, 'damping_n_s_m': .18, 'preload_n': .15,
    'travel_m': .010, 'contact_mass_kg': .002, 'coil_rest_length_m': .014,
    'spring_rest_face_x': .0005, 'positive_face_x': .0435,
    'slot_y': -.0095, 'battery_axis_z': .0128,
    'battery_length_m': .049, 'battery_radius_m': .0068,
}


def spring_force(q, velocity, parameters=PARAMETERS):
    """Force in remote +X, with compression represented by negative q."""
    return parameters['preload_n'] - parameters['stiffness_n_m'] * q - parameters['damping_n_s_m'] * velocity


def remote_node():
    from build_scene import box
    # Open pocket down to the floor; the positive button and lip must be cleared
    # before the spring can push the seated battery against the positive contact.
    parts = [((0,0,.003),(.134,.041,.006)), ((-.041,0,.0095),(.052,.041,.019)),
             ((0,.019,.011),(.134,.003,.016)), ((0,-.019,.011),(.134,.003,.016)),
             ((-.065,0,.011),(.004,.038,.016)), ((.065,0,.011),(.004,.038,.016)),
             ((0,0,.011),(.134,.002,.012)), ((.046,0,.010),(.003,.038,.012)),
             # A retaining lip at the positive end, above the battery end.
             ((.0455,-.0095,.021),(.005,.016,.003))]
    visuals='\n'.join(box(p,s,'.92 .92 .87') for p,s in parts)
    colliders='\n'.join(box(p,s,'',True) for p,s in parts)
    colliders+='\n'+box((.0445,-.0095,.0128),(.002,.010,.014),'',True)
    # The coil is a visible, massless indicator of the lumped spring's length.
    # Only its x scale is updated; no physical body pose is prescribed.
    points=[];faces=[];rings=8
    for i in range(121):
        u=i/120;angle=2*math.pi*5*u
        normal=[0,math.cos(angle),math.sin(angle)]
        tangent=[.014,-.003*10*math.pi*math.sin(angle),.003*10*math.pi*math.cos(angle)]
        length=math.sqrt(sum(x*x for x in tangent));tangent=[x/length for x in tangent]
        binormal=[tangent[1]*normal[2]-tangent[2]*normal[1],
                  tangent[2]*normal[0],-tangent[1]*normal[0]+tangent[0]*normal[1]]
        center=[.014*u,.003*math.cos(angle),.003*math.sin(angle)]
        for j in range(rings):
            phase=2*math.pi*j/rings
            p=[center[k]+.00035*(normal[k]*math.cos(phase)+binormal[k]*math.sin(phase)) for k in range(3)]
            points.append(' '.join(f'{x:.8f}' for x in p))
        if i:
            for j in range(rings):
                a=(i-1)*rings+j;b=(i-1)*rings+(j+1)%rings
                faces.append(f'{a} {b} {b+rings} {a+rings} -1')
    indices=' '.join(faces)
    return f'''DEF REMOTE Solid {{
 translation -.058 .043 .010
 name "remote"
 newtonFriction .3
 children [
 {visuals}
 Pose {{ translation .0175 .0095 .0128 rotation 0 1 0 1.57079632679 children [ Shape {{ appearance PBRAppearance {{ baseColor .1 .1 .1 roughness .4 metalness .4 }} geometry Cylinder {{ radius .0065 height .05 }} }} ] }}
 DEF POSITIVE_TERMINAL Solid {{
   translation .0445 -0.0095 .0128 name "positive_terminal"
   children [ Shape {{ appearance PBRAppearance {{ baseColor .72 .74 .78 roughness .35 metalness .8 }} geometry Box {{ size .002 .010 .014 }} }} ]
 }}
 Pose {{ translation -.0141 -.0095 .0128 children [
   DEF SPRING_COIL Transform {{ scale 1.0001 1 1 children [ Shape {{
     appearance PBRAppearance {{ baseColor .65 .68 .72 roughness .3 metalness .8 }}
     geometry IndexedFaceSet {{ coord Coordinate {{ point [ {', '.join(points)} ] }} coordIndex [ {indices} ] }}
   }} ] }}
 ] }}
 SliderJoint {{
   jointParameters DEF SPRING_JOINT JointParameters {{ axis 1 0 0 minStop -.010 maxStop 0 }}
   endPoint DEF NEGATIVE_TERMINAL Solid {{
     translation -.0001 -.0095 .0128 name "negative_terminal" newtonFriction .3
     children [ Shape {{ appearance PBRAppearance {{ baseColor .72 .74 .78 roughness .35 metalness .8 }} geometry Box {{ size .0012 .012 .013 }} }} ]
     boundingObject Box {{ size .0012 .012 .013 }}
     physics Physics {{ density -1 mass .002 centerOfMass [ 0 0 0 ] inertiaMatrix [ .0000000522 .0000000284 .0000000242 0 0 0 ] }}
   }}
 }}
 ]
 boundingObject Group {{ children [ {colliders} ] }}
 physics Physics {{ density -1 mass .055 }}
}}
'''


def spring_controller_node():
    return '''DEF COMPARTMENT_CONTROLLER Robot {
 name "battery_compartment_controller" supervisor TRUE
 controller "battery_compartment"
}
'''


def on_positive_contact(point):
    """A real remote/battery contact within the fixed button's contact face."""
    return abs(point[0]-.0435)<.001 and abs(point[1]+.0095)<.0055 and abs(point[2]-.0128)<.0075


def on_floor_contact(point):
    return -.0095<point[0]<.041 and -.0175<point[1]<-.001 and abs(point[2]-.006)<.001
