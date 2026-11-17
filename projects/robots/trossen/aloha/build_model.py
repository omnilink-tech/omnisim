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
"""Convert the pinned ACT ViperX MJCF link descriptions into a native URDF.

Geometry and inertias are preserved; actuator effort and finger travel are
explicit simulation assumptions. No MJCF simulation is used for the demo.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
ASSETS=ROOT/'source/act/assets'
NAMES=['waist','shoulder','elbow','forearm_roll','wrist_angle','wrist_rotate','left_finger','right_finger']

def vec(s, default='0 0 0'):
    return np.array([float(x) for x in (s or default).split()])

def rotation(el):
    if 'quat' in el.attrib:
        q=vec(el.get('quat'))
        return Rotation.from_quat([*q[1:],q[0]])
    # MJCF's default eulerseq="xyz" uses intrinsic rotations; URDF RPY is extrinsic.
    return Rotation.from_euler('XYZ',vec(el.get('euler')))

def fmt(values): return ' '.join(f'{v:.10g}' for v in values)

def origin(parent, el):
    ET.SubElement(parent,'origin',xyz=el.get('pos','0 0 0'),rpy=fmt(rotation(el).as_euler('xyz')))

def key(name): return name.replace('vx300s_','').replace('/','_')

def build():
    import trimesh
    robot=ET.Element('robot',name='aloha_original')
    ET.SubElement(robot,'link',name='workcell')
    meshes={m.get('name'):m for m in ET.parse(ASSETS/'vx300s_dependencies.xml').getroot().findall('.//mesh')}
    (ROOT/'meshes').mkdir(exist_ok=True)
    for mesh in meshes.values():
        model=trimesh.load(ASSETS/mesh.get('file'),force='mesh')
        model.apply_scale(vec(mesh.get('scale'),'1 1 1'))
        model.export(ROOT/'meshes'/mesh.get('file'))
    def visit(body,parent):
        if body.get('name','').endswith('camera_focus'): return
        name=key(body.get('name'))
        link=ET.SubElement(robot,'link',name=name)
        j=body.find('joint')
        joint=ET.SubElement(robot,'joint',name=key(j.get('name')) if j is not None else name+'_fixed',
                            type=('prismatic' if j.get('type')=='slide' else 'revolute') if j is not None else 'fixed')
        ET.SubElement(joint,'parent',link=parent); ET.SubElement(joint,'child',link=name)
        origin(joint,body)
        if j is not None:
            ET.SubElement(joint,'axis',xyz=j.get('axis','0 0 1'))
            low,high=vec(j.get('range'))
            finger='finger' in j.get('name')
            if finger: low,high=(.005,.065) if 'left_finger' in name else (-.065,-.005)
            ET.SubElement(joint,'limit',lower=str(low),upper=str(high),effort='15' if finger else '80',velocity='.2' if finger else '5')
            initial={'waist':0,'shoulder':-.96,'elbow':1.16,'forearm_roll':0,'wrist_angle':-.3,'wrist_rotate':0,'left_finger':.02239,'right_finger':-.02239}[j.get('name').split('/')[-1]]
            ET.SubElement(joint,'dynamics',damping='0.05' if finger else '0.1')
            ET.SubElement(joint,'rest').text=str(initial)
        inertial=body.find('inertial')
        if inertial is not None:
            inertia=ET.SubElement(link,'inertial'); origin(inertia,inertial)
            ET.SubElement(inertia,'mass',value=inertial.get('mass'))
            xx,yy,zz=vec(inertial.get('diaginertia'))
            ET.SubElement(inertia,'inertia',ixx=str(xx),iyy=str(yy),izz=str(zz),ixy='0',ixz='0',iyz='0')
        else:
            inertia=ET.SubElement(link,'inertial'); ET.SubElement(inertia,'mass',value='0.1')
            ET.SubElement(inertia,'inertia',ixx='.0001',iyy='.0001',izz='.0001',ixy='0',ixz='0',iyz='0')
        for g in body.findall('geom'):
            mesh=meshes[g.get('mesh')]
            for kind in ['visual','collision']:
                # Upstream base collision is disabled.
                if kind=='collision' and g.get('contype')=='0': continue
                if kind=='collision' and name.endswith('_finger_link'):
                    # Whole-mesh convexification bridges the shaped jaw face.
                    # Slice source triangles in the finger's link coordinates.
                    model=trimesh.load(ASSETS/mesh.get('file'),force='mesh')
                    model.apply_scale(vec(mesh.get('scale'),'1 1 1'))
                    T=np.eye(4);T[:3,:3]=rotation(g).as_matrix();T[:3,3]=vec(g.get('pos'))
                    model.apply_transform(T)
                    vertices=model.vertices;ends=vertices[model.edges_unique]
                    cuts=[-1,.04,.06,.073,.079,1]
                    for i,(lo,hi) in enumerate(zip(cuts,cuts[1:])):
                        points=[vertices[(vertices[:,0]>=lo)&(vertices[:,0]<=hi)]]
                        for cut in (lo,hi):
                            mask=((ends[:,0,0]<cut)&(ends[:,1,0]>cut))|((ends[:,1,0]<cut)&(ends[:,0,0]>cut))
                            e=ends[mask];t=(cut-e[:,0,0])/(e[:,1,0]-e[:,0,0])
                            points.append(e[:,0]+t[:,None]*(e[:,1]-e[:,0]))
                        hull=trimesh.convex.convex_hull(np.concatenate(points))
                        filename=f'meshes/{name}_collision_{i}.stl';hull.export(ROOT/filename)
                        node=ET.SubElement(link,'collision');geom=ET.SubElement(node,'geometry')
                        ET.SubElement(geom,'mesh',filename=filename)
                    continue
                node=ET.SubElement(link,kind); origin(node,g)
                geom=ET.SubElement(node,'geometry')
                ET.SubElement(geom,'mesh',filename='meshes/'+mesh.get('file'))
                if kind=='visual':
                    mat=ET.SubElement(node,'material',name='white_fingers' if 'finger' in name else 'black_robot')
                    ET.SubElement(mat,'color',rgba='.92 .92 .89 1' if 'finger' in name else '.065 .07 .075 1')
        if name.endswith('_finger_link'):
            # Explicit simulation addition: a thin, flat contact pad over the
            # tapered jaw. Both collision and visible geometry use this shape.
            y=-.019 if 'left_finger_link' in name else .019
            for kind in ('visual','collision'):
                node=ET.SubElement(link,kind)
                ET.SubElement(node,'origin',xyz=f'.078 {y} 0',rpy='0 0 0')
                geom=ET.SubElement(node,'geometry');ET.SubElement(geom,'box',size='.014 .004 .012')
                if kind=='visual':
                    material=ET.SubElement(node,'material',name='estimated_contact_pad')
                    ET.SubElement(material,'color',rgba='.08 .08 .08 1')
        for child in body.findall('body'): visit(child,name)
    for side in ['left','right']:
        body=ET.parse(ASSETS/f'vx300s_{side}.xml').getroot().find('body')
        body.set('pos',fmt(vec(body.get('pos'))-[0,.5,0]))
        visit(body,'workcell')
    ET.indent(robot)
    import copy
    # Two independently anchored robots avoid a dynamic fixed-joint branch below
    # a synthetic workcell link. Each has the original arm base as its root.
    for side in ['left','right']:
        arm=ET.Element('robot',name='aloha_'+side)
        for el in robot:
            name=el.get('name','')
            if (name==side or name.startswith(side+'_')) and name!=side+'_fixed':arm.append(copy.deepcopy(el))
        ET.indent(arm)
        ET.ElementTree(arm).write(ROOT/f'aloha_{side}.urdf',encoding='utf-8',xml_declaration=True)

def fk(values, side):
    body=ET.parse(ASSETS/f'vx300s_{side}.xml').getroot().find('body')
    outputs={}; index=0
    def walk(b,T):
        nonlocal index
        A=np.eye(4); A[:3,:3]=rotation(b).as_matrix(); A[:3,3]=vec(b.get('pos'))
        if b is body: A[1,3]-=.5
        T=T@A
        j=b.find('joint')
        if j is not None:
            q=values[index]; index+=1
            J=np.eye(4)
            if j.get('type')=='slide': J[:3,3]=vec(j.get('axis'))*q
            else: J[:3,:3]=Rotation.from_rotvec(vec(j.get('axis'))*q).as_matrix()
            T=T@J
        outputs[key(b.get('name'))]=T
        for child in b.findall('body'):
            if not child.get('name').endswith('camera_focus'): walk(child,T)
    walk(body,np.eye(4))
    return outputs

if __name__=='__main__': build()
