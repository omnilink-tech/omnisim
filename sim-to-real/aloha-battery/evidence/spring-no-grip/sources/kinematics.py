# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Standard-library URDF arm solver for the isolated controller runtime."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

I=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def scale(a,s):return [x*s for x in a]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def norm(a):return math.sqrt(dot(a,a))
def transpose(a):return [list(row) for row in zip(*a)]
def mv(a,b):return [dot(row,b) for row in a]
def mm(a,b):return [[dot(row,col) for col in zip(*b)] for row in a]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def matrix(values):return [list(values[i:i+3]) for i in (0,3,6)]
def limited(v,maximum):return scale(v,min(1,maximum/max(norm(v),1e-15)))


def rotation(v):
    angle=norm(v)
    if angle<1e-12:return [r[:] for r in I]
    x,y,z=scale(v,1/angle);c=math.cos(angle);s=math.sin(angle);d=1-c
    return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],
            [y*x*d+z*s,c+y*y*d,y*z*d-x*s],
            [z*x*d-y*s,z*y*d+x*s,c+z*z*d]]


def rotvec(a):
    angle=math.acos(max(-1,min(1,(sum(a[i][i] for i in range(3))-1)/2)))
    v=[a[2][1]-a[1][2],a[0][2]-a[2][0],a[1][0]-a[0][1]]
    if angle<1e-7:return scale(v,.5)
    if math.pi-angle<1e-6:
        axis=[math.sqrt(max(0,(a[i][i]+1)/2)) for i in range(3)]
        pivot=max(range(3),key=lambda i:axis[i])
        for j in range(3):
            if j!=pivot:axis[j]=math.copysign(axis[j],a[pivot][j]+a[j][pivot])
        return scale(axis,angle)
    return scale(v,angle/(2*math.sin(angle)))


def linear_solve(a,b):
    rows=[list(row)+[rhs] for row,rhs in zip(a,b)];n=len(rows)
    for j in range(n):
        pivot=max(range(j,n),key=lambda i:abs(rows[i][j]))
        rows[j],rows[pivot]=rows[pivot],rows[j]
        if abs(rows[j][j])<1e-14:raise ValueError('Singular arm Jacobian')
        rows[j]=scale(rows[j],1/rows[j][j])
        for i in range(n):
            if i!=j:rows[i]=sub(rows[i],scale(rows[j],rows[i][j]))
    return [row[-1] for row in rows]


class Arm:
    def __init__(self,side='right'):
        model=ET.parse(Path(__file__).parent/f'aloha_{side}.urdf').getroot()
        self.base=[-.469 if side=='left' else .469,0,0]
        self.base_rotation=I if side=='left' else rotation([0,0,math.pi])
        self.chain=[];self.limits=[]
        for name in ['waist','shoulder','elbow','forearm_roll','wrist_angle','wrist_rotate']:
            joint=model.find(f'joint[@name="{side}_{name}"]');o=joint.find('origin')
            xyz=list(map(float,o.get('xyz').split()));r,p,y=map(float,o.get('rpy').split())
            R=mm(rotation([0,0,y]),mm(rotation([0,p,0]),rotation([r,0,0])))
            axis=list(map(float,joint.find('axis').get('xyz').split()))
            self.chain.append((xyz,R,axis));lim=joint.find('limit')
            self.limits.append((float(lim.get('lower')),float(lim.get('upper'))))

    def fk(self,q):
        p=self.base[:];R=self.base_rotation;origins=[];axes=[]
        for value,(xyz,origin_R,axis) in zip(q,self.chain):
            p=add(p,mv(R,xyz));R=mm(R,origin_R)
            origins.append(p);axes.append(mv(R,axis));R=mm(R,rotation(scale(axis,value)))
        return p,R,origins,axes

    def solve(self,point,orientation,guess,tcp=0):
        q=list(guess)
        for _ in range(45):
            p,R,origins,axes=self.fk(q);p=add(p,mv(R,[tcp,0,0]))
            ep=sub(point,p);er=rotvec(mm(orientation,transpose(R)))
            if norm(ep)<.00008 and norm(er)<.0007:return q
            columns=[cross(a,sub(p,o))+scale(a,.15) for a,o in zip(axes,origins)]
            error=ep+scale(er,.15)
            a=[[dot(c,d)+(1e-7 if i==j else 0) for j,d in enumerate(columns)] for i,c in enumerate(columns)]
            dq=linear_solve(a,[dot(c,error) for c in columns])
            largest=max(abs(x) for x in dq)
            if largest>.12:dq=scale(dq,.12/largest)
            q=[max(lo+1e-7,min(hi-1e-7,v+d)) for v,d,(lo,hi) in zip(q,dq,self.limits)]
        raise ValueError(f'Unreachable feedback pose: translation error {norm(ep):.5f} m, rotation {norm(er):.5f} rad')
