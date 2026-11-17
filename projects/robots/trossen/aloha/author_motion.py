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
# limitations under the License."""Authored Cartesian reconstruction of the battery task, not recorded actions."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
NAMES = ['waist','shoulder','elbow','forearm_roll','wrist_angle','wrist_rotate']


class Arm:
    def __init__(self, side):
        model=ET.parse(ROOT/f'aloha_{side}.urdf').getroot()
        self.base=np.eye(4);self.base[0,3]=-.469 if side=='left' else .469
        if side=='right':self.base[:3,:3]=Rotation.from_rotvec([0,0,np.pi]).as_matrix()
        self.chain=[]; self.lo=[];self.hi=[]
        for name in NAMES:
            joint=model.find(f'joint[@name="{side}_{name}"]');origin=joint.find('origin')
            T=np.eye(4);T[:3,3]=np.fromstring(origin.get('xyz'),sep=' ')
            T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
            self.chain.append((T,np.fromstring(joint.find('axis').get('xyz'),sep=' ')))
            limit=joint.find('limit');self.lo.append(float(limit.get('lower')));self.hi.append(float(limit.get('upper')))

    def fk(self,q):
        T=self.base.copy()
        for value,(origin,axis) in zip(q,self.chain):
            A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix();T=T@origin@A
        return T

    def solve(self, point, orientation, guess, tcp=.151):
        def error(q):
            T=self.fk(q);p=T[:3,3]+T[:3,:3]@np.array([tcp,0,0])
            return np.r_[p-point,.15*Rotation.from_matrix(orientation@T[:3,:3].T).as_rotvec()]
        fit=least_squares(error,guess,bounds=(self.lo,self.hi),xtol=1e-9,gtol=1e-9,ftol=1e-9,max_nfev=80)
        if np.linalg.norm(error(fit.x)[:3])>.0005:raise ValueError(f'Unreachable point {point}: {error(fit.x)}')
        return fit.x


def build():
    episode=json.loads((ROOT/'episode.json').read_text())
    left,right=Arm('left'),Arm('right')
    initial=np.array(episode['reference'][0]); grasp=np.array(episode['reference'][150])
    orientation=np.array([[0.,0.,-1.],[0.,-1.,0.],[-1.,0.,0.]])
    # Explicit estimates: scene, timing, grip command, and Cartesian paths.
    table=.010; pickup=np.array([.085,.016,table+.0068]);drop=np.array([-.0405,.0335,table+.027])
    waypoints=[(0,pickup+[0,0,.23],.040),(1.2,pickup+[0,0,.08],.040),
               (2.6,pickup,.040),(3.2,pickup,.023),(3.7,pickup,.023),
               (5.0,pickup+[0,0,.08],.023),(6.6,drop+[0,0,.06],.023),
               (8.0,drop,.023),(8.4,drop,.023),(8.9,drop,.041),
               (9.4,drop,.041),(10.7,drop+[0,0,.10],.041),(12,drop+[.10,0,.17],.041)]
    targets=[];q=grasp[8:14]
    # Left hand approaches the end of the remote with its closed fingers.
    left_start=left.fk(initial[:6]);left_R=left.fk(episode['reference'][225][:6])[:3,:3]
    left_goal=left.solve(np.array([-.126,.043,table+.014]),left_R,np.array(episode['reference'][225][:6]),tcp=.152)
    for k in range(600):
        t=k/50
        for (ta,pa,ga),(tb,pb,gb) in zip(waypoints,waypoints[1:]):
            if ta<=t<=tb:
                u=(t-ta)/(tb-ta);s=u*u*u*(10-15*u+6*u*u);p=pa+s*(pb-pa);g=ga+s*(gb-ga);break
        q=right.solve(p,orientation,q)
        u=np.clip((t-2.5)/1.5,0,1);s=u*u*(3-2*u);ql=initial[:6]+s*(left_goal-initial[:6])
        if t>=9.2:
            u=np.clip((t-9.2)/2.0,0,1);s=u*u*u*(10-15*u+6*u*u)
            ql=left_goal+s*(initial[:6]-left_goal)
        targets.append([*ql,.016,-.016,*q,g,-g])
    cfg={**episode,'targets':targets,'motion_mode':'authored simulation',
         'table_height':table,'battery_position':pickup.tolist(),'remote_position':[-.058,.043,table],
         'authored_parameters':{'tcp_m':.151,'grip_m':.023,'drop_centre_m':drop.tolist(),
                                'source':'Task-inspired IK trajectory; not a dataset action replay'}}
    (ROOT/'authored_motion.json').write_text(json.dumps(cfg))
    print('Wrote 600 authored motor-target frames')


if __name__=='__main__':build()


