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
# limitations under the License."""Contact-mechanism reconstruction: grasp near one end, compress, pivot, release."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from author_motion import Arm, ROOT


def build():
    episode=json.loads((ROOT/'episode.json').read_text())
    left,right=Arm('left'),Arm('right');initial=np.array(episode['reference'][0])
    vertical=np.array([[0.,0.,-1.],[0.,-1.,0.],[-1.,0.,0.]])
    remote=np.array([-.058,.043,.010]);pickup=np.array([.069,.016,.0168])
    angle=np.deg2rad(25);grasp_offset=.019
    def contact_pose(x,theta):
        axis=np.array([np.cos(theta),0,np.sin(theta)])
        # x is the contacting surface, including the radial extent of the
        # tilted flat end. Its centre alone underestimates compression.
        return remote+np.array([x+.0068*np.sin(theta),-.0095,.0128])+.0245*axis
    approach=contact_pose(.002,angle);compressed=contact_pose(-.0085,angle)
    seated=contact_pose(-.0085,np.deg2rad(18))
    waypoints=[
        (0,pickup+[0,0,.23],0,.040,'approach'),
        (1.2,pickup+[0,0,.08],0,.040,'approach'),
        (2.6,pickup,0,.040,'grasp'),(3.4,pickup,0,.032,'grasp'),
        (4.0,pickup,0,.032,'grasp'),(5.8,pickup+[0,0,.08],0,.032,'carry'),
        (8.4,approach+[0,0,.025],angle,.032,'align'),
        (9.8,approach,angle,.032,'engage_spring'),
        (11.6,compressed,angle,.032,'compress'),
        (13.0,seated,np.deg2rad(18),.032,'seat'),
        (13.6,seated,np.deg2rad(18),.047,'release'),
        (14.8,seated+[0,0,.06],0,.047,'withdraw')]
    # After releasing the tilted battery, use the empty jaw tips to press the
    # raised positive end down and toward the spring, as in the reference.
    press_points=[(14.8,[.045,-.0095,.080],.047,'press_approach'),
                  (16.2,[.045,-.0095,.080],.0285,'press_approach'),
                  (17.5,[.045,-.0095,.037],.0285,'press_approach'),
                  (20.0,[.034,-.0095,.0215],.0285,'press'),
                  (21.0,[.034,-.0095,.0215],.0285,'press'),
                  (23.0,[.045,-.0095,.100],.0285,'press_withdraw'),
                  (25.0,[.120,-.0095,.160],.040,'press_withdraw'),
                  (28.0,[.120,-.0095,.160],.040,'inspect')]
    q=initial[8:14];targets=[];stages=[];poses=[]
    left_R=left.fk(episode['reference'][225][:6])[:3,:3]
    left_goal=left.solve(np.array([-.108,.043,.022]),left_R,np.array(episode['reference'][225][:6]),tcp=.1467)
    for k in range(1400):
        t=k/50
        tool_pose=None
        for a,b in zip(waypoints,waypoints[1:]):
            ta,pa,aa,ga,sa=a;tb,pb,ab,gb,sb=b
            if ta<=t<=tb:
                u=(t-ta)/(tb-ta);s=u**3*(10-15*u+6*u*u)
                p=pa+s*(pb-pa);theta=aa+s*(ab-aa);g=ga+s*(gb-ga);stage=sb;break
        if t>=14.8:
            for a,b in zip(press_points,press_points[1:]):
                ta,pa,ga,sa=a;tb,pb,gb,sb=b
                if ta<=t<=tb:
                    u=(t-ta)/(tb-ta);s=u**3*(10-15*u+6*u*u)
                    tip=remote+np.array(pa)+s*(np.array(pb)-pa);g=ga+s*(gb-ga);stage=sb;break
            R=Rotation.from_rotvec([0,np.deg2rad(20),0]).as_matrix()@vertical
            q=right.solve(tip,R,q,tcp=.154)
            tool_pose={'tip_in_remote':(tip-remote).tolist(),'rotation_in_remote':R.tolist(),'tcp_m':.154}
        else:
            axis=np.array([np.cos(theta),0,np.sin(theta)])
            R=Rotation.from_rotvec([0,-theta,0]).as_matrix()@vertical
            tip=p+grasp_offset*axis
            q=right.solve(tip,R,q,tcp=.147)
        u=np.clip((t-2.5)/1.5,0,1);s=u*u*(3-2*u);ql=initial[:6]+s*(left_goal-initial[:6])
        if t>=23.0:
            u=np.clip((t-23.0)/2.2,0,1);s=u**3*(10-15*u+6*u*u);ql=left_goal+s*(initial[:6]-left_goal)
        closing=np.clip((t-3.5)/.7,0,1);left_g=.058-.012*closing**2*(3-2*closing)
        opening=np.clip((t-22.2)/.6,0,1);left_g+=.012*opening**2*(3-2*opening)
        targets.append([*ql,left_g,-left_g,*q,g,-g]);stages.append(stage)
        poses.append({'battery_center':p.tolist(),'battery_tilt_rad':float(theta),'tcp':tip.tolist(),
                      'tool_pose':tool_pose})
    cfg={**episode,'targets':targets,'motion_mode':'spring-insertion','spring_compartment':True,
         'reference':[episode['reference'][min(k,599)] for k in range(1400)],'duration_s':28,
         'battery_position':pickup.tolist(),'remote_position':remote.tolist(),'table_height':.010,
         'stages':stages,'desired_battery_poses':poses,
         'authored_parameters':{'grasp_offset_m':grasp_offset,'tool_pitch_deg':0,'insertion_tilt_deg':25,
         'description':'Estimated compress-seat-release motor trajectory; no object pose actuation'}}
    (ROOT/'spring_motion.json').write_text(json.dumps(cfg),encoding='utf-8')
    print('Wrote 1400 spring insertion targets')


if __name__=='__main__':build()
