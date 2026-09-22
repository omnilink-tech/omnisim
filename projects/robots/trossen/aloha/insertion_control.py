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
# limitations under the License."""Object-pose feedback converted to arm motor targets; no object pose writes."""
import math
from kinematics import Arm,add,sub,scale,mv,mm,matrix,transpose,rotation,rotvec,limited


class InsertionControl:
    def __init__(self,cfg,battery,remote,grip,spring_joint):
        self.cfg=cfg;self.battery=battery;self.remote=remote;self.grip=grip
        self.arm=Arm('right');self.relative=None;self.relative_rotation=None
        self.previous_position=None;self.previous_rotation=None;self.q=None
        self.position_offset=None;self.orientation_offset=None
        self.spring_joint=spring_joint;self.relief=0.0

    def command(self,k,nominal):
        if k<290:return nominal
        B=matrix(self.battery.getOrientation());G=matrix(self.grip.getOrientation())
        bp=self.battery.getPosition();gp=self.grip.getPosition()
        measured=mv(transpose(G),sub(bp,gp))
        if self.relative is None:
            self.relative=measured;self.relative_rotation=mm(transpose(B),G)
            self.previous_position=gp;self.previous_rotation=G;self.q=nominal[:6]
        elif self.cfg['stages'][k] in ('align','engage_spring','compress','seat'):
            # Correct small sliding within a still-closed grasp. The desired
            # battery path remains independent of the measured battery pose.
            delta=[max(-.001,min(.001,x)) for x in sub(measured,self.relative)]
            self.relative=add(self.relative,scale(delta,.15))
        desired=self.cfg['desired_battery_poses'][k]
        R=matrix(self.remote.getOrientation())
        p=add(self.remote.getPosition(),mv(R,sub(desired['battery_center'],self.cfg['remote_position'])))
        # Leave travel in reserve instead of driving into the terminal stop.
        compression=-self.spring_joint.getSFFloat()
        limit=.008 if self.cfg['stages'][k]=='press' else .009
        wanted=min(.012,max(0,compression-limit)*6) if self.cfg['stages'][k] in ('compress','seat','press') else 0
        self.relief += .3*(wanted-self.relief)
        p=add(p,mv(R,[self.relief,0,0]))
        target_B=mm(R,rotation([0,math.pi/2-desired['battery_tilt_rad'],0]))
        if self.position_offset is None:
            self.position_offset=sub(bp,p)
            self.orientation_offset=rotvec(mm(B,transpose(target_B)))
        # Enter feedback continuously: remove the measured setup discrepancy
        # with a one-second minimum-jerk blend, not a step in arm velocity.
        u=max(0,min(1,(k-290)/60));fade=1-u**3*(10-15*u+6*u*u)
        p=add(p,scale(self.position_offset,fade))
        target_B=mm(rotation(scale(self.orientation_offset,fade)),target_B)
        target_G=mm(target_B,self.relative_rotation);target_p=sub(p,mv(target_G,self.relative))
        if desired.get('tool_pose'):
            tool=desired['tool_pose'];target_G=mm(R,tool['rotation_in_remote'])
            tip=add(self.remote.getPosition(),mv(R,add(tool['tip_in_remote'],[self.relief,0,0])))
            target_p=sub(tip,mv(target_G,[tool['tcp_m'],0,0]))
        target_p=add(self.previous_position,limited(sub(target_p,self.previous_position),.002))
        rot=limited(rotvec(mm(target_G,transpose(self.previous_rotation))),.012)
        target_G=mm(rotation(rot),self.previous_rotation)
        self.q=self.arm.solve(target_p,target_G,self.q)
        self.previous_position=target_p;self.previous_rotation=target_G
        return [*self.q,*nominal[6:]]
