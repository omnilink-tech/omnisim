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

"""Wheel-only follower. Coordination permission is the experimental variable."""
import json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'husky_extreme_terrain'))
from husky_extreme_terrain import find_link_by_name,clamp,wrap_pi
from omnisim import Supervisor

def main():
    r=Supervisor();dt=int(r.getBasicTimeStep());rid=r.getName()
    direction=1 if rid in ('A','C') else -1
    y0={'A':-2.3,'B':2.3,'C':-4.1,'D':4.1}[rid]
    # Arrivals and departures stay on their own side of the forecourt.
    route=[(-3.9*direction,y0),(-3.1*direction,0),(2.7*direction,0),
           (4.3*direction,y0),(6*direction,y0)]
    motors=[r.getDevice(n) for n in ('front_left_wheel_motor','rear_left_wheel_motor','front_right_wheel_motor','rear_right_wheel_motor')]
    for m in motors:m.setPosition(float('inf'));m.setVelocity(0)
    r.step(dt);r.step(dt)
    node=r.getSelf();pose=find_link_by_name(node,'base_link') or node
    shared=r.getFromDef('DIRECTOR').getField('customData')
    peers={}
    for other in 'ABCD':
        n=r.getFromDef('ROBOT_'+other)
        if n and other!=rid:peers[other]=find_link_by_name(n,'base_link') or n
    wp=0;done=False;next_publish=0;left=right=0
    while r.step(dt)!=-1:
        t=r.getTime();data=json.loads(shared.getSFString());p=pose.getPosition();rot=pose.getOrientation()
        yaw=math.atan2(rot[3],rot[0]);mode=data.get('mode','clear')
        if not done and math.hypot(p[0]-route[wp][0],p[1]-route[wp][1])<.35:
            if wp==len(route)-1:done=True
            else:wp+=1
        allowed=t>=data.get('start_delay',3) and not done and not data.get('halt',False)
        if mode in ('clear','early') and wp>=1:
            allowed=allowed and rid in data.get('admitted',[])
        if mode=='polite' and wp>=1:
            allowed=allowed and all(math.hypot(p[0]-q.getPosition()[0],p[1]-q.getPosition()[1])>=2.1 for q in peers.values())
        if allowed:
            tx,ty=route[wp];err=wrap_pi(math.atan2(ty-p[1],tx-p[0])-yaw)
            if abs(err)>.25:
                linear=.015;angular=clamp(err*3.2,-2,2)
            else:
                linear=.64*max(.5,math.cos(err));angular=clamp(err*2.6,-.9,.9)
            lt=clamp((linear-angular*.2854)/.1651,-6,6)
            rt=clamp((linear+angular*.2854)/.1651,-6,6)
        else:lt=rt=0
        # Slew limits preserve wheel/contact dynamics; never write a body pose.
        left+=clamp(lt-left,-.35,.35);right+=clamp(rt-right,-.35,.35)
        for m in motors[:2]:m.setVelocity(left)
        for m in motors[2:]:m.setVelocity(right)
        if t>=next_publish:
            node.getField('customData').setSFString(json.dumps({'id':rid,'wp':wp,'done':done,'waiting':not allowed,'left':left,'right':right}))
            next_publish=t+.08

if __name__=='__main__':main()
