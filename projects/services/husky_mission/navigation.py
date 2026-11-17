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

"""Small pose-feedback waypoint controller; all actuation remains wheel driven."""
import math


def clamp(x,lo,hi):return max(lo,min(hi,x))


class Waypoint:
    def __init__(self,goal,sim_time):
        self.goal=goal;self.started=sim_time;self.braking=False;self.settled_since=None;self.result=None

    def update(self,state):
        x,y=self.goal;dx=x-state['x'];dy=y-state['y'];distance=math.hypot(dx,dy)
        if state['sim_time']-self.started>60:
            self.result={'arrived':False,'refused':'waypoint_deadline'};return 0.,0.
        if distance<.05:self.braking=True
        if self.braking and distance>.12:self.braking=False
        if self.braking:
            if state['measured_speed_m_s']<.025:
                if self.settled_since is None:self.settled_since=state['sim_time']
                if state['sim_time']-self.settled_since>=.4:
                    self.result={'arrived':True,'achieved_xy':[state['x'],state['y']],'error_m':distance,'settled':True}
            else:self.settled_since=None
            return 0.,0.
        heading=math.atan2(dy,dx);error=math.atan2(math.sin(heading-state['yaw']),math.cos(heading-state['yaw']))
        linear=min(.25,.7*distance)*max(0.,math.cos(error)) if abs(error)<.65 else 0.
        angular=clamp(1.5*error,-.5,.5)
        return linear,angular
