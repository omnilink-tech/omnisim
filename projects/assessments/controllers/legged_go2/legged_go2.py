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

"""Reference Go2 controller with an observed-foot-contact handover guard.

The original policy, gait, motors and blending are unchanged. This wrapper only
checks whether a scheduled transition may start. No body/pose writes or forces.
"""
import math
import os
from pathlib import Path
import runpy
from handover_guard import support_ready

ROOT=Path(__file__).resolve().parents[4]
state={}
namespace=runpy.run_path(str(ROOT/'projects/policies/research/controllers/go2_baton_deploy/go2_baton_deploy.py'),run_name='assessment_reference')
main=namespace['main']
g=main.__globals__
original_robot=g['_Robot']


def robot_factory():
    robot=original_robot()
    state['robot']=robot
    return robot


original_gate=g['Go2BatonHost'].support_gate


def guarded_support(self,gait):
    phase_gate=original_gate(self,gait)
    names={}
    def measured_gate(phase,tol=.15):
        if not phase_gate(phase,tol):return False
        if os.environ.get('GO2_ASSESS_GATE','contact')=='phase':return True
        robot=state['robot'];node=robot.getSelf()
        contacts=set()
        for point in node.getContactPoints(True):
            if point.node_id not in names:
                field=robot.getFromId(point.node_id).getField('name')
                names[point.node_id]=field.getSFString() if field else ''
            contacts.add(names[point.node_id])
        o=node.getOrientation()
        if not all(math.isfinite(value) for value in o):return False
        tilt=math.acos(max(-1.,min(1.,o[8])))
        limit=.12 if os.environ.get('GO2_ASSESS_GATE')=='level' else .25
        allowed=support_ready(contacts,tilt,limit)
        if allowed:self.log(f'CONTACT_SUPPORT t={robot.getTime():.3f}s feet=4 tilt={tilt:.4f} limit={limit:.2f}')
        return allowed
    return measured_gate


g['_Robot']=robot_factory
g['Go2BatonHost'].support_gate=guarded_support
original_step=g['BATON'].step


def measured_step(host,st,*args,**kwargs):
    before=st.switches
    result=original_step(host,st,*args,**kwargs)
    if st.switches!=before:
        host.log(f'HANDOVER mode={st.target} sim_time_s={state["robot"].getTime():.3f}')
    return result


g['BATON'].step=measured_step
if __name__=='__main__':raise SystemExit(main())
