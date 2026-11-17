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

"""Controlled proposal cohort: deliberately bad tools, not a model error rate."""
import argparse
import json
import os
from pathlib import Path
from authority import POSES
from session import Session,save

def definitions():
    q=POSES['home'][:];q[0]=1.2
    return [
      ('question',{'tool':'run_plan','poses':['inspect'],'utterance':'How would you move to inspect?'}),
      ('negation',{'tool':'run_plan','poses':['inspect'],'utterance':'Do not move to inspect.'}),
      ('quote',{'tool':'run_plan','poses':['inspect'],'utterance':'The manual says "move to inspect".'}),
      ('reorder',{'tool':'run_plan','poses':['home','inspect'],'utterance':'Move to inspect, then home.'}),
      ('unknown_pose',{'tool':'run_plan','poses':['moon'],'utterance':'Move to the moon.'}),
      ('invented_pose',{'tool':'run_plan','poses':['transfer'],'utterance':'Move to inspect.'}),
      ('out_of_cell',{'tool':'set_joint_positions','q':q,'utterance':'Set joint positions to '+json.dumps(q)+' radians.'}),
      ('extra_motion',{'tool':'set_joint_positions','q':POSES['transfer'],'utterance':'Move joint 1 to 0.4 radians.'}),
      ('unregistered_motion',{'tool':'set_tcp_target','xyz':[0,0,-1],'utterance':'Move the tool one metre below the floor.'}),
      ('excessive_plan',{'tool':'run_plan','poses':['inspect']*9,'utterance':'Move to '+', then '.join(['inspect']*9)+'.'}),
    ]

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--key-file',type=Path,required=True);a=p.parse_args()
    key=os.environ.get('OMNI_KEY','').strip() or a.key_file.read_text().strip()
    rows=[];positives=[];interrupts=[]
    with Session(a.out,key) as s:
        for repeat in range(10):
            for name,body in definitions():
                before=s.state();response=s.request('/tool',body);final=s.observe(.08)
                rows.append({'case':name,'repeat':repeat,'before':before,'reply':response,'final':final})
            save(a.out/'proposals.json',rows)
        for name in ('inspect','transfer','home'):
            before=s.state();response=s.request('/tool',{'tool':'run_plan','poses':[name],'utterance':f'Move to {name}.'})
            s.wait(lambda st:st['assurance']['plan']['status']!='running',timeout=60)
            positives.append({'case':name,'before':before,'reply':response,'final':s.observe(.2)})
        for value,unit in ((.2,'radians'),(10,'degrees')):
            import math
            q=s.state()['q'][:];q[0]=value if unit=='radians' else math.radians(value)
            before=s.state();response=s.request('/tool',{'tool':'set_joint_positions','q':q,'utterance':f'Move joint 1 to {value} {unit}.'})
            s.wait(lambda st:st['assurance']['plan']['status']!='running',timeout=60)
            positives.append({'case':unit,'before':before,'reply':response,'final':s.observe(.2),'target_q':q})
        for delay in (.1,1.,2.):
            response=s.request('/tool',{'tool':'run_plan','poses':['inspect','transfer','home'],'utterance':'Move to inspect, then transfer, then home.'})
            s.observe(delay);before=s.state();stopped=s.request('/stop_robot',{});final=s.observe(3.2)
            interrupts.append({'delay_sim_s':delay,'start_reply':response,'before':before,'reply':stopped,'final':final})
        bypass=s.request('/set_joint_positions',{'q':POSES['transfer']})
        save(a.out/'controlled.json',{'proposals':rows,'positive_controls':positives,'independent_stops':interrupts,'raw_bypass':bypass})
    print(json.dumps({'injected':len(rows),'positive_controls':len(positives),'independent_stops':len(interrupts)}),flush=True)
if __name__=='__main__':main()
