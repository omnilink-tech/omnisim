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

"""A bounded arm workcell contract, not a general collision-safety certificate.

Only this authority creates completion receipts, from measured joint state.
The model proposes tools; it cannot change the profile or authorize itself.
"""
import copy
import json
import math
import re
import threading
import time
import uuid

POSES={
 'home':[-.1732,.1855,.7417,0.,2.2144,0.],
 'inspect':[-.45,.12,.90,0.,2.12,.10],
 'transfer':[.40,.25,.80,.08,2.20,-.10],
 'park':[0.,.10,1.,0.,2.04,0.],
}
LIMITS=[(-.65,.65),(0.,.4),(.6,1.15),(-.2,.2),(1.85,2.45),(-.3,.3)]
ALIASES={'home':'home','inspect':'inspect','inspection':'inspect','transfer':'transfer','park':'park','parking':'park'}
NO_ORDER=re.compile(r'\b(?:do not|don.t|never|must not|without|not yet|hypothetical|pretend|imagine)\b|[?]|["“”]',re.I)
ACTION=re.compile(r'\b(?:move|go|visit|return|run|execute|set|position|take)\b',re.I)

def named_order(text):
 if NO_ORDER.search(text) or not ACTION.search(text):return []
 # An explicitly named unsupported pose must not turn into a guessed home.
 if re.search(r'\b(?:moon|ceiling|unknown|secret|danger|outside)\b',text,re.I):return []
 return [ALIASES[m.group().lower()] for m in re.finditer(r'\b(?:home|inspect|inspection|transfer|park|parking)\b',text,re.I)]

def valid_q(q):
 return isinstance(q,list) and len(q)==6 and all(type(v) in (float,int) and math.isfinite(v) and lo<=v<=hi for v,(lo,hi) in zip(q,LIMITS))

def joint_authorized(text,q,current):
 if NO_ORDER.search(text) or not ACTION.search(text):return False
 names=named_order(text)
 if len(names)==1:return max(abs(a-b) for a,b in zip(q,POSES[names[0]]))<1e-6
 array=re.search(r'\[[^\[\]]+\]',text)
 if array:
  try:
   expected=json.loads(array.group())
   if 'degree' in text.lower():expected=[math.radians(v) for v in expected]
   return len(expected)==6 and all(type(v) in (int,float) and math.isfinite(v) for v in expected) and max(abs(a-b) for a,b in zip(q,expected))<1e-6
  except (ValueError,TypeError):return False
 match=re.search(r'\bjoint\s*([1-6])\s+(?:to|at)\s+([-+]?\d+(?:\.\d+)?)\s*(degrees?|radians?|rad)\b',text,re.I)
 if not match:return False
 i=int(match[1])-1;value=float(match[2]);value=math.radians(value) if match[3].lower().startswith('deg') else value
 return abs(q[i]-value)<1e-6 and all(abs(q[j]-current[j])<=.002 for j in range(6) if j!=i)

class Authority:
 def __init__(self,driver,journal=None):
  self.driver=driver;self.journal=journal;self.lock=threading.RLock();self.context=threading.local()
  self.cancel=threading.Event();self.thread=None;self.held=False;self.ready=False;self.fault=None;self.used_turns=set()
  self.record={'id':None,'status':'idle','requested':[],'receipts':[],'reason':None}
 def bind_turn(self,text,arm_hold=True):
  self.context.text=text or '';self.context.turn=uuid.uuid4().hex
  # Authenticated operator text is bound before model dispatch on both chat
  # entry paths. An explicit hold arms independently of model tool selection.
  if arm_hold and re.match(r'^\s*(?:please\s+)?(?:hold|pause|lock)\b',text or '',re.I) and not NO_ORDER.search(text):
   self.held=True;self.halt('stopped')
  stop=re.match(r'^\s*(?:please\s+)?(stop|halt|cancel)\b',text or '',re.I)
  if arm_hold and stop and not NO_ORDER.search(text):
   # Independent operator Stop/Cancel: a model's latency or refusal must
   # never keep an already-running plan moving after this explicit control.
   self.halt('cancelled' if stop[1].lower()=='cancel' else 'stopped')
  if arm_hold and re.match(r'^\s*(?:please\s+)?(?:resume|unlock)\b',text or '',re.I):
   # Resume is an explicit operator control, just like hold and Stop. A
   # missing model tool call must not leave a requested unlock unperformed.
   # The existing validation still rejects questions, quotes and negations.
   self.resume()
 def snapshot(self):
  with self.lock:return {'ready':self.ready,'held':self.held,'fault':self.fault,'plan':copy.deepcopy(self.record)}
 def log(self,event,**data):
  if self.journal:self.journal({'event':event,**data})
 def state(self):
  s=self.driver.measured_state()
  if not isinstance(s.get('q'),list) or len(s['q'])!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in s['q']):raise ValueError('unmeasured_joints')
  if time.time()-s.get('wall_time',0)>2:raise ValueError('stale_telemetry')
  if s.get('fault'):raise ValueError('robot_fault')
  return s
 def observe(self,state):
  # Boot homing is excluded. Commands are inhibited until the qualified cell
  # has been reached physically; an absent/failed ready state is an error.
  if not self.ready and state['sim_time']>3 and valid_q(state['q']) and state.get('max_joint_speed_rad_s') is not None and state['max_joint_speed_rad_s']<.03:self.ready=True
  if self.ready and any(not lo-.05<=v<=hi+.05 for v,(lo,hi) in zip(state['q'],LIMITS)):
   self.fault='measured_cell_boundary_exceeded';self.halt('failed')
  if self.ready and state.get('max_joint_speed_rad_s',0)>1.6:
   self.fault='measured_joint_speed_exceeded';self.halt('failed')
 def refuse(self,reason):self.log('proposal.refused',reason=reason,text=getattr(self.context,'text',''));return {'accepted':False,'error':reason,'measured':self.snapshot()}
 def start(self,poses=None,q=None):
  text=getattr(self.context,'text','');token=getattr(self.context,'turn',None)
  try:s=self.state()
  except ValueError as exc:return self.refuse(str(exc))
  if q is not None:
   if not valid_q(q):return self.refuse('outside_qualified_joint_cell')
   if not joint_authorized(text,q,s['q']):return self.refuse('joint_proposal_not_authorized')
   steps=[{'name':'joint-target','q':list(q)}]
  else:
   if not isinstance(poses,list) or not 1<=len(poses)<=8 or any(type(p) is not str or p not in POSES for p in poses):return self.refuse('unknown_pose_or_plan')
   if poses!=named_order(text):return self.refuse('plan_differs_from_operator_order')
   steps=[{'name':p,'q':POSES[p][:]} for p in poses]
  with self.lock:
   if not self.ready or self.fault:return self.refuse('not_ready_or_faulted')
   if self.held:return self.refuse('operator_hold_active')
   if self.thread and self.thread.is_alive():return self.refuse('plan_busy')
   if not token or token in self.used_turns:return self.refuse('duplicate_or_missing_operator_turn')
   self.used_turns.add(token);self.cancel=threading.Event();cancel=self.cancel
   self.record={'id':uuid.uuid4().hex,'status':'running','requested':steps,'receipts':[],'reason':None,'started_sim_s':s['sim_time']}
   self.driver.release_latch();self.log('plan.started',record=self.snapshot())
   self.thread=threading.Thread(target=self.execute,args=(steps,cancel),daemon=True);self.thread.start()
   return {'accepted':True,'plan':self.snapshot()}
 def execute(self,steps,cancel):
  try:
   for step in steps:
    if cancel.is_set():return
    self.state();result=self.driver.perform(step['q'],cancel);state=self.state()
    with self.lock:
     if cancel.is_set():return
     error=max(abs(a-b) for a,b in zip(state['q'],step['q']))
     if not result.get('verified') or error>.04 or state.get('max_joint_speed_rad_s',math.inf)>.03:
      self.log('arrival.rejected',result=result,measured=state);raise ValueError('arrival_not_physically_verified')
     receipt={'name':step['name'],'target_q':step['q'],'measured_q':state['q'],'max_error_rad':error,'sim_time':state['sim_time'],'max_speed_rad_s':state['max_joint_speed_rad_s']}
     if 'measurement_step' in state:receipt['measurement_step']=state['measurement_step']
     self.record['receipts'].append(receipt);self.log('step.verified',plan_id=self.record['id'],receipt=receipt)
   with self.lock:
    if cancel.is_set():return
    self.record.update(status='completed',finished_sim_s=self.state()['sim_time']);self.log('plan.completed',record=self.snapshot())
  except Exception as exc:
   with self.lock:
    if not cancel.is_set():self.record.update(status='failed',reason=str(exc));self.driver.hold_latch();self.log('plan.failed',record=self.snapshot())
 def halt(self,reason='stopped'):
  with self.lock:
   self.cancel.set();self.driver.hold_latch()
   if self.record['status']=='running' or (self.record['status']=='stopped' and reason=='cancelled'):
    sample=self.driver.measured_state()
    self.record.update(status=reason,reason='operator_'+reason,halt_sim_s=sample['sim_time'])
    # This sample was persisted before the operator's halt arrived. It is
    # the conservative settling-clock origin, not a post-command observation.
    if 'measurement_step' in sample:self.record['halt_after_measurement_step']=sample['measurement_step']
    self.log('plan.'+reason,record=self.snapshot())
 def hold(self):
  text=getattr(self.context,'text','')
  if NO_ORDER.search(text) or not re.search(r'\b(?:hold|pause|lock)\b',text,re.I):return self.refuse('hold_not_requested')
  self.held=True;self.halt('stopped');return {'accepted':True,'measured':self.snapshot()}
 def resume(self):
  text=getattr(self.context,'text','')
  if re.search(r'[?"“”]|\b(?:not|don.t|never|manual|quoted|pretend|says)\b',text,re.I) or not re.match(r'^\s*(?:please\s+)?(?:resume|unlock)\b',text,re.I):return self.refuse('explicit_resume_required')
  if self.fault:return self.refuse('fault_requires_investigation')
  self.held=False;return {'accepted':True,'automatic_motion':False,'measured':self.snapshot()}
 def response(self,state=None):
  s=self.snapshot() if state is None else state;p=s['plan'];prefix=('Arm fault: '+s['fault']+'.') if s['fault'] else 'Arm held.' if s['held'] else 'Arm ready.' if s['ready'] else 'Arm not ready.'
  if not p['id']:return prefix+' No verified plan.'
  names=', '.join(r['name'] for r in p['receipts']) or 'none'
  return prefix+f" Last plan {p['id'][:8]} {p['status']}. Verified steps: {names}."
 def public_reply(self,payload):
  state=self.snapshot()
  return {**payload,'response':('OmniLink request failed. ' if payload.get('error') else '')+self.response(state),'verified_arm':state,'response_basis':'measured joint receipts; model prose is not completion evidence'}
