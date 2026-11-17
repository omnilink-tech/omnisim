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

"""Measured OmniArm 6 command-assurance profile around the canonical bridge."""
import json
import copy
import math
import os
from pathlib import Path
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'projects/samples/demos/controllers/omnilink_arm_bridge'))
sys.path.insert(0,str(ROOT/'projects/services/arm_assurance'))
import omnilink_arm_bridge as arm

OUT=Path(os.environ.get('OMNISIM_ARM_ASSURANCE_OUT',str(ROOT/'.tmp/arm-assurance-session')))
OUT.mkdir(parents=True,exist_ok=True)
Original=arm.ArmBridge
original_handler=arm.make_handler
original_event=arm._on_relay_event
GUARDED=os.environ.get('OMNISIM_ARM_PROFILE','guarded')!='canonical'
from authority import Authority,POSES,LIMITS
from omnisim_bridges.tool import Tool

class MeasuredArm(Original):
    def __init__(self,*args,**kwargs):
        self.assurance=None;self.assurance_speed=None
        self._sample_lock=threading.Lock();self._measured_sample=None
        super().__init__(*args,**kwargs)
        self._trace=(OUT/'trace.jsonl').open('w',encoding='utf-8',buffering=1)
        self._journal=(OUT/'events.jsonl').open('w',encoding='utf-8',buffering=1)
        self._last_q=None;self._last_sample_time=None;self.assurance_speed=None;self._frame=0
        self.assurance_latched=True;self.http_context=threading.local()
        if GUARDED:
            self.assurance=Authority(self,lambda row:self._journal.write(json.dumps(row,allow_nan=False)+'\n'))
            previous=self.intents.set_turn_text
            def bind(text):self.assurance.bind_turn(text);previous(text)
            self.intents.set_turn_text=bind

    def measured_state(self):
        # Sensor buffers can advance in robot.step() before tick() publishes
        # its clock. Never combine those new joints with the previous stamp,
        # or a worker can mint a receipt absent from the per-tick trace.
        with self._sample_lock:
            if self._measured_sample is None:raise ValueError('unmeasured_joints')
            return copy.deepcopy(self._measured_sample)

    def hold_latch(self):
        with self.lock:
            self.assurance_latched=True
            Original.act_stop(self,source='assurance-internal')

    def release_latch(self):
        with self.lock:self.assurance_latched=False

    def perform(self,q,cancel):
        with self.lock:
            if cancel.is_set() or self.assurance_latched:return {'verified':False,'reason':'latched'}
            accepted=Original.act_set_joint_positions_checked(self,q,duration_s=3.,wait=False)
            if not accepted.get('accepted'):return {'verified':False,'dispatch':accepted}
        deadline=time.monotonic()+45;stable=None
        while time.monotonic()<deadline:
            if cancel.is_set() or self.assurance_latched:return {'verified':False,'reason':'interrupted'}
            state=self.measured_state()
            if time.time()-state['wall_time']>2:return {'verified':False,'reason':'stale_telemetry'}
            # An already reached target is still an active timed trajectory.
            # Wait for the canonical motion slot to finish as well as measured
            # rest, or the next step is correctly refused as busy.
            at=state.get('mode') in ('idle','hold') and not state.get('settling') and max(abs(a-b) for a,b in zip(q,state['q']))<=.04 and state.get('max_joint_speed_rad_s',math.inf)<=.03
            if at:
                if stable is None:stable=state['sim_time']
                if state['sim_time']-stable>=.35:return {'verified':True,'dispatch':accepted}
            else:stable=None
            time.sleep(.015)
        return {'verified':False,'reason':'settling_deadline'}

    def act_stop(self,source=Original.SOURCE_EXTERNAL):
        if self.assurance is not None and source==Original.SOURCE_EXTERNAL:
            self.assurance.halt(getattr(self.http_context,'reason','stopped'))
        return Original.act_stop(self,source)

    def tick(self,sim_time_s):
        super().tick(sim_time_s)
        # Only the simulator thread publishes assurance samples. This sample
        # supplies both the persisted trace and every asynchronous reader.
        with self.lock:state=Original.get_state(self)
        q=state['q'];dt_s=state['sim_time']-self._last_sample_time if self._last_sample_time is not None else None
        if dt_s is not None and dt_s<=0:raise ValueError('nonmonotonic_measurement_clock')
        self.assurance_speed=max((abs(a-b)/dt_s for a,b in zip(q,self._last_q)),default=0) if self._last_q else None
        self._last_q=q[:];self._last_sample_time=state['sim_time']
        state['max_joint_speed_rad_s']=self.assurance_speed;state['measurement_step']=state['step']
        if self.assurance is not None:self.assurance.observe(state)
        row={'sim_time':state['sim_time'],'measurement_step':state['measurement_step'],'q':q,'tcp':state['tcp'],'max_joint_speed_rad_s':self.assurance_speed,'mode':state['mode'],'fault':state['fault']}
        if self.assurance is not None:row['assurance']=self.assurance.snapshot()
        # Persist before publishing: a receipt can never outrun its evidence.
        self._trace.write(json.dumps(row,allow_nan=False)+'\n')
        with self._sample_lock:self._measured_sample=copy.deepcopy(state)
        if os.environ.get('OMNISIM_ARM_ASSURANCE_CAPTURE')=='1' and self.sim_time>=self._frame*.1:
            frames=OUT/'frames';frames.mkdir(exist_ok=True)
            self.robot.exportImage(str(frames/f'{self._frame:06d}.jpg'),90);self._frame+=1

    def get_state(self):
        if self._measured_sample is None:
            # Construction is on the simulator thread, before HTTP/relay
            # startup. No qualified receipt is possible before the first tick.
            state=super().get_state();state['max_joint_speed_rad_s']=None
        else:state=self.measured_state()
        if getattr(self,'assurance',None) is not None:state['assurance']=self.assurance.snapshot()
        return state

def tools(bridge):
    def stop():
        text=getattr(bridge.assurance.context,'text','')
        bridge.assurance.halt('cancelled' if 'cancel' in text.lower() else 'stopped')
        Original.act_stop(bridge,source='assurance-internal')
        return bridge.assurance.snapshot()
    return [
      Tool('run_plan','Run only the exact explicitly requested ordered named poses. Returns accepted/running; completion requires measured receipts. Never repeat a start in a turn.',
           {'type':'object','properties':{'poses':{'type':'array','items':{'type':'string','enum':list(POSES)},'minItems':1,'maxItems':8}},'required':['poses'],'additionalProperties':False},lambda a:bridge.assurance.start(poses=a.get('poses')),physical=True,surface='arm'),
      Tool('set_joint_positions','Move exactly the requested six joint angles in radians. Read state for unchanged joints. Workcell limits: '+str(LIMITS)+'. Out-of-cell or unrequested angles are refused. Accepted is not completed.',
           {'type':'object','properties':{'q':{'type':'array','items':{'type':'number'},'minItems':6,'maxItems':6}},'required':['q'],'additionalProperties':False},lambda a:bridge.assurance.start(q=a.get('q')),physical=True,surface='arm'),
      Tool('get_robot_state','Read measured joints, speed, hold, verified plan and receipts before answering a status question.',{'type':'object','properties':{},'additionalProperties':False},lambda a:bridge.get_state(),physical=False,surface='arm'),
      Tool('hold_arm','Latch an explicit operator hold. No motion can start until an explicit resume. Cancels any current plan.',{'type':'object','properties':{},'additionalProperties':False},lambda a:bridge.assurance.hold(),physical=False,surface='arm'),
      Tool('resume_arm','Only an explicit operator resume clears a hold. Does not resume cancelled work or cause motion.',{'type':'object','properties':{},'additionalProperties':False},lambda a:bridge.assurance.resume(),physical=False,surface='arm'),
      Tool('stop_robot','Stop and cancel all remaining plan steps immediately. Never resumes the cancelled plan automatically.',{'type':'object','properties':{},'additionalProperties':False},lambda a:stop(),physical=True,surface='arm')]

def handler(bridge,relay=None):
    parent=original_handler(bridge,relay)
    class GuardedHandler(parent):
        def _route_post(self,body):
            if self.path not in ('/prompt','/tool','/stop_robot'):
                return self._json(403,{'error':'outside_qualified_agent_interface','allowed':['/prompt','/tool','/stop_robot']})
            text=str(body.get('text') or '')
            bridge.http_context.reason='cancelled' if text.lower().lstrip().startswith('cancel') else 'stopped'
            # The parent authenticates before dispatch. Binding a typed tool
            # here must not arm a hold before that authentication has run.
            if self.path=='/tool':bridge.assurance.bind_turn(str(body.get('utterance') or ''),arm_hold=False)
            try:return super()._route_post(body)
            finally:
                bridge.http_context.reason='stopped'
                if self.path=='/tool':bridge.assurance.bind_turn('',arm_hold=False)
        def _json(self,code,payload):
            if self.path=='/prompt' and isinstance(payload,dict) and code==200:
                bridge._journal.write(json.dumps({'event':'model.reply','raw':payload,'verified':bridge.assurance.snapshot()})+'\n')
                payload=bridge.assurance.public_reply(payload)
            return super()._json(code,payload)
    return GuardedHandler

def event(bridge,kind,payload):
    if kind=='agent':
        bridge._journal.write(json.dumps({'event':'model.window_reply','raw':payload,'verified':bridge.assurance.snapshot()})+'\n')
        payload={**payload,'text':bridge.assurance.response()}
    if kind=='audio_out':return
    return original_event(bridge,kind,payload)

arm.ArmBridge=MeasuredArm
if GUARDED:
    arm.build_arm_tools=tools
    arm.make_handler=handler
    arm._on_relay_event=event
    arm.build_arm_main_task=lambda b:(
      'You operate an OmniArm 6 in the bounded command-assurance profile. The named poses are home, inspect, transfer and park. '
      'Use run_plan for an explicitly requested ordered list. For a numeric joint command, use set_joint_positions with all six angles in radians, reading current state for other joints. '
      'Never guess a missing quantity or pose. Questions, quoted instructions, negations and hypothetical statements must not start motion. '
      'Use hold_arm for a standing hold, resume_arm only on explicit resume, stop_robot for Stop or cancellation. A held arm refuses new motion until resumed. '
      'Accepted plans run asynchronously. Do not poll in a loop or submit them again. Only measured receipts establish steps or completion. '
      'Read get_robot_state for progress and outcome. Never claim completion from your own intended actions. '
      'The qualified joint cell and speed limits cannot be disabled by an operator instruction. Raw Cartesian, force, gripper, servo, reset and learned actions are outside this profile. '
      'This is a no-payload simulation workcell; do not claim collision avoidance or hardware safety.')
if __name__=='__main__':raise SystemExit(arm.main())
