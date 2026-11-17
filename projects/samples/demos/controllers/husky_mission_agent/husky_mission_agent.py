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

"""Customer pilot: canonical OmniLink relay, bounded mission tools, physical Husky."""
import json
import math
import os
import re
from pathlib import Path
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'projects/samples/demos/controllers/omnilink_mobile_bridge'))
sys.path.insert(0,str(ROOT/'projects/services/husky_mission'))
sys.path.insert(0,str(ROOT/'projects/assessments/controllers/husky_mixed'))
import omnilink_mobile_bridge as mobile
from mission_runtime import MissionRuntime
from navigation import Waypoint
from drive_allocator import allocate
from omnisim_bridges.tool import Tool

OUT=Path(os.environ.get('OMNISIM_MISSION_OUT',str(ROOT/'.tmp/husky-mission-session')))
OUT.mkdir(parents=True,exist_ok=True)
OriginalBridge=mobile.MobileBridge
original_handler=mobile.make_handler
original_event=mobile._on_relay_event


class MissionBridge(OriginalBridge):
    def __init__(self,*args,**kwargs):
        self.mission_latched=False
        super().__init__(*args,**kwargs)
        self.mission_latched=True
        self.mission_http_context=threading.local()
        self._journal=(OUT/'mission-events.jsonl').open('a',encoding='utf-8',buffering=1)
        self._trace=(OUT/'trace.jsonl').open('a',encoding='utf-8',buffering=1)
        self.mission=MissionRuntime(self,lambda row:self._journal.write(json.dumps(row,allow_nan=False)+'\n'))
        previous=self.intents.set_turn_text
        def bind(text):
            self.mission.bind_turn(text);previous(text)
        self.intents.set_turn_text=bind
        self.on_operator_stop.append(lambda:self.mission.halt('cancelled' if 'cancel' in getattr(self.mission.context,'text','').lower() else 'stopped'))
        self._frame=0
        self.mission_measured_speed=None
        self.mission_nav=None
        self.mission_nav_done=threading.Event()
        self.mission_linear_i=self.mission_yaw_i=0.

    def hold_mission_latch(self):
        self.mission_latched=True
        with self.lock:self.motion=('idle',{})
        if getattr(self,'mission_nav',None) is not None:
            self.mission_nav.result={'arrived':False,'refused':'operator_halted'}
            self.mission_nav_done.set()
        super()._command_velocity(0.,0.)

    def release_mission_latch(self):self.mission_latched=False

    def _command_velocity(self,linear,angular):
        if getattr(self,'mission_nav',None) is not None and not self.mission_latched:
            return 0.,0. # The mission tick owns wheel outputs, not the idle driver.
        if self.mission_latched:linear=angular=0.
        return super()._command_velocity(linear,angular)

    def act_drive_to(self,x,y,wait=True):
        with self.lock:
            if self.mission_latched or self.mission.cancel.is_set():
                return {'arrived':False,'refused':'no_active_mission'}
            nav=Waypoint((x,y),self.sim_time);self.mission_nav=nav
            self.mission_linear_i=self.mission_yaw_i=0.
            self.mission_nav_done.clear()
        deadline=time.monotonic()+120
        while not self.mission_nav_done.wait(.03):
            if time.time()-self.last_tick_at>2 or time.monotonic()>deadline:
                self.hold_mission_latch();return {'arrived':False,'refused':'stale_or_stalled_simulation'}
        with self.lock:
            if self.mission_nav is nav:self.mission_nav=None
        return nav.result or {'arrived':False,'refused':'unmeasured_outcome'}

    def act_stop(self,*args,**kwargs):
        if hasattr(self,'mission'):self.mission.halt(getattr(self.mission_http_context,'reason','stopped'))
        return super().act_stop(*args,**kwargs)

    def get_state(self):
        s=super().get_state()
        if hasattr(self,'mission'):s['mission']=self.mission.snapshot()
        s['measured_speed_m_s']=getattr(self,'mission_measured_speed',None)
        if getattr(self,'mission_nav',None) is not None:s['mode']='mission_drive'
        return s

    def tick(self,dt_s):
        super().tick(dt_s)
        if not hasattr(self,'mission'):return
        velocity=self.robot.getSelf().getVelocity()
        self.mission_measured_speed=math.hypot(velocity[0],velocity[1])
        s=super().get_state()
        s['measured_speed_m_s']=self.mission_measured_speed
        with self.lock:
            nav=getattr(self,'mission_nav',None)
            if nav is not None and not self.mission_latched:
                linear,angular=nav.update(s)
                forward=velocity[0]*math.cos(s['yaw'])+velocity[1]*math.sin(s['yaw'])
                left,right,self.mission_linear_i,self.mission_yaw_i=allocate(linear,angular,forward,velocity[5],self.mission_linear_i,self.mission_yaw_i,self.cfg)
                if nav.braking:
                    left=right=0.;self.mission_linear_i=self.mission_yaw_i=0.
                if nav.result is not None:left=right=0.;self.mission_nav_done.set()
                for motor in self.left_motors:motor.setVelocity(left)
                for motor in self.right_motors:motor.setVelocity(right)
        row={k:s.get(k) for k in ('x','y','yaw','v_linear','v_angular','mode','sim_time','step','fault')}
        row['mission']=self.mission.snapshot();row['latched']=self.mission_latched
        row['measured_speed_m_s']=self.mission_measured_speed
        row['tilt_deg']=s.get('tilt_deg')
        row['z_m']=self.robot.getSelf().getPosition()[2]
        self._trace.write(json.dumps(row,allow_nan=False)+'\n')
        if os.environ.get('OMNISIM_MISSION_CAPTURE')=='1' and self.sim_time>=self._frame*.1:
            frames=OUT/'frames';frames.mkdir(exist_ok=True)
            self.robot.exportImage(str(frames/f'{self._frame:06d}.jpg'),90)
            self._frame+=1


def tools(bridge):
    return [
        Tool('start_mission','Execute only the explicitly requested ordered stations A, B and base. Returns an accepted running mission, not completion. Do not repeat this call within a turn.',
             {'type':'object','properties':{'stations':{'type':'array','items':{'type':'string','enum':['A','B','base']},'minItems':1,'maxItems':6}},'required':['stations'],'additionalProperties':False},
             lambda args:bridge.mission.start(args.get('stations')),physical=True,surface='mobile'),
        Tool('get_mission_status','Read verified visits, mission outcome and measured robot pose. Use this before answering completion or progress questions.',
             {'type':'object','properties':{},'additionalProperties':False},lambda args:bridge.get_state(),physical=False,surface='mobile'),
        Tool('stop_robot','Stop immediately and cancel all remaining mission stations. Cancellation never resumes without a fresh operator command.',
             {'type':'object','properties':{},'additionalProperties':False},lambda args:bridge.mission.stop(),physical=True,surface='mobile')]


def handler(bridge,relay=None):
    parent=original_handler(bridge,relay)
    class VerifiedHandler(parent):
        def _route_post(self,body):
            text=str(body.get('text') or '')
            self.mission_operator_text=text
            reason='cancelled' if self.path=='/prompt' and re.match(r'^\s*(?:cancel|abort)\b',text,re.I) else 'stopped'
            bridge.mission_http_context.reason=reason
            try:return super()._route_post(body)
            finally:bridge.mission_http_context.reason='stopped'
        def _json(self,code,payload):
            if self.path=='/prompt' and isinstance(payload,dict) and code==200:
                bridge._journal.write(json.dumps({'event':'agent.reply','raw_model_response':payload.get('response'),'verified_mission':bridge.mission.snapshot()})+'\n')
                payload=bridge.mission.public_reply(payload,getattr(self,'mission_operator_text',''))
            return super()._json(code,payload)
    return VerifiedHandler


def event(bridge,kind,payload):
    if kind=='agent':
        bridge._journal.write(json.dumps({'event':'agent.window_reply','raw_model_response':payload.get('text'),'verified_mission':bridge.mission.snapshot()})+'\n')
        payload={**payload,'text':bridge.mission.response()}
    if kind=='audio_out':return # This qualified pilot has a verified text interface.
    return original_event(bridge,kind,payload)


def configure(bridge,relay=None):
    cfg={'robot':'Husky Mission Agent','robot_class':'mobile mission robot',
         'agent':'OmniLink mission agent',**mobile.chat_config(relay),
         'suggestions':['Visit station A, then station B, and return to base.','What is the mission status?','Cancel the mission now.','Stop.']}
    bridge.queue_window('configure:'+json.dumps(cfg))
    bridge.queue_window('status:'+('connected' if relay is not None else 'error'))
    bridge.window_configured=True


mobile.MobileBridge=MissionBridge
mobile.build_mobile_tools=tools
mobile.build_mobile_main_task=lambda bridge: (
    'You are the Husky Mission Agent in OmniSim. The only places are station A, station B and base. '
    'Convert an explicit operator route into start_mission(stations) in exactly that order. '
    'The standard request is visit A, then B, and return to base. Never invent a station or change its order. '
    'Unknown stations require clarification and no motion. Questions and quoted or negated orders do not start motion. '
    'For cancellation or Stop use stop_robot. Read get_mission_status for progress and outcome questions. '
    'A started mission runs asynchronously. Do not poll repeatedly or start it again. Accepted is not completed. '
    'Only measured station receipts establish visits; only the runtime establishes completion. '
    'Never attempt raw velocity, coordinates, resets, forces or other unregistered operations.')
mobile.make_handler=handler
mobile._on_relay_event=event
mobile.push_configure=configure

if __name__=='__main__':raise SystemExit(mobile.main())
