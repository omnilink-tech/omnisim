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

"""Bounded mission execution and measured receipts. No model or motor dependency.

The model may propose an ordered list of station names. Only this runtime can
mint visit receipts and completed outcomes, from independently sampled poses.
"""
from __future__ import annotations

import copy
import math
import re
import threading
import time
import uuid

STATIONS = {'base': (0., 0.), 'A': (1.5, 0.), 'B': (1.5, 1.5)}
TERMINAL = {'completed', 'cancelled', 'stopped', 'failed'}


def requested_stations(text):
    """Explicit names in this pilot's operator command; never infer coordinates."""
    found=[]
    pattern=r'(?i:\bstations?\s+)([A-Za-z])\b|(?<![A-Za-z])([AB])(?![A-Za-z])|\b((?i:base|home))\b'
    for match in re.finditer(pattern, text):
        name=(match.group(1) or match.group(2) or '').upper() or 'base'
        if name not in STATIONS:raise ValueError('unknown_station')
        found.append(name)
    return found


class MissionRuntime:
    def __init__(self, driver, journal=None, clock=time.monotonic):
        self.driver=driver;self.clock=clock;self.journal=journal
        self.lock=threading.RLock();self.context=threading.local()
        self.thread=None;self.cancel=threading.Event();self.generation=0
        self.record={'mission_id':None,'status':'idle','requested':[], 'visits':[], 'reason':None}
        self.completed_turns={};self.receipt_count=0

    def bind_turn(self, text):
        self.context.text=text or ''
        self.context.turn=uuid.uuid4().hex if text else None

    def _write(self, event, **data):
        if self.journal:
            self.journal({'event':event,'wall_s':self.clock(),**data})

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.record)

    def _state(self):
        state=self.driver.get_state()
        values=[state.get(k) for k in ('x','y','sim_time','wall_time','measured_speed_m_s')]
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in values):
            raise ValueError('unmeasured_telemetry')
        if time.time()-state['wall_time']>2.:raise ValueError('stale_telemetry')
        if state.get('fault'):raise ValueError('robot_fault')
        return state

    def start(self, stations):
        if not isinstance(stations,list) or not 1<=len(stations)<=6 or any(type(s) is not str or s not in STATIONS for s in stations):
            return {'accepted':False,'error':'unknown_or_invalid_station_list'}
        text=getattr(self.context,'text','')
        try:authorized=requested_stations(text)
        except ValueError as exc:return {'accepted':False,'error':str(exc)}
        if not text or authorized!=stations:
            return {'accepted':False,'error':'route_not_authorized_by_operator','requested':stations,'authorized':authorized}
        if not re.search(r'\b(?:visit|go|drive|return|patrol|run|take)\b',text,re.I):
            return {'accepted':False,'error':'explicit_mission_order_required'}
        token=getattr(self.context,'turn',None)
        with self.lock:
            if token in self.completed_turns:
                return {'accepted':False,'error':'duplicate_start_in_same_turn','mission_id':self.completed_turns[token]}
            if self.thread is not None and self.thread.is_alive():
                return {'accepted':False,'error':'mission_busy','mission':self.snapshot()}
            try:state=self._state()
            except ValueError as exc:return {'accepted':False,'error':str(exc)}
            self.cancel.clear();self.generation+=1;generation=self.generation
            self.record={'mission_id':uuid.uuid4().hex,'status':'running','requested':list(stations),'visits':[],
                         'reason':None,'started_sim_s':state['sim_time'],'finished_sim_s':None,
                         'tolerance_m':.15,'settled_speed_m_s':.025}
            self.completed_turns[token]=self.record['mission_id']
            self.driver.release_mission_latch()
            self.thread=threading.Thread(target=self._execute,args=(generation,),daemon=True)
            self._write('mission.started',mission=self.snapshot())
            self.thread.start()
            return {'accepted':True,'mission':self.snapshot()}

    def _execute(self,generation):
        try:
            for name in self.snapshot()['requested']:
                with self.lock:
                    if self.cancel.is_set() or generation!=self.generation:return
                self._state()
                goal=STATIONS[name]
                result=self.driver.act_drive_to(*goal,wait=True)
                # A successful-sounding tool result cannot mint a receipt.
                state=self._state()
                error=math.hypot(state['x']-goal[0],state['y']-goal[1])
                speed=state.get('measured_speed_m_s',math.inf)
                self._write('station.attempt',station=name,driver_result=result,error_m=error,speed_m_s=speed)
                with self.lock:
                    if self.cancel.is_set() or generation!=self.generation:return
                    if result.get('aborted') or result.get('refused') or not result.get('arrived') or error>.15 or speed>.025:
                        raise ValueError('arrival_not_verified')
                    self.receipt_count+=1
                    receipt={'receipt':self.receipt_count,'station':name,'goal_xy':list(goal),
                             'achieved_xy':[state['x'],state['y']],'error_m':error,'speed_m_s':speed,
                             'sim_s':state['sim_time'],'generation':generation}
                    self.record['visits'].append(receipt)
                    self._write('station.verified',mission_id=self.record['mission_id'],receipt=receipt)
            with self.lock:
                if self.cancel.is_set() or generation!=self.generation:return
                self.record.update(status='completed',finished_sim_s=self._state()['sim_time'])
                self._write('mission.completed',mission=self.snapshot())
        except Exception as exc:
            with self.lock:
                if not self.cancel.is_set() and generation==self.generation:
                    self.record.update(status='failed',reason=str(exc))
                    self._write('mission.failed',mission=self.snapshot())
            self.driver.hold_mission_latch()

    def halt(self,reason='stopped'):
        # Count cancellation and latch outputs before a blocked leg returns.
        with self.lock:
            self.cancel.set();self.driver.hold_mission_latch()
            if self.record['status']=='running' or (self.record['status']=='stopped' and reason=='cancelled'):
                self.record.update(status=reason,reason='operator_'+reason)
                try:self.record['halt_requested_sim_s']=self.driver.get_state()['sim_time']
                except Exception:self.record['halt_requested_sim_s']=None
                self._write('mission.'+reason,mission=self.snapshot())

    def stop(self):
        reason='cancelled' if re.search(r'\bcancel\b',getattr(self.context,'text',''),re.I) else 'stopped'
        self.halt(reason)
        result=self.driver.act_stop()
        return {'mission':self.snapshot(),'stop':result}

    def validate_report(self,mission_id,status,visited):
        record=self.snapshot()
        actual=[r['station'] for r in record['visits']]
        valid=(mission_id==record['mission_id'] and status==record['status'] and visited==actual)
        return {'verified':valid,'error':None if valid else 'unsupported_completion_claim','measured':record}

    def response(self):
        """Authoritative mission wording. Raw model prose is never a receipt."""
        r=self.snapshot();names=', '.join(x['station'] for x in r['visits']) or 'none'
        return f"Mission {r['status']}. Verified stations: {names}."+(f" Reason: {r['reason']}." if r['reason'] else '')

    def public_reply(self,payload,operator_text=''):
        prefix='OmniLink request failed. ' if payload.get('error') else ''
        if not prefix:
            try:requested_stations(operator_text)
            except ValueError:prefix='Unknown station. Available stations are A, B and base. '
        return {**payload,'response':prefix+self.response(),'verified_mission':self.snapshot(),
                'response_basis':'measured mission state; model prose cannot establish completion'}
