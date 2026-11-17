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

"""Physical mixed-surface test driver. No body state writes or force injection."""
import json
import math
import os
from pathlib import Path
import runpy
import traceback
from omnisim import Supervisor
from drive_allocator import allocate,lookahead_point

ROOT=Path(__file__).resolve().parents[4]


def clamp(x,lo,hi):return max(lo,min(hi,x))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))


def main():
    out=Path(os.environ['OMNISIM_ASSESSMENT_OUT'])
    s=json.loads((out/'spec.json').read_text())
    cfgmod=runpy.run_path(str(ROOT/'projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py'))
    cfg=cfgmod['get_config']('husky');names=cfgmod['WHEEL_MOTORS'][cfg['layout']]
    sim=Supervisor();node=sim.getSelf()
    motors={side:[sim.getDevice(n) for n in names[side]] for side in ('left','right')}
    sensors=[]
    for group in motors.values():
        for motor in group:
            if motor is None:raise RuntimeError('Husky wheel motor missing')
            motor.setPosition(float('inf'));motor.setVelocity(0)
            sensor=motor.getPositionSensor();sensor.enable(16);sensors.append(sensor)
    capture=os.environ.get('OMNISIM_ASSESSMENT_CAPTURE')=='1'
    frames=out/'frames'
    if capture:frames.mkdir()
    started=None;finished=None;frame=0;seq=0;frame_times=[]
    sign=s['turn_sign'];radius=s['turn_radius_m'];trigger=s['trigger_x_m']
    goal=(trigger+radius,sign*radius)
    route_mode=s.get('steering_mode')=='traction-route' and s['layout']=='split' and s['task']=='turn' and sign==1
    course=[(0,0),(.4,-.12),(s['boundary_x_m']-.35,-.18),(trigger,.16*radius),
            (trigger+.6*radius,.42*radius),(trigger+.8*radius,.72*radius),(trigger+.85*radius,radius)]
    complete=False;linear_mix=0.;yaw_mix=0.;emergency=False;aligning=False
    with (out/'trace.jsonl').open('w',encoding='utf-8',buffering=1) as trace:
        while sim.step(16)!=-1:
            t=sim.getTime();p=node.getPosition();v=node.getVelocity();o=node.getOrientation()
            yaw=math.atan2(o[3],o[0]);tilt=math.acos(clamp(o[8],-1,1))
            if started is None and p[0]>=trigger:started=t
            speed=s['cruise_m_s']
            if s['policy']=='surface-cap' and p[0]>=s['cap_start_x_m']:speed=min(speed,s['cap_m_s'])
            command_v=0.;omega=0.;path_error=0.
            if t>=1 and started is None:
                command_v=speed
                omega=clamp(-2.5*yaw-1.5*p[1],-.8,.8)
            elif started is not None and s['task']=='turn' and finished is None and not route_mode:
                dy=sign*p[1]
                phi=clamp(math.atan2(p[0]-trigger,radius-dy),0,math.pi/2)
                nearest=(trigger+radius*math.sin(phi),sign*radius*(1-math.cos(phi)))
                path_error=math.hypot(p[0]-nearest[0],p[1]-nearest[1])
                ahead=min(math.pi/2,phi+.35/radius)
                target=(trigger+radius*math.sin(ahead),sign*radius*(1-math.cos(ahead)))
                alpha=wrap(math.atan2(target[1]-p[1],target[0]-p[0])-yaw)
                distance=math.hypot(p[0]-goal[0],p[1]-goal[1])
                if distance<.18:aligning=True
                if aligning:
                    command_v=0.
                    omega=clamp(2*wrap(sign*math.pi/2-yaw),-.6,.6)
                    if abs(wrap(sign*math.pi/2-yaw))<.04:finished=t;omega=0.
                else:
                    command_v=min(speed,max(.12,1.3*distance))
                    omega=clamp(2*command_v*math.sin(alpha)/max(.20,math.hypot(target[0]-p[0],target[1]-p[1])),-1.2,1.2)
            if started is not None and s['task']=='turn':
                phi=clamp(math.atan2(p[0]-trigger,radius-sign*p[1]),0,math.pi/2)
                path_error=math.hypot(p[0]-(trigger+radius*math.sin(phi)),p[1]-sign*radius*(1-math.cos(phi)))
            if route_mode and t>=1 and finished is None:
                distance=math.hypot(p[0]-goal[0],p[1]-goal[1])
                if started is not None and distance<.23 and not aligning:
                    aligning=True;linear_mix=yaw_mix=0.
                if aligning:
                    heading_error=wrap(sign*math.pi/2-yaw)
                    omega=clamp(2*heading_error,-.6,.6)
                    command_v=.10 if abs(heading_error)<.15 and goal[1]-p[1]>.08 else 0.
                    if distance<.23 and abs(heading_error)<.04 and math.hypot(v[0],v[1])<.015:
                        finished=t;command_v=omega=0.
                else:
                    target=lookahead_point(p,course)
                    alpha=wrap(math.atan2(target[1]-p[1],target[0]-p[0])-yaw)
                    # Start decelerating before the low-traction goal. A late
                    # position-only stop leaves lateral momentum uncorrected.
                    command_v=min(s['cap_m_s'] if started is None else .18,max(.06,.6*distance))
                    omega=clamp(2*command_v*math.sin(alpha)/max(.20,math.hypot(target[0]-p[0],target[1]-p[1])),-1.2,1.2)
            # Both variants use physical body feedback. The repair changes
            # allocation as well as the mapped route and turn-speed target.
            if started is not None and s['task']=='turn' and path_error>1.:
                emergency=True
            if emergency or (command_v==0 and omega==0):
                left=right=0.;linear_mix=yaw_mix=0.
            else:
                forward=v[0]*math.cos(yaw)+v[1]*math.sin(yaw)
                if route_mode:
                    left,right,linear_mix,yaw_mix=allocate(command_v,omega,forward,v[5],linear_mix,yaw_mix,cfg)
                else:
                    linear_mix=clamp(linear_mix+4*(command_v-forward)*.016,-.6,.6)
                    yaw_mix=clamp(yaw_mix+6*(omega-v[5])*.016/cfg['yaw_rate_gain'],-3.3,3.3)
                    diff=yaw_mix*cfg['half_track_m']/cfg['wheel_radius_m']
                    common=(command_v+linear_mix)/cfg['wheel_radius_m']
                    common=clamp(common,-max(0,6-abs(diff)),max(0,6-abs(diff)))
                    left=common-diff;right=common+diff
            for side,value in [('left',left),('right',right)]:
                for motor in motors[side]:motor.setVelocity(value)
            wheel_pos=[sensor.getValue() for sensor in sensors]
            trace.write(json.dumps(dict(seq=seq,t_s=t,x_m=p[0],y_m=p[1],z_m=p[2],vx_m_s=v[0],vy_m_s=v[1],wz_rad_s=v[5],
                yaw_rad=yaw,tilt_rad=tilt,path_error_m=path_error,event_started=started is not None,
                turn_finished=finished is not None,left_command_rad_s=left,right_command_rad_s=right,
                commanded_v_m_s=command_v,commanded_yaw_rad_s=omega,wheel_position_rad=wheel_pos,
                contacts=len(node.getContactPoints(True)),emergency_corridor_stop=emergency),allow_nan=False)+'\n')
            seq+=1
            if capture and t>=frame*.1:
                sim.exportImage(str(frames/f'{frame:05d}.jpg'),90);frame_times.append(dict(frame=frame,t_s=t));frame+=1
            if t>=s['duration_s']:
                complete=True;break
    for group in motors.values():
        for motor in group:motor.setVelocity(0)
    (out/'completion.json').write_text(json.dumps(dict(complete=complete,samples=seq,last_t_s=t,trigger_t_s=started,turn_finished_t_s=finished,frames=frame),indent=2))
    if capture:(out/'frame-times.json').write_text(json.dumps(frame_times,indent=2))
    sim.simulationQuit(0)


if __name__=='__main__':
    try:main()
    except BaseException:
        Path(os.environ['OMNISIM_ASSESSMENT_OUT'],'controller-error.txt').write_text(traceback.format_exc())
        raise
