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

"""Measure the passage experiments, issue permissions, and record main-view footage."""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'husky_extreme_terrain'))
from husky_extreme_terrain import find_link_by_name,axis_angle_to_target
from omnisim import Supervisor

CAMERAS={'wide':([12,-17,16],[0,0,0]),'top':([0,-.01,22],[0,0,0]),
         'door':([.5,-7,8],[0,0,.20]),'west':([-4,-7,6],[-3,0,.2]),'east':([4,-7,6],[3,0,.2]),
         'dock':([-7,-5,8],[-4.5,2.5,.1])}

def main():
    r=Supervisor();dt=int(r.getBasicTimeStep());config=json.loads(r.getCustomData())
    out=Path(config['run_dir']);out.mkdir(parents=True,exist_ok=True)
    capture=config['capture'];frames=out/'frames'
    if capture:frames.mkdir(exist_ok=True)
    r.step(dt);r.step(dt)
    nodes={rid:r.getFromDef('ROBOT_'+rid) for rid in 'ABCD'[:config['count']]}
    poses={rid:find_link_by_name(n,'base_link') or n for rid,n in nodes.items()}
    children=r.getRoot().getField('children');view=None
    for i in range(children.getCount()):
        n=children.getMFNode(i)
        if n.getTypeName()=='Viewpoint':view=n;break
    camera=config['camera'];applied=None
    start=time.time();telemetry=[];events=[];capture_index=[];admitted=[];owner=None;released=[]
    next_sample=0;next_frame=config['capture_start'];frame_n=0;finish=None;finish_t=None
    min_sep=100;previous={};last_move=3;contact_ticks=0;active_contacts=set()
    shared=r.getSelf().getField('customData')
    while r.step(dt)!=-1:
        t=r.getTime()
        state={}
        for rid,p in poses.items():
            xyz=p.getPosition();rot=p.getOrientation()
            try:s=json.loads(nodes[rid].getField('customData').getSFString() or '{}')
            except ValueError:s={}
            state[rid]=dict(x=xyz[0],y=xyz[1],z=xyz[2],yaw=math.atan2(rot[3],rot[0]),**s)
        close_pairs=[]
        for i,a in enumerate(state):
            for b in list(state)[i+1:]:
                d=math.hypot(state[a]['x']-state[b]['x'],state[a]['y']-state[b]['y'])
                min_sep=min(min_sep,d)
                if d<1.5:close_pairs.append((a,b))
        touches=set()
        for a,b in close_pairs:
            # Reciprocal main-physics contact positions establish robot/robot
            # contact; proximity alone is not a collision measurement.
            ca={tuple(round(v,4) for v in c.point) for c in nodes[a].getContactPoints(True)}
            cb={tuple(round(v,4) for v in c.point) for c in nodes[b].getContactPoints(True)}
            shared_points=ca & cb
            if shared_points:
                touches.add((a,b));contact_ticks+=1
                if (a,b) not in active_contacts:
                    events.append({'t':round(t,3),'event':'robot_contact','robots':[a,b],'points':list(shared_points)})
        active_contacts=touches
        if config['mode'] in ('early','clear') and not finish:
            if owner:
                s=state[owner];direction=1 if owner in ('A','C') else -1
                free=(s['x']*direction>0 if config['mode']=='early' else s.get('wp',0)>=4)
                if free:
                    events.append({'t':round(t,3),'event':'release','robot':owner,'x':s['x'],'y':s['y'],'wp':s.get('wp')})
                    released.append(owner);owner=None
            if owner is None:
                waiting=[rid for rid in state if rid not in admitted]
                if waiting:
                    owner=waiting[0];admitted.append(owner)
                    events.append({'t':round(t,3),'event':'grant','robot':owner})
        shared.setSFString(json.dumps(dict(config,owner=owner,admitted=admitted,halt=finish is not None)))
        if t>=next_sample:
            delta=sum(math.hypot(s['x']-previous.get(k,s)['x'],s['y']-previous.get(k,s)['y']) for k,s in state.items())
            if delta>.025:last_move=t
            previous=state
            telemetry.append({'t':round(t,3),'owner':owner,'robots':state})
            next_sample=t+.16
        outcome=None
        if all(s.get('done',False) for s in state.values()):outcome='success'
        elif t>config['time_limit']:outcome='timeout'
        elif t>12 and t-last_move>7:outcome='deadlock'
        if outcome and finish is None:
            finish=outcome;finish_t=t
            events.append({'t':round(t,3),'event':outcome})
        shot=camera
        if camera=='story':
            shot='wide' if t<7 else ('door' if t<28 else ('east' if t<48 else 'wide'))
        if camera=='final':shot='top' if t<136 else 'dock'
        if capture and view and shot!=applied:
            eye,target=CAMERAS[shot]
            view.getField('position').setSFVec3f(eye)
            view.getField('orientation').setSFRotation(axis_angle_to_target(tuple(eye),tuple(target)))
            applied=shot
            events.append({'t':round(t,3),'event':'camera','shot':shot,'position':eye,'target':target})
        if capture and next_frame<=t<=config['capture_end']+.032:
            r.exportImage(str(frames/f'frame_{frame_n:06d}.png'),90)
            capture_index.append({'frame':frame_n,'t':round(t,4),'camera':shot,'robots':state})
            frame_n+=1;next_frame+=.032*config['capture_speed']
        end_capture=capture and t>config['capture_end']
        if (finish is not None and t-finish_t>=5) or end_capture:
            result=dict(config,outcome=finish or 'capture_window',sim_time_s=round(t,3),finish_time_s=finish_t,
                        completed=[k for k,s in state.items() if s.get('done')],min_center_distance_m=min_sep,
                        final=state,events=events,telemetry=telemetry,frame_count=frame_n,
                        controller_inputs=['exact simulated poses','known routes','central passage permission'],
                        pose_writes=False,robot_contact_pair_ticks=contact_ticks,
                        contact_measurement='Reciprocal contact positions from robot subtree queries every physics tick when centers are within 1.5 m.',
                        wall_time_s=round(time.time()-start,2))
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            if capture:(out/'capture_index.json').write_text(json.dumps(capture_index)+'\n')
            print('[passage] RESULT '+json.dumps({k:v for k,v in result.items() if k not in ['events','telemetry']}),flush=True)
            r.simulationQuit(0);return

if __name__=='__main__':main()
