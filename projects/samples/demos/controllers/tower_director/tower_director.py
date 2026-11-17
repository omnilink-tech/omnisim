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

"""Build and judge a tower. Payload poses are read only; fingers supply all grip forces."""
import json,math,time,sys,subprocess
from pathlib import Path
from omnisim import Supervisor
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'husky_extreme_terrain'))
from husky_extreme_terrain import axis_angle_to_target
CAMERAS={'bench':([3.3,-5.3,3.7],[0,0,.55]),'wide':([4.1,-6.8,3.9],[0,0,1.15]),'front':([.15,-5.8,2.5],[0,0,1.22]),
         'low':([1.5,-3,1.25],[0,0,.8]),'pick':([-2.1,-2.7,1.7],[-.85,0,.5]),
         'high':([1.35,-2.9,2.9],[0,0,1.9])}

def main():
    r=Supervisor();dt=int(r.getBasicTimeStep());c=json.loads(r.getCustomData());out=Path(c['run_dir'])
    frames=out/'frames'
    if c['capture']:frames.mkdir(exist_ok=True)
    r.step(dt);r.step(dt)
    blocks=[r.getFromDef(f'BLOCK_{i:02d}') for i in range(20)]
    robots={k:r.getFromDef('ROBOT_'+k) for k in 'AB'}
    wrists={k:r.getFromDef(k+'_X') for k in 'AB'}
    pads={k:[r.getFromDef(k+'_FINGER_L'),r.getFromDef(k+'_FINGER_R')] for k in 'AB'}
    base={'A':-1.45,'B':1.45};commands={k:{'x':0,'y':0,'z':0,'finger_l':0,'finger_r':0} for k in 'AB'}
    children=r.getRoot().getField('children');view=next(children.getMFNode(i) for i in range(children.getCount()) if children.getMFNode(i).getTypeName()=='Viewpoint')
    events=[];telemetry=[];index=[];placed=[];i=0;phase='wait';phase_t=0;nextsample=0;nextframe=c['start'];finish=None;finish_t=None
    started=time.time();frame_n=0;applied=None;target=None;grip_offset=[0,0,0];peak=0;airborne_checks=[]
    last_frame_wall=0;next_thermal_check=0
    def emit(name,**kw):
        event={'t':round(r.getTime(),4),'event':name,'block':i+1,'robot':'AB'[i%2],**kw};events.append(event)
        with (out/'progress.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
    def change(p):
        nonlocal phase,phase_t
        phase=p;phase_t=r.getTime();emit(p)
    def move(rid,xyz,closed=False):
        commands[rid].update(x=xyz[0]-base[rid],y=xyz[1],z=xyz[2]-.65,
                            finger_l=-.034 if closed else 0,finger_r=.034 if closed else 0)
    while r.step(dt)!=-1:
        now=time.monotonic()
        if now>=next_thermal_check and c.get('thermal_pause_c'):
            try:
                def gpu_temp():return int(subprocess.run(['nvidia-smi','--query-gpu=temperature.gpu','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=5).stdout.strip().splitlines()[0])
                temp=gpu_temp()
                with (out/'thermal.jsonl').open('a') as f:f.write(json.dumps({'wall':time.time(),'gpu_c':temp})+'\n')
                if temp>=c['thermal_pause_c']:
                    while temp>c['thermal_resume_c']:
                        time.sleep(15);temp=gpu_temp()
                        with (out/'thermal.jsonl').open('a') as f:f.write(json.dumps({'wall':time.time(),'gpu_c':temp,'cooling_pause':True})+'\n')
            except (OSError,ValueError,IndexError,subprocess.TimeoutExpired):pass
            next_thermal_check=time.monotonic()+15
        t=r.getTime();poses=[list(b.getPosition()) for b in blocks]
        rid='AB'[i%2];wp=list(wrists[rid].getPosition());p=poses[min(i,19)]
        elapsed=t-phase_t
        current_height=max([poses[j][2]+.05-.35 for j in placed if math.hypot(*poses[j][:2])<.3]+[0]);peak=max(peak,current_height)
        def level(j):return (j//2 if j<6 else j-3) if c['mode']=='buttress' else j
        toppled=[j for j in placed if poses[j][2]<.35+level(j)*.1-.04 or math.hypot(*poses[j][:2])>.38]
        if not finish and toppled:
            finish='collapse';finish_t=t;emit('collapse',fallen=[j+1 for j in toppled]);change('finished')
        if not finish:
            if phase=='wait' and t>=c['start_delay']:
                target=[p[0],p[1],.65];move(rid,target);change('above_supply')
            elif phase=='above_supply' and math.dist(wp,target)<.008 and elapsed>.4:
                target=[p[0],p[1],p[2]+.005];move(rid,target);change('descend')
            elif phase=='descend' and math.dist(wp,target)<.006 and elapsed>.3:
                move(rid,target,c['mode']!='no_grip');change('squeeze')
            elif phase=='squeeze' and elapsed>1.8:
                grip_offset=[p[k]-wp[k] for k in range(3)];target=[wp[0],wp[1],max(.72,.65+i*.1)]
                move(rid,target,c['mode']!='no_grip');change('lift')
            elif phase=='lift' and math.dist(wp,target)<.009 and elapsed>.5:
                bc={tuple(round(v,4) for v in q.point) for q in blocks[i].getContactPoints()}
                hits=[len(bc & {tuple(round(v,4) for v in q.point) for q in pad.getContactPoints()}) for pad in pads[rid]]
                airborne_checks.append({'block':i+1,'t':t,'height':p[2],'pad_contacts':hits,'offset':grip_offset})
                emit('lift_check',height=p[2],pad_contacts=hits)
                if p[2]<.52 or min(hits)==0 or math.dist(p,wp)>.025:
                    finish='missed_grip';finish_t=t;change('finished')
                else:
                    xy=[0,0]
                    if c['mode']=='buttress' and i<6:xy=[-.080 if i%2==0 else .080,0]
                    if c['mode']=='relative' and placed:xy=poses[placed[-1]][:2]
                    xy[0]+=c['bias']
                    target=[xy[0]-grip_offset[0],xy[1]-grip_offset[1],wp[2]]
                    move(rid,target,True);change('carry')
            elif phase=='carry' and math.dist(wp,target)<.008 and elapsed>.5:
                actual_offset=[p[k]-wp[k] for k in range(3)]
                emit('placement_check',initial_grip_offset=grip_offset,actual_grip_offset=actual_offset,block_position=p,wrist_position=wp)
                if c['mode'] in ('feedback','buttress'):grip_offset=actual_offset
                h=(poses[placed[-1]][2]+.1 if placed else .4)
                if c['mode']=='buttress' and i<6:h=.4 if i<2 else poses[i-2][2]+.1
                target[2]=h-grip_offset[2]+(.075 if c['mode']=='drop' else .003 if c['mode'] in ('feedback','buttress') else .001)
                move(rid,target,True);change('place')
            elif phase=='place' and math.dist(wp,target)<.006 and elapsed>.4:
                move(rid,target,False);change('release')
            elif phase=='release' and elapsed>1.8:
                placed.append(i);emit('placed',position=p)
                target=[wp[0],wp[1],wp[2]+.22];move(rid,target);change('retract')
            elif phase=='retract' and math.dist(wp,target)<.009 and elapsed>.3:
                target=[base[rid],.80,max(.65,wp[2])];move(rid,target);change('park')
            elif phase=='park' and math.dist(wp,target)<.012 and elapsed>.4:
                if i+1>=c['count']:change('countdown')
                else:i+=1;change('wait')
            elif phase=='countdown' and elapsed>=10:
                finish='standing';finish_t=t;emit('standing',height_m=current_height);change('finished')
            if elapsed>35 and phase!='wait':finish='motion_timeout';finish_t=t;emit('timeout',phase=phase,actual=wp,target=target);change('finished')
        r.setCustomData(json.dumps(c|{'commands':commands}))
        state={'t':round(t,4),'phase':phase,'block':i+1,'placed':len(placed),'height':current_height,'blocks':poses,'wrists':{k:list(w.getPosition()) for k,w in wrists.items()}}
        if t>=nextsample:telemetry.append(state);nextsample=t+.08
        camera=c['camera']
        if camera=='story':
            camera='wide' if i<6 else 'front' if i<14 else 'high' if i<18 else 'front'
            if i==19 and phase in ('carry','place','release'):camera='high'
            if phase in ('countdown','finished'):camera='front'
        if c['capture'] and camera!=applied:
            eye,aim=CAMERAS[camera];view.getField('position').setSFVec3f(eye);view.getField('orientation').setSFRotation(axis_angle_to_target(tuple(eye),tuple(aim)));applied=camera
        if c['capture'] and nextframe<=t<=c['end']+.032:
            cadence=(1 if phase in ('countdown','finished') else 2 if len(placed)>=18 else 12) if c.get('adaptive') else c['speed']
            time.sleep(max(0,c.get('wall_frame_interval',0)-(time.monotonic()-last_frame_wall)))
            r.exportImage(str(frames/f'frame_{frame_n:06d}.png'),90);index.append(state|{'frame':frame_n,'camera':camera,'speed':cadence});frame_n+=1;nextframe+=.032*cadence
            last_frame_wall=time.monotonic()
            with (out/'capture_index.jsonl').open('a') as f:f.write(json.dumps(index[-1])+'\n')
        if (finish and t-finish_t>=5) or t>c['end']:
            result={'config':c,'outcome':finish or 'capture_window','finish_time':finish_t,'placed':len(placed),'height_m':current_height,'peak_height_m':peak,
                    'events':events,'telemetry':telemetry,'airborne_checks':airborne_checks,'frame_count':frame_n,'wall_time_s':time.time()-started,
                    'payload_pose_writes':False,'grasp':'physical opposing finger contacts; no attachments','controller_inputs':'Exact simulated block and wrist poses, central alternating turn order'}
            (out/'result.json').write_text(json.dumps(result,indent=2));(out/'capture_index.json').write_text(json.dumps(index));r.simulationQuit(0);return

if __name__=='__main__':main()
