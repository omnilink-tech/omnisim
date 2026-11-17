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
CAMERAS={'wide':([5.2,-8.4,5.2],[0,0,1.15]),'front':([.15,-7.3,3.45],[0,0,1.58]),'oblique':([4.5,-7.2,4.2],[0,0,1.5]),
         'bench':([4.5,-6.1,4.1],[0,0,.62]),'low':([1.8,-3.4,1.65],[0,0,.95]),
         'high':([1.35,-3.8,3.65],[0,0,2.25]),'pick':([-1.15,-2.15,1.2],[-.65,-1.1025,.57]),
         'clearance':([.7,-1.5,1.15],[0,0,.43])}

ACTIVE_SUPERVISOR=None

def main():
    global ACTIVE_SUPERVISOR
    r=Supervisor();dt=int(r.getBasicTimeStep());c=json.loads(r.getCustomData());out=Path(c['run_dir'])
    ACTIVE_SUPERVISOR=r
    out.mkdir(parents=True,exist_ok=True);frames=out/'frames'
    if c['capture']:frames.mkdir(exist_ok=True)
    r.step(dt);r.step(dt)
    blocks=[r.getFromDef(f'BLOCK_{i:02d}') for i in range(100)]
    robots={k:r.getFromDef('ROBOT_'+k) for k in 'AB'}
    wrists={k:r.getFromDef(k+'_X') for k in 'AB'}
    pads={k:[r.getFromDef(k+'_FINGER_L'),r.getFromDef(k+'_FINGER_R')] for k in 'AB'}
    tracked=not c.get('legacy_reads',False);pose_checks=[];control_ms=c.get('control_ms',32);positions={}
    if tracked:
        # Boxes are root Solids: their translation fields are world positions.
        # Subscribe to those three values rather than 100 full pose matrices.
        for node in blocks:
            field=node.getField('translation');field.enableSFTracking(control_ms);positions[id(node)]=field
        for node in wrists.values():node.enablePoseTracking(0)
    def position(node):
        if not tracked:return list(node.getPosition())
        if id(node) in positions:return list(positions[id(node)].getSFVec3f())
        matrix=node.getPose();return [matrix[3],matrix[7],matrix[11]]
    def check_pose(node,label):
        cached=position(node);direct=list(node.getPosition());error=max(abs(a-b) for a,b in zip(cached,direct))
        pose_checks.append({'t':r.getTime(),'node':label,'max_error_m':error})
        if error>1e-5:raise RuntimeError(f'Pose subscription differs from synchronous position: {label}: {error}')
    base={'A':-2.1,'B':2.1};commands={k:{'x':0,'y':0,'z':0,'finger_l':0,'finger_r':0} for k in 'AB'}
    children=r.getRoot().getField('children');view=next(children.getMFNode(i) for i in range(children.getCount()) if children.getMFNode(i).getTypeName()=='Viewpoint')
    events=[];telemetry=[];index=[];placed=[];i=0;phase='wait';phase_t=0;nextsample=0;nextframe=c['start'];finish=None;finish_t=None
    started=time.time();frame_n=0;applied=None;target=None;grip_offset=[0,0,0];peak=0;airborne_checks=[]
    last_frame_wall=0;next_thermal_check=0;next_live=0
    def emit(name,**kw):
        event={'t':round(r.getTime(),4),'event':name,'block':i+1,'robot':'AB'[i%2],**kw};events.append(event)
        with (out/'progress.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
    def change(p):
        nonlocal phase,phase_t
        phase=p;phase_t=r.getTime();emit(p)
    def move(rid,xyz,closed=False):
        close=.012 if c['slim'] else .034
        commands[rid].update(x=xyz[0]-base[rid],y=xyz[1],z=xyz[2]-.65,
                            finger_l=-close if closed else 0,finger_r=close if closed else 0)
    while r.step(control_ms)!=-1:
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
        t=r.getTime();poses=[position(b) for b in blocks]
        rid='AB'[i%2];wp=position(wrists[rid]);p=poses[min(i,99)]
        elapsed=t-phase_t
        current_height=max([poses[j][2]+.05-.35 for j in placed if math.hypot(*poses[j][:2])<.3]+[0]);peak=max(peak,current_height)
        def level(j):return (j//2 if j<6 else j-3) if c['mode']=='buttress' else j//4
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
                grip_offset=[p[k]-wp[k] for k in range(3)];target=[wp[0],wp[1],max(.72,.65+level(i)*.1)]
                if target[2]>3.64:
                    finish='reach_limit';finish_t=t;emit('reach_limit',required_height=target[2],wrist_limit=3.65);change('finished')
                else:move(rid,target,c['mode']!='no_grip');change('lift')
            elif phase=='lift' and math.dist(wp,target)<.009 and elapsed>.5:
                if tracked:
                    check_pose(blocks[i],f'BLOCK_{i:02d}');check_pose(wrists[rid],rid+'_X')
                bc={tuple(round(v,4) for v in q.point) for q in blocks[i].getContactPoints()}
                hits=[len(bc & {tuple(round(v,4) for v in q.point) for q in pad.getContactPoints()}) for pad in pads[rid]]
                airborne_checks.append({'block':i+1,'t':t,'height':p[2],'pad_contacts':hits,'offset':grip_offset})
                emit('lift_check',height=p[2],pad_contacts=hits)
                if p[2]<.52 or min(hits)==0 or math.dist(p,wp)>.025:
                    finish='missed_grip';finish_t=t;change('finished')
                else:
                    xy=[0,0]
                    if c['mode']=='buttress' and i<6:xy=[-.080 if i%2==0 else .080,0]
                    if c['mode']=='quad':xy=poses[i-4][:2] if i>=4 else [(-.08 if i%2==0 else .08),(-.09 if i%4<2 else .09)]
                    if c['mode']=='quad' and c.get('bond_offset',0):
                        # Alternate the complete layer around the tower center.
                        # Motor targets bridge the seams; payloads remain free.
                        offset=c['bond_offset']*(1 if level(i)%2 else -1)
                        xy=[(-.08 if i%2==0 else .08)+offset,(-.09 if i%4<2 else .09)+offset]
                    target=[xy[0]-grip_offset[0],xy[1]-grip_offset[1],wp[2]]
                    move(rid,target,True);change('carry')
            elif phase=='carry' and math.dist(wp,target)<.008 and elapsed>.5:
                actual_offset=[p[k]-wp[k] for k in range(3)]
                emit('placement_check',initial_grip_offset=grip_offset,actual_grip_offset=actual_offset,block_position=p,wrist_position=wp)
                grip_offset=actual_offset
                h=(poses[placed[-1]][2]+.1 if placed else .4)
                if c['mode']=='buttress' and i<6:h=.4 if i<2 else poses[i-2][2]+.1
                if c['mode']=='quad':h=poses[i-4][2]+.1 if i>=4 else .4
                if c['mode']=='quad' and c.get('bond_offset',0) and i>=4:
                    support=[poses[j][2] for j in placed if level(j)==level(i)-1 and abs(poses[j][0]-p[0])<.155 and abs(poses[j][1]-p[1])<.155]
                    if not support:raise RuntimeError('No overlapping support under the requested placement')
                    h=max(support)+.1
                target[2]=h-grip_offset[2]+.003
                move(rid,target,True);change('place')
            elif phase=='place' and math.dist(wp,target)<.006 and elapsed>.4:
                move(rid,target,False);change('release')
            elif phase=='release' and elapsed>1.8:
                placed.append(i);emit('placed',position=p)
                target=[wp[0],wp[1],wp[2]+.22];move(rid,target);change('retract')
            elif phase=='retract' and math.dist(wp,target)<.009 and elapsed>.3:
                target=[base[rid],1.5,max(.65,wp[2])];move(rid,target);change('park')
            elif phase=='park' and math.dist(wp,target)<.012 and elapsed>.4:
                if i+1>=c['count']:change('countdown')
                else:i+=1;change('wait')
            elif phase=='countdown' and elapsed>=10:
                finish='standing';finish_t=t;emit('standing',height_m=current_height);change('finished')
            if elapsed>35 and phase!='wait':finish='motion_timeout';finish_t=t;emit('timeout',phase=phase,actual=wp,target=target);change('finished')
        r.setCustomData(json.dumps(c|{'commands':commands}))
        state={'t':round(t,4),'phase':phase,'block':i+1,'placed':len(placed),'height':current_height,'blocks':poses,'wrists':{k:position(w) for k,w in wrists.items()}}
        if t>=nextsample:telemetry.append(state);nextsample=t+.2
        if t>=next_live:
            (out/'live.json').write_text(json.dumps({k:state[k] for k in ['t','phase','block','placed','height']}));next_live=t+2
        camera=c['camera']
        if camera=='story':
            camera='wide' if i<12 else 'low' if i<28 else 'front' if i<50 else 'oblique' if i<64 else 'front' if i<76 else 'high' if i<92 else 'front'
            if i==99 and phase in ('carry','place','release'):camera='high'
            if phase in ('countdown','finished'):camera='oblique'
        if c['capture'] and camera!=applied:
            eye,aim=CAMERAS[camera];view.getField('position').setSFVec3f(eye);view.getField('orientation').setSFRotation(axis_angle_to_target(tuple(eye),tuple(aim)));applied=camera
        if c['capture'] and nextframe<=t<=c['end']+.032:
            cadence=(1 if phase in ('countdown','finished') or (i==99 and phase in ('carry','place','release','retract')) else 4 if i==99 else 8 if len(placed)>=96 else c['speed']) if c.get('adaptive') else c['speed']
            time.sleep(max(0,c.get('wall_frame_interval',0)-(time.monotonic()-last_frame_wall)))
            r.exportImage(str(frames/f'frame_{frame_n:06d}.png'),90);index.append(state|{'frame':frame_n,'camera':camera,'speed':cadence});frame_n+=1;nextframe+=.032*cadence
            last_frame_wall=time.monotonic()
            with (out/'capture_index.jsonl').open('a') as f:f.write(json.dumps(index[-1])+'\n')
        if (finish and t-finish_t>=5) or t>c['end']:
            result={'config':c,'outcome':finish or 'capture_window','finish_time':finish_t,'placed':len(placed),'height_m':current_height,'peak_height_m':peak,
                    'events':events,'telemetry':telemetry,'airborne_checks':airborne_checks,'frame_count':frame_n,'wall_time_s':time.time()-started,
                    'tracked_pose_reads':tracked,'director_period_ms':control_ms,'physics_period_ms':dt,'pose_cache_checks':pose_checks,
                    'payload_pose_writes':False,'grasp':'physical opposing finger contacts; no attachments','controller_inputs':'Exact simulated block and wrist poses, central alternating turn order'}
            (out/'result.json').write_text(json.dumps(result,indent=2));(out/'capture_index.json').write_text(json.dumps(index));r.simulationQuit(0);return

if __name__=='__main__':
    try:main()
    except Exception:
        if ACTIVE_SUPERVISOR is not None:ACTIVE_SUPERVISOR.simulationQuit(1)
        raise
