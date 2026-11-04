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

"""Scoped physical Go2 handover and G1 squat assessments; no training or pose support.

Source and runtime are fingerprinted per attempt. Invalid runs are ERROR, never FAIL.
"""
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import uuid
from .registry import ROOT
from .runner import digest, read_json, save, _reap_owned, runtime_hashes
from omnisim.paths import resolve_omnisim_binary, linux_runtime_env


@dataclass(frozen=True)
class LeggedSpec:
    robot: str
    duration_s: float = 32.
    handover_s: float = 12.
    support_gate: bool = True
    squat_ff_ank: float = .08
    squat_rise_s: float = 5.5
    squat_descend_s: float = 3.5
    ground_mu: float = 2.
    g1_task: str = 'squat'
    ankle_bias: float = -.06
    gate_mode: str = 'contact'
    morph_ticks: int = 30

    def __post_init__(self):
        if self.robot not in ('go2','g1'):
            raise ValueError('Legged tasks are Go2 walk/stand/walk and G1 squat/return only')
        for key,value in asdict(self).items():
            if key in ('robot','support_gate','g1_task','gate_mode'): continue
            if isinstance(value,bool) or not math.isfinite(value): raise ValueError('Finite task parameters required')
        if not 20 <= self.duration_s <= 60 or not 5 <= self.handover_s <= 15:
            raise ValueError('Task duration outside the bounded reference scope')
        if not 0 <= self.squat_ff_ank <= .2 or not 2 <= self.squat_rise_s <= 8 or not 2 <= self.squat_descend_s <= 5:
            raise ValueError('Squat tuning outside reference scope')
        if self.g1_task not in ('stand','squat') or not -.2<=self.ankle_bias<=.1: raise ValueError('Invalid standing task or ankle trim')
        if self.robot=='g1' and self.g1_task=='squat' and self.duration_s < 3+2*(self.squat_descend_s+1+self.squat_rise_s+1.2):
            raise ValueError('Observation must cover two complete squats and the return hold')
        if not .5 <= self.ground_mu <= 2: raise ValueError('Contact coefficient outside reference scope')
        if type(self.support_gate) is not bool or self.gate_mode not in ('phase','contact','level'): raise ValueError('Invalid handover guard')
        if type(self.morph_ticks) is not int or not 1<=self.morph_ticks<=120: raise ValueError('Blend must be 1 to 120 ticks')


def grade(spec, rows, completion, sidecar, controller_log):
    invalid=[]
    if not completion.get('complete') or not rows: invalid.append('incomplete_episode')
    if not sidecar: invalid.append('physics_not_finalized')
    if rows:
        if rows[-1]['t_s'] < spec.duration_s-.02: invalid.append('observation_truncated')
        if any(abs(b['t_s']-a['t_s']-.016)>.001 for a,b in zip(rows,rows[1:])): invalid.append('trace_discontinuity')
    if spec.robot=='go2' and 'ONNX loaded:' not in controller_log: invalid.append('policy_not_loaded')
    if 'FATAL' in controller_log or 'Traceback' in controller_log: invalid.append('controller_error')
    if invalid: return dict(status='ERROR',reasons=invalid,metrics={})
    active=[r for r in rows if r['t_s'] >= (2. if spec.robot=='go2' else 1.)]
    fall=[r for r in active if abs(r['roll_rad'])>.8 or abs(r['pitch_rad'])>.8 or r['position_m'][2] < (.18 if spec.robot=='go2' else .38)]
    reasons=[]
    if fall: reasons.append('fell_or_lost_upright_posture')
    metrics=dict(first_fall_s=fall[0]['t_s'] if fall else None,
                 max_tilt_rad=max(r['tilt_rad'] for r in active),
                 final_height_m=rows[-1]['position_m'][2],
                 contact_sample_fraction=sum(r['contacts']>0 for r in active)/len(active),
                 observation_s=rows[-1]['t_s'], assist='none', pose_writes=False)
    if spec.robot=='go2':
        switches=len(re.findall(r'BATON switch',controller_log))
        metrics.update(switches=switches, forward_progress_m=rows[-1]['position_m'][0]-rows[0]['position_m'][0])
        if switches<2: reasons.append('walk_stand_walk_not_completed')
        if metrics['forward_progress_m']<1.: reasons.append('insufficient_forward_progress')
        if spec.gate_mode=='level':
            changes=re.findall(r'HANDOVER mode=(stand|walk) sim_time_s=([0-9.]+)',controller_log)
            if len(changes)!=2 or [mode for mode,_ in changes]!=['stand','walk']:
                return dict(status='ERROR',reasons=['handover_timing_evidence_missing'],metrics=metrics)
            standing,walking=[float(t) for _,t in changes]
            resumed=min(rows,key=lambda row:abs(row['t_s']-walking))
            metrics.update(stand_window_s=walking-standing,return_walk_window_s=rows[-1]['t_s']-walking,
                           return_walk_progress_m=rows[-1]['position_m'][0]-resumed['position_m'][0],
                           stand_started_s=standing,walk_resumed_s=walking)
            if metrics['stand_window_s']<5.:reasons.append('standing_window_under_5s')
            if metrics['return_walk_window_s']<3. or metrics['return_walk_progress_m']<.5:
                reasons.append('return_walk_not_demonstrated')
    elif spec.g1_task=='squat':
        done=re.search(r'SQUAT_DONE reps=(\d+).*final upright=True -> SUCCESS',controller_log)
        start=[r['position_m'][2] for r in rows if 1<=r['t_s']<=1.8]
        nominal=sum(start)/len(start)
        depth=nominal-min(r['position_m'][2] for r in active)
        end=[r for r in rows if r['t_s']>=spec.duration_s-1]
        metrics.update(squat_depth_m=depth, nominal_height_m=nominal,
                       return_height_error_m=abs(sum(r['position_m'][2] for r in end)/len(end)-nominal),
                       completed_reps=int(done[1]) if done else 0)
        if not done: reasons.append('two_squats_and_return_not_completed')
        if not .06<=depth<=.25: reasons.append('squat_depth_outside_6_to_25_cm')
        if metrics['return_height_error_m']>.05: reasons.append('did_not_return_to_standing_height')
        if max(r['tilt_rad'] for r in end)>.2: reasons.append('return_not_upright')
    else:
        end=[r for r in rows if r['t_s']>=spec.duration_s-2]
        def foot_only(row):
            names=row.get('contact_nodes',[])
            return any('left_ankle_roll' in n for n in names) and any('right_ankle_roll' in n for n in names) and all('ankle_roll' in n for n in names)
        metrics.update(max_horizontal_drift_m=max(math.hypot(r['position_m'][0],r['position_m'][1]) for r in active),
                       final_tilt_rad=max(r['tilt_rad'] for r in end),
                       final_two_foot_contact_fraction=sum(foot_only(r) for r in end)/len(end))
        if metrics['final_tilt_rad']>.2: reasons.append('standing_not_upright')
        if metrics['final_height_m']<.55: reasons.append('standing_height_not_maintained')
        if metrics['max_horizontal_drift_m']>.25: reasons.append('standing_drift_exceeded_25_cm')
        if metrics['final_two_foot_contact_fraction']<.9: reasons.append('two_foot_support_not_maintained')
    return dict(status='FAIL' if reasons else 'PASS',reasons=reasons,metrics=metrics)


def inputs(robot):
    common=[ROOT/'projects/assessments/controllers/legged_observer/legged_observer.py']+list((ROOT/'omnisim/assessment').glob('*.py'))
    if robot=='go2':
        common.extend((ROOT/'projects/assessments/controllers/legged_go2').glob('*.py'))
        paths=['projects/policies/research/controllers/go2_baton_deploy',
               'projects/policies/control', 'projects/policies/training/baton.py','omnisim/policy',
               'projects/policies/research/inference/policies/gpu_go2_shadow_main',
               'projects/policies/ghosts/go2', 'projects/robots/unitree/go2',
               'projects/policies/research/worlds/go2_baton_deploy.omniworld']
    else:
        paths=['projects/policies/controllers/humanoid_stand_deploy','projects/robots/unitree/g1/urdf',
               'projects/policies/worlds/g1_hstand_deploy.omniworld']
    paths+=['resources/web/wwi/protoVisualizer/templating',
            'projects/objects/floors/protos/RectangleArena.proto','projects/objects/floors/protos/Floor.proto',
            'projects/appearances/protos/Parquetry.proto','projects/appearances/protos/BrushedAluminium.proto']
    for path in paths:
        p=ROOT/path
        common.extend(q for q in p.rglob('*') if q.is_file() and q.suffix not in ('.pyc','.pt')) if p.is_dir() else common.append(p)
    return {p.relative_to(ROOT).as_posix():digest(p) for p in common if '__pycache__' not in p.parts}


def world(spec):
    original=ROOT/('projects/policies/research/worlds/go2_baton_deploy.omniworld' if spec.robot=='go2' else 'projects/policies/worlds/g1_hstand_deploy.omniworld')
    text=original.read_text()
    text=re.sub(r'url "([^"]+)"',lambda m:f'url "{(original.parent/m[1]).resolve().as_posix()}"',text)
    text=text.replace('URDFRobot {','DEF ASSESSMENT_ROBOT URDFRobot {')
    if spec.robot=='go2':text=text.replace('controller "go2_baton_deploy"','controller "legged_go2"')
    text=text.replace('WorldInfo {',f'WorldInfo {{ newtonGroundMu {spec.ground_mu} newtonSubsteps {8 if spec.robot=="go2" else 4}')
    # Fixed, identical camera for every variant; all transforms remain physical.
    # +X-forward / +Z-up camera basis, aimed at the robot from offset (2.8,-3,2.4).
    text=re.sub(r'Viewpoint \{.*?\}', 'Viewpoint { orientation -0.2598510411 0.112918868 0.9590238609 2.3518751779 position 2.8 -3 3.1 }',text,flags=re.S)
    return text+'\nRobot { name "assessment observer" supervisor TRUE controller "legged_observer" }\n'


def run(spec, directory, capture=False, wall_timeout_s=180):
    if capture: wall_timeout_s=max(wall_timeout_s,600)
    directory=Path(directory).resolve()
    binary=Path(os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary() or '').resolve()
    if not binary.is_file(): raise ValueError('Run python -m omnisim doctor first')
    source=inputs(spec.robot)
    directory.mkdir(parents=True,exist_ok=False)
    save(directory/'spec.json',asdict(spec))
    contents=world(spec)
    (directory/'world.omniworld').write_text(contents)
    stage=ROOT/f'projects/assessments/worlds/.legged_{uuid.uuid4().hex}.omniworld'
    stage.write_text(contents)
    env=linux_runtime_env(ROOT)
    for key in list(env):
        if key.startswith(('OMNISIM_NEWTON_','OMNISIM_INENGINE_','HSTAND_','HUMANOID_','GO2_','BATON_','OMNISIM_ASSESSMENT_')) or key=='OMNISIM_NO_WINDOW': env.pop(key)
    env.update(OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(directory/'engine.log'),
               OMNISIM_NEWTON_DUMP_MJMODEL=str(directory/'compiled-model.txt'),
               OMNISIM_NEWTON_SEED_POSE='1',OMNISIM_NEWTON_SEED_REBUILD='1',OMNISIM_URDF_USE_INERTIA='1',
               OMNISIM_ASSESSMENT_OUT=str(directory),OMNISIM_ASSESSMENT_CAPTURE='1' if capture else '0',
               WEBOTS_EXTRA_PROJECT_PATH=os.pathsep.join([str(ROOT/'projects/policies'),str(ROOT/'projects/policies/research')]))
    if spec.robot=='go2':
        env.update(OMNISIM_NEWTON_TARGET_KE='250',OMNISIM_NEWTON_TARGET_KD='6',
                   GO2_GHOST_LUT=str(ROOT/'projects/policies/ghosts/go2/go2_shadow_ghost_lut.json'),
                   GO2_STAND_LUT=str(ROOT/'projects/policies/ghosts/go2/go2_stand_ghost_lut.json'),
                   GO2_POLICY_ONNX=str(ROOT/'projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx'),
                   GO2_DEPLOY_LOG=str(directory/'controller.log'),GO2_GAIT_VX='.4',GO2_GAIT_FREQ='1.8',
                   GO2_GAIT_DUTY='.6',GO2_GAIT_STEP_H='.05',GO2_GAIT_BODY_H='.30',GO2_GAIT_RAMP_S='1',
                   GO2_ACT_SCALE='.15',GO2_GHOST_FF='1',BATON_SCHEDULE=f'walk:{spec.handover_s},stand:6,walk:20',
                   GO2_ASSESS_GATE=spec.gate_mode,
                   BATON_MORPH_TICKS=str(spec.morph_ticks),BATON_DS_GATE='1' if spec.support_gate else '0')
    else:
        env.update(OMNISIM_NEWTON_TARGET_KE='400',OMNISIM_NEWTON_TARGET_KD='60',
                   HUMANOID_STAND_SPEC=str(ROOT/f'projects/policies/controllers/humanoid_stand_deploy/specs/g1{"_squat" if spec.g1_task=="squat" else "_armsdown"}.json'),
                   HUMANOID_STAND_LOG=str(directory/'controller.log'),HSTAND_WARMUP_RELOAD='0',
                   HSTAND_SQUAT='1' if spec.g1_task=='squat' else '0',HSTAND_ANK_BIAS=str(spec.ankle_bias),HSTAND_SQUAT_FF_ANK=str(spec.squat_ff_ank),
                   HSTAND_SQUAT_RISE_S=str(spec.squat_rise_s),HSTAND_SQUAT_DESCEND_S=str(spec.squat_descend_s))
    manifest=dict(task='go2_gait_transition' if spec.robot=='go2' else f'g1_{spec.g1_task}',
                  sources_sha256=source,binary_sha256=digest(binary),runtime_sha256=runtime_hashes(binary),
                  parameters=asdict(spec),runtime_environment={k:v for k,v in env.items() if k.startswith(('GO2_','BATON_','HSTAND_','HUMANOID_','OMNISIM_NEWTON_'))},
                  assist='none; no support harness, no external push, no root pose/velocity writes',capture=capture)
    # Environment is constructed from the allowlist above; no credentials are captured.
    save(directory/'manifest.json',manifest)
    args=[str(binary),'--batch','--mode=fast','--minimize','--stdout','--stderr']
    if not capture: args.append('--no-rendering')
    args.append(str(stage))
    children={};proc=None;error=None
    started=time.monotonic()
    try:
        import psutil
        with (directory/'stdout.log').open('w',encoding='utf-8') as stdout:
            proc=subprocess.Popen(args,cwd=ROOT,env=env,stdout=stdout,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            manifest['owned_pid']=proc.pid
            while proc.poll() is None:
                try:
                    for child in psutil.Process(proc.pid).children(recursive=True):
                        child.create_time();children[child.pid]=child
                except psutil.NoSuchProcess: pass
                if time.monotonic()-started>wall_timeout_s or (directory/'observer-error.txt').exists():
                    error='watchdog_or_observer_error';break
                time.sleep(.1)
            manifest['engine_exit_code']=proc.poll()
    except (OSError,subprocess.SubprocessError) as exc: error=str(exc)
    finally:
        manifest['cleanup_errors']=_reap_owned(proc,children) if proc else []
        stage.unlink(missing_ok=True)
    rows=[json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()] if (directory/'trace.jsonl').exists() else []
    log=(directory/'controller.log').read_text(errors='replace') if (directory/'controller.log').exists() else ''
    result=grade(spec,rows,read_json(directory/'completion.json',{}),read_json(directory/'engine.log.newton.json',{}),log)
    if error or manifest['engine_exit_code']!=0 or manifest['cleanup_errors']:
        result.update(status='ERROR',reasons=result['reasons']+[error or 'engine_exit_or_cleanup_error'])
    manifest.update(wall_elapsed_s=time.monotonic()-started,launch_error=error,
                    artifacts_sha256={p.name:digest(p) for p in directory.iterdir() if p.is_file() and p.name!='manifest.json'})
    save(directory/'manifest.json',manifest)
    save(directory/'verdict.json',result)
    from projects.policies.common.env_fingerprint import collect
    save(directory/'fingerprint.json',collect(engine_log_path=directory/'engine.log',repo_root=ROOT,warm=False))
    return result


def campaign(robot, directory):
    """Frozen discovery grid, selection using only grid data, then fresh confirmations/holdouts."""
    directory=Path(directory).resolve()
    directory.mkdir(parents=True,exist_ok=False)
    sources=inputs(robot)
    for name in sources:
        dest=directory/'source'/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
    state=dict(schema_version=1,robot=robot,task='gait_transition' if robot=='go2' else 'standing_stability',
               complete=False,source_sha256=sources,cases=[],selected_pair=None,
               criteria='32 s observation; no fall >0.8 rad tilt or low root; Go2 two handovers and >1 m progress; G1 <0.2 rad final tilt, <25 cm drift, >55 cm root, >=90% two-foot-only final contacts',
               selection='Candidate selected from grid PASS only; holdouts never used for selection',
               held_out_inputs=[11,13] if robot=='go2' else [48,60])
    def append(spec,kind):
        if inputs(robot)!=sources: raise ValueError('Assessment inputs changed during campaign; retain and restart')
        name=f'case-{len(state["cases"]):03d}'
        result=run(spec,directory/name)
        row=dict(case=name,kind=kind,spec=asdict(spec),**result)
        state['cases'].append(row)
        save(directory/'campaign.json',state)
        print(json.dumps(dict(case=name,kind=kind,status=result['status'])),flush=True)
        return row
    if robot=='go2':
        for handover in (8,10,12,14):
            for gate in (False,True): append(LeggedSpec('go2',handover_s=handover,support_gate=gate,morph_ticks=1 if gate else 30),'grid')
        pairs=[(a,b) for a in state['cases'] for b in state['cases'] if a['status']=='FAIL' and b['status']=='PASS' and not a['spec']['support_gate'] and b['spec']['support_gate'] and a['spec']['handover_s']==b['spec']['handover_s']]
    else:
        for trim in (0,-.02,-.04,-.06,-.08,-.10): append(LeggedSpec('g1',g1_task='stand',ankle_bias=trim),'grid')
        baseline=state['cases'][0]
        passing=[r for r in state['cases'] if r['status']=='PASS']
        passing.sort(key=lambda r:r['metrics']['max_tilt_rad'])
        pairs=[(baseline,p) for p in passing if baseline['status']=='FAIL']
    if not pairs:
        state['reason']='no_qualified_improvement_pair';save(directory/'campaign.json',state);return state
    before,after=pairs[0]
    state['selected_pair']=[before['case'],after['case']]
    for original in (before,after):
        for _ in range(2):
            r=append(LeggedSpec(**original['spec']),'confirmation')
            if r['status']!=original['status']: state['reason']='confirmation_changed_outcome'
    for holdout in state['held_out_inputs']:
        for original in (before,after):
            params=dict(original['spec'])
            params['handover_s' if robot=='go2' else 'duration_s']=holdout
            append(LeggedSpec(**params),'holdout')
    state['complete']=not state.get('reason') and all(c['status']!='ERROR' for c in state['cases'])
    # A measured campaign can contain expected failures, but its selected candidate
    # must also pass every fresh confirmation and held-out challenge before delivery.
    candidate_cases=[c for c in state['cases'] if c['kind'] in ('confirmation','holdout') and (c['spec']['support_gate'] if robot=='go2' else c['spec']['ankle_bias']==after['spec']['ankle_bias'])]
    if any(c['status']!='PASS' for c in candidate_cases):
        state.update(complete=False,reason='candidate_failed_confirmation_or_holdout')
    state['counts']={s:sum(c['status']==s for c in state['cases']) for s in ('PASS','FAIL','ERROR')}
    save(directory/'campaign.json',state)
    return state
