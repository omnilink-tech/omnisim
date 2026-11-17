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

"""Husky mixed-surface braking and path-turning; physical wheels, offline oracle.

Surface coefficients are imposed contact parameters, not calibrated materials.
Every experiment owns a fresh engine. The controller changes only motor targets.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import uuid

from omnisim.paths import REPO_ROOT as ROOT, linux_runtime_env, resolve_omnisim_binary
from .runner import digest, read_json, runtime_hashes, save, _reap_owned
from .schema import BrakingSpec
from .scenarios import make_model
from .registry import get_profile


@dataclass(frozen=True)
class MixedSpec:
    schema_version: int = 1
    robot: str = 'husky'
    task: str = 'brake'
    layout: str = 'crossing'
    policy: str = 'cruise'
    steering_mode: str = 'legacy'
    cruise_m_s: float = .6
    cap_m_s: float = .35
    high_mu: float = .8
    low_mu: float = .04
    boundary_x_m: float = 1.2
    cap_start_x_m: float = .45
    trigger_x_m: float = 1.9
    payload_kg: float = 0.
    turn_radius_m: float = 1.
    turn_sign: int = 1
    duration_s: float = 24.
    max_stop_excursion_m: float = .25
    stop_deadline_s: float = 2.
    max_lateral_excursion_m: float = .15
    max_path_error_m: float = .30
    max_approach_lateral_m: float = .30
    max_goal_error_m: float = .25
    max_heading_error_rad: float = .15
    max_tilt_rad: float = .50
    settled_speed_m_s: float = .025
    timestep_ms: int = 16

    def __post_init__(self):
        if self.robot != 'husky' or type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError('This task is qualified for the reference Husky only, schema 1')
        if self.task not in ('brake', 'turn') or self.layout not in ('crossing', 'split', 'uniform-high', 'uniform-low') or self.policy not in ('cruise', 'surface-cap'):
            raise ValueError('Unknown mixed-surface task, layout or policy')
        if self.steering_mode not in ('legacy', 'traction-route'):
            raise ValueError('Unknown steering allocator')
        if self.steering_mode == 'traction-route' and self.policy != 'surface-cap':
            raise ValueError('The mapped repair requires the surface-cap policy')
        if type(self.timestep_ms) is not int or self.timestep_ms != 16 or type(self.turn_sign) is not int or self.turn_sign not in (-1, 1):
            raise ValueError('Requires 16 ms observations and turn_sign +/-1')
        for field in fields(self):
            if field.name in ('robot', 'schema_version', 'task', 'layout', 'policy', 'steering_mode', 'timestep_ms', 'turn_sign'):
                continue
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{field.name} must be finite and nonnegative')
        if not .15 <= self.cap_m_s <= self.cruise_m_s <= .8:
            raise ValueError('Requires .15 <= cap <= cruise <= .8 m/s')
        if not .02 <= self.low_mu <= self.high_mu <= 1.2 or not 0 <= self.payload_kg <= 20:
            raise ValueError('Fixture range: contact mu .02..1.2 and payload 0..20 kg')
        if not .2 <= self.cap_start_x_m <= self.boundary_x_m-.4 or not self.boundary_x_m+.5 <= self.trigger_x_m <= 3:
            raise ValueError('The speed-cap zone must precede the surface; task trigger must clear the rear axle')
        if not .7 <= self.turn_radius_m <= 1.5 or not 18 <= self.duration_s <= 40:
            raise ValueError('Radius/observation outside fixture range')
        for name in ('max_stop_excursion_m', 'stop_deadline_s', 'max_lateral_excursion_m', 'max_path_error_m', 'max_approach_lateral_m', 'max_goal_error_m', 'max_heading_error_rad', 'max_tilt_rad', 'settled_speed_m_s'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')


def inputs():
    profile = get_profile('husky')
    files = set((ROOT/'omnisim/assessment').glob('*.py'))
    files.update((ROOT/'projects/assessments/controllers/husky_mixed').glob('*.py'))
    files.add(ROOT/'projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py')
    files.add(ROOT/'projects/samples/demos/worlds/chat/omnilink_husky.omniworld')
    files.add(ROOT/'scripts/dev/qualify_husky_mixed.py')
    files.add(ROOT/'scripts/dev/qualify_husky_split_repair.py')
    files.update(p for p in profile.model.parent.parent.rglob('*') if p.is_file() and p.suffix != '.pyc')
    for name in ('projects/objects/backgrounds/protos/OmniSimSky.proto', 'projects/objects/lights/protos/OmniSimSun.proto', 'projects/objects/lights/protos/OmniSimSunMarker.proto'):
        files.add(ROOT/name)
    return {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(files)}


def make_world(spec, directory):
    model, _, mass, extra_mass, _ = make_model(BrakingSpec(schema_version=2, ground_mu=.01, payload_kg=spec.payload_kg), directory)
    # Every wheel is explicitly .01; both sides of each contact have equal
    # priority in the pinned runtime. MuJoCo takes max: the floor sets contact mu.
    # No runtime mutation of friction or the robot's pose is needed.
    patches = []
    def patch(name, x0, x1, y0, y1, mu):
        color = '.17 .21 .25' if mu == spec.high_mu else '.48 .67 .70'
        patches.append(dict(name=name, center=[(x0+x1)/2, (y0+y1)/2, -.1], half_size=[(x1-x0)/2,(y1-y0)/2,.1], mu=mu))
        return f'''DEF {name} Solid {{ translation {(x0+x1)/2} {(y0+y1)/2} -.1 newtonFriction {mu}
 children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness .85 metalness 0 }} geometry Box {{ size {x1-x0} {y1-y0} .2 }} }} ]
 boundingObject Box {{ size {x1-x0} {y1-y0} .2 }} }}'''
    bx = spec.boundary_x_m
    if spec.layout.startswith('uniform'):
        floor = patch('UNIFORM', -6, 12, -6, 8, spec.high_mu if spec.layout == 'uniform-high' else spec.low_mu)
    else:
        floor = patch('APPROACH', -6, bx, -6, 8, spec.high_mu)
        if spec.layout == 'crossing':
            floor += patch('LOW_PATCH', bx, 12, -6, 8, spec.low_mu)
        else:
            floor += patch('LOW_LEFT', bx, 12, 0, 8, spec.low_mu)
            floor += patch('HIGH_RIGHT', bx, 12, -6, 0, spec.high_mu)
    # Purely visual marks have no colliders. They do not add support or drag.
    marks = ''
    for x,y in [(spec.trigger_x_m, 0), (spec.trigger_x_m+spec.turn_radius_m, spec.turn_sign*spec.turn_radius_m)]:
        marks += f'''Transform {{ translation {x} {y} .003 children [ Shape {{ appearance PBRAppearance {{ baseColor .85 .7 .19 roughness 1 metalness 0 }} geometry Box {{ size .05 .18 .005 }} }} ] }}\n'''
    h=get_profile('husky').spawn_height_m+.03
    # Fixed, world-space camera, identical for every case; no tracking can
    # disguise a stopping excursion. OmniSim's camera frame is +X forward,
    # +Y left, +Z up, matching the harness's framing implementation.
    eye=[4.7,-4.2,4.8];target=[2.0,.3,0]
    def norm(v):
        length=math.sqrt(sum(x*x for x in v));return [x/length for x in v]
    forward=norm([b-a for a,b in zip(eye,target)])
    right=norm([forward[1],-forward[0],0])
    up=[right[1]*forward[2]-right[2]*forward[1],right[2]*forward[0]-right[0]*forward[2],right[0]*forward[1]-right[1]*forward[0]]
    m=[[forward[i],-right[i],up[i]] for i in range(3)]
    angle=math.acos(max(-1,min(1,(m[0][0]+m[1][1]+m[2][2]-1)/2)))
    axis=norm([m[2][1]-m[1][2],m[0][2]-m[2][0],m[1][0]-m[0][1]])
    view=f'position {" ".join(map(str,eye))} orientation {" ".join(map(str,axis))} {angle} fieldOfView .7'
    world=f'''#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
WorldInfo {{ basicTimeStep 16 title "Husky mixed-surface {spec.task}" newtonSolver "mujoco" newtonGroundMu .01 newtonSubsteps 8 }}
Viewpoint {{ {view} }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}
{floor}
{marks}
DEF ASSESSMENT_ROBOT URDFRobot {{ url "{model.as_posix()}" translation 0 0 {h}
 name "husky" supervisor TRUE controller "husky_mixed" }}
'''
    (directory/'world.omniworld').write_text(world, encoding='utf-8')
    return world, dict(patches=patches, expected_mass_kg=mass+extra_mass+spec.payload_kg, robot_geom_mu=.01,
                      material_scope='Rigid, imposed Coulomb contact coefficients; no water, mud, soil deformation or calibrated material identity')


def verify_fixture(directory, fixture):
    path=directory/'compiled-model.txt'
    if not path.exists(): return dict(valid=False, reason='compiled_model_missing')
    text=path.read_text()
    masses=[float(x) for x in re.findall(r'^body .*?mass=([\d.eE+-]+)', text, re.M)]
    floors=[]; dynamic=[]
    for line in text.splitlines():
        if not line.startswith('geom '): continue
        mu=float(re.search(r'fric=\[([^,]+)', line)[1])
        if 'body=0 ' in line:
            center=[float(x) for x in re.search(r'pos=\[([^\]]+)\]', line)[1].split(',')]
            size=[float(x) for x in re.search(r'size=\[([^\]]+)\]', line)[1].split(',')]
            floors.append(dict(center=center,half_size=size,mu=mu))
        else: dynamic.append(mu)
    adopted=len(floors)==len(fixture['patches']) and all(any(
        abs(g['mu']-p['mu'])<.0002 and all(abs(a-b)<.0002 for a,b in zip(g['center']+g['half_size'],p['center']+p['half_size']))
        for g in floors) for p in fixture['patches'])
    valid=adopted and bool(dynamic) and all(abs(mu-.01)<.0002 for mu in dynamic) and abs(sum(masses)-fixture['expected_mass_kg'])<.03
    return dict(valid=valid, floor_patches=floors, dynamic_geom_mu=dynamic, compiled_mass_kg=sum(masses),
                effective_contact_rule='max(floor mu, robot .01), both explicit priority 1 in pinned runtime', physical_geometry_matches=adopted)


def grade(spec, rows, completion, runtime, adoption):
    reasons=[]; errors=[]; metrics={}
    if runtime.get('backend')!='newton' or not runtime.get('finalised') or runtime.get('degraded'):
        errors.append('physics_runtime_not_finalised')
    if not adoption.get('valid'): errors.append('physical_fixture_not_verified')
    if not completion.get('complete'): errors.append('incomplete_controller_observation')
    required=('t_s','x_m','y_m','z_m','vx_m_s','vy_m_s','wz_rad_s','yaw_rad','tilt_rad','left_command_rad_s','right_command_rad_s')
    if not rows or any(not all(isinstance(r.get(k),(int,float)) and math.isfinite(r[k]) for k in required) for r in rows):
        errors.append('missing_or_nonfinite_trace')
    elif any(b['t_s']-a['t_s']<.015 or b['t_s']-a['t_s']>.017 for a,b in zip(rows,rows[1:])):
        errors.append('noncontiguous_observation')
    if errors:return dict(status='ERROR',reasons=errors,metrics=metrics)
    if rows[-1]['t_s'] < spec.duration_s-.017: return dict(status='ERROR',reasons=['short_observation'],metrics={})
    if any(abs(r['x_m'])>11 or not -5<r['y_m']<7 or r['z_m']<-.2 for r in rows):
        return dict(status='ERROR',reasons=['left_physical_fixture'],metrics={})
    trigger=next((i for i,r in enumerate(rows) if r['event_started']),None)
    motion=[r for r in rows if r['t_s']>=1]
    metrics.update(max_tilt_rad=max(r['tilt_rad'] for r in motion),final_height_m=rows[-1]['z_m'],
                   max_speed_m_s=max(math.hypot(r['vx_m_s'],r['vy_m_s']) for r in motion),
                   surface_entered=any(r['x_m']>spec.boundary_x_m+.4 for r in rows),
                   observation_s=rows[-1]['t_s'],pose_writes=False,external_forces=False)
    if metrics['max_tilt_rad']>spec.max_tilt_rad or min(r['z_m'] for r in motion)<.12: reasons.append('unstable_or_tipped')
    if trigger is None: reasons.append('task_trigger_not_reached')
    else:
        start=rows[trigger]; tail=rows[trigger:]
        approach_lateral=max(abs(r['y_m']) for r in rows[:trigger+1])
        metrics['max_approach_lateral_m']=approach_lateral
        if approach_lateral>spec.max_approach_lateral_m:reasons.append('approach_corridor_exceeded')
        metrics.update(trigger_t_s=start['t_s'],trigger_x_m=start['x_m'],trigger_speed_m_s=math.hypot(start['vx_m_s'],start['vy_m_s']))
        target=spec.cruise_m_s if spec.policy=='cruise' else spec.cap_m_s
        approach=[r for r in rows if start['t_s']-.3<=r['t_s']<=start['t_s']]
        if len(approach)<15 or not all(.80*target<=math.hypot(r['vx_m_s'],r['vy_m_s'])<=1.20*target for r in approach):
            reasons.append('approach_speed_not_qualified')
        if spec.task=='brake':
            excursion=max(r['x_m'] for r in tail)-start['x_m']
            lateral=max(abs(r['y_m']-start['y_m']) for r in tail)
            settle=None
            for i,r in enumerate(tail):
                if tail[-1]['t_s']-r['t_s']<.5:break
                if all(math.hypot(v['vx_m_s'],v['vy_m_s'])<=spec.settled_speed_m_s and abs(v['wz_rad_s'])<=.05 for v in tail[i:]):
                    settle=r['t_s']-start['t_s'];break
            metrics.update(max_forward_excursion_m=excursion,max_lateral_excursion_m=lateral,settled_after_s=settle,
                           residual_speed_m_s=math.hypot(tail[-1]['vx_m_s'],tail[-1]['vy_m_s']),
                           approach_speed_reduction_m_s=spec.cruise_m_s-target)
            if excursion>spec.max_stop_excursion_m:reasons.append('stopping_excursion_exceeded')
            if lateral>spec.max_lateral_excursion_m:reasons.append('lateral_excursion_exceeded')
            if settle is None or settle>spec.stop_deadline_s:reasons.append('stopping_deadline_exceeded')
            if max(start['x_m']-r['x_m'] for r in tail)>.05:reasons.append('rollback_exceeded')
            if not all(r['left_command_rad_s']==0 and r['right_command_rad_s']==0 for r in tail): errors.append('non_immediate_braking')
        else:
            radius=spec.turn_radius_m; sign=spec.turn_sign
            goal=(spec.trigger_x_m+radius,sign*radius)
            final=rows[-1]
            goal_error=math.hypot(final['x_m']-goal[0],final['y_m']-goal[1])
            yaw_error=abs(math.atan2(math.sin(final['yaw_rad']-sign*math.pi/2),math.cos(final['yaw_rad']-sign*math.pi/2)))
            # Recompute geometric errors from measured poses. The controller's
            # own readout never decides the offline verdict.
            def arc_error(r):
                phi=max(0,min(math.pi/2,math.atan2(r['x_m']-spec.trigger_x_m,radius-sign*r['y_m'])))
                return math.hypot(r['x_m']-spec.trigger_x_m-radius*math.sin(phi),r['y_m']-sign*radius*(1-math.cos(phi)))
            path_error=max(arc_error(r) for r in tail)
            final_window=[r for r in tail if r['t_s']>=rows[-1]['t_s']-1]
            metrics.update(max_path_error_m=path_error,final_goal_error_m=goal_error,final_heading_error_rad=yaw_error,
                           turn_finished_t_s=completion.get('turn_finished_t_s'),
                           final_speed_m_s=math.hypot(final['vx_m_s'],final['vy_m_s']),
                           approach_speed_reduction_m_s=spec.cruise_m_s-target)
            if not completion.get('turn_finished_t_s'): reasons.append('turn_not_completed')
            if path_error>spec.max_path_error_m:reasons.append('turn_corridor_exceeded')
            if goal_error>spec.max_goal_error_m:reasons.append('turn_goal_error_exceeded')
            if yaw_error>spec.max_heading_error_rad:reasons.append('turn_heading_error_exceeded')
            if not all(math.hypot(r['vx_m_s'],r['vy_m_s'])<=spec.settled_speed_m_s for r in final_window):reasons.append('turn_not_settled')
    return dict(status='ERROR' if errors else 'FAIL' if reasons else 'PASS',reasons=errors+reasons,metrics=metrics)


def regrade(directory):
    directory=Path(directory)
    spec=MixedSpec(**read_json(directory/'spec.json'))
    manifest=read_json(directory/'manifest.json',{})
    try:rows=[json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
    except (OSError,ValueError):rows=[]
    result=grade(spec,rows,read_json(directory/'completion.json',{}),read_json(directory/'engine.log.newton.json',{}),read_json(directory/'adoption.json',{}))
    for name,expected in manifest.get('artifacts_sha256',{}).items():
        path=directory/name
        if Path(name).name!=name or not path.is_file() or digest(path)!=expected:
            result['status']='ERROR';result['reasons'].append('artifact_hash_mismatch:'+name)
    if manifest.get('engine_exit_code')!=0 or manifest.get('launch_error') or manifest.get('cleanup_errors'):
        result['status']='ERROR';result['reasons'].append('engine_launch_exit_or_cleanup_error')
    return result


def run(spec, directory, capture=False, wall_timeout_s=150):
    directory=Path(directory).resolve()
    binary=Path(os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary() or '').resolve()
    if not binary.is_file():raise ValueError('Run python -m omnisim doctor first')
    source=inputs()
    directory.mkdir(parents=True,exist_ok=False)
    save(directory/'spec.json',asdict(spec))
    contents,fixture=make_world(spec,directory)
    stage=ROOT/f'projects/assessments/worlds/.mixed_{uuid.uuid4().hex}.omniworld'
    stage.write_text(contents,encoding='utf-8')
    env=linux_runtime_env(ROOT)
    for key in list(env):
        if key.startswith(('OMNISIM_NEWTON_','OMNISIM_INENGINE_','OMNISIM_ASSESSMENT_')) or key=='OMNISIM_NO_WINDOW':env.pop(key)
    env.update(OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(directory/'engine.log'),
               OMNISIM_NEWTON_DUMP_MJMODEL=str(directory/'compiled-model.txt'),
               OMNISIM_ASSESSMENT_OUT=str(directory),OMNISIM_ASSESSMENT_CAPTURE='1' if capture else '0')
    runtime=binary.parent/'newton-runtime'
    env['PATH']=os.pathsep.join([str(runtime),str(binary.parent),env.get('PATH','')])
    manifest=dict(schema_version=1,task='husky_mixed_'+spec.task,sources_sha256=source,binary_sha256=digest(binary),runtime_sha256=runtime_hashes(binary),
                  fixture=fixture,capture=capture,runtime_settings=dict(solver='mujoco',newtonSubsteps=8,timestep_ms=16),
                  motion='wheel motor velocity only; no root pose/velocity writes, no external force, no braking delay')
    save(directory/'manifest.json',manifest)
    args=[str(binary),'--batch','--mode=fast','--minimize','--stdout','--stderr']
    if not capture:args.append('--no-rendering')
    args.append(str(stage))
    children={};proc=None;error=None;started=time.monotonic()
    try:
        import psutil
        with (directory/'stdout.log').open('w',encoding='utf-8') as stdout:
            proc=subprocess.Popen(args,cwd=ROOT,env=env,stdout=stdout,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            manifest['owned_pid']=proc.pid
            while proc.poll() is None:
                try:
                    for child in psutil.Process(proc.pid).children(recursive=True):
                        child.create_time();children[child.pid]=child
                except psutil.NoSuchProcess:pass
                if time.monotonic()-started>(600 if capture else wall_timeout_s) or (directory/'controller-error.txt').exists():
                    error='watchdog_or_controller_error';break
                time.sleep(.1)
            manifest['engine_exit_code']=proc.poll()
    except (OSError,subprocess.SubprocessError) as exc:error=str(exc)
    finally:
        manifest['cleanup_errors']=_reap_owned(proc,children) if proc else []
        stage.unlink(missing_ok=True)
    from projects.policies.common.env_fingerprint import collect
    save(directory/'fingerprint.json',collect(engine_log_path=directory/'engine.log',repo_root=ROOT,warm=False))
    save(directory/'adoption.json',verify_fixture(directory,fixture))
    manifest.update(wall_elapsed_s=time.monotonic()-started,launch_error=error,
                    artifacts_sha256={p.name:digest(p) for p in directory.iterdir() if p.is_file() and p.name not in ('manifest.json','verdict.json')})
    save(directory/'manifest.json',manifest)
    result=regrade(directory)
    save(directory/'verdict.json',result)
    return result


def rerun(case,directory,capture=False):
    case=Path(case)
    manifest=read_json(case/'manifest.json')
    binary=Path(os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary() or '').resolve()
    if manifest['sources_sha256']!=inputs() or manifest['binary_sha256']!=digest(binary) or manifest['runtime_sha256']!=runtime_hashes(binary):
        raise ValueError('Recorded source/engine/runtime differs from this checkout; restore frozen inputs')
    return run(MixedSpec(**read_json(case/'spec.json')),directory,capture)
