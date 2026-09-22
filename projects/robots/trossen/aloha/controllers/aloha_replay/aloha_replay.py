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
# limitations under the License.# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
"""Recorded ALOHA motor targets; all task objects evolve through physics."""
import argparse, json, math, os, sys, tempfile, traceback
from pathlib import Path
from omnisim import Supervisor

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from evaluate import evaluate
JOINTS=['waist','shoulder','elbow','forearm_roll','wrist_angle','wrist_rotate','left_finger','right_finger']

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--config'); args=ap.parse_args()
    robot=Supervisor(); dt=int(robot.getBasicTimeStep())
    default='spring_motion.json' if robot.getFromDef('SPRING_JOINT') else 'episode.json'
    cfg=json.loads(Path(args.config or ROOT/default).read_text())
    out=Path(cfg.get('out_dir') or tempfile.mkdtemp(prefix='omnisim-aloha-')); out.mkdir(parents=True,exist_ok=True)
    print('[ALOHA] Evidence '+str(out),flush=True)
    side='right' if robot.getName()=='aloha_right' else 'left'
    motors=[robot.getDevice(side+'_'+joint+'_motor') for joint in JOINTS]
    sensors=[m.getPositionSensor() for m in motors]
    for s in sensors:s.enable(dt)
    nodes={}
    def walk(n):
        if n.getTypeName() in ('Solid','Robot'):
            f=n.getField('name')
            if f:nodes[f.getSFString()]=n
        f=n.getField('children')
        if f:
            for i in range(f.getCount()):walk(f.getMFNode(i))
        f=n.getField('endPoint')
        if f and f.getSFNode():walk(f.getSFNode())
    joint_nodes={}
    def collect_joints(n):
        if n.getTypeName() in ('HingeJoint','SliderJoint'):
            p=n.getField('jointParameters').getSFNode()
            e=n.getField('endPoint').getSFNode()
            joint_nodes[e.getField('name').getSFString()]=p
        f=n.getField('children')
        if f:
            for i in range(f.getCount()):collect_joints(f.getMFNode(i))
        f=n.getField('endPoint')
        if f and f.getSFNode():collect_joints(f.getSFNode())
    for label in ['ALOHA_LEFT','ALOHA_RIGHT']:
        walk(robot.getFromDef(label));collect_joints(robot.getFromDef(label))
    battery=robot.getFromDef('BATTERY'); remote=robot.getFromDef('REMOTE')
    nodes.update(battery=battery,remote=remote,table=robot.getFromDef('TABLE'))
    terminals={}
    spring_joint=None
    if cfg.get('spring_compartment'):
        terminals={name:robot.getFromDef(label) for name,label in [('negative_terminal','NEGATIVE_TERMINAL'),('positive_terminal','POSITIVE_TERMINAL')]}
        nodes.update(terminals)
        spring_joint=robot.getFromDef('SPRING_JOINT').getField('position')
    hands={name:n for name,n in nodes.items() if name.endswith('_finger_link')}
    grip=nodes['right_gripper_link']
    def step(count=1):
        if robot.step(dt*count)==-1:raise RuntimeError('Engine stopped before completion')
    def command(row):
        for m,v in zip(motors,row[8:] if side=='right' else row[:8]):m.setPosition(v)
    if side=='right':
        control=None
        if cfg.get('spring_compartment') and cfg.get('feedback_control',True) and not cfg.get('no_grip'):
            from insertion_control import InsertionControl
            control=InsertionControl(cfg,battery,remote,grip,spring_joint)
        command(cfg['targets'][0])
        step(round(1800/dt))
        start=robot.getTime()
        executed=[]
        for k,row in enumerate(cfg['targets']):
            row=list(row)
            if cfg.get('no_grip'):row[14:16]=[.06,-.06]
            if control:
                row[8:]=control.command(k,row[8:])
            if cfg.get('spring_compartment'):
                robot.getSelf().getField('customData').setSFString(json.dumps({'frame':k,'joint_targets':row[8:],
                    'feedback':'measured grip transform and remote pose' if control else 'fixed motor targets'}))
            executed.append(row)
            command(row)
            step(round(1000/cfg['fps']/dt))
        (out/'executed_targets.json').write_text(json.dumps(executed))
        while robot.step(dt)!=-1:pass
        return
    def contacts(node):return {tuple(round(x,5) for x in c.point) for c in node.getContactPoints()}
    def local(node,other):
        R=node.getOrientation(); d=[a-b for a,b in zip(other.getPosition(),node.getPosition())]
        return [sum(R[j*3+i]*d[j] for j in range(3)) for i in range(3)]
    try:
        targets=cfg['targets']; initial={n:(list(nodes[n].getField('translation').getSFVec3f()),list(nodes[n].getField('rotation').getSFRotation())) for n in ['battery','remote']}
        for i,n in enumerate(initial):nodes[n].getField('translation').setSFVec3f([2+i,2,.2]);nodes[n].resetPhysics()
        command(targets[0])
        step(round(1600/dt))
        for n,(p,r) in initial.items():
            nodes[n].getField('translation').setSFVec3f(cfg.get(n+'_position',p))
            nodes[n].getField('rotation').setSFRotation(cfg.get(n+'_rotation',r));nodes[n].resetPhysics()
        step(round(200/dt))
        initial_z=battery.getPosition()[2]
        start=robot.getTime(); max_speed=0; max_z=0; rows=[]; previous=battery.getPosition()
        (out/'frames').mkdir(exist_ok=True)
        with (out/'trace.jsonl').open('w') as trace, (out/'poses.jsonl').open('w') as poses:
            for k,target in enumerate(targets):
                target=list(target)
                if cfg.get('no_grip'):target[14:16]=[.06,-.06]
                command(target)
                step(round(1000/cfg['fps']/dt))
                p=battery.getPosition();max_speed=max(max_speed,math.dist(battery.getVelocity()[:3],[0,0,0]));max_z=max(max_z,p[2])
                bp=contacts(battery)
                touched=[n for n,node in {**hands,**terminals,'remote':remote,'table':nodes['table']}.items() if bp.intersection(contacts(node))]
                if spring_joint:
                    from compartment import on_positive_contact, on_floor_contact
                    R=remote.getOrientation();origin=remote.getPosition()
                    def point_local(point):
                        d=[x-y for x,y in zip(point,origin)]
                        return [sum(R[j*3+i]*d[j] for j in range(3)) for i in range(3)]
                    positive_points=[point_local(p) for p in bp.intersection(contacts(remote)) if on_positive_contact(point_local(p))]
                    floor_points=[point_local(p) for p in bp.intersection(contacts(remote)) if on_floor_contact(point_local(p))]
                    if positive_points:touched.append('positive_terminal')
                rp=contacts(remote)
                remote_contacts=[n for n,node in hands.items() if rp.intersection(contacts(node))]
                order=['shoulder_link','upper_arm_link','upper_forearm_link','lower_forearm_link','wrist_link','gripper_link','left_finger_link','right_finger_link']
                achieved=[joint_nodes[s+'_'+n].getField('position').getSFFloat() for s in ['left','right'] for n in order]
                row={'frame':k,'time_s':robot.getTime()-start,'command':target,'achieved':achieved,
                     'reference':cfg['reference'][k], 'battery_position':battery.getPosition(),'battery_rotation':battery.getOrientation(),
                     'battery_velocity':battery.getVelocity(),'remote_position':remote.getPosition(),'remote_rotation':remote.getOrientation(),
                     'battery_in_remote':local(remote,battery),'battery_in_gripper':local(grip,battery),'contacts':touched,
                     'remote_contacts':remote_contacts}
                if spring_joint:
                    row.update(spring_compression_m=-spring_joint.getSFFloat(),stage=cfg['stages'][k],
                               negative_contact_max_depth_m=max((c.depth for c in battery.getContactPoints()
                                  if tuple(round(x,5) for x in c.point) in contacts(terminals['negative_terminal'])),default=0),
                               housing_contact_max_depth_m=max((c.depth for c in battery.getContactPoints()
                                  if tuple(round(x,5) for x in c.point) in rp),default=0),
                               positive_contact_points=positive_points,
                               floor_contact_points=floor_points,
                               negative_terminal_in_remote=local(remote,terminals['negative_terminal']),
                               positive_terminal_in_remote=local(remote,terminals['positive_terminal']))
                    row['right_control']=json.loads(robot.getFromDef('ALOHA_RIGHT').getField('customData').getSFString() or '{}')
                rows.append(row);trace.write(json.dumps(row)+'\n');trace.flush()
                poses.write(json.dumps({'time':row['time_s'],'poses':{n:node.getPose() for n,node in nodes.items() if n!='table'}})+'\n')
                if cfg.get('capture') or k in cfg.get('screenshot_frames',[]):robot.exportImage(str(out/'frames'/f'sim_{k:05d}.jpg'),95)
                if k%100==0:print('[ALOHA] '+str(k)+' battery='+str(row['battery_position'])+' contacts='+str(touched),flush=True)
        step(round(1000/dt))
        if spring_joint and cfg.get('out_dir'):
            for _ in range(50):
                if (out/'mechanism-audit.json').exists():break
                step()
            if not (out/'mechanism-audit.json').exists():raise RuntimeError('Missing physics-step contact audit')
        result={'complete':True,'frames':len(rows),'max_battery_height_m':max_z,'max_battery_speed_m_s':max_speed,
                'final_battery_position':battery.getPosition(),'final_battery_in_remote':local(remote,battery),
                'final_battery_rotation':battery.getOrientation(),'final_remote_rotation':remote.getOrientation(),
                'final_battery_velocity':battery.getVelocity(),'actuation':'Motor.setPosition','timed_episode_pose_writes':0,
                'motion_mode':cfg.get('motion_mode','recorded action')}
        result['metric_sampling_hz']=cfg['fps']
        result['runtime_environment']={key:os.environ.get(key) for key in ('OMNISIM_NEWTON_FINGER_KE','OMNISIM_NEWTON_MULTICCD')}
        result.update(evaluate(rows,result,cfg['fps'],initial_z))
        if spring_joint:
            from evaluate import evaluate_insertion
            result.update(evaluate_insertion(rows,result,cfg['fps']))
            result['physics_step_audit']=(json.loads((out/'mechanism-audit.json').read_text()) if cfg.get('out_dir')
                                          else {'complete':False,'contact_depth_passed':False,'reason':'Run the replay CLI for coordinated evidence'})
            result['task_success']=bool(result['task_success'] and result['physics_step_audit']['complete']
                                        and result['physics_step_audit']['contact_depth_passed'])
        (out/'result.json').write_text(json.dumps(result,indent=2));print('[ALOHA] complete '+str(out),flush=True)
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc());raise
    while robot.step(dt)!=-1:pass

if __name__=='__main__':
    try:
        main()
    except Exception:
        if '--config' in sys.argv:
            error_cfg=json.loads(Path(sys.argv[sys.argv.index('--config')+1]).read_text())
            if error_cfg.get('out_dir'):
                Path(error_cfg['out_dir'],'error.txt').write_text(traceback.format_exc())
        raise
