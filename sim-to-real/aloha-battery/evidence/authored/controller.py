# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at https://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
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
    cfg=json.loads(Path(args.config or ROOT/'episode.json').read_text())
    out=Path(cfg.get('out_dir') or tempfile.mkdtemp(prefix='omnisim-aloha-')); out.mkdir(parents=True,exist_ok=True)
    print('[ALOHA] Evidence '+str(out),flush=True)
    robot=Supervisor(); dt=int(robot.getBasicTimeStep())
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
    hands={name:n for name,n in nodes.items() if name.endswith('_finger_link')}
    grip=nodes['right_gripper_link']
    def step(count=1):
        if robot.step(dt*count)==-1:raise RuntimeError('Engine stopped before completion')
    def command(row):
        for m,v in zip(motors,row[8:] if side=='right' else row[:8]):m.setPosition(v)
    if side=='right':
        command(cfg['targets'][0])
        step(900)
        start=robot.getTime()
        for k,row in enumerate(cfg['targets']):
            row=list(row)
            if cfg.get('no_grip'):row[14:16]=[.06,-.06]
            command(row)
            step(round(1000/cfg['fps']/dt))
        while robot.step(dt)!=-1:pass
        return
    def contacts(node):return {tuple(round(x,4) for x in c.point) for c in node.getContactPoints()}
    def local(node,other):
        R=node.getOrientation(); d=[a-b for a,b in zip(other.getPosition(),node.getPosition())]
        return [sum(R[j*3+i]*d[j] for j in range(3)) for i in range(3)]
    try:
        targets=cfg['targets']; initial={n:(list(nodes[n].getField('translation').getSFVec3f()),list(nodes[n].getField('rotation').getSFRotation())) for n in ['battery','remote']}
        for i,n in enumerate(initial):nodes[n].getField('translation').setSFVec3f([2+i,2,.2]);nodes[n].resetPhysics()
        command(targets[0])
        step(800)
        for n,(p,r) in initial.items():
            nodes[n].getField('translation').setSFVec3f(cfg.get(n+'_position',p))
            nodes[n].getField('rotation').setSFRotation(cfg.get(n+'_rotation',r));nodes[n].resetPhysics()
        step(100)
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
                touched=[n for n,node in {**hands,'remote':remote,'table':nodes['table']}.items() if bp.intersection(contacts(node))]
                rp=contacts(remote)
                remote_contacts=[n for n,node in hands.items() if rp.intersection(contacts(node))]
                order=['shoulder_link','upper_arm_link','upper_forearm_link','lower_forearm_link','wrist_link','gripper_link','left_finger_link','right_finger_link']
                achieved=[joint_nodes[s+'_'+n].getField('position').getSFFloat() for s in ['left','right'] for n in order]
                row={'frame':k,'time_s':robot.getTime()-start,'command':target,'achieved':achieved,
                     'reference':cfg['reference'][k], 'battery_position':battery.getPosition(),'battery_rotation':battery.getOrientation(),
                     'battery_velocity':battery.getVelocity(),'remote_position':remote.getPosition(),'remote_rotation':remote.getOrientation(),
                     'battery_in_remote':local(remote,battery),'battery_in_gripper':local(grip,battery),'contacts':touched,
                     'remote_contacts':remote_contacts}
                rows.append(row);trace.write(json.dumps(row)+'\n');trace.flush()
                poses.write(json.dumps({'time':row['time_s'],'poses':{n:node.getPose() for n,node in nodes.items() if n!='table'}})+'\n')
                if cfg.get('capture') or k in cfg.get('screenshot_frames',[]):robot.exportImage(str(out/'frames'/f'sim_{k:05d}.jpg'),95)
                if k%100==0:print('[ALOHA] '+str(k)+' battery='+str(row['battery_position'])+' contacts='+str(touched),flush=True)
        step(500)
        result={'complete':True,'frames':len(rows),'max_battery_height_m':max_z,'max_battery_speed_m_s':max_speed,
                'final_battery_position':battery.getPosition(),'final_battery_in_remote':local(remote,battery),
                'final_battery_rotation':battery.getOrientation(),'final_remote_rotation':remote.getOrientation(),
                'final_battery_velocity':battery.getVelocity(),'actuation':'Motor.setPosition','timed_episode_pose_writes':0,
                'motion_mode':cfg.get('motion_mode','recorded action')}
        result['metric_sampling_hz']=cfg['fps']
        result['runtime_environment']={'OMNISIM_NEWTON_FINGER_KE':os.environ.get('OMNISIM_NEWTON_FINGER_KE')}
        result.update(evaluate(rows,result,cfg['fps'],initial_z))
        (out/'result.json').write_text(json.dumps(result,indent=2));print('[ALOHA] complete '+str(out),flush=True)
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc());raise
    while robot.step(dt)!=-1:pass

if __name__=='__main__':main()
