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
# limitations under the License."""Apply an equal/opposite spring force at every physics step, never a pose write."""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
from omnisim import Supervisor

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from compartment import PARAMETERS, spring_force, on_positive_contact, on_floor_contact
from evaluate import cylinder_in_slot


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config');args=ap.parse_args()
    cfg=json.loads(Path(args.config).read_text()) if args.config else {}
    parameters={**PARAMETERS,**cfg.get('spring_parameters',{})}
    robot=Supervisor();dt=int(robot.getBasicTimeStep())
    remote=robot.getFromDef('REMOTE');head=robot.getFromDef('NEGATIVE_TERMINAL')
    joint=robot.getFromDef('SPRING_JOINT').getField('position')
    coil=robot.getFromDef('SPRING_COIL').getField('scale')
    dynamic_remote=bool(remote.getField('physics').getSFNode())
    out=Path(cfg['out_dir']) if cfg.get('out_dir') else None
    probe=cfg.get('spring_probe',False)
    contact_probe=cfg.get('contact_probe')
    audit=bool(cfg.get('spring_compartment') and out)
    battery=robot.getFromDef('BATTERY') if contact_probe or audit else None
    positive=robot.getFromDef('POSITIVE_TERMINAL') if contact_probe else None
    trace=None
    if out:
        out.mkdir(parents=True,exist_ok=True)
        trace=(out/'spring.jsonl').open('w')
    rows=[];previous_q=joint.getSFFloat();tick=0
    audit_result={'complete':False,'sampling_hz':1000/dt,'samples':0,
                  'negative_contact_peak_depth_m':0.0,'housing_contact_peak_depth_m':0.0,
                  'spring_compression_peak_m':0.0}
    # Finalize the world before submitting external forces.
    robot.step(dt)
    try:
        while True:
            q=joint.getSFFloat();velocity=(q-previous_q)/(dt/1000);previous_q=q
            force=spring_force(q,velocity,parameters)
            R=remote.getOrientation();direction=[R[0],R[3],R[6]]
            # Reaction applied at the same physical point preserves force and torque.
            p=head.getPosition();base=remote.getPosition();d=[a-b for a,b in zip(p,base)]
            offset=[sum(R[j*3+i]*d[j] for j in range(3)) for i in range(3)]
            head.addForce([force*x for x in direction],False)
            if dynamic_remote:remote.addForceWithOffset([-force*x for x in direction],offset,False)
            t=robot.getTime()
            if audit and t>=1.8:
                points=lambda n:{tuple(round(x,5) for x in c.point) for c in n.getContactPoints()}
                bp=battery.getContactPoints()
                for label,node in [('negative',head),('housing',remote)]:
                    shared=points(node)
                    depth=max((c.depth for c in bp if tuple(round(x,5) for x in c.point) in shared),default=0)
                    key=label+'_contact_peak_depth_m'
                    if depth>audit_result[key]:
                        audit_result[key]=depth;audit_result[label+'_peak_time_s']=t-1.8
                audit_result['samples']+=1
                audit_result['spring_compression_peak_m']=max(audit_result['spring_compression_peak_m'],-q)
            load=.75 if probe and .4<=t<1.4 else 0.0
            if load:head.addForce([-load*x for x in direction],False)
            if contact_probe in ('seated','drop') and 1.4<=t<1.8:battery.addForce([0,0,.6],False)
            coil_length=parameters['coil_rest_length_m']
            if tick%10==0:coil.setSFVec3f([max(.001,coil_length+q)/coil_length,1,1])
            row={'time_s':t,'position_m':q,'compression_m':-q,'velocity_m_s':velocity,
                 'spring_force_n':force,'probe_load_n':load}
            if contact_probe:
                points=lambda n:{tuple(round(x,5) for x in c.point) for c in n.getContactPoints()}
                bp=points(battery)
                rel=mv_local(remote,battery.getPosition())
                row.update(battery_in_remote=rel,
                           battery_rotation=battery.getOrientation(),remote_rotation=remote.getOrientation(),
                           battery_position=battery.getPosition(),
                           negative_points=[mv_local(remote,p) for p in bp&points(head)],
                           positive_points=[local for p in bp&points(remote) if on_positive_contact(local:=mv_local(remote,p))],
                           floor_points=[local for p in bp&points(remote) if on_floor_contact(local:=mv_local(remote,p))],
                           contact_points=[{'id':c.node_id,'point':mv_local(remote,c.point),'depth':c.depth} for c in battery.getContactPoints()])
            if trace and (probe or contact_probe or tick%10==0):
                trace.write(json.dumps(row)+'\n');trace.flush()
            if probe or contact_probe:rows.append(row)
            if contact_probe and tick in [round(s*1000/dt) for s in (.2,1.2,2.2)]:robot.exportImage(str(out/f'probe-{tick}.png'),100)
            if contact_probe and t>=2.4:
                tail=rows[-round(200/dt):]
                retained=all(r['compression_m']>=.003 and r['negative_points'] and r['positive_points']
                             and cylinder_in_slot(r['battery_in_remote'],r['battery_rotation'],r['remote_rotation'])
                             and max((c['depth'] for c in r['contact_points']),default=0)<=.0015 for r in tail)
                if contact_probe=='table':retained=all(abs(r['battery_position'][2]-.0168)<.0005 for r in tail)
                result={'complete':True,'case':contact_probe,'parameters':parameters,'passed':bool(retained),
                        'floor_contact_observed':any(r['floor_points'] for r in tail),
                        'final':row,'upward_test_force_n':.6 if contact_probe!='table' else 0,
                        'test_interval_s':[1.4,1.8] if contact_probe!='table' else None,
                        'runtime_environment':{key:os.environ.get(key) for key in ('OMNISIM_NEWTON_FINGER_KE','OMNISIM_NEWTON_MULTICCD')},
                        'body_pose_writes':0,'scope':'Isolated mechanism/geometry test, not a robot insertion'}
                (out/'spring-result.json').write_text(json.dumps(result,indent=2))
                contact_probe=None
                if trace:trace.close();trace=None
            if probe and t>=2.4:
                loaded=[r['compression_m'] for r in rows if 1.2<r['time_s']<1.4]
                equilibrium=sum(loaded)/len(loaded)
                expected=(.75-parameters['preload_n'])/parameters['stiffness_n_m'] if parameters['stiffness_n_m'] else None
                result={'complete':True,'parameters':parameters,'equilibrium_compression_m':equilibrium,
                        'expected_compression_m':expected,'released_compression_m':-q,
                        'passed':expected is not None and abs(equilibrium-expected)<.0003 and abs(q)<.0003,
                        'body_pose_writes':0,'spring_update_hz':1000/dt}
                (out/'spring-result.json').write_text(json.dumps(result,indent=2))
                if trace:trace.close();trace=None
                probe=False
            if trace and not probe and not contact_probe and t>=1.8+len(cfg.get('targets',[]))/cfg.get('fps',50)+1.0:
                trace.close();trace=None
                if audit:
                    audit_result['complete']=True
                    audit_result['contact_depth_passed']=max(audit_result['negative_contact_peak_depth_m'],audit_result['housing_contact_peak_depth_m'])<=.0015
                    (out/'mechanism-audit.json').write_text(json.dumps(audit_result,indent=2))
                    audit=False
            tick+=1
            if robot.step(dt)==-1:break
    except Exception:
        if out:(out/'spring-error.txt').write_text(traceback.format_exc())
        raise
    finally:
        if trace:trace.close()


def mv_local(node,point):
    R=node.getOrientation();d=[x-y for x,y in zip(point,node.getPosition())]
    return [sum(R[j*3+i]*d[j] for j in range(3)) for i in range(3)]


if __name__=='__main__':main()
