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

"""Frozen Husky mixed-surface qualification, with prospectively declared inputs.

Run with: python scripts/dev/qualify_husky_mixed.py --out <new-directory>
Development is separate. This script does not tune a controller or requirements.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from omnisim.assessment.mixed import MixedSpec, inputs, regrade, run
from omnisim.assessment.runner import digest, read_json, save


def definitions():
    base=MixedSpec(cruise_m_s=.6)
    plan=[]
    def add(spec,kind):
        plan.append(dict(case=f'case-{len(plan):03d}',kind=kind,spec=asdict(spec)))
    for task in ('brake','turn'):
        for layout in ('crossing','split'):
            for payload in (0,10):
                for mu in (.04,.1,.2):
                    for policy in ('cruise','surface-cap'):
                        add(replace(base,task=task,layout=layout,payload_kg=payload,low_mu=mu,policy=policy),'grid')
    for task in ('brake','turn'):
        for layout in ('uniform-high','uniform-low'):
            for policy in ('cruise','surface-cap'):
                add(replace(base,task=task,layout=layout,policy=policy),'control')
    selected={task:[next(c['case'] for c in plan if c['kind']=='grid' and c['spec']['task']==task and c['spec']['layout']=='crossing' and c['spec']['payload_kg']==0 and c['spec']['low_mu']==.04 and c['spec']['policy']==policy) for policy in ('cruise','surface-cap')] for task in ('brake','turn')}
    for task,pair in selected.items():
        for case in pair:
            original=next(c for c in plan if c['case']==case)
            for _ in range(2):add(MixedSpec(**original['spec']),'confirmation')
    # Neither these contact coefficients nor these load/boundary/radius
    # combinations have been evaluated during controller development.
    prospective=[dict(layout='crossing',low_mu=.06,payload_kg=5,boundary_x_m=1.3,trigger_x_m=2.0,turn_radius_m=1.1),
                 dict(layout='split',low_mu=.12,payload_kg=5,boundary_x_m=1.1,trigger_x_m=1.8,turn_radius_m=.9),
                 dict(layout='crossing',low_mu=.15,payload_kg=15,boundary_x_m=1.3,trigger_x_m=2.0,turn_radius_m=1.2,turn_sign=-1)]
    for conditions in prospective:
        for task in ('brake','turn'):
            for policy in ('cruise','surface-cap'):
                add(replace(base,task=task,policy=policy,**conditions),'holdout')
    return plan,selected,prospective


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--workers',type=int,choices=[1,2],default=2)
    args=parser.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    plan,pairs,prospective=definitions();source=inputs()
    plan_record=dict(created_utc=datetime.now(timezone.utc).isoformat(),robot='husky',plan=plan,selected_pairs=pairs,prospective_inputs=prospective,
                     selection='Both comparisons prespecified from development. No selection or tuning using qualification or prospective results.',
                     validation_boundary='Only listed physical rigid-surface fixtures and reference driver. Surface-cap is a lower speed with a throughput cost, not increased friction or a general controller repair.')
    save(out/'plan.json',plan_record)
    for name,expected in source.items():
        target=out/'source'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,target)
        assert digest(target)==expected
    state=dict(schema_version=1,robot='husky',task='mixed_surface_mobility',complete=False,source_sha256=source,plan_sha256=digest(out/'plan.json'),
               selected_pairs=pairs,prospective_inputs=prospective,validation_boundary=plan_record['validation_boundary'],cases=[],rejected_attempts=[])
    save(out/'campaign.json',state)
    def execute(case):
        if inputs()!=source:raise ValueError('Source changed after freeze; retain evidence and restart a new qualification')
        target=out/case['case'];result=run(MixedSpec(**case['spec']),target)
        return {**case,**result}
    # Keep prospective data unopened until the entire grid, controls and
    # confirmations are complete. Every attempt remains in its own directory.
    for stage in ('grid','control','confirmation','holdout'):
        batch=[c for c in plan if c['kind']==stage]
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            for result in pool.map(execute,batch):
                state['cases'].append(result)
                save(out/'campaign.json',state)
                print(json.dumps(dict(case=result['case'],kind=stage,task=result['spec']['task'],layout=result['spec']['layout'],policy=result['spec']['policy'],mu=result['spec']['low_mu'],payload=result['spec']['payload_kg'],status=result['status'],reasons=result['reasons'])),flush=True)
                if result['status']=='ERROR':raise ValueError('Invalid attempt retained; investigate before proceeding')
        if stage=='confirmation':
            for task,pair in pairs.items():
                originals=[next(c for c in state['cases'] if c['case']==name) for name in pair]
                assert [c['status'] for c in originals]==['FAIL','PASS'],task
                for original in originals:
                    for repeated in [c for c in state['cases'] if c['kind']=='confirmation' and c['spec']==original['spec']]:
                        assert repeated['status']==original['status'],repeated['case']
                        for metric in ('max_forward_excursion_m','max_path_error_m','final_goal_error_m','final_heading_error_rad'):
                            if metric in original['metrics']:
                                assert abs(repeated['metrics'][metric]-original['metrics'][metric])<=.005,(repeated['case'],metric)
    compiled={}
    for task,pair in pairs.items():
        hashes=[digest(out/name/'compiled-model.txt') for name in pair]
        assert hashes[0]==hashes[1],(task,'Physical fixture changed between variants')
        compiled[task]=hashes[0]
    for case in state['cases']:assert regrade(out/case['case'])=={k:case[k] for k in ('status','reasons','metrics')},case['case']
    state.update(complete=True,counts=dict(Counter(c['status'] for c in state['cases'])),matched_compiled_model_sha256=compiled,
                 known_candidate_failures=[c['case'] for c in state['cases'] if c['spec']['policy']=='surface-cap' and c['status']=='FAIL'],
                 qualification='bounded improvements with explicitly retained split-surface and any prospective limits')
    save(out/'campaign.json',state)
    print(json.dumps(dict(complete=True,episodes=len(state['cases']),counts=state['counts'],known_candidate_failures=state['known_candidate_failures'])),flush=True)


if __name__=='__main__':main()
