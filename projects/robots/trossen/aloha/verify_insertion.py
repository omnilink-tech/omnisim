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
"""Re-evaluate saved insertion evidence and compare a repeat and open-grip control."""
import argparse
import hashlib
import json
from pathlib import Path
from evaluate import evaluate, evaluate_insertion


def read(path):
    return json.loads(path.read_text())


def check_run(path):
    cfg=read(path/'config.json'); result=read(path/'result.json')
    manifest=read(path/'run_manifest.json')
    rows=[json.loads(line) for line in (path/'trace.jsonl').read_text().splitlines()]
    assert result['complete'] and len(rows)==result['frames']==len(cfg['targets'])
    assert [row['frame'] for row in rows]==list(range(len(rows)))
    assert result['timed_episode_pose_writes']==0 and result['actuation']=='Motor.setPosition'
    audit=read(path/'mechanism-audit.json')
    assert audit['complete'] and audit['sampling_hz']==1000
    assert audit['samples']>=cfg['duration_s']*audit['sampling_hz']
    depth_ok=max(audit['negative_contact_peak_depth_m'],audit['housing_contact_peak_depth_m'])<=.0015
    checks={path/'config.json':manifest['config_sha256'],
            path/'world.omniworld.txt':manifest['world_sha256']}
    checks.update({path/'sources'/name:digest for name,digest in manifest['controller_sha256'].items()})
    checks.update({path/'sources'/f'aloha_{side}_spring.urdf':digest for side,digest in manifest['model_sha256'].items()})
    assert all(hashlib.sha256(file.read_bytes()).hexdigest()==digest for file,digest in checks.items())
    engine=read(path/'engine.newton.json')
    assert engine['finalised'] and not engine['degraded'] and engine['backend']=='newton'
    placement=evaluate(rows,result,cfg['fps'],rows[0]['battery_position'][2])
    mechanics=evaluate_insertion(rows,placement,cfg['fps'])
    passed=bool(mechanics['task_success'] and depth_ok)
    assert result['task_success']==passed
    return {'passed':passed,'placement':placement,'mechanics':mechanics,'physics_step_audit':audit,
            'verified_source_hashes':len(checks)},read(path/'executed_targets.json')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--repeat',type=Path,required=True)
    ap.add_argument('--no-grip',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    result,targets=check_run(args.run)
    repeat,_=check_run(args.repeat)
    control,control_targets=check_run(args.no_grip)
    identical=(args.run/'trace.jsonl').read_bytes()==(args.repeat/'trace.jsonl').read_bytes()
    same_commands=all(a[:14]==b[:14] and b[14:]==[.06,-.06] for a,b in zip(targets,control_targets))
    same_commands=same_commands and len(targets)==len(control_targets)
    passed=(result['passed'] and repeat['passed'] and identical and same_commands
            and not control['placement']['carried'] and not control['placement']['inside_slot'])
    report={'passed':bool(passed),'run':result,'repeat':repeat,'no_grip':control,
            'repeat_trace_identical':identical,'no_grip_same_arm_commands':same_commands,
            'sampled_frames':len(targets),'scope':'Two deterministic simulation runs and one negative control; not a general success rate or hardware validation'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ('passed','repeat_trace_identical','no_grip_same_arm_commands','sampled_frames')},indent=2))
    return 0 if passed else 2


if __name__=='__main__':raise SystemExit(main())
