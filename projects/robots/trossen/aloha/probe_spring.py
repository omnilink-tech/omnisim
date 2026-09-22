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
# limitations under the License."""Measure the physical terminal displacement under a known load and on release."""
import argparse,json,time,uuid,urllib.request
from pathlib import Path
from build_spring_scene import scene_text, ROOT


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--harness',required=True)
    ap.add_argument('--out-dir',type=Path,required=True);ap.add_argument('--no-spring',action='store_true')
    ap.add_argument('--case',choices=['spring-law','seated','drop','table'],default='spring-law')
    args=ap.parse_args();out=args.out_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    cfg={'out_dir':str(out),'spring_probe':args.case=='spring-law','contact_probe':args.case if args.case!='spring-law' else None}
    if args.no_spring:cfg['spring_parameters']={'stiffness_n_m':0,'damping_n_s_m':0,'preload_n':0}
    (out/'config.json').write_text(json.dumps(cfg))
    scene=scene_text();start=scene.index('DEF BATTERY Solid' if args.case=='spring-law' else 'DEF ALOHA_LEFT');end=scene.index('DEF COMPARTMENT_CONTROLLER')
    scene=scene[:start]+scene[end:]
    if args.case!='spring-law':
        height=.0228 if args.case=='seated' else .080
        scene=scene.replace('translation .069 .016 .0168',f'translation -.039 .0335 {height}')
        if args.case=='table':
            scene=scene.replace(f'translation -.039 .0335 {height}','translation .10 .10 .060')
            scene=scene.replace('translation 0 0 -.025','translation 0 0 -.015')
        if args.case=='seated':
            scene=scene.replace('axis 1 0 0 minStop -.010','position -.006 axis 1 0 0 minStop -.010')
            scene=scene.replace('translation -.0001 -.0095 .0128','translation -.0061 -.0095 .0128')
    scene=scene.replace('physics Physics { density -1 mass .055 }','')
    scene=scene.replace('controller "battery_compartment"',f'controller "battery_compartment" controllerArgs [ "--config" "{(out/"config.json").as_posix()}" ]')
    scene=scene.replace('ALOHA - spring compartment development','Spring probe '+uuid.uuid4().hex)
    path=ROOT/'worlds'/('.spring_probe_'+uuid.uuid4().hex+'.omniworld');path.write_text(scene,encoding='utf-8')
    (out/'world.omniworld.txt').write_text(scene,encoding='utf-8')
    req=urllib.request.Request(args.harness.rstrip('/')+'/world/sync',data=json.dumps({'path':str(path),'light':True}).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=120) as response:reply=json.load(response)
    if not reply.get('ok'):raise RuntimeError(reply)
    deadline=time.monotonic()+120
    while time.monotonic()<deadline:
        if (out/'spring-error.txt').exists():raise RuntimeError((out/'spring-error.txt').read_text())
        if (out/'spring-result.json').exists():
            result=json.loads((out/'spring-result.json').read_text());print(json.dumps(result,indent=2))
            expected_failure=args.no_spring or args.case=='drop'
            return 0 if result['passed']!=expected_failure else 2
        time.sleep(.2)
    raise TimeoutError('No complete spring evidence')


if __name__=='__main__':raise SystemExit(main())
