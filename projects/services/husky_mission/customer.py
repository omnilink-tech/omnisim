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

"""Public customer CLI. Requires the customer's own OmniKey for fresh agent runs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from qualification import definitions,run_case
from session import ROOT,save
from verify import verify_campaign


def verify():
    manifest=ROOT/'delivery-manifest.json'
    if manifest.exists():
        files=json.loads(manifest.read_text())['files_sha256']
        for name,expected in files.items():
            target=(ROOT/name).resolve()
            if not target.is_relative_to(ROOT) or hashlib.sha256(target.read_bytes()).hexdigest()!=expected:
                raise ValueError('Delivery inventory changed: '+name)
    else:files={}
    result=verify_campaign(ROOT/'evidence');return {**result,'inventory_files':len(files)}


def main():
    p=argparse.ArgumentParser(description='Husky Mission Agent: measured simulation and rerunnable acceptance tests')
    sub=p.add_subparsers(dest='command',required=True);sub.add_parser('verify')
    for name in ('run','accept','suite'):
        q=sub.add_parser(name);q.add_argument('--out',type=Path,required=True)
        if name=='run':q.add_argument('--case',required=True);q.add_argument('--capture',action='store_true')
    a=p.parse_args()
    try:
        if a.command=='verify':print(json.dumps(verify()));return 0
        key=os.environ.get('OMNI_KEY','').strip()
        if not key.startswith('olink_'):raise ValueError('Connect your OmniKey and model provider first; set OMNI_KEY in this process. No account is included.')
        if a.out.resolve().is_relative_to(ROOT/'evidence'):raise ValueError('Fresh runs cannot overwrite frozen evidence')
        plan=definitions()
        if a.command=='run':
            c=next((c for c in plan if c['case']==a.case),None)
            if c is None:raise ValueError('Unknown case ID')
            result=run_case(c,a.out,key,a.capture);print(json.dumps(result));return {'PASS':0,'FAIL':1,'ERROR':2}[result['verdict']['status']]
        selected=plan if a.command=='suite' else [plan[i] for i in (0,3,6,9,12)]
        a.out.mkdir(parents=True,exist_ok=False);results=[]
        for case in selected:
            result=run_case(case,a.out/case['case'],key);results.append(result)
            save(a.out/'acceptance.json',{'complete':False,'cases':results})
            print(json.dumps(result),flush=True)
        passed=all(c['verdict']['status']=='PASS' for c in results)
        save(a.out/'acceptance.json',{'complete':True,'status':'PASS' if passed else 'FAIL','cases':results,
                                    'scope':'Fresh runs with this model/provider/runtime; no bitwise determinism or physical-robot qualification implied.'})
        return 0 if passed else 1
    except Exception as exc:
        print(str(exc),file=sys.stderr);return 2

if __name__=='__main__':raise SystemExit(main())
