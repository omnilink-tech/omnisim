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

"""Single bounded rendered preview, run ONLY through thermal_guard.py."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[5]  # repo root (was the literal O:/omnisim in _scratch)
OUT = ROOT / '_scratch/foundry_realism/preview_final'
OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT))
from omnisim.dev.runner import omnisim_env


def api(path,body=None):
    request = urllib.request.Request('http://127.0.0.1:6889'+path,
              data=json.dumps(body).encode() if body is not None else None,
              headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(request,timeout=60) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        print(exc.read().decode(),flush=True)
        raise


env = omnisim_env()
env['OMNISIM_LOG_PATH'] = str(OUT/'preview.log')
env['ORC_FOUNDRY_OUTPUT'] = str(OUT/'preview.events.jsonl')
env['ORC_FOUNDRY_DEMO'] = '1' if '--combat' in sys.argv else '0'
env['ORC_FOUNDRY_DURATION'] = '30'
with (OUT/'harness.log').open('w',encoding='utf-8') as log:
    harness = subprocess.Popen([sys.executable,'scripts/harness/omnisim_harness.py',
            '--port','6889','--supervisor-port','6890','--engine-mode','realtime'],
            cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        for _ in range(40):
            try:
                api('/sim/state')
                break
            except (OSError,ValueError):
                time.sleep(.25)
        result = api('/world/load',{'path':str(ROOT/'projects/robot_combat/orc/worlds/orc_foundry.omniworld'),
                                    'light':True,'wait_s':50})
        (OUT/'load.json').write_text(json.dumps(result,indent=2))
        print('LOAD',result.get('ok'),result.get('load_state'),flush=True)
        if not result.get('ok'):
            raise RuntimeError('World did not load')
        frame = api('/scene/frame',{'def':'ANVIL','push':False})
        (OUT/'frame.json').write_text(json.dumps(frame,indent=2))
        time.sleep(2)
        shot = api('/world/screenshot',{'path':str(OUT/'foundry_preview.png')})
        print('SCREENSHOT',json.dumps(shot),flush=True)
        if '--combat' in sys.argv:
            captured = False
            deadline = time.monotonic()+90
            while time.monotonic() < deadline:
                time.sleep(1)
                rows = [json.loads(line) for line in (OUT/'preview.events.jsonl').read_text().splitlines()]
                samples = [r for r in rows if r['type']=='sample']
                if samples and samples[-1]['t'] > 10 and not captured:
                    print('COMBAT',api('/world/screenshot',{'path':str(OUT/'foundry_combat.png')}),flush=True)
                    captured = True
                if any(r['type']=='result' for r in rows):
                    time.sleep(1)
                    print('AFTERMATH',api('/world/screenshot',{'path':str(OUT/'foundry_aftermath.png')}),flush=True)
                    break
            else:
                raise RuntimeError('Encounter did not finish in the bounded observation')
        (OUT/'state.json').write_text(json.dumps(api('/sim/state'),indent=2))
    finally:
        harness.terminate()
        harness.wait(timeout=10)
