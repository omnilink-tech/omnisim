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

"""Retain an owned, authenticated physical development run. Never prints keys."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from omnisim.paths import linux_runtime_env,resolve_omnisim_binary
from omnisim.assessment.runner import _reap_owned,digest,runtime_hashes


def request(base,path,body=None):
    raw=None if body is None else json.dumps(body).encode()
    req=urllib.request.Request(base+path,data=raw,headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=180) as r:return r.status,json.loads(r.read())
    except urllib.error.HTTPError as e:return e.code,json.loads(e.read())


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--live',action='store_true');p.add_argument('--capture',action='store_true');p.add_argument('--key-file',type=Path);p.add_argument('--interrupt',choices=['cancel','stop','live-cancel'])
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    port=18765
    with socket.socket() as s:s.bind(('127.0.0.1',port))
    binary=Path(resolve_omnisim_binary());env=linux_runtime_env(ROOT)
    key=os.environ.get('OMNI_KEY','').strip() or (a.key_file.read_text().strip() if a.key_file else '')
    if not key.startswith('olink_'):raise ValueError('Invalid key-file format; nothing launched')
    env.update(OMNI_KEY=key,OMNILINK_ENGINE='g1-engine',OMNILINK_MODEL='',OMNILINK_AGENT_TAG='mission-dev-'+uuid.uuid4().hex[:8],
        OMNILINK_MEMORY='0',OMNILINK_PROFILE_SYNC='0',OMNISIM_BRIDGE_PARSER_FIRST='0',OMNILINK_EVENT_WAKE='0',
        OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(out/'engine.log'),OMNISIM_MISSION_OUT=str(out),
        OMNISIM_MISSION_CAPTURE='1' if a.capture else '0',OMNILINK_TRACE=str(out/'relay-trace.jsonl'),
        OMNISIM_NEWTON_DUMP_MJMODEL=str(out/'compiled-model.txt'))
    runtime=binary.parent/'newton-runtime';env['PATH']=os.pathsep.join([str(runtime),str(binary.parent),env.get('PATH','')])
    source=ROOT/'projects/samples/demos/worlds/services/husky_mission_agent.omniworld'
    world=source.with_name('.mission_'+uuid.uuid4().hex+'.omniworld')
    text=source.read_text(encoding='utf-8').replace('"8765"',f'"{port}"')
    world.write_text(text,encoding='utf-8');(out/'world.omniworld').write_text(text,encoding='utf-8')
    args=[str(binary),'--port=1286','--batch','--mode=fast','--minimize','--stdout','--stderr']
    if not a.capture:args.append('--no-rendering')
    args.append(str(world));children={};proc=None;result={};base=f'http://127.0.0.1:{port}'
    try:
        with (out/'stdout.log').open('w',encoding='utf-8') as log:
            proc=subprocess.Popen(args,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            import psutil
            deadline=time.monotonic()+75
            while time.monotonic()<deadline:
                for child in psutil.Process(proc.pid).children(recursive=True):children[child.pid]=child
                if proc.poll() is not None:raise RuntimeError('Owned engine exited before bridge readiness')
                try:
                    status,state=request(base,'/state')
                    if state.get('sim_time',0)>1 and state.get('measured_speed_m_s') is not None:break
                except (OSError,ValueError):pass
                time.sleep(.25)
            else:raise RuntimeError('Bridge readiness deadline exceeded')
            result['initial']=state;result['tools']=request(base,'/tools')
            prompt='Visit station A, then station B, and return to base.'
            if a.live:result['dispatch']=request(base,'/prompt',{'text':prompt,'timeout_s':120})
            else:result['dispatch']=request(base,'/tool',{'tool':'start_mission','stations':['A','B','base'],'utterance':prompt})
            print(json.dumps({'dispatch':result['dispatch']}),flush=True)
            if a.interrupt:
                deadline=time.monotonic()+30
                while time.monotonic()<deadline:
                    _,state=request(base,'/state')
                    if state['mission']['status']=='running' and state['sim_time']-state['mission']['started_sim_s']>=.5:break
                    time.sleep(.03)
                result['before_interrupt']=state
                if a.interrupt=='stop':result['interrupt']=request(base,'/stop_robot',{})
                elif a.interrupt=='live-cancel':result['interrupt']=request(base,'/prompt',{'text':'Cancel the mission now.','timeout_s':120})
                else:result['interrupt']=request(base,'/tool',{'tool':'stop_robot','utterance':'Cancel the mission now.'})
                _,after=request(base,'/state');end=after['sim_time']+5
                while after['sim_time']<end:
                    time.sleep(.05);_,after=request(base,'/state')
                result['post_interrupt_observation']=after
            deadline=time.monotonic()+180
            while time.monotonic()<deadline:
                _,state=request(base,'/state')
                if state['mission']['status'] in ('completed','failed','cancelled','stopped'):break
                time.sleep(.15)
            result['final']=state;result['usage']=request(base,'/usage')
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
            print(json.dumps({'mission':state['mission'],'usage':result['usage']}),flush=True)
            request(base,'/stop_robot',{})
    finally:
        if proc:
            try:
                import psutil
                for child in psutil.Process(proc.pid).children(recursive=True):children[child.pid]=child
            except psutil.NoSuchProcess:pass
            result['cleanup_errors']=_reap_owned(proc,children)
        world.unlink(missing_ok=True)
        result['binary_sha256']=digest(binary);result['runtime_sha256']=runtime_hashes(binary)
        (out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        for f in out.iterdir():
            if f.is_file() and f.suffix in ('.json','.jsonl','.log') and key.encode() in f.read_bytes():
                raise RuntimeError('Credential found in retained evidence; do not publish this output')

if __name__=='__main__':main()
