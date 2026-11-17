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

"""Own one simulator and its controllers; no credentials are written to evidence."""
from __future__ import annotations
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


def save(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def source_hashes():
    paths=set((ROOT/'projects/services/husky_mission').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/husky_mission_agent').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/omnilink_mobile_bridge').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/_omnilink_relay').glob('*.py'))
    paths.update((ROOT/'packages/omnisim-bridges/src/omnisim_bridges').glob('*.py'))
    paths.update((ROOT/'omnisim/assessment').glob('*.py'))
    paths.add(ROOT/'projects/assessments/controllers/husky_mixed/drive_allocator.py')
    paths.add(ROOT/'projects/samples/demos/worlds/services/husky_mission_agent.omniworld')
    paths.add(ROOT/'projects/policies/common/env_fingerprint.py')
    paths.update(p for p in (ROOT/'projects/robots/clearpath/husky_description').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    paths.update(p for p in (ROOT/'resources/projects/plugins/robot_windows/omnilink_chat').rglob('*') if p.is_file())
    for name in ('projects/objects/backgrounds/protos/OmniSimSky.proto','projects/objects/lights/protos/OmniSimSun.proto','projects/objects/lights/protos/OmniSimSunMarker.proto'):
        paths.add(ROOT/name)
    return {p.relative_to(ROOT).as_posix():digest(p) for p in sorted(paths)}


class Session:
    def __init__(self,out,key,*,capture=False,profile=False,reference=False):
        self.out=Path(out).resolve();self.key=key;self.capture=capture;self.profile=profile
        self.children={};self.proc=None;self.requests=[];self.manifest={}
        self.reference=reference
    def request(self,path,body=None,timeout=150):
        data=None if body is None else json.dumps(body).encode()
        req=urllib.request.Request(self.base+path,data=data,headers={'Content-Type':'application/json'})
        started=time.monotonic()
        try:
            with urllib.request.urlopen(req,timeout=timeout) as r:code=r.status;reply=json.loads(r.read())
        except urllib.error.HTTPError as e:code=e.code;reply=json.loads(e.read())
        self.requests.append({'path':path,'body':body,'http_status':code,'reply':reply,'wall_elapsed_s':time.monotonic()-started})
        return code,reply
    def state(self):return self.request('/state')[1]
    def wait(self,predicate,timeout=150):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=self.state()
            if predicate(state):return state
            if self.proc.poll() is not None:raise RuntimeError('Owned engine exited')
            time.sleep(.05)
        raise TimeoutError('Observation condition not reached; retain evidence')
    def observe(self,seconds):
        start=self.state()['sim_time'];return self.wait(lambda s:s['sim_time']>=start+seconds)
    def __enter__(self):
        import psutil
        self.out.mkdir(parents=True,exist_ok=False)
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        self.base=f'http://127.0.0.1:{port}'
        binary=Path(resolve_omnisim_binary());env=linux_runtime_env(ROOT)
        env.pop('OMNI_KEY',None)
        if self.key:env['OMNI_KEY']=self.key
        env.update(OMNILINK_ENGINE=os.environ.get('OMNILINK_ENGINE','g1-engine'),OMNILINK_MODEL=os.environ.get('OMNILINK_MODEL',''),OMNILINK_AGENT_TAG='mission-'+uuid.uuid4().hex[:10],
            OMNILINK_MEMORY='0',OMNILINK_PROFILE_SYNC='1' if self.profile else '0',OMNISIM_BRIDGE_PARSER_FIRST='0',OMNILINK_EVENT_WAKE='0',
            OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(self.out/'engine.log'),OMNISIM_MISSION_OUT=str(self.out),
            OMNISIM_MISSION_CAPTURE='1' if self.capture else '0',OMNILINK_TRACE=str(self.out/'relay-trace.jsonl'),
            OMNISIM_NEWTON_DUMP_MJMODEL=str(self.out/'compiled-model.txt'))
        runtime=binary.parent/'newton-runtime';env['PATH']=os.pathsep.join([str(runtime),str(binary.parent),env.get('PATH','')])
        temporary=self.out/'temp';temporary.mkdir();env['TEMP']=env['TMP']=str(temporary)
        source=ROOT/'projects/samples/demos/worlds/services/husky_mission_agent.omniworld'
        self.world=source.with_name('.mission_'+uuid.uuid4().hex+'.omniworld')
        contents=source.read_text(encoding='utf-8').replace('"8765"',f'"{port}"')
        if self.reference:
            contents=contents.replace('controller "husky_mission_agent"','controller "omnilink_mobile_bridge"')
            env['OMNISIM_MOBILE_TRACE_PATH']=str(self.out/'reference-trace.jsonl')
        self.world.write_text(contents,encoding='utf-8');(self.out/'world.omniworld').write_text(contents,encoding='utf-8')
        self.manifest={'sources_sha256':source_hashes(),'binary_sha256':digest(binary),'runtime_sha256':runtime_hashes(binary),
                       'settings':{'solver':'MuJoCo CPU','timestep_ms':16,'configured_substeps':8,'parser_first':False,'memory':False},
                       'capture':self.capture,'credential_included':False,'owned_process':True}
        save(self.out/'manifest.json',self.manifest)
        args=[str(binary),'--batch','--mode=fast','--minimize','--stdout','--stderr']
        if not self.capture:args.append('--no-rendering')
        args.append(str(self.world));self.log=(self.out/'stdout.log').open('w',encoding='utf-8')
        try:
            self.proc=subprocess.Popen(args,cwd=ROOT,env=env,stdout=self.log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            deadline=time.monotonic()+75
            while time.monotonic()<deadline:
                for child in psutil.Process(self.proc.pid).children(recursive=True):self.children[child.pid]=child
                if self.proc.poll() is not None:raise RuntimeError('Owned engine exited before readiness')
                try:
                    s=self.state()
                    if s.get('sim_time',0)>1 and (self.reference or s.get('measured_speed_m_s') is not None):return self
                except (OSError,ValueError):pass
                time.sleep(.15)
            raise TimeoutError('Bridge readiness deadline')
        except BaseException:
            self.__exit__(*sys.exc_info());raise
    def __exit__(self,kind,error,tb):
        import psutil
        if self.proc:
            try:
                for child in psutil.Process(self.proc.pid).children(recursive=True):self.children[child.pid]=child
            except psutil.NoSuchProcess:pass
            self.manifest['cleanup_errors']=_reap_owned(self.proc,self.children)
        self.world.unlink(missing_ok=True);self.log.close()
        from projects.policies.common.env_fingerprint import collect
        save(self.out/'fingerprint.json',collect(engine_log_path=self.out/'engine.log',repo_root=ROOT,warm=False))
        save(self.out/'requests.json',self.requests)
        self.manifest['error_type']=None if kind is None else kind.__name__
        self.manifest['artifacts_sha256']={p.name:digest(p) for p in self.out.iterdir() if p.is_file() and p.name not in ('manifest.json','verdict.json')}
        save(self.out/'manifest.json',self.manifest)
        if self.key:
            for f in self.out.iterdir():
                if f.is_file() and f.suffix in ('.json','.jsonl','.log','.txt') and self.key.encode() in f.read_bytes():
                    raise RuntimeError('Credential detected in retained evidence; publication refused')
