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
import re

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from omnisim.paths import linux_runtime_env,resolve_omnisim_binary
from omnisim.assessment.runner import _reap_owned,digest,runtime_hashes


def save(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def resolve_agent_tag(explicit=None,environ=None):
    """Keep scratch identities stable; reject tags the bridge would rewrite."""
    environ=os.environ if environ is None else environ
    tag=explicit if explicit is not None else (environ.get('OMNILINK_AGENT_TAG') or 'arm-assurance-demo')
    if not isinstance(tag,str):raise ValueError('Agent tag must be a string')
    tag=tag.strip()
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,32}',tag) or tag!=tag.strip('-'):raise ValueError('Use an unchanged 1-32 character scratch tag containing letters, digits, underscores, dots or hyphens')
    return tag


def source_hashes():
    paths=set((ROOT/'projects/services/arm_assurance').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/arm_assurance_bridge').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/omnilink_arm_bridge').glob('*.py'))
    paths.update((ROOT/'projects/samples/demos/controllers/_omnilink_relay').glob('*.py'))
    paths.update((ROOT/'packages/omnisim-bridges/src/omnisim_bridges').glob('*.py'))
    paths.update((ROOT/'omnisim/assessment').glob('*.py'))
    paths.add(ROOT/'projects/samples/demos/worlds/services/omniarm6_assurance.omniworld')
    paths.add(ROOT/'projects/policies/common/env_fingerprint.py')
    paths.add(ROOT/'tests/test_arm_assurance.py')
    paths.add(ROOT/'packages/omnisim-bridges/tests/test_relay_refusal_history.py')
    paths.add(ROOT/'arm_assurance.py')
    paths.add(ROOT/'projects/services/arm_assurance/README.md')
    paths.update(p for p in (ROOT/'projects/robots/omnisim/omniarm6').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    paths.update(p for p in (ROOT/'resources/projects/plugins/robot_windows/omnilink_chat').rglob('*') if p.is_file())
    for name in ('projects/objects/backgrounds/protos/OmniSimSky.proto','projects/objects/lights/protos/OmniSimSun.proto','projects/objects/lights/protos/OmniSimSunMarker.proto'):
        paths.add(ROOT/name)
    return {p.relative_to(ROOT).as_posix():digest(p) for p in sorted(paths)}


class Session:
    def __init__(self,out,key,*,capture=False,profile=False,reference=False,mode='realtime',agent_tag=None):
        self.out=Path(out).resolve();self.key=key;self.capture=capture;self.profile=profile
        self.children={};self.proc=None;self.requests=[];self.manifest={}
        self.reference=reference
        self.agent_tag=resolve_agent_tag(agent_tag)
        if mode not in ('realtime','fast'):raise ValueError('Unsupported engine mode')
        self.engine_mode=mode
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
        env.update(OMNILINK_ENGINE=os.environ.get('OMNILINK_ENGINE','g1-engine'),OMNILINK_MODEL=os.environ.get('OMNILINK_MODEL',''),OMNILINK_AGENT_TAG=self.agent_tag,
            OMNILINK_MEMORY='0',OMNILINK_PROFILE_SYNC='1' if self.profile else '0',OMNISIM_BRIDGE_PARSER_FIRST='0',OMNILINK_EVENT_WAKE='0',
            OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(self.out/'engine.log'),OMNISIM_ARM_ASSURANCE_OUT=str(self.out),
            OMNISIM_ARM_ASSURANCE_CAPTURE='1' if self.capture else '0',OMNILINK_TRACE=str(self.out/'relay-trace.jsonl'),
            OMNISIM_ARM_PROFILE='canonical' if self.reference else 'guarded',
            OMNISIM_NEWTON_DUMP_MJMODEL=str(self.out/'compiled-model.txt'))
        runtime=binary.parent/'newton-runtime';env['PATH']=os.pathsep.join([str(runtime),str(binary.parent),env.get('PATH','')])
        temporary=self.out/'temp';temporary.mkdir();env['TEMP']=env['TMP']=str(temporary)
        # The account profile can be reused; local commitments and action
        # history cannot. Override any inherited durable-state directory.
        env['OMNILINK_INTENT_STATE_DIR']=str(temporary/'intent-state')
        source=ROOT/'projects/samples/demos/worlds/services/omniarm6_assurance.omniworld'
        self.world=source.with_name('.assurance_'+uuid.uuid4().hex+'.omniworld')
        contents=source.read_text(encoding='utf-8').replace('"8765"',f'"{port}"')
        self.world.write_text(contents,encoding='utf-8');(self.out/'world.omniworld').write_text(contents,encoding='utf-8')
        self.manifest={'sources_sha256':source_hashes(),'binary_sha256':digest(binary),'runtime_sha256':runtime_hashes(binary),
                       'settings':{'solver':'MuJoCo CPU','timestep_ms':16,'configured_substeps':8,'parser_first':False,'memory':False,'profile_sync':self.profile,'engine_mode':self.engine_mode,'profile':'canonical' if self.reference else 'guarded',
                                   'agent_tag':self.agent_tag,'account_identity_policy':'stable scratch profile identifier reused sequentially; fresh local relay and history per process','local_state_policy':'new private per-run intent/action-journal directory'},
                       'capture':self.capture,'credential_included':False,'owned_process':True}
        save(self.out/'manifest.json',self.manifest)
        args=[str(binary),'--batch','--mode='+self.engine_mode,'--minimize','--stdout','--stderr']
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
                    qualified=self.reference or (s.get('assurance',{}).get('ready') and not s['assurance'].get('fault'))
                    if qualified and s.get('sim_time',0)>3 and s.get('max_joint_speed_rad_s') is not None and s['max_joint_speed_rad_s']<.03 and s.get('mode') in ('idle','hold'):return self
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
