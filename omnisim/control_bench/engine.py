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
"""Owned simulator sessions, measured observations, and bounded tool dispatch."""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import time
import uuid
import urllib.error
import urllib.request

from omnisim.paths import REPO_ROOT, resolve_omnisim_binary, linux_runtime_env
from .tasks import ROBOTS, READS, action_valid, position


class InfrastructureError(RuntimeError):
    pass


def request(url, body=None, timeout=90):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type":"application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        if e.code >= 500: raise InfrastructureError(f"Bridge HTTP {e.code}") from e
        return json.loads(e.read())
    except (OSError, ValueError) as e:
        raise InfrastructureError(f"Bridge transport {type(e).__name__}") from e


def failed(result):
    detail = result.get("result", result)
    return (result.get("status") in ("error", "err", "refused") or bool(result.get("error"))
            or detail.get("accepted") is False or isinstance(detail.get("error"), str)
            or detail.get("timed_out") is True or detail.get("settled") is False)


class Engine:
    """New engine per episode: no memory, object, velocity or hold leaks."""
    def __init__(self, robot, directory, key):
        if not key: raise ValueError("An OmniKey is required for every live episode")
        self.robot = robot
        self.surface = ROBOTS[robot]["surface"]
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.proc = None
        self.world = None
        self.log = None
        with socket.socket() as s:
            s.bind(("127.0.0.1",0)); self.port = s.getsockname()[1]
        self.url = f"http://127.0.0.1:{self.port}"
        source = REPO_ROOT/'projects/samples/demos/worlds/chat'/ROBOTS[robot]["world"]
        self.world = source.with_name(f'.control_bench_{uuid.uuid4().hex}.omniworld')
        contents = source.read_text(encoding="utf-8")
        if '"8765"' not in contents: raise ValueError("World has no replaceable bridge port")
        self.world.write_text(contents.replace('"8765"',f'"{self.port}"'),encoding="utf-8")
        binary = resolve_omnisim_binary()
        if not binary: raise InfrastructureError("OmniSim binary not found")
        runtime = Path(binary).parent/'newton-runtime'
        env = linux_runtime_env(REPO_ROOT)
        env.update(OMNISIM_HOME=str(REPO_ROOT), OMNISIM_NO_WINDOW="1",
                   WARP_CACHE_PATH=str(REPO_ROOT/'.tmp/control-bench-warp-cache'),
                   OMNISIM_LOG_PATH=str(self.directory/'engine.log'), OMNI_KEY=key,
                   OMNILINK_MEMORY="0", OMNILINK_PRESENCE="0", OMNILINK_USAGE="0",
                   OMNILINK_PROFILE_SYNC="0", OMNILINK_EDGE="0", OMNILINK_EVENT_WAKE="0",
                   OMNILINK_INTENT_STATE_DIR=str(self.directory/'intents'),
                   OMNILINK_AGENT_TAG=f'control-bench-{uuid.uuid4().hex[:12]}')
        env['PATH'] = os.pathsep.join([str(runtime),str(Path(binary).parent),env.get('PATH','')])
        self.log = (self.directory/'stdout.log').open('w',encoding='utf-8')
        try:
            self.proc = subprocess.Popen([binary,'--batch','--mode=fast','--no-rendering',
                                          '--minimize','--stdout','--stderr',str(self.world)],
                                         cwd=REPO_ROOT,env=env,stdout=self.log,stderr=subprocess.STDOUT,
                                         creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            end = time.monotonic()+90
            while time.monotonic()<end:
                if self.proc.poll() is not None: raise InfrastructureError("Owned engine exited during startup")
                logfile=self.directory/'engine.log'
                if logfile.exists():
                    logtext=logfile.read_text(encoding='utf-8',errors='replace')
                    if 'NO PHYSICS BACKEND IS AVAILABLE' in logtext or 'FATAL:' in logtext:
                        raise InfrastructureError('Physics startup failed; see engine.log')
                try:
                    st=self.state()
                    sidecar=self.directory/'engine.log.newton.json'
                    if sidecar.exists():
                        proof=json.loads(sidecar.read_text())
                        if not proof.get('finalised') or proof.get('degraded'):
                            raise InfrastructureError("No non-degraded physics runtime")
                        # Wait until controller home motion has settled before recording home.
                        if st.get('mode') in ('idle','hold'):
                            time.sleep(.2)
                            self.home=position(self.state(),self.surface)
                            self.physics=proof
                            return
                except (InfrastructureError,ValueError):
                    pass
                time.sleep(.2)
            raise InfrastructureError("Owned bridge did not become ready with physics proof")
        except BaseException:
            self.close(); raise

    def state(self, objects=False):
        st=request(self.url+'/state',timeout=5)
        position(st,self.surface)
        if objects and self.surface=='arm':
            roster=self.tool({'tool':'locate_objects','args':{}},'List the objects.')
            if failed(roster): raise InfrastructureError('Object observations unavailable')
            st['objects']=roster.get('result',roster).get('objects',[])
        return st

    def tool(self, action, utterance):
        action_valid(action,self.surface)
        args=dict(action['args'])
        if action['tool'] in ('drive_forward','turn','set_joint_positions','reset_to_home','pick','place'):
            args['wait']=True
        return request(self.url+'/tool',{'tool':action['tool'],**args,'utterance':utterance,'surface':self.surface})

    def setup(self, actions):
        self.setup_trace=[]
        for action in actions:
            # Setup is explicit typed fixture control, not a language turn.
            before=self.state(objects=True)
            result=self.tool(action,'')
            self.setup_trace.append({'action':action,'before':before,'output':result,'after':self.state(objects=True)})
            if failed(result): raise InfrastructureError('Fixture setup failed')
        time.sleep(.15)
        return self.state(objects=True)

    def close(self):
        errors=[]
        if self.proc is not None and self.proc.poll() is None:
            import psutil
            try:
                parent=psutil.Process(self.proc.pid)
                owned=parent.children(recursive=True)+[parent]
                for p in owned:
                    try: p.terminate()
                    except psutil.NoSuchProcess: pass
                    except psutil.AccessDenied: errors.append('child termination denied')
                _,alive=psutil.wait_procs(owned,timeout=4)
                for p in alive:
                    try: p.kill()
                    except psutil.NoSuchProcess: pass
                    except psutil.AccessDenied: errors.append('child kill denied')
                self.proc.wait(timeout=5)
            except psutil.NoSuchProcess: pass
            except (psutil.AccessDenied,subprocess.TimeoutExpired) as exc:
                errors.append(type(exc).__name__)
                # The Popen handle belongs to this runner even where process
                # enumeration is restricted. Never target an unrelated PID.
                try:
                    self.proc.terminate(); self.proc.wait(timeout=5)
                except (OSError,subprocess.TimeoutExpired) as stop_exc:
                    errors.append(type(stop_exc).__name__)
        if self.log: self.log.close()
        if self.world: self.world.unlink(missing_ok=True)
        if errors:
            (self.directory/'cleanup-errors.json').write_text(json.dumps(errors))
        return errors


class Observation:
    def __init__(self, engine, prompt, fault=None, deadline_s=90, max_actions=24):
        self.engine,self.prompt,self.fault=engine,prompt,fault
        self.started=time.monotonic(); self.deadline=self.started+deadline_s
        self.max_actions=max_actions
        self.initial=engine.state(objects=True)
        self.samples=[]; self.trace=[]; self.errors=[]; self.injected=0
        self.done=threading.Event()
        self.thread=threading.Thread(target=self._sample,daemon=True)
        self.thread.start()

    def _sample(self):
        while not self.done.wait(.03):
            try: self.samples.append(self.engine.state())
            except Exception as exc: self.errors.append(type(exc).__name__)

    def dispatch(self, action):
        if time.monotonic()>self.deadline or len(self.trace)>=self.max_actions:
            raise TimeoutError('Episode action/time budget exhausted')
        action_valid(action,self.engine.surface)
        before=self.engine.state()
        physical=action['tool'] not in READS|{'stop_robot'}
        inject=physical and (self.fault=='all_motion_unavailable' or
                             (self.fault=='first_motion_unavailable' and self.injected==0))
        if inject:
            self.injected+=1
            out={'error':'temporary_unavailable','accepted':False,'moved':False}
        else:
            out=self.engine.tool(action,self.prompt)
        after=self.engine.state(objects=self.engine.surface=='arm')
        self.samples.append(after)
        self.trace.append({'action':action,'before':before,'after':after,'output':out,'injected':inject})
        return out

    def finish(self, response, budget_exhausted=False):
        self.done.set(); self.thread.join(timeout=6)
        if self.thread.is_alive(): self.errors.append('sampler_not_stopped')
        final=self.engine.state(objects=True)
        return {'initial':self.initial,'final':final,'samples':self.samples,'trace':self.trace,
                'sampling_errors':self.errors,'response':response,'budget_exhausted':budget_exhausted,
                'elapsed_s':time.monotonic()-self.started}
