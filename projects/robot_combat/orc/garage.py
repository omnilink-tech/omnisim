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
"""Local Foundry garage. Starts only its own temperature-guarded matches."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
import webbrowser

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(HERE/'shared'))
from foundry_config import normalize_match, catalog, build_stats
from build_foundry import build
from omnisim.dev.runner import omnisim_env


class Garage:
    def __init__(self, storage=None):
        self.storage = Path(storage or ROOT/'_scratch/foundry_garage')
        self.storage.mkdir(parents=True,exist_ok=True)
        self.lock = threading.RLock()
        self.token = secrets.token_urlsafe(32)
        self.process = None
        self.worker = None
        self.current = None
        self.history = []
        try:
            saved = json.loads((self.storage/'garage.json').read_text())
            self.draft = normalize_match(saved['draft'])
            self.history = saved.get('history',[])[:12]
        except (OSError, ValueError, KeyError, TypeError):
            self.draft = normalize_match()

    def persist(self):
        temporary = self.storage/'garage.next.json'
        temporary.write_text(json.dumps({'draft':self.draft,'history':self.history},indent=2),encoding='utf-8')
        temporary.replace(self.storage/'garage.json')

    def save(self, data):
        draft = normalize_match(data)
        with self.lock:
            self.draft = draft
            self.persist()
        return draft

    def active(self):
        return self.current is not None and self.current['status'] in ('starting','running','stopping')

    def start(self, data):
        match = normalize_match(data)
        with self.lock:
            if self.active():
                raise RuntimeError('A match is already running. Finish or stop it first.')
            match_id = time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3)
            folder = self.storage/match_id
            folder.mkdir()
            (folder/'builds.json').write_text(json.dumps(match,indent=2),encoding='utf-8')
            self.draft = match
            self.persist()
            self.current = {'id':match_id,'status':'starting','match':match,'t':0,'remaining_s':match['duration_s'],
                            'fighters':{},'events':[],'result':None,'error':None,'gpu_peak_c':None,
                            'preview':False,'preview_revision':0,'cancelled':False}
            self.worker = threading.Thread(target=self._run,args=(match_id,match,folder),daemon=True)
            self.worker.start()
            return self.snapshot()

    def stop(self):
        with self.lock:
            if self.active():
                self.current['cancelled'] = True
                self.current['status'] = 'stopping'
                process = self.process
                if process is not None and process.poll() is None:
                    # This is our guard PID. Its Windows job owns/reaps only
                    # the match engine and controllers when the handle closes.
                    if os.name == 'nt':
                        process.terminate()
                    else:
                        process.send_signal(signal.SIGINT)
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps({'current':self.current,'history':self.history}))

    def apply_event(self,event):
        with self.lock:
            state = self.current
            if event.get('type') == 'sample':
                state.update(t=event['t'],remaining_s=event.get('remaining_s',0),fighters=event.get('fighters',{}))
                if state['status'] != 'stopping':
                    state['status'] = 'running'
                state['phase'] = event.get('phase')
            elif event.get('type') == 'result':
                state['result'] = event
                state['fighters'] = event.get('fighters',state['fighters'])
            elif event.get('type') == 'error':
                state['error'] = event.get('message','The match could not continue.')
            if event.get('type') in ('detach','overturned','immobilized','result','error'):
                state['events'] = (state['events']+[event])[-16:]

    def _run(self,match_id,match,folder):
        world = HERE/'worlds'/('.foundry_match_'+match_id+'.omniworld')
        returncode = None
        try:
            world.write_text(build(match,garage=True),encoding='utf-8')
            # Keep the exact authored world with the match record after the
            # temporary sibling (needed for controller discovery) is removed.
            (folder/'match.omniworld').write_bytes(world.read_bytes())
            os.environ.setdefault('PYTHON_HOME',str(Path(sys.executable).parent))
            env = omnisim_env()
            env.update(OMNISIM_LOG_PATH=str(folder/'engine.log'),ORC_FOUNDRY_OUTPUT=str(folder/'events.jsonl'),
                       WARP_CACHE_PATH=str(ROOT/'_scratch/foundry/warp_cache'),PYTHONUNBUFFERED='1')
            # A session's blueprint, not inherited demo/probe settings, owns
            # its duration and controls. No arbitrary executable enters here.
            for key in ('ORC_FOUNDRY_DEMO','ORC_FOUNDRY_DURATION','ORC_FOUNDRY_CAPTURE','ORC_FOUNDRY_PROBE'):
                env.pop(key,None)
            command = [sys.executable,str(ROOT/'scripts/dev/thermal_guard.py'),'run',
                       '--ceiling','75','--interval','.5','--precool','65','--',
                       sys.executable,str(ROOT/'scripts/dev/headless_runner.py'),str(world),
                       '--duration',str(max(900,match['duration_s']*30)),
                       '--gui','--wait-for-step','--realtime','--fail-on-runaway','--race-attempts','1']
            with (folder/'run.log').open('w',encoding='utf-8') as log:
                with self.lock:
                    if self.current['cancelled']:
                        return
                    self.process = subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
                offset = 0
                pending = b''
                def read_events():
                    nonlocal offset,pending
                    path = folder/'events.jsonl'
                    if not path.exists():
                        return
                    with path.open('rb') as stream:
                        stream.seek(offset)
                        chunk = stream.read()
                        offset = stream.tell()
                    lines = (pending+chunk).split(b'\n')
                    pending = lines.pop()
                    for line in lines:
                        if line.strip():
                            self.apply_event(json.loads(line))
                    preview = folder/'preview.jpg'
                    if preview.exists():
                        with self.lock:
                            self.current.update(preview=True,preview_revision=preview.stat().st_mtime_ns)
                while self.process.poll() is None:
                    read_events()
                    time.sleep(.25)
                returncode = self.process.wait()
                read_events()
        except Exception as exc:
            with self.lock:
                self.current['error'] = str(exc)
            self.stop()
        finally:
            process = self.process
            if process is not None and process.poll() is None:
                self.stop()
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            world.unlink(missing_ok=True)
            run_log = (folder/'run.log').read_text(errors='replace') if (folder/'run.log').exists() else ''
            peaks = re.findall(r'thermal_guard: peak=(\d+) C',run_log)
            with self.lock:
                state = self.current
                if peaks:
                    state['gpu_peak_c'] = int(peaks[-1])
                if state['cancelled'] and not state['error']:
                    state['status'] = 'stopped'
                elif returncode != 0 or state['error'] or not state['result']:
                    state['status'] = 'error'
                    state['error'] = state['error'] or (
                        'GPU protection stopped the match. Let the laptop cool before retrying.' if 'BREACHED' in run_log else
                        'The GPU temperature sensor was unavailable. The match was stopped.' if 'SENSOR LOST' in run_log or 'cannot read GPU' in run_log else
                        'The arena closed before the match finished. You can retry your saved builds.')
                else:
                    state['status'] = 'finished'
                record = json.loads(json.dumps(state))
                (folder/'result.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
                self.history = ([record]+self.history)[:12]
                self.process = None
                self.persist()


def make_server(garage,port=0):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def reply(self,code,payload,content_type='application/json'):
            data = json.dumps(payload).encode() if content_type == 'application/json' else payload
            self.send_response(code)
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):
                pass

        def allowed(self):
            host = self.headers.get('Host','')
            return host in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.allowed():
                return self.reply(403,{'error':'Open the garage using its local address.'})
            path = urlsplit(self.path).path
            if path in ('/','/app.js','/style.css'):
                name,mime = {'/':('index.html','text/html; charset=utf-8'),
                             '/app.js':('app.js','text/javascript; charset=utf-8'),
                             '/style.css':('style.css','text/css; charset=utf-8')}[path]
                return self.reply(200,(HERE/'garage_ui'/name).read_bytes(),mime)
            if path == '/api/bootstrap':
                with garage.lock:
                    return self.reply(200,{'catalog':catalog(),'draft':garage.draft,'token':garage.token,**garage.snapshot()})
            if path == '/api/state':
                return self.reply(200,garage.snapshot())
            if path == '/api/preview.jpg':
                current = garage.snapshot()['current']
                image = garage.storage/current['id']/'preview.jpg' if current else None
                if image and image.exists():
                    return self.reply(200,image.read_bytes(),'image/jpeg')
                return self.reply(404,{'error':'The arena preview is not ready yet.'})
            return self.reply(404,{'error':'Page not found.'})

        def do_POST(self):
            origin = self.headers.get('Origin')
            expected = (f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}')
            if (not self.allowed() or (origin and origin not in expected) or
                    self.headers.get('X-ORC-Token') != garage.token):
                return self.reply(403,{'error':'Reload the garage and try again.'})
            if self.headers.get('Content-Type','').split(';')[0] != 'application/json':
                return self.reply(415,{'error':'Send a robot blueprint as JSON.'})
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 32768:
                    return self.reply(413,{'error':'This blueprint is too large.'})
                data = json.loads(self.rfile.read(length))
                path = urlsplit(self.path).path
                if path == '/api/preview':
                    match = normalize_match(data)
                    return self.reply(200,{'match':match,'stats':{slot:build_stats(robot) for slot,robot in match['robots'].items()}})
                if path == '/api/save':
                    return self.reply(200,{'draft':garage.save(data)})
                if path == '/api/start':
                    return self.reply(202,garage.start(data))
                if path == '/api/stop':
                    return self.reply(200,garage.stop())
                return self.reply(404,{'error':'Action not found.'})
            except (ValueError,TypeError,KeyError) as exc:
                return self.reply(400,{'error':str(exc)})
            except RuntimeError as exc:
                return self.reply(409,{'error':str(exc)})
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)


def serve(open_browser=True):
    garage = Garage()
    server = make_server(garage)
    url = f'http://127.0.0.1:{server.server_port}'
    (garage.storage/'session.json').write_text(json.dumps({'url':url,'pid':os.getpid()}))
    print(f'Foundry garage: {url}',flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        garage.stop()
        if garage.worker:
            garage.worker.join(timeout=20)
        server.server_close()
