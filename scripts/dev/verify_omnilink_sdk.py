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
"""Exercise the installed OmniLink SDK over loopback HTTP. No account or billing.

Run with the actual controller interpreter. The HTTP responses are scripted;
this checks SDK/relay compatibility, not model quality or robot physics.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def main():
    for flag in ('VERSION_CHECK', 'PRESENCE', 'JOURNAL_PERSIST'):
        os.environ['OMNILINK_' + flag] = '0'
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'packages/omnisim-bridges/src'))
    from omnisim_bridges import relay
    import omnilink
    requests = []
    dispatched = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            requests.append((self.path, self.headers.get('Authorization'), body))
            if len(requests) == 1:
                payload = {'ok': True, 'text': '', 'toolCalls': [{
                    'id': 'verify-1', 'name': 'drive_forward', 'arguments': {'distance': 0.25}}]}
            else:
                payload = {'ok': True, 'text': 'Motion measured.', 'toolCalls': []}
            data = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    relay.BASE_URL = f'http://127.0.0.1:{server.server_port}'

    def dispatch(args):
        dispatched.append(args)
        return {'commanded': args['distance'], 'achieved': args['distance'], 'settled': True}

    agent = None
    try:
        tool = relay.Tool(name='drive_forward', description='Drive a measured distance.',
                          parameters={'type': 'object', 'properties': {'distance': {'type': 'number'}},
                                      'required': ['distance']}, dispatch=dispatch)
        agent = relay.OmniLinkRelay(omni_key='olink_loopback_test_only', agent_name='SDK verification',
                                   main_task='Verify transport', tools=[tool], model='',
                                   memory_enabled=False, usage_enabled=False, voice_out_enabled=False)
        result = agent.dispatch_sync('Drive 0.25 metres.', timeout_s=15)
        assert dispatched == [{'distance': 0.25}], result
        assert len(requests) == 2, result
        assert all(path == '/api/chat' and auth == 'Bearer olink_loopback_test_only'
                   for path, auth, _ in requests)
        assert any(m.get('role') == 'tool' for m in requests[1][2]['messages'])
        assert 'Motion measured.' in str(result), result
        print(json.dumps({'status': 'PASS', 'sdk': omnilink.__version__,
                          'sdk_path': omnilink.__file__, 'chat_requests': len(requests),
                          'tool_dispatches': len(dispatched), 'transport': 'loopback scripted HTTP'}))
    finally:
        if agent:
            agent.close()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
