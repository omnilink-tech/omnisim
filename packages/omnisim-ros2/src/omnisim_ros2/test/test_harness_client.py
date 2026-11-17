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

"""Tests for the harness HTTP client against a real in-process HTTP server.

Uses a stdlib ``http.server`` rather than monkeypatching ``urllib`` so the test
exercises actual sockets, headers and status codes -- the layer where the
"return 4xx bodies instead of raising" contract actually lives. No ROS and no
simulator required.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from omnisim_ros2.harness_client import HarnessClient, HarnessUnreachable

RECORDED = []


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _reply(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        RECORDED.append((self.command, self.path, raw.decode() if raw else ""))

        if self.path.startswith("/sim/state"):
            self._send(200, {"world": "w.omniworld", "running": True, "sim_time_ms": 1234.0})
        elif self.path.startswith("/boom"):
            # A coded harness error: the body is the useful part, not the status.
            self._send(422, {"ok": False, "error": "no node with DEF X", "code": "DEF_NOT_FOUND"})
        elif self.path.startswith("/bare"):
            self._send(400, {"error": "steps must be >= 1"})
        elif self.path.startswith("/notjson"):
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.end_headers()
            self.wfile.write(b"\x89PNG\r\n")
        elif self.path.startswith("/array"):
            self._send(200, [1, 2, 3])
        else:
            self._send(200, {"ok": True, "path": self.path})

    do_GET = _reply
    do_POST = _reply

    def _send(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


@pytest.fixture()
def client():
    RECORDED.clear()
    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host, port = server.server_address
    yield HarnessClient(f"http://{host}:{port}", timeout_s=5)
    server.shutdown()


def test_success_is_ok(client):
    r = client.sim_state()
    assert r.ok
    assert r.status == 200
    assert r.body["world"] == "w.omniworld"
    assert r.error == ""


def test_coded_error_is_returned_not_raised(client):
    """A 4xx must not raise: the structured body is the whole point."""
    r = client.get("/boom")
    assert not r.ok
    assert r.status == 422
    assert r.code == "DEF_NOT_FOUND"
    assert "DEF_NOT_FOUND" in r.error
    assert "no node with DEF X" in r.error


def test_bare_error_without_code(client):
    r = client.get("/bare")
    assert not r.ok
    assert r.code == ""
    assert r.error == "steps must be >= 1"


def test_ok_false_in_a_200_body_is_still_a_failure(client):
    r = client.post("/anything")
    assert r.ok  # no "ok" key present at all -> success
    # An explicit ok:false must win even on a 200.
    from omnisim_ros2.harness_client import HarnessResponse

    assert not HarnessResponse(status=200, body={"ok": False, "error": "nope"}).ok
    assert HarnessResponse(status=200, body={}).ok


def test_binary_response_keeps_raw_bytes(client):
    r = client.get("/notjson")
    assert r.ok
    assert r.raw.startswith(b"\x89PNG")
    assert r.body == {}


def test_bare_json_array_is_wrapped(client):
    r = client.get("/array")
    assert r.body == {"items": [1, 2, 3]}


def test_unreachable_raises(client):
    dead = HarnessClient("http://127.0.0.1:9", timeout_s=2)
    with pytest.raises(HarnessUnreachable) as exc:
        dead.sim_state()
    # The message must tell the user how to fix it.
    assert "omnisim harness" in str(exc.value).lower()


def test_query_params_drop_none(client):
    client.scene_tree(bounds=False)
    assert RECORDED[-1][1] == "/scene/tree"
    client.scene_tree(bounds=True)
    assert RECORDED[-1][1] == "/scene/tree?bounds=1"


def test_def_names_are_url_quoted(client):
    """A DEF-less node is named '#185'; an unquoted '#' would truncate the path."""
    client.robot_joints("#185")
    assert RECORDED[-1][1] == "/robot/%23185/joints"
    client.scene_node("A B")
    assert RECORDED[-1][1] == "/scene/node/A%20B"


def test_step_body(client):
    client.sim_step(7)
    method, path, raw = RECORDED[-1]
    assert (method, path) == ("POST", "/sim/step")
    assert json.loads(raw) == {"steps": 7}


def test_delete_sends_defs_list(client):
    client.scene_delete(["A", "B"])
    assert json.loads(RECORDED[-1][2]) == {"defs": ["A", "B"]}
