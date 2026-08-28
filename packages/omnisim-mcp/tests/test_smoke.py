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

"""Smoke tests for omnisim-mcp — protocol shape + graceful degradation.

These do NOT need a running harness: the point is to prove the JSON-RPC surface
is correct and that a dead harness produces a clean isError, never a crash.
"""
import json
import sys
from pathlib import Path

# Allow running this file directly without pip-installing (mirrors the
# omnisim-bridges tests -- without it a bare `pytest` at the repo root
# cannot collect this file at all).
PKG_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_mcp import server


def test_initialize_handshake():
    resp = server._handle({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                           "params": {"protocolVersion": "2025-06-18"}})
    assert resp["id"] == 1
    r = resp["result"]
    # we echo the client's protocol version when it sends one
    assert r["protocolVersion"] == "2025-06-18"
    assert "tools" in r["capabilities"]
    assert r["serverInfo"]["name"] == "omnisim-mcp"


def test_initialized_notification_has_no_reply():
    assert server._handle({"jsonrpc": "2.0", "method": "notifications/initialized"}) is None


def test_tools_list_is_complete_and_well_formed():
    resp = server._handle({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
    tools = resp["result"]["tools"]
    names = {t["name"] for t in tools}
    # the load -> inspect -> screenshot -> reload authoring loop must be present
    assert {"load_world", "get_scene_tree", "screenshot", "render_stats",
            "sim_step", "get_events", "harness_status"} <= names
    for t in tools:
        assert t["description"] and isinstance(t["inputSchema"], dict)
        assert t["inputSchema"]["type"] == "object"


def test_unknown_method_is_jsonrpc_error():
    resp = server._handle({"jsonrpc": "2.0", "id": 3, "method": "no/such"})
    assert resp["error"]["code"] == -32601


def test_unknown_tool_is_tool_error_not_crash():
    resp = server._handle({"jsonrpc": "2.0", "id": 4, "method": "tools/call",
                           "params": {"name": "does_not_exist", "arguments": {}}})
    assert resp["result"]["isError"] is True


def test_dead_harness_degrades_gracefully(monkeypatch):
    # point at a port nothing is listening on; every tool must isError cleanly
    monkeypatch.setattr(server, "DEFAULT_HARNESS", "http://127.0.0.1:1")
    monkeypatch.setattr(server, "HTTP_TIMEOUT_S", 1.0)

    status = server._tools_call({"name": "harness_status", "arguments": {}})
    # harness_status reports unreachable as data, not an error
    payload = json.loads(status["content"][0]["text"])
    assert payload["reachable"] is False

    load = server._tools_call({"name": "load_world",
                               "arguments": {"path": "x.wbt"}})
    assert load["isError"] is True
    assert "harness" in load["content"][0]["text"].lower()


def test_missing_required_arg_is_reported():
    resp = server._tools_call({"name": "load_world", "arguments": {}})
    assert resp["isError"] is True
    assert "path" in resp["content"][0]["text"]


def test_load_world_tool_uses_safe_sync_by_default(monkeypatch):
    calls = []
    monkeypatch.setattr(server, "_json_call",
                        lambda method, path, body=None: calls.append((method, path, body)) or {"ok": True})
    server.t_load_world({"path": "scene.wbt", "settle_steps": 12})
    assert calls == [("POST", "/world/sync",
                      {"path": "scene.wbt", "settle_steps": 12})]


def test_load_world_tool_can_force_controller_restarting_reload(monkeypatch):
    calls = []
    monkeypatch.setattr(server, "_json_call",
                        lambda method, path, body=None: calls.append((method, path, body)) or {"ok": True})
    server.t_load_world({"path": "scene.wbt", "force_reload": True,
                         "settle_steps": 12})
    assert calls == [("POST", "/world/load", {"path": "scene.wbt"})]
