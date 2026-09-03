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


def test_new_endpoint_tools_are_listed():
    resp = server._handle({"jsonrpc": "2.0", "id": 7, "method": "tools/list"})
    names = {t["name"] for t in resp["result"]["tools"]}
    assert {"get_capabilities", "world_sync", "scene_spawn", "scene_delete",
            "scene_set_pose", "sim_snapshot", "sim_restore", "list_snapshots",
            "get_grips", "robot_devices", "robot_joints_set", "robot_ik"} <= names


def test_sim_reset_forwards_its_arguments(monkeypatch):
    calls = []
    monkeypatch.setattr(server, "_json_call",
                        lambda method, path, body=None: calls.append((method, path, body)) or {"ok": True})
    server.t_sim_reset({"restore": None, "verify": True, "settle_steps": 3})
    assert calls == [("POST", "/sim/reset",
                      {"restore": None, "verify": True, "settle_steps": 3})]


def test_screenshot_path_passes_harness_json_through(monkeypatch):
    # The harness answers a server-side write with measured JSON
    # ({path, bytes, pixels, ...}); the tool must pass it through, never
    # fabricate {"written": <echoed arg>, "bytes": <len of the JSON body>}.
    harness_body = {"path": "C:/real/shot.png", "bytes": 123456,
                    "pixels": [1280, 720]}
    monkeypatch.setattr(
        server, "_request",
        lambda method, path, body=None, base=None:
            (200, {"Content-Type": "application/json"},
             json.dumps(harness_body).encode()))
    out = server.t_screenshot({"path": "asked/for.png"})
    payload = json.loads(out[0]["text"])
    assert payload == harness_body
    assert "written" not in payload


def test_ping_is_answered_from_the_reader_path():
    # ping must never be routed through the tool worker queue
    resp = server._handle({"jsonrpc": "2.0", "id": 9, "method": "ping"})
    assert resp == {"jsonrpc": "2.0", "id": 9, "result": {}}


def test_load_world_tool_can_force_controller_restarting_reload(monkeypatch):
    calls = []
    monkeypatch.setattr(server, "_json_call",
                        lambda method, path, body=None: calls.append((method, path, body)) or {"ok": True})
    server.t_load_world({"path": "scene.wbt", "force_reload": True,
                         "settle_steps": 12})
    assert calls == [("POST", "/world/load", {"path": "scene.wbt"})]


# --- 2026-09-02: tools for the routes that had none, and the stale-text pins ---

def _capture(monkeypatch):
    calls = []
    monkeypatch.setattr(server, "_json_call",
                        lambda method, path, body=None: calls.append((method, path, body)) or {"ok": True})
    return calls


def test_every_harness_route_family_has_a_tool():
    resp = server._handle({"jsonrpc": "2.0", "id": 11, "method": "tools/list"})
    names = {t["name"] for t in resp["result"]["tools"]}
    assert {"robot_damage", "robot_damage_events", "robot_damage_reset",
            "robot_damage_inject", "read_bench", "scene_node_particles",
            "rebuild_physics"} <= names


def test_damage_tools_hit_their_routes(monkeypatch):
    calls = _capture(monkeypatch)
    server.t_robot_damage({})
    server.t_robot_damage_events({"since": 3, "limit": 10})
    server.t_robot_damage_reset({})
    server.t_robot_damage_inject({"part": "left_arm", "state": "broken", "hp_delta": -5})
    assert calls == [
        ("GET", "/robot/damage", None),
        ("GET", "/robot/damage/events?since=3&limit=10", None),
        ("POST", "/robot/damage/reset", {}),
        ("POST", "/robot/damage/inject",
         {"part": "left_arm", "state": "broken", "hp_delta": -5}),
    ]


def test_damage_inject_requires_part():
    resp = server._tools_call({"name": "robot_damage_inject", "arguments": {}})
    assert resp["isError"] is True and "part" in resp["content"][0]["text"]


def test_read_bench_and_particles_forward_their_query(monkeypatch):
    calls = _capture(monkeypatch)
    server.t_read_bench({"n": 7})
    server.t_read_bench({})
    server.t_scene_node_particles({"def": "SHEET", "sample": 20})
    server.t_scene_node_particles({"def": "SHEET"})
    assert calls == [
        ("GET", "/debug/read_bench?n=7", None),
        ("GET", "/debug/read_bench", None),
        ("GET", "/scene/node/SHEET/particles?sample=20", None),
        ("GET", "/scene/node/SHEET/particles", None),
    ]


def test_spawn_and_delete_forward_the_rebuild_opt_in(monkeypatch):
    calls = _capture(monkeypatch)
    server.t_scene_spawn({"vrml": "Solid {}", "def": "X", "physics": "rebuild"})
    server.t_scene_delete({"def": "X", "physics": "rebuild"})
    assert calls == [
        ("POST", "/scene/spawn", {"vrml": "Solid {}", "def": "X", "physics": "rebuild"}),
        ("POST", "/scene/delete", {"def": "X", "physics": "rebuild"}),
    ]
    for name in ("scene_spawn", "scene_delete"):
        schema = server.TOOLS[name][2]
        assert schema["properties"]["physics"]["enum"] == ["rebuild"]
        assert "rebuild_physics" in server.TOOLS[name][1]
        assert "until the world is reloaded" not in server.TOOLS[name][1]


def test_load_world_tracking_object_goes_to_world_load(monkeypatch):
    # /world/sync has no per-tracker toggle, so a `tracking` request is a load.
    calls = _capture(monkeypatch)
    server.t_load_world({"path": "w.omniworld",
                         "tracking": {"contacts": False, "joint_limits": True}})
    assert calls == [("POST", "/world/load",
                      {"path": "w.omniworld",
                       "tracking": {"contacts": False, "joint_limits": True}})]
    props = server.TOOLS["load_world"][2]["properties"]
    assert set(props["tracking"]["properties"]) == {"contacts", "joint_limits", "grips"}
    assert "run-headless" in server.TOOLS["load_world"][1]  # the light rule, stated


def test_load_world_documents_the_light_default(monkeypatch):
    """Light is the HARNESS default since 2026-09-02 (a load naming neither
    `light` nor `tracking` runs light). The tool must say so, name the way
    back, and keep sending a bare load bare so the harness applies -- and
    reports -- its own default rather than the MCP layer echoing one."""
    desc = server.TOOLS["load_world"][1]
    assert "DEFAULT SINCE 2026-09-02" in desc
    assert "default_applied" in desc
    assert "light=false" in desc and "OMNISIM_HARNESS_LIGHT=0" in desc
    assert "An explicit value ALWAYS wins" in desc
    props = server.TOOLS["load_world"][2]["properties"]
    assert "2026-09-02" in props["light"]["description"]
    assert "Pass false" in props["light"]["description"]
    assert "2026-09-02" in server.TOOLS["world_sync"][2]["properties"]["light"]["description"]
    assert "2026-09-02" in server.TOOLS["get_grips"][1]
    assert "TRACKER_NOT_RUNNING" in server.TOOLS["get_grips"][1]
    assert "2026-09-02" in server.TOOLS["sim_step"][1]
    # A bare call stays bare: no `light` is invented client-side.
    calls = _capture(monkeypatch)
    server.t_load_world({"path": "w.omniworld", "force_reload": True})
    assert calls == [("POST", "/world/load", {"path": "w.omniworld"})]
    calls.clear()
    server.t_load_world({"path": "w.omniworld", "light": False, "force_reload": True})
    assert calls == [("POST", "/world/load", {"path": "w.omniworld", "light": False})]


def test_get_contacts_describes_paired_and_tracking(monkeypatch):
    desc = server.TOOLS["get_contacts"][1]
    assert "paired" in desc and "tracking" in desc and "LIGHT MODE" in desc
    calls = _capture(monkeypatch)
    server.t_get_contacts({})
    server.t_get_contacts({"settle_steps": 2})
    assert calls == [("GET", "/sim/contacts", None),
                     ("GET", "/sim/contacts?settle_steps=2", None)]


def test_descriptions_carry_the_documented_pointers():
    assert "frame" in server.TOOLS["screenshot"][1]
    assert "recommended_max_steps_per_request" in server.TOOLS["sim_step"][1]
    # The harness speaks HTTP/1.1 keep-alive since 2026-09-01; the docstring
    # must not present HTTP/1.0 as the current state.
    assert "which\nit does today" not in server.__doc__
    assert "HTTP/1.1" in server.__doc__
