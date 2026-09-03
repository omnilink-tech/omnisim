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

"""Pins on the tracking-mode DEFAULT of ``POST /world/load`` (light since 2026-09-02).

The injected supervisor's per-step contact / joint-limit / grip trackers made a
default load SLOWER than the run-headless the harness replaces (fleet arena:
12.1 s vs 4.1 s to load, /sim/step 573-606 ms vs 6-35 ms). The default flipped
to light on 2026-09-02 under a dual contract, and these tests pin every half
of it:

* a request naming neither ``light`` nor ``tracking`` runs light and SAYS so
  (``tracking.default_applied: true`` + one sentence naming the way back);
* an explicit ``light`` (either value) or any ``tracking`` object always wins;
* ``OMNISIM_HARNESS_LIGHT`` is value-parsed: unset or ``1`` -> light, ``0`` ->
  full, both directions pinned at the resolver, the state and the HTTP layer;
* ``GET /capabilities`` -> ``limits.tracking_default`` is built by the SAME
  function as the load response's block, so the two cannot drift;
* the first tracker-fed read in a session whose tracker is not running emits
  ONE ``world.warning`` per load that names whether light was the default.
"""

from __future__ import annotations

import inspect
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

import omnisim_harness as h  # noqa: E402
from omnisim_harness import HarnessState, make_handler  # noqa: E402

SOLID = """#OMNISIM R2025a utf8
WorldInfo {
}
DEF BOX Solid {
  translation 0 0 1
  children [
    Shape {
      geometry Box {
        size 0.4 0.4 0.4
      }
    }
  ]
}
"""


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------


class _LoadStub:
    """Just enough state for the /world/load and /sim/grips routes."""

    started_at = time.time()

    def __init__(self, default_light: bool = True, grips_enabled: bool = False):
        self.default_light = default_light
        self.default_light_source = "built-in"
        self.light_supervisor = default_light
        self.tracking_supervisor = None
        self.light_default_applied = False
        self.loads: list[dict] = []
        self.warned: list[str] = []
        self.grips_enabled = grips_enabled

    def load_world(self, world_path, wait_s, with_supervisor, light, default_applied=False):
        self.loads.append({"path": world_path, "light": light,
                           "default_applied": default_applied,
                           "tracking": self.tracking_supervisor})
        return {"ok": True, "world": world_path}

    def supervisor_call(self, cmd, args=None):
        assert cmd == "sim_grips"
        if self.grips_enabled:
            return {"grips": [], "tracking": {"enabled": True}}
        return {"grips": [], "tracking": {"enabled": False, "reason": "--light"}}

    def light_read_warning(self, surface, detail=None):
        self.warned.append(surface)


@pytest.fixture()
def serve():
    servers: list = []

    def _start(state):
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        # poll_interval: shutdown() joins serve_forever()'s select() loop, so the
        # default 0.5 s poll cost ~0.5 s of teardown per test (measured 2026-09-02).
        thread = threading.Thread(
            target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
        )
        thread.start()
        servers.append((server, thread))
        return f"http://127.0.0.1:{server.server_address[1]}"

    yield _start
    for server, _thread in servers:
        server.shutdown()
        server.server_close()


def _request(url: str, method: str = "GET", body: dict | None = None):
    data = json.dumps(body or {}).encode("utf-8") if method == "POST" else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def _real_state(tmp_path: Path, monkeypatch) -> HarnessState:
    monkeypatch.setattr(h, "resolve_omnisim_binary", lambda home: tmp_path / "omnisim-bin")
    return HarnessState(tmp_path)


# ---------------------------------------------------------------------------
# the HTTP contract: neither named -> default; explicit always wins
# ---------------------------------------------------------------------------


def test_load_without_light_or_tracking_runs_light_by_default(serve):
    state = _LoadStub()
    status, body = _request(serve(state) + "/world/load", "POST", {"path": "w.wbt"})
    assert status == 200 and body["ok"] is True
    assert state.loads == [{"path": "w.wbt", "light": True,
                            "default_applied": True, "tracking": None}]
    assert state.light_supervisor is True


def test_explicit_light_false_keeps_full_tracking(serve):
    state = _LoadStub()
    _request(serve(state) + "/world/load", "POST", {"path": "w.wbt", "light": False})
    assert state.loads[0]["light"] is False
    assert state.loads[0]["default_applied"] is False
    assert state.light_supervisor is False


def test_tracking_object_without_light_is_partial_not_default(serve):
    state = _LoadStub()
    _request(serve(state) + "/world/load", "POST",
             {"path": "w.wbt", "tracking": {"contacts": False}})
    assert state.loads[0]["light"] is False
    assert state.loads[0]["default_applied"] is False
    assert state.loads[0]["tracking"] == {"contacts": False}
    assert state.tracking_supervisor == {"contacts": False}


def test_explicit_light_true_wins_over_a_tracking_object(serve):
    state = _LoadStub()
    _request(serve(state) + "/world/load", "POST",
             {"path": "w.wbt", "light": True, "tracking": {"grips": True}})
    assert state.loads[0]["light"] is True
    assert state.loads[0]["default_applied"] is False


def test_the_default_is_not_sticky_across_requests(serve):
    """An earlier explicit `light: false` must not leak into a later request
    that names nothing -- the default is the process default, not the last
    value seen (that stickiness was how a `tracking` object used to be
    silently overridden by a previous request's light=true)."""
    state = _LoadStub()
    base = serve(state)
    _request(base + "/world/load", "POST", {"path": "w.wbt", "light": False})
    _request(base + "/world/load", "POST", {"path": "w.wbt"})
    assert [l["light"] for l in state.loads] == [False, True]
    assert [l["default_applied"] for l in state.loads] == [False, True]


def test_a_full_process_default_is_applied_and_reported_as_the_default(serve):
    state = _LoadStub(default_light=False)
    _request(serve(state) + "/world/load", "POST", {"path": "w.wbt"})
    assert state.loads[0]["light"] is False
    assert state.loads[0]["default_applied"] is True


# ---------------------------------------------------------------------------
# OMNISIM_HARNESS_LIGHT, both directions, at every layer
# ---------------------------------------------------------------------------


def test_resolver_unset_and_one_mean_light(monkeypatch):
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    assert h.resolve_light_default() == (True, "built-in")
    for raw in ("1", "true", "yes", "on"):
        monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", raw)
        assert h.resolve_light_default() == (True, f"OMNISIM_HARNESS_LIGHT={raw}"), raw


def test_resolver_zero_means_full(monkeypatch):
    # (An EMPTY value is deliberately not pinned: Windows deletes a variable
    # set to "", so `OMNISIM_HARNESS_LIGHT=` reads as unset there and as
    # falsey on POSIX -- use 0/1.)
    for raw in ("0", "false", "No", " 0 "):
        monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", raw)
        default, source = h.resolve_light_default()
        assert default is False, raw
        assert source.startswith("OMNISIM_HARNESS_LIGHT=")


def test_harness_state_default_is_light_when_env_unset(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    state = _real_state(tmp_path, monkeypatch)
    assert state.default_light is True
    assert state.default_light_source == "built-in"
    assert state.light_supervisor is True
    assert state.light_default_applied is False


def test_harness_state_env_zero_restores_the_full_default(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", "0")
    state = _real_state(tmp_path, monkeypatch)
    assert state.default_light is False
    assert state.light_supervisor is False
    monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", "1")
    assert _real_state(tmp_path, monkeypatch).default_light is True


# ---------------------------------------------------------------------------
# the response block and /capabilities are ONE description
# ---------------------------------------------------------------------------


def test_tracking_block_names_the_default_and_the_way_back(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    state = _real_state(tmp_path, monkeypatch)
    block = state.tracking_block(True, default_applied=True)
    assert block["mode"] == "light" and block["light"] is True
    assert block["default_applied"] is True
    assert block["default"] == h.tracking_default_block(True, "built-in")
    assert block["default"]["since"] == "2026-09-02"
    # One sentence naming how to get the trackers back, with every lever.
    assert '{"light": false}' in block["default_note"]
    assert '"tracking"' in block["default_note"]
    assert "OMNISIM_HARNESS_LIGHT=0" in block["default_note"]

    explicit = state.tracking_block(False, default_applied=False)
    assert explicit["mode"] == "full" and explicit["default_applied"] is False
    assert "default_note" not in explicit and "default" not in explicit
    # Full is no longer "the backward-compatible default" and must not say so.
    assert "backward-compatible" not in explicit["hint"]
    assert "2026-09-02" in explicit["hint"]

    state.tracking_supervisor = {"contacts": False}
    partial = state.tracking_block(False, default_applied=False)
    assert partial["mode"] == "partial"
    assert partial["disabled_flags"] == ["--no-contacts"]


def test_capabilities_limits_carry_the_same_default_block(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    state = _real_state(tmp_path, monkeypatch)
    caps = state.capabilities()
    assert caps["limits"]["tracking_default"] == h.tracking_default_block(True, "built-in")
    assert caps["limits"]["tracking_default"]["mode"] == "light"
    assert caps["supervisor"]["light_default_applied"] is False
    assert "OMNISIM_HARNESS_LIGHT=0" in caps["limits"]["tracking_default"]["revert"]
    assert "explicit_wins" in caps["limits"]["tracking_default"]

    monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", "0")
    full = _real_state(tmp_path, monkeypatch).capabilities()["limits"]["tracking_default"]
    assert full["mode"] == "full" and full["light"] is False
    assert full["source"] == "OMNISIM_HARNESS_LIGHT=0"


def test_capabilities_light_gap_entry_names_the_default(tmp_path, monkeypatch):
    src = inspect.getsource(HarnessState.capabilities)
    entry = src[src.index("if light:"):]
    assert "2026-09-02" in entry
    # The workaround quotes the re-measured 17-47x, not the pre-3b952b61d
    # ~790x, and names the per-tracker route back as well as light=false.
    workaround = entry[entry.index('"workaround"'):entry.index("return {")]
    assert "17-47x" in workaround and "~790x" not in workaround
    assert "`tracking` object" in workaround
    assert "limits.tracking_default" in workaround


# ---------------------------------------------------------------------------
# /world/sync reaches a first load through its own path (the MCP tool does)
# ---------------------------------------------------------------------------


def _sync_state(tmp_path: Path, monkeypatch) -> tuple[HarnessState, Path, list]:
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    state = _real_state(tmp_path, monkeypatch)
    world = tmp_path / "initial.wbt"
    world.write_text(SOLID, encoding="utf-8")
    loads: list = []

    def fake_load(path, wait_s, with_supervisor, light, source_text=None):
        loads.append((path, with_supervisor, light))
        state.current_world = str(path)
        return {"ok": True, "load_state": "complete", "load_ms": 5, "diagnostics": []}

    monkeypatch.setattr(state, "_load_world_locked", fake_load)
    return state, world, loads


def test_sync_first_load_applies_the_default_and_says_so(tmp_path, monkeypatch):
    state, world, loads = _sync_state(tmp_path, monkeypatch)
    result = state.sync_world(str(world), 3)
    assert result["mode"] == "full_reload"
    assert loads == [(world.resolve(), True, True)]
    assert result["tracking"]["mode"] == "light"
    assert result["tracking"]["default_applied"] is True
    assert "default_note" in result["tracking"]
    assert state.light_default_applied is True


def test_sync_explicit_light_false_is_not_the_default(tmp_path, monkeypatch):
    state, world, loads = _sync_state(tmp_path, monkeypatch)
    result = state.sync_world(str(world), 3, light=False)
    assert loads == [(world.resolve(), True, False)]
    assert result["tracking"]["mode"] == "full"
    assert result["tracking"]["default_applied"] is False
    assert state.light_default_applied is False


# ---------------------------------------------------------------------------
# the first tracker-fed read in a light session is loud, once per load
# ---------------------------------------------------------------------------


def test_light_read_warning_fires_once_per_load_and_names_the_default(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    state = _real_state(tmp_path, monkeypatch)
    state.light_supervisor = True
    state.light_default_applied = True
    for _ in range(3):
        state.light_read_warning("GET /sim/grips")
    events = state.log_buffer.since(0, 100)
    assert len(events) == 1
    evt = events[0]
    assert evt["type"] == "world.warning"
    assert evt["code"] == h.LIGHT_MODE_READ_CODE == "TRACKER_NOT_RUNNING"
    assert "GET /sim/grips" in evt["message"]
    assert "BY DEFAULT" in evt["message"]
    assert "2026-09-02" in evt["message"]
    assert '{"light": false}' in evt["message"]
    assert "/sim/contacts is unaffected" in evt["message"]

    # A reload re-arms it (the load paths reset the set), and a REQUESTED
    # light mode is worded as such, not as the default.
    state._light_read_warned = set()
    state.light_default_applied = False
    state.light_read_warning("GET /sim/grips")
    events = state.log_buffer.since(0, 100)
    assert len(events) == 2
    assert "BY DEFAULT" not in events[1]["message"]
    assert '{"light": true}' in events[1]["message"]


def test_light_read_warning_names_a_per_tracker_toggle(tmp_path, monkeypatch):
    state = _real_state(tmp_path, monkeypatch)
    state.light_supervisor = False
    state.light_default_applied = False
    state.tracking_supervisor = {"grips": False}
    state.light_read_warning("GET /sim/grips")
    (evt,) = state.log_buffer.since(0, 100)
    assert '"grips": false' in evt["message"]


def test_sim_grips_route_warns_only_when_the_tracker_is_off(serve):
    state = _LoadStub(grips_enabled=False)
    base = serve(state)
    status, body = _request(base + "/sim/grips")
    assert status == 200 and body["tracking"]["enabled"] is False
    _request(base + "/sim/grips")
    assert state.warned == ["GET /sim/grips", "GET /sim/grips"]  # dedup is the state's job

    live = _LoadStub(grips_enabled=True)
    status, body = _request(serve(live) + "/sim/grips")
    assert status == 200 and live.warned == []


def test_load_paths_rearm_the_read_warning():
    src = inspect.getsource(HarnessState._load_world_locked) + inspect.getsource(
        HarnessState._try_hot_reload)
    assert src.count("self._light_read_warned = set()") == 2


# ---------------------------------------------------------------------------
# the surfaces an agent reads before it ever sends a request
# ---------------------------------------------------------------------------


def test_routes_and_startup_banner_name_the_default():
    routes = {(r["method"], r["path"]): r for r in h.ROUTES}
    summary = routes[("POST", "/world/load")]["summary"]
    assert "LIGHT by default" in summary and "2026-09-02" in summary
    assert "OMNISIM_HARNESS_LIGHT=0" in summary
    main_src = inspect.getsource(h.main)
    assert "tracking default: LIGHT" in main_src
    assert "tracking default: FULL" in main_src
    assert "LIGHT IS THE DEFAULT SINCE 2026-09-02" in h.__doc__
