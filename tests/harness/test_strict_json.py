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

"""Strict-JSON contract of the harness HTTP layer.

The supervisor forwards whatever the engine reports; uninitialised
transforms can carry NaN, and Python's json module happily parses AND
re-emits bare ``NaN`` — which is invalid JSON (RFC 8259) and breaks
non-Python clients on `GET /scene/tree`. These tests pin down:

  1. `sanitize_nonfinite` — the recursive scrubber (NaN/Inf -> null),
  2. the end-to-end HTTP guarantee: a real `ThreadingHTTPServer` running
     the harness handler, backed by a stub supervisor that returns
     NaN-laden payloads, must emit response BYTES with no bare NaN /
     Infinity token, parseable by a strict JSON parser.

Also covers the supervised-load result plumbing (`load_state` reporting:
in_progress vs bind_failed) that the slow-cold-load fix introduced.

Run with:
    pytest tests/harness/test_strict_json.py
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

import omnisim_harness  # noqa: E402
from omnisim_harness import (  # noqa: E402
    HarnessState,
    IDEMPOTENT_SUPERVISOR_COMMANDS,
    make_handler,
    sanitize_nonfinite,
)

NAN = float("nan")
INF = float("inf")


# ---------------------------------------------------------------------------
# sanitize_nonfinite
# ---------------------------------------------------------------------------


def test_nan_becomes_none():
    assert sanitize_nonfinite(NAN) is None


def test_infinities_become_none():
    assert sanitize_nonfinite(INF) is None
    assert sanitize_nonfinite(-INF) is None


def test_finite_values_pass_through_unchanged():
    obj = {"f": 1.5, "i": 7, "s": "x", "b": True, "n": None, "z": 0.0}
    assert sanitize_nonfinite(obj) == obj


def test_nested_structures_are_scrubbed():
    obj = {
        "nodes": [
            {"position": [1.0, NAN, 3.0], "orientation": (0.0, 0.0, INF, -INF)},
            {"fields": {"deep": {"x": NAN}}},
        ],
    }
    out = sanitize_nonfinite(obj)
    assert out["nodes"][0]["position"] == [1.0, None, 3.0]
    assert out["nodes"][0]["orientation"] == [0.0, 0.0, None, None]
    assert out["nodes"][1]["fields"]["deep"]["x"] is None


def test_nonfinite_dict_keys_do_not_crash_strict_dump():
    # json.dumps stringifies float keys with repr -> "NaN" (and raises
    # under allow_nan=False); the sanitizer maps such keys to "null".
    out = sanitize_nonfinite({NAN: 1, 2.5: "ok"})
    dumped = json.dumps(out, allow_nan=False)
    assert "NaN" not in dumped


def test_sanitized_output_round_trips_under_allow_nan_false():
    obj = {"a": [NAN, INF, -INF, 1.25], "b": {"c": NAN}}
    dumped = json.dumps(sanitize_nonfinite(obj), allow_nan=False)  # must not raise
    parsed = json.loads(dumped, parse_constant=_reject_constant)
    assert parsed == {"a": [None, None, None, 1.25], "b": {"c": None}}


def _reject_constant(name: str):
    raise AssertionError(f"bare non-finite JSON token emitted: {name}")


# ---------------------------------------------------------------------------
# End-to-end: harness HTTP layer emits strict JSON even when the supervisor
# hands back NaN (the exact /scene/tree failure seen by non-Python clients).
# ---------------------------------------------------------------------------


class _StubState:
    """Minimal stand-in for HarnessState: only what the /scene/tree GET
    route touches (`supervisor_call`), plus `started_at` for /healthz."""

    started_at = time.time()

    def supervisor_call(self, cmd, args=None):
        assert cmd == "scene_tree"
        return {
            "nodes": [
                {
                    "type": "Robot",
                    "def": "HUSKY",
                    "position": [NAN, 0.0, INF],
                    "orientation": [0.0, 0.0, 1.0, NAN],
                },
            ],
        }


@pytest.fixture()
def live_server():
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(_StubState()))
    # poll_interval: shutdown() joins serve_forever()'s select() loop, so the
    # default 0.5 s poll cost ~0.5 s of teardown per test (measured 2026-09-02).
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
    )
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_address[1]}"
    finally:
        server.shutdown()
        server.server_close()


def test_scene_tree_bytes_contain_no_bare_nan(live_server):
    with urllib.request.urlopen(f"{live_server}/scene/tree", timeout=5) as resp:
        raw = resp.read()
    text = raw.decode("utf-8")
    assert "NaN" not in text
    assert "Infinity" not in text
    parsed = json.loads(text, parse_constant=_reject_constant)
    node = parsed["nodes"][0]
    assert node["position"] == [None, 0.0, None]
    assert node["orientation"] == [0.0, 0.0, 1.0, None]


# ---------------------------------------------------------------------------
# Supervised-load result plumbing (slow-cold-load fix)
# ---------------------------------------------------------------------------


def _bare_state() -> HarnessState:
    """A HarnessState without __init__ (no binary resolution, no engine):
    just the fields the load-result reporting paths read."""
    s = object.__new__(HarnessState)
    s.lock = threading.Lock()
    s.proc = None
    s.supervisor = None
    s.supervisor_connected_at = None
    s.current_world = None
    s.current_sibling = None
    s.last_load_started_at = None
    s.last_load_completed_at = None
    s.last_load_ok = None
    s.last_load_ms = None
    s.last_diagnostics = []
    s.last_exit_code = None
    s.started_at = time.time()
    s._load_generation = 0
    s._bind_state = None
    s._load_lock = threading.Lock()
    s.binary = Path("omnisim-bin")
    s.omnisim_home = Path(".")
    return s


def _bind(status: str, **extra) -> dict:
    bind = {
        "gen": 0,
        "status": status,
        "world": "w.wbt",
        "started_at": time.time() - 2.0,
        "patience_s": 1.0,
        "detail": "",
        "exit_code": None,
        "elapsed_s": None,
        "event": threading.Event(),
    }
    bind.update(extra)
    return bind


def test_load_result_waiting_reports_in_progress_and_keeps_engine():
    state = _bare_state()
    result = state._supervised_load_result(Path("w.wbt"), _bind("waiting"), 1.0)
    assert result["ok"] is True
    assert result["load_state"] == "in_progress"
    assert result["supervisor"].startswith("load_in_progress:")
    assert result["diagnostics"] == []
    # The in-progress report must NOT mark the load failed — the engine is
    # alive and the background waiter still owns the outcome.
    assert state.last_load_ok is None


def test_load_result_stalled_reports_bind_failed_with_diagnostic():
    state = _bare_state()
    diag = {
        "code": "SUPERVISOR_BIND_STALLED",
        "severity": "error",
        "message": "engine made no visible progress",
        "raw": "",
    }
    state.last_diagnostics = [diag]
    state.last_load_ok = False
    result = state._supervised_load_result(
        Path("w.wbt"), _bind("stalled", detail="engine made no visible progress"), 1.0
    )
    assert result["ok"] is False
    assert result["load_state"] == "bind_failed"
    assert result["supervisor"].startswith("unavailable:")
    assert result["diagnostics"] == [diag]


def test_sim_state_reports_in_progress_bind():
    state = _bare_state()
    state._bind_state = _bind("waiting")
    snapshot = state.sim_state()
    assert snapshot["load_state"] == "in_progress"
    assert snapshot["supervisor_bind"]["status"] == "waiting"
    # The fingerprint keys probe_existing_harness relies on must survive.
    assert "webots_home" in snapshot and "binary" in snapshot
    # /sim/state itself must be strictly serializable.
    json.dumps(sanitize_nonfinite(snapshot), allow_nan=False)


def test_mutating_supervisor_commands_are_not_retried():
    for cmd in ("step", "reset", "world_load", "damage_reset", "damage_inject"):
        assert cmd not in IDEMPOTENT_SUPERVISOR_COMMANDS
    for cmd in ("scene_tree", "screenshot", "robots_list", "events_drain"):
        assert cmd in IDEMPOTENT_SUPERVISOR_COMMANDS


def test_wait_clamp_allows_slow_platform_windows():
    # wait_s=90 used to be silently clamped to 60 — below the measured
    # 46–79 s WSL2 cold load of warehouse_husky. The ceiling must now
    # accommodate the caller's window up to the hard ceiling.
    assert omnisim_harness.MAX_LOAD_WAIT_S >= 300.0
    assert omnisim_harness.LOAD_BIND_HARD_CEILING_S >= 300.0
