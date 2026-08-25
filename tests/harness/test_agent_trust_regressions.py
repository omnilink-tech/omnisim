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

"""Five harness defects catalogued by the 2026-08-12 agent diagnostic round.

Three agent-built cells were run against OmniSim and every place the TOOL lied
to the agent was recorded. All five below were MEASURED; the frequencies are
out of three cells. Each test in this file was demonstrated RED against the
unfixed harness before the fix landed.

1. (2/3) `/sim/reset` freezes the scene and a later `/world/load` reports
   `ok` while the supervisor is not connected.
2. (2/3) `/world/screenshot` returns 200 + `image/png` + a ZERO-BYTE body.
3. (1/3) `/capabilities` served a STALE sidecar as this session's physics,
   on a harness with no world loaded.
4. (1/3) `/sim/reset` returned a VALIDATION error behind HTTP 503.
5. (2/3) `/robots` counted the harness's own injected supervisor.

The unifying rule these pin down: a harness answer must never be more
confident than the thing it measured. An empty body is not a picture, a
foreign sidecar is not this run's provenance, a caller's typo is not a server
outage, and an injected node is not part of the user's world.
"""

from __future__ import annotations

import base64
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
from omnisim_harness import (  # noqa: E402
    HarnessState,
    SupervisorRPCError,
    make_handler,
)

# A real, minimal (1x1) PNG. The falsifier cases need bytes that are actually
# a picture, so a "no image" verdict cannot be reached by rejecting everything.
ONE_PIXEL_PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhf"
    "DwAChwGA60e6kgAAAABJRU5ErkJggg=="
)


# ---------------------------------------------------------------------------
# Plumbing
# ---------------------------------------------------------------------------


class StubState:
    """Minimal stand-in for HarnessState for the HTTP-layer tests.

    `supervisor_call` is driven by a per-command table of either a dict (the
    reply) or an exception instance (raised), so a test can reproduce exactly
    what the supervisor did in the measured session.
    """

    def __init__(self, replies: dict | None = None, screenshot_writes: bytes | None = None):
        self.started_at = time.time()
        self.replies = replies or {}
        self.screenshot_writes = screenshot_writes
        self.calls: list = []
        self.viewport = None
        self.live = True
        self.live_detail = ""

    # -- the surface the handler touches ---------------------------------
    def supervisor_call(self, cmd, args=None):
        self.calls.append((cmd, args))
        if cmd == "screenshot" and self.screenshot_writes is not None:
            Path(args["path"]).write_bytes(self.screenshot_writes)
        reply = self.replies.get(cmd, {})
        if isinstance(reply, Exception):
            raise reply
        return reply

    def supervisor_live_check(self):
        return (self.live, self.live_detail)

    def sim_state(self):
        return {"sim_time_ms": 0.0}

    def note_render(self, digest, sim_ms):
        return {"identical_to_previous": False}

    def note_png_bytes(self, data):
        pass

    def note_png_path(self, path):
        pass


@pytest.fixture()
def serve():
    """Start a harness HTTP server around a stub state; yields a `post`/`get`
    pair that return (status, headers, body-bytes) and never raise on 4xx/5xx.
    """
    servers: list = []

    def _start(state):
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
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
            return resp.status, dict(resp.headers), resp.read()
    except urllib.error.HTTPError as exc:
        return exc.code, dict(exc.headers), exc.read()


def bare_state(tmp_path: Path) -> HarnessState:
    """A HarnessState without __init__: no binary resolution, no engine."""
    s = object.__new__(HarnessState)
    s.lock = threading.Lock()
    s.proc = None
    s.supervisor = None
    s.supervisor_connected_at = None
    s.supervisor_host = "127.0.0.1"
    s.supervisor_port = 6790
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
    s._world_log_offset = 0
    s._world_log_lock = threading.Lock()
    s._stdout_thread = None
    s._stderr_thread = None
    s.log_buffer = h.LogRingBuffer()
    s.log_path = tmp_path / "omnisim_log.txt"
    s.light_supervisor = False
    s.viewport = None
    s.step_samples = __import__("collections").deque(maxlen=32)
    s._runtime_mutation_warned = set()
    s.binary = Path("omnisim-bin")
    s.omnisim_home = tmp_path
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


class FakeClient:
    """A SupervisorClient stand-in: connected, and answering (or refusing)
    real RPCs independently of `ping`."""

    def __init__(self, alive: bool = True, connected: bool = True):
        self.alive = alive
        self._connected = connected
        self.closed = False
        self.calls: list = []

    def is_connected(self):
        return self._connected and not self.closed

    def call(self, cmd, args=None):
        self.calls.append(cmd)
        if cmd == "ping":
            return {}
        if not self.alive:
            raise SupervisorRPCError("supervisor RPC failed: supervisor closed the connection")
        return {"sim_time_ms": 0.0}

    def close(self):
        self.closed = True

    def set_rpc_timeout(self, t):
        pass


class FakeProc:
    def __init__(self, ret=None):
        self.ret = ret
        self.pid = 4242

    def poll(self):
        return self.ret


# ===========================================================================
# DEFECT 1 -- /sim/reset freezes the scene; a later load reports ok with no
#             supervisor.
# ===========================================================================


def test_reset_discloses_that_it_re_pins_every_motor(serve):
    """MEASURED: fresh harness, single load, supervisor connected. /sim/reset
    reported "authored poses restored"; 1250 subsequent steps advanced 20.0 s
    of sim time at normal per-step cost; all 10 robots read 0.00 m net AND
    0.00 m path. A second cell read its wheel joints frozen at 980.14 rad.

    ROOT CAUSE (engine, read from source): the reset walks the scene via
    `root()->reset("__init__")` and `OmMotor::reset` re-pins
    `mTargetPosition` to the joint's CURRENT position and clears
    `mUserControl` -- so a wheel running in velocity mode (`setPosition(inf)`)
    becomes a POSITION HOLD at wherever it was, which is exactly the 980.14
    rad that cell read back. The supervisor path passes
    `restartControllers=false`, so no controller is restarted to re-issue the
    command. The harness cannot fix that from Python; what it MUST NOT do is
    report a bare success and let the agent conclude the world is broken.
    """
    state = StubState(replies={"reset": {"sim_time_ms": 0.0, "restored": "__init__"}})
    url = _start_reset_server(serve, state)
    status, _hdrs, body = _request(f"{url}/sim/reset", "POST", {})
    assert status == 200
    payload = json.loads(body)
    act = payload.get("actuation")
    assert act, "/sim/reset must disclose what the reset did to actuation"
    assert act["motors_retargeted"] is True
    assert act["controllers_restarted"] is False
    # The consequence must be reachable by a caller that only reads `warning`.
    assert "warning" in payload
    blob = json.dumps(payload)
    assert "OmMotor::reset" in blob          # names the mechanism
    assert "980.14" in blob                  # names the measured symptom
    assert act["workarounds"], "a disclosed defect without a workaround is a dead end"


def test_reset_keeps_a_pre_existing_supervisor_warning(serve):
    """The falsifier for the disclosure: it must ADD to the supervisor's own
    warning (e.g. 'no snapshot named X'), never overwrite it."""
    state = StubState(replies={"reset": {"sim_time_ms": 0.0, "restored": None,
                                         "warning": "no snapshot named 'x' in this world"}})
    url = _start_reset_server(serve, state)
    _status, _hdrs, body = _request(f"{url}/sim/reset", "POST", {})
    payload = json.loads(body)
    assert "no snapshot named 'x'" in payload["warning"]
    assert "motor" in payload["warning"].lower()


def test_reset_does_not_report_ok_when_the_supervisor_is_gone(serve):
    """The minimum honest fix. If the reset RPC returned but the supervisor is
    no longer there, the caller must not read a 200."""
    state = StubState(replies={"reset": {"sim_time_ms": 0.0}})
    state.live = False
    state.live_detail = "supervisor closed the connection"
    url = _start_reset_server(serve, state)
    status, _hdrs, body = _request(f"{url}/sim/reset", "POST", {})
    assert status >= 500, "a reset with no supervisor behind it is not a 200"
    payload = json.loads(body)
    assert payload.get("code") == "SUPERVISOR_LOST"
    assert payload.get("supervisor_connected") is False


def _start_reset_server(serve, state):
    return serve(state)


def test_load_result_is_not_complete_without_a_live_supervisor(tmp_path):
    """MEASURED: `supervisor_connected: false` while `load_state: "complete"`.
    A supervised load whose supervisor is not there is not a complete load,
    whatever the bind waiter last recorded."""
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.supervisor = None                       # the waiter's client died
    result = state._supervised_load_result(Path("w.wbt"), _bind("connected"), 1.0)
    assert result["supervisor_connected"] is False
    assert result["load_state"] != "complete"
    assert result["ok"] is False


def test_load_result_stays_complete_with_a_live_supervisor(tmp_path):
    """The falsifier: the check must not fail a load that genuinely worked."""
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.supervisor = FakeClient(alive=True)
    result = state._supervised_load_result(Path("w.wbt"), _bind("connected"), 1.0)
    assert result["supervisor_connected"] is True
    assert result["load_state"] == "complete"
    assert result["ok"] is True


def test_sim_state_does_not_call_a_supervisorless_session_complete(tmp_path):
    """/sim/state is what an agent polls to decide the session is usable. It
    must not answer `complete` for a supervised world whose supervisor is
    gone -- that pair of fields was the measured contradiction."""
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.current_world = "w.wbt"
    state.current_sibling = Path(".omnisim_harness_w.wbt")   # supervised load
    state.last_load_ok = True
    state.supervisor = None
    snap = state.sim_state()
    assert snap["supervisor_connected"] is False
    assert snap["load_state"] != "complete"
    assert snap["load_state"] == "supervisor_lost"


def test_sim_state_still_reports_complete_for_a_bare_load(tmp_path):
    """Falsifier: a `with_supervisor=false` load has no supervisor by design
    and must keep reading `complete`."""
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.current_world = "w.wbt"
    state.current_sibling = None                              # bare load
    state.last_load_ok = True
    state.supervisor = None
    assert state.sim_state()["load_state"] == "complete"


def test_hot_reload_refuses_a_supervisor_that_cannot_answer_a_real_rpc(tmp_path, monkeypatch):
    """"The supervisor never rebinds on the second load."

    The old supervisor keeps LISTENING until the engine actually swaps worlds,
    and it answers pings while it dies -- so the bind probe (two pings) can
    adopt a corpse, and the hot reload then reports `load_state: complete` /
    `supervisor: connected` on a session that has no working supervisor at
    all. Adoption must be confirmed with a REAL RPC, and a failure must fall
    back to the cold launch rather than be reported as a success.
    """
    world = tmp_path / "w.wbt"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo {\n}\n", encoding="utf-8")
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.supervisor = FakeClient(alive=True)
    monkeypatch.setattr(h, "_tcp_port_in_use", lambda *a, **k: False)
    dying = FakeClient(alive=False)            # ping ok, real RPC fails
    monkeypatch.setattr(state, "_try_connect_supervisor",
                        lambda *a, **k: dying)
    out = state._try_hot_reload(world, wait_s=0.2)
    assert out is None, "adopting a corpse must fall back to a cold launch"
    assert dying.closed, "the rejected client must not be leaked"


def test_hot_reload_accepts_a_supervisor_that_answers(tmp_path, monkeypatch):
    """Falsifier: a healthy new supervisor must still hot-reload."""
    world = tmp_path / "w.wbt"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo {\n}\n", encoding="utf-8")
    state = bare_state(tmp_path)
    state.proc = FakeProc()
    state.supervisor = FakeClient(alive=True)
    monkeypatch.setattr(h, "_tcp_port_in_use", lambda *a, **k: False)
    fresh = FakeClient(alive=True)
    monkeypatch.setattr(state, "_try_connect_supervisor", lambda *a, **k: fresh)
    out = state._try_hot_reload(world, wait_s=0.2)
    assert out is not None
    assert out["load_state"] == "complete"
    assert out["supervisor_connected"] is True
    assert "sim_state" in fresh.calls, "adoption must be proven by a real RPC"


# ===========================================================================
# DEFECT 2 -- /world/screenshot: 200 + image/png + zero bytes.
# ===========================================================================


def test_screenshot_never_returns_200_with_an_empty_body(serve):
    """MEASURED (2/3 cells): HTTP 200, `Content-Type: image/png`, and a
    zero-byte body. Not a rendering-disabled case -- the same scene rendered
    603 KB through the capture service moments later.

    The cost of this one is out of all proportion to its size: agents worked
    around it by reaching for the capture service, which renders 1920x1080
    through WREN and took the owner's laptop GPU to 86 C, and the
    `.capture_*` sibling world it leaves behind got a correct 10-robot run
    graded FAIL on the wrong file.
    """
    state = StubState(replies={"screenshot": {"ok": True}})   # writes nothing
    url = serve(state)
    status, headers, body = _request(f"{url}/world/screenshot", "POST", {})
    assert not (status == 200 and body == b""), (
        "200 + image/png + 0 bytes is the exact measured lie")
    assert status >= 400
    assert headers.get("Content-Type") == "application/json"
    payload = json.loads(body)
    assert payload["code"] == "SCREENSHOT_EMPTY"
    assert payload["error"]


def test_screenshot_rejects_bytes_that_are_not_a_png(serve):
    """A non-empty body that is not a PNG is the same lie with more bytes."""
    state = StubState(replies={"screenshot": {"ok": True}},
                      screenshot_writes=b"not a png at all")
    url = serve(state)
    status, _headers, body = _request(f"{url}/world/screenshot", "POST", {})
    assert status >= 400
    assert json.loads(body)["code"] == "SCREENSHOT_NOT_PNG"


def test_screenshot_path_form_reports_an_empty_file(serve, tmp_path):
    """The server-side-path form took the same 200: it reported the path it
    was given without ever checking that a picture landed there."""
    target = tmp_path / "shot.png"
    state = StubState(replies={"screenshot": {"ok": True}}, screenshot_writes=b"")
    url = serve(state)
    status, _headers, body = _request(
        f"{url}/world/screenshot", "POST", {"path": str(target)})
    assert status >= 400
    assert json.loads(body)["code"] == "SCREENSHOT_EMPTY"


def test_the_new_refusal_codes_are_published_on_capabilities():
    """A code an agent cannot discover is a code it cannot branch on. Both
    screenshot refusals are chosen inside a conditional, which the source
    scanner cannot see -- so they have to be declared."""
    published = h.known_request_error_codes()
    for code in ("SCREENSHOT_EMPTY", "SCREENSHOT_NOT_PNG", "ARGUMENT_INVALID",
                 "SUPERVISOR_UNAVAILABLE", "SUPERVISOR_INTERNAL_ERROR",
                 "SUPERVISOR_LOST"):
        assert code in published, code
    # ... and must stay disjoint from the LOAD diagnostic codes.
    assert not set(published) & set(h.known_diagnostic_codes())


def test_a_real_png_is_still_served(serve):
    """The falsifier. A working screenshot must be unaffected -- otherwise
    the fix is just a different lie."""
    state = StubState(replies={"screenshot": {"ok": True}},
                      screenshot_writes=ONE_PIXEL_PNG)
    url = serve(state)
    status, headers, body = _request(f"{url}/world/screenshot", "POST", {})
    assert status == 200
    assert headers.get("Content-Type") == "image/png"
    assert body == ONE_PIXEL_PNG


def test_a_real_png_is_still_served_to_a_path(serve, tmp_path):
    target = tmp_path / "shot.png"
    state = StubState(replies={"screenshot": {"ok": True}},
                      screenshot_writes=ONE_PIXEL_PNG)
    url = serve(state)
    status, _headers, body = _request(
        f"{url}/world/screenshot", "POST", {"path": str(target)})
    assert status == 200
    payload = json.loads(body)
    assert payload["path"] == str(target)
    assert payload["bytes"] == len(ONE_PIXEL_PNG)


# ===========================================================================
# DEFECT 3 -- /capabilities served a STALE sidecar as this session's physics.
# ===========================================================================


def _write_sidecar(state: HarnessState, age_s: float = 0.0) -> Path:
    state.log_path.write_text("INFO: nothing\n", encoding="utf-8")
    sidecar = Path(str(state.log_path) + ".newton.json")
    sidecar.write_text(json.dumps({
        "backend": "newton", "solver": "MuJoCo (cpu/mj_step)",
        "degraded": False, "finalised": True}), encoding="utf-8")
    if age_s:
        old = time.time() - age_s
        import os
        os.utime(sidecar, (old, old))
    return sidecar


def test_capabilities_does_not_attribute_a_foreign_sidecar_to_a_loadless_session(tmp_path):
    """MEASURED: a full attestation -- backend newton, solver named, a 50-body
    census -- with `source: "sidecar"` at `sidecar_age_s: 85.6`, on a harness
    with NO WORLD LOADED. The documented contract is that the sidecar's
    presence means "Newton drove THIS run"; a run that never happened cannot
    be attested, and any provenance claim built on it is void.
    """
    state = bare_state(tmp_path)
    _write_sidecar(state, age_s=85.6)
    caps = state.capabilities()
    physics = caps["physics"]
    assert physics["source"] != "sidecar"
    assert physics["source"] in ("sidecar_stale", "sidecar_absent")
    assert physics["backend"] == "unverified"
    assert physics.get("stale") is True


def test_capabilities_does_not_serve_a_foreign_body_census(tmp_path):
    """The 50-body census came from the same foreign log and is the number an
    agent quotes as "the world is being simulated"."""
    state = bare_state(tmp_path)
    state.log_path.write_text(
        "INFO: [OmNewtonBackend] registered 50 dynamic + 3 static Newton bodies\n",
        encoding="utf-8")
    caps = state.capabilities()
    bodies = caps["physics"]["bodies"]
    assert bodies["dynamic_bodies_registered"] is None
    assert bodies["source"] == "no_world_loaded"


def test_capabilities_reports_a_sidecar_written_for_the_current_load(tmp_path):
    """The falsifier: a sidecar that postdates this session's load is exactly
    the case the field exists for and must still read `sidecar`."""
    state = bare_state(tmp_path)
    state.last_load_started_at = time.time() - 5.0
    _write_sidecar(state)
    physics = state.capabilities()["physics"]
    assert physics["source"] == "sidecar"
    assert physics["backend"] == "newton"


def test_capabilities_still_flags_a_sidecar_older_than_the_current_load(tmp_path):
    state = bare_state(tmp_path)
    state.last_load_started_at = time.time()
    _write_sidecar(state, age_s=300.0)
    physics = state.capabilities()["physics"]
    assert physics["source"] == "sidecar_stale"
    assert physics["backend"] == "unverified"


# ===========================================================================
# DEFECT 4 -- a validation error behind HTTP 503.
# ===========================================================================


def test_reset_argument_rejection_is_a_4xx_not_a_503(serve):
    """MEASURED: `{"error": "'restore' must be a snapshot name or null"}`
    behind a 503, which urllib/requests raise on -- so the caller saw a server
    failure instead of its own bad argument, and retried."""
    state = StubState(replies={
        "reset": SupervisorRPCError("'restore' must be a snapshot name or null")})
    url = serve(state)
    status, _headers, body = _request(f"{url}/sim/reset", "POST", {"restore": 7})
    assert 400 <= status < 500, "a caller's bad argument is not a server outage"
    payload = json.loads(body)
    assert payload["code"] == "ARGUMENT_INVALID"


def test_a_genuinely_unavailable_supervisor_is_still_a_503(serve):
    """The falsifier. Reclassifying transport failures as 4xx would be the
    same defect pointed the other way."""
    state = StubState(replies={
        "reset": SupervisorRPCError(
            "supervisor not connected (load a world with with_supervisor=true)")})
    url = serve(state)
    status, _headers, _body = _request(f"{url}/sim/reset", "POST", {})
    assert status == 503


def test_supervisor_internal_errors_stay_5xx(serve):
    state = StubState(replies={
        "reset": SupervisorRPCError("internal: TypeError('boom')")})
    url = serve(state)
    status, _headers, _body = _request(f"{url}/sim/reset", "POST", {})
    assert status >= 500


@pytest.mark.parametrize("message,expected", [
    ("'restore' must be a snapshot name or null", 400),
    ("'name' must be a string", 400),
    ("robot_joints requires a 'def' string", 400),
    ("no node with DEF 'NOPE'", 404),
    ("supervisor RPC failed: supervisor closed the connection", 503),
    ("supervisor is not connected", 503),
    ("could not connect to supervisor at 127.0.0.1:6790 within 2.0s", 503),
    ("internal: KeyError('x')", 500),
])
def test_supervisor_error_status_separates_caller_from_transport(message, expected):
    status, _code = h.classify_supervisor_error(message)
    assert status == expected, message


def test_every_endpoint_that_forwards_caller_arguments_uses_the_classifier():
    """The audit the round asked for: an endpoint that hands the caller's own
    arguments to the supervisor must map a rejection to a 4xx. `_supervisor_call`
    is the unconditional-503 helper, so it must not appear on those routes."""
    import inspect
    src = inspect.getsource(make_handler)
    for cmd in ('"reset"', '"sim_snapshot"', '"sim_restore"', '"robot_joints"',
                '"robot_devices"', '"damage_inject"'):
        # each of these is reached through the coded helper, never the bare one
        idx = src.find(f"_supervisor_call({cmd}")
        assert idx == -1, f"{cmd} still routed through the unconditional-503 helper"


# ===========================================================================
# DEFECT 5 -- /robots counts the harness's own injected supervisor.
# ===========================================================================


def _roster(n: int = 10) -> dict:
    robots = [{"def": f"HUSKY_{i}", "name": f"husky_{i}", "model": "Husky",
               "controller": "drive_forward", "type": "Robot",
               "position": [0.0, 0.0, 0.0], "orientation": [0, 0, 1, 0],
               "num_joints": 4} for i in range(n)]
    robots.append({"def": "#939", "name": "harness_supervisor", "model": "",
                   "controller": "harness_supervisor", "type": "Robot",
                   "position": [0.0, 0.0, 0.0], "orientation": [0, 0, 1, 0],
                   "num_joints": 0})
    return {"robots": robots}


def test_robots_does_not_count_the_injected_supervisor(serve):
    """MEASURED (2/3 cells): `'#939' | 'harness_supervisor'` alongside the
    real robots, so an agent asserting "exactly 10 robots" read 11. The node
    is not in the user's .wbt -- the harness put it there."""
    state = StubState(replies={"robots_list": _roster(10)})
    url = serve(state)
    status, _headers, body = _request(f"{url}/robots")
    assert status == 200
    payload = json.loads(body)
    names = [r["name"] for r in payload["robots"]]
    assert "harness_supervisor" not in names
    assert len(payload["robots"]) == 10
    # Excluding silently would be its own lie: say what was removed.
    assert payload["harness_injected"] == ["harness_supervisor"]


def test_robots_can_opt_the_injected_supervisor_back_in(serve):
    state = StubState(replies={"robots_list": _roster(10)})
    url = serve(state)
    _status, _headers, body = _request(f"{url}/robots?include_harness=1")
    payload = json.loads(body)
    assert len(payload["robots"]) == 11
    injected = [r for r in payload["robots"] if r["name"] == "harness_supervisor"]
    assert injected and injected[0]["harness_injected"] is True


def test_scene_tree_marks_the_injected_supervisor(serve):
    """The same problem one endpoint over. /scene/tree is a scene dump, so the
    node must stay in it -- but it must be labelled, because it is not in the
    file the agent is authoring."""
    state = StubState(replies={"scene_tree": {"nodes": [
        {"type": "Robot", "def": "HUSKY_0", "name": "husky_0"},
        {"type": "Robot", "def": "#939", "name": "harness_supervisor"},
    ]}})
    url = serve(state)
    _status, _headers, body = _request(f"{url}/scene/tree")
    payload = json.loads(body)
    marked = [n for n in payload["nodes"] if n.get("harness_injected")]
    assert [n["name"] for n in marked] == ["harness_supervisor"]
    assert payload["harness_injected"] == ["harness_supervisor"]
    assert not payload["nodes"][0].get("harness_injected")


def test_the_injected_name_cannot_drift_from_the_stanza():
    """The filter is only correct while it matches the name actually injected;
    both stanzas must be built from the same constant."""
    assert h.HARNESS_SUPERVISOR_NAME == "harness_supervisor"
    for stanza in (h.SUPERVISOR_INJECT_STANZA, h.SUPERVISOR_INJECT_STANZA_LIGHT):
        assert f'name "{h.HARNESS_SUPERVISOR_NAME}"' in stanza
        assert f'controller "{h.HARNESS_SUPERVISOR_NAME}"' in stanza
