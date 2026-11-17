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

"""The pause / break tools and the OmniLink command surface.

These run against a REAL socket -- a throwaway ``http.server`` standing in for
a robot's bridge and for the harness -- rather than a monkeypatched
``_json_call``, because the things most worth pinning here live in the
transport: which path was called, what the body actually carried, and whether
a non-200 envelope survived the trip intact. No engine, no harness, no bridge
controller is started; the whole file is engine-free and takes well under a
second.

What it exists to prevent, in order of how expensive each one was to learn:

* An ``id`` riding along in the tool arguments. The bridge's ``/tool`` pops
  ``tool`` and dispatches EVERYTHING ELSE in the body as the argument map, so
  a transport field arrives at the safety gate as ``unknown_arg``. The Mavic
  once refused a perfectly good takeoff that way.
* A wrapper that "helpfully" interprets the envelope. ``error`` inside a
  control result is the CONTROL error in metres or radians -- a settled stop
  reports ``error: 4.3e-11`` -- and a layer that reads it as a failure flag
  reports a flawless manoeuvre as a failure.
* A 401 ``omnikey_required`` rendered as a crash. OmniLink requires an OmniKey
  on every plan including Free; the user needs the bridge's own sentence, not
  a stack trace.
* The ungated REST verbs (``/drive_forward``, ``/turn``, ``/set_velocity``,
  ``/stop_robot``) becoming reachable from here by a later edit.
"""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

PKG_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_mcp import server


# --------------------------------------------------------------------------- #
# A throwaway HTTP server that records what it was asked and replies verbatim  #
# --------------------------------------------------------------------------- #
class _Recorder:
    """Scripted responses keyed by path, plus the log of what arrived."""

    def __init__(self, routes):
        # routes: {path: (status, body_obj)} or {path: callable(body)->(status, obj)}
        self.routes = routes
        self.seen = []          # [(method, path, parsed_body, headers)]
        # `seen` carries the ROUTE with the query string cut off, which is
        # what every pre-cursor test wants. A cursor-paged read (robot_events)
        # is the one place the query string IS the payload, so it is kept
        # separately rather than by changing the tuple every test indexes.
        self.raw_paths = []     # ["/events?since=12&limit=5", ...]


def _handler_factory(rec):
    class _H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):  # keep pytest output clean
            pass

        def _respond(self, method):
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b""
            try:
                body = json.loads(raw) if raw else None
            except json.JSONDecodeError:
                body = {"__unparsed__": raw.decode("utf-8", "replace")}
            path = self.path.split("?", 1)[0]
            rec.seen.append((method, path, body, dict(self.headers)))
            rec.raw_paths.append(self.path)
            route = rec.routes.get(path)
            if route is None:
                status, payload = 404, {"ok": False, "error": "not_found"}
            elif callable(route):
                status, payload = route(body)
            else:
                status, payload = route
            out = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)

        def do_POST(self):
            self._respond("POST")

        def do_GET(self):
            self._respond("GET")

        def do_DELETE(self):
            self._respond("DELETE")

    return _H


@pytest.fixture(autouse=True)
def _short_timeout(monkeypatch):
    """The shipped 130 s timeout is right against a real harness and wrong for
    a test: a mistake here should fail in seconds, not look like a hang."""
    monkeypatch.setattr(server, "HTTP_TIMEOUT_S", 5.0)


@pytest.fixture
def fake_service():
    """Start a recorder on an ephemeral port; yields (recorder, port)."""
    started = []

    def start(routes):
        rec = _Recorder(routes)
        # THREADING, deliberately. The client pools one keep-alive connection
        # per base URL, and a single-threaded HTTPServer speaking HTTP/1.1 sits
        # inside handle_one_request waiting for the next request on that
        # socket -- it never returns to the accept loop, so shutdown() blocks
        # forever and the whole test session hangs. (Measured: it did.)
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), _handler_factory(rec))
        httpd.daemon_threads = True
        t = threading.Thread(target=httpd.serve_forever, daemon=True)
        t.start()
        started.append(httpd)
        return rec, httpd.server_address[1]

    yield start
    for httpd in started:
        # Drop the pooled socket FIRST: the OS reuses ephemeral ports, and a
        # cached connection to a dead server on a port a later test wins back
        # is a flake that only shows up on someone else's machine.
        server._POOL._drop("http://127.0.0.1:%d" % httpd.server_address[1])
        httpd.shutdown()
        httpd.server_close()


def _payload(content):
    """The JSON an MCP text content block carries."""
    assert content[0]["type"] == "text"
    return json.loads(content[0]["text"])


# --------------------------------------------------------------------------- #
# Task 3 -- the command surface                                                #
# --------------------------------------------------------------------------- #
# A real bridge answer for `drive_forward 1 m`: the control result is MEASURED,
# and `error` in it is the residual distance, not a failure flag.
SETTLED_DRIVE = {
    "status": "ok",
    "tool": "drive_forward",
    "result": {"commanded": 1.0, "achieved": 0.9999999957,
               "error": 4.3e-9, "settled": True, "units": "m"},
}

REFUSED = {
    "ok": False,
    "error": "refused_by_gate",
    "message": "interrogative utterance carries no command",
    "details": {"tool": "drive_forward"},
}

OMNIKEY_REQUIRED = {
    "ok": False,
    "error": "omnikey_required",
    "response": ("Connect OmniLink with your OmniKey and a model provider to "
                 "send AI instructions."),
    "actions": [],
    "setup_url": "https://www.omnilink-agents.com/agents/start",
}


def test_settled_drive_result_passes_through_untouched(fake_service):
    rec, port = fake_service({"/tool": (200, SETTLED_DRIVE)})
    out = server.t_robot_tool({"tool": "drive_forward",
                               "args": {"distance": 1.0}, "port": port})
    got = _payload(out)
    # `http_status` is the ONLY key this layer may add.
    assert got.pop("http_status") == 200
    assert got == SETTLED_DRIVE
    # The control result survives bit for bit -- in particular `error`, which is
    # a residual in metres and must never be read as, or turned into, a failure.
    assert got["result"]["error"] == pytest.approx(4.3e-9)
    assert got["result"]["settled"] is True


def test_refused_by_gate_passes_through_as_data(fake_service):
    rec, port = fake_service({"/tool": (400, REFUSED)})
    out = server.t_robot_tool({"tool": "drive_forward",
                               "args": {"distance": 1.0}, "port": port})
    got = _payload(out)
    assert got.pop("http_status") == 400
    assert got == REFUSED
    assert got["error"] == "refused_by_gate"


def test_omnikey_required_401_reaches_the_caller_with_its_message(fake_service):
    rec, port = fake_service({"/prompt": (401, OMNIKEY_REQUIRED)})
    out = server.t_robot_prompt({"text": "drive forward 1 m", "port": port})
    got = _payload(out)
    assert got.pop("http_status") == 401
    assert got == OMNIKEY_REQUIRED
    # The user-facing sentence and the way to fix it both survive.
    assert "OmniKey" in got["response"]
    assert got["setup_url"].startswith("https://")


def test_service_unavailable_503_reaches_the_caller_too(fake_service):
    body = dict(OMNIKEY_REQUIRED, error="omnilink_unavailable",
                response="OmniLink could not connect. Check your OmniKey and "
                         "model connection, then restart the robot.")
    rec, port = fake_service({"/prompt": (503, body)})
    got = _payload(server.t_robot_prompt({"text": "go home", "port": port}))
    assert got.pop("http_status") == 503
    assert got == body


def test_no_id_key_ever_reaches_the_bridge_as_a_tool_argument(fake_service):
    """The Mavic once refused a good takeoff because an `id` reached the gate
    as `unknown_arg`. Transport fields are stripped on the way IN."""
    rec, port = fake_service({"/tool": (200, SETTLED_DRIVE)})
    server.t_robot_tool({
        "tool": "takeoff",
        "args": {"altitude": 3.0, "id": "mcp-req-17", "tool": "sneaky",
                 "port": 9999},
        "port": port,
    })
    method, path, body, _headers = rec.seen[-1]
    assert (method, path) == ("POST", "/tool")
    assert body == {"tool": "takeoff", "altitude": 3.0}
    for forbidden in ("id", "port"):
        assert forbidden not in body
    assert body["tool"] == "takeoff"  # args could not overwrite the tool name


def test_no_id_key_reaches_the_bridge_on_a_prompt_either(fake_service):
    rec, port = fake_service({"/prompt": (200, {"ok": True, "response": "ok",
                                                "actions": []})})
    server.t_robot_prompt({"text": "stop", "port": port, "id": "mcp-req-9"})
    _m, _p, body, _h = rec.seen[-1]
    assert body == {"text": "stop"}


def test_robot_state_is_a_pure_read_on_the_documented_path(fake_service):
    state = {"pose": {"x": 1.5, "y": -0.25, "yaw": 0.0}, "fault": None,
             "last_tick_ms": 4128, "autonomy_hold": False}
    rec, port = fake_service({"/get_robot_state": (200, state)})
    got = _payload(server.t_robot_state({"port": port}))
    assert got.pop("http_status") == 200
    assert got == state
    method, path, body, _h = rec.seen[-1]
    assert (method, path, body) == ("POST", "/get_robot_state", {})


def test_default_port_is_8765_and_an_explicit_port_wins(fake_service):
    rec, port = fake_service({"/get_robot_state": (200, {"ok": True})})
    assert server.DEFAULT_BRIDGE_PORT == 8765
    assert server._bridge_port({}) == 8765
    assert server._bridge_port({"port": None}) == 8765
    assert server._bridge_port({"port": 6090}) == 6090      # the Mavic
    assert server._bridge_port({"port": "8775"}) == 8775    # langsoak Husky
    server.t_robot_state({"port": port})
    assert rec.seen[-1][1] == "/get_robot_state"


def test_the_ungated_rest_verbs_are_unreachable_from_here():
    """`/drive_forward`, `/turn`, `/set_velocity` and `/stop_robot` bypass the
    safety gate (GATE_COVERAGE.md). The allowlist is enforced at call time, so
    a later edit cannot quietly add one."""
    assert set(server.BRIDGE_PATHS) == {"/prompt", "/tool", "/get_robot_state",
                                        "/events"}
    for ungated in ("/drive_forward", "/turn", "/set_velocity", "/stop_robot",
                    "/reset_to_home"):
        with pytest.raises(ValueError) as exc:
            server._bridge_call(ungated, 8765, {})
        assert "safety gate" in str(exc.value)
    # and no handler in the whole module names one
    src = Path(server.__file__).read_text(encoding="utf-8")
    for ungated in ("\"/drive_forward\"", "\"/set_velocity\"", "\"/stop_robot\""):
        assert ungated not in src


def test_a_query_string_cannot_smuggle_in_an_ungated_verb():
    """`robot_events` made the allowlist check the ROUTE rather than the whole
    string, so that `/events?since=12` is allowed. The obvious wrong way to do
    that is a prefix or substring match, which would let `/stop_robot?x=1`
    through — so the cut is pinned from both sides."""
    for smuggled in ("/stop_robot?since=1", "/drive_forward?limit=5",
                     "/set_velocity?types=a"):
        with pytest.raises(ValueError) as exc:
            server._bridge_call(smuggled, 8765, None, method="GET")
        assert "safety gate" in str(exc.value)


def test_unreachable_bridge_is_a_clean_tool_error_naming_the_ports():
    resp = server._tools_call({"name": "robot_state",
                               "arguments": {"port": 1}})
    assert resp["isError"] is True
    text = resp["content"][0]["text"]
    assert "bridge" in text and "8765" in text and "6090" in text


def test_bridge_tools_are_listed_and_documented():
    names = {t["name"] for t in server._tools_list()["tools"]}
    assert {"robot_prompt", "robot_tool", "robot_state"} <= names
    for name in ("robot_prompt", "robot_tool", "robot_state"):
        desc = server.TOOLS[name][1]
        assert "8765" in desc and "6090" in desc and "8775" in desc
    # the two rules an agent must not have to discover by breaking something
    assert "CONTROL ERROR" in server.TOOLS["robot_prompt"][1]
    assert "omnikey_required" in server.TOOLS["robot_prompt"][1]
    assert "unknown_arg" in server.TOOLS["robot_tool"][1]
    assert "refused_by_gate" in server.TOOLS["robot_prompt"][1]


def test_robot_tool_rejects_a_non_object_args():
    resp = server._tools_call({"name": "robot_tool",
                               "arguments": {"tool": "turn", "args": [1, 2]}})
    assert resp["isError"] is True
    assert "object" in resp["content"][0]["text"]


# --------------------------------------------------------------------------- #
# robot_events -- the BRIDGE's own ring (Track D4), not the harness stream      #
# --------------------------------------------------------------------------- #
# A real bridge page: two events stamped in SIM time, a cursor to continue from,
# and a `dropped` count that is the whole reason the envelope exists.
EVENTS_PAGE = {
    "events": [
        {"type": "joint.limit_hit", "sim_time": 12.416, "step": 1552,
         "robot": "husky", "detail": {"joint": "arm_elbow", "limit": "upper"}},
        {"type": "motion.timed_out", "sim_time": 13.104, "step": 1638,
         "robot": "husky", "detail": {"tool": "drive_forward",
                                      "commanded": 300.0, "achieved": 4.2}},
    ],
    "next_since": 44,
    "dropped": 0,
    "total": 44,
}


def test_robot_events_is_a_get_with_no_body_on_the_ring(fake_service):
    rec, port = fake_service({"/events": (200, EVENTS_PAGE)})
    got = _payload(server.t_robot_events({"port": port}))
    assert got.pop("http_status") == 200
    assert got == EVENTS_PAGE           # verbatim, as every bridge reply is
    method, path, body, _h = rec.seen[-1]
    assert (method, path, body) == ("GET", "/events", None)
    assert rec.raw_paths[-1] == "/events"   # no cursor invented client-side


def test_robot_events_pages_with_the_cursor_it_was_given(fake_service):
    rec, port = fake_service({"/events": (200, EVENTS_PAGE)})
    server.t_robot_events({"port": port, "since": 44, "limit": 20,
                           "types": "joint.limit_hit,motion.timed_out"})
    raw = rec.raw_paths[-1]
    assert raw.startswith("/events?")
    assert "since=44" in raw and "limit=20" in raw
    assert "types=joint.limit_hit%2Cmotion.timed_out" in raw


def test_robot_events_accepts_a_types_list_as_well_as_a_string(fake_service):
    rec, port = fake_service({"/events": (200, EVENTS_PAGE)})
    server.t_robot_events({"port": port,
                           "types": ["fault.controller_lost", "gate.refused"]})
    assert "types=fault.controller_lost%2Cgate.refused" in rec.raw_paths[-1]


def test_robot_events_since_zero_is_sent_not_swallowed(fake_service):
    """`since=0` is a legitimate cursor -- the start of the ring -- and a
    falsy-check would drop it and silently re-read the whole ring instead."""
    rec, port = fake_service({"/events": (200, EVENTS_PAGE)})
    server.t_robot_events({"port": port, "since": 0})
    assert "since=0" in rec.raw_paths[-1]


def test_robot_events_dropped_count_survives_the_trip(fake_service):
    """`dropped` is the count of events the caller will NEVER see. A wrapper
    that summarised the page and lost it would turn a data-loss report into a
    clean-looking read -- exactly the shape of failure this surface exists to
    make impossible."""
    lossy = dict(EVENTS_PAGE, dropped=17, next_since=910)
    _rec, port = fake_service({"/events": (200, lossy)})
    got = _payload(server.t_robot_events({"port": port, "since": 800}))
    assert got["dropped"] == 17 and got["next_since"] == 910


def test_robot_events_non_200_reaches_the_caller_with_its_code(fake_service):
    """A bridge built before D4 has no /events. That must arrive as the
    bridge's own 404, not as a crash and not as an empty ring: 'no events'
    and 'this bridge cannot tell you about events' are different answers."""
    missing = {"ok": False, "error": "not_found",
               "message": "this bridge serves no /events"}
    _rec, port = fake_service({"/nothing": (200, {})})
    got = _payload(server.t_robot_events({"port": port}))
    assert got["http_status"] == 404
    assert got["error"] == "not_found"
    assert missing["error"] == "not_found"          # the shape we expect


def test_robot_events_unreachable_bridge_is_a_clean_tool_error():
    resp = server._tools_call({"name": "robot_events", "arguments": {"port": 1}})
    assert resp["isError"] is True
    text = resp["content"][0]["text"]
    assert "bridge" in text and "8765" in text


def test_robot_events_is_listed_and_states_what_it_cannot_see(fake_service):
    names = {t["name"] for t in server._tools_list()["tools"]}
    assert "robot_events" in names
    desc = server.TOOLS["robot_events"][1]
    # the envelope a caller has to page with
    for field in ("next_since", "dropped", "since"):
        assert field in desc
    # the three honest limits, each one a real way to be misled
    assert "not get_events" in desc.lower()      # two rings, two cursors
    assert "restart" in desc                      # the ring dies with the bridge
    assert "sub-links" in desc                    # contact.began blind spot
    assert "8765" in desc and "6090" in desc and "8775" in desc
    # and it is a read: nothing about it may imply actuation
    props = server.TOOLS["robot_events"][2]["properties"]
    assert set(props) == {"since", "limit", "types", "port"}


def test_bridge_token_is_forwarded_when_configured(fake_service, monkeypatch):
    rec, port = fake_service({"/get_robot_state": (200, {"ok": True})})
    monkeypatch.setattr(server, "BRIDGE_TOKEN", "s3cret")
    server.t_robot_state({"port": port})
    headers = rec.seen[-1][3]
    assert headers.get("Authorization") == "Bearer s3cret"


# --------------------------------------------------------------------------- #
# Tasks 1 and 2 -- pause, resume, break                                        #
# --------------------------------------------------------------------------- #
def _harness(fake_service, monkeypatch, routes):
    rec, port = fake_service(routes)
    monkeypatch.setattr(server, "DEFAULT_HARNESS", f"http://127.0.0.1:{port}")
    return rec


def test_pause_and_resume_hit_their_routes(fake_service, monkeypatch):
    rec = _harness(fake_service, monkeypatch, {
        "/sim/pause": (200, {"ok": True, "paused": True,
                             "lease_remaining_ms": 30000}),
        "/sim/resume": (200, {"ok": True, "paused": False}),
    })
    got = _payload(server.t_sim_pause({"lease_ms": 5000}))
    assert got["paused"] is True and got["lease_remaining_ms"] == 30000
    server.t_sim_resume({})
    assert [(m, p, b) for m, p, b, _h in rec.seen] == [
        ("POST", "/sim/pause", {"lease_ms": 5000}),
        ("POST", "/sim/resume", {}),
    ]


def test_pause_without_a_lease_sends_a_bare_body(fake_service, monkeypatch):
    """No lease invented client-side: the harness applies -- and reports -- its
    own default of 30 s, capped at 300 s."""
    rec = _harness(fake_service, monkeypatch, {"/sim/pause": (200, {"ok": True})})
    server.t_sim_pause({})
    assert rec.seen[-1][2] == {}


def test_break_arms_with_types_filter_and_once(fake_service, monkeypatch):
    armed = {"break_id": "b1", "armed_types": ["contact.began"],
             "diagnostics": []}
    rec = _harness(fake_service, monkeypatch, {"/sim/break": (200, armed)})
    got = _payload(server.t_sim_break({
        "types": ["contact.began"], "filter": {"def": "BOX",
                                               "counterpart": "PLATE"},
        "lease_ms": 60000, "once": True}))
    assert got.pop("http_status") == 200
    assert got == armed
    assert rec.seen[-1][2] == {
        "types": ["contact.began"],
        "filter": {"def": "BOX", "counterpart": "PLATE"},
        "lease_ms": 60000, "once": True}


def test_break_accepts_a_comma_separated_types_string(fake_service, monkeypatch):
    rec = _harness(fake_service, monkeypatch,
                   {"/sim/break": (200, {"break_id": "b2"})})
    server.t_sim_break({"types": "contact.began, joint.limit_hit"})
    assert rec.seen[-1][2] == {"types": ["contact.began", "joint.limit_hit"]}


def test_light_mode_refusal_reaches_the_caller_with_its_code(fake_service,
                                                             monkeypatch):
    """A break that can never fire is refused, not armed. The diagnostic and
    the way out both have to survive the trip."""
    refusal = {"ok": False, "code": "event_type_silenced_in_light_mode",
               "error": "contact.began is not emitted in a light session",
               "workaround": "reload the world with {\"light\": false}"}
    _harness(fake_service, monkeypatch, {"/sim/break": (400, refusal)})
    got = _payload(server.t_sim_break({"types": ["contact.began"]}))
    assert got.pop("http_status") == 400
    assert got == refusal


def test_breaks_list_is_a_get(fake_service, monkeypatch):
    rec = _harness(fake_service, monkeypatch,
                   {"/sim/breaks": (200, {"breaks": []})})
    assert _payload(server.t_sim_breaks({}))["breaks"] == []
    assert rec.seen[-1][:2] == ("GET", "/sim/breaks")


def test_break_clear_prefers_delete(fake_service, monkeypatch):
    rec = _harness(fake_service, monkeypatch, {
        "/sim/break/b1": (200, {"break_id": "b1", "removed": True})})
    got = _payload(server.t_sim_break_clear({"break_id": "b1"}))
    assert got["removed"] is True and got["route"] == "DELETE /sim/break/<id>"
    assert rec.seen[-1][:2] == ("DELETE", "/sim/break/b1")
    assert len(rec.seen) == 1  # no speculative second call


def test_break_clear_falls_back_to_post_when_delete_is_not_routed(
        fake_service, monkeypatch):
    """The harness lane may ship the disarm as a POST. Fall back on a MISSING
    ROUTE (an error carrying no break_id), never on the route's own refusal."""
    rec = _harness(fake_service, monkeypatch, {
        "/sim/break/delete": (200, {"break_id": "b1", "removed": True})})
    got = _payload(server.t_sim_break_clear({"break_id": "b1"}))
    assert got["removed"] is True and got["route"] == "POST /sim/break/delete"
    assert [(m, p) for m, p, _b, _h in rec.seen] == [
        ("DELETE", "/sim/break/b1"), ("POST", "/sim/break/delete")]
    assert rec.seen[-1][2] == {"break_id": "b1"}


def test_break_clear_does_not_retry_a_genuine_unknown_id(fake_service,
                                                         monkeypatch):
    rec = _harness(fake_service, monkeypatch, {
        "/sim/break/nope": (404, {"break_id": "nope", "removed": False,
                                  "code": "BREAK_NOT_FOUND"})})
    got = _payload(server.t_sim_break_clear({"break_id": "nope"}))
    assert got["removed"] is False
    assert len(rec.seen) == 1  # one honest 404, not two confusing ones


def test_pause_and_break_tools_state_their_limits():
    pause = server.TOOLS["sim_pause"][1]
    assert "LEASED" in pause and "30 s" in pause and "300 s" in pause
    assert "SINGLE-STEPPING" in pause and "lease_remaining_ms" in pause
    brk = server.TOOLS["sim_break"][1]
    # the two honest limits the plan requires, verbatim in substance
    assert "NEXT SUPERVISOR TICK" in brk and "not sub-step" in brk
    assert "event_type_silenced_in_light_mode" in brk
    assert '{"light": false}' in brk
    assert "break.hit" in brk
    assert "lease_remaining_ms" in server.TOOLS["sim_step"][1]


def test_module_docstring_no_longer_claims_there_is_no_pause():
    doc = server.__doc__
    assert "no pause" not in doc
    assert "sim_pause" in doc and "sim_break" in doc
    # what is genuinely still missing stays stated
    assert "run-diff" in doc and "never velocity" in doc


def test_tool_count_is_the_number_the_readme_quotes():
    readme = (Path(server.__file__).resolve().parents[2] / "README.md")
    assert f"{len(server.TOOLS)} tools" in readme.read_text(encoding="utf-8")
