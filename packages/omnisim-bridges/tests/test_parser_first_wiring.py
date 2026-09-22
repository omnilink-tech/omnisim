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

"""THE PARSER IS ON THE LIVE `/prompt` PATH. Both of them.

WHY THIS FILE EXISTS. The README, the four OmniLink guides and
`positioning.md` all say: "a deterministic parser interprets the operator's
sentence first, and a model is called only for what the parser declines."
On 2026-09-22 that was true of no shipped code path. `route.short_circuit`
had NO production caller at all -- the mobile bridge imported it as
`shared_short_circuit` and never called that name -- and `route_if_enabled`
had four call sites, every one of them inside an `IntentRouter.dispatch`
body that had just been deleted. So the model answered every live turn and
`parser_stats()` reported zeros, while the only "parser first" evidence in
the tree came from `tests/benchmarks/shiftdemo/run_cascade.py`, which calls
`interpret()` IN THE BENCHMARK PROCESS and POSTs frames to `/tool`. It
never touched a bridge's `/prompt`.

These tests drive the real entry-point functions -- `_route_post` and
`handle_wwi_message` / `handle_wwi`, exec'd out of the shipped controller
files -- so a future edit that quietly unwires the parser again fails here
rather than in a demo.

THE ORDER UNDER TEST IS: access check -> parser -> model -> gate.
  * access first  -- pinned in test_omnikey_access.py, which must keep
                     passing unchanged. The parser must NEVER become a
                     keyless path (2026-09-22 owner policy).
  * parser        -- a confident COMMAND is answered here, with NO relay
                     dispatch, and the turn is counted.
  * model         -- everything the parser declines still reaches the relay.
  * gate          -- `route.execute` calls `gate.check` before any frame
                     reaches an adapter, so the parser path is vetted on
                     the same rail a model-produced frame is.
"""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List
from unittest.mock import Mock

import pytest

from omnisim_bridges import route as _route
from omnisim_bridges.http_security import (
    RequestError, finite_number, nonempty_string, require_field)

ROOT = Path(__file__).resolve().parents[3]
CONTROLLERS = ROOT / "projects/samples/demos/controllers"
COURIER = (ROOT / "projects/robots/omnisim/omnitug500/controllers"
           / "omnitug500_courier/omnitug500_courier.py")

BRIDGE_FILE = {
    "mobile": CONTROLLERS / "omnilink_mobile_bridge/omnilink_mobile_bridge.py",
    "arm": CONTROLLERS / "omnilink_arm_bridge/omnilink_arm_bridge.py",
    "quadruped": CONTROLLERS / "omnilink_quadruped_bridge/omnilink_quadruped_bridge.py",
    "courier": COURIER,
    "drone": CONTROLLERS / "mavic_omnilink_bridge/mavic_omnilink_bridge.py",
    "drone_window": CONTROLLERS / "mavic_omnilink_bridge/mavic_chat_router.py",
}

WINDOW_FN = {"mobile": "handle_wwi_message", "arm": "handle_wwi_message",
             "quadruped": "handle_wwi", "courier": "handle_wwi"}

# A command each surface's grammar parses exactly, and which the bridge
# under test actually serves, so the parse reaches an adapter.
COMMAND = {"mobile": "drive forward 2 metres", "arm": "open the gripper",
           "quadruped": "sit", "courier": "stop"}

# A challenge to the robot's own account. `interpret` classifies it
# CONVERSATION, `short_circuit` abstains, and route.py's own comment
# explains why that matters: a quadruped once answered this one by
# actuating reset_to_home.
CHALLENGE = "you never stopped, I was watching - admit it"


def load_function(path: Path, name: str, namespace: Dict[str, Any]):
    """Exec one top-level function out of a controller, with no imports.

    Same trick as test_omnikey_access.py: the controllers import `omnisim`
    (the simulator's own module) at module scope, so importing them is not
    possible off a running engine.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__",
                             names=[ast.alias(name="annotations")], level=0),
              fn],
        type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[name]


class FakeBridge:
    """The bits of a bridge the parser and the /prompt preamble touch."""

    intents = None
    window_configured = True

    def __init__(self) -> None:
        self.calls: List[Any] = []
        self.window: List[str] = []
        self.last_external_cmd = 0.0
        self.last_external_src = ""
        self.robot_id = "fake"
        self.model = "Fake"
        self.capabilities = {}

    # -- window / bookkeeping ------------------------------------- #
    def queue_window(self, line: str) -> None:
        self.window.append(line)

    def note_external_command(self, src: str) -> None:
        self.last_external_src = src

    def end_chat_turn(self, marker: Any, actions: Any = None) -> None:
        self.calls.append(("end_chat_turn", actions))

    # -- actions the four grammars reach -------------------------- #
    def act_drive_forward(self, distance: float, speed: Any = None,
                          wait: bool = False) -> Dict[str, Any]:
        self.calls.append(("drive_forward", distance))
        return {"accepted": True}

    def act_turn(self, angle_rad: float, wait: bool = False) -> Dict[str, Any]:
        self.calls.append(("turn", angle_rad))
        return {"accepted": True}

    def act_open_gripper(self) -> Dict[str, Any]:
        self.calls.append(("open_gripper", None))
        return {"accepted": True}

    def act_sit(self) -> Dict[str, Any]:
        self.calls.append(("sit", None))
        return {"accepted": True}

    def act_stop(self) -> Dict[str, Any]:
        self.calls.append(("stop", None))
        return {"accepted": True, "stationary": True,
                "measured": {"speed_mps": 0.0, "over_s": 0.5}}


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    """Default behaviour, every time: unset means the parser answers."""
    monkeypatch.delenv("OMNISIM_BRIDGE_PARSER_FIRST", raising=False)
    _route._STATS.update(turns=0, eligible=0, short_circuited=0,
                         relay_calls=0)
    _route._STATS["by_intent"] = {}
    yield


# ── HTTP `/prompt` ───────────────────────────────────────────────────

def _http_prompt(kind: str, text: str, bridge: FakeBridge, relay: Mock):
    """Run the shipped `_route_post` for POST /prompt {text}."""
    ns: Dict[str, Any] = {
        "relay": relay, "bridge": bridge,
        "nonempty_string": nonempty_string, "require_field": require_field,
        "finite_number": finite_number, "RequestError": RequestError,
        "shared_short_circuit": _route.short_circuit,
        "_shared_short_circuit": _route.short_circuit,
        "shared_reply_payload": _route.reply_payload,
        "_reply_payload": _route.reply_payload,
        # PROTOCOL.md §5.7.2 / plan D3: the bridges stamp `via` on every 200
        # from /prompt through this helper. Both spellings, because the
        # bridges disagree on the underscore prefix.
        "shared_stamp_via": _route.stamp_via,
        "_shared_stamp_via": _route.stamp_via,
        "_tx_begin": lambda *a, **k: None,
        "_tx_end": lambda *a, **k: None,
        "_tx_journal_seq": lambda *a, **k: 0,
        "_merge_auto_reads": lambda *a, **k: None,
        "connection_error": lambda: {"error": "omnikey_required"},
    }
    if kind == "courier":
        # The courier reads its own body inside do_POST; the four demo
        # bridges are handed one.
        fn = load_function(BRIDGE_FILE[kind], "do_POST", ns)
        handler = SimpleNamespace(path="/prompt",
                                  _body=lambda: {"text": text},
                                  _json=lambda code, body: (code, body))
        return fn(handler)
    fn = load_function(BRIDGE_FILE[kind], "_route_post", ns)
    handler = SimpleNamespace(path="/prompt",
                              _json=lambda code, body: (code, body))
    return fn(handler, {"text": text})


@pytest.mark.parametrize("kind", ["mobile", "arm", "quadruped", "courier"])
def test_http_command_is_answered_by_the_parser_with_no_model(kind):
    """THE HEADLINE CLAIM, on the HTTP path. No relay dispatch at all."""
    bridge, relay = FakeBridge(), Mock()
    status, body = _http_prompt(kind, COMMAND[kind], bridge, relay)

    assert status == 200
    relay.dispatch_sync.assert_not_called()
    relay.dispatch_async.assert_not_called()
    assert body["via"] == "parser"
    assert [c[0] for c in bridge.calls if c[0] != "end_chat_turn"], \
        "the parser must have actuated something"
    assert body["actions"], "the reply must name the tools it ran"
    assert _route._STATS["turns"] == 1
    assert _route._STATS["short_circuited"] == 1


@pytest.mark.parametrize("kind", ["mobile", "arm", "quadruped", "courier"])
def test_http_conversation_still_reaches_the_model(kind):
    """What the parser DECLINES is exactly what the model gets."""
    bridge, relay = FakeBridge(), Mock()
    relay.dispatch_sync.return_value = {"response": "ok", "actions": []}
    _http_prompt(kind, CHALLENGE, bridge, relay)

    relay.dispatch_sync.assert_called_once()
    assert relay.dispatch_sync.call_args.args[0] == CHALLENGE
    assert bridge.calls == [] or all(c[0] == "end_chat_turn"
                                     for c in bridge.calls)
    assert _route._STATS["short_circuited"] == 0
    assert _route._STATS["relay_calls"] == 1


def test_the_drone_http_prompt_parses_before_it_pays():
    """The Mavic reaches the parser through mavic_chat_router._StateBridge.

    It is the one bridge whose act_* live in a separate module, so the
    wiring is the easiest of the five to get wrong and not notice.
    """
    seen: List[Any] = []

    class _FakeStateBridge:
        def __init__(self, state):
            seen.append(state)

        def act_takeoff(self, altitude=None):
            seen.append(("takeoff", altitude))
            return {"accepted": True}

    relay = Mock()
    ns: Dict[str, Any] = {
        "state": SimpleNamespace(omnilink_relay=relay),
        "connection_error": lambda: {"error": "omnikey_required"},
        "nonempty_string": nonempty_string, "require_field": require_field,
        "_shared_short_circuit": _route.short_circuit,
        "_reply_payload": _route.reply_payload,
        # §5.7.2 / D3: `via` is stamped on every 200 from /prompt.
        "_shared_stamp_via": _route.stamp_via,
        "_StateBridge": _FakeStateBridge,
    }
    fn = load_function(BRIDGE_FILE["drone"], "_route_post", ns)
    handler = SimpleNamespace(
        path="/prompt", _read_json=lambda: {"text": "take off to 3 metres"},
        _json=lambda code, body: (code, body))
    status, body = fn(handler)

    assert status == 200 and body["via"] == "parser"
    relay.dispatch_sync.assert_not_called()
    assert ("takeoff", 3.0) in seen


# ── The robot-window path ────────────────────────────────────────────

def _window_prompt(kind: str, text: str, bridge: FakeBridge, relay: Mock):
    """Run the shipped window handler for a `prompt:<text>` wwi message.

    The real `_parser_first_window` is loaded too, with its worker forced
    inline so the assertions are deterministic. The shipped default spawns
    a daemon thread -- `handle_wwi*` runs on the SIM thread and a compound
    order asks its first motion to block, which there is a deadlock.
    """
    path = BRIDGE_FILE[kind]
    inner = load_function(path, "_parser_first_window", {
        "shared_parser_window": _route.parser_first_window,
        "_shared_parser_window": _route.parser_first_window,
        "_tx_begin": lambda *a, **k: None,
        "_tx_end": lambda *a, **k: None,
    })

    def _inline(*args, **kwargs):
        return inner(*args, spawn=lambda fn: fn())

    ns: Dict[str, Any] = {
        "_parser_first_window": _inline,
        "_tx_window_cb": lambda *a: a[-1],
        "_autoread_window_cb": lambda *a: a[-1],
        "_on_relay_event": lambda *a: None,
        "on_relay_event": lambda *a: None,
        "reject_window_prompt": lambda b: b.queue_window("error:no key"),
        "push_configure": lambda *a: None,
    }
    fn = load_function(path, WINDOW_FN[kind], ns)
    fn(bridge, relay, "prompt:" + text)


@pytest.mark.parametrize("kind", ["mobile", "arm", "quadruped", "courier"])
def test_window_command_is_answered_by_the_parser_with_no_model(kind):
    """The window path is the one a human types into. Same rule.

    A fix that only covered HTTP would demo wrongly: the flagship demo is
    someone right-clicking the robot and typing.
    """
    bridge, relay = FakeBridge(), Mock()
    _window_prompt(kind, COMMAND[kind], bridge, relay)

    relay.dispatch_async.assert_not_called()
    relay.dispatch_sync.assert_not_called()
    assert [c[0] for c in bridge.calls if c[0] != "end_chat_turn"], \
        "the parser must have actuated something"
    assert any(line.startswith("agent:") for line in bridge.window)
    assert any(line.startswith("tool:") for line in bridge.window)
    assert bridge.window[-1] == "status:idle", "the panel needs a terminal"
    assert _route._STATS["turns"] == 1
    assert _route._STATS["short_circuited"] == 1


@pytest.mark.parametrize("kind", ["mobile", "arm", "quadruped", "courier"])
def test_window_conversation_still_reaches_the_model(kind):
    bridge, relay = FakeBridge(), Mock()
    _window_prompt(kind, CHALLENGE, bridge, relay)

    relay.dispatch_async.assert_called_once()
    assert relay.dispatch_async.call_args.args[0] == CHALLENGE
    assert [c for c in bridge.calls if c[0] != "end_chat_turn"] == []
    assert _route._STATS["short_circuited"] == 0


def test_the_drone_window_path_parses_before_it_pays():
    relay = Mock()
    state = SimpleNamespace(omnilink_relay=relay, lock=_DummyLock(),
                            window_outbox=[], x=0.0, y=0.0, z=0.0, yaw=0.0,
                            mode="idle", fault=None, target_x=None,
                            target_y=None, target_yaw=None,
                            target_altitude=0.0)
    ns: Dict[str, Any] = {}
    real = load_function(BRIDGE_FILE["drone_window"], "_parser_first_window", ns)
    handler = load_function(BRIDGE_FILE["drone_window"], "handle_wwi_message", {
        "queue_window": lambda st, line: st.window_outbox.append(line),
        "push_configure": lambda st: None,
        "_act_stop": lambda st: None,
        "_parser_first_window": (
            lambda st, text, to_model, spawn=None:
            real(st, text, to_model, spawn=lambda fn: fn())),
    })
    # `_parser_first_window` imports _StateBridge and queue_window from the
    # module's own globals; give it the real ones.
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "_mavic_chat_router_under_test", BRIDGE_FILE["drone_window"])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ns["_StateBridge"] = mod._StateBridge
    ns["queue_window"] = mod.queue_window

    handler(state, "prompt:take off to 3 metres")

    relay.dispatch_async.assert_not_called()
    assert state.target_altitude == pytest.approx(3.0)
    assert state.mode == "takeoff"
    assert state.window_outbox[-1] == "status:idle"


class _DummyLock:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


# ── The gate is on the parser path ───────────────────────────────────

def test_the_parser_path_is_gated_and_actuates_nothing_when_refused():
    """⚠️ IF THIS EVER FAILS, THE WIRING CREATED AN UNGATED ACTUATION PATH.

    `route.execute` calls `gate.check(utterance, frames, surface=...)`
    before a COMMAND frame reaches an adapter, which is the only reason
    parser-first is safe to ship: the gate does not care who produced the
    frames.
    """
    bridge, relay = FakeBridge(), Mock()
    # Parses as a confident COMMAND (drive_forward{distance: 2.0}); the
    # gate vetoes it on `retracted`. "drive forward 300 metres" would NOT
    # test this: the interpreter calls that one AMBIGUOUS and it never
    # reaches the gate at all.
    status, body = _http_prompt(
        "mobile", "drive forward 2 metres - sorry, ignore that",
        bridge, relay)

    assert status == 200 and body["via"] == "parser"
    assert [c for c in bridge.calls if c[0] != "end_chat_turn"] == [], \
        "a gate-refused order must reach no actuator"
    assert body["actions"][0]["result"] == "refused"
    assert body["error"], "a refused action must surface as a top-level error"
    relay.dispatch_sync.assert_not_called()


def test_a_gate_refusal_is_not_handed_on_to_the_model():
    """The one decline rule that must NOT apply.

    `parser_first_run` hands an all-`unsupported` parse to the model, on
    the grounds that nothing was actuated and the model may serve the
    request another way. A gate REFUSAL is the opposite case: passing it
    on is an invitation to find another way to do what the gate vetoed.
    """
    b = FakeBridge()
    out = _route.short_circuit(
        b, "drive forward 2 metres - sorry, ignore that", "mobile")
    assert out is not None, "a refusal must be ANSWERED, never handed on"
    assert out["tools"][0][1] == "refused"
    assert b.calls == []


def test_an_action_this_robot_cannot_do_is_declined_to_the_model():
    """`unsupported` is a DECLINE, not an answer.

    The OmniTug courier has stations and a route queue, not a
    `drive_forward`. Closing that turn with "this robot cannot drive
    forward" would make parser-first strictly worse than the relay it
    replaced, so it goes to the model with nothing actuated.
    """
    class _StationRover:
        """A courier-shaped bridge: a stop, and no wheels verb at all."""

        def __init__(self) -> None:
            self.calls: List[Any] = []

        def act_stop(self):
            self.calls.append("stop")
            return {"accepted": True}

    b = _StationRover()
    assert not hasattr(b, "act_drive_forward")
    assert _route.short_circuit(b, "drive forward 2 metres", "mobile") is None
    assert b.calls == []
    assert _route._STATS["relay_calls"] == 1


# ── The opt-out still works, on the live path ────────────────────────

@pytest.mark.parametrize("kind", ["mobile", "arm"])
def test_the_zero_opt_out_restores_shadow_mode_on_the_live_path(
        kind, monkeypatch):
    """`OMNISIM_BRIDGE_PARSER_FIRST=0` is the one-variable revert.

    It is what a live run reverts to if parser-first turns out to demo
    badly, so it has to be exercised through the shipped handler and not
    only through `route` in isolation.
    """
    monkeypatch.setenv("OMNISIM_BRIDGE_PARSER_FIRST", "0")
    bridge, relay = FakeBridge(), Mock()
    relay.dispatch_sync.return_value = {"response": "ok", "actions": []}
    _http_prompt(kind, COMMAND[kind], bridge, relay)

    relay.dispatch_sync.assert_called_once()
    assert [c for c in bridge.calls if c[0] != "end_chat_turn"] == []
    assert _route._STATS["eligible"] == 1, "shadow mode still measures"
    assert _route._STATS["short_circuited"] == 0
