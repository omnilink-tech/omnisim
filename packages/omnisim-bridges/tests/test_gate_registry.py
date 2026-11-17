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

"""The gate learns its schema from the bridge, not from a second copy.

Every test here is a defect that shipped. `gate.SPECS` was hand-written
while the bridges published a correct JSON schema for every tool at
registration time, and the two drifted: on 2026-09-21 the arm registered
19 tools of which 10 were absent from SPECS.

That drift failed in BOTH directions, which is why there are two groups
of tests below and why fixing only one of them would have been worse than
fixing neither -- a gate that refuses real work gets switched off.
"""
import copy

import pytest

from omnisim_bridges import gate
from omnisim_bridges.tool import Tool


@pytest.fixture(autouse=True)
def _restore_specs():
    """SPECS is module-global and register_tools mutates it."""
    saved = copy.deepcopy(gate.SPECS)
    yield
    gate.SPECS.clear()
    gate.SPECS.update(saved)


def tool(name, props=None, required=(), physical=True):
    return Tool(name, "", {"type": "object",
                           "properties": props or {},
                           "required": list(required)},
                lambda a: {}, physical=physical)


NUM = {"type": "number"}


# ── Direction 1: tools SPECS did not know were NOT GATED ─────────────
#
# check() emits `unknown_tool` and skips every other rule, and all five
# call sites filter `unknown_tool` out -- correctly, since the caller has
# already resolved the tool. The net effect was that anything the table
# had never heard of reached a motor with no checks at all.

def test_an_unregistered_tool_is_not_gated_at_all():
    """The defect, pinned. `grasp` was invisible to every rule."""
    out = gate.check("how much reach have you got left?",
                     [{"tool": "grasp", "args": {"width": 99.0}}])
    assert [r.rule for r in out] == ["unknown_tool"]


def test_registering_puts_an_arm_tool_under_the_intent_rules():
    """A QUESTION MAY NOT CLOSE A GRIPPER. It could, until registration."""
    gate.register_tools([tool("grasp", {"width": NUM})], surface="arm")
    out = gate.check("how much reach have you got left?",
                     [{"tool": "grasp", "args": {"width": 0.05}}])
    assert [r.rule for r in out] == ["interrogative"]


def test_registering_puts_an_arm_tool_under_the_schema_rules():
    gate.register_tools([tool("set_tcp_target", {"x": NUM, "y": NUM, "z": NUM})],
                        surface="arm")
    out = gate.check("move the tool to the tray",
                     [{"tool": "set_tcp_target", "args": {"turbo": True}}])
    assert [r.rule for r in out] == ["unknown_arg"]


def test_a_read_only_tool_still_answers_a_question():
    """The carve-out survives registration -- answering is what it is for."""
    gate.register_tools([tool("get_reach_envelope", physical=False)],
                        surface="arm")
    assert gate.check("how much reach have you got left?",
                      [{"tool": "get_reach_envelope", "args": {}}]) == []


# ── Direction 2: tools SPECS knew, with args it did not ──────────────
#
# Both of these were live HTTP 400s. The features were unreachable over
# the platform's callback, and neither was a judgement the gate made --
# it was schema drift wearing a safety verdict's clothes.

def test_place_xyz_was_refused_and_is_not_any_more():
    assert [r.rule for r in gate.check(
        "put it on the pallet",
        [{"tool": "place", "args": {"xyz": [0.3, 0.2, 0.1]}}])] == ["unknown_arg"]
    gate.register_tools([tool("place", {"xyz": {"type": "array"}})], surface="arm")
    assert gate.check("put it on the pallet",
                      [{"tool": "place", "args": {"xyz": [0.3, 0.2, 0.1]}}]) == []


def test_attach_trolley_def_was_refused_and_is_not_any_more():
    assert [r.rule for r in gate.check(
        "attach the trolley",
        [{"tool": "attach_trolley", "args": {"def": "TROLLEY_C"}}])] == ["unknown_arg"]
    gate.register_tools([tool("attach_trolley", {"def": {"type": "string"}})],
                        surface="mobile")
    assert gate.check("attach the trolley",
                      [{"tool": "attach_trolley", "args": {"def": "TROLLEY_C"}}]) == []


def test_merging_keeps_the_parsers_vocabulary():
    """The parser emits place{object,target}; the arm posts place{xyz}.

    Both have to work, or registration fixes the bridge by breaking the
    interpreter.
    """
    gate.register_tools([tool("place", {"xyz": {"type": "array"}})], surface="arm")
    assert gate.check("put the block on the tray",
                      [{"tool": "place",
                        "args": {"object": "block", "target": "tray"}}]) == []


# ── Registration may not widen what a bridge is allowed to do ────────

def test_a_registered_physical_tool_is_guarded_never_safe():
    """SAFE is the de-escalation exemption and is only granted by hand.

    Inheriting it from a registry would let a bridge name a tool into the
    carve-out and skip the intent rules.
    """
    gate.register_tools([tool("stop_and_fling", {})], surface="mobile")
    assert gate.SPECS["stop_and_fling"]["tier"] == gate.GUARDED
    assert [r.rule for r in gate.check(
        "did you stop and fling it?",
        [{"tool": "stop_and_fling", "args": {}}])] == ["interrogative"]


def test_a_bridge_cannot_downgrade_a_physical_tool():
    """Upgrades propagate, downgrades do not -- or a bridge opts its own
    actuator out of the gate."""
    gate.register_tools([tool("drive_forward", {"distance": NUM}, physical=False)],
                        surface="mobile")
    assert gate.SPECS["drive_forward"]["physical"] is True


def test_registration_is_idempotent():
    t = [tool("grasp", {"width": NUM})]
    assert gate.register_tools(t, surface="arm") == ["grasp"]
    assert gate.register_tools(t, surface="arm") == []


# ── The rails stay here, and the fixture keeps freezing them ─────────

def test_magnitude_rails_still_apply_to_a_registered_tool():
    gate.register_tools([tool("takeoff", {"altitude": NUM})], surface="drone")
    assert [r.rule for r in gate.check(
        "get up to 200 metres and have a look",
        [{"tool": "takeoff", "args": {"altitude": 200.0}}])] == ["implausible"]


def test_an_undeclared_tool_falls_back_to_the_name_heuristic():
    """physical= is optional, so adding the field changed nothing on the
    day it landed."""
    t = Tool("drive_forward", "", {"type": "object", "properties": {}},
             lambda a: {})
    assert t.physical is True
    assert t.declared_physical is False
    assert Tool("get_line_counts", "", {}, lambda a: {}).physical is False


# ── Per-surface rails: the same tool, two robots, two right answers ──
#
# `move_body` is served by the drone AND the quadruped, with the same
# three argument names meaning different things. One global rail could
# only ever be right for one of them.

def test_a_drone_climb_is_not_a_quadruped_body_shift():
    """40 m is routine for one robot and absurd for the other."""
    assert gate.check("climb 40 metres",
                      [{"tool": "move_body", "args": {"vertical": 40.0}}],
                      surface="drone") == []
    assert [r.rule for r in gate.check(
        "shift the body down 40 metres",
        [{"tool": "move_body", "args": {"vertical": 40.0}}],
        surface="quadruped")] == ["implausible"]


def test_a_drone_still_has_a_ceiling():
    assert [r.rule for r in gate.check(
        "climb 300 metres",
        [{"tool": "move_body", "args": {"vertical": 300.0}}],
        surface="drone")] == ["implausible"]


def test_the_rail_does_not_depend_on_registration_order():
    """THE BUG THIS FIXES. `surface` was recorded on the spec by whichever
    bridge registered first, so a drone registered before a quadruped
    allowed a 40 m body shift and the reverse order refused every real
    drone climb. An order-dependent safety property is worse than a
    wrong one, because it looks correct in whichever order you test."""
    for order in (("drone", "quadruped"), ("quadruped", "drone")):
        gate.SPECS["move_body"].pop("surface", None)
        for surf in order:
            gate.register_tools([tool("move_body", {"vertical": NUM})],
                                surface=surf)
        assert gate.check("climb 40 metres",
                          [{"tool": "move_body", "args": {"vertical": 40.0}}],
                          surface="drone") == [], order
        assert [r.rule for r in gate.check(
            "shift down 40 metres",
            [{"tool": "move_body", "args": {"vertical": 40.0}}],
            surface="quadruped")] == ["implausible"], order


def test_the_altitude_rail_finally_fires():
    """MAX_ALTITUDE_M sat in gate.py unreachable: no drone frame could get
    to it, because the Mavic had no gated surface at all."""
    assert [r.rule for r in gate.check(
        "get up to 200 metres and have a look",
        [{"tool": "takeoff", "args": {"altitude": 200.0}}],
        surface="drone")] == ["implausible"]


def test_a_question_no_longer_lands_the_aircraft():
    """`land` is an ordinary English word, and the drone's keyword ladder
    landed the aircraft on this sentence."""
    assert [r.rule for r in gate.check(
        "where did the package land?",
        [{"tool": "land", "args": {}}],
        surface="drone")] == ["interrogative"]


def test_a_shared_tool_with_no_caller_surface_takes_the_STRICTER_rail():
    """A caller that passes no surface at all.

    Guessing air is the unsafe guess -- it raises a 1 m limit to 120 m.
    Guessing ground is wrong only in the direction that refuses a
    legitimate climb, which is visible and fixed by passing surface=.
    Every shipped caller now passes one; this is the floor, not the plan.
    """
    for order in (("drone", "quadruped"), ("quadruped", "drone")):
        gate.SPECS["move_body"].pop("surfaces", None)
        gate.SPECS["move_body"].pop("surface", None)
        for surf in order:
            gate.register_tools([tool("move_body", {"vertical": NUM})],
                                surface=surf)
        assert [r.rule for r in gate.check(
            "shift the body down 40 metres",
            [{"tool": "move_body", "args": {"vertical": 40.0}}])] == \
            ["implausible"], f"unsafe fallback with registration order {order}"


# ── Direction 3: registration must CARRY the caller's surface ────────
#
# `OmniLinkRelay.__init__` registered with
# `surface=getattr(self, "surface", None)` and never assigned `self.surface`,
# so the expression was a constant None and EVERY tool of EVERY bridge was
# registered surface-less. Nothing failed loudly: with no recorded surface
# and no caller surface a tool takes the ground / body-shift rail, so the
# only symptom was a drone refused its own climb. ⚠️ That is NOT the same
# as "no surface means the strictest rail", which is false as a general
# rule (a tool exactly one class registered keeps that class's rail) and
# which this package has had to unpick three times. Both halves are pinned
# here -- the relay assigns it, and each bridge passes its own.

import ast                                                   # noqa: E402
import pathlib                                               # noqa: E402

_CONTROLLERS = (pathlib.Path(__file__).resolve().parents[3]
                / "projects/samples/demos/controllers")

BRIDGE_SURFACES = [
    ("omnilink_mobile_bridge/omnilink_mobile_bridge.py", "mobile"),
    ("omnilink_arm_bridge/omnilink_arm_bridge.py", "arm"),
    ("omnilink_quadruped_bridge/omnilink_quadruped_bridge.py", "quadruped"),
    ("mavic_omnilink_bridge/mavic_omnilink_bridge.py", "drone"),
]


@pytest.mark.parametrize("rel,surface", BRIDGE_SURFACES,
                         ids=[s for _, s in BRIDGE_SURFACES])
def test_every_bridge_builds_its_relay_with_its_own_surface(rel, surface):
    """Source-level, because constructing these needs the engine.

    The four names are the ones interpret.py declares. A bridge that omits
    the keyword gets None, which is the defect this replaced.
    """
    path = _CONTROLLERS / rel
    if not path.exists():                                # pragma: no cover
        pytest.skip(f"{path} not present")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, "id", None) == "OmniLinkRelay"]
    assert calls, f"{rel} never constructs an OmniLinkRelay"
    for call in calls:
        passed = {k.arg: k.value for k in call.keywords}
        assert "surface" in passed, (
            f"{rel} builds its relay with no surface=; every tool it serves "
            f"would be registered surface-less, so a caller that declares no "
            f"surface either gets the ground / body-shift rail -- which "
            f"refuses this robot's own legitimate magnitudes if it is not a "
            f"ground robot")
        value = passed["surface"]
        assert isinstance(value, ast.Constant) and value.value == surface, (
            f"{rel} passes surface={ast.dump(value)}, expected {surface!r}")


def test_the_relay_registers_its_tools_with_its_surface():
    """End to end through the real constructor, offline.

    `self.surface` must be assigned BEFORE register_tools() runs, which is
    the ordering the original defect got wrong by never assigning it at all.
    """
    from omnisim_bridges.relay import OmniLinkRelay

    probe = tool("probe_climb", {"vertical": NUM})
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="SurfaceProbe",
        main_task="test",
        tools=[probe],
        usage_enabled=False,
        memory_enabled=False,
        surface="drone",
    )
    assert relay.surface == "drone"
    spec = gate.SPECS["probe_climb"]
    assert spec["surface"] == "drone", "registered without the caller's surface"
    assert spec["surfaces"] == {"drone"}


def test_no_registered_tool_is_left_without_a_surface():
    """Register each bridge's declared surface and check nothing is None.

    A tool whose spec carries `surface: None` is a tool nobody claimed, and
    the gate then judges it on whatever rail the table happens to hold.
    """
    for surface in ("mobile", "arm", "quadruped", "drone"):
        gate.register_tools([tool(f"probe_{surface}", {"distance": NUM})],
                            surface=surface)
    for name in ("probe_mobile", "probe_arm", "probe_quadruped", "probe_drone"):
        spec = gate.SPECS[name]
        assert spec.get("surface") is not None, f"{name} registered surface-less"
        assert spec.get("surfaces"), f"{name} has an empty surfaces set"


# ── A transport field is not an argument ─────────────────────────────

def test_a_request_id_never_reaches_the_gate_as_an_argument():
    """⚠️ THIS REFUSED A GOOD TAKEOFF. Do not let it regress.

    `id` is PROTOCOL.md 3.3 request-idempotency metadata. Left in the body
    it arrives at the gate as a tool argument, trips `unknown_arg`, and a
    perfectly good `takeoff{altitude: 3}` comes back refused_by_gate -- a
    transport detail wearing a safety verdict's clothes. Three of the five
    `/tool` handlers never stripped it; `serve_tool` is now the only one,
    and it does.

    The first assertion is the proof that stripping is load-bearing: the
    gate genuinely does refuse the un-stripped call.
    """
    from omnisim_bridges.bridge_base import serve_tool, tool_args

    gate.register_tools([tool("takeoff", {"altitude": NUM})], surface="drone")

    raw = {"tool": "takeoff", "id": "req-00000000", "altitude": 3.0}
    assert "unknown_arg" in (gate.reject_toolcall(
        "takeoff", {k: v for k, v in raw.items() if k != "tool"},
        surface="drone") or ""), (
        "the gate stopped minding `id`; this test no longer proves anything")

    assert tool_args(raw) == {"altitude": 3.0}

    seen = {}
    code, payload = serve_tool(
        "takeoff", raw, lambda args: seen.update(args) or {"accepted": True},
        surface="drone")
    assert code == 200, payload
    assert seen == {"altitude": 3.0}, "a transport field reached the tool"


def test_serve_tool_strips_every_transport_field_it_declares():
    """`utterance` is an argument to the GATE, never to the tool."""
    from omnisim_bridges.bridge_base import serve_tool

    gate.register_tools([tool("drive_forward", {"distance": NUM})],
                        surface="mobile")
    seen = {}
    code, payload = serve_tool(
        "drive_forward",
        {"tool": "drive_forward", "id": "r1", "request_id": "r1",
         "utterance": "drive forward 2 metres", "distance": 2.0},
        lambda args: seen.update(args) or {"accepted": True},
        surface="mobile")
    assert code == 200, payload
    assert seen == {"distance": 2.0}


def test_a_nonzero_control_error_is_not_a_failure():
    """A successful motion must not come back as `status: "err"`.

    `error` in a motion result is the CONTROL error: a signed float, the gap
    between what was commanded and what the robot achieved. Measured live on
    the Husky, 2026-09-22: `drive_forward {distance: 1.0}` returned
    `achieved: 0.9977, error: -0.00228, settled: true` -- and the handler
    reported `"status": "err"`, because a non-zero float is truthy. Only a
    control error of EXACTLY 0.0 would have been reported as success, so every
    real motion looked like a failure to an agent driving through `/tool`.

    The distinction is the TYPE, not the truthiness: a real failure is a
    string. This is the same trap AGENTS.md records, the one that once printed
    "I could not stop: 4.3e-11" after a perfect stop.
    """
    from omnisim_bridges.bridge_base import serve_tool

    settled = {"accepted": True, "commanded": 1.0, "achieved": 0.9977114200592041,
               "error": -0.0022885799407958984, "settled": True, "timed_out": False}
    code, payload = serve_tool("drive_forward", {"tool": "drive_forward", "distance": 1.0},
                               lambda args: settled, surface="mobile", registered=True)
    assert code == 200
    assert payload["status"] == "ok", (
        "a settled motion with 2.3 mm of control error was reported as a failure")
    assert payload["result"]["error"] == -0.0022885799407958984, "the float must survive intact"

    # A control error of exactly zero is still a success, and still not special.
    code, payload = serve_tool("stop_robot", {"tool": "stop_robot"},
                               lambda args: {"accepted": True, "error": 0.0, "settled": True},
                               surface="mobile", registered=True)
    assert code == 200 and payload["status"] == "ok"

    # A STRING error is a real failure and must still be reported as one.
    code, payload = serve_tool("drive_forward", {"tool": "drive_forward", "distance": 1.0},
                               lambda args: {"error": "motor offline"},
                               surface="mobile", registered=True)
    assert code == 200 and payload["status"] == "err"

    # ...including the one spelling that carries its own status code. (It
    # carries a distance: the gate requires one since 2026-09-23, and a
    # distance-less drive would be refused 400 before the handler ran.)
    code, payload = serve_tool("drive_forward", {"tool": "drive_forward", "distance": 1.0},
                               lambda args: {"error": "unknown tool 'drive_forward'"},
                               surface="mobile", registered=True)
    assert code == 404 and payload["status"] == "err"


def test_serve_tool_refuses_an_unregistered_tool_without_asking_the_gate():
    """Existence is the BRIDGE's question -- the gate drops `unknown_tool`."""
    from omnisim_bridges.bridge_base import serve_tool

    code, payload = serve_tool("no_such_tool", {"tool": "no_such_tool"},
                               lambda args: {"ran": True},
                               surface="mobile", registered=False)
    assert code == 503 and payload["error"] == "tool_not_registered"


def test_serve_tool_refuses_an_implausible_magnitude():
    """The rail that needs no sentence, through the one shared handler."""
    from omnisim_bridges.bridge_base import serve_tool

    ran = []
    code, payload = serve_tool(
        "drive_forward", {"tool": "drive_forward", "distance": 300.0},
        lambda args: ran.append(args) or {"accepted": True}, surface="mobile")
    assert code == 400 and payload["error"] == "refused_by_gate"
    assert "implausible" in payload["message"]
    assert ran == [], "a refused call still reached the robot"
