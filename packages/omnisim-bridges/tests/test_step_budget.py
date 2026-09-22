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

"""The shipped bridges' waits, counted in SIM STEPS (Track D1 step 3).

⚠️ THIS FILE DRIVES THE SHIPPED CODE, not a copy of it. Each wait loop is
lifted out of its controller with the AST trick `test_parser_first_wiring`
uses -- the controllers import `omnisim` at module scope and cannot be
imported off a running engine -- and then run against a fake clock. A test
that re-implemented the loop would prove only that the test is
self-consistent.

WHAT IT PINS, AND WHY IT MATTERS. A wall-clock wait gives the robot N
seconds of the OPERATOR's time whatever the sim is doing with them: on a
machine at 0.4x realtime a 10 s budget bought 4 s of robot, the motion was
cut off mid-flight, and the caller was told it "timed out" -- a verdict
about the machine's load, reported as a fact about the robot. And the
inverse failure is worse: a world that is not stepping at all would expire
every budget instantly and tell an agent to reissue commands into a frozen
sim. A stall is not a timeout, and the two are separate answers here.
"""

from __future__ import annotations

import ast
import pathlib
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any, Dict

import pytest

PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges.bridge_base import SimClock       # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
CONTROLLERS = ROOT / "projects/samples/demos/controllers"
MOBILE = CONTROLLERS / "omnilink_mobile_bridge/omnilink_mobile_bridge.py"
ARM = CONTROLLERS / "omnilink_arm_bridge/omnilink_arm_bridge.py"
QUAD = CONTROLLERS / "omnilink_quadruped_bridge/omnilink_quadruped_bridge.py"
MAVIC = CONTROLLERS / "mavic_omnilink_bridge/mavic_omnilink_bridge.py"
ALL_BRIDGES = {"mobile": MOBILE, "arm": ARM, "quadruped": QUAD,
               "drone": MAVIC}


def load_method(path: pathlib.Path, name: str, namespace: Dict[str, Any]):
    """Exec one function/method out of a controller, with no imports."""
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


class _FakeMobile:
    """Exactly the attributes `_await_completion` reads."""

    WAIT_MAX_S = 30.0
    WAIT_POLL_S = 0.005

    def __init__(self, dt_s: float = 0.032) -> None:
        self.clock = SimClock(dt_s=dt_s)
        self.lock = threading.RLock()
        self.motion_seq = 1
        self.last_completion = None
        self.bridge_instance_id = "test-instance"
        self.last_tick_at = time.time()

    def tick(self, n: int = 1) -> None:
        """What the sim thread would have done."""
        for _ in range(n):
            self.clock.tick()
            self.last_tick_at = time.time()


def _await(bridge, seq: int, budget_s: float):
    fn = load_method(MOBILE, "_await_completion", {"time": time})
    return fn(bridge, seq, budget_s)


# ── The verdict comes from steps ────────────────────────────────────

def test_a_wait_that_is_answered_returns_the_measurement() -> None:
    b = _FakeMobile()
    b.last_completion = {"seq": 1, "verb": "drive_forward", "commanded": 2.0,
                         "achieved": 1.9977, "error": -0.0023,
                         "settled": True, "timed_out": False}
    out = _await(b, 1, 5.0)
    assert out["achieved"] == 1.9977
    assert out["settled"] is True


def test_a_wall_clock_second_does_not_expire_a_step_budget() -> None:
    # THE TRUNCATION BUG, pinned. A tenth-of-a-second budget is three steps
    # of a 32 ms world. Sleeping past it in WALL time must not end the wait:
    # the sim has not moved, so the robot has not been given its time.
    b = _FakeMobile()
    start = time.time()

    def _answer_late():
        time.sleep(0.25)          # far past the wall deadline of the old code
        b.tick(3)                 # ... and only NOW does the sim advance
        b.last_completion = {"seq": 1, "verb": "turn", "achieved": 1.57,
                             "settled": True, "timed_out": False}

    threading.Thread(target=_answer_late, daemon=True).start()
    out = _await(b, 1, 0.1)
    assert out.get("timed_out") is not True, (
        "a wall-clock second expired a budget measured in sim steps")
    assert out["achieved"] == 1.57
    assert (time.time() - start) > 0.2


def test_a_budget_that_really_runs_out_reports_a_timeout() -> None:
    # The other direction: the sim IS stepping, the motion never reports,
    # and that is a genuine timeout. `achieved` is null -- never the
    # commanded value standing in for a measurement nobody made.
    b = _FakeMobile()

    def _step_on():
        for _ in range(40):
            time.sleep(0.002)
            b.tick()

    threading.Thread(target=_step_on, daemon=True).start()
    out = _await(b, 1, 0.5)                    # ~16 steps at 32 ms
    assert out["timed_out"] is True
    assert out["achieved"] is None
    assert out["steps"] >= 16
    assert "sim_time_start" in out and "sim_time_end" in out


def test_a_frozen_world_reports_a_stall_not_a_timeout() -> None:
    # ⚠️ TELLING THESE APART IS THE POINT. An agent told "timed out" will
    # reissue the command; an agent told the world stopped stepping will
    # not. The step budget alone cannot end this wait -- the counter never
    # moves -- so the wall clock keeps exactly this one job.
    b = _FakeMobile()
    b.last_tick_at = time.time() - 30.0        # the sim thread died long ago
    out = _await(b, 1, 5.0)
    assert out.get("stalled") is True
    assert out["timed_out"] is False
    assert out["achieved"] is None
    assert "not a motion that failed" in out["note"]


class _FakeArm:
    """Exactly the attributes the ARM's `_await_completion` reads."""

    WAIT_MAX_S = 50.0
    WAIT_POLL_S = 0.02
    WAIT_STALL_S = 2.0

    def __init__(self, dt_s: float = 0.032) -> None:
        self.clock = SimClock(dt_s=dt_s)
        self.lock = threading.RLock()
        self.motion_seq = 1
        self.last_completion = None
        self.last_tick_at = time.time()

    def tick(self, n: int = 1) -> None:
        for _ in range(n):
            self.clock.tick()
            self.last_tick_at = time.time()


# Every bridge whose wait can TELL a stall from a timeout, and the fake that
# supplies exactly the attributes its loop reads. The quadruped and the
# flying bridge are deliberately absent: neither has a blocking completion
# wait at all, so neither can distinguish the condition, and inventing the
# field for them would publish `stalled: false` about a question they never
# asked. An absent field is honest; a false one is not.
STALL_AWARE = {"mobile": (MOBILE, _FakeMobile), "arm": (ARM, _FakeArm)}


@pytest.mark.parametrize("name", sorted(STALL_AWARE))
def test_a_stall_has_exactly_one_name_on_the_wire(name) -> None:
    """⚠️ ONE CONCEPT, ONE FIELD, ON EVERY BRIDGE THAT CAN SEE IT.

    PROTOCOL.md §5.4.1 rule 8: a frozen world can never expire a step
    budget, so a wait that discovers the sim has stopped stepping answers
    `stalled: true` with `timed_out: false` -- an agent told "timed out"
    reissues the command into a simulation that is not running.

    The rule was obeyed by both shipped implementations and SPELLED
    DIFFERENTLY by each: the mobile bridge returned `stalled: true`, the arm
    `measurable: false`. Two names for one concept on one wire contract is
    how a client comes to handle one robot class and silently not the other
    -- it branches on the name it met first and the other robot's stall
    reads as an ordinary failure. This test fails if EITHER bridge reverts
    to its own spelling.
    """
    path, factory = STALL_AWARE[name]
    bridge = factory()
    bridge.last_tick_at = time.time() - 30.0      # the sim thread died long ago
    fn = load_method(path, "_await_completion", {"time": time})
    out = fn(bridge, 1, 5.0)

    assert out.get("stalled") is True, (
        f"the {name} bridge does not report a frozen world as `stalled`; it "
        f"answered {sorted(out)} -- PROTOCOL §5.4.1 rule 8 names ONE field "
        f"and a client cannot be asked to learn a second spelling per robot "
        f"class")
    assert out.get("timed_out") is False, (
        f"the {name} bridge called a stall a timeout; an agent told that "
        f"reissues the command into a sim that is not running")
    assert out.get("achieved") is None
    assert out.get("settled") is False
    # A stall is a wait that did not complete, so it carries the sim window
    # like every other measured result (rule 7) -- zero steps, which is the
    # measurement that says the world never moved.
    assert out.get("steps") == 0
    assert "sim_time_start" in out and "sim_time_end" in out


def test_the_arm_keeps_measurable_as_an_alias_beside_stalled() -> None:
    # `measurable` is NOT deleted: this bridge also emits it for a genuinely
    # different unmeasurable -- a mirroring hardware link, where the world is
    # stepping perfectly well and the vendor controller owns the trajectory.
    # Anything already reading it keeps working; what changed is that it is
    # no longer the ONLY signal that the sim froze.
    arm = _FakeArm()
    arm.last_tick_at = time.time() - 30.0
    fn = load_method(ARM, "_await_completion", {"time": time})
    out = fn(arm, 1, 5.0)
    assert out.get("measurable") is False
    assert out.get("stalled") is True
    src = ARM.read_text(encoding="utf-8")
    assert '"measurable": False,' in src, (
        "the mirroring-hardware branch's `measurable` was removed with the "
        "stall's; a client reading it for a real arm now sees nothing")


@pytest.mark.parametrize("name", ["quadruped", "drone"])
def test_the_bridges_that_cannot_see_a_stall_do_not_claim_one(name) -> None:
    # An absent field is honest. `stalled: false` from a bridge with no
    # completion wait would be a measurement nobody made -- the same defect
    # as reporting a commanded value under a measured key (§5.4.1 rule 1).
    #
    # ⚠️ This also guards a NAME COLLISION. The flying bridge already has a
    # "stall" of its own -- `stall_verdict` / `stall_step`, an aircraft
    # making no progress towards a goto target -- which is a fact about the
    # ROBOT while §5.4.1's `stalled` is a fact about the SIMULATOR. If that
    # one ever reaches the wire it needs its own name, or a client draining
    # both streams will read a wedged drone as a frozen world.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert '"stalled"' not in src, (
        f"{name} puts a `stalled` field on the wire but has no blocking "
        f"completion wait to measure §5.4.1's stall with -- if this is the "
        f"drone's no-progress stall, give it a different key")


def test_a_superseded_motion_still_short_circuits_the_wait() -> None:
    b = _FakeMobile()
    b.motion_seq = 2                           # a later command took the slot
    out = _await(b, 1, 5.0)
    assert out["superseded"] is True
    assert out["achieved"] is None


def test_the_measured_window_rides_on_the_reply() -> None:
    b = _FakeMobile()

    def _step_on():
        for _ in range(40):
            time.sleep(0.002)
            b.tick()

    threading.Thread(target=_step_on, daemon=True).start()
    out = _await(b, 1, 0.3)
    assert out["sim_time_end"] >= out["sim_time_start"]
    assert out["steps"] == pytest.approx(
        round((out["sim_time_end"] - out["sim_time_start"]) / 0.032), abs=1)


# ── The shipped sources, pinned ─────────────────────────────────────

@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_reports_last_tick_at_in_sim_seconds(name) -> None:
    # PROTOCOL.md §5.3 has always shown `last_tick_at` in SIM seconds. Every
    # bridge sent `time.time()` -- 1.7 billion where the spec's example
    # reads 12.448 -- so a client differencing it against `sim_time` got the
    # age of the Unix epoch.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert '"last_tick_at": self.sim_time' in src or \
           '"last_tick_at": self.sim_time,' in src, \
        f"{name} does not report last_tick_at in sim seconds"
    assert '"wall_time"' in src, f"{name} dropped the wall clock entirely"


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_no_bridge_reports_the_wall_clock_as_last_tick_at(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert '"last_tick_at": time.time()' not in src
    assert '"last_tick_at": self.last_tick_at' not in src
    assert '"last_tick_at": now' not in src


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_serves_the_event_stream(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "serve_events(" in src, f"{name} serves no GET /events"
    assert '"events"' in src, f"{name} does not report /state.events"


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_publishes_its_ungated_paths(name) -> None:
    # PROTOCOL.md §5.2.1: `ungated_paths` is the LOAD-BEARING half.
    # Publishing `present: true` without it invites the reading "everything
    # here is vetted", which is false of every bridge in this tree.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "safety_gate_block(" in src, f"{name} publishes no safety_gate"
    assert "ungated_paths" in src, f"{name} omits ungated_paths"


@pytest.mark.parametrize("name", ["mobile", "arm", "quadruped"])
def test_the_ground_bridges_file_gate_refusals_as_events(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "bridge=bridge" in src, (
        f"{name} does not hand its ring to serve_tool, so a gate refusal "
        f"on /tool is filed nowhere")


def test_the_drone_files_gate_refusals_against_its_state_object() -> None:
    src = MAVIC.read_text(encoding="utf-8")
    assert "bridge=state" in src


@pytest.mark.parametrize("name", ["mobile", "arm", "quadruped"])
def test_the_ground_bridges_hold_through_the_lease_not_a_bare_step(name) -> None:
    # ⚠️ Never collapse the main loop back to a bare `robot.step()` while a
    # lease can be live: a step against a paused engine blocks, and while it
    # is blocked nobody can deliver the resume.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "hold.step_or_hold(" in src, f"{name} does not honour a hold"
    assert "while robot.step(timestep) != -1:" not in src, (
        f"{name} still has the unconditional main-loop step")


@pytest.mark.parametrize("name", ["mobile", "arm", "quadruped"])
def test_stop_always_runs_under_a_hold(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert 'request_release("stop_robot")' in src, (
        f"{name}'s stop cannot lift a lockstep hold, so it cannot reach the "
        f"actuators while the world is frozen")


def test_the_drone_declares_lockstep_unsupported_rather_than_faking_it() -> None:
    # A held drone is a drone whose rotors stop being commanded. Saying so
    # is the honest answer; wiring a hold that drops it out of the sky is
    # not, and neither is silently ignoring the variable.
    src = MAVIC.read_text(encoding="utf-8")
    assert '"lockstep"' in src and '"supported": False' in src


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_no_bridge_reads_the_engine_clock_from_get_state(name) -> None:
    # A supervisor read from an HTTP thread stalls the controller<->sim step
    # exchange; a couple per second dragged the warehouse demo to ~0.2x
    # realtime. `get_state` / `snapshot` runs on an HTTP worker.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        if node.name not in ("get_state", "snapshot", "get_state_for_query"):
            continue
        body = ast.get_source_segment(src, node) or ""
        assert "getTime()" not in body, (
            f"{name}.{node.name} calls the engine clock from an HTTP thread")


@pytest.mark.parametrize("name", ["mobile", "arm"])
def test_the_wait_loops_take_a_step_budget(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "_await_completion")
    body = ast.get_source_segment(src, fn) or ""
    assert "clock.budget(" in body, f"{name}'s wait is not a step budget"
    assert "budget.expired()" in body


def test_the_mobile_settle_windows_are_step_budgets() -> None:
    src = MOBILE.read_text(encoding="utf-8")
    tree = ast.parse(src)
    for target in ("_stop_measure_rest", "_measure_body_rates"):
        fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == target)
        body = ast.get_source_segment(src, fn) or ""
        assert "clock.budget(" in body, f"{target} still waits on a wall clock"


@pytest.mark.parametrize("name", ["mobile", "arm"])
def test_every_measured_result_carries_the_sim_window(name) -> None:
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "_record_completion")
    body = ast.get_source_segment(src, fn) or ""
    for field in ("sim_time_start", "sim_time_end", "steps"):
        assert field in body, f"{name}'s completion record omits {field}"
    assert "emit_motion_outcome" in body, (
        f"{name} does not file motion.timed_out / motion.unsettled")


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_stamps_via_on_a_prompt_reply(name) -> None:
    # PROTOCOL.md §5.7.2 / plan D3: the sweep records WHICH STAGE answered
    # and refuses to guess -- a missing `via` reads as null, the two-door
    # comparison becomes undetermined, and the run fails by design.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "stamp_via(" in src, f"{name} can return a 200 from /prompt with no via"


# ── The relay seam (plan D4 step 5 / D5) ────────────────────────────

@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_hands_the_relay_its_own_handle(name) -> None:
    # ⚠️ WITHOUT THIS THE WHOLE EVENT LOOP IS DEAD CODE, silently: the relay
    # owns the wake dispatcher and the presence beat and reads the bridge
    # through one handle, so no handle means no ring, no wakes, and a beat
    # that reports a held or faulted robot as a healthy one. "Nothing woke
    # me" and "nothing happened" look identical from outside.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "attach_relay(" in src, f"{name} never attaches its relay"


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_every_bridge_opens_the_prompt_door_on_its_profile(name) -> None:
    # `prompt_callback_url` + `surface` is THE SWITCH: until a bridge sends
    # them the platform cannot know this robot serves a sentence door, so it
    # keeps using /api/chat + /tool and the parser-first handoff is dead
    # code on the live tree.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    assert "profile_extras(" in src, f"{name} pushes no prompt door"


def test_profile_extras_agrees_with_the_capability_block() -> None:
    # Two hand-written copies of "which paths are vetted" is exactly how the
    # two gates drifted. One source, both consumers.
    from omnisim_bridges.bridge_base import (
        _capabilities_body, profile_extras, safety_gate_block,
    )

    class _B:
        robot_id = "husky"
        surface = "mobile"
        GATED_PATHS = ["/prompt", "/tool"]
        UNGATED_PATHS = ["/drive_forward", "/turn"]
        capabilities = {"actions": ["drive_forward", "turn"]}

    b = _B()
    extras = profile_extras(b, http_port=8765, surface="mobile")
    assert extras["prompt_callback_url"] == "http://127.0.0.1:8765/prompt"
    assert extras["surface"] == "mobile"
    assert "prompt" in extras["actions"]
    assert extras["safety_gate"]["ungated_paths"] == ["/drive_forward", "/turn"]
    assert extras["safety_gate"] == safety_gate_block(
        "mobile", gated_paths=b.GATED_PATHS, ungated_paths=b.UNGATED_PATHS)
    # And the block a client reads from /capabilities is the same object's
    # worth of truth.
    caps = _capabilities_body(b, "mobile")
    assert caps["safety_gate"]["ungated_paths"] == \
        extras["safety_gate"]["ungated_paths"]


def test_attach_relay_survives_a_relay_that_has_none_of_the_methods() -> None:
    # An older package's relay has no attach_bridge; a bridge must still
    # come up, reporting less rather than crashing at boot.
    from omnisim_bridges.bridge_base import attach_relay

    class _OldRelay:
        pass

    r = _OldRelay()
    assert attach_relay(r, object(), event_sink=lambda k, p: None) is r
    assert attach_relay(None, object()) is None


def test_attach_relay_passes_all_three_handles() -> None:
    from omnisim_bridges.bridge_base import attach_relay
    seen = {}

    class _Relay:
        def attach_bridge(self, b):
            seen["bridge"] = b

        def set_event_sink(self, s):
            seen["sink"] = s

        def set_window_raiser(self, r):
            seen["raiser"] = r

    bridge = object()
    sink = lambda k, p: None          # noqa: E731
    raiser = lambda: None             # noqa: E731
    attach_relay(_Relay(), bridge, event_sink=sink, window_raiser=raiser)
    assert seen == {"bridge": bridge, "sink": sink, "raiser": raiser}


@pytest.mark.parametrize("name", sorted(ALL_BRIDGES))
def test_the_window_raiser_does_not_touch_the_robot_api(name) -> None:
    # ⚠️ It is invoked from the PRESENCE thread. The controller API is not
    # thread-safe and a couple of threaded reads per second have been
    # measured dragging the sim to ~0.2x realtime. `queue_window` appends
    # under the bridge's own lock and the SIM THREAD drains the outbox, so
    # the marshalling is the outbox itself.
    src = ALL_BRIDGES[name].read_text(encoding="utf-8")
    idx = src.find("window_raiser=")
    assert idx > 0, f"{name} registers no window raiser"
    snippet = src[idx:idx + 300]
    assert "queue_window(" in snippet
    for forbidden in ("robot.", "wwiSendText", "getSelf(", "simulationSet"):
        assert forbidden not in snippet, (
            f"{name}'s window raiser touches the Robot API from the "
            f"presence thread")


# ── The env hatches, value-parsed ───────────────────────────────────

def test_the_new_hatches_are_value_parsed() -> None:
    # `=0` must mean OFF. Every presence-gated flag in this tree has
    # eventually surprised somebody who set one to 0 expecting that.
    from omnisim_bridges.hold import env_flag
    import os
    # OMNISIM_TEST_HATCH is a TEST SENTINEL, not a product hatch: a throwaway
    # name used to prove every new boolean hatch is VALUE-parsed, so that "=0"
    # means off rather than merely being present. It is set and popped inside
    # this test and must never be read by shipped code.
    os.environ["OMNISIM_TEST_HATCH"] = "0"
    try:
        assert env_flag("OMNISIM_TEST_HATCH", True) is False
        os.environ["OMNISIM_TEST_HATCH"] = "1"
        assert env_flag("OMNISIM_TEST_HATCH", False) is True
    finally:
        os.environ.pop("OMNISIM_TEST_HATCH", None)


def test_the_bridges_read_the_lockstep_hatch_through_one_helper() -> None:
    # One spelling of "is lockstep on", so a bridge cannot invent a second.
    from omnisim_bridges import hold as _hold
    src = (PKG_SRC / "omnisim_bridges" / "hold.py").read_text(encoding="utf-8")
    assert src.count('"OMNISIM_BRIDGE_LOCKSTEP"') == 1
    assert _hold.lockstep_enabled.__module__ == "omnisim_bridges.hold"
    # Naming the variable in a message or a capability block is fine and
    # useful; READING it is what must happen in exactly one place, or a
    # bridge grows its own idea of what "on" means (presence-gated, say).
    for path in ALL_BRIDGES.values():
        body = path.read_text(encoding="utf-8")
        for spelling in ('environ.get("OMNISIM_BRIDGE_LOCKSTEP"',
                         'environ["OMNISIM_BRIDGE_LOCKSTEP"]',
                         "getenv(\"OMNISIM_BRIDGE_LOCKSTEP\""):
            assert spelling not in body, (
                f"{path.name} reads the hatch itself instead of asking "
                f"hold.lockstep_enabled()")
