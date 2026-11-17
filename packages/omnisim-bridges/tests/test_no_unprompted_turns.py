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

"""A restarted robot does not move until someone asks it to.

The defect (2026-10-17, installed v9.2.5, arm bridge driving the
WIRobotics ALLEX arm, "take 2"): after an engine restart the relay was
woken by `joint.limit_hit` -- the relaxed fingers come to rest ON their
stops in the first sim second -- and short-term memory had just restored
the previous session's operator sentence. The wake turn handed the model
that history, and the arm executed the whole previous task with no new
prompt. Three rules, pinned here:

1. Events during startup / settle never start a turn.
2. A sentence restored from a previous session is never executed without
   an operator turn in THIS session: no wake fires before one, and the
   restored window carries an explicit session-boundary instruction.
3. Event-triggered turns are OFF unless `OMNILINK_EVENT_WAKE=1`.

Nothing here needs an engine, a key or a network: the OmniLink client is a
fake, and so is the event ring (the real ring is used where noted).
"""

from __future__ import annotations

import threading
import time

import pytest

from omnisim_bridges import relay as relay_mod
from omnisim_bridges.events import EventRing, JointLimitDetector
from omnisim_bridges.relay import OmniLinkRelay
from omnisim_bridges.tool import Tool


PREVIOUS_SENTENCE = ("Hover over the red tag, then the blue tag, then the "
                     "green tag, and come back home.")


class FakeClient:
    """Answers get_memory with a previous session's conversation and records
    every chat call. A chat call that happens at all is the failure in the
    take-2 test; elsewhere it returns plain text."""

    def __init__(self, memory=None, reply=None):
        self.memory = list(memory or [])
        self.chats = []
        self.reply = reply or {"ok": True, "text": "ok", "toolCalls": []}
        self.called = threading.Event()

    def get_memory(self, agent):
        return list(self.memory)

    def set_memory(self, agent, blob):
        self.memory = list(blob)

    def chat(self, **kwargs):
        self.chats.append(kwargs)
        self.called.set()
        return dict(self.reply)


def previous_session():
    """What the platform hands back after take 1: the operator's sentence
    and the robot's reply (Gemini shape, as `_to_memory_format` writes)."""
    return [
        {"role": "user", "parts": [{"text": PREVIOUS_SENTENCE}]},
        {"role": "model", "parts": [{"text": "On it: red, blue, green."}]},
        {"role": "model", "parts": [{"text": "Done. Back at home."}]},
    ]


class Bridge:
    robot_id = "allex_right_arm"
    model = "ALLEX"
    world = "allex_tags"
    fault = None
    held = False

    def __init__(self, ring):
        self.events = ring
        self.sim_time = 0.0


@pytest.fixture()
def env(monkeypatch, tmp_path):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    monkeypatch.setenv("OMNILINK_JOURNAL_SYNC", "0")
    monkeypatch.delenv("OMNILINK_EVENT_WAKE", raising=False)
    return monkeypatch


def make_relay(monkeypatch, client, *, memory=True, moved=None):
    monkeypatch.setattr(relay_mod, "OmniLinkClient", lambda **kw: client)
    moved = moved if moved is not None else []
    tools = [Tool("pick", "move the arm",
                  {"type": "object", "properties": {}},
                  lambda a: (moved.append(a) or {"ok": True}))]
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="NoReplayProbe", main_task="test",
        tools=tools, usage_enabled=False, memory_enabled=memory,
        surface="arm")
    return relay, moved


def recorder(relay):
    dispatched = []

    def _record(text, on_event, lane="main", origin="operator"):
        if origin != "wake":
            relay.note_operator_turn()
        dispatched.append((origin, text))
        return relay_mod.DispatchHandle()

    relay.dispatch_async = _record            # type: ignore[assignment]
    return dispatched


# ── 3. Off by default ────────────────────────────────────────────────

def test_event_wake_is_off_by_default(env) -> None:
    relay, _ = make_relay(env, FakeClient(), memory=False)
    try:
        assert relay.event_wake_stats()["enabled"] is False
    finally:
        relay.close()


def test_default_relay_never_wakes_even_armed_and_settled(env) -> None:
    """Off means off: an operator turn and a quiet, settled scene still do
    not make an event buy a turn."""
    relay, _ = make_relay(env, FakeClient(), memory=False)
    ring = EventRing(robot="allex")
    relay.attach_bridge(Bridge(ring))
    dispatched = recorder(relay)
    try:
        relay.poll_events_once(now=0.0)
        relay.note_operator_turn()
        ring.emit("joint.limit_hit", sim_time=60.0, step=1, joint="j1")
        assert relay.poll_events_once(now=100.0) == 0
        assert dispatched == []
        # ...and the event is still READABLE: off is a budget, not a blindfold.
        page = relay.tools["get_events"].dispatch({"limit": 5})
        assert [e["type"] for e in page["events"]] == ["joint.limit_hit"]
    finally:
        relay.close()


def test_opt_in_is_documented_hatch(env) -> None:
    env.setenv("OMNILINK_EVENT_WAKE", "1")
    relay, _ = make_relay(env, FakeClient(), memory=False)
    try:
        assert relay.event_wake_stats()["enabled"] is True
    finally:
        relay.close()


# ── 2. No wake before this session's first operator turn ─────────────

@pytest.fixture()
def opted_in(env):
    env.setenv("OMNILINK_EVENT_WAKE", "1")
    relay, moved = make_relay(env, FakeClient(), memory=False)
    ring = EventRing(robot="allex")
    relay.attach_bridge(Bridge(ring))
    dispatched = recorder(relay)
    try:
        yield relay, ring, dispatched
    finally:
        relay.close()


def test_no_wake_before_any_operator_turn(opted_in) -> None:
    relay, ring, dispatched = opted_in
    relay.poll_events_once(now=0.0)
    ring.emit("joint.limit_hit", sim_time=60.0, step=1, joint="j1")
    assert relay.poll_events_once(now=100.0) == 0
    assert dispatched == []
    stats = relay.event_wake_stats()
    assert stats["armed"] is False
    assert stats["held_back"] == {"no_operator_turn": 1}


def test_an_event_from_before_the_operator_turn_never_wakes_later(
        opted_in) -> None:
    """The race the arming rule must survive: the event is filed, THEN the
    operator speaks, THEN the poller looks. That event predates the turn
    and must not buy one."""
    relay, ring, dispatched = opted_in
    relay.poll_events_once(now=0.0)
    ring.emit("joint.limit_hit", sim_time=60.0, step=1, joint="j1")
    relay.note_operator_turn()
    assert relay.poll_events_once(now=100.0) == 0
    assert relay.event_wake_stats()["held_back"] == {"before_operator_turn": 1}
    # And it is consumed, not deferred: later polls do not find it again.
    assert relay.poll_events_once(now=200.0) == 0
    assert dispatched == []
    # A NEW event after the turn is the opt-in working as designed.
    ring.emit("motion.timed_out", sim_time=61.0, step=2, tool="pick")
    assert relay.poll_events_once(now=300.0) == 1
    assert dispatched[-1][0] == "wake"


def test_dispatch_async_arms_and_a_wake_does_not(env) -> None:
    """Arming is wired into the relay itself, so no bridge can forget it --
    and a wake turn must not arm its own successors."""
    env.setenv("OMNILINK_EVENT_WAKE", "1")
    relay, _ = make_relay(env, FakeClient(), memory=False)
    try:
        relay.dispatch_async("[Event] x", lambda k, p: None, origin="wake")
        assert relay.event_wake_stats()["operator_turns"] == 0
        assert relay.event_wake_stats()["armed"] is False
        relay.dispatch_async("hello", lambda k, p: None)
        assert relay.event_wake_stats()["operator_turns"] == 1
        assert relay.event_wake_stats()["armed"] is True
    finally:
        relay.close()


def test_a_parser_answered_sentence_also_arms(opted_in) -> None:
    relay, ring, dispatched = opted_in
    relay.note_parser_turn("open the gripper", "Gripper open.")
    assert relay.event_wake_stats()["armed"] is True


def test_armed_before_the_ring_existed_resolves_conservatively(env) -> None:
    env.setenv("OMNILINK_EVENT_WAKE", "1")
    relay, _ = make_relay(env, FakeClient(), memory=False)
    dispatched = recorder(relay)
    try:
        relay.note_operator_turn()            # no bridge attached yet
        ring = EventRing(robot="allex")
        ring.emit("joint.limit_hit", sim_time=60.0, step=1, joint="j0")
        relay.attach_bridge(Bridge(ring))
        relay.poll_events_once(now=0.0)       # primes
        ring.emit("joint.limit_hit", sim_time=61.0, step=2, joint="j1")
        assert relay.poll_events_once(now=100.0) == 0, (
            "an event of unknown age relative to the turn must not wake")
        ring.emit("joint.limit_hit", sim_time=62.0, step=3, joint="j2")
        assert relay.poll_events_once(now=200.0) == 1
    finally:
        relay.close()


# ── 1. Startup / settle never wakes ──────────────────────────────────

def test_an_event_in_the_first_sim_seconds_never_wakes(opted_in) -> None:
    relay, ring, dispatched = opted_in
    relay.poll_events_once(now=0.0)
    relay.note_operator_turn()
    ring.emit("joint.limit_hit", sim_time=0.3, step=1, joint="finger_1")
    assert relay.poll_events_once(now=100.0) == 0
    assert relay.event_wake_stats()["held_back"] == {"startup_settle": 1}
    assert dispatched == []


def test_an_event_right_after_priming_never_wakes(opted_in) -> None:
    """Wall-clock settle: covers a bridge (re)started into a world whose
    clock is already running, where the sim-time rule cannot see startup,
    and an event that carries no sim time at all."""
    relay, ring, dispatched = opted_in
    relay.poll_events_once(now=0.0)
    relay.note_operator_turn()
    ring.emit("joint.limit_hit", sim_time=600.0, step=1, joint="finger_1")
    assert relay.poll_events_once(
        now=relay_mod.WAKE_SETTLE_WALL_S - 0.5) == 0
    assert relay.event_wake_stats()["held_back"] == {"startup_settle": 1}
    assert relay._wake_hold_reason(
        {"type": "joint.limit_hit", "seq": 99, "sim_time": None},
        relay_mod.WAKE_SETTLE_WALL_S - 0.5) == "startup_settle"
    ring.emit("joint.limit_hit", sim_time=601.0, step=2, joint="finger_2")
    assert relay.poll_events_once(now=relay_mod.WAKE_SETTLE_WALL_S + 0.5) == 1


def test_hand_at_rest_on_its_stops_is_held_through_the_real_detector(
        opted_in) -> None:
    """The ALLEX case through the REAL JointLimitDetector: fingers settle
    onto their lower stops in the first sim second and stay there. The
    detector fires once per joint (hysteresis); none of it is a wake."""
    relay, ring, dispatched = opted_in
    relay.poll_events_once(now=0.0)
    relay.note_operator_turn()
    det = JointLimitDetector(ring)
    names = [f"finger_{i}" for i in range(4)]
    limits = [(0.0, 1.5)] * 4
    for step in range(1, 250):                # 1 s at 4 ms
        sim_t = step * 0.004
        q = [max(0.0, 0.3 - sim_t)] * 4       # relaxing onto the stop
        det.update(names, q, limits, sim_time=sim_t, step=step)
        relay.poll_events_once(now=100.0 + step)
    assert ring.total == 4, "one limit_hit per finger, as observed"
    assert dispatched == []
    assert relay.event_wake_stats()["held_back"] == {"startup_settle": 4}


# ── The take-2 reproduction, end to end ──────────────────────────────

@pytest.mark.parametrize("wake_env", [None, "1"])
def test_take2_restart_with_restored_memory_does_not_move_the_arm(
        env, wake_env) -> None:
    """The observed sequence, with the REAL dispatcher and worker: a relay
    restores the previous session's sentence, the bridge files a
    `joint.limit_hit` while the hand settles, and nobody types anything.
    The model must never be called and the arm must never be commanded --
    with the default, AND with the wake opted in."""
    if wake_env is not None:
        env.setenv("OMNILINK_EVENT_WAKE", wake_env)
    client = FakeClient(
        memory=previous_session(),
        # If a turn ever ran, the model would do exactly what take 2 did.
        reply={"ok": True, "text": "",
               "toolCalls": [{"id": "c1", "name": "pick", "arguments": {}}]})
    relay, moved = make_relay(env, client, memory=True)
    try:
        assert any(m.get("content") == PREVIOUS_SENTENCE
                   for m in relay.history), "the restore is part of the repro"
        ring = EventRing(robot="allex")
        relay.attach_bridge(Bridge(ring))
        relay.poll_events_once(now=0.0)
        ring.emit("joint.limit_hit", sim_time=0.3, step=75,
                  joint="R_Index_MCP", position=0.0, limit=0.0, side="min")
        for i in range(1, 40):
            relay.poll_events_once(now=float(i))
        # Then the scene sits idle, settled, well past both settle windows.
        ring.emit("joint.limit_hit", sim_time=30.0, step=7500,
                  joint="R_Thumb_CMC", position=0.0, limit=0.0, side="min")
        relay.poll_events_once(now=500.0)
        time.sleep(0.3)                       # give a stray turn time to run
        assert client.chats == [], "no model turn without a fresh prompt"
        assert moved == [], "the arm must not move on its own after a restart"
        assert relay.event_wake_stats()["wakes"] == 0
    finally:
        relay.close()


# ── The restored window says what it is ──────────────────────────────

def test_restored_history_carries_the_session_boundary_note(env) -> None:
    """Even with a fresh prompt, the model is told that the restored
    sentence is a previous session's and is not to be resumed. Continuity
    of conversation, never re-execution."""
    client = FakeClient(memory=previous_session())
    relay, moved = make_relay(env, client, memory=True)
    try:
        out = relay.dispatch_sync("Hi, are you there?", timeout_s=10)
        assert out.get("response") == "ok"
        si = client.chats[0]["system_instruction"]
        # Per-turn notes travel as liveContext so the cached mainTask never changes.
        assert "SESSION BOUNDARY" not in si["mainTask"]
        task = si["liveContext"]
        assert "SESSION BOUNDARY" in task
        assert "restored from a PREVIOUS session" in task
        assert "Do NOT carry out, repeat, resume or continue" in task
        # The restored sentence is still there for continuity...
        assert any(m.get("content") == PREVIOUS_SENTENCE
                   for m in client.chats[0]["messages"])
        assert moved == []
    finally:
        relay.close()


def test_a_fresh_session_has_no_boundary_note(env) -> None:
    client = FakeClient()
    relay, _ = make_relay(env, client, memory=True)
    try:
        relay.dispatch_sync("Hi", timeout_s=10)
        si = client.chats[0]["system_instruction"]
        assert "SESSION BOUNDARY" not in si["mainTask"]
        assert "SESSION BOUNDARY" not in si.get("liveContext", "")
    finally:
        relay.close()


def test_the_same_words_typed_again_this_session_are_not_restored(env) -> None:
    """Identity, not text: an operator who repeats the old sentence in THIS
    session has asked again, and that copy is theirs to have executed."""
    client = FakeClient(memory=previous_session())
    relay, _ = make_relay(env, client, memory=True)
    try:
        fresh = {"role": "user", "content": PREVIOUS_SENTENCE}
        assert relay._session_boundary_note([fresh]) == ""
        restored = [m for m in relay.history
                    if m.get("content") == PREVIOUS_SENTENCE]
        assert relay._session_boundary_note(restored + [fresh]).startswith(
            "SESSION BOUNDARY: 1 message(s)")
    finally:
        relay.close()
