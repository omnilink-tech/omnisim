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

"""The wake: an event in the sim buys AT MOST one model turn.

A wake is a chat round trip, measured at ~5.6k prompt tokens. The whole
design is therefore about what it REFUSES to spend: only physical or
safety events qualify (decision L4), at most one per 10 s and 30 per
session, and `OMNILINK_EVENT_WAKE=0` turns the whole thing off while
leaving the ring readable.

Every test here drives a FAKE ring, so nothing below needs an engine, a
key or a network. The ring's real implementation is pinned separately in
`test_event_ring.py`; what is pinned HERE is the dispatcher's arithmetic,
which is the part that spends the operator's money.
"""

from __future__ import annotations

import pytest

from omnisim_bridges import relay as relay_mod
from omnisim_bridges.relay import OmniLinkRelay, wake_text
from omnisim_bridges.tool import Tool


# ── A ring that answers `since` the way EventRing does ───────────────

class FakeRing:
    """Minimal stand-in: monotonic seq, cursor-paged, oldest first."""

    def __init__(self) -> None:
        self.events = []
        self._seq = 0
        self.dropped = 0

    def add(self, type: str, *, detail=None, sim_time: float = 1.0) -> dict:
        self._seq += 1
        evt = {"seq": self._seq, "type": type, "sim_time": sim_time,
               "step": self._seq, "robot": "probe", "source": "bridge",
               "detail": detail if detail is not None else {}}
        self.events.append(evt)
        return evt

    def since(self, cursor: int = 0, *, limit: int = 100, types=None) -> dict:
        out = [dict(e) for e in self.events
               if e["seq"] > cursor and (types is None or e["type"] in types)]
        out = out[:limit]
        newest = self.events[-1]["seq"] if self.events else 0
        return {"events": out,
                "next_since": out[-1]["seq"] if out else max(cursor, newest),
                "dropped": self.dropped,
                "total": self._seq}

    @property
    def total(self) -> int:
        return self._seq

    def last(self):
        return dict(self.events[-1]) if self.events else None


class FakeBridge:
    robot_id = "probe"
    model = "ProbeBot"
    world = "probe_world"
    fault = None
    held = False

    def __init__(self, ring: FakeRing) -> None:
        self.events = ring
        self.sim_time = 4.25


@pytest.fixture()
def wired(monkeypatch, tmp_path):
    """A relay with no threads, no network and a fake ring attached.

    `dispatch_async` is replaced with a recorder: this file is about WHICH
    turns get enqueued and how many, not about what the model answers.
    """
    # Never write into the shared journal dir: a test that clobbered the live
    # demo's record would be the same accident as a scratch run taking over a
    # production agent name.
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")   # no heartbeat thread
    monkeypatch.setenv("OMNILINK_EDGE", "0")       # no edge connector
    monkeypatch.delenv("OMNILINK_EVENT_WAKE", raising=False)

    probe = Tool("drive_forward", "move",
                 {"type": "object", "properties": {}}, lambda a: {"ok": True})
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="WakeProbe",
        main_task="test",
        tools=[probe],
        usage_enabled=False,
        memory_enabled=False,
        surface="mobile",
    )
    ring = FakeRing()
    relay.attach_bridge(FakeBridge(ring))

    dispatched = []

    def _record(text, on_event):
        dispatched.append(text)
        return relay_mod.DispatchHandle()

    relay.dispatch_async = _record            # type: ignore[assignment]
    relay._dispatched = dispatched            # type: ignore[attr-defined]
    try:
        yield relay, ring, dispatched
    finally:
        relay.close()


def prime(relay, ring) -> None:
    """Consume the backlog. The first poll never wakes -- see below."""
    relay.poll_events_once(now=0.0)


# ── The rate limit ───────────────────────────────────────────────────

def test_a_burst_of_a_hundred_events_buys_exactly_one_wake(wired) -> None:
    """THE load-bearing assertion. Without the bucket this is 100 model
    turns, which on the measured 5.6k prompt tokens each is most of a
    day's budget spent on one noisy second of simulation."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    for i in range(100):
        ring.add("joint.limit_hit", detail={"joint": f"j{i}"})

    woken = relay.poll_events_once(now=100.0)

    assert woken == 1, "a burst must not buy a turn per event"
    assert len(dispatched) == 1
    stats = relay.event_wake_stats()
    assert stats["wakes"] == 1
    assert stats["suppressed"] == 99, (
        "the 99 that did not wake must be COUNTED -- an invisible policy is "
        "one nobody can tell is too tight")


def test_a_second_burst_inside_the_interval_buys_nothing(wired) -> None:
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=100.0) == 1
    ring.add("motion.timed_out")
    assert relay.poll_events_once(now=105.0) == 0, "5 s < the 10 s floor"
    assert len(dispatched) == 1


def test_the_interval_opens_again(wired) -> None:
    """The limit is a rate, not a one-shot: a robot that keeps failing must
    still be able to say so again."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=100.0) == 1
    ring.add("motion.timed_out")
    assert relay.poll_events_once(now=100.0 + relay_mod.WAKE_MIN_INTERVAL_S) == 1
    assert len(dispatched) == 2


def test_the_session_budget_is_final(wired, monkeypatch) -> None:
    relay, ring, dispatched = wired
    monkeypatch.setattr(relay_mod, "WAKE_MAX_PER_SESSION", 2)
    monkeypatch.setattr(relay_mod, "WAKE_MIN_INTERVAL_S", 0.0)
    prime(relay, ring)
    for i in range(10):
        ring.add("fault.controller_lost", detail={"i": i})
        relay.poll_events_once(now=200.0 + i)
    assert len(dispatched) == 2, "the session cap must stop the stream dead"


# ── The policy: only physical or safety events ───────────────────────

@pytest.mark.parametrize("etype", [
    "contact.ended", "grip.acquired", "grip.released", "controller.log",
    "world.warning", "motion.unsettled", "hold.expired",
])
def test_a_non_waking_type_produces_no_wake(wired, etype) -> None:
    relay, ring, dispatched = wired
    prime(relay, ring)
    for _ in range(100):
        ring.add(etype)
    assert relay.poll_events_once(now=300.0) == 0
    assert dispatched == []


@pytest.mark.parametrize("etype", [
    "joint.limit_hit", "motion.timed_out", "fault.controller_lost",
    "fault.telemetry_stale", "break.hit", "damage.impact",
])
def test_a_physical_or_safety_type_does_wake(wired, etype) -> None:
    """The red half: if this ever goes green-by-vacuity (nothing wakes at
    all) the test above would still pass, so both directions are pinned."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add(etype)
    assert relay.poll_events_once(now=400.0) == 1
    assert etype in dispatched[0]


def test_an_unknown_type_does_not_wake(wired) -> None:
    """A type nobody has classified costs nothing. The alternative --
    waking on anything unrecognised -- makes every new detector a billing
    change by default."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add("telemetry.sampled")
    assert relay.poll_events_once(now=500.0) == 0


def test_a_floor_contact_does_not_wake(wired) -> None:
    """`contact.began` wakes for a NON-floor body only. A robot standing on
    the ground is the world working normally."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add("contact.began", detail={"body": "RectangleArena_FLOOR"})
    assert relay.poll_events_once(now=600.0) == 0
    ring.add("contact.began", detail={"body": "PALLET_C"})
    assert relay.poll_events_once(now=600.0) == 1


def test_a_refusal_on_the_agents_own_turn_does_not_wake(wired) -> None:
    """A `gate.refused` the model caused is already in the reply it is
    reading. Waking it about that is a loop with a bill."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    ring.add("gate.refused", detail={"origin": "relay", "tool": "drive_forward"})
    assert relay.poll_events_once(now=700.0) == 0
    ring.add("gate.refused", detail={"origin": "tool", "tool": "drive_forward"})
    assert relay.poll_events_once(now=700.0) == 1, (
        "a refusal that arrived over HTTP is news to the agent")


# ── The hatch ────────────────────────────────────────────────────────

def test_the_hatch_disables_the_wake_and_fills_nothing_of_its_own(
        monkeypatch, tmp_path) -> None:
    """`OMNILINK_EVENT_WAKE=0`: zero wakes, and the ring is untouched --
    the relay must not compensate by writing events of its own."""
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    monkeypatch.setenv("OMNILINK_EVENT_WAKE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="WakeOffProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False, surface="mobile")
    ring = FakeRing()
    relay.attach_bridge(FakeBridge(ring))
    dispatched = []
    relay.dispatch_async = lambda t, cb: (dispatched.append(t)
                                          or relay_mod.DispatchHandle())
    try:
        relay.poll_events_once(now=0.0)
        for _ in range(50):
            ring.add("joint.limit_hit")
        assert relay.poll_events_once(now=800.0) == 0
        assert dispatched == []
        assert ring.total == 50, "the relay must not add events of its own"
        # ...and the events are still READABLE. Disabling the wake is a
        # budget decision, not a blindfold.
        page = relay.tools["get_events"].dispatch({"limit": 5})
        assert page["available"] is True and len(page["events"]) == 5
    finally:
        relay.close()


def test_the_hatch_is_value_parsed(monkeypatch) -> None:
    """`=0` is off. A presence-gated hatch would ARM on `=0`, which is the
    OMNISIM_REQUIRE_NEWTON trap class."""
    for raw, expected in (("0", False), ("false", False), ("no", False),
                          ("off", False), ("1", True), ("", True)):
        monkeypatch.setenv("OMNILINK_EVENT_WAKE", raw)
        assert relay_mod._env_flag("OMNILINK_EVENT_WAKE", True) is expected


# ── Cursor discipline ────────────────────────────────────────────────

def test_the_first_poll_primes_the_cursor_and_never_wakes(wired) -> None:
    """A bridge that restarts with a full ring must not wake on a backlog
    that was already answered -- or that happened before anyone listened."""
    relay, ring, dispatched = wired
    for _ in range(20):
        ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=0.0) == 0, "the priming pass must be silent"
    assert dispatched == []
    ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=900.0) == 1, "and then it must work"


def test_priming_skips_a_backlog_bigger_than_one_page(wired) -> None:
    """The bug this caught: priming used to take `next_since` from a
    limit-capped drain, so a ring holding more than one page left the rest
    of the backlog unread and the SECOND poll woke on history."""
    relay, ring, dispatched = wired
    for _ in range(500):
        ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=0.0) == 0
    assert relay.event_wake_stats()["cursor"] == ring.total
    assert relay.poll_events_once(now=1.0) == 0, (
        "the second poll must not discover a backlog the first skipped")
    assert dispatched == []


def test_the_cursor_advances_even_when_nothing_wakes(wired) -> None:
    """Otherwise a quiet stream is re-scanned forever and one late waking
    event would be judged against a stale cursor."""
    relay, ring, dispatched = wired
    prime(relay, ring)
    for _ in range(5):
        ring.add("controller.log")
    relay.poll_events_once(now=1000.0)
    assert relay.event_wake_stats()["cursor"] == ring.total


# ── The turn itself ──────────────────────────────────────────────────

def test_the_wake_text_is_the_specified_shape() -> None:
    text = wake_text({"type": "joint.limit_hit", "sim_time": 12.5,
                      "detail": {"joint": "arm_1", "q": 1.57}})
    assert text.startswith("[Event] joint.limit_hit at sim t=12.500: ")
    assert "arm_1" in text


def test_the_wake_text_survives_a_missing_clock() -> None:
    assert "sim t=unknown" in wake_text({"type": "fault.controller_lost"})


def test_a_long_detail_is_bounded() -> None:
    text = wake_text({"type": "damage.impact", "sim_time": 1.0,
                      "detail": {"note": "x" * 5000}})
    assert len(text) < 500, "one noisy event must not write a kB into history"


def test_the_wake_goes_through_dispatch_not_straight_to_a_tool(wired) -> None:
    """The wake is an ordinary turn: the model decides what to do and the
    GATE vets whatever it calls. Nothing here may reach an actuator."""
    relay, ring, dispatched = wired
    calls = []
    relay.tools["drive_forward"].dispatch = lambda a: calls.append(a)
    prime(relay, ring)
    ring.add("motion.timed_out", detail={"tool": "drive_forward"})
    relay.poll_events_once(now=1100.0)
    assert len(dispatched) == 1
    assert calls == [], "a wake must never actuate on its own"


def test_a_refused_enqueue_refunds_the_budget(wired) -> None:
    """A queue that is briefly full must not burn the session's thirty
    wakes on turns that never ran."""
    relay, ring, dispatched = wired

    def _refuse(text, on_event):
        handle = relay_mod.DispatchHandle()
        handle.cancel()
        return handle

    relay.dispatch_async = _refuse            # type: ignore[assignment]
    prime(relay, ring)
    ring.add("joint.limit_hit")
    assert relay.poll_events_once(now=1200.0) == 0
    stats = relay.event_wake_stats()
    assert stats["wakes"] == 0 and stats["suppressed"] == 1


# ── get_events ───────────────────────────────────────────────────────

def test_get_events_is_registered_by_the_relay(wired) -> None:
    """Registered HERE rather than per bridge, for the same reason
    get_action_history is: a per-bridge opt-in silently misses the bridges
    that forget, and every tool added later."""
    relay, ring, dispatched = wired
    assert "get_events" in relay.tools
    assert "get_events" in [d["name"] for d in relay.tool_defs]


def test_get_events_reads_the_ring(wired) -> None:
    relay, ring, dispatched = wired
    ring.add("contact.began", detail={"body": "PALLET_C"})
    ring.add("joint.limit_hit", detail={"joint": "j1"})
    page = relay.tools["get_events"].dispatch({"limit": 10})
    assert page["available"] is True
    assert [e["type"] for e in page["events"]] == ["contact.began",
                                                   "joint.limit_hit"]
    assert page["next_since"] == 2
    filtered = relay.tools["get_events"].dispatch({"types": "joint.limit_hit"})
    assert [e["type"] for e in filtered["events"]] == ["joint.limit_hit"]


def test_get_events_without_a_ring_says_so_instead_of_answering_empty(
        monkeypatch, tmp_path) -> None:
    """An empty list reads as "nothing happened", which is a claim. An
    explicit `available: false` cannot be mistaken for one -- and returning
    None would be worse still, since a null tool result gets narrated."""
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="NoRingProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False)
    try:
        page = relay.tools["get_events"].dispatch({})
        assert page is not None, "abstaining with None is not a safe non-answer"
        assert page["available"] is False
        assert page["events"] == []
        assert "not" in page["reason"]
        assert "NOT evidence" in page["note"]
    finally:
        relay.close()


# ── Against the REAL ring ────────────────────────────────────────────

def test_the_dispatcher_drives_the_real_event_ring(monkeypatch,
                                                   tmp_path) -> None:
    """The fake above could drift from `events.EventRing`. This one uses the
    real ring and the real `may_wake`, so the two halves are pinned
    together: the ring's own floor rule, its cursor and its envelope."""
    from omnisim_bridges.events import EventRing

    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="RealRingProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False, surface="mobile")
    ring = EventRing(robot="husky")
    dispatched = []
    relay.dispatch_async = lambda t, cb: (dispatched.append(t)
                                          or relay_mod.DispatchHandle())
    try:
        relay.attach_bridge(FakeBridge(ring))   # type: ignore[arg-type]
        relay.poll_events_once(now=0.0)

        ring.emit("contact.began", sim_time=1.0, step=10, body="FLOOR")
        assert relay.poll_events_once(now=10.0) == 0, "the floor is not news"

        ring.emit("joint.limit_hit", sim_time=2.0, step=20,
                  joint="arm_1", q=1.57)
        assert relay.poll_events_once(now=20.0) == 1
        assert dispatched[0].startswith("[Event] joint.limit_hit at sim t=2.000")

        page = relay.tools["get_events"].dispatch({"limit": 10})
        assert page["available"] is True
        assert [e["type"] for e in page["events"]] == ["contact.began",
                                                       "joint.limit_hit"]
        assert page["dropped"] == 0
    finally:
        relay.close()


def test_a_ring_that_raises_is_reported_as_unknown_not_quiet(wired) -> None:
    relay, ring, dispatched = wired

    def _boom(*a, **k):
        raise RuntimeError("ring exploded")

    ring.since = _boom                        # type: ignore[assignment]
    page = relay.tools["get_events"].dispatch({})
    assert page["available"] is False
    assert "failed" in page["reason"]
    # ...and the poller survives it, because it runs on the heartbeat thread.
    assert relay.poll_events_once(now=1300.0) == 0
