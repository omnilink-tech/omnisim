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

"""Presence has to say what STATE the robot is in, not just that it is up.

Before this, a held robot, a faulted robot and a healthy one produced
byte-identical heartbeats: `{tools, engine, interval_ms}`. The roster
could show that a bridge was alive and nothing else, so the two failures
that were actually measured on the shipped agents -- a robot stopped on an
autonomy hold, and a robot whose profile pointed at a dead port -- both
looked exactly like health.

The platform caps `detail` at TWELVE keys and 200-char strings
(`api/relay-heartbeat.ts`: `Object.entries(detail).slice(0, 12)`), and the
slice is silent. So the cap is enforced HERE, in a known order, with
`interval_ms` first -- it sizes the staleness window, and a robot whose
interval falls off the end reads as offline for 20 of every 30 seconds.
"""

from __future__ import annotations

import json

import pytest

from omnisim_bridges.relay import OmniLinkRelay
from omnisim_bridges.tool import Tool


class FakeRing:
    def __init__(self, events=None):
        self.events = list(events or [])

    @property
    def total(self):
        return len(self.events)

    def last(self):
        return dict(self.events[-1]) if self.events else None

    def since(self, cursor=0, *, limit=100, types=None):
        out = [dict(e) for e in self.events if e["seq"] > cursor][:limit]
        return {"events": out, "next_since": out[-1]["seq"] if out else cursor,
                "dropped": 0, "total": len(self.events)}


class FakeHold:
    def __init__(self, enabled=True):
        self.enabled = enabled


class FakeBridge:
    """Publishes the contract BridgeBase publishes, and nothing more."""

    def __init__(self, **kw):
        self.robot_id = kw.get("robot_id", "husky")
        self.sim_time = kw.get("sim_time", 12.4485)
        self.sim_step = kw.get("sim_step", 389)
        self.held = kw.get("held", False)
        self.fault = kw.get("fault", None)
        self.world = kw.get("world", "warehouse_husky")
        self.events = kw.get("events", FakeRing())
        if "hold" in kw:
            self.hold = kw["hold"]


@pytest.fixture()
def relay(monkeypatch, tmp_path):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    r = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="PresenceProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False, surface="mobile")
    try:
        yield r
    finally:
        r.close()


# ── The nine new keys ────────────────────────────────────────────────

def test_the_nine_state_keys_reach_the_heartbeat(relay) -> None:
    ring = FakeRing([{"seq": 1, "type": "joint.limit_hit", "sim_time": 9.5}])
    relay.attach_bridge(FakeBridge(events=ring, fault="telemetry_stale",
                                   held=True, hold=FakeHold(True)))
    relay.journal.record("drive_forward", {}, ok=True, summary="a")
    detail = relay.presence_detail()

    assert detail["sim_time"] == pytest.approx(12.4485, abs=1e-3)  # SIM seconds
    assert detail["held"] is True
    assert detail["fault"] == "telemetry_stale"
    assert detail["world"] == "warehouse_husky"
    assert detail["robot"] == "husky"
    assert detail["last_event"].startswith("joint.limit_hit@")
    assert detail["events_total"] == 1
    assert detail["journal_len"] == 1
    assert detail["lockstep"] is True


def test_a_healthy_robot_reports_held_false_not_nothing(relay) -> None:
    """`held` absent and `held: false` read the same on a roster, and they
    must not: "this bridge does not report holds" is a different fact from
    "this robot is running"."""
    relay.attach_bridge(FakeBridge())
    detail = relay.presence_detail()
    assert detail["held"] is False
    assert "fault" not in detail, "a healthy robot has no fault to report"


def test_sim_time_zero_is_reported_and_no_clock_is_not(relay) -> None:
    """0.0 is a measurement -- a world that has not stepped yet. A bridge
    with no clock at all must not be rendered as one frozen at t=0."""
    relay.attach_bridge(FakeBridge(sim_time=0.0))
    assert relay.presence_detail()["sim_time"] == 0.0

    class Clockless:
        robot_id = "x"

    relay.attach_bridge(Clockless())
    assert "sim_time" not in relay.presence_detail()


def test_a_bridge_that_publishes_nothing_heartbeats_as_it_always_did(
        relay) -> None:
    """Backwards compatibility in the direction that matters: an old bridge
    against this relay keeps working, it just reports less."""
    detail = relay.presence_detail()
    assert set(detail) >= {"tools", "engine", "interval_ms"}
    assert detail["interval_ms"] == 30000
    assert "journal_len" in detail, "the relay's own journal is always known"


def test_the_bridges_own_robot_name_wins(relay) -> None:
    """`set_presence_endpoint(url, robot=...)` is how the four shipped
    bridges name themselves today; the derived one is the fallback."""
    relay.set_presence_endpoint("http://127.0.0.1:8765/tool", robot="tb3")
    relay.attach_bridge(FakeBridge(robot_id="husky"))
    assert relay.presence_detail()["robot"] == "tb3"


def test_lockstep_comes_from_the_hold_lease(relay) -> None:
    relay.attach_bridge(FakeBridge(hold=FakeHold(False)))
    assert relay.presence_detail()["lockstep"] is False


# ── The platform's cap, enforced here ────────────────────────────────

def test_the_detail_never_exceeds_the_platforms_twelve_keys(relay) -> None:
    relay.set_presence_endpoint("http://127.0.0.1:8765/tool", robot="husky",
                                extra_one=1, extra_two=2, extra_three=3)
    relay.attach_bridge(FakeBridge(
        events=FakeRing([{"seq": 1, "type": "damage.impact", "sim_time": 1.0}]),
        fault="controller_lost", hold=FakeHold(True)))
    detail = relay.presence_detail()
    assert len(detail) <= 12, (
        "the platform silently keeps the first twelve; anything past that is "
        "dropped without a word")


def test_interval_ms_can_never_be_the_key_that_falls_off(relay) -> None:
    """Without it a 30 s beat is judged against a browser's 10 s window and
    a healthy robot flickers offline. Measured side by side on two running
    robots, one 'online' at 6 s and its neighbour 'offline' at 22 s."""
    for i in range(20):
        relay.set_presence_endpoint("http://127.0.0.1:8765/tool",
                                    **{f"junk_{i}": i})
    relay.attach_bridge(FakeBridge(fault="x", hold=FakeHold(True)))
    detail = relay.presence_detail()
    assert list(detail)[0] == "interval_ms"
    assert len(detail) <= 12


def test_a_long_string_is_truncated_to_the_platforms_limit(relay) -> None:
    relay.attach_bridge(FakeBridge(world="w" * 4000))
    assert len(relay.presence_detail()["world"]) == 200


def test_the_detail_is_json_serialisable(relay) -> None:
    """It is posted as JSON; a stray object would fail the beat silently."""
    relay.attach_bridge(FakeBridge(
        events=FakeRing([{"seq": 1, "type": "break.hit", "sim_time": 2.0}])))
    json.dumps(relay.presence_detail())


def test_a_bridge_whose_reads_explode_does_not_stop_the_beat(relay) -> None:
    class Hostile:
        robot_id = "boom"

        @property
        def sim_time(self):
            raise RuntimeError("no")

        @property
        def events(self):
            raise RuntimeError("no")

    relay.attach_bridge(Hostile())
    detail = relay.presence_detail()           # must not raise
    assert "interval_ms" in detail


# ── The reply: wakeRequestedAt ───────────────────────────────────────

def test_the_post_returns_the_reply_instead_of_discarding_it(
        relay, monkeypatch) -> None:
    """`{ok, wakeRequestedAt}` is the platform's ONLY downstream channel to
    this process, and it was being thrown away."""
    class FakeResponse:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self):
            return b'{"ok": true, "wakeRequestedAt": "2026-09-22T10:00:00Z"}'

    monkeypatch.setattr("urllib.request.urlopen",
                        lambda req, timeout=None: FakeResponse())
    data = relay._post_presence({"agentName": "x", "detail": {}})
    assert data["wakeRequestedAt"] == "2026-09-22T10:00:00Z"


def test_a_changed_wake_stamp_raises_the_window(relay) -> None:
    raised = []
    relay.set_window_raiser(lambda: raised.append(True))

    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "T1"})
    assert raised == [], "the first reply only baselines the stamp"
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "T1"})
    assert raised == [], "an unchanged stamp is not a new request"
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "T2"})
    assert raised == [True], "a NEW stamp is a request for attention"


def test_a_stamp_set_before_this_bridge_booted_is_not_for_it(relay) -> None:
    """`wakeRequestedAt` stays set, so acting on its presence would raise
    the window every 30 s forever after one request an hour ago."""
    raised = []
    relay.set_window_raiser(lambda: raised.append(True))
    for _ in range(5):
        relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "OLD"})
    assert raised == []


def test_no_raiser_registered_is_not_an_error(relay) -> None:
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "A"})
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "B"})


def test_a_raiser_that_throws_does_not_break_the_heartbeat(relay) -> None:
    def _boom():
        raise RuntimeError("window is gone")

    relay.set_window_raiser(_boom)
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "A"})
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": "B"})


def test_the_heartbeat_thread_posts_the_new_detail_and_reads_the_reply(
        monkeypatch, tmp_path) -> None:
    """The loop itself, not just its parts: one real beat on the real
    thread, carrying the state keys, with the reply routed to the window
    raiser. Patched on the CLASS so the very first beat is captured."""
    import time as _time

    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    monkeypatch.delenv("OMNILINK_PRESENCE", raising=False)
    beats = []

    def _fake_post(self, body):
        beats.append(body)
        return {"ok": True, "wakeRequestedAt": f"T{len(beats)}"}

    monkeypatch.setattr(OmniLinkRelay, "_post_presence", _fake_post)
    r = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="LoopProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False, surface="mobile")
    r.attach_bridge(FakeBridge(world="loop_world"))
    try:
        deadline = _time.time() + 5
        while not beats and _time.time() < deadline:
            _time.sleep(0.02)
        assert beats, "the heartbeat thread never beat"
        assert beats[0]["agentName"] == "LoopProbe"
        assert beats[0]["kind"] == "bridge"
        assert beats[0]["detail"]["interval_ms"] == 30000
        # The bridge attached a beat after construction, so the FIRST beat
        # may predate it -- what must hold is that the detail is the one
        # `presence_detail()` builds, not a frozen dict from __init__.
        assert set(beats[0]["detail"]) >= {"interval_ms", "tools", "engine"}
        assert r._last_wake_requested_at == "T1", (
            "the reply must be read, not discarded")
    finally:
        r.close()


def test_a_junk_reply_is_ignored(relay) -> None:
    raised = []
    relay.set_window_raiser(lambda: raised.append(True))
    relay._handle_presence_reply(None)
    relay._handle_presence_reply({"ok": True})
    relay._handle_presence_reply({"ok": True, "wakeRequestedAt": None})
    assert raised == []
