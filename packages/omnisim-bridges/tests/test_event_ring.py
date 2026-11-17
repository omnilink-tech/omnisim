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

"""The bridge event ring, its detectors, and the wake policy (Track D4).

⚠️ EVERY DETECTOR TEST HERE IS WRITTEN TO GO RED WHEN ITS DETECTOR IS
DISABLED, and that was checked by disabling each one in turn rather than
assumed. An assertion that has never failed may be vacuous -- this tree has
already shipped a structurally-impossible contact-pairing check that read
green for weeks. Each detector's test below asserts on the EVENT, not on the
call: `detector.update(...)` returning a list is not evidence that anything
reached the ring.

Engine-free. Nothing in this file imports Webots, launches a binary or opens
a socket.
"""

from __future__ import annotations

import pathlib
import sys
import time

import pytest

PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges import events as ev             # noqa: E402
from omnisim_bridges.bridge_base import (            # noqa: E402
    GATE_RULES,
    SimClock,
    events_query,
    safety_gate_block,
    serve_events,
)


# ── The ring ────────────────────────────────────────────────────────

def test_seq_is_monotonic_from_one() -> None:
    ring = ev.EventRing(capacity=16)
    ring.emit("motion.timed_out", sim_time=1.0, step=31)
    ring.emit("joint.limit_hit", sim_time=1.032, step=32)
    seqs = [e["seq"] for e in ring.since(0)["events"]]
    assert seqs == [1, 2]
    assert ring.total == 2


def test_event_shape_is_the_contract() -> None:
    ring = ev.EventRing(capacity=8, robot="husky")
    ring.emit("gate.refused", sim_time=2.5, step=78, tool="drive_forward",
              reason="distance=300 exceeds the 50 m rail")
    evt = ring.last()
    assert set(evt) == {"seq", "type", "sim_time", "step", "robot",
                        "source", "detail"}
    assert evt["type"] == "gate.refused"
    assert evt["sim_time"] == 2.5
    assert evt["step"] == 78
    assert evt["robot"] == "husky"
    assert evt["source"] == "bridge"
    assert evt["detail"]["tool"] == "drive_forward"


def test_cursor_paging_never_repeats_and_never_skips() -> None:
    ring = ev.EventRing(capacity=64)
    for i in range(10):
        ring.emit("contact.began", sim_time=i * 0.032, step=i, body=f"crate_{i}")
    first = ring.since(0, limit=4)
    assert len(first["events"]) == 4
    assert first["next_since"] == 4
    second = ring.since(first["next_since"], limit=4)
    assert [e["seq"] for e in second["events"]] == [5, 6, 7, 8]
    third = ring.since(second["next_since"])
    assert [e["seq"] for e in third["events"]] == [9, 10]
    assert ring.since(third["next_since"])["events"] == []


def test_dropped_counts_evictions_and_missed_counts_this_cursor() -> None:
    ring = ev.EventRing(capacity=4)
    for i in range(10):
        ring.emit("contact.began", sim_time=0.0, step=i)
    out = ring.since(0)
    assert out["dropped"] == 6            # six fell off the ring
    assert out["missed"] == 6             # and this cursor will never see them
    assert out["total"] == 10
    assert out["buffered"] == 4
    # A cursor that is already past the evicted window missed nothing.
    assert ring.since(7)["missed"] == 0


def test_type_filter_still_advances_the_cursor() -> None:
    # A filtered poller that never advanced its cursor re-scanned the whole
    # buffer on every poll and re-delivered the first match forever.
    ring = ev.EventRing(capacity=32)
    ring.emit("contact.began", sim_time=0.0, step=1, body="crate")
    for i in range(5):
        ring.emit("controller.log", sim_time=0.0, step=2 + i, line="noise")
    out = ring.since(0, types=["contact.began"])
    assert [e["type"] for e in out["events"]] == ["contact.began"]
    nxt = ring.since(out["next_since"], types=["contact.began"])
    assert nxt["events"] == []
    assert nxt["next_since"] == 6          # advanced past the noise


def test_emit_never_raises_on_a_bad_stamp() -> None:
    # A detector must not be able to take the sim thread down.
    ring = ev.EventRing(capacity=4)
    ring.emit("motion.timed_out", sim_time="not a number", step=0)
    assert ring.total == 0                 # dropped, not raised
    ring.emit("motion.timed_out", sim_time=1.0, step=1)
    assert ring.total == 1


def test_summary_is_the_state_events_block() -> None:
    ring = ev.EventRing(capacity=8)
    assert ring.summary() == {"total": 0, "last": None, "next_since": 0,
                              "dropped": 0}
    ring.emit("hold.expired", sim_time=9.0, step=280)
    s = ring.summary()
    assert s["total"] == 1 and s["next_since"] == 1
    assert s["last"]["type"] == "hold.expired"


def test_capacity_hatch_is_value_parsed(monkeypatch) -> None:
    monkeypatch.setenv("OMNISIM_BRIDGE_EVENT_CAPACITY", "32")
    assert ev.EventRing().since(0)["capacity"] == 32
    monkeypatch.setenv("OMNISIM_BRIDGE_EVENT_CAPACITY", "not-a-number")
    assert ev.EventRing().since(0)["capacity"] == ev.DEFAULT_CAPACITY


# ── Detector: joint.limit_hit ───────────────────────────────────────

def test_joint_limit_detector_fires_on_the_stop() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.JointLimitDetector(ring, margin=0.02)
    names = ["shoulder", "elbow"]
    limits = [(-1.57, 1.57), (-2.0, 2.0)]
    # Well inside: nothing.
    det.update(names, [0.0, 0.0], limits, sim_time=1.0, step=31)
    assert ring.total == 0
    # Elbow parked on its upper stop.
    fired = det.update(names, [0.0, 1.995], limits, sim_time=1.032, step=32)
    assert fired == ["elbow"]
    evt = ring.last()
    assert evt["type"] == "joint.limit_hit"
    assert evt["detail"]["joint"] == "elbow"
    assert evt["detail"]["side"] == "max"
    assert evt["detail"]["limit"] == 2.0
    assert evt["sim_time"] == 1.032 and evt["step"] == 32


def test_joint_limit_detector_does_not_spam_while_parked() -> None:
    # A joint resting on its stop jitters at solver noise. Without
    # hysteresis this emitted one event per tick and a wake budget went on
    # a joint that was not moving.
    ring = ev.EventRing(capacity=256)
    det = ev.JointLimitDetector(ring, margin=0.02)
    for i in range(50):
        det.update(["j"], [-1.5699 + (i % 2) * 1e-5], [(-1.57, 1.57)],
                   sim_time=i * 0.032, step=i)
    assert ring.total == 1


def test_joint_limit_detector_rearms_only_after_clearing_the_band() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.JointLimitDetector(ring, margin=0.02, release_frac=2.0)
    det.update(["j"], [1.565], [(-1.57, 1.57)], sim_time=0.0, step=0)
    assert ring.total == 1
    det.update(["j"], [1.551], [(-1.57, 1.57)], sim_time=0.032, step=1)
    assert ring.total == 1                 # inside the release band: no re-arm
    det.update(["j"], [0.0], [(-1.57, 1.57)], sim_time=0.064, step=2)
    det.update(["j"], [1.565], [(-1.57, 1.57)], sim_time=0.096, step=3)
    assert ring.total == 2                 # came back, went again: reported


def test_joint_limit_detector_ignores_junk_readback() -> None:
    ring = ev.EventRing(capacity=8)
    det = ev.JointLimitDetector(ring)
    det.update(["j"], [None], [(-1.0, 1.0)], sim_time=0.0, step=0)
    det.update(["j"], [0.99], [None], sim_time=0.0, step=1)
    det.update(["j"], [0.99], [(1.0, -1.0)], sim_time=0.0, step=2)
    assert ring.total == 0


# ── Detector: contact.began ─────────────────────────────────────────

def test_contact_detector_fires_on_a_named_non_floor_body() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.ContactDetector(ring)
    fired = det.update(["FLOOR", "CRATE_A"], sim_time=3.2, step=100,
                       robot="husky")
    assert fired == ["CRATE_A"]
    evt = ring.last()
    assert evt["type"] == "contact.began"
    assert evt["detail"]["body"] == "CRATE_A"


def test_contact_detector_declares_its_blind_spot_in_the_event() -> None:
    # An agent reading an empty contact stream must be able to tell "nothing
    # touched" from "this instrument cannot see it". getContactPoints()
    # reports top-level Solids only; URDF sub-links are invisible to it.
    ring = ev.EventRing(capacity=8)
    ev.ContactDetector(ring).update(["PALLET"], sim_time=1.0, step=1)
    detail = ring.last()["detail"]
    assert detail["scope"] == "top-level"
    assert "sub-link" in detail["blind_spot"]


def test_contact_detector_is_silent_about_the_floor() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.ContactDetector(ring)
    det.update(["FLOOR", "ground_plane", "arena_terrain"], sim_time=0.0, step=0)
    assert ring.total == 0


def test_contact_detector_reports_a_began_not_a_continuing_touch() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.ContactDetector(ring)
    for step in range(20):
        det.update(["CRATE"], sim_time=step * 0.032, step=step)
    assert ring.total == 1
    det.update([], sim_time=1.0, step=21)      # let go
    det.update(["CRATE"], sim_time=1.032, step=22)
    assert ring.total == 2                     # touched it again: reported


# ── Detector: the bridge's own health ───────────────────────────────

def test_fault_detector_fires_on_the_rising_edge_only() -> None:
    ring = ev.EventRing(capacity=32)
    det = ev.FaultDetector(ring)
    assert det.on_tick(None, sim_time=0.0, step=0) is None
    assert ring.total == 0
    det.on_tick("motor_unreachable", sim_time=1.0, step=31)
    assert ring.total == 1
    assert ring.last()["type"] == "fault.controller_lost"
    assert ring.last()["detail"]["code"] == "motor_unreachable"
    for i in range(10):                     # still faulted: still one event
        det.on_tick("motor_unreachable", sim_time=1.0 + i, step=32 + i)
    assert ring.total == 1


def test_fault_detector_accepts_the_protocol_fault_object() -> None:
    ring = ev.EventRing(capacity=8)
    ev.FaultDetector(ring).on_tick(
        {"code": "estop", "message": "operator stop", "since_t": 12.0},
        sim_time=12.0, step=375)
    assert ring.last()["detail"]["code"] == "estop"


def test_telemetry_stale_fires_from_a_thread_the_stall_did_not_stop() -> None:
    # ⚠️ THE VACUITY TRAP IN ITS PUREST FORM. `fault.telemetry_stale` means
    # the sim thread has STOPPED, so a detector that only ran in tick()
    # could never fire for it. poll_stale() runs on whatever thread is
    # serving the request and reads cached floats only.
    ring = ev.EventRing(capacity=32)
    det = ev.FaultDetector(ring, stale_after_s=2.0)
    assert det.poll_stale(wall_now=1000.0, last_tick_wall=999.5,
                          sim_time=4.0, step=125) is False
    assert ring.total == 0
    assert det.poll_stale(wall_now=1010.0, last_tick_wall=999.5,
                          sim_time=4.0, step=125) is True
    evt = ring.last()
    assert evt["type"] == "fault.telemetry_stale"
    assert evt["detail"]["age_s"] == pytest.approx(10.5)
    # Edge-triggered: a poller at 20 Hz must not file 20 events a second.
    for _ in range(20):
        det.poll_stale(wall_now=1011.0, last_tick_wall=999.5,
                       sim_time=4.0, step=125)
    assert ring.total == 1


# ── motion.timed_out / motion.unsettled ─────────────────────────────

def test_motion_outcome_emits_on_a_timeout() -> None:
    ring = ev.EventRing(capacity=32)
    kind = ev.emit_motion_outcome(
        ring, {"verb": "drive_forward", "seq": 3, "commanded": 300.0,
               "achieved": 4.2, "error": -295.8, "unit": "m",
               "settled": False, "timed_out": True, "steps": 938},
        sim_time=30.0, step=938, robot="husky")
    assert kind == "motion.timed_out"
    evt = ring.last()
    assert evt["type"] == "motion.timed_out"
    assert evt["detail"]["achieved"] == 4.2
    assert evt["detail"]["steps"] == 938


def test_motion_outcome_is_silent_on_a_clean_motion() -> None:
    # ⚠️ THE FLOAT-ERROR TRAP. `error` here is the CONTROL error -- achieved
    # minus commanded -- and a textbook 2 m drive carries -0.0022. Reading
    # it as a failure is the bug that has shipped three times; the most
    # recent made every successful /tool motion report status "err".
    ring = ev.EventRing(capacity=32)
    kind = ev.emit_motion_outcome(
        ring, {"verb": "drive_forward", "commanded": 2.0, "achieved": 1.9977,
               "error": -0.0022885799407958984, "unit": "m",
               "settled": True, "timed_out": False},
        sim_time=8.0, step=250)
    assert kind is None
    assert ring.total == 0


def test_motion_outcome_emits_unsettled_when_it_gave_up_short() -> None:
    ring = ev.EventRing(capacity=32)
    kind = ev.emit_motion_outcome(
        ring, {"verb": "turn", "commanded": 1.5708, "achieved": 1.2,
               "error": -0.3708, "unit": "rad", "settled": False,
               "timed_out": False, "corrections": 4},
        sim_time=12.0, step=375)
    assert kind == "motion.unsettled"
    assert ring.last()["type"] == "motion.unsettled"


def test_motion_outcome_says_nothing_about_a_superseded_motion() -> None:
    # Nobody measured it. An event claiming it failed would be an invention.
    ring = ev.EventRing(capacity=32)
    kind = ev.emit_motion_outcome(
        ring, {"verb": "turn", "achieved": None, "error": None,
               "settled": False, "timed_out": False, "superseded": True},
        sim_time=1.0, step=31)
    assert kind is None and ring.total == 0


def test_motion_outcome_tolerates_no_ring() -> None:
    assert ev.emit_motion_outcome(None, {"timed_out": True},
                                  sim_time=0.0, step=0) is None


# ── gate.refused ────────────────────────────────────────────────────

def test_the_tool_path_files_the_RULE_not_just_the_prose() -> None:
    # ⚠️ MEASURED ON A LIVE HUSKY. A correctly refused 300 m drive filed
    # `rule: ""` beside `reason: "implausible: distance=300 exceeds the
    # 50 m rail"`. `rule` is the field a PROGRAM branches on -- `reason` is
    # prose and may change between releases -- so an empty one makes the
    # event unusable for exactly the automated reaction it exists to
    # enable, while still looking populated to a schema check.
    #
    # Hence the assertion is on the VALUE, not the key. `assert "rule" in
    # detail` would have passed against the defect.
    from omnisim_bridges.bridge_base import serve_tool

    class _B:
        robot_id = "husky"
        sim_time = 4.688
        sim_step = 146

        def __init__(self):
            self.events = ev.EventRing(capacity=8, robot="husky")

    b = _B()
    code, payload = serve_tool(
        "drive_forward",
        {"tool": "drive_forward", "distance": 300,
         "utterance": "drive forward 300 metres"},
        lambda args: {"ok": True}, surface="mobile", bridge=b)
    assert code == 400
    evt = b.events.last()
    assert evt["type"] == "gate.refused"
    assert evt["detail"]["rule"] == "implausible"
    assert evt["detail"]["origin"] == "tool"
    assert evt["detail"]["reason"].startswith("implausible: ")
    # The 400 carries the same fact, so a /tool caller need not re-parse
    # the prose either.
    assert payload["details"]["rule"] == "implausible"


def test_the_prompt_path_files_the_rule_too() -> None:
    # The other origin. Here the rule comes off the Rejection object
    # directly rather than out of a string, so the two paths could easily
    # have disagreed -- which is the whole reason both are pinned.
    from omnisim_bridges import route as _route
    from omnisim_bridges import interpret as _i

    class _B:
        robot_id = "husky"
        sim_time = 9.0
        sim_step = 281
        intents = None

        def __init__(self):
            self.events = ev.EventRing(capacity=8, robot="husky")

        def act_drive_forward(self, distance=None, wait=False):
            raise AssertionError("the gate must stop this before the bridge")

    b = _B()
    out = _route.execute(b, _i.Interpretation(
        intent=_i.COMMAND,
        frames=[_i.Frame("drive_forward", {"distance": 500.0})],
        confidence=0.99,
        text="how far could you drive if I asked you to?"), surface="mobile")
    assert out["tools"][0][1] == "refused"
    evt = b.events.last()
    assert evt["type"] == "gate.refused"
    assert evt["detail"]["rule"] in ("interrogative", "implausible")
    assert evt["detail"]["origin"] == "prompt"


def test_no_refusal_may_file_an_empty_rule() -> None:
    # The invariant behind both tests above, stated once. Every string the
    # two producers can hand the emit site must yield a usable name --
    # including the fail-closed wrapper's, which contains no rule at all.
    from omnisim_bridges.bridge_base import refusal_rule
    for reason in ("implausible: distance=300 exceeds the 50 m rail",
                   "interrogative: the utterance asks a question",
                   "safety gate unavailable (ImportError)",
                   "safety gate errored (TypeError)",
                   "", None, 12345):
        rule = refusal_rule(reason)
        assert rule, f"{reason!r} produced an empty rule"
        assert " " not in rule


def test_the_fail_closed_wrapper_never_borrows_a_real_rule_name() -> None:
    # ⚠️ The enumeration is open and a client must tolerate an unknown
    # name, but it must never be told `interrogative` when the truth is
    # that nothing was checked at all.
    from omnisim_bridges.bridge_base import GATE_RULES, refusal_rule
    got = refusal_rule("safety gate unavailable (ImportError)")
    assert got == "gate_unavailable"
    assert got not in GATE_RULES


def test_both_producers_of_gate_refused_spell_the_fallback_alike() -> None:
    # A consumer branching on `rule` cannot tell which producer wrote the
    # event, so the two must not invent different names for the same fact.
    from omnisim_bridges.bridge_base import refusal_rule
    from omnisim_bridges.relay import _refusal_rule as relay_rule
    for reason in ("implausible: distance=300 exceeds the 50 m rail",
                   "safety gate unavailable (ImportError)"):
        assert refusal_rule(reason) == relay_rule(reason)


def test_gate_refusal_records_the_surface_it_arrived_on() -> None:
    ring = ev.EventRing(capacity=8)
    ev.emit_gate_refusal(ring, "drive_forward",
                         "distance=300.0 exceeds the 50 m rail",
                         sim_time=5.0, step=156, robot="husky",
                         origin="tool", rule="implausible",
                         utterance="drive forward 300 metres")
    evt = ring.last()
    assert evt["type"] == "gate.refused"
    assert evt["detail"]["origin"] == "tool"
    assert evt["detail"]["rule"] == "implausible"
    assert evt["detail"]["tool"] == "drive_forward"


# ── The harness pass-through ────────────────────────────────────────

def test_harness_events_are_re_emitted_with_their_source_named() -> None:
    ring = ev.EventRing(capacity=32)
    clock = SimClock(dt_s=0.032)
    for i in range(1, 11):
        clock.tick(i * 0.032)
    feed = ev.HarnessEventFeed(ring, "http://127.0.0.1:6789", robot="husky",
                               clock=clock)
    n = feed.poll_once(lambda cur: {
        "events": [
            {"seq": 4, "type": "damage.impact", "t_sim_ms": 1280,
             "robot": "HUSKY", "severity": 0.4},
            {"seq": 5, "type": "controller.log", "t_sim_ms": 1290,
             "line": "our own stdout coming back around"},
        ],
        "next_since": 5})
    assert n == 1, "the log types must not be forwarded: they would loop"
    evt = ring.last()
    assert evt["type"] == "damage.impact"
    assert evt["source"] == "harness"
    assert evt["detail"]["severity"] == 0.4
    # The harness's own `robot` field would collide with emit()'s keyword
    # parameter; it is kept under a name that says whose it is.
    assert evt["detail"]["harness_robot"] == "HUSKY"
    assert evt["robot"] == "husky"
    # TWO CLOCKS, BOTH REPORTED. `sim_time` is when it ARRIVED here;
    # `harness_t_sim_ms` is when it HAPPENED over there.
    assert abs(evt["sim_time"] - 0.32) < 1e-9
    assert evt["detail"]["harness_t_sim_ms"] == 1280
    assert feed.cursor == 5


def test_a_harness_that_went_away_cannot_stop_the_bridge_reporting() -> None:
    ring = ev.EventRing(capacity=8)

    def _boom(cursor):
        raise OSError("connection refused")

    feed = ev.HarnessEventFeed(ring, "http://127.0.0.1:6789")
    assert feed.poll_once(_boom) == 0
    assert feed.errors == 1
    assert feed.poll_once(lambda c: "not a dict") == 0
    assert feed.errors == 2
    ring.emit("motion.timed_out", sim_time=1.0, step=31)
    assert ring.total == 1


def test_the_harness_cursor_only_moves_forward() -> None:
    ring = ev.EventRing(capacity=8)
    feed = ev.HarnessEventFeed(ring, "http://x")
    feed.poll_once(lambda c: {"events": [], "next_since": 12})
    assert feed.cursor == 12
    feed.poll_once(lambda c: {"events": [], "next_since": 3})
    assert feed.cursor == 12


def test_the_feed_is_off_unless_a_harness_url_names_one(monkeypatch) -> None:
    ring = ev.EventRing(capacity=8)
    monkeypatch.delenv("OMNISIM_HARNESS_URL", raising=False)
    assert ev.harness_feed(ring) is None
    monkeypatch.setenv("OMNISIM_HARNESS_URL", "http://127.0.0.1:6789")
    monkeypatch.setenv("OMNISIM_BRIDGE_HARNESS_EVENTS", "0")
    assert ev.harness_feed(ring) is None, "the opt-out is value-parsed"


# ── The wake policy (decision L4) ───────────────────────────────────

def test_wake_policy_is_a_flat_bool_table() -> None:
    assert isinstance(ev.WAKE_POLICY, dict)
    assert all(isinstance(k, str) for k in ev.WAKE_POLICY)
    assert all(isinstance(v, bool) for v in ev.WAKE_POLICY.values())
    for name in ev.BRIDGE_EVENT_TYPES:
        assert name in ev.WAKE_POLICY, f"{name} has no wake verdict"


def test_only_physical_and_safety_events_wake() -> None:
    waking = {k for k, v in ev.WAKE_POLICY.items() if v}
    assert waking == {
        "motion.timed_out", "joint.limit_hit", "fault.controller_lost",
        "fault.telemetry_stale", "gate.refused", "contact.began",
        "break.hit", "damage.impact", "damage.state_transition",
    }


def test_may_wake_refines_contact_to_non_floor_bodies() -> None:
    assert ev.may_wake({"type": "contact.began", "detail": {"body": "CRATE"}})
    assert not ev.may_wake({"type": "contact.began",
                            "detail": {"body": "FLOOR"}})
    assert not ev.may_wake({"type": "contact.began", "detail": {}})


def test_may_wake_will_not_re_wake_on_the_agents_own_refusal() -> None:
    # A refusal on /prompt is already in the reply the model is reading.
    # Waking it about that is a loop with a bill attached.
    assert ev.may_wake({"type": "gate.refused", "detail": {"origin": "tool"}})
    assert not ev.may_wake({"type": "gate.refused",
                            "detail": {"origin": "prompt"}})
    assert not ev.may_wake({"type": "gate.refused",
                            "detail": {"origin": "relay"}})


def test_may_wake_is_closed_by_default() -> None:
    assert not ev.may_wake({"type": "something.new", "detail": {}})
    assert not ev.may_wake("not an event")


def test_detector_table_covers_every_bridge_type() -> None:
    assert set(ev.EVENT_TYPE_DETECTORS) == set(ev.BRIDGE_EVENT_TYPES)


def test_event_types_are_emitted_through_the_scannable_call_form() -> None:
    # v9 standing rule: a self-scan reads the emitting module's source for
    # `emit("<type>", ...)`. A type name built at runtime is invisible to it.
    import re
    found = set()
    for name in ("events.py", "hold.py"):
        src = (PKG_SRC / "omnisim_bridges" / name).read_text(encoding="utf-8")
        found |= set(re.findall(r'emit\(\s*"([a-z_]+\.[a-z_]+)"', src))
    declared = set(ev.BRIDGE_EVENT_TYPES)
    assert declared - found == set(), f"declared but never emitted: {declared - found}"
    assert found - declared == set(), f"emitted but not declared: {found - declared}"


# ── GET /events ─────────────────────────────────────────────────────

class _StubBridge:
    robot_id = "husky"

    def __init__(self) -> None:
        self.last_tick_at = time.time()
        self.events = ev.EventRing(capacity=16, robot="husky")
        self.sim_time = 4.096
        self.sim_step = 128
        self.fault_detector = ev.FaultDetector(self.events, stale_after_s=2.0)


def test_serve_events_envelope_matches_the_harness_shape() -> None:
    b = _StubBridge()
    b.events.emit("joint.limit_hit", sim_time=4.0, step=125, joint="elbow")
    out = serve_events(b, "/events?since=0&limit=10")
    assert set(["events", "next_since", "dropped", "total"]).issubset(out)
    assert out["robot"] == "husky"
    assert out["sim_time"] == 4.096
    assert len(out["events"]) == 1


def test_serve_events_polls_the_staleness_detector() -> None:
    # The reader is the only thread still running when the sim thread stops,
    # so this is where a stall becomes an event.
    b = _StubBridge()
    b.last_tick_at = 0.0                   # i.e. "never", vs time.time()
    out = serve_events(b, "/events")
    assert any(e["type"] == "fault.telemetry_stale" for e in out["events"])


def test_events_query_is_tolerant_of_a_lost_cursor() -> None:
    assert events_query("/events") == {"since": 0, "limit": 100, "types": None}
    assert events_query("/events?since=12&limit=5")["since"] == 12
    assert events_query("/events?since=banana")["since"] == 0
    assert events_query("/events?types=contact.began,gate.refused")["types"] == [
        "contact.began", "gate.refused"]


def test_serve_events_on_a_bridge_with_no_ring_does_not_explode() -> None:
    class _Old:
        robot_id = "legacy"
    out = serve_events(_Old(), "/events")
    assert out["events"] == [] and "error" in out


# ── D2: the safety_gate block (PROTOCOL §5.2.1) ─────────────────────

def test_safety_gate_block_publishes_the_ungated_half() -> None:
    block = safety_gate_block("mobile",
                              gated_paths=["/prompt", "/tool"],
                              ungated_paths=["/drive_forward", "/turn"])
    assert block["present"] is True
    assert block["implementation"] == "omnisim_bridges.gate"
    assert block["surface"] == "mobile"
    assert block["fails_closed"] is True
    assert block["gated_paths"] == ["/prompt", "/tool"]
    # ⚠️ The load-bearing half. present:true with no ungated_paths reads as
    # "everything here is vetted", which is false of every bridge in this
    # tree.
    assert block["ungated_paths"] == ["/drive_forward", "/turn"]
    assert set(block["rules"]) == set(GATE_RULES)


def test_safety_gate_rails_come_from_the_gate_not_a_copy() -> None:
    from omnisim_bridges import gate as g
    rails = safety_gate_block("mobile")["rails"]
    assert rails["max_distance_m"] == g.MAX_DISTANCE_M
    assert rails["max_angle_rad"] == g.MAX_ANGLE_RAD
    assert rails["max_speed_mps"] == g.MAX_SPEED_MPS
    assert rails["max_altitude_m"] == g.MAX_ALTITUDE_M
    assert rails["max_body_shift_m"] == g.MAX_BODY_SHIFT_M


def test_safety_gate_declares_which_vertical_rail_this_surface_takes() -> None:
    # The whole reason `surface` exists: move_body{vertical} is a 120 m
    # climb on a drone and a 1.0 m body shift on a quadruped.
    assert safety_gate_block("drone")["vertical_rail"] == "max_altitude_m"
    assert safety_gate_block("quadruped")["vertical_rail"] == "max_body_shift_m"


def test_safety_gate_does_not_guess_a_rail_for_a_surfaceless_bridge() -> None:
    # ⚠️ "No surface means the strictest rail" is true of a SHARED tool and
    # false of every other: with no caller surface the gate falls back to the
    # surface RECORDED on the spec whenever exactly one class registered the
    # tool. On the shipped tree only the Mavic registers `move_body`, so a
    # surface-less call to it gets the 120 m AIR rail. A bridge that declares
    # no surface therefore cannot say which rail it will get, and publishing
    # a guess would be publishing a safety property nobody checked.
    assert safety_gate_block(None)["vertical_rail"] is None


# ── `surface` on the wire: accepted, stripped, and NOT honoured ─────

def test_surface_is_stripped_from_the_argument_set() -> None:
    # The platform sends `surface` on every /tool call. Left in the body it
    # reaches the gate as an argument and a perfectly good drive comes back
    # `400 unknown_arg: surface is not a parameter of drive_forward` --
    # which would refuse EVERY tool call against EVERY shipped bridge.
    from omnisim_bridges.bridge_base import tool_args
    body = {"tool": "drive_forward", "id": "req-1", "utterance": "drive 2 m",
            "surface": "mobile", "request_id": "x", "distance": 2.0}
    assert tool_args(body) == {"distance": 2.0}


def test_a_tool_call_behaves_identically_with_and_without_surface() -> None:
    from omnisim_bridges.bridge_base import serve_tool
    seen = []

    def _dispatch(args):
        seen.append(dict(args))
        return {"commanded": 2.0, "achieved": 1.99, "error": -0.01,
                "settled": True}

    plain = serve_tool("drive_forward", {"tool": "drive_forward",
                                         "distance": 2.0},
                       _dispatch, surface="mobile")
    withsurf = serve_tool("drive_forward", {"tool": "drive_forward",
                                            "distance": 2.0,
                                            "surface": "mobile"},
                          _dispatch, surface="mobile")
    assert plain == withsurf
    assert seen[0] == seen[1] == {"distance": 2.0}


def test_a_caller_cannot_buy_a_looser_rail_by_declaring_a_surface() -> None:
    # ⚠️ THE HALF THAT MATTERS. The rail a shared verb is judged against is
    # picked by the surface, so a caller who could declare "drone" would buy
    # the 120 m altitude rail for a quadruped's body shift -- turning a
    # safety property into a client-chosen setting. The bridge's own surface
    # is authoritative; the caller's is accepted for wire compatibility and
    # dropped on the floor.
    from omnisim_bridges import gate as g
    from omnisim_bridges.bridge_base import serve_tool
    g.register_tools([], surface="quadruped")
    called = []
    body = {"tool": "move_body", "vertical": 40.0, "surface": "drone"}
    code, payload = serve_tool("move_body", body,
                               lambda args: called.append(args) or {"ok": True},
                               surface="quadruped")
    assert code == 400, "a caller-declared surface changed the rail"
    assert called == [], "nothing may actuate on a refused call"
    assert payload["error"] == "refused_by_gate"


def test_gate_block_rules_exist_in_gate() -> None:
    # The published list is a promise about what a client may see. If a rule
    # name here has no Rejection site in gate.py, the promise is fiction.
    src = (PKG_SRC / "omnisim_bridges" / "gate.py").read_text(encoding="utf-8")
    for rule in GATE_RULES:
        assert f'"{rule}"' in src, f"{rule} is published but not in gate.py"
    import re
    emitted = set(re.findall(r'Rejection\(\s*[^,]+,\s*"([a-z_]+)"', src))
    # `unknown_tool` is emitted internally and dropped at every call site,
    # so a client never sees one and it is deliberately not published.
    assert emitted - {"unknown_tool"} <= set(GATE_RULES), (
        f"gate.py can emit rules /capabilities does not publish: "
        f"{emitted - {'unknown_tool'} - set(GATE_RULES)}")
