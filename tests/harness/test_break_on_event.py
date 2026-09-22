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

"""Engine-free pins on the pause lease and on break-on-event.

The live-engine half lives in `test_break_integration.py` (gated on
`OMNISIM_HARNESS_INTEGRATION_URL`). What is pinned HERE is everything that can
be decided without a simulator:

* `PauseLease` semantics -- take / extend / expire / release / lift -- against
  a stub supervisor that records its mode changes, because the lease DEADLINE
  is a safety property (a crashed agent must not freeze the engine) and a
  safety property with no test is a hope.
* `step_or_hold`, the one line in the main loop that makes `resume`
  deliverable at all. A plain `supervisor.step()` there deadlocks the whole
  harness; this pins that it does not step while held and DOES resume itself
  when the lease expires.
* `BreakRegistry` arming, matching and refusals. The light-mode refusal is the
  headline: a break armed on `contact.began` in a light session could never
  fire, and accepting it silently would hand an agent a breakpoint that is
  indistinguishable from "the bug did not happen".
* The event-type self-scan: adding `break.hit` must leave
  `verify_event_types()` clean (no `undeclared`, no `declared_not_emitted`),
  because `/capabilities` publishes that verdict.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from conftest import (HARNESS_DIR, SUPERVISOR_DIR,  # noqa: E402
                      exec_supervisor_slice, supervisor_source)

sys.path.insert(0, str(SUPERVISOR_DIR))
sys.path.insert(0, str(HARNESS_DIR))

import event_bus as eb  # noqa: E402
import observe  # noqa: E402


# ---------------------------------------------------------------------------
# Stubs
# ---------------------------------------------------------------------------


class StubSupervisor:
    """Records every simulationSetMode and counts steps.

    `mode` starts at 1 (not PAUSE) so the tests can prove the lease restores
    the caller's ACTUAL mode rather than assuming RUN.
    """

    def __init__(self, mode: int = 1):
        self.mode = mode
        self.mode_log: list[int] = []
        self.steps = 0
        self.step_return = 0

    def simulationGetMode(self) -> int:  # noqa: N802 - engine binding name
        return self.mode

    def simulationSetMode(self, mode: int) -> None:  # noqa: N802
        self.mode = mode
        self.mode_log.append(mode)

    def step(self, ms: int) -> int:
        self.steps += 1
        return self.step_return


class _CommandError(Exception):
    pass


@pytest.fixture(scope="module")
def lease_mod():
    """`PauseLease`, `pause_lifted` and `step_or_hold`, exec'd out of source.

    `harness_supervisor.py` cannot be imported under stock pytest (it needs
    the controller runtime), so the slice pattern in conftest is used. The
    annotations on `pause_lifted` / `step_or_hold` are evaluated at def time
    inside the slice, hence the `Supervisor` placeholder in the namespace.
    """
    import contextlib
    import time
    return exec_supervisor_slice(
        "class PauseLease:", "def dispatch_commands()",
        contextlib=contextlib, time=time, observe=observe,
        Supervisor=object, CommandError=_CommandError)


# ---------------------------------------------------------------------------
# PauseLease
# ---------------------------------------------------------------------------


def test_take_pauses_and_reports_status(lease_mod):
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=1)
    out = lease.take(sup, 5_000, sim_time_ms=1234.0)
    assert lease.held is True
    assert sup.mode == observe.SIMULATION_MODE_PAUSE
    assert out["paused"] is True
    assert out["lease_ms"] == 5_000
    assert out["paused_at_sim_ms"] == 1234.0
    assert 0 < out["lease_remaining_ms"] <= 5_000


def test_second_take_extends_rather_than_failing(lease_mod):
    """A second POST /sim/pause while held must EXTEND, not error.

    And it must not overwrite `prev_mode` with PAUSE -- that would strand the
    caller's real mode and `resume` would restore the wrong one.
    """
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=1)
    lease.take(sup, 1_000, sim_time_ms=0.0)
    first_remaining = lease.remaining_ms()
    first_taken_at = lease.taken_at_ms
    out = lease.take(sup, 60_000, sim_time_ms=999.0)
    assert lease.held is True
    assert lease.prev_mode == 1, "prev_mode was overwritten by the re-take"
    assert lease.taken_at_ms == first_taken_at, "paused_at_sim_ms moved on a re-take"
    assert out["lease_remaining_ms"] > first_remaining
    assert out["lease_ms"] == 60_000


def test_lease_ms_is_clamped_to_the_safety_bounds(lease_mod):
    PauseLease = lease_mod["PauseLease"]
    lease = PauseLease()
    sup = StubSupervisor()
    assert lease.take(sup, 10**9, 0.0)["lease_ms"] == PauseLease.MAX_MS
    lease.release(sup)
    assert lease.take(sup, 1, 0.0)["lease_ms"] == PauseLease.MIN_MS


def test_release_is_idempotent_and_restores_the_prior_mode(lease_mod):
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=2)
    lease.take(sup, 5_000, 0.0)
    first = lease.release(sup)
    assert first == {"paused": False, "was_paused": True, "released_by": "resume"}
    assert sup.mode == 2, "release did not restore the caller's own mode"
    second = lease.release(sup)
    assert second["was_paused"] is False, "a second resume must not be an error"
    assert lease.held is False


def test_expired_reports_true_once_the_deadline_passes(lease_mod, monkeypatch):
    mod_time = lease_mod["time"]
    clock = {"t": 1000.0}
    monkeypatch.setattr(mod_time, "monotonic", lambda: clock["t"])
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor()
    lease.take(sup, 1_000, 0.0)
    assert lease.expired() is False
    clock["t"] += 1.5
    assert lease.expired() is True
    assert lease.remaining_ms() == 0


def test_step_or_hold_does_not_step_while_held(lease_mod, monkeypatch):
    """The whole reason the main loop is not a bare `supervisor.step()`."""
    mod_time = lease_mod["time"]
    clock = {"t": 0.0}
    monkeypatch.setattr(mod_time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(mod_time, "sleep", lambda _s: None)
    step_or_hold = lease_mod["step_or_hold"]
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=1)

    assert step_or_hold(sup, 8, lease) == 0
    assert sup.steps == 1, "a free-running loop must step"

    lease.take(sup, 1_000, 0.0)
    for _ in range(5):
        assert step_or_hold(sup, 8, lease) == 0
    assert sup.steps == 1, "the engine advanced while the lease was held"

    # The deadline is the safety property: a client that dies must not freeze
    # the engine. Nobody calls resume here -- the loop frees itself.
    clock["t"] += 2.0
    assert step_or_hold(sup, 8, lease) == 0
    assert lease.held is False, "the lease did not self-expire"
    assert sup.mode == 1, "expiry did not restore the pre-pause mode"
    assert sup.steps == 2, "the loop did not resume stepping after expiry"


def test_pause_lifted_restores_the_pause_even_when_the_body_raises(lease_mod):
    pause_lifted = lease_mod["pause_lifted"]
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=1)
    lease.take(sup, 5_000, 0.0)
    with pytest.raises(RuntimeError):
        with pause_lifted(sup, lease) as lifted:
            assert lifted is True
            assert sup.mode == 1, "the body did not run against a running engine"
            raise RuntimeError("boom")
    assert sup.mode == observe.SIMULATION_MODE_PAUSE
    assert lease.held is True, "lifting is not releasing"


def test_pause_lifted_is_a_no_op_when_nothing_is_held(lease_mod):
    pause_lifted = lease_mod["pause_lifted"]
    lease = lease_mod["PauseLease"]()
    sup = StubSupervisor(mode=1)
    with pause_lifted(sup, lease) as lifted:
        assert lifted is False
    assert sup.mode_log == []


def test_break_registry_lease_bounds_match_the_lease_itself(lease_mod):
    """`main()` passes these three through; a drift would make the arm-time
    clamp disagree with the take-time clamp."""
    PauseLease = lease_mod["PauseLease"]
    reg = eb.BreakRegistry(eb.EventBus(), 8,
                           lease_default_ms=PauseLease.DEFAULT_MS,
                           lease_min_ms=PauseLease.MIN_MS,
                           lease_max_ms=PauseLease.MAX_MS)
    assert (reg.lease_default_ms, reg.lease_min_ms, reg.lease_max_ms) == (
        PauseLease.DEFAULT_MS, PauseLease.MIN_MS, PauseLease.MAX_MS)
    src = supervisor_source()
    assert "lease_default_ms=PauseLease.DEFAULT_MS" in src
    assert "lease_min_ms=PauseLease.MIN_MS" in src
    assert "lease_max_ms=PauseLease.MAX_MS" in src


# ---------------------------------------------------------------------------
# The declared event vocabulary
# ---------------------------------------------------------------------------


def test_break_hit_is_declared_and_the_self_scan_stays_clean():
    """`/capabilities` publishes this verdict; a drift there is a false claim."""
    assert "break.hit" in eb.SUPERVISOR_EVENT_TYPES
    assert eb.EVENT_TYPE_PRODUCERS["break.hit"] == "BreakRegistry"
    result = eb.verify_event_types(
        (SUPERVISOR_DIR / "event_bus.py").read_text(encoding="utf-8"),
        supervisor_source())
    assert result["undeclared"] == []
    assert result["declared_not_emitted"] == []
    assert result["verified"] is True
    assert "break.hit" in result["emitters_found"]


def test_event_type_count_is_eleven():
    """Seven supervisor producers + break.hit + three harness log types."""
    import omnisim_harness as h
    total = set(eb.SUPERVISOR_EVENT_TYPES) | set(h.LOG_EVENT_TYPES)
    assert len(total) == 11, sorted(total)


def test_break_hit_is_not_silenced_by_light_mode():
    """A light session silences contact/grip/joint -- it must NOT silence the
    record that a break froze the engine, which is what an agent polls for."""
    assert eb.EVENT_TYPE_PRODUCERS["break.hit"] not in eb.LIGHT_MODE_DISABLED_PRODUCERS


# ---------------------------------------------------------------------------
# BreakRegistry: arming + refusals
# ---------------------------------------------------------------------------


def _registry(disabled=(), step_ms=8):
    bus = eb.EventBus()
    return bus, eb.BreakRegistry(bus, step_ms, disabled)


def test_arm_returns_break_id_armed_types_and_diagnostics():
    _bus, reg = _registry()
    out = reg.arm(["contact.began"], {"def": "BOX"}, sim_time_ms=100.0)
    assert out["refused"] is False
    assert out["break_id"]
    assert out["armed_types"] == ["contact.began"]
    assert out["diagnostics"] == []
    assert out["once"] is True
    assert out["armed"] is True
    assert out["hits"] == 0
    assert out["lease_ms"] == reg.lease_default_ms
    assert reg.list_breaks()[0]["break_id"] == out["break_id"]


def test_light_session_refuses_a_contact_break_with_the_named_code():
    """CASE 3 of the live plan, decided here without an engine."""
    _bus, reg = _registry(disabled=("ContactTracker", "JointLimitTracker", "GripTracker"))
    out = reg.arm(["contact.began"], None)
    assert out["refused"] is True
    assert out["code"] == "BREAK_EVENT_TYPE_UNAVAILABLE"
    assert out["armed_types"] == []
    codes = [d["code"] for d in out["diagnostics"]]
    assert codes == ["event_type_silenced_in_light_mode"]
    diag = out["diagnostics"][0]
    assert diag["event_type"] == "contact.began"
    assert diag["producer"] == "ContactTracker"
    assert '"light": false' in diag["workaround"]
    assert set(diag["silenced_types"]) == {
        "contact.began", "contact.ended", "grip.acquired", "grip.released",
        "joint.limit_hit"}
    assert reg.list_breaks() == [], "a refused break must not be armed"


def test_light_session_still_allows_a_damage_break():
    """--light drops three trackers, not the damage tracker. Refusing a
    damage break too would be the opposite dishonesty."""
    _bus, reg = _registry(disabled=("ContactTracker", "JointLimitTracker", "GripTracker"))
    out = reg.arm(["damage.impact"], None)
    assert out["refused"] is False
    assert reg.breakable_types() == ["damage.impact", "damage.state_transition"]


def test_unknown_and_unbreakable_types_are_refused():
    _bus, reg = _registry()
    assert reg.arm(["contact.begun"], None)["diagnostics"][0]["code"] == "event_type_unknown"
    assert reg.arm(["break.hit"], None)["diagnostics"][0]["code"] == "event_type_not_breakable"
    log = reg.arm(["controller.log"], None)
    assert log["diagnostics"][0]["code"] == "event_type_not_on_supervisor_bus"


def test_empty_types_is_refused_and_names_what_is_breakable():
    _bus, reg = _registry()
    out = reg.arm([], None)
    assert out["code"] == "BREAK_TYPES_REQUIRED"
    assert "contact.began" in out["diagnostics"][0]["breakable_types"]


def test_unknown_filter_key_is_refused():
    _bus, reg = _registry()
    out = reg.arm(["contact.began"], {"nodeId": "3"})
    assert out["code"] == "BREAK_FILTER_INVALID"
    assert out["diagnostics"][0]["code"] == "filter_key_unknown"


def test_a_filter_key_no_armed_type_carries_is_a_warning_not_a_refusal():
    """It does not stop the break firing; it stops it NARROWING, which is the
    opposite failure and just as surprising."""
    _bus, reg = _registry()
    out = reg.arm(["joint.limit_hit"], {"def": "ARM"})
    assert out["refused"] is False
    assert out["diagnostics"][0]["code"] == "filter_key_not_matchable_for_armed_types"
    assert out["diagnostics"][0]["key"] == "def"


def test_lease_ms_over_the_cap_is_clamped_with_a_diagnostic():
    _bus, reg = _registry()
    out = reg.arm(["contact.began"], None, lease_ms=10**7)
    assert out["lease_ms"] == reg.lease_max_ms
    assert out["diagnostics"][0]["code"] == "lease_ms_clamped"


def test_break_limit_is_enforced():
    _bus, reg = _registry()
    for _ in range(eb.MAX_ARMED_BREAKS):
        assert reg.arm(["contact.began"], None)["refused"] is False
    over = reg.arm(["contact.began"], None)
    assert over["code"] == "BREAK_LIMIT_REACHED"


def test_remove_reports_whether_anything_was_disarmed():
    _bus, reg = _registry()
    bid = reg.arm(["contact.began"], None)["break_id"]
    assert reg.remove(bid) == {"break_id": bid, "removed": True}
    assert reg.remove(bid) == {"break_id": bid, "removed": False}
    assert reg.list_breaks() == []


def test_declared_diagnostic_codes_cover_every_code_the_registry_emits():
    """The enum `/capabilities` publishes must not drift from the call sites."""
    src = (SUPERVISOR_DIR / "event_bus.py").read_text(encoding="utf-8")
    import re
    emitted = set(re.findall(r'"code": "([a-z_]+)"', src))
    assert emitted <= set(eb.BREAK_DIAGNOSTIC_CODES), sorted(
        emitted - set(eb.BREAK_DIAGNOSTIC_CODES))


# ---------------------------------------------------------------------------
# BreakRegistry: matching
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("filt,evt,expected", [
    ({}, {"type": "contact.began", "a_def": "BOX", "b_def": "FLOOR"}, True),
    ({"def": "BOX"}, {"type": "contact.began", "a_def": "BOX", "b_def": "FLOOR"}, True),
    ({"def": "BOX"}, {"type": "contact.began", "a_def": "FLOOR", "b_def": "BOX"}, True),
    ({"def": "CRATE"}, {"type": "contact.began", "a_def": "BOX", "b_def": "FLOOR"}, False),
    ({"def": "BOX", "counterpart": "FLOOR"},
     {"type": "contact.began", "a_def": "BOX", "b_def": "FLOOR"}, True),
    ({"def": "BOX", "counterpart": "WALL"},
     {"type": "contact.began", "a_def": "BOX", "b_def": "FLOOR"}, False),
    ({"joint": "elbow"}, {"type": "joint.limit_hit", "joint": "elbow"}, True),
    ({"joint": "wrist"}, {"type": "joint.limit_hit", "joint": "elbow"}, False),
    ({"def": "GRIP"}, {"type": "grip.acquired", "gripper_def": "GRIP",
                       "held_def": "CAN"}, True),
])
def test_filter_matching(filt, evt, expected):
    brk = {"types": [evt["type"]], "filter": filt}
    assert eb.BreakRegistry.matches(brk, evt) is expected


def test_a_break_never_matches_a_type_it_was_not_armed_on():
    brk = {"types": ["contact.began"], "filter": {}}
    assert eb.BreakRegistry.matches(
        brk, {"type": "contact.ended", "a_def": "A", "b_def": "B"}) is False


# ---------------------------------------------------------------------------
# BreakRegistry: scan + fire
# ---------------------------------------------------------------------------


class FakeLease:
    def __init__(self):
        self.held = False
        self.takes: list[tuple[int, float]] = []

    def take(self, supervisor, lease_ms, sim_time_ms):  # noqa: ARG002
        self.held = True
        self.takes.append((lease_ms, sim_time_ms))
        return {"paused": True, "lease_remaining_ms": lease_ms,
                "paused_at_sim_ms": sim_time_ms}


def test_scan_fires_takes_the_lease_and_emits_break_hit():
    bus, reg = _registry()
    reg.arm(["contact.began"], {"def": "BOX"}, lease_ms=5_000)
    lease = FakeLease()

    bus.emit("contact.began", {"a_def": "OTHER", "b_def": "FLOOR"}, t_sim_ms=80)
    assert reg.scan(None, lease, 80.0) == []
    assert lease.held is False, "a non-matching event took the lease"

    bus.emit("contact.began", {"a_def": "BOX", "b_def": "FLOOR"}, t_sim_ms=496)
    hits = reg.scan(None, lease, 496.0)
    assert len(hits) == 1
    hit = hits[0]
    assert lease.held is True
    assert lease.takes == [(5_000, 496.0)]
    assert hit["matched_type"] == "contact.began"
    assert hit["paused_at_sim_ms"] == 496.0
    assert hit["hold_latency_ms"] == 0.0
    assert hit["hold_latency_steps"] == 0
    assert hit["paused"] is True
    assert hit["event"]["a_def"] == "BOX"

    emitted = [e for e in bus.since(0) if e["type"] == "break.hit"]
    assert len(emitted) == 1
    assert emitted[0]["break_id"] == hit["break_id"]
    assert emitted[0]["t_sim_ms"] == 496
    assert reg.last_hit["break_id"] == hit["break_id"]


def test_hold_latency_is_measured_not_assumed():
    """An event seen one tick late reports it, rather than claiming zero."""
    bus, reg = _registry(step_ms=8)
    reg.arm(["contact.began"], None)
    lease = FakeLease()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=1000)
    hit = reg.scan(None, lease, 1024.0)[0]
    assert hit["hold_latency_ms"] == 24.0
    assert hit["hold_latency_steps"] == 3


def test_once_disarms_and_a_repeating_break_does_not():
    bus, reg = _registry()
    once_id = reg.arm(["contact.began"], {"def": "A"}, once=True)["break_id"]
    many_id = reg.arm(["contact.ended"], {"def": "A"}, once=False)["break_id"]
    lease = FakeLease()

    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=8)
    bus.emit("contact.ended", {"a_def": "A", "b_def": "B"}, t_sim_ms=16)
    reg.scan(None, lease, 16.0)
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=24)
    bus.emit("contact.ended", {"a_def": "A", "b_def": "B"}, t_sim_ms=32)
    reg.scan(None, lease, 32.0)

    table = {b["break_id"]: b for b in reg.list_breaks()}
    assert table[once_id]["armed"] is False and table[once_id]["hits"] == 1
    assert table[many_id]["armed"] is True and table[many_id]["hits"] == 2


def test_a_break_cannot_fire_on_events_that_predate_it():
    bus, reg = _registry()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=8)
    reg.arm(["contact.began"], None)
    lease = FakeLease()
    assert reg.scan(None, lease, 16.0) == []
    assert lease.held is False


def test_break_hit_does_not_retrigger_a_break():
    """A `break.hit` on the bus must never match anything, or one hit would
    cascade into an unbounded emit loop."""
    bus, reg = _registry()
    reg.arm(["contact.began"], None, once=False)
    lease = FakeLease()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=8)
    reg.scan(None, lease, 8.0)
    before = bus.total
    reg.scan(None, lease, 16.0)
    reg.scan(None, lease, 24.0)
    assert bus.total == before, "the break re-fired on its own break.hit record"


def test_scan_with_nothing_armed_is_cheap_and_advances_the_cursor():
    bus, reg = _registry()
    for i in range(50):
        bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=i)
    lease = FakeLease()
    assert reg.scan(None, lease, 50.0) == []
    reg.arm(["contact.began"], None)
    assert reg.scan(None, lease, 51.0) == [], "armed on history"


def test_a_failed_lease_take_is_recorded_not_swallowed():
    class ExplodingLease:
        held = False

        def take(self, *_a, **_k):
            raise RuntimeError("no simulation mode control on this build")

    bus, reg = _registry()
    reg.arm(["contact.began"], None)
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=8)
    hit = reg.scan(None, ExplodingLease(), 8.0)[0]
    assert hit["paused"] is False
    assert "no simulation mode control" in hit["lease_error"]


# ---------------------------------------------------------------------------
# Harness surface
# ---------------------------------------------------------------------------


def test_harness_publishes_the_break_routes_and_the_feature():
    import omnisim_harness as h
    paths = {(r["method"], r["path"]) for r in h.ROUTES}
    assert ("POST", "/sim/break") in paths
    assert ("GET", "/sim/breaks") in paths
    assert ("DELETE", "/sim/break/<break_id>") in paths
    assert ("POST", "/sim/pause") in paths
    assert ("POST", "/sim/resume") in paths


def test_break_refusal_codes_are_discoverable():
    import omnisim_harness as h
    codes = set(h.known_request_error_codes())
    assert "BREAK_EVENT_TYPE_UNAVAILABLE" in codes
    assert "BREAK_NOT_FOUND" in codes
    assert set(h.BREAK_REFUSAL_CODES) <= codes


def test_nothing_lists_pause_or_break_as_unsupported():
    """C1's capability check: `sim.pause` sat in `not_supported` until
    0aafab096 shipped the route. It must not creep back."""
    import omnisim_harness as h
    src = (HARNESS_DIR / "omnisim_harness.py").read_text(encoding="utf-8")
    features = src[src.index('features = ['):src.index('not_supported = [')]
    for name in ("sim.pause", "sim.resume", "sim.break"):
        assert f'"{name}"' in features
    table = src[src.index('not_supported = ['):]
    table = table[:table.index('return {')]
    for name in ("sim.pause", "sim.resume", "sim.break"):
        assert f'"feature": "{name}"' not in table
    # sim.watch IS a real gap (v9 D7) and must stay declared.
    assert any(e["feature"] == "sim.watch" for e in _not_supported_static(h))


def _not_supported_static(h) -> list[dict]:
    """The literal `not_supported` rows, read without a live session."""
    src = (HARNESS_DIR / "omnisim_harness.py").read_text(encoding="utf-8")
    body = src[src.index('not_supported = ['):]
    return [{"feature": m} for m in __import__("re").findall(
        r'\{"feature": "([^"]+)"', body[:body.index('] + ENGINE_NOT_SUPPORTED')])]
