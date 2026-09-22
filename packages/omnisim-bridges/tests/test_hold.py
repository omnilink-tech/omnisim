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

"""The lockstep hold (Track D6), driven by a stub supervisor.

⚠️ THIS FILE EXISTS BECAUSE THE FIRST HOLD IN THIS TREE SHIPPED WITHOUT ONE.
`PauseLease` landed on 2026-09-15 with no test and seven defects, and was
only fixed on 2026-09-22. Four of those seven are behavioural and each has a
test below named after it:

  - a frozen clock over a moving engine  -> test_take_flushes_the_queued_pause
  - an expiry that hung the loop forever -> test_expiry_restores_with_a_read
  - a lease surviving a world load       -> test_release_is_idempotent_...
  - a re-take overwriting the caller's mode -> test_extending_a_hold_keeps_...

Engine-free: `_StubSupervisor` records what a real one would have been told.
"""

from __future__ import annotations

import pathlib
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any, Dict, List

import pytest

PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))
_TESTS = pathlib.Path(__file__).resolve().parent
if str(_TESTS) not in sys.path:
    sys.path.insert(0, str(_TESTS))

# Helpers only (the controller loader and the fake bridges), imported as a
# module so none of its tests are collected a second time from this file.
import test_step_budget as _sb                        # noqa: E402
from omnisim_bridges.bridge_base import SimClock     # noqa: E402

from omnisim_bridges import events as ev              # noqa: E402
from omnisim_bridges.hold import (                    # noqa: E402
    SIMULATION_MODE_PAUSE,
    HoldLease,
    env_flag,
    lockstep_enabled,
)

MODE_RUN = 1


class _StubNode:
    def __init__(self, owner):
        self.owner = owner

    def getPosition(self):
        self.owner.round_trips += 1
        return [0.0, 0.0, 0.0]


class _StubSupervisor:
    """Records the mode traffic a real Supervisor would have seen."""

    def __init__(self, mode: int = MODE_RUN, has_self: bool = True) -> None:
        self.mode = mode
        self.modes_set: list = []
        self.round_trips = 0
        self.steps = 0
        self._node = _StubNode(self) if has_self else None

    def simulationGetMode(self) -> int:
        return self.mode

    def simulationSetMode(self, mode: int) -> None:
        self.mode = mode
        self.modes_set.append(mode)

    def getSelf(self):
        return self._node

    def step(self, timestep: int) -> int:
        self.steps += 1
        return 0


# ── The hatch ───────────────────────────────────────────────────────

def test_lockstep_is_off_by_default(monkeypatch) -> None:
    monkeypatch.delenv("OMNISIM_BRIDGE_LOCKSTEP", raising=False)
    assert lockstep_enabled() is False


def test_the_hatch_is_value_parsed_not_presence_gated(monkeypatch) -> None:
    # Every presence-gated flag in this tree has eventually surprised
    # somebody who set it to 0 expecting that to turn something off.
    monkeypatch.setenv("OMNISIM_BRIDGE_LOCKSTEP", "1")
    assert lockstep_enabled() is True
    for off in ("0", "false", "no", "off", ""):
        monkeypatch.setenv("OMNISIM_BRIDGE_LOCKSTEP", off)
        assert lockstep_enabled() is False, off
    monkeypatch.setenv("OMNISIM_BRIDGE_LOCKSTEP", "yes")
    assert lockstep_enabled() is True
    # OMNISIM_NOT_SET_ANYWHERE is a TEST SENTINEL, not a product hatch: a name
    # deliberately never set anywhere, used to prove env_flag returns its
    # caller's default rather than False when a variable is absent. It exists
    # only in this assertion and must never be read by shipped code.
    assert env_flag("OMNISIM_NOT_SET_ANYWHERE", default=True) is True


def test_a_disabled_hold_is_a_plain_step() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=False)
    out = hold.take(sup, sim_time=1.0, step=31)
    assert hold.held is False
    assert "refused" in out
    assert sup.modes_set == []                 # never touched the engine
    assert hold.step_or_hold(sup, 32, sim_time=1.0, step=31) == 0
    assert sup.steps == 1                      # and it DID step


# ── Defect 1: a frozen clock over a moving engine ───────────────────

def test_take_flushes_the_queued_pause() -> None:
    # `simulationSetMode` only QUEUES; the request rides the controller's
    # next round trip, and a held loop makes none. Measured before the
    # harness fix: a break fired at t=168 ms and the engine ran to ~400 ms,
    # ~29 basic steps of scene motion after the "freeze".
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=2.0, step=62)
    assert hold.held is True
    assert sup.modes_set == [SIMULATION_MODE_PAUSE]
    assert sup.round_trips == 1                # the flush actually happened


def test_a_hold_does_not_step() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=2.0, step=62)
    for _ in range(3):
        assert hold.step_or_hold(sup, 32, sim_time=2.0, step=62) == 0
    assert sup.steps == 0


# ── Defect 2: an expiry that hung the loop forever ──────────────────

def test_expiry_restores_with_a_read_and_the_loop_resumes() -> None:
    # The harness's version left the un-pause for the loop's next step(),
    # and a step against a paused engine blocks -- engine and controller
    # each waiting on the other. A 1 s lease expired and the supervisor
    # never answered again, which turns the deadline (the safety property)
    # into the thing that breaks it.
    sup = _StubSupervisor()
    ring = ev.EventRing(capacity=8)
    hold = HoldLease(events=ring, robot_id="husky", enabled=True)
    hold.take(sup, lease_ms=HoldLease.MIN_MS, sim_time=1.0, step=31)
    hold.deadline = time.monotonic() - 0.001        # as if the lease ran out
    assert hold.expired() is True
    rc = hold.step_or_hold(sup, 32, sim_time=1.0, step=31)
    assert hold.held is False
    assert sup.modes_set[-1] == MODE_RUN            # restored, not left paused
    assert sup.round_trips == 2                     # take flush + release flush
    assert rc == 0 and sup.steps == 1               # and it stepped again


def test_expiry_is_announced_on_the_ring() -> None:
    sup = _StubSupervisor()
    ring = ev.EventRing(capacity=8)
    hold = HoldLease(events=ring, robot_id="husky", enabled=True)
    hold.take(sup, sim_time=5.0, step=156)
    hold.deadline = time.monotonic() - 0.001
    hold.step_or_hold(sup, 32, sim_time=5.0, step=156)
    evt = ring.last()
    assert evt["type"] == "hold.expired"
    assert evt["robot"] == "husky"
    assert evt["sim_time"] == 5.0


# ── Defect 3: a lease that outlived what took it ────────────────────

def test_release_is_idempotent_and_safe_on_an_unheld_lease() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    out = hold.release(sup, reason="world_load")
    assert out["was_held"] is False and hold.held is False
    hold.take(sup, sim_time=0.0, step=0)
    assert hold.release(sup)["was_held"] is True
    assert hold.release(sup)["was_held"] is False
    assert hold.held is False


# ── Defect 4: a re-take overwriting the caller's mode ───────────────

def test_extending_a_hold_keeps_the_original_mode() -> None:
    # Capture prev_mode ONCE. Re-taking a live lease must not overwrite it
    # with PAUSE and strand the caller there on release.
    sup = _StubSupervisor(mode=MODE_RUN)
    hold = HoldLease(enabled=True)
    hold.take(sup, lease_ms=5_000, sim_time=0.0, step=0)
    first_deadline = hold.deadline
    hold.take(sup, lease_ms=60_000, sim_time=1.0, step=31)
    assert hold.prev_mode == MODE_RUN
    assert hold.deadline > first_deadline           # extended
    assert hold.holds_taken == 1                    # extended, not re-entered
    hold.release(sup)
    assert sup.mode == MODE_RUN


# ── The lease bounds ────────────────────────────────────────────────

def test_lease_is_clamped_at_both_ends() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    assert hold.take(sup, lease_ms=1)["lease_ms"] == HoldLease.MIN_MS
    hold.release(sup)
    assert hold.take(sup, lease_ms=10**9)["lease_ms"] == HoldLease.MAX_MS


def test_lease_default_has_an_env_override(monkeypatch) -> None:
    monkeypatch.setenv("OMNISIM_BRIDGE_HOLD_LEASE_MS", "5000")
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    assert hold.take(sup)["lease_ms"] == 5000
    hold.release(sup)
    monkeypatch.setenv("OMNISIM_BRIDGE_HOLD_LEASE_MS", "banana")
    assert hold.take(sup)["lease_ms"] == HoldLease.DEFAULT_MS


def test_remaining_ms_is_zero_when_not_held() -> None:
    hold = HoldLease(enabled=True)
    assert hold.remaining_ms() == 0
    assert hold.status()["held"] is False


# ── Lifting for a motion ────────────────────────────────────────────

def test_lifted_runs_the_body_unpaused_and_re_takes_after() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=0.0, step=0)
    sup.modes_set.clear()
    with hold.lifted(sup) as lifted:
        assert lifted is True
        assert sup.mode == MODE_RUN            # the motion CAN advance
        assert hold.held is True               # but the lease is not released
    assert sup.mode == SIMULATION_MODE_PAUSE
    assert sup.modes_set == [MODE_RUN, SIMULATION_MODE_PAUSE]


def test_lifted_re_takes_even_when_the_body_raised() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=0.0, step=0)
    try:
        with hold.lifted(sup):
            raise RuntimeError("the motion blew up")
    except RuntimeError:
        pass
    assert sup.mode == SIMULATION_MODE_PAUSE   # not stranded running


def test_lifted_on_an_unheld_lease_does_nothing() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    with hold.lifted(sup) as lifted:
        assert lifted is False
    assert sup.modes_set == []


# ── The window leak, counted rather than hidden ─────────────────────

def test_the_window_leak_is_bounded_and_counted() -> None:
    # `wwiReceiveText` needs a step, so a fully held loop makes the robot
    # window mute. The loop pumps at most one step per second while held,
    # ONLY when the window is open, and publishes the count -- because any
    # non-zero value is world motion during a nominally frozen run.
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=0.0, step=0)
    hold.LEAK_PERIOD_S = 0.05
    leaked = 0
    for _ in range(4):
        if hold.step_or_hold(sup, 32, window_open=True) == 0:
            pass
        leaked = hold.leak_steps
    assert leaked == 1                          # one per period, not per call
    assert hold.status()["leak_steps"] == 1
    time.sleep(0.06)
    hold.step_or_hold(sup, 32, window_open=True)
    assert hold.leak_steps == 2


def test_no_leak_when_the_window_is_closed() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=0.0, step=0)
    for _ in range(5):
        hold.step_or_hold(sup, 32, window_open=False)
    assert hold.leak_steps == 0
    assert sup.steps == 0


# ── A supervisor that cannot do any of this ─────────────────────────

def test_lockstep_declines_a_robot_it_cannot_reach_the_engine_through() -> None:
    # `supervisor TRUE` is set by all 22 chat worlds (plan L5), but a bridge
    # copied into a world that forgot must neither die NOR hold blind. This
    # used to "degrade" by holding through the coupling alone -- and a hold
    # that cannot make a round trip cannot learn that its engine has gone,
    # which is exactly how a held bridge outlived its world on its port
    # (see test_a_held_bridge_dies_with_its_engine). So it declines, and
    # says why in the status a client reads.
    class _Plain:
        def __init__(self):
            self.steps = 0

        def step(self, ts):
            self.steps += 1
            return 0

    plain = _Plain()
    hold = HoldLease(enabled=True)
    out = hold.take(plain, sim_time=0.0, step=0)
    assert hold.held is False
    assert "supervisor" in out["refused"]
    assert "supervisor" in hold.status()["declined"]
    # The loop keeps stepping: the world runs free rather than freezing.
    for step in range(10):
        assert hold.sync(plain, busy=False, step=step) is None
        assert hold.step_or_hold(plain, 32) == 0
    assert plain.steps == 10


# ── The defect lockstep added: a held bridge outlived its engine ────

class _DeadEngine(Exception):
    """Stands in for libController's `exit(1)` on a broken pipe."""


class _MortalSupervisor(_StubSupervisor):
    """A supervisor whose engine can die. After `die()`, any round trip
    ends the process, exactly as `g_pipe.c: broken_pipe()` does."""

    def __init__(self) -> None:
        super().__init__()
        self.dead = False
        owner = self

        class _Node:
            def getPosition(self):
                if owner.dead:
                    raise _DeadEngine("pipe broken: exit(1)")
                owner.round_trips += 1
                return [0.0, 0.0, 0.0]

        self._node = _Node()

    def die(self) -> None:
        self.dead = True

    def step(self, timestep: int) -> int:
        if self.dead:
            raise _DeadEngine("pipe broken: exit(1)")
        return super().step(timestep)


def test_a_held_loop_reaches_the_engine_every_heartbeat() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=1.0, step=31)
    after_take = sup.round_trips
    # Not yet due: a held loop spins every 5 ms and must not read each time.
    hold.step_or_hold(sup, 32)
    assert sup.round_trips == after_take
    hold._last_heartbeat -= HoldLease.HEARTBEAT_S       # a period passes
    assert hold.step_or_hold(sup, 32) == 0
    assert sup.round_trips == after_take + 1
    assert hold.heartbeats == 1
    # ...and it is a READ, never a step: the world stays frozen.
    assert sup.steps == 0 and hold.held is True


def test_a_held_bridge_dies_with_its_engine() -> None:
    # MEASURED 2026-09-22 (the determinism probe, --lockstep): the probe's
    # teardown killed a trial's engine while its bridge was HELD. A held
    # loop makes no round trips, so the bridge never found out, kept
    # serving HTTP on 8765, and the next trial's bridge bound the same port
    # beside it. The next request landed on the orphan, whose first robot
    # read hit the dead pipe: `ConnectionResetError`, and no traceback
    # anywhere. A free-running bridge dies inside robot.step() instead.
    sup = _MortalSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=1.0, step=31)
    sup.die()
    # Within one heartbeat period, the held loop must touch the engine and
    # so meet the broken pipe -- here, the stand-in for exit(1).
    deadline = time.monotonic() + HoldLease.HEARTBEAT_S + 1.0
    with pytest.raises(_DeadEngine):
        while time.monotonic() < deadline:
            hold.step_or_hold(sup, 32)
    assert time.monotonic() < deadline


def test_a_heartbeat_that_cannot_reach_the_engine_stops_holding() -> None:
    # No round trip possible at all (no self node any more): holding blind
    # is the failure mode, so release and take a real step -- harmless on a
    # live engine, fatal to the process on a dead one, as free-running is.
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.take(sup, sim_time=1.0, step=31)
    sup._node = None
    hold._last_heartbeat -= HoldLease.HEARTBEAT_S
    assert hold.step_or_hold(sup, 32) == 0
    assert hold.held is False
    assert sup.steps == 1
    assert sup.mode == MODE_RUN                          # not stranded paused


# ── Defect 7 again: an expired lease taken straight back ───────────

def test_an_expired_lease_is_not_taken_straight_back() -> None:
    # MEASURED 2026-09-22: after a 30 s lease expired, sync() re-took the
    # hold on the next idle iteration. The world advanced exactly ONE basic
    # step per lease (sim_time 0.064 -> 0.080 at t=30 s, then frozen again),
    # so a dead client froze the demo anyway -- the thing the lease exists
    # to prevent.
    sup = _StubSupervisor()
    ring = ev.EventRing(capacity=8)
    hold = HoldLease(events=ring, enabled=True)
    hold.sync(sup, busy=False, step=0)
    assert hold.sync(sup, busy=False, step=10) == "taken"
    hold.deadline = time.monotonic() - 0.001            # the lease ran out
    hold.step_or_hold(sup, 32, step=10)
    assert hold.held is False
    assert ring.last()["type"] == "hold.expired"
    for step in range(11, 200):                         # nobody asks for anything
        assert hold.sync(sup, busy=False, step=step) is None
        hold.step_or_hold(sup, 32, step=step)
    assert hold.held is False
    assert hold.status()["armed"] is False
    assert sup.steps == 1 + 189                         # it really runs free
    # The next command re-arms it, and the world is held again once that
    # command has settled.
    hold.sync(sup, busy=True, step=200)
    assert hold.status()["armed"] is True
    for step in range(201, 204):
        assert hold.sync(sup, busy=False, step=step) is None
    assert hold.sync(sup, busy=False, step=204) == "taken"


# ── A wait begun inside a hold is not a stall ───────────────────────

def test_a_wait_begun_inside_a_hold_is_not_a_stall() -> None:
    # MEASURED 2026-09-22: `POST /drive_forward {"wait": true}` issued 6 s
    # into a hold answered in 0.0 s with `stalled: true, steps: 0` -- while
    # the robot then drove 0.9855 m. `last_tick_at` had been frozen by the
    # hold, so the wall-clock stall test fired on the waiter's first poll,
    # before the loop had even seen the work. A stall is "no tick for the
    # grace since the wait BEGAN".
    clock = SimClock(dt_s=0.032)
    budget = clock.budget(5.0)
    held_since = time.time() - 6.0
    assert budget.stalled(last_tick_wall=held_since) is False
    # A world that then really does not step is still a stall.
    time.sleep(0.05)
    assert budget.stalled(last_tick_wall=held_since, grace_s=0.02) is True


def _feed_completion_after(bridge, delay_s: float, seq: int = 1) -> None:
    def _run():
        time.sleep(delay_s)               # the loop sees the work, releases
        bridge.tick(3)
        bridge.last_completion = {"seq": seq, "verb": "drive_forward",
                                  "achieved": 0.9855, "settled": True,
                                  "timed_out": False}
    threading.Thread(target=_run, daemon=True).start()


@pytest.mark.parametrize("name", ["mobile", "arm"])
def test_a_blocking_motion_issued_during_a_hold_is_measured(name) -> None:
    # The shipped wait loops, lifted out of the controllers the way
    # test_step_budget does. Six seconds of hold, then the motion runs.
    path, factory = {"mobile": (_sb.MOBILE, _sb._FakeMobile),
                     "arm": (_sb.ARM, _sb._FakeArm)}[name]
    fn = _sb.load_method(path, "_await_completion", {"time": time})
    bridge = factory()
    bridge.last_tick_at = time.time() - 6.0
    _feed_completion_after(bridge, 0.3)
    out = fn(bridge, 1, 5.0)
    assert out.get("stalled") is not True, (
        f"the {name} bridge called a held world a stalled one: {out}")
    assert out["achieved"] == 0.9855


def test_an_arm_stop_issued_during_a_hold_measures_its_rest() -> None:
    # Same trap on the STOP path, which lockstep promises always runs: the
    # stop lifted the hold, but its rest measurement checked the frozen
    # `last_tick_at` first and reported "the simulation is not stepping".
    fake = SimpleNamespace(
        STOP_SETTLE_S=0.30, STOP_TAIL_FRAC=0.5, STOP_MIN_SPAN_S=0.04,
        WAIT_STALL_S=2.0, WAIT_POLL_S=0.02, lock=threading.RLock(),
        _q_hist=[], joint_names=["j1"], last_tick_at=time.time() - 6.0)
    # Load BEFORE the world moves: parsing the controller takes long enough
    # that ticks started first would refresh `last_tick_at` and hide the bug.
    fn = _sb.load_method(_sb.ARM, "_stop_measure_rest",
                         {"time": time, "threading": threading,
                          "Dict": Dict, "Any": Any, "List": List})
    t_halt = time.time()

    def _ticks():
        time.sleep(0.05)                     # the loop picks up the release
        for _ in range(20):
            time.sleep(0.02)
            now = time.time()
            with fake.lock:
                fake._q_hist.append((now, [0.25]))
            fake.last_tick_at = now
    threading.Thread(target=_ticks, daemon=True).start()
    box: dict = {}
    worker = threading.Thread(               # never the main thread: see it
        target=lambda: box.update(out=fn(fake, t_halt, [0.25])))
    worker.start()
    worker.join(5.0)
    out = box["out"]
    assert "not stepping" not in (out.get("reason") or ""), out
    assert out["max_joint_radps"] == 0.0


# ── The loop's state machine: hold while idle, run while working ────

def test_sync_holds_only_after_the_motion_has_settled() -> None:
    # Re-freezing on the same step the motion ended would measure a stopped
    # world and call it a settled robot: the final zero-velocity write and
    # the pose samples the result is measured from both need steps.
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    assert hold.sync(sup, busy=True, step=100) is None
    assert hold.held is False
    for step in range(101, 104):               # settle_steps default is 4
        assert hold.sync(sup, busy=False, step=step) is None
    assert hold.sync(sup, busy=False, step=104) == "taken"
    assert hold.held is True


def test_sync_releases_the_moment_there_is_work() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.sync(sup, busy=False, step=0)
    hold.sync(sup, busy=False, step=10)
    assert hold.held is True
    assert hold.sync(sup, busy=True, step=11) == "released"
    assert hold.held is False
    assert sup.mode == MODE_RUN


def test_a_stop_lifts_the_hold_from_another_thread_without_touching_the_api() -> None:
    # ⚠️ An HTTP thread must never call release() itself: it touches
    # simulationSetMode and flushes it with a supervisor read, and under a
    # hold the sim thread is parked so nothing would service that pipe.
    # The request is a flag; the LOOP does the work on the right thread.
    sup = _StubSupervisor()
    hold = HoldLease(enabled=True)
    hold.sync(sup, busy=False, step=0)
    hold.sync(sup, busy=False, step=10)
    assert hold.held is True
    before = list(sup.modes_set)
    hold.request_release("stop_robot")          # the HTTP thread's whole job
    assert sup.modes_set == before              # it touched nothing
    assert hold.sync(sup, busy=False, step=11) == "released"
    assert hold.held is False


def test_sync_is_inert_when_lockstep_is_off() -> None:
    sup = _StubSupervisor()
    hold = HoldLease(enabled=False)
    for step in range(20):
        assert hold.sync(sup, busy=False, step=step) is None
    assert hold.held is False
    assert sup.modes_set == []


def test_flush_survives_a_supervisor_with_no_self_node() -> None:
    assert HoldLease.flush(_StubSupervisor(has_self=False)) is False
    assert HoldLease.flush(object()) is False
