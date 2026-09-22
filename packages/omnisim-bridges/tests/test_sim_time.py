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

"""The bridge's two clocks (Track D1).

THE BUG THIS FILE PINS. `last_tick_at` was `time.time()` on every bridge
while PROTOCOL.md §5.3 has always shown it in SIM seconds -- 1.7 billion
where the spec's own example reads 12.448. A client differencing it against
`sim_time` got the age of the Unix epoch, and nothing in the wire contract
said which ruler it was holding.

The second half is that the two clocks are DIFFERENT RULERS and mixing them
silently truncates work: a wall-clock wait on a sim running at 0.4x realtime
gives the robot 40% of the time it was asked for and then reports that the
robot timed out.

Engine-free.
"""

from __future__ import annotations

import pathlib
import sys
import time

PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges.bridge_base import (        # noqa: E402
    BridgeBase,
    SimClock,
    StepBudget,
    attach_telemetry,
    events_summary,
    telemetry_tick,
)


# ── SimClock ────────────────────────────────────────────────────────

def test_the_clock_counts_sim_seconds_and_steps() -> None:
    clock = SimClock(dt_s=0.032)
    assert clock.sim_time == 0.0 and clock.step == 0
    for i in range(1, 11):
        clock.tick(i * 0.032)
    assert clock.step == 10
    assert abs(clock.sim_time - 0.32) < 1e-9


def test_the_clock_falls_forward_when_the_engine_gives_no_time() -> None:
    # A bridge that integrates its own clock (the Mavic) passes None.
    clock = SimClock(dt_s=0.008)
    for _ in range(125):
        clock.tick()
    assert clock.step == 125
    assert abs(clock.sim_time - 1.0) < 1e-9


def test_a_junk_time_does_not_stall_the_clock() -> None:
    clock = SimClock(dt_s=0.032)
    clock.tick(None)
    clock.tick("not a number")
    assert clock.step == 2
    assert clock.sim_time > 0.0


def test_the_sim_clock_and_the_wall_clock_are_different_rulers() -> None:
    # The whole point. 100 sim-steps of a 32 ms world is 3.2 SIM seconds and
    # very nearly no wall time at all -- or, on a loaded machine, eight wall
    # seconds. Neither number is convertible into the other, and a wait that
    # believes it can is the truncation bug.
    clock = SimClock(dt_s=0.032)
    wall0 = time.time()
    for i in range(1, 101):
        clock.tick(i * 0.032)
    assert abs(clock.sim_time - 3.2) < 1e-9
    assert (time.time() - wall0) < 3.0
    assert clock.wall_time >= wall0


def test_steps_for_converts_a_timeout_into_a_budget() -> None:
    clock = SimClock(dt_s=0.032)
    assert clock.steps_for(3.2) == 100
    assert clock.steps_for(0.0) == 1          # never a zero-step wait
    assert clock.steps_for(-5) == 1
    assert clock.steps_for("nonsense") == 1


def test_a_degenerate_timestep_does_not_divide_by_zero() -> None:
    assert SimClock(dt_s=0.0).dt > 0.0
    assert SimClock(dt_s=-1).dt > 0.0


# ── StepBudget ──────────────────────────────────────────────────────

def test_a_budget_expires_on_steps_not_on_seconds() -> None:
    clock = SimClock(dt_s=0.032)
    budget = clock.budget(0.32)               # ten steps
    assert budget.steps == 10
    for _ in range(9):
        clock.tick()
        assert not budget.expired()
    clock.tick()
    assert budget.expired()
    assert budget.elapsed_steps == 10


def test_a_frozen_world_never_expires_a_budget() -> None:
    # ⚠️ AND THAT IS CORRECT. A world that is not stepping has not used up
    # the robot's time; it has stopped giving it any. Reporting that as a
    # timeout tells an agent to reissue a command into a sim that is not
    # running, which is the one thing it must not do.
    clock = SimClock(dt_s=0.032)
    budget = clock.budget(0.1)
    time.sleep(0.15)
    assert not budget.expired()
    # The wall clock keeps exactly one job, and it is a different question.
    # The grace runs from when the WAIT began (a lockstep hold freezes the
    # last tick on purpose -- see StepBudget.stalled), so it is shorter than
    # the 0.15 s this wait has already sat through.
    assert budget.stalled(last_tick_wall=time.time() - 10.0,
                          grace_s=0.1) is True
    assert budget.stalled(last_tick_wall=time.time(), grace_s=0.1) is False


def test_the_budget_reports_the_window_it_measured_over() -> None:
    clock = SimClock(dt_s=0.032)
    for i in range(1, 11):
        clock.tick(i * 0.032)
    budget = clock.budget(1.0)
    for i in range(11, 21):
        clock.tick(i * 0.032)
    out = budget.result()
    assert set(out) == {"sim_time_start", "sim_time_end", "steps"}
    assert abs(out["sim_time_start"] - 0.32) < 1e-6
    assert abs(out["sim_time_end"] - 0.64) < 1e-6
    assert out["steps"] == 10


def test_remaining_steps_never_goes_negative() -> None:
    clock = SimClock(dt_s=0.032)
    budget = StepBudget(clock, 0.064)
    for _ in range(10):
        clock.tick()
    assert budget.remaining_steps() == 0
    assert budget.elapsed_steps == 10


def test_stalled_tolerates_junk() -> None:
    budget = SimClock().budget(1.0)
    assert budget.stalled(last_tick_wall=None) is False


# ── The BridgeBase contract other agents read ───────────────────────

class _Stub(BridgeBase):
    robot_id = "stub"
    model = "StubBot"

    def act_stop(self):
        return {"halted_at": time.time()}


def test_the_six_telemetry_names_exist_without_an_init() -> None:
    # ⚠️ THE DOCUMENTED WAY TO WRITE A BRIDGE IS TO SUBCLASS AND DEFINE YOUR
    # OWN __init__ WITHOUT CALLING super() -- this package's own quickstart
    # does exactly that. A contract that only holds for cooperative
    # subclasses is not a contract, so every name below resolves lazily.
    b = _Stub()
    assert isinstance(b.sim_time, float)
    assert isinstance(b.sim_step, int)
    assert b.held is False
    assert b.fault is None
    assert b.world == ""
    assert b.events is not None
    assert b.events.total == 0


def test_the_clock_tick_advances_what_readers_read() -> None:
    b = _Stub()
    b.sim_clock = SimClock(dt_s=0.032)
    for i in range(1, 4):
        b.clock_tick(i * 0.032)
    assert b.sim_step == 3
    assert abs(b.sim_time - 0.096) < 1e-9


def test_state_reports_sim_seconds_and_keeps_the_wall_clock_separate() -> None:
    # PROTOCOL.md §5.3's own example reads `"last_tick_at": 12.448`. This
    # used to send 1.7 billion.
    b = _Stub()
    b.sim_clock = SimClock(dt_s=0.032)
    for i in range(1, 391):
        b.clock_tick(i * 0.032)
    state = b.get_state()
    assert abs(state["sim_time"] - 12.48) < 1e-6
    assert state["last_tick_at"] == state["sim_time"]
    assert state["last_tick_at"] < 1e6, "last_tick_at is SIM seconds, not epoch"
    assert state["wall_time"] > 1e9, "the wall clock keeps its own field"
    assert state["step"] == 390
    assert state["held"] is False
    assert state["events"] == {"total": 0, "last": None, "next_since": 0,
                               "dropped": 0}


# ── attach_telemetry / telemetry_tick, the shared installer ─────────

class _Plain:
    """A demo-bridge-shaped object: plain attributes, no properties."""
    robot_id = "husky"
    fault = None


def test_the_installer_wires_one_shape_for_every_bridge() -> None:
    b = _Plain()
    attach_telemetry(b, dt_s=0.032, robot_id="husky", surface="mobile",
                     joints=False)
    assert b.events is not None
    assert b.fault_detector is not None
    assert b.contact_detector is not None
    assert b.joint_detector is None           # asked for, honestly absent
    assert b.hold is not None
    assert b.surface == "mobile"
    assert b.sim_time == 0.0 and b.sim_step == 0


def test_telemetry_tick_mirrors_the_clock_onto_plain_attributes() -> None:
    b = _Plain()
    attach_telemetry(b, dt_s=0.032, robot_id="husky")
    for i in range(1, 5):
        telemetry_tick(b, i * 0.032)
    assert b.sim_step == 4
    assert abs(b.sim_time - 0.128) < 1e-9


def test_telemetry_tick_files_a_rising_edge_fault() -> None:
    b = _Plain()
    attach_telemetry(b, dt_s=0.032, robot_id="husky")
    telemetry_tick(b, 0.032)
    assert b.events.total == 0
    b.fault = "wheel_motor_unreachable"
    telemetry_tick(b, 0.064)
    assert b.events.last()["type"] == "fault.controller_lost"
    telemetry_tick(b, 0.096)
    assert b.events.total == 1                # edge, not level


def test_the_installer_does_not_fight_a_computed_property() -> None:
    # BridgeBase computes sim_time from its clock; a demo bridge stores a
    # float. The installer must handle both without raising -- a reader does
    # getattr(bridge, "sim_time", 0.0) and cannot tell the difference.
    b = _Stub()
    attach_telemetry(b, dt_s=0.016, robot_id="stub")
    telemetry_tick(b, 0.016)
    assert b.sim_step == 1
    assert abs(b.sim_time - 0.016) < 1e-9


def test_telemetry_tick_is_a_no_op_on_a_bridge_with_no_clock() -> None:
    class _Old:
        pass
    old = _Old()
    telemetry_tick(old, 1.0)                  # must not raise
    assert events_summary(old) == {"total": 0, "last": None,
                                   "next_since": 0, "dropped": 0}
