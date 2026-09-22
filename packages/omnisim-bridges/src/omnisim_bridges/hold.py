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

"""The lockstep hold: a bridge that freezes the world between commands.

OPT-IN, DEFAULT OFF: `OMNISIM_BRIDGE_LOCKSTEP=1`, value-parsed (`=0` is off).

WHY
---
An OmniLink-driven run is not reproducible today. On the bitwise CPU solver
(`newtonSolver "mujoco"`) a deterministic policy produced identical decisions
and still differed on 9 of 148 trace lines across 5 trials (AB_ARMS,
2026-09-19), because the world keeps running while the agent thinks and the
number of steps between one command and the next depends on how fast the
model answered. Hold the world between commands and that variable is gone.

WHAT IT IS NOT
--------------
Reproducibility of a RUN, not replay of one. There is still no record, no
replay and no run-diff, and `POST /sim/snapshot` is still not a checkpoint.

THE MECHANISM, AND WHY IT IS THIS ONE
-------------------------------------
`synchronization` defaults TRUE, so the engine waits on this controller's
`robot.step()`. NOT stepping is therefore already a hold on the coupling --
but the engine free-runs in `--mode=fast` independently of any one
controller, so the mode has to be set too, and a queued mode change is not a
delivered one. This is `PauseLease` from
`projects/default/controllers/harness_supervisor/harness_supervisor.py:948`,
adapted, and it is that one deliberately: it is the only hold mechanism in
the tree that has been tested across HTTP calls (43 engine-free tests + 4
live cases, 2026-09-22).

THE SEVEN DEFECTS THAT SHIPPED WITH THE FIRST ONE, AND WHAT STOPS EACH HERE
---------------------------------------------------------------------------
`0aafab096` (2026-09-15) shipped `PauseLease` with no test. Do not
re-introduce any of these:

1. **A frozen clock over a moving engine.** `simulationSetMode(PAUSE)` only
   QUEUES the request; it reaches the engine on the controller's next round
   trip, and a held pause makes no round trips. Measured before the fix: a
   break fired at t=168 ms and the engine ran on to ~400 ms. `flush()` below
   forces the request out with a real read.
2. **A lease expiry that hung the loop forever.** The un-pause was left for
   the loop's next `step()`, and a step against a paused engine blocks --
   engine and controller each waiting on the other. `release()` delivers the
   restore with a READ, never with a step.
3. **A lease that survived a world load.** A bridge process does not outlive
   its world, but it does outlive a `/reset_to_home` and an idle-loop
   restart, so `release()` is called on both and on every stop.
4. **The caller's mode overwritten by a re-take.** `take()` captures
   `prev_mode` ONCE; extending a live lease must not strand the original
   mode as PAUSE.
5. **Detection lagging behind the hold.** Not applicable to a bridge, which
   detects in its own tick.
6. **No test.** This module ships with `tests/test_hold.py` and every branch
   below is driven by a stub supervisor.
7. **An unbounded hold.** The lease self-expires (30 s default, 300 s cap)
   so a dead client cannot freeze a demo forever, and `stop_robot` always
   runs. ⚠️ Expiring is not enough: the loop must not take the hold straight
   back. It did, and a live run measured the world advancing exactly one
   basic step per 30 s lease. An expired lease now stays disarmed until the
   next command.

THE DEFECT THIS ADAPTATION ADDED, FOUND LIVE ON 2026-09-22
---------------------------------------------------------
**A held bridge outlived its engine, on its port.** A free-running bridge
dies with its engine inside `robot.step()`, when libController's pipe read
fails and it calls `exit(1)`. A held bridge makes no round trips, so it
kept serving HTTP for the rest of its lease after its engine was killed.
The next world's bridge bound the same port beside it (Windows
`SO_REUSEADDR`), and requests were then split between the two. One that
reached the orphan made it touch the dead pipe mid-request: the client saw
`ConnectionResetError` and no traceback existed anywhere, because
libController also sends the controller's stderr to an in-process pipe
that only a step drains. `HEARTBEAT_S` is the fix: one engine round trip
per half second while held, so a held bridge dies with its engine the way
a free-running one does. A robot that cannot make that round trip is
declined, not held.

THE WINDOW LEAK
---------------
`wwiReceiveText` -- the in-sim robot window's only channel -- needs a step.
A fully held loop makes the window mute. The loop therefore pumps at most
one step per second WHILE HELD and ONLY when the window is open, and reports
it as `hold.leak_steps` in `/state`. That leak is visible, bounded and
counted rather than hidden, and if it defeats the trace hashes the window is
declared unsupported under lockstep -- recorded, not worked around.
"""

from __future__ import annotations

import contextlib
import os
import time
from typing import Any, Dict, Optional

__all__ = ["HoldLease", "lockstep_enabled", "env_flag"]

# Webots' simulation mode enum. Duplicated as a literal rather than imported
# because this package has ZERO Webots imports by contract (bridge_base's
# docstring) -- it must import on a real robot with no simulator present.
SIMULATION_MODE_PAUSE = 0


def env_flag(name: str, default: bool = False) -> bool:
    """A VALUE-parsed boolean hatch. `=0` means off, and it has to: every
    presence-gated flag in this tree has eventually surprised somebody who
    set it to 0 expecting that to disable something."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off", "")


def lockstep_enabled() -> bool:
    return env_flag("OMNISIM_BRIDGE_LOCKSTEP", False)


def _lease_default_ms() -> int:
    # OMNISIM_BRIDGE_HOLD_LEASE_MS overrides how long a lockstep hold may keep
    # the world frozen before it lifts itself, in milliseconds. A hold is leased
    # rather than permanent for one reason: a client that dies mid-hold must not
    # be able to freeze a simulation forever, which is the same guarantee the
    # harness pause makes. Unparseable or empty falls back to
    # HoldLease.DEFAULT_MS rather than raising -- a bad value in an environment
    # variable must not stop a bridge from starting.
    raw = (os.environ.get("OMNISIM_BRIDGE_HOLD_LEASE_MS") or "").strip()
    if not raw:
        return HoldLease.DEFAULT_MS
    try:
        return int(raw)
    except ValueError:
        return HoldLease.DEFAULT_MS


class HoldLease:
    """A held pause on the world, owned by a bridge's main loop.

    Every method is safe to call when lockstep is disabled: `take` refuses,
    `held` stays False, and `step_or_hold` is a plain `robot.step()`. A
    bridge therefore wires it unconditionally and the hatch alone decides.
    """

    # Same bounds as the harness, deliberately: two hold mechanisms with
    # different deadlines is two things to reason about.
    DEFAULT_MS = 30_000
    MIN_MS = 1_000
    MAX_MS = 300_000

    # While held, pump at most one step this often so the robot window is
    # not mute. Counted, published, and zero when the window is closed.
    LEAK_PERIOD_S = 1.0

    # While held, make one engine round trip this often. See `step_or_hold`:
    # a held bridge makes no other round trips, so without this it cannot
    # find out that its engine has gone, and it outlives it on its port.
    HEARTBEAT_S = 0.5

    def __init__(self, events: Any = None, robot_id: str = "", *,
                 enabled: Optional[bool] = None) -> None:
        self.events = events
        self.robot_id = robot_id
        self._enabled = lockstep_enabled() if enabled is None else bool(enabled)
        self.deadline: Optional[float] = None     # time.monotonic() seconds
        self.prev_mode: Optional[int] = None
        self.taken_at_sim: Optional[float] = None
        self.leak_steps = 0
        self._last_leak = 0.0
        self.holds_taken = 0
        # Did the LAST `step_or_hold` actually advance the world? The loop
        # needs this, not just `held`: a leak step is a real step, and the
        # iteration that made it must run the tick and pump the window --
        # otherwise the leak buys a step of physics and none of the service
        # it exists to provide, which is the worst of both.
        self.last_advanced = False
        self._release_requested = ""
        self._active_step: Optional[int] = None
        # Engine round trips made while held, and when the last one was.
        self.heartbeats = 0
        self._last_heartbeat = 0.0
        # False after a lease EXPIRES, until the next command. Without it the
        # loop re-took the hold on the very next idle iteration: measured
        # 2026-09-22, the world advanced exactly ONE basic step per 30 s
        # lease, so a dead client froze the demo after all (defect 7).
        self._armed = True
        # Why `take` last declined, or "" -- published in `status`.
        self.declined = ""

    # ── state ───────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def held(self) -> bool:
        return self.deadline is not None

    def expired(self) -> bool:
        return self.deadline is not None and time.monotonic() >= self.deadline

    def remaining_ms(self) -> int:
        if self.deadline is None:
            return 0
        return max(0, int((self.deadline - time.monotonic()) * 1000.0))

    def status(self, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "lockstep": self._enabled,
            "held": self.held,
            "lease_remaining_ms": self.remaining_ms(),
            "held_since_sim_t": self.taken_at_sim,
            # ⚠️ PUBLISHED BECAUSE IT IS THE CAVEAT ON THE WHOLE FEATURE: any
            # non-zero value is world motion that happened while the run was
            # nominally frozen, to keep the robot window alive.
            "leak_steps": self.leak_steps,
            "holds_taken": self.holds_taken,
            "heartbeats": self.heartbeats,
            # False from a lease expiry until the next command: the world is
            # free-running on purpose, not because the hold is broken.
            "armed": self._armed,
        }
        if self.declined:
            out["declined"] = self.declined
        if extra:
            out.update(extra)
        return out

    # ── the engine handshake ────────────────────────────────────────

    @staticmethod
    def flush(supervisor: Any) -> bool:
        """Force a queued mode change out to the engine, NOW.

        ⚠️ `simulationSetMode` only QUEUES; the request rides the
        controller's next round trip, and a held loop makes none. Without
        this the bridge would report a frozen clock over a moving scene,
        which is worse than having no hold at all. `getPosition()` on our own
        node is the cheapest real round trip; the value is discarded.
        """
        try:
            node = supervisor.getSelf()
            if node is None:
                return False
            node.getPosition()
            return True
        except Exception:                                # noqa: BLE001
            return False

    def take(self, supervisor: Any, lease_ms: Optional[int] = None, *,
             sim_time: float = 0.0, step: int = 0) -> Dict[str, Any]:
        """Enter (or extend) the hold. A no-op unless lockstep is enabled."""
        if not self._enabled:
            return self.status({"refused": "lockstep is off "
                                           "(OMNISIM_BRIDGE_LOCKSTEP=1 to enable)"})
        ms = _lease_default_ms() if lease_ms is None else int(lease_ms)
        ms = max(self.MIN_MS, min(ms, self.MAX_MS))
        get_mode = getattr(supervisor, "simulationGetMode", None)
        set_mode = getattr(supervisor, "simulationSetMode", None)
        if (get_mode is None or set_mode is None
                or getattr(supervisor, "getSelf", None) is None):
            # ⚠️ DECLINE, DO NOT DEGRADE. A hold must be able to reach the
            # engine while it holds: that is how it learns the engine has
            # gone (see `step_or_hold`). A robot without supervisor powers
            # has no way to make that round trip, so a hold here would
            # outlive its engine and keep the port. Plan L5 already requires
            # `supervisor TRUE`; this makes a world that forgot say so.
            self.declined = ("lockstep needs supervisor powers "
                             "(supervisor TRUE) to reach the engine while "
                             "it holds; this robot has none, so the world "
                             "runs free")
            return self.status({"refused": self.declined})
        if not self.held:
            # ONCE. Re-taking a live lease must not overwrite the caller's
            # mode with PAUSE and strand it there on release.
            if get_mode is not None:
                try:
                    self.prev_mode = get_mode()
                except Exception:                        # noqa: BLE001
                    self.prev_mode = None
            self.taken_at_sim = float(sim_time)
            self.holds_taken += 1
            if set_mode is not None and self.prev_mode != SIMULATION_MODE_PAUSE:
                try:
                    set_mode(SIMULATION_MODE_PAUSE)
                    self.flush(supervisor)
                except Exception:                        # noqa: BLE001
                    pass
            # The flush above was a round trip; the next is due a period on.
            self._last_heartbeat = time.monotonic()
        self.deadline = time.monotonic() + (ms / 1000.0)
        return self.status({"lease_ms": ms})

    def release(self, supervisor: Any, reason: str = "resume", *,
                sim_time: float = 0.0, step: int = 0) -> Dict[str, Any]:
        """Leave the hold and restore the pre-hold mode. Idempotent.

        ⚠️ THE RESTORE IS DELIVERED WITH A READ, NOT WITH THE LOOP'S NEXT
        STEP. A step against a paused engine blocks, so leaving the un-pause
        for `step_or_hold` has the engine and the controller each waiting on
        the other -- measured on the harness, where a 1 s lease expired and
        the supervisor never answered again. That turns the deadline, which
        exists so a crashed client cannot freeze the engine, into the thing
        that freezes it.
        """
        was = self.held
        if was:
            set_mode = getattr(supervisor, "simulationSetMode", None)
            if set_mode is not None and self.prev_mode is not None:
                try:
                    set_mode(self.prev_mode)
                    self.flush(supervisor)
                except Exception:                        # noqa: BLE001
                    pass
        self.deadline = None
        self.prev_mode = None
        self.taken_at_sim = None
        return self.status({"was_held": was, "released_by": reason})

    @contextlib.contextmanager
    def lifted(self, supervisor: Any):
        """Run a body that NEEDS the world to advance, with the hold lifted.

        The lease stays held: lifting is a property of this call, not a
        release, so the caller is still in lockstep when it returns and its
        deadline keeps running. `finally` re-takes the pause even if the body
        raised, so a failed motion cannot strand the world running.
        """
        set_mode = getattr(supervisor, "simulationSetMode", None)
        lift = self.held and set_mode is not None and self.prev_mode is not None
        if lift:
            try:
                set_mode(self.prev_mode)
            except Exception:                            # noqa: BLE001
                lift = False
        try:
            yield lift
        finally:
            if lift:
                try:
                    set_mode(SIMULATION_MODE_PAUSE)
                    self.flush(supervisor)
                except Exception:                        # noqa: BLE001
                    pass

    # ── the state machine the loop drives ───────────────────────────

    def request_release(self, reason: str = "command") -> None:
        """Ask the LOOP to lift the hold. Safe from any thread.

        ⚠️ AN HTTP THREAD MUST NEVER CALL `release()` ITSELF. Releasing
        touches `simulationSetMode` and flushes it with a supervisor read,
        and the controller API is not thread-safe -- that call from an HTTP
        worker is precisely what dragged the warehouse demo to ~0.2x
        realtime, and under a hold it would be worse: the sim thread is
        parked, so the pipe has nobody to service it. So a request is a
        FLAG, and the loop does the work on the right thread.

        `stop_robot` uses it, because a stop must land in the world rather
        than wait for the operator's next command (plan D6 step 1: stop
        always runs).
        """
        self._release_requested = reason or "command"

    def sync(self, supervisor: Any, *, busy: bool, sim_time: float = 0.0,
             step: int = 0, settle_steps: int = 4) -> Optional[str]:
        """SIM THREAD ONLY. Hold while idle, run while there is work.

        `settle_steps` is why a command does not have to fight the hold to
        finish: after the motion slot empties, the world keeps stepping for
        a few more steps so the final zero-velocity write, the settle window
        and the pose samples the result is measured from all actually land.
        Re-freezing on the same step the motion ended would measure a
        stopped world and call it a settled robot.

        Returns "taken", "released" or None.
        """
        if not self._enabled:
            return None
        reason = getattr(self, "_release_requested", "")
        if busy or reason:
            self._release_requested = ""
            self._active_step = int(step)
            # A command is what re-arms a hold that ran out its lease.
            self._armed = True
            if self.held:
                self.release(supervisor, reason=(reason or "motion_in_flight"),
                             sim_time=sim_time, step=step)
                return "released"
            return None
        active = getattr(self, "_active_step", None)
        if active is None:
            self._active_step = int(step)
            return None
        if self.held or (int(step) - int(active)) < int(settle_steps):
            return None
        if not self._armed:
            # The lease EXPIRED and nobody has asked for anything since. The
            # expiry exists so a dead client cannot freeze the demo; taking
            # the hold straight back would undo it one step later.
            return None
        self.take(supervisor, sim_time=sim_time, step=step)
        return "taken" if self.held else None

    # ── the main loop's step ────────────────────────────────────────

    def step_or_hold(self, robot: Any, timestep: int, *,
                     sim_time: float = 0.0, step: int = 0,
                     window_open: bool = False) -> int:
        """The bridge loop's `robot.step()`, with a hold honoured.

        Returns what `robot.step()` returned (-1 terminates), or 0 while the
        hold is on -- "not terminated, and deliberately did not advance".

        ⚠️ Never replace this with a bare `robot.step()` while a lease can be
        live: see `release`.
        """
        if self.held:
            if self.expired():
                self.release(robot, reason="lease_expired",
                             sim_time=sim_time, step=step)
                self._armed = False       # free-run until the next command
                self._emit_expired(sim_time, step)
            elif self._heartbeat_due() and not self._heartbeat(robot):
                # Could not reach the engine at all. Holding blind is how a
                # bridge outlives its world, so stop holding and take a real
                # step: against a live engine that is harmless, against a
                # dead one libController ends the process there.
                self.release(robot, reason="heartbeat_failed",
                             sim_time=sim_time, step=step)
                self._armed = False
            elif window_open and self._leak_due():
                # The window's only channel needs a step. Bounded, counted
                # and published; see the module docstring.
                self.leak_steps += 1
                self._last_leak = time.monotonic()
                self.last_advanced = True
                return robot.step(timestep)
            else:
                # Do not step. Sleep briefly so a held loop does not spin a
                # core; the HTTP threads keep being serviced, which is what
                # keeps `release` deliverable.
                time.sleep(0.005)
                self.last_advanced = False
                return 0
        self.last_advanced = True
        return robot.step(timestep)

    def _leak_due(self) -> bool:
        return (time.monotonic() - self._last_leak) >= self.LEAK_PERIOD_S

    def _heartbeat_due(self) -> bool:
        return (time.monotonic() - self._last_heartbeat) >= self.HEARTBEAT_S

    def _heartbeat(self, robot: Any) -> bool:
        """One engine round trip while held. SIM THREAD ONLY.

        ⚠️ THIS IS HOW A HELD BRIDGE DIES WITH ITS ENGINE. A free-running
        bridge learns its engine is gone inside `robot.step()`: the pipe
        read fails and libController ends the process (`g_pipe.c`,
        `broken_pipe()` -> `exit(1)`). A held bridge makes no round trips,
        so before this existed it outlived its engine for the rest of the
        lease -- still serving HTTP on its port. MEASURED 2026-09-22: the
        determinism probe's teardown killed a trial's engine while the
        bridge was held; the next trial's bridge bound the same port
        alongside it (Windows `SO_REUSEADDR`), the next trial's request
        landed on the orphan, the orphan's first robot read hit the dead
        pipe, and the client saw `ConnectionResetError` with no traceback
        anywhere. Every "lockstep resets the connection" report traced to
        that.

        The round trip is `flush()`'s: a paused engine services reads (the
        premise of the harness's `paused_reads`), so on a live engine this
        costs one small read every HEARTBEAT_S and advances nothing.
        Returns False only when no round trip could be made at all.
        """
        self._last_heartbeat = time.monotonic()
        ok = self.flush(robot)
        if ok:
            self.heartbeats += 1
        return ok

    def _emit_expired(self, sim_time: float, step: int) -> None:
        ring = self.events
        if ring is None:
            return
        try:
            ring.emit("hold.expired", sim_time=float(sim_time), step=int(step),
                      robot=self.robot_id,
                      note=("the lockstep hold ran out its lease and the "
                            "world resumed on its own"),
                      leak_steps=self.leak_steps)
        except Exception:                                # noqa: BLE001
            pass
