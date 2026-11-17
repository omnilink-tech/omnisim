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

"""The bridge's own event stream: a ring, the detectors that fill it, and
the policy that says which events may cost an operator a model turn.

WHY A BRIDGE NEEDS ITS OWN EVENT STREAM
---------------------------------------
Until v9 the sim→agent channel was PULL ONLY. A joint pinned against its
limit, a motion that quietly timed out, a refusal on an ungated path -- none
of them reached the agent unless a tool result happened to carry it, and the
agent had no way to ask "what happened while I was thinking?". The World
Harness has had `GET /sim/events` since v8; a bridge is the other half of
the pair and had nothing, so a robot could not speak up about its own body.

This module is that half. It deliberately mirrors the harness's envelope
(`events[]`, `next_since`, `dropped`) so one client can drain both with one
loop, and so the harness's own events can be re-emitted here under
`source: "harness"` without reshaping them.

THREE RULES THAT SHAPE EVERY LINE BELOW
---------------------------------------
1. **Nothing here may touch the Robot API.** The controller API is not
   thread-safe and a supervisor read from an HTTP thread drags the sim to
   ~0.2x realtime (see `MainThreadCalls` in the mobile bridge). Every
   detector takes *values* -- floats, names, dicts -- and never a device.
   That is also what makes them testable with no engine.
2. **A detector that has never gone red may be vacuous.** Each one here was
   written against a test that fails when the detector is removed, because
   this tree has already shipped a structurally-impossible contact-pairing
   check that read green for weeks.
3. **`error` in a motion result is the CONTROL error, a float.** A perfect
   stop carries `4.3e-11`. `emit_motion_outcome` below tests
   `isinstance(err, str)` and nothing else; the truthiness bug has shipped
   three times, most recently making every successful `/tool` motion report
   `status: "err"`.

ONE EVENT
---------
    {"seq": 12, "type": "joint.limit_hit", "sim_time": 4.096, "step": 128,
     "robot": "husky", "source": "bridge", "detail": {...}}

`seq` is monotonic from 1 and never resets while the process lives, so a
cursor is meaningful across drains. `sim_time` is SIM seconds (never the
wall clock) and `step` is the bridge's own tick counter -- the two clocks of
D1. `source` is `"bridge"` for something this bridge measured and
`"harness"` for something re-emitted from `GET /sim/events`.
"""

from __future__ import annotations

import collections
import os
import threading
from typing import Any, Dict, List, Optional, Sequence

__all__ = [
    "EventRing",
    "BRIDGE_EVENT_TYPES",
    "HARNESS_EVENT_TYPES",
    "EVENT_TYPE_DETECTORS",
    "WAKE_POLICY",
    "may_wake",
    "DEFAULT_CAPACITY",
    "JointLimitDetector",
    "ContactDetector",
    "contact_bodies",
    "FaultDetector",
    "HarnessEventFeed",
    "harness_feed",
    "emit_motion_outcome",
    "emit_gate_refusal",
    "FLOOR_NAMES",
]


# 512 events at the ~1-5 events/second a bridge actually produces is minutes
# of headroom, and the ring is per-bridge-process rather than per-world, so
# the memory cost is a few hundred kB. `dropped` reports eviction rather than
# hiding it. Value-parsed hatch, as every hatch in this tree must be.
def _capacity_from_env(default: int = 512) -> int:
    raw = (os.environ.get("OMNISIM_BRIDGE_EVENT_CAPACITY") or "").strip()
    if not raw:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return max(16, min(value, 100_000))


DEFAULT_CAPACITY = 512

# The envelope's own field names. Anything re-emitted from another producer
# must have these stripped before it is passed as **detail: they are
# keyword-only parameters of `emit()` and a collision is a TypeError, raised
# on whatever thread was forwarding.
_RESERVED_EVENT_KEYS = ("type", "seq", "sim_time", "step", "robot", "source",
                        "detail")


# ── The taxonomy ────────────────────────────────────────────────────
#
# ⚠️ Emitted through the literal `emit("<type>", ...)` call form, because
# `event_bus.scan_emit_calls()` reads the emitting module's SOURCE to check
# the published list against the call sites (v9 standing rule). A type name
# built at runtime is invisible to that scan and must not be introduced.

BRIDGE_EVENT_TYPES = (
    # D1's step-budget verdicts, emitted where a motion is recorded.
    "motion.timed_out",
    "motion.unsettled",
    # Joint readback against the declared limits.
    "joint.limit_hit",
    # The bridge's own health.
    "fault.controller_lost",
    "fault.telemetry_stale",
    # A refusal by the one safety gate, on /prompt or /tool.
    "gate.refused",
    # Supervisor contact points. ⚠️ TOP-LEVEL ONLY -- see ContactDetector.
    "contact.began",
    # D6: a lockstep hold that ran out its lease and released itself.
    "hold.expired",
)

# Re-emitted verbatim (with `source: "harness"`) when OMNISIM_HARNESS_URL is
# set. Listed so a client can tell a type this bridge MEASURES from one it
# merely FORWARDS: the second kind stops arriving the moment the harness
# goes away, and that is not a statement about the robot.
HARNESS_EVENT_TYPES = (
    "break.hit",
    "damage.impact",
    "damage.state_transition",
    "joint.limit_hit",
    "contact.began",
    "contact.ended",
)

# Which detector owns each bridge-side type. Published so a client reading an
# empty stream can tell "nothing happened" from "nothing is watching".
EVENT_TYPE_DETECTORS = {
    "motion.timed_out": "emit_motion_outcome",
    "motion.unsettled": "emit_motion_outcome",
    "joint.limit_hit": "JointLimitDetector",
    "fault.controller_lost": "FaultDetector",
    "fault.telemetry_stale": "FaultDetector",
    "gate.refused": "emit_gate_refusal",
    "contact.began": "ContactDetector",
    "hold.expired": "HoldLease",
}


# ── Which events may wake the agent (plan decision L4) ───────────────
#
# A wake is a MODEL TURN and costs about 5.6k prompt tokens. An unbounded
# wake on a contact stream would spend the operator's budget on the floor,
# so the table is deliberately narrow: only PHYSICAL or SAFETY events, and
# only the ones an agent could not have learned from its own tool result.
#
# The table is by TYPE. Two types need a per-event refinement as well, and
# `may_wake()` below applies it:
#   - `contact.began` only wakes for a NON-FLOOR body (the detector already
#     refuses to emit for the floor, so this is belt and braces);
#   - `gate.refused` only wakes when the refusal did NOT come from the
#     agent's own turn. A refusal on `/prompt` is already in the reply the
#     model is reading; waking it about that would be a loop with a bill.
WAKE_POLICY: Dict[str, bool] = {
    # bridge-side
    "motion.timed_out": True,
    "motion.unsettled": False,     # a sloppy stop is not an emergency
    "joint.limit_hit": True,
    "fault.controller_lost": True,
    "fault.telemetry_stale": True,
    "gate.refused": True,          # refined by may_wake(): external only
    "contact.began": True,         # refined by may_wake(): non-floor only
    "hold.expired": False,         # bookkeeping, not physics
    # harness pass-through
    "break.hit": True,
    "damage.impact": True,
    "damage.state_transition": True,
    "contact.ended": False,
    "grip.acquired": False,
    "grip.released": False,
    "controller.log": False,
    "world.warning": False,
    "world.error": False,
}

# Bodies whose contact is the world working normally. Matched
# case-insensitively as a substring of the contacting node's name.
FLOOR_NAMES = ("floor", "ground", "terrain", "arena", "plane", "solid_floor")


def _is_floor(name: str) -> bool:
    low = (name or "").strip().lower()
    if not low:
        return True          # unnamed: not a body an agent can act on
    return any(k in low for k in FLOOR_NAMES)


def may_wake(event: Dict[str, Any]) -> bool:
    """Whether this event is allowed to cost a model turn.

    Type-level policy from WAKE_POLICY, plus the two per-event refinements
    L4 calls for. The RATE limit is not here: it belongs to the dispatcher,
    which owns the token bucket and the session budget.
    """
    if not isinstance(event, dict):
        return False
    etype = str(event.get("type") or "")
    if not WAKE_POLICY.get(etype, False):
        return False
    detail = event.get("detail") or {}
    if etype == "contact.began":
        return not _is_floor(str(detail.get("body") or detail.get("with") or ""))
    if etype == "gate.refused":
        # `origin` is the surface the refused call arrived on. A refusal the
        # agent caused in its own turn is already in front of it.
        return str(detail.get("origin") or "") not in ("relay", "prompt", "wake")
    return True


class EventRing:
    """A bounded, monotonic-seq, cursor-paged ring of bridge events.

    Thread-safe by a plain lock: detectors emit from the sim thread, the
    gate emits from an HTTP thread, and `/events` drains from a third. The
    lock is held only for list surgery -- never across a Robot API call,
    because there are none in this file.
    """

    def __init__(self, capacity: Optional[int] = None, *,
                 robot: str = "", source: str = "bridge") -> None:
        cap = _capacity_from_env(DEFAULT_CAPACITY) if capacity is None else int(capacity)
        self._events: collections.deque = collections.deque(maxlen=max(1, cap))
        self._lock = threading.Lock()
        self._counter = 0
        self._dropped = 0
        self._robot = robot
        self._source = source

    # ── producer ────────────────────────────────────────────────────

    def emit(self, type: str, *, sim_time: float, step: int,
             robot: str = "", source: str = "bridge", **detail: Any) -> None:
        """File one event. Never raises: a detector must not be able to take
        the bridge down, and an event nobody can file is still better than a
        traceback on the sim thread."""
        try:
            evt = {
                "seq": 0,
                "type": str(type),
                "sim_time": float(sim_time),
                "step": int(step),
                "robot": str(robot or self._robot),
                "source": str(source or self._source),
                "detail": dict(detail),
            }
        except (TypeError, ValueError):
            return
        with self._lock:
            if len(self._events) == self._events.maxlen:
                self._dropped += 1
            self._counter += 1
            evt["seq"] = self._counter
            self._events.append(evt)

    # ── consumer ────────────────────────────────────────────────────

    def since(self, cursor: int = 0, *, limit: int = 100,
              types: Optional[Sequence[str]] = None) -> Dict[str, Any]:
        """Events with `seq > cursor`, oldest first.

        `next_since` is the cursor to send next time. It advances to the
        last event RETURNED, so a `limit`-capped drain resumes where it
        stopped -- and when a `types` filter drops everything, it advances
        to the newest event in the ring instead, or a filtered poller would
        re-scan the whole buffer forever.

        `dropped` is the cumulative eviction count, the same meaning the
        harness's `dropped_sup` carries: non-zero means poll faster or raise
        `limit`. `missed` is the sharper number -- how many events THIS
        cursor will never see because they were evicted before it read
        them.
        """
        try:
            cursor = max(0, int(cursor))
        except (TypeError, ValueError):
            cursor = 0
        try:
            limit = max(1, min(int(limit), 1000))
        except (TypeError, ValueError):
            limit = 100
        wanted = set(types) if types else None
        out: List[Dict[str, Any]] = []
        with self._lock:
            snapshot = list(self._events)
            total = self._counter
            dropped = self._dropped
        oldest = snapshot[0]["seq"] if snapshot else total + 1
        newest = snapshot[-1]["seq"] if snapshot else total
        for evt in snapshot:
            if evt["seq"] <= cursor:
                continue
            if wanted is not None and evt["type"] not in wanted:
                continue
            out.append(dict(evt))
            if len(out) >= limit:
                break
        next_since = out[-1]["seq"] if out else max(cursor, newest)
        return {
            "events": out,
            "next_since": int(next_since),
            "dropped": int(dropped),
            "total": int(total),
            "missed": int(max(0, oldest - 1 - cursor)) if snapshot else 0,
            "buffered": len(snapshot),
            "capacity": int(self._events.maxlen or 0),
        }

    @property
    def total(self) -> int:
        """Every event ever filed, including the evicted ones."""
        with self._lock:
            return self._counter

    def last(self) -> Optional[Dict[str, Any]]:
        """The newest event still in the ring, or None."""
        with self._lock:
            if not self._events:
                return None
            return dict(self._events[-1])

    # ── bookkeeping ─────────────────────────────────────────────────

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    def summary(self) -> Dict[str, Any]:
        """`{total, last, next_since}` -- the `/state.events` block."""
        with self._lock:
            last = dict(self._events[-1]) if self._events else None
            return {
                "total": self._counter,
                "last": last,
                "next_since": self._counter,
                "dropped": self._dropped,
            }


# ── Detector: joint limits ──────────────────────────────────────────

class JointLimitDetector:
    """`joint.limit_hit` from joint readback against DECLARED limits.

    Fed positions and limits, never devices. Hysteresis is not optional: a
    joint parked on its stop jitters across the band edge at solver noise
    and would emit an event per tick. It arms when the position enters the
    margin and does not re-arm until the joint has come back a release
    fraction of that margin -- the same shape as the supervisor's
    `JointLimitTracker`.
    """

    def __init__(self, ring: EventRing, *, margin: float = 0.02,
                 release_frac: float = 2.0) -> None:
        self.ring = ring
        self.margin = float(margin)
        self.release_frac = float(release_frac)
        self._armed: Dict[str, str] = {}      # joint -> "min" | "max"

    def update(self, names: Sequence[str], q: Sequence[float],
               limits: Sequence[Any], *, sim_time: float, step: int,
               robot: str = "") -> List[str]:
        """Returns the joint names that fired this call (for tests, and so a
        caller can act without draining the ring)."""
        fired: List[str] = []
        for idx, name in enumerate(names or ()):
            if idx >= len(q) or idx >= len(limits):
                break
            pos = q[idx]
            pair = limits[idx]
            if not isinstance(pos, (int, float)) or isinstance(pos, bool):
                continue
            if not (isinstance(pair, (list, tuple)) and len(pair) >= 2):
                continue
            lo, hi = pair[0], pair[1]
            if not isinstance(lo, (int, float)) or not isinstance(hi, (int, float)):
                continue
            if hi <= lo:
                continue
            side: Optional[str] = None
            if pos <= lo + self.margin:
                side = "min"
            elif pos >= hi - self.margin:
                side = "max"
            was = self._armed.get(name)
            if side is None:
                # Release only once the joint is clear of the band by the
                # release factor, so noise on the edge cannot re-trigger.
                if was is not None:
                    clear = (pos > lo + self.margin * self.release_frac
                             and pos < hi - self.margin * self.release_frac)
                    if clear:
                        self._armed.pop(name, None)
                continue
            if was == side:
                continue                      # already reported this stop
            self._armed[name] = side
            self.ring.emit(
                "joint.limit_hit", sim_time=sim_time, step=step, robot=robot,
                joint=name, position=float(pos), limit=float(lo if side == "min" else hi),
                side=side, margin=self.margin,
                overshoot=float(pos - lo if side == "min" else pos - hi),
            )
            fired.append(name)
        return fired


# ── Detector: contacts ──────────────────────────────────────────────

class ContactDetector:
    """`contact.began` for a NAMED, NON-FLOOR body.

    ⚠️ KNOWN BLIND SPOT, DECLARED IN THE EVENT ITSELF. The supervisor's
    `getContactPoints()` reports contacts on TOP-LEVEL Solids only: a URDF
    robot's sub-links are invisible to it (recorded 2026-09,
    `reference_supervisor_contacts_blind_to_urdf_links`). An agent reading
    an empty contact stream must be able to tell "nothing touched" from
    "this instrument cannot see it", so every event carries
    `scope: "top-level"` and the absence of an event is never evidence.

    The floor is filtered because a mobile robot is in contact with it for
    the whole run; an event per tick about the ground is how a wake budget
    gets spent on nothing.
    """

    SCOPE = "top-level"

    def __init__(self, ring: EventRing, *,
                 floor_names: Sequence[str] = FLOOR_NAMES) -> None:
        self.ring = ring
        self.floor_names = tuple(n.lower() for n in floor_names)
        self._touching: set = set()

    def _floor(self, name: str) -> bool:
        low = (name or "").strip().lower()
        if not low:
            return True
        return any(k in low for k in self.floor_names)

    def update(self, bodies: Sequence[str], *, sim_time: float, step: int,
               robot: str = "") -> List[str]:
        """`bodies` is the set of top-level Solid names currently in contact
        with this robot, as the supervisor reports them."""
        now = {str(b) for b in (bodies or ()) if str(b).strip()}
        began = now - self._touching
        self._touching = now
        fired: List[str] = []
        for body in sorted(began):
            if self._floor(body):
                continue
            self.ring.emit(
                "contact.began", sim_time=sim_time, step=step, robot=robot,
                body=body, scope=self.SCOPE,
                blind_spot=("getContactPoints() sees top-level Solids only; "
                            "URDF sub-link contacts are invisible to it"),
            )
            fired.append(body)
        return fired


def contact_bodies(self_points: Any, candidates: Any,
                   *, tol: float = 1e-4) -> List[str]:
    """Name the bodies a robot is touching, by joining on the shared POINT.

    ⚠️ NEVER JOIN ON `ContactPoint.node_id`. The engine streams the QUERIED
    solid's own id there (`OmSupervisorUtilities::pushContactPointsToStream`),
    so keying on it makes every pair (X, X) and every event claims a body is
    touching itself -- measured in the harness, where FLOOR reported ids
    [9,9,9,9] and CRATE_BOT [14,14,14,14] for the same four contacts. The
    shared contact POINT is the only field that identifies one physical
    contact from both sides.

    `self_points` is this robot's contact points; `candidates` maps a body
    name to that body's contact points. Both come from
    `Node.getContactPoints()`, gathered on the SIM THREAD by the caller --
    this function does no engine work, which is what makes it testable.

    A candidate list is bounded ON PURPOSE: the harness can afford to walk
    every Solid in the world, a controller's tick cannot. The cost of that
    choice is that an unwatched body produces no event, which is why
    `ContactDetector`'s events carry their scope.
    """
    def _key(p: Any):
        try:
            return (round(float(p[0]) / tol), round(float(p[1]) / tol),
                    round(float(p[2]) / tol))
        except (TypeError, ValueError, IndexError):
            return None

    mine = {k for k in (_key(p) for p in (self_points or ())) if k is not None}
    if not mine:
        return []
    out: List[str] = []
    for name, points in (candidates or {}).items():
        theirs = {k for k in (_key(p) for p in (points or ())) if k is not None}
        if mine & theirs:
            out.append(str(name))
    return sorted(out)


# ── Detector: the bridge's own health ───────────────────────────────

class FaultDetector:
    """`fault.controller_lost` and `fault.telemetry_stale`.

    TWO CALL SITES, AND THAT IS THE POINT. `on_tick` runs on the sim thread
    and sees the bridge's `fault` field go non-null. `poll_stale` runs on
    whatever thread is SERVING a request, because the staleness it detects
    is *the sim thread having stopped* -- a detector that only ran in
    `tick()` could never fire for the one condition that means `tick()` is
    not running. That is the vacuity trap in its purest form.

    `poll_stale` reads cached floats only. It issues no Robot API call, so
    it is safe from an HTTP thread (`MainThreadCalls`).
    """

    def __init__(self, ring: EventRing, *, stale_after_s: float = 2.0) -> None:
        self.ring = ring
        self.stale_after_s = float(stale_after_s)
        self._fault: Optional[str] = None
        self._stale = False

    @staticmethod
    def _code(fault: Any) -> Optional[str]:
        if fault is None:
            return None
        if isinstance(fault, dict):
            return str(fault.get("code") or fault.get("message") or "fault")
        text = str(fault).strip()
        return text or None

    def on_tick(self, fault: Any, *, sim_time: float, step: int,
                robot: str = "") -> Optional[str]:
        """Rising edge on the bridge's declared fault (PROTOCOL §5.3)."""
        code = self._code(fault)
        if code == self._fault:
            return None
        self._fault = code
        if code is None:
            return None                       # cleared: re-armed, no event
        self.ring.emit(
            "fault.controller_lost", sim_time=sim_time, step=step, robot=robot,
            code=code, source_field="fault",
        )
        return code

    def clear_stale(self) -> None:
        """Called from the tick: the world IS stepping, so re-arm."""
        self._stale = False

    def poll_stale(self, *, wall_now: float, last_tick_wall: float,
                   sim_time: float, step: int, robot: str = "") -> bool:
        """Edge-triggered: fires once per stall, not once per poll."""
        try:
            age = float(wall_now) - float(last_tick_wall)
        except (TypeError, ValueError):
            return False
        if age <= self.stale_after_s:
            self._stale = False
            return False
        if self._stale:
            return False
        self._stale = True
        self.ring.emit(
            "fault.telemetry_stale", sim_time=sim_time, step=step, robot=robot,
            age_s=round(age, 3), budget_s=self.stale_after_s,
            note=("the simulation tick has stopped, so every reading this "
                  "bridge serves is at least this old"),
        )
        return True


# ── Emitters that live where the fact is known ──────────────────────

def emit_motion_outcome(ring: Optional[EventRing], completion: Dict[str, Any],
                        *, sim_time: float, step: int,
                        robot: str = "") -> Optional[str]:
    """`motion.timed_out` / `motion.unsettled` from a completion record.

    Called from `_record_completion` -- i.e. on the SIM THREAD, at the one
    place a motion result is written.

    ⚠️ `completion["error"]` is the CONTROL error, a float: achieved minus
    commanded. It is passed through as a measurement and is NEVER read as a
    failure. The verdict comes from `timed_out` and `settled`, which D1's
    step budget decides -- not from a wall clock and not from `error`.
    """
    if ring is None or not isinstance(completion, dict):
        return None
    if completion.get("superseded"):
        return None                       # nobody measured it; say nothing
    timed_out = bool(completion.get("timed_out"))
    settled = completion.get("settled")
    if not timed_out and settled is not False:
        return None
    etype = "motion.timed_out" if timed_out else "motion.unsettled"
    detail = {
        "verb": completion.get("verb", "?"),
        "seq": completion.get("seq"),
        "commanded": completion.get("commanded"),
        "achieved": completion.get("achieved"),
        "control_error": completion.get("error"),
        "unit": completion.get("unit", ""),
        "settled": bool(settled),
        "steps": completion.get("steps"),
        "sim_time_start": completion.get("sim_time_start"),
        "sim_time_end": completion.get("sim_time_end"),
    }
    if timed_out:
        ring.emit("motion.timed_out", sim_time=sim_time, step=step,
                  robot=robot, **detail)
    else:
        ring.emit("motion.unsettled", sim_time=sim_time, step=step,
                  robot=robot, **detail)
    return etype


# ── The harness pass-through (plan D4 step 3) ───────────────────────

class HarnessEventFeed:
    """Re-emit the World Harness's `/sim/events` onto this bridge's ring.

    WHY A THREAD AND NOT THE TICK. The plan says "the tick polls
    `/sim/events`". It must not: that is a BLOCKING HTTP CALL, and the tick
    is the thread the whole simulation waits on -- a harness that is slow,
    paused or gone would stall the world for the socket timeout, every
    poll. Emitting, unlike reading the Robot API, is thread-safe (the ring
    has a lock and touches no device), so the poll runs on its own daemon
    thread and nothing about the sim thread's rules is bent. The cost of
    the deviation is that a harness event's `step` is the step the bridge
    had reached when it ARRIVED, not when it happened -- so the harness's
    own `t_sim_ms` is preserved in the detail rather than thrown away.

    `OMNISIM_HARNESS_URL` already exists and was undocumented; the MCP
    server and the ROS 2 client both read it.
    """

    #: Only the types worth forwarding. The harness's log types
    #: (`controller.log`, `world.warning`) are this bridge's own stdout
    #: coming back around and would loop.
    FORWARD = HARNESS_EVENT_TYPES

    def __init__(self, ring: EventRing, url: str, *, robot: str = "",
                 period_s: float = 0.5, clock: Any = None) -> None:
        self.ring = ring
        self.url = url.rstrip("/")
        self.robot = robot
        self.period_s = float(period_s)
        self.clock = clock
        self.cursor = 0
        self.errors = 0
        self.forwarded = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # -- the pure half, driven by a fake `fetch` in the tests ---------

    def poll_once(self, fetch: Any) -> int:
        """`fetch(cursor) -> dict` in the harness's envelope. Returns the
        number of events forwarded. NEVER RAISES: a harness that went away
        must not be able to stop a robot from reporting its own events."""
        try:
            batch = fetch(self.cursor)
        except Exception:
            self.errors += 1
            return 0
        if not isinstance(batch, dict):
            self.errors += 1
            return 0
        events = batch.get("events") or []
        sim_time = float(getattr(self.clock, "sim_time", 0.0) or 0.0)
        step = int(getattr(self.clock, "step", 0) or 0)
        n = 0
        for evt in events:
            if not isinstance(evt, dict):
                continue
            etype = str(evt.get("type") or "")
            if etype not in self.FORWARD:
                continue
            # ⚠️ STRIP THE ENVELOPE'S OWN NAMES FIRST. `emit()` takes
            # `robot`, `source`, `sim_time` and `step` as keyword-only
            # parameters and the harness's payload carries a `robot` of its
            # own, so passing the payload through as **detail raised
            # `got multiple values for keyword argument 'robot'` -- on the
            # poll thread, where the only symptom would have been a silent
            # `errors` counter. The harness's value is KEPT, under a name
            # that says whose it is.
            detail = {k: v for k, v in evt.items()
                      if k not in _RESERVED_EVENT_KEYS
                      and k not in ("t_sim_ms",)}
            if evt.get("robot") is not None:
                detail["harness_robot"] = evt.get("robot")
            # The harness's own clock, preserved: it is the instant the
            # event HAPPENED, where our `sim_time` is the instant it
            # arrived. Two rulers, both reported, neither pretending to be
            # the other.
            if evt.get("t_sim_ms") is not None:
                detail["harness_t_sim_ms"] = evt.get("t_sim_ms")
            if evt.get("seq") is not None:
                detail["harness_seq"] = evt.get("seq")
            self.ring.emit(etype, sim_time=sim_time, step=step,
                           robot=self.robot, source="harness", **detail)
            n += 1
        nxt = batch.get("next_since")
        if isinstance(nxt, int) and nxt > self.cursor:
            self.cursor = nxt
        self.forwarded += n
        return n

    # -- the network half --------------------------------------------

    def _fetch(self, cursor: int) -> Dict[str, Any]:
        import json as _json
        import urllib.request
        url = f"{self.url}/sim/events?since={int(cursor)}&limit=100"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            return _json.loads(resp.read().decode("utf-8"))

    def start(self) -> "HarnessEventFeed":
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="omnilink-harness-events")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:                              # pragma: no cover
        import time as _time
        while not self._stop.is_set():
            self.poll_once(self._fetch)
            _time.sleep(self.period_s)

    def status(self) -> Dict[str, Any]:
        return {"url": self.url, "cursor": self.cursor,
                "forwarded": self.forwarded, "errors": self.errors,
                "types": list(self.FORWARD)}


def harness_feed(ring: EventRing, *, robot: str = "",
                 clock: Any = None) -> Optional[HarnessEventFeed]:
    """Start a feed iff `OMNISIM_HARNESS_URL` names one.

    `OMNISIM_BRIDGE_HARNESS_EVENTS=0` opts out (value-parsed) for a session
    that wants the harness attached for everything else but not mirrored
    onto the bridge's own stream.
    """
    # OMNISIM_HARNESS_URL names the World Harness to talk to, as a base URL.
    # One meaning, three consumers: this bridge feed, the MCP server and the
    # ROS 2 harness client. Here its PRESENCE is the signal that a harness
    # exists at all -- unset means "no harness", so the bridge raises no
    # forwarded events rather than polling something that is not there.
    url = (os.environ.get("OMNISIM_HARNESS_URL") or "").strip()
    if not url:
        return None
    opt = (os.environ.get("OMNISIM_BRIDGE_HARNESS_EVENTS") or "").strip().lower()
    if opt in ("0", "false", "no", "off"):
        return None
    return HarnessEventFeed(ring, url, robot=robot, clock=clock).start()


def emit_gate_refusal(ring: Optional[EventRing], tool: str, reason: str, *,
                      sim_time: float, step: int, robot: str = "",
                      origin: str = "tool", rule: str = "",
                      utterance: str = "") -> None:
    """`gate.refused`, filed where the gate said no.

    `origin` is the surface the refused call arrived on (`tool`, `prompt`,
    `relay`, `action`, `window`). `may_wake()` uses it: a refusal the agent
    caused in its own turn is already in front of it and must not buy a
    second model turn.
    """
    if ring is None:
        return
    ring.emit("gate.refused", sim_time=sim_time, step=step, robot=robot,
              tool=str(tool or ""), reason=str(reason or ""),
              rule=str(rule or ""), origin=str(origin or ""),
              utterance=str(utterance or "")[:200])
