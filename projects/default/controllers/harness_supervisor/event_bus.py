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

"""Supervisor-side event bus + producers.

This module is the source of `/sim/events` (supervisor side). It exports:

- `EventBus`: a bounded ring buffer of dicts, each tagged with a monotonic
  `seq` and one of the documented `type` strings.
- `ContactTracker`: per-step diff of contact pairs, emitting
  `contact.began` / `contact.ended`.
- `JointLimitTracker`: per-step joint position read, emitting
  `joint.limit_hit` with hysteresis to avoid oscillation across the band
  edge.
- `GripTracker`: stateful wrapper around `observe.detect_grips`, emitting
  `grip.acquired` / `grip.released` and tracking `since_t_ms`.
- `BreakRegistry`: armed BREAK conditions. When an event on the bus matches
  one, it takes the session's pause lease so the scene stops at the moment of
  interest, records the hit, and emits `break.hit` on the same bus.

All trackers are pure-Python and unit-testable with stub Supervisors.
"""

from __future__ import annotations

import collections
import re
from typing import Iterable

import observe


# Default event ring-buffer size. 4096 events at ~5 events/step and 32ms
# steps gives ~25s of headroom — well above the typical agent poll
# interval. Drops are surfaced via `events_total` so an agent can detect
# lag.
DEFAULT_BUFFER_SIZE = 4096

# ---------------------------------------------------------------------------
# The authoritative supervisor-side event-type list
# ---------------------------------------------------------------------------
#
# `GET /capabilities` publishes the event taxonomy, and the whole point of
# publishing it from the code is that a hand-maintained copy drifts: four of
# the twelve names PROTOCOL.md §10 used to document were never emitted
# (docs/developer/agent-native-api.md, Appendix B). So the list below is
# declared next to the producers AND cross-checked against them at runtime
# by `verify_event_types()`, which scans the emitting modules' own source
# for `emit("<type>", ...)` calls. A new producer with a new type name shows
# up as `undeclared`; a name that stops being emitted shows up as
# `declared_not_emitted`. Neither can pass silently into the published list.
#
# The harness contributes three more types of its own (`controller.log`,
# `world.warning`, `world.error`) — see LOG_EVENT_TYPES in
# scripts/harness/omnisim_harness.py, verified the same way.
SUPERVISOR_EVENT_TYPES = (
    "contact.began",
    "contact.ended",
    "joint.limit_hit",
    "grip.acquired",
    "grip.released",
    "damage.impact",
    "damage.state_transition",
    # The eleventh type overall (seven here + three harness log types + this
    # one). It is not produced by a per-step tracker: BreakRegistry emits it
    # when an ARMED break matched one of the types above and took the pause
    # lease, so `break.hit` is the record that the engine is now held.
    "break.hit",
)

# Which producer owns each type. `--light` mode (P6) skips the contact,
# joint-limit and grip trackers, so those types are *implemented but not
# active* in a light session — a distinction an agent filtering
# `/sim/events?types=` has to know, because the filter is an exact-match
# allowlist that returns an empty stream rather than an error.
EVENT_TYPE_PRODUCERS = {
    "contact.began": "ContactTracker",
    "contact.ended": "ContactTracker",
    "joint.limit_hit": "JointLimitTracker",
    "grip.acquired": "GripTracker",
    "grip.released": "GripTracker",
    "damage.impact": "DamageTracker",
    "damage.state_transition": "DamageTracker",
    "break.hit": "BreakRegistry",
}

# Producers disabled by `--light` (harness_supervisor.LIGHT_MODE).
LIGHT_MODE_DISABLED_PRODUCERS = ("ContactTracker", "JointLimitTracker", "GripTracker")

_EMIT_CALL_RE = re.compile(r'emit\(\s*"([a-z_]+\.[a-z_]+)"')


def scan_emit_calls(*sources: str) -> list[str]:
    """Event type strings passed to `emit(...)` in the given source texts.

    A source scan rather than a registry because producers emit literals
    inline; this way the published list is derived from the call sites that
    actually run, with no second place to update.
    """
    found: set[str] = set()
    for src in sources:
        if src:
            found.update(_EMIT_CALL_RE.findall(src))
    return sorted(found)


def verify_event_types(*sources: str) -> dict:
    """Cross-check SUPERVISOR_EVENT_TYPES against the emit call sites."""
    found = scan_emit_calls(*sources)
    declared = list(SUPERVISOR_EVENT_TYPES)
    undeclared = [t for t in found if t not in declared]
    missing = [t for t in declared if t not in found]
    return {
        "types": declared,
        "emitters_found": found,
        "undeclared": undeclared,
        "declared_not_emitted": missing,
        "verified": not undeclared and not missing,
        "source": "scanned emit() call sites in event_bus.py + harness_supervisor.py",
    }


class EventBus:
    """A monotonic-seq ring buffer.

    Producers call `emit(type, payload, t_sim_ms=...)` with their event
    type and JSON-serialisable payload. Consumers call `since(seq, limit,
    types=)` to drain. seq is monotonic and never resets — agents can
    cross-reference a seq across multiple drains.
    """

    def __init__(self, buffer_size: int = DEFAULT_BUFFER_SIZE):
        self._events: collections.deque = collections.deque(maxlen=buffer_size)
        self._counter = 0
        self._dropped = 0

    def emit(self, type_: str, payload: dict, t_sim_ms: float | None = None) -> dict:
        if len(self._events) == self._events.maxlen:
            self._dropped += 1
        self._counter += 1
        evt: dict = {
            "seq": self._counter,
            "type": type_,
        }
        if t_sim_ms is not None:
            evt["t_sim_ms"] = int(t_sim_ms)
        # Shallow merge — payloads should not collide with reserved keys
        # (seq, type, t_sim_ms). If they do, payload wins, which is fine
        # for testing but a producer bug in production.
        evt.update(payload)
        self._events.append(evt)
        return evt

    def since(self, since_seq: int, limit: int = 256,
              types: Iterable[str] | None = None) -> list[dict]:
        type_set = set(types) if types is not None else None
        out: list[dict] = []
        for evt in self._events:
            if evt["seq"] <= since_seq:
                continue
            if type_set is not None and evt["type"] not in type_set:
                continue
            out.append(evt)
            if len(out) >= limit:
                break
        return out

    @property
    def total(self) -> int:
        return self._counter

    @property
    def dropped(self) -> int:
        return self._dropped

    @property
    def buffered(self) -> int:
        return len(self._events)

    def reset(self) -> None:
        self._events.clear()
        self._counter = 0
        self._dropped = 0


# ---------------------------------------------------------------------------
# Contact tracker — per-step diff
# ---------------------------------------------------------------------------


class ContactTracker:
    """Emits contact.began / contact.ended by diffing contact-pair sets
    across steps.

    The diff key is the pair of DEF-or-id identifier STRINGS produced by
    `observe.contact_pairs`, not the contact point's `node_id`. That field does
    not mean what its name suggests: the engine streams the *queried* solid's
    own id (`OmSupervisorUtilities::pushContactPointsToStream`), so keying on it
    made every pair (X, X) and every emitted event claimed a body was touching
    ITSELF (measured: FLOOR reported ids [9,9,9,9] and CRATE_BOT [14,14,14,14]
    for the same four contacts). `observe.contact_pairs` joins the two sides on
    the shared contact point instead, which is the only field that identifies
    one physical contact from both bodies.
    """

    def __init__(self, supervisor, bus: EventBus):
        self._supervisor = supervisor
        self._bus = bus
        self._prev: set[tuple[str, str]] = set()
        self._last_point: dict[tuple[str, str], list[float]] = {}

    def poll(self, sim_time_ms: float) -> None:
        current: set[tuple[str, str]] = set()
        points: dict[tuple[str, str], list[float]] = {}
        for contact in observe.contact_pairs(self._supervisor):
            a = contact.get("a_def")
            b = contact.get("b_def")
            if a is None or b is None:
                continue          # half-contact: the partner is not a Solid we walk
            key = (a, b) if a <= b else (b, a)
            current.add(key)
            points.setdefault(key, contact.get("point") or [0.0, 0.0, 0.0])

        for key in current - self._prev:
            self._bus.emit("contact.began", {
                "a_def": key[0],
                "b_def": key[1],
                "point": points.get(key, [0.0, 0.0, 0.0]),
            }, t_sim_ms=sim_time_ms)
        for key in self._prev - current:
            self._bus.emit("contact.ended", {
                "a_def": key[0],
                "b_def": key[1],
            }, t_sim_ms=sim_time_ms)

        self._prev = current
        self._last_point = points

    def current_pairs(self) -> list[tuple[str, str]]:
        """Current contact pairs as DEF-or-id string tuples. Used by
        GripTracker without re-walking the scene.
        """
        return sorted(self._prev)


# ---------------------------------------------------------------------------
# Joint-limit tracker
# ---------------------------------------------------------------------------


class JointLimitTracker:
    """Emits joint.limit_hit with hysteresis.

    A joint is "in" the lower band when `position <= min_stop + tol_hit`
    and "out" again when `position > min_stop + tol_clear` (similarly
    for upper). Only the on-edge transition emits.
    """

    HIT_TOL = 1e-3
    CLEAR_TOL = 5e-3

    def __init__(self, supervisor, bus: EventBus):
        self._supervisor = supervisor
        self._bus = bus
        # state per joint id: None | "lower" | "upper"
        self._state: dict[int, str | None] = {}

    def poll(self, sim_time_ms: float) -> None:
        # Was: getRoot() + a full observe._walk() on EVERY basic step, plus
        # getId() and the jointParameters node handle re-fetched per joint per
        # step. That is the same pathology 3b952b61d fixed for solids, and it
        # slipped through because this called observe._walk directly instead of
        # going through the cache. The walk is the expensive half: it recurses
        # `children` AND `endPoint`, and a joint's endPoint is precisely what
        # drags it through the entire robot subtree, one getTypeName round-trip
        # per node -- and a round-trip is serviced at an engine step boundary.
        # observe.cached_joints() shares the solid cache's invalidation
        # (spawn/delete + a 120-poll backstop). minStop/maxStop stay LIVE reads
        # below so a retuned joint limit cannot produce a stale limit event.
        for j, jid, params in observe.cached_joints(self._supervisor):
            position = observe._sf_float(params, "position")
            min_stop = observe._sf_float(params, "minStop")
            max_stop = observe._sf_float(params, "maxStop")
            if position is None or min_stop is None or max_stop is None:
                continue
            if min_stop == 0.0 and max_stop == 0.0:
                # Unconstrained joint, skip.
                continue

            current_state = self._state.get(jid)
            new_state: str | None = current_state
            if current_state is None:
                if position <= min_stop + self.HIT_TOL:
                    new_state = "lower"
                elif position >= max_stop - self.HIT_TOL:
                    new_state = "upper"
            elif current_state == "lower":
                if position > min_stop + self.CLEAR_TOL:
                    new_state = None
            elif current_state == "upper":
                if position < max_stop - self.CLEAR_TOL:
                    new_state = None

            if new_state != current_state and new_state is not None:
                # Emit only on entering a band, not on leaving — agents
                # care about "joint hit a stop," not "joint left a stop."
                joint_name = None
                devices = j.getField("device")
                if devices is not None:
                    try:
                        if devices.getCount() > 0:
                            first = devices.getMFNode(0)
                            joint_name = observe._sf_string(first, "name")
                    except Exception:
                        pass
                # Find the owning robot via id table walk: cheap
                # because we already walked above. Skip for now —
                # agents can correlate via joint name.
                self._bus.emit("joint.limit_hit", {
                    "joint": joint_name,
                    "side": new_state,
                    "position": position,
                    "lower": min_stop,
                    "upper": max_stop,
                }, t_sim_ms=sim_time_ms)
            self._state[jid] = new_state


# ---------------------------------------------------------------------------
# Grip tracker
# ---------------------------------------------------------------------------


class GripTracker:
    """Emits grip.acquired / grip.released and tracks since_t_ms.

    Stable membership: a grip is reported only after it has held for
    `STABLE_STEPS` consecutive polls. This filters transient
    multi-finger touches that happen during approach.
    """

    STABLE_STEPS = 3

    def __init__(self, supervisor, bus: EventBus):
        self._supervisor = supervisor
        self._bus = bus
        # (gripper_def, held_def) -> {"first_seen_step": int,
        #                              "since_t_ms": int|None}
        self._candidates: dict[tuple[str, str], dict] = {}
        self._step_counter = 0

    def poll(self, contact_pairs: list[tuple[str, str]],
             robot_subtree_index: dict[str, str], sim_time_ms: float) -> None:
        self._step_counter += 1
        grips_now = observe.detect_grips(contact_pairs, robot_subtree_index)
        keys_now: set[tuple[str, str]] = set()
        for g in grips_now:
            keys_now.add((g["gripper_def"], g["held_def"]))

        # Track new candidates / promote stable ones to grip.acquired.
        for key in keys_now:
            entry = self._candidates.get(key)
            if entry is None:
                self._candidates[key] = {
                    "first_seen_step": self._step_counter,
                    "since_t_ms": None,
                }
            elif entry["since_t_ms"] is None and \
                    self._step_counter - entry["first_seen_step"] >= self.STABLE_STEPS:
                entry["since_t_ms"] = int(sim_time_ms)
                self._bus.emit("grip.acquired", {
                    "gripper_def": key[0],
                    "held_def": key[1],
                }, t_sim_ms=sim_time_ms)

        # Drop / release candidates that disappeared.
        gone = [k for k in self._candidates if k not in keys_now]
        for key in gone:
            entry = self._candidates.pop(key)
            if entry["since_t_ms"] is not None:
                self._bus.emit("grip.released", {
                    "gripper_def": key[0],
                    "held_def": key[1],
                    "held_for_ms": int(sim_time_ms) - entry["since_t_ms"],
                }, t_sim_ms=sim_time_ms)

    def active_grips(self) -> list[dict]:
        out: list[dict] = []
        for (gripper, held), entry in self._candidates.items():
            if entry["since_t_ms"] is None:
                continue
            out.append({
                "gripper_def": gripper,
                "held_def": held,
                "since_t_ms": entry["since_t_ms"],
            })
        return out


# ---------------------------------------------------------------------------
# Break registry — arm a condition, freeze the scene when it fires
# ---------------------------------------------------------------------------
#
# The debugging primitive this codebase was missing. `PauseLease` (in
# harness_supervisor.py) can hold the engine across HTTP calls; this decides
# WHEN to take it, from the events the trackers above already produce. The
# agent arms a condition, the simulation free-runs, and the first matching
# event stops it — so the scene an agent then reads is the scene at the moment
# of interest instead of the scene ~90 ms later.
#
# ⚠️ HONEST LATENCY. The hold is taken on the supervisor tick that scans the
# bus, not inside the solver sub-step. `BreakRegistry.scan()` runs in the main
# loop immediately after the per-step producers poll and again after the client
# drain, i.e. BEFORE the loop's next `step_or_hold`, so no further sim time
# passes for an event produced in the same iteration — but an event produced
# inside a multi-step `/sim/step` RPC is only seen when that RPC returns. Every
# hit therefore carries its own MEASURED `hold_latency_ms`
# (`paused_at_sim_ms - event.t_sim_ms`) and `hold_latency_steps`. Read the
# number; do not assume sub-step precision.

# Which event fields carry a DEF-or-id identity the `def` / `counterpart`
# filters can match. A type absent from a row here cannot be filtered by DEF
# at all, and arming one with a `def` filter earns a diagnostic rather than a
# silent never-match.
BREAK_IDENTITY_FIELDS: dict[str, tuple[str, ...]] = {
    "contact.began": ("a_def", "b_def"),
    "contact.ended": ("a_def", "b_def"),
    "grip.acquired": ("gripper_def", "held_def"),
    "grip.released": ("gripper_def", "held_def"),
    # joint.limit_hit carries a joint NAME, not an owning-robot DEF (see
    # JointLimitTracker: "agents can correlate via joint name"). damage.*
    # carries a part name, which is not a DEF either.
    "joint.limit_hit": (),
    "damage.impact": (),
    "damage.state_transition": (),
    "break.hit": (),
}

# Which event field the `joint` filter matches.
BREAK_JOINT_FIELDS: dict[str, str] = {"joint.limit_hit": "joint"}

BREAK_FILTER_KEYS = ("def", "counterpart", "joint")

# A break cannot be armed on `break.hit` (it would re-trigger on its own
# record) or on a harness-side log type (`controller.log`, `world.warning`,
# `world.error`) — those are produced by the harness's LogRingBuffer and never
# reach this bus, so the supervisor could never see them fire.
BREAK_UNBREAKABLE_TYPES = ("break.hit",)
HARNESS_LOG_EVENT_TYPES = ("controller.log", "world.warning", "world.error")

MAX_ARMED_BREAKS = 32

# Every `diagnostics[].code` POST /sim/break can return. Declared, not
# scattered, so `GET /capabilities` can publish the enum -- the same reason
# the event-type list is declared next to its producers.
BREAK_DIAGNOSTIC_CODES: tuple[str, ...] = (
    "types_missing",
    "event_type_unknown",
    "event_type_not_breakable",
    "event_type_not_on_supervisor_bus",
    "event_type_silenced_in_light_mode",
    "filter_key_unknown",
    "filter_key_not_matchable_for_armed_types",
    "break_limit_reached",
    "lease_ms_clamped",
)


class BreakRegistry:
    """Armed break conditions over the event bus.

    `arm()` validates a request against what THIS session can actually emit
    and either registers it or refuses it with a structured diagnostic — a
    break on `contact.began` in a light session can never fire, and accepting
    it silently is the blind spot these instruments exist to name.

    `scan()` is called from the supervisor main loop with the session's
    `PauseLease`. It drains the bus from its own cursor, and for every event
    matching an armed break it takes the lease (freezing the engine), records
    the hit and emits `break.hit` through the same `emit()` call form the
    capability self-scan reads.
    """

    def __init__(self, bus: EventBus, basic_step_ms: int,
                 disabled_producers: Iterable[str] = (),
                 lease_default_ms: int = 30_000,
                 lease_min_ms: int = 1_000,
                 lease_max_ms: int = 300_000) -> None:
        self._bus = bus
        self._basic_step_ms = max(1, int(basic_step_ms))
        self._disabled = tuple(disabled_producers)
        self.lease_default_ms = int(lease_default_ms)
        self.lease_min_ms = int(lease_min_ms)
        self.lease_max_ms = int(lease_max_ms)
        self._breaks: dict[str, dict] = {}
        self._order: list[str] = []
        self._counter = 0
        self._cursor = 0
        self.last_hit: dict | None = None
        # The ENGINE's own clock, and how far it moved over the last
        # supervisor tick. See note_tick(): this is the ruler the hold latency
        # actually has to be quoted in.
        self.engine_time_ms: float | None = None
        self.engine_tick_ms: float | None = None

    def note_tick(self, engine_time_ms: float) -> None:
        """Record the engine clock at the top of a supervisor tick.

        ⚠️ THE TWO CLOCKS ARE NOT THE SAME RULER, and the difference is the
        whole honest story about break latency. `sim_time_ms` is the
        SUPERVISOR's own counter, incremented once per loop iteration; the
        engine free-runs in `--mode=fast` and, on a small world, runs vastly
        faster than this controller's loop. MEASURED 2026-09-22 on the
        3-body break_drop fixture: a `/sim/reset` returned with the boxes
        re-lifted at supervisor t=8 ms and the very next tracker poll -- one
        supervisor tick later, supervisor t=16 ms -- already saw both boxes
        at rest on the floor, i.e. ~700 ms of ENGINE time inside one
        supervisor tick.

        A break is detected at a poll, so it lands within ONE supervisor tick
        of the physical event. `hold_latency_ms` (supervisor clock) is
        therefore always ~0 and, on its own, flatters the mechanism; the
        number that bounds the real thing is this tick measured in engine ms,
        published on every hit as `hold_latency_engine_ms_max`.
        """
        if engine_time_ms is None:
            return
        prev = self.engine_time_ms
        if prev is not None and engine_time_ms >= prev:
            self.engine_tick_ms = float(engine_time_ms) - float(prev)
        self.engine_time_ms = float(engine_time_ms)

    # -- introspection ------------------------------------------------------

    def suppressed_types(self) -> list[str]:
        """Declared types this session's flags silence (light mode et al)."""
        return sorted(t for t, p in EVENT_TYPE_PRODUCERS.items()
                      if p in self._disabled)

    def breakable_types(self) -> list[str]:
        suppressed = set(self.suppressed_types())
        return [t for t in SUPERVISOR_EVENT_TYPES
                if t not in BREAK_UNBREAKABLE_TYPES and t not in suppressed]

    def _public(self, brk: dict) -> dict:
        return {
            "break_id": brk["break_id"],
            "types": list(brk["types"]),
            "filter": dict(brk["filter"]),
            "once": brk["once"],
            "armed": brk["armed"],
            "hits": brk["hits"],
            "lease_ms": brk["lease_ms"],
            "armed_at_sim_ms": brk["armed_at_sim_ms"],
            "last_hit_sim_ms": brk["last_hit_sim_ms"],
        }

    def list_breaks(self) -> list[dict]:
        return [self._public(self._breaks[b]) for b in self._order
                if b in self._breaks]

    # -- arming -------------------------------------------------------------

    def arm(self, types, filt, lease_ms=None, once=True,
            sim_time_ms: float = 0.0) -> dict:
        """Register a break, or return a REFUSAL dict (`refused: True`).

        Refusals are values, not exceptions, because each one carries a
        `diagnostics[]` list the harness forwards verbatim in its 4xx body.
        """
        if not isinstance(types, (list, tuple)) or not types:
            return self._refusal(
                "BREAK_TYPES_REQUIRED",
                "'types' must be a non-empty list of event type strings",
                [{"code": "types_missing",
                  "detail": "POST /sim/break requires a 'types' list of event type strings",
                  "breakable_types": self.breakable_types()}])
        clean_types: list[str] = []
        for t in types:
            if not isinstance(t, str) or not t:
                return self._refusal(
                    "BREAK_TYPES_REQUIRED",
                    "'types' entries must be non-empty strings", [])
            if t not in clean_types:
                clean_types.append(t)

        suppressed = set(self.suppressed_types())
        diagnostics: list[dict] = []
        fatal: list[dict] = []
        for t in clean_types:
            if t in BREAK_UNBREAKABLE_TYPES:
                fatal.append({
                    "code": "event_type_not_breakable",
                    "event_type": t,
                    "detail": ("break.hit is the record a break WRITES; arming a break "
                               "on it would re-trigger on its own output"),
                    "workaround": "arm the underlying event type instead",
                })
            elif t in HARNESS_LOG_EVENT_TYPES:
                fatal.append({
                    "code": "event_type_not_on_supervisor_bus",
                    "event_type": t,
                    "detail": ("controller.log / world.warning / world.error are produced by "
                               "the HARNESS's log ring buffer from the engine's stdout, not by "
                               "the supervisor, so the supervisor never sees one and a break "
                               "armed on it could not fire"),
                    "workaround": ("poll GET /sim/events with log_since= for log events; they "
                                   "are not breakable"),
                })
            elif t not in SUPERVISOR_EVENT_TYPES:
                fatal.append({
                    "code": "event_type_unknown",
                    "event_type": t,
                    "detail": f"{t!r} is not a declared supervisor event type",
                    "known_types": list(SUPERVISOR_EVENT_TYPES),
                })
            elif t in suppressed:
                fatal.append({
                    "code": "event_type_silenced_in_light_mode",
                    "event_type": t,
                    "producer": EVENT_TYPE_PRODUCERS.get(t),
                    "detail": (f"this session runs with {', '.join(self._disabled)} not "
                               f"constructed (--light, or a per-tracker flag), so no "
                               f"{t} is ever emitted and a break armed on it could "
                               f"never fire"),
                    "workaround": ("reload the world with light disabled -- POST /world/load "
                                   "with {\"light\": false} for all three trackers, or a "
                                   "`tracking` object naming just the ones you need -- then "
                                   "arm the break again"),
                    "silenced_types": sorted(suppressed),
                })
        if fatal:
            codes = sorted({d["code"] for d in fatal})
            return self._refusal(
                "BREAK_EVENT_TYPE_UNAVAILABLE",
                ("refused: a break on "
                 + ", ".join(sorted({d["event_type"] for d in fatal}))
                 + " could never fire in this session ("
                 + ", ".join(codes) + ")"),
                fatal + diagnostics)

        clean_filter: dict = {}
        if filt is not None:
            if not isinstance(filt, dict):
                return self._refusal("BREAK_FILTER_INVALID",
                                     "'filter' must be an object", [])
            for key, value in filt.items():
                if key not in BREAK_FILTER_KEYS:
                    return self._refusal(
                        "BREAK_FILTER_INVALID",
                        f"unknown filter key {key!r}",
                        [{"code": "filter_key_unknown", "key": key,
                          "known_keys": list(BREAK_FILTER_KEYS)}])
                if value is None:
                    continue
                if not isinstance(value, str) or not value:
                    return self._refusal(
                        "BREAK_FILTER_INVALID",
                        f"filter.{key} must be a non-empty string", [])
                clean_filter[key] = value

        # Non-fatal honesty: a filter key that no armed type carries does not
        # stop the break from firing, it stops it from NARROWING — which is the
        # opposite failure and just as surprising.
        for key in ("def", "counterpart"):
            if key in clean_filter and not any(
                    BREAK_IDENTITY_FIELDS.get(t) for t in clean_types):
                diagnostics.append({
                    "code": "filter_key_not_matchable_for_armed_types",
                    "key": key,
                    "armed_types": list(clean_types),
                    "detail": (f"none of the armed types carries a DEF identity field, so "
                               f"filter.{key} cannot narrow this break; it will fire on any "
                               f"event of the armed types"),
                    "matchable_types": sorted(t for t, f in BREAK_IDENTITY_FIELDS.items() if f),
                })
        if "joint" in clean_filter and not any(
                t in BREAK_JOINT_FIELDS for t in clean_types):
            diagnostics.append({
                "code": "filter_key_not_matchable_for_armed_types",
                "key": "joint",
                "armed_types": list(clean_types),
                "detail": ("only joint.limit_hit carries a joint name, so filter.joint "
                           "cannot narrow this break"),
                "matchable_types": sorted(BREAK_JOINT_FIELDS),
            })

        if len(self._breaks) >= MAX_ARMED_BREAKS:
            return self._refusal(
                "BREAK_LIMIT_REACHED",
                f"at most {MAX_ARMED_BREAKS} breaks may be armed at once",
                [{"code": "break_limit_reached", "limit": MAX_ARMED_BREAKS,
                  "armed": len(self._breaks)}])

        if lease_ms is None:
            effective_lease = self.lease_default_ms
        else:
            try:
                effective_lease = int(lease_ms)
            except (TypeError, ValueError):
                return self._refusal("BREAK_LEASE_INVALID",
                                     "'lease_ms' must be an integer", [])
            clamped = max(self.lease_min_ms, min(effective_lease, self.lease_max_ms))
            if clamped != effective_lease:
                diagnostics.append({
                    "code": "lease_ms_clamped",
                    "requested_ms": effective_lease,
                    "effective_ms": clamped,
                    "detail": ("the lease deadline is a safety property: a client that arms a "
                               "break and then dies must not freeze the engine forever"),
                    "bounds_ms": [self.lease_min_ms, self.lease_max_ms],
                })
            effective_lease = clamped

        self._counter += 1
        break_id = f"brk{self._counter}"
        record = {
            "break_id": break_id,
            "types": clean_types,
            "filter": clean_filter,
            "once": bool(once),
            "armed": True,
            "hits": 0,
            "lease_ms": effective_lease,
            "armed_at_sim_ms": float(sim_time_ms),
            "last_hit_sim_ms": None,
        }
        self._breaks[break_id] = record
        self._order.append(break_id)
        # A break must never fire on an event that happened BEFORE it was
        # armed: fast-forward the scan cursor to the current end of the bus.
        self._cursor = max(self._cursor, self._bus.total)
        out = self._public(record)
        out["armed_types"] = list(clean_types)
        out["diagnostics"] = diagnostics
        out["refused"] = False
        return out

    def remove(self, break_id: str) -> dict:
        existed = self._breaks.pop(break_id, None) is not None
        if existed and break_id in self._order:
            self._order.remove(break_id)
        return {"break_id": break_id, "removed": existed}

    def clear(self) -> int:
        n = len(self._breaks)
        self._breaks.clear()
        self._order.clear()
        return n

    @staticmethod
    def _refusal(code: str, error: str, diagnostics: list[dict]) -> dict:
        return {"refused": True, "code": code, "error": error,
                "diagnostics": diagnostics, "armed_types": []}

    # -- matching -----------------------------------------------------------

    @staticmethod
    def matches(brk: dict, evt: dict) -> bool:
        etype = evt.get("type")
        if etype not in brk["types"]:
            return False
        filt = brk["filter"]
        if not filt:
            return True
        idents = [evt.get(f) for f in BREAK_IDENTITY_FIELDS.get(etype, ())]
        idents = [v for v in idents if isinstance(v, str)]
        want_def = filt.get("def")
        want_other = filt.get("counterpart")
        if want_def is not None and want_def not in idents:
            return False
        if want_other is not None:
            if want_other not in idents:
                return False
            if want_def is not None and want_def == want_other \
                    and idents.count(want_def) < 2:
                return False
        want_joint = filt.get("joint")
        if want_joint is not None:
            field = BREAK_JOINT_FIELDS.get(etype)
            if field is None or evt.get(field) != want_joint:
                return False
        return True

    # -- the scan the main loop runs ---------------------------------------

    def scan(self, supervisor, lease, sim_time_ms: float,
             limit: int = 512) -> list[dict]:
        """Drain new events; freeze the engine on the first armed match.

        Returns the hit records produced by this call (usually empty).
        `lease` is the session's `PauseLease`; `supervisor` is handed to it.
        """
        # Sample the ENGINE clock here rather than from the main loop: the
        # main loop skips its own bookkeeping while a lease is held, so a
        # session that is being single-stepped would stamp every hit with a
        # clock frozen at whenever it was last free-running (measured: a hit
        # carrying engine_time_ms_at_hold = 196000 on a scene whose engine
        # clock was 1016). `supervisor.getTime()` is a local cached read, no
        # IPC, and it is updated by exactly the thing that matters -- a step.
        if supervisor is not None:
            try:
                self.note_tick(supervisor.getTime() * 1000.0)
            except Exception:  # noqa: BLE001 -- a stub supervisor has no clock
                pass
        if not any(b["armed"] for b in self._breaks.values()):
            # Nothing armed: keep the cursor at the end of the bus so a break
            # armed later cannot fire on history.
            self._cursor = self._bus.total
            return []
        events = self._bus.since(self._cursor, limit=limit)
        if not events:
            return []
        self._cursor = events[-1]["seq"]
        hits: list[dict] = []
        for evt in events:
            if evt.get("type") in BREAK_UNBREAKABLE_TYPES:
                continue
            for break_id in list(self._order):
                brk = self._breaks.get(break_id)
                if brk is None or not brk["armed"]:
                    continue
                if not self.matches(brk, evt):
                    continue
                hits.append(self._fire(brk, evt, supervisor, lease, sim_time_ms))
        return hits

    def _fire(self, brk: dict, evt: dict, supervisor, lease,
              sim_time_ms: float) -> dict:
        # Take (or extend) the pause lease FIRST: everything after this runs
        # against a frozen engine.
        lease_status: dict = {}
        if lease is not None:
            try:
                lease_status = lease.take(supervisor, brk["lease_ms"], sim_time_ms)
            except Exception as exc:  # noqa: BLE001 -- a failed hold must still be recorded
                lease_status = {"paused": False, "lease_error": str(exc)}
        paused_at = float(sim_time_ms)
        t_event = evt.get("t_sim_ms")
        latency_ms = (paused_at - float(t_event)) if t_event is not None else None
        brk["hits"] += 1
        brk["last_hit_sim_ms"] = paused_at
        if brk["once"]:
            brk["armed"] = False
        payload = {
            "break_id": brk["break_id"],
            "matched_type": evt.get("type"),
            "matched_seq": evt.get("seq"),
            "matched": {k: v for k, v in evt.items() if k not in ("seq", "type")},
            "paused_at_sim_ms": paused_at,
            # MEASURED, never assumed: how much sim time passed between the
            # event and the hold. Zero when both happened on the same tick.
            "hold_latency_ms": latency_ms,
            "hold_latency_steps": (None if latency_ms is None
                                   else int(round(latency_ms / self._basic_step_ms))),
            # The engine's own clock at the hold, and the width of the
            # supervisor tick the detection happened on -- which is the
            # HONEST upper bound on how much simulated time passed between
            # the physical event and the freeze. `hold_latency_ms` above is
            # the same delta measured on the supervisor's own counter, where
            # it is ~0 by construction.
            "engine_time_ms_at_hold": self.engine_time_ms,
            "engine_tick_ms": self.engine_tick_ms,
            "hold_latency_engine_ms_max": self.engine_tick_ms,
            "paused": bool(lease_status.get("paused", False)),
            "lease_ms": brk["lease_ms"],
            "lease_remaining_ms": lease_status.get("lease_remaining_ms"),
            "once": brk["once"],
            "still_armed": brk["armed"],
            "hits": brk["hits"],
        }
        if "lease_error" in lease_status:
            payload["lease_error"] = lease_status["lease_error"]
        self._bus.emit("break.hit", payload, t_sim_ms=sim_time_ms)
        record = dict(payload)
        record["event"] = dict(evt)
        self.last_hit = record
        return record
