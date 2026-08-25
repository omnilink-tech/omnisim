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
