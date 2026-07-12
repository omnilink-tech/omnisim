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
from typing import Iterable

import observe


# Default event ring-buffer size. 4096 events at ~5 events/step and 32ms
# steps gives ~25s of headroom — well above the typical agent poll
# interval. Drops are surfaced via `events_total` so an agent can detect
# lag.
DEFAULT_BUFFER_SIZE = 4096


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

    Uses node ids (ints, stable per-process) for the diff key — DEF
    strings are nice for the wire payload but two nodes might share a
    DEF or have none. We cache id -> def lookups for the payload.
    """

    def __init__(self, supervisor, bus: EventBus):
        self._supervisor = supervisor
        self._bus = bus
        self._prev: set[tuple[int, int]] = set()
        self._id_to_def: dict[int, str] = {}
        self._last_point: dict[tuple[int, int], list[float]] = {}

    def _refresh_id_table(self) -> list:
        # Walk solids cheaply; cache identifiers for the duration of one
        # step. Cheap enough to do every step on small worlds; large
        # worlds may want a longer cadence — that's a future tuning
        # knob.
        root = self._supervisor.getRoot()
        solids: list = []
        observe._walk(root, observe._is_solid, solids)
        self._id_to_def.clear()
        for s in solids:
            try:
                sid = s.getId()
            except Exception:
                continue
            self._id_to_def[sid] = observe._node_def_or_id(s)
        return solids

    def poll(self, sim_time_ms: float) -> None:
        solids = self._refresh_id_table()
        current: set[tuple[int, int]] = set()
        points: dict[tuple[int, int], list[float]] = {}
        for s in solids:
            try:
                sid = s.getId()
                cps = s.getContactPoints(False) or []
            except Exception:
                continue
            for cp in cps:
                other_id = getattr(cp, "node_id", None)
                if other_id is None:
                    continue
                key = (min(sid, other_id), max(sid, other_id))
                current.add(key)
                if key not in points:
                    try:
                        points[key] = [float(x) for x in cp.point]
                    except Exception:
                        points[key] = [0.0, 0.0, 0.0]

        # New contacts
        for key in current - self._prev:
            a, b = key
            self._bus.emit("contact.began", {
                "a_def": self._id_to_def.get(a, f"#{a}"),
                "b_def": self._id_to_def.get(b, f"#{b}"),
                "point": points.get(key, [0.0, 0.0, 0.0]),
            }, t_sim_ms=sim_time_ms)
        # Ended contacts
        for key in self._prev - current:
            a, b = key
            self._bus.emit("contact.ended", {
                "a_def": self._id_to_def.get(a, f"#{a}"),
                "b_def": self._id_to_def.get(b, f"#{b}"),
            }, t_sim_ms=sim_time_ms)

        self._prev = current
        self._last_point = points

    def current_pairs(self) -> list[tuple[str, str]]:
        """Return current contact pairs as DEF-or-id string tuples. Used
        by GripTracker without re-walking the scene.
        """
        out: list[tuple[str, str]] = []
        for a, b in self._prev:
            out.append((
                self._id_to_def.get(a, f"#{a}"),
                self._id_to_def.get(b, f"#{b}"),
            ))
        return out


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
        root = self._supervisor.getRoot()
        joints: list = []
        observe._walk(root, observe._is_joint, joints)
        for j in joints:
            try:
                jid = j.getId()
            except Exception:
                continue
            params = observe._joint_parameters(j)
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
