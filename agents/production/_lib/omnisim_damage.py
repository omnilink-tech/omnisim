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

"""omnisim_damage — agent-side helpers for the OmniSim damage system.

Phase 13 of docs/developer/damage-system-plan.md. Wraps the harness's
`/robot/damage*` HTTP endpoints in ergonomic Python so agents don't
hand-roll urllib + JSON every time. Three layers:

1. `DamageClient` — a thin HTTP wrapper. Caches `last_step_id` for
   incremental event polling so an agent loop doesn't re-process old
   events. Returns typed objects, not raw JSON.

2. `DamageState` / `DamageEvent` — typed snapshots with convenience
   methods (`broken_parts()`, `health()`, `is_state_transition()`).

3. `ReactionPolicy` — pre-built reaction recipes the agent author wires
   into their loop. Each is a small function that polls the client and
   does the right thing on transitions.

Usage:

    from omnisim_damage import DamageClient, ReactionPolicy

    client = DamageClient(host="127.0.0.1", port=6789)

    state = client.snapshot()
    if state.is_game_over:
        ...

    for evt in client.events():
        if evt.is_state_transition() and evt.to_state == "broken":
            print(f"{evt.part} just broke!")

    # Or use a ready-made policy:
    ReactionPolicy.disengage_on_chassis_broken(client, on_disengage=quit_loop)
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Typed event / state objects
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PartHealth:
    """Per-part record returned in DamageState.damage."""

    part: str
    state: str  # "pristine" / "scuffed" / "damaged" / "broken"
    hp: float
    hp_max: float
    last_impact_step: int
    last_impact_J: float
    total_impulse_J: float

    @classmethod
    def from_dict(cls, part: str, raw: dict) -> "PartHealth":
        return cls(
            part=part,
            state=str(raw.get("state", "pristine")),
            hp=float(raw.get("hp", 0.0)),
            hp_max=float(raw.get("hp_max", 1.0)),
            last_impact_step=int(raw.get("last_impact_step", 0)),
            last_impact_J=float(raw.get("last_impact_J", 0.0)),
            total_impulse_J=float(raw.get("total_impulse_J", 0.0)),
        )

    @property
    def fraction(self) -> float:
        """HP as a fraction of max (0.0–1.0). 0.0 if hp_max is bogus."""
        return self.hp / self.hp_max if self.hp_max > 0 else 0.0


@dataclass(frozen=True)
class DamageState:
    """Snapshot of the supervisor's damage state, returned by snapshot()."""

    robot: str
    attached: bool
    is_game_over: bool
    damage: dict[str, PartHealth]
    events_total: int
    events_buffered: int
    events_dropped: int
    contacts_seen: int
    raw: dict = field(repr=False, default_factory=dict)

    @classmethod
    def from_dict(cls, raw: dict) -> "DamageState":
        damage_raw = raw.get("damage") or {}
        return cls(
            robot=str(raw.get("robot", "")),
            attached=bool(raw.get("attached", False)),
            is_game_over=bool(raw.get("game_over", False)),
            damage={part: PartHealth.from_dict(part, rec)
                    for part, rec in damage_raw.items()},
            events_total=int(raw.get("events_total", 0)),
            events_buffered=int(raw.get("events_buffered", 0)),
            events_dropped=int(raw.get("events_dropped", 0)),
            contacts_seen=int(raw.get("contacts_seen", 0)),
            raw=raw,
        )

    def health(self, part: str) -> float:
        """HP fraction (0.0–1.0) for `part`, or 0.0 if unknown."""
        rec = self.damage.get(part)
        return rec.fraction if rec is not None else 0.0

    def state_of(self, part: str) -> str:
        """Logical state of `part`, or 'pristine' if unknown."""
        rec = self.damage.get(part)
        return rec.state if rec is not None else "pristine"

    def broken_parts(self) -> list[str]:
        return sorted(p for p, r in self.damage.items() if r.state == "broken")

    def damaged_parts(self) -> list[str]:
        """Parts at `damaged` or worse (i.e. damaged or broken)."""
        return sorted(p for p, r in self.damage.items()
                      if r.state in ("damaged", "broken"))

    def any_damage(self) -> bool:
        return any(r.state != "pristine" for r in self.damage.values())


@dataclass(frozen=True)
class DamageEvent:
    """Single event from the ring buffer. May be an impact or a state
    transition; check via `is_state_transition()` / `is_impact()`.
    """

    type: str
    step_id: int
    sim_time_ms: int
    part: str
    raw: dict = field(repr=False, default_factory=dict)

    @classmethod
    def from_dict(cls, raw: dict) -> "DamageEvent":
        return cls(
            type=str(raw.get("type", "impact")),
            step_id=int(raw.get("step_id", 0)),
            sim_time_ms=int(raw.get("sim_time_ms", 0)),
            part=str(raw.get("part", "")),
            raw=raw,
        )

    def is_impact(self) -> bool:
        return self.type == "impact"

    def is_state_transition(self) -> bool:
        return self.type == "state_transition"

    @property
    def impulse_J(self) -> float:
        return float(self.raw.get("impulse_J", 0.0))

    @property
    def from_state(self) -> str:
        return str(self.raw.get("from_state", ""))

    @property
    def to_state(self) -> str:
        return str(self.raw.get("to_state", ""))

    @property
    def hp(self) -> float:
        return float(self.raw.get("hp", 0.0))


# ---------------------------------------------------------------------------
# DamageClient
# ---------------------------------------------------------------------------


class DamageClientError(Exception):
    pass


class DamageClient:
    """Thin HTTP wrapper around the harness's /robot/damage* endpoints.

    Holds the cursor for incremental event polling so successive calls
    to `events()` only see new entries. Stateless across reconnects —
    if the harness restarts and replays from step_id=1 the cursor will
    reset on the first poll.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 6789,
                 timeout_s: float = 2.0):
        self.host = host
        self.port = port
        self.timeout_s = float(timeout_s)
        self._last_step_id = 0

    # --- internal HTTP helpers ----------------------------------------

    def _url(self, path: str, query: dict | None = None) -> str:
        q = ""
        if query:
            q = "?" + urllib.parse.urlencode(query)
        return f"http://{self.host}:{self.port}{path}{q}"

    def _get(self, path: str, query: dict | None = None) -> dict:
        url = self._url(path, query)
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise DamageClientError(f"GET {url} failed: {exc}") from exc
        try:
            return json.loads(body)
        except json.JSONDecodeError as exc:
            raise DamageClientError(f"GET {url} returned non-JSON: {exc}") from exc

    def _post(self, path: str, body: dict | None = None) -> dict:
        url = self._url(path)
        data = json.dumps(body or {}).encode("utf-8")
        req = urllib.request.Request(
            url, data=data, method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                raw = resp.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise DamageClientError(f"POST {url} failed: {exc}") from exc
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            raise DamageClientError(f"POST {url} returned non-JSON: {exc}") from exc

    # --- public API ---------------------------------------------------

    def snapshot(self) -> DamageState:
        """Fetch the current damage state. One HTTP roundtrip."""
        return DamageState.from_dict(self._get("/robot/damage"))

    def events(self, since: int | None = None,
               limit: int = 256) -> list[DamageEvent]:
        """Fetch new events since the last call (or since `since` if
        provided). Updates the internal cursor on success so successive
        calls only see new entries.
        """
        cursor = self._last_step_id if since is None else int(since)
        raw = self._get("/robot/damage/events",
                        {"since": cursor, "limit": int(limit)})
        events_raw = raw.get("events") or []
        last = int(raw.get("last_step_id", cursor))
        if since is None:
            self._last_step_id = max(self._last_step_id, last)
        return [DamageEvent.from_dict(e) for e in events_raw]

    def reset(self) -> None:
        """Heal all parts back to pristine without resetting the sim.
        Resets the local event cursor too — anything before this call
        is now ancient history.
        """
        self._post("/robot/damage/reset")
        self._last_step_id = 0

    def inject(self, part: str, *, state: str | None = None,
               hp_delta: float | None = None) -> PartHealth:
        """Test/debug hook: directly set a part's state or apply a HP
        delta, bypassing the contact pipeline. Returns the part's
        post-inject record. Raises DamageClientError on unknown part.
        """
        body: dict = {"part": part}
        if state is not None:
            body["state"] = state
        if hp_delta is not None:
            body["hp_delta"] = float(hp_delta)
        raw = self._post("/robot/damage/inject", body)
        if "error" in raw:
            raise DamageClientError(f"inject {part}: {raw['error']}")
        return PartHealth.from_dict(part, raw)

    def reset_cursor(self) -> None:
        """Reset only the local events cursor without touching server
        state. Useful when an agent wants to re-process the entire
        buffer."""
        self._last_step_id = 0


# ---------------------------------------------------------------------------
# ReactionPolicy — pre-built reaction recipes
# ---------------------------------------------------------------------------


class ReactionPolicy:
    """Opinionated, reusable reaction recipes. Each method runs against
    a `DamageClient` and either:

    - polls once and acts (`*_once` variants), or
    - blocks and watches indefinitely (`watch_*` variants), suitable
      for running on a background thread.

    The watch variants return when the predicate fires, after invoking
    the supplied callback (if any). Exit conditions are documented per
    method.
    """

    # ---- one-shot snapshot helpers ----------------------------------

    @staticmethod
    def is_critical(client: DamageClient, threshold: float = 0.2) -> bool:
        """True if any part's HP fraction is at or below `threshold`,
        OR the game_over flag is set.
        """
        state = client.snapshot()
        if state.is_game_over:
            return True
        return any(rec.fraction <= threshold for rec in state.damage.values())

    @staticmethod
    def should_disengage(client: DamageClient) -> bool:
        """True when the chassis is broken (game_over) — agents should
        stop their primary task and report.
        """
        return client.snapshot().is_game_over

    # ---- watcher variants -------------------------------------------

    @staticmethod
    def watch_for_chassis_broken(client: DamageClient,
                                  on_break: callable | None = None,
                                  poll_interval_s: float = 0.25,
                                  max_wait_s: float | None = None) -> bool:
        """Block until the chassis transitions to broken (or max_wait_s
        elapses, returning False). Calls `on_break(event)` once when
        the transition fires. Useful as a single-shot "wake up when
        the robot dies" sentinel.
        """
        deadline = (time.monotonic() + max_wait_s) if max_wait_s else None
        while True:
            for evt in client.events():
                if (evt.is_state_transition() and evt.part == "chassis"
                        and evt.to_state == "broken"):
                    if on_break is not None:
                        on_break(evt)
                    return True
            if deadline is not None and time.monotonic() >= deadline:
                return False
            time.sleep(poll_interval_s)

    @staticmethod
    def watch_for_state_transitions(client: DamageClient,
                                     on_transition: callable,
                                     parts: set[str] | None = None,
                                     poll_interval_s: float = 0.25,
                                     max_wait_s: float | None = None) -> int:
        """Block and emit each state-transition event to
        `on_transition(event)`. Optionally filter by part name. Returns
        the count of emitted transitions; exits when max_wait_s
        elapses (or never if max_wait_s is None — caller should run
        this on a background thread or signal-cancel).
        """
        deadline = (time.monotonic() + max_wait_s) if max_wait_s else None
        emitted = 0
        while True:
            for evt in client.events():
                if not evt.is_state_transition():
                    continue
                if parts is not None and evt.part not in parts:
                    continue
                on_transition(evt)
                emitted += 1
            if deadline is not None and time.monotonic() >= deadline:
                return emitted
            time.sleep(poll_interval_s)

    # ---- compound policies (single-step) ----------------------------

    @staticmethod
    def disengage_on_chassis_broken(client: DamageClient,
                                     on_disengage: callable | None = None,
                                     poll_interval_s: float = 0.25) -> None:
        """Convenience: alias for watch_for_chassis_broken with no
        timeout. Designed for use in an agent's main loop or worker
        thread; once the callback runs the function returns and the
        caller decides what to do next.
        """
        ReactionPolicy.watch_for_chassis_broken(
            client,
            on_break=on_disengage,
            poll_interval_s=poll_interval_s,
            max_wait_s=None,
        )
