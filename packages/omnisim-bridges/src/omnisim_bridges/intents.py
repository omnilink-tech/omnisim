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

"""Deferred intents: the layer that makes "do X WHEN Y" survive the turn.

WHY THIS EXISTS
---------------
Every tool an OmniLink bridge exposed was IMMEDIATE -- stop_robot,
drive_forward, resume_autonomy. There was no way for a model to express
"stop after you park this cart" or "don't go in there until I say so", so a
conditional order got a natural-language promise ("I'll park it and then
wait for your signal") plus a read-only get_robot_state, and *nothing was
recorded anywhere*. The chat turn ended, the promise evaporated, and 60 s
later the idle loop resumed as though the operator had never spoken.

Measured failure ladder that motivated this module (tug_a, cloud engine):

  1. "stop"                                        -> stop_robot. WORKED.
  2. "after you place this cart, stop until I tell
      you to continue"                             -> prose promise, never paused.
  3. "do one more delivery, then park and wait"    -> prose promise, kept cycling.
  4. "if you pick up another cart, tell me first"  -> prose promise, no tools at all.
  5. "don't enter the pick cell until I say so"    -> called stop_robot, which
     LOOKED compliant -- but it only stopped NOW. The constraint was not
     stored, so the 60 s auto-resume walked the tug straight back into the
     cell. A robot that agrees to a rule, looks compliant, then violates it
     is the worst failure mode on this list.

WHAT THIS IS
------------
A per-robot store of things the operator asked for that CANNOT be done now:

  * PENDING INTENTS -- (trigger, action) pairs evaluated by the robot's own
    autonomy loop at clean task boundaries, on task events, and on leg
    transitions. Actions are "pause" and "notify".
  * CONSTRAINTS -- standing restrictions the loop checks BEFORE acting, so a
    forbidden zone makes the loop DECLINE that work and keep doing other
    work, rather than merely halting once.
  * THE HOLD -- the fix for "until I tell you to continue". The bridges'
    pause is a 60 s quiet window that auto-resumes; a held robot does not
    auto-resume at all, and only an explicit resume_autonomy clears it.

Everything is bounded. An intent whose trigger never fires EXPIRES and logs;
a constraint expires; even the hold expires (generously) rather than wedging
a robot forever. Nothing in here can deadlock the line.

HONESTY CONTRACT
----------------
``schedule`` / ``set_constraint`` REFUSE what they cannot express and return
a ``say`` string the model is told to relay verbatim, e.g. "I can't schedule
that -- I can stop after the current task instead." A silent accept would
reproduce the exact bug this module exists to kill: an agreement with
nothing behind it.

THREADING
---------
Touched by the HTTP thread (tool dispatch, /state) and the autonomy thread
(boundary evaluation). One RLock guards the whole store; every accessor
returns copies.
"""

from __future__ import annotations

import json
import os
import re
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

__all__ = [
    "IntentStore",
    "DEFAULT_TTL_S",
    "DEFAULT_HOLD_MAX_S",
    "DEFAULT_CONSTRAINT_TTL_S",
    "CANONICAL_CONDITIONS",
]

# Bounded by construction. A never-satisfied trigger must age out and say so
# in the log, not sit in the store forever pretending to be live.
DEFAULT_TTL_S = 900.0             # 15 min for a pending intent
DEFAULT_CONSTRAINT_TTL_S = 1800.0  # 30 min for a standing rule
DEFAULT_HOLD_MAX_S = 1800.0       # 30 min ceiling on "hold until told"

# Canonical trigger vocabulary. Robot-specific aliases map onto these so the
# operator's own noun ("deliveries", "picks") reaches the same machinery.
CANONICAL_CONDITIONS = (
    "after_current_task",
    "after_n_tasks",
    "on_next_pickup",
    "at_leg",
)

_ALIASES = {
    "after_current_task": "after_current_task",
    "after_this_task": "after_current_task",
    "after_current_delivery": "after_current_task",
    "after_current_pick": "after_current_task",
    "after_current_cart": "after_current_task",
    "after_n_tasks": "after_n_tasks",
    "after_n_deliveries": "after_n_tasks",
    "after_n_picks": "after_n_tasks",
    "after_n_cycles": "after_n_tasks",
    "on_next_pickup": "on_next_pickup",
    "on_next_pick": "on_next_pickup",
    "on_next_cart": "on_next_pickup",
    "on_next_grasp": "on_next_pickup",
    "at_leg": "at_leg",
    "on_leg": "at_leg",
}


def _now() -> float:
    return time.time()


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


def _default_state_path(robot_id: str) -> str:
    """Where one robot's commitments live between runs.

    Deliberately OUTSIDE the repo (a temp dir by default) so a demo run
    never leaves dirty files in a checkout, and overridable with
    OMNILINK_INTENT_STATE_DIR for a deployment that wants them somewhere
    durable.
    """
    base = os.environ.get("OMNILINK_INTENT_STATE_DIR") or os.path.join(
        tempfile.gettempdir(), "omnisim_intents")
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", str(robot_id) or "robot")[:64]
    # Honour the same isolation tag the platform identity uses. Without this
    # the tag isolates the profile, the memory and the action journal but NOT
    # the commitments -- measured: a tagged test run left a 30-minute
    # `no_new_pickups` constraint sitting in the SHIPPED demo's state file,
    # so the next plain launch would have come up with its dispatch tug
    # refusing work for reasons nobody could see.
    tag = (os.environ.get("OMNILINK_AGENT_TAG") or "").strip()
    if tag:
        safe += "-" + re.sub(r"[^A-Za-z0-9_.-]", "-", tag)[:32].strip("-")
    return os.path.join(base, f"intents_{safe}.json")


# Bump when the on-disk shape changes incompatibly; an older file is then
# ignored rather than half-restored into a store that cannot read it.
STATE_VERSION = 1

# How stale saved state may be and still be restored.
#
# Persistence exists to survive a RELOAD -- seconds to a minute. It is not
# meant to carry an order across sessions: measured on the first live run,
# a constraint set during a chat session silently restricted the NEXT plain
# `launch.bat` demo, which came up refusing the park row for no reason the
# operator could see. An order given to a robot that has since been shut down
# and restarted hours later is not a live commitment, it is a ghost. Beyond
# this window the state is ignored (and said so in the log) rather than
# resurrected.
STALE_STATE_S = 900.0


class _Rec(dict):
    """Plain dict record -- JSON-serialisable straight into /state."""


class IntentStore:
    """Per-robot deferred-intent + constraint store.

    Parameters
    ----------
    robot_id
        Used only in log lines.
    task_noun / task_plural
        What one unit of work is called on THIS robot ("delivery" on the
        dispatch tug, "pick" on the arm). Purely cosmetic, but it is what
        makes the refusal strings and the /state block read honestly.
    conditions
        The trigger names this robot actually advertises, in the order the
        tool description should list them. Anything outside the alias table
        is refused.
    rules
        ``{rule_name: human description}`` -- the standing restrictions this
        robot's loop genuinely enforces. A rule not in here is REFUSED; that
        is the whole point (see the honesty contract above).
    legs
        Known ``at_leg`` values, for the refusal message.
    log
        ``f(str)`` sink. The bridges pass their idle-loop logger so intent
        events land in OMNILINK_IDLE_LOG next to the work they gate.
    on_pause
        ``f(hold: bool, intent: dict)`` -- called when a pause intent fires.
        The bridge stops motion and (when ``hold`` is False) arms its normal
        quiet-window pause. The HOLD itself lives in this store.
    on_notify
        ``f(text: str, intent: dict)`` -- called when a notify intent fires.
    """

    def __init__(self,
                 robot_id: str,
                 *,
                 task_noun: str = "task",
                 task_plural: str = "tasks",
                 conditions: Sequence[str] = CANONICAL_CONDITIONS,
                 rules: Optional[Dict[str, str]] = None,
                 legs: Sequence[str] = (),
                 log: Optional[Callable[[str], None]] = None,
                 on_pause: Optional[Callable[[bool, dict], None]] = None,
                 on_notify: Optional[Callable[[str, dict], None]] = None,
                 ttl_s: float = DEFAULT_TTL_S,
                 constraint_ttl_s: float = DEFAULT_CONSTRAINT_TTL_S,
                 hold_max_s: float = DEFAULT_HOLD_MAX_S,
                 state_path: Optional[str] = None,
                 persist: Optional[bool] = None) -> None:
        self.robot_id = robot_id
        self.task_noun = task_noun
        self.task_plural = task_plural
        self.conditions = list(conditions)
        self.rules = dict(rules or {})
        self.legs = list(legs)
        self._log_fn = log
        self._on_pause = on_pause
        self._on_notify = on_notify
        self.ttl_s = float(ttl_s)
        self.constraint_ttl_s = float(constraint_ttl_s)
        self.hold_max_s = float(hold_max_s)

        self._lock = threading.RLock()
        self._seq = 0
        self._pending: List[_Rec] = []
        self._done: List[_Rec] = []        # fired / expired / cleared, capped
        self._constraints: List[_Rec] = []
        self._notes: List[_Rec] = []       # fired notifications, capped

        # DURABILITY. Measured failure: the operator says "after two more
        # deliveries stop and notify me, and never go near TROLLEY_E", the
        # robot answers "I've scheduled a pause and set a standing
        # restriction" -- and a world reload drops BOTH, silently. The
        # conversation memory (which lives on the platform) still recalls the
        # sentence, so the robot can recite a safety rule it is no longer
        # obeying. That asymmetry is the bug: the layer that remembers the
        # PROMISE outlived the layer that ENFORCES it.
        self._persist = (_env_flag("OMNILINK_INTENT_PERSIST", True)
                         if persist is None else bool(persist))
        self._state_path = state_path or _default_state_path(robot_id)
        self._last_saved: Optional[str] = None
        # NB: _restore() runs at the END of __init__, not here -- it rebases
        # onto self.counters and can set self.hold, both of which are
        # initialised further down.

        # Monotonic counts of completed work, fed by sync_tasks().
        #   "tasks"      -- ANY completed job. What after_current_task waits
        #                   for, so a robot that is doing real work but not
        #                   the specific KIND of work still reaches a
        #                   boundary. (Measured: the dispatch tug with a full
        #                   park row only does collections, so counting parks
        #                   alone starved "after you place this cart" for the
        #                   whole 210 s budget.)
        #   "deliveries" -- the delivery-specific tally after_n_deliveries
        #                   counts, so "one more DELIVERY" still means a
        #                   delivery and not any old job.
        self.counters = {"tasks": 0, "deliveries": 0}
        self.leg = ""

        # MEASURED DURATIONS -- the only honest basis for "how much longer?".
        # Nothing here is configured or guessed: the store already sees every
        # leg transition (note_leg) and every completed job (sync_tasks), so
        # timing them costs one subtraction and turns "I can't estimate that"
        # into a number with a stated basis. Empty until the robot has
        # actually done the thing once; progress() says so rather than
        # inventing a figure.
        self._leg_t0 = _now()
        self._leg_hist: Dict[str, List[float]] = {}
        self._task_t0 = _now()
        self._task_hist: List[float] = []

        # THE HOLD.
        self.hold = False
        self.hold_since = 0.0
        self.hold_until = 0.0     # bounded ceiling; not an auto-resume window
        self.hold_words = ""
        self.hold_intent = ""
        # The operator's verbatim text for the turn in flight (see hold_now).
        self._turn_text = ""

        # LAST, on purpose: restoring rebases onto self.counters and may set
        # self.hold, so it has to run after both exist or a restored hold
        # would be overwritten with False and the robot would quietly resume.
        if self._persist:
            self._restore()

    def set_turn_text(self, text: str) -> None:
        """Hand the store the operator's OWN words for this chat turn.

        The bridge sets it before routing a prompt and clears it after. It is
        the ground truth a tool argument cannot forge."""
        with self._lock:
            self._turn_text = str(text or "")

    @property
    def tasks(self) -> int:
        """Completed jobs of ANY kind. Kept as an attribute-shaped view so
        every existing log line and read keeps working."""
        return self.counters["tasks"]

    @tasks.setter
    def tasks(self, v: int) -> None:
        self.counters["tasks"] = int(v)

    # ── logging ──────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        line = f"[intents] {msg}"
        if self._log_fn is not None:
            try:
                self._log_fn(line)
                return
            except Exception:
                pass
        print(line, flush=True)

    def _next_id(self, prefix: str) -> str:
        self._seq += 1
        return f"{prefix}{self._seq}"

    # ── condition parsing / refusals ─────────────────────────────────

    def _supported_conditions(self) -> List[str]:
        out = []
        for c in self.conditions:
            canon = _ALIASES.get(c, c)
            if canon == "after_n_tasks":
                out.append(f"{c}:N")
            elif canon == "at_leg":
                out.append("at_leg:<leg>")
            else:
                out.append(c)
        return out

    def _refuse(self, reason: str, alt: str) -> dict:
        """A refusal the model can RELAY. No silent accepts."""
        return {
            "accepted": False,
            "reason": reason,
            "supported_conditions": self._supported_conditions(),
            "supported_constraints": sorted(self.rules),
            # The model is instructed to say this verbatim. It is the whole
            # difference between "sure, I'll do that" (a lie) and an honest
            # counter-offer.
            "say": alt,
        }

    def _parse_condition(self, condition: Any, count: Any,
                         leg: Any) -> Any:
        raw = str(condition or "").strip()
        if not raw:
            return self._refuse(
                "no condition given",
                "I need to know WHEN — I can stop after the current "
                f"{self.task_noun}, after N {self.task_plural}, on my next "
                "pickup, or when I reach a named leg.")
        head, _, tail = raw.partition(":")
        head = head.strip().lower().replace(" ", "_").replace("-", "_")
        tail = tail.strip()
        canon = _ALIASES.get(head)
        advertised = {_ALIASES.get(str(c).strip().lower(),
                                   str(c).strip().lower())
                      for c in self.conditions}
        if canon is None or canon not in advertised:
            return self._refuse(
                f"unsupported condition {raw!r}",
                f"I can't schedule that — I can stop after the current "
                f"{self.task_noun} instead, or after a set number of "
                f"{self.task_plural}.")
        if canon == "after_n_tasks":
            n = count
            if n is None and tail:
                n = tail
            # which named counter this trigger watches (see self.counters)
            which = "deliveries" if head == "after_n_deliveries" else "tasks"
            try:
                n = int(float(n))
            except (TypeError, ValueError):
                return self._refuse(
                    f"condition {raw!r} needs a count",
                    f"How many {self.task_plural} should I finish first?")
            if n < 1 or n > 100:
                return self._refuse(
                    f"count {n} out of range (1..100)",
                    f"I can wait for between 1 and 100 more "
                    f"{self.task_plural}.")
            return (canon, n, "", which)
        if canon == "at_leg":
            lg = str(leg or tail or "").strip()
            if not lg:
                return self._refuse(
                    "at_leg needs a leg name",
                    "Which leg? I know: " + (", ".join(self.legs) or "none"))
            if self.legs and lg not in self.legs:
                return self._refuse(
                    f"unknown leg {lg!r}",
                    "I don't have a leg called that. I know: "
                    + (", ".join(self.legs) or "none")
                    + f" — or I can stop after the current {self.task_noun}.")
            return (canon, 0, lg, "tasks")
        return (canon, 0, "", "tasks")

    # ── scheduling API (called by tools) ─────────────────────────────

    def schedule(self, kind: str, condition: Any, *,
                 count: Any = None, leg: Any = None,
                 until_told: bool = True, message: str = "",
                 words: str = "", ttl_s: Optional[float] = None) -> dict:
        """Record a deferred (trigger -> action) intent, or refuse it."""
        parsed = self._parse_condition(condition, count, leg)
        if isinstance(parsed, dict):
            return parsed
        canon, n, lg, which = parsed
        kind = "notify" if str(kind).lower().startswith("notif") else "pause"
        with self._lock:
            self._tick_locked()
            now = _now()
            # DEDUPE. Small models re-emit the same tool call several times in
            # one turn (measured: 2x stop_after_current_task, 7x
            # set_constraint from a 7B local model), and two identical
            # pending pauses would both fire and read as two separate
            # promises. Re-arming an identical intent REFRESHES it.
            for old in self._pending:
                t = old["trigger"]
                if (old["kind"] == kind and t["type"] == canon
                        and t["count"] == n and t["leg"] == lg
                        and old.get("counter", "tasks") == which):
                    old["expires_at"] = round(
                        now + float(ttl_s or self.ttl_s), 3)
                    if words:
                        old["words"] = str(words)
                    out = dict(old)
                    out["accepted"] = True
                    out["already_pending"] = True
                    out["say"] = self._commitment(old)
                    return out
            rec = _Rec({
                "id": self._next_id("intent-"),
                "kind": kind,
                "trigger": {"type": canon, "count": n, "leg": lg,
                            "raw": str(condition)},
                "action": ({"type": "pause",
                            "until_told": bool(until_told)}
                           if kind == "pause" else
                           {"type": "notify", "message": str(message or ""),
                            "pause_first": bool(until_told)}),
                "status": "pending",
                "created_at": round(now, 3),
                "expires_at": round(now + float(ttl_s or self.ttl_s), 3),
                # The operator's OWN words, kept verbatim. When the model
                # later reads pending_intents back it quotes the order it was
                # actually given rather than its own paraphrase of it.
                "words": str(words or ""),
                "counter": which,
                "base_tasks": self.counters.get(which, 0),
                # ARMED = the trigger has been met. FIRED = the action has
                # actually been taken. They are separate so a trigger met at
                # an unsafe moment (mid-tow) is deferred, never lost.
                "armed": False,
                "armed_at": 0.0,
                "fired_at": 0.0,
                "detail": "",
            })
            self._pending.append(rec)
            self._flush_locked()
            desc = self._describe(rec)
            self._log(f"{rec['id']} ARMED: {desc}"
                      + (f' — operator said: "{rec["words"]}"'
                         if rec["words"] else ""))
            out = dict(rec)
        out["accepted"] = True
        out["say"] = self._commitment(rec)
        return out

    # A hold is only honest when the operator actually named a condition for
    # coming back. Measured: a 7B model routed a BARE "stop" to
    # hold_until_told 4 times out of 4 -- the robot stopped (fine) but also
    # silently lost its auto-resume (not fine, and not what "stop" means).
    # Descriptions did not fix it; verifying the operator's own words does.
    _HOLD_MARKERS = (
        "until", "til", "till", "unless", "when i", "when you hear",
        "say so", "my signal", "the signal", "i come back", "i get back",
        "i tell", "i say", "let me know", "wait for me", "hold there",
        "stay there", "stay put", "don't move", "dont move", "do not move",
        "further notice", "for now",
    )

    def hold_now(self, words: str = "", ttl_s: Optional[float] = None,
                 force: bool = False) -> dict:
        """Pause NOW and do NOT auto-resume. The fix for "until I tell you".

        REFUSES when the operator's own words contain no condition for
        resuming -- a plain "stop" is stop_robot, and quietly turning it into
        an indefinite hold is the same class of lie this module exists to
        stop: the operator gets behaviour they did not ask for and is not
        told."""
        # PREFER THE OPERATOR'S ACTUAL TURN TEXT over the model-supplied
        # `words`. Checking `words` alone did not work: told to hold on a
        # bare "stop", the model simply invented words with an "until" in
        # them and the guard passed. What the operator typed is the only
        # thing it cannot rewrite -- the bridge hands it to the store for the
        # duration of the turn (set_turn_text).
        text = (self._turn_text or str(words or "")).strip().lower()
        if not force:
            if not text:
                return {
                    "accepted": False,
                    "reason": ("hold_until_told needs the operator's own "
                               "words, to check they asked to be waited for"),
                    "say": ("Stopping now. Tell me when to carry on."),
                    "use_instead": "stop_robot",
                }
            if not any(m in text for m in self._HOLD_MARKERS):
                self._log(f"refused hold_until_told for {words!r} — no "
                          "resume condition in the operator's words")
                return {
                    "accepted": False,
                    "reason": (f"{words!r} does not say how long to hold; a "
                               "plain stop is an ordinary halt that returns "
                               "to work on its own"),
                    "say": ("Stopped. I'll pick my job back up shortly unless "
                            "you tell me to stay put."),
                    "use_instead": "stop_robot",
                }
        with self._lock:
            self._engage_hold(words=words, intent_id="operator",
                              ttl_s=ttl_s)
            st = self._hold_state_locked()
            self._flush_locked()
        if self._on_pause is not None:
            try:
                self._on_pause(True, {"id": "operator", "words": words})
            except Exception as e:  # pragma: no cover - hook is bridge code
                self._log(f"on_pause hook failed: {e!r}")
        st["accepted"] = True
        st["say"] = ("Holding here. I will not go back to work on my own — "
                     "tell me to carry on and I will resume.")
        return st

    def release_hold(self, reason: str = "resume_autonomy") -> dict:
        """Clear the hold. Only an explicit resume does this."""
        with self._lock:
            was = self.hold
            if was:
                held = _now() - self.hold_since
                self._log(f"hold RELEASED by {reason} after {held:.0f}s")
            self.hold = False
            self.hold_since = 0.0
            self.hold_until = 0.0
            self.hold_words = ""
            self.hold_intent = ""
            self._flush_locked()
        return {"released": bool(was)}

    def set_constraint(self, rule: Any, words: str = "",
                       ttl_s: Optional[float] = None) -> dict:
        """Record a standing restriction the autonomy loop checks BEFORE
        acting. Refuses any rule this robot does not genuinely enforce."""
        key = str(rule or "").strip().lower().replace(" ", "_").replace("-", "_")
        if key not in self.rules:
            return self._refuse(
                f"unsupported constraint {rule!r}",
                "I can't enforce that rule. I can enforce: "
                + (", ".join(sorted(self.rules)) or "nothing")
                + f" — or I can simply stop after the current {self.task_noun}.")
        with self._lock:
            self._tick_locked()
            for c in self._constraints:
                if c["rule"] == key and c["status"] == "active":
                    c["expires_at"] = round(
                        _now() + float(ttl_s or self.constraint_ttl_s), 3)
                    out = dict(c)
                    out["accepted"] = True
                    out["already_active"] = True
                    out["say"] = self._constraint_commitment(c)
                    return out
            now = _now()
            rec = _Rec({
                "id": self._next_id("rule-"),
                "rule": key,
                "means": self.rules[key],
                "status": "active",
                "created_at": round(now, 3),
                "expires_at": round(
                    now + float(ttl_s or self.constraint_ttl_s), 3),
                "words": str(words or ""),
                "blocked": 0,
                "last_block": "",
            })
            self._constraints.append(rec)
            self._flush_locked()
            self._log(f"{rec['id']} CONSTRAINT SET: {key} ({rec['means']})"
                      + (f' — operator said: "{rec["words"]}"'
                         if rec["words"] else ""))
            out = dict(rec)
        out["accepted"] = True
        out["say"] = self._constraint_commitment(rec)
        return out

    # A rule cannot be lifted this soon after being set. Measured: a small
    # model answered "don't start any more picks until I say so" with
    # set_constraint IMMEDIATELY followed by clear_constraint and replied
    # "restriction lifted" -- a fabricated agreement with extra steps.
    CLEAR_GRACE_S = 15.0

    def clear_constraint(self, cid: Any = None) -> dict:
        with self._lock:
            now = _now()
            fresh = [c for c in self._constraints
                     if c["status"] == "active"
                     and now - c["created_at"] < self.CLEAR_GRACE_S
                     and (cid in (None, "", "all") or c["id"] == cid
                          or c["rule"] == cid)]
            if fresh:
                self._log(f"refused to clear {fresh[0]['id']} "
                          f"({fresh[0]['rule']}) — set "
                          f"{now - fresh[0]['created_at']:.0f}s ago")
                return {
                    "accepted": False, "cleared": [],
                    "reason": ("that restriction was set seconds ago; it "
                               "cannot be lifted in the same breath"),
                    "say": ("That restriction is still in force — I only "
                            "lift it when you tell me to, not in the same "
                            "answer where you set it."),
                }
            hits = []
            for c in self._constraints:
                if c["status"] != "active":
                    continue
                if cid in (None, "", "all") or c["id"] == cid or c["rule"] == cid:
                    c["status"] = "cleared"
                    hits.append(dict(c))
                    self._log(f"{c['id']} CONSTRAINT CLEARED ({c['rule']}) "
                              f"after {c['blocked']} block(s)")
            self._constraints = [c for c in self._constraints
                                 if c["status"] == "active"]
        if not hits:
            return {"accepted": False, "cleared": [],
                    "reason": "no active constraint matched",
                    "say": "There was no standing restriction to lift."}
        self._flush_locked()
        return {"accepted": True, "cleared": hits,
                "say": "Restriction lifted — back to normal work."}

    def cancel(self, iid: Any = None) -> dict:
        """Drop a pending intent, and SAY WHICH ONE.

        Measured failure: the operator asked to cancel something that had
        never been created (a notify_when the store had already refused). The
        model guessed an id, that id happened to belong to a LIVE "after two
        more deliveries, notify me" promise, the store cancelled it, and the
        whole reply was the word "Cancelled." The operator believed a phantom
        had been dropped and had in fact silently lost a real commitment.

        The store cannot know whether the id matches what the operator
        described -- only the operator can. So the answer is to make the
        cancellation impossible to miss: name what was dropped, in the
        operator's own words, so a wrong guess is corrected in the next
        breath instead of discovered when the robot fails to stop.
        """
        with self._lock:
            self._tick_locked()
            wanted = None if iid in (None, "", "all") else str(iid)

            # A bare "cancel" with several promises outstanding used to drop
            # ALL of them. That is never what "cancel that one" means, and the
            # blast radius is every commitment the operator is relying on.
            if wanted is None and len(self._pending) > 1:
                listing = "; ".join(
                    f"{it['id']} ({self._describe(it)})" for it in self._pending)
                return {
                    "accepted": False, "cancelled": [],
                    "pending": [it["id"] for it in self._pending],
                    "say": ("I have more than one thing scheduled, so I don't "
                            f"want to guess which to drop. I'm holding: {listing}. "
                            "Which one should I cancel — or should I cancel all?"),
                }

            if wanted is not None and not any(
                    it["id"] == wanted for it in self._pending):
                have = ", ".join(it["id"] for it in self._pending) or "nothing"
                return {
                    "accepted": False, "cancelled": [],
                    "pending": [it["id"] for it in self._pending],
                    "say": (f"I have no pending {wanted}. I'm currently holding: "
                            f"{have}. Tell me which of those you mean and I'll "
                            "drop it."),
                }

            hits = []
            for it in self._pending:
                if wanted is None or it["id"] == wanted:
                    it["status"] = "cancelled"
                    hits.append(dict(it))
                    self._log(f"{it['id']} CANCELLED")
            self._pending = [i for i in self._pending
                             if i["status"] == "pending"]
            self._done = (self._done + hits)[-40:]
            self._flush_locked()

        if not hits:
            return {"accepted": False, "cancelled": [],
                    "say": "I had nothing scheduled to cancel."}

        # Quote the operator back to themselves where we have their words --
        # "cancelled: stop after two more deliveries" is checkable in a way
        # that "Cancelled." is not.
        described = []
        for h in hits:
            words = (h.get("words") or "").strip()
            described.append(f"{self._describe(h)}"
                             + (f' (you said: "{words}")' if words else ""))
        joined = "; ".join(described)
        return {"accepted": True, "cancelled": hits,
                "say": f"Cancelled — I will no longer {joined}."}

    # ── phrasing ─────────────────────────────────────────────────────

    def _describe(self, rec: dict) -> str:
        t = rec["trigger"]
        if t["type"] == "after_current_task":
            when = f"after the current {self.task_noun}"
        elif t["type"] == "after_n_tasks":
            when = (f"after {t['count']} more "
                    f"{self.task_noun if t['count'] == 1 else self.task_plural}")
        elif t["type"] == "on_next_pickup":
            when = "on my next pickup"
        else:
            when = f"when I reach leg {t['leg']!r}"
        if rec["kind"] == "pause":
            act = ("pause and HOLD (no auto-resume)"
                   if rec["action"]["until_told"] else "pause")
        else:
            act = "notify the operator" + (
                " and hold" if rec["action"].get("pause_first") else "")
        return f"{when} -> {act}"

    def _commitment(self, rec: dict) -> str:
        """What the model should SAY. It is now backed by a real record."""
        t = rec["trigger"]
        if t["type"] == "after_current_task":
            when = f"as soon as I finish this {self.task_noun}"
        elif t["type"] == "after_n_tasks":
            when = (f"after {t['count']} more "
                    f"{self.task_noun if t['count'] == 1 else self.task_plural}")
        elif t["type"] == "on_next_pickup":
            when = "the next time I pick something up"
        else:
            when = f"when I get to the {t['leg']} leg"
        if rec["kind"] == "notify":
            return (f"Noted — {when} I will tell you before I go on. "
                    "It is on my list until then.")
        if rec["action"]["until_told"]:
            return (f"Understood — I will keep working, then {when} I will "
                    "stop and stay stopped until you tell me to carry on.")
        return f"Understood — I will stop {when}."

    def _constraint_commitment(self, rec: dict) -> str:
        return (f"Understood — {rec['means']} I will keep doing my other "
                "work and stay out until you lift it.")

    # ── the HOLD ─────────────────────────────────────────────────────

    def _engage_hold(self, words: str, intent_id: str,
                     ttl_s: Optional[float] = None) -> None:
        now = _now()
        if not self.hold:
            self._log(f"HOLD engaged by {intent_id} — autonomy will NOT "
                      f"auto-resume (ceiling "
                      f"{float(ttl_s or self.hold_max_s):.0f}s)")
        self.hold = True
        self.hold_since = now
        self.hold_until = now + float(ttl_s or self.hold_max_s)
        self.hold_words = str(words or "")
        self.hold_intent = intent_id

    def hold_active(self) -> bool:
        """True while a "do not auto-resume" hold is in force.

        Bounded: past the ceiling it lapses and LOGS, so a forgotten hold
        cannot wedge the line forever."""
        with self._lock:
            if self.hold and _now() >= self.hold_until:
                self._log(f"HOLD EXPIRED after "
                          f"{self.hold_until - self.hold_since:.0f}s ceiling "
                          "— resuming autonomy (operator never sent a resume)")
                self.hold = False
                self.hold_since = 0.0
                self.hold_until = 0.0
                self.hold_words = ""
                self.hold_intent = ""
            return self.hold

    def _hold_state_locked(self) -> dict:
        if not self.hold:
            return {"active": False}
        now = _now()
        return {"active": True,
                "since": round(self.hold_since, 3),
                "held_s": round(now - self.hold_since, 1),
                "ceiling_in_s": round(max(0.0, self.hold_until - now), 1),
                "by": self.hold_intent,
                "words": self.hold_words,
                "clears_on": "resume_autonomy"}

    # ── constraint queries (called by the autonomy loop BEFORE acting) ─

    def constraint(self, rule: str) -> Optional[dict]:
        """The active constraint for ``rule``, or None. Cheap; call freely."""
        with self._lock:
            self._tick_locked()
            for c in self._constraints:
                if c["rule"] == rule and c["status"] == "active":
                    return dict(c)
        return None

    def note_block(self, rule: str, detail: str) -> None:
        """Record that a constraint actually declined an action. This is the
        evidence that the rule is being CHECKED, not just stored."""
        with self._lock:
            for c in self._constraints:
                if c["rule"] == rule and c["status"] == "active":
                    c["blocked"] += 1
                    c["last_block"] = str(detail)[:160]
                    n = c["blocked"]
                    break
            else:
                return
        # Throttle: a blocked leg is retried every cycle pass.
        last = getattr(self, "_block_log_t", {})
        now = _now()
        if now - last.get(rule, 0.0) > 20.0:
            last[rule] = now
            self._block_log_t = last
            self._log(f"constraint {rule} DECLINED: {detail} (block #{n})")

    # ── trigger evaluation (called by the autonomy loop) ─────────────

    def sync_tasks(self, count: Optional[int] = None, detail: str = "",
                   safe: bool = True,
                   deliveries: Optional[int] = None) -> List[dict]:
        """AUTHORITATIVE completed-work counter push. THE key hook.

        ``count`` is the robot's own MONOTONIC counter -- carts parked
        (parks_total) or parts placed (picks). The autonomy loop pushes it on
        every poll, so a trigger can no longer be missed because one
        particular call site was skipped.

        THIS REPLACED A HAND-PLACED BOUNDARY HOOK, AND THAT HOOK WAS BROKEN.
        The first cut only signalled a boundary from the tail of the dispatch
        cycle, after _park_at_staging() returned True. Two ways that lost the
        event outright:
          * the cart is parked (parks_total++) but the tug then does a whole
            COLLECT trip before reaching the tail, so "after you place this
            cart" was still pending 150 s later; and
          * if _park_at_staging() returned False (obstacle hold, leg
            timeout) the cycle returned early and the boundary was never
            signalled AT ALL -- the intent sat pending until it expired.
        Reproduced on the cloud engine with the intent created while the tug
        was IDLE: delivered_total reached 1, completed_tasks stayed 0, the
        intent never fired. Deriving the count from the same monotonic
        counter /state already publishes makes the two agree by
        construction.

        ``safe`` says whether the robot could stop RIGHT NOW without
        stranding anything (not towing, not holding a shared claim). When it
        is False a satisfied trigger is ARMED rather than fired, and fires at
        the first safe poll -- the boundary is recorded either way. That is
        the difference between "do not pause mid-tow" and "throw the event
        away because the robot happened to be mid-tow", which is what the
        first carrying-guard did."""
        with self._lock:
            self._tick_locked()
            changed = False
            for name, v in (("tasks", count), ("deliveries", deliveries)):
                if v is None:
                    continue
                if int(v) > self.counters.get(name, 0):
                    self.counters[name] = int(v)
                    changed = True
            if count is None and deliveries is None:
                self.counters["tasks"] += 1
                changed = True
            if changed:
                # One completed job = one measured sample. Keep the last few
                # only: a line that speeds up should not be estimated from
                # the shape it had ten minutes ago.
                now_t = _now()
                self._task_hist = (self._task_hist
                                   + [now_t - self._task_t0])[-8:]
                self._task_t0 = now_t
                self._match_locked(
                    "task", detail or ("count=%d" % self.counters["tasks"]))
            fired = self._fire_armed_locked(safe)
        self._deliver(fired)
        return fired

    # Back-compat name, for callers that KNOW they are at a clean boundary.
    def note_task(self, count: Optional[int] = None,
                  detail: str = "") -> List[dict]:
        return self.sync_tasks(count, detail, safe=True)

    def note_event(self, event: str, detail: str = "",
                   safe: bool = True) -> List[dict]:
        """A named event happened ("pickup").

        ``safe`` defaults True on purpose: "tell me before you move it" is
        meant to hold the robot the moment it picks something up, which is
        by definition a not-empty-handed moment."""
        with self._lock:
            self._tick_locked()
            self._match_locked(event, detail)
            fired = self._fire_armed_locked(safe)
        self._deliver(fired)
        return fired

    def note_leg(self, leg: str, detail: str = "",
                 safe: bool = True) -> List[dict]:
        leg = str(leg or "")
        with self._lock:
            if leg == self.leg:
                return []
            now_t = _now()
            if self.leg:
                self._leg_hist.setdefault(self.leg, [])
                self._leg_hist[self.leg] = (
                    self._leg_hist[self.leg] + [now_t - self._leg_t0])[-8:]
            self._leg_t0 = now_t
            self.leg = leg
            self._tick_locked()
            self._match_locked("leg", detail or leg)
            fired = self._fire_armed_locked(safe)
        self._deliver(fired)
        return fired

    def tick(self) -> None:
        """Age out expired intents/constraints. Safe to call in a poll."""
        with self._lock:
            self._tick_locked()

    def _tick_locked(self) -> None:
        now = _now()
        keep = []
        for it in self._pending:
            if now >= it["expires_at"]:
                it["status"] = "expired"
                it["detail"] = "trigger never fired"
                # BOUNDED BY DESIGN. An intent that can never be satisfied
                # ages out loudly instead of sitting in the store forever
                # pretending the robot is still waiting for it.
                self._log(f"{it['id']} EXPIRED after "
                          f"{it['expires_at'] - it['created_at']:.0f}s "
                          f"({self._describe(it)}) — trigger never fired"
                          + (f'; operator said: "{it["words"]}"'
                             if it["words"] else ""))
                self._done.append(it)
            else:
                keep.append(it)
        self._pending = keep
        ckeep = []
        for c in self._constraints:
            if now >= c["expires_at"]:
                c["status"] = "expired"
                self._log(f"{c['id']} CONSTRAINT EXPIRED after "
                          f"{c['expires_at'] - c['created_at']:.0f}s "
                          f"({c['rule']}, {c['blocked']} block(s)) — normal "
                          "work resumes")
            else:
                ckeep.append(c)
        self._constraints = ckeep
        self._done = self._done[-40:]
        # Every public mutator funnels through _tick_locked, so flushing here
        # captures ALL of them -- including any added later, which a
        # per-method save call would eventually forget.
        self._flush_locked()

    # ---- durability ------------------------------------------------- #

    def _snapshot_locked(self) -> Dict[str, Any]:
        return {
            "version": STATE_VERSION,
            "robot_id": self.robot_id,
            "saved_at": round(_now(), 3),
            "seq": self._seq,
            "pending": [dict(r) for r in self._pending],
            "constraints": [dict(r) for r in self._constraints],
            "hold": bool(self.hold),
            "hold_since": self.hold_since,
            "hold_until": self.hold_until,
        }

    def _flush_locked(self) -> None:
        """Write state if it changed. Never raises -- a read-only disk must
        degrade to the old in-memory behaviour, not break the robot."""
        if not self._persist:
            return
        try:
            snap = self._snapshot_locked()
            # Compare without the timestamp so an idle tick is not a write.
            cmp = dict(snap)
            cmp.pop("saved_at", None)
            blob = json.dumps(cmp, sort_keys=True, default=str)
            if blob == self._last_saved:
                return
            os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
            tmp = f"{self._state_path}.{os.getpid()}.tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(snap, fh, default=str)
            os.replace(tmp, self._state_path)   # atomic; never a torn file
            self._last_saved = blob
        except Exception as exc:
            self._persist = False               # complain once, then stop
            self._log(f"intent persistence disabled ({exc})")

    def _restore(self) -> None:
        """Reload commitments from a previous run of this robot.

        Two deliberate adjustments, because a reload is not a time machine:

        * COUNTERS RESTART AT ZERO, so "after two more deliveries" is rebased
          onto the new count. Keeping the old baseline would fire the intent
          instantly (the counter can never reach an already-passed number) or
          never -- both silently wrong. Rebasing keeps the operator's actual
          meaning: two more from here.
        * TTLs ARE REFRESHED. An order should not die because the world was
          reloaded inside its window; the clock starts again with the robot.
        """
        try:
            with open(self._state_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return
        except Exception as exc:
            self._log(f"could not read intent state ({exc}); starting clean")
            return

        if not isinstance(data, dict) or data.get("version") != STATE_VERSION:
            self._log("intent state version mismatch; starting clean")
            return
        if data.get("robot_id") != self.robot_id:
            return

        now = _now()
        try:
            age = now - float(data.get("saved_at") or 0.0)
        except (TypeError, ValueError):
            age = float("inf")
        if age > STALE_STATE_S:
            self._log(f"ignoring intent state from {age / 60.0:.0f} min ago "
                      f"(older than the {STALE_STATE_S / 60.0:.0f} min reload "
                      "window) — starting clean")
            try:
                os.remove(self._state_path)
            except Exception:
                pass
            return

        restored_i = restored_c = 0
        try:
            for raw in data.get("pending") or []:
                if not isinstance(raw, dict) or raw.get("status") != "pending":
                    continue
                rec = _Rec(dict(raw))
                rec["base_tasks"] = self.counters.get(
                    rec.get("counter", "tasks"), 0)
                rec["expires_at"] = round(now + self.ttl_s, 3)
                rec["restored"] = True
                self._pending.append(rec)
                restored_i += 1
            for raw in data.get("constraints") or []:
                if not isinstance(raw, dict) or raw.get("status") == "expired":
                    continue
                rec = _Rec(dict(raw))
                rec["expires_at"] = round(now + self.constraint_ttl_s, 3)
                rec["restored"] = True
                self._constraints.append(rec)
                restored_c += 1
            self._seq = max(int(data.get("seq") or 0), self._seq)
            # An autonomy hold is restored too: "stop until I tell you" must
            # not be undone by a reload, which is exactly when a robot
            # quietly going back to work would be least expected.
            if bool(data.get("hold")):
                self.hold = True
                self.hold_since = now
                self.hold_until = now + self.hold_max_s
        except Exception as exc:
            self._log(f"intent state partially restored ({exc})")

        if restored_i or restored_c or self.hold:
            note = (f"restored {restored_i} pending intent(s), {restored_c} "
                    f"constraint(s)"
                    f"{', autonomy hold' if self.hold else ''} "
                    f"from the previous run ({age:.0f}s ago)")
            self._log(note)
            # ALSO surface it in the state payload. _restore() runs inside
            # __init__, before the bridge has attached its log sink, so the
            # line above never reaches OMNILINK_IDLE_LOG -- an operator could
            # not see that a reload had rehydrated commitments. This field
            # shows up in /intents and /state, which they do read.
            self.restored_note = note

    def _matches(self, it: dict, kind: str, detail: str) -> bool:
        t = it["trigger"]
        ty = t["type"]
        if ty == "after_current_task":
            return kind == "task"
        if ty == "after_n_tasks":
            got = self.counters.get(it.get("counter", "tasks"), 0)
            return kind == "task" and got >= it["base_tasks"] + t["count"]
        if ty == "on_next_pickup":
            return kind == "pickup"
        if ty == "at_leg":
            return kind == "leg" and detail == t["leg"]
        return False

    def _match_locked(self, kind: str, detail: str) -> None:
        """Mark every pending intent whose trigger is now satisfied as ARMED.

        Arming is STICKY, and separate from firing. That the work the
        operator referred to has happened is recorded the moment it happens;
        whether the robot can safely stop is a different question, answered
        by _fire_armed_locked."""
        for it in self._pending:
            if it.get("armed"):
                continue
            if self._matches(it, kind, detail):
                it["armed"] = True
                it["armed_at"] = round(_now(), 3)
                it["detail"] = f"{kind}:{detail}" if detail else kind
                self._log(f"{it['id']} TRIGGER MET ({self._describe(it)}; "
                          f"trigger={it['detail']}, tasks={self.tasks})")

    def _fire_armed_locked(self, safe: bool) -> List[dict]:
        """Fire every armed intent, or defer them all until it is safe."""
        armed = [it for it in self._pending if it.get("armed")]
        if not armed:
            return []
        if not safe:
            # Throttled: this is polled, and a tow leg runs for minutes.
            if _now() - getattr(self, "_defer_log_t", 0.0) > 20.0:
                self._defer_log_t = _now()
                self._log(
                    "%d intent(s) waiting for a safe moment to stop -- the "
                    "robot is still holding something or on a shared claim; "
                    "the trigger is RECORDED and fires as soon as it is "
                    "clear" % len(armed))
            return []
        fired: List[dict] = []
        for it in armed:
            it["status"] = "fired"
            it["fired_at"] = round(_now(), 3)
            self._log(f"{it['id']} FIRED ({self._describe(it)}; "
                      f"trigger={it['detail']}, tasks={self.tasks})")
            self._done.append(it)
            fired.append(dict(it))
            if it["kind"] == "pause" and it["action"]["until_told"]:
                self._engage_hold(it["words"], it["id"])
            elif (it["kind"] == "notify"
                  and it["action"].get("pause_first")):
                self._engage_hold(it["words"], it["id"])
        self._pending = [it for it in self._pending if not it.get("armed")]
        return fired

    def _deliver(self, fired: List[dict]) -> None:
        """Run the bridge-side hooks OUTSIDE the lock (they stop motors)."""
        for it in fired:
            if it["kind"] == "pause":
                if self._on_pause is not None:
                    try:
                        self._on_pause(bool(it["action"]["until_told"]), it)
                    except Exception as e:
                        self._log(f"on_pause hook failed: {e!r}")
            else:
                text = (it["action"].get("message")
                        or (f'{it["words"]}' if it["words"] else "")
                        or "the condition you asked about just happened")
                # WHAT WAS PROMISED vs WHAT ACTUALLY HAPPENED.
                #
                # `message` is a sentence the MODEL wrote when the order was
                # accepted, minutes before anything occurred, and it is fired
                # back verbatim. So it can assert an outcome nothing verified:
                # measured, a notify armed as "I have staged a fresh cart for
                # you" fired on a trigger whose real event was a LOADED cart
                # going out to the pickup. Nothing had been staged. The
                # sentence was not a lie when written -- it was a forecast,
                # delivered later as if it were an observation.
                #
                # The note now carries the measured event alongside it, so a
                # relaying model has the truth available and the two can never
                # be confused: `promised_text` is what was undertaken,
                # `fired_because` is what the robot actually did.
                note = _Rec({"id": it["id"], "at": round(_now(), 3),
                             "sim_note": text,
                             "promised_text": text,
                             "fired_because": it.get("detail", "") or "trigger met",
                             "counter_at_fire": self.counters.get(
                                 it.get("counter", "tasks"), 0),
                             "verified": False,
                             "trigger": it["trigger"]["type"],
                             "words": it["words"]})
                with self._lock:
                    self._notes.append(note)
                    self._notes = self._notes[-20:]
                self._log(f"{it['id']} NOTIFY: {text}")
                if it["action"].get("pause_first") and self._on_pause is not None:
                    try:
                        self._on_pause(True, it)
                    except Exception as e:
                        self._log(f"on_pause hook failed: {e!r}")
                if self._on_notify is not None:
                    try:
                        self._on_notify(text, it)
                    except Exception as e:
                        self._log(f"on_notify hook failed: {e!r}")

    def ack_notes(self) -> List[dict]:
        """Read AND clear delivered notifications (the model has relayed
        them). Kept separate from ``state`` so a status poll does not
        silently swallow a notification nobody has seen."""
        with self._lock:
            out = [dict(n) for n in self._notes]
            self._notes = []
        return out

    # ── read-out ─────────────────────────────────────────────────────

    def listing(self) -> dict:
        """What list_pending_intents returns -- and what /state embeds."""
        with self._lock:
            self._tick_locked()
            now = _now()
            pend = []
            for it in self._pending:
                r = dict(it)
                r["expires_in_s"] = round(max(0.0, it["expires_at"] - now), 1)
                r["means"] = self._describe(it)
                # An ARMED intent has had its trigger met and is only waiting
                # for a moment the robot can safely stop. Saying so is the
                # difference between "still waiting for you to place it" and
                # "placed it, stopping as soon as I am clear".
                if it.get("armed"):
                    r["status"] = "armed"
                    r["means"] += " [trigger MET — stopping as soon as I am "
                    r["means"] += "clear]"
                if it["trigger"]["type"] == "after_n_tasks":
                    got = self.counters.get(it.get("counter", "tasks"), 0)
                    r["remaining"] = max(
                        0, it["base_tasks"] + it["trigger"]["count"] - got)
                pend.append(r)
            cons = []
            for c in self._constraints:
                r = dict(c)
                r["expires_in_s"] = round(max(0.0, c["expires_at"] - now), 1)
                cons.append(r)
            out = {
                "pending_intents": pend,
                "constraints": cons,
                # Present only when a reload rehydrated commitments, so the
                # operator (and the model) can see WHY the robot came back up
                # already holding an order nobody gave it this session.
                **({"restored_from_previous_run": self.restored_note}
                   if getattr(self, "restored_note", "") else {}),
                "autonomy_hold": self._hold_state_locked(),
                "notifications": [dict(n) for n in self._notes],
                "completed_tasks": self.counters["tasks"],
                "counters": dict(self.counters),
                "task_noun": self.task_noun,
                "recent": [
                    {"id": r["id"], "status": r["status"],
                     "means": self._describe(r), "detail": r.get("detail", ""),
                     "words": r.get("words", "")}
                    for r in self._done[-6:]
                ],
            }
        # A truthful one-liner the model can read straight back to the
        # operator instead of inventing one.
        out["summary"] = self._summary(out)
        return out

    @staticmethod
    def _summary(listing: dict) -> str:
        bits = []
        for it in listing["pending_intents"]:
            bits.append(f"{it['id']}: {it['means']}")
        for c in listing["constraints"]:
            bits.append(f"{c['id']}: {c['means']}")
        if listing["autonomy_hold"].get("active"):
            bits.append("HELD — waiting for you to tell me to carry on")
        if not bits:
            return "Nothing scheduled and no standing restrictions."
        return "; ".join(bits)

    # ── progress / ETA ───────────────────────────────────────────────
    #
    # "How much longer until you finish this delivery?" used to be answered
    # with a description of the present, because nothing in the tool surface
    # carried a duration -- so the model had the choice of inventing a number
    # or changing the subject, and it changed the subject. The store already
    # timestamps every leg change and every completed job; this turns that
    # into an ESTIMATE WITH A STATED BASIS, and says "I don't know yet" out
    # loud when there is no sample to base it on. It is never a promise.

    @staticmethod
    def _mean(xs: Sequence[float]) -> float:
        xs = list(xs)
        return (sum(xs) / len(xs)) if xs else 0.0

    def progress(self) -> dict:
        """Where this robot is in its work, plus a measured-basis estimate.

        Every duration in here was TIMED on this robot in this session. No
        configured nominal times, no guesses: with zero samples the reply
        says ``known: false`` and hands back a sentence admitting it."""
        with self._lock:
            now = _now()
            leg = self.leg or "idle"
            leg_elapsed = now - self._leg_t0
            leg_samples = list(self._leg_hist.get(leg, ()))
            task_samples = list(self._task_hist)
            task_elapsed = now - self._task_t0
            done = self.counters.get("tasks", 0)
            deliveries = self.counters.get("deliveries", 0)
        out: Dict[str, Any] = {
            "leg": leg,
            "leg_elapsed_s": round(leg_elapsed, 1),
            "task_noun": self.task_noun,
            "completed_tasks": done,
            "completed_deliveries": deliveries,
            "current_task_elapsed_s": round(task_elapsed, 1),
            # Everything below is derived from measured samples. Labelled so
            # the model relays it as an estimate, which is what it is.
            "is_estimate": True,
            "basis": "mean of this robot's own measured times this session",
        }
        if leg_samples:
            typ = self._mean(leg_samples)
            out["leg_typical_s"] = round(typ, 1)
            out["leg_samples"] = len(leg_samples)
            out["leg_remaining_estimate_s"] = round(
                max(0.0, typ - leg_elapsed), 1)
        else:
            out["leg_samples"] = 0
            out["leg_remaining_estimate_s"] = None
        if task_samples:
            typ = self._mean(task_samples)
            out["task_typical_s"] = round(typ, 1)
            out["task_samples"] = len(task_samples)
            out["task_remaining_estimate_s"] = round(
                max(0.0, typ - task_elapsed), 1)
            out["known"] = True
        else:
            out["task_samples"] = 0
            out["task_typical_s"] = None
            out["task_remaining_estimate_s"] = None
            out["known"] = False
        out["say"] = self._progress_say(out)
        return out

    def _progress_say(self, p: dict) -> str:
        noun = self.task_noun
        leg = p["leg"]
        where = (f"I'm on the {leg} leg, {p['leg_elapsed_s']:.0f} seconds into it"
                 if leg and leg != "idle" else
                 f"I'm between {self.task_plural} right now")
        if not p.get("known"):
            # THE HONEST BRANCH. No completed job yet means no measured time,
            # and a made-up minute here is exactly the failure this replaces.
            return (f"{where}. I haven't finished a {noun} yet this session, "
                    f"so I have no measured time to estimate from — I'd only "
                    f"be guessing, and I won't.")
        est = p.get("task_remaining_estimate_s")
        typ = p.get("task_typical_s")
        n = p.get("task_samples")
        lead = (f"{where}. My last {noun} took about {typ:.0f} seconds"
                if n == 1 else
                f"{where}. My last {n} {self.task_plural} took about "
                f"{typ:.0f} seconds each")
        if est is None:
            return lead + "."
        if est <= 1.0:
            return (lead + f", and this one is already past that — it should "
                    f"finish any moment. That's an estimate from measured "
                    f"times, not a schedule.")
        return (lead + f", so roughly {est:.0f} more seconds on this one. "
                f"That's an estimate from measured times, not a schedule.")

    def state(self) -> dict:
        """The /state fragment. Same data, flat, cheap."""
        li = self.listing()
        out = {
            "pending_intents": li["pending_intents"],
            "constraints": li["constraints"],
            "autonomy_hold": li["autonomy_hold"],
            "notifications": li["notifications"],
            "intents_summary": li["summary"],
            "progress": self.progress(),
        }
        # This enumerates keys rather than copying `li`, so anything added to
        # listing() is invisible here unless it is added twice. That bit:
        # `restored_from_previous_run` reached /intents and never /state, while
        # the comment that introduced it claimed both. get_robot_state is the
        # tool the model is told to always call, and ask_robot (the Foreman's
        # only window onto a robot) reads /state -- so a robot that came back
        # holding an order nobody gave it this session could not explain why,
        # and the Foreman rule written to surface exactly that was unreachable.
        if getattr(self, "restored_note", ""):
            out["restored_from_previous_run"] = self.restored_note
        return out


# ── tool builders ────────────────────────────────────────────────────
#
# The DESCRIPTIONS are the load-bearing part of this file. The measured bug
# was not that the model could not schedule -- it was that no tool existed,
# so the model produced prose. These strings exist to make the model REACH
# for a tool the moment it hears "after", "once", "when", "until" or "don't
# ... until", and to make replying-without-calling feel wrong.


def _minutes_to_ttl(minutes: Any) -> Optional[float]:
    """Operator-stated duration -> seconds, or None for the store default.

    Measured: asked for a ten-minute restriction, the robot replied "I will
    stay out of the cart park row for the next 10 minutes" and the store
    applied its 30-minute default, because the tool had no duration parameter
    at all. The sentence was not the model guessing -- it was faithfully
    relaying the operator, while the mechanism quietly did something else. A
    promise the system cannot keep is a fabrication no matter who said it.
    """
    try:
        m = float(minutes)
    except (TypeError, ValueError):
        return None
    if m <= 0:
        return None
    # Bounded: a constraint is a safety rule, and one that outlives the
    # operator's shift because they said "forever" is its own hazard.
    return max(60.0, min(m * 60.0, 12 * 3600.0))


def build_intent_tools(tool_cls: Any, store: IntentStore) -> List[Any]:
    """Build the deferred-intent tool set for a bridge.

    ``tool_cls`` is the relay's ``Tool`` class, passed in so this module
    stays free of relay imports (and so a bridge without a relay can skip
    the whole thing)."""
    if tool_cls is None:
        return []
    noun, plural = store.task_noun, store.task_plural
    conds = ", ".join(store._supported_conditions())
    rules = "; ".join(f"{k} = {v}" for k, v in sorted(store.rules.items()))

    deferred_rule = (
        "THIS IS A REAL, PERSISTENT COMMITMENT: it is written to the robot's "
        "intent store, survives the end of this chat turn, and is checked by "
        "the robot's own autonomy loop. Replying 'I'll do that' WITHOUT "
        "calling one of these tools records NOTHING — the robot carries on "
        "as if the operator never spoke, which is a lie, not an answer.")

    tools = [
        tool_cls(
            # NAMED `pause_...`, NOT `stop_...`, ON PURPOSE. It was
            # stop_after_current_task first, and a 7B model routed a BARE
            # "stop" to it in two runs out of three -- keyword anchoring on
            # the tool NAME, which no amount of "not for a bare stop" in the
            # description fixed (saying it there made it worse: it put the
            # token "stop" at the top of the description too). With the token
            # gone from the name, stop_robot is the only tool matching
            # "stop", and the bare case routed correctly again. The HTTP
            # /intents route still accepts the old action name.
            name="pause_after_current_task",
            description=(
                f"Wait until the current {noun} is FINISHED, then pause -- "
                "instead of halting dead right now. For an order that names "
                "a later moment: 'after…', 'once you have finished…', 'when "
                f"you are done with that…', 'finish this {noun} then…'. An "
                "order with no later moment in it is NOT this tool. Use this "
                f"for 'after you place this cart, stop', 'finish that {noun} "
                "then wait', 'stop when you're done with that'. The robot "
                "KEEPS WORKING until the boundary, then pauses. "
                "until_told=true (the default) means it will NOT auto-resume "
                "after the usual quiet minute — only an explicit "
                "resume_autonomy brings it back, which is what 'stop until I "
                "tell you to continue' actually means. Use stop_robot instead "
                "ONLY when the operator wants motion to cease immediately. "
                + deferred_rule),
            parameters={
                "type": "object",
                "properties": {
                    "until_told": {
                        "type": "boolean",
                        "description": ("true (default) = stay stopped until "
                                        "the operator says carry on. false = "
                                        "an ordinary pause that auto-resumes "
                                        "after the quiet window.")},
                    "words": {
                        "type": "string",
                        "description": ("The operator's own words, verbatim. "
                                        "Stored so the robot can quote the "
                                        "order back later.")},
                },
            },
            dispatch=lambda args: store.schedule(
                "pause", "after_current_task",
                until_told=_as_bool(args.get("until_told"), True),
                words=str(args.get("words") or "")),
        ),
        tool_cls(
            name="hold_until_told",
            description=(
                "ONLY for an order that says how long: 'stay there UNTIL I "
                "tell you', 'wait for my signal', 'don't move until I come "
                "back'. A plain 'stop' / 'halt' with no 'until' in it is "
                "stop_robot, NOT this -- this one also switches off the "
                "automatic return to work, which the operator did not ask "
                "for. What it does: stops now AND cancels the usual "
                "auto-resume about a minute after the last command, so the "
                "robot stays put however long it takes. "
                "Only resume_autonomy clears it. If the operator wants the "
                f"current {noun} finished FIRST, use "
                "pause_after_current_task instead. " + deferred_rule),
            parameters={
                "type": "object",
                "properties": {
                    "words": {"type": "string",
                              "description": "The operator's own words."},
                },
            },
            dispatch=lambda args: store.hold_now(
                words=str(args.get("words") or "")),
        ),
        tool_cls(
            name="pause_when",
            description=(
                "Schedule a pause on a CONDITION the robot's own loop can "
                f"actually detect. Supported conditions: {conds}. Use for "
                f"'do one more {noun} then park and wait' "
                f"(condition=after_n_{plural}, count=1), 'stop the next time "
                "you pick something up' (condition=on_next_pickup), or "
                "'pause when you get to <leg>' (condition=at_leg). If the "
                "operator's condition is NOT in that list the tool REFUSES "
                "and hands you a sentence to relay — say it, do not invent "
                "an agreement. " + deferred_rule),
            parameters={
                "type": "object",
                "properties": {
                    "condition": {"type": "string",
                                  "description": f"One of: {conds}"},
                    "count": {"type": "number",
                              "description": (f"How many more {plural} to "
                                              "finish first (for the "
                                              f"after_n_{plural} condition).")},
                    "leg": {"type": "string",
                            "description": "Leg name (for at_leg)."},
                    "until_told": {"type": "boolean",
                                   "description": ("true (default) = do not "
                                                   "auto-resume afterwards.")},
                    "words": {"type": "string",
                              "description": "The operator's own words."},
                },
                "required": ["condition"],
            },
            dispatch=lambda args: store.schedule(
                "pause", args.get("condition"), count=args.get("count"),
                leg=args.get("leg"),
                until_told=_as_bool(args.get("until_told"), True),
                words=str(args.get("words") or "")),
        ),
        tool_cls(
            name="notify_when",
            description=(
                "Record a promise to TELL the operator when something "
                f"happens. Supported conditions: {conds}. Use for 'if you "
                "pick up another cart, tell me before you move it' "
                "(condition=on_next_pickup, pause_first=true) or 'let me know "
                f"when you've done another {noun}'. When the condition fires "
                "the robot records the notification in its state (and holds "
                "first if pause_first is set), so the next time you are asked "
                "you can report it truthfully. " + deferred_rule),
            parameters={
                "type": "object",
                "properties": {
                    "condition": {"type": "string",
                                  "description": f"One of: {conds}"},
                    "count": {"type": "number",
                              "description": f"Count for after_n_{plural}."},
                    "leg": {"type": "string", "description": "Leg for at_leg."},
                    "message": {"type": "string",
                                "description": ("What to report when it "
                                                "fires.")},
                    "pause_first": {
                        "type": "boolean",
                        "description": ("true = ALSO hold the robot when the "
                                        "condition fires, so it does not act "
                                        "before the operator has been told. "
                                        "'tell me BEFORE you move it' means "
                                        "true.")},
                    "words": {"type": "string",
                              "description": "The operator's own words."},
                },
                "required": ["condition"],
            },
            dispatch=lambda args: store.schedule(
                "notify", args.get("condition"), count=args.get("count"),
                leg=args.get("leg"),
                until_told=_as_bool(args.get("pause_first"), False),
                message=str(args.get("message") or ""),
                words=str(args.get("words") or "")),
        ),
        tool_cls(
            name="list_pending_intents",
            description=(
                "List everything the robot is currently WAITING to do: "
                "scheduled pauses, standing restrictions, whether it is on "
                "hold, and any notifications that have fired. Call this "
                "whenever the operator asks 'what are you waiting for?', "
                "'what did I tell you?', 'are you still holding?' — and "
                "answer from the reply, never from memory of this "
                "conversation."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: store.listing(),
        ),
        tool_cls(
            # "How much longer?" is a QUANTITATIVE question, and answering it
            # with a description of the present ("I'm towing the cart east
            # now") is a hedge. This tool returns the robot's own MEASURED
            # times so the answer is a number with a basis -- or an explicit
            # "no sample yet", which is also an answer.
            name="estimate_time_remaining",
            description=(
                f"How far through the current {noun} the robot is, and how "
                "much longer it is likely to take. Call this for 'how much "
                "longer?', 'when will you be done?', 'how long does that "
                f"take?', 'are you nearly finished?'. Returns the current leg "
                "and how long it has been on it, plus an ESTIMATE built from "
                f"the durations of this robot's OWN last few {plural} this "
                "session (never a configured or invented figure). Relay the "
                "'say' sentence, INCLUDING that it is an estimate. If "
                "'known' is false the robot has not finished one yet and has "
                "nothing to measure from — say that plainly instead of "
                "producing a comforting number."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: store.progress(),
        ),
        tool_cls(
            name="cancel_pending_intent",
            description=("Drop a scheduled intent the operator has changed "
                         "their mind about. Omit the id to clear all of "
                         "them."),
            parameters={
                "type": "object",
                "properties": {
                    "id": {"type": "string",
                           "description": "Intent id from "
                                          "list_pending_intents."},
                },
            },
            dispatch=lambda args: store.cancel(args.get("id")),
        ),
    ]

    if store.rules:
        tools += [
            tool_cls(
                name="set_constraint",
                description=(
                    "Record a STANDING RESTRICTION the robot must respect "
                    "from now on. This is NOT a stop: the robot keeps doing "
                    "all its other work and simply declines any job that "
                    "would break the rule, and it checks the rule BEFORE "
                    "each move. Use for 'don't go in there until I say so', "
                    "'stay out of X', 'no more pickups for now'. "
                    "Supported rules — " + rules + ". A rule outside that "
                    "list is REFUSED with a sentence to relay; do not agree "
                    "to a rule the robot cannot enforce. Calling stop_robot "
                    "for a 'don't do X until I say so' order is WRONG: it "
                    "stops the robot once and the rule is forgotten, so "
                    "autonomy walks straight back into the thing the "
                    "operator forbade. " + deferred_rule),
                parameters={
                    "type": "object",
                    "properties": {
                        "rule": {"type": "string",
                                 "description": "One of: "
                                                + ", ".join(sorted(store.rules))},
                        "words": {"type": "string",
                                  "description": "The operator's own words."},
                        "minutes": {
                            "type": "number",
                            "description": (
                                "How long the restriction lasts, if the "
                                "operator named a duration ('for ten minutes', "
                                "'for the rest of the shift'). Omit when they "
                                "did not say -- do NOT invent one, and do not "
                                "repeat a duration back to them unless you "
                                "passed it here, because what you say and what "
                                "is enforced must be the same number."),
                        },
                    },
                    "required": ["rule"],
                },
                dispatch=lambda args: store.set_constraint(
                    args.get("rule"), words=str(args.get("words") or ""),
                    ttl_s=_minutes_to_ttl(args.get("minutes"))),
            ),
            tool_cls(
                name="clear_constraint",
                description=(
                    "Lift a standing restriction — ONLY when the operator has "
                    "explicitly said it no longer applies ('you can go back "
                    "in', 'that is fine now', 'lift it'). NEVER call this in "
                    "the same turn as set_constraint: an operator who just "
                    "said 'don't do X' has not also said 'you may do X'. Omit "
                    "the id to lift them all."),
                parameters={
                    "type": "object",
                    "properties": {
                        "id": {"type": "string",
                               "description": ("Constraint id or rule name "
                                               "from list_pending_intents.")},
                    },
                },
                dispatch=lambda args: store.clear_constraint(args.get("id")),
            ),
        ]
    return tools


def _as_bool(v: Any, default: bool) -> bool:
    if v is None:
        return default
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in ("1", "true", "yes", "y", "on"):
        return True
    if s in ("0", "false", "no", "n", "off"):
        return False
    return default


# Tool names that SCHEDULE rather than command. A bridge treats these like a
# question for pause-arming purposes: scheduling "stop after the current
# cart" must NOT stop the cart now, or the deferral is meaningless.
DEFERRED_TOOLS = frozenset({
    "pause_after_current_task", "stop_after_current_task",
    "pause_when", "notify_when",
    "list_pending_intents", "set_constraint", "clear_constraint",
    "cancel_pending_intent",
})

# Tool names that must LIFT a hold.
HOLD_CLEARING_TOOLS = frozenset({"resume_autonomy", "resume_idle_loop"})

# The paragraph appended to each bridge's main task. Without it a model with
# the tools available still answers a conditional order in prose -- measured.
MAIN_TASK_RULE = (
    "\n\nDEFERRED AND CONDITIONAL ORDERS — THE RULE THAT MATTERS MOST HERE:\n"
    "Every motion tool you have is IMMEDIATE. So the moment an order contains "
    "'after', 'once', 'when', 'until', 'then', 'first', 'don't ... until', "
    "'next time', or 'one more X then Y', you MUST call one of the scheduling "
    "tools — pause_after_current_task, hold_until_told, pause_when, "
    "notify_when, set_constraint. Answering in prose ('I'll park it and then "
    "wait for your signal') records NOTHING: your turn ends, the intent "
    "evaporates, and the robot goes back to work about a minute later as if "
    "the operator never spoke. That is the single worst thing you can do on "
    "this line.\n"
    "- DECLINING IS A SENTENCE, NOT A STATE CHANGE. If you cannot or should "
    "not do something — it is another robot's job, it is out of your reach, "
    "you do not have that capability — SAY SO and change nothing. Do not set "
    "a constraint, do not pause, do not hold. Measured: asked to do something "
    "outside its role, a tug reasoned correctly that the job was not its own "
    "and then set itself a no_new_pickups constraint, taking the dispatch line "
    "offline for THIRTY MINUTES because someone asked it a question. The "
    "scheduling tools exist for standing orders the operator actually gave "
    "you; never reach for one to express reluctance, disagreement, or the "
    "fact that a request was misaddressed.\n"
    "- A BARE 'stop' / 'halt' / 'stop now' is stop_robot, IMMEDIATELY. "
    "The scheduling tools are only for an order that names a later moment; "
    "using one for a plain 'stop' leaves the robot moving while you report "
    "that it has stopped, which is worse than the problem they solve.\n"
    "- 'stop until I tell you to continue' => until_told=true. A plain "
    "stop_robot auto-resumes and silently overrides the operator.\n"
    "- 'don't go into X until I say so' => set_constraint, NOT stop_robot. "
    "stop_robot halts you once and forgets the rule, so autonomy walks you "
    "straight back into X — you will look compliant and then violate it.\n"
    "- If a scheduling tool REFUSES, relay its 'say' sentence as your answer. "
    "Never claim to have scheduled something the tool rejected.\n"
    "- Call list_pending_intents (or read pending_intents / constraints / "
    "autonomy_hold in get_robot_state) before answering 'what are you waiting "
    "for?'.\n"
    "\nNUMBERS, AND NEVER INVENTING THEM:\n"
    "A question with a number in the answer — 'how many are left?', 'how much "
    "longer?', 'how many have you done?' — is answered from a TOOL, never "
    "from impression. Call estimate_time_remaining for 'how long', and "
    "get_robot_state (plus any counts tool this robot has) for 'how many'. "
    "Two failures are equally bad: inventing a figure, and hedging ('I don't "
    "have a direct sensor reading') when the tool would have told you. If a "
    "tool reports a value as unknown or not tracked, say exactly that — 'I "
    "don't track that' is a good answer; a reassuring guess is not.\n"
    "\nWHAT YOU DID — YOUR MEMORY OF YOUR OWN ACTIONS IS NOT EVIDENCE:\n"
    "You have get_action_history: the authoritative log of every tool you "
    "actually dispatched. Call it BEFORE answering 'what did you just do', "
    "'did that land', 'did you stop', 'how far did you get', 'which one did "
    "you pick', or 'recap what I asked'. Answering those from recollection "
    "is how you end up stating a distance you never measured or a cart you "
    "never moved.\n"
    "- Never cite 'my logs' or 'my records' unless you have just called "
    "get_action_history and are quoting it.\n"
    "- If the operator insists you did — or did not — do something, CHECK "
    "before you agree. Confidence is not evidence. Do not apologise for a "
    "failure the log does not show, and do not deny an action the log does "
    "show; if the log contradicts them, say so plainly and cite the entry.\n"
    "- Say what you DID, not what you intended. If you are about to write "
    "'I am proceeding to...' or 'I will now...', either call the tool that "
    "does it in this same turn, or schedule it — a promise with no tool call "
    "behind it is a false report, not a plan.\n"
    "- A COUNT OF THINGS IN THE WORLD IS NOT A COUNT OF YOUR WORK. Fields "
    "that describe a place (how many carts are in the park row, how many "
    "parts are on the line) include work done before you started and by "
    "others. Only the fields explicitly labelled as THIS robot's totals "
    "(delivered_total, jobs_total, and the equivalents on other robots) say "
    "what you did. If your own total is 0, you have done none of it — no "
    "matter how full the row looks.\n"
    "- If the operator's question rests on something you did not do, correct "
    "the premise before acting on it. Checking costs one tool call; driving "
    "off to retrieve a cart you never parked wastes a trip and confirms a "
    "false belief.\n"
    "- A FIRED NOTIFICATION IS A FORECAST YOU WROTE EARLIER, not a report of "
    "what happened. Its 'promised_text' is the sentence you undertook to say; "
    "'fired_because' is what the robot ACTUALLY did to trigger it. Relay the "
    "measured event, and only repeat the promised sentence if it matches. "
    "When you ARM a notification, write a message that says what you will "
    "TELL the operator ('I'll let you know when this delivery is done'), "
    "never one that asserts an outcome you cannot yet have observed ('I have "
    "staged a fresh cart for you') — it will be delivered verbatim, later, as "
    "though you had just seen it."
)
