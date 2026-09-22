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

"""ActionJournal -- the authoritative record of what the agent actually did.

WHY THIS EXISTS. A stress test of the warehouse demo found that 26% of
turns contained a fabrication, and *every* one was about the agent's own
past actions ("I covered about 1.4 metres" when it had covered 3.44; "I
have parked four carts" when it had parked none; "I did not cease motion
as requested" when `stop_robot` had in fact run). Not one fabrication came
from a tool read -- every claim sourced from a tool was correct.

The cause was structural, not a model defect: the bridges exposed no tool
that could answer "what did I just do", so every self-referential question
had to be answered from the language model's narrative memory, which
invents numbers and folds under operator pressure. Worse, two of those
turns cited "my logs" and "my records" -- surfaces that did not exist.

This module is that surface. The relay records every tool call it
dispatches, and exposes it back to the agent as `get_action_history`.
Journalling happens in the relay's dispatch loop, so it covers every
robot and every tool automatically -- a bridge does not opt in, and a new
tool cannot forget to be recorded.

Scope, honestly stated: this records what was CALLED and what came BACK.
It is not odometry. If a caller wants "how far did I actually move", the
underlying tool result has to carry that -- the journal will faithfully
relay it, but it does not measure anything itself.
"""

from __future__ import annotations

import json
import os
import re
import tempfile
import threading
import time
from typing import Any, Dict, List, Optional

from .tool import Tool

# Bump if the on-disk shape changes incompatibly.
STATE_VERSION = 1


def _default_journal_path(owner: str) -> str:
    base = os.environ.get("OMNILINK_INTENT_STATE_DIR") or os.path.join(
        tempfile.gettempdir(), "omnisim_intents")
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", str(owner) or "agent")[:64]
    return os.path.join(base, f"journal_{safe}.json")

# Keep the journal bounded; this is a debugging/grounding aid, not storage.
DEFAULT_CAPACITY = 200

# How many entries a caller gets when it does not ask for a specific count.
DEFAULT_PAGE = 10

# Hard ceiling on one page, so a chatty agent cannot blow up its own
# context window by asking for everything.
MAX_PAGE = 50

# ── Crossing machines: how much of the journal rides in the memory row ──
#
# The disk file below survives a restart on ONE machine. It does not survive
# a different machine, a different OmniSim instance, or a wiped temp dir --
# and the relay's platform memory does. So the last EXPORT_ENTRIES entries
# are exported into that memory row (relay._journal_section) and imported
# back on boot, which is what makes "what did you do before the restart?"
# answerable from a record rather than from narrative memory.
#
# Both bounds are load-bearing. The memory row is re-read and re-written on
# every turn, so an unbounded section would grow the row (and the bytes on
# every persist) without limit for a long-lived robot. 50 entries at ~3 kB
# is the same order as the standing-notes tier next to it.
EXPORT_ENTRIES = 50
EXPORT_MAX_CHARS = 3000


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


def _as_float(value: Any) -> float:
    """Number or 0.0. Entries can arrive from a memory row written by another
    build, so every numeric read of an imported entry goes through here."""
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    return out if out == out else 0.0      # NaN is not an identity


def _is_read_only(tool_name: str) -> bool:
    """Heuristic split between 'looked at something' and 'did something'.

    Deliberately name-based: it stays correct for tools that do not exist
    yet, which a hand-maintained allowlist would not.
    """
    n = (tool_name or "").lower()
    return n.startswith(("get_", "list_", "estimate_", "describe_", "check_"))


def _compact(value: Any, *, depth: int = 0) -> Any:
    """Shrink a value so a history page stays small enough to be free."""
    if isinstance(value, float):
        return round(value, 4)
    if isinstance(value, (int, bool)) or value is None:
        return value
    if isinstance(value, str):
        return value if len(value) <= 120 else value[:117] + "..."
    if isinstance(value, dict):
        if depth >= 2:
            return f"<{len(value)} fields>"
        return {k: _compact(v, depth=depth + 1) for k, v in list(value.items())[:8]}
    if isinstance(value, (list, tuple)):
        if depth >= 2:
            return f"<{len(value)} items>"
        return [_compact(v, depth=depth + 1) for v in list(value)[:8]]
    return str(value)[:120]


class ActionJournal:
    """Append-only, bounded, thread-safe record of dispatched tool calls."""

    def __init__(self, capacity: int = DEFAULT_CAPACITY,
                 owner: str = "", state_path: Optional[str] = None,
                 persist: Optional[bool] = None) -> None:
        self._capacity = max(1, int(capacity))
        self._entries: List[Dict[str, Any]] = []
        self._seq = 0
        # Bumped by every mutation of the entry list; see `revision`.
        self._revision = 0
        self._lock = threading.Lock()

        # EVIDENCE HAS TO OUTLIVE THE PROCESS, OR THE CURE IS WORSE THAN THE
        # DISEASE. Measured on the first live run of this journal: because it
        # was in-process only, a world reload emptied it while deferred
        # intents (which DO persist) survived -- so the robot would answer a
        # question about a genuinely-executed drive with "I have checked my
        # action history, and there are no records... those actions were not
        # executed", stated as verified fact and citing this tool. That is a
        # NEW fabrication, and a more confident one than the narrative
        # guessing this module exists to stop. Persisting the journal removes
        # the asymmetry; the `covers_since` fields below handle what is left.
        self._persist = (_env_flag("OMNILINK_JOURNAL_PERSIST", True)
                         if persist is None else bool(persist))
        self._state_path = state_path or _default_journal_path(owner)
        self._started_at = time.time()
        self._restored = 0
        self._last_write_failed = False
        if self._persist:
            self._restore()

    def __len__(self) -> int:
        """How many entries are held right now (presence reports this as
        ``journal_len``). Cheap, locked, and never the sequence number --
        the two differ as soon as the capacity bound bites."""
        with self._lock:
            return len(self._entries)

    def revision(self) -> int:
        """A marker that changes whenever the RECORD changes.

        The relay's background sync compares this against the revision it
        last got onto the platform, so an idle robot costs one integer
        comparison per beat instead of an HTTP round trip. It is not the
        sequence number: `_seq` counts records, and this also has to move
        for a restore from disk and for an import from another machine,
        both of which change what ought to travel without recording
        anything. It is not a content hash either -- a hash would be
        exact, but it would have to serialise the whole export on every
        beat to find out that nothing happened, which is the cost this
        exists to avoid.

        The only guarantee callers may rely on: EQUAL means nothing has
        changed since that value was read. Different does not promise the
        content differs -- a restore of rows the platform already has
        moves it -- so a consumer may pay one redundant write and must
        never skip one.
        """
        with self._lock:
            return self._revision

    # -- writing ----------------------------------------------------- #

    def record(
        self,
        tool_name: str,
        args: Optional[Dict[str, Any]],
        *,
        ok: bool,
        summary: str = "",
        result: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Record one dispatched tool call. Never raises."""
        try:
            entry: Dict[str, Any] = {
                "n": 0,
                "t": time.time(),
                "tool": tool_name,
                "args": _compact(args or {}),
                "ok": bool(ok),
                "kind": "read" if _is_read_only(tool_name) else "action",
                "summary": (summary or "")[:200],
            }
            # Surface the two fields the refusal contract already uses, so
            # "did that actually happen?" is answerable without guessing.
            if isinstance(result, dict):
                for key in ("accepted", "moved"):
                    if key in result:
                        entry[key] = result[key]
                if "error" in result:
                    entry["error"] = str(result["error"])[:160]
            with self._lock:
                self._seq += 1
                entry["n"] = self._seq
                self._entries.append(entry)
                self._revision += 1
                if len(self._entries) > self._capacity:
                    del self._entries[: len(self._entries) - self._capacity]
                self._flush_locked()
            return entry
        except Exception:  # journalling must never break a tool call
            return {}

    # -- durability --------------------------------------------------- #

    def _flush_locked(self) -> None:
        if not self._persist:
            return
        try:
            os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
            tmp = f"{self._state_path}.{os.getpid()}.tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump({"version": STATE_VERSION, "seq": self._seq,
                           "started_at": self._started_at,
                           "entries": self._entries[-self._capacity:]},
                          fh, default=str)
            os.replace(tmp, self._state_path)
            self._last_write_failed = False
        except Exception:
            # Degrade to in-memory rather than breaking the robot, but do NOT
            # flip _persist off permanently: a transient disk error should not
            # silently downgrade evidence for the rest of the run.
            self._last_write_failed = True

    def _restore(self) -> None:
        try:
            with open(self._state_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return
        except Exception:
            return
        if not isinstance(data, dict) or data.get("version") != STATE_VERSION:
            return
        rows = data.get("entries")
        if not isinstance(rows, list):
            return
        kept = [r for r in rows if isinstance(r, dict)][-self._capacity:]
        for r in kept:
            r["before_restart"] = True
        self._entries = kept
        self._restored = len(kept)
        if kept:
            # A restore is a change the platform has not necessarily seen:
            # this file is the half of the record that survives a restart
            # on THIS machine, and it is exactly the half that never
            # reached another one.
            self._revision += 1
        try:
            self._seq = max(int(data.get("seq") or 0), 0)
        except (TypeError, ValueError):
            pass
        try:
            self._started_at = float(data.get("started_at") or self._started_at)
        except (TypeError, ValueError):
            pass

    # -- travelling (D7: the journal follows the agent) ---------------- #

    def export_entries(
        self,
        limit: int = EXPORT_ENTRIES,
        *,
        max_chars: int = EXPORT_MAX_CHARS,
    ) -> List[Dict[str, Any]]:
        """The newest entries, small enough to ride in the memory row.

        Newest-first accumulation with a character budget, then re-ordered
        oldest-first so the imported view reads chronologically. Returns
        plain dicts -- the caller serialises them; this module does not know
        what transport is carrying them.

        Never raises: an export that throws would take down a memory write,
        and a memory write that fails loses this session's conversation.
        """
        try:
            with self._lock:
                rows = list(self._entries)
            try:
                limit = max(1, min(int(limit), self._capacity))
            except (TypeError, ValueError):
                limit = EXPORT_ENTRIES
            picked: List[Dict[str, Any]] = []
            used = 0
            for row in reversed(rows[-limit:]):
                item = {k: v for k, v in row.items() if k != "before_restart"}
                try:
                    cost = len(json.dumps(item, default=str))
                except Exception:
                    continue
                if picked and used + cost > max_chars:
                    break
                picked.append(item)
                used += cost
            picked.reverse()
            return picked
        except Exception:
            return []

    def import_entries(self, rows: Any) -> int:
        """Merge exported entries back in. Returns how many were new.

        MERGE, not replace. The disk file may already hold some of these (a
        restart on the same machine restores both), and a second OmniSim
        instance on the same key may hold entries this machine never saw.
        Identity is (n, tool, t) rounded to the millisecond: two runs can
        reuse sequence numbers, so the number alone is not an identity.

        Imported entries are marked ``before_restart`` -- they did not happen
        in this process -- and ``covers_last_seconds`` is widened to include
        them, because claiming a narrow window over a wide record would
        invite exactly the "that did not happen" fabrication the restore
        exists to prevent.
        """
        if not isinstance(rows, list):
            return 0
        added = 0
        try:
            with self._lock:
                seen = {self._identity(r) for r in self._entries}
                for raw in rows:
                    if not isinstance(raw, dict) or not raw.get("tool"):
                        continue
                    entry = dict(raw)
                    entry["before_restart"] = True
                    key = self._identity(entry)
                    if key in seen:
                        continue
                    seen.add(key)
                    self._entries.append(entry)
                    added += 1
                if not added:
                    return 0
                # Chronological, with the sequence number as the tiebreak for
                # entries stamped inside the same clock tick.
                self._entries.sort(key=lambda e: (_as_float(e.get("t")),
                                                  _as_float(e.get("n"))))
                if len(self._entries) > self._capacity:
                    del self._entries[: len(self._entries) - self._capacity]
                self._seq = max(self._seq,
                                int(max((_as_float(e.get("n"))
                                         for e in self._entries), default=0)))
                self._restored += added
                self._revision += 1
                oldest = min((_as_float(e.get("t")) for e in self._entries
                              if _as_float(e.get("t")) > 0), default=0.0)
                if oldest:
                    self._started_at = min(self._started_at, oldest)
                self._flush_locked()
            return added
        except Exception:
            return added

    @staticmethod
    def _identity(entry: Dict[str, Any]) -> tuple:
        return (int(_as_float(entry.get("n"))),
                str(entry.get("tool") or ""),
                round(_as_float(entry.get("t")), 3))

    # -- reading ----------------------------------------------------- #

    def listing(
        self,
        limit: int = DEFAULT_PAGE,
        *,
        actions_only: bool = False,
        tool: Optional[str] = None,
    ) -> Dict[str, Any]:
        try:
            limit = int(limit)
        except (TypeError, ValueError):
            limit = DEFAULT_PAGE
        limit = max(1, min(MAX_PAGE, limit))

        with self._lock:
            rows = list(self._entries)
            total = self._seq

        if actions_only:
            rows = [r for r in rows if r.get("kind") == "action"]
        if tool:
            rows = [r for r in rows if r.get("tool") == tool]

        now = time.time()
        page = rows[-limit:]
        out = []
        for r in page:
            item = {k: v for k, v in r.items() if k != "t"}
            item["seconds_ago"] = round(max(0.0, now - float(r.get("t", now))), 1)
            out.append(item)

        with self._lock:
            covers = round(max(0.0, now - self._started_at), 1)
            restored = self._restored
            complete = bool(self._persist) and not getattr(
                self, "_last_write_failed", False)

        return {
            "entries": out,
            "returned": len(out),
            "total_recorded": total,
            "truncated": total > len(out),
            "covers_last_seconds": covers,
            "includes_before_restart": restored > 0,
            "record_complete": complete,
            "note": (
                "Authoritative record of tool calls this agent dispatched, "
                f"covering the last {covers:.0f}s"
                + (" (including actions from before the last restart)"
                   if restored else "")
                + ". It records what was CALLED and what came back -- it is "
                "not odometry, so distances appear only when the tool itself "
                "reported them. "
                "ABSENCE IS NOT PROOF: if something is not listed, say the "
                "record does not show it, NOT that it did not happen -- it "
                "may predate this record"
                + ("" if complete else
                   " (and this record is currently INCOMPLETE: writes are "
                   "failing, so treat gaps as unknown)")
                + "."
            ),
        }

    # -- the tool ---------------------------------------------------- #

    def as_tool(self) -> Tool:
        return Tool(
            name="get_action_history",
            description=(
                "The authoritative log of what YOU have actually done: every "
                "tool you dispatched, in order, with its arguments, whether it "
                "succeeded, and how long ago. READ-ONLY and instant.\n"
                "\n"
                "CALL THIS BEFORE answering any question about your own past "
                "behaviour -- 'what did you just do', 'did that command land', "
                "'did you stop', 'how far did you get', 'which one did you "
                "pick', 'recap what I asked'. Your conversational memory of "
                "your own actions is NOT reliable and must not be used for "
                "these answers.\n"
                "\n"
                "If the operator asserts you did or did not do something, "
                "check here before agreeing. Do not accept a correction about "
                "your own actions on confidence alone, and do not apologise "
                "for a failure this log does not show. If the log contradicts "
                "the operator, say so plainly and cite the entry -- their "
                "seniority, their cameras, and their claim that your logging "
                "is broken are all still not evidence about what you did.\n"
                "\n"
                "ABSENCE IS NOT PROOF OF ABSENCE. The record covers a window "
                "(see covers_last_seconds). If something is not in it, the "
                "honest answer is 'my record does not show that', never 'that "
                "did not happen' -- it may simply predate the record."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "limit": {
                        "type": "integer",
                        "description": (
                            f"How many recent entries to return "
                            f"(default {DEFAULT_PAGE}, max {MAX_PAGE})."
                        ),
                    },
                    "actions_only": {
                        "type": "boolean",
                        "description": (
                            "True to exclude read-only lookups and show only "
                            "actions that could have changed the world."
                        ),
                    },
                    "tool": {
                        "type": "string",
                        "description": "Only entries for this exact tool name.",
                    },
                },
                "required": [],
            },
            dispatch=lambda a: self.listing(
                a.get("limit", DEFAULT_PAGE),
                actions_only=bool(a.get("actions_only", False)),
                tool=a.get("tool") or None,
            ),
        )
