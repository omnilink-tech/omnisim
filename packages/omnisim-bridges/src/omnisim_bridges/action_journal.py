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


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


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
        try:
            self._seq = max(int(data.get("seq") or 0), 0)
        except (TypeError, ValueError):
            pass
        try:
            self._started_at = float(data.get("started_at") or self._started_at)
        except (TypeError, ValueError):
            pass

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
