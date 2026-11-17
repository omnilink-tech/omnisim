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

"""The runner's trace writer (AgentBench SPEC 4.5).

A thin layer over ``common/trace.py``'s :class:`~agentbench.common.trace.Trace`
so an LLM run writes into **the same** ``trace.jsonl`` the Phase-0 scripted
agents write, with the same record kinds -- ``message`` / ``tool_call`` /
``final`` -- plus the LLM-specific metadata (per-turn token counts, provider
stop reasons, budget trips).

Two decisions worth naming:

**Nothing is lost, but the JSONL stays readable.** SPEC 4.5 says the trace file
is never truncated. A base64 screenshot or a 4 MB scene tree would make the
file unusable to read, so results above ``max_result_bytes`` are written whole
to ``tool_results/<n>_<name>.txt`` and the JSONL record carries the byte count,
a sha256 and a head excerpt. Nothing is discarded; a reviewer can verify the
excerpt against the file.

**Leak suspicion is a recorded field, not an assumption.** ``run_shell``
cannot be path-guarded on a bare host (see :mod:`.isolation`), so every command
and every result is scanned for the benchmark package's own path and a hit sets
``leak_suspect: true``. That turns "did this agent read the grader?" into a
grep instead of an argument.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

DEFAULT_MAX_RESULT_BYTES = 32 * 1024


def _text_of(result) -> str:
    if result is None:
        return ""
    if isinstance(result, str):
        return result
    try:
        return json.dumps(result, default=str, ensure_ascii=False)
    except Exception:                                    # noqa: BLE001
        return str(result)


class RunnerTrace:
    """Wraps a :class:`Trace`; adds turn metadata and result spilling."""

    def __init__(self, trace, run_dir, *, guard=None,
                 max_result_bytes=DEFAULT_MAX_RESULT_BYTES):
        self.trace = trace
        self.run_dir = Path(run_dir)
        self.spill_dir = self.run_dir / "tool_results"
        self.guard = guard
        self.max_result_bytes = int(max_result_bytes)
        self.leak_suspects = 0
        self.spilled = 0

    # -- passthrough -----------------------------------------------------
    @property
    def turns(self):
        return self.trace.turns

    @property
    def tool_calls(self):
        return self.trace.tool_calls

    @property
    def path(self):
        return self.trace.path

    def event(self, kind, **fields):
        return self.trace.event(kind, **fields)

    # -- records ---------------------------------------------------------
    def turn(self, text, *, usage=None, stop_reason=None, tool_names=(),
             raw=None):
        """One assistant turn: the canonical ``message`` record + metadata."""
        rec = self.trace.turn(text or "")
        self.trace.event("turn_meta", turn=self.trace.turns,
                         usage=(usage.as_dict() if usage is not None else None),
                         provider_stop_reason=stop_reason,
                         tool_calls=list(tool_names), provider=raw)
        return rec

    def tool(self, name, args, *, result=None, error=None, dt_s=None,
             extra=None):
        """One tool call, with full arguments and a non-lossy result."""
        n = self.trace.tool_calls + 1
        payload, meta = self._result_payload(n, name, result)
        leak = False
        if self.guard is not None:
            leak = bool(self.guard.flags(_text_of(args))
                        or self.guard.flags(_text_of(result)))
            if leak:
                self.leak_suspects += 1
        rec = self.trace.tool(name, args, result=payload, error=error,
                              dt_s=dt_s)
        # With a guard present, ALWAYS emit the meta record -- an explicit
        # `leak_suspect: false` is what makes "did this agent read the grader?"
        # a grep rather than an inference from absence.
        if meta or extra or self.guard is not None:
            self.trace.event("tool_meta", n=n, name=name, leak_suspect=leak,
                             **(meta or {}), **(extra or {}))
        return rec

    def final(self, text, self_verified):
        return self.trace.final(text, self_verified)

    def limit(self, reason, detail, ledger, budget):
        """A budget trip. Loud by design -- SPEC 2 makes it a result."""
        return self.trace.event("budget_exhausted", reason=reason,
                                detail=detail, ledger=ledger.as_dict(),
                                budget=budget.as_dict())

    # -- result spilling -------------------------------------------------
    def _result_payload(self, n, name, result):
        text = _text_of(result)
        if len(text.encode("utf-8", "replace")) <= self.max_result_bytes:
            return result, None
        self.spill_dir.mkdir(parents=True, exist_ok=True)
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
        dest = self.spill_dir / ("%04d_%s.txt" % (n, safe))
        dest.write_text(text, encoding="utf-8", errors="replace")
        self.spilled += 1
        blob = text.encode("utf-8", "replace")
        payload = {
            "spilled_to_file": str(dest),
            "bytes": len(blob),
            "sha256": hashlib.sha256(blob).hexdigest(),
            "head": text[:2000],
            "why": ("result exceeded %d bytes; the JSONL keeps a hash + head "
                    "and the full text is in the file (SPEC 4.5: nothing is "
                    "discarded)" % self.max_result_bytes),
        }
        return payload, {"spilled_to_file": str(dest), "bytes": len(blob)}
