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

"""Append-only run trace (AgentBench SPEC 4.5).

Every run writes ``trace.jsonl``: every action the agent took with full
arguments, every result, timestamps. Truncation is a *transcript-view*
concern; the file is never truncated. Phase 0's scripted agents write the
same records an LLM loop will, so the trace schema is exercised before a
single token is spent.
"""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path


def utcnow() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


class Trace:
    """Append-only JSONL writer. One instance per run directory."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.run_dir / "trace.jsonl"
        self.t0 = time.perf_counter()
        self.turns = 0
        self.tool_calls = 0
        self._fh = open(self.path, "a", encoding="utf-8")

    def _write(self, rec):
        rec["utc"] = utcnow()
        rec["t_s"] = round(time.perf_counter() - self.t0, 6)
        self._fh.write(json.dumps(rec, default=str) + "\n")
        self._fh.flush()
        return rec

    def event(self, kind, **fields):
        return self._write({"kind": kind, **fields})

    def turn(self, text):
        self.turns += 1
        return self._write({"kind": "message", "role": "assistant",
                            "turn": self.turns, "text": text})

    def tool(self, name, args, result=None, error=None, dt_s=None):
        self.tool_calls += 1
        return self._write({"kind": "tool_call", "name": name, "args": args,
                            "result": result, "error": error,
                            "dt_s": dt_s, "n": self.tool_calls})

    def final(self, text, self_verified):
        return self._write({"kind": "final", "text": text,
                            "self_verified": bool(self_verified)})

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
