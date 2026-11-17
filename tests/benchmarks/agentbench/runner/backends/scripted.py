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

"""The scripted backend: a fixed list of tool calls, replayed as a "model".

This is not a stub and not a mock of the model call -- it is a *first-class
backend*, and it is load-bearing three ways:

1. **It is how the runner is tested without a credential.** The loop, the tool
   set, the isolation, the trace and the grader integration are all exercised
   end to end by replaying a known-good solution and checking the real grader
   says PASS. Nothing about the loop is bypassed: the same
   ``turn(system, messages, tools)`` call, the same tool dispatch, the same
   budget accounting, the same trace records.
2. **It is how the oracle can be re-expressed as a model.** The Phase-0 oracle
   (``agents/oracle_a1.py``) proves the task is passable by calling Python
   directly. A script proves it is passable *through the tool surface an LLM
   has* -- which is a different and stronger claim, and it is the claim the
   ``shell`` condition rests on.
3. **It is how a run is replayed deterministically.** A recorded trace can be
   turned back into a script, so a suspicious result can be re-executed rather
   than re-argued (SPEC 4.5).

Script format (``agentbench/script/v1``)::

    {"schema": "agentbench/script/v1",
     "name": "a1_oracle_replay",
     "description": "why this script exists",
     "turns": [
       {"text": "what the model 'said'",
        "tool_calls": [{"name": "write_file",
                        "arguments": {"path": "x.wbt", "content": "..."}}],
        "usage": {"tokens_in": 1200, "tokens_out": 800},
        "repeat": 1,          # optional: emit this turn N times
        "sleep_s": 0.0},      # optional: simulate model latency
       {"text": "final answer, no tool calls"}
     ]}

Any string in ``text`` or in a tool call's ``arguments`` may contain
``{{VAR}}`` placeholders, substituted at replay time from the run's variables
(scratch dir, repo root, the Husky URDF path, the assigned ports). A script is
therefore machine-independent while still producing absolute paths -- which
matters because ``URDFRobot { url ... }`` resolves relative to the world file
and a per-run scratch dir cannot hold a relative path back into the repo.

**Cost is ``None``, always.** A replay has no list price, so
``price_table()`` returns ``None`` and the row's ``usd`` is ``null``. The
declared ``usage`` numbers are still counted by the ledger, because that is
how the token-budget limit is tested -- but they are labelled ``synthetic`` in
the trace so nobody mistakes them for a measurement.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from agentbench.runner.backends.base import (
    BackendExhausted, ModelBackend, ModelTurn, ToolCall, Usage)

SCHEMA = "agentbench/script/v1"


def substitute(obj, variables: dict):
    """Recursively replace ``{{VAR}}`` in every string inside ``obj``."""
    if isinstance(obj, str):
        out = obj
        for k, v in variables.items():
            out = out.replace("{{%s}}" % k, str(v))
        return out
    if isinstance(obj, list):
        return [substitute(x, variables) for x in obj]
    if isinstance(obj, dict):
        return {k: substitute(v, variables) for k, v in obj.items()}
    return obj


def load_script(path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    schema = data.get("schema")
    if schema != SCHEMA:
        raise ValueError("script %s: expected schema %r, got %r"
                         % (path, SCHEMA, schema))
    if not isinstance(data.get("turns"), list) or not data["turns"]:
        raise ValueError("script %s: 'turns' must be a non-empty list" % path)
    return data


class ScriptedBackend(ModelBackend):
    """Replays ``turns`` from a script file. One turn per ``turn()`` call."""

    kind = "scripted"

    def __init__(self, script_path, variables=None, *, model=None):
        self.script_path = str(script_path)
        self.script = load_script(script_path)
        self.variables = dict(variables or {})
        self.name = self.script.get("name") or Path(script_path).stem
        # The "model" identifier a row records. Deliberately prefixed so a
        # scripted row can never be mistaken for a model run in an aggregate.
        self.model = model or ("scripted:%s" % self.name)
        self.temperature = None
        self._queue = self._expand(self.script["turns"])
        self._served = 0

    # -- script expansion ------------------------------------------------
    @staticmethod
    def _expand(turns):
        out = []
        for t in turns:
            n = int(t.get("repeat", 1))
            if n < 1:
                raise ValueError("script turn 'repeat' must be >= 1")
            for _ in range(n):
                out.append(t)
        return out

    @property
    def remaining(self) -> int:
        return len(self._queue) - self._served

    def describe(self):
        d = super().describe()
        d.update({"script": self.script_path, "script_name": self.name,
                  "script_turns": len(self._queue),
                  "usage_is_synthetic": True})
        return d

    def price_table(self):
        return None            # a replay has no list price -> usd is None

    # -- the contract ----------------------------------------------------
    def turn(self, system, messages, tools) -> ModelTurn:
        if self._served >= len(self._queue):
            raise BackendExhausted(
                "script %s exhausted after %d turns"
                % (self.name, self._served))
        spec = self._queue[self._served]
        self._served += 1

        if spec.get("sleep_s"):
            time.sleep(float(spec["sleep_s"]))

        text = substitute(spec.get("text", ""), self.variables)
        calls = []
        known = {t["name"] for t in (tools or [])}
        for i, c in enumerate(spec.get("tool_calls") or []):
            name = c["name"]
            args = substitute(c.get("arguments") or {}, self.variables)
            # A script that calls a tool the condition does not grant is a
            # BUG IN THE SCRIPT, and it must be loud: silently dropping it
            # would let a `shell`-condition replay quietly use a harness tool.
            if known and name not in known:
                raise ValueError(
                    "script %s turn %d calls %r, which is not in the %s tool "
                    "set (%s)" % (self.name, self._served, name,
                                  "current", ", ".join(sorted(known))))
            calls.append(ToolCall(id="scripted_%d_%d" % (self._served, i),
                                  name=name, arguments=args))

        u = spec.get("usage") or {}
        usage = Usage(tokens_in=u.get("tokens_in"),
                      tokens_out=u.get("tokens_out"),
                      tokens_cache_read=u.get("tokens_cache_read"),
                      tokens_cache_write=u.get("tokens_cache_write"))

        # Provider-native assistant content, in the canonical (Anthropic)
        # shape, so the loop's history handling is identical to a live run.
        content = []
        if text:
            content.append({"type": "text", "text": text})
        for c in calls:
            content.append({"type": "tool_use", "id": c.id, "name": c.name,
                            "input": c.arguments})

        return ModelTurn(
            text=text, tool_calls=calls, usage=usage,
            stop_reason="tool_use" if calls else "end_turn",
            assistant_content=content,
            raw={"script_turn": self._served, "usage_is_synthetic": True})
