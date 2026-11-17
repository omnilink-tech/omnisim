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

"""Budgets, and the accounting that makes exhausting one a *recorded* result.

SPEC 2: "**Timeout = 3 x par**, hard. **Max turns = 60.** **Max tokens =
400k in / 60k out.** Any budget exhausted -> ``FAIL`` (not ``INVALID``) --
running out of budget is a result."

That last clause is the whole design constraint. A limit that silently
truncates the loop is indistinguishable from a model that gave up, so every
limit here produces a named :class:`StopReason` that the trace and the result
row carry. And SPEC 3.1's "unmeasured cost is ``null``, never ``0.0``" applies
to the ledger too: a backend that reports no token usage leaves
``tokens_in``/``tokens_out`` as ``None``, it does not report zero.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

# SPEC 2 constants. Overridable per run, but these are the defaults the
# published numbers are produced with.
DEFAULT_MAX_TURNS = 60
DEFAULT_MAX_TOOL_CALLS = 240
DEFAULT_MAX_TOKENS_IN = 400_000
DEFAULT_MAX_TOKENS_OUT = 60_000


class StopReason:
    """Why the loop stopped. Exactly one of these ends every run."""

    MODEL_STOPPED = "model_stopped"        # the only clean end
    MAX_TURNS = "max_turns"
    MAX_TOOL_CALLS = "max_tool_calls"
    DEADLINE = "deadline"
    TOKEN_BUDGET = "token_budget"
    BACKEND_ERROR = "backend_error"
    SCRIPT_EXHAUSTED = "script_exhausted"  # scripted backend ran out of turns
    NO_CREDENTIAL = "no_credential"
    REFUSAL = "refusal"                    # provider declined the request

    BUDGET_REASONS = (MAX_TURNS, MAX_TOOL_CALLS, DEADLINE, TOKEN_BUDGET)

    @classmethod
    def is_budget(cls, reason) -> bool:
        return reason in cls.BUDGET_REASONS


@dataclass
class Budget:
    """The four limits. ``0``/``None`` means "no limit" for that dimension."""

    max_turns: int = DEFAULT_MAX_TURNS
    max_tool_calls: int = DEFAULT_MAX_TOOL_CALLS
    max_tokens_in: int = DEFAULT_MAX_TOKENS_IN
    max_tokens_out: int = DEFAULT_MAX_TOKENS_OUT
    # Absolute wall clock (``time.time()`` scale), matching AgentContext's
    # ``deadline_s`` so the runner and the harness agree on one clock.
    deadline_s: float = 0.0
    # A tool call whose own timeout would run past the deadline is clamped;
    # this is the ceiling for a single call when the deadline is far away.
    tool_timeout_s: float = 180.0

    def as_dict(self):
        return {"max_turns": self.max_turns,
                "max_tool_calls": self.max_tool_calls,
                "max_tokens_in": self.max_tokens_in,
                "max_tokens_out": self.max_tokens_out,
                "deadline_s": self.deadline_s,
                "tool_timeout_s": self.tool_timeout_s}


@dataclass
class Ledger:
    """Actuals. Every field here ends up in the result row's ``metrics``.

    ``turns`` counts assistant messages containing >= 1 tool call (SPEC 3.1's
    definition), which is **not** the same as the number of model calls -- the
    final text-only message is a model call but not a turn. Both are recorded,
    because conflating them silently understates cost.
    """

    turns: int = 0
    model_calls: int = 0
    tool_calls: int = 0
    tokens_in: int | None = None
    tokens_out: int | None = None
    tokens_cache_read: int | None = None
    tokens_cache_write: int | None = None
    t0: float = field(default_factory=time.time)
    stop_reason: str | None = None
    stop_detail: str = ""
    # Per-turn usage, so a reviewer can see where the tokens went.
    per_turn: list = field(default_factory=list)

    # -- accounting ------------------------------------------------------
    def add_usage(self, usage):
        """Fold one turn's usage in. ``None`` in stays ``None``."""
        if usage is None:
            return
        self.per_turn.append(usage.as_dict())
        for attr in ("tokens_in", "tokens_out", "tokens_cache_read",
                     "tokens_cache_write"):
            inc = getattr(usage, attr, None)
            if inc is None:
                continue
            cur = getattr(self, attr)
            setattr(self, attr, (0 if cur is None else cur) + int(inc))

    @property
    def elapsed_s(self) -> float:
        return time.time() - self.t0

    # -- limit checks ----------------------------------------------------
    def check(self, budget: Budget) -> str | None:
        """The pre-model-call gate. Returns a StopReason or None."""
        if budget.max_turns and self.turns >= budget.max_turns:
            return StopReason.MAX_TURNS
        if budget.deadline_s and time.time() >= budget.deadline_s:
            return StopReason.DEADLINE
        return self.check_tokens(budget)

    def check_tokens(self, budget: Budget) -> str | None:
        """The post-turn gate. Token limits can only be checked after the
        fact -- the API bills what it bills -- so a run may overshoot the
        budget by one turn. That is recorded, not hidden."""
        if (budget.max_tokens_in and self.tokens_in is not None
                and self.tokens_in >= budget.max_tokens_in):
            return StopReason.TOKEN_BUDGET
        if (budget.max_tokens_out and self.tokens_out is not None
                and self.tokens_out >= budget.max_tokens_out):
            return StopReason.TOKEN_BUDGET
        return None

    def check_tool_call(self, budget: Budget) -> str | None:
        """The pre-tool-execution gate."""
        if budget.max_tool_calls and self.tool_calls >= budget.max_tool_calls:
            return StopReason.MAX_TOOL_CALLS
        if budget.deadline_s and time.time() >= budget.deadline_s:
            return StopReason.DEADLINE
        return None

    def remaining_s(self, budget: Budget) -> float | None:
        if not budget.deadline_s:
            return None
        return max(0.0, budget.deadline_s - time.time())

    def tool_timeout(self, budget: Budget) -> float:
        rem = self.remaining_s(budget)
        if rem is None:
            return budget.tool_timeout_s
        # Leave a slice for the loop to write its trace and return cleanly.
        return max(1.0, min(budget.tool_timeout_s, rem - 2.0))

    def as_dict(self):
        return {"turns": self.turns, "model_calls": self.model_calls,
                "tool_calls": self.tool_calls, "tokens_in": self.tokens_in,
                "tokens_out": self.tokens_out,
                "tokens_cache_read": self.tokens_cache_read,
                "tokens_cache_write": self.tokens_cache_write,
                "elapsed_s": round(self.elapsed_s, 3),
                "stop_reason": self.stop_reason,
                "stop_detail": self.stop_detail}
