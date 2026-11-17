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

"""Budget accounting.

The one that matters for honesty is
:func:`test_unreported_usage_stays_none_not_zero`: SPEC 3.1 says unmeasured
cost is ``null``, never ``0.0``, and a ledger that initialised its counters to
0 would quietly turn "we did not measure this" into "this was free".
"""

from __future__ import annotations

import time

from agentbench.runner.backends.base import Usage, usd_for
from agentbench.runner.budget import Budget, Ledger, StopReason


def test_unreported_usage_stays_none_not_zero():
    led = Ledger()
    assert led.tokens_in is None and led.tokens_out is None
    led.add_usage(Usage())                      # a turn that reported nothing
    assert led.tokens_in is None, "None must not decay to 0"
    led.add_usage(Usage(tokens_in=10, tokens_out=2))
    assert (led.tokens_in, led.tokens_out) == (10, 2)
    led.add_usage(Usage(tokens_in=5))
    assert (led.tokens_in, led.tokens_out) == (15, 2)


def test_add_usage_ignores_none_turn():
    led = Ledger()
    led.add_usage(None)
    assert led.per_turn == []


def test_per_turn_usage_is_recorded():
    led = Ledger()
    led.add_usage(Usage(tokens_in=1, tokens_out=2))
    led.add_usage(Usage(tokens_in=3, tokens_out=4))
    assert [u["tokens_in"] for u in led.per_turn] == [1, 3]


def test_max_turns_gate():
    b = Budget(max_turns=2, deadline_s=0)
    led = Ledger()
    assert led.check(b) is None
    led.turns = 2
    assert led.check(b) == StopReason.MAX_TURNS


def test_max_tool_calls_gate():
    b = Budget(max_tool_calls=3, deadline_s=0)
    led = Ledger()
    led.tool_calls = 3
    assert led.check_tool_call(b) == StopReason.MAX_TOOL_CALLS


def test_deadline_gate():
    b = Budget(deadline_s=time.time() - 1)
    led = Ledger()
    assert led.check(b) == StopReason.DEADLINE
    assert led.check_tool_call(b) == StopReason.DEADLINE


def test_token_gates_both_directions():
    b = Budget(max_tokens_in=100, max_tokens_out=10, deadline_s=0)
    led = Ledger()
    assert led.check_tokens(b) is None          # nothing measured yet
    led.add_usage(Usage(tokens_in=100, tokens_out=1))
    assert led.check_tokens(b) == StopReason.TOKEN_BUDGET
    led2 = Ledger()
    led2.add_usage(Usage(tokens_in=1, tokens_out=10))
    assert led2.check_tokens(b) == StopReason.TOKEN_BUDGET


def test_zero_means_unlimited():
    b = Budget(max_turns=0, max_tool_calls=0, max_tokens_in=0,
               max_tokens_out=0, deadline_s=0)
    led = Ledger()
    led.turns = 10_000
    led.tool_calls = 10_000
    led.add_usage(Usage(tokens_in=10**9, tokens_out=10**9))
    assert led.check(b) is None
    assert led.check_tool_call(b) is None


def test_tool_timeout_is_clamped_by_the_deadline():
    b = Budget(tool_timeout_s=180.0, deadline_s=time.time() + 10)
    led = Ledger()
    assert 6.0 <= led.tool_timeout(b) <= 8.5    # 10s left, minus the reserve
    b2 = Budget(tool_timeout_s=5.0, deadline_s=time.time() + 3600)
    assert led.tool_timeout(b2) == 5.0
    b3 = Budget(tool_timeout_s=180.0, deadline_s=time.time() + 1)
    assert led.tool_timeout(b3) >= 1.0          # never zero or negative


def test_budget_reasons_are_classified():
    for r in (StopReason.MAX_TURNS, StopReason.MAX_TOOL_CALLS,
              StopReason.DEADLINE, StopReason.TOKEN_BUDGET):
        assert StopReason.is_budget(r)
    for r in (StopReason.MODEL_STOPPED, StopReason.BACKEND_ERROR,
              StopReason.SCRIPT_EXHAUSTED, StopReason.NO_CREDENTIAL,
              StopReason.REFUSAL):
        assert not StopReason.is_budget(r)


# -- cost ------------------------------------------------------------------

def test_usd_is_none_for_an_unpriced_model():
    assert usd_for("some-unlisted-model", 1000, 1000) is None


def test_usd_is_none_when_tokens_were_not_measured():
    assert usd_for("claude-opus-5", None, 10) is None


def test_usd_uses_the_pinned_list_price():
    # 1M in @ $5, 1M out @ $25.
    assert usd_for("claude-opus-5", 1_000_000, 1_000_000) == 30.0
    # Cache reads are billed separately and cheaply.
    assert usd_for("claude-opus-5", 0, 0, 1_000_000) == 0.5
