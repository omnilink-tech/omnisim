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

"""The API-driven agent runner (AgentBench SPEC 4).

**Why this package exists.** The A/B conditions run so far were Claude Code
subagents, and that harness injects ``CLAUDE.md`` / ``AGENTS.md`` into every
subagent's context before its first turn. A genuinely documentation-free
condition is therefore *structurally impossible* there, which invalidates the
ablation. Everything the model sees has to be something this package put in
front of it: our system prompt, our tool set, nothing injected.

Layout, and what each piece owns:

``backends/``
    The model. ``anthropic_api`` talks to the Messages API; ``scripted``
    replays a fixed list of tool calls from JSON. The loop only knows
    ``ModelBackend.turn(system, messages, tools) -> ModelTurn``, so an
    OpenAI-compatible or OmniLink backend is a new file, not a loop edit.

``tools/``
    Declarative, diffable tool-set manifests. ``shell`` is the byte-identical
    cross-simulator baseline (SPEC 4.1); ``shell_plus_tools`` adds OmniSim's
    harness surface, **generated from the shipped MCP server's own registry**
    so we cannot hand-tune our tool descriptions for the benchmark (SPEC 4.2).
    Every manifest dumps to JSON and hashes; the hash lands in the result row.

``budget.py``
    Max turns, max tool calls, wall-clock deadline, token budget. All four are
    enforced *and* recorded as actuals -- a budget that trips is a recorded
    stop reason, never a silent truncation (SPEC 2: budget exhaustion is a
    ``FAIL``, which requires knowing it happened).

``isolation.py``
    Per-run scratch dir, per-run ports, per-run engine log path, and a shell
    environment built from an allowlist rather than inherited -- so nothing
    leaks the answer into the agent's process (SPEC 4.3).

``trace.py``
    ``trace.jsonl``: every turn, every tool call with full arguments, every
    result, timestamps, per-turn token counts, the final message. Large
    results spill to ``tool_results/`` with a sha256 rather than being
    truncated away (SPEC 4.5: the file is never truncated).

``loop.py``
    System prompt + task prompt in, tool calls out, results back, until the
    model stops or a limit trips.

The public entry point is :func:`run_agent`; ``agents/llm.py`` is the thin
adapter that lets ``run_agentbench.py`` drive it as just another agent.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Self-bootstrapping, same convention as adapters/omnisim/harness.py: put
# `tests/benchmarks` (the `agentbench` package root) on sys.path so this
# package can be imported by absolute path -- by pytest walking up the
# __init__.py chain, or by `python -m agentbench.runner` from anywhere.
_BENCHMARKS = str(Path(__file__).resolve().parents[2])
if _BENCHMARKS not in sys.path:
    sys.path.insert(0, _BENCHMARKS)

from agentbench.runner.budget import Budget, Ledger, StopReason  # noqa: E402
from agentbench.runner.config import RunnerConfig                # noqa: E402
from agentbench.runner.loop import (                             # noqa: E402
    AgentLoop, LoopResult, run_agent)
from agentbench.runner.tools.manifest import ToolSet, ToolSpec   # noqa: E402

__all__ = [
    "AgentLoop", "Budget", "Ledger", "LoopResult", "RunnerConfig",
    "StopReason", "ToolSet", "ToolSpec", "run_agent",
]
