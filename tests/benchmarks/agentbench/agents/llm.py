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

"""``--agent llm``: drive the API-driven runner as just another agent.

This is the whole adapter. ``run_agentbench.py`` calls ``fn(ctx) ->
AgentResult``; :mod:`agentbench.runner` needs a sandbox, a tool set, a backend
and a budget; everything in between is env-var plumbing and honest mapping of
one result object onto the other.

**Why a separate agent instead of reusing ``external``.** ``agents/external.py``
grades an artifact produced somewhere else and explicitly gives no credit for
the copy -- it cannot record turns, tokens, a stop reason, or a trace of how the
artifact came to be. Those are exactly the columns SPEC 3.1 requires, and they
only exist if the loop that produced the artifact is inside the harness. So the
LLM run is a first-class agent whose ``AgentResult`` carries real metrics, and
the grader still measures the artifact independently.

Configuration is environment (see :mod:`agentbench.runner.config`); the two that
matter most:

    AGENTBENCH_CONDITION   shell | shell+tools      -- the ablation axis
    AGENTBENCH_BACKEND     anthropic | scripted     -- model or replay

    # a real model run
    ANTHROPIC_API_KEY=... AGENTBENCH_CONDITION=shell \\
      python run_agentbench.py --tasks A1 --agent llm

    # the same loop, no credential, replaying a known-good solution
    AGENTBENCH_BACKEND=scripted \\
      AGENTBENCH_SCRIPT=runner/scripts/a1_oracle_replay.json \\
      python run_agentbench.py --tasks A1 --agent llm

**Missing credential.** SPEC 3.3 has a ``SKIPPED`` outcome for "no credential
for this cell", but an agent cannot set the outcome -- the grader owns it. So
this adapter returns a result with ``error`` set and a final message naming the
exact env var, which grades as a ``FAIL`` with ``progress=0``. That is
*reported* here rather than silently mislabelled: mapping a missing credential
to ``SKIPPED`` needs a small change in ``run_agentbench.py``, which this agent
does not own.
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents.base import AgentResult              # noqa: E402
from agentbench.runner.backends import BackendUnavailable    # noqa: E402
from agentbench.runner.config import RunnerConfig            # noqa: E402
from agentbench.runner.loop import run_agent                 # noqa: E402


def run(ctx):
    res = AgentResult()
    try:
        cfg = RunnerConfig.from_env()
    except Exception as exc:                                # noqa: BLE001
        msg = "runner configuration is invalid: %s" % exc
        ctx.trace.event("llm_config_error", error=msg)
        ctx.trace.final(msg, False)
        res.final_message = msg
        res.error = msg
        return res

    ctx.trace.event("llm_agent_start", condition=cfg.spec_condition,
                    backend=cfg.backend, model=cfg.model,
                    script=cfg.script, scaffold=cfg.scaffold)

    try:
        loop = run_agent(ctx, cfg)
    except BackendUnavailable as exc:
        # No credential / no SDK. Nothing was measured, and the row must not
        # pretend otherwise: metrics stay None (SPEC 3.1).
        msg = ("cannot run the %s backend: %s" % (cfg.backend, exc))
        ctx.trace.event("llm_backend_unavailable", backend=cfg.backend,
                        error=str(exc))
        ctx.trace.final(msg, False)
        res.final_message = msg
        res.error = msg
        res.deviations.append(
            "no credential for backend %r -- SPEC 3.3 would call this "
            "SKIPPED, but an agent cannot set the outcome, so this row grades "
            "as FAIL/progress=0. See agents/llm.py." % cfg.backend)
        res.artifacts["condition"] = cfg.spec_condition
        res.artifacts["backend"] = cfg.backend
        return res
    except Exception as exc:                                # noqa: BLE001
        msg = "runner raised: %s: %s" % (type(exc).__name__, exc)
        ctx.trace.event("llm_runner_error", traceback=traceback.format_exc())
        ctx.trace.final(msg, False)
        res.final_message = msg
        res.error = msg
        return res

    # -- map LoopResult -> AgentResult --------------------------------------
    led = loop.ledger
    res.final_message = loop.final_message
    res.self_verified = loop.self_verified
    res.turns = led.turns
    res.tool_calls = led.tool_calls
    res.tokens_in = led.tokens_in
    res.tokens_out = led.tokens_out
    res.tokens_cache_read = led.tokens_cache_read
    res.usd = loop.usd
    res.deviations.extend(loop.deviations)
    res.artifacts.update({
        # The ablation axis and the exact surface it was given -- a reviewer
        # can diff the manifest and recompute the hash.
        "condition": cfg.spec_condition,
        "tool_set": loop.tool_set,
        "tools_sha256": loop.tools_sha256,
        "manifest_sha256": loop.manifest_sha256,
        "system_prompt_sha": loop.system_prompt_sha,
        "system_full_sha": loop.system_full_sha,
        "scaffold_sha": cfg.scaffold,
        # Why the loop ended. `model_stopped` is the only clean value.
        "stop_reason": loop.stop_reason,
        "stop_detail": loop.stop_detail,
        "hit_budget": loop.hit_budget,
        "model_calls": led.model_calls,
        "agent_wall_s": round(led.elapsed_s, 3),
        "port_survivors": loop.port_survivors,
    })
    res.artifacts.update(loop.artifacts)
    res.artifacts["backend_describe"] = loop.backend
    if loop.error:
        res.error = loop.error
    return res


def run_shell_only(ctx):
    """``--agent llm_shell``: force the documentation-free ``shell`` condition.

    Registered separately so an A/B pair cannot be produced by forgetting to
    set ``AGENTBENCH_CONDITION`` -- the condition is the measurement, and it
    should not be silently inherited from a stale environment variable.
    """
    cfg = RunnerConfig.from_env(condition="shell")
    return _run_with(ctx, cfg)


def run_full_surface(ctx):
    """``--agent llm_tools``: force the ``shell+tools`` condition."""
    cfg = RunnerConfig.from_env(condition="shell_plus_tools")
    return _run_with(ctx, cfg)


def _run_with(ctx, cfg):
    import os
    prev = os.environ.get("AGENTBENCH_CONDITION")
    os.environ["AGENTBENCH_CONDITION"] = cfg.condition
    try:
        return run(ctx)
    finally:
        if prev is None:
            os.environ.pop("AGENTBENCH_CONDITION", None)
        else:
            os.environ["AGENTBENCH_CONDITION"] = prev
