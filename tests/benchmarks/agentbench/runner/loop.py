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

"""The agent loop (AgentBench SPEC 4.2): sim-agnostic, backend-agnostic.

    system prompt + task prompt -> model -> tool calls -> results -> repeat

until the model stops or a limit trips. The loop knows three things: a backend
with a ``turn()`` method, a tool set with handlers, and a budget. It does not
know which simulator, which task, or which provider -- which is what lets one
loop serve every cell of the comparison.

What the loop guarantees, and why each one is here:

* **Every exit is named.** There is no path out of :meth:`AgentLoop.run` that
  does not set a :class:`~agentbench.runner.budget.StopReason`. SPEC 2 makes
  budget exhaustion a ``FAIL`` rather than an ``INVALID``, which is only
  meaningful if the row can say *which* budget.
* **Actuals for all four limits.** Turns, tool calls, tokens and wall clock are
  recorded whether or not they tripped, so a cell that finished comfortably and
  a cell that finished on the last turn are distinguishable.
* **Provider content is echoed verbatim.** ``ModelTurn.assistant_content`` goes
  back into the history unmodified -- thinking blocks on current Claude models
  must replay unchanged, and reconstructing the assistant turn from text +
  tool calls would silently break that.
* **Nothing is injected.** The first user message is the task prompt, byte for
  byte. No reminders, no scaffolding, no repo documentation. That is the whole
  reason this package exists.
"""

from __future__ import annotations

import json
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path

from agentbench.runner.backends import BackendExhausted, get_backend
from agentbench.runner.budget import Budget, Ledger, StopReason
from agentbench.runner.config import RunnerConfig
from agentbench.runner.isolation import Sandbox
from agentbench.runner.trace import RunnerTrace
from agentbench.runner.tools import ToolResult, get_tool_set

# `self_verified` (SPEC 3.1) is a heuristic over the trace, and the rule is
# written down here so it can be argued with rather than trusted:
#
#   self_verified == there exists a call to a READ-BACK tool strictly after the
#                    last call to a STATE-CHANGING tool.
#
# So "authored a world and then ran it" verifies; "authored a world and stopped"
# does not. `run_shell` counts as read-back because running the simulator and
# reading the numbers is the verification we want to observe -- at the cost of
# also counting a shell command that only wrote a file. It is reported, never
# scored: SPEC 2.3 is explicit that self_verified is not part of any pass
# criterion.
MUTATING_TOOLS = frozenset({"write_file", "load_world", "sim_reset",
                            "look_at", "frame", "orbit"})
READBACK_TOOLS = frozenset({
    "run_shell", "read_file", "list_dir",
    "harness_status", "get_scene_tree", "get_scene_node", "get_viewpoint",
    "visible", "screenshot", "render_stats", "sim_step", "get_events",
    "list_robots", "get_robot_joints", "get_contacts", "get_diagnostics"})


@dataclass
class LoopResult:
    """Everything the row needs from one agent run."""

    final_message: str = ""
    stop_reason: str = StopReason.MODEL_STOPPED
    stop_detail: str = ""
    self_verified: bool = False
    ledger: Ledger | None = None
    usd: float | None = None
    tool_set: str = ""
    tools_sha256: str = ""
    manifest_sha256: str = ""
    system_prompt_sha: str = ""
    system_full_sha: str = ""
    backend: dict = field(default_factory=dict)
    sandbox: dict = field(default_factory=dict)
    deviations: list = field(default_factory=list)
    artifacts: dict = field(default_factory=dict)
    error: str | None = None
    port_survivors: list = field(default_factory=list)

    @property
    def hit_budget(self) -> bool:
        return StopReason.is_budget(self.stop_reason)

    def as_dict(self):
        d = {k: v for k, v in self.__dict__.items() if k != "ledger"}
        d["metrics"] = self.ledger.as_dict() if self.ledger else None
        d["hit_budget"] = self.hit_budget
        return d


class AgentLoop:
    def __init__(self, *, backend, tool_set, sandbox, trace, budget=None,
                 system="", prompt="", run_dir=None, system_shas=None,
                 extra_artifacts=None):
        self.backend = backend
        self.tool_set = tool_set
        self.sandbox = sandbox
        self.trace = trace
        self.budget = budget or Budget()
        self.system = system
        self.prompt = prompt
        self.run_dir = Path(run_dir or sandbox.run_dir)
        # Passed IN rather than patched onto the result afterwards: the loop
        # writes runner_result.json and the `runner_done` trace event itself,
        # and a field set after that write is a field missing from the audit.
        self.system_shas = dict(system_shas or {})
        self.extra_artifacts = dict(extra_artifacts or {})
        self.ledger = Ledger()
        self.messages = []
        self.deviations = list(getattr(sandbox, "deviations", []) or [])
        self._tool_history = []          # (index, name) in execution order

    # -- helpers ---------------------------------------------------------
    def _tool_defs(self):
        return self.tool_set.as_anthropic_tools()

    def _self_verified(self) -> bool:
        last_mutate = -1
        for i, name in self._tool_history:
            if name in MUTATING_TOOLS:
                last_mutate = i
        return any(i > last_mutate and name in READBACK_TOOLS
                   for i, name in self._tool_history)

    def _execute(self, call) -> ToolResult:
        spec = self.tool_set.get(call.name)
        if spec is None or spec.handler is None:
            return ToolResult(
                text=("error: no such tool %r. Available: %s"
                      % (call.name, ", ".join(self.tool_set.names))),
                is_error=True)
        timeout = self.ledger.tool_timeout(self.budget)
        t0 = time.perf_counter()
        try:
            out = spec.handler(call.arguments, timeout)
        except Exception as exc:                          # noqa: BLE001
            # A tool crash is the agent's problem to route around, not the
            # benchmark's to hide: hand the model the error and keep going.
            out = ToolResult(text="%s: %s" % (type(exc).__name__, exc),
                             is_error=True,
                             meta={"traceback": traceback.format_exc()})
        dt = time.perf_counter() - t0
        if not isinstance(out, ToolResult):
            out = ToolResult(text=str(out))
        out.meta.setdefault("duration_s", round(dt, 3))
        return out

    def _tool_result_block(self, call, out: ToolResult):
        return {"type": "tool_result", "tool_use_id": call.id,
                "content": out.text or "(no output)",
                "is_error": bool(out.is_error)}

    # -- the loop --------------------------------------------------------
    def run(self) -> LoopResult:
        manifest_path = self.run_dir / ("tool_set.%s.json" % self.tool_set.id)
        manifest_sha = self.tool_set.dump(manifest_path)
        self.trace.event(
            "runner_start", tool_set=self.tool_set.id,
            n_tools=len(self.tool_set), tools_sha256=self.tool_set.tools_sha256,
            manifest_sha256=manifest_sha, manifest=str(manifest_path),
            backend=self.backend.describe(), budget=self.budget.as_dict(),
            sandbox=self.sandbox.as_dict(),
            system_prompt_chars=len(self.system))

        self.messages = [{"role": "user", "content": self.prompt}]
        final_text = ""
        stop = None
        detail = ""

        while True:
            stop = self.ledger.check(self.budget)
            if stop:
                detail = "tripped before the model call"
                break

            try:
                turn = self.backend.turn(self.system, self.messages,
                                         self._tool_defs())
            except BackendExhausted as exc:
                stop, detail = StopReason.SCRIPT_EXHAUSTED, str(exc)
                break
            except Exception as exc:                      # noqa: BLE001
                stop, detail = StopReason.BACKEND_ERROR, "%s: %s" % (
                    type(exc).__name__, exc)
                self.trace.event("backend_error",
                                 traceback=traceback.format_exc())
                break

            self.ledger.model_calls += 1
            self.ledger.add_usage(turn.usage)
            if turn.wants_tools:
                # SPEC 3.1: a turn is an assistant message with >= 1 tool call.
                self.ledger.turns += 1
            self.trace.turn(turn.text, usage=turn.usage,
                            stop_reason=turn.stop_reason,
                            tool_names=[c.name for c in turn.tool_calls],
                            raw=turn.raw)
            if turn.text:
                final_text = turn.text

            if turn.stop_reason == "refusal":
                stop = StopReason.REFUSAL
                detail = json.dumps((turn.raw or {}).get("stop_details"))
                break
            if turn.stop_reason == "max_tokens":
                self.deviations.append(
                    "a model response was truncated by max_tokens (%s); the "
                    "turn's output is incomplete"
                    % getattr(self.backend, "max_tokens", "?"))

            over = self.ledger.check_tokens(self.budget)
            if over:
                stop, detail = over, "token budget reached after turn %d" % (
                    self.ledger.model_calls)
                break

            if not turn.wants_tools:
                if turn.stop_reason == "pause_turn":
                    # A server-side tool paused mid-turn: re-send so the
                    # provider resumes. No extra user message -- the provider
                    # detects the trailing server_tool_use itself.
                    self.messages.append(
                        {"role": "assistant",
                         "content": turn.assistant_content})
                    continue
                stop, detail = StopReason.MODEL_STOPPED, (
                    turn.stop_reason or "")
                break

            self.messages.append({"role": "assistant",
                                  "content": turn.assistant_content})

            results = []
            for call in turn.tool_calls:
                gate = self.ledger.check_tool_call(self.budget)
                if gate:
                    stop, detail = gate, (
                        "tripped with %d tool call(s) from turn %d unexecuted"
                        % (len(turn.tool_calls) - len(results),
                           self.ledger.model_calls))
                    for skipped in turn.tool_calls[len(results):]:
                        self.trace.event("tool_skipped", name=skipped.name,
                                         reason=gate)
                    break
                self.ledger.tool_calls += 1
                out = self._execute(call)
                self._tool_history.append((self.ledger.tool_calls, call.name))
                self.trace.tool(call.name, call.arguments,
                                result=(out.data if out.data is not None
                                        else out.text),
                                error=(out.text if out.is_error else None),
                                dt_s=out.meta.get("duration_s"),
                                extra={"is_error": out.is_error,
                                       "model_text_chars": len(out.text)})
                results.append(self._tool_result_block(call, out))
            if stop:
                break
            self.messages.append({"role": "user", "content": results})

        # -- close out ---------------------------------------------------
        self.ledger.stop_reason = stop or StopReason.MODEL_STOPPED
        self.ledger.stop_detail = detail
        if StopReason.is_budget(self.ledger.stop_reason):
            self.trace.limit(self.ledger.stop_reason, detail, self.ledger,
                             self.budget)
            self.deviations.append(
                "budget exhausted: %s (%s)" % (self.ledger.stop_reason,
                                               detail))
        elif self.ledger.stop_reason != StopReason.MODEL_STOPPED:
            self.deviations.append("stopped: %s (%s)"
                                   % (self.ledger.stop_reason, detail))

        verified = self._self_verified()
        if not final_text:
            final_text = ("[no final message: the run ended with %s]"
                          % self.ledger.stop_reason)
        self.trace.final(final_text, verified)

        res = LoopResult(
            final_message=final_text, stop_reason=self.ledger.stop_reason,
            stop_detail=detail, self_verified=verified, ledger=self.ledger,
            usd=self.backend.usd(self.ledger.tokens_in, self.ledger.tokens_out,
                                 self.ledger.tokens_cache_read),
            tool_set=self.tool_set.id,
            tools_sha256=self.tool_set.tools_sha256,
            manifest_sha256=manifest_sha,
            system_prompt_sha=self.system_shas.get("base_sha", ""),
            system_full_sha=self.system_shas.get("full_sha", ""),
            backend=self.backend.describe(),
            sandbox=self.sandbox.as_dict(),
            deviations=self.deviations,
            artifacts=dict(
                {"trace": str(self.trace.path),
                 "tool_set_manifest": str(manifest_path),
                 "leak_suspect_tool_calls": getattr(
                     self.trace, "leak_suspects", 0),
                 # The Ledger accounts cache writes but AgentResult has no
                 # field for them, so this artifact is how the figure reaches
                 # rows.jsonl's metrics (agents/llm.py folds artifacts in;
                 # run_agentbench.py reads it -- plan §0.2, Phase R item 2).
                 "tokens_cache_write": self.ledger.tokens_cache_write},
                **self.extra_artifacts),
            port_survivors=self.sandbox.teardown())
        if res.port_survivors:
            # SPEC 4.3: something the agent started outlived the run. The
            # caller should treat this as INVALID rather than score it.
            res.deviations.append(
                "ports still bound after the run: %s -- a service the agent "
                "started outlived it" % res.port_survivors)
        self.trace.event("runner_done", **res.as_dict())
        (self.run_dir / "runner_result.json").write_text(
            json.dumps(res.as_dict(), indent=2, default=str), encoding="utf-8")
        return res


def run_agent(ctx, config: RunnerConfig | None = None) -> LoopResult:
    """Build the sandbox, tool set and backend for ``ctx``, then run the loop.

    ``ctx`` is an ``agents/base.py`` ``AgentContext``, so this is the one
    function ``agents/llm.py`` needs -- and the one entry point a future
    ``run_agentbench.py --agent llm --condition ...`` would call.
    """
    from agentbench.runner import prompts as prompt_mod

    cfg = config or RunnerConfig.from_env()
    if not cfg.deadline_s:
        cfg.deadline_s = float(getattr(ctx, "deadline_s", 0.0) or 0.0)
    if cfg.max_wall_s:
        capped = time.time() + cfg.max_wall_s
        # min(), never max(): an operator cap may only tighten the task's
        # timeout, never extend it past 3 x par.
        cfg.deadline_s = min(cfg.deadline_s or capped, capped)

    sandbox = Sandbox.create(ctx.run_dir, ctx.scratch_dir,
                             repo=getattr(ctx, "repo", None) or Path.cwd(),
                             read_deny=cfg.read_deny)
    tool_set = get_tool_set(cfg.condition, sandbox)
    initial = sorted(p.name for p in Path(ctx.scratch_dir).iterdir()) \
        if Path(ctx.scratch_dir).is_dir() else []
    composed = prompt_mod.compose(tool_set, sandbox, initial_files=initial)
    backend = get_backend(cfg.backend, **cfg.backend_kwargs(
        variables=sandbox.variables()))
    rtrace = RunnerTrace(ctx.trace, ctx.run_dir, guard=sandbox.guard,
                         max_result_bytes=cfg.max_result_bytes)
    rtrace.event("runner_config", **cfg.as_dict())

    # Written before the run so the exact bytes that were sent are on disk even
    # if the run dies mid-loop.
    prompt_path = Path(ctx.run_dir) / "system_prompt.txt"
    prompt_path.write_text(composed["text"], encoding="utf-8")

    loop = AgentLoop(backend=backend, tool_set=tool_set, sandbox=sandbox,
                     trace=rtrace, budget=cfg.budget(),
                     system=composed["text"], prompt=ctx.prompt,
                     run_dir=ctx.run_dir, system_shas=composed,
                     extra_artifacts={"system_prompt": str(prompt_path)})
    try:
        return loop.run()
    finally:
        backend.close()
