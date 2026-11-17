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

"""Runner configuration, and how a run is parameterised from the outside.

``run_agentbench.py`` calls agents as ``fn(ctx) -> AgentResult`` with no
per-agent arguments, so the LLM agent's knobs arrive as environment variables.
Everything here is recorded into the row, because a number whose conditions are
not recorded is not a number (SPEC 3).

    AGENTBENCH_CONDITION      shell | shell+tools          (default: shell)
    AGENTBENCH_BACKEND        anthropic | scripted         (default: anthropic)
    AGENTBENCH_MODEL          model id                     (backend default)
    AGENTBENCH_SCRIPT         path to a replay script      (scripted backend)
    AGENTBENCH_MAX_TURNS      default 60   (SPEC 2)
    AGENTBENCH_MAX_TOOL_CALLS default 240
    AGENTBENCH_MAX_TOKENS_IN  default 400000 (SPEC 2)
    AGENTBENCH_MAX_TOKENS_OUT default 60000  (SPEC 2)
    AGENTBENCH_TEMPERATURE    default 0 (dropped on models that reject it)
    AGENTBENCH_STREAM         1 to stream the model call
    AGENTBENCH_THINKING       adaptive | disabled          (provider default)
    AGENTBENCH_EFFORT         low|medium|high|xhigh|max    (provider default)
    AGENTBENCH_TOOL_TIMEOUT_S per-tool-call ceiling, default 180
    AGENTBENCH_MAX_RESULT_BYTES  trace spill threshold, default 32768
    AGENTBENCH_EXTRA_PARAMS   JSON merged into the model request
    AGENTBENCH_READ_DENY      repo-relative paths (``:`` or ``;`` separated)
                              made invisible to the agent's file tools, or a
                              preset name (default: empty -- no ablation)

Note the asymmetry with :mod:`.isolation`: the runner reads ``AGENTBENCH_*``
from its own environment, and strips every one of them from the *agent's*
environment. That is deliberate -- ``AGENTBENCH_EXTERNAL_ARTIFACT`` points at a
finished world file, and an agent that can read it is not solving the task.

The docs-ablation cell (SPEC 6.3; agent-edge-validation-plan Phase R item 6)
----------------------------------------------------------------------------

``AGENTBENCH_READ_DENY=docs_ablation`` selects the named preset
(:data:`~agentbench.runner.isolation.READ_DENY_PRESETS`)::

    AGENTS.md  CLAUDE.md  docs/developer

Denied paths and everything under them become invisible AND unreadable to the
agent's file tools, and a read attempt returns the same error as a
nonexistent path -- so the ablated agent cannot infer what is being hidden
(the property is asserted by ``runner/tests/test_docs_ablation.py``). The
question the cell answers: if OmniSim's result collapses without the manual,
the honest headline is "our documentation is the moat", not "our API is the
moat".

Inclusion rationale -- the target is the agent-facing MANUAL, not the code:

* ``AGENTS.md`` + ``docs/developer/`` -- exactly the pre-registered set (SPEC
  6.3, plan item 6): the manual that "literally hands over the non-obvious
  answers" and the developer-doc tree it links into.
* ``CLAUDE.md`` -- included beyond the pre-registered pair because it is the
  manual's loader stub (its content is ``@AGENTS.md``, and the §0 ablation
  leak was precisely the CLAUDE.md -> AGENTS.md auto-injection); ablating the
  manual while leaving its alias readable would be a hole, not a cell.
* ``DEMOS.md`` -- deliberately KEPT readable: it is a world catalogue whose
  facts (paths, names) are recoverable by ``ls``; it carries none of the
  non-obvious engine answers the §0 agent credited.
* ``scripts/harness/README.md`` (and every other subsystem README /
  docstring) -- deliberately KEPT readable. Arguable -- it documents the
  agent surface -- but in-tree subsystem READMEs are ordinary open-source
  hygiene that every competitor's repo also ships un-ablated, so denying
  them would construct a "repo with no docs at all" condition and the cell
  would stop isolating the differentiated manual's contribution.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

from agentbench.runner.budget import (
    DEFAULT_MAX_TOKENS_IN, DEFAULT_MAX_TOKENS_OUT, DEFAULT_MAX_TOOL_CALLS,
    DEFAULT_MAX_TURNS, Budget)
from agentbench.runner.isolation import parse_read_deny
from agentbench.runner.tools import normalize_condition


def _env_int(name, default):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return default
    return int(raw)


def _env_float(name, default):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return default
    return float(raw)


def _env_bool(name, default=False):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


@dataclass
class RunnerConfig:
    condition: str = "shell"
    backend: str = "anthropic"
    model: str | None = None
    script: str | None = None
    temperature: float | None = 0.0
    stream: bool = False
    thinking: str | None = None
    effort: str | None = None
    max_turns: int = DEFAULT_MAX_TURNS
    max_tool_calls: int = DEFAULT_MAX_TOOL_CALLS
    max_tokens_in: int = DEFAULT_MAX_TOKENS_IN
    max_tokens_out: int = DEFAULT_MAX_TOKENS_OUT
    tool_timeout_s: float = 180.0
    deadline_s: float = 0.0
    # An extra wall-clock cap on top of the task's own timeout (3 x par). Only
    # ever tightens the deadline -- it cannot buy a run more time than the SPEC
    # allows. Exists so the deadline path can be exercised, and so an operator
    # can bound a run without editing task metadata.
    max_wall_s: float = 0.0
    max_result_bytes: int = 32 * 1024
    extra_params: dict = field(default_factory=dict)
    # Parsed AGENTBENCH_READ_DENY (tuple of repo-relative paths; empty = no
    # ablation). Recorded so the row can be attributed to its cell. NOTE the
    # enforcement itself lives in Sandbox.create, which reads the same env
    # var -- so for the env-driven path the recorded and enforced values
    # agree by construction; a programmatic override of this field alone
    # does NOT reach the sandbox (see the report note in Sandbox.create).
    read_deny: tuple = ()
    # Provenance of the scaffold itself, so two rows can be compared.
    scaffold: str = "agentbench/runner/v1"

    # -- construction ----------------------------------------------------
    @classmethod
    def from_env(cls, **overrides):
        raw = os.environ.get("AGENTBENCH_EXTRA_PARAMS") or ""
        cfg = cls(
            condition=normalize_condition(
                os.environ.get("AGENTBENCH_CONDITION") or "shell"),
            backend=(os.environ.get("AGENTBENCH_BACKEND")
                     or "anthropic").strip().lower(),
            model=os.environ.get("AGENTBENCH_MODEL") or None,
            script=os.environ.get("AGENTBENCH_SCRIPT") or None,
            temperature=_env_float("AGENTBENCH_TEMPERATURE", 0.0),
            stream=_env_bool("AGENTBENCH_STREAM", False),
            thinking=os.environ.get("AGENTBENCH_THINKING") or None,
            effort=os.environ.get("AGENTBENCH_EFFORT") or None,
            max_turns=_env_int("AGENTBENCH_MAX_TURNS", DEFAULT_MAX_TURNS),
            max_tool_calls=_env_int("AGENTBENCH_MAX_TOOL_CALLS",
                                    DEFAULT_MAX_TOOL_CALLS),
            max_tokens_in=_env_int("AGENTBENCH_MAX_TOKENS_IN",
                                   DEFAULT_MAX_TOKENS_IN),
            max_tokens_out=_env_int("AGENTBENCH_MAX_TOKENS_OUT",
                                    DEFAULT_MAX_TOKENS_OUT),
            tool_timeout_s=_env_float("AGENTBENCH_TOOL_TIMEOUT_S", 180.0),
            max_wall_s=_env_float("AGENTBENCH_MAX_WALL_S", 0.0),
            max_result_bytes=_env_int("AGENTBENCH_MAX_RESULT_BYTES",
                                      32 * 1024),
            extra_params=(json.loads(raw) if raw.strip() else {}),
            read_deny=parse_read_deny(
                os.environ.get("AGENTBENCH_READ_DENY") or ""),
        )
        for k, v in overrides.items():
            if v is not None:
                setattr(cfg, k, v)
        return cfg

    # -- derived ---------------------------------------------------------
    def budget(self) -> Budget:
        return Budget(max_turns=self.max_turns,
                      max_tool_calls=self.max_tool_calls,
                      max_tokens_in=self.max_tokens_in,
                      max_tokens_out=self.max_tokens_out,
                      deadline_s=self.deadline_s,
                      tool_timeout_s=self.tool_timeout_s)

    def backend_kwargs(self, variables=None) -> dict:
        if self.backend == "scripted":
            return {"script": self.script, "variables": variables or {},
                    "model": self.model}
        return {"model": self.model, "temperature": self.temperature,
                "stream": self.stream, "thinking": self.thinking,
                "effort": self.effort, "extra_params": self.extra_params}

    def as_dict(self):
        d = dict(self.__dict__)
        d["spec_condition"] = self.spec_condition
        return d

    @property
    def spec_condition(self) -> str:
        from agentbench.runner.tools import SPEC_CONDITION_NAME
        return SPEC_CONDITION_NAME.get(self.condition, self.condition)
