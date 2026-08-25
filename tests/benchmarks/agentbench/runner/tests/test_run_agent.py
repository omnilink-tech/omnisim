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

"""``agents/llm.py`` + :func:`run_agent`: the seam ``run_agentbench.py`` uses.

No simulator is involved -- these run the noop script, so what is under test is
the plumbing: env -> config -> sandbox -> tool set -> backend -> loop ->
``AgentResult``, and whether the provenance a reviewer needs (condition, both
manifest hashes, both prompt hashes, stop reason) is actually present and
persisted, not merely computed.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from agentbench.agents import llm
from agentbench.agents.base import AgentContext
from agentbench.common.trace import Trace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


@pytest.fixture()
def ctx(tmp_path):
    run_dir = tmp_path / "run"
    scratch = run_dir / "scratch"
    scratch.mkdir(parents=True)
    trace = Trace(run_dir)
    c = AgentContext(task_id="A1_husky_swarm_10", prompt="Build me a scene.",
                     scratch_dir=scratch, run_dir=run_dir,
                     repo=Path(__file__).resolve().parents[4],
                     trace=trace, deadline_s=time.time() + 300, seed=0)
    yield c
    trace.close()


def _scripted_env(monkeypatch, script="a1_noop.json", condition="shell"):
    monkeypatch.setenv("AGENTBENCH_BACKEND", "scripted")
    monkeypatch.setenv("AGENTBENCH_SCRIPT", str(SCRIPTS / script))
    monkeypatch.setenv("AGENTBENCH_CONDITION", condition)


def test_agent_result_carries_the_metrics_the_spec_requires(ctx, monkeypatch):
    _scripted_env(monkeypatch)
    res = llm.run(ctx)
    assert res.error is None
    assert res.final_message.startswith("Done")
    assert res.turns == 0 and res.tool_calls == 0      # the noop calls nothing
    assert res.tokens_in == 3500 and res.tokens_out == 60
    assert res.usd is None, "a replay has no list price"
    assert res.self_verified is False


def test_agent_result_carries_the_condition_and_both_hashes(ctx, monkeypatch):
    _scripted_env(monkeypatch)
    res = llm.run(ctx)
    a = res.artifacts
    assert a["condition"] == "shell"
    assert a["tool_set"] == "shell"
    assert len(a["tools_sha256"]) == 64
    assert len(a["manifest_sha256"]) == 64
    assert len(a["system_prompt_sha"]) == 64
    assert len(a["system_full_sha"]) == 64
    assert a["stop_reason"] == "model_stopped"
    assert a["hit_budget"] is False
    assert Path(a["tool_set_manifest"]).is_file()
    assert Path(a["system_prompt"]).is_file()


def test_provenance_is_persisted_not_just_returned(ctx, monkeypatch):
    """Regression: the prompt hashes used to be attached AFTER the loop had
    already written runner_result.json, so the audit file had empty fields."""
    _scripted_env(monkeypatch)
    res = llm.run(ctx)
    saved = json.loads((Path(ctx.run_dir) / "runner_result.json")
                       .read_text(encoding="utf-8"))
    assert saved["system_prompt_sha"] == res.artifacts["system_prompt_sha"]
    assert saved["system_full_sha"] == res.artifacts["system_full_sha"]
    assert saved["system_prompt_sha"], "must not be empty"
    assert saved["manifest_sha256"] == res.artifacts["manifest_sha256"]


def test_the_condition_selects_the_surface(ctx, monkeypatch):
    _scripted_env(monkeypatch, condition="shell+tools")
    res = llm.run(ctx)
    assert res.artifacts["condition"] == "shell+tools"
    assert res.artifacts["tool_set"] == "shell_plus_tools"
    manifest = json.loads(Path(res.artifacts["tool_set_manifest"])
                          .read_text(encoding="utf-8"))
    assert manifest["n_tools"] == 22


def test_pinned_condition_agents_ignore_a_stale_env_var(ctx, monkeypatch):
    _scripted_env(monkeypatch, condition="shell+tools")
    res = llm.run_shell_only(ctx)
    assert res.artifacts["condition"] == "shell"
    # ...and the env var is restored, so the next cell is not affected.
    import os
    assert os.environ["AGENTBENCH_CONDITION"] == "shell+tools"


def test_the_system_prompt_is_the_shared_file_plus_generated_blocks(
        ctx, monkeypatch):
    _scripted_env(monkeypatch)
    res = llm.run(ctx)
    text = Path(res.artifacts["system_prompt"]).read_text(encoding="utf-8")
    from agentbench.runner.prompts import base_system_prompt
    assert base_system_prompt() in text, "the shared prompt must be verbatim"
    assert "## Your tools" in text and "run_shell" in text
    assert "## Your environment" in text
    # The ablation depends on this: nothing about the repo's documentation,
    # and no task hints, may be injected here.
    low = text.lower()
    for leak in ("husky", "newtonsolver", "urdfrobot", "agents.md",
                 "claude.md", "supervisor true"):
        assert leak not in low, "%r leaked into the system prompt" % leak


def test_missing_credential_is_reported_not_faked(ctx, monkeypatch):
    monkeypatch.setenv("AGENTBENCH_BACKEND", "anthropic")
    monkeypatch.setenv("AGENTBENCH_CONDITION", "shell")
    from agentbench.runner.backends import anthropic_api
    monkeypatch.setattr(anthropic_api, "credential_source", lambda: None)
    res = llm.run(ctx)
    assert res.error and "ANTHROPIC_API_KEY" in res.error
    # Nothing was measured, so nothing may be reported (SPEC 3.1).
    assert res.tokens_in is None and res.tokens_out is None and res.usd is None
    assert res.turns == 0 and res.tool_calls == 0
    assert any("SKIPPED" in d for d in res.deviations)


def test_a_bad_condition_fails_before_the_run(ctx, monkeypatch):
    monkeypatch.setenv("AGENTBENCH_CONDITION", "shell+magic")
    res = llm.run(ctx)
    assert res.error and "unknown condition" in res.error


def test_scripted_backend_without_a_script_is_an_error(ctx, monkeypatch):
    monkeypatch.setenv("AGENTBENCH_BACKEND", "scripted")
    monkeypatch.delenv("AGENTBENCH_SCRIPT", raising=False)
    monkeypatch.setenv("AGENTBENCH_CONDITION", "shell")
    res = llm.run(ctx)
    assert res.error and "script" in res.error.lower()


def test_an_operator_wall_cap_only_tightens_the_deadline(ctx, monkeypatch):
    """AGENTBENCH_MAX_WALL_S must never buy more time than 3 x par."""
    _scripted_env(monkeypatch, script="limit_deadline.json")
    monkeypatch.setenv("AGENTBENCH_MAX_WALL_S", "1.5")
    monkeypatch.setenv("AGENTBENCH_MAX_TURNS", "0")
    monkeypatch.setenv("AGENTBENCH_MAX_TOOL_CALLS", "0")
    res = llm.run(ctx)
    assert res.artifacts["stop_reason"] == "deadline"
    assert res.artifacts["agent_wall_s"] < 10.0
    # A cap LONGER than the context's deadline must not extend it.
    from agentbench.runner.config import RunnerConfig
    cfg = RunnerConfig.from_env()
    cfg.max_wall_s = 10 ** 6
    ctx.deadline_s = time.time() + 5
    from agentbench.runner.loop import run_agent
    import contextlib
    with contextlib.suppress(Exception):
        run_agent(ctx, cfg)
    assert cfg.deadline_s <= time.time() + 6


# -- registry wiring -------------------------------------------------------

def test_llm_agents_are_registered_but_not_in_agent_all():
    from agentbench import agents as registry
    for name in ("llm", "llm_shell", "llm_tools"):
        assert ("A1_husky_swarm_10", name) in registry.REGISTRY
        assert name in registry.AGENT_NAMES
        # ...but a bare `--agent all` must stay free and deterministic.
        assert name not in registry.agents_for("A1_husky_swarm_10")


def test_llm_agents_carry_no_pass_expectation():
    """An LLM run's outcome is unknown by construction (like `external`)."""
    from agentbench import agents as registry
    entry = registry.get("A1_husky_swarm_10", "llm")
    assert entry["expect_pass"] is None
    assert entry["expect_failures"] is None
