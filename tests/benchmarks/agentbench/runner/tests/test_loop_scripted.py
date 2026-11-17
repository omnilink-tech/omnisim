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

"""The loop, driven by the scripted backend.

These are the tests that make the runner trustworthy without a credential:
every limit is shown to produce its own named stop reason (SPEC 2 makes budget
exhaustion a *result*, so it has to be identifiable), the tool-call/tool-result
id round trip is checked, and a script that asks for a tool the condition does
not grant fails loudly instead of quietly succeeding.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from agentbench.common.trace import Trace
from agentbench.runner.backends import get_backend
from agentbench.runner.budget import Budget, StopReason
from agentbench.runner.loop import AgentLoop
from agentbench.runner.trace import RunnerTrace
from agentbench.runner.tools import get_tool_set


def write_script(tmp_path, name, turns):
    p = tmp_path / ("%s.json" % name)
    p.write_text(json.dumps({"schema": "agentbench/script/v1", "name": name,
                             "description": "unit test", "turns": turns}),
                 encoding="utf-8")
    return p


def build_loop(tmp_path, script, *, condition="shell", budget=None,
               prompt="do the thing"):
    from agentbench.runner.isolation import Sandbox
    run_dir = tmp_path / "run"
    sandbox = Sandbox.create(run_dir, run_dir / "scratch", ports=False)
    tool_set = get_tool_set(condition, sandbox)
    backend = get_backend("scripted", script=script,
                          variables=sandbox.variables())
    trace = Trace(run_dir)
    rtrace = RunnerTrace(trace, run_dir, guard=sandbox.guard)
    loop = AgentLoop(backend=backend, tool_set=tool_set, sandbox=sandbox,
                     trace=rtrace, budget=budget or Budget(deadline_s=0),
                     system="sys", prompt=prompt, run_dir=run_dir)
    return loop, sandbox, trace


CHEAP = {"name": "list_dir", "arguments": {"path": "."}}


# -- clean completion ------------------------------------------------------

def test_a_script_that_stops_ends_as_model_stopped(tmp_path):
    s = write_script(tmp_path, "clean", [
        {"text": "looking", "tool_calls": [CHEAP],
         "usage": {"tokens_in": 100, "tokens_out": 10}},
        {"text": "all done", "usage": {"tokens_in": 120, "tokens_out": 20}}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.MODEL_STOPPED
    assert res.hit_budget is False
    assert res.final_message == "all done"
    # A turn is an assistant message with >= 1 tool call (SPEC 3.1); the
    # text-only closing message is a model call but not a turn.
    assert res.ledger.turns == 1
    assert res.ledger.model_calls == 2
    assert res.ledger.tool_calls == 1
    assert (res.ledger.tokens_in, res.ledger.tokens_out) == (220, 30)


def test_scripted_runs_have_no_usd_even_though_tokens_are_counted(tmp_path):
    s = write_script(tmp_path, "priced", [
        {"text": "x", "usage": {"tokens_in": 10**6, "tokens_out": 10**6}}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    assert res.ledger.tokens_in == 10**6
    assert res.usd is None, "a replay has no list price; usd must be null"


def test_manifest_and_result_are_written_next_to_the_run(tmp_path):
    s = write_script(tmp_path, "artifacts", [{"text": "done"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    manifest = Path(res.artifacts["tool_set_manifest"])
    assert manifest.is_file()
    dumped = json.loads(manifest.read_text(encoding="utf-8"))
    assert dumped["manifest_sha256"] == res.manifest_sha256
    assert dumped["tools_sha256"] == res.tools_sha256
    assert (tmp_path / "run" / "runner_result.json").is_file()


# -- the four limits -------------------------------------------------------

def test_max_turns_is_recorded_as_the_stop_reason(tmp_path, scripts_dir):
    loop, _sb, trace = build_loop(
        tmp_path, scripts_dir / "limit_max_turns.json",
        budget=Budget(max_turns=3, deadline_s=0))
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.MAX_TURNS
    assert res.hit_budget
    assert res.ledger.turns == 3
    assert any("budget exhausted" in d for d in res.deviations)


def test_max_tool_calls_is_recorded_as_the_stop_reason(tmp_path, scripts_dir):
    loop, _sb, trace = build_loop(
        tmp_path, scripts_dir / "limit_max_tool_calls.json",
        budget=Budget(max_turns=0, max_tool_calls=6, deadline_s=0))
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.MAX_TOOL_CALLS
    assert res.ledger.tool_calls == 6
    assert "unexecuted" in res.stop_detail


def test_deadline_is_recorded_as_the_stop_reason(tmp_path, scripts_dir):
    loop, _sb, trace = build_loop(
        tmp_path, scripts_dir / "limit_deadline.json",
        budget=Budget(max_turns=0, max_tool_calls=0,
                      deadline_s=time.time() + 1.0))
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.DEADLINE
    assert res.ledger.elapsed_s >= 0.7


def test_token_budget_is_recorded_as_the_stop_reason(tmp_path, scripts_dir):
    loop, _sb, trace = build_loop(
        tmp_path, scripts_dir / "limit_tokens.json",
        budget=Budget(max_turns=0, max_tool_calls=0, max_tokens_in=400_000,
                      max_tokens_out=0, deadline_s=0))
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.TOKEN_BUDGET
    # Token limits are only checkable after the call, so the run may overshoot
    # by one turn. That is recorded, not hidden.
    assert res.ledger.tokens_in >= 400_000


def test_every_limit_is_visible_in_the_trace(tmp_path, scripts_dir):
    loop, _sb, trace = build_loop(
        tmp_path, scripts_dir / "limit_max_turns.json",
        budget=Budget(max_turns=2, deadline_s=0))
    loop.run()
    trace.close()
    recs = [json.loads(l) for l in
            (tmp_path / "run" / "trace.jsonl").read_text(
                encoding="utf-8").splitlines() if l.strip()]
    trips = [r for r in recs if r.get("kind") == "budget_exhausted"]
    assert len(trips) == 1 and trips[0]["reason"] == StopReason.MAX_TURNS


# -- the loop cannot invent success ----------------------------------------

def test_a_script_that_does_nothing_produces_no_artifact(tmp_path,
                                                         scripts_dir):
    loop, sandbox, trace = build_loop(tmp_path,
                                      scripts_dir / "a1_noop.json")
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.MODEL_STOPPED
    assert res.ledger.tool_calls == 0
    assert res.self_verified is False
    assert [*Path(sandbox.scratch_dir).rglob("*.omniworld"),
            *Path(sandbox.scratch_dir).rglob("*.wbt")] == []


def test_script_exhaustion_is_its_own_stop_reason(tmp_path):
    # Last turn asks for a tool, so the loop wants another turn and the
    # script has none: that is NOT a model that stopped.
    s = write_script(tmp_path, "short", [{"text": "call", "tool_calls":
                                          [CHEAP]}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.SCRIPT_EXHAUSTED


def test_calling_a_tool_the_condition_does_not_grant_is_loud(tmp_path):
    s = write_script(tmp_path, "wrong_surface", [
        {"text": "using the harness", "tool_calls": [
            {"name": "load_world", "arguments": {"path": "x.wbt"}}]}])
    loop, _sb, trace = build_loop(tmp_path, s, condition="shell")
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.BACKEND_ERROR
    assert "load_world" in res.stop_detail


def test_unknown_tool_is_returned_to_the_model_as_an_error(tmp_path):
    """A tool a real model hallucinates is its problem to route around.

    Distinct from the case above: there, a *script* named a tool the condition
    does not grant, which is a bug in the fixture and must abort the run. Here
    the call reaches the loop, and the loop's job is to hand back a usable
    error instead of crashing.
    """
    from agentbench.runner.backends import ToolCall
    s = write_script(tmp_path, "hallucinated", [{"text": "x"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    out = loop._execute(ToolCall(id="t1", name="definitely_not_a_tool",
                                 arguments={}))
    trace.close()
    assert out.is_error
    assert "no such tool" in out.text
    assert "run_shell" in out.text, "the error should list what IS available"


# -- history integrity -----------------------------------------------------

def test_tool_use_ids_round_trip_into_tool_results(tmp_path):
    s = write_script(tmp_path, "ids", [
        {"text": "two calls", "tool_calls": [CHEAP, CHEAP]},
        {"text": "done"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    loop.run()
    trace.close()
    assistant = [m for m in loop.messages if m["role"] == "assistant"][0]
    user = [m for m in loop.messages if m["role"] == "user"][1]
    want = [b["id"] for b in assistant["content"] if b["type"] == "tool_use"]
    got = [b["tool_use_id"] for b in user["content"]]
    assert want == got and len(want) == 2


def test_first_user_message_is_the_prompt_verbatim(tmp_path):
    prompt = "Build me a scene with 10 Huskies in it."
    s = write_script(tmp_path, "verbatim", [{"text": "ok"}])
    loop, _sb, trace = build_loop(tmp_path, s, prompt=prompt)
    loop.run()
    trace.close()
    assert loop.messages[0] == {"role": "user", "content": prompt}


# -- self_verified heuristic ----------------------------------------------

def test_write_then_read_back_counts_as_self_verified(tmp_path):
    s = write_script(tmp_path, "verified", [
        {"text": "write", "tool_calls": [
            {"name": "write_file", "arguments": {"path": "a.txt",
                                                 "content": "hi"}}]},
        {"text": "check", "tool_calls": [
            {"name": "read_file", "arguments": {"path": "a.txt"}}]},
        {"text": "done"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    assert res.self_verified is True


def test_write_and_stop_is_not_self_verified(tmp_path):
    s = write_script(tmp_path, "unverified", [
        {"text": "write", "tool_calls": [
            {"name": "write_file", "arguments": {"path": "a.txt",
                                                 "content": "hi"}}]},
        {"text": "done, it works"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    res = loop.run()
    trace.close()
    assert res.self_verified is False


# -- isolation -------------------------------------------------------------

def test_writes_outside_scratch_are_refused(tmp_path):
    target = tmp_path / "outside.txt"
    s = write_script(tmp_path, "escape", [
        {"text": "escaping", "tool_calls": [
            {"name": "write_file", "arguments": {"path": str(target),
                                                 "content": "x"}}]},
        {"text": "done"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    loop.run()
    trace.close()
    assert not target.exists()


def test_reading_the_benchmark_package_is_refused(tmp_path):
    from agentbench.common.paths import AGENTBENCH
    s = write_script(tmp_path, "peek", [
        {"text": "peeking", "tool_calls": [
            {"name": "read_file",
             "arguments": {"path": str(AGENTBENCH / "graders" / "a1.py")}}]},
        {"text": "done"}])
    loop, _sb, trace = build_loop(tmp_path, s)
    loop.run()
    trace.close()
    recs = [json.loads(l) for l in
            (tmp_path / "run" / "trace.jsonl").read_text(
                encoding="utf-8").splitlines() if l.strip()]
    call = [r for r in recs if r.get("kind") == "tool_call"][0]
    assert "refused" in (call["error"] or "")


def test_harness_tools_are_wired_and_fail_honestly_when_nothing_listens(
        tmp_path):
    """SPEC 2.3 makes starting the harness part of the task, so in the
    ``shell+tools`` condition the harness tools must be callable *and* must say
    clearly that nothing is listening yet -- not raise, and not look like a
    benchmark bug to the model."""
    s = write_script(tmp_path, "probe", [
        {"text": "is a harness up?", "tool_calls": [
            {"name": "harness_status", "arguments": {}}]},
        {"text": "no harness yet"}])
    loop, _sb, trace = build_loop(tmp_path, s, condition="shell+tools")
    res = loop.run()
    trace.close()
    assert res.stop_reason == StopReason.MODEL_STOPPED
    assert res.ledger.tool_calls == 1
    recs = [json.loads(l) for l in
            (tmp_path / "run" / "trace.jsonl").read_text(
                encoding="utf-8").splitlines() if l.strip()]
    call = [r for r in recs if r.get("kind") == "tool_call"][0]
    text = json.dumps(call["result"])
    assert "reachable" in text and "false" in text.lower()
    assert "omnisim_harness.py" in text, "the shipped hint must survive"


@pytest.mark.parametrize("var", ["ANTHROPIC_API_KEY", "PYTHONHASHSEED",
                                 "AGENTBENCH_EXTERNAL_ARTIFACT"])
def test_answer_leaking_env_vars_never_reach_the_agent(tmp_path, monkeypatch,
                                                       var):
    monkeypatch.setenv(var, "leaked")
    from agentbench.runner.isolation import Sandbox
    sb = Sandbox.create(tmp_path / "run", tmp_path / "run" / "scratch",
                        ports=False)
    assert var not in sb.env()
