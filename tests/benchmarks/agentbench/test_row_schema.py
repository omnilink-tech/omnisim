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

"""Row-schema hardening (Phase R items 2-3, agent-edge-validation-plan §0.2).

What these tests forbid, each one a defect the first A/B actually shipped:

  * a SCORED row (llm*/external -- any agent whose outcome is a measurement,
    not a Phase-0 expectation) carrying ``condition: "scripted"``. 56 of the
    58 rows in the first campaign did, which made the A/B unqueryable;
  * ``t_agent_s`` written as a copy of ``t_total_s`` (the two clocks were the
    same number; the real agent clock survived only in the trace);
  * ``tokens_cache_write`` accounted by the Ledger but dropped from the row;
  * campaign rows that exist only in the gitignored ``results/`` scratch tree
    ("no row, no result" -- the ``--publish`` mechanism).

No simulator: every synthetic agent here produces NO artifact, so the grader
fails it structurally without launching anything (same trick as
``runner/tests/test_recorder_shadow.py``'s ``_cheat`` cell).
"""

from __future__ import annotations

import json
import hashlib
import subprocess
import sys
import time
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[1]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench import agents as agent_registry          # noqa: E402
from agentbench import run_agentbench                    # noqa: E402
from agentbench import tasks as task_registry            # noqa: E402
from agentbench.agents.base import AgentResult           # noqa: E402

TASK = task_registry.get("A1_husky_swarm_10")
REPO = Path(__file__).resolve().parents[3]


@pytest.fixture(autouse=True)
def _fast_fingerprint(monkeypatch):
    """The machine block is not under test, and the real fingerprint hashes
    the engine binary (seconds). Stubbed, and visibly so in the row."""
    monkeypatch.setattr(run_agentbench.ob_results, "machine_fingerprint",
                        lambda engine_log_path=None: {"stubbed_by": __name__})


def _cell(tmp_path, monkeypatch, fn, *, expect_pass=None, condition=None,
          fake_sim=False, name="_synth"):
    """Register ``fn`` as a throwaway agent for A1 and run one cell."""
    monkeypatch.setitem(agent_registry.REGISTRY, (TASK.id, name),
                        {"fn": fn, "expect_pass": expect_pass,
                         "expect_failures": None})
    return run_agentbench.run_cell(TASK, name, 0, tmp_path, quiet=True,
                                   condition=condition, fake_sim=fake_sim)


def _noop(ctx):
    return AgentResult()


def _with_artifacts(**artifacts):
    def fn(ctx):
        res = AgentResult()
        res.final_message = "done (synthetic)"
        res.artifacts.update(artifacts)
        return res
    return fn


# --- task 2: the row guard -------------------------------------------------

def test_a_scored_row_cannot_carry_condition_scripted(tmp_path, monkeypatch):
    """The enforcement itself: a scored agent that records no condition, with
    no --condition given, must NOT fall back to "scripted" -- the row is
    INVALID (SPEC 3.3: missing attribution) with condition None."""
    row, verdict = _cell(tmp_path, monkeypatch, _noop)   # no artifact -> FAIL
    assert row["condition"] is None, row["condition"]
    assert row["outcome"] == "INVALID"
    assert verdict.outcome == "INVALID"
    assert any("scored row has no condition" in d for d in row["deviations"])
    assert any("condition unattributed" in n for n in row["notes"])


def test_no_row_path_yields_scripted_for_a_scored_kind(tmp_path, monkeypatch):
    """Even a non-graded row (--fake-sim -> SKIPPED) may not read "scripted"
    for a scored kind: condition is None, the SKIPPED outcome survives."""
    row, verdict = _cell(tmp_path, monkeypatch, _noop, fake_sim=True)
    assert verdict.outcome == "SKIPPED"
    assert row["outcome"] == "SKIPPED", "the guard must not flip SKIPPED"
    assert row["condition"] is None
    assert any("scored row has no condition" in d for d in row["deviations"])


def test_agent_recorded_condition_beats_the_cli_flag(tmp_path, monkeypatch):
    row, _ = _cell(tmp_path, monkeypatch,
                   _with_artifacts(condition="shell+tools"),
                   condition="shell_only")
    assert row["condition"] == "shell+tools"
    assert row["outcome"] == "FAIL"       # attributed, so the FAIL stands


def test_cli_condition_fills_in_when_the_agent_is_silent(tmp_path,
                                                         monkeypatch):
    """--condition, normalised to the SPEC 5 spelling, keeps a scored row
    attributable -- and therefore scoreable."""
    row, _ = _cell(tmp_path, monkeypatch, _noop, condition="full_surface")
    assert row["condition"] == "shell+tools"
    assert row["outcome"] == "FAIL"
    assert not any("scored row has no condition" in d
                   for d in row["deviations"])


def test_phase0_scripted_rows_keep_the_scripted_fallback(tmp_path,
                                                         monkeypatch):
    """The guard is scoped: oracle/null/wrong/parade rows are expectation
    checks, and "scripted" remains their honest condition."""
    row, verdict = run_agentbench.run_cell(TASK, "null", 0, tmp_path,
                                           quiet=True)
    assert row["condition"] == "scripted"
    assert row["outcome"] == "FAIL"
    assert row["expectation"]["ok"] is True


def test_is_scored_kind_is_the_registry_convention():
    """"Scored" is derived from expect_pass=None (agents/__init__.py's own
    convention), not a hand-kept name list."""
    for (task_id, kind), entry in agent_registry.REGISTRY.items():
        scored = run_agentbench.is_scored_kind(entry)
        if kind in ("oracle", "null", "wrong", "parade"):
            assert not scored, (task_id, kind)
        if kind in ("llm", "llm_shell", "llm_tools", "external"):
            assert scored, (task_id, kind)


def test_an_error_row_is_unattributed_never_scripted(tmp_path, monkeypatch):
    def boom(ctx):
        raise RuntimeError("synthetic agent crash")
    row, verdict = _cell(tmp_path, monkeypatch, boom)
    assert row["outcome"] == "ERROR", "ERROR is not flipped to INVALID"
    assert row["condition"] is None
    # The agent never ran to completion: its clock must be None, not a copy
    # of the harness clock (never fabricate).
    assert row["metrics"]["t_agent_s"] is None
    assert row["metrics"]["t_total_s"] > 0


# --- task 4: clock separation ----------------------------------------------

def test_t_agent_s_is_the_agents_own_clock_when_available(tmp_path,
                                                          monkeypatch):
    row, _ = _cell(tmp_path, monkeypatch,
                   _with_artifacts(condition="shell", agent_wall_s=1.234))
    m = row["metrics"]
    assert m["t_agent_s"] == 1.234
    assert m["t_total_s"] > 0
    # The regression this file exists for: the two clocks written as one
    # number (run_agentbench.py pre-Phase-R wrote round(t_total,3) to both).
    assert m["t_agent_s"] != m["t_total_s"]


def test_t_agent_s_falls_back_to_the_measured_fn_wall(tmp_path, monkeypatch):
    def slowish(ctx):
        time.sleep(0.05)
        res = AgentResult()
        res.artifacts["condition"] = "shell"
        return res
    row, _ = _cell(tmp_path, monkeypatch, slowish)
    m = row["metrics"]
    assert m["t_agent_s"] is not None
    assert 0.05 <= m["t_agent_s"] <= m["t_total_s"], m


# --- task 3: tokens_cache_write --------------------------------------------

def test_tokens_cache_write_reaches_the_metrics_block(tmp_path, monkeypatch):
    row, _ = _cell(tmp_path, monkeypatch,
                   _with_artifacts(condition="shell", tokens_cache_write=777))
    assert row["metrics"]["tokens_cache_write"] == 777


def test_unmeasured_cache_write_is_null_never_zero(tmp_path, monkeypatch):
    row, _ = _cell(tmp_path, monkeypatch,
                   _with_artifacts(condition="shell"))
    assert row["metrics"]["tokens_cache_write"] is None


def test_the_loop_plumbs_cache_write_into_artifacts(tmp_path, monkeypatch):
    """End to end through agents/llm.py + the scripted backend: the Ledger's
    cache-write figure must surface in AgentResult.artifacts, which is the
    only road it has into the row (AgentResult has no field for it)."""
    from agentbench.agents import llm
    from agentbench.common.trace import Trace
    from agentbench.agents.base import AgentContext

    script = tmp_path / "cache_write.json"
    script.write_text(json.dumps({
        "schema": "agentbench/script/v1", "name": "cache_write",
        "turns": [{"text": "Done.",
                   "usage": {"tokens_in": 10, "tokens_out": 5,
                             "tokens_cache_write": 42}}]}), encoding="utf-8")
    monkeypatch.setenv("AGENTBENCH_BACKEND", "scripted")
    monkeypatch.setenv("AGENTBENCH_SCRIPT", str(script))
    monkeypatch.setenv("AGENTBENCH_CONDITION", "shell")

    run_dir = tmp_path / "run"
    scratch = run_dir / "scratch"
    scratch.mkdir(parents=True)
    trace = Trace(run_dir)
    try:
        ctx = AgentContext(task_id=TASK.id, prompt="Build me a scene.",
                           scratch_dir=scratch, run_dir=run_dir, repo=REPO,
                           trace=trace, deadline_s=time.time() + 300, seed=0)
        res = llm.run(ctx)
    finally:
        trace.close()
    assert res.error is None
    assert res.artifacts["tokens_cache_write"] == 42
    assert res.artifacts["agent_wall_s"] > 0


def test_fake_sim_cell_prints_without_crashing(tmp_path, monkeypatch, capsys):
    """Regression (pre-existing, found by the CLI smoke for this file): the
    --fake-sim exp dict has no pass_matches/failures_match keys, and the
    non-quiet per-cell print KeyError'd on them -- so every --fake-sim CLI
    cell (SPEC 7's no-sim harness check) came out ERROR."""
    monkeypatch.setitem(agent_registry.REGISTRY, (TASK.id, "_synth"),
                        {"fn": _noop, "expect_pass": None,
                         "expect_failures": None})
    row, verdict = run_agentbench.run_cell(TASK, "_synth", 0, tmp_path,
                                           quiet=False, fake_sim=True)
    assert row["outcome"] == "SKIPPED"
    assert "expectation: -" in capsys.readouterr().out


# --- task 1: the --condition flag ------------------------------------------

def test_condition_flag_rejects_values_outside_the_alias_table():
    with pytest.raises(SystemExit):                       # argparse exit 2
        run_agentbench.main(["--condition", "bogus", "--list"])


def test_condition_flag_sets_the_runner_env(monkeypatch, capsys):
    monkeypatch.setenv("AGENTBENCH_CONDITION", "sentinel")
    rc = run_agentbench.main(["--condition", "shell_only", "--list"])
    assert rc == 0
    import os
    assert os.environ["AGENTBENCH_CONDITION"] == "shell_only"


# --- task 5: --publish ------------------------------------------------------

def _fake_run(tmp_path, name="20260731_000000"):
    src = tmp_path / "results" / name
    src.mkdir(parents=True)
    rows = [{"task": "A1", "outcome": "FAIL"}, {"task": "A1",
                                                "outcome": "PASS"}]
    (src / "rows.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    return src


def test_publish_copies_rows_and_writes_checkable_meta(tmp_path):
    src = _fake_run(tmp_path)
    dest = run_agentbench.publish_run(src, dest_root=tmp_path / "pub")
    assert dest == tmp_path / "pub" / src.name
    data = (src / "rows.jsonl").read_bytes()
    assert (dest / "rows.jsonl").read_bytes() == data
    meta = json.loads((dest / "publish_meta.json").read_text("utf-8"))
    assert meta["n_rows"] == 2
    assert meta["rows_sha256"] == hashlib.sha256(data).hexdigest()
    assert meta["run_id"] == src.name
    assert meta["source"] == str(src)


def test_publish_refuses_to_overwrite_a_publication(tmp_path):
    src = _fake_run(tmp_path)
    run_agentbench.publish_run(src, dest_root=tmp_path / "pub")
    with pytest.raises(FileExistsError):
        run_agentbench.publish_run(src, dest_root=tmp_path / "pub")


def test_publish_requires_a_finished_run(tmp_path):
    empty = tmp_path / "not_a_run"
    empty.mkdir()
    with pytest.raises(FileNotFoundError):
        run_agentbench.publish_run(empty, dest_root=tmp_path / "pub")


def test_publish_cli_verb_reports_refusal_as_exit_2(tmp_path, capsys):
    rc = run_agentbench.main(["--publish", str(tmp_path / "missing")])
    assert rc == 2
    assert "publish refused" in capsys.readouterr().err


def test_results_published_is_tracked_not_gitignored():
    """The whole point of the mechanism: the destination must survive a
    commit. results/.gitignore covers only results/."""
    pub = run_agentbench.RESULTS_PUBLISHED
    assert pub.name == "results_published"
    assert (pub / "README.md").is_file()
    probe = "tests/benchmarks/agentbench/results_published/x/rows.jsonl"
    rc = subprocess.run(["git", "-C", str(REPO), "check-ignore", "-q", probe],
                        capture_output=True).returncode
    assert rc != 0, "%s is gitignored -- published rows would be lost" % probe
