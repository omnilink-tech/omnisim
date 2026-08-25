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

"""Red evidence for the campaign driver: schedule shape, credential/model
gates, flake retries with attribution, the INVALID ceiling, resume-from-state
and the exploratory smoke path -- each observed producing its behaviour from
fixtures constructed to hit exactly it. Executors are injected; no subprocess
and no simulator run here (``--dry-run`` is the live end-to-end proof)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import campaign as campaign_mod  # noqa: E402
import f_eval  # noqa: E402
from campaign import (  # noqa: E402
    Campaign, CampaignError, CredentialMissing, RunSpec, build_schedule,
    run_smoke)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def child_row(spec, outcome="PASS", calls=10):
    """What run_agentbench.py's cell path hands back for one run."""
    return {"suite": "agenticsimbench/v0.3", "task": spec.task, "sim": "omnisim",
            "condition": spec.condition, "repeat": 0,
            "outcome": outcome,
            "agent": {"model": "claude-test-1", "kind": spec.agent},
            "machine": {"machine": {"id": "fixture-machine"}},
            "metrics": {"tool_calls": calls, "t_agent_s": 1.0,
                        "t_total_s": 2.0, "tokens_in": 100,
                        "tokens_out": 10, "tokens_cache_read": None}}


class StubExecutor:
    """Injectable executor: records every call, replays scripted outcomes.

    ``script[run_key]`` is a list of outcomes consumed one per attempt;
    anything unscripted returns PASS."""

    def __init__(self, script=None):
        self.script = {k: list(v) for k, v in (script or {}).items()}
        self.calls = []               # (run_key, attempt)

    def __call__(self, spec, attempt_dir, attempt):
        self.calls.append((spec.key, attempt))
        seq = self.script.get(spec.key)
        outcome = seq.pop(0) if seq else "PASS"
        return child_row(spec, outcome=outcome)


def read_rows(path):
    return [json.loads(ln) for ln in
            Path(path).read_text(encoding="utf-8").splitlines()
            if ln.strip()]


def make_campaign(tmp_path, *, sims=("omnisim",), dry_run=False,
                  executor=None, model="claude-test-1", cid="c1"):
    return Campaign(tmp_path / cid, model=model, sims=list(sims),
                    dry_run=dry_run, executor=executor or StubExecutor(),
                    publish=False, quiet=True)


# ---------------------------------------------------------------------------
# schedule shape (plan §3.1: 70 per sim, 140 for two; A1 at n = 10)
# ---------------------------------------------------------------------------

def test_schedule_is_70_runs_for_one_sim():
    sched = build_schedule(["omnisim"])
    assert len(sched) == 70


def test_schedule_is_140_runs_for_two_sims():
    sched = build_schedule(["omnisim", "webots"])
    assert len(sched) == 140


def test_schedule_a1_runs_at_n10_and_lane_b_at_n5():
    sched = build_schedule(["omnisim"])
    a1 = [s for s in sched if s.task == campaign_mod.LANE_A_TASK]
    assert len(a1) == 20                       # 2 conditions x n=10
    assert all(s.lane == "A" and s.n == 10 for s in a1)
    for task in campaign_mod.LANE_B_TASKS:
        for cond in campaign_mod.CONDITIONS:
            cell = [s for s in sched
                    if s.task == task and s.condition == cond]
            assert len(cell) == 5
            assert all(s.lane == "B" and s.n == 5 for s in cell)


def test_schedule_conditions_map_to_pinned_agents():
    sched = build_schedule(["omnisim"])
    assert {s.agent for s in sched if s.condition == "shell"} == {"llm_shell"}
    assert ({s.agent for s in sched if s.condition == "shell+tools"}
            == {"llm_tools"})


def test_schedule_rejects_unknown_sim():
    with pytest.raises(CampaignError):
        build_schedule(["gazebo"])


# ---------------------------------------------------------------------------
# the gates
# ---------------------------------------------------------------------------

def test_scored_run_refuses_without_credential(tmp_path, monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    rc = campaign_mod.main(["--model", "claude-test-1",
                            "--campaign-id", "nokey",
                            "--out-root", str(tmp_path)])
    assert rc == 2
    assert not (tmp_path / "nokey").exists()   # refused before any state


def test_scored_run_refuses_default_model(tmp_path, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-test")
    rc = campaign_mod.main(["--campaign-id", "nomodel",
                            "--out-root", str(tmp_path)])
    assert rc == 2
    assert not (tmp_path / "nomodel").exists()


def test_smoke_also_gated_on_credential(tmp_path, monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    rc = campaign_mod.main(["--smoke", "--model", "claude-test-1",
                            "--campaign-id", "smokegate",
                            "--out-root", str(tmp_path)])
    assert rc == 2


def test_dry_run_needs_no_credential_or_model(tmp_path, monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    # not main() (which would subprocess 70 cells); the class-level dry run
    # with an injected executor proves the gate logic + walk shape.
    camp = make_campaign(tmp_path, dry_run=True, model=None)
    result = camp.run()
    assert result["meta"]["completed"] == 70
    assert result["published"] is None


# ---------------------------------------------------------------------------
# rows, retries, attribution
# ---------------------------------------------------------------------------

def test_happy_path_writes_one_row_per_run_and_report(tmp_path):
    ex = StubExecutor()
    camp = make_campaign(tmp_path, executor=ex)
    result = camp.run()
    rows = read_rows(camp.rows_path)
    assert len(rows) == 70
    assert len(ex.calls) == 70
    assert result["meta"]["outcomes"] == {"PASS": 70}
    # every row carries the campaign block with the pinned model + repeat
    for r in rows:
        assert r["campaign"]["model_flag"] == "claude-test-1"
        assert r["repeat"] == r["campaign"]["repeat"]
    assert Path(result["report"]).is_file()


def test_flake_retry_attributed_never_silent(tmp_path):
    key = "omnisim/B1_overlap_audit/shell/r0"
    ex = StubExecutor({key: ["INVALID", "INVALID", "PASS"]})
    camp = make_campaign(tmp_path, executor=ex)
    camp.run()
    rows = [r for r in read_rows(camp.rows_path)
            if r["campaign"]["cell"] == "omnisim/B1_overlap_audit/shell"
            and r["campaign"]["repeat"] == 0]
    assert [r["outcome"] for r in rows] == ["INVALID", "INVALID", "PASS"]
    assert [r["campaign"]["superseded"] for r in rows] == [True, True, False]
    assert rows[1]["campaign"]["retry_reason"] == "INVALID"
    assert rows[2]["campaign"]["attempt"] == 3
    assert camp.state["retries"][key] == 2
    assert camp.state["completed"][key]["outcome"] == "PASS"
    # superseded attempts are invisible to the F evaluator
    ev = f_eval.evaluate((r, "x#L%d" % i)
                         for i, r in enumerate(read_rows(camp.rows_path), 1))
    cell = ev.col.cells[("omnisim", "shell", "B1_overlap_audit")]
    assert cell.n_total == 5 and cell.invalids == 0


def test_retry_budget_is_two_then_the_invalid_stands(tmp_path):
    key = "omnisim/C2_fall_through_floor/shell/r1"
    ex = StubExecutor({key: ["INVALID", "INVALID", "INVALID", "PASS"]})
    camp = make_campaign(tmp_path, executor=ex)
    camp.run()
    attempts = [a for k, a in ex.calls if k == key]
    assert attempts == [0, 1, 2]               # 1 launch + 2 retries, no more
    assert camp.state["completed"][key]["outcome"] == "INVALID"


def test_invalid_ceiling_marks_cell_published_as_unreliable(tmp_path):
    # 2 of 5 repeats end INVALID after all retries -> 40% > 20% -> flagged
    cell = "omnisim/B2_subject_in_frame/shell"
    script = {cell + "/r1": ["INVALID"] * 3, cell + "/r3": ["INVALID"] * 3}
    camp = make_campaign(tmp_path, executor=StubExecutor(script))
    camp.run()
    flag = camp.state["unreliable_cells"][cell]
    assert flag["label"] == "published-as-unreliable"
    assert flag["invalid"] == 2 and flag["n"] == 5
    meta = json.loads((camp.dir / "campaign_meta.json").read_text("utf-8"))
    assert cell in meta["unreliable_cells"]


def test_one_invalid_of_five_stays_below_ceiling(tmp_path):
    cell = "omnisim/B2_subject_in_frame/shell"
    script = {cell + "/r1": ["INVALID"] * 3}   # 1/5 = 20%, not > 20%
    camp = make_campaign(tmp_path, executor=StubExecutor(script))
    camp.run()
    assert cell not in camp.state["unreliable_cells"]


def test_skipped_in_scored_campaign_stops_without_completing(tmp_path):
    key0 = build_schedule(["omnisim"])[0].key
    ex = StubExecutor({key0: ["SKIPPED"]})
    camp = make_campaign(tmp_path, executor=ex)
    with pytest.raises(CredentialMissing):
        camp.run()
    assert key0 not in camp.state["completed"]     # resume will re-run it
    rows = read_rows(camp.rows_path)
    assert len(rows) == 1 and rows[0]["outcome"] == "SKIPPED"


def test_skipped_is_normal_in_dry_run(tmp_path):
    class SkippedStub(StubExecutor):
        def __call__(self, spec, attempt_dir, attempt):
            self.calls.append((spec.key, attempt))
            return child_row(spec, outcome="SKIPPED")
    camp = make_campaign(tmp_path, dry_run=True, model=None,
                         executor=SkippedStub())
    result = camp.run()
    assert result["meta"]["completed"] == 70
    assert result["meta"]["outcomes"] == {"SKIPPED": 70}


# ---------------------------------------------------------------------------
# resume
# ---------------------------------------------------------------------------

class ExplodesAfter(StubExecutor):
    def __init__(self, n):
        super().__init__()
        self.n = n

    def __call__(self, spec, attempt_dir, attempt):
        if len(self.calls) >= self.n:
            raise RuntimeError("simulated interruption")
        return super().__call__(spec, attempt_dir, attempt)


def test_resume_continues_from_next_incomplete_run(tmp_path):
    ex1 = ExplodesAfter(7)
    camp1 = make_campaign(tmp_path, executor=ex1)
    with pytest.raises(RuntimeError):
        camp1.run()
    assert len(camp1.state["completed"]) == 7

    ex2 = StubExecutor()
    camp2 = make_campaign(tmp_path, executor=ex2)      # same dir -> resume
    camp2.run()
    done_first = {k for k, _ in ex1.calls}
    done_second = {k for k, _ in ex2.calls}
    assert len(done_second) == 63                      # never re-runs the 7
    assert not (done_first & done_second)
    assert len(camp2.state["completed"]) == 70
    # rows are append-only across the interruption: 7 + 63
    assert len(read_rows(camp2.rows_path)) == 70


def test_resume_refuses_changed_configuration(tmp_path):
    camp1 = make_campaign(tmp_path, executor=ExplodesAfter(1))
    with pytest.raises(RuntimeError):
        camp1.run()
    with pytest.raises(CampaignError, match="resume refused"):
        make_campaign(tmp_path, model="claude-other-model")
    with pytest.raises(CampaignError, match="resume refused"):
        make_campaign(tmp_path, sims=("omnisim", "webots"))


# ---------------------------------------------------------------------------
# smoke
# ---------------------------------------------------------------------------

def test_smoke_is_exploratory_and_never_in_campaign_rows(tmp_path):
    ex = StubExecutor()
    row = run_smoke(tmp_path / "c1", model="claude-test-1",
                    executor=ex, quiet=True)
    assert row["campaign"]["exploratory"] is True
    assert row["campaign"]["smoke"] is True
    smoke_rows = read_rows(tmp_path / "c1" / "smoke" / "rows.jsonl")
    assert len(smoke_rows) == 1
    assert not (tmp_path / "c1" / "rows.jsonl").exists()
    assert not (tmp_path / "c1" / "state.json").exists()
    # the F evaluator refuses to aggregate it
    ev = f_eval.evaluate((r, "s#L1") for r in smoke_rows)
    assert ev.col.cells == {}
    assert ev.col.excluded == {"exploratory": 1}


def test_smoke_runs_exactly_one_cell(tmp_path):
    ex = StubExecutor()
    run_smoke(tmp_path / "c1", model="claude-test-1", executor=ex,
              quiet=True)
    assert len(ex.calls) == 1
    key, attempt = ex.calls[0]
    assert key.startswith("omnisim/%s/shell/" % campaign_mod.SMOKE_DEFAULT_TASK)
    assert attempt == 0


# ---------------------------------------------------------------------------
# completion artefacts
# ---------------------------------------------------------------------------

def test_finish_writes_meta_and_f_report_without_publish(tmp_path):
    camp = make_campaign(tmp_path)
    result = camp.run()
    meta = json.loads((camp.dir / "campaign_meta.json").read_text("utf-8"))
    assert meta["model"] == "claude-test-1"
    assert meta["schedule_len"] == 70
    assert meta["completed"] == 70
    report = Path(result["report"]).read_text(encoding="utf-8")
    # every scored stub row is PASS -> the evaluator sees real cells
    assert "F-surface" in report
    assert result["published"] is None                 # publish=False here


def test_row_repeat_rewritten_to_schedule_index(tmp_path):
    camp = make_campaign(tmp_path)
    camp.run()
    rows = [r for r in read_rows(camp.rows_path)
            if r["campaign"]["cell"] == "omnisim/B3_measure_and_report/shell"]
    assert sorted(r["repeat"] for r in rows) == [0, 1, 2, 3, 4]
    assert all(r["campaign"]["child_repeat"] == 0 for r in rows)
