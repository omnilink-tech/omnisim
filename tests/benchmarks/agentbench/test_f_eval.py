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

"""Red evidence for the F evaluator itself (plan §5.5 applied to §2.4).

Every verdict branch is driven by a synthetic row fixture constructed to hit
exactly that branch: survive, each of the four narrow templates (both adverse
ones included), withdrawal, the death condition, unevaluable channels, the
>= 3-passes definedness floor, the dpass >= 0 gate, and the best-condition
near-tie rule. A green evaluator that was never observed producing each
verdict is not evidence the evaluator works.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import f_eval  # noqa: E402

B = list(f_eval.LANE_B_TASKS)          # the 5 decision tasks
A1 = f_eval.LANE_A_TASK


# ---------------------------------------------------------------------------
# fixture builders
# ---------------------------------------------------------------------------

def row(task, cond, outcome="PASS", calls=10, sim="omnisim", repeat=0,
        campaign=None, model="claude-test-1"):
    r = {"suite": "agenticsimbench/v0.3", "task": task, "sim": "omnisim",
         "condition": cond, "repeat": repeat, "outcome": outcome,
         "agent": {"model": model},
         "machine": {"machine": {"id": "fixture-machine"}},
         "metrics": {"tool_calls": calls, "t_agent_s": 1.5, "t_total_s": 9.0,
                     "tokens_in": 1000, "tokens_out": 100,
                     "tokens_cache_read": None}}
    camp = {"sim": sim}
    if campaign:
        camp.update(campaign)
    r["campaign"] = camp
    return r


def cell_rows(task, cond, passes, n=5, calls=10, fail_calls=None,
              sim="omnisim"):
    out = []
    for i in range(n):
        if i < passes:
            out.append(row(task, cond, "PASS", calls, sim, repeat=i))
        else:
            out.append(row(task, cond, "FAIL",
                           fail_calls if fail_calls is not None else calls,
                           sim, repeat=i))
    return out


def ev_of(rows):
    return f_eval.evaluate((r, "fixture#L%d" % i)
                           for i, r in enumerate(rows, start=1))


def surface_rows(spec):
    """``spec[task] = (passes_tools, calls_tools, passes_shell,
    calls_shell)`` -> rows for all five Lane B tasks on omnisim."""
    rows = []
    for task, (pt, ct, ps, cs) in spec.items():
        rows += cell_rows(task, f_eval.TOOLS, pt, calls=ct)
        rows += cell_rows(task, f_eval.SHELL, ps, calls=cs)
    return rows


# ---------------------------------------------------------------------------
# the three-way verdict, branch by branch
# ---------------------------------------------------------------------------

def test_survives_when_both_channels_hold():
    ev = ev_of(surface_rows({t: (5, 8, 3, 20) for t in B}))
    pe = ev.surface
    assert pe.completion_holds and pe.cost_holds
    assert pe.verdict == f_eval.SURVIVES
    assert not pe.unevaluable_channels
    text = f_eval.render(ev)
    assert "survives" in text
    assert "Survival licenses nothing" in text
    # the report itself must clear the forbidden-sentence check
    assert f_eval.check_forbidden(text, ev.col.sims) == []


def test_narrow_cost_only_template():
    # same completions in both conditions, materially fewer calls with tools
    ev = ev_of(surface_rows({t: (4, 8, 4, 20) for t in B}))
    pe = ev.surface
    assert pe.cost_holds and not pe.completion_holds
    assert pe.verdict == f_eval.NARROW
    assert pe.template == f_eval.TEMPLATE_COST_ONLY
    # the other channel's table is printed regardless
    text = f_eval.render(ev)
    assert "Completion channel" in text and "Cost channel" in text


def test_narrow_completion_only_template():
    # more completions, identical call cost
    ev = ev_of(surface_rows({t: (5, 10, 3, 10) for t in B}))
    pe = ev.surface
    assert pe.completion_holds and not pe.cost_holds
    assert pe.verdict == f_eval.NARROW
    assert pe.template == f_eval.TEMPLATE_COMPLETION_ONLY


def test_narrow_adverse_fewer_completions_template():
    # cost channel holds on three tasks, but one eligible task LOSES two
    # completions -> the adverse "fewer completions" template.
    spec = {B[0]: (4, 8, 4, 20), B[1]: (4, 8, 4, 20), B[2]: (4, 8, 4, 20),
            B[3]: (1, 10, 3, 10),          # dpass = -2, adverse
            B[4]: (3, 10, 3, 10)}
    ev = ev_of(surface_rows(spec))
    pe = ev.surface
    assert pe.cost_holds
    assert not pe.completion_holds and pe.adverse_n == 1
    assert pe.verdict == f_eval.NARROW
    assert pe.template == f_eval.TEMPLATE_FEWER_COMPLETIONS


def test_narrow_adverse_materially_more_calls_template():
    # completion holds, but the surface pays materially more calls on every
    # defined task -> the adverse "materially more calls" template.
    spec = {B[0]: (5, 20, 3, 10), B[1]: (5, 20, 3, 10),
            B[2]: (4, 20, 3, 10), B[3]: (3, 20, 3, 10),
            B[4]: (3, 20, 3, 10)}
    ev = ev_of(surface_rows(spec))
    pe = ev.surface
    assert pe.completion_holds
    assert not pe.cost_holds and pe.cost_adverse_n >= f_eval.COUNTING_TASKS
    assert pe.verdict == f_eval.NARROW
    assert pe.template == f_eval.TEMPLATE_MORE_CALLS


def test_withdrawn_when_neither_channel_holds():
    ev = ev_of(surface_rows({t: (4, 10, 4, 10) for t in B}))
    pe = ev.surface
    assert not pe.completion_holds and not pe.cost_holds
    assert not pe.unevaluable_channels        # evaluable, and still fails
    assert pe.verdict == f_eval.WITHDRAWN
    assert "WITHDRAWN" in f_eval.render(ev)


def test_withdrawn_by_each_conjunct_form():
    # completion fails via the adverse clause alone (>= 2 qualifying tasks
    # but one adverse), cost fails via the counting floor: both fail ->
    # withdrawal even though the surface "won" two tasks.
    spec = {B[0]: (5, 10, 3, 10), B[1]: (5, 10, 3, 10),
            B[2]: (1, 10, 3, 10),          # dpass = -2 kills channel A
            B[3]: (3, 10, 3, 10), B[4]: (3, 10, 3, 10)}
    ev = ev_of(surface_rows(spec))
    pe = ev.surface
    assert pe.qualifying_n >= 2 and pe.adverse_n == 1
    assert not pe.completion_holds
    assert pe.counting_n == 0 and not pe.cost_holds
    assert pe.verdict == f_eval.WITHDRAWN


# ---------------------------------------------------------------------------
# eligibility, definedness, the gate
# ---------------------------------------------------------------------------

def test_unevaluable_completion_channel_named_in_leading_sentence():
    # only 3 tasks have any pass -> completion channel unevaluable (< 4
    # eligible); cost channel still holds on the 3 defined tasks.
    spec = {B[0]: (5, 5, 5, 10), B[1]: (5, 5, 5, 10), B[2]: (5, 5, 5, 10),
            B[3]: (0, 10, 0, 10), B[4]: (0, 10, 0, 10)}
    ev = ev_of(surface_rows(spec))
    pe = ev.surface
    assert pe.completion_unevaluable and not pe.completion_holds
    assert pe.cost_holds
    assert any("completion channel (A)" in u
               for u in pe.unevaluable_channels)
    text = f_eval.render(ev)
    # the verdict text opens by naming the unevaluable channel
    body = text.split("\n\n")[1]
    assert "Unevaluable channels" in body
    assert "completion channel (A)" in text


def test_unevaluable_cost_channel():
    # every task eligible, but no task reaches the >= 3-passes-both floor
    ev = ev_of(surface_rows({t: (5, 5, 2, 10) for t in B}))
    pe = ev.surface
    assert pe.defined_n == 0 and pe.cost_unevaluable
    assert not pe.cost_holds
    assert any("cost channel (B)" in u for u in pe.unevaluable_channels)
    # completion channel still holds -> narrow finding, cost table printed
    assert pe.completion_holds and pe.verdict == f_eval.NARROW


def test_definedness_floor_undefined_ratio_below_three_passes():
    ev = ev_of(surface_rows({B[0]: (3, 5, 2, 10),
                             **{t: (5, 5, 5, 10) for t in B[1:]}}))
    ln = next(x for x in ev.surface.lines if x.task == B[0])
    assert not ln.defined
    assert ln.ratio is None        # a 2-pass median is a coin flip, not a stat
    assert not ln.counts


def test_gate_rejects_cost_win_on_completion_losing_task():
    # Rcalls = 0.25 <= 0.85, but dpass = -1: the task must NOT count.
    ev = ev_of(surface_rows({B[0]: (4, 5, 5, 20),
                             **{t: (4, 10, 4, 10) for t in B[1:]}}))
    ln = next(x for x in ev.surface.lines if x.task == B[0])
    assert ln.defined and ln.ratio is not None and ln.ratio <= 0.85
    assert ln.delta < 0
    assert not ln.counts


def test_eligibility_excludes_never_passed_tasks():
    spec = {t: (5, 8, 3, 20) for t in B}
    spec[B[4]] = (0, 10, 0, 10)            # neither condition ever passed
    ev = ev_of(surface_rows(spec))
    ln = next(x for x in ev.surface.lines if x.task == B[4])
    assert not ln.eligible
    assert ev.surface.eligible_n == 4
    assert "unresolvable by this design" in f_eval.render(ev)


# ---------------------------------------------------------------------------
# best condition + death condition
# ---------------------------------------------------------------------------

def test_best_condition_decisive_totals():
    ev = ev_of(surface_rows({t: (5, 30, 2, 10) for t in B}))
    bc = ev.best["omnisim"]
    assert bc.totals == {f_eval.SHELL: 10, f_eval.TOOLS: 25}
    assert bc.condition == f_eval.TOOLS    # totals differ by > 1


def test_best_condition_near_tie_goes_to_cheaper():
    # totals 20 vs 20 -> within 1 -> lower sum of per-task medians wins,
    # and here that is shell (10 calls/task vs 30).
    ev = ev_of(surface_rows({t: (4, 30, 4, 10) for t in B}))
    bc = ev.best["omnisim"]
    assert abs(bc.totals[f_eval.SHELL] - bc.totals[f_eval.TOOLS]) <= 1
    assert bc.condition == f_eval.SHELL
    assert "lower sum of per-task median" in bc.rule


def test_best_condition_near_tie_cheaper_can_be_tools():
    ev = ev_of(surface_rows({t: (4, 10, 4, 30) for t in B}))
    assert ev.best["omnisim"].condition == f_eval.TOOLS


def test_death_condition_fires():
    rows = surface_rows({t: (3, 20, 2, 25) for t in B})   # omnisim best=tools
    for t in B:                                            # webots: better+cheaper
        rows += cell_rows(t, f_eval.SHELL, 4, calls=10, sim="webots")
        rows += cell_rows(t, f_eval.TOOLS, 3, calls=15, sim="webots")
    ev = ev_of(rows)
    assert ev.comparative_sim == "webots"
    assert ev.death is not None and ev.death.evaluated
    assert ev.death.fired
    text = f_eval.render(ev)
    assert "DEATH CONDITION FIRED" in text
    assert f_eval.check_forbidden(text, ev.col.sims) == []


def test_death_condition_does_not_fire_when_omnisim_ahead():
    rows = surface_rows({t: (5, 8, 3, 20) for t in B})
    for t in B:
        rows += cell_rows(t, f_eval.SHELL, 2, calls=30, sim="webots")
        rows += cell_rows(t, f_eval.TOOLS, 2, calls=30, sim="webots")
    ev = ev_of(rows)
    assert ev.death is not None and not ev.death.fired
    assert "Did not fire" in f_eval.render(ev)


def test_comparative_channels_use_c_and_d_letters():
    rows = surface_rows({t: (5, 8, 3, 20) for t in B})
    for t in B:      # webots arm exists but nothing ever passes -> unevaluable
        rows += cell_rows(t, f_eval.SHELL, 0, calls=30, sim="webots")
        rows += cell_rows(t, f_eval.TOOLS, 0, calls=30, sim="webots")
    ev = ev_of(rows)
    assert ev.comparative is not None
    assert any("(D)" in u for u in ev.comparative.unevaluable_channels)


def test_no_competitor_means_comparative_not_evaluated():
    ev = ev_of(surface_rows({t: (5, 8, 3, 20) for t in B}))
    assert ev.comparative is None
    assert any("conjunct (ii) is not evaluated" in u
               for u in ev.unevaluable_channels)


# ---------------------------------------------------------------------------
# row hygiene: exclusions, sim attribution
# ---------------------------------------------------------------------------

def test_superseded_and_exploratory_rows_are_excluded():
    rows = surface_rows({t: (5, 8, 3, 20) for t in B})
    rows.append(row(B[0], f_eval.TOOLS, "PASS", 1,
                    campaign={"superseded": True}))
    rows.append(row(B[0], f_eval.TOOLS, "PASS", 1,
                    campaign={"exploratory": True}))
    rows.append(row(B[0], f_eval.TOOLS, "PASS", 1,
                    campaign={"dry_run": True}))
    ev = ev_of(rows)
    cell = ev.col.cells[("omnisim", f_eval.TOOLS, B[0])]
    assert cell.n_total == 5                       # the extras never landed
    assert ev.col.excluded == {"superseded": 1, "exploratory": 2}


def test_sim_read_from_campaign_block_over_row_sim():
    # the run_agentbench child always writes sim="omnisim"; campaign.sim is
    # the scheduled truth (the known interface gap, campaign.py docstring)
    rows = cell_rows(B[0], f_eval.SHELL, 3, sim="webots")
    ev = ev_of(rows)
    assert ("webots", f_eval.SHELL, B[0]) in ev.col.cells
    assert ("omnisim", f_eval.SHELL, B[0]) not in ev.col.cells


# ---------------------------------------------------------------------------
# the report emitter and its forbidden-sentence self-check
# ---------------------------------------------------------------------------

def test_check_forbidden_catches_each_pattern():
    bads = ["OmniSim is faster than everything",
            "the most agent-driveable simulator",
            "agents get more done on OmniSim",
            "our harness pays for itself now",
            "the claim survived F",
            "we are the first simulator to do this",
            "Webots FAILED the hot-reload task",
            "4 of 4 frontier capabilities demonstrated",
            "better than Gazebo in every cell"]
    for bad in bads:
        assert f_eval.check_forbidden(bad, ["omnisim"]), bad


def test_check_forbidden_than_webots_depends_on_sims_present():
    txt = "more outcomes than Webots on this set"
    assert f_eval.check_forbidden(txt, ["omnisim"])            # webots not run
    assert not f_eval.check_forbidden(txt, ["omnisim", "webots"])


def test_emit_refuses_forbidden_output(tmp_path, monkeypatch):
    ev = ev_of(surface_rows({t: (5, 8, 3, 20) for t in B}))
    monkeypatch.setattr(f_eval, "render",
                        lambda _ev: "this report says the claim survived F")
    out = tmp_path / "report.md"
    with pytest.raises(f_eval.ForbiddenSentenceError):
        f_eval.emit(ev, out)
    assert not out.exists()                # never written


def test_emit_writes_clean_report_with_required_header(tmp_path):
    ev = ev_of(surface_rows({t: (5, 8, 3, 20) for t in B})
               + cell_rows(A1, f_eval.SHELL, 9, n=10, calls=30)
               + cell_rows(A1, f_eval.TOOLS, 9, n=10, calls=40))
    out = f_eval.emit(ev, tmp_path / "report.md")
    text = out.read_text(encoding="utf-8")
    # header disclosures (plan §2.4 / §2.3)
    assert "Resolution limit" in text
    assert "conventional, not power-derived" in text
    assert "fixture-machine" in text                   # machine id
    assert "claude-test-1" in text                     # model id
    assert "Wilson" in text
    assert "never summed" in text                      # tokens rule
    assert "t_agent_s leading" in text
    # Lane A reported separately
    assert A1 in text and "never in the decision set" in text
    # traceability to rows sources
    assert "fixture#L1" in text
    # tokens columns separate, t_agent_s before t_total_s in the table
    assert "| t_agent_s | t_total_s |" in text


def test_wilson_interval():
    assert f_eval.wilson(0, 0) is None
    lo, hi = f_eval.wilson(5, 5)
    assert lo > 0.5 and hi == 1.0
    lo, hi = f_eval.wilson(2, 5)
    assert 0.0 < lo < 0.4 < hi < 0.9


def test_cli_main_writes_report(tmp_path):
    rows_path = tmp_path / "rows.jsonl"
    import json
    with open(rows_path, "w", encoding="utf-8") as fh:
        for r in surface_rows({t: (5, 8, 3, 20) for t in B}):
            fh.write(json.dumps(r) + "\n")
    out = tmp_path / "f_report.md"
    rc = f_eval.main(["--rows", str(rows_path), "--out", str(out)])
    assert rc == 0
    assert out.is_file()
    assert str(rows_path) in out.read_text(encoding="utf-8")
