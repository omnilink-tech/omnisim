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

"""Unit tests for the ladder's agent-cell runner.

**No Claude session, no engine, no GPU, no network.** Every test here runs on
a synthetic tree or on the repo's own file list, and the two that exercise a
whole cell inject a stub session and a stub grader -- which is the point of
those injection points existing.

What each block is defending, in the order the plan cares about:

* the **quarantine**: the ladder's answer key (graders, thresholds, oracles,
  the plan, the endurance doc) can never reach a workspace -- proven on a
  synthetic tree AND on this repo's real tracked file list;
* the **container**: exactly the files ``meta.json`` declares, and a
  present-but-undeclared file is reported rather than shipped;
* the **deliverable**: each column's declared convention, and the honest
  refusal for a column that has none;
* the **row**: ``capability-ladder-plan.md`` §3's taxonomy, its cost fields
  with unmeasured as ``null``, and its blocker codes;
* the **protocol**: resume, a usage limit as a deferral rather than a run, and
  the row on disk BEFORE any teardown.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
LADDER = HERE.parent
BENCHMARKS = LADDER.parent
REPO = BENCHMARKS.parents[1]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import stage_workspaces as cc         # noqa: E402
from ladder import tasks as ladder_tasks                      # noqa: E402
from ladder.cell import oracles                               # noqa: E402
from ladder.cell import run_ladder_campaign as camp           # noqa: E402
from ladder.cell import run_ladder_cell as cell               # noqa: E402
from ladder.cell import stage_ladder_workspace as staging     # noqa: E402

ALL_TASKS = ("T1_arrive", "T2_transfer", "T3_quadruped", "T4_humanoid")


# --- the quarantine -----------------------------------------------------------


def test_select_ladder_files_excludes_the_ladder_answer_key():
    """Graders, cores, fixtures, meta.json, containers and the committed
    ORACLES -- every one of them is the answer, and none may be staged."""
    tracked = [
        "AGENTS.md",
        "tests/benchmarks/ladder/graders/t1_core.py",
        "tests/benchmarks/ladder/graders/fixtures_t4.py",
        "tests/benchmarks/ladder/tasks/T1_arrive/meta.json",
        "tests/benchmarks/ladder/tasks/T4_humanoid/container/ARENA.txt",
        "tests/benchmarks/ladder/adapters/mujoco/run_t3.py",
        "tests/benchmarks/ladder/adapters/mujoco/BRINGUP_T3.md",
        "tests/benchmarks/ladder/cell/run_ladder_cell.py",
        "docs/developer/capability-ladder-plan.md",
        "docs/developer/g1-endurance-2026-08-01.md",
        "tests/benchmarks/agentbench/graders/c2_core.py",
        "social/funding/plan.md",
        "cloud/runpod/README.md",
    ]
    copied, excluded = staging.select_ladder_files(tracked)
    assert copied == ["AGENTS.md"]
    for rel in tracked[1:]:
        assert rel in excluded, "%s reached the workspace" % rel


def test_select_ladder_files_keeps_the_product():
    """The quarantine removes the answer key, not the product. A workspace
    that is not the product measures nothing anybody ships."""
    tracked = ["AGENTS.md", "README.md", "PROTOCOL.md",
               "docs/developer/skill-library.md",
               "docs/developer/quickstart.md",
               "scripts/harness/omnisim_harness.py",
               "tests/benchmarks/omnibench/run_all.py",
               "src/omnisim/gui/main.cpp"]
    copied, excluded = staging.select_ladder_files(tracked)
    assert set(copied) == set(tracked)
    assert excluded == []


def test_junctioned_dirs_are_not_copied():
    copied, excluded = staging.select_ladder_files(
        ["projects/robots/x.urdf", "lib/controller/python/omnisim/__init__.py",
         "msys64/mingw64/bin/x.dll", "resources/x.png", "AGENTS.md"])
    assert copied == ["AGENTS.md"]
    assert excluded == []          # reached through the junction, not excluded


def test_the_repo_s_real_tracked_tree_stages_no_ladder_answer_key():
    """The exclusion, proven against THIS repo rather than a fixture.

    A synthetic list proves the rule; this proves the rule covers the tree it
    will actually be run on -- including any ladder file added after the rule
    was written.
    """
    out = subprocess.run(["git", "-C", str(REPO), "ls-files", "-z"],
                         capture_output=True, check=True)
    tracked = [p for p in out.stdout.decode("utf-8", "replace").split("\0")
               if p]
    copied, excluded = staging.select_ladder_files(tracked)
    leaked = [p for p in copied
              if p.startswith(staging.QUARANTINE_PATH_PREFIXES)
              or p in staging.QUARANTINE_PATH_FILES]
    assert leaked == [], "answer key would be staged: %s" % leaked[:10]
    # and the two named docs really are in the tree (so this is not vacuous)
    assert "docs/developer/capability-ladder-plan.md" in excluded
    assert "docs/developer/g1-endurance-2026-08-01.md" in excluded
    assert any(p.startswith("tests/benchmarks/ladder/") for p in excluded)


def test_answer_key_violations_finds_planted_material(tmp_path):
    ws = tmp_path / "ws"
    (ws / "tests" / "benchmarks" / "ladder" / "graders").mkdir(parents=True)
    (ws / "tests/benchmarks/ladder/graders/t1_core.py").write_text(
        "x = 1\n", encoding="utf-8")
    (ws / "docs").mkdir()
    (ws / "docs" / "leak.md").write_text(
        "The bar is ARRIVE_TOL_M = 0.25 metres.\n", encoding="utf-8")
    (ws / "docs" / "plan.md").write_text(
        "see the capability ladder plan\n", encoding="utf-8")
    (ws / "clean.md").write_text("the product does things\n", encoding="utf-8")

    hits = staging.answer_key_violations(ws)
    paths = {h["path"] for h in hits}
    assert "tests/benchmarks/ladder/graders/t1_core.py" in paths
    assert "docs/leak.md" in paths
    assert "docs/plan.md" in paths
    assert "clean.md" not in paths
    kinds = {h["path"]: h["kind"] for h in hits}
    assert kinds["tests/benchmarks/ladder/graders/t1_core.py"] \
        == "quarantined_path"
    assert kinds["docs/leak.md"] == "answer_key_content"


def test_threshold_markers_are_case_sensitive(tmp_path):
    """The measured false positive, kept as a test.

    Compiled case-insensitively, the constant markers matched
    ``net_displacement_m`` / ``arrive_tol_m`` in the unrelated
    ``tests/benchmarks/warehouse/`` tool, and every omnisim cell would have
    been INVALID on four false positives before a session ran. A leaked bar
    is SCREAMING_SNAKE; an ordinary physical column name is not.
    """
    ws = tmp_path / "ws"
    ws.mkdir()
    (ws / "ordinary.py").write_text(
        'out = {"net_displacement_m": x, "arrive_tol_m": 0.7}\n',
        encoding="utf-8")
    (ws / "leaked.py").write_text("NET_DISPLACEMENT_M = 10.0\n",
                                  encoding="utf-8")
    paths = {h["path"] for h in staging.answer_key_violations(ws)}
    assert paths == {"leaked.py"}


def test_answer_key_violations_skips_the_tier_s_own_container(tmp_path):
    """``container/PROVENANCE.txt`` legitimately names its own tier. It is the
    task, deliberately shipped, and must not read as a leak."""
    ws = tmp_path / "ws"
    (ws / "container").mkdir(parents=True)
    (ws / "container" / "PROVENANCE.txt").write_text(
        "T4_humanoid -- what is in this container\n", encoding="utf-8")
    assert staging.answer_key_violations(ws) == []


def test_transcript_answer_key_hits(tmp_path):
    good = tmp_path / "clean.jsonl"
    good.write_text('{"type":"assistant"}\n', encoding="utf-8")
    assert staging.transcript_answer_key_hits(good) == ([], None)

    bad = tmp_path / "leak.jsonl"
    bad.write_text('{"text":"cat tests/benchmarks/ladder/graders/t1_core.py"}',
                   encoding="utf-8")
    hits, reason = staging.transcript_answer_key_hits(bad)
    assert hits and reason is None

    hits, reason = staging.transcript_answer_key_hits(None)
    assert hits == [] and reason           # unchecked, never "clean"


def test_ladder_redaction_extends_then_restores():
    before = cc.REDACT_PATTERNS
    with staging.ladder_redaction():
        assert len(cc.REDACT_PATTERNS) > len(before)
        text, reds = cc.redact_markdown(
            "The engine loads worlds.\n\nThis is what the capability ladder "
            "grades.\n")
        assert "capability ladder" not in text
        assert "The engine loads worlds." in text
        assert reds
    assert cc.REDACT_PATTERNS is before


# --- the container --------------------------------------------------------------


@pytest.mark.parametrize("task_id", ALL_TASKS)
def test_every_declared_container_file_exists(task_id):
    task = ladder_tasks.get(task_id)
    declared = (task.meta.get("container", {}) or {}).get("files", [])
    assert declared, "%s declares no container files" % task_id
    missing = [f for f in declared if not (task.container_dir / f).is_file()]
    assert missing == []


@pytest.mark.parametrize("task_id", ALL_TASKS)
def test_stage_container_stages_exactly_what_is_declared(tmp_path, task_id):
    task = ladder_tasks.get(task_id)
    ws = tmp_path / task_id
    ws.mkdir()
    prompt, rec = staging.stage_container(ws, task)

    # The tier's own sentence is delivered verbatim and FIRST; the only
    # addition is the instrument note, which is identical on every column and
    # every tier (see staging.ENVIRONMENT_NOTE for why withholding it would
    # measure familiarity with one CLI's timeout rather than capability).
    assert prompt.startswith(task.prompt) and task.prompt.strip()
    assert prompt == task.prompt + staging.ENVIRONMENT_NOTE
    declared = {f.replace("\\", "/") for f in rec["declared"]}
    assert set(rec["staged"]) == declared
    assert rec["declared_but_missing"] == []
    on_disk = {p.relative_to(ws / "container").as_posix()
               for p in (ws / "container").rglob("*") if p.is_file()}
    assert on_disk == declared, "the workspace holds files the task did not " \
                                "declare: %s" % (on_disk - declared)
    # nothing else task-related anywhere in the workspace
    outside = {p.relative_to(ws).as_posix() for p in ws.rglob("*")
               if p.is_file() and not p.relative_to(ws).parts[0] == "container"}
    assert outside == set()
    # and the layout survives (T1's package:// resolution depends on it)
    if task_id == "T1_arrive":
        assert (ws / "container" / "husky_description" / "urdf"
                / "husky.urdf").is_file()


def test_stage_container_reports_present_but_undeclared(tmp_path,
                                                        monkeypatch):
    task = ladder_tasks.get("T3_quadruped")

    class _Proxy:
        id = task.id
        meta = task.meta
        prompt = task.prompt
        container_dir = tmp_path / "src"

    (_Proxy.container_dir / "walker_description" / "urdf").mkdir(parents=True)
    (_Proxy.container_dir / "walker_description" / "urdf"
     / "walker.urdf").write_text("<robot/>", encoding="utf-8")
    (_Proxy.container_dir / "PROVENANCE.txt").write_text("x", encoding="utf-8")
    (_Proxy.container_dir / "SOLUTION.txt").write_text("here is the gait",
                                                       encoding="utf-8")
    ws = tmp_path / "ws"
    ws.mkdir()
    _prompt, rec = staging.stage_container(ws, _Proxy())
    assert rec["present_but_undeclared_NOT_staged"] == ["SOLUTION.txt"]
    assert not (ws / "container" / "SOLUTION.txt").exists()


# --- the deliverable --------------------------------------------------------------


def _touch(p, mtime, text="x"):
    import os
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")
    os.utime(p, (mtime, mtime))
    return p


def test_discover_deliverable_world(tmp_path):
    ws = tmp_path / "ws"
    _touch(ws / "worlds" / "old.wbt", 1000)
    new = _touch(ws / "worlds" / "mine.wbt", 3000)
    path, kind, note, diag = cell.discover_deliverable("omnisim", ws, 2000.0)
    assert path == new and kind == "world" and note
    assert diag["source"] == "workspace" and not diag["ambiguous"]


def test_discover_deliverable_scene_dir(tmp_path):
    ws = tmp_path / "ws"
    d = ws / "build" / "deliverable"
    d.mkdir(parents=True)
    (d / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (d / "drive.py").write_text("pass", encoding="utf-8")
    path, kind, note, diag = cell.discover_deliverable("mujoco", ws, 0.0)
    assert path == d and kind == "scene_dir" and not diag["ambiguous"]


def test_discover_deliverable_unknown_column_is_ours(tmp_path):
    path, kind, note, diag = cell.discover_deliverable("gazebo", tmp_path, 0.0)
    assert path is None and kind is None
    assert "blocker is OURS" in note
    assert diag["instrument_blocker"] == "deliverable_convention_missing"


def test_discover_deliverable_none_when_the_session_wrote_nothing(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    path, _kind, note, diag = cell.discover_deliverable("omnisim", ws, 0.0)
    assert path is None and note
    # nothing swept either -- so this is NOT an instrument blocker, it is a
    # session that left nothing, which classify() reads as agent_lost.
    assert not diag["ambiguous"] and diag["instrument_blocker"] is None


# --- the deliverable: the 2026-08-02 omnisim/T2 defect, three ways -----------


def test_the_workspace_deliverable_beats_a_planted_repo_artifact(tmp_path):
    """⚠ THE REGRESSION. Reproduces ``20260802_155111_omnisim_T2`` in
    miniature: the agent's authored world in the workspace, and NEWER
    repo-swept artifacts preserved beside the cell. The old rule (delegate to
    ``cc_lane.discover_artifact`` with ``repo_artifacts`` as an extra root)
    ranked the swept artifact ahead of the workspace and graded a throwaway;
    the deliverable must now be the workspace file, and the swept files must
    survive as evidence and appear nowhere else."""
    ws = tmp_path / "ws"
    mine = _touch(ws / "projects" / "worlds" / "omniarm6_block_in_bin.wbt", 3000)
    swept = tmp_path / "cell" / "repo_artifacts" / "projects" / "worlds"
    planted = _touch(swept / "omniarm6_block_in_bin.wbt", 9000)      # NEWER
    probe = _touch(swept / ".omnisim_probe_pyramidal_mu2.wbt", 9500)

    path, kind, note, diag = cell.discover_deliverable(
        "omnisim", ws, 2000.0,
        swept_roots=(tmp_path / "cell" / "repo_artifacts",))

    assert path == mine, "a swept artifact must never outrank the workspace"
    assert kind == "world" and not diag["ambiguous"]
    assert diag["source"] == "workspace"
    # preserved, still on disk, and named as evidence -- never as output
    assert planted.is_file() and probe.is_file()
    assert str(planted) in diag["swept_evidence"]
    assert str(planted) not in diag["candidates"]
    assert "evidence only" in note


def test_a_tool_generated_probe_world_is_never_the_deliverable(tmp_path):
    """The other half of the same defect: the winning file was a
    ``.omnisim_probe_*.wbt`` the session's own sweep script wrote. A
    dot-prefixed ``.wbt`` is this tree's convention for a generated sibling
    (``.omnisim_runaway_*``), so it is machinery, not a deliverable."""
    ws = tmp_path / "ws"
    mine = _touch(ws / "w" / "scene.wbt", 3000)
    for i, cfg in enumerate(("ellip10_default", "ellip30_gentle_mu2",
                             "pyramidal_mu2")):
        _touch(ws / "w" / (".omnisim_probe_%s.wbt" % cfg), 5000 + i)
    _touch(ws / "w" / "_agentbench_phaseB_scene.wbt", 6000)

    path, _kind, _note, diag = cell.discover_deliverable("omnisim", ws, 2000.0)
    assert path == mine
    assert diag["candidates"] == [str(mine)]
    assert cell.is_tool_generated_world(".omnisim_runaway_scene.wbt")
    assert cell.is_tool_generated_world("_agentbench_phaseB_scene.wbt")
    assert not cell.is_tool_generated_world("scene.wbt")


def test_only_swept_evidence_is_an_instrument_blocker_not_agent_lost(tmp_path):
    """Nothing in the workspace, and the sweep holds the session's output:
    that is our discovery walk failing, so the cell must go INVALID with a
    named instrument blocker -- never ``agent_lost``, which blames the
    agent."""
    ws = tmp_path / "ws"
    ws.mkdir()
    swept = tmp_path / "repo_artifacts"
    _touch(swept / "projects" / "worlds" / "mine.wbt", 9000)

    path, _kind, note, diag = cell.discover_deliverable(
        "omnisim", ws, 2000.0, swept_roots=(swept,))
    assert path is None
    assert diag["ambiguous"]
    assert diag["instrument_blocker"] == "deliverable_not_in_workspace"
    assert "REAL REPO" in note

    value, blocker, basis = cell.classify(
        session_status="ok", deliverable_found=False,
        deliverable_rule_known=True, grade_error=None, verdict=None,
        deliverable_ambiguity=diag["ambiguity_reason"])
    assert (value, blocker) == (None, None) and "INVALID" in basis


def test_two_candidates_inside_the_tie_window_are_ambiguous(tmp_path):
    """"Newest" is not a rule when two files land in the same moment."""
    ws = tmp_path / "ws"
    _touch(ws / "a.wbt", 5000.0)
    _touch(ws / "b.wbt", 5000.5)
    path, _kind, _note, diag = cell.discover_deliverable("omnisim", ws, 1000.0)
    assert path is not None                      # a best guess is still made
    assert diag["ambiguous"]
    assert diag["instrument_blocker"] == "deliverable_ambiguous"
    assert "does not separate them" in diag["ambiguity_reason"]


def test_outside_the_tie_window_newest_is_a_real_ordering(tmp_path):
    ws = tmp_path / "ws"
    _touch(ws / "a.wbt", 5000.0)
    b = _touch(ws / "b.wbt", 5000.0 + cell.DELIVERABLE_TIE_WINDOW_S + 1)
    path, _kind, _note, diag = cell.discover_deliverable("omnisim", ws, 1000.0)
    assert path == b and not diag["ambiguous"]


def test_every_column_declares_a_workspace_path(tmp_path):
    """The rule "a swept repo artifact is not gradeable output" only has an
    exception for a column that declares no workspace path. None does -- so
    the exception is a guarded refusal, not a live fallback."""
    for col, rule in cell.DELIVERABLE_RULES.items():
        assert rule.get("workspace_path") is True, col


def test_instrument_blockers_are_disjoint_from_the_section_3_3_taxonomy():
    """§3.3's codes explain a ``not_achieved`` cell. An instrument blocker
    accompanies an INVALID one, and the two vocabularies must not be
    confusable in a published row."""
    assert not (set(cell.INSTRUMENT_BLOCKERS) & set(cell.BLOCKERS))


# --- the blocker taxonomy ---------------------------------------------------------


class _V:
    """The minimum of a Verdict the classifier reads."""

    def __init__(self, outcome="FAIL", failed=(), progress=3,
                 measurements=None):
        self.outcome = outcome
        self.failed = list(failed)
        self.progress = progress
        self.measurements = measurements or {}


def test_classify_timeout_is_budget_exhausted():
    v, b, why = cell.classify(session_status="timeout",
                              deliverable_found=False,
                              deliverable_rule_known=True, grade_error=None,
                              verdict=None)
    assert (v, b) == ("not_achieved", "budget_exhausted") and why


def test_classify_no_deliverable_is_agent_lost():
    v, b, _ = cell.classify(session_status="ok", deliverable_found=False,
                            deliverable_rule_known=True, grade_error=None,
                            verdict=None)
    assert (v, b) == ("not_achieved", "agent_lost")


def test_classify_unknown_column_is_ours_not_the_simulator_s():
    v, b, _ = cell.classify(session_status="ok", deliverable_found=False,
                            deliverable_rule_known=False, grade_error=None,
                            verdict=None)
    assert (v, b) == ("not_achieved", "scaffolding_defect_ours")


def test_classify_missing_phase_b_runner_is_ours():
    v, b, _ = cell.classify(session_status="ok", deliverable_found=True,
                            deliverable_rule_known=True,
                            grade_error="NotImplementedError(...)",
                            verdict=None)
    assert (v, b) == ("not_achieved", "scaffolding_defect_ours")


def test_classify_pass_is_achieved():
    v, b, _ = cell.classify(session_status="ok", deliverable_found=True,
                            deliverable_rule_known=True, grade_error=None,
                            verdict=_V(outcome="PASS", progress=4))
    assert (v, b) == ("achieved", None)


def test_classify_reds_that_only_our_missing_channels_could_cause():
    """§4's binding rule, mechanically: a red assertion that depends only on a
    channel WE never built is never published as somebody else's gap."""
    v = _V(failed=["T3.4"], progress=3,
           measurements={"unanswered_channels": {"applied_support": ["T3.4"]}})
    value, blocker, why = cell.classify(
        session_status="ok", deliverable_found=True,
        deliverable_rule_known=True, grade_error=None, verdict=v)
    assert (value, blocker) == ("not_achieved", "scaffolding_defect_ours")
    assert "T3.4" in why


def test_classify_a_real_physical_shortfall_is_the_simulator_s():
    v = _V(failed=["T3.1"], progress=3, measurements={})
    value, blocker, _ = cell.classify(
        session_status="ok", deliverable_found=True,
        deliverable_rule_known=True, grade_error=None, verdict=v)
    assert (value, blocker) == ("not_achieved", "no_physics_capability")


def test_classify_invalid_is_not_a_blocker():
    value, blocker, why = cell.classify(
        session_status="ok", deliverable_found=True,
        deliverable_rule_known=True, grade_error=None,
        verdict=_V(outcome="INVALID", progress=2))
    assert value is None and blocker is None and "INVALID" in why


def test_every_blocker_the_classifier_can_emit_is_in_the_taxonomy():
    cases = [
        dict(session_status="timeout", deliverable_found=False,
             deliverable_rule_known=True, grade_error=None, verdict=None),
        dict(session_status="launch_failed", deliverable_found=False,
             deliverable_rule_known=True, grade_error=None, verdict=None),
        dict(session_status="error", deliverable_found=False,
             deliverable_rule_known=True, grade_error=None, verdict=None),
        dict(session_status="oracle_missing", deliverable_found=False,
             deliverable_rule_known=True, grade_error=None, verdict=None),
        dict(session_status="oracle_failed", deliverable_found=False,
             deliverable_rule_known=True, grade_error=None, verdict=None),
        dict(session_status="ok", deliverable_found=True,
             deliverable_rule_known=True, grade_error=None,
             verdict=_V(failed=["X"], progress=1)),
        dict(session_status="ok", deliverable_found=True,
             deliverable_rule_known=True, grade_error=None,
             verdict=_V(failed=["X"], progress=2), phase_b_broke=True),
        dict(session_status="ok", deliverable_found=True,
             deliverable_rule_known=True, grade_error=None,
             verdict=_V(failed=["X"], progress=2)),
    ]
    for kw in cases:
        _v, blocker, _why = cell.classify(**kw)
        assert blocker in cell.BLOCKERS, blocker


def test_an_ambiguous_deliverable_outranks_every_agent_attributable_branch():
    """The ordering matters as much as the rule: an ambiguous deliverable must
    beat ``agent_lost`` (no deliverable), ``no_physics_capability`` (reds) and
    ``install_failed`` (phase B produced nothing) -- all three of which read as
    somebody else's verdict on a measurement we broke. The 2026-08-02 cell
    landed on ``install_failed``."""
    for kw in (
        dict(deliverable_found=False, verdict=None),
        dict(deliverable_found=True, verdict=_V(failed=["T2.1"], progress=3)),
        dict(deliverable_found=True, verdict=_V(failed=["T2.1"], progress=1),
             phase_b_broke=True),
    ):
        value, blocker, basis = cell.classify(
            session_status="ok", deliverable_rule_known=True,
            grade_error=None,
            deliverable_ambiguity="two candidates, no rule", **kw)
        assert (value, blocker) == (None, None)
        assert "INVALID rather than not_achieved" in basis


def test_a_timeout_still_says_the_agent_was_working():
    _v, blocker, basis = cell.classify(
        session_status="timeout", deliverable_found=False,
        deliverable_rule_known=True, grade_error=None, verdict=None)
    assert blocker == "budget_exhausted"
    assert "WORKING" in basis and "did not fail at the task" in basis


# --- the session cap, and how a session ended ---------------------------------------


class _Sess:
    def __init__(self, status="ok", **meta):
        self.status = status
        self.meta = meta


def test_the_session_cap_is_explicit_and_names_its_source():
    task = ladder_tasks.get("T2_transfer")
    cap, src = cell.session_cap(task)
    assert cap == float(task.timeout_s) and "meta.json" in src
    cap, src = cell.session_cap(task, 900)
    assert cap == 900.0 and "override" in src

    class _NoCap:
        id, rung, par_s, timeout_s = "X", "T9", 100, None
    cap, src = cell.session_cap(_NoCap())
    assert cap == cell.DEFAULT_SESSION_CAP_S and "package default" in src


def test_the_measured_cost_table_records_the_run_the_cap_was_sized_from():
    m = cell.MEASURED_SESSION_COST[("T2_transfer", "omnisim")]
    assert m["cap_hit"] is False
    assert m["t_agent_s"] < m["cap_s"]        # the cap was NOT what ended it
    assert "20260802_155111_omnisim_T2" in m["cell"]


def test_a_cap_that_landed_on_a_working_agent_is_recorded_distinctly():
    end = cell.classify_session_end(_Sess("timeout", timed_out=True,
                                          wall_s=5400.0), 5400.0)
    assert end["reason"] == "cap_reached" and end["was_working"] is True
    assert end["fraction_of_cap"] == 1.0


def test_a_clean_finish_is_not_recorded_as_working(tmp_path):
    end = cell.classify_session_end(_Sess("ok", wall_s=100.0), 5400.0,
                                    transcript=None)
    assert end["reason"] == "completed" and end["was_working"] is False


def test_a_session_that_yielded_to_a_background_task_is_its_own_outcome(
        tmp_path):
    """⚠ WHAT ACTUALLY ENDED THE FIRST PAID LADDER CELL, and it was NOT the
    cap: at 2266.8 s of 5400 s the CLI auto-backgrounded a >10-minute
    foreground command and the agent ended its turn to await a notification
    that headless ``claude -p`` never delivers."""
    tr = tmp_path / "transcript.jsonl"
    tr.write_text(json.dumps({
        "type": "user", "message": {"content": [{
            "type": "tool_result",
            "content": "Command running in background with ID: bdinwal2p. "
                       "Output -> ...tasks/bdinwal2p.output. "
                       "You will be notified when it completes."}]}}) + "\n",
        encoding="utf-8")
    assert cell.pending_background_tasks(tr) == ["bdinwal2p"]

    end = cell.classify_session_end(_Sess("ok", wall_s=2266.8), 5400.0,
                                    transcript=tr)
    assert end["reason"] == "background_work_pending"
    assert end["was_working"] is True
    assert end["fraction_of_cap"] == 0.4198     # 42% of the cap, not 100%
    assert end["pending_background_tasks"] == ["bdinwal2p"]


def test_a_background_task_that_reported_back_is_not_pending(tmp_path):
    tr = tmp_path / "transcript.jsonl"
    tr.write_text(
        "Command running in background with ID: abc123. You will be notified\n"
        "later: shell abc123 Exit code 0\n", encoding="utf-8")
    assert cell.pending_background_tasks(tr) == []


# --- the row ------------------------------------------------------------------------


def _row(**kw):
    task = ladder_tasks.get(kw.pop("task_id", "T4_humanoid"))
    base = dict(
        task=task, column="omnisim", repeat=0, cell_value="not_achieved",
        blocker="agent_lost", blocker_basis="because",
        verdict=None, metrics=cell.empty_metrics(),
        agent_block={"kind": "claude_code", "model_pinned": "claude-opus-5"},
        workspace_block={}, artifacts={}, notes=[], deviations=[],
        machine={"id": "test"}, dry_run=False, t_total_s=1.0,
        answer_key={"workspace_clean": True})
    base.update(kw)
    return cell.build_row(**base)


def test_row_carries_the_plan_section_3_taxonomy():
    row = _row()
    assert row["suite"] == "ladder/v0"
    assert row["cell"]["value"] in cell.CELL_VALUES
    assert row["cell"]["blocker"] in cell.BLOCKERS
    assert row["cell"]["blocker_owner"]
    assert row["cell"]["not_expressible"] is None
    assert row["cell"]["review"]["relabelled_by"] is None
    assert row["rung"] == "T4" and row["task"] == "T4_humanoid"
    assert row["scoring_class"] == "exploratory"
    assert "never quotable as a pass rate" in row["achieved_rule"]
    assert row["expectation"], "the pre-registration must ride in the row"


def test_row_cost_fields_are_null_never_zero():
    row = _row()
    m = row["metrics"]
    for k in ("tool_calls", "turns", "t_agent_s", "tokens_in", "tokens_out",
              "tokens_cache_read", "tokens_cache_write", "gpu_hours", "usd"):
        assert m[k] is None, "%s was defaulted rather than measured" % k
    assert m["interventions"] == 0
    assert row["interventions"] == 0
    assert row["metrics_source"] == cell.METRICS_SOURCE_AGENT
    assert "NEVER summed" in row["metrics_notes"]["tokens"]


def test_row_carries_every_provenance_sub_label():
    row = _row()
    prov = row["provenance"]
    for k in ("reuse_class", "method", "hold_mechanism", "external_support",
              "support_attestation", "distance_to_termination_m",
              "termination_cause", "venue"):
        assert k in prov
    assert prov["reuse_class"] is None
    assert "not publishable" in prov["reuse_class_note"].lower()


def test_row_pulls_the_tier_measurements_the_grader_produced():
    """T4's two required fields ride from the verdict into the row, un-clipped
    and un-invented."""
    class _Verdict:
        def as_dict(self):
            return {"outcome": "PASS", "progress": 4,
                    "progress_name": "graded_pass",
                    "assertions": {"T4.1": {"ok": True}},
                    "failed_assertion": None, "failed_assertions": [],
                    "notes": [], "measurements": {
                        "distance_to_termination_m": 24.94,
                        "termination_cause": "arena_geometry",
                        "cell": "T4-supported", "method": "scripted",
                        "reuse_class": None,
                        "external_support": {"peak_force_fraction": 0.0205}}}
    row = _row(verdict=_Verdict(), cell_value="achieved", blocker=None)
    assert row["provenance"]["distance_to_termination_m"] == 24.94
    assert row["provenance"]["termination_cause"] == "arena_geometry"
    assert row["provenance"]["support_cell"] == "T4-supported"
    assert row["outcome"] == "PASS"


def test_an_invalid_reason_overrides_the_cell_value():
    row = _row(invalid_reason="answer key in the workspace")
    assert row["outcome"] == "INVALID"
    assert row["cell"]["value"] is None and row["cell"]["blocker"] is None
    assert row["cell"]["invalid_reason"]


def test_dry_run_rows_are_marked_and_are_not_cells():
    row = _row(dry_run=True)
    assert row["dry_run"] is True
    assert row["condition"] == cell.DRY_CONDITION
    assert row["metrics_source"] == cell.METRICS_SOURCE_DRY


# --- the group reading ---------------------------------------------------------------


def test_group_summary_is_solved_at_least_once_never_a_pass_rate():
    rows = [{"cell_key": "T1_arrive:omnisim:r0",
             "cell": {"value": "not_achieved", "blocker": "agent_lost"}},
            {"cell_key": "T1_arrive:omnisim:r1",
             "cell": {"value": "achieved"}},
            {"cell_key": "T1_arrive:omnisim:r2",
             "cell": {"value": "not_achieved", "blocker": "agent_lost"}}]
    s = camp.summarise_group(rows, "T1_arrive")
    assert s["solved_at_least_once"] is True
    assert s["cell_value"] == "achieved"
    assert s["n_achieved"] == 1 and s["n_runs"] == 3
    assert "pass_rate" not in s
    assert "never quotable as a pass rate" in s["rule"].lower()


def test_a_re_run_cell_replaces_its_own_row_rather_than_doubling_it(tmp_path):
    """The wrong denominator, measured on the first full dry run: an
    interrupted campaign re-ran three cells and the group summarised itself
    over SIX rows."""
    g = tmp_path / "grp"
    g.mkdir()
    r0 = {"cell_key": "T1_arrive:omnisim:r0", "cell": {"value": "not_achieved",
                                                       "blocker": "unknown"}}
    camp.Campaign._append_group_row(g, r0)
    camp.Campaign._append_group_row(g, dict(r0, cell={"value": "achieved"}))
    camp.Campaign._append_group_row(
        g, {"cell_key": "T1_arrive:omnisim:r1", "cell": {"value": "achieved"}})
    rows = [json.loads(ln) for ln in
            (g / "rows.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 2
    s = camp.summarise_group(rows, "T1_arrive")
    assert s["n_runs"] == 2 and s["n_achieved"] == 2


def test_a_group_of_only_invalid_cells_has_no_cell_value():
    """An INVALID cell is not a measurement, so a group of them is not a
    ``not_achieved`` (§3.1). Measured 2026-08-02 on the corrected
    ``T2_transfer:omnisim:r0`` row: the group read ``not_achieved`` from one
    cell in which nothing about the agent was measured."""
    rows = [{"cell_key": "T2_transfer:omnisim:r0",
             "cell": {"value": None, "blocker": None,
                      "invalid_reason": "graded the wrong file",
                      "instrument_blocker": "deliverable_not_in_workspace"}}]
    s = camp.summarise_group(rows, "T2_transfer")
    assert s["cell_value"] is None
    assert s["n_invalid"] == 1 and s["n_measured"] == 0
    assert s["not_measured_warning"]
    assert s["solved_at_least_once"] is False


def test_the_corrected_T2_row_on_disk_reads_as_invalid_and_not_measured():
    """The real row, as it now sits in ``results/``: two rows for one
    cell_key, the original struck through, the correction winning the dedupe,
    and the group refusing to call it a result."""
    p = (LADDER / "results" / "ladder_cell"
         / "20260802_155111_omnisim_T2" / "rows.jsonl")
    if not p.is_file():                     # results/ is not always present
        pytest.skip("the 2026-08-02 cell dir is not in this checkout")
    rows = [json.loads(ln) for ln in
            p.read_text(encoding="utf-8").splitlines() if ln.strip()]
    assert len(rows) == 2
    old, new = rows
    assert old["ladder"]["superseded"] is True
    assert old["cell"]["value"] == "not_achieved"       # still visible
    assert old["cell"]["blocker"] == "install_failed"
    assert new["outcome"] == "INVALID"
    assert new["cell"]["value"] is None and new["cell"]["blocker"] is None
    assert new["cell"]["instrument_blocker"] in cell.INSTRUMENT_BLOCKERS
    assert new["superseded_verdict"]["blocker"] == "install_failed"
    assert new["session_end"]["reason"] == "background_work_pending"
    assert new["session_end"]["was_working"] is True
    s = camp.summarise_group(rows, "T2_transfer")
    assert s["n_runs"] == 1 and s["cell_value"] is None


def test_group_summary_flags_a_column_hiding_behind_unknown():
    rows = [{"cell": {"value": "not_achieved", "blocker": "unknown"}}] * 3
    s = camp.summarise_group(rows, "T2_transfer")
    assert s["modal_blocker"] == "unknown"
    assert s["not_diagnosed_warning"]


# --- the protocol ---------------------------------------------------------------------


def test_a_usage_limit_is_a_deferral_not_a_run(monkeypatch, tmp_path):
    calls = {"n": 0}

    def _fake(prompt, workspace, env, run_dir, *, model, timeout_s):
        calls["n"] += 1
        if calls["n"] == 1:
            return ({"is_error": True,
                     "result": "You've hit your session limit · resets "
                               "7:10pm"}, {"permission_mode": "x"})
        return ({"is_error": False, "result": "done", "session_id": "s",
                 "num_turns": 3}, {"permission_mode": "x", "wall_s": 1.0})

    monkeypatch.setattr(cell.cc_cell, "run_claude_cell", _fake)
    monkeypatch.setattr(cell.cc_cell, "find_transcript", lambda *a, **k: None)
    out = cell.run_agent_session(
        prompt="p", workspace=tmp_path, env={}, run_dir=tmp_path,
        model="m", timeout_s=1.0, rate_limit_backoff_s=0.0,
        max_rate_limit_retries=3, version="v", sleep=lambda s: None)
    assert out.status == "ok"
    assert len(out.deferrals) == 1
    assert calls["n"] == 2


def test_an_exhausted_quota_raises_and_writes_no_row(monkeypatch, tmp_path):
    monkeypatch.setattr(
        cell.cc_cell, "run_claude_cell",
        lambda *a, **k: ({"is_error": True, "result": "usage limit reached"},
                         {"permission_mode": "x"}))
    with pytest.raises(cell.RateLimited):
        cell.run_agent_session(
            prompt="p", workspace=tmp_path, env={}, run_dir=tmp_path,
            model="m", timeout_s=1.0, rate_limit_backoff_s=0.0,
            max_rate_limit_retries=1, version="v", sleep=lambda s: None)
    assert not (tmp_path / "rows.jsonl").exists()


# --- whole-cell tests with an injected session and grader --------------------------------


@pytest.fixture
def fake_env(tmp_path, monkeypatch):
    """A tiny template + every slow/global side effect stubbed out."""
    root = tmp_path / "staging_root"
    tpl = root / "templates" / "fake"
    tpl.mkdir(parents=True)
    (tpl / "AGENTS.md").write_text("the product\n", encoding="utf-8")
    (root / "templates" / "fake.manifest.json").write_text(
        json.dumps({"sim": "fake", "included_file_count": 1,
                    "filelist_sha256": "abc", "junctions": [], "links": [],
                    "excluded": [], "redactions": [],
                    "known_disclosures": []}), encoding="utf-8")

    monkeypatch.setattr(staging, "build_template",
                        lambda column, r, **kw: tpl)
    monkeypatch.setattr(staging, "reap_port_listeners", lambda *a, **k: [])
    monkeypatch.setattr(staging, "sweep_workspace_processes",
                        lambda *a, **k: [])
    monkeypatch.setattr(staging, "sweep_repo_junction_artifacts",
                        lambda *a, **k: [])
    monkeypatch.setattr(cell.ob_results, "machine_fingerprint",
                        lambda **k: {"machine_id": "test"})
    monkeypatch.setattr(cell.concurrency, "resource_guard",
                        lambda **k: (True, {"source": "stub"}))
    # the ladder ships no starting scene, so the fake task is a real one
    return root


def _ok_session(**kw):
    ws = Path(kw["workspace"])
    (ws / "mine.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
    return cell.SessionOutcome("ok", cc={}, start_ts=0.0,
                               end_ts=1.0)


class _StubVerdict:
    outcome = "PASS"
    progress = 4
    failed = []
    measurements = {"method": "scripted", "reuse_class": None}

    def as_dict(self):
        return {"outcome": "PASS", "progress": 4,
                "progress_name": "graded_pass",
                "assertions": {"T1.1": {"ok": True}},
                "failed_assertion": None, "failed_assertions": [],
                "notes": [], "measurements": dict(self.measurements)}


def test_the_row_is_on_disk_before_teardown_runs(fake_env, monkeypatch,
                                                 tmp_path):
    """The ordering somebody already paid for: a teardown that raises on a
    locked file must never be able to lose a row the quota already bought."""
    seen = {}

    real_teardown = staging.teardown_workspace_resilient

    def _teardown(ws, **kw):
        p = Path(out) / "rows.jsonl"
        seen["rows_existed"] = p.is_file()
        seen["rows_bytes"] = p.stat().st_size if p.is_file() else 0
        return real_teardown(ws, **kw)

    monkeypatch.setattr(staging, "teardown_workspace_resilient", _teardown)
    out = tmp_path / "cellout"
    row = cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=out, use_locks=False,
        session_fn=_ok_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    assert seen["rows_existed"] is True and seen["rows_bytes"] > 0
    assert row["cell"]["value"] == "achieved"
    assert (out / "rows.jsonl").is_file()


def test_agent_metrics_reach_the_row_and_the_pin_is_never_inferred(
        fake_env, tmp_path):
    """The path the dry run cannot exercise: a real session's self-reported
    figures land in the row, the two model fields stay separate, and anything
    the instrument did not report stays null."""
    def _session(**kw):
        ws = Path(kw["workspace"])
        (ws / "mine.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
        return cell.SessionOutcome(
            "ok", start_ts=0.0, end_ts=9.0,
            cc={"num_turns": 12, "tool_calls": 37, "total_cost_usd": 1.23,
                "t_agent_s": 812.5, "tokens_in": 40000, "tokens_out": 6000,
                "tokens_cache_read": 900000, "tokens_cache_write": None,
                "model": "claude-opus-5-20260130", "session_id": "abc",
                "permission_mode": "dangerously-skip-permissions",
                "transcript": None, "tool_calls_reason": None})

    row = cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=tmp_path / "cm",
        use_locks=False, model="claude-opus-5", session_fn=_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    m = row["metrics"]
    assert (m["turns"], m["tool_calls"], m["usd"]) == (12, 37, 1.23)
    assert m["tokens_in"] == 40000 and m["tokens_cache_read"] == 900000
    assert m["tokens_cache_write"] is None       # unmeasured stays null
    assert m["gpu_hours"] is None
    assert row["metrics_source"] == cell.METRICS_SOURCE_AGENT
    assert row["agent"]["model_pinned"] == "claude-opus-5"
    assert row["agent"]["model_reported"] == "claude-opus-5-20260130"
    assert row["condition"] == cell.CONDITION and row["dry_run"] is False


def test_answer_key_in_the_workspace_is_invalid_and_spends_no_session(
        fake_env, monkeypatch, tmp_path):
    called = {"session": False}

    def _session(**kw):
        called["session"] = True
        return _ok_session(**kw)

    def _plant(column, r, **kw):
        tpl = Path(r) / "templates" / "planted"
        (tpl / "docs").mkdir(parents=True, exist_ok=True)
        (tpl / "docs" / "leak.md").write_text(
            "the bar is MIN_MEAN_SPEED_MPS 0.15\n", encoding="utf-8")
        (Path(r) / "templates" / "planted.manifest.json").write_text(
            json.dumps({"sim": "planted", "junctions": [], "links": [],
                        "redactions": [], "known_disclosures": []}),
            encoding="utf-8")
        return tpl

    monkeypatch.setattr(staging, "build_template", _plant)
    out = tmp_path / "cellout2"
    row = cell.run_cell("omnisim", "T3_quadruped", root=fake_env,
                        out_dir=out, use_locks=False, session_fn=_session)
    assert called["session"] is False
    assert row["outcome"] == "INVALID"
    assert row["cell"]["value"] is None
    assert row["answer_key_exposure"]["workspace_clean"] is False
    assert row["answer_key_exposure"]["workspace_hits"]
    assert (out / "rows.jsonl").is_file()


def test_a_transcript_that_reached_the_answer_key_is_invalid(
        fake_env, monkeypatch, tmp_path):
    leak = tmp_path / "transcript.jsonl"
    leak.write_text('{"t":"read tests/benchmarks/ladder/graders/t1_core.py"}',
                    encoding="utf-8")

    def _session(**kw):
        ws = Path(kw["workspace"])
        (ws / "mine.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
        return cell.SessionOutcome("ok", cc={"transcript": str(leak)},
                                   start_ts=0.0, end_ts=1.0)

    row = cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=tmp_path / "c3",
        use_locks=False, session_fn=_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    assert row["outcome"] == "INVALID"
    assert row["answer_key_exposure"]["transcript_hits"]


def test_a_column_with_no_deliverable_convention_blocks_as_ours(
        fake_env, tmp_path):
    row = cell.run_cell(
        "gazebo", "T1_arrive", root=fake_env, out_dir=tmp_path / "c4",
        use_locks=False,
        session_fn=lambda **kw: cell.SessionOutcome("ok", cc={}, start_ts=0.0,
                                                    end_ts=1.0))
    assert row["cell"]["blocker"] == "scaffolding_defect_ours"
    assert row["cell"]["blocker_owner"].startswith("US")


def test_a_whole_cell_prefers_the_workspace_over_a_planted_repo_artifact(
        fake_env, monkeypatch, tmp_path):
    """⚠ THE PAID REGRESSION, end to end through ``run_cell``.

    A synthetic workspace holding the agent's authored world, plus a planted
    ``repo_artifacts/`` holding a NEWER swept copy and a newer throwaway
    probe. The cell must grade the workspace file, must never grade the
    swept ones, must keep them on disk as evidence, and must record which
    pass found the deliverable.
    """
    out = tmp_path / "c_prefers_ws"
    swept = out / "repo_artifacts" / "projects" / "worlds"
    _touch(swept / "mine.wbt", 9_000_000_000)
    _touch(swept / ".omnisim_probe_pyramidal_mu2.wbt", 9_000_000_001)

    def _session(**kw):
        ws = Path(kw["workspace"])
        (ws / "projects").mkdir(parents=True, exist_ok=True)
        (ws / "projects" / "mine.wbt").write_text("WorldInfo {}\n",
                                                  encoding="utf-8")
        return cell.SessionOutcome("ok", cc={}, start_ts=0.0, end_ts=1.0)

    graded = {}

    def _grade(task, column, *, deliverable, **kw):
        graded["path"] = str(deliverable)
        return _StubVerdict(), None, None, "stub"

    row = cell.run_cell("omnisim", "T1_arrive", root=fake_env, out_dir=out,
                        use_locks=False, session_fn=_session, grade_fn=_grade)

    assert graded["path"].endswith("mine.wbt")
    assert "repo_artifacts" not in graded["path"], \
        "phase B must never be handed a swept repo artifact"
    assert "probe" not in graded["path"]
    assert row["cell"]["value"] == "achieved"
    disc = row["cell"]["blocker_evidence"]["deliverable_discovery"]
    assert disc["source"] in ("workspace", "workspace-through-junction")
    assert not disc["ambiguous"]
    # the evidence survives, untouched
    assert (swept / "mine.wbt").is_file()
    assert (swept / ".omnisim_probe_pyramidal_mu2.wbt").is_file()


def test_the_repo_sweep_runs_AFTER_grading_not_before(fake_env, monkeypatch,
                                                      tmp_path):
    """⚠ THE ROOT CAUSE, pinned as an ordering.

    The junction sweep PRESERVES-THEN-DELETES: on omnisim it removes exactly
    the files a session wrote through the ``projects/`` junction, which is
    where an agent asked to author a world naturally saves it. Running it
    before discovery therefore deletes the deliverable out from under the
    walk -- which is what happened on 20260802_155111_omnisim_T2 -- and what
    is left is a quarantine copy whose relative asset and controller
    references no longer resolve (phase B logged *"Cannot open URDF file
    .../repo_artifacts/.../../../../robots/omnisim/omniarm6/omniarm6_2f85.urdf"*,
    0 robots, 0 rows).

    This stub sweep behaves like the real one (it deletes what it preserves)
    and records the order, so the test fails if the sweep ever moves back
    ahead of discovery or grading.
    """
    out = tmp_path / "c_order"
    order = []

    def _sweep(dest, window=None, **kw):
        order.append("sweep:%s" % ("post" if window else "pre"))
        if window is None:                       # the pre-session quarantine
            return []
        moved = []
        for p in sorted([*Path(str(ws_holder["ws"])).rglob("*.omniworld"),
                         *Path(str(ws_holder["ws"])).rglob("*.wbt")]):
            d = Path(dest) / p.name
            d.parent.mkdir(parents=True, exist_ok=True)
            d.write_bytes(p.read_bytes())
            p.unlink()                           # preserve-then-DELETE
            moved.append({"rel": p.name, "action": "preserved_and_deleted"})
        return moved

    ws_holder = {}

    def _session(**kw):
        ws_holder["ws"] = kw["workspace"]
        p = Path(kw["workspace"]) / "projects" / "worlds" / "mine.wbt"
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("WorldInfo {}\n", encoding="utf-8")
        return cell.SessionOutcome("ok", cc={}, start_ts=0.0, end_ts=1.0)

    def _grade(task, column, *, deliverable, **kw):
        order.append("grade")
        assert Path(str(deliverable)).is_file(), \
            "the deliverable must still exist when phase B runs"
        return _StubVerdict(), None, None, "stub"

    monkeypatch.setattr(staging, "sweep_repo_junction_artifacts", _sweep)
    row = cell.run_cell("omnisim", "T1_arrive", root=fake_env, out_dir=out,
                        use_locks=False, session_fn=_session, grade_fn=_grade)

    assert order == ["sweep:pre", "grade", "sweep:post"], order
    assert row["cell"]["value"] == "achieved"


def test_a_whole_cell_with_an_unidentifiable_deliverable_is_invalid(
        fake_env, tmp_path):
    """Two candidates a second apart: the cell must be INVALID with the
    instrument blocker named, must NOT be graded at all (an engine run on a
    file we cannot attribute produces a number nobody may use), and the
    grader's own notes must be preceded by the disclaimer."""
    out = tmp_path / "c_ambig"
    called = {"graded": False}

    def _session(**kw):
        ws = Path(kw["workspace"])
        _touch(ws / "a.wbt", 5000.0, "WorldInfo {}\n")
        _touch(ws / "b.wbt", 5000.4, "WorldInfo {}\n")
        return cell.SessionOutcome("ok", cc={}, start_ts=1000.0, end_ts=6000.0)

    def _grade(*a, **k):
        called["graded"] = True
        return _StubVerdict(), None, None, "stub"

    row = cell.run_cell("omnisim", "T1_arrive", root=fake_env, out_dir=out,
                        use_locks=False, session_fn=_session, grade_fn=_grade)

    assert called["graded"] is False, "an ambiguous cell must not be graded"
    assert row["outcome"] == "INVALID"
    assert row["cell"]["value"] is None and row["cell"]["blocker"] is None
    assert row["cell"]["instrument_blocker"] == "deliverable_ambiguous"
    assert row["cell"]["instrument_blocker_owner"].startswith("US")
    assert "INSTRUMENT BLOCKER" in row["notes"][0]
    # never an agent-attributable verdict
    assert row["cell"]["blocker"] not in cell.BLOCKERS


def test_a_cell_records_the_cap_it_ran_under_and_how_the_session_ended(
        fake_env, tmp_path):
    row = cell.run_cell(
        "omnisim", "T2_transfer", root=fake_env, out_dir=tmp_path / "c_cap",
        use_locks=False, session_fn=_ok_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    task = ladder_tasks.get("T2_transfer")
    assert row["session_cap_s"] == float(task.timeout_s)
    assert "meta.json" in row["session_cap_source"]
    assert row["session_end"]["reason"] == "completed"
    assert row["session_end"]["cap_s"] == float(task.timeout_s)


def test_not_expressible_is_recorded_never_inferred(fake_env, tmp_path):
    with pytest.raises(SystemExit):
        cell.run_cell("gazebo", "T2_transfer", root=fake_env,
                      out_dir=tmp_path / "c5", use_locks=False,
                      not_expressible={"justification": "no grasp verb"})
    row = cell.run_cell(
        "gazebo", "T2_transfer", root=fake_env, out_dir=tmp_path / "c6",
        use_locks=False,
        not_expressible={"justification": "the API listing has no verb for X",
                         "sim_doc_citation": "their docs, section 4",
                         "required_evidence_line": "grip"})
    assert row["cell"]["value"] == "NOT_EXPRESSIBLE"
    assert row["cell"]["not_expressible"]["sim_doc_citation"]
    assert row["metrics"]["tool_calls"] is None


# --- the campaign ---------------------------------------------------------------------


class _Args:
    def __init__(self, **kw):
        self.campaign_id = "t"
        self.lane = "A"
        self.groups = "T1_arrive:mujoco"
        self.column = "mujoco"
        self.n = 2
        self.model = cell.DEFAULT_MODEL
        self.dry_run = True
        self.root = None
        self.results_root = None
        self.lock_root = None
        self.engine_slots = 2
        self.timeout_s = None
        self.backend = None
        self.rate_limit_backoff_s = 0.0
        self.max_rate_limit_retries = 1
        self.min_free_ram_gb = 0.0
        self.max_cpu_load_pct = 100.0
        self.no_publish = True
        self.__dict__.update(kw)


def test_campaign_resume_skips_done_cells(tmp_path, monkeypatch):
    monkeypatch.setattr(camp.staging, "sweep_pending_deletes",
                        lambda *a, **k: {"deleted": [], "failed": []})
    monkeypatch.setattr(camp.concurrency, "resource_guard",
                        lambda **k: (True, {}))
    seen = []

    def _fake_cell(column, task_id, **kw):
        seen.append((task_id, column, kw["repeat"]))
        return {"cell": {"value": "achieved"}, "outcome": "PASS",
                "task": task_id, "column": column}

    args = _Args(root=str(tmp_path / "st"),
                 results_root=str(tmp_path / "res"))
    assert camp.Campaign(args, run_cell=_fake_cell).run() == 0
    assert len(seen) == 2
    # a second identical run resumes and re-runs nothing
    seen.clear()
    assert camp.Campaign(args, run_cell=_fake_cell).run() == 0
    assert seen == []
    summary = json.loads(
        (Path(args.results_root) / "t" / "groups" / "T1_arrive_mujoco"
         / "group_summary.json").read_text(encoding="utf-8"))
    assert summary["solved_at_least_once"] is True


def test_campaign_refuses_to_resume_under_a_changed_pin(tmp_path,
                                                        monkeypatch):
    monkeypatch.setattr(camp.staging, "sweep_pending_deletes",
                        lambda *a, **k: {"deleted": [], "failed": []})
    monkeypatch.setattr(camp.concurrency, "resource_guard",
                        lambda **k: (True, {}))
    args = _Args(root=str(tmp_path / "st"),
                 results_root=str(tmp_path / "res"))
    camp.Campaign(args, run_cell=lambda *a, **k: {"cell": {"value": "x"}}
                  ).run()
    with pytest.raises(SystemExit):
        camp.Campaign(_Args(root=args.root, results_root=args.results_root,
                            model="some-other-model"))


def test_campaign_stops_the_lane_on_a_quota_pause(tmp_path, monkeypatch):
    monkeypatch.setattr(camp.staging, "sweep_pending_deletes",
                        lambda *a, **k: {"deleted": [], "failed": []})
    monkeypatch.setattr(camp.concurrency, "resource_guard",
                        lambda **k: (True, {}))

    def _limited(*a, **kw):
        raise cell.RateLimited("usage limit")

    args = _Args(root=str(tmp_path / "st"),
                 results_root=str(tmp_path / "res"))
    assert camp.Campaign(args, run_cell=_limited).run() == 3
    state = json.loads((Path(args.results_root) / "t" / "state.json")
                       .read_text(encoding="utf-8"))
    statuses = [c["status"] for c in state["cells"].values()]
    assert statuses == ["deferred"]          # and the lane stopped


def test_parse_groups_rejects_an_unknown_tier():
    assert camp.parse_groups("T1:omnisim,T4:mujoco") == [
        ("T1_arrive", "omnisim"), ("T4_humanoid", "mujoco")]
    with pytest.raises(KeyError):
        camp.parse_groups("T9_nope:omnisim")


# --- the oracle registry ----------------------------------------------------------------


def test_oracle_registry_covers_only_what_is_committed():
    """An oracle is registered ONLY where a scripted control has actually been
    run and graded, because ``has_oracle`` false is what turns an ungraded
    (tier, column) into ``scaffolding_defect_ours`` instead of a finding about
    the simulator. Registering one we have not demonstrated would convert our
    own hole into the column's failure -- the exact inversion §4's binding rule
    exists to prevent.
    """
    assert oracles.has_oracle("T1_arrive", "mujoco")
    assert oracles.has_oracle("T3_quadruped", "mujoco")
    # omnisim/T1 landed 2026-08-02: PASS 5/5 through the full cell pipeline
    # (ODE), so it is committed. Its three siblings are not.
    assert oracles.has_oracle("T1_arrive", "omnisim")
    assert not oracles.has_oracle("T2_transfer", "omnisim")
    assert not oracles.has_oracle("T3_quadruped", "omnisim")
    assert not oracles.has_oracle("T4_humanoid", "omnisim")


# --- the phase-B routing guard ----------------------------------------------------------


def test_phase_b_routing_refuses_the_wrong_tier_s_runner():
    """The hazard, held down by a test.

    ``ladder.graders.t1`` / ``t4`` call the column's GENERIC
    ``run_standalone``, and on the MuJoCo column that name is T2's runner. A
    T1 deliverable re-run through the manipulation runner would still produce
    a verdict, which is worse than a refusal.
    """
    fn, attr, refusal = cell.phase_b_runner("mujoco", "T2")
    assert fn is not None and attr == "run_standalone" and refusal is None

    fn, attr, refusal = cell.phase_b_runner("mujoco", "T3")
    assert fn is not None and attr == "t3_run_standalone"

    fn, attr, refusal = cell.phase_b_runner("mujoco", "T1")
    assert fn is None and refusal and "wrong tier" in refusal

    fn, attr, refusal = cell.phase_b_runner("gazebo", "T1")
    assert fn is None and "no ladder-side channel module" in refusal


def test_a_refused_route_is_our_blocker_not_the_simulator_s(tmp_path):
    task = ladder_tasks.get("T1_arrive")
    verdict, phase_b, err, path = cell.grade(
        task, "mujoco", deliverable=tmp_path / "d", run_dir=tmp_path)
    assert verdict is None and err and "REFUSED" in err
    value, blocker, _ = cell.classify(
        session_status="ok", deliverable_found=True,
        deliverable_rule_known=True, grade_error=err, verdict=None)
    assert (value, blocker) == ("not_achieved", "scaffolding_defect_ours")


def test_a_missing_oracle_is_our_blocker(tmp_path):
    task = ladder_tasks.get("T4_humanoid")
    res = oracles.run_oracle(task, "omnisim", tmp_path, tmp_path, tmp_path)
    assert res.ok is False and "no committed scripted oracle" in res.error


# --- a CAPPED session is not an empty row (2026-08-02, omnisim/T2 r1) ------
#
# The cap landed 90 s after the agent wrote "task complete". The old runner
# gated deliverable discovery on ``session.status == "ok"``, so the row said
# `deliverable: null`, `outcome: null`, `metrics: all null` -- and the
# workspace holding a finished controller, its kinematics and a proof bundle
# was deleted. Three separate losses from one early return. Each test below
# was confirmed RED against that code before the fix landed.


def _capped_session(**kw):
    """A session killed by the cap, having left a finished deliverable."""
    ws = Path(kw["workspace"])
    (ws / "mine.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
    (ws / "container").mkdir(exist_ok=True)
    (ws / "container" / "worked.py").write_text("# real work\n",
                                                encoding="utf-8")
    return cell.SessionOutcome("timeout", cc={}, start_ts=0.0, end_ts=1.0,
                               meta={"timed_out": True, "wall_s": 5947.3})


def test_a_capped_session_still_gets_its_deliverable_graded(fake_env,
                                                           tmp_path):
    """RED before the fix: discovery was gated on status == 'ok'."""
    row = cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=tmp_path / "o",
        use_locks=False, session_fn=_capped_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    assert row["artifacts"]["deliverable_found"] is not None, (
        "a capped session's work must be discovered, not discarded")
    assert row["post_cap_grading"] is not None
    assert row["post_cap_grading"]["outcome"] == "PASS"


def test_a_capped_session_that_graded_pass_is_still_not_achieved(fake_env,
                                                                tmp_path):
    """The other half: grading a capped cell must not promote it.

    A slow agent does not get the tier. ``outcome`` stays null so nobody
    scanning the grid counts a cut-off session as a win, and the PASS lives
    only in the block that says in its own words that it is not the cell.
    """
    row = cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=tmp_path / "o",
        use_locks=False, session_fn=_capped_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    assert row["cell"]["value"] == "not_achieved"
    assert row["cell"]["blocker"] == "budget_exhausted"
    assert row["outcome"] is None, (
        "a capped cell has no authoritative outcome; a PASS here would be "
        "counted as a tier result by the first person to read the grid")
    assert "NOT the cell's outcome" in row["post_cap_grading"]["authority"]


def test_the_agents_working_area_survives_teardown(fake_env, tmp_path):
    """RED before the fix: only the one discovered world was copied out."""
    out = tmp_path / "o"
    cell.run_cell(
        "omnisim", "T1_arrive", root=fake_env, out_dir=out, use_locks=False,
        session_fn=_capped_session,
        grade_fn=lambda *a, **k: (_StubVerdict(), None, None, "stub"))
    assert (out / "workspace_container" / "worked.py").is_file(), (
        "the controllers and proof bundle beside the deliverable are the "
        "substance and must outlive the workspace")


def test_a_killed_session_recovers_its_transcript_without_a_session_id(
        tmp_path, monkeypatch):
    """The CLI files the transcript under a slug of the session cwd, so a
    killed run that returned no ``session_id`` is still recoverable."""
    home = tmp_path / "home"
    ws = tmp_path / "Temp" / "ladder_cell" / "instances" / "20260802_x"
    ws.mkdir(parents=True)
    slug = "".join(c if c.isalnum() else "-" for c in str(ws.resolve()))
    d = home / ".claude" / "projects" / slug
    d.mkdir(parents=True)
    (d / "ab96da80.jsonl").write_text('{"type":"user"}\n', encoding="utf-8")

    monkeypatch.setattr(cell.Path, "home", staticmethod(lambda: home))
    found = cell.find_transcript_by_workspace(ws)
    assert found is not None and found.name == "ab96da80.jsonl"


def test_transcript_recovery_does_not_match_a_neighbouring_workspace(
        tmp_path, monkeypatch):
    """The slug is derived from the full path, not a prefix: two cells whose
    names differ only by their timestamp must not read each other's record."""
    home = tmp_path / "home"
    base = tmp_path / "instances"
    a, b = base / "20260802_155116_omnisim_T2_r0", base / "20260802_171852_omnisim_T2_r1"
    a.mkdir(parents=True)
    b.mkdir(parents=True)
    slug_a = "".join(c if c.isalnum() else "-" for c in str(a.resolve()))
    d = home / ".claude" / "projects" / slug_a
    d.mkdir(parents=True)
    (d / "aaa.jsonl").write_text("{}\n", encoding="utf-8")

    monkeypatch.setattr(cell.Path, "home", staticmethod(lambda: home))
    assert cell.find_transcript_by_workspace(a) is not None
    assert cell.find_transcript_by_workspace(b) is None


def test_the_transcript_slug_does_not_collapse_runs_of_separators(tmp_path,
                                                                 monkeypatch):
    r"""RED before 2026-08-03: `C:\Users` slugged to `C-Users`, the CLI writes
    `C--Users`, and a finished 64-minute session lost its transcript.

    The CLI maps each non-alphanumeric character to its own dash. Any regex
    with a `+` collapses the colon and the separator into one and matches
    nothing, on every Windows path there is.
    """
    home = tmp_path / "home"
    ws = tmp_path / "Temp" / "inst" / "cell_r1"
    ws.mkdir(parents=True)
    resolved = str(ws.resolve())
    cli_slug = "".join(c if c.isalnum() else "-" for c in resolved)
    d = home / ".claude" / "projects" / cli_slug
    d.mkdir(parents=True)
    (d / "sess.jsonl").write_text("{}\n", encoding="utf-8")

    monkeypatch.setattr(cell.Path, "home", staticmethod(lambda: home))
    found = cell.find_transcript_by_workspace(ws)
    assert found is not None and found.name == "sess.jsonl", (
        "the slug must match the CLI's char-for-char mapping")


# --- a session that RAN is not a launch failure (2026-08-03) ----------------


def _plant(tmp_path, monkeypatch, *, stdout_text, turns, with_transcript=True):
    home = tmp_path / "home"
    ws = tmp_path / "Temp" / "inst" / "cell_r1"
    ws.mkdir(parents=True)
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "cc_stdout.json").write_text(stdout_text, encoding="utf-8")
    if with_transcript:
        slug = "".join(c if c.isalnum() else "-" for c in str(ws.resolve()))
        d = home / ".claude" / "projects" / slug
        d.mkdir(parents=True)
        (d / "s.jsonl").write_text(
            "\n".join('{"message": {"role": "assistant"}}'
                      for _ in range(turns)), encoding="utf-8")
    monkeypatch.setattr(cell.Path, "home", staticmethod(lambda: home))
    return ws, run_dir


def test_a_finished_session_with_no_json_envelope_is_recovered(tmp_path,
                                                               monkeypatch):
    """The measured case: the CLI printed the agent's final message as text
    instead of the envelope, and a 64-minute session that built a working
    deliverable was filed as launch_failed with every metric null."""
    ws, run_dir = _plant(tmp_path, monkeypatch,
                         stdout_text="The task is complete. Held 14.34 s.",
                         turns=40)
    got = cell._recover_completed_session(ws, run_dir)
    assert got is not None
    doc, note = got
    assert doc["is_error"] is False
    assert "14.34" in doc["result"]
    assert "ABSENT rather than estimated" in note


def test_a_real_launch_failure_is_NOT_recovered(tmp_path, monkeypatch):
    """The falsifier. A launch that never started writes no stdout, so the
    honest label stays launch_failed -- recovering it would invent a session."""
    ws, run_dir = _plant(tmp_path, monkeypatch, stdout_text="", turns=40)
    assert cell._recover_completed_session(ws, run_dir) is None


def test_stdout_without_a_transcript_is_NOT_recovered(tmp_path, monkeypatch):
    """Text alone is not evidence an agent ran: both pieces are required."""
    ws, run_dir = _plant(tmp_path, monkeypatch, stdout_text="something",
                         turns=0, with_transcript=False)
    assert cell._recover_completed_session(ws, run_dir) is None


def test_a_transcript_with_almost_no_turns_is_NOT_recovered(tmp_path,
                                                            monkeypatch):
    ws, run_dir = _plant(tmp_path, monkeypatch, stdout_text="hi", turns=1)
    assert cell._recover_completed_session(ws, run_dir) is None


def test_the_recovered_document_invents_no_cost_numbers(tmp_path, monkeypatch):
    """A synthesised envelope must not carry token or dollar figures: they are
    not on disk, and a made-up number in a row is worse than a null one."""
    ws, run_dir = _plant(tmp_path, monkeypatch, stdout_text="done", turns=9)
    doc, _note = cell._recover_completed_session(ws, run_dir)
    for forbidden in ("usage", "total_cost_usd", "num_turns", "modelUsage"):
        assert forbidden not in doc


# --- every column is told its own deliverable contract (2026-08-03) ---------


def test_each_column_gets_a_deliverable_contract(tmp_path):
    """MEASURED: a MuJoCo cell wrote its controller as a self-contained script
    -- the obvious shape -- and phase B, which needs a control callback it can
    drive, stepped the scene with ctrl_writes=0. Three assertions went red for
    a reason that was neither MuJoCo's nor the agent's."""
    for column in ("omnisim", "mujoco", "webots"):
        ws = tmp_path / column
        ws.mkdir()
        text = staging.write_deliverable_contract(ws, column)
        assert text, column
        assert (ws / "DELIVERABLE.md").is_file(), column


def test_the_mujoco_contract_names_the_callback_and_says_why():
    t = staging.DELIVERABLE_CONTRACT["mujoco"]
    assert "def control(model, data, t)" in t
    assert "scene.xml" in t
    # the reason matters as much as the rule: an agent told only "name it X"
    # learns a magic word, one told WHY can adapt when the shape changes
    assert "library" in t and "no agent present" in t


def test_the_scene_columns_are_not_asked_for_a_callback():
    """The asymmetry is real and must not be flattened: a .wbt IS the scene, so
    those columns are told to author one and nothing more."""
    for column in ("omnisim", "webots"):
        t = staging.DELIVERABLE_CONTRACT[column]
        assert ".wbt" in t and "def control" not in t


def test_no_contract_mentions_a_benchmark_task():
    """It is instruction about the INSTRUMENT, not about the task."""
    for column, t in staging.DELIVERABLE_CONTRACT.items():
        low = t.lower()
        for leak in ("t2_transfer", "bench_arm", "block.urdf", "bin.urdf",
                     "ladder", "loopbench", "grasp"):
            assert leak not in low, (column, leak)


# --- a missing attribute is not evidence of emptiness (2026-08-04) ----------


class _PhaseBStub:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _VerdictStub:
    def __init__(self, vacuous):
        self._v = vacuous

    def as_dict(self):
        return {"assertions": {"T2.1": {"ok": True, "vacuous": self._v}}}


def test_a_runner_that_reports_no_series_is_NOT_called_broken():
    """RED before the fix, and it ran in OUR favour. Only one column's runner
    returns an object with a `t`; MuJoCo's returns an exit code, so every
    MuJoCo cell was filed install_failed -- "our scaffolding" -- and a real
    finding about MuJoCo could never be published as one."""
    assert cell.phase_b_broke(_PhaseBStub(rc=0)) is False


def test_a_verdict_with_real_measurements_means_phase_b_ran():
    assert cell.phase_b_broke(_PhaseBStub(rc=0), _VerdictStub(vacuous=False)) is False


def test_an_explicit_error_is_still_broken():
    assert cell.phase_b_broke(_PhaseBStub(error="scene did not compile")) is True


def test_an_empty_series_is_still_broken():
    assert cell.phase_b_broke(_PhaseBStub(t=[])) is True
    assert cell.phase_b_broke(_PhaseBStub(t=None)) is True


def test_all_vacuous_assertions_do_not_rescue_a_broken_run():
    """The falsifier: a verdict whose every clause is vacuous measured
    nothing, so it must not be taken as proof the re-run produced evidence."""
    assert cell.phase_b_broke(_PhaseBStub(t=[]), _VerdictStub(vacuous=True)) is True
