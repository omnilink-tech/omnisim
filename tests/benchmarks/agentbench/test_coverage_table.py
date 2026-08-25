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

"""Tests for the red-evidence coverage table (plan 5.5, order-of-work item 2).

Two properties are load-bearing:

  * the generator must flag an assertion whose only red evidence is the
    ``null`` agent as UNVALIDATED -- that is the exact loophole A1.3 hid
    behind for weeks;
  * every committed fixture verdict must match its REGISTRY
    ``expect_failures`` declaration -- the coverage table cites those files,
    so a drifted declaration would launder an unobserved red.

No test here runs the engine; they read the committed evidence and exercise
the pure ``compute_coverage`` function.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))

from agentbench import agents as agent_registry  # noqa: E402
from agentbench import coverage_table  # noqa: E402
from agentbench.agents import a1_fixtures_extra  # noqa: E402

PHASE0 = HERE / "phase0_validation"
A1 = "A1_husky_swarm_10"


def _verdict(agent, task, failed, fname=None, outcome="FAIL"):
    return (agent, fname or ("%s.verdict.json" % agent), {
        "task": task, "outcome": outcome,
        "assertions": {a: {"ok": False, "what": ""} for a in failed},
        "failed_assertions": sorted(failed),
    })


# --- the rule itself, on synthetic inputs -----------------------------------


def test_null_only_red_is_unvalidated():
    """A null-agent red validates nothing (the A1.3 loophole, plan 5.5)."""
    cov = coverage_table.compute_coverage(
        universe={"T": {"T.1": "a thing"}},
        registry={},
        verdicts=[_verdict("null", "T", {"T.1"})])
    (row,) = cov["rows"]
    assert row["validated"] is False
    assert "null" in row["unvalidated_reason"]
    assert row["null_only_red"] == ["null.verdict.json"]
    assert cov["summary"]["validated"] == 0


def test_non_null_fixture_red_validates():
    cov = coverage_table.compute_coverage(
        universe={"T": {"T.1": "a thing"}},
        registry={("T", "wrong"): {"expect_failures": {"T.1"}}},
        verdicts=[_verdict("null", "T", {"T.1"}),
                  _verdict("wrong", "T", {"T.1"})])
    (row,) = cov["rows"]
    assert row["validated"] is True
    assert row["targeted_fixtures"] == ["wrong"]
    assert [o["agent"] for o in row["observed_red"]] == ["wrong"]


def test_no_red_evidence_at_all_is_unvalidated():
    cov = coverage_table.compute_coverage(
        universe={"T": {"T.2": "another thing"}},
        registry={("T", "wrong"): {"expect_failures": {"T.2"}}},
        verdicts=[])
    (row,) = cov["rows"]
    assert row["validated"] is False
    assert "no committed red verdict" in row["unvalidated_reason"]


def test_extra_evidence_is_recorded_but_not_end_to_end_validation():
    cov = coverage_table.compute_coverage(
        universe={"T": {"T.1": "a thing"}},
        registry={},
        verdicts=[],
        extra={("T", "T.1"): {"kind": "core-unit", "end_to_end": False,
                              "fixture": "some test", "reason": "structural"}})
    (row,) = cov["rows"]
    assert row["validated"] is False
    assert row["extra_evidence"]["kind"] == "core-unit"
    assert "structural" in row["unvalidated_reason"]


# --- the committed evidence -------------------------------------------------


@pytest.mark.parametrize("name", sorted(a1_fixtures_extra._FIXTURE_FNS))
def test_new_fixture_expectation_matches_committed_verdict(name):
    """expect_failures must equal the measured red set, file for file."""
    entry = agent_registry.REGISTRY[(A1, name)]
    path = PHASE0 / ("%s.%s.verdict.json" % (A1, name))
    assert path.exists(), (
        "no committed verdict for fixture %r -- run it through "
        "run_agentbench.py and commit the graded verdict" % name)
    v = json.loads(path.read_text(encoding="utf-8"))
    assert v["task"] == A1
    assert v["outcome"] != "PASS", "a negative fixture must not PASS"
    assert set(v["failed_assertions"]) == set(entry["expect_failures"]), (
        "fixture %r: REGISTRY expects %s but the committed verdict measured "
        "%s" % (name, sorted(entry["expect_failures"]),
                sorted(v["failed_assertions"])))


def test_targeted_assertions_now_have_non_null_red_evidence():
    """The six previously-uncovered A1 assertions, minus the structural one.

    A1.10 is asserted separately: its red evidence is core-level and the
    table must say UNVALIDATED end-to-end rather than quietly counting it.
    """
    cov = coverage_table.build(PHASE0)
    rows = {(r["task"], r["assertion"]): r for r in cov["rows"]}
    for aid in ("A1.1", "A1.2", "A1.3", "A1.7", "A1.9"):
        row = rows[(A1, aid)]
        assert row["validated"], (
            "%s should now be validated; reason: %s"
            % (aid, row["unvalidated_reason"]))
        assert row["observed_red"], aid
        assert all(o["agent"] != "null" for o in row["observed_red"])


def test_a1_10_is_flagged_unvalidated_with_the_structural_reason():
    cov = coverage_table.build(PHASE0)
    row = {(r["task"], r["assertion"]): r for r in cov["rows"]}[(A1, "A1.10")]
    assert row["validated"] is False
    assert row["extra_evidence"] is not None
    assert row["extra_evidence"]["kind"] == "core-unit"
    assert "structural" in (row["unvalidated_reason"] or "")


def test_committed_coverage_files_match_regeneration():
    """COVERAGE.md / coverage.json are generated artifacts; drift = stale."""
    cov = coverage_table.build(PHASE0)
    md = coverage_table.render_markdown(cov)
    js = json.dumps(cov, indent=2, default=str) + "\n"
    md_path = PHASE0 / "COVERAGE.md"
    js_path = PHASE0 / "coverage.json"
    assert md_path.exists() and js_path.exists(), (
        "run coverage_table.py to generate the committed table")
    assert md_path.read_text(encoding="utf-8") == md, (
        "COVERAGE.md is stale -- regenerate with coverage_table.py")
    assert js_path.read_text(encoding="utf-8") == js, (
        "coverage.json is stale -- regenerate with coverage_table.py")


def test_core_unattributed_verdict_is_grader_output_and_red_on_a1_10():
    """The A1.10 core-level evidence really is the grader saying INVALID."""
    v = a1_fixtures_extra.core_unattributed_verdict()
    assert "A1.10" in v.failed
    assert v.outcome == "INVALID"
    assert any("synthetic" in n for n in v.notes)


def test_committed_a1_10_core_verdict_matches_regeneration():
    """The committed core-level verdict file is regenerable grader output."""
    path = PHASE0 / ("%s.core_unattributed.verdict.json" % A1)
    assert path.exists()
    committed = json.loads(path.read_text(encoding="utf-8"))
    fresh = a1_fixtures_extra.core_unattributed_verdict().as_dict()
    assert committed["failed_assertions"] == fresh["failed_assertions"]
    assert committed["outcome"] == fresh["outcome"] == "INVALID"
