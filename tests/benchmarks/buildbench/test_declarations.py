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

"""The honesty invariants of BuildBench, enforced mechanically.

A capability matrix is easy to bias and the bias lives in the declarations, so
the rules in SPEC 0.2 are tested rather than trusted. These tests are cheap,
need no simulator, no GPU and no network, and they are the gate on adding a
task.

What they cannot check is the judgement half of rule 1 -- whether a task is
*genuinely* worth doing rather than reverse-engineered from a competitor's
gaps. The mechanical half (the justification may not name a simulator) is here;
the rest is on the author and the reviewer, and saying so is part of the
contract.
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from buildbench import tasks as T                                  # noqa: E402


HERE = os.path.dirname(os.path.abspath(__file__))

#: Names that must not appear in a `why_this_is_real_work` justification.
#: If the reason a task matters mentions a simulator, the task was chosen for
#: who can run it rather than for the work it represents (SPEC 0.2 rule 1).
SIM_NAMES = [
    "omnisim", "webots", "mujoco", "gazebo", "isaac", "coppelia", "genesis",
    "pybullet", "sapien", "newton", "bullet", "ode", "physx", "dart",
    "cyberbotics", "nvidia", "unity", "unreal",
]


def _ids():
    return [t.id for t in T.TASKS]


# --- structural ------------------------------------------------------------

def test_five_tasks_registered_with_unique_ids():
    assert len(T.TASKS) == 5
    assert len(set(_ids())) == 5
    assert set(T.BY_ID) == set(_ids())


@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_every_task_declares_every_sim(task):
    assert {c.sim for c in task.claims} == set(T.SIMS), (
        "%s must declare a claim for every simulator in the matrix; a missing "
        "row is an invisible gap, not a neutral one" % task.id)


@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_vocabularies_are_closed(task):
    assert task.build_status in T.BUILD_STATUSES
    for c in task.claims:
        assert c.verdict in T.VERDICTS, "%s/%s" % (task.id, c.sim)
        assert c.verification_status in T.STATUSES, "%s/%s" % (task.id, c.sim)


@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_prompt_and_capability_are_stated(task):
    assert task.prompt.strip(), task.id
    assert task.demonstrates.strip(), task.id
    assert task.grading_sketch.strip(), (
        "%s must say what would be measured, in physical units -- a task "
        "with no measurable deliverable is not gradeable" % task.id)


# --- SPEC 0.2 rule 1: genuine robotics work ---------------------------------

@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_justification_stands_on_its_own(task):
    why = task.why_this_is_real_work
    assert len(why.strip()) > 80, (
        "%s: 'why this is real work' must actually argue the case" % task.id)
    low = why.lower()
    for name in SIM_NAMES:
        assert not re.search(r"\b%s\b" % re.escape(name), low), (
            "%s: the justification names %r. A task's worth must be arguable "
            "WITHOUT reference to who can run it (SPEC 0.2 rule 1) -- if it "
            "only makes sense once you know who cannot do it, the task is "
            "reverse-engineered from a competitor's gaps."
            % (task.id, name))


# --- SPEC 0.2 rule 2: NOT_EXPRESSIBLE must carry evidence -------------------

@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_not_expressible_names_the_missing_capability(task):
    for c in task.claims:
        if c.verdict == T.NOT_EXPRESSIBLE:
            assert c.missing_capability.strip(), (
                "%s/%s: NOT_EXPRESSIBLE must name the SPECIFIC absent thing -- "
                "a verb, a device, a node type, a service. 'It cannot really "
                "do this' is an assertion, not a finding (SPEC 0.2 rule 2)."
                % (task.id, c.sim))


@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_publishable_claims_carry_citations(task):
    for c in task.claims:
        if c.verification_status in (T.CITED, T.MEASURED):
            assert c.citations, (
                "%s/%s is %s but cites nothing. Only a resolvable citation or "
                "an evidence record lifts a claim above UNVERIFIED (SPEC 2)."
                % (task.id, c.sim, c.verification_status))
            for cit in c.citations:
                assert cit.where.strip() and cit.says.strip()


@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_measured_claims_point_at_an_evidence_file_that_exists(task):
    """MEASURED is the only status licensing a statement about behaviour."""
    for c in task.claims:
        if c.verification_status != T.MEASURED:
            continue
        ev = [cit for cit in c.citations
              if "buildbench/evidence/" in cit.where.replace("\\", "/")]
        assert ev, (
            "%s/%s is MEASURED but cites no evidence record under "
            "buildbench/evidence/. A measurement with no record is a memory."
            % (task.id, c.sim))
        for cit in ev:
            rel = cit.where.replace("\\", "/").split("buildbench/", 1)[1]
            path = os.path.join(HERE, *rel.split("/"))
            assert os.path.isfile(path), (
                "%s/%s cites a missing evidence file: %s"
                % (task.id, c.sim, cit.where))


def test_unverified_claims_are_not_publishable():
    """The derived flag must actually gate; a matrix reads it (SPEC 2 rule 1)."""
    for t in T.TASKS:
        for c in t.claims:
            if c.verification_status == T.UNVERIFIED:
                assert not c.publishable, "%s/%s" % (t.id, c.sim)


def test_the_matrix_marks_every_unverified_cell():
    """A rendering may never show an unverified verdict unmarked."""
    text = T.matrix()
    n_unverified = sum(1 for t in T.TASKS for c in t.claims
                       if not c.publishable)
    assert text.count("!UNVERIFIED") == n_unverified
    assert n_unverified > 0, (
        "if this ever reaches zero, check it is because claims were VERIFIED "
        "and not because the marker was removed")


# --- SPEC 0.2 rule 3: the matrix may not be one-sided ------------------------

def test_at_least_one_task_is_not_expressible_on_omnisim():
    """A suite in which we express everything is not credible (SPEC 0.2 rule 3).

    This is the cheapest possible guard against a brochure: it does not prove
    the task set is fair, it only makes an entirely self-serving one fail.
    """
    ours = [t.claim("omnisim") for t in T.TASKS]
    assert any(c.verdict != T.EXPRESSIBLE for c in ours), (
        "every registered task is EXPRESSIBLE on omnisim. Either the task set "
        "is biased or it is incomplete -- SPEC 5 names the tasks the suite "
        "already owes (a ros2_control/Nav2/MoveIt task, which OmniSim's ROS 2 "
        "sidecar does NOT cover; per-entity spawn/delete services). "
        "Register one rather than deleting this test.")


def test_competitors_are_not_uniformly_negative():
    """Rule 3, the other direction: they must express things too."""
    for sim in ("webots", "mujoco"):
        verdicts = {t.claim(sim).verdict for t in T.TASKS}
        assert verdicts != {T.NOT_EXPRESSIBLE}, (
            "%s is NOT_EXPRESSIBLE on every single task. That is far more "
            "likely to be a bad task set than a true fact about a mature "
            "simulator (SPEC 0.2 rule 3)." % sim)


def test_every_competitor_negative_verdict_records_a_challenge():
    """Write down why we might be wrong, before anyone runs anything."""
    for t in T.TASKS:
        for c in t.claims:
            if c.sim == "omnisim":
                continue
            if c.verdict in (T.NOT_EXPRESSIBLE, T.PARTIAL):
                assert c.challenges, (
                    "%s/%s declares a gap in someone else's product with no "
                    "recorded reason to doubt it. SPEC 4.1 requires the "
                    "challenge to be written down before anything runs."
                    % (t.id, c.sim))


# --- SPEC 0.2 rule 4: the oracle/null gate ----------------------------------

@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_nothing_claims_to_be_gated_without_a_gate_record(task):
    """A task nobody can demonstrably complete is not a capability claim."""
    gate = task.as_dict()["oracle_null_gate"]
    if task.build_status == T.GATED:
        assert all(v != "NOT RUN" for v in gate.values()), (
            "%s is marked GATED but no (task, simulator) cell has an oracle "
            "and a null. The gate is per-arm and our own arm is not exempt "
            "(SPEC 0.2 rule 4)." % task.id)
    else:
        assert all(v == "NOT RUN" for v in gate.values())


def test_no_task_is_gated_yet_and_the_spec_says_so():
    """Keeps SPEC's status line honest as the tree changes."""
    gated = [t.id for t in T.TASKS if t.build_status == T.GATED]
    spec = open(os.path.join(HERE, "SPEC.md"), encoding="utf-8").read()
    if not gated:
        assert "DECLARATION ONLY" in spec, (
            "no task is gated, so the SPEC must still say DECLARATION ONLY")
    else:
        assert "DECLARATION ONLY" not in spec, (
            "%s is gated; the SPEC status line is stale" % gated)


# --- blocked tasks -----------------------------------------------------------

@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_blocked_tasks_keep_their_reason(task):
    if task.build_status == T.BLOCKED:
        assert len(task.blocked_reason.strip()) > 120, (
            "%s is BLOCKED and must record WHY, in enough detail to be "
            "checkable. A blocked task is never silently deleted (SPEC 7.5) "
            "-- it is the most useful row in a capability matrix." % task.id)


def test_b2_is_blocked_with_its_measurement_attached():
    """Pins the one thing this suite has actually measured."""
    b2 = T.BY_ID["B2_granular_traversal"]
    assert b2.build_status == T.BLOCKED
    ours = b2.claim("omnisim")
    assert ours.verdict == T.NOT_EXPRESSIBLE
    assert ours.verification_status == T.MEASURED
    assert os.path.isfile(os.path.join(HERE, "evidence",
                                       "2026-08-11-granular.md"))


# --- risks -------------------------------------------------------------------

def test_task_risk_references_resolve():
    for t in T.TASKS:
        for r in t.depends_on_risks:
            assert r in T.RISKS, "%s references unknown risk %r" % (t.id, r)


def test_every_risk_names_the_tasks_it_threatens():
    for key, r in T.RISKS.items():
        assert r["threatens"], key
        for tid in r["threatens"]:
            assert tid in T.BY_ID, "%s threatens unknown task %r" % (key, tid)
        assert r["status"] in T.STATUSES, key


def test_the_two_open_risks_are_still_open():
    """If one of these is closed, close it with evidence and update the SPEC."""
    assert T.RISKS["risk_1_sensors_under_batching"]["status"] == T.UNVERIFIED
    assert T.RISKS["risk_2_njmax_silent_overflow"]["status"] == T.UNVERIFIED
    assert T.RISKS["risk_3_granular_graded_task"]["status"] == T.MEASURED


# --- generated files stay in sync -------------------------------------------

@pytest.mark.parametrize("task", T.TASKS, ids=_ids())
def test_meta_and_prompt_files_match_the_registry(task):
    d = os.path.join(HERE, "tasks", task.id)
    meta = os.path.join(d, "meta.json")
    prompt = os.path.join(d, "prompt.txt")
    assert os.path.isfile(meta), "run: python -m ...buildbench.tasks --write-meta"
    assert os.path.isfile(prompt)
    with open(meta, encoding="utf-8") as fh:
        on_disk = json.load(fh)
    assert on_disk == task.as_dict(), (
        "%s/meta.json has drifted from tasks.py, which is the single source "
        "of truth. Regenerate with --write-meta." % task.id)
    with open(prompt, encoding="utf-8") as fh:
        assert fh.read().strip() == task.prompt.strip()


# --- the suite does not disturb AgentBench ----------------------------------

def test_agentbench_is_still_there():
    """BuildBench is a SIBLING. AgentBench holds the only credible results in
    this tree and the freeze discipline that makes them credible; it must not
    be renamed, moved, converted or deleted by anything done here."""
    ab = os.path.normpath(os.path.join(HERE, "..", "agentbench"))
    for required in ("SPEC.md", "sims.py", "tasks", "graders", "preregister"):
        assert os.path.exists(os.path.join(ab, required)), (
            "agentbench/%s is missing" % required)
