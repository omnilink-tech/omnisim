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

"""Unit tests for the B1 ``overlap_audit`` neutral core. **No simulator.**

Same discipline as ``test_neutral_core.py``: every fixture is a synthetic
evidence bundle, so the grader can be tested, argued with, and re-run by a
third party without our stack.

    pytest tests/benchmarks/agentbench/graders/test_b1_core.py -q

The load-bearing part is ``B1_RED_MAP`` + the parametrized red-evidence test:
per plan 5.5, every B1 assertion enters the suite already OBSERVED FAILING on
a deliberately wrong answer/world, and the fixture->assertion table is a
structured constant the coverage table can read. The answers graded here are
built by the same functions the scripted fixture agents in
``agents/b1_fixtures.py`` emit, so the string tested is the string a Phase-0
run produces.

NOTE for the orchestrator: ``test_neutral_core.py``'s AST guard
(``test_cores_contain_no_simulator_specific_vocabulary``) should have
``b1_core`` added to its parametrize list. This file cannot edit it (shared
file, another lane), so it applies the SAME guard to ``b1_core`` locally by
importing the guard's own helpers.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.agents import b1_fixtures                         # noqa: E402
from agentbench.graders import b1_core                            # noqa: E402
from agentbench.graders.evidence import (                         # noqa: E402
    Body, BodyInventory, EvidenceBundle)
from agentbench.graders.verdict import (                          # noqa: E402
    CORE_PHYSICAL, CORE_STRUCTURAL, FAIL, INVALID, PASS)
# The shared AST guard, applied locally to b1_core (see the module docstring).
from agentbench.graders.test_neutral_core import (                # noqa: E402
    _SIM_TOKENS, _code_strings_and_names)

# The six-robot layout of the shipped world, in metres. husky_one/husky_two
# interpenetrate deeply; every other pair is metres clear. The grader must
# DERIVE this from the bundle -- nothing below is handed to it.
LAYOUT = {
    "husky_one": (0.0, 0.0),
    "husky_two": (0.3, 0.1),
    "husky_three": (4.0, 0.0),
    "husky_four": (8.0, 0.0),
    "husky_five": (4.0, 4.0),
    "husky_six": (8.0, 4.0),
}
# The same layout with the overlap removed (the phantom fixture's world).
LAYOUT_CLEAR = dict(LAYOUT, husky_two=(0.3, -4.0))
HALF_X, HALF_Y = 0.5, 0.35
OVERLAP_PAIR = "husky_one+husky_two"


def b1_bundle(*, layout=None, frozen=True, drop_aabb_for=(), t0_error=None,
              names=None):
    layout = dict(LAYOUT if layout is None else layout)
    bodies = []
    for i, (name, (x, y)) in enumerate(sorted(layout.items())):
        has_aabb = name not in drop_aabb_for
        bodies.append(Body(
            body_id="#%d" % (100 + i),
            name=(names[i] if names is not None else name), kind="Robot",
            position=(x, y, 0.2),
            aabb_min=(x - HALF_X, y - HALF_Y, 0.0) if has_aabb else None,
            aabb_max=(x + HALF_X, y + HALF_Y, 0.45) if has_aabb else None,
            n_joints=4, dynamic=True, robot_class=True,
            identity_evidence="synthetic fixture"))
    return EvidenceBundle(
        task=b1_core.TASK, sim="synthetic", adapter="tests",
        t0=BodyInventory(bodies=bodies, frozen=frozen,
                         t_s=0.0 if frozen else None,
                         source="synthetic frozen scan", error=t0_error))


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


NAMES = sorted(LAYOUT)


# --- ground truth is measured, never hardcoded -------------------------------


def test_b1_ground_truth_is_derived_from_the_bundle_geometry():
    truth = b1_core.measure_ground_truth(b1_bundle())
    assert truth["overlapping"] == [OVERLAP_PAIR]
    assert truth["ambiguous"] == []
    assert truth["measured_all"] and truth["frozen"]
    assert truth["pair_clearance_m"][OVERLAP_PAIR] <= -b1_core.OVERLAP_MIN_M
    clear = {k: c for k, c in truth["pair_clearance_m"].items()
             if k != OVERLAP_PAIR}
    assert len(clear) == 14
    assert min(clear.values()) >= b1_core.CLEAR_MIN_M


def test_b1_ground_truth_moves_when_the_world_moves():
    """Relocate the overlap onto a different pair: the grader must follow.

    This is design requirement 1: no hardcoded pair name anywhere.
    """
    layout = dict(LAYOUT, husky_two=(0.3, -4.0),           # break one+two
                  husky_five=(7.8, 3.9))                   # overlap five+six
    truth = b1_core.measure_ground_truth(b1_bundle(layout=layout))
    assert truth["overlapping"] == ["husky_five+husky_six"]
    r = b1_core.grade(b1_bundle(layout=layout),
                      answer="Yes. husky_five and husky_six are overlapping.")
    assert r.outcome == PASS, r.summary()


def test_b1_thresholds_are_the_registered_constants():
    assert (b1_core.OVERLAP_MIN_M, b1_core.CLEAR_MIN_M) == (0.05, 0.05)


# --- the honest answers pass -------------------------------------------------


def test_b1_passes_an_honest_yes_answer():
    r = b1_core.grade(b1_bundle(), answer=(
        "Yes. husky_one and husky_two are overlapping -- their bounding "
        "boxes interpenetrate by about 0.45 m."))
    assert r.outcome == PASS, r.summary()
    assert len(r.assertions) == 4
    assert r.progress == 4
    assert not r.vacuous


def test_b1_passes_an_honest_no_answer_on_a_clear_scene():
    """Incidental names in a "no" answer are not overlap claims."""
    r = b1_core.grade(b1_bundle(layout=LAYOUT_CLEAR), answer=(
        "No -- no two robots overlap. The closest pair is husky_one and "
        "husky_three, about 2.7 m apart."))
    assert r.outcome == PASS, r.summary()
    assert _ids(r)["B1.4"].measured["claimed pair"] is None


def test_b1_passes_an_honest_verbose_answer():
    """Naming the clear robots too must not read as a shotgun, provided the
    committed pair is the one adjacent to the overlap cue."""
    r = b1_core.grade(b1_bundle(), answer=(
        "Yes: husky_three, husky_four, husky_five and husky_six are all "
        "parked far apart from one another with clear space around them. "
        "husky_one and husky_two are overlapping."))
    assert r.outcome == PASS, r.summary()
    assert _ids(r)["B1.3"].measured["committed pair"] == OVERLAP_PAIR


def test_b1_first_cue_wins_so_a_yes_may_deny_other_pairs():
    r = b1_core.grade(b1_bundle(), answer=(
        "Yes -- husky_one and husky_two overlap; no other pair does."))
    assert r.outcome == PASS, r.summary()
    assert _ids(r)["B1.1"].measured["verdict"] == "yes"


# --- 🔴 red evidence: every fixture observed failing (plan 5.5) --------------
#
# fixture -> (bundle layout, the EXACT answer the scripted agent emits,
#             assertions that must go red). This is the structured constant
#             the plan-5.5 coverage table reads; the parametrized test below
#             is what makes it MEASURED rather than believed, and a further
#             test pins it equal to agents/b1_fixtures.EXPECTED_FAILURES.

B1_RED_MAP = {
    # (a) wrong pair: confident, well-formed, names a pair that is metres
    #     clear. Only the physical pair check goes red.
    "wrong_pair": (LAYOUT,
                   b1_fixtures.wrong_pair_answer("husky_three", "husky_six"),
                   {"B1.4"}),
    # (b) shotgun: lists every pair; commits to none (the B3 anti-shotgun
    #     lesson replayed against pair naming).
    "shotgun": (LAYOUT, b1_fixtures.shotgun_answer(NAMES),
                {"B1.3", "B1.4"}),
    # (c) yes with no pair named: half the deliverable withheld.
    "yes_only": (LAYOUT, b1_fixtures.yes_only_answer(), {"B1.3", "B1.4"}),
    # (d) the artifact world has NO overlap but the answer claims one.
    "phantom": (LAYOUT_CLEAR,
                b1_fixtures.phantom_answer("husky_one", "husky_two"),
                {"B1.2", "B1.4"}),
    # (e) "no" on the shipped world, where one pair measurably overlaps.
    "missed": (LAYOUT, b1_fixtures.missed_answer(), {"B1.2", "B1.4"}),
    # (f) no verdict at all: the targeted red for B1.1 (the null agent also
    #     reds B1.1, but a null red validates nothing -- plan 5.5).
    "waffle": (LAYOUT, b1_fixtures.waffle_answer(),
               {"B1.1", "B1.2", "B1.4"}),
}


@pytest.mark.parametrize("fixture", sorted(B1_RED_MAP))
def test_b1_negative_fixture_goes_red_exactly_where_declared(fixture):
    layout, answer, expected = B1_RED_MAP[fixture]
    r = b1_core.grade(b1_bundle(layout=layout), answer=answer)
    assert r.outcome == FAIL, r.summary()
    assert set(r.failed) == expected, (
        "%s: expected red %s, measured red %s\n%s"
        % (fixture, sorted(expected), sorted(r.failed), r.summary()))


def test_b1_red_map_matches_the_fixture_agents_declaration():
    """One table, two homes: the scripted agents' expect_failures sets must be
    the measured ones, or a Phase-0 run would report MISMATCH."""
    assert {k: v[2] for k, v in B1_RED_MAP.items()} \
        == b1_fixtures.EXPECTED_FAILURES


def test_b1_every_assertion_has_a_targeted_red_fixture():
    """The plan-5.5 coverage requirement, structural: no B1 assertion may rely
    on the null agent for its only red."""
    covered = set()
    for _layout, _answer, expected in B1_RED_MAP.values():
        covered |= expected
    assert covered == {"B1.1", "B1.2", "B1.3", "B1.4"}


def test_b1_wrong_pair_red_is_measured_in_metres():
    layout, answer, _exp = B1_RED_MAP["wrong_pair"]
    r = b1_core.grade(b1_bundle(layout=layout), answer=answer)
    m = _ids(r)["B1.4"].measured
    assert m["claimed pair"] == "husky_six+husky_three"
    assert m["claimed pair clearance (m)"] >= b1_core.CLEAR_MIN_M
    assert m["measured overlapping pairs"] == [OVERLAP_PAIR]


def test_b1_shotgun_commits_to_no_pair():
    layout, answer, _exp = B1_RED_MAP["shotgun"]
    r = b1_core.grade(b1_bundle(layout=layout), answer=answer)
    m = _ids(r)["B1.3"].measured
    assert m["committed pair"] is None
    assert len(m["robot names mentioned"]) == 6


def test_b1_null_agent_answer_passes_nothing():
    """SPEC 7.1: no task may be passable by doing nothing."""
    r = b1_core.grade(b1_bundle(), answer=(
        "Done -- everything is set up and working as requested."))
    assert r.outcome == FAIL
    assert "B1.1" in r.failed and "B1.2" in r.failed and "B1.4" in r.failed


# --- vacuity and INVALID (design requirement 3) ------------------------------


def test_b1_unfrozen_scan_grades_but_reports_the_geometry_vacuous():
    """A free-running inventory cannot answer an 'at t=0' question. The
    physical assertions still evaluate -- the numbers may be true -- but the
    row must say their witness was absent (the A1.3 mechanic)."""
    r = b1_core.grade(b1_bundle(frozen=False), answer=(
        "Yes. husky_one and husky_two are overlapping."))
    a2, a4 = _ids(r)["B1.2"], _ids(r)["B1.4"]
    assert a2.ok is True and a4.ok is True       # they "passed"...
    assert a2.vacuous and a4.vacuous             # ...and could not have failed
    assert set(r.vacuous) == {"B1.2", "B1.4"}
    assert "~vacuous" in r.summary()
    assert any("not marked frozen" in n for n in r.notes)
    # the structural halves never depended on the geometry witness
    assert not _ids(r)["B1.1"].vacuous and not _ids(r)["B1.3"].vacuous


def test_b1_invalid_without_any_t0_scan():
    b = b1_bundle()
    b.t0 = BodyInventory(bodies=[], error="phase B not run")
    r = b1_core.grade(b, answer="Yes. husky_one and husky_two overlap.")
    assert r.outcome == INVALID


def test_b1_invalid_with_fewer_than_two_robots():
    b = b1_bundle()
    b.t0.bodies = b.t0.bodies[:1]
    r = b1_core.grade(b, answer="Yes. husky_one and husky_two overlap.")
    assert r.outcome == INVALID


def test_b1_invalid_when_a_robot_has_no_measured_bounds():
    """'No other pair overlaps' cannot be certified around an unmeasured
    robot -- grading anyway would be the (id,id) mistake again."""
    r = b1_core.grade(b1_bundle(drop_aabb_for={"husky_five"}),
                      answer="Yes. husky_one and husky_two overlap.")
    assert r.outcome == INVALID
    assert any("husky_five" in n for n in r.notes)


def test_b1_invalid_when_a_pair_sits_in_the_ambiguous_band():
    """0.02 m of AABB interpenetration is inside (-0.05, +0.05): the proxy
    cannot honestly call it either way, so the scene is not gradable."""
    r = b1_core.grade(b1_bundle(layout=dict(LAYOUT, husky_five=(7.98, 3.3))),
                      answer="Yes. husky_one and husky_two overlap.")
    assert r.outcome == INVALID
    assert any("ambiguous" in n for n in r.notes)


def test_b1_invalid_when_robot_names_are_not_distinct():
    r = b1_core.grade(b1_bundle(names=["husky_a"] * 6),
                      answer="Yes. husky_a and husky_a overlap.")
    assert r.outcome == INVALID


# --- structure ---------------------------------------------------------------


def test_b1_every_assertion_declares_how_it_could_fail():
    r = b1_core.grade(b1_bundle(), answer="Yes. husky_one and husky_two "
                                          "are overlapping.")
    for a in r.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_b1_basis_separates_physical_from_structural():
    r = b1_core.grade(b1_bundle(), answer="Yes. husky_one and husky_two "
                                          "are overlapping.")
    basis = r.basis_summary()
    assert set(basis[CORE_PHYSICAL]) == {"B1.2", "B1.4"}
    assert set(basis[CORE_STRUCTURAL]) == {"B1.1", "B1.3"}


def test_b1_fixture_registry_is_registry_shaped():
    """The orchestrator merges agents/b1_fixtures.REGISTRY verbatim; check the
    shape it promises without importing (or editing) agents/__init__.py."""
    reg = b1_fixtures.REGISTRY
    assert (b1_fixtures.TASK_ID, "null") in reg
    for (task_id, name), entry in reg.items():
        assert task_id == "B1_overlap_audit"
        assert callable(entry["fn"])
        assert entry["expect_pass"] is False
        if name == "null":
            assert entry["expect_failures"] is None
        else:
            assert entry["expect_failures"] \
                == b1_fixtures.EXPECTED_FAILURES[name]
    assert set(n for _t, n in reg) \
        == {"null"} | set(b1_fixtures.EXPECTED_FAILURES)


def test_b1_core_contains_no_simulator_specific_vocabulary():
    """The same AST guard test_neutral_core applies to a1/b3/c2 cores, applied
    to b1_core until the shared parametrize list picks it up."""
    for text in _code_strings_and_names(b1_core.__file__):
        low = text.lower()
        for token in _SIM_TOKENS:
            assert token not in low, (
                "b1_core.py mentions %r in code (not a docstring): %r"
                % (token, text))
