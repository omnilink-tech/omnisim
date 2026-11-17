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

"""Unit tests for T3's neutral core. **No simulator involved.**

Every fixture is synthetic numpy, so this file runs in a second on a machine
with no build, no GPU and no network -- which is the point of the neutral core:
a third party can re-derive every verdict in the ladder without our stack.

    pytest tests/benchmarks/ladder/graders/test_t3_core.py -q

Five tests carry more weight than the rest:

:func:`test_every_negative_fixture_reddens_exactly_its_own_assertion` is the
executable form of the red-evidence rule (``capability-ladder-plan.md`` §5c.2),
exact in both directions.

:func:`test_the_method_is_recorded_and_never_graded` is the executable form of
§2 T3's ruling that training is permitted and not required. It grades one run
under all four method values and requires the assertions to come out identical
-- because "we promise not to grade it" is a comment, and this is a test.

:func:`test_an_unattestable_support_is_excluded_not_credited` is §2 T3.4's own
consequence: a cell that cannot prove nothing held the robot up is recorded as
``unverified`` and excluded from comparison, never silently passed and never
failed as if our missing channel were somebody else's capability gap.

:func:`test_a_run_the_world_ended_is_recorded_as_such_not_as_a_bad_gait` is the
executable form of the measured build note in
``docs/developer/g1-endurance-2026-08-01.md`` §8, where six clean runs of this
tree's own flagship would have been published as a degrading gait by a grader
that did not know where the walls were.

:func:`test_the_aggregate_contact_signal_of_a_perfect_walk_never_changes` is
why the transition count is per body: on the oracle -- a textbook trot -- the
aggregate "is this robot touching the ground" signal is **constant**, and a
grader that counted it would score a perfect walk at zero footfalls.

The AST guard that keeps the cores simulator-neutral is **replicated at the
bottom of this file**, deliberately, until T3's modules are added to the shared
``test_neutral_core.NEUTRAL_MODULES`` tuple.
"""

from __future__ import annotations

import ast
import importlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders.verdict import (  # noqa: E402
    CORE_PHYSICAL, FAIL, INVALID, MIXED, PASS)
from ladder import adapters, tasks  # noqa: E402
from ladder.graders import fixtures_t3, t3, t3_core  # noqa: E402
from ladder.graders import t3_evidence as t3ev  # noqa: E402


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the positive control ----------------------------------------------------


def test_the_oracle_run_passes_every_assertion():
    v = t3_core.grade(fixtures_t3.oracle())
    assert v.outcome == PASS, v.summary()
    assert len(v.assertions) == 5
    assert v.progress == 4
    assert not v.vacuous
    a = _ids(v)
    assert a["T3.1"].measured["furthest from the start reached (m)"] >= 10.0
    assert a["T3.2"].measured["samples over the bound"] == 0
    assert a["T3.3"].measured[
        "mean speed made good over the scored window (m/s)"] >= 0.15
    assert a["T3.4"].measured["support attestation"] == t3ev.ATTESTED


def test_the_oracle_evidence_satisfies_the_ladders_contract():
    assert t3ev.check_t3_evidence(fixtures_t3.oracle()) == []
    assert t3ev.unanswered_channels(fixtures_t3.oracle()) == {}


# --- the thresholds ----------------------------------------------------------


def test_thresholds_are_the_plans_numbers():
    """A drift in any of these is a tier change, not a refactor.

    capability-ladder-plan.md §5a: moving a threshold voids the pass and
    forces a version bump and a full re-run.
    """
    assert t3_core.NET_DISPLACEMENT_M == 10.0
    assert (t3_core.FALL_Z_FRACTION, t3_core.MAX_TILT_RAD) == (0.60, 0.8)
    assert t3_core.MIN_MEAN_SPEED_MPS == 0.15
    assert t3_core.MIN_BOB_RMS_M == 0.005
    assert t3_core.MIN_CONTACT_TRANSITIONS == 8
    assert (t3_core.MAX_SUPPORT_FORCE_FRACTION,
            t3_core.MAX_SUPPORT_TORQUE_NM) == (0.02, 2.0)


def test_the_task_file_and_the_core_agree_on_every_constant():
    """The task's meta.json is what a reader opens; the core is what runs.

    Two copies of a threshold is one threshold and one lie waiting to happen,
    so they are compared rather than trusted -- in BOTH directions.
    """
    task = tasks.get(t3_core.TASK)
    for name, value in task.constants.items():
        assert hasattr(t3_core, name), "meta.json declares unknown %r" % name
        assert getattr(t3_core, name) == pytest.approx(value), name
    declared = set(task.constants)
    in_core = {n for n in t3_core.__all__
               if n.isupper() and isinstance(getattr(t3_core, n), (int, float))
               and not isinstance(getattr(t3_core, n), bool)}
    assert in_core - declared == set(), in_core - declared


# --- the red-evidence rule ---------------------------------------------------


@pytest.mark.parametrize("name", sorted(fixtures_t3.FIXTURE_ASSERTION_MAP))
def test_every_negative_fixture_reddens_exactly_its_own_assertion(name):
    """THE test the red-evidence rule exists for.

    Exact, in both directions: the declared assertion must be red, and no
    other one may be. §5c.2's own caveat is that "a null agent turning every
    assertion red does not satisfy this rule" -- so a fixture that reddens two
    assertions has stopped being evidence that either check discriminates.
    """
    _verdict, problems = fixtures_t3.check_fixture(name)
    assert not problems, "%s: %s" % (name, "; ".join(problems))


def test_every_assertion_has_a_negative_fixture():
    assert fixtures_t3.uncovered_assertions() == ()
    covered = {r["assertion"] for r in fixtures_t3.coverage_table()}
    assert covered == set(fixtures_t3.ASSERTIONS)


def test_the_two_fixtures_the_plan_names_by_hand_exist():
    """§5c.2: "a crawling robot for T3.3, a robot on a crane for T3.4"."""
    m = fixtures_t3.FIXTURE_ASSERTION_MAP
    assert m["slid_without_gait"]["red"] == ("T3.3",)
    assert m["held_up_beyond_the_threshold"]["red"] == ("T3.4",)


def test_the_repaired_clause_has_a_must_pass_control_in_the_map():
    """A repair that only has reds behind it has not been shown to be safe.

    ``slid_without_gait`` proves the surviving gate still catches a slider;
    ``slow_static_gait`` proves the retired one was catching a walker. Both
    are declared rows in the map, so both are re-checked on every run.
    """
    m = fixtures_t3.FIXTURE_ASSERTION_MAP
    assert m["slow_static_gait"]["red"] == ()
    assert m["slow_static_gait"]["outcome"] == PASS
    assert m["slow_static_gait"]["vacuous"] == {}


def test_the_coverage_table_names_a_fixture_for_each_assertion():
    text = fixtures_t3.render_coverage_table()
    for aid in fixtures_t3.ASSERTIONS:
        assert aid in text
    assert "every assertion has a negative fixture" in text
    # ...and the fixtures that must PASS are visible as such, because a reader
    # scanning only the red rows would conclude the method is graded and that
    # an unattested support is a failure
    assert "scripted_controller_loaded" in text
    assert "support_not_attested" in text
    # ...including the must-PASS control for the 2026-08-02 T3.3 repair: a
    # coverage table that listed only reds would leave the one run the repair
    # exists for invisible
    assert "slow_static_gait" in text
    assert "MUST-PASS CONTROL" in text


def test_every_assertion_declares_how_it_could_fail():
    v = t3_core.grade(fixtures_t3.oracle())
    for a in v.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_basis_map_separates_measured_physics_from_adapter_claims():
    v = t3_core.grade(fixtures_t3.oracle())
    basis = v.basis_summary()
    assert set(basis[CORE_PHYSICAL]) == {"T3.1", "T3.2", "T3.3"}
    assert set(basis[MIXED]) == {"T3.4", "T3.5"}
    assert v.as_dict()["basis"] == basis


# --- the method: recorded, never graded --------------------------------------


@pytest.mark.parametrize("method", t3ev.METHODS)
def test_the_method_is_recorded_and_never_graded(method):
    """§2 T3's ruling, made executable rather than promised in a comment.

    "A cell that reaches the outcome by training a policy and a cell that
    reaches it by a model-based gait are both achieved, with method recorded."
    So the same run must grade identically under every method -- and the value
    must still reach the row.
    """
    ev = fixtures_t3.oracle()
    ev.controller.declared_method = method
    v = t3_core.grade(ev)
    baseline = t3_core.grade(fixtures_t3.oracle())
    assert [(a.id, a.ok) for a in v.assertions] == [
        (a.id, a.ok) for a in baseline.assertions]
    assert v.measurements["method"] == method


def test_an_unrecognised_method_is_recorded_as_unknown_not_dropped():
    ev = fixtures_t3.oracle()
    ev.controller.declared_method = "evolutionary, obviously"
    v = t3_core.grade(ev)
    assert v.measurements["method"] == "unknown"
    assert v.measurements["controller"]["declared_as"] == (
        "evolutionary, obviously")


def test_a_hand_written_gait_passes_with_its_method_printed():
    v = t3_core.grade(fixtures_t3.scripted_controller_loaded())
    assert v.outcome == PASS
    assert v.measurements["method"] == "scripted"


def test_the_labels_reach_the_row_even_when_nothing_else_does():
    """A cell that failed before any physics was read is still a T3 cell.

    §8 forbids quoting a T3 cell without its reuse_class and (for the support
    claim) its measured support, so no verdict may omit them.
    """
    ev = fixtures_t3.oracle()
    ev.bundle.artifact = None
    v = t3_core.grade(ev)
    assert v.outcome == FAIL and len(v.failed) == 5
    for key in ("method", "reuse_class", "support_attestation",
                "external_support", "excluded_from_comparison"):
        assert key in v.measurements


# --- reuse_class: the grader refuses to decide it ----------------------------


def test_reuse_class_is_carried_null_and_loudly():
    """§9 Q3: the boundary is fuzzy and the reviewer arbitrates.

    A grader that guessed would be inventing the one label the plan says a
    human owns, so the field is present, null, and accompanied by a note that
    a cell published with it still null is not publishable.
    """
    v = t3_core.grade(fixtures_t3.oracle())
    assert "reuse_class" in v.measurements
    assert v.measurements["reuse_class"] is None
    assert any("reuse_class" in n and "not publishable" in n for n in v.notes)
    assert v.as_dict()["measurements"]["reuse_class"] is None


# --- T3.4's support attestation ----------------------------------------------


def test_an_unattestable_support_is_excluded_not_credited():
    """§2 T3.4's own consequence, verbatim and executable.

    Not failed -- that would publish our missing channel as somebody else's
    capability gap (§4). Not credited -- that is what §5c exists to prevent.
    Recorded, flagged vacuous, and excluded from comparison.
    """
    v = t3_core.grade(fixtures_t3.support_not_attested())
    assert v.outcome == PASS
    assert v.failed == []
    assert v.measurements["support_attestation"] == t3ev.UNVERIFIED
    assert v.measurements["excluded_from_comparison"] is True
    a = _ids(v)["T3.4"]
    assert [f.clause for f in a.vacuous_clauses] == [
        "nothing pulled or pushed on it"]
    assert any("EXCLUDED" in n for n in v.notes)


def test_an_attested_support_over_the_threshold_fails_and_prints_the_number():
    v = t3_core.grade(fixtures_t3.held_up_beyond_the_threshold())
    assert v.failed == ["T3.4"]
    a = _ids(v)["T3.4"]
    assert not a.vacuous, "the instrument worked; something was carrying it"
    assert a.measured["peak applied force (N)"] == pytest.approx(180.0)
    assert a.measured["...as a multiple of body weight"] > 1.5
    assert a.measured["per axis"]["force (N)"]["z"] == pytest.approx(180.0)
    assert v.measurements["support_attestation"] == t3ev.ATTESTED
    assert v.measurements["excluded_from_comparison"] is False


def test_a_wrench_with_no_body_weight_to_compare_it_against_is_unverified():
    """0.02 x m.g is not a threshold without m.g, and the row says so."""
    ev = fixtures_t3.oracle()
    ev.base_physics.mass_kg = None
    v = t3_core.grade(ev)
    assert v.measurements["support_attestation"] == t3ev.UNVERIFIED
    assert v.measurements["excluded_from_comparison"] is True
    assert _ids(v)["T3.4"].vacuous
    assert v.outcome == PASS, "unverified is not a failure"


def test_touching_something_that_is_not_the_ground_fails_the_contact_half():
    v = t3_core.grade(fixtures_t3.leaning_on_scene_geometry())
    assert v.failed == ["T3.4"]
    a = _ids(v)["T3.4"]
    assert a.measured["which bodies those were"] == ["wall"]
    assert a.measured["peak applied force (N)"] == 0.0


def test_a_contact_after_the_walk_ended_is_an_arena_finding_not_a_support_one():
    """Reading 7: the robot resting against whatever stopped it.

    The tier grades the walk. A contact that happens once the scored window
    has closed is counted, printed and kept out of the clause.
    """
    t, xyz, rot = fixtures_t3._walk()
    ev = fixtures_t3._evidence(
        t, xyz, rot, gait=fixtures_t3._contacts(t, foreign=(("wall", 36.0,
                                                             40.0),)))
    v = t3_core.grade(ev)
    a = _ids(v)["T3.4"]
    assert a.ok is True
    assert a.measured["contacts with a body that is not the ground, inside "
                      "the scored window"] == 0
    assert a.measured["the same after the walk had ended -- an arena finding, "
                      "not a support finding"] > 0


def test_an_adapter_cannot_relabel_the_thing_holding_the_robot_as_ground():
    """The task's names are authoritative, and the disagreement is printed.

    An adapter free to decide what counts as the ground could call the crane
    the floor and pass T3.4 outright. Here it does exactly that, and the
    contact is still foreign.
    """
    t, xyz, rot = fixtures_t3._walk()
    gait = fixtures_t3._contacts(t)
    gait.contacts.append(t3ev.GroundContact(
        robot_body=fixtures_t3.BASE_NAME, other_body="gantry",
        other_is_ground=True, other_is_robot=False, t_s=10.0))
    v = t3_core.grade(fixtures_t3._evidence(t, xyz, rot, gait=gait))
    assert v.failed == ["T3.4"]
    a = _ids(v)["T3.4"]
    assert a.measured["which bodies those were"] == ["gantry"]
    assert a.measured["the adapter and the task disagreed about the ground"]


# --- T3.5's controller-load clause -------------------------------------------


def test_a_controller_that_never_loaded_fails_on_a_clean_exit_code():
    """The trap that has already voided a quadruped result in this tree."""
    v = t3_core.grade(fixtures_t3.policy_never_loaded())
    assert v.failed == ["T3.5"]
    a = _ids(v)["T3.5"]
    assert a.measured["exit code"] == 0
    assert a.measured["error-class lines"] == 0
    assert a.measured["reached finalize"] is True
    assert a.measured[
        "the policy or controller is attested to have loaded"] is False
    assert not a.vacuous, "the attestation exists; it says no"


def test_no_load_attestation_at_all_is_red_and_vacuous():
    v = t3_core.grade(fixtures_t3.no_policy_attestation())
    a = _ids(v)["T3.5"]
    assert a.ok is False
    assert [f.clause for f in a.vacuous_clauses] == [
        "the thing that drives the robot actually loaded"]
    assert "never attested" in a.detail


def test_missing_attribution_is_invalid_not_failed():
    v = t3_core.grade(fixtures_t3.missing_attribution())
    assert v.outcome == INVALID
    assert "T3.5" in v.failed
    assert any("unattributed" in n for n in v.notes)


# --- T3.1's readings ---------------------------------------------------------


def test_the_distance_graded_is_the_furthest_reached_not_the_final_position():
    """Reading 1: a robot pushed back along a wall after the bar still passed.

    The measured shape of the flagship run in g1-endurance-2026-08-01.md §4.
    """
    t, xyz, rot = fixtures_t3._walk()
    xyz = np.array(xyz)
    back = t >= 36.0
    xyz[back, 0] = xyz[back, 0][0] - 2.0 * (t[back] - 36.0)
    v = t3_core.grade(fixtures_t3._evidence(t, xyz, rot))
    a = _ids(v)["T3.1"]
    assert a.measured["furthest from the start reached (m)"] >= 10.0
    assert a.measured["distance from the start at the end (m)"] < 10.0
    assert a.ok is True


def test_a_teleport_breaks_the_continuity_clause_even_going_further():
    v = t3_core.grade(fixtures_t3.teleported())
    a = _ids(v)["T3.1"]
    assert a.ok is False
    assert a.measured["furthest from the start reached (m)"] > 10.0
    assert a.measured["largest single inter-sample displacement (m)"] > (
        t3_core.MAX_STEP_JUMP_M)


def test_a_clock_that_goes_backwards_is_not_one_continuous_run():
    ev = fixtures_t3.oracle()
    t = np.array(ev.base_pose.t)
    t[900:] = t[900:] - 5.0
    ev.base_pose.t = t
    assert _ids(t3_core.grade(ev))["T3.1"].ok is False


def test_the_teleport_clause_is_vacuous_on_a_series_too_coarse_to_see_one():
    """A 0.5 m bound at a 1 Hz sample rate is not a teleport test."""
    ev = fixtures_t3.oracle()
    ev.base_pose.t = np.arange(ev.base_pose.n_samples) * 1.0
    a = _ids(t3_core.grade(ev))["T3.1"]
    jump = [f for f in a.falsifiers if f.clause == "one continuous run"][0]
    assert not jump.present


# --- T3.2's readings ---------------------------------------------------------


def test_a_turning_walk_is_not_a_fall():
    """Reading: yaw is discarded. A robot that turned has turned."""
    t, xyz, _rot = fixtures_t3._walk()
    rot = fixtures_t3._rot_track(t, yaw=np.linspace(0.0, 2.5, len(t)))
    v = t3_core.grade(fixtures_t3._evidence(t, xyz, rot))
    a = _ids(v)["T3.2"]
    assert a.ok is True
    assert a.measured["largest |roll| (rad)"] < 0.2


def test_the_standing_height_is_per_robot_and_not_a_constant():
    """A robot half the size is graded by the same rule, not the same number."""
    ev = fixtures_t3.oracle()
    ev.standing.z_m = 0.21
    ev.base_pose.xyz = np.array(ev.base_pose.xyz)
    ev.base_pose.xyz[:, 2] = ev.base_pose.xyz[:, 2] - 0.21
    a = _ids(t3_core.grade(ev))["T3.2"]
    assert a.ok is True
    assert a.measured["the height it had to stay above (m)"] == pytest.approx(
        0.126)


def test_an_unattested_standing_height_falls_back_and_says_so():
    v = t3_core.grade(fixtures_t3.no_attested_standing_height())
    a = _ids(v)["T3.2"]
    assert a.ok is True
    assert a.vacuous
    assert "first recorded sample" in a.measured[
        "how the standing height was decided"]


def test_without_orientation_the_upright_clause_is_red_and_vacuous():
    v = t3_core.grade(fixtures_t3.no_base_orientation())
    a = _ids(v)["T3.2"]
    assert a.ok is False
    assert [f.clause for f in a.vacuous_clauses] == ["stayed upright"]
    assert a.measured["largest |roll| (rad)"] is None


def test_the_combined_tilt_is_reported_and_is_not_the_threshold():
    """0.8 rad of roll AND 0.8 of pitch is ~1.1 rad off vertical and passes."""
    t, xyz, _rot = fixtures_t3._walk()
    rot = fixtures_t3._rot_track(t, roll_amp=0.75, pitch_amp=0.75, freq=0.05)
    a = _ids(t3_core.grade(fixtures_t3._evidence(t, xyz, rot)))["T3.2"]
    assert a.ok is True
    assert a.measured["largest angle off vertical, which is NOT the tier's "
                      "threshold (rad)"] > t3_core.MAX_TILT_RAD


# --- T3.3's readings ---------------------------------------------------------


def test_the_aggregate_contact_signal_of_a_perfect_walk_never_changes():
    """Reading 6, and why the count is per body.

    The oracle is a textbook trot: some foot is always down. Counted in
    aggregate a perfect walk scores ZERO transitions; counted per body it
    scores hundreds. This is the test that keeps the reading honest.
    """
    ev = fixtures_t3.oracle()
    ground, _foreign, _d = t3ev.classify_contacts(ev.gait, ev.surface)
    ts, bodies, state = t3ev.contact_states(ev.gait, ground)
    assert len(bodies) == 4
    aggregate = state.any(axis=1)
    assert aggregate.all(), "some foot is down at every sample"
    assert int(np.count_nonzero(np.diff(aggregate.astype(np.int8)))) == 0
    per_body, cycles, _per = t3_core.contact_transitions(state)
    assert per_body > 100 and cycles > 40


def test_a_body_that_slides_fails_on_the_footfalls():
    v = t3_core.grade(fixtures_t3.slid_without_gait())
    assert v.failed == ["T3.3"]
    a = _ids(v)["T3.3"]
    assert a.measured["make-and-break transitions of ground contact"] == 0
    assert not a.vacuous, "the contacts were reported; they never broke"
    # the bob is still measured and still printed -- it just decides nothing
    assert a.measured["vertical oscillation about its trend, RMS (m)"] < (
        t3_core.MIN_BOB_RMS_M)
    assert a.measured["...the retired bob clause would have passed"] is False


def test_a_contact_channel_with_no_query_times_cannot_see_a_lifted_foot():
    v = t3_core.grade(fixtures_t3.blind_contact_query())
    a = _ids(v)["T3.3"]
    assert a.ok is False
    assert "picked its feet up" in [f.clause for f in a.vacuous_clauses]
    assert a.measured["times the contact query ran"] is None


# --- T3.3's gait-liveness predicate, repaired 2026-08-02 ---------------------
#
# Reading 5b in ``t3_core``: ground-contact make/breaks remain the gate for
# "it stepped rather than slid"; the vertical bob became a reported
# measurement. These five tests are the executable form of that decision --
# that the bar it replaced is still visible, that the fixture the bob used to
# be needed for is still red without it, and that the walk the bob used to fail
# now passes.


def test_the_bob_is_measured_and_printed_and_is_not_a_pass_condition():
    """The whole of the 2026-08-02 decision, in one row.

    A verdict that stopped printing the bob would have hidden a measurement;
    one that stopped printing the bar it used to be compared against would have
    made the change undiscoverable from the artifact.
    """
    a = _ids(t3_core.grade(fixtures_t3.oracle()))["T3.3"]
    assert a.measured["vertical oscillation about its trend, RMS (m)"] > 0.0
    assert a.measured[
        "...against the bar that was RETIRED on 2026-08-02, which is now "
        "measured and never graded (m)"] == t3_core.MIN_BOB_RMS_M
    assert a.measured["...the retired bob clause would have passed"] is True
    assert "NEVER GRADED" in a.threshold["vertical RMS about the trend (m)"]
    # ...and no falsifier claims the bob can fail the assertion any more,
    # because it cannot, and a declared way-to-fail that is not applied is the
    # over-claim the falsifier machinery exists to prevent
    assert "bobbed like something on legs" not in [
        f.clause for f in a.falsifiers]
    assert {f.clause for f in a.falsifiers} == {"moved at a walking pace",
                                                "picked its feet up"}


def test_a_slow_statically_stable_walk_passes_and_says_the_old_bar_failed_it():
    """THE must-PASS control: the false negative the repair exists for.

    0.27 m/s, 16.47 m, 732 make-and-break ground-contact transitions, a
    0.0018 m bob. It is a walk by every reading of the word, and the retired
    clause failed it.
    """
    v = t3_core.grade(fixtures_t3.slow_static_gait())
    assert v.outcome == PASS, v.summary()
    assert v.failed == []
    a = _ids(v)["T3.3"]
    assert a.measured[
        "mean speed made good over the scored window (m/s)"] == pytest.approx(
            0.27, abs=0.01)
    assert a.measured["make-and-break transitions of ground contact"] >= 700
    bob = a.measured["vertical oscillation about its trend, RMS (m)"]
    assert bob < t3_core.MIN_BOB_RMS_M
    assert a.measured["...the retired bob clause would have passed"] is False
    # the row says, in its own words, that this is a pass the old reading lost
    assert "PASSES ONLY UNDER THE 2026-08-02 READING" in a.detail
    assert "%.5f" % bob in a.detail
    # and the distance clause is untouched: it really did cross ten metres
    assert _ids(v)["T3.1"].measured[
        "furthest from the start reached (m)"] > 16.0


def test_the_old_predicate_reddens_the_control_and_the_new_one_does_not():
    """The before/after, computed rather than asserted from memory.

    The retired conjunct is re-applied by hand to the same row: it fails, the
    graded assertion passes, and nothing else about the run changed.
    """
    a = _ids(t3_core.grade(fixtures_t3.slow_static_gait()))["T3.3"]
    bob = a.measured["vertical oscillation about its trend, RMS (m)"]
    speed = a.measured["mean speed made good over the scored window (m/s)"]
    edges = a.measured["make-and-break transitions of ground contact"]
    old = bool(speed >= t3_core.MIN_MEAN_SPEED_MPS
               and bob >= t3_core.MIN_BOB_RMS_M
               and edges >= t3_core.MIN_CONTACT_TRANSITIONS)
    assert old is False, "the fixture must reproduce the demonstrated red"
    assert a.ok is True


def test_the_slider_still_reddens_through_the_transition_clause_alone():
    """(a) The guard that keeps the repair from being a weakening.

    ``slid_without_gait`` is §5c's named "crawling robot" and it used to fail
    two conjuncts at once. Here the bob is put BACK -- a full 0.012 m of it,
    the oracle's own -- while the four feet stay welded to the floor. If the
    make-and-break clause were not carrying the claim on its own, this would
    now be green.
    """
    t, xyz, rot = fixtures_t3._walk()          # bobs and rolls like the oracle
    ev = fixtures_t3._evidence(t, xyz, rot,
                               gait=fixtures_t3._contacts(t, gait=False))
    v = t3_core.grade(ev)
    a = _ids(v)["T3.3"]
    assert a.measured["vertical oscillation about its trend, RMS (m)"] > (
        t3_core.MIN_BOB_RMS_M), "this variant DOES bob"
    assert a.measured["...the retired bob clause would have passed"] is True
    assert a.measured["make-and-break transitions of ground contact"] == 0
    assert a.ok is False, "the transition clause carries it alone"
    assert v.failed == ["T3.3"], "and it still isolates one assertion"


def test_the_repair_did_not_open_the_hole_it_was_accused_of_opening():
    """The exploit the bob clause was guarding against, priced.

    A body that never leaves the ground now has to defeat TWO surviving
    clauses to be graded as a walk: >= 8 make-and-break transitions AND
    >= 0.15 m/s made good over the scored window. Neither is reachable by
    sliding: a dragged body reports zero transitions, and a body that cycles
    contacts on the spot goes nowhere. Both halves are asserted here.
    """
    t, xyz, rot = fixtures_t3._walk(bob=0.0, roll_amp=0.0, pitch_amp=0.0)
    dragged = t3_core.grade(fixtures_t3._evidence(
        t, xyz, rot, gait=fixtures_t3._contacts(t, gait=False)))
    assert _ids(dragged)["T3.3"].ok is False

    # cycling its feet at a standstill: transitions in the hundreds, no travel
    t2, _xyz2, rot2 = fixtures_t3._walk(speed=0.0)
    on_the_spot = t3_core.grade(fixtures_t3._evidence(
        t2, fixtures_t3._base_track(t2, speed=0.0), rot2,
        gait=fixtures_t3._contacts(t2)))
    a = _ids(on_the_spot)["T3.3"]
    assert a.measured["make-and-break transitions of ground contact"] > 8
    assert a.measured[
        "mean speed made good over the scored window (m/s)"] < (
            t3_core.MIN_MEAN_SPEED_MPS)
    assert a.ok is False


def test_a_series_too_coarse_to_resolve_a_gait_says_so_beside_the_bob():
    """The witness that used to hang off the retired clause, kept as a caveat.

    A 0.5 s sample interval aliases a gait cycle away, so the bob printed on
    such a row is not a measurement of a bob. That was a vacuity witness while
    the bob was graded; now that it is only reported, it is a caveat ON the
    number and is printed next to it.
    """
    ev = fixtures_t3.oracle()
    ev.base_pose.t = np.arange(ev.base_pose.n_samples) * 0.5
    a = _ids(t3_core.grade(ev))["T3.3"]
    assert a.measured["the sample interval resolves a gait cycle, so the bob "
                      "above is measured rather than aliased"] is False
    fine = _ids(t3_core.grade(fixtures_t3.oracle()))["T3.3"]
    assert fine.measured["the sample interval resolves a gait cycle, so the "
                         "bob above is measured rather than aliased"] is True


def test_the_whole_run_mean_is_printed_and_labelled_as_not_the_walking_speed():
    v = t3_core.grade(fixtures_t3.arena_bound())
    a = _ids(v)["T3.3"]
    scored = a.measured["mean speed made good over the scored window (m/s)"]
    whole = a.measured["the same over the WHOLE recording, which is NOT the "
                       "walking speed (m/s)"]
    assert scored >= t3_core.MIN_MEAN_SPEED_MPS
    assert whole < scored / 1.5, "the wall halves it, and both are printed"


# --- the arena, and the run the world ended ----------------------------------


def test_a_run_the_world_ended_is_recorded_as_such_not_as_a_bad_gait():
    """g1-endurance-2026-08-01.md §8, made executable.

    Six clean runs of this tree's flagship were ended by an arena wall and
    would have been published as "the walk degrades to 0.037 m/s" by a grader
    that did not know where the walls were.
    """
    v = t3_core.grade(fixtures_t3.arena_bound())
    assert v.failed == ["T3.1"], "the distance, and nothing else"
    assert v.measurements["termination"]["how the run ended"] == (
        t3ev.TERMINATION_ARENA)
    assert v.measurements["arena"]["the run became bound by the region's "
                                   "edge"] is True
    assert v.measurements["arena"]["the run-up requirement was met"] is False
    assert any("world, not the walker" in n for n in v.notes)
    # ...and the clause says it could not have passed in a region that size
    a = _ids(v)["T3.1"]
    assert [f.clause for f in a.vacuous_clauses] == ["crossed ten metres"]


def test_a_plateau_in_open_floor_is_a_stall_and_is_scored_in_full():
    """The guard on reading 3: only a boundary truncates the window."""
    t, xyz, rot = fixtures_t3._walk(stop_at=15.0)
    v = t3_core.grade(fixtures_t3._evidence(t, xyz, rot))   # the BIG arena
    assert v.measurements["termination"]["how the run ended"] == (
        t3ev.TERMINATION_CONTROLLER)
    a = _ids(v)["T3.3"]
    # scored over the whole run, so the twenty-five standing seconds count:
    # 4.5 m in 40 s is 0.11 m/s, and the clause says so
    assert a.measured["the scored window is (s)"] > 35.0
    assert a.measured["mean speed made good over the scored window (m/s)"] < (
        t3_core.MIN_MEAN_SPEED_MPS)
    assert a.ok is False


def test_a_fall_is_named_as_the_termination_even_next_to_a_wall():
    v = t3_core.grade(fixtures_t3.fell_height())
    assert v.measurements["termination"]["how the run ended"] == (
        t3ev.TERMINATION_FELL)
    assert "t = 35" in v.measurements["termination"]["why"]


def test_with_no_arena_channel_no_stop_is_ever_declared():
    """The conservative reading: an unknown boundary truncates nothing."""
    ev = fixtures_t3.arena_bound()
    ev.arena = t3ev.EMPTY_ARENA
    v = t3_core.grade(ev)
    assert v.measurements["termination"]["how the run ended"] == (
        t3ev.TERMINATION_CONTROLLER)
    assert v.measurements["arena"]["the run-up requirement was met"] is None


def test_the_two_headroom_fields_use_the_plans_own_vocabulary():
    """§3.2's ``distance_to_termination_m`` and ``termination_cause``.

    Adopted verbatim rather than forked: one recorder with two vocabularies
    for one measurement would make a T3 cell and a T4 cell need translating
    before they could be read side by side. Neither is a pass condition.
    """
    assert set(t3ev.TERMINATIONS) == {"fell", "arena_geometry", "time_limit",
                                      "controller_stopped", "unknown"}
    v = t3_core.grade(fixtures_t3.oracle())
    assert v.measurements["termination_cause"] in t3ev.TERMINATIONS
    assert v.measurements["distance_to_termination_m"] == pytest.approx(
        12.0, abs=0.05)


def test_the_distance_to_termination_is_not_clipped_at_the_bar():
    """"A 10.1 m and a 29.5 m are distinguishable in the published grid."""
    short = t3_core.grade(fixtures_t3.stopped_short())
    long_ = t3_core.grade(fixtures_t3.oracle())
    assert short.measurements["distance_to_termination_m"] < 10.0
    assert long_.measurements["distance_to_termination_m"] > 10.0
    assert short.measurements["termination_cause"] == t3ev.TERMINATION_TIME_LIMIT


def test_the_headroom_fields_are_present_even_when_nothing_was_measured():
    ev = fixtures_t3.oracle()
    ev.bundle.artifact = None
    v = t3_core.grade(ev)
    assert v.measurements["distance_to_termination_m"] is None
    assert v.measurements["termination_cause"] == t3ev.TERMINATION_UNKNOWN


def test_the_stated_free_run_up_is_the_plans_multiple_of_the_bar():
    v = t3_core.grade(fixtures_t3.oracle())
    assert v.measurements["arena"]["the free run-up this task requires (m)"] \
        == pytest.approx(t3_core.MIN_RUN_UP_FACTOR * t3_core.NET_DISPLACEMENT_M)
    assert v.measurements["arena"]["the run-up requirement was met"] is True


# --- absent evidence ---------------------------------------------------------


def test_no_deliverable_scores_zero_of_five():
    ev = fixtures_t3.oracle()
    ev.bundle.artifact = None
    v = t3_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert v.progress == 0


def test_a_missing_ground_declaration_is_invalid_not_failed():
    """The names are task data. Losing one is a broken instrument."""
    ev = fixtures_t3.oracle()
    ev.surface = t3ev.WalkingSurface()
    v = t3_core.grade(ev)
    assert v.outcome == INVALID
    assert v.outcome != FAIL
    assert any("walking surface" in n or "ground the robot walks on" in n
               for n in v.notes)


def test_a_series_about_the_wrong_body_is_refused_rather_than_graded():
    """Every number in such a verdict would look fine and be about a decoy."""
    ev = fixtures_t3.oracle()
    ev.base_pose.body = "some_other_body"
    v = t3_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert "the task names" in _ids(v)["T3.1"].detail


def test_a_series_that_does_not_say_which_body_it_is_about_is_still_graded():
    ev = fixtures_t3.oracle()
    ev.base_pose.body = ""
    assert t3_core.grade(ev).outcome == PASS


# --- the measurement helpers -------------------------------------------------


def test_roll_pitch_inverts_the_convention_the_fixtures_build():
    t = np.array([0.0, 1.0])
    rot = fixtures_t3._rot_track(t, roll_amp=0.0, pitch_amp=0.0, yaw=1.0,
                                 topple_at=-1.0, topple_roll=0.4)
    roll, pitch = t3ev.roll_pitch(rot)
    assert roll[-1] == pytest.approx(0.4, abs=1e-9)
    assert pitch[-1] == pytest.approx(0.0, abs=1e-9)
    assert t3ev.roll_pitch(None) == (None, None)


def test_detrended_rms_removes_a_slope_and_keeps_the_oscillation():
    t = np.arange(0.0, 10.0, 0.01)
    z = 0.4 + 0.03 * t + 0.01 * np.sin(2.0 * np.pi * 1.5 * t)
    assert t3ev.detrended_rms(t, z) == pytest.approx(0.01 / np.sqrt(2.0),
                                                     rel=1e-2)
    assert t3ev.detrended_rms(t, np.full(len(t), 0.4)) == pytest.approx(0.0,
                                                                        abs=1e-12)


def test_trailing_plateau_measures_the_stop_at_the_end_only():
    t = np.arange(0.0, 10.0, 0.1)
    xy = np.stack([np.minimum(t, 5.0) * 0.3, np.zeros(len(t))], axis=1)
    secs, i0 = t3_core.trailing_plateau(t, xy, 0.25)
    assert secs == pytest.approx(5.0, abs=1.0)
    assert t[i0] >= 4.0


def test_an_arena_stop_needs_all_three_conditions():
    t = np.arange(0.0, 40.0, 0.1)
    xy = np.stack([np.minimum(t, 20.0) * 0.3, np.zeros(len(t))], axis=1)
    edge = t3ev.ArenaBounds(aabb_min=(-3.0, -4.0, 0.0),
                            aabb_max=(6.3, 4.0, 0.0), source="fixture")
    far = t3ev.ArenaBounds(aabb_min=(-3.0, -4.0, 0.0),
                           aabb_max=(60.0, 4.0, 0.0), source="fixture")
    assert t3_core.arena_stop_index(t, xy, edge) is not None
    assert t3_core.arena_stop_index(t, xy, far) is None       # not at an edge
    assert t3_core.arena_stop_index(t, xy, t3ev.EMPTY_ARENA) is None
    moving = np.stack([t * 0.3, np.zeros(len(t))], axis=1)
    assert t3_core.arena_stop_index(t, moving, edge) is None  # never stopped


def test_scored_end_prefers_the_bar_then_the_arena_then_the_run():
    t = np.arange(0.0, 40.0, 0.1)
    disp = t * 0.3
    i, rule = t3_core.scored_end(t, disp, None)
    assert t[i] == pytest.approx(33.4, abs=0.2) and "bar" in rule
    short = np.minimum(t, 20.0) * 0.3
    i, rule = t3_core.scored_end(t, short, 200)
    assert i == 200 and "walking region" in rule
    i, rule = t3_core.scored_end(t, short, None)
    assert i == len(t) - 1 and "last sample" in rule


def test_contact_transitions_counts_makes_and_breaks_and_reports_cycles():
    state = np.array([[True], [False], [True], [False], [True]])
    edges, cycles, per = t3_core.contact_transitions(state)
    assert edges == 4 and cycles == 2
    assert per[0] == {"makes": 2, "breaks": 2}
    assert t3_core.contact_transitions(None) == (None, None, {})


def test_contact_states_needs_the_times_the_query_ran():
    c = [t3ev.GroundContact(robot_body="foot", other_body="ground",
                            other_is_ground=True, t_s=0.1)]
    without = t3ev.GaitContactObservation(contacts=c, supported=True)
    assert t3ev.contact_states(without, c) == (None, [], None)
    with_times = t3ev.GaitContactObservation(
        contacts=c, sample_times=np.array([0.0, 0.1, 0.2]), supported=True)
    ts, bodies, state = t3ev.contact_states(with_times, c)
    assert bodies == ["foot"]
    assert state[:, 0].tolist() == [False, True, False]


def test_classify_contacts_resolves_every_disagreement_the_strict_way():
    surface = t3ev.WalkingSurface(names=("ground",), source="fixture")
    obs = t3ev.GaitContactObservation(contacts=[
        t3ev.GroundContact(robot_body="foot", other_body="ground",
                           other_is_ground=True),
        t3ev.GroundContact(robot_body="foot", other_body="ground",
                           other_is_ground=False),      # adapter denies it
        t3ev.GroundContact(robot_body="foot", other_body="crane",
                           other_is_ground=True),       # adapter claims it
    ], supported=True)
    ground, foreign, disagreements = t3ev.classify_contacts(obs, surface)
    assert len(ground) == 1 and len(foreign) == 2
    assert len(disagreements) == 2


def test_displacement_from_start_is_horizontal_only():
    xy = np.array([[0.0, 0.0], [3.0, 4.0]])
    assert t3_core.displacement_from_start(xy).tolist() == [0.0, 5.0]


# --- the applied-support channel ---------------------------------------------


def test_an_unwired_support_channel_and_a_measured_zero_are_different():
    """There is deliberately no "nothing was applied" default."""
    assert t3ev.EMPTY_APPLIED_SUPPORT.attested is None
    assert not t3ev.EMPTY_APPLIED_SUPPORT.usable
    measured = t3ev.AppliedSupport(attested=True, peak_force_n=0.0,
                                   peak_torque_nm=0.0, fraction_nonzero=0.0,
                                   source="fixture")
    assert measured.usable and measured.peak_force == 0.0


def test_the_support_channel_reports_per_axis_peaks():
    """The G1 measurement is why: 0 N vertical, 69 N.m of attitude, all walk."""
    n = 10
    s = t3ev.AppliedSupport(
        attested=True, t=np.arange(n) * 0.1,
        force=np.zeros((n, 3)),
        torque=np.tile(np.array([69.2, 21.4, 0.0]), (n, 1)),
        source="fixture")
    assert s.per_channel["force (N)"]["z"] == 0.0
    assert s.per_channel["torque (N.m)"]["x"] == pytest.approx(69.2)
    assert s.fraction_window_nonzero == pytest.approx(1.0)


# --- the ladder's own evidence channels --------------------------------------


def test_the_evidence_contract_names_the_usual_adapter_mistakes():
    ev = fixtures_t3.oracle()
    ev.surface.source = ""
    ev.base_pose.rot = None
    ev.standing.z_m = None
    ev.support = t3ev.EMPTY_APPLIED_SUPPORT
    ev.world.gravity_mps2 = None
    ev.controller = t3ev.EMPTY_CONTROLLER_LOAD
    ev.arena = t3ev.EMPTY_ARENA
    problems = t3ev.check_t3_evidence(ev)
    assert any("source citation" in p for p in problems)
    assert any("no orientation" in p for p in problems)
    assert any("standing height" in p for p in problems)
    assert any("applied support" in p for p in problems)
    assert any("body weight" in p for p in problems)
    assert any("controller load" in p for p in problems)
    assert any("arena" in p for p in problems)


def test_every_channel_gap_names_the_assertions_it_breaks():
    for channel, assertions in t3ev.T3_CHANNEL_ASSERTIONS.items():
        for aid in assertions:
            assert aid in fixtures_t3.ASSERTIONS, (channel, aid)
    # the arena breaks NOTHING: it is a finding, never a pass condition
    assert t3ev.T3_CHANNEL_ASSERTIONS["arena"] == ()


def test_the_ladder_declares_what_a_new_column_must_add_for_T3():
    keys = {k for k, _why in t3ev.LADDER_REQUIRED_EVIDENCE_T3}
    assert keys == {"walking_surface", "base_pose", "standing_height",
                    "gait_contacts", "applied_support", "base_mass",
                    "controller_load", "arena"}
    for _k, why in t3ev.LADDER_REQUIRED_EVIDENCE_T3:
        assert why


def test_the_standing_height_is_derivable_from_the_frozen_inventory():
    """The one T3 channel that needs no new sampler on any column."""
    bundle = fixtures_t3.oracle().bundle
    h = t3ev.standing_height_from_bundle(bundle, name=fixtures_t3.BASE_NAME)
    assert h.usable
    assert h.z_m == pytest.approx(fixtures_t3.STAND_Z)
    assert not t3ev.standing_height_from_bundle(bundle, name="nobody").usable


def test_the_arena_is_derivable_from_the_frozen_inventory():
    bundle = fixtures_t3.oracle().bundle
    surface = t3ev.WalkingSurface(names=("ground",), source="fixture")
    arena = t3ev.arena_from_bundle(bundle, surface)
    assert arena.usable
    assert arena.free_run_up_from((0.0, 0.0)) > 15.0
    assert not t3ev.arena_from_bundle(
        bundle, t3ev.WalkingSurface(names=("nothing",))).usable


def test_a_pose_series_derived_from_the_frozen_bundle_has_no_orientation():
    """The gap, asserted rather than described: the blocker is ours."""
    from agentbench.graders.evidence import Trajectory
    bundle = fixtures_t3.oracle().bundle
    bundle.trajectory = Trajectory(
        body_ids=["#31"], t=np.arange(3) * 0.02, xyz=np.zeros((1, 3, 3)),
        dt_s=0.02, source="frozen")
    series = t3ev.pose_series_from_bundle(bundle, name=fixtures_t3.BASE_NAME)
    assert series.usable
    assert not series.has_orientation


# --- the shim over the ladder's adapters -------------------------------------


class _StubColumn:
    """A column that produces a bundle and none of T3's channels."""

    def __init__(self, bundle):
        self.bundle = bundle

    def build_bundle(self, *_a, **_kw):
        return self.bundle


def test_a_column_with_no_t3_channels_falls_back_and_names_every_gap(
        monkeypatch):
    """The whole point of the gap machinery: the blocker must land on us."""
    bundle = fixtures_t3.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = t3.build_evidence(t3_core.TASK, sim="stub")
    gaps = t3ev.unanswered_channels(ev)
    assert set(gaps) >= {"base_pose", "standing_height", "gait_contacts",
                         "applied_support", "base_mass", "gravity",
                         "controller_load"}
    # ...and the one the frozen contract CAN answer is not in the gap list
    assert "arena" not in gaps
    assert any("scaffolding_defect_ours" in n for n in ev.notes)


def test_a_column_that_supplies_channels_is_used_unchanged(monkeypatch):
    ref = fixtures_t3.oracle()
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(ref.bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = t3.build_evidence(
        t3_core.TASK, sim="stub",
        channels={"base_pose": ref.base_pose, "standing": ref.standing,
                  "gait": ref.gait, "support": ref.support,
                  "arena": ref.arena, "base_physics": ref.base_physics,
                  "world": ref.world, "controller": ref.controller})
    ev.robot_name = fixtures_t3.BASE_NAME       # the stub bundle's own body
    assert t3ev.unanswered_channels(ev) == {}
    assert t3_core.grade(ev).outcome == PASS


def test_the_ground_names_come_from_the_task_file_with_its_citation(
        monkeypatch):
    bundle = fixtures_t3.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = t3.build_evidence(t3_core.TASK, sim="stub")
    assert "ground" in ev.surface.names
    assert "meta.json" in ev.surface.source
    assert ev.robot_name == "base_link"


def test_the_grade_entry_point_records_the_unanswered_channels(monkeypatch):
    bundle = fixtures_t3.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    v = t3.grade(None, sim="stub")
    assert "unanswered_channels" in v.measurements
    assert any("scaffolding_defect_ours" in n for n in v.notes)


# --- the task assets ---------------------------------------------------------


def test_the_prompt_is_one_sentence():
    task = tasks.get(t3_core.TASK)
    assert task.prompt.count(".") == 1
    assert task.prompt.endswith(".")
    assert "\n" not in task.prompt


# §5a's outcome-not-feature rule: "a tier statement may not name a file format,
# an endpoint, a node type, a solver, a training method, a checkpoint, or any
# product's proper noun."
_FORBIDDEN_IN_PROMPT = (
    "urdf", "wbt", "sdf", "usd", "mjcf", "xacro", "xml", "json", "yaml",
    "omnisim", "webots", "gazebo", "isaac", "mujoco", "genesis", "newton",
    "ode", "solver", "supervisor", "controller", "harness", "proto", "node",
    "endpoint", "http", "api", "checkpoint", "policy", "reinforcement",
    "import", "train", "training", "gait", "trot", "reward")


def test_the_prompt_names_no_format_no_endpoint_and_no_method():
    """The last few tokens are T3's own: HOW it walks is not the task."""
    task = tasks.get(t3_core.TASK)
    low = task.prompt.lower()
    for token in _FORBIDDEN_IN_PROMPT:
        assert not re.search(r"\b%s\b" % re.escape(token), low), token


def test_the_prompt_states_the_outcome_and_not_the_method():
    task = tasks.get(t3_core.TASK)
    low = task.prompt.lower()
    for word in ("four-legged", "ten metres", "without falling"):
        assert word in low


def test_the_container_ships_the_description_and_nothing_else():
    task = tasks.get(t3_core.TASK)
    on_disk = sorted(p.relative_to(task.container_dir).as_posix()
                     for p in task.container_dir.rglob("*") if p.is_file())
    declared = sorted(
        (task.container_dir / n).relative_to(task.container_dir).as_posix()
        for n in task.meta["container"]["files"])
    assert on_disk == declared
    assert not any(p.suffix in (".wbt", ".sdf", ".usd", ".xml", ".py", ".sh")
                   for p in task.container_files), \
        "the container ships a scene, a script or a converted model"


def test_the_container_ships_no_control_code_and_no_gait():
    task = tasks.get(t3_core.TASK)
    descriptions = [p for p in task.container_files if p.suffix == ".urdf"]
    assert len(descriptions) == 1
    text = descriptions[0].read_text(encoding="utf-8").lower()
    for word in ("trajectory", "waypoint", "gait", "policy", "controller",
                 "checkpoint"):
        assert word not in text, word


def test_the_shipped_description_parses_and_references_no_outside_file():
    """A malformed description would fail every column for OUR reason.

    And a reference to a file we did not ship would fail whichever column
    resolves paths least forgivingly, which is the T1 open question this
    container exists to avoid repeating.
    """
    task = tasks.get(t3_core.TASK)
    root = ET.parse(task.container_files[0]).getroot()
    assert root.tag == "robot" and root.get("name")
    assert root.findall(".//mesh") == []
    for el in root.iter():
        for key, value in el.attrib.items():
            assert "://" not in value, (el.tag, key, value)
            assert not value.startswith(("/", "~")), (key, value)
    for link in root.findall("link"):
        assert link.find("inertial") is not None, link.get("name")
        mass = link.find("inertial/mass")
        assert mass is not None and float(mass.get("value")) > 0.0


def test_the_robot_is_a_quadruped_with_twelve_driven_joints_and_no_welds():
    """The one structural claim the container makes about itself, checked."""
    task = tasks.get(t3_core.TASK)
    root = ET.parse(task.container_files[0]).getroot()
    joints = root.findall("joint")
    assert len(root.findall("link")) == 13
    assert len(joints) == 12
    assert all(j.get("type") == "revolute" for j in joints), \
        "a fixed joint would let an importer fuse away a named foot body"
    for leg in ("fl", "fr", "rl", "rr"):
        for pre, axis in (("hip", "1 0 0"), ("thigh", "0 1 0"),
                          ("knee", "0 1 0")):
            j = [x for x in joints if x.get("name") == "%s_%s_joint"
                 % (pre, leg)]
            assert len(j) == 1, (pre, leg)
            assert j[0].find("axis").get("xyz") == axis
            lim = j[0].find("limit")
            assert float(lim.get("effort")) > 0.0
            assert float(lim.get("upper")) > float(lim.get("lower"))
    # every knee is bent: an upper limit at or above zero admits a straight
    # leg, which is the two-link solution's own singularity
    for leg in ("fl", "fr", "rl", "rr"):
        knee = [x for x in joints if x.get("name") == "knee_%s_joint" % leg][0]
        assert float(knee.find("limit").get("upper")) < 0.0


def test_the_declared_mass_matches_what_the_task_file_says():
    task = tasks.get(t3_core.TASK)
    root = ET.parse(task.container_files[0]).getroot()
    total = sum(float(m.get("value")) for m in root.iter("mass"))
    assert total == pytest.approx(task.meta["robot"]["mass_kg_declared"])


def test_the_task_declares_which_bodies_are_the_ground_and_why_it_is_task_data():
    task = tasks.get(t3_core.TASK)
    ground = task.meta["ground"]
    assert "ground" in ground["names"] and len(ground["names"]) >= 4
    assert "no adapter may supply it" in ground["why_not_in_the_container"] \
        or "could label the thing holding the robot up" in \
        ground["why_not_in_the_container"]
    assert ground["open_question_for_the_freeze"]


def test_the_task_states_a_free_run_up_of_at_least_one_and_a_half_bars():
    """The build note g1-endurance-2026-08-01.md §8 requires of every column."""
    task = tasks.get(t3_core.TASK)
    arena = task.meta["arena"]
    assert arena["min_free_run_up_m"] == pytest.approx(
        t3_core.MIN_RUN_UP_FACTOR * t3_core.NET_DISPLACEMENT_M)
    assert arena["min_free_run_up_m"] >= 1.5 * t3_core.NET_DISPLACEMENT_M
    assert arena["why"] and arena["the_guard_on_that_last_one"]


def test_the_task_declares_the_method_is_recorded_and_not_graded():
    task = tasks.get(t3_core.TASK)
    block = task.meta["method"]
    assert block["graded"] is False
    assert set(block["values"]) == set(t3ev.METHODS)
    assert block["why_not"] and block["recorded_where"]


def test_the_task_declares_the_support_attestation_rule_and_its_consequence():
    task = tasks.get(t3_core.TASK)
    block = task.meta["support_attestation"]
    assert set(block["values"]) == {t3ev.ATTESTED, t3ev.UNVERIFIED}
    assert "EXCLUDED FROM COMPARISON" in block["the_rule"]
    assert "not credited" in block["the_consequence_as_implemented"] \
        or "NOT credited" in block["the_consequence_as_implemented"]
    assert "support_not_attested" in block["the_consequence_as_implemented"]


def test_the_task_says_the_grader_cannot_decide_reuse_class():
    task = tasks.get(t3_core.TASK)
    block = task.meta["reuse_class"]
    assert "reviewer" in block["decided_by"]
    assert block["why_the_grader_leaves_it_null"]


def test_the_task_flags_the_one_clause_that_goes_beyond_the_plans_text():
    """Reading 2. Declared in the file a reviewer opens, not buried."""
    note = tasks.get(t3_core.TASK).meta["grading_readings"]["T3.1_derived_clause"]
    assert "DERIVED" in note and "MAX_STEP_JUMP_M" in note


def test_the_task_records_the_open_questions_it_found_rather_than_hiding_them():
    """§5c: a gap a reviewer should see BEFORE the freeze, not after a grid."""
    readings = tasks.get(t3_core.TASK).meta["grading_readings"]
    for key in ("T3_OPEN_QUESTION_a_run_that_walks_and_then_stops_is_not_penalised",
                "T3_OPEN_QUESTION_no_clause_reads_the_terrain"):
        assert readings[key].startswith("OPEN AT THE FREEZE")
    # the bob question that used to stand here is DECIDED, not deleted, and
    # the reading below asserts the decision is on the record instead
    assert not [k for k in readings if "OPEN_QUESTION" in k and "bob" in k]


def test_the_task_records_the_bob_decision_with_its_date_and_its_reason():
    """§5a: a repaired predicate is a decision, and it has to be attackable.

    The record must survive as three separate statements: what the defect was
    (with the demonstration that found it), what was decided, and that no
    threshold moved -- because "we fixed it" with the measurement deleted is
    indistinguishable from moving a bar after seeing a result.
    """
    readings = tasks.get(t3_core.TASK).meta["grading_readings"]
    decision = readings["T3.3_gait_liveness_is_the_transition_count"]
    defect = readings["T3.3_REPAIRED_2026-08-02"]

    assert decision.startswith("DECIDED 2026-08-02")
    assert "NO THRESHOLD WAS MOVED" in decision
    assert "slow_static_gait" in decision and "slid_without_gait" in decision
    # the tier's own text still states the bob; the divergence is declared
    assert "still states the bob" in decision

    # the demonstration that forced it is kept, numbers and all
    for number in ("0.27 m/s", "16.5 m", "733", "0.0018 m", "0.005 m"):
        assert number in defect, number
    assert "MIN_BOB_RMS_M" in readings["T3.3_bob"]
    assert "GRADED IN NONE" in readings["T3.3_bob"]

    # ...and the constant survives in the task file at the plan's value, so a
    # drift in it is still visible in a diff
    assert tasks.get(t3_core.TASK).constants["MIN_BOB_RMS_M"] == 0.005
    assert "no longer applied as a pass condition" in (
        tasks.get(t3_core.TASK).meta["constants_note"].lower())


def test_the_task_records_whether_the_shipped_robot_has_ever_walked():
    """§5c: the tier is not publishable until an oracle proves achievability.

    The record must say WHICH column, and it must be honest about whether the
    demonstration is reproducible from this tree.
    """
    block = tasks.get(t3_core.TASK).meta["container"]["authored_here"]
    assert block["demonstrated"].startswith(("YES", "NO", "NOT"))
    assert block["the_demonstration_is_not_yet_a_committed_oracle"]
    assert block["the_recipe_the_demonstration_used"]
    assert len(block["what_the_demonstration_found_about_THIS_ASSET"]) >= 3


def test_the_achievability_proof_is_reproducible_and_names_its_divergences():
    """§0.2's "no row, no result": a proof nobody can re-run is a claim.

    The first demonstration ran from a scratch harness outside this tree. The
    record must now say the oracle is committed, what re-running it measured,
    and -- because the recipe was prose and under-determined -- where the
    rebuild had to depart from it. A reproduction with the differences omitted
    is a reproduction nobody can check.
    """
    block = tasks.get(t3_core.TASK).meta["container"]["authored_here"]
    assert block["the_demonstration_is_not_yet_a_committed_oracle"].startswith(
        "CLOSED")
    assert "run_t3.py" in block[
        "the_demonstration_is_not_yet_a_committed_oracle"]

    got = block["reproduced_2026-08-02"]
    assert "PASS 5/5" in got
    assert "31.5416 m" in got and "34.73" in got, "both numbers, not just ours"
    assert "BIT-IDENTICAL" in got

    # every divergence is named, with the measurement that forced it
    diffs = block["the_divergences_from_the_scratch_recipe"]
    assert len(diffs) == 3
    for key in ("GROUND", "BODY SWAY", "ACTUATOR FORCE LIMITS"):
        assert any(d.startswith(key) for d in diffs), key

    # ...and the bob measurement that seconds the T3.3 repair is on the record
    assert "0.005 m BAR" in got or "RETIRED 0.005 m BAR" in got


def test_the_task_declares_its_pre_registered_expectation():
    """§5c.1: written before any cell runs, printed beside the outcome."""
    task = tasks.get(t3_core.TASK)
    exp = task.meta["expectation"]
    assert exp["omnisim"].startswith("achieved")
    assert "assembled" in exp["omnisim"]
    assert exp["reasoning"] and exp["pre_registered"]
    assert task.meta["scoring_class"] == "exploratory"
    assert task.repeats_default == 3


def test_the_task_meta_is_valid_json_and_names_its_grader():
    task = tasks.get(t3_core.TASK)
    assert task.meta["grader"] == "ladder.graders.t3"
    json.dumps(task.meta)          # round-trips
    assert task.rung == "T3"
    # the recorded window must fit ten metres at the tier's own speed floor
    assert task.standalone["duration_s"] >= (t3_core.NET_DISPLACEMENT_M
                                             / t3_core.MIN_MEAN_SPEED_MPS)
    assert set(task.surfaces) == set(task.meta["ground"]["names"])


# --- the structural guard, replicated ----------------------------------------
#
# ``ladder/graders/test_neutral_core.py`` walks the AST of every neutral core
# and refuses any simulator-specific token in CODE (docstrings are exempt on
# purpose: explaining why a boundary exists requires naming what is on the
# other side of it). T3's two cores are not in its NEUTRAL_MODULES tuple yet --
# adding them is a one-line change to a file this work is not permitted to
# edit -- so the guard is replicated here in the meantime. A guard that only
# covers the tiers that existed when it was written stops being a guard the
# moment the ladder grows, and that is exactly the failure this duplication
# exists to avoid.

_SHARED_GUARD = importlib.import_module("ladder.graders.test_neutral_core")
T3_NEUTRAL_MODULES = (t3_core, t3ev)


@pytest.mark.parametrize("mod", T3_NEUTRAL_MODULES)
def test_the_T3_cores_contain_no_simulator_specific_vocabulary(mod):
    for text in _SHARED_GUARD._code_strings_and_names(mod.__file__):
        low = text.lower()
        for token in _SHARED_GUARD.SIM_TOKENS:
            assert token not in low, (
                "%s mentions %r in code (not a docstring): %r"
                % (Path(mod.__file__).name, token, text))


@pytest.mark.parametrize("mod", T3_NEUTRAL_MODULES)
def test_the_T3_cores_import_no_simulator_module(mod):
    """Checked on the import graph, not the vocabulary.

    A core could import a per-simulator module without ever spelling its name
    in a string. Note that ``t3.py`` -- the task entry point -- is NOT a
    neutral core and does import the adapters, exactly as ``t1.py`` and
    ``t2.py`` do.
    """
    banned = ("adapters", "controllers", "headless", "launcher")
    tree = ast.parse(Path(mod.__file__).read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        for name in names:
            for b in banned:
                assert b not in (name or "").split("."), (
                    "%s imports %r, which is column-specific"
                    % (Path(mod.__file__).name, name))


def test_the_shared_guard_should_also_list_T3s_modules():
    """A reminder with a payload, not a landmine.

    Skips (rather than fails) while the shared tuple is missing T3, because
    the file that owns it is outside this work's scope and a hard failure
    would break somebody else's suite. The replicated guard above means the
    modules ARE checked either way; this test exists so the duplication gets
    removed rather than forgotten.
    """
    missing = [m.__name__ for m in T3_NEUTRAL_MODULES
               if m not in _SHARED_GUARD.NEUTRAL_MODULES]
    if missing:
        pytest.skip(
            "ladder/graders/test_neutral_core.py NEUTRAL_MODULES does not yet "
            "list %s -- add them (and import them) there, then delete the "
            "replicated guard in this file" % ", ".join(missing))


# --- resolving a base a column's importer did not name ---------------------


class _B:
    def __init__(self, name, mass_kg=None):
        self.name = name
        self.body_id = name
        self.mass_kg = mass_kg


class _Inv:
    source = "a test inventory"

    def __init__(self, bodies):
        self.bodies = bodies


class _Bundle:
    def __init__(self, bodies):
        self.t0 = _Inv(bodies)
        self.roster = _Inv([])


class _Ev:
    def __init__(self, bodies, robot_name="base_link", declared=None,
                 series_body=None):
        self.bundle = _Bundle(bodies)
        self.robot_name = robot_name
        self.robot_mass_kg_declared = declared
        self.base_pose = type("P", (), {"body": series_body})()


def test_the_declared_name_still_wins_when_it_is_present():
    ev = _Ev([_B("base_link", 10.8), _B("walker", 10.8)], declared=10.8)
    body, why = t3_core.select_base_body(ev)
    assert body.name == "base_link" and "carrying the name" in why


def test_a_base_the_importer_renamed_is_resolved_by_declared_mass():
    """The measured case: the root link folds into the robot node, so no body
    carries 'base_link' -- but one carries exactly the declared mass."""
    ev = _Ev([_B("walker", 10.8), _B("thigh_fl", 0.9), _B("shank_fl", 0.4)],
             declared=10.8)
    body, why = t3_core.select_base_body(ev)
    assert body.name == "walker"
    assert "resolved by MASS" in why and "10.8" in why


def test_the_mass_fallback_cannot_pick_a_limb():
    """The check is identity, not tolerance: nothing else is within 1%."""
    ev = _Ev([_B("walker", 6.0), _B("thigh_fl", 0.9)], declared=10.8)
    body, why = t3_core.select_base_body(ev)
    assert body is None and "no body matches the declared" in why


def test_two_bodies_at_the_declared_mass_are_refused_not_guessed():
    ev = _Ev([_B("a", 10.8), _B("b", 10.8)], declared=10.8)
    body, why = t3_core.select_base_body(ev)
    assert body is None and "AMBIGUOUS" in why


def test_a_column_that_declares_no_mass_gets_the_old_refusal():
    ev = _Ev([_B("walker", 10.8)], declared=None)
    body, why = t3_core.select_base_body(ev)
    assert body is None and "is in the run's inventory" in why


def test_the_naming_guard_accepts_the_mass_resolved_body_only():
    """The series may be about 'walker' when 'walker' IS the resolved base --
    and must still be refused when it is about something else."""
    ok = _Ev([_B("walker", 10.8)], declared=10.8, series_body="walker")
    assert t3_core.series_naming_problems(ok) == []

    bad = _Ev([_B("walker", 10.8)], declared=10.8, series_body="thigh_fl")
    assert t3_core.series_naming_problems(bad), (
        "a series about some other body must still be refused")
