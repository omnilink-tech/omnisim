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

"""Unit tests for T2's neutral core. **No simulator involved.**

Every fixture is synthetic numpy, so this file runs in a second on a machine
with no build, no GPU and no network -- which is the point of the neutral
core: a third party can re-derive every verdict in the ladder without our
stack.

    pytest tests/benchmarks/ladder/graders/ -q

Three tests carry more weight than the rest:

:func:`test_every_negative_fixture_reddens_exactly_its_own_assertion` is the
executable form of the red-evidence rule (``capability-ladder-plan.md``
§5c.2), exact in both directions.

:func:`test_the_hold_mechanism_is_recorded_and_never_graded` is the executable
form of §2 T2's ruling. It grades one run under all four mechanism values and
requires the assertions to come out identical -- because "we promise not to
grade it" is a comment, and this is a test.

:func:`test_the_frame_is_the_end_effectors_and_not_the_worlds` is what stops
T2.3 from being a much weaker check than it reads as: on the oracle, the same
held object passes in the end-effector's frame and FAILS without the rotation.
"""

from __future__ import annotations

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
from ladder.graders import fixtures_t2, t2_core  # noqa: E402
from ladder.graders import ladder_evidence as lev  # noqa: E402


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the positive control ----------------------------------------------------


def test_the_oracle_run_passes_every_assertion():
    v = t2_core.grade(fixtures_t2.oracle())
    assert v.outcome == PASS, v.summary()
    assert len(v.assertions) == 5
    assert v.progress == 4
    assert not v.vacuous
    a = _ids(v)
    assert a["T2.1"].measured["inside the container's bounding box"] is True
    assert a["T2.2"].measured[
        "longest continuous window clear by >= 0.05 m (s)"] > 10.0
    assert a["T2.3"].measured["a 10.0 s window inside both bounds exists, "
                              "with the object being carried in it"]
    assert a["T2.4"].measured["mass (kg)"] == pytest.approx(0.2)


def test_the_oracle_evidence_satisfies_the_ladders_contract():
    assert lev.check_t2_evidence(fixtures_t2.oracle()) == []


# --- the thresholds ----------------------------------------------------------


def test_thresholds_are_the_plans_numbers():
    """A drift in any of these is a tier change, not a refactor.

    capability-ladder-plan.md §5a: moving a threshold voids the pass and
    forces a version bump and a full re-run.
    """
    assert (t2_core.AT_REST_SPEED_MPS, t2_core.AT_REST_WINDOW_S) == (0.02, 1.0)
    assert (t2_core.LIFT_CLEARANCE_M, t2_core.LIFT_DWELL_S) == (0.05, 3.0)
    assert (t2_core.HOLD_S, t2_core.HOLD_RMS_M, t2_core.HOLD_MAX_M) == (
        10.0, 0.02, 0.04)
    assert (t2_core.MIN_MASS_KG, t2_core.MIN_GRAVITY_MPS2) == (0.0, 0.0)


def test_the_task_file_and_the_core_agree_on_every_constant():
    """The task's meta.json is what a reader opens; the core is what runs.

    Two copies of a threshold is one threshold and one lie waiting to happen,
    so they are compared rather than trusted -- in BOTH directions.
    """
    task = tasks.get(t2_core.TASK)
    for name, value in task.constants.items():
        assert hasattr(t2_core, name), "meta.json declares unknown %r" % name
        assert getattr(t2_core, name) == pytest.approx(value), name
    declared = set(task.constants)
    in_core = {n for n in t2_core.__all__
               if n.isupper() and isinstance(getattr(t2_core, n),
                                             (int, float))
               and not isinstance(getattr(t2_core, n), bool)}
    assert in_core - declared == set(), in_core - declared


# --- the red-evidence rule ---------------------------------------------------


@pytest.mark.parametrize("name", sorted(fixtures_t2.FIXTURE_ASSERTION_MAP))
def test_every_negative_fixture_reddens_exactly_its_own_assertion(name):
    """THE test the red-evidence rule exists for.

    Exact, in both directions: the declared assertion must be red, and no
    other one may be. §5c.2's own caveat is that "a null agent turning every
    assertion red does not satisfy this rule" -- so a fixture that reddens two
    assertions has stopped being evidence that either check discriminates.
    """
    _verdict, problems = fixtures_t2.check_fixture(name)
    assert not problems, "%s: %s" % (name, "; ".join(problems))


def test_every_assertion_has_a_negative_fixture():
    assert fixtures_t2.uncovered_assertions() == ()
    covered = {r["assertion"] for r in fixtures_t2.coverage_table()}
    assert covered == set(fixtures_t2.ASSERTIONS)


def test_the_coverage_table_names_a_fixture_for_each_assertion():
    text = fixtures_t2.render_coverage_table()
    for aid in fixtures_t2.ASSERTIONS:
        assert aid in text
    assert "every assertion has a negative fixture" in text
    # ...and the two fixtures that must PASS are visible as such, because a
    # reader scanning only the red rows would conclude the mechanism is graded
    assert "welded_object" in text


def test_every_assertion_declares_how_it_could_fail():
    v = t2_core.grade(fixtures_t2.oracle())
    for a in v.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_basis_map_separates_measured_physics_from_adapter_claims():
    v = t2_core.grade(fixtures_t2.oracle())
    basis = v.basis_summary()
    assert set(basis[CORE_PHYSICAL]) == {"T2.1", "T2.2", "T2.3"}
    assert set(basis[MIXED]) == {"T2.4", "T2.5"}
    assert v.as_dict()["basis"] == basis


# --- the mechanism: recorded, never graded -----------------------------------


@pytest.mark.parametrize("mechanism", lev.HOLD_MECHANISMS)
def test_the_hold_mechanism_is_recorded_and_never_graded(mechanism):
    """§2 T2's ruling, made executable rather than promised in a comment.

    Failing an attachment-based grasp would be writing the task around a
    technique this tree itself abandoned, so the same run must grade
    identically under every mechanism -- and the value must still reach the
    row, because §8 forbids quoting a T2 cell without it.
    """
    ev = fixtures_t2.oracle()
    ev.grip.mechanism = mechanism
    v = t2_core.grade(ev)
    baseline = t2_core.grade(fixtures_t2.oracle())
    assert [(a.id, a.ok) for a in v.assertions] == [
        (a.id, a.ok) for a in baseline.assertions]
    assert v.measurements["hold_mechanism"] == mechanism


def test_an_attachment_transfers_and_passes_with_the_mechanism_printed():
    v = t2_core.grade(fixtures_t2.welded_object())
    assert v.outcome == PASS
    assert v.measurements["hold_mechanism"] == "attachment"
    assert v.measurements["hold_mechanism_detail"][
        "an attachment binds the two bodies"] is True


def test_an_unrecognised_mechanism_is_recorded_as_unknown_not_dropped():
    ev = fixtures_t2.oracle()
    ev.grip.mechanism = "magnets, obviously"
    v = t2_core.grade(ev)
    assert v.measurements["hold_mechanism"] == "unknown"
    assert v.measurements["hold_mechanism_detail"]["declared_as"] == (
        "magnets, obviously")


def test_the_mechanism_reaches_the_row_even_when_nothing_else_does():
    """A cell that failed before any physics was read is still a T2 cell."""
    ev = fixtures_t2.oracle()
    ev.bundle.artifact = None
    v = t2_core.grade(ev)
    assert v.outcome == FAIL and len(v.failed) == 5
    assert v.measurements["hold_mechanism"] == "friction"


# --- T2.3's frame ------------------------------------------------------------


def test_the_frame_is_the_end_effectors_and_not_the_worlds():
    """The oracle holds the block rigidly while the wrist yaws 90 degrees.

    In the tool's frame the block is a fixed point; in the world frame it
    traces an 8 cm arc. If the core ever graded the world-frame offset, this
    same run would read as a 2.4 cm RMS slip -- so both numbers are computed
    and the row prints them side by side.
    """
    v = t2_core.grade(fixtures_t2.oracle())
    a = _ids(v)["T2.3"]
    assert a.ok is True
    frame_rms = a.measured["deviation in that window, RMS (m)"]
    world = a.measured["the same measured WITHOUT the carrier's rotation, "
                       "which is NOT the frame the tier states"]
    assert frame_rms <= t2_core.HOLD_RMS_M
    assert world["holds"] is False
    assert world["rms (m)"] > t2_core.HOLD_RMS_M
    assert world["rms (m)"] > 3.0 * frame_rms


def test_a_carrier_with_no_orientation_makes_the_frame_clause_vacuous():
    """Red because the tier's requirement was never measured, and it says so.

    The blocker for a cell in this state is ours. It must never read as "the
    grip slipped".
    """
    v = t2_core.grade(fixtures_t2.no_end_effector_orientation())
    a = _ids(v)["T2.3"]
    assert a.ok is False
    assert a.vacuous
    assert [f.clause for f in a.vacuous_clauses] == [
        "measured in the carrier's own frame"]
    assert "orientation" in a.detail
    assert "~vacuous" in v.summary()


def test_two_series_on_different_clocks_cannot_be_compared():
    """A skew wider than a sample is not one clock, and is refused."""
    ev = fixtures_t2.oracle()
    ev.end_effector.t = np.asarray(ev.end_effector.t) + 0.5
    v = t2_core.grade(ev)
    a = _ids(v)["T2.3"]
    assert a.ok is False
    assert [f.clause for f in a.vacuous_clauses] == [
        "measured in the carrier's own frame"]
    assert a.measured["clock skew between the two series (s)"] >= 0.5


# --- T2.3's carry gate (reading 4b, the 2026-08-02 repair) -------------------


def test_a_scene_at_rest_is_not_a_hold():
    """THE regression for the vacuous green this clause used to report.

    The run delivers the block honestly and then stops dead for 19 s with the
    arm parked 0.6 m away. The bounds are satisfied over that tail -- the row
    still says so, in as many words -- and the assertion is RED, because
    nothing in the tail was holding anything.
    """
    v = t2_core.grade(fixtures_t2.at_rest_never_held())
    a = _ids(v)["T2.3"]
    assert v.failed == ["T2.3"]
    assert a.ok is False
    assert not a.vacuous, "the instrument worked; the run did not hold"
    # ...and the pre-repair reading is printed rather than deleted, so a
    # reader can see exactly what used to be graded as a pass.
    assert a.measured["a window inside both bounds in which the object was at "
                      "rest on a surface throughout -- static geometry, NOT a "
                      "grip"] is True
    assert "not a grip" in a.detail


def test_the_repair_does_not_touch_a_run_that_really_held_it():
    """Both fixtures with a real grip stay exactly as they were."""
    for fn in (fixtures_t2.oracle, fixtures_t2.welded_object):
        a = _ids(t2_core.grade(fn()))["T2.3"]
        assert a.ok is True
        assert a.measured["samples in that window with the object off the "
                          "surface or moving"] > 0


def test_a_dragged_object_was_still_held_so_T2_3_stays_green():
    """The repair may not quietly absorb T2.2's job.

    ``never_lifted`` is gripped for the whole run and never leaves the
    surface. It must fail the LIFT and pass the HOLD -- the tier states the
    two separately, and a repair that made a drag fail both would be a tier
    change wearing a bug fix's clothes. This is why the carry gate is
    "off the surface OR moving" and not "off the surface".
    """
    v = t2_core.grade(fixtures_t2.never_lifted())
    assert v.failed == ["T2.2"]
    assert _ids(v)["T2.3"].ok is True


def test_a_static_hold_in_mid_air_is_a_hold():
    """And the other half of the disjunction: an object held perfectly still.

    "Hold it for ten seconds" invites exactly this answer -- lift the block,
    stop, wait. Nothing moves, so the MOVING half of the gate is silent and
    the AIRBORNE half is the whole reason this passes.
    """
    z = 0.725
    z_table = fixtures_t2.TABLE_TOP_Z + fixtures_t2.BLOCK_HALF
    z_bin = fixtures_t2.BIN_FLOOR_TOP_Z + fixtures_t2.BLOCK_HALF
    keys = ((0.0, (0.55, 0.0, z_table)), (3.0, (0.55, 0.0, z_table)),
            (5.0, (0.55, 0.0, z)), (16.0, (0.55, 0.0, z)),
            (18.0, (0.55, 0.0, z_bin)), (20.0, (-0.50, 0.0, z_bin)))
    ev = fixtures_t2._evidence(
        *fixtures_t2._tracks(keys=keys, yaw=(5.0, 5.01)))
    a = _ids(t2_core.grade(ev))["T2.3"]
    assert a.ok is True
    assert a.measured["samples in that window with the object off the "
                      "surface or moving"] > 0
    # ...and it is the CLEARANCE that carries it: over the whole ten seconds
    # in the air the object does not move at all, so a gate that only asked
    # "did it move" would call this static hold a scene at rest.
    t = np.asarray(ev.object_pose.t)
    inside = (t >= 6.0) & (t <= 15.0)
    speed = t2_core.sample_speed(t, ev.object_pose.xyz)
    assert float(speed[inside].max()) < t2_core.AT_REST_SPEED_MPS


def test_the_carry_gate_reports_its_witness_absent_with_nothing_underneath():
    """No bounded surface anywhere: "off the surface" is unanswerable.

    Only "it moved" is left, which cannot certify a hold that stood still, so
    the clause says it could not have caught one rather than implying it did.
    """
    ev = fixtures_t2.oracle()
    ev.support_surfaces = []
    a = _ids(t2_core.grade(ev))["T2.3"]
    assert "something was holding it" in [f.clause for f in a.vacuous_clauses]
    # the oracle's object does move, so the assertion itself still stands
    assert a.ok is True


def test_hold_windows_ungated_is_the_old_reading_and_is_reported_as_such():
    """The window arithmetic on its own is unchanged; only the gate is new."""
    t = np.arange(0.0, 12.0, 0.01)
    still = np.zeros((len(t), 3))
    ungated = t2_core.hold_windows(t, still, rms_tol=0.02, max_tol=0.04,
                                   min_s=10.0)
    assert ungated["holds"] and ungated["holds_ignoring_carry"]
    gated = t2_core.hold_windows(t, still, rms_tol=0.02, max_tol=0.04,
                                 min_s=10.0,
                                 carried=np.zeros(len(t), dtype=bool))
    assert not gated["holds"]
    assert gated["holds_ignoring_carry"], "the old reading stays visible"
    assert gated["carried_samples"] == 0


def test_a_sample_counts_as_carried_when_it_is_clear_or_when_it_moves():
    airborne = np.array([True, False, False, False])
    speed = np.array([0.0, 0.0, 0.5, 0.001])
    assert t2_core.carried_samples(airborne, speed).tolist() == [
        True, False, True, False]
    t = np.array([0.0, 0.1, 0.2])
    xyz = np.array([[0.0, 0, 0], [0.05, 0, 0], [0.05, 0, 0]])
    assert t2_core.sample_speed(t, xyz).tolist() == pytest.approx(
        [0.5, 0.5, 0.0])


def test_a_grasp_that_slips_within_the_bounds_still_counts_as_held():
    """The bound is a bound, not zero: 1 cm of creep is inside 2 cm RMS."""
    ev = fixtures_t2.oracle()
    t = np.asarray(ev.object_pose.t)
    ev.object_pose.xyz = np.asarray(ev.object_pose.xyz) + np.stack(
        [0.010 * np.sin(2.0 * np.pi * 0.5 * t), np.zeros(len(t)),
         np.zeros(len(t))], axis=1)
    v = t2_core.grade(ev)
    assert _ids(v)["T2.3"].ok is True


# --- T2.1's readings ---------------------------------------------------------


def test_an_object_still_falling_when_the_clock_stops_is_not_at_rest():
    v = t2_core.grade(fixtures_t2.released_above_the_rim())
    a = _ids(v)["T2.1"]
    assert a.ok is False
    assert a.measured["mean speed over the last 1.0 s (m/s)"] > (
        t2_core.AT_REST_SPEED_MPS)
    assert a.measured["seconds of samples in that window"] >= 1.0


def test_an_unattested_rim_is_graded_permissively_and_says_so():
    """The fallback can only make the clause easier, so it is published."""
    v = t2_core.grade(fixtures_t2.no_attested_rim())
    a = _ids(v)["T2.1"]
    assert a.ok is True
    assert a.vacuous
    assert "not attested" in [f.detail for f in a.vacuous_clauses][0]
    assert "bounding box" in a.measured["how the rim was decided"]


def test_without_the_objects_size_the_centre_is_graded_and_the_row_says_so():
    v = t2_core.grade(fixtures_t2.unbounded_object())
    a = _ids(v)["T2.1"]
    assert a.ok is True
    assert [f.clause for f in a.vacuous_clauses] == [
        "under the rim, not perched on it"]
    assert a.measured["its lowest point at the end (m)"] == pytest.approx(
        a.measured["the object's centre at the end (m)"][2])


def test_a_degenerate_container_box_is_not_a_containment_measurement():
    """Neither END of the transfer is measurable without a container box.

    "Ended inside it" and "started outside it" are the same containment test
    read at the two ends of the run, so a box that is not a box makes both
    vacuous. Reporting only one would imply the other had been checked.
    """
    ev = fixtures_t2.oracle()
    ev.container.aabb_max = (ev.container.aabb_min[0],
                             ev.container.aabb_min[1],
                             ev.container.aabb_min[2])
    v = t2_core.grade(ev)
    a = _ids(v)["T2.1"]
    assert a.ok is False
    assert sorted(f.clause for f in a.vacuous_clauses) == [
        "ended inside the container", "started outside the container"]


# --- T2.1's start clause (reading 2b, the 2026-08-02 repair) -----------------


def test_lifting_the_object_out_of_the_container_and_back_is_not_a_transfer():
    """THE regression for the transfer that never happened.

    Before T2.1 read where the object began, this artifact graded PASS 5/5: a
    flawless carry, out of the bin and back into it. The tier's own outcome
    sentence says the object STARTS ON A SURFACE, so the tier was not grading
    what it states.
    """
    v = t2_core.grade(fixtures_t2.started_in_the_container())
    assert v.failed == ["T2.1"]
    a = _ids(v)["T2.1"]
    assert a.ok is False
    assert not a.vacuous, "the instrument worked; the run was not a transfer"
    assert a.measured[
        "started outside the container's bounding box"] is False


def test_the_start_clause_reddens_ALONE_inside_T2_1():
    """It may not quietly absorb the three clauses T2.1 already had.

    A fixture that also lost "ends inside", "under the rim" or "at rest" would
    have stopped isolating the new clause -- and would have made the new
    clause look like it was doing work the old ones were already doing.
    """
    a = _ids(t2_core.grade(fixtures_t2.started_in_the_container()))["T2.1"]
    assert a.measured["inside the container's bounding box"] is True
    assert a.measured["its lowest point at the end (m)"] < a.measured[
        "the container rim (m)"]
    assert a.measured["mean speed over the last 1.0 s (m/s)"] < (
        t2_core.AT_REST_SPEED_MPS)


def test_the_start_is_read_from_the_same_series_as_the_end():
    """No new channel: both ends come off one body on one clock.

    A column that can answer "where did it end up" can answer "where did it
    begin" by construction, which is why this clause could be added without
    asking any column for anything new.
    """
    ev = fixtures_t2.oracle()
    a = _ids(t2_core.grade(ev))["T2.1"]
    assert a.measured["the object's centre at the start (m)"] == [
        round(float(x), 4) for x in np.asarray(ev.object_pose.xyz)[0]]
    assert a.measured["started outside the container's bounding box"] is True


def test_an_object_that_never_moved_at_all_fails_the_start_clause():
    """The degenerate case the clause exists for, stated as its own test."""
    ev = fixtures_t2.oracle()
    xyz = np.asarray(ev.object_pose.xyz)
    inside = np.tile(np.asarray([-0.50, 0.0,
                                 fixtures_t2.BIN_FLOOR_TOP_Z
                                 + fixtures_t2.BLOCK_HALF]), (len(xyz), 1))
    ev.object_pose.xyz = inside
    a = _ids(t2_core.grade(ev))["T2.1"]
    assert a.measured["started outside the container's bounding box"] is False
    assert a.measured["inside the container's bounding box"] is True
    assert a.ok is False


# --- T2.2's reading ----------------------------------------------------------


def test_clearance_is_measured_only_against_what_is_under_the_object():
    """Reading 3: a surface the object is nowhere near cannot support it.

    Graded literally against EVERY static surface, an object resting in a bin
    on the floor is below the table it came from and the tier could never be
    satisfied. Here the block ends 0.045 m up inside the bin while a 0.40 m
    table exists elsewhere in the scene, and the lift still measures.
    """
    v = t2_core.grade(fixtures_t2.oracle())
    a = _ids(v)["T2.2"]
    assert a.ok is True
    assert "table" in a.measured["surfaces beneath it at some point"]
    assert a.measured["samples with nothing beneath the object"] == 0


def test_with_no_static_surface_bounded_the_lift_clause_is_vacuous():
    ev = fixtures_t2.oracle()
    ev.support_surfaces = []
    v = t2_core.grade(ev)
    a = _ids(v)["T2.2"]
    assert a.ok is False
    assert "cleared the surface" in [f.clause for f in a.vacuous_clauses]
    assert a.measured["static bounded surfaces in the run"] == 0


def test_a_surface_the_adapter_cannot_call_static_is_not_a_datum():
    """An adapter that cannot say "static" is not saying it."""
    s = lev.SupportSurface(body="x", aabb_min=(0, 0, 0), aabb_max=(1, 1, 1),
                           static=None)
    assert not s.usable
    assert lev.SupportSurface(body="x", aabb_min=(0, 0, 0),
                              aabb_max=(1, 1, 1), static=True).usable


# --- T2.4's clauses ----------------------------------------------------------


def test_a_massless_prop_fails_even_though_it_reached_the_bin():
    """The anti-cheat, on kinematics identical to the oracle's."""
    v = t2_core.grade(fixtures_t2.massless_prop())
    assert v.failed == ["T2.4"]
    a = _ids(v)["T2.4"]
    assert a.measured["mass (kg)"] == 0.0
    assert _ids(t2_core.grade(fixtures_t2.oracle()))["T2.1"].ok is True


def test_an_unknown_mass_cannot_establish_a_real_body():
    """Red AND vacuous: unknown is not zero and it is not fine either."""
    ev = fixtures_t2.oracle()
    ev.object_physics.mass_kg = None
    v = t2_core.grade(ev)
    a = _ids(v)["T2.4"]
    assert a.ok is False
    assert "has mass" in [f.clause for f in a.vacuous_clauses]


def test_gravity_switched_off_is_a_world_where_nothing_has_to_be_held():
    ev = fixtures_t2.oracle()
    ev.world.gravity_mps2 = 0.0
    v = t2_core.grade(ev)
    assert v.failed == ["T2.4"]


def test_the_teleport_clause_is_vacuous_on_a_series_too_coarse_to_see_one():
    """A 0.5 m bound at a 1 Hz sample rate is not a teleport test."""
    ev = fixtures_t2.oracle()
    ev.object_pose.t = np.arange(ev.object_pose.n_samples) * 1.0
    ev.end_effector.t = ev.object_pose.t
    v = t2_core.grade(ev)
    a = _ids(v)["T2.4"]
    jump = [f for f in a.falsifiers if f.clause == "moved like a body"][0]
    assert not jump.present


# --- selecting the bodies ----------------------------------------------------


def test_the_declared_name_selects_the_object_from_the_inventory():
    ev = fixtures_t2.oracle()
    body, why = t2_core.select_role_body(ev, "block")
    assert body is not None and body.name == "block"
    assert "declares" in why


def test_a_series_about_the_wrong_body_is_refused_rather_than_graded():
    """Every number in such a verdict would look fine and be about a decoy."""
    ev = fixtures_t2.oracle()
    ev.object_pose.body = "some_other_body"
    v = t2_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert "the task names" in _ids(v)["T2.1"].detail


def test_a_series_that_does_not_say_which_body_it_is_about_is_still_graded():
    """"The adapter did not say" is not "the adapter said the wrong thing"."""
    ev = fixtures_t2.oracle()
    ev.object_pose.body = ""
    assert t2_core.grade(ev).outcome == PASS


# --- absent evidence ---------------------------------------------------------


def test_no_deliverable_scores_zero_of_five():
    ev = fixtures_t2.oracle()
    ev.bundle.artifact = None
    v = t2_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert v.progress == 0


def test_no_object_series_is_a_fail_with_the_reason_named():
    ev = fixtures_t2.oracle()
    ev.object_pose = lev.PoseSeries(
        body="block", error="the sample file was never written")
    v = t2_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert _ids(v)["T2.2"].detail == "the sample file was never written"


def test_a_missing_role_name_is_invalid_not_failed():
    """The names are task data. Losing one is a broken instrument."""
    ev = fixtures_t2.oracle()
    ev.roles.container_name = ""
    v = t2_core.grade(ev)
    assert v.outcome == INVALID
    assert v.outcome != FAIL
    assert any("container" in n for n in v.notes)


def test_missing_attribution_is_invalid_not_failed():
    v = t2_core.grade(fixtures_t2.missing_attribution())
    assert v.outcome == INVALID
    assert "T2.5" in v.failed
    assert any("unattributed" in n for n in v.notes)


# --- the measurement helpers -------------------------------------------------


def test_mean_speed_covers_at_least_the_requested_window():
    """Off by a fraction of a sample and the clause's witness never fires."""
    t = np.arange(0.0, 5.0, 0.016)
    xyz = np.stack([t * 0.5, np.zeros(len(t)), np.zeros(len(t))], axis=1)
    speed, intervals, covered = t2_core.mean_speed_over(t, xyz, 1.0)
    assert speed == pytest.approx(0.5, abs=1e-9)
    assert covered >= 1.0 and intervals > 1


def test_mean_speed_is_none_when_there_is_nothing_to_average():
    speed, intervals, covered = t2_core.mean_speed_over(
        [0.0], [[0.0, 0.0, 0.0]], 1.0)
    assert speed is None and intervals == 0 and covered == 0.0


def test_longest_true_window_reports_a_single_sample_run_as_a_run():
    t = np.arange(6) * 1.0
    secs, i0, i1 = t2_core.longest_true_window(
        t, [False, True, False, True, True, False])
    assert (secs, i0, i1) == (1.0, 3, 4)
    secs, i0, i1 = t2_core.longest_true_window(t, [False] * 6)
    assert (secs, i0, i1) == (0.0, None, None)


def test_lift_clearance_takes_the_smallest_gap_to_what_is_underneath():
    surfaces = [lev.SupportSurface(body="floor", aabb_min=(-5, -5, -1),
                                   aabb_max=(5, 5, 0.0), static=True),
                lev.SupportSurface(body="table", aabb_min=(0.2, -0.4, 0.0),
                                   aabb_max=(0.8, 0.4, 0.4), static=True)]
    xy = np.array([[0.5, 0.0], [0.0, 0.0]])
    lowest = np.array([0.6, 0.6])
    clear, defined, touched = t2_core.lift_clearance(xy, lowest, surfaces)
    assert defined.all()
    assert clear[0] == pytest.approx(0.2)     # over the table
    assert clear[1] == pytest.approx(0.6)     # only the floor is underneath
    assert set(touched) == {"floor", "table"}


def test_lift_clearance_marks_samples_with_nothing_underneath_undefined():
    surfaces = [lev.SupportSurface(body="table", aabb_min=(0.2, -0.4, 0.0),
                                   aabb_max=(0.8, 0.4, 0.4), static=True)]
    clear, defined, _touched = t2_core.lift_clearance(
        np.array([[0.5, 0.0], [9.0, 0.0]]), np.array([0.6, 0.6]), surfaces)
    assert defined.tolist() == [True, False]
    assert np.isnan(clear[1])


def test_frame_relative_undoes_the_carriers_rotation():
    rot = np.array([[[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]])
    obj = np.array([[1.0, 2.0, 3.0]])
    holder = np.array([[1.0, 0.0, 3.0]])
    # the object is 2 m along world +y, which is the carrier's own +x
    assert t2_core.frame_relative(obj, holder, rot)[0] == pytest.approx(
        [2.0, 0.0, 0.0])


def test_hold_windows_finds_a_window_only_when_one_is_long_enough():
    t = np.arange(0.0, 12.0, 0.01)
    steady = np.zeros((len(t), 3))
    res = t2_core.hold_windows(t, steady, rms_tol=0.02, max_tol=0.04,
                               min_s=10.0)
    assert res["holds"] and res["longest_s"] >= 10.0
    short = t2_core.hold_windows(t[:500], steady[:500], rms_tol=0.02,
                                 max_tol=0.04, min_s=10.0)
    assert not short["holds"] and short["windows_tested"] == 0


def test_hold_windows_reports_the_tightest_window_even_when_it_fails():
    t = np.arange(0.0, 12.0, 0.01)
    r = np.stack([0.5 * np.sin(t), np.zeros(len(t)), np.zeros(len(t))],
                 axis=1)
    res = t2_core.hold_windows(t, r, rms_tol=0.02, max_tol=0.04, min_s=10.0)
    assert not res["holds"]
    assert res["rms_m"] > 0.02 and res["max_m"] is not None


# --- the ladder's own evidence channels --------------------------------------


def test_the_evidence_contract_names_the_usual_adapter_mistakes():
    ev = fixtures_t2.oracle()
    ev.roles.source = ""
    ev.end_effector.rot = None
    ev.object_physics.mass_kg = None
    ev.world.gravity_mps2 = None
    ev.support_surfaces = []
    ev.object_aabb = None
    problems = lev.check_t2_evidence(ev)
    assert any("source citation" in p for p in problems)
    assert any("no orientation" in p for p in problems)
    assert any("object mass" in p for p in problems)
    assert any("gravity" in p for p in problems)
    assert any("support surfaces" in p for p in problems)
    assert any("object size" in p for p in problems)


def test_a_pose_series_derived_from_the_frozen_bundle_has_no_orientation():
    """The gap, asserted rather than described.

    The published motion contract carries position and velocity. A column
    with no ladder-side sampler therefore cannot answer T2.3 at all, and the
    blocker for that cell is ours.
    """
    from agentbench.graders.evidence import Trajectory
    bundle = fixtures_t2.oracle().bundle
    bundle.trajectory = Trajectory(
        body_ids=["#21"], t=np.arange(3) * 0.016,
        xyz=np.zeros((1, 3, 3)), dt_s=0.016, source="frozen")
    bundle.roster.bodies = [b for b in bundle.t0.bodies if b.name == "block"]
    series = lev.pose_series_from_bundle(bundle, name="block")
    assert series.usable
    assert not series.has_orientation


def test_body_physics_from_the_frozen_bundle_answers_dynamic_and_not_mass():
    bundle = fixtures_t2.oracle().bundle
    phys = lev.body_physics_from_bundle(bundle, name="block")
    assert phys.dynamic is True
    assert phys.mass_kg is None
    assert "no mass field" in phys.error


def test_container_from_the_frozen_bundle_answers_the_box_and_not_the_rim():
    bundle = fixtures_t2.oracle().bundle
    c = lev.container_from_bundle(bundle, "bin")
    assert c.usable and not c.rim_attested
    assert c.rim_height == pytest.approx(fixtures_t2.BIN_AABB[1][2])
    assert "permissive" in c.rim_source


def test_support_surfaces_are_the_static_bounded_non_robot_bodies():
    bundle = fixtures_t2.oracle().bundle
    names = {s.body for s in lev.support_surfaces_from_bundle(bundle)}
    # not the arm (a robot) and not the block (dynamic)
    assert names == {"ground", "table", "bin"}


def test_the_pose_series_reports_its_own_sample_rate_and_skew():
    ev = fixtures_t2.oracle()
    assert ev.object_pose.dt_s == pytest.approx(fixtures_t2.DT)
    idx, skew = ev.end_effector.align_to(ev.object_pose.t)
    assert skew == pytest.approx(0.0, abs=1e-9)
    assert idx[0] == 0


# --- the shim over agentbench's adapters -------------------------------------


class _StubColumn:
    """A column that produces a bundle and none of T2's channels."""

    def __init__(self, bundle):
        self.bundle = bundle

    def build_bundle(self, *_a, **_kw):
        return self.bundle


def test_a_column_with_no_t2_channels_falls_back_and_names_every_gap(
        monkeypatch):
    """The whole point of the gap machinery: the blocker must land on us."""
    bundle = fixtures_t2.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = adapters.build_t2_evidence(t2_core.TASK, sim="stub")
    gaps = adapters.t2_unanswered_channels(ev)
    assert set(gaps) >= {"object_pose", "end_effector_pose", "object_mass",
                         "gravity", "grip"}
    # ...and the three the frozen contract CAN answer are not in the gap list
    assert "container_geometry" not in gaps
    assert "support_surfaces" not in gaps
    assert ev.object_aabb is not None
    assert any("scaffolding_defect_ours" in n for n in ev.notes)


def test_a_column_that_supplies_channels_is_used_unchanged(monkeypatch):
    ref = fixtures_t2.oracle()
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(ref.bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = adapters.build_t2_evidence(
        t2_core.TASK, sim="stub",
        channels={"object_pose": ref.object_pose,
                  "end_effector": ref.end_effector,
                  "container": ref.container, "grip": ref.grip,
                  "object_physics": ref.object_physics, "world": ref.world,
                  "support_surfaces": ref.support_surfaces,
                  "object_aabb": ref.object_aabb})
    assert adapters.t2_unanswered_channels(ev) == {}
    assert t2_core.grade(ev).outcome == PASS


def test_the_roles_come_from_the_task_file_and_carry_its_citation(
        monkeypatch):
    bundle = fixtures_t2.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = adapters.build_t2_evidence(t2_core.TASK, sim="stub")
    assert ev.roles.object_name == "block"
    assert ev.roles.end_effector_name == "tool"
    assert ev.roles.container_name == "bin"
    assert "meta.json" in ev.roles.source


def test_the_ladder_declares_what_a_new_column_must_add_for_T2():
    keys = {k for k, _why in adapters.LADDER_REQUIRED_EVIDENCE_T2}
    assert keys == {"roles", "object_pose", "end_effector_pose",
                    "container_geometry", "object_mass", "grip"}
    for _k, why in adapters.LADDER_REQUIRED_EVIDENCE_T2:
        assert why
    # T1's published list is not silently grown into a union: a column can be
    # ready for one rung and not the next, and one merged list would hide it.
    assert adapters.LADDER_REQUIRED_EVIDENCE is (
        adapters.LADDER_REQUIRED_EVIDENCE_T1)


def test_every_channel_gap_names_the_assertions_it_breaks():
    for channel, assertions in adapters.T2_CHANNEL_ASSERTIONS.items():
        for aid in assertions:
            assert aid in fixtures_t2.ASSERTIONS, (channel, aid)
    # the grip channel breaks NOTHING, because the mechanism is never graded
    assert adapters.T2_CHANNEL_ASSERTIONS["grip"] == ()


# --- the task assets ---------------------------------------------------------


def test_the_prompt_is_one_sentence():
    task = tasks.get(t2_core.TASK)
    assert task.prompt.count(".") == 1
    assert task.prompt.endswith(".")
    assert "\n" not in task.prompt


# §5a's outcome-not-feature rule: "a tier statement may not name a file
# format, an endpoint, a node type, a solver, a training method, a
# checkpoint, or any product's proper noun."
_FORBIDDEN_IN_PROMPT = (
    "urdf", "wbt", "sdf", "usd", "mjcf", "xacro", "xml", "json", "yaml",
    "omnisim", "webots", "gazebo", "isaac", "mujoco", "genesis", "newton",
    "ode", "solver", "supervisor", "controller", "proto", "node", "endpoint",
    "http", "api", "checkpoint", "policy", "reinforcement", "import",
    "friction", "suction", "weld", "attach", "contact")


def test_the_prompt_names_no_format_no_endpoint_and_no_mechanism():
    """The last few tokens are T2's own: the mechanism is not the task."""
    task = tasks.get(t2_core.TASK)
    low = task.prompt.lower()
    for token in _FORBIDDEN_IN_PROMPT:
        assert not re.search(r"\b%s\b" % re.escape(token), low), token


def test_the_prompt_states_the_outcome_and_not_the_method():
    task = tasks.get(t2_core.TASK)
    low = task.prompt.lower()
    for word in ("block", "bin", "ten seconds"):
        assert word in low


def test_the_container_ships_the_descriptions_and_nothing_else():
    task = tasks.get(t2_core.TASK)
    on_disk = sorted(p.relative_to(task.container_dir).as_posix()
                     for p in task.container_dir.rglob("*") if p.is_file())
    declared = sorted(
        (task.container_dir / n).relative_to(task.container_dir).as_posix()
        for n in task.meta["container"]["files"])
    assert on_disk == declared
    assert not any(p.suffix in (".wbt", ".sdf", ".usd", ".xml", ".py", ".sh")
                   for p in task.container_files), \
        "the container ships a scene, a script or a converted model"


def test_the_container_ships_three_descriptions_and_no_control_code():
    task = tasks.get(t2_core.TASK)
    urdfs = [p for p in task.container_files if p.suffix == ".urdf"]
    assert len(urdfs) == 3
    text = "\n".join(p.read_text(encoding="utf-8") for p in urdfs).lower()
    for word in ("trajectory", "waypoint", "grasp_pose", "controller"):
        assert word not in text, word


@pytest.mark.parametrize("name", ["bench_arm/urdf/bench_arm.urdf",
                                  "props/urdf/block.urdf",
                                  "props/urdf/bin.urdf"])
def test_every_shipped_description_parses_and_references_no_outside_file(name):
    """A malformed description would fail every column for our reason.

    And a reference to a file we did not ship would fail whichever column
    resolves paths least forgivingly, which is the T1 open question this
    container exists to avoid repeating.
    """
    task = tasks.get(t2_core.TASK)
    root = ET.parse(task.container_dir / name).getroot()
    assert root.tag == "robot" and root.get("name")
    assert root.findall(".//mesh") == []
    # Attribute values only: a comment may say the word "package://" while
    # explaining why there are none, and a text search cannot tell the two
    # apart.
    for el in root.iter():
        for key, value in el.attrib.items():
            assert "://" not in value, (name, el.tag, key, value)
            assert not value.startswith(("/", ".", "~")), (name, key, value)
    for link in root.findall("link"):
        assert link.find("inertial") is not None, link.get("name")
        mass = link.find("inertial/mass")
        assert mass is not None and float(mass.get("value")) > 0.0


def test_the_arm_can_actually_reach_and_actually_close_on_the_block():
    """The one geometric claim the container makes about itself, checked.

    It is NOT a claim that the task is achievable -- see meta.json
    container.authored_here. It is the minimum: an arm that cannot reach the
    floor, or whose jaws cannot close on a 0.05 m cube, would make the tier
    unachievable for a reason that has nothing to do with any simulator.
    """
    task = tasks.get(t2_core.TASK)
    root = ET.parse(task.container_dir
                    / "bench_arm/urdf/bench_arm.urdf").getroot()
    reach = 0.0
    for joint in root.findall("joint"):
        xyz = joint.find("origin").get("xyz").split()
        reach += abs(float(xyz[2]))
    assert reach > 0.60, reach
    fingers = [j for j in root.findall("joint")
               if j.get("type") == "prismatic"]
    assert len(fingers) == 2
    for j in fingers:
        offset = abs(float(j.find("origin").get("xyz").split()[1]))
        travel = float(j.find("limit").get("upper"))
        # inner face at full travel must be inside the block's half-width
        assert offset - travel - 0.006 < 0.025 < offset - 0.006


def test_the_task_declares_which_body_is_which_and_why_it_is_task_data():
    task = tasks.get(t2_core.TASK)
    assert task.roles["object"] == "block"
    assert task.roles["end_effector"] == "tool"
    assert task.roles["container"] == "bin"
    assert "no adapter may supply it" in task.roles["why_not_in_the_container"]
    assert "meta.json" in tasks.roles_citation(task)


def test_the_task_declares_the_mechanism_is_recorded_and_not_graded():
    task = tasks.get(t2_core.TASK)
    block = task.meta["hold_mechanism"]
    assert block["graded"] is False
    assert set(block["values"]) == set(lev.HOLD_MECHANISMS)
    assert block["why_not"] and block["recorded_where"]


def test_the_teleport_clause_is_recorded_as_ratified_not_as_derived():
    """Reading 5, after the 2026-08-02 ruling.

    The clause used to be this grader going beyond the plan's table, flagged
    as such and awaiting the freeze. It is now IN the tier text, and the task
    file a reviewer opens must say so rather than still asking the question.
    """
    task = tasks.get(t2_core.TASK)
    note = task.meta["grading_readings"]["T2.4_derived_clause"]
    assert "STATED AND DECIDED 2026-08-02" in note
    assert "capability-ladder-plan.md section 2 T2's own T2.4 row" in note
    assert "MAX_STEP_JUMP_M" in note and "teleported_object" in note
    # ...and it still declares no number of its own: T1.3's constant, re-used
    assert t2_core.MAX_STEP_JUMP_M == 0.5


def test_the_task_records_the_start_clause_as_decided_with_its_fixture():
    """Reading 2b: the decision, its fixture and its limit, in the task."""
    readings = tasks.get(t2_core.TASK).meta["grading_readings"]
    assert "T2_OPEN_QUESTION_no_clause_reads_where_the_object_STARTED" not in \
        readings, "the clause landed; it is no longer an open question"
    note = readings["T2.1_started_outside_DECIDED_2026-08-02"]
    assert "DECIDED AND FIXED before the freeze" in note
    assert "started_in_the_container" in note
    assert "started_in_the_container" in fixtures_t2.FIXTURE_ASSERTION_MAP
    # the thing it deliberately does NOT assert stays on the record
    assert "on a surface" in note


def test_the_tall_crate_hole_is_recorded_as_a_stated_limit_not_a_repair():
    """Decision 6: published as a limit, with its containment, not fixed."""
    readings = tasks.get(t2_core.TASK).meta["grading_readings"]
    assert "T2.2_OPEN_QUESTION_resting_on_something_tall" not in readings
    note = readings["T2.2_STATED_LIMIT_resting_on_something_tall"]
    assert "STATED LIMIT AT THE FREEZE" in note
    assert "DELIBERATELY NOT FIXED" in note
    # ...and the containment that makes it publishable is stated with it
    assert "T2.1" in note
    # the carry gate's cross-reference must not have been left dangling
    assert "T2.2_STATED_LIMIT_resting_on_something_tall" in readings[
        "T2.3_carry_gate"]


def test_the_task_records_the_T2_3_repair_as_decided_not_as_a_question():
    """The vacuous green is fixed in code; meta.json must say so, and how.

    The reading the code implements and the reading a reviewer opens are two
    copies of one decision, which is one decision and one lie waiting to
    happen -- so the constants the gate is built from are checked against the
    core rather than trusted, in both directions.
    """
    readings = tasks.get(t2_core.TASK).meta["grading_readings"]
    assert "T2.3_OPEN_QUESTION_FOR_THE_FREEZE" not in readings, \
        "the repair landed; it is no longer an open question"
    gate = readings["T2.3_carry_gate"]
    assert "DECIDED 2026-08-02" in gate
    assert "LIFT_CLEARANCE_M" in gate and "AT_REST_SPEED_MPS" in gate
    assert "NO THRESHOLD MOVED" in gate
    assert "at_rest_never_held" in gate
    assert "at_rest_never_held" in fixtures_t2.FIXTURE_ASSERTION_MAP
    # the defect itself stays on the record next to its repair
    assert "4.0e-08" in readings["T2.3_REPAIRED_2026-08-02"]


def test_every_pre_freeze_audit_finding_is_recorded_with_its_disposition():
    """§5c: a reviewer must see the gap AND what was decided about it.

    Each of these was demonstrated on a deliberately wrong artifact before the
    freeze. Three dispositions are legitimate -- fixed, ratified as a stated
    limit, or resolved by relabelling -- and "still open" is not one of them
    any more, because the owner has ruled on all three.
    """
    readings = tasks.get(t2_core.TASK).meta["grading_readings"]
    for key in ("T2.2_STATED_LIMIT_resting_on_something_tall",
                "T2.1_started_outside_DECIDED_2026-08-02",
                "T1_SIBLING_AUDIT_2026-08-02"):
        note = readings[key]
        assert "demonstrated" in note.lower()
        assert any(word in note for word in ("DECIDED", "STATED LIMIT",
                                             "RESOLVED"))
    # no entry may still be posing one of these as an unanswered question
    assert not [k for k in readings if "OPEN_QUESTION" in k]


def test_the_task_declares_its_pre_registered_expectation():
    """§5c.1: written before any cell runs, printed beside the outcome."""
    task = tasks.get(t2_core.TASK)
    exp = task.meta["expectation"]
    assert exp["omnisim"].startswith("not_achieved")
    assert exp["reasoning"] and exp["pre_registered"]
    assert task.meta["scoring_class"] == "exploratory"
    assert task.repeats_default == 3


def test_the_task_meta_is_valid_json_and_names_its_grader():
    task = tasks.get(t2_core.TASK)
    assert task.meta["grader"] == "ladder.graders.t2"
    json.dumps(task.meta)          # round-trips
    assert task.rung == "T2"
    assert task.standalone["duration_s"] >= (t2_core.HOLD_S
                                             + t2_core.LIFT_DWELL_S)
