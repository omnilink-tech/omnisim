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

"""Unit tests for the sim-agnostic grader cores. **No simulator involved.**

Every fixture here is a synthetic evidence bundle built from numpy arrays, so
these tests run in milliseconds on a machine with no OmniSim build, no GPU and
no network -- which is the point of the neutral core: the graders can be tested,
argued with, and re-run by a third party without our stack.

    pytest tests/benchmarks/agentbench/graders/test_neutral_core.py -q

Two of these tests exist because of specific incidents:

  * ``test_vacuity_fires_when_contact_evidence_is_absent`` and
    ``test_vacuity_catches_the_id_id_self_pair_bug`` -- A1.3's robot-robot
    contact clause could not fail for weeks (the node-id lookup returned the
    queried solid's own id, so every pair was ``(id, id)`` and the filter
    matched nothing, on every world, forever). It read as a pass.
  * ``test_cores_contain_no_simulator_specific_vocabulary`` -- the structural
    guard that keeps the split honest as the suite grows.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench import adapters                                  # noqa: E402
from agentbench.graders import a1_core, b3_core, c2_core          # noqa: E402
from agentbench.graders.evidence import (                         # noqa: E402
    Body, BodyInventory, ContactObservation, ContactPair, EngineAttribution,
    EvidenceBundle, IdentityRule, ProcessFacts, Trajectory)
from agentbench.graders.verdict import (                          # noqa: E402
    ADAPTER_ATTESTED, CORE_PHYSICAL, FAIL, INVALID, PASS)

DT = 0.032


# --- synthetic fixtures -----------------------------------------------------


def _robot_body(i, *, x, y=0.0, behaviour="husky_random", joints=4,
                dynamic=True, half=0.57, robot_class=True):
    return Body(body_id="#%d" % (100 + i), name="husky_%d" % i, kind="Robot",
                position=(x, y, 0.2),
                aabb_min=(x - half, y - 0.5, 0.05),
                aabb_max=(x + half, y + 0.5, 0.45),
                n_joints=joints, behaviour=behaviour,
                behaviour_declared=behaviour, dynamic=dynamic,
                robot_class=robot_class,
                identity_evidence="synthetic fixture")


def _track(bearing_rad, *, length_m, n, z=0.2, wobble=0.0):
    """A straight (or gently wobbling) constant-speed track, metres."""
    s = np.linspace(0.0, length_m, n)
    x = s * math.cos(bearing_rad)
    y = s * math.sin(bearing_rad)
    if wobble:
        y = y + wobble * np.sin(np.linspace(0.0, 6 * math.pi, n))
    return np.stack([x, y, np.full(n, z)], axis=1)


def a1_bundle(*, n_robots=10, bearings=None, length_m=6.0, window_s=30.0,
              behaviour="husky_random", starts=None, contacts=None,
              attribution="ok", declared=None, spacing=3.0, z_offsets=None,
              overlap=False, artifact="/synthetic/husky_swarm_10.wbt"):
    """A full A1 evidence bundle, from numpy. Defaults describe a PASSing run."""
    n = int(round(window_s / DT)) + 1
    t = np.arange(n) * DT
    if bearings is None:                       # ten well-spread bearings
        bearings = [2 * math.pi * i / n_robots for i in range(n_robots)]
    half = 1.6 if overlap else 0.57            # overlap => AABBs interpenetrate
    bodies = [_robot_body(i, x=spacing * i, behaviour=behaviour, half=half)
              for i in range(n_robots)]
    xyz = np.stack([_track(bearings[i], length_m=length_m, n=n,
                           z=0.2 + (0.0 if z_offsets is None
                                    else z_offsets[i]))
                    + np.array([spacing * i, 0.0, 0.0])
                    for i in range(n_robots)], axis=0)
    if starts is None:
        starts = {behaviour: n_robots}
    if contacts is None:
        contacts = ContactObservation(
            pairs=[], steps=10, supported=True, total_observed=40,
            distinct_named=12, source="synthetic contact scan")
    attrib = None
    if attribution == "ok":
        attrib = EngineAttribution(backend="synthetic", solver="synthetic-pgs",
                                   source="a synthetic fixture, not a run")
    return EvidenceBundle(
        task=a1_core.TASK, sim="synthetic", adapter="tests",
        artifact=artifact,
        identity=IdentityRule(
            label="Husky", requirement="the robot the task named",
            scene_rule="synthetic: robot_class is set on the fixture",
            declaration_rule="synthetic: declared_count is given",
            declared_count=n_robots if declared is None else declared),
        roster=BodyInventory(bodies=bodies, frozen=False,
                             source="synthetic live query"),
        t0=BodyInventory(bodies=bodies, frozen=True, t_s=0.0,
                         source="synthetic frozen scan"),
        contacts=contacts,
        trajectory=Trajectory(body_ids=[b.body_id for b in bodies], t=t,
                              xyz=xyz, dt_s=DT, recorded_s=float(t[-1]),
                              complete=True, source="synthetic recorder"),
        process=ProcessFacts(exit_code=0, timed_out=False, error_lines=[],
                             log_available=True, behaviour_starts=starts,
                             start_source="synthetic process table",
                             driver_completed=True, reached_finalize=True,
                             finalize_evidence="driver completed 30s of sim"),
        attribution=attrib, live_load_ok=True)


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- A1: the physical core --------------------------------------------------


def test_a1_core_passes_a_clean_synthetic_swarm():
    v = a1_bundle()
    r = a1_core.grade(v)
    assert r.outcome == PASS, r.summary()
    assert len(r.assertions) == 10
    assert r.progress == 4
    a = _ids(r)
    assert a["A1.5"].measured["min net displacement (m)"] == pytest.approx(6.0)
    assert a["A1.6"].measured["min path length (m)"] == pytest.approx(6.0)
    assert a["A1.8"].measured["circular resultant R"] < a1_core.MAX_R


def test_a1_core_fails_only_the_randomness_gate_on_a_parade():
    """Ten correct, moving robots on ONE heading: R = 1.0, nothing else wrong."""
    r = a1_core.grade(a1_bundle(bearings=[0.3] * 10))
    assert r.outcome == FAIL
    assert r.failed == ["A1.8"]
    assert _ids(r)["A1.8"].measured["circular resultant R"] == pytest.approx(1.0)


def test_a1_core_reports_randomness_unmeasurable_when_nothing_moved():
    """A static scene must not score R = 1.0 from atan2(0, 0)."""
    r = a1_core.grade(a1_bundle(length_m=0.0, behaviour="<none>", starts={}))
    assert r.outcome == FAIL
    a = _ids(r)
    assert a["A1.8"].measured["circular resultant R"] is None
    assert a["A1.8"].measured[
        "robots with an undefined bearing (net displacement < 0.05 m)"] == 10
    assert set(r.failed) >= {"A1.5", "A1.6", "A1.8"}


def test_a1_core_thresholds_are_the_spec_numbers():
    """A drift in any of these is a SPEC violation, not a refactor."""
    assert (a1_core.N_ROBOTS, a1_core.SETTLE_S, a1_core.WINDOW_S) == (
        10, 1.0, 30.0)
    assert (a1_core.MIN_NET_DISP_M, a1_core.MIN_PATH_M) == (2.0, 4.0)
    assert (a1_core.MAX_ABS_DZ_M, a1_core.MIN_Z_M) == (0.30, -0.10)
    assert (a1_core.MAX_R, a1_core.MIN_AABB_CLEARANCE_M) == (0.70, 0.05)


def test_a1_core_displacement_floor_is_the_min_not_the_mean():
    """Nine robots at 6 m and one at 1 m must FAIL: A1.5 is a min over robots."""
    b = a1_bundle()
    b.trajectory.xyz[3, :, :2] = b.trajectory.xyz[3, 0, :2]   # robot 3 frozen
    r = a1_core.grade(b)
    assert "A1.5" in r.failed and "A1.6" in r.failed
    assert _ids(r)["A1.5"].measured["min net displacement (m)"] == 0.0


def test_a1_core_fails_a1_3_on_interpenetrating_aabbs():
    r = a1_core.grade(a1_bundle(overlap=True, spacing=1.0))
    assert "A1.3" in r.failed
    m = _ids(r)["A1.3"].measured
    assert m["min pairwise AABB clearance (m)"] < a1_core.MIN_AABB_CLEARANCE_M


def test_a1_core_fails_a1_3_on_a_reported_robot_robot_contact():
    contacts = ContactObservation(
        pairs=[ContactPair(a="#101", b="#102", a_robot=True, b_robot=True,
                           point=(1.0, 0.0, 0.1), step=3)],
        steps=10, supported=True, total_observed=41, distinct_named=13,
        source="synthetic contact scan")
    r = a1_core.grade(a1_bundle(contacts=contacts))
    assert "A1.3" in r.failed
    assert _ids(r)["A1.3"].measured[
        "robot-robot contacts in first 10 steps"] == 1


def test_a1_core_fails_a1_7_when_a_robot_sinks():
    b = a1_bundle()
    b.trajectory.xyz[7, -1, 2] = -0.4
    r = a1_core.grade(b)
    assert "A1.7" in r.failed
    assert _ids(r)["A1.7"].measured["min z over all samples (m)"] == -0.4


def test_a1_core_keeps_both_halves_of_all_ten_are_controlled():
    """Declared behaviours but no processes started is still a failure."""
    r = a1_core.grade(a1_bundle(starts={"husky_random": 4}))
    assert "A1.4" in r.failed
    m = _ids(r)["A1.4"].measured
    assert m["bodies with a behaviour field set"] == 10
    assert m["behaviour processes started"] == {"husky_random": 4}


def test_a1_core_missing_attribution_is_invalid_not_failed():
    """SPEC A1.10: an unattributed physics result is not a result."""
    r = a1_core.grade(a1_bundle(attribution=None))
    assert r.outcome == INVALID
    assert r.outcome != FAIL
    assert _ids(r)["A1.10"].basis == ADAPTER_ATTESTED
    assert any("unattributed" in n or "attribution" in n for n in r.notes)


def test_a1_core_no_artifact_scores_zero_of_ten():
    r = a1_core.grade(a1_bundle(artifact=None))
    assert r.outcome == FAIL
    assert len(r.failed) == 10
    assert r.progress == 0


def test_a1_core_no_motion_samples_keeps_the_structural_verdict():
    """A world that loaded but produced no motion data.

    The four structural assertions are still graded (the roster was read), the
    seven motion ones fail with the reason named, and the progress ordinal
    distinguishes "loaded" from "invalid" -- which is the whole point of §3.2.
    """
    b = a1_bundle()
    b.trajectory = None
    b.motion_error = "no recorder samples"
    r = a1_core.grade(b)
    assert r.outcome == FAIL
    assert set(r.failed) == {"A1.3", "A1.5", "A1.6", "A1.7", "A1.8", "A1.9",
                             "A1.10"}
    assert _ids(r)["A1.5"].detail == "no recorder samples"
    assert r.progress == 2                     # artifact_loads
    b.live_load_ok = False
    assert a1_core.grade(b).progress == 1      # artifact_invalid


def test_a1_core_identity_count_comes_from_the_adapter():
    """Nine declarations for ten bodies: the corroboration floor still bites."""
    r = a1_core.grade(a1_bundle(declared=9))
    assert "A1.1" in r.failed
    assert _ids(r)["A1.1"].measured[
        "declared robot instances in the artifact"] == 9


def test_a1_core_unanswerable_identity_is_not_a_pass():
    """An adapter that cannot answer "is this a Husky" must not score a pass.

    Absence is ``None``, and ``None`` fails the assertion AND reports the
    identity witness as absent -- it never reads as "ten Huskies confirmed".
    """
    b = a1_bundle()
    b.identity = None
    r = a1_core.grade(b)
    assert "A1.1" in r.failed
    identity_clause = [f for f in _ids(r)["A1.1"].falsifiers
                       if f.clause == "identity"][0]
    assert not identity_clause.present

    b2 = a1_bundle(declared=None)
    b2.identity.declared_count = None            # scene ok, declarations not
    r2 = a1_core.grade(b2)
    assert "A1.1" in r2.failed
    assert _ids(r2)["A1.1"].measured[
        "declared robot instances in the artifact"] is None


# --- the vacuity detector ---------------------------------------------------


def test_vacuity_fires_when_contact_evidence_is_absent():
    """THE regression test for the bug that hid for weeks.

    A contact query that reports nothing, with no witness that it could ever
    have reported something, makes A1.3's second clause unfalsifiable. The
    assertion still passes -- the AABBs really are clear -- but the verdict must
    say the contact half proved nothing.
    """
    blind = ContactObservation(pairs=[], steps=10, supported=True,
                              total_observed=None, distinct_named=None,
                              source="a query with no witness counters")
    r = a1_core.grade(a1_bundle(contacts=blind))
    a3 = _ids(r)["A1.3"]
    assert a3.ok is True                       # it "passed"...
    assert a3.vacuous                          # ...and could not have failed
    assert [f.clause for f in a3.vacuous_clauses] == ["no robot-robot contact"]
    assert "A1.3" in r.vacuous
    assert r.vacuous["A1.3"] == ["no robot-robot contact"]
    # and it is loud in the human-readable summary, not only in the JSON
    assert "~vacuous" in r.summary()


def test_vacuity_clears_when_the_contact_query_names_two_bodies():
    r = a1_core.grade(a1_bundle())             # distinct_named = 12
    a3 = _ids(r)["A1.3"]
    assert a3.ok is True
    assert not a3.vacuous
    assert "A1.3" not in r.vacuous


def test_vacuity_catches_the_id_id_self_pair_bug():
    """The historical failure mode, reproduced exactly.

    Every contact names the SAME body twice, so no robot-robot pair can ever be
    formed. ``distinct_named`` is 0 -- honestly counted -- and the clause is
    reported vacuous rather than passing silently.
    """
    self_pairs = [ContactPair(a="#101", b="#101", a_robot=True, b_robot=True,
                             step=k) for k in range(10)]
    broken = ContactObservation(pairs=self_pairs, steps=10, supported=True,
                               total_observed=10, distinct_named=0,
                               source="a node_id lookup that returns self")
    r = a1_core.grade(a1_bundle(contacts=broken))
    a3 = _ids(r)["A1.3"]
    assert a3.measured["robot-robot contacts in first 10 steps"] == 0
    assert a3.ok is True
    assert a3.vacuous
    assert "distinct bodies: 0" in a3.vacuous_clauses[0].detail


def test_vacuity_fires_when_no_contact_query_exists_at_all():
    none = ContactObservation(pairs=[], steps=10, supported=False,
                              source="this simulator exposes no contact query")
    r = a1_core.grade(a1_bundle(contacts=none))
    a3 = _ids(r)["A1.3"]
    assert a3.vacuous
    assert "no contact query" in a3.vacuous_clauses[0].detail


def test_every_assertion_declares_how_it_could_fail():
    r = a1_core.grade(a1_bundle())
    for a in r.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_basis_map_separates_core_physics_from_adapter_claims():
    r = a1_core.grade(a1_bundle())
    basis = r.basis_summary()
    # the metres-and-seconds assertions
    assert set(basis[CORE_PHYSICAL]) == {"A1.5", "A1.6", "A1.7", "A1.8"}
    # the one the adapter answers outright
    assert basis[ADAPTER_ATTESTED] == ["A1.10"]
    assert r.as_dict()["basis"] == basis


# --- C2 ---------------------------------------------------------------------


def c2_bundle(*, dynamic=True, drop_from=0.75, rest_z=0.25, floor_top=0.05,
              duration_s=5.0, attribution="ok", frozen_rest=False):
    n = int(round(duration_s / DT)) + 1
    t = np.arange(n) * DT
    if frozen_rest:                            # the freeze cheat: never falls
        z = np.full(n, drop_from)
    else:
        fall = np.clip(1.0 - np.exp(-t / 0.25), 0.0, 1.0)
        z = drop_from - (drop_from - rest_z) * fall
        z[t > 2.0] = rest_z
    xyz = np.stack([np.stack([np.zeros(n), np.zeros(n), z], axis=1)], axis=0)
    body = Body(body_id="#42", name=c2_core.ROBOT_NAME, kind="Robot",
                position=(0.0, 0.0, float(z[0])),
                aabb_min=(-0.3, -0.3, float(z[0]) - 0.2),
                aabb_max=(0.3, 0.3, float(z[0]) + 0.2),
                n_joints=0, dynamic=dynamic, robot_class=True,
                identity_evidence="synthetic")
    floor = Body(body_id="#7", name=c2_core.FLOOR_NAME, kind="Solid",
                 aabb_min=(-5.0, -5.0, floor_top - 0.1),
                 aabb_max=(5.0, 5.0, floor_top), robot_class=False)
    attrib = (EngineAttribution(backend="synthetic", source="a fixture")
              if attribution == "ok" else None)
    return EvidenceBundle(
        task=c2_core.TASK, sim="synthetic", adapter="tests",
        artifact="/synthetic/fall.wbt",
        roster=BodyInventory(bodies=[body], frozen=True, t_s=0.0,
                             source="synthetic frozen scan"),
        t0=BodyInventory(bodies=[body, floor], frozen=True, t_s=0.0,
                         source="synthetic frozen scan"),
        trajectory=Trajectory(body_ids=["#42"], t=t, xyz=xyz, dt_s=DT,
                              recorded_s=float(t[-1]), complete=True,
                              source="synthetic recorder"),
        process=ProcessFacts(exit_code=0, error_lines=[], log_available=True,
                             driver_completed=True, reached_finalize=True,
                             start_source="synthetic"),
        attribution=attrib)


def test_c2_core_passes_a_body_that_settles_on_the_floor():
    r = c2_core.grade(c2_bundle())
    assert r.outcome == PASS, r.summary()
    assert len(r.assertions) == 5


def test_c2_core_catches_the_freeze_cheat():
    """Physics removed and pinned mid-air: 'stays up', fails the anti-cheat."""
    r = c2_core.grade(c2_bundle(dynamic=False, frozen_rest=True))
    assert r.outcome == FAIL
    assert set(r.failed) == {"C2.3", "C2.4", "C2.5"}
    assert _ids(r)["C2.2"].ok is True          # the naive test still passes


def test_c2_core_measures_the_floor_the_agent_built():
    """Raise the floor by 0.5 m and the expected rest height follows it."""
    r = c2_core.grade(c2_bundle(floor_top=0.55, rest_z=0.75, drop_from=1.25))
    assert r.outcome == PASS, r.summary()
    assert _ids(r)["C2.5"].measured["floor top z (m)"] == pytest.approx(0.55)


def test_c2_core_missing_attribution_is_invalid():
    r = c2_core.grade(c2_bundle(attribution=None))
    assert r.outcome == INVALID


def test_c2_core_needs_the_floor_body_in_the_inventory():
    b = c2_bundle()
    b.t0.bodies = [b.t0.bodies[0]]             # drop the floor
    r = c2_core.grade(b)
    assert "C2.5" in r.failed
    assert not _ids(r)["C2.5"].falsifiers[0].present   # witness absent


# --- B3 ---------------------------------------------------------------------


def b3_bundle(*, gap_m=4.0, plinth_h=0.4):
    ground = Body(body_id="#1", name=b3_core.GROUND_NAME, kind="Robot",
                  position=(0.0, 0.0, 0.2), aabb_min=(-0.5, -0.5, 0.05),
                  aabb_max=(0.5, 0.5, 0.45), robot_class=True)
    plinth = Body(body_id="#2", name=b3_core.PLINTH_NAME, kind="Robot",
                  position=(gap_m, 0.0, 0.2 + plinth_h),
                  aabb_min=(gap_m - 0.5, -0.5, 0.05 + plinth_h),
                  aabb_max=(gap_m + 0.5, 0.5, 0.45 + plinth_h),
                  robot_class=True)
    return EvidenceBundle(
        task=b3_core.TASK, sim="synthetic",
        t0=BodyInventory(bodies=[ground, plinth], frozen=True, t_s=0.0,
                         source="synthetic frozen scan"))


def test_b3_core_accepts_a_correct_measured_answer():
    r = b3_core.grade(b3_bundle(gap_m=4.0), answer=(
        "They are 4.00 m apart centre to centre, and husky_plinth is taller."))
    assert r.outcome == PASS, r.summary()


def test_b3_core_rejects_a_confident_wrong_answer():
    r = b3_core.grade(b3_bundle(gap_m=4.0), answer=(
        "The two Huskies are 9.10 m apart centre to centre. husky_ground is "
        "the taller of the two."))
    assert set(r.failed) == {"B3.2", "B3.4"}


def test_b3_core_distance_tolerance_is_the_spec_number():
    """Measured off the ground truth, so the test cannot drift from the core.

    Either reading of "centre to centre" is accepted (body origin or geometric
    centre), so the boundary is taken from the furthest accepted target.
    """
    assert b3_core.DISTANCE_TOL_M == 0.30
    b = b3_bundle(gap_m=4.0)
    truth = b3_core.measure_ground_truth(b)
    far = max(truth["distance_m"], truth["aabb_distance_m"])
    ok = b3_core.grade(b, answer="%.4f m apart; husky_plinth is taller."
                       % (far + 0.29))
    bad = b3_core.grade(b, answer="%.4f m apart; husky_plinth is taller."
                        % (far + 0.31))
    assert ok.outcome == PASS and "B3.2" in bad.failed


def test_b3_core_invalid_without_ground_truth():
    b = b3_bundle()
    b.t0 = BodyInventory(bodies=[], error="phase B not run")
    r = b3_core.grade(b, answer="4.00 m; husky_plinth")
    assert r.outcome == INVALID


# -- B3 anti-shotgun (the regression tests) ---------------------------------

def _shotgun(step, hi, unit=" m"):
    """An answer that BLANKETS the plausible range instead of measuring it."""
    vals = [step * i for i in range(1, int(round(hi / step)) + 1)]
    return ("I did not inspect anything. The separation is somewhere in this "
            "range: %s. The robot on the plinth is taller."
            % ", ".join("%.1f%s" % (v, unit) for v in vals))


def test_b3_core_rejects_the_shotgun_answer_that_measured_nothing():
    """THE regression test: an answer that says it measured nothing must fail.

    Before the fix this exact answer scored **PASS 4/4**. Two independent
    holes, both of which had to close:

      * B3.2 took ``min`` of the error over EVERY number in the answer against
        BOTH ground truths, so a list at 0.5 m spacing -- never more than
        0.25 m from any value, against a 0.30 m tolerance -- covered the whole
        plausible range and always "matched";
      * ``_named_taller`` read any mention of "plinth"/"raised"/"on top" as
        naming the plinth robot, and the plinth robot is BY CONSTRUCTION the
        taller one, so B3.3 and B3.4 were free to anyone who restated the
        task's own initial conditions.

    B3.1 still passes, correctly: a number IS present. The physical clauses
    are the ones that must not.
    """
    r = b3_core.grade(b3_bundle(gap_m=4.0), answer=_shotgun(0.5, 8.0))
    assert r.outcome == FAIL, r.summary()
    assert set(r.failed) == {"B3.2", "B3.3", "B3.4"}
    assert _ids(r)["B3.2"].measured["scored stated (m)"] == pytest.approx(0.5)
    assert _ids(r)["B3.3"].measured["named"] is None


@pytest.mark.parametrize("step,hi,unit", [(0.5, 8.0, " m"), (0.1, 8.0, " m"),
                                          (0.5, 8.0, "")])
def test_b3_core_no_shotgun_spacing_can_pass(step, hi, unit):
    """Denser or unit-free, blanketing the range must never score a match.

    A rule of the form "reject when more than one number lands inside the
    tolerance" would NOT catch the 0.5 m list (only one of its values is within
    0.30 m of the truth). Scoring exactly one committed number does, at every
    spacing.
    """
    r = b3_core.grade(b3_bundle(gap_m=4.0), answer=_shotgun(step, hi, unit))
    assert "B3.2" in r.failed


def test_b3_core_scores_one_number_not_the_best_of_many():
    """An honest answer may carry other measurements and still be scored on
    the number it offered as the separation."""
    b = b3_bundle(gap_m=4.0)
    truth = b3_core.measure_ground_truth(b)
    answer = ("The plinth is 0.40 m high; the bounding-box tops are 0.85 m and "
              "0.45 m. The two Huskies are %.2f m apart centre to centre, and "
              "husky_plinth is taller." % truth["distance_m"])
    r = b3_core.grade(b, answer=answer)
    assert r.outcome == PASS, r.summary()
    assert _ids(r)["B3.2"].measured["scored stated (m)"] == pytest.approx(
        round(truth["distance_m"], 2))
    assert 0.4 in _ids(r)["B3.2"].measured[
        "other numbers in the answer, not scored"]


def test_b3_core_requires_the_answer_to_NAME_the_taller_body():
    """"The one on the plinth" is a restatement of the task, not a reading."""
    b = b3_bundle(gap_m=4.0)
    truth = b3_core.measure_ground_truth(b)
    r = b3_core.grade(b, answer=("They are %.2f m apart centre to centre. The "
                                 "robot on the raised plinth is taller."
                                 % truth["distance_m"]))
    assert set(r.failed) == {"B3.3", "B3.4"}
    assert _ids(r)["B3.2"].ok is True        # the distance half is unaffected


# --- the boundary itself ----------------------------------------------------

# Tokens that name one simulator's internals. A core that contains any of these
# outside a docstring has leaked back across the line the split exists to draw.
_SIM_TOKENS = ("urdfrobot", "husky.urdf", ".wbt", "newton", "sidecar",
               "physicsbackend", "gettypename", "boundingobject", "proto",
               "sdf", "usd", "mjcf", "rclpy", "harness", "recorder")


def _code_strings_and_names(path):
    """Every string literal and identifier in a module, MINUS docstrings."""
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docstrings:
                out.append(node.value)
        elif isinstance(node, ast.Name):
            out.append(node.id)
        elif isinstance(node, ast.Attribute):
            out.append(node.attr)
    return out


@pytest.mark.parametrize("mod", [a1_core, c2_core, b3_core,
                                 "agentbench.graders.b1_core",
                                 "agentbench.graders.b2_core",
                                 "agentbench.graders.c1_core",
                                 "agentbench.graders.evidence",
                                 "agentbench.graders.physical"])
def test_cores_contain_no_simulator_specific_vocabulary(mod):
    import importlib
    if isinstance(mod, str):
        mod = importlib.import_module(mod)
    path = mod.__file__
    for text in _code_strings_and_names(path):
        low = text.lower()
        for token in _SIM_TOKENS:
            assert token not in low, (
                "%s mentions %r in code (not a docstring): %r"
                % (Path(path).name, token, text))


def test_the_contract_self_check_accepts_a_well_formed_bundle():
    assert adapters.check_bundle(a1_bundle()) == []
    assert adapters.check_bundle(c2_bundle()) == []


def test_the_contract_self_check_names_the_usual_adapter_mistakes():
    b = a1_bundle()
    b.attribution.source = ""                  # an engine with no citation
    b.contacts.total_observed = None           # no vacuity witness
    b.contacts.distinct_named = None
    b.t0.frozen = False                        # a free-running "t=0"
    problems = adapters.check_bundle(b)
    assert any("attribution" in p for p in problems)
    assert any("vacuity witness" in p for p in problems)
    assert any("frozen" in p for p in problems)


def test_the_adapter_registry_resolves_omnisim_and_reports_missing_sims():
    for sim in ("omnisim", "webots"):       # the two implemented arms
        mod = adapters.resolve(sim)
        assert hasattr(mod, "build_bundle"), sim
    with pytest.raises(adapters.AdapterError):
        adapters.resolve("gazebo")          # registered, not implemented yet
    with pytest.raises(adapters.AdapterError):
        adapters.resolve("nosuchsim")
    assert "husky" in adapters.ROBOT_IDENTITIES
    assert {k for k, _ in adapters.REQUIRED_EVIDENCE} >= {
        "roster", "t0", "contacts", "trajectory", "process", "attribution",
        "identity"}
