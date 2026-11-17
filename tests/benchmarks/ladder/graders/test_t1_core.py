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

"""Unit tests for T1's neutral core. **No simulator involved.**

Every fixture is synthetic numpy, so this file runs in milliseconds on a
machine with no build, no GPU and no network -- which is the point of the
neutral core: a third party can re-derive every verdict in the ladder without
our stack.

    pytest tests/benchmarks/ladder/graders/ -q

The centre of the file is :func:`test_every_negative_fixture_reddens_exactly
_its_own_assertion`. It is the executable form of the red-evidence rule
(``capability-ladder-plan.md`` §5c.2) and it checks the half that rule warns
about most: not that a wrong artifact fails, but that it fails **the right
assertion and no other**. A fixture set where everything fails everything
would satisfy a naive reading and prove nothing.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders.verdict import (  # noqa: E402
    CORE_PHYSICAL, FAIL, INVALID, MIXED, PASS)
from ladder import adapters, tasks  # noqa: E402
from ladder.graders import fixtures, t1_core  # noqa: E402
from ladder.graders import ladder_evidence as lev  # noqa: E402


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the positive control ----------------------------------------------------


def test_the_oracle_run_passes_every_assertion():
    v = t1_core.grade(fixtures.oracle())
    assert v.outcome == PASS, v.summary()
    assert len(v.assertions) == 5
    assert v.progress == 4
    assert not v.vacuous
    a = _ids(v)
    assert a["T1.1"].measured["final horizontal error (m)"] == pytest.approx(
        0.0, abs=1e-6)
    assert a["T1.3"].measured["path length / straight line"] == pytest.approx(
        1.0, abs=0.02)
    assert a["T1.4"].measured[
        "contacts naming the robot and a non-robot surface"] > 0


def test_the_oracle_evidence_satisfies_both_contracts():
    ev = fixtures.oracle()
    assert lev.check_t1_evidence(ev) == []


# --- the thresholds ----------------------------------------------------------


def test_thresholds_are_the_plans_numbers():
    """A drift in any of these is a tier change, not a refactor.

    capability-ladder-plan.md §5a: moving a threshold voids the pass and
    forces a version bump and a full re-run.
    """
    assert (t1_core.ARRIVE_TOL_M, t1_core.HOLD_TOL_M, t1_core.DWELL_S) == (
        0.25, 0.35, 2.0)
    assert (t1_core.MAX_ABS_DZ_M, t1_core.MIN_Z_M) == (0.30, -0.10)
    assert (t1_core.MIN_PATH_RATIO, t1_core.MAX_PATH_RATIO) == (0.9, 5.0)
    assert t1_core.MAX_STEP_JUMP_M == 0.5


def test_the_task_file_and_the_core_agree_on_every_constant():
    """The task's meta.json is what a reader opens; the core is what runs.

    Two copies of a threshold is one threshold and one lie waiting to happen,
    so they are compared rather than trusted.
    """
    task = tasks.get(t1_core.TASK)
    for name, value in task.constants.items():
        assert hasattr(t1_core, name), "meta.json declares unknown %r" % name
        assert getattr(t1_core, name) == pytest.approx(value), name
    # ...and nothing the core exports as a threshold is missing from the file
    declared = set(task.constants)
    in_core = {n for n in t1_core.__all__
               if n.isupper() and isinstance(getattr(t1_core, n),
                                             (int, float))
               and not isinstance(getattr(t1_core, n), bool)}
    assert in_core - declared == set(), in_core - declared


# --- the red-evidence rule ---------------------------------------------------


@pytest.mark.parametrize("name", sorted(fixtures.FIXTURE_ASSERTION_MAP))
def test_every_negative_fixture_reddens_exactly_its_own_assertion(name):
    """THE test the red-evidence rule exists for.

    Exact, in both directions: the declared assertion must be red, and no
    other one may be. §5c.2's own caveat is that "a null agent turning every
    assertion red does not satisfy this rule" -- so a fixture that reddens two
    assertions has stopped being evidence that either check discriminates.
    """
    _verdict, problems = fixtures.check_fixture(name)
    assert not problems, "%s: %s" % (name, "; ".join(problems))


def test_every_assertion_has_a_negative_fixture():
    assert fixtures.uncovered_assertions() == ()
    covered = {r["assertion"] for r in fixtures.coverage_table()}
    assert covered == set(fixtures.ASSERTIONS)


def test_the_coverage_table_names_a_fixture_for_each_assertion():
    text = fixtures.render_coverage_table()
    for aid in fixtures.ASSERTIONS:
        assert aid in text
    assert "every assertion has a negative fixture" in text


def test_every_assertion_declares_how_it_could_fail():
    v = t1_core.grade(fixtures.oracle())
    for a in v.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_basis_map_separates_measured_physics_from_adapter_claims():
    v = t1_core.grade(fixtures.oracle())
    basis = v.basis_summary()
    assert set(basis[CORE_PHYSICAL]) == {"T1.1", "T1.2", "T1.3"}
    assert set(basis[MIXED]) == {"T1.4", "T1.5"}
    assert v.as_dict()["basis"] == basis


# --- the vacuity machinery ---------------------------------------------------


def test_a_blind_contact_query_is_red_AND_reported_vacuous():
    """The shape of the bug the vacuity detector was built for.

    A contact query that returns nothing and cannot say whether it *could*
    have returned something proves nothing either way. T1.4 is red because
    the tier requires a contact -- but the row must also say the clause could
    not have fired.
    """
    v = t1_core.grade(fixtures.blind_contact_query())
    a = _ids(v)["T1.4"]
    assert a.ok is False
    assert a.vacuous
    assert set(f.clause for f in a.vacuous_clauses) == {
        "touched the surface", "touched it while moving"}
    assert "~vacuous" in v.summary()


def test_a_working_query_that_saw_no_support_contact_is_red_but_NOT_vacuous():
    """A flying robot and a broken contact pipeline must not read the same."""
    v = t1_core.grade(fixtures.flying_no_contact())
    a = _ids(v)["T1.4"]
    assert a.ok is False
    touched = [f for f in a.falsifiers if f.clause == "touched the surface"][0]
    assert touched.present, "the query demonstrably named two distinct bodies"


def test_contacts_with_no_timestamps_pass_the_half_that_is_measurable():
    """A realistic partial adapter: it sees contacts, it cannot time them."""
    v = t1_core.grade(fixtures.untimed_contacts())
    assert v.outcome == PASS
    a = _ids(v)["T1.4"]
    assert a.vacuous
    assert [f.clause for f in a.vacuous_clauses] == ["touched it while moving"]
    assert a.measured["...of those, seen while the robot was moving"] is None


def test_the_jump_clause_is_vacuous_on_a_series_too_coarse_to_see_a_jump():
    """A 0.5 m bound at a 1 Hz sample rate is not a teleport test."""
    ev = fixtures.oracle()
    ev.bundle.trajectory.dt_s = 1.0
    v = t1_core.grade(ev)
    a = _ids(v)["T1.3"]
    jump = [f for f in a.falsifiers if f.clause == "did not jump"][0]
    assert not jump.present
    assert "T1.3" in v.vacuous


# --- T1.1's reading, stated in tests so it can be argued with ----------------


def test_a_robot_that_touches_the_point_then_settles_off_it_still_arrived():
    """The reading the module docstring commits to, made checkable.

    It reached the commanded point, then relaxed 0.30 m off it and held
    there. Both tolerances are satisfied -- best error inside the trailing
    band is ~0, the band itself is never left -- and that is arrival by any
    physical account.
    """
    ev = fixtures.oracle()
    xyz = ev.bundle.trajectory.xyz
    n_travel = int(round(fixtures.TRAVEL_S / fixtures.DT))
    tail = xyz.shape[1] - n_travel
    xyz[:, n_travel:, 1] += np.linspace(0.0, 0.30, tail)
    v = t1_core.grade(ev)
    assert v.outcome == PASS, v.summary()
    a = _ids(v)["T1.1"]
    assert a.measured["final horizontal error (m)"] == pytest.approx(0.30,
                                                                     abs=1e-3)
    assert a.measured[
        "best horizontal error inside the trailing window (m)"] < 0.25


def test_loitering_inside_the_outer_band_without_arriving_is_not_arrival():
    """The other half of the same reading: 0.35 m is a band, not the target."""
    t, xyz = fixtures._track(arrive_frac=(5.0 - 0.30) / 5.0)
    v = t1_core.grade(fixtures._evidence(t, xyz))
    assert v.failed == ["T1.1"]
    a = _ids(v)["T1.1"]
    assert a.measured["trailing window within 0.35 m (s)"] >= t1_core.DWELL_S
    assert a.measured[
        "best horizontal error inside the trailing window (m)"] > 0.25


# --- selecting the robot -----------------------------------------------------


def test_the_declared_name_selects_the_body_even_among_several():
    ev = fixtures.oracle()
    other = type(ev.bundle.roster.bodies[0])(
        body_id="#99", name="decoy", kind="Robot", robot_class=True,
        identity_evidence="synthetic")
    ev.bundle.roster.bodies.insert(0, other)
    index, body, why = t1_core.select_robot(ev)
    assert body.name == fixtures.ROBOT_NAME
    assert index == 1
    assert "declares" in why


def test_a_single_robot_under_a_different_name_is_still_graded():
    """An importer that names the body something else is not a FAIL."""
    ev = fixtures.oracle()
    ev.bundle.roster.bodies[0].name = "whatever_the_importer_called_it"
    index, body, why = t1_core.select_robot(ev)
    assert index == 0 and body is not None
    assert "single robot-class body" in why


def test_several_unnamed_robots_are_refused_rather_than_guessed_between():
    ev = fixtures.oracle()
    b = ev.bundle.roster.bodies[0]
    b.name = "one"
    twin = type(b)(body_id="#98", name="two", kind="Robot", robot_class=True,
                   identity_evidence="synthetic")
    ev.bundle.roster.bodies.append(twin)
    v = t1_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert "no one body to grade" in _ids(v)["T1.1"].detail


# --- absent evidence ---------------------------------------------------------


def test_no_deliverable_scores_zero_of_five():
    ev = fixtures.oracle()
    ev.bundle.artifact = None
    v = t1_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert v.progress == 0


def test_no_motion_series_is_a_fail_with_the_reason_named():
    ev = fixtures.oracle()
    ev.bundle.trajectory = None
    ev.bundle.motion_error = "the sample file was never written"
    v = t1_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert _ids(v)["T1.2"].detail == "the sample file was never written"


def test_a_series_with_no_time_base_cannot_be_graded():
    ev = fixtures.oracle()
    ev.bundle.trajectory.t = None
    v = t1_core.grade(ev)
    assert len(v.failed) == 5
    assert "no simulated-time base" in _ids(v)["T1.1"].detail


def test_a_missing_commanded_point_is_invalid_not_failed():
    """The waypoint is task data. Losing it is a broken instrument."""
    ev = fixtures.oracle()
    ev.waypoint = lev.Waypoint(xy=(float("nan"), 0.0), source="")
    v = t1_core.grade(ev)
    assert v.outcome == INVALID
    assert v.outcome != FAIL
    assert any("commanded point" in n for n in v.notes)


def test_missing_attribution_is_invalid_not_failed():
    v = t1_core.grade(fixtures.missing_attribution())
    assert v.outcome == INVALID
    assert "T1.5" in v.failed
    assert any("unattributed" in n for n in v.notes)


# --- the measurement helpers -------------------------------------------------


def test_trailing_dwell_measures_the_trailing_run_only():
    t = np.arange(11) * 1.0
    err = np.array([9, 9, 0.1, 0.1, 0.1, 9, 9, 0.2, 0.2, 0.2, 0.2])
    secs, i0 = t1_core.trailing_dwell(t, err, 0.35)
    assert i0 == 7 and secs == pytest.approx(3.0)


def test_trailing_dwell_is_zero_when_the_run_ends_outside_the_band():
    t = np.arange(5) * 1.0
    err = np.array([0.1, 0.1, 0.1, 0.1, 9.0])
    secs, i0 = t1_core.trailing_dwell(t, err, 0.35)
    assert secs == 0.0 and i0 is None


def test_max_step_jump_is_three_dimensional():
    xyz = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.1, 0.0, 0.9]])
    assert t1_core.max_step_jump(xyz) == pytest.approx(0.9)


def test_speed_at_reads_the_sample_nearest_the_contact():
    t = np.arange(5) * 0.1
    xyz = np.stack([np.arange(5) * 0.2, np.zeros(5), np.zeros(5)], axis=1)
    assert t1_core.speed_at(t, xyz, 0.15) == pytest.approx(2.0)
    assert t1_core.speed_at(t, xyz, None) is None


# --- the ladder's own evidence channels --------------------------------------


def test_the_evidence_contract_names_the_usual_adapter_mistakes():
    ev = fixtures.oracle()
    ev.waypoint.source = ""
    ev.support.total_observed = None
    ev.support.distinct_named = None
    ev.support.pairs[0].surface_is_robot = None
    problems = lev.check_t1_evidence(ev)
    assert any("source citation" in p for p in problems)
    assert any("vacuity witness" in p for p in problems)
    assert any("whether the other side is a robot" in p for p in problems)


def test_a_contact_whose_other_side_is_unattested_is_not_support():
    """An adapter that cannot say 'not a robot' is not saying it."""
    c = lev.SupportContact(robot_body="a", surface_body="b",
                           surface_is_robot=None)
    assert c.distinct and not c.is_support
    assert lev.SupportContact(robot_body="a", surface_body="b",
                              surface_is_robot=False).is_support


def test_the_fallback_derivation_reports_no_pairs_on_a_robot_robot_channel():
    """The gap, asserted rather than described.

    A bundle whose contact channel is the frozen robot-robot filter yields no
    support pairs at all -- which is why a column without a ladder-side
    sampler gets the gap note and a blocker that belongs to us.
    """
    from agentbench.graders.evidence import ContactObservation, ContactPair
    bundle = fixtures.oracle().bundle
    bundle.contacts = ContactObservation(
        pairs=[ContactPair(a="#1", b="#2", a_robot=True, b_robot=True)],
        supported=True, total_observed=8, distinct_named=8, source="frozen")
    derived = lev.support_from_bundle(bundle)
    assert derived.support_pairs == []
    assert derived.total_observed == 8          # the witness survives
    assert not derived.can_name_a_support_pair or not derived.support_pairs


# --- the shim over agentbench's adapters -------------------------------------


def test_the_shim_delegates_to_agentbenchs_registry():
    from agentbench import adapters as ab
    assert adapters.resolve("webots") is ab.resolve("webots")
    with pytest.raises(ab.AdapterError):
        adapters.resolve("nosuchsim")


def test_the_omnisim_column_supplies_the_ladders_extra_channel():
    channels = adapters.resolve_ladder_channels("omnisim")
    assert channels is not None
    assert hasattr(channels, "support_observation")
    assert hasattr(channels, "run_standalone")
    assert channels.support_observation(None) is None


def test_the_ladder_declares_what_a_new_column_must_add():
    keys = {k for k, _why in adapters.LADDER_REQUIRED_EVIDENCE}
    assert keys == {"waypoint", "support_contacts"}
    for _k, why in adapters.LADDER_REQUIRED_EVIDENCE:
        assert why


# --- the task assets ---------------------------------------------------------


def test_the_prompt_is_one_sentence():
    task = tasks.get(t1_core.TASK)
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
    "http", "api", "checkpoint", "policy", "reinforcement", "import")


def test_the_prompt_names_no_format_no_endpoint_and_no_products_name():
    task = tasks.get(t1_core.TASK)
    low = task.prompt.lower()
    for token in _FORBIDDEN_IN_PROMPT:
        assert not re.search(r"\b%s\b" % re.escape(token), low), token


def test_the_container_ships_the_description_and_nothing_else():
    task = tasks.get(t1_core.TASK)
    on_disk = sorted(p.relative_to(task.container_dir).as_posix()
                     for p in task.container_dir.rglob("*") if p.is_file())
    declared = sorted(
        (task.container_dir / n).relative_to(task.container_dir).as_posix()
        for n in task.meta["container"]["files"])
    assert on_disk == declared
    assert not any(p.suffix in (".wbt", ".sdf", ".usd", ".xml", ".py", ".sh")
                   for p in task.container_files), \
        "the container ships a scene, a script or a converted model"


def test_the_container_leaves_the_mesh_references_as_upstream_wrote_them():
    """Rewriting them would do one of the measured steps for every column."""
    task = tasks.get(t1_core.TASK)
    urdf = task.container_dir / "husky_description" / "urdf" / "husky.urdf"
    text = urdf.read_text(encoding="utf-8", errors="replace")
    assert "package://husky_description/meshes/" in text
    for ref in re.findall(r'filename="package://husky_description/([^"]+)"',
                          text):
        assert (task.container_dir / "husky_description" / ref).exists(), ref


def test_the_waypoint_resolves_against_the_runs_own_start():
    task = tasks.get(t1_core.TASK)
    assert task.waypoint_spec["frame"] == "start_relative"
    assert tasks.resolve_waypoint(task, (3.0, -2.0)) == (3.0, 3.0)
    assert "start_relative" in tasks.waypoint_citation(task)


def test_the_task_declares_its_pre_registered_expectation():
    """§5c.1: written before any cell runs, printed beside the outcome."""
    task = tasks.get(t1_core.TASK)
    exp = task.meta["expectation"]
    assert exp["omnisim"] and exp["reasoning"]
    assert task.meta["scoring_class"] == "exploratory"
    assert task.repeats_default == 3


def test_the_task_meta_is_valid_json_and_names_its_grader():
    task = tasks.get(t1_core.TASK)
    assert task.meta["grader"] == "ladder.graders.t1"
    json.dumps(task.meta)          # round-trips
    assert task.rung == "T1"
