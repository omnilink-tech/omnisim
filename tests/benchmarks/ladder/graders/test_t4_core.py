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

"""Unit tests for T4's neutral core. **No simulator involved.**

Every fixture is synthetic numpy, so this file runs in a second on a machine
with no build, no GPU and no network -- which is the point of the neutral core:
a third party can re-derive every verdict in the ladder without our stack.

    pytest tests/benchmarks/ladder/graders/test_t4_core.py -q

Six tests carry more weight than the rest:

:func:`test_the_four_reused_rows_are_the_other_cores_own_rows` is the
executable form of ``capability-ladder-plan.md`` §2 T4's *"T3.1, T3.2, T3.3,
T3.5 unchanged"*. It grades one run through both cores and requires those four
rows to come out field for field identical. A copy that happened to agree today
would pass a threshold test and fail this one the day either tier is corrected.

:func:`test_every_negative_fixture_reddens_exactly_its_own_assertion` is the
executable form of the red-evidence rule (§5c.2), exact in three directions:
the assertion, the vacuous clauses, **and the cell**.

:func:`test_a_supported_run_passes_in_the_other_cell_with_its_numbers` and
:func:`test_a_robot_carried_outright_is_published_not_failed` are §2 T4's most
counter-intuitive ruling made executable: *a supported run is a different cell,
not a failure*, and the figures are printed inside it.

:func:`test_the_support_profile_separates_an_idle_channel_from_a_live_one` is
the measured reason the profile is per axis: on the only real support rig this
tree has measured, the carrying channel was idle for the whole walk while the
attitude channel ran continuously, and one scalar hides that entirely.

:func:`test_a_run_bound_by_the_world_while_still_moving_is_seen_here_and_not_below`
is the one place this tier deliberately strengthens the tier below, and it
asserts the difference rather than describing it.

:func:`test_our_own_flagships_measured_speed_is_red_on_this_tier` is not a test
of the grader. It is a **finding**, kept executable so it cannot be forgotten
before the freeze.

The AST guard that keeps the cores simulator-neutral is **replicated at the
bottom of this file**, deliberately, until T4's modules are added to the shared
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
from ladder.graders import fixtures_t4, t3_core, t4, t4_core  # noqa: E402
from ladder.graders import t4_evidence as t4ev  # noqa: E402


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the positive control ----------------------------------------------------


def test_the_oracle_run_passes_every_assertion():
    v = t4_core.grade(fixtures_t4.oracle())
    assert v.outcome == PASS, v.summary()
    assert len(v.assertions) == 5
    assert v.progress == 4
    assert not v.vacuous
    a = _ids(v)
    assert a["T4.1"].measured["furthest from the start reached (m)"] >= 10.0
    assert a["T4.2"].measured["samples over the bound"] == 0
    assert a["T4.3"].measured[
        "mean speed made good over the scored window (m/s)"] >= 0.15
    assert a["T4.4"].measured["support attestation"] == t4ev.ATTESTED
    assert v.measurements["cell"] == t4ev.CELL_UNSUPPORTED


def test_the_oracle_evidence_satisfies_the_ladders_contract():
    assert t4ev.check_t4_evidence(fixtures_t4.oracle()) == []
    assert t4ev.unanswered_channels(fixtures_t4.oracle()) == {}


# --- "unchanged" means the same function -------------------------------------


# The two arena fixtures are excluded on purpose and the exclusion is the
# point: this tier's arena rule is deliberately stronger (see
# ``test_a_run_bound_by_the_world_while_still_moving_...``), so on those two
# the scored window legitimately differs and the rows legitimately differ with
# it. Everywhere else the reused rows must be identical.
_WINDOW_IDENTICAL = tuple(n for n in sorted(fixtures_t4.FIXTURE_ASSERTION_MAP)
                          if not n.startswith("arena_bound"))


@pytest.mark.parametrize("name", _WINDOW_IDENTICAL)
def test_the_four_reused_rows_are_the_other_cores_own_rows(name):
    """§2 T4: "T3.1, T3.2, T3.3, T3.5 unchanged" -- asserted, not promised.

    Both cores grade the same evidence; the four rows this tier does not write
    must come back field for field identical, id excepted. That is what makes
    a later correction to either tier propagate instead of silently diverging.
    """
    ev = fixtures_t4.FIXTURE_ASSERTION_MAP[name]["fn"]
    mine = _ids(t4_core.grade(ev()))
    theirs = _ids(t3_core.grade(ev()))
    for t4id, t3id in t4_core.REUSED_ROWS.items():
        assert mine[t4id].as_dict() == theirs[t3id].as_dict(), (name, t4id)


def test_the_reuse_map_names_the_four_rows_the_plan_names():
    assert t4_core.REUSED_ROWS == {"T4.1": "T3.1", "T4.2": "T3.2",
                                   "T4.3": "T3.3", "T4.5": "T3.5"}
    own = set(fixtures_t4.ASSERTIONS) - set(t4_core.REUSED_ROWS)
    assert own == {"T4.4"}, "this tier writes exactly one row of its own"


def test_this_tier_declares_no_threshold_of_its_own():
    """Every number it grades against is the tier below's module global.

    Not a style preference: the reused rows read THEIR module's globals, so a
    copy here would be a number that looks binding and is not.
    """
    tree = ast.parse(Path(t4_core.__file__).read_text(encoding="utf-8"))
    assigned = {t.id for node in ast.walk(tree)
                if isinstance(node, ast.Assign)
                for t in node.targets if isinstance(t, ast.Name)}
    for name in tasks.get(t4_core.TASK).constants:
        assert name not in assigned, (
            "%s is assigned in t4_core; it must be imported" % name)


# --- the thresholds ----------------------------------------------------------


def test_thresholds_are_the_plans_numbers():
    """A drift in any of these is a tier change, not a refactor.

    capability-ladder-plan.md §5a: moving a threshold voids the pass and
    forces a version bump and a full re-run. §2 T4 in particular kept the bar
    at 10.0 m verbatim AFTER measuring our own artifact against it.
    """
    assert t4_core.NET_DISPLACEMENT_M == 10.0
    assert (t4_core.FALL_Z_FRACTION, t4_core.MAX_TILT_RAD) == (0.60, 0.8)
    assert t4_core.MIN_MEAN_SPEED_MPS == 0.15
    assert t4_core.MIN_BOB_RMS_M == 0.005
    assert t4_core.MIN_CONTACT_TRANSITIONS == 8
    assert (t4_core.MAX_SUPPORT_FORCE_FRACTION,
            t4_core.MAX_SUPPORT_TORQUE_NM) == (0.02, 2.0)


def test_the_task_file_and_the_core_agree_on_every_constant():
    """The task's meta.json is what a reader opens; the core is what runs.

    Two copies of a threshold is one threshold and one lie waiting to happen,
    so they are compared rather than trusted -- in BOTH directions.
    """
    task = tasks.get(t4_core.TASK)
    for name, value in task.constants.items():
        assert hasattr(t4_core, name), "meta.json declares unknown %r" % name
        assert getattr(t4_core, name) == pytest.approx(value), name
    declared = set(task.constants)
    in_core = {n for n in t4_core.__all__
               if n.isupper() and isinstance(getattr(t4_core, n), (int, float))
               and not isinstance(getattr(t4_core, n), bool)}
    assert in_core - declared == set(), in_core - declared


# --- the red-evidence rule ---------------------------------------------------


@pytest.mark.parametrize("name", sorted(fixtures_t4.FIXTURE_ASSERTION_MAP))
def test_every_negative_fixture_reddens_exactly_its_own_assertion(name):
    """THE test the red-evidence rule exists for.

    Exact in three directions: the declared assertion must be red and no other
    one may be, the declared clauses must be vacuous and no other one may be,
    and the run must land in the declared cell. §5c.2's own caveat is that "a
    null agent turning every assertion red does not satisfy this rule".
    """
    _verdict, problems = fixtures_t4.check_fixture(name)
    assert not problems, "%s: %s" % (name, "; ".join(problems))


def test_every_assertion_has_a_negative_fixture():
    assert fixtures_t4.uncovered_assertions() == ()
    covered = {r["assertion"] for r in fixtures_t4.coverage_table()}
    assert covered == set(fixtures_t4.ASSERTIONS)


def test_every_cell_the_tier_can_publish_has_a_fixture_that_lands_in_it():
    """Otherwise the two-cell mechanism is implemented and not falsifiable."""
    assert fixtures_t4.uncovered_cells() == ()


def test_the_coverage_table_names_a_fixture_for_each_assertion():
    text = fixtures_t4.render_coverage_table()
    for aid in fixtures_t4.ASSERTIONS:
        assert aid in text
    assert "every assertion has a negative fixture" in text
    # ...and the fixtures that must PASS are visible as such, because a reader
    # scanning only the red rows would conclude that support is a failure
    assert "g1_shaped_support" in text
    assert "carried_outright" in text
    assert "support_not_attested" in text


def test_every_assertion_declares_how_it_could_fail():
    v = t4_core.grade(fixtures_t4.oracle())
    for a in v.assertions:
        assert a.falsifiers, "%s declares no way to fail" % a.id
        for f in a.falsifiers:
            assert f.how_to_fail and f.witness


def test_basis_map_separates_measured_physics_from_adapter_claims():
    v = t4_core.grade(fixtures_t4.oracle())
    basis = v.basis_summary()
    assert set(basis[CORE_PHYSICAL]) == {"T4.1", "T4.2", "T4.3"}
    assert set(basis[MIXED]) == {"T4.4", "T4.5"}
    assert v.as_dict()["basis"] == basis


# --- the two cells -----------------------------------------------------------


def test_a_supported_run_passes_in_the_other_cell_with_its_numbers():
    """§2 T4: a supported run is a different cell, not a failed one.

    The support profile here is the measured flagship's, on the weight-bearing
    balance rig this tree's every humanoid result runs on.
    """
    v = t4_core.grade(fixtures_t4.g1_shaped_support())
    assert v.outcome == PASS
    assert v.failed == []
    assert v.measurements["cell"] == t4ev.CELL_SUPPORTED
    text = v.measurements["cell_text"]
    for piece in ("supported:", "x body weight", "N.m", "% of window"):
        assert piece in text, text
    a = _ids(v)["T4.4"]
    assert a.ok is True
    assert a.measured["peak applied force (N)"] == pytest.approx(
        fixtures_t4.G1_FY_N)
    assert a.measured["...as a multiple of body weight"] == pytest.approx(
        fixtures_t4.G1_FY_N / (fixtures_t4.MASS_KG * fixtures_t4.G), abs=1e-4)


def test_a_robot_carried_outright_is_published_not_failed():
    """The plan's own worked cell, rendered from measurement.

    "achieved 1/3 (supported: peak 2.09 x body weight, 348 N.m, 100% of
    window)". A rig doing most of the work is a disclosure, not a red.
    """
    v = t4_core.grade(fixtures_t4.carried_outright())
    assert v.outcome == PASS
    assert v.measurements["cell"] == t4ev.CELL_SUPPORTED
    text = v.measurements["cell_text"]
    assert "2.09" in text and "348" in text and "100% of window" in text, text
    a = _ids(v)["T4.4"]
    assert a.measured["...as a multiple of body weight"] > 2.0
    assert a.measured["fraction of the scored window anything was applied"] \
        == pytest.approx(1.0)
    assert not a.vacuous, "the instrument worked; it measured a crane"


def test_the_wrench_never_fails_the_row_however_large_it_is():
    """The routing rule, isolated: the same walk, three support levels."""
    for fn in (fixtures_t4.oracle, fixtures_t4.g1_shaped_support,
               fixtures_t4.carried_outright):
        v = t4_core.grade(fn())
        assert _ids(v)["T4.4"].ok is True, fn.__name__
        assert v.outcome == PASS, fn.__name__
    cells = [t4_core.grade(fn()).measurements["cell"]
             for fn in (fixtures_t4.oracle, fixtures_t4.carried_outright)]
    assert cells == [t4ev.CELL_UNSUPPORTED, t4ev.CELL_SUPPORTED]


def test_touching_something_that_is_not_the_ground_fails_with_zero_force():
    """The half of T4.4 no force channel could ever see (reading 1)."""
    v = t4_core.grade(fixtures_t4.leaning_on_scene_geometry())
    assert v.failed == ["T4.4"]
    a = _ids(v)["T4.4"]
    assert a.measured["which bodies those were"] == ["wall"]
    assert a.measured["...of those, bodies the walking region is bounded "
                      "by"] == ["wall"]
    assert a.measured["peak applied force (N)"] == 0.0
    assert v.measurements["cell"] == t4ev.CELL_UNSUPPORTED


def test_a_contact_after_the_walk_ended_is_an_arena_finding_not_a_support_one():
    """The robot resting against whatever stopped it.

    The tier grades the walk. A contact that happens once the scored window
    has closed is counted, printed and kept out of the clause.
    """
    t, xyz, rot = fixtures_t4._walk()
    ev = fixtures_t4._evidence(
        t, xyz, rot, gait=fixtures_t4._contacts(t, foreign=(("wall", 40.0,
                                                             45.0),)))
    v = t4_core.grade(ev)
    a = _ids(v)["T4.4"]
    assert a.ok is True
    assert a.measured["contacts with a body that is not the ground, inside "
                      "the scored window"] == 0
    assert a.measured["the same after the walk had ended -- an arena finding, "
                      "not a support finding"] > 0


def test_an_adapter_cannot_relabel_the_thing_holding_the_robot_as_ground():
    """The task's names are authoritative, and the disagreement is printed."""
    t, xyz, rot = fixtures_t4._walk()
    gait = fixtures_t4._contacts(t)
    gait.contacts.append(t4ev.GroundContact(
        robot_body=fixtures_t4.BASE_NAME, other_body="gantry",
        other_is_ground=True, other_is_robot=False, t_s=10.0))
    v = t4_core.grade(fixtures_t4._evidence(t, xyz, rot, gait=gait))
    assert v.failed == ["T4.4"]
    a = _ids(v)["T4.4"]
    assert a.measured["which bodies those were"] == ["gantry"]
    assert a.measured["the adapter and the task disagreed about the ground"]


def test_an_unattestable_support_is_in_neither_cell_and_is_excluded():
    """§2 T4's own consequence, verbatim and executable.

    Not failed -- that would publish our missing channel as somebody else's
    capability gap (§4). Not credited -- that is what §5c exists to prevent.
    """
    v = t4_core.grade(fixtures_t4.support_not_attested())
    assert v.outcome == PASS
    assert v.failed == []
    assert v.measurements["cell"] == t4ev.CELL_UNVERIFIED
    assert v.measurements["cell"] not in (t4ev.CELL_SUPPORTED,
                                          t4ev.CELL_UNSUPPORTED)
    assert v.measurements["support_attestation"] == t4ev.UNVERIFIED
    assert v.measurements["excluded_from_comparison"] is True
    a = _ids(v)["T4.4"]
    assert sorted(f.clause for f in a.vacuous_clauses) == [
        "measured per channel", "what was applied to it was measured"]
    assert any("NEITHER published cell" in n for n in v.notes)


def test_a_wrench_with_no_body_weight_to_compare_it_against_is_unverified():
    """0.02 x m.g is not a boundary without m.g, and the row says so."""
    ev = fixtures_t4.oracle()
    ev.base_physics.mass_kg = None
    v = t4_core.grade(ev)
    assert v.measurements["cell"] == t4ev.CELL_UNVERIFIED
    assert v.measurements["excluded_from_comparison"] is True
    assert _ids(v)["T4.4"].vacuous
    assert v.outcome == PASS, "unverified is not a failure"


def test_half_a_wrench_is_not_a_wrench():
    """Force attested and torque not: the cell is decided by both."""
    t, xyz, rot = fixtures_t4._walk()
    support = fixtures_t4._support(t)
    support.torque = None
    v = t4_core.grade(fixtures_t4._evidence(t, xyz, rot, support=support))
    assert v.measurements["cell"] == t4ev.CELL_UNVERIFIED
    assert "one half of the wrench" in (
        v.measurements["external_support"]["error"] or "")


# --- per channel, and why -----------------------------------------------------


def test_the_support_profile_separates_an_idle_channel_from_a_live_one():
    """The measured reason the profile is per axis.

    On this tree's own rig, over a whole 10 m walk: the vertical carrying
    channel read 0.00 N and was live 0 % of the window while the attitude
    channel peaked at 69.2 N.m and was live 100 % of it. One scalar reports
    that as "held up, 100 % of the window" and hides the entire finding.
    """
    v = t4_core.grade(fixtures_t4.g1_shaped_support())
    per = _ids(v)["T4.4"].measured["per channel"]
    force, torque = per["force (N)"], per["torque (N.m)"]
    assert force["z"]["peak"] == 0.0
    assert force["z"]["fraction of the window it was non-zero"] == 0.0
    assert force["x"]["peak"] == 0.0
    assert force["y"]["peak"] == pytest.approx(fixtures_t4.G1_FY_N)
    assert force["y"]["fraction of the window it was non-zero"] == 1.0
    assert torque["x"]["peak"] == pytest.approx(fixtures_t4.G1_TX_NM)
    assert torque["x"]["fraction of the window it was non-zero"] == 1.0
    assert torque["z"]["peak"] == 0.0
    assert torque["z"]["fraction of the window it was non-zero"] == 0.0


def test_a_column_that_can_only_summarise_says_so_rather_than_pretending():
    v = t4_core.grade(fixtures_t4.summarised_support())
    assert v.outcome == PASS
    assert v.measurements["cell"] == t4ev.CELL_UNSUPPORTED
    a = _ids(v)["T4.4"]
    assert [f.clause for f in a.vacuous_clauses] == ["measured per channel"]
    assert a.measured["per channel"] is None
    assert any("per-axis series" in n for n in v.notes)


def test_the_profile_is_measured_over_the_walk_not_over_the_recording():
    """Reading 2: the published fraction has a stated denominator.

    Here the rig is live for the first half of the recording only, and the
    walk crosses the bar inside that half -- so the fraction over the walk is
    1.0 while the fraction over the whole log would be about a half.
    """
    t, xyz, rot = fixtures_t4._walk()
    ts = np.asarray(t)[::fixtures_t4.SUPPORT_STRIDE]
    live = (ts <= 20.0).astype(float)[:, None]
    support = t4ev.AppliedSupport(
        attested=True, t=ts,
        force=np.zeros((len(ts), 3)),
        torque=np.hstack([live * 50.0, np.zeros((len(ts), 2))]),
        source="a fixture whose rig switches off half way")
    v = t4_core.grade(fixtures_t4._evidence(t, xyz, rot, support=support))
    prof = v.measurements["external_support"]
    assert prof["measured over the walk rather than the whole recording"]
    assert prof["fraction of the window a torque was applied"] > 0.55
    whole = t4ev.support_profile(support, 334.5, force_fraction=0.02,
                                 torque_limit_nm=2.0)
    assert whole.torque_fraction_nonzero < 0.5
    assert whole.window_applied is False


def test_an_unwired_support_channel_and_a_measured_zero_are_different():
    """There is deliberately no "nothing was applied" default."""
    assert t4ev.EMPTY_APPLIED_SUPPORT.attested is None
    empty = t4ev.support_profile(t4ev.EMPTY_APPLIED_SUPPORT, 334.5,
                                 force_fraction=0.02, torque_limit_nm=2.0)
    assert empty.attested is False and empty.cell == t4ev.CELL_UNVERIFIED
    measured = t4ev.AppliedSupport(attested=True, peak_force_n=0.0,
                                   peak_torque_nm=0.0, fraction_nonzero=0.0,
                                   source="fixture")
    got = t4ev.support_profile(measured, 334.5, force_fraction=0.02,
                               torque_limit_nm=2.0)
    assert got.attested is True and got.cell == t4ev.CELL_UNSUPPORTED


def test_the_cell_boundary_is_the_plans_two_numbers():
    """0.02 x m.g and 2 N.m, either of which alone moves the cell."""
    weight = fixtures_t4.MASS_KG * fixtures_t4.G

    def cell(fz, tx):
        s = t4ev.AppliedSupport(attested=True, peak_force_n=fz,
                                peak_torque_nm=tx, fraction_nonzero=1.0,
                                source="fixture")
        return t4ev.support_profile(
            s, weight, force_fraction=t4_core.MAX_SUPPORT_FORCE_FRACTION,
            torque_limit_nm=t4_core.MAX_SUPPORT_TORQUE_NM).cell

    limit = t4_core.MAX_SUPPORT_FORCE_FRACTION * weight
    assert cell(limit, 2.0) == t4ev.CELL_UNSUPPORTED
    assert cell(limit * 1.001, 2.0) == t4ev.CELL_SUPPORTED
    assert cell(limit, 2.001) == t4ev.CELL_SUPPORTED


# --- the arena, and the run the world ended ----------------------------------


def test_a_run_bound_by_the_world_while_still_moving_is_seen_here_and_not_below():
    """Reading 5: this tier's one deliberate strengthening, asserted.

    The measured shape of the failure the rule exists for: the robot is
    pressed against the world and still locomoting sideways. A rule keyed on
    "the base stopped" -- which is the rule one rung down -- does not fire.
    """
    ev = fixtures_t4.arena_bound_still_moving()
    t = np.asarray(ev.base_pose.t, dtype=float)
    xy = np.asarray(ev.base_pose.xyz, dtype=float)[:, :2]
    assert t3_core.arena_stop_index(t, xy, ev.arena) is None
    idx = t4_core.arena_bound_index(t, xy, ev.arena)
    assert idx is not None
    v = t4_core.grade(ev)
    assert v.failed == ["T4.1"], "the distance, and nothing else"
    assert v.measurements["termination_cause"] == t4ev.TERMINATION_ARENA
    assert "still moving" in v.measurements["termination"]["why"]
    assert any("world, not the walker" in n for n in v.notes)


def test_a_run_the_world_ended_is_recorded_as_such_not_as_a_bad_gait():
    """g1-endurance-2026-08-01.md §8, made executable."""
    v = t4_core.grade(fixtures_t4.arena_bound())
    assert v.failed == ["T4.1"]
    assert v.measurements["termination"]["how the run ended"] == (
        t4ev.TERMINATION_ARENA)
    assert v.measurements["arena"]["the run became bound by the region's "
                                   "edge"] is True
    assert v.measurements["arena"]["the run-up requirement was met"] is False
    a = _ids(v)["T4.1"]
    assert [f.clause for f in a.vacuous_clauses] == ["crossed ten metres"]
    assert any("geometry-bound by construction" in n for n in v.notes)


def test_a_plateau_in_open_floor_is_a_stall_and_is_scored_in_full():
    """The guard: only a positively-attested boundary truncates the window."""
    t, xyz, rot = fixtures_t4._walk(stop_at=15.0)
    v = t4_core.grade(fixtures_t4._evidence(t, xyz, rot))   # the BIG arena
    assert v.measurements["termination_cause"] == (
        t4ev.TERMINATION_CONTROLLER)
    a = _ids(v)["T4.3"]
    assert a.measured["the scored window is (s)"] > 35.0
    assert a.measured["mean speed made good over the scored window (m/s)"] < (
        t4_core.MIN_MEAN_SPEED_MPS)
    assert a.ok is False


def test_a_fall_is_named_as_the_termination_even_next_to_a_wall():
    v = t4_core.grade(fixtures_t4.fell_height())
    assert v.measurements["termination_cause"] == t4ev.TERMINATION_FELL
    assert "t = 40" in v.measurements["termination"]["why"]


def test_with_no_arena_channel_the_cell_is_incomplete_and_says_so():
    """§2 T4 build note 1, as this file reads it (declared in meta.json)."""
    v = t4_core.grade(fixtures_t4.no_arena_channel())
    assert v.outcome == PASS
    assert v.measurements["arena_attestation"] == t4ev.UNVERIFIED
    assert v.measurements["excluded_from_comparison"] is True
    assert any("walking region" in r
               for r in v.measurements["comparison_exclusions"])
    assert v.measurements["arena"]["the run-up requirement was met"] is None
    assert v.measurements["termination_cause"] != t4ev.TERMINATION_ARENA
    assert any("INCOMPLETE" in n for n in v.notes)


def test_an_attested_arena_and_an_attested_wrench_are_two_exclusions():
    v = t4_core.grade(fixtures_t4.oracle())
    assert v.measurements["excluded_from_comparison"] is False
    assert v.measurements["comparison_exclusions"] == []
    ev = fixtures_t4.no_arena_channel()
    ev.support = t4ev.EMPTY_APPLIED_SUPPORT
    v = t4_core.grade(ev)
    assert len(v.measurements["comparison_exclusions"]) == 2


def test_the_stated_free_run_up_is_the_plans_multiple_of_the_bar():
    v = t4_core.grade(fixtures_t4.oracle())
    assert v.measurements["arena"]["the free run-up this task requires (m)"] \
        == pytest.approx(t4_core.MIN_RUN_UP_FACTOR * t4_core.NET_DISPLACEMENT_M)
    assert v.measurements["arena"]["the run-up requirement was met"] is True


# --- the headroom fields ------------------------------------------------------


def test_the_two_headroom_fields_use_the_plans_own_vocabulary():
    """§2 T4 and §3.2. Neither is a pass condition."""
    assert set(t4ev.TERMINATIONS) == {"fell", "arena_geometry", "time_limit",
                                      "controller_stopped", "unknown"}
    v = t4_core.grade(fixtures_t4.oracle())
    assert v.measurements["termination_cause"] in t4ev.TERMINATIONS
    assert v.measurements["distance_to_termination_m"] == pytest.approx(
        13.5, abs=0.05)


def test_the_distance_to_termination_is_not_clipped_at_the_bar():
    """A 10.1 m and a 29.5 m are distinguishable in the published grid."""
    short = t4_core.grade(fixtures_t4.stopped_short())
    long_ = t4_core.grade(fixtures_t4.oracle())
    assert short.measurements["distance_to_termination_m"] < 10.0
    assert long_.measurements["distance_to_termination_m"] > 10.0
    assert short.measurements["termination_cause"] == (
        t4ev.TERMINATION_TIME_LIMIT)


def test_the_labels_reach_the_row_even_when_nothing_else_does():
    """A cell that failed before any physics was read is still a T4 cell."""
    ev = fixtures_t4.oracle()
    ev.bundle.artifact = None
    v = t4_core.grade(ev)
    assert v.outcome == FAIL and len(v.failed) == 5
    for key in ("cell", "cell_text", "method", "reuse_class",
                "support_attestation", "external_support",
                "arena_attestation", "excluded_from_comparison",
                "distance_to_termination_m", "termination_cause"):
        assert key in v.measurements, key
    assert v.measurements["distance_to_termination_m"] is None
    assert v.measurements["termination_cause"] == t4ev.TERMINATION_UNKNOWN


def test_the_cell_in_the_measurements_is_the_one_the_row_graded():
    """One profile, computed once over the walk, published in both places."""
    for name in ("oracle", "g1_shaped_support", "carried_outright",
                 "support_not_attested", "summarised_support"):
        v = t4_core.grade(fixtures_t4.FIXTURE_ASSERTION_MAP[name]["fn"]())
        row = _ids(v)["T4.4"].measured
        assert row["the cell this run is published in"] == (
            v.measurements["cell"]), name
        assert row["the cell, as it must be printed"] == (
            v.measurements["cell_text"]), name
        assert v.measurements["external_support"]["cell"] == (
            v.measurements["cell"]), name


# --- the finding --------------------------------------------------------------


def test_our_own_flagships_measured_speed_is_red_on_this_tier():
    """NOT a test of the grader. A finding, kept executable.

    docs/developer/g1-endurance-2026-08-01.md: the shipped flagship crosses
    10.0 m at t = 82.88 s, six of six runs, zero falls -- 0.1207 m/s made good
    -- on the lambda = 0.9 weight-bearing balance rig. This tier inherits a
    0.15 m/s floor from the rung below, unchanged, so that artifact is RED on
    the speed clause while being GREEN on the distance the plan measured
    before the freeze.

    No threshold was moved. The question is recorded in the task's meta.json
    for whoever owns the tier: is a speed floor written for a four-legged
    robot the floor that was meant for a two-legged one?
    """
    v = t4_core.grade(fixtures_t4.flagship_measured_speed())
    assert v.failed == ["T4.3"]
    a = _ids(v)["T4.3"]
    speed = a.measured["mean speed made good over the scored window (m/s)"]
    assert speed == pytest.approx(fixtures_t4.G1_SPEED_MPS, abs=0.002)
    assert speed < t4_core.MIN_MEAN_SPEED_MPS
    # ...and it is simultaneously a supported cell, which is the shape our own
    # column's cell would take if this tier ran today
    assert v.measurements["cell"] == t4ev.CELL_SUPPORTED
    assert _ids(v)["T4.1"].ok is True, "the distance is not the problem"
    assert _ids(v)["T4.2"].ok is True, "it never fell"


def test_the_task_records_that_finding_rather_than_moving_the_number():
    readings = tasks.get(t4_core.TASK).meta["grading_readings"]
    note = readings[
        "T4.3_OPEN_QUESTION_our_own_flagship_is_below_the_speed_floor"]
    assert note.startswith("OPEN AT THE FREEZE")
    assert "NO THRESHOLD WAS MOVED" in note
    assert "0.1207" in note and "0.15" in note
    assert "balance harness" in note


# --- the method: recorded, never graded --------------------------------------


@pytest.mark.parametrize("method", t4ev.METHODS)
def test_the_method_is_recorded_and_never_graded(method):
    """"A cell that reaches the outcome by training a policy and a cell that
    reaches it by a model-based gait are both achieved, with method recorded."
    """
    ev = fixtures_t4.oracle()
    ev.controller.declared_method = method
    v = t4_core.grade(ev)
    baseline = t4_core.grade(fixtures_t4.oracle())
    assert [(a.id, a.ok) for a in v.assertions] == [
        (a.id, a.ok) for a in baseline.assertions]
    assert v.measurements["method"] == method


def test_an_unrecognised_method_is_recorded_as_unknown_not_dropped():
    ev = fixtures_t4.oracle()
    ev.controller.declared_method = "evolutionary, obviously"
    v = t4_core.grade(ev)
    assert v.measurements["method"] == "unknown"
    assert v.measurements["controller"]["declared_as"] == (
        "evolutionary, obviously")


def test_a_hand_written_gait_passes_with_its_method_printed():
    v = t4_core.grade(fixtures_t4.scripted_controller_loaded())
    assert v.outcome == PASS
    assert v.measurements["method"] == "scripted"


# --- reuse_class: the grader refuses to decide it ----------------------------


def test_reuse_class_is_carried_null_and_loudly():
    """§9 Q3: the boundary is fuzzy and the reviewer arbitrates."""
    v = t4_core.grade(fixtures_t4.oracle())
    assert "reuse_class" in v.measurements
    assert v.measurements["reuse_class"] is None
    assert any("reuse_class" in n and "not publishable" in n for n in v.notes)
    assert v.as_dict()["measurements"]["reuse_class"] is None


# --- T4.5's controller-load clause -------------------------------------------


def test_a_controller_that_never_loaded_fails_on_a_clean_exit_code():
    v = t4_core.grade(fixtures_t4.policy_never_loaded())
    assert v.failed == ["T4.5"]
    a = _ids(v)["T4.5"]
    assert a.measured["exit code"] == 0
    assert a.measured["error-class lines"] == 0
    assert a.measured["reached finalize"] is True
    assert a.measured[
        "the policy or controller is attested to have loaded"] is False
    assert not a.vacuous, "the attestation exists; it says no"


def test_missing_attribution_is_invalid_not_failed():
    v = t4_core.grade(fixtures_t4.missing_attribution())
    assert v.outcome == INVALID
    assert "T4.5" in v.failed
    assert any("unattributed" in n for n in v.notes)


# --- the gait clause, on two legs --------------------------------------------


def test_the_aggregate_contact_signal_of_a_perfect_walk_never_changes():
    """Why the count is per body, on a robot with only two of them.

    The oracle alternates: one foot is down at every sample. Counted in
    aggregate a perfect walk scores ZERO transitions; counted per foot it
    scores hundreds.
    """
    ev = fixtures_t4.oracle()
    ground, _foreign, _d = t4ev.classify_contacts(ev.gait, ev.surface)
    ts, bodies, state = t4ev.contact_states(ev.gait, ground)
    assert len(bodies) == 2
    aggregate = state.any(axis=1)
    assert aggregate.all(), "some foot is down at every sample"
    assert int(np.count_nonzero(np.diff(aggregate.astype(np.int8)))) == 0
    per_body, cycles, _per = t3_core.contact_transitions(state)
    assert per_body > 100 and cycles > 40


def test_a_body_that_slides_fails_on_the_footfalls_it_never_made():
    v = t4_core.grade(fixtures_t4.slid_without_gait())
    assert v.failed == ["T4.3"]
    a = _ids(v)["T4.3"]
    assert a.measured["make-and-break transitions of ground contact"] == 0
    assert a.measured["vertical oscillation about its trend, RMS (m)"] < (
        t4_core.MIN_BOB_RMS_M)
    assert not a.vacuous, "the contacts were reported; they never broke"


def test_the_bob_is_inherited_exactly_as_the_tier_below_leaves_it():
    """"Unchanged" cuts both ways, and this is the case that proves it.

    The rung below retired its vertical-oscillation clause on 2026-08-02 --
    still measured, still printed, never graded -- because it was demonstrated
    failing a robot that was plainly walking. This tier CALLS that row, so it
    inherited the repair without a line being written here. If the row is ever
    changed back, this tier changes back with it, which is what the plan's
    word "unchanged" has to mean.
    """
    ev = fixtures_t4.oracle()
    ev.base_pose.xyz = np.array(ev.base_pose.xyz)
    ev.base_pose.xyz[:, 2] = fixtures_t4.STAND_Z        # a perfectly flat base
    v = t4_core.grade(ev)
    a = _ids(v)["T4.3"]
    assert a.measured["vertical oscillation about its trend, RMS (m)"] \
        == pytest.approx(0.0, abs=1e-9)
    assert a.ok is True, ("the bob is measured and not graded one rung down, "
                          "so it is measured and not graded here")
    assert v.outcome == PASS


# --- absent evidence ---------------------------------------------------------


def test_no_deliverable_scores_zero_of_five():
    ev = fixtures_t4.oracle()
    ev.bundle.artifact = None
    v = t4_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert v.progress == 0


def test_a_missing_ground_declaration_is_invalid_not_failed():
    """The names are task data. Losing one is a broken instrument."""
    ev = fixtures_t4.oracle()
    ev.surface = t4ev.WalkingSurface()
    v = t4_core.grade(ev)
    assert v.outcome == INVALID
    assert v.outcome != FAIL
    assert any("walking surface" in n or "ground the robot walks on" in n
               for n in v.notes)


def test_a_series_about_the_wrong_body_is_refused_rather_than_graded():
    ev = fixtures_t4.oracle()
    ev.base_pose.body = "some_other_body"
    v = t4_core.grade(ev)
    assert v.outcome == FAIL
    assert len(v.failed) == 5
    assert "the task names" in _ids(v)["T4.1"].detail


# --- the ladder's own evidence channels --------------------------------------


def test_the_evidence_contract_names_the_usual_adapter_mistakes():
    ev = fixtures_t4.oracle()
    ev.surface.source = ""
    ev.base_pose.rot = None
    ev.standing.z_m = None
    ev.support = t4ev.EMPTY_APPLIED_SUPPORT
    ev.world.gravity_mps2 = None
    ev.controller = t4ev.EMPTY_CONTROLLER_LOAD
    ev.arena = t4ev.EMPTY_ARENA
    problems = t4ev.check_t4_evidence(ev)
    assert any("source citation" in p for p in problems)
    assert any("no orientation" in p for p in problems)
    assert any("standing height" in p for p in problems)
    assert any("neither published cell" in p for p in problems)
    assert any("body weight" in p for p in problems)
    assert any("controller load" in p for p in problems)
    assert any("arena" in p for p in problems)


def test_a_summarised_wrench_is_flagged_by_the_contract_too():
    ev = fixtures_t4.summarised_support()
    problems = t4ev.check_t4_evidence(ev)
    assert any("summarised rather than streamed" in p for p in problems)


def test_every_channel_gap_names_the_assertions_it_breaks():
    for channel, assertions in t4ev.T4_CHANNEL_ASSERTIONS.items():
        for aid in assertions:
            assert aid in fixtures_t4.ASSERTIONS, (channel, aid)
    # the arena breaks NOTHING: it is a finding, never a pass condition
    assert t4ev.T4_CHANNEL_ASSERTIONS["arena"] == ()


def test_the_ladder_declares_what_a_new_column_must_add_for_T4():
    keys = {k for k, _why in t4ev.LADDER_REQUIRED_EVIDENCE_T4}
    assert keys == {"walking_surface", "base_pose", "standing_height",
                    "gait_contacts", "applied_support", "base_mass",
                    "controller_load", "arena"}
    for _k, why in t4ev.LADDER_REQUIRED_EVIDENCE_T4:
        assert why
    per_axis = dict(t4ev.LADDER_REQUIRED_EVIDENCE_T4)["applied_support"]
    assert "PER" in per_axis and "AXIS" in per_axis


def test_the_evidence_is_the_tier_belows_evidence_with_no_field_added():
    """The claim T4Evidence makes about itself, checked.

    "T4's channels are T3's channels" is only worth writing if nothing
    quietly grew a field.
    """
    import dataclasses
    mine = {f.name for f in dataclasses.fields(t4ev.T4Evidence)}
    theirs = {f.name for f in dataclasses.fields(t4ev.T3Evidence)}
    assert mine == theirs
    assert issubclass(t4ev.T4Evidence, t4ev.T3Evidence)


# --- the shim over the ladder's adapters -------------------------------------


class _StubColumn:
    """A column that produces a bundle and none of T4's channels."""

    def __init__(self, bundle):
        self.bundle = bundle

    def build_bundle(self, *_a, **_kw):
        return self.bundle


class _T3OnlyColumn:
    """A column whose ladder module has only the tier-below channel hook."""

    def __init__(self, channels):
        self._channels = channels

    def t3_channels(self, _where, surface=None):
        return dict(self._channels)


def test_a_column_with_no_t4_channels_falls_back_and_names_every_gap(
        monkeypatch):
    """The whole point of the gap machinery: the blocker must land on us."""
    bundle = fixtures_t4.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = t4.build_evidence(t4_core.TASK, sim="stub")
    gaps = t4ev.unanswered_channels(ev)
    assert set(gaps) >= {"base_pose", "gait_contacts", "applied_support",
                         "base_mass", "gravity", "controller_load"}
    # ...and the two the frozen contract CAN answer are not in the gap list:
    # the settled standing height is the base's own z at t=0, and the walking
    # region is the union of the ground bodies' world boxes
    assert "arena" not in gaps
    assert "standing_height" not in gaps
    assert any("scaffolding_defect_ours" in n for n in ev.notes)


def test_a_column_with_only_the_tier_belows_hook_is_used_and_the_note_says_so(
        monkeypatch):
    """Legitimate reuse, recorded rather than silent."""
    ref = fixtures_t4.oracle()
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(ref.bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: _T3OnlyColumn({
                            "base_pose": ref.base_pose,
                            "standing": ref.standing, "gait": ref.gait,
                            "support": ref.support, "arena": ref.arena,
                            "base_physics": ref.base_physics,
                            "world": ref.world,
                            "controller": ref.controller}))
    ev = t4.build_evidence(t4_core.TASK, sim="stub")
    assert t4ev.unanswered_channels(ev) == {}
    assert any("supplies no T4 channel builder" in n for n in ev.notes)
    assert t4_core.grade(ev).outcome == PASS


def test_the_ground_names_come_from_the_task_file_with_its_citation(
        monkeypatch):
    bundle = fixtures_t4.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    ev = t4.build_evidence(t4_core.TASK, sim="stub")
    assert "ground" in ev.surface.names
    assert "meta.json" in ev.surface.source
    assert ev.robot_name == "base_link"


def test_the_grade_entry_point_records_the_unanswered_channels(monkeypatch):
    bundle = fixtures_t4.oracle().bundle
    monkeypatch.setattr(adapters, "resolve",
                        lambda sim=None: _StubColumn(bundle))
    monkeypatch.setattr(adapters, "resolve_ladder_channels",
                        lambda sim=None: None)
    v = t4.grade(None, sim="stub")
    assert "unanswered_channels" in v.measurements
    assert any("scaffolding_defect_ours" in n for n in v.notes)


# --- the task assets ---------------------------------------------------------


def test_the_prompt_is_one_sentence():
    task = tasks.get(t4_core.TASK)
    assert task.prompt.count(".") == 1
    assert task.prompt.endswith(".")
    assert "\n" not in task.prompt


# §5a's outcome-not-feature rule: "a tier statement may not name a file format,
# an endpoint, a node type, a solver, a training method, a checkpoint, or any
# product's proper noun." The last group is this tier's own: neither the
# technique nor the rig may appear in what the agent is asked for.
_FORBIDDEN_IN_PROMPT = (
    "urdf", "wbt", "sdf", "usd", "mjcf", "xacro", "xml", "json", "yaml",
    "omnisim", "webots", "gazebo", "isaac", "mujoco", "genesis", "newton",
    "ode", "solver", "supervisor", "controller", "harness", "proto", "node",
    "endpoint", "http", "api", "checkpoint", "policy", "reinforcement",
    "import", "train", "training", "gait", "support", "crane", "rig",
    "balance", "reward")


def test_the_prompt_names_no_format_no_endpoint_no_method_and_no_rig():
    task = tasks.get(t4_core.TASK)
    low = task.prompt.lower()
    for token in _FORBIDDEN_IN_PROMPT:
        assert not re.search(r"\b%s\b" % re.escape(token), low), token


def test_the_prompt_states_the_outcome_and_not_the_method():
    task = tasks.get(t4_core.TASK)
    low = task.prompt.lower()
    for word in ("two-legged", "ten metres", "without falling"):
        assert word in low


def test_the_container_ships_the_description_the_arena_rule_and_nothing_else():
    task = tasks.get(t4_core.TASK)
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
    task = tasks.get(t4_core.TASK)
    descriptions = [p for p in task.container_files if p.suffix == ".urdf"]
    assert len(descriptions) == 1
    text = descriptions[0].read_text(encoding="utf-8").lower()
    for word in ("trajectory", "waypoint", "gait", "policy", "controller",
                 "checkpoint"):
        assert word not in text, word


def test_the_container_states_the_run_up_requirement_to_the_agent():
    """§2 T4 build note 2: "stated in the task". The prompt cannot carry it."""
    task = tasks.get(t4_core.TASK)
    arena = [p for p in task.container_files if p.name == "ARENA.txt"]
    assert len(arena) == 1
    text = arena[0].read_text(encoding="utf-8")
    assert "15 METRES" in text.upper()
    assert str(int(task.meta["arena"]["min_free_run_up_m"])) in text
    low = text.lower()
    # it states a constraint on the WORLD, and not a technique
    for word in ("harness", "crane", "support", "rig", "policy", "train"):
        assert not re.search(r"\b%s\b" % word, low), word


def test_the_shipped_description_parses_and_references_no_outside_file():
    """A malformed description would fail every column for OUR reason."""
    task = tasks.get(t4_core.TASK)
    urdf = [p for p in task.container_files if p.suffix == ".urdf"][0]
    root = ET.parse(urdf).getroot()
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


def test_the_robot_is_a_biped_with_twelve_driven_joints_and_no_welds():
    """The structural claims the container makes about itself, checked."""
    task = tasks.get(t4_core.TASK)
    urdf = [p for p in task.container_files if p.suffix == ".urdf"][0]
    root = ET.parse(urdf).getroot()
    joints = root.findall("joint")
    assert len(root.findall("link")) == 13
    assert len(joints) == 12
    assert all(j.get("type") == "revolute" for j in joints), \
        "a fixed joint would let an importer fuse away a named foot body"
    for leg in ("l", "r"):
        for pre, axis in (("hip_yaw", "0 0 1"), ("hip_roll", "1 0 0"),
                          ("hip_pitch", "0 1 0"), ("knee", "0 1 0"),
                          ("ankle_pitch", "0 1 0"), ("ankle_roll", "1 0 0")):
            j = [x for x in joints if x.get("name") == "%s_%s_joint"
                 % (pre, leg)]
            assert len(j) == 1, (pre, leg)
            assert j[0].find("axis").get("xyz") == axis
            lim = j[0].find("limit")
            assert float(lim.get("effort")) > 0.0
            assert float(lim.get("upper")) > float(lim.get("lower"))
        # every knee is bent: an upper limit at or above zero admits a
        # straight leg, which is the two-link solution's own singularity
        knee = [x for x in joints if x.get("name") == "knee_%s_joint" % leg][0]
        assert float(knee.find("limit").get("upper")) < 0.0
    # ...and the two feet are the leaves, so they survive an importer that
    # fuses welded bodies -- there are none to fuse
    children = {j.find("child").get("link") for j in joints}
    parents = {j.find("parent").get("link") for j in joints}
    assert (children - parents) == {"foot_l", "foot_r"}


def test_the_declared_mass_matches_what_the_task_file_says():
    task = tasks.get(t4_core.TASK)
    urdf = [p for p in task.container_files if p.suffix == ".urdf"][0]
    total = sum(float(m.get("value")) for m in ET.parse(urdf).iter("mass"))
    assert total == pytest.approx(task.meta["robot"]["mass_kg_declared"])


def test_the_task_declares_which_bodies_are_the_ground_and_why_it_is_task_data():
    task = tasks.get(t4_core.TASK)
    ground = task.meta["ground"]
    assert "ground" in ground["names"] and len(ground["names"]) >= 4
    assert "could label the thing holding the robot up" in \
        ground["why_not_in_the_container"]
    assert ground["open_question_for_the_freeze"]


def test_the_task_states_a_free_run_up_of_at_least_one_and_a_half_bars():
    """The build note g1-endurance-2026-08-01.md §8 requires of every column."""
    task = tasks.get(t4_core.TASK)
    arena = task.meta["arena"]
    assert arena["min_free_run_up_m"] == pytest.approx(
        t4_core.MIN_RUN_UP_FACTOR * t4_core.NET_DISPLACEMENT_M)
    assert arena["min_free_run_up_m"] >= 1.5 * t4_core.NET_DISPLACEMENT_M
    assert arena["why"] and arena["the_guard_on_that_last_one"]
    assert arena["how_this_tier_differs_from_the_one_below"]


def test_the_task_declares_the_two_cells_and_their_boundary():
    task = tasks.get(t4_core.TASK)
    cells = task.meta["support"]["the_two_cells"]
    assert set(cells) == {t4ev.CELL_UNSUPPORTED, t4ev.CELL_SUPPORTED,
                          t4ev.CELL_UNVERIFIED}
    assert "MAX_SUPPORT_FORCE_FRACTION" in cells[t4ev.CELL_UNSUPPORTED]
    for piece in ("multiple of body weight", "peak applied torque",
                  "fraction of the walk window"):
        assert piece in cells[t4ev.CELL_SUPPORTED], piece
    assert "NOT ONE OF THE PLAN'S TWO CELLS" in cells[t4ev.CELL_UNVERIFIED]
    assert task.meta["support"]["two_cells_from_ONE_recording"]
    assert task.meta["support"]["the_wrench_never_fails_the_row"]


def test_the_task_declares_the_per_channel_requirement_and_its_measurement():
    block = tasks.get(t4_core.TASK).meta["support"]["measured_per_channel"]
    assert "0.00 N" in block["why"] and "69.2 N.m" in block["why"]
    assert "0 % OF THE WINDOW" in block["why"].upper()
    assert block["a_column_that_can_only_summarise"]
    assert block["and_what_it_does_not_prove"]


def test_the_task_carries_the_humanoid_disclosure_rule():
    """AGENTS.md's rule, in the file a reader of a T4 cell opens."""
    meta = tasks.get(t4_core.TASK).meta
    d = meta["disclosure"]
    assert "free-standing" in d["the_rule"]
    assert "AGENTS.md" in d["the_rule"]
    assert d["what_the_grader_can_and_cannot_do_about_it"]
    blob = json.dumps(meta)
    assert "balance harness" in blob or "balance rig" in blob


def test_the_task_declares_the_method_is_recorded_and_not_graded():
    block = tasks.get(t4_core.TASK).meta["method"]
    assert block["graded"] is False
    assert set(block["values"]) == set(t4ev.METHODS)
    assert block["why_not"] and block["recorded_where"]


def test_the_task_says_the_grader_cannot_decide_reuse_class():
    block = tasks.get(t4_core.TASK).meta["reuse_class"]
    assert "reviewer" in block["decided_by"]
    assert block["why_the_grader_leaves_it_null"]


def test_the_task_flags_every_reading_that_goes_beyond_the_plans_text():
    """Declared in the file a reviewer opens, not buried in a grader."""
    readings = tasks.get(t4_core.TASK).meta["grading_readings"]
    contact = readings["T4.4_the_contact_half_survives_and_that_is_a_READING"]
    assert contact.startswith("DECLARED, NOT TRANSCRIBED")
    arena = readings["T4_an_unattested_arena_excludes_the_cell_from_comparison"]
    assert "EXTENSION OF THE PLAN'S TEXT" in arena
    assert readings["T4_arena_bound_does_not_require_a_stop"].startswith(
        "THIS TIER STRENGTHENS")


def test_the_task_records_the_open_questions_it_found_rather_than_hiding_them():
    """§5c: a gap a reviewer should see BEFORE the freeze, not after a grid."""
    readings = tasks.get(t4_core.TASK).meta["grading_readings"]
    for key in ("T4.3_OPEN_QUESTION_our_own_flagship_is_below_the_speed_floor",
                "T4_OPEN_QUESTION_a_supported_cell_has_no_ceiling",
                "T4_OPEN_QUESTION_no_clause_reads_the_terrain",
                "T4_OPEN_QUESTION_a_run_that_walks_and_then_stops_is_not_penalised"):
        assert readings[key].startswith("OPEN AT THE FREEZE"), key
    # ...and the one this tier inherits already-repaired, declared as the text
    # divergence it is rather than as a question nobody answered
    bob = readings["T4.3_the_bob_clause_is_INHERITED_AS_RETIRED"]
    assert bob.startswith("INHERITED DIVERGENCE")
    assert "0.0031" in bob, "this tier's own demonstration is the confirmation"


def test_the_task_records_honestly_whether_the_robot_has_ever_walked():
    """§5c: the tier is not publishable until an oracle proves achievability.

    The record must say WHICH column, and it must not let a result on a
    different robot stand in for one on this description.
    """
    block = tasks.get(t4_core.TASK).meta["container"]["authored_here"]
    assert block["demonstrated"].startswith(("YES", "NO", "NOT"))
    assert block["what_our_own_evidence_does_and_does_not_say"]
    assert "DIFFERENT ROBOT" in block[
        "what_our_own_evidence_does_and_does_not_say"]
    assert len(block["what_a_demonstration_must_produce"]) >= 4
    # a demonstration is a claim a reader must be able to re-derive -- and
    # since 2026-08-02 it is re-derivable: the scratch script became a
    # committed oracle, so the record asserts the entry point, not the gap.
    assert block["the_demonstration_IS_a_committed_oracle"].startswith("YES")
    assert block["the_recipe_the_demonstration_used"]
    assert len(block["what_the_demonstration_found_about_THIS_ASSET"]) >= 3
    # ...and it must name BOTH cells, because it reached one and not the other
    assert t4ev.CELL_SUPPORTED in block["demonstrated"]
    assert block["and_the_unsupported_cell_was_attempted_and_FELL"]


def test_the_task_declares_its_pre_registered_expectation():
    """§5c.1: written before any cell runs, printed beside the outcome."""
    task = tasks.get(t4_core.TASK)
    exp = task.meta["expectation"]
    assert exp["T4-unsupported"].startswith("not_achieved")
    assert exp["T4-supported"].startswith("achieved")
    assert "assembled" in exp["T4-supported"]
    assert exp["reasoning"] and exp["pre_registered"]
    assert exp["what_this_file_adds_without_changing_the_pre_registration"]
    assert task.meta["scoring_class"] == "exploratory"
    assert task.repeats_default == 3


def test_the_task_meta_is_valid_json_and_names_its_grader():
    task = tasks.get(t4_core.TASK)
    assert task.meta["grader"] == "ladder.graders.t4"
    json.dumps(task.meta)          # round-trips
    assert task.rung == "T4"
    # the recorded window must fit ten metres at the tier's own speed floor,
    # AND the only long humanoid walk this tree has measured (82.9 s to the
    # bar), with room for a slower column to be measured rather than truncated
    assert task.standalone["duration_s"] >= (t4_core.NET_DISPLACEMENT_M
                                             / t4_core.MIN_MEAN_SPEED_MPS)
    assert task.standalone["duration_s"] >= 2.0 * 82.9
    assert set(task.surfaces) == set(task.meta["ground"]["names"])


# --- the structural guard, replicated ----------------------------------------
#
# ``ladder/graders/test_neutral_core.py`` walks the AST of every neutral core
# and refuses any simulator-specific token in CODE (docstrings are exempt on
# purpose: explaining why a boundary exists requires naming what is on the
# other side of it). T4's two cores are not in its NEUTRAL_MODULES tuple yet --
# adding them is a change to a file this work is not permitted to edit -- so
# the guard is replicated here in the meantime, exactly as the tier below did
# it. A guard that only covers the tiers that existed when it was written
# stops being a guard the moment the ladder grows.

_SHARED_GUARD = importlib.import_module("ladder.graders.test_neutral_core")
T4_NEUTRAL_MODULES = (t4_core, t4ev)


@pytest.mark.parametrize("mod", T4_NEUTRAL_MODULES)
def test_the_T4_cores_contain_no_simulator_specific_vocabulary(mod):
    for text in _SHARED_GUARD._code_strings_and_names(mod.__file__):
        low = text.lower()
        for token in _SHARED_GUARD.SIM_TOKENS:
            assert token not in low, (
                "%s mentions %r in code (not a docstring): %r"
                % (Path(mod.__file__).name, token, text))


@pytest.mark.parametrize("mod", T4_NEUTRAL_MODULES)
def test_the_T4_cores_import_no_simulator_module(mod):
    """Checked on the import graph, not the vocabulary.

    Note that ``t4.py`` -- the task entry point -- is NOT a neutral core and
    does import the adapters, exactly as its three siblings do.
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


def test_the_shared_guard_should_also_list_T4s_modules():
    """A reminder with a payload, not a landmine.

    Skips (rather than fails) while the shared tuple is missing T4, because
    the file that owns it is outside this work's scope and a hard failure
    would break somebody else's suite. The replicated guard above means the
    modules ARE checked either way; this test exists so the duplication gets
    removed rather than forgotten.
    """
    missing = [m.__name__ for m in T4_NEUTRAL_MODULES
               if m not in _SHARED_GUARD.NEUTRAL_MODULES]
    if missing:
        pytest.skip(
            "ladder/graders/test_neutral_core.py NEUTRAL_MODULES does not yet "
            "list %s -- add them (and import them) there, then delete the "
            "replicated guard in this file" % ", ".join(missing))
