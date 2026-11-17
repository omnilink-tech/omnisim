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

"""R2 arm_reach -- unit tests for the sim-neutral core.

These never launch a simulator. They build ``graders.evidence`` structures
directly, which is the point: if an assertion needs a simulator to be exercised,
it is not sim-neutral.

The scenario builders at the bottom (``good_bundle``, ``static_arm_bundle``,
``teleport_bundle``, ...) are shared with ``test_r2_discriminates.py``, which
uses them to prove the task cannot be passed by doing nothing. They live here so
there is one definition of "a clean R2 run" and it is the one both files score.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders import r2_core  # noqa: E402
from agentbench.graders.evidence import (Body, BodyInventory,  # noqa: E402
                                         EngineAttribution, EvidenceBundle,
                                         ProcessFacts, Trajectory)

TASK_DIR = (Path(__file__).resolve().parents[1] / "tasks" / "R2_arm_reach")
META = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))


# =============================================================================
# scenario builders -- shared with test_r2_discriminates.py
# =============================================================================

DT = 0.016
HOME = (0.0, 0.0, 0.10)


def reach_track(order=(0, 1, 2), dwell_s=0.8, dt=DT, approach_s=1.5,
                home=HOME):
    """A synthetic tip track in the BASE frame: fly to a target, hold, repeat.

    Returns ``(t, rel)``. ``order`` indexes ``TARGETS_BASE_FRAME``, so
    ``(2, 1, 0)`` is the same three poses visited backwards -- which is how the
    ordering clause is tested without inventing new coordinates.
    """
    cur = np.asarray(home, dtype=float)
    pts = [cur.copy()]                      # the track starts AT home
    for i in order:
        tgt = np.asarray(r2_core.TARGETS_BASE_FRAME[i], dtype=float)
        n = max(2, int(round(approach_s / dt)))
        for k in range(1, n + 1):
            pts.append(cur + (tgt - cur) * (k / float(n)))
        for _ in range(max(1, int(round(dwell_s / dt)))):
            pts.append(tgt.copy())
        cur = tgt
    rel = np.asarray(pts, dtype=float)
    return np.arange(len(rel), dtype=float) * dt, rel


def teleport_track(dwell_s=0.8, dt=DT, hold_home_s=0.5, home=HOME):
    """The cheat: no transit at all -- the tip is simply AT each target."""
    pts = [np.asarray(home, dtype=float)] * max(1, int(round(hold_home_s / dt)))
    for tgt in r2_core.TARGETS_BASE_FRAME:
        pts += [np.asarray(tgt, dtype=float)] * max(
            1, int(round(dwell_s / dt)))
    rel = np.asarray(pts, dtype=float)
    return np.arange(len(rel), dtype=float) * dt, rel


def bundle_from_tracks(t, tracks, *, joints=(6,), robot_class=(True,),
                       artifact="arm_reach.wbt", clean=True, attributed=True,
                       aabb_min=None, notes=()):
    """One ``EvidenceBundle`` from world-frame per-body tracks.

    ``tracks[0]`` is the arm; the rest are whatever else the recorder sampled.
    Roster, t=0 inventory and trajectory are keyed on the SAME body ids, which
    is the invariant the adapters honour and the core relies on.
    """
    tracks = [np.asarray(x, dtype=float) for x in tracks]
    ids = ["#%d" % (100 + i) for i in range(len(tracks))]
    joints = list(joints) + [0] * (len(tracks) - len(joints))
    robot_class = list(robot_class) + [True] * (len(tracks) - len(robot_class))

    def _body(i, position=None, with_aabb=False):
        lo = hi = None
        if with_aabb and aabb_min is not None:
            lo = tuple(aabb_min)
            hi = tuple(float(v) + 0.2 for v in aabb_min)
        return Body(body_id=ids[i], name="body_%d" % i, kind="Robot",
                    position=position, aabb_min=lo, aabb_max=hi,
                    n_joints=joints[i], dynamic=True,
                    robot_class=bool(robot_class[i]),
                    behaviour="a_controller",
                    identity_evidence="synthetic")

    b = EvidenceBundle(task=r2_core.TASK, sim="synthetic",
                       adapter="unit-test", artifact=artifact,
                       notes=list(notes))
    b.roster = BodyInventory(bodies=[_body(i) for i in range(len(tracks))],
                             frozen=True, t_s=0.0, source="unit-test roster")
    b.t0 = BodyInventory(
        bodies=[_body(i, position=tuple(tracks[i][0]), with_aabb=(i == 0))
                for i in range(len(tracks))],
        frozen=True, t_s=0.0, source="unit-test t=0 scan")
    b.trajectory = Trajectory(body_ids=list(ids), t=np.asarray(t, dtype=float),
                              xyz=np.stack(tracks), dt_s=r2_core.sample_dt_s(t),
                              recorded_s=float(np.asarray(t)[-1]),
                              complete=True, source="unit-test trajectory")
    b.process = ProcessFacts(exit_code=0 if clean else 1,
                             error_lines=[] if clean else ["ERROR: boom"],
                             driver_completed=bool(clean))
    if attributed:
        b.attribution = EngineAttribution(backend="newton", solver="mujoco",
                                          source="unit-test fixture")
    return b


def good_bundle(**kw):
    """A run that does the task: fixed base, a tool that flies to each target
    and holds it, in order, well inside the deadline."""
    t, rel = reach_track(**kw)
    base = np.zeros((len(t), 3))
    return bundle_from_tracks(t, [base, base + rel],
                              aabb_min=(-0.15, -0.15, 0.0))


def empty_bundle():
    """The null agent: nothing was produced."""
    b = EvidenceBundle(task=r2_core.TASK, sim="synthetic", adapter="unit-test",
                       artifact=None)
    return b


def static_arm_bundle():
    """A compliant six-joint arm with a tool that never moves."""
    n = int(round(20.0 / DT))
    t = np.arange(n, dtype=float) * DT
    base = np.zeros((n, 3))
    tip = np.tile(np.array([0.30, 0.0, 0.30]), (n, 1))
    return bundle_from_tracks(t, [base, tip], aabb_min=(-0.15, -0.15, 0.0))


def invisible_arm_bundle():
    """A six-joint arm and NOTHING else tracked -- the recorder gap."""
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    return bundle_from_tracks(t, [base], aabb_min=(-0.15, -0.15, 0.0))


def teleport_bundle():
    """The tip is placed on each target with no transit between them."""
    t, rel = teleport_track()
    base = np.zeros((len(t), 3))
    return bundle_from_tracks(t, [base, base + rel],
                              aabb_min=(-0.15, -0.15, 0.0))


def dragged_base_bundle():
    """The targets are traced correctly -- by dragging the whole arm."""
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    base[:, 0] = np.linspace(0.0, 0.5, len(t))
    return bundle_from_tracks(t, [base, base + rel],
                              aabb_min=(-0.15, -0.15, 0.0))


def coarse_bundle():
    """A correct run recorded too coarsely to witness a teleport."""
    t, rel = reach_track(dt=0.1)
    base = np.zeros((len(t), 3))
    return bundle_from_tracks(t, [base, base + rel],
                              aabb_min=(-0.15, -0.15, 0.0))


def pedestal_bundle():
    """The arm sits on a 0.5 m pedestal and its tip legitimately dips 5 cm
    below its own base ORIGIN -- but never below its own FEET."""
    t, rel = reach_track(home=(0.0, 0.0, -0.05))
    base = np.zeros((len(t), 3))
    base[:, 2] = 0.5
    return bundle_from_tracks(t, [base, base + rel],
                              aabb_min=(-0.15, -0.15, 0.0))


def underground_bundle():
    """A tip driven below the floor, on a simulator with no world-space AABB
    -- so the datum falls back to the base origin and must still catch it."""
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    tip = base + rel
    tip[:, 2] -= 0.30
    return bundle_from_tracks(t, [base, tip], aabb_min=None)


# =============================================================================
# the contract
# =============================================================================

def test_prompt_is_the_spec_text_verbatim():
    """The three targets in the file the agent reads must be the three the
    grader measures. Drift here is a task that asks for one thing and scores
    another."""
    prompt = (TASK_DIR / "prompt.txt").read_text(encoding="utf-8")
    for x, y, z in r2_core.TARGETS_BASE_FRAME:
        assert "(%.2f, %.2f, %.2f) m" % (x, y, z) in prompt
    assert "at least six actuated revolute joints" in prompt
    assert "at least 0.5 simulated seconds within 5 cm position error" in prompt
    assert "within 30 simulated seconds" in prompt
    assert "not direct end-effector teleportation" in prompt


def test_constants_are_read_from_meta_and_not_copied():
    """The grader must have exactly ONE source for every threshold."""
    c = META["constants"]
    assert r2_core.POSITION_TOL_M == c["POSITION_TOL_M"]
    assert r2_core.DWELL_S == c["DWELL_S"]
    assert r2_core.DEADLINE_S == c["DEADLINE_S"]
    assert r2_core.MIN_ARM_JOINTS == c["MIN_ARM_JOINTS"]
    assert r2_core.MAX_EE_STEP_M == c["MAX_EE_STEP_M"]
    assert r2_core.MAX_EE_SPEED_MPS == c["MAX_EE_SPEED_MPS"]
    assert r2_core.MAX_SAMPLE_DT_S == c["MAX_SAMPLE_DT_S"]
    assert r2_core.MIN_EE_TRAVEL_M == c["MIN_EE_TRAVEL_M"]
    assert r2_core.MAX_BASE_DRIFT_M == c["MAX_BASE_DRIFT_M"]
    assert r2_core.MAX_REACH_M == c["MAX_REACH_M"]
    assert r2_core.GROUND_CLEARANCE_TOL_M == c["GROUND_CLEARANCE_TOL_M"]
    assert [list(t) for t in r2_core.TARGETS_BASE_FRAME] == \
        [list(t) for t in c["TARGETS_BASE_FRAME"]]
    # ...and the read is live, not a copy that happens to agree today.
    assert r2_core._constants() == c


def test_meta_budget_follows_the_spec_rule():
    from agentbench import tasks as _t
    assert META["timeout_s"] == min(3 * META["par_s"],
                                    _t.TASK_HARD_CEILING_S)
    assert META["par_s"] == 600
    # ONE run per (task, arm) since 2026-08-10 (SPEC 3.5): the suite
    # stopped running repeats, so this is 1 and a single observation is
    # an outcome rather than a rate. It used to be 5.
    assert META["repeats_default"] == 1
    assert META["budget"]["runs_per_cell"] == 1
    assert META["budget"]["variance_measured"] is False
    assert META["tier"] == "robotics"
    assert META["grader"] == "agentbench.graders.r2"


def test_the_recording_outlives_the_deadline():
    """A recording that stops AT the deadline cannot tell 'finished at 30.1 s'
    from 'never finished', and R2.6 would be scoring the recorder."""
    sa = META["phases"]["standalone"]
    assert sa["duration_s"] == r2_core.RECORD_DURATION_S
    assert r2_core.RECORD_DURATION_S > r2_core.DEADLINE_S
    assert sa["settle_s"] == 0.0, (
        "t must start at 0 or the deadline is measured from the wrong origin")


def test_meta_declares_the_end_effector_gap_rather_than_hiding_it():
    """R1's precedent: a task with a known hole says so in meta.json, loudly,
    and says it must not be used for a published number until it is closed."""
    assert "end_effector_identification" in META
    assert "teleport_witness" in META
    for key in ("self_collision", "ground_contact", "actuated_and_revolute",
                "base_orientation"):
        assert key in META["not_expressible"]
    # The gap this test was written to guard is now CLOSED (2026-08-09: both
    # recorders gained per-step link tracks). The disclosure therefore has to
    # change with it -- a stale "known hole" warning is its own dishonesty --
    # but the SUCCESSOR claim still has to be asserted, or the closure is just
    # a comment. Both arms must be given the link cap, because a cap forwarded
    # to one arm and not the other is an instrument asymmetry that would read
    # as a capability difference.
    assert "RECORDER GAP CLOSED" in META["status"]
    assert META["phases"]["standalone"]["links"] >= 6, (
        "an arm has >= 6 joints, so fewer link tracks than that cannot "
        "observe its end effector")


def test_both_arms_ship_an_identical_starting_point():
    """A cross-simulator comparison is only like-for-like if the start is.

    R2 is an authoring task and ships NO asset, so 'identical' is checked as
    'the same file set on both sides' -- which is also what catches a future
    asset landing on one arm only.
    """
    a = sorted(p.name for p in (TASK_DIR / "initial").iterdir())
    b = sorted(p.name for p in (TASK_DIR / "initial_webots").iterdir())
    assert a == b == [".gitkeep"]
    assert META["initial_files"] == []


# =============================================================================
# pure maths
# =============================================================================

def test_path_length_and_max_step_are_metres():
    p = [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
    assert r2_core.path_length_xyz(p) == pytest.approx(7.0)
    assert r2_core.max_step_xyz(p) == pytest.approx(4.0)


def test_speed_is_metres_per_second_not_metres_per_sample():
    t = [0.0, 0.1, 0.2]
    p = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.5, 0.0, 0.0)]
    assert r2_core.max_speed_mps(t, p) == pytest.approx(5.0)
    assert r2_core.sample_dt_s(t) == pytest.approx(0.1)


def test_sample_period_is_the_median_so_one_dropped_sample_cannot_relax_it():
    t = [0.0, 0.016, 0.032, 0.400, 0.416, 0.432]
    assert r2_core.sample_dt_s(t) == pytest.approx(0.016, abs=1e-9)


def test_relative_track_is_the_tip_minus_the_base():
    tip = [(1.0, 2.0, 3.0), (1.5, 2.0, 3.0)]
    base = [(1.0, 1.0, 1.0), (1.0, 1.0, 1.0)]
    rel = r2_core.relative_track(tip, base)
    assert rel.tolist() == [[0.0, 1.0, 2.0], [0.5, 1.0, 2.0]]


def test_a_hold_must_be_contiguous_not_a_sum_of_visits():
    """A tip that dithers in and out of the tolerance ball has not held the
    pose; summing its visits would say it had."""
    t = np.arange(0, 2.0, 0.02)
    err = np.where((np.arange(len(t)) % 2) == 0, 0.0, 1.0)   # in, out, in, out
    assert r2_core.first_sustained(t, err, 0.05, 0.5) is None
    err2 = np.where(t < 0.9, 0.0, 1.0)                       # one 0.9 s window
    hit = r2_core.first_sustained(t, err2, 0.05, 0.5)
    assert hit is not None
    enter_i, complete_i, _exit_i = hit
    assert t[enter_i] == pytest.approx(0.0)
    assert float(t[complete_i] - t[enter_i]) >= 0.5


def test_targets_are_scheduled_in_order_and_stop_at_the_first_miss():
    t, rel = reach_track(order=(0, 1))          # only the first two targets
    sched = r2_core.dwell_schedule(t, rel)
    assert [r["reached"] for r in sched] == [True, True, False]
    assert sched[0]["complete_s"] < sched[1]["complete_s"]
    assert len(sched) == 3


def test_visiting_the_targets_backwards_does_not_satisfy_the_sequence():
    """Held last, target 1 is still 'reached' -- but nothing can follow it, and
    that is exactly what ordering means."""
    t, rel = reach_track(order=(2, 1, 0))
    sched = r2_core.dwell_schedule(t, rel)
    assert [r["reached"] for r in sched] == [True, False, False]


def test_a_short_hold_does_not_count():
    t, rel = reach_track(order=(0, 1, 2), dwell_s=0.1)
    sched = r2_core.dwell_schedule(t, rel)
    assert [r["reached"] for r in sched] == [False, False, False]
    assert sched[0]["closest_m"] == pytest.approx(0.0, abs=1e-9)


def test_base_drift_is_measured_from_the_first_sample():
    p = np.zeros((10, 3))
    p[5:, 0] = 0.25
    assert r2_core.max_drift_m(p) == pytest.approx(0.25)


# =============================================================================
# the teleport witness -- the arithmetic the 'not teleported' claim rests on
# =============================================================================

def test_a_jump_between_tolerance_balls_always_breaks_the_step_bound():
    """A teleport can start at the near edge of one tolerance ball and land on
    the near edge of the next, so the shortest jump available to a cheat is the
    closest target pair MINUS two tolerances. That must still exceed the
    per-sample bound, or a teleport hides inside a legal step."""
    d = r2_core.min_inter_target_distance()
    shortest_cheat = d - 2 * r2_core.POSITION_TOL_M
    # The closest pair is target 1 -> target 3, not the consecutive 1 -> 2:
    # 0.2739 m against 0.2872 m. Taking the minimum over ALL pairs is the
    # conservative bound and does not depend on which order a cheat picks.
    assert d == pytest.approx(0.2738613, abs=1e-6)
    assert shortest_cheat > r2_core.MAX_EE_STEP_M, (
        "the shortest jump between tolerance balls is %.4f m and the step "
        "bound is %.4f m -- a teleport would pass R2.4"
        % (shortest_cheat, r2_core.MAX_EE_STEP_M))


def test_the_step_bound_is_the_speed_bound_at_the_coarsest_accepted_dt():
    """Two bounds that disagree would fail honest runs at one sample period and
    excuse teleports at another. They are one bound, expressed twice."""
    assert r2_core.MAX_EE_STEP_M == pytest.approx(
        r2_core.MAX_EE_SPEED_MPS * r2_core.MAX_SAMPLE_DT_S)


def test_the_speed_bound_also_catches_the_cheat_at_every_accepted_dt():
    d = r2_core.min_inter_target_distance() - 2 * r2_core.POSITION_TOL_M
    assert d / r2_core.MAX_SAMPLE_DT_S > r2_core.MAX_EE_SPEED_MPS


def test_every_target_is_inside_the_reach_gate():
    """The candidate filter must not exclude a tip that is doing the task."""
    for tgt in r2_core.TARGETS_BASE_FRAME:
        assert math.dist(tgt, (0.0, 0.0, 0.0)) < r2_core.MAX_REACH_M


# =============================================================================
# end-effector selection
# =============================================================================

def test_the_tip_is_the_body_that_moves_most_within_reach():
    xyz = np.zeros((3, 50, 3))
    xyz[1, :, 0] = np.linspace(0.0, 0.4, 50)       # row 1: swings 0.4 m
    xyz[2, :, 0] = np.linspace(0.0, 0.1, 50)       # row 2: swings 0.1 m
    idx, report = r2_core.select_end_effector(xyz, 0)
    assert idx == 1
    assert report["candidates"] == 2
    assert report["chosen_path_m"] == pytest.approx(0.4, abs=1e-6)


def test_scenery_across_the_room_is_not_the_tip():
    xyz = np.zeros((3, 50, 3))
    xyz[1, :, 0] = np.linspace(0.0, 0.3, 50)       # the tool
    xyz[2, :, 0] = np.linspace(5.0, 30.0, 50)      # a rover driving away
    idx, report = r2_core.select_end_effector(xyz, 0)
    assert idx == 1
    assert report["out_of_reach"] == 1


def test_a_single_tracked_body_yields_no_tip_and_says_so():
    """The recorder gap, in one assertion: one tracked body is one track, and
    there is nothing left to call an end effector."""
    idx, report = r2_core.select_end_effector(np.zeros((1, 50, 3)), 0)
    assert idx is None
    assert report["candidates"] == 0
    assert report["tracks"] == 1


def test_selection_does_not_filter_on_movement():
    """A static tool must still be SELECTED, so the verdict can say 'the tip
    did not move' rather than 'we could not see a tip'. Those are an agent
    failure and a harness failure and must never render the same."""
    xyz = np.zeros((2, 50, 3))
    xyz[1, :, 0] = 0.3                             # present, never moves
    idx, report = r2_core.select_end_effector(xyz, 0)
    assert idx == 1
    assert report["chosen_path_m"] == pytest.approx(0.0)


# =============================================================================
# end to end
# =============================================================================

def test_a_clean_reach_run_passes_all_six():
    """The positive control. C2's disease was a task that could not FAIL; a
    task that cannot PASS is the same bug with the sign flipped."""
    v = r2_core.grade(good_bundle())
    assert v.outcome == "PASS", v.summary()
    assert len(v.assertions) == 6
    assert v.measurements["end_effector_candidates"] == 1
    assert v.progress == 4


def test_an_empty_world_fails_every_assertion():
    v = r2_core.grade(empty_bundle())
    assert v.outcome == "FAIL"
    assert v.failed == ["R2.1", "R2.2", "R2.3", "R2.4", "R2.5", "R2.6"]
    assert v.progress == 0


def test_a_five_joint_arm_is_not_an_arm():
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    b = bundle_from_tracks(t, [base, base + rel], joints=(5,))
    v = r2_core.grade(b)
    assert v.outcome == "FAIL"
    assert "R2.2" in v.failed


def test_two_six_joint_arms_are_ambiguous_and_fail_r2_2():
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    b = bundle_from_tracks(t, [base, base + rel], joints=(6, 6))
    v = r2_core.grade(b)
    assert "R2.2" in v.failed
    assert set(v.failed) == {"R2.2", "R2.3", "R2.4", "R2.5", "R2.6"}


def test_a_dirty_run_fails_r2_1_without_hiding_the_physics():
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    v = r2_core.grade(bundle_from_tracks(t, [base, base + rel], clean=False))
    assert v.failed == ["R2.1"], v.summary()


def test_an_unattributed_run_is_invalid_not_failed():
    t, rel = reach_track()
    base = np.zeros((len(t), 3))
    v = r2_core.grade(bundle_from_tracks(t, [base, base + rel],
                                         attributed=False))
    assert v.outcome == "INVALID"


def test_the_ground_datum_is_the_arm_FEET_not_its_base_origin():
    """An arm on a pedestal may legitimately reach below its own base origin.

    Reading the origin as the floor would fail that correct run, which is why
    the datum is the arm's world-space AABB bottom wherever one exists. The
    numbers here are chosen so the two datums DISAGREE about the verdict.
    """
    v = r2_core.grade(pedestal_bundle())
    a = [x for x in v.assertions if x.id == "R2.5"][0]
    assert a.measured["ground datum (m)"] == pytest.approx(0.0)
    assert a.measured["lowest tip height (m)"] == pytest.approx(0.45)
    assert "R2.5" not in v.failed, v.summary()
    # ...and the fallback datum would have called this 5 cm underground.
    assert 0.45 - 0.5 < -r2_core.GROUND_CLEARANCE_TOL_M


def test_without_world_bounds_the_datum_falls_back_and_says_so():
    """Upstream Webots' Supervisor API has no bounds query, so that arm's
    bundles carry no AABB. The fallback must still catch a dive AND must name
    itself, or a reader cannot tell which datum a row was scored against."""
    v = r2_core.grade(underground_bundle())
    a = [x for x in v.assertions if x.id == "R2.5"][0]
    assert "R2.5" in v.failed
    assert a.measured["ground datum (m)"] == pytest.approx(0.0)
    assert a.measured["lowest tip height (m)"] == pytest.approx(-0.20)
    assert "no world-space AABB" in a.detail


def test_a_run_that_finishes_after_the_deadline_fails_only_r2_6():
    # 3 targets x (approach + hold) must land past 30 s but inside the 35 s
    # recording: 3 x (9.5 + 1.0) = 31.5 s.
    t, rel = reach_track(approach_s=9.5, dwell_s=1.0)
    base = np.zeros((len(t), 3))
    v = r2_core.grade(bundle_from_tracks(t, [base, base + rel],
                                         aabb_min=(-0.15, -0.15, 0.0)))
    assert v.failed == ["R2.6"], v.summary()
    a = [x for x in v.assertions if x.id == "R2.6"][0]
    assert a.measured["third hold satisfied at (s)"] > r2_core.DEADLINE_S
