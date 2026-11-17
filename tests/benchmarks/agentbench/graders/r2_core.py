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

"""R2 ``arm_reach`` -- the **sim-neutral core** of the robotics tier's second
task.

Drive an actuated fixed-base arm's end effector through three Cartesian targets
in order, holding each one. That is the canonical manipulation primitive --
inverse kinematics plus joint control plus dwell -- and it is the first thing
anyone is taught about a manipulator in any simulator. Every primary comparator
(OmniSim, Webots, Isaac, Gazebo, CoppeliaSim, Genesis, MuJoCo) documents
articulated arms and joint actuators, so a simulator that cannot express this is
making a statement about itself rather than about the task.

    R2.1  the run is clean        exit 0, no error lines, driver completed
    R2.2  one arm, >= 6 joints    exactly one robot-class body with
                                  MIN_ARM_JOINTS joints or more
    R2.3  three targets, in order each held >= DWELL_S within POSITION_TOL_M,
                                  measured as end-effector MINUS base
    R2.4  actuated, not teleported  per-sample step and speed bounded, the tip
                                  actually travelled, and the base stayed put
    R2.5  nothing went into the ground  the tip never dropped below the arm's
                                  own mounting datum
    R2.6  finished inside the deadline  the third dwell completes <= DEADLINE_S

Everything here is metres and seconds. Nothing in this module knows what a
simulator is. Every threshold is READ from ``tasks/R2_arm_reach/meta.json`` --
there is no second copy to drift.

**How the end effector is identified, and what that costs.**
By GEOMETRY, never by name. R1 shipped a grader that keyed its obstacles on the
published ``OBSTACLE_n`` names; the first real agent called them ``crate A``..
``crate E`` and a perfectly correct world scored zero obstacles. Nothing in R2's
prompt suggests a name for a tool either, so the same mistake is available and
is not made here. The rule is:

    the arm    = the single robot-class body with >= MIN_ARM_JOINTS joints
    the tip    = the tracked body, OTHER than that base, which stays within
                 MAX_REACH_M of the base for the whole run and travels furthest

The base is fixed and links nearer the base sweep smaller arcs, so "within reach
and moves the most" *is* the tip -- in any simulator, under any naming, for any
URDF. Its failure modes, stated rather than discovered later:

1. **The dominant one, and it is ours, not the agent's.** Both shipped recorders
   sample TOP-LEVEL bodies only -- OmniSim's walks the scene for nodes whose
   type is ``Robot``; upstream Webots' enumerates ``root.children``. An arm
   authored as ONE ``Robot`` node therefore contributes exactly one trajectory
   row, its fixed base, and there is no tip track to measure at all. R2.3-R2.6
   then fail with ``end_effector_candidates = 0``, which is a statement about
   the recorder. The grader emits a note saying so on every such run, and
   meta.json's ``status`` says R2 must not be used for a published number until
   per-link sampling exists on BOTH arms. Reading one of those FAILs as an agent
   failure would be the C2 mistake with the sign flipped.
2. **A moving prop parked near the arm.** A body that is not the tool but is
   within MAX_REACH_M and travels further would be selected. It then has to hold
   all three targets in order to earn R2.3, which is doing the task.
3. **A tool rigid with the last link** is the same body as the last link and, on
   the current recorders, the same case as (1).
4. **Selection is not membership.** The neutral evidence carries no parent/child
   relation, so "belongs to the arm" is approximated by the reach gate. A
   two-arm scene where the second arm's tip is within 1.5 m of the first's base
   is ambiguous, and R2.2 fails such a scene first.

**Why the movement floor is an assertion and not a selection filter.** Selecting
the tip on "moves the most, and moved at least X" would collapse two different
findings into one: a static arm and an invisible arm would both report "no end
effector". They are not the same thing -- one is the agent's failure and one is
ours -- so selection uses reach only, and MIN_EE_TRAVEL_M is a clause of R2.4.

**What is deliberately NOT asserted.** Self-collision, which the prompt asks
for: the neutral contact channel carries only contacts whose two participants
resolve to *different* robot subtrees, and a self-collision by definition
resolves to one, so it is dropped before a grader could see it -- on both arms.
Faking it from geometry would be worse than the gap. Ground *contact* is
unavailable for the same reason (a floor is not a robot), so R2.5 asserts the
ground geometrically, against the arm's own mounting datum, and says so in its
own detail. Neither "actuated" nor "revolute" is expressible either: the neutral
evidence carries a joint COUNT, so R2.2 asserts six joints and states the
difference instead of overclaiming it.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, INVALID, MIXED,
                                        NO_ARTIFACT, Verdict)

TASK = "R2_arm_reach"


def _constants():
    """The thresholds, read from the task contract -- never a second copy.

    SPEC: *if a number is written in the spec, the code uses THAT number.* R1
    duplicated its constants into ``meta.json`` and pinned the pair with a test,
    which catches drift after the fact; reading them is strictly better, because
    there is nothing to drift.
    """
    p = (Path(__file__).resolve().parents[1] / "tasks" / TASK / "meta.json")
    return json.loads(p.read_text(encoding="utf-8"))["constants"]


_C = _constants()

#: The three targets, in the arm's base frame, metres.
TARGETS_BASE_FRAME = tuple(tuple(float(v) for v in t)
                           for t in _C["TARGETS_BASE_FRAME"])
POSITION_TOL_M = float(_C["POSITION_TOL_M"])
DWELL_S = float(_C["DWELL_S"])
DEADLINE_S = float(_C["DEADLINE_S"])
RECORD_DURATION_S = float(_C["RECORD_DURATION_S"])
MIN_ARM_JOINTS = int(_C["MIN_ARM_JOINTS"])
MAX_EE_SPEED_MPS = float(_C["MAX_EE_SPEED_MPS"])
MAX_SAMPLE_DT_S = float(_C["MAX_SAMPLE_DT_S"])
MAX_EE_STEP_M = float(_C["MAX_EE_STEP_M"])
MIN_EE_TRAVEL_M = float(_C["MIN_EE_TRAVEL_M"])
MAX_BASE_DRIFT_M = float(_C["MAX_BASE_DRIFT_M"])
MAX_REACH_M = float(_C["MAX_REACH_M"])
GROUND_CLEARANCE_TOL_M = float(_C["GROUND_CLEARANCE_TOL_M"])

_ALL = [("R2.1", "the run is clean"),
        ("R2.2", "there is one arm with at least six joints"),
        ("R2.3", "the three targets are reached, in order, and held"),
        ("R2.4", "it was actuated, not teleported"),
        ("R2.5", "nothing went into the ground"),
        ("R2.6", "the sequence finished inside the deadline")]

_BASIS = {"R2.1": MIXED, "R2.2": MIXED, "R2.3": CORE_PHYSICAL,
          "R2.4": CORE_PHYSICAL, "R2.5": CORE_PHYSICAL,
          "R2.6": CORE_PHYSICAL}

#: Emitted whenever the recorder gave us no tip to measure -- see the module
#: docstring, failure mode 1. A FAIL carrying this note is OUR blindness.
NO_EE_NOTE = (
    "no end-effector body was observable: the trajectory carries %d body "
    "track(s) and none of them, other than the arm's own base, stays within "
    "%.2f m of it. Both shipped recorders sample TOP-LEVEL bodies only, so an "
    "arm authored as a single Robot node exposes its base and nothing else -- "
    "R2.3-R2.6 below fail for want of evidence, which is a statement about the "
    "recorder and NOT about the agent. See meta.json 'status'.")


# --- pure trajectory maths ---------------------------------------------------


def path_length_xyz(p):
    """Integrated 3-D arc length of an (N, 3) track, metres."""
    p = np.asarray(p, dtype=float)
    if len(p) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def max_step_xyz(p):
    """Largest single-sample 3-D displacement, metres. Half the teleport
    witness -- the half the task brief asked for literally."""
    p = np.asarray(p, dtype=float)
    if len(p) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).max())


def max_speed_mps(t, p):
    """Largest sample-to-sample speed, m/s. The other half of the witness.

    A metre-per-sample bound silently changes meaning with the basic timestep --
    0.05 m is 3.1 m/s at 16 ms and 1.6 m/s at 32 ms -- so the bound that must
    hold across simulators is the speed. Both are asserted; see
    ``sample_dt_s``, which refuses to certify either from a recording too
    coarse to witness a jump.
    """
    t = np.asarray(t, dtype=float)
    p = np.asarray(p, dtype=float)
    if len(p) < 2 or len(t) != len(p):
        return 0.0
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dt = np.diff(t)
    ok = dt > 0
    if not ok.any():
        return float("inf")
    return float((dp[ok] / dt[ok]).max())


def sample_dt_s(t):
    """The recording's sample period, as the MEDIAN of the sample gaps.

    Median rather than mean: one dropped sample must not be able to relax the
    teleport witness for the whole run.
    """
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return None
    d = np.diff(t)
    d = d[d > 0]
    return float(np.median(d)) if len(d) else None


def relative_track(tip, base):
    """``tip - base``, sample by sample: the tip in the base's translated axes.

    The neutral evidence carries positions and never orientations, so this is
    the base *frame* only when the base is axis-aligned with the world -- the
    natural authoring, and the one an agent handed base-frame coordinates will
    choose. Declared in meta.json's ``not_expressible``; not hidden.
    """
    return np.asarray(tip, dtype=float) - np.asarray(base, dtype=float)


def target_error(rel, target):
    """Per-sample distance from the track to one target, metres."""
    return np.linalg.norm(np.asarray(rel, dtype=float)
                          - np.asarray(target, dtype=float), axis=1)


def first_sustained(t, err, tol, dwell_s, start=0):
    """The first CONTIGUOUS window from ``start`` where ``err <= tol`` lasts
    ``dwell_s``.

    Returns ``(enter_i, complete_i, exit_i)`` or ``None``. ``complete_i`` is the
    first sample at which the hold requirement is actually met, which is the
    time R2.6 measures the deadline against.

    Contiguous on purpose: a tip that dithers in and out of the tolerance ball
    has not held the pose, and summing its visits would say it had.
    """
    t = np.asarray(t, dtype=float)
    err = np.asarray(err, dtype=float)
    n = len(err)
    i = int(start)
    while i < n:
        if err[i] > tol:
            i += 1
            continue
        j = i
        while j + 1 < n and err[j + 1] <= tol:
            j += 1
        if float(t[j] - t[i]) >= dwell_s:
            m = i
            while m <= j and float(t[m] - t[i]) < dwell_s:
                m += 1
            return i, min(m, j), j
        i = j + 1
    return None


def dwell_schedule(t, rel, targets=TARGETS_BASE_FRAME, tol=POSITION_TOL_M,
                   dwell_s=DWELL_S):
    """Walk the targets IN ORDER, each hold starting no earlier than the last
    hold completed.

    Returns one record per target: ``{"index", "target", "reached",
    "enter_s", "complete_s", "hold_s", "closest_m"}``. ``closest_m`` is filled
    even for a target that was never held, so a verdict says *how close* rather
    than only *no*.
    """
    t = np.asarray(t, dtype=float)
    out = []
    cursor = 0
    for k, target in enumerate(targets):
        err = target_error(rel, target)
        rec = {"index": k + 1, "target": [float(v) for v in target],
               "reached": False, "enter_s": None, "complete_s": None,
               "hold_s": 0.0,
               "closest_m": (float(err[cursor:].min())
                             if len(err[cursor:]) else None)}
        hit = first_sustained(t, err, tol, dwell_s, start=cursor)
        if hit is not None:
            enter_i, complete_i, exit_i = hit
            rec.update(reached=True,
                       enter_s=float(t[enter_i]),
                       complete_s=float(t[complete_i]),
                       hold_s=float(t[exit_i] - t[enter_i]))
            cursor = complete_i
        out.append(rec)
        if not rec["reached"]:
            break                       # in order: no target k+1 without k
    while len(out) < len(targets):
        k = len(out)
        out.append({"index": k + 1,
                    "target": [float(v) for v in targets[k]],
                    "reached": False, "enter_s": None, "complete_s": None,
                    "hold_s": 0.0, "closest_m": None})
    return out


def select_end_effector(xyz, base_index, max_reach_m=MAX_REACH_M):
    """Pick the tip out of an (n_bodies, N, 3) stack. Geometry only.

    Candidates are every track except the base that stays within
    ``max_reach_m`` of the base at EVERY sample; the winner is the one that
    travels furthest, tie-broken by mean distance from the base (further out on
    the chain). Returns ``(index_or_None, report)``; the report is what the
    verdict prints, and its ``candidates`` count is the field that separates
    "our recorder saw no tip" from "the agent's tip missed".

    Note what is NOT a filter here: how far the candidate moved. Folding a
    movement floor into selection would make a static arm and an invisible arm
    report the same thing, and they are not the same thing.
    """
    xyz = np.asarray(xyz, dtype=float)
    report = {"tracks": int(xyz.shape[0]), "candidates": 0,
              "out_of_reach": 0, "chosen": None,
              "chosen_path_m": None, "chosen_mean_offset_m": None,
              "max_reach_m": float(max_reach_m)}
    if xyz.ndim != 3 or xyz.shape[0] < 2:
        return None, report
    base = xyz[int(base_index)]
    best = None
    for i in range(xyz.shape[0]):
        if i == int(base_index):
            continue
        offset = np.linalg.norm(xyz[i] - base, axis=1)
        if float(offset.max()) > float(max_reach_m):
            report["out_of_reach"] += 1
            continue
        report["candidates"] += 1
        score = (path_length_xyz(xyz[i]), float(offset.mean()))
        if best is None or score > best[0]:
            best = (score, i)
    if best is None:
        return None, report
    (path_m, mean_off), idx = best
    report.update(chosen=int(idx), chosen_path_m=round(path_m, 6),
                  chosen_mean_offset_m=round(mean_off, 6))
    return int(idx), report


def max_drift_m(p):
    """Largest displacement from the first sample, metres. A fixed base that
    moves is a robot being dragged."""
    p = np.asarray(p, dtype=float)
    if len(p) < 1:
        return 0.0
    return float(np.linalg.norm(p - p[0], axis=1).max())


def min_inter_target_distance(targets=TARGETS_BASE_FRAME):
    """The closest pair of targets, metres -- the teleport witness's margin."""
    worst = float("inf")
    for i in range(len(targets)):
        for j in range(i + 1, len(targets)):
            worst = min(worst, math.dist(targets[i], targets[j]))
    return worst


# --- grading -----------------------------------------------------------------


def _t0_body(bundle, body_id):
    for b in bundle.t0.bodies:
        if b.body_id == body_id:
            return b
    return None


def _traj_row(bundle, traj, body):
    """Row index of ``body`` in the trajectory, or None.

    By body id first (the adapters key both the roster and the trajectory on the
    same recorder id), falling back to the roster position, which is only valid
    for a FROZEN inventory -- a live one carries no index guarantee, which is
    why ``BodyInventory.frozen`` is a field and not a comment.
    """
    idx = traj.index_of(body.body_id)
    if idx is not None:
        return idx
    if bundle.roster.frozen:
        try:
            i = bundle.roster.bodies.index(body)
        except ValueError:
            return None
        return i if i < traj.n_bodies else None
    return None


def grade(bundle, *, self_verified=False):
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)
    L = bundle.lbl

    if bundle.artifact is None:
        v.progress = NO_ARTIFACT
        for aid, what in _ALL:
            v.add(aid, False, what, basis=_BASIS[aid],
                  detail=L("artifact_missing_detail", "no artifact"))
        return v.finish()
    v.artifacts["world"] = str(bundle.artifact)

    traj = bundle.trajectory
    if traj is None or traj.xyz is None or traj.n_samples < 2:
        v.progress = ARTIFACT_INVALID
        why = (bundle.motion_error or (traj.error if traj else None)
               or "fewer than two motion samples")
        for aid, what in _ALL:
            v.add(aid, False, what, detail=why, basis=_BASIS[aid])
        v.measurements.update(bundle.adapter_measurements.get("motion") or {})
        return v.finish()

    v.progress = ARTIFACT_RUNS
    v.measurements.update(bundle.adapter_measurements.get("motion") or {})

    attrib = bundle.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    if attrib is None:
        v.outcome = INVALID
        v.note(L("attribution_missing_note",
                 "no physics-engine attribution: the adapter could not name "
                 "the engine that drove this run from any channel it has."))
        return v

    # --- R2.1 the run is clean --------------------------------------------
    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    v.add("R2.1", bool(proc and proc.clean and complete), "the run is clean",
          measured={L("exit_code", "exit code"): proc.exit_code if proc
                    else None,
                    L("error_lines", "error lines"): len(errs),
                    L("driver_completed", "driver reached a clean quit"):
                        complete},
          threshold={L("exit_code", "exit code"): 0,
                     L("error_lines", "error lines"): 0},
          detail="; ".join(errs[:3]), basis=MIXED)

    # --- R2.2 one arm with at least six joints ----------------------------
    robots = [b for b in bundle.roster.bodies if b.robot_class]
    arms = [b for b in robots if (b.n_joints or 0) >= MIN_ARM_JOINTS]
    v.add("R2.2", len(arms) == 1, "there is one arm with at least six joints",
          measured={L("robot_bodies", "robot-class bodies"): len(robots),
                    L("joint_counts", "joints per robot body"):
                        [b.n_joints for b in robots][:6],
                    L("arms", "of those, with >= %d joints" % MIN_ARM_JOINTS):
                        len(arms),
                    L("names", "names"): [b.name for b in robots][:6]},
          threshold={L("arms", "bodies with >= %d joints" % MIN_ARM_JOINTS): 1},
          detail=("the neutral evidence carries a JOINT COUNT and nothing "
                  "finer -- both recorders count hinge/slider/ball joints in "
                  "the subtree without distinguishing motorised from passive "
                  "or revolute from prismatic, so 'six actuated revolute "
                  "joints' is asserted as 'six joints' and the gap is stated "
                  "rather than overclaimed"),
          basis=MIXED)

    if len(arms) != 1:
        for aid in ("R2.3", "R2.4", "R2.5", "R2.6"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail="no single arm to measure (R2.2)")
        return v.finish()

    arm = arms[0]
    base_row = _traj_row(bundle, traj, arm)
    if base_row is None:
        for aid in ("R2.3", "R2.4", "R2.5", "R2.6"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail=("the arm is in the roster but has no trajectory "
                          "row, so nothing about its motion is measurable"))
        return v.finish()

    t = np.asarray(traj.t, dtype=float)
    base_xyz = np.asarray(traj.xyz[base_row], dtype=float)
    base_drift = max_drift_m(base_xyz)

    ee_row, ee_report = select_end_effector(traj.xyz, base_row)
    v.measurements["end_effector_selection"] = ee_report
    v.measurements["end_effector_candidates"] = ee_report["candidates"]

    if ee_row is None:
        v.note(NO_EE_NOTE % (traj.n_bodies, MAX_REACH_M))
        gap = ("no end-effector body was observable (candidates=0 of %d "
               "track(s)); see this verdict's note and meta.json 'status' -- "
               "this is a recorder gap, not an agent failure"
               % traj.n_bodies)
        v.add("R2.3", False, dict(_ALL)["R2.3"], basis=_BASIS["R2.3"],
              detail=gap)
        # The base half of R2.4 IS measurable without a tip, so measure it and
        # report both -- an unmeasured clause is never quietly credited, and
        # half the evidence is not none of it.
        v.add("R2.4", False, "it was actuated, not teleported",
              measured={L("base_drift", "base drift from t=0 (m)"):
                        round(base_drift, 6),
                        L("ee_candidates", "end-effector candidates"): 0},
              threshold={L("base_drift", "base drift from t=0 (m)"):
                         "<= %.3f" % MAX_BASE_DRIFT_M,
                         L("ee_candidates", "end-effector candidates"): ">= 1"},
              detail=gap, basis=CORE_PHYSICAL)
        for aid in ("R2.5", "R2.6"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid], detail=gap)
        return v.finish()

    ee_xyz = np.asarray(traj.xyz[ee_row], dtype=float)
    rel = relative_track(ee_xyz, base_xyz)
    schedule = dwell_schedule(t, rel)
    v.measurements["target_schedule"] = schedule

    # --- R2.3 three targets, in order, held -------------------------------
    reached = [r for r in schedule if r["reached"]]
    v.add("R2.3", len(reached) == len(TARGETS_BASE_FRAME),
          "the three targets are reached, in order, and held",
          measured={L("targets_held", "targets held, in order"): len(reached),
                    L("hold_s", "hold per target (s)"):
                        [round(r["hold_s"], 4) for r in schedule],
                    L("closest_m", "closest approach per target (m)"):
                        [None if r["closest_m"] is None
                         else round(r["closest_m"], 4) for r in schedule],
                    L("complete_s", "hold satisfied at (s)"):
                        [r["complete_s"] for r in schedule]},
          threshold={L("targets_held", "targets held, in order"):
                     len(TARGETS_BASE_FRAME),
                     L("closest_m", "position error (m)"):
                         "<= %.3f" % POSITION_TOL_M,
                     L("hold_s", "hold (s)"): ">= %.2f" % DWELL_S},
          detail=("measured as end-effector MINUS base, sample by sample; the "
                  "neutral evidence carries no orientation, so 'base frame' is "
                  "world axes translated to the base origin"),
          basis=CORE_PHYSICAL)

    # --- R2.4 actuated, not teleported ------------------------------------
    step_m = max_step_xyz(ee_xyz)
    speed = max_speed_mps(t, ee_xyz)
    dt = sample_dt_s(t)
    travel = path_length_xyz(ee_xyz)
    dt_ok = dt is not None and dt <= MAX_SAMPLE_DT_S
    ok4 = (step_m <= MAX_EE_STEP_M and speed <= MAX_EE_SPEED_MPS
           and travel >= MIN_EE_TRAVEL_M and base_drift <= MAX_BASE_DRIFT_M
           and dt_ok)
    v.add("R2.4", ok4, "it was actuated, not teleported",
          measured={L("max_step", "largest single-sample step (m)"):
                    round(step_m, 6),
                    L("max_speed", "largest sample-to-sample speed (m/s)"):
                        round(speed, 6),
                    L("sample_dt", "sample period (s)"):
                        None if dt is None else round(dt, 6),
                    L("ee_travel", "distance the tip travelled (m)"):
                        round(travel, 6),
                    L("base_drift", "base drift from t=0 (m)"):
                        round(base_drift, 6)},
          threshold={L("max_step", "largest single-sample step (m)"):
                     "<= %.3f" % MAX_EE_STEP_M,
                     L("max_speed", "largest sample-to-sample speed (m/s)"):
                         "<= %.2f" % MAX_EE_SPEED_MPS,
                     L("sample_dt", "sample period (s)"):
                         "<= %.3f" % MAX_SAMPLE_DT_S,
                     L("ee_travel", "distance the tip travelled (m)"):
                         ">= %.2f" % MIN_EE_TRAVEL_M,
                     L("base_drift", "base drift from t=0 (m)"):
                         "<= %.3f" % MAX_BASE_DRIFT_M},
          detail=("the closest two targets are %.4f m apart and %.4f m apart "
                  "at the near edges of their tolerance balls, so a jump "
                  "between them exceeds the step bound at every sample period "
                  "the grader accepts; a recording coarser than %.3f s cannot "
                  "witness that and is refused rather than assumed clean"
                  % (min_inter_target_distance(),
                     min_inter_target_distance() - 2 * POSITION_TOL_M,
                     MAX_SAMPLE_DT_S)),
          basis=CORE_PHYSICAL)

    # --- R2.5 nothing went into the ground --------------------------------
    # The floor is not in the neutral evidence -- neither recorder samples it
    # and the contact channel drops any pair that is not robot-to-robot -- so
    # the arm's OWN mounting datum is the floor datum: its t=0 world AABB
    # bottom where the adapter has one, else its base origin at t=0.
    t0 = _t0_body(bundle, arm.body_id)
    if t0 is not None and t0.has_aabb:
        datum_z = float(t0.aabb_min[2])
        datum_src = "the arm's world-space AABB floor at t=0"
    else:
        datum_z = float(base_xyz[0][2])
        datum_src = ("the arm base origin at t=0 (no world-space AABB on this "
                     "simulator)")
    min_ee_z = float(ee_xyz[:, 2].min())
    clearance = min_ee_z - datum_z
    v.add("R2.5", clearance >= -GROUND_CLEARANCE_TOL_M,
          "nothing went into the ground",
          measured={L("min_ee_z", "lowest tip height (m)"):
                    round(min_ee_z, 6),
                    L("ground_datum", "ground datum (m)"): round(datum_z, 6),
                    L("clearance", "lowest tip clearance over datum (m)"):
                        round(clearance, 6)},
          threshold={L("clearance", "lowest tip clearance over datum (m)"):
                     ">= %.3f" % (-GROUND_CLEARANCE_TOL_M)},
          detail=("datum: %s. GEOMETRIC, not a contact read: the neutral "
                  "contact channel only carries pairs resolving to two "
                  "DIFFERENT robot subtrees, so neither a robot/floor contact "
                  "nor a SELF-collision can reach a grader on either arm -- "
                  "self-collision is therefore not scored at all rather than "
                  "faked from geometry" % datum_src),
          basis=CORE_PHYSICAL)

    # --- R2.6 finished inside the deadline --------------------------------
    last = schedule[-1]
    finish_s = (None if not last["reached"]
                else float(last["complete_s"]) - float(t[0]))
    v.add("R2.6", finish_s is not None and finish_s <= DEADLINE_S,
          "the sequence finished inside the deadline",
          measured={L("finish_s", "third hold satisfied at (s)"):
                    None if finish_s is None else round(finish_s, 4),
                    L("recorded_s", "simulated seconds recorded"):
                        None if traj.recorded_s is None
                        else round(float(traj.recorded_s), 4)},
          threshold={L("finish_s", "third hold satisfied at (s)"):
                     "<= %.1f" % DEADLINE_S},
          detail=("the recording runs to %.1f s so a run finishing just after "
                  "the deadline is measured rather than truncated into "
                  "ambiguity" % RECORD_DURATION_S),
          basis=CORE_PHYSICAL)

    return v.finish()
