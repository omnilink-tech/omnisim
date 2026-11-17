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

"""T1 ``arrive`` -- the **sim-neutral core** of the first ladder rung.

The tier statement is ``docs/developer/capability-ladder-plan.md`` §2 T1, and
its five rows are transcribed here threshold for threshold::

    T1.1  arrived                 within 0.25 m of the commanded point, and
                                  within 0.35 m of it for a continuous >= 2.0 s
                                  window ending at the run's end
    T1.2  stayed up               |z - settled start z| <= 0.30 m at EVERY
                                  sample, and z > -0.10 m at every sample
    T1.3  drove, not teleported   path length between 0.9x and 5.0x the
                                  straight-line start->goal distance, and no
                                  single inter-sample displacement > 0.5 m
    T1.4  touched the surface     >= 1 contact naming (a robot body, the
          while moving            surface under it) observed while moving,
                                  with the vacuity counters present
    T1.5  the run is real         finalize reached on independent evidence,
                                  zero error-class lines, engine attributed
                                  with a citation

Everything here is metres and seconds. The core reads a neutral evidence
bundle and the ladder's two extra channels
(:mod:`ladder.graders.ladder_evidence`) and nothing else: it cannot open a
scene file, cannot name a node type, and cannot ask which engine it is talking
to except through the attribution the adapter cited.

Three readings the tier table leaves open, decided here and stated so they can
be attacked -- plus one row whose LABEL was fixed before the freeze, after it
was demonstrated to claim more than the row measures
--------------------------------------------------------------------------

**T1.1's two tolerances.** "within 0.25 m ... and stays within 0.35 m ... for
a continuous window of >= 2.0 s ending at the run's end" is graded as: take
the maximal trailing run of samples whose horizontal error is <= 0.35 m; that
window must be >= 2.0 s long, and the **best** error inside it must be
<= 0.25 m. So a robot that touches the point and then settles 0.3 m off it
passes, and one that merely loiters in the 0.35 m band without ever reaching
0.25 m does not. The alternative reading -- the *final sample* within 0.25 m
-- fails a robot that arrived and then relaxed against its own suspension,
which is arrival by any physical account.

**T1.3's jump bound needs a sample rate to mean anything.** "no single
inter-sample displacement exceeds 0.5 m at the per-physics-step rate" is only
a teleport test if the samples really are per-step: at a 1 Hz sample rate a
robot doing 0.4 m/s never trips it. So the clause carries a witness --
the sampler must report its interval and it must be <= 0.05 s -- and a
coarser series makes the clause **vacuous** rather than passing it. The
displacement measured is the full 3-D one, so a body flicked upward is caught
here as well as by T1.2.

**T1.4's "the ground".** The tier says "(a robot body, the ground)". No
simulator-neutral definition of *the* ground exists -- one column's floor is a
static body, another's is a terrain primitive, another's is an infinite plane
with no body at all -- so the core reads it as **a distinct body the adapter
attests is not a robot, in contact with the robot under test while that robot
is moving**. The surface bodies actually touched are printed in the
measurement, so a reader can see whether that was a floor or a table. An
adapter that cannot say whether the other side is a robot supplies ``None``
and the clause reports itself vacuous; it never guesses.

**T1.4's LABEL was fixed on 2026-08-02; its CHECK was deliberately left
alone.** The row used to be called *"under its own actuation"*, and what it
measures is *"a contact naming the robot and a distinct non-robot body, seen
while the robot was moving"* -- which **a robot riding a moving pallet or a
conveyor satisfies**. Demonstrated during the pre-freeze audit: T1 graded PASS
5/5 on a robot whose only support contact was with a body named ``pallet``.
Two repairs were available and the cheap one is the wrong one:

* *make the check match the label* -- detect actuation. Nothing in any channel
  distinguishes "drove" from "was carried", so it would need wheel torque or
  joint velocity, which **no column in the roster supplies today**. It would
  impose a new capability requirement on every column, and the cells it would
  redden first would be reddened by *our* missing scaffolding, which §4 forbids
  publishing as somebody else's product failure.
* *make the label match the check* -- rename the row. It costs nothing, it
  removes an over-claim from a published grid, and it leaves the physical
  question ("was it carried, or did it drive?") visible as an open one rather
  than silently answered.

The second was ratified. The row is now **"touched the surface while moving"**
in this module, in the task's ``meta.json`` and in
``capability-ladder-plan.md`` §2 T1's own table, and the stronger reading is
recorded as *considered and declined*, with that reason, in the task file.
**Nothing about what is measured changed** -- the same contacts, the same
speed floor, the same vacuity counters, the same fixtures.

Robot selection
---------------

T1 grades one robot. The core picks it by, in order: the body whose name
matches the name the robot description declares (task data, passed in); else
the single robot-class body if there is exactly one; else nothing -- and "the
run reports N robot bodies and none carries the declared name" is a FAIL with
that sentence in the detail, never a silent pick of the luckiest one.
"""

from __future__ import annotations

import math

import numpy as np

from agentbench.graders import physical as ph
from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier,
                                        Verdict)
# Two helpers T1 shares with the tier above it, so they live in one place and
# cannot drift: the inter-sample jump bound (T1.3 here, T2.4 there) and the
# "did this run actually happen" assertion the plan states once and then
# refers back to as "as T1.5". Both are re-exported under the names this
# module already published.
from ladder.graders.ladder_evidence import (  # noqa: F401  (re-exported)
    max_step_jump, run_is_real)

TASK = "T1_arrive"

# --- the tier's thresholds, quoted so a drift is visible in a diff ----------
ARRIVE_TOL_M = 0.25          # T1.1  best horizontal error inside the dwell
HOLD_TOL_M = 0.35            # T1.1  the band the dwell window is measured in
DWELL_S = 2.0                # T1.1  continuous, ending at the run's end
MAX_ABS_DZ_M = 0.30          # T1.2  about the settled start height
MIN_Z_M = -0.10              # T1.2  it did not sink through
MIN_PATH_RATIO = 0.9         # T1.3  vs the straight-line start->goal distance
MAX_PATH_RATIO = 5.0         # T1.3
MAX_STEP_JUMP_M = 0.5        # T1.3  per inter-sample interval

# Not tier constants -- witness bounds the tier's wording implies. Both are
# declared rather than assumed, because each decides whether a clause is a
# measurement or a formality.
MAX_SAMPLE_DT_S = 0.05       # coarser than this and the jump bound is vacuous
MOVING_SPEED_EPS_MPS = 0.05  # below this the robot is not "in motion"
MIN_GOAL_DISTANCE_M = 0.25   # a goal closer than this makes the ratio unstable

_ALL = [("T1.1", "it arrived at the commanded point and stayed there"),
        ("T1.2", "it stayed up"),
        ("T1.3", "it drove there, it did not jump"),
        ("T1.4", "it touched the surface while it was moving"),
        ("T1.5", "the run is real")]

_BASIS = {"T1.1": CORE_PHYSICAL, "T1.2": CORE_PHYSICAL,
          "T1.3": CORE_PHYSICAL, "T1.4": MIXED, "T1.5": MIXED}


# --- selection ---------------------------------------------------------------


def select_robot(ev):
    """``(roster_index, body, why)`` for the robot T1 grades.

    See the module docstring for the rule. ``why`` is published in the verdict
    so a reader knows which body the metres belong to.
    """
    roster = ev.bundle.roster
    bodies = list(roster.bodies or [])
    if not bodies:
        return None, None, "the run reported no bodies at all"
    named = ev.robot_name or ""
    if named:
        for i, b in enumerate(bodies):
            if b.name == named:
                return i, b, ("the body carrying the name the robot "
                              "description declares (%r)" % named)
    robots = [(i, b) for i, b in enumerate(bodies) if b.robot_class]
    if len(robots) == 1:
        i, b = robots[0]
        return i, b, ("the single robot-class body in the scene (named %r); "
                      "the declared name %r was not found"
                      % (b.name, named or "<unset>"))
    return None, None, ("the run reports %d robot-class bodies and none "
                        "carries the name %r that the robot description "
                        "declares, so there is no one body to grade"
                        % (len(robots), named or "<unset>"))


def series_row_for(ev, index, body):
    """The pose-series row that belongs to ``body``, by id then by position.

    Public because the waypoint resolver needs the same row the grade will
    use: a start-relative commanded point measured against a different body
    than the one graded would be a silent, unfalsifiable error.
    """
    traj = ev.bundle.trajectory
    if traj is None or traj.xyz is None:
        return None, "no motion samples"
    if body is not None and body.body_id:
        j = traj.index_of(body.body_id)
        if j is not None:
            return j, "matched by body id"
    if index is not None and index < traj.n_bodies:
        return index, ("matched by inventory position (the series carries no "
                       "row for that body id)")
    return None, ("the motion series has %d rows and none belongs to the "
                  "graded body" % traj.n_bodies)


# --- measurement helpers (plain numbers in, plain numbers out) ---------------


def trailing_dwell(t, err, tol):
    """``(seconds, first_index)`` of the maximal trailing run with err <= tol.

    "Trailing" is the whole point: a robot that passed through the point at
    t = 4 s and drove off is not holding station at the end of the run.
    """
    t = np.asarray(t, dtype=float)
    err = np.asarray(err, dtype=float)
    n = len(err)
    if n == 0:
        return 0.0, None
    i = n - 1
    if err[i] > tol:
        return 0.0, None
    while i > 0 and err[i - 1] <= tol:
        i -= 1
    return float(t[-1] - t[i]), int(i)


def speed_at(t, xyz, t_query):
    """Planar speed at the sample nearest ``t_query``, m/s, or ``None``."""
    t = np.asarray(t, dtype=float)
    xyz = np.asarray(xyz, dtype=float)
    if len(t) < 2 or t_query is None:
        return None
    k = int(np.argmin(np.abs(t - float(t_query))))
    a = max(0, min(k, len(t) - 2))
    dt = float(t[a + 1] - t[a])
    if dt <= 0.0:
        return None
    d = xyz[a + 1][:2] - xyz[a][:2]
    return float(math.hypot(d[0], d[1]) / dt)


def moving_support_contacts(ev, row):
    """Support contacts seen while the robot was measurably moving.

    Returns ``(hits, speeds)``: the contacts whose timestamp lands on a sample
    where the planar speed is at least :data:`MOVING_SPEED_EPS_MPS`, and every
    speed looked up (so a reader can see how fast it was, not only that it
    passed).
    """
    traj = ev.bundle.trajectory
    hits, speeds = [], []
    if traj is None or traj.xyz is None or row is None:
        return hits, speeds
    xyz = traj.xyz[row]
    for c in ev.support.support_pairs:
        if c.t_s is None:
            continue
        s = speed_at(traj.t, xyz, c.t_s)
        if s is None:
            continue
        speeds.append(s)
        if s >= MOVING_SPEED_EPS_MPS:
            hits.append(c)
    return hits, speeds


# --- the grade ---------------------------------------------------------------


def grade(ev, *, self_verified=False):
    """Grade one T1 run from neutral evidence. Never raises."""
    b = ev.bundle
    v = Verdict(TASK, self_verified=self_verified)
    for note in list(getattr(b, "notes", []) or []) + list(ev.notes or []):
        v.note(note)
    v.measurements["waypoint"] = ev.waypoint.as_dict()
    v.measurements["provenance"] = ev.provenance()

    # ---- the deliverable -------------------------------------------------
    if b.artifact is None:
        v.progress = NO_ARTIFACT
        return _all_red(v, "no deliverable was produced")
    v.artifacts["deliverable"] = str(b.artifact)

    if not ev.waypoint.usable:
        v.outcome = INVALID
        v.note("no commanded point reached the grader, so nothing about "
               "arrival is measurable. The waypoint is task data and a "
               "missing one is a broken instrument, never a failed run.")
        return _all_red(v, "no commanded point", finish=False)

    traj = b.trajectory
    if traj is None or traj.xyz is None:
        v.progress = ARTIFACT_INVALID if b.live_load_ok is False \
            else ARTIFACT_RUNS
        why = (b.motion_error or (traj.error if traj else None)
               or "no motion samples")
        return _all_red(v, why)
    if traj.t is None or len(traj.t) != traj.n_samples:
        v.progress = ARTIFACT_RUNS
        return _all_red(v, "the pose series carries no simulated-time base, "
                           "so no window, rate or speed is measurable")

    index, body, why = select_robot(ev)
    row, row_why = series_row_for(ev, index, body)
    v.measurements["graded_body"] = {
        "selection": why, "series_row": row, "series_row_rule": row_why,
        "name": (body.name if body else None),
        "id": (body.body_id if body else None)}
    if row is None:
        v.progress = ARTIFACT_RUNS
        return _all_red(v, "%s; %s" % (why, row_why))

    v.progress = ARTIFACT_RUNS
    v.measurements.update(b.adapter_measurements.get("motion") or {})

    xyz = traj.xyz[row]
    xy = xyz[:, :2]
    z = xyz[:, 2]
    t = np.asarray(traj.t, dtype=float)
    err = np.hypot(xy[:, 0] - float(ev.waypoint.xy[0]),
                   xy[:, 1] - float(ev.waypoint.xy[1]))

    _t1_1(v, t, err)
    _t1_2(v, z, traj)
    _t1_3(v, ev, xy, xyz, traj)
    _t1_4(v, ev, row)
    attributed = _t1_5(v, b)

    if not attributed:
        v.outcome = INVALID
        v.note("no physics-engine attribution: the adapter could not name the "
               "engine that drove this run from any channel it has. An "
               "unattributed physics result is not a result.")
        return v
    return v.finish(GRADED_PASS)


def _all_red(v, why, *, finish=True):
    for aid, what in _ALL:
        v.add(aid, False, what, detail=why, basis=_BASIS[aid],
              falsifiers=[Falsifier(
                  "evidence present",
                  "a run that produced nothing to measure",
                  "a deliverable, a commanded point and a pose series over "
                  "the run", False, detail=why)])
    return v.finish() if finish else v


# --- T1.1 --------------------------------------------------------------------


def _t1_1(v, t, err):
    dwell_s, i0 = trailing_dwell(t, err, HOLD_TOL_M)
    best = float(err[i0:].min()) if i0 is not None else float(err.min())
    ok = bool(dwell_s >= DWELL_S and best <= ARRIVE_TOL_M)
    window_long_enough = bool(len(t) >= 2
                              and float(t[-1] - t[0]) >= DWELL_S)
    v.add("T1.1", ok, "it arrived at the commanded point and stayed there",
          measured={"final horizontal error (m)": round(float(err[-1]), 4),
                    "best horizontal error inside the trailing window (m)":
                        round(best, 4),
                    "closest approach over the whole run (m)":
                        round(float(err.min()), 4),
                    "trailing window within %.2f m (s)" % HOLD_TOL_M:
                        round(dwell_s, 3)},
          threshold={"best error (m)": "<= %.2f" % ARRIVE_TOL_M,
                     "trailing window (s)": ">= %.1f" % DWELL_S,
                     "band (m)": "<= %.2f" % HOLD_TOL_M},
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("arrived",
                        "a robot that never came within %.2f m of the "
                        "commanded point" % ARRIVE_TOL_M,
                        "a pose series and a commanded point, both in metres "
                        "in the same frame", True),
              Falsifier("held station",
                        "a robot that reached the point and then left, or "
                        "drifted out of the %.2f m band before the run ended"
                        % HOLD_TOL_M,
                        "a run at least %.1f s long, so a %.1f s trailing "
                        "window can exist at all" % (DWELL_S, DWELL_S),
                        window_long_enough,
                        detail="run length %.3f s" % (float(t[-1] - t[0])
                                                      if len(t) else 0.0)),
          ])


# --- T1.2 --------------------------------------------------------------------


def _t1_2(v, z, traj):
    z0 = float(z[0])
    dz = np.abs(z - z0)
    max_dz = float(dz.max())
    z_min = float(z.min())
    ok = bool(max_dz <= MAX_ABS_DZ_M and z_min > MIN_Z_M)
    v.add("T1.2", ok, "it stayed up",
          measured={"settled start z (m)": round(z0, 4),
                    "largest |z - start z| over all samples (m)":
                        round(max_dz, 4),
                    "lowest z over all samples (m)": round(z_min, 4),
                    "z at the end (m)": round(float(z[-1]), 4)},
          threshold={"|z - start z| (m)": "<= %.2f" % MAX_ABS_DZ_M,
                     "z (m)": "> %.2f" % MIN_Z_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "stayed at its ride height",
              "one sample more than %.2f m from the settled start height, or "
              "at or below %.2f m" % (MAX_ABS_DZ_M, MIN_Z_M),
              "a vertical series with at least two samples",
              traj.n_samples >= 2,
              detail="%d samples" % traj.n_samples)])


# --- T1.3 --------------------------------------------------------------------


def _t1_3(v, ev, xy, xyz, traj):
    start = (float(xy[0][0]), float(xy[0][1]))
    straight = ev.waypoint.error_from(start)
    path = ph.path_length(xy)
    ratio = (path / straight) if straight > 0 else None
    jump = max_step_jump(xyz)
    dt = traj.dt_s

    ratio_ok = (ratio is not None
                and MIN_PATH_RATIO <= ratio <= MAX_PATH_RATIO)
    jump_ok = jump <= MAX_STEP_JUMP_M
    ok = bool(ratio_ok and jump_ok)

    rate_witness = bool(dt is not None and float(dt) <= MAX_SAMPLE_DT_S
                        and traj.n_samples >= 2)
    v.add("T1.3", ok, "it drove there, it did not jump",
          measured={"straight-line start to commanded point (m)":
                        round(straight, 4),
                    "integrated path length (m)": round(path, 4),
                    "path length / straight line":
                        (None if ratio is None else round(ratio, 4)),
                    "largest single inter-sample displacement (m)":
                        round(jump, 4),
                    "sample interval (s)":
                        (None if dt is None else round(float(dt), 5))},
          threshold={"ratio": "%.1f .. %.1f" % (MIN_PATH_RATIO,
                                                MAX_PATH_RATIO),
                     "inter-sample displacement (m)":
                         "<= %.2f" % MAX_STEP_JUMP_M},
          basis=CORE_PHYSICAL,
          detail=("" if straight >= MIN_GOAL_DISTANCE_M else
                  "the commanded point is only %.3f m from the start, so the "
                  "ratio is not a stable measurement" % straight),
          falsifiers=[
              Falsifier("took a plausible route",
                        "a path shorter than the straight line (it did not "
                        "travel) or more than %.1fx it (it wandered)"
                        % MAX_PATH_RATIO,
                        "a start at least %.2f m from the commanded point, "
                        "so a ratio is defined" % MIN_GOAL_DISTANCE_M,
                        bool(straight >= MIN_GOAL_DISTANCE_M),
                        detail="straight-line distance %.3f m" % straight),
              Falsifier("did not jump",
                        "one inter-sample displacement over %.2f m -- a body "
                        "moved by something other than its own motion"
                        % MAX_STEP_JUMP_M,
                        "a pose series sampled at most every %.3f s, so a "
                        "jump is distinguishable from ordinary travel"
                        % MAX_SAMPLE_DT_S,
                        rate_witness,
                        detail=("sample interval %s s"
                                % ("unknown" if dt is None
                                   else "%.5f" % float(dt)))),
          ])


# --- T1.4 --------------------------------------------------------------------


def _t1_4(v, ev, row):
    """Contact with a distinct non-robot body, while the robot was moving.

    The name is the whole assertion and it is deliberately narrower than the
    row's old one (*"under its own actuation"*, renamed 2026-08-02 -- see the
    module docstring). Nothing here can tell a robot that drove from a robot
    that was carried, and no shipped column supplies the channel that could,
    so the row states what it measures instead of what a reader would like it
    to mean.
    """
    s = ev.support
    support = s.support_pairs
    hits, speeds = moving_support_contacts(ev, row)
    timed = s.timed
    # With no timestamps the during-motion half cannot be asked; the tier
    # still requires a support contact, so that half is graded and the other
    # is reported vacuous rather than waved through.
    ok = bool(support) and (bool(hits) if timed else True)
    v.add("T1.4", ok,
          "it touched the surface while it was moving",
          measured={"contacts naming the robot and a non-robot surface":
                        len(support),
                    "...of those, seen while the robot was moving":
                        (len(hits) if timed else None),
                    "surfaces it touched": s.surface_bodies,
                    "fastest speed at a support contact (m/s)":
                        (round(max(speeds), 4) if speeds else None),
                    "contacts of any kind observed": s.total_observed,
                    "contacts naming two distinct bodies": s.distinct_named},
          threshold={"support contacts": ">= 1",
                     "of those, while moving at >= %.2f m/s"
                     % MOVING_SPEED_EPS_MPS: ">= 1"},
          basis=MIXED,
          attested=(["support contacts: %s" % s.source] if s.source else []),
          falsifiers=[
              Falsifier("touched the surface",
                        "a body that reached the commanded point without "
                        "ever touching anything -- flown, dragged by a "
                        "constraint, or teleported",
                        "a contact query over the travel window that "
                        "demonstrably named two distinct bodies at least "
                        "once", s.can_name_a_support_pair,
                        detail=s.witness_detail),
              Falsifier("touched it while moving",
                        "contact only at rest -- a body that sat on the "
                        "surface, was moved by something else, and sat down "
                        "again",
                        "at least one contact carries a simulated timestamp "
                        "on the same clock as the pose series", timed,
                        detail=("%d of %d contacts are timestamped"
                                % (sum(1 for c in s.pairs
                                       if c.t_s is not None), len(s.pairs)))),
          ])


# --- T1.5 --------------------------------------------------------------------


def _t1_5(v, b):
    """The shared clause, with T1's id. Body in ``ladder_evidence``."""
    return run_is_real(v, "T1.5", b)


__all__ = ["TASK", "grade", "select_robot", "series_row_for", "trailing_dwell",
           "max_step_jump", "speed_at", "moving_support_contacts",
           "ARRIVE_TOL_M", "HOLD_TOL_M", "DWELL_S", "MAX_ABS_DZ_M", "MIN_Z_M",
           "MIN_PATH_RATIO", "MAX_PATH_RATIO", "MAX_STEP_JUMP_M",
           "MAX_SAMPLE_DT_S", "MOVING_SPEED_EPS_MPS", "MIN_GOAL_DISTANCE_M"]
