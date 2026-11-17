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

"""T4 ``humanoid`` -- the **sim-neutral core** of the fourth ladder rung.

The tier statement is ``docs/developer/capability-ladder-plan.md`` §2 T4, as
amended 2026-08-02, and it is stated there in one line::

    T3.1, T3.2, T3.3, T3.5 unchanged. T3.4 is REPLACED by the support
    measurement below.

So four of this tier's five rows are **not written here**. They are
:mod:`ladder.graders.t3_core`'s own row builders, **called**::

    T4.1  ten metres        t3_core._t3_1   net horizontal displacement >= 10.0
                                            m in one continuous run
    T4.2  never fell        t3_core._t3_2   base z >= 0.60 x its settled
                                            standing z at every sample, and
                                            |roll|, |pitch| <= 0.8 rad
    T4.3  walked            t3_core._t3_3   mean speed made good >= 0.15 m/s,
                                            >= 0.005 m RMS of vertical
                                            oscillation about its trend, >= 8
                                            make-and-break ground contacts
    T4.4  the support is    THIS MODULE     nothing but the ground touched it,
          measured                          AND what was applied is measured
                                            per channel -- which SELECTS THE
                                            CELL and never fails the row
    T4.5  the run is real   t3_core._t3_5   finalize on independent evidence,
                                            zero error-class lines, the engine
                                            attributed, the driver attested to
                                            have loaded

**"Unchanged" is implemented as "the same function", not as "the same
numbers".** A copy of T3.2 that happened to agree today is a copy that drifts
tomorrow, and the plan's word is *unchanged*. ``test_t4_core`` asserts it
mechanically: it grades one run through both cores and requires the four
reused rows to come out field for field identical to T3's, id excepted. Every
threshold those rows grade against is likewise ``t3_core``'s own module
global, **imported and not restated** -- a second copy here would be a number
that looks binding and is not, because the reused functions read T3's.

--------------------------------------------------------------------------
What this tier actually adds: two cells from one recording
--------------------------------------------------------------------------

§2 T4 permits a support rig and measures it instead of asserting its absence,
and it publishes the result as **two cells**:

===================  =====================================================
``T4-unsupported``   peak non-gravitational, non-contact force
                     <= 0.02 x m.g **and** <= 2 N.m -- numerically nothing
``T4-supported``     anything larger, published **with** the measured peak
                     force as a multiple of body weight, the peak applied
                     torque in N.m, and the fraction of the walk window
                     during which support was non-zero, printed inside the
                     cell
===================  =====================================================

**A supported run is not a failure of the tier.** That is the single most
consequential reading in this file and it is the plan's, not an inference:
*"the task permits a support harness, and every cell measures the support
instead of asserting its absence"*, and the worked example cell it prints by
hand is an ``achieved``. So T4.4's applied-wrench half is a **router**: it
decides which cell a run is published in and it can never turn the row red.
What can turn the row red is the other half, which T4 keeps: a contact with a
body that is not the ground. A robot leaning on a wall, dragged along a rail
or held by another body is not walking, and no force channel would see it.

The obvious objection -- that a robot carried outright would then pass -- is
answered by the other rows rather than by this one, and deliberately: T4.3
requires a gait (a bob about the trend, and make-and-break ground contact),
T4.1 requires continuity, and the **cell prints the number**. A run held up at
2.09 x body weight for 100 % of the window is published saying exactly that,
which is a stronger disclosure than a bare failure would have been and is what
§8 forbids omitting.

Seven readings this file makes, declared so they can be attacked
---------------------------------------------------------------

**1. T3.4's contact half survives; only its wrench half is replaced.** §2 T4
says T3.4 is "replaced by the support measurement", and the two-cell table
that follows is stated purely in force and torque units. Read strictly, the
contact clause would vanish -- and a humanoid propped against scene geometry
would land in ``T4-unsupported``, the cell the plan says must be "numerically
nothing". Keeping it costs no column anything (it is the clause it already
had one rung down) and it is what stops the unsupported cell from being
reachable by a robot that was physically held. **This is a reading, not a
transcription**, and it is declared in the task's ``meta.json`` for a reviewer
to attack before the freeze.

**2. The wrench is measured over the SCORED WINDOW, not over the recording.**
T3 measures its wrench half over whatever window the column attested, because
there a wrench outside the walk is still a wrench somebody applied. T4 cannot
do that: its published figure is *"the fraction of the walk window during
which support was non-zero"*, and the only long humanoid walk this tree has
measured spent two hundred seconds pressed against a wall after the walk
ended. A fraction averaged over that is not the fraction the plan asks for.
Where the column's wrench log carries no clock the window cannot be applied,
and the profile says so rather than pretending.

**3. Support is measured PER CHANNEL.** Measured, on this tree's own rig over
a whole 10 m walk: the vertical carry channel read **0.00 N for 0 %** of the
window while the attitude channel ran at a **69.2 N.m peak for 100 %** of it.
One scalar reports that rig as "36.8 N, non-zero 100 % of the window" and
hides the entire finding -- a reader cannot tell a rig that carries a robot
from one that only keeps it from toppling. A column that can total the wrench
but not stream it per axis still counts as attested and still lands in a cell;
its row says the profile could not be resolved per channel, and that is a
weaker cell rather than a rejected one.

**4. A column that cannot attest the wrench lands in NEITHER cell.** §2 T4:
*"Where a simulator cannot attest applied forces, the cell is ``achieved
(support unverified)`` and is excluded from comparison with an attested
cell."* So it passes, loudly: the clause reports its witness absent, the cell
is named ``T4-support-unverified`` -- deliberately not one of the two -- and
the exclusion is in the row. Failing it would publish our own missing channel
as somebody else's capability gap, which §4 forbids; crediting it silently is
what §5c exists to prevent.

**5. A run bound by the edge of the world is one that was AT the edge, not
one that stopped there.** This is where T4 strengthens T3's rule rather than
reusing it, and the reason is the measured shape of the run the rule exists
for: ``docs/developer/g1-endurance-2026-08-01.md`` §4 records `x` frozen at
12.90 m against a wall while `y` ran out to -6.88 m over two hundred seconds
-- the robot was pressed against the world and **still locomoting**, so a rule
keyed on "it stopped" can miss it entirely. :func:`arena_bound_index` asks
only that the base has been within :data:`ARENA_MARGIN_M` of the region's edge
for the last :data:`PLATEAU_WINDOW_S` of the recording. The guard against
using this to hide a bad gait is unchanged from one rung down and is the
conjunction itself: the column must **positively attest** where the region
ends, and a plateau in open floor is a stall, is named one, and is scored in
full.

**6. The two headroom fields are measurements and may never become pass
conditions.** ``distance_to_termination_m`` is the furthest the base actually
got, un-clipped at the bar, and ``termination_cause`` is one of five values
from the plan's own fixed vocabulary. They exist so a 10.1 m run and a 29.5 m
run are distinguishable in the published grid, and so that a run which stopped
because it **ran out of floor** is never read as a walk that degraded -- which
is exactly how six clean measured runs would have been misread. §8 forbids
quoting a T4 cell without the cause.

**7. ``reuse_class`` is not decidable here and the verdict says so**, exactly
as one rung down: §9 Q3 records that its boundary is fuzzy and that the
reviewer arbitrates, so the field is present, null, and loud.

**And one thing this file will not do.** ``AGENTS.md``'s humanoid disclosure
rule binds every statement about a supported humanoid walk, and no grader can
enforce prose. What it can do is make the number impossible to lose: the
support figures are written into **every** verdict, including the ones that
fail before any physics is read, and :meth:`SupportProfile.cell_text` renders
the sentence fragment a cell is published with.
"""

from __future__ import annotations

import numpy as np

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier,
                                        Verdict)
from ladder.graders import t3_core
from ladder.graders.t3_core import (  # noqa: F401  (the tier's own numbers)
    ARENA_MARGIN_M, FALL_Z_FRACTION, MAX_SAMPLE_DT_S, MAX_STEP_JUMP_M,
    MAX_SUPPORT_FORCE_FRACTION, MAX_SUPPORT_TORQUE_NM, MAX_TILT_RAD,
    MIN_BOB_RMS_M, MIN_CONTACT_TRANSITIONS, MIN_MEAN_SPEED_MPS,
    MIN_RUN_UP_FACTOR, NET_DISPLACEMENT_M, PLATEAU_TOL_M, PLATEAU_WINDOW_S)
from ladder.graders.t4_evidence import (ATTESTED, CELL_SUPPORTED,
                                        CELL_UNSUPPORTED, CELL_UNVERIFIED,
                                        TERMINATION_ARENA,
                                        TERMINATION_CONTROLLER,
                                        TERMINATION_FELL,
                                        TERMINATION_TIME_LIMIT,
                                        TERMINATION_UNKNOWN, UNVERIFIED,
                                        classify_contacts, support_profile)

TASK = "T4_humanoid"

# --- the rows this tier does not write ---------------------------------------
#
# ``capability-ladder-plan.md`` §2 T4: "T3.1, T3.2, T3.3, T3.5 unchanged."
# Implemented as the same function rather than the same text, and recorded as
# data so the task file, the test and a reader see one list.
REUSED_ROWS = {"T4.1": "T3.1", "T4.2": "T3.2", "T4.3": "T3.3", "T4.5": "T3.5"}

_ALL = [("T4.1", "it crossed ten metres in one continuous run"),
        ("T4.2", "it never fell"),
        ("T4.3", "it walked; it did not slide or drift"),
        ("T4.4", "nothing but the ground touched it, and what was applied to "
                 "it was measured"),
        ("T4.5", "the run is real")]

_BASIS = {"T4.1": CORE_PHYSICAL, "T4.2": CORE_PHYSICAL,
          "T4.3": CORE_PHYSICAL, "T4.4": MIXED, "T4.5": MIXED}

# Reading 5. T3's own sentence for this branch says the base "stopped" at the
# edge; the run this tier's rule exists for was still moving when the world
# ended it, so the sentence is the one thing that differs.
ARENA_WINDOW_RULE = ("the sample at which the base became bound by the edge "
                     "of the walking region -- the world ended this run, so "
                     "what came after it is not part of the walk")


# --- where the walk ends ------------------------------------------------------


def arena_bound_index(t, xy, arena, *, margin=ARENA_MARGIN_M,
                      window_s=PLATEAU_WINDOW_S):
    """The sample the run became bound by the edge of the world, or ``None``.

    Reading 5 in the module docstring. Two things must hold and the
    conjunction is the guard: the column must **positively attest** where the
    walking region ends, and the base must have been within ``margin`` of that
    edge for the whole trailing ``window_s`` of the recording. It deliberately
    does **not** ask whether the base stopped -- a body pressed against a wall
    and still locomoting sideways is bound by geometry, and that is the
    measured shape of every long humanoid walk this tree has recorded.

    A plateau in open floor satisfies neither condition and is a stall.
    """
    if arena is None or not arena.usable:
        return None
    t = np.asarray(t, dtype=float)
    xy = np.asarray(xy, dtype=float)
    if len(t) < 2 or len(xy) != len(t):
        return None
    d = np.array([arena.distance_to_boundary(p) for p in xy], dtype=float)
    if not np.isfinite(d).all() or float(d[-1]) > float(margin):
        return None
    i = len(t) - 1
    while i > 0 and float(d[i - 1]) <= float(margin):
        i -= 1
    if float(t[-1] - t[i]) < float(window_s):
        return None
    return int(i)


def scored_end(t, disp, arena_idx, *, bar=NET_DISPLACEMENT_M):
    """``(index, rule)`` -- where the window T4.3 is measured over ends.

    The **choice** -- the bar, then the region's edge, then the run's end -- is
    T3's function, called and not copied: it is the most consequential reading
    in either tier and two copies of it would drift. Only the arena branch's
    sentence is T4's, for the reason :func:`arena_bound_index` states.
    """
    i, rule = t3_core.scored_end(t, disp, arena_idx, bar=bar)
    reached = bool(len(np.asarray(disp)) and
                   bool((np.asarray(disp, dtype=float) >= float(bar)).any()))
    if arena_idx is not None and not reached and i == int(arena_idx):
        rule = ARENA_WINDOW_RULE
    return i, rule


# --- the grade ---------------------------------------------------------------


def grade(ev, *, self_verified=False):
    """Grade one T4 run from neutral evidence. Never raises."""
    b = ev.bundle
    v = Verdict(TASK, self_verified=self_verified)
    for note in list(getattr(b, "notes", []) or []) + list(ev.notes or []):
        v.note(note)

    # Computed once here over whatever the column logged, so that a verdict
    # which never reaches a pose series still carries the figures §8 forbids
    # omitting; recomputed over the walk itself the moment the walk is known.
    profile = _profile(ev)
    _record_the_fields_no_cell_may_omit(v, ev, profile)

    # ---- the deliverable -------------------------------------------------
    if b.artifact is None:
        v.progress = NO_ARTIFACT
        return _all_red(v, "no deliverable was produced")
    v.artifacts["deliverable"] = str(b.artifact)

    if not ev.surface.usable:
        v.outcome = INVALID
        v.note("the task did not name the ground the robot walks on, so "
               "'nothing but the ground touched it' has nothing to be "
               "measured against. The names are task data and a missing one "
               "is a broken instrument, never a failed run.")
        return _all_red(v, "the task named no walking surface", finish=False)

    base = ev.base_pose
    if not base.usable:
        v.progress = ARTIFACT_INVALID if b.live_load_ok is False \
            else ARTIFACT_RUNS
        return _all_red(v, base.error or "no pose series for the robot's base")

    problems = t3_core.series_naming_problems(ev)
    if problems:
        v.progress = ARTIFACT_RUNS
        return _all_red(v, "; ".join(problems))

    v.progress = ARTIFACT_RUNS
    v.measurements.update(b.adapter_measurements.get("motion") or {})

    body, why = t3_core.select_base_body(ev)
    v.measurements["graded_body"] = {
        "selection": why, "name": (body.name if body else None),
        "id": (body.body_id if body else None)}

    t = np.asarray(base.t, dtype=float)
    xyz = np.asarray(base.xyz, dtype=float)
    xy = xyz[:, :2]
    disp = t3_core.displacement_from_start(xy)

    # The window every speed, bob, transition count and support figure below is
    # measured over, computed ONCE.
    arena_idx = arena_bound_index(t, xy, ev.arena)
    i1, window_rule = scored_end(t, disp, arena_idx)
    i1 = int(min(max(i1, 0), len(t) - 1))
    _record_window(v, ev, t, xy, disp, i1, window_rule, arena_idx)

    # Reading 2: the published fraction is "of the walk window", so the profile
    # the row grades and the cell publishes is the one restricted to it.
    profile = _profile(ev, float(t[0]), float(t[i1]))
    _write_support(v, ev, profile)

    _reused(v, t3_core._t3_1, "T4.1", ev, t, xyz, disp)
    fell_at = _reused(v, t3_core._t3_2, "T4.2", ev, t, xyz)
    _reused(v, t3_core._t3_3, "T4.3", ev, t, xyz, disp, i1)
    _t4_4(v, ev, t, i1, profile)
    attributed = _reused(v, t3_core._t3_5, "T4.5", ev, b)

    _record_termination(v, ev, t, xy, disp, arena_idx, fell_at, i1)

    if not attributed:
        v.outcome = INVALID
        v.note("no physics-engine attribution: the adapter could not name the "
               "engine that drove this run from any channel it has. An "
               "unattributed physics result is not a result.")
        return v
    return v.finish(GRADED_PASS)


def _profile(ev, t_lo=None, t_hi=None):
    """The applied wrench, read into a profile against **this** tier's bounds.

    The two numbers are passed in from one place so the cell boundary and the
    row's printed threshold cannot disagree.
    """
    return support_profile(ev.support, ev.body_weight_n, t_lo, t_hi,
                           force_fraction=MAX_SUPPORT_FORCE_FRACTION,
                           torque_limit_nm=MAX_SUPPORT_TORQUE_NM)


def _reused(v, fn, aid, *args):
    """Grade one of the four rows §2 T4 states as **unchanged**, by calling it.

    The row that lands in the verdict is the other tier's row -- its
    thresholds, its measured keys, its falsifiers and its wording -- with its
    label changed to this tier's. Relabelling rather than re-implementing is
    the whole point: if that row is ever corrected, this tier inherits the
    correction instead of drifting away from it, which is what the plan's word
    "unchanged" has to mean to be worth writing down.
    """
    out = fn(v, *args)
    v.assertions[-1].id = aid
    return out


def _record_the_fields_no_cell_may_omit(v, ev, profile):
    """The labels §8 forbids quoting a T4 cell without.

    Recorded in EVERY verdict, including the ones that fail before any physics
    is read: if no prose sentence about a cell may omit them, no cell may omit
    them. The support figures here are over whatever the column logged; they
    are rewritten over the walk itself as soon as the walk is known.
    """
    _write_support(v, ev, profile)
    v.measurements["method"] = ev.controller.method
    v.measurements["controller"] = ev.controller.as_dict()
    v.measurements["walking_surface"] = ev.surface.as_dict()
    v.measurements["arena"] = ev.arena.as_dict()
    v.measurements["provenance"] = ev.provenance()
    # "The field was missing" and "the run ended at the clock" are different
    # statements, so both fields are present from the start.
    v.measurements["distance_to_termination_m"] = None
    v.measurements["termination_cause"] = TERMINATION_UNKNOWN
    # §9 Q3: the boundary is fuzzy and the reviewer arbitrates. A grader that
    # guessed would be inventing the one label the plan says a human owns.
    v.measurements["reuse_class"] = None
    v.note("reuse_class is not decidable from evidence and is left null: it "
           "asks whether the behaviour would exist without work done during "
           "the run, which is a judgement about the session and not a "
           "measurement of it. A cell published with it still null is not "
           "publishable.")

    if profile.cell == CELL_SUPPORTED:
        v.note("this run is published in the %s cell, and every sentence "
               "about it carries its own numbers: %s. A supported run is not "
               "a failed one -- it is a different cell -- and it may not be "
               "quoted without the figures."
               % (CELL_SUPPORTED, profile.cell_text()))
    elif profile.cell == CELL_UNVERIFIED:
        v.note("support_attestation: unverified -- %s. This run lands in "
               "NEITHER published cell: it is excluded from comparison with "
               "any cell that has an attested total, and it is never credited "
               "as if nothing had been holding the robot up."
               % (profile.error or "no applied-wrench total reached the "
                                   "grader"))
    if profile.attested and not profile.per_channel_attested:
        v.note("the applied wrench was attested as a summary rather than as a "
               "per-axis series, so this cell cannot say WHICH channel held "
               "the robot up. The only real support rig this tree has "
               "measured was idle on its carrying channel for the whole walk "
               "while its attitude channel ran continuously, and a single "
               "peak cannot distinguish those two.")
    if not ev.arena.usable:
        v.note("no walking-region extent reached the grader, so a run that "
               "ended on world geometry cannot be told from one that stalled "
               "and the task's stated free run-up could not be checked. On "
               "this tier that makes the cell INCOMPLETE rather than merely "
               "under-annotated, and it is excluded from comparison for that "
               "reason as well.")


def _write_support(v, ev, profile):
    """The support fields, and the two exclusions. Idempotent, no notes."""
    v.measurements["cell"] = profile.cell
    v.measurements["cell_text"] = profile.cell_text()
    v.measurements["support_attestation"] = (ATTESTED if profile.attested
                                             else UNVERIFIED)
    v.measurements["external_support"] = profile.as_dict()
    v.measurements["arena_attestation"] = (ATTESTED if ev.arena.usable
                                           else UNVERIFIED)
    reasons = []
    if not profile.attested:
        reasons.append("the applied wrench was not attested, so this run is "
                       "in neither published cell and is not comparable with "
                       "one that has a total")
    if not ev.arena.usable:
        reasons.append("the extent of the walking region was not attested, so "
                       "a run ended by world geometry is indistinguishable "
                       "from one that stalled")
    v.measurements["comparison_exclusions"] = reasons
    v.measurements["excluded_from_comparison"] = bool(reasons)


def _record_window(v, ev, t, xy, disp, i1, rule, arena_idx):
    """The scored window and the free run-up, printed in every verdict.

    The same measurement one rung down makes, with the same constants and this
    tier's own row id in the note it writes.
    """
    start = xy[0] if len(xy) else (0.0, 0.0)
    run_up = ev.arena.free_run_up_from(start)
    required = MIN_RUN_UP_FACTOR * NET_DISPLACEMENT_M
    v.measurements["scored_window"] = {
        "from (s)": (round(float(t[0]), 3) if len(t) else None),
        "to (s)": (round(float(t[i1]), 3) if len(t) else None),
        "how the end was chosen": rule,
        "distance from the start at the window's end (m)":
            (round(float(disp[i1]), 4) if len(disp) else None),
        "the whole recording is (s)":
            (round(float(t[-1] - t[0]), 3) if len(t) else None)}
    v.measurements["arena"] = dict(ev.arena.as_dict(), **{
        "longest straight run available from the start (m)":
            (None if run_up is None else round(float(run_up), 3)),
        "the free run-up this task requires (m)": round(float(required), 3),
        "the run-up requirement was met":
            (None if run_up is None else bool(run_up >= required)),
        "the run became bound by the region's edge": arena_idx is not None})
    if run_up is not None and run_up < required:
        v.note("the free walking region offers %.2f m of straight run from "
               "where this robot started, against the %.1f m this task states "
               "as its minimum. A run in it is geometry-bound by "
               "construction, and a red T4.1 here is a finding about the "
               "world the deliverable built, not about the gait."
               % (run_up, required))


def _record_termination(v, ev, t, xy, disp, arena_idx, fell_at, i1):
    """How the run ended, and how far it actually got. Never pass conditions.

    The plan's own two fields and its own five-value vocabulary, adopted and
    not paraphrased. The order is deliberate: a robot that went down went
    down, whatever it was next to; otherwise running out of world is the
    reading the endurance measurement says a grader must be able to state; and
    a stop that is **not** at a boundary is the thing driving the robot having
    stopped, and is named that rather than blamed on the world.

    One sentence differs from the same record one rung down, for the reason
    :func:`arena_bound_index` states: the arena branch reports the base as
    having been **at** the edge, not as having stopped there, because the run
    this branch exists for was still moving.
    """
    secs, i0 = t3_core.trailing_plateau(t, xy, PLATEAU_TOL_M)
    if len(t) < 2:
        how, why = TERMINATION_UNKNOWN, "the run produced too few samples to say"
    elif fell_at is not None:
        how, why = TERMINATION_FELL, ("the fall test was breached at t = "
                                      "%.3f s" % float(t[fell_at]))
    elif arena_idx is not None:
        how, why = TERMINATION_ARENA, (
            "the base was within %.2f m of the edge of the walking region for "
            "the last %.1f s of the recording -- whether or not it was still "
            "moving, the world was what it had left"
            % (float(ev.arena.distance_to_boundary(xy[-1]) or 0.0),
               float(t[-1] - t[int(arena_idx)])))
    elif secs >= PLATEAU_WINDOW_S and i0 is not None and i0 > 0:
        how, why = TERMINATION_CONTROLLER, (
            "the base stopped for the last %.1f s, and not at any edge this "
            "column attested" % secs)
    else:
        how, why = TERMINATION_TIME_LIMIT, ("the clock ran out while it was "
                                            "moving")
    # Un-clipped, whatever it was: a 10.1 m run and a 29.5 m run are different
    # findings and the bar must not flatten them into one.
    v.measurements["distance_to_termination_m"] = (
        round(float(disp.max()), 4) if len(disp) else None)
    v.measurements["termination_cause"] = how
    v.measurements["termination"] = {
        "how the run ended": how, "why": why,
        "the base did not move more than %.2f m for the last (s)"
        % PLATEAU_TOL_M: round(secs, 3),
        "graded up to sample": int(i1)}
    if how == TERMINATION_ARENA:
        v.note("the world, not the walker, ended this run. Every number over "
               "the scored window excludes the stretch after it, and a grader "
               "blind to the boundary would have published this walk as a "
               "gait that degraded -- which is precisely how the only long "
               "humanoid walk this tree has measured would have been read.")


def _all_red(v, why, *, finish=True):
    for aid, what in _ALL:
        v.add(aid, False, what, detail=why, basis=_BASIS[aid],
              falsifiers=[Falsifier(
                  "evidence present",
                  "a run that produced nothing to measure",
                  "a deliverable, the ground the task names, and a pose "
                  "series for the robot's base over the run", False,
                  detail=why)])
    return v.finish() if finish else v


# --- T4.4 --------------------------------------------------------------------


def _t4_4(v, ev, t, i1, profile):
    """The tier's own row: what touched it, and what was applied to it.

    Red **only** on a contact with a body that is not the ground, inside the
    scored window. The wrench selects the cell and never fails the row --
    reading 1 and the module docstring's two-cell table.
    """
    ground, foreign, disagreements = classify_contacts(ev.gait, ev.surface)
    lo, hi = (float(t[0]), float(t[i1])) if len(t) else (0.0, 0.0)
    # A contact after the walk has ended -- the robot resting against whatever
    # stopped it -- is an arena finding, not a support finding. An UNTIMED
    # foreign contact counts as inside, which is the strict direction: an
    # adapter that cannot say when something touched the robot does not get the
    # benefit of the doubt.
    marks = [bool(c.t_s is None or lo <= float(c.t_s) <= hi) for c in foreign]
    inside = [c for c, m in zip(foreign, marks) if m]
    after = [c for c, m in zip(foreign, marks) if not m]
    touched = []
    for c in inside:
        if c.other_body and c.other_body not in touched:
            touched.append(c.other_body)
    bounds = {str(x) for x in (ev.arena.boundary_bodies or [])}
    at_the_edge = [x for x in touched if x in bounds]
    ok = not inside

    v.add("T4.4", ok, "nothing but the ground touched it, and what was "
                      "applied to it was measured",
          measured={"contacts with a body that is not the ground, inside the "
                    "scored window": len(inside),
                    "which bodies those were": touched,
                    "...of those, bodies the walking region is bounded by":
                        at_the_edge,
                    "the same after the walk had ended -- an arena finding, "
                    "not a support finding": len(after),
                    "contacts with the ground": len(ground),
                    "the cell this run is published in": profile.cell,
                    "the cell, as it must be printed": profile.cell_text(),
                    "support attestation":
                        (ATTESTED if profile.attested else UNVERIFIED),
                    "peak applied force (N)": _r(profile.peak_force_n),
                    "...as a multiple of body weight":
                        _r(profile.peak_force_x_weight, 5),
                    "the force bound of the %s cell (N)" % CELL_UNSUPPORTED:
                        _r(profile.force_limit_n),
                    "peak applied torque (N.m)": _r(profile.peak_torque_nm),
                    "fraction of the scored window a force was applied":
                        _r(profile.force_fraction_nonzero, 5),
                    "fraction of the scored window a torque was applied":
                        _r(profile.torque_fraction_nonzero, 5),
                    "fraction of the scored window anything was applied":
                        _r(profile.any_fraction_nonzero, 5),
                    "per channel": profile.per_axis,
                    "measured over the walk rather than the whole recording":
                        profile.window_applied,
                    "body weight, m.g (N)": _r(profile.body_weight_n, 3),
                    "the adapter and the task disagreed about the ground":
                        disagreements},
          threshold={"contacts with anything but the ground": 0,
                     "the applied wrench": "MEASURED, not bounded -- it "
                                           "selects the cell (%s at <= %.2f x "
                                           "m.g and <= %.1f N.m, %s above "
                                           "either) and never fails this row"
                                           % (CELL_UNSUPPORTED,
                                              MAX_SUPPORT_FORCE_FRACTION,
                                              MAX_SUPPORT_TORQUE_NM,
                                              CELL_SUPPORTED)},
          basis=MIXED,
          attested=([x for x in [
              ("ground contacts: %s" % ev.gait.source)
              if ev.gait.source else None,
              ("applied support: %s" % profile.source)
              if profile.source else None,
              ("body mass: %s" % ev.base_physics.source)
              if ev.base_physics.source else None,
          ] if x]),
          detail=_t4_4_detail(profile),
          falsifiers=[
              Falsifier("nothing but the ground touched it",
                        "a robot leaning on a wall, propped on a crate, "
                        "dragged along a rail or held by another body -- a "
                        "contact naming a body the task does not declare as "
                        "the ground. No force channel would ever see it, "
                        "which is why this half of the clause survives on a "
                        "tier that permits support",
                        "a contact query that demonstrably named two distinct "
                        "bodies AND said, for each, whether the other side is "
                        "the ground",
                        bool(ev.gait.can_name_a_pair and ev.gait.classified),
                        detail=ev.gait.witness_detail),
              Falsifier("what was applied to it was measured",
                        "a column that cannot total the non-gravitational, "
                        "non-contact wrench on the base: it can say neither "
                        "that nothing held the robot up nor by how much "
                        "something did, so the run lands in neither published "
                        "cell",
                        "the applied wrench, positively attested, AND the "
                        "base's mass and the world's gravity -- the cell "
                        "boundary is 0.02 x m.g and the published figure is a "
                        "multiple of body weight, and without m.g there is "
                        "neither",
                        profile.attested,
                        detail=(profile.error or profile.source or "")),
              Falsifier("measured per channel",
                        "a rig whose carrying channel is idle while its "
                        "attitude channel runs continuously -- reported as "
                        "one peak and one fraction, it reads as a robot being "
                        "held up, and the two claims are not the same claim",
                        "the applied wrench as a per-axis series on the "
                        "simulated clock, so force and torque are separable "
                        "and each axis has its own peak and its own fraction "
                        "of the window",
                        profile.per_channel_attested,
                        detail=("per-axis series present"
                                if profile.per_channel_attested else
                                "the column summarised the wrench instead of "
                                "streaming it")),
          ])


def _t4_4_detail(profile):
    if not profile.attested:
        return ("support unverified -- this run is in neither published cell, "
                "is excluded from comparison with any cell that has an "
                "attested total, and the wrench clause below could not have "
                "failed")
    if profile.cell == CELL_SUPPORTED:
        return ("published as %s (%s); a supported run is a different cell, "
                "not a failed one, and may not be quoted without those "
                "figures" % (CELL_SUPPORTED, profile.cell_text()))
    return ""


def _r(x, digits=4):
    return None if x is None else round(float(x), digits)


__all__ = ["TASK", "grade", "arena_bound_index", "scored_end", "REUSED_ROWS",
           "ARENA_WINDOW_RULE",
           "NET_DISPLACEMENT_M", "FALL_Z_FRACTION", "MAX_TILT_RAD",
           "MIN_MEAN_SPEED_MPS", "MIN_BOB_RMS_M", "MIN_CONTACT_TRANSITIONS",
           "MAX_SUPPORT_FORCE_FRACTION", "MAX_SUPPORT_TORQUE_NM",
           "MAX_SAMPLE_DT_S", "MAX_STEP_JUMP_M", "MIN_RUN_UP_FACTOR",
           "ARENA_MARGIN_M", "PLATEAU_WINDOW_S", "PLATEAU_TOL_M"]
