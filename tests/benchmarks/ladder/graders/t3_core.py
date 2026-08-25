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

"""T3 ``quadruped`` -- the **sim-neutral core** of the third ladder rung.

The tier statement is ``docs/developer/capability-ladder-plan.md`` §2 T3, and
its five rows are transcribed here threshold for threshold::

    T3.1  ten metres        net horizontal displacement of the base >= 10.0 m
                            in a single continuous run (no reset, no restart)
    T3.2  never fell        base z >= 0.60 x its settled standing z at EVERY
                            recorded sample, and |roll|, |pitch| <= 0.8 rad at
                            every sample. The standing z is read per robot from
                            the t0 inventory, not fixed as a constant
    T3.3  walked, did not   mean forward speed >= 0.15 m/s over the window;
          slide or drift    AND >= 8 distinct make-and-break transitions of
                            (robot body, ground) contact, with the vacuity
                            counters present. The tier's middle clause -- the
                            base's vertical position oscillating with
                            >= 0.005 m RMS about its trend -- is MEASURED AND
                            PRINTED IN EVERY ROW AND IS NO LONGER A PASS
                            CONDITION (decided 2026-08-02; reading 5b below,
                            and the number it used to be compared against is
                            printed beside it in the row)
    T3.4  nothing held it   no contact with any body other than the ground, AND
          up                total non-gravitational, non-contact force applied
                            to the base <= 0.02 x m.g peak and <= 2 N.m of
                            applied torque. Where a simulator cannot attest the
                            applied-force total, the cell records
                            support_attestation: unverified and is EXCLUDED
                            from comparison with any cell that has it
    T3.5  the run is real   as T1.5, and the policy/controller is asserted to
                            have actually loaded (an exit code is not evidence)

Everything here is metres, seconds, radians, kilograms and newton-seconds of
nothing else. The core reads a neutral evidence bundle plus T3's own channels
(:mod:`ladder.graders.t3_evidence`) and nothing else: it cannot open a scene
file, cannot name a node type, and cannot ask which engine it is talking to
except through the attribution the adapter cited.

**The method is recorded, never graded.** ``method`` goes into every verdict's
measurements -- including the ones that fail before any physics is read -- and
**no assertion branches on it**. The plan states it as a field rather than a
condition (§3.2): *"a walk is a walk; how it was reached is a fact readers want
and is never a pass condition"*, and §2 T3 says the same of training in as many
words. ``test_t3_core`` asserts the independence mechanically, by grading one
run under all four method values and requiring byte-identical assertions.

**``reuse_class`` is NOT decidable here and the verdict says so.** §8 forbids
quoting a T3 cell without it, and §9 Q3 records that its boundary is fuzzy and
*"the reviewer arbitrates"*. So every verdict carries the field, carries it as
``None``, and carries a note saying a cell published with it still null is not
publishable. A grader that guessed would be inventing the one label the plan
says a human owns.

Seven readings the tier table leaves open, decided here and stated so they can
be attacked
--------------------------------------------------------------------------

**1. "Net horizontal displacement" is graded as the FURTHEST the base got from
its start, not its position at the end.** The measured alternative fails a real
run: this tree's own flagship walks past the bar and is then pushed sideways
along a wall for two hundred seconds
(``docs/developer/g1-endurance-2026-08-01.md`` §4), and an end-of-run reading
would score whatever the wall left it at. Both numbers are printed; the
furthest is the one graded. The clause cannot be exploited by a body that is
*thrown* ten metres and returns, because the continuity clause below and T3.2
both bite on that.

**2. T3.1's "single continuous run" is graded, and the tier does not say how.**
A reset is a clock that goes backwards; a restart is a body that reappears
somewhere else. So T3.1 carries a **derived** second clause -- the clock must
be strictly increasing and no single inter-sample displacement may exceed
:data:`MAX_STEP_JUMP_M`, which is T1.3's own number with T1.3's own rate
witness. It is derived, not transcribed, it is declared in the task's
``meta.json`` for a reviewer to attack before the freeze, and it is what the
``teleported`` fixture reddens.

**3. "The window" in T3.3 is the SCORED WINDOW, and defining it is the single
most consequential reading in this file.** It runs from the first sample to
whichever comes first: the sample at which the displacement bar is reached, the
sample at which the run became bound by the edge of the walking region, or the
run's end. The reason is measured, not theoretical --
``g1-endurance-2026-08-01.md`` §8: *"The world, not the policy, ended every
single run ... all six runs here would have been misread as 'the walk degrades
to 0.037 m/s after ~110 s' by a grader that did not know where the walls
were."* A mean speed taken over a recording that includes two hundred seconds
of a robot pinned against a wall is not the walking speed and must never be
published as one.

The obvious objection is that truncating the window could hide a gait that
degraded. Two guards: the truncation happens **only** where the arena channel
positively attests a boundary and the base is within
:data:`ARENA_MARGIN_M` of it (a plateau in open floor is a **stall** and is
scored in full), and the whole-run mean is printed beside the scored one,
labelled as not being the walking speed.

**4. "Mean forward speed" is speed made good, not speed along the body's own
nose.** No simulator-neutral definition of "forward" exists for an arbitrary
robot -- which body axis points forward is a modelling convention, and reading
it out of a rotation matrix would be reading our own convention into somebody
else's model. So it is (distance from the start at the window's end) / (the
window's elapsed time): a robot that walks a straight line scores its walking
speed, and one that walks in circles scores what it actually made good, which
is the honest answer to *"did it cross ten metres"*.

**5. "Oscillates about its trend" is the RMS of the residual from a
least-squares line** over the scored window. A bare standard deviation would
count a slow sag or a slow climb as a bob; a line removes exactly that and
leaves the gait. A body that slid scores zero.

**5b. That bob is MEASURED IN EVERY ROW AND GRADED IN NONE.** *(Decided
2026-08-02, before the freeze; the defect it repairs was demonstrated first --*
``tasks/T3_quadruped/meta.json`` *->*
``container.authored_here.what_the_demonstration_found_about_THIS_ASSET``
*and* ``adapters/mujoco/BRINGUP_T3.md`` *5.2.)* The clause failed a robot that
was plainly walking. Measured on this container's own achievability oracle: a
slow, correct, **statically stable** crawl covers **16.5 m at 0.27 m/s without
falling**, with **733 make-and-break ground-contact transitions**, and bobs
**0.0018 m RMS** -- red against 0.005 m. The same gait only reaches 0.0089 m
once it is driven to 0.49 m/s. A rigid body carried at a constant commanded
height does not bob much no matter how honestly it is walking, so the clause
was not measuring *gait*, it was measuring *how dynamic the gait is*. That is a
false negative of the same shape as the T2.3 defect repaired the same day: a
predicate that does not test what its sentence says.

So **ground-contact make/breaks remain the gate for "it stepped rather than
slid" and the bob becomes a reported measurement.** The reasoning, stated so it
can be attacked: the bob's failure mode is *demonstrated and real*, while the
failure it was guarding against -- a slider producing >= 8 make/breaks **while
also** covering 10 m at >= 0.15 m/s -- is much harder to produce, and T3.1 and
the speed clause already exclude it acting together. A body that is dragged has
no make/breaks at all (that is exactly what the ``slid_without_gait`` fixture
is), and a body that cycles contacts without going anywhere fails the speed
floor.

**NO THRESHOLD WAS MOVED.** :data:`MIN_CONTACT_TRANSITIONS` is still 8,
:data:`MIN_MEAN_SPEED_MPS` is still 0.15, and :data:`MIN_BOB_RMS_M` is still
0.005 -- it is retained, still computed, still printed, and printed **beside
the row's measured bob** with the label saying it is the bar the clause used to
apply, so a reader can re-apply the old reading by eye rather than having to
diff a grader. Any row that passes only because of this change says so in its
own ``detail``. What the tier's own *text* still states is the pre-repair
reading; that divergence is declared in the task file and in the plan's T3
section rather than left in the code.

**6. The make-and-break count is PER CONTACTING BODY, and an aggregate count
would be zero on a perfect walk.** A walking quadruped always has at least one
foot on the ground, so the aggregate "is this robot touching the ground" signal
never changes state. Counted per body, each footfall is a transition. Every
change of state -- a make **or** a break -- counts as one transition against
the tier's ``>= 8``; the number of *completed* lift-and-place cycles is
reported beside it as a measurement and is never the threshold.

**7. T3.4's contact half is measured over the SCORED WINDOW.** The robot in
reading 3 ends the run resting against the boundary it stopped at, and that
contact is not evidence about the walk that was graded. Foreign contacts
outside the window are counted and printed as an arena finding; the ones inside
it redden the clause. The applied-wrench half is measured over whatever window
the column attested, because a wrench that was applied outside the walk is
still a wrench somebody applied.

Robot selection
---------------

T3 grades one robot's base. The base pose series arrives keyed by the name the
robot description declares -- task data, exactly as T1's waypoint and T2's
roles are, and for the same reason: an adapter that could choose which body
counts as "the base" could choose the body that travelled furthest. A series
that says it is about some other body is refused, loudly, rather than graded.
"""

from __future__ import annotations

import numpy as np

from agentbench.graders import physical as ph
from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier,
                                        Verdict)
from ladder.graders.t3_evidence import (ATTESTED, TERMINATION_ARENA,
                                        TERMINATION_CONTROLLER,
                                        TERMINATION_FELL,
                                        TERMINATION_TIME_LIMIT,
                                        TERMINATION_UNKNOWN, UNVERIFIED,
                                        classify_contacts, contact_states,
                                        detrended_rms, max_step_jump,
                                        roll_pitch, run_is_real, tilt_from_up)

TASK = "T3_quadruped"

# --- the tier's thresholds, quoted so a drift is visible in a diff ----------
NET_DISPLACEMENT_M = 10.0          # T3.1
FALL_Z_FRACTION = 0.60             # T3.2  of the settled standing height
MAX_TILT_RAD = 0.8                 # T3.2  |roll| and |pitch|, separately
MIN_MEAN_SPEED_MPS = 0.15          # T3.3  over the scored window
# T3.3, about the trend -- MEASURED AND REPORTED IN EVERY ROW, NEVER GRADED
# since 2026-08-02 (reading 5b). The value is the tier's own and is
# deliberately NOT deleted: it is printed beside the measured bob as the bar
# the clause used to apply, which is what lets a reader re-derive the old
# verdict.
MIN_BOB_RMS_M = 0.005
MIN_CONTACT_TRANSITIONS = 8        # T3.3  make-and-break, per contacting body
MAX_SUPPORT_FORCE_FRACTION = 0.02  # T3.4  x m.g, peak
MAX_SUPPORT_TORQUE_NM = 2.0        # T3.4  peak

# Not tier constants -- witness bounds the tier's wording implies, plus the one
# derived clause reading 2 declares and the run-up rule the build note in
# ``g1-endurance-2026-08-01.md`` §8 requires. Each decides whether a clause is
# a measurement or a formality, so each is named rather than assumed.
MAX_SAMPLE_DT_S = 0.05        # coarser than this and a gait cycle is invisible
MAX_STEP_JUMP_M = 0.5         # T3.1's derived clause; T1.3's number
MIN_RUN_UP_FACTOR = 1.5       # the stated free run-up, as a multiple of the bar
ARENA_MARGIN_M = 0.5          # this close to the edge is "out of world"
PLATEAU_WINDOW_S = 5.0        # a stop must last this long to be a stop
PLATEAU_TOL_M = 0.25          # ...and move less than this while it lasts

_ALL = [("T3.1", "it crossed ten metres in one continuous run"),
        ("T3.2", "it never fell"),
        ("T3.3", "it walked; it did not slide or drift"),
        ("T3.4", "nothing held it up"),
        ("T3.5", "the run is real")]

_BASIS = {"T3.1": CORE_PHYSICAL, "T3.2": CORE_PHYSICAL,
          "T3.3": CORE_PHYSICAL, "T3.4": MIXED, "T3.5": MIXED}


# --- selection ---------------------------------------------------------------


def series_naming_problems(ev):
    """The base series claiming to be about a body other than the one named.

    An adapter that hands the core the wrong body's motion produces a verdict
    about the wrong thing, and every number in it looks fine. An empty ``body``
    is *"the adapter did not say"* and is allowed; a non-empty mismatch is
    refused.
    """
    got = (ev.base_pose.body or "").strip()
    want = (ev.robot_name or "").strip()
    if got and want and got != want:
        # ...unless the base was resolved by mass because this column's
        # importer does not reproduce the declared name. Then the series is
        # about the right body under a different name, and refusing would
        # grade the naming convention rather than the robot.
        resolved, _why = select_base_body(ev)
        if resolved is not None and (resolved.name or "").strip() == got:
            return []
        return ["the base series is about %r, but the task names %r"
                % (got, want)]
    return []


#: How close a candidate's mass must be to the declared mass, as a fraction,
#: before it may stand in for a base the importer did not name. Tight on
#: purpose: it is an identity check, not a tolerance. A robot's root body and
#: its limbs differ by far more than this, so the rule cannot pick a thigh.
BASE_MASS_MATCH_REL = 0.01


def select_base_body(ev):
    """``(body, why)`` for the inventory entry the task names as the base.

    Preferred: the body carrying the declared name. Failing that, **the body
    whose mass matches the declared mass**, which is an identity check rather
    than a guess -- and is the difference between measuring a column and
    refusing to.

    Why the fallback exists. The tier declares the base by the root-link name
    of its own description (``base_link``). A converter is free not to
    reproduce that: one folds the root link into the robot node and names it
    after the robot, another prefixes or namespaces every link. Measured, the
    shipped quadruped description imports as a body named ``walker`` with no
    ``base_link`` anywhere -- carrying the declared 10.8 kg exactly. Refusing
    that run scores the CONVERTER'S NAMING CONVENTION and reports it as the
    robot failing to walk, which is a verdict about the wrong thing.

    Requiring a mass match keeps it honest: it cannot be satisfied by any body
    but the one the task means, the reason rides in the ``why`` string, and a
    column with no declared mass gets the old refusal rather than a guess.
    """
    name = (ev.robot_name or "").strip()
    if not name:
        return None, "the task named no body for the robot under test"
    invs = (ev.bundle.t0, ev.bundle.roster)
    for inv in invs:
        for b in (getattr(inv, "bodies", None) or []):
            if b.name == name:
                return b, ("the body carrying the name the robot description "
                           "declares (%r), from %s"
                           % (name, inv.source or "an unattributed inventory"))

    declared = getattr(ev, "robot_mass_kg_declared", None)
    if declared is not None and float(declared) > 0.0:
        declared = float(declared)
        hits = []
        for inv in invs:
            for b in (getattr(inv, "bodies", None) or []):
                m = getattr(b, "mass_kg", None)
                if m is None:
                    continue
                if abs(float(m) - declared) <= BASE_MASS_MATCH_REL * declared:
                    hits.append((b, inv))
        # Exactly one, or the check has not identified anything.
        uniq = {id(b): (b, inv) for b, inv in hits}
        if len(uniq) == 1:
            b, inv = next(iter(uniq.values()))
            return b, ("no body is named %r -- this column's importer does not "
                       "reproduce the declared link name -- so the base was "
                       "resolved by MASS: %r carries %.4f kg against the "
                       "declared %.4f kg (within %.0f%%), uniquely, from %s"
                       % (name, b.name, float(b.mass_kg), declared,
                          BASE_MASS_MATCH_REL * 100.0,
                          inv.source or "an unattributed inventory"))
        if len(uniq) > 1:
            return None, ("no body is named %r, and the mass fallback is "
                          "AMBIGUOUS: %d bodies match the declared %.4f kg "
                          "(%s). Refusing rather than picking one"
                          % (name, len(uniq), declared,
                             ", ".join(sorted(b.name for b, _ in uniq.values()))))

    return None, ("no body named %r is in the run's inventory, so the facts "
                  "the tier asks for about it cannot be read%s" % (
                      name,
                      "" if declared is None else
                      (" -- and no body matches the declared %.4f kg either"
                       % declared)))


# --- measurement helpers (plain numbers in, plain numbers out) ---------------


def displacement_from_start(xy):
    """``(N,)`` horizontal distance from the first sample, metres."""
    xy = np.asarray(xy, dtype=float)
    if not len(xy):
        return np.zeros(0, dtype=float)
    d = xy - xy[0]
    return np.hypot(d[:, 0], d[:, 1])


def trailing_plateau(t, xy, tol_m):
    """``(seconds, index)`` of the trailing stretch that barely moved.

    The maximal run of samples at the end of the recording whose horizontal
    position stays within ``tol_m`` of the final one. It is the shape of a
    robot that has stopped, whatever stopped it.
    """
    t = np.asarray(t, dtype=float)
    xy = np.asarray(xy, dtype=float)
    n = len(t)
    if n < 2:
        return 0.0, (n - 1 if n else None)
    d = np.hypot(xy[:, 0] - xy[-1, 0], xy[:, 1] - xy[-1, 1])
    i = n - 1
    while i > 0 and d[i - 1] <= float(tol_m):
        i -= 1
    return float(t[-1] - t[i]), int(i)


def arena_stop_index(t, xy, arena, *, margin=ARENA_MARGIN_M,
                     window_s=PLATEAU_WINDOW_S, tol_m=PLATEAU_TOL_M):
    """The sample the run became bound by the edge of the world, or ``None``.

    Three things must all hold, and the conjunction is the guard against using
    this to hide a bad gait: the column must positively attest where the
    walking region ends, the base must have **stopped** there for at least
    ``window_s``, and it must be within ``margin`` of that edge. A plateau in
    open floor satisfies two of the three and is a stall, scored in full.
    """
    if arena is None or not arena.usable:
        return None
    secs, i0 = trailing_plateau(t, xy, tol_m)
    if i0 is None or secs < float(window_s):
        return None
    edge = arena.distance_to_boundary(np.asarray(xy, dtype=float)[-1])
    if edge is None or edge > float(margin):
        return None
    return int(i0)


def scored_end(t, disp, arena_idx, *, bar=NET_DISPLACEMENT_M):
    """``(index, rule)`` -- where the window T3.3 is measured over ends.

    Reading 3 in the module docstring. The bar wins over the arena when both
    apply, which is automatic: a run that reached the bar reached it before it
    ran out of world.
    """
    disp = np.asarray(disp, dtype=float)
    hit = np.nonzero(disp >= float(bar))[0]
    if len(hit):
        return int(hit[0]), ("the sample at which the displacement bar was "
                             "first reached")
    if arena_idx is not None and int(arena_idx) > 0:
        return int(arena_idx), ("the sample at which the base stopped at the "
                                "edge of the walking region -- the world ended "
                                "this run, so what came after it is not part "
                                "of the walk")
    return (len(t) - 1 if len(t) else 0), "the run's last sample"


def contact_transitions(state):
    """``(transitions, cycles, per_body)`` over an (M, K) contact-state array.

    Reading 6. ``transitions`` counts every change of state, a make **or** a
    break, summed over bodies -- that is what the tier's ``>= 8`` is graded
    against. ``cycles`` counts completed lift-and-place events per body
    (``min(makes, breaks)``) and is reported, never thresholded.
    """
    if state is None:
        return None, None, {}
    s = np.asarray(state, dtype=bool)
    if s.ndim != 2 or s.shape[0] < 2 or s.shape[1] == 0:
        return 0, 0, {}
    d = np.diff(s.astype(np.int8), axis=0)
    makes = (d == 1).sum(axis=0)
    breaks = (d == -1).sum(axis=0)
    per = {i: {"makes": int(makes[i]), "breaks": int(breaks[i])}
           for i in range(s.shape[1])}
    return (int(makes.sum() + breaks.sum()),
            int(np.minimum(makes, breaks).sum()), per)


def within(t, i0, i1):
    """Boolean mask over ``t`` for the closed window ``[t[i0], t[i1]]``."""
    t = np.asarray(t, dtype=float)
    if not len(t):
        return np.zeros(0, dtype=bool)
    return (t >= float(t[i0])) & (t <= float(t[i1]))


# --- the grade ---------------------------------------------------------------


def grade(ev, *, self_verified=False):
    """Grade one T3 run from neutral evidence. Never raises."""
    b = ev.bundle
    v = Verdict(TASK, self_verified=self_verified)
    for note in list(getattr(b, "notes", []) or []) + list(ev.notes or []):
        v.note(note)
    _record_the_fields_no_cell_may_omit(v, ev)

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

    problems = series_naming_problems(ev)
    if problems:
        v.progress = ARTIFACT_RUNS
        return _all_red(v, "; ".join(problems))

    v.progress = ARTIFACT_RUNS
    v.measurements.update(b.adapter_measurements.get("motion") or {})

    body, why = select_base_body(ev)
    v.measurements["graded_body"] = {
        "selection": why, "name": (body.name if body else None),
        "id": (body.body_id if body else None)}

    t = np.asarray(base.t, dtype=float)
    xyz = np.asarray(base.xyz, dtype=float)
    xy = xyz[:, :2]
    disp = displacement_from_start(xy)

    # The window every speed, bob and transition count below is measured over,
    # computed ONCE. Two copies of this would be two readings of "which part of
    # the recording is the walk", and they would drift.
    arena_idx = arena_stop_index(t, xy, ev.arena)
    i1, window_rule = scored_end(t, disp, arena_idx)
    i1 = int(min(max(i1, 0), len(t) - 1))
    _record_window(v, ev, t, xy, disp, i1, window_rule, arena_idx)

    _t3_1(v, ev, t, xyz, disp)
    fell_at = _t3_2(v, ev, t, xyz)
    _t3_3(v, ev, t, xyz, disp, i1)
    _t3_4(v, ev, t, i1)
    attributed = _t3_5(v, ev, b)

    _record_termination(v, ev, t, xy, disp, arena_idx, fell_at, i1)

    if not attributed:
        v.outcome = INVALID
        v.note("no physics-engine attribution: the adapter could not name the "
               "engine that drove this run from any channel it has. An "
               "unattributed physics result is not a result.")
        return v
    return v.finish(GRADED_PASS)


def _record_the_fields_no_cell_may_omit(v, ev):
    """The labels §8 forbids quoting a T3 cell without.

    Recorded in EVERY verdict, including the ones that fail before any physics
    is read, for the reason ``hold_mechanism`` is recorded in every T2 verdict:
    if no prose sentence about a cell may omit them, no cell may omit them.
    """
    attested = bool(ev.support.usable and ev.body_weight_n is not None)
    v.measurements["support_attestation"] = ATTESTED if attested else UNVERIFIED
    v.measurements["external_support"] = ev.support.as_dict()
    v.measurements["excluded_from_comparison"] = (not attested)
    v.measurements["method"] = ev.controller.method
    v.measurements["controller"] = ev.controller.as_dict()
    v.measurements["walking_surface"] = ev.surface.as_dict()
    v.measurements["arena"] = ev.arena.as_dict()
    v.measurements["provenance"] = ev.provenance()
    # Present in every verdict, including the ones that never reach a pose
    # series: §8 forbids quoting a cell of this shape without what ended the
    # run, and "the field was missing" and "the run ended at the clock" are
    # different statements.
    v.measurements["distance_to_termination_m"] = None
    v.measurements["termination_cause"] = TERMINATION_UNKNOWN
    # §9 Q3: the boundary is fuzzy and the reviewer arbitrates. A grader that
    # guessed would be inventing the one label the plan says a human owns --
    # so the field is present, null, and loud.
    v.measurements["reuse_class"] = None
    v.note("reuse_class is not decidable from evidence and is left null: it "
           "asks whether the behaviour would exist without work done during "
           "the run, which is a judgement about the session and not a "
           "measurement of it. A cell published with it still null is not "
           "publishable.")
    if not attested:
        if ev.support.usable:
            why = ("the applied wrench was attested, but the base's mass or "
                   "the world's gravity was not, so there is no 0.02 x m.g "
                   "for it to be compared against")
        else:
            why = (ev.support.error
                   or "no applied-wrench total reached the grader")
        v.note("support_attestation: unverified -- %s. The cell is EXCLUDED "
               "from comparison with any cell that has an attested support "
               "total, and is never credited as if nothing had been holding "
               "the robot up." % why)


def _record_window(v, ev, t, xy, disp, i1, rule, arena_idx):
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
               "construction, and a red T3.1 here is a finding about the "
               "world the deliverable built, not about the gait."
               % (run_up, required))


def _record_termination(v, ev, t, xy, disp, arena_idx, fell_at, i1):
    """How the run ended, and how far it actually got. Never pass conditions.

    The two fields ``capability-ladder-plan.md`` §3.2 states -- with its own
    vocabulary and its own names, adopted rather than paraphrased. They exist
    so that *"it ran out of floor"* is never read as *"it degraded"*, which
    §2 T4 records is exactly how six clean endurance runs would have been
    misread.

    The order is deliberate. A robot that went down went down, whatever it was
    next to. Otherwise, running out of world is the reading the endurance
    measurement says a recorder must be able to state, and a stop that is NOT
    at a boundary is the thing driving the robot having stopped, and is named
    that rather than blamed on the world.
    """
    secs, i0 = trailing_plateau(t, xy, PLATEAU_TOL_M)
    if len(t) < 2:
        how, why = TERMINATION_UNKNOWN, "the run produced too few samples to say"
    elif fell_at is not None:
        how, why = TERMINATION_FELL, ("the fall test was breached at t = "
                                      "%.3f s" % float(t[fell_at]))
    elif arena_idx is not None:
        how, why = TERMINATION_ARENA, (
            "the base stopped %.2f m from the edge of the walking region and "
            "stayed there for %.1f s"
            % (float(ev.arena.distance_to_boundary(xy[-1]) or 0.0), secs))
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
               "gait that degraded.")


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


# --- T3.1 --------------------------------------------------------------------


def _t3_1(v, ev, t, xyz, disp):
    furthest = float(disp.max()) if len(disp) else 0.0
    at_end = float(disp[-1]) if len(disp) else 0.0
    far_enough = bool(furthest >= NET_DISPLACEMENT_M)

    jump = max_step_jump(xyz)
    dt_all = np.diff(t)
    clock_ok = bool(len(dt_all) and bool((dt_all > 0.0).all()))
    continuous = bool(jump <= MAX_STEP_JUMP_M and clock_ok)
    ok = bool(far_enough and continuous)

    dt = ev.base_pose.dt_s
    rate_witness = bool(dt is not None and float(dt) <= MAX_SAMPLE_DT_S
                        and ev.base_pose.n_samples >= 2)
    run_up = ev.arena.free_run_up_from(xyz[0][:2] if len(xyz) else (0.0, 0.0))
    # A region smaller than the bar makes this clause unfalsifiable in the
    # other direction: it could not have passed. That is a statement about the
    # world the deliverable built, and it belongs in the row.
    room_witness = bool(run_up is None or run_up >= NET_DISPLACEMENT_M)

    v.add("T3.1", ok, "it crossed ten metres in one continuous run",
          measured={"furthest from the start reached (m)": round(furthest, 4),
                    "distance from the start at the end (m)": round(at_end, 4),
                    "integrated path length (m)":
                        round(ph.path_length(xyz[:, :2]), 4),
                    "largest single inter-sample displacement (m)":
                        round(float(jump), 4),
                    "the clock never went backwards": clock_ok,
                    "sample interval (s)":
                        (None if dt is None else round(float(dt), 5)),
                    "longest straight run the region allowed (m)":
                        (None if run_up is None else round(float(run_up), 3))},
          threshold={"net horizontal displacement (m)":
                         ">= %.1f" % NET_DISPLACEMENT_M,
                     "inter-sample displacement (m)":
                         "<= %.2f" % MAX_STEP_JUMP_M,
                     "the clock": "strictly increasing"},
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("crossed ten metres",
                        "a robot that stopped, fell or wandered short of "
                        "%.1f m from where it started" % NET_DISPLACEMENT_M,
                        "a walking region with at least %.1f m of straight "
                        "run available from the start, so the distance was "
                        "reachable at all" % NET_DISPLACEMENT_M,
                        room_witness,
                        detail=("the region allows %s m of straight run"
                                % ("an unknown length of" if run_up is None
                                   else "%.2f" % run_up))),
              Falsifier("one continuous run",
                        "a clock that goes backwards, or one inter-sample "
                        "displacement over %.2f m -- a body moved by "
                        "something other than its own legs, or a run stitched "
                        "together from two" % MAX_STEP_JUMP_M,
                        "a pose series sampled at most every %.3f s, so a "
                        "jump is distinguishable from ordinary travel"
                        % MAX_SAMPLE_DT_S,
                        rate_witness,
                        detail=("sample interval %s s"
                                % ("unknown" if dt is None
                                   else "%.5f" % float(dt)))),
          ])


# --- T3.2 --------------------------------------------------------------------


def _t3_2(v, ev, t, xyz):
    """Grade the fall test. Returns the first sample it was breached at."""
    z = xyz[:, 2]
    attested = ev.standing.usable
    standing = (float(ev.standing.z_m) if attested else float(z[0]))
    rule = (ev.standing.source if attested else
            "no settled standing height was attested for this robot, so the "
            "run's own first recorded sample was used -- which is taken after "
            "the settle window, and which a robot that never stood up would "
            "supply from the floor")
    floor_z = FALL_Z_FRACTION * standing
    min_z = float(z.min())
    height_ok = bool(min_z >= floor_z)

    roll, pitch = roll_pitch(ev.base_pose.rot)
    have_attitude = roll is not None and pitch is not None
    max_roll = (float(np.abs(roll).max()) if have_attitude else None)
    max_pitch = (float(np.abs(pitch).max()) if have_attitude else None)
    attitude_ok = bool(have_attitude and max_roll <= MAX_TILT_RAD
                       and max_pitch <= MAX_TILT_RAD)
    tilt = tilt_from_up(ev.base_pose.rot)
    ok = bool(height_ok and attitude_ok)

    bad = (z < floor_z)
    if have_attitude:
        bad = bad | (np.abs(roll) > MAX_TILT_RAD) | (np.abs(pitch)
                                                     > MAX_TILT_RAD)
    hits = np.nonzero(bad)[0]
    first = int(hits[0]) if len(hits) else None

    v.add("T3.2", ok, "it never fell",
          measured={"settled standing height (m)": round(standing, 4),
                    "how the standing height was decided": rule,
                    "lowest base height over all samples (m)": round(min_z, 4),
                    "the height it had to stay above (m)": round(floor_z, 4),
                    "largest |roll| (rad)":
                        (None if max_roll is None else round(max_roll, 4)),
                    "largest |pitch| (rad)":
                        (None if max_pitch is None else round(max_pitch, 4)),
                    # Reported, never graded: the tier bounds roll and pitch
                    # SEPARATELY, so 0.8 rad of each -- about 1.1 rad off
                    # vertical -- satisfies both. Printing it is how a reader
                    # sees that without deriving it.
                    "largest angle off vertical, which is NOT the tier's "
                    "threshold (rad)":
                        (None if tilt is None
                         else round(float(np.abs(tilt).max()), 4)),
                    "first sample it was breached at (s)":
                        (None if first is None else round(float(t[first]), 3)),
                    "samples over the bound": int(len(hits))},
          threshold={"base height (m)": ">= %.2f x the settled standing "
                                        "height" % FALL_Z_FRACTION,
                     "|roll| (rad)": "<= %.1f" % MAX_TILT_RAD,
                     "|pitch| (rad)": "<= %.1f" % MAX_TILT_RAD,
                     "at": "every recorded sample"},
          basis=CORE_PHYSICAL,
          attested=([x for x in [("standing height: %s" % ev.standing.source)
                                 if ev.standing.source else None] if x]),
          falsifiers=[
              Falsifier("stayed at its standing height",
                        "one sample with the base below %.2f x the height it "
                        "settled at -- it went down" % FALL_Z_FRACTION,
                        "a settled standing height attested for THIS robot, "
                        "independently of the run's own samples; without one "
                        "a robot that was already on the floor at t=0 sets "
                        "its own floor and cannot fail",
                        attested,
                        detail=(ev.standing.error or ev.standing.source or "")),
              Falsifier("stayed upright",
                        "one sample past %.1f rad of roll or pitch -- it "
                        "toppled, whether or not the base sank" % MAX_TILT_RAD,
                        "the base's orientation at every sample; the frozen "
                        "motion contract carries none, so a column with no "
                        "ladder-side sampler cannot answer this half at all",
                        have_attitude,
                        detail=("orientation %s"
                                % ("present" if have_attitude else "absent"))),
          ])
    return first


# --- T3.3 --------------------------------------------------------------------


def _t3_3(v, ev, t, xyz, disp, i1):
    elapsed = float(t[i1] - t[0]) if len(t) else 0.0
    reached = float(disp[i1]) if len(disp) else 0.0
    mean_speed = (reached / elapsed) if elapsed > 0.0 else None
    whole = float(t[-1] - t[0]) if len(t) else 0.0
    whole_mean = (float(disp[-1]) / whole) if whole > 0.0 else None
    speed_ok = bool(mean_speed is not None
                    and mean_speed >= MIN_MEAN_SPEED_MPS)

    dt = ev.base_pose.dt_s
    rate_witness = bool(dt is not None and float(dt) <= MAX_SAMPLE_DT_S)
    transitions_witness = bool(ev.gait.can_name_a_pair
                               and ev.gait.has_sample_times)

    # Reading 5b: computed on every run, printed on every row, and part of NO
    # pass condition. ``bob_met_the_retired_bar`` is the verdict the clause
    # would have returned before 2026-08-02, carried in the row so that what
    # changed is visible rather than deleted.
    bob = detrended_rms(t[:i1 + 1], xyz[:i1 + 1, 2])
    bob_met_the_retired_bar = (None if bob is None
                               else bool(bob >= MIN_BOB_RMS_M))

    ground, _foreign, _dis = classify_contacts(ev.gait, ev.surface)
    ts, bodies, state = contact_states(ev.gait, ground)
    if state is not None and len(ts):
        keep = (ts >= float(t[0])) & (ts <= float(t[i1]))
        state = state[keep]
    edges, cycles, _per = contact_transitions(state)
    contact_ok = bool(edges is not None and edges >= MIN_CONTACT_TRANSITIONS)
    ok = bool(speed_ok and contact_ok)

    # The one sentence a reader of a changed row needs, in the row: this cell
    # passes and the pre-repair reading would have failed it.
    changed = ("" if (bob is None or bob_met_the_retired_bar or not ok) else
               "PASSES ONLY UNDER THE 2026-08-02 READING: the vertical bob is "
               "%.5f m against the %.3f m this clause required until then, "
               "and the bob is now measured and never graded. What carries the "
               "'it stepped rather than slid' claim in this row is the %d "
               "make-and-break ground-contact transitions across %d robot "
               "bodies, at %.4f m/s made good."
               % (bob, MIN_BOB_RMS_M, int(edges or 0), len(bodies),
                  float(mean_speed or 0.0)))

    v.add("T3.3", ok, "it walked; it did not slide or drift",
          measured={"mean speed made good over the scored window (m/s)":
                        (None if mean_speed is None else round(mean_speed, 4)),
                    "the scored window is (s)": round(elapsed, 3),
                    "the same over the WHOLE recording, which is NOT the "
                    "walking speed (m/s)":
                        (None if whole_mean is None else round(whole_mean, 4)),
                    "vertical oscillation about its trend, RMS (m)":
                        (None if bob is None else round(bob, 5)),
                    # Reading 5b, printed rather than deleted: the bar this
                    # clause applied until 2026-08-02, and what it would have
                    # said, so the change is legible from the row alone.
                    "...against the bar that was RETIRED on 2026-08-02, which "
                    "is now measured and never graded (m)": MIN_BOB_RMS_M,
                    "...the retired bob clause would have passed":
                        bob_met_the_retired_bar,
                    "the sample interval resolves a gait cycle, so the bob "
                    "above is measured rather than aliased": rate_witness,
                    "make-and-break transitions of ground contact":
                        edges,
                    "...of those, completed lift-and-place cycles": cycles,
                    "robot bodies seen touching the ground": list(bodies),
                    "contacts of any kind observed": ev.gait.total_observed,
                    "contacts naming two distinct bodies":
                        ev.gait.distinct_named,
                    "times the contact query ran":
                        (len(np.asarray(ev.gait.sample_times))
                         if ev.gait.has_sample_times else None)},
          threshold={"mean speed (m/s)": ">= %.2f" % MIN_MEAN_SPEED_MPS,
                     "vertical RMS about the trend (m)":
                         "MEASURED, NEVER GRADED since 2026-08-02 -- the "
                         "tier's text still states >= %.3f and this grader "
                         "does not apply it (reading 5b)" % MIN_BOB_RMS_M,
                     "make-and-break transitions":
                         ">= %d" % MIN_CONTACT_TRANSITIONS},
          basis=CORE_PHYSICAL,
          detail=changed,
          attested=(["ground contacts: %s" % ev.gait.source]
                    if ev.gait.source else []),
          falsifiers=[
              Falsifier("moved at a walking pace",
                        "a robot that took the whole recording to cover the "
                        "ground -- crawling, shuffling, or dragged",
                        "a scored window with a positive elapsed time and at "
                        "least two samples in it",
                        bool(elapsed > 0.0 and i1 >= 1),
                        detail="window %.3f s, %d samples" % (elapsed, i1 + 1)),
              # There is deliberately NO "bobbed like something on legs"
              # falsifier here any more. A falsifier declares a way the
              # assertion can fail, and since 2026-08-02 the bob cannot fail
              # it (reading 5b) -- leaving the clause listed would advertise a
              # check this grader does not perform, which is exactly the shape
              # of over-claim the falsifier machinery exists to prevent. The
              # bob and its retired bar are in the measurements above, and the
              # sample-rate witness that used to hang off this clause is there
              # too, as a caveat on the number rather than as a gate.
              Falsifier("picked its feet up",
                        "a robot whose contact with the ground never breaks "
                        "-- which is what sliding, rolling and being dragged "
                        "all look like",
                        "a contact query that demonstrably named two distinct "
                        "bodies AND reported the times it ran; a list of "
                        "touches alone carries no evidence of a lifted foot",
                        transitions_witness,
                        detail=ev.gait.witness_detail),
          ])


# --- T3.4 --------------------------------------------------------------------


def _t3_4(v, ev, t, i1):
    ground, foreign, disagreements = classify_contacts(ev.gait, ev.surface)
    lo, hi = (float(t[0]), float(t[i1])) if len(t) else (0.0, 0.0)
    # Reading 7: a contact after the walk has ended -- the robot resting
    # against whatever stopped it -- is an arena finding, not evidence about
    # the walk that was graded. An UNTIMED foreign contact is counted as
    # inside, which is the strict direction: an adapter that cannot say when
    # something touched the robot does not get the benefit of the doubt.
    marks = [bool(c.t_s is None or lo <= float(c.t_s) <= hi) for c in foreign]
    inside = [c for c, m in zip(foreign, marks) if m]
    after = [c for c, m in zip(foreign, marks) if not m]
    touched = []
    for c in inside:
        if c.other_body and c.other_body not in touched:
            touched.append(c.other_body)
    clean = not inside

    weight = ev.body_weight_n
    attested = bool(ev.support.usable and weight is not None)
    limit_n = (MAX_SUPPORT_FORCE_FRACTION * weight
               if weight is not None else None)
    peak_f = ev.support.peak_force
    peak_m = ev.support.peak_torque
    if attested:
        wrench_ok = bool(peak_f is not None and peak_m is not None
                         and peak_f <= limit_n
                         and peak_m <= MAX_SUPPORT_TORQUE_NM)
    else:
        # The plan's own consequence: unverified is recorded and EXCLUDED from
        # comparison, not failed and not credited. Failing it would publish our
        # missing channel as somebody else's capability gap; passing it
        # silently is the thing §5c exists to prevent -- so it passes loudly,
        # with the clause's witness absent and the exclusion in the row.
        wrench_ok = True
    ok = bool(clean and wrench_ok)

    v.add("T3.4", ok, "nothing held it up",
          measured={"contacts with a body that is not the ground, inside the "
                    "scored window": len(inside),
                    "which bodies those were": touched,
                    "the same after the walk had ended -- an arena finding, "
                    "not a support finding": len(after),
                    "contacts with the ground": len(ground),
                    "support attestation":
                        (ATTESTED if attested else UNVERIFIED),
                    "peak applied force (N)":
                        (None if peak_f is None else round(float(peak_f), 4)),
                    "...as a multiple of body weight":
                        (None if (peak_f is None or not weight)
                         else round(float(peak_f) / float(weight), 5)),
                    "the force it had to stay under (N)":
                        (None if limit_n is None else round(limit_n, 4)),
                    "peak applied torque (N.m)":
                        (None if peak_m is None else round(float(peak_m), 4)),
                    "fraction of the window anything was applied":
                        ev.support.fraction_window_nonzero,
                    "per axis": ev.support.per_channel,
                    "body weight, m.g (N)":
                        (None if weight is None else round(weight, 3)),
                    "the adapter and the task disagreed about the ground":
                        disagreements},
          threshold={"contacts with anything but the ground": 0,
                     "peak applied force": "<= %.2f x m.g"
                                           % MAX_SUPPORT_FORCE_FRACTION,
                     "peak applied torque (N.m)":
                         "<= %.1f" % MAX_SUPPORT_TORQUE_NM},
          basis=MIXED,
          attested=([x for x in [
              ("ground contacts: %s" % ev.gait.source)
              if ev.gait.source else None,
              ("applied support: %s" % ev.support.source)
              if ev.support.source else None,
              ("body mass: %s" % ev.base_physics.source)
              if ev.base_physics.source else None,
          ] if x]),
          detail=("" if attested else
                  "support_attestation: unverified -- this cell is excluded "
                  "from comparison with any cell that has an attested support "
                  "total, and the clause below could not have failed"),
          falsifiers=[
              Falsifier("nothing but the ground touched it",
                        "a robot leaning on a wall, propped on a crate or "
                        "held by another robot -- a contact naming a body the "
                        "task does not declare as the ground",
                        "a contact query that demonstrably named two distinct "
                        "bodies AND said, for each, whether the other side is "
                        "the ground",
                        bool(ev.gait.can_name_a_pair and ev.gait.classified),
                        detail=ev.gait.witness_detail),
              Falsifier("nothing pulled or pushed on it",
                        "a body carried by an applied force -- a lift, a "
                        "spring, an attitude torque, anything that is neither "
                        "gravity nor a contact",
                        "the total non-gravitational, non-contact wrench on "
                        "the base, positively attested, AND the base's mass "
                        "and the world's gravity so a peak in force units has "
                        "a 0.02 x m.g to be compared against",
                        attested,
                        detail=(ev.support.error or ev.support.source
                                or ("the wrench was attested; the body weight "
                                    "was not"))),
          ])


# --- T3.5 --------------------------------------------------------------------


def _t3_5(v, ev, b):
    """T1.5's shared body, plus the one clause T3 adds to it.

    ``run_is_real`` lives in the shared channel module and is called, not
    forked: the plan states the assertion once for T1 and then says *"as
    T1.5"*, and two copies of it would be two copies of a threshold. T3's
    extra requirement -- *"the policy/controller is asserted to have actually
    loaded"* -- is appended to the assertion that helper produced, which keeps
    one clause in one place and still lets a reader see both in one row.
    """
    attributed = run_is_real(v, "T3.5", b)
    a = v.assertions[-1]
    c = ev.controller
    loaded = bool(c.loaded is True)

    a.measured["the policy or controller is attested to have loaded"] = c.loaded
    a.measured["what loaded"] = (c.identity or None)
    a.measured["the evidence that was read"] = (c.evidence or None)
    a.threshold["the policy or controller loaded"] = True
    a.ok = bool(a.ok and loaded)
    if c.source:
        a.attested.append("controller load: %s" % c.source)
    a.falsifiers.append(Falsifier(
        "the thing that drives the robot actually loaded",
        "a deploy whose model runtime was missing from the interpreter that "
        "spawns controllers: it runs a zero-residual baseline, moves nothing, "
        "and EXITS 0 -- so the exit code, the error stream and the finalize "
        "marker all read clean",
        "a positive attestation that the policy or controller loaded, naming "
        "the evidence it was read from. An exit code is not one, and neither "
        "is the absence of an error",
        c.attested,
        detail=(c.error or c.source or "")))
    if not loaded:
        bits = [x for x in (a.detail, ) if x]
        bits.append("the policy or controller was %s"
                    % ("attested NOT to have loaded -- the run produced "
                       "motion from something other than the deliverable, or "
                       "no motion at all" if c.loaded is False else
                       "never attested to have loaded, and an exit code is "
                       "not evidence that it did"))
        a.detail = "; ".join(bits)
    return attributed


__all__ = ["TASK", "grade", "series_naming_problems", "select_base_body",
           "displacement_from_start", "trailing_plateau", "arena_stop_index",
           "scored_end", "contact_transitions", "within",
           "NET_DISPLACEMENT_M", "FALL_Z_FRACTION", "MAX_TILT_RAD",
           "MIN_MEAN_SPEED_MPS", "MIN_BOB_RMS_M", "MIN_CONTACT_TRANSITIONS",
           "MAX_SUPPORT_FORCE_FRACTION", "MAX_SUPPORT_TORQUE_NM",
           "MAX_SAMPLE_DT_S", "MAX_STEP_JUMP_M", "MIN_RUN_UP_FACTOR",
           "ARENA_MARGIN_M", "PLATEAU_WINDOW_S", "PLATEAU_TOL_M"]
