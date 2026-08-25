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

"""T2 ``transfer`` -- the **sim-neutral core** of the second ladder rung.

The tier statement is ``docs/developer/capability-ladder-plan.md`` §2 T2, and
its five rows are transcribed here threshold for threshold::

    T2.1  transferred          the object's centre STARTS OUTSIDE the
                               container's world-space AABB and ENDS INSIDE it,
                               its lowest point below the container rim, and it
                               is at rest (speed < 0.02 m/s averaged over the
                               last 1.0 s)
    T2.2  lifted, not dragged  the object's lowest point is >= 0.05 m above
                               every static support surface for a continuous
                               >= 3.0 s
    T2.3  held ten seconds     over a continuous >= 10.0 s, the object's
                               position IN THE END-EFFECTOR'S FRAME varies by
                               <= 0.02 m RMS, max excursion <= 0.04 m
    T2.4  the object is a real body   mass > 0, gravity acting, dynamic, and no
                               single inter-sample displacement > 0.5 m at a
                               sample interval <= 0.05 s
    T2.5  the run is real      as T1.5

Everything here is metres, seconds and kilograms. The core reads a neutral
evidence bundle plus the ladder's own channels
(:mod:`ladder.graders.ladder_evidence`) and nothing else: it cannot open a
scene file, cannot name a node type, and cannot ask which engine it is talking
to except through the attribution the adapter cited.

**The mechanism is recorded, never graded.** ``hold_mechanism`` goes into
every verdict's measurements -- including the ones that fail before any
physics is read -- and **no assertion branches on it**. The plan states the
reasoning (§2 T2) and it is worth repeating where the code is: this tree's own
shipped bin-picking success uses a suction end-effector after finger-gripper
attempts topped out, and an earlier carry demo used a grasp-stabilisation
weld. Failing an attachment-based grasp would be writing the task around a
technique we ourselves abandoned. ``test_t2_core`` asserts the independence
mechanically, by grading one run under all four mechanism values and requiring
byte-identical assertions.

Five readings the tier table leaves open, decided here and stated so they can
be attacked -- plus two clauses (**2b** and **4b**) that were added before the
freeze, each after it had been *demonstrated* missing on a run that graded
PASS without it
--------------------------------------------------------------------------

**1. "The container rim" is not a field any simulator has.** An open-topped
box's rim is the top of its bounding box; a box with a lid or a raised handle
has a rim well below it. The core therefore takes the rim from the evidence
channel when the adapter can attest one, and otherwise falls back to the top
of the container's own AABB -- the **permissive** reading, which can only make
"below the rim" easier to satisfy. Which of the two was used is printed in the
row (``rim_rule``), and the fallback marks the clause's witness absent, so a
cell can never quietly claim to have measured a rim it never saw.

**2. "The object's lowest point" needs the object's size.** The pose series
carries a centre. The lowest point is the centre minus the object's own
vertical half-extent, read from its t=0 bounding box and applied to the centre
at every sample -- which assumes the object is not tumbling. A rotating object
therefore has its lowest point mis-stated by up to (half-diagonal minus
half-height), and for the 5 cm cube the container ships that is 2.3 cm. Where
no bounding box reaches the core at all, the clauses grade the **centre**, say
so, and report their witness absent.

**2b. A transfer has a FROM as well as a TO, and nothing used to read it.**
*(Clause decided 2026-08-02, before the freeze; the defect it fixes was
demonstrated first.)* T2 grades an END STATE (2.1), a LIFT (2.2) and a HOLD
(2.3), and until this clause existed **not one of the five assertions read
where the object began** -- so a run that picks the object *out* of the
container, holds it twelve seconds in the air above it and puts it back graded
**PASS 5/5**. Every row was honest about what it measured; the tier was simply
not asking. The tier's own OUTCOME sentence is *"an object that starts on a
surface ends up inside a container"*, so the intent was never ambiguous, and a
grader that cannot tell a transfer from a lift-and-replace is not grading the
sentence it quotes.

So T2.1 carries a fourth clause: **the object's centre at the run's first
sample must be OUTSIDE the container's AABB**. It is measured on the same
series, the same body and the same clock as the "ends inside" clause it is the
mirror of -- the first sample of exactly the series whose last sample T2.1
already grades -- so a column that can answer one can answer the other by
construction, and no new channel is required (the t=0 inventory carries the
same fact independently). The clause reports its witness absent for the same
reason the containment clause does: with no usable container box, "outside it"
is not a measurement. What it deliberately does **not** assert is *"on a
surface"*: that would grade where the agent chose to put the object down before
starting, which is scene authorship rather than transfer. Negative fixture:
``started_in_the_container``.

**3. "Above every static support surface" cannot mean every surface in the
world.** Read literally, an object resting in a bin on the floor is below the
top of the table it came from, so the clause could never hold at the end of a
successful run. The core reads it as: at each sample, the clearance is
measured against every static, bounded surface whose **horizontal footprint
overlaps the object's**, and the smallest of those clearances must be at least
0.05 m. A surface the object is nowhere near cannot be supporting it. Samples
with nothing beneath the object at all are counted and cannot contribute to
the sustained window -- "it was over the void" is not evidence of a lift.

**4. T2.3's ten seconds is graded on windows of exactly ten seconds.** The
tier asks for "a continuous >= 10.0 s" in which the deviation is bounded, and
the deviation is measured about the window's own mean -- so the bound depends
on the window, and "the longest window that satisfies it" is not a
well-defined maximum. The core resolves this by testing every window of
**exactly** 10.0 s: the tier's question is whether one exists, and that is
decidable, cheap and stable. The longest run of consecutive satisfying windows
is reported as a measurement, never as a threshold.

**4b. A ten-second window only counts if something was holding the object in
it.** *(Repair decided 2026-08-02, before the freeze; the defect it fixes was
demonstrated first --* ``adapters/mujoco/BRINGUP_T2.md`` *§5.)* Bounded
deviation **in the end-effector's frame** is satisfied by two entirely
different situations: an object held rigidly, and an object and a carrier that
are each independently motionless. On the T2 achievability run's own final
state -- block at rest on the bin floor, jaws open, arm parked 0.6 m away,
nothing in contact -- a 12 s window scored **RMS 4.0e-08 m, max excursion
3.8e-15 m** and satisfied the clause outright. Nothing was held. So the core
now requires each candidate window to contain at least one sample at which the
object was **being carried**, meaning either

* **clear of everything beneath it by >= LIFT_CLEARANCE_M** -- gravity is
  acting (T2.4 asserts it), so an object off the surface is being held up by
  *something*; or
* **moving at >= AT_REST_SPEED_MPS** -- a moving object that keeps a fixed
  offset to the carrier is moving *with* it, which is what a grip does.

No threshold moved: both numbers are the tier's own (T2.2's clearance and
T2.1's definition of "at rest"), and the 10.0 s / 0.02 m / 0.04 m bounds are
untouched. The disjunction is deliberate and the alternative was measured: the
cheapest recorded repair -- *"require the window to overlap T2.2's clearance
window"* -- is the **airborne** half alone, and on its own it makes T2.3 red
on any run that was never lifted, which (i) turns the dragged
``never_lifted`` fixture into a two-assertion red, (ii) makes T2.2 logically
redundant (a 10 s continuous clearance implies T2.2's 3 s one), and (iii)
grades a *drag* -- an object genuinely gripped for the whole run -- as a
failure to grip. That is a tier change smuggled in as a bug fix. The moving
half keeps T2.2 and T2.3 orthogonal, which is how the tier states them, while
the airborne half is what catches a static hold in mid-air (the most obvious
correct answer to *"hold it for ten seconds"*, which the moving half alone
would fail). Both are needed; neither is sufficient.

**What this repair still cannot catch -- a STATED LIMIT, ratified 2026-08-02,
not an open question:** the clearance it reads is measured against **static**
surfaces only (reading 3), so an object resting motionless on a *dynamic* body
taller than ``LIFT_CLEARANCE_M`` reads as airborne -- the same hole T2.2 has.
It is **not fixed, deliberately**: closing it needs the world AABB of every
dynamic body at every sample, which no column in the roster supplies, and the
alternative is a clause whose witness is permanently absent -- which is the
vacuity this tier spent its pre-freeze audit eliminating. What contains it is
T2.1: on the container this task ships, a crate tall enough to clear the bin's
own rim by ``LIFT_CLEARANCE_M`` puts the object's centre above the bin's box,
so the full cell does not pass on this hole. The limit is written into the
plan's T2 text and into the task's ``meta.json``
(``T2.2_STATED_LIMIT_resting_on_something_tall``) rather than papered over
here.

**5. T2's teleport guard: derived here, then RATIFIED INTO THE TIER TEXT.**
The tier's own gloss on T2.4 is *"a kinematic prop being flown to the bin
fails here"* -- and a body that jumps two metres between physics steps is
being flown as surely as a massless one is. The mass / dynamic / gravity
clauses catch the kinematic prop; they do not catch a scripted position
write on a body that does carry mass. So T2.4 carries a fourth clause, the
same 0.5 m inter-sample bound T1.3 states, with the same rate witness.
It was written here first as a **derived** clause the plan's table did not
state, flagged rather than smuggled so a reviewer could attack it -- and on
**2026-08-02, before the freeze, it was blessed into the tier text**:
``capability-ladder-plan.md`` §2 T2's own T2.4 row now states the bound and
cites T1.3's constant. It is no longer this module going beyond the plan; the
two say the same thing, and ``test_t2_core`` checks that they still do.

Body selection
--------------

T2 grades three bodies and the task names all three (object, end effector,
container). The names are **task data**, exactly as T1's waypoint is: an
adapter that could choose which body counts as "the object" could choose the
body that happens to be in the container. The core prefers the body carrying
the declared name and refuses -- loudly, with the sentence in the detail --
rather than picking the luckiest of several.
"""

from __future__ import annotations

import numpy as np

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier,
                                        Verdict)
from ladder.graders.ladder_evidence import run_is_real

TASK = "T2_transfer"

# --- the tier's thresholds, quoted so a drift is visible in a diff ----------
AT_REST_SPEED_MPS = 0.02     # T2.1  averaged over the trailing window
AT_REST_WINDOW_S = 1.0       # T2.1
LIFT_CLEARANCE_M = 0.05      # T2.2  above every static surface beneath it
LIFT_DWELL_S = 3.0           # T2.2  continuous
HOLD_S = 10.0                # T2.3  continuous, in the end-effector's frame
HOLD_RMS_M = 0.02            # T2.3
HOLD_MAX_M = 0.04            # T2.3
MIN_MASS_KG = 0.0            # T2.4  mass > 0, strictly
MIN_GRAVITY_MPS2 = 0.0       # T2.4  gravity acting, strictly

# Witness bounds the tier's wording implies rather than states, plus T2.4's
# jump bound -- which WAS this module's derived clause and is now the tier's
# own (reading 5; ratified into §2 T2's table 2026-08-02). Each decides
# whether a clause is a measurement or a formality, so each is named rather
# than assumed.
MAX_SAMPLE_DT_S = 0.05       # coarser than this and a jump bound sees nothing
MAX_CLOCK_SKEW_S = 0.05      # two series further apart are not one clock
MAX_STEP_JUMP_M = 0.5        # T2.4's jump bound; the same number T1.3 states
MIN_CONTAINER_SPAN_M = 0.01  # a degenerate box is not a container

_ALL = [("T2.1", "the object started outside the container and ended inside "
                 "it, at rest"),
        ("T2.2", "it was carried clear of the surface, not dragged"),
        ("T2.3", "the grip held it for ten seconds"),
        ("T2.4", "the object is a real body under gravity"),
        ("T2.5", "the run is real")]

_BASIS = {"T2.1": CORE_PHYSICAL, "T2.2": CORE_PHYSICAL,
          "T2.3": CORE_PHYSICAL, "T2.4": MIXED, "T2.5": MIXED}


# --- selection ---------------------------------------------------------------


def select_role_body(ev, name):
    """``(body, why)`` for the inventory entry the task names ``name``.

    The inventory answers the *facts* about a body (its size at t=0, whether
    it is dynamic); the pose series answers where it went. Both must be about
    the same body, which is why the name comes from the task and the lookup
    lives here rather than in an adapter.
    """
    if not name:
        return None, "the task named no body for this role"
    for inv in (ev.bundle.t0, ev.bundle.roster):
        for b in (getattr(inv, "bodies", None) or []):
            if b.name == name:
                return b, ("the body carrying the name the task declares "
                           "(%r), from %s" % (name, inv.source or "an "
                                              "unattributed inventory"))
    return None, ("no body named %r is in the run's inventory, so the facts "
                  "the tier asks for about it cannot be read" % name)


def series_naming_problems(ev):
    """Series that claim to be about a body other than the one named.

    An adapter that hands the core the wrong body's motion produces a verdict
    about the wrong thing, and every number in it looks fine. An empty
    ``body`` is *"the adapter did not say"* and is reported as a note; a
    non-empty mismatch is refused.
    """
    out = []
    for label, series, want in (
            ("object", ev.object_pose, ev.roles.object_name),
            ("end effector", ev.end_effector, ev.roles.end_effector_name)):
        got = (series.body or "").strip()
        if got and want and got != want:
            out.append("the %s series is about %r, but the task names %r"
                       % (label, got, want))
    want = (ev.roles.container_name or "").strip()
    got = (ev.container.body or "").strip()
    if got and want and got != want:
        out.append("the container geometry is about %r, but the task names %r"
                   % (got, want))
    return out


# --- measurement helpers (plain numbers in, plain numbers out) ---------------


def mean_speed_over(t, xyz, window_s):
    """``(mean speed, intervals, seconds covered)`` over the trailing window.

    The tier says *"speed averaged over the last 1.0 s"*; this is that
    average, taken over the inter-sample speeds inside the window. Returns
    ``(None, 0, 0.0)`` when the run does not have two samples in it.
    """
    t = np.asarray(t, dtype=float)
    xyz = np.asarray(xyz, dtype=float)
    if len(t) < 2:
        return None, 0, 0.0
    lo = float(t[-1]) - float(window_s)
    # The last sample at or before the window's start, so the window COVERS
    # the requested seconds rather than falling a fraction of a sample short
    # of them -- which would make the clause's own witness permanently absent.
    k = int(np.searchsorted(t, lo, side="right")) - 1
    k = min(max(k, 0), len(t) - 2)
    dt = np.diff(t[k:])
    d = np.linalg.norm(np.diff(xyz[k:], axis=0), axis=1)
    good = dt > 0.0
    if not good.any():
        return None, 0, 0.0
    speeds = d[good] / dt[good]
    return (float(speeds.mean()), int(good.sum()),
            float(t[-1] - t[k]))


def sample_speed(t, xyz):
    """Per-sample speed, m/s, one value per sample (the first repeats).

    Reading 4b needs to ask "was the object moving *here*", not "on average
    over the run", so this is the inter-sample speed carried forward to the
    sample it ends at. It is deliberately NOT the trailing average
    :func:`mean_speed_over` computes for T2.1: that one answers "has it come
    to rest by the end", this one answers "was anything happening at t".
    """
    t = np.asarray(t, dtype=float)
    xyz = np.asarray(xyz, dtype=float)
    n = len(t)
    if n < 2 or len(xyz) != n:
        return np.zeros(max(n, 0), dtype=float)
    dt = np.diff(t)
    d = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    step = np.where(dt > 0.0, d / np.where(dt > 0.0, dt, 1.0), 0.0)
    return np.concatenate([step[:1], step])


def carried_samples(airborne, speed):
    """Samples at which SOMETHING was doing work on the object.

    Reading 4b: off the surface (held up against gravity) **or** moving (moved
    by something). The negation -- at rest, on a surface -- is a scene in which
    nothing is happening, and a constant offset to a carrier measured across
    such a window is a statement about two static bodies rather than a grip.
    """
    airborne = np.asarray(airborne, dtype=bool)
    speed = np.asarray(speed, dtype=float)
    if len(speed) != len(airborne):
        return airborne
    return airborne | (speed >= AT_REST_SPEED_MPS)


def longest_true_window(t, mask):
    """``(seconds, i0, i1)`` of the longest continuous run of ``True``."""
    t = np.asarray(t, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    best, best_i0, best_i1 = 0.0, None, None
    i = 0
    n = len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        secs = float(t[j] - t[i])
        # "best_i0 is None or" matters: a run of ONE sample is 0.0 s long, and
        # a bare `secs > best` would report it as no run at all.
        if best_i0 is None or secs > best:
            best, best_i0, best_i1 = secs, i, j
        i = j + 1
    return best, best_i0, best_i1


def lift_clearance(xy, lowest_z, surfaces, half_xy=None):
    """``(clearance, defined, bodies)`` of the object over what is beneath it.

    Reading 3 in the module docstring: a surface can only be supporting the
    object if the object is horizontally over it, so the clearance at each
    sample is the smallest gap to any static bounded surface whose footprint
    overlaps the object's. ``defined`` is False where nothing at all is
    beneath the object -- those samples are not evidence of a lift.
    """
    xy = np.asarray(xy, dtype=float)
    lowest_z = np.asarray(lowest_z, dtype=float)
    n = len(lowest_z)
    hx, hy = (float(half_xy[0]), float(half_xy[1])) if half_xy else (0.0, 0.0)
    best = np.full(n, np.inf)
    defined = np.zeros(n, dtype=bool)
    touched = []
    for s in surfaces:
        if not s.usable:
            continue
        over = ((xy[:, 0] + hx >= float(s.aabb_min[0]))
                & (xy[:, 0] - hx <= float(s.aabb_max[0]))
                & (xy[:, 1] + hy >= float(s.aabb_min[1]))
                & (xy[:, 1] - hy <= float(s.aabb_max[1])))
        if not over.any():
            continue
        touched.append(s.body)
        gap = lowest_z - float(s.top_z)
        best = np.where(over, np.minimum(best, gap), best)
        defined = defined | over
    best = np.where(defined, best, np.nan)
    return best, defined, touched


def frame_relative(obj_xyz, holder_xyz, holder_rot):
    """The object's position in the holder's own frame, metres.

    ``r(t) = R(t)^T (p_obj(t) - p_hold(t))`` with ``R`` world-from-body.
    Rotation is the whole point: a rigidly held object whose wrist turns
    traces a circle in the world frame and a single point in the holder's, and
    a check that skipped the rotation would read a perfect grasp as a slip of
    twice the grip offset.
    """
    d = np.asarray(obj_xyz, dtype=float) - np.asarray(holder_xyz, dtype=float)
    rot = np.asarray(holder_rot, dtype=float)
    return np.einsum("nji,nj->ni", rot, d)


def hold_windows(t, r, *, rms_tol, max_tol, min_s, carried=None):
    """Windows of exactly ``min_s`` in which ``r`` stays within both bounds.

    Readings 4 and 4b in the module docstring. ``carried`` is the per-sample
    mask :func:`carried_samples` produces; a window with no carried sample in
    it is **not** a hold no matter how still the geometry was, because a scene
    at rest satisfies the bounds trivially. ``carried=None`` grades the bounds
    alone and is what the *world-frame* comparison and the unit tests of the
    window arithmetic use -- never the assertion.

    Returns whether a hold exists, the longest run of consecutive satisfying
    windows (a measurement), the tightest window seen either way -- so a
    failing row says how close it got -- and, separately,
    ``holds_ignoring_carry``: whether a window met the bounds with nothing
    holding the object. That last one is the vacuous green this clause used to
    report as a pass, kept visible in the row rather than deleted.
    """
    t = np.asarray(t, dtype=float)
    r = np.asarray(r, dtype=float)
    out = {"holds": False, "longest_s": 0.0, "rms_m": None, "max_m": None,
           "windows_tested": 0, "start_s": None, "end_s": None,
           "holds_ignoring_carry": False, "carried_samples": None}
    n = len(t)
    if n < 2 or r.ndim != 2 or r.shape[0] != n:
        return out
    ends = np.searchsorted(t, t + float(min_s), side="left")
    starts = np.nonzero(ends < n)[0]
    if not len(starts):
        return out
    e = ends[starts]
    m = (e - starts + 1).astype(float)
    s1 = np.vstack([np.zeros(3), np.cumsum(r, axis=0)])
    s2 = np.concatenate([[0.0], np.cumsum((r * r).sum(axis=1))])
    mean = (s1[e + 1] - s1[starts]) / m[:, None]
    var = (s2[e + 1] - s2[starts]) / m - (mean * mean).sum(axis=1)
    rms = np.sqrt(np.maximum(var, 0.0))
    out["windows_tested"] = int(len(starts))

    if carried is None:
        n_carried = m.astype(int)           # not gated: every sample counts
    else:
        cs = np.concatenate([[0], np.cumsum(
            np.asarray(carried, dtype=bool).astype(np.int64))])
        n_carried = (cs[e + 1] - cs[starts]).astype(int)

    bounded = np.zeros(len(starts), dtype=bool)
    mx = np.full(len(starts), np.nan)
    for k in np.nonzero(rms <= float(rms_tol))[0]:
        i, j = int(starts[k]), int(e[k])
        mx[k] = float(np.linalg.norm(r[i:j + 1] - mean[k], axis=1).max())
        bounded[k] = mx[k] <= float(max_tol)
    ok = bounded & (n_carried > 0)
    out["holds_ignoring_carry"] = bool(bounded.any())

    if ok.any():
        secs, a, b = longest_true_window(np.arange(len(starts),
                                                   dtype=float), ok)
        a = int(a)
        b = int(b)
        out["holds"] = True
        out["longest_s"] = float(t[int(e[b])] - t[int(starts[a])])
        out["rms_m"] = float(rms[a])
        out["max_m"] = float(mx[a])
        out["start_s"] = float(t[int(starts[a])])
        out["end_s"] = float(t[int(e[b])])
        out["carried_samples"] = int(n_carried[a])
        return out

    # Nothing held. Report the window that came closest to holding: the
    # tightest window that WAS a carry if any candidate window was, and
    # otherwise the tightest one there was -- so a run whose only quiet
    # stretch was a scene at rest says exactly that.
    pool = np.nonzero(n_carried > 0)[0]
    k = int(pool[np.argmin(rms[pool])]) if len(pool) else int(np.argmin(rms))
    i, j = int(starts[k]), int(e[k])
    out["rms_m"] = float(rms[k])
    out["max_m"] = float(np.linalg.norm(r[i:j + 1] - mean[k], axis=1).max())
    out["start_s"] = float(t[i])
    out["end_s"] = float(t[j])
    out["carried_samples"] = int(n_carried[k])
    return out


# --- the grade ---------------------------------------------------------------


def grade(ev, *, self_verified=False):
    """Grade one T2 run from neutral evidence. Never raises."""
    b = ev.bundle
    v = Verdict(TASK, self_verified=self_verified)
    for note in list(getattr(b, "notes", []) or []) + list(ev.notes or []):
        v.note(note)
    # Recorded in EVERY cell, including the ones that fail before any physics
    # is read: no prose sentence about a T2 cell may omit the mechanism, so no
    # T2 verdict may omit it either.
    v.measurements["hold_mechanism"] = ev.grip.recorded_mechanism
    v.measurements["hold_mechanism_detail"] = ev.grip.as_dict()
    v.measurements["roles"] = ev.roles.as_dict()
    v.measurements["container"] = ev.container.as_dict()
    v.measurements["provenance"] = ev.provenance()

    # ---- the deliverable -------------------------------------------------
    if b.artifact is None:
        v.progress = NO_ARTIFACT
        return _all_red(v, "no deliverable was produced")
    v.artifacts["deliverable"] = str(b.artifact)

    if not ev.roles.usable:
        v.outcome = INVALID
        v.note("the task did not name the %s, so there is nothing to measure "
               "against. The names are task data and a missing one is a "
               "broken instrument, never a failed run."
               % " and the ".join(ev.roles.missing))
        return _all_red(v, "the task named no %s"
                        % " and no ".join(ev.roles.missing), finish=False)

    obj = ev.object_pose
    if not obj.usable:
        v.progress = ARTIFACT_INVALID if b.live_load_ok is False \
            else ARTIFACT_RUNS
        return _all_red(v, obj.error or "no pose series for the object")

    problems = series_naming_problems(ev)
    if problems:
        v.progress = ARTIFACT_RUNS
        return _all_red(v, "; ".join(problems))

    v.progress = ARTIFACT_RUNS
    v.measurements.update(b.adapter_measurements.get("motion") or {})

    body, why = select_role_body(ev, ev.roles.object_name)
    v.measurements["graded_object"] = {
        "selection": why, "name": (body.name if body else None),
        "id": (body.body_id if body else None),
        "half extent at t=0 (m)": ev.object_half_extent}

    t = np.asarray(obj.t, dtype=float)
    xyz = np.asarray(obj.xyz, dtype=float)
    half = ev.object_half_extent
    hz = (float(half[2]) if half else None)
    lowest = xyz[:, 2] - (hz or 0.0)

    # The clearance is computed ONCE and read by two assertions: T2.2 grades
    # the lift with it, and T2.3 uses it (reading 4b) to tell a grip from a
    # scene at rest. Two copies of this walk would be two readings of "what is
    # under the object", and they would drift.
    surfaces = [s for s in ev.support_surfaces if s.usable]
    clear_m, defined, touched = lift_clearance(
        xyz[:, :2], lowest, surfaces, half_xy=(half[:2] if half else None))
    airborne = np.where(defined, clear_m, -np.inf) >= LIFT_CLEARANCE_M

    _t2_1(v, ev, t, xyz, lowest, hz)
    _t2_2(v, ev, t, surfaces, clear_m, defined, touched, airborne)
    _t2_3(v, ev, t, xyz, airborne, defined)
    _t2_4(v, ev, obj)
    attributed = run_is_real(v, "T2.5", b)

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
                  "a deliverable, the three bodies the task names, and a "
                  "pose series for the object over the run", False,
                  detail=why)])
    return v.finish() if finish else v


# --- T2.1 --------------------------------------------------------------------


def _t2_1(v, ev, t, xyz, lowest, hz):
    c = ev.container
    start = xyz[0]
    final = xyz[-1]
    # Reading 2b: the mirror of "ends inside", measured on the first sample of
    # the same series. A run that lifts the object OUT of the container and
    # puts it back is not a transfer, and until this clause existed nothing in
    # the tier said so.
    started_in = c.contains(start)
    started_outside = (None if started_in is None else bool(not started_in))
    inside = c.contains(final)
    rim = c.rim_height
    lowest_final = float(lowest[-1])
    below_rim = (None if rim is None else bool(lowest_final < float(rim)))
    speed, intervals, covered = mean_speed_over(t, xyz, AT_REST_WINDOW_S)
    at_rest = (None if speed is None
               else bool(speed < AT_REST_SPEED_MPS))
    ok = bool(started_outside is True and inside is True
              and below_rim is True and at_rest is True)

    span = c.span
    box_ok = bool(c.usable and span is not None
                  and min(span) >= MIN_CONTAINER_SPAN_M)
    v.add("T2.1", ok,
          "the object started outside the container and ended inside it, "
          "at rest",
          measured={"the object's centre at the start (m)":
                        [round(float(x), 4) for x in start],
                    "started outside the container's bounding box":
                        started_outside,
                    "the object's centre at the end (m)":
                        [round(float(x), 4) for x in final],
                    "inside the container's bounding box": inside,
                    "its lowest point at the end (m)": round(lowest_final, 4),
                    "the container rim (m)":
                        (None if rim is None else round(float(rim), 4)),
                    "how the rim was decided": c.rim_source,
                    "mean speed over the last %.1f s (m/s)"
                    % AT_REST_WINDOW_S:
                        (None if speed is None else round(speed, 5)),
                    "seconds of samples in that window": round(covered, 3)},
          threshold={"centre at the start": "outside the container's "
                                            "bounding box",
                     "centre at the end": "inside the container's bounding "
                                          "box",
                     "lowest point": "below the rim",
                     "speed (m/s)": "< %.2f" % AT_REST_SPEED_MPS},
          basis=CORE_PHYSICAL,
          detail=("" if box_ok else
                  "the container's bounding box is degenerate or absent "
                  "(span %s m), so containment is not a measurement"
                  % (span,)),
          falsifiers=[
              Falsifier("started outside the container",
                        "an object that was ALREADY in the container when the "
                        "run began -- lifted out, held over it and put back, "
                        "which every other clause in this tier reads as a "
                        "clean transfer",
                        "the object's centre at the run's first sample AND a "
                        "container with a non-degenerate world-space bounding "
                        "box, at least %.2f m on every axis, so 'outside it' "
                        "is a measurement" % MIN_CONTAINER_SPAN_M,
                        box_ok,
                        detail=(c.error or c.source or "")),
              Falsifier("ended inside the container",
                        "an object left beside the container, on the way to "
                        "it, or dropped short of it",
                        "a container with a non-degenerate world-space "
                        "bounding box, at least %.2f m on every axis"
                        % MIN_CONTAINER_SPAN_M,
                        box_ok,
                        detail=(c.error or c.source or "")),
              Falsifier("under the rim, not perched on it",
                        "an object balanced on the rim or resting on the "
                        "container's lid with its centre still inside the box",
                        "an attested rim height, AND the object's own "
                        "vertical extent so a centre can be turned into a "
                        "lowest point",
                        bool(hz is not None and c.rim_attested),
                        detail=("object half height %s m; rim %s"
                                % (("unknown -- the centre was graded instead"
                                    if hz is None else "%.4f" % hz),
                                   ("attested" if c.rim_attested else
                                    "not attested -- the permissive box top "
                                    "was used")))),
              Falsifier("came to rest",
                        "an object still moving when the clock ran out -- "
                        "released above the container and caught mid-fall",
                        "at least %.1f s of samples at the end of the run, so "
                        "an average over that window exists"
                        % AT_REST_WINDOW_S,
                        bool(intervals >= 1 and covered >= AT_REST_WINDOW_S),
                        detail="%d intervals over %.3f s" % (intervals,
                                                             covered)),
          ])


# --- T2.2 --------------------------------------------------------------------


def _t2_2(v, ev, t, surfaces, clear_m, defined, touched, high):
    secs, i0, i1 = longest_true_window(t, high)
    ok = bool(secs >= LIFT_DWELL_S)
    run_s = float(t[-1] - t[0]) if len(t) else 0.0
    best = (float(np.nanmax(clear_m)) if defined.any() else None)

    v.add("T2.2", ok, "it was carried clear of the surface, not dragged",
          measured={"longest continuous window clear by >= %.2f m (s)"
                    % LIFT_CLEARANCE_M: round(secs, 3),
                    "greatest clearance reached (m)":
                        (None if best is None else round(best, 4)),
                    "surfaces beneath it at some point": touched,
                    "static bounded surfaces in the run": len(surfaces),
                    "samples with nothing beneath the object":
                        int((~defined).sum()),
                    "window started at (s)":
                        (None if i0 is None else round(float(t[i0]), 3)),
                    "window ended at (s)":
                        (None if i1 is None else round(float(t[i1]), 3))},
          threshold={"clearance (m)": ">= %.2f" % LIFT_CLEARANCE_M,
                     "continuous (s)": ">= %.1f" % LIFT_DWELL_S},
          basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("cleared the surface",
                        "an object slid, rolled or dragged the whole way -- "
                        "never more than %.2f m above what was under it"
                        % LIFT_CLEARANCE_M,
                        "at least one static surface with a world-space "
                        "bounding box, so 'above' has a datum",
                        bool(surfaces),
                        detail=("%d static bounded surfaces" % len(surfaces))),
              Falsifier("held the lift",
                        "a momentary hop rather than a carry",
                        "a run at least %.1f s long, so a %.1f s window can "
                        "exist at all" % (LIFT_DWELL_S, LIFT_DWELL_S),
                        bool(run_s >= LIFT_DWELL_S),
                        detail="run length %.3f s" % run_s),
          ])


# --- T2.3 --------------------------------------------------------------------


def _t2_3(v, ev, t, xyz, airborne, defined):
    ee = ev.end_effector
    idx, skew = (ee.align_to(t) if ee.usable else (None, None))
    clock_ok = bool(skew is not None and skew <= MAX_CLOCK_SKEW_S)
    frame_ok = bool(ee.usable and ee.has_orientation and clock_ok)

    # Reading 4b. The carry gate, and its own witness: with nothing beneath
    # the object at any sample, "off the surface" is unanswerable and only
    # "moving" is left -- which cannot certify a hold that stood still in
    # mid-air, so the clause says so instead of quietly grading less.
    speed = sample_speed(t, xyz)
    carried = carried_samples(airborne, speed)
    carry_witness = bool(np.asarray(defined, dtype=bool).any())

    res = {"holds": False, "longest_s": 0.0, "rms_m": None, "max_m": None,
           "windows_tested": 0, "start_s": None, "end_s": None,
           "holds_ignoring_carry": False, "carried_samples": None}
    offset = None
    if ee.usable and idx is not None:
        ee_xyz = np.asarray(ee.xyz, dtype=float)[idx]
        offset = hold_windows(t, xyz - ee_xyz, rms_tol=HOLD_RMS_M,
                              max_tol=HOLD_MAX_M, min_s=HOLD_S,
                              carried=carried)
        if frame_ok:
            rot = np.asarray(ee.rot, dtype=float)[idx]
            res = hold_windows(t, frame_relative(xyz, ee_xyz, rot),
                               rms_tol=HOLD_RMS_M, max_tol=HOLD_MAX_M,
                               min_s=HOLD_S, carried=carried)
    ok = bool(frame_ok and res["holds"])
    run_s = float(t[-1] - t[0]) if len(t) else 0.0
    # The exact signature of the defect this clause was repaired for: the
    # geometry stood still for ten seconds and nothing was holding anything.
    at_rest_only = bool(res["holds_ignoring_carry"] and not res["holds"])

    v.add("T2.3", ok, "the grip held it for ten seconds",
          measured={"a %.1f s window inside both bounds exists, with the "
                    "object being carried in it" % HOLD_S: res["holds"],
                    "longest run of such windows (s)":
                        round(float(res["longest_s"]), 3),
                    "deviation in that window, RMS (m)":
                        (None if res["rms_m"] is None
                         else round(res["rms_m"], 5)),
                    "deviation in that window, max (m)":
                        (None if res["max_m"] is None
                         else round(res["max_m"], 5)),
                    "samples in that window with the object off the surface "
                    "or moving": res["carried_samples"],
                    # The whole stretch the run of satisfying windows covers
                    # -- printed so a reader can see the hold sitting inside
                    # the carry T2.2 reports, instead of taking it on trust.
                    # It is the span of the RUN, not of one window: the
                    # deviations above belong to the first window in it.
                    "that run of windows spans from (s)":
                        (None if res["start_s"] is None
                         else round(float(res["start_s"]), 3)),
                    "...to (s)":
                        (None if res["end_s"] is None
                         else round(float(res["end_s"]), 3)),
                    "windows tested": res["windows_tested"],
                    "a window inside both bounds in which the object was at "
                    "rest on a surface throughout -- static geometry, NOT a "
                    "grip": at_rest_only,
                    "the same measured WITHOUT the carrier's rotation, "
                    "which is NOT the frame the tier states":
                        (None if offset is None
                         else {"holds": offset["holds"],
                               "rms (m)": (None if offset["rms_m"] is None
                                           else round(offset["rms_m"], 5))}),
                    "clock skew between the two series (s)":
                        (None if skew is None else round(float(skew), 5))},
          threshold={"window (s)": ">= %.1f" % HOLD_S,
                     "RMS (m)": "<= %.2f" % HOLD_RMS_M,
                     "max excursion (m)": "<= %.2f" % HOLD_MAX_M,
                     "frame": "the end effector's own",
                     "the window is a carry": "the object off the surface by "
                                              ">= %.2f m or moving at >= "
                                              "%.2f m/s, at >= 1 sample"
                                              % (LIFT_CLEARANCE_M,
                                                 AT_REST_SPEED_MPS)},
          basis=CORE_PHYSICAL,
          detail=("the carrier's orientation did not reach the grader, so the "
                  "object's position in its frame was never measured"
                  if not frame_ok else
                  ("the only %.1f s windows inside the bounds are ones in "
                   "which the object sat motionless on a surface -- a scene "
                   "at rest, not a grip" % HOLD_S) if at_rest_only else ""),
          falsifiers=[
              Falsifier("held for ten seconds",
                        "a grasp that slipped, was released early, or never "
                        "closed -- no %.1f s window inside the bounds"
                        % HOLD_S,
                        "a run at least %.1f s long, so a %.1f s window can "
                        "exist at all" % (HOLD_S, HOLD_S),
                        bool(run_s >= HOLD_S),
                        detail="run length %.3f s" % run_s),
              Falsifier("measured in the carrier's own frame",
                        "an object sliding in the jaws while the arm rotates "
                        "-- which a world-frame offset reads as a hold, and a "
                        "rigid hold under rotation, which it reads as a slip",
                        "the carrier's orientation at every sample, on the "
                        "same clock as the object's (within %.3f s)"
                        % MAX_CLOCK_SKEW_S,
                        frame_ok,
                        detail=("orientation %s; clock skew %s"
                                % ("present" if (ee.usable
                                                 and ee.has_orientation)
                                   else "absent",
                                   "unknown" if skew is None
                                   else "%.5f s" % skew))),
              Falsifier("something was holding it",
                        "a scene at rest -- the object motionless on a "
                        "surface with the arm parked elsewhere, in which a "
                        "constant offset to the carrier is two static bodies "
                        "and not a grip",
                        "a static bounded surface beneath the object at some "
                        "sample, so 'clear of what is under it' is "
                        "answerable; without one, only 'it moved' is left "
                        "and a hold that stood still in the air cannot be "
                        "told from a scene at rest",
                        carry_witness,
                        detail=("%d of %d samples were off the surface or "
                                "moving; %d samples had nothing beneath the "
                                "object"
                                % (int(np.asarray(carried).sum()), len(t),
                                   int((~np.asarray(defined,
                                                    dtype=bool)).sum())))),
          ])


# --- T2.4 --------------------------------------------------------------------


def _t2_4(v, ev, obj):
    """mass / dynamic / gravity, plus the jump bound reading 5 describes.

    The jump bound is **the tier's**, not this module's: it was written here
    first as a derived clause and ratified into the plan's own T2.4 row on
    2026-08-02, before the freeze. It reuses T1.3's constant and T1.3's rate
    witness rather than declaring numbers of its own.
    """
    phys = ev.object_physics
    world = ev.world
    mass = phys.mass_kg
    mass_ok = bool(mass is not None and float(mass) > MIN_MASS_KG)
    dynamic_ok = bool(phys.dynamic is True)
    gravity = world.gravity_mps2
    gravity_ok = bool(gravity is not None
                      and float(gravity) > MIN_GRAVITY_MPS2)
    jump = obj.max_step_jump()
    jump_ok = bool(jump <= MAX_STEP_JUMP_M)
    dt = obj.dt_s
    rate_witness = bool(dt is not None and float(dt) <= MAX_SAMPLE_DT_S
                        and obj.n_samples >= 2)
    ok = bool(mass_ok and dynamic_ok and gravity_ok and jump_ok)

    v.add("T2.4", ok, "the object is a real body under gravity",
          measured={"mass (kg)": mass,
                    "a mass model is attached": phys.dynamic,
                    "gravity (m/s^2)": gravity,
                    "largest single inter-sample displacement (m)":
                        round(float(jump), 4),
                    "sample interval (s)":
                        (None if dt is None else round(float(dt), 5))},
          threshold={"mass (kg)": "> %.1f" % MIN_MASS_KG,
                     "a mass model is attached": True,
                     "gravity (m/s^2)": "> %.1f" % MIN_GRAVITY_MPS2,
                     "inter-sample displacement (m)":
                         "<= %.2f" % MAX_STEP_JUMP_M},
          basis=MIXED,
          attested=([x for x in [
              ("mass: %s" % phys.source) if phys.source else None,
              ("gravity: %s" % world.source) if world.source else None,
          ] if x]),
          falsifiers=[
              Falsifier("has mass",
                        "a massless body -- a fixture with no inertia that "
                        "any constraint can carry for free",
                        "the object's mass in kilograms, which the neutral "
                        "inventory has no field for and a ladder-side "
                        "sampler must supply",
                        bool(mass is not None),
                        detail=(phys.error or phys.source or "")),
              Falsifier("is dynamic",
                        "a body the scene declares immovable and then moves "
                        "anyway",
                        "the adapter answers 'a mass model is attached' "
                        "either way",
                        bool(phys.dynamic is not None)),
              Falsifier("gravity is acting",
                        "a world with gravity switched off, in which nothing "
                        "has to be held at all",
                        "the world's own gravity, attested with a citation",
                        bool(gravity is not None),
                        detail=(world.error or world.source or "")),
              Falsifier("moved like a body",
                        "one inter-sample displacement over %.2f m -- a "
                        "position written onto the object rather than a "
                        "force applied to it" % MAX_STEP_JUMP_M,
                        "a pose series sampled at most every %.3f s, so a "
                        "jump is distinguishable from ordinary travel"
                        % MAX_SAMPLE_DT_S,
                        rate_witness,
                        detail=("sample interval %s s"
                                % ("unknown" if dt is None
                                   else "%.5f" % float(dt)))),
          ])


__all__ = ["TASK", "grade", "select_role_body", "series_naming_problems",
           "mean_speed_over", "sample_speed", "carried_samples",
           "longest_true_window", "lift_clearance",
           "frame_relative", "hold_windows",
           "AT_REST_SPEED_MPS", "AT_REST_WINDOW_S", "LIFT_CLEARANCE_M",
           "LIFT_DWELL_S", "HOLD_S", "HOLD_RMS_M", "HOLD_MAX_M",
           "MIN_MASS_KG", "MIN_GRAVITY_MPS2", "MAX_SAMPLE_DT_S",
           "MAX_CLOCK_SKEW_S", "MAX_STEP_JUMP_M", "MIN_CONTAINER_SPAN_M"]
