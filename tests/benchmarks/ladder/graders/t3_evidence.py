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

"""The channels **T3** needs, which nothing upstream of it can answer.

``ladder_evidence.py`` is the ladder's shared channel module and it carries
T1's and T2's published shapes; this file is additive and lives beside it for
the same reason ``ladder_evidence`` lives beside the frozen
``agentbench/graders/evidence.py`` -- **a module that carries rows is not
edited to make room for a rung that has not run yet.** A T3 core consumes an
:class:`~agentbench.graders.evidence.EvidenceBundle` *plus* a
:class:`T3Evidence` wrapper, exactly as T1 and T2 do, and the shared pieces
(:func:`ladder_evidence.run_is_real`, :class:`~ladder_evidence.PoseSeries`,
:class:`~ladder_evidence.BodyPhysics`, :class:`~ladder_evidence.WorldPhysics`)
are **imported** rather than re-declared, so a threshold or a clause cannot
exist twice.

--------------------------------------------------------------------------
Gap 1: orientation, and a standing height that belongs to *this* robot
--------------------------------------------------------------------------

T3.2 is *"base ``z`` >= 0.60 x its settled standing ``z`` at every recorded
sample, and |roll|, |pitch| <= 0.8 rad at every sample. The standing ``z`` is
read per robot from the ``t0`` inventory, not fixed as a constant, so the
criterion is robot-neutral"* (``capability-ladder-plan.md`` §2 T3).

Two things follow. **Orientation is mandatory** -- the frozen ``Trajectory``
carries position and velocity and no rotation, so a column with no ladder-side
sampler cannot answer half of T3.2 at all, and the honest reading is a red
assertion whose blocker is *ours*. T3 therefore takes the base as a
:class:`~ladder_evidence.PoseSeries` (which already carries a world-from-body
rotation matrix per sample) rather than as a trajectory row. And **the standing
height is a per-robot datum with a citation**: :class:`StandingHeight` carries
it and says where it came from, because ``0.60 x standing z`` with the
standing z guessed is a threshold nobody can check.

--------------------------------------------------------------------------
Gap 2: "it walked" needs the times the contact query said **no**
--------------------------------------------------------------------------

T3.3 wants *">= 8 distinct make-and-break transitions of (robot body, ground)
contact"*. A list of observed contacts cannot answer that: a contact record
says a touch happened at *t*, and nothing in a list of touches says when a
foot was **off** the ground. So :class:`GaitContactObservation` carries the
times the query actually ran (``sample_times``) beside the contacts, and a
channel that omits them makes the transition clause report itself vacuous
rather than counting zero and calling the robot a slider.

**And the transitions are counted PER CONTACTING BODY, which is a reading, not
a transcription.** A walking quadruped always has at least one foot down, so
the *aggregate* "is the robot touching the ground" signal of a perfect walk
never changes state and would count **zero** transitions. Counted per body, a
trot at 1.6 Hz over 40 s produces hundreds. The reading is declared in the
task's ``meta.json`` and printed in the row.

--------------------------------------------------------------------------
Gap 3: the applied-wrench channel -- the largest new build in the programme
--------------------------------------------------------------------------

T3.4 is *"no contact with any body other than the ground, **and** total
non-gravitational, non-contact force applied to the robot's base is
<= 0.02 x m.g peak and <= 2 N.m of applied torque. Where a simulator cannot
attest the applied-force total, the cell records ``support_attestation:
unverified`` and is **excluded from comparison with any cell that has it**"*.

``capability-ladder-plan.md`` §9 Q2 records the risk plainly: this channel may
not exist on any column but ours, and if most columns can only answer
``unverified`` the assertion is nearly unenforceable cross-column. Nothing here
can fix that -- what it can do is make the two states impossible to confuse.
:class:`AppliedSupport` is **unverified unless a column positively attests the
wrench AND a weight datum exists to compare it against**, and the core's
consequence for unverified is the plan's: recorded, excluded from comparison,
and never silently credited.

Note what is *not* here: no ``supported``/``unsupported`` boolean. §2 T4 states
the reasoning for its own tier and it applies to the measurement either way --
*"'harnessed / unharnessed' hides the difference between a fingertip and a
crane"*. The channel carries newtons.

--------------------------------------------------------------------------
Gap 4: the world ended the run, and a grader that cannot see that lies
--------------------------------------------------------------------------

``docs/developer/g1-endurance-2026-08-01.md`` §8, from six measured runs:
*"The world, not the policy, ended every single run ... all six runs here would
have been misread as 'the walk degrades to 0.037 m/s after ~110 s' by a grader
that did not know where the walls were."* That is not a hypothetical failure
mode, it is a measured one, on this tree's own flagship, and it would have
turned a clean walk into a fabricated gait-degradation finding.

:class:`ArenaBounds` is the fix: the horizontal extent of the free walking
region, with a citation. The core uses it for two things -- to say whether the
run-up the task requires was actually present, and to distinguish *"it stopped
because it ran out of world"* from *"it stopped"*. A column that cannot supply
it gets the conservative reading: **no arena stop is ever declared**, the
scored window is the whole run, and the row says the boundary was unknown.

--------------------------------------------------------------------------
Gap 5: a controller that never loaded exits 0
--------------------------------------------------------------------------

T3.5 adds one clause to T1.5: *"the policy/controller is asserted to have
actually loaded (an exit code is not evidence -- see below)"*, and the "below"
is a trap that has already voided a quadruped result in this tree: with the
model runtime missing from the interpreter that spawns controllers, the deploy
runs a zero-residual baseline **and exits 0**. :class:`ControllerLoad` carries
the positive attestation and the evidence line it was read from. Absent, the
clause is red *and* vacuous and the blocker is ours.

``declared_method`` rides along in the same dataclass and is the plan's
``method`` field (§3.2): **recorded in every cell, graded in none.** A walk
reached by a learned policy and a walk reached by a model-based gait are both
``achieved``; how it was reached is a fact readers want and is never a pass
condition. :func:`ladder.graders.test_t3_core` asserts that mechanically.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ladder.graders.ladder_evidence import (  # noqa: E402,F401 (re-exported)
    UNITS, BodyPhysics, EMPTY_BODY_PHYSICS, EMPTY_POSE_SERIES,
    EMPTY_WORLD_PHYSICS, PoseSeries, WorldPhysics, body_physics_from_bundle,
    max_step_jump, pose_series_from_bundle, run_is_real)

# ``capability-ladder-plan.md`` §3.2: recorded in every cell, never a pass
# condition. The first three are the plan's own examples; the fourth is the
# honest answer from a column that did not say.
METHODS = ("learned_policy", "model_based", "scripted", "unknown")

# §3.2 again: the two values the support field may take, and the only two.
ATTESTED = "attested"
UNVERIFIED = "unverified"

# How a run ended. **The plan's own vocabulary, adopted verbatim rather than
# forked.** ``capability-ladder-plan.md`` §2 T4 and §3.2 state these five
# values, and the field they belong to, for T4 -- the tier that shares this
# tier's arena problem word for word. Inventing a parallel set of names for T3
# would give one recorder two vocabularies for one measurement, and a reader
# comparing a T3 cell with a T4 cell would have to translate. Neither this nor
# ``distance_to_termination_m`` is ever a pass condition.
TERMINATION_FELL = "fell"
TERMINATION_ARENA = "arena_geometry"          # it ran out of world
TERMINATION_TIME_LIMIT = "time_limit"         # the clock ran out while it went
TERMINATION_CONTROLLER = "controller_stopped"  # it stopped, and not at an edge
TERMINATION_UNKNOWN = "unknown"
TERMINATIONS = (TERMINATION_FELL, TERMINATION_ARENA, TERMINATION_TIME_LIMIT,
                TERMINATION_CONTROLLER, TERMINATION_UNKNOWN)


# --- geometry helpers two clauses share --------------------------------------


def roll_pitch(rot):
    """``(roll, pitch)`` per sample, radians, from world-from-body rotations.

    **The reading, declared rather than assumed.** ``rot`` is decomposed as
    ``R = Rz(yaw) Ry(pitch) Rx(roll)`` -- the intrinsic z-y-x convention -- and
    the yaw is discarded. That is the whole point: a robot that walks in a
    circle has turned, not fallen, and a tilt measure that folded yaw in would
    fail a turning walk for being a turning walk.

    Returns ``(None, None)`` when no rotation reached the grader, so the
    dependent clause can report itself vacuous instead of grading zeros.
    """
    if rot is None:
        return None, None
    r = np.asarray(rot, dtype=float)
    if r.ndim != 3 or r.shape[1:] != (3, 3) or not len(r):
        return None, None
    pitch = -np.arcsin(np.clip(r[:, 2, 0], -1.0, 1.0))
    roll = np.arctan2(r[:, 2, 1], r[:, 2, 2])
    return roll, pitch


def tilt_from_up(rot):
    """Angle between the body's own vertical and the world's, radians.

    Reported beside :func:`roll_pitch`, never graded. The tier bounds |roll|
    and |pitch| **separately**, so a body at 0.8 rad of each -- about 1.1 rad
    off vertical -- satisfies both bounds. Printing the combined angle is how a
    reader sees that, instead of having to derive it.
    """
    if rot is None:
        return None
    r = np.asarray(rot, dtype=float)
    if r.ndim != 3 or r.shape[1:] != (3, 3) or not len(r):
        return None
    return np.arccos(np.clip(r[:, 2, 2], -1.0, 1.0))


def detrended_rms(t, z):
    """RMS of ``z`` about its own straight-line trend, metres.

    T3.3 asks whether the base *"oscillates with >= 0.005 m RMS about its
    trend"*. "Its trend" is read as the least-squares line through the window:
    on flat ground a walk's trend is flat, and a slow sag or a slow climb is
    the thing a bare standard deviation would mistake for a bob. A body that
    slid has zero residual and a body that walked does not.
    """
    t = np.asarray(t, dtype=float)
    z = np.asarray(z, dtype=float)
    if len(t) < 3 or len(z) != len(t):
        return None
    span = float(t[-1] - t[0])
    if span <= 0.0:
        return None
    a, b = np.polyfit(t, z, 1)
    return float(np.sqrt(np.mean((z - (a * t + b)) ** 2)))


# --- gap 1: the standing height ----------------------------------------------


@dataclass
class StandingHeight:
    """The settled standing height of **this** robot's base, metres.

    Task-neutral and robot-neutral by construction: it is read per run from the
    frozen t=0 inventory, which is what ``capability-ladder-plan.md`` §2 T3
    requires (*"not fixed as a constant"*). A quadruped 0.2 m tall and one 0.8
    m tall are then graded by the same rule and not by the same number.

    ``z_m is None`` is legal and is the honest answer from a column whose
    inventory carries no position. The core then falls back to the run's own
    first recorded sample, marks the clause's witness absent, and prints which
    rule it used -- it never invents a height.
    """

    z_m: Optional[float] = None
    body: str = ""
    source: str = ""
    error: Optional[str] = None

    @property
    def usable(self):
        return (self.z_m is not None and not math.isnan(float(self.z_m))
                and float(self.z_m) > 0.0)

    def as_dict(self):
        return {"standing_z_m": self.z_m, "body": self.body,
                "source": self.source, "error": self.error}


EMPTY_STANDING_HEIGHT = StandingHeight(
    error="no settled standing height was supplied by the adapter")


def standing_height_from_bundle(bundle, name=None, body_id=None):
    """Best-effort :class:`StandingHeight` from the frozen t=0 inventory.

    Genuinely answerable from the published contract: ``Body`` already carries
    a world position and the inventory is taken after the settle window, so the
    body's own z at t=0 **is** its settled standing height. No new sampler is
    needed for this one, on any column.
    """
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    for b in (getattr(inv, "bodies", None) or []):
        if (name and b.name == name) or (body_id and b.body_id == body_id):
            pos = getattr(b, "position", None)
            if pos is None or len(pos) < 3:
                return StandingHeight(
                    body=(b.name or b.body_id or ""),
                    error=("the body named %r carries no world position in "
                           "the frozen inventory" % (name or body_id)))
            return StandingHeight(
                z_m=float(pos[2]), body=(b.name or b.body_id or ""),
                source=("the run's frozen t=0 inventory: %s. The inventory is "
                        "taken after the settle window, so the body's own z "
                        "is its settled standing height"
                        % (getattr(inv, "source", "") or "unattributed")))
    return StandingHeight(
        body=(name or body_id or ""),
        error="no body named %r is in the frozen inventory" % (name or body_id))


# --- task data: which bodies are the ground ----------------------------------


@dataclass
class WalkingSurface:
    """The names the task declares for the ground the robot walks on.

    **Task data, for T2's reason applied to a harder case.** T3.4 asks that
    *nothing but the ground* touched the robot, and an adapter that could
    decide which body counts as "the ground" could label the thing holding the
    robot up as ground and pass the assertion. So the names come from the task
    file, the adapter's own classification is recorded beside them, and the
    core takes the **stricter of the two** on every disagreement (see
    :func:`classify_contacts`).
    """

    names: tuple = ()
    source: str = ""

    @property
    def usable(self):
        return bool(self.names)

    def declares(self, name):
        n = (name or "").strip().lower()
        return bool(n) and n in {str(x).strip().lower() for x in self.names}

    def as_dict(self):
        return {"names": list(self.names), "source": self.source}


# --- gap 2: the gait-contact channel -----------------------------------------


@dataclass
class GroundContact:
    """One contact between the robot under test and a body outside it.

    ``other_is_ground`` is the **adapter's** claim about the other side and
    ``None`` is allowed -- it makes the dependent clause report its witness
    absent rather than guessing, which is the same discipline
    ``SupportContact.surface_is_robot`` already lives under.

    ``t_s`` is on the same clock as the base pose series. Without it a contact
    cannot be placed inside the scored window and cannot contribute to a
    transition count.
    """

    robot_body: Optional[str] = None
    other_body: Optional[str] = None
    other_is_ground: Optional[bool] = None
    other_is_robot: Optional[bool] = None
    point: Optional[tuple] = None
    t_s: Optional[float] = None
    step: Optional[int] = None

    @property
    def named_both(self):
        return bool(self.robot_body) and bool(self.other_body)

    @property
    def distinct(self):
        return self.named_both and self.robot_body != self.other_body


@dataclass
class GaitContactObservation:
    """Robot-to-world contacts over the walk, **plus when the query ran**.

    ``sample_times`` is the channel's whole reason for existing over a plain
    contact list: transitions are changes of state, and a list of touches
    carries no evidence of a *not*-touch. A channel with contacts and no sample
    times can say "it touched the ground" and cannot say "it lifted a foot",
    and the core reports the second clause vacuous rather than counting zero.

    ``total_observed`` / ``distinct_named`` carry the meaning they carry
    everywhere else in this suite: how much contact evidence EXISTED, not how
    much survived a filter. ``None`` means *"I could not count"* and must never
    be spelled ``0``.
    """

    contacts: list = field(default_factory=list)
    sample_times: Optional[object] = None      # (M,) simulated seconds
    supported: bool = False
    total_observed: Optional[int] = None
    distinct_named: Optional[int] = None
    steps_sampled: int = 0
    window_s: Optional[float] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def timed(self):
        return any(c.t_s is not None for c in self.contacts)

    @property
    def has_sample_times(self):
        return (self.sample_times is not None
                and len(np.asarray(self.sample_times)) >= 2)

    @property
    def classified(self):
        """Every contact says whether the other side is the ground."""
        return bool(self.contacts) and all(
            c.other_is_ground is not None for c in self.contacts)

    @property
    def can_name_a_pair(self):
        """Could a robot-to-world contact have been *reported* at all?

        True only when the pipeline demonstrably produced a contact naming two
        distinct bodies. An empty list with no witness is indistinguishable
        from a broken query -- and from a body the solver put to sleep.
        """
        if not self.supported:
            return False
        if any(c.distinct for c in self.contacts):
            return True
        if self.distinct_named is None:
            return False
        return int(self.distinct_named) > 0

    @property
    def witness_detail(self):
        if not self.supported:
            return ("the adapter reports no contact query over the walk on "
                    "this column"
                    + ("; %s" % self.error if self.error else ""))
        bits = ["contacts of any kind: %s"
                % ("unknown" if self.total_observed is None
                   else self.total_observed),
                "naming two distinct bodies: %s"
                % ("unknown" if self.distinct_named is None
                   else self.distinct_named),
                "times the query ran: %s"
                % (len(np.asarray(self.sample_times))
                   if self.has_sample_times else "not reported"),
                "steps sampled: %d" % self.steps_sampled]
        if self.error:
            bits.append("error: %s" % self.error)
        return "; ".join(bits)

    def as_dict(self):
        return {"contacts": len(self.contacts),
                "times the query ran": (len(np.asarray(self.sample_times))
                                        if self.has_sample_times else None),
                "contacts of any kind observed": self.total_observed,
                "contacts naming two distinct bodies": self.distinct_named,
                "every contact says which side is the ground": self.classified,
                "source": self.source, "error": self.error}


EMPTY_GAIT_CONTACTS = GaitContactObservation(
    error="no gait-contact channel was supplied by the adapter")


def classify_contacts(obs, surface):
    """``(ground, foreign, disagreements)`` over one observation.

    The rule, stated so it can be attacked: a contact counts as **ground** only
    when the task's own :class:`WalkingSurface` declares the other body's name
    *and* the adapter attests it is the ground. Every disagreement between the
    two is recorded and resolved **towards the stricter reading** in both
    directions --

    * the adapter says ground, the task does not name it -> **not ground**
      (otherwise an adapter could label the thing holding the robot up as the
      floor and pass T3.4 outright);
    * the task names it, the adapter says it is not -> **not ground** (an
      adapter that positively denies it knows something the task file cannot).

    ``foreign`` is what T3.4's first half counts: a contact with a body that is
    neither the ground nor part of the robot under test. A contact the adapter
    flags as another robot is foreign too -- another robot propping this one up
    is exactly what the clause exists to catch.
    """
    ground, foreign, disagreements = [], [], []
    for c in obs.contacts:
        if not c.distinct:
            continue
        named = surface.declares(c.other_body) if surface.usable else None
        attested = c.other_is_ground
        if named is not None and attested is not None and named != attested:
            note = ("the adapter says %r %s the ground and the task %s it"
                    % (c.other_body, "is" if attested else "is not",
                       "declares" if named else "does not name"))
            if note not in disagreements:
                disagreements.append(note)
        is_ground = bool(attested is True and (named is not False))
        if surface.usable:
            is_ground = bool(attested is True and named is True)
        if is_ground:
            ground.append(c)
        else:
            foreign.append(c)
    return ground, foreign, disagreements


def contact_states(obs, contacts):
    """``(times, bodies, state)`` -- which robot bodies were touching, when.

    ``state`` is (M, K) booleans over the times the query ran. Returns
    ``(None, [], None)`` when the channel did not report those times, because
    a transition is a change of state and a list of touches carries no
    evidence of a not-touch (gap 2 in the module docstring).

    A contact is placed on the nearest sample time and only if it lands within
    half a sample interval of it -- a contact stamped between two sweeps
    belongs to neither and is counted in ``unplaced`` rather than smeared onto
    whichever was closer.
    """
    if not obs.has_sample_times:
        return None, [], None
    ts = np.asarray(obs.sample_times, dtype=float)
    order = np.argsort(ts)
    ts = ts[order]
    bodies = sorted({c.robot_body for c in contacts if c.robot_body})
    if not bodies:
        return ts, [], np.zeros((len(ts), 0), dtype=bool)
    col = {b: i for i, b in enumerate(bodies)}
    state = np.zeros((len(ts), len(bodies)), dtype=bool)
    d = np.diff(ts)
    tol = (float(np.median(d)) / 2.0 if len(d) else 0.0) + 1e-9
    for c in contacts:
        if c.t_s is None or c.robot_body not in col:
            continue
        k = int(np.clip(np.searchsorted(ts, float(c.t_s)), 1, len(ts) - 1))
        k = k - 1 if abs(ts[k - 1] - c.t_s) <= abs(ts[k] - c.t_s) else k
        if abs(float(ts[k]) - float(c.t_s)) <= tol:
            state[k, col[c.robot_body]] = True
    return ts, bodies, state


# --- gap 3: the applied-wrench channel ---------------------------------------


@dataclass
class AppliedSupport:
    """Every non-gravitational, non-contact wrench applied to the base.

    Two shapes are accepted, because two shapes exist in the wild: a per-tick
    series (``t`` / ``force`` / ``torque``), which is what this tree's own
    deploy hook can print from the wrench it is already applying, and bare
    summaries (``peak_force_n`` / ``peak_torque_nm`` /
    ``fraction_nonzero``) from a column that can total the wrench but not
    stream it. Either is ``attested``; neither being present is not.

    **``attested`` is a positive claim and defaults to ``None``.** ``None`` and
    ``False`` both mean the column did not establish what was applied, which
    ``capability-ladder-plan.md`` §2 T3.4 turns into ``support_attestation:
    unverified`` and exclusion from comparison -- never into a pass. There is
    deliberately no "no support was applied" default: a channel that reports
    zero because it was never wired up and a channel that measured zero are the
    same reading unless the column says which it is.
    """

    attested: Optional[bool] = None
    t: Optional[object] = None                 # (M,) simulated seconds
    force: Optional[object] = None             # (M, 3) newtons, world frame
    torque: Optional[object] = None            # (M, 3) newton-metres
    peak_force_n: Optional[float] = None
    peak_torque_nm: Optional[float] = None
    fraction_nonzero: Optional[float] = None
    zero_tol_n: float = 1e-9
    source: str = ""
    error: Optional[str] = None

    @property
    def has_series(self):
        if self.force is None and self.torque is None:
            return False
        for arr in (self.force, self.torque):
            if arr is None:
                continue
            a = np.asarray(arr, dtype=float)
            if a.ndim != 2 or a.shape[1] != 3 or not len(a):
                return False
        return True

    @property
    def usable(self):
        """The column positively attested a wrench, and gave numbers for it."""
        if self.attested is not True:
            return False
        return bool(self.has_series
                    or (self.peak_force_n is not None
                        and self.peak_torque_nm is not None))

    @property
    def peak_force(self):
        """Largest applied force magnitude over the window, newtons."""
        if self.has_series and self.force is not None:
            f = np.asarray(self.force, dtype=float)
            return float(np.linalg.norm(f, axis=1).max())
        return (None if self.peak_force_n is None
                else float(self.peak_force_n))

    @property
    def peak_torque(self):
        """Largest applied torque magnitude over the window, newton-metres."""
        if self.has_series and self.torque is not None:
            m = np.asarray(self.torque, dtype=float)
            return float(np.linalg.norm(m, axis=1).max())
        return (None if self.peak_torque_nm is None
                else float(self.peak_torque_nm))

    @property
    def fraction_window_nonzero(self):
        """Fraction of the window in which anything at all was applied.

        §2 T4 requires this number printed inside a supported cell, and T3
        prints it too: "held up for 3 % of the walk" and "held up throughout"
        are different findings and a peak alone cannot tell them apart.
        """
        if self.has_series:
            n = 0
            live = None
            for arr in (self.force, self.torque):
                if arr is None:
                    continue
                a = np.linalg.norm(np.asarray(arr, dtype=float), axis=1)
                n = max(n, len(a))
                hot = a > float(self.zero_tol_n)
                live = hot if live is None else (live | hot)
            if live is None or not n:
                return None
            return float(np.count_nonzero(live)) / float(n)
        return (None if self.fraction_nonzero is None
                else float(self.fraction_nonzero))

    @property
    def per_channel(self):
        """Per-axis peaks, so a reader sees *which* way it was held up.

        The measured G1 support is the reason this is per-axis rather than one
        magnitude: on that rig the vertical channel read 0 N for the whole walk
        while the attitude channel was non-zero 100 % of it. One magnitude
        would have hidden the entire finding.
        """
        if not self.has_series:
            return None
        out = {}
        for label, arr in (("force (N)", self.force),
                           ("torque (N.m)", self.torque)):
            if arr is None:
                continue
            a = np.abs(np.asarray(arr, dtype=float))
            out[label] = {axis: round(float(a[:, i].max()), 6)
                          for i, axis in enumerate(("x", "y", "z"))}
        return out or None

    def as_dict(self):
        return {"attested": self.attested,
                "peak applied force (N)": self.peak_force,
                "peak applied torque (N.m)": self.peak_torque,
                "fraction of the window with anything applied":
                    self.fraction_window_nonzero,
                "per axis": self.per_channel,
                "samples": (len(np.asarray(self.force))
                            if self.has_series and self.force is not None
                            else None),
                "source": self.source, "error": self.error}


EMPTY_APPLIED_SUPPORT = AppliedSupport(
    error="no applied-wrench channel was supplied by the adapter, so what was "
          "holding the robot up is unknown rather than known to be nothing")


# --- gap 4: the arena ---------------------------------------------------------


@dataclass
class ArenaBounds:
    """The horizontal extent of the free walking region, world metres.

    Not "the world" and not "the floor mesh" -- the region the robot could
    actually cross. It is what makes two different findings distinguishable:
    *the walker stopped* and *the walker ran out of floor*.

    ``boundary_bodies`` names what bounds it, where the column can say. It is
    reported, never graded: a contact with one of them is already a foreign
    contact under T3.4 and needs no second rule.
    """

    aabb_min: Optional[tuple] = None
    aabb_max: Optional[tuple] = None
    boundary_bodies: list = field(default_factory=list)
    source: str = ""
    error: Optional[str] = None

    @property
    def has_aabb(self):
        return self.aabb_min is not None and self.aabb_max is not None

    @property
    def usable(self):
        s = self.span
        return bool(s is not None and min(s[0], s[1]) > 0.0)

    @property
    def span(self):
        if not self.has_aabb:
            return None
        return tuple(float(b) - float(a)
                     for a, b in zip(self.aabb_min, self.aabb_max))

    def free_run_up_from(self, xy):
        """The longest straight run available from ``xy``, metres.

        The greatest distance from the start to any point of the region -- i.e.
        to its furthest corner. It is an upper bound on what a straight-line
        walk could have covered, which is the right side to err on: a region
        that fails the task's stated minimum *by this measure* fails it by any
        measure.
        """
        if not self.usable:
            return None
        best = 0.0
        for cx in (float(self.aabb_min[0]), float(self.aabb_max[0])):
            for cy in (float(self.aabb_min[1]), float(self.aabb_max[1])):
                best = max(best, math.hypot(cx - float(xy[0]),
                                            cy - float(xy[1])))
        return best

    def distance_to_boundary(self, xy):
        """Distance from ``xy`` to the nearest edge of the region, metres.

        Negative outside it. Small means "there is no more floor that way",
        which is the signal an arena stop is built on.
        """
        if not self.usable:
            return None
        x, y = float(xy[0]), float(xy[1])
        return float(min(x - float(self.aabb_min[0]),
                         float(self.aabb_max[0]) - x,
                         y - float(self.aabb_min[1]),
                         float(self.aabb_max[1]) - y))

    def as_dict(self):
        return {"aabb_min_m": self.aabb_min, "aabb_max_m": self.aabb_max,
                "span_m": self.span, "boundary_bodies": self.boundary_bodies,
                "source": self.source, "error": self.error}


EMPTY_ARENA = ArenaBounds(
    error="no walking-region extent was supplied by the adapter, so a run "
          "that ended on world geometry cannot be told from one that stalled")


def arena_from_bundle(bundle, surface):
    """Best-effort :class:`ArenaBounds` from the frozen t=0 inventory.

    Answerable from the published contract wherever the walking surface is a
    bounded body: ``Body`` already carries a world AABB, so the union of the
    boxes of the bodies the task names as ground **is** the free region. It
    returns unusable for an unbounded ground -- an infinite plane has no extent
    and a column that models the floor that way genuinely cannot answer the
    question from this channel.
    """
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    lo, hi, seen = None, None, []
    for b in (getattr(inv, "bodies", None) or []):
        if not surface.declares(b.name) or not getattr(b, "has_aabb", False):
            continue
        seen.append(b.name)
        lo = (list(b.aabb_min) if lo is None
              else [min(a, c) for a, c in zip(lo, b.aabb_min)])
        hi = (list(b.aabb_max) if hi is None
              else [max(a, c) for a, c in zip(hi, b.aabb_max)])
    if lo is None:
        return ArenaBounds(
            error=("no body the task names as the walking surface (%s) is in "
                   "the frozen inventory with a world bounding box"
                   % (", ".join(surface.names) or "none declared")))
    return ArenaBounds(
        aabb_min=tuple(lo), aabb_max=tuple(hi),
        source=("the union of the world bounding boxes of %s, from the run's "
                "frozen t=0 inventory: %s"
                % (", ".join(seen),
                   getattr(inv, "source", "") or "unattributed")))


# --- gap 5: the controller-load attestation ----------------------------------


@dataclass
class ControllerLoad:
    """Did the thing that was supposed to drive the robot actually load?

    ``loaded`` is a **positive** attestation and ``None`` is the normal answer
    from a column with no channel for it -- which makes the clause red and
    vacuous, and the blocker ours. It is deliberately not defaulted to ``True``
    on a clean exit: the trap this clause exists for is precisely a deploy that
    ran a zero-residual baseline **and exited 0**.

    ``declared_method`` is the plan's ``method`` field (§3.2): recorded in
    every cell, graded in none. ``evidence`` is the line or marker the load was
    read from -- an attestation with no evidence is somebody's opinion.
    """

    declared_method: str = "unknown"
    loaded: Optional[bool] = None
    evidence: str = ""
    identity: str = ""          # what loaded: a checkpoint name, a hash, a file
    source: str = ""
    error: Optional[str] = None

    @property
    def method(self):
        """The declared method, normalised into the four. Never graded."""
        m = (self.declared_method or "").strip().lower().replace(" ", "_")
        return m if m in METHODS else "unknown"

    @property
    def attested(self):
        return self.loaded is not None

    def as_dict(self):
        return {"method": self.method, "declared_as": self.declared_method,
                "loaded": self.loaded, "evidence": self.evidence,
                "what loaded": self.identity, "source": self.source,
                "error": self.error}


EMPTY_CONTROLLER_LOAD = ControllerLoad(
    error="no controller-load attestation was supplied by the adapter; an "
          "exit code is not one")


# --- what T3's core is handed -------------------------------------------------


@dataclass
class T3Evidence:
    """One T3 run: the neutral bundle, unmodified, plus T3's channels.

    Composition rather than mutation, for the reason ``T1Evidence`` gives:
    ``bundle`` is byte for byte what the column's own ``build_bundle``
    returned, so a reader can check the ladder against the published adapter
    contract without diffing a subclass.
    """

    bundle: object
    base_pose: PoseSeries = field(default_factory=lambda: EMPTY_POSE_SERIES)
    standing: StandingHeight = field(
        default_factory=lambda: EMPTY_STANDING_HEIGHT)
    gait: GaitContactObservation = field(
        default_factory=lambda: EMPTY_GAIT_CONTACTS)
    support: AppliedSupport = field(
        default_factory=lambda: EMPTY_APPLIED_SUPPORT)
    arena: ArenaBounds = field(default_factory=lambda: EMPTY_ARENA)
    base_physics: BodyPhysics = field(
        default_factory=lambda: EMPTY_BODY_PHYSICS)
    world: WorldPhysics = field(default_factory=lambda: EMPTY_WORLD_PHYSICS)
    controller: ControllerLoad = field(
        default_factory=lambda: EMPTY_CONTROLLER_LOAD)
    surface: WalkingSurface = field(default_factory=WalkingSurface)
    robot_name: str = ""
    #: The mass the task declares for the robot, kilograms, or None. Used ONLY
    #: to resolve the base body when a column's importer does not reproduce the
    #: declared link name -- never to grade anything. See
    #: ``t3_core.select_base_body``.
    robot_mass_kg_declared: Optional[float] = None
    notes: list = field(default_factory=list)

    @property
    def body_weight_n(self):
        """``m.g`` for the robot under test, newtons, or ``None``.

        The datum T3.4's force threshold is stated as a fraction of. Absent, a
        peak in newtons cannot be compared with ``0.02 x m.g`` at all, which is
        why its absence makes the support **unverified** rather than making the
        threshold something else.
        """
        m = self.base_physics.mass_kg
        g = self.world.gravity_mps2
        if m is None or g is None:
            return None
        m, g = float(m), float(g)
        if m <= 0.0 or g <= 0.0:
            return None
        return m * g

    def provenance(self):
        base = {}
        try:
            base = dict(self.bundle.provenance())
        except Exception:  # noqa: BLE001  (evidence must never raise)
            base = {}
        base["base_pose"] = self.base_pose.source
        base["standing_height"] = self.standing.source
        base["gait_contacts"] = self.gait.source
        base["applied_support"] = self.support.source
        base["arena"] = self.arena.source
        base["base_mass"] = self.base_physics.source
        base["gravity"] = self.world.source
        base["controller_load"] = self.controller.source
        base["walking_surface"] = self.surface.source
        return base


# The channel an assertion dies without, so a gap is reported against the
# assertion it actually breaks rather than as a general shrug. Mirrors
# ``ladder.adapters.T2_CHANNEL_ASSERTIONS`` in shape and in purpose.
T3_CHANNEL_ASSERTIONS = {
    "base_pose": ("T3.1", "T3.2", "T3.3"),
    "base_orientation": ("T3.2",),
    "standing_height": ("T3.2",),
    "gait_contacts": ("T3.3", "T3.4"),
    "applied_support": ("T3.4",),
    "base_mass": ("T3.4",),
    "gravity": ("T3.4",),
    "controller_load": ("T3.5",),
    "arena": (),      # a finding, never a pass condition -- see the docstring
}

# What a column must be able to answer BEYOND agentbench's REQUIRED_EVIDENCE
# and the ladder's T1/T2 lists before a T3 cell on it can be published.
LADDER_REQUIRED_EVIDENCE_T3 = (
    ("walking_surface", "nothing -- which bodies count as the ground is task "
                        "data, for the reason the waypoint and the roles are. "
                        "An adapter that could decide what counts as the "
                        "ground could label the thing holding the robot up as "
                        "the floor."),
    ("base_pose", "the base's world position AND ORIENTATION over the run, on "
                  "the simulated clock. The orientation is not optional: T3.2 "
                  "bounds roll and pitch at every sample, and the frozen "
                  "motion contract carries neither."),
    ("standing_height", "the base's settled standing height, per robot, from "
                        "the t=0 inventory -- so the fall test is robot-"
                        "neutral instead of a constant somebody typed."),
    ("gait_contacts", "contacts naming a robot body and the body it is "
                      "standing on, timestamped, WITH the times the query ran "
                      "-- a list of touches carries no evidence of a lifted "
                      "foot, and the make-and-break count is the difference "
                      "between a walk and a slide."),
    ("applied_support", "every non-gravitational, non-contact wrench applied "
                        "to the base, as force (N) and torque (N.m), with the "
                        "fraction of the window it was live. Absent, the cell "
                        "is support_attestation: unverified and is excluded "
                        "from comparison -- never credited."),
    ("base_mass", "the base's mass in kilograms and the world's gravity: the "
                  "threshold is 0.02 x m.g and without m.g there is nothing "
                  "for a measured force to be compared against."),
    ("controller_load", "a positive attestation that the policy or controller "
                        "loaded, and the evidence line it was read from. An "
                        "exit code is not evidence -- a deploy whose model "
                        "runtime is missing runs a zero-residual baseline and "
                        "exits 0."),
    ("arena", "the horizontal extent of the free walking region. Not a pass "
              "condition: it is what stops a run that ended on world geometry "
              "from being published as a gait that degraded."),
)


def unanswered_channels(ev):
    """``{channel: (assertion, ...)}`` for what this evidence cannot answer.

    Computed from the evidence itself rather than from which hook fired, for
    the reason the T2 sibling gives: a column that supplies a channel and fills
    it with nothing has the same gap as one that supplies no channel at all,
    and the cell's blocker is ours either way.
    """
    out = {}
    if ev is None:
        return {"everything": ("T3.1", "T3.2", "T3.3", "T3.4", "T3.5")}
    if not ev.base_pose.usable:
        out["base_pose"] = T3_CHANNEL_ASSERTIONS["base_pose"]
    elif not ev.base_pose.has_orientation:
        out["base_orientation"] = T3_CHANNEL_ASSERTIONS["base_orientation"]
    if not ev.standing.usable:
        out["standing_height"] = T3_CHANNEL_ASSERTIONS["standing_height"]
    if not (ev.gait.supported and ev.gait.has_sample_times):
        out["gait_contacts"] = T3_CHANNEL_ASSERTIONS["gait_contacts"]
    if not ev.support.usable:
        out["applied_support"] = T3_CHANNEL_ASSERTIONS["applied_support"]
    if ev.base_physics.mass_kg is None:
        out["base_mass"] = T3_CHANNEL_ASSERTIONS["base_mass"]
    if ev.world.gravity_acting is None:
        out["gravity"] = T3_CHANNEL_ASSERTIONS["gravity"]
    if not ev.controller.attested:
        out["controller_load"] = T3_CHANNEL_ASSERTIONS["controller_load"]
    if not ev.arena.usable:
        out["arena"] = T3_CHANNEL_ASSERTIONS["arena"]
    return out


def check_t3_evidence(ev):
    """Contract violations in one :class:`T3Evidence`, as strings.

    The sibling of ``check_t1_evidence`` / ``check_t2_evidence``. A new column
    runs this, plus ``agentbench.adapters.check_bundle``, against its own
    output before anyone spends a token on a T3 cell.
    """
    p = []
    if ev is None:
        return ["no T3 evidence at all"]
    if not ev.surface.usable:
        p.append("walking surface: the task named no ground body, so 'nothing "
                 "but the ground touched it' has nothing to be measured "
                 "against")
    elif not ev.surface.source:
        p.append("walking surface: no source citation -- names with no "
                 "provenance are names somebody typed")
    if not ev.base_pose.usable:
        p.append("base pose: no usable series on a simulated clock (%s)"
                 % (ev.base_pose.error or "no reason given"))
    else:
        if not ev.base_pose.source:
            p.append("base pose: supplied with no source citation")
        if not ev.base_pose.has_orientation:
            p.append("base pose: position only, no orientation, so roll and "
                     "pitch are not measurable and the fall test grades half "
                     "of what the tier states")
    if not ev.standing.usable:
        p.append("standing height: not attested per robot, so the fall "
                 "threshold falls back to the run's own first sample (%s)"
                 % (ev.standing.error or "no reason given"))
    if not ev.gait.supported:
        p.append("gait contacts: no contact query over the walk (%s)"
                 % (ev.gait.error or "no reason given"))
    else:
        if not ev.gait.has_sample_times:
            p.append("gait contacts: the times the query ran were not "
                     "reported, so a lifted foot is unobservable and the "
                     "make-and-break clause reports vacuous")
        if ev.gait.total_observed is None and ev.gait.distinct_named is None:
            p.append("gait contacts: no vacuity witness (total_observed / "
                     "distinct_named both None), so 'it never touched the "
                     "ground' cannot be told apart from a query that names "
                     "nothing")
        if not ev.gait.classified:
            p.append("gait contacts: at least one contact does not say "
                     "whether the other side is the ground, so 'nothing but "
                     "the ground touched it' cannot be established")
    if not ev.support.usable:
        p.append("applied support: not attested, so the cell is "
                 "support_attestation: unverified and is excluded from "
                 "comparison with any cell that has it (%s)"
                 % (ev.support.error or "no reason given"))
    elif not ev.support.source:
        p.append("applied support: attested with no source citation")
    if ev.body_weight_n is None:
        p.append("body weight: the base's mass or the world's gravity is "
                 "unknown, so the 0.02 x m.g threshold has no value and the "
                 "support cannot be checked against it")
    if not ev.controller.attested:
        p.append("controller load: no positive attestation that the policy or "
                 "controller loaded (%s). An exit code is not one"
                 % (ev.controller.error or "no reason given"))
    elif ev.controller.loaded and not ev.controller.evidence:
        p.append("controller load: attested with no evidence line -- an "
                 "attestation with nothing behind it is an opinion")
    if not ev.arena.usable:
        p.append("arena: no walking-region extent, so a run that ended on "
                 "world geometry cannot be distinguished from one that "
                 "stalled, and the task's stated free run-up cannot be "
                 "checked (%s)" % (ev.arena.error or "no reason given"))
    return p


__all__ = ["UNITS", "METHODS", "ATTESTED", "UNVERIFIED",
           "TERMINATIONS", "TERMINATION_ARENA", "TERMINATION_CONTROLLER",
           "TERMINATION_FELL", "TERMINATION_TIME_LIMIT", "TERMINATION_UNKNOWN",
           "roll_pitch", "tilt_from_up", "detrended_rms",
           "StandingHeight", "EMPTY_STANDING_HEIGHT",
           "standing_height_from_bundle", "WalkingSurface",
           "GroundContact", "GaitContactObservation", "EMPTY_GAIT_CONTACTS",
           "classify_contacts", "contact_states",
           "AppliedSupport", "EMPTY_APPLIED_SUPPORT",
           "ArenaBounds", "EMPTY_ARENA", "arena_from_bundle",
           "ControllerLoad", "EMPTY_CONTROLLER_LOAD",
           "T3Evidence", "T3_CHANNEL_ASSERTIONS",
           "LADDER_REQUIRED_EVIDENCE_T3", "unanswered_channels",
           "check_t3_evidence",
           # re-exported from the shared ladder channels so a T3 core has one
           # import and the shared pieces cannot be re-declared here
           "BodyPhysics", "EMPTY_BODY_PHYSICS", "EMPTY_POSE_SERIES",
           "EMPTY_WORLD_PHYSICS", "PoseSeries", "WorldPhysics",
           "body_physics_from_bundle", "max_step_jump",
           "pose_series_from_bundle", "run_is_real"]
