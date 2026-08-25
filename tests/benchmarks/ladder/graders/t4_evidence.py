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

"""The one thing **T4** needs that T3 does not: the support, read per channel.

``capability-ladder-plan.md`` §2 T4 states the tier as *"T3.1, T3.2, T3.3,
T3.5 unchanged. T3.4 is **replaced** by the support measurement below"*. So
this module is deliberately thin, and what it is thin **of** is the point:

* T4's evidence channels are **T3's channels, unchanged**. The tier replaces an
  assertion, not an evidence contract, and :class:`T4Evidence` is a subclass of
  :class:`~ladder.graders.t3_evidence.T3Evidence` that adds no field. Every
  dataclass, every helper and every fallback builder in ``t3_evidence`` is
  imported and re-exported here so a T4 core has one import and **no shared
  piece can be re-declared**.
* What T4 adds is a **reading** of the applied-wrench channel that T3 does not
  need: :class:`SupportProfile`, computed over the walk's own scored window,
  **per channel**, and carrying the cell the run belongs in.

--------------------------------------------------------------------------
Why the profile is per channel, and why one number would have been a lie
--------------------------------------------------------------------------

``docs/developer/g1-endurance-2026-08-01.md`` §4, measured on this tree's own
flagship over a whole 10 m walk, body weight 334.5 N:

===========  ==========  =============================
channel      peak        fraction of the window live
===========  ==========  =============================
fz (carry)   **0.00 N**  **0 %**
fy (catch)   36.8 N      100 %
tx (roll)    **69.2 N.m**  **100 %**
ty (pitch)   21.4 N.m    100 %
tz (yaw)     0.0 N.m     0 %
===========  ==========  =============================

**The weight-bearing channel was idle for the entire walk while the attitude
channel ran continuously at a 69.2 N.m peak on a 34 kg robot.** A single
scalar -- "peak applied force 36.8 N, non-zero 100 % of the window" -- is true
and hides the whole finding: a reader could not tell a rig that carries a robot
from one that only keeps it from toppling, and those are different claims about
what the robot did. §2 T4's own reasoning for measuring rather than asserting
is *"'harnessed / unharnessed' hides the difference between a fingertip and a
crane"*; the same argument one level down is why the profile is per axis.

--------------------------------------------------------------------------
The two cells, and the third state that is neither
--------------------------------------------------------------------------

§2 T4 publishes **two cells from one recording**:

=====================  ===================================================
``T4-unsupported``     peak non-gravitational, non-contact force
                       <= 0.02 x m.g **and** <= 2 N.m -- numerically nothing
``T4-supported``       anything larger, published **with** the measured peak
                       force as a multiple of body weight, the peak applied
                       torque, and the fraction of the walk window during
                       which support was non-zero, printed inside the cell
=====================  ===================================================

A supported run is **not a failure of the tier** -- it is a different cell --
and §8 forbids quoting either without its support figures. :meth:`
SupportProfile.cell_text` renders exactly the string the plan writes by hand.

:data:`CELL_UNVERIFIED` is the third state and it is **not** one of the plan's
two cells: it is where a column that cannot attest the applied wrench lands.
§2 T4's own consequence -- *"Where a simulator cannot attest applied forces,
the cell is ``achieved (support unverified)`` and is excluded from comparison
with an attested cell"* -- is that it is recorded, excluded, and **never
silently credited and never failed**. Failing it would publish our own missing
channel as somebody else's capability gap, which §4 forbids.

--------------------------------------------------------------------------
And the reading that decides whether the whole tier is honest
--------------------------------------------------------------------------

The profile is measured over the **scored window** -- the walk -- and not over
whatever stretch of recording a column happened to log. The reason is the same
measured failure the arena rule exists for: in all six endurance runs the world
ended the run and the robot then spent two hundred seconds pressed against a
wall. A support fraction averaged over *that* is not the support applied during
the walk, and a peak taken from it is not the peak that carried the robot ten
metres. Where the column's wrench log carries no clock, the window cannot be
applied, the profile says so (``window_applied`` False), and the numbers are
the whole log's -- which is the honest weaker statement, not a silent one.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ladder.graders.t3_evidence import (  # noqa: E402,F401 (re-exported)
    ATTESTED, EMPTY_APPLIED_SUPPORT, EMPTY_ARENA, EMPTY_BODY_PHYSICS,
    EMPTY_CONTROLLER_LOAD, EMPTY_GAIT_CONTACTS, EMPTY_POSE_SERIES,
    EMPTY_STANDING_HEIGHT, EMPTY_WORLD_PHYSICS, METHODS, TERMINATION_ARENA,
    TERMINATION_CONTROLLER, TERMINATION_FELL, TERMINATION_TIME_LIMIT,
    TERMINATION_UNKNOWN, TERMINATIONS, UNITS, UNVERIFIED, AppliedSupport,
    ArenaBounds, BodyPhysics, ControllerLoad, GaitContactObservation,
    GroundContact, PoseSeries, StandingHeight, T3Evidence, WalkingSurface,
    WorldPhysics, arena_from_bundle, body_physics_from_bundle,
    classify_contacts, contact_states, detrended_rms, max_step_jump,
    pose_series_from_bundle, roll_pitch, run_is_real,
    standing_height_from_bundle, tilt_from_up)

# ``capability-ladder-plan.md`` §2 T4's own two cells, spelled as it spells
# them. They are cell NAMES, not outcomes: a run lands in one of them, and both
# are publishable.
CELL_UNSUPPORTED = "T4-unsupported"
CELL_SUPPORTED = "T4-supported"
# ...and the state that is neither, for a column that cannot say what was
# applied. Named differently on purpose so it can never be counted as one of
# the two published cells.
CELL_UNVERIFIED = "T4-support-unverified"
CELLS = (CELL_UNSUPPORTED, CELL_SUPPORTED, CELL_UNVERIFIED)

AXES = ("x", "y", "z")

# The two labels the plan's table states, in the units it states them in. They
# are imported by the core from ``t3_core`` -- the same numbers, one source --
# and named here only so a reader of this module sees what a profile is being
# compared against.
_FORCE_LABEL = "force (N)"
_TORQUE_LABEL = "torque (N.m)"


# --- the profile --------------------------------------------------------------


@dataclass
class SupportProfile:
    """What was applied to the base over the walk, per channel, and the cell.

    Every field is a measurement or a citation. **Nothing here is a pass
    condition** -- ``within_unsupported_bounds`` selects which of the plan's
    two cells the run is published in, and §2 T4 is explicit that a supported
    run is a different cell rather than a failed one.

    ``attested`` is a **positive** claim about the column, and it needs two
    things at once: the wrench itself, and a body weight for the force to be
    expressed as a multiple of. ``0.02 x m.g`` is not a threshold without
    ``m.g``, so a wrench with no mass behind it is ``unverified`` rather than
    compared against a number nobody has.
    """

    attested: bool = False
    per_channel_attested: bool = False
    window_applied: bool = False
    window: tuple = (None, None)
    samples: int = 0
    body_weight_n: Optional[float] = None
    force_limit_n: Optional[float] = None
    peak_force_n: Optional[float] = None
    peak_force_x_weight: Optional[float] = None
    force_fraction_nonzero: Optional[float] = None
    peak_torque_nm: Optional[float] = None
    torque_fraction_nonzero: Optional[float] = None
    any_fraction_nonzero: Optional[float] = None
    per_axis: Optional[dict] = None
    within_unsupported_bounds: Optional[bool] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def cell(self):
        """Which of §2 T4's cells this run is published in."""
        if not self.attested:
            return CELL_UNVERIFIED
        return (CELL_UNSUPPORTED if self.within_unsupported_bounds
                else CELL_SUPPORTED)

    def cell_text(self):
        """The parenthetical §2 T4 writes by hand, rendered from measurement.

        The plan's own worked example is
        ``achieved 1/3 (supported: peak 2.09 x body weight, 348 N.m, 100% of
        window; reuse_class: assembled)``. This produces the middle of it, so
        the three figures §8 forbids omitting cannot be omitted by the tool
        that prints the cell either.
        """
        if not self.attested:
            return ("support unverified: nothing attested what was applied to "
                    "this robot, so this cell is excluded from comparison "
                    "with any cell that has an attested total")
        return ("%s: peak %s x body weight, %s N.m, %s%% of window"
                % ("unsupported" if self.within_unsupported_bounds
                   else "supported",
                   _g(self.peak_force_x_weight), _g(self.peak_torque_nm),
                   _pct(self.any_fraction_nonzero)))

    def as_dict(self):
        return {"cell": self.cell,
                "cell_text": self.cell_text(),
                "attested": self.attested,
                "measured per channel": self.per_channel_attested,
                "measured over the walk rather than the whole recording":
                    self.window_applied,
                "the window it was measured over (s)": list(self.window),
                "samples in that window": self.samples,
                "peak applied force (N)": self.peak_force_n,
                "...as a multiple of body weight": self.peak_force_x_weight,
                "fraction of the window a force was applied":
                    self.force_fraction_nonzero,
                "peak applied torque (N.m)": self.peak_torque_nm,
                "fraction of the window a torque was applied":
                    self.torque_fraction_nonzero,
                "fraction of the window anything at all was applied":
                    self.any_fraction_nonzero,
                "per channel": self.per_axis,
                "body weight, m.g (N)": self.body_weight_n,
                "the force bound of the unsupported cell (N)":
                    self.force_limit_n,
                "within the unsupported cell's bounds":
                    self.within_unsupported_bounds,
                "source": self.source, "error": self.error}


EMPTY_SUPPORT_PROFILE = SupportProfile(
    error="no applied-wrench channel reached the grader, so what was holding "
          "this robot up is unknown rather than known to be nothing")


def _g(x, digits=4):
    return "unknown" if x is None else ("%.*g" % (digits, float(x)))


def _pct(x):
    return "unknown" if x is None else ("%.3g" % (100.0 * float(x)))


def _axis_stats(arr, tol):
    """``{axis: {peak, fraction non-zero}}`` for one (N, 3) channel."""
    a = np.abs(np.asarray(arr, dtype=float))
    n = max(len(a), 1)
    return {axis: {"peak": round(float(a[:, i].max()), 6),
                   "fraction of the window it was non-zero":
                       round(float(np.count_nonzero(a[:, i] > tol)) / n, 6)}
            for i, axis in enumerate(AXES)}


def _window_mask(t, lo, hi):
    if t is None or lo is None or hi is None:
        return None
    ts = np.asarray(t, dtype=float)
    if not len(ts):
        return None
    return (ts >= float(lo)) & (ts <= float(hi))


def support_profile(support, weight_n, t_lo=None, t_hi=None,
                    force_fraction=None, torque_limit_nm=None):
    """Read one :class:`AppliedSupport` into a :class:`SupportProfile`.

    ``force_fraction`` and ``torque_limit_nm`` are the unsupported cell's two
    bounds. They are **passed in** rather than declared here: they are the same
    two numbers T3.4 already states, the core owns them, and a second copy in
    this module would be a second threshold pretending to be one number.

    Never raises. A channel that cannot answer produces a profile that says so
    -- there is deliberately no "nothing was applied" default, because a
    channel that reports zero because it was never wired up and one that
    measured zero are the same reading unless the column says which it is.
    """
    if support is None:
        return EMPTY_SUPPORT_PROFILE
    weight = (None if weight_n is None or float(weight_n) <= 0.0
              else float(weight_n))
    limit_n = (None if (weight is None or force_fraction is None)
               else float(force_fraction) * weight)
    tol = float(getattr(support, "zero_tol_n", 1e-9) or 1e-9)

    if not support.usable:
        return SupportProfile(
            body_weight_n=weight, force_limit_n=limit_n,
            source=support.source,
            error=(support.error
                   or "the column did not positively attest what was applied"))

    force = (None if support.force is None
             else np.asarray(support.force, dtype=float))
    torque = (None if support.torque is None
              else np.asarray(support.torque, dtype=float))
    lo, hi = t_lo, t_hi
    window_applied = False
    samples = 0
    per_axis = None
    f_peak = m_peak = f_frac = m_frac = any_frac = None

    if support.has_series:
        mask = _window_mask(support.t, lo, hi)
        for arr in (force, torque):
            if arr is not None and mask is not None and len(mask) != len(arr):
                mask = None
        if mask is not None and bool(mask.any()):
            force = None if force is None else force[mask]
            torque = None if torque is None else torque[mask]
            window_applied = True
        else:
            lo = hi = None
        per_axis = {}
        live = None
        for label, arr in ((_FORCE_LABEL, force), (_TORQUE_LABEL, torque)):
            if arr is None:
                continue
            samples = max(samples, len(arr))
            per_axis[label] = _axis_stats(arr, tol)
            mag = np.linalg.norm(arr, axis=1)
            frac = (float(np.count_nonzero(mag > tol)) / max(len(mag), 1))
            hot = mag > tol
            live = hot if live is None else (live | hot)
            if label == _FORCE_LABEL:
                f_peak, f_frac = float(mag.max()), frac
            else:
                m_peak, m_frac = float(mag.max()), frac
        any_frac = (None if live is None
                    else float(np.count_nonzero(live)) / max(len(live), 1))
        per_channel = bool(per_axis) and force is not None and torque is not None
    else:
        # A column that can total the wrench but not stream it. Legal, and
        # weaker: no clock means no walk window, and no axes means the idle
        # channel and the continuous one cannot be told apart.
        f_peak, m_peak = support.peak_force, support.peak_torque
        any_frac = support.fraction_window_nonzero
        per_channel = False
        lo = hi = None

    if f_peak is None or m_peak is None:
        return SupportProfile(
            per_channel_attested=per_channel, window_applied=window_applied,
            window=(lo, hi), samples=samples, body_weight_n=weight,
            force_limit_n=limit_n, peak_force_n=f_peak,
            peak_torque_nm=m_peak, per_axis=per_axis,
            force_fraction_nonzero=f_frac, torque_fraction_nonzero=m_frac,
            any_fraction_nonzero=any_frac, source=support.source,
            error=("the column attested one half of the wrench and not the "
                   "other (force %s, torque %s), and the cell is decided by "
                   "both" % ("present" if f_peak is not None else "absent",
                             "present" if m_peak is not None else "absent")))
    if weight is None or limit_n is None:
        return SupportProfile(
            per_channel_attested=per_channel, window_applied=window_applied,
            window=(lo, hi), samples=samples, peak_force_n=f_peak,
            peak_torque_nm=m_peak, per_axis=per_axis,
            force_fraction_nonzero=f_frac, torque_fraction_nonzero=m_frac,
            any_fraction_nonzero=any_frac, source=support.source,
            error="the wrench was attested; the base's mass or the world's "
                  "gravity was not, so a peak in force units has no multiple "
                  "of body weight to be expressed as and no bound to be "
                  "compared against")

    within = bool(f_peak <= limit_n
                  and (torque_limit_nm is None
                       or m_peak <= float(torque_limit_nm)))
    return SupportProfile(
        attested=True, per_channel_attested=per_channel,
        window_applied=window_applied, window=(lo, hi), samples=samples,
        body_weight_n=weight, force_limit_n=limit_n,
        peak_force_n=f_peak, peak_force_x_weight=(f_peak / weight),
        force_fraction_nonzero=f_frac, peak_torque_nm=m_peak,
        torque_fraction_nonzero=m_frac, any_fraction_nonzero=any_frac,
        per_axis=per_axis, within_unsupported_bounds=within,
        source=support.source)


# --- what T4's core is handed -------------------------------------------------


@dataclass
class T4Evidence(T3Evidence):
    """One T4 run. **T3's evidence, unchanged, and no field added.**

    Stated as a subclass rather than an alias so a reader can see the claim
    being made: this tier replaces an *assertion*, not an evidence contract.
    Everything the two tiers grade -- the base pose with its orientation, the
    settled standing height, the gait contacts with the times the query ran,
    the applied wrench, the walking region, the mass, the gravity, the
    controller-load attestation -- is the same list, and a column that can
    serve T3 can serve T4.

    What T4 asks for **beyond** T3 is not a channel but a resolution: the
    wrench needs to arrive as a per-axis series on a clock, or the profile
    cannot say which channel held the robot up. That requirement lives in
    :data:`LADDER_REQUIRED_EVIDENCE_T4` and in the profile's own
    ``per_channel_attested``, where it degrades a cell's readability rather
    than failing a run.
    """


# The channel an assertion dies without. The four reused rows keep exactly the
# dependencies T3 gave them, because they ARE T3's rows; only the support row
# is T4's own, and it is the one that changed.
T4_CHANNEL_ASSERTIONS = {
    "base_pose": ("T4.1", "T4.2", "T4.3"),
    "base_orientation": ("T4.2",),
    "standing_height": ("T4.2",),
    "gait_contacts": ("T4.3", "T4.4"),
    "applied_support": ("T4.4",),
    "base_mass": ("T4.4",),
    "gravity": ("T4.4",),
    "controller_load": ("T4.5",),
    # Never a pass condition on either tier -- but on T4 its absence is
    # recorded as an incomplete measurement rather than shrugged at, because
    # §2 T4's build note 1 makes seeing the world's edge a requirement of the
    # recorder rather than a nicety.
    "arena": (),
}

LADDER_REQUIRED_EVIDENCE_T4 = (
    ("walking_surface", "nothing -- which bodies count as the ground is task "
                        "data, exactly as it is one rung down. An adapter "
                        "that could decide what counts as the ground could "
                        "label the thing holding the robot up as the floor."),
    ("base_pose", "the base's world position AND ORIENTATION over the run, on "
                  "the simulated clock. Not optional: the fall test bounds "
                  "roll and pitch at every sample."),
    ("standing_height", "the base's settled standing height, per robot, from "
                        "the t=0 inventory -- so the fall test is robot-"
                        "neutral instead of a constant somebody typed."),
    ("gait_contacts", "contacts naming a robot body and the body it is "
                      "standing on, timestamped, WITH the times the query "
                      "ran. A two-legged robot has one foot down for half of "
                      "every cycle, so the transitions are the difference "
                      "between a walk and a body being carried along."),
    ("applied_support", "every non-gravitational, non-contact wrench applied "
                        "to the base, as force (N) and torque (N.m), PER "
                        "AXIS and on the simulated clock. This is the tier's "
                        "own measurement and the thing that decides which of "
                        "the two cells a run is published in. A column that "
                        "can total the wrench but not stream it still counts "
                        "as attested and produces a weaker cell: it cannot "
                        "say whether the carry channel was idle while the "
                        "attitude channel ran, which is what the only "
                        "measurement we have of a real support rig showed. "
                        "AND IT MUST INCLUDE CONSTRAINT FORCES ON THE BASE. A "
                        "rig built as an explicit applied wrench is visible "
                        "here; one built as a weld, an equality constraint, a "
                        "kinematic base or a motion-captured attachment holds "
                        "the robot just as firmly and totals to ZERO unless "
                        "the column counts the constraint reaction. Nothing "
                        "downstream can detect the difference -- it reads what "
                        "the column attests -- so a column that omits them "
                        "publishes a held robot in the cell reserved for "
                        "numerically nothing."),
    ("base_mass", "the base's mass in kilograms and the world's gravity. The "
                  "cell boundary is 0.02 x m.g and the published figure is a "
                  "multiple of body weight; without m.g there is neither."),
    ("controller_load", "a positive attestation that the policy or controller "
                        "loaded, and the evidence line it was read from. An "
                        "exit code is not evidence."),
    ("arena", "the horizontal extent of the free walking region. Not a pass "
              "condition -- but on this tier a run whose sampler cannot see "
              "the edge of the world produces an INCOMPLETE cell, not a quiet "
              "one: every run of the only long humanoid walk this tree has "
              "measured was ended by geometry, and a grader blind to that "
              "would have published six clean walks as a gait that degraded."),
)


def unanswered_channels(ev):
    """``{channel: (assertion, ...)}`` for what this evidence cannot answer.

    Computed from the evidence rather than from which hook fired: a column
    that supplies a channel and fills it with nothing has the same gap as one
    that supplies no channel at all, and the cell's blocker is ours either way.
    """
    out = {}
    if ev is None:
        return {"everything": ("T4.1", "T4.2", "T4.3", "T4.4", "T4.5")}
    if not ev.base_pose.usable:
        out["base_pose"] = T4_CHANNEL_ASSERTIONS["base_pose"]
    elif not ev.base_pose.has_orientation:
        out["base_orientation"] = T4_CHANNEL_ASSERTIONS["base_orientation"]
    if not ev.standing.usable:
        out["standing_height"] = T4_CHANNEL_ASSERTIONS["standing_height"]
    if not (ev.gait.supported and ev.gait.has_sample_times):
        out["gait_contacts"] = T4_CHANNEL_ASSERTIONS["gait_contacts"]
    if not ev.support.usable:
        out["applied_support"] = T4_CHANNEL_ASSERTIONS["applied_support"]
    if ev.base_physics.mass_kg is None:
        out["base_mass"] = T4_CHANNEL_ASSERTIONS["base_mass"]
    if ev.world.gravity_acting is None:
        out["gravity"] = T4_CHANNEL_ASSERTIONS["gravity"]
    if not ev.controller.attested:
        out["controller_load"] = T4_CHANNEL_ASSERTIONS["controller_load"]
    if not ev.arena.usable:
        out["arena"] = T4_CHANNEL_ASSERTIONS["arena"]
    return out


def check_t4_evidence(ev):
    """Contract violations in one :class:`T4Evidence`, as strings.

    A new column runs this, plus ``agentbench.adapters.check_bundle``, against
    its own output before anyone spends a token on a T4 cell.
    """
    p = []
    if ev is None:
        return ["no T4 evidence at all"]
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
        p.append("applied support: not attested, so this run lands in neither "
                 "published cell -- it is recorded as support unverified and "
                 "excluded from comparison with any cell that has a total "
                 "(%s)" % (ev.support.error or "no reason given"))
    else:
        if not ev.support.source:
            p.append("applied support: attested with no source citation")
        if not ev.support.has_series:
            p.append("applied support: summarised rather than streamed, so "
                     "the profile cannot be measured per axis and cannot be "
                     "restricted to the walk. The only real support rig this "
                     "tree has measured was idle on its carry channel and "
                     "continuous on its attitude channel, and a summary "
                     "cannot say that")
    if ev.body_weight_n is None:
        p.append("body weight: the base's mass or the world's gravity is "
                 "unknown, so the cell boundary (0.02 x m.g) has no value and "
                 "the published figure has no multiple of body weight")
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
                 "stalled and the task's stated free run-up cannot be "
                 "checked. On this tier that makes the cell incomplete rather "
                 "than merely under-annotated (%s)"
                 % (ev.arena.error or "no reason given"))
    return p


__all__ = ["UNITS", "METHODS", "ATTESTED", "UNVERIFIED",
           "CELL_SUPPORTED", "CELL_UNSUPPORTED", "CELL_UNVERIFIED", "CELLS",
           "AXES", "SupportProfile", "EMPTY_SUPPORT_PROFILE",
           "support_profile", "T4Evidence", "T4_CHANNEL_ASSERTIONS",
           "LADDER_REQUIRED_EVIDENCE_T4", "unanswered_channels",
           "check_t4_evidence",
           # re-exported from the T3 channels, unchanged, so a T4 core has one
           # import and no shared piece can be re-declared here
           "TERMINATIONS", "TERMINATION_ARENA", "TERMINATION_CONTROLLER",
           "TERMINATION_FELL", "TERMINATION_TIME_LIMIT", "TERMINATION_UNKNOWN",
           "AppliedSupport", "EMPTY_APPLIED_SUPPORT", "ArenaBounds",
           "EMPTY_ARENA", "arena_from_bundle", "BodyPhysics",
           "EMPTY_BODY_PHYSICS", "body_physics_from_bundle", "ControllerLoad",
           "EMPTY_CONTROLLER_LOAD", "GaitContactObservation",
           "EMPTY_GAIT_CONTACTS", "GroundContact", "classify_contacts",
           "contact_states", "detrended_rms", "max_step_jump", "PoseSeries",
           "EMPTY_POSE_SERIES", "pose_series_from_bundle", "roll_pitch",
           "run_is_real", "StandingHeight", "EMPTY_STANDING_HEIGHT",
           "standing_height_from_bundle", "T3Evidence", "tilt_from_up",
           "WalkingSurface", "WorldPhysics", "EMPTY_WORLD_PHYSICS"]
