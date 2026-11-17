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

"""The channels the ladder needs and the frozen bundle has no field for.

``agentbench/graders/evidence.py`` is the cross-simulator contract and it is
**not edited here** -- it carries published rows. Everything below is additive
and lives in the ladder's own namespace; a ladder core consumes an
:class:`~agentbench.graders.evidence.EvidenceBundle` *plus* a
:class:`T1Evidence` / :class:`T2Evidence` wrapper, and never a modified bundle.

--------------------------------------------------------------------------
Gap 1: the commanded point is task data, and no adapter may supply it
--------------------------------------------------------------------------

T1 grades arrival at *a point a human named in a sentence*. That number is
the task's, and it must reach the core through the grader rather than through
a simulator: an adapter that could report the goal could report the goal it
observed the robot at. :class:`Waypoint` therefore travels beside the bundle,
carries the citation of the task file it was read from, and is never written
by adapter code.

--------------------------------------------------------------------------
Gap 2: nothing on either shipped column can answer "it touched the ground
while it was moving"
--------------------------------------------------------------------------

``ContactObservation`` is shaped generally enough -- ``ContactPair`` names two
bodies and flags each as robot or not -- but what the two implemented columns
actually *populate* is narrower than the shape, in three ways that each defeat
T1.4:

1. **Only robot-robot pairs survive.** The OmniSim recorder filters
   ``observe.list_contacts`` down to pairs whose two participants resolve to
   two DISTINCT robot subtrees and discards the rest, because the assertion it
   was built for (A1.3) is "no robot touched another robot". A robot-ground
   contact is exactly what that filter throws away.
2. **The window is wrong.** Contacts are sampled over the first
   ``contact_steps`` basic timesteps, *before* the settle phase and therefore
   before the robot has driven anywhere. T1.4 asks for a contact observed
   **during motion**.
3. **A resting body may report nothing at all.** A body idle longer than the
   scene's sleep threshold is disabled by the solver and generates no contact
   points, so "no contact" and "asleep" are the same reading unless something
   wakes it. This is why the vacuity counters are mandatory here rather than
   optional.

So the ladder defines :class:`SupportContactObservation`: contacts naming the
robot under test and the **non-robot body it is riding on**, timestamped in
simulated seconds so the core can ask whether the robot was moving when the
contact was seen. ``total_observed`` / ``distinct_named`` keep the same
meaning and the same discipline they have in the frozen bundle -- ``None``
means *"I could not count"* and makes the dependent clause report itself
vacuous, and it must never be spelled ``0``.

:func:`support_from_bundle` derives a best-effort observation from a plain
bundle for columns that have no ladder-side recorder yet. On both shipped
columns today it returns an **empty, unwitnessed** observation, which is the
honest answer: the channel does not exist there, T1.4 is red, and the cell's
blocker is ours (``scaffolding_defect_ours``) until a ladder recorder ships
for that column.

--------------------------------------------------------------------------
Gap 3: T2 asks five questions the frozen bundle cannot answer at all
--------------------------------------------------------------------------

T1 could be graded from a body's world position over time. T2 cannot, and the
shortfall is not a matter of degree:

1. **Orientation.** T2.3 measures the object's position *in the end-effector's
   frame*. ``Trajectory`` carries position and velocity and **no rotation** --
   so a rigidly held object whose wrist rotates traces a circle in the world
   frame, and grading the world-frame offset would read a perfect grasp as a
   0.1 m slip. :class:`PoseSeries` therefore carries a rotation matrix per
   sample, world-from-body, and a series without one makes T2.3's frame clause
   report itself vacuous rather than silently degrading to a translation.
2. **The object is not a robot.** Both shipped samplers enumerate *robots* and
   record those. The block is a plain body; nothing records it. So the object
   and the end-effector each arrive as their own :class:`PoseSeries`, keyed by
   the name the task declares.
3. **The container's geometry.** T2.1 is a containment test against a
   world-space AABB and a rim height. The AABB shape exists on ``Body``; the
   *rim* does not, and the two are not the same number the moment a container
   has a lid, a handle or a sloped lip -- so :class:`ContainerGeometry` carries
   the rim separately, says which rule produced it, and the core prints that
   rule beside the verdict.
4. **Mass.** T2.4 requires ``mass > 0``. ``Body`` carries ``dynamic`` (a mass
   model is attached) and no kilograms, and "a mass model is attached" is not
   "the mass is non-zero" -- a massless link is exactly the prop the assertion
   exists to catch. :class:`BodyPhysics` carries the kilograms;
   :class:`WorldPhysics` carries the gravity that has to be acting on them.
5. **The grip.** ``hold_mechanism`` is **recorded and never graded**
   (``capability-ladder-plan.md`` §2 T2: failing an attachment-based grasp
   would be writing the task around a technique this tree itself abandoned).
   :class:`GripObservation` is therefore a *reporting* channel: the core reads
   it into the row and no assertion may branch on it.

Every one of the five is a build task, not a simulator capability gap -- each
is reachable through the public API of every column in the roster. Which means
a red T2 assertion on a column with no ladder-side sampler is
``scaffolding_defect_ours`` and may never be published as a finding about
somebody else's product.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders.verdict import MIXED, Falsifier  # noqa: E402

# The units contract, identical to the frozen bundle's.
UNITS = ("metres, seconds, radians, kilograms; world frame; z is the axis the "
         "world's own gravity acts along")

# ``capability-ladder-plan.md`` §3.2: recorded in every T2 cell, never a pass
# condition. The order is the plan's.
HOLD_MECHANISMS = ("friction", "suction", "attachment", "unknown")


# --- helpers two tiers share -------------------------------------------------


def max_step_jump(xyz):
    """Largest single inter-sample 3-D displacement, metres.

    It lives here rather than in one tier's core because two tiers ask the
    same question of it: T1.3 asks whether a wheeled robot drove or was moved,
    and T2.4 asks whether the thing being carried is a body or a prop being
    flown. ``t1_core`` re-exports the name it already published.
    """
    xyz = np.asarray(xyz, dtype=float)
    if len(xyz) < 2:
        return 0.0
    d = np.diff(xyz, axis=0)
    return float(np.sqrt((d * d).sum(axis=1)).max())


def run_is_real(v, aid, b):
    """The "did this run actually happen" assertion, shared by every tier.

    ``capability-ladder-plan.md`` §2 states it once for T1 and then says
    *"as T1.5"* in every tier below, so it is written once here and both cores
    call it with their own assertion id. Returns whether the run is
    attributed -- an unattributed physics result is ``INVALID``, not ``FAIL``,
    and the caller owns that distinction because only it knows what else it
    measured.
    """
    proc = b.process
    attrib = b.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    errs = list(proc.error_lines) if proc else []
    finalized = bool(proc and proc.reached_finalize)
    driver_done = bool(proc and proc.driver_completed)
    ok = bool(proc is not None and finalized and not errs
              and not proc.timed_out and attrib is not None)
    v.add(aid, ok, "the run is real",
          measured={"exit code": (proc.exit_code if proc else None),
                    "error-class lines": len(errs),
                    "reached finalize": (proc.reached_finalize if proc
                                         else None),
                    "finalize evidence": (proc.finalize_evidence if proc
                                          else "none"),
                    "the driver reached its target simulated time":
                        driver_done,
                    "timed out": (bool(proc.timed_out) if proc else None),
                    "engine": (attrib.as_dict() if attrib else None)},
          threshold={"error-class lines": 0,
                     "reached finalize": True,
                     "engine": "attributed, with a citation"},
          detail="; ".join(errs[:3]),
          basis=MIXED,
          attested=([x for x in [
              ("error stream + exit code: %s"
               % (proc.log_source or "adapter-supplied")) if proc else None,
              ("engine attribution: %s" % attrib.source) if attrib else None,
          ] if x]),
          falsifiers=[
              Falsifier("reached finalize",
                        "a world that never finished building -- which an "
                        "exit code cannot see, because a failed load can "
                        "still exit 0 in seconds",
                        "the adapter answers 'did the world finalize' either "
                        "way, from an independent marker or from the "
                        "driver's own completion record",
                        bool(proc is not None
                             and proc.reached_finalize is not None),
                        detail=(proc.finalize_evidence if proc else "")),
              Falsifier("clean run",
                        "an error-class line or a timeout",
                        "the adapter captured the exit code AND the "
                        "simulator's own error stream",
                        bool(proc is not None and proc.exit_code is not None
                             and proc.log_available is not False)),
              Falsifier("engine attributed",
                        "a run with no engine verdict record, no explicit "
                        "engine pin and no documented engine invariant -- "
                        "which is INVALID, not FAIL",
                        "the adapter names the channel it read the "
                        "attribution from",
                        bool(attrib is None or attrib.source),
                        detail=(attrib.source if attrib else "unattributed")),
          ])
    return attrib is not None


# --- task data ---------------------------------------------------------------


@dataclass
class Waypoint:
    """The commanded point on the ground plane, in world metres.

    ``xy`` is the horizontal target. ``z`` is deliberately absent: the tier
    says *"a point on the floor"*, and asserting a height would grade the
    thickness of whatever surface the agent built rather than whether the
    robot got there.

    ``source`` is the citation -- the task file the number was read from. A
    waypoint with no source is a number somebody typed, and the core reports
    it as an absent witness rather than grading against it.
    """

    xy: tuple = (0.0, 0.0)
    label: str = "waypoint"
    source: str = ""

    @property
    def usable(self):
        try:
            x, y = (float(self.xy[0]), float(self.xy[1]))
        except (TypeError, ValueError, IndexError):
            return False
        return not (math.isnan(x) or math.isnan(y))

    def error_from(self, xy):
        """Horizontal distance from ``xy`` to the commanded point, metres."""
        return float(math.hypot(float(xy[0]) - float(self.xy[0]),
                                float(xy[1]) - float(self.xy[1])))

    def as_dict(self):
        return {"xy_m": [float(self.xy[0]), float(self.xy[1])],
                "label": self.label, "source": self.source}


# --- the support-contact channel ---------------------------------------------


@dataclass
class SupportContact:
    """One contact between the robot under test and the surface under it.

    ``t_s`` is the simulated time the contact was seen, on the **same clock as
    the trajectory** -- that is what lets the core ask "was it moving then?".
    ``None`` is allowed and makes the during-motion clause vacuous rather than
    failing it: an adapter that can see contacts but cannot time them has
    answered half the question honestly.
    """

    robot_body: Optional[str] = None
    surface_body: Optional[str] = None
    surface_is_robot: Optional[bool] = None
    point: Optional[tuple] = None
    t_s: Optional[float] = None
    step: Optional[int] = None

    @property
    def named_both(self):
        return bool(self.robot_body) and bool(self.surface_body)

    @property
    def distinct(self):
        return self.named_both and self.robot_body != self.surface_body

    @property
    def is_support(self):
        """A contact with a distinct body the adapter attests is not a robot.

        ``surface_is_robot is None`` is NOT accepted: an adapter that cannot
        say whether the other side is a robot cannot distinguish "resting on
        the ground" from "leaning on another robot", and guessing here is how
        a check that cannot fail gets written.
        """
        return self.distinct and self.surface_is_robot is False


@dataclass
class SupportContactObservation:
    """Robot-to-surface contacts over the motion window, plus their witnesses.

    ``total_observed`` / ``distinct_named`` carry exactly the meaning they
    carry in the frozen bundle: how much contact evidence EXISTED, not how
    much survived a filter. They are what makes "the robot never touched the
    ground" distinguishable from "the contact query names nothing".
    """

    pairs: list = field(default_factory=list)
    supported: bool = False
    total_observed: Optional[int] = None
    distinct_named: Optional[int] = None
    steps_sampled: int = 0
    window_s: Optional[float] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def support_pairs(self):
        return [p for p in self.pairs if p.is_support]

    @property
    def surface_bodies(self):
        out = []
        for p in self.support_pairs:
            if p.surface_body and p.surface_body not in out:
                out.append(p.surface_body)
        return out

    @property
    def can_name_a_support_pair(self):
        """Could a robot-to-surface contact have been *reported* at all?

        True only when the pipeline demonstrably produced a contact naming two
        distinct bodies. An empty list with no witness is indistinguishable
        from a broken query -- and from a body the solver put to sleep.
        """
        if not self.supported:
            return False
        if self.support_pairs:
            return True
        if self.distinct_named is None:
            return False
        return int(self.distinct_named) > 0

    @property
    def timed(self):
        """At least one pair carries a simulated timestamp."""
        return any(p.t_s is not None for p in self.pairs)

    @property
    def witness_detail(self):
        if not self.supported:
            return ("the adapter reports no contact query over the motion "
                    "window on this simulator"
                    + ("; %s" % self.error if self.error else ""))
        bits = ["contacts of any kind: %s"
                % ("unknown" if self.total_observed is None
                   else self.total_observed),
                "naming two distinct bodies: %s"
                % ("unknown" if self.distinct_named is None
                   else self.distinct_named),
                "steps sampled: %d" % self.steps_sampled]
        if self.error:
            bits.append("error: %s" % self.error)
        return "; ".join(bits)


EMPTY_SUPPORT = SupportContactObservation(
    error="no support-contact channel was supplied by the adapter")


def support_from_bundle(bundle):
    """Best-effort :class:`SupportContactObservation` from a plain bundle.

    The fallback for a column with no ladder-side recorder. It keeps the
    frozen bundle's vacuity counters (they are honestly counted there) and
    lifts any pair that already names one robot side and one non-robot side.

    On both columns shipped today that yields **no pairs**, because their
    contact channel is filtered to robot-robot and sampled before the motion
    window -- see the module docstring. That is a real, reportable gap and
    not a failure of the run being graded.
    """
    c = getattr(bundle, "contacts", None)
    if c is None:
        return SupportContactObservation(
            error="the bundle carries no contact channel at all")
    pairs = []
    for p in getattr(c, "pairs", ()) or ():
        a_robot, b_robot = bool(p.a_robot), bool(p.b_robot)
        if a_robot == b_robot:
            continue           # robot-robot, or neither: not support
        robot_side, surface_side = ((p.a, p.b) if a_robot else (p.b, p.a))
        pairs.append(SupportContact(robot_body=robot_side,
                                    surface_body=surface_side,
                                    surface_is_robot=False,
                                    point=p.point, step=p.step, t_s=None))
    return SupportContactObservation(
        pairs=pairs, supported=bool(getattr(c, "supported", False)),
        total_observed=getattr(c, "total_observed", None),
        distinct_named=getattr(c, "distinct_named", None),
        steps_sampled=int(getattr(c, "steps", 0) or 0),
        window_s=getattr(c, "window_s", None),
        source="derived from the run's general contact channel: %s"
               % (getattr(c, "source", "") or "unattributed"),
        error=getattr(c, "error", None))


# --- what a ladder core is handed --------------------------------------------


@dataclass
class T1Evidence:
    """One T1 run: the neutral bundle, unmodified, plus the ladder channels.

    Composition rather than mutation on purpose. ``bundle`` is exactly what
    ``agentbench.adapters.<sim>.build_bundle`` returned, byte for byte, so a
    reader can check the ladder against the published adapter contract without
    diffing a subclass.
    """

    bundle: object
    waypoint: Waypoint = field(default_factory=Waypoint)
    support: SupportContactObservation = field(
        default_factory=lambda: EMPTY_SUPPORT)
    robot_name: str = ""
    notes: list = field(default_factory=list)

    def provenance(self):
        base = {}
        try:
            base = dict(self.bundle.provenance())
        except Exception:  # noqa: BLE001  (evidence must never raise)
            base = {}
        base["waypoint"] = self.waypoint.source
        base["support_contacts"] = self.support.source
        return base


def check_t1_evidence(ev):
    """Contract violations in one :class:`T1Evidence`, as strings.

    The sibling of ``agentbench.adapters.check_bundle`` for the ladder's own
    channels. A new column runs BOTH against its own output before anyone
    spends a token on a ladder cell.
    """
    p = []
    if ev is None:
        return ["no T1 evidence at all"]
    if not ev.waypoint.usable:
        p.append("waypoint: no usable commanded point in metres")
    if not ev.waypoint.source:
        p.append("waypoint: no source citation -- a goal with no provenance "
                 "is a number somebody typed")
    s = ev.support
    if s.supported and s.total_observed is None and s.distinct_named is None:
        p.append("support contacts: no vacuity witness (total_observed / "
                 "distinct_named both None), so 'it never touched the "
                 "ground' cannot be told apart from a query that names "
                 "nothing -- the dependent clause will report vacuous")
    if s.pairs and not s.source:
        p.append("support contacts: pairs supplied with no source citation")
    for c in s.pairs:
        if c.surface_is_robot is None:
            p.append("support contacts: a pair does not say whether the "
                     "other side is a robot, so 'resting on the ground' and "
                     "'leaning on another robot' are the same reading")
            break
    return p


# --- T2's channels -----------------------------------------------------------
#
# Gap 3 in the module docstring, one dataclass per shortfall. Every one of
# them is SI, world frame, and absence is representable everywhere: ``None``
# means "I could not answer", never "the answer is zero".


@dataclass
class PoseSeries:
    """One body's world pose over time -- position **and orientation**.

    ``rot`` is (N, 3, 3) **world-from-body** rotation matrices, not a
    quaternion, an axis-angle or an Euler triple: converting whichever of
    those a simulator hands out is the adapter's job, and everything
    downstream of this dataclass is convention-free (the same choice
    ``CameraPose`` made for the camera channel).

    ``rot=None`` is legal and is the honest answer from a sampler that records
    positions only. It does not silently degrade the frame computation -- the
    dependent clause reports itself vacuous instead.
    """

    body: str = ""
    t: Optional[object] = None              # (N,) simulated seconds
    xyz: Optional[object] = None            # (N, 3) metres, world frame
    rot: Optional[object] = None            # (N, 3, 3) world-from-body
    source: str = ""
    error: Optional[str] = None

    @property
    def n_samples(self):
        return 0 if self.xyz is None else int(np.asarray(self.xyz).shape[0])

    @property
    def has_time(self):
        return (self.t is not None and self.xyz is not None
                and len(np.asarray(self.t)) == self.n_samples
                and self.n_samples >= 2)

    @property
    def has_orientation(self):
        if self.rot is None or self.xyz is None:
            return False
        r = np.asarray(self.rot, dtype=float)
        return r.ndim == 3 and r.shape == (self.n_samples, 3, 3)

    @property
    def usable(self):
        """Enough to measure anything at all from: poses on a clock."""
        return bool(self.has_time and self.n_samples >= 2)

    @property
    def dt_s(self):
        """Median sample interval, seconds, or ``None``."""
        if self.t is None or len(np.asarray(self.t)) < 2:
            return None
        d = np.diff(np.asarray(self.t, dtype=float))
        return float(np.median(d)) if len(d) else None

    @property
    def span_s(self):
        if self.t is None or not len(np.asarray(self.t)):
            return None
        t = np.asarray(self.t, dtype=float)
        return float(t[-1] - t[0])

    def max_step_jump(self):
        return max_step_jump(self.xyz) if self.xyz is not None else 0.0

    def align_to(self, t_ref):
        """``(indices, max_skew_s)`` -- this series' nearest sample per time.

        Two series that were not written by the same sampling loop are not on
        the same clock, and a frame computation across a skewed pair reads a
        held object as a moving one. The skew is returned rather than
        swallowed so the caller can refuse it.
        """
        if self.t is None or t_ref is None:
            return None, None
        ts = np.asarray(self.t, dtype=float)
        tr = np.asarray(t_ref, dtype=float)
        if not len(ts) or not len(tr):
            return None, None
        if len(ts) == 1:
            idx = np.zeros(len(tr), dtype=int)
            return idx, float(np.abs(ts[0] - tr).max())
        hi = np.clip(np.searchsorted(ts, tr), 1, len(ts) - 1)
        lo = hi - 1
        pick = np.where(np.abs(ts[lo] - tr) <= np.abs(ts[hi] - tr), lo, hi)
        return pick, float(np.abs(ts[pick] - tr).max())


EMPTY_POSE_SERIES = PoseSeries(
    error="no pose series for this body was supplied by the adapter")


def pose_series_from_bundle(bundle, body_id=None, name=None):
    """Best-effort :class:`PoseSeries` from a plain bundle's motion channel.

    The fallback for a column with no ladder-side sampler. It can only ever
    produce **position**: the frozen ``Trajectory`` carries no rotation, so the
    result reports ``has_orientation`` False and the frame clause that depends
    on it reports itself vacuous. That is the honest reading of what the
    published contract can answer, not a degradation to be hidden.
    """
    traj = getattr(bundle, "trajectory", None)
    if traj is None or traj.xyz is None:
        return PoseSeries(
            body=(name or body_id or ""),
            error=(getattr(bundle, "motion_error", None)
                   or (traj.error if traj is not None else None)
                   or "the bundle carries no motion channel at all"))
    row = None
    if body_id:
        row = traj.index_of(body_id)
    if row is None and name:
        inv = getattr(bundle, "roster", None)
        if inv is not None:
            j = inv.index_of_name(name)
            if j is not None and j < traj.n_bodies:
                row = j
    if row is None:
        return PoseSeries(
            body=(name or body_id or ""),
            error=("the motion channel has %d rows and none belongs to %r"
                   % (traj.n_bodies, name or body_id)))
    return PoseSeries(body=(name or body_id or ""), t=traj.t,
                      xyz=traj.xyz[row], rot=None,
                      source=("derived from the run's general motion "
                              "channel: %s; it carries position only"
                              % (traj.source or "unattributed")))


@dataclass
class ContainerGeometry:
    """The container, as the two numbers T2.1 is stated in.

    ``rim_z`` is the height of the **opening** -- the lip an object has to
    drop below to be in rather than on. It is separate from the AABB on
    purpose: for an open-topped box the two coincide, and for anything with a
    lid, a handle or a raised edge they do not, so a core that assumed
    ``aabb_max[2]`` would quietly grade "below the highest point of the
    container's bounding box", which is a different and easier claim.

    ``rim_z is None`` is allowed. The core then falls back to the AABB top,
    prints ``rim_rule`` saying so, and the fallback is **permissive** -- it can
    only make the clause easier to satisfy -- which is why the rule is
    published in the row rather than assumed.
    """

    body: str = ""
    aabb_min: Optional[tuple] = None
    aabb_max: Optional[tuple] = None
    rim_z: Optional[float] = None
    rim_rule: str = ""
    t_s: Optional[float] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def has_aabb(self):
        return self.aabb_min is not None and self.aabb_max is not None

    @property
    def span(self):
        """``(dx, dy, dz)`` of the box, metres, or ``None``."""
        if not self.has_aabb:
            return None
        return tuple(float(b) - float(a)
                     for a, b in zip(self.aabb_min, self.aabb_max))

    @property
    def usable(self):
        s = self.span
        return bool(s is not None and min(s) > 0.0)

    @property
    def rim_height(self):
        """The height an object's lowest point must be under, metres."""
        if self.rim_z is not None:
            return float(self.rim_z)
        return float(self.aabb_max[2]) if self.has_aabb else None

    @property
    def rim_attested(self):
        return self.rim_z is not None

    @property
    def rim_source(self):
        if self.rim_z is not None:
            return self.rim_rule or "supplied by the adapter, rule not stated"
        return ("no rim was attested, so the top of the container's own "
                "bounding box is used; that is the permissive reading and it "
                "over-states the rim for any container with a lid or a "
                "raised edge")

    def contains(self, xyz):
        """Is this world point inside the container's AABB?"""
        if not self.has_aabb:
            return None
        p = [float(v) for v in xyz]
        return all(float(self.aabb_min[i]) <= p[i] <= float(self.aabb_max[i])
                   for i in range(3))

    def as_dict(self):
        return {"body": self.body, "aabb_min_m": self.aabb_min,
                "aabb_max_m": self.aabb_max, "rim_z_m": self.rim_height,
                "rim_rule": self.rim_source, "t_s": self.t_s,
                "source": self.source, "error": self.error}


EMPTY_CONTAINER = ContainerGeometry(
    error="no container geometry was supplied by the adapter")


def container_from_bundle(bundle, name):
    """Best-effort :class:`ContainerGeometry` from the frozen t=0 inventory.

    Genuinely answerable from the published contract, with one shortfall that
    is stated rather than papered over: ``Body`` carries a world AABB and no
    rim, so the result has ``rim_z=None`` and the core falls back to the box
    top -- the permissive reading, printed in the row.
    """
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    body = None
    for b in (getattr(inv, "bodies", None) or []):
        if b.name == name:
            body = b
            break
    if body is None:
        return ContainerGeometry(
            body=(name or ""),
            error="no body named %r is in the frozen inventory" % name)
    if not body.has_aabb:
        return ContainerGeometry(
            body=(body.name or body.body_id or ""),
            error="the body named %r has no world-space bounding box" % name)
    return ContainerGeometry(
        body=(body.name or body.body_id or ""), aabb_min=body.aabb_min,
        aabb_max=body.aabb_max, rim_z=None,
        t_s=getattr(inv, "t_s", None),
        source=("the run's frozen t=0 inventory: %s. It carries a bounding "
                "box and no rim" % (getattr(inv, "source", "")
                                    or "unattributed")))


def object_aabb_from_bundle(bundle, name):
    """``(min_xyz, max_xyz)`` of the named body at t=0, or ``None``."""
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    for b in (getattr(inv, "bodies", None) or []):
        if b.name == name and b.has_aabb:
            return (b.aabb_min, b.aabb_max)
    return None


@dataclass
class SupportSurface:
    """One static thing an object could be resting on, as a world AABB.

    ``static`` is the adapter's attestation. ``None`` is not accepted as
    "static": a surface that might be moving is not a fixed datum to measure a
    lift against, and guessing here is how a clause that cannot fail gets
    written.
    """

    body: str = ""
    aabb_min: Optional[tuple] = None
    aabb_max: Optional[tuple] = None
    static: Optional[bool] = None
    source: str = ""

    @property
    def has_aabb(self):
        return self.aabb_min is not None and self.aabb_max is not None

    @property
    def usable(self):
        return bool(self.has_aabb and self.static is True)

    @property
    def top_z(self):
        return float(self.aabb_max[2]) if self.has_aabb else None


def support_surfaces_from_bundle(bundle):
    """Static, bounded, non-robot bodies from the frozen t=0 inventory.

    This one genuinely IS answerable from the published contract: ``Body``
    already carries a world AABB, ``dynamic`` and ``robot_class``. A body the
    adapter declares non-dynamic, not a robot and bounded is a fixed surface
    an object can rest on, and that is the whole requirement.
    """
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    out = []
    for b in (getattr(inv, "bodies", None) or []):
        if b.dynamic is not False or b.robot_class is True or not b.has_aabb:
            continue
        out.append(SupportSurface(
            body=(b.name or b.body_id or ""), aabb_min=b.aabb_min,
            aabb_max=b.aabb_max, static=True,
            source=("the run's frozen t=0 inventory: %s"
                    % (getattr(inv, "source", "") or "unattributed"))))
    return out


@dataclass
class BodyPhysics:
    """The kilograms the frozen ``Body`` has no field for.

    ``Body.dynamic`` says *a mass model is attached*. T2.4 asks for
    ``mass > 0``, and those are different questions: a massless link carries a
    mass model of zero, and it is exactly the prop the assertion exists to
    catch. So the number is carried, and ``None`` (unknown) makes the clause
    vacuous rather than passing it.
    """

    body: str = ""
    mass_kg: Optional[float] = None
    dynamic: Optional[bool] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def has_mass(self):
        return self.mass_kg is not None

    def as_dict(self):
        return {"body": self.body, "mass_kg": self.mass_kg,
                "dynamic": self.dynamic, "source": self.source,
                "error": self.error}


EMPTY_BODY_PHYSICS = BodyPhysics(
    error="no mass was supplied by the adapter")


def body_physics_from_bundle(bundle, name=None, body_id=None):
    """Best-effort :class:`BodyPhysics` from the frozen t=0 inventory.

    It can answer ``dynamic`` and **cannot answer mass** -- there is no
    kilogram anywhere in the published contract. The result therefore carries
    ``mass_kg=None`` and the mass clause reports itself vacuous, which is a
    gap in our scaffolding and never a property of the run.
    """
    inv = getattr(bundle, "t0", None) or getattr(bundle, "roster", None)
    body = None
    for b in (getattr(inv, "bodies", None) or []):
        if (name and b.name == name) or (body_id and b.body_id == body_id):
            body = b
            break
    if body is None:
        return BodyPhysics(body=(name or body_id or ""),
                           error=("no body named %r is in the frozen "
                                  "inventory" % (name or body_id)))
    return BodyPhysics(
        body=(body.name or body.body_id or ""), mass_kg=None,
        dynamic=body.dynamic,
        source=("the run's frozen t=0 inventory: %s. It answers 'a mass model "
                "is attached' and carries no kilograms"
                % (getattr(inv, "source", "") or "unattributed")),
        error="the published inventory contract has no mass field")


@dataclass
class WorldPhysics:
    """Is gravity acting, and how hard. One number, and its citation."""

    gravity_mps2: Optional[float] = None    # magnitude
    gravity_vec: Optional[tuple] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def gravity_acting(self):
        if self.gravity_mps2 is None:
            return None
        return bool(float(self.gravity_mps2) > 0.0)

    def as_dict(self):
        return {"gravity_mps2": self.gravity_mps2,
                "gravity_vec_mps2": self.gravity_vec,
                "source": self.source, "error": self.error}


EMPTY_WORLD_PHYSICS = WorldPhysics(
    error="no gravity attestation was supplied by the adapter")


@dataclass
class GripContact:
    """One contact between a part of the arm and the object it is moving."""

    holder_body: Optional[str] = None
    held_body: Optional[str] = None
    point: Optional[tuple] = None
    t_s: Optional[float] = None
    step: Optional[int] = None

    @property
    def named_both(self):
        return bool(self.holder_body) and bool(self.held_body)

    @property
    def distinct(self):
        return self.named_both and self.holder_body != self.held_body


@dataclass
class GripObservation:
    """How the object was held. **Recorded in every cell, graded in none.**

    ``capability-ladder-plan.md`` §2 T2 states the reasoning and it is worth
    repeating where the code lives: this tree's own shipped bin-picking
    success uses a suction end-effector after finger-gripper attempts topped
    out, and an earlier carry demo used a grasp-stabilisation weld. Failing an
    attachment-based grasp would be writing the task around a technique we
    ourselves abandoned -- the inverse of task-rigging and equally
    uninformative.

    So the core reads ``mechanism`` into the row and **no assertion may branch
    on it**. ``test_t2_core`` asserts that mechanically, by grading the same
    run under all four values and requiring identical assertions.
    """

    mechanism: str = "unknown"
    mechanism_source: str = ""
    contacts: list = field(default_factory=list)
    attachment: Optional[bool] = None       # a constraint binds the two bodies
    supported: bool = False
    total_observed: Optional[int] = None
    distinct_named: Optional[int] = None
    steps_sampled: int = 0
    window_s: Optional[float] = None
    source: str = ""
    error: Optional[str] = None

    @property
    def recorded_mechanism(self):
        """The value that goes in the cell, normalised into the four."""
        m = (self.mechanism or "").strip().lower()
        return m if m in HOLD_MECHANISMS else "unknown"

    @property
    def holder_bodies(self):
        out = []
        for c in self.contacts:
            if c.distinct and c.holder_body not in out:
                out.append(c.holder_body)
        return out

    @property
    def timed(self):
        return any(c.t_s is not None for c in self.contacts)

    def as_dict(self):
        return {"hold_mechanism": self.recorded_mechanism,
                "declared_as": self.mechanism,
                "mechanism_source": (self.mechanism_source
                                     or "not stated by the adapter"),
                "an attachment binds the two bodies": self.attachment,
                "contacts naming the arm and the object":
                    len([c for c in self.contacts if c.distinct]),
                "parts of the arm that touched it": self.holder_bodies,
                "contacts of any kind observed": self.total_observed,
                "contacts naming two distinct bodies": self.distinct_named,
                "steps sampled": self.steps_sampled,
                "source": self.source, "error": self.error}


EMPTY_GRIP = GripObservation(
    error="no grip channel was supplied by the adapter, so the mechanism is "
          "unknown rather than absent")


# --- task data, T2's ---------------------------------------------------------


@dataclass
class RoleNames:
    """Which body is the object, which is the container, which is the hand.

    Task data, exactly like :class:`Waypoint` and for the same reason: an
    adapter that could choose which body counts as "the object" could choose
    the body that happens to be in the container. The names come from the
    files the container ships, the citation says which file, and no adapter
    may write them.
    """

    object_name: str = ""
    end_effector_name: str = ""
    container_name: str = ""
    source: str = ""

    @property
    def usable(self):
        return bool(self.object_name and self.end_effector_name
                    and self.container_name)

    @property
    def missing(self):
        return [k for k, v in (("object", self.object_name),
                               ("end effector", self.end_effector_name),
                               ("container", self.container_name)) if not v]

    def as_dict(self):
        return {"object": self.object_name,
                "end_effector": self.end_effector_name,
                "container": self.container_name, "source": self.source}


# --- what T2's core is handed ------------------------------------------------


@dataclass
class T2Evidence:
    """One T2 run: the neutral bundle, unmodified, plus T2's channels.

    Composition rather than mutation, for the reason :class:`T1Evidence`
    gives: ``bundle`` is byte for byte what the column's own
    ``build_bundle`` returned, so a reader can check the ladder against the
    published adapter contract without diffing a subclass.
    """

    bundle: object
    roles: RoleNames = field(default_factory=RoleNames)
    object_pose: PoseSeries = field(
        default_factory=lambda: EMPTY_POSE_SERIES)
    end_effector: PoseSeries = field(
        default_factory=lambda: EMPTY_POSE_SERIES)
    container: ContainerGeometry = field(
        default_factory=lambda: EMPTY_CONTAINER)
    grip: GripObservation = field(default_factory=lambda: EMPTY_GRIP)
    object_physics: BodyPhysics = field(
        default_factory=lambda: EMPTY_BODY_PHYSICS)
    world: WorldPhysics = field(default_factory=lambda: EMPTY_WORLD_PHYSICS)
    support_surfaces: list = field(default_factory=list)
    object_aabb: Optional[tuple] = None      # (min_xyz, max_xyz) at t=0
    notes: list = field(default_factory=list)

    @property
    def object_half_extent(self):
        """``(hx, hy, hz)`` of the object at t=0, metres, or ``None``.

        The object's own size is what turns a centre into a lowest point, and
        without it two of T2's clauses lose their meaning. Absent, the core
        grades the CENTRE and says so -- it never invents a size.
        """
        if not self.object_aabb:
            return None
        lo, hi = self.object_aabb
        if lo is None or hi is None:
            return None
        return tuple(abs(float(b) - float(a)) / 2.0 for a, b in zip(lo, hi))

    def provenance(self):
        base = {}
        try:
            base = dict(self.bundle.provenance())
        except Exception:  # noqa: BLE001  (evidence must never raise)
            base = {}
        base["roles"] = self.roles.source
        base["object_pose"] = self.object_pose.source
        base["end_effector_pose"] = self.end_effector.source
        base["container_geometry"] = self.container.source
        base["grip"] = self.grip.source
        base["object_mass"] = self.object_physics.source
        base["gravity"] = self.world.source
        base["support_surfaces"] = (self.support_surfaces[0].source
                                    if self.support_surfaces else "")
        return base


def check_t2_evidence(ev):
    """Contract violations in one :class:`T2Evidence`, as strings.

    The sibling of :func:`check_t1_evidence`. A new column runs this, plus
    ``agentbench.adapters.check_bundle``, against its own output before anyone
    spends a token on a T2 cell.
    """
    p = []
    if ev is None:
        return ["no T2 evidence at all"]
    if not ev.roles.usable:
        p.append("roles: the task did not name the %s"
                 % " and the ".join(ev.roles.missing))
    if not ev.roles.source:
        p.append("roles: no source citation -- names with no provenance are "
                 "names somebody typed")
    for label, series in (("object", ev.object_pose),
                          ("end effector", ev.end_effector)):
        if not series.usable:
            p.append("%s pose: no usable series on a simulated clock (%s)"
                     % (label, series.error or "no reason given"))
        elif not series.source:
            p.append("%s pose: supplied with no source citation" % label)
    if ev.end_effector.usable and not ev.end_effector.has_orientation:
        p.append("end effector pose: position only, no orientation, so "
                 "'the object's position in the end-effector's frame' is not "
                 "measurable and the clause that needs it reports vacuous")
    if not ev.container.usable:
        p.append("container: no usable world-space bounding box (%s)"
                 % (ev.container.error or "no reason given"))
    elif not ev.container.rim_attested:
        p.append("container: no rim height attested, so the permissive "
                 "bounding-box top is used -- see the rim_source property")
    if not ev.object_physics.has_mass:
        p.append("object mass: unknown, so 'the object is a real body' cannot "
                 "be established (%s)"
                 % (ev.object_physics.error or "no reason given"))
    if ev.world.gravity_acting is None:
        p.append("gravity: no attestation, so 'gravity acting' cannot be "
                 "established (%s)" % (ev.world.error or "no reason given"))
    if not [s for s in ev.support_surfaces if s.usable]:
        p.append("support surfaces: none bounded and attested static, so "
                 "'lifted, not dragged' has no datum to measure against")
    if ev.object_aabb is None:
        p.append("object size: no bounding box at t=0, so a centre cannot be "
                 "turned into a lowest point and the clauses that need one "
                 "grade the centre instead")
    if not ev.grip.source and not ev.grip.error:
        p.append("grip: supplied with neither a source citation nor a reason "
                 "-- the mechanism is recorded in every cell and a cell may "
                 "not be quoted without it")
    return p


__all__ = ["UNITS", "HOLD_MECHANISMS", "max_step_jump", "run_is_real",
           "Waypoint", "SupportContact", "SupportContactObservation",
           "EMPTY_SUPPORT", "T1Evidence", "support_from_bundle",
           "check_t1_evidence",
           "PoseSeries", "EMPTY_POSE_SERIES", "pose_series_from_bundle",
           "ContainerGeometry", "EMPTY_CONTAINER", "container_from_bundle",
           "object_aabb_from_bundle", "SupportSurface",
           "support_surfaces_from_bundle", "BodyPhysics",
           "EMPTY_BODY_PHYSICS", "body_physics_from_bundle", "WorldPhysics",
           "EMPTY_WORLD_PHYSICS", "GripContact", "GripObservation",
           "EMPTY_GRIP", "RoleNames", "T2Evidence", "check_t2_evidence"]
