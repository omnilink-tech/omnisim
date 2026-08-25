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

"""The **evidence bundle** -- the only thing a grader core is allowed to read.

SPEC 6.2.6 is the rule this module implements: *"Sim-agnostic graders, per-sim
adapters. Every assertion is stated in physical units (metres, seconds,
contacts, exit codes). A grader that reads an OmniSim-shaped JSON response is a
bug."*

So each grader is split in two:

    adapters/<sim>/  ->  EvidenceBundle  ->  graders/<task>_core.py  ->  Verdict
    (sim-specific)       (this module)       (physical units only)

A **core** consumes nothing but the dataclasses below: per-body pose/velocity
time series in SI units, a body inventory with names and world-space AABBs,
contact pairs, process/exit facts, and one engine attribution. It cannot ask
what a ``URDFRobot`` is, cannot read a ``.newton.json`` sidecar, and cannot
grep a ``.wbt``. Those three things are exactly what made A1 unrunnable on any
other simulator, and all three now live behind this boundary.

Three fields deserve their own explanation, because they are where the honesty
of a cross-sim comparison actually lives:

``Body.robot_class`` / ``IdentityRule``
    "Is this a Husky?" has no sim-neutral answer -- OmniSim resolves URDF
    imports into plain ``Robot`` nodes and keeps the ``husky.urdf`` reference
    only in the world text, upstream Webots instantiates a ``Husky`` PROTO,
    Gazebo spawns an SDF model with a ``name``. So the **adapter** answers it,
    in its own terms, and publishes the rule it used (``IdentityRule``). The
    *count*, the *distinctness* and every metre and second stay in the core.

``ContactObservation.total_observed`` / ``.distinct_named``
    The vacuity witnesses. A1.3's "no robot-robot contact" assertion could not
    fail for weeks: the underlying node id returned the queried solid's own id,
    so every pair was ``(id, id)`` and the robot-robot filter matched nothing,
    forever. An empty result therefore proved nothing. These two counters are
    the evidence that the contact pipeline is *capable* of producing the
    counter-example -- see ``graders.verdict.Falsifier``.

``EvidenceBundle.labels``
    Presentation only, and it carries no logic. The v0 rows already published
    for OmniSim spell some measured keys in OmniSim's own words ("husky.urdf
    refs in the artifact"). The core writes neutral defaults and lets the
    adapter override the *rendered wording* so a v0 row stays byte-comparable
    across this refactor. Never branch on a label.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# The units contract, quoted in one place so no adapter has to guess.
UNITS = ("metres, seconds, radians, kilograms; world frame; z is the axis the "
         "world's own gravity acts along")


# --- structure ---------------------------------------------------------------


@dataclass
class Body:
    """One body in the scene, at one instant, in SI units.

    ``body_id`` must be unique within the run and stable across the queries a
    single adapter makes; it is what the core counts for distinctness. Anything
    the core cannot express in SI or in a count belongs in ``kind`` /
    ``identity_evidence``, which are **recorded and never asserted on**.
    """

    body_id: str
    name: str = ""
    kind: str = ""                          # the sim's own type name
    position: Optional[tuple] = None        # (x, y, z) m, world frame
    aabb_min: Optional[tuple] = None        # world-space AABB, m
    aabb_max: Optional[tuple] = None
    n_joints: Optional[int] = None
    behaviour: Optional[str] = None         # the EFFECTIVE behaviour, or None
    behaviour_declared: Optional[str] = None  # what the scene declares, verbatim
    dynamic: Optional[bool] = None          # a mass model is attached
    robot_class: Optional[bool] = None      # the adapter's IdentityRule answer
    identity_evidence: str = ""             # why, in the adapter's own terms
    member_of: Optional[str] = None         # owning body id, for a LINK body

    # -- derived -----------------------------------------------------------
    @property
    def has_aabb(self):
        return self.aabb_min is not None and self.aabb_max is not None

    @property
    def aabb(self):
        """``(min_xyz, max_xyz)`` or None."""
        return (self.aabb_min, self.aabb_max) if self.has_aabb else None

    @property
    def aabb_center(self):
        if not self.has_aabb:
            return None
        return tuple((float(a) + float(b)) / 2.0
                     for a, b in zip(self.aabb_min, self.aabb_max))

    @property
    def top_z(self):
        return None if not self.has_aabb else float(self.aabb_max[2])

    @property
    def height(self):
        return (None if not self.has_aabb
                else float(self.aabb_max[2]) - float(self.aabb_min[2]))

    @property
    def controlled(self):
        """A behaviour is genuinely attached.

        Two fields, deliberately: ``behaviour_declared`` is the string the
        scene carries (which may be a sim's "no controller" sentinel -- OmniSim
        and Webots both write ``"<none>"``), while ``behaviour`` is what the
        adapter says will actually run. A body can declare a behaviour field
        and still be uncontrolled, and a grader that conflates the two reports
        ten controlled robots in a scene where nothing moves.
        """
        return bool(self.behaviour)


@dataclass
class BodyInventory:
    """What bodies exist, from one query, with that query's provenance.

    ``frozen`` matters: an inventory read while the world free-runs cannot
    answer any "at t=0" question, and a core that is handed one must say so
    rather than quietly grading a moved scene (measured on OmniSim's harness: a
    ``GET /robots`` on a 10-Husky world cost 22 s of simulated driving).
    """

    bodies: list = field(default_factory=list)
    t_s: Optional[float] = None
    frozen: Optional[bool] = None
    source: str = ""                        # citation, in the adapter's terms
    error: Optional[str] = None

    def __iter__(self):
        return iter(self.bodies)

    def __len__(self):
        return len(self.bodies)

    @property
    def robots(self):
        """Bodies the adapter attests match the task's identity rule.

        ⚠ A LINK of a robot is never one of these. ``member_of`` is what says
        "part of robot X"; ``robot_class`` answers "is this body the robot the
        task named", and a forearm is not. Several cores count this list
        (R1.2's "exactly one robot", R2.2's "one arm with >= six joints"), so an
        adapter that marked links robot-class would report a six-link arm as six
        arms and fail a correct world.
        """
        return [b for b in self.bodies if b.robot_class]

    @property
    def independent(self):
        """Bodies that are not part of another -- the scene's own objects."""
        return [b for b in self.bodies if not b.member_of]

    def links_of(self, body_id):
        """The bodies the adapter attests are parts of ``body_id``."""
        return [b for b in self.bodies if b.member_of == body_id]

    @property
    def names(self):
        return [b.name or "" for b in self.bodies]

    def by_name(self, name):
        for b in self.bodies:
            if b.name == name:
                return b
        return None

    def index_of_name(self, name):
        for i, b in enumerate(self.bodies):
            if b.name == name:
                return i
        return None


EMPTY_INVENTORY = BodyInventory(source="not supplied by the adapter")


# --- contacts ----------------------------------------------------------------


@dataclass
class ContactPair:
    """One contact, named in body ids where the sim can name both sides."""

    a: Optional[str] = None
    b: Optional[str] = None
    a_robot: bool = False
    b_robot: bool = False
    point: Optional[tuple] = None           # world-space, m
    step: Optional[int] = None

    @property
    def named_both(self):
        return bool(self.a) and bool(self.b)

    @property
    def distinct(self):
        return self.named_both and self.a != self.b

    @property
    def robot_robot(self):
        return bool(self.a_robot and self.b_robot and self.distinct)


@dataclass
class ContactObservation:
    """Contacts over a window, **plus the evidence that a hit was possible.**

    ``total_observed`` and ``distinct_named`` are the vacuity witnesses. Leave
    them ``None`` if the adapter genuinely does not know: ``None`` makes the
    dependent assertion report *"vacuous: evidence absent"*, which is the
    honest answer and the one that would have caught the ``(id, id)`` bug on
    day one. Setting them to 0 when you did not count is a lie the grader will
    believe.
    """

    pairs: list = field(default_factory=list)
    steps: int = 0
    window_s: Optional[float] = None
    supported: bool = False                 # the sim exposes a contact query
    total_observed: Optional[int] = None    # contacts of ANY kind seen
    distinct_named: Optional[int] = None     # ...naming two distinct bodies
    source: str = ""
    error: Optional[str] = None

    @property
    def robot_robot_pairs(self):
        return [p for p in self.pairs if p.robot_robot]

    @property
    def can_name_a_robot_pair(self):
        """Could a robot-robot contact have been *reported* at all?

        True only when the pipeline demonstrably produced at least one contact
        naming two distinct bodies. An empty contact list with no witness is
        indistinguishable from a broken query.
        """
        if not self.supported:
            return False
        if self.robot_robot_pairs:
            return True
        if self.distinct_named is None:
            return False
        return int(self.distinct_named) > 0

    @property
    def witness_detail(self):
        if not self.supported:
            return "the adapter reports no contact query on this simulator"
        bits = ["contacts of any kind: %s" % ("unknown"
                                              if self.total_observed is None
                                              else self.total_observed),
                "naming two distinct bodies: %s"
                % ("unknown" if self.distinct_named is None
                   else self.distinct_named)]
        if self.error:
            bits.append("error: %s" % self.error)
        return "; ".join(bits)


# --- motion ------------------------------------------------------------------


#: The track kinds a row of :class:`Trajectory` may have. Recorded, never
#: asserted on -- a core states its questions in metres, and a "kind" is a
#: statement about the scene graph. It exists so a core can ASK for the right
#: rows (R2 wants the arm's links; R3 wants one named body) rather than having
#: to infer membership from geometry alone.
TRACK_KINDS = ("robot", "link", "solid")


@dataclass
class Trajectory:
    """Per-body pose (and optionally velocity) time series, SI units.

    ``t`` is (N,) simulated seconds, monotone increasing, starting at the first
    recorded sample. ``xyz`` is (n_bodies, N, 3) metres in the world frame;
    ``vel``, when supplied, is (n_bodies, N, 3) m/s. ``body_ids`` indexes rows
    of ``xyz`` onto ``BodyInventory`` entries.

    **Index invariant an adapter must honour:** row *i* of ``xyz`` is
    ``body_ids[i]``, and a core that looks a body up by name in a **frozen**
    inventory (``BodyInventory.frozen is True``) and then indexes ``xyz`` with
    that position must get the same body. Free-running inventories (a live
    query while the world steps) carry no such guarantee, which is why
    ``frozen`` is a field and not a comment.

    **The three descriptive row vectors, and why they exist.** ``kinds``,
    ``parents`` and ``names`` are parallel to ``body_ids`` and may be empty
    (an adapter that has nothing to say leaves them so). They were added when
    the recorders stopped sampling only top-level ``Robot`` nodes: a run may
    now carry a row for a body INSIDE a robot (``kinds[i] == "link"``,
    ``parents[i] ==`` the owning body's id) and for a named non-robot body
    (``"solid"``). That pair IS the sim-neutral identity of a link -- "link of
    robot X" -- and it deliberately is not a name: R1 shipped a grader keyed on
    the published ``OBSTACLE_n`` names, the first real agent called them
    "crate A", and a correct world scored zero. Nothing in R2's prompt suggests
    a name for a tool body either.

    **The rows a robot-counting core must use is still the ROSTER, not this.**
    A link is not a robot and is never in the roster; ``n_bodies`` counts every
    row. ``n_top_level`` is the count that excludes links, for a core that
    wants "how many independent bodies" rather than "how many tracks".
    """

    body_ids: list = field(default_factory=list)
    t: Optional[np.ndarray] = None
    xyz: Optional[np.ndarray] = None
    vel: Optional[np.ndarray] = None
    dt_s: Optional[float] = None
    recorded_s: Optional[float] = None
    complete: bool = False
    source: str = ""
    error: Optional[str] = None
    kinds: list = field(default_factory=list)     # per row, see TRACK_KINDS
    parents: list = field(default_factory=list)   # per row, owning body id
    names: list = field(default_factory=list)     # per row, the scene's name

    @property
    def n_bodies(self):
        return 0 if self.xyz is None else int(self.xyz.shape[0])

    @property
    def n_samples(self):
        return 0 if self.xyz is None else int(self.xyz.shape[1])

    def xy(self, i):
        return self.xyz[i][:, :2]

    def z(self, i):
        return self.xyz[i][:, 2]

    def index_of(self, body_id):
        try:
            return self.body_ids.index(body_id)
        except ValueError:
            return None

    # -- the descriptive vectors -------------------------------------------
    def kind_of(self, i):
        """``kinds[i]`` or ``""`` when the adapter did not classify the rows.

        ``""`` is "unclassified", never "robot": an adapter that says nothing
        must not have a guess attributed to it.
        """
        return self.kinds[i] if i < len(self.kinds) else ""

    def parent_of(self, i):
        return self.parents[i] if i < len(self.parents) else None

    def name_of(self, i):
        return self.names[i] if i < len(self.names) else ""

    def index_of_name(self, name):
        """The FIRST row whose scene name is ``name``, or None.

        How a core asks for one specific body -- "give me the cube" -- when the
        task ships that body and its name is therefore the fixture's, not the
        agent's. A core must not use this for anything the AGENT named; that is
        the R1 mistake, and geometry is the answer there.
        """
        if not name:
            return None
        for i, nm in enumerate(self.names):
            if nm == name:
                return i
        return None

    def rows_of_kind(self, kind):
        """Row indices whose kind is ``kind``. ``[]`` when unclassified."""
        return [i for i, k in enumerate(self.kinds) if k == kind]

    def links_of(self, parent_body_id):
        """Row indices of the link bodies belonging to one body.

        The membership channel R2's end-effector selection lacks: it currently
        approximates "belongs to this arm" with a reach gate, and this is the
        structural answer when the adapter supplies one.
        """
        return [i for i, k in enumerate(self.kinds)
                if k == "link" and self.parent_of(i) == parent_body_id]

    @property
    def n_top_level(self):
        """Rows that are not links -- i.e. independent bodies in the scene.

        Equals ``n_bodies`` for every unclassified recording, so a core that
        switches to it does not change what an older run measured.
        """
        if not self.kinds:
            return self.n_bodies
        return sum(1 for k in self.kinds if k != "link")

    def tail(self, i, window_s):
        """The z track of body ``i`` over the last ``window_s`` seconds."""
        z = self.z(i)
        if self.t is None or not len(self.t):
            return z
        return z[self.t >= (float(self.t[-1]) - float(window_s))]


# --- camera ------------------------------------------------------------------
#
# Lifted verbatim from graders/b2_core.py at integration, exactly as that
# module's docstring planned: declared there so B2 could be written and tested
# first, shaped to move here unchanged, with EvidenceBundle growing one
# optional ``view`` field.


def _cam_vec(v):
    if v is None:
        return None
    try:
        out = tuple(float(x) for x in v)
    except (TypeError, ValueError):
        return None
    return out if len(out) == 3 else None


def _cam_norm(v):
    return math.sqrt(sum(x * x for x in v))


def _cam_unit(v):
    v = _cam_vec(v)
    if v is None:
        return None
    n = _cam_norm(v)
    return None if n < 1e-12 else tuple(x / n for x in v)


@dataclass
class CameraPose:
    """Where the camera is, where it looks, and how wide it sees. SI units.

    ``forward`` and ``up`` are **world-frame unit vectors**, not a rotation in
    any one simulator's parameterisation: converting an axis-angle, a
    quaternion, a matrix or an Euler triple into two world vectors is the
    adapter's job, and so is knowing which local axis that simulator calls
    "forward". Everything downstream of this dataclass is convention-free.

    ``fov_h_rad`` / ``fov_v_rad`` are the FULL horizontal and vertical angles.
    Either may be ``None``; see :meth:`half_angles` for how the missing one is
    resolved and what that costs.
    """

    position: Optional[tuple] = None
    forward: Optional[tuple] = None
    up: Optional[tuple] = None
    fov_h_rad: Optional[float] = None
    fov_v_rad: Optional[float] = None
    aspect: Optional[float] = None
    source: str = ""

    @property
    def usable(self):
        """A pose complete enough to compute a frustum test from."""
        if _cam_vec(self.position) is None or _cam_unit(self.forward) is None:
            return False
        h, v, _how = self.half_angles()
        return h is not None and v is not None

    def half_angles(self):
        """``(half_h_rad, half_v_rad, how)`` -- the frustum half-angles.

        A camera that reports one angle and an aspect ratio gets the other by
        the pinhole relation ``tan(v/2) = tan(h/2) / aspect``. A camera that
        reports one angle and no aspect gets a **square** frustum, which is the
        conservative reading on the unknown axis; ``how`` says so, and the
        verdict prints it.
        """
        h, v, a = self.fov_h_rad, self.fov_v_rad, self.aspect
        how = "both angles reported"
        if h is not None and v is None:
            if a:
                v = 2.0 * math.atan(math.tan(0.5 * float(h)) / float(a))
                how = "vertical angle derived from the aspect ratio"
            else:
                v, how = h, ("vertical angle unknown and no aspect ratio; "
                             "treated as equal to the horizontal one")
        elif v is not None and h is None:
            if a:
                h = 2.0 * math.atan(math.tan(0.5 * float(v)) * float(a))
                how = "horizontal angle derived from the aspect ratio"
            else:
                h, how = v, ("horizontal angle unknown and no aspect ratio; "
                             "treated as equal to the vertical one")
        if h is None or v is None:
            return None, None, "no field of view reported"
        return 0.5 * float(h), 0.5 * float(v), how


@dataclass
class ViewEvidence:
    """The camera channel for one run: where it started, where it ended.

    ``initial`` is the pose the task shipped and ``final`` is the pose at the
    end of the run, **however the agent set it** -- by editing the scene file,
    by pushing it over a live session, by scripting the viewer. The core does
    not care which, and must not: the two routes cost different amounts on
    different simulators and pricing that difference is the campaign's job, not
    the grader's.

    ``artifact_parsed`` is three-valued on purpose. ``True``: the agent's edited
    scene was read back successfully. ``False``: it was not -- the agent broke
    the file, which is an agent failure and fails B2.1. ``None``: no scene edit
    was involved, so the question does not arise.
    """

    final: Optional[CameraPose] = None
    initial: Optional[CameraPose] = None
    artifact_parsed: Optional[bool] = None
    artifact: Optional[str] = None
    source: str = ""
    error: Optional[str] = None


EMPTY_VIEW = ViewEvidence(error="not supplied by the adapter")


def check_view(view):
    """Contract violations in one :class:`ViewEvidence`, as strings.

    The sibling of ``adapters.check_bundle`` for the camera channel; the
    bundle self-check delegates to this whenever ``bundle.view`` is present.
    A new adapter runs this against its own output before anyone spends
    tokens on a cell.
    """
    p = []
    if view is None:
        return ["no camera evidence at all: B2 cannot be graded"]
    if view.error:
        p.append("camera evidence reports an error: %s" % view.error)
    for name, cam in (("final", view.final), ("initial", view.initial)):
        if cam is None:
            p.append("%s camera pose is absent" % name)
            continue
        if not cam.source:
            p.append("%s camera pose has no source citation" % name)
        if _cam_vec(cam.position) is None:
            p.append("%s camera pose has no position in metres" % name)
        if _cam_unit(cam.forward) is None:
            p.append("%s camera pose has no unit forward axis" % name)
        if cam.half_angles()[0] is None:
            p.append("%s camera pose reports no field of view" % name)
        if cam.up is not None and _cam_unit(cam.up) is None:
            p.append("%s camera pose has an up axis that is not a vector"
                     % name)
    return p


# --- process / attribution ---------------------------------------------------


@dataclass
class ProcessFacts:
    """What the *process* did, as opposed to what the physics did.

    ``driver_completed`` is the grader's own recorder/driver attesting that it
    reached the requested simulated time and asked the simulator to quit --
    backend-agnostic evidence that the world was built, finalized and stepped.
    An exit code alone is never accepted: on OmniSim a failed load still exits
    0 in 2-4 s (SPEC A1.9).
    """

    exit_code: Optional[int] = None
    timed_out: bool = False
    error_lines: list = field(default_factory=list)
    log_available: Optional[bool] = None    # was the sim's error stream read?
    log_source: str = ""                    # ...from where (the citation)
    behaviour_starts: dict = field(default_factory=dict)
    start_source: str = ""
    driver_completed: Optional[bool] = None
    reached_finalize: Optional[bool] = None
    finalize_evidence: str = ""
    wall_s: Optional[float] = None
    attempts_used: int = 1

    @property
    def clean(self):
        return bool(self.exit_code == 0 and not self.error_lines
                    and not self.timed_out)


@dataclass
class EngineAttribution:
    """Which physics engine/solver actually drove the run, and how we know.

    SPEC A1.10: not a pass/fail gate -- either engine passes -- but a run with
    **no** attribution is ``INVALID``, because an unattributed physics result
    is not a result. ``source`` is the citation and is mandatory: a sidecar
    verdict, an explicit config pin, or a documented structural invariant of
    that simulator ("upstream Webots R2025a ships ODE only, <cite>").
    """

    backend: str
    solver: Optional[str] = None
    degraded: bool = False
    source: str = ""
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        out = {"backend": self.backend, "solver": self.solver,
               "degraded": bool(self.degraded)}
        out.update(self.extra)
        out["source"] = self.source
        return out


# --- identity ----------------------------------------------------------------


@dataclass
class IdentityRule:
    """"Is this body the robot the task asked for?", answered per simulator.

    The task states the requirement; the adapter states how it decided and
    publishes both rules with the scaffolding (SPEC 6.2.6). ``declared_count``
    is the same question asked of the *artifact* rather than the loaded scene:
    on OmniSim, ``husky.urdf`` url references (the engine expands URDF imports
    at parse time, so the reference survives only in the file); on upstream
    Webots, ``Husky`` PROTO instantiations. ``None`` means the adapter cannot
    answer -- which the core reports as an absent witness, never as a pass.
    """

    label: str = ""
    requirement: str = ""
    scene_rule: str = ""
    declaration_rule: str = ""
    declared_count: Optional[int] = None


# --- the bundle --------------------------------------------------------------


@dataclass
class EvidenceBundle:
    """Everything a core may believe about one run, in physical units.

    Absence is always representable and always distinguishable from zero:
    every optional field is ``None`` when the adapter could not answer, and the
    reason goes in ``notes`` or in the matching ``*_error``.
    """

    task: str
    sim: str = ""
    adapter: str = ""
    artifact: Optional[str] = None          # what the agent produced, or None
    identity: Optional[IdentityRule] = None
    roster: BodyInventory = field(default_factory=BodyInventory)
    t0: BodyInventory = field(default_factory=BodyInventory)
    contacts: Optional[ContactObservation] = None
    trajectory: Optional[Trajectory] = None
    view: Optional[ViewEvidence] = None
    motion_error: Optional[str] = None
    process: Optional[ProcessFacts] = None
    attribution: Optional[EngineAttribution] = None
    live_load_ok: Optional[bool] = None     # a live session loaded the artifact
    notes: list = field(default_factory=list)
    labels: dict = field(default_factory=dict)
    adapter_measurements: dict = field(default_factory=dict)

    # -- presentation ------------------------------------------------------
    def lbl(self, key, default):
        """The rendered wording for a measured/threshold key. No logic here."""
        return self.labels.get(key, default)

    # -- provenance --------------------------------------------------------
    def provenance(self):
        """Where every part of this bundle came from, for the report."""
        return {
            "sim": self.sim, "adapter": self.adapter,
            "roster": self.roster.source, "t0": self.t0.source,
            "contacts": self.contacts.source if self.contacts else None,
            "motion": self.trajectory.source if self.trajectory else None,
            "view": self.view.source if self.view else None,
            "attribution": (self.attribution.source
                            if self.attribution else None),
        }

    def measurement_slot(self, name):
        """The adapter's own payload for one phase -- recorded, never graded."""
        return self.adapter_measurements.get(name)
