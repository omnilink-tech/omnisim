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

"""T4's negative fixtures -- the red-evidence rule, made executable.

``capability-ladder-plan.md`` §5c.2, binding and standing: *"no assertion
enters a ladder cell until it has been observed FAILING on a deliberately
wrong artifact, with that negative fixture named in the assertion's record. A
green assertion is not evidence that the assertion works ... A ``null`` agent
turning every assertion red does not satisfy this rule."*

Every fixture below is the :func:`oracle` with **one** thing wrong, and
:data:`FIXTURE_ASSERTION_MAP` records which single assertion it must turn red,
which clauses it must leave vacuous, **and which of the tier's two cells it
must land in**. A fixture that reddens two assertions, or the wrong one, or
publishes in the wrong cell, is a test failure.

The graders are shared with the rung below; **this synthetic scene is not**,
and deliberately. T4 grades a two-legged robot, its support profile is the
whole point of the tier, and a fixture module that tried to serve both robots
would be a scene with an if-statement in it. What is shared is every line of
grading logic (:mod:`ladder.graders.t4_core` calls the other core's row
builders directly), which is where sharing pays and where drift would hurt.

Seven fixtures exist to prove something is **not** graded, or is graded
*loudly*, rather than to prove something is:

* :func:`g1_shaped_support` PASSes in the **``T4-supported``** cell carrying
  the real measured support profile of this tree's own flagship
  (``docs/developer/g1-endurance-2026-08-01.md`` §4): the carrying channel at
  **0.00 N for 0 %** of the window while the attitude channel runs at a
  **69.2 N.m peak for 100 %** of it. §2 T4 says a supported run is a different
  cell and not a failure, so the evidence that it passes -- with its figures
  printed inside the cell -- has to be as executable as any red here.
* :func:`carried_outright` PASSes in the same cell under **2.09 x body weight
  and 348 N.m for 100 % of the window**, which is the plan's own worked
  example rendered from measurement rather than typed. It is the fixture that
  makes the tier's most counter-intuitive rule impossible to misread: this run
  is *published*, with the number, not failed.
* :func:`oracle` and it are the two-cell mechanism: the same recording shape,
  the same five rows, two different cells.
* :func:`support_not_attested` PASSes in **neither** cell, flagged
  ``T4-support-unverified`` and excluded from comparison. Failing it would
  publish our own missing channel as somebody else's capability gap.
* :func:`summarised_support` PASSes with the per-channel clause vacuous: a
  column that can total the wrench but not stream it per axis is in a real and
  weaker state, and the row says which.
* :func:`no_arena_channel` PASSes with ``arena_attestation: unverified`` and
  the cell marked incomplete -- §2 T4's build note 1 makes seeing the edge of
  the world a requirement, and a column that cannot is not quietly graded.
* :func:`no_attested_standing_height` PASSes with the fall test's datum clause
  vacuous: the height came from the run's own first sample, which a robot that
  never stood up would have supplied from the floor.

And one fixture is a **finding about this tree**, not about the grader:

* :func:`flagship_measured_speed` is red on **T4.3**, and the numbers in it
  are not invented. The shipped flagship crosses 10.0 m at t = 82.88 s, which
  is **0.1207 m/s** made good, against the tier's inherited **0.15 m/s** floor.
  So the artifact whose distance the plan measured before the freeze -- and
  which caused §2 T4's expectation to be revised to *achieved* -- **fails the
  tier's speed clause**. No threshold was moved and none should be: 0.15 m/s
  is the plan's own number by way of T3.3, and moving it after seeing a
  measurement is what §5a voids a pass for. It is recorded here, executably,
  and in the task's ``meta.json``, so it is decided before the freeze rather
  than discovered in a grid.

The synthetic scene, once, so every fixture below can be read against it::

    ground     static, top z = 0.00, x in [-3, 28], y in [-4, 4]
               -- 28.3 m of straight run from the start, against the 15.0 m
               this task states as its minimum free run-up
    strider    a two-legged robot: base 'base_link' standing at z = 0.780,
               two feet 'foot_l' 'foot_r', 34.1 kg under 9.81 -- so body
               weight is 334.5 N and the unsupported cell's force bound is
               0.02 x that, 6.69 N
    the walk   0.30 m/s along +x for 45 s = 13.5 m; the base bobs 0.012 m at
               1.0 Hz and rolls/pitches a few hundredths of a radian with it;
               the feet alternate -- left, then right, half a cycle each
    support    attested, per axis, and identically zero: nothing is holding it

The mass and the standing height are the measured flagship's, on purpose: it
makes every support figure in this file directly comparable with the only real
measurement of a support rig this tree has, and it means the fixtures publish
the same multiples of body weight a reader will meet in a cell.

The alternation is not decoration. A two-legged robot has exactly one foot
down for most of a cycle, so the *aggregate* "is this robot touching the
ground" signal never changes state and a grader counting it would score a
perfect walk at **zero** footfalls. Counted per foot it scores hundreds.
``test_t4_core.py`` asserts both numbers on this same oracle.

Run the coverage table::

    python -m ladder.graders.fixtures_t4

Everything here is synthetic numpy: no simulator, no build, no GPU, no
network. A third party can re-derive every red in this file on a laptop.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, EngineAttribution, EvidenceBundle, IdentityRule,
    ProcessFacts)
from agentbench.graders.verdict import FAIL, INVALID, PASS  # noqa: E402
from ladder.graders import t4_core  # noqa: E402
from ladder.graders.t4_evidence import (  # noqa: E402
    CELL_SUPPORTED, CELL_UNSUPPORTED, CELL_UNVERIFIED, AppliedSupport,
    ArenaBounds, BodyPhysics, ControllerLoad, GaitContactObservation,
    GroundContact, PoseSeries, StandingHeight, T4Evidence, WalkingSurface,
    WorldPhysics)

DT = 0.02                        # a plausible sampling interval, seconds
G = 9.81

BASE_NAME = "base_link"
GROUND_NAME = "ground"
FEET = ("foot_l", "foot_r")

STAND_Z = 0.780                  # the base's settled standing height, m
MASS_KG = 34.1                   # so m.g = 334.5 N and 0.02 x m.g = 6.69 N
WALK_SPEED = 0.30                # m/s
GAIT_HZ = 1.0                    # gait cycles per second
BOB_AMP = 0.012                  # m, so the RMS about the trend is ~0.0085
ROLL_AMP = 0.04                  # rad
PITCH_AMP = 0.03                 # rad
RUN_S = 45.0                     # 13.5 m at WALK_SPEED
CONTACT_STRIDE = 2               # the contact query runs every other sample
SUPPORT_STRIDE = 10

ARENA = ((-3.0, -4.0), (28.0, 4.0))
SMALL_ARENA = ((-3.0, -4.0), (6.5, 4.0))    # the two arena-bound fixtures

# The measured support profile of this tree's own flagship, over a whole 10 m
# walk, from docs/developer/g1-endurance-2026-08-01.md section 4. Held as
# constants here rather than as sinusoids so that every per-axis peak this
# module publishes is EXACTLY the number that document reports.
G1_FY_N = 36.8                   # lateral catch, non-zero 100 % of the window
G1_TX_NM = 69.2                  # roll attitude, non-zero 100 % of the window
G1_TY_NM = 21.4                  # pitch attitude, non-zero 100 % of the window
G1_SPEED_MPS = 0.1207            # 10.0 m at t = 82.88 s, six runs, zero falls
# The rig's own clamps, which the plan's worked example cell quotes.
RIG_FZ_N = 700.0                 # ~2.09 x this robot's body weight
RIG_TX_NM = 348.0

SURFACE_SOURCE = "the task's own meta file, read by the grader"
POSE_SOURCE = "a synthetic base pose series (a fixture, not a run)"
STANDING_SOURCE = "a synthetic settled-height read at t=0 (a fixture)"
CONTACT_SOURCE = "a synthetic ground-contact scan over the walk (a fixture)"
SUPPORT_SOURCE = "a synthetic applied-wrench log, per tick (a fixture)"
ARENA_SOURCE = "a synthetic scan of the walking region's extent (a fixture)"
MASS_SOURCE = "a synthetic body-mass read at t=0 (a fixture)"
GRAVITY_SOURCE = "a synthetic world-gravity read (a fixture)"
LOAD_SOURCE = "a synthetic controller-start log (a fixture)"
LOAD_EVIDENCE = ("the driver printed its own model-load line naming the "
                 "checkpoint before its first control tick")


# --- track construction ------------------------------------------------------


def _clock(dt=DT, run_s=RUN_S):
    return np.arange(int(round(run_s / dt)) + 1) * dt


def _base_track(t, *, speed=WALK_SPEED, bob=BOB_AMP, freq=GAIT_HZ,
                stop_at=None, collapse_at=None, collapse_z=0.35,
                teleport_at=None, teleport_m=0.0, slide_at=None,
                slide_speed=0.0):
    """The base's world position over the run.

    ``stop_at`` freezes x (the robot stopped making forward progress);
    ``collapse_at`` freezes x **and** takes z down to ``collapse_z`` over one
    second (it went down); ``teleport_at`` adds a single step of
    ``teleport_m`` to x; ``slide_at`` starts a sideways drift in -y, which is
    the measured shape of a humanoid pressed against a wall and still
    locomoting.
    """
    walk_t = np.copy(t)
    hold = collapse_at if collapse_at is not None else stop_at
    if hold is not None:
        walk_t = np.minimum(walk_t, float(hold))
    x = float(speed) * walk_t
    if teleport_at is not None:
        x = x + np.where(t >= float(teleport_at), float(teleport_m), 0.0)
    y = np.zeros(len(t))
    if slide_at is not None:
        y = -float(slide_speed) * np.clip(t - float(slide_at), 0.0, None)
    z = STAND_Z + float(bob) * np.sin(2.0 * np.pi * float(freq) * t)
    if collapse_at is not None:
        f = np.clip((t - float(collapse_at)) / 1.0, 0.0, 1.0)
        z = z * (1.0 - f) + float(collapse_z) * f
    return np.stack([x, y, z], axis=1)


def _rot_track(t, *, roll_amp=ROLL_AMP, pitch_amp=PITCH_AMP, freq=GAIT_HZ,
               yaw=0.0, topple_at=None, topple_roll=1.1):
    """World-from-body rotation matrices, from a roll/pitch/yaw triple.

    Composed as ``Rz(yaw) Ry(pitch) Rx(roll)`` -- the same intrinsic z-y-x
    convention the shared ``roll_pitch`` helper inverts, so a fixture that
    builds a 1.1 rad roll is read back as a 1.1 rad roll and the test is a
    test of the threshold rather than of the algebra.
    """
    w = 2.0 * np.pi * float(freq) * t
    roll = float(roll_amp) * np.sin(w)
    pitch = float(pitch_amp) * np.sin(w + np.pi / 3.0)
    if topple_at is not None:
        f = np.clip((t - float(topple_at)) / 1.0, 0.0, 1.0)
        roll = roll * (1.0 - f) + float(topple_roll) * f
    yaw_a = np.full(len(t), float(yaw)) if np.isscalar(yaw) else np.asarray(
        yaw, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw_a), np.sin(yaw_a)
    rot = np.zeros((len(t), 3, 3), dtype=float)
    rot[:, 0, 0] = cy * cp
    rot[:, 0, 1] = cy * sp * sr - sy * cr
    rot[:, 0, 2] = cy * sp * cr + sy * sr
    rot[:, 1, 0] = sy * cp
    rot[:, 1, 1] = sy * sp * sr + cy * cr
    rot[:, 1, 2] = sy * sp * cr - cy * sr
    rot[:, 2, 0] = -sp
    rot[:, 2, 1] = cp * sr
    rot[:, 2, 2] = cp * cr
    return rot


def _contacts(t, *, freq=GAIT_HZ, stride=CONTACT_STRIDE, gait=True,
              counters=True, foreign=(), sample_times=True):
    """The ground-contact channel for one attempt.

    ``gait=False`` puts both feet down at every sample and never lifts one --
    which is what a slide, a roll and a body carried along all look like.
    ``foreign`` is a sequence of ``(body, t_from, t_to)`` for contacts with
    something that is not the ground.
    """
    ts = np.asarray(t, dtype=float)[::int(stride)]
    out = []
    for i, tt in enumerate(ts):
        if gait:
            phase = (float(tt) * float(freq)) % 1.0
            down = (FEET[0],) if phase < 0.5 else (FEET[1],)
        else:
            down = FEET
        for foot in down:
            out.append(GroundContact(
                robot_body=foot, other_body=GROUND_NAME, other_is_ground=True,
                other_is_robot=False, t_s=float(tt), step=int(i * stride),
                point=(0.0, 0.0, 0.0)))
    for body, t_from, t_to in foreign:
        for tt in ts:
            if float(t_from) <= float(tt) <= float(t_to):
                out.append(GroundContact(
                    robot_body=BASE_NAME, other_body=body,
                    other_is_ground=False, other_is_robot=False,
                    t_s=float(tt), point=(0.0, 0.0, 0.0)))
    return GaitContactObservation(
        contacts=out, sample_times=(ts if sample_times else None),
        supported=True,
        total_observed=(len(out) * 2 if counters else None),
        distinct_named=(len(out) if counters else None),
        steps_sampled=len(ts),
        window_s=(float(t[-1] - t[0]) if len(t) else None),
        source=CONTACT_SOURCE)


def _support(t, *, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0),
             attested=True, stride=SUPPORT_STRIDE):
    """A per-tick applied-wrench log. Zero everywhere unless asked otherwise.

    Held constant over the run rather than modulated, so that every per-axis
    peak this module publishes is exactly the number written in the constant.
    """
    ts = np.asarray(t, dtype=float)[::int(stride)]
    n = len(ts)
    return AppliedSupport(
        attested=attested, t=ts,
        force=np.tile(np.asarray(force, dtype=float), (n, 1)),
        torque=np.tile(np.asarray(torque, dtype=float), (n, 1)),
        source=SUPPORT_SOURCE)


def _g1_support(t):
    """The measured flagship profile: carry idle, attitude continuous.

    ``fz = 0.00 N`` for **0 %** of the window and ``tx = 69.2 N.m`` for
    **100 %** of it, with the lateral catch at 36.8 N -- the numbers in
    ``g1-endurance-2026-08-01.md`` §4, not numbers invented to be dramatic.

    One consequence of holding both attitude axes at once is stated rather
    than hidden: the torque **magnitude** peak here is hypot(69.2, 21.4) =
    72.43 N.m, while the source document reports the two per-axis peaks
    separately. The per-axis figures this fixture publishes are exact; the
    magnitude is the honest consequence of a fixture in which both are live at
    every sample.
    """
    return _support(t, force=(0.0, G1_FY_N, 0.0),
                    torque=(G1_TX_NM, G1_TY_NM, 0.0))


# --- the bundle and the channels ---------------------------------------------


def _bodies(arena=ARENA):
    strider = Body(
        body_id="#41", name=BASE_NAME, kind="Robot",
        position=(0.0, 0.0, STAND_Z),
        aabb_min=(-0.13, -0.16, 0.55), aabb_max=(0.13, 0.16, 1.02),
        n_joints=12, dynamic=True, robot_class=True, behaviour="walk",
        behaviour_declared="walk", identity_evidence="synthetic fixture")
    ground = Body(
        body_id="#2", name=GROUND_NAME, kind="Solid",
        aabb_min=(arena[0][0], arena[0][1], -0.10),
        aabb_max=(arena[1][0], arena[1][1], 0.0),
        dynamic=False, robot_class=False,
        identity_evidence="synthetic fixture")
    return [strider], [ground]


def _bundle(*, attribution=True, error_lines=(), finalize=True, exit_code=0,
            timed_out=False, deliverable="/synthetic/T4_humanoid/run.txt",
            arena=ARENA):
    movers, statics = _bodies(arena)
    return EvidenceBundle(
        task=t4_core.TASK, sim="synthetic", adapter="ladder.fixtures_t4",
        artifact=deliverable,
        identity=IdentityRule(
            label="two-legged robot",
            requirement="the robot the description in the container declares",
            scene_rule="synthetic: robot_class is set on the fixture",
            declaration_rule="synthetic: not counted", declared_count=1),
        roster=BodyInventory(bodies=list(movers), frozen=True, t_s=0.0,
                             source="synthetic frozen scan"),
        t0=BodyInventory(bodies=list(movers) + list(statics), frozen=True,
                         t_s=0.0, source="synthetic frozen scan"),
        process=ProcessFacts(
            exit_code=exit_code, timed_out=timed_out,
            error_lines=list(error_lines), log_available=True,
            log_source="synthetic error stream", behaviour_starts={"walk": 1},
            start_source="synthetic process table",
            driver_completed=bool(finalize), reached_finalize=bool(finalize),
            finalize_evidence=("the driver reached its target simulated time"
                               if finalize else "none"),
            wall_s=140.0),
        attribution=(EngineAttribution(
            backend="synthetic", solver="synthetic-pgs",
            source="a synthetic fixture, not a run") if attribution else None))


def _evidence(t, xyz, rot, *, gait=None, support=None, arena=ARENA,
              arena_obj=None, boundary=(), standing_z=STAND_Z,
              mass_kg=MASS_KG, gravity=G, method="learned_policy", loaded=True,
              orientation=True, **kw):
    bundle = _bundle(arena=arena, **kw)
    return T4Evidence(
        bundle=bundle,
        base_pose=PoseSeries(body=BASE_NAME, t=t, xyz=xyz,
                             rot=(rot if orientation else None),
                             source=POSE_SOURCE),
        standing=(StandingHeight(z_m=standing_z, body=BASE_NAME,
                                 source=STANDING_SOURCE)
                  if standing_z is not None else
                  StandingHeight(body=BASE_NAME,
                                 error="this column reports no settled "
                                       "standing height")),
        gait=(gait if gait is not None else _contacts(t)),
        support=(support if support is not None else _support(t)),
        arena=(arena_obj if arena_obj is not None else
               ArenaBounds(aabb_min=(arena[0][0], arena[0][1], -0.10),
                           aabb_max=(arena[1][0], arena[1][1], 0.0),
                           boundary_bodies=list(boundary),
                           source=ARENA_SOURCE)),
        base_physics=BodyPhysics(body=BASE_NAME, mass_kg=mass_kg, dynamic=True,
                                 source=MASS_SOURCE),
        world=WorldPhysics(gravity_mps2=gravity,
                           gravity_vec=(0.0, 0.0, -gravity),
                           source=GRAVITY_SOURCE),
        controller=ControllerLoad(
            declared_method=method, loaded=loaded,
            evidence=(LOAD_EVIDENCE if loaded else
                      ("that line never appeared; the driver ran to "
                       "completion and exited 0 anyway" if loaded is False
                       else "")),
            identity=("a checkpoint, by content hash" if loaded else ""),
            source=(LOAD_SOURCE if loaded is not None else ""),
            error=(None if loaded is not None else
                   "this column has no controller-load channel")),
        surface=WalkingSurface(names=(GROUND_NAME,), source=SURFACE_SOURCE),
        robot_name=BASE_NAME)


def _walk(**kw):
    """``(t, xyz, rot)`` for the standard walk, with keyword overrides."""
    run_s = kw.pop("run_s", RUN_S)
    base_kw = {k: kw.pop(k) for k in
               ("speed", "bob", "stop_at", "collapse_at", "collapse_z",
                "teleport_at", "teleport_m", "slide_at", "slide_speed")
               if k in kw}
    rot_kw = {k: kw.pop(k) for k in
              ("roll_amp", "pitch_amp", "yaw", "topple_at", "topple_roll")
              if k in kw}
    freq = kw.pop("freq", GAIT_HZ)
    if kw:
        raise TypeError("unknown track option(s): %s" % ", ".join(sorted(kw)))
    t = _clock(run_s=run_s)
    return (t, _base_track(t, freq=freq, **base_kw),
            _rot_track(t, freq=freq, **rot_kw))


# --- the fixtures ------------------------------------------------------------


def oracle():
    """A two-legged robot that walks 13.5 m on its legs, holding nothing.

    The positive control **and** one half of the two-cell mechanism: nothing
    is applied to it at any sample, the wrench channel says so per axis, and
    the run is published in ``T4-unsupported``.
    """
    return _evidence(*_walk())


def stopped_short():
    """T4.1: the same gait, the same everything -- for 8.4 m.

    The run simply ends before ten metres. It is deliberately NOT a robot that
    walked and then stopped: a stop would drag the mean speed down over the
    scored window and redden T4.3 as well, and the fixture would stop
    isolating T4.1.
    """
    return _evidence(*_walk(run_s=28.0))


def teleported():
    """T4.1: one 3.0 m step sideways in a single sample, mid-walk.

    Everything else is the oracle's, and the robot ends up FURTHER than ten
    metres from where it started -- which is the point. The distance clause is
    green and the continuity clause is red, and a tier that only measured
    distance would have scored this as a walk.
    """
    return _evidence(*_walk(teleport_at=15.0, teleport_m=3.0))


def arena_bound():
    """T4.1: it ran out of world at 6.2 m and stopped there.

    A small region (7.6 m of straight run available, against the 15.0 m this
    task states), a robot that walks to 0.3 m from its edge at a healthy
    0.31 m/s and then stays there for twenty-five seconds, still cycling its
    feet. T4.1 is red because the distance was not covered -- and the row
    records ``termination: arena_geometry``, prints the run-up against the
    requirement, and reports that the "crossed ten metres" clause **could not
    have passed** in a region this size. The speed, the bob and the footfalls
    are all scored over the walk and not over the twenty-five seconds at the
    wall, which is what stops this from reading as a gait failure.
    """
    t, xyz, rot = _walk(speed=0.31, stop_at=20.0)
    return _evidence(t, xyz, rot, arena=SMALL_ARENA)


def arena_bound_still_moving():
    """T4.1: it ran out of world -- and never stopped. **T4's own rule.**

    The measured shape of the failure this tier's arena rule exists for
    (``docs/developer/g1-endurance-2026-08-01.md`` §4): `x` froze against a
    wall at 12.90 m while `y` ran out to -6.88 m over two hundred seconds. The
    robot was pressed against the world and **still locomoting**, so a rule
    keyed on "the base stopped" -- which is the rule one rung down -- does not
    fire at all, and the run reads as a gait that degraded.

    Here: the same walk to 0.3 m from the region's far edge, then a steady
    0.15 m/s sideways drift for the remaining twenty-five seconds. It never
    stops. ``test_t4_core`` asserts that the other tier's rule returns
    ``None`` on this exact series and that this tier's does not.
    """
    t, xyz, rot = _walk(speed=0.31, stop_at=20.0, slide_at=20.0,
                        slide_speed=0.15)
    return _evidence(t, xyz, rot, arena=SMALL_ARENA)


def fell_height():
    """T4.2: it crosses ten metres and then goes down onto its side.

    The collapse is after the bar, on purpose: the scored window ends where
    the bar was reached, so the speed, the bob and the footfalls are all the
    oracle's, and only the fall test -- which the tier states over EVERY
    recorded sample -- sees it.
    """
    return _evidence(*_walk(collapse_at=40.0, collapse_z=0.35))


def fell_attitude():
    """T4.2: it crosses ten metres and then rolls to 1.1 rad at height.

    The base height is left at its standing value while the roll goes past the
    bound, which is not what a real topple looks like -- and that is
    deliberate. A physical topple reddens the height clause too, and this
    fixture exists to show the attitude clause failing **on its own**.
    """
    return _evidence(*_walk(topple_at=40.0, topple_roll=1.1))


def no_base_orientation():
    """T4.2 red AND vacuous: the base's pose carries no rotation.

    The most likely real partial column: the frozen motion contract carries
    position and velocity and no rotation at all, so a column with no
    ladder-side sampler cannot answer half of the fall test. Red because the
    tier's requirement was never measured; the blocker is ours.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, orientation=False)


def flagship_measured_speed():
    """T4.3: **this tree's own flagship, at the speed it actually walks.**

    Not a synthetic threshold probe. ``g1-endurance-2026-08-01.md`` records
    the shipped flagship crossing 10.0 m at t = 82.88 s in six of six runs
    with zero falls -- **0.1207 m/s made good** -- and §2 T4 inherits T3.3's
    **0.15 m/s** floor unchanged. So the artifact whose distance was measured
    before the freeze, and which caused the tier's own expectation to be
    revised from ``not_achieved`` to ``achieved``, is **red on the speed
    clause**.

    The support profile here is the measured one too, so this fixture is
    simultaneously a ``T4-supported`` cell and a T4.3 failure: exactly the
    shape our own column's cell would take if the tier ran today.

    **No threshold was moved.** 0.15 m/s is the plan's number by way of T3.3,
    and moving it after seeing a measurement is what §5a voids a pass for.
    Whoever owns the tier decides before the freeze whether a humanoid speed
    floor inherited from a quadruped tier is the floor they meant.
    """
    t, xyz, rot = _walk(speed=G1_SPEED_MPS, run_s=110.0, freq=0.9)
    return _evidence(t, xyz, rot, support=_g1_support(t))


def slid_without_gait():
    """T4.3: it covers 13.5 m and never walks.

    The base is held at a constant height (no bob) and both feet stay in
    contact for the whole run (no make-and-break). It is what a body slid,
    rolled, or carried along by a constraint looks like from the outside, and
    every other assertion is satisfied: it went the distance, it never fell,
    nothing was applied to it, the run is real.
    """
    t, xyz, rot = _walk(bob=0.0, roll_amp=0.0, pitch_amp=0.0)
    return _evidence(t, xyz, rot, gait=_contacts(t, gait=False))


def blind_contact_query():
    """T4.3 red AND vacuous: nothing could have reported a footfall.

    The shape of the historical failure the vacuity machinery exists for -- a
    contact query that returns nothing and cannot say whether it *could* have
    returned something. Here it also does not report the times it ran, so a
    lifted foot is unobservable in principle. The assertion is red (the tier
    requires transitions) and the row must also say the clause proved nothing.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, gait=GaitContactObservation(
        contacts=[], sample_times=None, supported=True, total_observed=None,
        distinct_named=None, steps_sampled=0,
        window_s=float(t[-1] - t[0]),
        source="a contact query with no witness counters and no record of "
               "when it ran"))


def leaning_on_scene_geometry():
    """T4.4: the half of the clause a force channel can never see.

    A contact naming a body the task does not declare as the ground, inside
    the scored window, and that body is one the walking region is bounded by
    -- so the row names it as an arena contact as well. **No force is applied
    to the base at all**: the wrench half is clean, the run is in the
    ``T4-unsupported`` cell by the numbers, and it is still red, because a
    robot leaning on a wall is not walking and no applied-wrench total would
    ever have shown it.

    This is the fixture that justifies reading 1 -- keeping T3.4's contact
    half on a tier whose wrench half was replaced.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, boundary=("wall",),
                     gait=_contacts(t, foreign=(("wall", 5.0, 25.0),)))


def policy_never_loaded():
    """T4.5: the deploy exited 0 and the thing that drives it never loaded.

    The trap that has already voided a quadruped result in this tree: with the
    model runtime missing from the interpreter that spawns controllers, the
    deploy runs a zero-residual baseline **and exits 0**. Here the process
    facts are impeccable -- exit 0, no error lines, finalize reached -- and the
    load attestation says no. An exit code is not evidence.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, loaded=False)


def no_policy_attestation():
    """T4.5 red AND vacuous: nothing said whether it loaded, either way.

    A realistic partial column, and the honest reading of it: the tier
    requires the load to be **asserted**, so an absent channel is a red
    assertion whose blocker is ours. It must never read as "the policy failed
    to load".
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, loaded=None)


def never_finalized():
    """T4.5: the world never finished building. Exit code still 0."""
    return _evidence(*_walk(), finalize=False)


def error_lines():
    """T4.5: the engine wrote an error-class line during the run."""
    return _evidence(*_walk(), error_lines=["ERROR: a fixture's error line"])


def missing_attribution():
    """T4.5 red, verdict INVALID: nothing names the engine that drove it."""
    return _evidence(*_walk(), attribution=False)


def g1_shaped_support():
    """PASS in ``T4-supported``: **the measured rig, published as a cell.**

    The support profile is this tree's own flagship's, measured over a whole
    10 m walk (``g1-endurance-2026-08-01.md`` §4): the **carrying channel
    reads 0.00 N and is live 0 %** of the window, while the **attitude channel
    peaks at 69.2 N.m and is live 100 %** of it, with a 36.8 N lateral catch
    at 0.110 x body weight. Against the unsupported cell's bounds -- 6.69 N
    and 2 N.m for this robot -- that is 5.5x and 34.6x over, decisively
    supported.

    The walk itself is the oracle's, so every other row is green and this
    fixture isolates one thing: **a supported run PASSES, in the other cell,
    with its figures printed inside it.** A single scalar would have reported
    this rig as "held up, 100 % of the window" and hidden that it never
    carried the robot's weight at all -- which is why the per-axis profile is
    a requirement and not an ornament.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, support=_g1_support(t))


def carried_outright():
    """PASS in ``T4-supported``: the plan's own worked cell, from measurement.

    ``capability-ladder-plan.md`` §2 T4 prints one example cell by hand:
    *"achieved 1/3 (supported: peak 2.09 x body weight, 348 N.m, 100% of
    window; reuse_class: assembled)"*. This is a run that produces exactly
    that parenthetical -- 700 N of vertical carry on a 334.5 N robot, held for
    the whole window.

    It exists to make the tier's most counter-intuitive rule impossible to
    misread: a robot that was **carried the whole way** is not a failed T4
    run. It is a published cell with a number in it, and §8 forbids quoting it
    without the number. What stops that from being a free pass is the other
    rows: it still has to travel ten metres continuously, stay upright, bob
    like something on legs and make and break ground contact, and the cell
    still says 2.09 x body weight out loud.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot,
                     support=_support(t, force=(0.0, 0.0, RIG_FZ_N),
                                      torque=(RIG_TX_NM, 0.0, 0.0)))


def support_not_attested():
    """PASS in **neither** cell. ``T4-support-unverified``, and loud.

    §2 T4's own consequence, made executable: *"Where a simulator cannot
    attest applied forces, the cell is ``achieved (support unverified)`` and
    is excluded from comparison with an attested cell."* Failing it would
    publish our missing channel as somebody else's capability gap, which §4
    forbids. Crediting it silently is what §5c exists to prevent. So it passes
    **loudly**: both wrench clauses report their witnesses absent, the cell is
    named ``T4-support-unverified`` rather than either published cell, and the
    exclusion is in the row and in the notes.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, support=AppliedSupport(
        attested=None,
        error="this column exposes no applied-wrench total for a body"))


def summarised_support():
    """PASS with the per-channel clause vacuous: totalled, not streamed.

    A column that can answer *"the largest thing we applied was X"* but cannot
    stream the wrench per axis on a clock. It is attested, it lands in a real
    cell, and it **cannot say which channel held the robot up** -- the exact
    distinction the only real measurement we have turns on. The row says so
    rather than publishing a figure that reads as if it had been resolved.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, support=AppliedSupport(
        attested=True, peak_force_n=0.0, peak_torque_nm=0.0,
        fraction_nonzero=0.0,
        source="a synthetic column that totals the wrench and does not "
               "stream it (a fixture)"))


def no_arena_channel():
    """PASS, and the cell is INCOMPLETE: nobody knows where the world ends.

    §2 T4's build note 1 makes this a requirement rather than a nicety: *"The
    T4 recorder must fail loudly on arena contact, not record a plateau ... a
    T4 cell whose recorder cannot see it is ``support_attestation``-style
    incomplete rather than quietly graded."* So the run passes on its
    assertions -- none of them depends on the region -- and the verdict
    carries ``arena_attestation: unverified``, an exclusion reason naming it,
    and a note. A reader of this cell can see that a geometry-bound run and a
    stalled one were indistinguishable in it.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, arena_obj=ArenaBounds(
        error="this column reports no walking-region extent"))


def no_attested_standing_height():
    """PASS with a vacuous clause: the fall datum came from the run itself.

    Exactly what a column with no per-robot height read can answer, so it is
    the state some columns will be in. The run's own first sample is used, the
    row says so -- and the clause reports that it could not have caught a
    robot that was already on the floor when the recording started, because
    such a robot would have supplied its own floor as the datum.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, standing_z=None)


def scripted_controller_loaded():
    """PASS, ``method: scripted``: the method is recorded and never graded.

    Not a negative. The tier permits training and does not require it -- *"a
    cell that reaches the outcome by training a policy and a cell that reaches
    it by a model-based gait are both ``achieved``, with ``method``
    recorded"* -- so the evidence that a hand-written gait passes, with its
    method printed, has to be as executable as every red above. What is NOT
    waived is the load attestation: a scripted controller must still be
    attested to have started.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, method="scripted")


# --- the map ----------------------------------------------------------------
#
# fixture -> what it must do to the verdict. Read by test_t4_core.py, printed
# by coverage_table(). Every column is EXACT: a fixture that reddens something
# not listed, leaves a clause vacuous that is not listed, or publishes in a
# cell that is not listed, has stopped isolating one thing.

_C_TEN_METRES = "crossed ten metres"
_C_CONTINUOUS = "one continuous run"
_C_STANDING = "stayed at its standing height"
_C_UPRIGHT = "stayed upright"
_C_FEET_UP = "picked its feet up"
_C_ONLY_GROUND = "nothing but the ground touched it"
_C_MEASURED = "what was applied to it was measured"
_C_PER_CHANNEL = "measured per channel"
_C_LOADED = "the thing that drives the robot actually loaded"

FIXTURE_ASSERTION_MAP = {
    "oracle": {
        "fn": oracle, "red": (), "outcome": PASS, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing -- this is the positive control, and the "
                         "unsupported half of the two-cell mechanism"},
    "stopped_short": {
        "fn": stopped_short, "red": ("T4.1",), "outcome": FAIL, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the same gait, for 8.4 m"},
    "teleported": {
        "fn": teleported, "red": ("T4.1",), "outcome": FAIL, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "one 3.0 m inter-sample step; ends further than 10 m"},
    "arena_bound": {
        "fn": arena_bound, "red": ("T4.1",), "outcome": FAIL,
        "vacuous": {"T4.1": (_C_TEN_METRES,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the world ran out at 6.2 m and it stopped there -- "
                         "recorded as termination: arena_geometry, NOT as a "
                         "gait failure"},
    "arena_bound_still_moving": {
        "fn": arena_bound_still_moving, "red": ("T4.1",), "outcome": FAIL,
        "vacuous": {"T4.1": (_C_TEN_METRES,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the world ran out at 6.2 m and it kept locomoting "
                         "sideways along the wall -- the measured shape a "
                         "stop-based rule cannot see"},
    "fell_height": {
        "fn": fell_height, "red": ("T4.2",), "outcome": FAIL, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "crosses 10 m, then the base sinks to 0.35 m"},
    "fell_attitude": {
        "fn": fell_attitude, "red": ("T4.2",), "outcome": FAIL, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "crosses 10 m, then rolls to 1.1 rad"},
    "no_base_orientation": {
        "fn": no_base_orientation, "red": ("T4.2",), "outcome": FAIL,
        "vacuous": {"T4.2": (_C_UPRIGHT,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the base's rotation never reached the grader"},
    "flagship_measured_speed": {
        "fn": flagship_measured_speed, "red": ("T4.3",), "outcome": FAIL,
        "vacuous": {}, "cell": CELL_SUPPORTED,
        "wrong_with_it": "nothing synthetic -- this tree's own flagship walks "
                         "10.0 m at 0.1207 m/s and the tier's inherited floor "
                         "is 0.15 m/s"},
    "slid_without_gait": {
        "fn": slid_without_gait, "red": ("T4.3",), "outcome": FAIL,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "13.5 m at a constant height with both feet never "
                         "leaving the floor -- slid, not walked"},
    "blind_contact_query": {
        "fn": blind_contact_query, "red": ("T4.3",), "outcome": FAIL,
        "vacuous": {"T4.3": (_C_FEET_UP,), "T4.4": (_C_ONLY_GROUND,)},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "no footfalls, and no witness that one could have "
                         "been reported"},
    "leaning_on_scene_geometry": {
        "fn": leaning_on_scene_geometry, "red": ("T4.4",), "outcome": FAIL,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "in contact with a body the task does not declare as "
                         "the ground, during the walk, with zero force "
                         "applied -- the half of the clause no wrench channel "
                         "could ever see"},
    "policy_never_loaded": {
        "fn": policy_never_loaded, "red": ("T4.5",), "outcome": FAIL,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the driver never loaded, and exited 0"},
    "no_policy_attestation": {
        "fn": no_policy_attestation, "red": ("T4.5",), "outcome": FAIL,
        "vacuous": {"T4.5": (_C_LOADED,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing said whether the driver loaded, either way"},
    "never_finalized": {
        "fn": never_finalized, "red": ("T4.5",), "outcome": FAIL,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "the world never finished building; exit code 0"},
    "error_lines": {
        "fn": error_lines, "red": ("T4.5",), "outcome": FAIL, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "an error-class line during the run"},
    "missing_attribution": {
        "fn": missing_attribution, "red": ("T4.5",), "outcome": INVALID,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing names the engine that drove the run"},
    "g1_shaped_support": {
        "fn": g1_shaped_support, "red": (), "outcome": PASS, "vacuous": {},
        "cell": CELL_SUPPORTED,
        "wrong_with_it": "nothing gradeable -- the measured flagship support "
                         "profile (carry idle 0 % of the window, attitude "
                         "69.2 N.m for 100 %) is a CELL, not a failure"},
    "carried_outright": {
        "fn": carried_outright, "red": (), "outcome": PASS, "vacuous": {},
        "cell": CELL_SUPPORTED,
        "wrong_with_it": "nothing gradeable -- carried at 2.09 x body weight "
                         "for the whole window, which is published with the "
                         "number rather than failed"},
    "summarised_support": {
        "fn": summarised_support, "red": (), "outcome": PASS,
        "vacuous": {"T4.4": (_C_PER_CHANNEL,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing physical -- the column totals the wrench "
                         "and cannot stream it per axis, so the cell cannot "
                         "say WHICH channel held the robot up"},
    "support_not_attested": {
        "fn": support_not_attested, "red": (), "outcome": PASS,
        "vacuous": {"T4.4": tuple(sorted((_C_MEASURED, _C_PER_CHANNEL)))},
        "cell": CELL_UNVERIFIED,
        "wrong_with_it": "nothing physical -- the column cannot attest what "
                         "was applied, so the run is in NEITHER published "
                         "cell and is excluded from comparison rather than "
                         "credited"},
    "no_arena_channel": {
        "fn": no_arena_channel, "red": (), "outcome": PASS, "vacuous": {},
        "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing physical -- nobody attested where the world "
                         "ends, so a geometry-bound run and a stalled one are "
                         "indistinguishable and the cell is INCOMPLETE"},
    "no_attested_standing_height": {
        "fn": no_attested_standing_height, "red": (), "outcome": PASS,
        "vacuous": {"T4.2": (_C_STANDING,)}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing physical -- no per-robot standing height, "
                         "so the fall datum is the run's own first sample and "
                         "a robot already on the floor could not be caught"},
    "scripted_controller_loaded": {
        "fn": scripted_controller_loaded, "red": (), "outcome": PASS,
        "vacuous": {}, "cell": CELL_UNSUPPORTED,
        "wrong_with_it": "nothing gradeable -- the gait was hand-written, "
                         "which is RECORDED as the method and never graded"},
}

# Every assertion the tier states must appear as some fixture's target.
ASSERTIONS = tuple(aid for aid, _what in t4_core._ALL)


def uncovered_assertions():
    """Assertions no negative fixture drives red. Must be empty at freeze."""
    covered = set()
    for spec in FIXTURE_ASSERTION_MAP.values():
        covered.update(spec["red"])
    return tuple(a for a in ASSERTIONS if a not in covered)


def uncovered_cells():
    """Cells no fixture lands in. Must be empty: all three are reachable."""
    seen = {spec["cell"] for spec in FIXTURE_ASSERTION_MAP.values()}
    return tuple(c for c in (CELL_UNSUPPORTED, CELL_SUPPORTED, CELL_UNVERIFIED)
                 if c not in seen)


def coverage_table():
    """``[{assertion, fixture, wrong_with_it, outcome, cell}]``, one per red."""
    rows = []
    for aid in ASSERTIONS:
        for name, spec in FIXTURE_ASSERTION_MAP.items():
            if aid in spec["red"]:
                rows.append({"assertion": aid, "fixture": name,
                             "wrong_with_it": spec["wrong_with_it"],
                             "outcome": spec["outcome"],
                             "cell": spec["cell"]})
    return rows


def render_coverage_table():
    lines = ["T4 red-evidence coverage -- every assertion, the fixture that "
             "turns it red",
             "",
             "%-7s  %-30s  %-9s  %s" % ("assert", "fixture", "outcome",
                                        "what is wrong with the artifact"),
             "%-7s  %-30s  %-9s  %s" % ("-" * 7, "-" * 30, "-" * 9, "-" * 46)]
    for r in coverage_table():
        lines.append("%-7s  %-30s  %-9s  %s"
                     % (r["assertion"], r["fixture"], r["outcome"],
                        r["wrong_with_it"]))
    missing = uncovered_assertions()
    lines += ["", ("every assertion has a negative fixture"
                   if not missing else
                   "UNCOVERED (must be empty before freeze): %s"
                   % ", ".join(missing))]
    gaps = uncovered_cells()
    lines += [("every cell the tier can publish has a fixture that lands in it"
               if not gaps else
               "CELLS WITH NO FIXTURE (must be empty): %s" % ", ".join(gaps))]
    passers = [n for n, s in FIXTURE_ASSERTION_MAP.items()
               if s["outcome"] == PASS and n != "oracle"]
    lines += ["", "fixtures that must PASS, and what they prove:"]
    for name in passers:
        spec = FIXTURE_ASSERTION_MAP[name]
        lines.append("  %-30s [%s] %s"
                     % (name, spec["cell"], spec["wrong_with_it"]))
    return "\n".join(lines)


def observed_vacuous(verdict):
    """``{assertion: (clause, ...)}``, normalised against the map."""
    return {aid: tuple(sorted(clauses))
            for aid, clauses in verdict.vacuous.items()}


def expected_vacuous(spec):
    return {aid: tuple(sorted(clauses))
            for aid, clauses in (spec.get("vacuous") or {}).items()}


def check_fixture(name):
    """``(verdict, problems)`` for one fixture, against its declared row."""
    spec = FIXTURE_ASSERTION_MAP[name]
    v = t4_core.grade(spec["fn"]())
    problems = []
    red, want = tuple(sorted(v.failed)), tuple(sorted(spec["red"]))
    if red != want:
        problems.append("red is %s, declared %s" % (red or ("-",),
                                                    want or ("-",)))
    if v.outcome != spec["outcome"]:
        problems.append("outcome is %s, declared %s"
                        % (v.outcome, spec["outcome"]))
    if observed_vacuous(v) != expected_vacuous(spec):
        problems.append("vacuous clauses are %s, declared %s"
                        % (observed_vacuous(v), expected_vacuous(spec)))
    if v.measurements.get("cell") != spec["cell"]:
        problems.append("cell is %s, declared %s"
                        % (v.measurements.get("cell"), spec["cell"]))
    return v, problems


def main():
    print(render_coverage_table())
    print("")
    print("live check -- grading every fixture now:")
    bad = 0
    for name in FIXTURE_ASSERTION_MAP:
        v, problems = check_fixture(name)
        bad += 1 if problems else 0
        vac = ";".join("%s:%s" % (a, "/".join(c))
                       for a, c in sorted(observed_vacuous(v).items()))
        print("  %-30s %-8s red=%-8s cell=%-24s method=%-14s end=%-16s "
              "vacuous=%s"
              % (name, v.outcome, ",".join(sorted(v.failed)) or "-",
                 v.measurements.get("cell"), v.measurements.get("method"),
                 v.measurements.get("termination_cause") or "-", vac or "-"))
        if problems:
            print("      MISMATCH: %s" % "; ".join(problems))
    print("")
    print("the two cells, rendered as they must be published:")
    for name in ("oracle", "g1_shaped_support", "carried_outright",
                 "support_not_attested"):
        v = t4_core.grade(FIXTURE_ASSERTION_MAP[name]["fn"]())
        print("  %-30s %s" % (name, v.measurements.get("cell_text")))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
