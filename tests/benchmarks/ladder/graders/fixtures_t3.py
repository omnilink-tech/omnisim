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

"""T3's negative fixtures -- the red-evidence rule, made executable.

``capability-ladder-plan.md`` §5c.2, binding and standing: *"no assertion
enters a ladder cell until it has been observed FAILING on a deliberately
wrong artifact, with that negative fixture named in the assertion's record. A
green assertion is not evidence that the assertion works ... A ``null`` agent
turning every assertion red does not satisfy this rule."*

That last sentence is what shapes this module, exactly as it shapes
:mod:`ladder.graders.fixtures` and :mod:`ladder.graders.fixtures_t2`. Every
fixture below is the :func:`oracle` run with **one** thing wrong, and
:data:`FIXTURE_ASSERTION_MAP` records which single assertion it must turn red
-- so a fixture that reddens two assertions, or the wrong one, is a test
failure. §5c names two of these by hand and both are here: *"a crawling robot
for T3.3, a robot on a crane for T3.4"*.

Five fixtures exist to prove something is **not** graded, or is graded
*loudly*, rather than to prove something is:

* :func:`slow_static_gait` PASSes, and it is the **must-PASS control for the
  2026-08-02 repair of T3.3's gait-liveness predicate**
  (``t3_core`` reading 5b). Before that repair it was RED, and it should never
  have been: a statically-stable crawl covering 16.47 m at 0.27 m/s with 732
  make-and-break ground-contact transitions was failed by a ``>= 0.005 m``
  vertical-bob clause on a 0.0018 m bob. The bob is still measured and still
  printed in its row, beside the bar that used to apply and the verdict that
  bar would have returned -- it simply is not a pass condition any more. Its
  sibling :func:`slid_without_gait` is what proves the repair did not open a
  hole: it is red through the make-and-break clause **alone**.
* :func:`scripted_controller_loaded` PASSes with ``method: scripted``. §3.2
  makes the method a recorded field and never a pass condition, and §2 T3 says
  it in as many words -- *"a cell that reaches the outcome by training a policy
  and a cell that reaches it by a model-based gait are both achieved"* -- so
  the evidence that it is recorded-not-graded has to be as executable as the
  rest.
* :func:`support_not_attested` PASSes with ``support_attestation:
  unverified``, the wrench clause reporting its witness absent, and
  ``excluded_from_comparison`` set. That is the plan's consequence exactly:
  recorded and excluded, **never** silently credited and never failed as if
  our missing channel were somebody else's capability gap.
* :func:`arena_bound` is red on T3.1 and is *not* a gait finding, and the row
  says so: ``termination: arena_geometry``, the free run-up printed against the
  minimum this task states, and T3.1's "crossed ten metres" clause reporting
  that it could not have passed. It is the executable form of the measured
  build note in ``docs/developer/g1-endurance-2026-08-01.md`` §8 -- *"all six
  runs here would have been misread as 'the walk degrades to 0.037 m/s after
  ~110 s' by a grader that did not know where the walls were."*
* :func:`no_attested_standing_height` PASSes with the fall test's datum clause
  vacuous: the height came from the run's own first sample, which a robot that
  never stood up would have supplied from the floor.

The synthetic scene, once, so every fixture below can be read against it::

    ground     static, top z = 0.00, x in [-3, 25], y in [-4, 4]
               -- 25.3 m of straight run from the start, against the 15.0 m
               this task states as its minimum free run-up
    walker     a quadruped: base 'walker' standing at z = 0.42, four feet
               'foot_fl' 'foot_fr' 'foot_rl' 'foot_rr', 12.0 kg, under 9.81
    the walk   0.30 m/s along +x for 40 s = 12.0 m; the base bobs 0.012 m at
               1.6 Hz and rolls/pitches a few hundredths of a radian with it;
               the feet trot -- (fl, rr) then (fr, rl), half a cycle each
    support    attested, and identically zero: nothing is holding it up

The trot is not decoration. It is what makes T3.3 a test of a **gait** rather
than of travel: the aggregate "is this robot touching the ground" signal never
changes state across the whole walk -- some foot is always down -- and counted
that way a perfect walk scores **zero** transitions. Counted per foot it scores
hundreds. ``test_t3_core.py`` asserts both numbers on this same oracle.

Run the coverage table::

    python -m ladder.graders.fixtures_t3

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
from ladder.graders import t3_core  # noqa: E402
from ladder.graders.t3_evidence import (  # noqa: E402
    AppliedSupport, ArenaBounds, BodyPhysics, ControllerLoad,
    GaitContactObservation, GroundContact, PoseSeries, StandingHeight,
    T3Evidence, WalkingSurface, WorldPhysics)

DT = 0.02                        # a plausible sampling interval, seconds
G = 9.81

BASE_NAME = "walker"
GROUND_NAME = "ground"
FEET = ("foot_fl", "foot_fr", "foot_rl", "foot_rr")

STAND_Z = 0.42                   # the base's settled standing height, m
MASS_KG = 12.0
WALK_SPEED = 0.30                # m/s
GAIT_HZ = 1.6                    # gait cycles per second
BOB_AMP = 0.012                  # m, so the RMS about the trend is ~0.0085
ROLL_AMP = 0.04                  # rad
PITCH_AMP = 0.03                 # rad
RUN_S = 40.0                     # 12.0 m at WALK_SPEED
CONTACT_STRIDE = 2               # the contact query runs every other sample
SUPPORT_STRIDE = 10

ARENA = ((-3.0, -4.0), (25.0, 4.0))
SMALL_ARENA = ((-3.0, -4.0), (6.5, 4.0))    # used by arena_bound only

# The slow, statically-stable, entirely correct walk the retired bob clause
# used to FAIL -- see :func:`slow_static_gait`. Every number here is chosen to
# reproduce the shape of the measurement that got the clause retired
# (``tasks/T3_quadruped/meta.json`` ->
# ``container.authored_here.what_the_demonstration_found_about_THIS_ASSET``):
# 0.27 m/s, ~16.5 m covered, hundreds of footfalls, and a vertical bob well
# under the 0.005 m the clause used to require.
SLOW_SPEED = 0.27                # m/s
SLOW_RUN_S = 61.0                # 16.47 m at SLOW_SPEED
SLOW_GAIT_HZ = 2.48              # a crawl's cycle rate, not a trot's
SLOW_BOB_AMP = 0.00255           # m, so the RMS about the trend is 0.0018
SLOW_ROLL_AMP = 0.012            # rad; a statically stable crawl barely rolls
SLOW_PITCH_AMP = 0.009           # rad

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
                stop_at=None, collapse_at=None, collapse_z=0.20,
                teleport_at=None, teleport_m=0.0):
    """The base's world position over the run.

    ``stop_at`` freezes x (the robot stopped walking); ``collapse_at`` freezes
    x **and** takes z down to ``collapse_z`` over one second (it went down);
    ``teleport_at`` adds a single step of ``teleport_m`` to x.
    """
    walk_t = np.copy(t)
    hold = collapse_at if collapse_at is not None else stop_at
    if hold is not None:
        walk_t = np.minimum(walk_t, float(hold))
    x = float(speed) * walk_t
    if teleport_at is not None:
        x = x + np.where(t >= float(teleport_at), float(teleport_m), 0.0)
    z = STAND_Z + float(bob) * np.sin(2.0 * np.pi * float(freq) * t)
    if collapse_at is not None:
        f = np.clip((t - float(collapse_at)) / 1.0, 0.0, 1.0)
        z = z * (1.0 - f) + float(collapse_z) * f
    return np.stack([x, np.zeros(len(t)), z], axis=1)


def _rot_track(t, *, roll_amp=ROLL_AMP, pitch_amp=PITCH_AMP, freq=GAIT_HZ,
               yaw=0.0, topple_at=None, topple_roll=1.1):
    """World-from-body rotation matrices, from a roll/pitch/yaw triple.

    Composed as ``Rz(yaw) Ry(pitch) Rx(roll)`` -- the same intrinsic z-y-x
    convention :func:`t3_evidence.roll_pitch` inverts, so a fixture that builds
    a 1.1 rad roll is read back as a 1.1 rad roll and the test is a test of the
    threshold rather than of the algebra.
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

    ``gait=False`` puts all four feet down at every sample and never lifts one
    -- which is what a slide, a roll and a drag all look like. ``foreign`` is a
    sequence of ``(body, t_from, t_to)`` for contacts with something that is
    not the ground.
    """
    ts = np.asarray(t, dtype=float)[::int(stride)]
    out = []
    for i, tt in enumerate(ts):
        if gait:
            phase = (float(tt) * float(freq)) % 1.0
            down = (FEET[0], FEET[3]) if phase < 0.5 else (FEET[1], FEET[2])
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
    """A per-tick applied-wrench log. Zero everywhere unless asked otherwise."""
    ts = np.asarray(t, dtype=float)[::int(stride)]
    n = len(ts)
    return AppliedSupport(
        attested=attested, t=ts,
        force=np.tile(np.asarray(force, dtype=float), (n, 1)),
        torque=np.tile(np.asarray(torque, dtype=float), (n, 1)),
        source=SUPPORT_SOURCE)


# --- the bundle and the channels ---------------------------------------------


def _bodies(arena=ARENA):
    walker = Body(
        body_id="#31", name=BASE_NAME, kind="Robot",
        position=(0.0, 0.0, STAND_Z),
        aabb_min=(-0.30, -0.18, 0.10), aabb_max=(0.30, 0.18, 0.50),
        n_joints=12, dynamic=True, robot_class=True, behaviour="walk",
        behaviour_declared="walk", identity_evidence="synthetic fixture")
    ground = Body(
        body_id="#2", name=GROUND_NAME, kind="Solid",
        aabb_min=(arena[0][0], arena[0][1], -0.10),
        aabb_max=(arena[1][0], arena[1][1], 0.0),
        dynamic=False, robot_class=False,
        identity_evidence="synthetic fixture")
    return [walker], [ground]


def _bundle(*, attribution=True, error_lines=(), finalize=True, exit_code=0,
            timed_out=False, deliverable="/synthetic/T3_quadruped/run.txt",
            arena=ARENA):
    movers, statics = _bodies(arena)
    return EvidenceBundle(
        task=t3_core.TASK, sim="synthetic", adapter="ladder.fixtures_t3",
        artifact=deliverable,
        identity=IdentityRule(
            label="four-legged robot",
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
            wall_s=95.0),
        attribution=(EngineAttribution(
            backend="synthetic", solver="synthetic-pgs",
            source="a synthetic fixture, not a run") if attribution else None))


def _evidence(t, xyz, rot, *, gait=None, support=None, arena=ARENA,
              standing_z=STAND_Z, mass_kg=MASS_KG, gravity=G,
              method="learned_policy", loaded=True, orientation=True, **kw):
    bundle = _bundle(arena=arena, **kw)
    return T3Evidence(
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
        arena=ArenaBounds(aabb_min=(arena[0][0], arena[0][1], -0.10),
                          aabb_max=(arena[1][0], arena[1][1], 0.0),
                          boundary_bodies=[], source=ARENA_SOURCE),
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
    t = _clock(run_s=run_s)
    base_kw = {k: kw.pop(k) for k in
               ("speed", "bob", "stop_at", "collapse_at", "collapse_z",
                "teleport_at", "teleport_m") if k in kw}
    rot_kw = {k: kw.pop(k) for k in
              ("roll_amp", "pitch_amp", "yaw", "topple_at", "topple_roll")
              if k in kw}
    if kw:
        raise TypeError("unknown track option(s): %s" % ", ".join(sorted(kw)))
    return t, _base_track(t, **base_kw), _rot_track(t, **rot_kw)


# --- the fixtures ------------------------------------------------------------


def oracle():
    """A quadruped that walks 12 m on its legs, upright, holding nothing."""
    return _evidence(*_walk())


def stopped_short():
    """T3.1: the same gait, the same everything -- for 6.9 m.

    The run simply ends before ten metres. It is deliberately NOT a robot that
    walked and then stopped: a stop would drag the mean speed down over the
    scored window and redden T3.3 as well, and the fixture would stop
    isolating T3.1.
    """
    return _evidence(*_walk(run_s=23.0))


def teleported():
    """T3.1: one 3.0 m step sideways in a single sample, mid-walk.

    Everything else is the oracle's, and the robot ends up FURTHER than ten
    metres from where it started -- which is the point. The distance clause is
    green and the continuity clause is red, and a tier that only measured
    distance would have scored this as a walk.
    """
    return _evidence(*_walk(teleport_at=10.0, teleport_m=3.0))


def arena_bound():
    """T3.1: it ran out of world at 6.2 m, and the row has to say so.

    The measured failure mode this exists for is
    ``docs/developer/g1-endurance-2026-08-01.md`` §8: the shipped flagship's
    six runs were **all** ended by the arena wall, and a grader blind to the
    boundary would have published them as a gait that degraded to 0.037 m/s.

    So: a small region (7.6 m of straight run available, against the 15.0 m
    this task states), a robot that walks to 0.3 m from its edge at a healthy
    0.31 m/s and then stays there for twenty seconds, still cycling its feet.
    T3.1 is red because the distance was not covered -- and the row records
    ``termination: arena_geometry``, prints the run-up against the requirement,
    and reports that the "crossed ten metres" clause **could not have passed**
    in a region this size. The speed, the bob and the footfalls are all scored
    over the walk and not over the twenty seconds of standing at the wall,
    which is what stops this from reading as a gait failure.
    """
    t, xyz, rot = _walk(speed=0.31, stop_at=20.0)
    return _evidence(t, xyz, rot, arena=SMALL_ARENA)


def fell_height():
    """T3.2: it crosses ten metres and then goes down onto its belly.

    The collapse is after the bar, on purpose: the scored window ends where the
    bar was reached, so the speed, the bob and the footfalls are all the
    oracle's, and only the fall test -- which the tier states over EVERY
    recorded sample -- sees it.
    """
    return _evidence(*_walk(collapse_at=35.0, collapse_z=0.20))


def fell_attitude():
    """T3.2: it crosses ten metres and then rolls onto its side at height.

    The base height is left at its standing value while the roll goes to 1.1
    rad, which is not what a real topple looks like -- and that is deliberate.
    A physical topple reddens the height clause too, and this fixture exists to
    show the attitude clause failing **on its own**.
    """
    return _evidence(*_walk(topple_at=35.0, topple_roll=1.1))


def too_slow():
    """T3.3: it covers the distance at 0.12 m/s, which is not a walk here.

    Same gait, same footfalls, same bob -- the tier's speed floor is what this
    isolates, and the robot still travels 10.8 m so T3.1 stays green.
    """
    return _evidence(*_walk(run_s=90.0, speed=0.12))


def slid_without_gait():
    """T3.3: §5c's "crawling robot" -- it covers 12 m and never walks.

    The base is held at a constant height (no bob) and all four feet stay in
    contact for the whole run (no make-and-break). It is what a body slid,
    rolled, or dragged by a constraint looks like from the outside, and every
    other assertion is satisfied: it went the distance, it never fell, nothing
    was applied to it, the run is real.

    **Since the 2026-08-02 repair this fixture reddens T3.3 through the
    make-and-break clause ALONE**, and that is what makes the repair safe
    rather than a weakening: the bob is still 0.0 m and is still printed, but
    it decides nothing, and the assertion is red because the transition count
    is **0** against the tier's ``>= 8``.
    ``test_the_slider_still_reddens_through_the_transition_clause_alone``
    proves it by restoring the bob and leaving the contacts welded down -- the
    assertion stays red.
    """
    t, xyz, rot = _walk(bob=0.0, roll_amp=0.0, pitch_amp=0.0)
    return _evidence(t, xyz, rot, gait=_contacts(t, gait=False))


def slow_static_gait():
    """PASS: the slow, statically-stable walk the old bob clause FAILED.

    **The must-PASS control for the 2026-08-02 repair**, and a negative fixture
    in the only sense that matters here: before the repair this run was RED,
    and it should never have been. It is the measured shape of this container's
    own achievability oracle driven slowly
    (``meta.json`` -> ``grading_readings.T3.3_REPAIRED_2026-08-02``): a
    four-beat crawl at **0.27 m/s** covering **16.47 m** without falling, with
    **732 make-and-break ground-contact transitions** across all four feet --
    and a vertical bob of **0.0018 m RMS**, comfortably under the **0.005 m**
    the clause used to require.

    Nothing about it is a slide. The feet demonstrably leave the floor
    hundreds of times, the robot makes good more than a metre and a half of
    ground per bob cycle, and the reason the base barely rises and falls is
    that a statically stable gait holds it at a constant commanded height --
    which is a property of the gait style, not evidence about whether it
    walked.

    The row prints the bob, prints the retired bar beside it, reports that the
    retired clause **would not** have passed, and carries the sentence saying
    this cell passes only under the new reading.
    """
    t = _clock(run_s=SLOW_RUN_S)
    xyz = _base_track(t, speed=SLOW_SPEED, bob=SLOW_BOB_AMP, freq=SLOW_GAIT_HZ)
    rot = _rot_track(t, roll_amp=SLOW_ROLL_AMP, pitch_amp=SLOW_PITCH_AMP,
                     freq=SLOW_GAIT_HZ)
    return _evidence(t, xyz, rot, gait=_contacts(t, freq=SLOW_GAIT_HZ))


def blind_contact_query():
    """T3.3 red AND vacuous: nothing could have reported a footfall.

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


def held_up_beyond_the_threshold():
    """T3.4: §5c's "robot on a crane" -- and the crane is measured.

    A steady 180 N upward, which is 1.53 x this robot's body weight against a
    threshold of 0.02 x. The walk is otherwise the oracle's and every other
    assertion is green, which is the whole point: this robot **did** cross ten
    metres on its legs, and something was carrying it while it did.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, support=_support(t, force=(0.0, 0.0, 180.0)))


def leaning_on_scene_geometry():
    """T3.4: the other half of the clause -- it is touching something else.

    A contact naming a body the task does not declare as the ground, inside
    the scored window. No force is applied to the base at all, so the wrench
    half is clean and only the contact half fails.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot,
                     gait=_contacts(t, foreign=(("wall", 5.0, 25.0),)))


def support_not_attested():
    """PASS, ``support_attestation: unverified``. **Not a silent pass.**

    The plan's own consequence, made executable: *"Where a simulator cannot
    attest the applied-force total, the cell records ``support_attestation:
    unverified`` and is excluded from comparison with any cell that has it."*
    Failing it would publish our missing channel as somebody else's capability
    gap, which §4 forbids. Crediting it silently is what §5c exists to prevent.
    So it passes **loudly**: the clause reports its witness absent, the cell
    carries ``unverified`` and ``excluded_from_comparison``, and the note says
    the run is not comparable with an attested one.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, support=AppliedSupport(
        attested=None,
        error="this column exposes no applied-wrench total for a body"))


def policy_never_loaded():
    """T3.5: the deploy exited 0 and the thing that drives it never loaded.

    The trap that has already voided a quadruped result in this tree (§2 T3):
    with the model runtime missing from the interpreter that spawns
    controllers, the deploy runs a zero-residual baseline **and exits 0**. Here
    the process facts are impeccable -- exit 0, no error lines, finalize
    reached -- and the load attestation says no. An exit code is not evidence.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, loaded=False)


def no_policy_attestation():
    """T3.5 red AND vacuous: nothing said whether it loaded, either way.

    A realistic partial column, and the honest reading of it: the tier requires
    the load to be **asserted**, so an absent channel is a red assertion whose
    blocker is ours. It must never read as "the policy failed to load".
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, loaded=None)


def never_finalized():
    """T3.5: the world never finished building. Exit code still 0."""
    return _evidence(*_walk(), finalize=False)


def error_lines():
    """T3.5: the engine wrote an error-class line during the run."""
    return _evidence(*_walk(), error_lines=["ERROR: a fixture's error line"])


def missing_attribution():
    """T3.5 red, verdict INVALID: nothing names the engine that drove it."""
    return _evidence(*_walk(), attribution=False)


def no_attested_standing_height():
    """PASS with a vacuous clause: the fall datum came from the run itself.

    Exactly what a column with no per-robot height read can answer, so it is
    the state some columns will be in. The run's own first sample is used, the
    row says so -- and the clause reports that it could not have caught a robot
    that was already on the floor when the recording started, because such a
    robot would have supplied its own floor as the datum.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, standing_z=None)


def no_base_orientation():
    """T3.2 red AND vacuous: the base's pose carries no rotation.

    The most likely real partial column: the frozen motion contract carries
    position and velocity and no rotation at all, so a column with no
    ladder-side sampler cannot answer half of the fall test. Red because the
    tier's requirement was never measured; the blocker is ours.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, orientation=False)


def scripted_controller_loaded():
    """PASS, ``method: scripted``: the method is recorded and never graded.

    Not a negative. §2 T3 permits training and does not require it -- *"a cell
    that reaches the outcome by training a policy and a cell that reaches it by
    a model-based gait are both ``achieved``, with ``method`` recorded"* -- so
    the evidence that a hand-written gait passes, with its method printed, has
    to be as executable as every red above. What is NOT waived is the load
    attestation: a scripted controller must still be attested to have started.
    """
    t, xyz, rot = _walk()
    return _evidence(t, xyz, rot, method="scripted")


# --- the map ----------------------------------------------------------------
#
# fixture -> what it must do to the verdict. Read by test_t3_core.py, printed
# by coverage_table(). Both columns are EXACT, for the reason fixtures.py gives
# at the same place: a fixture that reddens something not listed here has
# stopped isolating one assertion.

_C_TEN_METRES = "crossed ten metres"
_C_CONTINUOUS = "one continuous run"
_C_STANDING = "stayed at its standing height"
_C_UPRIGHT = "stayed upright"
_C_FEET_UP = "picked its feet up"
_C_ONLY_GROUND = "nothing but the ground touched it"
_C_NO_WRENCH = "nothing pulled or pushed on it"
_C_LOADED = "the thing that drives the robot actually loaded"

FIXTURE_ASSERTION_MAP = {
    "oracle": {
        "fn": oracle, "red": (), "outcome": PASS, "vacuous": {},
        "wrong_with_it": "nothing -- this is the positive control"},
    "stopped_short": {
        "fn": stopped_short, "red": ("T3.1",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "the same gait, for 6.9 m"},
    "teleported": {
        "fn": teleported, "red": ("T3.1",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "one 3.0 m inter-sample step; ends further than 10 m"},
    "arena_bound": {
        "fn": arena_bound, "red": ("T3.1",), "outcome": FAIL,
        "vacuous": {"T3.1": (_C_TEN_METRES,)},
        "wrong_with_it": "the world ran out at 6.2 m -- recorded as "
                         "termination: arena_geometry, NOT as a gait failure"},
    "fell_height": {
        "fn": fell_height, "red": ("T3.2",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "crosses 10 m, then the base sinks to 0.20 m"},
    "fell_attitude": {
        "fn": fell_attitude, "red": ("T3.2",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "crosses 10 m, then rolls to 1.1 rad"},
    "no_base_orientation": {
        "fn": no_base_orientation, "red": ("T3.2",), "outcome": FAIL,
        "vacuous": {"T3.2": (_C_UPRIGHT,)},
        "wrong_with_it": "the base's rotation never reached the grader"},
    "too_slow": {
        "fn": too_slow, "red": ("T3.3",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "covers 10.8 m, at 0.12 m/s"},
    "slid_without_gait": {
        "fn": slid_without_gait, "red": ("T3.3",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "12 m at a constant height with four feet never "
                         "leaving the floor -- slid, not walked. Red through "
                         "the make-and-break clause ALONE (0 transitions "
                         "against >= 8) since the bob stopped being graded"},
    "slow_static_gait": {
        "fn": slow_static_gait, "red": (), "outcome": PASS, "vacuous": {},
        "wrong_with_it": "nothing -- the MUST-PASS CONTROL for the 2026-08-02 "
                         "repair: a statically-stable crawl, 16.47 m at 0.27 "
                         "m/s with 732 footfall transitions and a 0.0018 m "
                         "bob, which the retired >= 0.005 m clause FAILED"},
    "blind_contact_query": {
        "fn": blind_contact_query, "red": ("T3.3",), "outcome": FAIL,
        "vacuous": {"T3.3": (_C_FEET_UP,), "T3.4": (_C_ONLY_GROUND,)},
        "wrong_with_it": "no footfalls, and no witness that one could have "
                         "been reported"},
    "held_up_beyond_the_threshold": {
        "fn": held_up_beyond_the_threshold, "red": ("T3.4",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "walks the whole way under a steady 180 N lift -- "
                         "1.53 x body weight, attested"},
    "leaning_on_scene_geometry": {
        "fn": leaning_on_scene_geometry, "red": ("T3.4",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "in contact with a body the task does not declare as "
                         "the ground, during the walk"},
    "policy_never_loaded": {
        "fn": policy_never_loaded, "red": ("T3.5",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "the driver never loaded, and exited 0"},
    "no_policy_attestation": {
        "fn": no_policy_attestation, "red": ("T3.5",), "outcome": FAIL,
        "vacuous": {"T3.5": (_C_LOADED,)},
        "wrong_with_it": "nothing said whether the driver loaded, either way"},
    "never_finalized": {
        "fn": never_finalized, "red": ("T3.5",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "the world never finished building; exit code 0"},
    "error_lines": {
        "fn": error_lines, "red": ("T3.5",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "an error-class line during the run"},
    "missing_attribution": {
        "fn": missing_attribution, "red": ("T3.5",), "outcome": INVALID,
        "vacuous": {},
        "wrong_with_it": "nothing names the engine that drove the run"},
    "support_not_attested": {
        "fn": support_not_attested, "red": (), "outcome": PASS,
        "vacuous": {"T3.4": (_C_NO_WRENCH,)},
        "wrong_with_it": "nothing physical -- the column cannot attest what "
                         "was applied, so the cell is unverified and EXCLUDED "
                         "from comparison rather than credited"},
    "no_attested_standing_height": {
        "fn": no_attested_standing_height, "red": (), "outcome": PASS,
        "vacuous": {"T3.2": (_C_STANDING,)},
        "wrong_with_it": "nothing physical -- no per-robot standing height, "
                         "so the fall datum is the run's own first sample and "
                         "a robot already on the floor could not be caught"},
    "scripted_controller_loaded": {
        "fn": scripted_controller_loaded, "red": (), "outcome": PASS,
        "vacuous": {},
        "wrong_with_it": "nothing gradeable -- the gait was hand-written, "
                         "which is RECORDED as the method and never graded"},
}

# Every assertion the tier states must appear as some fixture's target.
ASSERTIONS = tuple(aid for aid, _what in t3_core._ALL)


def uncovered_assertions():
    """Assertions no negative fixture drives red. Must be empty at freeze."""
    covered = set()
    for spec in FIXTURE_ASSERTION_MAP.values():
        covered.update(spec["red"])
    return tuple(a for a in ASSERTIONS if a not in covered)


def coverage_table():
    """``[{assertion, fixture, wrong_with_it, outcome}]``, one row per red."""
    rows = []
    for aid in ASSERTIONS:
        for name, spec in FIXTURE_ASSERTION_MAP.items():
            if aid in spec["red"]:
                rows.append({"assertion": aid, "fixture": name,
                             "wrong_with_it": spec["wrong_with_it"],
                             "outcome": spec["outcome"]})
    return rows


def render_coverage_table():
    lines = ["T3 red-evidence coverage -- every assertion, the fixture that "
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
    passers = [n for n, s in FIXTURE_ASSERTION_MAP.items()
               if s["outcome"] == PASS and n != "oracle"]
    lines += ["", "fixtures that must PASS, and what they prove:"]
    for name in passers:
        lines.append("  %-30s %s"
                     % (name, FIXTURE_ASSERTION_MAP[name]["wrong_with_it"]))
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
    v = t3_core.grade(spec["fn"]())
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
        term = v.measurements.get("termination_cause")
        print("  %-30s %-8s red=%-8s support=%-10s method=%-14s "
              "end=%-14s vacuous=%s"
              % (name, v.outcome, ",".join(sorted(v.failed)) or "-",
                 v.measurements.get("support_attestation"),
                 v.measurements.get("method"), term or "-", vac or "-"))
        if problems:
            print("      MISMATCH: %s" % "; ".join(problems))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
