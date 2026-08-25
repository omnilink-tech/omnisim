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

"""T2's negative fixtures -- the red-evidence rule, made executable.

``capability-ladder-plan.md`` §5c.2, binding and standing: *"no assertion
enters a ladder cell until it has been observed FAILING on a deliberately
wrong artifact, with that negative fixture named in the assertion's record. A
green assertion is not evidence that the assertion works ... A ``null`` agent
turning every assertion red does not satisfy this rule."*

That last sentence is what shapes this module, exactly as it shapes
:mod:`ladder.graders.fixtures`. Every fixture below is the :func:`oracle` run
with **one** thing wrong, and :data:`FIXTURE_ASSERTION_MAP` records which
single assertion it must turn red -- so a fixture that reddens two assertions,
or the wrong one, is a test failure.

Two fixtures exist to prove something is *not* graded rather than that it is:
:func:`welded_object` carries ``hold_mechanism: attachment`` and **passes**,
and :func:`mechanism_not_recorded` passes with the mechanism ``unknown``. The
plan is explicit that grading the mechanism would be writing the task around a
technique this tree itself abandoned (§2 T2), so the evidence that it is
recorded-not-graded has to be as executable as the rest.

The synthetic scene, once, so every fixture below can be read against it::

    ground   static, top z = 0.00, spanning the whole scene
    table    static, top z = 0.40, x in [0.20, 0.80], y in [-0.40, 0.40]
    bin      static, x in [-0.75, -0.25], y in [-0.25, 0.25], z in [0, 0.30],
             rim at z = 0.30, inner floor top at z = 0.02
    block    0.05 m cube, 0.2 kg, dynamic; starts resting on the table
    tool     the end effector; holds the block at (0.08, 0, -0.06) in ITS OWN
             frame, and yaws through 90 degrees during the carry

That yaw is not decoration. It is what makes the oracle a test of T2.3's
*frame*: held rigidly, the block traces an 8 cm arc in the world and a single
point in the tool's frame, so the same run reads as a clean hold in the frame
the tier states and as a 2.4 cm RMS slip without the rotation. The verdict
prints both numbers side by side and
``test_t2_core.py`` asserts they disagree.

Two fixtures below are **repair regressions** rather than ordinary negatives --
each is an artifact that used to grade PASS and now does not, kept here so the
defect cannot come back unnoticed.

:func:`at_rest_never_held` is the first: a genuine transfer whose jaws were
closed for only 7.0 s, followed by 19 s in which the arm has parked 0.6 m away
and neither body moves at all. T2.3 measured deviation about a window's own
mean and asked nothing about whether anything was happening, so the still tail
satisfied it -- demonstrated on the MuJoCo achievability run's own final state
at RMS 4.0e-08 m. Its row still prints that reading, beside the graded one, so
what was wrong stays legible.

:func:`started_in_the_container` is the second, and it is the more embarrassing
of the two: the block is lifted **out** of the bin, carried, held rigidly for
twelve seconds and put back, and every one of the five assertions was green.
Nothing was transferred. T2.1 now reads where the object began (``t2_core``
reading 2b), and within T2.1 only that one clause reddens -- the other three
stay green, because the run really does end inside the container, under its
rim, at rest.

Run the coverage table::

    python -m ladder.graders.fixtures_t2

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
from ladder.graders import t2_core  # noqa: E402
from ladder.graders.ladder_evidence import (  # noqa: E402
    BodyPhysics, ContainerGeometry, GripContact, GripObservation, PoseSeries,
    RoleNames, T2Evidence, WorldPhysics, support_surfaces_from_bundle)

DT = 0.016                        # a plausible basic timestep, seconds
G = 9.81

OBJECT_NAME = "block"
EE_NAME = "tool"
CONTAINER_NAME = "bin"
ARM_NAME = "arm"

GROUND_AABB = ((-5.0, -5.0, -0.10), (5.0, 5.0, 0.0))
TABLE_AABB = ((0.20, -0.40, 0.0), (0.80, 0.40, 0.40))
BIN_AABB = ((-0.75, -0.25, 0.0), (-0.25, 0.25, 0.30))
BIN_RIM_Z = 0.30
BIN_FLOOR_TOP_Z = 0.02
# A container sunk flush into the table, used by exactly one fixture so an
# object can be DRAGGED into it without ever being lifted. See never_lifted.
SUNKEN_AABB = ((0.05, -0.15, 0.35), (0.35, 0.15, 0.50))
SUNKEN_RIM_Z = 0.50

BLOCK_HALF = 0.025
OBJECT_MASS_KG = 0.2
TABLE_TOP_Z = 0.40
GRIP_OFFSET = (0.08, 0.0, -0.06)   # where the block sits in the tool's frame

RUN_S = 20.0
GRASP_S = 3.0                      # the jaws close
RELEASE_S = 18.0                   # the jaws open
CARRY_START_S = 5.0
CARRY_END_S = 16.0
PARK_AWAY_M = 0.6                  # how far the arm retreats once it parks

ROLES_SOURCE = "the task's own meta file, read by the grader"
OBJECT_SOURCE = "a synthetic object pose series (a fixture, not a run)"
EE_SOURCE = "a synthetic end-effector pose series (a fixture, not a run)"
CONTAINER_SOURCE = "a synthetic frozen scan of the container (a fixture)"
GRIP_SOURCE = "a synthetic grip scan over the carry window (a fixture)"
MASS_SOURCE = "a synthetic body-mass read at t=0 (a fixture)"
GRAVITY_SOURCE = "a synthetic world-gravity read (a fixture)"

# The path the block takes when everything goes right: rest, lift, carry,
# lower, rest. (t, (x, y, z)) with the z of the block's CENTRE.
BASE_KEYS = ((0.0, (0.55, 0.0, TABLE_TOP_Z + BLOCK_HALF)),
             (GRASP_S, (0.55, 0.0, TABLE_TOP_Z + BLOCK_HALF)),
             (CARRY_START_S, (0.55, 0.0, 0.725)),
             (CARRY_END_S, (-0.50, 0.0, 0.725)),
             (RELEASE_S, (-0.50, 0.0, BIN_FLOOR_TOP_Z + BLOCK_HALF)),
             (RUN_S, (-0.50, 0.0, BIN_FLOOR_TOP_Z + BLOCK_HALF)))


# --- track construction ------------------------------------------------------


def _smoothstep(f):
    return f * f * (3.0 - 2.0 * f)


def _clock(dt=DT, run_s=RUN_S):
    return np.arange(int(round(run_s / dt)) + 1) * dt


def _path(t, keys):
    """Smoothstep interpolation through ``(time, xyz)`` keys."""
    out = np.zeros((len(t), 3), dtype=float)
    keys = list(keys)
    for (t0, p0), (t1, p1) in zip(keys[:-1], keys[1:]):
        m = (t >= t0) & (t <= t1)
        span = max(float(t1) - float(t0), 1e-9)
        s = _smoothstep(np.clip((t[m] - t0) / span, 0.0, 1.0))
        for a in range(3):
            out[m, a] = p0[a] + s * (p1[a] - p0[a])
    out[t < keys[0][0]] = keys[0][1]
    out[t > keys[-1][0]] = keys[-1][1]
    return out


def _yaw_frames(t, t0=CARRY_START_S, t1=CARRY_END_S):
    """World-from-body rotations: a 90 degree yaw across the carry.

    ``t0``/``t1`` default to the shared carry window so every fixture built on
    :data:`BASE_KEYS` is unchanged; a fixture with its own timeline passes its
    own, because a carrier still rotating after it has parked would move the
    object in its frame while nothing physical was happening.
    """
    span = max(t1 - t0, 1e-9)
    th = -0.5 * np.pi * _smoothstep(
        np.clip((t - t0) / span, 0.0, 1.0))
    c, s = np.cos(th), np.sin(th)
    rot = np.zeros((len(t), 3, 3), dtype=float)
    rot[:, 0, 0] = c
    rot[:, 0, 1] = -s
    rot[:, 1, 0] = s
    rot[:, 1, 1] = c
    rot[:, 2, 2] = 1.0
    return rot


def _fall(t, xyz, fall_at, rest_z):
    """Let go at ``fall_at``: free fall, then rest on ``rest_z``."""
    k = int(np.searchsorted(t, float(fall_at)))
    if k >= len(t):
        return xyz
    tau = t[k:] - t[k]
    z = float(xyz[k, 2]) - 0.5 * G * tau * tau
    xyz[k:, 0] = xyz[k, 0]
    xyz[k:, 1] = xyz[k, 1]
    xyz[k:, 2] = np.maximum(z, float(rest_z))
    return xyz


def _tracks(*, keys=BASE_KEYS, hold_start=GRASP_S, hold_end=RELEASE_S,
            fall_at=None, rest_z=BIN_FLOOR_TOP_Z + BLOCK_HALF,
            slip=None, dt=DT, run_s=RUN_S, yaw=None, park_s=None):
    """``(t, object xyz, tool xyz, tool rot)`` for one synthetic attempt.

    ``slip(t)`` returns an (N, 3) perturbation **in the tool's own frame**,
    applied to the object only -- which is how a grasp that slides in the jaws
    is built without also moving the arm that is holding it.

    ``park_s`` makes the arm **stop**: after the jaws open it retreats
    :data:`PARK_AWAY_M` horizontally and 0.30 m up over ``park_s`` seconds and
    then holds exactly still for the rest of the run, instead of drifting
    upward until the clock ends. That stillness is the point -- see
    :func:`at_rest_never_held`.
    """
    t = _clock(dt, run_s)
    obj = _path(t, keys)
    if fall_at is not None:
        obj = _fall(t, obj, fall_at, rest_z)
    rot = _yaw_frames(t, *(yaw or (CARRY_START_S, CARRY_END_S)))

    # The tool is placed from the UNPERTURBED object, and the slip is then
    # added to the object alone. Deriving the tool from the slipped object
    # instead would move the arm with the block and make every slip fixture
    # read as a perfect grasp -- which it did, before this comment existed.
    tool = obj - np.einsum("nij,j->ni", rot, np.asarray(GRIP_OFFSET, float))
    if slip is not None:
        obj = obj + np.einsum("nij,nj->ni", rot, slip(t))

    # Before the jaws close and after they open the tool is on its own path:
    # it comes down to the block and it lifts away. Freezing its horizontal
    # position after release matters -- otherwise a dropped block drags the
    # tool down with it and the two never separate.
    pre = t < hold_start
    if pre.any():
        s = _smoothstep(np.clip(t[pre] / max(hold_start, 1e-9), 0.0, 1.0))
        tool[pre, 2] = tool[pre, 2] + 0.30 * (1.0 - s)
    post = t > hold_end
    if post.any():
        k = int(np.searchsorted(t, float(hold_end)))
        k = min(k, len(t) - 1)
        span = float(park_s if park_s is not None else (run_s - hold_end))
        s = _smoothstep(np.clip((t[post] - hold_end) / max(span, 1e-9),
                                0.0, 1.0))
        tool[post, 0] = tool[k, 0] + (0.0 if park_s is None
                                      else PARK_AWAY_M * s)
        tool[post, 1] = tool[k, 1]
        tool[post, 2] = tool[k, 2] + 0.30 * s
    return t, obj, tool, rot


def _sine_slip(amplitude, freq_hz, t0, t1, axis=0):
    """A perturbation in the tool frame, zero outside ``[t0, t1]``."""
    def slip(t):
        out = np.zeros((len(t), 3), dtype=float)
        m = (t >= t0) & (t <= t1)
        out[m, axis] = amplitude * np.sin(2.0 * np.pi * freq_hz * (t[m] - t0))
        return out
    return slip


# --- the bundle and the channels ---------------------------------------------


def _bodies(container_aabb=BIN_AABB, object_start=None, mass_model=True,
            object_bounded=True):
    start = object_start or BASE_KEYS[0][1]
    block = Body(
        body_id="#21", name=OBJECT_NAME, kind="Solid",
        position=tuple(float(v) for v in start),
        aabb_min=((start[0] - BLOCK_HALF, start[1] - BLOCK_HALF,
                   start[2] - BLOCK_HALF) if object_bounded else None),
        aabb_max=((start[0] + BLOCK_HALF, start[1] + BLOCK_HALF,
                   start[2] + BLOCK_HALF) if object_bounded else None),
        dynamic=bool(mass_model), robot_class=False,
        identity_evidence="synthetic fixture")
    arm = Body(body_id="#7", name=ARM_NAME, kind="Robot",
               position=(0.0, 0.0, 0.0), aabb_min=(-0.2, -0.2, 0.0),
               aabb_max=(0.2, 0.2, 1.0), n_joints=8, dynamic=True,
               robot_class=True, behaviour="move", behaviour_declared="move",
               identity_evidence="synthetic fixture")
    ground = Body(body_id="#2", name="ground", kind="Solid",
                  aabb_min=GROUND_AABB[0], aabb_max=GROUND_AABB[1],
                  dynamic=False, robot_class=False,
                  identity_evidence="synthetic fixture")
    table = Body(body_id="#3", name="table", kind="Solid",
                 aabb_min=TABLE_AABB[0], aabb_max=TABLE_AABB[1],
                 dynamic=False, robot_class=False,
                 identity_evidence="synthetic fixture")
    box = Body(body_id="#4", name=CONTAINER_NAME, kind="Solid",
               aabb_min=container_aabb[0], aabb_max=container_aabb[1],
               dynamic=False, robot_class=False,
               identity_evidence="synthetic fixture")
    return [arm, block], [ground, table, box]


def _bundle(*, attribution=True, error_lines=(), finalize=True, exit_code=0,
            timed_out=False, deliverable="/synthetic/T2_transfer/run.txt",
            container_aabb=BIN_AABB, object_start=None, mass_model=True,
            object_bounded=True):
    movers, statics = _bodies(container_aabb, object_start, mass_model,
                              object_bounded)
    return EvidenceBundle(
        task=t2_core.TASK, sim="synthetic", adapter="ladder.fixtures_t2",
        artifact=deliverable,
        identity=IdentityRule(
            label="arm",
            requirement="the arm the description in the container declares",
            scene_rule="synthetic: robot_class is set on the fixture",
            declaration_rule="synthetic: not counted", declared_count=1),
        roster=BodyInventory(bodies=list(movers), frozen=True, t_s=0.0,
                             source="synthetic frozen scan"),
        t0=BodyInventory(bodies=list(movers) + list(statics), frozen=True,
                         t_s=0.0, source="synthetic frozen scan"),
        process=ProcessFacts(
            exit_code=exit_code, timed_out=timed_out,
            error_lines=list(error_lines), log_available=True,
            log_source="synthetic error stream", behaviour_starts={"move": 1},
            start_source="synthetic process table",
            driver_completed=bool(finalize), reached_finalize=bool(finalize),
            finalize_evidence=("the driver reached its target simulated time"
                               if finalize else "none"),
            wall_s=61.0),
        attribution=(EngineAttribution(
            backend="synthetic", solver="synthetic-pgs",
            source="a synthetic fixture, not a run") if attribution else None))


def _grip(t, *, mechanism="friction", attachment=False, every=40,
          first=None, last=None, source=GRIP_SOURCE, contacts=True):
    n = len(t)
    lo = int(np.searchsorted(t, GRASP_S)) if first is None else int(first)
    hi = int(np.searchsorted(t, RELEASE_S)) if last is None else int(last)
    out = []
    if contacts:
        for i in range(lo, min(hi, n), every):
            for jaw in ("left_finger", "right_finger"):
                out.append(GripContact(holder_body=jaw,
                                       held_body=OBJECT_NAME,
                                       t_s=float(t[i]), step=i))
    return GripObservation(
        mechanism=mechanism,
        mechanism_source=("the fixture declares it; a real column reads it "
                          "from what actually binds the two bodies"),
        contacts=out, attachment=attachment, supported=True,
        total_observed=(len(out) * 3 if out else 0),
        distinct_named=(len(out) if out else 0),
        steps_sampled=max(0, (hi - lo) // max(1, every)),
        window_s=float(t[-1] - t[0]) if n else None, source=source)


def _evidence(t, obj_xyz, tool_xyz, tool_rot, *, container_aabb=BIN_AABB,
              rim_z=BIN_RIM_Z, mass_kg=OBJECT_MASS_KG, mass_model=True,
              gravity=G, orientation=True, grip=None, object_bounded=True,
              **kw):
    bundle = _bundle(container_aabb=container_aabb, mass_model=mass_model,
                     object_bounded=object_bounded, **kw)
    block = bundle.t0.by_name(OBJECT_NAME)
    return T2Evidence(
        bundle=bundle,
        roles=RoleNames(object_name=OBJECT_NAME, end_effector_name=EE_NAME,
                        container_name=CONTAINER_NAME, source=ROLES_SOURCE),
        object_pose=PoseSeries(body=OBJECT_NAME, t=t, xyz=obj_xyz,
                               source=OBJECT_SOURCE),
        end_effector=PoseSeries(body=EE_NAME, t=t, xyz=tool_xyz,
                                rot=(tool_rot if orientation else None),
                                source=EE_SOURCE),
        container=ContainerGeometry(
            body=CONTAINER_NAME, aabb_min=container_aabb[0],
            aabb_max=container_aabb[1], rim_z=rim_z,
            rim_rule=("the height of the opening, measured on the container "
                      "the task shipped"),
            t_s=float(t[-1]) if len(t) else None, source=CONTAINER_SOURCE),
        grip=(grip if grip is not None else _grip(t)),
        object_physics=BodyPhysics(
            body=OBJECT_NAME, mass_kg=mass_kg, dynamic=bool(mass_model),
            source=MASS_SOURCE),
        world=WorldPhysics(gravity_mps2=gravity,
                           gravity_vec=(0.0, 0.0, -gravity),
                           source=GRAVITY_SOURCE),
        support_surfaces=support_surfaces_from_bundle(bundle),
        object_aabb=((block.aabb_min, block.aabb_max)
                     if (block is not None and block.has_aabb) else None))


# --- the fixtures ------------------------------------------------------------


def oracle():
    """A block lifted off a table, carried, held, and set down in the bin."""
    return _evidence(*_tracks())


def never_lifted():
    """T2.2: dragged the whole way, and it still ends up inside.

    The container here is sunk flush into the table so the object can slide in
    without ever leaving the surface -- which is the only way to redden the
    lift clause WITHOUT also reddening the transfer clause. A fixture that
    simply stopped short of the bin would fail T2.1 as well and stop isolating
    anything.
    """
    z = TABLE_TOP_Z + BLOCK_HALF
    keys = ((0.0, (0.70, 0.0, z)), (GRASP_S, (0.70, 0.0, z)),
            (CARRY_START_S, (0.70, 0.0, z)), (CARRY_END_S, (0.20, 0.0, z)),
            (RELEASE_S, (0.20, 0.0, z)), (RUN_S, (0.20, 0.0, z)))
    return _evidence(*_tracks(keys=keys), container_aabb=SUNKEN_AABB,
                     rim_z=SUNKEN_RIM_Z)


def dropped_en_route():
    """T2.1: lifted and held, then dropped on the floor short of the bin."""
    keys = ((0.0, BASE_KEYS[0][1]), (GRASP_S, BASE_KEYS[1][1]),
            (CARRY_START_S, (0.55, 0.0, 0.725)),
            (CARRY_END_S, (-0.10, 0.0, 0.725)),
            (RELEASE_S, (-0.10, 0.0, 0.725)), (RUN_S, (-0.10, 0.0, 0.725)))
    return _evidence(*_tracks(keys=keys, hold_end=CARRY_END_S,
                              fall_at=CARRY_END_S, rest_z=BLOCK_HALF))


def beside_the_container():
    """T2.1: the whole motion, and then set down 0.15 m beside the bin."""
    keys = ((0.0, BASE_KEYS[0][1]), (GRASP_S, BASE_KEYS[1][1]),
            (CARRY_START_S, (0.55, 0.0, 0.725)),
            (CARRY_END_S, (-0.15, 0.0, 0.725)),
            (RELEASE_S, (-0.15, 0.0, BLOCK_HALF)),
            (RUN_S, (-0.15, 0.0, BLOCK_HALF)))
    return _evidence(*_tracks(keys=keys))


def released_above_the_rim():
    """T2.1: opened the jaws over the bin and the clock ran out mid-fall.

    All three of T2.1's clauses are red here and that is the point: the tier
    asks for an object that ended up IN the container and stopped, and a
    body still accelerating downward above the rim has done neither. It is
    the distinction the "at rest" window exists to draw.
    """
    keys = ((0.0, BASE_KEYS[0][1]), (GRASP_S, BASE_KEYS[1][1]),
            (CARRY_START_S, (0.55, 0.0, 0.725)),
            (CARRY_END_S, (-0.50, 0.0, 0.725)),
            (19.85, (-0.50, 0.0, 0.725)), (RUN_S, (-0.50, 0.0, 0.725)))
    return _evidence(*_tracks(keys=keys, hold_end=19.85, fall_at=19.85,
                              rest_z=-10.0))


def held_three_seconds():
    """T2.3: the grip is only steady for 3 s, and it still makes the bin.

    The object slides in the jaws from t = 8 s on. It is delivered anyway --
    which is what keeps this fixture about the hold and not about the
    transfer.
    """
    return _evidence(*_tracks(slip=_sine_slip(0.06, 0.5, 8.0, 17.0)))


def held_but_jittering():
    """T2.3: held for the full carry, 2.3 cm RMS of it, which is over.

    Deliberately tuned to break the RMS bound and NOT the excursion bound
    (amplitude 0.033 m: RMS 0.023 > 0.02, peak 0.033 < 0.04), so the fixture
    is evidence that the RMS clause is doing work on its own.
    """
    return _evidence(*_tracks(slip=_sine_slip(0.033, 0.7, 4.0, 17.0)))


def at_rest_never_held():
    """T2.3: the ten quiet seconds are AFTER the arm let go and parked.

    **The fixture that would have caught the vacuous green.** T2.3 measured
    the object's position in the end-effector's frame about a window's own
    mean, and nothing required anything to be happening in that window -- so a
    scene at rest satisfied it. Demonstrated on the MuJoCo achievability run's
    own final state (block on the bin floor, jaws open, arm parked 0.6 m away,
    nothing in contact): RMS 4.0e-08 m, max excursion 3.8e-15 m, PASS.

    So this run does everything right EXCEPT the hold: the block is picked up,
    carried, delivered and released after **7.0 s** in the jaws -- three
    seconds short -- and the arm then retreats 0.6 m and **stops**, leaving
    19 s in which neither body moves at all. The old predicate found its ten
    seconds in that tail. The repaired one does not, because at every sample
    of it the block is sitting motionless on the bin floor.

    It is the oracle with one thing wrong, and the one thing is the length of
    the grip. T2.1, T2.2, T2.4 and T2.5 are all satisfied -- the transfer
    genuinely happened -- which is what keeps this fixture about T2.3 and not
    about a run that failed everywhere.
    """
    run_s, grasp, release = 30.0, 2.0, 9.0
    z_table = TABLE_TOP_Z + BLOCK_HALF
    z_bin = BIN_FLOOR_TOP_Z + BLOCK_HALF
    keys = ((0.0, (0.55, 0.0, z_table)), (grasp, (0.55, 0.0, z_table)),
            (4.0, (0.55, 0.0, 0.725)), (7.0, (-0.50, 0.0, 0.725)),
            (release, (-0.50, 0.0, z_bin)), (run_s, (-0.50, 0.0, z_bin)))
    t, obj, tool, rot = _tracks(keys=keys, hold_start=grasp,
                                hold_end=release, run_s=run_s,
                                yaw=(grasp, 7.0), park_s=2.0)
    return _evidence(t, obj, tool, rot,
                     grip=_grip(t, first=int(np.searchsorted(t, grasp)),
                                last=int(np.searchsorted(t, release))))


def started_in_the_container():
    """T2.1: lifted OUT of the bin, held twelve seconds, and put back.

    **The fixture that would have caught the transfer that never happened.**
    Until T2.1 read where the object began, this artifact graded **PASS 5/5**:
    it ends inside the container (T2.1's other three clauses are all satisfied
    -- inside, under the rim, at rest), it is carried clear for far longer than
    T2.2 asks, it is held rigidly in the tool's frame through a 90 degree yaw
    for longer than T2.3 asks, it has mass under gravity and the run is real.
    Nothing was transferred anywhere. The tier's OUTCOME sentence says the
    object *starts on a surface*, so the intent was never in doubt -- only the
    graded rows were silent.

    It is the oracle with one thing wrong, and the one thing is where the block
    was at t = 0. Everything the block does after t = 2 s is a textbook carry,
    which is what keeps this fixture about the new clause and not about a run
    that failed everywhere: within T2.1, the three original clauses stay green
    and only "started outside the container" reddens.
    """
    run_s, grasp, release = 30.0, 2.0, 20.0
    z_air = 0.725
    z_bin = BIN_FLOOR_TOP_Z + BLOCK_HALF
    start = (-0.50, 0.0, z_bin)
    keys = ((0.0, start), (grasp, start), (5.0, (-0.50, 0.0, z_air)),
            (17.0, (-0.50, 0.0, z_air)), (release, start), (run_s, start))
    t, obj, tool, rot = _tracks(keys=keys, hold_start=grasp,
                                hold_end=release, run_s=run_s,
                                yaw=(5.0, 17.0), park_s=2.0)
    return _evidence(t, obj, tool, rot, object_start=start,
                     grip=_grip(t, first=int(np.searchsorted(t, grasp)),
                                last=int(np.searchsorted(t, release))))


def no_end_effector_orientation():
    """T2.3 red AND vacuous: the carrier's pose has no rotation in it.

    A realistic partial column, and the one that matters most here: with
    position only, "in the end-effector's frame" is not measurable at all, so
    the clause reports itself vacuous and the assertion is red because the
    tier's requirement was never met. The blocker for a cell in this state is
    ours, not the simulator's.
    """
    return _evidence(*_tracks(), orientation=False)


def massless_prop():
    """T2.4: the anti-cheat -- a massless, non-dynamic prop flown to the bin.

    Its kinematics are the oracle's, exactly. It is FLOWN rather than dragged
    on purpose: a dragged prop would redden the lift clause as well and stop
    isolating T2.4, and "flown" is the word the tier itself uses for what this
    assertion exists to catch.
    """
    return _evidence(*_tracks(), mass_kg=0.0, mass_model=False)


def teleported_object():
    """T2.4: one 0.9 m inter-sample jump, and the tool jumps with it.

    The tool moves with the object, so the grip still reads as rigid and T2.3
    stays green -- which is exactly the artifact a frame-relative hold check
    cannot catch and a motion-plausibility clause can.
    """
    t, obj, tool, rot = _tracks()
    k = int(np.searchsorted(t, 10.0))
    m = int(np.searchsorted(t, 12.0))
    shift = np.zeros_like(obj)
    shift[k:m, 1] = 0.9 * (1.0 - _smoothstep(
        np.linspace(0.0, 1.0, m - k)))
    shift[k, 1] = 0.9
    return _evidence(t, obj + shift, tool + shift, rot)


def welded_object():
    """PASS, mechanism ``attachment``: the object was bound, not gripped.

    Not a negative. The plan forbids grading the mechanism (§2 T2), so the
    evidence that an attachment-based transfer PASSES -- with the mechanism
    printed in the cell -- has to be as executable as every red above.
    """
    t, obj, tool, rot = _tracks()
    return _evidence(t, obj, tool, rot,
                     grip=_grip(t, mechanism="attachment", attachment=True,
                                contacts=False))


def mechanism_not_recorded():
    """PASS, mechanism ``unknown``: nothing said how it was held.

    Also not a negative. A column with no grip channel still produces a
    gradeable cell; what it may not do is produce one with the mechanism
    field missing, because no prose sentence about a T2 cell may omit it.
    """
    t, obj, tool, rot = _tracks()
    return _evidence(t, obj, tool, rot,
                     grip=GripObservation(
                         mechanism="", supported=False,
                         error="no grip channel on this column"))


def unbounded_object():
    """PASS with a vacuous clause: the object's own size never arrived.

    The centre is graded instead of the lowest point, which cannot catch a
    block balanced on the rim -- so the row says the clause proved less than
    it looks like it proved.
    """
    return _evidence(*_tracks(), object_bounded=False)


def no_attested_rim():
    """PASS with a vacuous clause: the container's box, but no rim height.

    Exactly what the frozen inventory can answer on its own, so it is the
    state most columns will actually be in until a ladder-side sampler ships
    a rim. The permissive box top is used, the row says so, and the clause
    that would have caught a block perched on the rim reports that it could
    not have caught one.
    """
    return _evidence(*_tracks(), rim_z=None)


def never_finalized():
    """T2.5: the world never finished building. Exit code still 0."""
    return _evidence(*_tracks(), finalize=False)


def error_lines():
    """T2.5: the engine wrote an error-class line during the run."""
    return _evidence(*_tracks(), error_lines=["ERROR: a fixture's error line"])


def missing_attribution():
    """T2.5 red, verdict INVALID: nothing names the engine that drove it."""
    return _evidence(*_tracks(), attribution=False)


# --- the map ----------------------------------------------------------------
#
# fixture -> what it must do to the verdict. Read by test_t2_core.py, printed
# by coverage_table(). Both columns are EXACT, for the reason fixtures.py
# gives at the same place: a fixture that reddens something not listed here
# has stopped isolating one assertion.

_C_STARTED_OUTSIDE = "started outside the container"
_C_INSIDE = "ended inside the container"
_C_UNDER_RIM = "under the rim, not perched on it"
_C_AT_REST = "came to rest"
_C_CLEARED = "cleared the surface"
_C_TEN_SECONDS = "held for ten seconds"
_C_FRAME = "measured in the carrier's own frame"
_C_HAS_MASS = "has mass"

FIXTURE_ASSERTION_MAP = {
    "oracle": {
        "fn": oracle, "red": (), "outcome": PASS, "vacuous": {},
        "wrong_with_it": "nothing -- this is the positive control"},
    "never_lifted": {
        "fn": never_lifted, "red": ("T2.2",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "dragged into a sunken container, never lifted"},
    "dropped_en_route": {
        "fn": dropped_en_route, "red": ("T2.1",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "lifted and held, then dropped short of the bin"},
    "beside_the_container": {
        "fn": beside_the_container, "red": ("T2.1",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "set down 0.15 m beside the bin, not in it"},
    "released_above_the_rim": {
        "fn": released_above_the_rim, "red": ("T2.1",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "released over the bin; still falling at the end"},
    "started_in_the_container": {
        "fn": started_in_the_container, "red": ("T2.1",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "lifted OUT of the bin, held 12 s and put back -- a "
                         "flawless carry in which nothing was transferred"},
    "held_three_seconds": {
        "fn": held_three_seconds, "red": ("T2.3",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "steady for 3 s, then slides in the jaws"},
    "held_but_jittering": {
        "fn": held_but_jittering, "red": ("T2.3",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "held the whole way at 2.3 cm RMS, over the bound"},
    "at_rest_never_held": {
        "fn": at_rest_never_held, "red": ("T2.3",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "delivered, but the jaws were only closed 7 s; the "
                         "ten still seconds are the parked tail afterwards"},
    "no_end_effector_orientation": {
        "fn": no_end_effector_orientation, "red": ("T2.3",), "outcome": FAIL,
        "vacuous": {"T2.3": (_C_FRAME,)},
        "wrong_with_it": "the carrier's rotation never reached the grader"},
    "massless_prop": {
        "fn": massless_prop, "red": ("T2.4",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "a massless, non-dynamic prop flown to the bin"},
    "teleported_object": {
        "fn": teleported_object, "red": ("T2.4",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "one 0.9 m inter-sample jump mid-carry"},
    "never_finalized": {
        "fn": never_finalized, "red": ("T2.5",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "the world never finished building; exit code 0"},
    "error_lines": {
        "fn": error_lines, "red": ("T2.5",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "an error-class line during the run"},
    "missing_attribution": {
        "fn": missing_attribution, "red": ("T2.5",), "outcome": INVALID,
        "vacuous": {},
        "wrong_with_it": "nothing names the engine that drove the run"},
    "welded_object": {
        "fn": welded_object, "red": (), "outcome": PASS, "vacuous": {},
        "wrong_with_it": "nothing gradeable -- the object was bound rather "
                         "than gripped, which is RECORDED and never graded"},
    "mechanism_not_recorded": {
        "fn": mechanism_not_recorded, "red": (), "outcome": PASS,
        "vacuous": {},
        "wrong_with_it": "nothing gradeable -- no grip channel, so the "
                         "mechanism is 'unknown' and the cell still stands"},
    "unbounded_object": {
        "fn": unbounded_object, "red": (), "outcome": PASS,
        "vacuous": {"T2.1": (_C_UNDER_RIM,)},
        "wrong_with_it": "nothing physical -- the object's own size never "
                         "arrived, so the centre is graded instead of its "
                         "lowest point"},
    "no_attested_rim": {
        "fn": no_attested_rim, "red": (), "outcome": PASS,
        "vacuous": {"T2.1": (_C_UNDER_RIM,)},
        "wrong_with_it": "nothing physical -- no rim was attested, so the "
                         "permissive box top was used and 'perched on the "
                         "rim' could not have been caught"},
}

# Every assertion the tier states must appear as some fixture's target.
ASSERTIONS = tuple(aid for aid, _what in t2_core._ALL)


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
    lines = ["T2 red-evidence coverage -- every assertion, the fixture that "
             "turns it red",
             "",
             "%-7s  %-28s  %-9s  %s" % ("assert", "fixture", "outcome",
                                        "what is wrong with the artifact"),
             "%-7s  %-28s  %-9s  %s" % ("-" * 7, "-" * 28, "-" * 9, "-" * 46)]
    for r in coverage_table():
        lines.append("%-7s  %-28s  %-9s  %s"
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
        lines.append("  %-28s %s" % (name,
                                     FIXTURE_ASSERTION_MAP[name]
                                     ["wrong_with_it"]))
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
    v = t2_core.grade(spec["fn"]())
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
        print("  %-28s %-8s red=%-8s mech=%-11s vacuous=%-34s %s"
              % (name, v.outcome, ",".join(sorted(v.failed)) or "-",
                 v.measurements.get("hold_mechanism"), vac or "-",
                 "ok" if not problems else "MISMATCH: "
                 + "; ".join(problems)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
