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

"""spec.py -- the ladder's SCENE CONSTANTS and its ANALYTIC GROUND TRUTH.

Everything in this file is derived from first principles: Newtonian mechanics
plus the geometry the world files declare.  **No expected value in this module
was ever read out of a running simulator**, and none may be.  A golden captured
from today's behaviour would have certified every engine defect this repo found
in the week before the ladder was written (a phantom z=0 collision plane that
caught bodies which should have fallen; ``setVelocity`` silently ignored after
world finalize; wheels that did not rotate while the chassis slid the right
distance).

The scene constants are the SINGLE source of truth for all three arms:
``worldgen.py`` emits the OmniSim ``.wbt``, the upstream-Webots ``.wbt`` and the
MuJoCo MJCF from these numbers, so a change here changes all three scenes
together and can never desynchronise the expectation from the scene.

Every assertion is on a PHYSICAL QUANTITY in SI units -- a rest height in
metres, a fall interval in seconds, an angular rate in rad/s.  No assertion
mentions an engine API, a node type, a field name or a log line, so the same
check runs unmodified against all three simulators.

TOLERANCES
----------
Each tolerance below carries its physical derivation in the constant's comment.
The rule is: a tolerance may be justified by integrator order, contact
compliance, sampling rate or Coulomb slip -- never by "what happens to pass".
If a measured value lands outside its tolerance the row is RED and the engine
owes an explanation; the row always reports the measured value, the expected
value and the signed margin so the explanation can be checked.
"""

from __future__ import annotations

import math

# --------------------------------------------------------------------------
# Scene constants -- shared by all three arms and all five rungs.
# --------------------------------------------------------------------------

G = 9.81                     # m/s^2, declared explicitly in every scene
DT = 0.004                   # s, basic timestep declared in every scene
MU = 1.0                     # Coulomb friction, declared in every scene

# A floor whose top is NOT at z=0.  This is deliberate: an implicit ground
# plane at z=0 (a real defect this engine shipped) is invisible to any scene
# whose floor already sits at z=0, because the phantom plane and the authored
# floor then coincide.  Lifting the floor to z=0.5 makes the two separable.
# SIZE IS LOAD-BEARING -- DO NOT SHRINK THIS FLOOR.  Rung 4 drives for
# RUNG4_DURATION at RUNG4_OMEGA_CMD * WHEEL_R, and its leading wheel runs
# ahead of the robot origin by WHEEL_X + WHEEL_R, so the floor's half-extent
# must be at least
#     v * T + WHEEL_X + WHEEL_R = 0.4 * 6.5 + 0.2 + 0.1 = 2.9 m.
# The 4 x 4 m floor this started with put the lip at x = +2.0, which the rover
# reached at t ~ 4.9 s -- INSIDE the 2-6 s measurement window.  It then drove
# off, and the failure signature was "distance short, wheels turning,
# roll_ratio 0.73", which reads exactly like a traction defect and is not one.
#
# 2.9 m is the requirement for an engine that ROLLS CORRECTLY, and that is not
# the number to size the floor by.  A rover that runs off the lip produces a
# DIFFERENT failure signature -- beached, wheels spinning, zero motion -- and
# that signature masks whatever real defect pushed it there.  Measured on
# OmniSim 2026-08-12: a launch-transient overrun gains the rover 1.37 m of
# free distance in its first second, so it reached x = 4.27 m (body + wheel
# lead) and fell off an 8 x 8 floor at t = 5.2 s, and the run reported the
# fall rather than the overrun.  The half-extent below is 10 m: more than 3x
# what a correct engine needs, so that a BROKEN engine's error is still
# measured as that error.
FLOOR_SIZE = (20.0, 20.0, 0.2)           # full extents, m
FLOOR_CENTER_Z = 0.4                     # m
FLOOR_TOP = FLOOR_CENTER_Z + FLOOR_SIZE[2] / 2.0        # = 0.5 m

BOX_EDGE = 0.2                           # m, cube
BOX_HALF = BOX_EDGE / 2.0                # = 0.1 m
BOX_MASS = 1.0                           # kg

# Rung 1: authored exactly at the analytic rest height.
REST_Z = FLOOR_TOP + BOX_HALF            # = 0.6 m
RUNG1_SPAWN_Z = REST_Z
RUNG1_DURATION = 2.0                     # s
RUNG1_SETTLE_WINDOW = 0.5                # s -- the mean is taken over the tail

# Rung 2: dropped from a height whose free-fall distance to contact is 1.0 m.
RUNG2_SPAWN_Z = 1.6                      # m  (bottom at 1.5, floor top 0.5)
RUNG2_DROP_M = RUNG2_SPAWN_Z - BOX_HALF - FLOOR_TOP     # = 1.0 m
RUNG2_DURATION = 3.0                     # s
RUNG2_SETTLE_WINDOW = 0.5                # s
# Two heights the box crosses during free fall.  The fall INTERVAL between them
# is the primary assertion because it is invariant to when the controller's
# clock starts relative to the engine's -- see fall_interval_s() below.
RUNG2_GATE_HI = 1.2                      # m, box centre (0.4 m of fall done)
RUNG2_GATE_LO = REST_Z                   # m, box centre at first contact

# Rung 3: one hinge, motor-driven, about a VERTICAL axis so gravity exerts no
# torque about it, with no floor in the scene -- a genuinely unloaded joint.
RUNG3_LINK_MASS = 0.5                    # kg
RUNG3_LINK_LEN = 0.4                     # m
RUNG3_LINK_RADIUS = 0.02                 # m
RUNG3_HINGE_Z = 0.5                      # m
RUNG3_OMEGA_CMD = 2.0                    # rad/s
RUNG3_SPINUP = 0.5                       # s, discarded
RUNG3_WIN_A = (0.5, 1.5)                 # s, measured at omega = RUNG3_OMEGA_CMD
RUNG3_ZERO_AT = 1.5                      # s, command switches to 0
RUNG3_WIN_B = (2.0, 3.0)                 # s, measured at omega = 0
RUNG3_DURATION = 3.0                     # s

# Rung 4: a wheeled robot driving in a straight line on the same floor.
#
# ! WHAT THIS RUNG DOES NOT PROVE.  ``wheel_omega`` and ``rolling_consistency``
# establish that the wheels turn CONSISTENTLY WITH THE MOTION.  Nothing here
# establishes that they PROPELLED it.  Measured on the MuJoCo arm 2026-08-12:
# cutting the wheel motors entirely still passed rung 4 at 4.0006 rad/s,
# because an undriven wheel dragged over mu=1 ground rolls at exactly v/r.
# The rung's real content is "the body and its wheels are kinematically
# consistent, over the whole run, and the body never outruns them" -- which is
# the property the 33 shipped slide-not-roll worlds violated.  Proving
# propulsion needs a different signal (motor torque, or a zero-friction
# control in which a driven robot must NOT move); it is not asserted here and
# must not be claimed.
#
# FOUR driven wheels, not two plus a caster.  A passive caster needs a
# LOW-friction contact next to a HIGH-friction one, and Newton exposes exactly
# one global friction value (``newtonGroundMu``) -- the scene simply cannot
# express it, and a caster sliding at mu=1.0 would inject an unbounded,
# engine-specific drag term into a ground truth that is supposed to be
# analytic.  Four symmetric driven wheels keep every ground contact a rolling
# one, which is the invariant the rung exists to test.
WHEEL_R = 0.1                            # m
WHEEL_W = 0.05                           # m, cylinder length along the axle
WHEEL_MASS = 0.5                         # kg each
CHASSIS_SIZE = (0.6, 0.4, 0.15)          # m, full extents
CHASSIS_MASS = 5.0                       # kg
WHEEL_X = 0.2                            # m, |x| offset of each axle
WHEEL_Y = CHASSIS_SIZE[1] / 2.0 + WHEEL_W / 2.0         # = 0.225 m
ROBOT_Z = FLOOR_TOP + WHEEL_R            # = 0.6 m, axle height at rest
RUNG4_OMEGA_CMD = 4.0                    # rad/s -> v = 0.4 m/s
RUNG4_WIN = (2.0, 6.0)                   # s, measured AFTER the spin-up
RUNG4_DURATION = 6.5                     # s

# (tag, sign of x offset, sign of y offset).  Every arm must use these tags:
# they are the keys of ``wheel_q`` in the sample document.
WHEELS = (("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1))
WHEEL_TAGS = tuple(t for t, _, _ in WHEELS)

RUNG0_DURATION = 1.0                     # s
RUNG0_STEPS = int(round(RUNG0_DURATION / DT))           # = 250


# --------------------------------------------------------------------------
# Rung 5 -- a distance sensor facing a wall, swept toward it.
# --------------------------------------------------------------------------
#
# WHY THE CARRIER IS SWEPT KINEMATICALLY AND NOT DRIVEN.  Rungs 1-4 already
# measure dynamics.  Rung 5 measures SENSING, and a sensor rung whose sensor
# pose is produced by a wheel model cannot separate "the ray is wrong" from
# "the rover did not get where it was told".  The carrier here is a body with
# no physics whose pose the driver WRITES on a known schedule, so the sensor's
# position at every instant is a scene fact rather than a simulation result.
# Rung 6 is the same sensor under real locomotion, which is where the two are
# put back together.
#
# WHY IT DWELLS BEFORE IT SWEEPS.  The three engines do not agree on whether a
# pose written during step k is visible to a sensor read at step k or at step
# k+1.  A quarter-second dwell at the start (and a park at the end) makes the
# static readings insensitive to that ordering, so ``range_static`` and
# ``range_final`` measure the RAY and not the arm's write/read interleave.
RUNG5_X0 = 0.0                           # m, carrier origin at t=0
RUNG5_CARRIER_EDGE = 0.1                 # m, cube; the sensor clears it
RUNG5_SENSOR_DX = 0.1                    # m, sensor ahead of the carrier origin
RUNG5_SENSOR_Z = FLOOR_TOP + 0.3         # = 0.8 m, a horizontal ray
RUNG5_WALL_SIZE = (0.2, 4.0, 1.6)        # full extents, m
RUNG5_WALL_FACE_X = 3.0                  # m, the wall's NEAR face
RUNG5_WALL_CENTER_X = RUNG5_WALL_FACE_X + RUNG5_WALL_SIZE[0] / 2.0   # = 3.1
RUNG5_WALL_CENTER_Z = FLOOR_TOP + RUNG5_WALL_SIZE[2] / 2.0           # = 1.3
RUNG5_MAX_RANGE = 5.0                    # m, the sensor's declared span
RUNG5_SWEEP_V = 0.5                      # m/s
RUNG5_T_DWELL = 0.25                     # s parked at x0 before the sweep
RUNG5_T_SWEEP = 4.0                      # s of sweeping
RUNG5_T_PARK = 0.5                       # s parked at the far end
RUNG5_DURATION = RUNG5_T_DWELL + RUNG5_T_SWEEP + RUNG5_T_PARK        # = 4.75
RUNG5_TRAVEL = RUNG5_SWEEP_V * RUNG5_T_SWEEP                         # = 2.0 m
RUNG5_STANDOFF = RUNG5_WALL_FACE_X - (RUNG5_X0 + RUNG5_SENSOR_DX)    # = 2.9 m
RUNG5_FINAL_RANGE = RUNG5_STANDOFF - RUNG5_TRAVEL                    # = 0.9 m
# Windows in which the carrier is provably stationary, so the reading there is
# a pure geometric fact.  Each stops one dwell/park short of the transition.
RUNG5_DWELL_WIN = (0.0, RUNG5_T_DWELL - 0.05)                        # (0, 0.2)
RUNG5_PARK_WIN = (RUNG5_DURATION - RUNG5_T_PARK + 0.05, RUNG5_DURATION)


def rung5_x_cmd(t):
    """Commanded carrier x at simulated time ``t``.  All three arms drive this.

    Dwell, then a constant-velocity sweep, then park.  Saturating at both ends
    is what makes ``range_static``/``range_final`` independent of the engine's
    write/read ordering.
    """
    return RUNG5_X0 + RUNG5_SWEEP_V * min(max(t - RUNG5_T_DWELL, 0.0),
                                          RUNG5_T_SWEEP)


def rung5_expected_range(x_sensor):
    """Geometric range from a sensor at ``x_sensor`` to the wall's near face."""
    return RUNG5_WALL_FACE_X - x_sensor


# --------------------------------------------------------------------------
# Rung 6 -- drive forward, stop when the sensor reads below a threshold.
# --------------------------------------------------------------------------
#
# The rung-4 rover, unchanged, with a forward-facing sensor at the same height
# as its axle.  It drives at the rung-4 command until the sensor first reads
# below RUNG6_STOP_GAP, then every wheel is commanded to zero.
#
# THE FINAL GAP IS MEASURED FROM THE POSE, NOT FROM THE SENSOR.  A sensor that
# is frozen, offset or fabricated would otherwise report the gap it was
# supposed to produce and the rung would grade the sensor with the sensor.
RUNG6_WALL_FACE_X = 3.0                  # m
RUNG6_WALL_SIZE = RUNG5_WALL_SIZE        # the same wall, so a range that is
RUNG6_WALL_CENTER_X = RUNG6_WALL_FACE_X + RUNG6_WALL_SIZE[0] / 2.0
RUNG6_WALL_CENTER_Z = FLOOR_TOP + RUNG6_WALL_SIZE[2] / 2.0
# ... right in rung 5 and wrong in rung 6 cannot be blamed on the target.
RUNG6_SENSOR_DX = CHASSIS_SIZE[0] / 2.0 + 0.02       # = 0.32 m, 20 mm proud
RUNG6_SENSOR_Z = ROBOT_Z                 # = 0.6 m, axle height
RUNG6_STOP_GAP = 0.5                     # m, the commanded threshold
RUNG6_DURATION = 8.0                     # s
RUNG6_SETTLE_WINDOW = 1.0                # s, the tail in which it must be stopped
RUNG6_START_GAP = RUNG6_WALL_FACE_X - RUNG6_SENSOR_DX                # = 2.68 m
RUNG6_APPROACH_M = RUNG6_START_GAP - RUNG6_STOP_GAP                  # = 2.18 m

# HOW FAR PAST THE TRIGGER THE ROVER MAY LEGITIMATELY GO.  Two terms, both
# derived from the scene:
#
#  * LATENCY.  The gap is sampled once per basic step, the decision is taken on
#    that sample and the command lands on the next step: at most three steps of
#    travel at the cruise speed, 3 v dt = 4.8 mm.
#  * BRAKING.  Commanded to zero, each wheel's servo can develop maxTorque/R at
#    the contact, which for this rover is far more force than the ground can
#    transmit, so the deceleration is FRICTION limited at a = mu g and the
#    braking distance is v^2/(2 mu g) = 8.2 mm.  That is a LOWER bound (maximum
#    deceleration); a real contact reaches full braking force over a few solver
#    steps rather than instantly, so the bound carries a factor of 3.
#
# The sum, 29 mm, is 5.9% of the 0.5 m threshold and 17x smaller than the
# failure it exists to catch -- a rover that ignores the sensor travels the
# whole remaining 0.5 m into the wall.
RUNG6_LATENCY_STEPS = 3
RUNG6_BRAKE_FACTOR = 3.0
RUNG6_CRUISE_V = 0.4                     # m/s == rolling_speed(RUNG4_OMEGA_CMD)

# --------------------------------------------------------------------------
# Rung 7 -- five independent robots, each with its own command.
# --------------------------------------------------------------------------
#
# Five copies of the rung-4 rover in parallel lanes, each commanded a DIFFERENT
# wheel rate, so each has its own analytic target and a command that leaked
# from one robot to another is visible as a wrong distance rather than as a
# plausible fleet average.
#
# WHAT "NO ROBOT PERTURBS ANOTHER" MEANS HERE, AND WHY IT IS NOT A CONTACT
# COUNT.  Each robot's distance is judged against the value it would travel
# ALONE (omega_i r T).  If any robot's presence changed another's motion, that
# robot's own distance moves off its solo expectation and the check goes red --
# a stronger claim than "no contact was reported", and one that does not depend
# on a contact API at all.  This tree has shipped contact reads that returned
# an empty set for a scene with 1008 contacts in it; a geometric separation
# measured from the poses cannot do that.
#
# NO RADIO.  Coordination between robots is deliberately NOT part of this rung:
# MuJoCo has no Emitter/Receiver, and a rung that one arm structurally cannot
# express would produce a NOT_EXPRESSIBLE verdict that says nothing about the
# physics.  Independent multi-robot dynamics is expressible everywhere and is
# the thing being measured.
RUNG7_N = 5
RUNG7_TAGS = tuple("r%d" % i for i in range(RUNG7_N))
RUNG7_OMEGA = (2.0, 3.0, 4.0, 5.0, 6.0)  # rad/s, one per robot -> 0.2..0.6 m/s
RUNG7_LANE_DY = 1.5                      # m between adjacent lanes
RUNG7_Y = tuple((i - (RUNG7_N - 1) / 2.0) * RUNG7_LANE_DY
                for i in range(RUNG7_N))                 # -3 .. +3
RUNG7_WIN = (2.0, 6.0)                   # s, after the spin-up, as rung 4
RUNG7_DURATION = 6.5                     # s
# Overall width of one rover: outer face of one wheel to the other.
RUNG7_ROBOT_WIDTH = 2.0 * (WHEEL_Y + WHEEL_W / 2.0)      # = 0.5 m
RUNG7_CLEARANCE = RUNG7_LANE_DY - RUNG7_ROBOT_WIDTH      # = 1.0 m of air

# --------------------------------------------------------------------------
# Rung 8 -- a gripper lifts a payload off a table and carries it to a target.
# --------------------------------------------------------------------------
#
# A CARTESIAN GANTRY, NOT AN ARTICULATED ARM.  The claim rung 8 makes is about
# the GRASP -- payload airborne, tracking the gripper, ending at the target --
# and none of it needs a revolute chain.  An articulated arm would add inverse
# kinematics and its own pose error to a measurement that is not about either,
# and those belong to a later rung.  Two prismatic stages (traverse, lift) plus
# two prismatic fingers put the gripper exactly where the schedule says with no
# solving at all, so any deviation of the PAYLOAD is the payload's.
#
# THE WRIST ORIGIN IS AUTHORED AT THE PART'S CENTRE.  That is what makes
# ``carry_rel`` an analytic zero rather than a self-reference: the payload's
# centre and the wrist's origin coincide at t=0 by construction, and they must
# still coincide at the end of the carry if the grasp held.  A grasp check that
# takes its reference from the run cannot tell a part that was gripped
# correctly from one that was gripped in the wrong place and then held there.
#
# FRICTION IS RAISED FOR THIS RUNG, ON PURPOSE.  The global MU = 1.0 makes the
# required pinch force marginal, and a rung that fails because the operator did
# not tune mu is a rung about tuning.  At RUNG8_MU the Coulomb bound needs only
# m g / (2 mu) = 0.33 N per pad and the commanded RUNG8_GRIP_N is 9x that, so
# the measurement is "does a grasp with a large Coulomb margin hold", NOT "what
# is the smallest friction this engine can grip at".  The latter is a sweep and
# is not claimed here.
RUNG8_MU = 3.0                           # Coulomb friction for this scene only
RUNG8_TABLE_SIZE = (1.2, 0.6, 0.2)       # full extents, m
RUNG8_TABLE_CENTER_Z = FLOOR_TOP + RUNG8_TABLE_SIZE[2] / 2.0         # = 0.6
RUNG8_TABLE_TOP = RUNG8_TABLE_CENTER_Z + RUNG8_TABLE_SIZE[2] / 2.0   # = 0.7
RUNG8_PART_EDGE = 0.06                   # m, cube
RUNG8_PART_MASS = 0.2                    # kg
RUNG8_PART_Z0 = RUNG8_TABLE_TOP + RUNG8_PART_EDGE / 2.0              # = 0.73
RUNG8_GRASP_Z = RUNG8_PART_Z0            # the wrist origin, by construction
RUNG8_BASE_Z = 1.2                       # m, the gantry rail
RUNG8_CARRIAGE_SIZE = (0.12, 0.12, 0.08)
RUNG8_CARRIAGE_MASS = 1.0
RUNG8_WRIST_SIZE = (0.10, 0.16, 0.02)    # the plate the pads hang from
RUNG8_WRIST_MASS = 0.2
RUNG8_WRIST_PLATE_DZ = 0.06              # plate offset above the wrist origin,
                                         # so it clears the part by 20 mm
RUNG8_PAD_SIZE = (0.03, 0.012, 0.05)     # x, y, z full extents
RUNG8_PAD_MASS = 0.02
# Pad centre at first contact with the part, and where it starts.
RUNG8_PAD_TOUCH_Y = RUNG8_PART_EDGE / 2.0 + RUNG8_PAD_SIZE[1] / 2.0  # = 0.036
RUNG8_PAD_OPEN_Y = 0.060                 # m, 24 mm of clearance each side
RUNG8_GRIP_N = 3.0                       # N per pad, normal to the part
RUNG8_LIFT_H = 0.15                      # m
RUNG8_LIFT_V = 0.10                      # m/s
RUNG8_TRAVERSE_X = 0.45                  # m, the place target
RUNG8_TRAVERSE_V = 0.15                  # m/s
RUNG8_T_SETTLE = 0.5                     # s: part settles, fingers open
RUNG8_T_CLOSE = 1.5                      # s: fingers closed by here
RUNG8_T_LIFT = 3.0                       # s: lift complete (0.15 m at 0.10)
RUNG8_T_TRAV = 6.0                       # s: traverse complete (0.45 m at 0.15)
RUNG8_DURATION = 7.5                     # s
RUNG8_HOLD_WIN = (6.5, 7.5)              # s, the carried-at-the-target window
RUNG8_SPEED_SPAN = 0.05                  # s, the window a speed is read over
RUNG8_PART_TARGET_Z = RUNG8_PART_Z0 + RUNG8_LIFT_H                   # = 0.88


def rung8_grip_force_bound(mass=RUNG8_PART_MASS, mu=RUNG8_MU, g=G, pads=2):
    """Least normal force per pad that can hold ``mass`` by friction alone.

    ``N >= m g / (pads * mu)``.  RUNG8_GRIP_N is roughly 9x this, which is what
    lets the rung assert "a grasp holds" without also asserting where the
    engine's friction limit is.
    """
    return mass * g / (pads * mu)


def rung8_bite_m(kp_n_per_m, ke_n_per_m):
    """How far INSIDE the part a position-controlled pad must be commanded.

    The pad's servo and the contact act in series, so the interference needed
    to develop ``RUNG8_GRIP_N`` is ``F (1/kp + 1/ke)``.  Both stiffnesses are
    engine-specific -- the contract owns the FORCE, each arm owns the actuator
    that produces it -- so this helper takes them as arguments and lives here
    only so the algebra is written once.

    An arm whose engine offers a true force mode should use it and ignore this.
    """
    kp = float(kp_n_per_m)
    ke = float(ke_n_per_m)
    return RUNG8_GRIP_N * ((1.0 / kp if kp > 0 else 0.0)
                           + (1.0 / ke if ke > 0 else 0.0))


def rung8_lift_z(t):
    """Commanded lift-joint position (m above the grasp height) at time ``t``."""
    if t <= RUNG8_T_CLOSE:
        return 0.0
    return min(RUNG8_LIFT_V * (t - RUNG8_T_CLOSE), RUNG8_LIFT_H)


def rung8_traverse_x(t):
    """Commanded traverse-joint position (m) at time ``t``."""
    if t <= RUNG8_T_LIFT:
        return 0.0
    return min(RUNG8_TRAVERSE_V * (t - RUNG8_T_LIFT), RUNG8_TRAVERSE_X)


# ==========================================================================
# TIER C / E -- rungs 9, 11 and 18.  Designed in PLAN_9_20.md.
# ==========================================================================
#
# NUMBERING, stated once so nothing goes missing silently.  PLAN_9_20.md
# designs eleven rungs and numbers the recorded-reality one 19 and the
# closed-kinematic-loop one 18.  THREE are built, and the build brief's
# numbering is the one in the tree: rung 18 IS the recorded-reality rung.
# The plan's closed-loop design (PLAN_9_20.md section 5.10) is NOT built and is
# referred to here by name rather than by number, so no built rung and no
# designed rung share one.
#
# WHY THESE THREE AND NOT THE OTHER EIGHT.  PLAN_9_20.md section 0.1 is the
# reason and it is not a scheduling note: on correctness assertions alone this
# ladder cannot differentiate us from upstream Webots anywhere in tiers C or D,
# and it can never beat MuJoCo on fidelity -- MuJoCo IS our solver.  A rung
# whose only outcome is "three mature engines agree" is a week spent
# confirming that nobody has a bug.  These three are the ones where our own
# answer is UNKNOWN or KNOWN-BAD:
#
#   9   determinism   -- bitwise on CPU mj_step, REFUTED on mujoco_warp
#                        (0 of 24 same-config cold pairs), cross-machine
#                        untested.  A rung we partly fail, scoped honestly.
#   11  scale         -- the njmax question two other lanes have left open in
#                        both directions.
#   18  reality       -- the only ground truth in this ladder that is not
#                        ours, and the only check anywhere in it that we can
#                        lose and cannot win (``embed_gap``).


# --------------------------------------------------------------------------
# Rung 9 -- determinism, with its own sensitivity control
# --------------------------------------------------------------------------
#
# THE SCENE IS A PILE, AND THAT IS THE WHOLE DESIGN.  Determinism is trivially
# satisfied by a frozen world -- zero motion reproduces exactly -- so a
# determinism rung needs two things a two-body scene cannot give it: enough
# simultaneous contact pairs that the mechanism behind the known GPU refutation
# can bite, and a control proving the scene amplifies rather than damps.
#
# The refutation's mechanism is cited from mujoco_warp's own source: contact
# pairs are assigned with ``pairid = wp.atomic_add(...)``
# (collision_driver.py), so the pair ORDER is a nondeterministic race and it
# needs many simultaneous pairs to express itself.  The same measurement
# campaign that found 0 of 24 bitwise pairs across six scenes also recorded a
# SINGLE-CONTACT scene reproducing bit-identical 3/3 cold on the same path.  A
# two-body rung 9 would therefore return a FALSE GREEN on the GPU variant --
# the same shape as rung 5's static scene, which "cannot refute a stale-scene
# freeze and is not offered as doing so".
#
# 25 resting cubes give of order 100 contact points, inside the 80-320 band
# where the divergence was measured.
RUNG9_GRID = 5                           # 5 x 5 resting cubes
RUNG9_GAP = 0.001                        # m of air between neighbours
RUNG9_PITCH = BOX_EDGE + RUNG9_GAP       # = 0.201 m centre to centre
RUNG9_PILE_Z = FLOOR_TOP + BOX_HALF      # = 0.6 m, one layer on the floor
RUNG9_PILE_TOP = FLOOR_TOP + BOX_EDGE    # = 0.7 m, the surface it is dropped on
# The 26th cube is released over the OUTER CORNER OF THE PILE, so a quarter of
# its footprint is supported, its centre of mass sits exactly over the edge of
# that support, and the other three quarters overhang a 0.2 m drop to the
# floor.  It must topple, and which way it topples is set by numbers far below
# anything physical -- which is precisely the property the sensitivity control
# needs to exist at all.
#
# ⚠ THE OBVIOUS PLACEMENT IS WRONG AND WAS MEASURED WRONG.  The first version
# of this rung dropped the cube over the corner of the CENTRE cube, at
# (BOX_HALF, BOX_HALF).  With a 1 mm gap the pile's pitch is 0.201 and a 0.2 m
# cube placed there overlaps FOUR neighbours by a quarter each, so it is not
# balanced on a corner at all -- it is centred on a 2 x 2 group and perfectly
# supported.  Measured on OmniSim: it landed at z = 0.799800 and stayed, the
# pile never moved, and a 1 um seed produced 1.0058 um of separation after 8 s.
# The rung's control read as "this engine damps perturbations" when the true
# answer was "this scene has nothing to amplify".
RUNG9_DROP_XY = (RUNG9_GRID - 1) / 2.0 * RUNG9_PITCH + BOX_HALF   # = 0.502 m
RUNG9_SPAWN_Z = 1.6                      # m, as rung 2
RUNG9_FIRST_CONTACT_Z = RUNG9_PILE_TOP + BOX_HALF    # = 0.8 m
# ⚠ BOTH GATES ARE CLEAR OF FIRST CONTACT, AND THAT IS A CORRECTION.
#
# They were 1.2 and 0.8 -- rung 2's convention, where the lower gate IS the
# first-contact height.  That works at rung 2's stride of 1 and does not work
# here, and the upstream-Webots arm found it: at RUNG9_SAMPLE_EVERY = 5 the
# sample AFTER the lower crossing is already contact-decelerated (measured on
# ODE: t=0.400 z=0.807352, t=0.420 z=0.792858 against a free-fall 0.7265), so
# the reducer's linear interpolation drags the crossing up to a full sample
# interval late.  It read 0.126670 s against an analytic 0.118286 -- RED -- and
# the SAME replica re-run at stride 1 read 0.118282 s, 4 us from analytic.
#
# The check was therefore grading CONTACT HARDNESS as fall time: a soft contact
# decelerates the cube earlier in the straddling interval than a hard one, and
# OmniSim's 2.5 ms and ODE's 8.4 ms differed in exactly that direction.  With
# the gates clear, all three arms read the SAME 0.147685 s.
#
# Both gates now sit in provable free fall.  The lower one clears first contact
# by 0.2 m, which at the 3.43 m/s the cube carries there is 2.9 sample
# intervals -- so the pair of samples straddling the crossing are both
# ballistic whatever the contact model does.  The interval is LONGER than
# before (0.1478 s vs 0.1183 s), so FALL_INTERVAL_TOL is a smaller fraction of
# it, not a larger one.
RUNG9_GATE_HI = 1.4                      # m, box centre (0.2 m of fall done)
RUNG9_GATE_LO = 1.0                      # m, 0.2 m clear of first contact
RUNG9_DURATION = 8.0                     # s
RUNG9_N_BODIES = RUNG9_GRID ** 2 + 1     # = 26

# Sample stride (CONTRACT.md amendment D).  26 bodies x 3 coords x 2000 steps
# x 3 runs is 468k floats, and a determinism document must round-trip float64
# EXACTLY, so it cannot be shrunk by formatting.  A divergence that appears and
# disappears entirely inside 20 ms is not a divergence anyone can act on, and a
# real one grows: every measured GPU pair that diverged at all reached
# 4.15e-05 m by 120 steps and 9.152 m by 1000.  THE STRIDE IS THE CONTRACT'S.
RUNG9_SAMPLE_EVERY = 5                   # steps between samples (= 20 ms)

# The sensitivity control.  1 um is 1000x below PENETRATION_TOL -- physically
# irrelevant by this ladder's own standards -- and it is applied to the dropped
# cube's spawn x.
RUNG9_EPS = 1e-6                         # m

# HOW MUCH THE SCENE MUST AMPLIFY.  A FACTOR, not a distance, and the change
# from a distance is a correction with its cause on the record.
#
# It was RUNG9_SEP_MIN = 1e-3 m, i.e. "1000x amplification of a 1 um seed",
# and the derivation printed beside it justified only a LOWER BOUND (round-off
# cannot reach it); 1000 was a round number.  The upstream-Webots arm then
# measured 202x on ODE against 145,346x and 145,509x on the two MuJoCo-family
# arms -- so the round number was about to fail one engine for amplifying "only"
# two hundred fold, which is not a claim mechanics supports.  ⚠ THIS WAS
# CHANGED AFTER SEEING THAT RESULT, which is disclosed rather than quietly
# done; what makes it a correction rather than a fit is that the new threshold
# is derived from what it must EXCLUDE and lands two-and-a-half orders below
# the smallest measured value instead of just under it.
#
# What it must exclude, and nothing else:
#   * a FROZEN world, which amplifies exactly 1.0 (measured on both arms via
#     the ``frozen`` fault).  This is the case the control exists for;
#   * float64 round-off, which over 2000 steps contributes ~1e-14 relative --
#     about 1e-8 of the seed, i.e. an amplification indistinguishable from 1.
#
# A factor of 10 separates unambiguously from both and is deliberately far
# below every measured value.  IT IS A FLOOR ON "THE SCENE IS NOT FROZEN", NOT
# A GRADE ON HOW CHAOTIC AN ENGINE IS -- nothing in mechanics says how chaotic
# a pile OUGHT to be, so the 700x spread between the arms is REPORTED beside
# the row and never judged.
RUNG9_AMPLIFY_MIN = 10.0                 # dimensionless

# MEASURED OVER THE WHOLE RUN, NOT AT t_end, and this is a deliberate departure
# from PLAN_9_20.md section 5.1.  Friction is dissipative: a pile that settles
# is an attractor, and two runs seeded 1 um apart can legitimately reconverge
# to the same resting configuration.  At t_end that reads as "the scene is not
# chaotic"; over the whole run it reads as what it is -- the perturbation was
# amplified and then damped out.  The claim the control has to support is that
# THE SCENE AMPLIFIES, which is a statement about the trajectory and not about
# its limit point.
#
# ! Declared, not closed: a red ``sensitivity_shortfall`` remains ambiguous
# between "this engine damps perturbations" and "this scene is not chaotic on
# this engine".  Read it only alongside ``repeat_delta``; the informative
# signal is the PAIR.  The measured amplification factor is reported beside the
# row either way, so the ambiguity can be sized rather than argued about.

# Fault magnitudes.  Replica b's spawn is moved by this much; a run that is
# bitwise reproducible cannot survive it, and it is far too small to support
# any physical claim.  It is emitted with %.17g so it round-trips through the
# scene file; the arms' ordinary %.6f formatter would write it as "0" and the
# fault would silently not happen.
#
# ⚠ 1e-7 m, AND THE VALUE IS A MEASUREMENT RATHER THAN A CHOICE.  It started at
# 1e-12 m -- a picometre, which is what "obviously unphysical" looks like -- and
# the fault DID NOT GO RED: the run came back bitwise identical.  Sweeping the
# magnitude separated the two explanations (a plumbing bug is
# magnitude-independent; a precision floor is not):
#
#     nudge      max |x - honest| over the run
#     1e-12 m    0            (bitwise identical)
#     1e-09 m    0            (bitwise identical)
#     1e-08 m    0.0109 m
#     1e-07 m    0.0228 m
#     1e-06 m    0.1455 m
#
# **THE SCENE POSE REACHES THE SOLVER IN SINGLE PRECISION.**  The direct
# evidence is in the readback rather than the sweep: the dropped cube is
# authored at x = 0.502 and the supervisor reads it back as
# 0.50199997425079346, which is exactly float32(0.502).  The threshold sits
# between 1e-9 and 1e-8 because 0.502 happens to lie 4.05e-9 below the midpoint
# between its two neighbouring float32 values -- so the smallest expressible
# perturbation depends on the COORDINATE, not only on the exponent, and a
# margin has to be taken over float32 epsilon at the coordinate rather than
# over the observed threshold.  eps at 0.502 is 5.96e-8; 1e-7 clears it.
#
# It is still 1e4 times smaller than PENETRATION_TOL, the tightest length in
# this ladder, and 10x smaller than the sensitivity seed, so nothing physical
# can be claimed against it.
RUNG9_FAULT_NUDGE = 1e-7                 # m, replica b's spawn offset
RUNG9_FAULT_SHORT = 0.5                  # fraction of the run replica b does

#: The replica set.  THE TAGS AND PARAMS ARE THE CONTRACT'S, NOT THE ARM'S
#: (CONTRACT.md amendment A): an arm that chose its own perturbation produces a
#: row that is not comparable and nothing in the table would say so.
RUNG9_RUNS = (("a", 0.0), ("b", 0.0), ("c", RUNG9_EPS))


def rung9_pile_xy():
    """World (x, y) of each resting cube, in the contract's own order.

    The tags are ``p<row><col>`` with row/col counting from 0, so an arm cannot
    silently reorder the pile and compare the wrong bodies to each other.
    """
    half = (RUNG9_GRID - 1) / 2.0
    return [("p%d%d" % (i, j),
             (i - half) * RUNG9_PITCH, (j - half) * RUNG9_PITCH)
            for i in range(RUNG9_GRID) for j in range(RUNG9_GRID)]


RUNG9_BODY_TAGS = tuple([t for t, _x, _y in rung9_pile_xy()] + ["drop"])


# --------------------------------------------------------------------------
# Rung 11 -- fidelity at scale
# --------------------------------------------------------------------------
#
# Rung 4's rover, N of them in parallel lanes, each commanded its own wheel
# rate.  Every robot must meet the SAME analytic target it would meet alone,
# with the SAME tolerance, at every fleet size.
#
# NO N-DEPENDENT SLACK, and that is the rung.  The failure it exists to catch
# is a silently truncated constraint vector, documented at 9% displacement
# error -- nearly 2x DISTANCE_TOL.  A tolerance that grew with N would be
# pre-authorised to miss exactly the defect it was built for.
#
# BIT-IDENTITY OF ROBOT i ACROSS N IS NOT ASSERTED, and refusing to assert it
# is a decision rather than an omission (PLAN_9_20.md section 4.5).  Adding
# robots changes the size and the ordering of the constraint system;
# floating-point summation is not associative; robot i can differ in the last
# ULP at N = 16 versus N = 1 in an engine with no defect at all.  A red that
# means nothing trains everyone to ignore the row.  The quantity is MEASURED
# and REPORTED beside the row instead (``solo_deviation_max``), because how
# fast that ULP grows is genuinely interesting -- it is simply not a verdict.
RUNG11_N = (1, 4, 8, 16)                 # the judged sweep
RUNG11_N_VARIANT = 32                    # published beside it, not a row
RUNG11_DURATION = RUNG4_DURATION         # = 6.5 s
RUNG11_WIN = RUNG4_WIN                   # = (2.0, 6.0) s
RUNG11_LANE_DY = RUNG7_LANE_DY           # = 1.5 m
RUNG11_ROBOTS_TOTAL = sum(RUNG11_N)      # = 29 across the whole cell

# Sample stride (amendment D).  16 robots x 7 series x 1625 steps is 182k
# floats per run.  8 ms is set by the FASTEST thing the reduction has to see:
# ``_max_overrun`` evaluates over sliding 0.1 s sub-windows and needs several
# samples inside one, and RIDE_SETTLE_S is 0.2 s.  At the fastest command the
# wheel turns 0.048 rad per sample, two orders inside the unwrap fold.
RUNG11_SAMPLE_EVERY = 2                  # steps between samples (= 8 ms)

# Each robot's command CYCLES the rung-7 rates, so no fleet contains one
# repeated command: a rate that leaked from one robot to another reads as a
# wrong distance rather than as a plausible fleet average, and an engine that
# simulates one robot and copies it lands every clone on one target.
RUNG11_OMEGA = RUNG7_OMEGA


def rung11_omega(i):
    """Commanded wheel rate of robot ``i``, rad/s."""
    return RUNG11_OMEGA[i % len(RUNG11_OMEGA)]


def rung11_y(i, n):
    """Lane centre of robot ``i`` in a fleet of ``n``, metres."""
    return (i - (n - 1) / 2.0) * RUNG11_LANE_DY


def rung11_floor_size(n):
    """Floor extents for a fleet of ``n``.  Generous ON PURPOSE.

    FLOOR_SIZE's own comment argues this and the argument is inherited: the
    half-extent a CORRECT engine needs is ``v_max T + WHEEL_X + WHEEL_R``, and
    sizing to that is wrong, because a rover that runs off the lip produces a
    beached-and-spinning signature that MASKS whatever defect pushed it there.
    Measured on rung 4: a launch transient gained 1.37 m of free distance and
    the rover fell off an 8 x 8 floor, and the run reported the fall rather
    than the overrun.  So the along-track extent carries 3x.

    Across track the robots do not travel at all, so 3x means nothing there;
    the margin is one WHOLE SPARE LANE each side, which is 6x the rover's own
    half-width and 15x LATERAL_TOL.
    """
    v = rolling_speed(max(RUNG11_OMEGA))
    half_x = 3.0 * (v * RUNG11_DURATION + WHEEL_X + WHEEL_R)
    half_y = (n - 1) / 2.0 * RUNG11_LANE_DY + RUNG11_LANE_DY
    return (2.0 * half_x, 2.0 * half_y, FLOOR_SIZE[2])


# The starve-the-budget fault.  THE MECHANISM IS NOT "SET njmax LOWER", and
# getting that wrong is why an earlier instrument could not make this bite.
#
# Read from the engine's own source (src/omnisim/physics/omnisim_newton_runtime.py,
# the constraint-buffer note at world build): "newton raises a too-small njmax
# to the INITIAL nefc at construction, so the initial counts are the FLOOR, not
# the requirement".  ``newtonNjmax`` is therefore a FLOOR: the cap that governs
# truncation is ``max(requested_or_256, nefc_at_t0)``.
#
# Consequence, and it is the whole design of this fault: a fleet that spawns IN
# CONTACT has nefc_at_t0 = 32 N already, so the cap is auto-raised to the peak
# and NOTHING CAN EVER OVERFLOW -- which is exactly the 384/384 reading a
# previous attempt got at N = 12 and honestly refused to call a pass.  A fleet
# that spawns CLEAR of the ground has nefc_at_t0 ~ 0, keeps the 256 default,
# and overflows the moment 32 N exceeds it, i.e. from N = 9.
#
# So the fault lifts the fleet off the floor at t = 0 and lets it settle.  The
# rung-4 rover spawns at ROBOT_Z = FLOOR_TOP + WHEEL_R, exactly touching; this
# clearance is 50x the measured resting penetration and 1/2 the wheel radius,
# so the fleet lands within one step-count of t = 0 and the run is otherwise
# the honest one.
RUNG11_CLEAR_M = 0.05                    # m above the floor at t = 0
RUNG11_FAULT_N = 16                      # the fleet size the faults are run at
RUNG11_FAULT_ROBOT = 7                   # index of the robot a fault breaks
# RUNG11_FAULT_OFFSET_M is set beside rung 7's, in the fault-magnitude block
# below, so the two cannot drift: they are the same fault at two fleet sizes
# and a red that meant different things at N = 5 and N = 16 would not be
# comparable.

# Constraint budget, declared under CONTRACT.md 3b R4 and section 3E.  GENEROUS,
# never sized at a measured peak: setting newtonNjmax to a scene's own peak
# (320) moved results 8.81 m versus every other size, with a 1.71 m run-to-run
# spread, while 512 / 2048 / 4096 agreed to 1e-4.  32 N at N = 32 is 1024, so
# 4096 is 4x the largest fleet in the family.
RUNG11_NJMAX = 4096
RUNG11_NCONMAX = 4096

# The rule of thumb the measurement is read against: a 4-wheel-drive rover
# resting on flat ground stands on 4 contacts, and a MuJoCo contact with a
# pyramidal cone carries 8 constraint rows -- hence 32 rows per rover.  It is
# REPORTED rather than asserted here: it is an engine-internal count, not a
# physical quantity, and this ladder judges only physical quantities.
RUNG11_ROWS_PER_ROBOT = 32


# --------------------------------------------------------------------------
# Rung 18 -- agreement with recorded reality
# --------------------------------------------------------------------------
#
# CONTRACT.md section 1 forbids an expected value read out of a running
# simulator.  This rung's expected values come from a MEASUREMENT OF PHYSICAL
# REALITY -- 550 recorded tosses of an acrylic cube onto a wooden table,
# AprilTag/TagSLAM tracked at 148 Hz -- which amendment F admits and which
# leaves section 1 intact: a golden captured from today's behaviour certifies
# today's defects; a tracked cube does not know what a simulator is.
#
# THE DATASET IS NOT REBUILT HERE AND NO PHYSICAL CONSTANT OF IT IS RE-DECLARED.
# tests/benchmarks/omnibench/lane1r owns the recording, its licence, the cube's
# mass/geometry/inertia, the sampling rate, the quaternion convention and the
# self-calibration -- all of which it RE-DERIVES on every run rather than
# trusting the dataset's own metadata.  This rung reads them through
# ``rung18_dataset()``.  If this ladder and lane1r ever disagree about the
# cube's inertia, LANE1R IS RIGHT.
#
# What this rung owns is the part lane1r does not have: a contract-owned toss
# subset, an acceptance band with a derivation, and a cross-arm check.
RUNG18_LANE1R = ("tests", "benchmarks", "omnibench", "lane1r")

#: The judged subset -- fixed, contract-owned, and never chosen by an arm.
RUNG18_INDICES = tuple(range(50))
#: The subset the FAULT battery runs on.  A fault must redden its assertion
#: decisively, and every one of rung 18's does so by a factor of several; 50
#: tosses per fault would cost 21 minutes to prove something six show.  It is
#: contract-owned for the same reason the judged set is.
RUNG18_FAULT_INDICES = tuple(range(6))
#: Every subset an arm is allowed to have run.  A document reporting anything
#: else is judged RED for provenance rather than scored -- an arm that could
#: pick its own tosses could pick the easy ones.
RUNG18_SUBSETS = (RUNG18_INDICES, RUNG18_FAULT_INDICES)

# ``none`` is the published scale, NOT the physically self-consistent one.
# lane1r measures a ~2.2% length-scale factor in the tracked data and can undo
# it, but the baselines below were computed WITHOUT that correction, so the
# ladder row must be uncorrected or the comparison is not like for like.
RUNG18_SCALE = "none"

#: Published per-simulator baselines, rolling every toss out from its measured
#: initial condition (Acosta, Yang & Posa, "Validating Robotics Simulators on
#: Real-World Impacts", RA-L 2022, arXiv:2110.00541, Table II).  Position error
#: is a percentage of cube width; rotation error is degrees.  EXTERNAL, and
#: written by people who never heard of this project.
RUNG18_BASELINES = {
    "Drake":  {"pos_pct": 13.5, "pos_sd": 8.2,  "rot_deg": 16.5, "rot_sd": 20.0},
    "Bullet": {"pos_pct": 14.9, "pos_sd": 8.9,  "rot_deg": 16.5, "rot_sd": 20.2},
    "MuJoCo": {"pos_pct": 25.1, "pos_sd": 10.8, "rot_deg": 21.7, "rot_sd": 21.4},
}

# THE ACCEPTANCE BAND, AND WHY IT IS NOT "WE WIN".  The published MuJoCo row
# plus one published standard deviation.  Three properties make it the right
# bound and each of them matters:
#
#  * it is EXTERNAL.  Nothing in this repo can move it, and it was not chosen
#    after seeing our number;
#  * it is the row for the engine we EMBED, so passing it means "our
#    translation layer did not lose what the solver gave us", which is the only
#    honest claim available against our own vendor;
#  * it is NOT a good score.  The best engine measured on this data manages
#    13.5%.  A rung that a user could read as "OmniSim is accurate" would be
#    lying; this one says "OmniSim is where the field is", and the field is not
#    very good at tossed cubes.
RUNG18_POS_BOUND = (RUNG18_BASELINES["MuJoCo"]["pos_pct"]
                    + RUNG18_BASELINES["MuJoCo"]["pos_sd"])       # = 35.9 %
RUNG18_ROT_BOUND = (RUNG18_BASELINES["MuJoCo"]["rot_deg"]
                    + RUNG18_BASELINES["MuJoCo"]["rot_sd"])       # = 43.1 deg

# THE TRANSLATION-FIDELITY BOUND -- the one check on this ladder we can only
# lose.  Our solver IS MuJoCo, so a gap between this arm's error and the BARE
# MuJoCo arm's error on the SAME tosses, against the SAME recording, scored by
# the SAME reducer, is our layer and not the physics.  5% of cube width is
# 5.2 mm: an eighth of the smaller of the two published standard deviations,
# and a fifth of the Drake-to-MuJoCo spread that separates the best engine in
# the field from ours.  A translation layer that costs more than a fifth of the
# distance between two different solvers is not preserving what it was given.
RUNG18_EMBED_GAP_TOL = 5.0               # percentage points of cube width

# Tunnelling.  Not a penetration check -- a compliant contact legitimately
# sinks a 0.37 kg cube by ~ v*tau/e = 22 mm at this dataset's impact speeds
# (MuJoCo's default solref timeconst is 0.02 s), so a millimetre bound on
# PENETRATION would grade contact softness and go red on every arm.  What is
# asserted instead is unambiguous: the cube's TOP FACE may never pass below the
# table's top surface.  That needs 52.4 mm of penetration -- more than twice
# the compliance bound -- and it is what a cube that fell THROUGH the table
# does on its way to the phantom z = 0 plane 0.5 m below.
RUNG18_TUNNEL_TOL = 0.001                # m

# Initial-condition fidelity.  This exists because this tree has shipped
# ``setVelocity`` being silently DROPPED at t = 0 -- a defect that would
# otherwise present here as "poor real-world agreement" and be charged to the
# contact model.  The quantity is the engine's OWN READBACK of the state it
# accepted, against the state it was asked for, less one step of gravity (the
# readback happens after one step, by which point gravity has legitimately
# changed vz).  1% of the commanded speed is what remains for the write itself.
RUNG18_IC_TOL = 0.01                     # relative

#: Per-toss simulated duration, seconds: 121 samples at the dataset's rate.
#: Read back from lane1r at run time; this is the value it implies and is
#: carried only so ``DURATION`` has an entry.
RUNG18_TOSS_S = 121.0 / 148.0            # = 0.8176 s


def rung18_dataset():
    """lane1r's dataset module, imported BY PATH.

    Not a copy and not a re-declaration.  lane1r is another lane's work and is
    read-only from here; this is the seam, and it is deliberately the only one.
    Raises ``ImportError`` with the path it tried, because an arm that silently
    fell back to constants of its own would produce a row that looks
    comparable to lane1r's published campaign and is not.
    """
    import importlib.util
    import os as _os
    import sys as _sys
    here = _os.path.dirname(_os.path.abspath(__file__))
    repo = _os.path.abspath(_os.path.join(here, _os.pardir, _os.pardir,
                                          _os.pardir))
    path = _os.path.join(repo, *(RUNG18_LANE1R + ("dataset.py",)))
    key = "ladder0_lane1r_dataset"
    if key in _sys.modules:
        return _sys.modules[key]
    if not _os.path.isfile(path):
        raise ImportError("lane1r dataset not found at %s -- rung 18's ground "
                          "truth is a vendored recording and this ladder may "
                          "not substitute constants of its own" % path)
    sp = importlib.util.spec_from_file_location(key, path)
    mod = importlib.util.module_from_spec(sp)
    _sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# Analytic ground truth
# --------------------------------------------------------------------------

def fall_time_s(drop_m, g=G):
    """Time to fall ``drop_m`` from rest: t = sqrt(2 d / g)."""
    return math.sqrt(2.0 * drop_m / g)


def fall_interval_s(z0, z1, z2, g=G):
    """Time for a body released from rest at ``z0`` to fall from ``z1`` to
    ``z2``.

    ``= sqrt(2 (z0-z2)/g) - sqrt(2 (z0-z1)/g)``.

    This is the rung-2 primary assertion because it is INVARIANT to a shift of
    the clock: if the engine free-runs a few steps before the controller's
    first tick, both crossings move by the same amount and the interval does
    not.  It still pins the dynamics, because the interval depends on g and on
    the release height through the velocity the body carries into ``z1``.
    """
    return fall_time_s(z0 - z2, g) - fall_time_s(z0 - z1, g)


def rolling_speed(omega, r=WHEEL_R):
    """Ground speed of a wheel rolling without slip: v = omega r."""
    return omega * r


# --------------------------------------------------------------------------
# Tolerances -- each with its derivation
# --------------------------------------------------------------------------

# Contact compliance.  Every solver here resolves contact with a soft
# constraint, so a resting body penetrates slightly.  MuJoCo's default
# ``solref`` is (timeconst=0.02 s, dampratio=1); the reference acceleration is
# a critically damped spring a = -(1/tau^2) x, so a body in equilibrium under
# gravity sits at penetration x = g tau^2 = 9.81 * 0.02^2 = 3.9 mm -- and, note,
# INDEPENDENT of mass, because the reference is an acceleration.  ODE-family
# solvers with default CFM/ERP land in the same millimetre band.  5 mm is that
# number rounded up.  It is ~1/20 of the box's half-height, so it cannot hide a
# body resting on the wrong surface.
REST_Z_TOL = 0.005                       # m

# The SHARP companion to REST_Z_TOL -- same quantity, different derivation --
# because 5 mm is the LOOSEST defensible bound and a real contact-stiffness
# regression hides under it.  Measured: a resting box penetrates 0.108 mm on
# both MuJoCo and OmniSim, and a contact 8x softer than default still passes
# the 5 mm check; it took 20x to make that check go red.
# MuJoCo's default ``solimp`` is (d0=0.9, d1=0.95, width=0.001): the constraint
# reaches essentially full stiffness within 1 mm of penetration, so a solver
# running those defaults comes to rest INSIDE the solimp width.  1 mm is that
# width -- a derived bound, and still ~10x the measured penetration, so it is
# not a fit to today's number.
#
# The two checks separate two claims, which is the point.  ``rest_z`` failing
# means the body is on the WRONG SURFACE (the 0.5 m error a phantom ground
# plane produces).  ``contact_penetration`` failing while ``rest_z`` passes
# means the contact is SOFTER than the solver's own defaults imply.  An engine
# whose contact model is legitimately softer -- an ODE-family ERP/CFM solver,
# say -- will fail this one, and the row reports by how much rather than
# hiding it.
PENETRATION_TOL = 0.001                  # m

# Sampling + integrator phase.  Crossing times are linearly interpolated
# between the two straddling samples, so the sampling error is second order
# (~ g dt^2 / 8 = 2e-5 m, i.e. microseconds in time).  What is left is the
# semi-implicit-Euler position bias, z_n = z0 - g t^2/2 - g t dt/2, which
# advances every crossing by dt/2 -- and which CANCELS in an interval between
# two crossings.  2 dt = 8 ms is therefore ~4x headroom over the residual, and
# is still 1.8% of the 166 ms interval being measured.
FALL_INTERVAL_TOL = 2.0 * DT             # s

# The first pose a controller observes should be the pose the world file
# authored.  One basic step of free fall is g dt^2/2 = 78 um; 20 ms allows the
# engine ~22 steps of free-run before the controller's first tick, which is
# generous, and anything past it is a real finding about the engine's start-up
# ordering rather than about the physics.
SPAWN_Z_TOL = 0.02                       # m

# An ideal velocity servo on an UNLOADED joint has zero steady-state error:
# no gravity torque about a vertical axis, no joint damping, no contact.  The
# only residual is the solver's own per-step velocity round-off.  1% of the
# command, floored at 0.01 rad/s, is three orders of magnitude above that and
# still 200x smaller than the failure it must catch (a joint that does not turn
# at all).
def omega_tol(cmd):
    return max(0.01 * abs(cmd), 0.01)    # rad/s


# Commanding zero must produce zero.  0.01 rad/s over the 1 s measurement
# window is 0.01 rad = 0.57 deg of creep -- below what any contact-free joint
# should accumulate, and far below a joint that kept spinning.
OMEGA_ZERO_TOL = 0.01                    # rad/s

# Rolling consistency v = omega r.  Steady-state slip for a rigid wheel on
# rigid ground at mu=1.0 carrying ~1.7 kg per wheel, driven at 0.4 m/s, is
# well under 1% in any of these solvers (the tractive force needed is only
# what overcomes the solver's own contact drag).  3% is ~3x headroom and is
# 30x smaller than the failure it exists to catch: a chassis that slides the
# right distance on wheels that never turned.
ROLL_CONSISTENCY_TOL = 0.03              # relative

# Distance travelled vs the commanded wheel rate.  Same slip term as above,
# plus the motor's own tracking error, measured over a window that starts
# 2 s after launch so the spin-up transient is entirely outside it.
DISTANCE_TOL = 0.05                      # relative

# Lateral drift.  A left/right-symmetric robot given identical commands on
# flat ground must travel straight; any lateral displacement is numerical
# asymmetry.  0.1 m over the 1.6 m the run covers is 6%, i.e. deliberately
# loose -- this assertion is here to catch a robot that veers or spins, not
# to grade symmetry.
LATERAL_TOL = 0.10                       # m

# THE BODY MAY NEVER OUTRUN ITS WHEELS.  This is a one-sided kinematic
# invariant and it needs no calibration: a wheel in contact with the ground
# can push the body forward at most v = omega r, and Coulomb slip can only
# make the body go SLOWER than that, never faster.  v/(omega r) > 1 is
# unphysical under any friction model, so the excess over 1 is asserted to be
# zero.  A body that lags its wheels (spin-up, wheelspin at the traction
# limit) produces zero excess and stays green.
#
# It exists because the windowed ``rolling_consistency`` check MISSED a real
# defect: measured on OmniSim 2026-08-12, the rover's wheels held the
# commanded 4.000 rad/s while the chassis travelled at up to 4.10 m/s -- a
# ratio of 10.25 against a rolling speed of 0.400 m/s -- for the first 1.3 s.
# The measurement window opened at t = 2 s, after the anomaly had ended, so
# every windowed assertion was green.  A steady-state window can only see
# steady state; this one sees the whole run.
ROLL_OVERRUN_TOL = 0.05                  # dimensionless excess over 1

# Ride height.  The rover has no suspension: its axle height is fixed by the
# wheel radius and the floor it is standing on, so the only legitimate
# variation is contact compliance -- 0.108 mm measured, and 3.9 mm by the
# loosest g*tau^2 bound above.  20 mm is 5x that bound and 20% of the wheel
# radius.  It catches a rover that is launched off the ground and one that
# has driven off the edge of the floor, both of which invalidate every other
# number in the rung while leaving them looking plausible.
RIDE_HEIGHT_TOL = 0.02                   # m
RIDE_SETTLE_S = 0.2                      # s of initial contact settling

# Range readings (rungs 5 and 6).  A ray against a box face is an exact
# geometric query: there is no integrator, no compliance and no contact model
# in it, so the only defensible slop is SAMPLE ALIGNMENT -- whether the engine
# reports a sensor read from before or after the step whose pose the arm
# recorded.  One step of the fastest motion in either rung is the bound:
# rung 5 sweeps at RUNG5_SWEEP_V, rung 6 cruises slower, so the sweep sets it.
# The 1 mm floor keeps a stationary reading from being asked for bit-exactness
# out of a float32 ray.  2 mm is 1450x smaller than the failure it must catch:
# a sensor frozen at t=0 is 2.0 m wrong by the end of the sweep.
RANGE_TOL = max(RUNG5_SWEEP_V * DT, 0.001)               # = 0.002 m

# Rung 6's stopping budget.  Derivation in the RUNG6_ block above:
# latency (3 steps of cruise) + friction-limited braking, the latter a lower
# bound carried at 3x because a contact reaches full braking force over a few
# solver steps rather than instantly.
RUNG6_BRAKE_M = RUNG6_CRUISE_V ** 2 / (2.0 * MU * G)     # = 8.155 mm
RUNG6_STOP_BOUND = (RUNG6_LATENCY_STEPS * RUNG6_CRUISE_V * DT
                    + RUNG6_BRAKE_FACTOR * RUNG6_BRAKE_M)        # = 29.3 mm

# The trigger reading.  The gap is sampled once per step while closing at
# RUNG6_CRUISE_V, so the first sample below the threshold lies within one
# step of travel of it; RANGE_TOL then covers the ray itself.
RUNG6_TRIGGER_TOL = 0.5 * RUNG6_CRUISE_V * DT + RANGE_TOL        # = 2.8 mm

# Rung 7 wheel rates.  The same 1% argument as ``omega_tol``, expressed
# relatively because the five robots carry five different commands and one
# absolute tolerance would be tight on the slowest and loose on the fastest.
OMEGA_REL_TOL = 0.01                     # relative

# Rung 8 payload pose.  The part hangs in two SOFT contacts.  At the commanded
# grip force a MuJoCo-family contact (reference acceleration -x/tau^2, tau =
# 0.02 s, effective mass ~ the part's 0.2 kg) penetrates
# F tau^2 / m = 3 * 4e-4 / 0.2 = 6 mm, and the tangential compliance carrying
# the part's 1.96 N of weight is of the same order.  10 mm is that bound
# rounded up.  It is 1/15 of the lift height and 1/45 of the traverse, so it
# cannot hide a payload that was dropped, left behind, or dragged.
RUNG8_POSE_TOL = 0.010                   # m

# Rung 8 payload speed, over the WHOLE run.  The payload may legitimately move
# at whatever the gantry commands; anything much faster is energy the contact
# solver put in.  3x the fastest commanded stage is the bound.  It exists
# because this repo has a MEASURED case of a pinch that ejected its part at
# 3.5 m/s and left it sitting on the gripper's own wrist plate -- an outcome
# whose final pose can look entirely plausible.
RUNG8_SPEED_BOUND = 3.0 * max(RUNG8_LIFT_V, RUNG8_TRAVERSE_V)    # = 0.45 m/s


# --------------------------------------------------------------------------
# Fault magnitudes -- shared, so a red proof means the same thing on every arm
# --------------------------------------------------------------------------
#
# These are the sizes of the deliberate breakages ``selftest.py`` injects.  They
# live here rather than in each arm for the same reason every other number
# does: three arms that broke their scenes by three different amounts would
# each be proving a different claim while reporting the same fault name, and
# "it went red" would stop being comparable.
#
# Each is sized to clear its target check's tolerance by a wide margin and to
# leave its companions' tolerances untouched -- a fault that only just fails is
# a fault whose red proves the boundary, not the wiring.

# Rung 5: how much further away the wall is authored.  75x RANGE_TOL, so
# ``range_static``/``range_tracks`` go red decisively while the SWEEP is
# untouched and ``sweep_span`` stays green.
RUNG5_FAULT_SHIFT = 0.15                 # m

# Rung 6: how far past the trigger the ``bounce`` fault drives before it is put
# back.  0.2 m is 7x RUNG6_STOP_BOUND, so ``min_gap`` is unambiguously red,
# and the rover is then placed at exactly the gap ``stop_gap`` expects so that
# check stays green.  That asymmetry is the whole argument for a whole-run
# invariant.
RUNG6_FAULT_BOUNCE_M = 0.2               # m past the threshold
RUNG6_FAULT_REST_GAP = RUNG6_STOP_GAP - RUNG6_STOP_BOUND / 2.0

# Rung 7: which robot is broken, and how far out of its lane ``lane_offset``
# spawns it.  0.5 m is 5x LATERAL_TOL and half the 1.0 m of air between lanes,
# so ``min_separation`` goes red without the robots ever touching -- the rung
# asserts non-interference, not crash avoidance, and the fault must respect
# that distinction or it would be proving the wrong check.
#
# IT IS A SCENE OFFSET, NOT A WALK, AND THAT IS A CORRECTION.  The first
# version had the supervisor write the robot's y every step while reading its
# x and z back, on the theory that the forward motion would be untouched.
# MEASURED: it was not.  A per-step field write costs the body its state --
# ``distance_worst`` came back 0.94 (the robot travelled 6% of its target) and
# ``wheel_omega_worst`` 0.93, so BOTH must-green companions went red and the
# fault proved nothing about ``min_separation`` that it did not also destroy.
# Spawning the robot in the wrong place instead leaves every per-robot number
# untouched, which is what a surgical fault has to do.
RUNG7_FAULT_ROBOT = 2                    # index into RUNG7_TAGS (the centre)
RUNG7_FAULT_OFFSET_M = 0.5               # m, toward +y (into r3's lane)

# Rung 11 runs the SAME fault at a 16-robot fleet size, and it is the same
# magnitude on purpose: a red that meant one thing at N = 5 and another at
# N = 16 would not be comparable across the two rungs.
RUNG11_FAULT_OFFSET_M = RUNG7_FAULT_OFFSET_M


# --------------------------------------------------------------------------
# The checks
# --------------------------------------------------------------------------

class Check:
    """One numeric assertion.  ``ok`` is computed, never asserted by hand."""

    def __init__(self, name, measured, expected, tol, unit, why, rel=False,
                 key=None):
        self.name = name
        self.measured = measured
        self.expected = expected
        self.tol = tol
        self.unit = unit
        self.why = why
        self.rel = rel
        # The measurement key this check reads.  The self-test mutates by KEY,
        # so every assertion can be shown to go red on its own quantity.
        self.key = key or name
        # Set only by ``apply_not_expressible`` from an arm's OWN declaration.
        # Never inferred here, and never from a failed run.
        self.not_expressible = None
        self.ne_invalid = None

    @property
    def error(self):
        if self.measured is None:
            return None
        if self.rel:
            denom = abs(self.expected) if self.expected else 1.0
            return (self.measured - self.expected) / denom
        return self.measured - self.expected

    @property
    def margin(self):
        """How much tolerance is left.  Negative == failed, and by how much."""
        e = self.error
        return None if e is None else self.tol - abs(e)

    @property
    def ok(self):
        m = self.margin
        return bool(m is not None and m >= 0.0)

    def as_dict(self):
        d = {
            "name": self.name, "measured": self.measured,
            "expected": self.expected, "tol": self.tol, "unit": self.unit,
            "rel": self.rel, "error": self.error, "margin": self.margin,
            "ok": self.ok, "why": self.why, "key": self.key,
        }
        if self.not_expressible:
            # ``ok`` stays False and is IGNORED by the runner for an N/E check:
            # not green, not red, and it does not set the exit code.  Keeping
            # the raw field rather than faking a green is the point -- an N/E
            # that were drawn as a pass would flatter every arm that lacks a
            # capability, which is the failure amendment B exists to prevent.
            d["not_expressible"] = dict(self.not_expressible)
        if self.ne_invalid:
            d["ne_invalid"] = self.ne_invalid
        return d


def check_rung(rung, m):
    """Ground truth for one rung.  ``m`` is the measurement dict produced by
    ``analysis.reduce_samples``; returns a list of :class:`Check`.

    ``m`` values may be ``None`` (the arm could not measure that quantity) --
    a ``None`` measurement is a FAILED check, never a skipped one.  "We did not
    look" must never read as "nothing was wrong"; that is exactly the failure
    mode of the log-only PASS this ladder exists to replace.
    """
    rung = int(rung)
    if rung == 0:
        return [
            Check("steps_completed", m.get("steps"), float(RUNG0_STEPS), 0.0,
                  "steps",
                  "the controller must complete every step it asked for; a "
                  "run that exits 0 having stepped zero times is the stale-"
                  "libController signature, and it looks identical to a pass",
                  key="steps"),
            Check("exit_code", m.get("exit_code"), 0.0, 0.0, "rc",
                  "a clean exit is part of the rung"),
            Check("finite_clock", m.get("sim_time_end"),
                  float(RUNG0_STEPS) * DT, 2.0 * DT, "s",
                  "the simulated clock must advance by steps x dt",
                  key="sim_time_end"),
        ]

    if rung == 1:
        return [
            Check("rest_z", m.get("rest_z"), REST_Z, REST_Z_TOL, "m",
                  "a box of half-height %.3f m resting on a floor whose top "
                  "is at z=%.3f m has its centre at z=%.3f m; contact "
                  "compliance may sink it by up to g*tau^2 ~ 4 mm"
                  % (BOX_HALF, FLOOR_TOP, REST_Z)),
            Check("contact_penetration", m.get("penetration"), 0.0,
                  PENETRATION_TOL, "m",
                  "how far the box sank INTO the floor.  Failing this while "
                  "rest_z passes means the contact is softer than the "
                  "solver's own defaults imply -- a different claim from "
                  "'the body is on the wrong surface'", key="penetration"),
            Check("z_drift", m.get("z_drift"), 0.0, REST_Z_TOL, "m",
                  "a body at rest must stay at rest: peak-to-peak z over the "
                  "settle window is zero for a body in equilibrium"),
        ]

    if rung == 2:
        return [
            Check("spawn_z", m.get("spawn_z"), RUNG2_SPAWN_Z, SPAWN_Z_TOL,
                  "m",
                  "the first pose the controller observes must be the pose "
                  "the world authored (z=%.2f m)" % RUNG2_SPAWN_Z),
            Check("fall_interval", m.get("fall_interval"),
                  fall_interval_s(RUNG2_SPAWN_Z, RUNG2_GATE_HI,
                                  RUNG2_GATE_LO),
                  FALL_INTERVAL_TOL, "s",
                  "free fall from rest at z=%.2f m: the interval between "
                  "crossing z=%.2f m and z=%.2f m is "
                  "sqrt(2(z0-z2)/g)-sqrt(2(z0-z1)/g); clock-shift invariant"
                  % (RUNG2_SPAWN_Z, RUNG2_GATE_HI, RUNG2_GATE_LO)),
            Check("fall_time_abs", m.get("fall_time_abs"),
                  fall_time_s(RUNG2_DROP_M), 4.0 * DT, "s",
                  "absolute time from the controller's t=0 to first contact, "
                  "sqrt(2d/g) for d=%.2f m; a shifted clock shows up here and "
                  "NOT in fall_interval, which is how the two separate"
                  % RUNG2_DROP_M),
            Check("rest_z", m.get("rest_z"), REST_Z, REST_Z_TOL, "m",
                  "after the drop the box must come to rest at the same "
                  "analytic height as rung 1"),
        ]

    if rung == 3:
        return [
            Check("omega_driven", m.get("omega_driven"), RUNG3_OMEGA_CMD,
                  omega_tol(RUNG3_OMEGA_CMD), "rad/s",
                  "an unloaded velocity-controlled joint reaches its command "
                  "exactly at steady state: no gravity torque about a "
                  "vertical axis, no damping, no contact"),
            Check("omega_zero", m.get("omega_zero"), 0.0, OMEGA_ZERO_TOL,
                  "rad/s",
                  "commanding zero must PRODUCE zero -- the case a servo that "
                  "silently ignores its input passes only by accident"),
            Check("angle_travelled", m.get("angle_driven"),
                  RUNG3_OMEGA_CMD * (RUNG3_WIN_A[1] - RUNG3_WIN_A[0]),
                  omega_tol(RUNG3_OMEGA_CMD) * (RUNG3_WIN_A[1]
                                                - RUNG3_WIN_A[0]),
                  "rad",
                  "integral form of the same claim, read from the joint's own "
                  "position sensor: theta = omega t", key="angle_driven"),
        ]

    if rung == 4:
        win = RUNG4_WIN[1] - RUNG4_WIN[0]
        v_expected = rolling_speed(RUNG4_OMEGA_CMD)
        return [
            Check("distance", m.get("distance"), v_expected * win,
                  DISTANCE_TOL, "m",
                  "d = omega r t for a wheel rolling without slip, measured "
                  "over a window that starts after the spin-up transient",
                  rel=True),
            Check("wheel_omega", m.get("wheel_omega"), RUNG4_OMEGA_CMD,
                  omega_tol(RUNG4_OMEGA_CMD), "rad/s",
                  "the wheels must actually TURN at the commanded rate -- "
                  "read from each wheel's own position sensor"),
            Check("rolling_consistency", m.get("roll_ratio"), 1.0,
                  ROLL_CONSISTENCY_TOL, "-",
                  "v_body / (omega_wheel r) == 1.  THE anti-slide assertion: "
                  "a chassis that slid the right distance on wheels that "
                  "never turned passes 'distance' and fails only here",
                  rel=True, key="roll_ratio"),
            Check("roll_overrun", m.get("roll_overrun"), 0.0,
                  ROLL_OVERRUN_TOL, "-",
                  "max over the WHOLE run of (v_body/(omega_wheel r) - 1), "
                  "clamped at 0.  A wheel can push a body at most omega r and "
                  "slip only makes it slower, so any excess is unphysical.  "
                  "This is the check a steady-state window cannot make"),
            Check("ride_height", m.get("ride_dev"), 0.0, RIDE_HEIGHT_TOL, "m",
                  "the axle of a suspensionless rover on flat ground stays at "
                  "floor_top + wheel_radius; a rover being launched, or one "
                  "that has driven off the edge, breaks every other number in "
                  "this rung while leaving them looking plausible",
                  key="ride_dev"),
            Check("lateral_drift", m.get("lateral"), 0.0, LATERAL_TOL, "m",
                  "a symmetric robot given identical wheel commands on flat "
                  "ground travels straight", key="lateral"),
        ]

    if rung == 5:
        return [
            Check("range_static", m.get("range_static"), RUNG5_STANDOFF,
                  RANGE_TOL, "m",
                  "with the carrier parked at x=%.2f m the sensor sits at "
                  "x=%.2f m and the wall's near face at x=%.2f m, so the ray "
                  "is %.2f m long -- pure geometry, no dynamics in it"
                  % (RUNG5_X0, RUNG5_X0 + RUNG5_SENSOR_DX, RUNG5_WALL_FACE_X,
                     RUNG5_STANDOFF)),
            Check("range_final", m.get("range_final"), RUNG5_FINAL_RANGE,
                  RANGE_TOL, "m",
                  "after sweeping %.2f m the same geometry gives %.2f m.  THE "
                  "READING MUST CHANGE: a sensor frozen at its t=0 value "
                  "passes range_static and fails only here"
                  % (RUNG5_TRAVEL, RUNG5_FINAL_RANGE)),
            Check("range_tracks", m.get("range_residual"), 0.0, RANGE_TOL, "m",
                  "max over the WHOLE run of |reading - (wall face - sensor "
                  "x)|, the sensor read against the pose recorded on the same "
                  "step.  A windowed check sees two instants; this sees every "
                  "one, which is what a sensor that stops updating needs",
                  key="range_residual"),
            Check("sweep_span", m.get("sweep_span"), RUNG5_TRAVEL, RANGE_TOL,
                  "m",
                  "the carrier's own peak-to-peak x.  It separates 'the "
                  "sensor is frozen' from 'the scene is frozen' -- without it "
                  "a world that never moved would be reported as a dead "
                  "sensor"),
        ]

    if rung == 6:
        return [
            Check("stop_gap", m.get("stop_gap"),
                  RUNG6_STOP_GAP - RUNG6_STOP_BOUND / 2.0,
                  RUNG6_STOP_BOUND / 2.0, "m",
                  "the rover stops when the sensor first reads below %.2f m, "
                  "and may then travel at most %.1f mm further (3 steps of "
                  "latency plus 3x the friction-limited braking distance "
                  "v^2/2 mu g).  MEASURED FROM THE POSE, so a fabricated "
                  "sensor cannot grade itself"
                  % (RUNG6_STOP_GAP, 1000.0 * RUNG6_STOP_BOUND)),
            Check("min_gap", m.get("min_gap"),
                  RUNG6_STOP_GAP - RUNG6_STOP_BOUND / 2.0,
                  RUNG6_STOP_BOUND / 2.0, "m",
                  "the SMALLEST gap over the whole run.  A rover that ran "
                  "into the wall and rebounded to a plausible resting place "
                  "passes stop_gap and fails only here -- the same shape as "
                  "rung 4's roll_overrun, and the reason a final-state check "
                  "is never enough"),
            Check("trigger_reading", m.get("trigger_reading"),
                  RUNG6_STOP_GAP - 0.5 * RUNG6_CRUISE_V * DT,
                  RUNG6_TRIGGER_TOL, "m",
                  "the sensor value on the step the stop was commanded: it "
                  "must be the threshold, reached from above within one step "
                  "of travel.  Unmeasurable -- and therefore RED -- when the "
                  "sensor never crossed the threshold at all"),
            Check("sensor_agrees", m.get("range_residual"), 0.0, RANGE_TOL,
                  "m",
                  "rung 5's claim under real locomotion: max over the whole "
                  "run of |reading - (wall face - sensor x)|, with the sensor "
                  "pose now produced by the wheels rather than written by the "
                  "driver", key="range_residual"),
            Check("stop_creep", m.get("stop_creep"), 0.0, REST_Z_TOL, "m",
                  "peak-to-peak x over the last %.1f s.  A stopped rover "
                  "stays stopped; the tolerance is the same contact-"
                  "compliance bound as a resting box" % RUNG6_SETTLE_WINDOW),
            Check("wheel_stop", m.get("wheel_stop"), 0.0, OMEGA_ZERO_TOL,
                  "rad/s",
                  "the wheels themselves must be at zero, not merely the "
                  "body: a chassis held still by friction while its wheels "
                  "spin passes stop_creep and fails here"),
        ]

    if rung == 7:
        win = RUNG7_WIN[1] - RUNG7_WIN[0]
        return [
            Check("distance_worst", m.get("distance_worst"), 0.0,
                  DISTANCE_TOL, "-",
                  "worst relative error over the %d robots of d_i vs "
                  "omega_i r t, each judged against the distance it would "
                  "travel ALONE over the %.1f s window.  This IS the "
                  "non-interference assertion: a robot perturbed by a "
                  "neighbour misses its own solo target"
                  % (RUNG7_N, win)),
            Check("wheel_omega_worst", m.get("wheel_omega_worst"), 0.0,
                  OMEGA_REL_TOL, "-",
                  "worst relative error of any robot's mean wheel rate "
                  "against ITS OWN command; five different commands, so a "
                  "command that leaked between robots shows up here"),
            Check("min_separation", m.get("min_separation"), RUNG7_LANE_DY,
                  LATERAL_TOL, "m",
                  "minimum over the WHOLE run of every robot pair's planar "
                  "centre distance.  All five start abreast %.1f m apart and "
                  "travel at different speeds, so they can only separate: the "
                  "minimum is the lane spacing, at t=0.  Asserted "
                  "geometrically rather than as a contact count, because a "
                  "contact read that returns nothing is indistinguishable "
                  "from nothing touching" % RUNG7_LANE_DY),
            Check("lateral_worst", m.get("lateral_worst"), 0.0, LATERAL_TOL,
                  "m",
                  "worst |y - y_0| over every robot and every sample: each "
                  "robot must stay in its own lane"),
            Check("roll_overrun_worst", m.get("roll_overrun_worst"), 0.0,
                  ROLL_OVERRUN_TOL, "-",
                  "rung 4's one-sided kinematic invariant, over the fleet and "
                  "the whole run: no body may outrun its own wheels"),
            Check("ride_worst", m.get("ride_worst"), 0.0, RIDE_HEIGHT_TOL, "m",
                  "worst axle deviation from floor_top + wheel_radius over "
                  "every robot and every sample after the initial settle"),
        ]

    if rung == 8:
        return [
            Check("part_rest_z", m.get("part_rest_z"), RUNG8_PART_Z0,
                  REST_Z_TOL, "m",
                  "before the fingers close, the payload rests where geometry "
                  "says: table top %.2f m plus half its %.2f m edge.  It "
                  "separates 'the grasp failed' from 'the scene was already "
                  "wrong'" % (RUNG8_TABLE_TOP, RUNG8_PART_EDGE)),
            Check("carry_rel", m.get("carry_rel"), 0.0, RUNG8_POSE_TOL, "m",
                  "max over the WHOLE run of |r_payload - r_wrist|.  The "
                  "wrist origin is AUTHORED at the payload's centre, so this "
                  "expectation is an analytic zero rather than a value read "
                  "from the run -- the payload tracks the gripper, or it does "
                  "not"),
            Check("lift_height", m.get("lift_height"), RUNG8_PART_TARGET_Z,
                  RUNG8_POSE_TOL, "m",
                  "payload centre while held at the target: rest height "
                  "%.2f m plus the commanded %.2f m lift"
                  % (RUNG8_PART_Z0, RUNG8_LIFT_H)),
            Check("place_x", m.get("place_x"), RUNG8_TRAVERSE_X,
                  RUNG8_POSE_TOL, "m",
                  "the payload ends at the commanded target, %.2f m from "
                  "where it was picked up -- %.0fx its own edge, so a carry "
                  "cannot be confused with a jiggle"
                  % (RUNG8_TRAVERSE_X, RUNG8_TRAVERSE_X / RUNG8_PART_EDGE)),
            Check("hold_clearance", m.get("hold_clearance"), RUNG8_LIFT_H,
                  RUNG8_POSE_TOL, "m",
                  "minimum over the whole carry of (payload underside - table "
                  "top).  AIRBORNE, asserted over every sample of the carry "
                  "rather than at the end of it, so a payload that touched "
                  "down mid-traverse cannot be rescued by where it finished",
                  key="hold_clearance"),
            Check("part_speed_max", m.get("part_speed_max"), 0.0,
                  RUNG8_SPEED_BOUND, "m/s",
                  "fastest the payload ever moves, read over %.0f ms windows "
                  "so sampling noise cannot set it.  The measured failure "
                  "this guards is a pinch that EJECTED its part at 3.5 m/s "
                  "and left it on the gripper's own wrist plate"
                  % (1000.0 * RUNG8_SPEED_SPAN)),
        ]

    if rung == 9:
        return [
            Check("repeat_delta", m.get("repeat_delta"), 0.0, 0.0, "m",
                  "max over EVERY body, coordinate and sample of |a - b| for "
                  "two runs of the identical scene from two fresh processes.  "
                  "Identical inputs, identical code path, one machine and one "
                  "build: anything but zero is state that leaked between runs "
                  "or an ordering that is not reproducible.  Whole-run, so two "
                  "runs that diverge at t=2 and reconverge by chance at t=8 "
                  "still fail.  ! SCOPED TO THE PRECISION THE POSE READBACK "
                  "CARRIES, which is MEASURED to be single: a cube authored at "
                  "x=0.502 reads back as 0.50199997425079346 = float32(0.502), "
                  "so a divergence living entirely below ~6e-8 m would be "
                  "invisible here.  The refutation this rung exists to see is "
                  "9.152 m by 1000 steps, eight orders above that"),
            Check("repeat_length", m.get("repeat_length"), 0.0, 0.0, "steps",
                  "|steps(a) - steps(b)|.  A run that ended early has a "
                  "trivially small delta OVER THE OVERLAP, which is how a "
                  "truncated replica passes a determinism check"),
            Check("sensitivity_shortfall", m.get("sensitivity_shortfall"),
                  0.0, 0.0, "x",
                  "max(0, %g - max|a - c| / %.0e), c being the same scene with "
                  "the dropped cube's spawn x moved by %.0e m.  THE CONTROL: a "
                  "frozen world is perfectly deterministic, so repeat_delta "
                  "alone is not evidence.  One-sided (the roll_overrun idiom).  "
                  "It is a FLOOR on 'the scene is not frozen' -- a frozen world "
                  "amplifies exactly 1 and round-off about 1e-8 of the seed -- "
                  "and NOT a grade on how chaotic an engine is: the measured "
                  "amplification is reported beside the row and is not judged"
                  % (RUNG9_AMPLIFY_MIN, RUNG9_EPS, RUNG9_EPS)),
            Check("fall_interval", m.get("fall_interval"),
                  fall_interval_s(RUNG9_SPAWN_Z, RUNG9_GATE_HI,
                                  RUNG9_GATE_LO),
                  FALL_INTERVAL_TOL, "s",
                  "rung 2's derivation, unchanged, on the dropped cube.  THE "
                  "ANALYTIC ANCHOR: an engine that is deterministic and WRONG "
                  "passes both determinism checks -- gravity at 5 m/s^2 is "
                  "exactly as reproducible as 9.81 -- and this is the only "
                  "check here that sees it"),
            Check("distinct_processes", m.get("distinct_processes"),
                  float(len(RUNG9_RUNS)), 0.0, "processes",
                  "the replicas must come from DISTINCT PROCESSES, asserted "
                  "from the pid and process-start time each run records.  A "
                  "determinism rung whose two replicas are one process -- or "
                  "one array copied -- measures the arm.  This is the one "
                  "place the ladder checks the arm rather than the engine, "
                  "and it is cheap", key="distinct_processes"),
        ]

    if rung == 11:
        return [
            Check("distance_worst", m.get("distance_worst"), 0.0,
                  DISTANCE_TOL, "-",
                  "worst relative error, over every robot of every fleet size "
                  "%s, of d_i against omega_i r t -- the distance that robot "
                  "would travel ALONE.  The tolerance is rung 4's and is NOT "
                  "widened with N: the failure this rung exists to catch is a "
                  "silently truncated constraint vector, documented at 9%% "
                  "displacement error, so a tolerance that grew with N would "
                  "be pre-authorised to miss it" % (RUNG11_N,)),
            Check("wheel_omega_worst", m.get("wheel_omega_worst"), 0.0,
                  OMEGA_REL_TOL, "-",
                  "worst relative error of any robot's mean wheel rate "
                  "against ITS OWN command.  The commands cycle, so an engine "
                  "that simulates one robot and copies it lands every clone "
                  "on one target and reds here"),
            Check("roll_overrun_worst", m.get("roll_overrun_worst"), 0.0,
                  ROLL_OVERRUN_TOL, "-",
                  "rung 4's one-sided kinematic invariant over the whole "
                  "fleet and the whole run: no body may outrun its own "
                  "wheels, at any N"),
            Check("ride_worst", m.get("ride_worst"), 0.0, RIDE_HEIGHT_TOL, "m",
                  "worst axle deviation from floor_top + wheel_radius over "
                  "every robot, every fleet and every sample after the "
                  "initial settle"),
            Check("lateral_worst", m.get("lateral_worst"), 0.0, LATERAL_TOL,
                  "m",
                  "worst |y - y_0| over every robot and every sample: each "
                  "robot stays in its own lane at every fleet size"),
            Check("separation_shortfall", m.get("separation_shortfall"), 0.0,
                  LATERAL_TOL, "m",
                  "max over every fleet of max(0, %.1f - min pairwise centre "
                  "distance over the whole run).  All robots start abreast one "
                  "lane apart and travel at different speeds, so they can only "
                  "separate.  Asserted GEOMETRICALLY from the poses rather "
                  "than as a contact count, because this tree has shipped a "
                  "contact read that returned an empty set for a scene "
                  "containing 1008 contacts" % RUNG11_LANE_DY),
            Check("robots_seen", m.get("robots_seen"),
                  float(RUNG11_ROBOTS_TOTAL), 0.0, "robots",
                  "every robot of every fleet was found and measured.  An "
                  "engine that silently DROPS robots otherwise reports a "
                  "well-behaved smaller fleet, and every other number in this "
                  "rung would look fine"),
        ]

    if rung == 18:
        return [
            Check("real_pos_err", m.get("real_pos_err"), 0.0,
                  RUNG18_POS_BOUND, "% of cube width",
                  "mean over %d recorded tosses of the mean ||p_sim - "
                  "p_real|| along the trajectory, as a percentage of cube "
                  "width.  The bound is the PUBLISHED MuJoCo baseline plus one "
                  "published standard deviation (%.1f + %.1f), set by a paper "
                  "that never heard of this project.  IT IS NOT 'we win': the "
                  "best engine measured on this data scores %.1f%%"
                  % (len(RUNG18_INDICES),
                     RUNG18_BASELINES["MuJoCo"]["pos_pct"],
                     RUNG18_BASELINES["MuJoCo"]["pos_sd"],
                     RUNG18_BASELINES["Drake"]["pos_pct"])),
            Check("real_rot_err", m.get("real_rot_err"), 0.0,
                  RUNG18_ROT_BOUND, "deg",
                  "same construction on the geodesic angle between the "
                  "simulated and recorded orientations (%.1f + %.1f)"
                  % (RUNG18_BASELINES["MuJoCo"]["rot_deg"],
                     RUNG18_BASELINES["MuJoCo"]["rot_sd"])),
            Check("tunnel_depth", m.get("tunnel_depth"), 0.0,
                  RUNG18_TUNNEL_TOL, "m",
                  "max over every sample of every toss of how far the cube's "
                  "TOP FACE went below the table's top surface.  Whole-run: a "
                  "replay that fell through the table can still score a "
                  "plausible MEAN error.  Not a penetration check -- a "
                  "compliant contact legitimately sinks this cube ~22 mm at "
                  "these impact speeds; passing the top face through needs "
                  "52 mm and is geometrically unambiguous"),
            Check("replay_ic_fidelity", m.get("ic_shortfall"), 0.0,
                  RUNG18_IC_TOL, "-",
                  "worst over the tosses of the engine's own readback of the "
                  "velocity and angular velocity it accepted, against the "
                  "recorded initial condition it was given, less one step of "
                  "gravity.  THIS EXISTS BECAUSE THIS TREE HAS SHIPPED "
                  "setVelocity BEING SILENTLY DROPPED AT t = 0 -- a defect "
                  "that would otherwise present as poor real-world agreement "
                  "and be charged to the contact model", key="ic_shortfall"),
            # Named for what it MEASURES, not for what it hopes: an earlier
            # name (``tosses_scored``) printed "measured 0" for a perfect cell,
            # which reads as a zero score in the table.
            Check("tosses_unscored", m.get("tosses_missing"), 0.0, 0.0,
                  "tosses",
                  "how many of the contract's tosses produced no scored "
                  "trajectory.  RED, never skipped, and RED too if the arm ran "
                  "a subset the contract does not own -- an arm free to pick "
                  "its own tosses could pick the easy ones",
                  key="tosses_missing"),
        ]

    raise ValueError("no such rung: %r" % (rung,))


def check_rung18_embed_gap(m_ours, m_reference):
    """The cross-arm translation-fidelity check.  CONTRACT.md section 3c.

    It cannot live in ``check_rung`` because it reads TWO cells: ours and the
    bare-MuJoCo reference's, on the same tosses, against the same recording,
    reduced by the same code.  ``run_ladder`` calls it once both are present.

    Our solver IS MuJoCo, so any gap between the two is our translation layer
    and not the physics.  It is the only check on this ladder we can lose and
    cannot win, which is exactly why it is the headline and the two
    agreement-with-reality bounds are floors.

    Returns ``None`` when either side has no per-toss record -- the runner
    reports that as N/E with the reason, never as a pass.
    """
    ours = (m_ours or {}).get("per_toss") or {}
    ref = (m_reference or {}).get("per_toss") or {}
    shared = sorted(set(ours) & set(ref))
    gap = None
    if shared:
        a = sum(ours[k]["pos_pct"] for k in shared) / len(shared)
        b = sum(ref[k]["pos_pct"] for k in shared) / len(shared)
        gap = abs(a - b)
    return Check(
        "embed_gap", gap, 0.0, RUNG18_EMBED_GAP_TOL, "% of cube width",
        "|our mean position error - the BARE MuJoCo arm's, on the %d tosses "
        "both arms scored|.  Our solver IS MuJoCo, so this is our translation "
        "layer measured against the engine we embed, with a ruler neither of "
        "us wrote.  THE ONLY CHECK HERE WE CAN ONLY FAIL" % len(shared),
        key="embed_gap")


#: Every-commit rungs.  ``--rungs all`` means THIS set.
RUNGS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11)
#: Rungs that cost minutes rather than seconds and must be asked for by number
#: (or with ``--rungs everything``).  Rung 18 launches one engine per toss.
RUNGS_ON_DEMAND = (18,)
ALL_RUNGS = tuple(sorted(RUNGS + RUNGS_ON_DEMAND))

RUNG_TITLE = {
    0: "empty world loads, steps, exits clean",
    1: "box resting on a floor",
    2: "box dropped from height",
    3: "one hinge + motor, no load",
    4: "wheeled robot, straight line",
    5: "distance sensor swept toward a wall",
    6: "drive forward, stop at a sensed threshold",
    7: "five robots, five commands, one floor",
    8: "gripper lifts a payload off a table",
    9: "same pile twice, plus a 1 um sensitivity control",
    11: "the same rover at N = 1, 4, 8, 16",
    18: "replaying 50 recorded real cube tosses",
}

DURATION = {
    0: RUNG0_DURATION, 1: RUNG1_DURATION, 2: RUNG2_DURATION,
    3: RUNG3_DURATION, 4: RUNG4_DURATION, 5: RUNG5_DURATION,
    6: RUNG6_DURATION, 7: RUNG7_DURATION, 8: RUNG8_DURATION,
    9: RUNG9_DURATION, 11: RUNG11_DURATION,
    # Per TOSS, not per cell: rung 18's cell is 50 of these.
    18: RUNG18_TOSS_S,
}

#: Rungs whose cell is more than one RUN of one scene family (amendment A).
MULTI_RUN = {
    9: tuple(t for t, _e in RUNG9_RUNS),
    11: tuple("n%d" % n for n in RUNG11_N),
    18: tuple("toss%04d" % i for i in RUNG18_INDICES),
}


# --------------------------------------------------------------------------
# NOT_EXPRESSIBLE -- CONTRACT.md amendment B
# --------------------------------------------------------------------------

class NotExpressible(RuntimeError):
    """An arm declared a refusal and a measurement for the same quantity.

    Fatal on purpose, like ``ArmImportCollision``.  A refusal and a number
    cannot both be true, and whichever one is wrong the row is not a
    measurement of anything.
    """


def apply_not_expressible(checks, declared):
    """Mark checks an arm has DECLARED it cannot express.

    ``declared`` is ``meta["not_expressible"]``: ``{check_name: {"missing":
    ..., "citation": ..., "status": ...}}``.  Rules, all load-bearing
    (CONTRACT.md amendment B):

    1. an N/E check is neither green nor red and does not set the exit code;
    2. a declaration missing ``missing`` or ``citation`` is judged RED -- "we
       did not look" must never read as "nothing was wrong";
    3. an arm that declares N/E for a check it also produced a number for
       raises :class:`NotExpressible` and stops the run;
    4. N/E is DECLARED IN THE ARM'S SOURCE, never inferred from a failed run.
       An arm that tried and failed reports RED.  This distinction is the whole
       value of the verdict and the easiest one to lose, so nothing in this
       function can produce an N/E that the arm did not write down.
    """
    declared = declared or {}
    out = []
    for chk in checks:
        d = declared.get(chk.name)
        if d is None:
            out.append(chk)
            continue
        if chk.measured is not None:
            raise NotExpressible(
                "arm declares %r NOT_EXPRESSIBLE and also measured it (%r).  "
                "A refusal and a measurement of the same quantity cannot both "
                "be true." % (chk.name, chk.measured))
        if not (isinstance(d, dict) and d.get("missing") and d.get("citation")):
            chk.ne_invalid = ("declaration lacks 'missing' and/or 'citation', "
                              "so it is judged RED rather than skipped")
            out.append(chk)
            continue
        chk.not_expressible = dict(d)
        out.append(chk)
    return out
