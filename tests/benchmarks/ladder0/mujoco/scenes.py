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

"""scenes.py -- the ladder's scenes, authored as MJCF.

Every geometric and physical number is read out of the ladder's shared
contract (``shared.spec``); nothing is written twice.  Changing a scene
constant there changes this MJCF and the analytic expectation together, so
they cannot drift apart.

Rungs 0-8 are one scene each.  Rungs 9, 11 and 18 are scene FAMILIES
(CONTRACT.md amendment A): one cell is several RUNS of one generator, and
:func:`run_specs` -- not this arm -- names them, from the contract.  Rung 18 is
the one scene in this file whose numbers are not the ladder's at all: it is a
translation of ``tests/benchmarks/omnibench/lane1r``'s cube-toss world, read
from that lane at emit time and cross-checked against its dataset module,
because rung 18's ground truth is a recording of physical reality and lane1r
owns it.

MUJOCO CONVENTIONS THAT DIFFER FROM VRML/.wbt
---------------------------------------------
These are the translations a reader comparing the three scene files needs, and
every one of them is a place a hand-written port can silently halve or double
a dimension:

* ``geom size`` is a HALF-extent.  A 0.2 m cube is ``size="0.1 0.1 0.1"``;
  the VRML ``Box.size`` for the same cube is ``0.2 0.2 0.2``.
* The ``cylinder`` primitive's axis is +Z, where Webots' is +Y.  Rather than
  carry a quaternion, every cylinder here uses ``fromto``, which states the
  two endpoints directly and leaves ``size`` meaning just the radius.  The
  axis is then visible in the file instead of encoded in a rotation.
* Mass: MJCF derives inertia from geometry and DENSITY (default 1000 kg/m^3)
  unless an explicit ``mass`` is given.  Every geom here declares ``mass``, so
  the inertia is the shape's own tensor scaled to the contract's mass -- the
  0.2 m cube would otherwise weigh 8 kg instead of 1 kg.
* There is NO implicit ground plane.  MuJoCo simulates exactly the geoms in
  the file, so the phantom z=0 collision plane that motivated lifting this
  floor to z=0.5 cannot exist here.  The floor is still lifted, because the
  three arms must run the same scene.

SOLVER SETTINGS
---------------
Deliberately MuJoCo's own defaults, with only gravity and timestep set from
the contract.  This arm is a reference implementation of the solver OmniSim
runs underneath, so it should be the un-tuned version of it: any setting this
file overrides is a setting whose effect gets attributed to the file instead
of to the engine.

EXACTLY TWO RUNGS DEVIATE, both on rung 8 and both stated in one place
(``scene_mu`` and ``solver_options``):

* ``friction`` is ``RUNG8_MU`` rather than ``MU`` -- the CONTRACT raises it,
  so that rung 8 measures "does a grasp with a large Coulomb margin hold"
  rather than "where is this engine's friction limit".
* ``noslip_iterations`` is turned on, because A GRASP IS NOT EXPRESSIBLE AT
  MUJOCO'S DEFAULTS.  With the default 0, this scene's part slid 44 mm down
  the pads during the lift and fell out, while the normal force stayed at its
  commanded value throughout.  ``solver_options`` carries the measurement,
  the convergence table that fixes the iteration count, and the two pieces of
  standard advice (elliptic cone, raised impratio) that made it WORSE.

The defaults in force (MuJoCo 3.x) are

    integrator  Euler        solver     Newton        iterations   100
    cone        pyramidal    impratio   1             tolerance    1e-8
    condim      3            solref     0.02 1        solimp  .9 .95 .001

``solref = (0.02 s, 1)`` is the one the contract's 5 mm rest tolerance is
derived from: a critically damped contact whose reference acceleration is
-x/tau^2 puts a body in gravitational equilibrium at penetration g*tau^2 =
3.9 mm, independent of mass.

``condim 3`` means contacts carry sliding friction only -- torsional and
rolling friction are off.  That is stated here because it matters to rung 4:
the wheels meet the ground with no rolling resistance, so a wheel turning at
the commanded rate is what the contract's v = omega*r expects, with no
velocity-dependent drag term hiding in the residual.  The friction triplet is
still declared in full so the file is self-describing.

VELOCITY SERVOS
---------------
Rungs 3 and 4 need the MJCF analogue of ``setVelocity`` on a Webots motor: the
``<velocity kv>`` actuator, which applies torque kv*(ctrl - qvel).  On an
UNLOADED joint its steady state is exact -- at qvel == ctrl the torque is zero
and there is nothing else to accelerate the joint -- so it cannot introduce a
steady-state bias into rung 3's assertion.

``kv`` is not free, though: MuJoCo's default Euler integrator treats actuator
damping EXPLICITLY (only joint damping is folded into the mass matrix), so the
velocity error decays by a factor (1 - kv*dt/I) per step and the servo
diverges once kv*dt/I >= 2.  Rather than pick a gain and check it, each kv
below is set to the DEADBEAT value

    kv = I / dt        i.e. kv*dt/I = 1, the factor is exactly 0

which annihilates the velocity error in a single step and sits at exactly half
the stability bound -- a 2x margin, derived rather than chosen.  Neither gain
was tuned against a measurement, and ``run.py`` re-derives kv*dt/I from the
COMPILED model's own ``dof_M0`` and reports it, so the margin is a measured
field in the results rather than a claim in a comment.

An unloaded deadbeat servo settles in one 4 ms step, five hundred times inside
the 0.5 s spin-up rung 3 discards and two thousand times inside rung 4's 2 s.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
MODELS = os.path.join(HERE, "models")


def _load_shared():
    """Bootstrap this arm's loader by path -- never ``import shared``.

    ``sys.modules`` is process-global and shared with the other two arms, so a
    generic module name is a collision waiting to happen; ``shared.py``'s
    docstring has the full argument.  Every other sibling then comes from
    ``shared.sibling()``.
    """
    key = "ladder0_mujoco_shared"
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "shared.py"))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


shared = _load_shared()
spec = shared.spec


# --------------------------------------------------------------------------
# velocity-servo gains, derived from the explicit-Euler stability bound
# --------------------------------------------------------------------------

KV_STABILITY_FRACTION = 0.5     # of the kv*dt/I < 2 explicit-Euler bound;
                                # 0.5 IS the deadbeat gain kv = I/dt


def _kv(inertia):
    """Deadbeat velocity-servo gain: ``kv = I/dt``.

    Half the explicit-Euler stability bound ``2 I / dt``, so the per-step
    velocity-error factor ``(1 - kv dt / I)`` is exactly zero.
    """
    return KV_STABILITY_FRACTION * 2.0 * inertia / spec.DT


# Rung 3 link: a cylinder of length L, radius r, mass m, hinged about a
# transverse axis through ONE END.  I = m (L^2/3 + r^2/4).
RUNG3_LINK_INERTIA = spec.RUNG3_LINK_MASS * (
    spec.RUNG3_LINK_LEN ** 2 / 3.0 + spec.RUNG3_LINK_RADIUS ** 2 / 4.0)
RUNG3_KV = _kv(RUNG3_LINK_INERTIA)

# Rung 4 wheel: a solid cylinder about its own axle.  I = m r^2 / 2.
RUNG4_WHEEL_INERTIA = 0.5 * spec.WHEEL_MASS * spec.WHEEL_R ** 2
RUNG4_KV = _kv(RUNG4_WHEEL_INERTIA)


# --------------------------------------------------------------------------
# MJCF fragments
# --------------------------------------------------------------------------

_HEADER = """<!-- GENERATED by tests/benchmarks/ladder0/mujoco/scenes.py.
     Ladder0 rung {rung} ({title}) -- MuJoCo arm.
     Every number here is read from the ladder's shared contract; do not
     hand-edit.  MJCF `size` is a HALF-extent where VRML `size` is a full
     extent, and `fromto` states a cylinder's axis explicitly. -->
<mujoco model="ladder0_rung{rung}_mujoco">

  <!-- Only gravity and the timestep are set from the contract.  Everything
       else is MuJoCo's default, on purpose: this arm is the un-tuned
       reference for the solver OmniSim runs underneath. -->
  <option timestep="{dt}" gravity="0 0 -{g}"{opt}/>

  <default>
    <!-- sliding / torsional / rolling.  condim defaults to 3, so only the
         first component is in force; the rest are declared for the record. -->
    <geom friction="{mu} 0.005 0.0001"/>
  </default>
"""

_FLOOR = """
  <worldbody>
    <!-- Floor slab: full extents {fx} x {fy} x {fz} m centred at z={cz},
         so its top surface is at z={top}.  Deliberately NOT at z=0. -->
    <geom name="floor" type="box" size="{hx} {hy} {hz}" pos="0 0 {cz}"
          rgba="0.4 0.42 0.45 1"/>
"""


def _box_body(name, z):
    return """
    <!-- {name}: {edge} m cube, {mass} kg, centre at z={z}. -->
    <body name="{name}" pos="0 0 {z}">
      <freejoint name="{name}_free"/>
      <geom name="{name}_geom" type="box" size="{h} {h} {h}" mass="{mass}"
            rgba="0.85 0.72 0.25 1"/>
    </body>
""".format(name=name, edge=spec.BOX_EDGE, mass=spec.BOX_MASS, z=z,
           h=spec.BOX_HALF)


def _wheel(name, x, y):
    """One driven wheel: hinge about +Y, cylinder stated by its endpoints.

    Positive omega about +Y drives the chassis along +X: the contact point at
    r = (0,0,-R) has velocity omega*Y_hat x r = -omega*R*X_hat relative to the
    body, so rolling without slip carries the body at +omega*R along X.
    """
    half_w = spec.WHEEL_W / 2.0
    return """
      <body name="{name}" pos="{x} {y} 0">
        <joint name="{name}_joint" type="hinge" axis="0 1 0"/>
        <geom name="{name}_geom" type="cylinder"
              fromto="0 {y0} 0  0 {y1} 0" size="{r}" mass="{mass}"
              rgba="0.15 0.15 0.18 1"/>
      </body>
""".format(name=name, x=x, y=y, y0=-half_w, y1=half_w, r=spec.WHEEL_R,
           mass=spec.WHEEL_MASS)


def _rover(prefix="", y0=0.0, sensor=None):
    """The rung-4 rover: chassis + four driven wheels, optionally sensored.

    ONE emitter for rungs 4, 6 and 7 so the three cannot drift apart.  Rung 6
    is "the rung-4 rover with a sensor" and rung 7 is "five of them in
    lanes"; if either carried its own copy of the chassis, a change to
    ``CHASSIS_SIZE`` would move one rung's scene and not the others', and the
    rows would still look comparable.

    ``prefix`` namespaces every body, joint, geom and site -- rung 4 passes
    "" and emits exactly the text it always has.
    """
    cx, cy, cz = spec.CHASSIS_SIZE
    site = ""
    if sensor is not None:
        dx, dz = sensor
        site = ('      <site name="%srange_site" pos="%s 0 %s" zaxis="1 0 0"\n'
                '            size="0.005" rgba="0.9 0.2 0.2 1"/>\n'
                % (prefix, dx, dz))
    out = """
    <!-- Chassis origin sits at the AXLE height, z = floor_top + wheel_r =
         {z}.  Its half-height is {chz}, so its underside clears the floor by
         {clear:.3f} m and never contacts: every ground contact in this scene
         is a wheel contact, which is the invariant the rung tests. -->
    <body name="{p}chassis" pos="0 {y0} {z}">
      <freejoint name="{p}chassis_free"/>
      <geom name="{p}chassis_geom" type="box" size="{chx} {chy} {chz}"
            mass="{cmass}" rgba="0.85 0.72 0.25 1"/>
{site}""".format(p=prefix, y0=y0, z=spec.ROBOT_Z, chx=cx / 2.0, chy=cy / 2.0,
                 chz=cz / 2.0, cmass=spec.CHASSIS_MASS, site=site,
                 clear=spec.ROBOT_Z - cz / 2.0 - spec.FLOOR_TOP)
    for name, sx, sy in spec.WHEELS:
        out += _wheel(prefix + name, sx * spec.WHEEL_X, sy * spec.WHEEL_Y)
    return out + "    </body>\n"


def _rover_actuators(prefix=""):
    """Four deadbeat velocity servos, one per wheel."""
    out = ("    <!-- Deadbeat velocity servo: kv = I/dt = %.6f, half the\n"
           "         explicit-Euler stability bound 2I/dt, for a wheel's\n"
           "         own I = m r^2 / 2 = %.6f kg m^2. -->\n"
           % (RUNG4_KV, RUNG4_WHEEL_INERTIA))
    for name, _sx, _sy in spec.WHEELS:
        out += ('    <velocity name="%s%s_motor" joint="%s%s_joint" '
                'kv="%.6f"/>\n' % (prefix, name, prefix, name, RUNG4_KV))
    return out


def _wall(name, face_x, size, center_z):
    """A wall whose NEAR FACE is at ``face_x``.

    Stated by its near face rather than its centre because that is the
    quantity the ground truth uses: a range reading is measured to the
    surface the ray hits, and expressing the scene in centre coordinates puts
    a half-thickness between the file and the expectation for a reader to get
    wrong.
    """
    sx, sy, sz = size
    return """
    <!-- {name}: near face at x={face}, so a ray from x_s reads {face}-x_s. -->
    <geom name="{name}" type="box" size="{hx} {hy} {hz}"
          pos="{cx} 0 {cz}" rgba="0.55 0.55 0.6 1"/>
""".format(name=name, face=face_x, hx=sx / 2.0, hy=sy / 2.0, hz=sz / 2.0,
           cx=face_x + sx / 2.0, cz=center_z)


def _floor_block(size=None):
    """The floor, verbatim from ``rungs.FLOOR_SIZE``.

    There is no override.  A scene number this arm could set for itself is a
    scene number that can disagree with the expectation the row is judged
    against, and the row is then silently meaningless rather than red.

    ``size`` exists for rung 11 alone and is NOT such a knob: the only value
    ever passed is ``rungs.rung11_floor_size(n)``, which is the contract's own
    function of the fleet size, and the fleet sizes are the contract's too.  A
    fleet 16 lanes wide does not fit on the single floor rungs 1-8 share, and
    the alternative -- one floor big enough for the largest fleet, used by all
    of them -- would change rungs 1-8's scene to serve rung 11.
    """
    fx, fy, fz = size or spec.FLOOR_SIZE
    # The slab's centre stays where the contract puts it, so its TOP stays at
    # FLOOR_TOP for every rung: an arm that moved the surface would move the
    # expectation of every check that reads a height.
    return _FLOOR.format(fx=fx, fy=fy, fz=fz, cz=spec.FLOOR_CENTER_Z,
                         top=spec.FLOOR_TOP, hx=fx / 2.0, hy=fy / 2.0,
                         hz=fz / 2.0)


# --------------------------------------------------------------------------
# rung 4's floor must be able to hold the whole run
# --------------------------------------------------------------------------
#
# MEASURED 2026-08-12, MuJoCo 3.8.1, when the contract's floor was 4 x 4 m: the
# rung-4 robot ran OFF THE EDGE.  It starts at x = 0 and is commanded at
# omega r = 0.400 m/s for 6.5 s = 2.600 m, but that floor only reached
# x = +2.000 m.  The front wheels' leading edge is at x + WHEEL_X + WHEEL_R,
# so the robot tipped over the lip at x = 1.70 m, i.e. t ~ 4.9 s -- INSIDE the
# t = 2..6 s measurement window.  Up to that instant the arm tracked the
# analytic value essentially exactly (x = 0.1831 m at t = 0.5 s to 1.7822 m at
# t = 4.5 s, i.e. 0.3998 m/s against an expected 0.4000); after it the chassis
# was stranded on the lip with its wheels spinning free -- which is precisely
# the "distance short, wheels turning, roll_ratio < 1" signature a traction
# defect produces.  That is the failure mode this bound exists to prevent: it
# does not merely lose the measurement, it MASKS whatever pushed the rover
# there behind a plausible-looking wrong answer.
#
# The requirement is a function of the contract alone,
#
#     v T + WHEEL_X + WHEEL_R = 0.400 * 6.5 + 0.2 + 0.1 = 2.900 m
#
# forward of the spawn.  ``rungs.FLOOR_SIZE`` has since been raised to 20 m
# square -- deliberately ~3x the requirement, so that a BROKEN engine's
# overrun is still measured as that overrun rather than as a fall.
#
# This is a PRECONDITION, not a warning: ``mjcf(4)`` refuses to emit a scene
# whose floor cannot hold the run.  A warning in a generated comment is read
# by nobody, and the previous version of this file emitted one into every
# rung-4 model for a whole campaign.

RUNG4_TRAVEL_M = spec.rolling_speed(spec.RUNG4_OMEGA_CMD) * spec.RUNG4_DURATION
RUNG4_REQUIRED_HALF_X = RUNG4_TRAVEL_M + spec.WHEEL_X + spec.WHEEL_R

# The wheel that leads the robot origin -- the same term in every rover rung.
ROVER_LEAD_M = spec.WHEEL_X + spec.WHEEL_R

# Rung 6 stops AT a wall, so the floor must reach past the wall rather than
# past the rover: a rover that drove off the lip short of the wall reports a
# gap it never closed, which reads as a sensor that triggered early.
RUNG6_REQUIRED_HALF_X = spec.RUNG6_WALL_FACE_X + spec.RUNG5_WALL_SIZE[0]

# Rung 7's requirement is set by the FASTEST rover, and its lateral one by the
# outermost lane plus half a robot.
RUNG7_REQUIRED_HALF_X = (spec.rolling_speed(max(spec.RUNG7_OMEGA))
                         * spec.RUNG7_DURATION + ROVER_LEAD_M)
RUNG7_REQUIRED_HALF_Y = (max(abs(y) for y in spec.RUNG7_Y)
                         + spec.RUNG7_ROBOT_WIDTH / 2.0)


class SceneTooSmall(ValueError):
    """The contract's floor cannot hold the run the contract commands."""


def _require_floor(need_half_x, who, why, need_half_y=0.0, have=None):
    """Refuse to emit a scene whose floor cannot hold the whole run.

    A rover that runs off the lip is beached with its wheels spinning, and
    that signature -- short distance, wheels turning, roll ratio below 1 --
    reads exactly like a traction defect.  It masked a real engine defect once
    already (see the rung-4 note above), so this is a PRECONDITION and not a
    warning in a generated comment.

    ``have`` names the floor being emitted, for the one rung whose floor is a
    function of the run (rung 11's fleet size).  It defaults to the shared
    ``rungs.FLOOR_SIZE``, so every other caller is unchanged.
    """
    have_x = (have or spec.FLOOR_SIZE)[0] / 2.0
    have_y = (have or spec.FLOOR_SIZE)[1] / 2.0
    if have_x < need_half_x or have_y < need_half_y:
        raise SceneTooSmall(
            "%s needs a floor reaching x >= %.3f m and y >= %.3f m (%s), but "
            "the floor gives %.3f x %.3f.  A run that leaves the floor "
            "is reported as the wrong defect; refusing to emit the scene."
            % (who, need_half_x, need_half_y, why, have_x, have_y))


WHEELS = (
    # name, x, y.  +X forward, +Y left.
    ("fl", +spec.WHEEL_X, +spec.WHEEL_Y),
    ("fr", +spec.WHEEL_X, -spec.WHEEL_Y),
    ("rl", -spec.WHEEL_X, +spec.WHEEL_Y),
    ("rr", -spec.WHEEL_X, -spec.WHEEL_Y),
)


# --------------------------------------------------------------------------
# rung 8's actuator gains -- DERIVED, like the velocity servos above
# --------------------------------------------------------------------------
#
# The contract owns the grip FORCE (``RUNG8_GRIP_N``) and delegates the
# actuator that produces it, offering ``spec.rung8_bite_m(kp, ke)`` for a
# position-controlled pad and saying that an arm whose engine has a true force
# mode should use that instead.  MuJoCo has one -- ``<motor gear="1">`` puts
# exactly ``ctrl`` newtons on the joint -- and IT DOES NOT WORK HERE.  That is
# a finding about the contact model, not about MuJoCo's actuators, and it is
# worth the paragraph because the same trap is waiting for any arm that reads
# "true force mode" and takes the short path:
#
#     MEASURED.  Commanding the contract's 3.0 N straight onto the pads drove
#     them 24 mm INTO a 60 mm part -- past its centre plane, to the joint
#     limit -- and the "grasp" then ejected the part at 1.36 m/s.  MuJoCo's
#     default contact is an ACCELERATION-referenced spring (solref tau=0.02 s,
#     reference acceleration -x/tau^2), so the penetration a given FORCE
#     produces is F tau^2 / m_eff, and m_eff is the CONSTRAINT's reduced mass
#     -- here dominated by the 0.02 kg pad, not by the 0.2 kg part.  At
#     3 N that is 66 mm of penetration before any equilibrium exists.  A
#     force command against a soft contact has no bounded steady state; a
#     POSITION command does, because the interference is what is commanded.
#
# So the pads are position-controlled and the interference comes from the
# contract's own helper.  ``ke`` is MuJoCo's contact stiffness under its
# default solref, m/tau^2, using the PART's mass -- which is the same
# derivation the contract's own RUNG8_POSE_TOL comment uses, so the two
# numbers come from one place.  ``kp`` is set by requiring the pad itself to
# deflect less than a tenth of RUNG8_POSE_TOL under the grip force.
#
# The force that ARRIVES is then bite / (1/kp + 1/ke), which is RUNG8_GRIP_N
# by construction if ke is right and less if the real constraint is softer.
# Either is fine and neither is fudged: the rung needs the grip to beat the
# Coulomb bound m g / (2 mu) = 0.33 N, and run.py REPORTS the measured contact
# normal force so the margin is a number in the result rather than a claim.

# The two prismatic stages are POSITION servos, and their gains come from two
# requirements the contract states, never from a value that happened to work:
#
#  * DROOP.  A position servo holding a static load sits at F/kp below its
#    command.  The lift axis carries wrist + pads + part, and that sag lands
#    directly in ``lift_height``, so it is held to a tenth of RUNG8_POSE_TOL.
#  * SETTLING.  Each stage must be at rest before the window that measures it
#    opens.  The tightest slack in the schedule is HOLD_WIN[0] - T_TRAV =
#    0.5 s; a third of that is the settling time required, and for a
#    critically damped second-order stage the 2% settling time is 5.8/w_n.
#
# ``run.py`` reads kp and kv back out of the COMPILED model and reports both
# explicit-Euler stability margins (kp dt^2/m < 4, kv dt/m < 2), so the claim
# that these gains are inside the bound is a measured field and not a comment.
RUNG8_DROOP_FRACTION = 0.1               # of RUNG8_POSE_TOL
RUNG8_DROOP_FRACTION_PAD = 0.1           # of RUNG8_POSE_TOL, for the pad
RUNG8_SETTLE_S = (spec.RUNG8_HOLD_WIN[0] - spec.RUNG8_T_TRAV) / 3.0   # 0.167 s
_SETTLE_WN = 5.8 / RUNG8_SETTLE_S        # rad/s, 2% settling for zeta = 1

# Masses each stage carries, from the contract's own numbers.
RUNG8_LIFT_MASS = (spec.RUNG8_WRIST_MASS + 2.0 * spec.RUNG8_PAD_MASS
                   + spec.RUNG8_PART_MASS)
RUNG8_TRAVERSE_MASS = RUNG8_LIFT_MASS + spec.RUNG8_CARRIAGE_MASS


def _kp_pos(mass, weight_n):
    """Position-servo stiffness: the larger of the droop and settling needs."""
    kp_settle = mass * _SETTLE_WN ** 2
    kp_droop = (weight_n / (RUNG8_DROOP_FRACTION * spec.RUNG8_POSE_TOL)
                if weight_n else 0.0)
    return max(kp_settle, kp_droop)


RUNG8_LIFT_KP = _kp_pos(RUNG8_LIFT_MASS, RUNG8_LIFT_MASS * spec.G)
RUNG8_LIFT_KV = 2.0 * math.sqrt(RUNG8_LIFT_KP * RUNG8_LIFT_MASS)
# The traverse is horizontal: gravity puts no steady load on it, so only the
# settling requirement applies.
RUNG8_TRAVERSE_KP = _kp_pos(RUNG8_TRAVERSE_MASS, 0.0)
RUNG8_TRAVERSE_KV = 2.0 * math.sqrt(RUNG8_TRAVERSE_KP * RUNG8_TRAVERSE_MASS)

RUNG8_CONTACT_TAU = 0.02                 # s, MuJoCo's default solref timeconst
RUNG8_KE = spec.RUNG8_PART_MASS / RUNG8_CONTACT_TAU ** 2          # 500 N/m
RUNG8_PAD_KP = spec.RUNG8_GRIP_N / (RUNG8_DROOP_FRACTION_PAD
                                    * spec.RUNG8_POSE_TOL)        # 3000 N/m
RUNG8_BITE = spec.rung8_bite_m(RUNG8_PAD_KP, RUNG8_KE)            # 7.0 mm

# The pad servo's damping lives in the JOINT, not in the actuator.  MuJoCo's
# Euler integrator treats an actuator's kv explicitly and folds JOINT damping
# implicitly into the mass matrix, and critical damping for this stage is
# kv = 2 sqrt(kp m) = 15.5, which at the pad's 0.02 kg gives kv dt/m = 3.1 --
# past the kv dt/m < 2 explicit bound.  The identical number placed in the
# joint is unconditionally stable.  Same physics, different integrator
# treatment; this is the same distinction that sets the wheel servos' kv.
RUNG8_PAD_DAMPING = 2.0 * math.sqrt(RUNG8_PAD_KP * spec.RUNG8_PAD_MASS)

# The pads must never touch EACH OTHER: a pad-pad contact silently holds the
# fingers open, and the part is then never gripped at all.  Measured while
# this arm's gripper was being built -- with the pads authored at a common
# origin they interpenetrated at q=0, the mutual contact drove them to a
# 44 mm steady-state error against their command, and the "grasp" lifted
# nothing while every commanded value looked right.  The joint's upper limit
# is one full pad width short of contact.
RUNG8_PAD_Q_MAX = spec.RUNG8_PAD_OPEN_Y - spec.RUNG8_PAD_SIZE[1]      # 48 mm
RUNG8_PAD_Q_TOUCH = spec.RUNG8_PAD_OPEN_Y - spec.RUNG8_PAD_TOUCH_Y    # 24 mm


def _pad(side, sign):
    """One gripper pad.  ``sign`` is +1 for the +Y pad, -1 for the -Y one.

    The joint axis points INWARD on both sides, so a single positive command
    closes the gripper and the two pads are symmetric by construction rather
    than by a sign the driver has to remember.  ``range`` stops them one full
    pad width short of touching each other.
    """
    sx, sy, sz = spec.RUNG8_PAD_SIZE
    return """
        <body name="pad_{side}" pos="0 {y0} 0">
          <joint name="pad_{side}_joint" type="slide" axis="0 {ax} 0"
                 range="0 {qmax:.6f}" damping="{b:.4f}"/>
          <geom name="pad_{side}_geom" type="box" size="{hx} {hy} {hz}"
                mass="{m}" rgba="0.2 0.55 0.75 1"/>
        </body>
""".format(side=side, y0=sign * spec.RUNG8_PAD_OPEN_Y, ax=-sign,
           qmax=RUNG8_PAD_Q_MAX, b=RUNG8_PAD_DAMPING, hx=sx / 2.0,
           hy=sy / 2.0, hz=sz / 2.0, m=spec.RUNG8_PAD_MASS)


def _gantry_scene():
    """Rung 8: a Cartesian gantry gripper, a table and a loose part.

    TWO PRISMATIC STAGES, NOT AN ARM.  The claim is about the GRASP, and a
    revolute chain would add inverse kinematics and its own pose error to a
    measurement that is not about either.  With slides, the gripper is
    wherever the schedule says with no solving at all, so any deviation of the
    PART is the part's.

    THE WRIST ORIGIN IS AUTHORED AT THE PART'S CENTRE (both at z =
    RUNG8_GRASP_Z), which is what makes the carry check an analytic zero
    rather than a self-reference: a grasp check that took its reference from
    the run could not tell a part gripped correctly from one gripped in the
    wrong place and then held there.

    FRICTION IS RUNG8_MU FOR THIS SCENE ONLY, per the contract -- the rung
    asks whether a grasp with a large Coulomb margin holds, not where this
    engine's friction limit is.
    """
    tx, ty, tz = spec.RUNG8_TABLE_SIZE
    cx, cy, cz = spec.RUNG8_CARRIAGE_SIZE
    wx, wy, wz = spec.RUNG8_WRIST_SIZE
    part_h = spec.RUNG8_PART_EDGE / 2.0
    out = """
  <worldbody>
    <!-- Table: top at z={ttop}.  The place target x={tx_target} is inside its
         half-extent {thx}, so a DROPPED part lands on the table rather than
         on the floor -- the airborne check has to separate "carried" from
         "resting on the nearest surface", and the nearest surface is here. -->
    <geom name="table" type="box" size="{thx} {thy} {thz}" pos="0 0 {tcz}"
          rgba="0.45 0.36 0.28 1"/>
    <geom name="floor" type="box" size="{fhx} {fhy} {fhz}" pos="0 0 {fcz}"
          rgba="0.4 0.42 0.45 1"/>

    <!-- The part: a {edge} m cube of {pmass} kg, resting on the table with
         its centre at z={pz0}. -->
    <body name="part" pos="0 0 {pz0}">
      <freejoint name="part_free"/>
      <geom name="part_geom" type="box" size="{ph} {ph} {ph}" mass="{pmass}"
            rgba="0.85 0.72 0.25 1"/>
    </body>

    <!-- Gantry.  Rail at z={basez}; the carriage traverses along X and the
         wrist lifts along Z.  At both joints' zero the wrist origin is at
         z={graspz}, which is the part's centre. -->
    <body name="carriage" pos="0 0 {basez}">
      <joint name="traverse" type="slide" axis="1 0 0"/>
      <geom name="carriage_geom" type="box" size="{chx} {chy} {chz}"
            mass="{cmass}" rgba="0.3 0.3 0.34 1"/>
      <body name="wrist" pos="0 0 {wdz}">
        <joint name="lift" type="slide" axis="0 0 1"/>
        <!-- The plate hangs {platedz} m above the wrist origin, so its
             underside clears the part's top by {clear:.3f} m. -->
        <geom name="wrist_geom" type="box" size="{whx} {why} {whz}"
              pos="0 0 {platedz}" mass="{wmass}" rgba="0.3 0.3 0.34 1"/>
{pads}      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- The two stages are POSITION servos; kp is the larger of the droop and
         settling requirements and kv is critical damping.  run.py reports
         both explicit-Euler stability margins from the compiled model. -->
    <position name="traverse_motor" joint="traverse" kp="{tkp:.4f}"
              kv="{tkv:.4f}"/>
    <position name="lift_motor" joint="lift" kp="{lkp:.4f}" kv="{lkv:.4f}"/>
    <!-- The pads are POSITION servos: ctrl is the joint's commanded closure
         in metres, and the interference that develops RUNG8_GRIP_N through
         the pad-and-contact series stiffness is spec.rung8_bite_m().  kv is 0
         because the damping is in the joint -- see the note above. -->
    <position name="pad_l_motor" joint="pad_l_joint" kp="{pkp:.4f}" kv="0"/>
    <position name="pad_r_motor" joint="pad_r_joint" kp="{pkp:.4f}" kv="0"/>
  </actuator>
""".format(
        thx=tx / 2.0, thy=ty / 2.0, thz=tz / 2.0,
        tcz=spec.RUNG8_TABLE_CENTER_Z, ttop=spec.RUNG8_TABLE_TOP,
        tx_target=spec.RUNG8_TRAVERSE_X,
        fhx=spec.FLOOR_SIZE[0] / 2.0, fhy=spec.FLOOR_SIZE[1] / 2.0,
        fhz=spec.FLOOR_SIZE[2] / 2.0, fcz=spec.FLOOR_CENTER_Z,
        edge=spec.RUNG8_PART_EDGE, pmass=spec.RUNG8_PART_MASS,
        pz0=spec.RUNG8_PART_Z0, ph=part_h,
        basez=spec.RUNG8_BASE_Z, graspz=spec.RUNG8_GRASP_Z,
        chx=cx / 2.0, chy=cy / 2.0, chz=cz / 2.0,
        cmass=spec.RUNG8_CARRIAGE_MASS,
        wdz=spec.RUNG8_GRASP_Z - spec.RUNG8_BASE_Z,
        whx=wx / 2.0, why=wy / 2.0, whz=wz / 2.0,
        wmass=spec.RUNG8_WRIST_MASS, platedz=spec.RUNG8_WRIST_PLATE_DZ,
        clear=spec.RUNG8_WRIST_PLATE_DZ - wz / 2.0 - part_h,
        pads=_pad("l", +1) + _pad("r", -1),
        tkp=RUNG8_TRAVERSE_KP, tkv=RUNG8_TRAVERSE_KV, pkp=RUNG8_PAD_KP,
        lkp=RUNG8_LIFT_KP, lkv=RUNG8_LIFT_KV)
    return out


RUNG8_NOSLIP_ITERATIONS = 5


def solver_options(rung):
    """Extra ``<option>`` attributes for one rung.  Empty except on rung 8.

    ``noslip_iterations`` IS REQUIRED FOR RUNG 8 AND IS NOT A TUNING KNOB.
    It is MuJoCo's own post-solve Gauss-Seidel pass over the FRICTION
    constraints, whose entire purpose is removing the drift a soft tangential
    constraint accumulates; it is off by default (0).  With it off, this
    scene's grasp CREEPS: measured, the part slid 44 mm down the pads during
    the 1.5 s lift and then fell out, ending back on the table -- with the
    normal force held at its commanded value the whole time.  Turning it on
    ends the run with the part 1.7 mm from where the gripper says it is.

    The count is an iteration BUDGET, not a physical constant, so it is fixed
    by CONVERGENCE rather than by a threshold.  Measured over the whole rung,
    same machine, MuJoCo 3.8.1:

        noslip   carry_rel      part z at the target     grip/pad
        0        0.440257 m     0.727793 (on the table)  0.000 N
        1        0.001686 m     0.880118                 1.296 N
        2        0.001686 m     0.877651                 1.256 N
        3        0.001686 m     0.877353                 1.254 N
        5,8,10,20,50  identical to 3 in every digit above

    Five is past the knee with margin, and the answer is insensitive to it --
    which is what makes it a budget rather than a fit.

    TWO THINGS THAT DID NOT WORK, because the obvious advice points at them.
    OmniSim's own friction-grasp guide prescribes an elliptic cone and a
    raised ``impratio``; on THIS scene, in bare MuJoCo, both made it worse.
    Measured: ``impratio=4`` left 40 mm of slip, ``impratio=10`` and
    ``impratio=50`` were worse than default, and ``cone="elliptic"`` with
    either dropped the part on the FLOOR (z = 0.530).  Stiffening the pad
    contact via a direct-stiffness ``solref`` did not fix it either.  The
    failure is friction DRIFT in the solve, and the pass built to remove
    friction drift is the thing that removes it.
    """
    rung = int(rung)
    if rung == 18:
        return _rung18_solver_options()
    if rung != 8:
        return ""
    return ' noslip_iterations="%d"' % RUNG8_NOSLIP_ITERATIONS


def rung18_declarations():
    """Rung 18's departures from MuJoCo's defaults, and where each comes from.

    CONTRACT.md 3b R4: a departure is declared in the open or it did not
    happen.  Every entry here is READ OUT OF LANE1R'S OWN ``WorldInfo`` and
    translated; none is chosen by this arm, and moving lane1r's world moves
    this scene with it.

    WHY THE WHOLE ``WorldInfo`` IS TRANSLATED AND NOT PART OF IT.  Rung 18 is
    the one scene in this file that is not the ladder's: its geometry, masses,
    inertia, friction, gravity and timestep are all read from lane1r, because
    the ground truth is a recording and lane1r owns the replay of it.  The two
    fields below sit in that same block and describe how that scene's friction
    is to be SOLVED.  Translating a world's masses and dropping its contact
    declaration is not "running the engine's defaults", it is replaying a
    DIFFERENT SCENE and reporting it as lane1r's.

    It is also the rung-8 rule (CONTRACT.md 3b R2) applied where it was
    written to apply: a pyramidal cone is a polygon INSCRIBED in the Coulomb
    cone, so asking for the ellipse asks for the model the scene already
    declares rather than for a better one, and ``impratio`` is the frictional
    constraint's impedance -- approximation accuracy, not a change of model.

    ! THE ORDER THIS DECISION WAS TAKEN IN, because CONTRACT.md 3b is explicit
    that choosing a rule after seeing the result is choosing it to fit: the
    defaults run was measured FIRST (43.023 % of cube width), the gap against
    OmniSim's published lane1r campaign on the same 50 tosses (24.845 %) was
    the reason to look, and the sweep in ``--sweep`` then showed the cone
    accounts for ALL of it and the impratio for 0.013 points.  The rule above
    is not derivable from that measurement and would have selected the same
    two fields without it; the measurement is why anyone went looking.  Both
    runs are published and the defaults datum is reproducible with
    ``python rung18.py --sweep``.
    """
    f = rung18_facts()
    out = {}
    if str(f["world_cone"]).lower() == "elliptic":
        out["cone"] = {
            "value": "elliptic", "mujoco_default": "pyramidal",
            "read_from": "lane1r WorldInfo.newtonCone",
            "why": "a pyramidal cone is a polygon inscribed in the Coulomb "
                   "cone lane1r's scene declares; the ellipse is that model, "
                   "not a better one (CONTRACT.md 3b R2)",
            "worth_pct_of_cube_width": 18.178}
    if float(f["world_impratio"]) > 0:
        out["impratio"] = {
            "value": float(f["world_impratio"]), "mujoco_default": 1.0,
            "read_from": "lane1r WorldInfo.newtonImpratio",
            "why": "frictional constraint impedance relative to the normal "
                   "one -- approximation accuracy, not a change of model",
            "worth_pct_of_cube_width": 0.013}
    return out


def _rung18_solver_options():
    d = rung18_declarations()
    out = ""
    if "cone" in d:
        out += ' cone="%s"' % d["cone"]["value"]
    if "impratio" in d:
        out += ' impratio="%g"' % d["impratio"]["value"]
    return out


# --------------------------------------------------------------------------
# rung 9 -- 25 resting cubes and a 26th dropped on the pile's OUTER corner
# --------------------------------------------------------------------------
#
# EVERY POSE IN THIS SCENE IS WRITTEN AS AN EXACT float64 LITERAL, and that is
# not tidiness.  The rung's ``seed_nudge`` fault moves the dropped cube's spawn
# x by ``RUNG9_FAULT_NUDGE`` -- a quantity far below what ``%.6f`` (the
# formatter this file uses everywhere else) can represent.  Rounded away, the
# faulted scene is BYTE-IDENTICAL to the honest one and the fault SILENTLY DOES
# NOT HAPPEN, while still reporting that it ran: a red proof that proves
# nothing.  ``repr(float(x))`` round-trips float64 exactly, so what the
# contract asked for is what the solver compiles.
#
# The magnitude is deliberately NOT restated here -- it is the contract's and
# it has already moved once during this arm's construction (1e-12 -> 1e-7).
# ``redcheck.py`` checks the property instead of the number: the faulted scene
# must differ from the honest one, it must carry the contract's exact sum, and
# ``%.6f`` of that sum must still collapse to the unfaulted value, so the
# hazard this formatter exists to avoid is re-demonstrated on every run.


def _f17(v):
    """An exact float64 literal.  See the note above -- this is load-bearing."""
    return repr(float(v))


def _rung9_cube(name, x, y, z, dynamic=True):
    """One cube of the pile, or the 26th.

    ``dynamic=False`` drops the ``freejoint`` and nothing else, which is the
    ``frozen`` fault: the body keeps its geom, so it is still a collider -- it
    simply cannot move.  A static world is PERFECTLY deterministic and the rung
    has to fail it anyway, which is the whole reason the sensitivity control
    and the analytic anchor exist.
    """
    return """
    <body name="{name}" pos="{x} {y} {z}">
{joint}      <geom name="{name}_geom" type="box" size="{h} {h} {h}" mass="{m}"
            rgba="{col}"/>
    </body>
""".format(name=name, x=_f17(x), y=_f17(y), z=_f17(z), h=_f17(spec.BOX_HALF),
           m=_f17(spec.BOX_MASS),
           joint=('      <freejoint name="%s_free"/>\n' % name
                  if dynamic else ""),
           # Keyed on the body's role alone, so the frozen scene differs from
           # the honest one in exactly ONE field -- the one the fault removes.
           col="0.9 0.35 0.15 1" if name == "drop" else "0.85 0.72 0.25 1")


def _rung9_bodies(eps=0.0, nudge=0.0, frozen=False):
    """The 5x5 pile plus the cube released over the pile's OUTER corner.

    ``RUNG9_DROP_XY``, never a corner this file computes: the contract's own
    comment records that the obvious placement -- over the CENTRE cube's corner
    -- is perfectly supported by four neighbours at a 1 mm pitch gap, lands
    flat, and amplifies nothing.  The scene follows whatever the contract says
    and this arm has no opinion about it.

    ``eps`` is the contract's sensitivity seed and ``nudge`` is its fault
    magnitude.  They are separate arguments and are never summed by a caller,
    so a run cannot be both the control and the fault by accident.
    """
    out = []
    for tag, x, y in spec.rung9_pile_xy():
        out.append(_rung9_cube(tag, x, y, spec.RUNG9_PILE_Z))
    out.append(_rung9_cube("drop", spec.RUNG9_DROP_XY + eps + nudge,
                           spec.RUNG9_DROP_XY, spec.RUNG9_SPAWN_Z,
                           dynamic=not frozen))
    return "".join(out)


# --------------------------------------------------------------------------
# rung 11 -- the rung-4 rover at N = 1, 4, 8, 16
# --------------------------------------------------------------------------
#
# NO CONSTRAINT BUDGET IS DECLARED HERE, and that is a measurement rather than
# an omission.  ``rungs.RUNG11_NJMAX`` exists because the OmniSim/Newton path
# pins 256 constraint rows by default and a 16-rover fleet needs 512.  MuJoCo's
# own default is ``njmax = -1`` -- read back from the compiled model and
# reported in ``scene_facts()`` -- which means the constraint arrays are sized
# from the arena at solve time and there is no cap to overflow.  Declaring the
# contract's 4096 here would therefore RESTRICT this arm relative to its own
# default while looking like a safety measure, so it is not declared and the
# readback is published instead.

RUNG11_REQUIRED_HALF_X = (spec.rolling_speed(max(spec.RUNG11_OMEGA))
                          * spec.RUNG11_DURATION + ROVER_LEAD_M)


def _rung11_required_half_y(n):
    """Across track: the outermost lane plus half a robot."""
    return (max(abs(spec.rung11_y(i, n)) for i in range(n))
            + spec.RUNG7_ROBOT_WIDTH / 2.0)


def _rung11_scene(n, fault=None):
    """``n`` copies of the rung-4 rover in the contract's own lanes.

    THE SAME ``_rover`` EMITTER as rungs 4, 6 and 7.  A rung that re-declared
    the chassis would let rung 4 and rung 11 drift apart while both still
    looked comparable, which is exactly the failure this file's one-emitter
    rule exists to prevent.

    ``lane_offset`` is applied HERE, as a SPAWN offset, and never as a per-step
    write: CONTRACT.md section 6 records that writing the victim's y every step
    cost the body its state and reddened both must-green companions, so the
    fault destroyed the thing it was meant to isolate.
    """
    floor = spec.rung11_floor_size(n)
    _require_floor(RUNG11_REQUIRED_HALF_X, "rung 11 (N = %d)" % n,
                   "the fastest rover is commanded %.1f rad/s for %.1f s and "
                   "the outermost lane is at y = %+.2f m"
                   % (max(spec.RUNG11_OMEGA), spec.RUNG11_DURATION,
                      spec.rung11_y(n - 1, n)),
                   need_half_y=_rung11_required_half_y(n), have=floor)
    out = _floor_block(size=floor)
    for i in range(n):
        y = spec.rung11_y(i, n)
        broken = (fault == "lane_offset" and i == spec.RUNG11_FAULT_ROBOT
                  and n == spec.RUNG11_FAULT_N)
        if broken:
            y += spec.RUNG11_FAULT_OFFSET_M
        out += ("\n    <!-- r%02d: lane y = %+.3f m, commanded %.1f rad/s%s -->"
                % (i, y, spec.rung11_omega(i),
                   "  [lane_offset: SPAWNED %+.2f m out of lane]"
                   % spec.RUNG11_FAULT_OFFSET_M if broken else ""))
        out += _rover(prefix="r%02d_" % i, y0=y)
    out += "  </worldbody>\n"
    out += "\n  <actuator>\n"
    for i in range(n):
        out += _rover_actuators(prefix="r%02d_" % i)
    out += "  </actuator>\n"
    return out


# --------------------------------------------------------------------------
# rung 18 -- lane 1R's cube-toss scene, as MJCF
# --------------------------------------------------------------------------
#
# NOT RE-AUTHORED.  Every number in this scene is READ, at emit time, out of
# ``tests/benchmarks/omnibench/lane1r`` -- the cube's edge, mass, inertia and
# friction from its ``dataset`` module (which the contract reaches through
# ``rungs.rung18_dataset()``), and the table's geometry, the gravity and the
# timestep out of its own ``worlds/cube_toss.wbt``.  The two are CROSS-CHECKED
# against each other and a disagreement raises rather than picking a winner:
# lane1r is right by contract, so a lane1r that disagrees with itself is a fact
# somebody has to see, not one this arm may resolve.
#
# THE INERTIA IS ISOTROPIC AND EXPLICIT (``<inertial diaginertia>``), never
# derived from geometry and density.  lane1r's own README settles why: a
# uniform 0.10 m cube of this mass gives exactly the 6.167e-4 the shipped MJCF
# carries, the real cube is hollow, and the URDF's 8.1e-4 is canonical.  An
# MJCF that let the compiler derive the tensor would silently replay a solid
# cube against a hollow cube's recording.

_LANE1R_DIR = os.path.join(
    os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir,
                                 os.pardir)),
    *spec.RUNG18_LANE1R)
_RUNG18_FACTS = None


def _strip_wbt_comments(text):
    """Drop ``#`` comments from a ``.wbt``.

    NOT optional, and it is the trap this parser hit first.  lane1r's world
    header warns, in prose, that "a world with basicTimeStep 0.6757 produces
    step(0)" -- so a regex for ``basicTimeStep`` over the raw file matches the
    WARNING before the field, and this arm compiled rung 18 at a 0.68 ms
    timestep read out of an English sentence.  It looked entirely plausible.
    """
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def _wbt_number(text, key, who):
    import re
    m = re.search(r"\b%s\s+(-?[0-9][0-9eE.+-]*)" % key, text)
    if not m:
        raise ValueError("lane1r's world declares no %r, so rung 18 cannot "
                         "read %s from it; refusing to substitute a constant "
                         "of this arm's own" % (key, who))
    return float(m.group(1))


def _wbt_box(text, who):
    import re
    m = re.search(r"Box\s*{\s*size\s+([0-9eE.+-]+)\s+([0-9eE.+-]+)\s+"
                  r"([0-9eE.+-]+)\s*}", text)
    if not m:
        raise ValueError("no Box size found for %s in lane1r's world" % who)
    return tuple(float(v) for v in m.groups())


def rung18_facts():
    """Every rung-18 scene number, read from lane1r and cross-checked.

    Cached, because ``mjcf(18)`` is called per cell and this touches the disk.
    """
    global _RUNG18_FACTS
    if _RUNG18_FACTS is not None:
        return _RUNG18_FACTS
    import re
    D = spec.rung18_dataset()
    path = os.path.join(_LANE1R_DIR, "worlds", "cube_toss.wbt")
    with open(path, "r", encoding="utf-8") as fh:
        text = _strip_wbt_comments(fh.read())
    if "WorldInfo" not in text:
        raise ValueError("lane1r's world has no WorldInfo block")
    world_info = text.split("WorldInfo", 1)[1].split("}", 1)[0]
    if "DEF TABLE Solid" not in text or "DEF CUBE Solid" not in text:
        raise ValueError("lane1r's world no longer declares DEF TABLE / DEF "
                         "CUBE; rung 18's scene is a translation of that file "
                         "and cannot be emitted from a file it does not match")
    head, cube_text = text.split("DEF CUBE Solid", 1)
    table_text = head.split("DEF TABLE Solid", 1)[1]
    m = re.search(r"translation\s+([0-9eE.+-]+)\s+([0-9eE.+-]+)\s+"
                  r"([0-9eE.+-]+)", table_text)
    if not m:
        raise ValueError("lane1r's table has no translation")
    table_pos = tuple(float(v) for v in m.groups())
    table_size = _wbt_box(table_text, "the table")
    cube_size = _wbt_box(cube_text, "the cube")
    facts = {
        "lane1r_world": path,
        "lane1r_dataset": D.__file__,
        "table_size_m": table_size,
        "table_center_z_m": table_pos[2],
        "table_top_m": table_pos[2] + table_size[2] / 2.0,
        "cube_edge_m": D.CUBE_EDGE_M,
        "cube_mass_kg": D.CUBE_MASS_KG,
        "cube_inertia_kg_m2": D.CUBE_INERTIA,
        "mu": D.CUBE_MU_STATIC,
        "gravity_m_s2": D.GRAVITY,
        "timestep_s": _wbt_number(world_info, "basicTimeStep",
                                  "its timestep") / 1000.0,
        "world_cube_edge_m": cube_size[0],
        "world_mu": _wbt_number(world_info, "newtonGroundMu", "its friction"),
        "world_gravity_m_s2": _wbt_number(world_info, "gravity",
                                          "its gravity"),
        # The two fields that say how lane1r asks for its friction to be
        # SOLVED.  Translated into MJCF below; see ``solver_options``.
        "world_cone": (re.search(r'newtonCone\s+"([^"]*)"', world_info).group(1)
                       if re.search(r'newtonCone\s+"', world_info) else ""),
        "world_impratio": (_wbt_number(world_info, "newtonImpratio",
                                       "its impratio")
                           if "newtonImpratio" in world_info else 0.0),
        "world_cube_mass_kg": _wbt_number(cube_text, "mass", "the cube's mass"),
        "sample_hz": D.SAMPLE_HZ,
    }
    m = re.search(r"inertiaMatrix\s*\[\s*([0-9eE.+-]+)", cube_text)
    facts["world_cube_inertia_kg_m2"] = float(m.group(1)) if m else None
    # lane1r is right by contract, so it must agree with ITSELF.  Anything
    # else is a drift somebody has to see rather than one this arm may pick a
    # side in.
    for a, b, what in (("cube_edge_m", "world_cube_edge_m", "the cube's edge"),
                       ("cube_mass_kg", "world_cube_mass_kg", "the cube's mass"),
                       ("cube_inertia_kg_m2", "world_cube_inertia_kg_m2",
                        "the cube's inertia"),
                       ("mu", "world_mu", "the friction"),
                       ("gravity_m_s2", "world_gravity_m_s2", "gravity")):
        x, y = facts[a], facts[b]
        if y is None or abs(x - y) > 1e-12 * max(1.0, abs(x)):
            raise ValueError(
                "lane1r disagrees with itself about %s: dataset.py says %r and "
                "worlds/cube_toss.wbt says %r.  Rung 18's ground truth is "
                "lane1r's and this arm may not choose between two versions of "
                "it." % (what, x, y))
    # A parsed number that landed in prose rather than in a field would still
    # look like a number; these are the two properties lane1r's own world
    # header says its timestep must have.
    dt = facts["timestep_s"]
    if not (0.0 < dt <= 0.01):
        raise ValueError("read a timestep of %r s out of lane1r's world, which "
                         "is not a plausible basicTimeStep -- the parse is "
                         "wrong, not the world" % dt)
    _RUNG18_FACTS = facts
    return facts


def _rung18_scene(fault=None):
    """lane 1R's table and cube, in MJCF.

    The cube is spawned at the table's centre with an arbitrary clearance: its
    real pose and BOTH its velocities are written into ``qpos``/``qvel`` per
    toss from the recording, so nothing about the authored pose reaches the
    measurement.  What the scene has to carry is the geometry, the mass, the
    inertia, the friction and the timestep.
    """
    f = rung18_facts()
    tx, ty, tz = f["table_size_m"]
    half = f["cube_edge_m"] / 2.0
    table = "" if fault == "table_hologram" else """
    <!-- Table: {tx} x {ty} x {tz} m, centre z={cz}, so its top is at z={top}
         -- lane1r lifts it off z=0 so the OmniSim/Newton implicit ground plane
         cannot double up with it, and the score undoes the lift. -->
    <geom name="table" type="box" size="{hx} {hy} {hz}" pos="0 0 {cz}"
          rgba="0.62 0.48 0.32 1"/>
""".format(tx=tx, ty=ty, tz=tz, cz=f["table_center_z_m"], top=f["table_top_m"],
           hx=tx / 2.0, hy=ty / 2.0, hz=tz / 2.0)
    return """
  <worldbody>{table}
    <!-- The cube: {edge} m edge, {mass} kg, and an ISOTROPIC {I} kg m^2 stated
         explicitly.  A compiler-derived tensor would be the 6.167e-4 of a
         UNIFORM cube; this one is hollow and lane1r's URDF value is canonical. -->
    <body name="cube" pos="0 0 {z0}">
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{I} {I} {I}"/>
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="box" size="{h} {h} {h}"
            rgba="0.6 0.0 0.0 1"/>
    </body>
  </worldbody>
""".format(table=table, edge=f["cube_edge_m"], mass=f["cube_mass_kg"],
           I=f["cube_inertia_kg_m2"], h=half,
           z0=f["table_top_m"] + half)


def _title(rung):
    """The contract's title for a rung, or a neutral one while it lands."""
    return spec.RUNG_TITLE.get(int(rung), "rung %d" % int(rung))


def scene_dt(rung):
    """Basic timestep for one rung, seconds.

    ``rungs.DT`` everywhere except rung 18, whose scene, ground truth and
    sampling grid all belong to lane 1R -- so its timestep is READ OUT OF
    LANE1R'S OWN WORLD FILE rather than declared here or borrowed from the
    ladder.  Comparing this arm against the OmniSim arm on that rung requires
    both to step the recording at the same rate, and lane1r owns that rate.
    """
    return rung18_facts()["timestep_s"] if int(rung) == 18 else spec.DT


def scene_mu(rung):
    """Coulomb friction for one rung -- ``MU`` everywhere except 8 and 18.

    The contract raises friction for rung 8 alone, so that the pinch has a
    large Coulomb margin and the rung measures "does a grasp hold" rather than
    "where is this engine's friction limit".  Rung 18's is neither the
    contract's nor this arm's: it is the acrylic-on-wood value lane1r measured
    for the cube it recorded, and it is read from there.
    """
    rung = int(rung)
    if rung == 8:
        return spec.RUNG8_MU
    if rung == 18:
        return rung18_facts()["mu"]
    return spec.MU


def mjcf(rung, run=None, fault=None):
    """The MJCF text for one rung, entirely from the shared contract.

    ``run`` is one entry of :func:`run_specs` -- the parameters of ONE RUN of a
    multi-run cell (CONTRACT.md amendment A): rung 9's perturbation, rung 11's
    fleet size.  Its keys and values are the contract's, never this arm's, and
    a caller that made one up would produce a row that is not comparable with
    the other arms' and nothing in the table would say so.

    ``fault`` is applied here only for faults that are STRUCTURAL -- a body
    that loses its joint, a robot spawned out of lane, a table that is not
    there.  Faults of the ACTUATION live in ``drivers.py`` and faults this
    arm expresses as a text edit live in ``run.py``'s ``_xml_for``.
    """
    rung = int(rung)
    run = run or {}
    out = _HEADER.format(rung=rung, title=_title(rung), dt=scene_dt(rung),
                         g=spec.G, mu=scene_mu(rung),
                         opt=solver_options(rung))

    if rung == 0:
        # Rung 0 is the null scene: no bodies at all.  It asserts that the
        # runtime compiles a model, advances the clock by steps*dt and exits
        # clean -- the floor under every other rung.
        out += "\n  <worldbody/>\n"

    elif rung == 1:
        out += _floor_block()
        out += _box_body("box", spec.RUNG1_SPAWN_Z)
        out += "  </worldbody>\n"

    elif rung == 2:
        out += _floor_block()
        out += _box_body("box", spec.RUNG2_SPAWN_Z)
        out += "  </worldbody>\n"

    elif rung == 3:
        # No floor: the joint must be genuinely unloaded.  The hinge axis is
        # VERTICAL, so gravity -- which still acts, and is declared -- exerts
        # no torque about it: the link's weight acts at (L/2, 0, 0) from the
        # hinge, and (L/2,0,0) x (0,0,-mg) points along Y, with zero Z
        # component.  The joint therefore has no load to hold and an ideal
        # velocity servo has exactly zero steady-state error.
        out += """
  <worldbody>
    <body name="link" pos="0 0 {z}">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="link_geom" type="cylinder" fromto="0 0 0  {L} 0 0"
            size="{r}" mass="{m}" rgba="0.85 0.72 0.25 1"/>
    </body>
  </worldbody>

  <actuator>
    <!-- Deadbeat velocity servo: kv = I/dt = {kv:.4f}, half the explicit-
         Euler stability bound 2I/dt, for a link hinged about one end with
         I = m(L^2/3 + r^2/4) = {I:.6f} kg m^2. -->
    <velocity name="hinge_motor" joint="hinge" kv="{kv:.6f}"/>
  </actuator>
""".format(z=spec.RUNG3_HINGE_Z, L=spec.RUNG3_LINK_LEN,
           r=spec.RUNG3_LINK_RADIUS, m=spec.RUNG3_LINK_MASS,
           kv=RUNG3_KV, I=RUNG3_LINK_INERTIA)

    elif rung == 4:
        _require_floor(RUNG4_REQUIRED_HALF_X, "rung 4",
                       "the rover is commanded %.3f m forward and its front "
                       "wheels lead the origin by %.3f m, so a shorter floor "
                       "drops it off the lip at t ~ %.2f s -- inside the "
                       "%.1f-%.1f s measurement window, and it is reported as "
                       "short distance on turning wheels, which reads exactly "
                       "like a traction defect"
                       % (RUNG4_TRAVEL_M, ROVER_LEAD_M,
                          (spec.FLOOR_SIZE[0] / 2.0 - ROVER_LEAD_M)
                          / spec.rolling_speed(spec.RUNG4_OMEGA_CMD),
                          spec.RUNG4_WIN[0], spec.RUNG4_WIN[1]))
        out += _floor_block()
        out += _rover()
        out += "  </worldbody>\n"
        out += "\n  <actuator>\n" + _rover_actuators() + "  </actuator>\n"

    elif rung == 5:
        # A distance sensor on a KINEMATIC carrier facing a wall.
        #
        # The carrier is a MOCAP body: no joints, no mass, no dynamics, and a
        # pose the driver writes directly each step.  That is exactly what the
        # contract asks for -- "a body with no physics whose pose the driver
        # WRITES on a known schedule" -- so the sensor's position at every
        # instant is a scene fact, and a disagreement between the reading and
        # the geometry cannot be blamed on the carrier not getting there.
        #
        # ``cutoff`` DECLARES THE SPAN AND DOES NOT LIMIT IT.  MuJoCo clamps
        # the reported value to +/-cutoff; it does NOT report "nothing there"
        # beyond it (measured: a wall 2.7 m away under cutoff=1.0 reads
        # 1.000000, which is indistinguishable from a wall at 1 m).  The wall
        # here is never further than RUNG5_STANDOFF, so the clamp
        # never binds and the field is a declaration of the modelled span
        # rather than a filter.  See rangefinder_conventions.py.
        out += _floor_block()
        out += _wall("wall", spec.RUNG5_WALL_FACE_X, spec.RUNG5_WALL_SIZE,
                     spec.RUNG5_WALL_CENTER_Z)
        out += """
    <!-- Carrier: {edge} m cube, mocap (kinematic).  The rangefinder site sits
         {dx} m ahead of the carrier ORIGIN, i.e. {proud} m proud of its front
         face, and points along +X via zaxis. -->
    <body name="carrier" mocap="true" pos="{x0} 0 {z}">
      <geom name="carrier_geom" type="box" size="{h} {h} {h}"
            rgba="0.85 0.72 0.25 1"/>
      <site name="range_site" pos="{dx} 0 0" zaxis="1 0 0" size="0.005"
            rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>

  <sensor>
    <!-- Reads the distance from the SITE ORIGIN along the site's +Z to the
         first surface, in metres.  -1 means the ray hit nothing. -->
    <rangefinder name="range" site="range_site" cutoff="{cut}"/>
  </sensor>
""".format(edge=spec.RUNG5_CARRIER_EDGE, h=spec.RUNG5_CARRIER_EDGE / 2.0,
           x0=spec.RUNG5_X0, z=spec.RUNG5_SENSOR_Z, dx=spec.RUNG5_SENSOR_DX,
           proud=spec.RUNG5_SENSOR_DX - spec.RUNG5_CARRIER_EDGE / 2.0,
           cut=spec.RUNG5_MAX_RANGE)

    elif rung == 6:
        # The rung-4 rover, emitted by the SAME helper, plus a forward sensor
        # and rung 5's wall.  The sensor sits at the axle height so its ray is
        # horizontal and its range to the wall is a plane geometry problem.
        _require_floor(RUNG6_REQUIRED_HALF_X, "rung 6",
                       "the rover approaches a wall whose near face is at "
                       "x = %.1f m and must stop short of it"
                       % spec.RUNG6_WALL_FACE_X)
        out += _floor_block()
        out += _wall("wall", spec.RUNG6_WALL_FACE_X, spec.RUNG5_WALL_SIZE,
                     spec.RUNG5_WALL_CENTER_Z)
        out += _rover(sensor=(spec.RUNG6_SENSOR_DX,
                              spec.RUNG6_SENSOR_Z - spec.ROBOT_Z))
        out += "  </worldbody>\n"
        out += """
  <sensor>
    <rangefinder name="range" site="range_site" cutoff="{cut}"/>
  </sensor>
""".format(cut=spec.RUNG5_MAX_RANGE)
        out += "\n  <actuator>\n" + _rover_actuators() + "  </actuator>\n"

    elif rung == 7:
        # Five rung-4 rovers in parallel lanes.  Same emitter, five prefixes:
        # a rung that re-declared the chassis would let rung 4 and rung 7
        # drift apart while both still looked comparable.
        _require_floor(RUNG7_REQUIRED_HALF_X, "rung 7",
                       "the fastest rover is commanded %.1f rad/s for %.1f s "
                       "and the outermost lane is at y = %+.1f m"
                       % (max(spec.RUNG7_OMEGA), spec.RUNG7_DURATION,
                          max(spec.RUNG7_Y)),
                       need_half_y=RUNG7_REQUIRED_HALF_Y)
        out += _floor_block()
        for tag, y in zip(spec.RUNG7_TAGS, spec.RUNG7_Y):
            out += ("\n    <!-- %s: lane y = %+.2f m, commanded %.1f rad/s -->"
                    % (tag, y, spec.RUNG7_OMEGA[spec.RUNG7_TAGS.index(tag)]))
            out += _rover(prefix=tag + "_", y0=y)
        out += "  </worldbody>\n"
        out += "\n  <actuator>\n"
        for tag in spec.RUNG7_TAGS:
            out += _rover_actuators(prefix=tag + "_")
        out += "  </actuator>\n"

    elif rung == 8:
        out += _gantry_scene()

    elif rung == 9:
        out += _floor_block()
        out += _rung9_bodies(eps=float(run.get("eps_m") or 0.0),
                             nudge=float(run.get("nudge_m") or 0.0),
                             frozen=(fault == "frozen"))
        out += "  </worldbody>\n"

    elif rung == 11:
        out += _rung11_scene(int(run.get("n") or spec.RUNG11_N[0]), fault=fault)

    elif rung == 18:
        out += _rung18_scene(fault=fault)

    else:
        raise ValueError("no such rung: %r" % (rung,))

    return out + "\n</mujoco>\n"


# --------------------------------------------------------------------------
# the RUNS of a cell -- CONTRACT.md amendment A
# --------------------------------------------------------------------------

def run_specs(rung, fault=None):
    """The runs of one cell, in the contract's own order.

    A single-run rung returns one unparameterised entry so callers need no
    special case.  EVERY TAG AND EVERY PARAMETER HERE IS READ FROM ``rungs.py``:
    an arm that chose its own perturbation, or its own fleet sizes, would
    produce a row that is not comparable and no assertion downstream could see
    it.
    """
    rung = int(rung)
    if rung == 9:
        out = []
        for tag, eps in spec.RUNG9_RUNS:
            entry = {"tag": tag, "eps_m": eps, "nudge_m": 0.0}
            if fault == "seed_nudge" and tag == "b":
                entry["nudge_m"] = spec.RUNG9_FAULT_NUDGE
            if fault == "short_b" and tag == "b":
                entry["short"] = spec.RUNG9_FAULT_SHORT
            out.append(entry)
        return out
    if rung == 11:
        return [{"tag": "n%d" % n, "n": n} for n in spec.RUNG11_N]
    if rung == 18:
        # One scene serves every toss: the initial condition is written into
        # qpos/qvel per toss, exactly as lane1r writes it into the node.
        return [{"tag": "toss"}]
    return [{"tag": None}]


def _model_tag(rung, run):
    """The file suffix for one run's committed model, or ``None``.

    Rung 9's replicas ``a`` and ``b`` are not two copies of one scene -- they
    are ONE SCENE RUN TWICE, so they share one file and a byte difference
    between them, even in a comment, would be a difference the rung exists to
    rule out.  ``c`` carries the sensitivity seed and is a different scene.
    """
    rung = int(rung)
    if rung == 9:
        return "c" if run.get("eps_m") else None
    if rung == 11:
        return None if run.get("n") == spec.RUNG11_N[0] else run.get("tag")
    return None


def model_path(rung, tag=None):
    return os.path.join(MODELS, "rung%d%s.xml"
                        % (int(rung), "_%s" % tag if tag else ""))


def write_all():
    """Regenerate every committed model file.  Returns the paths written.

    ``ALL_RUNGS``, so the on-demand rung 18 gets a committed scene too; and one
    file per RUN of a multi-run cell, minus the duplicates ``_model_tag``
    folds away.  Rung 18 is emitted best-effort: its scene is read out of
    another lane's tree, and a checkout without that lane must still be able to
    regenerate the nine scenes it does have.
    """
    if not os.path.isdir(MODELS):
        os.makedirs(MODELS)
    written, seen = [], set()
    for rung in spec.ALL_RUNGS:
        for run in run_specs(rung):
            p = model_path(rung, _model_tag(rung, run))
            if p in seen:
                continue
            seen.add(p)
            try:
                text = mjcf(rung, run=run)
            except Exception as exc:            # noqa: BLE001
                if rung == 18:
                    print("  rung 18 scene NOT regenerated: %r" % (exc,))
                    continue
                raise
            with open(p, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(text)
            written.append(p)
    return written


# --------------------------------------------------------------------------
# scene facts worth reporting alongside the measurements
# --------------------------------------------------------------------------

def scene_facts():
    """Numbers a reader needs to interpret the MuJoCo column."""
    return {
        "dt_s": spec.DT,
        "rung8_mu": spec.RUNG8_MU,
        "rung8_noslip_iterations": RUNG8_NOSLIP_ITERATIONS,
        "rung8_contact_ke_n_per_m": RUNG8_KE,
        "rung8_pad_kp_n_per_m": RUNG8_PAD_KP,
        "rung8_pad_damping_ns_per_m": RUNG8_PAD_DAMPING,
        "rung8_bite_m": RUNG8_BITE,
        "rung8_pad_q_touch_m": RUNG8_PAD_Q_TOUCH,
        "rung8_pad_q_max_m": RUNG8_PAD_Q_MAX,
        "rung8_grip_cmd_n": spec.RUNG8_GRIP_N,
        "rung8_grip_coulomb_bound_n": spec.rung8_grip_force_bound(),
        "rung8_lift_kp": RUNG8_LIFT_KP, "rung8_lift_kv": RUNG8_LIFT_KV,
        "rung8_traverse_kp": RUNG8_TRAVERSE_KP,
        "rung8_traverse_kv": RUNG8_TRAVERSE_KV,
        "rung6_required_floor_half_x_m": RUNG6_REQUIRED_HALF_X,
        "rung7_required_floor_half_x_m": RUNG7_REQUIRED_HALF_X,
        "rung7_required_floor_half_y_m": RUNG7_REQUIRED_HALF_Y,
        "gravity_m_s2": spec.G,
        "friction_mu": spec.MU,
        "floor_top_m": spec.FLOOR_TOP,
        "rung3_link_inertia_kg_m2": RUNG3_LINK_INERTIA,
        "rung3_kv": RUNG3_KV,
        "rung4_wheel_inertia_kg_m2": RUNG4_WHEEL_INERTIA,
        "rung4_kv": RUNG4_KV,
        "kv_stability_fraction": KV_STABILITY_FRACTION,
        "rung4_ground_clearance_m": (spec.ROBOT_Z - spec.CHASSIS_SIZE[2] / 2.0
                                     - spec.FLOOR_TOP),
        "rung2_impact_speed_m_s": math.sqrt(2.0 * spec.G * spec.RUNG2_DROP_M),
        "rung4_travel_m": RUNG4_TRAVEL_M,
        "rung4_contract_floor_half_x_m": spec.FLOOR_SIZE[0] / 2.0,
        "rung4_required_floor_half_x_m": RUNG4_REQUIRED_HALF_X,
        "rung4_floor_headroom_x_m": (spec.FLOOR_SIZE[0] / 2.0
                                     - RUNG4_REQUIRED_HALF_X),
        # Rung 9.  The seed and the nudge are reported as the LITERALS the
        # emitter writes, so "the perturbation survived the formatter" is a
        # field in the result rather than a claim in a comment.
        "rung9_bodies": spec.RUNG9_N_BODIES,
        "rung9_drop_xy_m": spec.RUNG9_DROP_XY,
        "rung9_sensitivity_seed_literal": _f17(spec.RUNG9_EPS),
        "rung9_fault_nudge_literal": _f17(spec.RUNG9_FAULT_NUDGE),
        "rung9_sample_every": spec.RUNG9_SAMPLE_EVERY,
        # Rung 11.  ``njmax``/``nconmax`` are read back from the COMPILED model
        # by run.py; -1 is MuJoCo's default and means "sized from the arena",
        # i.e. no cap for the contract's budget to protect against.
        "rung11_fleet_sizes": list(spec.RUNG11_N),
        "rung11_required_floor_half_x_m": RUNG11_REQUIRED_HALF_X,
        "rung11_floor_half_x_m": [spec.rung11_floor_size(n)[0] / 2.0
                                  for n in spec.RUNG11_N],
        "rung11_floor_half_y_m": [spec.rung11_floor_size(n)[1] / 2.0
                                  for n in spec.RUNG11_N],
        "rung11_constraint_budget_declared": False,
        "rung11_sample_every": spec.RUNG11_SAMPLE_EVERY,
    }


if __name__ == "__main__":
    for p in write_all():
        print(p)
