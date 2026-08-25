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
"""OMNIARM6 + Robotiq 2F-140 pick & place -- held by CONTACT FRICTION, nothing else.

Sibling of omniarm6_real_pick_place (the 2F-85 version). Same honesty rules, same
contact census, different gripper -- and the differences are not cosmetic, so
read THIS docstring rather than assuming the 2F-85's numbers carry over.

WHY THIS CONTROLLER EXISTS. Every other arm demo in this tree grips through
ArmBridge.act_grasp(), which by default welds the nearest DEF GRASP_* node to
the tool and teleports it to the TCP every tick. That hold is unfalsifiable: it
would work with the fingers wide open, for ever. This controller never calls
act_grasp, never writes the block's pose, and never creates a weld. If the
contact physics stops holding, the block falls.

THE GRIPPER. omniarm6_2f140_grip.urdf: every <visual> is the customer's CAD
(3d_models/2F-140_Assy_Open_20191022.STEP via scripts/dev/step_to_urdf.py) and
the collision is TEN axis-aligned boxes -- a gripper-body box, and per jaw two
pad halves, a fingertip bracket and the four-bar arm. The pad's inner face is at
exactly |q| by construction, so every number below is a real distance on the
grasp axis and not a joint reading that needs decoding.

THE CONTROL LAW. Size the jaws to the part before the arm moves, reach, drop,
close once to 10 mm of interference, verify by contact, lift, carry, place --
SIX commanded moves, one per phase, down from twelve. The engine builds
these joints as a PD servo with kp = effort*10 = 1250 N/m (OmBasicJoint.cpp:127),
so 10 mm of interference is ~12.5 N per pad; at mu 6 that is ~75 N of friction
per pad against a 1.47 N block. The 2F-140's datasheet grip force is 10-125 N,
so this sits at a tenth of what the real gripper can do.

===============================================================================
THE MOTION IS NOW SIX MOVES, ONE PER PHASE (2026-08-11). AND ONE IS NOT SAFE.
===============================================================================
User-reported, watching it run: make the calibration step go away and give me
one continuous fluid motion. Twelve commanded moves -> SIX: reach, descend,
lift, carry, lower, retreat, with nothing left that stops and restarts at the
part. Two changes did it, and a third was attempted and REVERTED BY MEASUREMENT.

1. THE CALIBRATE STAGE IS DELETED, because it re-measured a CONSTANT. It flew
   to a height 160 mm above the block and ran a convergence loop there -- up to
   four commanded moves -- to discover the FK-tool-point-to-pad-plane offset.
   Every revision of this file had already recorded that the offset is constant
   ("-1.03, +5.70, +8.10 mm, reproducible run to run"). It is now baked as
   CALIB_BAKED and RE-VERIFIED ON EVERY RUN FOR FREE: once the arm is at the
   reach pose, the offset is just tcp() - grip_point(), both derived from the
   arm's own measured joints, so checking it costs no motion at all. Measured
   drift on the shipped run: 0.698 / 0.007 / 0.029 mm.
   ⚠ Measure (FK tool point - pad plane), NOT (commanded goal - pad plane).
   The first version of the check used the commanded goal and reported 12 mm of
   drift on a constant that had not moved: with passes=1 the reach stops ~12 mm
   short of its goal and that tracking residual landed in the "constant". The
   old loop was immune only because it iterated until the two agreed -- which
   is exactly the expense being removed.

2. THE CORRECTION PASSES ARE OFF (passes=1 on reach / lift / carry / retreat).
   goto()'s extra passes are separate 0.5 s ramps -- a visible twitch at the end
   of each move -- and they correct nothing that the descent's and the place
   descent's own in-flight servos do not.

3. ⚠⚠ ONE MOVE FROM SPAWN STRAIGHT TO THE GRASP POSE WAS TRIED, AND IT CLIPPED
   THE PART. The argument was that the 160 mm waypoint is only staging, since
   descend_once corrects in flight wherever it starts. Both halves were wrong
   and both showed up as numbers on the first run:
     * the in-flight corrector integrates pad-plane error, and error is only
       meaningful once the ramp has stopped. Armed at 70% of a full transit it
       integrated the transit: correction -312.63 / -4.30 / -237.17 mm, pad
       plane 69 mm out. Re-arming at 92% cut the windup and still missed by
       77.8 mm, because by then --
     * sweeping the open jaws diagonally at the part instead of dropping onto
       it from above put a pad THROUGH the block: it moved 55.6 / 73.9 / -16.5
       mm before the gripper closed, 4 robot contacts were live at the close,
       and the run carried nothing (max relative rotation 179.7 deg,
       placed=False).
   The waypoint is the only thing preventing that -- there is no perception in
   this loop and no reactive stop -- so it stays, and `reach_nudge` is now a
   standing assertion in the verdict: the block must not move during the
   approach. "The approach does not touch the part" is a CLAIM this demo makes
   now, not a property of its staging.

===============================================================================
TWO USER-REPORTED REALISM DEFECTS, BOTH MEASURED AND BOTH FIXED (2026-08-11).
===============================================================================
"When it goes down, it moves a little bit around until it picks the cube", and
"while the gripper is rotating to the place position, the cube is not rotating
the same way -- it kind of has its own mind". Neither was visible in the
verdict this demo was shipping: the first was 7 commanded moves that nothing
counted, and the second was a ROTATION, while `drift` measures POSITION ONLY.

1. THE DESCENT IS NOW ONE SEGMENT (7 -> 1), and the run PRINTS the count.
   The old descent reached the grasp pose with a 3-pass closed loop on the FK
   tool point plus a 4-move re-aim on the measured pad mid-plane -- every one a
   separate little correction AT the block. Three error terms were being
   chased, and only the first is a constant:
     FK tool-point offset  CONSTANT (-1.03, +5.70, +8.10 mm), reproducible run
                           to run -> calibrated ONCE at the approach height,
                           160 mm up, where a correction is invisible.
     gravity sag           POSE-DEPENDENT: 0.46 mm at the approach pose and
                           12.32 mm at the grasp pose, entirely in joints 2 and
                           3 (-11.65 / -16.72 mrad, joints 1/4/5/6 exact). It
                           is STATIC -- identical at +0.5 s and +4.0 s of
                           settling -- and it is the engine's designed servo
                           behaviour, not a bug (OmBasicJoint.cpp:675 sets
                           targetKe = effortLimit*10, whose own comment says
                           that holds an arm shoulder "to <0.02 rad").
     DH model error        POSE-DEPENDENT, ~3.5 mm over the 160 mm descent; the
                           IK chain is six DH segments fitted to omniarm6.urdf.
   Calibrating only the constant left 12.32 mm; adding a joint-space sag
   integrator left 3.55 mm; servoing on grip_point() itself lands 0.008 / 0.003
   / 0.276 mm. The corrector runs INSIDE the one bridge motion (it bends that
   motion's own destination -- see descend_once), so it is one uninterrupted
   move, not a re-aim. If it ends outside tolerance the run FAILS and says so;
   it does not shuffle.

2. THE BLOCK NOW RIDES RIGIDLY (61.45 deg -> 0.60 deg of relative rotation).
   census() could not see this and neither could the verdict, so the ruler came
   first: sample R_tool^T R_block every step from the close through to the
   release and report max |angle| in degrees, WITH the axis in tool
   coordinates, because the two failure families want opposite fixes (about the
   pad NORMAL is torsion, which only friction resists; about either in-plane
   axis is tilt, which the contact patch resists geometrically).
   Measured: 61.45 deg about the TOOL AXIS mid-carry, returning to 2.9 deg --
   which is why every position check passed. Two causes, in order:
     (a) THE IK WAS SEEDED FROM A FIXED NOMINAL POSE. dls_ik_pose is a local
         method, so adjacent targets landed on branches ~180 deg apart at
         joints 4 and 6 and the bridge then interpolated between them in JOINT
         space: the tool's own world orientation swung 170 deg away and back
         inside one 3 s move with identical commanded end orientations.
         Seeding from the arm's current joints (goto seed="near") -> 13.46 deg.
     (b) THE WRIST WAS COUNTER-ROTATING 60.3 deg TO HOLD A FIXED COMPASS
         BEARING. Pick and place stand 60.3 deg apart in azimuth, so joint1
         swept -1.064 rad and joint6 swept +1.064 rad in world to cancel it.
         The gripper's world orientation stayed put; the block's did not.
         The residual is proportional to that counter-rotation RATE -- it peaks
         at peak wrist speed, and stretching the carry 3.0 -> 9.0 s took it
         13.46 -> 4.31 deg, i.e. tau*omega with tau = 0.43 s on both runs.
         Letting the tool ride round with the base (joint6 moves 0.009 rad
         instead of 1.064) -> 0.203 deg through the carry. The block arrives
         yawed by the azimuth it travelled, which is what being carried round
         looks like.
   WHY THIS AXIS AND NOT ANOTHER: the yaw is about the tool axis, which lies IN
   the pad plane, so it is resisted only by how far the contact patch spreads
   ACROSS the pad -- measured +/-11 mm in x against +/-33 mm in z, because the
   2F-140 pad is 22 mm wide and 65.5 mm long. It is this grasp's softest DOF by
   a factor of ~3, and torsional friction cannot help because that resists
   rotation about the pad NORMAL, a different axis.

   ⚠ MEASURED AND REFUTED FOR THE ROTATION SPECIFICALLY, so nobody re-runs
   them: newtonCondim 6 and newtonIterations 400 / newtonLsIterations 100 each
   reproduced 13.459 deg BIT-IDENTICALLY, along with every contact count and
   the penetration. (condim 6 had already been refuted against PENETRATION; it
   is now refuted against rotation too, which was a different failure mode and
   deserved its own test.) The rotation is a rigid-body/contact-geometry
   outcome, not a solver artifact -- do not go looking for it in the solver.
   NOT FULLY ATTRIBUTED: what supplies the ~0.2 N*m of yaw torque during the
   counter-rotation is not isolated. The rate-proportionality, the time
   constant and the fix are measured; the torque's origin is not.

===============================================================================
THE "FINGER CLOSING RATE COLLAPSES ~100x WITH ARM POSE" BUG IS SOLVED, AND THE
CAUSE WAS NOT IN THE GRIPPER AT ALL: THE PADS WERE STANDING ON THE TABLE.
===============================================================================
The previous revision measured, and could not explain, 156 mm/s of closing at
the spawn pose against 0.18 mm/s parked over the block, and shipped a
seat_pads() workaround that shuffled the arm until the pinch took. Four
hypotheses were tested and all four refuted -- servo gains, motor velocity,
target staleness, the compiled model. All four were about the ACTUATOR. The
answer was in the ENVIRONMENT, and one extra column found it: count the robot's
contacts while you measure the rate.

    arm pose            robot contacts    closing rate
    spawn, free space          0           >= 30 mm/s (limit to limit, exact)
    over the block             4           0.18-0.30 mm/s, BOTH directions

Those four contacts were all at world z = 0.1995-0.1998 -- the pick table's top
face -- up to 1.09 mm deep. THE PADS WERE RESTING ON THE TABLE. This world runs
newtonGroundMu 6 with an elliptic cone at newtonImpratio 100, i.e. friction
deliberately made near-rigid so a pinch cannot creep; 0.8 mm of pad into the
table is ~6.4 N of normal force and ~38 N of stiction against the ~21 N the
finger servo can pull at that error. The jaw physically could not slide inward.
It crept, symmetrically, in both directions, at a rate set by the solver's
residual slip -- which is exactly the signature that was mistaken for a servo
defect. The differential proof, all at one pose: the rate is identical with
bridge.tick() ON and OFF (so the bridge is innocent), identical opening and
closing (so it is not the target), and unchanged with the block deleted -- that
last leg proves nothing on its own, because a node deleted at runtime keeps its
collider under Newton, but the table was always the suspect.

And the reason the pads were on the table is a SECOND measurement error, this
one in the aim: THE FK TOOL POINT SITS ~8.4 mm ABOVE THE TRUE PAD MID-PLANE.
GRASP_Z 0.245 (the block's centre) nominally leaves 12.3 mm of pad-to-table
clearance; 8.4 mm of FK error plus ~4 mm of descent residual spent all of it and
0.3 mm more. The fix is not a bigger constant -- goto() now closes the loop on
grip_point() in Z as well as X and Y, so whatever OZ is wrong by, the aim
absorbs it, and the demo PRINTS its measured pad clearance and its robot-contact
count on every run. Measured after: 18.3 mm of clearance, 0 robot contacts
before closing, the close ramp tracked exactly, and the contact gate satisfied
on its FIRST poll (0.00 s).

seat_pads() is therefore DELETED, and so are the separate preclose and squeeze
stages it existed to rescue. The motion is now reach -> close -> lift -> move ->
place with no shuffling. A corollary worth having: the negative control is now
the same motion BY CONSTRUCTION rather than by bookkeeping, because there is no
grip-only manoeuvre left for it to mirror.

THE INSTRUMENT IS KEPT. PICK_DIAG=1 runs a full-stroke sweep at the spawn pose
(the regression guard for the ten-box collision) and then the rate battery at
the grasp pose, printing `robot_contacts` beside every rate. If a finger ever
seems slow again, read that column before touching a gain.

WHAT WAS TRIED FOR PENETRATION AND DID NOT WORK, so nobody re-runs it:
  * newtonCondim 6 (rolling friction): identical to condim 4, to 0.01 mm. A
    flat-face pinch gives rolling friction nothing to do.
  * newtonIterations 300 / newtonLsIterations 100: 0.75 vs 0.76 mm, i.e. noise.
  * newtonContactKe alone: raising it does NOT stiffen the contact. The engine
    derives MuJoCo's solref from the Ke/Kd PAIR, and measured, only the DAMPING
    RATIO moves -- ke 8000 -> 40000 at kd 200 took solref from [0.01, 1.118] to
    [0.01, 0.5] with timeconst unchanged, and scaling kd with sqrt(ke)
    reproduced the baseline to 0.01 mm at ke 20000 AND ke 40000. The one setting
    that read better (ke 20000 at kd 200 -> 0.43 mm) buys it by UNDER-DAMPING
    the contact, and ke 40000 at kd 200 shows where that road ends: 4.97 mm of
    penetration, the block 11.4 mm off-centre, placed tipped. Not shipped.
  * 5 mm of interference instead of 10: 0.59 mm, but the pinch drops to 2
    contacts per pad through the carry and the place lands 12 mm out.
  * grasping 3 mm lower (GRASP_Z 0.248): 0.75 vs 0.76 mm, and 3 mm less table
    clearance for it.
The 0.76 mm that remains is the pinch's own elastic deflection at 12.5 N/pad,
distributed as a slight tilt (0.755 mm at one pad corner, 0.004 mm at the
opposite one), and it is not reducible further without giving up grip.

NOT setForce, despite what docs/guide/friction-grasp.md says. setForce does not
put a Newton joint in force mode -- the PD servo stays live at effortLimit*10
N/m anchored at the last setPosition, so a "28 N squeeze" is really a spring
pulling to a target ~20 mm inside the part. It buries the pads, stores the
interference, and launches the part when the arm lifts. That is exactly how
friction_grasp_minimal "holds": the part is ejected at 3.5 m/s and lands on the
gripper's own wrist plate. OMNISIM_NEWTON_TORQUE_MODE=1 is the real force mode;
until a demo sets it, position control is the honest lever.

THE "7 mm STALL ASYMMETRY" WAS NOT A JOINT DEFECT -- IT WAS THE AIM, and the
same error in Z is what caused the rate collapse above. The FK tool point is a
constant ~5.5 mm out in Y and ~8.4 mm out in Z from the real pad mid-plane, so a
descent FK calls centred to 0.2 mm leaves the gripper 5 mm off laterally and
8 mm low. goto() re-aims on grip_point(), the pad mid-plane read off the finger
link Solids. Measured on the shipped run: three passes take the error from
(-2.6, -5.7, -8.4) mm to (+0.5, -0.0, -0.4) to (0.0, 0.0, 0.0), and the closed
pads come out symmetric to 0.01 mm (q=(0.03408, 0.03407), offcentre +0.00 mm).
The FK error itself is still NOT fixed -- the demo measures around it.

CENTRING THE PADS EXPOSED A PHYSICAL DEFECT that is still load-bearing. With the
block off-centre the grasp survived the carry; centred, it did not -- the block
rotated inside the gripper and levered the pads from 24 mm apart to 41 mm,
arriving tipped, identically on 3 of 3 runs. Root cause: every OmniSim contact
was condim 3, sliding friction only, so a pinched part spins about the contact
normal at zero cost. The world declares `newtonCondim 4` (torsional friction).
Keep it -- condim 6 adds nothing on top, but condim 3 still fails.

AIMING. Open-loop DLS IK lands 1.5-2.4 cm out at grasp height, which is fatal
when the pad clearance is millimetres. goto() closes the loop on the FK-measured
tool point: solve, move, measure the residual, re-solve for target-minus-
residual. Measured 2.4 cm -> 0.5 mm. This uses no sensing the arm does not
already have.

HONESTY. The verdict is geometric and adversarial: the block must be AIRBORNE
(clear of both tables) and CO-MOVING (its offset from the tool must stay fixed
while the tool travels). PICK_CONTROL_DROP=1 runs the negative control -- the
same motion with the jaws left 140 mm apart and no squeeze -- and the block must
be left behind; a "pass" there would mean the rig is holding it by something
other than the grip.

AND AIRBORNE + CO-MOVING IS STILL NOT ENOUGH: it proves the RIG holds the block,
not that the PADS do. The block is 90 mm tall against 65.5 mm pads, so it stands
proud of the pads at both ends and a "hold" could be a palm wedge or a shelf
with the fingers along for the ride -- which is exactly how the tree's CI-gated
friction_grasp_minimal "holds" (its part is ejected upward and lands on the
gripper's wrist plate). census() therefore asserts CONTACT: both pads must have
points on the block's own +/-y grasp faces while it is off the table. Measured
on the shipped configuration: 6 contacts per pad at the close, at the lift AND
through the 0.47 m carry, every one on a +/-y face at |block-local y| =
0.0346-0.0350 against a 0.035 half-width, peak pad penetration 0.76 mm, and ZERO
palm contacts at any sample.

THE PALM-WEDGE TEST ONLY GREW TEETH IN THIS REVISION. It used to be
structurally impossible to fail: OMNISIM_NEWTON_DUMP_MJMODEL showed 16 geoms and
NO gripper palm among them, because robotiq_2f140_base_link is a FIXED child of
link6 and OmSolid::flushPendingNewtonRegistrations DROPS a folded child's
boundingObject (OmSolid.cpp:3380) -- the mass rolls up, the geometry does not.
So the block could pass through the gripper body, the four-bar arms and the
fingertips, every one of which the CAD visual draws. The URDF now declares the
body box on link6 (the leader) and four boxes per jaw, and the compiled model
carries 23 geoms instead of 16. Self-collision cannot be a side effect: OmniSim
filters every intra-robot shape pair (OmNewtonBackend.cpp:2538-2601, nexclude=59
here), and PICK_DIAG=1 sweeps the full 100 mm stroke end to end as the guard.
census() also had to learn that "palm" now means link6: the body box is on the
LEADER of the fixed-joint merge, so a wedge reports under link6, and a name test
that only looked for a "robotiq" prefix would have filed it under `other` and
left palm_wedge False for a second reason.

⚠ BE PRECISE ABOUT WHAT THIS RUN EXERCISES. The pad boxes are proved by the
6-contacts-per-pad census. The body, bracket and arm boxes are proved to EXIST
and to be collidable (OMNISIM_NEWTON_DUMP_MJMODEL: geoms 7 and 10/11/14/15 on
the same bodies as the pad boxes, same contype 4 / conaffinity mask) and proved
not to jam the mechanism (the stroke sweep) -- but this world's block never
reaches them, and that is geometry, not luck: the gripper body's far face sits
78 mm behind the pad mid-plane while a 90 mm block grasped at mid-height stands
only 6.2 mm proud of the pad, and the fingertip bracket's inner face is set back
2 mm from the pad plane on purpose. So a palm wedge is impossible HERE for a
geometric reason, which is a very different claim from the previous revision's,
where it was impossible because the collider did not exist. A taller or wider
part will now be stopped; this one was never going to reach.

Two things make the contact assertion possible, and neither existed before the
2F-85 revision. (1) getContactPoints used to publish body-LOCAL support points
as world coordinates with depth hard-coded 0 -- the block "touched" link4 half a
metre away -- fixed engine-side in c59060688. (2) The node_id on a ContactPoint
names the QUERIED side, not the other body, so attributing a contact to a LINK
needs a second query against the robot's subtree; census() matches the two by
point identity.

WHAT WAS TAKEN FROM MUJOCO MENAGERIE (robotiq_2f85/2f85.xml). Menagerie has no
2F-140, but its 2F-85 is the reference model for this mechanism and two of its
choices port directly. (1) THE SPLIT PAD -- "Broke up collision pads into two
pads for more contacts": two abutting boxes along the finger's long axis, so the
pair resists pitch by NORMAL force instead of by friction torque. Adopted here
as two 32.75 mm halves whose union is bit-identical to the old single box;
measured 4 -> 6 contacts per pad, and 2 -> 6 sustained through the carry.
(2) EVERY STRUCTURAL LINK GETS A COLLIDER, with <exclude> for the adjacent
pairs; OmniSim filters all intra-robot pairs already, so the excludes are free.
WHAT COULD NOT BE PORTED: Menagerie's per-geom `priority`, `solimp`, `solref`,
`friction` and `condim` on the pads -- OmniSim exposes friction and condim
GLOBALLY only (newtonGroundMu / newtonCondim) and priority/solimp/margin/gap not
at all -- and its underactuated four-bar, built from <equality><connect> plus a
<tendon> and a spring-loaded passive DOF, which the URDF path here cannot
express; these two prismatic jaws remain a mid-stroke approximation. Note that
Menagerie's own pads run condim 3 and buy stability from the split pad plus an
elliptic cone at impratio 10; this world still needs condim 4 on top, which is
consistent with its single-collider history rather than a contradiction.
"""
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

from omnisim import Supervisor                              # noqa: E402
from omnilink_arm_bridge import (ArmBridge, dls_ik_pose,    # noqa: E402
                                 forward_kinematics_pose)
from _arm_configs import get_config                          # noqa: E402

OUT = os.environ.get("PICK_OUT", os.path.join(_HERE, "_2f140_pick_result.json"))
CONTROL_DROP = os.environ.get("PICK_CONTROL_DROP") == "1"
DEBUG = os.environ.get("PICK_CONTACT_DEBUG") == "1"

# link6 -> PAD MID-HEIGHT. 0.1655 (link6->flange) + 0.17645 (flange->pad mid).
# The 0.17645 is MEASURED off the CAD, not assumed: the pad's 14.32 cm2 inner
# face spans mesh z 0.14736..0.21284 and the body's 29.3 cm2 mounting face sits
# at mesh z 0.00362, so the pad runs 0.1437..0.2092 above the flange.
OZ = 0.34195
# 70 mm across the grasp axis => q = 0.035 at contact, DEAD CENTRE of the
# 2F-140's 0..0.070 per-pad stroke. That is deliberate: two prismatic joints
# are a parallel-jaw approximation of an angular four-bar linkage, whose pads
# are only truly parallel near mid-stroke (they toe in closed, out open).
# CLOTH: the particle radius, i.e. the fabric's half-thickness. The rigid demo
# used 0.035 for a 70 mm block; everything downstream scales off this constant.
BLOCK_HALF = float(os.environ.get("PICK_HALF", "0.010"))
# INHERITED FROM THE 2F-85 DEMO, WHERE 3/6/10 mm WAS SWEPT AND 10 mm WON. That
# sweep is NOT re-run here, so treat 10 mm as "the value the sibling demo
# validated", not as a 2F-140 measurement. What it means is different on this
# gripper: the target is a DISPLACEMENT, and this gripper's servo is 2.5x
# stiffer (kp 1250 vs 500), so the same 10 mm is ~12.5 N/pad here against the
# 2F-85's ~5 N. Still a quarter of the 2F-140's 125 N datasheet grip force.
# Overridable for sweeps.
# ⚠ 2.5 mm, not 10. Newton's own cloth grasp closes ~2 mm inside the particle
# shell and calls it a gentle pinch; a block tolerates squeezing, fabric is
# extruded. The jaws are still EXPECTED to stall short, on the particles.
INTERFERENCE = float(os.environ.get("PICK_INTERFERENCE", "0.0025"))
TABLE_TOP = 0.20

# ⚠ PAD FACE GEOMETRY, and the 7 mm error that made the first version of the
# sibling 2F-85 demo a lie. The invariant both grippers are built around is
#
#       the pad's INNER FACE sits at exactly y = q
#
# reached by cancelling the joint origin against the pad box's half-thickness.
# omniarm6_2f85_grip.urdf cancels 0.007 against 0.007 (a 14 mm pad);
# omniarm6_2f140_grip.urdf cancels 0.0095 against 0.0095 (the CAD-measured 19 mm
# pad). Writing q = BLOCK_HALF - halfT + clearance (i.e. treating the joint
# origin as an EXTRA offset) buries the pad halfT INTO the part before any force
# is applied. Measured on the 2F-85 with that bug: left pad 5.1 mm inside the
# block, right pad 0.9 mm clear -- a one-sided interpenetrating hold that still
# passed a carry test. So:
#   pad inner face  = q
#   clearance c     => q = BLOCK_HALF + c
#   interference i  => q = BLOCK_HALF - i
#
# ⚠ AND THE SQUEEZE IS A POSITION TARGET, NOT setForce. setForce does NOT put a
# Newton joint in force mode: the PD servo stays live at effortLimit*10 N/m
# anchored at the last setPosition, so setForce(-28) is really a spring pulling
# to a target ~20 mm inside the part -- it buries the pads, stores the
# interference, and launches the part when the arm lifts. That is exactly how
# friction_grasp_minimal "holds": the part is ejected at 3.5 m/s and lands on
# the gripper's wrist plate. Position mode is what the engine actually
# implements, so use it honestly.
#
# kp = effort * 10 (OmBasicJoint.cpp:127 and :675, read from the source; the
# 2F-85 demo measured 500 in the compiled model at effort 50, which agrees).
# The 2F-140's datasheet grip force is 10-125 N, so the URDF declares
# effort=125 and the servo comes out at 1250 N/m. 10 mm of interference is then
# ~12.5 N per pad; at mu 6 that is ~75 N of friction per pad against a 1.47 N
# block. The margin is absurd on purpose -- the demo is not trying to find the
# minimum grip, it is trying to make the hold unambiguous.
GRIP_KP = 1250.0
# Finger-link origin -> pad mid-height, i.e. half the 65.5 mm measured pad.
PAD_HALF_L = 0.03275

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)

IK = bridge.cfg["ik"]
IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]
TOP = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]

# ⚠ SIDEWAYS TOOL FOR A HANGING SHEET, AND THIS IS THE FIX FOR "THE ROBOT HITS
# THE RAIL". With TOP the tool points straight DOWN, which puts the wrist
# 0.25 m ABOVE the pads (OZ). Grasping a hanging cloth at z = 0.251 therefore
# puts the wrist at z ~ 0.50 -- level with the rail the sheet hangs from, and
# inside the sheet's own vertical span. The wrist ploughs through both on the
# way in; observed in the GUI as the arm passing through the rail, and in the
# telemetry as the fabric being shoved 38 mm.
#
# Raising the rail cannot fix that: the cloth hangs BELOW the rail, so a
# downward tool always has its wrist inside the fabric. The tool has to lie
# HORIZONTAL, pointing at the sheet along -x. Then the wrist trails 0.25 m
# BEHIND the pads in +x, outside the cloth entirely, and only the two pads
# ever enter the sheet's plane.
#
# Columns are the tool axes in world: tool x -> +z, tool y -> +y (the pads
# still separate along world y, which is what lets them close across a sheet
# in the x-z plane), tool z -> -x (the approach direction).
SIDE = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]

block = robot.getFromDef("BLOCK")   # None in the cloth world, by design
# ⚠ CLOTH MODE. This is the 2F-140 pick-and-place demo with the rigid block
# replaced by a hanging Cloth. A Cloth has NO supervisor node -- no pose, no
# contact points -- so every block-derived reading below degrades instead of
# crashing, and the grasp is proven from the engine-side cloth telemetry
# (OMNISIM_CLOTH_TELEMETRY) rather than from this controller. Do not read a
# clean exit here as evidence the fabric moved.
CLOTH_MODE = block is None
# Where the fabric hangs. The sheet is a vertical plane at y = 0 spanning
# x 0.41..0.51, so this is its centre at pinch height.
CLOTH_GRASP = (0.46, 0.0, float(os.environ.get("PICK_GRASP_Z", "0.251")))
omniarm6 = robot.getFromDef("OMNIARM6")

# Block half-extents (Box 0.05 0.05 0.09) -- used to name the face a contact
# landed on, which is the whole point of the census below.
BHALF = (0.035, 0.035, 0.045)

# Drive the finger motors DIRECTLY. The gripper effector only knows how to push
# position targets, which is exactly what does not grip.
fm = robot.getDevice("robotiq_2f140_finger_motor")
mm = robot.getDevice("robotiq_2f140_finger_mirror_motor")
fs = robot.getDevice("robotiq_2f140_finger_sensor")
ms = robot.getDevice("robotiq_2f140_finger_mirror_sensor")
for s in (fs, ms):
    s.enable(dt)
for m in (fm, mm):
    m.setAvailableForce(m.getMaxForce())
    # ⚠ EXPLICIT, even though getVelocity() already reports maxVelocity. See the
    # FINGER CLOSING RATE note in the module docstring: without this the pads
    # crawl at ~1.5 mm/s against a declared 70 mm/s.
    if os.environ.get("PICK_NO_SETVEL") != "1":
        m.setVelocity(m.getMaxVelocity())

def _finger_solids():
    """The two finger link Solids, resolved WITHOUT waiting for a contact.

    The URDF importer emits no DEFs, so the route is device -> the node that
    owns the device -> the joint's endPoint. Returns {} if any hop fails; the
    caller then falls back to the FK aim.
    """
    out = {}
    for key, dev in (("L", fs), ("R", ms)):
        try:
            n = robot.getFromDevice(getattr(dev, "_tag", None))
            j = n.getParentNode() if n is not None else None
            f = j.getField("endPoint") if j is not None else None
            ep = f.getSFNode() if f is not None else None
            if ep is not None:
                out[key] = ep
        except Exception:
            pass
    return out


FINGERS = _finger_solids()

log = []


def emit(s):
    log.append(s)
    print(s, flush=True)


# ── BLOCK-RELATIVE ORIENTATION: the ruler the verdict was BLIND to ──────
#
# ⚠ `drift` measures POSITION ONLY. A block that stays exactly where it is
# relative to the tool but SPINS inside the jaws passes every check this demo
# had -- carried, pinched, palm_wedge False, drift 8 mm, placed True -- and
# looks, to a human watching, like the cube "has a mind of its own". That is
# the same class of hole as the carry-vs-tray problem the census was added for:
# the assertion did not measure the thing that was wrong.
#
# The measurement is a relative rotation between two rigid bodies, so it needs
# no kinematic model and no FK:
#   R_rel(t) = R_tool(t)^T R_block(t)     block axes expressed in tool axes
#   D        = R_rel(t) R_rel(t0)^T       what the block did INSIDE the gripper,
#                                         expressed in the TOOL frame
# and |angle(D)| is the number. R_tool is read off the LEFT FINGER LINK Solid,
# not off FK: the finger joints are prismatic, so the link's orientation IS the
# gripper's, measured by the engine.
#
# ⚠ THE AXIS OF D IS THE DIAGNOSIS, NOT DECORATION, because the two families of
# relative rotation have different fixes:
#   * about the finger link's +y (the GRASP axis / the pad's own normal): pure
#     torsion about the contact normal. Two flat pads resist this ONLY through
#     torsional friction, i.e. through newtonCondim >= 4. Geometry gives nothing.
#   * about +x or +z (in the pad PLANE): tilting. The contact PATCH has extent,
#     so this is resisted by the normal-force distribution across the split
#     pads -- geometric, and orders of magnitude stiffer.
# So a torsional failure and a tilting failure look identical in `drift` and
# want opposite remedies. Report the axis.
TOOL = FINGERS.get("L")


def _mT_m(A, B):
    """A^T B, row-major flat 3x3."""
    return [sum(A[k * 3 + i] * B[k * 3 + j] for k in range(3))
            for i in range(3) for j in range(3)]


def _m_mT(A, B):
    """A B^T, row-major flat 3x3."""
    return [sum(A[i * 3 + k] * B[j * 3 + k] for k in range(3))
            for i in range(3) for j in range(3)]


ROT = {"ref": None, "phase": None, "max_deg": 0.0, "max_axis": None,
       "max_phase": None, "max_t": None, "n": 0, "per_phase": {},
       "last_deg": 0.0, "tool0": None, "block0": None}
# PICK_ROT_TRACE=<N>: emit one trace line every N steps (N=12 is ~0.1 s at
# basicTimeStep 8).
ROT_TRACE = int(os.environ.get("PICK_ROT_TRACE", "0"))


def rot_rel():
    """The block's orientation expressed in the gripper's own frame."""
    if TOOL is None:
        return None
    return _mT_m(TOOL.getOrientation(), borient())


def _ang_of(D):
    tr = D[0] + D[4] + D[8]
    return math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (tr - 1.0)))))


def rot_arm(tag):
    """Latch the reference: from here on, 0 deg means 'rigid with the jaws'."""
    ROT["ref"] = rot_rel()
    ROT["tool0"] = TOOL.getOrientation() if TOOL is not None else None
    ROT["block0"] = borient()
    ROT["phase"] = tag
    ROT["max_deg"] = 0.0
    ROT["max_axis"] = None
    ROT["n"] = 0
    ROT["per_phase"] = {}


def rot_phase(tag):
    ROT["phase"] = tag


def rot_sample():
    """One sample of |angle(D)|, in degrees, with D's axis in TOOL axes."""
    if ROT["ref"] is None or TOOL is None:
        return None
    cur = rot_rel()
    D = _m_mT(cur, ROT["ref"])
    tr = D[0] + D[4] + D[8]
    c = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    ang = math.degrees(math.acos(c))
    ax = [D[7] - D[5], D[2] - D[6], D[3] - D[1]]
    n = math.sqrt(sum(v * v for v in ax))
    ax = [v / n for v in ax] if n > 1e-12 else [0.0, 0.0, 0.0]
    ROT["n"] += 1
    ROT["last_deg"] = ang
    ph = ROT["phase"]
    if ang > ROT["per_phase"].get(ph, -1.0):
        ROT["per_phase"][ph] = ang
    if ang > ROT["max_deg"]:
        ROT["max_deg"] = ang
        ROT["max_axis"] = [round(v, 3) for v in ax]
        ROT["max_phase"] = ph
        ROT["max_t"] = round(robot.getTime(), 3)
    # ⚠ THE DECISIVE COLUMN, and the reason this trace exists rather than a
    # theory: |D| alone cannot tell "the block spun in the jaws" from "the TOOL
    # swung and the block was left behind". Log BOTH bodies' own world-frame
    # rotation since the reference next to it, and the question answers itself.
    if ROT_TRACE and ROT["n"] % ROT_TRACE == 0:
        tw = _ang_of(_m_mT(TOOL.getOrientation(), ROT["tool0"]))
        bw = _ang_of(_m_mT(borient(), ROT["block0"]))
        # block position in the TOOL frame + the two finger readings: enough to
        # tell a rotation that keeps the pinch from one that opens it.
        bp, tp = bpos(), TOOL.getPosition()
        Rt = TOOL.getOrientation()
        dv = [bp[k] - tp[k] for k in range(3)]
        loc = [dv[0] * Rt[c] + dv[1] * Rt[3 + c] + dv[2] * Rt[6 + c]
               for c in range(3)]
        emit("[rottrace] t=%7.3f %-8s rel=%7.2f  tool_world=%7.2f  "
             "block_world=%7.2f  axis_tool=(%+.2f,%+.2f,%+.2f) "
             "blk_in_tool_mm=(%+.1f,%+.1f,%+.1f) q_f=(%.4f,%.4f) q=%s"
             % (robot.getTime(), ph, ang, tw, bw, ax[0], ax[1], ax[2],
                loc[0] * 1000.0, loc[1] * 1000.0, loc[2] * 1000.0,
                fs.getValue(), ms.getValue(),
                [round(v, 3) for v in bridge._read_q()]))
    return ang


def rot_report(tag):
    if ROT["ref"] is None:
        return
    a = rot_sample()
    emit("[rot] %-9s now=%.2f deg  max=%.2f deg (phase=%s t=%s axis_tool=%s"
         " -- y is the GRASP axis: torsion; x/z are in-pad-plane: tilt)"
         % (tag, a, ROT["max_deg"], ROT["max_phase"], ROT["max_t"],
            ROT["max_axis"]))


# ── THE PRE-PICK RULER ─────────────────────────────────────────────────
# ROT above cannot see anything before the close: it measures the BLOCK in the
# gripper, and before the close there is no grasp to be rigid with. The
# user-visible complaint "the robot does a strange rotation before it picks the
# cube" is therefore invisible to every number this demo prints.
#
# What to measure. Every pre-pick move commands the SAME tool orientation
# (TOP), so the tool's world orientation ought to be constant from the moment
# it first reaches TOP until the jaws close. Any excursion in between is the
# tool taking the long way round -- so sample R_tool and report the largest
# angle between any sample and the pose the descent ends in. Alongside it,
# per-joint PATH LENGTH vs NET DISPLACEMENT: a joint that travels far more than
# it ends up moving went out and came back, which is the signature of a
# joint-space interpolation between two different IK branches.
WIND = {"on": os.environ.get("PICK_PREPICK_TRACE", "1") == "1",
        "R": [], "q_prev": None, "path": None, "q0": None, "n": 0}
WIND_EVERY = int(os.environ.get("PICK_PREPICK_EVERY", "6"))


def wind_sample():
    if not WIND["on"] or TOOL is None:
        return
    q = list(bridge._read_q())
    if WIND["q_prev"] is None:
        WIND["q_prev"] = q
        WIND["q0"] = list(q)
        WIND["path"] = [0.0] * len(q)
    else:
        for i in range(min(len(q), len(WIND["path"]))):
            WIND["path"][i] += abs(q[i] - WIND["q_prev"][i])
        WIND["q_prev"] = q
    WIND["n"] += 1
    if WIND["n"] % WIND_EVERY == 0:
        WIND["R"].append((round(robot.getTime(), 3), PHASE[0],
                          TOOL.getOrientation()))


def wind_report(tag):
    """Excursion of the tool's own world orientation, against the orientation
    it FINISHES in -- every pre-pick move commanded the same one.

    ⚠ THE HEADLINE NUMBER ALONE PROVES NOTHING, which is why the per-move table
    below it is the real output. "180 deg from the pose it ends in" is also
    what a move that simply HAS NOT ARRIVED YET reads, and the arm genuinely
    does start a half-turn away from TOP. The decisive comparison is per
    commanded move: NET (how far the tool's orientation actually moved between
    the start and the end of that move) against PEAK (the furthest it got from
    its own starting orientation while doing it). peak >> net means the tool
    left and came back inside one move -- an excursion nobody asked for.
    It is also the one measure immune to joint-angle WRAPPING: a q reading that
    flips +180 -> -180 injects a phantom 360 into a joint path length, while an
    orientation matrix cannot lie about where the tool is pointing."""
    if not WIND["on"] or not WIND["R"]:
        return None
    Rend = WIND["R"][-1][2]
    worst = (0.0, None, None)
    per_phase = {}
    for t, ph, R in WIND["R"]:
        a = _ang_of(_m_mT(R, Rend))
        if a > per_phase.get(ph, -1.0):
            per_phase[ph] = a
        if a > worst[0]:
            worst = (a, ph, t)
    # Per commanded move: net vs peak, both in the tool's own world frame.
    bounds = [(m["phase"], m["t"]) for m in MOVES]
    seg = []
    for i, (ph, t0) in enumerate(bounds):
        t1 = bounds[i + 1][1] if i + 1 < len(bounds) else 1e9
        pts = [(t, R) for t, _p, R in WIND["R"] if t0 <= t < t1]
        if len(pts) < 2:
            continue
        R0, R1 = pts[0][1], pts[-1][1]
        net = _ang_of(_m_mT(R1, R0))
        peak = max(_ang_of(_m_mT(R, R0)) for _t, R in pts)
        seg.append({"phase": ph, "t": t0, "net_deg": round(net, 2),
                    "peak_deg": round(peak, 2),
                    "excursion_deg": round(peak - net, 2)})
    for s in seg:
        emit("[prepick] move %-9s t=%-7s tool net=%7.2f deg  peak=%7.2f deg"
             "  EXCURSION=%7.2f deg%s"
             % (s["phase"], s["t"], s["net_deg"], s["peak_deg"],
                s["excursion_deg"],
                "   <-- went out and came back" if s["excursion_deg"] > 5.0
                else ""))
    q = list(bridge._read_q())
    swing = [(round(math.degrees(WIND["path"][i]), 1),
              round(math.degrees(q[i] - WIND["q0"][i]), 1))
             for i in range(min(len(q), len(WIND["path"])))]
    emit("[prepick] %s tool-orientation excursion vs the pose it ends in: "
         "max=%.2f deg (phase=%s t=%s) per-phase=%s"
         % (tag, worst[0], worst[1], worst[2],
            {k: round(v, 2) for k, v in per_phase.items()}))
    emit("[prepick] %s per-joint (path_deg, net_deg) = %s   -- path >> |net| "
         "means the joint went out and came back" % (tag, swing))
    return {"max_deg": round(worst[0], 3), "phase": worst[1], "t": worst[2],
            "per_phase": {k: round(v, 3) for k, v in per_phase.items()},
            "joint_path_net_deg": swing}


def _tick():
    """ONE simulation step. Every stepping loop in this file goes through here
    so the relative-rotation ruler cannot be blind to a phase by omission."""
    if robot.step(dt) == -1:
        return False
    bridge.tick(robot.getTime())
    rot_sample()
    wind_sample()
    return True


def step_for(secs):
    for _ in range(int(secs * 1000 / dt)):
        if not _tick():
            return False
    return True


def tcp():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))[0]


# ── MOTION LEDGER ──────────────────────────────────────────────────────
# Every commanded arm move, tagged with the phase it belongs to. The point is
# the DESCENT: the user's report was "when it goes down, it moves a little bit
# around until it picks the cube", which is a segment COUNT, so count it and
# print it rather than trusting that a refactor kept it at one.
MOVES = []
PHASE = ["init"]
# Filled in by the single-shot descent: the calibration it applied and the
# residual it achieved, so a regression shows up as a number and not as a mood.
DESCENT_AIM = {"err_mm": None, "calib_mm": None}


def phase(name):
    PHASE[0] = name


def move(q, dur):
    # ⚠ THE COMMANDED q GOES IN THE LEDGER, not just the timestamp. A branch
    # flip between two adjacent IK solves is a jump in the COMMAND, and the
    # bridge then interpolates the arm across it in joint space; without the
    # commanded vector the ledger cannot tell that apart from a clean move.
    _from = list(bridge._read_q())
    MOVES.append({"phase": PHASE[0], "t": round(robot.getTime(), 3),
                  "dur": dur,
                  "to_deg": [round(math.degrees(v), 1) for v in q],
                  "d_deg": [round(math.degrees(q[i] - _from[i]), 1)
                            for i in range(min(len(q), len(_from)))]})
    bridge.act_set_joint_positions(q, duration_s=dur)


def seg_count(name):
    return len([m for m in MOVES if m["phase"] == name])


def goto(xyz, dur=1.6, passes=3, tol=0.002, seed=None, ik=None, R=None):
    # In cloth mode every waypoint uses the sideways tool unless the caller
    # names an orientation explicitly.
    if R is None and CLOTH_MODE:
        R = SIDE
    """Closed-loop on the FK tool point -- see the module docstring.

    ⚠ `passes` > 1 IS VISIBLE MOTION. Each pass is a separate commanded move,
    so a 3-pass goto at the grasp pose is three little corrections the audience
    reads as the arm hunting for the block. Above the part that is free and
    harmless; at the part it is the defect. The descent therefore calls this
    with passes=1 and pays for its accuracy with a CALIBRATION taken up at the
    approach height instead -- see calibrate_descent().
    """
    goal, err = list(xyz), 1e9
    for _pass in range(passes):
        # ⚠ seed="near" IS NOT A MICRO-OPTIMISATION -- IT IS THE FIX FOR THE
        # WRIST WIND-UP. dls_ik_pose is a LOCAL method: seeded from the fixed
        # NOM_SEED it returns whichever branch that seed happens to fall into
        # for THIS target, and two adjacent targets can land on branches ~180
        # deg apart at joints 4 and 6. The bridge then interpolates the arm
        # between them in JOINT space, so the tool takes the long way round --
        # measured on the carry: the tool's own world orientation swung 170 deg
        # away from its start and back again inside one 3 s move, with the
        # commanded start and end orientations IDENTICAL. Seeding from where
        # the arm actually is picks the nearest solution instead.
        # ⚠ AND A CORRECTION PASS IS *ALWAYS* SEEDED NEAR, whatever the caller
        # asked for. Pass 0 may legitimately want the nominal seed -- it can be
        # a long move from an unrelated pose, where a local solve has no
        # branch worth preserving. Pass 1+ is a millimetre-scale correction to
        # a pose the arm is ALREADY IN, so re-seeding it from the nominal is
        # pure downside: measured on the approach + calibrate moves, it handed
        # back the opposite wrap of joint 6 (+180.0 deg after -180.0 deg -- the
        # SAME tool orientation) and the bridge then spun the wrist a full turn
        # in joint space to "reach" the pose it was already at.
        s = (list(bridge._read_q()) if (seed == "near" or _pass > 0)
             else list(NOM_SEED if seed is None else seed))
        q, _p, _r, _i = dls_ik_pose(IK["chain"], s,
                                    goal, TOP if R is None else R,
                                    (0.0, 0.0, OZ),
                                    IK if ik is None else ik, IK_LIMITS)
        move(q, dur)
        step_for(dur + 0.3)
        t = tcp()
        resid = [xyz[i] - t[i] for i in range(3)]
        err = max(abs(v) for v in resid)
        if err <= tol:
            break
        goal = [goal[i] + resid[i] for i in range(3)]
        dur = 0.5
    return err


def descend_once(pad_target, dur=None, tail=None, tag="descend", R=None,
                 yaw=0.0, servo_from=None):
    """ONE uninterrupted commanded move that lands the REAL PADS on
    `pad_target`, correcting in flight instead of re-aiming afterwards.

    ⚠ IT MUTATES THE LIVE BRIDGE MOTION'S OWN DESTINATION. That is deliberate
    and it is what keeps this ONE segment: ArmBridge.tick re-reads
    params["to_q"] every step while interpolating, and _finish_into_hold then
    copies it into the hold, so bending to_q mid-flight bends the SAME motion
    instead of issuing another. Do not "clean this up" into a second
    act_set_joint_positions -- that is precisely the shuffle this replaced.

    ⚠ AND IT CORRECTS ON grip_point(), NOT ON THE JOINTS. An earlier revision
    of this function integrated the JOINT residual (q_commanded - q_measured),
    which removed the gravity sag exactly -- 12.32 mm of height error down to
    0.29 mm -- and still left 3.55 mm in x, because sag is only two of the
    three error terms between "the goal I commanded" and "where the pads are":
        FK tool-point offset   CONSTANT (-1.0, +5.7, +8.1 mm), calibrated once
                               at the approach height and never re-measured
        gravity sag            POSE-DEPENDENT (0.46 mm at the approach pose,
                               12.3 mm at the grasp pose)
        DH model error         POSE-DEPENDENT (~3.5 mm over a 160 mm descent);
                               the arm's IK chain is six DH segments fitted to
                               omniarm6.urdf, not the URDF itself
    Only the first is a constant, so only the first can be calibrated up top.
    Servoing the last two on the measured pad plane costs nothing extra -- the
    measurement already exists and is already trusted for the aim -- and it is
    also what a real arm with a closed-loop tool frame does.
    """
    dur = DESCEND_S if dur is None else dur
    tail = SAG_TAIL_S if tail is None else tail
    # ⚠ MEASURED THE HARD WAY: THIS FRACTION IS NOT A TUNING KNOB, IT IS A
    # FUNCTION OF HOW FAR THE RAMP TRAVELS. The corrector integrates the
    # pad-plane error, and "error" is only meaningful once the motion is
    # essentially stopped -- before that the residual is dominated by the
    # motion's own tracking lag, i.e. by NOT HAVING ARRIVED YET. The default
    # 0.70 was fitted to a 160 mm drop, where 70% through leaves millimetres.
    # Reused unchanged on a full spawn->grasp reach it integrated the transit
    # itself: in-flight correction -312.63 / -4.30 / -237.17 mm and a pad plane
    # 69 mm from target. A longer ramp must arm the servo LATER.
    servo_from = SERVO_FROM if servo_from is None else servo_from
    goal = goal_for(pad_target, yaw)
    seed = list(bridge._read_q())
    R = TOP if R is None else R
    q, pe, rr, it = dls_ik_pose(IK["chain"], seed, list(goal), R,
                                (0.0, 0.0, OZ), IK_TIGHT, IK_LIMITS)
    move(q, dur)
    t0 = robot.getTime()
    corr = [0.0, 0.0, 0.0]
    i = 0
    for _ in range(int((dur + tail) * 1000 / dt)):
        if not _tick():
            break
        i += 1
        # ⚠ ARMED ONLY FOR THE LAST (1-SERVO_FROM) OF THE RAMP. Before that the
        # pad-plane error is dominated by the motion's own TRACKING LAG, and
        # integrating lag would drive the arm past the target and back -- the
        # visible shuffle in a different costume. The eased interp is nearly
        # stopped by then, so what is left is the real, static error.
        if (robot.getTime() - t0) < servo_from * dur or i % SERVO_EVERY:
            continue
        g = grip_point()
        if g is None:
            break
        e = [pad_target[k] - g[k] for k in range(3)]
        corr = [corr[k] + SERVO_KI * e[k] for k in range(3)]
        q, _p2, _r2, _i2 = dls_ik_pose(
            IK["chain"], list(q), [goal[k] + corr[k] for k in range(3)], R,
            (0.0, 0.0, OZ), IK_TIGHT, IK_LIMITS)
        kind, mp = bridge.motion
        tgt = mp.get("to_q") if kind == "interp" else mp.get("q")
        if tgt:
            tgt[:] = list(q)
        if TRACE:
            emit("[trace] %-9s t=%.3f pad_err=(%+.2f,%+.2f,%+.2f) corr=%s mm"
                 % (tag, robot.getTime() - t0, e[0] * 1000.0, e[1] * 1000.0,
                    e[2] * 1000.0, [round(v * 1000.0, 2) for v in corr]))
    emit("[pick] %s: ONE segment, %.2f s (+%.2f s settle); IK residual "
         "pos=%.5f rot=%.5f in %d iters; in-flight correction = %s mm"
         % (tag, dur, tail, pe, rr, it, [round(v * 1000.0, 2) for v in corr]))
    return q, corr


def fingers():
    return fs.getValue(), ms.getValue()


def grip_point():
    """Where the pads' symmetry plane ACTUALLY is, in world coordinates.

    ⚠ THIS IS NOT THE FK TOOL POINT, AND THE DIFFERENCE IS THE WHOLE DEFECT
    THIS DEMO SPENT A REVISION BLAMING ON THE MIRRORED PRISMATIC JOINT. The
    finger link origins sit at gripper-local y = +(0.0095+q_l) and -(0.0095+q_r),
    so their midpoint is offset from the gripper axis by exactly (q_l-q_r)/2;
    undo that and you have a point on the axis, measured from the engine's own
    Solid poses with no kinematic model in the loop. Measured against it, the
    FK tool point is a CONSTANT +5.5 mm out in y (5.66 / 5.73 / 5.45 mm at
    three unrelated arm poses), which is why aiming the FK point at the block
    left the gripper 4-5 mm off-centre and the two pads stalled ~7 mm apart on
    identical targets. The joints themselves are honest: the measured
    separation of the two link origins matches 0.019+q_l+q_r to 0.6 mm.

    ⚠ AND IT MUST BE TAKEN AT THE PAD'S MID-HEIGHT, not at the link origins.
    The origins sit 25 mm up the tool axis from the pad centre; the arm holds
    the tool ~3 deg off vertical (goto closes the loop on position only), so
    reading the axis at the wrong height smears that tilt into a ~1.3 mm
    lateral error. Measured: an aim loop that skipped this correction fixed the
    5.6 mm y error and introduced a 3.0 mm x error, and the block -- now
    pinched off-centre along the pads' long axis -- rotated inside the gripper
    during the carry and levered the pads open from 24 mm to 41 mm. It failed
    identically on all three runs, so this is a mechanism, not a flake.
    """
    L, R = FINGERS.get("L"), FINGERS.get("R")
    if L is None or R is None:
        return None
    lp, rp = L.getPosition(), R.getPosition()
    ql, qr = fingers()
    d = [lp[i] - rp[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    if n < 1e-9:
        return None
    u = [v / n for v in d]                      # gripper +y, in world
    m = L.getOrientation()                      # row-major, local -> world
    z = [m[2], m[5], m[8]]                      # the pad's own +z, in world
    return [0.5 * (lp[i] + rp[i]) - u[i] * 0.5 * (ql - qr) + z[i] * PAD_HALF_L
            for i in range(3)]


def open_fingers():
    for m in (fm, mm):
        m.setPosition(0.070)


TRACE = os.environ.get("PICK_TRACE") == "1"
# Jaw half-opening to PRE-SET before the arm moves. Not a workaround any more,
# just how a gripper is driven: you size the jaws to the part on approach, not
# at the part. 8 mm of clearance per side leaves an 86 mm opening for a 70 mm
# block, which absorbs the descent's few mm of lateral aim error without a pad
# clipping the block on the way down.
PRESET_CLEAR = float(os.environ.get("PICK_PRESET_CLEAR", "0.008"))
# Single closing ramp: from the preset clearance straight to the interference.
CLOSE_S = float(os.environ.get("PICK_CLOSE_S", "1.2"))
# Carry duration, in seconds, for the 0.47 m pick -> place move. A knob because
# it is the first suspect for a block that rotates inside the jaws: the arm
# swings through ~60 deg of base yaw here, and the joint-space interpolation
# does not hold the tool's orientation fixed on the way, so the block is asked
# to follow an angular acceleration that only friction can deliver.
CARRY_S = float(os.environ.get("PICK_CARRY_S", "3.0"))
# The single descent, approach height -> grasp pose. One segment, no passes.
DESCEND_S = float(os.environ.get("PICK_DESCEND_S", "2.0"))
# The transit: spawn pose -> 160 mm above the block, in ONE commanded move.
# 2.5 s rather than the old 2.0 s + a 0.5 s correction pass, so the arm covers
# the same ground in one unbroken ramp instead of two.
REACH_S = float(os.environ.get("PICK_REACH_S", "2.5"))
# The place descent. Longer than the pick descent on purpose: it reconfigures
# the elbow and wrist far more than the pick descent does (joints 2/3/5 move
# 0.19/0.77/0.58 rad against a 0.22 m drop), and relative rotation scales with
# rate -- see the carry note.
LOWER_S = float(os.environ.get("PICK_LOWER_S", "3.0"))
# ── GRAVITY-SAG CORRECTOR, applied INSIDE the descent ───────────────────
# ⚠ MEASURED, and it is not the FK error: after a single calibrated descent
# the pads landed 8.9 mm short in x and 12.3 mm LOW, and the joint residual was
# [0, -11.65, -16.72, 0, +0.07, 0] mrad -- joints 2 and 3, the two pitch joints
# that carry gravity, and NOTHING else. It is STATIC: identical at +0.5 s and
# at +4.0 s of settling, so it is not tracking lag and no amount of waiting
# removes it. It is the position servo's steady-state error under load, and it
# is the engine's designed behaviour, not a bug: OmBasicJoint.cpp:675 builds a
# position-controlled joint at targetKe = effortLimit*10, whose own comment
# says that holds "a 194 N*m arm shoulder to <0.02 rad" -- 0.02 rad is exactly
# what we are looking at.
# ⚠ AND IT IS POSE-DEPENDENT, which is why it cannot be folded into the
# approach-height calibration: measured 0.46 mm at the approach pose and
# 12.3 mm at the grasp pose, 160 mm lower. Calibrating it up there and
# subtracting it down here would remove 4% of it.
# So the descent carries an integral corrector that pushes the commanded joint
# target until the MEASURED joints reach the solved ones. It runs inside the
# one bridge motion (it mutates that motion's own destination in place), so it
# is not a second segment and there is no stop-and-restart: the last few mm of
# sag are taken out as the move decelerates.
# It is armed only for the last (1-SERVO_FROM) of the ramp, because before
# that the joint error is dominated by TRACKING LAG -- integrating lag would
# make the arm overshoot the target and then come back, which is the visible
# shuffle in a different costume.
SERVO_FROM = float(os.environ.get("PICK_SERVO_FROM", "0.70"))
SERVO_KI = float(os.environ.get("PICK_SERVO_KI", "0.10"))
SERVO_EVERY = int(os.environ.get("PICK_SERVO_EVERY", "4"))
SAG_TAIL_S = float(os.environ.get("PICK_SAG_TAIL_S", "1.0"))
# Bounded wait for BOTH pads to report contact after the ramp. It is the
# honesty gate, not padding -- see close_on_block(). Measured on the shipped
# configuration it is satisfied on the FIRST poll (0.00 s).
CLOSE_WAIT_S = float(os.environ.get("PICK_CLOSE_WAIT_S", "2.0"))
SQUEEZE_STATS = {"wait_s": None, "on_face": None}


def close_on_block(ramp_s=None):
    """ONE closing stage: ramp the position target from the preset clearance to
    the interference, then confirm by CONTACT before returning.

    ⚠ THIS USED TO BE THREE STAGES (preclose, squeeze, seat_pads) AND THE EXTRA
    TWO WERE A WORKAROUND FOR A BUG THAT IS NOW FIXED -- see THE FINGER CLOSING
    RATE in the module docstring. With the pads no longer parked on the table
    the jaws track this ramp exactly and the contact check below is satisfied on
    its FIRST poll, so the shuffling is gone and the motion reads as
    reach -> close -> lift.

    The check itself STAYS. The first version of this demo returned on a clock,
    printed "squeezed 10 mm interference" while the pads were still 22-38 mm
    clear, and its census then recorded ZERO pad contacts -- it passed on a
    grasp taken by accident during the lift. Waiting on contact makes the claim
    true by construction however fast or slow the jaws happen to be, so it is
    kept as the gate even though it now costs nothing.
    """
    ramp_s = CLOSE_S if ramp_s is None else ramp_s
    q0 = max(fingers())
    q1 = BLOCK_HALF - INTERFERENCE
    n = max(1, int(ramp_s * 1000 / dt))
    every = max(1, n // 8)
    for i in range(1, n + 1):
        q = q0 + (q1 - q0) * (i / float(n))
        fm.setPosition(q)
        mm.setPosition(q)
        if not _tick():
            return False
        if TRACE and (i % every == 0 or i == n):
            ql, qr = fingers()
            emit("[trace] %-9s t=%.3f q=(%.5f,%.5f) target=%.4f err=(%+.5f,%+.5f)"
                 % ("close", i * dt / 1000.0, ql, qr, q, q - ql, q - qr))
    t0 = robot.getTime()
    l = r = 0
    for _i in range(int(CLOSE_WAIT_S * 1000 / dt)):
        l, r = pads_on_block()
        if l > 0 and r > 0:
            break
        if not _tick():
            return False
    SQUEEZE_STATS["wait_s"] = round(robot.getTime() - t0, 3)
    SQUEEZE_STATS["on_face"] = [l, r]
    emit("[pick] closed to %.4f (%.0f mm interference, ~%.1f N/pad); pads on the "
         "block = (%d,%d) after %.2f s; q=(%.5f,%.5f)"
         % (q1, INTERFERENCE * 1000.0, GRIP_KP * INTERFERENCE, l, r,
            SQUEEZE_STATS["wait_s"], fingers()[0], fingers()[1]))
    return True


def pads_on_block():
    """(left, right) counts of contacts on the block's own +/-y GRASP faces.

    The cheap half of census(): same two-query attribution (the node_id on a
    ContactPoint names the QUERIED side, so the robot subtree query is the one
    that names a link), but it only counts, and only on the grasp faces.
    """
    bp, R = bpos(), borient()
    owner = {}
    for cp in omniarm6.getContactPoints(True):
        owner[tuple(round(v, 9) for v in cp.point)] = cp.node_id
    n = {"left": 0, "right": 0}
    for cp in bcontacts():
        d = [cp.point[i] - bp[i] for i in range(3)]
        loc = [d[0] * R[c] + d[1] * R[3 + c] + d[2] * R[6 + c] for c in range(3)]
        r = [abs(loc[i]) / BHALF[i] for i in range(3)]
        if r[1] < max(r) - 0.02:            # not flush with a +/-y face
            continue
        who = owner.get(tuple(round(v, 9) for v in cp.point))
        nm = _link_name(who) if who is not None else "world"
        if "left_finger" in nm:
            n["left"] += 1
        elif "right_finger" in nm:
            n["right"] += 1
    return n["left"], n["right"]


def release():
    open_fingers()
    step_for(1.0)


# ⚠ EVERY BLOCK READ GOES THROUGH THESE IN CLOTH MODE. A Cloth has no
# supervisor node, so block is None and any direct block.getX() is an
# AttributeError that kills the controller AFTER it has connected -- which the
# engine reports only as "controller exited with status: 1", with the sim
# still running and the cloth still swinging, i.e. it looks like a physics
# result and is not. Route every read through these instead.
def bpos():
    return list(block.getPosition()) if block is not None else list(CLOTH_GRASP)


def borient():
    return (block.getOrientation() if block is not None
            else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])


def bcontacts():
    return block.getContactPoints() if block is not None else []


def bz():
    if block is None:
        return float('nan')
    return bpos()[2]


def rel():
    if block is None:
        return (float('nan'),) * 3
    p, t = bpos(), tcp()
    return [p[i] - t[i] for i in range(3)]


_NAMES = {}


def _link_name(nid):
    if nid not in _NAMES:
        n, nm = robot.getFromId(nid), "?id%d" % nid
        if n is not None:
            f = n.getField("name")
            if f is not None:
                nm = f.getSFString()
        _NAMES[nid] = nm
    return _NAMES[nid]


def census(tag):
    """Every contact ON THE BLOCK, attributed to a robot link and a block face.

    ⚠ WHY THIS EXISTS. "The block is airborne and co-moving" proves the rig is
    holding it -- it does NOT prove the PADS are. The block is 90 mm tall and
    the pads are 50 mm, so 20.5 mm of block sticks up past the gripper's palm
    face; a hold could be a palm wedge with the fingers merely along for the
    ride. That failure mode is not hypothetical: the tree's CI-gated
    friction_grasp_minimal "holds" only because its part is ejected upward and
    lands on the gripper's wrist plate.

    ⚠ ATTRIBUTION IS A TWO-QUERY JOB. The node_id on a ContactPoint is the id on
    the QUERIED side, not the other body -- bcontacts() stamps the
    block's own id on every point (OmSupervisorUtilities::pushContactPointsToStream
    writes solid->uniqueId() when includeDescendants is false). The robot's
    subtree query is the one whose node_id names a LINK, so match the two lists
    by point. Both are built from the same native Newton contact vector in the
    same step, so the doubles are equal, not merely close.
    """
    bp, R = bpos(), borient()
    owner = {}
    for cp in omniarm6.getContactPoints(True):
        owner[tuple(round(v, 9) for v in cp.point)] = cp.node_id
        if DEBUG:
            emit("[raw robot] %-34s w=(%.4f,%.4f,%.4f) d=%.5f"
                 % (_link_name(cp.node_id), cp.point[0], cp.point[1],
                    cp.point[2], cp.depth))
    if DEBUG:
        emit("[raw block] bp=(%.4f,%.4f,%.4f) R=%s" % (bp[0], bp[1], bp[2],
                                                       [round(v, 3) for v in R]))
        for cp in bcontacts():
            emit("[raw block] id=%d w=(%.4f,%.4f,%.4f) d=%.5f"
                 % (cp.node_id, cp.point[0], cp.point[1], cp.point[2], cp.depth))
    pts = []
    for cp in bcontacts():
        d = [cp.point[i] - bp[i] for i in range(3)]
        # local = R^T * d  (getOrientation is row-major, local -> world)
        loc = [d[0] * R[c] + d[1] * R[3 + c] + d[2] * R[6 + c] for c in range(3)]
        # A contact on an edge or corner is at the extent of two or three axes
        # at once; naming only the largest ratio silently turns a bottom-face
        # table contact into an "x" contact. List every axis it is flush with.
        r = [abs(loc[i]) / BHALF[i] for i in range(3)]
        top = max(r)
        face = "".join("xyz"[i] + ("+" if loc[i] > 0 else "-")
                       for i in range(3) if r[i] > top - 0.02)
        who = owner.get(tuple(round(v, 9) for v in cp.point))
        pts.append({"link": _link_name(who) if who is not None else "world",
                    "face": face, "world_y": cp.point[1],
                    "local": [round(v, 5) for v in loc],
                    "depth": cp.depth})
    left = [p for p in pts if "left_finger" in p["link"]]
    right = [p for p in pts if "right_finger" in p["link"]]
    # ⚠ "PALM" MUST INCLUDE link6, AND FORGETTING THAT WOULD HAVE LEFT THIS
    # TEST TOOTHLESS A SECOND TIME. The gripper body's collision box is
    # declared on link6, not on robotiq_2f140_base_link, because the engine
    # drops a fixed-joint child's boundingObject (see the URDF). So a block
    # wedged against the gripper body reports as "link6", and a name test that
    # only looked for a "robotiq" prefix would file it under `other` and let
    # palm_wedge stay False -- exactly the failure mode the collider was added
    # to expose. Any gripper-body or wrist contact counts: it is not a pinch.
    PALM_LINKS = ("link6",)
    palm = [p for p in pts
            if (p["link"].startswith("robotiq") and "finger" not in p["link"])
            or p["link"] in PALM_LINKS]
    other = [p for p in pts
             if not p["link"].startswith("robotiq") and p["link"] not in PALM_LINKS]
    # (b) a pad contact only counts if it is on the block's GRASP FACE (|y| near
    # the half-width) -- a pad touching the block's top or bottom edge is not a
    # pinch, it is a shelf.
    # ⚠ WHERE THE PADS ACTUALLY ARE, read off the finger link Solids rather than
    # inferred from the joint sensors. This is the measurement that separates
    # "the mirrored prismatic joint is broken" from "the gripper is not centred
    # on the part": the two link ORIGINS are at gripper-local y = +(0.0095+q_l)
    # and -(0.0095+q_r) by construction, so their separation must equal
    # 0.019+q_l+q_r if the joints are honest, and the axis they straddle is the
    # thing the FK tool point claims to be on.
    ids = sorted({p_id for p_id, nm in _NAMES.items() if "finger" in nm})
    links = {}
    for nid in ids:
        n = robot.getFromId(nid)
        if n is not None:
            links[_NAMES[nid]] = list(n.getPosition())
    lf = [p for p in left if "y" in p["face"]]
    rf = [p for p in right if "y" in p["face"]]
    onface = lf + rf
    # ⚠ THE ASYMMETRY RULER. The pads' inner faces in WORLD y, straight off the
    # contact points -- no FK, no tool-frame model. Their midpoint vs the
    # block's own centre is how far off-centre the gripper closed on the part,
    # which is the quantity the joint readings only reflect indirectly.
    ly = sum(p["world_y"] for p in lf) / len(lf) if lf else None
    ry = sum(p["world_y"] for p in rf) / len(rf) if rf else None
    off = (0.5 * (ly + ry) - bp[1]) if (ly is not None and ry is not None) else None
    d = {"tag": tag, "n": len(pts), "left": len(left), "right": len(right),
         "palm": len(palm), "other": len(other), "pad_on_face": len(onface),
         "left_on_face": len(lf), "right_on_face": len(rf),
         "pad_face_y": [ly, ry], "block_y": bp[1],
         "finger_links": links, "q": list(fingers()), "tcp": list(tcp()),
         "offcentre_mm": round(off * 1000.0, 3) if off is not None else None,
         "max_depth_mm": round(max([p["depth"] for p in pts] + [0.0]) * 1000.0, 3),
         "pad_depth_mm": round(max([p["depth"] for p in onface] + [0.0]) * 1000.0, 3),
         "palm_depth_mm": round(max([p["depth"] for p in palm] + [0.0]) * 1000.0, 3),
         "points": pts}
    emit("[contact] %s n=%d L=%d(%d on-face) R=%d(%d on-face) palm=%d other=%d "
         "pad_depth=%.2fmm palm_depth=%.2fmm offcentre=%s"
         % (tag, d["n"], d["left"], d["left_on_face"], d["right"],
            d["right_on_face"], d["palm"], d["other"], d["pad_depth_mm"],
            d["palm_depth_mm"],
            "n/a" if d["offcentre_mm"] is None else "%+.2fmm" % d["offcentre_mm"]))
    for p in pts:
        emit("[contact]   %-34s face=%s local=%s depth=%.4fmm"
             % (p["link"], p["face"], p["local"], p["depth"] * 1000.0))
    return d


PX, PY = 0.46, 0.0
# WORLD z THE PAD MID-PLANE MUST SIT AT -- not the FK tool point, and not the
# block centre. The aim loop below drives grip_point()[2] here, so this number
# is a real height on the real pads and the two clearances it has to respect are
# arithmetic rather than hope:
#   pad far end  = 0.251 - 0.03275 = 0.2183  ->  18.3 mm above the table (0.200)
#   pad near end = 0.251 + 0.03275 = 0.2838  ->   6.2 mm below the block top
# ⚠ 0.245 (the block centre) IS NOT SAFE AND WAS THE BUG. It leaves only 12.3 mm
# of nominal pad-to-table clearance, and the previous revision spent all of it
# and 0.3 mm more on the FK tool point's own error, parking the pads ON the
# table. Anything below ~0.247 puts a 65.5 mm pad back into the table top.
# Mid-sheet in cloth mode: high enough that the pad (which extends below the
# tool point) is fully on fabric, low enough that the hem hangs free below.
GRASP_Z = float(os.environ.get("PICK_GRASP_Z",
                               "0.350" if block is None else "0.251"))
PLACE_X, PLACE_Y = 0.24, -0.42
# One-shot lateral aim bias, in metres, folded into the SAME goto calls the
# demo already makes -- so it changes where the gripper ends up without
# changing how many moves it took to get there.
YBIAS = float(os.environ.get("PICK_YBIAS", "0"))
AX, AY = PX, PY + YBIAS

DIAG = os.environ.get("PICK_DIAG") == "1"


def _rate(tag, target, secs, tick=True):
    """Command both fingers to `target`, step `secs`, report the mm/s achieved."""
    for _m in (fm, mm):
        _m.setPosition(target)
    q0 = fingers()
    t0 = robot.getTime()
    for _i in range(int(secs * 1000 / dt)):
        if robot.step(dt) == -1:
            return
        if tick:
            bridge.tick(robot.getTime())
    q1 = fingers()
    el = robot.getTime() - t0
    emit("[diag] t=%6.2f %-22s tick=%d target=%.4f  q %.5f,%.5f -> %.5f,%.5f  "
         "rate=(%+.2f,%+.2f) mm/s in %.2fs"
         % (robot.getTime(), tag, tick, target, q0[0], q0[1], q1[0], q1[1],
            (q1[0] - q0[0]) * 1000.0 / el, (q1[1] - q0[1]) * 1000.0 / el, el))


if DIAG:
    # ── FULL-STROKE SWEEP AT THE SPAWN POSE ──────────────────────────────
    # The regression guard for the whole-gripper colliders: four boxes per jaw
    # instead of one is exactly the change that AGENTS.md warns can snag and
    # lock a mechanism, so drive the jaws end to end and assert they reach both
    # URDF limits. Also the free-space baseline for the rate battery further
    # down -- the pair of numbers is what makes the grasp-pose reading mean
    # anything.
    for _tgt in (-0.030, 0.070, -0.030, 0.070):
        _rate("stroke -> %+.3f" % _tgt, _tgt, 2.0, tick=False)
    _ql, _qr = fingers()
    emit("[diag] STROKE %s (both jaws must reach 0.070 open and -0.030 closed)"
         % ("OK" if abs(_ql - 0.070) < 1e-3 and abs(_qr - 0.070) < 1e-3
            else "FAILED to reopen: q=(%.5f,%.5f)" % (_ql, _qr)))

if os.environ.get("PICK_DIAG") == "2":
    # ── WHICH JOINT? Drive the ARM motors directly (no bridge, no IK) and
    # measure the finger rate at each configuration.
    GRASP_Q = [0.0153, 0.1925, 1.4051, -0.0038, 1.635, -3.1202]

    def _armto(q, settle=3.0):
        for _m, _v in zip(bridge.motors, q):
            if _m is not None:
                _m.setPosition(_v)
        for _ in range(int(settle * 1000 / dt)):
            robot.step(dt)

    def _leg(tag, q):
        _armto(q)
        cps = omniarm6.getContactPoints(True)
        emit("[diag] --- %s arm=%s robot_contacts=%d"
             % (tag, [round(v, 3) for v in q], len(cps)))
        for _cp in cps:
            emit("[diag]      %-34s w=(%.3f,%.3f,%.3f) depth=%.3fmm"
                 % (_link_name(_cp.node_id), _cp.point[0], _cp.point[1],
                    _cp.point[2], _cp.depth * 1000.0))
        _rate(tag + " close", 0.030, 2.0, tick=False)
        _rate(tag + " open ", 0.060, 2.0, tick=False)

    Z = [0.0] * 6
    _leg("zero    ", Z)
    _leg("grasp   ", GRASP_Q)
    _leg("zero-2  ", Z)
    for _i in range(6):
        _q = list(Z)
        _q[_i] = GRASP_Q[_i]
        _leg("j%d only " % (_i + 1), _q)
    # cumulative: add one joint at a time
    for _i in range(6):
        _q = GRASP_Q[:_i + 1] + [0.0] * (5 - _i)
        _leg("j1..%d   " % (_i + 1), _q)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump({"diag": 2, "log": log}, fh, indent=1)
    robot.simulationQuit(0)
    while robot.step(dt) != -1:
        pass

open_fingers()
step_for(1.6)   # 70 mm of stroke at the URDF's 0.07 m/s velocity cap
emit("[pick] start block_z=%.4f fingers=%s control_drop=%s ybias=%.4f"
     % (bz(), fingers(), CONTROL_DROP, YBIAS))
# What the MOTOR layer thinks its limits are, as opposed to what the URDF
# declared and what the compiled mjModel got. The three can disagree, and when
# they do the symptom is a finger that tracks its target far slower than the
# declared velocity -- which is exactly what the first run of this demo did.
for _nm, _m in (("finger", fm), ("mirror", mm)):
    emit("[pick] motor %-6s maxForce=%.3f available=%.3f velocity=%.4f maxVel=%.4f"
         % (_nm, _m.getMaxForce(), _m.getAvailableForce(), _m.getVelocity(),
            _m.getMaxVelocity()))

# ⚠ PRE-SET THE JAWS NOW, BEFORE THE ARM MOVES. Not cosmetic sequencing: the
# fingers close at 156 mm/s here and at ~1.5 mm/s once the arm is parked over
# the block (both measured -- see the module docstring). Doing the 27 mm of
# bulk travel here is what makes the squeeze at the grasp pose a 7 mm move
# instead of a 34 mm one. It is also how a real gripper is driven: you
# pre-position the jaws to the part width on approach, not at the part.
if not CONTROL_DROP:
    for m in (fm, mm):
        m.setPosition(BLOCK_HALF + PRESET_CLEAR)
    step_for(1.2)
    emit("[pick] preset jaws -> %.4f,%.4f (target %.4f)"
         % (fingers()[0], fingers()[1], BLOCK_HALF + PRESET_CLEAR))

if os.environ.get("PICK_FINGER_PROBE") == "1":
    # ISOLATED FINGER PROBE, before the arm has moved at all: command a 40 mm
    # close into free space and time it. This is the control that separates
    # "the finger servo is intrinsically slow" from "something about the arm
    # pose or motion is holding it".
    for _m in (fm, mm):
        _m.setPosition(0.030)
    t0 = robot.getTime()
    for i in range(1, 1501):
        if robot.step(dt) == -1:
            break
        if i % 125 == 0:
            ql, qr = fingers()
            emit("[probe] t=%.2f q=(%.5f,%.5f) target=0.0300" % (robot.getTime() - t0, ql, qr))
        if max(abs(fingers()[0] - 0.030), abs(fingers()[1] - 0.030)) < 2e-4:
            emit("[probe] REACHED 0.030 at t=%.3f s" % (robot.getTime() - t0))
            break
    emit("[probe] done q=%s" % (fingers(),))
    open_fingers()
    step_for(2.0)

# ═══════════════════════════════════════════════════════════════════════
# THE DESCENT IS ONE MOVE. IT IS CALIBRATED UP HERE, NOT DOWN THERE.
# ═══════════════════════════════════════════════════════════════════════
# ⚠ USER-REPORTED DEFECT: "when it goes down, it moves a little bit around
# until it picks the cube". That was not a physics problem, it was this
# controller's aiming strategy showing on screen. The previous revision reached
# the grasp pose with SEVEN commanded moves (measured, ledger printed on every
# run): goto((AX,AY,GRASP_Z)) took up to three closed-loop passes on the FK
# tool point, and the pad-mid-plane re-aim below it took up to four more --
# every one of them a separate little correction at the block, which is exactly
# what a person watching reads as the arm hunting.
#
# THE FIX IS TO MOVE THE MEASUREMENT, NOT TO DELETE IT. Both error terms the
# corrections were chasing are CONSTANTS, so neither needs re-measuring at the
# target:
#   * the FK tool point sits a constant ~5.5 mm out in y and ~8.4 mm out in z
#     from the true pad mid-plane (5.66 / 5.73 / 5.45 mm at three unrelated arm
#     poses -- the measurement that closed the "7 mm stall asymmetry");
#   * the DLS solver returns as soon as pos_err < ik["tol"], which is 5e-3 for
#     the OMNIARM6 -- a 5 mm licence to stop early, on top of the servo's own
#     tracking residual. That is the other half of the 1.5-2.4 cm open-loop
#     error, and it is fixed rather than measured around: IK_TIGHT below drops
#     the tolerance to 1e-5 and raises the iteration budget.
# So: converge the aim at the APPROACH height, where a correction is 160 mm
# above the part and invisible; then command ONE goal, translated straight down
# by the exact APPROACH_DZ, and assert the achieved pad plane afterwards.
#
# ⚠ AND IT MUST ASSERT, NOT RETRY. If the single descent lands outside
# DESCENT_TOL the run says so and FAILS -- a demo that recovers by shuffling is
# the defect, so the failure has to be loud instead of absorbed.
APPROACH_DZ = 0.16
# Stand-off along +x for the cloth approach: far enough that the OPEN jaw
# (2F-140 fingers plus stroke) is clear of the sheet's +x edge at 0.51.
APPROACH_DX = float(os.environ.get("PICK_APPROACH_DX", "0.17"))
APPROACH_Z = GRASP_Z + APPROACH_DZ
# A copy of the arm's IK config with the early-out disarmed. `tol` 5e-3 is a
# sane default for a chat arm asked to wave; it is 5 mm of slop in a grasp
# whose pad clearance is 18 mm.
IK_TIGHT = dict(IK, tol=1e-5, pose_max_iters=400)
DESCENT_TOL = float(os.environ.get("PICK_DESCENT_TOL", "0.0015"))

# ── THE CALIBRATION IS A CONSTANT, SO IT IS BAKED, NOT PERFORMED ───────
# ⚠ THE CALIBRATE STAGE IS GONE (2026-08-11), AND ITS OWN EVIDENCE IS WHY.
# It used to fly the arm to an approach height 160 mm above the block and run
# a convergence loop there -- up to four commanded moves -- to discover the
# offset between the FK tool point and the real pad mid-plane. A viewer reads
# that as the arm fussing before it commits, which is exactly what it is.
#
# But every revision of this file has recorded that the quantity it discovers
# is a CONSTANT: "-1.03, +5.70, +8.10 mm, reproducible run to run", and
# "5.66 / 5.73 / 5.45 mm in y at three unrelated arm poses". Measuring a
# constant on every run, in front of the audience, buys nothing. So it is
# measured ONCE, written down here, and asserted rather than rediscovered.
#
# What is NOT constant -- gravity sag (0.46 mm at the approach pose, 12.32 mm
# at the grasp pose) and DH model error (~3.5 mm over the descent) -- was never
# what the calibrate stage fixed anyway. descend_once() servos those out IN
# FLIGHT on grip_point(), inside the single commanded motion, and lands
# 0.008 / 0.003 / 0.276 mm. That mechanism does all the work now.
#
# It is re-verified on EVERY run anyway, for free and without any motion, once
# the arm reaches the approach pose -- see the calib CHECK below. If that drift
# ever grows, fix this constant; do not restore the stage.
CALIB_BAKED = [-0.001034, 0.005703, 0.008104]   # metres, measured 2026-08-11
CALIB = list(CALIB_BAKED)
DESCENT_AIM["calib_mm"] = [round(v * 1000.0, 3) for v in CALIB]
if os.environ.get("PICK_AIM", "grip") == "fk":
    CALIB = [0.0, 0.0, 0.0]
    DESCENT_AIM["calib_mm"] = [0.0, 0.0, 0.0]
emit("[pick] calib: BAKED goal-minus-pad-plane offset = %s mm (constant; "
     "re-checked against a live measurement at the reach pose, below)"
     % (DESCENT_AIM["calib_mm"],))


def goal_for(pad_xyz, yaw=0.0):
    """The goal to COMMAND so the real pads end up at `pad_xyz`.

    ⚠ `yaw` IS NOT OPTIONAL DECORATION ONCE THE TOOL RIDES ROUND WITH THE BASE.
    CALIB is the FK-tool-point-to-pad-plane offset measured in WORLD axes with
    the tool at TOP; it is a fixed vector in the TOOL frame, so as soon as the
    carry yaws the tool by dpsi the world vector yaws with it. Because TOP is
    its own inverse, "rotate the offset into the new tool frame" reduces to
    Rz(dpsi) on the world vector. Forgetting this left the place descent
    correcting 12.1 mm in x and 10.5 mm in y in flight, and arriving 4.6 mm
    high."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [pad_xyz[0] + CALIB[0] * c - CALIB[1] * s,
            pad_xyz[1] + CALIB[0] * s + CALIB[1] * c,
            pad_xyz[2] + CALIB[2]]


# ── TWO MOVES TO THE PART: TRANSIT, THEN DROP. NO FUSSING IN BETWEEN. ──
# The pick used to take FOUR commanded moves: a two-pass approach, up to four
# calibrate nudges at the approach height, then the descent. Three of those
# four are gone -- the calibration is a baked constant now, and the correction
# passes are off -- so what is left is a transit and a drop, back to back.
#
# ⚠⚠ AND IT IS NOT ONE MOVE, BECAUSE ONE MOVE WAS TRIED AND IT CLIPPED THE
# PART. An earlier revision of this block reached from the spawn pose straight
# to the grasp pose in a single ramp, on the argument that the waypoint was
# only ever a staging convenience and that descend_once's in-flight corrector
# would deliver the accuracy wherever the move started. Both halves were wrong,
# and both showed up as numbers on the first run:
#
#   * ACCURACY. The corrector integrates the pad-plane error, and error only
#     means something once the ramp has essentially stopped. Armed at 70% of a
#     full spawn->grasp transit it integrated the transit itself: in-flight
#     correction -312.63 / -4.30 / -237.17 mm, pad plane 69 mm out. Re-arming
#     it at 92% fixed the windup (-31 / -17 / -115 mm) and still missed by
#     77.8 mm, because by then the second problem had already happened:
#   * COLLISION. Sweeping the open jaws diagonally at the part instead of
#     dropping onto it from above put a pad through the block: it moved
#     55.6 / 73.9 / -16.5 mm before the gripper ever closed, 4 robot contacts
#     were live at the close, and the run ended carrying nothing (max relative
#     rotation 179.7 deg, placed=False).
#
# The 160 mm waypoint is what prevents that, and it is the ONLY thing that
# does -- there is no perception in this loop and no reactive stop. It stays.
# `reach_nudge` below is kept as the standing assertion, because "the approach
# does not touch the part" is now a claim this demo makes rather than a
# property of its staging.
phase("reach")
_bp = bpos() if block is not None else list(CLOTH_GRASP)
PAD_TARGET = [_bp[0], _bp[1], GRASP_Z]
BLOCK_BEFORE = list(_bp)
# passes=1: a second pass here is a visible twitch 160 mm above the part, and
# it corrects nothing the descent's own in-flight servo will not.
# ⚠ LATERAL APPROACH IN CLOTH MODE, AND THE REASON IS GEOMETRIC, NOT STYLISTIC.
# The rigid demo descends a straight column onto a block that has clear air
# above it. A HANGING cloth does not: whatever holds it up is directly over the
# grasp point, and so is the upper half of the sheet. Descending drives the jaw
# through both -- observed in the GUI as the gripper passing through the rail.
# The sheet is a vertical plane at y = 0 and the pads separate along y, so
# coming in along +x at pinch height lets the OPEN jaw straddle the fabric with
# nothing crossing its path.
if CLOTH_MODE:
    _reach_goal = goal_for([PAD_TARGET[0] + APPROACH_DX, PAD_TARGET[1], PAD_TARGET[2]])
else:
    _reach_goal = goal_for([PAD_TARGET[0], PAD_TARGET[1], APPROACH_Z])
emit("[pick] reach    err=%.4f"
     % goto(_reach_goal, REACH_S, passes=1, seed="near", ik=IK_TIGHT))

# ── THE BAKED CONSTANT IS RE-CHECKED HERE, FOR FREE, ON EVERY RUN ──────
# ⚠ A BAKED NUMBER THAT NOTHING VERIFIES IS A LANDMINE. The calibrate stage
# was deleted because it re-measured a constant in front of the audience -- but
# "it is a constant" is an empirical claim about this gripper, this URDF and
# this IK chain, and any of the three can change under it. So measure it again
# right here and compare: the arm is already standing at the approach pose, so
# CALIB is just (what we commanded) minus (where the pads actually are), and
# reading it costs NO MOTION AT ALL. That is the whole trick -- the old stage's
# expense was never the measurement, it was the corrective moves it made after
# each one.
_g = grip_point()
if _g is not None:
    # ⚠ MEASURE (FK TOOL POINT - PAD PLANE), NOT (COMMANDED GOAL - PAD PLANE).
    # The first version of this check used the commanded goal and reported a
    # 12 mm drift on a constant that had not moved at all: with passes=1 the
    # reach deliberately stops ~12 mm short of its commanded goal, and that
    # tracking residual landed straight in the "constant". The old calibrate
    # loop was immune only because it ITERATED until commanded and actual
    # agreed -- which is the expense we just removed. Both terms here are
    # derived from the arm's ACTUAL measured joints, so how far the reach fell
    # short cannot contaminate the reading. This is the same quantity the old
    # stage printed as "FK tool point is +x,+y,+z mm from it".
    CALIB_NOW = [tcp()[k] - _g[k] for k in range(3)]
    CALIB_DRIFT = [round((CALIB_NOW[k] - CALIB[k]) * 1000.0, 3) for k in range(3)]
    DESCENT_AIM["calib_now_mm"] = [round(v * 1000.0, 3) for v in CALIB_NOW]
    DESCENT_AIM["calib_drift_mm"] = CALIB_DRIFT
    _cd = max(abs(v) for v in CALIB_DRIFT)
    CALIB_DRIFT_TOL_MM = float(os.environ.get("PICK_CALIB_DRIFT_TOL_MM", "3.0"))
    emit("[pick] calib CHECK: measured %s mm vs baked %s mm -> drift %s mm "
         "(worst %.3f, tol %.2f) %s"
         % (DESCENT_AIM["calib_now_mm"], DESCENT_AIM["calib_mm"], CALIB_DRIFT,
            _cd, CALIB_DRIFT_TOL_MM,
            "OK" if _cd <= CALIB_DRIFT_TOL_MM else
            "-- ⚠ THE CONSTANT HAS MOVED. Update CALIB_BAKED from the measured "
            "value above; the descent servo is absorbing the difference and "
            "will run out of authority if it grows."))
else:
    CALIB_DRIFT = None

phase("descend")
gx, gy, gz = goal_for(PAD_TARGET)
descend_once(PAD_TARGET)

# ── THE ASSERTION THAT REPLACES THE SHUFFLE ────────────────────────────
_g = grip_point()
if _g is not None:
    _bp = bpos() if block is not None else list(CLOTH_GRASP)
    _e = [_bp[0] - _g[0], _bp[1] - _g[1], GRASP_Z - _g[2]]
    DESCENT_AIM["err_mm"] = [round(v * 1000.0, 3) for v in _e]
    _worst = max(abs(v) for v in _e)
    emit("[pick] descend: achieved pad plane is %s mm from target (worst %.2f "
         "mm, tol %.2f mm) -> %s"
         % (DESCENT_AIM["err_mm"], _worst * 1000.0, DESCENT_TOL * 1000.0,
            "OK" if _worst <= DESCENT_TOL else "OUT OF TOLERANCE"))
    if _worst > DESCENT_TOL:
        emit("[pick] ⚠⚠ DESCENT OUT OF TOLERANCE. NOT re-aiming -- corrective "
             "motion at the part is the defect this replaced. Fix the "
             "calibration or the sag corrector, do not add a pass.")

# The pads' far end vs the table it is standing over. This is the number that
# was silently negative for the whole life of the previous revision, so it is
# printed on every run rather than reasoned about.
_g = grip_point()
if _g is not None:
    emit("[pick] pad clearance over the pick table: %.1f mm (pad far end z=%.4f, "
         "table top z=%.3f)" % ((_g[2] - PAD_HALF_L - TABLE_TOP) * 1000.0,
                                _g[2] - PAD_HALF_L, TABLE_TOP))
emit("[pick] robot contacts before closing: %d (must be 0 -- a pad resting on "
     "the table cannot slide inward at mu %s)"
     % (len(omniarm6.getContactPoints(True)), "6"))

# ── DID THE ONE-MOVE REACH TOUCH THE PART ON THE WAY IN? ───────────────
# ⚠ THIS ASSERTION IS THE PRICE OF DELETING THE APPROACH WAYPOINT, and it
# cannot be skipped in favour of the descent aim error, which is measured
# against the block's CURRENT position -- a block nudged by an incoming pad
# drags the target along with it and reports a perfect aim. Compare against
# where the block was BEFORE the reach started.
_bp_now = bpos()
REACH_NUDGE = [round((_bp_now[k] - BLOCK_BEFORE[k]) * 1000.0, 3)
               for k in range(3)]
_nudge_worst = max(abs(v) for v in REACH_NUDGE)
REACH_NUDGE_TOL_MM = float(os.environ.get("PICK_REACH_NUDGE_TOL_MM", "1.0"))
reach_clean = _nudge_worst <= REACH_NUDGE_TOL_MM
emit("[pick] reach: block moved %s mm during the approach (worst %.3f, tol "
     "%.2f) -> %s" % (REACH_NUDGE, _nudge_worst, REACH_NUDGE_TOL_MM,
                      "UNTOUCHED" if reach_clean else "THE PADS CLIPPED IT"))
if not reach_clean:
    emit("[pick] ⚠⚠ the single-move reach disturbed the part before the grasp. "
         "That is the failure mode the 160 mm approach waypoint used to "
         "prevent. Raise PICK_REACH_S, or restore a waypoint -- do NOT widen "
         "the tolerance.")

# Everything from t=0 to here commanded the SAME tool orientation, so this
# number should be small. It is the pre-pick counterpart of the carry's
# relative-rotation ruler, and it exists because a user could see something
# that no number in this run could.
PREPICK = wind_report("to-grasp")

if DIAG:
    # ── RATE-COLLAPSE DIAGNOSTIC BATTERY, AT THE GRASP POSE ──────────────
    # ⚠ THIS IS THE INSTRUMENT THAT FOUND THE BUG. Keep it. It is differential:
    # every leg commands the same finger move and changes exactly one thing, and
    # `robot_contacts` is the column that mattered -- the rate tracked it and
    # nothing else. Four earlier hypotheses (gains, motor velocity, target
    # staleness, the compiled model) were all about the ACTUATOR and all four
    # were refuted; the answer was in the environment.
    emit("[diag] arm q = %s" % [round(v, 4) for v in bridge._read_q()])
    for _cp in omniarm6.getContactPoints(True):
        emit("[diag]      %-34s w=(%.3f,%.3f,%.3f) depth=%.3fmm"
             % (_link_name(_cp.node_id), _cp.point[0], _cp.point[1],
                _cp.point[2], _cp.depth * 1000.0))
    _rate("A close, tick ON", 0.030, 3.0, tick=True)
    _rate("B open,  tick ON", 0.060, 3.0, tick=True)
    _rate("C close, tick OFF", 0.030, 3.0, tick=False)
    _rate("D open,  tick OFF", 0.060, 3.0, tick=False)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump({"diag": True, "log": log}, fh, indent=1)
    robot.simulationQuit(0)
    while robot.step(dt) != -1:
        pass


def pads():
    """Where each pad's INNER FACE sits on the grasp axis, vs the block's own
    faces. The joint readings alone are not interpretable: the two joints are
    mirrored (axes +y and -y), so equal readings mean symmetric pads, and a
    reading that grows means that pad moved OUTWARD.
    """
    ql, qr = fingers()
    r = rel()
    return ("L_face=%+.4f R_face=%+.4f | block_y=%+.4f faces=[%+.4f,%+.4f] "
            "bite_L=%+.1fmm bite_R=%+.1fmm"
            % (ql, -qr, r[1], r[1] - BLOCK_HALF, r[1] + BLOCK_HALF,
               ((r[1] + BLOCK_HALF) - ql) * 1000.0,
               (qr - (BLOCK_HALF - r[1])) * 1000.0 * -1.0))


if CONTROL_DROP:
    # ⚠ THE NEGATIVE CONTROL IS NOW THE SAME MOTION BY CONSTRUCTION, not by
    # bookkeeping. The previous revision had to run its seating legs in BOTH
    # arms so that "the control left the block behind" stayed a claim about the
    # GRIP rather than about the trajectory. With seat_pads() gone there is no
    # grip-only motion left: the control is this demo with the jaws left open.
    emit("[pick] CONTROL: jaws left %.0f mm apart, no squeeze -- the block must "
         "be left behind" % (2 * fingers()[0] * 1000.0))
else:
    phase("close")
    close_on_block()

# ⚠ LATCH THE ORIENTATION REFERENCE HERE, at the instant the jaws have the
# block and before the arm moves again. Everything after this is measured
# against "rigid with the gripper". Not armed in the negative control: there
# the block is never held, so "how far did it turn inside the jaws" has no
# referent and a large reading would mean nothing.
if not CONTROL_DROP:
    rot_arm("closed")
emit("[pick] pads after closing: %s" % pads())
c_squeeze = census("closed")
rel0 = rel()
phase("lift")
rot_phase("lift")
# passes=1 on all three free-space moves below. A second pass is a separate
# 0.5 s commanded ramp -- a visible twitch at the end of a move -- and it buys
# nothing here: the lift and carry only have to put the tool ABOVE the place
# point, and the place accuracy is delivered by the `lower` descent_once, which
# servos the real pad plane in flight exactly as the pick does.
emit("[pick] lift     err=%.4f" % goto((gx, gy, gz + 0.22), 2.0, passes=1,
                                       seed="near", ik=IK_TIGHT))
emit("[pick] lifted  block_z=%.4f  %s" % (bz(), pads()))
rot_report("lifted")
c_lift = census("lifted")
# ── THE CARRY, AND THE WRIST YAW THAT WAS TURNING THE BLOCK ────────────
# ⚠ MEASURED. Commanding the SAME world tool orientation at the pick and at
# the place is not free: the two stand 60.3 deg apart in azimuth, so joint1
# sweeps -1.064 rad on the way over and joint6 must sweep +1.064 rad IN WORLD
# to cancel it. The gripper's world orientation stays put -- and the block,
# which is held only by two 22 mm-wide pads, does not: it yawed 13.46 deg
# about the TOOL AXIS mid-carry and came back, on the run before this comment
# existed.
# The rotation is proportional to that counter-rotation RATE, not to the angle
# travelled and not to acceleration: it peaks at peak wrist speed, and a carry
# stretched 3x (3.0 s -> 9.0 s) took it 13.46 -> 4.31 deg, i.e. tau*omega with
# tau = 0.43 s on both runs. It is NOT a solver artifact -- newtonCondim 6 and
# newtonIterations 400/newtonLsIterations 100 each reproduced 13.459 deg
# BIT-IDENTICALLY.
# ⚠ AND IT IS THE GRASP'S WEAKEST AXIS BY CONSTRUCTION. The yaw the block does
# is about the tool axis, which lies IN the pad plane -- resisted only by how
# far the contact patch spreads ACROSS the pad, and the 2F-140's pad is 22 mm
# wide against 65.5 mm long. The measured patch is +/-11 mm in x against
# +/-33 mm in z, so this DOF is ~3x softer than the tilt DOF and ~0 stiffer
# than nothing. Torsional friction cannot help either: that resists rotation
# about the pad NORMAL, which is a different axis (this is why condim 6 does
# nothing here).
# THE FIX IS TO STOP DEMANDING THE COUNTER-ROTATION. A real arm transferring a
# part between two azimuths does not spin its wrist 60 deg to hold the tool at
# a fixed compass bearing -- it swings the base and lets the tool ride round.
# The carry therefore targets TOP rotated about world z by the azimuth the
# base is about to travel, so joint6 stays put and the excitation is gone.
# The block arrives yawed by that same angle, which is correct: it was carried
# round, not teleported. PICK_CARRY_YAW=0 restores the old fixed-bearing carry.
CARRY_R, CARRY_YAW = TOP, 0.0
if os.environ.get("PICK_CARRY_YAW", "1") == "1":
    _bp = bpos()
    _dpsi = math.atan2(PLACE_Y, PLACE_X) - math.atan2(_bp[1], _bp[0])
    _c, _s = math.cos(_dpsi), math.sin(_dpsi)
    # Rz(dpsi) * TOP, with TOP = [[1,0,0],[0,-1,0],[0,0,-1]]
    CARRY_R, CARRY_YAW = [[_c, _s, 0.0], [_s, -_c, 0.0], [0.0, 0.0, -1.0]], _dpsi
    emit("[pick] carry: letting the tool ride round with the base, %.1f deg of "
         "azimuth -- the wrist holds still instead of counter-rotating"
         % math.degrees(_dpsi))
phase("carry")
rot_phase("carry")
emit("[pick] carry    err=%.4f"
     % goto(goal_for([PLACE_X, PLACE_Y, GRASP_Z + 0.22], CARRY_YAW), CARRY_S,
            passes=1, seed="near", ik=IK_TIGHT, R=CARRY_R))
emit("[pick] carried block_z=%.4f  %s" % (bz(), pads()))
rot_report("carried")
c_carry = census("carried")

carried_ok = bz() > TABLE_TOP + 0.06

drift = max(abs(rel()[i] - rel0[i]) for i in range(3))

# ⚠ THE GRASP ASSERTION. Airborne + co-moving says the RIG holds the block; only
# this says the PADS do. Both pads must be on the block's own grasp faces (|y| ~
# BLOCK_HALF) while it is off the table -- one-sided contact is a shelf, not a
# pinch, and palm contact means the 90 mm block is partly wedged into the
# gripper body rather than pinched between the 65.5 mm pads. ⚠ THIS TEST HAS
# TEETH NOW AND DID NOT BEFORE: until this revision the gripper body had NO
# collider at all (the fixed-joint merge dropped it), so palm_wedge was
# structurally False whatever happened. The body box is now on link6 and a
# real wedge would be reported.
pinched = (c_lift["left_on_face"] > 0 and c_lift["right_on_face"] > 0
           and c_carry["left_on_face"] > 0 and c_carry["right_on_face"] > 0)
palm_wedge = c_lift["palm"] > 0 or c_carry["palm"] > 0
if palm_wedge:
    emit("[contact] ⚠⚠ PALM CONTACT DURING CARRY -- the hold is at least partly a "
         "wedge against the gripper body, NOT a two-finger pinch "
         "(lift palm=%d %.2fmm, carry palm=%d %.2fmm)"
         % (c_lift["palm"], c_lift["palm_depth_mm"],
            c_carry["palm"], c_carry["palm_depth_mm"]))
emit("[contact] VERDICT pinched=%s palm_wedge=%s max_pad_penetration=%.2fmm"
     % (pinched, palm_wedge,
        max(c_squeeze["pad_depth_mm"], c_lift["pad_depth_mm"],
            c_carry["pad_depth_mm"])))

# Back to the SAME pad-mid height the grasp was taken at, so the block
# returns to the table at the height it left it, plus a 3 mm drop.
phase("lower")
rot_phase("lower")
descend_once([PLACE_X, PLACE_Y, GRASP_Z + 0.003], dur=LOWER_S, tag="lower",
             R=CARRY_R, yaw=CARRY_YAW)
_g = grip_point()
if _g is not None:
    emit("[pick] lower: pad plane %+.2f,%+.2f,%+.2f mm from the place target"
         % ((PLACE_X - _g[0]) * 1000.0, (PLACE_Y - _g[1]) * 1000.0,
            (GRASP_Z + 0.003 - _g[2]) * 1000.0))
rot_report("lowered")
if not CONTROL_DROP:
    release()
# ⚠ STOP MEASURING AT THE RELEASE, not after. Once the jaws open the block is
# free and any rotation it does is the placement settling, not the carry.
rot_max_deg = ROT["max_deg"] if ROT["ref"] is not None else None
rot_axis = ROT["max_axis"]
rot_phases = {k: round(v, 3) for k, v in ROT["per_phase"].items()}
ROT["ref"] = None
phase("retreat")
emit("[pick] retreat  err=%.4f"
     % goto(goal_for([PLACE_X, PLACE_Y, GRASP_Z + 0.20], CARRY_YAW), 1.6,
            passes=1, seed="near", ik=IK_TIGHT, R=CARRY_R))
step_for(1.5)

p = bpos()
placed = (math.hypot(p[0] - PLACE_X, p[1] - PLACE_Y) < 0.10
          and p[2] > TABLE_TOP + 0.02)

# ── THE TWO NEW ACCEPTANCE NUMBERS ──────────────────────────────────────
# (1) DESCENT SEGMENTS. One commanded move from the approach height to the
#     grasp pose, or the motion reads as the arm hunting for the block.
# (2) MAX RELATIVE ROTATION. A rigid carry is ~1 deg; anything more is the
#     block turning inside the jaws, which `drift` cannot see.
descent_segments = seg_count("descend")
DESCENT_SEG_MAX = int(os.environ.get("PICK_DESCENT_SEG_MAX", "1"))
ROT_TOL_DEG = float(os.environ.get("PICK_ROT_TOL_DEG", "1.0"))
_de = DESCENT_AIM.get("err_mm")
descent_ok = (descent_segments <= DESCENT_SEG_MAX
              and _de is not None
              and max(abs(v) for v in _de) <= DESCENT_TOL * 1000.0)
rigid = (rot_max_deg is not None and rot_max_deg <= ROT_TOL_DEG)
emit("[pick] DESCENT segments=%d (limit %d) aim_err=%s mm (tol %.2f) -> %s   "
     "[ledger %s]"
     % (descent_segments, DESCENT_SEG_MAX, _de, DESCENT_TOL * 1000.0,
        "OK" if descent_ok else "SHUFFLE/OUT-OF-TOLERANCE",
        [(m["phase"], m["t"]) for m in MOVES]))
emit("[pick] ROTATION max relative block-in-gripper rotation = %s deg "
     "(tol %.2f) axis_tool=%s per-phase=%s -> %s"
     % ("n/a" if rot_max_deg is None else "%.2f" % rot_max_deg, ROT_TOL_DEG,
        rot_axis, rot_phases,
        "n/a" if rot_max_deg is None else ("RIGID" if rigid else "SLIPPING")))

# ⚠ reach_clean JOINS THE VERDICT. Deleting the approach waypoint moved a
# failure mode from "impossible by staging" to "prevented by timing", and a
# demo may not ship a new failure mode that only a human watching would catch
# -- that is the carry-vs-tray lesson, and the rotation lesson, a third time.
ok = ((not carried_ok and not placed) if CONTROL_DROP
      else (carried_ok and placed and pinched and rigid and descent_ok
            and reach_clean))

emit("[pick] RESULT carried=%s pinched=%s palm_wedge=%s drift=%.4fm "
     "max_rel_rot=%s deg descent_segments=%d placed=%s final=(%.3f,%.3f,%.3f)"
     " -> %s"
     % (carried_ok, pinched, palm_wedge, drift,
        "n/a" if rot_max_deg is None else "%.2f" % rot_max_deg,
        descent_segments, placed, p[0], p[1], p[2], "PASS" if ok else "FAIL"))

with open(OUT, "w", encoding="utf-8") as fh:
    json.dump({"control_drop": CONTROL_DROP, "carried": carried_ok,
               "pinched": pinched, "palm_wedge": palm_wedge,
               "contacts": [c_squeeze, c_lift, c_carry],
               "drift_m": drift, "placed": placed, "ok": ok,
               "max_rel_rot_deg": rot_max_deg, "rel_rot_axis_tool": rot_axis,
               "rel_rot_per_phase_deg": rot_phases,
               "rel_rot_tol_deg": ROT_TOL_DEG, "rigid_carry": rigid,
               "descent_segments": descent_segments,
               "descent_ok": descent_ok, "moves": MOVES,
               "reach_nudge_mm": REACH_NUDGE, "reach_clean": reach_clean,
               "descent_aim_err_mm": DESCENT_AIM.get("err_mm"),
               "close": dict(SQUEEZE_STATS),
               "final": list(p), "hold_mechanism": "friction",
               "interference_m": INTERFERENCE, "grip_n_per_pad": GRIP_KP * INTERFERENCE, "log": log}, fh, indent=1)

if os.environ.get("PICK_AUTOQUIT"):
    robot.simulationQuit(0 if ok else 1)
while robot.step(dt) != -1:
    bridge.tick(robot.getTime())
