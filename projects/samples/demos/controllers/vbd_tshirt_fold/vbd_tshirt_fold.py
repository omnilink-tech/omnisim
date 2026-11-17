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
"""Pinch one edge of a t-shirt lying flat on a table, carry it across, fold it.

Drives `newton_tshirt_fold.omniworld`, the world that runs BOTH solvers at once:

    newtonSolver "mujoco+vbd"
        the `mjc` entry owns every rigid body and every joint  (gantry, table, jaws)
        the `vbd` entry owns every particle                    (the shirt)
        a Proxy mapping couples them, so the gripper feels the fabric without
        MuJoCo ever knowing that particles exist.

It is the same gantry shape as `vbd_tshirt_grasp` and `vbd_cloth_grasp`, and it
is deliberately written as their sibling. Three things differ, and each one
changes what the controller may claim.

⚠ 1. A POSITION SENSOR IS TRUSTWORTHY HERE, AND IT IS NOT IN THE SIBLINGS.
The two VBD demos refuse to read `PositionSensor` and go to `Supervisor` +
`getFromDef(...).getPosition()` instead, because under `newtonSolver "vbd"`
there is no mj_model and VBD -- like every maximal-coordinate solver --
integrates body_q and never maintains joint_q, so a sensor reads 0.0 for ever,
silently. That reasoning does NOT carry over. On `"mujoco+vbd"` the runtime
leaves `_force_mujoco` True (`omnisim_newton_runtime.py`: every solver string
except the bare `"vbd"` takes the untouched SolverMuJoCo path), so the gantry is
a MuJoCo articulation with a real joint_q and a `PositionSensor` on these five
sliders is the honest instrument.

⚠ 2. ...BUT THE WORLD DOES NOT DECLARE ANY. Read as authored, none of the five
joints carries a `PositionSensor`, so this controller has NO readback channel at
all and every `achieved` field below is `null`. That is reported, once, loudly,
at startup -- it is never papered over by echoing the command back as if it had
been measured. To turn the measurement on, add to each joint's `device [ ]`:

        PositionSensor { name "gantry_x_sensor" }        (and the other four)

Nothing here has to change: the probe is `Motor.getPositionSensor()`, which
resolves the sensor coupled to the same joint whatever it is named.

⚠ 3. THIS CONTROLLER STILL CANNOT SCORE THE FOLD, and does not try. Cloth
particles are not scene nodes: there is no supervisor accessor and no HTTP
endpoint for them, so the fabric is invisible from in here. The verdict comes
from joining this log with the engine's own OMNISIM_CLOTH_TELEMETRY JSONL after
the run. What this file contributes to that join is one measurable quantity --
GAP_EXCESS, how far the jaws fall SHORT of the separation they were just
commanded. An empty gripper reaches its command at any opening; a loaded one
cannot, because the fabric is a spring it has to compress. So

    gap_excess = measured_pad_separation - 2 * (PAD_REST_OFFSET - jaw_command)

is positive only while something is genuinely between the pads. It is relative
to the LIVE command, not to a fixed threshold, because a threshold guessed from
the closed command is cleared trivially by an open jaw and reports "holding" for
phases that happened before the gripper went anywhere near the shirt. (That
mistake is on the record in `vbd_sponge_dishwash`, where it scored all 492
samples of a run as a hold.) With no sensors declared, gap_excess is `null` too.

RUN
    # NOTE: do NOT set OMNISIM_CLOTH_SELF_CONTACT=0 here the way the grasp demos
    # do. A fold STACKS fabric on fabric; without self-contact the carried sleeve
    # sinks through the body of the shirt and there is no fold to look at. The
    # world declares newtonClothSelfContact 1 on purpose and it is the expensive
    # choice -- see the world header for the 17x measurement behind it.
    OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/tshirt_fold_cloth.jsonl \
    OMNISIM_CLOTH_TELEMETRY_EVERY=10 \
    FOLD_LOG=$PWD/.build_tmp/tshirt_fold_pads.jsonl \
    python -m omnisim run-headless \
      projects/samples/demos/worlds/physics/newton_tshirt_fold.omniworld \
      --duration 400

    FOLD_CONTROL_MISS=1    THE REQUIRED NEGATIVE CONTROL. Identical schedule,
                           identical heights, identical timings, identical table
                           contact -- the grab and place points are translated in
                           Y to bare table, so the jaws close on empty air. If
                           that run also produces a fold, the fold is not being
                           produced by the grasp.

⚠ SCOPE. Folding a garment properly is a research problem. ONE clean fold is the
deliverable: pinch one sleeve, lift, carry it across the body, set it down. A
full folding routine is not in scope and this controller does not pretend to be
a step toward one.
"""

from __future__ import annotations

import json
import math
import os
import sys

from omnisim import Robot

# --------------------------------------------------------------------------
# THE WORLD'S OWN NUMBERS. Every constant in this block is a copy of something
# declared in newton_tshirt_fold.omniworld. They are copied rather than derived
# because a controller cannot read a Cloth or a motor range back, and they are
# listed together so that the copy is auditable in one place. If you change the
# world, change these -- the startup checks below will otherwise silently pass
# on a stale model of the scene.
# --------------------------------------------------------------------------

GANTRY_ROOT = (0.0, 0.0, 1.35)   # DEF GANTRY Robot { translation 0 0 1.35 }

# LinearMotor min/maxPosition, in the order (min, max). Joint coordinates are
# DELTAS from the gantry root, so every joint starts at 0 and no PD has a step
# to chase at t = 0.
MOTOR_RANGE = {
    "gantry_x": (-0.60, 0.60),
    "gantry_y": (-0.45, 0.45),
    "gantry_z": (-0.62, 0.05),   # negative == downward from the root
    "jaw_left": (-0.20, 0.06),
    "jaw_right": (-0.20, 0.06),
}

TABLE_TOP = 0.75                 # DEF TABLE, translation z 0.725 + half of 0.05

# Jaw pad: Box { size 0.07 0.01 0.03 } -- 0.07 along X (the approach), 0.01
# along Y (the closing axis), 0.03 along Z (the height that has to straddle the
# fabric without hitting the table).
PAD_HALF_THICK = 0.005           # half of the 10 mm closing-axis thickness
PAD_HALF_HEIGHT = 0.015          # half of the 30 mm height
PAD_REST_OFFSET = 0.050          # authored |y| of each pad -> a 100 mm open gap

PARTICLE_RADIUS = 0.0046         # DEF TSHIRT Cloth { particleRadius 0.0046 }

# MEASURED off meshes/tshirt_md.obj (9468 verts) pushed through the world's own
# transform -- `translation 0 0 0.86, rotation 1 0 0 -1.5708`, which maps mesh
# local x -> world x and local z -> world y. This is the AUTHORED REST mesh, not
# the settled garment; settling flattens the tube and will move these by
# millimetres, not centimetres.
#
#     world x   [-0.3234, +0.3234]   sleeve tip to sleeve tip
#     world y   [ 0.0000, +0.6434]   hem edge (y=0) to shoulders (y=0.6434)
#     world z   [ 0.7234, +0.9966]   the tube, before gravity flattens it
#
# ⚠ THE GARMENT IS NOT CENTRED ON THE ORIGIN IN Y. The mesh's local z runs
# 0 -> 0.6434 rather than +-0.3217, and the world's rotation carries that offset
# straight through, so the shirt lies entirely in the +Y half-plane. See the
# GEOMETRY FINDINGS block below -- this is the single fact that decides where
# the grab point has to be, and it is why FOLD_SHIRT_Y0 exists.
SHIRT_HALF_X = 0.3234
SHIRT_Y_SPAN = 0.6434            # hem edge at mesh y=0, shoulders at mesh y=+span

# ⚠ A T-SHIRT IS NOT ITS BOUNDING BOX, AND THE DIFFERENCE IS EXACTLY WHERE THIS
# DEMO GRABS. Between the sleeve line and the torso there is no fabric at all:
# at |x| >= 0.22 the garment exists only from y ~ 0.33 to y ~ 0.57 (sleeve),
# while at |x| <= 0.18 it runs from the hem at y ~ 0 to the shoulders at
# y ~ 0.64 (torso). A bbox test calls the point (-0.28, 0.00) "on the shirt";
# the mesh says that point has ZERO vertices within a whole pad footprint of it.
# So the check below uses the measured silhouette instead: (x band centre, y_min,
# y_max) over +-0.02 m bands of the same 9468-vertex rest mesh, in the MESH's
# own y -- add FOLD_SHIRT_Y0 to get world y.
SHIRT_SILHOUETTE = (
    (-0.34, 0.3779, 0.3785), (-0.30, 0.3471, 0.4490), (-0.26, 0.3358, 0.5196),
    (-0.22, 0.3285, 0.5672), (-0.18, 0.0027, 0.5831), (-0.14, 0.0018, 0.5976),
    (-0.10, 0.0030, 0.6172), (-0.06, 0.0010, 0.6354), (-0.02, 0.0055, 0.6434),
    (0.02, 0.0052, 0.6431), (0.06, 0.0044, 0.6350), (0.10, 0.0001, 0.6166),
    (0.14, 0.0000, 0.5980), (0.18, 0.0047, 0.5828), (0.22, 0.3385, 0.5657),
    (0.26, 0.3350, 0.5167), (0.30, 0.3566, 0.4435), (0.34, 0.3750, 0.3842),
)

# The flattened sleeve's y-centre at the grab x, measured in the same pass:
# vertices within +-0.02 of x = -0.28 span y in [0.3401, 0.4855], midpoint
# 0.4128. Closing 100 mm jaws about that line puts both pads INSIDE a 145 mm
# band of fabric, so the pinch GATHERS the two flattened walls of the sleeve
# rather than pinching one flat panel. That is a different mechanism from the
# hanging-sheet demos and it is the one the `sleeve` preset of vbd_tshirt_grasp
# was built on, where 12 particles came into the jaws together.
SLEEVE_LINE_Y = 0.4128

# --------------------------------------------------------------------------
# GEOMETRY FINDINGS -- read this before tuning anything.
#
# Four numbers in the brief this controller was written to do not survive
# contact with the mesh and the table. All four are exposed as env vars so the
# world can be fixed without touching code, and all four are re-derived and
# PRINTED at startup so a stale assumption cannot hide.
#
# (1) THE BRIEFED GRAB POINT (-0.28, 0.0, ...) IS EMPTY AIR. World x = -0.28 is
#     the sleeve line, but the shirt's y footprint is [0, 0.6434], not the
#     [-0.32, +0.32] the brief assumed. Counted directly: a 70 x 100 mm pad
#     footprint at (-0.28, 0.00) contains ZERO vertices, and so does the place
#     point (+0.24, 0.00). Run as briefed, the demo and its own negative control
#     would be the same experiment. The fabric at x = -0.28 is at y in
#     [0.3401, 0.4855], hence FOLD_GRAB_Y's default of SHIRT_Y0 + 0.4128.
#
# (2) THE SHIRT OVERHANGS THE TABLE. The table is 1.0 m in Y, so it ends at
#     y = +0.50; the shoulders reach y = +0.6434. 143 mm of garment starts over
#     thin air and will drape off the edge during `settle`.
#     Both (1) and (2) have ONE fix, in the world rather than here:
#         DEF TSHIRT Cloth { translation 0 -0.3217 0.86 }
#     which recentres the footprint to y in [-0.3217, +0.3217] -- exactly the
#     [-0.32, +0.32] the brief describes. If you make that edit, set
#     FOLD_SHIRT_Y0=-0.3217 and every default below follows it; nothing else
#     needs to move.
#
# (3) THE SHIRT STARTS 27 mm INSIDE THE TABLE. Authored world z runs from
#     0.7234, and the table top is 0.75. The world's comment says it is
#     "released a little ABOVE the table"; the mesh says its lower wall begins
#     26.6 mm below it. With self-contact ON, a t=0 interpenetration is the
#     worst place to start. `translation 0 <y> 0.90` clears it by 13 mm.
#
# (4) THE BRIEFED GRAB HEIGHT DRIVES THE PADS INTO THE TABLE. FOLD_GRAB_Z is the
#     pad CENTRE, so a pad bottom sits 15 mm below it: at the briefed 0.755 the
#     bottom is 0.740, i.e. 10 mm inside a static table. gantry_z's PD stiffness
#     is maxForce * 10 = 50,000 N/m, so the servo holds that error with ~500 N
#     pressing down through a 70 x 10 mm face. It is not an instability -- the
#     table is static and simply stops the pad at 0.750 -- but it crushes
#     whatever fabric is under the pad for the whole grasp.
#     0.765 puts the pad bottom exactly on the table top, and the fabric still
#     lies inside the pad's vertical span: one settled layer's particle centres
#     sit at TABLE_TOP + r = 0.7546 and a second at ~0.7638, both inside
#     [0.750, 0.780]. The default below stays at the briefed 0.755 so the
#     contract is honoured verbatim, and startup WARNS with the number.
# --------------------------------------------------------------------------


def env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return float(raw)
    except ValueError:
        print("vbd_tshirt_fold: WARNING %s=%r is not a number, using %r"
              % (name, raw, default), flush=True)
        return default


def env_flag(name, default=False):
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    return raw.strip().lower() not in ("0", "false", "off", "no")


def smoothstep(a, b, u):
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    return a + (b - a) * (u * u * (3.0 - 2.0 * u))


def num(v, nd=6):
    """JSON-safe number. NaN/inf become null rather than invalid JSON.

    A PositionSensor reads NaN until it has been enabled AND one step has
    elapsed, so this is on the hot path for the first sample of every run.
    """
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if not math.isfinite(f) else round(f, nd)


def shirt_y_extent(x):
    """(y_min, y_max) of fabric at world x, in MESH y -- or None for no fabric.

    Nearest-band lookup over SHIRT_SILHOUETTE, whose +-0.02 bands tile the 0.04
    grid exactly. Coarse on purpose: this answers "is there any garment here",
    which is the question that decides whether a run is distinguishable from its
    own negative control. It cannot answer "will the pinch catch N particles" --
    nothing outside the engine can, because it is a question about the SETTLED
    garment and this is the rest mesh.
    """
    best, dist = None, 1e9
    for cx, lo, hi in SHIRT_SILHOUETTE:
        d = abs(x - cx)
        if d < dist:
            best, dist = (lo, hi), d
    return best if dist <= 0.02 else None


def clamp_cmd(name, value, clamped):
    lo, hi = MOTOR_RANGE[name]
    if value < lo or value > hi:
        clamped.setdefault(name, []).append(round(value, 6))
        return lo if value < lo else hi
    return value


def main():
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    # ---------------------------------------------------------------- pinch
    # GRIP_COMPRESSION is how far the pad faces close INSIDE the particle
    # shell, and it is the ONLY thing that generates grip force here. These
    # joints are POSITION_VELOCITY servos: `setForce` does not put a Newton
    # joint into force mode -- the PD stays live, anchored at the last
    # setPosition -- so a squeeze is commanded as a position with a known
    # interference, never as a force.
    #
    # ⚠ RE-DERIVED, NOT COPIED. vbd_tshirt_grasp uses 0.002 against a 0.010
    # particle radius (newton's own guidance, ~0.2 x r). This world's radius is
    # 0.0046, less than half of that, so 0.2 x r would be 0.92 mm. 1.5 mm is
    # deliberately richer than the guidance -- 0.33 x r -- because a fold has to
    # survive a 0.52 m traverse, not just a vertical lift, and a slip mid-carry
    # is indistinguishable from a fold that never took.
    grip_compression = env_float("FOLD_GRIP_COMPRESSION", 0.0015)

    # Jaw joint coordinate: 0 == open (the authored pad pose), + == closing.
    # Both jaws take the SAME command because their axes point at each other
    # (JL axis 0 1 0, JR axis 0 -1 0), which keeps the pinch symmetric by
    # construction rather than by two matching sign flips.
    # !! LAYERS, not layer. This term was PARTICLE_RADIUS (one flat sheet) and it
    # made the gripper EXTRUDE the fabric instead of holding it. Measured on the
    # first run: gap excess peaked at 13.15 mm during `close` -- i.e. it really
    # did catch two sleeve walls -- then decayed monotonically to 0.00 mm before
    # `lift` even finished. That decay is not slip, it is the PD servo squeezing
    # cloth out sideways, because a one-layer derivation commands a 6.2 mm
    # face-to-face gap onto ~13 mm of gathered fabric and simply crushes it out.
    #
    # A hem or sleeve edge is a folded tube: the pads meet TWO walls. Biting
    # 2r - compression leaves a gap just under the real thickness, which is what
    # a friction grasp needs -- fabric is held by normal force through friction,
    # not by being flattened.
    grip_layers = max(env_float("FOLD_GRIP_LAYERS", 2.0), 1.0)
    # !! THE SELF-CONTACT TERM, and it is why "grasping needs self-contact OFF"
    # was never a law of physics. With self-contact ON, newton pushes any two
    # fabric surfaces apart until they sit `particle_self_contact_radius` apart
    # (a SURFACE distance -- read evaluate_self_contact_force_norm in
    # newton/_src/solvers/vbd/particle_vbd_kernels.py: the barrier activates at
    # dis < collision_radius). So the stack between the pads genuinely needs
    #
    #     2 * PARTICLE_RADIUS            (each layer held off its pad face)
    #   + (layers-1) * self_contact_r    (layer-to-layer separation)
    #
    # of face-to-face gap before compression even begins. The old grasp worlds
    # commanded 6.2 mm onto a stack that needs 13.8, a 7.6 mm violation the
    # fabric could only resolve by EJECTING ITSELF -- which is precisely the
    # "24x tracking error with self-contact ON" that made every deformable-grasp
    # world turn it off (and turning it off is what makes a garment render as a
    # self-intersecting ghost). Account for the term and both halves can be on.
    #
    # FOLD_SELF_CONTACT_RADIUS must match OMNISIM_CLOTH_SELF_CONTACT_RADIUS when
    # that env var is set; the default matches the runtime's own default, which
    # is the world's particleRadius.
    self_contact_r = env_float("FOLD_SELF_CONTACT_RADIUS",
                               env_float("OMNISIM_CLOTH_SELF_CONTACT_RADIUS",
                                         PARTICLE_RADIUS))
    closed_half_gap = (PAD_HALF_THICK
                       + PARTICLE_RADIUS
                       + 0.5 * (grip_layers - 1.0) * self_contact_r
                       - grip_compression)
    jaw_open = 0.0
    jaw_closed = PAD_REST_OFFSET - closed_half_gap
    #   closed_half_gap = 0.005 + 0.0046 - 0.0015 = 0.0081   (pad CENTRE to axis)
    #   jaw_closed      = 0.050 - 0.0081        = 0.0419     (<= maxPosition 0.06)
    #   pad centre-to-centre closed             = 0.0162     (16.2 mm)
    #   pad FACE-to-face closed                 = 0.0062     ( 6.2 mm)
    # against one flat layer 2r = 9.2 mm thick (3.0 mm of total interference)
    # and, at the sleeve, two gathered walls 18.4 mm thick. The jaws will not
    # reach 0.0419 with the sleeve in them, and that shortfall IS the signal --
    # see gap_excess in the header.

    # ⚠ Opening back to the authored 100 mm gap does not necessarily let go. By
    # release the fabric has gathered into a bundle, and pads that re-open
    # INSIDE the bundle stop being a pinch and become two ledges -- measured on
    # the patch world, where it produced a false FAIL. The competing failure is
    # the opposite one and it is specific to a FOLD: the pads are at fabric
    # height over a just-placed layer, so every millimetre of outward travel
    # rakes across the fold and can drag it open. The compromise is a 180 mm
    # gap: wider than anything 100 mm jaws could have gathered, and 40 mm of
    # outward travel per pad rather than the 190 mm the hanging demos use.
    release_half_gap = env_float("FOLD_RELEASE_HALF_GAP", 0.090)
    jaw_release = PAD_REST_OFFSET - release_half_gap          # 0.050 - 0.090 = -0.040

    # ------------------------------------------------------------ waypoints
    # The world y at which the shirt's MESH ORIGIN sits, i.e. the y of
    # `DEF TSHIRT Cloth { translation ... }`. The world as authored says 0, and
    # the mesh is not centred on its own origin, so the garment lies in
    # y in [0, 0.6434]. Recentre the Cloth and set this to match; every default
    # below is expressed relative to it, so nothing else has to move.
    shirt_y0 = env_float("FOLD_SHIRT_Y0", -0.3217)   # world now centres the shirt

    grab_x = env_float("FOLD_GRAB_X", -0.28)
    grab_y = env_float("FOLD_GRAB_Y", shirt_y0 + SLEEVE_LINE_Y)
    grab_z = env_float("FOLD_GRAB_Z", 0.765)     # 0.755 pressed the pads 10 mm THROUGH the table
    target_x = env_float("FOLD_TARGET_X", 0.24)
    target_y = env_float("FOLD_TARGET_Y", grab_y)     # a fold is a pure +X carry
    target_z = env_float("FOLD_TARGET_Z", 0.80)
    lift_dz = env_float("FOLD_LIFT", 0.12)
    clear_z = env_float("FOLD_CLEAR_Z", 0.95)         # approach / retract height
    carry_z = grab_z + lift_dz

    # THE NEGATIVE CONTROL. One variable moves: the Y of the whole trajectory.
    # Heights, timings, traverse length, jaw schedule and the pads' contact with
    # the table are all untouched, so the only thing that differs between the two
    # runs is whether there is fabric between the jaws. -0.42 is clear of the
    # garment in BOTH world variants (un-recentred: 0.42 m from the hem edge at
    # y=0; recentred: 0.098 m past it) and still over the table, whose Y half-
    # span is 0.50 -- so the pads still land on a surface, which matters,
    # because a control that also stopped touching the table would be changing
    # two things at once.
    control_miss = env_flag("FOLD_CONTROL_MISS", False)
    miss_y = env_float("FOLD_MISS_Y", -0.42)
    if control_miss:
        grab_y = miss_y
        target_y = miss_y

    strict = env_flag("FOLD_STRICT", False)

    # ------------------------------------------------------------- attach
    # KINEMATIC GRAB. The friction pinch above is kept (it still shapes the
    # fabric between the pads and it is what the negative control exercises),
    # but the thing that actually CARRIES the garment is a particle attach:
    # at the moment the close converges, the controller asks the engine --
    # via the OMNISIM_CLOTH_ATTACH_CMD command file both processes inherit --
    # to pin the cloth particles at the pinch point to the left pad body, and
    # it releases them at the entry to `release`. The campaign (12 laptop
    # configs + 8 cloud seeds, valid controls) measured that VBD's
    # velocity-regularised friction cannot hold a static pinch -- the creep
    # velocity under load W is ~ W*eps_u/(2*mu*N), nonzero for every mu -- so
    # a friction-only carry is not a tuning problem to keep chasing.
    #
    # The ack file is the HONEST half of the protocol: the engine reports how
    # many particles the grab actually selected, and this controller logs
    # that number -- an attach that selected 0 (the control arm, by design)
    # is a recorded miss, never an assumed hold.
    attach_cmd_path = os.environ.get("OMNISIM_CLOTH_ATTACH_CMD", "")
    attach_on = env_flag("FOLD_ATTACH", True) and bool(attach_cmd_path)
    attach_body = int(env_float("FOLD_ATTACH_BODY", 6))   # left pad in this world
    attach_radius = env_float("FOLD_ATTACH_RADIUS", 0.045)
    attach_max = int(env_float("FOLD_ATTACH_MAX", 0))     # 0 = uncapped

    # ---------------------------------------------------------------- phases
    # Cloth with self-contact ON is the expensive configuration in this tree, so
    # these are sized to the motion and not padded. Every one is overridable
    # because the right answer depends on the settle behaviour, which is a
    # physics question this controller cannot see.
    phases = [
        # the shirt falls the last centimetres and the tube flattens. Longer
        # than the grasp demos' 0.50 s, which start from an already-hanging
        # garment: this one has to fall AND flatten.
        ("settle", env_float("FOLD_T_SETTLE", 1.50)),
        # to (grab_x, grab_y) at clear_z, jaws open. Everything moves at once;
        # clear_z is 0.18 m above the settled fabric so nothing sweeps it.
        ("approach", env_float("FOLD_T_APPROACH", 0.80)),
        # straight down onto the grab point. Slow: the pads finish against the
        # table and a fast arrival is a hammer blow through a 50 kN/m servo.
        ("descend", env_float("FOLD_T_DESCEND", 0.70)),
        # jaws open -> closed, IN PLACE (newton's guidance: never close while
        # translating, or the pads shovel the fabric out of the gap).
        ("close", env_float("FOLD_T_CLOSE", 0.50)),
        ("lift", env_float("FOLD_T_LIFT", 0.90)),
        # 0.52 m across the body. Mean 0.33 m/s -- cloth does not tolerate a
        # fast carry, and this is the phase where a weak pinch shows up as slip.
        ("traverse", env_float("FOLD_T_TRAVERSE", 1.60)),
        # only 0.075 m of descent, deliberately given a long window: smoothstep
        # eases out, so the tail of this phase is where the carried fabric
        # relaxes onto the layer below before anything lets go.
        ("lower", env_float("FOLD_T_LOWER", 0.90)),
        ("release", env_float("FOLD_T_RELEASE", 0.50)),
        # up and away. The move is done in the first 60% (see schedule) so the
        # rest is a static window with the fold unobstructed -- this controller
        # cannot pause the engine (see the end of main), so the alternative is
        # ending the run with the pads still moving.
        ("retract", env_float("FOLD_T_RETRACT", 1.20)),
    ]

    bounds, acc = [], 0
    for name, secs in phases:
        n = max(int(round(secs / dt)), 1)
        bounds.append((name, acc, acc + n))
        acc += n
    total = acc

    # ---------------------------------------------------------------- devices
    motors = {}
    for name in ("gantry_x", "gantry_y", "gantry_z", "jaw_left", "jaw_right"):
        m = robot.getDevice(name)
        if m is None:
            print("vbd_tshirt_fold: FATAL device '%s' missing -- check the world's "
                  "LinearMotor names against MOTOR_RANGE at the top of this file."
                  % name, flush=True)
            return 1
        motors[name] = m

    # Probe for readback. `Motor.getPositionSensor()` resolves the sensor
    # COUPLED TO THE SAME JOINT, whatever it is called, so it finds one no
    # matter what the world names it; the by-name probe below is a fallback for
    # a sensor that somehow is not coupled. `Robot.devices` is consulted
    # directly rather than calling getDevice() blind, because getDevice() prints
    # to stderr on a miss and a miss is the expected case here.
    known = getattr(robot, "devices", None)
    sensors = {}
    for name, m in motors.items():
        s = None
        try:
            s = m.getPositionSensor()
        except Exception:                                        # noqa: BLE001
            s = None
        if s is None and isinstance(known, dict):
            for guess in (name + "_sensor", name + "_pos", name + "_position"):
                if guess in known:
                    s = robot.getDevice(guess)
                    break
        if s is not None:
            try:
                s.enable(dt_ms)
            except Exception:                                    # noqa: BLE001
                s = None
        sensors[name] = s
    measured = all(sensors[n] is not None for n in motors)

    # ------------------------------------------------------- derived geometry
    pad_bottom = grab_z - PAD_HALF_HEIGHT
    table_clearance = pad_bottom - TABLE_TOP
    layer1_z = TABLE_TOP + PARTICLE_RADIUS                  # one settled layer
    layer2_z = layer1_z + 2.0 * PARTICLE_RADIUS             # a second on top of it
    fabric_in_pad = (pad_bottom <= layer1_z <= grab_z + PAD_HALF_HEIGHT)

    # THE FOLD'S MATERIAL BUDGET. The crease lands halfway between the grab and
    # the place point, so the fabric between crease and grip is a fixed length;
    # the grip's straight-line distance from the crease must not exceed it.
    crease_x = 0.5 * (grab_x + target_x)
    material = abs(grab_x - crease_x)
    chord_carry = math.hypot(target_x - crease_x, carry_z - layer1_z)
    chord_place = math.hypot(target_x - crease_x, target_z - layer1_z)
    slack_carry = material - chord_carry
    slack_place = material - chord_place
    # ⚠ slack_carry is NEGATIVE for any non-zero lift and no choice of
    # FOLD_TARGET_X fixes it: chord = hypot(material, dz) >= material always.
    # The budget can only be met with the grip back at fabric height, which is
    # what `lower` is for. Until then the fabric pays the difference by sliding
    # -- the crease migrates and the far half of the shirt is dragged toward the
    # gripper. That is real fold behaviour rather than a bug, but it scales with
    # FOLD_LIFT, so if the shirt is being dragged across the table, lower the
    # lift before suspecting the grasp.

    # ---------------------------------------------------------------- report
    print("vbd_tshirt_fold: dt=%d ms, %d steps (%.2f s), control_miss=%s"
          % (dt_ms, total, total * dt, control_miss), flush=True)
    print("vbd_tshirt_fold: grab=(%.4f, %.4f, %.4f)  place=(%.4f, %.4f, %.4f)  "
          "lift=%.3f clear=%.3f"
          % (grab_x, grab_y, grab_z, target_x, target_y, target_z, lift_dz, clear_z),
          flush=True)
    print("vbd_tshirt_fold: jaw open=%.4f closed=%.4f release=%.4f  "
          "(closed half-gap %.4f, face gap %.4f, one layer %.4f)"
          % (jaw_open, jaw_closed, jaw_release, closed_half_gap,
             2.0 * (PARTICLE_RADIUS - grip_compression), 2.0 * PARTICLE_RADIUS),
          flush=True)
    print("vbd_tshirt_fold: PHASES " + json.dumps(
        [{"phase": n, "start": s, "end": e} for n, s, e in bounds]), flush=True)

    problems = []
    if not measured:
        missing = [n for n in motors if sensors[n] is None]
        # ⚠ ASCII ONLY IN print(). The engine captures a controller's stdout
        # through a cp1252 console on Windows, so a "⚠" here raises
        # UnicodeEncodeError and kills the controller at startup -- which the
        # engine reports as `exited with status: 1` and nothing else. The
        # warning glyphs in this file live in comments, never in output.
        print("vbd_tshirt_fold: !! NO POSITION READBACK on %s -- every `achieved` "
              "and `error` in the log below will be null, and gap_excess with "
              "them. This run records what was COMMANDED and nothing else. Fix: "
              "add `PositionSensor { name \"<joint>_sensor\" }` to each joint's "
              "device list; joint_q is real on the mujoco+vbd path."
              % ",".join(missing), flush=True)
    if table_clearance < 0.0:
        problems.append(
            "pad bottom %.4f is %.1f mm INSIDE the table top %.3f; the gantry_z "
            "servo (ke = 50000 N/m) will hold that with ~%.0f N pressing through "
            "the pads. FOLD_GRAB_Z=%.3f puts the bottom exactly on the table with "
            "both fabric layers still inside the pad's span."
            % (pad_bottom, -table_clearance * 1000.0, TABLE_TOP,
               -table_clearance * 50000.0, TABLE_TOP + PAD_HALF_HEIGHT))
    if not fabric_in_pad:
        problems.append(
            "a settled layer at z=%.4f is OUTSIDE the pad's vertical span "
            "[%.4f, %.4f] -- the jaws would close above or below the fabric."
            % (layer1_z, pad_bottom, grab_z + PAD_HALF_HEIGHT))
    # Fabric-presence check against the measured rest SILHOUETTE. It is a check
    # on the AUTHORED mesh, so it cannot prove a grab lands on fabric after
    # settling -- but it does catch the whole-garment miss, which is the failure
    # that makes a demo indistinguishable from its own control.
    #
    # Two different questions, deliberately not the same test: the run needs
    # fabric ON THE PINCH LINE, and the control needs no fabric anywhere in the
    # 100 mm the open jaws sweep.
    extent = shirt_y_extent(grab_x)
    grab_y_mesh = grab_y - shirt_y0
    if extent is None:
        band = "no fabric at all at x=%.3f" % grab_x
        on_pinch_line, swept_clear = False, True
    else:
        lo, hi = extent[0] + shirt_y0, extent[1] + shirt_y0
        band = "fabric at x=%.3f spans y [%.4f, %.4f]" % (grab_x, lo, hi)
        on_pinch_line = (extent[0] - 0.02) <= grab_y_mesh <= (extent[1] + 0.02)
        swept_clear = (grab_y + PAD_REST_OFFSET < lo) or (grab_y - PAD_REST_OFFSET > hi)
    if control_miss:
        if not swept_clear:
            problems.append(
                "FOLD_CONTROL_MISS is set but the open jaws still sweep fabric at "
                "(%.3f, %.3f): %s, and the pads span y +-%.3f about the line. The "
                "control would grab something and prove nothing."
                % (grab_x, grab_y, band, PAD_REST_OFFSET))
        else:
            print("vbd_tshirt_fold: control arm OK -- the open jaws at (%.3f, %.3f) "
                  "sweep y [%.4f, %.4f] and %s"
                  % (grab_x, grab_y, grab_y - PAD_REST_OFFSET,
                     grab_y + PAD_REST_OFFSET, band), flush=True)
    elif not on_pinch_line:
        problems.append(
            "the grab point (%.3f, %.3f) has NO FABRIC ON IT -- %s. The jaws would "
            "close on empty air and this run would be its own negative control. A "
            "t-shirt is not its bounding box: between the sleeve and the torso "
            "there is a gap. Set FOLD_GRAB_Y to the middle of that band (the "
            "sleeve line at the default grab x is %.4f), or set FOLD_SHIRT_Y0 if "
            "you recentred the Cloth."
            % (grab_x, grab_y, band, shirt_y0 + SLEEVE_LINE_Y))
    if slack_place < 0.0:
        problems.append(
            "the fold is %.1f mm short of material even at the place height: "
            "%.4f m of fabric between crease x=%.3f and the grip, against a %.4f m "
            "chord. The shirt will be dragged rather than folded; shorten the "
            "traverse or lower FOLD_TARGET_Z."
            % (-slack_place * 1000.0, material, crease_x, chord_place))

    for p in problems:
        print("vbd_tshirt_fold: !! GEOMETRY " + p, flush=True)
    print("vbd_tshirt_fold: GEOMETRY " + json.dumps({
        "pad_bottom_at_grab": num(pad_bottom),
        "table_clearance": num(table_clearance),
        "settled_layer_z": [num(layer1_z), num(layer2_z)],
        "fabric_inside_pad_span": fabric_in_pad,
        "crease_x": num(crease_x),
        "material": num(material),
        "chord_at_carry": num(chord_carry), "slack_at_carry": num(slack_carry),
        "chord_at_place": num(chord_place), "slack_at_place": num(slack_place),
    }), flush=True)
    if problems and strict:
        print("vbd_tshirt_fold: FATAL FOLD_STRICT=1 and %d geometry check(s) failed "
              "-- refusing to spend a cloth run on a scene that cannot produce the "
              "result it would be scored on." % len(problems), flush=True)
        return 1

    # ---------------------------------------------------------------- schedule
    root_x, root_y, root_z = GANTRY_ROOT

    def schedule(step):
        """-> (phase, world_x, world_y, world_z, jaw_cmd). World frame; the
        conversion to joint coordinates happens once, at the command site."""
        for name, s, e in bounds:
            if step < e:
                u = (step - s) / max(e - s, 1)
                if name == "settle":
                    # Parked at the authored joint zero: pad centre z = 1.35,
                    # 0.58 m above the table and nowhere near the garment.
                    return name, root_x, root_y, root_z, jaw_open
                if name == "approach":
                    return (name,
                            smoothstep(root_x, grab_x, u),
                            smoothstep(root_y, grab_y, u),
                            smoothstep(root_z, clear_z, u), jaw_open)
                if name == "descend":
                    return (name, grab_x, grab_y,
                            smoothstep(clear_z, grab_z, u), jaw_open)
                if name == "close":
                    return (name, grab_x, grab_y, grab_z,
                            smoothstep(jaw_open, jaw_closed, u))
                if name == "lift":
                    return (name, grab_x, grab_y,
                            smoothstep(grab_z, carry_z, u), jaw_closed)
                if name == "traverse":
                    return (name, smoothstep(grab_x, target_x, u),
                            smoothstep(grab_y, target_y, u), carry_z, jaw_closed)
                if name == "lower":
                    return (name, target_x, target_y,
                            smoothstep(carry_z, target_z, u), jaw_closed)
                if name == "release":
                    return (name, target_x, target_y, target_z,
                            smoothstep(jaw_closed, jaw_release, u))
                # retract: clear the fold in the first 60% of the phase, then
                # hold still so the run does not end mid-motion.
                return (name, target_x, target_y,
                        smoothstep(target_z, clear_z, min(u / 0.6, 1.0)), jaw_release)
        return "done", target_x, target_y, clear_z, jaw_release

    # ---------------------------------------------------------------- logging
    log_path = os.environ.get("FOLD_LOG") or os.path.join(
        os.getcwd(), ".build_tmp", "tshirt_fold_pads.jsonl")
    try:
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
    except OSError:
        pass
    log_every = max(int(env_float("FOLD_LOG_EVERY", 5)), 1)

    # buffering=1 (line buffered) is REQUIRED, not tidiness: the engine kills the
    # controller at end of run, and a default-buffered file loses every row still
    # in the buffer -- which is how a run that drove the gantry correctly produced
    # a ZERO-ROW telemetry file and looked like a controller that never started.
    fh = open(log_path, "w", encoding="utf-8", buffering=1)
    fh.write(json.dumps({
        "meta": True,
        "controller": "vbd_tshirt_fold",
        "world": "newton_tshirt_fold.omniworld",
        "control_miss": control_miss,
        "measured": measured,
        "dt": num(dt), "steps": total, "log_every": log_every,
        "grab": [num(grab_x), num(grab_y), num(grab_z)],
        "place": [num(target_x), num(target_y), num(target_z)],
        "lift": num(lift_dz), "clear_z": num(clear_z), "carry_z": num(carry_z),
        "shirt_y0": num(shirt_y0),
        "jaw": {"open": num(jaw_open), "closed": num(jaw_closed),
                "release": num(jaw_release),
                "closed_half_gap": num(closed_half_gap),
                "grip_compression": num(grip_compression)},
        "pad": {"half_thick": PAD_HALF_THICK, "half_height": PAD_HALF_HEIGHT,
                "rest_offset": PAD_REST_OFFSET, "size": [0.07, 0.01, 0.03]},
        "particle_radius": PARTICLE_RADIUS,
        # The rest-mesh footprint this run's checks were made against, so the
        # join downstream can tell a stale controller from a re-authored world
        # without re-measuring the OBJ.
        "shirt_rest_footprint": {
            "half_x": SHIRT_HALF_X,
            "mesh_y": [0.0, SHIRT_Y_SPAN],
            "world_y": [num(shirt_y0), num(shirt_y0 + SHIRT_Y_SPAN)],
            "extent_at_grab_x": None if extent is None else [num(extent[0]), num(extent[1])],
        },
        "phases": [{"phase": n, "start": s, "end": e} for n, s, e in bounds],
        "geometry_warnings": problems,
        "attach": {"enabled": attach_on, "body": attach_body,
                   "radius": num(attach_radius), "max": attach_max,
                   "cmd_path": attach_cmd_path or None},
    }) + "\n")

    # ---- attach command channel ----------------------------------------
    _attach_ack_ofs = [0]
    _attach_pending = [0]      # acks still expected

    def _attach_send(cmd_obj):
        if not attach_on:
            return
        try:
            with open(attach_cmd_path, "a", encoding="utf-8") as cf:
                cf.write(json.dumps(cmd_obj) + "\n")
                cf.flush()
            _attach_pending[0] += 1
            fh.write(json.dumps({"event": "attach_cmd", "step": step,
                                 "cmd": cmd_obj}) + "\n")
            print("vbd_tshirt_fold: attach_cmd %s" % json.dumps(cmd_obj),
                  flush=True)
        except OSError as e:
            print("vbd_tshirt_fold: !! attach channel write failed (%r)" % (e,),
                  flush=True)

    def _attach_poll_ack():
        if not attach_on or _attach_pending[0] <= 0:
            return
        try:
            ap = attach_cmd_path + ".ack"
            if not os.path.exists(ap):
                return
            with open(ap, "r", encoding="utf-8") as af:
                af.seek(_attach_ack_ofs[0])
                new = af.read()
            if not new:
                return
            _attach_ack_ofs[0] += len(new.encode("utf-8"))
            for line in new.splitlines():
                line = line.strip()
                if not line:
                    continue
                _attach_pending[0] = max(0, _attach_pending[0] - 1)
                fh.write(json.dumps({"event": "attach_ack", "step": step,
                                     "ack": json.loads(line)}) + "\n")
                print("vbd_tshirt_fold: attach_ack %s" % line, flush=True)
        except (OSError, ValueError) as e:
            print("vbd_tshirt_fold: !! attach ack read failed (%r)" % (e,),
                  flush=True)

    def _pad_world_pos():
        """Where the pinch point really is: root + ACHIEVED joint values,
        falling back per-axis to the commanded grab point."""
        ax = _achieved("gantry_x")
        ay = _achieved("gantry_y")
        az = _achieved("gantry_z")
        return (root_x + ax if ax is not None else grab_x,
                root_y + ay if ay is not None else grab_y,
                root_z + az if az is not None else grab_z)

    clamped = {}
    step = 0
    prev_phase = None

    # ---- ARRIVAL-GATED SCHEDULING --------------------------------------
    # The schedule used to advance on the CLOCK alone, and the measured result
    # was jaws closing in MID-AIR: at the moment `close` began, the gantry had
    # achieved z -0.535 against a -0.585 command -- the pads were 50 mm above
    # the fabric, closed like a beak, then descended already shut. Every
    # confusing grip-excess transient in this demo's history was that lag
    # (amplified, before the split-proxy fix, by pads that VBD saw as 200 kg).
    #
    # sched_step is the schedule's own clock. It freezes at a phase boundary
    # until the measured state says the previous phase actually finished, so
    # in-phase timing is untouched and a sensorless world degrades exactly to
    # the old clock-driven behaviour (every gate passes vacuously on null).
    # Each gate has a timeout (FOLD_GATE_TIMEOUT x the next phase's length):
    # a wedged gate must not deadlock the run, and a timeout is printed
    # because a gate that timed out is itself a measurement.
    arrival_tol = env_float("FOLD_ARRIVAL_TOL", 0.004)
    gate_timeout_mult = env_float("FOLD_GATE_TIMEOUT", 3.0)
    phase_len = {n: (e - s) for n, s, e in bounds}

    def _achieved(name):
        srec = sensors.get(name)
        if srec is None:
            return None
        v = srec.getValue()
        return None if v != v else float(v)

    def _gantry_arrived(cmd_map):
        for nm in ("gantry_x", "gantry_y", "gantry_z"):
            got = _achieved(nm)
            if got is not None and abs(got - cmd_map[nm]) > arrival_tol:
                return False
        return True

    _jaw_prev = [None]

    def _gate_open(next_phase, cmd_map):
        if next_phase in ("close", "lower", "release", "traverse"):
            return _gantry_arrived(cmd_map)
        if next_phase == "lift":
            # Two conditions, BOTH required, and the order they were learned:
            # STILLNESS -- a jaw velocity of ~zero says the pinch found its
            # equilibrium (blocked jaws never reach their command, and that
            # block IS a successful pinch, so position-vs-command is wrong
            # here). And PROGRESS -- the jaws must have MOVED at least 40% of
            # the way toward the close command first. Stillness alone cannot
            # tell "converged" from "never started": on the pod, full-surface
            # contact made the close so viscous the jaws sat at -0.0003 for the
            # whole phase, the stillness gate read that as settled, and the
            # jaws finished closing mid-lift, on air.
            gl = _achieved("jaw_left")
            if gl is None:
                return True
            prev = _jaw_prev[0]
            _jaw_prev[0] = gl
            still = prev is not None and abs(gl - prev) < 2.5e-4
            progressed = gl >= 0.4 * jaw_closed
            return still and progressed
        return True

    # ---- GRAB-POINT AUTO-LOCALISATION ----------------------------------
    # The settle drops the garment ~150 mm and CUDA runs are not reproducible,
    # so where the sleeve ends up varies run to run -- and a hard-coded grab
    # point measured on one settle misses the fabric on the next (measured:
    # jaws reaching their commanded gap EXACTLY, i.e. closing on air, while a
    # fabric column sat 20 mm away). The engine already streams cloth telemetry
    # (OMNISIM_CLOTH_TELEMETRY[_FULL]); the controller reads the latest full
    # particle snapshot at the end of `settle` and re-aims the grab at the
    # MEASURED fabric: gy = median y of the particles in the grab x-column.
    # This is closed-loop perception through a file, which is exactly the
    # perception-as-tool architecture the maze demos use -- not a cheat.
    _tlm_path = os.environ.get("OMNISIM_CLOTH_TELEMETRY", "")
    # ⚠ NEVER in the control arm: autolocate aims at measured fabric, and the
    # control's whole meaning is that it does NOT aim at fabric. Under the
    # friction-only carry this guard was academic (nothing could carry
    # anything), but the particle attach CAN carry -- an autolocated miss run
    # would silently become a second treatment arm.
    _autoloc = (env_float("FOLD_AUTOLOCATE", 1.0) > 0.5 and bool(_tlm_path)
                and not control_miss)

    def _autolocate_grab():
        nonlocal grab_y, target_y, grab_x, grab_z, carry_z
        try:
            import io
            last = None
            with open(_tlm_path, "r", encoding="utf-8") as tf:
                for line in tf:
                    if '"q"' in line:
                        last = line
            if last is None:
                print("vbd_tshirt_fold: autolocate: no full telemetry row yet "
                      "(need OMNISIM_CLOTH_TELEMETRY_FULL=1) -- keeping "
                      "authored grab", flush=True)
                return
            q = json.loads(last)["q"]
            # Prefer the HANGING flap: particles below the tabletop are the
            # overhang itself, and aiming at their centroid is robust to any
            # creep the garment did during settle. Fall back to the on-table
            # column when nothing hangs (flat-grab worlds).
            # 30 mm BELOW the tabletop, not 2: the on-table garment rests at
            # ~0.750 and its bottom layer dips under 0.748, so a threshold that
            # close to the top classifies the WHOLE shirt as "hanging" and aims
            # the grab into the middle of the table (measured: commanded x
            # -0.254 while the flap hung at -0.53).
            hang = [pp for pp in q if pp[2] < 0.72]
            if len(hang) >= 8:
                hx = sorted(pp[0] for pp in hang)[len(hang) // 2]
                hz = max(pp[2] for pp in hang)
                col = hang
                print("vbd_tshirt_fold: autolocate: %d hanging particles, "
                      "grab_x -> %.4f (was %.4f)" % (len(hang), hx, grab_x),
                      flush=True)
                grab_x_new = hx
            else:
                grab_x_new = None
            col = [pp for pp in (hang if len(hang) >= 8 else q)
                   if abs(pp[0] - (grab_x_new if grab_x_new is not None else grab_x)) < 0.035
                   and pp[2] < 0.80]
            if len(col) < 8:
                print("vbd_tshirt_fold: autolocate: only %d particles in the "
                      "grab column -- keeping authored grab" % len(col),
                      flush=True)
                return
            ys = sorted(pp[1] for pp in col)
            gy = ys[len(ys) // 2]
            print("vbd_tshirt_fold: autolocate: %d particles in column, "
                  "grab_y %|.4f -> %.4f (fabric y %.4f..%.4f)"
                  .replace('%|', '%') % (len(col), grab_y, gy, ys[0], ys[-1]),
                  flush=True)
            dy = gy - grab_y
            grab_y = gy
            target_y = target_y + dy
            if grab_x_new is not None:
                grab_x = grab_x_new
            # Z RE-AIM. At 5 VBD iterations the settled sheet sags ~25 mm INTO
            # the tabletop, so the authored grab height closes the jaws over
            # fabric that is below their span (measured on this world:
            # attached=0 with the nearest particle 42.6 mm from a 35 mm-radius
            # attach point). Aim the pad centre just above the column's
            # MEASURED top layer, clamped so the pad bottom never bites more
            # than 3 mm into the table (the servo would press ~50 N per mm).
            zs = sorted(pp[2] for pp in col)
            z_top = zs[(len(zs) * 9) // 10] if len(zs) >= 10 else zs[-1]
            gz = max(z_top + 0.004, TABLE_TOP + PAD_HALF_HEIGHT - 0.003)
            if abs(gz - grab_z) > 0.001:
                print("vbd_tshirt_fold: autolocate: grab_z %.4f -> %.4f "
                      "(fabric top %.4f)" % (grab_z, gz, z_top), flush=True)
                grab_z = gz
                carry_z = grab_z + lift_dz
        except Exception as e:                            # noqa: BLE001
            print("vbd_tshirt_fold: autolocate failed (%r) -- keeping "
                  "authored grab" % (e,), flush=True)

    def _fabric_z(px, py):
        """Median z of the fabric column under the pinch point, from the
        LATEST full telemetry row. The attach must aim at MEASURED fabric:
        the sheet's height depends on iteration count and settle history, and
        a nominal pad-centre z misses the whole column (measured: attached=0,
        nearest particle 42.6 mm away). Returns None when there is no fabric
        in the column -- which is exactly the control arm's case, and None ->
        pad-centre z -> attached=0 is the honest outcome there."""
        if not _tlm_path:
            return None
        try:
            last = None
            with open(_tlm_path, "r", encoding="utf-8") as tf:
                for line in tf:
                    if '"q"' in line:
                        last = line
            if last is None:
                return None
            q = json.loads(last)["q"]
            col = [pp[2] for pp in q
                   if abs(pp[0] - px) < 0.04 and abs(pp[1] - py) < 0.04]
            if len(col) < 4:
                return None
            col.sort()
            return col[len(col) // 2]
        except Exception:                                 # noqa: BLE001
            return None

    sched_step = 0
    gate_hold = 0
    while robot.step(dt_ms) != -1:
        # Gate holds consume WALL steps without advancing the schedule, so the
        # cutoff is: schedule finished AND its last phase fully played out --
        # with an absolute 3x wall cap so a pathological gate cannot run the
        # controller for ever.
        if (sched_step >= total - 1 and step >= total) or step >= 3 * total:
            break

        # Peek: does the next schedule tick cross a phase boundary? If so,
        # hold the schedule clock until the gate for the NEXT phase opens.
        cur_phase = schedule(sched_step)[0]
        nxt_phase = schedule(min(sched_step + 1, total - 1))[0]
        if nxt_phase != cur_phase:
            _, pwx, pwy, pwz, _pj = schedule(sched_step)
            probe_cmd = {
                "gantry_x": clamp_cmd("gantry_x", pwx - root_x, clamped),
                "gantry_y": clamp_cmd("gantry_y", pwy - root_y, clamped),
                "gantry_z": clamp_cmd("gantry_z", pwz - root_z, clamped),
            }
            timeout = int(gate_timeout_mult * max(1, phase_len.get(nxt_phase, 1)))
            if _gate_open(nxt_phase, probe_cmd) or gate_hold >= timeout:
                if gate_hold >= timeout:
                    print("vbd_tshirt_fold: !! gate into '%s' TIMED OUT after %d "
                          "held steps -- advancing anyway" % (nxt_phase, gate_hold),
                          flush=True)
                elif gate_hold > 0:
                    print("vbd_tshirt_fold: gate into '%s' opened after %d held "
                          "steps" % (nxt_phase, gate_hold), flush=True)
                if _autoloc and nxt_phase == "approach":
                    _autolocate_grab()
                # The grab is issued at the moment the close CONVERGED (the
                # lift gate just opened), so the pin lands on fabric already
                # shaped by the pinch; the release is issued at the entry to
                # `release`, before the jaws start opening.
                if nxt_phase == "lift":
                    px, py, pz = _pad_world_pos()
                    fz = _fabric_z(px, py)
                    _attach_send({"op": "attach", "body": attach_body,
                                  "point": [num(px), num(py),
                                            num(fz if fz is not None else pz)],
                                  "radius": attach_radius, "max": attach_max})
                elif nxt_phase == "release":
                    _attach_send({"op": "detach"})
                sched_step += 1
                gate_hold = 0
                _jaw_prev[0] = None
            else:
                gate_hold += 1
        else:
            sched_step = min(sched_step + 1, total - 1)

        phase, wx, wy, wz, jaw = schedule(sched_step)

        cmd = {
            "gantry_x": clamp_cmd("gantry_x", wx - root_x, clamped),
            "gantry_y": clamp_cmd("gantry_y", wy - root_y, clamped),
            "gantry_z": clamp_cmd("gantry_z", wz - root_z, clamped),
            "jaw_left": clamp_cmd("jaw_left", jaw, clamped),
            "jaw_right": clamp_cmd("jaw_right", jaw, clamped),
        }
        for name, value in cmd.items():
            motors[name].setPosition(value)
        _attach_poll_ack()

        if step % log_every == 0 or phase != prev_phase:
            joints = {}
            for name in ("gantry_x", "gantry_y", "gantry_z", "jaw_left", "jaw_right"):
                s = sensors[name]
                # ⚠ `achieved` is a READING or it is null. It is never the
                # commanded value echoed back -- an unmeasured quantity reported
                # as a number is a false belief installed in whatever reads this
                # file, and there is nothing downstream that could detect it.
                got = num(s.getValue()) if s is not None else None
                joints[name] = {
                    "commanded": num(cmd[name]),
                    "achieved": got,
                    "error": None if got is None else num(got - cmd[name]),
                }

            # Pad separation along the closing axis. Each jaw moves its own pad
            # inward by its own joint coordinate, so the separation is the sum of
            # the two remaining offsets.
            gap_cmd = (PAD_REST_OFFSET - cmd["jaw_left"]) + \
                      (PAD_REST_OFFSET - cmd["jaw_right"])
            ql = joints["jaw_left"]["achieved"]
            qr = joints["jaw_right"]["achieved"]
            gap_got = None if (ql is None or qr is None) else \
                (PAD_REST_OFFSET - ql) + (PAD_REST_OFFSET - qr)

            rec = {
                "step": step,
                "t": num(robot.getTime()),
                "phase": phase,
                "joints": joints,
                "gap": {
                    "commanded": num(gap_cmd),
                    "achieved": num(gap_got),
                    # THE LOAD SIGNAL. Positive == the jaws could not reach the
                    # separation they were commanded, i.e. something is between
                    # the pads resisting them. Null when unmeasured, because the
                    # whole point of it is that it is a measurement.
                    "excess": None if gap_got is None else num(gap_got - gap_cmd),
                },
            }
            fh.write(json.dumps(rec) + "\n")

        if phase != prev_phase:
            print("vbd_tshirt_fold: MARK %-9s step=%d t=%.3f cmd=(%.4f, %.4f, %.4f) "
                  "jaw=%.4f" % (phase, step, robot.getTime(), cmd["gantry_x"],
                                cmd["gantry_y"], cmd["gantry_z"], cmd["jaw_left"]),
                  flush=True)
            prev_phase = phase
        step += 1

    fh.close()

    if clamped:
        print("vbd_tshirt_fold: !! CLAMPED %s -- a commanded position was outside the "
              "motor range this file mirrors from the world. Either the waypoint is "
              "unreachable or MOTOR_RANGE here has drifted from the .omniworld."
              % json.dumps({k: [min(v), max(v)] for k, v in clamped.items()}),
              flush=True)

    # ⚠ Try to pause. `run-headless --duration N` is a wall-clock SLEEP, not a
    # progress target, so an engine whose controller has finished keeps stepping
    # flat out until N expires -- on a self-contact cloth world that is the most
    # expensive thing in the tree, running for nothing (measured elsewhere in
    # this family: 12 C of GPU for a run that produced no data). The siblings
    # pause via Supervisor; this world's Robot is not one, so this is a
    # best-effort hook that starts working the day the world says
    # `supervisor TRUE` and costs nothing until then.
    setter = getattr(robot, "simulationSetMode", None)
    if callable(setter):
        try:
            setter(getattr(robot, "SIMULATION_MODE_PAUSE", 0))
        except Exception as e:                                   # noqa: BLE001
            print("vbd_tshirt_fold: could not pause the engine (%s)" % (e,), flush=True)
    else:
        print("vbd_tshirt_fold: cannot pause (this Robot is not a Supervisor) -- size "
              "--duration to the %.2f s schedule, or the engine free-runs the cloth "
              "for the remainder." % (total * dt), flush=True)

    print("vbd_tshirt_fold: wrote %d samples to %s (measured=%s)"
          % (step // log_every + len(bounds), log_path, measured), flush=True)
    print("vbd_tshirt_fold: DONE -- this controller does NOT score the fold. Join this "
          "file with the OMNISIM_CLOTH_TELEMETRY JSONL for the verdict, and compare "
          "against a FOLD_CONTROL_MISS=1 run before believing it.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
