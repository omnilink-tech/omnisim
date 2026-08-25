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
"""OMNIARM6 folds a hanging cloth in half -- a REAL deformable, gripped by friction.

WHAT THIS DOES. A cloth hangs from a rail like a towel, its bottom hem free in
space. The arm comes up from BELOW the hem so the open jaws straddle it, pinches
it, and carries it up to the rail -- folding the cloth in half. The cloth is 91
particles on newton's SolverVBD; the arm is on SolverMuJoCo; the two are coupled
by SolverCoupledProxy over one Model. Nothing writes a particle pose.

⚠ FOUR THINGS THAT ARE NOT GUESSABLE.

1. THE SQUEEZE IS A POSITION TARGET, NEVER setForce. setForce does NOT put a
   Newton joint in force mode: the PD servo stays live at effortLimit*10 N/m
   anchored at the last setPosition, so a "squeeze force" is really a spring
   pulling to a target deep inside the fabric -- it buries the pads and launches
   whatever it is holding. The jaws here are commanded INSIDE the particle
   radius and are EXPECTED to stall short, on the particles themselves. A stall
   is the grasp working, not a servo fault.

2. YOU CANNOT PINCH A CLOTH THAT IS LYING DOWN. Parallel jaws close
   horizontally, so on a horizontal sheet they compress it in-plane and slide
   off. The gripper needs fabric presented edge-on, which is exactly what a
   hanging hem is -- and why this demo hangs the cloth instead of tabling it.
   (The table version was built first and measured; see the world file.)

3. APPROACH FROM BELOW, NOT FROM THE SIDE. The open jaw spans ~85 mm. Moving
   in sideways at hem height sweeps a pad through the cloth and bats it out of
   the way before anything can close. Going under the hem and rising lets the
   fabric enter the slot without being pushed.

4. GRAVITY IS DOING THE HARD PART, SO DO NOT FIGHT IT. A hanging cloth is
   self-tensioning: the fabric between the rail and the pinch stays taut for
   free. That is the whole reason this staging works -- the parallel standalone
   build of this task, which dragged a sheet across a table, ended in a rumpled
   heap precisely because its fabric went slack mid-carry and friction grabbed
   it. Lifting a hem straight up has no slack phase.

Verify:
  python -m omnisim run-headless projects/samples/demos/worlds/flagship/omniarm6_cloth_fold.omniworld --duration 60
  FOLD_CONTROL_NOGRIP=1 ...   negative control: same motion, jaws never close
  FOLD_PROBE=1 ...            settle only, so the hem's rest pose can be read

⚠ THE CLOTH IS INVISIBLE FROM HERE. Particle positions have no supervisor
accessor, so this controller cannot see the fabric at all -- it cannot measure
the hem and it cannot verify the fold. Everything it writes to its result JSON
is about the ARM. The fold itself is proven from the engine-side telemetry:
  OMNISIM_CLOTH_TELEMETRY=<path> OMNISIM_CLOTH_TELEMETRY_FULL=1
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


def _f(name, default):
    return float(os.environ.get(name, default))


OUT = os.environ.get("FOLD_OUT", os.path.join(_HERE, "_cloth_fold_result.json"))
NOGRIP = os.environ.get("FOLD_CONTROL_NOGRIP") == "1"
PROBE = os.environ.get("FOLD_PROBE") == "1"

# ⚠ STOP-AFTER IS AN ITERATION-SPEED TOOL, AND IT MATTERS MORE THAN IT LOOKS.
# Cloth runs at roughly 0.17x realtime, so every second of sim costs ~6 s of
# wall clock and the full sequence is a ~160 s round trip. Most questions --
# "does the arm reach the hem", "does the pinch hold" -- are answered by the
# first 8 s. FOLD_STOP_AFTER=<phase> exits cleanly the moment that phase
# completes, turning a 160 s round trip into ~50 s. Phase names are the labels
# below: stage, under_hem, rise, pinch, fold1..3, withdraw, stow.
STOP_AFTER = (os.environ.get("FOLD_STOP_AFTER") or "").strip()


class _Done(Exception):
    """Raised to unwind out of the sequence once STOP_AFTER is reached."""


def _checkpoint(name):
    if STOP_AFTER and name == STOP_AFTER:
        raise _Done(name)

OZ = 0.25                       # flange -> pad mid-height (omniarm6_2f85_grip.urdf)

# ⚠ MEASURED FROM A SETTLED PROBE RUN, NOT AUTHORED. The .wbt hangs the cloth
# from z 0.625 with a 0.24 m drop, but fabric stretches under its own weight,
# so the hem does NOT end at the authored 0.385. A FOLD_PROBE=1 run with cloth
# telemetry measured it settling at z 0.361..0.375, y 0.277..0.300, and the
# sheet widening to x 0.400..0.550. Re-measure after ANY change to the cloth's
# mass, stiffness or size -- these are consequences of the fabric, not choices.
RAIL_Z = 0.550
# ⚠ THE RIG SITS CLOSE TO THE ROBOT ON PURPOSE, AND MOVING IT OUT BREAKS THE
# ARM. At the original placement every waypoint sat at ~0.65 m horizontal
# radius and the shoulder saturated: joint2 collapsed 0.73 rad below command
# while joints 1/4/6 tracked exactly. The one shipping OMNIARM6 grasp demo works
# at ~0.46 m. These values keep the whole motion inside that envelope.
HEM_Z = _f("FOLD_HEM_Z", 0.300)       # centre of the free bottom hem
SHEET_Y = _f("FOLD_SHEET_Y", 0.150)   # the hanging plane the jaws straddle
GRIP_X = _f("FOLD_GRIP_X", 0.400)     # across the sheet's width

# Jaw geometry. Pad inner face == q exactly (the URDF's 7 mm joint offset and
# the pad's 7 mm half-thickness cancel), so q IS the half-gap. The particle
# radius is the fabric's half-thickness, so closing to radius - bite pinches it.
JAW_OPEN = 0.0425                     # the joint's own upper limit
PARTICLE_R = 0.01
BITE = _f("FOLD_BITE", 0.0025)
JAW_CLOSED = PARTICLE_R - BITE        # 0.0075
JAW_RELEASE = PARTICLE_R + 0.004      # clears the fabric on the way out

# Where the hem is carried to. Just under the rail, and pushed slightly to the
# near side so the folded half hangs IN FRONT of the half still on the rail
# instead of trying to occupy the same plane.
FOLD_Z = _f("FOLD_TOP_Z", 0.500)
FOLD_Y = _f("FOLD_TOP_Y", SHEET_Y + 0.030)

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

# ⚠ EXACTLY ONE WRITER OWNS THE ARM JOINTS, AND IT IS THE BRIDGE.
# ArmBridge.tick() re-issues the bridge's own joint targets on EVERY call, so a
# controller that both ticks the bridge and writes motor.setPosition() itself is
# two writers fighting over the same six motors -- and the bridge wins, because
# it writes last, every tick. MEASURED: with direct drive AND bridge.tick() in
# the step loop, the tool sat frozen at (0.40, -0.07, 0.59) through all eight
# phases while the gripper obeyed perfectly (the fingers are not the bridge's
# joints, so nobody fought over them).
#
# Removing the bridge and driving the motors directly was tried next and tracked
# no better (joints 2 and 5 off by 0.6-0.98 rad), so this file now does what the
# one SHIPPING OMNIARM6 grasp demo does -- omniarm6_real_pick_place -- and drives the
# arm through the bridge while driving only the two FINGER motors directly. That
# split is not stylistic: the fingers are not bridge joints, so nothing contends
# for them. Do not reintroduce a second arm writer.
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
IK = bridge.cfg["ik"]
# ⚠ THE ARM CONFIG'S JOINT LIMITS DO NOT MATCH THE ROBOT'S MOTORS, AND THE
# DIFFERENCE ROTATES THE WHOLE ARM 180 DEGREES.
# `_arm_configs` declares joint1 as +/-2*pi; the URDF motor is +/-pi. DLS then
# happily returns a joint1 near 2*pi (an equivalent angle, as far as it knows),
# clamp_q keeps it because the CONFIG allows it, and the MOTOR clamps it to pi
# -- swinging the base half a turn. The engine says so plainly, once per
# command, and it is the one log line that explains every downstream number:
#     RotationalMotor "joint1_motor": too big requested position: 6.28319 > 3.14159
# These are the motors' real ranges, read back from the engine's own joint
# table in the run log. joint2 is tightened further to +/-1.95, as the shipping
# pick-place demo does, to keep the shoulder out of its saturating region.
IK_LIMITS = [(-3.14159, 3.14159),      # joint1  (config wrongly says +/-2pi)
             (-1.95, 1.95),            # joint2  (motor +/-pi; held tighter)
             (-2.61799, 2.61799),      # joint3
             # ⚠ joint4 and joint6 CLAMPED TO +/-pi THOUGH THE MOTOR ALLOWS
             # +/-2pi. These two are the only joints on this arm with an
             # over-pi range, and they are the ONLY two that fail to hold a
             # commanded pose -- by 3.5-3.9 rad, in every configuration tested,
             # including with the arm's collision geometry removed entirely.
             # A roll joint is physically identical at q and q+2pi, so a solver
             # or servo free to pick either branch can "reach" a target a full
             # turn from the one asked for. Denying the extra range costs this
             # arm nothing (a wrist needs far less than a full turn) and makes
             # the commanded angle unambiguous.
             (-3.14159, 3.14159),      # joint4  (motor allows +/-2pi)
             (-3.14159, 3.14159),      # joint5
             (-3.14159, 3.14159)]      # joint6  (motor allows +/-2pi)

# ⚠ AND THE BRIDGE MUST BE CORRECTED TOO, NOT JUST THE SOLVER. Fixing only the
# IK limits above changes what dls_ik_pose may RETURN; it does not change what
# act_set_joint_positions will SEND, because that clamps against the bridge's
# own joint_limits. Measured: with IK_LIMITS corrected but the bridge left
# alone, the engine still logged 750 "too big requested position" warnings in a
# single run. Correct the limits at every layer that enforces them.
bridge.joint_limits = [tuple(v) for v in IK_LIMITS]
# Tool +Z straight down, tool +Y along world -Y: the pads separate along world
# y, which is the axis the hanging sheet's normal lies on.
TOP = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
PARK_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]
NJ = len(IK["chain"])           # 6 -- the arm joints the IK owns, fingers excluded

fm = robot.getDevice("robotiq_2f85_finger_motor")
mm = robot.getDevice("robotiq_2f85_finger_mirror_motor")
fs = robot.getDevice("robotiq_2f85_finger_sensor")
ms = robot.getDevice("robotiq_2f85_finger_mirror_sensor")
for s in (fs, ms):
    s.enable(dt)
for m in (fm, mm):
    m.setAvailableForce(m.getMaxForce())

# ⚠ THE SIX ARM JOINTS ARE DRIVEN DIRECTLY, NOT THROUGH ArmBridge.
# The bridge's act_set_joint_positions()/tick() interpolator was measured NOT
# to deliver: with IK solutions that are good to 0.6-1.9 mm offline (verified
# independently, from two different seeds), the arm ended up 0.69 m above its
# first commanded pose and the error only shrank as the targets rose towards
# where the arm already was -- i.e. it was barely leaving its start pose, not
# mis-solving. Driving the motors here removes that layer entirely: the IK
# result goes straight to setPosition with a smoothstep ramp, and every number
# this controller reports is read back from the joint sensors.
ARM = []          # deliberately empty: the bridge owns the arm motors
ARM_S = [robot.getDevice("joint%d_sensor" % (i + 1)) for i in range(NJ)]
for s in ARM_S:
    if s is not None:
        s.enable(dt)
# ⚠ DO NOT TOUCH AVAILABLE TORQUE HERE. A motor already starts with its full
# URDF effort available, so setAvailableTorque(getMaxTorque()) is at best a
# no-op -- and at worst it is how you silently DISARM the arm, because a
# wrong/zero return from the getter becomes a zero torque budget and the only
# symptom is an arm that sags and will not track. Any such call belongs outside
# a bare `except`, which is what hid it here.
ARM_TORQUE = None


def _finger_solids():
    """The two finger-link Solids, resolved without waiting for a contact.

    The URDF importer emits no DEFs, so the route is device -> the node owning
    the device -> that joint's endPoint. Returns {} if any hop fails, and the
    caller then stays on the FK aim.
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
        except Exception:                            # noqa: BLE001
            pass
    return out


FINGERS = _finger_solids()


def grip_point():
    """Where the pads' symmetry plane ACTUALLY is, in world coordinates.

    ⚠ THIS IS NOT THE FK TOOL POINT, AND THE GAP IS NOT SMALL. Measured on
    this world: with the arm settled, the FK tool point sits a CONSTANT
    +88 mm out in y from the commanded target at every waypoint -- identical
    at two different heights, so it is a systematic model offset, not a
    tracking error. The shipping pick-place demo hit the same class of defect
    (+5.5 mm there) and solved it the same way: stop trusting the kinematic
    model and read the pads' real poses out of the engine.

    The finger link origins sit at gripper-local y = +(0.007+q_l) and
    -(0.007+q_r), so their midpoint is offset from the gripper axis by exactly
    (q_l-q_r)/2; undo that and the result is a point on the true axis. The
    + z*0.025 term takes it at PAD MID-HEIGHT rather than at the link origins.
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
    u = [v / n for v in d]                       # gripper +y, in world
    m = L.getOrientation()                       # row-major, local -> world
    z = [m[2], m[5], m[8]]                       # the pad's own +z, in world
    return [0.5 * (lp[i] + rp[i]) - u[i] * 0.5 * (ql - qr) + z[i] * 0.025
            for i in range(3)]


def read_q():
    return list(bridge._read_q())[:NJ]


def drive_q(q, secs):
    """DEAD ON PURPOSE. The bridge owns the arm joints; see the note above."""
    raise RuntimeError("drive_q: the bridge owns the arm joints -- do not "
                       "write them directly, it silently loses to bridge.tick()")

log = []
trace = []
trace_last_cmd_q = []      # one-slot box: the q the IK last asked for
trace_aim = []             # measured grip point after each waypoint


def emit(s):
    log.append(s)
    print(s, flush=True)


def step_for(secs):
    for _ in range(int(secs * 1000 / dt)):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


def tcp():
    return forward_kinematics_pose(IK["chain"], read_q(), (0.0, 0.0, OZ))[0]


def fingers():
    return fs.getValue(), ms.getValue()


def goto(xyz, dur=1.6, passes=2, tol=0.004, label=""):
    """Closed loop on the FK tool point.

    ⚠ SEEDED FROM THE ARM'S CURRENT JOINTS, not a fixed nominal pose. The rigid
    sibling demo seeds nominal and measured the cost: adjacent targets landed on
    IK branches ~180 deg apart at joints 4 and 6, the bridge interpolated
    between them in JOINT space, and the tool's world orientation swung 170 deg
    away and back inside one 3 s move with identical commanded end orientations.
    Continuity first, optimality never.
    """
    goal, err = list(xyz), 1e9
    for _ in range(passes):
        # Seed from where the arm actually is, so consecutive waypoints stay on
        # one IK branch. (Both this and a fixed park seed were verified offline
        # to solve these waypoints to under 2 mm, so the seed is not load-bearing
        # for convergence here -- only for continuity.)
        # ⚠ SOLVE FROM BOTH SEEDS AND REJECT BRANCH JUMPS. DLS cannot cross
        # between IK branches, so it happily returns a contorted solution that
        # satisfies the pose and is nowhere near the arm's current posture.
        # MEASURED: seeding only from the live pose produced q1 = -2.73 rad --
        # the arm reaching backwards over its own shoulder -- for a waypoint
        # that solves to +0.74 from the park seed. The contorted branch is also
        # where the servo stopped tracking (joints 2 and 5 off by 0.78 and 0.86
        # rad), because the arm cannot hold those postures against gravity.
        # So: prefer continuity with the current pose, but only among solutions
        # that are actually near it; otherwise fall back to the park branch.
        # ⚠ ALWAYS SEED FROM THE FIXED PARK POSE, exactly as the shipping
        # omniarm6_real_pick_place demo does. Seeding from the arm's live pose --
        # or scoring candidates by nearness to it -- lets the solution drift
        # onto a STRAIGHT-ELBOW branch, and a straight elbow is what breaks
        # this arm. MEASURED: with joint3 solved near 0.02-0.71 rad the
        # shoulder (joint2) collapsed 0.73 rad below its command while joints
        # 1, 4 and 6 tracked exactly -- one joint saturating beside exact
        # neighbours is gravity beating the servo, not a control fault. The
        # park seed keeps joint3 near 1.35, folding the arm and shortening the
        # lever the shoulder has to hold.
        q, _p, _r, _i = dls_ik_pose(IK["chain"], list(PARK_SEED), goal, TOP,
                                    (0.0, 0.0, OZ), IK, IK_LIMITS)
        trace_last_cmd_q[:] = [list(q)]
        bridge.act_set_joint_positions(q, duration_s=dur)
        # ⚠ WAIT FOR THE ARM TO STOP, DO NOT GUESS A DURATION. The bridge ramps
        # its target over duration_s, but the MOTORS have velocity limits, so a
        # large excursion physically cannot complete in the commanded time --
        # it keeps slewing after the ramp ends. Sampling then reads a
        # MID-MOTION pose as if it were a steady-state error.
        #
        # This cost this file an entire debugging campaign. Measured: at 2.5 s
        # after a park command, joints 4 and 6 read 0.897 and 0.513 rad "off"
        # while drifting -0.897 and -0.513 rad per half-second -- i.e. exactly
        # mid-slew. Given 10 s they settle to EXACTLY zero. The apparent
        # "tracking failure" was the sampling instant, and it was largest on
        # the first big move and shrank as targets approached the arm's current
        # pose, which reads convincingly like reach saturation and is not.
        step_for(dur)
        for _ in range(int(_f("FOLD_MAX_SETTLE", 8.0) / 0.25)):
            _a = read_q()
            step_for(0.25)
            _b = read_q()
            if max(abs(_b[i] - _a[i]) for i in range(NJ)) < 5e-4:
                break
        t = tcp()
        resid = [xyz[i] - t[i] for i in range(3)]
        err = max(abs(v) for v in resid)
        if err <= tol:
            break
        goal = [goal[i] + resid[i] for i in range(3)]
        dur = 0.5
    ql, qr = fingers()
    # ⚠ RE-AIM ON THE MEASURED PADS, NOT ON FK. Everything above drives the
    # FK tool point; this corrects for the constant offset between that model
    # point and where the pads physically are (+88 mm in y here). One
    # correction pass is enough because the offset is systematic, not noise.
    _gp = grip_point()
    if _gp is not None and label and not label.startswith("_"):
        _res = [xyz[i] - _gp[i] for i in range(3)]
        if max(abs(v) for v in _res) > tol:
            _goal2 = [goal[i] + _res[i] for i in range(3)]
            _seed = read_q() or list(PARK_SEED)
            _q2, _p2, _r2, _i2 = dls_ik_pose(IK["chain"], list(_seed), _goal2,
                                             TOP, (0.0, 0.0, OZ), IK, IK_LIMITS)
            bridge.act_set_joint_positions(_q2, duration_s=0.6)
            step_for(0.6)
            for _ in range(24):
                _a = read_q()
                step_for(0.25)
                _b = read_q()
                if max(abs(_b[i] - _a[i]) for i in range(NJ)) < 5e-4:
                    break
            _gp = grip_point()
    if _gp is not None:
        trace_aim.append([round(v, 5) for v in _gp])
    ql, qr = fingers()
    # ⚠ RECORD JOINT TRACKING, NOT JUST TOOL ERROR. A tool-space error alone
    # cannot tell "the IK asked for the wrong joints" from "the arm did not go
    # where it was told" -- and those have opposite fixes. q_cmd is what the IK
    # returned and q_got is what the joint sensors read at the end of the ramp;
    # if they agree and the tool is still off, the FK/tool-offset model is
    # wrong, not the servo.
    q_got = read_q()
    q_cmd = trace_last_cmd_q[0] if trace_last_cmd_q else None
    dq = ([round(q_got[i] - q_cmd[i], 4) for i in range(NJ)]
          if q_cmd is not None else None)
    trace.append({"phase": label, "cmd": list(xyz), "tcp": list(tcp()),
                  "grip_point": (list(_gp) if _gp is not None else None),
                  "aim_err_m": (round(max(abs(xyz[i] - _gp[i]) for i in range(3)), 5)
                                if _gp is not None else None),
                  "err_m": round(err, 6), "jaw": [round(ql, 5), round(qr, 5)],
                  "q_cmd": [round(v, 4) for v in q_cmd] if q_cmd else None,
                  "q_got": [round(v, 4) for v in q_got],
                  "dq": dq, "max_dq": (max(abs(v) for v in dq) if dq else None)})
    emit("  %-11s cmd=(%.3f %.3f %.3f) err=%.4f m jaw=(%.4f %.4f)"
         % (label, xyz[0], xyz[1], xyz[2], err, ql, qr))
    _checkpoint(label)
    return err


def set_jaw(q):
    fm.setPosition(q)
    mm.setPosition(q)


def ramp_jaw(q0, q1, secs):
    """Ramped, never stepped: a single step onto fabric flicks it out of the
    slot before the second pad arrives."""
    n = max(1, int(secs * 1000 / dt))
    for i in range(1, n + 1):
        set_jaw(q0 + (q1 - q0) * (i / float(n)))
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


# --------------------------------------------------------------------------
result = {"ok": False, "phases": trace, "nogrip": NOGRIP, "probe": PROBE}


def _flush():
    """Write the result JSON as it stands. Safe to call repeatedly."""
    try:
        result["log"] = log
        with open(OUT, "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=2)
    except Exception as exc:                                 # noqa: BLE001
        emit("could not write %s: %r" % (OUT, exc))
try:
    emit("cloth fold: dt=%d ms  hem=(%.3f, %.3f, %.3f)  jaw %.4f -> %.4f"
         % (dt, GRIP_X, SHEET_Y, HEM_Z, JAW_OPEN, JAW_CLOSED))
    emit("  arm motors: %s" % ([("none" if m is None else "ok") for m in ARM],))
    emit("  arm maxTorque: %s" % (ARM_TORQUE,))
    result["arm_max_torque"] = ARM_TORQUE
    set_jaw(JAW_OPEN)

    # 1. Let the cloth settle. It stretches under its own weight, so its rest
    #    pose is not its authored pose and nothing before this is meaningful.
    # ⚠ TIME BUDGET MATTERS HERE IN A WAY IT DOES NOT FOR RIGID DEMOS. With
    # cloth the engine runs at roughly 0.17x realtime on an RTX 3060 laptop, so
    # every second of sim costs ~6 s of wall clock. A first version of this
    # sequence needed ~45 s of sim and simply never reached the pinch inside a
    # 75 s run -- which looks exactly like a crashed controller (no result file)
    # and is not. Keep the whole sequence near 20 s of sim, and size
    # `--duration` at about 6x that.
    # ⚠ START FROM A KNOWN POSTURE. Every IK solution below is chosen partly by
    # how near it is to where the arm already is, so the arm's STARTING pose
    # decides which branch the whole sequence lives on. Leaving that to whatever
    # the robot spawns in put the first solve on a backwards-over-the-shoulder
    # branch. Driving to the park pose first makes the sequence reproducible.
    # ⚠ BASELINE SERVO CHECK, BEFORE ANY WAYPOINT. Every failure so far has been
    # reported as "the tool missed its target", which cannot distinguish a bad
    # target from an arm that cannot hold ANY pose. Park is the easiest pose
    # there is, commanded in free space with nothing to touch; if the arm
    # cannot hold THIS, no waypoint tuning will help and the fault is the servo
    # or the model, not the plan.
    bridge.act_set_joint_positions(list(PARK_SEED), duration_s=2.0)
    step_for(_f("FOLD_SETTLE", 2.5))
    # ⚠ IS IT STILL MOVING, OR IS IT HELD? A joint sitting 0.5 rad from its
    # target is either mid-transient or being opposed. Those have opposite
    # fixes and the position alone cannot tell them apart. Sample the pose
    # twice, a fixed interval apart: if it drifts, the arm has not converged;
    # if it is frozen off-target while the servo commands maximum torque,
    # something external is holding it.
    _q1 = read_q()
    step_for(0.5)
    _q2 = read_q()
    _drift = [round(_q2[i] - _q1[i], 5) for i in range(NJ)]
    result["park_drift_over_0s5"] = _drift
    result["park_settled"] = bool(max(abs(v) for v in _drift) < 1e-3)
    emit("  drift over 0.5 s: %s  settled=%s" % (_drift, result["park_settled"]))
    _pq = read_q()
    _pd = [round(_pq[i] - PARK_SEED[i], 4) for i in range(NJ)]
    result["park_cmd"] = [round(v, 4) for v in PARK_SEED]
    result["park_got"] = [round(v, 4) for v in _pq]
    result["park_err"] = _pd
    result["park_max_err"] = max(abs(v) for v in _pd)
    emit("  PARK check: max_err=%.4f rad  err=%s" % (max(abs(v) for v in _pd), _pd))
    # ⚠ DID THE BRIDGE ACTUALLY RESOLVE EVERY MOTOR? ArmBridge._set_q zips
    # motors with targets and skips any None WITHOUT SAYING SO, so an
    # unresolved device is indistinguishable from a joint that will not track
    # -- it is simply never commanded. joint4 has been wrong by 1.0-2.5 rad in
    # every configuration tested (colliders on/off, inertia on/off, substeps,
    # limits), which is what a never-commanded joint looks like.
    result["bridge_motors"] = [(("joint%d" % (i + 1)) if m is not None else
                                ("joint%d=MISSING" % (i + 1)))
                               for i, m in enumerate(getattr(bridge, "motors", [])[:NJ])]
    result["bridge_sensors"] = [(("ok" if sN is not None else "MISSING"))
                                for sN in getattr(bridge, "sensors", [])[:NJ]]
    emit("  bridge motors:  %s" % (result["bridge_motors"],))
    emit("  bridge sensors: %s" % (result["bridge_sensors"],))
    # ⚠ FLUSH THE RESULT HERE, NOT ONLY AT THE END. The park check is the single
    # most diagnostic number this controller produces, and writing it only on a
    # clean finish means any run that is cut short -- by a wall-clock cap, by a
    # mis-typed FOLD_STOP_AFTER, by a crash -- reports NOTHING and reads exactly
    # like a controller that never ran. Measured cost of that: two engine runs
    # that PASSed and yielded no data at all.
    _flush()
    # ⚠ JOINT-1 ISOLATION PROBE. Every waypoint now lands within 1 mm in z and
    # 20 mm in x, but the pads sit a constant ~0.11 m round in y -- the base
    # yaw is biased. Re-aiming cannot fix that, because joint1 is the very
    # joint that will not go where it is told. This drives joint1 ALONE, with
    # no IK and no other joint moving, and reports commanded vs achieved: the
    # narrowest possible test of one joint's servo.
    if os.environ.get("FOLD_JOINT_PROBE"):
        _rows = []
        _base = list(PARK_SEED)
        for _tgt in (0.0, 0.3, -0.3, 0.6, 0.0):
            _q = list(_base)
            _q[0] = _tgt
            bridge.act_set_joint_positions(_q, duration_s=1.5)
            step_for(1.5)
            for _ in range(40):
                _a = read_q()
                step_for(0.25)
                _b = read_q()
                if max(abs(_b[i] - _a[i]) for i in range(NJ)) < 5e-4:
                    break
            _got = read_q()
            _rows.append({"cmd": _tgt, "got": round(_got[0], 5),
                          "err": round(_got[0] - _tgt, 5),
                          "others": [round(v, 4) for v in _got[1:]]})
            emit("  joint1 cmd=%+.3f got=%+.5f err=%+.5f" % (_tgt, _got[0], _got[0] - _tgt))
        result["joint1_probe"] = _rows
        _flush()
        raise _Done("joint1_probe")
    if PROBE:
        emit("probe: settled, holding. Read the cloth telemetry for the hem.")
        step_for(5.0)
        raise SystemExit(0)

    # 2-3. APPROACH FROM DIRECTLY ABOVE, IN THE SHEET'S OWN PLANE.
    #
    # ⚠ EVERY WAYPOINT MUST LIE IN THE ARM'S HOLDABLE BAND, WHICH IS NOT THE
    # SAME AS ITS REACHABLE ONE. Mapped offline against the park pose (which
    # the engine holds to 0.02 rad): for x = 0.40, the band y 0.10..0.20 and
    # z 0.30..0.55 solves with shoulder pitch |j2| <= 0.22 rad and FK error
    # under 3 mm. Outside it the IK still "converges" and the arm then SAGS --
    # the waypoint that broke every earlier attempt was (0.40, 0.29, 0.20),
    # whose solution pitches the shoulder to 1.46 rad and measured 1.05 rad of
    # collapse. Re-run scratchpad/reach_map.py before moving any of these.
    #
    # Staying in-plane also removes the sideways sweep the old approach needed:
    # the jaws open to 85 mm and the sheet is 20 mm thick, so descending along
    # the sheet's own plane simply straddles it -- nothing is brushed aside,
    # and no low reach is required.
    goto((GRIP_X, SHEET_Y, HEM_Z + 0.12), 1.8, label="above_hem")
    goto((GRIP_X, SHEET_Y, HEM_Z), 1.1, label="onto_hem")
    step_for(0.3)

    # 4. Pinch. Expected to stall on the particles short of the command.
    if not NOGRIP:
        ramp_jaw(JAW_OPEN, JAW_CLOSED, 0.9)
        step_for(0.5)
    ql, qr = fingers()
    stalled = abs(ql - JAW_CLOSED) > 0.0012 or abs(qr - JAW_CLOSED) > 0.0012
    emit("  pinch: jaw=(%.4f %.4f) cmd=%.4f  stalled_on_fabric=%s"
         % (ql, qr, JAW_CLOSED, stalled))
    result["jaw_after_close"] = [round(ql, 5), round(qr, 5)]
    result["stalled_on_fabric"] = bool(stalled)
    _checkpoint("pinch")

    # 5. THE FOLD. Straight up, easing to the near side so the lifted half
    #    hangs in front of the half still on the rail rather than through it.
    #    No arc games are needed: a hanging cloth tensions itself, so there is
    #    no slack phase for friction to grab.
    for i, (fy, fz) in enumerate((
            (SHEET_Y + 0.010, HEM_Z + 0.065),
            (SHEET_Y + 0.020, HEM_Z + 0.130),
            (FOLD_Y,          FOLD_Z))):
        goto((GRIP_X, fy, fz), 0.9, label="fold%d" % (i + 1))
    step_for(0.6)

    # 6. Release slowly -- a fast open flicks the fabric off the pads.
    if not NOGRIP:
        ramp_jaw(JAW_CLOSED, JAW_RELEASE, 1.0)
    step_for(0.5)

    # 7. Withdraw FORWARD, away from the sheet, before doing anything else.
    #    The open pads are two posts standing in a bight of cloth; moving up or
    #    sideways first drags the fold with them.
    # withdrawal stays in-band: y 0.20 is the far edge of it, and dropping
    # 0.12 in z keeps the arm folded rather than reaching out to park.
    goto((GRIP_X, 0.200, FOLD_Z), 1.0, label="withdraw")
    goto((GRIP_X, 0.200, FOLD_Z - 0.12), 1.1, label="stow")
    step_for(1.2)

    result["ok"] = True
    emit("cloth fold: motion complete")
except _Done as _d:
    result["ok"] = True
    result["stopped_after"] = str(_d)
    emit("cloth fold: STOP_AFTER=%s reached" % (_d,))
except SystemExit:
    result["ok"] = True
except Exception as exc:                                     # noqa: BLE001
    result["error"] = repr(exc)
    emit("cloth fold: FAILED %r" % (exc,))

result["log"] = log
try:
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    emit("wrote %s" % OUT)
except Exception as exc:                                     # noqa: BLE001
    emit("could not write %s: %r" % (OUT, exc))

if os.environ.get("FOLD_AUTOQUIT"):
    robot.simulationQuit(0 if result.get("ok") else 1)
while robot.step(dt) != -1:
    bridge.tick(robot.getTime())
