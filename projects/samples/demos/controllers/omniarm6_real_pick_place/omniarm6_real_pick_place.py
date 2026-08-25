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
"""OMNIARM6 real pick & place -- the block is held by CONTACT FRICTION, nothing else.

WHY THIS CONTROLLER EXISTS. Every other arm demo in this tree grips through
ArmBridge.act_grasp(), which by default welds the nearest DEF GRASP_* node to
the tool and teleports it to the TCP every tick. That hold is unfalsifiable: it
would work with the fingers wide open, for ever. This controller never calls
act_grasp, never writes the block's pose, and never creates a weld. If the
contact physics stops holding, the block falls.

THE CONTROL LAW. Pre-close by position to 1 mm of clearance, then ramp the
position target 3 mm PAST each block face. The engine builds these joints as a
PD servo with kp = 500 N/m (measured in the compiled model), so 3 mm of
interference is 1.5 N per pad; at mu 6 that is ~9 N of friction per pad against
a 1.47 N block.

⚠ NOT setForce, despite what docs/guide/friction-grasp.md says. setForce does
not put a Newton joint in force mode -- the PD servo stays live at
effortLimit*10 N/m anchored at the last setPosition, so a "28 N squeeze" is
really a spring pulling to a target ~20 mm inside the part. It buries the pads,
stores the interference, and launches the part when the arm lifts. That is
exactly how friction_grasp_minimal "holds": the part is ejected at 3.5 m/s and
lands on the gripper's own wrist plate. OMNISIM_NEWTON_TORQUE_MODE=1 is the
real force mode; until a demo sets it, position control is the honest lever.

⚠ THE "7 mm STALL ASYMMETRY" WAS NOT A JOINT DEFECT -- IT WAS THE AIM. Earlier
revisions of this file blamed the mirrored prismatic joint for the two fingers
stalling ~7 mm apart on identical targets (left 0.0201, right 0.0287). Measured
against the engine's own Solid poses, the joints are honest: the separation of
the two finger link ORIGINS matches 0.014+q_l+q_r to 0.6 mm at every sample, and
both pads are genuinely on the block (3 contacts each, 0.03-1.02 mm deep). What
is wrong is the FK tool point this demo aims with: it sits a CONSTANT +5.5 mm
out in y from the real gripper axis (5.66 / 5.73 / 5.45 mm at three unrelated
arm poses). So a descent FK calls centred to 0.2 mm leaves the gripper 4-5 mm
off the block, the pads close asymmetrically around it, and the block then
slides that same distance to re-centre itself once airborne -- which is exactly
what the old "3.1 mm carry drift" was. goto() now re-aims on grip_point(), the
pad midplane read off the finger link Solids, and the asymmetry goes from
8.6 mm to 0.12 mm. ⚠ The FK error itself is NOT fixed -- see grip_point().

⚠ AND CENTRING THE PADS EXPOSED A SECOND, PHYSICAL DEFECT. With the block
off-centre the grasp survived the carry; centred, it did not -- the block
rotated inside the gripper and levered the pads from 24 mm apart to 41 mm,
arriving tipped (z=0.225 instead of 0.245), identically on 3 of 3 runs. Root
cause: every OmniSim contact was condim 3, i.e. sliding friction only, so a
pinched part spins about the contact normal at zero cost. The world now
declares `newtonCondim 4` (torsional friction), which holds it.

AIMING. Open-loop DLS IK lands 1.5-2.4 cm out at grasp height, which is fatal
when the pad clearance is 1 mm. goto() closes the loop on the FK-measured tool
point: solve, move, measure the residual, re-solve for target-minus-residual.
Measured 2.4 cm -> 0.5 mm. This uses no sensing the arm does not already have.

HONESTY. The verdict is geometric and adversarial: the block must be AIRBORNE
(clear of both tables) and CO-MOVING (its offset from the tool must stay fixed
while the tool travels). PICK_CONTROL_DROP=1 runs the negative control --
identical motion with no squeeze -- and the block must be left behind; a "pass"
there would mean the rig is holding it by something other than the grip.

⚠ AND AIRBORNE + CO-MOVING IS STILL NOT ENOUGH: it proves the RIG holds the
block, not that the PADS do. The block is 90 mm tall against 50 mm pads, so
20.5 mm of it sticks up past the gripper's palm face and a "hold" could be a
palm wedge with the fingers along for the ride -- which is exactly how the
tree's CI-gated friction_grasp_minimal "holds" (its part is ejected upward and
lands on the gripper's wrist plate). census() therefore asserts CONTACT: both
pads must have points on the block's own +/-y grasp faces while it is off the
table. Measured on the shipped configuration: 4 contacts per pad at the
squeeze and the lift, 2 per pad through the carry, peak penetration 1.00 mm,
and ZERO palm contacts at any sample.

⚠ Two things make that assertion possible, and neither existed before this
revision. (1) getContactPoints used to publish body-LOCAL support points as
world coordinates with depth hard-coded 0 -- the block "touched" link4 half a
metre away -- fixed engine-side in the same change. (2) The node_id on a
ContactPoint names the QUERIED side, not the other body, so attributing a
contact to a LINK needs a second query against the robot's subtree; census()
matches the two by point identity.

⚠ A PALM WEDGE IS ALSO STRUCTURALLY IMPOSSIBLE HERE, AND THAT IS ITSELF A BUG.
OMNISIM_NEWTON_DUMP_MJMODEL shows 16 geoms and no gripper palm among them: the
robotiq_2f85_base_link's 60x80x60 mm collision box is DROPPED when the fixed
joint merges it into link6 (the merged body carries the summed mass, 1.50264 kg
= 0.577639 + 0.925, but only link6's cylinder). So the palm cannot collide with
anything, in this world or any other URDF robot with a fixed-joint collider.
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

OUT = os.environ.get("PICK_OUT", os.path.join(_HERE, "_real_pick_result.json"))
CONTROL_DROP = os.environ.get("PICK_CONTROL_DROP") == "1"
DEBUG = os.environ.get("PICK_CONTACT_DEBUG") == "1"

OZ = 0.25                       # flange -> finger throat (omniarm6_2f85_grip.urdf)
BLOCK_HALF = 0.025              # 50 mm across the grasp axis
CLEARANCE = 0.001               # pre-close to 1 mm off each face
# Swept 3 / 6 / 10 mm. Both pads stall at the SAME place regardless of the
# target (left ~0.0201, right ~0.0288), so this does not set the clamp force --
# it sets how hard the servo keeps pushing after the stall. 3 mm placed the
# block tipped (z=0.225); 10 mm places it standing on target (z=0.245) with
# 3.1 mm of carry drift. Overridable for sweeps.
INTERFERENCE = float(os.environ.get("PICK_INTERFERENCE", "0.010"))
TABLE_TOP = 0.20

# ⚠ PAD FACE GEOMETRY, and the 7 mm error that made the first version of this
# demo a lie. omniarm6_2f85_grip.urdf puts the finger joint origin at y = 0.007 and
# the pad box (14 mm thick in y) centred on its own link origin, so the pad's
# INNER face sits at exactly y = q -- the joint offset and the pad half-thickness
# cancel. Writing q = BLOCK_HALF - 0.007 + clearance (i.e. treating 0.007 as an
# extra offset) buries the pad 7 mm INTO the part before any force is applied.
# Measured with that bug: left pad 5.1 mm inside the block, right pad 0.9 mm
# clear -- a one-sided interpenetrating hold that still passed a carry test.
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
# implements, so use it honestly: kp = 500 N/m on these fingers (measured in the
# compiled model), so 3 mm of interference is 1.5 N per pad. At mu 6 that yields
# ~9 N of friction per pad against a 1.47 N block -- a 12x margin without ever
# driving through the part.
GRIP_KP = 500.0                 # measured: compiled model kp for the fingers

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)

IK = bridge.cfg["ik"]
IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]
TOP = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]

block = robot.getFromDef("BLOCK")
omniarm6 = robot.getFromDef("OMNIARM6")

# Block half-extents (Box 0.05 0.05 0.09) -- used to name the face a contact
# landed on, which is the whole point of the census below.
BHALF = (0.025, 0.025, 0.045)

# Drive the finger motors DIRECTLY. The gripper effector only knows how to push
# position targets, which is exactly what does not grip.
fm = robot.getDevice("robotiq_2f85_finger_motor")
mm = robot.getDevice("robotiq_2f85_finger_mirror_motor")
fs = robot.getDevice("robotiq_2f85_finger_sensor")
ms = robot.getDevice("robotiq_2f85_finger_mirror_sensor")
for s in (fs, ms):
    s.enable(dt)
for m in (fm, mm):
    m.setAvailableForce(m.getMaxForce())

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


def step_for(secs):
    for _ in range(int(secs * 1000 / dt)):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


def tcp():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))[0]


def goto(xyz, dur=1.6, passes=3, tol=0.002):
    """Closed-loop on the FK tool point -- see the module docstring."""
    goal, err = list(xyz), 1e9
    for _ in range(passes):
        q, _p, _r, _i = dls_ik_pose(IK["chain"], list(NOM_SEED), goal, TOP,
                                    (0.0, 0.0, OZ), IK, IK_LIMITS)
        bridge.act_set_joint_positions(q, duration_s=dur)
        step_for(dur + 0.3)
        t = tcp()
        resid = [xyz[i] - t[i] for i in range(3)]
        err = max(abs(v) for v in resid)
        if err <= tol:
            break
        goal = [goal[i] + resid[i] for i in range(3)]
        dur = 0.5
    return err


def fingers():
    return fs.getValue(), ms.getValue()


def grip_point():
    """Where the pads' symmetry plane ACTUALLY is, in world coordinates.

    ⚠ THIS IS NOT THE FK TOOL POINT, AND THE DIFFERENCE IS THE WHOLE DEFECT
    THIS DEMO SPENT A REVISION BLAMING ON THE MIRRORED PRISMATIC JOINT. The
    finger link origins sit at gripper-local y = +(0.007+q_l) and -(0.007+q_r),
    so their midpoint is offset from the gripper axis by exactly (q_l-q_r)/2;
    undo that and you have a point on the axis, measured from the engine's own
    Solid poses with no kinematic model in the loop. Measured against it, the
    FK tool point is a CONSTANT +5.5 mm out in y (5.66 / 5.73 / 5.45 mm at
    three unrelated arm poses), which is why aiming the FK point at the block
    left the gripper 4-5 mm off-centre and the two pads stalled ~7 mm apart on
    identical targets. The joints themselves are honest: the measured
    separation of the two link origins matches 0.014+q_l+q_r to 0.6 mm.

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
    return [0.5 * (lp[i] + rp[i]) - u[i] * 0.5 * (ql - qr) + z[i] * 0.025
            for i in range(3)]


def open_fingers():
    for m in (fm, mm):
        m.setPosition(0.0425)


def preclose():
    """Position stage: to 1 mm of clearance, NOT into contact."""
    q = BLOCK_HALF + CLEARANCE
    for m in (fm, mm):
        m.setPosition(q)
    return q


def squeeze(ramp_s=1.0):
    """Squeeze stage: ramp the position target from clearance to interference.

    Ramped rather than stepped so the pads meet the block gently -- a single
    3 mm step is a 1.5 N impulse onto a 0.15 kg part and nudges it before the
    second pad arrives.
    """
    q0, q1 = BLOCK_HALF + CLEARANCE, BLOCK_HALF - INTERFERENCE
    n = max(1, int(ramp_s * 1000 / dt))
    for i in range(1, n + 1):
        q = q0 + (q1 - q0) * (i / float(n))
        fm.setPosition(q)
        mm.setPosition(q)
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


def release():
    open_fingers()
    step_for(1.0)


def bz():
    return block.getPosition()[2]


def rel():
    p, t = block.getPosition(), tcp()
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
    the QUERIED side, not the other body -- block.getContactPoints() stamps the
    block's own id on every point (OmSupervisorUtilities::pushContactPointsToStream
    writes solid->uniqueId() when includeDescendants is false). The robot's
    subtree query is the one whose node_id names a LINK, so match the two lists
    by point. Both are built from the same native Newton contact vector in the
    same step, so the doubles are equal, not merely close.
    """
    bp, R = block.getPosition(), block.getOrientation()
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
        for cp in block.getContactPoints():
            emit("[raw block] id=%d w=(%.4f,%.4f,%.4f) d=%.5f"
                 % (cp.node_id, cp.point[0], cp.point[1], cp.point[2], cp.depth))
    pts = []
    for cp in block.getContactPoints():
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
    palm = [p for p in pts if p["link"].startswith("robotiq") and "finger" not in p["link"]]
    other = [p for p in pts if not p["link"].startswith("robotiq")]
    # (b) a pad contact only counts if it is on the block's GRASP FACE (|y| near
    # the half-width) -- a pad touching the block's top or bottom edge is not a
    # pinch, it is a shelf.
    # ⚠ WHERE THE PADS ACTUALLY ARE, read off the finger link Solids rather than
    # inferred from the joint sensors. This is the measurement that separates
    # "the mirrored prismatic joint is broken" from "the gripper is not centred
    # on the part": the two link ORIGINS are at gripper-local y = +(0.007+q_l)
    # and -(0.007+q_r) by construction, so their separation must equal
    # 0.014+q_l+q_r if the joints are honest, and the axis they straddle is the
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
GRASP_Z = 0.245                 # block mid-height: pads clear the table by 20 mm
PLACE_X, PLACE_Y = 0.24, -0.42
# One-shot lateral aim bias, in metres, folded into the SAME goto calls the
# demo already makes -- so it changes where the gripper ends up without
# changing how many moves it took to get there. That separation is the point:
# PICK_AIM=grip re-aims with extra gotos, and an A/B against it needs an arm
# that centres the gripper with the motion held fixed.
YBIAS = float(os.environ.get("PICK_YBIAS", "0"))
AX, AY = PX, PY + YBIAS

open_fingers()
step_for(1.0)
emit("[pick] start block_z=%.4f fingers=%s control_drop=%s ybias=%.4f"
     % (bz(), fingers(), CONTROL_DROP, YBIAS))

emit("[pick] approach err=%.4f" % goto((AX, AY, GRASP_Z + 0.16), 2.0))
emit("[pick] descend  err=%.4f" % goto((AX, AY, GRASP_Z), 2.0))

r = rel()
emit("[pick] at block: lateral=%.1fmm vertical=%.1fmm"
     % (math.hypot(r[0], r[1]) * 1000.0, r[2] * 1000.0))

# ⚠ RE-AIM ON THE MEASURED PAD AXIS, not on FK. See grip_point(): the FK tool
# point is a constant ~5.5 mm out in y, so a descent that FK calls centred to
# 0.2 mm actually leaves the gripper 4-5 mm off the block. The pads then close
# asymmetrically (measured q_l 0.0201 / q_r 0.0287 on identical targets) and
# the block slides that same distance to re-centre itself once airborne --
# which is exactly the 3.1 mm "carry drift". PICK_AIM=fk restores the old aim.
gx, gy = AX, AY
if os.environ.get("PICK_AIM", "grip") != "fk":
    for _ in range(3):
        g = grip_point()
        if g is None:
            emit("[pick] aim: finger Solids not resolvable -- staying on FK")
            break
        bp = block.getPosition()
        ex, ey = bp[0] - g[0], bp[1] - g[1]
        emit("[pick] aim: pad axis is %+.1f,%+.1f mm off the block "
             "(FK tool point says %+.1f,%+.1f mm)"
             % (-ex * 1000.0, -ey * 1000.0, -r[0] * 1000.0, -r[1] * 1000.0))
        if math.hypot(ex, ey) <= 0.0005:
            break
        gx, gy = gx + ex, gy + ey
        goto((gx, gy, GRASP_Z), 1.0)

if CONTROL_DROP:
    emit("[pick] CONTROL: no squeeze -- the block must be left behind")
else:
    q_pre = preclose()
    step_for(0.8)
    emit("[pick] preclose -> %.4f (target %.4f)" % (fingers()[0], q_pre))
    squeeze()
    emit("[pick] squeezed %.0f mm interference -> fingers=%s (~%.2f N/pad)"
         % (INTERFERENCE * 1000.0, fingers(), GRIP_KP * INTERFERENCE))

def pads():
    """Where each pad's INNER FACE sits on the grasp axis, vs the block's own
    faces. The joint readings alone are not interpretable: the two joints are
    mirrored (axes +y and -y), so equal readings mean symmetric pads, and a
    reading that grows means that pad moved OUTWARD.
    """
    ql, qr = fingers()
    r = rel()
    # Pad inner faces in the tool frame, and the block's faces relative to it.
    return ("L_face=%+.4f R_face=%+.4f | block_y=%+.4f faces=[%+.4f,%+.4f] "
            "bite_L=%+.1fmm bite_R=%+.1fmm"
            % (ql, -qr, r[1], r[1] - BLOCK_HALF, r[1] + BLOCK_HALF,
               ((r[1] + BLOCK_HALF) - ql) * 1000.0,
               (qr - (BLOCK_HALF - r[1])) * 1000.0 * -1.0))


emit("[pick] pads after squeeze: %s" % pads())
c_squeeze = census("squeezed")
rel0 = rel()
emit("[pick] lift     err=%.4f" % goto((gx, gy, GRASP_Z + 0.22), 2.0))
emit("[pick] lifted  block_z=%.4f  %s" % (bz(), pads()))
c_lift = census("lifted")
emit("[pick] carry    err=%.4f" % goto((PLACE_X, PLACE_Y, GRASP_Z + 0.22), 3.0))
emit("[pick] carried block_z=%.4f  %s" % (bz(), pads()))
c_carry = census("carried")

carried_ok = bz() > TABLE_TOP + 0.06
drift = max(abs(rel()[i] - rel0[i]) for i in range(3))

# ⚠ THE GRASP ASSERTION. Airborne + co-moving says the RIG holds the block; only
# this says the PADS do. Both pads must be on the block's own grasp faces (|y| ~
# BLOCK_HALF) while it is off the table -- one-sided contact is a shelf, not a
# pinch, and palm contact means the 90 mm block is partly wedged into the
# gripper body rather than pinched between 50 mm pads.
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

emit("[pick] lower    err=%.4f" % goto((PLACE_X, PLACE_Y, GRASP_Z + 0.005), 2.0))
if not CONTROL_DROP:
    release()
emit("[pick] retreat  err=%.4f" % goto((PLACE_X, PLACE_Y, GRASP_Z + 0.20), 1.6))
step_for(1.5)

p = block.getPosition()
placed = (math.hypot(p[0] - PLACE_X, p[1] - PLACE_Y) < 0.10
          and p[2] > TABLE_TOP + 0.02)
ok = ((not carried_ok and not placed) if CONTROL_DROP
      else (carried_ok and placed and pinched))

emit("[pick] RESULT carried=%s pinched=%s palm_wedge=%s drift=%.4fm placed=%s "
     "final=(%.3f,%.3f,%.3f) -> %s"
     % (carried_ok, pinched, palm_wedge, drift, placed, p[0], p[1], p[2],
        "PASS" if ok else "FAIL"))

with open(OUT, "w", encoding="utf-8") as fh:
    json.dump({"control_drop": CONTROL_DROP, "carried": carried_ok,
               "pinched": pinched, "palm_wedge": palm_wedge,
               "contacts": [c_squeeze, c_lift, c_carry],
               "drift_m": drift, "placed": placed, "ok": ok,
               "final": list(p), "hold_mechanism": "friction",
               "interference_m": INTERFERENCE, "grip_n_per_pad": GRIP_KP * INTERFERENCE, "log": log}, fh, indent=1)

if os.environ.get("PICK_AUTOQUIT"):
    robot.simulationQuit(0 if ok else 1)
while robot.step(dt) != -1:
    bridge.tick(robot.getTime())
