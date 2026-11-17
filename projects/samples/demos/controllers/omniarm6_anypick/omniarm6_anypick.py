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

"""ANYPICK: shape-agnostic suction bin picking for the OMNIARM6.

The bin-picking demo (omniarm6_bin_picking) empties a bin of identical cubes; its
pick is hardcoded to a cube (suction point = centre + half-edge, hold offset =
0.025). AnyPick generalises that pick to ARBITRARY rigid parts. The bin here
holds a MIX of cubes, capsule tubes (lying and standing) and L-shaped tubes
(compound two-box bodies), dumped together, and the arm empties the bin and
SORTS the parts by shape into three trays.

The AnyPick algorithm:

  1. Each part kind carries a small primitive registry (boxes + capsules in the
     part's body frame) -- in a real cell this comes from CAD.
  2. At pick time, enumerate candidate SUCTION ANCHORS on the part's surface:
     every box face centre, every capsule end cap, and (for a lying capsule)
     the top line of the barrel. Transform them by the part's CURRENT pose.
  3. Keep the anchors whose outward normal points up (cup-reachable from
     above), and take the HIGHEST one. So a flat L picks on a limb's top face,
     a standing tube picks on its end cap, a lying tube picks on its barrel.
  4. Descend the cup onto the anchor and PRESS TO CONTACT: the vacuum only
     engages when the MEASURED cup lip (FK on the measured joints) is at the
     part's surface (gap <= GRAB_EPS, laterally on it), pressing further in
     small steps and chasing the live anchor if the part settles or rolls. A
     cup that never reaches the surface retreats and retries; it can NOT grab
     through air. The LONG suction tool (omniarm6_suction_long.urdf, lip =
     flange + 0.200 m) keeps the fat wrist colliders above the bin wall top
     on a straight-down descent -- only the thin stick enters the bin; near a
     wall a SMALL lean toward the bin centre (wall_lean) clears the cup lip
     of parts sitting flush against the wall.
  5. On engage the part is frozen exactly where it sits relative to the cup
     AT THE MOMENT OF CONTACT (measured offset, not a snap-to-flush teleport),
     so nothing ever jumps to the cup like a magnet -- the hold starts from
     the touching pose it already has.
  6. Carry and drop heights are computed per pick from how far the part hangs
     BELOW its anchor, so a long part grabbed near one end still clears the
     bin and tray walls.

Everything else (camera-driven top-of-pile choice, vacuum weld with per-tick
velocity zeroing, bin pinning, recovery pass) is the proven bin-picking
recipe. Calls warmup_reload() at startup to dodge the COLD-FIRST-LOAD bug --
see docs/developer/real-grasp-and-the-cold-first-load-trap.md.

The world bakes newtonStatics/newtonCompoundColliders into WorldInfo (the
L-tubes and the movable bin are compound multi-box bodies), so no env vars
are required. Set ANYPICK_AUTOQUIT=1 to exit with a status when done.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import time  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, warmup_reload, dls_ik_pose,  # noqa: E402
                                 forward_kinematics_pose, _mat3_to_axis_angle,
                                 _mat_mul)
from _arm_configs import get_config  # noqa: E402

OZ = 0.3655                     # tool point = the cup LIP (flange + 0.200, LONG tool: omniarm6_suction_long.urdf)
OUT = os.environ.get("ANYPICK_OUT", "_anypick_result.txt")

# Carry / approach geometry. Carry and drop LIP heights are computed PER PICK
# from the part's extent below its anchor (see anypick_carry_z) -- these are
# the bounds.
CARRY_MIN = 0.32                # lip height floor (a cube barely needs more)
CARRY_MAX = 0.45                # lip height ceiling (IK comfort)
WALL_CLEAR = 0.27               # part BOTTOM must ride above this (bin walls top out at 0.241)
DROP_BOTTOM = 0.045             # part bottom at release, above the tray floor
LIFT_RISE_OK = 0.08             # a grasped part must RISE by this after the lift
MIN_UP = 0.6                    # anchor normal . z gate: cup-reachable from above

# Contact-honest vacuum: the cup must TOUCH the part before the vacuum can
# engage (no magnet-like action at a distance). Touch = the FK-measured lip is
# within GRAB_EPS above (or pressed slightly into) the part's surface anchor,
# and laterally on it.
GRAB_EPS = 0.004                # max measured lip->surface gap that counts as touching
PRESS_DEPTHS = (0.002, 0.0, -0.003, -0.006, -0.009, -0.012)  # descent targets vs anchor
PRESS_IN_MAX = 0.025            # accept a lip pressed at most this far past the anchor
HORIZ_EPS = 0.012               # lip must be laterally on the anchor too

# Near-wall TILTED approach. With the LONG tool (omniarm6_suction_long.urdf,
# lip = flange + 0.200) the fat wrist colliders -- link6 r=0.05 bottoming at
# the flange, link5 r=0.10 above it -- ride ABOVE the bin wall top (z=0.241)
# even with the lip pressed onto a floor-level part (flange >= ~0.25), so a
# straight-down descent no longer parks the wrist on the wall: only the thin
# r=0.024 stick enters the bin. What remains is the CUP itself: the r=0.027
# lip of a part sitting flush against a wall overhangs the wall line, so keep
# a SMALL lean toward the bin centre near walls (a few degrees clears the
# lip + stick; the old 29-deg wrist-clearing lean predates the long tool and
# cost suction quality on every near-wall pick).
WALL_HALF = 0.18                # bin interior half-extent (inner wall faces)
WALL_NEAR = 0.135               # start leaning when closer than this to a wall
MAX_TILT = 0.15                 # max lean (rad) pressed right against a wall
REACH_MAX = 0.75                # parts whose anchor xy is beyond this are unreachable

# ── AnyPick shape registry ───────────────────────────────────────────
# Body-frame primitives per part kind (from CAD, in a real cell). boxes are
# (centre, half-extents); caps(ules) are (centre, radius, half_height) along
# the body z-axis. MUST match the geometry authored in omniarm6_anypick.omniworld.
KIND_GEOM = {
    "cube": {
        "boxes": [((0.0, 0.0, 0.0), (0.025, 0.025, 0.025))],
        "caps": [],
    },
    "ltube": {
        "boxes": [((0.0, 0.0, 0.0), (0.05, 0.016, 0.016)),
                  ((-0.034, 0.051, 0.0), (0.016, 0.035, 0.016))],
        "caps": [],
    },
    "tube": {
        "boxes": [],
        "caps": [((0.0, 0.0, 0.0), 0.021, 0.04)],
    },
}

# Three sorted destinations on the south side, one per shape kind. Each is a
# walled tray; slots keep consecutive drops from landing on each other.
KIND_SLOTS = {
    "cube": [(0.32 + dx, -0.44 + dy) for dy in (-0.05, 0.05) for dx in (-0.05, 0.05)],
    "ltube": [(-0.02 + dx, -0.44 + dy) for dy in (-0.055, 0.055) for dx in (-0.045, 0.045)],
    "tube": [(-0.36 + dx, -0.44 + dy) for dy in (-0.05, 0.05) for dx in (-0.05, 0.05)],
}
KIND_ZONE = {                   # x-range of each tray pad (y/z shared below)
    "cube": (0.18, 0.46),
    "ltube": (-0.17, 0.13),
    "tube": (-0.50, -0.22),
}

robot = Supervisor()
# Dodge the COLD-FIRST-LOAD bug: on a fresh build the arm under-tracks its pick
# poses; a one-time reload at startup makes the first load track accurately.
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
# No finger gripper: the end-effector is the suction cup tool link in
# omniarm6_suction.urdf. gripper_id=None -> the bridge does pure IK.
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()
IK = bridge.cfg["ik"]

# IK limits with joint2 capped to its PHYSICAL range. The URDF declares
# +-pi, but the real arm stops at j2 ~= 2.0 (upper-arm collider meets the
# base column). With the LONG tool the unconstrained DLS settles into an
# elbow-back branch demanding j2 = 2.0-2.4 for in-bin presses: the commands
# clamp at the physical stop, every press lands high + laterally off, and
# the cup bulldozes parts instead of touching them (measured 2026-07-18,
# solved-vs-measured joint traces). Capping j2 forces the elbow-forward
# branch, which solves the whole bin grid at j2 <= 0.7 (offline probe:
# 0/50 fails, worst 4.9 mm).
IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)

# Fixed IK seed on the elbow-forward POSITIVE wrist branch (j2+j3+j5 = +pi
# for a top-down tool). DLS is a local method: seeded from the measured q it
# stays on whatever branch the arm is on, and the NEGATIVE branch (which the
# home posture leads to) cannot press the LONG tool into the bin within the
# j2 stop -- chained seeds reproduce exactly the live 50-100 mm stalls.
# Seeding every solve from this one posture keeps all solutions on the
# feasible branch AND mutually consistent (small joint-space hops between
# consecutive picks). Offline: whole bin grid + press ladder + leans solve
# 0/50 fails, worst 4.6 mm, j2 <= 0.55, j5 in [0.97, 2.04].
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]

# Top-down camera bridge: BIN_CAM's controller writes the parts it recognises
# into its customData as "<nodeId>:<colour>,..." (occlusion -> top of pile).
_cam_node = robot.getFromDef("BIN_CAM")
_cam_data = _cam_node.getField("customData") if _cam_node else None
ID_TO_NAME = {}                 # nodeId -> "PART_n", filled in discover_parts

# The bin is a free dynamic body (same node as the bin-picking demo); during
# picking it is PINNED at home so the cup's bumps can't drift it.
_binnode = robot.getFromDef("MOVABLE_BIN")
_bin_tr = _binnode.getField("translation") if _binnode else None
_bin_rot = _binnode.getField("rotation") if _binnode else None
BIN_HOME = [0.46, 0.0, 0.0]


def emit(s):
    fh.write(s + "\n")
    print(s, flush=True)


def _tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


def _R(node):
    o = node.getOrientation()
    return [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]


def _rotv(R, v):
    return [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]]


def _world(c, R, b):
    w = _rotv(R, b)
    return [c[0] + w[0], c[1] + w[1], c[2] + w[2]]


# ── AnyPick core: orientation-aware suction-anchor selection ─────────
def anypick_anchor(node, kind):
    """The best suction anchor on `node`'s surface RIGHT NOW: among candidate
    anchors (box face centres, capsule end caps, lying-capsule barrel top
    line) transformed by the part's current pose, keep those whose outward
    normal points up (the cup approaches from above) and return the HIGHEST,
    as a world point. Falls back to the highest candidate regardless of
    normal, so a part resting on an edge still gets picked at its top."""
    R = _R(node)
    c = node.getPosition()
    g = KIND_GEOM[kind]
    cands = []                                  # (world_z, world_point, n_up)
    for bc, bh in g["boxes"]:
        for ax in range(3):
            for s in (1.0, -1.0):
                nb = [0.0, 0.0, 0.0]
                nb[ax] = s
                pb = [bc[0] + nb[0] * bh[0], bc[1] + nb[1] * bh[1], bc[2] + nb[2] * bh[2]]
                pw = _world(c, R, pb)
                nw = _rotv(R, nb)
                cands.append((pw[2], pw, nw[2]))
    for bc, r, hh in g["caps"]:
        aw = _rotv(R, [0.0, 0.0, 1.0])          # world capsule axis
        for s in (1.0, -1.0):                   # the two end caps
            pb = [bc[0], bc[1], bc[2] + s * (hh + r)]
            pw = _world(c, R, pb)
            cands.append((pw[2], pw, aw[2] * s))
        if abs(aw[2]) < 0.92:                   # lying: top line of the barrel
            w = [-aw[0] * aw[2], -aw[1] * aw[2], 1.0 - aw[2] * aw[2]]
            n = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) or 1.0
            w = [w[0] / n, w[1] / n, w[2] / n]
            cw = _world(c, R, bc)
            cands.append((cw[2] + r * w[2], [cw[0] + r * w[0], cw[1] + r * w[1], cw[2] + r * w[2]], w[2]))
    up = [cd for cd in cands if cd[2] >= MIN_UP]
    pool = up if up else cands
    # Highest first, but among near-equal heights (a flat L's two limb faces)
    # prefer the most BIN-CENTRAL candidate: a wall-side anchor needs a deep
    # tilt the IK may not deliver, while an equally-high central one picks
    # clean. 4 mm height bins.
    pool.sort(key=lambda cd: (-round(cd[0] / 0.004),
                              math.hypot(cd[1][0] - BIN_HOME[0], cd[1][1] - BIN_HOME[1])))
    return pool[0][1]


def part_min_z(node, kind):
    """Lowest world-z point of the part at its current pose (box corners +
    capsule segment ends minus radius)."""
    R = _R(node)
    c = node.getPosition()
    zmin = 1e9
    g = KIND_GEOM[kind]
    for bc, bh in g["boxes"]:
        for sx in (1.0, -1.0):
            for sy in (1.0, -1.0):
                for sz in (1.0, -1.0):
                    pb = [bc[0] + sx * bh[0], bc[1] + sy * bh[1], bc[2] + sz * bh[2]]
                    zmin = min(zmin, _world(c, R, pb)[2])
    for bc, r, hh in g["caps"]:
        for s in (1.0, -1.0):
            pb = [bc[0], bc[1], bc[2] + s * hh]
            zmin = min(zmin, _world(c, R, pb)[2] - r)
    return zmin


def anypick_carry_z(ext):
    """Carry LIP height for a part hanging `ext` below its anchor: its bottom
    must clear the bin walls, within IK-comfortable bounds."""
    return min(CARRY_MAX, max(CARRY_MIN, WALL_CLEAR + ext))


# ── SUCTION (vacuum) hold, frozen at the CONTACT pose ────────────────
# The vacuum is a kinematic weld: the part tracks the cup lip every tick. The
# held offset is MEASURED at engage time (part centre relative to the actual
# FK lip position), so engaging the vacuum moves NOTHING -- the part keeps the
# exact touching pose it already has and simply starts riding the cup. That is
# what a real suction cup / sticky gum does; the old anchor-based offset
# teleported the part flush to the lip, which read as a magnet whenever the
# arm had stopped a few mm short. Straight-down hang (not rotation-coupled):
# a mid-carry wrist reconfigure can't swing the part off. Velocity is pinned
# to zero every tick -- a teleported dynamic body otherwise accumulates
# velocity and bounces on a warm (reloaded) solver.
_suck = None        # (node, tr_field, rot_field, d_world, rot0)


def suck_on(node):
    global _suck
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    pos, _R_ = _tcp_pose()
    c = node.getPosition()
    d = [c[0] - pos[0], c[1] - pos[1], c[2] - pos[2]]   # measured: zero jump at engage
    try:
        node.setVelocity([0.0] * 6)             # kill any pile velocity at grab
    except Exception:
        pass
    _suck = (node, node.getField("translation"), node.getField("rotation"), d, rot0)


def suck_apply():
    if _suck is None:
        return
    node, tr, rot, d, rot0 = _suck
    pos, _R_ = _tcp_pose()
    tr.setSFVec3f([pos[0] + d[0], pos[1] + d[1], pos[2] + d[2]])
    rot.setSFRotation(rot0)
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass


def suck_off():
    global _suck
    if _suck is not None:
        try:
            _suck[0].resetPhysics()
        except Exception:
            pass
    _suck = None


def step_for(secs):
    """Advance the sim, driving the bridge motion, the vacuum hold and the
    bin pin each tick."""
    n = int(secs * 1000 / dt)
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if _suck is not None:
            suck_apply()
        elif _binnode is not None:
            # Re-home the bin only when a bump has actually drifted it.
            bp = _binnode.getPosition()
            if (abs(bp[0] - BIN_HOME[0]) > 0.004 or abs(bp[1] - BIN_HOME[1]) > 0.004
                    or abs(bp[2] - BIN_HOME[2]) > 0.006):
                _bin_tr.setSFVec3f(BIN_HOME)
                _bin_rot.setSFRotation([0.0, 0.0, 1.0, 0.0])
    return True


DEBUG = bool(os.environ.get("ANYPICK_DEBUG"))


def pose(xyz, dur=1.4, settle=0.3):
    # Solved here (not via act_set_tcp_pose) so the j2-capped IK_LIMITS
    # apply -- the bridge's internal solve uses the full URDF limits.
    top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), list(xyz),
                                        top, (0.0, 0.0, OZ), IK, IK_LIMITS)
    if perr > 0.02:
        emit("[any]   ! IK not converged for pose (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
             % (xyz[0], xyz[1], xyz[2], perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)
    if DEBUG:
        lip, _R_ = _tcp_pose()
        emit("[dbg] pose tgt=(%.3f,%.3f,%.3f) sq=%s mq=%s lip=(%.3f,%.3f,%.3f)"
             % (xyz[0], xyz[1], xyz[2], ["%.2f" % v for v in q],
                ["%.2f" % v for v in bridge._read_q()],
                lip[0], lip[1], lip[2]))


def _rot_axis(a, t):
    """3x3 rotation about unit axis a by angle t (Rodrigues)."""
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]]


def wall_lean(x, y):
    """(lean_x, lean_y, tilt) for a pick at (x, y): unit horizontal direction
    toward the bin centre + lean angle, scaled by how deep (x, y) sits in the
    wall keep-out band. None when a straight-down approach has wrist
    clearance."""
    lx = ly = 0.0
    dx, dy = x - BIN_HOME[0], y - BIN_HOME[1]
    if WALL_HALF - abs(dx) < WALL_NEAR:
        lx = -math.copysign(1.0, dx) * (1.0 - max(0.0, WALL_HALF - abs(dx)) / WALL_NEAR)
    if WALL_HALF - abs(dy) < WALL_NEAR:
        ly = -math.copysign(1.0, dy) * (1.0 - max(0.0, WALL_HALF - abs(dy)) / WALL_NEAR)
    m = math.hypot(lx, ly)
    if m < 1e-6:
        return None
    return lx / m, ly / m, MAX_TILT * math.sqrt(min(1.0, m))


def approach_pose(x, y, z, lean, dur=1.2, settle=0.25):
    """Move the cup LIP to (x,y,z). Straight top-down when lean is None;
    otherwise the tool is tilted by lean=(lx,ly,theta) so the wrist leans
    toward the bin centre and clears the wall the part sits against."""
    if lean is None:
        pose([x, y, z], dur=dur, settle=settle)
        return
    lx, ly, theta = lean
    top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    # Rot about (z x lean) by theta sends the wrist direction (world +z when
    # top-down) to z*cos + lean*sin: leaning toward the bin centre.
    rt = _mat_mul(_rot_axis([-ly, lx, 0.0], theta), top)
    q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), [x, y, z], rt,
                                        (0.0, 0.0, OZ), IK, IK_LIMITS)
    if perr > 0.02:
        emit("[any]   ! IK not converged for approach (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
             % (x, y, z, perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)
    if DEBUG:
        lip, _R_ = _tcp_pose()
        emit("[dbg] appr tgt=(%.3f,%.3f,%.3f) sq=%s mq=%s lip=(%.3f,%.3f,%.3f)"
             % (x, y, z, ["%.2f" % v for v in q],
                ["%.2f" % v for v in bridge._read_q()],
                lip[0], lip[1], lip[2]))


# ── Scene bookkeeping ────────────────────────────────────────────────
def discover_parts():
    """Every DEF PART_<n> in the scene as {name: (node, kind)}; kind comes
    from the Solid's name field ("cube_3" -> cube). Fills ID_TO_NAME for the
    camera."""
    parts = {}
    i = 1
    misses = 0
    while misses < 6:
        name = "PART_%d" % i
        node = robot.getFromDef(name)
        if node is not None:
            kind = (node.getField("name").getSFString() or "cube").split("_")[0]
            if kind not in KIND_GEOM:
                kind = "cube"
            parts[name] = (node, kind)
            ID_TO_NAME[node.getId()] = name
            misses = 0
        else:
            misses += 1
        i += 1
    return parts


def camera_detection():
    """Set of part names the top-down camera currently sees (top of pile)."""
    out = set()
    if _cam_data is None:
        return out
    raw = _cam_data.getSFString() or ""
    for tok in raw.split(","):
        if ":" not in tok:
            continue
        sid, _, _col = tok.partition(":")
        try:
            name = ID_TO_NAME.get(int(sid))
        except ValueError:
            name = None
        if name:
            out.add(name)
    return out


def pz(node):
    p = node.getPosition()
    return [round(p[0], 3), round(p[1], 3), round(p[2], 3)]


def in_bin(node):
    p = node.getPosition()
    bp = _binnode.getPosition() if _binnode is not None else BIN_HOME
    return (abs(p[0] - bp[0]) <= 0.20 and abs(p[1] - bp[1]) <= 0.20
            and p[2] < 0.28)


def on_zone(node, kind):
    """Is the part resting on its matching shape tray?"""
    p = node.getPosition()
    x0, x1 = KIND_ZONE[kind]
    return x0 <= p[0] <= x1 and -0.58 <= p[1] <= -0.30 and p[2] < 0.16


def next_pick(parts, done):
    """The next part to pick: among the parts the camera sees (top of the
    pile), the highest one still in the bin; scene-tree fallback if the
    camera is occluded. Returns (name, node, kind, source) or None."""
    seen = camera_detection()
    cands = [(n,) + parts[n] for n in seen
             if n in parts and n not in done and in_bin(parts[n][0])]
    src = "cam"
    if not cands:
        cands = [(n,) + parts[n] for n in parts
                 if n not in done and in_bin(parts[n][0])]
        src = "scene"
    if not cands:
        return None
    cands.sort(key=lambda c: c[1].getPosition()[2], reverse=True)
    return cands[0][0], cands[0][1], cands[0][2], src


# ── The pick itself ──────────────────────────────────────────────────
def press_to_contact(node, kind, lean):
    """Descend the cup onto the part and verify a REAL touch before the
    vacuum may engage: the FK-measured lip must be within GRAB_EPS above (or
    at most PRESS_IN_MAX pressed past) the part's CURRENT surface anchor, and
    laterally on it. Each step re-reads the anchor -- if the part settles or
    rolls under the approach, the cup chases the live surface instead of
    grabbing where it used to be. Returns True when touching; False means the
    cup never reached the surface (-> no vacuum, the pick is retried)."""
    gap = horiz = 1e9
    for i, depth in enumerate(PRESS_DEPTHS):
        a = anypick_anchor(node, kind)                   # live surface, every step
        approach_pose(a[0], a[1], a[2] + depth, lean,
                      dur=(1.2 if i == 0 else 0.35), settle=(0.25 if i == 0 else 0.1))
        lip, _R_ = _tcp_pose()
        a2 = anypick_anchor(node, kind)
        gap = lip[2] - a2[2]
        horiz = math.hypot(lip[0] - a2[0], lip[1] - a2[1])
        if -PRESS_IN_MAX <= gap <= GRAB_EPS and horiz <= HORIZ_EPS:
            emit("[any]   touch ok: gap=%.1fmm lateral=%.1fmm%s"
                 % (gap * 1000.0, horiz * 1000.0,
                    " (tilted %.0f deg)" % math.degrees(lean[2]) if lean else ""))
            return True
    emit("[any]   NO TOUCH (gap=%.1fmm lateral=%.1fmm) -> no vacuum, retreating"
         % (gap * 1000.0, horiz * 1000.0))
    return False


def anypick(name, node, kind, slot):
    """One AnyPick: anchor on the part's current top surface, press to
    contact, vacuum on, lift, carry to the kind's tray slot, release. True if
    the part left the bin."""
    anchor = anypick_anchor(node, kind)
    if math.hypot(anchor[0], anchor[1]) > REACH_MAX:     # flung out of the workspace
        emit("[any]   anchor %.2f m out -> unreachable, skipping" % math.hypot(anchor[0], anchor[1]))
        return False
    z0 = node.getPosition()[2]
    pose([anchor[0], anchor[1], anypick_carry_z(0.1)])   # stage above the anchor
    anchor = anypick_anchor(node, kind)                  # re-read: the pile may have shifted
    ext = max(0.0, anchor[2] - part_min_z(node, kind))
    carry = anypick_carry_z(ext)
    lean = wall_lean(anchor[0], anchor[1])               # tilt the wrist off a nearby wall
    emit("[any]   anchor=(%.3f,%.3f,%.3f) hangs %.3f below; carry lip %.2f"
         % (anchor[0], anchor[1], anchor[2], ext, carry))
    if not press_to_contact(node, kind, lean):           # must TOUCH before sucking
        approach_pose(anchor[0], anchor[1], carry, lean)
        pose([anchor[0], anchor[1], carry])
        return False
    suck_on(node)                                        # vacuum ON at the contact pose
    step_for(0.3)
    lp = _tcp_pose()[0]
    approach_pose(lp[0], lp[1], carry, lean)             # lift straight up (keep the tilt)
    pose([lp[0], lp[1], carry], dur=0.8)                 # square the tool back up
    if node.getPosition()[2] < z0 + LIFT_RISE_OK:        # vacuum didn't take
        suck_off()
        step_for(0.2)
        return False
    sx, sy = slot
    pose([sx, sy, carry], dur=1.8)                       # carry over the tray
    pose([sx, sy, DROP_BOTTOM + ext])                    # lower into the tray
    suck_off()
    step_for(0.6)                                        # release -> drop in
    pose([sx, sy, carry])                                # retreat up
    return not in_bin(node)


# ── Run ──────────────────────────────────────────────────────────────
step_for(6.0)                                           # let the dumped pile settle
parts = discover_parts()
kinds = {}
for n, (_nd, k) in parts.items():
    kinds[k] = kinds.get(k, 0) + 1
emit("[any] start    %d parts: %s" % (
    len(parts), ", ".join("%d %s" % (v, k) for k, v in sorted(kinds.items()))))
emit("[any] camera sees %d parts on top of the pile" % len(camera_detection()))

done = set()
slot_i = {k: 0 for k in KIND_GEOM}
tries = {}
guard = 0
while guard < 150:
    guard += 1
    step_for(0.3)                                       # arm clear -> camera refresh
    pick = next_pick(parts, done)
    if pick is None:
        break                                           # bin empty
    name, node, kind, src = pick
    tries[name] = tries.get(name, 0) + 1
    slot = KIND_SLOTS[kind][slot_i[kind] % len(KIND_SLOTS[kind])]
    emit("[any] picking %s (%s, %s, try %d) @ %s" % (name, kind, src, tries[name], pz(node)))
    if anypick(name, node, kind, slot):
        done.add(name)
        slot_i[kind] += 1
    elif not in_bin(node):
        done.add(name)                                  # left the bin some other way
    elif tries[name] >= 4:
        done.add(name)
        emit("[any]   -> %s unliftable after %d tries" % (name, tries[name]))

# RECOVERY: re-pick anything that ended off its matching tray (missed the
# drop, rolled out, or landed in the wrong tray). Isolated now, so a clean
# AnyPick fixes it.
rtries = {}
for _ in range(40):
    bad = next((n for n in parts
                if not on_zone(parts[n][0], parts[n][1])
                and rtries.get(n, 0) < 3), None)
    if bad is None:
        break
    node, kind = parts[bad]
    slot = KIND_SLOTS[kind][slot_i[kind] % len(KIND_SLOTS[kind])]
    emit("[any] RECOVER %s (%s) from %s -> %s tray" % (bad, kind, pz(node), kind))
    if anypick(bad, node, kind, slot):
        slot_i[kind] += 1
    else:
        rtries[bad] = rtries.get(bad, 0) + 1

# Final tally.
bridge.act_reset_to_home()
step_for(1.2)
remaining = [n for n in parts if in_bin(parts[n][0])]
emptied = sum(1 for n in parts if not in_bin(parts[n][0]))
sorted_ok = sum(1 for n in parts if on_zone(parts[n][0], parts[n][1]))
n = len(parts)
ok = emptied >= 0.9 * n and sorted_ok >= 0.75 * n
emit("[any] RESULT emptied=%d/%d sorted=%d/%d bin_empty=%s remaining=%s | "
     "wall=%.1fs sim=%.1fs"
     % (emptied, n, sorted_ok, n, not remaining, remaining,
        time.time() - t0, robot.getTime()))
emit("PASS" if ok else "FAIL")

if os.environ.get("ANYPICK_AUTOQUIT"):
    try:
        robot.simulationQuit(0 if ok else 1)
    except Exception:
        pass
else:
    while robot.step(dt) != -1:
        bridge.tick(robot.getTime())
        if _suck is not None:
            suck_apply()
