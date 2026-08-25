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

"""ANYPICK LINE: an L-shaped conveyor feeds bins of mixed parts to AnyPick.

Extends the omniarm6_anypick demo (shape-agnostic suction picking) into a line
cell: an L-shaped conveyor brings BINS of mixed parts (cubes, capsule tubes,
L-shaped tubes) to a pick station in front of the OMNIARM6. Each bin arrives in
a DIFFERENT ORIENTATION (yawed on the belt), the robot empties it -- sorting
the parts by shape into three trays -- and the emptied bin departs along the
other leg of the L while the next bin advances. Three bins, one after the
other, all emptied.

The conveyor legs are static scenery; the bins ride them KINEMATICALLY: the
controller advances each bin and its (settled) contents as a rigid ensemble
every tick -- per-tick pose set + setVelocity(0), the proven carry recipe --
so transport is deterministic under Newton. At the station the contents are
live physics in the pinned bin and the picking is exactly the contact-honest
AnyPick:

  * per-shape, orientation-aware suction anchors (box faces / capsule caps /
    lying-capsule barrel line), highest up-facing candidate, bin-central on
    near-ties;
  * vacuum engages ONLY at measured contact (FK lip within GRAB_EPS of the
    live surface, laterally on it; press-and-chase descent); the hold freezes
    the MEASURED contact pose -- nothing ever snaps to the cup;
  * near-wall approaches TILT toward the bin centre so the fat wrist
    colliders clear the wall top -- here computed in the BIN'S OWN FRAME,
    because a yawed bin's walls are not axis-aligned;
  * carry / drop heights computed per pick from the part's hang below its
    anchor (all raised by the belt height).

Calls warmup_reload() at startup (cold-first-load dodge). Launch with
OMNISIM_NEWTON_STATICS=1 and OMNISIM_NEWTON_COMPOUND_COLLIDERS=1. Set
ANYPICK_AUTOQUIT=1 to exit with a status when done.
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

OZ = 0.25                       # tool point = the cup LIP (flange + ~0.085)
OUT = os.environ.get("ANYPICK_OUT", "_anypick_line_result.txt")

BELT_TOP = 0.08                 # conveyor surface height; bins ride at z = BELT_TOP
STATION = (0.46, 0.0)           # pick station: the corner of the L, in front of the arm
CONVEY_SPEED = 0.32             # m/s ensemble transport speed (kinematic, safe)
PARK_X = (1.78, 1.33, 0.88)     # where emptied bins 1..3 park on the out leg

# Carry / approach geometry (the omniarm6_anypick values, raised by BELT_TOP).
CARRY_MIN = 0.40
CARRY_MAX = 0.50
WALL_CLEAR = 0.35               # part BOTTOM rides above this (line-bin walls top out at
                                # ~0.222 world: 0.12-tall walls on the 0.08 belt -- extra margin)
DROP_BOTTOM = 0.035             # part bottom at release, above the tray floor
                                # (low: fall energy is what bounces parts out)
LIFT_RISE_OK = 0.08
MIN_UP = 0.6

# Contact-honest vacuum (see omniarm6_anypick).
GRAB_EPS = 0.004
PRESS_DEPTHS = (0.002, 0.0, -0.003, -0.006, -0.009, -0.012)
PRESS_IN_MAX = 0.025
HORIZ_EPS = 0.012

# Near-wall tilted approach -- in the BIN FRAME (bins arrive yawed).
WALL_HALF = 0.18
WALL_NEAR = 0.135
MAX_TILT = 0.50
REACH_MAX = 0.80                # recovery can stretch for belt drops at ~0.78

# ── AnyPick shape registry (must match omniarm6_anypick_line.omniworld) ───────
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
    # New kinds are PURE REGISTRY ENTRIES -- no new pick code. bar: a long
    # block; tbar: a T-bracket (two boxes). Sorted by FAMILY: blocks (cube,
    # bar) -> red tray, brackets (ltube, tbar) -> green, tubes -> blue.
    "bar": {
        "boxes": [((0.0, 0.0, 0.0), (0.07, 0.015, 0.015))],
        "caps": [],
    },
    "tbar": {
        "boxes": [((0.0, 0.0, 0.0), (0.05, 0.016, 0.016)),
                  ((0.0, 0.043, 0.0), (0.016, 0.027, 0.016))],
        "caps": [],
    },
    # gear: disc approximated by crossed boxes (translation-only compound) --
    # a KNOWN class for the learned classifier.
    "gear": {
        "boxes": [((0.0, 0.0, 0.0), (0.052, 0.02, 0.012)),
                  ((0.0, 0.0, 0.0), (0.02, 0.052, 0.012))],
        "caps": [],
    },
    # "unknown" = a part the perception cannot classify. The geometry here is
    # a PLACEHOLDER never used for grasping: unknown parts are picked
    # MODEL-FREE (the camera publishes a depth-image surface anchor) and go
    # to the black bin.
    "unknown": {
        "boxes": [((0.0, 0.0, 0.0), (0.045, 0.045, 0.012))],
        "caps": [],
    },
}

# Body-frame bounding-box centre per kind. The Recognition API reports an
# object's BBOX centre, not its frame origin -- identical for cubes/tubes
# but 35 mm apart for an L, which silently wrecked every L estimate (the
# startup calibration line caught it: mean pose err 18.6 mm vs 3 mm noise).
def _bbox_centre(kind):
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for bc, bh in KIND_GEOM[kind]["boxes"]:
        for i in range(3):
            lo[i] = min(lo[i], bc[i] - bh[i])
            hi[i] = max(hi[i], bc[i] + bh[i])
    for bc, r, hh in KIND_GEOM[kind]["caps"]:
        ext = (r, r, hh + r)
        for i in range(3):
            lo[i] = min(lo[i], bc[i] - ext[i])
            hi[i] = max(hi[i], bc[i] + ext[i])
    return [(lo[i] + hi[i]) / 2.0 for i in range(3)]


KIND_SLOTS = {
    "cube": [(0.32 + dx, -0.44 + dy) for dy in (-0.07, 0.0, 0.07)
             for dx in (-0.075, -0.025, 0.025, 0.075)],
    "ltube": [(-0.02 + dx, -0.44 + dy) for dy in (-0.075, 0.0, 0.075) for dx in (-0.045, 0.045)],
    "tube": [(-0.36 + dx, -0.44 + dy) for dy in (-0.07, 0.0, 0.07) for dx in (-0.05, 0.05)],
    "unknown": [(-0.10 + dx, 0.36 + dy) for dy in (-0.05, 0.05) for dx in (-0.05, 0.05)],
}
KIND_ZONE = {                   # (x0, x1, y0, y1) of each tray pad
    "cube": (0.18, 0.46, -0.58, -0.30),
    "ltube": (-0.17, 0.13, -0.58, -0.30),
    "tube": (-0.50, -0.22, -0.58, -0.30),
    "unknown": (-0.23, 0.03, 0.23, 0.49),
}
# Sorting rule: the three trays belong to the three STANDARD part types
# (cubes -> red, L-brackets/T-brackets -> green, tubes -> blue); everything
# else -- gears and bars included, even though the classifier RECOGNIZES
# them -- is a non-standard part and goes to the BLACK bin.
KIND_SLOTS["bar"] = list(KIND_SLOTS["unknown"])
KIND_SLOTS["tbar"] = list(KIND_SLOTS["ltube"])
KIND_SLOTS["gear"] = list(KIND_SLOTS["unknown"])
KIND_ZONE["bar"] = KIND_ZONE["unknown"]
KIND_ZONE["tbar"] = KIND_ZONE["ltube"]
KIND_ZONE["gear"] = KIND_ZONE["unknown"]
# Slot counters are PER TRAY, not per kind: kinds sharing a tray used to
# each start at slot 0, dropping parts onto the ones already there -- the
# second-layer drop bounced off the pile and clean out of the tray.
TRAY_OF = {"cube": "red", "bar": "black", "gear": "black",
           "ltube": "green", "tbar": "green",
           "tube": "blue", "unknown": "black"}
KIND_BBOXC = {k: _bbox_centre(k) for k in KIND_GEOM}

robot = Supervisor()
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()
IK = bridge.cfg["ik"]

_cam_node = robot.getFromDef("BIN_CAM")
_cam_data = _cam_node.getField("customData") if _cam_node else None
ID_TO_NAME = {}


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


def _rot_axis(a, t):
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]]


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


# ── Bins on the conveyor ─────────────────────────────────────────────
# BINS[i] = {node, tr, rot, pose: [x, y, yaw]} -- `pose` is the bin's TARGET
# pose; step_for re-pins any bin that drifts off its target (gripper bumps,
# solver jitter), exactly the single-bin pinning recipe. The conveyor code
# updates `pose` as it advances an ensemble.
BINS = []
for i in (1, 2, 3):
    nd = robot.getFromDef("BIN_%d" % i)
    if nd is not None:
        p = nd.getPosition()
        r = nd.getField("rotation").getSFRotation()
        yaw = r[3] * (1.0 if r[2] >= 0 else -1.0)
        BINS.append({"node": nd, "tr": nd.getField("translation"),
                     "rot": nd.getField("rotation"), "pose": [p[0], p[1], yaw]})

_active_bin = None              # index into BINS while picking


def bin_pin_check(b):
    """Re-pin bin `b` at its target pose if it drifted. The dead-band is a
    full centimetre: a tighter pin FIGHTS solver jitter tick after tick and
    the repeated snap pumps energy into the loose contents (a queued bin
    once ejected a tube over its wall)."""
    tx, ty, tyaw = b["pose"]
    p = b["node"].getPosition()
    if (abs(p[0] - tx) > 0.010 or abs(p[1] - ty) > 0.010
            or abs(p[2] - BELT_TOP) > 0.010):
        b["tr"].setSFVec3f([tx, ty, BELT_TOP])
        b["rot"].setSFRotation([0.0, 0.0, 1.0, tyaw])
        try:
            b["node"].setVelocity([0.0] * 6)
        except Exception:
            pass


# ── SUCTION (vacuum) hold, frozen at the CONTACT pose ────────────────
# The hold is BREAKABLE: the seal has a shear budget. Each tick, before
# re-welding, measure how far physics fought the part away from where the
# weld placed it last tick (a free hold only accumulates ~1 mm of gravity
# per tick; a part SNAGGED on a wall, the robot or another part racks up
# much more). Past SEAL_SHEAR the seal lets go and the part drops where it
# is -- a snag becomes a clean, recoverable drop instead of a forced
# teleport through the obstacle.
_suck = None
SEAL_SHEAR = 0.008              # residual displacement that breaks the seal (m)
_seal_last = None               # where the weld placed the part last tick
_seal_broke = False             # set when the hold failed mid-carry


def suck_on(node):
    global _suck, _seal_last, _seal_broke
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    pos, _R_ = _tcp_pose()
    c = node.getPosition()
    d = [c[0] - pos[0], c[1] - pos[1], c[2] - pos[2]]   # measured: zero jump at engage
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass
    _suck = (node, node.getField("translation"), node.getField("rotation"), d, rot0)
    _seal_last = None
    _seal_broke = False


def suck_apply():
    global _suck, _seal_last, _seal_broke
    if _suck is None:
        return
    node, tr, rot, d, rot0 = _suck
    if _seal_last is not None:
        p = node.getPosition()
        resid = math.dist(p, _seal_last)
        if resid > SEAL_SHEAR:
            emit("[line]   GRIP BROKE (shear %.0f mm: snagged) -> part dropped"
                 % (resid * 1000.0))
            _seal_broke = True
            suck_off()
            return
    pos, _R_ = _tcp_pose()
    want = [pos[0] + d[0], pos[1] + d[1], pos[2] + d[2]]
    tr.setSFVec3f(want)
    rot.setSFRotation(rot0)
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass
    _seal_last = want


def suck_off():
    global _suck, _seal_last
    if _suck is not None:
        try:
            _suck[0].resetPhysics()
        except Exception:
            pass
    _suck = None
    _seal_last = None


# ── Background staging: the NEXT bin advances while this one is picked ─
STAGE_Y = 0.64                  # waiting point just behind the station
PARTS = None                    # set after discover_parts()
_staging = None                 # bin index currently advancing to STAGE_Y


def _staging_tick():
    """One tick of background bin advance: while the robot empties the
    active bin, the next one creeps from the queue to STAGE_Y (squaring its
    yaw on the way), so the bin swap is a short hop instead of the full
    in-feed travel. Same ensemble discipline as convey(): rigid rider
    teleport + velocity zeroing, riders recaptured every tick."""
    global _staging
    if _staging is None or PARTS is None:
        return
    b = BINS[_staging]
    bx, by, yaw = b["pose"]
    if abs(by - STAGE_Y) < 0.004 and abs(bx - 0.46) < 0.004 and abs(yaw) < 0.02:
        _staging = None
        return
    lin = CONVEY_SPEED * dt / 1000.0
    ang = YAW_RATE * dt / 1000.0
    nx = bx + max(-lin, min(lin, 0.46 - bx))
    ny = by + max(-lin, min(lin, STAGE_Y - by))
    nyaw = yaw + max(-ang, min(ang, -yaw))
    cyw, syw = math.cos(yaw), math.sin(yaw)
    riders = []
    for n, (nd, _k, pbi) in PARTS.items():
        if pbi != _staging:
            continue
        p = nd.getPosition()
        u = (p[0] - bx) * cyw + (p[1] - by) * syw
        v = -(p[0] - bx) * syw + (p[1] - by) * cyw
        if abs(u) <= 0.175 and abs(v) <= 0.175 and p[2] < BELT_TOP + 0.28:
            riders.append(nd)
    rz = _rz(nyaw - yaw)
    b["pose"] = [nx, ny, nyaw]
    b["tr"].setSFVec3f([nx, ny, BELT_TOP])
    b["rot"].setSFRotation([0.0, 0.0, 1.0, nyaw])
    try:
        b["node"].setVelocity([0.0] * 6)
    except Exception:
        pass
    for nd in riders:
        p = nd.getPosition()
        rr = _rotv(rz, [p[0] - bx, p[1] - by, 0.0])
        nd.getField("translation").setSFVec3f([nx + rr[0], ny + rr[1], p[2]])
        if abs(nyaw - yaw) > 1e-9:
            nR = _mat_mul(rz, _R(nd))
            nd.getField("rotation").setSFRotation(_mat3_to_axis_angle(nR))
        try:
            nd.setVelocity([0.0] * 6)
        except Exception:
            pass


def step_for(secs):
    """Advance the sim: bridge motion, vacuum hold, every bin pinned to its
    target pose, and the staged bin creeping forward in the background."""
    n = int(secs * 1000 / dt)
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if _suck is not None:
            suck_apply()
        for b in BINS:
            bin_pin_check(b)
        _staging_tick()
    return True


TCP_SPEED = 0.85                # m/s nominal tool speed for auto-durations
APPROACH_SPEED = 0.45           # m/s for descents toward contact (slower)


def pose(xyz, dur=None, settle=0.15):
    """Move the cup LIP to xyz, top-down. Duration auto-scales with the move
    distance (TCP_SPEED) unless given -- short hops stopped taking a fixed
    1.4 s each, roughly doubling cell throughput."""
    if dur is None:
        lip, _R0 = _tcp_pose()
        dur = min(1.6, max(0.3, math.dist(lip, xyz) / TCP_SPEED))
    bridge.act_set_tcp_pose(tuple(xyz), tcp_offset_z=OZ, duration_s=dur)
    step_for(dur + settle)


def wall_lean(x, y):
    """Near-wall lean for a pick at world (x, y), computed in the ACTIVE
    bin's own frame (the bin arrives yawed, so its walls are not world
    axis-aligned), then rotated back to world."""
    if _active_bin is None:
        return None
    bx, by, yaw = BINS[_active_bin]["pose"]
    cy, sy = math.cos(yaw), math.sin(yaw)
    dx, dy = x - bx, y - by
    u = dx * cy + dy * sy           # world -> bin frame
    v = -dx * sy + dy * cy
    lu = lv = 0.0
    if WALL_HALF - abs(u) < WALL_NEAR:
        lu = -math.copysign(1.0, u) * (1.0 - max(0.0, WALL_HALF - abs(u)) / WALL_NEAR)
    if WALL_HALF - abs(v) < WALL_NEAR:
        lv = -math.copysign(1.0, v) * (1.0 - max(0.0, WALL_HALF - abs(v)) / WALL_NEAR)
    m = math.hypot(lu, lv)
    if m < 1e-6:
        return None
    lu, lv = lu / m, lv / m
    lx = lu * cy - lv * sy          # bin frame -> world
    ly = lu * sy + lv * cy
    return lx, ly, MAX_TILT * math.sqrt(min(1.0, m))


_last_ik = "none"               # diagnostics: what the last approach commanded
_seal_gap = 0.0                 # measured seal depth of the current hold (m, <=0)


def approach_pose(x, y, z, lean, dur=None, settle=0.15):
    """Move the cup LIP to (x,y,z), tilted by `lean` when given; duration
    auto-scales with the move distance (APPROACH_SPEED) unless given. The
    tilted IK is GATED on its reported position error: an unconverged
    solution is never commanded (blind execution used to swing the arm
    wildly and once dropped a carried part onto the robot's own shoulder).
    If the full tilt won't converge, try free tool-yaw variants, then 60%
    and 30% tilt; the last resort is the plain top-down pose -- safe, and
    if a wall then blocks the wrist the contact gate simply reports NO
    TOUCH and the pick is retried."""
    global _last_ik
    if dur is None:
        lip, _R0 = _tcp_pose()
        dur = min(1.3, max(0.35, math.dist(lip, [x, y, z]) / APPROACH_SPEED))
    if lean is not None:
        lx, ly, theta = lean
        top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        # The suction cup is AXISYMMETRIC, so the tool's own yaw is free:
        # try 4 tool-yaw variants at each tilt scale -- full tilt with a
        # rotated wrist is feasible where the nominal wrist hits limits.
        # Degrading the tilt is the LAST resort (a weak tilt stalls on the
        # wall), and plain top-down the safe fallback.
        for scale in (1.0, 0.6, 0.3):
            for yawc in (0.0, 1.5708, -1.5708, 3.1416):
                cyc, syc = math.cos(yawc), math.sin(yawc)
                rzt = [[cyc, -syc, 0.0], [syc, cyc, 0.0], [0.0, 0.0, 1.0]]
                rt = _mat_mul(_mat_mul(_rot_axis([-ly, lx, 0.0], theta * scale), top), rzt)
                q, perr, rerr, _it = dls_ik_pose(IK["chain"], bridge._read_q(), [x, y, z],
                                                 rt, (0.0, 0.0, OZ), IK, bridge.joint_limits)
                if perr < 0.015 and rerr < 0.2:
                    _last_ik = "tilt%.0f%% yaw%.0f perr=%.0fmm" % (
                        scale * 100, math.degrees(yawc), perr * 1000)
                    bridge.act_set_joint_positions(q, duration_s=dur)
                    step_for(dur + settle)
                    return
        _last_ik = "fallback-vertical (no tilted IK converged)"
    else:
        _last_ik = "vertical"
    pose([x, y, z], dur=dur, settle=settle)


# ── AnyPick core (identical algorithm to omniarm6_anypick) ──────────────
def anypick_anchor(node, kind):
    """Anchor from the part's TRUE pose -- used only by the vacuum-seal
    sensor model (the cup's pressure switch must compare against the real
    surface) and by the end-of-line recovery sweep."""
    return anypick_anchor_pose(node.getPosition(), _R(node), kind)


def seal_check(node, kind, lip):
    """SEAL-SENSOR model (the cup's vacuum/pressure switch): (gap, lateral)
    of the lip vs the part's TRUE surface. A real switch seals anywhere on
    the part, so this measures against the nearest sealable surface:
      * tubes: point-to-CAPSULE-AXIS -- lateral is distance from the axis
        line (a cup anywhere ALONG the barrel seals fine; the old single
        mid-point candidate read a perfectly-seated cup 35 mm off), gap is
        height above the LOCAL curved surface;
      * boxes: the nearest up-facing face-centre candidate."""
    if kind == "tube":
        bc, r, hh = KIND_GEOM["tube"]["caps"][0]
        c = node.getPosition()
        a = _rotv(_R(node), [0.0, 0.0, 1.0])
        t = max(-hh, min(hh, sum((lip[i] - c[i]) * a[i] for i in range(3))))
        q = [c[i] + t * a[i] for i in range(3)]
        d = math.hypot(lip[0] - q[0], lip[1] - q[1])
        surf_z = q[2] + math.sqrt(max(0.0, r * r - d * d))
        return lip[2] - surf_z, d, q
    cands = _anchor_candidates(node.getPosition(), _R(node), kind)
    # STICKY GUM grips any face it touches, not only near-horizontal ones:
    # accept candidates down to 20-deg-off-vertical faces (the pad still
    # needs to PRESS onto the surface from above to bond).
    up = [cd for cd in cands if cd[2] >= 0.35] or cands
    a2 = min(up, key=lambda cd: math.hypot(cd[1][0] - lip[0], cd[1][1] - lip[1]))[1]
    return lip[2] - a2[2], math.hypot(lip[0] - a2[0], lip[1] - a2[1]), a2


def _anchor_candidates(c, R, kind):
    """All candidate suction anchors for a part of `kind` at pose (c, R), as
    (world_z, world_point, normal_up) tuples."""
    g = KIND_GEOM[kind]
    cands = []
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
        aw = _rotv(R, [0.0, 0.0, 1.0])
        for s in (1.0, -1.0):
            pb = [bc[0], bc[1], bc[2] + s * (hh + r)]
            pw = _world(c, R, pb)
            cands.append((pw[2], pw, aw[2] * s))
        if abs(aw[2]) < 0.92:
            w = [-aw[0] * aw[2], -aw[1] * aw[2], 1.0 - aw[2] * aw[2]]
            n = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) or 1.0
            w = [w[0] / n, w[1] / n, w[2] / n]
            cw = _world(c, R, bc)
            cands.append((cw[2] + r * w[2], [cw[0] + r * w[0], cw[1] + r * w[1], cw[2] + r * w[2]], w[2]))
    return cands


def anypick_anchor_pose(c, R, kind):
    """The best suction anchor for a part of `kind` at pose (c, R) -- works
    on PERCEPTION estimates just as well as on true poses."""
    if kind == "unknown":
        # MODEL-FREE: the camera already published a depth-image SURFACE
        # anchor as the "position" -- it IS the grasp point.
        return list(c)
    if kind == "tube":
        # A capsule's recognition ORIENTATION is degenerate (square cross-
        # section: the reported frame comes back axis-permuted/spun, 93 deg
        # off in calibration) -- but a lying tube's top-line anchor needs no
        # orientation at all: centre + radius * z-up.
        r = KIND_GEOM["tube"]["caps"][0][1]
        return [c[0], c[1], c[2] + r]
    cands = _anchor_candidates(c, R, kind)
    up = [cd for cd in cands if cd[2] >= MIN_UP]
    pool = up if up else cands
    bx, by = (BINS[_active_bin]["pose"][:2] if _active_bin is not None else STATION)
    pool.sort(key=lambda cd: (-round(cd[0] / 0.004),
                              math.hypot(cd[1][0] - bx, cd[1][1] - by)))
    return pool[0][1]


def part_min_z_pose(c, R, kind):
    if kind == "unknown":
        return BELT_TOP + 0.023                 # model-free: assume it may
                                                # extend down to the bin floor
    if kind == "tube":
        return c[2] - KIND_GEOM["tube"]["caps"][0][1]   # lying: orientation-free
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
    return min(CARRY_MAX, max(CARRY_MIN, WALL_CLEAR + ext))


def press_to_contact(node, kind, lean, anchor, est=None):
    """LOOK-THEN-MOVE descent onto the perception-estimated `anchor` (the
    camera is occluded by the arm down here, so there is no visual servo:
    the snapshot is what a real cell would have), pressing deeper in steps.
    The vacuum may only engage when the SEAL SENSOR -- the cup's pressure
    switch, a local sensor modelled from the true surface geometry --
    confirms the lip is ON the part (gap <= GRAB_EPS, laterally on it).
    Perception error beyond what the press can absorb fails the seal and
    the pick is retried from a fresh snapshot."""
    gap = horiz = 1e9
    for i, depth in enumerate(PRESS_DEPTHS):
        if i == 0:
            # Stage the TILTED orientation at +10 cm first: the low solve then
            # starts from a nearby seed and converges where a cold one diverged.
            approach_pose(anchor[0], anchor[1], anchor[2] + depth + 0.10, lean, settle=0.1)
        approach_pose(anchor[0], anchor[1], anchor[2] + depth, lean,
                      dur=(None if i == 0 else 0.35), settle=(0.2 if i == 0 else 0.1))
        lip, _R_ = _tcp_pose()
        if kind == "unknown":
            # MODEL-FREE pressure switch: no registry, but the surface gauge
            # is still TRUE-referenced -- the offset between the depth-
            # camera anchor and the part body was frozen at pick start, so
            # the live surface height tracks the part even if it settled.
            # (Checking only against the static anchor let the pad stop a
            # visible few mm short and weld anyway -- the magnet, again.)
            hh = (est or {}).get("halfh")
            surf = node.getPosition()[2] + hh if hh is not None else anchor[2]
            gap = lip[2] - surf
            horiz = math.hypot(lip[0] - anchor[0], lip[1] - anchor[1])
            near = anchor
        else:
            gap, horiz, near = seal_check(node, kind, lip)   # grip sensor: TRUE surface
        if kind == "tube" and 0.0 < horiz <= 0.09:
            # SEAL-SEEK (tubes only): a compliant cup probes toward the
            # surface the sensor feels; re-aim the next press step there.
            # The 9 cm basin covers both post-glance roll drift AND a
            # LEANING tube, whose axis sits up to ~6 cm from the flat-lying
            # assumption (recognition orientation is degenerate for
            # capsules, so the lean direction is unobservable).
            anchor = [near[0], near[1], max(anchor[2], near[2] + 0.018)]
        if -PRESS_IN_MAX <= gap <= GRAB_EPS and horiz <= HORIZ_EPS:
            global _seal_gap
            _seal_gap = gap
            emit("[line]   grip ok: gap=%.1fmm lateral=%.1fmm%s"
                 % (gap * 1000.0, horiz * 1000.0,
                    " (tilted %.0f deg)" % math.degrees(lean[2]) if lean else ""))
            return True
    emit("[line]   NO GRIP (gap=%.1fmm lateral=%.1fmm, ik=%s) -> pad never touched, retreating"
         % (gap * 1000.0, horiz * 1000.0, _last_ik))
    return False


def anypick(name, node, est, slot):
    """One perception-driven AnyPick: shape, pose and grasp anchor all come
    from the camera estimate `est` (kind/pos/R); the scene tree is only
    touched by the seal-sensor model and the physics bookkeeping."""
    kind = est["kind"]
    anchor = anypick_anchor_pose(est["pos"], est["R"], kind)
    if math.hypot(anchor[0], anchor[1]) > REACH_MAX:
        emit("[line]   anchor %.2f m out -> unreachable, skipping" % math.hypot(anchor[0], anchor[1]))
        return False
    z0 = node.getPosition()[2]
    ext = max(0.0, anchor[2] - part_min_z_pose(est["pos"], est["R"], kind))
    carry = anypick_carry_z(ext)
    if kind == "tube":
        # CONFIRM GLANCE for rollers: stage OFFSET to the south so the cup
        # does not block the camera's line of sight to the target, take a
        # fresh estimate from there, and descend on it -- cuts the snapshot-
        # to-touch window (which a slowly drifting tube outruns) to <1 s.
        pose([anchor[0], anchor[1] - 0.13, carry], settle=0.1)
        fresh = camera_detection().get(name)
        if fresh is not None:
            est = fresh
            anchor = anypick_anchor_pose(est["pos"], est["R"], kind)
            ext = max(0.0, anchor[2] - part_min_z_pose(est["pos"], est["R"], kind))
            carry = anypick_carry_z(ext)
    else:
        pose([anchor[0], anchor[1], carry])              # stage above the anchor
    lean = wall_lean(anchor[0], anchor[1])
    emit("[line]   est anchor=(%.3f,%.3f,%.3f) hangs %.3f below; carry lip %.2f"
         % (anchor[0], anchor[1], anchor[2], ext, carry))
    if kind == "unknown":
        # Model-free pressure-switch gauge. NOT anchor-referenced: the depth
        # anchor inherits the range image's height bias (ray geometry +
        # patch sampling on narrow limbs), and a frozen anchor offset
        # faithfully reproduced that bias as a visible grip gap -- the
        # plus/U magnet. In the bin the part RESTS ON THE FLOOR (known
        # height), so its true top is centre + (centre - floor): an
        # unbiased, registry-free gauge.
        if _active_bin is not None:
            est["halfh"] = node.getPosition()[2] - (BELT_TOP + 0.023)
        else:
            est["halfh"] = anchor[2] - node.getPosition()[2]
        # The depth anchor's Z is biased high by up to ~2 cm (range-image
        # geometry), so a press schedule referenced to it bottoms out ABOVE
        # the real surface -- the lingering hover on these parts. The camera
        # contributes only the grasp XY; the press references the unbiased
        # rest-geometry surface, letting the pad land flush (slight press-in)
        # like every known kind.
        anchor[2] = node.getPosition()[2] + est["halfh"]
    if not press_to_contact(node, kind, lean, anchor, est):
        approach_pose(anchor[0], anchor[1], carry, lean)
        pose([0.32, -0.22, carry])                       # park CLEAR of the camera
        return False
    suck_on(node)
    step_for(0.2)
    if _suck is not None:
        # The MEASURED hold tells the true hang: a pad gripped off-centre on
        # a leaning part carries it lower than the estimate predicted, and an
        # under-estimated carry height snagged parts on the bin wall (grip
        # broke, part rode away with the bin). 5.5 cm covers the largest
        # part extent below its centre.
        hang = -_suck[3][2] + 0.055
        carry = min(CARRY_MAX, max(carry, WALL_CLEAR + hang))
    lp = _tcp_pose()[0]
    # GENTLE EXTRACTION: the first 6 cm of lift is slow and vertical so the
    # held part disengages its neighbours instead of plowing them -- fast
    # extraction scattered and PROPPED neighbouring parts (the source of the
    # unpickable leaning stacks).
    approach_pose(lp[0], lp[1], min(carry, lp[2] + 0.06), lean, dur=0.55, settle=0.05)
    approach_pose(lp[0], lp[1], carry, lean)
    pose([lp[0], lp[1], carry], dur=0.5)
    if node.getPosition()[2] < z0 + LIFT_RISE_OK:
        suck_off()
        step_for(0.2)
        return False
    sx, sy = slot
    # Compensate the GRIP OFFSET at the drop: with a noisy estimate the pad
    # grips off-centre, so the part hangs offset from the lip; aim the lip
    # so the PART lands on the slot. CAPPED at 6 cm (a huge measured offset
    # means a weird grip -- compensating it in full aimed releases clean off
    # the tray), and the final release point is CLAMPED inside the tray
    # bounds either way: parts must never be released over a wall.
    if _suck is not None:
        sx -= max(-0.06, min(0.06, _suck[3][0]))
        sy -= max(-0.06, min(0.06, _suck[3][1]))
    zx0, zx1, zy0, zy1 = KIND_ZONE[kind]
    # LONG parts (tubes, bars, brackets) can overhang the lip by ~6 cm when
    # gripped near one end -- a 6 cm clamp still let the far end clip the
    # tray wall on the way down and get slapped loose. Keep their WHOLE
    # descent corridor inside the walls.
    marg = 0.09 if kind in ("tube", "bar", "ltube", "tbar") else 0.06
    sx = max(zx0 + marg, min(zx1 - marg, sx))
    sy = max(zy0 + marg, min(zy1 - marg, sy))
    # Waypoint IN FRONT of the base first for the SOUTH trays: a direct
    # diagonal from a north-edge pick to a west tray swings the hanging part
    # across the robot's own column. The black (unknown) bin sits NORTH-WEST
    # -- its direct route already clears the base.
    if sy < 0:
        pose([0.40, -0.12, carry], settle=0.05)
    pose([sx, sy, carry], settle=0.1)
    # Raise the drop by the measured SEAL DEPTH: a pad gripped N mm into the
    # part carries it N mm higher relative to the lip, and dropping at the
    # nominal height pressed one tube INTO the tray floor -- the release
    # then ejected it metres (deep-interpenetration resolution). Tubes get a
    # GENTLER release (lower drop): they roll on any bounce and can hop the
    # tray wall.
    # PLACE tubes, don't drop them (a bounce becomes a roll). The deep-grip
    # raise is NEVER capped: capping it once released a 24 mm-deep-gripped
    # tube 12 mm INTO the tray floor and the interpenetration pop ejected it
    # clean over the wall. (Tubes can't deep-grip any more anyway: their
    # press gate stops at -8 mm.)
    # PLACE tubes low (a bounce becomes a roll); the deep-grip raise stays
    # uncapped but tubes cannot deep-grip any more (their press gate stops
    # at -8 mm), so releases are always clean.
    drop_b = 0.03 if kind == "tube" else DROP_BOTTOM
    zt = drop_b + ext + max(0.0, -_seal_gap)
    # STRICTLY TOP-DOWN placement: the arm interpolates in JOINT space, so a
    # single long descend command arcs sideways mid-path and SWINGS the
    # hanging part into the tray wall. Square up just above the walls first,
    # then descend in short stepped segments whose joint-space arc is
    # millimetres -- a true vertical placement.
    pose([sx, sy, 0.26], settle=0.08)
    pose([sx, sy, (0.26 + zt) / 2.0], dur=0.45, settle=0.05)
    pose([sx, sy, zt], dur=0.45, settle=0.1)
    suck_off()
    step_for(0.4)
    pose([sx, sy, carry])
    if _seal_broke:
        # The hold failed mid-carry; the part lies wherever it fell. Count
        # the pick only if it left the bin -- the recovery sweep (or a
        # retry) deals with the rest.
        return _active_bin is None or not in_bin_i(node, _active_bin)
    return True


# ── Scene bookkeeping ────────────────────────────────────────────────
def discover_parts():
    """{name: (node, kind, bin_index)} for every DEF PART_<n>; bin_index by
    nearest bin centre at startup."""
    parts = {}
    i = 1
    misses = 0
    while misses < 8:
        name = "PART_%d" % i
        node = robot.getFromDef(name)
        if node is not None:
            kind = (node.getField("name").getSFString() or "cube").split("_")[0]
            if kind not in KIND_GEOM:
                kind = "cube"
            p = node.getPosition()
            bi = min(range(len(BINS)),
                     key=lambda k: math.hypot(p[0] - BINS[k]["pose"][0],
                                              p[1] - BINS[k]["pose"][1]))
            parts[name] = (node, kind, bi)
            ID_TO_NAME[node.getId()] = name
            misses = 0
        else:
            misses += 1
        i += 1
    return parts


def camera_detection():
    """{name: {kind, pos, R}} -- world-frame PERCEPTION estimates published
    by the anypick_cam controller (occlusion on: only unblocked parts). The
    estimates carry +-3 mm noise; everything the grasp needs (which part,
    what shape, where) comes from here, not from the scene tree."""
    out = {}
    if _cam_data is None:
        return out
    raw = _cam_data.getSFString() or ""
    for tok in raw.split(","):
        f = tok.split(":")
        if len(f) != 9:
            continue
        try:
            name = ID_TO_NAME.get(int(f[0]))
            kind = f[1]
            pos = [float(f[2]), float(f[3]), float(f[4])]
            R = _rot_axis([float(f[5]), float(f[6]), float(f[7])], float(f[8]))
        except (ValueError, ZeroDivisionError):
            continue
        if name and kind in KIND_GEOM:
            # Recognition positions are BBOX centres; shift to the body
            # frame origin that KIND_GEOM's primitives are expressed in.
            off = _rotv(R, KIND_BBOXC[kind])
            out[name] = {"kind": kind,
                         "pos": [pos[0] - off[0], pos[1] - off[1], pos[2] - off[2]],
                         "R": R}
    return out


def pz(node):
    p = node.getPosition()
    return [round(p[0], 3), round(p[1], 3), round(p[2], 3)]


def in_bin_i(node, bi):
    # Tested in the BIN'S ROTATED FRAME (an axis-aligned box misclassifies
    # the corners of a yawed bin -- a corner part once got orphaned on the
    # belt because the conveyor's rider test missed it).
    p = node.getPosition()
    bx, by, yaw = BINS[bi]["pose"]
    cy, sy = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - bx, p[1] - by
    u = dx * cy + dy * sy
    v = -dx * sy + dy * cy
    return abs(u) <= 0.21 and abs(v) <= 0.21 and p[2] < BELT_TOP + 0.28


def in_any_bin(node):
    return any(in_bin_i(node, k) for k in range(len(BINS)))


def on_zone(node, kind):
    p = node.getPosition()
    x0, x1, y0, y1 = KIND_ZONE[kind]
    return x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p[2] < 0.16


def true_est(node, kind):
    """An est-shaped dict from the TRUE pose -- the camera-blind fallback
    (and the end-of-line recovery, where the cell knows where it dropped
    things)."""
    return {"kind": kind, "pos": node.getPosition(), "R": _R(node)}


def next_pick(parts, done, bi):
    """Next part to pick FROM BIN bi, chosen from the camera's PERCEPTION
    estimates (highest estimated part still in the bin). Scene-tree fallback
    only when the camera is fully occluded, and logged as such."""
    seen = camera_detection()
    cands = [(n, parts[n][0], seen[n]) for n in seen
             if n in parts and n not in done and parts[n][2] == bi
             and in_bin_i(parts[n][0], bi)]
    src = "cam"
    if not cands:
        cands = [(n, parts[n][0], true_est(parts[n][0], parts[n][1])) for n in parts
                 if n not in done and parts[n][2] == bi and in_bin_i(parts[n][0], bi)]
        src = "scene"
    if not cands:
        return None
    cands.sort(key=lambda c: c[2]["pos"][2], reverse=True)
    return cands[0][0], cands[0][1], cands[0][2], src


# ── Conveyor: kinematic ensemble transport ───────────────────────────
YAW_RATE = 0.15                 # rad/s yaw easing (gentle: fast pivots pop light
                                # parts over the walls via contact impulses)


def convey(bi, waypoints, yaw_to=None, parts=None, speed=None):
    """Advance bin `bi` and everything inside it along `waypoints` (a list of
    (x, y)) at CONVEY_SPEED, easing its yaw to `yaw_to` over the move. Time-
    parameterized, so a pure in-place PIVOT (zero path length) also animates
    at YAW_RATE -- that's how the corner repositions a bin for a hard pick.
    The bin and its riding parts move as a RIGID ENSEMBLE each tick (pose set
    + zero velocity): deterministic transport, contents keep their settled
    arrangement."""
    b = BINS[bi]

    def riders_now():
        # Re-evaluated EVERY tick, not snapshotted: a part missed at the
        # start used to be left behind on the belt for the rest of the run.
        # STRICTLY INTERIOR (0.175 < the inner wall faces at 0.18): even a
        # wall-hugging cube's centre is at 0.155, so everything resting
        # inside is captured -- but never a part perched in the wall band,
        # which the teleport would EMBED in the wall collider (one tube was
        # ballistically ejected hundreds of km by exactly that).
        out = []
        if parts:
            ox, oy, oyaw = b["pose"]
            cyw, syw = math.cos(oyaw), math.sin(oyaw)
            for n, (nd, _k, pbi) in parts.items():
                if pbi != bi:
                    continue
                p = nd.getPosition()
                u = (p[0] - ox) * cyw + (p[1] - oy) * syw
                v = -(p[0] - ox) * syw + (p[1] - oy) * cyw
                if abs(u) <= 0.175 and abs(v) <= 0.175 and p[2] < BELT_TOP + 0.28:
                    out.append(nd)
        return out

    pts = [tuple(b["pose"][:2])] + [tuple(w) for w in waypoints]
    segs = []
    total = 0.0
    for k in range(len(pts) - 1):
        L = math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
        segs.append((pts[k], pts[k + 1], L))
        total += L
    yaw0 = b["pose"][2]
    dyaw_total = 0.0 if yaw_to is None else (yaw_to - yaw0)
    dur = max(total / (speed or CONVEY_SPEED), abs(dyaw_total) / YAW_RATE, 0.1)
    n = max(1, int(dur * 1000.0 / dt))
    for i in range(1, n + 1):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        t = i / float(n)
        s = total * t
        nx, ny = pts[-1]
        for (p0, p1, L) in segs:
            if s <= L or (p0, p1, L) is segs[-1]:
                f = 0.0 if L < 1e-9 else min(1.0, s / L)
                nx, ny = p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f
                break
            s -= L
        nyaw = yaw0 + dyaw_total * t
        ox, oy, oyaw = b["pose"]
        riders = riders_now()                       # vs the OLD pose, pre-update
        dy_step = nyaw - oyaw
        rz = _rz(dy_step)
        b["pose"] = [nx, ny, nyaw]
        b["tr"].setSFVec3f([nx, ny, BELT_TOP])
        b["rot"].setSFRotation([0.0, 0.0, 1.0, nyaw])
        try:
            b["node"].setVelocity([0.0] * 6)
        except Exception:
            pass
        for nd in riders:
            p = nd.getPosition()
            rr = _rotv(rz, [p[0] - ox, p[1] - oy, 0.0])
            nd.getField("translation").setSFVec3f([nx + rr[0], ny + rr[1], p[2]])
            if abs(dy_step) > 1e-9:
                nR = _mat_mul(rz, _R(nd))
                nd.getField("rotation").setSFRotation(_mat3_to_axis_angle(nR))
            try:
                nd.setVelocity([0.0] * 6)
            except Exception:
                pass
        for ob in BINS:
            if ob is not b:
                bin_pin_check(ob)
    return True


# ── Station repositioning: bring a hard part into the sweet spot ─────
# The arm picks reliably in a SWEET BOX in front of it; a part in the far
# corner of a yawed bin (x up to ~0.76) or hugging the near wall (x ~0.28,
# too close to the base for the tilted IK) is out of it. The corner station
# can SLIDE the bin along the out leg and PIVOT it, exactly like a corner
# transfer, so the target part comes to the arm instead.
SWEET_X = (0.38, 0.58)
SWEET_Y = (-0.16, 0.16)
STATION_X = (0.42, 0.62)        # how far the bin centre may sit on the corner


def _sweet_score(x, y):
    """0 inside the sweet box; distance outside it otherwise."""
    dx = max(SWEET_X[0] - x, 0.0, x - SWEET_X[1])
    dy = max(SWEET_Y[0] - y, 0.0, y - SWEET_Y[1])
    return math.hypot(dx, dy)


def jolt_off_wall(node, bi, attempt=0):
    """Physical declutter for a part hugging a wall or wedged in a corner
    (where no wrist tilt can reach it): slide the BIN ONLY -- no rider
    teleport -- so the wall collider physically SHOVES the part toward the
    bin centre. SINGLE-AXIS: a diagonal shove on a corner part cancels
    itself (the part just slides along the other wall); push off the
    deepest wall only, alternating axes across attempts."""
    p = node.getPosition()
    b = BINS[bi]
    bx, by, yaw = b["pose"]
    u, v = p[0] - bx, p[1] - by                       # bin squared: frame = world
    du = WALL_HALF - abs(u)                           # depth into each keep-out
    dv = WALL_HALF - abs(v)
    pick_x = (du < dv) ^ (attempt % 2 == 1)           # deepest first, alternate
    if pick_x:
        nx = min(0.64, max(0.40, bx - math.copysign(0.06, u)))
        ny = by
    else:
        nx = bx
        ny = min(0.10, max(-0.10, by - math.copysign(0.06, v)))
    if abs(nx - bx) < 0.01 and abs(ny - by) < 0.01:
        return
    emit("[line]   ~~ wall-shove: slide bin under the parts (%.2f,%.2f) ~~" % (nx, ny))
    # SLOW: the wall pushes 50 g parts; at belt speed the kinematic wall
    # catapults them clean out of the bin.
    convey(bi, [(nx, ny)], yaw_to=None, parts=None, speed=0.05)
    step_for(0.6)                                      # shoved parts settle


def conveyor_jog(bi, parts):
    """Freeze rolling contents: a short RIDER-ATTACHED slide velocity-zeroes
    every captured part each tick of the move. The solver has no rolling
    friction, so a nudged capsule rolls forever -- this is the conveyor's
    deterministic way to stop it."""
    b = BINS[bi]
    bx, by, _yaw = b["pose"]
    nx = min(0.64, max(0.40, bx + (0.04 if bx < 0.52 else -0.04)))
    emit("[line]   ~~ conveyor jog: freeze rolling contents ~~")
    convey(bi, [(nx, by)], yaw_to=None, parts=parts, speed=0.10)
    step_for(0.3)


def reposition_for(node, bi, parts):
    """If `node` sits outside the sweet box, slide/pivot bin `bi` on the
    corner so it lands inside (best (shift, pivot) over a small search),
    carrying the whole ensemble. No-op when the part is already fine."""
    b = BINS[bi]
    bx, by, yaw = b["pose"]
    p = node.getPosition()
    cur = _sweet_score(p[0], p[1])
    if cur < 0.005:
        return
    # SLIDE ONLY -- no mid-session pivots. Ensemble pivots fight the live
    # contacts and leak parts (one tube popped over a wall, another rode a
    # wall top out of the cell); translation-with-riders has never leaked.
    # The bin was squared to yaw 0 on arrival, so a slide along the out leg
    # directly fixes the x of any part; |y| outliers rely on the tilt and,
    # as a last resort, the slow wall-shove.
    ox = p[0] - bx
    cx = min(STATION_X[1], max(STATION_X[0], 0.48 - ox))
    if _sweet_score(cx + ox, p[1]) >= cur - 0.02 or abs(cx - bx) < 0.02:
        return None                                 # no slide helps
    emit("[line]   ~~ repositioning bin: slide to x=%.2f ~~" % cx)
    convey(bi, [(cx, by)], yaw_to=None, parts=parts)
    step_for(0.5)                                   # contents settle
    return (cx - bx, 0.0)                           # odometry for stale estimates


# ── Run ──────────────────────────────────────────────────────────────
step_for(5.0)                                  # parts settle into their bins
parts = discover_parts()
PARTS = parts
per_bin = {k: sum(1 for n in parts if parts[n][2] == k) for k in range(len(BINS))}
emit("[line] start    %d parts in %d bins: %s" % (
    len(parts), len(BINS), ", ".join("bin%d=%d" % (k + 1, per_bin[k]) for k in per_bin)))

# ── DATASET MODE: shuffle parts under the camera instead of picking ──
# With ANYPICK_DATASET=1 each bin is conveyed to the station and its parts
# are teleported to random poses ~40 times while anypick_cam (with
# ANYPICK_CAPTURE=<dir>) saves labelled crops -- the recognition COLOUR is
# the labelling oracle, the classifier must learn SHAPE from the image.
if os.environ.get("ANYPICK_DATASET"):
    import random as _rnd
    _r = _rnd.Random(11)
    _SHUF = [(-0.10, -0.09), (0.02, -0.10), (0.12, -0.08),
             (0.10, 0.09), (-0.02, 0.10), (-0.12, 0.08), (0.0, 0.0)]
    emit("[line] DATASET MODE: shuffling parts under the camera")
    for bi in range(len(BINS)):
        if _staging == bi:
            _staging = None
        convey(bi, [STATION], yaw_to=0.0, parts=parts)
        _active_bin = bi
        mine = [n for n in parts if parts[n][2] == bi]
        for _shot in range(40):
            slots = list(_SHUF)
            _r.shuffle(slots)
            for k, n in enumerate(mine):
                nd, kind, _b = parts[n]
                u, v = slots[k % len(slots)]
                u += _r.uniform(-0.015, 0.015)
                v += _r.uniform(-0.015, 0.015)
                yaw = _r.uniform(0.0, 6.28)
                if kind == "tube":
                    rr = _mat_mul(_rz(yaw), _rot_axis([0.0, 1.0, 0.0], 1.5708))
                    rotf = _mat3_to_axis_angle(rr)
                    z = 0.103 + 0.021 + 0.004
                else:
                    rotf = [0.0, 0.0, 1.0, yaw]
                    zh = max([bh[2] for _c, bh in KIND_GEOM[kind]["boxes"]] or [0.02])
                    z = 0.103 + zh + 0.005
                nd.getField("translation").setSFVec3f([STATION[0] + u, STATION[1] + v, z])
                nd.getField("rotation").setSFRotation(rotf)
                try:
                    nd.setVelocity([0.0] * 6)
                except Exception:
                    pass
            step_for(0.55)                     # settle; the camera captures
        _active_bin = None
        convey(bi, [(PARK_X[bi], 0.0)], yaw_to=0.0, parts=parts)
    emit("[line] DATASET DONE")
    try:
        robot.simulationQuit(0)
    except Exception:
        pass
    raise SystemExit


done = set()
slot_i = {t: 0 for t in set(TRAY_OF.values())}
for bi in range(len(BINS)):
    yaw_arrive = BINS[bi]["pose"][2]
    emit("[line] === BIN %d: conveying to the station (arrives yaw %.0f deg, "
         "squared by the corner) ===" % (bi + 1, math.degrees(yaw_arrive)))
    bridge.act_reset_to_home()
    step_for(0.8)
    if _staging == bi:
        _staging = None                        # take over from the background mover
    # SQUARE ON ARRIVAL: ease the yaw to 0 over the approach, like a corner
    # squaring guide (mostly already done while the bin STAGED in the
    # background during the previous bin's picking). The pick session then
    # runs in the proven axis-aligned geometry.
    if not convey(bi, [STATION], yaw_to=0.0, parts=parts):
        break
    if bi + 1 < len(BINS):
        _staging = bi + 1                      # next bin advances in the background
    _active_bin = bi
    step_for(1.0)                              # contents settle after transport
    if bi == 0:
        # Perception sanity check, once the first bin is under the camera:
        # catches a camera-frame convention error immediately instead of as
        # mysterious downstream misses. Per part: position + orientation err.
        _seen0 = camera_detection()
        for n in sorted(_seen0):
            if n not in parts:
                continue
            e = math.dist(_seen0[n]["pos"], parts[n][0].getPosition())
            rt = _R(parts[n][0])
            re = _seen0[n]["R"]
            tr = sum(re[i][0] * rt[i][0] + re[i][1] * rt[i][1] + re[i][2] * rt[i][2]
                     for i in range(3))
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))
            emit("[line] camera calib %s (%s/%s): pos err %.1f mm, rot err %.0f deg"
                 % (n, _seen0[n]["kind"], parts[n][1], 1000.0 * e, ang))
    tries = {}
    shaken = set()
    guard = 0
    while guard < 90:
        guard += 1
        step_for(0.15)
        pick = next_pick(parts, done, bi)
        if pick is None:
            break                              # this bin is empty
        name, node, est, src = pick
        tries[name] = tries.get(name, 0) + 1
        emit("[line] picking %s (%s, %s, try %d) @ est %s" % (
            name, est["kind"], src, tries[name],
            [round(v, 3) for v in est["pos"]]))
        if tries[name] >= 3:
            if parts[name][1] == "tube":
                conveyor_jog(bi, parts)             # NEVER shove a tube: a rider-less
                                                    # shove spins it up and it never stops
            else:
                jolt_off_wall(node, bi, tries[name])  # shove it off its wall
            est = None                              # the part MOVED: re-observe
        moved = reposition_for(node, bi, parts)     # bring a hard part to the arm
        if est is not None and moved:
            # Conveyor odometry: a slide moves the riders by exactly the
            # bin's delta, so the snapshot stays valid after correction.
            est["pos"][0] += moved[0]
            est["pos"][1] += moved[1]
        if est is None:
            step_for(0.3)                           # arm parked clear -> re-observe
            est = camera_detection().get(name) or true_est(node, parts[name][1])
        if est["kind"] == "tube" and src == "cam":
            # Capsules ROLL (the solver has no rolling resistance), and a
            # look-then-move snapshot can't hit a moving target -- the 31 mm
            # "calibration error" on a tube was simply the tube still
            # rolling. Confirm stillness across consecutive observations;
            # a tube that won't settle gets FROZEN by a conveyor jog.
            still = False
            for _w in range(5):
                p_prev = est["pos"]
                step_for(0.25)
                fresh = camera_detection().get(name)
                if fresh is None:
                    break
                est = fresh
                if math.dist(p_prev, est["pos"]) < 0.006:
                    still = True
                    break
            if not still:
                conveyor_jog(bi, parts)
                est = camera_detection().get(name) or true_est(node, parts[name][1])
        kind = est["kind"]
        slot = KIND_SLOTS[kind][slot_i[TRAY_OF[kind]] % len(KIND_SLOTS[kind])]
        if anypick(name, node, est, slot):
            done.add(name)
            slot_i[TRAY_OF[kind]] += 1
        elif not in_bin_i(node, bi):
            done.add(name)
        elif tries[name] >= 4:
            if name not in shaken:
                # Recirculate ONCE: a propped/stacked part often lies flat
                # after a shake; a part that defies a full second round is
                # retired (endless shaking just thrashes the pile).
                emit("[line]   %s defies picking -> shake the bin, one more round" % name)
                shaken.add(name)
                if parts[name][1] == "tube":
                    conveyor_jog(bi, parts)
                else:
                    jolt_off_wall(node, bi, tries[name])
                step_for(0.8)
                tries[name] = 0
            else:
                done.add(name)
                emit("[line]   -> %s unliftable after two rounds" % name)
    left = [n for n in parts if parts[n][2] == bi and in_bin_i(parts[n][0], bi)]
    emit("[line] === BIN %d emptied (%s left inside); conveying away ===" %
         (bi + 1, len(left) or "none"))
    _active_bin = None
    bridge.act_reset_to_home()
    step_for(0.8)
    convey(bi, [(PARK_X[bi], STATION[1])], yaw_to=0.0, parts=parts)

# RECOVERY: re-pick anything off its matching tray (missed drops, rolls).
_active_bin = None
rtries = {}
for _ in range(40):
    bad = next((n for n in parts
                if not on_zone(parts[n][0], parts[n][1])
                and not in_any_bin(parts[n][0])
                and rtries.get(n, 0) < 3), None)
    if bad is None:
        break
    node, kind, _bi = parts[bad]
    slot = KIND_SLOTS[kind][slot_i[TRAY_OF[kind]] % len(KIND_SLOTS[kind])]
    emit("[line] RECOVER %s (%s) from %s -> %s tray" % (bad, kind, pz(node), kind))
    if anypick(bad, node, true_est(node, kind), slot):
        slot_i[TRAY_OF[kind]] += 1
    else:
        rtries[bad] = rtries.get(bad, 0) + 1

# Final tally.
bridge.act_reset_to_home()
step_for(1.2)
n = len(parts)
emptied = sum(1 for nm in parts if not in_any_bin(parts[nm][0]))
sorted_ok = sum(1 for nm in parts if on_zone(parts[nm][0], parts[nm][1]))
remaining = ["%s@%s" % (nm, pz(parts[nm][0])) for nm in parts if in_any_bin(parts[nm][0])]
stray = ["%s@%s" % (nm, pz(parts[nm][0])) for nm in parts
         if not in_any_bin(parts[nm][0]) and not on_zone(parts[nm][0], parts[nm][1])]
ok = emptied >= 0.9 * n and sorted_ok >= 0.75 * n
emit("[line] RESULT emptied=%d/%d sorted=%d/%d remaining=%s stray=%s | wall=%.1fs sim=%.1fs"
     % (emptied, n, sorted_ok, n, remaining, stray, time.time() - t0, robot.getTime()))
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
