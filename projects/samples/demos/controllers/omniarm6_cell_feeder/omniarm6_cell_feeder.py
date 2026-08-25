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

"""ASSEMBLY CELL -- FEEDER arm (base at origin).

Empties the parts bin with the UNIVERSAL depth-only pick stack
(universal_cam anchors, no registry -- see omniarm6_universal_pick.py, whose
pick machinery this reuses verbatim) and sets each part down on the
HANDOFF PAD for the builder arm.

Coordination is stigmergic: the feeder only places when the pad is EMPTY
(no part node within PAD_R of the pad centre). No IPC.

The pad sits near the reach limit (radius 0.72 m), so the pad leg runs a
LOW-PROFILE route: normal carry height over the bin, then a lowered
waypoint toward the pad (a high carry at that radius is outside the IK
sphere with the long tool).

The BUILDER owns the cell verdict + AUTOQUIT; this controller never quits
the sim.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import time  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, dls_ik_pose,  # noqa: E402
                                 forward_kinematics_pose, _mat_mul,
                                 _mat3_to_axis_angle)
from _arm_configs import get_config  # noqa: E402

OZ = 0.3655
OUT = os.environ.get("ANYPICK_OUT", "_cell_feeder_result.txt")

CARRY_MIN = 0.32
CARRY_MAX = 0.42
WALL_CLEAR = 0.27
LIFT_RISE_OK = 0.07
HANG_PAD = 0.05

GRAB_EPS = 0.006
PRESS_DEPTHS = (0.002, -0.003, -0.008, -0.013, -0.018)
PRESS_IN_MAX = 0.030
HORIZ_EPS = 0.014
NODE_NEAR = 0.09

WALL_HALF = 0.18
WALL_NEAR = 0.135
MAX_TILT = 0.15
REACH_MAX = 0.75

BIN_HOME = [0.46, 0.0, 0.0]
PARK = (0.24, -0.34, 0.42)              # observation park (outside camera cone)
PAD = (0.65, -0.32)                     # handoff pad centre
PAD_R = 0.10                            # pad occupancy radius
PAD_WAY = (0.50, -0.25, 0.30)           # en-route waypoint (safe radius)
PAD_APPROACH_Z = 0.14                   # lowered leg toward the pad
PAD_PLACE_Z = 0.062                     # lip at set-down (pad 0.012 + slab 0.03 + pad)

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()
IK = bridge.cfg["ik"]

IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]

_cam_node = robot.getFromDef("BIN_CAM")
_cam_data = _cam_node.getField("customData") if _cam_node else None
_binnode = robot.getFromDef("MOVABLE_BIN")
_bin_tr = _binnode.getField("translation") if _binnode else None
_bin_rot = _binnode.getField("rotation") if _binnode else None


def emit(s):
    fh.write(s + "\n")
    print("[feeder] " + s, flush=True)


def _tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


_suck = None


def suck_on(node):
    global _suck
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    pos, _R_ = _tcp_pose()
    c = node.getPosition()
    d = [c[0] - pos[0], c[1] - pos[1], c[2] - pos[2]]
    try:
        node.setVelocity([0.0] * 6)
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
    n = int(secs * 1000 / dt)
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if _suck is not None:
            suck_apply()
        elif _binnode is not None:
            bp = _binnode.getPosition()
            if (abs(bp[0] - BIN_HOME[0]) > 0.004 or abs(bp[1] - BIN_HOME[1]) > 0.004
                    or abs(bp[2] - BIN_HOME[2]) > 0.006):
                _bin_tr.setSFVec3f(BIN_HOME)
                _bin_rot.setSFRotation([0.0, 0.0, 1.0, 0.0])
    return True


def _rot_axis(a, t):
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]]


TOP = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]


def pose(xyz, dur=1.4, settle=0.3):
    q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), list(xyz),
                                        TOP, (0.0, 0.0, OZ), IK, IK_LIMITS)
    if perr > 0.02:
        emit("  ! IK not converged for pose (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
             % (xyz[0], xyz[1], xyz[2], perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)


def approach_pose(x, y, z, lean, dur=1.2, settle=0.25):
    if lean is None:
        pose([x, y, z], dur=dur, settle=settle)
        return
    lx, ly, theta = lean
    rt = _mat_mul(_rot_axis([-ly, lx, 0.0], theta), TOP)
    q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), [x, y, z], rt,
                                        (0.0, 0.0, OZ), IK, IK_LIMITS)
    if perr > 0.02:
        emit("  ! IK not converged for approach (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
             % (x, y, z, perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)


def wall_lean(x, y):
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


def discover_parts():
    parts = {}
    i = 1
    misses = 0
    while misses < 6:
        name = "PART_%d" % i
        node = robot.getFromDef(name)
        if node is not None:
            parts[name] = node
            misses = 0
        else:
            misses += 1
        i += 1
    return parts


def in_bin(node):
    p = node.getPosition()
    return (abs(p[0] - BIN_HOME[0]) < 0.24 and abs(p[1] - BIN_HOME[1]) < 0.24
            and p[2] < 0.30)


def pad_occupied(parts):
    for node in parts.values():
        p = node.getPosition()
        if math.hypot(p[0] - PAD[0], p[1] - PAD[1]) < PAD_R and p[2] < 0.12:
            return True
    return False


def nearest_part(parts, lip):
    best, bd = None, 1e9
    for name, node in parts.items():
        c = node.getPosition()
        d = math.sqrt(sum((c[k] - lip[k]) ** 2 for k in range(3)))
        if d < bd:
            best, bd = name, d
    return (best, bd) if best else (None, 1e9)


def cam_read():
    if _cam_data is None:
        return 0, 0, []
    raw = _cam_data.getSFString() or ""
    tick = mat = 0
    anchors = []
    for tok in raw.split(";"):
        if tok.startswith("t="):
            tick = int(tok[2:] or 0)
        elif tok.startswith("m="):
            mat = int(tok[2:] or 0)
        elif tok:
            try:
                x, y, z = (float(v) for v in tok.split(","))
                anchors.append((x, y, z))
            except ValueError:
                pass
    return tick, mat, anchors


def observe():
    pose(list(PARK), dur=1.0, settle=0.2)
    t_before, _m, _a = cam_read()
    for _ in range(40):
        if not step_for(0.1):
            break
        t_now, m, anchors = cam_read()
        if t_now >= t_before + 3:
            return m, anchors
    _t, m, anchors = cam_read()
    return m, anchors


def press_to_contact(anchor, lean):
    gap = horiz = 1e9
    for i, depth in enumerate(PRESS_DEPTHS):
        approach_pose(anchor[0], anchor[1], anchor[2] + depth, lean,
                      dur=(1.2 if i == 0 else 0.35), settle=(0.25 if i == 0 else 0.1))
        lip, _R_ = _tcp_pose()
        gap = lip[2] - anchor[2]
        horiz = math.hypot(lip[0] - anchor[0], lip[1] - anchor[1])
        if -PRESS_IN_MAX <= gap <= GRAB_EPS and horiz <= HORIZ_EPS:
            emit("  touch ok: gap=%.1fmm lateral=%.1fmm" % (gap * 1000.0, horiz * 1000.0))
            return True
    emit("  NO TOUCH (gap=%.1fmm lateral=%.1fmm) -> retreat" % (gap * 1000.0, horiz * 1000.0))
    return False


def feed_one(parts, anchor):
    """Universal pick from the bin, then set the part down on the pad."""
    if math.hypot(anchor[0], anchor[1]) > REACH_MAX:
        emit("  anchor %.2f m out -> skip" % math.hypot(anchor[0], anchor[1]))
        return False
    lean = wall_lean(anchor[0], anchor[1])
    carry = min(CARRY_MAX, max(CARRY_MIN, WALL_CLEAR + HANG_PAD + (anchor[2] - 0.022)))
    pose([anchor[0], anchor[1], carry])
    if not press_to_contact(anchor, lean):
        approach_pose(anchor[0], anchor[1], carry, lean)
        return False
    lip, _R_ = _tcp_pose()
    name, nd = nearest_part(parts, lip)
    if name is None or nd > NODE_NEAR:
        emit("  touch but no node within %.0fmm -> retreat" % (NODE_NEAR * 1000))
        pose([anchor[0], anchor[1], carry])
        return False
    node = parts[name]
    z0 = node.getPosition()[2]
    suck_on(node)
    step_for(0.3)
    lp = _tcp_pose()[0]
    approach_pose(lp[0], lp[1], carry, lean)
    pose([lp[0], lp[1], carry], dur=0.8)
    if node.getPosition()[2] < z0 + LIFT_RISE_OK:
        emit("  lift failed -> release")
        suck_off()
        step_for(0.2)
        return False
    emit("  grabbed %s -> pad" % name)
    pose(list(PAD_WAY), dur=1.4)                 # safe-radius waypoint
    pose([PAD[0], PAD[1], PAD_APPROACH_Z], dur=1.2)
    pose([PAD[0], PAD[1], PAD_PLACE_Z], dur=0.8)
    suck_off()
    step_for(0.5)
    pose([PAD[0], PAD[1], PAD_APPROACH_Z], dur=0.8)
    pose(list(PAD_WAY), dur=1.0)
    return not in_bin(node)


# ── Run ──────────────────────────────────────────────────────────────
step_for(6.0)
parts = discover_parts()
emit("start: %d parts to feed" % len(parts))

fed = 0
guard = 0
empty_reads = 0
while guard < 80 and empty_reads < 3:
    guard += 1
    if pad_occupied(parts):
        step_for(1.0)                            # builder hasn't taken it yet
        continue
    m, anchors = observe()
    if m < 8 or not anchors:
        empty_reads += 1
        emit("camera: material=%dpx anchors=%d (empty read %d/3)"
             % (m, len(anchors), empty_reads))
        continue
    empty_reads = 0
    if feed_one(parts, anchors[0]):
        fed += 1
        emit("fed %d so far" % fed)

emit("DONE: fed %d/%d | wall=%.1fs sim=%.1fs"
     % (fed, len(parts), time.time() - t0, robot.getTime()))
pose(list(PARK), dur=1.0)
while robot.step(dt) != -1:                      # builder owns the AUTOQUIT
    bridge.tick(robot.getTime())
    if _suck is not None:
        suck_apply()
