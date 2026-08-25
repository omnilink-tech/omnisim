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

"""ASSEMBLY CELL -- BUILDER arm (base at (1.30, 0), yawed pi).

Whenever the handoff pad is occupied, takes the part and stacks it on the
FIXTURE: a 4-slab tower. Owns the cell verdict + AUTOQUIT.

Frames: the ArmBridge solves IK in the arm's BASE frame, and this arm is
translated + yawed in the world, so every world target goes through
W2B(x,y,z) = (1.30 - x, -y, z) and every FK lip comes back through the
inverse (same formula). The top-down tool frame is unchanged by the base
yaw (the cup is symmetric about the tool axis).

Decisions here read part poses from the scene: legitimate for an ASSEMBLY
cell (a fixtured product with known slab height 0.03 -- the universal
no-model claim belongs to the FEEDER side). Touch stays gauged (press to
the computed slab top, vacuum only within the gauge window), the weld is
measured-offset, and the final tower tally is the honest ruler.

Interlock with the feeder: on seeing the pad occupied, wait 2 s and
re-verify (the feeder is still retreating), then go. The feeder operates
x < 0.5, the builder x > 0.55 except the shared pad moment.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import time  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, dls_ik_pose,  # noqa: E402
                                 forward_kinematics_pose, _mat3_to_axis_angle)
from _arm_configs import get_config  # noqa: E402

OZ = 0.3655
OUT = os.environ.get("ANYPICK_OUT", "_cell_builder_result.txt")

BASE = (1.24, 0.0)                      # world pose of this arm's base (yaw pi)
                                        # (1.24 not 1.30: at 1.30 the pad sat at
                                        # radius 0.72 -- the reach edge -- and
                                        # presses there overshot ~2 cm)

GRAB_EPS = 0.006
PRESS_DEPTHS = (0.002, -0.003, -0.008, -0.013)
PRESS_IN_MAX = 0.030
HORIZ_EPS = 0.014
NODE_NEAR = 0.09
LIFT_RISE_OK = 0.06

PAD = (0.65, -0.32)                     # world
PAD_R = 0.10
PAD_WAY_W = (0.80, -0.25, 0.30)         # world waypoint toward the pad
PAD_APPROACH_Z = 0.14

SLAB_H = 0.03                           # the product spec
FIX = (0.84, 0.30)                      # fixture centre (world)
FIX_TOP = 0.018
FIX_WAY_W = (0.90, 0.18, 0.30)          # world waypoint toward the fixture
PARK_W = (1.06, -0.30, 0.40)            # idle park (world)

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()
IK = bridge.cfg["ik"]

IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]


def emit(s):
    fh.write(s + "\n")
    print("[builder] " + s, flush=True)


def W2B(x, y, z):
    return (BASE[0] - x, -y, z)


def B2W(x, y, z):
    return (BASE[0] - x, -y, z)             # the yaw-pi transform is its own inverse


def _tcp_pose_world():
    p, R = forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))
    return B2W(p[0], p[1], p[2])


_suck = None


def suck_on(node):
    global _suck
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    lipw = _tcp_pose_world()
    c = node.getPosition()
    d = [c[0] - lipw[0], c[1] - lipw[1], c[2] - lipw[2]]
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass
    _suck = (node, node.getField("translation"), node.getField("rotation"), d, rot0)


def suck_apply():
    if _suck is None:
        return
    node, tr, rot, d, rot0 = _suck
    lipw = _tcp_pose_world()
    tr.setSFVec3f([lipw[0] + d[0], lipw[1] + d[1], lipw[2] + d[2]])
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
    return True


TOP = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]


def pose_w(xyz, dur=1.4, settle=0.3):
    """Move the lip to a WORLD position, top-down."""
    b = W2B(*xyz)
    q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), list(b),
                                        TOP, (0.0, 0.0, OZ), IK, IK_LIMITS)
    if perr > 0.02:
        emit("  ! IK not converged for pose (%.3f,%.3f,%.3f)w: err=%.1fmm iters=%d"
             % (xyz[0], xyz[1], xyz[2], perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)


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


def part_on_pad(parts):
    for name, node in parts.items():
        p = node.getPosition()
        if math.hypot(p[0] - PAD[0], p[1] - PAD[1]) < PAD_R and p[2] < 0.12:
            v = node.getVelocity()
            if max(abs(x) for x in v[:3]) < 0.02:      # settled, not mid-place
                return name, node
    return None, None


def press_to_contact_w(anchor):
    """Press to a WORLD anchor (slab top), gauge lip-vs-anchor honestly."""
    gap = horiz = 1e9
    for i, depth in enumerate(PRESS_DEPTHS):
        pose_w([anchor[0], anchor[1], anchor[2] + depth],
               dur=(1.2 if i == 0 else 0.35), settle=(0.25 if i == 0 else 0.1))
        lip = _tcp_pose_world()
        gap = lip[2] - anchor[2]
        horiz = math.hypot(lip[0] - anchor[0], lip[1] - anchor[1])
        if -PRESS_IN_MAX <= gap <= GRAB_EPS and horiz <= HORIZ_EPS:
            emit("  touch ok: gap=%.1fmm lateral=%.1fmm" % (gap * 1000.0, horiz * 1000.0))
            return True
    emit("  NO TOUCH (gap=%.1fmm lateral=%.1fmm) -> retreat" % (gap * 1000.0, horiz * 1000.0))
    return False


def build_one(name, node, k):
    """Take the pad part, stack it as tower level k (0-based)."""
    p = node.getPosition()
    anchor = (p[0], p[1], p[2] + SLAB_H / 2.0)   # slab top (product spec)
    pose_w(list(PAD_WAY_W), dur=1.2)
    pose_w([anchor[0], anchor[1], PAD_APPROACH_Z], dur=1.0)
    if not press_to_contact_w(anchor):
        pose_w([anchor[0], anchor[1], PAD_APPROACH_Z], dur=0.8)
        pose_w(list(PAD_WAY_W), dur=0.8)
        return False
    z0 = node.getPosition()[2]
    suck_on(node)
    step_for(0.3)
    lip = _tcp_pose_world()
    # lift INWARD (toward this arm's base), not straight up: the pad sits
    # near the reach edge, and a vertical lift there is outside the IK sphere.
    pose_w([lip[0] + 0.10, lip[1] + 0.05, 0.26], dur=1.0)
    if node.getPosition()[2] < z0 + LIFT_RISE_OK:
        emit("  lift failed -> release")
        suck_off()
        step_for(0.2)
        return False
    emit("  grabbed %s -> tower level %d" % (name, k + 1))
    place_lip = FIX_TOP + (k + 1) * SLAB_H + 0.004
    pose_w(list(FIX_WAY_W), dur=1.2)
    pose_w([FIX[0], FIX[1], 0.30], dur=1.0)
    pose_w([FIX[0], FIX[1], place_lip + 0.06], dur=0.9)
    pose_w([FIX[0], FIX[1], place_lip], dur=0.8)
    suck_off()
    step_for(0.5)
    pose_w([FIX[0], FIX[1], 0.30], dur=0.8)
    pose_w(list(PARK_W), dur=1.0)
    return True


def on_fixture(node):
    p = node.getPosition()
    return abs(p[0] - FIX[0]) < 0.05 and abs(p[1] - FIX[1]) < 0.05 and p[2] > FIX_TOP


# ── Run ──────────────────────────────────────────────────────────────
step_for(6.0)
parts = discover_parts()
n = len(parts)
emit("start: building a %d-slab tower on the fixture" % n)
pose_w(list(PARK_W), dur=1.2)

placed = 0
idle = 0.0
while placed < n and idle < 90.0:
    name, node = part_on_pad(parts)
    if node is None:
        step_for(0.6)
        idle += 0.6
        continue
    step_for(2.0)                                # feeder retreat interlock
    name2, node2 = part_on_pad(parts)
    if node2 is None:
        continue
    idle = 0.0
    if build_one(name2, parts[name2], placed):
        placed += 1

# Verdict: the tower, measured. Every slab on the fixture within 5 cm xy,
# and the stack really stacked (top slab at ~FIX_TOP + n*SLAB_H).
step_for(1.5)
onfix = [k for k in parts if on_fixture(parts[k])]
ztop = max((parts[k].getPosition()[2] for k in onfix), default=0.0)
want_top = FIX_TOP + (n - 0.5) * SLAB_H          # top slab CENTRE for a full tower
ok = len(onfix) == n and ztop > want_top - 0.02
emit("RESULT placed=%d/%d on_fixture=%d ztop=%.3f (full-tower top ~%.3f) | wall=%.1fs sim=%.1fs"
     % (placed, n, len(onfix), ztop, want_top, time.time() - t0, robot.getTime()))
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
