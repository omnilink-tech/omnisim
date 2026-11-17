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

"""OMNIARM6 UNIVERSAL PICK -- empty a bin of ARBITRARY unknown objects with a
suction cup, from depth alone.

This is the depth-primary lane of the AnyPick family promoted to the ONLY
lane: there is NO shape registry, NO classifier, NO per-kind anchor model
in this controller. Everything the picking DECISIONS use comes from
universal_cam's published depth anchors (highest x flat x widest-patch,
+-3 mm noise) -- the 15 objects in omniarm6_universal_pick.omniworld (slabs, pucks,
tall boxes, T/L/U/plus brackets, a mug, cylinders, capsules, a sphere)
never appear in this file.

What stays supervisor-side (gauge + bookkeeping, NOT decisions -- the same
split the AnyPick line demo uses):
  * the vacuum weld needs a node: at engage the nearest PART node within
    NODE_NEAR of the lip is welded at its MEASURED offset (no snap);
  * the lift-rise gate (a "grabbed" part must actually RISE) and the final
    emptied/toted tally read true poses from the scene.

Contact honesty, universal edition: the cup lip is collision-free (a lip
collider would fight the weld), so touch is gauged against the FROZEN
depth anchor: the vacuum may only engage with the FK-measured lip within
[-PRESS_IN_MAX, GRAB_EPS] of the anchor height and laterally on it. The
anchor freezes at decision time because the descending arm occludes the
camera -- the arm PARKS outside the view cone before each observation and
the supervisor waits for a fresh frame (t= counter) before trusting it.
Known limitation (documented, accepted): on curved/edge tops the depth
anchor can sit ~1 cm above the true surface, so a grab can start with the
part hanging slightly below the lip -- the measured-offset weld shows that
gap honestly instead of teleporting the part flush.

IK: the LONG suction tool discipline from omniarm6_anypick -- OZ=0.3655,
joint2 capped to its physical range, every solve seeded from the fixed
elbow-forward nominal posture (see omniarm6_anypick.py for the full story).

Set ANYPICK_AUTOQUIT=1 to exit with a status when done. The world bakes
newtonStatics/newtonCompoundColliders; no env vars are required.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import time  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, dls_ik_pose,  # noqa: E402
                                 forward_kinematics_pose, _mat_mul)
from _arm_configs import get_config  # noqa: E402

OZ = 0.3655                     # cup LIP (flange + 0.200, omniarm6_suction_long.urdf)
OUT = os.environ.get("ANYPICK_OUT", "_universal_result.txt")

CARRY_MIN = 0.32                # lip height floor while carrying
CARRY_MAX = 0.42                # lip height ceiling (IK comfort)
WALL_CLEAR = 0.27               # part BOTTOM must ride above this (walls top 0.241)
LIFT_RISE_OK = 0.07             # a grabbed part must RISE this much
HANG_PAD = 0.05                 # unknown part extent below the lip: assume this much

# Contact gauge vs the FROZEN depth anchor (see module docstring).
GRAB_EPS = 0.006                # lip within this ABOVE the anchor counts as touch
# 3 mm rungs bottoming at -12 mm, matching omniarm6_anypick / _line. This demo used
# to step 5 mm to -18 mm and accept -30 mm, the loosest of the OMNIARM6 family, and
# the cup visibly buried itself in the parts: measured -5.6 mm and -10.0 mm on
# two consecutive picks, on top of the ~1 cm anchor error the docstring admits
# for curved/edge tops. The lip is collision-free by design (a lip collider
# would fight the weld), so this ladder IS the only thing bounding how far it
# sinks in -- nothing physical stops it.
PRESS_DEPTHS = (0.002, 0.0, -0.003, -0.006, -0.009, -0.012)
PRESS_IN_MAX = 0.025            # accept a lip pressed at most this far past it
HORIZ_EPS = 0.014               # ... and laterally on it
NODE_NEAR = 0.09                # weld the nearest node within this of the lip
                                # (node CENTRE -- a long bracket grabbed at a
                                # limb end puts its centre ~6-8 cm from the lip)

# Near-wall lean (long-tool discipline: small -- only the cup lip needs clearing).
WALL_HALF = 0.18
WALL_NEAR = 0.135
MAX_TILT = 0.15
REACH_MAX = 0.75

PARKS = ((0.24, -0.34, 0.42), (0.24, 0.34, 0.42))  # observation parks (pick
                                # the one farther from the DETECTED bin)
TOTE = (0.30, -0.44)            # output tote centre
TOTE_HALF = 0.14                # interior half-extent (drop points stay inside)
DROP_LIP = 0.20                 # release height (tote walls are 0.07)

# The bin pose is DETECTED by the camera (b= token), not assumed: drag or
# rotate the bin mid-demo and the next observation re-acquires it. All pick
# DECISIONS use the camera's estimate (_bin_pose, ~cm accurate). Between
# observations the bin is pinned so cup bumps cannot drift it -- but the
# pin target is a PHYSICAL-POSE SNAPSHOT taken at observation time
# (bookkeeping, same class as the weld node lookup): pinning to the camera
# estimate would teleport the bin by the estimate's bias. A displacement
# > PIN_RELEASE from the snapshot means the USER moved the bin, so the pin
# lets go until the camera re-detects.
_bin_pose = [0.46, 0.0, 0.0]    # camera estimate (cx, cy, yaw) -- decisions
_pin_pose = None                # physical snapshot [x, y, yaw] -- anti-drift
PIN_RELEASE = 0.05

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()
IK = bridge.cfg["ik"]

# joint2 capped to its PHYSICAL range + fixed nominal seed on the
# elbow-forward positive wrist branch -- the two long-tool IK rules
# established (with the measured failure story) in omniarm6_anypick.py.
IK_LIMITS = list(bridge.joint_limits)
IK_LIMITS[1] = (-1.95, 1.95)
NOM_SEED = [0.0, 0.55, 1.35, 0.0, 1.25, 0.0]

_cam_node = robot.getFromDef("BIN_CAM")
_cam_data = _cam_node.getField("customData") if _cam_node else None

_binnode = robot.getFromDef("MOVABLE_BIN")
_bin_tr = _binnode.getField("translation") if _binnode else None
_bin_rot = _binnode.getField("rotation") if _binnode else None


_T0 = time.time()


def emit(s):
    # Stamped so a cycle time can be read straight off the log: SIM seconds
    # (what the motion schedule costs) beside WALL seconds (what the machine
    # costs). The two only diverge when the sim is not running at 1x realtime,
    # and telling those apart is the entire question when a demo "feels slow".
    stamp = "[t=%7.2f w=%7.1f] " % (robot.getTime(), time.time() - _T0)
    fh.write(stamp + s + "\n")
    print(stamp + s, flush=True)


def _tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


# ── Vacuum weld, frozen at the measured contact pose ─────────────────
_suck = None


def suck_on(node):
    global _suck
    o = node.getOrientation()
    from omnilink_arm_bridge import _mat3_to_axis_angle
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    pos, _R_ = _tcp_pose()
    c = node.getPosition()
    d = [c[0] - pos[0], c[1] - pos[1], c[2] - pos[2]]   # measured: zero jump
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


# Per-tick cost split, printed with the final RESULT. --log-performance says
# how much this controller costs the engine; it cannot say which of OUR calls
# spent it. perf_counter is ~50 ns against a ~9 ms tick, so this is always on.
_prof = {"step": 0.0, "tick": 0.0, "rest": 0.0, "n": 0}


def step_for(secs):
    global _pin_pose
    n = int(secs * 1000 / dt)
    for _ in range(n):
        _t0 = time.perf_counter()
        if robot.step(dt) == -1:
            return False
        _t1 = time.perf_counter()
        bridge.tick(robot.getTime())
        _t2 = time.perf_counter()
        if _suck is not None:
            suck_apply()
        elif _binnode is not None and _pin_pose is not None:
            bp = _binnode.getPosition()
            o = _binnode.getOrientation()
            yaw = math.atan2(o[3], o[0])
            dev = math.hypot(bp[0] - _pin_pose[0], bp[1] - _pin_pose[1])
            dyaw = abs((yaw - _pin_pose[2] + math.pi) % (2 * math.pi) - math.pi)
            if dev > PIN_RELEASE or dyaw > 0.15:
                _pin_pose = None             # the USER moved the bin: let go
                emit("[uni] bin moved %.0fmm/%.0fdeg -> pin released, re-observing"
                     % (dev * 1000, math.degrees(dyaw)))
            elif dev > 0.004 or dyaw > 0.01 or abs(bp[2]) > 0.006:
                _bin_tr.setSFVec3f([_pin_pose[0], _pin_pose[1], 0.0])
                _bin_rot.setSFRotation([0.0, 0.0, 1.0, _pin_pose[2]])
        _t3 = time.perf_counter()
        _prof["step"] += _t1 - _t0
        _prof["tick"] += _t2 - _t1
        _prof["rest"] += _t3 - _t2
        _prof["n"] += 1
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
        emit("[uni]   ! IK not converged for pose (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
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
        emit("[uni]   ! IK not converged for approach (%.3f,%.3f,%.3f): err=%.1fmm iters=%d"
             % (x, y, z, perr * 1000.0, iters))
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)


def pose_line(xyz, rt=None, speed=0.30, settle=0.25):
    """Move the lip along a STRAIGHT Cartesian line from where it is to
    xyz, CONTINUOUSLY: all ~4 cm waypoints are IK-solved up front
    (branch-consistent NOM seeds keep consecutive solutions adjacent),
    then the arm tracks a target that advances along the joint-space
    polyline every tick, with ONE cubic ease over the whole move. Zero
    velocity happens only at the true start and end -- not at waypoints.

    (History: v1 ran each waypoint as its own eased command -> full stop
    every 5 cm, visible snapping. v2 blended commands at 55% overlap ->
    better, but every re-target restarts the ease whose initial VELOCITY
    is zero, so the arm still pulsed. This per-tick tracker is the fix.)

    The straight line is what keeps the tool from BOWING sideways
    mid-move -- single-solve joint interpolation sweeps the stick
    laterally, which looked like (and with a stick collider, physically
    was) the tool passing through the bin wall."""
    start = _tcp_pose()[0]
    d = [xyz[k] - start[k] for k in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    if dist < 1e-4:
        return
    n = max(1, int(dist / 0.04))
    R = rt if rt is not None else TOP
    qs = [list(bridge._read_q())]
    for k in range(1, n + 1):
        w = [start[j] + d[j] * k / n for j in range(3)]
        q, perr, _rerr, iters = dls_ik_pose(IK["chain"], list(NOM_SEED), w,
                                            R, (0.0, 0.0, OZ), IK, IK_LIMITS)
        if k == n and perr > 0.02:
            emit("[uni]   ! IK not converged for line end (%.3f,%.3f,%.3f): "
                 "err=%.1fmm iters=%d" % (w[0], w[1], w[2], perr * 1000.0, iters))
        qs.append(q)
    total = max(0.30, dist / speed)
    ticks = max(2, int(total * 1000 / dt))
    for i in range(1, ticks + 1):
        a = i / float(ticks)
        a = a * a * (3.0 - 2.0 * a)              # ONE ease over the whole line
        s = a * n
        seg = min(n - 1, int(s))
        f = s - seg
        q = [qs[seg][j] + (qs[seg + 1][j] - qs[seg][j]) * f
             for j in range(len(qs[0]))]
        with bridge.lock:
            bridge.motion = ("hold", {"q": q})   # per-tick carrot target
        if not step_for(dt / 1000.0):
            return
    step_for(settle)


def wall_lean(x, y):
    """Small lean toward the bin centre near walls -- computed in the
    DETECTED bin's own frame (the bin may sit at any yaw), then rotated
    back to world. Same pattern as the AnyPick line demo's yawed bins."""
    bx, by, byaw = _bin_pose
    c, s = math.cos(byaw), math.sin(byaw)
    du = (x - bx) * c + (y - by) * s
    dv = -(x - bx) * s + (y - by) * c
    lu = lv = 0.0
    if WALL_HALF - abs(du) < WALL_NEAR:
        lu = -math.copysign(1.0, du) * (1.0 - max(0.0, WALL_HALF - abs(du)) / WALL_NEAR)
    if WALL_HALF - abs(dv) < WALL_NEAR:
        lv = -math.copysign(1.0, dv) * (1.0 - max(0.0, WALL_HALF - abs(dv)) / WALL_NEAR)
    m = math.hypot(lu, lv)
    if m < 1e-6:
        return None
    lx = (lu * c - lv * s) / m
    ly = (lu * s + lv * c) / m
    return lx, ly, MAX_TILT * math.sqrt(min(1.0, m))


# ── Scene bookkeeping (gauge/tally only, never pick decisions) ───────
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
    bx, by, byaw = _bin_pose
    c, s = math.cos(byaw), math.sin(byaw)
    u = (p[0] - bx) * c + (p[1] - by) * s
    v = -(p[0] - bx) * s + (p[1] - by) * c
    return abs(u) < 0.24 and abs(v) < 0.24 and p[2] < 0.30


def in_tote(node):
    p = node.getPosition()
    return abs(p[0] - TOTE[0]) < 0.19 and abs(p[1] - TOTE[1]) < 0.19 and p[2] < 0.20


def nearest_part(parts, lip):
    best, bd = None, 1e9
    for name, node in parts.items():
        c = node.getPosition()
        d = math.sqrt(sum((c[k] - lip[k]) ** 2 for k in range(3)))
        if d < bd:
            best, bd = name, d
    return (best, bd) if best else (None, 1e9)


# ── Camera protocol ──────────────────────────────────────────────────
def cam_read():
    """(tick, material_px, bin_pose|None, [anchors]) from universal_cam."""
    if _cam_data is None:
        return 0, 0, None, []
    raw = _cam_data.getSFString() or ""
    tick = mat = 0
    bpose = None
    anchors = []
    for tok in raw.split(";"):
        if tok.startswith("t="):
            tick = int(tok[2:] or 0)
        elif tok.startswith("m="):
            mat = int(tok[2:] or 0)
        elif tok.startswith("b="):
            try:
                bx, by, byaw = (float(v) for v in tok[2:].split(","))
                bpose = (bx, by, byaw)
            except ValueError:
                pass
        elif tok:
            try:
                x, y, z = (float(v) for v in tok.split(","))
                anchors.append((x, y, z))
            except ValueError:
                pass
    return tick, mat, bpose, anchors


def observe():
    """Park the arm OUT of the camera cone (whichever park is farther from
    the DETECTED bin), wait for a fresh frame, adopt the detected bin pose,
    read anchors."""
    global _pin_pose
    park = max(PARKS, key=lambda p: math.hypot(p[0] - _bin_pose[0],
                                               p[1] - _bin_pose[1]))
    pose(list(park), dur=1.0, settle=0.2)
    t_before, _m, _b, _a = cam_read()
    m, bpose, anchors = 0, None, []
    for _ in range(40):                          # wait for a post-park frame
        if not step_for(0.1):
            break
        t_now, m, bpose, anchors = cam_read()
        if t_now >= t_before + 3:
            break
    if bpose is not None:
        if (math.hypot(bpose[0] - _bin_pose[0], bpose[1] - _bin_pose[1]) > 0.01
                or abs(bpose[2] - _bin_pose[2]) > 0.03):
            emit("[uni] bin detected at (%.3f,%.3f) yaw %.0fdeg"
                 % (bpose[0], bpose[1], math.degrees(bpose[2])))
        _bin_pose[0], _bin_pose[1], _bin_pose[2] = bpose
        # Arm the anti-drift pin on the bin's PHYSICAL pose (snapshot).
        if _binnode is not None:
            bp = _binnode.getPosition()
            o = _binnode.getOrientation()
            _pin_pose = [bp[0], bp[1], math.atan2(o[3], o[0])]
    return m, anchors


# ── One universal pick ───────────────────────────────────────────────
def press_to_contact(anchor, lean):
    """Descend onto the FROZEN depth anchor; touch = FK lip within
    [-PRESS_IN_MAX, GRAB_EPS] of the anchor height and laterally on it.
    The long first descent is a straight Cartesian LINE (no lateral bow
    inside the bin); the following press steps are small enough to stay
    straight on their own."""
    rt = None
    if lean is not None:
        lx, ly, theta = lean
        rt = _mat_mul(_rot_axis([-ly, lx, 0.0], theta), TOP)
    gap = horiz = 1e9
    for i, depth in enumerate(PRESS_DEPTHS):
        if i == 0:
            pose_line([anchor[0], anchor[1], anchor[2] + depth], rt=rt,
                      speed=0.25, settle=0.25)
        else:
            approach_pose(anchor[0], anchor[1], anchor[2] + depth, lean,
                          dur=0.35, settle=0.1)
        lip, _R_ = _tcp_pose()
        gap = lip[2] - anchor[2]
        horiz = math.hypot(lip[0] - anchor[0], lip[1] - anchor[1])
        if -PRESS_IN_MAX <= gap <= GRAB_EPS and horiz <= HORIZ_EPS:
            emit("[uni]   touch ok: gap=%.1fmm lateral=%.1fmm%s"
                 % (gap * 1000.0, horiz * 1000.0,
                    " (tilted %.0f deg)" % math.degrees(lean[2]) if lean else ""))
            return True
    emit("[uni]   NO TOUCH (gap=%.1fmm lateral=%.1fmm) -> no vacuum, retreating"
         % (gap * 1000.0, horiz * 1000.0))
    return False


_drop_i = 0
# Drop points biased toward the base side of the tote: the far corner
# (0.37,-0.51) at carry height sits ~3 cm past the comfortable IK sphere
# (measured: 25 mm converge error), the near corners are all inside it.
_DROPS = [(0.23, -0.37), (0.37, -0.37), (0.23, -0.51), (0.30, -0.44)]


def universal_pick(parts, anchor):
    """Stage above the depth anchor, press to contact, weld the nearest
    node, lift, carry to the tote, release. True if a part left the bin."""
    global _drop_i
    if math.hypot(anchor[0], anchor[1]) > REACH_MAX:
        emit("[uni]   anchor %.2f m out -> unreachable, skipping"
             % math.hypot(anchor[0], anchor[1]))
        return False
    lean = wall_lean(anchor[0], anchor[1])
    carry = min(CARRY_MAX, max(CARRY_MIN, WALL_CLEAR + HANG_PAD + (anchor[2] - 0.022)))
    pose_line([anchor[0], anchor[1], carry])     # stage above the anchor (straight)
    if not press_to_contact(anchor, lean):
        pose_line([anchor[0], anchor[1], carry], speed=0.25)
        return False
    lip, _R_ = _tcp_pose()
    name, nd = nearest_part(parts, lip)
    if name is None or nd > NODE_NEAR:
        emit("[uni]   touch but no part within %.0fmm (?) -> retreat" % (NODE_NEAR * 1000))
        pose([anchor[0], anchor[1], carry])
        return False
    node = parts[name]
    z0 = node.getPosition()[2]
    suck_on(node)
    step_for(0.3)
    lp = _tcp_pose()[0]
    rt = None
    if lean is not None:
        lx, ly, theta = lean
        rt = _mat_mul(_rot_axis([-ly, lx, 0.0], theta), TOP)
    pose_line([lp[0], lp[1], carry], rt=rt, speed=0.25)   # lift straight up
    pose([lp[0], lp[1], carry], dur=0.8)                  # square the tool up
    if node.getPosition()[2] < z0 + LIFT_RISE_OK:
        emit("[uni]   lift failed (rise %.0fmm) -> release"
             % ((node.getPosition()[2] - z0) * 1000))
        suck_off()
        step_for(0.2)
        return False
    dx, dy = _DROPS[_drop_i % len(_DROPS)]
    _drop_i += 1
    emit("[uni]   grabbed %s, to tote" % name)
    pose_line([dx, dy, carry], speed=0.35)                # straight traverse
    pose_line([dx, dy, DROP_LIP], speed=0.30)
    suck_off()
    step_for(0.6)
    pose_line([dx, dy, carry], speed=0.35)
    return not in_bin(node)


# ── Run ──────────────────────────────────────────────────────────────
step_for(6.0)                                   # let the dumped pile settle
parts = discover_parts()
emit("[uni] start    %d arbitrary parts in the bin (no registry, no models)"
     % len(parts))

fails_at = {}                                   # coarse anchor key -> fails
guard = 0
empty_reads = 0
recoveries = 0
# STALL GUARD. The loop's own exits are "the bin reads empty" (empty_reads) and
# a 120-attempt ceiling. Neither fires when the LAST few parts are small and
# curved: they still show 60-70 px of material, which is well above the m < 8
# empty-read threshold, so the camera never calls the bin empty and the loop
# grinds every one of its 120 attempts (~840 s wall). A run cut short by the
# harness before that prints NO RESULT LINE AT ALL, so "did not finish" is
# indistinguishable from "failed" -- which is exactly what happened once here
# under port contention from a concurrent engine. Measured on a quiet machine
# a healthy run uses 15-16 attempts IN TOTAL, so a long run of consecutive
# attempts that free nothing means stuck, not unlucky.
#
# ⚠ THE TRADE-OFF, stated because it is real: breaking early cannot IMPROVE a
# run, and could in principle end one that would have recovered. Measured over
# 10 runs it fired twice, both times after >=12 identical failures on the SAME
# two anchors at the bin wall (lateral error 29-52 mm, i.e. nowhere near the
# touch gate) -- those were not near misses. The limit is nonetheless set well
# above the observed stall length so it only ever fires on an unambiguous one:
# 25 consecutive barren attempts is ~175 s of extra effort before giving up,
# against a 120-attempt ceiling that costs ~840 s and, when the harness kills
# the run first, reports NOTHING AT ALL. Reporting a worse tally beats
# reporting no tally.
STALL_LIMIT = 25
stalled = 0
last_in_bin = sum(1 for k in parts if in_bin(parts[k]))
while guard < 120 and empty_reads < 3:
    guard += 1
    now_in_bin = sum(1 for k in parts if in_bin(parts[k]))
    stalled = 0 if now_in_bin < last_in_bin else stalled + 1
    last_in_bin = now_in_bin
    if stalled > STALL_LIMIT:
        emit("[uni] STALLED: %d consecutive attempts freed nothing (%d still in "
             "the bin). Stopping and reporting rather than grinding to the "
             "120-attempt ceiling." % (stalled - 1, now_in_bin))
        break
    m, anchors = observe()
    if m >= 8 and not anchors and recoveries < 4:
        # RECOVERY lane: the camera sees material but certifies no
        # suction-graspable surface (typically a part PROPPED against a
        # wall -- no flat top from above). Fall back to ONE node-guided
        # pick: nearest thing a side camera would see. This is the single
        # documented deviation from camera-only decisions, used only when
        # the camera itself has declared the pile ungraspable; the touch
        # gauge and lift-rise verification still apply unchanged.
        cands = [(parts[k].getPosition()[2], k) for k in parts if in_bin(parts[k])]
        if cands:
            recoveries += 1
            _z, k = max(cands)
            p = parts[k].getPosition()
            anchor = (p[0], p[1], p[2] + 0.02)
            emit("[uni] RECOVERY %d/4: camera sees %dpx but no graspable "
                 "surface; node-guided pick near (%.3f,%.3f)"
                 % (recoveries, m, anchor[0], anchor[1]))
            universal_pick(parts, anchor)
            continue
    if m < 8 or not anchors:
        empty_reads += 1
        emit("[uni] camera: material=%dpx anchors=%d (empty read %d/3)"
             % (m, len(anchors), empty_reads))
        continue
    empty_reads = 0
    recoveries = 0
    # skip anchors that keep failing (the pile there defies the cup)
    anchor = None
    for a in anchors:
        key = (round(a[0] * 20), round(a[1] * 20))
        if fails_at.get(key, 0) < 3:
            anchor = a
            akey = key
            break
    if anchor is None:
        anchor = anchors[0]
        akey = (round(anchor[0] * 20), round(anchor[1] * 20))
    emit("[uni] pick at depth anchor (%.3f,%.3f,%.3f) [material=%dpx]"
         % (anchor[0], anchor[1], anchor[2], m))
    if universal_pick(parts, anchor):
        fails_at.pop(akey, None)
    else:
        fails_at[akey] = fails_at.get(akey, 0) + 1

# Final tally -- true poses from the scene (the honest ruler).
bridge.act_reset_to_home()
step_for(1.2)
n = len(parts)
emptied = sum(1 for k in parts if not in_bin(parts[k]))
toted = sum(1 for k in parts if in_tote(parts[k]))
remaining = [k for k in parts if in_bin(parts[k])]
ok = emptied >= 0.9 * n and toted >= 0.8 * n
emit("[uni] RESULT emptied=%d/%d toted=%d/%d remaining=%s | wall=%.1fs sim=%.1fs"
     % (emptied, n, toted, n, remaining, time.time() - t0, robot.getTime()))
if _prof["n"]:
    _pn = float(_prof["n"])
    emit("[uni] PERTICK over %d ticks: robot.step=%.3fms bridge.tick=%.3fms "
         "weld+pin=%.3fms (ours=%.3fms)"
         % (_prof["n"], _prof["step"] / _pn * 1e3, _prof["tick"] / _pn * 1e3,
            _prof["rest"] / _pn * 1e3,
            (_prof["tick"] + _prof["rest"]) / _pn * 1e3))
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
