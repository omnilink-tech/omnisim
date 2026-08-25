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

"""Empty-and-SORT bin picking for the OMNIARM6 with a SUCTION (vacuum) end-effector.

A single supervisor controller (no HTTP, no second controller) so it runs
headless and deterministically. It clears a DENSE box of 36 cubes (4x3x3, three
layers, 18 red / 18 blue) and SORTS them by colour into two walled trays:

  1. Enumerate every DEF PART_* still inside the bin footprint.
  2. Pick the HIGHEST remaining one (the exposed top of the pile).
  3. Read its colour from its recognitionColors / PBRAppearance baseColor.
  4. Bring the cup straight down onto the cube's TOP face, engage the vacuum
     (suction_pick), lift straight up, carry to the next slot in the MATCHING
     colour tray, release.
  5. Retry any cube that didn't lift; recover any that ended off its tray.
  6. Repeat until the box is empty.

WHY SUCTION: a finger gripper FAILS on a dense bin -- 50 mm cubes tessellate
into a solid block the 85 mm 2F-85 jaws can't get two-sided clearance on, and
shaking to free them flings cubes out of reach (best ~6-21/36; see the cube /
pipe / L-shape / bar journey in docs). A vacuum cup grips the TOP face from
directly above, so packing, rolling, and jaw geometry stop mattering entirely:
the pile empties top-down with NO shaking. The cup is a real tool link on the
wrist (omniarm6_suction.urdf -- a flared rubber cup, no fingers); the vacuum is
modelled as a kinematic weld that engages ON PHYSICAL TOUCH: the descent
touch-servos until the lip measurably rests on the cube's top face, then the
hold keeps the MEASURED lip->cube offset from that moment (suck_on / suck_apply
/ suck_off) -- the cube is carried where it was touched, never snapped to an
idealized pose, so it reads as suction, not a force field. The cube hangs
straight down from the tool (not rotation-coupled) so a mid-carry wrist
reconfigure can't swing it off.

NOTE: calls warmup_reload() at startup to dodge the COLD-FIRST-LOAD bug. See
docs/developer/real-grasp-and-the-cold-first-load-trap.md.

The bin + tray walls are real Newton static colliders (launch with
OMNISIM_NEWTON_STATICS=1; see the world header). Set BINPICK_AUTOQUIT=1 to exit
with a status when done.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, dls_ik_pose, _mat_mul,  # noqa: E402
                                 warmup_reload, forward_kinematics_pose,
                                 _mat_vec, _mat3_to_axis_angle)
from _arm_configs import get_config  # noqa: E402

OZ = 0.25                       # tool point = the cup LIP (flange + ~0.085)
OUT = os.environ.get("BINPICK_OUT", "_binpick_result.txt")

# Bin footprint (interior) — a part with its xy inside this and z below carry
# height is still "in the box".
BIN_X = (0.27, 0.65)
BIN_Y = (-0.19, 0.19)

# Carry / approach geometry.
CARRY_Z = 0.34                  # suction lift/carry height -- clears the bin walls WITH MARGIN:
                                # worst-case cube bottom = CARRY_Z - dz_max(0.035) - cube(0.05)
                                # = 0.255 m vs wall top 0.241 m (14 mm clear). At 0.30 a cube
                                # hanging at the dz clamp clipped the wall top on the way out.
APPROACH_DZ = 0.18              # how far above a part the tool stages first
LIFT_OK_Z = 0.13                # a grasped part must clear this (off the floor)
DROP_Z = 0.10                   # release height — above the 0.05 tray walls so
                                # the cube drops in onto the growing pile
GRASP_WIDTH = 0.058             # pre-open width: barely over a 0.05 cube so the
                                # narrow fingers pinch a proud cube's exposed top
                                # without diving into / ploughing the dense pile

# Two sorted destinations, both on the south side (y<0). RED keeps its original
# pad (zone centre x=0.30); BLUE sits on RED's NEAR side (zone centre x=-0.02),
# tucked between red and the robot base so the blue tray is the closest one to
# the arm. Each gets a 4x4 grid of slots so sorted cubes don't pile up.
RED_SLOTS = [(x, y)
             for y in (-0.39, -0.43, -0.47, -0.51)
             for x in (0.24, 0.28, 0.32, 0.36)]
BLUE_SLOTS = [(x, y)
              for y in (-0.39, -0.43, -0.47, -0.51)
              for x in (-0.08, -0.04, 0.00, 0.04)]

MAX_ATTEMPTS = 2                # grasp retries WITHIN one pick (lift slipped)
MAX_PART_TRIES = 4             # whole-pick retries per part (knocked off a stack)

robot = Supervisor()
# Dodge the COLD-FIRST-LOAD bug: on a fresh build the arm under-tracks its grasp
# poses (lands ~1 cm short), which costs picks; a one-time reload at startup makes
# the first load track accurately. See docs/developer/real-grasp-and-the-cold-first-
# load-trap.md. Opt out with OMNISIM_NO_WARMUP=1 (the cold baseline does).
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
# No finger gripper: the end-effector is a real SUCTION cup (a tool link in
# omniarm6_suction.urdf, no fingers). gripper_id=None -> the bridge does pure IK and
# every act_grasp/open/width call is a safe no-op. The "grip" is the vacuum weld below.
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()

# Top-down camera bridge: the BIN_CAM robot's controller writes the cubes it
# recognises (occlusion -> top of the pile) into its customData as
# "<nodeId>:<colour>,...". We read that field to drive which cube to pick.
_cam_node = robot.getFromDef("BIN_CAM")
_cam_data = _cam_node.getField("customData") if _cam_node else None
ID_TO_NAME = {}                 # nodeId -> "PART_n", filled in discover_parts


def camera_detection():
    """{name: colour} for cubes the camera currently sees. Empty if the camera
    is occluded (e.g. the arm is over the bin) or unavailable."""
    out = {}
    if _cam_data is None:
        return out
    raw = _cam_data.getSFString() or ""
    for tok in raw.split(","):
        if ":" not in tok:
            continue
        sid, _, col = tok.partition(":")
        try:
            name = ID_TO_NAME.get(int(sid))
        except ValueError:
            name = None
        if name:
            out[name] = col
    return out


# ── Trained manipulation policy: PRE-GRASP PUSH + grasp residual ─────
# A tiny RL policy (trained by projects/policies: omniarm6_grasp_train) that expands the
# rigid top-down grasp's action space into MULTI-STEP manipulation. It outputs
# a 6-D action conditioned on the local cube configuration + the target's wall
# gaps:
#     [push_x, push_y,  grasp_dx, grasp_dy, grasp_dz, grasp_dyaw]
# The first two dims are a GATED pre-grasp push (the closed gripper sweeps the
# packed target toward open space so the wide jaws can get around it); the last
# four are the grasp-pose residual. Falls back to the plain grasp if no ONNX is
# present, or if BINPICK_USE_RL=0.
N_NEAR = 6
GP_OBS = 3 + N_NEAR * 3 + 4      # up-vec(3) + 6 neighbours(18) + 4 wall-gaps
GP_GRASP_SCALE = np.array([0.025, 0.025, 0.015, 0.79], dtype=np.float32)
IK = bridge.cfg["ik"]

# ── Movable bin + grasp-weld (for AGITATION) ────────────────────────
# The bin is a free dynamic body. When picking stalls, the robot grabs a wall and
# shakes/tilts the bin to tumble wall-locked cubes into reachable spots. The hold
# uses a grasp-stabilization weld (track the bin pose to the gripper TCP each tick)
# -- reliable on a heavy bin full of loose parts, and the shake is the point so the
# weld reads fine. See real-grasp journey doc.
_binnode = robot.getFromDef("MOVABLE_BIN")
_bin_tr = _binnode.getField("translation") if _binnode else None
_bin_rot = _binnode.getField("rotation") if _binnode else None
BIN_HOME = [0.46, 0.0, 0.0]            # the bin's resting translation
# The suction cup is a rigid tool link on the wrist (omniarm6_suction.urdf), so it just
# rides the flange -- no node to pin. The pick is the vacuum weld (suck_on/apply/off).
_held = False
_hold_relpos = [0.0, 0.0, 0.0]
_hold_R0T = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
# The bin is a DYNAMIC body (so we can grab + shake it). It used to be PINNED to
# home during picking (teleport back when drifted >4 mm) -- but the snap-back IS
# a visible 3-5 mm instantaneous jump, and it can re-create penetration with the
# pile that the solver resolves by shoving the bin again (a snap oscillation).
# That WAS the "bin shakes on every pick" bug. Now the 30 kg bin pins itself:
# static friction (~440 N) dwarfs every bounded pick force (12 N tether, relaxed
# touch press), and every consumer reads the bin's LIVE position (in_bin
# footprint, grasp targets), so millimetre drift is harmless. Re-enable only for
# debugging with BINPICK_PIN_BIN=1; the deliberate re-home after an agitation
# shake is separate and stays.
_pin_bin = os.environ.get("BINPICK_PIN_BIN", "0") == "1"


def _matT(m):
    return [[m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]]]


def _tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


def grab_hold():
    """Weld the bin to the gripper at its current relative pose."""
    global _held, _hold_relpos, _hold_R0T
    pos, R = _tcp_pose()
    bp = _binnode.getPosition()
    d = [bp[0] - pos[0], bp[1] - pos[1], bp[2] - pos[2]]
    _hold_R0T = _matT(R)
    _hold_relpos = _mat_vec(_hold_R0T, d)
    _held = True


def release_hold():
    global _held
    _held = False


def _apply_hold():
    pos, R = _tcp_pose()
    off = _mat_vec(R, _hold_relpos)
    _bin_tr.setSFVec3f([pos[0] + off[0], pos[1] + off[1], pos[2] + off[2]])
    _bin_rot.setSFRotation(_mat3_to_axis_angle(_mat_mul(R, _hold_R0T)))


# ── SUCTION (vacuum) end-effector ───────────────────────────────────
# A suction cup doesn't need to get fingers AROUND a part -- it grips the part's TOP
# face from directly above and lifts. So it takes the top of the pile with no side
# clearance: a dense bin empties top-down with no shaking, no rolling, no jaw-geometry
# limit. We model the vacuum as a kinematic weld of the picked part to the tool,
# engaged on measured touch and released over the tray.

# The tool point (OZ) is the cup LIP. The vacuum engages ON TOUCH (the descent
# servos until the lip measurably rests on the part's top face) and the hold
# keeps the MEASURED lip->part offset from that moment -- the part is carried
# exactly where it was touched, it is never snapped to an idealized pose.
# Clamps keep a bad engage from carrying the part on a visible air gap (dz)
# or far off the tool axis (dxy, which would mis-place it on the tray).
CUP_HOLD_DZ_MIN = 0.020   # part pressed 5 mm into the lip at most (rubber squish)
CUP_HOLD_DZ_MAX = 0.035   # 10 mm air gap at most AT ENGAGE (flush = 0.025)
CUP_HOLD_DXY_MAX = 0.020  # lateral engage offset carried as-is up to 2 cm
# After engage the vacuum DRAWS the part flush along the cup axis (what real
# suction does the instant the seal forms): dz converges to flush at this rate,
# so a part engaged with a residual air gap doesn't LEVITATE below the lip for
# the whole carry -- it visibly sucks up onto the cup within ~0.3 s. Smooth and
# axis-only: no lateral re-centring (that would look like the part sliding).
CUP_HOLD_DZ_FLUSH = 0.025
CUP_SUCK_RATE = 0.0006    # m per tick (~37 mm/s at the 16 ms basic step)

# The HOLD is a kinematic weld at the MEASURED engage offset. (A capped soft
# force-tether carry was tried and REVERTED, 2026-07-12: against this world's
# mu=5 contacts it either broke away on wedged picks -- the dropped cubes
# bouncing off the pile read as "cubes dancing everywhere" -- or, with the cap
# raised, still let held cubes rattle between neighbours. The weld carries
# rock-steady; the bin-shake the weld's prying once caused is gone because the
# BIN is now STATIC in the world, which is the actual fix for the shake.)
_suck = None        # (node, tr_field, rot_field, (dx,dy,dz) engage offset, rot0)


def suck_on(node):
    """Engage the vacuum AT TOUCH: record the measured lip->part offset and weld the
    part at that relative pose (no snap to an idealized pose). The offset is kept
    straight DOWN in the world frame (not rotated by the tool orientation): the arm
    sometimes reconfigures its wrist mid-carry, and a rotation-coupled offset then
    SWINGS the held part far off and drops it in the wrong place (the mis-sort bug)."""
    global _suck
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    pos, _R = _tcp_pose()
    p = node.getPosition()
    off = (max(-CUP_HOLD_DXY_MAX, min(CUP_HOLD_DXY_MAX, p[0] - pos[0])),
           max(-CUP_HOLD_DXY_MAX, min(CUP_HOLD_DXY_MAX, p[1] - pos[1])),
           max(CUP_HOLD_DZ_MIN, min(CUP_HOLD_DZ_MAX, pos[2] - p[2])))
    try:
        node.setVelocity([0.0] * 6)                   # kill any pile velocity at grab
    except Exception:
        pass
    _suck = (node, node.getField("translation"), node.getField("rotation"), off, rot0, None)


# KNOCK-OFF rule (brutal realism): a real vacuum seal breaks when the held part
# strikes an obstacle. The weld teleports the part each tick, so a strike shows
# up as the SOLVER pushing the part out of penetration -- i.e. the part's actual
# position diverging from where the weld put it last tick. Divergence past this
# threshold = the part is being forced against something hard: release the weld
# and let physics take it (it falls; the retry/recovery loop picks it back up).
#
# ARMED ONLY IN TRANSIT (above the bin walls). Inside the bin the part is still
# touching the pile it is being peeled out of, and the solver's normal resistance
# to that (plus the vacuum draw-up pulling against it) is NOT a strike -- checking
# there fired false knock-offs mid-peel. Above the wall top the only things left
# to hit are the bin wall and the trays, which is exactly the case to model.
_KNOCKOFF_EPS = 0.008
_KNOCKOFF_ARM_Z = 0.25    # wall top 0.241 -- above this the part is in free transit


def suck_apply():
    """Hold the sucked part at its measured engage offset (called every physics tick).
    Releases the weld -- the part FALLS -- if the part is striking an obstacle."""
    global _suck
    if _suck is None:
        return
    _node, tr, rot, off, rot0, prev_cmd = _suck
    if prev_cmd is not None and prev_cmd[2] > _KNOCKOFF_ARM_Z:
        a = _node.getPosition()
        err = math.dist(a, prev_cmd)
        if err > _KNOCKOFF_EPS:
            emit("[bin]   part KNOCKED OFF the cup (struck an obstacle in transit, %.0f mm)"
                 " -- it falls" % (err * 1000.0))
            _suck = None
            return
    # Vacuum draw-up: converge dz to flush so the part never levitates below
    # the lip (rate-limited -- a smooth suck-up, not a snap).
    if off[2] > CUP_HOLD_DZ_FLUSH:
        off = (off[0], off[1], max(CUP_HOLD_DZ_FLUSH, off[2] - CUP_SUCK_RATE))
    elif off[2] < CUP_HOLD_DZ_FLUSH:
        off = (off[0], off[1], min(CUP_HOLD_DZ_FLUSH, off[2] + CUP_SUCK_RATE))
    pos, _R = _tcp_pose()
    cmd = [pos[0] + off[0], pos[1] + off[1], pos[2] - off[2]]
    _suck = (_node, tr, rot, off, rot0, cmd)
    tr.setSFVec3f(cmd)
    rot.setSFRotation(rot0)                             # keep the part's grasp orientation
    # PIN the velocity to zero every tick: the part is a dynamic body teleported to
    # follow the cup; without this it accumulates velocity and fights the teleport
    # (bounce/jitter on a warm, stiff solver).
    try:
        _node.setVelocity([0.0] * 6)
    except Exception:
        pass


def suck_off():
    """Release the vacuum: drop the part (let physics take over)."""
    global _suck
    if _suck is not None:
        try:
            _suck[0].resetPhysics()
        except Exception:
            pass
    _suck = None


# Wall-gap normalisation (must match the trainer) — uses THIS bin's footprint;
# the fixed norm + clip make "0 == packed against a wall" read the same here as
# in the (smaller) training bin, so the local push policy transfers.
GAP_NORM = 0.12
# Pre-grasp push primitive (must match the trainer). The gate is env-overridable
# so deploy can fire the policy's learned push directions even when its
# deterministic (no-exploration) mean push magnitude sits below the training gate.
PUSH_GATE = float(os.environ.get("BINPICK_PUSH_GATE", "0.40"))
PUSH_Z = 0.05
PUSH_BACK = 0.05
PUSH_LEN = 0.08

_gp_path = os.environ.get(
    "GRASP_POLICY_ONNX",
    os.path.join(_HERE, "..", "..", "..", "..", "rl", "inference",
                 "policies", "omniarm6_grasp", "policy.onnx"))
_gp = None
if os.environ.get("BINPICK_USE_RL", "1") != "0":
    try:
        import onnxruntime as ort
        if os.path.exists(_gp_path):
            _gp = ort.InferenceSession(_gp_path, providers=["CPUExecutionProvider"])
    except Exception:
        _gp = None


def _up_vec(node):
    o = node.getOrientation()
    return [o[2], o[5], o[8]]


def _wall_gaps(p):
    return [min(1.5, max(0.0, (p[0] - BIN_X[0]) / GAP_NORM)),
            min(1.5, max(0.0, (BIN_X[1] - p[0]) / GAP_NORM)),
            min(1.5, max(0.0, (p[1] - BIN_Y[0]) / GAP_NORM)),
            min(1.5, max(0.0, (BIN_Y[1] - p[1]) / GAP_NORM))]


def policy_action(node, parts, done):
    """(push_x, push_y, dx, dy, dz, dyaw) from the policy, conditioned on the
    cube's orientation, nearest neighbours and wall gaps (same obs the policy
    trained on). All zeros if no policy / disabled."""
    if _gp is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    tp = node.getPosition()
    others = [parts[n] for n in parts if n not in done and parts[n] is not node]
    nb = sorted((math.dist(o.getPosition(), tp), o) for o in others)[:N_NEAR]
    obs = list(_up_vec(node))
    for _, o in nb:
        p = o.getPosition()
        obs += [(p[0] - tp[0]) / 0.07, (p[1] - tp[1]) / 0.07, (p[2] - tp[2]) / 0.07]
    while len(obs) < 3 + N_NEAR * 3:
        obs += [0.0]
    obs += _wall_gaps(tp)
    a = _gp.run(None, {"obs": np.asarray(obs[:GP_OBS], dtype=np.float32)[None, :]})[0][0]
    g = a[2:6] * GP_GRASP_SCALE
    return (float(a[0]), float(a[1]),
            float(g[0]), float(g[1]), float(g[2]), float(g[3]))


def push_clear(tx, ty, px, py):
    """GATED pre-grasp push: if |(px,py)| clears the gate, bring the closed
    gripper down on the -dir side of the target and sweep +dir, shoving it
    toward open space so the jaws can get around it. Returns True if it pushed."""
    m = math.hypot(px, py)
    if _gp is None or m < PUSH_GATE:
        return False
    dx, dy = px / m, py / m
    sx, sy = tx - dx * PUSH_BACK, ty - dy * PUSH_BACK
    ex, ey = tx + dx * PUSH_LEN, ty + dy * PUSH_LEN
    emit("[bin]   ~~ pre-grasp push toward open space ~~")
    bridge.act_grasp()                              # close -> a thin blade
    step_for(0.3)
    grasp_pose(sx, sy, CARRY_Z, 0.0)                # stage above start
    grasp_pose(sx, sy, PUSH_Z, 0.0)                 # descend beside the target
    grasp_pose(ex, ey, PUSH_Z, 0.0, dur=1.3)        # sweep, shoving the target
    grasp_pose(ex, ey, CARRY_Z, 0.0)               # lift clear
    bridge.act_open_gripper()
    step_for(0.4)
    return True


def grasp_pose(x, y, z, yaw, dur=1.4):
    """Top-down grasp pose rotated about world Z by yaw (lets the policy turn the
    jaws toward a gap). yaw == 0 uses the bridge's plain top-down path."""
    if abs(yaw) < 1e-4:
        bridge.act_set_tcp_pose((x, y, z), tcp_offset_z=OZ, duration_s=dur)
    else:
        c, s = math.cos(yaw), math.sin(yaw)
        rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
        top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        q, _, _, _ = dls_ik_pose(IK["chain"], bridge._read_q(), [x, y, z],
                                 _mat_mul(rz, top), (0.0, 0.0, OZ), IK,
                                 bridge.joint_limits)
        bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + 0.3)


def emit(s):
    fh.write(s + "\n")
    print(s, flush=True)


def step_for(secs):
    """Advance the sim, driving the bridge motion each tick (and the bin weld when
    an agitation hold is active)."""
    n = int(secs * 1000 / dt)
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if _suck is not None:
            suck_apply()                        # vacuum: held part tracks the cup lip
        if _held:
            _apply_hold()                       # agitation: bin tracks the gripper
        elif _pin_bin and _binnode is not None:
            # Only correct when the bin has actually DRIFTED past a few mm (a gripper
            # bump). Resetting every tick jitters the cubes and slows the sim; this
            # leaves the bin alone when it's already home, so picking is clean+fast.
            bp = _binnode.getPosition()
            if (abs(bp[0] - BIN_HOME[0]) > 0.004 or abs(bp[1] - BIN_HOME[1]) > 0.004
                    or abs(bp[2] - BIN_HOME[2]) > 0.006):
                _bin_tr.setSFVec3f(BIN_HOME)
                _bin_rot.setSFRotation([0.0, 0.0, 1.0, 0.0])
    return True


def discover_parts():
    """Every DEF PART_<n> present in the scene, as {name: node}. Also fills
    ID_TO_NAME so camera recognition ids map back to cubes."""
    parts = {}
    i = 1
    misses = 0
    while misses < 6:                 # tolerate gaps in the numbering
        name = "PART_%d" % i
        node = robot.getFromDef(name)
        if node is not None:
            parts[name] = node
            ID_TO_NAME[node.getId()] = name
            misses = 0
        else:
            misses += 1
        i += 1
    return parts


def part_color(node):
    """'red' or 'blue' from the part's PBRAppearance baseColor."""
    try:
        shape = node.getField("children").getMFNode(0)
        appr = shape.getField("appearance").getSFNode()
        col = appr.getField("baseColor").getSFColor()
        return "red" if col[0] >= col[2] else "blue"
    except Exception:
        return "red"


def pz(node):
    p = node.getPosition()
    return [round(p[0], 3), round(p[1], 3), round(p[2], 3)]


def in_bin(node):
    # Footprint follows the (movable) bin so a small drift doesn't misclassify
    # cubes as "emptied". Half-extents 0.20 (a touch outside the 0.19 interior).
    p = node.getPosition()
    bp = _binnode.getPosition() if _binnode is not None else BIN_HOME
    return (abs(p[0] - bp[0]) <= 0.20 and abs(p[1] - bp[1]) <= 0.20
            and p[2] < CARRY_Z - 0.05)


def on_zone(node, color):
    """Is the part resting on its matching colour pad?"""
    p = node.getPosition()
    if p[2] >= 0.16:
        return False
    if color == "red":
        return 0.16 <= p[0] <= 0.44 and -0.57 <= p[1] <= -0.31
    return -0.16 <= p[0] <= 0.12 and -0.57 <= p[1] <= -0.31


def pose(xyz, dur=1.4, settle=0.3):
    bridge.act_set_tcp_pose(tuple(xyz), tcp_offset_z=OZ, duration_s=dur)
    step_for(dur + settle)


def next_pick(parts, done):
    """Choose the next cube to pick, DRIVEN BY THE CAMERA: among the cubes the
    top-down camera currently recognises (occlusion -> the exposed top of the
    pile), take the highest one still in the bin. If the camera sees nothing
    usable (occluded / stale), fall back to the highest in-bin cube from the
    scene tree so the run never stalls. Returns (name, node, colour, source)."""
    seen = camera_detection()                          # {name: colour}
    cands = [(n, parts[n], seen[n]) for n in seen
             if n in parts and n not in done and in_bin(parts[n])]
    src = "cam"
    if not cands:                                      # camera blind -> fallback
        cands = [(n, parts[n], part_color(parts[n])) for n in parts
                 if n not in done and in_bin(parts[n])]
        src = "scene"
    if not cands:
        return None
    cands.sort(key=lambda c: c[1].getPosition()[2], reverse=True)
    return cands[0][0], cands[0][1], cands[0][2], src


def spread_pile():
    """SINGULATION: rake the closed gripper back and forth through the pile to
    flatten it and open gaps between cubes, so the wide 2F-85 fingers can get
    around a cube. Run on a packed bin before / between picks -- real bin-
    pickers do this when a pile is too dense to grasp."""
    emit("[bin]   ~~ raking the pile to separate the cubes (singulation) ~~")
    bridge.act_grasp()                              # close fingers -> a rake
    step_for(0.4)
    z = 0.045                                       # blade just above the floor
    xa, xb = BIN_X[0] + 0.04, BIN_X[1] - 0.04
    ya, yb = BIN_Y[0] + 0.04, BIN_Y[1] - 0.04
    for k, yy in enumerate((-0.11, -0.037, 0.037, 0.11)):   # lanes along X
        a, b = (xa, xb) if k % 2 == 0 else (xb, xa)
        pose([a, yy, z + 0.12], dur=0.9)
        pose([a, yy, z], dur=0.7)
        pose([b, yy, z], dur=1.5)                   # drag across
        pose([b, yy, z + 0.12], dur=0.7)
    for k, xx in enumerate((0.35, 0.42, 0.50, 0.57)):       # lanes along Y
        a, b = (ya, yb) if k % 2 == 0 else (yb, ya)
        pose([xx, a, z + 0.12], dur=0.9)
        pose([xx, a, z], dur=0.7)
        pose([xx, b, z], dur=1.5)
        pose([xx, b, z + 0.12], dur=0.7)
    bridge.act_open_gripper()
    step_for(1.5)                                   # let the spread pile settle


def empty_one(name, node, slot):
    """Pick `node` out of the bin and place it on `slot`. The policy may first
    run a GATED pre-grasp push to declutter the target, then a residual-adjusted
    grasp. Returns True once the part is lifted and resting on a pad."""
    px, py, dx, dy, dz, dyaw = policy_action(node, parts, done)  # push + residual
    p0 = node.getPosition()
    push_clear(p0[0], p0[1], px, py)               # gated declutter (no-op if 0)
    for attempt in range(1, MAX_ATTEMPTS + 1):
        p = node.getPosition()                     # re-read (push moved it)
        ax, ay, az = p[0] + dx, p[1] + dy, p[2] + dz
        bridge.act_set_gripper_width(GRASP_WIDTH)  # pre-open narrow (see const)
        step_for(0.4)
        grasp_pose(ax, ay, az + APPROACH_DZ, dyaw)  # stage above (yawed)
        grasp_pose(ax, ay, az, dyaw)                # descend
        bridge.act_grasp()                          # close — friction grip
        step_for(1.4)                               # let the grip settle
        grasp_pose(ax, ay, CARRY_Z, dyaw)           # lift straight up
        if node.getPosition()[2] < LIFT_OK_Z:       # grasp slipped
            emit("[bin]   %s grasp slipped, attempt %d/%d"
                 % (name, attempt, MAX_ATTEMPTS))
            bridge.act_open_gripper()
            step_for(0.3)
            continue
        sx, sy = slot
        grasp_pose(sx, sy, CARRY_Z, dyaw, dur=1.8)  # carry over the tray slot
        grasp_pose(sx, sy, DROP_Z, dyaw)            # lower to place
        bridge.act_open_gripper()                   # release
        step_for(1.0)
        grasp_pose(sx, sy, CARRY_Z, dyaw)           # retreat up
        out = not in_bin(node)
        emit("[bin]   %s -> %s @ %s out=%s" % (name, slot, pz(node), out))
        return out
    return False


# ── Sequential declutter mode (multi-step push/grasp loop; optional) ─
# Off by default. With BINPICK_DECLUTTER=1 the deploy runs a per-macro-step
# push/grasp LOOP: if the target has a touching same-layer neighbour blocking
# the jaws, push that blocker away (neighbour-clearing push), else grasp with
# the proven single-step residual; the loop re-observes so a cleared cube is
# picked next.
#
# FINDING (kept as a documented, reproducible negative result): on this dense
# 36-cube bin EVERY push variant tried emptied FEWER cubes than the no-push
# residual -- single-step push dormant 18, forced 17, learned-PPO 6, heuristic
# sequential 8. The 85 mm jaws can't get TWO-SIDED clearance on a wall-locked
# 50 mm cube, and a push just re-packs the pile. So the bottleneck is the
# end-effector geometry, not the policy. The learned multi-step approach lives
# in projects/policies/controllers/omniarm6_declutter_train (PPO actor-critic); it reached
# ~80% in its small training scenarios but under-generalised to the full bin.
DECL_ON = os.environ.get("BINPICK_DECLUTTER", "0") != "0"
DC_PUSH_CELL, DC_PUSH_LEN, DC_PUSH_Z = 0.052, 0.07, 0.05
DC_MAX_PUSH = 3                                    # per-cube push budget at deploy


def decl_push(node, px, py):
    """Neighbour-clearing push: descend the closed gripper on the +dir blocker
    and sweep it away, opening a gap on the target's +dir side."""
    m = math.hypot(px, py) or 1.0
    dx, dy = px / m, py / m
    tp = node.getPosition()
    sx, sy = tp[0] + dx * DC_PUSH_CELL, tp[1] + dy * DC_PUSH_CELL
    ex, ey = tp[0] + dx * (DC_PUSH_CELL + DC_PUSH_LEN), tp[1] + dy * (DC_PUSH_CELL + DC_PUSH_LEN)
    emit("[bin]   ~~ declutter push: clear blocker along (%.2f, %.2f) ~~" % (dx, dy))
    bridge.act_grasp()
    step_for(0.3)
    grasp_pose(sx, sy, CARRY_Z, 0.0)
    grasp_pose(sx, sy, DC_PUSH_Z, 0.0)
    grasp_pose(ex, ey, DC_PUSH_Z, 0.0, dur=1.2)
    grasp_pose(ex, ey, CARRY_Z, 0.0)
    bridge.act_open_gripper()
    step_for(0.4)


def grasp_place(node, slot, residual):
    """One residual grasp attempt + place into the colour tray. Returns out."""
    dx, dy, dz, dyaw = residual
    p = node.getPosition()
    ax, ay, az = p[0] + dx, p[1] + dy, p[2] + dz
    bridge.act_set_gripper_width(GRASP_WIDTH)
    step_for(0.4)
    grasp_pose(ax, ay, az + APPROACH_DZ, dyaw)
    grasp_pose(ax, ay, az, dyaw)
    bridge.act_grasp()
    step_for(1.4)
    grasp_pose(ax, ay, CARRY_Z, dyaw)
    if node.getPosition()[2] < LIFT_OK_Z:
        bridge.act_open_gripper(); step_for(0.3)
        return False
    sx, sy = slot
    grasp_pose(sx, sy, CARRY_Z, dyaw, dur=1.8)
    grasp_pose(sx, sy, DROP_Z, dyaw)
    bridge.act_open_gripper(); step_for(1.0)
    grasp_pose(sx, sy, CARRY_Z, dyaw)
    return not in_bin(node)


def suction_pick(node, slot):
    """SUCTION pick-and-place. Bring the cup lip straight down onto the cube's TOP face,
    engage the vacuum (weld the cube flush to the lip), lift straight up, carry to the
    matching colour tray, release. NO side clearance is needed -- the cup takes the top of
    the pile -- so a dense bin empties top-down with no shaking and no jaw-geometry limit.
    Returns True if the cube ended out of the bin."""
    p = node.getPosition()
    top = p[2] + 0.025
    grasp_pose(p[0], p[1], CARRY_Z, 0.0)                 # stage above the cube
    grasp_pose(p[0], p[1], top + 0.005, 0.0)             # bring the cup lip to the top face
    # TOUCH SERVO: the commanded pose can leave a real air gap (PD sag / IK
    # undershoot). Nudge the lip down until it MEASURABLY rests on the cube's
    # top face, so the vacuum engages on physical contact instead of snapping
    # the cube up onto the cup from centimetres away. Contact is detected as
    # "the gap stopped shrinking": once the lip is on the cube, further nudges
    # only squeeze the pile against the bin floor (and visibly shake the bin),
    # so stop there. The press floor is 4 mm below the cube's original top.
    z_cmd = top + 0.005
    prev_gap = None
    for _ in range(6):
        gap = _tcp_pose()[0][2] - (node.getPosition()[2] + 0.025)
        if gap <= 0.002:
            break
        if prev_gap is not None and prev_gap - gap < 0.001:
            break                                        # pressing, not approaching
        prev_gap = gap
        z_cmd = max(z_cmd - min(max(gap, 0.002), 0.008), top - 0.004)
        grasp_pose(p[0], p[1], z_cmd, 0.0, dur=0.3)
    # RELAX to the MEASURED touch height: the servo's last command sits below
    # the contact point, so without this the PD keeps PRESSING into the pile at
    # motor force through engage -- a press that wedges neighbour cubes outward
    # against the walls (splitting-wedge multiplication) and rocks the bin on
    # every pick (measured 2-4 mm even with a 30 kg bin).
    grasp_pose(p[0], p[1], _tcp_pose()[0][2] + 0.001, 0.0, dur=0.25)
    suck_on(node); step_for(0.3)                         # vacuum ON at touch (no snap)
    # SLOW PEEL: ease the first few cm of lift so a cube leaving a dense pile
    # slides off its neighbours instead of prying them (and the bin) at speed.
    grasp_pose(p[0], p[1], min(top + 0.08, CARRY_Z), 0.0, dur=0.9)
    grasp_pose(p[0], p[1], CARRY_Z, 0.0)                 # lift straight up
    if node.getPosition()[2] < LIFT_OK_Z:                # vacuum didn't take (rare)
        suck_off(); step_for(0.2)
        return False
    grasp_pose(slot[0], slot[1], CARRY_Z, 0.0, dur=1.8)  # carry to the tray
    grasp_pose(slot[0], slot[1], DROP_Z, 0.0)
    suck_off(); step_for(0.6)                            # vacuum OFF -> drop into the tray
    grasp_pose(slot[0], slot[1], CARRY_Z, 0.0)
    return not in_bin(node)


def _same_layer_blocker(node, others):
    """Nearest cube at the target's height (the real obstacle to a top-down jaw).
    Returns (blocker_node, distance) or (None, inf)."""
    p = node.getPosition()
    best, bd = None, 1e9
    for o in others:
        q = o.getPosition()
        if abs(q[2] - p[2]) < 0.045 and math.hypot(q[0] - p[0], q[1] - p[1]) < bd:
            best, bd = o, math.hypot(q[0] - p[0], q[1] - p[1])
    return best, bd


def run_declutter():
    """Sequential push/grasp LOOP. Each macro-step: if the target has a touching
    same-layer neighbour blocking the jaws, push that blocker away (neighbour-
    clearing push, direction = away-from-target geometry); otherwise grasp with
    the proven single-step residual. Loop re-observes so a cleared cube is picked
    next. (The PPO actor in projects/policies/omniarm6_declutter learns this push/grasp
    decision; on the full bin its small-scenario policy under-generalised, so the
    deploy loop gates on a robust geometric blocked-test and keeps the learned
    push primitive + the proven grasp residual.)"""
    done = set()
    emptied = red_i = blue_i = 0
    pushes, gfails = {}, {}
    guard = 0
    while guard < 250:
        guard += 1
        step_for(0.3)
        pick = next_pick(parts, done)
        if pick is None:
            break
        name, node, color, _ = pick
        others = [parts[n] for n in parts
                  if n not in done and parts[n] is not node and in_bin(parts[n])]
        slot = (RED_SLOTS[red_i % len(RED_SLOTS)] if color == "red"
                else BLUE_SLOTS[blue_i % len(BLUE_SLOTS)])
        blk, bd = _same_layer_blocker(node, others)
        if blk is not None and bd < 0.064 and pushes.get(name, 0) < DC_MAX_PUSH:   # PUSH
            q = blk.getPosition()
            p = node.getPosition()
            emit("[bin] PUSH %s (%s, push#%d) clear blocker @ d=%.3f"
                 % (name, color, pushes.get(name, 0) + 1, bd))
            decl_push(node, q[0] - p[0], q[1] - p[1])   # shove the blocker away
            pushes[name] = pushes.get(name, 0) + 1
            if not in_bin(node):
                done.add(name)
        else:                                                                      # GRASP
            _, _, dx, dy, dz, dyaw = policy_action(node, parts, done)   # proven residual
            emit("[bin] GRASP %s (%s, pushes=%d) @ %s"
                 % (name, color, pushes.get(name, 0), pz(node)))
            if grasp_place(node, slot, (dx, dy, dz, dyaw)):
                done.add(name); emptied += 1
                if color == "red":
                    red_i += 1
                else:
                    blue_i += 1
            elif not in_bin(node):
                done.add(name)
            else:
                gfails[name] = gfails.get(name, 0) + 1
                if gfails[name] >= 2:
                    emit("[bin]   %s unrecoverable; retiring" % name)
                    done.add(name)
    return done, emptied


# ── Grab-and-agitate (singulation by manipulating the whole bin) ────
def agitate_pose(x, y, z, rx=0.0, ry=0.0, dur=0.6, settle=0.15):
    """Move the tool to (x,y,z) with the top-down grasp tilted rx about world X and
    ry about world Y. Used while the bin is welded to the gripper so the bin tilts
    and the loose cubes inside slide/tumble. The weld is applied each tick in step_for."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    Rx = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]
    Ry = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]
    top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    Rt = _mat_mul(_mat_mul(Ry, Rx), top)
    q, _, _, _ = dls_ik_pose(IK["chain"], bridge._read_q(), [x, y, z], Rt,
                             (0.0, 0.0, OZ), IK, bridge.joint_limits)
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + settle)


def agitate_bin():
    """Grab the bin by its NORTH wall, lift it, SHAKE + TILT it to tumble wall-locked
    cubes into reachable spots, then set it back at home and release. The weld holds
    the bin to the gripper; the loose cubes redistribute for real. This is the whole
    idea -- manipulate the bin (everything we built) to beat the jaw-clearance limit."""
    emit("[bin]   ~~ AGITATE: grab the bin + shake it to free the stuck cubes ~~")
    gx, gy, gz = 0.46, 0.196, 0.20         # grab the north wall, just below the rim
    bridge.act_reset_to_home(); step_for(0.5)
    bridge.act_open_gripper(); step_for(0.4)
    agitate_pose(gx, gy, gz + 0.16, dur=1.5)     # stage above the wall
    agitate_pose(gx, gy, gz, dur=1.1)            # descend straddling the wall
    bridge.act_grasp(); step_for(0.8)
    grab_hold()                                  # weld bin -> gripper
    # Lift only a little, tilt SLOWLY, and barely shake: the cubes must slide and
    # tumble to free the stuck ones, but a fast/large motion flings them over the
    # 0.22 m walls. Slow tilts (cubes slide under gravity, not thrown) + a tiny slosh
    # + a settle after each tilt keep them in while still redistributing the pile.
    lz = gz + 0.04
    agitate_pose(gx, gy, lz, dur=1.3)
    for ry in (0.16, -0.16, 0.18, -0.18, 0.0):   # slow rolls about Y -> slide in X
        agitate_pose(gx, gy, lz, ry=ry, dur=1.1, settle=0.35)
    for rx in (0.15, -0.15, 0.0):                # slow rolls about X -> slide in Y
        agitate_pose(gx, gy, lz, rx=rx, dur=1.1, settle=0.35)
    for _ in range(2):                           # gentle slosh
        agitate_pose(gx + 0.025, gy, lz, dur=0.5)
        agitate_pose(gx - 0.025, gy, lz, dur=0.5)
    agitate_pose(gx, gy, gz, dur=1.3)            # lower the bin back home
    step_for(0.4)
    release_hold()                               # unlink the weld
    # FORCE the bin exactly home + level (guarantee no drift / tilt left over), then
    # let the cubes settle into their new, reachable positions.
    _bin_tr.setSFVec3f([BIN_HOME[0], BIN_HOME[1], BIN_HOME[2]])
    _bin_rot.setSFRotation([0.0, 0.0, 1.0, 0.0])
    step_for(1.4)
    bridge.act_open_gripper(); step_for(0.4)
    agitate_pose(gx, gy, gz + 0.22, dur=1.2)     # retreat up
    bridge.act_reset_to_home(); step_for(1.5)    # clear the camera, let cubes settle
    emit("[bin]   ~~ agitation done; re-observing the bin ~~")


MAX_AGITATES = int(os.environ.get("BINPICK_MAX_AGITATES", "5"))

# ── Run ──────────────────────────────────────────────────────────────
step_for(6.0)                                      # let the dumped pile settle
parts = discover_parts()
reds = sum(1 for n in parts if part_color(parts[n]) == "red")
emit("[bin] start    %d parts (%d red / %d blue)" % (len(parts), reds, len(parts) - reds))
emit("[bin] camera sees %d cubes on top of the pile" % len(camera_detection()))
emit("[bin] RL push+grasp policy: %s" % ("ON" if _gp is not None else "OFF (plain grasp)"))
emit("[bin] sequential declutter mode: %s" % ("ON" if DECL_ON else "off (default)"))

# Fast isolated check of the agitation alone (BINPICK_AGITATE_TEST=1): do ONE shake
# right away and report how the cubes redistributed + whether any escaped + bin drift.
if os.environ.get("BINPICK_AGITATE_TEST"):
    before = [n for n in parts if in_bin(parts[n])]
    p0 = {n: parts[n].getPosition()[:2] for n in before}
    b0 = _binnode.getPosition()
    emit("[agtest] BEFORE: %d cubes in bin; bin@(%.3f,%.3f,%.3f)"
         % (len(before), b0[0], b0[1], b0[2]))
    agitate_bin()
    after = [n for n in parts if in_bin(parts[n])]
    b1 = _binnode.getPosition()
    moved = sum(1 for n in after
                if abs(parts[n].getPosition()[0] - p0.get(n, [9, 9])[0]) > 0.02
                or abs(parts[n].getPosition()[1] - p0.get(n, [9, 9])[1]) > 0.02)
    emit("[agtest] AFTER: %d in bin (escaped=%d, moved>2cm=%d/%d); bin@(%.3f,%.3f,%.3f)"
         % (len(after), len(before) - len(after), moved, len(before),
            b1[0], b1[1], b1[2]))
    emit("PASS" if (len(before) - len(after)) == 0 and moved >= 6 else "FAIL")
    bridge.act_reset_to_home(); step_for(1.0)
    if os.environ.get("BINPICK_AUTOQUIT"):
        try:
            robot.simulationQuit(0)
        except Exception:
            pass
    DECL_ON = None                                  # skip the normal run below

if DECL_ON is None:
    pass
elif DECL_ON:
    # Multi-step: each macro-step the policy chooses push or grasp (push a
    # blocker, re-observe, then grasp) — cracks wall-locked cubes.
    done, emptied = run_declutter()
else:
    # SUCTION pick-and-sort. The vacuum takes the TOP of the pile, so there is NO jaw-
    # clearance limit and NO need to shake: just take the highest cube each pass and the
    # dense bin empties top-down, cube by cube, until it's empty.
    done = set()
    red_i = blue_i = 0
    tries = {}
    last_color = None                                 # colour-batching (see pick order)
    guard = 0
    GUARD_MAX = 300
    while guard < GUARD_MAX:
        guard += 1
        step_for(0.3)                                 # arm clear -> scene/camera refresh
        inbin = [n for n in parts if n not in done and in_bin(parts[n])]
        if not inbin:
            break                                     # bin empty
        # PICK ORDER (travel-aware). Top-down safety still rules: only cubes in the
        # current top LAYER are candidates (a lower cube can have one above it).
        # Within the layer, two things cut the arm's swing:
        #   * same COLOUR as the last pick -> consecutive trips go to the SAME tray
        #     (a strict z-sort alternated red/blue, so the arm crossed between the
        #     two trays on nearly every trip);
        #   * then NEAREST to where the tool already is (same-layer cubes differ
        #     only by settling noise, so z-order was effectively arbitrary and made
        #     the arm zigzag across the bin).
        inbin.sort(key=lambda n: parts[n].getPosition()[2], reverse=True)   # highest first
        ztop = parts[inbin[0]].getPosition()[2]
        layer = [n for n in inbin if ztop - parts[n].getPosition()[2] < 0.03]
        _tp = _tcp_pose()[0]

        def _cost(n):
            q = parts[n].getPosition()
            d2 = (q[0] - _tp[0]) ** 2 + (q[1] - _tp[1]) ** 2
            same = (last_color is not None and part_color(parts[n]) == last_color)
            return (0 if same else 1, d2)

        layer.sort(key=_cost)
        name = layer[0]
        node = parts[name]
        color = part_color(node)
        tries[name] = tries.get(name, 0) + 1
        slot = (RED_SLOTS[red_i % len(RED_SLOTS)] if color == "red"
                else BLUE_SLOTS[blue_i % len(BLUE_SLOTS)])
        emit("[bin] picking %s (%s, top, try %d) @ %s" % (name, color, tries[name], pz(node)))
        if suction_pick(node, slot):
            done.add(name)
            last_color = color                        # keep working the same tray
            if color == "red":
                red_i += 1
            else:
                blue_i += 1
        elif not in_bin(node):
            done.add(name)                            # left the bin some other way
        elif tries[name] >= 4:
            done.add(name)                            # genuinely stuck -> give up
            emit("[bin]   -> %s unliftable after %d tries" % (name, tries[name]))
        # else: leave it -- after the cubes above it clear + settle, a retry succeeds
    emptied = sum(1 for n in parts if not in_bin(parts[n]))

    # RECOVERY: vacuum up anything that ended off its matching tray (a cube that missed,
    # rolled out, or landed in the wrong tray). Isolated now, so a clean top pick fixes it.
    rtries = {}
    for _ in range(80):
        bad = next((n for n in parts
                    if not on_zone(parts[n], part_color(parts[n]))
                    and rtries.get(n, 0) < 3), None)
        if bad is None:
            break
        node = parts[bad]
        col = part_color(node)
        slot = (RED_SLOTS[red_i % len(RED_SLOTS)] if col == "red"
                else BLUE_SLOTS[blue_i % len(BLUE_SLOTS)])
        emit("[bin] RECOVER %s (%s) from %s -> %s tray" % (bad, col, pz(node), col))
        if suction_pick(node, slot):
            if col == "red":
                red_i += 1
            else:
                blue_i += 1
        else:
            rtries[bad] = rtries.get(bad, 0) + 1

# Final tally. Recompute emptied from the FINAL scene (the value above was measured
# before the recovery pass, so it understated the result once recovery ran).
bridge.act_reset_to_home()
step_for(1.2)
remaining = [n for n in parts if in_bin(parts[n])]
emptied = sum(1 for n in parts if not in_bin(parts[n]))
sorted_ok = sum(1 for n in parts if on_zone(parts[n], part_color(parts[n])))
box_empty = not remaining
n = len(parts)
# "Decent performance" bar: clears >=90% of the box and lands >=75% in the
# right tray. The last back-row cubes nearest a wall are the hard case for a
# gripper this size; a couple sticking does not fail the demo.
ok = emptied >= 0.9 * n and sorted_ok >= 0.75 * n
emit("[bin] RESULT emptied=%d/%d sorted=%d/%d box_empty=%s remaining=%s | "
     "wall=%.1fs sim=%.1fs"
     % (emptied, n, sorted_ok, n, box_empty,
        remaining, time.time() - t0, robot.getTime()))
emit("PASS" if ok else "FAIL")

if os.environ.get("BINPICK_AUTOQUIT"):
    try:
        robot.simulationQuit(0 if ok else 1)
    except Exception:
        pass
else:
    while robot.step(dt) != -1:
        bridge.tick(robot.getTime())
