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

"""OMNIARM6 TOSS-TO-PLACE deploy controller (Shadowing Component 3 -- direct ghost deploy).

Picks a cube with the vacuum cup, then executes the feasibility-CERTIFIED toss ghost
(projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz) and releases the vacuum at the planned
instant, throwing the cube into a bin BEYOND the arm's kinematic reach.

This is the DIRECT deploy of the certified ghost (no RL residual): a fully-actuated, bolted-
down arm tracks a feasible joint trajectory well, so we test first whether the ghost simply
reproduces in OmniSim Newton.

RATED LIMITS ARE RESPECTED (2026-07-11). Every motor command is sourced from the robot's OWN
URDF limits -- the velocity cap from `Motor.getMaxVelocity()` (omniarm6_suction.urdf declares
3.1415 / 3.4906 rad/s) and the torque cap from `Motor.getMaxTorque()` (194 / 102 / 34 N.m).
Nothing here commands the arm past its datasheet, and the world does NOT disable the engine's
effort clamp. The certified ghost peaks at 3.037 rad/s = 97% of the rated joint speed, so it
is feasible on the real machine BY CONSTRUCTION (see generate_omniarm6_toss.py).

THE VACUUM HOLD IS FORCE-COUPLED, NOT A TELEPORT. The held cube is carried by a PD force
(`Node.addForce`), not by a per-tick `setSFVec3f` pose write. This matters enormously: under
the Newton/MuJoCo backend a per-tick Supervisor POSE write on any dynamic body calls
`eval_fk` + flags `_mjc_dirty`, forcing a full MuJoCo<->Newton state resync EVERY tick, which
destroys the arm's accumulated joint velocity. With the old teleport hold the shoulder could
not exceed ~1.2 rad/s (it needs 3.04) even at full rated torque, and the throw went BACKWARDS.
The old workaround was OMNISIM_NEWTON_NO_EFFORT_LIMIT=1 -- i.e. deleting the arm's rated torque
cap so it could brute-force through its own resync losses. That workaround is GONE.
Because the cube is now held by a real force, its release velocity is its OWN physical
velocity: we simply stop applying the force. Nothing is hand-set at release.

env:
  OMNIARM6_TOSS_GHOST   path to the ghost npz (default: repo ghosts/omniarm6_toss_ghost.npz)
  OMNIARM6_TOSS_AUTOQUIT=1   quit after one throw (headless verify)
  OMNIARM6_TOSS_OUT     result file (default: _toss_result.txt at the repo root)
  OMNIARM6_TOSS_TRACE=1      per-tick joint tracking trace (debug)
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import numpy as np  # noqa: E402
from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, warmup_reload,  # noqa: E402
                                 forward_kinematics_pose)
from _arm_configs import get_config  # noqa: E402

OZ = 0.25                       # tool point = suction cup lip (flange + ~0.085)
LEAD = float(os.environ.get("OMNIARM6_TOSS_LEAD", "0.0"))    # swing reference lead-time (s)
G = 9.81
# Landing height used by the release projection: the cube's CENTRE at rest on the bin floor
# (bin floor top face = 0.02, cube half-extent = 0.025). Getting this right is what makes the
# ballistic aim land on the bin centre instead of short.
BIN_RIM_Z = 0.045


def ballistic_land_x(p, v, zb=BIN_RIM_Z):
    """Forward landing x of a projectile released at world pos p with world vel v, or None."""
    z, vz = float(p[2]), float(v[2])
    disc = vz * vz + 2.0 * G * (z - zb)
    if disc < 0 or v[0] <= 0.0:
        return None
    t = (vz + math.sqrt(disc)) / G
    return None if t <= 0 else p[0] + v[0] * t
CUP_HOLD_DZ = 0.025             # cube centre rides a half-cube below the lip (flush)
# Aim the ballistic arc at the bin CENTRE (SKID=0). The cube touches down still carrying
# ~2.7 m/s of horizontal speed and skids forward a few cm before stopping, so it comes to rest
# past the centre -- but still inside. Do NOT "correct" for the skid by aiming short: releasing
# earlier leaves the arm mid-follow-through, and the cup then CLIPS the cube on its way past
# (measured: aiming 12 cm short dropped it OUTSIDE the bin at x=0.69). Late release = clean release.
SKID = float(os.environ.get("OMNIARM6_TOSS_SKID", "0.0"))
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
PICK_XY = (0.45, 0.0)          # pedestal location (cube sits on top)
PICK_TOP_Z = 0.15              # cube top face height (pedestal 0.10 + cube 0.05)

_REPO = os.environ.get("OMNISIM_HOME") or os.path.abspath(os.path.join(_HERE, "..", "..", "..", "..", ".."))
GHOST = os.environ.get("OMNIARM6_TOSS_GHOST",
                       os.path.join(_REPO, "projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz"))
OUT = os.environ.get("OMNIARM6_TOSS_OUT", os.path.join(_REPO, "_toss_result.txt"))
AUTOQUIT = os.environ.get("OMNIARM6_TOSS_AUTOQUIT", "0") != "0"

robot = Supervisor()
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
dt_s = dt / 1000.0
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
IK = bridge.cfg["ik"]
fh = open(OUT, "w", encoding="utf-8", buffering=1)


def emit(msg):
    print(msg)
    fh.write(msg + "\n")


# --- load the certified ghost ---
g = np.load(GHOST, allow_pickle=True)
GQ = g["q"][:, :6].astype(float)            # [N, 6] joint angles in JOINTS order
GDT = float(g["dt"])
REL_K = int(g["release_k"])
BIN_X = float(g["bin_x"]) if "bin_x" in g.files else 1.30
N_GHOST = GQ.shape[0]
T_REL = REL_K * GDT
T_SWING = (N_GHOST - 1) * GDT
AIM_X = BIN_X - SKID            # ballistic aim point (see SKID)
emit(f"[toss] ghost {GHOST}")
emit(f"[toss] {N_GHOST} steps @ {GDT*1000:.0f}ms, swing {T_SWING:.2f}s, release @ {T_REL:.2f}s, target bin x={BIN_X:.2f}")
emit(f"[toss] aim x={AIM_X:.2f} (bin {BIN_X:.2f} minus {SKID*100:.0f} cm touchdown skid)")

# --- joint motors for direct swing control ---
# RATED LIMITS, SOURCED FROM THE ROBOT ITSELF. getMaxVelocity()/getMaxTorque() return the
# URDF <limit velocity=.../ effort=.../> the importer baked into each motor, so we can never
# drift out of sync with omniarm6_suction.urdf. We command the motor AT its rated cap -- never
# past it. (The old code hard-coded setVelocity(8.0), 2.5x the OMNIARM6's rated 3.1415 rad/s;
# the engine clamped it back down to the URDF value and printed an over-speed WARNING on
# every joint. The clamp meant the 8.0 never did anything except produce that warning.)
def _rated(motor, getter, fallback):
    try:
        v = float(getattr(motor, getter)())
        if v > 0.0 and math.isfinite(v):
            return v
    except Exception:
        pass
    return fallback


motors = []
VMAX = []
TMAX = []
for i, jn in enumerate(JOINTS):
    m = robot.getDevice(jn + "_motor") or robot.getDevice(jn)
    if m is None:
        emit(f"[toss] FATAL: motor for {jn} not found")
        raise SystemExit(1)
    vmax = _rated(m, "getMaxVelocity", 3.1415)
    tmax = _rated(m, "getMaxTorque", _rated(m, "getMaxForce", 100.0))
    VMAX.append(vmax)
    TMAX.append(tmax)
    motors.append(m)


def arm_rated_limits():
    """(Re)assert the rated caps on every motor. The bridge's planned moves lower the
    velocity cap to time a move, so this restores it before the swing."""
    for i, m in enumerate(motors):
        for setter, val in (("setVelocity", VMAX[i]), ("setAvailableTorque", TMAX[i]),
                            ("setAvailableForce", TMAX[i])):
            try:
                getattr(m, setter)(val)
            except Exception:
                pass


arm_rated_limits()
emit("[toss] rated caps from URDF: vel=%s rad/s  torque=%s N.m"
     % ([round(v, 4) for v in VMAX], [round(t, 1) for t in TMAX]))

PART = robot.getFromDef("PART")


def tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


# --- VACUUM HOLD: FORCE-COUPLED, never a per-tick pose write -------------------------
# A per-tick Supervisor setSFVec3f on a dynamic Newton body calls eval_fk over the whole
# model and flags _mjc_dirty, forcing a MuJoCo<->Newton resync EVERY tick that wipes the
# arm's accumulated joint velocity (measured: the shoulder could not pass ~1.2 rad/s of the
# 3.04 it needs, even at full rated torque -- the cube went backwards). addForce() instead
# only accumulates into the backend's external-wrench buffer: no eval_fk, no resync.
# So the cube is CARRIED BY A FORCE, exactly as a real vacuum cup carries it.
# The hold law is INVERSE-DYNAMICS + a soft PD trim:
#     F = m*(a_cup + g)          <- feed-forward: the force a real cup must exert to make the
#                                   cube follow the cup's own acceleration, incl. holding it up
#       + Kp*(x_cup - x) + Kd*(v_cup - v)   <- trim: only mops up the small residual
# The feed-forward carries the whole whip, so the PD gains stay LOW and the hold is
# numerically stable at the 16 ms control tick. (A stiff pure-PD hold -- Kp=1500 on a 0.2 kg
# cube => omega ~ 87 rad/s vs a 16 ms tick -- goes unstable and NaNs the world; measured.)
PART_MASS = 0.2                 # kg, matches the PART Physics node in the world
HOLD_KP = float(os.environ.get("OMNIARM6_TOSS_HOLD_KP", "300.0"))   # N/m  (omega ~ 39 rad/s)
HOLD_KD = float(os.environ.get("OMNIARM6_TOSS_HOLD_KD", "16.0"))    # N.s/m (~critically damped)
HOLD_FMAX = float(os.environ.get("OMNIARM6_TOSS_HOLD_FMAX", "60.0"))  # N, safety clamp
GRAV = 9.81

_suck = None
_tcp_prev = None      # previous cup-target position -> cup velocity
_tcpv_prev = None     # previous cup velocity        -> cup acceleration


def suck_on(node):
    global _suck, _tcp_prev, _tcpv_prev
    _suck = node
    _tcp_prev = None            # restart the FD history so the first tick sees zero cup motion
    _tcpv_prev = None


def _cup_target():
    """World point the cube's centre should ride at: a half-cube below the cup lip."""
    pos, _R = tcp_pose()
    return [pos[0], pos[1], pos[2] - CUP_HOLD_DZ]


def _cup_kinematics(target):
    """Finite-difference the cup target -> (velocity, acceleration). These are the hold's
    velocity reference and its feed-forward term."""
    global _tcp_prev, _tcpv_prev
    v = [0.0, 0.0, 0.0] if _tcp_prev is None else \
        [(target[i] - _tcp_prev[i]) / dt_s for i in range(3)]
    a = [0.0, 0.0, 0.0] if _tcpv_prev is None else \
        [(v[i] - _tcpv_prev[i]) / dt_s for i in range(3)]
    _tcp_prev = list(target)
    _tcpv_prev = list(v)
    return v, a


def suck_apply(_unused=None):
    """One tick of vacuum. The cube stays a fully dynamic body: it is never teleported and
    never velocity-set -- only a force is applied, so it accumulates its own true momentum."""
    if _suck is None:
        return
    node = _suck
    tgt = _cup_target()
    tv, ta = _cup_kinematics(tgt)
    p = node.getPosition()
    try:
        v = list(node.getVelocity()[:3])
    except Exception:
        v = [0.0, 0.0, 0.0]
    if not all(math.isfinite(x) for x in list(p) + list(v)):
        return
    f = [PART_MASS * (ta[i] + (GRAV if i == 2 else 0.0))
         + HOLD_KP * (tgt[i] - p[i])
         + HOLD_KD * (tv[i] - v[i]) for i in range(3)]
    mag = math.sqrt(sum(x * x for x in f))
    if mag > HOLD_FMAX:                       # never let a numerical spike launch the cube
        f = [x * HOLD_FMAX / mag for x in f]
    try:
        node.addForce(f, False)               # world frame; re-applied every tick
    except Exception:
        pass


def suck_off():
    global _suck
    _suck = None


def release_throw(node, vel):
    """VACUUM OFF. Nothing is written to the cube: it has been carried by a real force, so it
    already CARRIES its own physical velocity. Stopping the force is the whole release -- the
    flight that follows is pure Newton ballistics on a state we never touched."""
    global _suck
    _suck = None


def step_for(secs, held=True):
    """Advance the sim `secs`, ticking the bridge motion plan and (if holding) the vacuum."""
    for _ in range(max(1, int(secs * 1000 / dt))):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if held:
            suck_apply()
    return True


def settle(secs):
    for _ in range(max(1, int(secs * 1000 / dt))):
        if robot.step(dt) == -1:
            return False
    return True


def interp_q(tau):
    """Ghost joint targets at swing time tau (s)."""
    f = tau / GDT
    k = int(f)
    if k >= N_GHOST - 1:
        return GQ[-1]
    a = f - k
    return GQ[k] * (1 - a) + GQ[k + 1] * a


def main():
    settle(0.3)

    # 1. PICK the cube with the vacuum cup (top-down)
    emit("[toss] pick: stage above cube")
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], 0.32), tcp_offset_z=OZ, duration_s=1.4)
    step_for(1.6, held=False)
    emit("[toss] pick: descend onto cube top")
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], PICK_TOP_Z + 0.01), tcp_offset_z=OZ, duration_s=1.2)
    step_for(1.4, held=False)
    suck_on(PART)
    emit("[toss] vacuum ON")
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], 0.35), tcp_offset_z=OZ, duration_s=1.0)
    step_for(1.2, held=True)

    # 2. GO TO the ghost's windup pose (joint-space planned move)
    emit("[toss] move to windup pose")
    bridge.act_set_joint_positions(list(GQ[0]), duration_s=2.4)
    step_for(2.8, held=True)
    emit(f"[toss] windup target = {[round(float(x),2) for x in GQ[0]]}")
    emit(f"[toss] windup actual = {[round(x,2) for x in bridge._read_q()]}")

    # 3. THE SWING -- direct per-tick ghost tracking; release at the planned instant.
    # The planned move above lowered each motor's velocity cap to TIME that move, so restore
    # the RATED cap (from the URDF, via getMaxVelocity) before the swing. We restore it to the
    # rated value -- not above it. The certified ghost peaks at 97% of rated, so it fits.
    arm_rated_limits()
    emit("[toss] SWING")
    released = False
    tau = 0.0
    _trace = os.environ.get("OMNIARM6_TOSS_TRACE", "0") != "0"
    _q_prev = None
    _proj_prev = [None]        # previous tick's ballistic projection (closest-tick release)
    # run the swing + a couple extra seconds of flight observation
    n_steps = int((T_SWING + 2.5) * 1000 / dt)
    for _ in range(n_steps):
        if robot.step(dt) == -1:
            break
        # Position control, but LEAD the reference by a fixed time (feedforward). The Newton
        # PD's tracking lag is ~proportional to the target velocity, so an angle-lead of
        # rate*LEAD (= what a fixed time-lead gives) compensates it across the whole swing.
        if tau <= T_SWING:
            qd = interp_q(min(tau + LEAD, T_SWING))
            for i, m in enumerate(motors):
                m.setPosition(float(qd[i]))
        pos, _R = tcp_pose()
        if _trace and tau <= T_SWING:
            _qa = bridge._read_q()
            _qref = interp_q(min(tau, T_SWING))
            _qd = [0.0] * 6 if _q_prev is None else [(_qa[i] - _q_prev[i]) / dt_s for i in range(6)]
            _q_prev = list(_qa)
            emit("[trace] t=%.3f qref=%s q=%s qd=%s err=%s" % (
                tau,
                [round(float(x), 3) for x in _qref],
                [round(float(x), 3) for x in _qa],
                [round(float(x), 2) for x in _qd],
                [round(float(_qref[i] - _qa[i]), 3) for i in range(6)]))
        if not released:
            # ADAPTIVE release, AIMED BY THE PAYLOAD ITSELF. The cube is carried by a real
            # force, so its OWN measured position + velocity are what decide where it lands --
            # read them straight off the body and release at the instant its ballistic
            # projection reaches the bin. This aims by where the arm IS actually throwing, not
            # where the ghost planned, so it is robust to any residual tracking error.
            cur_pos = PART.getPosition()
            try:
                cur_vel = list(PART.getVelocity()[:3])
            except Exception:
                cur_vel = [0.0, 0.0, 0.0]
            proj = ballistic_land_x(cur_pos, cur_vel)
            usable = cur_vel[0] > 0.3 and cur_pos[2] > 0.5 and proj is not None
            # CLOSEST-TICK release. We can only let go on a tick boundary, and at ~2.8 m/s the
            # cube covers ~4.5 cm per 16 ms tick -- so "release on the first tick past the bin"
            # systematically overshoots by up to a tick. Instead, extrapolate the projection one
            # tick ahead and let go on whichever tick lands NEARER the bin centre.
            do_release = False
            if usable:
                nxt = proj + (proj - _proj_prev[0]) if _proj_prev[0] is not None else None
                if nxt is None:
                    do_release = proj >= AIM_X
                else:
                    do_release = abs(proj - AIM_X) <= abs(nxt - AIM_X)
                _proj_prev[0] = proj
            do_release = do_release or (tau >= T_SWING)
            if do_release:
                release_throw(PART, cur_vel)    # vacuum OFF -- the cube keeps its own velocity
                spd = math.sqrt(sum(v * v for v in cur_vel))
                ang = math.degrees(math.atan2(cur_vel[2], cur_vel[0]))
                emit(f"[toss] RELEASE @ tau={tau:.2f}s  cube=({cur_pos[0]:.3f},{cur_pos[2]:.3f})  "
                     f"|v|={spd:.2f} m/s  angle={ang:+.1f} deg  proj_x={proj if proj else 0.0:.2f}")
                released = True
            else:
                suck_apply()
        tau += dt_s

    # 4. report where the cube ended up
    settle(0.4)
    fp = PART.getPosition()
    err = abs(fp[0] - BIN_X)
    in_bin = (abs(fp[0] - BIN_X) < 0.20 and abs(fp[1]) < 0.20 and fp[2] < 0.20)
    emit(f"[toss] cube final pos = ({fp[0]:.3f}, {fp[1]:.3f}, {fp[2]:.3f})")
    emit(f"[toss] landed {err*100:.1f} cm from bin centre (x={BIN_X:.2f})  "
         f"-> {'IN BIN' if in_bin else 'MISS'}")
    emit(f"[toss] RESULT: {'PASS' if in_bin else 'FAIL'}")

    if AUTOQUIT:
        fh.close()
        robot.simulationQuit(0)
    else:
        while robot.step(dt) != -1:
            pass


main()
