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

"""Arm A (THROWER) for the two-arm throw & catch demo.

Picks the cube and executes the certified toss ghost, releasing toward arm B's catch zone. This
is the working OMNIARM6 toss deploy (see omniarm6_toss_deploy.py / omniarm6-toss-shadowing.md), minus the
bin -- the cube is meant to be caught in flight, not to land. After the throw it idles.

Shares omniarm6_toss_deploy.py's two fixes (2026-07-11), for the same reasons -- read that file's
header for the full story:
  * RATED LIMITS: the velocity + torque caps are read off the motor (getMaxVelocity /
    getMaxTorque = the URDF <limit>), never hard-coded past them. The old setVelocity(8.0) was
    2.5x the OMNIARM6's rated 3.1415 rad/s; the engine clamped it and warned on every joint.
  * FORCE-COUPLED VACUUM: the cube is carried by addForce(), not by a per-tick setSFVec3f
    teleport. That teleport forced a MuJoCo<->Newton resync every tick which wiped the arm's
    accumulated joint velocity -- the shoulder stalled at ~1.2 rad/s of the 3.04 rad/s the ghost
    needs, and the throw came out BACKWARDS (|v|=0.41 m/s, proj_x=-0.76). THAT is why the catch
    never happened. It was masked by OMNISIM_NEWTON_NO_EFFORT_LIMIT=1 (which simply deleted the
    arm's rated torque cap so it could brute-force through the resync losses).
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

OZ = 0.25
CUP_HOLD_DZ = 0.025
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
EFFORT = (194.0, 194.0, 102.0, 102.0, 34.0, 34.0)
PICK_XY = (0.45, 0.0)
PICK_TOP_Z = 0.15
G = 9.81
THROW_PROJ_X = float(os.environ.get("OMNIARM6_TC_THROW_X", "1.30"))   # release when proj reaches this
BIN_RIM_Z = 0.10

_REPO = os.environ.get("OMNISIM_HOME") or os.path.abspath(os.path.join(_HERE, "..", "..", "..", "..", ".."))
GHOST = os.environ.get("OMNIARM6_TOSS_GHOST",
                       os.path.join(_REPO, "projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz"))

robot = Supervisor()
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
dt_s = dt / 1000.0
_fh = open(os.environ.get("OMNIARM6_TC_THROW_OUT", os.path.join(_REPO, "_throw_result.txt")), "w",
           encoding="utf-8", buffering=1)


def emit(m, **_):
    print(m, flush=True)
    _fh.write(m + "\n")
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
IK = bridge.cfg["ik"]

g = np.load(GHOST, allow_pickle=True)
GQ = g["q"][:, :6].astype(float)
GDT = float(g["dt"])
N_GHOST = GQ.shape[0]
T_SWING = (N_GHOST - 1) * GDT

def _rated(motor, getter, fallback):
    """The motor's OWN rated cap, as the URDF importer baked it in from <limit>."""
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
    VMAX.append(_rated(m, "getMaxVelocity", 3.1415))
    TMAX.append(_rated(m, "getMaxTorque", _rated(m, "getMaxForce", EFFORT[i])))
    motors.append(m)


def arm_rated_limits():
    """(Re)assert the RATED caps -- never above them. The bridge's planned moves lower the
    velocity cap to time a move, so this restores it before the swing."""
    for i, m in enumerate(motors):
        for setter, val in (("setVelocity", VMAX[i]), ("setAvailableTorque", TMAX[i]),
                            ("setAvailableForce", TMAX[i])):
            try:
                getattr(m, setter)(val)
            except Exception:
                pass


arm_rated_limits()

PART = robot.getFromDef("PART")
_suck = None


def tcp_pose():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))


# VACUUM HOLD -- force-coupled: inverse-dynamics feed-forward m*(a_cup + g) plus a soft PD trim.
# NEVER a per-tick pose write (that resyncs MuJoCo every tick and stalls the swing -- see the
# module docstring). The gains are deliberately soft; the feed-forward carries the whip, so the
# hold stays numerically stable at the 16 ms control tick.
PART_MASS = 0.2
HOLD_KP = float(os.environ.get("OMNIARM6_TOSS_HOLD_KP", "300.0"))
HOLD_KD = float(os.environ.get("OMNIARM6_TOSS_HOLD_KD", "16.0"))
HOLD_FMAX = float(os.environ.get("OMNIARM6_TOSS_HOLD_FMAX", "60.0"))
_tcp_prev = None
_tcpv_prev = None


def suck_on(node):
    global _suck, _tcp_prev, _tcpv_prev
    _suck = node
    _tcp_prev = None
    _tcpv_prev = None


def _cup_target():
    pos, _R = tcp_pose()
    return [pos[0], pos[1], pos[2] - CUP_HOLD_DZ]


def _cup_kinematics(target):
    global _tcp_prev, _tcpv_prev
    if _tcp_prev is None:
        v = [0.0, 0.0, 0.0]
    else:
        v = [(target[i] - _tcp_prev[i]) / dt_s for i in range(3)]
    if _tcpv_prev is None:
        a = [0.0, 0.0, 0.0]
    else:
        a = [(v[i] - _tcpv_prev[i]) / dt_s for i in range(3)]
    _tcp_prev = list(target)
    _tcpv_prev = list(v)
    return v, a


def suck_apply():
    """One tick of vacuum: a real force, so the cube accumulates its OWN true momentum."""
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
    f = [PART_MASS * (ta[i] + (G if i == 2 else 0.0))
         + HOLD_KP * (tgt[i] - p[i])
         + HOLD_KD * (tv[i] - v[i]) for i in range(3)]
    mag = math.sqrt(sum(x * x for x in f))
    if mag > HOLD_FMAX:
        f = [x * HOLD_FMAX / mag for x in f]
    try:
        node.addForce(f, False)
    except Exception:
        pass


def release_throw(node, vel):
    """VACUUM OFF. The cube was carried by a real force, so it already carries its own physical
    velocity -- stopping the force IS the release. Nothing is written to the body."""
    global _suck
    _suck = None


def step_for(secs, held=True):
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
    f = tau / GDT
    k = int(f)
    if k >= N_GHOST - 1:
        return GQ[-1]
    a = f - k
    return GQ[k] * (1 - a) + GQ[k + 1] * a


def ballistic_land_x(p, v, zb=BIN_RIM_Z):
    z, vz = float(p[2]), float(v[2])
    disc = vz * vz + 2.0 * G * (z - zb)
    if disc < 0 or v[0] <= 0.0:
        return None
    t = (vz + math.sqrt(disc)) / G
    return None if t <= 0 else p[0] + v[0] * t


def main():
    settle(0.3)
    print("[throw] pick")
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], 0.32), tcp_offset_z=OZ, duration_s=1.4)
    step_for(1.6, held=False)
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], PICK_TOP_Z + 0.01), tcp_offset_z=OZ, duration_s=1.2)
    step_for(1.4, held=False)
    suck_on(PART)
    bridge.act_set_tcp_pose((PICK_XY[0], PICK_XY[1], 0.35), tcp_offset_z=OZ, duration_s=1.0)
    step_for(1.2, held=True)

    emit(f"[throw] windup target={[round(float(x),2) for x in GQ[0]]} actual={[round(x,2) for x in bridge._read_q()]}")
    bridge.act_set_joint_positions(list(GQ[0]), duration_s=2.4)
    step_for(2.8, held=True)
    emit(f"[throw] windup reached={[round(x,2) for x in bridge._read_q()]}")

    emit("[throw] SWING")
    arm_rated_limits()          # the planned move lowered the cap to time itself; restore RATED
    released = False
    tau = 0.0
    n_steps = int((T_SWING + 1.5) * 1000 / dt)
    for _ in range(n_steps):
        if robot.step(dt) == -1:
            break
        if tau <= T_SWING:
            qd = interp_q(tau)
            for i, m in enumerate(motors):
                m.setPosition(float(qd[i]))
        if not released:
            # Aim by the PAYLOAD itself: it is carried by a real force, so its own measured
            # position + velocity are what decide where it actually flies.
            cur_pos = PART.getPosition()
            try:
                cur_vel = list(PART.getVelocity()[:3])
            except Exception:
                cur_vel = [0.0, 0.0, 0.0]
            proj = ballistic_land_x(cur_pos, cur_vel)
            usable = cur_vel[0] > 0.3 and cur_pos[2] > 0.5 and proj is not None
            if (usable and proj >= THROW_PROJ_X - 0.03) or (tau >= T_SWING):
                release_throw(PART, cur_vel)
                spd = math.sqrt(sum(v * v for v in cur_vel))
                emit(f"[throw] RELEASE tau={tau:.2f} cube=({cur_pos[0]:.3f},{cur_pos[2]:.3f}) "
                     f"|v|={spd:.2f} vel=({cur_vel[0]:.2f},{cur_vel[1]:.2f},{cur_vel[2]:.2f}) "
                     f"proj_x={proj if proj else 0.0:.2f}")
                released = True
            else:
                suck_apply()        # hold the cube to the cup until release
        tau += dt_s

    while robot.step(dt) != -1:   # idle (the catcher takes over the cube)
        pass


main()
