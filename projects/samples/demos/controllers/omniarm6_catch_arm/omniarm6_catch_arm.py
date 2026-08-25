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

"""Arm B (CATCHER) for the two-arm throw & catch demo.

B faces arm A and catches the thrown cube IN FLIGHT. It pre-positions its suction cup at the
interception point, watches the cube (live world pose/velocity), arms once the cube is airborne,
and catches the instant the cube crosses the cup's plane at the right height -- killing the cube's
momentum (the new free-body setVelocity engine fix) and welding it to the cup. Then it lifts the
caught cube to show the catch.

The catch is a genuine feedback step: the trigger is driven by the OBSERVED cube, not a script.
Geometry is env-tunable (OMNIARM6_TC_CATCH_X / _Z) for iteration.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

import math  # noqa: E402
import numpy as np  # noqa: E402
from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (ArmBridge, warmup_reload,  # noqa: E402
                                 forward_kinematics_pose, _mat3_to_axis_angle)
from _arm_configs import get_config  # noqa: E402

OZ = 0.25
CUP_HOLD_DZ = 0.0          # caught cube rides flush at the cup point
# Arm B world placement (must match omniarm6_throw_catch.omniworld): translation + 180deg about z.
B_X = 1.35
# Interception point (WORLD): where the thrown cube passes through B's reach.
CATCH_X = float(os.environ.get("OMNIARM6_TC_CATCH_X", "0.85"))
CATCH_Z = float(os.environ.get("OMNIARM6_TC_CATCH_Z", "0.72"))
ENGAGE_Z_TOL = 0.13       # height window for the catch trigger
ARM_SPEED = 1.5           # cube speed (m/s) above which we consider it "thrown/airborne"

robot = Supervisor()
warmup_reload(robot)
dt = int(robot.getBasicTimeStep())
dt_s = dt / 1000.0
_fh = open(os.environ.get("OMNIARM6_TC_CATCH_OUT", "_catch_result.txt"), "w",
           encoding="utf-8", buffering=1)


def emit(m, **_):
    print(m, flush=True)
    _fh.write(m + "\n")
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
IK = bridge.cfg["ik"]
PART = robot.getFromDef("PART")
_suck = None


def w2l(w):
    """World -> arm-B local (B at (B_X,0,0) rotated 180deg about z; involutive)."""
    return (B_X - w[0], -w[1], w[2])


def l2w(l):
    return (B_X - l[0], -l[1], l[2])


def tcp_local():
    return forward_kinematics_pose(IK["chain"], bridge._read_q(), (0.0, 0.0, OZ))[0]


def tcp_world():
    return l2w(tcp_local())


def suck_on(node):
    global _suck
    o = node.getOrientation()
    rot0 = _mat3_to_axis_angle([[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]])
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass
    _suck = (node, node.getField("translation"), node.getField("rotation"), rot0)


def suck_apply():
    if _suck is None:
        return
    node, tr, rot, rot0 = _suck
    pw = tcp_world()
    tr.setSFVec3f([pw[0], pw[1], pw[2] - CUP_HOLD_DZ])
    rot.setSFRotation(rot0)
    try:
        node.setVelocity([0.0] * 6)
    except Exception:
        pass


def step_for(secs, held=False):
    for _ in range(max(1, int(secs * 1000 / dt))):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
        if held:
            suck_apply()
    return True


def main():
    if robot.step(dt) == -1:
        return
    # 1. Pre-position the cup at the interception point (B-local), facing up-ish (top-down pose).
    cl = w2l((CATCH_X, 0.0, CATCH_Z))
    emit(f"[catch] pre-position cup at world=({CATCH_X:.2f},0,{CATCH_Z:.2f}) local=({cl[0]:.2f},{cl[1]:.2f},{cl[2]:.2f})")
    bridge.act_set_tcp_pose((cl[0], cl[1], cl[2]), tcp_offset_z=OZ, duration_s=2.2)
    step_for(2.6)
    cw0 = tcp_world()
    emit(f"[catch] cup settled at world=({cw0[0]:.2f},{cw0[1]:.2f},{cw0[2]:.2f})")

    # 2. Watch the cube; arm once it's airborne; catch on plane-crossing at the right height.
    armed = False
    caught = False
    while robot.step(dt) != -1:
        bridge.tick(robot.getTime())
        cp = PART.getPosition()
        cv = PART.getVelocity()
        speed = math.sqrt(cv[0] ** 2 + cv[1] ** 2 + cv[2] ** 2)
        if not caught:
            if not armed and speed > ARM_SPEED and cp[0] > 0.0 and cv[0] > 1.0:
                armed = True
                emit(f"[catch] ARMED -- cube airborne ({speed:.1f} m/s) at ({cp[0]:.2f},{cp[2]:.2f})", flush=True)
            if armed:
                cw = tcp_world()
                # trigger: cube has reached the cup's x-plane at ~the cup height
                if cp[0] >= cw[0] - 0.03 and abs(cp[2] - cw[2]) < ENGAGE_Z_TOL and abs(cp[1]) < 0.18:
                    suck_on(PART)        # zero the cube's velocity + weld to the cup = CAUGHT
                    suck_apply()
                    caught = True
                    emit(f"[catch] *** CAUGHT *** at cube=({cp[0]:.2f},{cp[1]:.2f},{cp[2]:.2f}) "
                          f"cup=({cw[0]:.2f},{cw[2]:.2f})", flush=True)
                elif cp[0] > cw[0] + 0.25 or cp[2] < 0.15:
                    emit(f"[catch] MISS -- cube past the cup at ({cp[0]:.2f},{cp[2]:.2f})", flush=True)
                    break
        else:
            suck_apply()
        if caught:
            break

    if not caught:
        emit("[catch] RESULT: MISS", flush=True)
        while robot.step(dt) != -1:
            pass
        return

    # 3. Lift the caught cube to show the catch.
    emit("[catch] lift")
    lift_l = w2l((CATCH_X + 0.2, 0.0, CATCH_Z + 0.25))
    bridge.act_set_tcp_pose((lift_l[0], lift_l[1], lift_l[2]), tcp_offset_z=OZ, duration_s=1.4)
    step_for(1.8, held=True)
    cp = PART.getPosition()
    cw = tcp_world()
    held = math.dist(cp, cw) < 0.12
    emit(f"[catch] cube=({cp[0]:.2f},{cp[1]:.2f},{cp[2]:.2f}) cup=({cw[0]:.2f},{cw[1]:.2f},{cw[2]:.2f})")
    emit(f"[catch] RESULT: {'CAUGHT + HELD' if held else 'caught but dropped'}")

    while robot.step(dt) != -1:
        suck_apply()


main()
