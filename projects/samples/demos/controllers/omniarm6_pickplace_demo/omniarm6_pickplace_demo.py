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

"""Self-contained physics pick & place for the OMNIARM6 + Robotiq 2F-85.

Single controller (no HTTP, no second controller), so it runs headless
and deterministically. It reuses the omnilink_arm_bridge IK + gripper:
instantiates an ArmBridge (without its HTTP server), drives top-down
6-DOF poses, closes the real finger motors to grip the cube by friction
(no kinematic weld), lifts, moves, and places it.

Verifies numerically: reads the cube's world pose and asserts it was
lifted off the floor and placed at the target. Writes PASS/FAIL to
PICKPLACE_OUT (default _pickplace_result.txt).
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import ArmBridge  # noqa: E402
from _arm_configs import get_config  # noqa: E402

OZ = 0.25                      # tool point = finger throat (flange + ~0.085)
PICK = [0.46, 0.0, 0.025]
PLACE = [0.30, 0.34, 0.025]
CUBE = os.environ.get("CUBE_DEF", "GRASP_CUBE_A")
OUT = os.environ.get("PICKPLACE_OUT", "_pickplace_result.txt")

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6",
                   gripper_id="robotiq_2f85_phys")
cube = robot.getFromDef(CUBE)
fh = open(OUT, "w", encoding="utf-8", buffering=1)
t0 = time.time()


def emit(s):
    fh.write(s + "\n")
    print(s, flush=True)


def step_for(secs):
    """Advance the sim, driving the bridge motion each tick."""
    n = int(secs * 1000 / dt)
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


def cz():
    p = cube.getPosition()
    return [round(p[0], 3), round(p[1], 3), round(p[2], 3)]


def pose(xyz, dur=1.5, settle=0.4):
    r = bridge.act_set_tcp_pose(tuple(xyz), tcp_offset_z=OZ, duration_s=dur)
    step_for(dur + settle)
    return r


# Let the arm settle at home.
step_for(1.0)
emit(f"[demo] start    cube={cz()}")

bridge.act_open_gripper()
step_for(0.8)
pose([PICK[0], PICK[1], 0.20])           # above the cube
emit(f"[demo] above    cube={cz()}")
pose(PICK)                               # lower so fingers straddle it
emit(f"[demo] lowered  cube={cz()}")
bridge.act_grasp()                       # close fingers — physics grip
step_for(1.2)
emit(f"[demo] grasped  cube={cz()}")
pose([PICK[0], PICK[1], 0.28])           # lift straight up
lifted = cz()
emit(f"[demo] lifted   cube={lifted}")
pose([PLACE[0], PLACE[1], 0.28], dur=1.8)  # carry over the target
emit(f"[demo] moved    cube={cz()}")
pose([PLACE[0], PLACE[1], 0.06])         # lower to place
bridge.act_open_gripper()                # release
step_for(2.0)
final = cz()
emit(f"[demo] placed   cube={final}")

place_dist = ((final[0] - PLACE[0]) ** 2 + (final[1] - PLACE[1]) ** 2) ** 0.5
lifted_ok = lifted[2] > 0.12
placed_ok = place_dist < 0.10 and final[2] < 0.07
emit(f"[demo] RESULT lifted_z={lifted[2]:.3f} lifted_ok={lifted_ok} | "
     f"place_dist={place_dist*1000:.0f}mm final_z={final[2]:.3f} placed_ok={placed_ok} | "
     f"wall={time.time()-t0:.1f}s sim={robot.getTime():.1f}s")
emit("PASS" if (lifted_ok and placed_ok) else "FAIL")

# Headless verification sets PICKPLACE_AUTOQUIT=1 so the process exits with
# a status. Run interactively (no env var) and we instead idle here, holding
# the final pose, so the window stays open instead of slamming shut.
if os.environ.get("PICKPLACE_AUTOQUIT"):
    try:
        robot.simulationQuit(0 if (lifted_ok and placed_ok) else 1)
    except Exception:
        pass
else:
    while robot.step(dt) != -1:
        bridge.tick(robot.getTime())
