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

"""Kinematic-body probe for tests/test_newton_kinematic_native.py.

Two scenarios, selected by OMNISIM_KIN_SCENARIO:

  push  -- runs on a bodiless PROBE supervisor. A physics-less SLAB box
           (boundingObject only) is teleported horizontally in small per-tick
           increments (a supervisor-driven conveyor) so its front face plows
           into a dynamic BALL resting on the floor. Records the ball pose
           before/after the sweep and the minimum ball z seen (tunnelling
           watch). The poses ARE the verdict; assertions live in the test.

  rest  -- runs on the RIG robot whose HingeJoint's physics-less endPoint is
           the SLAB. A dynamic BALL is dropped onto the stationary slab.
           Records the ball's final z and the minimum z seen after the
           impact window (rest-height / tunnelling verdict).

Output: JSON at OMNISIM_KIN_PROBE_OUT.
"""
import json
import os
import sys

from omnisim import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
scenario = os.environ.get("OMNISIM_KIN_SCENARIO", "push")

ball = robot.getFromDef("BALL")
out = {"scenario": scenario, "min_ball_z": float("inf")}


def ball_pos():
    p = ball.getPosition()
    return [float(p[0]), float(p[1]), float(p[2])]


def advance(ms):
    """Step ~ms of sim time, tracking the ball's minimum z the whole way."""
    n = max(1, int(round(ms / dt)))
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
        z = ball_pos()[2]
        if z < out["min_ball_z"]:
            out["min_ball_z"] = z
    return True


if scenario == "push":
    slab = robot.getFromDef("SLAB")
    tr = slab.getField("translation")
    advance(400)                        # settle: ball at rest on the floor
    out["p_ball_start"] = ball_pos()
    sp = slab.getPosition()
    out["p_slab_start"] = [float(sp[0]), float(sp[1]), float(sp[2])]
    # Teleport the slab +x in 0.02 m per-tick increments (2.5 m/s effective at
    # basicTimeStep 8) -- the conveyor/kinematic-prop drive pattern. One giant
    # jump would engulf the ball in a single step (undefined ejection); the
    # incremental sweep is the semantics a kinematic mover actually has.
    x = out["p_slab_start"][0]
    for _ in range(60):                 # 1.2 m total travel
        x += 0.02
        tr.setSFVec3f([x, 0.0, 0.2])
        if robot.step(dt) == -1:
            break
        z = ball_pos()[2]
        if z < out["min_ball_z"]:
            out["min_ball_z"] = z
    advance(400)                        # let the pushed ball settle
    out["p_ball_end"] = ball_pos()
    sp = slab.getPosition()
    out["p_slab_end"] = [float(sp[0]), float(sp[1]), float(sp[2])]
else:                                   # rest
    advance(2500)                       # drop + impact + settle
    out["p_ball_end"] = ball_pos()

path = os.environ.get("OMNISIM_KIN_PROBE_OUT", "kinematic_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("kinematic_native_probe: wrote %s\n" % path)
sys.stdout.flush()
