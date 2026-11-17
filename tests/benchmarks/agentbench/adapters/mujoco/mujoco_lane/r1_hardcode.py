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

"""R1 negative control -- the HARDCODED path. Plausible, and blind.

The third fixture of the gate, and the one that says where the gate's edge is.
It drives the same robot, on the same scene, with the same control law and the
same planner as ``r1_oracle.py`` -- and differs in exactly one thing: where
the map comes from. The oracle's map starts EMPTY and is filled by beams. This
one is filled once, before the wheels turn, by reading
``benchmark_assets/obstacles.json``. Not one ray is cast for the whole run.

Isolating perception as the only variable is the point. Two facts follow, and
they matter in opposite directions:

1. **On the published layout it PASSES all six assertions.** R1's behavioural
   evidence -- arrived, collision-free, drove a real 11.5 m+ path -- is
   satisfied by a robot that perceived nothing, because the layout it memorised
   is the layout it was graded on. This is not a defect discovered here: it is
   what ``meta.json``'s ``anti_hardcode`` and ``status`` already declare, and
   this fixture is the demonstration under those words. R1 on its own does not
   prove perception; it proves that a *blind reactive* robot cannot pass (the
   probe) and that a robot with a correct map can.
2. **On a layout it has not seen it FAILS**, by driving into a box that moved
   -- while the oracle, on the same moved layout, still arrives. That is
   GRADE-TIME PLACEMENT (``r1_core.sample_layout`` drawing the layout,
   ``common/r1_placement.py`` placing it), measured rather than argued.

So the honest reading of a green R1 cell depends entirely on whether its row
says the layout was placed. Unplaced, it means "the agent got a robot to the
goal around the obstacles without touching anything"; placed, it means the
robot did that on a layout nobody published. Since 2026-08-10 every cell
places, and a cell that cannot is blocked rather than graded -- so this
fixture's PASS above is reproducible only by grading a run with no placement
at all, which is exactly what makes it the negative control.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import r1_oracle as R                                        # noqa: E402

#: The frozen asset the task hands the agent -- read here, sensed in the oracle.
OBSTACLES_JSON = (Path(__file__).resolve().parents[3] / "tasks"
                  / "R1_lidar_nav" / "initial" / "benchmark_assets"
                  / "obstacles.json")


def map_from_the_published_file(path=OBSTACLES_JSON):
    """An occupancy grid built from the asset file. No sensor is involved."""
    spec = json.loads(Path(path).read_text(encoding="utf-8"))["obstacles"]
    occ = np.zeros((R._N, R._N), dtype=bool)
    for o in spec:
        cx, cy = o["position"][0], o["position"][1]
        sx, sy = o["size"][0], o["size"][1]
        for ix in range(R._N):
            for iy in range(R._N):
                px, py = R._centre(ix, iy)
                if abs(px - cx) <= sx / 2 and abs(py - cy) <= sy / 2:
                    occ[ix, iy] = True
    return occ


def plan_once(occ):
    """One plan, made before the run and never revised."""
    for radius in R.INFLATE_FALLBACK_M:
        blocked = R.inflate(occ, radius)
        start = R._cell(*R.START_XY)
        if blocked[start]:
            blocked[start] = False
        found = R.astar(blocked, start, R._cell(*R.GOAL_XY))
        if found:
            path = R.shorten(blocked, found)
            path[-1] = R.GOAL_XY
            return path
    return [R.START_XY, R.GOAL_XY]


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "r1_oracle.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rover")

    path = plan_once(map_from_the_published_file())
    print("[hardcode] %d waypoints planned from obstacles.json before the "
          "first step; no beam will be cast" % len(path))

    every = max(1, int(round(1.0 / (R.CONTROL_HZ * model.opt.timestep))))
    plan_i, step = 1, 0
    while data.time < R.GIVE_UP_S:
        mujoco.mj_step(model, data)
        step += 1
        if data.time < 0.05 or step % every:
            continue
        x, y = float(data.xpos[bid][0]), float(data.xpos[bid][1])
        yaw = R._yaw(data, bid)
        d_goal = math.hypot(R.GOAL_XY[0] - x, R.GOAL_XY[1] - y)
        if d_goal <= R.GOAL_STOP_M:
            break
        while (plan_i < len(path) - 1
               and math.hypot(path[plan_i][0] - x,
                              path[plan_i][1] - y) < R.LOOKAHEAD_M):
            plan_i += 1
        tx, ty = path[min(plan_i, len(path) - 1)]
        err = R._wrap(math.atan2(ty - y, tx - x) - yaw)
        w = max(-R.W_MAX_RPS, min(R.W_MAX_RPS, R.K_HEADING * err))
        v = 0.0 if abs(err) > R.TURN_IN_PLACE_RAD else min(
            R.V_MAX_MPS * math.cos(err), 0.9 * d_goal + 0.05)
        data.ctrl[0] = (v - w * R.WHEEL_BASE_M / 2.0) / R.WHEEL_RADIUS_M
        data.ctrl[1] = (v + w * R.WHEEL_BASE_M / 2.0) / R.WHEEL_RADIUS_M

    data.ctrl[:] = 0.0
    t_stop = data.time
    while data.time - t_stop < 0.6:
        mujoco.mj_step(model, data)
    x, y = float(data.xpos[bid][0]), float(data.xpos[bid][1])
    print("[hardcode] final xy=(%.3f, %.3f), %.3f m from the goal after %.1f s"
          % (x, y, math.hypot(R.GOAL_XY[0] - x, R.GOAL_XY[1] - y), data.time))


if __name__ == "__main__":
    main()
