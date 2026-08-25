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

"""Driver for ``r1_probe.xml`` -- a **deliberately blind** run.

It reads the forward rangefinder and prints it, and then ignores it: the wheels
are held at a constant equal speed, so the rover heads straight at the goal
from the declared start pose and drives into ``OBSTACLE_1``, which is exactly
what the straight START->GOAL line is blocked by.

That is the point. This is an instrument probe, not an R1 solution. A
"collision-free" verdict that could not have failed is worth nothing, so the
arm has to be able to show a robot-vs-obstacle contact actually being named
before "nothing was hit" means anything on it. A passing R1 run needs a
navigator that consumes the sensor; writing one belongs to the oracle/null gate
(SPEC 7.1), not here.
"""

import sys

import mujoco

WHEEL_SPEED = 9.0           # rad/s, both wheels: straight ahead, no steering


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "r1_probe.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    data.ctrl[:] = WHEEL_SPEED

    rf = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_forward")
    n = 0
    while True:
        mujoco.mj_step(model, data)
        n += 1
        if rf >= 0 and n % 500 == 0:
            # Read, report -- and do nothing with it.
            print("t=%.2f forward range=%.3f m"
                  % (data.time, data.sensordata[rf]))


if __name__ == "__main__":
    main()
