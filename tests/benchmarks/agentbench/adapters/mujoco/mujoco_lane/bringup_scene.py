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

"""Bring-up driver for ``bringup_scene.xml`` -- an ordinary MuJoCo program.

This is what a MuJoCo deliverable looks like and it is written the way an
author would write it, not the way the grader would prefer: it takes the model
path from ``sys.argv[1]``, compiles it, sets a couple of actuator commands and
runs its own ``mj_step`` loop. Nothing in it imports, knows about or cooperates
with AgentBench.

It loops **without an end condition on purpose**, so the bring-up exercises the
path that matters on this arm: MuJoCo has no ``--duration`` and no auto-exit,
so the grader's recording window -- not the driver -- has to be able to stop a
run, and it does that by raising out of ``mj_step``. A driver with its own
``while t < T`` is the easy case; this is the one worth proving.
"""

import sys

import mujoco

WHEEL_LEFT = 9.0            # rad/s
WHEEL_RIGHT = 7.0           # ...different, so the rover arcs rather than
#                             driving in a straight line and never turning


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "bringup_scene.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                             "wheel_left_motor")
    right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              "wheel_right_motor")
    data.ctrl[left] = WHEEL_LEFT
    data.ctrl[right] = WHEEL_RIGHT

    while True:
        mujoco.mj_step(model, data)


if __name__ == "__main__":
    main()
