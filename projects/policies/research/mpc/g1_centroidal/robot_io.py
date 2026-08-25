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

"""RobotIO abstraction (the Newton-MPC-stack plan, layer C).

The controller stack depends ONLY on these dataclasses, never on Newton or Unitree
objects directly. NewtonBackend (below) fills CentroidalState from OmniSim's in-engine
mujoco_warp model; a future UnitreeBackend would fill it from SDK2 LowState. This is the
boundary that lets the same MPC+WBC run in sim and (later) on hardware.

Scope here: the SIMULATION backend, reading the live engine state the in-engine module
already has (world.solver.mj_model / mjw_data). Pure data + a thin reader.
"""
import math

import numpy as np


class CentroidalState:
    """Body-level state the centroidal MPC reasons about (the SRBM variables)."""
    __slots__ = ("t", "com", "com_vel", "L", "quat", "omega", "feet", "contact",
                 "mass", "g", "qpos", "qvel")

    def __init__(self):
        self.t = 0.0
        self.com = np.zeros(3)          # CoM position, world
        self.com_vel = np.zeros(3)      # CoM linear velocity, world
        self.L = np.zeros(3)            # angular momentum about the CoM, world
        self.quat = np.array([1.0, 0, 0, 0])  # base orientation (w,x,y,z)
        self.omega = np.zeros(3)        # base angular velocity, world
        self.feet = {}                  # leg -> foot position (world, 3)
        self.contact = {}               # leg -> bool (loaded)
        self.mass = 0.0
        self.g = np.array([0.0, 0.0, -9.81])
        self.qpos = None                # full mjc qpos (for the WBC)
        self.qvel = None

    def roll_pitch_yaw(self):
        w, x, y, z = self.quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
