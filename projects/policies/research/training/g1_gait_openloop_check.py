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

"""Reference check: does G1's open-loop human-gait survive in plain
mujoco on its kp100 dump model? Calibrates expectations for the humanoid
walk pilot — if the G1 reference also falls open-loop, the RL residual is
what carries it and the pilot needs recipe iterations, not a 'fixed'
reference."""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

from projects.policies.control.gait import g1_human_gait as ghg

MJCF = str(REPO / "projects/robots/unitree/g1/urdf/g1_legs_kp100.mjcf.xml")
DT = 0.016
PHYS_DT = 0.004

JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
]

m = mujoco.MjModel.from_xml_path(MJCF)
m.opt.timestep = PHYS_DT
d = mujoco.MjData(m)
gp = ghg.GaitParams()
stand = ghg.standing_pose(gp)
qadr = {}
aadr = {}
for i, jn in enumerate(JOINTS):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    qadr[jn] = m.jnt_qposadr[jid]
    aadr[jn] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
d.qpos[0:3] = [0.0, 0.0, 0.76]
d.qpos[3:7] = [1, 0, 0, 0]
for i, jn in enumerate(JOINTS):
    d.qpos[qadr[jn]] = stand[i]
    d.ctrl[aadr[jn]] = stand[i]
mujoco.mj_forward(m, d)
for _ in range(int(0.5 / PHYS_DT)):
    mujoco.mj_step(m, d)

phase = ghg.DS_PHASE
t = 0.0
x0 = float(d.qpos[0])
fell = None
for step in range(int(10.0 / DT)):
    legs, _, _ = ghg.targets_np(phase, gp, t_since_start=t)
    for i, jn in enumerate(JOINTS):
        d.ctrl[aadr[jn]] = legs[i]
    for _ in range(int(DT / PHYS_DT)):
        mujoco.mj_step(m, d)
    phase += 2.0 * math.pi * gp.freq * DT
    t += DT
    if d.qpos[2] < 0.45 and fell is None:
        fell = t
        break
    if step % 31 == 30:
        q = d.qpos[3:7]
        roll = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]),
                          1 - 2 * (q[1] ** 2 + q[2] ** 2))
        pitch = math.asin(max(-1, min(1, 2 * (q[0] * q[2] - q[3] * q[1]))))
        print(f"t={t:5.2f} x={d.qpos[0]-x0:+.3f} z={d.qpos[2]:.3f} "
              f"roll={roll:+.3f} pitch={pitch:+.3f} vx={d.qvel[0]:+.2f}")
dist = float(d.qpos[0]) - x0
print(f"G1 open-loop: dist={dist:+.2f} m  "
      f"{'OK' if fell is None else f'FELL at {fell:.2f}s'}")
