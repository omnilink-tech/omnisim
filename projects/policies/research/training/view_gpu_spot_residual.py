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

"""Live MuJoCo viewer for the GPU residual Spot walker.

Loads the SAME MJCF + collision patches + gait/IK model layer as the
GPU mjwarp trainer, then steps a single env on the MuJoCo CPU
backend with the ONNX (or .pt) policy in the loop. A Python-only
visualisation so the user can SEE the GPU-trained policy walking
without having to wire an OmniSim controller variant.

Run:
    python projects/policies/research/training/view_gpu_spot_residual.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer
import torch
import torch.nn as nn

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

from projects.policies.control.spot_gait import GaitParams                       # noqa
from projects.policies.control.spot_gait_np import foot_targets_batched          # noqa
from projects.policies.control.spot_kinematics_np import inverse_kinematics_batched  # noqa
from projects.policies.research.training.gpu_mjwarp_residual_trainer import (             # noqa
    NJ, OBS_DIM, NOMINAL, RES_SCALE, DT, BZ_FAIL,
    JOINT_LIMITS_LO, JOINT_LIMITS_HI, quat_to_rp, proj_gravity,
)

MJCF = r"C:\tmp\spot_newton_fixed.xml"
PT = REPO / "projects/policies/research/training/runs/gpu_spot_residual_v3/policy_main.pt"
FIXED_VX = 0.5


class AC(nn.Module):
    def __init__(self):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                nn.Linear(256, 128), nn.Tanh(),
                                nn.Linear(128, NJ))
        self.v = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                               nn.Linear(256, 128), nn.Tanh(),
                               nn.Linear(128, 1))
        self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

    def forward(self, obs):
        return self.pi(obs), self.v(obs).squeeze(-1), self.log_std


def main():
    mjm = mujoco.MjModel.from_xml_path(MJCF)
    # Same collision patch as the trainer.
    for gid in (1, 2, 3, 5, 6, 8, 9, 11, 12):
        mjm.geom_contype[gid] = 0
        mjm.geom_conaffinity[gid] = 0
    mjd = mujoco.MjData(mjm)

    gait = GaitParams(neutral_front_x=0.322, neutral_rear_x=-0.274,
                      neutral_lateral_y=0.344, ground_z=-0.62,
                      step_height=0.04)
    seed_body_z = -gait.ground_z + 0.01

    def reset():
        mjd.qpos[:] = 0.0
        mjd.qpos[0:3] = [0, 0, seed_body_z]
        mjd.qpos[3:7] = [1, 0, 0, 0]
        # Joints in MJCF order = controller order
        mjd.qpos[7:19] = [+0.30, +0.30, -0.60,
                          -0.30, +0.30, -0.60,
                          +0.30, +0.30, -0.60,
                          -0.30, +0.30, -0.60]
        mjd.qvel[:] = 0.0
        mujoco.mj_forward(mjm, mjd)

    reset()

    ac = AC()
    ac.load_state_dict(torch.load(str(PT), map_location="cpu"))
    ac.eval()

    vel_cmd = np.array([FIXED_VX, 0.0, 0.0], dtype=np.float32)
    t_clock = 0.0
    last_action = np.zeros(NJ, dtype=np.float32)
    nominal_4x3 = NOMINAL.reshape(4, 3)

    def build_obs():
        vlin = mjd.qvel[0:3].astype(np.float32)
        vang = mjd.qvel[3:6].astype(np.float32)
        pg = proj_gravity(mjd.qpos[3:7][None, :].astype(np.float32))[0]
        q = mjd.qpos[7:19].astype(np.float32)
        qd = mjd.qvel[6:18].astype(np.float32)
        clock = np.float32((t_clock % gait.period_s) / gait.period_s)
        obs = np.concatenate([vlin, vang, pg, q, qd, last_action,
                              vel_cmd, [clock]]).astype(np.float32)
        return np.clip(np.nan_to_num(obs, nan=0.0, posinf=10, neginf=-10),
                       -10, 10)

    def policy_step():
        nonlocal t_clock, last_action
        t_clock += DT
        obs = build_obs()[None, :]
        with torch.no_grad():
            mu, _, _ = ac(torch.from_numpy(obs))
            action = mu.numpy()[0]
        # Model layer: gait -> IK -> joint targets + residual
        gait_foot = foot_targets_batched(
            np.array([t_clock], dtype=np.float64),
            np.array([vel_cmd[0]], dtype=np.float64),
            np.array([vel_cmd[1]], dtype=np.float64),
            np.array([vel_cmd[2]], dtype=np.float64),
            gait)
        q_target = inverse_kinematics_batched(gait_foot)[0]   # (4, 3)
        nan_mask = np.isnan(q_target)
        if nan_mask.any():
            q_target = np.where(nan_mask, nominal_4x3, q_target)
        q_ctrl = q_target.reshape(NJ) + action * RES_SCALE
        q_mjcf = np.clip(q_ctrl, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        # MJCF actuator ctrl layout: [pos, vel] per joint, interleaved.
        ctrl = np.zeros(mjm.nu, dtype=np.float32)
        ctrl[0::2] = q_mjcf
        mjd.ctrl[:] = ctrl
        last_action = action

    print(f"loaded {PT.name}; cmd vx={FIXED_VX}")
    print("Viewer opening — close the window to exit. Auto-resets on fall.")

    with mujoco.viewer.launch_passive(mjm, mjd) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 135
        step_count = 0
        last_time = time.time()
        while viewer.is_running():
            policy_step()
            mujoco.mj_step(mjm, mjd)
            step_count += 1
            # Track body for the camera
            viewer.cam.lookat[:] = mjd.qpos[0:3]
            viewer.sync()
            # Auto-reset on fall
            if mjd.qpos[2] < BZ_FAIL:
                print(f"  step {step_count}: fall (bz={mjd.qpos[2]:.3f}); "
                      f"reset")
                reset()
                t_clock = 0.0
                last_action[:] = 0.0
                step_count = 0
            # Pace to real-time roughly
            now = time.time()
            sleep_dt = DT - (now - last_time)
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            last_time = time.time()


if __name__ == "__main__":
    main()
