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

"""Open a MuJoCo viewer and watch the GPU-trained OmniQuad policy walk.

Runs the trained torch policy + the CPG/pitch-trim feedforward in a single
MuJoCo instance using the EXACT model the policy trained on (the Newton
export), reproducing the trainer's per-step control loop. Resets on a fall
so the window keeps showing the gait.

Run:
    python projects/policies/research/tools/show_omniquad_walk.py \
        --ckpt projects/policies/research/training/runs/gpu_omniquad/policy_f26b.pt \
        --cpg-freq 2.6 --cpg-hipy 0.16
"""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO / "projects" / "rl" / "training"))
from gpu_mjwarp_trainer import (classify_joints, quat_to_rp, proj_gravity,
                                OBS_DIM, NJ, QPOS_J0, QVEL_J0, ACTION_SCALE,
                                PITCH_TRIM, CPG_KNEE, DT, STAND_Z,
                                ROLL_FAIL, PITCH_FAIL, BZ_FAIL)


def main():
    import mujoco, mujoco.viewer, torch, torch.nn as nn
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=str(REPO / "projects/policies/research/training/runs/gpu_omniquad/policy_f26b.pt"))
    ap.add_argument("--mjcf", default=r"C:\tmp\omniquad_newton_export.xml")
    ap.add_argument("--cpg-freq", type=float, default=2.6)
    ap.add_argument("--cpg-hipy", type=float, default=0.16)
    ap.add_argument("--vx", type=float, default=0.5)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.mjcf)
    d = mujoco.MjData(m)
    nominal, phase_off, is_hipy, is_knee = classify_joints(m)
    trim = is_hipy.astype(np.float32) * PITCH_TRIM

    class Pi(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))
        def forward(self, o): return self.pi(o)
    pi = Pi()
    sd = torch.load(args.ckpt, map_location="cpu")
    pi.load_state_dict({k: v for k, v in sd.items() if k.startswith("pi.")})
    pi.eval()
    print(f"[show] loaded {args.ckpt}  cpg_freq={args.cpg_freq} vx_cmd={args.vx}")

    seed = d.qpos.copy()
    seed[0:3] = [0, 0, STAND_Z]; seed[3:7] = [1, 0, 0, 0]
    seed[QPOS_J0:QPOS_J0 + NJ] = nominal

    def reset():
        d.qpos[:] = seed; d.qvel[:] = 0
        mujoco.mj_forward(m, d)

    state = {"phase": 0.0, "last_a": np.zeros(NJ, np.float32)}

    def control():
        qpos, qvel = d.qpos, d.qvel
        vlin, vang = qvel[0:3], qvel[3:6]
        pg = proj_gravity(qpos[None, 3:7])[0]
        q = qpos[QPOS_J0:QPOS_J0 + NJ]; qd = qvel[QVEL_J0:QVEL_J0 + NJ]
        clock = state["phase"] % 1.0
        vel_cmd = np.array([args.vx, 0, 0], np.float32)
        obs = np.concatenate([vlin, vang, pg, q, qd, state["last_a"], vel_cmd, [clock]]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs), -10, 10)
        with torch.no_grad():
            a = pi(torch.from_numpy(obs)).numpy()
        a = np.clip(a, -1, 1).astype(np.float32)
        state["last_a"] = a
        state["phase"] += args.cpg_freq * DT
        theta = 2 * np.pi * (state["phase"] + phase_off)
        s = np.maximum(0.0, np.sin(theta))
        cpg = is_hipy * args.cpg_hipy * np.cos(theta) - is_knee * CPG_KNEE * (s * s)
        targets = nominal + trim + cpg + ACTION_SCALE * a
        d.ctrl[0::2] = targets
        d.ctrl[1::2] = 0.0

    reset()
    print("[show] opening viewer window... close it to quit.")
    with mujoco.viewer.launch_passive(m, d) as v:
        # follow the robot
        v.cam.distance = 3.0; v.cam.elevation = -18; v.cam.azimuth = 120
        steps = 0
        while v.is_running():
            t0 = time.time()
            control()
            mujoco.mj_step(m, d)
            steps += 1
            roll, pitch = quat_to_rp(d.qpos[None, 3:7])
            roll, pitch = float(roll[0]), float(pitch[0])
            if (abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL
                    or d.qpos[2] < BZ_FAIL or steps > 4000):
                print(f"  fell/timeout at step {steps}, bx={d.qpos[0]:+.2f}m -> reset")
                reset(); state["phase"] = 0.0; state["last_a"][:] = 0; steps = 0
            v.cam.lookat[:] = d.qpos[0:3]
            v.sync()
            dt = m.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
