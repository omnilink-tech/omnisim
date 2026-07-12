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

"""Compare survival under different obs computations.

Tests three obs modes for the qd_act field of the policy obs:
  1. raw         - MJX's training convention: use dx.qvel directly
                   (matches mjx_gpu.py build_obs)
  2. fd_16ms     - finite-diff over 16ms (8 physics substeps per policy
                   call). Closer to what a real velocity sensor gives.
  3. fd_2ms      - finite-diff over 2ms (1 physics substep). Closer to
                   raw qvel but biased low by half (avg accel * dt / 2).

Plus the deploy gap test: the rl_deploy controller can only finite-diff
because Webots Motor doesn't expose joint velocity. If the policy is
robust to fd_16ms (option 2), it'll deploy cleanly; if it only works
with raw qvel (option 1), we need to retrain with finite-diff baked
into MJX.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


def trial(policy, spec, mode, n_steps=1024, bz_fail=0.30, vx=0.0):
    import mujoco
    import onnxruntime as ort
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert

    n_act = len(spec["joint_names"])
    nominal = np.array(spec["nominal_pose"], dtype=np.float32)
    lo = np.array(spec["joint_limits_lo"], dtype=np.float32)
    hi = np.array(spec["joint_limits_hi"], dtype=np.float32)
    action_scale = float(spec.get("action_scale", 0.15))
    obs_dim = int(spec["obs_dim"])
    spawn = spec.get("spawn_translation", [0, 0, 0.88])
    per_joint = spec.get("motor_pid_per_joint", []) or []
    if per_joint:
        kp = [float(p[0]) for p in per_joint]
        kv = [float(p[2]) for p in per_joint]
    else:
        mp = spec.get("motor_pid", (20.0, 0.0, 0.3))
        kp = float(mp[0]); kv = float(mp[2])

    m = load_or_convert(Path(spec["urdf_path"]), actuator_joints=list(spec["joint_names"]),
                        joint_limits=list(zip(spec["joint_limits_lo"], spec["joint_limits_hi"])),
                        kp=kp, kv=kv)
    sess = ort.InferenceSession(str(policy),
                                providers=["CPUExecutionProvider"])

    d = mujoco.MjData(m)
    d.qpos[0]=spawn[0]; d.qpos[1]=spawn[1]; d.qpos[2]=spawn[2]
    d.qpos[3:7]=[1.0,0,0,0]
    d.qpos[7:7+n_act]=nominal
    d.qvel[:]=0
    d.ctrl[:]=nominal

    if mode == "raw":
        substeps = 1
    elif mode == "fd_16ms":
        substeps = 8
    elif mode == "fd_2ms":
        substeps = 1
    else:
        raise ValueError(mode)

    last_action = np.zeros(n_act, dtype=np.float32)
    prev_q = nominal.copy()
    vel_cmd = np.array([vx, 0.0, 0.0], dtype=np.float32)
    t_sim = 0.0

    for step in range(n_steps):
        v_lin = d.qvel[:3].astype(np.float32)
        v_ang = d.qvel[3:6].astype(np.float32)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, d.qpos[3:7])
        R = R.reshape(3,3)
        proj_g = np.array([-R[0,2], -R[1,2], -R[2,2]], dtype=np.float32)
        q_act = d.qpos[7:7+n_act].astype(np.float32)
        if mode == "raw":
            qd_act = d.qvel[6:6+n_act].astype(np.float32)
        else:
            qd_act = ((q_act - prev_q) / 0.016).astype(np.float32)
        prev_q = q_act.copy()
        clock = (t_sim * 0.5) % 1.0
        obs = np.zeros(obs_dim, dtype=np.float32)
        s=0
        obs[s:s+3]=v_lin; s+=3
        obs[s:s+3]=v_ang; s+=3
        obs[s:s+3]=proj_g; s+=3
        obs[s:s+n_act]=q_act; s+=n_act
        obs[s:s+n_act]=qd_act; s+=n_act
        obs[s:s+n_act]=last_action; s+=n_act
        obs[s:s+3]=vel_cmd; s+=3
        obs[s]=clock
        obs = np.clip(np.nan_to_num(obs), -10, 10).astype(np.float32)
        action = sess.run(None, {"obs": obs.reshape(1,-1)})[0][0]
        action = np.clip(action, -1, 1).astype(np.float32)
        last_action = action.copy()
        target = np.clip(nominal + action_scale * action, lo, hi)
        d.ctrl[:] = target
        for _ in range(substeps):
            mujoco.mj_step(m, d)
            t_sim += m.opt.timestep
        if d.qpos[2] < bz_fail:
            return step
    return n_steps


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    args = p.parse_args()
    spec_path = args.policy.parent / "robot_spec.json"
    spec = json.loads(spec_path.read_text())
    print(f"Policy: {args.policy}")
    for mode in ("raw", "fd_2ms", "fd_16ms"):
        n = trial(args.policy, spec, mode)
        print(f"  qd_mode={mode:<8s} survived {n:>4} / 1024 steps  "
              f"({n*0.016:.2f} s)")


if __name__ == "__main__":
    main()
