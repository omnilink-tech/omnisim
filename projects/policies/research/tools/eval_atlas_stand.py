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

"""Headless evaluation: how long does Atlas stand under a trained policy?

Runs MuJoCo (not MJX — much faster startup, no JIT) with the Atlas spec
loaded, drives ctrl from the ONNX policy each tick, and reports:

  * survival_time:  seconds the pelvis stayed above bz_fail=0.30
  * fall_event:     when bz dropped below bz_fail (or None if survived full episode)
  * final_pose:     pelvis z, roll, pitch at episode end

Usage:
    python projects/policies/research/tools/eval_atlas_stand.py \\
        --policy projects/policies/research/inference/policies/atlas_stand_v2/policy.onnx \\
        --episode-steps 1024 --vx 0.0

Validates the same observation packing + action scaling the deploy
controller uses, but in-process (no Webots overhead). Useful for quick
"did this checkpoint learn to stand" checks while training iterates.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True,
                   help="ONNX policy path")
    p.add_argument("--spec", type=Path, default=None,
                   help="robot_spec.json (defaults to policy's sibling)")
    p.add_argument("--episode-steps", type=int, default=1024)
    p.add_argument("--bz-fail", type=float, default=0.30)
    p.add_argument("--vx", type=float, default=0.0,
                   help="velocity command vx (m/s)")
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--wz", type=float, default=0.0)
    p.add_argument("--n-trials", type=int, default=3)
    args = p.parse_args()

    spec_path = args.spec or (args.policy.parent / "robot_spec.json")
    if not args.policy.exists():
        print(f"ERROR: policy not found: {args.policy}", file=sys.stderr)
        return 1
    if not spec_path.exists():
        print(f"ERROR: spec not found: {spec_path}", file=sys.stderr)
        return 1

    import json
    spec = json.loads(spec_path.read_text())
    joint_names = list(spec["joint_names"])
    n_act = len(joint_names)
    nominal = np.array(spec["nominal_pose"], dtype=np.float32)
    lo = np.array(spec["joint_limits_lo"], dtype=np.float32)
    hi = np.array(spec["joint_limits_hi"], dtype=np.float32)
    action_scale = float(spec.get("action_scale", 0.15))
    obs_dim = int(spec["obs_dim"])
    spawn = spec.get("spawn_translation", [0, 0, 0.88])
    cpg_freq = float(spec.get("cpg_freq_hz", 0.0))
    cpg_hip_y = float(spec.get("cpg_hip_y_amp", 0.0))
    cpg_knee = float(spec.get("cpg_knee_amp", 0.0))
    cpg_phase = spec.get("cpg_leg_phase", [0.0] * n_act)
    per_joint_pid = spec.get("motor_pid_per_joint", []) or []

    # Lazy MuJoCo + ONNX imports.
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
    import mujoco
    import onnxruntime as ort

    urdf = Path(spec["urdf_path"])
    if per_joint_pid:
        kp = [float(p[0]) for p in per_joint_pid]
        kv = [float(p[2]) for p in per_joint_pid]
    else:
        mp = spec.get("motor_pid", (20.0, 0.0, 0.3))
        kp = float(mp[0]); kv = float(mp[2])

    m = load_or_convert(urdf, actuator_joints=joint_names,
                        joint_limits=list(zip(spec["joint_limits_lo"],
                                              spec["joint_limits_hi"])),
                        kp=kp, kv=kv)
    print(f"[eval] model: nq={m.nq} nu={m.nu} dt={m.opt.timestep}")

    sess = ort.InferenceSession(str(args.policy),
                                providers=["CPUExecutionProvider"])
    print(f"[eval] policy: {args.policy.name}")

    def cpg_offsets(t):
        if cpg_freq <= 0.0 or (cpg_hip_y == 0.0 and cpg_knee == 0.0):
            return np.zeros(n_act, dtype=np.float32)
        base = np.zeros(n_act, dtype=np.float32)
        omega = 2 * math.pi * cpg_freq
        for i, jn in enumerate(joint_names):
            phase = float(cpg_phase[i]) if i < len(cpg_phase) else 0.0
            theta = omega * t + 2 * math.pi * phase
            if jn.endswith("_hpy"):
                base[i] = cpg_hip_y * math.cos(theta)
            elif jn.endswith("_kny"):
                s = max(0.0, math.sin(theta))
                base[i] = -cpg_knee * s * s
        return base

    survival_times = []
    pelvis_z_max = []
    pelvis_z_end = []
    pitch_end = []

    for trial in range(args.n_trials):
        d = mujoco.MjData(m)
        d.qpos[0] = spawn[0]; d.qpos[1] = spawn[1]; d.qpos[2] = spawn[2]
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        d.qpos[7:7+n_act] = nominal
        d.qvel[:] = 0
        d.ctrl[:] = nominal

        last_action = np.zeros(n_act, dtype=np.float32)
        prev_q = nominal.copy()
        vel_command = np.array([args.vx, args.vy, args.wz], dtype=np.float32)
        sim_dt = m.opt.timestep
        # Policy decision interval = 16 ms (matches Webots basicTimeStep).
        # Substep 8 physics ticks per policy call so each call advances
        # the same 16 ms of simulated time the training assumed in its
        # clock + finite-diff math.
        policy_dt = 0.016
        substeps = max(1, int(round(policy_dt / sim_dt)))

        survived = args.episode_steps
        z_max = float(d.qpos[2])
        t_sim = 0.0

        for step in range(args.episode_steps):
            # Build obs (legged_locomotion layout — matches deploy).
            v_lin = d.qvel[:3].astype(np.float32)
            v_ang = d.qvel[3:6].astype(np.float32)
            qw, qx, qy, qz = d.qpos[3:7]
            proj_g = np.array([
                -(2.0 * (qx * qz - qw * qy)) * -1.0,  # = 2*(qx*qz - qw*qy)
                -(2.0 * (qy * qz + qw * qx)) * -1.0,
                -(1.0 - 2.0 * (qx * qx + qy * qy)) * -1.0,
            ], dtype=np.float32)
            # Actually use the rl_deploy convention exactly:
            # proj_g = (-R[0,2], -R[1,2], -R[2,2])  in body frame.
            R = np.zeros((3, 3))
            mujoco.mju_quat2Mat(R.ravel(), d.qpos[3:7])
            proj_g = np.array([-R[0, 2], -R[1, 2], -R[2, 2]], dtype=np.float32)

            q_act = d.qpos[7:7+n_act].astype(np.float32)
            # MJX training (post-fix) uses finite-diff qd over the 16 ms
            # policy step — matching what Webots's deploy controller
            # gets from PositionSensor deltas. Eval must do the same so
            # this number tells us about deploy survival, not raw-qvel
            # survival.
            qd_act = ((q_act - prev_q) / policy_dt).astype(np.float32)
            prev_q = q_act.copy()

            clock_freq = cpg_freq if cpg_freq > 0 else 0.5
            clock = (t_sim * clock_freq) % 1.0

            obs = np.zeros(obs_dim, dtype=np.float32)
            s = 0
            obs[s:s+3] = v_lin; s += 3
            obs[s:s+3] = v_ang; s += 3
            obs[s:s+3] = proj_g; s += 3
            obs[s:s+n_act] = q_act; s += n_act
            obs[s:s+n_act] = qd_act; s += n_act
            obs[s:s+n_act] = last_action; s += n_act
            obs[s:s+3] = vel_command; s += 3
            obs[s] = clock
            obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
            obs = np.clip(obs, -10.0, 10.0)

            action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
            action = np.clip(action, -1.0, 1.0).astype(np.float32)
            last_action = action.copy()

            cpg = cpg_offsets(t_sim)
            target = nominal + cpg + action_scale * action
            target = np.clip(target, lo, hi)
            d.ctrl[:] = target

            for _ in range(substeps):
                mujoco.mj_step(m, d)
                t_sim += sim_dt

            z = float(d.qpos[2])
            if z > z_max:
                z_max = z
            if z < args.bz_fail:
                survived = step
                break

        bz = float(d.qpos[2])
        qw, qx, qy, qz = d.qpos[3:7]
        pitch = math.asin(max(-1, min(1, 2*(qw*qy - qz*qx))))
        survival_times.append(survived * policy_dt)
        pelvis_z_max.append(z_max)
        pelvis_z_end.append(bz)
        pitch_end.append(math.degrees(pitch))
        print(f"  trial {trial}: survived {survived:>4}/{args.episode_steps} steps "
              f"({survived*policy_dt:5.2f} s), "
              f"z_max={z_max:.3f} z_end={bz:.3f} pitch_end={math.degrees(pitch):+5.1f}°")

    mean_surv = sum(survival_times) / len(survival_times)
    print(f"[eval] mean survival: {mean_surv:.2f} s "
          f"(max {max(survival_times):.2f} s, min {min(survival_times):.2f} s)")
    print(f"[eval] mean z_end:    {sum(pelvis_z_end)/len(pelvis_z_end):.3f} m")
    print(f"[eval] mean pitch_end: {sum(pitch_end)/len(pitch_end):+.1f}°")
    full_ep_secs = args.episode_steps * 0.016
    if mean_surv >= 0.95 * full_ep_secs:
        print(f"[eval] PASS: policy survives the full episode "
              f"({full_ep_secs:.1f} s)")
        return 0
    print(f"[eval] FAIL: policy falls before {full_ep_secs:.1f} s")
    return 1


if __name__ == "__main__":
    sys.exit(main())
