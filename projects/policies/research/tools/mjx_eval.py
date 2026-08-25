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

"""Evaluate an MJX-trained policy in MJX (its native sim).

If the policy walks forward here but not in Webots, the gap is
sim-to-sim. If it doesn't walk forward here either, training itself
didn't produce a good policy.

Usage (from repo root, inside the WSL venv):
    /root/venv-gpu/bin/python projects/policies/research/tools/mjx_eval.py \
        --checkpoint projects/policies/research/training/runs/omniquad_mjx_v3/omniquad_final.msgpack \
        --duration 10 --vx 0.5
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--envs", type=int, default=32)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.5")

    import jax, jax.numpy as jnp
    import flax.linen as nn
    from flax import serialization
    import mujoco
    from mujoco import mjx
    from typing import Sequence

    from projects.policies.research.backends.omniquad_robot_spec import OMNIQUAD
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert

    # Load checkpoint to extract net architecture + params.
    with open(args.checkpoint, "rb") as f:
        blob = serialization.from_bytes(None, f.read())
    raw_arch = blob["net_arch"]
    if isinstance(raw_arch, dict):
        net_arch = [int(raw_arch[k]) for k in sorted(raw_arch, key=lambda x: int(x))]
    else:
        net_arch = [int(x) for x in raw_arch]
    n_act = int(blob["n_actuators"])
    obs_dim = int(blob["obs_dim"])
    log_std_init = float(blob["log_std_init"])
    print(f"[eval] net_arch={net_arch} n_act={n_act} obs_dim={obs_dim}")

    class MlpActor(nn.Module):
        hidden: Sequence[int]
        out_dim: int
        log_std_init: float
        @nn.compact
        def __call__(self, x):
            for h in self.hidden:
                x = nn.tanh(nn.Dense(h)(x))
            mean = nn.Dense(self.out_dim)(x)
            log_std = self.param(
                "log_std",
                lambda key: jnp.full((self.out_dim,), self.log_std_init))
            return mean, log_std

    actor = MlpActor(hidden=tuple(net_arch), out_dim=n_act, log_std_init=log_std_init)
    params = blob["actor"]
    print(f"[eval] loaded params, log_std={np.asarray(params['params']['log_std'])[:3]}")

    # Build MJX physics with the same URDF the training used.
    m = load_or_convert(
        OMNIQUAD.urdf_path,
        actuator_joints=list(OMNIQUAD.joint_names),
        joint_limits=list(zip(OMNIQUAD.joint_limits_lo, OMNIQUAD.joint_limits_hi)),
    )
    mx = mjx.put_model(m)
    print(f"[eval] mjx model nq={mx.nq} nv={mx.nv} nu={mx.nu}")

    # Reset envs to standing pose at spawn translation.
    def reset_one(rng):
        d = mjx.make_data(mx)
        # Spawn-translate the root (assumes free joint as first 7 qpos).
        qpos = d.qpos.at[0].set(float(OMNIQUAD.spawn_translation[0]))
        qpos = qpos.at[1].set(float(OMNIQUAD.spawn_translation[1]))
        qpos = qpos.at[2].set(float(OMNIQUAD.spawn_translation[2]))
        # Identity quaternion for orientation.
        qpos = qpos.at[3].set(1.0).at[4].set(0.0).at[5].set(0.0).at[6].set(0.0)
        # Set actuated joints to nominal pose.
        for i, q in enumerate(OMNIQUAD.nominal_pose):
            qpos = qpos.at[7 + i].set(float(q))
        d = d.replace(qpos=qpos)
        # Settle 20 ticks before policy starts driving (matches training).
        def settle_step(d, _):
            return mjx.step(mx, d.replace(ctrl=jnp.zeros(mx.nu))), None
        d, _ = jax.lax.scan(settle_step, d, None, length=20)
        return d

    rng = jax.random.PRNGKey(args.seed)
    keys = jax.random.split(rng, args.envs)
    states = jax.vmap(reset_one)(keys)

    # Build obs from the env state (mirror of what mjx_gpu.py does).
    nominal = jnp.asarray(OMNIQUAD.nominal_pose)
    vel_cmd = jnp.asarray([args.vx, 0.0, 0.0])

    def build_obs(d, last_action, clock):
        # Linear/angular vel: free-joint qvel components.
        v_lin = d.qvel[:3]
        v_ang = d.qvel[3:6]
        # Projected gravity in body frame (from quaternion).
        q = d.qpos[3:7]  # wxyz
        # Body-z column of rotation: g_body = R^T @ [0,0,-1]
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        g_x = -2.0 * (qx*qz - qw*qy) * -1.0
        g_y = -2.0 * (qy*qz + qw*qx) * -1.0
        g_z = -(1.0 - 2.0 * (qx*qx + qy*qy)) * -1.0
        proj_g = jnp.array([g_x, g_y, g_z])
        # Joint positions and velocities (after free joint).
        q_act = d.qpos[7:7+n_act]
        qd_act = d.qvel[6:6+n_act]
        obs = jnp.concatenate([
            v_lin, v_ang, proj_g, q_act, qd_act, last_action, vel_cmd,
            jnp.array([clock])
        ])
        return obs

    @jax.jit
    def policy_action(obs_batch):
        mean, log_std = jax.vmap(lambda o: actor.apply(params, o))(obs_batch)
        # Deterministic: use mean (no exploration noise at eval).
        return jnp.clip(mean, -1.0, 1.0)

    @jax.jit
    def step_batch(states, actions):
        # Apply action: target = nominal + action_scale * action, then mujoco step.
        targets = nominal + float(OMNIQUAD.action_scale) * actions
        # Position actuators: target IS the ctrl. (build_actuators added one per joint.)
        new_states = jax.vmap(lambda s, a: mjx.step(mx, s.replace(ctrl=a)))(states, targets)
        return new_states

    step_dt = float(m.opt.timestep)
    n_steps = int(args.duration / step_dt)
    print(f"[eval] running {n_steps} steps at dt={step_dt}s ({args.envs} envs)")

    last_action = jnp.zeros((args.envs, n_act))
    sim_time = 0.0
    vxs, bzs = [], []
    for i in range(n_steps):
        obs = jax.vmap(lambda s, la: build_obs(s, la, (sim_time * 0.5) % 1.0))(states, last_action)
        actions = policy_action(obs)
        states = step_batch(states, actions)
        last_action = actions
        sim_time += step_dt
        # Sample first env every ~step
        if i % 50 == 0:
            v_lin = states.qvel[:, :3]
            bz = states.qpos[:, 2]
            vxs.append(float(v_lin[:, 0].mean()))
            bzs.append(float(bz.mean()))
    v_lin = states.qvel[:, :3]
    bz_final = float(states.qpos[:, 2].mean())
    print(f"\n[eval] final mean vx (across {args.envs} envs): {float(v_lin[:, 0].mean()):+.4f} m/s")
    print(f"[eval] final mean vy: {float(v_lin[:, 1].mean()):+.4f} m/s")
    print(f"[eval] final body z (across envs): {bz_final:+.3f} (spawn z = {OMNIQUAD.spawn_translation[2]})")
    print(f"[eval] vx samples over time: {[f'{v:+.3f}' for v in vxs[:12]]}")
    print(f"[eval] bz samples over time: {[f'{v:.3f}' for v in bzs[:12]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
