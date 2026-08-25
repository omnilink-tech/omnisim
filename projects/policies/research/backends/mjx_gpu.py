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

"""MuJoCo MJX backend — GPU-batched PPO training.

This is OmniSim's "fast track" for RL: replaces the per-env OmniSim
subprocess (~150 steps/s) with a JAX-compiled batched physics step
running 1000+ envs in parallel on GPU (or vectorized on CPU as a
fallback). Same observation/action contract as the sb3 path, so the
exported ONNX policy drops into the same deploy controller.

Speed (rough, your mileage will vary):
    1024 envs on a single 3060   ~  20 000  steps/s
    1024 envs on a A100          ~ 100 000  steps/s
    1024 envs on CPU JAX         ~     500  steps/s   (this Windows box)

Layout:
  * `MjxQuadrupedEnv` — pure-JAX batched env with the same 49-D obs /
    12-D residual action / legged_gym-style reward we use elsewhere.
  * `train_ppo` — small ~250-LOC PPO loop in JAX/flax. Not as battle-
    tested as SB3 but does the job for legged locomotion.
  * `export_onnx` — converts the trained flax policy params into an
    ONNX MLP that matches the SB3 export bit-for-bit (same input
    name 'obs', same output 'action').

The implementation is deliberately self-contained (no rsl_rl
dependency) so the install footprint stays at `pip install jax mujoco
flax optax onnx`.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Dict, NamedTuple, Sequence, Tuple

from .base import RobotSpec, TrainingConfig


class MjxBackend:
    name = "mjx"

    def is_available(self) -> bool:
        try:
            import jax  # noqa: F401
            import mujoco  # noqa: F401
            from mujoco import mjx as _mjx  # noqa: F401
            import flax  # noqa: F401
            import optax  # noqa: F401
            return True
        except ImportError:
            return False

    def train(self, robot, config, out_dir):
        # Lazy imports so the SB3-only user doesn't pay for JAX startup.
        import jax
        import jax.numpy as jnp
        import flax.linen as nn
        import optax
        import numpy as np
        from flax.training.train_state import TrainState
        from flax import serialization
        import mujoco
        from mujoco import mjx

        from ._urdf_to_mjcf import load_or_convert

        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"[mjx] devices: {jax.devices()}")

        # ── Physics setup ──
        # Pull PD gains from the RobotSpec so heavy robots (e.g. Atlas
        # at 175 kg) get stiff enough actuators to hold their stance.
        # motor_pid = (kp, ki, kd); MJX position actuators use (kp, kv).
        # Optional per-joint override via robot.motor_pid_per_joint —
        # used by Atlas to clamp arms+back+neck at high gain while
        # keeping legs moderate so the policy's residual has authority.
        mp_kp, _mp_ki, mp_kv = robot.motor_pid
        per_joint = getattr(robot, "motor_pid_per_joint", None)
        # The default is an empty tuple, which is truthy-False; only
        # use per-joint gains when the sequence is actually populated.
        if per_joint:
            kp_arg = [float(p[0]) for p in per_joint]
            kv_arg = [float(p[2]) for p in per_joint]
        else:
            kp_arg = float(mp_kp)
            kv_arg = float(mp_kv)
        m = load_or_convert(
            robot.urdf_path,
            actuator_joints=list(robot.joint_names),
            joint_limits=list(zip(robot.joint_limits_lo, robot.joint_limits_hi)),
            kp=kp_arg,
            kv=kv_arg,
        )
        mx = mjx.put_model(m)
        nq = m.nq    # generalized position dim
        nv = m.nv    # generalized velocity dim
        n_actuators = m.nu
        print(f"[mjx] mujoco model: nq={nq} nv={nv} nu={n_actuators}")

        # Sanity: actuator count should match joint_names; some URDFs
        # add base-link DOFs that aren't actuated.
        if n_actuators != len(robot.joint_names):
            print(
                f"[mjx] WARNING: nu={n_actuators} != len(joint_names)="
                f"{len(robot.joint_names)}. Proceeding with nu actuators."
            )

        nominal = jnp.array(robot.nominal_pose[:n_actuators], dtype=jnp.float32)
        lo = jnp.array(robot.joint_limits_lo[:n_actuators], dtype=jnp.float32)
        hi = jnp.array(robot.joint_limits_hi[:n_actuators], dtype=jnp.float32)
        action_scale = jnp.float32(robot.action_scale)
        max_ep_steps = int(robot.max_episode_steps)

        # Observation packing. Mirrors the OmniSim deploy controller's
        # layout (see docs/developer/rl-pipeline.md).
        obs_dim = int(robot.obs_dim)
        if obs_dim < 9 + 3*n_actuators + 4:
            raise ValueError(
                f"obs_dim={obs_dim} too small for n_actuators={n_actuators}; "
                "need at least 9 + 3*n_actuators + 4."
            )

        def build_obs(dx, last_action, vel_cmd, t_clock, qd_act):
            v_lin = dx.qvel[:3]                      # base linear vel
            v_ang = dx.qvel[3:6]                     # base angular vel
            # Projected gravity in body frame: rotate world -z by body
            # quaternion. For a free joint, qpos[3:7] is the orientation
            # quaternion (w, x, y, z).
            q = dx.qpos[3:7]
            # Quaternion-rotate (0, 0, -1) into body frame.
            w, x, y, z = q[0], q[1], q[2], q[3]
            # Inverse rotation (conjugate) applied to (0, 0, -1) gives:
            proj_g = jnp.array([
                2.0 * (x*z - w*y) * -1.0,
                2.0 * (y*z + w*x) * -1.0,
                (1.0 - 2.0 * (x*x + y*y)) * -1.0,
            ])
            # Joint state — the actuated joints (skip the free-joint
            # qpos[:7]). qd_act is passed in as a finite-difference over
            # the policy's decision interval (0.016 s), NOT raw dx.qvel.
            # This matches what the Webots deploy controller computes
            # from PositionSensor samples — without this contract, a
            # policy trained on raw qvel will face-plant at deploy
            # because the obs distribution is ~2x off (raw qvel is
            # instantaneous; finite-diff over 16 ms averages it).
            q_act = dx.qpos[7:7 + n_actuators]

            obs = jnp.concatenate([
                v_lin,            # 3
                v_ang,            # 3
                proj_g,           # 3
                q_act,            # n_actuators
                qd_act,           # n_actuators
                last_action,      # n_actuators
                vel_cmd,          # 3
                jnp.array([t_clock], dtype=jnp.float32),   # 1
            ])
            # Pad / truncate to obs_dim.
            obs = jnp.pad(obs, (0, max(0, obs_dim - obs.shape[0])))
            return jnp.nan_to_num(obs[:obs_dim], nan=0.0, posinf=10.0, neginf=-10.0).clip(-10, 10)

        # Build the reward function from the recipe registry. This lets
        # different robots use different reward shaping (legged_locomotion
        # vs manipulation vs balance) without editing this backend.
        from . import reward_recipes
        recipe_fn = reward_recipes.get(robot.obs_recipe)
        weights = reward_recipes.RewardWeights(
            lin_vel_wt=config.r_lin_vel_wt,
            ang_vel_wt=config.r_ang_vel_wt,
            vz_wt=config.r_vz_wt,
            rollpitch_wt=config.r_rollpitch_wt,
            ang_xy_wt=config.r_ang_xy_wt,
            action_rate_wt=config.r_action_rate_wt,
            joint_acc_wt=config.r_joint_acc_wt,
            alive_bonus=config.r_alive_bonus,
            vx_bonus_wt=getattr(config, "r_vx_bonus_wt", 0.0),
            vx_deadband=getattr(config, "r_vx_deadband", 0.05),
        )

        def reward_fn(dx, action, prev_action, vel_cmd, qd_acc):
            state = {
                "qpos": dx.qpos, "qvel": dx.qvel,
                "q_quat": dx.qpos[3:7],
                "v_lin": dx.qvel[:3], "v_ang": dx.qvel[3:6],
                "q_act": dx.qpos[7:7 + n_actuators],
                "qd_act": dx.qvel[6:6 + n_actuators],
            }
            return recipe_fn(state, action, prev_action, vel_cmd, qd_acc, weights)

        def termination(dx):
            bz = dx.qpos[2]
            # Reuse projected-gravity calculation for tilt check.
            q = dx.qpos[3:7]
            w, x, y, z = q[0], q[1], q[2], q[3]
            # roll ≈ atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
            roll = jnp.arctan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
            pitch = jnp.arcsin(jnp.clip(2.0 * (w*y - z*x), -1.0, 1.0))
            return ((bz < config.bz_fail)
                    | (jnp.abs(roll) > config.roll_fail)
                    | (jnp.abs(pitch) > config.pitch_fail))

        # ── Per-env state container (NamedTuple = JAX-pytree-friendly) ──
        class EnvState(NamedTuple):
            mjx_data: Any           # mjx.Data
            last_action: Any        # (nu,)
            prev_action: Any        # (nu,) for action-rate reward
            prev_qvel_act: Any      # (nu,) for joint-acc
            prev_qpos_act: Any      # (nu,) for obs qd finite-diff
            vel_cmd: Any            # (3,)
            episode_step: Any       # int
            episode_return: Any     # float
            rng: Any                # jax.Array

        def reset_one(rng):
            # Sample velocity command and reset mjx data.
            rng, k1, k2 = jax.random.split(rng, 3)
            vx = jax.random.uniform(k1, (), minval=config.vx_min, maxval=config.vx_max)
            wz = jax.random.uniform(k2, (), minval=config.wz_min, maxval=config.wz_max)
            vel_cmd = jnp.array([vx, 0.0, wz])
            d = mjx.make_data(mx)
            # Place at spawn pose + nominal joint config.
            qpos = jnp.array(d.qpos)
            qpos = qpos.at[0].set(robot.spawn_translation[0])
            qpos = qpos.at[1].set(robot.spawn_translation[1])
            qpos = qpos.at[2].set(robot.spawn_translation[2])
            # Free-joint quaternion: identity (1, 0, 0, 0). The URDF
            # importer may have made an explicit free joint or not;
            # qpos[3:7] are the quaternion components either way.
            qpos = qpos.at[3].set(1.0)
            qpos = qpos.at[4:7].set(0.0)
            qpos = qpos.at[7:7+n_actuators].set(nominal)
            d = d.replace(qpos=qpos, qvel=jnp.zeros_like(d.qvel))
            return EnvState(
                mjx_data=d,
                last_action=jnp.zeros((n_actuators,), dtype=jnp.float32),
                prev_action=jnp.zeros((n_actuators,), dtype=jnp.float32),
                prev_qvel_act=jnp.zeros((n_actuators,), dtype=jnp.float32),
                prev_qpos_act=nominal,
                vel_cmd=vel_cmd,
                episode_step=jnp.int32(0),
                episode_return=jnp.float32(0.0),
                rng=rng,
            )

        # MJX physics step (2 ms) vs policy decision interval (16 ms).
        # The Webots deploy world runs at basicTimeStep=16 ms — one
        # controller tick = 16 ms of physics. To match, MJX advances
        # 8 physics substeps per policy call. Anything less makes the
        # finite-diff qd_obs come out 8× too small and the trained
        # policy face-plants at deploy.
        _PHYS_SUBSTEPS = 8

        def step_one(state, action):
            # action in [-1, 1]; map to joint targets.
            clipped = jnp.clip(action, -1.0, 1.0)
            target = jnp.clip(nominal + action_scale * clipped, lo, hi)
            d = state.mjx_data
            d = d.replace(ctrl=target)
            # Substep physics so each policy decision spans 16 ms.
            def _substep(dd, _):
                return mjx.step(mx, dd), None
            d, _ = jax.lax.scan(_substep, d, None, length=_PHYS_SUBSTEPS)

            t_clock = (state.episode_step * 0.016 * 0.5) % 1.0
            # Finite-difference qd over the 16 ms policy step — matches
            # the Webots deploy controller's PositionSensor delta.
            q_act = d.qpos[7:7+n_actuators]
            qd_act_obs = (q_act - state.prev_qpos_act) / 0.016
            obs = build_obs(d, clipped, state.vel_cmd, t_clock, qd_act_obs)
            # joint-acc: derivative of qvel_act.
            qvel_act = d.qvel[6:6+n_actuators]
            qd_acc = (qvel_act - state.prev_qvel_act) / 0.016
            # Cap qd_acc so a single explosive step can't poison reward
            # (reward has a w_joint_acc * sum(qd_acc**2) term).
            qd_acc = jnp.clip(qd_acc, -500.0, 500.0)
            reward = reward_fn(d, clipped, state.prev_action, state.vel_cmd, qd_acc)
            # Defensive: physics can produce NaN/Inf on a bad contact;
            # mask so a single bad env doesn't poison the whole batch.
            reward = jnp.nan_to_num(reward, nan=0.0, posinf=0.0, neginf=0.0)

            done = termination(d) | (state.episode_step + 1 >= max_ep_steps)
            # If physics exploded to NaN/Inf, force a reset.
            done = done | jnp.any(jnp.isnan(d.qpos)) | jnp.any(jnp.isinf(d.qvel))
            # Auto-reset on done.
            def do_reset(_):
                rng_, _key = jax.random.split(state.rng)
                new = reset_one(rng_)
                obs2 = build_obs(new.mjx_data, new.last_action,
                                 new.vel_cmd, 0.0,
                                 jnp.zeros((n_actuators,), dtype=jnp.float32))
                return new, obs2

            def do_continue(_):
                new = EnvState(
                    mjx_data=d,
                    last_action=clipped,
                    prev_action=clipped,
                    prev_qvel_act=qvel_act,
                    prev_qpos_act=q_act,
                    vel_cmd=state.vel_cmd,
                    episode_step=state.episode_step + 1,
                    episode_return=state.episode_return + reward,
                    rng=state.rng,
                )
                return new, obs

            new_state, next_obs = jax.lax.cond(done, do_reset, do_continue, operand=None)
            return new_state, next_obs, reward, done

        # ── PPO ──

        class MlpActor(nn.Module):
            hidden: Sequence[int]
            out_dim: int
            log_std_init: float
            @nn.compact
            def __call__(self, x):
                for h in self.hidden:
                    x = nn.tanh(nn.Dense(h)(x))
                mean = nn.Dense(self.out_dim)(x)
                log_std = self.param("log_std",
                    lambda key: jnp.full((self.out_dim,), self.log_std_init))
                return mean, log_std

        class MlpCritic(nn.Module):
            hidden: Sequence[int]
            @nn.compact
            def __call__(self, x):
                for h in self.hidden:
                    x = nn.tanh(nn.Dense(h)(x))
                return nn.Dense(1)(x).squeeze(-1)

        actor = MlpActor(hidden=tuple(config.net_arch), out_dim=n_actuators,
                         log_std_init=config.log_std_init)
        critic = MlpCritic(hidden=tuple(config.net_arch))

        rng = jax.random.PRNGKey(0)
        rng, ka, kc = jax.random.split(rng, 3)
        dummy_obs = jnp.zeros((obs_dim,), dtype=jnp.float32)
        actor_params = actor.init(ka, dummy_obs)
        critic_params = critic.init(kc, dummy_obs)

        # NB: removed optax.clip_by_global_norm() — empirically that chain
        # was producing Dense-layer grads that flax's TrainState.apply_gradients
        # treated as zero, so the policy MLP never updated (only log_std,
        # which has a different gradient source via the entropy term).
        # Re-enable if a training run produces NaN losses.
        tx = optax.adam(config.learning_rate)
        actor_state = TrainState.create(apply_fn=actor.apply, params=actor_params, tx=tx)
        critic_state = TrainState.create(apply_fn=critic.apply, params=critic_params, tx=tx)

        N = int(config.parallel_envs)
        n_steps = int(config.n_steps_per_env)

        # Vmapped reset/step.
        rng, *keys = jax.random.split(rng, N + 1)
        states = jax.vmap(reset_one)(jnp.stack(keys))
        # initial obs (qd=0 — robot hasn't moved yet)
        def _obs0(s):
            return build_obs(s.mjx_data, s.last_action, s.vel_cmd, 0.0,
                             jnp.zeros((n_actuators,), dtype=jnp.float32))
        obs_batch = jax.vmap(_obs0)(states)

        @jax.jit
        def select_action(params, obs_batch, rng):
            mean, log_std = jax.vmap(lambda o: actor.apply(params, o))(obs_batch)
            std = jnp.exp(log_std)
            rng, k = jax.random.split(rng)
            eps = jax.random.normal(k, mean.shape)
            action = mean + std * eps
            # NB: sum log_std over axis=-1 (action dim), NOT the full batch.
            # vmap broadcasts the log_std param to shape (B, act_dim), so a
            # plain jnp.sum sums B*act_dim entries instead of act_dim. That
            # made old_logp scale with the rollout batch size while the
            # loss-side recompute used a DIFFERENT batch size, producing
            # a huge logp-old_logp gap → ratios exp(20) clamped → PPO
            # clip activated → mean had zero gradient. Spent a chunk of a
            # session debugging this; please leave the axis=-1.
            logp = (-0.5 * jnp.sum(((action - mean)/std)**2, axis=-1)
                    - jnp.sum(log_std, axis=-1)
                    - 0.5 * mean.shape[-1] * jnp.log(2*jnp.pi))
            return action, logp, rng

        @jax.jit
        def step_batch(states, actions):
            new_states, obs2, r, d = jax.vmap(step_one)(states, actions)
            return new_states, obs2, r, d

        @jax.jit
        def critic_value(params, obs_batch):
            return jax.vmap(lambda o: critic.apply(params, o))(obs_batch)

        # ── Rollout & GAE buffer ──
        # Both rollout and GAE use lax.scan so the entire 24-step rollout
        # fuses into a single XLA op. Without this, the Python for-loop
        # spawns one device dispatch per step (24x roundtrip overhead).
        def rollout_step(carry, _):
            actor_p, critic_p, states, obs_batch, rng = carry
            action, logp, rng = select_action(actor_p, obs_batch, rng)
            val = critic_value(critic_p, obs_batch)
            new_states, next_obs, r, d = step_batch(states, action)
            outs = (obs_batch, action, logp, r, val, d)
            new_carry = (actor_p, critic_p, new_states, next_obs, rng)
            return new_carry, outs

        def gae_step(carry, inp):
            gae, next_val = carry
            r, v, d = inp
            delta = r + config.gamma * next_val * (1.0 - d) - v
            gae = delta + config.gamma * config.gae_lambda * (1.0 - d) * gae
            return (gae, v), gae

        def rollout(actor_state, critic_state, states, obs_batch, rng):
            init = (actor_state.params, critic_state.params, states, obs_batch, rng)
            (_, _, states, obs_batch, rng), (
                obs_buf, act_buf, logp_buf, rew_buf, val_buf, done_buf
            ) = jax.lax.scan(rollout_step, init, None, length=n_steps)
            last_val = critic_value(critic_state.params, obs_batch)
            # GAE backwards-scan (reverse=True).
            (_, _), adv = jax.lax.scan(
                gae_step,
                (jnp.zeros(N), last_val),
                (rew_buf, val_buf, done_buf.astype(jnp.float32)),
                reverse=True,
            )
            ret = adv + val_buf
            return (obs_buf, act_buf, logp_buf, adv, ret, states, obs_batch, rng)

        rollout = jax.jit(rollout)

        def ppo_update(actor_state, critic_state, obs, act, old_logp, adv, ret):
            # Normalise advantages, clamping the std floor so a degenerate
            # batch (all-zero adv) can't divide-by-zero.
            adv = (adv - adv.mean()) / (adv.std() + 1e-6)
            adv = jnp.clip(adv, -10.0, 10.0)
            # Cap returns: if the value head briefly blows up, mse loss
            # would blow up too and corrupt the params.
            ret = jnp.clip(ret, -100.0, 100.0)
            def actor_loss_fn(params):
                mean, log_std = jax.vmap(lambda o: actor.apply(params, o))(obs)
                # Cap log_std so std can't go to 0 or infinity.
                log_std = jnp.clip(log_std, -5.0, 2.0)
                std = jnp.exp(log_std)
                # axis=-1 — see the same fix in select_action above.
                logp = (-0.5 * jnp.sum(((act - mean)/std)**2, axis=-1)
                        - jnp.sum(log_std, axis=-1)
                        - 0.5 * mean.shape[-1] * jnp.log(2*jnp.pi))
                ratio = jnp.exp(jnp.clip(logp - old_logp, -20.0, 20.0))
                clipped = jnp.clip(ratio, 1 - config.clip_range, 1 + config.clip_range)
                policy_loss = -jnp.mean(jnp.minimum(ratio * adv, clipped * adv))
                ent = jnp.mean(jnp.sum(log_std + 0.5 * jnp.log(2*jnp.pi*jnp.e), -1))
                return policy_loss - config.ent_coef * ent
            def critic_loss_fn(params):
                v = jax.vmap(lambda o: critic.apply(params, o))(obs)
                return jnp.mean((v - ret) ** 2)
            actor_grad = jax.grad(actor_loss_fn)(actor_state.params)
            critic_grad = jax.grad(critic_loss_fn)(critic_state.params)
            # If grads contain NaN/Inf, zero them out so apply_gradients is
            # effectively a no-op for that update. Previously this branch
            # used jax.tree_util.tree_map(jnp.where(ok, new, old)) to
            # conditionally revert the whole TrainState — but a TrainState
            # is not a pure jax pytree (it carries apply_fn, the optax tx,
            # etc.) and walking the where over those non-array leaves
            # silently produced state where params never actually updated
            # (verified empirically: two unrelated runs saved bit-identical
            # actor params). The zero-out approach is simpler and correct.
            def _zero_if_bad(tree):
                ok = jnp.all(jnp.stack([
                    jnp.all(jnp.isfinite(l))
                    for l in jax.tree_util.tree_leaves(tree)
                ]))
                return jax.tree_util.tree_map(
                    lambda l: jnp.where(ok, l, jnp.zeros_like(l)), tree)
            actor_grad = _zero_if_bad(actor_grad)
            critic_grad = _zero_if_bad(critic_grad)
            actor_state = actor_state.apply_gradients(grads=actor_grad)
            critic_state = critic_state.apply_gradients(grads=critic_grad)
            return actor_state, critic_state

        ppo_update = jax.jit(ppo_update)

        # ── Training loop ──
        total_collected = 0
        last_log = time.time()
        last_ckpt = 0
        ep_return_running = 0.0
        update_count = 0
        start = time.time()
        ckpt_dir = out_dir / "checkpoints"
        ckpt_dir.mkdir(parents=True, exist_ok=True)

        while total_collected < config.total_steps:
            obs_buf, act_buf, logp_buf, adv, ret, states, obs_batch, rng = rollout(
                actor_state, critic_state, states, obs_batch, rng
            )
            # Flatten for PPO update.
            flat_obs = obs_buf.reshape(-1, obs_dim)
            flat_act = act_buf.reshape(-1, n_actuators)
            flat_logp = logp_buf.reshape(-1)
            flat_adv = adv.reshape(-1)
            flat_ret = ret.reshape(-1)
            B = flat_obs.shape[0]
            batch_size = min(config.batch_size, B)
            for _ in range(config.n_epochs):
                rng, k = jax.random.split(rng)
                perm = jax.random.permutation(k, B)
                for s in range(0, B, batch_size):
                    idx = perm[s:s + batch_size]
                    actor_state, critic_state = ppo_update(
                        actor_state, critic_state,
                        flat_obs[idx], flat_act[idx],
                        flat_logp[idx], flat_adv[idx], flat_ret[idx],
                    )
            total_collected += n_steps * N
            update_count += 1
            mean_ret = float(jnp.mean(ret))
            ep_return_running = 0.9 * ep_return_running + 0.1 * mean_ret

            if time.time() - last_log > 5 or update_count <= 3:
                elapsed = time.time() - start
                fps = total_collected / max(elapsed, 1e-3)
                # Pull a few diagnostics so a bad batch is visible.
                # (obs_buf/rew/adv come from the closure-local rollout state
                # of the last update; cheap host syncs.)
                print(f"[mjx] step {total_collected:>8}  fps={fps:5.0f}  "
                      f"ema_return={ep_return_running:+.2f}  "
                      f"upd={update_count}  mean_ret={mean_ret:+.3f}")
                last_log = time.time()

            if total_collected - last_ckpt >= config.checkpoint_every_steps:
                last_ckpt = total_collected
                ckpt = ckpt_dir / f"{robot.name}_{total_collected}.msgpack"
                with open(ckpt, "wb") as f:
                    f.write(serialization.to_bytes({
                        "actor": actor_state.params,
                        "critic": critic_state.params,
                        "net_arch": list(config.net_arch),
                        "n_actuators": n_actuators,
                        "obs_dim": obs_dim,
                        "log_std_init": config.log_std_init,
                    }))
                print(f"[mjx] saved checkpoint {ckpt.name}")

        final_ckpt = out_dir / f"{robot.name}_final.msgpack"
        with open(final_ckpt, "wb") as f:
            f.write(serialization.to_bytes({
                "actor": actor_state.params,
                "critic": critic_state.params,
                "net_arch": list(config.net_arch),
                "n_actuators": n_actuators,
                "obs_dim": obs_dim,
                "log_std_init": config.log_std_init,
            }))
        elapsed = time.time() - start
        print(f"[mjx] final {final_ckpt.name}  elapsed {elapsed:.1f}s "
              f"({total_collected/max(elapsed,1e-3):.0f} steps/s)")
        return final_ckpt

    def export_onnx(self, ckpt, out, robot):
        """Convert MJX msgpack checkpoint -> ONNX MLP matching SB3 export."""
        import numpy as np
        from flax import serialization
        import jax.numpy as jnp
        import flax.linen as nn
        import torch
        import torch.nn as tnn

        with open(ckpt, "rb") as f:
            blob = serialization.from_bytes(None, f.read())

        # serialization.from_bytes(None, ...) returns the raw nested
        # structure. Layers are at blob["actor"]["params"]["Dense_<i>"]
        # with "kernel" (shape [in, out]) and "bias" (shape [out,]).
        actor_params = blob["actor"]
        params_dict = actor_params.get("params", actor_params)

        # msgpack encodes Python lists as dicts with int-string keys.
        raw_arch = blob["net_arch"]
        if isinstance(raw_arch, dict):
            net_arch = [int(raw_arch[k]) for k in sorted(raw_arch, key=lambda x: int(x))]
        else:
            net_arch = [int(x) for x in raw_arch]
        n_actuators = int(blob["n_actuators"])
        obs_dim = int(blob["obs_dim"])

        def get_layer(layer_idx, key):
            arr = params_dict[f"Dense_{layer_idx}"][key]
            return np.asarray(arr, dtype=np.float32)

        # Build torch model: hidden_0, hidden_1, ..., mean_out.
        layers: list = []
        last_dim = int(obs_dim)
        for i, h in enumerate(net_arch):
            h = int(h)
            ln = tnn.Linear(last_dim, h)
            with torch.no_grad():
                w = torch.from_numpy(get_layer(i, "kernel").T.copy())
                b = torch.from_numpy(get_layer(i, "bias").copy())
                ln.weight.copy_(w)
                ln.bias.copy_(b)
            layers.append(ln)
            layers.append(tnn.Tanh())
            last_dim = h

        head = tnn.Linear(last_dim, int(n_actuators))
        with torch.no_grad():
            w = torch.from_numpy(get_layer(len(net_arch), "kernel").T.copy())
            b = torch.from_numpy(get_layer(len(net_arch), "bias").copy())
            head.weight.copy_(w)
            head.bias.copy_(b)
        layers.append(head)

        net = tnn.Sequential(*layers)
        net.eval()
        out.parent.mkdir(parents=True, exist_ok=True)
        dummy = torch.zeros((1, int(obs_dim)), dtype=torch.float32)
        torch.onnx.export(
            net, dummy, str(out),
            input_names=["obs"], output_names=["action"],
            dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
            opset_version=14,
        )

        # Sanity check: ONNX inference matches a torch forward.
        import onnxruntime as ort
        sess = ort.InferenceSession(str(out), providers=["CPUExecutionProvider"])
        test = np.random.randn(1, int(obs_dim)).astype(np.float32)
        onnx_out = sess.run(None, {"obs": test})[0][0]
        with torch.no_grad():
            torch_out = net(torch.from_numpy(test)).numpy()[0]
        diff = float(np.max(np.abs(onnx_out - torch_out)))
        print(f"[mjx] ONNX wrote {out}; sanity diff onnx vs torch: {diff:.2e}")
        if diff > 1e-4:
            print(f"[mjx] WARNING: ONNX export diverges from torch (diff={diff:.3e})")

        # Sidecar spec so the generic deploy controller can drive any URDF.
        spec_path = robot.write_sidecar(out.parent)
        print(f"[mjx] wrote spec sidecar {spec_path}")


def factory() -> MjxBackend:
    return MjxBackend()
