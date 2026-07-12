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

"""Reward recipes — pluggable per-robot reward shaping.

A recipe takes the current physics state (everything the trainer has)
plus the per-step config and returns a scalar reward. Recipes are
JAX-friendly: they should be pure functions over jax tensors so the
MJX trainer can JIT them.

Registered recipes:

  legged_locomotion  : vx/wz tracking, orientation, smoothness, alive bonus.
                       Spot, ANYmal, quadrupeds. Default.
  manipulation       : end-effector position tracking + joint smoothness.
                       Reach-and-touch arms.
  balance            : orientation + base-velocity penalties only. No
                       command tracking. Bipedal standing / pendulum-balance.

To add a recipe:
  1. Write a function with the (state, action, prev_action, vel_cmd,
     prev_qvel_act, params) signature and return a scalar reward.
  2. Register it via `register("name", fn)`.

Recipes only depend on data the trainer already extracts: base pose
quaternion + linear/angular velocity, joint positions/velocities,
last action, velocity command. New observation channels (e.g. end-
effector tracking target) come from the RobotSpec.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict


@dataclass
class RewardWeights:
    """Per-recipe-tunable knobs. Mirrors fields on TrainingConfig so
    weights can come from env vars / CLI at training time."""
    lin_vel_wt: float = 1.0
    ang_vel_wt: float = 0.5
    vz_wt: float = -1.0
    rollpitch_wt: float = -0.1
    ang_xy_wt: float = -0.02
    action_rate_wt: float = -0.005
    joint_acc_wt: float = -2.5e-7
    alive_bonus: float = 0.05
    # Direct forward-velocity bonus: r += vx_bonus_wt * max(0, vx -
    # vx_deadband). Unbounded incentive for higher forward speed; needed
    # because the exp-tracking term alone is too gentle near vx=0 to
    # pull the policy out of the "stand still" local optimum.
    vx_bonus_wt: float = 0.0
    vx_deadband: float = 0.05
    # Manipulation-specific.
    ee_track_wt: float = 1.0
    # Generic clamp on final reward.
    clip_min: float = -5.0
    clip_max: float = +5.0


_REGISTRY: Dict[str, Callable] = {}


def register(name: str, fn: Callable) -> None:
    _REGISTRY[name] = fn


def get(name: str) -> Callable:
    if name not in _REGISTRY:
        raise KeyError(f"Unknown reward recipe {name!r}. "
                       f"Registered: {sorted(_REGISTRY)}")
    return _REGISTRY[name]


def list_recipes() -> list[str]:
    return sorted(_REGISTRY)


# ── Built-in recipes ──
# Each recipe accepts (state_dict, action, prev_action, vel_cmd,
# qd_acc, weights) and returns a scalar jax tensor. state_dict has:
#   qpos        - full mjx generalized coords
#   qvel        - full mjx generalized velocities
#   q_quat      - base quaternion (qpos[3:7]) — (w, x, y, z)
#   v_lin       - base linear vel in world frame (qvel[:3])
#   v_ang       - base angular vel in world frame (qvel[3:6])
#   q_act, qd_act - actuated joint pos & vel
# We compute everything from these uniform fields so recipes are
# physics-engine-agnostic in principle.


def _legged_locomotion(state, action, prev_action, vel_cmd, qd_acc, w):
    import jax.numpy as jnp
    v_lin = state["v_lin"]
    v_ang = state["v_ang"]
    q_quat = state["q_quat"]
    qw, qx, qy, qz = q_quat[0], q_quat[1], q_quat[2], q_quat[3]
    # Tracking xy velocity.
    lin_err = jnp.sum((v_lin[:2] - vel_cmd[:2]) ** 2)
    r_lin = w.lin_vel_wt * jnp.exp(-lin_err / 0.25)
    # Tracking yaw rate.
    wz_err = (v_ang[2] - vel_cmd[2]) ** 2
    r_ang = w.ang_vel_wt * jnp.exp(-wz_err / 0.25)
    # Penalties.
    r_vz = w.vz_wt * v_lin[2] ** 2
    # Body x-axis projection on world z (gravity stable when 0).
    gx = 2.0 * (qx * qz - qw * qy) * -1.0
    gy = 2.0 * (qy * qz + qw * qx) * -1.0
    r_rp = w.rollpitch_wt * (gx * gx + gy * gy)
    r_wxy = w.ang_xy_wt * (v_ang[0] ** 2 + v_ang[1] ** 2)
    r_rate = w.action_rate_wt * jnp.sum((action - prev_action) ** 2)
    r_acc = w.joint_acc_wt * jnp.sum(qd_acc * qd_acc)
    r_alive = w.alive_bonus
    # Unbounded forward-velocity bonus — direct pull toward walking.
    r_vx_bonus = w.vx_bonus_wt * jnp.maximum(0.0, v_lin[0] - w.vx_deadband)
    r = r_lin + r_ang + r_vz + r_rp + r_wxy + r_rate + r_acc + r_alive + r_vx_bonus
    return jnp.clip(r, w.clip_min, w.clip_max)


def _manipulation(state, action, prev_action, vel_cmd, qd_acc, w):
    """End-effector reach: vel_cmd is interpreted as the (x, y, z) target
    in the base frame; reward is the negative distance from the end
    effector position (passed via state['ee_pos']) to the target.

    Caller is expected to put 'ee_pos' in state — the MJX env can derive
    this from the actuator chain's tip body (TODO: requires the spec to
    name the end-effector body).
    """
    import jax.numpy as jnp
    ee_pos = state.get("ee_pos", jnp.zeros(3))
    target = vel_cmd  # repurposed as xyz target
    dist = jnp.sqrt(jnp.sum((ee_pos - target) ** 2) + 1e-6)
    r_track = w.ee_track_wt * jnp.exp(-4.0 * dist)
    r_rate = w.action_rate_wt * jnp.sum((action - prev_action) ** 2)
    r_acc = w.joint_acc_wt * jnp.sum(qd_acc * qd_acc)
    return jnp.clip(r_track + r_rate + r_acc + w.alive_bonus,
                    w.clip_min, w.clip_max)


def _balance(state, action, prev_action, vel_cmd, qd_acc, w):
    """Stay level: penalise body angular velocity + projected-gravity tilt.
    Doesn't track any command. Use for inverted-pendulum-style demos."""
    import jax.numpy as jnp
    v_lin = state["v_lin"]
    v_ang = state["v_ang"]
    q_quat = state["q_quat"]
    qw, qx, qy, qz = q_quat[0], q_quat[1], q_quat[2], q_quat[3]
    gx = 2.0 * (qx * qz - qw * qy) * -1.0
    gy = 2.0 * (qy * qz + qw * qx) * -1.0
    r_rp = w.rollpitch_wt * (gx * gx + gy * gy)
    r_wxy = w.ang_xy_wt * (v_ang[0] ** 2 + v_ang[1] ** 2)
    r_vz = w.vz_wt * v_lin[2] ** 2
    r_rate = w.action_rate_wt * jnp.sum((action - prev_action) ** 2)
    return jnp.clip(r_rp + r_wxy + r_vz + r_rate + w.alive_bonus,
                    w.clip_min, w.clip_max)


register("legged_locomotion", _legged_locomotion)
register("manipulation", _manipulation)
register("balance", _balance)
