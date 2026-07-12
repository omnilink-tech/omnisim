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

"""Gymnasium env: G1 legs-only humanoid standing in MuJoCo.

Single-env CPU MuJoCo wrapper. Observation, action, and reward chosen
to be the minimum needed to learn standing from scratch (no analytic
baseline, no gait). Mirrors the shape of spot_residual_env but adapted
for biped + no IK/gait.

Obs (48-dim float32):
    pelvis_lin_vel(3) + pelvis_ang_vel(3) + proj_gravity(3) +
    joint_q(13) + joint_qd(13) + last_action(13)

Action (13-dim, [-1,1] each):
    Joint-space delta in radians, scaled by ACT_SCALE = 0.3 rad,
    added to NOMINAL_POSE to form the position target for each
    actuator.

Reward:
    + 1.0  alive bonus
    - 1.0 * roll^2 + pitch^2  (upright)
    - 0.5 * (pelvis_lin_vel norm)  (don't translate)
    - 0.1 * (pelvis_ang_vel norm)  (don't spin)
    - 0.01 * action^2 sum  (smoothness)
    - 5.0 * max(0, BZ_TARGET - bz)^2  (height penalty if sinking)

Episode termination:
    bz < 0.45  OR  |roll| > 0.8  OR  |pitch| > 0.8  OR  step > MAX_EP

Spawn variation: each reset perturbs joint qpos by ±0.05 rad and
pelvis xy by ±0.03 m so the policy doesn't memorize one initial state.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

from projects.policies.research.backends.g1_robot_spec import (  # noqa: E402
    JOINT_NAMES, NOMINAL_POSE,
)


LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
NOMINAL_BY_JOINT = dict(zip(JOINT_NAMES, NOMINAL_POSE))
NOMINAL_LEGS = np.array([NOMINAL_BY_JOINT[j] for j in LEGS_JOINTS], dtype=np.float32)
NJ = 13

ACT_SCALE = 0.3      # rad per action unit. For G1 the policy carries most of
                     # the work; the analytic baseline is just a soft prior.
                     # Spot used 0.15 because its gait-IK baseline already walks.
SPAWN_Z = 0.78
BZ_TARGET = 0.70
BZ_FAIL = 0.45
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
MAX_EP = 500
SUBSTEPS = 4  # MuJoCo timestep is 2 ms; one env-step = 8 ms (125 Hz control)

# Analytic balance PD baseline. Mirrors g1_model_walk.py: ankle-only,
# small gains. The residual policy is the part that has to actually
# stabilize the biped — the analytic layer just keeps the robot
# pointed in a reasonable direction long enough for the policy to
# react. Hip balance signs are robot-specific and easy to get wrong;
# leaving them out of the baseline keeps the bug surface small.
KP_ANKLE_PITCH = -1.5
KD_ANKLE_PITCH = -0.2
KP_ANKLE_ROLL = -1.5
KD_ANKLE_ROLL = -0.2
BAL_CLAMP = 0.2

# Indices into LEGS_JOINTS for ankle/hip pitch+roll.
_LEFT_ANKLE_PITCH = LEGS_JOINTS.index("left_ankle_pitch_joint")
_RIGHT_ANKLE_PITCH = LEGS_JOINTS.index("right_ankle_pitch_joint")
_LEFT_ANKLE_ROLL = LEGS_JOINTS.index("left_ankle_roll_joint")
_RIGHT_ANKLE_ROLL = LEGS_JOINTS.index("right_ankle_roll_joint")
_LEFT_HIP_PITCH = LEGS_JOINTS.index("left_hip_pitch_joint")
_RIGHT_HIP_PITCH = LEGS_JOINTS.index("right_hip_pitch_joint")
_LEFT_HIP_ROLL = LEGS_JOINTS.index("left_hip_roll_joint")
_RIGHT_HIP_ROLL = LEGS_JOINTS.index("right_hip_roll_joint")


def _quat_to_rpy(qw, qx, qy, qz):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _proj_gravity(qw, qx, qy, qz):
    """Gravity (0,0,-1) rotated into the body frame."""
    # Rotation matrix from body to world via quaternion.
    # Then transpose (= inverse for unit quaternion) to go world -> body.
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 1 - 2 * (qx * qx + qz * qz)
    r12 = 2 * (qy * qz - qw * qx)
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 1 - 2 * (qx * qx + qy * qy)
    # body = R^T * world. world_g = (0,0,-1).
    return (-r02, -r12, -r22)


class G1StandEnv(gym.Env):
    """Single-instance MuJoCo G1 standing env."""

    metadata = {"render_modes": []}

    def __init__(self, urdf_path: str | None = None, seed: int | None = None):
        super().__init__()
        if urdf_path is None:
            urdf_path = REPO / "projects/robots/unitree/g1/urdf/g1_legs_mjcf.urdf"
        urdf_path = Path(urdf_path)
        from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
        self.model = load_or_convert(urdf_path, actuator_joints=list(LEGS_JOINTS),
                                     kp=250.0, kv=10.0)
        self.data = mujoco.MjData(self.model)
        self._rng = np.random.default_rng(seed)

        # Joint qpos addresses + ctrl indices.
        self._qpos_idx = []
        self._qvel_idx = []
        for jname in LEGS_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise RuntimeError(f"joint {jname} not in MJCF")
            self._qpos_idx.append(self.model.jnt_qposadr[jid])
            self._qvel_idx.append(self.model.jnt_dofadr[jid])
        self._qpos_idx = np.array(self._qpos_idx, dtype=np.int32)
        self._qvel_idx = np.array(self._qvel_idx, dtype=np.int32)

        self._ctrl_map = np.full(NJ, -1, dtype=np.int32)
        for i in range(self.model.nu):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for j, jname in enumerate(LEGS_JOINTS):
                if jname in n:
                    self._ctrl_map[j] = i
                    break
        if (self._ctrl_map < 0).any():
            missing = [LEGS_JOINTS[i] for i in range(NJ) if self._ctrl_map[i] < 0]
            raise RuntimeError(f"missing actuators: {missing}")

        self.observation_space = spaces.Box(low=-10.0, high=10.0, shape=(48,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(NJ,), dtype=np.float32)

        self._steps = 0
        self._last_action = np.zeros(NJ, dtype=np.float32)
        self._prev_lin_pos = np.zeros(3, dtype=np.float32)

    def _obs(self) -> np.ndarray:
        q = self.data.qpos
        qd = self.data.qvel
        # Pelvis linear vel (qvel[0:3] is base linear velocity).
        lin_vel = qd[0:3].astype(np.float32)
        ang_vel = qd[3:6].astype(np.float32)
        qw, qx, qy, qz = q[3], q[4], q[5], q[6]
        proj_g = np.array(_proj_gravity(qw, qx, qy, qz), dtype=np.float32)
        joint_q = q[self._qpos_idx].astype(np.float32) - NOMINAL_LEGS
        joint_qd = qd[self._qvel_idx].astype(np.float32)
        obs = np.concatenate([lin_vel, ang_vel, proj_g, joint_q, joint_qd, self._last_action])
        return obs.astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        # Spawn pose with small random perturbation.
        spawn_x = float(self._rng.uniform(-0.03, 0.03))
        spawn_y = float(self._rng.uniform(-0.03, 0.03))
        self.data.qpos[0] = spawn_x
        self.data.qpos[1] = spawn_y
        self.data.qpos[2] = SPAWN_Z
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        joint_noise = self._rng.uniform(-0.05, 0.05, size=NJ).astype(np.float32)
        for k, idx in enumerate(self._qpos_idx):
            self.data.qpos[idx] = float(NOMINAL_LEGS[k] + joint_noise[k])
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self._steps = 0
        self._last_action = np.zeros(NJ, dtype=np.float32)
        self._prev_lin_pos = np.array(self.data.qpos[0:3], dtype=np.float32)
        self._prev_roll = 0.0
        self._prev_pitch = 0.0
        return self._obs(), {}

    def _balance_baseline(self) -> np.ndarray:
        """Compute analytic balance correction on top of NOMINAL_POSE.

        Mirrors Spot's analytic baseline (NOMINAL pose + body-pose PD).
        Returns 13-dim joint-target vector. The residual policy adds
        ±ACT_SCALE rad on top of this.
        """
        qw, qx, qy, qz = self.data.qpos[3:7]
        roll, pitch, _ = _quat_to_rpy(qw, qx, qy, qz)
        # Sim dt for rate (env-step granularity).
        dt = max(SUBSTEPS * float(self.model.opt.timestep), 1e-6)
        roll_rate = (roll - self._prev_roll) / dt
        pitch_rate = (pitch - self._prev_pitch) / dt
        self._prev_roll = roll
        self._prev_pitch = pitch

        ap = KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate
        ap = float(np.clip(ap, -BAL_CLAMP, BAL_CLAMP))
        ar = KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate
        ar = float(np.clip(ar, -BAL_CLAMP, BAL_CLAMP))

        targets = NOMINAL_LEGS.copy()
        targets[_LEFT_ANKLE_PITCH] += ap
        targets[_RIGHT_ANKLE_PITCH] += ap
        targets[_LEFT_ANKLE_ROLL] += ar
        targets[_RIGHT_ANKLE_ROLL] += ar
        return targets

    def step(self, action: np.ndarray):
        action = np.clip(action.astype(np.float32), -1.0, 1.0)
        # Residual recipe: baseline (NOMINAL + analytic balance PD) +
        # tiny policy delta. With ACT_SCALE=0.15 rad the policy can
        # nudge joints to fix what the baseline misses.
        baseline = self._balance_baseline()
        targets = baseline + ACT_SCALE * action
        for k, ci in enumerate(self._ctrl_map):
            self.data.ctrl[ci] = float(targets[k])

        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

        self._steps += 1
        self._last_action = action.copy()

        bx, by, bz = self.data.qpos[0:3]
        qw, qx, qy, qz = self.data.qpos[3:7]
        roll, pitch, _ = _quat_to_rpy(qw, qx, qy, qz)
        lin_vel = float(np.linalg.norm(self.data.qvel[0:3]))
        ang_vel = float(np.linalg.norm(self.data.qvel[3:6]))

        # Reward shape, post-tuning:
        #   - Alive bonus is the dominant signal: every step you don't
        #     fall over earns +1. Across a 500-step episode that's +500
        #     ceiling, far larger than any penalty.
        #   - Upright bonus (1 - roll^2 - pitch^2, clipped to 0) rewards
        #     better-than-tipping postures. Quadratic shape gives a
        #     soft gradient toward upright without dominating early.
        #   - Lin/ang vel penalties small enough to not punish the
        #     policy for ankle/hip corrections that briefly move the
        #     COM. With ACT_SCALE=0.3 the policy MUST make body
        #     adjustments to balance; punish them lightly.
        #   - Action L2 keeps the policy smooth.
        upright = max(0.0, 1.0 - roll * roll - pitch * pitch)
        action_pen = float(np.sum(action ** 2))

        reward = (
            1.0                       # alive
            + 0.5 * upright           # posture quality bonus
            - 0.1 * lin_vel
            - 0.05 * ang_vel
            - 0.01 * action_pen
        )

        terminated = (
            bz < BZ_FAIL
            or abs(roll) > ROLL_FAIL
            or abs(pitch) > PITCH_FAIL
        )
        truncated = self._steps >= MAX_EP

        info = {"bz": bz, "roll": roll, "pitch": pitch}
        return self._obs(), float(reward), bool(terminated), bool(truncated), info


def make_env(seed: int | None = None):
    def thunk():
        return G1StandEnv(seed=seed)
    return thunk


if __name__ == "__main__":
    env = G1StandEnv(seed=0)
    obs, _ = env.reset()
    print(f"obs shape: {obs.shape}; action shape: {env.action_space.shape}")
    total = 0.0
    for i in range(50):
        a = np.zeros(NJ, dtype=np.float32)
        obs, r, term, trunc, info = env.step(a)
        total += r
        if term or trunc:
            print(f"ep ended @ step {i}, total reward {total:.2f}, info={info}")
            break
    else:
        print(f"50 zero-action steps: total reward {total:.2f}")
