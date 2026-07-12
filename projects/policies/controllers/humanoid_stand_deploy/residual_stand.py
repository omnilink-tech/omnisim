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

"""Residual-RL overlay for the DETERMINISTIC stand (the faithful, gap-free path).

This is the deploy side of the "train where you deploy" residual experiment. The
deterministic stand (`humanoid_stand_deploy`) is a genuinely STABLE base (holds
120 s + survives the cube barrage on its own). This module loads a small RL
residual trained on that SAME stable base and ADDS it to the leg targets, so the
residual is a bounded correction the stable base carries -- NOT the load-bearing
stabiliser (the failure mode of the walk-pipeline residual, which toppled ~1.2 s
in deploy because its baseline could not stand alone).

THE OBSERVATION GAP, CLOSED BY CONSTRUCTION
-------------------------------------------
The historic train->deploy "obs gap" was a representation mismatch: the deploy
read base angular velocity from `getVelocity()` (world frame) while the trainer
read mujoco `qvel[3:6]` (body frame); joint velocity was finite-diff in deploy
but exact in the trainer. Both are now defined in ONE place --
``projects/policies/research/backends/g1_env_core`` -- and THIS module assembles
the policy's observation through exactly those shared functions. The trainer
(`--stand` mode) assembles the identical 50-d vector through the same module, so
the policy is fed in-distribution observations in deploy. The gait sin/cos slot
is frozen (a stand has no gait phase) to (sin,cos)=(0,1).

Layout (the shared 50-d core; NO frame-stack / lookahead / vx tail for the stand)::

    obs = [ lin_vel(3, world) | ang_vel(3, body) | proj_gravity(3) |
            q_leg - nominal(13) | qd_leg(13) | last_action(13) | gait(0,1) ]  == 50

Enable in deploy with ``HSTAND_RESIDUAL=/abs/path/to/policy.onnx`` (the
humanoid_stand_deploy controller imports this and adds the residual to its leg
targets each tick). Opt-in; absent -> the pure deterministic stand is unchanged.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[4]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from projects.policies.research.backends import g1_env_core as core
from projects.policies.research.backends import g1_physics_spec as SPEC

# The 13 legs+waist joints the residual acts on, in the canonical SPEC order
# (== g1.json joint_order[:13] == humanoid_stand_deploy joint_order[:13]).
LEGS_JOINTS = list(SPEC.LEGS_JOINTS)
NOMINAL_LEGS = np.asarray(SPEC.NOMINAL_LEGS, dtype=np.float32)
NJ = len(LEGS_JOINTS)                       # 13
ACT_SCALE = float(SPEC.ACT_SCALE)           # 0.3 rad, the residual authority
CORE_OBS_DIM = core.CORE_OBS_DIM            # 50
# Frozen gait slot for a stand (no phase): sin(0)=0, cos(0)=1.
_GAIT_FROZEN = np.array([[0.0, 1.0]], dtype=np.float32)


class StandResidual:
    """Loads the ONNX residual + assembles the gap-free 50-d obs each tick.

    Usage (in humanoid_stand_deploy, once the legs settle)::

        res = StandResidual(onnx_path, dt=step_dt, act_scale=ACT_SCALE)
        ...
        delta = res.step(ori, vel6, q_leg)   # 13-vec, ADD to the leg targets
        targets[leg_jn] += delta[i]
    """

    def __init__(self, onnx_path: str, *, dt: float,
                 act_scale: float = ACT_SCALE, qd_tau: float = 0.0,
                 res_scale_vec: np.ndarray | None = None):
        import onnxruntime as ort  # ort=1.26.0 ships in the deploy python
        self.sess = ort.InferenceSession(
            str(onnx_path), providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.out_name = self.sess.get_outputs()[0].name
        in_shape = self.sess.get_inputs()[0].shape
        self.obs_in = int(in_shape[-1]) if in_shape and in_shape[-1] else CORE_OBS_DIM
        self.dt = float(dt)
        self.act_scale = float(act_scale)
        self.qd_est = core.JointVelEstimator(self.dt, qd_tau)
        self.last_action = np.zeros(NJ, dtype=np.float32)
        # Per-joint residual scale (optional taper, e.g. smaller on roll/yaw).
        self.res_scale_vec = (np.ones(NJ, dtype=np.float32) if res_scale_vec is None
                              else np.asarray(res_scale_vec, dtype=np.float32))
        self._seeded = False

    def reset(self, q_leg) -> None:
        """Seed the qd estimator at the settled pose so the first qd is 0."""
        q = np.asarray(q_leg, dtype=np.float32)
        self.qd_est.reset(q)
        self.last_action[:] = 0.0
        self._seeded = True

    def build_obs(self, ori, vel6, q_leg) -> np.ndarray:
        """The 50-d core obs, assembled through the SHARED g1_env_core funcs so it
        is bit-identical to the trainer's ``--stand`` obs. Returns [1, 50] float32."""
        q = np.asarray(q_leg, dtype=np.float32)
        if not self._seeded:
            self.reset(q)
        lin = np.asarray(vel6[0:3], dtype=np.float32).reshape(1, 3)
        ang_body = np.asarray(core.world_ang_to_body_matrix(ori, vel6[3:6]),
                              dtype=np.float32).reshape(1, 3)
        pg = np.asarray(core.proj_gravity_from_matrix(ori),
                        dtype=np.float32).reshape(1, 3)
        qrel = (q - NOMINAL_LEGS).reshape(1, NJ)
        qd = self.qd_est.update(q).astype(np.float32).reshape(1, NJ)
        la = self.last_action.reshape(1, NJ)
        obs = core.assemble_walk_obs(lin, ang_body, pg, qrel, qd, la,
                                     _GAIT_FROZEN, sanitize=True)
        return np.asarray(obs, dtype=np.float32)

    def step(self, ori, vel6, q_leg) -> np.ndarray:
        """Run the policy; return the 13-vec leg target DELTA (rad) to ADD on the
        deterministic baseline. Action is clamped [-1,1] (the ONNX already does
        this) then scaled by act_scale * per-joint res_scale."""
        obs = self.build_obs(ori, vel6, q_leg)
        action = self.sess.run([self.out_name], {self.in_name: obs})[0]
        action = np.clip(np.asarray(action, dtype=np.float32).reshape(NJ), -1.0, 1.0)
        self.last_action[:] = action
        return self.act_scale * self.res_scale_vec * action
