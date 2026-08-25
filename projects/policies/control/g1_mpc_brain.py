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

"""g1_mpc_brain.py -- deploy adapter so the MPC brain runs INSIDE OmniSim/Newton.

The MPC (g1_mpc.py) plans by rolling physics forward, so in deploy it carries a
MuJoCo "planner" SIDECAR: Newton simulates the real G1, and each control tick this
adapter copies the real state (which the deploy hands us via BrainState) into the
MuJoCo sidecar, runs the MPPI rollouts there, and returns joint targets to Newton.

It exports the g1_walk_deploy brain-hook interface (G1Brain / BrainState /
BrainConfig), so it is a drop-in:

    G1_BRAIN=1  G1_BRAIN_MODULE=projects.policies.control.g1_mpc_brain

Caveats: (1) the plan is computed against MuJoCo while Newton evolves the real
robot -- a model mismatch the per-tick re-planning only partly absorbs; (2) the
rollouts make this MUCH slower than realtime, so OmniSim runs in slow motion.
Tune the planner cost with G1_MPC_K / G1_MPC_ITERS (lighter = faster, less stable).
"""
from __future__ import annotations
import math
import os
from dataclasses import dataclass

import numpy as np
import mujoco

from projects.policies.control.gait import g1_human_gait as ghg
from projects.policies.control.g1_brain import LIM_LO, LIM_HI
from projects.policies.control.g1_mpc import G1MPC, RES_J
import projects.policies.control.g1_brain_sim as _sim   # ARM_NOMINAL + Plant adr maps


@dataclass
class BrainState:
    """Exactly the fields the g1_walk_deploy hook fills each tick."""
    roll: float
    pitch: float
    yaw: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    vx: float
    vy: float
    vz: float
    bz: float
    joint_q: np.ndarray
    joint_qd: np.ndarray
    t: float


@dataclass
class BrainConfig:
    K: int = 32
    iters: int = 1
    H: int = 50
    nthread: int = 1          # 1 in deploy: the threaded rollout pool contends
    #                           with Newton/warp inside the controller process.
    @classmethod
    def from_env(cls, env=None):
        env = os.environ if env is None else env
        def _i(name, cur):
            v = env.get(name)
            return int(v) if v not in (None, "") else cur
        return cls(K=_i("G1_MPC_K", cls.K), iters=_i("G1_MPC_ITERS", cls.iters),
                   H=_i("G1_MPC_H", cls.H), nthread=_i("G1_MPC_NTHREAD", cls.nthread))


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class G1Brain:
    """Drop-in deploy brain that runs MPC against a MuJoCo sidecar."""

    def __init__(self, cfg: BrainConfig | None = None):
        cfg = cfg or BrainConfig.from_env()
        self.mpc = G1MPC(dict(K=cfg.K, iters=cfg.iters, H=cfg.H, nthread=cfg.nthread))
        self._nstep = 0
        self._diag = os.environ.get("G1_MPC_DIAG", "")
        self._log(f"[g1_mpc_brain] MPC loaded: K={cfg.K} iters={cfg.iters} "
                  f"H={cfg.H} nthread={cfg.nthread}")

    def _log(self, msg):
        if not self._diag:
            return
        try:
            with open(self._diag, "a") as fh:
                fh.write(msg + "\n")
        except Exception:
            pass
        self.plant = self.mpc.plant            # the MuJoCo sidecar
        self.m, self.d = self.plant.m, self.plant.d
        # arm DOF addresses (Plant only exposes qadr for arms)
        self.arm_dadr = np.array([
            self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in _sim.ARM_JOINTS])
        self.phase = ghg.DS_PHASE
        self.rng = np.random.default_rng(0)
        self.arms = ghg.targets_np(self.phase, self.mpc.gp, t_since_start=0.0)[1]

    def standing_pose(self) -> np.ndarray:
        return self.mpc.standing_pose().astype(np.float32)

    def step(self, s: BrainState, dt: float) -> np.ndarray:
        d = self.d
        # --- reconstruct the MuJoCo sidecar state from what the deploy measured ---
        d.qpos[0:3] = (0.0, 0.0, s.bz)             # absolute x/y irrelevant to MPC
        d.qpos[3:7] = _quat_from_rpy(s.roll, s.pitch, s.yaw)
        d.qpos[self.plant.leg_qadr] = np.asarray(s.joint_q, dtype=np.float64)
        d.qpos[self.plant.arm_qadr] = _sim.ARM_NOMINAL
        d.qvel[0:3] = (s.vx, s.vy, s.vz)
        d.qvel[3:6] = (s.roll_rate, s.pitch_rate, s.yaw_rate)
        d.qvel[self.plant.leg_dadr] = np.asarray(s.joint_qd, dtype=np.float64)
        d.qvel[self.arm_dadr] = 0.0
        mujoco.mj_forward(self.m, d)
        # --- plan against the sidecar, fold onto the ghost gait ---
        if self._nstep < 8:
            import time as _t
            _t0 = _t.perf_counter()
            r0 = self.mpc.plan(self.phase, max(0.0, s.t), self.rng)
            self._log(f"  step {self._nstep}: plan {_t.perf_counter()-_t0:.3f}s "
                      f"bz={s.bz:.3f} roll={s.roll:+.3f} pitch={s.pitch:+.3f}")
        else:
            r0 = self.mpc.plan(self.phase, max(0.0, s.t), self.rng)
        self._nstep += 1
        legs, self.arms, _ = ghg.targets_np(self.phase, self.mpc.gp,
                                            t_since_start=max(0.0, s.t))
        tgt = np.clip(legs.astype(np.float64), LIM_LO, LIM_HI)
        tgt[RES_J] += r0
        np.clip(tgt, LIM_LO, LIM_HI, out=tgt)
        self.phase = (self.phase + self.mpc.om * dt) % (2.0 * math.pi)
        return tgt.astype(np.float32)
