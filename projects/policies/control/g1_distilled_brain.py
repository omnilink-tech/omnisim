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

"""g1_distilled_brain.py -- REAL-TIME deploy brain: the MPC distilled into a
fast policy. Drop-in for the g1_walk_deploy G1_BRAIN hook:

    G1_BRAIN=1  G1_BRAIN_MODULE=projects.policies.control.g1_distilled_brain

The policy (trained by _scratch/distill_train.py from MPC teacher data) maps the
state to the MPC's 8-dim balance residual via a tiny numpy MLP -- microseconds per
tick, so OmniSim runs in REAL TIME (unlike the MPC, whose physics-rollout planning
is ~20x too slow). No torch / mujoco at runtime, just numpy.
"""
from __future__ import annotations
import math
import os
from dataclasses import dataclass

import numpy as np

from projects.policies.control.gait import g1_human_gait as ghg
from projects.policies.control.g1_brain import LIM_LO, LIM_HI
from projects.policies.control.g1_distill import make_obs, numpy_policy, RES_J


@dataclass
class BrainState:
    roll: float; pitch: float; yaw: float
    roll_rate: float; pitch_rate: float; yaw_rate: float
    vx: float; vy: float; vz: float; bz: float
    joint_q: np.ndarray; joint_qd: np.ndarray; t: float


@dataclass
class BrainConfig:
    vx: float = 0.4
    freq: float = 1.3
    x0: float = -0.03
    weights: str = ""
    @classmethod
    def from_env(cls, env=None):
        env = os.environ if env is None else env
        def _f(n, c):
            v = env.get(n); return float(v) if v not in (None, "") else c
        return cls(vx=_f("G1_GAIT_VX", cls.vx), freq=_f("G1_GAIT_FREQ", cls.freq),
                   x0=_f("G1_BRAIN_X0", cls.x0),
                   weights=env.get("G1_DISTILL_WEIGHTS", cls.weights))


class G1Brain:
    def __init__(self, cfg: BrainConfig | None = None):
        cfg = cfg or BrainConfig.from_env()
        path = cfg.weights or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "_scratch", "distill_weights.npz")
        z = np.load(path)
        self.w = {k: z[k] for k in z.files}
        if "n_layers" in self.w:
            self.w["n_layers"] = int(self.w["n_layers"])
        self.gp = ghg.GaitParams(style="ik", vx=cfg.vx, freq=cfg.freq, x0=cfg.x0)
        self.om = 2.0 * math.pi * self.gp.freq
        self.phase = ghg.DS_PHASE
        self._diag = os.environ.get("G1_DISTILL_DIAG", "")
        self._nstep = 0
        self._log(f"[g1_distilled_brain] LOADED policy {path} "
                  f"({self.w.get('n_layers','?')} layers)")

    def _log(self, msg):
        if not self._diag:
            return
        try:
            with open(self._diag, "a") as fh:
                fh.write(msg + "\n")
        except Exception:
            pass
        self.arms = ghg.targets_np(self.phase, self.gp, t_since_start=0.0)[1]

    def standing_pose(self) -> np.ndarray:
        return ghg.standing_pose(self.gp).astype(np.float32)

    def step(self, s: BrainState, dt: float) -> np.ndarray:
        obs = make_obs(s.roll, s.pitch, s.yaw, s.roll_rate, s.pitch_rate, s.yaw_rate,
                       s.vx, s.vy, s.vz, s.bz, s.joint_q, s.joint_qd, self.phase)
        r0 = numpy_policy(self.w, obs)                      # 8-dim residual, ~us
        if getattr(self, "_diag", "") and getattr(self, "_nstep", 0) < 6:
            self._nstep = getattr(self, "_nstep", 0) + 1
            self._log(f"  s{self._nstep} obs[vx,vy,vz,roll,pit,rr,pr,yr,bz]="
                      f"{np.round(obs[:9],3).tolist()} act={np.round(r0,3).tolist()}")
        legs, self.arms, _ = ghg.targets_np(self.phase, self.gp,
                                            t_since_start=max(0.0, s.t))
        tgt = np.clip(legs.astype(np.float64), LIM_LO, LIM_HI)
        tgt[RES_J] += r0
        np.clip(tgt, LIM_LO, LIM_HI, out=tgt)
        self.phase = (self.phase + self.om * dt) % (2.0 * math.pi)
        return tgt.astype(np.float32)
