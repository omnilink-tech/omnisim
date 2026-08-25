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

"""Sanity-check the CPG trot prior in isolation.

Drives the OmniSim env with all-zero policy actions for ~10 seconds, so
only the CPG base offsets move the joints. Reports forward velocity,
distance, upright fraction, and termination cause.

Goals when this passes:
  * upright > 80% (CPG alone keeps the body up)
  * mean_vx > 0 (trot produces SOME forward drift; doesn't need to be fast)
  * no immediate termination (>5s of survival)

If those hold, PPO has a much easier residual-learning task than from
scratch — the policy starts from a non-trivial walking gait and only
has to refine.

Usage (from repo root):
    OMNIQUAD_CPG_FREQ_HZ=1.5 OMNIQUAD_CPG_HIP_Y_AMP=0.20 OMNIQUAD_CPG_KNEE_AMP=0.30 \
        python projects/policies/research/tools/cpg_sanity.py
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))

from omniquad_env import OmniQuadEnv, OBS_DIM, ACT_DIM


def main() -> int:
    os.environ.setdefault("OMNIQUAD_CPG_FREQ_HZ", "1.5")
    os.environ.setdefault("OMNIQUAD_CPG_HIP_Y_AMP", "0.20")
    os.environ.setdefault("OMNIQUAD_CPG_KNEE_AMP", "0.30")
    os.environ.setdefault("OMNIQUAD_FIXED_VX", "0.5")
    os.environ.setdefault("OMNIQUAD_FIXED_WZ", "0.0")

    duration_s = 10.0
    print(f"[cpg_sanity] launching env, will drive zero actions for {duration_s}s")
    env = OmniQuadEnv(env_id=0, verbose=False)
    t_reset0 = time.time()
    obs, _ = env.reset()
    print(f"[cpg_sanity] reset done in {time.time()-t_reset0:.1f}s")

    step_dt = 0.016
    n_steps = int(duration_s / step_dt)
    velocities = []
    obss = []
    terminated_at = None
    rewards = []
    for i in range(n_steps):
        a = np.zeros(ACT_DIM, dtype=np.float32)
        obs, rew, term, trunc, _ = env.step(a)
        velocities.append(obs[0:3].copy())
        obss.append(obs.copy())
        rewards.append(rew)
        if term or trunc:
            terminated_at = i + 1
            break
    env.close()

    if not velocities:
        print("[cpg_sanity] no steps run, abort")
        return 1
    v = np.array(velocities)
    o = np.array(obss)
    proj_g_z = o[:, 8]
    upright = (proj_g_z < -0.7).mean() * 100.0
    mean_vx = float(v[:, 0].mean())
    mean_vy = float(v[:, 1].mean())
    mean_vz = float(v[:, 2].mean())
    dist_x = float(np.cumsum(v[:, 0]).sum()) * step_dt
    print(f"[cpg_sanity] ran {len(v)} steps  terminated_at={terminated_at}")
    print(f"[cpg_sanity] mean v: vx={mean_vx:+.3f} vy={mean_vy:+.3f} vz={mean_vz:+.3f}")
    print(f"[cpg_sanity] distance_x={dist_x:+.3f} m")
    print(f"[cpg_sanity] upright={upright:.1f}%   mean_reward={np.mean(rewards):+.3f}")
    ok = upright > 60.0 and terminated_at is None
    print(f"[cpg_sanity] CPG stable in isolation? {ok}")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
