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

"""Smoke test for SpotEnv: random-action rollout, sanity-check obs/reward.

Launches one SpotEnv, runs N random-action steps, prints stats.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from spot_env import SpotEnv, OBS_DIM, ACT_DIM


def main() -> int:
    print("[validate] launching SpotEnv (verbose)…")
    env = SpotEnv(env_id=0, verbose=True)
    t0 = time.time()
    obs, info = env.reset()
    t1 = time.time()
    print(f"[validate] reset done in {t1-t0:.2f}s, obs.shape={obs.shape} obs.dtype={obs.dtype}")
    assert obs.shape == (OBS_DIM,), obs.shape

    n_steps = 200
    rewards = []
    obs_buf = []
    dones = []
    rng = np.random.default_rng(0)
    t_step_start = time.time()
    for i in range(n_steps):
        # Small random actions to avoid blowing up the body immediately
        a = rng.uniform(-0.3, 0.3, size=(ACT_DIM,)).astype(np.float32)
        obs, reward, term, trunc, info = env.step(a)
        rewards.append(reward)
        obs_buf.append(obs.copy())
        dones.append(term or trunc)
        if term or trunc:
            print(f"[validate] terminated/truncated at step {i+1} term={term} trunc={trunc}")
            obs, info = env.reset()
    t_step_end = time.time()
    elapsed = t_step_end - t_step_start
    print(f"[validate] {n_steps} steps in {elapsed:.2f}s = {n_steps/elapsed:.0f} steps/sec")

    arr = np.stack(obs_buf)
    print(f"[validate] obs stats: mean={arr.mean():+.3f} std={arr.std():.3f} min={arr.min():+.3f} max={arr.max():+.3f}")
    print(f"[validate] reward stats: mean={np.mean(rewards):+.3f} std={np.std(rewards):.3f} sum={np.sum(rewards):+.3f}")
    print(f"[validate] termination rate: {np.mean(dones)*100:.1f}%")
    env.close()
    print("[validate] DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
