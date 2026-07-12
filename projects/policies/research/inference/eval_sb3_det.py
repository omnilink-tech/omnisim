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

"""Deterministic eval of an SB3 PPO checkpoint against SpotEnv.

Training ep_len_mean is measured on STOCHASTIC rollouts (the policy samples
actions ~N(mu, std), std~0.37). That exploration noise constantly kicks the
gait and causes falls, so training ep_len UNDER-estimates the deployed
(deterministic, mean-action) policy. This runs the policy with
deterministic=True and reports the real survival length.

Set the Newton physics env (kd, trim, friction, CPG) the same way the run
used -- this script does NOT set them, so export them or run via the same
shell that launched training.

Run from repo root:
    OMNISIM_NEWTON_TARGET_KD=60 SPOT_PITCH_TRIM=0.20 SPOT_FIXED_VX=0.5 ... \
      python projects/policies/research/inference/eval_sb3_det.py \
        --ckpt projects/policies/research/training/runs/spot_newton_v31_kd60/checkpoints/spot_120000_steps.zip \
        --episodes 5
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", type=Path, required=True)
    p.add_argument("--episodes", type=int, default=5)
    p.add_argument("--max-steps", type=int, default=1024)
    args = p.parse_args()

    import numpy as np
    from stable_baselines3 import PPO
    from spot_env import SpotEnv

    env = SpotEnv(env_id=0, verbose=False)
    model = PPO.load(str(args.ckpt), device="cpu")
    print(f"[det-eval] {args.ckpt.name}  episodes={args.episodes}")

    lens, dxs = [], []
    for ep in range(args.episodes):
        obs, _ = env.reset()
        steps = 0
        vxs = []
        for steps in range(1, args.max_steps + 1):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            # obs[0:3] = body linear velocity; obs[0] = forward (vx).
            vxs.append(float(obs[0]))
            if terminated or truncated:
                break
        # mean vx over the steady portion (skip the first 20-step settle)
        mean_vx = float(np.mean(vxs[20:])) if len(vxs) > 20 else float(np.mean(vxs))
        lens.append(steps)
        dxs.append(mean_vx)
        print(f"  ep{ep}: len={steps:4d}  mean_vx={mean_vx:+.3f} m/s  "
              f"(dist~{mean_vx*steps*0.032:+.2f} m @32ms/step)")
    env.close()
    print(f"[det-eval] mean len={np.mean(lens):.0f}  max={max(lens)}  "
          f"mean vx={np.mean(dxs):+.3f} m/s")
    return 0


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    sys.exit(main())
