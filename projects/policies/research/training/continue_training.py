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

"""Continue a previous SB3 PPO training run from a saved checkpoint.

Loads a .zip checkpoint, swaps in the current environment (which reads
its reward weights, CPG params, etc. from env vars), and trains for
additional steps. Saves checkpoints to a new run dir so the original
isn't overwritten.

This is the canonical way to push a stable-but-slow walker toward
faster motion — start from a known-good policy and gently shift
the reward gradient.

Run from repo root:
    SPOT_R_VX_BONUS_WT=12.0 SPOT_R_VZ_WT=-0.2 ... \
        python projects/policies/research/training/continue_training.py \
            --from projects/policies/research/training/runs/spot_walk_v11/checkpoints/spot_30000_steps.zip \
            --run-name spot_walk_v12 \
            --steps 100000 \
            --lr 1e-4 --target-kl 0.02 --max-grad-norm 0.3 \
            --ckpt-every 5000
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--from", dest="from_zip", type=Path, required=True,
                   help="path to a previous SB3 .zip checkpoint")
    p.add_argument("--run-name", required=True)
    p.add_argument("--steps", type=int, default=100_000)
    p.add_argument("--envs", type=int, default=1)
    p.add_argument("--n-steps", type=int, default=2048)
    p.add_argument("--batch-size", type=int, default=512)
    p.add_argument("--n-epochs", type=int, default=10)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--ent-coef", type=float, default=0.005)
    p.add_argument("--target-kl", type=float, default=None)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--ckpt-every", type=int, default=10_000)
    p.add_argument("--vf-warmup-steps", type=int, default=0,
                   help="If >0, first train ONLY the value function (actor "
                        "frozen) for this many steps before normal training. "
                        "Fixes the warm-start value shock: a loaded policy's "
                        "critic is calibrated to the OLD dynamics/reward, so "
                        "its advantages are garbage and the first PPO update "
                        "wrecks the gait. Warming the critic first gives valid "
                        "advantages before the actor moves.")
    args = p.parse_args()

    run_dir = REPO_ROOT / "projects" / "rl" / "training" / "runs" / args.run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"[continue] warm-starting from {args.from_zip}")
    print(f"[continue] run_dir = {run_dir}")
    print(f"[continue] steps={args.steps:,}  envs={args.envs}")
    print(f"[continue] lr={args.lr}  target_kl={args.target_kl}  "
          f"max_grad_norm={args.max_grad_norm}")
    # Echo back the reward weights so the log captures what was used.
    for k in sorted(os.environ):
        if k.startswith("SPOT_R_") or k.startswith("SPOT_CPG_") or k.startswith("SPOT_FIXED_"):
            print(f"[continue] env {k}={os.environ[k]}")

    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
    from stable_baselines3.common.callbacks import CheckpointCallback

    sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))
    from spot_env import SpotEnv

    def make_env(env_id):
        return lambda: SpotEnv(env_id=env_id, verbose=False)
    factories = [make_env(i) for i in range(args.envs)]
    vec_env = DummyVecEnv(factories)
    vec_env = VecMonitor(vec_env, filename=str(run_dir / "monitor.csv"))

    model = PPO.load(
        str(args.from_zip),
        env=vec_env,
        device="cpu",
        custom_objects={
            "learning_rate": args.lr,
            "n_steps": args.n_steps,
            "batch_size": args.batch_size,
            "n_epochs": args.n_epochs,
            "ent_coef": args.ent_coef,
            "target_kl": args.target_kl,
            "max_grad_norm": args.max_grad_norm,
            "clip_range": 0.2,
        },
    )
    model.tensorboard_log = str(run_dir / "tb")
    print(f"[continue] loaded model; existing num_timesteps={model.num_timesteps}")

    ckpt_cb = CheckpointCallback(
        save_freq=max(1, args.ckpt_every // max(1, args.envs)),
        save_path=str(run_dir / "checkpoints"),
        name_prefix="spot",
    )

    def _is_critic(name: str) -> bool:
        return ("mlp_extractor.value_net" in name) or name.startswith("value_net")

    start = time.time()
    try:
        if args.vf_warmup_steps > 0:
            import torch
            full_opt = model.policy.optimizer
            critic_params = [p for n, p in model.policy.named_parameters() if _is_critic(n)]
            actor_params = [p for n, p in model.policy.named_parameters() if not _is_critic(n)]
            # Swap in an optimizer over the CRITIC params only. The actor
            # still receives gradients from the PPO loss but is never
            # stepped, so the gait is perfectly preserved while the critic
            # learns valid Newton-value estimates. NOTE: requires_grad=False
            # alone does NOT freeze here -- Adam keeps applying stale
            # momentum to the "frozen" actor tensors and the policy drifts
            # (observed: a "frozen" warm-up still collapsed ep_len 487->76).
            # A critic-only optimizer truly holds the actor.
            model.policy.optimizer = torch.optim.Adam(critic_params, lr=args.lr)
            print(f"[continue] value-warmup: {args.vf_warmup_steps:,} steps, "
                  f"optimizer over {len(critic_params)} critic tensors only "
                  f"({len(actor_params)} actor tensors held). "
                  f"Watch explained_variance climb toward 1.0 with ep_len steady.")
            model.learn(
                total_timesteps=args.vf_warmup_steps,
                callback=None,
                progress_bar=False,
                reset_num_timesteps=False,
            )
            model.policy.optimizer = full_opt
            print("[continue] value-warmup done; restored full optimizer, starting fine-tune")
        model.learn(
            total_timesteps=args.steps,
            callback=ckpt_cb,
            progress_bar=False,
            reset_num_timesteps=False,  # keep counter; useful for tensorboard
        )
    except KeyboardInterrupt:
        print("[continue] interrupted; saving partial model")

    final = run_dir / "spot_final.zip"
    model.save(str(final))
    elapsed = time.time() - start
    print(f"[continue] saved {final}")
    print(f"[continue] elapsed {elapsed:.1f}s "
          f"({args.steps/max(elapsed,1e-3):.0f} steps/s)")
    vec_env.close()
    return 0


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    sys.exit(main())
