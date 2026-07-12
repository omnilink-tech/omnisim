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

"""Custom trainer for the residual Spot policy.

Tiny PPO on an 18-dim obs / 12-dim action space sitting on top of the
model walker. Doesn't share much with train_robot.py because the obs/
act shapes, network sizes, and reward semantics all differ -- and we
want a small dependency surface so this stays the algorithm we own.

Run from repo root:
    python projects/policies/research/training/train_residual.py --steps 50000 \
        --run-name spot_residual_v1
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))

RUNS_DIR = REPO_ROOT / "projects" / "rl" / "training" / "runs"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--steps", type=int, default=50_000)
    p.add_argument("--envs", type=int, default=1)
    p.add_argument("--n-steps", type=int, default=1024)
    p.add_argument("--batch-size", type=int, default=512)
    p.add_argument("--n-epochs", type=int, default=4)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip-range", type=float, default=0.2)
    p.add_argument("--ent-coef", type=float, default=0.003)
    p.add_argument("--target-kl", type=float, default=0.02)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--log-std-init", type=float, default=-1.5,
                   help="Default -1.5 (std=0.22). Residual policy starts "
                        "near zero so the model walker's behavior is "
                        "preserved before training kicks in.")
    p.add_argument("--ckpt-every", type=int, default=5_000)
    args = p.parse_args()

    run_dir = RUNS_DIR / args.run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"[train_residual] run={args.run_name}  steps={args.steps:,}  "
          f"envs={args.envs}  lr={args.lr}")
    for k in sorted(os.environ):
        if k.startswith("SPOT_"):
            print(f"[train_residual] env {k}={os.environ[k]}")

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
    from stable_baselines3.common.callbacks import CheckpointCallback

    from spot_residual_env import SpotResidualEnv

    def make_env(eid):
        return lambda: SpotResidualEnv(env_id=eid, verbose=False)

    vec_env = DummyVecEnv([make_env(i) for i in range(args.envs)])
    vec_env = VecMonitor(vec_env, filename=str(run_dir / "monitor.csv"))

    policy_kwargs = dict(
        net_arch=[64, 64],
        activation_fn=torch.nn.Tanh,
        log_std_init=args.log_std_init,
    )

    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        learning_rate=args.lr,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        target_kl=args.target_kl,
        max_grad_norm=args.max_grad_norm,
        policy_kwargs=policy_kwargs,
        verbose=1,
        tensorboard_log=str(run_dir / "tb"),
        device="cpu",
    )

    ckpt_cb = CheckpointCallback(
        save_freq=max(1, args.ckpt_every // max(1, args.envs)),
        save_path=str(run_dir / "checkpoints"),
        name_prefix="spot_res",
    )

    start = time.time()
    try:
        model.learn(total_timesteps=args.steps, callback=ckpt_cb,
                    progress_bar=False)
    except KeyboardInterrupt:
        print("[train_residual] interrupted; saving partial model")

    final = run_dir / "spot_res_final.zip"
    model.save(str(final))
    elapsed = time.time() - start
    print(f"[train_residual] saved {final}")
    print(f"[train_residual] elapsed {elapsed:.1f}s "
          f"({args.steps/max(elapsed, 1e-3):.0f} steps/s)")

    vec_env.close()
    return 0


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    sys.exit(main())
