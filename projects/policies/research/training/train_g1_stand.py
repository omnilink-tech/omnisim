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

"""SB3 PPO trainer for the G1 standing env.

Goal: learn a policy that keeps the legs-only G1 upright for 500 env
steps (≈4 s sim time) starting from a slightly perturbed NOMINAL_POSE.
Target survival rate ≥ 95 % of the episode budget. Once reached, this
becomes the analytic baseline that an OmniSim residual deploy
controller layers a Newton-physics correction on top of.

Single-env CPU MuJoCo; SB3 PPO. Typical convergence under this reward
shape is a few hundred thousand env steps (~5-20 min wall clock on a
single laptop CPU). The Spot residual_main policy needed 20 k steps,
but that was residual-on-top-of-a-walking-baseline; for from-scratch
biped standing the budget is larger.

Outputs:
  projects/policies/research/training/runs/g1_stand/policy.zip   (SB3 SavedModel)
  projects/policies/research/training/runs/g1_stand/policy.onnx  (deployable)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv  # noqa: E402

from projects.policies.research.envs.g1_stand_env import G1StandEnv  # noqa: E402


RUN_DIR = REPO / "projects/policies/research/training/runs/g1_stand"


class StandStatsCallback(BaseCallback):
    """Logs mean episode length / reward to stdout every N steps."""

    def __init__(self, every: int = 5000):
        super().__init__(verbose=0)
        self._every = every
        self._last = 0
        self._ep_returns: list[float] = []
        self._ep_lengths: list[int] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            ep = info.get("episode")
            if ep is not None:
                self._ep_returns.append(float(ep["r"]))
                self._ep_lengths.append(int(ep["l"]))
        if self.num_timesteps - self._last >= self._every:
            self._last = self.num_timesteps
            if self._ep_returns:
                last = self._ep_returns[-50:]
                last_l = self._ep_lengths[-50:]
                print(f"[train] t={self.num_timesteps:>7d}  "
                      f"ep_return_mean={float(np.mean(last)):+7.2f}  "
                      f"ep_len_mean={float(np.mean(last_l)):6.1f}")
            else:
                print(f"[train] t={self.num_timesteps:>7d}  (no completed episodes yet)")
        return True


def export_onnx(model: PPO, env: G1StandEnv, out_path: Path) -> None:
    """Export the policy network to ONNX for deploy.

    SB3's PPO has a `policy` module exposing `forward(obs)` returning
    actions, values, log_prob. We wrap the action head only for deploy.
    """
    class DeployPolicy(torch.nn.Module):
        def __init__(self, sb3_policy):
            super().__init__()
            self.sb3 = sb3_policy

        def forward(self, obs):  # type: ignore[override]
            # SB3 PPO MlpPolicy: features = extractor(obs); mean = mlp(features)
            # deterministic action = mean
            features = self.sb3.extract_features(obs)
            latent_pi = self.sb3.mlp_extractor.forward_actor(features)
            mean = self.sb3.action_net(latent_pi)
            return torch.tanh(mean)  # squash to [-1, 1] like the action space

    wrapped = DeployPolicy(model.policy)
    wrapped.eval()
    dummy_obs = torch.zeros(1, env.observation_space.shape[0], dtype=torch.float32)
    torch.onnx.export(
        wrapped,
        dummy_obs,
        str(out_path),
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17,
    )
    print(f"[train] exported ONNX -> {out_path} ({out_path.stat().st_size} bytes)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=200_000,
                        help="Total training timesteps (default 200k).")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=str, default=str(RUN_DIR))
    parser.add_argument("--n-steps", type=int, default=2048,
                        help="PPO n_steps (rollout length).")
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--n-envs", type=int, default=1,
                        help="Parallel env count. >1 uses SubprocVecEnv.")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.n_envs > 1:
        # Per-worker seed so each env explores from a different starting
        # state — otherwise SubprocVecEnv's identical seed across workers
        # gives N identical trajectories.
        env_fns = [
            (lambda i=i: Monitor(G1StandEnv(seed=args.seed + i)))
            for i in range(args.n_envs)
        ]
        venv = SubprocVecEnv(env_fns, start_method="spawn")
    else:
        env_fns = [lambda: Monitor(G1StandEnv(seed=args.seed))]
        venv = DummyVecEnv(env_fns)

    model = PPO(
        "MlpPolicy",
        venv,
        learning_rate=args.lr,
        n_steps=args.n_steps,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=dict(net_arch=dict(pi=[256, 128], vf=[256, 128])),
        seed=args.seed,
        verbose=0,
    )

    t0 = time.time()
    callback = StandStatsCallback(every=5000)
    model.learn(total_timesteps=args.steps, callback=callback, log_interval=1)
    elapsed = time.time() - t0
    print(f"[train] training done in {elapsed:.1f} s "
          f"({args.steps / elapsed:.0f} env-steps/s)")

    zip_path = out_dir / "policy.zip"
    model.save(str(zip_path))
    print(f"[train] saved policy -> {zip_path}")

    env = G1StandEnv(seed=args.seed + 999)

    # Eval first so failure of ONNX export below doesn't hide
    # whether the policy itself works.
    obs, _ = env.reset()
    total_r = 0.0
    steps = 0
    for _ in range(500):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, _ = env.step(action)
        total_r += r
        steps += 1
        if term or trunc:
            break
    print(f"[train] eval: survived {steps} steps, total reward {total_r:+.2f}")

    # ONNX export — PyTorch on Windows prints emoji that the cp1252
    # console can't encode, which raises before the file is written.
    # Redirect torch's verbose printer to stdout-with-replace.
    try:
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            export_onnx(model, env, out_dir / "policy.onnx")
        # Print the captured (now harmless) output, replacing un-encodable chars.
        sys.stdout.write(buf.getvalue().encode("ascii", "replace").decode("ascii"))
    except Exception as e:
        print(f"[train] ONNX export failed (non-fatal): {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
