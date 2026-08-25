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

"""Eval push-recovery survival rate under random perturbations.

Runs N episodes through OmniQuadResidualEnv with periodic horizontal Δv
impulses injected by omniquad_residual_agent. For each episode, reports
episode length (truncated at MAX_EPISODE_STEPS=1024 ≈ 16.4s of sim
time). Compare the SAME script with --policy <onnx> vs --no-policy
(zero residual) to see how much the trained policy adds.

Example:
    # Baseline (model walker, no policy)
    python projects/policies/research/tools/eval_push_recovery.py \\
        --no-policy --dv-min 0.30 --dv-max 0.30 --n-episodes 10

    # Trained policy
    python projects/policies/research/tools/eval_push_recovery.py \\
        --policy projects/policies/research/inference/policies/omniquad_residual_perturb_main/policy.onnx \\
        --dv-min 0.30 --dv-max 0.30 --n-episodes 10
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "projects" / "rl" / "envs"))


def _load_onnx(path: Path):
    import onnxruntime as ort
    sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0].name
    def predict(obs: np.ndarray) -> np.ndarray:
        return sess.run(None, {inp: obs.astype(np.float32).reshape(1, -1)})[0].ravel()
    return predict


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, default=None,
                   help="ONNX policy. If omitted, runs with zero residual (model walker baseline).")
    p.add_argument("--no-policy", action="store_true",
                   help="Force zero residual even if --policy is set. Useful for A/B.")
    p.add_argument("--dv-min", type=float, default=0.20)
    p.add_argument("--dv-max", type=float, default=0.40)
    p.add_argument("--interval-s", type=float, default=2.5)
    p.add_argument("--start-s", type=float, default=2.0)
    p.add_argument("--n-episodes", type=int, default=5)
    p.add_argument("--world", type=Path,
                   default=REPO / "projects" / "rl" / "worlds"
                   / "omniquad_residual_train_perturb_newton.omniworld")
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    # Configure the Newton + perturbation env vars BEFORE OmniQuadResidualEnv
    # launches Webots — they're inherited by the spawned process.
    os.environ["OMNIQUAD_TRAIN_WORLD"] = str(args.world)
    os.environ["OMNISIM_URDF_USE_INERTIA"] = "1"
    os.environ["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    os.environ["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    os.environ.setdefault("OMNISIM_NEWTON_SEED_POSE", "1")
    os.environ.setdefault("OMNISIM_NEWTON_TARGET_KE", "250")
    os.environ.setdefault("OMNISIM_NEWTON_TARGET_KD", "60")
    os.environ.setdefault("OMNISIM_NEWTON_GROUND_MU", "2.0")
    os.environ["OMNIQUAD_PERTURB_DV_MIN"] = str(args.dv_min)
    os.environ["OMNIQUAD_PERTURB_DV_MAX"] = str(args.dv_max)
    os.environ["OMNIQUAD_PERTURB_INTERVAL_S"] = str(args.interval_s)
    os.environ["OMNIQUAD_PERTURB_START_S"] = str(args.start_s)

    use_policy = (args.policy is not None) and (not args.no_policy)
    if use_policy:
        if not args.policy.exists():
            print(f"missing: {args.policy}")
            return 2
        predict = _load_onnx(args.policy)
        label = args.policy.name
    else:
        predict = lambda obs: np.zeros(12, dtype=np.float32)
        label = "<zero residual (model walker)>"

    np.random.seed(args.seed)

    from omniquad_residual_env import OmniQuadResidualEnv
    env = OmniQuadResidualEnv(env_id=0, verbose=False)

    print(f"[eval_push_recovery] policy={label}")
    print(f"[eval_push_recovery] perturbations: dv in [{args.dv_min:.2f}, {args.dv_max:.2f}] m/s "
          f"every {args.interval_s}s (first at {args.start_s}s)")
    print(f"[eval_push_recovery] world={args.world.name}  episodes={args.n_episodes}")
    print()

    ep_lens = []
    t0 = time.time()
    try:
        obs, _ = env.reset()
        for ep in range(args.n_episodes):
            steps = 0
            done = False
            while not done:
                action = predict(obs)
                obs, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                steps += 1
            ep_lens.append(steps)
            sim_s = steps * 16 / 1000.0  # step_ms=16
            tag = "OK " if steps >= 1020 else "FELL"
            print(f"  episode {ep+1}: {tag}  steps={steps:4d}  sim_t={sim_s:.2f}s")
            if ep < args.n_episodes - 1:
                obs, _ = env.reset()
    finally:
        env.close()

    elapsed = time.time() - t0
    ep_lens_arr = np.array(ep_lens)
    survived = int((ep_lens_arr >= 1020).sum())
    print()
    print(f"[eval_push_recovery] survived: {survived}/{args.n_episodes}  "
          f"({100.0 * survived / args.n_episodes:.0f}%)")
    print(f"[eval_push_recovery] ep_len: mean={ep_lens_arr.mean():.0f}  "
          f"median={int(np.median(ep_lens_arr))}  min={ep_lens_arr.min()}  max={ep_lens_arr.max()}")
    print(f"[eval_push_recovery] wall {elapsed:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
