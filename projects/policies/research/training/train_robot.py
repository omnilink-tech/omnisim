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

"""Generic RL trainer — train any registered OmniSim robot with any backend.

Run from repo root:

    # default (SB3, slow, always works)
    python projects/policies/research/training/train_robot.py --robot omniquad --backend sb3 \
        --envs 4 --steps 500000

    # MJX (GPU-batched, ~1000x faster on Linux/WSL2 with cuda jaxlib)
    python projects/policies/research/training/train_robot.py --robot omniquad --backend mjx \
        --envs 1024 --steps 10000000

    # Train a different robot (must be registered in robot_registry.py)
    python projects/policies/research/training/train_robot.py --robot anymal --backend mjx ...

    # See what's available
    python projects/policies/research/training/train_robot.py --backend list
    python projects/policies/research/training/train_robot.py --robot list

All backends emit the same ONNX schema. The OmniSim generic deploy
controller (projects/policies/controllers/rl_deploy/) loads the ONNX + the
matching robot_spec.json sidecar (auto-written by the backend during
export) and runs the policy without further configuration.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))

from projects.policies.research.backends import list_backends, get_backend
from projects.policies.research.backends.robot_registry import list_robots, get_robot
from projects.policies.research.backends.base import TrainingConfig


RUNS_DIR = REPO_ROOT / "projects" / "rl" / "training" / "runs"
ONNX_DIR = REPO_ROOT / "projects" / "rl" / "inference" / "policies"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="omniquad",
                        help="registered robot name (use 'list' to print)")
    parser.add_argument("--backend", default="sb3",
                        help="training backend (use 'list' to print)")
    parser.add_argument("--reward-recipe", default=None,
                        help="override the robot's default reward recipe "
                             "(legged_locomotion, manipulation, balance, ...)"
                             "; the backend's reward function reads this.")
    parser.add_argument("--envs", type=int, default=4, help="parallel envs")
    parser.add_argument("--steps", type=int, default=300_000, help="total timesteps")
    parser.add_argument("--run-name", type=str, default=None,
                        help="output subdir under runs/ (default: timestamp)")
    parser.add_argument("--n-steps", type=int, default=512,
                        help="PPO rollout buffer per env per update")
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--n-epochs", type=int, default=4)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--ent-coef", type=float, default=0.005)
    parser.add_argument("--vf-coef", type=float, default=0.5)
    parser.add_argument("--device", default="auto",
                        help="'cpu', 'cuda', or 'auto'")
    parser.add_argument("--single", action="store_true",
                        help="(sb3 only) use DummyVecEnv instead of subproc")
    parser.add_argument("--ckpt-every", type=int, default=50_000)
    parser.add_argument("--no-export-onnx", action="store_true",
                        help="skip auto-ONNX-export after training")
    parser.add_argument("--log-std-init", type=float, default=None,
                        help="initial log-std for the policy's action "
                             "distribution. Default -1.0 (std=0.37); use 0.5 "
                             "(std=1.6) for aggressive walk-discovery on a "
                             "CPG-warm-started controller.")
    parser.add_argument("--target-kl", type=float, default=None,
                        help="early-stop a PPO update epoch when its KL "
                             "from the rollout policy exceeds this. Use "
                             "~0.02 to protect against catastrophic policy "
                             "shifts that destroy a working walker.")
    parser.add_argument("--max-grad-norm", type=float, default=None,
                        help="override gradient norm clip (default 1.0).")
    args = parser.parse_args()

    if args.backend == "list":
        avail = list_backends()
        print("OmniSim RL backends:")
        for name, ok in avail.items():
            print(f"  {name:8s}  {'available' if ok else 'missing deps / install'}")
        return 0
    if args.robot == "list":
        print("OmniSim registered robots:")
        for name in list_robots():
            print(f"  {name}")
        return 0

    try:
        robot = get_robot(args.robot)
    except (ValueError, ImportError) as e:
        print(f"[train_robot] ERROR: {e}")
        return 2

    if args.reward_recipe:
        # Override (mutate a copy by replacing the attribute).
        robot = robot.__class__(**{**robot.__dict__, "obs_recipe": args.reward_recipe})
        print(f"[train_robot] obs_recipe override: {robot.obs_recipe}")

    run_name = args.run_name or time.strftime(f"{robot.name}_{args.backend}_%Y%m%d_%H%M%S")
    run_dir = RUNS_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"[train_robot] robot={robot.name} backend={args.backend} run_dir={run_dir}")

    backend = get_backend(args.backend)
    if not backend.is_available():
        print(f"[train_robot] backend {args.backend!r} not available. Required deps:")
        if args.backend == "mjx":
            print("  pip install jax jaxlib mujoco-mjx flax optax")
        elif args.backend == "isaac":
            print("  Install Isaac Sim + Isaac Lab "
                  "(https://isaac-sim.github.io/IsaacLab/)")
            print("  Then set ISAAC_LAB_PATH=/path/to/IsaacLab")
        return 2

    extra_cfg = {}
    if args.log_std_init is not None:
        extra_cfg["log_std_init"] = args.log_std_init
    if args.target_kl is not None:
        extra_cfg["target_kl"] = args.target_kl
    if args.max_grad_norm is not None:
        extra_cfg["max_grad_norm"] = args.max_grad_norm
    config = TrainingConfig(
        total_steps=args.steps,
        parallel_envs=args.envs,
        n_steps_per_env=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        learning_rate=args.lr,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        vf_coef=args.vf_coef,
        device=args.device,
        run_name=run_name,
        checkpoint_every_steps=args.ckpt_every,
        **extra_cfg,
        # Reward weights from env vars (the walk-focused trainer uses these).
        r_lin_vel_wt=float(os.environ.get("OMNIQUAD_R_LIN_VEL_WT", 1.0)),
        r_ang_vel_wt=float(os.environ.get("OMNIQUAD_R_ANG_VEL_WT", 0.5)),
        r_vz_wt=float(os.environ.get("OMNIQUAD_R_VZ_WT", -1.0)),
        r_rollpitch_wt=float(os.environ.get("OMNIQUAD_R_ROLLPITCH_WT", -0.1)),
        r_ang_xy_wt=float(os.environ.get("OMNIQUAD_R_ANG_XY_WT", -0.02)),
        r_action_rate_wt=float(os.environ.get("OMNIQUAD_R_ACTION_RATE_WT", -0.005)),
        r_joint_acc_wt=float(os.environ.get("OMNIQUAD_R_JOINT_ACC_WT", -2.5e-7)),
        r_alive_bonus=float(os.environ.get("OMNIQUAD_R_ALIVE_BONUS", 0.05)),
        r_vx_bonus_wt=float(os.environ.get("OMNIQUAD_R_VX_BONUS_WT", 0.0)),
        r_vx_deadband=float(os.environ.get("OMNIQUAD_R_VX_DEADBAND", 0.05)),
    )

    print(f"[train_robot] training for {args.steps:,} timesteps via '{backend.name}' ...")
    start = time.time()
    ckpt = backend.train(robot=robot, config=config, out_dir=run_dir)
    print(f"[train_robot] trained checkpoint: {ckpt}")
    print(f"[train_robot] elapsed: {time.time() - start:.1f} s")

    if not args.no_export_onnx:
        onnx_path = ONNX_DIR / run_name / "policy.onnx"
        try:
            backend.export_onnx(ckpt, onnx_path, robot)
            print(f"[train_robot] ONNX:  {onnx_path}")
            print(f"[train_robot] spec:  {onnx_path.parent / 'robot_spec.json'}")
            print(f"[train_robot] deploy:")
            print(f"    OMNISIM_POLICY_ONNX={onnx_path} \\")
            print(f"      omnisim-bin.exe projects/policies/worlds/<robot>_rl_deploy.wbt")
        except Exception as e:
            print(f"[train_robot] ONNX export failed ({e}); checkpoint kept")
            return 1
    return 0


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    sys.exit(main())
