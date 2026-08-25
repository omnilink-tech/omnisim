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

"""SB3 CPU backend — the original OmniSim RL training path.

Wraps stable-baselines3 PPO + SubprocVecEnv of OmniSim subprocess envs.
This is the slow, always-works baseline. ~150-500 steps/s aggregate.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

from .base import RobotSpec, TrainingConfig

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


class Sb3CpuBackend:
    name = "sb3"

    def is_available(self) -> bool:
        try:
            import stable_baselines3  # noqa: F401
            return True
        except ImportError:
            return False

    def train(self, robot, config, out_dir):
        # Import here so the dep is lazy.
        import numpy as np
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import (
            SubprocVecEnv, DummyVecEnv, VecMonitor,
        )
        from stable_baselines3.common.callbacks import CheckpointCallback

        # The OmniQuadEnv lives in projects/policies/research/envs/omniquad_env.py. For
        # non-OmniQuad robots, a similar gym env should be plugged in.
        sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))
        from omniquad_env import OmniQuadEnv

        # OmniQuad is the only robot the sb3 backend supports today (the gym
        # env subprocess is OmniQuad-specific). Other robots can train via
        # mjx by pointing to their own URDF + RobotSpec.
        if robot.name != "omniquad":
            raise NotImplementedError(
                f"sb3 backend currently wraps OmniQuadEnv; {robot.name!r} "
                "needs its own OmniSim subprocess env. The mjx backend "
                "is robot-agnostic via URDF -> MJCF conversion."
            )

        out_dir.mkdir(parents=True, exist_ok=True)

        # Reward weights -> env vars (consumed by omniquad_rl_agent.py).
        env_vars = {
            "OMNIQUAD_R_LIN_VEL_WT": str(config.r_lin_vel_wt),
            "OMNIQUAD_R_ANG_VEL_WT": str(config.r_ang_vel_wt),
            "OMNIQUAD_R_VZ_WT": str(config.r_vz_wt),
            "OMNIQUAD_R_ROLLPITCH_WT": str(config.r_rollpitch_wt),
            "OMNIQUAD_R_ANG_XY_WT": str(config.r_ang_xy_wt),
            "OMNIQUAD_R_ACTION_RATE_WT": str(config.r_action_rate_wt),
            "OMNIQUAD_R_JOINT_ACC_WT": str(config.r_joint_acc_wt),
            "OMNIQUAD_R_ALIVE_BONUS": str(config.r_alive_bonus),
        }
        os.environ.update(env_vars)

        def make_env(env_id):
            return lambda: OmniQuadEnv(env_id=env_id, verbose=False)

        factories = [make_env(i) for i in range(config.parallel_envs)]
        # DummyVecEnv on Windows: SubprocVecEnv has been flaky on this
        # platform (multiprocessing handle DuplicateHandle access denied).
        if sys.platform == "win32" or config.parallel_envs == 1:
            vec_env = DummyVecEnv(factories)
        else:
            vec_env = SubprocVecEnv(factories, start_method="spawn")
        vec_env = VecMonitor(vec_env, filename=str(out_dir / "monitor.csv"))

        policy_kwargs = dict(
            net_arch=list(config.net_arch),
            activation_fn=torch.nn.Tanh,
            log_std_init=config.log_std_init,
        )

        model = PPO(
            policy="MlpPolicy",
            env=vec_env,
            learning_rate=config.learning_rate,
            n_steps=config.n_steps_per_env,
            batch_size=config.batch_size,
            n_epochs=config.n_epochs,
            gamma=config.gamma,
            gae_lambda=config.gae_lambda,
            clip_range=config.clip_range,
            ent_coef=config.ent_coef,
            vf_coef=config.vf_coef,
            max_grad_norm=config.max_grad_norm,
            target_kl=config.target_kl,
            policy_kwargs=policy_kwargs,
            verbose=1,
            tensorboard_log=str(out_dir / "tb"),
            device=config.device,
        )

        ckpt_cb = CheckpointCallback(
            save_freq=max(1, config.checkpoint_every_steps // max(1, config.parallel_envs)),
            save_path=str(out_dir / "checkpoints"),
            name_prefix=robot.name,
            save_replay_buffer=False,
            save_vecnormalize=False,
        )

        start = time.time()
        try:
            model.learn(
                total_timesteps=config.total_steps,
                callback=ckpt_cb,
                progress_bar=False,
            )
        except KeyboardInterrupt:
            print("[sb3] interrupted; saving partial model")

        ckpt = out_dir / f"{robot.name}_final.zip"
        model.save(str(ckpt))
        elapsed = time.time() - start
        print(f"[sb3] elapsed {elapsed:.1f} s ({config.total_steps/max(elapsed,1e-3):.0f} steps/s)")

        vec_env.close()
        return ckpt

    def export_onnx(self, ckpt, out, robot):
        # Reuse the existing onnx exporter logic.
        sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "inference"))
        from export_onnx import OnnxablePolicy  # type: ignore
        import torch
        import numpy as np
        from stable_baselines3 import PPO

        model = PPO.load(str(ckpt), device="cpu")
        wrapper = OnnxablePolicy(model.policy)
        wrapper.eval()
        out.parent.mkdir(parents=True, exist_ok=True)
        dummy = torch.zeros(1, robot.obs_dim, dtype=torch.float32)
        torch.onnx.export(
            wrapper, dummy, str(out),
            input_names=["obs"], output_names=["action"],
            dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
            opset_version=14,
        )
        spec_path = robot.write_sidecar(out.parent)
        print(f"[sb3] wrote ONNX {out}")
        print(f"[sb3] wrote spec sidecar {spec_path}")


def factory() -> Sb3CpuBackend:
    return Sb3CpuBackend()
