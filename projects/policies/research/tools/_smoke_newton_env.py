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

"""Smoke test: spawn one SpotEnv under Newton physics and step it.

This is the prerequisite for kicking off PPO training under Newton --
if SpotEnv can't reset/step against spot_rl_newton.wbt, training won't
work either.

Sets SPOT_TRAIN_WORLD to the Newton training world plus the Newton
opt-in env vars (URDF inertia, force-MuJoCo, wrapper-uses-own-shape).
Resets, runs 50 random-action steps, prints obs / reward / done at
each step, exits.

Run from repo root:
    python projects/policies/research/tools/_smoke_newton_env.py
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))


def main() -> int:
    # Newton opt-ins. These must be in the process env BEFORE SpotEnv
    # spawns Webots so the subproc inherits them.
    os.environ["SPOT_TRAIN_WORLD"] = str(REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl_newton.wbt")
    os.environ["OMNISIM_URDF_USE_INERTIA"] = "1"
    os.environ["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    os.environ["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    os.environ.setdefault("OMNISIM_NEWTON_LOG", str(REPO_ROOT / ".build_tmp" / "newton_solver.log"))
    os.environ["PYTHONIOENCODING"] = "utf-8"

    print(f"[smoke] world: {os.environ['SPOT_TRAIN_WORLD']}")

    import numpy as np
    from spot_env import SpotEnv, OBS_DIM, ACT_DIM

    env = SpotEnv(env_id=42, verbose=True)
    print("[smoke] env constructed; calling reset()...")
    t0 = time.time()
    obs, _info = env.reset()
    print(f"[smoke] reset() returned in {time.time()-t0:.1f}s; obs shape={obs.shape}")

    rng = np.random.default_rng(0)
    # Throughput test: small actions around zero (don't tip the robot
    # immediately), reset on term/trunc to keep stepping. Average over
    # 300 steps to estimate training feasibility.
    total_steps = 0
    total_resets = 0
    t_steps_start = time.time()
    for _ in range(300):
        action = rng.uniform(-0.05, 0.05, size=(ACT_DIM,)).astype(np.float32)
        obs, reward, term, trunc, _ = env.step(action)
        total_steps += 1
        if term or trunc:
            obs, _ = env.reset()
            total_resets += 1
    dt_steps = time.time() - t_steps_start
    sps = total_steps / dt_steps
    print(f"[smoke] {total_steps} steps in {dt_steps:.1f}s "
          f"= {sps:.0f} steps/s  (resets={total_resets})")
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
