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

"""Continue a OmniQuad PPO run under NEWTON physics (warm-start from a ckpt).

continue_training.py loads a checkpoint and keeps training, but it does NOT
set the Newton-side opt-ins (world, URDF inertia, MuJoCo solver, wrapper
shape, seed-pose, leg-spring gains, foot friction, CPG prior) -- those are
injected by train_omniquad_newton.py for fresh runs. This wrapper injects the
SAME physics env around continue_training.py so a warm-continue runs on
bit-identical dynamics to the run that produced the checkpoint.

Physics opt-ins are forced (they define the sim and MUST match the source
run). CPG prior is forced too (must match deploy + the source run). Reward
weights / curriculum (OMNIQUAD_R_*, OMNIQUAD_FIXED_*) are left to the environment
so the caller can shape the curriculum per phase -- e.g. bump the
orientation penalty for a balance phase, or add the vx bonus + raise
OMNIQUAD_FIXED_VX for a speed phase.

Run from repo root, e.g. (balance phase):
    OMNIQUAD_R_ROLLPITCH_WT=-0.25 OMNIQUAD_FIXED_VX=0.1 \\
      python projects/policies/research/training/continue_omniquad_newton.py \\
        --from projects/policies/research/training/runs/omniquad_newton_v21_mu2/checkpoints/omniquad_60000_steps.zip \\
        --run-name omniquad_newton_v23_balance --steps 150000 --envs 2 \\
        --lr 2e-4 --max-grad-norm 0.5 --ckpt-every 25000
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
    env = os.environ.copy()
    if not env.get("OMNISIM_SESSION"):
        import uuid
        env["OMNISIM_SESSION"] = uuid.uuid4().hex[:8]
    # --- Physics: MUST match the source run (forced, not setdefault). ---
    env["OMNIQUAD_TRAIN_WORLD"] = str(repo / "projects" / "rl" / "worlds" / "omniquad_rl_newton.omniworld")
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    env["OMNISIM_NEWTON_SEED_POSE"] = "1"
    env.setdefault("OMNISIM_NEWTON_TARGET_KE", "250")
    env.setdefault("OMNISIM_NEWTON_TARGET_KD", "25")
    env.setdefault("OMNISIM_NEWTON_GROUND_MU", "2.0")
    # --- CPG prior: must match deploy + source run. ---
    env.setdefault("OMNIQUAD_CPG_FREQ_HZ", "1.75")
    env.setdefault("OMNIQUAD_CPG_HIP_Y_AMP", "0.13")
    env.setdefault("OMNIQUAD_CPG_KNEE_AMP", "0.22")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "continue_training.py"),
        *sys.argv[1:],
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
