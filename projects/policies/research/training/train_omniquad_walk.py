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

"""PPO training for OmniQuad — WALK-focused reward shaping.

Identical to train_omniquad.py but reads reward weights from env vars so
this variant can push harder on velocity tracking and remove the alive
bonus that creates a "stand still" local optimum.

Run from repo root:
    python projects/policies/research/training/train_omniquad_walk.py --envs 4 --steps 500000 \
        --run-name omniquad_walk

Env vars consumed by the controller (omniquad_rl_agent.py):
    OMNIQUAD_R_LIN_VEL_WT    forward velocity tracking weight (default 1.0)
    OMNIQUAD_R_ALIVE_BONUS   per-tick alive bonus (default 0.05)
    OMNIQUAD_R_VZ_WT         vertical velocity penalty (default -1.0)

This file just sets aggressive walk-incentive defaults and re-uses the
training entrypoint.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
    env = os.environ.copy()
    # Stronger velocity tracking, no alive bonus -> standing still costs
    # the velocity-tracking penalty without any survival reward to offset
    # it. The policy is incentivised to actually move.
    env["OMNIQUAD_R_LIN_VEL_WT"] = "2.5"
    env["OMNIQUAD_R_ALIVE_BONUS"] = "0.0"
    # Default args to train_omniquad.py:
    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "train_omniquad.py"),
        *sys.argv[1:],  # forward remaining args
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
