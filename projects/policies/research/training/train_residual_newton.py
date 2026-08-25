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

"""PPO residual-RL training for OmniQuad under NEWTON physics.

Same recipe as train_residual.py (model walker + 12-dim foot-offset
residual policy), but routes OmniQuadResidualEnv at the Newton training
world and enables the Newton-side opt-ins from train_omniquad_newton.py
that produced the working "OmniQuad stands and walks open-loop" baseline.

Opt-ins:
  OMNIQUAD_TRAIN_WORLD               -> omniquad_residual_train_newton.omniworld
  OMNISIM_URDF_USE_INERTIA       -> 1
  OMNISIM_NEWTON_FORCE_MUJOCO    -> 1
  OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE -> 1
  OMNISIM_NEWTON_SEED_POSE       -> 1
  OMNISIM_NEWTON_TARGET_KE/KD    -> 250 / 60
  OMNISIM_NEWTON_GROUND_MU       -> 2.0

The residual recipe under these settings: open-loop model walker
(gait + IK + balance, no policy) already walks +5.03 m / 30 s STRAIGHT
under Newton MuJoCo CPU — better than the ODE model walker baseline.
The residual policy refines that to bring lateral drift / yaw closer
to zero.

Run from repo root:
    python projects/policies/research/training/train_residual_newton.py \\
        --run-name omniquad_residual_newton_v1 --steps 50000
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

    env["OMNIQUAD_TRAIN_WORLD"] = str(
        repo / "projects" / "rl" / "worlds" / "omniquad_residual_train_newton.omniworld"
    )
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    env.setdefault("OMNISIM_NEWTON_SEED_POSE", "1")
    env.setdefault("OMNISIM_NEWTON_TARGET_KE", "250")
    env.setdefault("OMNISIM_NEWTON_TARGET_KD", "60")
    env.setdefault("OMNISIM_NEWTON_GROUND_MU", "2.0")
    env.setdefault("PYTHONIOENCODING", "utf-8")

    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "train_residual.py"),
        *sys.argv[1:],
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
