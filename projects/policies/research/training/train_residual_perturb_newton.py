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

"""PPO residual-RL training for OmniQuad push-recovery under NEWTON physics.

Same recipe as train_residual_newton.py, with horizontal velocity-
impulse perturbations injected during training. The model walker is
structurally blind to external disturbances — gait engine assumes a
clean trot, balance PD only adjusts foot z. The ±3 cm foot-offset
residual is the right control authority to step into a push and
widen the support polygon. This is the canonical "what justifies
the RL part" experiment.

Default knobs (env-overridable):
  OMNIQUAD_PERTURB_DV_MAX=0.40    max Δv (m/s) applied per perturbation
                              0.4 m/s × ~32 kg = ~13 Ns ≈ small cube hit
  OMNIQUAD_PERTURB_DV_MIN=0.10    floor so every perturbation is noticeable
  OMNIQUAD_PERTURB_INTERVAL_S=2.5 seconds between perturbations
  OMNIQUAD_PERTURB_START_S=2.0    grace period before first perturbation
  OMNIQUAD_RES_R_UPRIGHT_BONUS_WT=1.0  weight on exp(-(r²+p²)/0.1) term

Run from repo root:
    python projects/policies/research/training/train_residual_perturb_newton.py \\
        --run-name omniquad_residual_perturb_v1 --steps 50000
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
        repo / "projects" / "rl" / "worlds" / "omniquad_residual_train_perturb_newton.omniworld"
    )
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    env.setdefault("OMNISIM_NEWTON_SEED_POSE", "1")
    env.setdefault("OMNISIM_NEWTON_TARGET_KE", "250")
    env.setdefault("OMNISIM_NEWTON_TARGET_KD", "60")
    env.setdefault("OMNISIM_NEWTON_GROUND_MU", "2.0")
    env.setdefault("PYTHONIOENCODING", "utf-8")

    # Perturbation curriculum knobs (overridable from the outside).
    env.setdefault("OMNIQUAD_PERTURB_DV_MIN", "0.10")
    env.setdefault("OMNIQUAD_PERTURB_DV_MAX", "0.40")
    env.setdefault("OMNIQUAD_PERTURB_INTERVAL_S", "2.5")
    env.setdefault("OMNIQUAD_PERTURB_START_S", "2.0")

    # Upright bonus has to be ON for push recovery — the flat alive
    # bonus has no gradient near the threshold, so PPO can't tell
    # "marginally leaning" from "perfectly level."
    env.setdefault("OMNIQUAD_RES_R_UPRIGHT_BONUS_WT", "1.0")
    env.setdefault("OMNIQUAD_RES_R_UPRIGHT_BONUS_SCALE", "0.10")
    # vx tracking deprioritized so the policy doesn't fight the push
    # by lunging forward. Heading + lateral penalties stay since
    # "stay on course" is still the task.
    env.setdefault("OMNIQUAD_RES_R_VX_WT", "1.0")
    env.setdefault("OMNIQUAD_RES_R_HEADING_WT", "-1.0")
    env.setdefault("OMNIQUAD_RES_R_LATERAL_WT", "-0.5")

    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "train_residual.py"),
        *sys.argv[1:],
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
