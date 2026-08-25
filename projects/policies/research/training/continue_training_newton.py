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

"""Warm-start an SB3 PPO run against Newton physics from an existing
checkpoint (typically the ODE-trained omniquad_walk_v12).

Wraps continue_training.py with the Newton-side opt-ins:

  OMNIQUAD_TRAIN_WORLD                 -> omniquad_rl_newton.omniworld
  OMNISIM_URDF_USE_INERTIA         -> 1 (real per-link inertia tensors)
  OMNISIM_NEWTON_FORCE_MUJOCO      -> 1 (MuJoCo CPU solver under Newton)
  OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE -> 1 (OmniQuad chassis bounding box)

Default reward + CPG knobs are tuned for a sane "keep walking" curriculum
on the warm-started policy:

  OMNIQUAD_R_LIN_VEL_WT      = 2.0   (track commanded vx without dominating)
  OMNIQUAD_R_ALIVE_BONUS     = 0.5   (positive every step the body is upright
                                  -- defeats the "freeze for -10/step is
                                  worse than +0/step" local optimum the
                                  fresh-init 100k run got stuck in)
  OMNIQUAD_R_VZ_WT           = -0.2  (mild vertical-velocity penalty)
  OMNIQUAD_R_ANG_XY_WT       = -0.2  (mild yaw-rate penalty)
  OMNIQUAD_R_ROLLPITCH_WT    = -0.2  (mild roll/pitch penalty)
  OMNIQUAD_FIXED_VX          = 0.3   (slower-than-train_omniquad_walk curriculum:
                                  matches the policy's existing capability,
                                  ramps later runs to 0.5)
  OMNIQUAD_FIXED_WZ          = 0.0

Run from repo root:
    python projects/policies/research/training/continue_training_newton.py \\
        --from projects/policies/research/training/runs/omniquad_walk_v12/checkpoints/omniquad_140000_steps.zip \\
        --run-name omniquad_newton_v2 \\
        --steps 200000 --ckpt-every 25000 \\
        --lr 1e-4 --target-kl 0.03 --max-grad-norm 0.5
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
    env = os.environ.copy()
    # Unique session namespace -- disjoint Webots ports + scratch files
    # from any parallel agent session (see session_isolation.py).
    if not env.get("OMNISIM_SESSION"):
        import uuid
        env["OMNISIM_SESSION"] = uuid.uuid4().hex[:8]
    env["OMNIQUAD_TRAIN_WORLD"] = str(repo / "projects" / "rl" / "worlds" / "omniquad_rl_newton.omniworld")
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    # MuJoCo CPU is the stable solver. XPBD on GPU NaN's on OmniQuad under
    # the URDF effort/inertia config -- not usable for training a
    # walker (the policy can't learn against a physics that explodes).
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    # CPG MUST match the warm-start policy's training. omniquad_walk_v12
    # (the ODE walker we warm-start from) was trained under ODE with
    # this exact CPG trot, and the deploy sidecar uses it too. Keeping
    # it here means the only thing PPO has to adapt is the ODE->Newton
    # actuator-dynamics gap, with the gait prior already in the policy.
    env.setdefault("OMNIQUAD_CPG_FREQ_HZ", "1.75")
    env.setdefault("OMNIQUAD_CPG_HIP_Y_AMP", "0.13")
    env.setdefault("OMNIQUAD_CPG_KNEE_AMP", "0.22")
    # Conservative reward to PRESERVE the walker, not reshape it: modest
    # alive bonus, moderate velocity tracking, gentle stability
    # penalties, NO joint-limit penalty or vx bonus (those reshape the
    # gait and risk destroying the warm-started walking behaviour).
    env.setdefault("OMNIQUAD_R_LIN_VEL_WT", "1.5")
    env.setdefault("OMNIQUAD_R_ALIVE_BONUS", "0.5")
    env.setdefault("OMNIQUAD_R_VZ_WT", "-0.2")
    env.setdefault("OMNIQUAD_R_ANG_XY_WT", "-0.1")
    env.setdefault("OMNIQUAD_R_ROLLPITCH_WT", "-0.2")
    env.setdefault("OMNIQUAD_FIXED_VX", "0.3")
    env.setdefault("OMNIQUAD_FIXED_WZ", "0.0")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "continue_training.py"),
        *sys.argv[1:],
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
