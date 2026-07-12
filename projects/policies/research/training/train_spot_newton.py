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

"""PPO training for Spot under NEWTON physics (instead of the default ODE).

Equivalent to train_spot_walk.py but points SpotEnv at the Newton
training world and enables the Newton-side opt-ins:

  - SPOT_TRAIN_WORLD          -> projects/policies/worlds/spot_rl_newton.wbt
  - OMNISIM_URDF_USE_INERTIA  -> 1 (real URDF inertia tensors)
  - OMNISIM_NEWTON_FORCE_MUJOCO -> 1 (CPU MuJoCo solver, articulation-stable)
  - OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE -> 1 (chassis bounding box on the
    URDFRobot wrapper, since Spot's legs hang below the chassis envelope)

The trained policy will be calibrated for Newton's actuator dynamics
(stiff position spring + URDF effort/velocity caps), not ODE's internal
PD. Deploy through the spot_newton_demo world (or any world with
`physicsBackend "newton"`).

Run from repo root:
    python projects/policies/research/training/train_spot_newton.py --envs 4 --steps 300000 \\
        --run-name spot_newton_v1
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
    env = os.environ.copy()
    # Unique session namespace so this run's Webots ports + scratch files
    # never collide with a parallel agent session sharing the box. All
    # SpotEnv subprocesses inherit OMNISIM_SESSION and derive disjoint
    # ports from it (see projects/policies/research/envs/session_isolation.py).
    if not env.get("OMNISIM_SESSION"):
        import uuid
        env["OMNISIM_SESSION"] = uuid.uuid4().hex[:8]
    env["SPOT_TRAIN_WORLD"] = str(repo / "projects" / "rl" / "worlds" / "spot_rl_newton.wbt")
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    # Spawn each episode in the standing pose (Newton seeds joints at 0 =
    # straight legs otherwise, and the fold-into-stance transient tips the
    # robot) and use a stiff-enough leg spring to actually HOLD the stance
    # (the default ke=20 sags). Without these the from-scratch policy never
    # gets a stable platform to learn balance on. See
    # project_spot_newton_standing memory.
    env.setdefault("OMNISIM_NEWTON_SEED_POSE", "1")
    env.setdefault("OMNISIM_NEWTON_TARGET_KE", "250")
    # kd=60 (not the legacy 25): the higher joint damping suppresses the
    # slow oscillation that made the open-loop trot diverge -- it nearly
    # doubled bare-gait survival (policy off, ~150 -> ~265 steps) and gave
    # the first deterministic policy that balances the FULL 1023-step
    # episode. kd>~80 over-damps and (with this trim) nose-dives.
    env.setdefault("OMNISIM_NEWTON_TARGET_KD", "60")
    # Foot friction. The default backend mu=1.0 lets sphere feet SLIDE ~1 m
    # while merely standing under SolverMuJoCo (friction-probe finding); the
    # slip is sharply eliminated at mu=2.0 (feet plant <4 cm). Slipping feet
    # give no lateral restoring force, which is why the v20 policy's roll
    # went marginal under the trot (height + pitch held, roll diverged to
    # ~0.9 rad). Plant the feet so the policy has a stable contact to learn
    # balance on. Deploy MUST set the same value (sidecar / deploy env).
    env.setdefault("OMNISIM_NEWTON_GROUND_MU", "2.0")
    # Feedforward pitch trim. The bare CPG trot (policy disabled) pitches
    # the body NOSE-DOWN to -1.0 rad (PITCH_FAIL) by ~step 90 under Newton
    # at mu=2.0 -- the planted front feet pivot and the trot's forward
    # propulsion tips the body over. This structural gait instability (NOT
    # the policy) capped every run at ep_len ~100. A +0.20 rad fore/aft hip
    # bias re-levels the gait equilibrium (pure-gait pitch then stays in a
    # bounded +/-0.2 band with height steady). Deploy MUST set the same
    # value. See project_spot_newton_walking memory.
    env.setdefault("SPOT_PITCH_TRIM", "0.20")
    # From-scratch curriculum (the 100k run with ALIVE_BONUS=0 collapsed
    # to a 'freeze' policy at reward -10/step because doing nothing was
    # better than moving + falling). The current settings let PPO
    # discover gait gradually:
    #   - alive bonus rewards staying upright while exploring
    #   - velocity tracking weight halved so it doesn't dominate
    #   - vx target = 0.1 (the policy needs to walk at all before
    #     having to walk fast)
    env.setdefault("SPOT_R_LIN_VEL_WT", "1.0")
    env.setdefault("SPOT_R_ALIVE_BONUS", "1.0")
    env.setdefault("SPOT_FIXED_VX", "0.1")
    env.setdefault("SPOT_FIXED_WZ", "0.0")
    # STAGE 1 = stand first. The v4 lesson: loading the vx bonus +
    # joint-limit penalty up front made the policy lunge and fall
    # before it could balance (ep_len stuck at ~45). Start with just
    # the alive bonus + saturating velocity-tracking term so PPO
    # learns to stay upright; layer the vx bonus / joint penalty in a
    # follow-up continue_training run once ep_len saturates near 1023.
    env.setdefault("SPOT_R_JOINT_LIMIT_WT", "0.0")
    env.setdefault("SPOT_R_VX_BONUS_WT", "0.0")
    # CPG trot prior MUST match deploy. The deploy sidecar (SPOT spec)
    # uses 1.75 Hz / hip 0.13 / knee 0.22; if training leaves the CPG
    # off (the old default) the policy learns a gait that has no trot
    # underneath it, then deploy adds the trot the policy never saw and
    # the body tips in <1 s. Training with the same CPG means the policy
    # learns a residual ON the trot, exactly as the ODE spot_walk_v12
    # recipe did.
    env.setdefault("SPOT_CPG_FREQ_HZ", "1.75")
    env.setdefault("SPOT_CPG_HIP_Y_AMP", "0.13")
    env.setdefault("SPOT_CPG_KNEE_AMP", "0.22")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    cmd = [
        sys.executable,
        str(repo / "projects" / "rl" / "training" / "train_spot.py"),
        *sys.argv[1:],
    ]
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
