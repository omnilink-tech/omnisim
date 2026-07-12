#!/usr/bin/env python3
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

"""Reproducible launcher for the G1 Newton/mjwarp WALK deploy.

This script REPLACES the hand-maintained `env = ...` wall in the deploy recipe
(``docs/developer/g1-deploy-walk.md`` lines 68-75). Instead of pasting a block of
``OMNISIM_NEWTON_*`` knobs that can silently drift from the trainer, the PHYSICS
model env is derived from the single source of truth
(``projects/policies/research/backends/g1_physics_spec.py`` + ``g1_physics.json`` + the prim
URDF) via ``SPEC.newton_env()``. Change the physics model in ONE place (the JSON
/ URDF) and both the trainer and this deploy launch follow.

What this launcher sets, in two layers:

  1. PHYSICS-MODEL env (from the spec) -- ``OMNISIM_NEWTON_FORCE_MUJOCO``,
     ``OMNISIM_NEWTON_MJWARP``, ``OMNISIM_NEWTON_STATICS``,
     ``OMNISIM_NEWTON_SUBSTEPS``, ``OMNISIM_URDF_USE_INERTIA``,
     ``OMNISIM_NEWTON_SEED_POSE``, ``OMNISIM_NEWTON_TARGET_KE/KD``,
     ``OMNISIM_NEWTON_GROUND_MU``, ``OMNISIM_NEWTON_CONTACT_KE/KD``.
     These come straight from ``SPEC.newton_env()`` and are injected as
     DEFAULTS -- a var the caller already exported in the environment is left
     untouched (you can override one knob without forking the spec).

  2. PER-POLICY / GAIT env (NOT part of the physics model) -- the gait model,
     style, target velocity, frequency, ramp, obs stacking/lookahead, gait
     amplitudes, residual scale, and the (off) balance-PD gains. These describe
     the *policy* that was trained, not the simulator, so they stay here as
     launcher defaults (also overridable from the environment). The exact values
     mirror the winning ``gpu_newton_g1_walk_ft_pdoff_clamp`` recipe documented
     in ``docs/developer/g1-deploy-walk.md`` lines 68-75.

Then it execs ``scripts/dev/headless_runner.py <world> --gui --realtime --cold
--port <p> --duration 200`` -- the exact deploy launch from the recipe.

Usage:
    python scripts/dev/g1_deploy_launch.py [--world W] [--policy P]
                                           [--port N] [--duration S]
    python scripts/dev/g1_deploy_launch.py --print-env   # resolve + print, no launch
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents
             if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

# Defaults straight from the winning recipe (docs/developer/g1-deploy-walk.md).
DEFAULT_WORLD = "projects/policies/worlds/g1_walk_arms_deploy_mjwarp.wbt"
DEFAULT_POLICY = "runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.onnx"

# PER-POLICY / GAIT env -- NOT physics-model, so these live in the launcher
# (overridable). Values transcribed from docs/developer/g1-deploy-walk.md
# lines 68-75 (the gpu_newton_g1_walk_ft_pdoff_clamp recipe). G1_ACT_SCALE is
# sourced from the spec (the residual scale == the policy's --res-scale, which
# the spec records as the contract act_scale). The balance PD is OFF (=0): the
# policy was trained PD-off and learns all balance itself.
GAIT_ENV = {
    "G1_GAIT_MODEL": "human",
    "G1_GAIT_STYLE": "winter",
    "G1_GAIT_VX": "0.4",
    "G1_GAIT_FREQ": "1.3",
    "G1_GAIT_RAMP_S": "2.0",
    "G1_OBS_STACK": "4",
    "G1_OBS_LOOKAHEAD": "0.1,0.4",
    "G1_GAIT_A_LAT": "0.05",
    "G1_GAIT_A_ANKLE": "0.0",
    "G1_GAIT_A_ARM": "0.25",
    "G1_GAIT_HIP_SCALE": "0.9",
    "G1_ACT_SCALE": SPEC._num(SPEC.ACT_SCALE),  # == policy --res-scale (0.3)
    "G1_BAL_KP_P": "0",
    "G1_BAL_KD_P": "0",
    "G1_BAL_KP_R": "0",
    "G1_BAL_KD_R": "0",
}


def resolve_env(world: str, policy: str) -> dict[str, str]:
    """Compute the full set of env vars this launcher would inject.

    Spec-derived physics env + per-policy gait env are injected as DEFAULTS:
    a var the caller already set in os.environ is preserved. Returns the
    effective values (post-merge) for inspection / launch.
    """
    resolved: dict[str, str] = {}
    # Layer 1: physics model (single source of truth).
    for k, v in SPEC.newton_env().items():
        resolved[k] = os.environ.get(k, v)
    # Layer 2: per-policy / gait defaults.
    for k, v in GAIT_ENV.items():
        resolved[k] = os.environ.get(k, v)
    # The policy path the deploy controller picks up.
    resolved["G1_POLICY_ONNX"] = os.environ.get("G1_POLICY_ONNX", policy)
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Reproducible spec-driven launcher for the G1 Newton walk deploy")
    parser.add_argument("--world", default=DEFAULT_WORLD,
                        help=f"World file (default: {DEFAULT_WORLD})")
    parser.add_argument("--policy", default=DEFAULT_POLICY,
                        help=f"Policy ONNX path (default: {DEFAULT_POLICY})")
    parser.add_argument("--port", type=int, default=1234,
                        help="Robot-window/extern server port (default: 1234)")
    parser.add_argument("--duration", type=int, default=200,
                        help="Seconds to run before stopping (default: 200)")
    parser.add_argument("--print-env", action="store_true",
                        help="Resolve and print the env this launcher would inject, "
                             "then exit WITHOUT launching the simulator.")
    args = parser.parse_args()

    # Resolve policy to an absolute-ish path the controller can find.
    policy = args.policy
    policy_path = Path(policy)
    if not policy_path.is_absolute():
        # The recipe's bare "runs/..." is relative to projects/policies/research/training.
        cand = _REPO / "projects/policies/research/training" / policy
        policy = str(cand if cand.parent.parent.exists() else _REPO / policy)

    env = resolve_env(args.world, policy)

    if args.print_env:
        print("# Resolved env for the G1 Newton walk deploy")
        print(f"# world  = {args.world}")
        print(f"# policy = {env['G1_POLICY_ONNX']}")
        print(f"# port   = {args.port}    duration = {args.duration}s")
        print(f"# launcher: scripts/dev/headless_runner.py <world> "
              f"--gui --realtime --cold --port {args.port} --duration {args.duration}")
        print("#")
        print("# --- physics model (from g1_physics_spec.newton_env) ---")
        for k in sorted(SPEC.newton_env()):
            print(f"{k}={env[k]}")
        print("# --- per-policy / gait (launcher defaults) ---")
        for k in GAIT_ENV:
            print(f"{k}={env[k]}")
        print(f"G1_POLICY_ONNX={env['G1_POLICY_ONNX']}")
        return 0

    # Inject defaults WITHOUT clobbering anything the caller already set.
    for k, v in env.items():
        os.environ.setdefault(k, v)

    runner = _REPO / "scripts" / "dev" / "headless_runner.py"
    world = args.world
    if not Path(world).is_absolute():
        world = str(_REPO / world)
    cmd = [
        sys.executable, "-u", str(runner), world,
        "--gui", "--realtime", "--cold",
        "--port", str(args.port),
        "--duration", str(args.duration),
    ]
    print(f"[g1_deploy_launch] world={world}")
    print(f"[g1_deploy_launch] policy={os.environ['G1_POLICY_ONNX']}")
    print(f"[g1_deploy_launch] launching: {' '.join(cmd)}")
    return subprocess.call(cmd, env=os.environ.copy())


if __name__ == "__main__":
    sys.exit(main())
