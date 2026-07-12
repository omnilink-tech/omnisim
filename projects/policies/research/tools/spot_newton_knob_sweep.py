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

"""Sweep Newton engine knob sets for the Spot model-only walker.

The May-2026 documented recipe (KE=250/KD=60/MU=2.0/SEED_POSE/FORCE_MUJOCO)
falls at ~1.2 s on the current binary. The G1 walk recipe (verified 2026-06-10
on this binary) adds SUBSTEPS=4 / STATICS=1 / BASE_GUARD=1 / SEED_REBUILD=1.
This sweep finds which knob set restores the Spot walk.

Run: python _scratch/spot_newton_knob_sweep.py
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

REPO = next(_p for _p in Path(__file__).resolve().parents
            if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
WORLD = REPO / "projects" / "policies" / "research" / "worlds" / "spot_model_walk_demo.wbt"
VERIFY = REPO / "projects" / "policies" / "research" / "tools" / "verify_straight_walk.py"

BASE = {
    "OMNISIM_HOME": str(REPO),
    "OMNISIM_URDF_USE_INERTIA": "1",
    "OMNISIM_NEWTON_FORCE_MUJOCO": "1",
    "OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE": "1",
    "OMNISIM_NEWTON_SEED_POSE": "1",
    "OMNISIM_NEWTON_TARGET_KE": "250",
    "OMNISIM_NEWTON_TARGET_KD": "60",
    "OMNISIM_NEWTON_GROUND_MU": "2.0",
}

VARIANTS = {
    "mjwarp (G1 engine path)": {"OMNISIM_NEWTON_MJWARP": "1"},
    "mjwarp+substeps4": {
        "OMNISIM_NEWTON_MJWARP": "1",
        "OMNISIM_NEWTON_SUBSTEPS": "4",
    },
    "base_guard OFF": {"OMNISIM_NEWTON_BASE_GUARD": "0"},
    "mesh_to_ode (revert W1 meshes)": {"OMNISIM_NEWTON_MESH_TO_ODE": "1"},
    "xpbd (drop FORCE_MUJOCO)": {"OMNISIM_NEWTON_FORCE_MUJOCO": ""},
    "old-proven ke1500/kd30": {
        "OMNISIM_NEWTON_TARGET_KE": "1500",
        "OMNISIM_NEWTON_TARGET_KD": "30",
    },
    "joint-clamp OFF": {"OMNISIM_NEWTON_DISABLE_JOINT_CLAMP": "1"},
}


def run_variant(name: str, extra: dict) -> None:
    env = os.environ.copy()
    env.update(BASE)
    env.update(extra)
    for k, v in list(env.items()):
        if v == "":
            del env[k]  # empty value means "unset" (C++ getenv must see null)
    print(f"\n########## {name} ##########", flush=True)
    subprocess.run(
        [sys.executable, str(VERIFY), "--no-policy", "--world", str(WORLD),
         "--duration", "20"],
        env=env, cwd=str(REPO),
    )


def main() -> None:
    names = sys.argv[1:] or list(VARIANTS)
    for name in names:
        run_variant(name, VARIANTS[name])


if __name__ == "__main__":
    main()
