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

"""Launch the Newton deploy world (GUI) with a chosen policy ONNX
and the standard Newton opt-ins. Use after training to actually see
the resulting walker in the GUI.

Run from repo root:
    python projects/policies/research/tools/_show_newton_policy.py \\
        --policy projects/policies/research/inference/policies/omniquad_newton_v2/policy.onnx
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--vx", type=float, default=0.3)
    args = p.parse_args()
    if not args.policy.exists():
        print(f"policy not found: {args.policy}")
        return 1

    webots = REPO_ROOT / "msys64" / "mingw64" / "bin" / "webots.exe"
    world = REPO_ROOT / "projects" / "rl" / "worlds" / "omniquad_newton_demo.omniworld"

    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["OMNIQUAD_POLICY_ONNX"] = str(args.policy)
    env["OMNISIM_POLICY_ONNX"] = str(args.policy)
    env["OMNIQUAD_DEPLOY_TRACE"] = "1"
    env["OMNISIM_DEPLOY_TRACE"] = "1"
    env["OMNIQUAD_VX"] = str(args.vx)
    env["OMNIQUAD_VY"] = "0.0"
    env["OMNIQUAD_WZ"] = "0.0"
    env["OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE"] = "1"
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    # MuJoCo CPU is stable; XPBD GPU NaN's under the URDF effort/inertia
    # config for this OmniQuad. Unset OMNISIM_NEWTON_FORCE_MUJOCO if you
    # want XPBD's faster runtime instead.
    env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    env["OMNISIM_NEWTON_LOG"] = str(REPO_ROOT / ".build_tmp" / "newton_show.log")
    print(f"[show] policy = {args.policy}")
    print(f"[show] world  = {world}")
    print(f"[show] vx     = {args.vx}")
    subprocess.Popen([str(webots), str(world)], env=env, cwd=str(REPO_ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
