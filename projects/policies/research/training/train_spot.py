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

"""Back-compat shim — forwards to train_robot.py with --robot spot.

The original training entrypoint. Kept so existing scripts and the
shipped demos still work; new code should call train_robot.py directly
to make the robot choice explicit.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()

    sys.path.insert(0, str(REPO_ROOT))

    # Forward every CLI arg, injecting --robot spot if the user didn't
    # already pass one.
    args = list(sys.argv[1:])
    if not any(a == "--robot" or a.startswith("--robot=") for a in args):
        args = ["--robot", "spot"] + args

    sys.argv = [str(REPO_ROOT / "projects" / "rl" / "training" / "train_robot.py")] + args
    from projects.policies.research.training import train_robot  # noqa: E402
    sys.exit(train_robot.main())
