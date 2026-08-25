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

"""AgentBench path constants and the omnibench module bridge.

Machine attribution, the row writer and the engine launcher are REUSED from
``tests/benchmarks/omnibench`` (SPEC 1: "do not fork it"). This module is the
single place that puts them on ``sys.path``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent            # .../agentbench/common
AGENTBENCH = HERE.parent                          # .../agentbench
BENCHMARKS = AGENTBENCH.parent                    # .../tests/benchmarks
REPO = AGENTBENCH.parents[2]                      # repo root

CONTROLLERS = AGENTBENCH / "controllers"
TASKS_DIR = AGENTBENCH / "tasks"
RESULTS_DIR = AGENTBENCH / "results"

HUSKY_URDF = (REPO / "projects" / "robots" / "clearpath"
              / "husky_description" / "urdf" / "husky.urdf")

_OMNIBENCH = BENCHMARKS / "omnibench"


def _load(mod_name, path):
    """Load an omnibench module by file path.

    Deliberately not ``sys.path`` + ``from common import ...``: omnibench's
    package is literally named ``common``, and so is ours, so a path-order
    accident would silently import the wrong one.
    """
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(mod_name, mod)
    spec.loader.exec_module(mod)
    return mod


engine_launch = _load("agentbench_ob_engine_launch",
                      _OMNIBENCH / "common" / "engine_launch.py")
ob_results = _load("agentbench_ob_results",
                   _OMNIBENCH / "common" / "results.py")


def as_wbt_path(p) -> str:
    """A path string a .wbt / controllerArgs entry can hold on any platform."""
    return str(Path(p).resolve()).replace("\\", "/")


__all__ = [
    "HERE", "AGENTBENCH", "BENCHMARKS", "REPO", "CONTROLLERS", "TASKS_DIR",
    "RESULTS_DIR", "HUSKY_URDF", "engine_launch", "ob_results", "as_wbt_path",
]
