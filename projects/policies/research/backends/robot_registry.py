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

"""Robot registry — maps a string name to its `RobotSpec`.

Adding a new robot is one entry here + one spec module. Discovery is
lazy: each robot module is only imported on first lookup, so importing
this registry doesn't drag in every robot's URDF.

To register a new robot:
  1. Write `projects/policies/research/backends/<robot>_robot_spec.py` exporting a
     module-level `ROBOT = RobotSpec(...)`.
  2. Add `"<name>": ("projects.policies.research.backends.<robot>_robot_spec", "ROBOT")`
     to ROBOTS below.

That's it. The trainer's `--robot <name>` picks it up.
"""
from __future__ import annotations

import importlib
from typing import Dict, Tuple

from .base import RobotSpec


# name -> (module path, attribute name)
ROBOTS: Dict[str, Tuple[str, str]] = {
    "spot":  ("projects.policies.research.backends.spot_robot_spec",  "SPOT"),
    "atlas": ("projects.policies.research.backends.atlas_robot_spec", "ATLAS"),
    "g1":    ("projects.policies.research.backends.g1_robot_spec",    "G1"),
}


def list_robots() -> list[str]:
    return sorted(ROBOTS)


def get_robot(name: str) -> RobotSpec:
    if name not in ROBOTS:
        raise ValueError(
            f"Unknown robot {name!r}. Registered: {list_robots()}. "
            f"To add one, see projects/policies/research/backends/robot_registry.py docstring."
        )
    mod_path, attr = ROBOTS[name]
    mod = importlib.import_module(mod_path)
    spec = getattr(mod, attr)
    if not isinstance(spec, RobotSpec):
        raise TypeError(f"{mod_path}.{attr} is {type(spec).__name__}, "
                        f"expected RobotSpec")
    return spec
