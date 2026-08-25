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

"""Supervisor controller used by optim_bench.py worlds.

Steps the simulation a fixed number of times, then calls
`simulationQuit(0)` so the simulator exits cleanly and writes the
performance log.

Step count is read from `--steps N` in `controllerArgs`. Default 1000.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from controller import Supervisor


def parse_steps(argv: list[str], default: int = 1000) -> int:
    for i, arg in enumerate(argv):
        if arg == "--steps" and i + 1 < len(argv):
            try:
                return int(argv[i + 1])
            except ValueError:
                pass
        if arg.startswith("--steps="):
            try:
                return int(arg.split("=", 1)[1])
            except ValueError:
                pass
    return default


def parse_arg(argv: list[str], name: str) -> str | None:
    for i, arg in enumerate(argv):
        if arg == name and i + 1 < len(argv):
            return argv[i + 1]
        if arg.startswith(name + "="):
            return arg.split("=", 1)[1]
    return None


def write_robot_state(sup: Supervisor, path: Path, robot_count: int, steps: int) -> None:
    robots = []
    for index in range(robot_count):
        def_name = f"BOT_{index:03d}"
        node = sup.getFromDef(def_name)
        if node is None:
            raise RuntimeError(f"missing benchmark robot DEF {def_name}")
        robots.append({
            "def": def_name,
            "position": list(node.getPosition()),
            "orientation": list(node.getOrientation()),
            "velocity": list(node.getVelocity()),
        })
    payload = {
        "basic_time_step_ms": sup.getBasicTimeStep(),
        "steps": steps,
        "time_s": sup.getTime(),
        "robots": robots,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    sup = Supervisor()
    time_step = int(sup.getBasicTimeStep())
    target = parse_steps(sys.argv)
    state_out_arg = parse_arg(sys.argv, "--state-out")
    robot_count_arg = parse_arg(sys.argv, "--robot-count")
    robot_count = int(robot_count_arg) if robot_count_arg else 0

    steps = 0
    while sup.step(time_step) != -1:
        steps += 1
        if steps >= target:
            if state_out_arg:
                write_robot_state(sup, Path(state_out_arg), robot_count, steps)
            sup.simulationQuit(0)
            break


if __name__ == "__main__":
    main()
