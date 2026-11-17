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

"""Noisy controller used to stress the controller stdout pipe.

Each step prints `--bytes N` characters to stdout (default 4096) and
optionally drives wheels (default off). The point is to expose pipe-
size limits in the controller<->simulator IPC: with a too-small kernel
pipe, the write blocks and the controller bucket time spikes.

Usage in a `.wbt` Robot.controllerArgs:
    ["--bytes" "4096"]
"""

from __future__ import annotations

import sys

from controller import Robot


def parse_bytes(argv: list[str], default: int = 4096) -> int:
    for i, a in enumerate(argv):
        if a == "--bytes" and i + 1 < len(argv):
            try:
                return int(argv[i + 1])
            except ValueError:
                pass
        if a.startswith("--bytes="):
            try:
                return int(a.split("=", 1)[1])
            except ValueError:
                pass
    return default


def main() -> None:
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    n = parse_bytes(sys.argv)
    payload = "x" * max(n - 1, 0) + "\n"  # one newline so the line is countable

    while robot.step(time_step) != -1:
        sys.stdout.write(payload)
        sys.stdout.flush()


if __name__ == "__main__":
    main()
