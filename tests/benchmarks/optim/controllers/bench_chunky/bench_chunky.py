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

"""Controller used by the IPC-stress bench.

Each step:
  - Sets `customData` to a `--bytes N` payload (stresses simulator-side
    IPC READER: bytes flow controller -> simulator inside the request).
  - Reads several sensors (stresses simulator-side IPC WRITER: bytes
    flow simulator -> controller inside the response).

Used by `optim_bench.py chunky`. The point is to exercise the
OmController::readRequest copy chain (item 4) and writeAnswer pipeline
(item 5) with measurable per-step payloads.
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
    # Reserve up to 16 bytes for a per-step counter so the simulator can't
    # dedupe (setCustomData no-ops when the value is unchanged).
    counter_width = 16
    base_len = max(n - counter_width, 0)
    base = "x" * base_len

    # Enable sensors if present so the response packet has real content.
    sensors = []
    for name in ("gps", "imu", "compass", "accelerometer", "gyro"):
        d = robot.getDevice(name)
        if d is not None:
            d.enable(time_step)
            sensors.append((name, d))

    step_no = 0
    while robot.step(time_step) != -1:
        step_no += 1
        suffix = f"{step_no:0{counter_width}d}"
        robot.setCustomData(base + suffix)
        # Force-touch the sensor values so the controller really consumes them.
        for name, d in sensors:
            try:
                if hasattr(d, "getValues"):
                    d.getValues()
                elif hasattr(d, "getValue"):
                    d.getValue()
            except Exception:
                pass


if __name__ == "__main__":
    main()
