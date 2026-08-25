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

"""Minimal robot controller used by camera-heavy bench worlds.

Enables the `front_camera` device (and optionally a `front_depth`
RangeFinder) at every step. The point is to force per-step sensor
rendering without doing any non-trivial controller-side work, so the
benchmark isolates simulator render + IPC cost.
"""

from __future__ import annotations

from controller import Robot


def main() -> None:
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    cam = robot.getDevice("front_camera")
    if cam is not None:
        cam.enable(time_step)

    depth = robot.getDevice("front_depth")
    if depth is not None:
        depth.enable(time_step)

    while robot.step(time_step) != -1:
        pass


if __name__ == "__main__":
    main()
