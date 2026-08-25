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

"""Fixed overhead camera for the OMNITUG500 moving-coverage demo.

Plain Robot whose only job is to save top-down proof frames at a few steps as
the rover drives its loop, so the moving red coverage area can be shown as
stills. The live view is the GUI; this just captures evidence.
"""

import os
import traceback

from omnisim import Robot

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "..", "_scratch", "omnitug500_lidar_patrol"))

# PERIOD=12 s, basicTimeStep=16 ms -> ~750 steps/loop. These span ~one full loop.
SHOT_STEPS = [120, 245, 370, 495, 620, 745]


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    cam = robot.getDevice("cam")
    if cam is None:
        return
    cam.enable(ts)

    targets = set(SHOT_STEPS)
    step_count = 0
    while robot.step(ts) != -1:
        step_count += 1
        if step_count in targets:
            path = os.path.join(OUT_DIR, f"frame_{step_count:04d}.png")
            try:
                cam.saveImage(path, 100)
                with open(os.path.join(OUT_DIR, "cam.log"), "a", encoding="utf-8") as f:
                    f.write(f"saved {path}\n")
            except Exception:
                with open(os.path.join(OUT_DIR, "cam.log"), "a", encoding="utf-8") as f:
                    f.write("saveImage failed:\n" + traceback.format_exc() + "\n")


if __name__ == "__main__":
    main()
