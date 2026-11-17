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

"""Fixed overhead camera for the OMNITUG500 explore-and-map demo.

Saves a numbered top-down frame every SAVE_EVERY steps so the exploration can be
shown as stills. The live view is the GUI; this just captures evidence.
"""

import os
import traceback

from omnisim import Robot

OUT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "..", "_scratch", "omnitug500_explore"))

SAVE_EVERY = 80
MAX_FRAMES = 30


def main():
    os.makedirs(OUT, exist_ok=True)
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    cam = robot.getDevice("cam")
    if cam is None:
        return
    cam.enable(ts)
    step = 0
    saved = 0
    while robot.step(ts) != -1:
        step += 1
        if step % SAVE_EVERY == 0 and saved < MAX_FRAMES:
            try:
                cam.saveImage(os.path.join(OUT, f"view_{step:05d}.png"), 95)
                saved += 1
            except Exception:
                with open(os.path.join(OUT, "cam.log"), "a", encoding="utf-8") as f:
                    f.write("saveImage failed:\n" + traceback.format_exc() + "\n")


if __name__ == "__main__":
    main()
