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
"""Paint the Display red, then its left half green, and read what the Camera facing its screen sees."""
from pathlib import Path
import json
import sys
from omnisim import Robot

robot = Robot()
step = int(robot.getBasicTimeStep())
out = Path(sys.argv[1])
cam = robot.getDevice("camera")
cam.enable(step)
disp = robot.getDevice("display")
w, h = cam.getWidth(), cam.getHeight()


def sample():
    img = cam.getImage()
    px = {}
    for name, (u, v) in {"left": (0.3, 0.5), "right": (0.7, 0.5)}.items():
        i = 4 * (int(v * h) * w + int(u * w))
        px[name] = [img[i + 2], img[i + 1], img[i]]          # BGRA -> RGB
    return px


rows = {}
try:
    disp.setColor(0xFF0000)
    disp.fillRectangle(0, 0, disp.getWidth(), disp.getHeight())
    for _ in range(10):
        robot.step(step)
    rows["red"] = sample()
    # the camera faces the screen, so its left is the display's left (pixel x 0)
    disp.setColor(0x00FF00)
    disp.fillRectangle(0, 0, disp.getWidth() // 2, disp.getHeight())
    for _ in range(10):
        robot.step(step)
    rows["half_green"] = sample()
    (out / "done.json").write_text(json.dumps(rows), encoding="utf-8")
except Exception as e:  # noqa: BLE001
    (out / "error.txt").write_text(repr(e), encoding="utf-8")
robot.step(step)
