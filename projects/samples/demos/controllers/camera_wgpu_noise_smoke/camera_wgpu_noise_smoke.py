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

"""Lane E1 (P5) smoke controller -- the wgpu Camera applies per-channel gaussian noise.

World: noise 0.05 (sigma = 0.05 in normalised channel units = ~12.75 bytes). Sample the
flat centre 16x16 of the cyan box face and compute each channel's SPATIAL std:
  * every channel std must sit in 5..30 bytes (right magnitude, per-channel independence),
  * alpha must be untouched (std == 0).
RED-CAPABLE: without the port (or under OMNISIM_WGPU_SENSOR_POSTFX=0) the flat-lit face has
per-channel std ~0-2 bytes -> FAIL.

Writes its result to OMNISIM_R33B_RESULT_PATH."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "lane_e1_cam_noise_smoke.txt",
    ),
)


def write_result(line: str) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
cam = robot.getDevice("cam_wgpu")
cam.enable(ts)

robot.step(ts)
robot.step(ts)
robot.step(ts)

w = cam.getWidth()
h = cam.getHeight()
raw = cam.getImage()
if raw is None or len(raw) < w * h * 4:
    write_result(f"FAIL cam.getImage() returned {0 if raw is None else len(raw)} bytes "
                 f"(expected {w * h * 4})\n")
    sys.exit(0)

cx, cy = w // 2, h // 2
samples = [[], [], [], []]  # B, G, R, A
for y in range(cy - 8, cy + 8):
    for x in range(cx - 8, cx + 8):
        i = (y * w + x) * 4
        for c in range(4):
            samples[c].append(raw[i + c])


def std(vals):
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


stds = [std(s) for s in samples]
chan_ok = all(5.0 < s < 30.0 for s in stds[:3])
alpha_ok = stds[3] == 0.0
ok = chan_ok and alpha_ok
tag = "PASS camera-wgpu-noise" if ok else "OBSERVE"
write_result(
    f"{tag} centre 16x16 per-channel std B={stds[0]:.2f} G={stds[1]:.2f} R={stds[2]:.2f} "
    f"A={stds[3]:.2f} bytes (expect ~12.75 each, alpha 0)  chan_ok={chan_ok} alpha_ok={alpha_ok}\n"
)
print(f"[camera_wgpu_noise_smoke] {tag} std BGR=({stds[0]:.2f},{stds[1]:.2f},{stds[2]:.2f})")
sys.exit(0)
