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

"""Lane E1 (P5) smoke controller -- the wgpu RangeFinder applies noise + quantization.

World: noise 0.1 + resolution 0.25, box face at 0.70 m. The port applies noise BEFORE
quantization (WREN pass order), so around the face:
  * every finite depth is a multiple of 0.25 within 1e-4 (quantization live),
  * the centre 16x16 block shows MORE THAN ONE distinct quantized level (sigma 0.1 spreads
    0.70 across the 0.625/0.875 bucket edges ~ 24% of samples -- noise live).
RED-CAPABLE: without the port (or under OMNISIM_WGPU_SENSOR_POSTFX=0) the depth is the
smooth clamped 0.70/cos profile -- not on the 0.25 grid -> FAIL.

Writes its result to OMNISIM_R33B_RESULT_PATH."""

import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "lane_e1_rf_postfx_smoke.txt",
    ),
)


def write_result(line: str) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
rf = robot.getDevice("rf_wgpu")
rf.enable(ts)

robot.step(ts)
robot.step(ts)
robot.step(ts)

w = rf.getWidth()
h = rf.getHeight()
img = rf.getRangeImage()
if img is None or len(img) < w * h:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {w * h})\n")
    sys.exit(0)

step = 0.25
off_grid = 0
worst = ""
levels = set()
cx, cy = w // 2, h // 2
for y in range(h):
    for x in range(w):
        v = img[y * w + x]
        if v != v or v == float("inf"):
            continue
        frac = abs(v / step - round(v / step))
        if frac > 4e-4:
            off_grid += 1
            if not worst:
                worst = f"px ({x},{y}) got={v:.6f} ({frac * step:.6f} m off the 0.25 grid)"
        if abs(x - cx) <= 8 and abs(y - cy) <= 8:
            levels.add(round(v / step))

grid_ok = off_grid == 0
noise_ok = len(levels) >= 2
ok = grid_ok and noise_ok
tag = "PASS rangefinder-wgpu-postfx" if ok else "OBSERVE"
write_result(
    f"{tag} off-grid px={off_grid}/{w * h} {worst}; distinct centre levels="
    f"{sorted(l * step for l in levels)}  grid_ok={grid_ok} noise_ok={noise_ok}\n"
)
print(f"[rangefinder_wgpu_postfx_smoke] {tag} off_grid={off_grid} levels={len(levels)}")
sys.exit(0)
