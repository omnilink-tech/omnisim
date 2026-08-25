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

"""Lane E1 (P5+P6) smoke controller -- the wgpu Lidar applies authored range quantization.

World: resolution 0.25, fov 0.8 (every ray hits the 0.70 m face). depth_resolution.frag is
floor(r / 0.25 + 0.5) * 0.25, so:
  * the centre column (0.7000 m true) must read exactly 0.75,
  * EVERY column must be a multiple of 0.25 within 1e-4.
RED-CAPABLE: without the port (or under OMNISIM_WGPU_SENSOR_POSTFX=0) the profile is the
smooth 0.70/cos(theta) -- 0.7000 is not a multiple of 0.25 -> FAIL.

Writes its result to OMNISIM_R33B_RESULT_PATH."""

import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "lane_e1_lidar_resolution_smoke.txt",
    ),
)


def write_result(line: str) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
lidar = robot.getDevice("lidar_wgpu")
lidar.enable(ts)

robot.step(ts)
robot.step(ts)
robot.step(ts)

res = lidar.getHorizontalResolution()
img = lidar.getRangeImage()
if img is None or len(img) < res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {res})\n")
    sys.exit(0)

cx = res // 2
center = img[cx]
step = 0.25
off_grid = 0
worst = ""
for i in range(res):
    v = img[i]
    if v != v or v == float("inf"):
        continue
    frac = abs(v / step - round(v / step))
    if frac > 4e-4:  # 1e-4 m at step 0.25
        off_grid += 1
        if not worst:
            worst = f"col {i} got={v:.6f} ({frac * step:.6f} m off the 0.25 grid)"

center_ok = abs(center - 0.75) < 1e-3
grid_ok = off_grid == 0
ok = center_ok and grid_ok
tag = "PASS lidar-wgpu-resolution" if ok else "OBSERVE"
write_result(
    f"{tag} center({cx})={center:.4f} m (expect 0.7500) off-grid cols={off_grid}/{res} "
    f"{worst}  center_ok={center_ok} grid_ok={grid_ok}\n"
)
print(f"[lidar_wgpu_resolution_smoke] {tag} center={center:.4f} off_grid={off_grid}")
sys.exit(0)
