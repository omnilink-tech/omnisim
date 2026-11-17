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

"""Lane E1 (P5+P6) smoke controller -- the wgpu Lidar applies authored range noise.

World: noise 0.1 (sigma = 0.10 m ABSOLUTE, no maxRange scaling), fov 1.2 so flank columns
miss the box. Against range[theta] = 0.70/cos(theta) over the hit columns:
  * residual std must be ~0.10 m (accept 0.05..0.20 for n~50),
  * residual mean must be ~0 (accept |mean| < 0.06),
  * miss columns must read +inf (the WREN range-clip semantics the port reproduces).
RED-CAPABLE: without the port (or under OMNISIM_WGPU_SENSOR_POSTFX=0) std ~ 0 and the
flanks read the clamp value 10.0 -> FAIL.

Writes its result to OMNISIM_R33B_RESULT_PATH."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "lane_e1_lidar_noise_smoke.txt",
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
fov = lidar.getFov()
img = lidar.getRangeImage()
if img is None or len(img) < res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {res})\n")
    sys.exit(0)

dtheta = -fov / res
theta0 = fov / 2.0 + dtheta / 2.0
residuals = []
miss_cols = 0
miss_inf = 0
miss_worst = ""
for i in range(res):
    theta = theta0 + i * dtheta
    if abs(theta) < 0.38:  # solid hit, away from the box edge
        expected = 0.70 / math.cos(theta)
        residuals.append(img[i] - expected)
    elif abs(theta) > 0.43:  # solid miss: WREN semantics with noise>0 -> +inf
        miss_cols += 1
        if math.isinf(img[i]):
            miss_inf += 1
        elif not miss_worst:
            miss_worst = f"col {i} theta={theta:+.3f} got={img[i]:.4f} (expected +inf)"

n = len(residuals)
mean = sum(residuals) / n if n else 0.0
std = math.sqrt(sum((r - mean) ** 2 for r in residuals) / n) if n else 0.0

std_ok = 0.05 < std < 0.20
mean_ok = abs(mean) < 0.06
inf_ok = miss_cols > 0 and miss_inf == miss_cols
ok = std_ok and mean_ok and inf_ok and n > 20
tag = "PASS lidar-wgpu-noise" if ok else "OBSERVE"
write_result(
    f"{tag} residual std={std:.4f} m mean={mean:+.4f} m over {n} hit cols; "
    f"miss cols inf {miss_inf}/{miss_cols} {miss_worst}  "
    f"std_ok={std_ok} mean_ok={mean_ok} inf_ok={inf_ok}\n"
)
print(f"[lidar_wgpu_noise_smoke] {tag} std={std:.4f} mean={mean:+.4f} inf={miss_inf}/{miss_cols}")
sys.exit(0)
