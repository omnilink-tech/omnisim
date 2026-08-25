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

"""R5h smoke controller -- verify multi-layer + wide-FOV Lidar together.

Combines the R5g (multi-frustum azimuth stitch) and R5f (per-layer vertical
resample + orientation) checks on one 7-layer, fov=2.0 (2-frustum) Lidar facing
an upward-offset box:
 1. centre pixel (centre layer j=3, centre col) ~ 0.70 m.
 2. centre layer (phi=0) horizontal profile matches 0.70/cos(theta) ACROSS the
    theta=0 frustum seam -> validates the wide stitch on the centre layer.
 3. ORIENTATION: at the centre column, more layers ABOVE centre see the box than
    BELOW (box offset +z) -> validates the vertical resample inside the wide path.

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5h_smoke.txt)."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5h_smoke.txt",
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
layers = lidar.getNumberOfLayers()
fov = lidar.getFov()
maxr = lidar.getMaxRange()
img = lidar.getRangeImage()
if img is None or len(img) < layers * res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {layers*res})\n")
    sys.exit(0)

cx = res // 2
cl = layers // 2
center = img[cl * res + cx]

dtheta = -fov / res
theta0 = fov / 2.0 + dtheta / 2.0
max_err = 0.0
worst = ""
checked = 0
for i in range(res):
    theta = theta0 + i * dtheta
    if abs(theta) >= 0.40:
        continue
    expected = 0.70 / math.cos(theta)
    err = abs(img[cl * res + i] - expected)
    if err > max_err:
        max_err = err
        worst = f"col {i} theta={theta:+.3f} got={img[cl*res+i]:.4f} exp={expected:.4f}"
    checked += 1


def hit(j):
    return img[j * res + cx] < maxr * 0.9


above = sum(1 for j in range(cl) if hit(j))
below = sum(1 for j in range(cl + 1, layers) if hit(j))

center_ok = abs(center - 0.70) < 0.02
profile_ok = max_err < 0.02 and checked > 0
orient_ok = above > below
ok = center_ok and profile_ok and orient_ok
tag = "PASS lidar-wgpu-ml-widefov" if ok else "OBSERVE"
write_result(
    f"{tag} layers={layers} fov={fov:.2f} center(L{cl},{cx})={center:.4f} m  "
    f"profile max_err={max_err:.4f}/{checked}cols ({worst})  "
    f"orient above={above} below={below}  "
    f"center_ok={center_ok} profile_ok={profile_ok} orient_ok={orient_ok}\n"
)
print(f"[lidar_wgpu_ml_widefov_smoke] {tag} center={center:.4f} max_err={max_err:.4f} "
      f"above={above} below={below}")
sys.exit(0)
