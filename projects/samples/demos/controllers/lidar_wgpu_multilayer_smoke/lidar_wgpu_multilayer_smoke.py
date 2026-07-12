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

"""R5f smoke controller -- verify the wgpu MULTI-LAYER Lidar device path.

Three checks (with OMNISIM_LIDAR_WGPU=1):
 1. centre pixel (centre layer j=3, centre col) ~ 0.70 m (box front face).
 2. centre layer (phi=0) horizontal profile matches range=0.70/cos(theta) --
    validates the horizontal angular resample exactly.
 3. ORIENTATION: the box is offset upward (z=0.15), so more layers ABOVE the
    centre see it than BELOW. A vertical layer-flip would reverse this, so
    asserting above_hits > below_hits validates the ndc_y->row orientation
    (which a vertically-symmetric box could not).

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5f_smoke.txt)."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5f_smoke.txt",
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
img = lidar.getRangeImage()  # layers*res floats, row-major data[layer*res + col]
if img is None or len(img) < layers * res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {layers*res})\n")
    sys.exit(0)

cx = res // 2
cl = layers // 2  # centre layer (odd layer count -> exact phi=0)
center = img[cl * res + cx]

# (2) centre-layer (phi=0) horizontal profile -> 0.70 / cos(theta).
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


# (3) orientation: at centre col, hits above centre vs below.
def hit(j):
    return img[j * res + cx] < 5.0


above = sum(1 for j in range(cl) if hit(j))                  # j<cl: top, +phi
below = sum(1 for j in range(cl + 1, layers) if hit(j))      # j>cl: bottom, -phi

center_ok = abs(center - 0.70) < 0.02
profile_ok = max_err < 0.02 and checked > 0
orient_ok = above > below
ok = center_ok and profile_ok and orient_ok
tag = "PASS lidar-wgpu-multilayer" if ok else "OBSERVE"
write_result(
    f"{tag} layers={layers} center(L{cl},{cx})={center:.4f} m  "
    f"profile max_err={max_err:.4f} over {checked} cols ({worst})  "
    f"orient above_hits={above} below_hits={below}  "
    f"center_ok={center_ok} profile_ok={profile_ok} orient_ok={orient_ok}\n"
)
print(f"[lidar_wgpu_multilayer_smoke] {tag} center={center:.4f} "
      f"max_err={max_err:.4f} above={above} below={below}")
sys.exit(0)
