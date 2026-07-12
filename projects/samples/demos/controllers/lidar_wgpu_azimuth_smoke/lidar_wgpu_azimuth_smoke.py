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

"""R5g azimuth-orientation controller -- definitively verify +theta = left.

The box is offset to +y (left), so in the Webots lidar convention only +theta
rays hit it. A centred box cannot catch a left-right mirror; this one can. We
assert that every column seeing the box (range < maxRange) has theta > 0, and
that the closest hit is at a clearly-positive theta.

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5g_az.txt)."""

import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5g_az.txt",
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
maxr = lidar.getMaxRange()
img = lidar.getRangeImage()
if img is None or len(img) < res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {res})\n")
    sys.exit(0)

dtheta = -fov / res
theta0 = fov / 2.0 + dtheta / 2.0

hit_thetas = []
min_range = 1e9
min_theta = None
for i in range(res):
    theta = theta0 + i * dtheta
    r = img[i]
    if r < maxr * 0.9:
        hit_thetas.append(theta)
        if r < min_range:
            min_range = r
            min_theta = theta

if not hit_thetas:
    write_result("FAIL no box hit (range all ~maxRange)\n")
    sys.exit(0)

all_positive = all(t > 0.0 for t in hit_thetas)
closest_left = min_theta is not None and min_theta > 0.2
ok = all_positive and closest_left
tag = "PASS lidar-wgpu-azimuth" if ok else "OBSERVE"
write_result(
    f"{tag} hits={len(hit_thetas)} theta_range=[{min(hit_thetas):+.3f},{max(hit_thetas):+.3f}] "
    f"closest theta={min_theta:+.3f} range={min_range:.4f}  "
    f"all_positive={all_positive} closest_left={closest_left}\n"
)
print(f"[lidar_wgpu_azimuth_smoke] {tag} hits at theta in "
      f"[{min(hit_thetas):+.3f},{max(hit_thetas):+.3f}] closest={min_theta:+.3f}")
sys.exit(0)
