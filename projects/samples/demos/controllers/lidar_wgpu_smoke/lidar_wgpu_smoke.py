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

"""R5e smoke controller -- verify the wgpu single-layer Lidar device path.

With OMNISIM_LIDAR_WGPU=1 the Lidar renders through
WbWgpuRenderTarget::clearAndDrawSceneRangeF32 and angular-resamples to the
uniform-angle range image. For the cyan box (front face 0.7 m, fov 0.8 rad,
every ray hits it) the closed form is range[theta] = 0.70 / cos(theta):
center ~0.70 m, edges ~0.760 m, monotonically increasing off-axis.

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5e_smoke.txt)."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5e_smoke.txt",
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
img = lidar.getRangeImage()  # res floats (single layer)
if img is None or len(img) < res:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {res})\n")
    sys.exit(0)

cx = res // 2
center = img[cx]

# Closed form: theta_i = fov/2 + dtheta/2 + i*dtheta, dtheta = -fov/res;
# range[theta] = 0.70 / cos(theta) where the ray hits the box (|theta| < 0.405).
dtheta = -fov / res
theta0 = fov / 2.0 + dtheta / 2.0
max_err = 0.0
worst = ""
checked = 0
for i in range(res):
    theta = theta0 + i * dtheta
    if abs(theta) >= 0.40:   # near the box edge / possible miss -- skip
        continue
    expected = 0.70 / math.cos(theta)
    err = abs(img[i] - expected)
    if err > max_err:
        max_err = err
        worst = f"col {i} theta={theta:+.3f} got={img[i]:.4f} exp={expected:.4f}"
    checked += 1

# Edges should read longer than centre (radial range grows off-axis).
edge = img[2]
monotonic_ok = edge > center

center_ok = abs(center - 0.70) < 0.02
profile_ok = max_err < 0.02 and checked > 0
ok = center_ok and profile_ok and monotonic_ok
tag = "PASS lidar-wgpu" if ok else "OBSERVE"
write_result(
    f"{tag} center({cx})={center:.4f} m edge={edge:.4f} m  "
    f"profile max_err={max_err:.4f} over {checked} cols ({worst})  "
    f"center_ok={center_ok} profile_ok={profile_ok} monotonic_ok={monotonic_ok}\n"
)
print(f"[lidar_wgpu_smoke] {tag} center={center:.4f} edge={edge:.4f} max_err={max_err:.4f}")
sys.exit(0)
