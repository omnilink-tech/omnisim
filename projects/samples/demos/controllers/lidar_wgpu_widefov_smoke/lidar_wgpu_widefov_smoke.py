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

"""R5g smoke controller -- verify the wgpu WIDE-FOV (multi-frustum) Lidar path.

The Lidar has fieldOfView 2.0 rad, so the wgpu path stitches 2 sub-frustums
with the seam at azimuth 0. The centred box straddles that seam. Checks:
 1. centre ~0.70 m (box front face).
 2. profile range[theta]=0.70/cos(theta) for |theta|<0.4 -- this spans the
    theta=0 seam, so a stitch discontinuity would blow up max_err.
 3. SEAM continuity: the two columns straddling theta=0 agree to <0.01 m.
 4. wide flanks (|theta|>0.6) read ~maxRange (side frustums see nothing).

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5g_smoke.txt)."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5g_smoke.txt",
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

cx = res // 2
center = img[cx]

# (2) profile across the seam.
max_err = 0.0
worst = ""
checked = 0
# (3) seam: columns straddling theta=0.
seam_lo = None  # last col with theta>0
seam_hi = None  # first col with theta<=0
for i in range(res):
    theta = theta0 + i * dtheta
    if theta > 0:
        seam_lo = i
    elif seam_hi is None:
        seam_hi = i
    if abs(theta) < 0.40:
        expected = 0.70 / math.cos(theta)
        err = abs(img[i] - expected)
        if err > max_err:
            max_err = err
            worst = f"col {i} theta={theta:+.3f} got={img[i]:.4f} exp={expected:.4f}"
        checked += 1

seam_gap = abs(img[seam_lo] - img[seam_hi]) if (seam_lo is not None and seam_hi is not None) else 9.9

# (4) wide flanks see nothing.
flank_ok = True
flank_detail = ""
for i in range(res):
    theta = theta0 + i * dtheta
    if abs(theta) > 0.6:
        if img[i] < maxr * 0.9:
            flank_ok = False
            flank_detail = f"col {i} theta={theta:+.3f} range={img[i]:.3f} (expected ~maxRange)"
            break

center_ok = abs(center - 0.70) < 0.02
profile_ok = max_err < 0.02 and checked > 0
seam_ok = seam_gap < 0.01
ok = center_ok and profile_ok and seam_ok and flank_ok
tag = "PASS lidar-wgpu-widefov" if ok else "OBSERVE"
write_result(
    f"{tag} fov={fov:.2f} center={center:.4f} m  profile max_err={max_err:.4f}/{checked}cols ({worst})  "
    f"seam_gap={seam_gap:.4f}  flank_ok={flank_ok} {flank_detail}  "
    f"center_ok={center_ok} profile_ok={profile_ok} seam_ok={seam_ok}\n"
)
print(f"[lidar_wgpu_widefov_smoke] {tag} center={center:.4f} max_err={max_err:.4f} seam_gap={seam_gap:.4f}")
sys.exit(0)
