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

"""R5l wide-FOV multi-layer ROTATING Lidar test (the last Lidar config on wgpu).

Composes the R5g multi-frustum azimuth stitch with the R5j/R5k rotating window:
wide fieldOfView (2.0 rad -> 2 sub-frustums), 7 layers, rotating head, with a box
on the front-RIGHT (-y) + offset UP (z=0.25). The same discriminants as R5k apply
-- if the multi-frustum stitch or the rotating placement were wrong, the box
would land in the wrong column or wrong side.

World (lidar_wgpu_wide_rotating_smoke.omniworld): res 360 (1 deg/col), fov 2.0,
verticalFieldOfView 0.8, 7 layers, maxRange 10, rotating, defaultFrequency 1.

Buffer convention (updatePointCloud, rotating): forward (theta=0) is column
res/2 = 180; +theta = left = c < 180, so a -theta (right) box lands RIGHT of
centre at c(theta) = res/2 - theta*res/(2*pi).

Checks (with OMNISIM_LIDAR_WGPU=1):
 (1) centre-layer box hits exist,
 (2) ALL centre-layer box hits are RIGHT of centre and in the predicted band
     (azimuth placement through the multi-frustum stitch),
 (3) centre-layer closest range ~0.66..0.80 m (front face),
 (4) ORIENTATION: at the box column, more layers above centre see it than below
     (the up-offset box, per-layer elevation resample).

Writes PASS/FAIL to OMNISIM_R33B_RESULT_PATH (or OMNISIM_LIDAR_RESULT_PATH, or
_scratch/lidar_wide_rotating.txt)."""

import os
import math
import sys

from omnisim import Robot

OUT = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.environ.get(
        "OMNISIM_LIDAR_RESULT_PATH",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..", "..", "..", "_scratch", "lidar_wide_rotating.txt"),
    ),
)


def write(line):
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
lidar = robot.getDevice("lidar_wgpu")
lidar.enable(ts)
lidar.enablePointCloud()

# Sweep long enough for the head to cover the right-side box.
STEPS = 10
for _ in range(STEPS):
    robot.step(ts)

res = lidar.getHorizontalResolution()
layers = lidar.getNumberOfLayers()
fov = lidar.getFov()
mr = lidar.getMaxRange()
img = list(lidar.getRangeImage())   # layers*res floats, data[layer*res + col]
if len(img) < layers * res:
    write("FAIL getRangeImage() returned %d (expected %d)\n" % (len(img), layers * res))
    sys.exit(0)

center_layer = layers // 2   # j=3 for 7 layers -> phi ~ 0
center_col = res // 2        # forward


def is_box(r):
    return math.isfinite(r) and 0.1 < r < mr * 0.9


cl_box_cols = [i for i in range(res) if is_box(img[center_layer * res + i])]


def col_of(theta):
    return res / 2.0 - theta * res / (2.0 * math.pi)


# Box azimuth extent from its true corners: near atan2(-0.1,1.3)=-0.077,
# far atan2(-0.7,0.7)=-0.785.
band_lo = int(math.floor(col_of(-0.077))) - 5   # ~179
band_hi = int(math.ceil(col_of(-0.785))) + 6    # ~231

have_box = len(cl_box_cols) > 0
all_right = all(c > center_col for c in cl_box_cols)
in_band = all(band_lo <= c <= band_hi for c in cl_box_cols)

closest_i = min(cl_box_cols, key=lambda i: img[center_layer * res + i]) if cl_box_cols else -1
closest_r = img[center_layer * res + closest_i] if closest_i >= 0 else float("nan")
range_ok = math.isfinite(closest_r) and 0.66 < closest_r < 0.80

col = closest_i if closest_i >= 0 else center_col
above = sum(1 for j in range(center_layer) if is_box(img[j * res + col]))
below = sum(1 for j in range(center_layer + 1, layers) if is_box(img[j * res + col]))
orient_ok = above > below

ok = have_box and all_right and in_band and range_ok and orient_ok
write(
    "%s layers=%d fov=%.2f cl_box_cols=%s%s n=%d band=[%d,%d] all_right=%s in_band=%s "
    "range_ok=%s closest_col=%d closest_r=%.4f orient above=%d below=%d orient_ok=%s "
    "(res=%d)\n"
    % ("PASS lidar-wgpu-wide-rotating" if ok else "FAIL",
       layers, fov, cl_box_cols[:8], "..." if len(cl_box_cols) > 8 else "", len(cl_box_cols),
       band_lo, band_hi, all_right, in_band, range_ok, closest_i, closest_r,
       above, below, orient_ok, res)
)
sys.exit(0)
