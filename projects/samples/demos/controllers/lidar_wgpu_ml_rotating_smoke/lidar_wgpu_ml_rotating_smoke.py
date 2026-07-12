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

"""R5k multi-layer ROTATING Lidar test: exact-column azimuth + layer orientation.

Composes the two earlier rotating/multi-layer verification ideas on one world
(lidar_wgpu_ml_rotating_smoke.wbt): a 7-layer rotating lidar + a box that is both
front-RIGHT (one-sided azimuth) and offset UP (asymmetric elevation).

  - 360-column rotating buffer: forward (theta=0) is column res/2=180; +theta =
    left = c < 180. The right box lands at columns > 180, in the band
    c(theta) = res/2 - theta*res/(2pi) for theta in [-0.5..-0.07] -> ~184..209.
  - 7 layers, verticalFieldOfView 0.8: layer 0 = top (+phi), layer 6 = bottom.
    The up-offset box (elevation centre +0.245) is seen by MORE layers above the
    centre layer (j<3) than below (j>3).

Checks (with OMNISIM_LIDAR_WGPU=1):
 (1) box hits exist on the centre layer,
 (2) ALL centre-layer box hits are RIGHT of centre and in the predicted band
     (azimuth placement -- anti-mirror + anti-offset, the R5j crux),
 (3) centre-layer closest range ~0.70..0.78 m (front face, 0.70/cos(theta)),
 (4) ORIENTATION: at the box's azimuth column, more layers above centre see the
     box than below (the R5f vertical-resample orientation),
so both the azimuth stitch and the per-layer elevation hold simultaneously.

Writes PASS/FAIL to OMNISIM_R33B_RESULT_PATH (or OMNISIM_LIDAR_RESULT_PATH,
or _scratch/lidar_ml_rotating.txt)."""

import os
import math
import sys

from omnisim import Robot

OUT = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.environ.get(
        "OMNISIM_LIDAR_RESULT_PATH",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..", "..", "..", "_scratch", "lidar_ml_rotating.txt"),
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


# (1)+(2): centre-layer azimuth placement.
cl_box_cols = [i for i in range(res) if is_box(img[center_layer * res + i])]


def col_of(theta):
    return res / 2.0 - theta * res / (2.0 * math.pi)


# Box azimuth extent from its true corners: near edge atan2(-0.1, 1.3) = -0.077,
# far edge atan2(-0.7, 0.7) = -0.785 (the rotating head sweeps across both).
band_lo = int(math.floor(col_of(-0.077))) - 5   # near edge - margin (~179)
band_hi = int(math.ceil(col_of(-0.785))) + 6    # far edge + margin (~231)

have_box = len(cl_box_cols) > 0
all_right = all(c > center_col for c in cl_box_cols)
in_band = all(band_lo <= c <= band_hi for c in cl_box_cols)

closest_i = min(cl_box_cols, key=lambda i: img[center_layer * res + i]) if cl_box_cols else -1
closest_r = img[center_layer * res + closest_i] if closest_i >= 0 else float("nan")
range_ok = math.isfinite(closest_r) and 0.66 < closest_r < 0.80

# (4): orientation at the box column -- more layers above centre than below.
col = closest_i if closest_i >= 0 else center_col
above = sum(1 for j in range(center_layer) if is_box(img[j * res + col]))
below = sum(1 for j in range(center_layer + 1, layers) if is_box(img[j * res + col]))
orient_ok = above > below

ok = have_box and all_right and in_band and range_ok and orient_ok
write(
    "%s layers=%d cl_box_cols=%s%s n=%d band=[%d,%d] all_right=%s in_band=%s "
    "range_ok=%s closest_col=%d closest_r=%.4f orient above=%d below=%d orient_ok=%s "
    "(res=%d fov=%.3f)\n"
    % ("PASS lidar-wgpu-ml-rotating" if ok else "FAIL",
       layers, cl_box_cols[:8], "..." if len(cl_box_cols) > 8 else "", len(cl_box_cols),
       band_lo, band_hi, all_right, in_band, range_ok, closest_i, closest_r,
       above, below, orient_ok, res, fov)
)
sys.exit(0)
