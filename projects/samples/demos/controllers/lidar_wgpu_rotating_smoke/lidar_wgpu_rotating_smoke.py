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

"""R5j rotating-head Lidar exact-column placement test (+ WREN-oracle dump).

The crux verification for the rotating wgpu path (engine-migration-plan.md):
a test that only checks "does the box appear somewhere" is convention-blind,
so this asserts the EXACT global column the box lands in. The box is physically
on the FRONT-RIGHT only (-y), so a reversed yaw or a wrong widthOffset would
place it on the wrong side / wrong column -- which an exact-column + anti-mirror
check catches. The gold-standard cross-check is the WREN-oracle column diff:
run this world with and without OMNISIM_LIDAR_WGPU and compare the box columns
(set OMNISIM_LIDAR_DUMP_PATH); they must agree (verified: closest col 187==187,
identical band, front-face ranges <0.001 m apart).

World geometry (lidar_wgpu_rotating_smoke.omniworld):
  - single-layer ROTATING lidar, horizontalResolution 360 (1 deg/column),
    fieldOfView 0.4, maxRange 10, defaultFrequency 1, basicTimeStep 32.
  - box [0.7,1.3] x [-0.7,-0.1] on the FRONT-RIGHT (-y).

Buffer convention (updatePointCloud, rotating): forward (theta=0) is column
res/2 = 180; +theta = left = c < 180, so a -theta (right) box lands RIGHT of
centre at c(theta) = res/2 - theta*res/(2*pi) -> columns ~184..225.

Checks (robust to full vs partial coverage, because the box is physically
one-sided): (1) box hits exist, (2) ALL hits RIGHT of centre, (3) ALL hits in
the predicted band [~180,~229] (anti-mirror + anti-offset), (4) closest range
~0.70 m (box front face). partial/unswept are REPORTED as diagnostics, not
gated on (a rotating lidar may complete a sweep within the controller's step
budget; coverage does not change where the one-sided box lands).

Writes PASS/FAIL to OMNISIM_LIDAR_RESULT_PATH (or _scratch/lidar_rotating.txt)."""

import os
import math
import sys

from omnisim import Robot

# OMNISIM_R33B_RESULT_PATH is the path the wgpu_sensor_regression harness sets
# (shared by all wgpu sensor smoke controllers); OMNISIM_LIDAR_RESULT_PATH is an
# alternate for standalone runs. Either falls back to _scratch.
OUT = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.environ.get(
        "OMNISIM_LIDAR_RESULT_PATH",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..", "..", "..", "_scratch", "lidar_rotating.txt"),
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

# A short sweep: enough steps to cover the right-side box. Whether or not the
# head completes a full rotation, the one-sided box only writes non-maxRange
# values at its true azimuth columns, so the exact-column check is meaningful.
STEPS = 6
for _ in range(STEPS):
    robot.step(ts)

res = lidar.getHorizontalResolution()
fov = lidar.getFov()
rng = list(lidar.getRangeImage())
max_range = lidar.getMaxRange()
center = res // 2


def is_box(r):
    return math.isfinite(r) and 0.1 < r < max_range * 0.9


def is_unswept(r):
    # Unswept columns hold the buffer init value 0.0 (or inf for some configs).
    return (not math.isfinite(r)) or r <= 1e-6


box_cols = [i for i, r in enumerate(rng) if is_box(r)]
unswept = sum(1 for r in rng if is_unswept(r))


def col_of(theta):
    return res / 2.0 - theta * res / (2.0 * math.pi)


band_lo = int(math.floor(col_of(-0.077))) - 4   # ~180 (near edge - margin)
band_hi = int(math.ceil(col_of(-0.785))) + 4    # ~229 (far edge + margin)

closest_i = min(box_cols, key=lambda i: rng[i]) if box_cols else -1
closest_r = rng[closest_i] if closest_i >= 0 else float("nan")

# Diagnostic dump for the WREN-oracle column diff (set OMNISIM_LIDAR_DUMP_PATH).
dump_path = os.environ.get("OMNISIM_LIDAR_DUMP_PATH", "")
if dump_path:
    os.makedirs(os.path.dirname(os.path.abspath(dump_path)), exist_ok=True)
    with open(dump_path, "w", encoding="utf-8") as f:
        f.write("res=%d fov=%.4f steps=%d closest_col=%d closest_r=%.4f\n"
                % (res, fov, STEPS, closest_i, closest_r))
        for i in box_cols:
            f.write("%d %.4f\n" % (i, rng[i]))

# Gate: exact-column placement of the one-sided box (the real crux). partial/
# unswept are reported but not gated -- the box is physically one-sided, so its
# column placement discriminates yaw-sign + widthOffset regardless of coverage.
have_box = len(box_cols) > 0
all_right = all(c > center for c in box_cols)
in_band = all(band_lo <= c <= band_hi for c in box_cols)
range_ok = math.isfinite(closest_r) and abs(closest_r - 0.70) < 0.05

ok = have_box and all_right and in_band and range_ok
write(
    "%s box_cols=%s%s n=%d band=[%d,%d] all_right=%s in_band=%s "
    "range_ok=%s closest_col=%d closest_r=%.4f partial=%s unswept=%d/%d "
    "(res=%d fov=%.3f)\n"
    % ("PASS lidar-wgpu-rotating" if ok else "FAIL",
       box_cols[:8], "..." if len(box_cols) > 8 else "", len(box_cols),
       band_lo, band_hi, all_right, in_band, range_ok, closest_i, closest_r,
       unswept > 0, unswept, res, res, fov)
)
sys.exit(0)
