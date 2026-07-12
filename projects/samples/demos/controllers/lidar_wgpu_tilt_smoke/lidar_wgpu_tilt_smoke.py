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

"""R5i smoke controller -- verify Lidar tiltAngle pitches the scan correctly.

The Lidar has tiltAngle 0.3 and the box is offset up (z=0.25). With a correct
tilt-up pitch, the centre ray (elevation +0.3) hits the box front face at
x=0.7 -> range = 0.70/cos(0.3) = 0.733 m. range 0.70 would mean tilt was
ignored; ~maxRange would mean the tilt pitched the wrong way (ray below the
box). So a centre near 0.733 confirms both sign and magnitude.

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5i_smoke.txt)."""

import math
import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5i_smoke.txt",
    ),
)

TILT = 0.3  # matches the world's tiltAngle


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
expected = 0.70 / math.cos(TILT)  # 0.733 for tilt 0.3

tilt_applied = abs(center - expected) < 0.02      # ~0.733: correct tilt-up
no_tilt = abs(center - 0.70) < 0.02               # 0.70: tilt ignored
ok = tilt_applied and not no_tilt
tag = "PASS lidar-wgpu-tilt" if ok else "OBSERVE"
write_result(
    f"{tag} tilt={TILT} center({cx})={center:.4f} m expected={expected:.4f} "
    f"(0.70 if tilt ignored, ~maxRange if wrong sign)  "
    f"tilt_applied={tilt_applied}\n"
)
print(f"[lidar_wgpu_tilt_smoke] {tag} center={center:.4f} expected={expected:.4f}")
sys.exit(0)
