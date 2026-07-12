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

"""R5c smoke controller — verify the wgpu RangeFinder device path produces a
metric depth image. With OMNISIM_RANGEFINDER_WGPU=1 the RangeFinder renders
through WbWgpuRenderTarget::clearAndDrawSceneDepthF32; the centre pixel should
read ~0.70 m (the cyan box front face at x=0.7).

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r5c_smoke.txt by
default) so the headless harness can read it after the controller exits."""

import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r5c_smoke.txt",
    ),
)


def write_result(line: str) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
rf = robot.getDevice("rf_wgpu")
rf.enable(ts)

robot.step(ts)
robot.step(ts)
robot.step(ts)

w = rf.getWidth()
h = rf.getHeight()
img = rf.getRangeImage()  # list of w*h floats (metres)
if img is None or len(img) < w * h:
    write_result(f"FAIL getRangeImage() returned {0 if img is None else len(img)} "
                 f"(expected {w * h})\n")
    sys.exit(0)

cx, cy = w // 2, h // 2
center = img[cy * w + cx]
finite = [v for v in img if v == v]  # drop NaN
vmin = min(finite) if finite else float("nan")
vmax = max(finite) if finite else float("nan")

# Expected: centre ~0.70 m (box front face), background = maxRange (10.0).
ok = abs(center - 0.70) < 0.05
tag = "PASS rangefinder-wgpu" if ok else "OBSERVE"
write_result(
    f"{tag} center({cx},{cy})={center:.4f} m  min={vmin:.4f} max={vmax:.4f}  "
    f"({w}x{h}={w*h} px)\n"
)
print(f"[rangefinder_wgpu_smoke] {tag} center={center:.4f} m min={vmin:.4f} max={vmax:.4f}")
sys.exit(0)
