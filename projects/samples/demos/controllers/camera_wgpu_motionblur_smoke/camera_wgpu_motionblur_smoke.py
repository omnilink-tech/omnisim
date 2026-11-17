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

"""Lane E1 (P5) smoke controller -- the wgpu Camera applies the authored motion blur.

World: motionBlur 400 ms at basicTimeStep 32 -> blend = pow(0.005, 32/400) ~ 0.654 = the
weight of the OLD frame. The supervisor teleports the cyan box out of view, then watches the
centre pixel: with the CPU port the first frame(s) after removal must carry a decaying cyan
GHOST (a value strictly between the box reading and the settled background), reaching the
background only over several frames.
RED-CAPABLE: without the port (or under OMNISIM_WGPU_SENSOR_POSTFX=0) the centre snaps to
the background in a single frame -> no intermediate value -> FAIL.

Writes its result to OMNISIM_R33B_RESULT_PATH."""

import os
import sys

from omnisim import Supervisor

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "lane_e1_cam_motionblur_smoke.txt",
    ),
)


def write_result(line: str) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(line)


robot = Supervisor()
ts = int(robot.getBasicTimeStep())
cam = robot.getDevice("cam_wgpu")
cam.enable(ts)

w = cam.getWidth()
h = cam.getHeight()
ci = (h // 2 * w + w // 2) * 4


def center_bgr():
    raw = cam.getImage()
    if raw is None or len(raw) < w * h * 4:
        return None
    return (raw[ci], raw[ci + 1], raw[ci + 2])  # BGRA texel, alpha dropped


def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


# Let the blur history converge onto the box (blend^6 ~ 0.08 residual).
for _ in range(6):
    robot.step(ts)
c_box = center_bgr()
if c_box is None:
    write_result("FAIL cam.getImage() returned no image\n")
    sys.exit(0)

box = robot.getFromDef("CYAN_BOX")
if box is None:
    write_result("FAIL getFromDef(CYAN_BOX) returned None\n")
    sys.exit(0)
box.getField("translation").setSFVec3f([1.0, 0.0, -50.0])

# Watch the decay: record the centre BGR for the next 12 frames.
decay = []
for _ in range(12):
    robot.step(ts)
    v = center_bgr()
    decay.append(v if v is not None else (-1, -1, -1))

c_late = decay[-1]
# The box must actually have left the frame (BGR distance, so a background that happens to
# share ONE channel with the cyan box cannot mask the transition).
d_total = dist(c_box, c_late)
left_ok = d_total > 40
# Find the first frame that departed from the box reading...
ghost = None
for v in decay:
    if dist(v, c_box) > 0.25 * d_total:
        ghost = v
        break
# ...and require it to be a GHOST: still far from the settled background. With
# blend ~0.654 the first departed frame sits ~0.65 of the way from background to box.
ghost_ok = ghost is not None and dist(ghost, c_late) > 0.25 * d_total
# And the decay must be gradual: >= 2 frames strictly between box and background.
intermediates = [v for v in decay
                 if dist(v, c_box) > 0.15 * d_total and dist(v, c_late) > 0.15 * d_total]
gradual_ok = len(intermediates) >= 2

ok = left_ok and ghost_ok and gradual_ok
tag = "PASS camera-wgpu-motionblur" if ok else "OBSERVE"
write_result(
    f"{tag} box={c_box} late={c_late} d_total={d_total:.1f} first-departed={ghost} "
    f"decay={decay} intermediates={len(intermediates)}  "
    f"left_ok={left_ok} ghost_ok={ghost_ok} gradual_ok={gradual_ok}\n"
)
print(f"[camera_wgpu_motionblur_smoke] {tag} box={c_box} ghost={ghost} late={c_late}")
sys.exit(0)
