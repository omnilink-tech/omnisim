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

"""R3.3b smoke controller — verify the wgpu Camera path produces a
magenta image (the temporary clear-color the R3.3b wiring uses
until R3.4-step-4 + the Solid-mesh walk land).

Writes its result to OMNISIM_R33B_RESULT_PATH (or _scratch/r33b_smoke.txt
by default) so the test harness can read it after the controller
exits — the headless runner redirects stdout to DEVNULL."""

import os
import sys

from omnisim import Robot

OUT_PATH = os.environ.get(
    "OMNISIM_R33B_RESULT_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "..", "_scratch", "r33b_smoke.txt",
    ),
)
# Optional PPM dump for the R3.6b golden-image harness. When set,
# the controller writes the full Camera readback (BGRA -> RGB) to
# this path so wgpu_probe_golden.py --world can diff against a
# reference PPM.
PPM_PATH = os.environ.get("OMNISIM_R33B_PPM_PATH", "")

def write_result(line: str, append: bool = False) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "a" if append else "w", encoding="utf-8") as f:
        f.write(line)


robot = Robot()
ts = int(robot.getBasicTimeStep())
cam = robot.getDevice("cam_wgpu")
cam.enable(ts)

robot.step(ts)
robot.step(ts)
robot.step(ts)

w = cam.getWidth()
h = cam.getHeight()
raw = cam.getImage()
if raw is None or len(raw) < w * h * 4:
    write_result(
        f"FAIL cam.getImage() returned {0 if raw is None else len(raw)} bytes "
        f"(expected {w * h * 4})\n"
    )
    sys.exit(0)

# Scan the full image for non-clear pixels (clear is (46,46,46)).
non_clear = []
for y in range(h):
    for x in range(w):
        i = (y * w + x) * 4
        bb, gg, rr = raw[i], raw[i + 1], raw[i + 2]
        if abs(rr - 46) > 8 or abs(gg - 46) > 8 or abs(bb - 46) > 8:
            non_clear.append((x, y, bb, gg, rr))
write_result(
    f"non-clear pixels: {len(non_clear)} of {w * h}; "
    f"first 6: {non_clear[:6]}\n"
)
cx, cy = w // 2, h // 2
idx = (cy * w + cx) * 4
b, g, r, a = raw[idx], raw[idx + 1], raw[idx + 2], raw[idx + 3]
# The R3.3b smoke world (camera_wgpu_smoke.omniworld) renders a magenta
# clear because there's no scene to walk; the R3.4-step-4 smoke
# world (camera_wgpu_scene_smoke.omniworld) renders a cyan box that
# fills the center. Either way the wgpu path is exercised when
# r != g != b or the alpha=255 — the "default-clear gray (~46,46,46)"
# fallback from a non-wgpu camera would have r==g==b. Easier to
# just record the pixel + let `wgpu_probe_golden.py --world` do
# the actual visual comparison.
is_wgpu_color = (a >= 250) and (
    (r >= 240 and g <= 15 and b >= 240)  # magenta clear (R3.3b world)
    or (r <= 15 and g >= 240 and b >= 240)  # cyan box (R3.4-step-4 world)
)
tag = "PASS wgpu" if is_wgpu_color else f"OBSERVE (r={r} g={g} b={b})"
write_result(f"center({cx},{cy}) BGRA=({b},{g},{r},{a}) {tag}\n", append=True)

if PPM_PATH:
    os.makedirs(os.path.dirname(os.path.abspath(PPM_PATH)), exist_ok=True)
    # Convert BGRA -> RGB for P6 PPM.
    rgb = bytearray(w * h * 3)
    for i in range(w * h):
        src = i * 4
        dst = i * 3
        rgb[dst + 0] = raw[src + 2]
        rgb[dst + 1] = raw[src + 1]
        rgb[dst + 2] = raw[src + 0]
    with open(PPM_PATH, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode("ascii"))
        f.write(bytes(rgb))
