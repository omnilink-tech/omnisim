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

"""P1/P9 gate controller -- sample a wgpu Camera at several well-separated steps.

Drives the gate worlds for the WREN-deletion runbook's P1 (deformables on the sensor
path) and P9 (granular particles on the sensor path). It exists because the assertion
that matters cannot be made from ONE frame:

  * "the sensor sees the cloth" is a cloth-vs-no-cloth comparison, and
  * "the sensor sees the cloth MOVING" is a frame-vs-frame comparison.

A sheet frozen at its rest pose passes the first and fails the second, and that is
exactly what a shared static "has the clock advanced" flag produces once a second
renderer exists (see OmWgpuMeshCache::vertexEpochIs). So this writes the RAW readback at
each sample step, not a summary, and lets the harness do the arithmetic.

Outputs, into $OMNISIM_CAM_SAMPLE_DIR (default _scratch/cam_samples):
  sample_<step>.ppm   binary P6, the camera readback converted BGRA -> RGB
  samples.txt         one line per sample: step, sim time, mean RGB, checksum, and the
                      count of pixels that are neither floor-grey nor sky

The camera device name is $OMNISIM_CAM_SAMPLE_DEVICE (default "cloth_cam") and the
sample steps are $OMNISIM_CAM_SAMPLE_STEPS (default "40,120,240").
"""

import os
import sys

from omnisim import Robot

OUT_DIR = os.environ.get("OMNISIM_CAM_SAMPLE_DIR") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "_scratch", "cam_samples",
)
DEVICE = os.environ.get("OMNISIM_CAM_SAMPLE_DEVICE", "cloth_cam")
STEPS = [int(s) for s in os.environ.get("OMNISIM_CAM_SAMPLE_STEPS", "40,120,240").split(",") if s.strip()]

os.makedirs(OUT_DIR, exist_ok=True)
LOG = os.path.join(OUT_DIR, "samples.txt")


def note(line):
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


# Truncate any previous run's log so a stale line can never be read as this run's result.
with open(LOG, "w", encoding="utf-8") as _f:
    _f.write("")

robot = Robot()
ts = int(robot.getBasicTimeStep())
cam = robot.getDevice(DEVICE)
if cam is None:
    note("FAIL no device %s" % DEVICE)
    sys.exit(0)
cam.enable(ts)

w = cam.getWidth()
h = cam.getHeight()
note("info device=%s w=%d h=%d timestep=%d steps=%s" % (DEVICE, w, h, ts, STEPS))

step = 0
last = max(STEPS) if STEPS else 0
while robot.step(ts) != -1:
    step += 1
    if step in STEPS:
        raw = cam.getImage()
        if raw is None or len(raw) < w * h * 4:
            note("step=%d FAIL bytes=%d" % (step, 0 if raw is None else len(raw)))
        else:
            rgb = bytearray(w * h * 3)
            rs = gs = bs = 0
            checksum = 0
            for i in range(w * h):
                b = raw[i * 4]
                g = raw[i * 4 + 1]
                r = raw[i * 4 + 2]
                rgb[i * 3] = r
                rgb[i * 3 + 1] = g
                rgb[i * 3 + 2] = b
                rs += r
                gs += g
                bs += b
                # Order-sensitive rolling checksum: two frames with the same histogram but
                # different geometry (a sheet that MOVED) must not collide.
                checksum = (checksum * 131 + r * 7 + g * 13 + b * 17 + i) & 0xFFFFFFFF
            n = float(w * h)
            ppm = os.path.join(OUT_DIR, "sample_%04d.ppm" % step)
            with open(ppm, "wb") as f:
                f.write(("P6\n%d %d\n255\n" % (w, h)).encode("ascii"))
                f.write(bytes(rgb))
            note("step=%d t=%.3f mean=%.3f,%.3f,%.3f checksum=%d ppm=%s"
                 % (step, robot.getTime(), rs / n, gs / n, bs / n, checksum, os.path.basename(ppm)))
    if step >= last:
        # Keep stepping (the runner owns the run length) but stop paying for readbacks.
        cam.disable()
        note("done last_sample_step=%d" % last)
        break

while robot.step(ts) != -1:
    pass
