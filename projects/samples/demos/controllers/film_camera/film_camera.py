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
"""Film a run: save every Camera on this robot every N simulated milliseconds.

    controllerArgs [ "<period_ms>" ]          default 125 (8 frames per sim second)
    OMNISIM_FILM_DIR=<dir>                    frames land in <dir>/<camera name>/<sim ms>.jpg

Frames are stamped with SIMULATOR time, so an edit can line them up with any
log that records sim time (the ops-bench trace does), and a run that renders
slower than real time still yields an even, correct timeline. A fixed camera
robot never moves and has no physics, so it does not touch the scene it films.
"""
import os
import sys

from omnisim import Camera, Robot

robot = Robot()
step_ms = int(robot.getBasicTimeStep())
period_ms = int(sys.argv[1]) if len(sys.argv) > 1 else 125
# Directory the film_camera controller saves frames into (<dir>/<camera name>/<sim ms>.jpg); unset = film nothing.
out = os.environ.get("OMNISIM_FILM_DIR", "")
cams = [d for d in (robot.getDeviceByIndex(i) for i in range(robot.getNumberOfDevices()))
        if isinstance(d, Camera)]
if not out:
    print("[film_camera] OMNISIM_FILM_DIR is not set: nothing to film", flush=True)
for cam in cams:
    cam.enable(max(step_ms, period_ms))
    if out:
        os.makedirs(os.path.join(out, cam.getName()), exist_ok=True)
print(f"[film_camera] {len(cams)} camera(s) every {period_ms} sim-ms -> {out or '(off)'}", flush=True)

next_ms = 0
while robot.step(step_ms) != -1:
    now_ms = int(round(robot.getTime() * 1000))
    if not out or now_ms < next_ms:
        continue
    for cam in cams:
        cam.saveImage(os.path.join(out, cam.getName(), f"{now_ms:09d}.jpg"), 92)
    next_ms = now_ms + period_ms
