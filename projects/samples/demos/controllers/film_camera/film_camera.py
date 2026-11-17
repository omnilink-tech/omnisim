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
    controllerArgs [ "<period_ms>" "<start_ms>" ]   no frame before start_ms; frames on the grid
                                              start_ms + n * period_ms (omnisim_core.film: a seek shot's
                                              first frame lands on the moment it seeks to)
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
start_ms = int(sys.argv[2]) if len(sys.argv) > 2 else None
# Directory the film_camera controller saves frames into (<dir>/<camera name>/<sim ms>.jpg); unset = film nothing.
out = os.environ.get("OMNISIM_FILM_DIR", "")
cams = [d for d in (robot.getDeviceByIndex(i) for i in range(robot.getNumberOfDevices()))
        if isinstance(d, Camera)]
if not out:
    print("[film_camera] OMNISIM_FILM_DIR is not set: nothing to film", flush=True)
for cam in cams:
    if start_ms is None:
        cam.enable(max(step_ms, period_ms))
    if out:
        os.makedirs(os.path.join(out, cam.getName()), exist_ok=True)
print(f"[film_camera] {len(cams)} camera(s) every {period_ms} sim-ms -> {out or '(off)'}", flush=True)

if start_ms is None:
    next_ms = 0
    while robot.step(step_ms) != -1:
        now_ms = int(round(robot.getTime() * 1000))
        if not out or now_ms < next_ms:
            continue
        for cam in cams:
            cam.saveImage(os.path.join(out, cam.getName(), f"{now_ms:09d}.jpg"), 92)
        next_ms = now_ms + period_ms
else:
    # A camera renders on its own sampling period, so with a period that is not a whole number of steps the
    # image saved at a frame time can be several steps old. Here each camera renders exactly the step a frame
    # is taken on: enabled for that step only, so every frame is the scene at the time in its name.
    next_ms, live = start_ms, False
    while True:
        now_ms = int(round(robot.getTime() * 1000))
        due = bool(out) and now_ms + step_ms >= next_ms
        if due != live:
            for cam in cams:
                cam.enable(step_ms) if due else cam.disable()
            live = due
        if robot.step(step_ms) == -1:
            break
        now_ms = int(round(robot.getTime() * 1000))
        if not due or now_ms < next_ms:
            continue
        for cam in cams:
            cam.saveImage(os.path.join(out, cam.getName(), f"{now_ms:09d}.jpg"), 92)
        while next_ms <= now_ms:          # stay on the grid (a frame late by part of a step does not drift the rest)
            next_ms += period_ms
