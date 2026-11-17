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

"""Purposeful main-view coverage for the adaptive interception Agent Build film.

The controller is inert unless ``OMNISIM_INTERCEPT_CAPTURE_DIR`` is set.  Its
story profile uses direct cuts between locked compositions: a workspace master,
a thrower detail, a bird's-eye flight reference, and a catcher detail.  No shot
tracks or orbits for decoration.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from omnisim import Supervisor


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def axis_angle_to_target(position, target):
    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    norm = math.sqrt(fx * fx + fy * fy + fz * fz)
    if norm < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    fx, fy, fz = fx / norm, fy / norm, fz / norm
    ux, uy, uz = 0.0, 0.0, 1.0
    dot = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - dot * fx, uy - dot * fy, uz - dot * fz
    unorm = math.sqrt(ux * ux + uy * uy + uz * uz)
    if unorm < 1e-9:
        ux, uy, uz, unorm = 0.0, 1.0, 0.0, 1.0
    ux, uy, uz = ux / unorm, uy / unorm, uz / unorm
    yx, yy, yz = uy * fz - uz * fy, uz * fx - ux * fz, ux * fy - uy * fx
    matrix = ((fx, yx, ux), (fy, yy, uy), (fz, yz, uz))
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    angle = math.acos(clamp((trace - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    denom = 2.0 * math.sin(angle)
    return [
        (matrix[2][1] - matrix[1][2]) / denom,
        (matrix[0][2] - matrix[2][0]) / denom,
        (matrix[1][0] - matrix[0][1]) / denom,
        angle,
    ]


def find_viewpoint(robot):
    root = robot.getRoot()
    children = root.getField("children") if root is not None else None
    for index in range(children.getCount() if children is not None else 0):
        node = children.getMFNode(index)
        if node is not None and node.getTypeName() == "Viewpoint":
            return node
    return None


def composition(profile, sim_time):
    wide = ([2.55, -3.75, 2.15], [0.72, 0.00, 0.62], "workspace_master")
    thrower = ([1.55, -2.15, 1.55], [0.12, 0.00, 0.68], "thrower_detail")
    overhead = ([0.75, -0.60, 3.85], [0.75, 0.00, 0.42], "flight_birdseye")
    # The disturbed part travels toward +y.  Looking back from +y keeps the
    # orange payload on the camera side of the catcher instead of hiding it
    # behind the upper arm during the catch and trophy lift.
    catcher = ([2.45, 2.20, 1.55], [1.10, 0.20, 0.60], "catcher_detail")
    if profile == "wide":
        return wide
    if profile == "overhead":
        return overhead
    if profile == "detail":
        return thrower if sim_time < 8.65 else catcher
    if sim_time < 5.0:
        return wide
    if sim_time < 7.70:
        return thrower
    if sim_time < 8.72:
        return overhead
    return catcher


robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())
capture_value = os.environ.get("OMNISIM_INTERCEPT_CAPTURE_DIR", "").strip()
if not capture_value:
    while robot.step(dt_ms) != -1:
        pass
    raise SystemExit(0)

capture_dir = Path(capture_value).resolve()
capture_dir.mkdir(parents=True, exist_ok=True)
profile = os.environ.get("OMNISIM_INTERCEPT_CAMERA_PROFILE", "story").strip().lower()
if profile not in {"story", "wide", "overhead", "detail"}:
    profile = "story"
fps = max(1, int(os.environ.get("OMNISIM_INTERCEPT_CAPTURE_FPS", "30")))
acceleration = max(0.25, float(os.environ.get("OMNISIM_INTERCEPT_CAPTURE_ACCELERATION", "1")))
start_s = max(0.0, float(os.environ.get("OMNISIM_INTERCEPT_CAPTURE_START_S", "2.0")))
end_s = max(start_s + 1.0, float(os.environ.get("OMNISIM_INTERCEPT_CAPTURE_END_S", "14.0")))
autoquit = os.environ.get("OMNISIM_INTERCEPT_FILM_AUTOQUIT", "1") != "0"
viewpoint = find_viewpoint(robot)
if viewpoint is None:
    print("[intercept_camera] no Viewpoint", flush=True)
    raise SystemExit(1)

next_frame_s = start_s
frame_index = 0
cuts = []
last_shot = None
while robot.step(dt_ms) != -1:
    sim_time = float(robot.getTime())
    position, target, shot = composition(profile, sim_time)
    if shot != last_shot:
        viewpoint.getField("position").setSFVec3f(position)
        viewpoint.getField("orientation").setSFRotation(
            axis_angle_to_target(position, target)
        )
        cuts.append({"sim_time_s": round(sim_time, 3), "shot": shot,
                     "position": position, "target": target})
        last_shot = shot
        print(f"[intercept_camera] DIRECT_CUT {shot} t={sim_time:.3f}", flush=True)
    if sim_time + 1e-9 >= next_frame_s and sim_time <= end_s + 1e-9:
        robot.exportImage(str(capture_dir / f"frame_{frame_index:06d}.png"), 100)
        frame_index += 1
        next_frame_s += acceleration / fps
    if sim_time >= end_s:
        receipt = {
            "version": 1,
            "source": "OmniSim wgpu main view via Supervisor.exportImage",
            "profile": profile,
            "fps": fps,
            "acceleration": acceleration,
            "capture_sim_time_s": [start_s, end_s],
            "frames": frame_index,
            "cuts": cuts,
            "camera_rule": "locked compositions; direct cuts only; wide-detail-birdseye-detail",
        }
        (capture_dir.parent / f"{capture_dir.name}_capture_receipt.json").write_text(
            json.dumps(receipt, indent=2) + "\n", encoding="utf-8"
        )
        print(f"[intercept_camera] COMPLETE frames={frame_index}", flush=True)
        if autoquit:
            robot.simulationQuit(0)
        break

while robot.step(dt_ms) != -1:
    pass
