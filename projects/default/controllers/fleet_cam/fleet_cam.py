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

"""fleet_cam — supervisor + Camera device that records a fixed-ish wide shot
of a world to a PNG frame sequence, headlessly.

Uses the only reliable automated-render path on this machine: a Camera
device's own offscreen buffer via camera.saveImage (NOT Supervisor.exportImage,
which paints grey when the main 3D view is minimized). Run the binary WITHOUT
--batch (camera.enable segfaults under batch render paths) — --minimize is fine
because the camera renders its own buffer independent of the main window.

The camera robot aims itself with a look-at each frame, so a slow lateral
drift across the run reads as a gentle pan over the whole scene.

Env knobs:
  FLEET_CAM_DIR     output frame dir (default <repo>/youtube_videos/fleet_frames)
  FLEET_CAM_FRAMES  number of frames to dump (default 900)
  FLEET_CAM_STEPS   sim steps advanced per frame (default 2; 2*16ms ~ 30fps)
  FLEET_CAM_WARMUP  warmup steps before frame 0 (default 60)
"""

from __future__ import annotations

import math
import os
import sys

from omnisim import Supervisor

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
OUT_DIR = os.environ.get("FLEET_CAM_DIR", os.path.join(REPO_ROOT, "youtube_videos", "fleet_frames"))
OUT_DIR = os.path.abspath(OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)

N_FRAMES = int(os.environ.get("FLEET_CAM_FRAMES", "900"))
STEPS_PER_FRAME = int(os.environ.get("FLEET_CAM_STEPS", "2"))
WARMUP_STEPS = int(os.environ.get("FLEET_CAM_WARMUP", "60"))

# Wide 3/4 shot of the whole 20x20 arena, looking along +y and down. A slow
# lateral drift on x over the run gives a gentle pan.
EYE_START = (-3.0, -20.0, 22.0)
EYE_END = (3.0, -20.0, 22.0)
TARGET = (0.0, 0.0, 0.3)


def log(msg: str) -> None:
    print(f"[fleet_cam] {msg}", flush=True)


def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def _norm(a):
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Axis-angle so a camera at `eye` (forward=+X, up=+Z) faces `target`.
    Same construction as site_screenshot.look_at: R columns [F, -Rt, Uo]."""
    f = _norm(_sub(target, eye))
    if abs(f[0] * up[0] + f[1] * up[1] + f[2] * up[2]) > 0.999:
        up = (0.0, 1.0, 0.0)
    d = f[0] * up[0] + f[1] * up[1] + f[2] * up[2]
    uo = _norm((up[0] - d * f[0], up[1] - d * f[1], up[2] - d * f[2]))
    rt = _cross(f, uo)
    m = [
        [f[0], -rt[0], uo[0]],
        [f[1], -rt[1], uo[1]],
        [f[2], -rt[2], uo[2]],
    ]
    trace = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    x = m[2][1] - m[1][2]
    y = m[0][2] - m[2][0]
    z = m[1][0] - m[0][1]
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-6:
        return [0.0, 0.0, 1.0, angle]
    return [x / n, y / n, z / n, angle]


def _lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def main() -> int:
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())

    cam = robot.getDevice("cam")
    if cam is None:
        log("ERROR: no Camera device named 'cam'")
        return 1
    cam.enable(ts)

    self_node = robot.getSelf()
    tf = self_node.getField("translation")
    rf = self_node.getField("rotation")

    log(f"out={OUT_DIR} frames={N_FRAMES} steps/frame={STEPS_PER_FRAME} "
        f"cam={cam.getWidth()}x{cam.getHeight()}")

    # Park the camera at the opening pose and let lighting/physics settle so
    # frame 0 isn't mid-spawn-drop.
    tf.setSFVec3f(list(EYE_START))
    rf.setSFRotation(look_at(EYE_START, TARGET))
    for _ in range(WARMUP_STEPS):
        if robot.step(ts) == -1:
            return 0

    saved = 0
    for i in range(N_FRAMES):
        t = i / max(1, N_FRAMES - 1)
        eye = _lerp(EYE_START, EYE_END, t)
        tf.setSFVec3f(list(eye))
        rf.setSFRotation(look_at(eye, TARGET))
        for _ in range(STEPS_PER_FRAME):
            if robot.step(ts) == -1:
                log(f"sim ended early at frame {i}")
                robot.simulationQuit(0)
                return 0
        path = os.path.join(OUT_DIR, f"frame_{i:06d}.png")
        if cam.saveImage(path, 95) == 0:
            saved += 1
        else:
            log(f"saveImage failed at frame {i}")
        if i % 60 == 0:
            log(f"frame {i}/{N_FRAMES}")

    log(f"DONE saved={saved}/{N_FRAMES}")
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        import traceback
        log("FATAL:\n" + traceback.format_exc())
        sys.exit(1)
