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

"""SparkRover Mini, posed for the showcase stills: wheels braked, the arms moved on their
servos to a pose, and the face on the robot's own 800 x 480 Display with the head
camera's live frame as an inset (the drive controller's Face, without the phase word).

    controllerArgs [ "--pose" "wave" ]       # wave | wave_r | concept | rest | hello
    controllerArgs [ "--face" "happy" ]      # happy | neutral

The screen is drawn by this controller through the Display device, and the inset is the
Camera device's real frame; nothing is painted onto the render afterwards.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from omnisim import Robot

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "projects/samples/demos/controllers/spark_rover_drive"))
from spark_rover_drive import Face  # noqa: E402   (the drive test's face and inset)

# shoulder, elbow per side (rad; +shoulder swings the arm forward and up, +elbow bends it up)
POSES = {
    "rest": {"left": (0.15, 0.25), "right": (0.15, 0.25)},
    "wave": {"left": (1.5, 1.5), "right": (0.3, 0.5)},       # the robot's left arm up
    "wave_r": {"left": (0.3, 0.5), "right": (1.5, 1.5)},     # the robot's right arm up
    "concept": {"left": (0.12, 1.2), "right": (0.12, 1.2)},    # forearms forward, as on the sheet
    "greet_r": {"left": (0.12, 0.2), "right": (0.75, 1.0)},  # the right arm lifted, a gesture
    "hello": {"left": (1.05, 0.9), "right": (1.05, 0.9)},
}
WHEELS = ("fl", "fr", "rl", "rr")


class ShowFace(Face):
    """The drive test's face, drawn bigger and in the robot's accent colours, with no phase word."""

    TEAL, PINK = 0x3FE3D2, 0xFF8FD2

    def draw(self, mood, blink, power=None):
        d = self.d
        d.setColor(self.BG)
        d.fillRectangle(0, 0, self.w, self.h)
        for cx in (250, 550):
            d.setColor(self.TEAL)
            if mood == "happy":                       # upturned arcs, as on the concept sheet
                d.fillOval(cx, 205, 92, 80)
                d.setColor(self.BG)
                d.fillOval(cx, 240, 96, 78)
            else:
                d.fillOval(cx, 190, 62, 80)
        d.setColor(self.PINK)
        for cx in (175, 625):                         # cheeks: two short strokes each
            for k in (0, 26):
                d.fillPolygon([cx - 14 + k, cx - 2 + k, cx + 6 + k, cx - 6 + k],
                              [292, 262, 266, 296])
        d.setColor(self.PINK)
        self._smile(400, 285, 80, 70, 70)            # a filled smile
        d.setColor(self.DIM)
        d.setFont("LiberationSans-Regular", 18, True)
        d.drawText("camera", self.w - 172, self.h - 128)
        self.draws += 1


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--pose", default="wave", choices=sorted(POSES))
    p.add_argument("--face", default="happy", choices=("happy", "neutral"))
    p.add_argument("--ramp-s", type=float, default=1.2)
    p.add_argument("--left", help="shoulder,elbow (rad) for the left arm, over the pose's")
    p.add_argument("--right", help="shoulder,elbow (rad) for the right arm, over the pose's")
    a = p.parse_args()
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    for t in WHEELS:
        m = robot.getDevice(f"wheel_{t}_motor")
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    goal = {}
    pose = dict(POSES[a.pose])
    for side in ("left", "right"):
        if getattr(a, side):
            pose[side] = tuple(float(v) for v in getattr(a, side).split(","))
    for side, (qs, qe) in pose.items():
        goal[f"shoulder_{side}"] = qs
        goal[f"elbow_{side}"] = qe
    motors = {j: robot.getDevice(j) for j in goal}
    cam = robot.getDevice("camera")
    cam.enable(100)
    face = ShowFace(robot.getDevice("display"), cam)
    t = 0.0
    while robot.step(dt_ms) != -1:
        t += dt_ms / 1000.0
        f = min(1.0, t / a.ramp_s)
        f = 0.5 - 0.5 * math.cos(math.pi * f)           # ease the arms up from rest
        for j, q in goal.items():
            motors[j].setPosition(f * q)
        face.update(a.face, t)                           # redraws on a change; inset every frame


if __name__ == "__main__":
    main()
