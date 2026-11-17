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

"""g1_armhold -- pose the (welded) G1 arm at a sweep of ELBOW angles so a sibling
camera-bot (g1_armshot) can photograph each and we can SEE which elbow value gives
a completely straight arm. Legs held at the demo squat; base is staticBase so it
just floats fixed while we cycle the elbow. Shared schedule with g1_armshot.
"""
from __future__ import annotations
import sys
from omnisim import Robot

# Shared schedule (MUST match g1_armshot): elbow value per 3s window.
ELBOWS = [0.45, 0.55, 0.65]
WIN_S = 4.0

ARM = {
    "left_shoulder_pitch_joint": 1.5708, "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 1.5708, "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0, "right_wrist_roll_joint": 0.0,
}
LEGS = {
    "left_hip_pitch_joint": -0.30, "left_knee_joint": 0.52, "left_ankle_pitch_joint": -0.23,
    "right_hip_pitch_joint": -0.30, "right_knee_joint": 0.52, "right_ankle_pitch_joint": -0.23,
}
ELBOW_JOINTS = ["left_elbow_joint", "right_elbow_joint"]


def main() -> int:
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    motors = {}
    for jn in list(ARM) + list(LEGS) + ELBOW_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            try:
                m.setAvailableTorque(200.0)
            except Exception:
                pass
        motors[jn] = m
    # static pose for arms (non-elbow) + legs
    for jn, v in {**ARM, **LEGS}.items():
        if motors.get(jn) is not None:
            motors[jn].setPosition(float(v))

    last_idx = -1
    k = 0
    while robot.step(ts) != -1:
        t = k * ts / 1000.0
        idx = min(int(t / WIN_S), len(ELBOWS) - 1)
        if idx != last_idx:
            ev = ELBOWS[idx]
            for jn in ELBOW_JOINTS:
                if motors.get(jn) is not None:
                    motors[jn].setPosition(float(ev))
            print(f"[g1_armhold] t={t:.1f}s window {idx} elbow={ev:+.4f}", flush=True)
            last_idx = idx
        if t > len(ELBOWS) * WIN_S + 1.0:
            break
        k += 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
