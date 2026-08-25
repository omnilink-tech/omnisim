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

"""Minimal 2-wheel forward driver, no random walk, no recovery.
Sets both wheels to +3 rad/s and traces position every 1 s. Used to
isolate whether the TB3 URDF import produces a drivable robot."""

from __future__ import annotations

import math
import os
import sys

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


def main() -> int:
    robot = _Robot()
    step = int(robot.getBasicTimeStep())

    left = robot.getDevice("wheel_left_joint_motor")
    right = robot.getDevice("wheel_right_joint_motor")
    if left is None or right is None:
        sys.stderr.write("missing wheel motors\n")
        return 1
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    # Pure forward command, both sides equal.
    speed = 3.0
    left.setVelocity(speed)
    right.setVelocity(speed)

    self_node = robot.getSelf()
    # Also look up wheel + chassis link nodes by name so we can compare.
    wheel_l_node = robot.getFromDef("wheel_left_link") if hasattr(robot, "getFromDef") else None
    if wheel_l_node is None:
        # Try by-name search via supervisor scene
        try:
            root = robot.getRoot() if hasattr(robot, "getRoot") else None
            sys.stderr.write(f"[straight] no DEF lookup, root={root}\n")
        except Exception:
            pass

    # PositionSensor on wheel motor to read actual rotation.
    sensor = left.getPositionSensor()
    if sensor is not None:
        sensor.enable(step)

    trace_dir = r"C:\tmp\husky_trace"
    os.makedirs(trace_dir, exist_ok=True)
    f = open(os.path.join(trace_dir, f"{robot.getName()}_straight.log"), "w", buffering=1)

    sim_ms = 0
    next_ms = 0
    initial = None
    while robot.step(step) != -1:
        sim_ms += step
        if sim_ms >= next_ms:
            pos = self_node.getPosition()
            wheel_angle = sensor.getValue() if sensor is not None else 0.0
            if initial is None:
                initial = pos
            dx, dy, dz = pos[0]-initial[0], pos[1]-initial[1], pos[2]-initial[2]
            f.write(f"t_ms={sim_ms} pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                    f"delta=({dx:+.3f},{dy:+.3f},{dz:+.3f}) "
                    f"wheel_angle={wheel_angle:+.2f} "
                    f"cmd_speed={speed}\n")
            next_ms = sim_ms + 1000
    return 0


if __name__ == "__main__":
    sys.exit(main())
