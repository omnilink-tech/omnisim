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

"""What is the sonar actually seeing? Sit still and report."""
import os
import sys

from omnisim import Supervisor


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    sonar = robot.getDevice("sonar")
    sonar.enable(dt_ms)
    scan = robot.getDevice("scan_motor")
    scan.setPosition(0.0)
    left = robot.getDevice("left_wheel_motor")
    right = robot.getDevice("right_wheel_motor")
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    self_node = robot.getSelf()
    obs = robot.getFromDef("OBSTACLE")

    for i in range(200):
        robot.step(dt_ms)
    p = self_node.getPosition()
    op = obs.getPosition() if obs else [None] * 3
    print(f"[probe] rover at {[round(v,4) for v in p]}", flush=True)
    print(f"[probe] obstacle at {[round(v,4) for v in op]}", flush=True)
    import math
    r = self_node.getOrientation()
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -r[6]))))
    roll = math.degrees(math.atan2(r[7], r[8]))
    yaw = math.degrees(math.atan2(r[3], r[0]))
    print(f"[probe] roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f} deg",
          flush=True)
    print(f"[probe] sonar reads {sonar.getValue():.5f} m", flush=True)
    if op[0] is not None:
        gap = (op[0] - 0.12) - (p[0] + 0.13)
        print(f"[probe] true gap sensor->obstacle face = {gap:.4f} m", flush=True)

    # now move the obstacle far away and re-read: if the reading does not
    # change, the sensor is seeing the robot itself
    if obs is not None:
        f = obs.getField("translation")
        t = f.getSFVec3f()
        f.setSFVec3f([20.0, t[1], t[2]])
        for i in range(200):
            robot.step(dt_ms)
        print(f"[probe] obstacle moved to x=20; sonar now "
              f"{sonar.getValue():.5f} m", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        print("[probe] FAILED " + traceback.format_exc(), flush=True)
        raise
