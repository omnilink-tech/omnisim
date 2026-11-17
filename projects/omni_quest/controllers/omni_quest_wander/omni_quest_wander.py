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

"""omni_quest_wander — background roaming robots that make the scene feel alive.

A generic, robot-agnostic wander controller. Auto-detects its drive motors
(4-wheel skid-steer Husky/Jackal, or 2-wheel TurtleBot), then roams within a
bounded circle: drive forward, change heading at random intervals, steer back
toward the centre at the boundary, and recover (reverse + turn) if it wedges.

Controller args:  --cx X  --cy Y  --radius R  --speed S
"""

from __future__ import annotations

import math
import os
import random
import sys
from pathlib import Path

from omnisim import Supervisor

PROJ = Path(__file__).resolve().parents[2]
SKID = ("front_left_wheel_motor", "rear_left_wheel_motor",
        "front_right_wheel_motor", "rear_right_wheel_motor")
TWO = ("wheel_left_joint", "wheel_right_joint")


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse(argv):
    cx = cy = 0.0
    radius, speed = 18.0, 2.0
    i = 0
    while i < len(argv):
        if argv[i] == "--cx" and i + 1 < len(argv):
            cx = float(argv[i + 1]); i += 2
        elif argv[i] == "--cy" and i + 1 < len(argv):
            cy = float(argv[i + 1]); i += 2
        elif argv[i] == "--radius" and i + 1 < len(argv):
            radius = float(argv[i + 1]); i += 2
        elif argv[i] == "--speed" and i + 1 < len(argv):
            speed = float(argv[i + 1]); i += 2
        else:
            i += 1
    return cx, cy, radius, speed


def main() -> int:
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    dt = ts / 1000.0
    self_node = sup.getSelf()
    cx, cy, radius, speed = parse(sys.argv[1:])

    skid = [sup.getDevice(n) for n in SKID]
    if all(m is not None for m in skid):
        left, right = [skid[0], skid[1]], [skid[2], skid[3]]
    else:
        l, r = sup.getDevice(TWO[0]), sup.getDevice(TWO[1])
        if l is None or r is None:
            print(f"[wander] FATAL: no drive motors on {sup.getName()!r}",
                  file=sys.stderr)
            return 1
        left, right = [l], [r]
    for m in left + right:
        m.setPosition(float("inf")); m.setVelocity(0.0)

    def drive(ls, rs):
        for m in left:
            m.setVelocity(ls)
        for m in right:
            m.setVelocity(rs)

    rng = random.Random(sup.getName())
    turn = 0.0
    turn_timer = 0
    stuck = 0
    last = (0.0, 0.0)
    next_check = 0.0
    print(f"[wander] {sup.getName()} roaming r={radius} m around "
          f"({cx},{cy}) at {speed} rad/s", flush=True)
    roam_log = None
    if os.environ.get("OMNI_QUEST_VERIFY"):
        try:
            roam_log = open(PROJ / f"_roam_{sup.getName()}.csv", "w",
                            encoding="utf-8", buffering=1)
            roam_log.write("t,x,y,z,yaw\n")
        except Exception:
            roam_log = None
    next_pose_log = 0.0

    while sup.step(ts) != -1:
        pos = self_node.getPosition()
        ori = self_node.getOrientation()
        yaw = math.atan2(ori[3], ori[0])
        t = sup.getTime()
        if roam_log is not None and t >= next_pose_log:
            roam_log.write(f"{t:.2f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},"
                           f"{math.degrees(yaw):.1f}\n")
            next_pose_log = t + 0.5

        if t >= next_check:
            moved = math.hypot(pos[0] - last[0], pos[1] - last[1])
            stuck = stuck + 1 if moved < 0.12 else 0
            last = (pos[0], pos[1])
            next_check = t + 1.5

        if stuck >= 1:                                   # wedged -> reverse STRAIGHT out
            # Both wheels reverse (translates backward to break a jam), with a
            # small alternating curl so successive backs don't retrace.
            d = 0.8 if (stuck % 2) else -0.8
            drive(-speed - d, -speed + d)
            if stuck >= 3:
                stuck = 0
            continue

        dist_c = math.hypot(pos[0] - cx, pos[1] - cy)
        if dist_c > radius:                              # CONTAIN: face centre, drive in
            err = wrap_pi(math.atan2(cy - pos[1], cx - pos[0]) - yaw)
            if abs(err) > 0.5:                           # pivot in place toward centre
                s = 1.0 if err > 0 else -1.0
                drive(-1.6 * s, 1.6 * s)
            else:
                drive(speed, speed)                      # aimed in -> go
        else:                                            # WANDER inside the pasture
            turn_timer -= 1
            if turn_timer <= 0:
                turn = rng.uniform(-1.0, 1.0)
                turn_timer = rng.randint(int(1.5 / dt), int(4.0 / dt))
            drive(speed - turn, speed + turn)
    return 0


if __name__ == "__main__":
    sys.exit(main())
