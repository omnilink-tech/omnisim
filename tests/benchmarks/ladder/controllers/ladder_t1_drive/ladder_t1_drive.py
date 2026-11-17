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

"""T1's scripted control on the OmniSim column: drive north, stop, dwell.

**Not a ladder cell and not a result.** This is a control a human wrote
knowing the thresholds, exactly like the MuJoCo column's ``run_t1``. Its only
claim is *"the task is achievable here"*, so that a later agent cell failing it
is a statement about the agent rather than about the asset.

How it knows where it is
------------------------

Through ``supervisor TRUE`` on its own node, reading its own pose. A scripted
oracle is allowed to: it is not competing, and the alternative -- open-loop
wheel odometry -- would put the arrival threshold at the mercy of skid-steer
slip, which is a property of the tyres and not of the question T1 asks.

The commanded point is resolved the way the task file says: the graded body's
position at the FIRST sample after the settle window, plus the declared offset.
The offset arrives as an argument from the oracle, which reads it from
``meta.json`` -- never from anything observed at runtime.
"""

from __future__ import annotations

import math
import sys

from controller import Supervisor

WHEELS = ("front_left_wheel_motor", "front_right_wheel_motor",
          "rear_left_wheel_motor", "rear_right_wheel_motor")

CRUISE_RAD_S = 3.0        # ~0.50 m/s on a 0.1651 m wheel
APPROACH_M = 1.2          # start easing here
ARRIVE_M = 0.10           # commanded stop radius, well inside T1's 0.25 m
MIN_RAD_S = 0.35          # below this the wheels stall rather than creep
DWELL_S = 4.0             # T1 needs 2.0 s; double it

# Every one of the five is overridable from controllerArgs. T1 does not need
# that -- its thresholds are loose enough that the defaults clear them -- but
# LOOPBENCH's L2 asks an agent to hit a tight arrival AND a deadline, and a
# tunable control is what makes the target's reachability checkable by a
# scripted probe before any agent is asked to find it.


def _arg(flag, default):
    if flag in sys.argv:
        return float(sys.argv[sys.argv.index(flag) + 1])
    return default


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    settle_s = _arg("--settle-s", 0.5)
    off_x = _arg("--offset-x", 0.0)
    off_y = _arg("--offset-y", 5.0)
    duration_s = _arg("--duration-s", 60.0)
    cruise = _arg("--cruise-rad-s", CRUISE_RAD_S)
    approach = _arg("--approach-m", APPROACH_M)
    arrive = _arg("--arrive-m", ARRIVE_M)
    min_rad = _arg("--min-rad-s", MIN_RAD_S)
    brake = _arg("--brake-rad-s", 0.0)   # reverse torque pulse on arrival

    motors = []
    for name in WHEELS:
        m = robot.getDevice(name)
        if m is None:
            print("[t1_drive] MISSING MOTOR %r -- cannot drive" % name)
            return 1
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors.append(m)

    me = robot.getSelf()
    if me is None:
        print("[t1_drive] no self node; the robot needs supervisor TRUE")
        return 1

    # settle, then take the start the task file's rule names
    t_end_settle = settle_s
    while robot.step(dt) != -1 and robot.getTime() < t_end_settle:
        pass
    p = me.getPosition()
    start = (float(p[0]), float(p[1]))
    goal = (start[0] + off_x, start[1] + off_y)
    print("[t1_drive] start=(%.4f, %.4f) goal=(%.4f, %.4f)"
          % (start[0], start[1], goal[0], goal[1]))

    arrived_at = None
    while robot.step(dt) != -1:
        now = robot.getTime()
        if now > duration_s:
            break
        p = me.getPosition()
        dx, dy = goal[0] - float(p[0]), goal[1] - float(p[1])
        dist = math.hypot(dx, dy)

        if arrived_at is None and dist <= arrive:
            arrived_at = now
            print("[t1_drive] arrived at t=%.2fs, dist=%.4f m" % (now, dist))

        if arrived_at is not None:
            # An optional reverse pulse: at speed a wheeled base coasts past
            # the point on friction alone, and zero torque is not a brake.
            if brake > 0.0 and now - arrived_at < 0.25:
                v = -brake
            else:
                v = 0.0                 # stop and stay stopped: the dwell
        elif dist > approach:
            v = cruise
        else:
            # ease in so the stop is a stop, not an overshoot-and-return
            v = max(min_rad, cruise * (dist / approach))

        for m in motors:
            m.setVelocity(v)

        if arrived_at is not None and now - arrived_at > DWELL_S + 2.0:
            # dwelt long enough; keep stepping so the recorder keeps sampling
            pass

    p = me.getPosition()
    print("[t1_drive] final=(%.4f, %.4f) err=%.4f m"
          % (float(p[0]), float(p[1]),
             math.hypot(goal[0] - float(p[0]), goal[1] - float(p[1]))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
