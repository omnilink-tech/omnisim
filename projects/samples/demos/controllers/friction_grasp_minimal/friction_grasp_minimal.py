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

"""Close two fingers on a part, lift it, hold it, put it down. No weld.

The companion to ``worlds/starter/friction_grasp_minimal.omniworld``. Read that file
first -- the physics settings are there and each one is commented.

Two things in here are the whole point, and both are counter-intuitive enough
that people rediscover them the expensive way:

**Squeeze with FORCE, not with a position target.** A position servo drives the
error to zero and then, having arrived, applies almost nothing. A grip needs a
sustained inward push, so the fingers are commanded with ``setForce`` and left
there. ``setPosition`` is for getting the fingers near the part; ``setForce``
is for holding it.

**Releasing means reversing the force, not asking for a position.** On the
Newton path a motor put into force control keeps that force until something
withdraws it, so ``setForce(+N)`` is undone by ``setForce(-N)`` and not by
``setPosition``.

The run prints one line per phase with the part's measured height, so a failed
grasp is visible as a number rather than as a picture: if the part is on the
floor while the fingers are up, it slipped.
"""

from __future__ import annotations

import json
import os
import sys

from omnisim import Supervisor

#: Where the run writes its verdict.
#:
#: ⚠ A controller's ``print`` is NOT reliably readable. On Windows the
#: simulator is a GUI-subsystem binary with no console, so controller stdout
#: goes nowhere unless the launcher captures it. Any controller whose output
#: is evidence should write a FILE. This one does both.
RESULT_PATH = os.environ.get("GRASP_RESULT", "friction_grasp_result.json")

#: Inward squeeze per finger, newtons. NEGATIVE because each finger's slider
#: axis points AWAY from the part, so a negative force is the one that closes
#: it -- the sign follows the joint axis, not the word "grip". Enough to hold a
#: 0.15 kg part at mu=3 with margin; far more crushes a light part out sideways.
GRIP_N = -25.0
#: The reverse pulse that opens the fingers again (see the module docstring:
#: a force-controlled motor is released by reversing the force).
RELEASE_N = 12.0
#: Fingers at rest, which is already ~9 mm clear of the part on each side.
OPEN_M = 0.0
#: Wrist height for the approach, metres on the lift slider (negative is down).
#: Puts the finger pads level with the middle of the part.
LIFT_M = -0.205

APPROACH_S = 1.5
CLOSE_S = 1.5
HOLD_S = 4.0
LOWER_S = 1.5


def main() -> int:
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())

    lift = robot.getDevice("lift")
    fingers = [robot.getDevice("finger_left"), robot.getDevice("finger_right")]
    if lift is None or any(f is None for f in fingers):
        print("[grasp] missing a motor; check the world's device names")
        return 1

    part = robot.getFromDef("PART")
    if part is None:
        print("[grasp] no DEF PART in the world")
        return 1

    def z():
        return part.getPosition()[2]

    start_z = z()          # measured, not assumed: the rest height follows
    #                        from the collider, and a substituted collider
    #                        rests somewhere the file does not predict.

    def run_for(seconds, label):
        end = robot.getTime() + seconds
        while robot.step(dt) != -1 and robot.getTime() < end:
            pass
        print("[grasp] %-9s t=%5.2fs  part_z=%.4f m" % (label, robot.getTime(), z()))

    # 1. open the fingers and drop the wrist around the part
    for f in fingers:
        f.setPosition(OPEN_M)
    lift.setPosition(LIFT_M)
    run_for(APPROACH_S, "approach")

    # 2. squeeze. FORCE, not position -- a servo that has arrived pushes with
    #    almost nothing, and the part slides straight back out.
    for f in fingers:
        f.setForce(GRIP_N)
    run_for(CLOSE_S, "close")

    # 3. lift, still squeezing. Friction alone is carrying the part now.
    lift.setPosition(0.0)
    run_for(HOLD_S, "hold")
    held_z = z()

    # 4. put it down and let go. Reversing the force is what opens a
    #    force-controlled motor; asking for a position does not.
    lift.setPosition(LIFT_M)
    run_for(LOWER_S, "lower")
    for f in fingers:
        f.setForce(RELEASE_N)
    run_for(1.5, "release")

    # The verdict, as a number. The part starts at z=0.06; held clear of the
    # floor it sits far above that, and a slipped part is back down near it.
    lifted = held_z - start_z
    verdict = "HELD" if lifted > 0.05 else "SLIPPED"
    print("[grasp] RESULT held_z=%.4f m  lifted_by=%.4f m  -> %s"
          % (held_z, lifted, verdict))
    try:
        with open(RESULT_PATH, "w", encoding="utf-8") as fh:
            json.dump({"verdict": verdict, "held_z_m": round(held_z, 5),
                       "lifted_by_m": round(lifted, 5),
                       "start_z_m": round(start_z, 5), "grip_n": GRIP_N,
                       "note": "held by friction only; nothing welded or "
                               "teleported"}, fh, indent=1)
    except OSError as exc:
        print("[grasp] could not write %s: %r" % (RESULT_PATH, exc))
    return 0


if __name__ == "__main__":
    sys.exit(main())
