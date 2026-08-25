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

"""husky_terrain_nav — drive forward across rough terrain, holding course.

The basic ``drive_forward`` controller issues identical velocity to all
four wheels and has no feedback. On a single dead-center rock the Husky
climbs over fine, but the first asymmetric rock-climb produces a yaw
moment, and with no steering correction the drift compounds until the
robot wedges itself sideways against the next obstacle.

This controller closes that loop:

* Reads pose via the Supervisor API (the Husky is ``supervisor TRUE``).
* Compares heading and lateral position to the commanded straight-line
  course (target heading 0, target y = 0).
* Applies differential wheel velocity to correct.
* Holds a base velocity of ~0.4 m/s — slow enough to climb rocks
  without launching them.

Controller args (positional):
    [0] base_velocity_rad_s  (default 2.5)
"""

from __future__ import annotations

import math
import sys

from omnisim import Supervisor


BASE_VELOCITY_RAD_S = 2.5
HEADING_GAIN = 1.2       # gentler than the v1 (1.8) — was fighting wheel climb
LATERAL_GAIN = 0.4       # gentler than the v1 (0.8) — lateral drift during rock
                          # climb is unavoidable; over-aggressive correction makes
                          # one side of wheels slip
MAX_SCALE = 1.6          # cap on per-wheel scale factor
MIN_SCALE = 0.0          # never reverse (don't back up over a rock)

WHEEL_NAMES = {
    "fl": "front_left_wheel_motor",
    "fr": "front_right_wheel_motor",
    "rl": "rear_left_wheel_motor",
    "rr": "rear_right_wheel_motor",
}


def main() -> int:
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    self_node = sup.getSelf()
    if self_node is None:
        print("[husky_terrain_nav] FATAL: getSelf() returned None", file=sys.stderr)
        return 1

    args = sys.argv[1:]
    base_v = BASE_VELOCITY_RAD_S
    if args:
        try:
            base_v = float(args[0])
        except ValueError:
            pass

    motors: dict[str, object] = {}
    for key, dev_name in WHEEL_NAMES.items():
        m = sup.getDevice(dev_name)
        if m is None:
            print(f"[husky_terrain_nav] FATAL: device {dev_name!r} not found",
                  file=sys.stderr)
            return 1
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors[key] = m

    print(f"[husky_terrain_nav] base_v={base_v} rad/s "
          f"heading_gain={HEADING_GAIN} lateral_gain={LATERAL_GAIN}", flush=True)

    while sup.step(ts) != -1:
        # 3x3 rotation matrix, row-major flat list:
        # [R00 R01 R02 R10 R11 R12 R20 R21 R22]
        # World-frame direction of the husky's local +X is column 0.
        rot = self_node.getOrientation()
        heading = math.atan2(rot[3], rot[0])   # yaw, target = 0

        pos = self_node.getPosition()
        y = pos[1]                              # lateral, target = 0

        # Combined error: positive value => steer left (turn toward +Y).
        # A leftward (turn-left) steer for a skid-steer = right wheels
        # faster than left wheels.
        steer = -HEADING_GAIN * heading - LATERAL_GAIN * y

        # Translate steer into per-side scale factors centered at 1.0.
        left_scale = max(MIN_SCALE, min(MAX_SCALE, 1.0 - steer))
        right_scale = max(MIN_SCALE, min(MAX_SCALE, 1.0 + steer))

        motors["fl"].setVelocity(base_v * left_scale)
        motors["fr"].setVelocity(base_v * right_scale)
        motors["rl"].setVelocity(base_v * left_scale)
        motors["rr"].setVelocity(base_v * right_scale)

    return 0


if __name__ == "__main__":
    sys.exit(main())
