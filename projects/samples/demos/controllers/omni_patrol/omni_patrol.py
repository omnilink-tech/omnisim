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
"""omni_patrol — kinematic garden patrol for the launcher's hello-world lot.

Drives DEF PATROL_ROVER on a fixed rounded-rectangle loop around the house so a
viewer can watch the lighting stack (OmniLight probe field, PCSS sun shadows,
SSR, the emissive light bar at night) play over a MOVING body. The world is the
Beauty Bench lot — a pure look scene with no colliders — so the rover is
deliberately kinematic: no Physics nodes anywhere in its subtree, and this
supervisor writes its pose every tick (the mocap pattern). Wheel spin is visual,
driven through the nested WHEEL_SPIN_* poses (spokes make the spin readable).

The loop ring was sized against the world's props: x in [-5.6, 5.6],
y in [-7.15, 4.2], corner radius 1.2 — clear of the hedges, plinth, wheelbarrow,
lantern, fence line and both trees. Crossing the slate path (top z=0.055 vs lawn
0.02) the rover eases up 35 mm so the wheels ride the pavement, not through it.
"""

import math
import os

from omnisim import Supervisor

SPEED = float(os.environ.get("OMNI_PATROL_SPEED", "0.9") or 0.9)  # m/s along the loop
WHEEL_RADIUS = 0.12
BASE_Z = 0.02        # lawn top
PATH_X = -1.4        # slate path centreline (the front-gate crossing)
PATH_LIFT = 0.035    # slate top (0.055) - lawn top (0.02)

# Rounded rectangle: centreline the wheels track.
XMIN, XMAX = -5.6, 5.6
YMIN, YMAX = -7.15, 4.2
R = 1.2


def build_loop():
    """Piecewise segments of the rounded rect, counter-clockwise from the
    east leg. Each entry: (kind, data, length)."""
    segs = []
    # east leg, heading +y
    segs.append(("line", (XMAX, YMIN + R, 0.0, 1.0), (YMAX - R) - (YMIN + R)))
    # NE corner (centre (XMAX-R, YMAX-R)), from angle 0 to pi/2
    segs.append(("arc", (XMAX - R, YMAX - R, 0.0), math.pi / 2 * R))
    # north leg, heading -x
    segs.append(("line", (XMAX - R, YMAX, -1.0, 0.0), (XMAX - R) - (XMIN + R)))
    # NW corner, angle pi/2 to pi
    segs.append(("arc", (XMIN + R, YMAX - R, math.pi / 2), math.pi / 2 * R))
    # west leg, heading -y
    segs.append(("line", (XMIN, YMAX - R, 0.0, -1.0), (YMAX - R) - (YMIN + R)))
    # SW corner, angle pi to 3pi/2
    segs.append(("arc", (XMIN + R, YMIN + R, math.pi), math.pi / 2 * R))
    # south leg, heading +x (the front-gate path crossing)
    segs.append(("line", (XMIN + R, YMIN, 1.0, 0.0), (XMAX - R) - (XMIN + R)))
    # SE corner, angle 3pi/2 to 2pi
    segs.append(("arc", (XMAX - R, YMIN + R, 3 * math.pi / 2), math.pi / 2 * R))
    return segs, sum(s[2] for s in segs)


SEGS, PERIMETER = build_loop()


def sample(dist):
    """(x, y, heading) at arc-length dist along the loop."""
    d = dist % PERIMETER
    for kind, data, length in SEGS:
        if d > length:
            d -= length
            continue
        if kind == "line":
            x0, y0, dx, dy = data
            return x0 + dx * d, y0 + dy * d, math.atan2(dy, dx)
        cx, cy, a0 = data
        a = a0 + d / R
        # CCW around the corner centre; tangent leads the radius by +90 deg
        return cx + R * math.cos(a), cy + R * math.sin(a), a + math.pi / 2
    # numeric edge: wrap
    x0, y0, dx, dy = SEGS[0][1]
    return x0, y0, math.atan2(dy, dx)


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def main():
    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())

    node = sup.getSelf()
    tr_field = node.getField("translation")
    rot_field = node.getField("rotation")

    # Nested wheel-spin poses; dotted DEF path first, bare name as fallback.
    # OMNI_PATROL_SPIN=0 disables the per-tick wheel writes (diagnostic).
    spin_fields = []
    if os.environ.get("OMNI_PATROL_SPIN", "1") != "0":
        for name in ("WHEEL_SPIN_FL", "WHEEL_SPIN_FR", "WHEEL_SPIN_RL", "WHEEL_SPIN_RR"):
            wheel = sup.getFromDef("PATROL_ROVER." + name) or sup.getFromDef(name)
            if wheel is not None:
                spin_fields.append(wheel.getField("rotation"))
        if not spin_fields:
            print("omni_patrol: wheel poses not found -- driving without wheel spin", flush=True)
    print(f"omni_patrol: {len(spin_fields)} wheel spin fields", flush=True)

    # Start phase along the loop in metres (0 = east-leg start). Lets a shot or a
    # test put the rover at a known place without editing the world.
    try:
        dist = float(os.environ.get("OMNI_PATROL_DIST0", "0") or 0.0)
    except ValueError:
        dist = 0.0
    spin = 0.0
    print(f"omni_patrol: driving, dist0={dist:.1f} m, loop {PERIMETER:.1f} m", flush=True)
    tick = 0
    while sup.step(dt) != -1:
        tick += 1
        if tick % 150 == 0:
            x0, y0, _ = sample(dist)
            print(f"omni_patrol: t={tick * dt / 1000.0:.1f}s dist={dist:.1f} pos=({x0:.2f},{y0:.2f})", flush=True)
        step_s = dt / 1000.0
        dist += SPEED * step_s
        spin += (SPEED / WHEEL_RADIUS) * step_s
        x, y, heading = sample(dist)

        # Ease up onto the slate path at the front-gate crossing.
        z = BASE_Z
        if y < -2.0:
            t = 1.0 - (abs(x - PATH_X) - 0.7) / 0.6  # 1 inside |dx|<0.7, 0 past 1.3
            z += PATH_LIFT * smoothstep(t)

        tr_field.setSFVec3f([x, y, z])
        rot_field.setSFRotation([0.0, 0.0, 1.0, heading])
        for f in spin_fields:
            f.setSFRotation([0.0, 0.0, 1.0, spin % (2.0 * math.pi)])


if __name__ == "__main__":
    main()
