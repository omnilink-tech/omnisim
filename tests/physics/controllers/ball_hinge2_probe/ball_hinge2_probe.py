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

"""BallJoint / Hinge2Joint probe for tests/test_newton_ball_hinge2.py.

Runs ON the rig Robot itself (it owns the joint, hence the motors and the
position sensors -- a sibling supervisor could not reach them; OmniSim restricts
device APIs to the owning controller). `--mode ball` or `--mode hinge2`.

MODE "ball" -- a 1 kg bob hangs off a 3-DoF BallJoint, 0.4 m below and 0.15 m
to the +x side of the anchor, and swings. Records, per tick:
  * the three position sensors (so the test can assert WHICH axes move: the
    swing is planar in x-z, i.e. about the joint's SECOND axis, and axes 1 and 3
    must stay quiet),
  * the bob's distance to the anchor in the rig's own frame (so the test can
    assert the joint CONSTRAINS: a bob that lost its constraint free-falls and
    that radius grows without bound),
  * the bob's world z (it must actually move).

MODE "hinge2" -- both axes are position-controlled servos on an endpoint whose
COM sits ON the anchor (so gravity torque is ~0 and this measures the actuator +
readback, not the load). Timeline:
    settle -> command (axis1 = +0.4, axis2 = -0.3) -> hold -> sample A
           -> command (axis1 = +0.4 unchanged, axis2 = +0.3) -> hold -> sample B
A proves both axes take a target; B proves axis 2 moves AGAIN and that axis 1
does NOT follow it -- the composed-hinge failure mode is axis2 silently frozen
or bleeding into axis1.

The samples ARE the verdict; every assertion lives in the test.
Output: OMNISIM_BH2_PROBE_OUT (JSON).
"""
import json
import math
import os
import sys

from omnisim import Supervisor

MODE = "ball"
if "--mode" in sys.argv:
    MODE = sys.argv[sys.argv.index("--mode") + 1]

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

#: joint anchor in the rig's own frame -- MUST match the world text the test writes
ANCHOR_LOCAL = (0.0, 0.0, 1.0)


def sensor(name):
    d = robot.getDevice(name)
    if d is not None:
        d.enable(dt)
    return d


def read(d):
    if d is None:
        return 0.0
    v = d.getValue()
    # A sensor read before its first refresh returns NaN; report it as 0 rather
    # than poisoning every max() downstream.
    return 0.0 if v is None or math.isnan(v) else float(v)


def advance(ms):
    n = max(1, int(round(ms / dt)))
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
    return True


ps1, ps2, ps3 = sensor("ps1"), sensor("ps2"), sensor("ps3")
out = {"mode": MODE, "dt": dt}

if MODE == "ball":
    rig = robot.getSelf()
    bob = robot.getFromDef("BOB")
    peak = [0.0, 0.0, 0.0]     # max |angle| seen per axis
    signed = [0.0, 0.0, 0.0]   # the signed value AT the peak, for the sign check
    radii = []
    z_min, z_max = 1e9, -1e9
    advance(4 * dt)            # let Newton finalize before the first sample
    out["r0"] = None
    for _ in range(int(round(3000.0 / dt))):
        if robot.step(dt) == -1:
            break
        p = bob.getPosition()
        rp = rig.getPosition()
        ro = rig.getOrientation()          # row-major 3x3
        # anchor in world = rig_pos + R_rig * ANCHOR_LOCAL
        ax = rp[0] + ro[0] * ANCHOR_LOCAL[0] + ro[1] * ANCHOR_LOCAL[1] + ro[2] * ANCHOR_LOCAL[2]
        ay = rp[1] + ro[3] * ANCHOR_LOCAL[0] + ro[4] * ANCHOR_LOCAL[1] + ro[5] * ANCHOR_LOCAL[2]
        az = rp[2] + ro[6] * ANCHOR_LOCAL[0] + ro[7] * ANCHOR_LOCAL[1] + ro[8] * ANCHOR_LOCAL[2]
        r = math.sqrt((p[0] - ax) ** 2 + (p[1] - ay) ** 2 + (p[2] - az) ** 2)
        if out["r0"] is None:
            out["r0"] = r
        radii.append(r)
        z_min = min(z_min, p[2])
        z_max = max(z_max, p[2])
        for i, d in enumerate((ps1, ps2, ps3)):
            v = read(d)
            if abs(v) > peak[i]:
                peak[i] = abs(v)
                signed[i] = v
    out["peak_abs"] = peak
    out["signed_at_peak"] = signed
    out["r_min"] = min(radii) if radii else None
    out["r_max"] = max(radii) if radii else None
    out["bob_z_min"] = z_min
    out["bob_z_max"] = z_max
    out["bob_end"] = [float(v) for v in bob.getPosition()]

elif MODE == "hinge2":
    m1 = robot.getDevice("m1")
    m2 = robot.getDevice("m2")
    out["target_a"] = [0.4, -0.3]
    out["target_b"] = [0.4, 0.3]
    advance(20 * dt)                        # settle + finalize
    out["rest"] = [read(ps1), read(ps2)]
    m1.setPosition(0.4)
    m2.setPosition(-0.3)
    advance(2500)
    out["a"] = [read(ps1), read(ps2)]
    m2.setPosition(0.3)                     # axis 1 target deliberately untouched
    advance(2500)
    out["b"] = [read(ps1), read(ps2)]
    # A third phase with BOTH axes re-commanded, so a reader can tell "axis 1 is
    # stuck at its first target" from "axis 1 tracks".
    m1.setPosition(-0.2)
    advance(2500)
    out["c"] = [read(ps1), read(ps2)]
    out["target_c"] = [-0.2, 0.3]

else:
    out["error"] = "unknown mode %r" % MODE

path = os.environ.get("OMNISIM_BH2_PROBE_OUT", "bh2_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("ball_hinge2_probe(%s): wrote %s\n" % (MODE, path))
sys.stdout.flush()
