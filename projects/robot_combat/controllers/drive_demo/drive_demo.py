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

"""drive_demo -- straight-line wheel-drive verification for the Newton wheel-flapping work.

Drives all four wheels forward at a fixed angular velocity and each step logs the chassis
pose + per-wheel joint angle to $OMNISIM_HOME/_scratch/drive_<name>.csv. The per-wheel angle
is the signal the old combat_smoke lacked: a ROLLING wheel winds monotonically (angle ~ omega*t),
a FLAPPING wheel oscillates (angle reverses / stays ~0). Chassis tilt catches the bob.

controllerArgs: [omega_rad_s]   (default 12.0)
Run from PowerShell so the embedded interpreter finds warp (else silent ODE).
"""
import math
import os
import sys
from pathlib import Path

from omnisim import Supervisor

robot = Supervisor()
ts = int(robot.getBasicTimeStep())
args = sys.argv[1:]
omega = float(args[0]) if args else 12.0

WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]
motors, sensors = [], []
for w in WHEELS:
    m = robot.getDevice(w + "_motor")
    s = robot.getDevice(w + "_sensor")
    if m is None or s is None:
        sys.stderr.write("[drive_demo] missing device for %s\n" % w)
        continue
    m.setPosition(float("inf"))
    m.setVelocity(omega)
    s.enable(ts)
    motors.append(m)
    sensors.append(s)

self_node = robot.getSelf()
name = robot.getName()
home = os.environ.get("OMNISIM_HOME") or str(Path(__file__).resolve().parents[4])
out = os.path.join(home, "_scratch", "drive_%s.csv" % name)
os.makedirs(os.path.dirname(out), exist_ok=True)
fh = open(out, "w", buffering=1, encoding="utf-8")
fh.write("t,x,y,z,tilt_deg,fl,fr,rl,rr\n")
sys.stderr.write("[drive_demo] %s omega=%.1f -> %s\n" % (name, omega, out))

t = 0.0
while robot.step(ts) != -1:
    t += ts / 1000.0
    p = self_node.getPosition()
    o = self_node.getOrientation()           # 9-element row-major rotation matrix
    # tilt of the body +Z axis from world +Z = acos(R[2][2]); ~0 deg when flat, grows when it bobs.
    cz = max(-1.0, min(1.0, o[8]))
    tilt = math.degrees(math.acos(cz))
    ang = [s.getValue() for s in sensors]
    while len(ang) < 4:
        ang.append(float("nan"))
    fh.write("%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f\n"
             % (t, p[0], p[1], p[2], tilt, ang[0], ang[1], ang[2], ang[3]))
