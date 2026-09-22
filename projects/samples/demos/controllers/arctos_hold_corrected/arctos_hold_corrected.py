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

"""Hold one pose on the corrected Arctos description and report the droop.

The per-joint torque ceiling is baked into the URDF <limit effort=> before the
run; this controller only commands the pose, lets it settle, and measures how
far each joint ended up from where it was told to go.
"""
import argparse
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260914\hold\results"
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
POSES = {"arm_out_90": {"joint2": math.pi / 2}, "home": {}}

ap = argparse.ArgumentParser()
ap.add_argument("--pose", required=True)
ap.add_argument("--effort", required=True)
args, _ = ap.parse_known_args()

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

motors, sensors, missing = {}, {}, []
for j in ARM:
    m = robot.getDevice(f"{j}_motor")
    s = robot.getDevice(f"{j}_sensor")
    if m is None or s is None:
        missing.append(j)
        continue
    s.enable(dt)
    m.setVelocity(2.0)
    motors[j], sensors[j] = m, s

if missing:
    print(f"[hold] FATAL missing devices {missing}", flush=True)
    sys.exit(1)

cmd = {j: POSES[args.pose].get(j, 0.0) for j in ARM}
for j in ARM:
    motors[j].setPosition(cmd[j])

# settle
T_SETTLE = 6.0
t = 0.0
while robot.step(dt) != -1 and t < T_SETTLE:
    t += dt / 1000.0

ach = {j: sensors[j].getValue() for j in ARM}
err = {j: ach[j] - cmd[j] for j in ARM}

# a joint that cannot hold its commanded angle sags away from it; call it held
# if it is within a quarter of a degree
HELD_RAD = math.radians(0.25)
held = {j: bool(abs(err[j]) <= HELD_RAD) for j in ARM}

res = {
    "pose": args.pose,
    "joint2_effort_Nm": float(args.effort),
    "settle_s": T_SETTLE,
    "commanded_rad": cmd,
    "achieved_rad": ach,
    "error_rad": err,
    "error_deg": {j: math.degrees(err[j]) for j in ARM},
    "held": held,
    "joint2_held": held["joint2"],
    "joint2_error_deg": math.degrees(err["joint2"]),
}
os.makedirs(OUT, exist_ok=True)
path = os.path.join(OUT, f"hold_{args.pose}_e{float(args.effort):05.1f}.json")
with open(path, "w") as f:
    json.dump(res, f, indent=1)
print(f"[hold] pose={args.pose} effort={args.effort} "
      f"joint2_err={math.degrees(err['joint2']):+.4f} deg "
      f"held={held['joint2']}", flush=True)
sys.stdout.flush()
