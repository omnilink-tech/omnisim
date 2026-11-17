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

"""Probe: drive the X30 through _crawl_motion verbs and print the
measured results plus fall checks. controllerArgs:
  --vx 0.14 --freq 0.7 --wz 0.3 --seq "walk:2,turn:90,walk:1,turn:-90,goto:2:1"
"""
import argparse, math, os, sys, json, time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", "..", "..",
                                                "projects", "samples", "demos", "controllers", "omnilink_quadruped_bridge")))
import _crawl_motion as cm
from omnisim import Supervisor

p = argparse.ArgumentParser()
p.add_argument("--robot", default="x30")
p.add_argument("--vx", type=float, default=None)
p.add_argument("--freq", type=float, default=None)
p.add_argument("--wz", type=float, default=None)
p.add_argument("--lead-walk", type=float, default=None)
p.add_argument("--lead-turn", type=float, default=None)
p.add_argument("--seq", default="walk:2,turn:90,walk:1,turn:-90,goto:2:1")
p.add_argument("--settle", type=float, default=2.0)
p.add_argument("--out", default=None)
p.add_argument("--gait", default="trot")
p.add_argument("--step-height", type=float, default=None)
a, _ = p.parse_known_args()

if a.step_height:
    for k in cm._dr.ROBOTS: cm._dr.ROBOTS[k]["step_height"] = a.step_height
robot = Supervisor()
ts = int(robot.getBasicTimeStep()); dt = ts / 1000.0
cfg = cm._dr.ROBOTS[a.robot]
motors = {}
for leg in cm.LEGS:
    pre = cfg["prefixes"][leg]
    for jn, key in zip(cfg["joints"], cm.JOINTS):
        m = robot.getDevice(f"{pre}_{jn}_motor") or robot.getDevice(f"{pre}_{jn}")
        s = m.getPositionSensor(); s.enable(ts)
        motors[(leg, key)] = (m, s)
me = robot.getSelf()
terrain = cm._dr.TerrainMap.load(robot, "TERRAIN_GEOM", "TERRAIN")
d = cm.CrawlDriver(a.robot, dt, terrain=terrain, gait=a.gait, vx_max=a.vx, freq=a.freq, wz_max=a.wz,
                   lead_walk_s=a.lead_walk, lead_turn_s=a.lead_turn)
robot.step(ts)
start = {k: (v[1].getValue()) for k, v in motors.items()}
t0 = robot.getTime()
out = open(a.out, "w") if a.out else None

def log(obj):
    line = json.dumps(obj)
    print("[x30_probe] " + line, flush=True)
    if out: out.write(line + "\n"); out.flush()

queue = []
for item in a.seq.split(","):
    f = item.split(":")
    queue.append(f)
cur = None
min_up = 1.0; max_tilt = 0.0; zmin = 9
wall0 = time.time()
while robot.step(ts) != -1:
    t = robot.getTime() - t0
    pos = me.getPosition(); o = me.getOrientation()
    yaw = math.atan2(o[3], o[0])
    up_z = o[8]; min_up = min(min_up, up_z); zmin = min(zmin, pos[2])
    if t < a.settle:
        s = t / a.settle; s = s * s * (3 - 2 * s)
        for leg in cm.LEGS:
            for i, key in enumerate(cm.JOINTS):
                st = start[(leg, key)]
                motors[(leg, key)][0].setPosition(st + (d.stand_q[leg][i] - st) * s)
        continue
    if up_z < 0.5:
        log({"event": "FELL", "t": t, "pos": pos, "up_z": up_z}); break
    if cur is None:
        if not queue:
            log({"event": "done", "t": t, "min_up_z": min_up, "zmin": zmin,
                 "rtf": t / max(1e-6, time.time() - wall0)}); break
        f = queue.pop(0)
        if f[0] == "walk": cur = d.walk(float(f[1]))
        elif f[0] == "turn": cur = d.turn(math.radians(float(f[1])))
        elif f[0] == "goto": cur = d.walk_to(float(f[1]), float(f[2]), None if len(f) < 4 else math.radians(float(f[3])))
        elif f[0] == "wait": cur = d.halt()
        log({"event": "issue", "cmd": f, "t": t, "pose": [pos[0], pos[1], yaw]})
    q = d.tick(robot.getTime(), pos[0], pos[1], yaw, up_body=(o[6], o[7]))
    if int(t / dt) % int(round(1.0 / dt)) == 0:
        log({"event": "pose", "t": round(t, 3), "x": round(pos[0], 4), "y": round(pos[1], 4), "z": round(pos[2], 4), "yaw": round(yaw, 4), "up_z": round(up_z, 4), "vx_cmd": round(d.vx, 3), "wz_cmd": round(d.wz, 3), "blend": round(d.blend, 2)})
    for leg in cm.LEGS:
        for i, key in enumerate(cm.JOINTS):
            motors[(leg, key)][0].setPosition(q[leg][i])
    c = d.completed
    if c is not None and c["seq"] == cur:
        c = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in c.items()}
        c["min_up_z_so_far"] = round(min_up, 3)
        log({"event": "result", **c}); cur = None
if out: out.close()
