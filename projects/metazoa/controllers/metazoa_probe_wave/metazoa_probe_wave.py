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

"""Wave-direction / steering-sign probe on ONE welded 4-chain.

Four phases of PHASE_S each: +dphi, -dphi, +dphi with +yaw bias, +dphi with
-yaw bias. Reports the chain's displacement projected on its own spine axis
(tail = spine index 0 -> head = index 3; positive = head-first) and its yaw
change per phase. This is the measurement the reef controller needs and could
not get from ecology runs (every run so far backed away from its recruit).
"""
import json
import math
import os
import sys
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
ALIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ALIFE)
from mz import organism as ORG  # noqa: E402

RUN = os.path.join(ALIFE, "_run", "probe_wave")
cfg = json.load(open(os.path.join(RUN, "config.json"), encoding="utf-8"))
SPINE = cfg["spine"]                    # tail -> head cell ids
PATTERN = cfg["pattern"]
GEN = cfg["genome"]
PHASE_S = cfg.get("phase_s", 15.0)
SETTLE = 25

r = Supervisor()
DT = int(r.getBasicTimeStep())
DT_S = DT / 1000.0
roots = {i: r.getFromDef("CELL_%d" % i) for i in SPINE}
hinge = {i: r.getFromDef("CELL_%d_HINGE_PARAMS" % i).getField("position") for i in SPINE}
n = len(SPINE)
bp = {"target_length": 8, "dock_rotation_pattern": PATTERN, "branch_rule": "none"}


def yaw_of(node):
    R = node.getOrientation()
    return math.atan2(R[3], R[0])


def snapshot():
    ps = [roots[i].getPosition() for i in SPINE]
    cx = sum(p[0] for p in ps) / n
    cy = sum(p[1] for p in ps) / n
    sx, sy = ps[-1][0] - ps[0][0], ps[-1][1] - ps[0][1]
    L = math.hypot(sx, sy) or 1.0
    return (cx, cy), (sx / L, sy / L), yaw_of(roots[-1] if False else roots[SPINE[-1]])


phases = [tuple(p) for p in cfg.get("phases", [("+dphi", 1.0, 0.0), ("-dphi", -1.0, 0.0), ("+dphi steer+", 1.0, 1.0), ("+dphi steer-", 1.0, -1.0)])]
RUDDER = bool(cfg.get("rudder", False))   # yaw cells carry NO wave: hinge = bias_yaw + steer_gain*steer
out = {"phases": [], "genome": GEN, "pattern": PATTERN}
tick, ph, ph_t0, start = 0, -1, 0.0, None
locked = False
while r.step(DT) != -1:
    t = tick * DT_S
    if tick == SETTLE and not locked:
        for a, b in zip(SPINE[1:], SPINE[:-1]):
            r.getFromDef("CELL_%d_F_TAIL" % a).getField("isLocked").setSFBool(True)
        locked = True
    if tick > SETTLE:
        k = min(len(phases) - 1, int((t - SETTLE * DT_S) / PHASE_S))
        if k != ph:
            if ph >= 0:
                (cx, cy), axis, yaw = snapshot()
                (c0, a0, y0) = start
                dx, dy = cx - c0[0], cy - c0[1]
                out["phases"].append({"name": phases[ph][0],
                                      "along_spine_m": dx * a0[0] + dy * a0[1],   # + = toward the head
                                      "lateral_m": -dx * a0[1] + dy * a0[0],
                                      "dist_m": math.hypot(dx, dy),
                                      "yaw_change_rad": (yaw - y0 + math.pi) % (2 * math.pi) - math.pi})
            ph = k
            start = snapshot()
            ph_t0 = t
        name, sgn, steer = phases[ph]
        g = dict(GEN)
        g["dphi"] = sgn * abs(GEN["dphi"])
        ramp = min(1.0, (t - ph_t0) / 1.0)
        targets = ORG.chain_targets(g, bp, n, t, steer)
        for i, cid in enumerate(SPINE):
            if RUDDER and ORG.axis_of(bp, i) == "yaw":
                targets[i] = g["bias_yaw"] + g["steer_gain"] * steer
            hinge[cid].setSFFloat(ramp * targets[i])
    tick += 1
    if tick > SETTLE + int(len(phases) * PHASE_S / DT_S) + 5:
        break

(cx, cy), axis, yaw = snapshot()
(c0, a0, y0) = start
dx, dy = cx - c0[0], cy - c0[1]
out["phases"].append({"name": phases[ph][0], "along_spine_m": dx * a0[0] + dy * a0[1],
                      "lateral_m": -dx * a0[1] + dy * a0[0], "dist_m": math.hypot(dx, dy),
                      "yaw_change_rad": (yaw - y0 + math.pi) % (2 * math.pi) - math.pi})
json.dump(out, open(os.path.join(RUN, "result.json"), "w"), indent=1)
r.simulationQuit(0)
