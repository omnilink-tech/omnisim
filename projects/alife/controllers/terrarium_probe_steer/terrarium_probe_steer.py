#!/usr/bin/env python3

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

"""Steering probe director: constant turn command per creature, measure yaw.

Uses the SAME ecology.joint_targets modulation the life director uses, so what
is measured here is exactly the steering channel the ecosystem relies on.
"""
import json
import math
import os
import sys
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
ALIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
RUN = os.path.join(ALIFE, "_run", "probe_steer")
sys.path.insert(0, ALIFE)
from alife import ecology as eco  # noqa: E402

SETTLE = 150            # hip-servo rocking after the drop lasts past 60 ticks
MEASURE_S = 15.0

POP = json.load(open(os.path.join(RUN, "population.json"), encoding="utf-8"))
r = Supervisor()
DT = int(r.getBasicTimeStep())
TICKS = SETTLE + int(MEASURE_S * 1000 / DT)

roots, fields, missing = {}, {}, []
for g in POP:
    i = g["slot"]
    n = r.getFromDef("CREATURE_%d" % i)
    if n is None:
        missing.append("CREATURE_%d" % i)
        continue
    roots[i] = n
    for k, pair in enumerate(g["body"]["pairs"]):
        for side in "LR":
            for jt in (["H", "K"] if len(pair["segments"]) > 1 else ["H"]):
                d = "C%d_P%d_%s_%s_PARAMS" % (i, k, side, jt)
                p = r.getFromDef(d)
                f = p.getField("position") if p is not None else None
                if f is None:
                    missing.append(d)
                else:
                    fields[(i, k, side, jt)] = f
if missing:
    print("[steer] MISSING %d: %s" % (len(missing), missing[:6]), flush=True)
print("[steer] %d creatures / %d joints, DT=%d" % (len(roots), len(fields), DT), flush=True)

# turn -> left/right scales exactly as ecology.steer does it
SCALES = {}
for g in POP:
    gain = g["brain"]["steer_gain"]
    t = g["_turn"]
    SCALES[g["slot"]] = (max(0.0, 1.0 - gain * t), max(0.0, 1.0 + gain * t))

def yaw_of(node):
    return eco.yaw_from_orientation(node.getOrientation())

engine_ms, tick = [], 0
start_yaw, unwrapped, last_yaw, path, last_pos = {}, {}, {}, {}, {}
bufs = {g["slot"]: {} for g in POP}

while True:
    ts = time.perf_counter()
    if r.step(DT) == -1:
        break
    engine_ms.append((time.perf_counter() - ts) * 1000.0)
    t = tick * DT / 1000.0

    for g in POP:
        i = g["slot"]
        if i not in roots:
            continue
        ls, rs = SCALES[i]
        tg = eco.joint_targets(g["brain"], t, ls, rs, out=bufs[i])
        for (k, side, jt), v in tg.items():
            f = fields.get((i, k, side, jt))
            if f is not None:
                f.setSFFloat(v)

    if tick == SETTLE:
        for i in roots:
            y = yaw_of(roots[i])
            start_yaw[i] = y; unwrapped[i] = 0.0; last_yaw[i] = y
            path[i] = 0.0; last_pos[i] = roots[i].getPosition()
    elif tick > SETTLE and tick % 5 == 0:
        for i in roots:
            y = yaw_of(roots[i])
            unwrapped[i] += eco.wrap_angle(y - last_yaw[i]); last_yaw[i] = y
            p = roots[i].getPosition()
            path[i] += math.dist(p[:2], last_pos[i][:2]); last_pos[i] = p

    tick += 1
    if tick >= TICKS:
        break

out = {"creatures": [], "engine_ms_per_step_median": None}
for g in POP:
    i = g["slot"]
    if i not in unwrapped:
        continue
    out["creatures"].append({
        "slot": i, "turn": g["_turn"],
        "yaw_deg": math.degrees(unwrapped[i]),
        "path_m": path[i],
        "curvature": (unwrapped[i] / path[i]) if path[i] > 0.05 else 0.0,
    })
w = sorted(engine_ms[50:] or engine_ms)
out["engine_ms_per_step_median"] = w[len(w) // 2] if w else None
json.dump(out, open(os.path.join(RUN, "result.json"), "w", encoding="utf-8"), indent=1)
print("[steer] wrote result", flush=True)
r.simulationQuit(0)
