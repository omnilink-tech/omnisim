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

"""Teleport-freeze probe: does translation+rotation+resetPhysics+setVelocity
leave a creature physically live? Creature 0 gets the director's exact revive
sequence at tick 200; creature 1 is the untouched control. Both are driven by
the same hip sine so a frozen body is unmistakable."""
import json, math, os, sys, time
from omnisim import Supervisor
HERE = os.path.dirname(os.path.abspath(__file__)); ALIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
r = Supervisor(); DT = int(r.getBasicTimeStep())
nodes = {i: r.getFromDef("CREATURE_%d" % i) for i in range(2)}
hips = {i: r.getFromDef("C%d_P0_L_H_PARAMS" % i).getField("position") for i in range(2)}
hipsR = {i: r.getFromDef("C%d_P0_R_H_PARAMS" % i).getField("position") for i in range(2)}
MODE = os.environ.get("TP_MODE", "full")   # full | novel (no setVelocity) | none
log = {"mode": MODE, "rows": []}
tick = 0
while r.step(DT) != -1:
    t = tick * DT / 1000.0
    for i in range(2):
        hips[i].setSFFloat(0.35 * math.sin(2 * math.pi * 1.0 * t))
        hipsR[i].setSFFloat(0.35 * math.sin(2 * math.pi * 1.0 * t + math.pi))
    if tick == 200 and MODE != "none":
        n = nodes[0]
        n.getField("translation").setSFVec3f([0.0, 2.0, 0.35])
        n.getField("rotation").setSFRotation([0, 0, 1, 0.5])
        n.resetPhysics()
        if MODE == "full":
            n.setVelocity([0.0] * 6)
    if tick % 60 == 0:
        row = [tick] + [round(v, 4) for v in nodes[0].getPosition()] + [round(v, 4) for v in nodes[1].getPosition()]
        log["rows"].append(row)
    tick += 1
    if tick >= 900:
        break
json.dump(log, open(os.path.join(ALIFE, "_run", "tp_%s.json" % MODE), "w"), indent=1)
r.simulationQuit(0)
