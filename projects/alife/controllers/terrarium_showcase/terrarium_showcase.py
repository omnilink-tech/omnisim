#!/usr/bin/env python3
"""Showcase director: drive evolved champions forever, for watching or capture.

Same actuation path as the evolution director (batched postponed field writes to
HingeJointParameters.position, one round trip per tick for the whole cast), but
it never scores and never quits, and it periodically resets the cast so a loop
can be filmed.

Env:
  LOOP_TICKS   ticks per lap before the cast is reset (0 = never reset)
  SETTLE_TICKS ticks to let creatures drop and settle before the lap timer runs
"""
import json
import math
import os

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.normpath(os.path.join(HERE, "..", "..", "_run"))
POP_PATH = os.path.join(RUN, "showcase_population.json")

LOOP = int(os.environ.get("LOOP_TICKS", "0"))
SETTLE = int(os.environ.get("SETTLE_TICKS", "60"))

with open(POP_PATH, encoding="utf-8") as f:
    POP = json.load(f)

r = Supervisor()
# Derive the tick from the world -- never hardcode it. The CPG phase
# computation (t = tick*DT) breaks silently if this drifts from
# WorldInfo.basicTimeStep.
DT = int(r.getBasicTimeStep())

roots, fields = {}, {}
for i, g in enumerate(POP):
    n = r.getFromDef("CREATURE_%d" % i)
    if n is None:
        continue
    roots[i] = n
    for j in range(len(g["limbs"])):
        p = r.getFromDef("C%d_J%d_PARAMS" % (i, j))
        fld = p.getField("position") if p is not None else None
        if fld is not None:
            fields[(i, j)] = fld

print("[showcase] driving %d champions / %d joints" % (len(roots), len(fields)),
      flush=True)
for i, g in enumerate(POP):
    print("[showcase]   %d: %s  (gen %s, fitness %.3f m)"
          % (i, g["id"], g.get("_gen", "?"), g.get("_fitness", 0.0)), flush=True)

home = {i: list(roots[i].getField("translation").getSFVec3f()) for i in roots}
start, lap, tick = {}, 0, 0

while r.step(DT) != -1:
    t = lap * (DT / 1000.0)

    for (i, j), fld in fields.items():
        g = POP[i]
        lb = g["limbs"][j]
        fld.setSFFloat(lb["bias"] + lb["amp"]
                       * math.sin(2.0 * math.pi * g["freq"] * t + lb["phase"]))

    if lap == SETTLE:
        for i in roots:
            start[i] = list(roots[i].getPosition())

    if lap % 250 == 0 and lap > SETTLE and start:
        rank = sorted(((math.dist(roots[i].getPosition()[:2], start[i][:2]), i)
                       for i in roots), reverse=True)
        print("[showcase] t=%5d  " % lap + "  ".join(
            "%s=%.2fm" % (POP[i]["id"], d) for d, i in rank[:4]), flush=True)

    lap += 1
    tick += 1
    if LOOP and lap >= LOOP:
        # Reset the cast to their pads and run the lap again.
        for i in roots:
            roots[i].getField("translation").setSFVec3f(home[i])
            roots[i].getField("rotation").setSFRotation([0, 0, 1, 0])
            roots[i].resetPhysics()
        start, lap = {}, 0
        print("[showcase] --- lap reset ---", flush=True)
