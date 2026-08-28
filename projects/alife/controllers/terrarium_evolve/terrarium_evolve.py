#!/usr/bin/env python3
"""Evolution director: actuate every creature from its genome, score locomotion.

One process drives the entire population. Creatures are `controller "<none>"`
Robots with no process of their own; this supervisor writes each hinge's
`HingeJointParameters.position` field every tick. Those writes are POSTPONED and
drained as one batch immediately before the engine pushes motor targets into
Newton, so the whole population costs ONE round trip per tick.

Measured alternative, for the record: `setJointPosition` is a blocking IPC flush
whose cost scales with engine step time (0.307 ms/call at 12 ms/step, 9.96 ms at
27 ms/step). At 32 joints that is 319 ms/tick against 0.223 ms for the batched
path -- about 1400x. Never actuate with setJointPosition.

Writes _run/fitness.json. Env: PROBE_TICKS, SETTLE_TICKS.
"""
import json
import math
import os
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.normpath(os.path.join(HERE, "..", "..", "_run"))
POP_PATH = os.path.join(RUN, "population.json")
OUT_PATH = os.path.join(RUN, "fitness.json")

TICKS = int(os.environ.get("PROBE_TICKS", "700"))
SETTLE = int(os.environ.get("SETTLE_TICKS", "60"))   # let them drop and settle

with open(POP_PATH, encoding="utf-8") as f:
    POP = json.load(f)

r = Supervisor()
# Derive the tick from the world -- never hardcode it. The CPG phase
# computation (t = tick*DT) breaks silently if this drifts from
# WorldInfo.basicTimeStep.
DT = int(r.getBasicTimeStep())
out = {"n": len(POP), "ticks": TICKS, "settle": SETTLE, "creatures": {}, "notes": []}


def note(m):
    print("[evolve] %s" % m, flush=True)
    out["notes"].append(m)


# ------------------------------------------------------------------ resolve
roots, fields, missing = {}, {}, []
for i, g in enumerate(POP):
    n = r.getFromDef("CREATURE_%d" % i)
    if n is None:
        missing.append("CREATURE_%d" % i)
        continue
    roots[i] = n
    for j in range(len(g["limbs"])):
        p = r.getFromDef("C%d_J%d_PARAMS" % (i, j))
        fld = p.getField("position") if p is not None else None
        if fld is None:
            missing.append("C%d_J%d_PARAMS" % (i, j))
        else:
            fields[(i, j)] = fld

if missing:
    note("MISSING %d refs, first: %s" % (len(missing), missing[:5]))
note("driving %d creatures / %d joints" % (len(roots), len(fields)))

engine_ms = []
start = {}
tick = 0

while True:
    ts = time.perf_counter()
    if r.step(DT) == -1:
        break
    engine_ms.append((time.perf_counter() - ts) * 1000.0)

    t = tick * (DT / 1000.0)

    # ---------------------------------------------------------- CPG actuation
    # target = bias + amp*sin(2*pi*freq*t + phase), one batch of postponed
    # field SETs drained just before the engine global motor push.
    for (i, j), fld in fields.items():
        g = POP[i]
        lb = g["limbs"][j]
        fld.setSFFloat(lb["bias"] + lb["amp"]
                       * math.sin(2.0 * math.pi * g["freq"] * t + lb["phase"]))

    # Record the start pose only AFTER the drop settles, so fitness measures
    # locomotion rather than the fall from spawn height.
    if tick == SETTLE:
        for i in roots:
            start[i] = list(roots[i].getPosition())

    if tick % 200 == 0 and tick > SETTLE and start:
        best = max((math.dist(roots[i].getPosition()[:2], start[i][:2]), i)
                   for i in roots)
        note("t=%4d best=%.3f m (creature %d)" % (tick, best[0], best[1]))

    tick += 1
    if tick >= TICKS:
        break

# ------------------------------------------------------------------ scoring
for i in roots:
    p = list(roots[i].getPosition())
    s = start.get(i)
    rec = {"id": POP[i]["id"], "end": p, "start": s}
    if s is None:
        rec.update({"fitness": 0.0, "status": "no_start"})
    elif any(not math.isfinite(v) for v in p) or max(abs(v) for v in p) > 500.0:
        # MuJoCo's instability channel is read by NOTHING -- a creature that
        # NaNs or launches emits no engine log line and the run still PASSes.
        rec.update({"fitness": 0.0, "status": "diverged"})
    elif p[2] < s[2] - 1.0:
        # Fell off the floor edge. There is no implicit ground plane, so it is
        # now falling forever; its XY reading is wherever it left the world.
        rec.update({"fitness": 0.0, "status": "off_floor"})
    else:
        rec.update({"fitness": math.dist(p[:2], s[:2]),
                    "dz": p[2] - s[2], "status": "ok"})
    out["creatures"][str(i)] = rec

if engine_ms:
    w = sorted(engine_ms[40:] or engine_ms)
    out["engine_ms_per_step_median"] = w[len(w) // 2]

ok = [(v["fitness"], v["id"]) for v in out["creatures"].values() if v["status"] == "ok"]
ok.sort(reverse=True)
out["best"] = {"fitness": ok[0][0], "id": ok[0][1]} if ok else None
out["ticks_run"] = tick

os.makedirs(RUN, exist_ok=True)
with open(OUT_PATH, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
note("best=%s  engine=%.1f ms/step  -> %s"
     % (out["best"], out.get("engine_ms_per_step_median", -1), OUT_PATH))

# End the run NOW. `run-headless --duration N` is a wall-clock SLEEP, not a
# progress target: without this the engine idles out the remaining budget and a
# generation costs 61 s to do 3 s of simulation. The fitness file is already
# flushed above, so quitting here loses nothing.
r.simulationQuit(0)
