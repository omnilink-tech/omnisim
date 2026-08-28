"""Steering-channel sweep on ONE body: which asymmetry turns it cleanly?
Conditions per slot come from population.json ("_cond"). Measures curvature
(yaw/path), speed and flips over MEASURE_S after a settle."""
import json, math, os, sys, time
from omnisim import Supervisor
HERE = os.path.dirname(os.path.abspath(__file__)); ALIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
RUN = os.path.join(ALIFE, "_run", "probe_steer2"); sys.path.insert(0, ALIFE)
from alife import ecology as eco
POP = json.load(open(os.path.join(RUN, "population.json"), encoding="utf-8"))
r = Supervisor(); DT = int(r.getBasicTimeStep())
SETTLE, MEASURE_S = 250, 20.0
TICKS = SETTLE + int(MEASURE_S * 1000 / DT)
roots, fields = {}, {}
for g in POP:
    i = g["slot"]; n = r.getFromDef("CREATURE_%d" % i); roots[i] = n
    for k, pair in enumerate(g["body"]["pairs"]):
        for side in "LR":
            for jt in (["H", "K"] if len(pair["segments"]) > 1 else ["H"]):
                p = r.getFromDef("C%d_P%d_%s_%s_PARAMS" % (i, k, side, jt))
                if p is not None: fields[(i, k, side, jt)] = p.getField("position")
def yaw_of(n): return eco.yaw_from_orientation(n.getOrientation())
st = {i: {"yaw": 0.0, "last": None, "path": 0.0, "lp": None, "flips": 0, "down": False, "zs": 0.0, "zn": 0} for i in roots}
bufs = {i: {} for i in roots}; ems = []; tick = 0
while True:
    ts = time.perf_counter()
    if r.step(DT) == -1: break
    ems.append((time.perf_counter() - ts) * 1000)
    t = tick * DT / 1000.0
    for g in POP:
        i = g["slot"]; c = g["_cond"]
        tg = eco.joint_targets(g["brain"], t, c["ls"], c["rs"], bufs[i], c["lb"], c["rb"], ramp=min(1.0, t / 1.5))
        for key, v in tg.items():
            f = fields.get((i,) + key)
            if f is not None: f.setSFFloat(v)
        if tick >= SETTLE and tick % 5 == 0:
            s = st[i]; R = roots[i].getOrientation(); y = math.atan2(R[3], R[0]); p = roots[i].getPosition()
            if s["last"] is not None:
                s["yaw"] += eco.wrap_angle(y - s["last"]); s["path"] += math.dist(p[:2], s["lp"][:2])
            s["last"], s["lp"] = y, p
            s["zs"] += p[2]; s["zn"] += 1
            down = R[8] < 0.35
            if down and not s["down"]: s["flips"] += 1
            s["down"] = down
    tick += 1
    if tick >= TICKS: break
out = {"rows": [], "engine_ms": sorted(ems[50:])[len(ems[50:]) // 2]}
for g in POP:
    s = st[g["slot"]]
    out["rows"].append({"cond": g["_cond"]["name"], "yaw_deg": math.degrees(s["yaw"]), "path_m": s["path"],
                        "curv": s["yaw"] / s["path"] if s["path"] > 0.05 else 0.0,
                        "speed": s["path"] / MEASURE_S, "flips": s["flips"],
                        "mean_z": s["zs"] / max(1, s["zn"])})
json.dump(out, open(os.path.join(RUN, "result.json"), "w"), indent=1)
r.simulationQuit(0)
