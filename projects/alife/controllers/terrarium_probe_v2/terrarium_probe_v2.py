#!/usr/bin/env python3
"""Gate-A probe director for alife v2 (DESIGN_v2.md, "Verification gates").

Drives the probe_v2 world's creatures from their genome CPGs and measures, in
ONE headless run:

  REST     torso height after a 60-tick settle with every joint held at 0,
           against the geometric expectation the generator wrote into the
           population file. Wrong height == wrong capsule orientation.
  COST     engine ms/step, timed AROUND r.step() (engine step + IPC).
  MOTION   per-creature horizontal displacement from the settled pose, so we
           know the bodies locomote at all.
  REVIVE   tick 800: creature 0 is PARKED (teleport to the pit over nothing,
           resetPhysics -> free-fall, zero contacts). Tick 2000: REVIVED to
           its home pose. 120 ticks later: its z vs rest, and |v|.

Actuation is the measured path: batched supervisor field writes to
HingeJointParameters.position, drained as one round trip per tick. Never
setJointPosition (blocking IPC flush, ~1400x costlier under load).

Writes _run/probe_v2_result.json every 200 ticks and at the end, then
simulationQuit(0) -- `run-headless --duration N` is a wall-clock SLEEP.
"""
import json
import math
import os
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.normpath(os.path.join(HERE, "..", "..", "_run"))
POP_PATH = os.path.join(RUN, "probe_v2_population.json")
OUT_PATH = os.path.join(RUN, "probe_v2_result.json")

T_REST_EARLY = 60                  # the contract's 60-tick reading (reported)
SETTLE = 150                       # the reading the verdict uses: 0.48 s is
                                   # still mid-bounce on the hip servos
T_PARK = 800
T_FALL_CHECK = T_PARK + 300
T_REVIVE = 2000
T_REVIVE_CHECK = T_REVIVE + 120
T_END = 2400
PARK = [60.0, 60.0, 5.0]
SUBJECT = 0                        # the creature that takes the park/revive trip

with open(POP_PATH, encoding="utf-8") as f:
    POP = json.load(f)

r = Supervisor()
# Derive the tick from the world -- never hardcode it. The CPG phase
# computation (t = tick*DT) breaks silently if this drifts from
# WorldInfo.basicTimeStep.
DT = int(r.getBasicTimeStep())

R = {"dt_ms": DT, "n": len(POP), "rest": {}, "cost": {}, "motion": {},
     "revive": {}, "watchdog": [], "notes": []}


def note(msg):
    print("[probe_v2] %s" % msg, flush=True)
    R["notes"].append(msg)


def dump():
    try:
        with open(OUT_PATH, "w", encoding="utf-8") as f:
            json.dump(R, f, indent=1)
    except Exception as exc:              # never let bookkeeping kill the run
        print("[probe_v2] dump failed: %s" % exc, flush=True)


# ---------------------------------------------------------------- resolve
# creatures[slot] = {node, trans, rot, joints: [(field, brain_joint, side)]}
creatures, missing = {}, []
for g in POP:
    i = g["slot"]
    n = r.getFromDef("CREATURE_%d" % i)
    if n is None:
        missing.append("CREATURE_%d" % i)
        continue
    joints = []
    for k, p in enumerate(g["body"]["pairs"]):
        for side in ("L", "R"):
            for s_i in range(len(p["segments"])):
                jn, jname = ("H", "hip") if s_i == 0 else ("K", "knee")
                dname = "C%d_P%d_%s_%s_PARAMS" % (i, k, side, jn)
                pn = r.getFromDef(dname)
                fld = pn.getField("position") if pn is not None else None
                if fld is None:
                    missing.append(dname)
                else:
                    joints.append((fld, g["brain"]["pairs"][k][jname], side))
    creatures[i] = {"g": g, "node": n, "trans": n.getField("translation"),
                    "rot": n.getField("rotation"), "joints": joints,
                    "mode": "settle", "t0": SETTLE}
if missing:
    note("MISSING %d DEFs/fields, first: %s" % (len(missing), missing[:6]))
R["joints_resolved"] = sum(len(c["joints"]) for c in creatures.values())
note("driving %d creatures / %d joints, DT=%d ms"
     % (len(creatures), R["joints_resolved"], DT))


def actuate(c, tick):
    """CPG targets for one creature; mirror_phase on the right side."""
    br = c["g"]["brain"]
    w = 2.0 * math.pi * br["freq"] * ((tick - c["t0"]) * DT / 1000.0)
    mirror = br["mirror_phase"]
    for fld, j, side in c["joints"]:
        ph = j["phase"] + (mirror if side == "R" else 0.0)
        fld.setSFFloat(j["bias"] + j["amp"] * math.sin(w + ph))


def hold_zero(c):
    for fld, _j, _s in c["joints"]:
        fld.setSFFloat(0.0)


def lin_speed(node):
    """|linear| of getVelocity(). REPORTED, NOT USED FOR A VERDICT: measured
    on creatures sitting at exactly their predicted rest height it reads
    0.03-0.43 m/s and grows with distance from the world origin, i.e. it is
    the spatial twist's linear part (v at the origin), not the COM speed."""
    v = node.getVelocity()
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


FD_TICKS = 10                      # finite-difference window for drift speed
pos_samples = {}                   # (tick, slot) -> position, taken FD_TICKS early


def drift_speed(i, c, tick):
    """COM speed as position drift over the last FD_TICKS ticks -- the honest
    'is it at rest' number."""
    prev = pos_samples.get((tick - FD_TICKS, i))
    if prev is None:
        return None
    return math.dist(c["node"].getPosition(), prev) / (FD_TICKS * DT / 1000.0)


def teleport(c, pos, yaw=0.0):
    """Move a creature and bring it to rest. MEASURED (this probe, first run):
    `resetPhysics()` does NOT zero a Newton body's velocity -- OmSolid.cpp
    resetSingleSolidPhysics() clears only the display fields and says so --
    and the teleport's own reset_body_pose zeroes body_qd but then eval_fk
    re-derives it from the free joint's joint_qd, which still holds the old
    value. A creature revived after 9.6 s of free-fall came back at 94 m/s and
    punched through the floor (z=-94 at the check). `setVelocity` (W3.2) writes
    BOTH body_qd and the free-joint joint_qd, so it is the call that sticks."""
    c["trans"].setSFVec3f(list(pos))
    c["rot"].setSFRotation([0.0, 0.0, 1.0, yaw])
    c["node"].resetPhysics()
    c["node"].setVelocity([0.0] * 6)


def _mat(node):
    o = node.getOrientation()
    return [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _tr(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _rx(s):
    c, sn = math.cos(s), math.sin(s)
    return [[1, 0, 0], [0, c, -sn], [0, sn, c]]


def pose_diag(c):
    """What the engine actually did with the authored frames: torso pitch/roll,
    every hinge angle recovered from link orientations (parent^T * link, with
    the authored splay roll divided out on the hip), and each foot's lowest
    point. Separates 'servo not holding 0' from 'torso toppled' from
    'collider in the wrong place' without a second run."""
    g = c["g"]
    i = g["slot"]
    Rt = _mat(c["node"])
    # ENU, torso +x forward: pitch = -asin(R[2][0]), roll = atan2(R[2][1], R[2][2])
    out = {"pitch_deg": math.degrees(-math.asin(max(-1.0, min(1.0, Rt[2][0])))),
           "roll_deg": math.degrees(math.atan2(Rt[2][1], Rt[2][2])),
           "joints": {}, "foot_z": {}}
    for k, p in enumerate(g["body"]["pairs"]):
        for side in ("L", "R"):
            s = p["splay"] if side == "L" else -p["splay"]
            parent, parentR = c["node"], Rt
            for s_i, seg in enumerate(p["segments"]):
                jn = "H" if s_i == 0 else "K"
                dname = "C%d_P%d_%s_%s" % (i, k, side, jn)
                link = r.getFromDef(dname + "_LINK")
                if link is None:
                    continue
                Rl = _mat(link)
                rel = _mul(_tr(parentR), Rl)
                if s_i == 0:
                    rel = _mul(rel, _rx(-s))          # divide out the authored roll
                out["joints"][dname] = math.atan2(rel[0][2], rel[0][0])
                parent, parentR = link, Rl
            # foot: last link origin + R_link * (0, 0, -L) - r
            pl = parent.getPosition()
            L, rad = p["segments"][-1]["length"], p["segments"][-1]["radius"]
            out["foot_z"]["P%d_%s" % (k, side)] = pl[2] - parentR[2][2] * L - rad
    return out


def rest_reading(tag, tick):
    rows = {}
    for i, c in creatures.items():
        p = c["node"].getPosition()
        exp = c["g"]["rest_expected"]
        rows[str(i)] = {"measured_z": p[2], "expected_z": exp,
                        "error_mm": (p[2] - exp) * 1000.0,
                        "drift_speed": drift_speed(i, c, tick),
                        "twist_linear_norm": lin_speed(c["node"]), "diag": pose_diag(c)}
    R[tag] = rows
    worst = max(abs(v["error_mm"]) for v in rows.values()) if rows else -1
    note("%s t=%d  worst |error| %.1f mm  " % (tag.upper(), tick, worst) + "  ".join(
        "%d:%.3f/%.3f(drift %.3f m/s)" % (int(i), v["measured_z"], v["expected_z"],
                                          v["drift_speed"] or -1.0)
        for i, v in rows.items()))
    for i, v in rows.items():
        d = v["diag"]
        pe = creatures[int(i)]["g"].get("pitch_expected_deg")
        v["pitch_expected_deg"] = pe
        note("  %s: pitch %+.1f (expected %s) roll %+.1f  |joint|max %.3f rad  foot_z %s"
             % (i, d["pitch_deg"], "%+.1f" % pe if pe is not None else "n/a", d["roll_deg"],
                max((abs(a) for a in d["joints"].values()), default=0.0),
                " ".join("%s=%+.3f" % (k, z) for k, z in d["foot_z"].items())))


engine_ms, loop_ms = [], []
start = {}
tick = 0

while True:
    ts = time.perf_counter()
    if r.step(DT) == -1:
        break
    engine_ms.append((time.perf_counter() - ts) * 1000.0)
    t0 = time.perf_counter()

    # ------------------------------------------------------------- REST check
    if tick + FD_TICKS in (T_REST_EARLY, SETTLE, T_REVIVE_CHECK):
        for i, c in creatures.items():
            pos_samples[(tick, i)] = list(c["node"].getPosition())
    if tick == T_REST_EARLY:
        rest_reading("rest_t60", tick)
    # Per-tick trace of one stable creature's first hip through the settle
    # window: a servo limit-cycle shows up as a +-rad wobble here.
    if 100 <= tick < SETTLE and 4 in creatures:
        d = pose_diag(creatures[4])
        R.setdefault("trace_c4", []).append(
            [tick, round(creatures[4]["node"].getPosition()[2], 4),
             round(d["joints"].get("C4_P0_L_H", 0.0), 4),
             round(d["joints"].get("C4_P0_L_K", 0.0), 4), round(d["pitch_deg"], 2)])
    if tick == SETTLE:
        rest_reading("rest", tick)
        for i, c in creatures.items():
            start[i] = list(c["node"].getPosition())
            c["mode"] = "run"

    # ------------------------------------------------------------- park / revive
    if tick == T_PARK and SUBJECT in creatures:
        c = creatures[SUBJECT]
        p = c["node"].getPosition()
        teleport(c, PARK)
        c["mode"] = "parked"
        R["revive"]["parked_at_tick"] = tick
        R["revive"]["pos_before_park"] = list(p)
        note("PARK t=%d creature %d -> %s" % (tick, SUBJECT, PARK))

    if tick == T_FALL_CHECK and SUBJECT in creatures:
        p = creatures[SUBJECT]["node"].getPosition()
        R["revive"]["parked_pos_after_%d_ticks" % (tick - T_PARK)] = list(p)
        note("FALL t=%d creature %d at z=%.1f (free-falling: %s)"
             % (tick, SUBJECT, p[2], p[2] < PARK[2] - 1.0))

    if tick == T_REVIVE and SUBJECT in creatures:
        c = creatures[SUBJECT]
        home = list(c["g"]["home"])
        teleport(c, home, 0.0)
        c["mode"] = "reviving"
        R["revive"]["revived_at_tick"] = tick
        R["revive"]["home"] = home
        note("REVIVE t=%d creature %d -> %s" % (tick, SUBJECT, home))

    if tick == T_REVIVE_CHECK and SUBJECT in creatures:
        c = creatures[SUBJECT]
        p = c["node"].getPosition()
        exp = c["g"]["rest_expected"]
        sp = drift_speed(SUBJECT, c, tick)
        R["revive"].update({"check_tick": tick, "z": p[2], "expected_z": exp,
                            "error_mm": (p[2] - exp) * 1000.0, "drift_speed": sp,
                            "twist_linear_norm": lin_speed(c["node"]),
                            "diag": pose_diag(c),
                            "xy_error_m": math.dist(p[:2], c["g"]["home"][:2]),
                            "verdict": "PASS" if (abs(p[2] - exp) < 0.02
                                                  and sp is not None and sp < 0.05) else "FAIL"})
        note("REVIVE-CHECK t=%d z=%.4f (expected %.4f, %.1f mm)  drift %.4f m/s  %s"
             % (tick, p[2], exp, (p[2] - exp) * 1000.0, sp if sp is not None else -1.0,
                R["revive"]["verdict"]))
        start[SUBJECT] = list(p)
        c["mode"] = "run"
        c["t0"] = tick

    # ------------------------------------------------------------- actuation
    for c in creatures.values():
        if c["mode"] == "run":
            actuate(c, tick)
        elif c["mode"] in ("parked", "reviving"):
            hold_zero(c)
        # "settle": leave the authored 0 targets alone

    # ------------------------------------------------------------- telemetry
    if tick % 200 == 0 and tick > SETTLE:
        disp = {i: math.dist(c["node"].getPosition()[:2], start[i][:2])
                for i, c in creatures.items() if c["mode"] == "run" and i in start}
        R["motion"]["disp_t%d" % tick] = disp
        note("t=%4d  " % tick + "  ".join("%d:%.2f" % (i, d) for i, d in sorted(disp.items())))
        dump()

    # MuJoCo's instability warning channel is read by NOTHING: an exploding or
    # NaN-ing creature emits no engine log line and run-headless still PASSes.
    # Parked slots are exempt: they are free-falling by design.
    if tick % 100 == 0:
        for i, c in creatures.items():
            if c["mode"] == "parked":
                continue
            p = c["node"].getPosition()
            if any(not math.isfinite(v) for v in p) or max(abs(v) for v in p) > 1e4:
                msg = "t=%d CREATURE_%d diverged: %s" % (tick, i, p)
                if msg not in R["watchdog"]:
                    R["watchdog"].append(msg)
                    note("WATCHDOG " + msg)

    loop_ms.append((time.perf_counter() - t0) * 1000.0)
    tick += 1
    if tick >= T_END:
        break

# ---------------------------------------------------------------- verdicts
final = {}
for i, c in creatures.items():
    p = list(c["node"].getPosition())
    s = start.get(i)
    rec = {"id": c["g"]["id"], "end": p, "start": s}
    if s is None:
        rec.update({"disp_m": 0.0, "status": "no_start"})
    elif any(not math.isfinite(v) for v in p) or max(abs(v) for v in p) > 1e4:
        rec.update({"disp_m": 0.0, "status": "diverged"})
    elif p[2] < s[2] - 1.0:
        rec.update({"disp_m": 0.0, "status": "off_floor"})
    else:
        rec.update({"disp_m": math.dist(p[:2], s[:2]), "dz": p[2] - s[2], "status": "ok"})
    if i == SUBJECT:
        rec["note"] = "displacement measured from the revive at tick %d" % T_REVIVE_CHECK
    final[str(i)] = rec
R["motion"]["final"] = final
R["motion"]["ticks_run"] = tick
ok = [v["disp_m"] for v in final.values() if v["status"] == "ok"]
R["motion"]["best_m"] = max(ok) if ok else None
R["motion"]["moving_count"] = sum(1 for d in ok if d > 0.05)

if engine_ms:
    w = sorted(engine_ms[SETTLE + 50:] or engine_ms)
    R["cost"] = {"engine_ms_per_step_median": w[len(w) // 2],
                 "engine_ms_per_step_p90": w[int(len(w) * 0.9)],
                 "engine_ms_per_step_mean": sum(w) / len(w),
                 "budget_ms": DT, "realtime": w[len(w) // 2] < DT,
                 "note": "timed around r.step() (engine step + IPC), ticks %d.." % (SETTLE + 50)}
if loop_ms:
    wl = sorted(loop_ms[SETTLE + 50:] or loop_ms)
    R["cost"]["controller_ms_per_tick_median"] = wl[len(wl) // 2]

rest_ok = all(abs(v["error_mm"]) < 20.0 for v in R["rest"].values()) if R["rest"] else False
R["verdict"] = {
    "rest": "PASS" if rest_ok else "FAIL",
    "revive": R["revive"].get("verdict", "NOT_RUN"),
    "motion": "PASS" if (R["motion"]["best_m"] or 0.0) > 0.05 else "FAIL",
    "watchdog": "PASS" if not R["watchdog"] else "FAIL",
}
dump()
note("DONE rest=%s revive=%s motion=%s(best %.3f m, %d moving) engine=%.2f ms/step -> %s"
     % (R["verdict"]["rest"], R["verdict"]["revive"], R["verdict"]["motion"],
        R["motion"]["best_m"] or 0.0, R["motion"]["moving_count"],
        R["cost"].get("engine_ms_per_step_median", -1), OUT_PATH))

# End the run NOW: the result is flushed, and run-headless would otherwise
# sleep out the rest of its --duration.
r.simulationQuit(0)
