#!/usr/bin/env python3
"""Terrarium probe director -- the instrument for P1..P6.

This controller does NOT implement a terrarium. It settles the questions whose
answers decide the terrarium architecture, in ONE headless run.

  P1 BATCHED ACTUATION   Does a postponed supervisor FIELD write to
     HingeJointParameters.position actuate a `controller "<none>"` creature?
     SETTLED PASS in probe 0 (0.29 m displacement, 48 hinges at kd=2).

  P2 GLOBAL VELOCITY WIPE   Does teleporting a TOP-LEVEL Solid zero every joint
     velocity in the world (OmSolid.cpp:1192, gated on upperPose() == nullptr)?
     Probe 0 was INCONCLUSIVE: the witness fell 2.67 -> 0.125 rather than to
     exactly 0.0, which is equally consistent with "no wipe" and with "wipe,
     then one step of motor re-acceleration". Probe 1 adds the missing control:
     an identical before/after read at a tick with NO teleport, plus a full
     per-tick velocity trace around every event, so a discontinuity is visible
     rather than inferred from two samples.

  P3 POSE SUPPRESSION   Does Pose{} wrapping suppress that wipe? Only
     meaningful if P2 fires; reported as N/A otherwise.

  P4 COST   Engine ms/step at 60 dynamic bodies, timed AROUND r.step() (probe 0
     timed only the loop body and so measured controller overhead, not physics).
     Both numbers are now reported separately.

  P5 IPC   setJointPosition cost from a synchronization TRUE supervisor.
     Probe 0: 0.3065 ms/call, vs 0.17 ms for 32 batched field writes.

  P6 LOCOMOTION FEASIBILITY   The premise check. Probe 0's creatures oscillated
     in place (displacement rose to 0.52 m then fell to 0.01 m) because every
     creature ran the same symmetric belly-dragging gait. Here each of the 8
     actuated creatures runs a DIFFERENT gait (phase pattern, frequency,
     amplitude, and a fore/aft bias that lifts the torso off its belly).
     If none achieves net displacement, evolution has no gradient to climb and
     the morphology must change before anything else is built.

Results -> projects/alife/_probe_result.json (rewritten every 200 ticks).
"""
import json
import math
import os
import time

from omnisim import Supervisor

TICKS = int(os.environ.get("PROBE_TICKS", "500"))
DT = 8                      # must match WorldInfo.basicTimeStep
ACTUATED = list(range(8))   # group A (0-3 top-level) + group B (4-7 Pose-wrapped)
NLIMB = 4

T_CONTROL_A = 150           # P2 control: identical read, NO teleport
T_TELEPORT_TOPLEVEL = 200   # slot 8  (top-level)
T_CONTROL_B = 300          # P3 control: identical read, NO teleport
T_TELEPORT_POSED = 350     # slot 10 (Pose-wrapped)
T_IPC_BENCH = 420

WITNESS = "CREATURE_0_LIMB_0"   # untouched group-A limb; the wipe detector

# Trace the witness every tick across these windows so a one-tick discontinuity
# is visible in the shape of the signal, not inferred from a single pair.
TRACE = [(145, 160), (195, 210), (295, 310), (345, 360)]

# P6 gait bank. bias lifts the belly: axis is +y (pitch), so a POSITIVE angle
# swings a FRONT limb (+x) downward and a NEGATIVE angle swings a REAR limb
# (-x) downward. Uniform bias sign would splay the creature flat.
def gait(i):
    banks = [
        ("trot",        [0.0, math.pi, math.pi, 0.0],                 1.0, 0.7),
        ("bound",       [0.0, 0.0, math.pi, math.pi],                 1.0, 0.7),
        ("rotary",      [0.0, math.pi / 2, math.pi, 3 * math.pi / 2], 1.0, 0.7),
        ("fast_trot",   [0.0, math.pi, math.pi, 0.0],                 2.2, 0.5),
        ("pronk",       [0.0, 0.0, 0.0, 0.0],                         1.4, 0.7),
        ("pace",        [0.0, math.pi, 0.0, math.pi],                 1.5, 0.6),
        ("slow_walk",   [0.0, math.pi / 2, math.pi / 2, 0.0],         0.6, 0.9),
        ("asym_crawl",  [0.0, 2.4, 1.2, 3.6],                         1.8, 0.8),
    ]
    name, phase, freq, amp = banks[i % len(banks)]
    bias = [0.75, 0.75, -0.75, -0.75]      # front down, rear down
    return name, phase, freq, amp, bias


OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "_probe_result.json")

r = Supervisor()
R = {"p1": {}, "p2": {}, "p3": {}, "p4": {}, "p5": {}, "p6": {},
     "trace": {}, "notes": [], "watchdog": []}


def note(msg):
    print("[probe] %s" % msg, flush=True)
    R["notes"].append(msg)


def dump():
    try:
        with open(os.path.normpath(OUT), "w", encoding="utf-8") as f:
            json.dump(R, f, indent=2)
    except Exception as exc:              # never let bookkeeping kill the run
        print("[probe] dump failed: %s" % exc, flush=True)


# ---------------------------------------------------------------- resolve DEFs
params, roots, missing = {}, {}, []
for i in range(12):
    n = r.getFromDef("CREATURE_%d" % i)
    if n is None:
        missing.append("CREATURE_%d" % i)
    else:
        roots[i] = n
for i in ACTUATED:
    for j in range(NLIMB):
        p = r.getFromDef("C%d_J%d_PARAMS" % (i, j))
        f = p.getField("position") if p is not None else None
        if f is None:
            missing.append("C%d_J%d_PARAMS.position" % (i, j))
        else:
            params[(i, j)] = f

witness = r.getFromDef(WITNESS)
if witness is None:
    missing.append(WITNESS)
if missing:
    note("MISSING DEFs/fields (%d): %s" % (len(missing), missing[:8]))
note("resolved %d position fields, %d creature roots" % (len(params), len(roots)))

GAITS = {i: gait(i) for i in ACTUATED}
R["p6"]["gaits"] = {i: {"name": GAITS[i][0], "freq": GAITS[i][2], "amp": GAITS[i][3]}
                    for i in ACTUATED}

start_pos = {i: list(roots[i].getPosition()) for i in roots}
R["p1"]["start_pos"] = start_pos

engine_ms, loop_ms = [], []
pending = None          # (key, vel_before) -> read the 'after' on the next tick
tick = 0

while True:
    # Time AROUND step(): this is engine step + IPC, i.e. what the world costs.
    ts = time.perf_counter()
    if r.step(DT) == -1:
        break
    engine_ms.append((time.perf_counter() - ts) * 1000.0)

    t0 = time.perf_counter()
    t = tick * (DT / 1000.0)

    # ---------------------------------------------------------------- P1 drive
    # 32 postponed field SETs, drained as ONE batch immediately before the
    # engine global motor push (OmSimulationWorld.cpp:200-203 then :276).
    for (i, j), f in params.items():
        _n, phase, freq, amp, bias = GAITS[i]
        f.setSFFloat(bias[j] + amp * math.sin(2.0 * math.pi * freq * t + phase[j]))

    # ------------------------------------------------ witness velocity tracing
    if witness is not None:
        for lo, hi in TRACE:
            if lo <= tick <= hi:
                v = witness.getVelocity()
                R["trace"].setdefault(str(lo), []).append(
                    [tick, max(abs(x) for x in v)])
                break

    # -------------------------------------------- P2/P3 + controls: read after
    if pending is not None:
        key, before = pending
        after = list(witness.getVelocity()) if witness else None
        mb = max(abs(v) for v in before) if before else None
        ma = max(abs(v) for v in after) if after else None
        R[key] = dict(R.get(key, {}))
        R[key].update({"max_abs_before": mb, "max_abs_after": ma,
                       "ratio": (ma / mb) if (mb and mb > 1e-12) else None,
                       "zeroed": (ma == 0.0)})
        note("%-10s |v| %.6g -> %.6g   ratio=%.4g  zeroed=%s"
             % (key, mb, ma, R[key]["ratio"] or float("nan"), R[key]["zeroed"]))
        pending = None

    # controls: same read pattern, no teleport at all
    if tick == T_CONTROL_A:
        pending = ("p2_control", list(witness.getVelocity()))
    if tick == T_CONTROL_B:
        pending = ("p3_control", list(witness.getVelocity()))

    if tick == T_TELEPORT_TOPLEVEL and 8 in roots:
        before = list(witness.getVelocity())
        roots[8].getField("translation").setSFVec3f([-3.0, 0.0, 0.35])
        note("P2: teleported CREATURE_8 (top-level) crypt -> arena")
        pending = ("p2", before)

    if tick == T_TELEPORT_POSED and 10 in roots:
        before = list(witness.getVelocity())
        roots[10].getField("translation").setSFVec3f([3.0, 0.0, 0.35])
        note("P3: teleported CREATURE_10 (Pose-wrapped) crypt -> arena")
        pending = ("p3", before)

    # ------------------------------------------------------------- P5 IPC cost
    if tick == T_IPC_BENCH:
        j0 = r.getFromDef("C0_J0")
        if j0 is None:
            R["p5"] = {"status": "skipped_no_def"}
            note("P5: SKIP -- DEF C0_J0 not resolvable")
        else:
            try:
                tb = time.perf_counter()
                for k in range(16):
                    j0.setJointPosition(0.3 if k % 2 else -0.3, 1)
                el = (time.perf_counter() - tb) / 16.0 * 1000.0
                R["p5"] = {"status": "ok", "ms_per_call": el, "calls": 16}
                note("P5: setJointPosition %.4f ms/call (sync TRUE supervisor)" % el)
            except Exception as exc:
                R["p5"] = {"status": "error", "error": repr(exc)}
                note("P5: ERROR %r" % exc)

    # ------------------------------------------------------------ P1/P6 telemetry
    if tick % 100 == 0 and tick > 0:
        disp = {i: math.dist(roots[i].getPosition()[:2], start_pos[i][:2])
                for i in roots}
        R["p1"]["disp_t%d" % tick] = disp
        note("t=%4d  " % tick + "  ".join(
            "%d:%s=%.3f" % (i, GAITS[i][0][:4], disp[i]) for i in ACTUATED))
        dump()

    # ------------------------------------------ watchdog: silent NaN / blowup
    # MuJoCo's instability warning channel is read by NOTHING: an exploding or
    # NaN-ing creature emits no engine log line and run-headless still PASSes.
    if tick % 100 == 0:
        for i in roots:
            p = roots[i].getPosition()
            if any(not math.isfinite(v) for v in p) or max(abs(v) for v in p) > 1e4:
                msg = "t=%d CREATURE_%d diverged: %s" % (tick, i, p)
                if msg not in R["watchdog"]:
                    R["watchdog"].append(msg)
                    note("WATCHDOG " + msg)

    loop_ms.append((time.perf_counter() - t0) * 1000.0)
    tick += 1
    if tick >= TICKS:
        break

# ---------------------------------------------------------------- verdicts
end_pos = {i: list(roots[i].getPosition()) for i in roots}
final = {i: math.dist(end_pos[i][:2], start_pos[i][:2]) for i in roots}
R["p1"].update({"end_pos": end_pos, "final_disp": final, "ticks": tick})
R["p1"]["verdict"] = "PASS" if any(final[i] > 0.05 for i in ACTUATED if i in final) else "FAIL"

# P6: rank the gaits. A non-flat spread is what evolution needs to climb.
rank = sorted(((final[i], i, GAITS[i][0]) for i in ACTUATED if i in final), reverse=True)
R["p6"]["ranking"] = [{"slot": i, "gait": g, "disp_m": d} for d, i, g in rank]
R["p6"]["best_m"] = rank[0][0] if rank else None
R["p6"]["worst_m"] = rank[-1][0] if rank else None
R["p6"]["spread"] = (rank[0][0] - rank[-1][0]) if rank else None
R["p6"]["verdict"] = "PASS" if rank and rank[0][0] > 0.5 else "WEAK"

if engine_ms:
    warm = sorted(engine_ms[50:] or engine_ms)
    R["p4"]["engine_ms_per_step_median"] = warm[len(warm) // 2]
    R["p4"]["engine_ms_per_step_mean"] = sum(warm) / len(warm)
if loop_ms:
    warml = sorted(loop_ms[50:] or loop_ms)
    R["p4"]["controller_ms_per_tick_median"] = warml[len(warml) // 2]
R["p4"]["dynamic_bodies"] = 60
R["p4"]["note"] = "engine_ms is timed around r.step() (engine step + IPC)"

# P3 is only meaningful if P2 actually fired.
if not R.get("p2", {}).get("zeroed"):
    R["p3"]["applicability"] = "N/A -- P2 did not zero, so there was no wipe to suppress"

note("P1=%s  P6=%s best=%.3f m (%s)  spread=%.3f  engine=%.3f ms/step"
     % (R["p1"]["verdict"], R["p6"]["verdict"], R["p6"]["best_m"] or 0.0,
        rank[0][2] if rank else "-", R["p6"]["spread"] or 0.0,
        R["p4"].get("engine_ms_per_step_median", 0.0)))
dump()
note("wrote %s" % os.path.normpath(OUT))
