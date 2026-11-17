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

"""Metazoa P1 gate -- the cell probe (DESIGN.md "P1 gate"). Owner: A.

One supervisor, no per-cell process. It resolves every cell by DEF, docks by
writing the active face's `isLocked` SFBool (Connector.isLocked -> lock();
the engine welds only if a partner face is within tolerance), drives every
hinge through batched `CELL_<i>_HINGE_PARAMS.position` field writes, and
measures with getPosition()/getOrientation() only (never getPose, never
setVelocity). DT comes from getBasicTimeStep().

Probe world (12 cells, --probe, the default):
  0  sign witness (cell 11): +0.6 rad on the hinge, read where the nose went
     in the tail's frame -> the hinge sign convention for README.md.
  1  chain of 4 (cells 0-3, rotations [0,0,0,0]): lock the three junctions
     at t = 1 s, drive the travelling wave A 0.8 / omega 5 / dphi 1.57 for
     20 s. Reports max junction separation (weld holds <=> <= 0.02 m),
     centroid speed (gate > 0.15 m/s), upright at the end.
  2  alternating chain (cells 4-7, rotations [0,1,0,1]): same drive, the yaw
     hinges get bias +0.5 for 10 s then -0.5: curvature sign and magnitude.
  3  negative control (cells 8, 9 with a 0.10 m face gap): lock, flip cell 8,
     cell 9 must not follow (gate < 0.01 m).
  4  lone flip (cell 10): flip_sequence for 20 s; progress along +x
     (gate > 0.3 m), ends upright.
Cost world (24 resting cells, --cost): engine ms/step around step() (gate
<= 8 ms).

The wave and flip formulas are re-implemented here from DESIGN.md (B's
organism.py is written in parallel and is NOT imported).

Result -> --out (default projects/metazoa/_run/probe_p1.json), then
simulationQuit(0) -- load-bearing, run-headless --duration is a wall-clock
sleep otherwise.
"""
import json
import math
import os
import sys
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from mz import cell as C  # noqa: E402


def _arg(name, default=None, cast=str):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return cast(sys.argv[i + 1])
    return default


COST = "--cost" in sys.argv
OUT = _arg("--out", os.path.join(ROOT, "_run", "probe_p1_cost.json" if COST else "probe_p1.json"))
A_AMP = _arg("--A", 0.8, float)
OMEGA = _arg("--omega", 5.0, float)
DPHI = _arg("--dphi", 1.57, float)
STEER = _arg("--steer", 0.5, float)
FADE_S = _arg("--fade", 1.0, float)
FLIP_SIGN = _arg("--flip-sign", "auto")          # auto | + | -
FLIP_PERIOD = _arg("--flip-period", 2.0, float)
FLIP_FOLD = _arg("--flip-fold", 2.4, float)
N_CELLS = _arg("--cells", 24 if COST else 12, int)
COST_TICKS = _arg("--cost-ticks", 625, int)
DRIVE_S = _arg("--drive", 20.0, float)
NO_LOCK = "--no-lock" in sys.argv          # control arm: never write isLocked
NO_FLIP = "--no-flip" in sys.argv          # P1b: lone cells are inert by decision

CHAIN_A = [0, 1, 2, 3]
CHAIN_B = [4, 5, 6, 7]
CHAIN_B_ROT = [0, 1, 0, 1]
NEG_A, NEG_B = 8, 9
LONE = 10
WITNESS = 11

T_SIGN0, T_SIGN1 = 0.3, 1.2
T_LOCK = 1.0
T_DRIVE0 = 1.5
T_FLIP0 = 1.5

r = Supervisor()
dt = int(r.getBasicTimeStep())
DT = dt / 1000.0
R = {"mode": "cost" if COST else "probe", "dt_ms": dt, "args": sys.argv[1:],
     "params": {"A": A_AMP, "omega": OMEGA, "dphi": DPHI, "steer": STEER, "fade_s": FADE_S,
                "flip_sign": FLIP_SIGN, "flip_period": FLIP_PERIOD, "flip_fold": FLIP_FOLD},
     "notes": [], "watchdog": [], "missing": []}


def note(msg):
    print("[probe_p1] %s" % msg, flush=True)
    R["notes"].append(msg)


def dump():
    try:
        os.makedirs(os.path.dirname(os.path.normpath(OUT)), exist_ok=True)
        with open(os.path.normpath(OUT), "w", encoding="utf-8") as f:
            json.dump(R, f, indent=1)
    except Exception as exc:                       # never let bookkeeping kill the run
        print("[probe_p1] dump failed: %s" % exc, flush=True)


def finish(code=0):
    dump()
    note("wrote %s" % os.path.normpath(OUT))
    r.simulationQuit(code)
    r.step(dt)
    sys.exit(0)


# ---------------------------------------------------------------- resolve
class Cell:
    def __init__(self, i):
        d = C.cell_defs(i)
        self.i = i
        self.robot = r.getFromDef(d["robot"])
        self.nose = r.getFromDef(d["nose"])
        hp = r.getFromDef(d["hinge_params"])
        self.pos_field = hp.getField("position") if hp is not None else None
        self.faces = {f: r.getFromDef(n) for f, n in d["faces"].items()}
        self.lock_fields = {f: (n.getField("isLocked") if n is not None else None)
                            for f, n in self.faces.items()}
        for k, v in (("robot", self.robot), ("nose", self.nose), ("hinge_params.position", self.pos_field)):
            if v is None:
                R["missing"].append("%s.%s" % (d["robot"], k))
        for f, n in self.faces.items():
            if n is None or self.lock_fields[f] is None:
                R["missing"].append(d["faces"][f])

    def pos(self):
        return list(self.robot.getPosition())

    def nose_pos(self):
        return list(self.nose.getPosition())

    def rot(self):
        o = self.robot.getOrientation()
        return [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]

    def yaw(self):
        o = self.robot.getOrientation()
        return math.atan2(o[3], o[0])

    def up(self):
        """World-z component of the tail block's local +z: +1 upright, -1 inverted."""
        return self.robot.getOrientation()[8]

    def face_pos(self, f):
        return list(self.faces[f].getPosition())

    def set_hinge(self, target):
        if self.pos_field is not None:
            self.pos_field.setSFFloat(float(target))

    def hinge_field(self):
        return float(self.pos_field.getSFFloat()) if self.pos_field is not None else None

    def lock(self, f, state=True):
        fld = self.lock_fields.get(f)
        if fld is not None:
            fld.setSFBool(bool(state))


cells = {i: Cell(i) for i in range(N_CELLS)}
V3 = r.getFromDef(C.def_name(0, "ROLL_TB")) is not None    # cell v3 = belly + side rollers
V2 = V3 or r.getFromDef(C.def_name(0, "ROLL_T")) is not None
BODIES_PER_CELL = 6 if V3 else (4 if V2 else 2)
R["cell_version"] = "v3 (belly + side rollers)" if V3 else ("v2 (belly rollers)" if V2 else "v1")
if R["missing"]:
    note("MISSING (%d): %s" % (len(R["missing"]), R["missing"][:10]))
note("resolved %d cells, dt=%d ms, mode=%s" % (len(cells), dt, R["mode"]))


# ---------------------------------------------------------------- formulas (DESIGN.md)
def wave_target(k, t, bias):
    """hinge_k = bias + A sin(omega t + k dphi), A faded in over FADE_S."""
    f = min(1.0, t / FADE_S) if FADE_S > 0 else 1.0
    return f * (bias + A_AMP * math.sin(OMEGA * t + k * DPHI))


def flip_target(t, sign):
    """M-TRAN somersault: fast fold to +-FLIP_FOLD over 25 % of the period,
    hold 10 %, slow unfold to 0 over 65 %."""
    p = (t % FLIP_PERIOD) / FLIP_PERIOD
    if p < 0.25:
        a = FLIP_FOLD * (p / 0.25)
    elif p < 0.35:
        a = FLIP_FOLD
    else:
        a = FLIP_FOLD * (1.0 - (p - 0.35) / 0.65)
    return sign * a


def dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def centroid(idx):
    ps = [cells[i].pos() for i in idx]
    return [sum(p[k] for p in ps) / len(ps) for k in range(3)]


def junction_sep(i, j):
    """Distance between cell i's nose face origin and cell j's tail face origin."""
    return math.dist(cells[i].face_pos("f_nose"), cells[j].face_pos("f_tail"))


def signed_turn(v0, v1):
    return math.atan2(v0[0] * v1[1] - v0[1] * v1[0], v0[0] * v1[0] + v0[1] * v1[1])


def chain_heading(idx):
    """Mean heading of the cells' +x axes (the hinge wave cancels)."""
    sx = sum(math.cos(cells[i].yaw()) for i in idx)
    sy = sum(math.sin(cells[i].yaw()) for i in idx)
    return math.atan2(sy, sx)


def curvature_of(track):
    """track: [(t, x, y, heading)] at a fixed interval. turn = unwrapped
    heading change over the window; path = net centroid displacement;
    curvature = turn / path (a wobbling centroid at near-zero speed makes
    a per-segment heading estimate meaningless, so it is not used)."""
    if len(track) < 2:
        return {"path_m": 0.0, "turn_rad": None, "curvature_rad_per_m": None}
    turn = 0.0
    for a, b in zip(track, track[1:]):
        d = b[3] - a[3]
        turn += (d + math.pi) % (2.0 * math.pi) - math.pi
    path = dist2(track[0][1:3], track[-1][1:3])
    return {"path_m": path, "turn_rad": turn,
            "heading_start": track[0][3], "heading_end": track[-1][3],
            "curvature_rad_per_m": (turn / path) if path > 0.05 else None,
            "duration_s": track[-1][0] - track[0][0]}


engine_ms = []
tick = 0


def step():
    global tick
    t0 = time.perf_counter()
    rc = r.step(dt)
    engine_ms.append((time.perf_counter() - t0) * 1000.0)
    tick += 1
    if rc == -1:
        note("engine stopped the controller at tick %d" % tick)
        dump()
        sys.exit(0)
    if tick % 100 == 0:
        for i, c in cells.items():
            p = c.pos()
            if any(not math.isfinite(v) for v in p) or max(abs(v) for v in p) > 1e4:
                msg = "t=%.2f CELL_%d diverged: %s" % (tick * DT, i, p)
                if msg not in R["watchdog"]:
                    R["watchdog"].append(msg)
                    note("WATCHDOG " + msg)


def engine_stats(skip=50):
    warm = sorted(engine_ms[skip:] or engine_ms)
    if not warm:
        return {}
    return {"median": warm[len(warm) // 2], "mean": sum(warm) / len(warm),
            "p90": warm[int(0.9 * (len(warm) - 1))], "n": len(warm)}


# ================================================================ COST MODE
if COST:
    for _ in range(int(round(1.0 / DT))):
        step()
    z0 = {i: cells[i].pos()[2] for i in cells}
    engine_ms.clear()
    for _ in range(COST_TICKS):
        step()
    st = engine_stats(skip=0)
    R["cost"] = {"cells": len(cells), "dynamic_bodies": BODIES_PER_CELL * len(cells),
                 "engine_ms_per_step": st, "ticks": COST_TICKS,
                 "rest_z_mean": sum(z0.values()) / len(z0), "rest_z_min": min(z0.values()),
                 "rest_z_max": max(z0.values()),
                 "gate_ms_le_8": bool(st and st["median"] <= 8.0)}
    note("COST: %d cells  median %.3f ms/step  mean %.3f  p90 %.3f  rest z %.4f..%.4f  -> %s"
         % (len(cells), st["median"], st["mean"], st["p90"], R["cost"]["rest_z_min"],
            R["cost"]["rest_z_max"], "PASS" if R["cost"]["gate_ms_le_8"] else "FAIL"))
    finish(0)


# ================================================================ PROBE MODE
start = {i: cells[i].pos() for i in cells}
R["start"] = {"pos": start}
sign = {"commanded": 0.6}
flip_sign = {"+": 1.0, "-": -1.0}.get(FLIP_SIGN, None)
chain_b_roll = [C.chain_placement((0, 0, 0, 0, 0), k, 0.01, CHAIN_B_ROT)["roll"] for k in range(4)]
chain_b_kind = [C.hinge_kind(rl) for rl in chain_b_roll]
R["chain_b_axes"] = chain_b_kind

sepA0 = [junction_sep(CHAIN_A[k], CHAIN_A[k + 1]) for k in range(3)]
sepB0 = [junction_sep(CHAIN_B[k], CHAIN_B[k + 1]) for k in range(3)]
sepN0 = junction_sep(NEG_A, NEG_B)
note("authored junction separations: A %s  B %s  neg %.4f"
     % (["%.4f" % s for s in sepA0], ["%.4f" % s for s in sepB0], sepN0))

max_sepA, max_sepB = [0.0] * 3, [0.0] * 3
sep_traceA, sep_traceB = [], []
trackA, trackB, trackN, trackL = [], [], [], []
neg_start = cells[NEG_B].pos()
lone_start = cells[LONE].pos()
lone_x0 = C.mat_vec(cells[LONE].rot(), [1.0, 0.0, 0.0])
locked_at = None
drive_end = T_DRIVE0 + DRIVE_S
T_END = max(drive_end, T_FLIP0 + DRIVE_S) + 0.5
sample_every = max(1, int(round(0.2 / DT)))
lone_up_trace = []

while True:
    t = tick * DT
    # ---- 0 sign witness
    if T_SIGN0 <= t < T_SIGN1:
        cells[WITNESS].set_hinge(sign["commanded"])
    elif t >= T_SIGN1 and "nose_local" not in sign:
        c = cells[WITNESS]
        d = [a - b for a, b in zip(c.nose_pos(), c.pos())]
        local = C.mat_vec(C.transpose(c.rot()), d)
        sign["nose_local"] = local
        sign["hinge_field_readback"] = c.hinge_field()
        sign["tail_up"] = c.up()
        sign["nose_z_world_minus_tail"] = d[2]
        if local[2] > 0.005:
            sign["convention"] = "positive LIFTS the nose (toward tail-frame +z)"
            nose_down = -1.0
        elif local[2] < -0.005:
            sign["convention"] = "positive DROPS the nose (toward tail-frame -z)"
            nose_down = 1.0
        else:
            sign["convention"] = "UNDETERMINED (nose did not leave the seam plane)"
            nose_down = 1.0
        if flip_sign is None:
            flip_sign = nose_down
        sign["flip_sign_used"] = flip_sign
        R["sign"] = sign
        note("SIGN: commanded +%.2f -> nose in tail frame %s; field readback %s; %s; flip sign %+.0f"
             % (sign["commanded"], ["%.4f" % v for v in local], sign["hinge_field_readback"],
                sign["convention"], flip_sign))
        cells[WITNESS].set_hinge(0.0)

    # ---- 1/2/3 lock the junctions (active side = the earlier cell's nose face)
    if locked_at is None and t >= T_LOCK and NO_LOCK:
        locked_at = t
        note("t=%.2f CONTROL ARM: isLocked NOT written" % t)
    if locked_at is None and t >= T_LOCK:
        for k in range(3):
            cells[CHAIN_A[k]].lock("f_nose", True)
            cells[CHAIN_B[k]].lock("f_nose", True)
        cells[NEG_A].lock("f_nose", True)
        locked_at = t
        note("t=%.2f isLocked TRUE written on 7 nose faces" % t)

    # ---- 1 chain A: straight travelling wave
    if T_DRIVE0 <= t < drive_end:
        td = t - T_DRIVE0
        for k, i in enumerate(CHAIN_A):
            cells[i].set_hinge(wave_target(k, td, 0.0))
        # ---- 2 chain B: alternating, yaw hinges biased by the steer
        steer = STEER if td < DRIVE_S / 2.0 else -STEER
        for k, i in enumerate(CHAIN_B):
            bias = steer if chain_b_kind[k] == "yaw" else 0.0
            cells[i].set_hinge(wave_target(k, td, bias))
    elif t >= drive_end:
        for i in CHAIN_A + CHAIN_B:
            cells[i].set_hinge(0.0)

    # ---- 3/4 flips
    if T_FLIP0 <= t < T_FLIP0 + DRIVE_S and flip_sign is not None:
        tf = t - T_FLIP0
        cells[NEG_A].set_hinge(flip_target(tf, flip_sign))
        if not NO_FLIP:
            cells[LONE].set_hinge(flip_target(tf, flip_sign))
    elif t >= T_FLIP0 + DRIVE_S:
        cells[NEG_A].set_hinge(0.0)
        cells[LONE].set_hinge(0.0)

    step()
    t = tick * DT

    # ---- measurements
    if locked_at is not None:
        for k in range(3):
            max_sepA[k] = max(max_sepA[k], junction_sep(CHAIN_A[k], CHAIN_A[k + 1]))
            max_sepB[k] = max(max_sepB[k], junction_sep(CHAIN_B[k], CHAIN_B[k + 1]))
    if tick % sample_every == 0:
        sep_traceA.append([round(t, 3)] + [round(junction_sep(CHAIN_A[k], CHAIN_A[k + 1]), 4) for k in range(3)])
        sep_traceB.append([round(t, 3)] + [round(junction_sep(CHAIN_B[k], CHAIN_B[k + 1]), 4) for k in range(3)])
        ca, cb = centroid(CHAIN_A), centroid(CHAIN_B)
        trackA.append((round(t, 3), ca[0], ca[1], chain_heading(CHAIN_A)))
        trackB.append((round(t, 3), cb[0], cb[1], chain_heading(CHAIN_B)))
        pn, pl = cells[NEG_B].pos(), cells[LONE].pos()
        trackN.append((round(t, 3), pn[0], pn[1], junction_sep(NEG_A, NEG_B)))
        trackL.append((round(t, 3), pl[0], pl[1], pl[2], cells[LONE].up()))
    if t >= T_END:
        break

# ---------------------------------------------------------------- verdicts
def window(track, t0, t1):
    return [s for s in track if t0 - 1e-6 <= s[0] <= t1 + 1e-6]


def net_speed(track, t0, t1):
    w = window(track, t0, t1)
    if len(w) < 2:
        return None
    return dist2(w[0][1:3], w[-1][1:3]) / (w[-1][0] - w[0][0])


def best_window_speed(track, t0, t1, win=5.0):
    w = window(track, t0, t1)
    best = 0.0
    for a in w:
        for b in w:
            if b[0] - a[0] >= win - 1e-6 and b[0] - a[0] <= win + 0.25:
                best = max(best, dist2(a[1:3], b[1:3]) / (b[0] - a[0]))
    return best


up_end = {i: cells[i].up() for i in cells}
R["end"] = {"pos": {i: cells[i].pos() for i in cells}, "up": up_end}

# 1 chain A
R["chain_a"] = {
    "cells": CHAIN_A, "rotations": [0, 0, 0, 0], "gap_authored": 0.01,
    "junction_sep_authored": sepA0, "junction_sep_max": max_sepA,
    "junction_sep_end": [junction_sep(CHAIN_A[k], CHAIN_A[k + 1]) for k in range(3)],
    "sep_trace": sep_traceA, "locked": not NO_LOCK,
    "weld_holds": all(s <= 0.02 for s in max_sepA),
    "speed_net_m_s": net_speed(trackA, T_DRIVE0, drive_end),
    "speed_best_5s_m_s": best_window_speed(trackA, T_DRIVE0, drive_end),
    "displacement_m": dist2(centroid(CHAIN_A)[:2], trackA[0][1:3]),
    "upright_end": all(up_end[i] > 0.7 for i in CHAIN_A),
    "up_end": [up_end[i] for i in CHAIN_A],
    "track": trackA,
}
R["chain_a"]["gate_speed_gt_0p15"] = bool((R["chain_a"]["speed_net_m_s"] or 0.0) > 0.15)

# 2 chain B: two steering phases
tb_mid = T_DRIVE0 + DRIVE_S / 2.0
ph1 = curvature_of(window(trackB, T_DRIVE0 + FADE_S, tb_mid))
ph2 = curvature_of(window(trackB, tb_mid + FADE_S, drive_end))
yaw_head = cells[CHAIN_B[0]].yaw()
R["chain_b"] = {
    "cells": CHAIN_B, "rotations": CHAIN_B_ROT, "axes": chain_b_kind,
    "junction_sep_authored": sepB0, "junction_sep_max": max_sepB,
    "junction_sep_end": [junction_sep(CHAIN_B[k], CHAIN_B[k + 1]) for k in range(3)],
    "sep_trace": sep_traceB, "locked": not NO_LOCK,
    "weld_holds": all(s <= 0.02 for s in max_sepB),
    "speed_net_m_s": net_speed(trackB, T_DRIVE0, drive_end),
    "phase_plus": dict(ph1, steer=STEER), "phase_minus": dict(ph2, steer=-STEER),
    "upright_end": all(up_end[i] > 0.7 for i in CHAIN_B),
    "head_yaw_end": yaw_head,
    "track": trackB,
}
k1, k2 = ph1.get("curvature_rad_per_m"), ph2.get("curvature_rad_per_m")
R["chain_b"]["speed_net_phase_plus_m_s"] = net_speed(trackB, T_DRIVE0 + FADE_S, tb_mid)
R["chain_b"]["speed_net_phase_minus_m_s"] = net_speed(trackB, tb_mid + FADE_S, drive_end)
R["chain_b"]["steering_sign_flips"] = (k1 is not None and k2 is not None and k1 * k2 < 0)
R["chain_b"]["curvature_per_unit_steer"] = ((k1 - k2) / (2.0 * STEER)) if (k1 is not None and k2 is not None) else None

# 3 negative control
neg_disp = dist2(cells[NEG_B].pos(), neg_start)
R["negative"] = {
    "cells": [NEG_A, NEG_B], "face_gap_authored": 0.10, "junction_sep_authored": sepN0,
    "junction_sep_min": min(s[3] for s in trackN), "junction_sep_end": junction_sep(NEG_A, NEG_B),
    "cell9_displacement_m": neg_disp, "gate_lt_0p01": neg_disp < 0.01,
    "cell8_displacement_m": dist2(cells[NEG_A].pos(), start[NEG_A]),
    "cell8_up_end": up_end[NEG_A],
}

# 4 lone flip
pl = cells[LONE].pos()
d = [a - b for a, b in zip(pl, lone_start)]
progress = d[0] * lone_x0[0] + d[1] * lone_x0[1]
lateral = -d[0] * lone_x0[1] + d[1] * lone_x0[0]
R["flip"] = {
    "cell": LONE, "sign": flip_sign, "period_s": FLIP_PERIOD, "fold_rad": FLIP_FOLD,
    "progress_along_x_m": progress, "lateral_m": lateral, "gate_gt_0p3": progress > 0.3,
    "up_end": up_end[LONE], "ends_upright": up_end[LONE] > 0.7,
    "up_min": min(s[4] for s in trackL), "z_max": max(s[3] for s in trackL),
    "dropped": NO_FLIP, "track": trackL,
}

# the sign witness after its one 0.6 rad fold: how far a free cell coasts
R.setdefault("sign", {})["witness_displacement_end_m"] = dist2(cells[WITNESS].pos(), start[WITNESS])
R["sign"]["witness_end_pos"] = cells[WITNESS].pos()
R["engine_ms_per_step"] = engine_stats()
R["engine_ms_per_step"]["note"] = ("%d cells (%d dynamic bodies), 7 welds, 10 actuated hinges; timed around step()"
                                   % (len(cells), BODIES_PER_CELL * len(cells)))
R["ticks"] = tick

note("CHAIN A: end sep %s" % ["%.4f" % v for v in R["chain_a"]["junction_sep_end"]])
note("CHAIN B: end sep %s" % ["%.4f" % v for v in R["chain_b"]["junction_sep_end"]])
note("CHAIN A: max sep %s  net %.3f m/s  best5s %.3f  disp %.3f m  upright %s  -> weld %s, speed %s"
     % (["%.4f" % s for s in max_sepA], R["chain_a"]["speed_net_m_s"] or 0.0,
        R["chain_a"]["speed_best_5s_m_s"], R["chain_a"]["displacement_m"],
        R["chain_a"]["upright_end"], "PASS" if R["chain_a"]["weld_holds"] else "FAIL",
        "PASS" if R["chain_a"]["gate_speed_gt_0p15"] else "FAIL"))
note("CHAIN B: axes %s  max sep %s  net %.3f m/s  k(+%.1f)=%s  k(-%.1f)=%s  sign flips %s"
     % (chain_b_kind, ["%.4f" % s for s in max_sepB], R["chain_b"]["speed_net_m_s"] or 0.0,
        STEER, k1, STEER, k2, R["chain_b"]["steering_sign_flips"]))
note("NEGATIVE: sep %.4f -> min %.4f, cell 9 moved %.4f m -> %s"
     % (sepN0, R["negative"]["junction_sep_min"], neg_disp,
        "PASS" if R["negative"]["gate_lt_0p01"] else "FAIL"))
note("FLIP: sign %+.0f progress %.3f m lateral %.3f up_end %.2f -> %s"
     % (flip_sign or 0.0, progress, lateral, up_end[LONE],
        "PASS" if R["flip"]["gate_gt_0p3"] else "FAIL"))
note("ENGINE: %.3f ms/step median" % R["engine_ms_per_step"].get("median", 0.0))
finish(0)
