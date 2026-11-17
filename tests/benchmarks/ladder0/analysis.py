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

"""analysis.py -- raw samples -> physical measurement.  Simulator-neutral.

All three arms (OmniSim, upstream Webots, MuJoCo) emit the SAME sample
document, and this module is the only thing that reduces it.  That is the
mechanism that keeps the ground truth engine-independent: no arm computes its
own verdict, no arm gets to define what "the fall time" means, and a bug in
the reduction is a bug in all three rows at once rather than a silent bias in
one of them.

Sample document (``samples.json``)::

    {"rung": 2, "sim": "omnisim", "dt": 0.004, "steps": 750,
     "sim_time_end": 3.0,
     "t":       [0.0, 0.004, ...],          # simulated seconds
     "box_z":   [1.6, 1.5999, ...],         # rungs 1, 2
     "joint_q": [0.0, 0.008, ...],          # rung 3, radians (MAY be wrapped)
     "body_x":  [...], "body_y": [...],     # rung 4, metres
     "body_z":  [...],                      # rung 4, metres -- ride_dev reads
                                            # this; omitting it makes
                                            # ride_height RED for want of a
                                            # number, not for a bad rover
     "wheel_q": {"fl": [...], "fr": [...],  # rungs 4 and 6, radians
                 "rl": [...], "rr": [...]},  #   (MAY be wrapped)

     "range":   [2.9, 2.9, ...],            # rungs 5, 6 -- METRES, the raw
                                            # geometric range.  Never the
                                            # engine's raw lookup-table value.

     "robots":  {"r0": {"x": [...], "y": [...], "z": [...],   # rung 7
                        "wheel_q": {"fl": [...], ...}}, ...},

     "part_x": [...], "part_y": [...], "part_z": [...],       # rung 8
     "wrist_x": [...], "wrist_y": [...], "wrist_z": [...],    # rung 8

     "wall": {"t_start": ..., "t_first_step": ..., "t_end": ...}}

Joint angles are read as WRAPPED-TOLERANT: the reduction differences
consecutive samples and folds each difference into (-pi, pi] before
accumulating, so it recovers the true travelled angle whether the engine's
position sensor accumulates without bound or wraps.  At the rates this ladder
commands (<= 4 rad/s) and dt = 4 ms the per-step step is <= 0.016 rad, three
orders of magnitude inside the fold, so the recovery is unambiguous.
"""

from __future__ import annotations

import math

try:                                    # package import ...
    from . import rungs
except ImportError:                     # ... or run as a plain script
    import rungs


# --------------------------------------------------------------------------
# small numeric helpers (stdlib only -- the ladder must run on a bare python)
# --------------------------------------------------------------------------

def interp_at(ts, ys, t):
    """Linear interpolation of ``ys`` at time ``t``.  ``None`` if out of range."""
    if not ts or not ys or len(ts) != len(ys):
        return None
    if t < ts[0] or t > ts[-1]:
        return None
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    if ts[hi] == ts[lo]:
        return ys[lo]
    f = (t - ts[lo]) / (ts[hi] - ts[lo])
    return ys[lo] + f * (ys[hi] - ys[lo])


def first_downward_crossing(ts, ys, level):
    """Time at which ``ys`` first falls to ``level``, linearly interpolated.

    ``None`` if it never does.  Sub-sample interpolation matters here: at
    4.4 m/s impact speed one 4 ms sample is 18 mm, so a nearest-sample crossing
    time would carry a 4 ms quantisation that is half the whole tolerance.
    """
    if not ts or len(ts) != len(ys):
        return None
    for i in range(1, len(ys)):
        if ys[i - 1] > level >= ys[i]:
            dy = ys[i] - ys[i - 1]
            if dy == 0:
                return ts[i]
            f = (level - ys[i - 1]) / dy
            return ts[i - 1] + f * (ts[i] - ts[i - 1])
    return None


def unwrap_cumulative(qs):
    """Cumulative travelled angle from a possibly-wrapped angle series."""
    out = [0.0]
    for i in range(1, len(qs)):
        d = qs[i] - qs[i - 1]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d <= -math.pi:
            d += 2.0 * math.pi
        out.append(out[-1] + d)
    return out


def window_rate(ts, cumulative, t0, t1):
    """Mean rate of ``cumulative`` between ``t0`` and ``t1``, or ``None``."""
    a = interp_at(ts, cumulative, t0)
    b = interp_at(ts, cumulative, t1)
    if a is None or b is None or t1 <= t0:
        return None
    return (b - a) / (t1 - t0)


def window_delta(ts, series, t0, t1):
    a = interp_at(ts, series, t0)
    b = interp_at(ts, series, t1)
    if a is None or b is None:
        return None
    return b - a


def tail_window(ts, ys, seconds):
    """The (values) lying within the last ``seconds`` of the record."""
    if not ts:
        return []
    cut = ts[-1] - seconds
    return [y for t, y in zip(ts, ys) if t >= cut]


def window_mean(ts, ys, t0, t1):
    """Mean of ``ys`` over the samples with ``t0 <= t <= t1``, or ``None``."""
    if not ts or not ys or len(ts) != len(ys):
        return None
    vals = [y for t, y in zip(ts, ys) if t0 <= t <= t1]
    return (sum(vals) / len(vals)) if vals else None


def window_ptp(ts, ys, t0, t1):
    """Peak-to-peak of ``ys`` over ``t0 <= t <= t1``, or ``None``."""
    if not ts or not ys or len(ts) != len(ys):
        return None
    vals = [y for t, y in zip(ts, ys) if t0 <= t <= t1]
    return (max(vals) - min(vals)) if vals else None


def since(ts, ys, t0):
    """The values of ``ys`` at samples with ``t >= t0``."""
    if not ts or not ys or len(ts) != len(ys):
        return []
    return [y for t, y in zip(ts, ys) if t >= t0]


def max_speed(ts, xs, ys, zs, span_s):
    """Fastest |dr|/dt over sliding ``span_s`` windows, or ``None``.

    Read over a window rather than between adjacent samples on purpose: at
    dt = 4 ms a 1 mm jitter differences to 0.25 m/s, which is the same order as
    the bound this feeds, while a real ejection moves the body a visible
    distance in the same window.
    """
    n_ok = [s for s in (xs, ys, zs) if s and len(s) == len(ts)]
    if len(ts) < 3 or not n_ok:
        return None
    dt = ts[1] - ts[0]
    n = max(1, int(round(span_s / dt))) if dt > 0 else 1
    worst = None
    for i in range(0, len(ts) - n):
        j = i + n
        span = ts[j] - ts[i]
        if span <= 0:
            continue
        d2 = 0.0
        for s in n_ok:
            d2 += (s[j] - s[i]) ** 2
        v = math.sqrt(d2) / span
        worst = v if worst is None else max(worst, v)
    return worst


def runs_of(samples):
    """The RUNS of a multi-run cell (CONTRACT.md amendment A), by tag.

    A rung owns one generator; a cell is one or more runs of scenes that
    generator produced, from one ``arm.run()`` call.  A single-run document is
    returned as ``{}`` so the ordinary reductions above are untouched.
    """
    out = {}
    for r in (samples.get("runs") or []):
        tag = r.get("tag")
        if tag is not None:
            out[str(tag)] = r
    return out


def _series_pairs(run_a, run_b):
    """Aligned (name, coord, a_series, b_series) over two runs' bodies.

    Only bodies and coordinates BOTH runs carry; the length mismatch is
    reported separately by ``repeat_length`` so it cannot hide here.
    """
    ba = run_a.get("bodies") or {}
    bb = run_b.get("bodies") or {}
    for name in sorted(set(ba) & set(bb)):
        for coord in ("x", "y", "z"):
            sa = ba[name].get(coord) or []
            sb = bb[name].get(coord) or []
            if sa and sb:
                yield name, coord, sa, sb


def _max_abs_diff(run_a, run_b):
    """max |a - b| over every body, coordinate and overlapping sample.

    Returns ``(worst, where)`` or ``(None, None)`` when nothing lines up.
    ``where`` names the body and coordinate, so a red is attributable to a
    body rather than to "the run".
    """
    worst, where = None, None
    for name, coord, sa, sb in _series_pairs(run_a, run_b):
        n = min(len(sa), len(sb))
        for k in range(n):
            d = abs(sa[k] - sb[k])
            if worst is None or d > worst:
                worst, where = d, "%s.%s@%d" % (name, coord, k)
    return worst, where


def _fleet_measure(ts, robots, omega_of, win, tag_order):
    """One fleet's per-robot physics.  Shared by rungs 7-style and rung 11.

    Returns ``(per_robot, worsts)``.  Every quantity is the one rung 4 defines,
    evaluated per robot: distance against ITS OWN command, wheel rate against
    ITS OWN command, lateral excursion, ride height, and the one-sided
    roll-overrun invariant over the whole run.
    """
    t0, t1 = win
    span = t1 - t0
    per = {}
    d_err, w_err, lat, ride, over = [], [], [], [], []
    for i, tag in enumerate(tag_order):
        r = robots.get(tag) or {}
        xs = list(r.get("x") or [])
        ys = list(r.get("y") or [])
        zs = list(r.get("z") or [])
        wq = r.get("wheel_q") or {}
        if not (ts and xs and len(xs) == len(ts)):
            continue
        omega_cmd = omega_of(i)
        expect = rungs.rolling_speed(omega_cmd) * span
        info = {"omega_cmd": omega_cmd, "expect_m": expect}
        dx = window_delta(ts, xs, t0, t1)
        dy = (window_delta(ts, ys, t0, t1)
              if ys and len(ys) == len(ts) else None)
        if dx is not None:
            dist = math.hypot(dx, dy) if dy is not None else abs(dx)
            info["distance_m"] = dist
            if expect:
                d_err.append(abs(dist - expect) / expect)
                info["distance_rel_err"] = (dist - expect) / expect
        if ys and len(ys) == len(ts):
            lat.append(max(abs(y - ys[0]) for y in ys))
            info["lateral_m"] = lat[-1]
        if zs and len(zs) == len(ts):
            dev = [abs(z - rungs.ROBOT_Z)
                   for tv, z in zip(ts, zs) if tv >= rungs.RIDE_SETTLE_S]
            if dev:
                ride.append(max(dev))
                info["ride_dev_m"] = ride[-1]
        cum, rates = {}, []
        for name in rungs.WHEEL_TAGS:
            series = wq.get(name)
            if not series or len(series) != len(ts):
                continue
            c = unwrap_cumulative(series)
            cum[name] = c
            rate = window_rate(ts, c, t0, t1)
            if rate is not None:
                rates.append(rate)
        if rates and omega_cmd:
            mean = sum(rates) / len(rates)
            info["omega_meas"] = mean
            w_err.append(abs(mean - omega_cmd) / omega_cmd)
        if cum:
            o = _max_overrun(ts, xs, ys, cum)
            if o is not None:
                over.append(o)
                info["roll_overrun"] = o
        per[tag] = info
    worsts = {
        "distance_worst": max(d_err) if d_err else None,
        "wheel_omega_worst": max(w_err) if w_err else None,
        "lateral_worst": max(lat) if lat else None,
        "ride_worst": max(ride) if ride else None,
        "roll_overrun_worst": max(over) if over else None,
    }
    return per, worsts


def _min_separation(ts, robots, tag_order):
    """Smallest pairwise planar centre distance over the WHOLE run.

    ``None`` unless EVERY robot of the fleet has a full track: a fleet reported
    with a robot missing must not be scored as a well-separated smaller one.
    """
    tracks = []
    for tag in tag_order:
        r = robots.get(tag) or {}
        xs, ys = list(r.get("x") or []), list(r.get("y") or [])
        if xs and ys and len(xs) == len(ts) and len(ys) == len(ts):
            tracks.append((xs, ys))
    if len(tracks) != len(tag_order) or len(tracks) < 2 or not ts:
        return None
    worst = None
    for k in range(len(ts)):
        for a in range(len(tracks)):
            for b in range(a + 1, len(tracks)):
                d = math.hypot(tracks[a][0][k] - tracks[b][0][k],
                               tracks[a][1][k] - tracks[b][1][k])
                worst = d if worst is None else min(worst, d)
    return worst


# --------------------------------------------------------------------------
# rung 18 -- comparing a replay to a RECORDING.  Pure stdlib on purpose: the
# ladder must run on a bare python, and lane1r's scorer needs numpy.
#
# The metric is lane1r's, which is Acosta, Yang & Posa's (RA-L 2022), so the
# number lands beside their published baselines instead of needing a scale of
# its own.  ``test_ladder_hygiene`` pins this reducer against lane1r's on the
# same inputs, because two implementations of one ruler is exactly how a
# benchmark quietly stops meaning anything.
# --------------------------------------------------------------------------

def _slerp(q0, q1, u):
    """Shortest-arc interpolation between two wxyz quaternions.

    Slerp and not lerp-then-normalise: lerping biases the interpolated
    orientation toward the nearer sample, which on a tumbling cube is a
    systematic error in the direction of whichever engine sampled faster.
    """
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0.0:                                  # q and -q are one rotation
        q1 = [-c for c in q1]
        d = -d
    if d > 0.9995:
        q = [a + u * (b - a) for a, b in zip(q0, q1)]
        n = math.sqrt(sum(c * c for c in q)) or 1.0
        return [c / n for c in q]
    th0 = math.acos(max(-1.0, min(1.0, d)))
    th = th0 * u
    q2 = [b - a * d for a, b in zip(q0, q1)]
    n = math.sqrt(sum(c * c for c in q2)) or 1.0
    q2 = [c / n for c in q2]
    return [a * math.cos(th) + b * math.sin(th) for a, b in zip(q0, q2)]


def _quat_angle_deg(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(max(0.0, min(1.0, d))))


def _resample_pose(t_src, pos, quat, t_dst):
    """Interpolate a SIMULATED trajectory onto the measurement's own times.

    Interpolation happens on the simulated side only.  The measurement is never
    resampled, so it is never smoothed by us -- a scorer that smoothed the
    reference would be grading itself.
    """
    P, Q = [], []
    n = len(t_src)
    for t in t_dst:
        if n == 0:
            return [], []
        if t <= t_src[0]:
            P.append(pos[0]); Q.append(quat[0]); continue
        if t >= t_src[-1]:
            P.append(pos[-1]); Q.append(quat[-1]); continue
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if t_src[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = t_src[hi] - t_src[lo]
        u = 0.0 if span <= 0 else (t - t_src[lo]) / span
        P.append([a + u * (b - a) for a, b in zip(pos[lo], pos[hi])])
        Q.append(_slerp(quat[lo], quat[hi], u))
    return P, Q


def _score_toss(run):
    """One replayed toss against its recording.  ``None`` if unusable."""
    ref = run.get("reference") or {}
    t_ref = list(ref.get("t") or [])
    p_ref = [list(p) for p in (ref.get("pos") or [])]
    q_ref = [list(q) for q in (ref.get("quat") or [])]
    t_sim = list(run.get("t") or [])
    p_sim = [list(p) for p in (run.get("pos") or [])]
    q_sim = [list(q) for q in (run.get("quat") or [])]
    edge = ref.get("cube_edge_m")
    top = ref.get("table_top")
    if not (t_ref and p_ref and q_ref and t_sim and p_sim and q_sim
            and edge and top is not None):
        return None
    if not (len(t_sim) == len(p_sim) == len(q_sim)):
        return None
    # Undo the table lift: the world puts the table top at ``table_top`` so the
    # engine's implicit z = 0 plane cannot double up with it; the recording's
    # table is at z = 0.
    p_sim = [[p[0], p[1], p[2] - top] for p in p_sim]
    P, Q = _resample_pose(t_sim, p_sim, q_sim, t_ref)
    if not P:
        return None
    n = min(len(P), len(p_ref), len(q_ref))
    dp = [math.sqrt(sum((P[k][c] - p_ref[k][c]) ** 2 for c in range(3)))
          for k in range(n)]
    dr = [_quat_angle_deg(Q[k], q_ref[k]) for k in range(n)]
    half = edge / 2.0
    # How far the cube's TOP FACE went below the table's surface.  A compliant
    # contact sinks the CENTRE; only a body that has passed through the table
    # puts its top face under it.
    tunnel = max([max(0.0, -(p[2] + half)) for p in p_sim] or [0.0])
    out = {
        "pos_pct": 100.0 * (sum(dp) / len(dp)) / edge,
        "pos_pct_final": 100.0 * dp[-1] / edge,
        "rot_deg": sum(dr) / len(dr),
        "rot_deg_final": dr[-1],
        "tunnel_m": tunnel,
        "n_compared": n,
    }
    ic = run.get("ic") or {}
    # The engine's own READBACK of what it accepted, against what it was asked
    # for.  One step of gravity is subtracted because the readback happens
    # after one step, by which point gravity has legitimately changed vz.
    rels = []
    for want, got, floor in (("want_vel", "got_vel", ic.get("grav_step") or 0.0),
                             ("want_omega_world", "got_omega_world", 0.0)):
        w, g = ic.get(want), ic.get(got)
        if not (w and g and len(w) == len(g)):
            continue
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(w, g)))
        mag = math.sqrt(sum(a * a for a in w))
        if mag <= 0:
            continue
        rels.append(max(0.0, err - floor) / mag)
    if rels:
        out["ic_rel"] = max(rels)
    return out


def _max_overrun(ts, xs, ys, cum, span_s=0.1, omega_floor=1.0):
    """max over the run of ``v_body/(omega_wheel r) - 1``, clamped at zero.

    Evaluated over sliding ``span_s`` sub-windows so a single noisy sample
    cannot set the maximum, and only where the mean wheel rate exceeds
    ``omega_floor`` -- below that the ratio's denominator is near zero and the
    quantity is meaningless rather than large.

    Returns 0.0 when the body never outruns its wheels, which is what a
    correctly rolling robot (and a slipping one) produces.
    """
    if len(ts) < 3:
        return None
    dt = ts[1] - ts[0]
    n = max(2, int(round(span_s / dt))) if dt > 0 else 2
    worst = 0.0
    seen = False
    for i in range(0, len(ts) - n, max(1, n // 2)):
        j = i + n
        span = ts[j] - ts[i]
        if span <= 0:
            continue
        dx = xs[j] - xs[i]
        dy = ((ys[j] - ys[i]) if ys and len(ys) == len(ts) else 0.0)
        v = math.hypot(dx, dy) / span
        rates = [abs((c[j] - c[i]) / span) for c in cum.values()]
        omega = sum(rates) / len(rates)
        if omega < omega_floor:
            continue
        seen = True
        worst = max(worst, v / (omega * rungs.WHEEL_R) - 1.0)
    return max(0.0, worst) if seen else None


# --------------------------------------------------------------------------
# the reduction
# --------------------------------------------------------------------------

def reduce_samples(samples, exit_code=None):
    """Sample document -> measurement dict consumed by ``rungs.check_rung``.

    Never raises on malformed input: a quantity that cannot be computed comes
    back ``None``, and ``rungs.check_rung`` treats ``None`` as a FAILED check.
    A reduction that threw would abort the row, and an aborted row is
    indistinguishable from a row that was never run.
    """
    rung = int(samples.get("rung", -1))
    ts = list(samples.get("t") or [])
    m = {
        "steps": (float(samples["steps"])
                  if samples.get("steps") is not None else None),
        "sim_time_end": (float(samples["sim_time_end"])
                         if samples.get("sim_time_end") is not None else None),
        "exit_code": (float(exit_code) if exit_code is not None else None),
        "n_samples": len(ts),
    }

    if rung in (1, 2):
        z = list(samples.get("box_z") or [])
        if ts and z and len(ts) == len(z):
            settle = (rungs.RUNG1_SETTLE_WINDOW if rung == 1
                      else rungs.RUNG2_SETTLE_WINDOW)
            tail = tail_window(ts, z, settle)
            if tail:
                m["rest_z"] = sum(tail) / len(tail)
                m["z_drift"] = max(tail) - min(tail)
            m["spawn_z"] = z[0]
            m["min_z"] = min(z)
            m["final_z"] = z[-1]
            if "rest_z" in m:
                # Positive == sank INTO the floor.  Reported always, so the
                # number is visible even when the coarse rest_z check passes.
                m["penetration"] = rungs.REST_Z - m["rest_z"]
                m["penetration_mm"] = m["penetration"] * 1000.0
        if rung == 2 and ts and z:
            t_hi = first_downward_crossing(ts, z, rungs.RUNG2_GATE_HI)
            t_lo = first_downward_crossing(ts, z, rungs.RUNG2_GATE_LO)
            m["t_gate_hi"] = t_hi
            m["t_gate_lo"] = t_lo
            if t_hi is not None and t_lo is not None:
                m["fall_interval"] = t_lo - t_hi
            m["fall_time_abs"] = t_lo

    elif rung == 3:
        q = list(samples.get("joint_q") or [])
        if ts and q and len(ts) == len(q):
            cum = unwrap_cumulative(q)
            m["omega_driven"] = window_rate(ts, cum, *rungs.RUNG3_WIN_A)
            m["omega_zero"] = window_rate(ts, cum, *rungs.RUNG3_WIN_B)
            m["angle_driven"] = window_delta(ts, cum, *rungs.RUNG3_WIN_A)
            m["angle_total"] = cum[-1]

    elif rung == 4:
        xs = list(samples.get("body_x") or [])
        ys = list(samples.get("body_y") or [])
        wq = samples.get("wheel_q") or {}
        t0, t1 = rungs.RUNG4_WIN
        win = t1 - t0
        if ts and xs and len(ts) == len(xs):
            dx = window_delta(ts, xs, t0, t1)
            dy = (window_delta(ts, ys, t0, t1)
                  if ys and len(ys) == len(ts) else None)
            if dx is not None:
                planar = (math.hypot(dx, dy) if dy is not None else abs(dx))
                m["distance"] = planar
                m["v_body"] = planar / win
                m["dx"] = dx
            if dy is not None:
                m["lateral"] = dy
        rates = []
        for name in ("fl", "fr", "rl", "rr"):
            series = wq.get(name)
            if not series or len(series) != len(ts):
                continue
            r = window_rate(ts, unwrap_cumulative(series), t0, t1)
            if r is not None:
                rates.append(r)
        if rates:
            m["wheel_omega"] = sum(rates) / len(rates)
            m["wheel_omega_each"] = rates
            m["wheel_omega_spread"] = max(rates) - min(rates)
        vb = m.get("v_body")
        ww = m.get("wheel_omega")
        if vb is not None and ww not in (None, 0.0):
            m["roll_ratio"] = vb / (ww * rungs.WHEEL_R)

        # --- whole-run invariants (NOT windowed) -----------------------
        # A steady-state window can only see steady state.  These two look at
        # every sample, because the defect that motivated them lived entirely
        # in the first 1.3 s and every windowed check was green.
        zs = list(samples.get("body_z") or [])
        if ts and zs and len(zs) == len(ts):
            dev = [abs(zv - rungs.ROBOT_Z)
                   for tv, zv in zip(ts, zs) if tv >= rungs.RIDE_SETTLE_S]
            if dev:
                m["ride_dev"] = max(dev)
        cum = {}
        for name in rungs.WHEEL_TAGS:
            series = wq.get(name)
            if series and len(series) == len(ts):
                cum[name] = unwrap_cumulative(series)
        if ts and xs and len(xs) == len(ts) and cum:
            m["roll_overrun"] = _max_overrun(ts, xs, ys, cum)

    elif rung == 5:
        xs = list(samples.get("body_x") or [])
        rng = list(samples.get("range") or [])
        if ts and rng and len(rng) == len(ts):
            m["range_static"] = window_mean(ts, rng, *rungs.RUNG5_DWELL_WIN)
            m["range_final"] = window_mean(ts, rng, *rungs.RUNG5_PARK_WIN)
        if ts and xs and len(xs) == len(ts):
            m["sweep_span"] = max(xs) - min(xs)
            m["sweep_end_x"] = xs[-1]
        if (ts and rng and xs and len(rng) == len(ts)
                and len(xs) == len(ts)):
            # The whole-run cross-check: the ray against the pose recorded on
            # the SAME step.  A windowed reduction cannot see a sensor that
            # stops updating between the windows.
            resid = [abs(r - rungs.rung5_expected_range(x
                                                        + rungs.RUNG5_SENSOR_DX))
                     for r, x in zip(rng, xs)]
            m["range_residual"] = max(resid)
            m["range_span"] = max(rng) - min(rng)

    elif rung == 6:
        xs = list(samples.get("body_x") or [])
        rng = list(samples.get("range") or [])
        wq = samples.get("wheel_q") or {}
        # The gap is a POSE measurement, never the sensor's own opinion of it.
        gaps = [rungs.RUNG6_WALL_FACE_X - (x + rungs.RUNG6_SENSOR_DX)
                for x in xs] if (ts and xs and len(xs) == len(ts)) else []
        if gaps:
            m["min_gap"] = min(gaps)
            m["start_gap"] = gaps[0]
            tail = (ts[-1] - rungs.RUNG6_SETTLE_WINDOW, ts[-1])
            m["stop_gap"] = window_mean(ts, gaps, *tail)
            m["stop_creep"] = window_ptp(ts, xs, *tail)
        if ts and rng and len(rng) == len(ts):
            # The trigger is re-derived from the recorded series rather than
            # taken from the driver's own report of when it decided: the
            # driver's threshold is a COMMAND, the crossing is a measurement.
            for t, r in zip(ts, rng):
                if r < rungs.RUNG6_STOP_GAP:
                    m["trigger_reading"] = r
                    m["trigger_t"] = t
                    break
            if gaps:
                m["range_residual"] = max(abs(r - g)
                                          for r, g in zip(rng, gaps))
        rates = []
        if ts:
            tail0 = ts[-1] - rungs.RUNG6_SETTLE_WINDOW
            for name in rungs.WHEEL_TAGS:
                series = wq.get(name)
                if not series or len(series) != len(ts):
                    continue
                r = window_rate(ts, unwrap_cumulative(series), tail0, ts[-1])
                if r is not None:
                    rates.append(r)
        if rates:
            m["wheel_stop"] = max(rates, key=abs)
            m["wheel_stop_each"] = rates

    elif rung == 7:
        robots = samples.get("robots") or {}
        t0, t1 = rungs.RUNG7_WIN
        win = t1 - t0
        d_err, w_err, lat, ride, over = [], [], [], [], []
        per = {}
        for i, tag in enumerate(rungs.RUNG7_TAGS):
            r = robots.get(tag) or {}
            xs = list(r.get("x") or [])
            ys = list(r.get("y") or [])
            zs = list(r.get("z") or [])
            wq = r.get("wheel_q") or {}
            if not (ts and xs and len(xs) == len(ts)):
                continue
            omega_cmd = rungs.RUNG7_OMEGA[i]
            expect = rungs.rolling_speed(omega_cmd) * win
            dx = window_delta(ts, xs, t0, t1)
            dy = (window_delta(ts, ys, t0, t1)
                  if ys and len(ys) == len(ts) else None)
            info = {"omega_cmd": omega_cmd, "expect_m": expect}
            if dx is not None:
                dist = math.hypot(dx, dy) if dy is not None else abs(dx)
                info["distance_m"] = dist
                if expect:
                    d_err.append(abs(dist - expect) / expect)
                    info["distance_rel_err"] = (dist - expect) / expect
            if ys and len(ys) == len(ts):
                lat.append(max(abs(y - ys[0]) for y in ys))
                info["lateral_m"] = lat[-1]
            if zs and len(zs) == len(ts):
                dev = [abs(z - rungs.ROBOT_Z)
                       for tv, z in zip(ts, zs) if tv >= rungs.RIDE_SETTLE_S]
                if dev:
                    ride.append(max(dev))
                    info["ride_dev_m"] = ride[-1]
            cum, rates = {}, []
            for name in rungs.WHEEL_TAGS:
                series = wq.get(name)
                if not series or len(series) != len(ts):
                    continue
                c = unwrap_cumulative(series)
                cum[name] = c
                rate = window_rate(ts, c, t0, t1)
                if rate is not None:
                    rates.append(rate)
            if rates and omega_cmd:
                mean = sum(rates) / len(rates)
                info["omega_meas"] = mean
                w_err.append(abs(mean - omega_cmd) / omega_cmd)
            if cum:
                o = _max_overrun(ts, xs, ys, cum)
                if o is not None:
                    over.append(o)
                    info["roll_overrun"] = o
            per[tag] = info
        if d_err:
            m["distance_worst"] = max(d_err)
        if w_err:
            m["wheel_omega_worst"] = max(w_err)
        if lat:
            m["lateral_worst"] = max(lat)
        if ride:
            m["ride_worst"] = max(ride)
        if over:
            m["roll_overrun_worst"] = max(over)
        m["per_robot"] = per
        m["n_robots_seen"] = len(per)
        # Minimum pairwise separation over every sample.  Requires every
        # robot's series -- a fleet reported with a robot missing must not be
        # scored as a well-separated smaller fleet.
        tracks = []
        for tag in rungs.RUNG7_TAGS:
            r = robots.get(tag) or {}
            xs, ys = list(r.get("x") or []), list(r.get("y") or [])
            if xs and ys and len(xs) == len(ts) and len(ys) == len(ts):
                tracks.append((xs, ys))
        if len(tracks) == rungs.RUNG7_N and ts:
            worst = None
            for k in range(len(ts)):
                for a in range(len(tracks)):
                    for b in range(a + 1, len(tracks)):
                        d = math.hypot(tracks[a][0][k] - tracks[b][0][k],
                                       tracks[a][1][k] - tracks[b][1][k])
                        worst = d if worst is None else min(worst, d)
            m["min_separation"] = worst
            if worst is not None:
                m["min_clearance"] = worst - rungs.RUNG7_ROBOT_WIDTH

    elif rung == 8:
        px = list(samples.get("part_x") or [])
        py = list(samples.get("part_y") or [])
        pz = list(samples.get("part_z") or [])
        wx = list(samples.get("wrist_x") or [])
        wy = list(samples.get("wrist_y") or [])
        wz = list(samples.get("wrist_z") or [])
        ok_part = bool(ts and pz and len(pz) == len(ts))
        if ok_part:
            m["part_rest_z"] = interp_at(ts, pz, rungs.RUNG8_T_SETTLE)
            m["lift_height"] = window_mean(ts, pz, *rungs.RUNG8_HOLD_WIN)
            carry = [z - rungs.RUNG8_PART_EDGE / 2.0 - rungs.RUNG8_TABLE_TOP
                     for z in since(ts, pz, rungs.RUNG8_T_LIFT)]
            if carry:
                m["hold_clearance"] = min(carry)
            m["part_final_z"] = pz[-1]
        if ts and px and len(px) == len(ts):
            m["place_x"] = window_mean(ts, px, *rungs.RUNG8_HOLD_WIN)
            m["part_final_x"] = px[-1]
        if ok_part and px and wx and wz and len(wx) == len(ts) \
                and len(wz) == len(ts) and len(px) == len(ts):
            rel = []
            for k in range(len(ts)):
                dxk = px[k] - wx[k]
                dyk = ((py[k] - wy[k]) if py and wy and len(py) == len(ts)
                       and len(wy) == len(ts) else 0.0)
                dzk = pz[k] - wz[k]
                rel.append(math.sqrt(dxk * dxk + dyk * dyk + dzk * dzk))
            m["carry_rel"] = max(rel)
            m["carry_rel_final"] = rel[-1]
        m["part_speed_max"] = max_speed(ts, px, py, pz, rungs.RUNG8_SPEED_SPAN)

    elif rung == 9:
        runs = runs_of(samples)
        m["runs_seen"] = sorted(runs)
        a, b, c = (runs.get("a"), runs.get("b"), runs.get("c"))
        # PROVENANCE FIRST.  An arm replaying a cached result, or running its
        # replicas inside one interpreter, passes every determinism check ever
        # written.  Distinctness is asserted on the (pid, process start) PAIR
        # because a pid is reused within a session.
        ids = set()
        for r in runs.values():
            if r.get("pid") is not None:
                ids.add((r.get("pid"), r.get("proc_start")))
        m["distinct_processes"] = float(len(ids)) if ids else None
        m["process_ids"] = sorted(str(i) for i in ids)
        if a and b:
            delta, where = _max_abs_diff(a, b)
            m["repeat_delta"] = delta
            m["repeat_delta_where"] = where
            sa = a.get("steps")
            sb = b.get("steps")
            if sa is not None and sb is not None:
                m["repeat_length"] = float(abs(int(sa) - int(sb)))
        if a and c:
            sep, where = _max_abs_diff(a, c)
            m["sensitivity_max_sep"] = sep
            m["sensitivity_where"] = where
            if sep is not None:
                # Reported ALWAYS, judged only against a floor.  The spread
                # between engines is real -- 202x on ODE against ~145,000x on
                # the two MuJoCo-family arms -- and nothing in mechanics says
                # how chaotic a pile ought to be, so the number is published
                # and the check only asks that the scene amplified at all.
                m["amplification"] = sep / rungs.RUNG9_EPS
                m["sensitivity_shortfall"] = max(
                    0.0, rungs.RUNG9_AMPLIFY_MIN - m["amplification"])
        if a:
            ts_a = list(a.get("t") or [])
            drop = ((a.get("bodies") or {}).get("drop") or {}).get("z") or []
            if ts_a and drop and len(ts_a) == len(drop):
                t_hi = first_downward_crossing(ts_a, drop, rungs.RUNG9_GATE_HI)
                t_lo = first_downward_crossing(ts_a, drop, rungs.RUNG9_GATE_LO)
                m["t_gate_hi"], m["t_gate_lo"] = t_hi, t_lo
                if t_hi is not None and t_lo is not None:
                    m["fall_interval"] = t_lo - t_hi
                m["drop_final_z"] = drop[-1]

    elif rung == 11:
        runs = runs_of(samples)
        m["runs_seen"] = sorted(runs)
        worst = {k: None for k in ("distance_worst", "wheel_omega_worst",
                                   "lateral_worst", "ride_worst",
                                   "roll_overrun_worst")}
        per_n, seen, short = {}, 0, None
        solo_x = None
        solo_dev = None
        for n in rungs.RUNG11_N:
            run = runs.get("n%d" % n)
            if not run:
                continue
            ts_n = list(run.get("t") or [])
            robots = run.get("robots") or {}
            tags = tuple("r%02d" % i for i in range(n))
            per, w = _fleet_measure(ts_n, robots, rungs.rung11_omega,
                                    rungs.RUNG11_WIN, tags)
            seen += len(per)
            sep = _min_separation(ts_n, robots, tags)
            if sep is not None:
                s = max(0.0, rungs.RUNG11_LANE_DY - sep)
                short = s if short is None else max(short, s)
            per_n["n%d" % n] = {"n": n, "robots_measured": len(per),
                                "min_separation": sep, "worst": dict(w),
                                "per_robot": per}
            for k in worst:
                v = w.get(k)
                if v is not None:
                    worst[k] = v if worst[k] is None else max(worst[k], v)
            # Robot 0 carries the same command at every N and starts at the
            # same x, so its along-track history is comparable across the
            # sweep.  REPORTED, NEVER JUDGED -- see rungs.py: bit-identity
            # across N is not physically required and a correct engine can
            # miss it in the last ULP.
            r0 = (robots.get("r00") or {}).get("x") or []
            if ts_n and r0 and len(r0) == len(ts_n):
                if n == 1:
                    solo_x = r0
                elif solo_x is not None:
                    k = min(len(solo_x), len(r0))
                    d = max(abs(r0[i] - solo_x[i]) for i in range(k))
                    solo_dev = d if solo_dev is None else max(solo_dev, d)
                    per_n["n%d" % n]["solo_deviation_m"] = d
        m.update(worst)
        m["separation_shortfall"] = short
        m["robots_seen"] = float(seen)
        m["per_n"] = per_n
        m["solo_deviation_max"] = solo_dev

    elif rung == 18:
        runs = samples.get("runs") or []
        requested = list(samples.get("requested") or [])
        per_toss, pos, rot, tunnel, ic = {}, [], [], [], []
        for run in runs:
            idx = (run.get("params") or {}).get("index")
            s = _score_toss(run)
            if idx is None or s is None:
                continue
            per_toss[str(idx)] = s
            pos.append(s["pos_pct"])
            rot.append(s["rot_deg"])
            tunnel.append(s["tunnel_m"])
            if s.get("ic_rel") is not None:
                ic.append(s["ic_rel"])
        if pos:
            m["real_pos_err"] = sum(pos) / len(pos)
            m["real_pos_sd"] = math.sqrt(
                sum((p - m["real_pos_err"]) ** 2 for p in pos) / len(pos))
            m["real_pos_median"] = sorted(pos)[len(pos) // 2]
        if rot:
            m["real_rot_err"] = sum(rot) / len(rot)
            m["real_rot_sd"] = math.sqrt(
                sum((r - m["real_rot_err"]) ** 2 for r in rot) / len(rot))
        if tunnel:
            m["tunnel_depth"] = max(tunnel)
        if ic:
            m["ic_shortfall"] = max(ic)
        m["per_toss"] = per_toss
        m["tosses_scored"] = float(len(per_toss))
        # PROVENANCE.  The subset must be one the CONTRACT owns; an arm free to
        # choose its own tosses could choose the easy ones, and no assertion
        # downstream could see it.  An unrecognised subset leaves
        # ``tosses_missing`` unmeasured, which is judged RED.
        if tuple(requested) in rungs.RUNG18_SUBSETS:
            m["tosses_missing"] = float(len(requested) - len(per_toss))
            m["subset_size"] = float(len(requested))
        m["subset_is_contract_owned"] = tuple(requested) in rungs.RUNG18_SUBSETS

    return m


def wall_timings(samples, proc_t0=None, proc_t1=None):
    """Wall-clock facts for one run, in seconds.

    ``startup_s``  process spawn -> the controller's FIRST completed step.
                   This is the number the two 2026-08-12 perf commits claim to
                   move, and it is the only part of the run that is not
                   physics.
    ``total_s``    process spawn -> process exit.
    ``step_s``     first step -> last step (the simulated part).
    """
    w = samples.get("wall") or {}
    out = {"startup_s": None, "total_s": None, "step_s": None}
    if proc_t0 is not None and w.get("t_first_step") is not None:
        out["startup_s"] = float(w["t_first_step"]) - float(proc_t0)
    if proc_t0 is not None and proc_t1 is not None:
        out["total_s"] = float(proc_t1) - float(proc_t0)
    if w.get("t_first_step") is not None and w.get("t_end") is not None:
        out["step_s"] = float(w["t_end"]) - float(w["t_first_step"])
    return out
