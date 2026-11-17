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

"""OmniBench lane-1 scorer — pure numpy, engine-agnostic.

Takes a recording .npz (common/recording.py format) + a test id and returns
the SPEC metrics. It knows NOTHING about which engine produced the file;
masses/inertias/analytic references come from common/scenes.py only.

CLI:  python score.py <recording.npz> --test T1
API:  metrics, notes = score("T1", path)
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import scenes  # noqa: E402
from common.recording import load_recording  # noqa: E402

EPS = 1e-12


def quat_to_mat(q):
    """(S,4) wxyz -> (S,3,3) rotation matrices (world <- body)."""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), EPS)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty(q.shape[:1] + (3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def world_inertia(quat, diag):
    """(S,3,3) world-frame inertia R diag(I) R^T from wxyz quats."""
    R = quat_to_mat(quat)
    I = np.diag(np.asarray(diag, dtype=np.float64))
    return np.einsum("sij,jk,slk->sil", R, I, R)


def _idx_at(t, t_target):
    return int(np.argmin(np.abs(t - t_target)))


# ---------------------------------------------------------------------------
# T1 bounce
# ---------------------------------------------------------------------------

def score_t1(t, bodies):
    T1 = scenes.T1
    z = bodies[T1.body]["pos"][:, 2]
    vz = bodies[T1.body]["vel"][:, 2]
    dt = float(np.median(np.diff(t)))

    # Contact detection: in free flight vz decreases by ~g*dt each step; any
    # step where vz INCREASES is (part of) a contact. Robust to soft-contact
    # dips and to penetration depth.
    dvz = np.diff(vz)
    contact = dvz > 0.25 * scenes.GRAVITY * dt
    # group consecutive contact steps; flights are the gaps between groups
    peaks = []
    i = 0
    n = len(contact)
    # skip to the first contact (initial drop)
    while i < n and not contact[i]:
        i += 1
    while i < n and len(peaks) < T1.n_peaks:
        while i < n and contact[i]:       # consume the contact group
            i += 1
        j = i
        while j < n and not contact[j]:   # flight
            j += 1
        if j > i:
            # Height ABOVE THE FLOOR TOP, not above z=0: the contact
            # scenes were lifted off z=0 so the engine's implicit ground
            # plane stops doubling the bottom manifold (validity audit
            # §6). Rest is FLOOR_TOP + r, so that is what an apex is
            # measured from. The analytic reference is unchanged -- the
            # lift is a pure translation.
            apex = float(np.max(z[i:j + 1])) - T1.sphere_r - scenes.FLOOR_TOP
            peaks.append(max(apex, 0.0))
        i = j
    notes = []
    ana = T1.analytic_peaks()
    if len(peaks) < T1.n_peaks:
        notes.append("only %d/%d bounce peaks detected; missing peaks scored "
                     "as h=0" % (len(peaks), T1.n_peaks))
        peaks = peaks + [0.0] * (T1.n_peaks - len(peaks))
    rel = (np.array(peaks) - np.array(ana)) / np.array(ana)
    return {"bounce_height_rmse_rel": float(np.sqrt(np.mean(rel ** 2)))}, \
        notes, {"peaks": peaks, "analytic": ana}


# ---------------------------------------------------------------------------
# T2 incline
# ---------------------------------------------------------------------------

def _tangential(pos, vel, theta_deg):
    d, _ = scenes.incline_frame(theta_deg)
    d = np.asarray(d)
    s = (pos - pos[0]) @ d
    v = vel @ d
    return s, v


def score_t2(t, bodies):
    T2 = scenes.T2
    i_settle = _idx_at(t, T2.settle_t)
    stick_viol = 0.0
    slide_errs = []
    max_stick = None
    min_slip = None
    notes = []
    for name in T2.bodies():
        th = T2.body_angle(name)
        b = bodies[name]
        s, v = _tangential(b["pos"], b["vel"], th)
        disp_3s = abs(s[-1] - s[i_settle])
        moved = disp_3s > T2.slip_classify_m
        if moved:
            min_slip = th if min_slip is None else min(min_slip, th)
        else:
            max_stick = th if max_stick is None else max(max_stick, th)
        if th < T2.theta_c_deg:
            viol = float(np.max(np.abs(s[i_settle:] - s[i_settle])))
            stick_viol = max(stick_viol, viol)
        else:
            i0, i1 = _idx_at(t, T2.fit_t0), _idx_at(t, T2.fit_t1)
            a_meas = float(np.polyfit(t[i0:i1 + 1], v[i0:i1 + 1], 1)[0])
            a_ana = T2.slide_accel(th)
            slide_errs.append(abs(a_meas - a_ana) / a_ana)
    if max_stick is None:       # everything slipped
        notes.append("all angles slipped; transition bracket floored at "
                     "lowest tested angle")
        max_stick = T2.angles_deg[0]
    if min_slip is None:        # nothing slipped
        notes.append("no angle slipped; transition bracket capped at "
                     "highest tested angle")
        min_slip = T2.angles_deg[-1]
    trans = 0.5 * (max_stick + min_slip)
    metrics = {
        "stick_violation_max_m": stick_viol,
        "slide_accel_rel_err": float(np.mean(slide_errs)) if slide_errs else None,
        "transition_angle_err_deg": abs(trans - T2.theta_c_deg),
    }
    notes.append("stick displacement measured from t=%.1fs (skips the "
                 "soft-contact settle); slide accel = linear fit of "
                 "tangential speed over t=[%.1f,%.1f]s"
                 % (T2.settle_t, T2.fit_t0, T2.fit_t1))
    return metrics, notes, {}


# ---------------------------------------------------------------------------
# T3 roll
# ---------------------------------------------------------------------------

def score_t3(t, bodies):
    T3 = scenes.T3
    b = bodies[T3.body]
    s, v = _tangential(b["pos"], b["vel"], T3.theta_deg)
    i0, i1 = _idx_at(t, T3.fit_t0), _idx_at(t, T3.fit_t1)
    a_meas = float(np.polyfit(t[i0:i1 + 1], v[i0:i1 + 1], 1)[0])
    ie = _idx_at(t, T3.slip_eval_t)
    v_t = v[ie]
    omega_y = b["omega"][ie, 1]     # rolling downhill (+x) spins about +y
    slip = abs(v_t - T3.sphere_r * omega_y) / max(abs(v_t), EPS)
    return {"roll_accel_rel_err": abs(a_meas - T3.accel_analytic) / T3.accel_analytic,
            "slip_ratio": float(slip)}, [], {"a_meas": a_meas,
                                             "a_analytic": T3.accel_analytic}


# ---------------------------------------------------------------------------
# T4 pendulum energy
# ---------------------------------------------------------------------------

def _chain_energy(t, bodies):
    """Total mechanical energy per sample; potential relative to z at t=0."""
    T4 = scenes.T4
    E = np.zeros_like(t)
    KE = np.zeros_like(t)
    for name in T4.bodies:
        b = bodies[name]
        m = T4.link_m
        v2 = np.sum(b["vel"] ** 2, axis=1)
        Iw = world_inertia(b["quat"], T4.link_inertia_diag)
        w = b["omega"]
        ke = 0.5 * m * v2 + 0.5 * np.einsum("si,sij,sj->s", w, Iw, w)
        KE += ke
        E += ke + m * scenes.GRAVITY * (b["pos"][:, 2] - b["pos"][0, 2])
    return E, KE


def score_t4(t, bodies):
    T4 = scenes.T4
    E, KE = _chain_energy(t, bodies)
    scale = float(np.max(KE))    # E(0) == 0 for a horizontal release, so
    # normalize by the peak kinetic energy (the physical energy scale of the
    # swing) instead of |E(0)|.
    scale = max(scale, EPS)
    drift = abs(E[-1] - E[0]) / scale
    slope = float(np.polyfit(t, E / scale, 1)[0])   # rel-units per second
    # -- metric v1 blow-up guard --------------------------------------------
    # The analytic energy scale of the swing: from the horizontal release the
    # chain's max possible KE is the PE drop to the hanging pose,
    # sum_i m*g*x_i(0) (~22 J). A mid-window explosion inflates the RECORDED
    # peak KE together with |E(t)| (measured on the dt=32 blow-ups:
    # max|E|/peakKE == 1.0 exactly, while drift/peakKE collapses to ~1e-8 —
    # the "fake 0.0"), so the blow-up ratio must be normalized by the
    # analytic scale, never the recorded one.
    e_scale_ana = T4.link_m * scenes.GRAVITY * float(sum(T4.com_x0))
    blowup = float(np.max(np.abs(E)) / e_scale_ana)
    blew_up = bool(blowup > 10.0)
    notes = ["energy_drift normalized by peak KE (%.3f J): E(0)=0 exactly "
             "for the horizontal release, so |E0| normalization is "
             "undefined" % scale,
             "energy_blowup_max (metric v1) = max|E(t)| / analytic swing "
             "energy (%.2f J); >10 sets energy_blew_up and marks the "
             "peak-KE-normalized drift as meaningless (an exploding run "
             "inflates its own normalizer)" % e_scale_ana]
    if blew_up:
        notes.append("ENERGY BLEW UP mid-window (blowup=%.3g): the "
                     "energy_drift_rel value is an artifact of the inflated "
                     "peak-KE normalizer, not conservation" % blowup)
    return {"energy_drift_rel": float(drift),
            "energy_drift_slope": slope,
            "energy_blowup_max": blowup,
            "energy_blew_up": blew_up}, notes, \
        {"E_scale_J": scale, "E_scale_analytic_J": e_scale_ana}


# ---------------------------------------------------------------------------
# T5 momentum
# ---------------------------------------------------------------------------

def score_t5(t, bodies):
    T5 = scenes.T5
    m = scenes.T4.link_m
    names = list(T5.bodies)
    pos = np.stack([bodies[n]["pos"] for n in names])    # (B,S,3)
    vel = np.stack([bodies[n]["vel"] for n in names])
    P = m * np.sum(vel, axis=0)                          # (S,3)
    M = m * len(names)
    rcom = np.sum(pos, axis=0) * m / M
    vcom = P / M
    L = np.zeros_like(P)
    for n in names:
        b = bodies[n]
        r_rel = b["pos"] - rcom
        v_rel = b["vel"] - vcom
        L += m * np.cross(r_rel, v_rel)
        Iw = world_inertia(b["quat"], T5.link_inertia_diag)
        L += np.einsum("sij,sj->si", Iw, b["omega"])
    lin_max = float(np.max(np.linalg.norm(P, axis=1)))
    i_free = _idx_at(t, T5.drive_t)
    L0 = L[i_free]
    drift_abs = float(np.linalg.norm(L[-1] - L0))
    # v0 metric: normalized by |L| at the end of the actuation window — a
    # small, near-arbitrary residual for a symmetric couple, so the ratio was
    # noisy and non-monotonic across the dt sweep (0.06-8.0 on ODE while its
    # linear momentum sat at 1e-14). Kept for row-to-row continuity.
    drift_v0 = drift_abs / max(np.linalg.norm(L0), EPS)
    # metric v1: normalize by the PEAK |L| reached during the actuation
    # window — the physical angular-momentum scale the drive actually
    # injected. Fallback: if even the peak is tiny (< 1e-6), report the
    # absolute drift in kg*m^2/s instead of a 0/0 ratio.
    L_peak = float(np.max(np.linalg.norm(L[:i_free + 1], axis=1)))
    notes = []
    if L_peak < 1e-6:
        drift = drift_abs
        notes.append("actuation-window peak |L| < 1e-6 kg*m^2/s: "
                     "angular_momentum_drift_rel is the ABSOLUTE drift in "
                     "kg*m^2/s (metric v1 fallback), not a ratio")
    else:
        drift = drift_abs / L_peak
        notes.append("angular_momentum_drift_rel (metric v1) normalized by "
                     "peak |L| during the actuation window (%.4g kg*m^2/s); "
                     "the v0 post-actuation-|L| normalization is kept as "
                     "angular_momentum_drift_rel_v0" % L_peak)
    return {"linear_momentum_max": lin_max,
            "angular_momentum_drift_rel": float(drift),
            "angular_momentum_drift_rel_v0": float(drift_v0),
            "angular_momentum_drift_abs": drift_abs}, \
        notes, {"L_free_norm": float(np.linalg.norm(L0)),
                "L_peak_actuation_norm": L_peak}


# ---------------------------------------------------------------------------
# T6 stack
# ---------------------------------------------------------------------------

def score_t6(t, bodies):
    T6 = scenes.T6
    names = T6.bodies()
    pos = np.stack([bodies[n]["pos"] for n in names])    # (B,S,3) bottom->top
    B = len(names)
    # survivors at the end: within 2 cm of start XY and z-rank preserved
    xy_drift = np.linalg.norm(pos[:, -1, :2] - pos[:, 0, :2], axis=1)
    z_end = pos[:, -1, 2]
    rank = np.argsort(np.argsort(z_end))
    survivors = int(np.sum((xy_drift < T6.survivor_xy_m)
                           & (rank == np.arange(B))))
    # top-box creep over the last creep_window seconds
    i5 = _idx_at(t, t[-1] - T6.creep_window)
    top = pos[-1]
    creep = float(np.linalg.norm(top[-1] - top[i5]) / max(t[-1] - t[i5], EPS))
    # penetration at steady state (last 1 s), from box-centre z spacing of
    # the (nominally axis-aligned) stack + ground
    i_ss = _idx_at(t, t[-1] - 1.0)
    z = pos[:, i_ss:, 2]                                 # (B,W)
    # Ground penetration is measured from the FLOOR TOP, which is no
    # longer z=0 (see scenes.FLOOR_TOP). Without this the whole stack
    # would read as penetrating by FLOOR_TOP.
    pen_ground = np.maximum(0.0, (scenes.FLOOR_TOP + T6.box_half) - z[0])
    pen_pairs = np.maximum(0.0, 2 * T6.box_half - (z[1:] - z[:-1]))
    max_pen = float(max(np.max(pen_ground), np.max(pen_pairs)))
    notes = ["penetration derived from recorded centre spacing assuming "
             "axis-aligned boxes (valid for a surviving stack)"]
    return {"stack_survivors": survivors,
            "settle_creep_m_s": creep,
            "max_penetration_m": max_pen}, notes, {}


# ---------------------------------------------------------------------------
# T7 spin
# ---------------------------------------------------------------------------

def score_t7(t, bodies):
    T7 = scenes.T7
    b = bodies[T7.body]
    Iw = world_inertia(b["quat"], T7.inertia_diag)
    L = np.einsum("sij,sj->si", Iw, b["omega"])
    ke = 0.5 * np.einsum("si,sij,sj->s", b["omega"], Iw, b["omega"])
    L0n = max(np.linalg.norm(L[0]), EPS)
    return {"angmom_drift_rel": float(np.linalg.norm(L[-1] - L[0]) / L0n),
            "rot_ke_drift_rel": float(abs(ke[-1] - ke[0]) / max(ke[0], EPS))}, \
        [], {"L0": float(L0n), "KE0": float(ke[0])}


_SCORERS = {"T1": score_t1, "T2": score_t2, "T3": score_t3, "T4": score_t4,
            "T5": score_t5, "T6": score_t6, "T7": score_t7}


def score(test, npz_path):
    """-> (metrics dict, notes list). Engine-agnostic."""
    test = scenes.normalize_test_id(test)
    t, bodies = load_recording(npz_path)
    missing = [n for n in scenes.BODIES[test] if n not in bodies]
    if missing:
        raise ValueError("recording %s missing bodies %s for %s"
                         % (npz_path, missing, test))
    metrics, notes, extras = _SCORERS[test](t, bodies)
    metrics = {k: v for k, v in metrics.items() if v is not None}
    return metrics, notes, extras


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", help="path to a recording .npz")
    ap.add_argument("--test", required=True, help="T1..T7 or SPEC name")
    args = ap.parse_args(argv)
    metrics, notes, extras = score(args.test, args.recording)
    print(json.dumps({"metrics": metrics, "notes": notes, "extras": extras},
                     indent=2))


if __name__ == "__main__":
    main()
