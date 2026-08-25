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

"""Score a lane-1R replay against the measurement it was launched from.

THE METRIC is the one Acosta, Yang & Posa used (RA-L 2022), so the result
lands beside their published baselines instead of needing its own scale:

    position error : ||p_sim - p_real|| averaged over the trajectory,
                     expressed as a PERCENT OF CUBE WIDTH (0.1048 m)
    rotation error : the geodesic angle between the orientations, in degrees,
                     averaged over the trajectory

    Drake   13.5 +/- 8.2 %   16.5 +/- 20.0 deg
    Bullet  14.9 +/- 8.9 %   16.5 +/- 20.2 deg
    MuJoCo  25.1 +/- 10.8%   21.7 +/- 21.4 deg

Two mechanics worth stating because they are where a scorer can flatter
itself:

* **Resampling.** The engine records on a 1 ms grid and the measurement lives
  on a 148 Hz one, so the simulated trajectory is linearly interpolated onto
  the measurement times. Interpolation happens on the SIMULATED side only --
  the measurement is never resampled, so it is never smoothed by us.
  Orientation is interpolated by slerp; lerping quaternions and renormalising
  would bias the angle toward the nearer sample.

* **The IC gate.** A run whose initial condition did not take is not a
  physics measurement, it is a different experiment. The controller records
  what the engine actually accepted, and any run whose achieved velocity or
  angular velocity misses the request by more than `--ic-tol` is REFUSED with
  its numbers shown, not silently averaged in.

  The linear-velocity check has a deliberate floor: the state is read back
  after one 1 ms step, by which point gravity has already changed vz by
  ~0.0098 m/s. On a slow toss that is a percent or two of |v| through no
  fault of the write, so the tolerance is relative with an absolute allowance
  of one step of gravity.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dataset as D                      # noqa: E402

TABLE_TOP = 0.5                          # must match run.py / the .wbt
BASELINES = {
    "Drake":   {"pos_pct": 13.5, "pos_sd": 8.2,  "rot_deg": 16.5, "rot_sd": 20.0},
    "Bullet":  {"pos_pct": 14.9, "pos_sd": 8.9,  "rot_deg": 16.5, "rot_sd": 20.2},
    "MuJoCo":  {"pos_pct": 25.1, "pos_sd": 10.8, "rot_deg": 21.7, "rot_sd": 21.4},
}


def _slerp(q0, q1, u):
    """Shortest-arc slerp between two wxyz quaternions."""
    d = float(np.dot(q0, q1))
    if d < 0.0:                          # q and -q are the same rotation
        q1, d = -q1, -d
    if d > 0.9995:                       # nearly parallel: lerp is exact enough
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(max(-1.0, min(1.0, d)))
    th = th0 * u
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * math.cos(th) + q2 * math.sin(th)


def resample(t_src, pos, quat, t_dst):
    """Linear/slerp interpolation of a simulated trajectory onto t_dst."""
    P = np.empty((len(t_dst), 3))
    Q = np.empty((len(t_dst), 4))
    for i, t in enumerate(t_dst):
        if t <= t_src[0]:
            P[i], Q[i] = pos[0], quat[0]
            continue
        if t >= t_src[-1]:
            P[i], Q[i] = pos[-1], quat[-1]
            continue
        k = int(np.searchsorted(t_src, t) - 1)
        k = max(0, min(k, len(t_src) - 2))
        span = t_src[k + 1] - t_src[k]
        u = 0.0 if span <= 0 else (t - t_src[k]) / span
        P[i] = pos[k] + u * (pos[k + 1] - pos[k])
        Q[i] = _slerp(quat[k], quat[k + 1], u)
    return P, Q


def quat_angle_deg(a, b):
    """Geodesic angle between wxyz quaternions, in degrees (sign-invariant)."""
    d = np.abs(np.sum(a * b, axis=-1)).clip(0.0, 1.0)
    return np.degrees(2.0 * np.arccos(d))


def score_one(npz_path, index, scale="none", calib=None, ic_tol=0.05):
    z = np.load(str(npz_path), allow_pickle=True)
    ic = json.loads(str(z["ic"]))

    # --- IC gate: was this even the right experiment? ---------------------
    wv = np.array(ic["want_vel"]); gv = np.array(ic["got_vel"])
    ww = np.array(ic["want_omega_world"]); gw = np.array(ic["got_omega_world"])
    grav_step = 9.81 * (float(z["step_ms"]) / 1000.0)      # ~0.0098 m/s
    vel_ok = np.linalg.norm(gv - wv) <= ic_tol * np.linalg.norm(wv) + grav_step
    om_ok = np.linalg.norm(gw - ww) <= ic_tol * np.linalg.norm(ww) + 1e-6
    if not (vel_ok and om_ok):
        return {"index": index, "status": "ic_refused",
                "vel_err": float(np.linalg.norm(gv - wv)),
                "omega_err": float(np.linalg.norm(gw - ww)),
                "note": "initial condition did not take; this is not a physics "
                        "measurement of the intended toss"}

    ref = D.load(index, scale=scale, calib=calib)
    t_ref = ref["t"]
    n = int(z["n_samples"])
    t_sim = np.asarray(z["t"])[:n]
    p_sim = np.asarray(z["pos"])[:n].copy()
    q_sim = np.asarray(z["quat"])[:n]
    p_sim[:, 2] -= TABLE_TOP                       # undo the table lift

    P, Q = resample(t_sim, p_sim, q_sim, t_ref)

    dp = np.linalg.norm(P - ref["pos"], axis=1)
    pos_pct = 100.0 * dp / D.CUBE_EDGE_M
    rot_deg = quat_angle_deg(Q, ref["quat"])

    return {
        "index": index, "status": "ok", "scale": scale,
        "pos_pct_mean": float(pos_pct.mean()),
        "pos_pct_final": float(pos_pct[-1]),
        "rot_deg_mean": float(rot_deg.mean()),
        "rot_deg_final": float(rot_deg[-1]),
        "pos_m_mean": float(dp.mean()),
        "n_compared": int(len(t_ref)),
        "ic_vel_rel": float(ic["vel_rel"]), "ic_omega_rel": float(ic["omega_rel"]),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="Score lane-1R replays.")
    ap.add_argument("--runs", required=True, help="dir holding toss*.npz")
    ap.add_argument("--scale", choices=("none", "metric"), default="none")
    ap.add_argument("--ic-tol", type=float, default=0.05)
    ap.add_argument("--json", help="write per-toss rows here")
    a = ap.parse_args(argv)

    runs = sorted(Path(a.runs).glob("toss*_%s.npz" % a.scale))
    if not runs:
        print("no toss*_%s.npz under %s" % (a.scale, a.runs))
        return 1
    calib = D.calibrate() if a.scale == "metric" else None

    rows = []
    for p in runs:
        idx = int(p.stem.split("_")[0].replace("toss", ""))
        rows.append(score_one(p, idx, scale=a.scale, calib=calib,
                              ic_tol=a.ic_tol))

    ok = [r for r in rows if r["status"] == "ok"]
    refused = [r for r in rows if r["status"] != "ok"]
    print("lane 1R -- OmniSim vs %d measured cube tosses (scale=%s)"
          % (len(ok), a.scale))
    if refused:
        print("  REFUSED %d run(s) whose IC did not take:" % len(refused))
        for r in refused[:5]:
            print("    toss %-4d vel_err=%.4g omega_err=%.4g"
                  % (r["index"], r["vel_err"], r["omega_err"]))
    if not ok:
        return 1

    pp = np.array([r["pos_pct_mean"] for r in ok])
    rr = np.array([r["rot_deg_mean"] for r in ok])
    print()
    print("  %-24s %-22s %s" % ("simulator", "position err (% width)",
                                "rotation err (deg)"))
    print("  %-24s %-22s %s"
          % ("OmniSim / Newton", "%.1f +/- %.1f" % (pp.mean(), pp.std()),
             "%.1f +/- %.1f" % (rr.mean(), rr.std())))
    for k, v in BASELINES.items():
        print("  %-24s %-22s %s"
              % ("%s (published)" % k, "%.1f +/- %.1f" % (v["pos_pct"], v["pos_sd"]),
                 "%.1f +/- %.1f" % (v["rot_deg"], v["rot_sd"])))
    print()
    print("  median position err %.1f%%   median rotation err %.1f deg"
          % (np.median(pp), np.median(rr)))
    # ASCII only. A decorative U+26A0 here crashes on a cp1252 console, which
    # is exactly how run_all.py's --help once died; the lesson did not need
    # relearning and this comment is here so it does not get relearnt again.
    print("  NOTE: the baselines are quoted over all 550 tosses; this run "
          "covers %d. They are also NOT scale-corrected, so compare them only "
          "to --scale none." % len(ok))

    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=2), encoding="utf-8")
        print("[score] wrote %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
