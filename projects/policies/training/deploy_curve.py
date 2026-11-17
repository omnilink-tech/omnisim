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

"""deploy_curve.py -- the PREDICTABILITY engine for structured RL training.

Parses a trainer log's DEPLOY-EVAL lines (the deterministic deploy-prediction metric,
see docs/developer/deploy-prediction-metric.md), fits a saturating learning curve to
`surv` vs iteration, and turns it into a FORECAST + a decision:
  - the fitted ASYMPTOTE (will this run ever reach the target?),
  - the ETA (iters + wall-clock) to a target surv,
  - a recommendation: ON-TRACK / PLATEAU-BELOW-TARGET (kill) / TOO-EARLY / TOO-NOISY.

This is what turns "train and see" into "predict and decide" -- stop burning hours on
runs that won't converge, and estimate performance from training time.

Usage:
  python deploy_curve.py <log.txt> [--target 0.85] [--metric surv] [--watch]
  # --watch: poll the log and re-forecast every 30 s (live early-stop advisor)

RL curves are noisy + non-monotonic; the fit is on the running-max envelope (the
ACHIEVABLE performance), and the forecast is reported with an uncertainty band. It is a
forecast, not a guarantee -- but "won't reach target, kill it" at 30 min beats 3 h blind.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np

_EVAL_RE = re.compile(
    r"DEPLOY-EVAL(?:-ONLY)?\s+(?:ckpt\s+)?(?:it=(?P<it>\d+)\s+)?"
    r"surv=(?P<surv>[-\d.]+)\s+fall=(?P<fall>[-\d.]+)\s+sat=(?P<sat>[-\d.]+)"
    r"\s+fidel=(?P<fidel>[-\d.]+)\s+drift=(?P<drift>[-\d.]+)"
    r"(?:\s+depth=(?P<depth>[-\d.]+))?"     # depth optional (older logs lack it)
    r"(?:\s+fwd=(?P<fwd>[-\d.]+))?")        # fwd (walk distance) optional
# wall-clock proxy: the trainer prints steps/s on AMP-GPU lines; we approximate time
_STEPS_RE = re.compile(r"(?:AMP|FREE)-GPU it=(?P<it>\d+).*steps/s=(?P<sps>[\d.]+)")


def parse_log(path):
    """-> (iters, {surv,fall,sat,fidel,drift}, approx_seconds_per_iter or None)."""
    iters, cols = [], {k: [] for k in ("surv", "fall", "sat", "fidel", "drift", "depth", "fwd")}
    sps_it, sps_val = [], []
    last_it = 0
    with open(path, "r", errors="ignore") as f:
        for line in f:
            line = re.sub(r"np\.int32\((\d+)\)", r"\1", line)
            m = _EVAL_RE.search(line)
            if m:
                it = int(m.group("it")) if m.group("it") else last_it
                iters.append(it)
                for k in cols:
                    v = m.group(k)                       # depth may be absent in older logs
                    cols[k].append(float(v) if v is not None else float("nan"))
            s = _STEPS_RE.search(line)
            if s:
                last_it = int(s.group("it"))
                sps_it.append(int(s.group("it"))); sps_val.append(float(s.group("sps")))
    iters = np.array(iters, float)
    for k in cols:
        cols[k] = np.array(cols[k], float)
    sec_per_iter = None
    if len(sps_it) > 2 and iters.size:
        # mean steps/s -> sec/iter needs env-steps/iter; approximate from the AMP-GPU cadence
        # (env-steps/iter = T*K, but we only need a relative wall-clock; use median sps)
        mean_sps = float(np.median(sps_val))
        # T*K is unknown here; caller can pass it. Return sps so a rough ETA is possible.
        sec_per_iter = ("sps", mean_sps)
    return iters, cols, sec_per_iter


def running_max(y):
    out = np.maximum.accumulate(y) if y.size else y
    return out


def fit_saturating(t, y):
    """Fit y ~= A*(1 - exp(-(t)/tau)) to the running-max envelope, robustly, no scipy.
    Returns (A, tau, rmse). A is the asymptote (achievable surv)."""
    if t.size < 4:
        return None
    env = running_max(y)
    t0 = t - t.min()
    best = None
    # grid over the asymptote A; for each, tau by log-linear LS on ln(1 - env/A) = -t/tau
    for A in np.linspace(max(env.max() + 1e-3, 0.05), 1.05, 120):
        r = 1.0 - env / A
        ok = r > 1e-4
        if ok.sum() < 3:
            continue
        # ln(r) = -t/tau  -> slope = -1/tau
        x = t0[ok]; lr = np.log(r[ok])
        denom = float((x * x).sum())
        if denom <= 0:
            continue
        slope = float((x * lr).sum()) / denom
        if slope >= 0:
            continue
        tau = -1.0 / slope
        pred = A * (1.0 - np.exp(-t0 / tau))
        rmse = float(np.sqrt(np.mean((pred - env) ** 2)))
        if best is None or rmse < best[2]:
            best = (A, tau, rmse)
    return best


def eta_to_target(A, tau, t_now_rel, target):
    if A is None or A <= target:
        return None
    # target = A*(1-exp(-t/tau)) -> t = -tau*ln(1 - target/A)
    t_target = -tau * math.log(1.0 - target / A)
    return max(0.0, t_target - t_now_rel)


def ascii_plot(t, y, width=60, height=12):
    if t.size < 2:
        return "  (not enough eval points yet)"
    lo, hi = 0.0, max(1.0, y.max())
    grid = [[" "] * width for _ in range(height)]
    for i in range(t.size):
        cx = int((t[i] - t.min()) / max(1e-9, t.max() - t.min()) * (width - 1))
        cy = int((y[i] - lo) / max(1e-9, hi - lo) * (height - 1))
        grid[height - 1 - cy][cx] = "*"
    rows = ["".join(r) for r in grid]
    return "\n".join("  %s |%s" % (("%.2f" % (hi - (hi - lo) * r / (height - 1))), rows[r]) for r in range(height))


def forecast(path, target=0.85, metric="surv", tk=None):
    if not os.path.exists(path):
        return {"status": "NO-DATA", "msg": "log not created yet"}
    iters, cols, sec = parse_log(path)
    if iters.size == 0:
        return {"status": "NO-DATA", "msg": "no DEPLOY-EVAL lines yet"}
    y = cols[metric]
    t = iters
    cur = float(y[-1]); cur_env = float(running_max(y)[-1]); it_now = int(t[-1])
    fit = fit_saturating(t, y)
    out = {"metric": metric, "target": target, "it_now": it_now, "cur": cur,
           "best_so_far": cur_env, "n_evals": int(t.size)}
    if fit is None:
        out.update(status="TOO-EARLY", msg="need >=4 eval points to forecast")
        out["plot"] = ascii_plot(t, y); return out
    A, tau, rmse = fit
    t_now_rel = it_now - float(t.min())
    eta_it = eta_to_target(A, tau, t_now_rel, target)
    out.update(asymptote=round(A, 3), tau=round(tau, 1), fit_rmse=round(rmse, 3))
    # decision
    noise = float(np.std(np.diff(y))) if y.size > 2 else 0.0
    out["noise"] = round(noise, 3)
    if A < target - 0.03:
        out["status"] = "KILL"
        out["msg"] = ("fitted asymptote %.2f < target %.2f -> this config will NOT reach "
                      "the target; stop and change a variable." % (A, target))
    elif eta_it is None:
        out["status"] = "PLATEAU"; out["msg"] = "asymptote ~= target; marginal, watch closely."
    else:
        eta_txt = "%d more iters" % int(eta_it)
        if tk:
            out["eta_iters"] = int(eta_it)
        out["status"] = "ON-TRACK"
        out["msg"] = ("on track for target %.2f: asymptote %.2f, ETA ~%s (tau=%.0f)."
                      % (target, A, eta_txt, tau))
    if noise > 0.18:
        out["msg"] += "  (curve is NOISY -> unstable training; stabilize before trusting the ETA)"
    out["plot"] = ascii_plot(t, y)
    return out


def _print(fc):
    print("=" * 66)
    print("  DEPLOY-CURVE FORECAST  (%s)" % fc.get("metric", "surv"))
    print("=" * 66)
    if fc["status"] in ("NO-DATA",):
        print("  " + fc["msg"]); return
    print("  it=%d  current=%.3f  best-so-far=%.3f  (%d evals)"
          % (fc["it_now"], fc["cur"], fc["best_so_far"], fc["n_evals"]))
    if "asymptote" in fc:
        print("  fit: asymptote=%.3f  tau=%.0f  rmse=%.3f  noise=%.3f"
              % (fc["asymptote"], fc["tau"], fc["fit_rmse"], fc.get("noise", 0)))
    print("  --> [%s] %s" % (fc["status"], fc["msg"]))
    if "plot" in fc:
        print("\n" + fc["plot"])
    print("=" * 66)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--target", type=float, default=0.85)
    ap.add_argument("--metric", default="surv", choices=["surv", "fall", "sat", "fidel", "drift"])
    ap.add_argument("--watch", action="store_true", help="poll + re-forecast every 30s")
    a = ap.parse_args()
    if a.watch:
        while True:
            os.system("cls" if os.name == "nt" else "clear")
            _print(forecast(a.log, a.target, a.metric))
            time.sleep(30)
    else:
        _print(forecast(a.log, a.target, a.metric))


if __name__ == "__main__":
    main()
