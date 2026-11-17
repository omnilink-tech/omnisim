"""Drive one headless OmniSim run of a generated fleet world and score it.

Run from PowerShell (a Windows python), NOT from MSYS bash: the embedded
interpreter resolves warp/newton differently under an MSYS environment.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

HOME = r"O:\omnisim"
BIN = os.path.join(HOME, "msys64", "mingw64", "bin", "omnisim-bin.exe")
SCR = os.path.join(HOME, "_scratch", "wheelscale")
OUT = os.path.join(SCR, "out")


def gen(tag, n, **kw):
    world = os.path.join(SCR, "worlds", tag + ".wbt")
    cmd = [sys.executable, os.path.join(SCR, "gen_world.py"), "-n", str(n),
           "--out", world]
    for k, v in kw.items():
        if v is None:
            continue
        cmd += ["--" + k.replace("_", "-"), str(v)]
    subprocess.run(cmd, check=True, capture_output=True)
    return world


def run(tag, world, duration_ms=20000, timeout=600, stats=10, env_extra=None):
    log = os.path.join(OUT, tag + ".log")
    nlog = os.path.join(OUT, tag + ".newton.log")
    probe = os.path.join(OUT, tag + ".probe.json")
    for p in (log, nlog, probe, log + ".newton.json"):
        if os.path.exists(p):
            os.remove(p)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = HOME
    env.pop("WEBOTS_HOME", None)
    env["OMNISIM_LOG_PATH"] = log
    env["OMNISIM_NEWTON_LOG"] = nlog
    env["FLEET_PROBE_OUT"] = probe
    env["FLEET_PROBE_DURATION_MS"] = str(duration_ms)
    env["FLEET_PROBE_SAMPLE_MS"] = os.environ.get("FLEET_PROBE_SAMPLE_MS", "500")
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    if stats is not None:
        env["OMNISIM_NEWTON_CONSTRAINT_STATS"] = str(stats)
    if env_extra:
        env.update({k: str(v) for k, v in env_extra.items()})
    cmd = [BIN, world, "--batch", "--mode=fast", "--no-rendering", "--minimize",
           "--stdout", "--stderr"]
    t0 = time.time()
    killed = False
    with open(os.path.join(OUT, tag + ".stdout"), "wb") as so:
        p = subprocess.Popen(cmd, env=env, cwd=HOME, stdout=so,
                             stderr=subprocess.STDOUT)
        try:
            rc = p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
            rc = -9
            killed = True
    return {"tag": tag, "world": world, "rc": rc, "killed": killed,
            "wall_s": round(time.time() - t0, 1), "log": log, "nlog": nlog,
            "probe": probe}


def score(res, expect_speed=0.4128, tol_frac=0.30):
    """Behavioural verdict from the probe trace, not from the log."""
    out = dict(res)
    try:
        with open(res["probe"]) as f:
            blob = json.load(f)
    except Exception as exc:
        out["verdict"] = "NO_PROBE(%r)" % (exc,)
        return out
    out["sim_time_s"] = blob["sim_time_s"]
    rows = []
    for r in blob["robots"]:
        tr = r["trace"]
        x0, y0, z0 = tr[0][1], tr[0][2], tr[0][3]
        x1, y1, z1 = tr[-1][1], tr[-1][2], tr[-1][3]
        dt = tr[-1][0] - tr[0][0]
        dx = x1 - x0
        dy = y1 - y0
        disp = math.hypot(dx, dy)
        speeds = [s[4] for s in tr[2:] if s[4] == s[4]]
        wt = r["wheel_trace"]
        # mean per-wheel |omega| over the second half of the run
        half = wt[len(wt) // 2:]
        wmeans = []
        for wi in range(1, len(wt[0])):
            vals = [s[wi] for s in half if s[wi] == s[wi]]
            wmeans.append(sum(vals) / len(vals) if vals else float("nan"))
        rows.append({
            "name": r["name"], "n_wheels": r["n_wheels"],
            "dx": round(dx, 3), "dy": round(dy, 3), "disp": round(disp, 3),
            "z0": round(z0, 4), "z1": round(z1, 4),
            "zmin": round(min(s[3] for s in tr), 4),
            "zmax": round(max(s[3] for s in tr), 4),
            "mean_speed": round(disp / dt, 4) if dt else 0.0,
            "speed_sd": round(_sd(speeds), 4),
            "wheel_w": [round(w, 3) for w in wmeans],
            "wheel_w_min": round(min(wmeans), 3) if wmeans else float("nan"),
            "nan": any(v != v for s in tr for v in s),
        })
    out["robots"] = rows
    disps = [r["disp"] for r in rows]
    ww = [r["wheel_w_min"] for r in rows]
    out["disp_min"] = round(min(disps), 3)
    out["disp_max"] = round(max(disps), 3)
    out["disp_mean"] = round(sum(disps) / len(disps), 3)
    out["disp_spread"] = round(max(disps) - min(disps), 3)
    out["wheel_w_min"] = round(min(ww), 3)
    out["z_range"] = [round(min(r["zmin"] for r in rows), 4),
                      round(max(r["zmax"] for r in rows), 4)]
    out["any_nan"] = any(r["nan"] for r in rows)
    return out


def _sd(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def connect_fails(log, run_pid=None):
    """How many controllers died on the IPC handshake (5 s deadline)."""
    p = log + ".connect_error.txt"
    names = set()
    try:
        with open(p, errors="replace") as f:
            for line in f:
                if "pipe-" in line or "pipe\\" in line or "\\pipe\\" in line:
                    names.add(line.rsplit("-", 1)[-1].split(")")[0])
    except Exception:
        return 0
    return len(names)


def newton_peaks(nlog):
    peaks, buffers, over = [], [], []
    try:
        with open(nlog) as f:
            for line in f:
                if "constraint peak" in line:
                    peaks.append(line.strip())
                if "constraint buffers" in line:
                    buffers.append(line.strip())
                if "OVERFLOW" in line:
                    over.append(line.strip())
    except Exception:
        pass
    return {"buffers": buffers, "peak_first": peaks[0] if peaks else None,
            "peak_last": peaks[-1] if peaks else None, "n_peaks": len(peaks),
            "overflow_lines": len(over)}


def log_greps(log):
    hits = []
    try:
        with open(log, errors="replace") as f:
            for line in f:
                low = line.lower()
                if ("overflow" in low or "error" in low
                        or "warning" in low and "newton" in low):
                    hits.append(line.strip()[:220])
    except Exception:
        pass
    return hits[:25]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("-n", type=int, required=True)
    ap.add_argument("--solver", default="mujoco")
    ap.add_argument("--njmax", type=int, default=0)
    ap.add_argument("--nconmax", type=int, default=0)
    ap.add_argument("--substeps", type=int, default=0)
    ap.add_argument("--z", type=float, default=0.3)
    ap.add_argument("--spacing", type=float, default=12.0)
    ap.add_argument("--duration-ms", type=int, default=20000)
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--stats", type=int, default=10)
    ap.add_argument("--env", action="append", default=[])
    a = ap.parse_args()
    w = gen(a.tag, a.n, solver=a.solver, njmax=a.njmax, nconmax=a.nconmax,
            substeps=a.substeps, z=a.z, spacing=a.spacing)
    extra = dict(kv.split("=", 1) for kv in a.env)
    r = run(a.tag, w, duration_ms=a.duration_ms, timeout=a.timeout,
            stats=a.stats, env_extra=extra)
    s = score(r)
    s["newton"] = newton_peaks(r["nlog"])
    s["log_hits"] = log_greps(r["log"])
    with open(os.path.join(OUT, a.tag + ".score.json"), "w") as f:
        json.dump(s, f, indent=1)
    brief = {k: s.get(k) for k in ("tag", "rc", "killed", "wall_s", "sim_time_s",
                                   "disp_min", "disp_max", "disp_mean",
                                   "disp_spread", "wheel_w_min", "z_range",
                                   "any_nan")}
    print(json.dumps(brief))
    print(json.dumps(s["newton"]))
    for h in s["log_hits"]:
        print("LOG| " + h)
