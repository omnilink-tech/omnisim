"""Run an EXISTING repo world unmodified under the constraint instrument.

Reads behaviour out of the engine's own '[OmNewtonBackend] step N ... bK=(x,y,z)'
telemetry, so no supervisor has to be injected into a world we must not touch.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run as R  # noqa: E402

STEP = re.compile(r"\[OmNewtonBackend\] step (\d+) ")
BODY = re.compile(r"b(\d+)=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--world", required=True)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--env", action="append", default=[])
    ap.add_argument("--stats", type=int, default=10)
    a = ap.parse_args()

    log = os.path.join(R.OUT, a.tag + ".log")
    nlog = os.path.join(R.OUT, a.tag + ".newton.log")
    for p in (log, nlog, log + ".connect_error.txt", log + ".newton.json"):
        if os.path.exists(p):
            os.remove(p)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = R.HOME
    env.pop("WEBOTS_HOME", None)
    env["OMNISIM_LOG_PATH"] = log
    env["OMNISIM_NEWTON_LOG"] = nlog
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    env["OMNISIM_NEWTON_CONSTRAINT_STATS"] = str(a.stats)
    env.update(dict(kv.split("=", 1) for kv in a.env))
    cmd = [R.BIN, os.path.join(R.HOME, a.world), "--batch", "--mode=fast",
           "--no-rendering", "--minimize", "--stdout", "--stderr"]
    t0 = time.time()
    with open(os.path.join(R.OUT, a.tag + ".stdout"), "wb") as so:
        p = subprocess.Popen(cmd, env=env, cwd=R.HOME, stdout=so,
                             stderr=subprocess.STDOUT)
        try:
            rc = p.wait(timeout=a.seconds)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
            rc = -9
    wall = round(time.time() - t0, 1)

    frames = []
    errs = 0
    with open(log, errors="replace") as f:
        for line in f:
            if line.startswith("ERROR"):
                errs += 1
            m = STEP.search(line)
            if not m:
                continue
            pos = {int(i): (float(x), float(y), float(z))
                   for (i, x, y, z) in BODY.findall(line)}
            frames.append((int(m.group(1)), pos))
    moved = {}
    if len(frames) >= 2:
        first, last = frames[0][1], frames[-1][1]
        for k in sorted(set(first) & set(last)):
            dx = last[k][0] - first[k][0]
            dy = last[k][1] - first[k][1]
            moved[k] = round((dx * dx + dy * dy) ** 0.5, 3)
    out = {"tag": a.tag, "world": a.world, "rc": rc, "wall_s": wall,
           "errors": errs, "connect_fails": R.connect_fails(log),
           "steps_logged": frames[-1][0] if frames else 0,
           "newton": R.newton_peaks(nlog),
           "body_disp": moved,
           "log_hits": R.log_greps(log)}
    with open(os.path.join(R.OUT, a.tag + ".world.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: out[k] for k in ("tag", "rc", "wall_s", "errors",
                                          "connect_fails", "steps_logged")}))
    print(json.dumps(out["newton"]))
    print("body_disp:", json.dumps(out["body_disp"]))
    for h in out["log_hits"][:6]:
        print("LOG| " + h)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
