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

"""Lane 1R runner: replay measured cube tosses through OmniSim.

One engine launch per toss. The runner owns every frame conversion, so the
controller receives numbers it can apply directly and the conversions live in
one place where they can be tested:

  * position   + TABLE_TOP, because the world's table is lifted off z=0 to
    avoid Newton's implicit ground plane (see the .wbt header).
  * rotation   dataset wxyz quaternion -> the .wbt axis-angle `rotation`.
  * omega      BODY -> WORLD, because that is what setVelocity() expects and
    the dataset stores body frame. Measured, see dataset.OMEGA_FRAME.

Nothing here interprets the result; scoring is score.py's job.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE.parent))        # omnibench/
sys.path.insert(0, str(HERE))               # lane1r/

from common import engine_launch            # noqa: E402
import dataset as D                         # noqa: E402

WORLD = HERE / "worlds" / "cube_toss.wbt"
#: Must match the .wbt: table box centre 0.4, half-thickness 0.1 -> top 0.5.
TABLE_TOP = 0.5
#: 121 samples at 148 Hz.
DURATION_MS = 818.0


def build_ic(traj):
    """-> dict of env-ready strings, plus the numbers for the record."""
    q0, p0, w0, v0 = traj["q0"], traj["p0"], traj["w0"], traj["v0"]
    axis, angle = D.quat_to_axis_angle(q0)
    w_world = D.omega_to_world(q0, w0)
    # |omega| is rotation-invariant; if the rotation changed it, the matrix is
    # not a rotation and everything downstream is quietly wrong.
    if abs(np.linalg.norm(w_world) - np.linalg.norm(w0)) > 1e-9:
        raise RuntimeError("body->world rotation changed |omega|: %.12g -> %.12g"
                           % (np.linalg.norm(w0), np.linalg.norm(w_world)))
    pos = [float(p0[0]), float(p0[1]), float(p0[2]) + TABLE_TOP]
    return {
        "LANE1R_POS": "%.12g %.12g %.12g" % tuple(pos),
        "LANE1R_ROT": "%.12g %.12g %.12g %.12g" % (axis[0], axis[1], axis[2], angle),
        "LANE1R_VEL": "%.12g %.12g %.12g" % tuple(float(x) for x in v0),
        "LANE1R_OMEGA_WORLD": "%.12g %.12g %.12g" % tuple(float(x) for x in w_world),
    }, {"pos_world": pos, "axis_angle": [axis, angle],
        "omega_world": w_world.tolist(), "omega_body": w0.tolist()}


def run_one(index, out_dir, scale="none", calib=None, timeout_s=240,
            step_ms=1, keep_logs=False, contact=None, tag_suffix=""):
    """One toss. `contact` overrides the world's authored contact parameters.

    ⚠ Env beats field in the engine, so passing `contact` genuinely changes
    what is simulated -- that is the point during identification, and exactly
    why the runner otherwise POPS these variables rather than inheriting them.
    A stale export from the parent shell would silently retune the whole lane.
    """
    # ⚠ ABSOLUTE, always. A controller runs with its own directory as cwd, not
    # the repo root, so a relative LANE1R_OUT resolves somewhere else entirely:
    # np.savez then fails, and so do the traceback and breadcrumb writers that
    # exist to report it -- their own `except: pass` swallows the second
    # failure. The symptom is a silent exit 1 with no npz and no diagnostic,
    # which cost a debugging round here.
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = "toss%04d_%s%s" % (index, scale, tag_suffix)
    npz = out_dir / (tag + ".npz")
    log = out_dir / (tag + ".engine.log")
    console = out_dir / (tag + ".console.log")
    for p in (npz, log, Path(str(log) + ".newton.json")):
        if p.exists():
            p.unlink()

    traj = D.load(index, scale=scale, calib=calib)
    envs, ic_dbg = build_ic(traj)

    env = engine_launch.build_env("newton", str(log), repo=REPO)
    env.update(envs)
    env["LANE1R_OUT"] = str(npz)
    env["LANE1R_LABEL"] = tag
    env["LANE1R_DURATION_MS"] = "%.6g" % DURATION_MS
    env["LANE1R_STEP_MS"] = str(step_ms)
    # The world declares newtonGroundMu 0.15, but a stale export from the
    # parent shell would win over the field and silently change the friction
    # the whole lane is measuring.
    env.pop("OMNISIM_NEWTON_GROUND_MU", None)
    env.pop("OMNISIM_NEWTON_CONTACT_KD", None)
    if contact:
        if contact.get("mu") is not None:
            env["OMNISIM_NEWTON_GROUND_MU"] = "%.10g" % contact["mu"]
        if contact.get("ke") is not None:
            env["OMNISIM_NEWTON_CONTACT_KE"] = "%.10g" % contact["ke"]
        if contact.get("kd") is not None:
            env["OMNISIM_NEWTON_CONTACT_KD"] = "%.10g" % contact["kd"]

    binpath = engine_launch.resolve_binary(REPO)
    # RETRY, because a clean exit still burns a port for ~120 s.
    # The engine allocates its TCP port from [1234, 1294] -- 60 slots. A
    # search loop launches ~27 engines/minute at 4 jobs, and Windows holds
    # each client socket in TIME_WAIT long after the engine is gone, so the
    # range saturates from ORDERLY shutdowns, not just the killed-on-timeout
    # case AGENTS.md describes. Measured: 14 sockets still held in that range
    # from a handful of debug runs. When it saturates the engine cannot bind
    # and dies instantly, which read as "every single evaluation failed" and
    # cost an identification campaign.
    attempts = int(os.environ.get("LANE1R_LAUNCH_ATTEMPTS", "3"))
    for attempt in range(1, attempts + 1):
        rc, wall_s, timed_out = engine_launch.launch_once(
            binpath, str(WORLD), env, str(console), timeout_s,
            extra_args=("--stdout", "--stderr"), cwd=str(REPO))
        if npz.exists():
            break
        if attempt < attempts:
            time.sleep(2.0 * attempt)      # let TIME_WAIT drain

    verdict = engine_launch.newton_verdict(str(log))
    rec = {"index": index, "scale": scale, "contact": contact,
           "rc": rc, "timed_out": bool(timed_out),
           "wall_s": round(wall_s, 2), "newton_verdict": verdict,
           "attempts": attempt,
           "npz": str(npz) if npz.exists() else None, "ic": ic_dbg}
    if not npz.exists():
        rec["status"] = "gap"
        rec["error"] = ("no npz: the controller did not finish (rc=%s, "
                        "timed_out=%s). Console: %s" % (rc, timed_out, console))
        return rec
    rec["status"] = "ok"
    if not keep_logs:
        for p in (log, console, Path(str(log) + ".newton.json")):
            if p.exists():
                p.unlink()
    return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description="Replay measured cube tosses.")
    ap.add_argument("--out", required=True)
    ap.add_argument("--indices", help="e.g. 0-19 or 0,5,7 (default: 0-9)")
    ap.add_argument("--scale", choices=("none", "metric"), default="none",
                    help="'none' is comparable to the published baselines; "
                         "'metric' undoes the dataset's 2.2%% length scale")
    ap.add_argument("--step-ms", type=int, default=1)
    ap.add_argument("--timeout", type=int, default=240)
    ap.add_argument("--keep-logs", action="store_true")
    a = ap.parse_args(argv)

    spec = a.indices or "0-9"
    idx = []
    for part in spec.split(","):
        if "-" in part:
            lo, hi = part.split("-")
            idx.extend(range(int(lo), int(hi) + 1))
        else:
            idx.append(int(part))

    calib = D.calibrate() if a.scale == "metric" else None
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = out / "runs.jsonl"
    ok = gaps = 0
    with open(rows, "a", encoding="utf-8") as fh:
        for i in idx:
            r = run_one(i, out, scale=a.scale, calib=calib,
                        timeout_s=a.timeout, step_ms=a.step_ms,
                        keep_logs=a.keep_logs)
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            if r["status"] == "ok":
                ok += 1
                print("[lane1r] toss %-4d ok   (%.1fs)" % (i, r["wall_s"]))
            else:
                gaps += 1
                print("[lane1r] toss %-4d GAP  %s" % (i, r["error"][:110]))
    print("[lane1r] %d ok, %d gaps -> %s" % (ok, gaps, rows))
    return 1 if gaps and not ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
