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

"""Does an OmniLink-driven run reproduce? Measure it, with and without the hold.

WHY THIS EXISTS, and why it is not `ab_arms.py`
-----------------------------------------------
The v9 plan's D6 verification says to run `agents/production/husky_maze/scripts/
ab_arms.py --lockstep`. That rig cannot answer the question: it drives
`husky_omnilink_bridge`, a standalone controller that imports no
`omnisim_bridges` at all, so it has none of the sim-time work, none of the step
budgets and no hold. A number from it would describe an untouched code path.

So this drives the bridge that DOES have the work -- the mobile bridge in
`omnilink_husky.omniworld` -- through its own deterministic parser. No model is
involved: every sentence here is one the parser answers, so the decision
sequence is fixed by construction and the only thing that can vary is the
coupling between an HTTP client and a free-running engine. That is precisely the
residual the hold exists to remove:

    measured 2026-09-19, husky_maze, n=5, CPU mujoco (bitwise) solver:
    identical decisions every run, md5 1a1967889876fb66a4d8fea1e6dc1786,
    but 9 of 148 trace lines differed, every one by +/-0.01 rad of yaw.
    "The nondeterminism is in the COUPLING, not the policy."

What is hashed is what the AGENT can see: the measured result of every command
(achieved / error / settled / steps) plus the settled pose after each one. A
hash over wall-clock timings would measure the machine, not the simulator.

Usage:  python determinism_probe.py --trials 3 [--lockstep]
One engine at a time, always reaped. Nothing here touches a world file.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
import pathlib
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

HOME = str(pathlib.Path(__file__).resolve().parents[3])
BIN = os.path.join(HOME, "msys64", "mingw64", "bin", "omnisim-bin.exe")
WORLD = os.path.join(HOME, "projects", "samples", "demos", "worlds", "chat",
                     "omnilink_husky.omniworld")
BASE = "http://127.0.0.1:8765"

# Parser-answerable sentences only. If any of these ever reaches the model the
# probe is measuring something else entirely, so `via` is asserted per command.
SCRIPT = [
    "drive forward 1 metre",
    "turn left 90 degrees",
    "drive forward 1 metre",
    "stop",
]

# Rounding for the pose that enters the hash. The 2026-09-19 residual was one
# unit in the last printed decimal of yaw at 2 dp, so hashing at 2 dp would
# hide exactly the effect being measured. 4 dp keeps it visible.
POSE_DP = 4


def post(path: str, body: dict, timeout: float = 180.0) -> dict:
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def get(path: str, timeout: float = 30.0) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_up(deadline_s: float = 120.0) -> bool:
    """Wait for the bridge to SERVE, not merely to exist.

    ⚠️ Do not probe `/health`: this bridge does not serve it, and the trap is
    that `curl -s ... >/dev/null && ok` reports success on a 404 because curl
    exits 0 for any response it received. urllib raises on a 404 instead, so a
    shell probe and a Python probe disagreed about whether the same live bridge
    was up. `/capabilities` is a route it really serves, and an HTTPError still
    proves something is listening -- which is the actual question here.
    """
    end = time.time() + deadline_s
    while time.time() < end:
        try:
            urllib.request.urlopen(BASE + "/capabilities", timeout=3).read()
            return True
        except urllib.error.HTTPError:
            return True          # answered, just not 200: the server is up
        except Exception:
            time.sleep(1.0)
    return False


# How the CLIENT decides the robot is at rest before sending the next command.
# This is the variable under test, so it is a flag rather than a constant.
#   velocity  every HTTP client's naive rule: both wheel rates near zero.
#             ⚠️ Under a hold a robot reads zero because the WORLD is frozen,
#             not because the motion finished, so this returns before a
#             motion has even started and the next command comes back busy.
#   idle      the bridge's own motion slot is empty (`mode == "idle"`).
#   held      idle AND the world is re-frozen (`held is True`). The only rule
#             that is correct under lockstep: it waits for the hold itself.
PACE = "velocity"


def at_rest(st: dict) -> bool:
    if PACE == "held":
        return st.get("mode") == "idle" and st.get("held") is True
    if PACE == "idle":
        return st.get("mode") == "idle"
    return (abs(float(st.get("v_linear") or 0.0)) < 1e-3
            and abs(float(st.get("v_angular") or 0.0)) < 1e-3)


def settle(max_s: float = 30.0) -> dict:
    """Poll until the base is at rest, so the next command is not refused busy.

    Returns the settled state. This is a WALL-CLOCK poll on purpose: it is the
    client-side pacing whose interaction with a free-running engine is the thing
    under test. Making it sim-paced would erase the effect.
    """
    end = time.time() + max_s
    st = {}
    while time.time() < end:
        st = post("/get_robot_state", {})
        if at_rest(st):
            return st
        time.sleep(0.1)
    return st


def one_trial(lockstep: bool, log_path: str) -> dict:
    env = dict(os.environ)
    env["OMNISIM_HOME"] = HOME
    env["OMNISIM_LOG_PATH"] = log_path
    env["OMNILINK_AGENT_TAG"] = "trackd-determinism"
    if lockstep:
        env["OMNISIM_BRIDGE_LOCKSTEP"] = "1"
    else:
        env.pop("OMNISIM_BRIDGE_LOCKSTEP", None)

    with open(log_path + ".stdout", "wb") as out:
        proc = subprocess.Popen(
            [BIN, "--batch", "--mode=fast", "--no-rendering", "--minimize",
             "--stdout", "--stderr", WORLD],
            stdout=out, stderr=subprocess.STDOUT, env=env,
            creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
    try:
        if not wait_up():
            return {"ok": False, "why": "bridge never answered /health"}
        trace = []
        for sentence in SCRIPT:
            rep = post("/prompt", {"text": sentence})
            via = rep.get("via")
            if via != "parser":
                return {"ok": False,
                        "why": f"{sentence!r} was answered by {via!r}, not the "
                               f"parser -- this probe would be measuring a model"}
            acts = rep.get("actions") or []
            for a in acts:
                trace.append({
                    "tool": a.get("tool"),
                    "result": a.get("result"),
                    "summary": a.get("summary"),
                })
            st = settle()
            trace.append({
                "after": sentence,
                "x": round(float(st.get("x") or 0.0), POSE_DP),
                "y": round(float(st.get("y") or 0.0), POSE_DP),
                "sim_time": round(float(st.get("sim_time") or 0.0), 3),
            })
        ev = get("/events?since=0&limit=50")
        final = post("/get_robot_state", {})
        blob = json.dumps(trace, sort_keys=True)
        return {
            "ok": True,
            "hash": hashlib.md5(blob.encode()).hexdigest(),
            "trace": trace,
            "events_total": ev.get("total"),
            "final": {k: round(float(final.get(k) or 0.0), POSE_DP)
                      for k in ("x", "y")},
            "held": final.get("held"),
            "sim_time": final.get("sim_time"),
        }
    finally:
        try:
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        except Exception:
            pass
        try:
            proc.wait(timeout=8)
        except Exception:
            proc.kill()
            proc.wait(timeout=8)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--lockstep", action="store_true")
    ap.add_argument("--out", default="")
    ap.add_argument("--pace", choices=("velocity", "idle", "held"),
                    default="velocity",
                    help="how the client decides the robot is at rest")
    args = ap.parse_args()
    global PACE
    PACE = args.pace

    here = tempfile.mkdtemp(prefix="trackd_det_")
    runs = []
    for i in range(args.trials):
        log = os.path.join(here, f"det_{'ls' if args.lockstep else 'free'}_{i}.log")
        print(f"[probe] trial {i + 1}/{args.trials} "
              f"lockstep={args.lockstep} ...", flush=True)
        r = one_trial(args.lockstep, log)
        if not r.get("ok"):
            print(f"[probe] ABORT: {r.get('why')}")
            return 2
        print(f"[probe]   hash {r['hash']}  final {r['final']}  "
              f"held={r['held']}  events={r['events_total']}", flush=True)
        runs.append(r)
        time.sleep(2)

    hashes = {r["hash"] for r in runs}
    identical = len(hashes) == 1
    print()
    print(f"=== lockstep={args.lockstep}  pace={args.pace}  trials={len(runs)}")
    print(f"=== distinct trace hashes: {len(hashes)}  -> "
          f"{'IDENTICAL' if identical else 'NOT identical'}")
    if not identical:
        base = runs[0]["trace"]
        for i, r in enumerate(runs[1:], start=1):
            diff = [(j, base[j], r["trace"][j])
                    for j in range(min(len(base), len(r["trace"])))
                    if base[j] != r["trace"][j]]
            print(f"  trial 0 vs {i}: {len(diff)} of {len(base)} trace entries differ")
            for j, a, b in diff[:6]:
                print(f"    [{j}] {json.dumps(a)}")
                print(f"       -> {json.dumps(b)}")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump({"lockstep": args.lockstep, "pace": args.pace, "identical": identical,
                       "hashes": sorted(hashes), "runs": runs}, fh, indent=1)
        print(f"wrote {args.out}")
    return 0 if identical else 1


if __name__ == "__main__":
    sys.exit(main())
