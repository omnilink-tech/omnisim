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

"""What one robot task actually costs, per architecture, measured.

    python tests/benchmarks/costbench/run.py --arm deterministic
    python tests/benchmarks/costbench/run.py --arm llm --cycles 5

THE TASK
    A Husky drives forward 2 m, turns 90 degrees, and drives back 2 m.
    One cycle, three commands, repeated. Chosen because it is the most
    boring possible robot job: nothing about the 7,000th repetition needs
    judgement that the first did not.

THE ARMS
    deterministic  no key. The parser interprets, the bridge executes.
    llm            OMNI_KEY attached, so every command is a relay turn and
                   the model decides each step.

ENVIRONMENT (the llm arm only; the deterministic arm needs nothing)
    OMNI_KEY          the OmniKey itself. Preferred.
    OMNI_KEY_FILE     a path to a file holding one, when you would rather not
                      export the key. Read only if OMNI_KEY is unset; there is
                      no default path, because this file ships.
    OMNILINK_LIB_SRC  a source checkout of the OmniLink SDK to prepend to the
                      controller's PYTHONPATH. Only needed when the published
                      `omnilink` package is not installed in the controller's
                      interpreter.

WHAT IS MEASURED, NOT ASSUMED
    tokens per command   from the bridge's /usage, which reports the
                         platform's own rollup per turn -- the same meter
                         husky_maze/docs/COSTS.md used, not a token count
                         guessed from response text.
    seconds per cycle    wall clock, so the 24-hour extrapolation rests on
                         this robot's real cadence rather than on a guess.
                         An earlier projection assumed 12 s and that
                         assumption carried the whole number.

⚠️ WHAT IS STILL EXTRAPOLATED. Cost for 24 hours is measured cost per cycle
multiplied out. It assumes the cadence holds and that context length stays
flat, which compaction makes roughly true and nothing here proves. The
per-cycle figure is the measurement; the daily figure is arithmetic on it.

⚠️ THE MIXED ARM IS A SUM OF TWO HALVES, NOT A SHIPPED FEATURE. "Say it
once, let the tree run it" costs one interpreted turn plus a free loop.
Both halves are measured here, but OmniLink does not yet wire one
instruction to a repeating deterministic loop, and this script does not
pretend otherwise.

PRICES are list, per COSTS.md (May 2026). Caching lowers them ~20%.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import subprocess
import sys
import time
import urllib.error
import urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # pragma: no cover
    pass

ROOT = pathlib.Path(__file__).resolve().parents[3]
WORLD = "projects/samples/demos/worlds/chat/omnilink_husky.omniworld"
PORT = 8765

# Where the llm arm's OmniKey comes from. In order: OMNI_KEY in the
# environment, or the file OMNI_KEY_FILE points at. Nothing is defaulted to a
# path on anyone's disk -- this file ships, and an operator's key store is not
# a public fact about the benchmark. `python -m omnisim key` explains how to
# set OMNI_KEY; a key VALUE is never written into this tree either way.
KEY_FILE_ENV = "OMNI_KEY_FILE"

CYCLE = ["drive forward 2 metres", "turn left 90 degrees",
         "drive forward 2 metres"]

# $ per million units. COSTS.md, list prices, May 2026.
PRICES = {
    "g1-engine": ("Gemini 3 Flash", 0.50, 3.00),
    "g2-engine": ("GPT", 1.00, 5.00),
    "g3-engine": ("Grok", 1.00, 5.00),
    "g4-engine": ("Claude", 3.00, 15.00),
    "g5-engine": ("Ollama (self-hosted)", 0.0, 0.0),
}


def _get(path, timeout=10):
    with urllib.request.urlopen(f"http://127.0.0.1:{PORT}{path}",
                                timeout=timeout) as r:
        return json.loads(r.read().decode())


def _post(path, payload, timeout=180):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}{path}", method="POST",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_up(budget=150):
    t0 = time.time()
    while time.time() - t0 < budget:
        try:
            _get("/state", timeout=2)
            return True
        except urllib.error.HTTPError:
            return True
        except Exception:
            time.sleep(1)
    return False


def launch(arm):
    import tempfile
    logdir = pathlib.Path(tempfile.gettempdir()) / "costbench"
    logdir.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "OMNISIM_LOG_PATH": str(logdir / f"{arm}.log")}
    key = os.environ.get("OMNI_KEY", "").strip()
    env.pop("OMNI_KEY", None)
    if arm == "llm":
        # Read from the environment or the key store, never from a command
        # line or this file.
        key_file = os.environ.get(KEY_FILE_ENV, "").strip()
        if not key and key_file:
            path = pathlib.Path(key_file)
            if not path.exists():
                raise SystemExit(f"{KEY_FILE_ENV} points at {path}, which does not exist")
            key = path.read_text(encoding="utf-8").strip()
        if not key:
            raise SystemExit(
                "the llm arm needs an OmniKey: set OMNI_KEY, or set "
                f"{KEY_FILE_ENV} to a file holding one (`python -m omnisim key`)")
        env["OMNI_KEY"] = key
        # ⚠️ The key alone is not enough. The bridge wraps its whole relay
        # import in `except Exception`, so if the CONTROLLER's interpreter
        # cannot import `omnilink`, the relay is silently stubbed out and
        # the demo falls back to the offline router -- reporting "no
        # OMNI_KEY", which is not what happened. Controller stdout is
        # discarded on Windows, so nothing says so. Put the client library
        # on the path explicitly. OMNILINK_LIB_SRC is for a source checkout of
        # the SDK; with the published package installed it is unnecessary.
        lib_src = os.environ.get("OMNILINK_LIB_SRC", "").strip()
        if lib_src and pathlib.Path(lib_src).exists():
            env["PYTHONPATH"] = os.pathsep.join(
                [lib_src] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "run-headless", WORLD,
         "--duration", "1800"],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env)
    if not wait_up():
        reap(proc)
        raise SystemExit(f"{arm}: bridge never came up")
    return proc


def reap(proc):
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)]
                   if os.name == "nt" else ["kill", "-9", str(proc.pid)],
                   capture_output=True)
    time.sleep(2)


def _pose():
    st = _get("/state")
    q = st.get("pose") or st
    return (float(q.get("x", 0.0)), float(q.get("y", 0.0)),
            float(q.get("yaw", 0.0)))


def _settle(timeout=25.0):
    """Wait until the robot actually stops.

    ⚠️ Without this the bench measured API latency, not the robot: commands
    return as soon as they are accepted, so three of them completed in 0.1 s
    and the 24-hour extrapolation implied 2.6 million cycles a day. The
    cadence that matters is how long the WORK takes, because that is what
    sets how often anyone has to talk to the robot.
    """
    last = _pose()
    stable = 0
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        now = _pose()
        if all(abs(a - b) < 0.004 for a, b in zip(now, last)):
            stable += 1
            if stable >= 3:
                return
        else:
            stable = 0
        last = now


def _read_usage():
    try:
        latest = (_get("/usage") or {}).get("latest") or {}
    except Exception:
        return None
    if not latest:
        return None
    return (int(latest.get("input_units") or latest.get("input_tokens") or 0),
            int(latest.get("output_units") or latest.get("output_tokens") or 0),
            str(latest.get("engine") or ""))


def usage_after_turn(previous, budget_s=12.0):
    """Wait for THIS turn's usage delta, not the previous turn's.

    The relay computes usage on a background worker after the reply lands,
    so reading /usage the instant a prompt returns races it and reports the
    turn before. Poll until the payload changes.
    """
    t0 = time.time()
    while time.time() - t0 < budget_s:
        cur = _read_usage()
        if cur is not None and cur != previous:
            return cur
        time.sleep(0.4)
    return _read_usage() or (0, 0, "")


def run(arm, cycles, budget_usd):
    proc = launch(arm)
    turns, tin, tout, engine = 0, 0, 0, ""
    last_usage = None
    t0 = time.time()
    try:
        for c in range(cycles):
            for text in CYCLE:
                _post("/prompt", {"text": text})
                _settle()
                turns += 1
                fresh = usage_after_turn(last_usage)
                last_usage = fresh
                i, o, e = fresh
                tin += i
                tout += o
                engine = e or engine
                spent = _cost(engine, tin, tout)
                if budget_usd and spent > budget_usd:
                    print(f"  !! budget ${budget_usd:.2f} reached after "
                          f"{turns} turns; stopping", flush=True)
                    raise KeyboardInterrupt
            print(f"  cycle {c + 1}/{cycles}  turns={turns} "
                  f"tokens={tin + tout:,}  {time.time() - t0:.1f}s", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - t0
        reap(proc)
    return {"arm": arm, "turns": turns, "cycles_done": max(1, turns // len(CYCLE)),
            "input_units": tin, "output_units": tout, "engine": engine,
            "elapsed_s": round(elapsed, 1)}


def _cost(engine, tin, tout):
    _, pin, pout = PRICES.get(engine, PRICES["g1-engine"])
    return tin / 1e6 * pin + tout / 1e6 * pout


def report(r):
    name, pin, pout = PRICES.get(r["engine"], PRICES["g1-engine"])
    total = _cost(r["engine"], r["input_units"], r["output_units"])
    cycles = r["cycles_done"]
    per_cycle_s = r["elapsed_s"] / max(1, cycles)
    per_cycle_usd = total / max(1, cycles)
    cycles_per_day = 86400 / per_cycle_s if per_cycle_s else 0
    day = per_cycle_usd * cycles_per_day

    print(f"\n-- {r['arm']} --")
    print(f"  engine              {r['engine'] or '(none — no model was called)'}"
          + (f"  [{name}]" if r["engine"] else ""))
    print(f"  commands issued     {r['turns']}")
    print(f"  cycles completed    {cycles}")
    print(f"  tokens              {r['input_units']:,} in / {r['output_units']:,} out")
    print(f"  wall clock          {r['elapsed_s']}s  "
          f"({per_cycle_s:.1f}s per cycle, MEASURED)")
    print(f"  cost of this run    ${total:.4f}")
    print(f"  cost per cycle      ${per_cycle_usd:.5f}")
    print(f"  -> 24 h at this cadence ({cycles_per_day:,.0f} cycles) "
          f"${day:,.2f}")
    return {**r, "usd_run": round(total, 6),
            "usd_per_cycle": round(per_cycle_usd, 6),
            "s_per_cycle": round(per_cycle_s, 2),
            "usd_24h": round(day, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="append",
                    choices=["deterministic", "llm"], default=None)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--budget", type=float, default=1.00,
                    help="stop the llm arm once it has spent this many USD")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    arms = args.arm or ["deterministic"]

    rows = []
    for arm in arms:
        print(f"\n=== {arm} ===", flush=True)
        rows.append(report(run(arm, args.cycles, args.budget)))

    if len(rows) > 1:
        print("\n=== head to head, 24 hours of the same robot task ===")
        for r in rows:
            print(f"  {r['arm']:<15} ${r['usd_24h']:>12,.2f}")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(rows, indent=2),
                                          encoding="utf-8")
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
