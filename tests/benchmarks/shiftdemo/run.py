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

"""A Husky works a 100-prompt shift, and we count every penny it costs.

    python tests/benchmarks/shiftdemo/run.py --dry      # price it, spend nothing
    python tests/benchmarks/shiftdemo/run.py            # the demo

⚠️ MOBILE ONLY. THIS RUNNER HAS NOT BEEN GENERALIZED; USE run_cascade.py
--------------------------------------------------------------------
`run_cascade.py --profile husky|ur5e|lite3|mavic` is the one that works on
every robot class. This file still reads the pose as

    float(s.get("x", 0)), float(s.get("y", 0)), float(s.get("yaw", 0))

and an arm bridge publishes `q`/`tcp`/`gripper`, so pointed at anything
but a mobile base it returns (0,0,0) for every sample. That does not
error -- it scores. Every `cmd` fails and every ask / chat / refuse /
halt PASSES, because those four only require the robot to stay still, and
the headline comes out "zero false actuations" on a robot nobody could
see. See tests/benchmarks/shiftdemo/profiles.py for the fail-loud read
and the VOID check that replaced it.

It is kept because the cost accounting here (banking `/usage` deltas
exactly when `parser.relay_calls` increments) is the only record of how
that number is obtained, and because the run it produced is cited.

WHAT THIS IS MEANT TO SHOW
--------------------------
Cost scales with what you SAY, not with what the robot DOES.

A hundred prompts drive a shift's worth of motion. The deterministic
parser answers what it can for nothing; the model is called only for what
the parser declines; the gate vets everything either produces; the
executor runs what survives. Three of those four stages are free.

The number that makes the point is not the total. It is the ratio between
what this shift cost and what it would have cost with a model deciding
every command — including the thousands of wheel commands nobody typed.

HOW IT IS COUNTED
-----------------
  model calls      only the prompts that actually reached the relay
  tokens           reported by the provider, never estimated
  robot commands   what the executor dispatched, which is far more than
                   the number of prompts
  refusals         what the gate stopped, and which rule stopped it

⚠️ EVERY COST HERE IS MEASURED, NOT MODELLED. The per-call price comes
from the provider's own usageMetadata at the list rates in
agents/production/husky_maze/docs/COSTS.md. The counterfactual column is
arithmetic on those same measured tokens — it is a PRICE, not a run, and
is labelled as such wherever it appears.

⚠️ AND IT IS JUDGED BY THE ROBOT. A prompt counts as executed only if the
pose moved; a refusal counts only if it did not. A demo that trusts its
own reply text is a demo that can lie, and this project has been caught
by that twice.
"""
from __future__ import annotations

import argparse, json, math, os, pathlib, subprocess, sys, time
import threading, urllib.error, urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from script import SHIFT  # noqa: E402

WORLD = "projects/samples/demos/worlds/chat/omnilink_husky.omniworld"
PORT = 8765
ARENA_M = 5.5
POS_TOL, YAW_TOL = 0.03, 0.03

# Gemini Flash list rates, per COSTS.md — the same ones costbench priced.
USD_IN_PER_M, USD_OUT_PER_M = 0.50, 3.00
# Measured in tests/benchmarks/costbench: what ONE bridge turn costs.
MEASURED_IN, MEASURED_OUT = 5592, 84
PER_CALL_USD = (MEASURED_IN * USD_IN_PER_M + MEASURED_OUT * USD_OUT_PER_M) / 1e6


def get(path, t=20, tries=3):
    last = None
    for i in range(tries):
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{PORT}{path}", timeout=t) as r:
                return json.loads(r.read().decode())
        except Exception as exc:
            last = exc
            if i + 1 < tries:
                time.sleep(0.4 * (i + 1))
    raise last


def post(path, payload, t=180):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}{path}", method="POST",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=t) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        try:
            return e.code, json.loads(body)
        except Exception:
            return e.code, {"raw": body[:200]}


def pose():
    s = get("/state")
    return float(s.get("x", 0)), float(s.get("y", 0)), float(s.get("yaw", 0))


def settle(timeout=30.0):
    last, stable, t0 = pose(), 0, time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        now = pose()
        if all(abs(a - b) < 0.004 for a, b in zip(now, last)):
            stable += 1
            if stable >= 3:
                return now
        else:
            stable = 0
        last = now
    return pose()


def stats():
    """(latest_in, latest_out, relay_calls, short_circuited).

    ⚠️ `latest_usage` is the MOST RECENT TURN's delta, not a running
    total -- the relay keeps no cumulative counter. Reading it as a total
    would have under-counted the whole shift to whatever the last call
    happened to cost. What IS cumulative is parser_stats: `relay_calls`
    and `short_circuited`. So the loop accumulates the per-turn delta
    exactly when relay_calls increments, and never otherwise.
    """
    try:
        u = get("/usage") or {}
    except Exception:
        return 0, 0, 0, 0
    latest = u.get("latest") or {}
    ps = u.get("parser") or {}
    return (int(latest.get("input_units") or 0),
            int(latest.get("output_units") or 0),
            int(ps.get("relay_calls") or 0),
            int(ps.get("short_circuited") or 0))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="print the shift and its worst-case price; run nothing")
    ap.add_argument("--budget-usd", type=float, default=0.40)
    ap.add_argument("--out", default=str(HERE / "shift_result.json"))
    ap.add_argument("--offline", action="store_true",
                    help="no model at all: parser + gate only, $0.00")
    a = ap.parse_args()

    kinds = {}
    for _, k in SHIFT:
        kinds[k] = kinds.get(k, 0) + 1
    worst = len(SHIFT) * PER_CALL_USD
    print(f"shift: {len(SHIFT)} prompts  " +
          "  ".join(f"{k}={v}" for k, v in sorted(kinds.items())))
    print(f"worst case (every prompt reaches the model): ${worst:.3f}")
    print(f"budget cap: ${a.budget_usd:.2f}")
    if worst > a.budget_usd and not a.offline:
        print("  -> parser-first is ON, so most prompts never reach it;")
        print("     the run aborts if measured spend passes the cap.")
    if a.dry:
        return 0

    logdir = pathlib.Path(os.environ.get("TEMP", "/tmp")) / "shiftdemo"
    logdir.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items()}
    env["OMNISIM_LOG_PATH"] = str(logdir / "shift.log")
    # The cascade: the parser answers confident COMMANDs itself, so only
    # what it declines costs anything.
    env["OMNISIM_BRIDGE_PARSER_FIRST"] = "1"
    if a.offline:
        env.pop("OMNI_KEY", None)

    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "run-headless", WORLD,
         "--duration", "5400"], cwd=str(ROOT), stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, env=env)
    rows, t0 = [], time.time()
    moved_total = 0.0
    try:
        for _ in range(200):
            try:
                get("/state", 2); break
            except Exception:
                time.sleep(1)
        else:
            print("bridge never came up"); return 1
        settle(8.0)
        origin = pose()
        tin = tout = 0
        _, _, calls_prev, sc0 = stats()
        calls_total = 0
        print(f"\n{'#':>3} {'kind':<7} {'moved':>7} {'via':<7} prompt")
        print("-" * 74)

        for i, (text, kind) in enumerate(SHIFT, 1):
            p0 = pose()
            code, body = post("/prompt", {"text": text})
            p1 = settle()
            d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            dyaw = abs((p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)
            moved = d > POS_TOL or dyaw > YAW_TOL
            moved_total += d
            via = (body or {}).get("via") or ("relay" if code == 200 else "?")
            rows.append({
                "n": i, "prompt": text, "kind": kind, "http": code,
                "moved": moved, "dist_m": round(d, 3),
                "dyaw_rad": round(dyaw, 3), "via": via,
                "error": (body or {}).get("error"),
                "reply": str((body or {}).get("response"))[:120],
                "actions": [x.get("tool") for x in ((body or {}).get("actions") or [])],
                "x": round(p1[0], 3), "y": round(p1[1], 3),
            })
            expect_move = (kind == "cmd")
            rows[-1]["as_expected"] = (moved == expect_move)
            if i % 10 == 0 or moved != expect_move:
                print(f"{i:>3} {kind:<7} {d:>6.2f}m {via:<7} {text[:44]}")
            li, lo, calls_now, _sc = stats()
            hit_model = calls_now > calls_prev
            if hit_model:
                # A model call happened on this turn: bank its measured
                # tokens once, then move the watermark.
                tin += li
                tout += lo
                calls_total += (calls_now - calls_prev)
                calls_prev = calls_now
            rows[-1]["model_call"] = hit_model
            rows[-1]["tokens"] = [li, lo] if hit_model else [0, 0]
            spend = (tin * USD_IN_PER_M + tout * USD_OUT_PER_M) / 1e6
            if spend > a.budget_usd:
                print(f"\n  ABORT: measured spend ${spend:.3f} passed the "
                      f"${a.budget_usd:.2f} cap at prompt {i}")
                break
            if max(abs(p1[0]), abs(p1[1])) > ARENA_M:
                print(f"\n  ABORT: off the floor at ({p1[0]:.2f}, {p1[1]:.2f})")
                break
        _, _, _, sc1 = stats()
    finally:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)

    dt = time.time() - t0
    spend = (tin * USD_IN_PER_M + tout * USD_OUT_PER_M) / 1e6
    model_calls = calls_total
    by_parser = sum(1 for r in rows if r["via"] == "parser")
    executed = sum(1 for r in rows if r["moved"])
    per_kind = {}
    for r in rows:
        k = r["kind"]
        d = per_kind.setdefault(k, [0, 0])
        d[1] += 1
        d[0] += 1 if r.get("as_expected") else 0
    refused = [r for r in rows if r.get("error")]
    robot_cmds = sum(len(r["actions"]) for r in rows)
    end = (rows[-1]["x"], rows[-1]["y"]) if rows else (0, 0)

    print("\n" + "=" * 74)
    print(f"SHIFT COMPLETE  {len(rows)} prompts in {dt/60:.1f} min")
    print("=" * 74)
    print(f"  robot commands dispatched   {robot_cmds}")
    print(f"  prompts that moved it       {executed}")
    print(f"  distance driven             {moved_total:.1f} m")
    print(f"  ended at                    ({end[0]:.2f}, {end[1]:.2f})  "
          f"origin ({origin[0]:.2f}, {origin[1]:.2f})")
    print(f"  refused by the gate         {len(refused)}")
    print("  behaved as expected, by kind:")
    for k in ("cmd", "halt", "ask", "refuse", "chat"):
        if k in per_kind:
            ok_, n_ = per_kind[k]
            print(f"    {k:<8} {ok_:>3}/{n_:<3}"
                  + ("   <- moved when it should not" if k != "cmd" and ok_ < n_ else ""))
    print()
    print(f"  answered by the parser      {by_parser}   (free)")
    print(f"  reached the model           {model_calls}")
    print(f"  tokens                      {tin} in / {tout} out  (provider-reported)")
    print(f"  ** MEASURED COST            ${spend:.4f} **")
    print()
    n = max(1, robot_cmds)
    print(f"  PRICED, NOT RUN -- a model deciding each of the {n} robot")
    print(f"  commands at the same measured rate: ${n * PER_CALL_USD:.2f}")
    if spend > 0:
        print(f"  ratio                       {n * PER_CALL_USD / spend:,.0f}x")
    print(f"  per hour of shift           ${spend / max(dt/3600, 1e-9):.4f}/h")

    pathlib.Path(a.out).write_text(json.dumps({
        "prompts": len(rows), "minutes": round(dt / 60, 2),
        "robot_commands": robot_cmds, "distance_m": round(moved_total, 2),
        "by_parser": by_parser, "model_calls": model_calls,
        "tokens_in": tin, "tokens_out": tout, "usd": round(spend, 6),
        "counterfactual_usd": round(robot_cmds * PER_CALL_USD, 4),
        "refused": len(refused), "rows": rows}, indent=2), encoding="utf-8")
    print(f"\n  wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
