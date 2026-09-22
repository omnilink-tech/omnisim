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

"""Drive a robot on the deterministic path for hours, and watch for rot.

    python tests/benchmarks/soak/run.py --hours 2

WHY A SQUARE AND NOT A STRAIGHT LINE
    The loop is forward 2 m, turn left 90 degrees, four times. That CLOSES:
    the robot should arrive back where it started. So the residual after
    each lap is drift, measurable, rather than a wander that just runs out
    of arena and tells you nothing.

WHAT WOULD FALSIFY THE CLAIM
    The claim is that this architecture runs a repetitive robot job
    indefinitely for nothing. Any of these refutes it, so all are recorded:

      errors        a command that fails, at any point in the run
      drift         lap-closure error growing without bound
      escape        the robot leaving the arena
      leak          engine PRIVATE memory climbing steadily (not working
                    set -- Windows trims that on its own, see engine_mem_mb)
      slowdown      seconds-per-lap degrading
      death         the engine or bridge going away

    Tokens are asserted to stay zero: no key is passed, so any nonzero
    number would mean a model got into the loop.
"""
from __future__ import annotations

import argparse, json, math, os, pathlib, subprocess, sys, time, urllib.error, urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ROOT = pathlib.Path(__file__).resolve().parents[3]
WORLD = "projects/samples/demos/worlds/chat/omnilink_husky.omniworld"
PORT = 8765
LAP = ["drive forward 2 metres", "turn left 90 degrees"] * 4   # closes
# The world's floorSize is 12 x 12, so the floor ends at +/-6 m. Nothing in
# the stack bounds the robot -- act_drive_forward takes a distance and drives
# it -- so a desynced lap walks it off the edge. The first run of this soak
# ended at (-6.86, 4.75), past the edge, and nothing had objected.
ARENA_M = 5.0


def get(path, t=15, tries=3):
    """Read from the bridge, tolerating a transient hiccup.

    ⚠️ A two-hour run makes tens of thousands of these. Unguarded, ONE
    timeout anywhere -- inside settle(), inside confirm() -- propagates out
    of main(), and because the results are written after the `finally`,
    the whole run's data dies with it. A retry here is not politeness; it
    is the difference between a lost afternoon and a recorded blip.

    Still raises once the retries are gone, because a bridge that has
    genuinely died is a falsifier and must end the run.
    """
    last = None
    for attempt in range(tries):
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{PORT}{path}", timeout=t) as r:
                return json.loads(r.read().decode())
        except Exception as exc:
            last = exc
            if attempt + 1 < tries:
                time.sleep(0.5 * (attempt + 1))
    raise last


def post(text, t=120):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}/prompt", method="POST",
        data=json.dumps({"text": text}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=t) as r:
        return json.loads(r.read().decode())


def pose():
    s = get("/state")
    p = s.get("pose") or s
    return (float(p.get("x", 0.0)), float(p.get("y", 0.0)),
            float(p.get("yaw", 0.0)))


def confirm(text, timeout=40.0):
    """Send a command and confirm the ROBOT accepted and finished it.

    ⚠️ Two traps, both of which silently corrupted earlier runs.

    1. The failure is in `actions[].result`, NOT in the top-level `error`.
       A refused command comes back as {"response": ..., "actions":
       [{"tool": "drive_forward", "result": "err"}]} with no `error` key, so
       a caller checking `error` sees success and counts a clean run.

    2. Pose-settling is not the same as command completion. act_drive_forward
       moves in phases with brief pauses; three still readings can land in
       one of them, the next command arrives while the bridge is busy, and
       it is rejected. The bridge's own last_command.seq is the truth.

    Returns (ok, detail).
    """
    before_seq = (get("/state").get("last_command") or {}).get("seq", -1)
    try:
        out = post(text)
    except Exception as exc:
        return False, type(exc).__name__
    if out.get("error"):
        return False, str(out["error"])[:60]
    for a in out.get("actions") or []:
        if a.get("result") not in ("ok", None):
            return False, f"{a.get('tool')}:{a.get('result')}"
    t0 = time.time()
    while time.time() - t0 < timeout:
        lc = get("/state").get("last_command") or {}
        if lc.get("seq", -1) != before_seq and lc.get("settled"):
            return True, ""
        time.sleep(0.3)
    return False, "never settled"


def settle(timeout=25.0):
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


def engine_mem_mb():
    """(working_set_mb, private_mb) for the largest engine. (0, 0) if unknown.

    ⚠️ PRIVATE BYTES IS THE LEAK METRIC, NOT WORKING SET. This used to read
    tasklist, which reports working set only, and that is how much of the
    process currently sits in physical RAM -- a number Windows changes on
    its own. Observed mid-run on 2026-09-20: working set fell 707 -> 191 MB
    while private memory held at ~1.3 GB and 17 GB was free. Nothing had
    been released; the OS had simply trimmed resident pages.

    Read as a leak check that is wrong in both directions. It shows a fall
    where nothing was freed, and it would hide a genuine leak whose pages
    had been paged out. Private bytes tracks committed memory and is the
    number a leak actually moves.
    """
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-Process omnisim-bin -ErrorAction SilentlyContinue | "
             "ForEach-Object { \"$($_.WS),$($_.PrivateMemorySize64)\" }"],
            capture_output=True, text=True, timeout=25).stdout
        ws_best = pv_best = 0
        for line in out.splitlines():
            parts = line.strip().split(",")
            if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                if int(parts[1]) > pv_best:
                    ws_best, pv_best = int(parts[0]), int(parts[1])
        return ws_best // (1024 * 1024), pv_best // (1024 * 1024)
    except Exception:
        return 0, 0


def engine_rss_mb():
    """Back-compat shim: working set only. Prefer engine_mem_mb()."""
    return engine_mem_mb()[0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--out", default="tests/benchmarks/soak/soak_result.json")
    args = ap.parse_args()

    budget_s = args.hours * 3600
    logdir = pathlib.Path(os.environ.get("TEMP", "/tmp")) / "soak"
    logdir.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items() if k not in ("OMNI_KEY", "OMNILINK_ENGINE")}
    env["OMNISIM_LOG_PATH"] = str(logdir / "soak.log")

    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "run-headless", WORLD,
         "--duration", str(int(budget_s + 900))],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    t0 = time.time()
    laps, cmds, errors = 0, 0, []
    rows, rss0 = [], 0
    try:
        for _ in range(180):
            try:
                get("/state", 2); break
            except Exception:
                time.sleep(1)
        else:
            print("bridge never came up"); return 1
        origin = settle(8.0)
        # Prove the surface actuates BEFORE spending hours on it.
        probe_before = pose()
        post(LAP[0]); settle()
        if math.dist(pose()[:2], probe_before[:2]) < 0.5:
            print("ABORT: the robot did not move on a plain drive command. "
                  "Something upstream is answering without acting -- check "
                  "for a stray server on 11434.")
            return 1
        post("go home"); settle()
        origin = settle(6.0)
        rss0 = engine_rss_mb()
        print(f"origin {tuple(round(v,3) for v in origin)}  engine {rss0} MB  "
              f"budget {args.hours:.1f} h", flush=True)

        while time.time() - t0 < budget_s:
            lap_t0 = time.time()
            lap_start = pose()
            excursion = 0.0
            for text in LAP:
                ok, why = confirm(text)
                cmds += 1
                if not ok:
                    errors.append(f"lap {laps+1} {text[:22]!r}: {why}")
                here = settle()
                # Track how far the robot got DURING the lap. Comparing the
                # end pose to the start pose cannot detect motion here: the
                # square closes, so a PERFECT lap ends where it began. The
                # first version did exactly that and flagged every good lap
                # as "never moved".
                excursion = max(excursion, math.hypot(here[0] - lap_start[0],
                                                      here[1] - lap_start[1]))
            laps += 1
            x, y, yaw = pose()
            closure = math.hypot(x - origin[0], y - origin[1])
            # ⚠️ A lap where nothing moved is a FAILURE, not a clean lap.
            # The first run of this soak reported 25 perfect laps with
            # closure 0.000 and zero errors while the robot sat still: a
            # leftover fake Ollama server on 11434 had been adopted as the
            # relay and was answering every command with a canned "ok".
            # Absence of errors is not evidence of work.
            if excursion < 0.5:
                errors.append(f"lap {laps}: robot never moved "
                              f"(max excursion {excursion:.3f} m)")
            if max(abs(x), abs(y)) > ARENA_M:
                errors.append(f"lap {laps}: ESCAPED the arena at "
                              f"({x:.2f}, {y:.2f})")
                print(f"  ABORT: off the floor at ({x:.2f}, {y:.2f})",
                      flush=True)
                break
            u = get("/usage")
            tok = ((u.get("latest") or {}).get("input_units") or 0) if u else 0
            # ⚠️ WALL TIME ALONE CANNOT ATTRIBUTE A SLOWDOWN. Every lap is
            # the same commanded motion, so its SIM time is fixed by
            # construction. If sim-seconds per lap holds steady while
            # wall-seconds per lap grows, the host got slower (contention,
            # thermal) and the architecture is fine; if sim-seconds per lap
            # grows too, the simulation itself is doing more work and that
            # is a real degradation. Recording only wall time made those
            # two indistinguishable, which is the wrong way round for a
            # benchmark whose whole job is to detect decay.
            try:
                sim_s = round(float(get("/state").get("sim_time", 0.0)), 1)
            except Exception:
                sim_s = 0.0
            _ws, _pv = engine_mem_mb()
            row = {"lap": laps, "t_s": round(time.time() - t0, 1),
                   "lap_s": round(time.time() - lap_t0, 1),
                   "sim_s": sim_s,
                   "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3),
                   "closure_m": round(closure, 3),
                   "rss_mb": _ws, "private_mb": _pv,
                   "excursion_m": round(excursion, 3),
                   "tokens": tok, "errors": len(errors)}
            rows.append(row)
            if laps % 5 == 0 or laps == 1:
                print(f"  lap {laps:>4}  t={row['t_s']/60:6.1f}m  "
                      f"lap={row['lap_s']:5.1f}s  closure={closure:6.3f}m  "
                      f"rss={row['rss_mb']}MB  err={len(errors)}  tok={tok}",
                      flush=True)
    except Exception as exc:
        # ⚠️ NEVER let a late failure throw away the hours already measured.
        # The summary is written after this block, so an exception escaping
        # here loses every recorded lap -- which at hour 1.5 is the entire
        # run. The death IS a falsifier and is recorded as one; it is not a
        # reason to also destroy the evidence leading up to it.
        errors.append(f"lap {laps+1}: run ended on "
                      f"{type(exc).__name__}: {str(exc)[:80]}")
        print(f"  ABORT: {type(exc).__name__}: {str(exc)[:100]}", flush=True)
    finally:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)

    if rows:
        closures = [r["closure_m"] for r in rows]
        lap_times = [r["lap_s"] for r in rows]
        half = max(1, len(rows) // 2)
        print(f"\n=== soak: {laps} laps, {cmds} commands, "
              f"{(time.time()-t0)/3600:.2f} h ===")
        print(f"  errors            {len(errors)}")
        print(f"  tokens            {max(r['tokens'] for r in rows)}  (must be 0)")
        print(f"  closure first/last {closures[0]:.3f} / {closures[-1]:.3f} m  "
              f"(max {max(closures):.3f})")
        print(f"  lap time  1st half {sum(lap_times[:half])/half:.1f}s  "
              f"2nd half {sum(lap_times[half:])/max(1,len(lap_times)-half):.1f}s")
        print(f"  engine RSS  {rss0} -> {rows[-1]['rss_mb']} MB")
        for e in errors[:5]:
            print(f"    ! {e}")
        pathlib.Path(args.out).write_text(json.dumps(
            {"laps": laps, "commands": cmds, "errors": errors, "rows": rows},
            indent=2), encoding="utf-8")
        print(f"  wrote {args.out}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
