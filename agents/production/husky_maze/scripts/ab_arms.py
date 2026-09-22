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
"""A/B the husky_maze arms: DETERMINISTIC solver vs LLM agent, same bridge, same tools.

Why this exists
---------------
`docs/RESULTS.md` has been RETRACTED since 2026-07-19 ("treat the Husky Maze demo
as unquantified") because every number on it predates `b18bd7a3`, which removed
teleport-through-walls recovery and turned `complete_mission` from agent
self-report into a real grader. Nobody re-measured afterwards. This rig is that
re-measurement, and it adds the arm comparison:

  arm D (deterministic)  agents/production/husky_maze/solve.py    -- BFS / wall-follow, no model
  arm L (llm)            scripts/chat_drive.py                    -- OmniLink agent, needs OMNI_KEY

Both drive the SAME bridge (`husky_omnilink_bridge` on :6070) through the SAME
motion primitives, and both are graded by the SAME check: the bridge verifies
every claimed cell against the trail it recorded from the physics body pose.

What it measures per trial: success (bridge grader, not self-report), wall-clock,
cells actually visited, and a TRACE HASH -- the arm's own stdout with timing and
addresses stripped. Identical hashes across trials == provably zero behavioural
variance, which is the property the whole question turns on.

Runs are strictly SEQUENTIAL: the bridge port is fixed at 6070 by the world, and
this host is thermally capped to one heavy job at a time.

  python agents/production/husky_maze/scripts/ab_arms.py --arm d --trials 1
  python agents/production/husky_maze/scripts/ab_arms.py --arm both --trials 5

Environment (arm L only; arm D needs nothing)
    OMNI_KEY       the OmniKey itself. Preferred.
    OMNI_KEY_FILE  a path to a file holding one, read only when OMNI_KEY is
                   unset. There is no default path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import socket
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
AGENT_DIR = ROOT / "agents" / "production" / "husky_maze"
BRIDGE_PORT = 6070
# Arm L's key: OMNI_KEY in the environment, else a file this points at.
KEY_FILE_ENV = "OMNI_KEY_FILE"
EYE_PORT = 6071
BRIDGE_URL = f"http://127.0.0.1:{BRIDGE_PORT}"

WORLDS = {
    "maze": "projects/samples/demos/worlds/flagship/husky_maze.omniworld",
    "unknown": "projects/samples/demos/worlds/flagship/husky_maze_unknown.omniworld",
}

# Both arms can run these. corners / visual / blind are agent-only STRUCTURALLY --
# solve.py has no code path for a multi-objective, visual or tag-only brief. That
# boundary is a result, not an omission; --arm l can be pointed at them separately.

DEFAULT_PROMPT = ("Read your mission brief with get_mission, plan a route, and drive "
                  "the husky to the goal. Claim completion only once you are there.")

# Lines whose content legitimately varies run to run (clock, durations, addresses,
# memory ids). Stripped before hashing so the hash reflects BEHAVIOUR, not timing.
_NOISE = re.compile(
    r"(\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}"          # timestamps
    r"|0x[0-9a-fA-F]{6,}"                                 # addresses
    r"|\b\d+\.\d{3,}\s*s\b"                               # durations
    r"|\belapsed[^,\n]*"
    r"|\btook\s+[\d.]+"
    r"|\bwindow=\d+s"
    r"|\btokens?=[\d,]+)")


def _scrub(text: str) -> str:
    return "\n".join(_NOISE.sub("<var>", ln).rstrip() for ln in text.splitlines())


def trace_hash(text: str) -> str:
    return hashlib.sha256(_scrub(text).encode("utf-8", "replace")).hexdigest()[:16]


# ── bridge I/O ───────────────────────────────────────────────────────────────
def _get(path: str, timeout: float = 4.0):
    import urllib.request
    with urllib.request.urlopen(BRIDGE_URL + path, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def port_open(port: int, host: str = "127.0.0.1", timeout: float = 0.25) -> bool:
    """Short timeout on purpose: a closed port on this host takes ~2.1 s to refuse."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


def wait_for_bridge(deadline_s: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < deadline_s:
        if port_open(BRIDGE_PORT):
            try:
                _get("/state")
                return True
            except Exception:
                pass
        time.sleep(0.5)
    return False


def wait_for_ports_closed(deadline_s: float = 30.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < deadline_s:
        if not port_open(BRIDGE_PORT) and not port_open(EYE_PORT):
            return True
        time.sleep(0.5)
    return False


# ── engine lifecycle ─────────────────────────────────────────────────────────
def engine_binary() -> Path:
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin", "bin/omnisim"):
        p = ROOT / rel
        if p.exists():
            return p
    raise SystemExit("omnisim-bin not found -- run `python -m omnisim doctor`")


def spawn_engine(world_rel: str, log_path: Path):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)          # never share the default log
    env["OMNISIM_NO_WINDOW"] = "1"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stdio = open(log_path.with_suffix(".stdio.txt"), "wb")   # the engine KEEPS its stdout
    proc = subprocess.Popen(
        [str(engine_binary()), str(ROOT / world_rel),
         "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr"],
        cwd=str(ROOT), env=env, stdout=stdio, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL)
    proc._stdio = stdio          # keep the handle alive
    return proc


def reap(proc) -> None:
    """Kill the tree we spawned -- engine AND its controller children. Never anything else."""
    if proc is None or proc.poll() is not None:
        return
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                   capture_output=True, text=True)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
    try:
        proc._stdio.close()
    except Exception:
        pass


# ── the arms ─────────────────────────────────────────────────────────────────
def run_arm(arm: str, timeout_s: int, prompt: str, max_turns: int):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(ROOT)
    env["HUSKY_BRIDGE_URL"] = BRIDGE_URL
    env["PYTHONIOENCODING"] = "utf-8"

    if arm == "d":
        cmd = [sys.executable, str(AGENT_DIR / "solve.py")]
    else:
        # OMNI_KEY, or a file OMNI_KEY_FILE points at. No default path: this
        # file ships, and an operator's key store is not a public fact about
        # the rig. The key VALUE never enters the tree either way.
        key_file = (os.environ.get(KEY_FILE_ENV) or "").strip()
        if not env.get("OMNI_KEY") and key_file:
            path = Path(key_file)
            if not path.exists():
                raise SystemExit(f"{KEY_FILE_ENV} points at {path}, which does not exist")
            env["OMNI_KEY"] = path.read_text(encoding="utf-8").strip()
        if not env.get("OMNI_KEY"):
            raise SystemExit(
                f"arm L needs an OmniKey: set OMNI_KEY, or set {KEY_FILE_ENV} "
                "to a file holding one (`python -m omnisim key`)")
        cmd = [sys.executable, str(AGENT_DIR / "scripts" / "chat_drive.py"),
               prompt, "--max-turns", str(max_turns)]

    t0 = time.time()
    try:
        cp = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True,
                            text=True, errors="replace", timeout=timeout_s)
        out, rc, timed_out = (cp.stdout or "") + (cp.stderr or ""), cp.returncode, False
    except subprocess.TimeoutExpired as e:
        out = ((e.stdout or b"").decode("utf-8", "replace") if isinstance(e.stdout, bytes)
               else (e.stdout or ""))
        rc, timed_out = -9, True
    return {"seconds": round(time.time() - t0, 2), "rc": rc,
            "timed_out": timed_out, "stdout": out}


def grade():
    """The bridge's own verdict. `complete` is the post-b18bd7a3 grader, not self-report."""
    out = {"mission_complete": None, "visited_cells": None, "pose": None}
    try:
        m = _get("/mission")
        out["mission_complete"] = bool(m.get("complete"))
        out["mission_log"] = m.get("log")
    except Exception as e:
        out["error"] = f"mission: {e}"
    try:
        s = _get("/state")
        v = s.get("visited_cells")
        out["visited_cells"] = len(v) if isinstance(v, list) else None
        out["visited"] = v
        out["goal_reached"] = s.get("goal_reached")
        out["current_cell"] = s.get("current_cell")
        pose = {k: s[k] for k in ("x", "y", "yaw") if k in s}
        out["pose"] = pose or None
    except Exception as e:
        out["error"] = (out.get("error") or "") + f" state: {e}"
    return out


# ── one trial ────────────────────────────────────────────────────────────────
def trial(arm: str, world_key: str, i: int, args, run_dir: Path) -> dict:
    world = WORLDS[world_key]
    tag = f"{world_key}_{arm}_{i:02d}"
    log = run_dir / f"engine_{tag}.log"
    rec = {"arm": arm, "world": world_key, "trial": i,
           "utc": datetime.now(timezone.utc).isoformat(timespec="seconds")}

    if port_open(BRIDGE_PORT):
        rec["error"] = f"port {BRIDGE_PORT} busy before launch -- an orphan bridge holds it"
        return rec

    proc = None
    try:
        t_launch = time.time()
        proc = spawn_engine(world, log)
        if not wait_for_bridge(args.load_timeout):
            rec["error"] = f"bridge never came up within {args.load_timeout}s"
            rec["engine_alive"] = proc.poll() is None
            return rec
        rec["load_seconds"] = round(time.time() - t_launch, 2)

        r = run_arm(arm, args.arm_timeout, args.prompt, args.max_turns)
        rec.update({k: r[k] for k in ("seconds", "rc", "timed_out")})
        rec["trace_sha"] = trace_hash(r["stdout"])
        rec.update(grade())
        (run_dir / f"stdout_{tag}.txt").write_text(r["stdout"], encoding="utf-8",
                                                   errors="replace")
    finally:
        reap(proc)
        rec["ports_closed"] = wait_for_ports_closed()
    return rec


def summarise(rows):
    print("\n" + "=" * 78)
    print("RESULT   reached = bridge's goal_reached, computed from the PHYSICS body pose")
    print("         claimed = the arm also filed a complete_mission claim the bridge verified")
    print("=" * 78)
    hdr = "%-5s %-8s %5s %8s %8s %9s %8s %7s" % (
        "arm", "world", "n", "reached", "claimed", "sec mean", "sec sd", "traces")
    print(hdr + "\n" + "-" * len(hdr))
    for arm in ("d", "l"):
        for wk in WORLDS:
            sel = [r for r in rows if r["arm"] == arm and r["world"] == wk
                   and not r.get("error")]
            if not sel:
                continue
            ok = sum(1 for r in sel if r.get("goal_reached"))
            claimed = sum(1 for r in sel if r.get("mission_complete"))
            secs = [r["seconds"] for r in sel if "seconds" in r]
            sd = statistics.stdev(secs) if len(secs) > 1 else 0.0
            uniq = len({r.get("trace_sha") for r in sel})
            print("%-5s %-8s %5d %8s %8s %9.1f %8.2f %7s" % (
                arm.upper(), wk, len(sel), f"{ok}/{len(sel)}",
                f"{claimed}/{len(sel)}",
                statistics.mean(secs) if secs else 0.0, sd,
                f"{uniq} uniq"))
    print("\n`traces = 1 uniq` means every run produced a byte-identical behavioural")
    print("trace -- zero variance. >1 means the arm did something different each time.")
    errs = [r for r in rows if r.get("error")]
    if errs:
        print("\nERRORS:")
        for r in errs:
            print("  %-5s %-8s #%02d  %s" % (r["arm"].upper(), r["world"],
                                             r["trial"], r["error"]))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=["d", "l", "both"], default="d")
    p.add_argument("--worlds", default="maze",
                   help="comma-separated: maze,unknown (default: maze)")
    p.add_argument("--trials", type=int, default=1)
    p.add_argument("--arm-timeout", type=int, default=900)
    p.add_argument("--load-timeout", type=int, default=180)
    p.add_argument("--max-turns", type=int, default=150)
    p.add_argument("--prompt", default=DEFAULT_PROMPT)
    p.add_argument("--out", default=None)
    p.add_argument("--cooldown", type=int, default=20,
                   help="seconds between trials (thermal policy: one heavy job at a time)")
    args = p.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.out) if args.out else AGENT_DIR / "evidence" / f"ab_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    world_keys = [w.strip() for w in args.worlds.split(",") if w.strip()]
    for wk in world_keys:
        if wk not in WORLDS:
            raise SystemExit(f"unknown world '{wk}' (have: {', '.join(WORLDS)})")
    arms = ["d", "l"] if args.arm == "both" else [args.arm]

    print(f"[ab] root      {ROOT}")
    print(f"[ab] evidence  {run_dir}")
    print(f"[ab] arms={arms} worlds={world_keys} trials={args.trials}  (SEQUENTIAL)")

    rows, jsonl = [], run_dir / "trials.jsonl"
    for wk in world_keys:
        for arm in arms:
            for i in range(1, args.trials + 1):
                print(f"\n[ab] --- arm {arm.upper()} / {wk} / trial {i}/{args.trials} ---",
                      flush=True)
                rec = trial(arm, wk, i, args, run_dir)
                rows.append(rec)
                with jsonl.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(rec) + "\n")
                print("[ab] %s  reached=%s claimed=%s  %ss  trace=%s%s" % (
                    "OK " if not rec.get("error") else "ERR",
                    rec.get("goal_reached"), rec.get("mission_complete"),
                    rec.get("seconds"),
                    rec.get("trace_sha"),
                    "  " + rec["error"] if rec.get("error") else ""), flush=True)
                if args.cooldown and not (arm == arms[-1] and i == args.trials
                                          and wk == world_keys[-1]):
                    time.sleep(args.cooldown)

    summarise(rows)
    (run_dir / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"\n[ab] wrote {jsonl}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
