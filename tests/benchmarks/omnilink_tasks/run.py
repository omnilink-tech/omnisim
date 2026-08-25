#!/usr/bin/env python3
# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Legacy single-robot lane -- one prompt, one demo world, one predicate.

NOT the agent benchmark. This is the world-smoke lane: it launches a
single-robot chat-demo world headlessly and checks that it loads, spawns a
bridge and responds to a prompt. Its recorded runs were made in mode "local"
(the bridge's regex intent router, engine null) -- no LLM in the loop.

The customer-facing agent benchmark is `matrix.py` in this directory: 16
tasks over five categories against the husky_swarm stack, graded from
measured pose and the recorded tool-call trace, with engine-matrix,
cost/latency capture and an offline dry run. See README.md.

For each task in tasks.TASKS:

  1. Launch the world headlessly via omnisim-bin.
  2. Wait for the bridge port to come up.
  3. Sample one /get_robot_state for the spawn record.
  4. POST the prompt to /prompt.
  5. Poll /get_robot_state at ~5 Hz, feeding each sample to the grader
     until it returns pass OR the timeout elapses.
  6. Tear down omnisim-bin and record the result.

Outputs:

  - JSON record per (task, run) in results/.
  - Console table at the end.

Mode: "local" by default (no OMNI_KEY -> bridge uses regex intent
router). Set OMNI_KEY in the environment to route via the OmniLink
relay; OMNILINK_ENGINE picks the engine.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[2]
WORLDS_DIR = REPO_ROOT / "projects" / "samples" / "demos" / "worlds"
RESULTS_DIR = THIS_DIR / "results"

sys.path.insert(0, str(THIS_DIR))
from tasks import TASKS, Task, get_task  # noqa: E402


def _webots_bin() -> Optional[Path]:
    candidates = [
        REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
        Path(os.environ.get("OMNISIM_HOME", "")) / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe" if os.environ.get("OMNISIM_HOME") else None,
        Path(os.environ.get("OMNISIM_HOME", "")) / "bin" / "omnisim-bin" if os.environ.get("OMNISIM_HOME") else None,
    ]
    for c in candidates:
        if c is not None and Path(c).exists():
            return Path(c)
    return None


def _post(url: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 2.0) -> Dict[str, Any]:
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8") or "{}")


def wait_for_bridge(port: int, timeout_s: float = 45.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            _post(f"http://127.0.0.1:{port}/list_robots", {}, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.5)
    return False


def run_task(task: Task, webots: Path, webots_port: int = 1252) -> Dict[str, Any]:
    world_path = WORLDS_DIR / task.world
    if not world_path.exists():
        return {"id": task.id, "passed": False, "error": f"world not found: {world_path}"}

    print(f"\n=== {task.id} === world={task.world} prompt={task.prompt!r}")

    log = THIS_DIR / f"_run_{task.id}.log"
    proc = subprocess.Popen(
        [str(webots), f"--port={webots_port}", "--no-rendering", "--batch",
         "--mode=fast", "--minimize", "--stdout", "--stderr", str(world_path)],
        stdout=open(log, "wb"), stderr=subprocess.STDOUT,
    )

    try:
        if not wait_for_bridge(task.bridge_port, timeout_s=45):
            return {"id": task.id, "passed": False, "error": "bridge never came up"}

        # Spawn-state sample first.
        history: List[Dict[str, Any]] = []
        try:
            history.append(_post(f"http://127.0.0.1:{task.bridge_port}/get_robot_state"))
        except Exception:
            pass

        # Send the prompt.
        t_start = time.time()
        try:
            _post(f"http://127.0.0.1:{task.bridge_port}/prompt",
                  {"text": task.prompt}, timeout=10)
        except Exception as e:
            return {"id": task.id, "passed": False, "error": f"prompt failed: {e}"}

        # Poll state at 5 Hz until the grader returns pass or timeout.
        passed = False
        metric: float = 0.0
        note = ""
        deadline = t_start + task.timeout_s
        while time.time() < deadline:
            time.sleep(0.2)
            try:
                history.append(_post(f"http://127.0.0.1:{task.bridge_port}/get_robot_state"))
            except Exception:
                continue
            passed, metric, note = task.grader(history)
            if passed:
                break
        duration = time.time() - t_start

        final_state = history[-1] if history else {}

        engine = os.environ.get("OMNILINK_ENGINE", "g1-engine")
        mode = "omnilink" if os.environ.get("OMNI_KEY", "").strip() else "local"

        record = {
            "id": task.id,
            "world": task.world,
            "prompt": task.prompt,
            "mode": mode,
            "engine": engine if mode == "omnilink" else None,
            "duration_s": round(duration, 2),
            "passed": passed,
            "metric": round(metric, 4),
            "note": note,
            "final_state": final_state,
            "timestamp": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
        return record
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", action="append", default=[],
                        help="Run only these task ids (can repeat).")
    parser.add_argument("--list", action="store_true",
                        help="List the task plan and exit.")
    parser.add_argument("--webots-port", type=int, default=1252)
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    plan = TASKS if not args.only else [get_task(tid) for tid in args.only]

    if args.list:
        for t in plan:
            print(f"  {t.id:20s} {t.world:30s} {t.prompt!r}")
        return 0

    webots = _webots_bin()
    if webots is None:
        print("ERROR: omnisim-bin not found. Set OMNISIM_HOME or run from repo root.")
        return 1

    RESULTS_DIR.mkdir(exist_ok=True)
    records: List[Dict[str, Any]] = []
    for task in plan:
        rec = run_task(task, webots, webots_port=args.webots_port)
        records.append(rec)
        if not args.no_save:
            ts = rec.get("timestamp", "").replace(":", "").replace("-", "")
            out = RESULTS_DIR / f"{task.id}-{ts}.json"
            out.write_text(json.dumps(rec, indent=2), encoding="utf-8")
            print(f"  -> {out.relative_to(REPO_ROOT)}")

    # Console summary table.
    print("\n" + "=" * 78)
    print(f"{'task':22s} {'mode':10s} {'dur(s)':8s} {'pass':6s} {'metric':10s} note")
    print("-" * 78)
    for r in records:
        passed = "PASS" if r.get("passed") else "FAIL"
        note = r.get("note") or r.get("error", "")
        print(f"{r['id']:22s} {r.get('mode','-'):10s} "
              f"{r.get('duration_s',0):>7.2f}s {passed:6s} "
              f"{r.get('metric',0):>9.3f}  {note}")

    fails = sum(1 for r in records if not r.get("passed"))
    print(f"\n{len(records) - fails}/{len(records)} passed.")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
