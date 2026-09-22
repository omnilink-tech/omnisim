#!/usr/bin/env python3
# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Legacy single-robot lane -- one prompt, one demo world, one predicate.

NOT the agent benchmark. This is the world-smoke lane: it launches a
single-robot chat-demo world headlessly and checks that it loads, spawns a
bridge and responds to a prompt.

⚠️ ITS RECORDED RUNS ARE HISTORY, AND THE MODE THAT PRODUCED THEM IS GONE.
Every file in results/ was written in mode "local" with engine null: the
bridge's keyword ladder answered a keyless POST /prompt, with no model
anywhere. The per-bridge ladders and the shared intent_router module were
DELETED on 2026-09-22, when an OmniKey became required for every OmniLink AI
experience, Free included. A bridge with no relay now refuses /prompt with
401 omnikey_required before the sentence is interpreted at all, so that mode
has no target and cannot be re-run; main() refuses rather than launch a world
and write fresh rows under the old label. Those result files are evidence of
runs that happened and are never edited -- read them as ladder runs. A re-run
today needs OMNI_KEY and goes through the parser-then-model path like every
other lane, and its rows are recorded as mode "omnilink".

⚠️ What the keyed lane should select if a no-model arm is wanted again is a
benchmark-design question for whoever owns this bench, not something this
runner should guess at. It does not substitute one.

The customer-facing agent benchmark is `matrix.py` in this directory: 16
tasks over five categories against the husky_swarm stack, graded from
measured pose and the recorded tool-call trace, with engine-matrix,
cost/latency capture and a dry run that needs neither key nor simulator.
See README.md.

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

Mode: OMNI_KEY is REQUIRED (2026-09-22) and the run is recorded as mode
"omnilink"; OMNILINK_ENGINE picks the engine. There is no keyless mode.
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

        # ⚠️ mode "local" IS NO LONGER WRITTEN. It meant "the bridge's keyword
        # ladder answered this, no model anywhere"; that ladder was deleted on
        # 2026-09-22 and a keyless /prompt is refused 401, so labelling a new
        # row "local" would file a refusal alongside the ladder-era rows in
        # results/ as if they measured the same thing. main() refuses a
        # keyless run outright; this label is the belt to that brace.
        keyed = bool(os.environ.get("OMNI_KEY", "").strip())
        engine = os.environ.get("OMNILINK_ENGINE", "g1-engine")
        mode = "omnilink" if keyed else "refused_no_omnikey"

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
    parser = argparse.ArgumentParser(
        description=(
            "Legacy single-robot WORLD-SMOKE lane: launch one chat-demo\n"
            "world headlessly, POST one prompt, grade the measured pose.\n"
            "\n"
            "OMNI_KEY is REQUIRED (2026-09-22). A bridge with no relay\n"
            "answers POST /prompt with 401 omnikey_required, so the keyless\n"
            "mode every row in results/ was recorded in -- the bridge's\n"
            "keyword ladder, deleted that day -- no longer exists, and a\n"
            "keyless run is refused instead of measuring the refusals."),
        formatter_class=argparse.RawDescriptionHelpFormatter)
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

    # ── The keyless lane is REFUSED, loudly, before a world is launched ──
    # This lane's only mode used to be keyless: the bridge's keyword ladder
    # answered POST /prompt with no model and no account. That ladder was
    # deleted on 2026-09-22 and an OmniKey is now required for every OmniLink
    # AI experience, Free included, so a keyless run here would launch a
    # world, collect `prompt failed: HTTP Error 401` on every task, and file
    # the rows in results/ next to the ladder-era rows that carry the same
    # world, the same prompt and the same predicate. Refusing is the point:
    # measuring a refusal and publishing it as a mode is worse than not
    # measuring at all.
    if not os.environ.get("OMNI_KEY", "").strip():
        print(
            "REFUSED: OMNI_KEY is not set.\n"
            "\n"
            "  This lane's old keyless mode is GONE. Until 2026-09-22 a\n"
            "  keyless POST /prompt was answered by the bridge's keyword\n"
            "  ladder (recorded as mode \"local\", engine null, no model in\n"
            "  the loop). The per-bridge ladders and the shared\n"
            "  omnisim_bridges.intent_router module were deleted that day,\n"
            "  and an OmniKey is required for every OmniLink AI experience,\n"
            "  Free included: with no relay the bridge answers\n"
            "  401 omnikey_required and nothing actuates.\n"
            "\n"
            "  So this run would measure refusals, not a robot. The existing\n"
            "  files in results/ are ladder-era evidence; they are never\n"
            "  edited, and no new row may be filed beside them under the\n"
            "  same label.\n"
            "\n"
            "  Run it with a key:   python -m omnisim key\n"
            "                       set OMNI_KEY=olink_...   (export on POSIX)\n"
            "  Then the bridge interprets with the deterministic parser first\n"
            "  and the model answers what the parser declines; the row is\n"
            "  recorded as mode \"omnilink\".\n"
            "\n"
            "  A no-model arm, if this bench wants one again, is a\n"
            "  benchmark-design decision for its owner. This runner does not\n"
            "  substitute one.")
        return 2

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
