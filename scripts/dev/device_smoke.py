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

"""Smoke-test every world in projects/samples/devices/worlds/ headlessly.

One-shot audit script: runs each device demo for 10s headless, records
pass/fail/timeout, writes results to device_smoke_results.json.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WORLDS_DIR = REPO_ROOT / "projects" / "samples" / "devices" / "worlds"
RUNNER = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"
RESULTS_PATH = REPO_ROOT / "device_smoke_results.json"
LOG_PATH = REPO_ROOT / "omnisim_log.txt"

DURATION = 8
TIMEOUT = 30


def kill_zombie_omnisim() -> int:
    """Kill any leftover webots processes from a previous crashed run."""
    if sys.platform != "win32":
        subprocess.run(["pkill", "-9", "-f", "webots"], check=False)
        return 0
    killed = 0
    for name in ("omnisim-bin.exe", "webots.exe"):
        proc = subprocess.run(
            ["taskkill", "/F", "/IM", name],
            capture_output=True, text=True, check=False,
        )
        if proc.returncode == 0:
            killed += 1
    return killed


def clear_log_with_retry(path: Path, attempts: int = 10, delay: float = 0.3) -> bool:
    """Try to delete the log file, waiting if the previous omnisim-bin holds the handle."""
    for i in range(attempts):
        if not path.exists():
            return True
        try:
            path.unlink()
            return True
        except (PermissionError, OSError):
            if i == attempts // 2:
                kill_zombie_omnisim()
            time.sleep(delay)
    return False


def main() -> int:
    worlds = sorted([*WORLDS_DIR.glob("*.omniworld"), *WORLDS_DIR.glob("*.wbt")])
    print(f"[device-smoke] Found {len(worlds)} device worlds")
    results = []
    start = time.time()

    for i, world in enumerate(worlds, 1):
        name = world.stem
        t0 = time.time()
        print(f"[{i:2d}/{len(worlds)}] {name}...", end=" ", flush=True)

        if not clear_log_with_retry(LOG_PATH):
            print("LOG_LOCKED (skipping)")
            results.append({
                "world": name,
                "path": str(world.relative_to(REPO_ROOT)).replace("\\", "/"),
                "status": "LOG_LOCKED",
                "returncode": None,
                "elapsed_sec": round(time.time() - t0, 2),
                "error_tail": "Could not clear omnisim_log.txt — previous omnisim-bin still holds it",
            })
            RESULTS_PATH.write_text(json.dumps(results, indent=2))
            time.sleep(2.0)
            continue

        try:
            proc = subprocess.run(
                [sys.executable, str(RUNNER), str(world), "--duration", str(DURATION)],
                cwd=REPO_ROOT,
                timeout=TIMEOUT,
                capture_output=True,
                text=True,
            )
            elapsed = time.time() - t0
            status = "PASS" if proc.returncode == 0 else "FAIL"
            err = (proc.stderr or proc.stdout or "")[-400:]
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            elapsed = time.time() - t0
            status = "TIMEOUT"
            err = f"exceeded {TIMEOUT}s"
            rc = None
        except Exception as e:
            elapsed = time.time() - t0
            status = "ERROR"
            err = repr(e)[:400]
            rc = None

        print(f"{status} ({elapsed:.1f}s)")
        results.append({
            "world": name,
            "path": str(world.relative_to(REPO_ROOT)).replace("\\", "/"),
            "status": status,
            "returncode": rc,
            "elapsed_sec": round(elapsed, 2),
            "error_tail": err.strip(),
        })

        RESULTS_PATH.write_text(json.dumps(results, indent=2))
        kill_zombie_omnisim()
        time.sleep(1.0)

    total = time.time() - start
    pass_count = sum(1 for r in results if r["status"] == "PASS")
    print(f"\n[device-smoke] Done in {total:.0f}s — {pass_count}/{len(results)} PASS")
    print(f"[device-smoke] Results: {RESULTS_PATH}")
    return 0 if pass_count == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
