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

"""Measure the engine STARTUP-RACE rate: N back-to-back headless launches, counted.

The race (public issue #3; `looks_like_startup_race` in headless_runner.py) is a
launch that dies during engine startup with the Qt teardown signature
(`QWaitCondition: Destroyed while threads are still waiting` /
`QThreadStorage: entry`), a non-zero exit, and NOT ONE ERROR/FATAL line. Its
rate was recorded as "roughly one launch in three" (AgentBench adapter,
2026-07) and 1-of-19 / 3-of-10 (batch_validate.py, 2026-08-15) -- all on
machine 9722d23d12a3, all on older binaries -- and re-measured as 0 of 80 on
the 2026-08-29 binary (25 raw + 25 raw-with-overlapping-teardown + 15 runner
+ 15 runner-heavier-worlds). A rate is only meaningful for a NAMED binary on a
NAMED machine, which is why this script prints both.

    python scripts/dev/launch_race_stress.py                     # 25 raw launches
    python scripts/dev/launch_race_stress.py -n 50 --overlap     # next engine starts while the previous tears down
    python scripts/dev/launch_race_stress.py --via-runner        # through run-headless --until-finalized (retries OFF)
    python scripts/dev/launch_race_stress.py --world a.omniworld --world b.omniworld   # rotate worlds

Exit code is 0 when no launch raced, 75 (EX_TEMPFAIL, the runner's own race
code) when at least one did, 1 when a launch failed for any OTHER reason -- a
world that genuinely fails to load must never be counted as "the flake".
Every launch gets its own OMNISIM_LOG_PATH under --out so the logs survive.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))
from headless_runner import EXIT_STARTUP_RACE, find_binary, looks_like_startup_race  # noqa: E402

DEFAULT_WORLD = "projects/samples/demos/worlds/physics/newton_smoke_test.omniworld"
HEADLESS_FLAGS = ["--minimize", "--batch", "--no-rendering", "--mode=fast", "--stdout", "--stderr"]


def _sha256_prefix(path: Path, n: int = 16) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:n]


def _env(log_path: Path) -> dict:
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    env["PATH"] = str(REPO_ROOT / "msys64" / "mingw64" / "bin") + os.pathsep + env.get("PATH", "")
    return env


def launch_raw(binary: Path, world: Path, log_path: Path, timeout_s: float, overlap: bool) -> dict:
    sidecar = Path(str(log_path) + ".newton.json")
    for p in (log_path, sidecar):
        if p.exists():
            p.unlink()
    t0 = time.time()
    proc = subprocess.Popen([str(binary), str(world)] + HEADLESS_FLAGS, env=_env(log_path), cwd=str(REPO_ROOT),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rc = None
    finalised = False
    while time.time() - t0 < timeout_s:
        rc = proc.poll()
        if rc is not None:
            break
        if sidecar.exists():
            finalised = True
            break
        time.sleep(0.2)
    if rc is None:
        proc.terminate()
        # --overlap: let the NEXT engine start while this one is still tearing
        # down (the shape a batch runner that kills on timeout produces).
        try:
            proc.wait(0.05 if overlap else 15)
        except subprocess.TimeoutExpired:
            if not overlap:
                proc.kill()
    return dict(rc=rc, finalised=finalised, secs=round(time.time() - t0, 2))


def launch_via_runner(world: Path, log_path: Path, timeout_s: float) -> dict:
    t0 = time.time()
    cmd = [sys.executable, str(REPO_ROOT / "scripts" / "dev" / "headless_runner.py"), str(world),
           "--until-finalized", "--race-attempts", "1"]
    try:
        res = subprocess.run(cmd, env=_env(log_path), cwd=str(REPO_ROOT), capture_output=True, text=True,
                             timeout=timeout_s)
        rc, out = res.returncode, res.stdout + res.stderr
    except subprocess.TimeoutExpired:
        rc, out = None, ""
    tail = out.strip().splitlines()[-1] if out.strip() else ""
    return dict(rc=rc, finalised=(rc == 0 and tail.endswith("PASS")), secs=round(time.time() - t0, 2),
                runner_tail=tail)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--launches", type=int, default=25)
    ap.add_argument("--world", action="append", help="world(s) to rotate through (default: newton_smoke_test)")
    ap.add_argument("--overlap", action="store_true",
                    help="start the next engine while the previous is still tearing down")
    ap.add_argument("--via-runner", action="store_true",
                    help="launch through headless_runner.py --until-finalized (its retry OFF)")
    ap.add_argument("--gap", type=float, default=0.0, help="seconds to sleep between launches (0 = back-to-back)")
    ap.add_argument("--timeout", type=float, default=90.0, help="per-launch ceiling in seconds")
    ap.add_argument("--out", default=".local-runs/launch_race_stress",
                    help="directory for per-launch logs + summary.json")
    args = ap.parse_args()

    binary = find_binary(REPO_ROOT)
    worlds = [REPO_ROOT / w for w in (args.world or [DEFAULT_WORLD])]
    for w in worlds:
        if not w.exists():
            print(f"world not found: {w}", file=sys.stderr)
            return 2
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    provenance = dict(binary=str(binary), binary_sha256_16=_sha256_prefix(binary),
                      binary_mtime=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(binary.stat().st_mtime)),
                      machine=platform.node(), platform=platform.platform(),
                      mode="runner" if args.via_runner else ("raw+overlap" if args.overlap else "raw"),
                      gap_s=args.gap, worlds=[str(w.relative_to(REPO_ROOT)) for w in worlds])
    print("launch_race_stress:", json.dumps(provenance), flush=True)

    rows = []
    for i in range(args.launches):
        world = worlds[i % len(worlds)]
        log_path = out / f"launch{i:03d}.log"
        if args.via_runner:
            row = launch_via_runner(world, log_path, args.timeout)
        else:
            row = launch_raw(binary, world, log_path, args.timeout, args.overlap)
        text = log_path.read_text(errors="replace") if log_path.exists() else ""
        row.update(i=i, world=world.name, raced=(looks_like_startup_race(text) and not row["finalised"]),
                   error_lines=sum(1 for ln in text.splitlines() if ln.startswith(("ERROR:", "FATAL:"))),
                   log_lines=len(text.splitlines()))
        rows.append(row)
        print(json.dumps(row), flush=True)
        if args.gap:
            time.sleep(args.gap)

    raced = sum(1 for r in rows if r["raced"])
    finalised = sum(1 for r in rows if r["finalised"])
    other_fail = sum(1 for r in rows if not r["finalised"] and not r["raced"])
    summary = dict(provenance=provenance, launches=len(rows), finalised=finalised, raced=raced,
                   failed_other=other_fail, race_rate=(raced / len(rows)) if rows else None)
    (out / "summary.json").write_text(json.dumps(dict(summary=summary, rows=rows), indent=1))
    print(f"SUMMARY launches={len(rows)} finalised={finalised} raced={raced} failed_other={other_fail} "
          f"binary={provenance['binary_sha256_16']} machine={provenance['machine']}")
    if other_fail:
        return 1
    return EXIT_STARTUP_RACE if raced else 0


if __name__ == "__main__":
    sys.exit(main())
