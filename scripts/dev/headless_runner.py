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

"""Run a world headlessly, monitor log output, exit after duration or on error.

Usage:
    python scripts/dev/headless_runner.py <world> [--duration 10] [--fail-on-warning]

This is the supported headless execution contract for OmniSim.
It starts the simulator in batch mode, monitors omnisim_log.txt for errors,
runs for the specified duration, then terminates and reports results.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def find_binary(webots_home: Path) -> Path:
    if sys.platform == "win32":
        candidates = [
            # Canonical post-Phase-B name first; legacy omnisim-bin.exe is
            # the same byte content (alias) but acts as fallback if the
            # post-build alias step hasn't run on this checkout yet.
            webots_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            webots_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            webots_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [
            webots_home / "Contents" / "MacOS" / "omnisim",
            webots_home / "Contents" / "MacOS" / "webots",
        ]
    else:
        candidates = [
            webots_home / "bin" / "omnisim-bin",
            webots_home / "bin" / "omnisim-bin",
            webots_home / "omnisim",
            webots_home / "webots",
        ]

    for c in candidates:
        if c.exists():
            return c
    raise SystemExit(f"Cannot find simulator binary under {webots_home}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a world headlessly and report results")
    parser.add_argument("world", help="Path to .wbt world file")
    parser.add_argument("--duration", type=int, default=10, help="Seconds to run before stopping (default: 10)")
    parser.add_argument("--fail-on-warning", action="store_true", help="Treat warnings as failures")
    parser.add_argument("--require-newton", action="store_true",
                        help="Set OMNISIM_REQUIRE_NEWTON=1 so the engine FAILS LOUDLY (non-zero exit) "
                             "if the Newton runtime can't initialise, instead of silently falling back "
                             "to ODE. Use for Newton deploy/CI runs that must assert Newton is active.")
    parser.add_argument("--cold", action="store_true",
                        help="Disable the one-time warm-up reload and test the RAW "
                             "cold first-load (which can behave differently from a "
                             "stabilised/post-reload session). See the cold-start "
                             "note printed at startup.")
    parser.add_argument("--performance-log", default="", help="Path to write performance log")
    parser.add_argument("--gui", action="store_true",
                        help="Open a REAL window (drop --minimize/--batch/--no-rendering) so "
                             "you can WATCH the run, while keeping the same reliable env "
                             "propagation (os.environ.copy -> Popen) the headless path uses. "
                             "Use to demo a Newton policy on the engine it was trained on.")
    parser.add_argument("--realtime", action="store_true",
                        help="Pace the simulation at 1.0x wall clock (--mode=realtime) "
                             "instead of as-fast-as-possible. Use with --gui so motion "
                             "plays at true speed now that Newton runs faster than "
                             "realtime.")
    parser.add_argument("--port", type=int, default=0,
                        help="Bind the extern-controller / robot-window server to this "
                             "port (passed through as --port=N). REQUIRED to run two "
                             "instances at once: a second instance otherwise collides "
                             "on the default 1234, falls back to 1235, and the GUI "
                             "tears down. 0 (default) leaves the port unset.")
    args = parser.parse_args()

    # OMNISIM_HOME is canonical. WEBOTS_HOME (the upstream name) is honoured
    # only when it actually points at an OmniSim checkout: many machines
    # carry a stale system WEBOTS_HOME from an old Webots install, and
    # silently resolving the binary there ran the wrong simulator.
    home_env = os.environ.get("OMNISIM_HOME")
    if not home_env:
        legacy = os.environ.get("WEBOTS_HOME")
        if legacy and (Path(legacy) / "scripts" / "dev" / "omnisim_dev.py").exists():
            home_env = legacy
        else:
            if legacy:
                print(f"[headless] note: ignoring WEBOTS_HOME={legacy} "
                      f"(not an OmniSim checkout); using this repo root.",
                      file=sys.stderr)
            home_env = str(REPO_ROOT)
    webots_home = Path(home_env)
    binary = find_binary(webots_home)
    world = Path(args.world)
    if not world.is_absolute():
        world = REPO_ROOT / world
    if not world.exists():
        raise SystemExit(f"World not found: {world}")

    # Use OMNISIM_LOG_PATH if set (per the parallel-runs convention in
    # AGENTS.md §3e); otherwise default to omnisim_log.txt at OMNISIM_HOME.
    env_log = os.environ.get("OMNISIM_LOG_PATH")
    log_path = Path(env_log) if env_log else webots_home / "omnisim_log.txt"

    # Clear old log. Tolerate WinError 32 (file locked by another process —
    # typically the user's interactive OmniSim holding the default log open):
    # in that case we honor OMNISIM_LOG_PATH for our own child and skip the
    # default-path cleanup.
    if log_path.exists():
        try:
            log_path.unlink()
        except PermissionError:
            print(f"[headless] {log_path} is locked by another process — "
                  f"set OMNISIM_LOG_PATH to a unique path for this run.",
                  file=sys.stderr)
            return 2

    # `--minimize` keeps the main window in a normal Qt event loop while
    # hiding it via the OS taskbar minimize. The alternative `--no-window`
    # mode skips main-window realization entirely, but Newton's embedded
    # CPython FFI deadlocks at the first few add_joint_revolute calls
    # under --no-window (confirmed for G1 + 20-husky multi-articulation
    # worlds — see engine-migration-plan.md §15 entry 2026-05-28). Until
    # the underlying main-loop / Python-event-loop interaction is fixed,
    # --minimize is the safe default for headless runs that involve
    # Newton-backed Solids. `--batch --no-rendering` keep the run cheap
    # despite the (minimized) window being technically present.
    mode = "--mode=realtime" if args.realtime else "--mode=fast"
    if args.gui:
        # Visible window: drop --minimize/--batch/--no-rendering so the 3D view
        # realises. Newton works in a full window (only --no-window deadlocks it).
        cmd = [str(binary), str(world), mode, "--stdout", "--stderr"]
    else:
        cmd = [
            str(binary),
            str(world),
            "--minimize",
            "--batch",
            "--no-rendering",
            mode,
            "--stdout",
            "--stderr",
        ]
    if args.port:
        cmd.append(f"--port={args.port}")
    if args.performance_log:
        cmd.append(f"--log-performance={args.performance_log}")

    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(webots_home)
    # The GUI/render path resolves Qt plugins + render resources via WEBOTS_HOME;
    # without it a --gui launch crashes during render init (Qt teardown) even
    # though the headless --no-rendering path is fine. Point it at THIS checkout
    # (also overrides any stale system WEBOTS_HOME from an old Webots install).
    env["WEBOTS_HOME"] = str(webots_home)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    if args.require_newton:
        # Assert Newton actually initialises -- the engine WbLog::fatal()s (non-zero
        # exit) instead of silently degrading to ODE. Guards against the silent-ODE-
        # fallback class of bug (e.g. the warp-banner-vs-DEVNULL FFI-smoke failure).
        env["OMNISIM_REQUIRE_NEWTON"] = "1"
        print("[headless] REQUIRE-NEWTON: engine will fail loudly if Newton can't init "
              "(no silent ODE fallback).")

    # ── COLD-START TRAP (RESOLVED) ────────────────────────────────────────────
    # Historically a fresh cold first-load under-tracked the Newton/MuJoCo articulation
    # (~1 cm), so precise grasps failed cold but worked after a reload -- headless runs
    # therefore defaulted to a warm-up reload. That bug is FIXED (verified 2026-07-05:
    # cold == warm bit-identical; root fix eb86f888 + finalize-time solver re-assert;
    # see docs/developer/real-grasp-and-the-cold-first-load-trap.md), so warmup_reload
    # is now a no-op by default and there is nothing to warm. --cold is kept for
    # explicitness (forces the workaround fully off even if someone set FORCE_WARMUP).
    if args.cold:
        env["OMNISIM_NO_WARMUP"] = "1"
        print("[headless] COLD MODE: warm-up reload force-disabled (the cold-load "
              "under-track is fixed, so this matches the default behaviour).")
    if sys.platform == "win32":
        # Controllers (generic.exe et al.) are MinGW-built and load
        # libgcc_s_seh-1.dll / libstdc++-6.dll / libwinpthread-1.dll at launch.
        # Those live in msys64\mingw64\bin, so that dir MUST be on the child's
        # PATH or the controller dies with "DLL not found". binary.parent
        # already covers it when the canonical mingw64\bin binary resolves, but
        # inject it explicitly so a fallback binary path can't drop the runtime.
        mingw_bin = webots_home / "msys64" / "mingw64" / "bin"
        path_prefix = [str(binary.parent)]
        if mingw_bin.is_dir() and mingw_bin != binary.parent:
            path_prefix.append(str(mingw_bin))
        env["PATH"] = ";".join(path_prefix) + ";" + env.get("PATH", "")

    print(f"[headless] Starting: {world.name}")
    print(f"[headless] Duration: {args.duration}s")
    print(f"[headless] Binary: {binary}")

    # 2026-05-28: route stdout/stderr to DEVNULL instead of PIPE. PIPE
    # buffers fill at ~64 KB on Windows, and once they're full the
    # simulator's writes block — which manifests as the world load
    # apparently stalling past 20 huskies. We read the diagnostic log
    # via the omnisim_log.txt file anyway, so the simulator's stdout
    # writes are pure noise from our perspective.
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for the specified duration, checking if the process exits early
    start = time.time()
    while time.time() - start < args.duration:
        ret = proc.poll()
        if ret is not None:
            if ret != 0:
                print(f"[headless] FAIL: simulator exited early with code {ret}")
                return 1
            else:
                print(f"[headless] Simulator exited cleanly before timeout")
                break
        time.sleep(0.5)
    else:
        # Duration elapsed, terminate
        print(f"[headless] Duration reached, stopping simulator...")
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    # Analyze log
    errors = 0
    warnings = 0
    if log_path.exists():
        content = log_path.read_text(errors="replace")
        for line in content.splitlines():
            if line.startswith("ERROR:") or line.startswith("FATAL:"):
                errors += 1
                print(f"[headless]   {line}")
            elif line.startswith("WARNING:"):
                warnings += 1
                if args.fail_on_warning:
                    print(f"[headless]   {line}")
    else:
        # No log means the simulator never started (bad DLL search path,
        # wrong binary, instant crash). A PASS here would be a verdict
        # with zero evidence — fail loudly instead.
        print("[headless] FAIL: no log file produced — the simulator never "
              "started or wrote nothing. Check the binary, PATH/DLLs and "
              "OMNISIM_LOG_PATH.")
        return 1

    print(f"[headless] Results: {errors} errors, {warnings} warnings")

    if errors > 0:
        print("[headless] FAIL")
        return 1
    if args.fail_on_warning and warnings > 0:
        print("[headless] FAIL (warnings treated as errors)")
        return 1

    print("[headless] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
