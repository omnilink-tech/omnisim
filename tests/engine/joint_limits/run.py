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

"""Launch the joint-limit stress test headlessly and report PASS/FAIL.

This is the standalone runner for the engine joint-limit guarantee
test. It is intentionally separate from the RL pipeline.

Exit codes:
  0 - no joint ever exceeded its URDF velocity_limit or position range
  1 - one or more joints violated their URDF limits (see result file)
  2 - test infrastructure failure (simulator didn't start, timeout, etc.)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
WORLD = REPO_ROOT / "tests" / "engine" / "joint_limits" / "worlds" / "stress.omniworld"


def find_binary() -> Path:
    if sys.platform == "win32":
        # omnisim-bin.exe only -- no webots-bin.exe fallback. No Makefile
        # produces that name any more, so a leftover copy is a stale artefact;
        # falling back to it would silently run a weeks-old binary.
        candidates = [
            REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [REPO_ROOT / "Contents" / "MacOS" / "omnisim"]
    else:
        candidates = [REPO_ROOT / "bin" / "omnisim-bin"]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError(
        f"OmniSim binary not found under {REPO_ROOT}/msys64/mingw64/bin/")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="Wallclock timeout in seconds (sim runs ~11s, fast mode much faster)")
    ap.add_argument("--keep-result", action="store_true",
                    help="Don't delete the result file after parsing")
    args = ap.parse_args()

    binary = find_binary()
    with tempfile.NamedTemporaryFile(prefix="omnisim_joint_limits_",
                                     suffix=".txt", delete=False) as tf:
        result_path = Path(tf.name)
    if result_path.exists():
        result_path.unlink()

    env = os.environ.copy()
    env["OMNISIM_TEST_RESULT_FILE"] = str(result_path)
    # Force MuJoCo solver under Newton. XPBD doesn't populate
    # state.joint_q each step (no eval_ik), so the joint-angle
    # readback path used by HingeJoint::position() returns 0 and
    # the test can't observe joint state. The documented
    # joint-velocity-limit violations are MuJoCo-specific anyway.
    env.setdefault("OMNISIM_NEWTON_FORCE_MUJOCO", "1")
    diag_path = result_path.with_suffix(".diag.txt")
    if diag_path.exists():
        diag_path.unlink()
    env["OMNISIM_TEST_DIAG_FILE"] = str(diag_path)

    # The controller's runtime.ini says COMMAND=python. On Windows, plain
    # "python" often hits the Microsoft Store stub. Inject the real
    # interpreter's directory (the one running THIS launcher) at the front
    # of PATH so the controller starts. Also inject msys64\mingw64\bin so
    # the generic controller shim's MinGW DLLs resolve (matches the
    # headless_runner convention).
    if sys.platform == "win32":
        python_dir = str(Path(sys.executable).parent)
        mingw_bin = str(REPO_ROOT / "msys64" / "mingw64" / "bin")
        env["PATH"] = python_dir + os.pathsep + mingw_bin + os.pathsep + env.get("PATH", "")

    cmd = [str(binary), "--batch", "--mode=fast", "--no-rendering",
           "--no-window", "--stdout", "--stderr", str(WORLD)]
    sys.stdout.write(f"[run] {' '.join(cmd)}\n")
    sys.stdout.write(f"[run] result file: {result_path}\n")
    sys.stdout.flush()

    t0 = time.time()
    sim_log_path = result_path.with_suffix(".simlog.txt")
    try:
        with open(sim_log_path, "wb") as log_fh:
            proc = subprocess.run(cmd, env=env, timeout=args.timeout,
                                  cwd=str(REPO_ROOT),
                                  stdout=log_fh, stderr=subprocess.STDOUT)
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        sys.stdout.write(f"[run] TIMEOUT after {args.timeout}s\n")
        rc = None
    wall = time.time() - t0
    sys.stdout.write(f"[run] simulator exited rc={rc} in {wall:.1f}s wall\n")
    sys.stdout.write(f"---- sim stdout/stderr (tail) ----\n")
    try:
        with open(sim_log_path, "r", errors="replace") as f:
            tail = f.read().splitlines()[-60:]
            sys.stdout.write("\n".join(tail) + "\n")
    except Exception as e:
        sys.stdout.write(f"(read failed: {e})\n")
    sys.stdout.write("----------------------------------\n")
    if rc is None:
        return 2

    if not result_path.exists():
        sys.stdout.write("[run] no result file produced -- test infrastructure failure\n")
        if diag_path.exists():
            sys.stdout.write("---- controller diag ----\n")
            sys.stdout.write(diag_path.read_text())
            sys.stdout.write("-------------------------\n")
        return 2

    result_text = result_path.read_text()
    sys.stdout.write("---- result ----\n")
    sys.stdout.write(result_text)
    sys.stdout.write("----------------\n")

    if not args.keep_result:
        try:
            result_path.unlink()
        except Exception:
            pass

    verdict = "FAIL"
    for ln in result_text.splitlines():
        if ln.startswith("verdict="):
            verdict = ln.split("=", 1)[1].strip()
            break
    sys.stdout.write(f"[run] verdict={verdict}\n")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
