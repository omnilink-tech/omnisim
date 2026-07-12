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

"""End-to-end deploy smoke test.

After training + ONNX export, this script:
  1. Verifies the ONNX policy loads via onnxruntime
  2. Launches the spot_rl_deploy world headlessly with SPOT_SIM_DURATION_S
  3. Reads the body's trajectory CSV from C:\\tmp\\husky_trace\\spot_deploy_*.log
  4. Prints summary: how long did the policy keep Spot upright? Did it walk?

Run from repo root:
    python projects/policies/research/tools/smoke_deploy.py \
        --policy projects/policies/research/inference/policies/spot_ppo_main/policy.onnx \
        --sim-duration 15
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
WORLD = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl_deploy.wbt"
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
TRACE_DIR = Path(r"C:\tmp\omnisim_rl_deploy")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--sim-duration", type=float, default=15.0)
    args = p.parse_args()

    if not args.policy.exists():
        print(f"[smoke_deploy] policy not found: {args.policy}")
        return 1

    print(f"[smoke_deploy] policy: {args.policy} ({args.policy.stat().st_size} bytes)")
    # Onnxruntime quick load
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
        inp = sess.get_inputs()[0]
        out = sess.get_outputs()[0]
        print(f"[smoke_deploy] ONNX inputs={inp.shape} -> outputs={out.shape}")
        test = sess.run(None, {inp.name: np.zeros((1, 49), dtype=np.float32)})
        print(f"[smoke_deploy] dummy inference: action[0:6] = {test[0][0][:6].round(3).tolist()}")
    except Exception as e:
        print(f"[smoke_deploy] ONNX load failed: {e}")
        return 1

    TRACE_DIR.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["SPOT_POLICY_ONNX"] = str(args.policy)
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(TRACE_DIR / "deploy.log")
    env["SPOT_DEPLOY_TRACE"] = "1"
    # Also clear stale trace file
    try:
        Path(r"C:\tmp\husky_trace\spot_deploy.csv").unlink(missing_ok=True)
    except Exception:
        pass

    cmd = [
        str(OMNISIM_BIN), str(WORLD),
        "--batch", "--mode=fast", "--no-rendering", "--minimize",
        "--stdout", "--stderr",
    ]
    print(f"[smoke_deploy] launching: {WORLD.name}")
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    start = time.time()
    deadline = start + args.sim_duration + 10
    while time.time() < deadline:
        if proc.poll() is not None:
            break
        time.sleep(0.2)
    if proc.poll() is None:
        # Kill via psutil so the controller child also dies.
        try:
            import psutil
            parent = psutil.Process(proc.pid)
            for c in parent.children(recursive=True):
                try:
                    c.kill()
                except Exception:
                    pass
            parent.kill()
        except Exception:
            proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            pass
    elapsed = time.time() - start
    print(f"[smoke_deploy] OmniSim exited after {elapsed:.1f} s wall")

    log = TRACE_DIR / "deploy.log"
    if log.exists():
        text = log.read_text(errors="replace")
        errors = sum(1 for ln in text.splitlines() if ln.startswith("ERROR:") or ln.startswith("FATAL:"))
        print(f"[smoke_deploy] deploy log: {len(text.splitlines())} lines, {errors} errors")
        if errors:
            for ln in text.splitlines():
                if ln.startswith("ERROR:") or ln.startswith("FATAL:"):
                    print(f"  {ln}")

    # Analyze body trace if produced.
    csv = Path(r"C:\tmp\husky_trace\spot_deploy.csv")
    if csv.exists():
        import csv as csvlib
        rows = list(csvlib.DictReader(csv.open()))
        if rows:
            def f(s): return float(s)
            tN = f(rows[-1]["t_ms"]) / 1000.0
            bx_start = f(rows[0]["bx"]); bx_end = f(rows[-1]["bx"])
            by_start = f(rows[0]["by"]); by_end = f(rows[-1]["by"])
            bz_min = min(f(r["bz"]) for r in rows)
            bz_end = f(rows[-1]["bz"])
            upright = sum(1 for r in rows if f(r["bz"]) > 0.3 and abs(f(r["roll"])) < 0.5 and abs(f(r["pitch"])) < 0.5)
            print(f"[smoke_deploy] trajectory: {len(rows)} samples over {tN:.2f}s")
            print(f"  net dx = {bx_end - bx_start:+.3f} m   net dy = {by_end - by_start:+.3f} m   final bz = {bz_end:.3f}")
            print(f"  min bz = {bz_min:.3f}   upright (bz>0.3, |rp|<0.5) = {100*upright/len(rows):.0f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
