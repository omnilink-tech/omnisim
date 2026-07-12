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

"""Headless eval of a deployed RL policy.

Launches the deploy world with SPOT_DEPLOY_TRACE=1, runs N seconds,
then summarizes the trace: forward distance, mean velocity, uprightness,
fall/no-fall.

Usage:
    python projects/policies/research/inference/eval_policy.py \
        --policy projects/policies/research/inference/policies/main/policy.onnx \
        --duration 20 \
        [--vx 0.4]
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
DEPLOY_WORLD = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl_deploy.wbt"
TRACE_PATH = Path(r"C:\tmp\husky_trace\spot_deploy.csv")


def kill_owned_tree(proc: subprocess.Popen | None) -> None:
    """Kill ONLY the Webots process we launched, plus its children
    (omnisim-bin spawns python.exe as the controller). We must never run a
    blanket `Get-Process omnisim-bin | Stop-Process` -- a concurrent Claude
    Code session may share this box, and a blanket kill takes down its live
    Webots too (see session_isolation.py / feedback_concurrent_sessions)."""
    if proc is None:
        return
    try:
        import psutil
        try:
            parent = psutil.Process(proc.pid)
            for child in parent.children(recursive=True):
                try:
                    child.kill()
                except Exception:
                    pass
            parent.kill()
        except psutil.NoSuchProcess:
            pass
    except ImportError:
        try:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception:
            pass
    try:
        proc.wait(timeout=3)
    except Exception:
        pass


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--duration", type=float, default=20.0, help="sim seconds")
    p.add_argument("--vx", type=float, default=0.4)
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--wz", type=float, default=0.0)
    args = p.parse_args()

    if not args.policy.exists():
        print(f"policy not found: {args.policy}", file=sys.stderr)
        return 1

    TRACE_PATH.parent.mkdir(parents=True, exist_ok=True)
    if TRACE_PATH.exists():
        TRACE_PATH.unlink()

    # Webots strips env vars from the controller subprocess in some configs,
    # so we ALSO copy the policy + a 'current_command.txt' to canonical
    # locations that the deploy controller reads directly.
    import shutil
    canonical_dir = REPO_ROOT / "projects" / "rl" / "inference" / "policies" / "spot_ppo_main"
    canonical_dir.mkdir(parents=True, exist_ok=True)
    canonical_policy = canonical_dir / "policy.onnx"
    if canonical_policy.resolve() != args.policy.resolve():
        shutil.copy2(args.policy, canonical_policy)
    cmd_file = REPO_ROOT / "projects" / "rl" / "inference" / "current_command.txt"
    cmd_file.write_text(f"{args.vx}\n{args.vy}\n{args.wz}\n")

    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = r"C:\tmp\spot_eval_log.txt"
    # Use absolute path for the policy env var (some setups have the
    # controller's cwd elsewhere so relative paths break).
    env["SPOT_POLICY_ONNX"] = str(args.policy.resolve())
    env["SPOT_DEPLOY_TRACE"] = "1"
    env["SPOT_VX"] = str(args.vx)
    env["SPOT_VY"] = str(args.vy)
    env["SPOT_WZ"] = str(args.wz)
    if sys.platform == "win32":
        env["PATH"] = str(OMNISIM_BIN.parent) + ";" + env.get("PATH", "")

    cmd = [
        str(OMNISIM_BIN),
        str(DEPLOY_WORLD),
        "--batch", "--mode=fast", "--no-rendering", "--minimize",
        "--stdout", "--stderr",
    ]
    print(f"[eval] launching OmniSim, duration={args.duration}s sim, cmd={args.vx},{args.vy},{args.wz}")
    eval_stdout_log = Path(r"C:\tmp\spot_eval_stdout.log")
    eval_stdout_log.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(eval_stdout_log, "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log_fh, stderr=subprocess.STDOUT)
    # Wait wall-clock duration*2 (fast mode does ~20x realtime, but allow margin)
    wall_budget = max(8.0, args.duration / 2.0)
    time.sleep(wall_budget)
    kill_owned_tree(proc)
    time.sleep(0.5)

    if not TRACE_PATH.exists():
        print("[eval] no trace produced; see C:/tmp/spot_eval_log.txt")
        return 1

    import csv
    with open(TRACE_PATH) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("[eval] empty trace")
        return 1

    def F(r, k): return float(r[k])

    t_ms = [F(r, "t_ms") for r in rows]
    bx = [F(r, "bx") for r in rows]
    by = [F(r, "by") for r in rows]
    bz = [F(r, "bz") for r in rows]
    roll = [F(r, "roll") for r in rows]
    pitch = [F(r, "pitch") for r in rows]
    yaw = [F(r, "yaw") for r in rows]

    dur_s = (t_ms[-1] - t_ms[0]) / 1000.0
    dx = bx[-1] - bx[0]
    dy = by[-1] - by[0]
    upright = sum(1 for z, r, p in zip(bz, roll, pitch)
                  if z > 0.3 and abs(r) < 0.7 and abs(p) < 0.7) / len(rows)
    fallen = any(z < 0.2 for z in bz)
    mean_vx = dx / max(dur_s, 1e-3)

    print(f"[eval] samples       : {len(rows)}")
    print(f"[eval] sim duration  : {dur_s:.2f} s")
    print(f"[eval] start pos     : ({bx[0]:+.3f}, {by[0]:+.3f}, {bz[0]:+.3f})")
    print(f"[eval] end pos       : ({bx[-1]:+.3f}, {by[-1]:+.3f}, {bz[-1]:+.3f})")
    print(f"[eval] forward dx    : {dx:+.3f} m   (target vx={args.vx})")
    print(f"[eval] lateral dy    : {dy:+.3f} m")
    print(f"[eval] mean vx       : {mean_vx:+.3f} m/s")
    print(f"[eval] upright frac  : {upright:.2f}")
    print(f"[eval] fall detected : {fallen}")
    print(f"[eval] end roll/pitch: {roll[-1]:+.2f} / {pitch[-1]:+.2f} rad   "
          f"({math.degrees(roll[-1]):+.0f} / {math.degrees(pitch[-1]):+.0f} deg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
