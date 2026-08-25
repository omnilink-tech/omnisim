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

"""Evaluate a trained policy in the Newton deploy world.

Plays the policy through projects/policies/worlds/omniquad_newton_demo.omniworld headless,
reads the body trace CSV the deploy controller writes, and reports the
same metrics pick_best_omnisim.py reports for the ODE deploy:

  - net forward distance
  - lateral drift
  - mean forward velocity
  - upright percentage
  - yaw drift
  - min body z (did the chassis sink)
  - time-to-fall (None = never fell)

Run from repo root:
    python projects/policies/research/tools/_eval_newton.py \\
        --policy projects/policies/research/inference/policies/omniquad_newton_v2/policy.onnx \\
        --duration 30 --vx 0.3
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
TRACE_PATH = Path(r"C:\tmp\husky_trace\omniquad_deploy.csv")
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
DEPLOY_WORLD = REPO_ROOT / "projects" / "rl" / "worlds" / "omniquad_newton_demo.omniworld"


def run_eval(policy_path: Path, duration_s: float, vx: float) -> dict:
    if TRACE_PATH.exists():
        TRACE_PATH.unlink()
    # MUST be absolute: Webots launches the controller with a cwd that
    # is NOT the repo root, so a relative policy path fails .exists()
    # in the controller and silently falls back to omniquad_ppo_main. That
    # bug made every trained-policy eval actually measure the default
    # policy (identical "falls at 0.688 s" traces for every checkpoint).
    policy_path = policy_path.resolve()
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["OMNIQUAD_POLICY_ONNX"] = str(policy_path)
    env["OMNISIM_POLICY_ONNX"] = str(policy_path)
    env["OMNIQUAD_DEPLOY_TRACE"] = "1"
    env["OMNISIM_DEPLOY_TRACE"] = "1"
    env["OMNIQUAD_VX"] = str(vx)
    env["OMNIQUAD_VY"] = "0.0"
    env["OMNIQUAD_WZ"] = "0.0"
    # MuJoCo CPU is the stable solver path; XPBD on GPU NaN'd out
    # under the URDF effort + inertia config for v12 deploy. Default
    # to MuJoCo here; the training run can override back to XPBD via
    # OMNIQUAD_TRAIN_WORLD + the env-var unset.
    env.setdefault("OMNISIM_NEWTON_FORCE_MUJOCO", "1")
    env.setdefault("OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE", "1")
    # CPG must match what the policy was TRAINED with. As of the
    # 1.75 Hz-CPG training recipe (train_omniquad_newton.py), the deploy
    # sidecar's 1.75 Hz trot is exactly what the policy expects, so we
    # leave the sidecar CPG in place (no override). For policies trained
    # before that (CPG off), pass OMNIQUAD_CPG_FREQ_HZ=0 in the parent.
    env.setdefault("OMNISIM_URDF_USE_INERTIA", "1")
    # Spawn the robot in its commanded standing pose (Newton seeds joints
    # at 0 = straight legs otherwise, and the fold-into-stance transient
    # tips it under MuJoCo). Override with OMNISIM_NEWTON_SEED_POSE=0.
    env.setdefault("OMNISIM_NEWTON_SEED_POSE", "1")
    env["OMNISIM_NEWTON_LOG"] = str(REPO_ROOT / ".build_tmp" / "newton_eval.log")
    cmd = [str(OMNISIM_BIN), str(DEPLOY_WORLD),
           "--batch", "--mode=fast", "--no-rendering", "--minimize",
           "--stdout", "--stderr"]
    proc = subprocess.Popen(cmd, env=env,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    deadline = time.time() + duration_s + 8
    while time.time() < deadline:
        if proc.poll() is not None:
            break
        time.sleep(0.3)
    if proc.poll() is None:
        try:
            import psutil
            parent = psutil.Process(proc.pid)
            for c in parent.children(recursive=True):
                try: c.kill()
                except Exception: pass
            parent.kill()
        except Exception:
            proc.terminate()
        try: proc.wait(timeout=4)
        except Exception: pass

    if not TRACE_PATH.exists():
        return {"error": "no trace CSV produced"}
    rows = list(csv.DictReader(TRACE_PATH.open()))
    if not rows:
        return {"error": "empty trace"}
    f = lambda s: float(s)
    bx = np.array([f(r["bx"]) for r in rows])
    by = np.array([f(r["by"]) for r in rows])
    bz = np.array([f(r["bz"]) for r in rows])
    roll = np.array([f(r["roll"]) for r in rows])
    pitch = np.array([f(r["pitch"]) for r in rows])
    yaw = np.array([f(r["yaw"]) for r in rows])
    vx_arr = np.array([f(r["vx"]) for r in rows])
    vy_arr = np.array([f(r["vy"]) for r in rows])
    # NaN guard: trace can flip to NaN after divergence.
    if np.isnan(bz).any():
        nan_at = int(np.argmax(np.isnan(bz)))
        return {"error": f"NaN at sample {nan_at} (~{rows[nan_at]['t_ms']} ms sim)"}
    duration = (f(rows[-1]["t_ms"]) - f(rows[0]["t_ms"])) / 1000.0
    fall_mask = (bz < 0.30) | (np.abs(roll) > 0.7) | (np.abs(pitch) > 0.7)
    time_to_fall = (float(rows[int(np.argmax(fall_mask))]["t_ms"]) / 1000.0
                    if fall_mask.any() else None)
    upright = ((bz > 0.30) & (np.abs(roll) < 0.5) & (np.abs(pitch) < 0.5))
    dpos = np.sqrt(np.diff(bx) ** 2 + np.diff(by) ** 2)
    return dict(
        n=len(rows),
        sim_duration=duration,
        forward_distance=float(bx[-1] - bx[0]),
        lateral_drift=float(by[-1] - by[0]),
        path_length=float(dpos.sum()),
        yaw_drift_rad=float(np.abs(yaw[-1] - yaw[0])),
        mean_vx=float(vx_arr.mean()),
        mean_speed=float(np.sqrt(vx_arr ** 2 + vy_arr ** 2).mean()),
        upright_pct=float(upright.mean()) * 100.0,
        min_bz=float(bz.min()),
        time_to_fall=time_to_fall,
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--vx", type=float, default=0.3)
    args = p.parse_args()
    if not args.policy.exists():
        print(f"[eval_newton] policy not found: {args.policy}")
        return 1
    print(f"[eval_newton] policy={args.policy.name}  duration={args.duration}s  vx={args.vx}")
    m = run_eval(args.policy, args.duration, args.vx)
    print()
    print("=== Newton deploy eval ===")
    if m.get("error"):
        print(f"  ERROR: {m['error']}")
        return 2
    print(f"  samples           : {m['n']}")
    print(f"  sim duration      : {m['sim_duration']:.1f} s")
    print(f"  forward distance  : {m['forward_distance']:+.3f} m")
    print(f"  lateral drift     : {m['lateral_drift']:+.3f} m")
    print(f"  path length       : {m['path_length']:.3f} m")
    print(f"  yaw drift         : {m['yaw_drift_rad']:+.3f} rad ({m['yaw_drift_rad']*57.3:+.1f} deg)")
    print(f"  mean vx           : {m['mean_vx']:+.3f} m/s   (target {args.vx})")
    print(f"  mean speed        : {m['mean_speed']:.3f} m/s")
    print(f"  upright pct       : {m['upright_pct']:.1f}%")
    print(f"  min body z        : {m['min_bz']:.3f} m")
    print(f"  time to fall      : {m['time_to_fall']}  (None = never fell)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
