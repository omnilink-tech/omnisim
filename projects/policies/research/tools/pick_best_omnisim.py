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

"""Pick the best checkpoint from a training run by *OmniSim deploy*
behaviour, not by training reward.

For each checkpoint .zip in a run dir, this script:
  1. Exports it to a temporary ONNX next to the .zip
  2. Launches the deploy world (`spot_rl_deploy.wbt`) headlessly with that
     ONNX as the policy
  3. Reads the body-trace CSV the deploy controller writes
  4. Computes: terminated_at, net forward distance, mean forward speed,
     yaw drift magnitude, upright fraction, min body z
  5. Scores by composite: `forward_distance × upright_frac × (1 / (1 + yaw_drift))`
     with a hard 0 score if terminated (no-fall is hard constraint).

Prints a leaderboard and optionally promotes the winner to a target dir.

Run from repo root (Windows PowerShell or Git Bash):
    python projects/policies/research/tools/pick_best_omnisim.py \\
        --run spot_omnisim_v1 \\
        --duration 30 \\
        --stride 2 \\
        --promote-to projects/policies/research/inference/policies/spot_omnisim_main

This uses the real deploy controller and the real deploy world, so the
"best" checkpoint it finds is the one that will actually look good when
you launch the GUI later. (Lesson learned: training reward and headless
MuJoCo eval are not enough — OmniSim ODE physics matters and you have
to measure there.)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
TRACE_PATH = Path(r"C:\tmp\husky_trace\spot_deploy.csv")
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
DEPLOY_WORLD = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl_deploy.wbt"


def export_zip_to_onnx(zip_path: Path) -> Path:
    """Re-export the checkpoint to an ONNX next to the .zip if missing."""
    onnx_path = zip_path.with_suffix(".onnx")
    if onnx_path.exists():
        return onnx_path
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    cmd = [
        sys.executable,
        str(REPO_ROOT / "projects" / "rl" / "inference" / "export_onnx.py"),
        "--model", str(zip_path),
        "--out", str(onnx_path),
    ]
    subprocess.run(cmd, env=env, capture_output=True, text=True, check=True)
    return onnx_path


def write_sidecar_for(onnx_path: Path):
    """Drop a robot_spec.json next to the ONNX matching the current
    SPOT runtime spec, so the deploy controller picks up the same
    motor_pid + action_scale + CPG values the training-side env used."""
    sys.path.insert(0, str(REPO_ROOT))
    from projects.policies.research.backends.spot_robot_spec import SPOT
    import dataclasses
    from pathlib import Path as _P
    spec = dataclasses.replace(
        SPOT,
        urdf_path=REPO_ROOT / "projects" / "robots" / "boston_dynamics" / "spot" / "urdf" / "spot.urdf",
    )
    spec.write_sidecar(onnx_path.parent)


def run_one_deploy(onnx_path: Path, duration_s: float, vx_target: float) -> dict:
    """Spawn Webots headless, point its deploy controller at this ONNX,
    wait, parse trace CSV."""
    if TRACE_PATH.exists():
        TRACE_PATH.unlink()
    env = os.environ.copy()
    env["SPOT_POLICY_ONNX"] = str(onnx_path)
    env["OMNISIM_POLICY_ONNX"] = str(onnx_path)
    env["SPOT_DEPLOY_TRACE"] = "1"
    env["SPOT_VX"] = str(vx_target)
    env["SPOT_VY"] = "0.0"
    env["SPOT_WZ"] = "0.0"
    cmd = [
        str(OMNISIM_BIN), str(DEPLOY_WORLD),
        "--batch", "--mode=fast", "--no-rendering", "--minimize",
        "--stdout", "--stderr",
    ]
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
    vx = np.array([f(r["vx"]) for r in rows])
    vy = np.array([f(r["vy"]) for r in rows])

    duration = (f(rows[-1]["t_ms"]) - f(rows[0]["t_ms"])) / 1000.0
    # path-length: sum of |Δpos| step-to-step
    dpos = np.sqrt(np.diff(bx) ** 2 + np.diff(by) ** 2)
    path_length = float(dpos.sum())
    forward_distance = float(bx[-1] - bx[0])
    lateral_drift = float(by[-1] - by[0])
    yaw_drift = float(np.abs(yaw[-1] - yaw[0]))
    mean_speed = float(np.sqrt(vx ** 2 + vy ** 2).mean())
    upright = ((bz > 0.45) & (np.abs(roll) < 0.5) & (np.abs(pitch) < 0.5))
    upright_pct = float(upright.mean()) * 100.0
    fall_mask = (bz < 0.30) | (np.abs(roll) > 0.7) | (np.abs(pitch) > 0.7)
    time_to_fall = (float(rows[int(np.argmax(fall_mask))]["t_ms"]) / 1000.0
                    if fall_mask.any() else None)

    return dict(
        n=len(rows),
        duration=duration,
        forward_distance=forward_distance,
        lateral_drift=lateral_drift,
        path_length=path_length,
        yaw_drift_rad=yaw_drift,
        mean_speed=mean_speed,
        mean_vx=float(vx.mean()),
        upright_pct=upright_pct,
        min_bz=float(bz.min()),
        time_to_fall=time_to_fall,
    )


def score(m: dict) -> float:
    """High = good. Composite that requires no-fall + forward motion +
    minimal yaw drift."""
    if m.get("error") or m.get("time_to_fall") is not None:
        return -1.0
    forward = max(0.0, m["forward_distance"])
    upright = m["upright_pct"] / 100.0
    yaw_pen = 1.0 / (1.0 + m["yaw_drift_rad"])  # 1.0 = no drift, 0.5 = 1 rad drift
    return forward * upright * yaw_pen


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True, help="run name under projects/policies/research/training/runs/")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--stride", type=int, default=1, help="eval every Nth checkpoint")
    p.add_argument("--promote-to", type=Path, default=None)
    args = p.parse_args()

    ckpt_dir = REPO_ROOT / "projects" / "rl" / "training" / "runs" / args.run / "checkpoints"
    if not ckpt_dir.exists():
        print(f"[pick] no checkpoint dir at {ckpt_dir}")
        return 1
    ckpts = sorted(ckpt_dir.glob("*.zip"), key=lambda q: int(q.stem.split("_")[-2]))
    ckpts = ckpts[::args.stride]
    if not ckpts:
        print(f"[pick] no checkpoints in {ckpt_dir}")
        return 1

    print(f"[pick] {len(ckpts)} checkpoints to eval at {args.duration}s each "
          f"in OmniSim deploy")

    rows = []
    for zp in ckpts:
        step = int(zp.stem.split("_")[-2])
        print(f"[pick] {zp.name} ...", end=" ", flush=True)
        try:
            onnx_path = export_zip_to_onnx(zp)
            write_sidecar_for(onnx_path)
            m = run_one_deploy(onnx_path, args.duration, args.vx)
        except Exception as e:
            print(f"ERROR {e}")
            continue
        m["step"] = step
        m["zip"] = zp
        m["onnx"] = onnx_path
        m["score"] = score(m)
        rows.append(m)
        if m.get("error"):
            print(f"ERROR: {m['error']}")
            continue
        ttf = "OK" if m["time_to_fall"] is None else f"fell@{m['time_to_fall']:.1f}s"
        print(f"{ttf:>8}  fwd={m['forward_distance']:+.2f}m "
              f"yaw={m['yaw_drift_rad']:+.2f}rad "
              f"upright={m['upright_pct']:4.1f}%  score={m['score']:+.3f}")

    if not rows:
        print("[pick] no successful evals")
        return 1
    rows.sort(key=lambda r: r["score"], reverse=True)
    print("\n=== OmniSim leaderboard (high score = good walker) ===")
    print(f"{'step':>8}  {'term':>10}  {'fwd_m':>8}  {'yaw_rad':>8}  "
          f"{'upright%':>9}  {'speed_m/s':>9}  {'score':>8}")
    for r in rows[:10]:
        term = "OK" if r.get("time_to_fall") is None else f"@{r['time_to_fall']:.1f}s"
        if r.get("error"):
            term = "ERR"
            print(f"{r['step']:>8}  {term:>10}  -- error --  score={r['score']:+.3f}")
            continue
        print(f"{r['step']:>8}  {term:>10}  "
              f"{r['forward_distance']:+8.2f}  "
              f"{r['yaw_drift_rad']:+8.3f}  "
              f"{r['upright_pct']:>8.1f}%  "
              f"{r['mean_speed']:>9.3f}  "
              f"{r['score']:>+8.3f}")

    best = rows[0]
    print(f"\n[pick] winner: step {best['step']}  score={best['score']:+.3f}")

    if args.promote_to and best.get("score", -1) > 0:
        args.promote_to.mkdir(parents=True, exist_ok=True)
        # Re-export from the .zip directly into the target dir so the embedded
        # external-data filename inside the ONNX matches the on-disk
        # `policy.onnx.data` (a plain copy preserves the original basename,
        # leaving ONNX runtime loading the wrong file and silently producing
        # garbage actions — observed in this repo at least once).
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        cmd = [
            sys.executable,
            str(REPO_ROOT / "projects" / "rl" / "inference" / "export_onnx.py"),
            "--model", str(best["zip"]),
            "--out", str(args.promote_to / "policy.onnx"),
        ]
        subprocess.run(cmd, env=env, capture_output=True, text=True, check=True)
        write_sidecar_for(args.promote_to / "policy.onnx")
        print(f"[pick] promoted to {args.promote_to}")

    return 0 if rows[0]["score"] > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
