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

"""Eval every saved checkpoint in a training run dir and pick the best one.

Walks projects/policies/research/training/runs/<run>/checkpoints/, exports each .zip to a
temporary ONNX, runs it in env mode with the matching CPG params, and
scores by:
    score = mean_vx * upright_pct * (1 - fall_indicator)
where fall_indicator is 1 if the episode terminated (otherwise 0).

Prints a sorted table and identifies the "best" — highest forward velocity
with minimal terminations. Optionally exports the winner to a target
policy directory with the correct sidecar.

Run from repo root:
    SPOT_CPG_FREQ_HZ=1.75 SPOT_CPG_HIP_Y_AMP=0.20 SPOT_CPG_KNEE_AMP=0.30 \
        python projects/policies/research/tools/pick_best_checkpoint.py \
            --run spot_walk_v9 \
            --promote-to projects/policies/research/inference/policies/spot_walk_main

For each checkpoint, the eval is 16 s of env stepping at fixed vx=0.5.
The full sweep takes ~5-10 minutes for a typical training run with ~20
checkpoints (each eval = one env spawn + 1000 ticks + clean shutdown).
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


def export_zip_to_onnx(zip_path: Path) -> Path:
    """Run export_onnx.py on a checkpoint .zip; return the ONNX path."""
    onnx_path = zip_path.with_suffix(".onnx")
    if onnx_path.exists():
        return onnx_path  # already exported
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
    """Write robot_spec.json next to the ONNX using the canonical Spot spec
    plus whatever CPG values are currently active in the env vars (those
    are what the policy was trained against)."""
    sys.path.insert(0, str(REPO_ROOT))
    from projects.policies.research.backends.spot_robot_spec import SPOT
    import dataclasses
    spec = dataclasses.replace(
        SPOT,
        cpg_freq_hz=float(os.environ.get("SPOT_CPG_FREQ_HZ", SPOT.cpg_freq_hz)),
        cpg_hip_y_amp=float(os.environ.get("SPOT_CPG_HIP_Y_AMP", SPOT.cpg_hip_y_amp)),
        cpg_knee_amp=float(os.environ.get("SPOT_CPG_KNEE_AMP", SPOT.cpg_knee_amp)),
    )
    spec.write_sidecar(onnx_path.parent)


def eval_checkpoint(onnx_path: Path, duration_s: float = 16.0,
                    vx_target: float = 0.5) -> dict:
    """Eval the policy in the training env for `duration_s`. Returns dict
    of metrics: mean_vx, upright_pct, terminated, ep_len."""
    sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))
    import onnxruntime as ort
    from spot_env import SpotEnv, ACT_DIM

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    env = SpotEnv(env_id=99, verbose=False)
    obs, _ = env.reset()
    step_dt = 0.016
    n_steps = int(duration_s / step_dt)
    vels = []
    proj_g_z = []
    terminated_at = None
    rews = []
    for i in range(n_steps):
        a = sess.run(None, {in_name: obs.reshape(1, -1).astype(np.float32)})[0][0]
        a = np.clip(a, -1.0, 1.0).astype(np.float32)
        obs, r, term, trunc, _ = env.step(a)
        vels.append(obs[0:3].copy())
        proj_g_z.append(obs[8])
        rews.append(r)
        if term or trunc:
            terminated_at = i + 1
            break
    env.close()

    vels = np.array(vels) if vels else np.zeros((1, 3))
    pgz = np.array(proj_g_z) if proj_g_z else np.zeros(1)
    return dict(
        mean_vx=float(vels[:, 0].mean()),
        mean_vy=float(vels[:, 1].mean()),
        upright_pct=float((pgz < -0.7).mean() * 100.0),
        ep_len=len(vels),
        terminated_at=terminated_at,
        mean_reward=float(np.mean(rews)) if rews else 0.0,
    )


def score(m: dict, max_ep_len: int) -> float:
    """Heuristic: forward velocity weighted by survival fraction, with a
    very large penalty for any termination. The user's priority is
    'never fall' so terminations dominate."""
    survived = (m.get("terminated_at") is None)
    fall_penalty = 0.0 if survived else -1.0
    return (m["mean_vx"] * (m["upright_pct"] / 100.0)
            * (m["ep_len"] / max_ep_len)
            + fall_penalty)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True,
                   help="run name under projects/policies/research/training/runs/")
    p.add_argument("--duration", type=float, default=16.0,
                   help="env eval duration per checkpoint (s)")
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--promote-to", type=Path, default=None,
                   help="if set, copy the winning ONNX + sidecar to this dir")
    args = p.parse_args()

    ckpt_dir = REPO_ROOT / "projects" / "rl" / "training" / "runs" / args.run / "checkpoints"
    if not ckpt_dir.exists():
        print(f"[pick] no checkpoint dir at {ckpt_dir}")
        return 1
    ckpts = sorted(ckpt_dir.glob("*.zip"),
                   key=lambda p: int(p.stem.split("_")[-2]))
    if not ckpts:
        print(f"[pick] no .zip checkpoints under {ckpt_dir}")
        return 1
    print(f"[pick] {len(ckpts)} checkpoints to eval")

    rows = []
    max_ep_len = int(args.duration / 0.016)
    for zp in ckpts:
        step_num = int(zp.stem.split("_")[-2])
        print(f"[pick] {zp.name} ...", flush=True)
        try:
            onnx_path = export_zip_to_onnx(zp)
            m = eval_checkpoint(onnx_path, args.duration, args.vx)
        except Exception as e:
            print(f"  ERROR: {e}")
            continue
        m["step"] = step_num
        m["zip"] = zp
        m["onnx"] = onnx_path
        m["score"] = score(m, max_ep_len)
        rows.append(m)
        survived = "TIMEOUT" if m["terminated_at"] is None else f"fell@{m['terminated_at']}"
        print(f"  ep_len={m['ep_len']:4d} {survived:>11s}  "
              f"vx={m['mean_vx']:+.3f}  upright={m['upright_pct']:5.1f}%  "
              f"score={m['score']:+.3f}")

    if not rows:
        print("[pick] no successful evals")
        return 1
    rows.sort(key=lambda r: r["score"], reverse=True)
    print("\n=== leaderboard (best first) ===")
    print(f"{'step':>7}  {'ep_len':>6}  {'term':>10}  {'vx':>8}  "
          f"{'upright':>8}  {'score':>8}")
    for r in rows[:10]:
        term = "TIMEOUT" if r["terminated_at"] is None else f"@{r['terminated_at']}"
        print(f"{r['step']:>7}  {r['ep_len']:>6}  {term:>10}  "
              f"{r['mean_vx']:+8.3f}  {r['upright_pct']:>7.1f}%  "
              f"{r['score']:>+8.3f}")

    best = rows[0]
    print(f"\n[pick] best: step={best['step']} score={best['score']:+.3f}")

    if args.promote_to:
        args.promote_to.mkdir(parents=True, exist_ok=True)
        # Re-export (the per-ckpt ONNX may have .data sidecar pieces)
        dest_onnx = args.promote_to / "policy.onnx"
        # Copy ONNX + any matching .onnx.data file
        shutil.copy2(best["onnx"], dest_onnx)
        data_src = best["onnx"].with_suffix(".onnx.data")
        if data_src.exists():
            shutil.copy2(data_src, dest_onnx.with_suffix(".onnx.data"))
        # Write a fresh sidecar with the current CPG params.
        write_sidecar_for(dest_onnx)
        print(f"[pick] promoted: {dest_onnx}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
