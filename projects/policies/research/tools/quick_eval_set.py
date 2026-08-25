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

"""Quickly eval a stride of checkpoints from a training run.

Usage:
    OMNIQUAD_CPG_FREQ_HZ=1.75 OMNIQUAD_CPG_HIP_Y_AMP=0.13 OMNIQUAD_CPG_KNEE_AMP=0.22 \
        python projects/policies/research/tools/quick_eval_set.py \
            --run omniquad_walk_v12 --stride 3 --duration 12 --vx 0.5
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


def export_zip_to_onnx(zip_path: Path) -> Path:
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


def eval_one(onnx_path: Path, duration_s: float, vx_target: float) -> dict:
    sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))
    import onnxruntime as ort
    from omniquad_env import OmniQuadEnv, ACT_DIM
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    env = OmniQuadEnv(env_id=99, verbose=False)
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
        upright_pct=float((pgz < -0.7).mean() * 100.0),
        ep_len=len(vels),
        terminated_at=terminated_at,
        mean_reward=float(np.mean(rews)) if rews else 0.0,
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True)
    p.add_argument("--stride", type=int, default=3, help="eval every Nth checkpoint")
    p.add_argument("--duration", type=float, default=12.0)
    p.add_argument("--vx", type=float, default=0.5)
    args = p.parse_args()

    ckpt_dir = REPO_ROOT / "projects" / "rl" / "training" / "runs" / args.run / "checkpoints"
    ckpts = sorted(ckpt_dir.glob("*.zip"), key=lambda p: int(p.stem.split("_")[-2]))
    selected = ckpts[::args.stride]
    if ckpts[-1] not in selected:
        selected.append(ckpts[-1])
    print(f"[quick] evaluating {len(selected)} / {len(ckpts)} checkpoints "
          f"(stride={args.stride}, duration={args.duration}s)")
    rows = []
    for zp in selected:
        step = int(zp.stem.split("_")[-2])
        print(f"[quick] {zp.name} ...", end=" ", flush=True)
        try:
            onnx = export_zip_to_onnx(zp)
            m = eval_one(onnx, args.duration, args.vx)
        except Exception as e:
            print(f"ERROR: {e}")
            continue
        m["step"] = step
        rows.append(m)
        survived = "TIMEOUT" if m["terminated_at"] is None else f"fell@{m['terminated_at']}"
        print(f"ep_len={m['ep_len']:4d} {survived:>11s}  "
              f"vx={m['mean_vx']:+.3f}  upright={m['upright_pct']:5.1f}%")

    if not rows:
        return 1
    # Sort: surviving timeout first, then by mean_vx descending
    def score(r):
        survived = r["terminated_at"] is None
        return (1 if survived else 0, r["mean_vx"])
    rows.sort(key=score, reverse=True)
    print("\n=== top by survival then speed ===")
    print(f"{'step':>8}  {'ep_len':>6}  {'term':>10}  vx        upright")
    for r in rows[:10]:
        term = "TIMEOUT" if r["terminated_at"] is None else f"@{r['terminated_at']}"
        print(f"{r['step']:>8}  {r['ep_len']:>6}  {term:>10}  "
              f"{r['mean_vx']:+.3f}  {r['upright_pct']:5.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
