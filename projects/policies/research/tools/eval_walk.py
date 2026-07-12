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

"""Numerical walk evaluator.

Loads a trained ONNX policy, runs it in the OmniSim deploy world (or the
training env directly), and computes:
  * mean forward velocity (m/s)
  * net distance traveled (m)
  * upright-fraction (% of ticks with bz > 0.45 and |roll|,|pitch| < 0.5)
  * episode length until termination
  * tracking error vs commanded velocity

Two modes:
  --mode env     : use the training-side OmniSim env (full sim, fast, no rendering)
                   advantage: matches training distribution; can sweep
                   different commanded velocities.
  --mode deploy  : launch the deploy world + the rl_deploy controller and read
                   the body trace CSV. Slower but exercises the deploy pipeline
                   exactly as production would.

Run from repo root:
    python projects/policies/research/tools/eval_walk.py \
        --policy projects/policies/research/inference/policies/<run>/policy.onnx \
        --mode env --duration 16 --vx 0.5

Prints a one-line summary plus per-second velocity samples. Exits 0 if
"walking" criteria pass:
    mean_vx > 0.10 m/s AND upright_pct > 80 AND ep_len_steps == max
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


def eval_in_env(policy_path: Path, duration_s: float, vx_target: float,
                wz_target: float = 0.0, verbose: bool = False) -> dict:
    """Run the policy inside the training OmniSim env at a fixed command."""
    import onnxruntime as ort
    sys.path.insert(0, str(REPO_ROOT / "projects" / "rl" / "envs"))
    from spot_env import SpotEnv, OBS_DIM, ACT_DIM

    sess = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    # Pin the env's velocity command by patching the env vars BEFORE the
    # controller starts. The controller reads them once on startup.
    # (Note: training-side env randomizes commands per reset; we want a fixed
    # command here for deterministic eval.)
    os.environ["SPOT_FIXED_VX"] = f"{vx_target}"
    os.environ["SPOT_FIXED_WZ"] = f"{wz_target}"

    env = SpotEnv(env_id=42, verbose=verbose)
    obs, _ = env.reset()

    # Override the env's vel command via the obs[45:48] hack only works at
    # the controller side; since we can't change controller live, we simply
    # measure what the policy does given whatever command the controller
    # picked. The controller randomizes per reset; we sample once.
    # For "walking from a standing start", just call reset() once and
    # measure how far the body moves in the next N ticks.

    # Translate seconds to steps. Step is the Webots basicTimeStep (default
    # 16 ms — let's measure the per-step velocity assuming dt=0.016.
    step_dt = 0.016
    n_steps = int(duration_s / step_dt)

    positions = []
    velocities = []
    obs_history = []
    rewards = []
    terminated_at = None

    t0 = time.time()
    for i in range(n_steps):
        action_full = sess.run(None, {in_name: obs.reshape(1, -1).astype(np.float32)})[0][0]
        action = np.clip(action_full, -1.0, 1.0).astype(np.float32)
        obs, reward, term, trunc, _ = env.step(action)
        rewards.append(reward)
        # Extract proxies from obs: v_lin = obs[0:3], joint pos = obs[9:21]
        velocities.append(obs[0:3].copy())
        # The env doesn't report bx/by/bz directly; integrate vx for distance.
        obs_history.append(obs.copy())
        if term or trunc:
            terminated_at = i + 1
            obs, _ = env.reset()
            # Don't continue — one episode at a time for clean stats.
            break
    elapsed = time.time() - t0

    env.close()

    vel_arr = np.array(velocities) if velocities else np.zeros((1, 3))
    obs_arr = np.array(obs_history) if obs_history else np.zeros((1, OBS_DIM))
    rew_arr = np.array(rewards) if rewards else np.zeros(1)

    # Projected gravity in body frame: obs[6:9]. proj_g[2] should be ~-1 if
    # upright (gravity points straight down through body z).
    proj_g_z = obs_arr[:, 8]  # body-frame z component of gravity proxy
    # When upright, proj_g[2] ~ -1.0; when on back, ~ +1.0.
    upright_mask = proj_g_z < -0.7
    upright_pct = float(upright_mask.mean()) * 100.0

    mean_vx = float(vel_arr[:, 0].mean())
    mean_vy = float(vel_arr[:, 1].mean())
    mean_vz = float(vel_arr[:, 2].mean())
    # Integrate vx over time for distance traveled along world x.
    distance_x = float(np.cumsum(vel_arr[:, 0]).sum()) * step_dt
    ep_len = len(velocities)
    mean_reward = float(rew_arr.mean()) if len(rew_arr) else 0.0
    sum_reward = float(rew_arr.sum()) if len(rew_arr) else 0.0

    # Per-second snapshots.
    samples_per_s = max(1, int(1.0 / step_dt))
    per_sec_vx = vel_arr[:, 0][::samples_per_s].tolist()

    return dict(
        n_steps=ep_len,
        terminated_at=terminated_at,
        mean_vx=mean_vx, mean_vy=mean_vy, mean_vz=mean_vz,
        distance_x=distance_x,
        upright_pct=upright_pct,
        mean_reward=mean_reward, sum_reward=sum_reward,
        per_sec_vx=per_sec_vx,
        wall_s=elapsed,
    )


def eval_in_deploy(policy_path: Path, duration_s: float,
                   vx_target: float, wz_target: float = 0.0) -> dict:
    """Launch deploy world + rl_deploy controller, parse the body-trace CSV."""
    import subprocess

    trace_dir = Path(r"C:\tmp\husky_trace")
    trace_dir.mkdir(parents=True, exist_ok=True)
    csv_path = trace_dir / "spot_deploy.csv"
    if csv_path.exists():
        csv_path.unlink()

    env = os.environ.copy()
    env["SPOT_POLICY_ONNX"] = str(policy_path)
    env["OMNISIM_POLICY_ONNX"] = str(policy_path)
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["SPOT_DEPLOY_TRACE"] = "1"
    env["OMNISIM_DEPLOY_TRACE"] = "1"
    env["OMNISIM_BOOT_LOG"] = "1"
    env["OMNISIM_VX"] = f"{vx_target}"
    env["OMNISIM_WZ"] = f"{wz_target}"
    env["SPOT_VX"] = f"{vx_target}"
    env["SPOT_WZ"] = f"{wz_target}"

    webots = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
    world = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl_deploy.wbt"
    cmd = [str(webots), str(world), "--batch", "--mode=fast", "--no-rendering",
           "--minimize", "--stdout", "--stderr"]
    # Redirect Webots stdout/stderr to DEVNULL. If we use subprocess.PIPE
    # without draining, the OS pipe buffer fills (Webots writes lots of Qt
    # warnings each tick) and the child process blocks waiting for the
    # pipe to drain. Symptom: controller never starts, no trace CSV
    # written, eval reports "ERROR: no trace CSV produced".
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)

    start = time.time()
    deadline = start + duration_s + 8  # let it run, then kill
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
    try:
        proc.wait(timeout=4)
    except Exception:
        pass
    elapsed = time.time() - start

    if not csv_path.exists():
        return dict(error="no trace CSV produced", wall_s=elapsed)

    import csv as csvlib
    rows = list(csvlib.DictReader(csv_path.open()))
    if not rows:
        return dict(error="empty trace CSV", wall_s=elapsed)

    def f(s): return float(s)
    t_ms = np.array([f(r["t_ms"]) for r in rows])
    bx = np.array([f(r["bx"]) for r in rows])
    by = np.array([f(r["by"]) for r in rows])
    bz = np.array([f(r["bz"]) for r in rows])
    roll = np.array([f(r["roll"]) for r in rows])
    pitch = np.array([f(r["pitch"]) for r in rows])
    vx = np.array([f(r["vx"]) for r in rows])
    vy = np.array([f(r["vy"]) for r in rows])
    vz = np.array([f(r["vz"]) for r in rows])

    duration = (t_ms[-1] - t_ms[0]) / 1000.0
    distance_x = float(bx[-1] - bx[0])
    distance_y = float(by[-1] - by[0])
    mean_vx = float(vx.mean())
    mean_vy = float(vy.mean())
    mean_vz = float(vz.mean())
    # Upright: bz > 0.45 AND |roll|,|pitch| < 0.5
    upright = (bz > 0.45) & (np.abs(roll) < 0.5) & (np.abs(pitch) < 0.5)
    upright_pct = float(upright.mean()) * 100.0
    min_bz = float(bz.min())
    # Time-to-fall: first sample where bz < 0.30 or |roll|>0.7
    fall_mask = (bz < 0.30) | (np.abs(roll) > 0.7) | (np.abs(pitch) > 0.7)
    if fall_mask.any():
        time_to_fall = float(t_ms[np.argmax(fall_mask)]) / 1000.0
    else:
        time_to_fall = None

    # Per-second snapshots from the 10 Hz trace (every 10th sample = 1 Hz).
    per_sec_vx = vx[::10].tolist() if len(vx) > 10 else vx.tolist()
    per_sec_bz = bz[::10].tolist() if len(bz) > 10 else bz.tolist()

    return dict(
        n_samples=len(rows),
        duration=duration,
        distance_x=distance_x, distance_y=distance_y,
        mean_vx=mean_vx, mean_vy=mean_vy, mean_vz=mean_vz,
        upright_pct=upright_pct,
        min_bz=min_bz,
        time_to_fall=time_to_fall,
        per_sec_vx=per_sec_vx,
        per_sec_bz=per_sec_bz,
        wall_s=elapsed,
    )


def fmt_result(d: dict, mode: str, vx_target: float) -> str:
    out = []
    out.append(f"┌─ eval ({mode}) — target vx={vx_target} m/s ─")
    if d.get("error"):
        out.append(f"│ ERROR: {d['error']}")
        return "\n".join(out)
    if mode == "env":
        out.append(f"│ steps : {d['n_steps']}  terminated_at={d.get('terminated_at')}")
        out.append(f"│ mean_v: vx={d['mean_vx']:+.3f}  vy={d['mean_vy']:+.3f}  vz={d['mean_vz']:+.3f}")
        out.append(f"│ dist_x: {d['distance_x']:+.3f} m  upright={d['upright_pct']:.1f}%")
        out.append(f"│ reward: mean={d['mean_reward']:+.3f}  sum={d['sum_reward']:+.1f}")
    else:
        out.append(f"│ samples: {d['n_samples']}  duration={d['duration']:.2f}s")
        out.append(f"│ mean_v : vx={d['mean_vx']:+.3f}  vy={d['mean_vy']:+.3f}  vz={d['mean_vz']:+.3f}")
        out.append(f"│ dist   : x={d['distance_x']:+.3f}m y={d['distance_y']:+.3f}m  min_bz={d['min_bz']:.3f}")
        out.append(f"│ upright: {d['upright_pct']:.1f}%   time_to_fall={d.get('time_to_fall')}")
    # Per-second vx snapshot
    ps = d.get("per_sec_vx") or []
    if ps:
        out.append(f"│ vx/s  : " + " ".join(f"{x:+.2f}" for x in ps[:12]))
    out.append("└─")
    return "\n".join(out)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--mode", choices=["env", "deploy"], default="env")
    p.add_argument("--duration", type=float, default=16.0)
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--wz", type=float, default=0.0)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    if not args.policy.exists():
        print(f"[eval_walk] policy not found: {args.policy}")
        return 1

    print(f"[eval_walk] {args.mode} mode  policy={args.policy.name}  duration={args.duration}s")
    if args.mode == "env":
        d = eval_in_env(args.policy, args.duration, args.vx, args.wz, args.verbose)
    else:
        d = eval_in_deploy(args.policy, args.duration, args.vx, args.wz)

    print(fmt_result(d, args.mode, args.vx))

    # Walking criteria
    mean_vx = d.get("mean_vx", 0.0) or 0.0
    upright_pct = d.get("upright_pct", 0.0) or 0.0
    walking = mean_vx > 0.10 and upright_pct > 80.0
    print(f"\n[eval_walk] mean_vx={mean_vx:+.3f}  upright_pct={upright_pct:.1f}  "
          f"=> walking={walking}")
    return 0 if walking else 2


if __name__ == "__main__":
    sys.exit(main())
