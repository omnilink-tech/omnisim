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

"""Headless MuJoCo deploy for an MJX-trained ONNX policy.

Runs the policy in **pure MuJoCo (CPU)** — no Webots, no Qt, no GL,
no visible window of any kind. This closes the sim-to-sim gap because
the deploy physics engine is exactly what MJX trained against (MuJoCo's
solver; MJX is just a JAX-batched wrapper around the same engine). A
policy that walks in MJX walks here at deploy, bit-for-bit.

The obs/action contract mirrors `mjx_gpu.py`'s training env:
  - 8 physics substeps × 2 ms = 16 ms per policy decision
  - obs = [v_lin(3), v_ang(3), proj_g(3), q_act(nu), qd_act_finite_diff(nu),
          last_action(nu), vel_cmd(3), clock(1)] padded to obs_dim
  - action ∈ [-1, 1]; target = nominal + action_scale * action
  - reset puts the robot at spawn_translation with identity rotation

Run from the repo root:
    python projects/policies/research/tools/mujoco_deploy.py \\
        --policy projects/policies/research/inference/policies/spot_mjx_v3/policy.onnx \\
        --duration 12 --vx 0.5 --trace .tmp/mujoco_deploy.csv

Reports numerical summary (mean_vx, distance, upright%, min_bz, time-to-fall)
in the same format as `eval_walk.py`'s deploy mode. Optionally writes a
CSV trace of pose+vel at 10 Hz.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


# ── obs/policy contract (mirror of mjx_gpu.py's build_obs) ──

def _proj_gravity_from_quat(q):
    """Project world -z into body frame given quaternion (w, x, y, z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([
        2.0 * (x*z - w*y) * -1.0,
        2.0 * (y*z + w*x) * -1.0,
        (1.0 - 2.0 * (x*x + y*y)) * -1.0,
    ], dtype=np.float32)


def build_obs(d, last_action, vel_cmd, clock, n_act, obs_dim, prev_q_act):
    """Match the build_obs in projects/policies/research/backends/mjx_gpu.py exactly."""
    v_lin = np.asarray(d.qvel[:3], dtype=np.float32)
    v_ang = np.asarray(d.qvel[3:6], dtype=np.float32)
    q = np.asarray(d.qpos[3:7], dtype=np.float32)        # (w, x, y, z)
    proj_g = _proj_gravity_from_quat(q)
    q_act = np.asarray(d.qpos[7:7+n_act], dtype=np.float32)
    qd_act = (q_act - prev_q_act) / 0.016                # finite-diff over 16 ms
    obs = np.concatenate([
        v_lin, v_ang, proj_g, q_act, qd_act,
        last_action.astype(np.float32),
        np.asarray(vel_cmd, dtype=np.float32),
        np.array([clock], dtype=np.float32),
    ])
    if obs.shape[0] < obs_dim:
        obs = np.concatenate([obs, np.zeros(obs_dim - obs.shape[0], dtype=np.float32)])
    obs = np.nan_to_num(obs[:obs_dim], nan=0.0, posinf=10.0, neginf=-10.0)
    return np.clip(obs, -10.0, 10.0).astype(np.float32), q_act


# ── deploy harness ──

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True,
                   help="path to policy.onnx (robot_spec.json must sit alongside)")
    p.add_argument("--duration", type=float, default=12.0, help="simulated seconds")
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--wz", type=float, default=0.0)
    p.add_argument("--trace", type=Path, default=None,
                   help="optional CSV path for the body trace (10 Hz)")
    p.add_argument("--bz-fail", type=float, default=0.30,
                   help="body z below this counts as a fall (matches training)")
    p.add_argument("--rp-fail", type=float, default=1.0,
                   help="|roll| or |pitch| above this counts as a fall")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--noise-std", type=float, default=0.0,
                   help="Gaussian noise added to actions at deploy. Mirrors "
                        "what training did (sample from N(mean, std)). Match "
                        "this to the trained policy's final log_std (exp(log_std)). "
                        "Default 0 = deterministic mean.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--view", action="store_true",
                   help="Open the MuJoCo interactive viewer and step at "
                        "real-time so a human can watch. Slows the run to ~1x "
                        "real time (vs ~90x without --view).")
    p.add_argument("--rate", type=float, default=1.0,
                   help="When --view is set, playback rate as a multiplier of "
                        "real time. 1.0 = real time, 0.5 = slow-mo, 2.0 = 2x.")
    args = p.parse_args()
    rng = np.random.default_rng(args.seed)

    # Load policy + spec.
    spec_path = args.policy.parent / "robot_spec.json"
    if not spec_path.exists():
        print(f"[deploy] ERROR: sidecar not found: {spec_path}")
        return 1

    from projects.policies.research.backends.base import RobotSpec
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
    import onnxruntime as ort
    import mujoco

    spec = RobotSpec.from_file(spec_path)
    n_act = len(spec.joint_names)
    obs_dim = int(spec.obs_dim)
    action_scale = float(spec.action_scale)
    nominal = np.asarray(spec.nominal_pose, dtype=np.float32)
    lo = np.asarray(spec.joint_limits_lo, dtype=np.float32)
    hi = np.asarray(spec.joint_limits_hi, dtype=np.float32)

    sess = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    # Build the MJCF model with the SAME actuator gains MJX training
    # used. Without these, load_or_convert defaults to kp=80 (4× stiffer
    # than Spot's motor_pid kp=20) — the policy's joint targets become
    # too aggressive and topple the robot inside 1 second.
    kp_arg = float(spec.motor_pid[0])
    kv_arg = float(spec.motor_pid[2])
    m = load_or_convert(
        spec.urdf_path,
        actuator_joints=list(spec.joint_names),
        joint_limits=list(zip(spec.joint_limits_lo, spec.joint_limits_hi)),
        kp=kp_arg,
        kv=kv_arg,
    )
    d = mujoco.MjData(m)

    # Reset: spawn pose + nominal joint positions.
    spawn_t = list(spec.spawn_translation)
    d.qpos[0] = float(spawn_t[0])
    d.qpos[1] = float(spawn_t[1])
    d.qpos[2] = float(spawn_t[2])
    d.qpos[3] = 1.0  # quat w
    d.qpos[4] = 0.0
    d.qpos[5] = 0.0
    d.qpos[6] = 0.0
    for i, q in enumerate(spec.nominal_pose):
        d.qpos[7 + i] = float(q)
    d.qvel[:] = 0.0
    d.ctrl[:] = nominal

    # 20-tick settle, matching the training reset (8 physics substeps × 20
    # = 160 physics ticks ~ 320 ms). Without this the first policy obs
    # comes from a still-falling body and the policy can stumble.
    mujoco.mj_resetData(m, d)
    d.qpos[0] = float(spawn_t[0]); d.qpos[1] = float(spawn_t[1]); d.qpos[2] = float(spawn_t[2])
    d.qpos[3] = 1.0; d.qpos[4] = 0.0; d.qpos[5] = 0.0; d.qpos[6] = 0.0
    for i, q in enumerate(spec.nominal_pose):
        d.qpos[7 + i] = float(q)
    d.ctrl[:] = nominal
    for _ in range(20 * 8):
        mujoco.mj_step(m, d)

    # Trace writer (optional).
    trace_f = None
    if args.trace is not None:
        args.trace.parent.mkdir(parents=True, exist_ok=True)
        trace_f = open(args.trace, "w", buffering=1)
        trace_f.write("t_ms,bx,by,bz,roll,pitch,yaw,vx,vy,vz\n")

    # Optional MuJoCo native viewer for visual playback. Imported lazily so
    # the headless path stays import-clean (mujoco.viewer requires GLFW
    # which a headless box may not have).
    viewer = None
    if args.view:
        try:
            import mujoco.viewer as mj_viewer
        except ImportError as e:
            print(f"[deploy] --view needs mujoco.viewer (GLFW). {e}")
            return 1
        viewer = mj_viewer.launch_passive(m, d)
        # Camera framing: look at the robot from the side, slightly above.
        viewer.cam.lookat[:] = [0.0, 0.0, 0.4]
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        viewer.sync()
        print(f"[deploy] viewer up — close the window to exit early")

    # Run loop.
    step_dt = 0.016   # policy decision interval (matches training)
    n_substeps = 8    # 8 × 2 ms = 16 ms per decision
    n_decisions = int(args.duration / step_dt)
    last_action = np.zeros(n_act, dtype=np.float32)
    prev_q_act = np.asarray(d.qpos[7:7+n_act], dtype=np.float32).copy()
    vel_cmd = np.array([args.vx, args.vy, args.wz], dtype=np.float32)

    # Metrics.
    bxs, bys, bzs = [], [], []
    rolls, pitches, yaws = [], [], []
    vxs, vys, vzs = [], [], []
    terminated_at = None
    sim_time = 0.0
    next_trace_ms = 0

    t_start = time.time()
    # Real-time pacing when the viewer is active.
    next_realtime = t_start
    for k in range(n_decisions):
        if viewer is not None and not viewer.is_running():
            print("[deploy] viewer closed by user")
            break
        # Clock — 0.5 Hz matches the legged_locomotion recipe's default
        # for the Webots controller; the MJX env uses the same.
        clock = (sim_time * 0.5) % 1.0
        obs, prev_q_act = build_obs(d, last_action, vel_cmd, clock, n_act,
                                    obs_dim, prev_q_act)
        action = sess.run(None, {in_name: obs.reshape(1, -1)})[0][0]
        if args.noise_std > 0.0:
            action = action + rng.normal(0.0, args.noise_std, action.shape).astype(np.float32)
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        last_action = action
        target = np.clip(nominal + action_scale * action, lo, hi)
        d.ctrl[:n_act] = target

        for _ in range(n_substeps):
            mujoco.mj_step(m, d)
        sim_time += step_dt

        if viewer is not None:
            viewer.sync()
            # Real-time pacing: sleep until next decision tick is due.
            next_realtime += step_dt / max(args.rate, 1e-3)
            now = time.time()
            if next_realtime > now:
                time.sleep(next_realtime - now)
            else:
                # We're behind real-time; don't let the lag compound.
                next_realtime = now

        # Body pose + vel.
        bx, by, bz = float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])
        qw, qx, qy, qz = float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6])
        roll  = math.atan2(2.0 * (qw*qx + qy*qz), 1.0 - 2.0 * (qx*qx + qy*qy))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw*qy - qz*qx))))
        yaw   = math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))
        vx, vy, vz = float(d.qvel[0]), float(d.qvel[1]), float(d.qvel[2])

        bxs.append(bx); bys.append(by); bzs.append(bz)
        rolls.append(roll); pitches.append(pitch); yaws.append(yaw)
        vxs.append(vx); vys.append(vy); vzs.append(vz)

        sim_ms = int(sim_time * 1000)
        if trace_f is not None and sim_ms >= next_trace_ms:
            trace_f.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                          f"{roll:.4f},{pitch:.4f},{yaw:.4f},"
                          f"{vx:.4f},{vy:.4f},{vz:.4f}\n")
            next_trace_ms = sim_ms + 100

        # Fall check (training-side termination criteria).
        if (bz < args.bz_fail
                or abs(roll) > args.rp_fail or abs(pitch) > args.rp_fail):
            if terminated_at is None:
                terminated_at = k + 1
            # Don't break — let the loop continue so we report a full
            # trace; the upright_pct computation already accounts for
            # the time after the fall.

    elapsed = time.time() - t_start
    if trace_f is not None:
        trace_f.close()
    if viewer is not None:
        viewer.close()

    # Summary.
    bxs = np.array(bxs); bys = np.array(bys); bzs = np.array(bzs)
    rolls = np.array(rolls); pitches = np.array(pitches)
    vxs = np.array(vxs); vys = np.array(vys); vzs = np.array(vzs)
    n = len(bxs)
    upright = ((bzs > 0.45) & (np.abs(rolls) < 0.5) & (np.abs(pitches) < 0.5))
    upright_pct = float(upright.mean()) * 100.0
    fall_mask = (bzs < args.bz_fail) | (np.abs(rolls) > args.rp_fail) | (np.abs(pitches) > args.rp_fail)
    time_to_fall = (float(np.argmax(fall_mask)) * step_dt) if fall_mask.any() else None

    print(f"┌─ mujoco deploy — target vx={args.vx} m/s ─")
    print(f"│ steps      : {n}  wall={elapsed:.1f}s  sim={n*step_dt:.1f}s  "
          f"speedup={(n*step_dt)/max(elapsed,1e-3):.1f}×")
    print(f"│ terminated : {'no' if terminated_at is None else f'step {terminated_at}'}")
    print(f"│ mean v     : vx={vxs.mean():+.3f}  vy={vys.mean():+.3f}  vz={vzs.mean():+.3f}")
    print(f"│ distance   : dx={bxs[-1]-bxs[0]:+.3f}m  dy={bys[-1]-bys[0]:+.3f}m  "
          f"|d|={math.hypot(bxs[-1]-bxs[0], bys[-1]-bys[0]):.3f}m")
    print(f"│ body z     : min={bzs.min():.3f}  final={bzs[-1]:.3f}")
    print(f"│ upright%   : {upright_pct:.1f}%")
    print(f"│ time_to_fall: {time_to_fall}")
    if not args.quiet:
        # Per-second vx samples.
        per_sec = max(1, int(1.0 / step_dt))
        snapshot = vxs[::per_sec][:14]
        print(f"│ vx/s       : " + " ".join(f"{v:+.2f}" for v in snapshot))
    print(f"└─")

    walking = (vxs.mean() > 0.05) and (upright_pct > 80) and (terminated_at is None)
    print(f"\n[deploy] mean_vx={vxs.mean():+.3f}  upright_pct={upright_pct:.1f}  "
          f"=> walking={walking}")
    return 0 if walking else 2


if __name__ == "__main__":
    sys.exit(main())
