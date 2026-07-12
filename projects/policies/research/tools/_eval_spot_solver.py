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

"""Faithful, fast Spot policy eval on Newton's SolverMuJoCo (the OmniSim
deploy engine) -- NO Webots/GUI, runs in seconds.

Loads the EXACT exported Spot MJCF into a newton.ModelBuilder via add_mjcf,
finalizes, and steps SolverMuJoCo with the SAME obs->ONNX->control loop the
deploy controller (spot_rl_deploy.py) runs in OmniSim:
  - obs[0:3]  body linear velocity (world)        = body_qd[root][0:3]
  - obs[3:6]  body angular velocity (body-local)  = R_root^T @ body_qd[root][3:6]
  - obs[6:9]  projected gravity (body-local)
  - obs[9:21] joint angles q (tree order)
  - obs[21:33] joint velocities qd
  - obs[33:45] last action
  - obs[45:48] velocity command (vx,0,0)
  - obs[48]   gait clock (sim_time * cpg_freq) % 1
  target = NOMINAL + pitch_trim_vec + cpg(t) + action_scale * action
  control.joint_target_pos[qd-indexed] = target

This is the ground-truth predictor for "does it walk in OmniSim": same engine
(SolverMuJoCo), same control (joint_target_pos), same obs, same CPG. The only
thing it can't catch is an OmniSim-side joint-name<->device mapping bug.

Run:
    python projects/policies/research/tools/_eval_spot_solver.py \
        --policy projects/policies/research/inference/policies/gpu_spot/policy.onnx \
        --mjcf C:/tmp/spot_newton_fixed.xml --vx 0.5 --secs 12
    # add --mjwarp to use mujoco_warp (use_mujoco_cpu=False) like MJWARP=1 deploy
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = [(leg, j) for leg in LEGS for j in ("hip_x", "hip_y", "knee")]
ACT_DIM = 12
OBS_DIM = 49
HIP_X_LEFT, HIP_X_RIGHT, HIP_Y_STAND, KNEE_STAND = 0.30, -0.30, 0.30, -0.60
NOMINAL = np.zeros(ACT_DIM, np.float32)
for _i, (_leg, _j) in enumerate(JOINT_ORDER):
    NOMINAL[_i] = (HIP_X_LEFT if "left" in _leg else HIP_X_RIGHT) if _j == "hip_x" \
        else (HIP_Y_STAND if _j == "hip_y" else KNEE_STAND)
CPG_LEG_PHASE = {"front_left": 0.0, "front_right": 0.5, "rear_left": 0.5, "rear_right": 0.0}


def cpg_offsets(t, freq, hip_amp, knee_amp, trot=False):
    base = np.zeros(ACT_DIM, np.float32)
    if freq <= 0.0:
        return base
    w = 2.0 * math.pi * freq
    for i, (leg, j) in enumerate(JOINT_ORDER):
        th = w * t + 2.0 * math.pi * CPG_LEG_PHASE[leg]
        if j == "hip_y":
            base[i] = hip_amp * math.cos(th)
        elif j == "knee":
            if trot:  # single flex gated to forward swing -> net forward push
                base[i] = -knee_amp * max(0.0, -math.sin(th))
            else:
                s = max(0.0, math.sin(th))
                base[i] = -knee_amp * (s * s)
    return base


def quat_to_R(q):  # q = (x,y,z,w) newton/warp body_q convention
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--policy", default=str(REPO / "projects/policies/research/inference/policies/gpu_spot/policy.onnx"))
    p.add_argument("--mjcf", default=r"C:\tmp\spot_newton_fixed.xml")
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--secs", type=float, default=12.0)
    p.add_argument("--dt", type=float, default=0.016)
    p.add_argument("--mjwarp", action="store_true", help="use_mujoco_cpu=False (mujoco_warp), matches OMNISIM_NEWTON_MJWARP=1")
    p.add_argument("--njmax", type=int, default=0, help="SolverMuJoCo njmax (constraint cap); 0=wrapper default")
    p.add_argument("--nconmax", type=int, default=0, help="SolverMuJoCo nconmax (contact cap); 0=wrapper default")
    p.add_argument("--settle", type=int, default=20)
    p.add_argument("--gait-only", action="store_true", help="zero the policy action: pure NOMINAL+trim+CPG open-loop")
    p.add_argument("--trot", action="store_true", help="single-flex trot knee gait (forward push) instead of legacy sin^2")
    args = p.parse_args()

    import warp as wp
    import newton
    import onnxruntime as ort

    spec = {}
    spec_path = Path(args.policy).parent / "robot_spec.json"
    if spec_path.exists():
        spec = json.loads(spec_path.read_text())
    cpg_freq = float(spec.get("cpg_freq_hz", 0.0))
    cpg_hip = float(spec.get("cpg_hip_y_amp", 0.0))
    cpg_knee = float(spec.get("cpg_knee_amp", 0.0))
    action_scale = float(spec.get("action_scale", 0.15))
    pitch_trim = float(spec.get("pitch_trim", 0.0))
    trim_vec = np.array([pitch_trim if j == "hip_y" else 0.0 for (_l, j) in JOINT_ORDER], np.float32)
    print(f"[eval-solver] spec: cpg_freq={cpg_freq} hip={cpg_hip} knee={cpg_knee} "
          f"action_scale={action_scale} pitch_trim={pitch_trim}")
    print(f"[eval-solver] engine={'mujoco_warp' if args.mjwarp else 'mj_step(cpu)'}  vx={args.vx}  secs={args.secs}")

    wp.init()
    builder = newton.ModelBuilder()
    builder.add_mjcf(args.mjcf)
    builder.add_ground_plane()
    model = builder.finalize()
    nq, nv = model.joint_coord_count, model.joint_dof_count
    print(f"[eval-solver] model nq={nq} nv={nv} bodies={model.body_count}")

    # seed standing pose: free joint q[0:7] = pos(3)+quat(xyzw), revolutes q[7:19]
    jq = model.joint_q.numpy()
    jq[0:3] = [0.0, 0.0, 0.7]
    jq[3:7] = [0.0, 0.0, 0.0, 1.0]
    jq[7:7 + ACT_DIM] = NOMINAL
    model.joint_q.assign(jq)

    skw = {}
    if args.njmax > 0:
        skw["njmax"] = args.njmax
    if args.nconmax > 0:
        skw["nconmax"] = args.nconmax
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=not args.mjwarp, **skw)
    state_a, state_b = model.state(), model.state()
    state_a.joint_q.assign(jq)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None

    sess = ort.InferenceSession(args.policy, providers=["CPUExecutionProvider"])

    def set_targets(target):
        tp = control.joint_target_pos.numpy()
        tp[6:6 + ACT_DIM] = target  # free joint occupies qd[0:6]
        control.joint_target_pos.assign(tp)

    # settle: hold nominal pose
    set_targets(NOMINAL + trim_vec)
    for _ in range(args.settle):
        state_a.clear_forces()
        if contacts is not None:
            model.collide(state_a, contacts)
            solver.step(state_a, state_b, control, contacts, args.dt)
        else:
            solver.step(state_a, state_b, control, None, args.dt)
        state_a, state_b = state_b, state_a

    last_action = np.zeros(ACT_DIM, np.float32)
    nsteps = int(args.secs / args.dt)
    x0 = float(state_a.body_q.numpy()[0, 0])
    bx_hist, bz_hist, vx_hist = [], [], []
    fell_at = None
    sim_t = 0.0
    for step in range(nsteps):
        bq = state_a.body_q.numpy()[0]
        bqd = state_a.body_qd.numpy()[0]
        pos = bq[0:3]
        R = quat_to_R(bq[3:7])
        v_lin = bqd[0:3].astype(np.float32)
        v_ang = (R.T @ bqd[3:6]).astype(np.float32)
        proj_g = (R.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)
        jq_now = state_a.joint_q.numpy()
        jqd_now = state_a.joint_qd.numpy()
        q = jq_now[7:7 + ACT_DIM].astype(np.float32)
        qd = jqd_now[6:6 + ACT_DIM].astype(np.float32)
        clock = (sim_t * (cpg_freq if cpg_freq > 0 else 0.5)) % 1.0
        obs = np.zeros(OBS_DIM, np.float32)
        obs[0:3] = v_lin; obs[3:6] = v_ang; obs[6:9] = proj_g
        obs[9:21] = q; obs[21:33] = qd; obs[33:45] = last_action
        obs[45:48] = [args.vx, 0.0, 0.0]; obs[48] = clock
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10, neginf=-10), -10, 10).astype(np.float32)

        action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        action = np.clip(action, -1, 1).astype(np.float32)
        last_action = action.copy()
        if args.gait_only:
            action = np.zeros(ACT_DIM, np.float32)
        cpg = cpg_offsets(sim_t, cpg_freq, cpg_hip, cpg_knee, trot=args.trot)
        target = NOMINAL + trim_vec + cpg + action_scale * action
        set_targets(target)

        state_a.clear_forces()
        if contacts is not None:
            model.collide(state_a, contacts)
            solver.step(state_a, state_b, control, contacts, args.dt)
        else:
            solver.step(state_a, state_b, control, None, args.dt)
        state_a, state_b = state_b, state_a
        sim_t += args.dt

        # fall detection from root orientation
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        bx_hist.append(float(pos[0])); bz_hist.append(float(pos[2])); vx_hist.append(float(v_lin[0]))
        if fell_at is None and (pos[2] < 0.30 or abs(roll) > 0.7 or abs(pitch) > 0.7 or not np.isfinite(pos[2])):
            fell_at = sim_t

    bx = np.array(bx_hist); bz = np.array(bz_hist); vxh = np.array(vx_hist)
    dist = float(bx[-1] - x0) if len(bx) else 0.0
    print("\n=== SolverMuJoCo Spot eval ===")
    print(f"  steps run        : {len(bx)} ({len(bx)*args.dt:.1f}s sim)")
    print(f"  forward distance : {dist:+.3f} m")
    print(f"  mean vx (alive)  : {np.nanmean(vxh):.3f} m/s   (target {args.vx})")
    print(f"  min body z       : {np.nanmin(bz):.3f} m")
    print(f"  fell at          : {fell_at}  (None = never fell)")
    if fell_at is None and dist > 0.5 * args.secs * args.vx * 0.4:
        print("  VERDICT: WALKS (survives + nets forward distance)")
    elif fell_at is None:
        print("  VERDICT: stands but little forward progress")
    else:
        print("  VERDICT: FALLS")


if __name__ == "__main__":
    main()
