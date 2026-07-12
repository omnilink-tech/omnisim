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

"""Closed-loop G1 stand policy eval on the CERTIFIED add_urdf + SolverMuJoCo path
(the one the binary-parity probe proved == the deploy binary to machine precision),
single env, deterministic. Dumps a per-tick trace in the parity schema so it can be
diffed against the SAME ONNX run in the deploy binary (g1_policy_probe).

This is the control: the trainer that produced the ONNX uses RAW mujoco_warp on an
MJCF (mjw.step), which does NOT match the binary's SolverMuJoCo.step. Running the
same policy here (SolverMuJoCo, like the binary) isolates whether the closed-loop
gap is the physics path (raw-mjw vs SolverMuJoCo) rather than obs/action/IC.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.research.training.build_g1_native_prim import _build_prim_urdf_xml  # noqa: E402
from projects.policies.research.training.build_g1_native import NJ, DOF0, QPOS0, SPAWN_Z  # noqa: E402

NOMINAL = np.array([
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00, 0.00], np.float32)
LIM_LO = np.array([-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
                   -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618], np.float32)
LIM_HI = np.array([2.880, 2.967, 2.758, 2.880, 0.524, 0.262,
                   2.880, 0.524, 2.758, 2.880, 0.524, 0.262, 2.618], np.float32)
RES_SCALE = 0.3
KP_ANK, KD_ANK, BAL_CLAMP, _DT = -1.5, -0.2, 0.2, 0.016
_L_AP, _R_AP, _L_AR, _R_AR = 4, 10, 5, 11
LEGS_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint"]


def _quat_xyzw_to_rotmat(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy),
            2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx),
            2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]


def build(ke, kd, static_base=False, spawn_z=SPAWN_Z):
    mb = newton.ModelBuilder()
    mb.default_shape_cfg.ke = 2500.0; mb.default_shape_cfg.kd = 100.0
    mb.default_shape_cfg.mu = 1.0
    mb.add_urdf(_build_prim_urdf_xml(),
                xform=wp.transform((0.0, 0.0, spawn_z), (0.0, 0.0, 0.0, 1.0)),
                floating=(not static_base), enable_self_collisions=False)
    free = 0 if static_base else DOF0
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    for j in range(NJ):
        d = free + j
        mb.joint_target_ke[d] = ke; mb.joint_target_kd[d] = kd
        mb.joint_target_mode[d] = pv
    if not static_base:
        mb.add_ground_plane()
    return mb.finalize(), free, (0 if static_base else QPOS0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ke", type=float, default=400.0)
    ap.add_argument("--kd", type=float, default=60.0)
    ap.add_argument("--settle", type=int, default=40)
    ap.add_argument("--ticks", type=int, default=200)
    ap.add_argument("--substeps", type=int, default=4)
    ap.add_argument("--dt", type=float, default=0.016)
    ap.add_argument("--static-base", action="store_true",
                    help="welded base (chaos-free closed-loop parity lane)")
    ap.add_argument("--spawn-z", type=float, default=None)
    args = ap.parse_args(argv)

    import onnxruntime as ort
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    spawn_z = args.spawn_z if args.spawn_z is not None else (1.20 if args.static_base else SPAWN_Z)
    wp.init()
    model, DOFB, QPOSB = build(args.ke, args.kd, args.static_base, spawn_z)
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    sa, sb = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, sa)
    sub_dt = args.dt / args.substeps

    def apply_step(target):
        nonlocal sa, sb
        tp = control.joint_target_pos.numpy()
        tp[DOFB:DOFB+NJ] = target
        control.joint_target_pos.assign(tp)
        for _ in range(args.substeps):
            sa.clear_forces()
            if contacts is not None:
                model.collide(sa, contacts)
                solver.step(sa, sb, control, contacts, sub_dt)
            else:
                solver.step(sa, sb, control, None, sub_dt)
            sa, sb = sb, sa

    # Analytic ankle-balance baseline -- IDENTICAL to gpu_newton_g1_stand_trainer
    # and the deploy controller (g1_policy_probe). NOMINAL + ankle PD on base
    # roll/pitch (finite-diff rates). Must match exactly or the closed-loop diff
    # is apples-to-oranges.
    _prev = {"roll": 0.0, "pitch": 0.0}

    def baseline_from_R(R):
        roll = math.atan2(R[7], R[8])
        pitch = -math.asin(max(-1.0, min(1.0, R[6])))
        rr = (roll - _prev["roll"]) / _DT
        pr = (pitch - _prev["pitch"]) / _DT
        _prev["roll"], _prev["pitch"] = roll, pitch
        ap = max(-BAL_CLAMP, min(BAL_CLAMP, KP_ANK * pitch + KD_ANK * pr))
        ar = max(-BAL_CLAMP, min(BAL_CLAMP, KP_ANK * roll + KD_ANK * rr))
        b = NOMINAL.copy()
        b[_L_AP] += ap; b[_R_AP] += ap
        b[_L_AR] += ar; b[_R_AR] += ar
        return b

    # settle: ramp straight->NOMINAL; advance _prev each step (matches the deploy
    # controller, which calls baseline() every settle step).
    for s in range(args.settle):
        frac = min(1.0, (s + 1) / max(1.0, 0.6 * args.settle))
        bq = sa.body_q.numpy()[0]
        baseline_from_R(_quat_xyzw_to_rotmat(float(bq[3]), float(bq[4]),
                                             float(bq[5]), float(bq[6])))
        apply_step(frac * NOMINAL)

    last_action = np.zeros(NJ, np.float32)
    prev_pg = None
    ticks = []
    for k in range(args.ticks):
        jq = sa.joint_q.numpy()
        q = jq[QPOSB:QPOSB+NJ].astype(np.float32)
        bq = sa.body_q.numpy()[0]
        R = _quat_xyzw_to_rotmat(float(bq[3]), float(bq[4]), float(bq[5]), float(bq[6]))
        pg = np.array([-R[6], -R[7], -R[8]], np.float32)   # -third row = -proj of world z
        # Reproducible angular-rate channel = finite-diff of proj_gravity (matches
        # train<->deploy; the engine base ang-vel does NOT -- proven by the parity
        # test). First tick has no previous frame -> 0.
        dpg = np.zeros(3, np.float32) if prev_pg is None else ((pg - prev_pg) / _DT).astype(np.float32)
        prev_pg = pg
        obs = np.concatenate([q - NOMINAL, pg, dpg, last_action]).astype(np.float32)
        out = sess.run(None, {in_name: obs[None, :]})[0][0]
        action = np.clip(out, -1.0, 1.0).astype(np.float32)
        target = np.clip(baseline_from_R(R) + RES_SCALE * action, LIM_LO, LIM_HI)
        apply_step(target)
        last_action = action
        jq = sa.joint_q.numpy(); bq = sa.body_q.numpy()[0]
        ticks.append({"k": k, "phase": "probe", "target": [float(x) for x in target],
                      "q": [float(x) for x in jq[QPOSB:QPOSB+NJ]],
                      "obs": [float(x) for x in obs],
                      "base_pos": [float(bq[0]), float(bq[1]), float(bq[2])],
                      "base_rot": _quat_xyzw_to_rotmat(float(bq[3]), float(bq[4]),
                                                       float(bq[5]), float(bq[6]))})
    out = {"schema": 1, "side": "trainer",
           "meta": {"robot": "g1", "construction": "add_urdf + SolverMuJoCo (certified path)",
                    "sequence": "policy", "njoints": NJ, "joint_order": LEGS_JOINTS,
                    "onnx": Path(args.onnx).name, "ke": args.ke, "kd": args.kd},
           "ticks": ticks}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out), encoding="utf-8")
    zf = ticks[-1]["base_pos"][2]
    print(f"[g1_policy_eval_addurdf] wrote {args.out} ticks={len(ticks)} final_base_z={zf:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
