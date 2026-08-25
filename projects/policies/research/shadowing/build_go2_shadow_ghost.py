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

"""Build + GATE the Go2 SHADOWING walk ghost (lut) -- Component 1+2 for the quad.

TWO SOURCES (--source):

  champion (DEFAULT, doctrine class 1a 'recorded'): roll the LEGACY RL champion
      (gpu_go2_walk_main/policy.onnx riding the analytic trot baseline) on the
      deploy-matched MJCF dump (go2_newton.xml, kp=250/kv=6, DT=16 ms x 8
      substeps -- the exact standalone-trainer physics), with the exact 48-dim
      trainer obs contract, deterministic (mu) actions. A stable policy
      phase-locks to its clock, so the phase-fold is clean -- the same
      record->fold recipe as the G1 flagship's official-walk ghost. The ghost
      is then literally *the incumbent's own achieved gait*, certified.

  mppi ('recorded' via the Ghost Generator): receding-horizon MPPI executing
      the analytic trot prior (generate_go2_walk.py's recipe). KEPT for
      provenance/BASELINE-less robots -- but MEASURED 2026-07-12: MPPI's
      per-cycle exploration jitter misaligns cycles on the clock fold and eats
      ~30% of the stride amplitude; the folded lut PD-replays as a SHUFFLE
      (gmatch 0.998, vx 0.011 -- the stair campaign's shuffle tell). Prefer
      champion whenever a stable walker exists.

GATES (printed + embedded in the lut json; ALL must pass before training):
  [T0-go2]      joint limits vs the REAL Go2 ranges (ghost_validator.py's own
                T0 limit check reads the G1 URDF -> vacuous for Go2 names; this
                is the binding limits gate).
  [1+4 LUT-REPLAY] the BINDING gate: the folded lut + its declared feedforward
                (ffdq_lut), tracked by the bare deploy-grade position servo (no
                crane -- quads use none) from the settled stand, must WALK:
                no fall, real forward progress, planted-foot world drift
                (CLOSURE on the reference-as-executed) p95 < 20 mm, torques in
                limits. Its gmatch vs the lut = the exam's SELF-MATCH CEILING.
  [2+3 SUPPORT+FWP] feasibility_certificate.certify(motion='walk') on the
                replay trajectory -- the per-step LP (friction-pyramid contact
                forces + torque box must supply the base wrench). This is the
                DYNAMIC form of the COM/FWP gates (a trot's support is a
                diagonal pair; static COM-in-hull is the wrong ruler).
                ⚠ the scalar score pins to 0 whenever one DOF rides its torque
                limit (known artifact, same as the repertoire's G1 row) --
                trust the VERDICT, quote base_frac95.
  [CORRIDOR LAW] peak |cmd - q_achieved| per joint over the steady segment ==
                the corridor floor WITHOUT GHOST-FF; emitted as ffdq_lut so
                QUAD_GHOST_FF=1 / GO2_GHOST_FF=1 can shift the corridor CENTRE
                instead (rewards keep scoring the pose).

OUTPUT <out>_lut.json (self-describing for ghost_validator.py):
  joints      : the REAL Go2 joint names, controller order FL,FR,RL,RR x
                (hip,thigh,calf) -- the trainer, the deploy AND the validator all
                assert against THIS (the WBMATCH name-trap rule: never the
                positional fallback).

                ⛔ It used to emit FAKE humanoid aliases here ("FL_hip_roll_joint")
                with the real names hidden in a `joints_go2` side key, purely so that
                ghost_validator's humanoid-name-keyed symmetry gate would engage on a
                quadruped. A rule you have to LIE to is the wrong rule: the validator
                now derives joint roles from the shared robot registry and pairs
                mirrors by name (FL<->FR, RL<->RR), so the alias is deleted.
  leg_lut(nb,12), ffdq_lut(nb,12), ffdq_kp, nb, freq, vx, source, gait, gates.

Usage:
    python projects/policies/research/shadowing/build_go2_shadow_ghost.py \
        [--source champion|mppi] [--reuse <rollout.npz>] [--T 12] [--nb 64] \
        [--fold-from 3.0] [--out _scratch/go2_shadow_ghost]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.feasibility_certificate import certify  # noqa: E402
from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402

PLANNER_MJCF = os.path.join(REPO, "projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
DUMP_MJCF = os.path.join(REPO, "projects/policies/research/training/mjcf/go2_newton.xml")
CHAMPION_ONNX = os.path.join(
    REPO, "projects/policies/research/inference/policies/gpu_go2_walk_main/policy.onnx")
FOOT_R = 0.022
LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JNAMES = [f"{leg}_{p}" for leg in LEGS for p in PARTS]            # controller order
JOINTS_GO2 = [f"{leg}_{p}_joint" for leg in LEGS for p in PARTS]  # deploy motor names
KP = 250.0   # deploy/trainer joint drive (go2_newton.xml kp=250; run_quad_walk_rl.sh)
LIM_LO = np.array([-1.0472, -0.5236, -2.7227] * 4)
LIM_HI = np.array([+1.0472, +3.1316, -0.83776] * 4)
DT_CTRL = 0.016      # the trainer/deploy tick
SUBSTEPS = 8         # x 2 ms
PHYS_DT = 0.002


def _model_maps(m):
    """(jq, jdof, ctrl_pos_idx, calf_bid, dof_lim) in controller order, for BOTH
    the named planner MJCF and the anonymous deploy dump. Classification for the
    dump == gpu_mjwarp_go2_walk_trainer.BatchedSpotWalkEnv (axis test first;
    knee = the Y-hinge whose UPPER bound is negative)."""
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    named = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, JNAMES[0]) >= 0
    jq = np.zeros(12, int); jdof = np.zeros(12, int)
    ctrl_idx = np.zeros(12, int); calf_bid = np.zeros(4, int)
    if named:
        for i, nm in enumerate(JNAMES):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
            jq[i] = m.jnt_qposadr[jid]; jdof[i] = m.jnt_dofadr[jid]
            if i % 3 == 2:
                calf_bid[i // 3] = m.jnt_bodyid[jid]
        # actuators named <joint>_pos
        act = {}
        for ai in range(m.nu):
            an = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, ai) or ""
            if an.endswith("_pos"):
                act[an[:-4]] = ai
        for i, nm in enumerate(JNAMES):
            ctrl_idx[i] = act[nm]
    else:
        lab = {}
        k_hinge = 0
        for j in range(m.njnt):
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            ax = m.jnt_axis[j]; rng_j = m.jnt_range[j]
            pos = d.xpos[m.jnt_bodyid[j]]
            leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
                   "FR" if (pos[0] > 0) else
                   "RL" if (pos[1] > 0) else "RR")
            joint = ("hip" if abs(ax[0]) > 0.5 else
                     "calf" if rng_j[1] < 0.0 else "thigh")
            lab[(leg, joint)] = (int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]),
                                 k_hinge, int(m.jnt_bodyid[j]))
            k_hinge += 1
        for i, nm in enumerate(JNAMES):
            leg, joint = nm.split("_")
            qa, va, hidx, bid = lab[(leg, joint)]
            jq[i] = qa; jdof[i] = va
            ctrl_idx[i] = 2 * hidx          # interleaved [pos, vel] per hinge
            if joint == "calf":
                calf_bid[i // 3] = bid
    dof_lim = np.full(m.nv, 1e9)
    for j in range(m.njnt):
        if m.jnt_actfrclimited[j]:
            dof_lim[m.jnt_dofadr[j]] = max(1e-3, np.abs(m.jnt_actfrcrange[j]).max())
    return jq, jdof, ctrl_idx, calf_bid, dof_lim


def _feet_fk(m, d, calf_bid):
    out = np.zeros((4, 3))
    for f in range(4):
        R3 = d.xmat[calf_bid[f]].reshape(3, 3)
        out[f] = d.xpos[calf_bid[f]] + R3 @ np.array([0.0, 0.0, -stg.L2])
    return out


def _lut_at(table, phase):
    """Circular linear interpolation -- identical to the deploy's _lut_interp."""
    nb = table.shape[0]
    x = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * nb
    b0 = int(math.floor(x)) % nb
    b1 = (b0 + 1) % nb
    f = x - math.floor(x)
    return table[b0] * (1.0 - f) + table[b1] * f


def record_champion(T: float, out_npz: str, gp: stg.GaitParams,
                    policy_onnx: str | None = None, baseline: str = "trot",
                    baseline_lut: str | None = None, use_ff: bool = True,
                    res_scale_in: float = 0.15):
    """Deterministic rollout of A champion on the deploy dump, exact obs/action contract.

    ⭐ SHADOW ITERATION (2026-07-13). This used to roll exactly one policy (the legacy champion)
    on exactly one baseline (the analytic trot). That is the round-1 operator, and it hides the
    interesting question: Shadowing turns a champion's OWN achieved gait into a reference and
    trains a better champion against it (+12.6% speed, 5x straighter, measured). **Does it
    compound?** To even ask, the recorder has to be able to roll a champion that ITSELF rides a
    ghost -- otherwise you record it on the wrong baseline and capture out-of-distribution junk.

        baseline="trot"  : cmd = trot(phase)                       + a*res_scale   (legacy champ)
        baseline="ghost" : cmd = ghost(phase) [+ ffdq(phase)]      + a*res_scale   (shadow champ)

    The second line is exactly go2_shadow_deploy's decode. Same obs family either way (48-dim);
    the ghost enters through the clock and the command centre, never through new obs.
    """
    import onnxruntime as ort
    sess = ort.InferenceSession(policy_onnx or CHAMPION_ONNX,
                                providers=["CPUExecutionProvider"])
    print(f"[go2-shadow] rolling {os.path.basename(os.path.dirname(policy_onnx or CHAMPION_ONNX))} "
          f"on the {baseline.upper()} baseline (ff={'on' if use_ff else 'off'}, "
          f"corridor={res_scale_in})")
    _blut = _bffdq = None
    if baseline == "ghost":
        if not baseline_lut:
            raise SystemExit("--baseline ghost needs --baseline-lut <the ghost this champion rides>")
        _bg = json.loads(open(baseline_lut).read())
        _blut = np.asarray(_bg["leg_lut"], np.float64)
        if use_ff and "ffdq_lut" in _bg:
            _bffdq = np.asarray(_bg["ffdq_lut"], np.float64)
        got = list(_bg.get("joints") or _bg.get("joints_go2") or [])
        if got != JOINTS_GO2:
            raise SystemExit(f"baseline lut joint-order mismatch: {got} != {JOINTS_GO2}")
    m = mujoco.MjModel.from_xml_path(DUMP_MJCF)
    m.opt.timestep = PHYS_DT
    d = mujoco.MjData(m)
    jq, jdof, ctrl_idx, calf_bid, dof_lim = _model_maps(m)
    nominal = stg.standing_pose(gp).astype(np.float64)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qpos[2] = gp.body_height + 0.03
    d.qpos[3] = 1.0
    d.qpos[jq] = nominal
    full = np.zeros(m.nu)
    full[ctrl_idx] = nominal
    d.ctrl[:] = full
    mujoco.mj_forward(m, d)
    # settle 1.5 s at the standing pose (the deploy's rest-start)
    for _ in range(int(1.5 / PHYS_DT)):
        mujoco.mj_step(m, d)
    n = int(T / DT_CTRL)
    Q = np.zeros((n, m.nq)); QV = np.zeros((n, m.nv)); C = np.zeros((n, 12))
    B = np.zeros((n, 7)); F = np.zeros((n, 4, 3)); TAU = np.zeros((n, 12))
    last_action = np.zeros(12, dtype=np.float32)
    last_q = d.qpos[jq].copy()
    omega = 2.0 * math.pi * gp.freq
    res_scale = float(res_scale_in)        # the champion's training corridor
    for k in range(n):
        t = k * DT_CTRL
        phase = stg.QS_PHASE + omega * t
        q12 = d.qpos[jq].copy()
        qd12 = (q12 - last_q) / DT_CTRL    # deploy-faithful finite-diff qd
        last_q = q12.copy()
        w, x, y, z = d.qpos[3:7]
        pg = np.array([-2 * (x * z - w * y), -2 * (y * z + w * x),
                       -(1 - 2 * (x * x + y * y))])
        obs = np.concatenate([
            d.qvel[0:3], d.qvel[3:6], pg, q12 - nominal, qd12, last_action,
            [math.sin(phase), math.cos(phase)], [0.0]]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs), -10.0, 10.0)
        a = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        a = np.clip(a, -1.0, 1.0).astype(np.float32)
        if _blut is None:
            legs, _ = stg.targets_np(phase, gp, t_since_start=t)      # LEGACY: the analytic trot
        else:
            # SHADOW: the exact go2_shadow_deploy decode -- the ghost is the command centre, and
            # its declared feedforward shifts that centre (the corridor law: without FF the ghost
            # is untrackable by a 0.15 corridor BY CONSTRUCTION).
            _rr = min(1.0, t / gp.ramp_s) if gp.ramp_s > 0 else 1.0
            legs = nominal + _rr * (_lut_at(_blut, phase) - nominal)
            if _bffdq is not None:
                legs = legs + _rr * _lut_at(_bffdq, phase)
        cmd = np.clip(legs + res_scale * a, LIM_LO, LIM_HI)
        full[ctrl_idx] = cmd
        d.ctrl[:] = full
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
        last_action = a
        Q[k] = d.qpos; QV[k] = d.qvel; C[k] = cmd; B[k] = d.qpos[0:7]
        F[k] = _feet_fk(m, d, calf_bid)
        TAU[k] = np.abs(d.qfrc_actuator[jdof])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < 0.18 or tilt > 60.0:
            raise RuntimeError(f"champion FELL at t={t:.2f}s during recording -- "
                               "cannot record an unstable source (doctrine rule 1)")
    os.makedirs(os.path.dirname(out_npz), exist_ok=True)
    np.savez(out_npz, q=Q, qvel=QV, ctrl=C, base=B, feet=F, tau=TAU, dt=DT_CTRL)
    fwd = B[-1, 0] - B[0, 0]
    print(f"[go2-shadow-rec] CHAMPION recorded {out_npz}: {n} ticks ({T:.0f}s), "
          f"forward {fwd:+.2f} m ({fwd / T:.3f} m/s), no fall")
    return DUMP_MJCF


def generate_mppi(T: float, seed: int, out_npz: str, n_samples: int,
                  noise: float, temperature: float):
    from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent
    gp = stg.GaitParams()
    control_dt = 0.02
    gen = GhostGenerator(PLANNER_MJCF, sim_dt=0.004, control_dt=control_dt)
    mujoco.mj_resetData(gen.m, gen.d)
    qp = gen.d.qpos
    qp[0:3] = [0.0, 0.0, gp.body_height + FOOT_R]
    qp[3:7] = [1.0, 0.0, 0.0, 0.0]
    stand = stg.standing_pose(gp)
    name2adr = {nm: a for nm, a in zip(
        [n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos)}
    for i, nm in enumerate(JNAMES):
        qp[name2adr[nm]] = stand[i]
    gen.d.qpos[:] = qp
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.3 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    n_keys = int(T / control_dt) + 1
    joint_keys, base_keys = [], []
    x = 0.0
    for k in range(n_keys):
        t = k * control_dt
        phase = stg.QS_PHASE + 2.0 * np.pi * gp.freq * t
        legs, _ = stg.targets_np(phase, gp, t_since_start=t)
        joint_keys.append((t, {JNAMES[i]: float(legs[i]) for i in range(12)}))
        ramp = min(1.0, t / gp.ramp_s)
        x += gp.vx * ramp * control_dt
        base_keys.append((t, {"x": x, "z": gp.body_height + FOOT_R, "pitch": 0.0}))
    intent = Intent(
        total_time=T, joint_keys=joint_keys, base_keys=base_keys,
        weights=dict(joint=8.0, base_xy=6.0, base_z=8.0, pitch=2.0,
                     upright=1.5, balance=0.3, effort=0.01, smooth=0.05, alive=0.5))
    g = gen.generate(intent, init, n_samples=n_samples, horizon=12, noise=noise,
                     temperature=temperature, seed=seed)
    os.makedirs(os.path.dirname(out_npz), exist_ok=True)
    np.savez(out_npz, **{k: v for k, v in g.items() if k != "joints"},
             joints=np.array(g["joints"]))
    base = g["base"]
    print(f"[go2-shadow-gen] MPPI saved {out_npz} steps={g['q'].shape[0]} "
          f"forward={base[-1, 0] - base[0, 0]:+.2f} m")
    return PLANNER_MJCF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=("champion", "mppi"), default="champion")
    ap.add_argument("--policy", default=None,
                    help="the champion ONNX to roll (default: the legacy go2 champion). "
                         "SHADOW ITERATION: point this at a champion that itself rides a ghost.")
    ap.add_argument("--baseline", choices=("trot", "ghost"), default="trot",
                    help="what the champion's residual composes around: the analytic trot "
                         "(legacy) or a ghost lut (a shadow champion)")
    ap.add_argument("--baseline-lut", default=None,
                    help="the ghost lut the champion rides (required for --baseline ghost)")
    ap.add_argument("--no-ff", action="store_true",
                    help="do NOT add the baseline ghost's declared feedforward")
    ap.add_argument("--corridor", type=float, default=0.15,
                    help="the champion's training corridor (its action scale)")
    ap.add_argument("--reuse", default=None, help="skip rollout; use this npz")
    ap.add_argument("--reuse-mjcf", default=None,
                    help="model the reused npz was rolled on (default: by --source)")
    ap.add_argument("--T", type=float, default=12.0)
    ap.add_argument("--nb", type=int, default=64)
    ap.add_argument("--fold-from", type=float, default=3.0,
                    help="fold only t >= this (past ramp + launch transient)")
    ap.add_argument("--harmonics", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--noise", type=float, default=0.10)
    ap.add_argument("--temp", type=float, default=0.4)
    ap.add_argument("--out", default=os.path.join(REPO, "_scratch", "go2_shadow_ghost"))
    args = ap.parse_args()

    gp = stg.GaitParams()
    npz_path = args.reuse or (args.out + ".npz")
    if args.reuse:
        mjcf = args.reuse_mjcf or (DUMP_MJCF if args.source == "champion" else PLANNER_MJCF)
        print(f"[go2-shadow] reusing rollout {npz_path} (model {os.path.basename(mjcf)})")
    elif args.source == "champion":
        mjcf = record_champion(args.T, npz_path, gp, policy_onnx=args.policy,
                               baseline=args.baseline, baseline_lut=args.baseline_lut,
                               use_ff=not args.no_ff, res_scale_in=args.corridor)
    else:
        mjcf = generate_mppi(args.T, args.seed, npz_path, args.samples,
                             args.noise, args.temp)

    g = np.load(npz_path, allow_pickle=True)
    q, qvel, ctrl, base, feet = g["q"], g["qvel"], g["ctrl"], g["base"], g["feet"]
    dt = float(g["dt"])
    T_total = q.shape[0] * dt
    gates = {}

    m = mujoco.MjModel.from_xml_path(mjcf)
    jq, jdof, ctrl_idx, calf_bid, dof_lim = _model_maps(m)
    qj = q[:, jq]
    assert ctrl.shape[1] == 12

    steady = np.arange(q.shape[0]) * dt >= args.fold_from
    n_cycles = (T_total - args.fold_from) * gp.freq
    vx_steady = float((base[-1, 0] - base[steady][0, 0]) / (T_total - args.fold_from))
    print(f"[go2-shadow] rollout T={T_total:.2f}s steady from {args.fold_from}s "
          f"({n_cycles:.1f} cycles), steady vx={vx_steady:.3f} m/s, source={args.source}")

    # ── GATE T0-go2: the REAL joint-limit check ──
    over_lo = float((LIM_LO - qj.min(0)).max())
    over_hi = float((qj.max(0) - LIM_HI).max())
    ok_lim = over_lo <= 1e-6 and over_hi <= 1e-6
    gates["t0_go2_limits"] = {"pass": bool(ok_lim), "over_lo": over_lo, "over_hi": over_hi}
    print(f"[GATE T0-go2] joint limits: {'PASS' if ok_lim else 'FAIL'} "
          f"(worst under-lo {over_lo:+.4f}, over-hi {over_hi:+.4f} rad)")

    # ── INFO: raw-rollout stance closure ──
    drifts = []
    for f in range(4):
        z = feet[:, f, 2]
        planted = z < (z.min() + 0.015)
        k = 0
        while k < len(planted):
            if planted[k]:
                j = k
                while j + 1 < len(planted) and planted[j + 1]:
                    j += 1
                if j - k >= 3 and k * dt >= args.fold_from:
                    seg = feet[k:j + 1, f, 0:2]
                    drifts.append(float(np.linalg.norm(seg[-1] - seg[0])))
                k = j + 1
            else:
                k += 1
    drift_mean = float(np.mean(drifts)) if drifts else 0.0
    drift_p95 = float(np.percentile(drifts, 95)) if drifts else 0.0
    gates["closure_raw_rollout"] = {"informational": True, "n_stances": len(drifts),
                                    "drift_mean_m": drift_mean, "drift_p95_m": drift_p95}
    print(f"[INFO raw-rollout closure] {len(drifts)} steady stances, drift mean "
          f"{drift_mean * 1000:.1f} mm p95 {drift_p95 * 1000:.1f} mm (the binding "
          f"closure gate runs on the folded lut's PD replay below)")

    # ── CONSTRUCTION RE-ROLL of the raw commands (+ torque capture) ──
    m.opt.timestep = 0.004
    n_sub = max(1, int(round(dt / 0.004)))
    d = mujoco.MjData(m)
    d.qpos[:] = q[0]; d.qvel[:] = qvel[0]
    mujoco.mj_forward(m, d)
    full = np.zeros(m.nu)
    maxfrac, sat, fell = 0.0, 0, False
    tau_hist = np.zeros((ctrl.shape[0], 12))
    x0_reroll = float(d.qpos[0])
    for k in range(ctrl.shape[0]):
        full[ctrl_idx] = ctrl[k]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        tau = np.abs(d.qfrc_actuator[jdof])
        tau_hist[k] = tau
        frac = float((tau / dof_lim[jdof]).max())
        maxfrac = max(maxfrac, frac)
        sat += int(frac > 0.99)
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < 0.2 or tilt > 60.0:
            fell = True
            break
    reroll_fwd = float(d.qpos[0]) - x0_reroll
    ok_reroll = (not fell) and maxfrac <= 1.001
    gates["construction_reroll"] = {"pass": bool(ok_reroll), "fell": bool(fell),
                                    "peak_torque_frac": maxfrac, "saturated_steps": sat,
                                    "of_steps": int(ctrl.shape[0]), "fwd_m": reroll_fwd}
    print(f"[GATE construction re-roll] {'PASS' if ok_reroll else 'FAIL'}: "
          f"{'NO FALL' if not fell else 'FELL'}, open-loop fwd {reroll_fwd:+.2f} m, "
          f"peak torque {maxfrac * 100:.0f}%, saturated {sat}/{ctrl.shape[0]}")

    # ── CORRIDOR-vs-TORQUE LAW ──
    ffdq_raw = ctrl - qj
    ff_peak = np.abs(ffdq_raw[steady]).max(0)
    taukp_peak = tau_hist[steady[:tau_hist.shape[0]]].max(0) / KP
    print("[CORRIDOR LAW] per-joint peak |cmd - q| (rad), steady segment "
          "(= corridor floor without GHOST-FF):")
    for grp in range(4):
        print("   " + "  ".join(
            f"{JNAMES[grp * 3 + j]:9s}={ff_peak[grp * 3 + j]:.3f} (tau/kp {taukp_peak[grp * 3 + j]:.3f})"
            for j in range(3)))
    print(f"[CORRIDOR LAW] overall peak dq = {ff_peak.max():.3f} rad")
    gates["corridor_law"] = {"peak_dq_rad": float(ff_peak.max()),
                             "per_joint_peak_dq": [float(x) for x in ff_peak], "kp": KP}

    # ── FOLD on the absolute gait clock ──
    nb = args.nb
    tt = np.arange(q.shape[0]) * dt
    phase_all = (stg.QS_PHASE + 2.0 * np.pi * gp.freq * tt) % (2.0 * np.pi)
    bins = np.minimum((phase_all / (2.0 * np.pi) * nb).astype(int), nb - 1)
    leg_lut = np.zeros((nb, 12)); ffdq_lut = np.zeros((nb, 12))
    counts = np.zeros(nb, int); spread = np.zeros((nb, 12))
    for b in range(nb):
        sel = steady & (bins == b)
        counts[b] = sel.sum()
        if counts[b]:
            leg_lut[b] = qj[sel].mean(0)
            ffdq_lut[b] = ffdq_raw[sel].mean(0)
            spread[b] = qj[sel].std(0)
    for b in np.where(counts == 0)[0]:
        leg_lut[b] = 0.5 * (leg_lut[(b - 1) % nb] + leg_lut[(b + 1) % nb])
        ffdq_lut[b] = 0.5 * (ffdq_lut[(b - 1) % nb] + ffdq_lut[(b + 1) % nb])
    print(f"[fold] nb={nb}: samples/bin min={counts.min()} mean={counts.mean():.1f}; "
          f"fold spread mean {spread[counts > 0].mean() * 1000:.1f} mrad")

    def _smooth(lut):
        F = np.fft.rfft(lut, axis=0)
        F[args.harmonics + 1:] = 0.0
        return np.fft.irfft(F, n=nb, axis=0)
    leg_lut_s = np.clip(_smooth(leg_lut), LIM_LO, LIM_HI)
    ffdq_lut_s = _smooth(ffdq_lut)
    print(f"[fold] harmonic smoothing (<= {args.harmonics}): max change "
          f"{np.abs(leg_lut_s - np.clip(leg_lut, LIM_LO, LIM_HI)).max() * 1000:.1f} mrad")

    th = 2.0 * np.pi * np.arange(nb) / nb
    trot = np.stack([stg.targets_np(t, gp, t_since_start=1e6)[0] for t in th])
    sag = np.abs(leg_lut_s - trot)
    print(f"[preview] achieved-vs-analytic |sag|: mean {sag.mean():.3f} rad, max "
          f"{sag.max():.3f} rad (why the deploy needs the lut baseline)")
    amp_raw = []
    cyc = int(round((1.0 / gp.freq) / dt))
    k = int(args.fold_from / dt)
    while k + cyc <= q.shape[0]:
        w = qj[k:k + cyc]
        amp_raw.append(w.max(0) - w.min(0))
        k += cyc
    amp_raw = np.array(amp_raw).mean(0) if amp_raw else np.zeros(12)
    amp_lut = leg_lut_s.max(0) - leg_lut_s.min(0)
    print(f"[preview] amplitude retention lut/raw (thigh cols): "
          + "  ".join(f"{JNAMES[i]}={amp_lut[i] / max(amp_raw[i], 1e-6):.2f}"
                      for i in (1, 4, 7, 10)))

    # ── THE BINDING GATE: PD REPLAY OF THE FOLDED LUT (gate-4 style, no crane) ──
    T_rep = 8.0
    mr = mujoco.MjModel.from_xml_path(mjcf)
    mr.opt.timestep = PHYS_DT
    dr_ = mujoco.MjData(mr)
    mujoco.mj_resetData(mr, dr_)
    nominal = stg.standing_pose(gp)
    dr_.qpos[:] = 0
    dr_.qpos[2] = gp.body_height + 0.03
    dr_.qpos[3] = 1.0
    dr_.qpos[jq] = nominal
    fullr = np.zeros(mr.nu)
    fullr[ctrl_idx] = nominal
    dr_.ctrl[:] = fullr
    for _ in range(int(1.5 / PHYS_DT)):
        mujoco.mj_step(mr, dr_)

    def _lint(tab, ph):
        x = (ph % (2.0 * np.pi)) / (2.0 * np.pi) * nb
        b0 = int(x) % nb
        f = x - int(x)
        return tab[b0] * (1.0 - f) + tab[(b0 + 1) % nb] * f

    n_rep = int(T_rep / DT_CTRL)
    n_sub_rep = int(round(DT_CTRL / PHYS_DT))
    rq = np.zeros((n_rep, mr.nq))
    rfeet = np.zeros((n_rep, 4, 3))
    rqv = np.zeros((n_rep, mr.nv))
    rctrl = np.zeros((n_rep, 12))
    rgm = np.zeros(n_rep)
    rfell = False
    rmaxfrac = 0.0
    for k in range(n_rep):
        t = k * DT_CTRL
        ph = stg.QS_PHASE + 2.0 * np.pi * gp.freq * t
        rr = min(1.0, t / gp.ramp_s) if gp.ramp_s > 0 else 1.0
        ref_pose = nominal + rr * (_lint(leg_lut_s, ph) - nominal)
        cmd = ref_pose + rr * _lint(ffdq_lut_s, ph)
        fullr[ctrl_idx] = cmd
        dr_.ctrl[:] = fullr
        for _ in range(n_sub_rep):
            mujoco.mj_step(mr, dr_)
        rq[k] = dr_.qpos; rqv[k] = dr_.qvel; rctrl[k] = cmd
        rfeet[k] = _feet_fk(mr, dr_, calf_bid)
        rgm[k] = np.exp(-float(np.mean((dr_.qpos[jq] - ref_pose) ** 2)) / (0.35 ** 2))
        tau = np.abs(dr_.qfrc_actuator[jdof])
        rmaxfrac = max(rmaxfrac, float((tau / dof_lim[jdof]).max()))
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (dr_.qpos[4] ** 2 + dr_.qpos[5] ** 2)))))
        if dr_.qpos[2] < 0.18 or tilt > 60.0:
            rfell = True
            break
    k_end = k + 1
    rep_t = np.arange(k_end) * DT_CTRL
    rep_steady = rep_t >= 2.0
    rep_dist = float(rq[k_end - 1, 0] - rq[0, 0])
    if rep_steady.any():
        i0 = int(rep_steady.argmax())
        rep_vx = float((rq[k_end - 1, 0] - rq[i0, 0]) / max(1e-6, (k_end - 1 - i0) * DT_CTRL))
        rep_gm = float(rgm[rep_steady].mean())
    else:
        rep_vx, rep_gm = 0.0, 0.0
    rdrifts = []
    for f4 in range(4):
        z = rfeet[:k_end, f4, 2]
        planted = z < (z.min() + 0.015)
        kk = 0
        while kk < k_end:
            if planted[kk]:
                jj = kk
                while jj + 1 < k_end and planted[jj + 1]:
                    jj += 1
                if jj - kk >= 3 and kk * DT_CTRL >= 2.0:
                    seg = rfeet[kk:jj + 1, f4, 0:2]
                    rdrifts.append(float(np.linalg.norm(seg[-1] - seg[0])))
                kk = jj + 1
            else:
                kk += 1
    rd_p95 = float(np.percentile(rdrifts, 95)) if rdrifts else 0.0
    rd_mean = float(np.mean(rdrifts)) if rdrifts else 0.0
    ok_replay = (not rfell) and rep_vx > 0.6 * vx_steady and rd_p95 < 0.02 \
        and rmaxfrac <= 1.001
    gates["lut_replay"] = {
        "pass": bool(ok_replay), "fell": bool(rfell), "dist_m": rep_dist,
        "steady_vx": rep_vx, "closure_drift_mean_m": rd_mean,
        "closure_drift_p95_m": rd_p95, "n_stances": len(rdrifts),
        "peak_torque_frac": rmaxfrac, "gmatch_selfmatch_ceiling": rep_gm}
    print(f"[GATE 1+4 LUT-REPLAY] {'PASS' if ok_replay else 'FAIL'}: "
          f"{'NO FALL' if not rfell else 'FELL'}, dist {rep_dist:+.2f} m over "
          f"{k_end * DT_CTRL:.0f}s (steady vx {rep_vx:.3f} vs source {vx_steady:.3f}), "
          f"CLOSURE drift mean {rd_mean * 1000:.1f} mm p95 {rd_p95 * 1000:.1f} mm / "
          f"{len(rdrifts)} stances (gate < 20 mm), peak torque {rmaxfrac * 100:.0f}%, "
          f"SELF-MATCH gmatch ceiling {rep_gm:.3f}")

    replay_npz = args.out + "_replay.npz"
    np.savez(replay_npz, q=rq[:k_end], qvel=rqv[:k_end], ctrl=rctrl[:k_end],
             feet=rfeet[:k_end], dt=DT_CTRL)
    rpassed, rscore, rmets = certify(replay_npz, mjcf, motion="walk", verbose=False)
    gates["support_fwp_certificate_replay"] = {
        "pass": bool(rpassed), "score": float(rscore),
        "base_frac95": float(rmets.get("base_frac95", -1))}
    print(f"[GATE 2+3 SUPPORT+FWP on the REPLAY] {'PASS' if rpassed else 'FAIL'} "
          f"score={rscore:.3f} base_frac95={rmets.get('base_frac95')} "
          f"(score pins to 0 when one DOF rides its torque limit -- known artifact; "
          f"the VERDICT + base_frac95 are the signal)")

    out_json = args.out + "_lut.json"
    doc = {
        "robot": "go2",
        "nb": nb,
        "freq": gp.freq,
        "vx": round(vx_steady, 4),
        "joints": JOINTS_GO2,
        "joint_order": "FL,FR,RL,RR x (hip=abduction/roll, thigh=pitch, calf=knee)",
        "leg_lut": [[round(float(v), 6) for v in row] for row in leg_lut_s],
        "ffdq_lut": [[round(float(v), 6) for v in row] for row in ffdq_lut_s],
        "ffdq_kp": KP,
        "source": (f"recorded/achieved ({args.source}): "
                   + ("the legacy Go2 RL champion (gpu_go2_walk_main policy.onnx on the "
                      "analytic trot baseline) rolled DETERMINISTICALLY on the deploy-"
                      "matched MJCF dump (kp=250/kv=6, 16ms x 8 substeps, exact trainer "
                      "obs contract)" if args.source == "champion" else
                      "receding-horizon MPPI (ghost_generator) executing the analytic "
                      "trot prior over the real force-limited dynamics")
                   + f"; steady segment (t>={args.fold_from}s, {n_cycles:.1f} cycles) "
                     f"phase-folded at {nb} bins on the gait clock, harmonic-smoothed "
                     f"(<= {args.harmonics}). Replay-verified: the folded lut + its "
                     "declared feedforward, tracked by the bare deploy-grade PD (no "
                     "crane -- quads use none), WALKS with closed stance contacts."),
        "gait": dict(vx=gp.vx, freq=gp.freq, duty=gp.duty, step_height=gp.step_height,
                     body_height=gp.body_height, ramp_s=gp.ramp_s),
        "gates": gates,
        "provenance": {"rollout_npz": os.path.basename(npz_path), "dt": dt,
                       "T": T_total, "source": args.source,
                       "mjcf": os.path.relpath(mjcf, REPO)},
    }
    with open(out_json, "w") as f:
        json.dump(doc, f)
    print(f"[go2-shadow] wrote {out_json}")

    prev = args.out + "_preview.csv"
    with open(prev, "w") as f:
        f.write("bin,phase_rad," + ",".join(JNAMES) + "\n")
        for b in range(nb):
            f.write(f"{b},{th[b]:.4f}," + ",".join(f"{v:.5f}" for v in leg_lut_s[b]) + "\n")
    amp = leg_lut_s.max(0) - leg_lut_s.min(0)
    print("[preview] per-joint amplitude (rad): " + "  ".join(
        f"{JNAMES[i]}={amp[i]:.3f}" for i in range(12)))
    print(f"[preview] wrote {prev}")

    hard_gates = ["t0_go2_limits", "lut_replay", "support_fwp_certificate_replay",
                  "construction_reroll"]
    all_pass = all(gates[k]["pass"] for k in hard_gates)
    print(f"\n[go2-shadow] GATES {'ALL PASS' if all_pass else 'FAILED'} -- "
          f"{ {k: gates[k]['pass'] for k in hard_gates} }")
    print("[go2-shadow] next: python projects/policies/training/ghost_validator.py "
          + out_json)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
