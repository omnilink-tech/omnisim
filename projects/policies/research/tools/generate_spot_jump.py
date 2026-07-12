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

"""HARDEST shadowing test: a Spot FORWARD LEAP with a real FLIGHT phase.

Pushes the pipeline past where the crouch/get-up lived (all feet on the ground):
  - C1: MPPI must find an EXPLOSIVE push-off -> ballistic flight -> landing. We
    drive it with a LAUNCH-VELOCITY task objective (the same Intent.task mechanism
    an arm toss uses to fling a payload) -- here it drives the base's own forward+
    up velocity during the launch window, so MPPI discovers the leg extension that
    achieves it. Keyframes give the phase structure (load -> extend -> tuck -> land).
  - This will EXPOSE limits: the verifier's open-loop "doesn't fall" gate is
    meaningless in flight (a leap IS a controlled fall); the tracker's tiny residual
    vs an explosive launch; the worst mujoco_warp->Newton gap (landing impact).

Override the GhostGenerator mappings on the instance (anonymous interleaved Spot dump),
same as generate_spot_getup.py.

Run: python projects/policies/research/tools/generate_spot_jump.py [--vx-launch 1.0 --vz-launch 1.7 ...]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import mujoco

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.control.gait import spot_trot_gait as stg                       # noqa: E402
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent  # noqa: E402

NJ = 12
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip_x", "hip_y", "knee")
LOWER_LEG_BODIES = (4, 7, 10, 13)
JL_LO = np.array([-1.50, -0.50, -1.20] * 4)
JL_HI = np.array([+1.50, +3.13, -0.01] * 4)


def _tilt(qpos):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2)))))


def _classify(m, d):
    label, k = {}, 0
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        ax, rng, pos = m.jnt_axis[j], m.jnt_range[j], d.xpos[m.jnt_bodyid[j]]
        leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
               "FR" if pos[0] > 0 else "RL" if pos[1] > 0 else "RR")
        jt = "hip_x" if abs(ax[0]) > 0.5 else "knee" if rng[0] < -0.9 else "hip_y"
        label[(leg, jt)] = (m.jnt_qposadr[j], 2 * k)
        k += 1
    qa, ci = np.zeros(NJ, int), np.zeros(NJ, int)
    for a, leg in enumerate(LEGS):
        for b, jt in enumerate(JOINTS):
            qa[a * 3 + b], ci[a * 3 + b] = label[(leg, jt)]
    return qa, ci


def _dict(v):
    return {f"{leg}_{jt}": float(v[a * 3 + b])
            for a, leg in enumerate(LEGS) for b, jt in enumerate(JOINTS)}


def _symmetrize(ctrl):
    """Force L/R mirror symmetry per (front,rear) so a forward leap is sagittal-plane
    clean (no twist/roll). hip_x mirrors (opposite sign); hip_y/knee are equal L<->R.
    Indices: FL 0,1,2  FR 3,4,5  RL 6,7,8  RR 9,10,11."""
    c = ctrl.copy()
    for (L, R) in ((0, 3), (6, 9)):          # hip_x: opposite sign
        m = 0.5 * (c[:, L] - c[:, R])
        c[:, L], c[:, R] = m, -m
    for (L, R) in ((1, 4), (2, 5), (7, 10), (8, 11)):   # hip_y, knee: equal
        a = 0.5 * (c[:, L] + c[:, R])
        c[:, L] = c[:, R] = a
    return c


def _smooth(ctrl, win=3):
    """Mild centered moving-average over time per joint (kills MPPI jitter; small
    window preserves the sharp launch extension)."""
    if win < 2:
        return ctrl
    k = np.ones(win) / win
    pad = win // 2
    out = np.empty_like(ctrl)
    for j in range(ctrl.shape[1]):
        p = np.pad(ctrl[:, j], pad, mode="edge")
        out[:, j] = np.convolve(p, k, mode="valid")[:ctrl.shape[0]]
    return out


def _reroll(gen, init, ctrl, mujoco):
    """Forward-sim the (cleaned) position targets open-loop -> a CONSISTENT, feasible
    ghost (q/qvel/base/feet match the cleaned ctrl). Returns the new ghost dict."""
    n_sub = gen.n_sub
    d = mujoco.MjData(gen.m)
    d.qpos[:] = init
    mujoco.mj_forward(gen.m, d)
    full = np.zeros(gen.m.nu)
    Q, QV, BASE, FEET, COM = [], [], [], [], []
    for k in range(ctrl.shape[0]):
        full[gen.pos_act] = np.clip(ctrl[k], gen.ctrl_lo, gen.ctrl_hi)
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(gen.m, d)
        Q.append(d.qpos.copy()); QV.append(d.qvel.copy()); BASE.append(d.qpos[:7].copy())
        COM.append(d.subtree_com[0].copy())
        FEET.append(np.array([d.xpos[b].copy() for b in LOWER_LEG_BODIES]))
    return dict(dt=gen.control_dt, q=np.array(Q), qvel=np.array(QV), ctrl=ctrl,
                base=np.array(BASE), com=np.array(COM), feet=np.array(FEET))


def symmetric_generate(gen, intent, init, n_samples, horizon, noise, temperature, seed):
    """MPPI that searches ONLY L/R-symmetric controls: a 6-DOF basis
    [front: hip_x,hip_y,knee ; rear: hip_x,hip_y,knee] expanded to 12 with the left
    legs mirroring the right (hip_x opposite, hip_y/knee equal). MPPI then optimises a
    strong symmetric launch BY CONSTRUCTION (no twist), reusing the generator's
    _rollout_cost (keyframes + balance + the launch task)."""
    rng = np.random.default_rng(seed)
    stand = init[gen.act_jqpos]                       # 12, controller order (settled stand)
    nom6 = np.array([stand[0], stand[1], stand[2], stand[6], stand[7], stand[8]])
    nominal = np.tile(nom6, (horizon, 1))
    n_steps = int(round(intent.total_time / gen.control_dt))
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[:] = init
    mujoco.mj_forward(gen.m, gen.d)
    scratch = mujoco.MjData(gen.m)

    def expand(s6):                                   # (...,6) -> (...,12)
        fhx, fhy, fk, rhx, rhy, rk = [s6[..., i] for i in range(6)]
        return np.stack([fhx, fhy, fk, -fhx, fhy, fk, rhx, rhy, rk, -rhx, rhy, rk], axis=-1)

    Q, QV, C, BASE, COM, FEET = [], [], [], [], [], []
    for k in range(n_steps):
        t0 = k * gen.control_dt
        dU = rng.normal(0.0, noise, size=(n_samples, horizon, 6))
        for h in range(1, horizon):
            dU[:, h] = 0.6 * dU[:, h] + 0.4 * dU[:, h - 1]
        samples = nominal[None] + dU
        samples[0] = nominal
        costs = np.empty(n_samples)
        for s in range(n_samples):
            scratch.qpos[:] = gen.d.qpos
            scratch.qvel[:] = gen.d.qvel
            scratch.time = gen.d.time
            mujoco.mj_forward(gen.m, scratch)
            U12 = np.clip(expand(samples[s]), gen.ctrl_lo, gen.ctrl_hi)
            costs[s] = gen._rollout_cost(scratch, U12, intent, t0)
        beta = costs.min()
        w = np.exp(-(costs - beta) / max(1e-6, temperature))
        w /= w.sum()
        nominal = np.einsum("s,shc->hc", w, samples)
        u12 = np.clip(expand(nominal[0]), gen.ctrl_lo, gen.ctrl_hi)
        full = np.zeros(gen.m.nu)
        full[gen.pos_act] = u12
        gen.d.ctrl[:] = full
        for _ in range(gen.n_sub):
            mujoco.mj_step(gen.m, gen.d)
        Q.append(gen.d.qpos.copy()); QV.append(gen.d.qvel.copy()); C.append(u12.copy())
        BASE.append(gen.d.qpos[:7].copy()); COM.append(gen.d.subtree_com[0].copy())
        FEET.append(np.array([gen.d.xpos[b].copy() for b in LOWER_LEG_BODIES]))
        nominal = np.vstack([nominal[1:], nominal[-1:]])
        if k % 25 == 0 or k == n_steps - 1:
            print(f"  [symgen] t={t0:4.2f}s base_z={gen.d.qpos[2]:.3f} cost={costs.min():.2f}")
    return dict(dt=gen.control_dt, q=np.array(Q), qvel=np.array(QV), ctrl=np.array(C),
                base=np.array(BASE), com=np.array(COM), feet=np.array(FEET))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(_REPO / "projects/policies/research/training/mjcf/spot_newton_fixed2.xml"))
    ap.add_argument("--load-z", type=float, default=0.34)     # crouch/load depth
    ap.add_argument("--ext-z", type=float, default=0.60)      # leg extension at launch
    ap.add_argument("--tuck-z", type=float, default=0.42)     # tucked legs in flight
    ap.add_argument("--vx-launch", type=float, default=1.0)   # target forward launch vel
    ap.add_argument("--vz-launch", type=float, default=1.7)   # target up launch vel
    ap.add_argument("--reach", type=float, default=0.8)       # forward distance of the leap
    ap.add_argument("--secs", type=float, default=1.6)
    ap.add_argument("--samples", type=int, default=300)
    ap.add_argument("--horizon", type=int, default=30)
    ap.add_argument("--noise", type=float, default=0.22)
    ap.add_argument("--w-launch", type=float, default=40.0)   # launch-velocity cost weight
    ap.add_argument("--smooth-win", type=int, default=3)      # control moving-avg window
    ap.add_argument("--no-clean", action="store_true",        # skip symmetrize+smooth+reroll
                    help="save the raw (chaotic) MPPI ghost instead of the cleaned one")
    ap.add_argument("--out", default=str(_REPO / "_scratch" / "spot_jump_ghost.npz"))
    a = ap.parse_args()

    gen = GhostGenerator(a.mjcf, sim_dt=0.004, control_dt=0.02)
    mujoco.mj_forward(gen.m, gen.d)
    qpos_adr, ctrl_pos = _classify(gen.m, gen.d)
    gen.pos_act = [int(c) for c in ctrl_pos]
    gen.act_jqpos = qpos_adr
    gen.act_name = [f"{leg}_{jt}_pos" for leg in LEGS for jt in JOINTS]
    gen.ctrl_lo, gen.ctrl_hi = JL_LO.copy(), JL_HI.copy()
    gen.foot_bodies = list(LOWER_LEG_BODIES)
    gen.fixed_base = False

    stand = stg.standing_pose(stg.GaitParams(body_height=0.55))
    load = stg.standing_pose(stg.GaitParams(body_height=a.load_z))
    ext = stg.standing_pose(stg.GaitParams(body_height=a.ext_z))
    tuck = stg.standing_pose(stg.GaitParams(body_height=a.tuck_z))

    # init: settle standing
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, 0.56]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(NJ):
        gen.d.qpos[qpos_adr[i]] = stand[i]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = stand
    gen.d.ctrl[:] = full
    for _ in range(int(0.3 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[jump] settled stand base_z={init[2]:.3f}")

    T = a.secs
    # phase fractions of T: load -> launch (extend) -> flight (tuck) -> land -> settle
    t_load, t_ext, t_flight, t_land = 0.22 * T, 0.34 * T, 0.50 * T, 0.78 * T
    intent = Intent(
        total_time=T,
        joint_keys=[(0.0, _dict(stand)), (t_load, _dict(load)), (t_ext, _dict(ext)),
                    (t_flight, _dict(tuck)), (t_land, _dict(stand)), (T, _dict(stand))],
        base_keys=[(0.0, {"x": 0.0, "z": 0.55, "pitch": 0.0}),
                   (t_load, {"x": 0.05, "z": a.load_z, "pitch": 0.0}),
                   (t_ext, {"x": 0.20, "z": 0.62, "pitch": -0.1}),
                   (t_flight, {"x": 0.45, "z": 0.62, "pitch": 0.0}),
                   (t_land, {"x": a.reach, "z": 0.50, "pitch": 0.0}),
                   (T, {"x": a.reach, "z": 0.55, "pitch": 0.0})],
        weights=dict(joint=3.0, base_xy=4.0, base_z=6.0, pitch=2.0,
                     upright=2.0, balance=2.0, effort=0.02, smooth=0.05, alive=0.5),
    )

    # LAUNCH task: during [t_load, t_ext] drive base linear velocity (qvel[0:3]) to
    # (vx_launch, 0, vz_launch). This is what makes MPPI EXTEND explosively + leave
    # the ground (the same task mechanism the toss uses to fling a payload).
    vtar = np.array([a.vx_launch, 0.0, a.vz_launch])

    def jump_task(g, d, t):
        if t_load <= t <= t_ext + 0.04:
            v = d.qvel[0:3]
            return a.w_launch * float(np.sum((v - vtar) ** 2))
        return 0.0
    intent.task = jump_task

    if a.no_clean:
        g = gen.generate(intent, init, n_samples=a.samples, horizon=a.horizon,
                         noise=a.noise, temperature=0.5, seed=0)
    else:
        # SYMMETRIC MPPI: search only L/R-mirrored controls -> a strong, twist-free
        # (realistic) leap by construction, instead of the chaotic asymmetric one.
        g = symmetric_generate(gen, intent, init, a.samples, a.horizon, a.noise, 0.5, 0)
        if a.smooth_win >= 2:
            g = _reroll(gen, init, _smooth(g["ctrl"], a.smooth_win), mujoco)
    asym = float(np.max(np.abs(g["ctrl"][:, [1, 2, 7, 8]] - g["ctrl"][:, [4, 5, 10, 11]])))
    print(f"[symgen] L/R asym = {asym:.4f} rad  (0 = perfectly symmetric)")
    joints = np.array([f"{leg}_{jt}" for leg in LEGS for jt in JOINTS])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    np.savez(a.out, **{k: v for k, v in g.items() if k != "joints"}, joints=joints)

    # ── jump-aware analysis (the open-loop "doesn't fall" gate is meaningless here) ──
    bz = g["base"][:, 2]
    bx = g["base"][:, 0]
    qv = g["qvel"]
    peak = float(bz.max())
    vz_peak = float(qv[:, 2].max())
    vx_peak = float(qv[:, 0].max())
    dx = float(bx[-1] - bx[0])
    # FLIGHT = all four feet above the ground (foot tip z > a small clearance).
    feet = g["feet"]                       # (T,4,3) lower-leg body pos; foot tip ~0.32 below
    foot_z = feet[:, :, 2] - stg.L2
    airborne = (foot_z.min(axis=1) > 0.04)
    flight_steps = int(airborne.sum())
    flight_s = flight_steps * float(g["dt"])
    end_z = float(bz[-1]); end_tilt = _tilt(g["q"][-1])
    print("=" * 72)
    print("SPOT FORWARD-LEAP GHOST -- MPPI (shadowing C1, with a FLIGHT phase)")
    print("=" * 72)
    print(f"  peak base_z={peak:.3f}  launch vel: vx_peak={vx_peak:.2f} vz_peak={vz_peak:.2f} m/s")
    print(f"  FLIGHT (all 4 feet airborne): {flight_s:.2f}s ({flight_steps} steps)")
    print(f"  forward leap dx={dx:+.2f} m;  landing base_z={end_z:.3f} tilt={end_tilt:.1f}deg")
    leapt = flight_steps >= 2 and dx > 0.3
    landed = end_z > 0.40 and end_tilt < 40
    print("-" * 72)
    print(f"VERDICT: {'LEAPT' if leapt else 'NO clear flight phase'}"
          f" + {'LANDED upright' if landed else 'did NOT land clean'} "
          f"-> {'C1 OK -> verify + train' if (leapt and landed) else 'RETRY (tune vz/vx-launch, w-launch, keyframes)'}")
    print(f"saved -> {a.out}")
    return 0 if (leapt and landed) else 1


if __name__ == "__main__":
    sys.exit(main())
