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

"""FULL shadowing Component 1 on a HARD non-gait motion: OmniQuad GET-UP from the ground.

Unlike the quasi-static crouch (which deploys feedforward), a get-up is a real
CONTACT HANDOFF -- the trunk rests on the floor, then the legs tuck under and push
the body up onto the feet. Feedforward can't do it (it needs feedback), so this is the
motion that genuinely earns the RL tracker (Component 3).

Pipeline here = Component 1 (+2): drop OmniQuad into a folded LYING pose and settle (trunk
on the ground), then receding-horizon MPPI DISCOVERS a feasible lying->stand get-up
over the real dynamics + trunk/foot contacts, and ghost_verifier certifies it.

OmniQuad dump has anonymous interleaved actuators + no foot/_pos names -> we OVERRIDE the
GhostGenerator's mappings on the instance (no edits to the shared generator).

Run: python projects/policies/research/tools/generate_omniquad_getup.py [--secs 4 --fold-knee -1.1 ...]
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
from projects.policies.control.gait import omniquad_trot_gait as stg                       # noqa: E402
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent  # noqa: E402
from projects.policies.research.shadowing.ghost_verifier import _inverse_dynamics_cert   # noqa: E402

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(_REPO / "projects/policies/research/training/mjcf/omniquad_newton_fixed2.xml"))
    ap.add_argument("--stand-z", type=float, default=0.55)
    ap.add_argument("--mid-z", type=float, default=0.32)      # tucked intermediate height
    ap.add_argument("--fold-knee", type=float, default=-0.40)  # knee in the lying pose
    ap.add_argument("--fold-hipy", type=float, default=0.30)   # thigh pitch in the lying pose
    ap.add_argument("--fold-hipx", type=float, default=1.30)   # hip ROLL splay (legs out to sides)
    ap.add_argument("--drop-z", type=float, default=0.32)      # base z to drop from (then settle)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--horizon", type=int, default=25)
    ap.add_argument("--noise", type=float, default=0.18)
    ap.add_argument("--balance", type=float, default=6.0)
    ap.add_argument("--upright", type=float, default=4.0)
    ap.add_argument("--settle-only", action="store_true")
    ap.add_argument("--out", default=str(_REPO / "_scratch" / "omniquad_getup_ghost.npz"))
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

    stand_legs = stg.standing_pose(stg.GaitParams(body_height=a.stand_z))
    mid_legs = stg.standing_pose(stg.GaitParams(body_height=a.mid_z))

    # --- lying init: folded legs, drop the trunk onto the floor, settle ---
    mujoco.mj_resetData(gen.m, gen.d)
    # splay the legs OUT to the sides (hip ROLL, alternating L/R sign) so the legs
    # don't prop the body -> the trunk drops flat onto the floor.
    _hxs = [+1.0, -1.0, +1.0, -1.0]                          # FL,FR,RL,RR
    fold = np.array([[s * a.fold_hipx, a.fold_hipy, a.fold_knee]
                     for s in _hxs]).reshape(-1)
    gen.d.qpos[0:3] = [0.0, 0.0, a.drop_z]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(NJ):
        gen.d.qpos[qpos_adr[i]] = fold[i]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = np.clip(fold, JL_LO, JL_HI)
    gen.d.ctrl[:] = full
    for _ in range(int(1.2 / gen.sim_dt)):                    # settle onto the belly
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    lying_legs = init[qpos_adr].copy()
    print(f"[getup] lying settled: base_z={init[2]:.3f}  tilt={_tilt(init):.1f}deg  "
          f"knees={np.round(lying_legs[2::3],2).tolist()}")
    if a.settle_only:
        return 0

    # --- intent: lying -> tuck (mid) -> stand -> hold ---
    T = a.secs
    intent = Intent(
        total_time=T,
        joint_keys=[(0.0, _dict(lying_legs)), (0.40 * T, _dict(mid_legs)),
                    (0.75 * T, _dict(stand_legs)), (T, _dict(stand_legs))],
        base_keys=[(0.0, {"z": float(init[2]), "pitch": 0.0}),
                   (0.40 * T, {"z": a.mid_z, "pitch": 0.0}),
                   (0.75 * T, {"z": a.stand_z, "pitch": 0.0}),
                   (T, {"z": a.stand_z, "pitch": 0.0})],
        weights=dict(joint=5.0, base_z=14.0, base_xy=3.0, pitch=3.0,
                     upright=a.upright, balance=a.balance, effort=0.02, smooth=0.05, alive=0.5),
    )
    g = gen.generate(intent, init, n_samples=a.samples, horizon=a.horizon,
                     noise=a.noise, temperature=0.5, seed=0)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    np.savez(a.out, **{k: v for k, v in g.items() if k != "joints"}, joints=np.array(g["joints"]))
    bz = g["base"][:, 2]
    print(f"[getup] generated {g['q'].shape[0]} steps; base_z {bz[0]:.3f} -> max {bz.max():.3f} "
          f"-> end {bz[-1]:.3f}  (stand target {a.stand_z})")
    rose = bz[-1] - bz[0]
    got_up = bz[-1] > a.stand_z - 0.10

    # ── verify: open-loop replay of the generated targets (correct OmniQuad actuators) ──
    m = mujoco.MjModel.from_xml_path(a.mjcf)
    m.opt.timestep = 0.004
    d = mujoco.MjData(m)
    n_sub = max(1, int(round(float(g["dt"]) / 0.004)))
    d.qpos[:] = g["q"][0]; d.qvel[:] = g["qvel"][0]; mujoco.mj_forward(m, d)
    full = np.zeros(m.nu)
    fell_at, bz_end_ol = None, 0.0
    for k in range(g["ctrl"].shape[0]):
        full[:] = 0.0
        full[gen.pos_act] = g["ctrl"][k]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        if _tilt(d.qpos) > 80.0:
            fell_at = k * float(g["dt"]); break
    bz_end_ol = float(d.qpos[2])

    print("=" * 72)
    print("OMNIQUAD GET-UP GHOST -- MPPI-GENERATED (shadowing C1) + verify")
    print("=" * 72)
    print(f"[generation]  rose {rose:+.3f} m, final base_z={bz[-1]:.3f} -> "
          f"{'STOOD UP' if got_up else 'did NOT reach stand'}")
    print(f"[open-loop replay] final base_z={bz_end_ol:.3f}  "
          f"{'collapsed at t=%.2fs (needs feedback = OK for RL if it rose first)' % fell_at if fell_at else 'sustained'}")
    try:
        _inverse_dynamics_cert(g, a.mjcf, 0.004, verbose=True)
    except Exception as ex:
        print(f"  [inverse-dyn] skipped ({ex})")
    print("-" * 72)
    ok = got_up
    print(f"VERDICT: {'C1 OK -- MPPI found a get-up -> send to RL tracker (C3)' if ok else 'RETRY -- MPPI did not stand up (tune keyframes/weights/samples)'}")
    print(f"saved -> {a.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
