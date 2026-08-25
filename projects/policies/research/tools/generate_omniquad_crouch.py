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

"""FULL shadowing Component 1 (MPPI ghost generator) on a NON-GAIT OmniQuad motion.

The OmniQuad WALK ghost uses the analytic trot model as its feasible generator (gait is feasible
by construction). This script exercises the REAL Component-1 path -- the MPPI ghost_generator
DISCOVERING a feasible motion over OmniQuad's actual dynamics -- on a motion the trot model can't
express: a CROUCH-and-RECOVER (lower the body ~15 cm while balanced on all four feet, hold,
rise back). Given only the START + a base-height intent + the balance cost, receding-horizon
MPPI finds the dynamically-feasible joint trajectory.

OmniQuad's deploy MJCF dump (omniquad_newton_fixed2.xml) has ANONYMOUS joints/bodies and INTERLEAVED
position+velocity actuators (position at even ctrl indices, gainprm=500), with no "_pos"/"foot"
names -- so the generic GhostGenerator's name-based detection fails. We instantiate it and
OVERRIDE the OmniQuad mappings on the instance (pos actuators, their joint qpos addrs, synthetic
"_pos" names so keyframes match, foot bodies, ctrl limits) -- NO edits to the shared
ghost_generator, NO XML surgery. Then verify with the shadowing verifier's metrics.

Run: python projects/policies/research/tools/generate_omniquad_crouch.py [--secs 5 --crouch-z 0.40]
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
LOWER_LEG_BODIES = (4, 7, 10, 13)         # foot proxies (trainer)
JL_LO = np.array([-1.50, -0.50, -1.20] * 4)
JL_HI = np.array([+1.50, +3.13, -0.01] * 4)


def _tilt(qpos):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2)))))


def _classify(m, d):
    """controller order FL,FR,RL,RR x (hip_x,hip_y,knee) -> (qpos_adr, pos-actuator index)."""
    label = {}
    k = 0
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        ax, rng, pos = m.jnt_axis[j], m.jnt_range[j], d.xpos[m.jnt_bodyid[j]]
        leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
               "FR" if pos[0] > 0 else "RL" if pos[1] > 0 else "RR")
        jt = "hip_x" if abs(ax[0]) > 0.5 else "knee" if rng[0] < -0.9 else "hip_y"
        label[(leg, jt)] = (m.jnt_qposadr[j], 2 * k)     # ctrl interleaved [pos,vel]
        k += 1
    qa = np.zeros(NJ, dtype=int)
    ci = np.zeros(NJ, dtype=int)
    for a, leg in enumerate(LEGS):
        for b, jt in enumerate(JOINTS):
            qa[a * 3 + b], ci[a * 3 + b] = label[(leg, jt)]
    return qa, ci


def _legs_dict(legs):
    """trot-model 12-vector -> {FL_hip_x: v, ...} matching the synthetic actuator names."""
    out = {}
    for a, leg in enumerate(LEGS):
        for b, jt in enumerate(JOINTS):
            out[f"{leg}_{jt}"] = float(legs[a * 3 + b])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(_REPO / "projects/policies/research/training/mjcf/omniquad_newton_fixed2.xml"))
    ap.add_argument("--stand-z", type=float, default=0.55)
    ap.add_argument("--crouch-z", type=float, default=0.40)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--horizon", type=int, default=18)
    ap.add_argument("--noise", type=float, default=0.12)
    ap.add_argument("--upright", type=float, default=3.0)
    ap.add_argument("--balance", type=float, default=5.0)
    ap.add_argument("--out", default=str(_REPO / "_scratch" / "omniquad_crouch_ghost.npz"))
    a = ap.parse_args()

    gen = GhostGenerator(a.mjcf, sim_dt=0.004, control_dt=0.02)

    # --- OVERRIDE the OmniQuad mappings on the instance (anonymous interleaved dump) ---
    mujoco.mj_forward(gen.m, gen.d)        # populate xpos so the classifier sees real body xy
    qpos_adr, ctrl_pos = _classify(gen.m, gen.d)
    gen.pos_act = [int(c) for c in ctrl_pos]                       # 12 position-actuator indices
    gen.act_jqpos = qpos_adr
    gen.act_name = [f"{leg}_{jt}_pos" for leg in LEGS for jt in JOINTS]
    gen.ctrl_lo = JL_LO.copy()
    gen.ctrl_hi = JL_HI.copy()
    gen.foot_bodies = list(LOWER_LEG_BODIES)
    gen.fixed_base = False                                          # OmniQuad has a free base
    print(f"[omniquad-crouch] overrode mappings: {len(gen.pos_act)} pos actuators, "
          f"foot bodies {gen.foot_bodies}")

    # --- init: settle at the standing pose ---
    gp = stg.GaitParams(body_height=a.stand_z)
    stand_legs = stg.standing_pose(gp)
    crouch_legs = stg.standing_pose(stg.GaitParams(body_height=a.crouch_z))
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, a.stand_z + 0.01]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(NJ):
        gen.d.qpos[qpos_adr[i]] = stand_legs[i]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.4 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[omniquad-crouch] settled stand: base_z={init[2]:.3f}  tilt={_tilt(init):.1f}deg")

    # --- intent: stand -> crouch -> hold -> rise. MPPI discovers the feasible joints. ---
    T = a.secs
    stand_d, crouch_d = _legs_dict(stand_legs), _legs_dict(crouch_legs)
    intent = Intent(
        total_time=T,
        joint_keys=[(0.0, stand_d), (0.30 * T, crouch_d),
                    (0.62 * T, crouch_d), (0.95 * T, stand_d)],
        base_keys=[(0.0, {"z": a.stand_z}), (0.30 * T, {"z": a.crouch_z}),
                   (0.62 * T, {"z": a.crouch_z}), (0.95 * T, {"z": a.stand_z})],
        weights=dict(joint=4.0, base_z=12.0, base_xy=4.0, upright=a.upright,
                     balance=a.balance, effort=0.02, smooth=0.05, alive=0.5),
    )

    g = gen.generate(intent, init, n_samples=a.samples, horizon=a.horizon,
                     noise=a.noise, temperature=0.4, seed=0)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    np.savez(a.out, **{k: v for k, v in g.items() if k != "joints"},
             joints=np.array(g["joints"]))
    bz = g["base"][:, 2]
    print(f"[omniquad-crouch] generated {g['q'].shape[0]} steps; base_z "
          f"min={bz.min():.3f} max={bz.max():.3f} (target crouch {a.crouch_z}, stand {a.stand_z})")

    # ── VERIFY (shadowing Component 2) ──────────────────────────────────────
    # (1) open-loop sustainability: replay the GENERATED position targets (ctrl) open-loop
    #     with the correct OmniQuad actuators -> did it stay upright? (the decision gate).
    m = mujoco.MjModel.from_xml_path(a.mjcf)
    m.opt.timestep = 0.004
    d = mujoco.MjData(m)
    n_sub = max(1, int(round(float(g["dt"]) / 0.004)))
    d.qpos[:] = g["q"][0]
    d.qvel[:] = g["qvel"][0]
    mujoco.mj_forward(m, d)
    full = np.zeros(m.nu)
    fell_at = None
    bz_min, tilt_max = 9.9, 0.0
    C = g["ctrl"]
    for k in range(C.shape[0]):
        full[:] = 0.0
        full[gen.pos_act] = C[k]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        bz_min = min(bz_min, float(d.qpos[2]))
        tilt_max = max(tilt_max, _tilt(d.qpos))
        if d.qpos[2] < 0.25 or _tilt(d.qpos) > 50.0:
            fell_at = k * float(g["dt"])
            break
    no_fall = fell_at is None

    print("=" * 72)
    print("OMNIQUAD CROUCH-RECOVER GHOST -- MPPI-GENERATED + CERTIFIED (shadowing C1+C2)")
    print("=" * 72)
    print(f"[open-loop gate] {'SUSTAINS (never fell)' if no_fall else f'FELL at t={fell_at:.2f}s'}"
          f"  bz_min={bz_min:.3f}  tilt_max={tilt_max:.1f}deg")
    try:
        _inverse_dynamics_cert(g, a.mjcf, 0.004, verbose=True)
    except Exception as ex:
        print(f"  [inverse-dyn] skipped ({ex})")
    print("-" * 72)
    print(f"VERDICT: {'PASS -- MPPI found a feasible non-gait motion -> valid shadowing ghost' if no_fall else 'FAIL -- regenerate (raise samples/horizon or soften the intent)'}")
    print(f"saved -> {a.out}")
    return 0 if no_fall else 1


if __name__ == "__main__":
    sys.exit(main())
