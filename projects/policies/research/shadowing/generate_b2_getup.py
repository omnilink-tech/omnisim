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

"""Generate a FEASIBLE B2 GET-UP (self-right from a fall) ghost with the Ghost
Generator (Shadowing Component 1) -- a HARD motion with no analytic reference,
to push the shadowing pipeline.

The robot starts SPRAWLED on its belly (legs splayed, base ~10 cm) and must
gather its legs under itself and PUSH UP to a stand. There is no hand-written
gait for this; the planner must DISCOVER the feasible push-up over the real
dynamics + belly/foot contacts.

Two get-up-specific changes vs the stock generator (kept in this subclass so
ghost_generator.py is untouched):
  * the stock cost adds a hard "fell over" penalty when base_z < 0.2 -- but a
    get-up LEGITIMATELY starts there, so we drop it (the base_z keyframes pull
    it up instead);
  * the upright/balance terms ramp IN over the motion (lying is fine early;
    upright + CoM-over-feet matter only at the stand).

Feasibility is proven by construction (re-roll the planner's own targets, with
the BELLY contact included). Run:
  python projects/policies/research/shadowing/generate_b2_getup.py
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent
from projects.policies.control.gait import b2_trot_gait as stg

MJCF = os.path.join(REPO, "projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml")
OUT = os.path.join(REPO, "_scratch", "b2_getup_ghost.npz")
JN = [f"{l}_{p}" for l in ("FL", "FR", "RL", "RR") for p in ("hip", "thigh", "calf")]


class GetupGen(GhostGenerator):
    """GhostGenerator whose state cost permits the fallen start (no z<0.2
    penalty); everything else identical."""
    def _state_cost(self, d, jtarget, btarget, w):
        c = 0.0
        if jtarget is not None and len(jtarget):
            qa = d.qpos[self._jt_adr]
            c += w["joint"] * float(np.mean((qa - self._jt_val) ** 2))
        if not self.fixed_base:
            if btarget:
                if "x" in btarget:
                    c += w["base_xy"] * (d.qpos[0] - btarget["x"]) ** 2
                if "y" in btarget:
                    c += w["base_xy"] * (d.qpos[1] - btarget["y"]) ** 2
                if "z" in btarget:
                    c += w["base_z"] * (d.qpos[2] - btarget["z"]) ** 2
                if "pitch" in btarget:
                    wq, xq, yq, zq = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
                    pitch = np.arcsin(max(-1.0, min(1.0, 2 * (wq * yq - zq * xq))))
                    c += w["pitch"] * (pitch - btarget["pitch"]) ** 2
            R22 = 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)
            tilt = np.arccos(max(-1.0, min(1.0, R22)))
            c += w["upright"] * max(0.0, tilt - 0.1) ** 2
            if self.foot_bodies:
                com = d.subtree_com[0]
                fc = np.mean([d.xpos[b] for b in self.foot_bodies], axis=0)
                c += w["balance"] * float((com[0] - fc[0]) ** 2 + (com[1] - fc[1]) ** 2)
            # NO hard fell-over penalty -- a get-up starts in the fallen state.
        return c


def main():
    gen = GetupGen(MJCF, sim_dt=0.004, control_dt=0.02)
    name2adr = {nm: a for nm, a in zip(
        [n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos)}

    # --- init: drop a sprawled robot onto its belly and settle ---
    mujoco.mj_resetData(gen.m, gen.d)
    qp = gen.d.qpos
    qp[0:3] = [0.0, 0.0, 0.40]
    qp[3:7] = [1.0, 0.0, 0.0, 0.0]
    sprawl = {f"{l}_hip": 0.0 for l in ("FL", "FR", "RL", "RR")}
    sprawl.update({f"{l}_thigh": 2.9 for l in ("FL", "FR", "RL", "RR")})   # legs folded fwd/flat
    sprawl.update({f"{l}_calf": -0.5 for l in ("FL", "FR", "RL", "RR")})
    for nm, v in sprawl.items():
        qp[name2adr[nm]] = v
    gen.d.qpos[:] = qp
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.8 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    tilt0 = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (init[4] ** 2 + init[5] ** 2)))))
    fallen = {nm: float(init[name2adr[nm]]) for nm in JN}
    print(f"[b2-getup] settled FALLEN: base_z={init[2]:.3f}  tilt={tilt0:.0f}deg  "
          f"FL=({fallen['FL_hip']:+.2f},{fallen['FL_thigh']:+.2f},{fallen['FL_calf']:+.2f})")

    # --- intent: gather legs under + PUSH UP to the B2 stand over 3 s, then HOLD ---
    stand = stg.standing_pose(stg.GaitParams())
    stand_d = {JN[i]: float(stand[i]) for i in range(12)}
    # mid crouch: legs tucked under (hips 0, thigh ~1.4, calf ~-1.4), base partway up
    crouch_d = {f"{l}_hip": 0.0 for l in ("FL", "FR", "RL", "RR")}
    crouch_d.update({f"{l}_thigh": 1.4 for l in ("FL", "FR", "RL", "RR")})
    crouch_d.update({f"{l}_calf": -1.4 for l in ("FL", "FR", "RL", "RR")})
    z0 = float(init[2])
    # End at the NOMINAL crouched stand (base ~0.53 == body_height 0.50 + foot r),
    # NOT over-extended, and HOLD it ~4 s so the policy shadows a STABLE endpoint
    # (the over-tall ghost was what made the deploy topple at the top). Strong
    # base_z + balance weights pull it to the nominal height instead of overshooting.
    Z_STAND = 0.53
    # FAST rise (crouch 2.2 s, full stand 3.4 s, hold to 7.5 s): this is the
    # ghost nb1 was trained on -- nb1 deterministically reaches a CLEAN full
    # stand (z=0.52, tilt~2deg) at the top, which the deploy's STRICT hand-off
    # then freezes to a static PD stand. (The slow 10 s rise made the deploy
    # WORSE -- the planner's long-horizon hold wobbled and the robot toppled
    # mid-push at z=0.28.)
    intent = Intent(
        total_time=7.5,
        joint_keys=[(0.0, fallen), (1.0, fallen), (2.2, crouch_d),
                    (3.4, stand_d), (7.5, stand_d)],
        base_keys=[(0.0, {"x": 0.0, "z": z0, "pitch": 0.0}),
                   (1.0, {"x": 0.0, "z": z0, "pitch": 0.0}),
                   (2.2, {"x": 0.0, "z": 0.32, "pitch": 0.0}),
                   (3.4, {"x": 0.0, "z": Z_STAND, "pitch": 0.0}),
                   (7.5, {"x": 0.0, "z": Z_STAND, "pitch": 0.0})],
        weights=dict(joint=5.0, base_xy=2.0, base_z=22.0, pitch=2.0,
                     upright=2.5, balance=4.0, effort=0.01, smooth=0.05, alive=0.5),
    )
    g = gen.generate(intent, init, n_samples=96, horizon=18, noise=0.22,
                     temperature=0.5, seed=0)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez(OUT, **{k: v for k, v in g.items() if k != "joints"},
             joints=np.array(g["joints"]), init_qpos=init)
    base = g["base"]
    print(f"[b2-getup] saved {OUT}  steps={g['q'].shape[0]}  "
          f"start_z={base[0,2]:.3f} -> final_z={base[-1,2]:.3f}")
    dt = g["dt"]
    for k in range(0, g["q"].shape[0], max(1, g["q"].shape[0] // 12)):
        b = base[k]
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (b[4] ** 2 + b[5] ** 2)))))
        print(f"    t={k*dt:4.2f}s  base_z={b[2]:.3f}  tilt={tilt:4.0f}deg")

    # --- feasibility by construction (re-roll the planner's own targets from
    # the SAME init the planner used; deterministic -> reproduces the rise) ---
    m, d = gen.m, mujoco.MjData(gen.m)
    d.qpos[:] = init; d.qvel[:] = 0.0; mujoco.mj_forward(m, d)
    dof_lim = np.full(m.nv, 1e9)
    for j in range(m.njnt):
        if m.jnt_actfrclimited[j]:
            dof_lim[m.jnt_dofadr[j]] = max(1e-3, np.abs(m.jnt_actfrcrange[j]).max())
    full = np.zeros(m.nu); maxfrac = 0.0; fell = False
    for k in range(g["ctrl"].shape[0]):
        full[gen.pos_act] = g["ctrl"][k]; d.ctrl[:] = full
        for _ in range(gen.n_sub):
            mujoco.mj_step(m, d)
        tau = np.abs(d.qfrc_actuator[gen.act_jdof])
        maxfrac = max(maxfrac, float((tau / dof_lim[gen.act_jdof]).max()))
    final_z = float(d.qpos[2])
    stood = final_z > 0.42
    print(f"\n[feasibility-by-construction] re-roll: final base_z={final_z:.3f} "
          f"({'STOOD UP' if stood else 'did not reach stand'}); "
          f"peak torque {maxfrac*100:.0f}% of limit")


if __name__ == "__main__":
    main()
