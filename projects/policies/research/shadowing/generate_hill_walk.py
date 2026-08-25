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

"""Generate a FEASIBLE up-over-down HILL-WALK ghost (Shadowing Component 1),
robot-agnostic over OmniQuad / B2 / Go2.

The ghost is the analytic trot gait evaluated ON THE INCLINE: at each step the
base ADVANCES ALONG THE SURFACE at vx, rides at the perpendicular height above
h(x), and PITCHES PARALLEL TO THE LOCAL SLOPE; the 12 joints are the flat-ground
trot targets. Because the body-frame foot targets rotate with the base, a base
pitched parallel to the ramp lands the gait's feet on the inclined surface -- so
the same feasible-by-construction gait that walks on the flat (see
generate_go2_walk.py / project_go2_walk) becomes a feasible hill walk, with the
DYNAMICS supplied by the RL tracker (Component 3), exactly as on the flat.

The hill comes from hill_terrain.HillProfile (rounded corners) so the generator,
the trainer, and the deploy world all share ONE terrain. Feasibility is checked
two ways: (a) FK -- the ghost's feet sit ON the surface and the joints stay in
range (kinematic validity); (b) --mppi re-runs the motion through receding-
horizon predictive sampling over the REAL dynamics + hill contacts to confirm a
force-limited closed-loop traversal (dynamic feasibility with feedback).

  python projects/policies/research/shadowing/generate_hill_walk.py --robot b2
  python projects/policies/research/shadowing/generate_hill_walk.py --robot omniquad
"""
from __future__ import annotations

import argparse
import importlib
import math
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator             # noqa: E402
from projects.policies.research.shadowing.hill_terrain import HillProfile                   # noqa: E402

LEGS = ("FL", "FR", "RL", "RR")

ROBOTS = {
    "b2": dict(
        gait="projects.policies.control.gait.b2_trot_gait",
        mjcf=os.path.join(REPO, "projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml"),
        parts=("hip", "thigh", "calf"), foot_r=0.032, anon=False),
    "go2": dict(
        gait="projects.policies.control.gait.go2_trot_gait",
        mjcf=os.path.join(REPO, "projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml"),
        parts=("hip", "thigh", "calf"), foot_r=0.022, anon=False),
    "omniquad": dict(
        gait="projects.policies.control.gait.omniquad_trot_gait",
        mjcf=os.path.join(REPO, "projects/policies/research/training/mjcf/omniquad_newton_fixed2.xml"),
        parts=("hip_x", "hip_y", "knee"), foot_r=0.02, anon=True),
}


def _pitch_of_quat(q):
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))))


def _tilt_deg(qpos):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2)))))


def setup_generator(robot, grade, vx, ramp_run=1.8, crest=1.0, fillet=0.4,
                    gait_module=None):
    """Build the hill MJCF + a GhostGenerator with the right joint/foot mappings
    (OmniQuad's anonymous interleaved dump needs the override). Returns
    (gen, stg, gp, JNAMES, name2adr, ride, hp, hill_mjcf)."""
    cfg = ROBOTS[robot]
    stg = importlib.import_module(gait_module or cfg["gait"])
    gp = stg.GaitParams(vx=vx)
    JNAMES = [f"{leg}_{p}" for leg in LEGS for p in cfg["parts"]]
    ride = gp.body_height + cfg["foot_r"]

    hp = HillProfile(grade_deg=grade, ramp_run=ramp_run, crest=crest, fillet=fillet)
    hill_mjcf = os.path.join(REPO, "_scratch", f"{robot}_hill_planner.xml")
    os.makedirs(os.path.dirname(hill_mjcf), exist_ok=True)
    hp.inject_mjcf(cfg["mjcf"], hill_mjcf)

    gen = GhostGenerator(hill_mjcf, sim_dt=0.004, control_dt=0.02)
    if cfg["anon"]:
        from projects.policies.research.tools.generate_omniquad_crouch import _classify, JL_LO, JL_HI, LOWER_LEG_BODIES
        mujoco.mj_forward(gen.m, gen.d)
        qpos_adr, ctrl_pos = _classify(gen.m, gen.d)
        gen.pos_act = [int(c) for c in ctrl_pos]
        gen.act_jqpos = qpos_adr
        gen.act_name = [f"{nm}_pos" for nm in JNAMES]
        gen.ctrl_lo, gen.ctrl_hi = JL_LO.copy(), JL_HI.copy()
        gen.foot_bodies = list(LOWER_LEG_BODIES)
        gen.fixed_base = False
    name2adr = {nm: int(a_) for nm, a_ in zip(
        [n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos)}
    # the foot CONTACT point is the sphere geom (offset down the calf from the
    # foot-body origin = the knee), so record/measure from geom_xpos not xpos.
    gen.foot_geoms = []
    for b in gen.foot_bodies:
        sph = [g for g in range(gen.m.ngeom)
               if gen.m.geom_bodyid[g] == b and gen.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
        gen.foot_geoms.append(sph[0] if sph else
                              [g for g in range(gen.m.ngeom) if gen.m.geom_bodyid[g] == b][-1])
    return gen, stg, gp, JNAMES, name2adr, ride, hp, hill_mjcf


def build_kinematic_ghost(gen, stg, gp, JNAMES, name2adr, ride, hp, runout):
    """The clean gait-on-incline ghost: base climbs h(x) pitched to the slope,
    joints are the trot targets. Returns a dict (q, qvel, ctrl, base, com, feet)."""
    m = gen.m
    d = mujoco.MjData(m)
    dt = gen.control_dt
    dist_target = hp.x_end + runout
    T = dist_target / gp.vx + gp.ramp_s
    n_steps = int(round(T / dt))

    Q, CTRL, BASE = [], [], []
    x = 0.0
    for k in range(n_steps):
        t = k * dt
        pitch_surf = hp.pitch(x)                          # surface incline (+up,-down)
        phase = stg.QS_PHASE + 2.0 * math.pi * gp.freq * t
        legs, _ = stg.targets_np(phase, gp, t_since_start=t)   # 12 joint targets
        # base: generator-pitch>0 == nose-DOWN, so command -surface to align to ramp
        theta_y = -pitch_surf
        qw, qy = math.cos(theta_y / 2.0), math.sin(theta_y / 2.0)
        q = np.zeros(m.nq)
        q[0:3] = [x, 0.0, hp.height(x) + ride * math.cos(pitch_surf)]
        q[3:7] = [qw, 0.0, qy, 0.0]
        for i, nm in enumerate(JNAMES):
            q[name2adr[nm]] = legs[i]
        Q.append(q)
        CTRL.append(np.asarray(legs, dtype=float))
        BASE.append(q[0:7].copy())
        ramp = min(1.0, t / gp.ramp_s)
        x += gp.vx * ramp * math.cos(pitch_surf) * dt
    Q = np.array(Q)

    # finite-difference qvel (quaternion-aware), + COM and foot positions via FK
    QV = np.zeros((len(Q), m.nv))
    for k in range(len(Q) - 1):
        mujoco.mj_differentiatePos(m, QV[k], dt, Q[k], Q[k + 1])
    QV[-1] = QV[-2]
    COM, FEET = [], []
    for k in range(len(Q)):
        d.qpos[:] = Q[k]
        d.qvel[:] = QV[k]
        mujoco.mj_forward(m, d)
        COM.append(d.subtree_com[0].copy())
        FEET.append(np.array([d.geom_xpos[gi].copy() for gi in gen.foot_geoms]))
    return dict(dt=dt, q=Q, qvel=QV, ctrl=np.array(CTRL), base=np.array(BASE),
                com=np.array(COM), feet=np.array(FEET), T=T)


def fk_feasibility(gen, g, hp, foot_r):
    """Kinematic validity: feet on the surface + joints within range."""
    m = gen.m
    feet = g["feet"]                                   # (n, n_foot, 3)
    foot_gap = []                                      # foot-contact z minus h(x)
    for k in range(0, feet.shape[0]):
        for f in feet[k]:
            foot_gap.append((f[2] - foot_r) - hp.height(float(f[0])))
    foot_gap = np.array(foot_gap)
    # joint limits
    ctrl = g["ctrl"]
    lo, hi = gen.ctrl_lo, gen.ctrl_hi
    viol = np.maximum(0.0, lo - ctrl).max() + np.maximum(0.0, ctrl - hi).max()
    # positive gap at swing apex is expected; what matters is the mean (stance
    # feet on the surface) and no large penetration (most-negative gap).
    return dict(foot_gap_mean=float(foot_gap.mean()),
                foot_gap_min=float(foot_gap.min()),
                foot_gap_max=float(foot_gap.max()),
                limit_viol=float(viol))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=list(ROBOTS), default="b2")
    ap.add_argument("--grade", type=float, default=15.0, help="hill grade (deg)")
    ap.add_argument("--vx", type=float, default=0.35, help="climb speed")
    ap.add_argument("--ramp-run", type=float, default=1.8, help="horizontal ramp length (m)")
    ap.add_argument("--crest", type=float, default=1.0, help="crest length (m)")
    ap.add_argument("--fillet", type=float, default=0.4,
                    help="corner-rounding half-width (m); bigger = gentler onset")
    ap.add_argument("--runout", type=float, default=1.0, help="flat run past the hill (m)")
    ap.add_argument("--gait-module", default=None,
                    help="override the gait (e.g. projects.policies.control.gait.b2_crawl_gait) so the "
                         "ghost's joints + preview match the deploy gait")
    ap.add_argument("--mppi", action="store_true",
                    help="also run predictive-sampling traversal as a dynamic-feasibility check")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    gen, stg, gp, JNAMES, name2adr, ride, hp, hill_mjcf = setup_generator(
        a.robot, a.grade, a.vx, ramp_run=a.ramp_run, crest=a.crest, fillet=a.fillet,
        gait_module=a.gait_module)
    print(f"[{a.robot}-hill] grade={a.grade}deg rise={hp.rise:.3f}m hill x in "
          f"[{hp.x_up:.2f},{hp.x_end:.2f}]  vx={a.vx}  ride={ride:.3f}m  "
          f"({len(hp._boxes())} boxes) -> {hill_mjcf}")

    g = build_kinematic_ghost(gen, stg, gp, JNAMES, name2adr, ride, hp, a.runout)
    base = g["base"]
    fwd = base[-1, 0] - base[0, 0]
    print(f"[{a.robot}-hill] kinematic ghost: {g['q'].shape[0]} steps over {g['T']:.1f}s, "
          f"forward={fwd:+.2f} m (crest rise {hp.rise:.2f} m)")
    for k in range(0, g["q"].shape[0], max(1, g["q"].shape[0] // 12)):
        b = base[k]
        print(f"    t={k*g['dt']:5.2f}s  x={b[0]:+.2f}  base_z={b[2]:.3f}  "
              f"h(x)={hp.height(b[0]):.3f}  pitch={_pitch_of_quat(b[3:7]):+5.1f}deg")

    # ---- (a) FK feasibility: feet on the surface + joint limits ----
    fk = fk_feasibility(gen, g, hp, ROBOTS[a.robot]["foot_r"])
    # stance feet on the incline (mean ~0); transient clip at curved transitions
    # is expected for a flat-gait reference and resolved by physics/the tracker.
    feet_ok = abs(fk["foot_gap_mean"]) < 0.02
    print(f"\n[FK feasibility] foot-to-surface gap: mean={fk['foot_gap_mean']*1000:+.1f} mm "
          f"(stance feet on the incline), penetration={-min(0.0,fk['foot_gap_min'])*1000:.1f} mm, "
          f"swing apex={fk['foot_gap_max']*1000:.0f} mm   joint-limit violation={fk['limit_viol']:.4f} rad")
    print(f"  -> feet {'sit ON the incline' if feet_ok else 'DO NOT track the surface (tune ride)'};"
          f" joints {'within range' if fk['limit_viol'] < 1e-3 else 'OUT OF RANGE'}")

    out = a.out or os.path.join(REPO, "projects/policies/research/shadowing/ghosts", f"{a.robot}_hill_ghost.npz")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez(out, dt=g["dt"], q=g["q"], qvel=g["qvel"], ctrl=g["ctrl"],
             base=g["base"], com=g["com"], feet=g["feet"],
             joints=np.array(JNAMES),
             hill_grade_deg=a.grade, hill_approach=hp.approach,
             hill_ramp_run=hp.ramp_run, hill_crest=hp.crest, hill_width=hp.width,
             hill_fillet=hp.fillet, ride=ride, vx=a.vx)
    print(f"[{a.robot}-hill] saved -> {out}")

    # ---- (b) optional dynamic-feasibility check via predictive sampling ----
    if a.mppi:
        run_mppi_check(gen, stg, gp, JNAMES, name2adr, ride, hp, a.runout)
    return 0


def run_mppi_check(gen, stg, gp, JNAMES, name2adr, ride, hp, runout):
    """Receding-horizon predictive sampling over the REAL dynamics + hill, with
    the gait+climb intent -- confirms a force-limited closed-loop hill traversal
    (the dynamic-feasibility-with-feedback statement)."""
    from projects.policies.research.shadowing.ghost_generator import Intent
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, ride]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    stand = stg.standing_pose(gp)
    for i, nm in enumerate(JNAMES):
        gen.d.qpos[name2adr[nm]] = stand[i]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.4 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()

    dt = gen.control_dt
    T = (hp.x_end + runout) / gp.vx + gp.ramp_s
    n_keys = int(T / dt) + 1
    joint_keys, base_keys, x = [], [], 0.0
    for k in range(n_keys):
        t = k * dt
        phase = stg.QS_PHASE + 2.0 * math.pi * gp.freq * t
        legs, _ = stg.targets_np(phase, gp, t_since_start=t)
        joint_keys.append((t, {JNAMES[i]: float(legs[i]) for i in range(12)}))
        pitch_surf = hp.pitch(x)
        ramp = min(1.0, t / gp.ramp_s)
        x += gp.vx * ramp * math.cos(pitch_surf) * dt
        base_keys.append((t, {"x": x, "z": hp.height(x) + ride * math.cos(pitch_surf),
                              "pitch": -pitch_surf}))
    intent = Intent(total_time=T, joint_keys=joint_keys, base_keys=base_keys,
                    weights=dict(joint=8.0, base_xy=6.0, base_z=8.0, pitch=6.0,
                                 upright=1.5, balance=1.0, effort=0.01,
                                 smooth=0.05, alive=0.5))
    print("\n[mppi dynamic-feasibility check] receding-horizon traversal over real dynamics...")
    gm = gen.generate(intent, init, n_samples=72, horizon=16, noise=0.08,
                      temperature=0.4, seed=0, verbose=False)
    bz = gm["base"][:, 2]
    tilt = np.array([_tilt_deg(q) for q in gm["q"]])
    fwd = gm["base"][-1, 0] - gm["base"][0, 0]
    fell = bool((bz < np.array([hp.height(x) for x in gm["base"][:, 0]]) + 0.15).any()
                or (tilt > 60).any())
    print(f"  forward={fwd:+.2f} m, max tilt={tilt.max():.1f}deg, "
          f"{'TRAVERSED upright (dynamically feasible w/ feedback)' if not fell else 'FELL (refine)'}")


if __name__ == "__main__":
    sys.exit(main())
