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

"""Ghost Verifier -- DYNAMIC / LOCOMOTION certificate (Shadowing Component 2,
locomotion extension).

The stock ghost_verifier gates on OPEN-LOOP PD sustainability, which any
dynamic gait fails (a walk is open-loop unstable -- it balances via state-
dependent foot placement). Its inverse-dynamics cert (mj_inverse on finite-
diff qacc) over-reports on contact-rich motions because MuJoCo's inverse
dynamics can't cleanly recover the contact forces from a noisy ghost.

This certificate instead asks the trajectory-optimization feasibility
question DIRECTLY, per step, with a small LP:

    does there exist a foot contact force (friction-cone valid, normal >= 0)
    AND a joint torque (within the actuator limits) that produce the ghost's
    acceleration under the rigid-body dynamics?

    M(q) qddot + bias(q,qdot) = S^T tau + sum_i J_ci^T f_i
      base rows (unactuated 0:6):  must be met by CONTACTS alone   <-- ZMP feasibility
      act  rows (6:nv):            define tau; require |tau| <= lim

If at (almost) every step such (f, tau) exists with ~0 base residual, the
ghost is DYNAMICALLY FEASIBLE -- independent of open-loop stability. qacc is
lightly smoothed to tame finite-difference noise; contacts are taken at any
foot within `contact_z` of the ground (robust to zero-penetration ghosts).

  python ghost_verifier_dyn.py            # self-validates on 3 Go2 ghosts
  from ghost_verifier_dyn import verify_dynamic
"""
from __future__ import annotations
import os
import sys
import numpy as np
import mujoco
from scipy.optimize import linprog

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _smooth(x, w=9):
    if w <= 1:
        return x
    k = np.ones(w) / w
    y = np.copy(x)
    for i in range(x.shape[1]):
        y[:, i] = np.convolve(x[:, i], k, mode="same")
    return y


def _foot_geoms(m):
    """geom ids of the foot colliders (geoms on bodies whose name marks a foot:
    'foot' for quadrupeds, 'ankle_roll' for the G1/biped flat foot)."""
    out = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if "foot" in nm or "ankle_roll" in nm:
            out.append(g)
    return out


def _step_feasible(m, d, qacc, foot_geoms, dof_lim, mu, contact_z, base_tol,
                   use_real=False):
    """One step's feasibility LP. Returns (base_resid_N, max_tau_frac, n_contacts).

    contact source: use_real=False = feet near the ground (robust for kinematic
    flat-walk ghosts whose feet sit at zero penetration); use_real=True = every
    ACTUAL robot/floor contact from mj_forward (a get-up is supported by legs/
    belly, not just feet -- those non-foot ground reactions must be counted)."""
    nv = m.nv
    M = np.zeros((nv, nv))
    # mujoco <=3.8 wants (m, dst, d.qM); >=3.9 wants (m, d, dst) and may not
    # expose d.qM at all. Try modern first, fall back -- verified empirically
    # on both vendored builds during the newton 1.2 -> 1.5 migration.
    try:
        mujoco.mj_fullM(m, d, M)
    except (TypeError, AttributeError):
        mujoco.mj_fullM(m, M, d.qM)
    bias = d.qfrc_bias.copy()
    rhs = M @ qacc + bias                      # required generalized force
    Js = []
    if use_real:
        for ci in range(d.ncon):
            con = d.contact[ci]
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if (b1 == 0) == (b2 == 0):
                continue                       # keep only robot<->floor contacts
            body = b2 if b1 == 0 else b1
            jacp = np.zeros((3, nv))
            mujoco.mj_jac(m, d, jacp, None, con.pos, body)
            Js.append(jacp)
    else:
        for g in foot_geoms:
            p = d.geom_xpos[g]
            if p[2] > contact_z:
                continue
            jacp = np.zeros((3, nv))
            mujoco.mj_jac(m, d, jacp, None, p, m.geom_bodyid[g])
            Js.append(jacp)
    nc = len(Js)
    if nc == 0:
        # nothing can produce the base wrench -> residual = full base force
        return float(np.linalg.norm(rhs[:6])), np.inf, 0
    # LP variables: f (3*nc) + slack s (6, >=0).  min sum(s)
    nf = 3 * nc
    A6 = np.hstack([J[:, :6].T for J in Js])        # 6 x 3nc  (base rows of sum J^T f)
    Aact = np.hstack([J[:, 6:].T for J in Js])      # (nv-6) x 3nc
    nx = nf + 6
    c = np.concatenate([np.zeros(nf), np.ones(6)])
    A_ub, b_ub = [], []
    # base residual:  A6 f - rhs_base = r,  |r| <= s
    for sgn in (+1.0, -1.0):
        block = np.zeros((6, nx))
        block[:, :nf] = sgn * A6
        block[:, nf:] = -np.eye(6)
        A_ub.append(block); b_ub.append(sgn * rhs[:6])
    # friction pyramid per contact: fz>=0; |fx|<=mu fz; |fy|<=mu fz
    for i in range(nc):
        fz = 3 * i + 2; fx = 3 * i; fy = 3 * i + 1
        for ax in (fx, fy):
            for sgn in (+1.0, -1.0):
                row = np.zeros(nx); row[ax] = sgn; row[fz] = -mu
                A_ub.append(row[None]); b_ub.append([0.0])
        row = np.zeros(nx); row[fz] = -1.0           # fz >= 0
        A_ub.append(row[None]); b_ub.append([0.0])
    # actuator torque limits:  tau = rhs_act - Aact f,  |tau| <= dof_lim
    lim = dof_lim[6:]
    for sgn in (+1.0, -1.0):
        block = np.zeros((nv - 6, nx))
        block[:, :nf] = sgn * Aact                   # +Aact f <= rhs_act + lim  (sgn=+1)
        A_ub.append(block); b_ub.append(sgn * rhs[6:] + lim)
    A_ub = np.vstack(A_ub); b_ub = np.concatenate(b_ub)
    bounds = [(None, None)] * nf + [(0, None)] * 6
    r = linprog(c, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method="highs")
    if not r.success:
        return base_tol * 10, np.inf, nc          # infeasible LP -> definitely not feasible
    f = r.x[:nf]
    base_resid = float(np.sum(r.x[nf:]))
    tau = rhs[6:] - Aact @ f
    max_tau_frac = float(np.max(np.abs(tau) / np.maximum(lim, 1e-6)))
    return base_resid, max_tau_frac, nc


def verify_legged(g, mjcf, sim_dt=0.004, mu=1.0, contact_z=0.05,
                  smooth_win=9, base_tol_frac=0.08, use_real_contacts=False,
                  verbose=True):
    """Feasibility certificate for a FLOATING-BASE LEGGED ghost (quadruped or
    biped). Drop-in sibling of ghost_verifier.verify_arm: takes the LOADED
    ghost `g` (npz dict) and returns (passed, score, metrics). Wire it into
    ghost_verifier.verify()'s floating-base branch -- it replaces the open-loop
    PD gate (which any dynamic gait fails) with the per-step contact-wrench LP."""
    q, qvel, dt = g["q"], g["qvel"], float(g["dt"])
    m = mujoco.MjModel.from_xml_path(mjcf); m.opt.timestep = sim_dt
    d = mujoco.MjData(m)
    mg = float(m.body_mass.sum() * 9.81)
    foot_geoms = _foot_geoms(m)
    dof_lim = np.full(m.nv, 1e9)
    for j in range(m.njnt):
        if m.jnt_actfrclimited[j]:
            dof_lim[m.jnt_dofadr[j]] = max(1e-3, np.abs(m.jnt_actfrcrange[j]).max())
    qvs = _smooth(qvel, smooth_win)
    base_res, tau_frac, ncs = [], [], []
    for k in range(2, q.shape[0] - 2):
        d.qpos[:] = q[k]; d.qvel[:] = qvs[k]
        mujoco.mj_forward(m, d)
        qacc = (qvs[k + 1] - qvs[k - 1]) / (2.0 * dt)
        br, tf, nc = _step_feasible(m, d, qacc, foot_geoms, dof_lim, mu, contact_z,
                                    base_tol_frac * mg, use_real=use_real_contacts)
        base_res.append(br); tau_frac.append(tf); ncs.append(nc)
    base_res = np.array(base_res); tau_frac = np.array(tau_frac)
    # robust aggregates (ignore the worst few % as finite-diff/contact-switch spikes)
    base95 = float(np.percentile(base_res, 95)); base_max = float(base_res.max())
    tau95 = float(np.percentile(tau_frac[np.isfinite(tau_frac)], 95)) if np.isfinite(tau_frac).any() else np.inf
    base_frac95 = base95 / mg
    no_contact = int(np.sum(np.array(ncs) == 0))
    base_ok = base_frac95 < base_tol_frac
    tau_ok = tau95 < 1.05
    passed = base_ok and tau_ok and no_contact <= 0.05 * len(ncs)
    if verbose:
        print(f"  ({q.shape[0]} steps @ {dt*1000:.0f}ms)  mg={mg:.0f}N")
        print(f"  base-wrench residual (contacts must supply it): "
              f"p95={base95:.1f}N ({base_frac95*100:.1f}% of mg)  max={base_max:.0f}N "
              f"-> {'OK' if base_ok else 'INFEASIBLE base motion'}")
        print(f"  joint torque: p95={tau95*100:.0f}% of limit  -> {'OK' if tau_ok else 'OVER LIMIT'}")
        print(f"  steps with NO foot contact (airborne base): {no_contact}/{len(ncs)}")
        print(f"[verify-dyn] {'PASS -- dynamically feasible (contacts + within-limit torque explain the motion)' if passed else 'FAIL -- not dynamically feasible'}")
    # feasibility score in [0,1]: per-constraint satisfaction relative to its PASS
    # threshold (1.0 = unloaded, 0.0 = at/over the gate), combined by the binding
    # (min) constraint and reported BY COMPONENT so a low score is attributable -- a
    # torque-tight but otherwise-clean gait scores low on s_tau by design, which is the
    # honest signal (it predicts the tracker will have little actuator headroom).
    frac_nc = no_contact / max(1, len(ncs))
    s_base = max(0.0, 1.0 - base_frac95 / max(base_tol_frac, 1e-6))
    s_tau = max(0.0, 1.0 - tau95 / 1.05) if np.isfinite(tau95) else 0.0
    s_contact = max(0.0, 1.0 - frac_nc / 0.05)
    score = float(min(s_base, s_tau, s_contact))
    if verbose:
        print(f"  feasibility score = {score:.2f}  "
              f"(s_base={s_base:.2f}, s_torque={s_tau:.2f}, s_contact={s_contact:.2f})")
    return passed, score, dict(base_frac95=base_frac95, tau95=tau95,
                               no_contact=no_contact, base_ok=base_ok, tau_ok=tau_ok,
                               s_base=round(s_base, 3), s_tau=round(s_tau, 3),
                               s_contact=round(s_contact, 3))


def verify_dynamic(npz_path, mjcf, **kw):
    """File-path convenience wrapper around verify_legged. Returns a dict."""
    g = np.load(npz_path, allow_pickle=True)
    if kw.get("verbose", True):
        print(f"[verify-dyn] {os.path.basename(npz_path)}", end="")
    passed, score, mets = verify_legged(g, mjcf, **kw)
    return dict(passed=passed, score=score, **mets)


# ----------------------------------------------------------------------------
# Self-validation: certify must PASS a feasible walk + FAIL infeasible ghosts.
# ----------------------------------------------------------------------------
def _build_kinematic_ghost(mjcf, base_z_fn, out, T=3.0, dt=0.02):
    """Build a kinematic Go2 trot ghost with a prescribed base-z(t)."""
    sys.path.insert(0, REPO)
    from projects.policies.control.gait import go2_trot_gait as stg
    m = mujoco.MjModel.from_xml_path(mjcf); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    mapping = {}
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        mapping[nm] = m.jnt_qposadr[j]
    LEGS = ("FL", "FR", "RL", "RR"); PARTS = ("hip", "thigh", "calf")
    JN = [f"{l}_{p}" for l in LEGS for p in PARTS]
    gp = stg.GaitParams()
    N = int(T / dt); nq, nv = m.nq, m.nv
    q = np.zeros((N, nq)); qvel = np.zeros((N, nv)); xs = np.zeros(N); x = 0.0
    for k in range(N):
        t = k * dt
        legs, _ = stg.targets_np(stg.QS_PHASE + 2 * np.pi * gp.freq * t, gp, t_since_start=t)
        x += gp.vx * min(1.0, t / gp.ramp_s) * dt; xs[k] = x
        q[k, 0] = x; q[k, 2] = base_z_fn(t); q[k, 3] = 1.0
        for i in range(12):
            q[k, mapping[JN[i]]] = legs[i]
    for k in range(N):
        km, kp = max(0, k - 1), min(N - 1, k + 1)
        denom = (kp - km) * dt or 1.0
        qvel[k, 0] = (xs[kp] - xs[km]) / denom
        qvel[k, 2] = (base_z_fn(kp * dt) - base_z_fn(km * dt)) / denom
        for i in range(12):
            a = mapping[JN[i]]; qvel[k, a - 1] = (q[kp, a] - q[km, a]) / denom
    np.savez(out, q=q, qvel=qvel, dt=np.float64(dt))


def _main():
    mjcf = os.path.join(REPO, "projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
    gen = os.path.join(REPO, "_scratch/go2_walk_ghost_generated.npz")
    flat = os.path.join(REPO, "_scratch/go2_flat_ghost.npz")
    levi = os.path.join(REPO, "_scratch/go2_levitate_ghost.npz")
    _build_kinematic_ghost(mjcf, lambda t: 0.322, flat)                 # flat constant-height base
    _build_kinematic_ghost(mjcf, lambda t: 0.322 + 0.6 * min(1.0, t / 2.0), levi)  # base flies to ~0.9m
    print("=" * 78, "\nA) PLANNER-GENERATED rollout walk ghost  (expect: PASS)\n" + "=" * 78)
    if os.path.exists(gen):
        verify_dynamic(gen, mjcf)
    else:
        print("  (generate it first: python projects/policies/research/shadowing/generate_go2_walk.py)")
    print("=" * 78, "\nB) HAND-AUTHORED FLAT-base trot ghost  (the one I shipped -- is it feasible?)\n" + "=" * 78)
    verify_dynamic(flat, mjcf)
    print("=" * 78, "\nC) LEVITATING base ghost  (physically impossible -- expect: FAIL)\n" + "=" * 78)
    verify_dynamic(levi, mjcf)


if __name__ == "__main__":
    _main()
