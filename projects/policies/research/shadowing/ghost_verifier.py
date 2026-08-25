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

"""Ghost Verifier -- Shadowing Component 2 (numerical feasibility certificate).

Certifies, INDEPENDENTLY of the RL, that a ghost is DYNAMICALLY FEASIBLE -- via INVERSE
DYNAMICS, not a tracker (a tracker conflates feasibility with open-loop stability; balance
motions are open-loop unstable yet perfectly trackable with feedback). For each step we set
(qpos, qvel, qacc) from the ghost and run mj_inverse:
  * UNACTUATED free-base DOFs (0:6): the required generalized force must be ~0 -- i.e. the
    base motion is explained by GRAVITY + CONTACT forces alone. A hand-drawn ghost needs a
    large phantom base force (INFEASIBLE); a generated ghost came from real contacts (~0).
    This is the dynamic-balance / ZMP-feasibility certificate.
  * ACTUATED DOFs: the required torque must be within the joint actuator-force limits.
A secondary diagnostic re-sims under an open-loop PD tracker to report whether the motion
merely NEEDS feedback (drifts but doesn't fall = fine for RL) vs is unsustainable (falls).

Aggregate: FEASIBILITY SCORE + PASS/FAIL gate. Only certified ghosts go to RL.

Routing (one entry point for every ghost type):
  * FIXED-BASE arm  -> verify_arm        (velocity / torque / tracking / ballistic-reach)
  * FLOATING-BASE legged (walk/run/crouch/get-up) -> verify_legged (ghost_verifier_dyn):
      the per-step contact-wrench LP -- the CORRECT dynamic-feasibility test. The legacy
      open-loop-PD gate is wrong for any dynamic gait (open-loop unstable) and the
      finite-diff inverse-dynamics cert over-reports; both are kept only as diagnostics.
  * floating-base with no recognised feet -> open-loop PD gate (fallback).

  python projects/policies/research/shadowing/ghost_verifier.py --npz <ghost.npz> --mjcf <model.xml>
  python projects/policies/research/shadowing/ghost_verifier.py --npz <getup.npz> --mjcf <m> --contacts real
"""
import argparse
import os
import sys

import numpy as np
import mujoco

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)
from projects.policies.research.shadowing.ghost_verifier_dyn import verify_legged, _foot_geoms

_NBASE = 6  # free-joint DOFs (unactuated)


def _tilt(qpos):
    return np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2)))))


def _inverse_dynamics_cert(g, mjcf, sim_dt, verbose):
    """Primary certificate: per-step inverse dynamics -> base-DOF residual + actuated torque."""
    m = mujoco.MjModel.from_xml_path(mjcf)
    m.opt.timestep = sim_dt
    d = mujoco.MjData(m)
    q, qvel, gdt = g["q"], g["qvel"], float(g["dt"])
    # actuated-DOF force limits from joint actuatorfrcrange (fallback: generous constant)
    try:
        lim = np.abs(m.jnt_actfrcrange)  # (njnt, 2)
        dof_lim = np.full(m.nv, 200.0)
        for j in range(m.njnt):
            if m.jnt_actfrclimited[j]:
                a = m.jnt_dofadr[j]
                n = {mujoco.mjtJoint.mjJNT_FREE: 6, mujoco.mjtJoint.mjJNT_BALL: 3}.get(m.jnt_type[j], 1)
                dof_lim[a:a + n] = max(1e-3, lim[j].max())
    except Exception:
        dof_lim = np.full(m.nv, 200.0)

    base_resid, tor_ratio = [], []
    for k in range(1, q.shape[0] - 1):
        d.qpos[:] = q[k]
        d.qvel[:] = qvel[k]
        d.qacc[:] = (qvel[k + 1] - qvel[k - 1]) / (2.0 * gdt)
        mujoco.mj_inverse(m, d)
        f = d.qfrc_inverse
        base_resid.append(float(np.linalg.norm(f[:_NBASE])))
        tor_ratio.append(float(np.max(np.abs(f[_NBASE:]) / dof_lim[_NBASE:])))
    base_resid = np.array(base_resid)
    tor_ratio = np.array(tor_ratio)
    max_base, mean_base = float(base_resid.max()), float(base_resid.mean())
    max_tor = float(tor_ratio.max())
    # weight ~ m*g gives the scale of "real" base forces; residual should be a small fraction
    mg = float(m.body_mass.sum() * 9.81)
    base_frac = max_base / mg
    base_ok = base_frac < 0.15            # phantom base force < 15% of body weight
    tor_ok = max_tor < 1.0                # actuated torques within limits
    if verbose:
        print(f"  [inverse-dyn] base residual: max={max_base:.1f}N (={base_frac*100:.0f}% of mg={mg:.0f}N) "
              f"mean={mean_base:.1f}N  -> {'OK' if base_ok else 'INFEASIBLE base motion'}")
        print(f"  [inverse-dyn] actuated torque: max={max_tor*100:.0f}% of limit  -> {'OK' if tor_ok else 'OVER LIMIT'}")
    return base_ok, tor_ok, dict(max_base=max_base, base_frac=base_frac, max_tor=max_tor)


def _pd_diagnostic(g, mjcf, sim_dt, verbose):
    """Secondary: open-loop PD tracking -> distinguishes needs-feedback (drifts, fine) from
    unsustainable (falls). NOT a feasibility gate (balance is open-loop unstable)."""
    m = mujoco.MjModel.from_xml_path(mjcf)
    m.opt.timestep = sim_dt
    d = mujoco.MjData(m)
    q, gdt = g["q"], float(g["dt"])
    n_sub = max(1, int(round(gdt / sim_dt)))
    pos_act = [a for a in range(m.nu)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith("_pos")] or list(range(m.nu))
    jqpos = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in pos_act], dtype=int)
    d.qpos[:] = q[0]
    d.qvel[:] = g["qvel"][0]
    mujoco.mj_forward(m, d)
    full = np.zeros(m.nu)
    fell = False
    for k in range(q.shape[0]):
        full[pos_act] = q[k, jqpos]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        if d.qpos[2] < 0.3 or _tilt(d.qpos) > 60.0:
            fell = True
            break
    if verbose:
        print(f"  [open-loop PD] {'FALLS -> unsustainable' if fell else 'drifts but does NOT fall -> needs feedback (OK for RL)'}")
    return not fell


def _is_fixed_base(m):
    return not any(m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(m.njnt))


def _ballistic_landing(p, v, zb, g_accel=9.81):
    z, vz = float(p[2]), float(v[2])
    disc = vz * vz + 2.0 * g_accel * (z - zb)
    if disc < 0:
        return None
    t = (vz + np.sqrt(disc)) / g_accel
    return None if t <= 0 else (p[0] + v[0] * t, p[1] + v[1] * t, t)


def verify_arm(g, mjcf, sim_dt, verbose):
    """Feasibility certificate for a FIXED-BASE arm ghost (e.g. an arm toss). No base/ZMP term;
    the binding constraints are JOINT VELOCITY + MOTOR TORQUE, plus open-loop tracking fidelity
    and (for a throw) ballistic-landing consistency. Returns (passed, score, metrics)."""
    # re-simulate at the SAME physics dt the ghost was generated with (else the open-loop
    # replay diverges from a perfectly-feasible ghost purely from an integrator mismatch).
    if "sim_dt" in g.files:
        sim_dt = float(g["sim_dt"])
    m = mujoco.MjModel.from_xml_path(mjcf)
    m.opt.timestep = sim_dt
    q, qvel, gdt = g["q"], g["qvel"], float(g["dt"])
    nj = m.nu

    # --- (1) VELOCITY margin: the binding constraint for a light thrown payload ---
    vlim = g["vel_limits"] if "vel_limits" in g.files else np.full(nj, 1e9)
    vlim = np.asarray(vlim, dtype=float)
    vel_ratio = float((np.abs(qvel[:, :nj]) / vlim).max())
    vel_ok = vel_ratio <= 1.05

    # --- (2) TORQUE margin via inverse dynamics on the TRUE arm (damping zeroed: the motor-
    # curve damping was a generation device; the real motor only needs inertial+gravity torque,
    # so this is the honest requirement). Compare to actuator force (URDF effort) limits. ---
    m_true = mujoco.MjModel.from_xml_path(mjcf)
    m_true.opt.timestep = sim_dt
    m_true.dof_damping[:] = 0.0
    d = mujoco.MjData(m_true)
    eff = np.abs(m.actuator_forcerange[:, 1]).copy()
    eff[eff < 1e-6] = 1e9
    tor_ratio = 0.0
    for k in range(1, q.shape[0] - 1):
        d.qpos[:] = q[k]
        d.qvel[:] = qvel[k]
        d.qacc[:] = (qvel[k + 1] - qvel[k - 1]) / (2.0 * gdt)
        mujoco.mj_inverse(m_true, d)
        tor_ratio = max(tor_ratio, float(np.max(np.abs(d.qfrc_inverse[:nj]) / eff)))
    tor_ok = tor_ratio < 1.0

    # --- (3) open-loop tracking fidelity: replay the ghost's position targets, measure drift ---
    d2 = mujoco.MjData(m)
    d2.qpos[:] = q[0]
    mujoco.mj_forward(m, d2)
    n_sub = max(1, int(round(gdt / sim_dt)))
    pos_act = [a for a in range(m.nu)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith("_pos")] or list(range(m.nu))
    jqpos = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in pos_act], dtype=int)
    max_drift = 0.0
    for k in range(q.shape[0]):
        d2.ctrl[pos_act] = g["ctrl"][k] if "ctrl" in g.files else q[k, jqpos]
        for _ in range(n_sub):
            mujoco.mj_step(m, d2)
        max_drift = max(max_drift, float(np.max(np.abs(d2.qpos[jqpos] - q[k, jqpos]))))
    track_ok = max_drift < 0.20  # rad

    # --- (4) ballistic check: the throw must (a) be internally consistent AND (b) actually
    # reach the TARGET bin. A too-far bin yields a dynamically-feasible ghost that lands short
    # -- task-infeasible, and the gate must reject it. ---
    land_ok = True       # internal consistency (release state -> saved landing)
    reach_ok = True      # task: landing lands in the target bin
    land_err = 0.0
    reach_err = 0.0
    if "release_k" in g.files and "tcp" in g.files:
        rk = int(g["release_k"])
        zb = float(g["bin_rim_z"]) if "bin_rim_z" in g.files else 0.2
        land = _ballistic_landing(g["tcp"][rk], g["tcpvel"][rk], zb)
        if land is not None:
            if "land" in g.files:
                land_err = abs(land[0] - float(g["land"][0]))
                land_ok = land_err < 0.02
            if "bin_x" in g.files:
                reach_err = abs(land[0] - float(g["bin_x"]))
                reach_ok = reach_err < 0.05

    passed = vel_ok and tor_ok and track_ok and land_ok and reach_ok
    # feasibility score in [0,1]: binding margin across velocity / torque / open-loop
    # tracking / (for a throw) TASK-REACH. 1 = comfortable, ->0 as any margin exhausts.
    # Including reach makes the score track the gate: a beyond-reach throw is dynamically
    # fine but TASK-infeasible, and the score must register that (else PASS/FAIL and score
    # disagree).
    s_reach = 1.0
    if "release_k" in g.files and "bin_x" in g.files:
        s_reach = max(0.0, 1.0 - reach_err / 0.05)
    score = float(min(max(0.0, 1.0 - vel_ratio + 0.05),
                      max(0.0, 1.0 - tor_ratio),
                      max(0.0, 1.0 - max_drift / 0.20),
                      s_reach))
    if verbose:
        print(f"  [velocity]  peak ratio = {vel_ratio:.2f} / 1.00  -> {'OK' if vel_ok else 'OVER LIMIT'}")
        print(f"  [torque]    peak ratio = {tor_ratio:.2f} / 1.00  -> {'OK' if tor_ok else 'OVER LIMIT'}")
        print(f"  [tracking]  max open-loop drift = {max_drift*1000:.0f} mrad  -> {'OK' if track_ok else 'DIVERGES'}")
        if "release_k" in g.files:
            print(f"  [ballistic] landing-consistency err = {land_err*100:.1f} cm  -> {'OK' if land_ok else 'INCONSISTENT'}")
            if "bin_x" in g.files:
                print(f"  [reaches]   lands {reach_err*100:.1f} cm from target bin @ {float(g['bin_x']):.2f} m "
                      f"-> {'OK' if reach_ok else 'SHORT (task-infeasible bin)'}")
        if "max_range" in g.files and "kin_reach_x" in g.files:
            print(f"  [frontier]  max throw {float(g['max_range']):.2f} m vs reach {float(g['kin_reach_x']):.2f} m")
    return passed, score, dict(vel_ratio=vel_ratio, tor_ratio=tor_ratio, max_drift=max_drift,
                               land_err=land_err, reach_err=reach_err)


def verify(npz_path, mjcf, sim_dt=0.004, contacts="auto", mu=1.0, contact_z=0.05, verbose=True):
    g = np.load(npz_path, allow_pickle=True)
    print(f"[verify] {npz_path}  ({g['q'].shape[0]} steps @ {float(g['dt'])*1000:.0f}ms)")
    m = mujoco.MjModel.from_xml_path(mjcf)
    # Fixed-base arm (e.g. an arm toss): velocity/torque/tracking/ballistic certificate.
    if _is_fixed_base(m):
        passed, score, mets = verify_arm(g, mjcf, sim_dt, verbose)
        print(f"[verify] {'PASS' if passed else 'FAIL'} -- feasibility score {score:.2f}  "
              f"(velocity {mets['vel_ratio']:.2f}, torque {mets['tor_ratio']:.2f})")
        return dict(passed=passed, score=score, **mets)
    # FLOATING-BASE legged ghost (walk/run/crouch/get-up/jump/hill/sit): route to the
    # HARDENED, motion-agnostic certificate (feasibility_certificate.certify). It asks the
    # trajectory-optimization feasibility question directly (do friction-cone support contacts
    # + within-limit joint torques explain the ghost's accelerations?), but with robust
    # support-contact reconstruction (incl. robots with no 'foot'-named body, e.g. OmniQuad, and
    # belly/knee/chair support), flight-aware base feasibility (ballistic vs levitation),
    # terrain-aware ground, and balance margins (ZMP / capturability). It returns
    # INDETERMINATE when support cannot be reconstructed -- never a vacuous PASS. Falls back to
    # the core LP (verify_legged, named feet only) then the open-loop gate if it is unavailable.
    try:
        from projects.policies.research.shadowing.feasibility_certificate import certify
        passed, score, mets = certify(g, mjcf, motion="auto", sim_dt=sim_dt,
                                      mu=mu, contact_z=contact_z, verbose=verbose)
        indet = bool(mets.get("indeterminate"))
        verdict = ("INDETERMINATE -- insufficient reconstructable support (check model colliders)"
                   if indet else
                   ("PASS -- dynamically feasible -> send to RL" if passed
                    else "FAIL -- not dynamically feasible -> fix the ghost"))
        print(f"[verify] {verdict}   (certificate: {mets.get('motion','?')}, score {score:.2f})")
        return dict(passed=passed, score=score, indeterminate=indet, certificate=mets)
    except Exception as e:  # pragma: no cover - defensive fallback
        if verbose:
            print(f"  [hardened certificate unavailable ({type(e).__name__}: {e}); falling back to verify_legged]")
    if _foot_geoms(m):
        use_real = (contacts == "real")
        passed, score, mets = verify_legged(g, mjcf, sim_dt=sim_dt, mu=mu,
                                            contact_z=contact_z,
                                            use_real_contacts=use_real, verbose=verbose)
        print(f"[verify] {'PASS -- dynamically feasible -> send to RL' if passed else 'FAIL -- not dynamically feasible -> fix the ghost'}"
              f"   (certificate: contact-wrench LP, score {score:.2f})")
        return dict(passed=passed, score=score, dynamic=mets)
    no_fall = _pd_diagnostic(g, mjcf, sim_dt, verbose)
    base_ok, tor_ok, idm = _inverse_dynamics_cert(g, mjcf, sim_dt, verbose)
    passed = no_fall
    print(f"[verify] {'PASS -- sustains open-loop -> send to RL' if passed else 'FAIL -- collapses open-loop -> fix the ghost'}"
          f"   (no feet recognised; inverse-dyn certificate EXPERIMENTAL)")
    return dict(passed=passed, no_fall=no_fall, id_experimental=dict(base_ok=base_ok, tor_ok=tor_ok, **idm))


def main():
    REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    p = argparse.ArgumentParser()
    p.add_argument("--npz", default=os.path.join(REPO, "projects/policies/research/shadowing/ghosts/g1_sitstand_ghost.npz"))
    p.add_argument("--mjcf", default=os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml"))
    p.add_argument("--contacts", choices=["auto", "foot", "real"], default="auto",
                   help="legged contact source: 'auto'/'foot' = feet near ground (walks); "
                        "'real' = all robot/floor contacts from mj_forward (get-ups)")
    p.add_argument("--mu", type=float, default=1.0, help="friction coefficient for the contact cone")
    p.add_argument("--contact-z", type=float, default=0.05, help="foot-near-ground threshold (m)")
    a = p.parse_args()
    verify(a.npz, a.mjcf, contacts=a.contacts, mu=a.mu, contact_z=a.contact_z)


if __name__ == "__main__":
    main()
