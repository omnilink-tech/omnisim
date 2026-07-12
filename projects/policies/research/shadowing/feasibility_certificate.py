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

"""Hardened, MOTION-AGNOSTIC feasibility certificate (Shadowing Component 2).

This is a NEW module that supersedes the contact front-end of the committed
``ghost_verifier_dyn.verify_legged`` while preserving its per-step LP *in spirit*
and its flat-walk verdicts *byte-for-byte*. The committed files
(``ghost_verifier.py``, ``ghost_verifier_dyn.py``, ``ghosts/*``) are left intact
for review; this module is additive.

THE PHYSICS QUESTION (unchanged):
    Per step, does there exist a set of friction-cone contact forces + within-
    limit joint torques that explain ``M(q) q'' + bias`` under rigid-body
    dynamics?  Base rows (0:6) must be met by CONTACTS alone (ZMP feasibility);
    actuated rows define tau, require |tau| <= lim.

WHAT CHANGES vs verify_legged (the LP core is reimplemented locally, identical
in spirit):
  1. CONTACT RECONSTRUCTION.  The committed cert only looks at name-matched
     'foot'/'ankle_roll' geoms and gates on absolute *center* height
     (``geom_xpos.z > contact_z``).  That misses (a) get-up support by
     belly/knees/elbows, (b) Spot's feet (no foot-named body -> empty set), and
     (c) any contact on a sloped/raised surface.  Here, ``support_contacts``
     unions three layered sources: all robot collision geoms by *size-aware
     bottom-surface* proximity to a terrain-aware local ground; named/leaf foot
     fast-path (for byte-stable walk); and external world-fixed support surfaces
     (chair seat/back for sit-stand).  Temporal hysteresis (+-2 steps) fills
     single-frame contact dropouts.
  2. FLIGHT-AWARE BASE FEASIBILITY.  When there is genuinely no contact the
     committed cert scores ``||rhs[:6]||`` as a hard violation -> a legitimate
     ballistic jump FAILs.  Here a no-contact step is classified: free fall
     (``||rhs[:6]|| <= flight_tol*mg``, gravity+inertia explain the base) is
     FEASIBLE; levitation (~mg required against nothing) is INFEASIBLE.  The
     ``no_contact <= 5%`` gate becomes a *total-flight-fraction* sanity cap.
  3. TERRAIN.  Ground height is a callable ``ground_h(x,y)`` (flat z=0 default;
     ``HillProfile.height`` for hills, with dynamics computed in the terrain
     MJCF, not the flat planner).
  4. MARGIN LAYER.  From the LP primal it reports literature balance margins
     (CoP / ZMP distance, required friction, torque headroom, capturability)
     -- ``capture`` is reported but NOT gated (it is the documented necessary-
     not-sufficient signal that flags the PASS-but-topples G1 walk).

BACKWARD COMPATIBILITY (hard requirement):  ``certify(..., motion='walk')`` uses
the center-height foot path and the same aggregates/constants as the committed
``verify_legged``, so go2/b2/g1 flat walk reproduce PASS 0.69/0.50/0.05 and the
levitating twin FAILs 0.00 -- verified against verify_certificate_suite.py.

  python projects/policies/research/shadowing/feasibility_certificate.py   # self-test table
  from feasibility_certificate import certify
"""
from __future__ import annotations

import os
import sys

import numpy as np
import mujoco
from scipy.optimize import linprog

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

GHOSTS = os.path.join(REPO, "projects/policies/research/shadowing/ghosts")
G = 9.81


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _smooth(x, w=9):
    """9-wide box smoothing per-column (same as ghost_verifier_dyn._smooth)."""
    if w <= 1:
        return x
    k = np.ones(w) / w
    y = np.copy(x)
    for i in range(x.shape[1]):
        y[:, i] = np.convolve(x[:, i], k, mode="same")
    return y


def _load(ghost):
    """ghost may be an npz path (str) or an already-loaded dict/NpzFile."""
    if isinstance(ghost, str):
        return np.load(ghost, allow_pickle=True)
    return ghost


def _is_fixed_base(m):
    return not any(m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(m.njnt))


def _dof_limits(m):
    dof_lim = np.full(m.nv, 1e9)
    for j in range(m.njnt):
        if m.jnt_actfrclimited[j]:
            dof_lim[m.jnt_dofadr[j]] = max(1e-3, np.abs(m.jnt_actfrcrange[j]).max())
    return dof_lim


def _leaf_bodies(m):
    parents = set(m.body_parentid[1:].tolist())
    return [b for b in range(1, m.nbody) if b not in parents]


def _foot_geoms_named(m):
    """Committed _foot_geoms: name-matched 'foot'/'ankle_roll' bodies."""
    out = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if "foot" in nm or "ankle_roll" in nm:
            out.append(g)
    return out


def candidate_geoms(m, support_bodies=None):
    """All robot collision geoms (not visual-only, not world).  Optionally
    restricted to a set of body ids."""
    out = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        if b == 0:
            continue  # world / floor
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue  # visual-only
        if support_bodies is not None and b not in support_bodies:
            continue
        out.append(g)
    return out


def _geom_bottom(m, d, g):
    """Lowest surface point of geom g: returns (x, y, z_bottom) in world."""
    p = d.geom_xpos[g]
    R = d.geom_xmat[g].reshape(3, 3)
    t = int(m.geom_type[g])
    sz = m.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:  # 2
        return float(p[0]), float(p[1]), float(p[2] - sz[0])
    if t == mujoco.mjtGeom.mjGEOM_CAPSULE:  # 3 : half-length sz[1] along local z, radius sz[0]
        best = None
        for s in (+1.0, -1.0):
            c = p + R @ np.array([0.0, 0.0, s * sz[1]])
            zb = c[2] - sz[0]
            if best is None or zb < best[2]:
                best = (float(c[0]), float(c[1]), float(zb))
        return best
    if t in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_MESH):  # 6 / 7 : 8 corners
        best = None
        for sx in (+1.0, -1.0):
            for sy in (+1.0, -1.0):
                for sz_ in (+1.0, -1.0):
                    c = p + R @ (sz[:3] * np.array([sx, sy, sz_]))
                    if best is None or c[2] < best[2]:
                        best = (float(c[0]), float(c[1]), float(c[2]))
        return best
    return float(p[0]), float(p[1]), float(p[2])


# ---------------------------------------------------------------------------
# support-contact reconstruction
# ---------------------------------------------------------------------------
def support_contacts(m, d, cfg):
    """Return a list of (world_point, body_id, normal) support contacts at the
    current d state.  Layered sources unioned + deduped by (body, 1cm point).

    cfg keys used: motion, margin, ground_h, support_bodies, support_geoms.
    For motion=='walk' uses the byte-compatible center-height foot path.
    """
    motion = cfg["motion"]
    margin = cfg["margin"]
    ground_h = cfg["ground_h"]

    # --- WALK compatibility path: exact committed behaviour (center height) ---
    if motion == "walk":
        out = []
        for g in cfg["walk_foot_geoms"]:
            p = d.geom_xpos[g]
            if p[2] > cfg["contact_z"]:
                continue
            out.append((np.array(p, dtype=float), int(m.geom_bodyid[g]),
                        np.array([0.0, 0.0, 1.0])))
        return out

    # --- general path: size-aware bottom-surface proximity to local ground ---
    raw = []
    sb = cfg.get("support_bodies")
    geoms = cfg["candidate_geoms"] if sb is None else candidate_geoms(m, sb)
    for g in geoms:
        bx, by, zb = _geom_bottom(m, d, g)
        gz = ground_h(bx, by)
        if (zb - gz) <= margin:
            pt = np.array([bx, by, gz], dtype=float)  # clamp z to local ground
            raw.append((pt, int(m.geom_bodyid[g]), np.array([0.0, 0.0, 1.0])))

    # --- external world-fixed support surfaces (chair seat/back for sit) ---
    for sgname in cfg.get("support_geoms", []):
        sg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, sgname)
        if sg < 0:
            continue
        sp = d.geom_xpos[sg]
        ssz = m.geom_size[sg]
        st = int(m.geom_type[sg])
        # top face of the support surface (for a box: center.z + half-z, world-aligned approx)
        s_top = sp[2] + (ssz[2] if st == mujoco.mjtGeom.mjGEOM_BOX else ssz[0])
        s_hx = ssz[0] if st == mujoco.mjtGeom.mjGEOM_BOX else ssz[0]
        s_hy = ssz[1] if st == mujoco.mjtGeom.mjGEOM_BOX else ssz[0]
        for g in geoms:
            bx, by, zb = _geom_bottom(m, d, g)
            # within the support's xy footprint and near its top face
            if (abs(bx - sp[0]) <= s_hx + margin and abs(by - sp[1]) <= s_hy + margin
                    and abs(zb - s_top) <= margin):
                raw.append((np.array([bx, by, s_top], dtype=float),
                            int(m.geom_bodyid[g]), np.array([0.0, 0.0, 1.0])))

    # dedup by (body, point within 1cm)
    out = []
    for pt, body, nrm in raw:
        dup = False
        for opt, obody, _ in out:
            if obody == body and np.linalg.norm(opt - pt) < 0.01:
                dup = True
                break
        if not dup:
            out.append((pt, body, nrm))
    return out


# ---------------------------------------------------------------------------
# per-step LP (reimplemented locally, mirrors ghost_verifier_dyn._step_feasible)
# ---------------------------------------------------------------------------
def _step_lp(m, d, qacc, contacts, dof_lim, mu, base_tol_N):
    """Solve the per-step feasibility LP for an explicit contact set.

    Returns dict with: base_resid (N, = sum slack), tau_frac, nc, and (when
    nc>0) per-contact forces + points for margins.  When nc==0 returns
    base_resid=||rhs[:6]||, tau_frac=inf, nc=0 (flight handled by caller).
    """
    nv = m.nv
    M = np.zeros((nv, nv))
    mujoco.mj_fullM(m, M, d.qM)
    bias = d.qfrc_bias.copy()
    rhs = M @ qacc + bias

    Js, pts = [], []
    for pt, body, _ in contacts:
        jacp = np.zeros((3, nv))
        mujoco.mj_jac(m, d, jacp, None, pt, body)
        Js.append(jacp)
        pts.append(pt)
    nc = len(Js)
    if nc == 0:
        return dict(base_resid=float(np.linalg.norm(rhs[:6])),
                    base_lin=float(np.linalg.norm(rhs[:3])),
                    base_ang=float(np.linalg.norm(rhs[3:6])),
                    tau_frac=np.inf, nc=0, rhs6=rhs[:6].copy())

    nf = 3 * nc
    A6 = np.hstack([J[:, :6].T for J in Js])      # 6 x 3nc
    Aact = np.hstack([J[:, 6:].T for J in Js])    # (nv-6) x 3nc
    nx = nf + 6
    c = np.concatenate([np.zeros(nf), np.ones(6)])
    A_ub, b_ub = [], []
    for sgn in (+1.0, -1.0):
        block = np.zeros((6, nx))
        block[:, :nf] = sgn * A6
        block[:, nf:] = -np.eye(6)
        A_ub.append(block)
        b_ub.append(sgn * rhs[:6])
    for i in range(nc):
        fz = 3 * i + 2
        fx = 3 * i
        fy = 3 * i + 1
        for ax in (fx, fy):
            for sgn in (+1.0, -1.0):
                row = np.zeros(nx)
                row[ax] = sgn
                row[fz] = -mu
                A_ub.append(row[None])
                b_ub.append([0.0])
        row = np.zeros(nx)
        row[fz] = -1.0
        A_ub.append(row[None])
        b_ub.append([0.0])
    lim = dof_lim[6:]
    for sgn in (+1.0, -1.0):
        block = np.zeros((nv - 6, nx))
        block[:, :nf] = sgn * Aact
        A_ub.append(block)
        b_ub.append(sgn * rhs[6:] + lim)
    A_ub = np.vstack(A_ub)
    b_ub = np.concatenate(b_ub)
    bounds = [(None, None)] * nf + [(0, None)] * 6
    r = linprog(c, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method="highs")
    if not r.success:
        return dict(base_resid=base_tol_N * 10, base_lin=base_tol_N * 10,
                    base_ang=0.0, tau_frac=np.inf, nc=nc, rhs6=rhs[:6].copy())
    f = r.x[:nf]
    base_resid = float(np.sum(r.x[nf:]))
    tau = rhs[6:] - Aact @ f
    tau_frac = float(np.max(np.abs(tau) / np.maximum(lim, 1e-6)))
    fz = f[2::3]
    ft = np.sqrt(f[0::3] ** 2 + f[1::3] ** 2)
    # margins
    fz_pos = np.maximum(fz, 0.0)
    req_mu = float(np.max(ft / np.maximum(fz_pos, 1e-6))) if fz_pos.sum() > 1e-6 else 0.0
    fric_slack = float(np.min(mu * fz_pos - ft)) if nc else 0.0
    tau_headroom = float(np.min((lim - np.abs(tau)) / np.maximum(lim, 1e-6)))
    # CoP and signed distance to support-polygon edge
    P = np.array(pts)
    d_zmp = _cop_margin(P[:, :2], fz_pos)
    return dict(base_resid=base_resid,
                base_lin=float(np.linalg.norm(r.x[nf:nf + 3])),
                base_ang=float(np.linalg.norm(r.x[nf + 3:nf + 6])),
                tau_frac=tau_frac, nc=nc, req_mu=req_mu, fric_slack=fric_slack,
                tau_headroom=tau_headroom, d_zmp=d_zmp, rhs6=rhs[:6].copy())


def _cop_margin(xy, fz):
    """Signed distance from the CoP (force-weighted contact xy) to the convex
    hull edge of the active contact points (>0 inside).  1-2 contacts -> 0."""
    if fz.sum() <= 1e-9 or len(xy) < 3:
        return 0.0
    cop = (xy * fz[:, None]).sum(axis=0) / fz.sum()
    try:
        from scipy.spatial import ConvexHull
        active = xy[fz > 1e-6]
        if len(active) < 3:
            return 0.0
        hull = ConvexHull(active)
        mind = np.inf
        for eq in hull.equations:  # ax+by+c=0, normal outward
            dist = -(eq[:2] @ cop + eq[2]) / (np.linalg.norm(eq[:2]) + 1e-12)
            mind = min(mind, dist)
        return float(mind)
    except Exception:
        return 0.0


# ---------------------------------------------------------------------------
# ground-height resolution
# ---------------------------------------------------------------------------
def _resolve_ground_and_model(ghost, mjcf, motion, opts):
    """Return (model_mjcf_path, ground_h_callable).  For hills, inject the
    terrain into the flat planner and use HillProfile.height; the dynamics are
    computed in the TERRAIN model (the flat MJCF is the wrong model for a hill
    ghost)."""
    if "ground_h" in opts and opts["ground_h"] is not None:
        return mjcf, opts["ground_h"]
    if opts.get("terrain_mjcf"):
        tm = opts["terrain_mjcf"]
        mm = mujoco.MjModel.from_xml_path(tm)
        dd = mujoco.MjData(mm)
        mujoco.mj_forward(mm, dd)

        def gh(x, y, _mm=mm, _dd=dd):
            gid = np.zeros(1, dtype=np.int32)
            dist = mujoco.mj_ray(_mm, _dd, np.array([x, y, 3.0]),
                                 np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
            return 3.0 - dist if dist >= 0 else 0.0
        return tm, gh
    if motion == "hill" or ("hill_grade_deg" in (ghost.files if hasattr(ghost, "files") else ghost)):
        from projects.policies.research.shadowing.hill_terrain import HillProfile
        gk = lambda k: float(ghost[k])
        hp = HillProfile(grade_deg=gk("hill_grade_deg"), approach=gk("hill_approach"),
                         ramp_run=gk("hill_ramp_run"), crest=gk("hill_crest"),
                         fillet=gk("hill_fillet"), width=gk("hill_width"))
        sc = os.path.join(REPO, "_scratch")
        os.makedirs(sc, exist_ok=True)
        robot = os.path.basename(mjcf).split("_")[0]
        out = os.path.join(sc, f"{robot}_hill_planner_cert.xml")
        hp.inject_mjcf(mjcf, out)
        return out, (lambda x, y, _hp=hp: _hp.height(x))
    return mjcf, (lambda x, y: 0.0)


# ---------------------------------------------------------------------------
# motion auto-detection
# ---------------------------------------------------------------------------
def _auto_motion(ghost, m):
    if _is_fixed_base(m):
        return "arm"
    keys = ghost.files if hasattr(ghost, "files") else list(ghost.keys())
    if "hill_grade_deg" in keys:
        return "hill"
    has_chair = any(any(s in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").lower()
                        for s in ("chair", "seat", "stool", "bench"))
                    for g in range(m.ngeom))
    q = ghost["q"]
    if has_chair:
        return "sit"
    # get-up: base rises from a low start to a much higher end
    z0, zT = float(q[0, 2]), float(q[-1, 2])
    if z0 < 0.5 * zT and z0 < 0.30:
        return "getup"
    # jump: a true aerial phase = ALL feet off the ground for a CONTIGUOUS run of
    # frames. A single stray airborne frame (a bobbing trot's swing apex) must NOT
    # trip this, else a walk is mislabelled 'jump' and scored on the looser path.
    feet = ghost["feet"] if "feet" in keys else None
    if feet is not None and np.ndim(feet) == 3:
        airborne = feet[:, :, 2].min(axis=1) > 0.15   # every foot off the ground
        run = 0
        for a in airborne:
            run = run + 1 if a else 0
            if run >= 3:
                return "jump"
    return "walk"


# ---------------------------------------------------------------------------
# per-motion option defaults
# ---------------------------------------------------------------------------
_MOTION_DEFAULTS = {
    "walk":  dict(margin=0.05, base_tol=0.08, flight=False, flight_tol=0.10),
    "getup": dict(margin=0.08, base_tol=0.08, flight=True,  flight_tol=0.12),
    # jumps: a generous free-fall tolerance -- the finite-difference of smoothed
    # qvel is noisy at the ballistic APEX (vertical velocity changes sign), so a
    # genuinely free-falling base reads up to ~15-20% mg there; still an order of
    # magnitude below the levitation signature (~100% mg), so the frozen-apex
    # negative control is rejected cleanly.
    "jump":  dict(margin=0.05, base_tol=0.08, flight=True,  flight_tol=0.20),
    "hill":  dict(margin=0.06, base_tol=0.08, flight=False, flight_tol=0.10),
    "sit":   dict(margin=0.06, base_tol=0.08, flight=False, flight_tol=0.10),
}


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------
def certify(ghost, mjcf, motion="auto", **opts):
    """Hardened motion-agnostic feasibility certificate.

    ghost : npz path (str) OR a loaded npz/dict.
    mjcf  : planner MJCF path (flat planner for hills; terrain injected here).
    motion: 'auto'|'walk'|'getup'|'jump'|'hill'|'sit'|'arm'.
    returns (passed: bool, score: float in [0,1], metrics: dict).
    """
    ghost = _load(ghost)
    probe = mujoco.MjModel.from_xml_path(mjcf)

    if motion == "auto":
        motion = _auto_motion(ghost, probe)

    # ---- arm: delegate to the committed verify_arm (unchanged behaviour) ----
    if motion == "arm":
        from projects.policies.research.shadowing.ghost_verifier import verify_arm
        passed, score, mets = verify_arm(ghost, mjcf, opts.get("sim_dt", 0.004),
                                         opts.get("verbose", False))
        mets = dict(mets)
        mets.update(motion="arm")
        return bool(passed), float(score), mets

    # ---- resolve terrain model + ground height (hills) ----
    model_path, ground_h = _resolve_ground_and_model(ghost, mjcf, motion, opts)

    md = _MOTION_DEFAULTS.get(motion, _MOTION_DEFAULTS["walk"])
    sim_dt = opts.get("sim_dt", 0.004)
    mu = opts.get("mu", 1.0)
    contact_z = opts.get("contact_z", 0.05)
    margin = opts.get("margin", md["margin"])
    smooth_win = opts.get("smooth_win", 9)
    base_tol = opts.get("base_tol", md["base_tol"])
    flight_tol = opts.get("flight_tol", md.get("flight_tol", 0.10))
    flight_frac_max = opts.get("flight_frac_max", 0.60)
    use_flight = opts.get("flight", md["flight"])
    verbose = opts.get("verbose", False)

    m = mujoco.MjModel.from_xml_path(model_path)
    m.opt.timestep = sim_dt
    d = mujoco.MjData(m)
    mg = float(m.body_mass.sum() * G)
    dof_lim = _dof_limits(m)

    q, qvel, dt = ghost["q"], ghost["qvel"], float(ghost["dt"])

    # contact-source config
    support_geoms = opts.get("support_geoms")
    if support_geoms is None and motion == "sit":
        support_geoms = ["chair_seat", "chair_back"]
    cfg = dict(
        motion=motion, margin=margin, contact_z=contact_z, ground_h=ground_h,
        support_geoms=support_geoms or [],
        support_bodies=opts.get("support_bodies"),
        candidate_geoms=candidate_geoms(m, opts.get("support_bodies")),
        walk_foot_geoms=(_foot_geoms_named(m) or candidate_geoms(m, set(_leaf_bodies(m)))),
    )

    qvs = _smooth(qvel, smooth_win)
    T = q.shape[0]

    # ---- pass 1: per-step contact sets + raw "in contact" flags + contact COUNT ----
    raw_in_contact = np.zeros(T, dtype=bool)
    n_contact = np.zeros(T, dtype=int)
    contact_sets = [None] * T
    for k in range(2, T - 2):
        d.qpos[:] = q[k]
        d.qvel[:] = qvs[k]
        mujoco.mj_forward(m, d)
        cs = support_contacts(m, d, cfg)
        contact_sets[k] = cs
        raw_in_contact[k] = len(cs) > 0
        n_contact[k] = len(cs)

    # temporal hysteresis (+-2): a contact active within +-2 steps is active now
    in_contact = raw_in_contact.copy()
    if motion in ("sit", "getup"):
        for k in range(2, T - 2):
            if not raw_in_contact[k] and raw_in_contact[max(0, k - 2):k + 3].any():
                in_contact[k] = True  # marker; we still need a geometric contact set

    # transition mask: +-2 frames around any flight<->stance switch AND any
    # CONTACT-COUNT change are impulse-spike frames (push-off / touchdown: a
    # finite-diff of smoothed qvel spikes for a few frames as feet peel off or
    # slap down) -> excluded from the hard verdict (reported as dropped). For a
    # jump this covers the partial-contact (nc 4->2->0) liftoff ramp, not just
    # the full-airborne switch; a steady grounded walk has no count change here.
    # transition mask: a TIGHT +-1 window around contact-count SWITCHES only
    # (feet peeling off / slapping down spike the finite-diffed base wrench for a
    # frame or two). We deliberately DO NOT exclude every partial-stance frame: a
    # get-up is ENTIRELY partial stance, so blanket exclusion would judge ZERO
    # frames and then rubber-stamp ANY base trajectory over the same joints (a real
    # exploit found in adversarial review: an 8 Hz base shake / 3 m/s sideways drift
    # grafted onto real getup joints passed score 1.00 because n_stance collapsed to
    # 0 and p95 of an empty array is 0). Partial-stance frames are judged on their
    # reconstructed contacts; if too few judged frames survive, the certificate
    # ABSTAINS (indeterminate, see the aggregation guard below) -- never PASSes.
    trans = np.zeros(T, dtype=bool)
    if use_flight:
        nc_seq = n_contact[2:T - 2]
        sw = 2 + np.where(np.diff(nc_seq) != 0)[0]
        for s in sw:
            trans[max(0, s - 2):min(T, s + 3)] = True

    # ---- pass 2: solve LP per step, classify flight ----
    base_res, tau_frac, ncs = [], [], []
    is_flight, dropped = [], 0
    margins = dict(req_mu=[], fric_slack=[], tau_headroom=[], d_zmp=[], k_dot=[])
    for k in range(2, T - 2):
        d.qpos[:] = q[k]
        d.qvel[:] = qvs[k]
        mujoco.mj_forward(m, d)
        qacc = (qvs[k + 1] - qvs[k - 1]) / (2.0 * dt)
        cs = contact_sets[k]
        # hysteresis fill: if marked in_contact but geometric set empty, reuse
        # the nearest neighbour's contact set (sit/getup seated-phase dropouts)
        if not cs and in_contact[k] and motion in ("sit", "getup"):
            for dk in (1, 2, -1, -2):
                if 0 <= k + dk < T and contact_sets[k + dk]:
                    cs = contact_sets[k + dk]
                    break
        res = _step_lp(m, d, qacc, cs, dof_lim, mu, base_tol * mg)
        nc = res["nc"]
        ncs.append(nc)
        if nc == 0:
            # FLIGHT classification
            br = res["base_resid"]  # = ||rhs[:6]||
            if use_flight and br <= flight_tol * mg:
                is_flight.append(True)
                base_res.append(br)
                tau_frac.append(0.0)  # gravity+inertia explain it; actuators free
            else:
                is_flight.append(False)  # levitation OR no-contact-non-flight
                base_res.append(br)
                tau_frac.append(np.inf)
        else:
            is_flight.append(False)
            base_res.append(res["base_resid"])
            tau_frac.append(res["tau_frac"])
            for mk, rk in (("req_mu", "req_mu"), ("fric_slack", "fric_slack"),
                           ("tau_headroom", "tau_headroom"), ("d_zmp", "d_zmp")):
                if rk in res:
                    margins[mk].append(res[rk])
        if trans[k]:
            dropped += 1

    base_res = np.array(base_res, dtype=float)
    tau_frac = np.array(tau_frac, dtype=float)
    ncs = np.array(ncs)
    is_flight = np.array(is_flight, dtype=bool)
    steps = np.arange(2, T - 2)
    keep = ~trans[steps]  # exclude transition frames from the hard verdict

    # takeoff/landing impulse: exclude STANCE frames within FLIGHT_PAD of any flight
    # frame -- the push-off ramp and touchdown impact are a multi-frame impulse the
    # kinematic ghost (penetrating feet) + finite-diff cannot cleanly support, and
    # they are genuinely transient (the steady pre-crouch / post-settle stance is
    # still judged). This is a NO-OP for flightless motions (get-ups have zero flight
    # frames), so it cannot zero out a get-up's judged set or reopen the rubber-stamp
    # exploit -- it only relaxes the verdict around a real ballistic transition.
    if use_flight and is_flight.any():
        pad = 3
        near_flight = np.convolve(is_flight.astype(int), np.ones(2 * pad + 1), mode="same") > 0
        keep = keep & ~(near_flight & (~is_flight))

    stance = (~is_flight) & (ncs > 0) & keep
    flight = is_flight & keep
    nocontact_nonflight = (ncs == 0) & (~is_flight) & keep  # the FAIL signature

    # ---- aggregate (robust p95), per phase, worse phase binds ----
    def p95(a):
        return float(np.percentile(a, 95)) if len(a) else 0.0

    indeterminate = False
    if motion == "walk":
        # BYTE-COMPATIBLE with the committed verify_legged: aggregate base/tau
        # over ALL steps (p95 ignores the transition spikes), 2-term score with
        # s_tau = 1 - tau95 (NOT /1.05), so go2/b2/g1 flat walk reproduce exactly.
        base_frac_stance = p95(base_res) / mg
        base_frac_flight = 0.0
        finite_tau = tau_frac[np.isfinite(tau_frac)]
        tau95 = p95(finite_tau) if len(finite_tau) else np.inf
        no_contact = int((ncs == 0).sum())
        flight_frac = 0.0
        n_bad_nocontact = no_contact
        base_ok = base_frac_stance < base_tol
        tau_ok = tau95 < 1.05
        flight_ok = True
        contact_ok = no_contact <= 0.05 * len(ncs)
        passed = base_ok and tau_ok and contact_ok
        s_base = max(0.0, 1.0 - base_frac_stance / max(base_tol, 1e-6))
        s_tau = max(0.0, 1.0 - tau95) if np.isfinite(tau95) else 0.0
        frac_bad = no_contact / max(1, len(ncs))
        s_contact = max(0.0, 1.0 - frac_bad / 0.05)
        base_frac95 = base_frac_stance
        score = float(max(0.0, min(s_base, s_tau)))  # committed 2-term combination
    else:
        base_stance95 = p95(base_res[stance]) if stance.any() else 0.0
        base_flight95 = p95(base_res[flight]) if flight.any() else 0.0
        base_frac_stance = base_stance95 / mg
        base_frac_flight = base_flight95 / mg
        finite_tau = tau_frac[stance & np.isfinite(tau_frac)]
        tau95 = p95(finite_tau) if len(finite_tau) else (np.inf if stance.any() and not np.isfinite(tau_frac[stance]).any() else 0.0)

        flight_frac = float(flight.sum()) / max(1, keep.sum())
        n_bad_nocontact = int(nocontact_nonflight.sum())

        # gates
        base_ok = (base_frac_stance < base_tol) and (base_frac_flight < flight_tol)
        tau_ok = (tau95 < 1.05)
        flight_ok = (flight_frac <= flight_frac_max)
        contact_ok = (n_bad_nocontact <= 0.05 * max(1, keep.sum()))
        passed = base_ok and tau_ok and flight_ok and contact_ok
        # ABSTAIN guard: the verdict must rest on actually-JUDGED support frames.
        # If too few non-flight frames had a reconstructable contact set (e.g. the
        # model lacks the trunk/belly collider a ground-supported phase needs, or
        # the contact margin is too tight), the certificate cannot honestly certify
        # -> INDETERMINATE, never a vacuous PASS. This is what closes the get-up
        # rubber-stamp exploit: with no judged frames there is nothing to certify.
        n_nonflight = int((keep & (~flight)).sum())
        n_judged = int(stance.sum())
        # Abstain ONLY when we genuinely cannot tell: too few judged frames AND no hard
        # infeasibility signal already firing. A clear levitation (many no-contact-non-
        # free-fall frames -> contact_ok False, or an over-limit base) is a FAIL, not an
        # abstention -- so the indeterminate verdict is reserved for "the model/ghost
        # doesn't give me enough reconstructable support to judge", never used to soften
        # a clear infeasibility into a non-committal answer.
        indeterminate = (n_judged < max(8, int(0.25 * max(1, n_nonflight)))
                         and base_ok and flight_ok and contact_ok)
        if indeterminate:
            passed = False

        # sub-scores
        base_frac95 = max(base_frac_stance, base_frac_flight * (base_tol / flight_tol))
        s_base = max(0.0, 1.0 - base_frac_stance / max(base_tol, 1e-6))
        if flight.any():
            s_base = min(s_base, max(0.0, 1.0 - base_frac_flight / max(flight_tol, 1e-6)))
        s_tau = max(0.0, 1.0 - tau95 / 1.05) if np.isfinite(tau95) else 0.0
        frac_bad = n_bad_nocontact / max(1, keep.sum())
        s_contact = min(max(0.0, 1.0 - flight_frac / flight_frac_max),
                        max(0.0, 1.0 - frac_bad / 0.05))
        score = float(min(s_base, s_tau, s_contact))

    # capturability (reported, NOT gated) -- the necessary-not-sufficient signal
    capture = _capture_margin(ghost, m, d, q, contact_sets, mg) if "com" in (ghost.files if hasattr(ghost, "files") else ghost) else None

    def agg(name, fn):
        v = margins[name]
        return round(float(fn(v)), 4) if v else None

    mets = dict(
        motion=motion, model=os.path.basename(model_path), mg=round(mg, 1),
        base_frac_stance=round(base_frac_stance, 4),
        base_frac_flight=round(base_frac_flight, 4),
        base_frac95=round(base_frac95, 4),
        tau95=round(tau95, 4) if np.isfinite(tau95) else float("inf"),
        flight_frac=round(flight_frac, 4),
        n_flight=int(flight.sum()), n_stance=int(stance.sum()),
        n_bad_nocontact=n_bad_nocontact, dropped_transition=dropped,
        indeterminate=bool(indeterminate),
        base_ok=base_ok, tau_ok=tau_ok, flight_ok=flight_ok, contact_ok=contact_ok,
        s_base=round(s_base, 3), s_tau=round(s_tau, 3), s_contact=round(s_contact, 3),
        req_mu=agg("req_mu", lambda v: np.percentile(v, 95)),
        tau_headroom=agg("tau_headroom", np.min),
        d_zmp=agg("d_zmp", np.median),
        capture=capture,
    )
    if verbose:
        print(f"[certify:{motion}] {os.path.basename(model_path)} mg={mg:.0f}N "
              f"({stance.sum()} stance / {flight.sum()} flight / {n_bad_nocontact} bad-nocontact)")
        print(f"  base stance p95={base_frac_stance*100:.1f}%mg flight p95={base_frac_flight*100:.1f}%mg "
              f"-> {'OK' if base_ok else 'INFEASIBLE'}")
        print(f"  tau p95={tau95*100:.0f}%lim -> {'OK' if tau_ok else 'OVER'}; "
              f"flight_frac={flight_frac*100:.0f}% -> {'OK' if flight_ok else 'TOO MUCH AIR'}")
        print(f"  {'PASS' if passed else 'FAIL'}  score={score:.2f} "
              f"(s_base={s_base:.2f} s_tau={s_tau:.2f} s_contact={s_contact:.2f})  capture={capture}")
    return bool(passed), score, mets


def _capture_margin(ghost, m, d, q, contact_sets, mg):
    """Capturability: xi = x_com + xdot_com/omega, omega=sqrt(g/z_com); signed
    distance of xi to the support polygon edge.  Reported only (necessary-not-
    sufficient).  Returns the median over steps with >=3 contacts."""
    keys = ghost.files if hasattr(ghost, "files") else list(ghost.keys())
    if "com" not in keys:
        return None
    com = ghost["com"]
    dt = float(ghost["dt"])
    vals = []
    T = q.shape[0]
    for k in range(2, T - 2):
        cs = contact_sets[k]
        if not cs or len(cs) < 3:
            continue
        zc = max(0.1, float(com[k, 2]))
        omega = np.sqrt(G / zc)
        vcom = (com[k + 1] - com[k - 1]) / (2.0 * dt)
        xi = com[k, :2] + vcom[:2] / omega
        P = np.array([c[0][:2] for c in cs])
        fz = np.ones(len(P))
        vals.append(_cop_margin_point(P, xi))
    return round(float(np.median(vals)), 4) if vals else None


def _cop_margin_point(xy, pt):
    if len(xy) < 3:
        return 0.0
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(xy)
        mind = np.inf
        for eq in hull.equations:
            dist = -(eq[:2] @ pt + eq[2]) / (np.linalg.norm(eq[:2]) + 1e-12)
            mind = min(mind, dist)
        return float(mind)
    except Exception:
        return 0.0


# ---------------------------------------------------------------------------
# self-test: run over the real ghost library + negative controls
# ---------------------------------------------------------------------------
_M = lambda p: os.path.join(REPO, p)
GO2 = _M("projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
B2 = _M("projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml")
SPOT = _M("projects/policies/research/training/mjcf/spot_newton_fixed2.xml")
G1F = _M("projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
G1SIT = _M("projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml")
GH = lambda n: os.path.join(GHOSTS, n)
SC = lambda n: os.path.join(REPO, "_scratch", n)


def _build_walk(mjcf, out, z=0.322):
    from projects.policies.research.shadowing.ghost_verifier_dyn import _build_kinematic_ghost
    _build_kinematic_ghost(mjcf, lambda t: z, out)


def _levitate(in_npz, out_npz, lift=0.6, t_full=2.0):
    g = dict(np.load(in_npz, allow_pickle=True))
    q, qvel, dt = g["q"].copy(), g["qvel"].copy(), float(g["dt"])
    N = q.shape[0]
    z0 = q[:, 2].copy()
    for k in range(N):
        q[k, 2] = z0[k] + lift * min(1.0, (k * dt) / t_full)
    for k in range(N):
        km, kp = max(0, k - 1), min(N - 1, k + 1)
        qvel[k, 2] = (q[kp, 2] - q[km, 2]) / ((kp - km) * dt or 1.0)
    np.savez(out_npz, q=q, qvel=qvel, dt=np.float64(dt))


def _frozen_apex_jump(in_npz, out_npz):
    """Hold the base at apex z with zero base velocity -> levitation signature."""
    g = dict(np.load(in_npz, allow_pickle=True))
    q, qvel, dt = g["q"].copy(), g["qvel"].copy(), float(g["dt"])
    zmax = float(q[:, 2].max())
    q[:, 2] = zmax
    qvel[:, 2] = 0.0
    out = {k: g[k] for k in g}
    out["q"] = q
    out["qvel"] = qvel
    np.savez(out_npz, **out)


def _graft_base(in_npz, out_npz, kind):
    """Graft a physically-impossible BASE trajectory onto the real joint angles
    (the adversarial test that broke the early get-up branch): kind='shake' = 8 Hz
    +-0.15 m vertical base oscillation; kind='drift' = 3 m/s steady sideways base
    translation with feet planted. A sound certificate must NOT return PASS."""
    g = dict(np.load(in_npz, allow_pickle=True))
    q, qvel, dt = g["q"].copy(), g["qvel"].copy(), float(g["dt"])
    t = np.arange(q.shape[0]) * dt
    if kind == "shake":
        q[:, 2] = q[:, 2] + 0.15 * np.sin(2 * np.pi * 8.0 * t)
        qvel[:, 2] = np.gradient(q[:, 2], dt)
    else:  # drift
        q[:, 1] = q[:, 1] + 3.0 * t
        qvel[:, 1] = 3.0
    out = {k: g[k] for k in g}
    out["q"] = q
    out["qvel"] = qvel
    np.savez(out_npz, **out)


def _self_test():
    from projects.policies.research.shadowing.certify_quadruped_biped import build_b2_walk_ghost, build_g1_walk_ghost

    # build walk ghosts + negative / adversarial controls
    _build_walk(GO2, SC("cert_go2_walk.npz"))
    build_b2_walk_ghost(B2, SC("cert_b2_walk.npz"))
    build_g1_walk_ghost(G1F, SC("cert_g1_walk.npz"))
    _levitate(SC("cert_go2_walk.npz"), SC("cert_go2_levi.npz"))
    _frozen_apex_jump(GH("spot_jump_ghost.npz"), SC("cert_spot_frozenjump.npz"))
    _graft_base(GH("spot_getup_ghost.npz"), SC("cert_sg_shake.npz"), "shake")
    _graft_base(GH("spot_getup_ghost.npz"), SC("cert_sg_drift.npz"), "drift")

    # expect: 'pass' = must PASS ; 'fail' = must FAIL (strictly infeasible, NOT indeterminate) ;
    #         'nopass' = must NOT pass (FAIL or INDETERMINATE both acceptable -- a documented
    #          ghost/model boundary, or an adversarial control we only require is not rubber-stamped).
    cases = [
        # (tag, ghost, mjcf, motion, expect)
        ("go2 walk",           SC("cert_go2_walk.npz"),   GO2,  "walk",  "pass"),
        ("b2 walk",            SC("cert_b2_walk.npz"),    B2,   "walk",  "pass"),
        ("g1 walk",            SC("cert_g1_walk.npz"),    G1F,  "walk",  "pass"),
        ("spot crouch",        GH("spot_crouch_ghost.npz"), SPOT, "auto", "pass"),
        ("spot getup",         GH("spot_getup_ghost.npz"),  SPOT, "getup", "pass"),
        ("b2 getup",           GH("b2_getup_ghost.npz"),    B2,   "getup", "pass"),
        ("b2 hill6",           GH("b2_hill6_ghost.npz"),    B2,   "hill",  "pass"),
        ("spot hill8",         GH("spot_hill8_ghost.npz"),  SPOT, "hill",  "pass"),
        ("g1 sitstand",        GH("g1_sitstand_ghost.npz"), G1SIT,"sit",   "pass"),
        # boundary: the explosive jump's kinematic ghost is not contact-consistent at the
        # push-off (penetrating feet + finite-diff base accel) -> base-infeasible loading
        # frames. The certificate FLAGS this honestly (it does not rubber-stamp); a clean
        # certification needs the generator's recorded contact forces, not a kinematic snapshot.
        ("spot jump",          GH("spot_jump_ghost.npz"),   SPOT, "jump",  "nopass"),
        # strict negatives (must FAIL, not merely abstain):
        ("go2 levitate",       SC("cert_go2_levi.npz"),     GO2,  "walk",  "fail"),
        ("spot frozen-jump",   SC("cert_spot_frozenjump.npz"), SPOT, "jump", "fail"),
        # adversarial: impossible base trajectory grafted onto real getup joints (the exploit
        # that broke the early branch) -- must NOT PASS (FAIL or INDETERMINATE):
        ("spot getup+shake",   SC("cert_sg_shake.npz"),     SPOT, "getup", "nopass"),
        ("spot getup+drift",   SC("cert_sg_drift.npz"),     SPOT, "getup", "nopass"),
    ]

    def _ok(expect, passed, indet):
        if expect == "pass":
            return bool(passed)
        if expect == "fail":
            return (not passed) and (not indet)
        return not passed  # 'nopass': FAIL or INDETERMINATE

    print(f"{'case':18s} {'motion':7s} {'verdict':7s} {'score':>5s}  "
          f"{'s_base':>6s} {'s_tau':>5s} {'s_con':>5s}  {'expect':6s} {'OK?'}")
    print("-" * 88)
    allok = True
    rows = []
    for tag, gp, mjcf, motion, want in cases:
        try:
            passed, score, mets = certify(gp, mjcf, motion=motion)
        except Exception as e:
            print(f"{tag:18s} ERROR: {type(e).__name__}: {e}")
            allok = False
            continue
        indet = bool(mets.get("indeterminate"))
        verdict = "PASS" if passed else ("INDET" if indet else "FAIL")
        match = _ok(want, passed, indet)
        allok = allok and match
        rows.append((tag, mets.get("motion", motion), passed, score, mets, want, match))
        print(f"{tag:18s} {mets.get('motion',motion):7s} {verdict:7s} "
              f"{score:5.2f}  {mets.get('s_base',0):6.2f} {mets.get('s_tau',0):5.2f} "
              f"{mets.get('s_contact',0):5.2f}  {want:6s} "
              f"{'ok' if match else 'MISMATCH'}")
    print("-" * 88)
    print(f"[feasibility-certificate] {'ALL CORRECT' if allok else 'MISMATCH(ES) PRESENT'}")
    # extra detail for the contact-rich cases
    print("\nbalance margins (contact-rich cases):")
    for tag, mo, passed, score, mets, want, match in rows:
        if mo in ("getup", "jump", "hill", "sit"):
            print(f"  {tag:16s} base_stance={mets['base_frac_stance']*100:5.1f}%mg "
                  f"base_flight={mets['base_frac_flight']*100:5.1f}%mg "
                  f"flight={mets['flight_frac']*100:4.0f}% bad_nc={mets['n_bad_nocontact']} "
                  f"d_zmp={mets.get('d_zmp')} capture={mets.get('capture')}")
    return allok


if __name__ == "__main__":
    ok = _self_test()
    sys.exit(0 if ok else 1)
