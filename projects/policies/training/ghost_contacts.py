#!/usr/bin/env python3
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

"""ghost_contacts.py -- contact-set specs for ghost validation, and the Contact-Wrench-Cone test.

WHY THIS EXISTS
ghost_dynamics.py v1 hardcoded "two G1 ankles, rectangular feet, flat ground at z=0, CoP solved on the
z=0 plane, double-support frames skipped". Those assumptions are invisible in the output, so it returns
plausible-looking numbers that are WRONG for three ghost classes we already ship:

    stairs  feet rest on treads at DIFFERENT heights -> the z=0 CoP plane is meaningless
    crawl   the contacts are HANDS and KNEES, not ankles -> it validates the wrong bodies
    carry   the payload is not in the model -> the wrench is short by the payload's weight

THE GENERAL TEST
Drop CoP-in-support as the primitive. For a single planar foot the Contact Wrench Cone decomposes in
closed form into exactly three conditions -- Coulomb friction, CoP inside the support area, and a bound
on the yaw (foot-spin) torque (Caron, Pham & Nakamura, ICRA 2015, arXiv:1501.04719). All three are
special cases of one question that stays meaningful for ANY number of contacts at ANY heights:

    do there EXIST contact forces, each inside its own friction cone and pressing (unilateral),
    whose resultant equals the centroidal wrench the motion demands?

That is a linear feasibility program. Solving it directly generalizes to hands+knees, to feet on
different stair treads, and to double support, with no special cases. Adding the joint-torque check on
top of the resulting force distribution is the Feasible Wrench Polytope, FWP = AWP ∩ CWC
(Orsolino et al. RA-L 2018, arXiv:1712.02731; arXiv:2202.01628).

⛔ We use the CWC as a CHECK, never as an objective. Maximising a stability margin is what our own
3-arm experiment (commit 621a9850) showed to be worse than useless: margin is bought with dynamic
aggressiveness, and it was velocity -- not margin -- that decided whether the policy could track.

The friction cone is LINEARIZED (4 edges per corner), so feasibility here is a conservative inner
approximation: FEASIBLE is trustworthy, INFEASIBLE deserves a second look at the edge count.

=== THE TWO THINGS A CONTACT SPEC MUST SAY, AND WE USED TO GET BOTH WRONG =====================

⛔ 1. WHERE THE PATCH IS. It is NOT the body origin. The G1's foot collision box hangs off the ankle:
   `pos=[0.035, 0, -0.030] size=[0.085, 0.03, 0.006]` -> the sole plane is 36 mm BELOW and 35 mm
   AHEAD of `*_ankle_roll_link`. Placing the patch at the origin puts the whole support polygon 3.5 cm
   behind the real one, which silently biases every COM-margin and CoP number we have ever printed.
   `patch` is now the measured offset, in body coordinates, of the CONTACT PLANE's centre.

⛔ 2. WHAT THE PATCH RESTS ON. A single scalar `surface_z` is a flat world. Stairs, ramps and rubble
   are the interesting ghosts. `terrain` makes the support height a FUNCTION of position, z = f(x,y),
   with the surface normal taken from its local gradient. A stair ghost validated against `surface_z=0`
   reports "flight" on every raised foot and then reports nothing at all -- a validator that quietly
   grades the wrong world.

Terrain types: flat | stairs | ramp | heightmap.  The CWC/FWP linear program never assumed coplanarity,
so non-coplanar support needs no special case; only the CoP==ZMP identity does, and that is refused.
"""
import json
import math

import numpy as np

# ---------------------------------------------------------------------------------------------------
# TERRAIN: the support surface as z = f(x, y), plus its normal.
# ---------------------------------------------------------------------------------------------------

def make_terrain(t):
    """Accept None, a dict, or a compact string 'stairs:riser=0.07,going=0.26,x0=0.12,n=5'."""
    if t is None:
        return {"type": "flat", "z": 0.0}
    if isinstance(t, dict):
        return t
    s = str(t)
    if ":" not in s:
        return {"type": s}
    head, rest = s.split(":", 1)
    out = {"type": head}
    for kv in rest.split(","):
        if not kv.strip():
            continue
        k, v = kv.split("=", 1)
        out[k.strip()] = float(v) if k.strip() != "n" else int(float(v))
    return out


def surface_z(T, x, y=0.0):
    """Height of the support surface under (x, y)."""
    ty = T.get("type", "flat")
    if ty == "flat":
        return float(T.get("z", 0.0))
    if ty == "stairs":
        # tread k (1-based) spans x in [x0 + (k-1)*going, x0 + k*going) and sits at z0 + k*riser.
        # Matches design_stair_climb_ghost.tread_x/tread_z: foot x on tread k is x0 + (k-0.5)*going.
        r, g = float(T["riser"]), float(T["going"])
        x0, z0, n = float(T.get("x0", 0.0)), float(T.get("z0", 0.0)), int(T["n"])
        if x < x0:
            return z0
        k = min(int(math.floor((x - x0) / g)) + 1, n)
        return z0 + max(0, k) * r
    if ty == "ramp":
        x0, sl = float(T.get("x0", 0.0)), float(T["slope"])
        x1 = float(T.get("x1", 1e9))
        return float(T.get("z0", 0.0)) + sl * (min(max(x, x0), x1) - x0)
    if ty == "heightmap":
        return float(np.interp(x, np.asarray(T["x"], float), np.asarray(T["z"], float)))
    raise ValueError("unknown terrain type: %s" % ty)


def surface_normal(T, x, y=0.0):
    """Unit normal of the support surface. Flat treads and floors -> +z; ramps/heightmaps tilt.

    A stair RISER is vertical and we never push on it, so the stairs normal is +z everywhere; the
    discontinuity at the nosing has no normal, and a corner landing there is simply not in contact.
    """
    ty = T.get("type", "flat")
    if ty in ("flat", "stairs"):
        return np.array([0.0, 0.0, 1.0])
    h = 1e-4
    dz = (surface_z(T, x + h, y) - surface_z(T, x - h, y)) / (2 * h)
    n = np.array([-dz, 0.0, 1.0])
    return n / np.linalg.norm(n)


# ---------------------------------------------------------------------------------------------------
# CONTACT SPECS. `patch` = contact-plane centre in BODY coordinates (measured from the collision geom).
# ---------------------------------------------------------------------------------------------------

# G1 foot: collision box pos=[0.035,0,-0.030] size=[0.085,0.03,0.006] -> sole plane at body z = -0.036.
G1_FEET = {
    "mu": 1.0,
    "swing_tol": 0.012,
    "terrain": {"type": "flat", "z": 0.0},
    "contacts": [
        {"name": "foot_L", "body": "left_ankle_roll_link", "half": [0.085, 0.03], "patch": [0.035, 0.0, -0.036]},
        {"name": "foot_R", "body": "right_ankle_roll_link", "half": [0.085, 0.03], "patch": [0.035, 0.0, -0.036]},
    ],
}

# hands + knees: what the crawl ghost actually stands on. Patch sizes are deliberately small and
# conservative -- a hand is not a foot; if you want a real number, measure the collision geom.
G1_CRAWL = {
    "mu": 1.0,
    "swing_tol": 0.012,
    "terrain": {"type": "flat", "z": 0.0},
    "contacts": [
        {"name": "knee_L", "body": "left_knee_link", "half": [0.04, 0.03], "patch": [0.0, 0.0, 0.0]},
        {"name": "knee_R", "body": "right_knee_link", "half": [0.04, 0.03], "patch": [0.0, 0.0, 0.0]},
        {"name": "hand_L", "body": "left_wrist_roll_rubber_hand", "half": [0.035, 0.025], "patch": [0.0, 0.0, 0.0]},
        {"name": "hand_R", "body": "right_wrist_roll_rubber_hand", "half": [0.035, 0.025], "patch": [0.0, 0.0, 0.0]},
    ],
}

PRESETS = {"feet": G1_FEET, "crawl": G1_CRAWL}


def load_spec(name_or_path):
    if name_or_path in PRESETS:
        return json.loads(json.dumps(PRESETS[name_or_path]))   # deep copy
    return json.load(open(name_or_path))


def corners(pos, rot3x3, half, patch=(0.0, 0.0, 0.0), axes=None):
    """The four corners of a rectangular contact patch, in world coords.

    `patch` is the patch centre in BODY coordinates. Defaulting it to the body origin is what put the
    G1's support polygon 3.5 cm behind its real one for the whole first generation of this tool.

    `axes` are the two BODY-frame unit vectors spanning the patch plane (default x, y -- a sole).
    ⛔ Not every contact is a sole: the G1's knee/shin box bears on its +x FACE, so its patch plane is
    the body y-z plane, and a kneeling ghost validated with x-y corners puts the support polygon
    through the shin bone instead of along it.
    """
    hx, hy = half
    a1, a2 = (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])) if axes is None else \
             (np.asarray(axes[0], float), np.asarray(axes[1], float))
    c = np.asarray(pos, float) + rot3x3 @ np.asarray(patch, float)
    return [c + rot3x3 @ (sx * hx * a1 + sy * hy * a2) for sx in (+1, -1) for sy in (+1, -1)]


def hull2d(pts):
    """Convex hull of 2-D points (Andrew monotone chain). Returns CCW vertices."""
    p = sorted(set((round(float(a), 9), round(float(b), 9)) for a, b in pts))
    if len(p) <= 2:
        return [np.array(q) for q in p]
    def half(ps):
        h = []
        for q in ps:
            while len(h) >= 2 and (h[-1][0]-h[-2][0])*(q[1]-h[-2][1]) - (h[-1][1]-h[-2][1])*(q[0]-h[-2][0]) <= 0:
                h.pop()
            h.append(q)
        return h
    lower, upper = half(p), half(p[::-1])
    return [np.array(q) for q in (lower[:-1] + upper[:-1])]


def margin_in_hull(pt, hull):
    """Signed distance of pt inside the convex hull (positive = inside, = distance to nearest edge).

    A DEGENERATE hull is not an error and must not return -1e9: once contact is decided per corner, a
    pitched foot at heel-strike or toe-off legitimately supports on an EDGE (2 points) or a point. Its
    margin is then <= 0 by construction -- the distance to the segment, negated. Returning a sentinel
    instead poisons the min-over-frames and hides every real margin behind one geometric artefact.
    """
    pt = np.asarray(pt, float)[:2]
    if len(hull) == 0:
        return -1e9
    if len(hull) == 1:
        return -float(np.linalg.norm(pt - hull[0][:2]))
    if len(hull) == 2:
        a, b = np.asarray(hull[0], float)[:2], np.asarray(hull[1], float)[:2]
        e = b - a; L2 = float(e @ e)
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, float((pt - a) @ e) / L2))
        return -float(np.linalg.norm(pt - (a + t * e)))
    d = 1e9
    n = len(hull)
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        e = b - a
        L = math.hypot(e[0], e[1])
        if L < 1e-12:
            continue
        # CCW hull -> inside is to the LEFT of each edge
        cross = (e[0] * (pt[1] - a[1]) - e[1] * (pt[0] - a[0])) / L
        d = min(d, cross)
    return d


def cone_edges(normal, mu, n_edge=4):
    """Linearized friction-cone edge directions about `normal` (conservative inner approximation)."""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    a = np.array([1.0, 0.0, 0.0])
    if abs(n @ a) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    t1 = np.cross(n, a); t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    return [n + mu * (math.cos(th) * t1 + math.sin(th) * t2)
            for th in (2 * math.pi * k / n_edge for k in range(n_edge))]


def build_columns(active, mu):
    """One LP column per (contact corner, friction-cone edge). Returns (cols, pts, owner, corner_pos, cbid).

    `owner[j]` maps column j back to its corner -- cone-edges-per-corner is NOT a usable stride, and
    assuming it was silently zeroed every contact force (base residual jumped to 100% of body weight).
    `cbid[i]` is the body each corner belongs to, carried explicitly rather than recovered by a
    nearest-corner search: on stairs two feet can be closer to each other than a foot is to itself.
    Each corner brings its OWN surface normal -- a foot straddling a nosing has corners on two planes.
    """
    corner_pos, cols, pts, owner, cbid = [], [], [], [], []
    for c in active:
        nrms = c.get("normals") or [c.get("normal", (0, 0, 1))] * len(c["corners"])
        for corner, nrm in zip(c["corners"], nrms):
            ci = len(corner_pos)
            corner_pos.append(np.asarray(corner, float)); cbid.append(c["bid"])
            for e in cone_edges(nrm, mu):
                cols.append(np.asarray(e, float)); pts.append(np.asarray(corner, float)); owner.append(ci)
    return cols, pts, owner, corner_pos, cbid


def support_demand(Gb, wb6, Gj, qfi_j, lims):
    """How much EXTERNAL wrench must something other than the contacts supply?

    Solve:  minimise ||e||_1  s.t.  Gb x + e = wb6,  x >= 0,  |tau(x)| <= limits.
    e is the residual wrench the feet cannot produce -- a crane, a handrail, a wall. It is 0 exactly
    when the motion is self-supporting, and its size says how far from self-supporting it is.

    ⛔ WHY THIS MATTERS TO US. Our "achieved" ghosts are recorded from policies running under the crane
    harness (HARNESS_KZ / HARNESS_FY / attitude PD). A crane applies external wrenches, so a recorded
    motion is NOT evidence of free-standing feasibility -- and "it was achieved, therefore it is
    achievable" is exactly the inference that hides this. Measured on the walk ghost: the COM never
    leaves the centreline (+/-15 mm) while the stance foot sits 80-100 mm to the side. No contact force
    distribution can hold that; the crane was doing it. This number quantifies the debt.

    Returns (force_N, torque_Nm) of the required external wrench, or (inf, inf) if even that cannot make
    the joint torques fit.
    """
    from scipy.optimize import linprog
    n = Gb.shape[1]
    nj = len(lims)
    # z = [x (n), e+ (6), e- (6)];  minimise sum(e+ + e-)
    c = np.concatenate([np.zeros(n), np.ones(12)])
    A_eq = np.hstack([Gb, np.eye(6), -np.eye(6)])
    A_ub = np.zeros((2 * nj, n + 12)); b_ub = np.zeros(2 * nj)
    A_ub[:nj, :n] = -Gj; b_ub[:nj] = np.asarray(lims) - np.asarray(qfi_j)
    A_ub[nj:, :n] = Gj;  b_ub[nj:] = np.asarray(lims) + np.asarray(qfi_j)
    r = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=wb6,
                bounds=[(0, None)] * n + [(0, None)] * 12, method="highs")
    if not r.success:
        return float("inf"), float("inf")
    e = r.x[n:n + 6] - r.x[n + 6:n + 12]
    return float(np.linalg.norm(e[0:3])), float(np.linalg.norm(e[3:6]))


def fwp_check_G(Gb, wb6, Gj, qfi_j, lims, owner, cols, ncorner):
    """FWP membership posed entirely in GENERALIZED coordinates.

    Gb (6 x n)  : base-dof generalized force produced by a unit weight on each friction-cone edge
    wb6 (6,)    : qfrc_inverse's floating-base rows -- what the contacts must supply
    Gj, qfi_j   : the same for the actuated joints, and the torques inverse dynamics demands
    Minimise t = peak |tau| / limit over all valid distributions; t <= 1 means the wrench is in the FWP.

    Posing the equality on the SAME generalized rows that produced the torques removes an entire class
    of bug: a wrench assembled independently (Newton-Euler on com_ddot / Ldot) disagrees with the
    inverse dynamics by the finite-difference error, which is invisible when the motion is near-static
    and ruinous when it is not.
    """
    from scipy.optimize import linprog
    n = Gb.shape[1]
    nj = len(lims)
    c = np.zeros(n + 1); c[-1] = 1.0
    A_ub = np.zeros((2 * nj, n + 1)); b_ub = np.zeros(2 * nj)
    A_ub[:nj, :n] = -Gj; A_ub[:nj, n] = -np.asarray(lims); b_ub[:nj] = -np.asarray(qfi_j)
    A_ub[nj:, :n] = Gj;  A_ub[nj:, n] = -np.asarray(lims); b_ub[nj:] = np.asarray(qfi_j)
    A_eq = np.hstack([Gb, np.zeros((6, 1))])
    r = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=wb6,
                bounds=[(0, None)] * n + [(0, None)], method="highs")
    if not r.success:
        return False, float("inf"), None
    x = r.x[:n]; t = float(r.x[n])
    f = np.zeros((ncorner, 3))
    for j in range(n):
        f[owner[j]] += x[j] * cols[j]
    return True, t, f


def fwp_check(cols, pts, owner, ncorner, com, wrench, Gj, qfi_j, lims):
    """FWP membership = CWC ∩ actuation, as ONE linear program.

    Does there exist a contact-force distribution x >= 0 (weights on friction-cone edges) such that
      (i)  its resultant equals the demanded centroidal wrench   A x = w        [CWC]
      (ii) the joint torques it leaves behind fit the effort limits             [actuation]
    We minimise t = the peak torque ratio over all valid distributions, so the reported number is the
    BEST achievable, not an arbitrary LP vertex. t <= 1 -> the wrench lies inside the FWP.

    With redundant contacts (double support) the force split is NOT unique, so joint torque is a
    property of the CONTROLLER's distribution, not of the ghost. Only the existence question is
    well-posed -- which is exactly what this answers.
    """
    from scipy.optimize import linprog
    n = len(cols)
    A = np.zeros((6, n))
    for j in range(n):
        A[0:3, j] = cols[j]
        A[3:6, j] = np.cross(pts[j] - com, cols[j])
    nj = len(lims)
    # variables z = [x (n), t (1)];  minimise t
    c = np.zeros(n + 1); c[-1] = 1.0
    # torque rows:  |qfi_j - Gj x| <= t * lim   ->   -Gj x - t*lim <= -qfi   and   Gj x - t*lim <= qfi
    A_ub = np.zeros((2 * nj, n + 1)); b_ub = np.zeros(2 * nj)
    A_ub[:nj, :n] = -Gj; A_ub[:nj, n] = -np.asarray(lims); b_ub[:nj] = -np.asarray(qfi_j)
    A_ub[nj:, :n] = Gj;  A_ub[nj:, n] = -np.asarray(lims); b_ub[nj:] = np.asarray(qfi_j)
    A_eq = np.hstack([A, np.zeros((6, 1))])
    r = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=wrench,
                bounds=[(0, None)] * n + [(0, None)], method="highs")
    if not r.success:
        return False, float("inf"), None
    x = r.x[:n]; t = float(r.x[n])
    f = np.zeros((ncorner, 3))
    for j in range(n):
        f[owner[j]] += x[j] * cols[j]
    return True, t, f


def cwc_feasible(active, com, wrench, mu, robustness_dirs=True):
    """Do contact forces exist -- each in its friction cone, each pressing -- whose resultant equals
    the demanded centroidal wrench? Returns (feasible, forces_per_corner, robustness_N).

    Variables x >= 0 weight the cone edges at every corner of every ACTIVE contact patch. Six equality
    rows fix the resultant force and the resultant moment about the COM. `robustness_N` is the largest
    horizontal wrench perturbation (N, worst of +/-x, +/-y) that stays feasible -- a wrench-space margin.
    """
    from scipy.optimize import linprog
    # ⛔ `owner` must map each column back to its corner explicitly: cone-edges-per-corner and corners
    # are NOT the same stride, and assuming they were silently zeroed every contact force.
    cols, pts, owner, corner_pos, _ = build_columns(active, mu)
    if not cols:
        return False, None, 0.0
    n = len(cols)
    A = np.zeros((6, n))
    for j, (e, p) in enumerate(zip(cols, pts)):
        A[0:3, j] = e
        A[3:6, j] = np.cross(p - com, e)
    res = linprog(np.zeros(n), A_eq=A, b_eq=wrench, bounds=[(0, None)] * n, method="highs")
    if not res.success:
        return False, None, 0.0
    f = np.zeros((len(corner_pos), 3))
    for j, e in enumerate(cols):
        f[owner[j]] += res.x[j] * e
    forces = list(zip(corner_pos, f))

    rob = 1e9
    if robustness_dirs:
        for u in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]):
            uu = np.zeros(6); uu[0:3] = u
            # max eps s.t. A x - eps*u = w, x >= 0
            A2 = np.hstack([A, -uu.reshape(6, 1)])
            c2 = np.zeros(n + 1); c2[-1] = -1.0            # maximize eps
            r2 = linprog(c2, A_eq=A2, b_eq=wrench, bounds=[(0, None)] * n + [(0, None)], method="highs")
            rob = min(rob, float(r2.x[-1]) if r2.success else 0.0)
    return True, forces, (0.0 if rob > 1e8 else rob)
