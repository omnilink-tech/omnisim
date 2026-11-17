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

"""ghost_synth_pushup.py -- SYNTHESIZE a PUSH-UP ghost: hands + toes, ~90 deg base pitch, ARM-CHAIN solve.

THE METHOD (ghost_synth.py, 2026-07-09 doctrine, generalized to a prone motion)
Plan the CONTACTS. Solve everything else. Handholds and toe-holds are world poses on flat ground,
pinned for the WHOLE routine (a push-up never moves its contacts -- deliberately the simplest possible
arms motion: no switching, no swings). Per frame we solve base (x, y, z) + base PITCH + the 12 leg
joints + the 10 arm-chain joints by Levenberg-Marquardt on the real mujoco Jacobians, so that the
palms lie flat on their handholds, the toe strips stay tangent on their footholds, the COM sits on a
designed point between toes and hands, and the base follows a C2 stroke profile. Nothing is a formula.

THIS IS THE FIRST GHOST WHERE (extensions E4 + E5 of the library plan, both live here):
  E4  the base ATTITUDE is a SOLVED variable. Unknown vector = [x, y, z, pitch] + joints; the
      quaternion is built as [cos(p/2), 0, sin(p/2), 0] in _fk, and the Jacobian gets ONE extra
      column: base rotational dof 4 (the free joint's y-rotation; with roll = yaw = 0 the body y axis
      IS the world y axis, so the column is exact, not an approximation). A low-weight nominal-pitch
      row (W_ATT, the analog of W_BZ) kills nullspace wander; it never fights closure.
  E5  ARM JOINTS are solved variables for hand contacts. ghost_close.path_joints(m, bid) walks the
      kinematic path base -> body for ANY body, so a hand contact's jdof automatically includes
      shoulder pitch/roll/yaw + elbow + wrist_roll. ONE filter on top of it: path_joints honestly
      reports waist_yaw on the hand path; the push-up is sagittal and the plan keeps the waist at 0,
      so waist_yaw is excluded from the unknowns instead of being solved into a trunk twist.
      There is NO per-frame arm formula anywhere in this file (the origin's counter-swing formula
      would overwrite a solved variable every LM iteration -- a fight the solver loses).

=== MEASURED GEOMETRY FACTS this design stands on (probe 2026-07-09, this campaign) =================
 1. THE PALM FACE. The rubber-hand collision box is [0.1267, -/+0.0062, +0.0117] half
    [0.1265, 0.036, 0.0535] on `*_wrist_roll_rubber_hand` (crawl URDF; the base MJCF carries no hand
    collider -- the patch is a spec entry, not a geom). The palm is the box's -z FACE: patch centre
    [0.1267, -/+0.0062, -0.0418], plane = body x-y. Contact on the -z face means body +z aligns with
    the surface normal -- the SAME orientation residual a foot sole uses. Realistic palm half
    [0.06, 0.03] (the full box would claim fingertip-to-heel-of-hand CoP authority; do not).
 2. NO WRIST PITCH EXISTS. The arm is shoulder p/r/y + elbow + wrist ROLL (about the forearm axis,
    hand body x). The palm normal (hand body -z) is PERPENDICULAR to the forearm, so
        palm flat on the ground  <=>  forearm HORIZONTAL.
    A straight vertical arm can NEVER plant a flat palm on the floor -- which is why the library
    probe's straight-arm (elbow 1.55) "push-up TOP" fit came out contorted (fit residual 3e-1,
    shoulder_pitch -2.91 on the -3.09 stop). The achievable flat-palm plank family is all-sagittal:
        shoulder_pitch = -(base_pitch + elbow),  roll = yaw = wrist = 0
    (verified: hand +z maps to world [0.05..0.07, y, 0.97+] across the family before the wrist/roll
    micro-corrections the solver adds).
 3. THE STROKE LIVES ON THE ELBOW 0 -> ~1.0, NOT 1.45 -> 0.2. Elbow convention: 0 = 90 deg BENT,
    +1.45 = straight (max shoulder->palm reach 0.420 m, measured; 1.55 is already past the optimum).
    In the flat-palm family the upper-arm elevation is (elbow - pi/2): elbow 0 = upper arm VERTICAL,
    shoulder 0.193 m above the elbow -- that is the TALLEST flat-palm posture, i.e. the push-up TOP
    reads elbow ~0.05-0.15 on the joint. Bending to elbow ~1.0 drops the shoulder 0.193*(1 - cos e)
    ~ 0.09 m and slides it aft 0.193*sin(e) ~ 0.16 m. So the joint-space stroke is INVERTED vs the
    naive reading of the convention; the request's "elbow 1.4-1.55 at top" is geometrically
    impossible with flat palms and is documented here as such.
 4. THE TOE CONTACT IS AN EDGE, MODELED HONESTLY. Foot box [0.035, 0, -0.030] half
    [0.085, 0.03, 0.006]: sole plane z = -0.036, TOE EDGE at x = +0.120. In a push-up the foot is
    pitched ~70-85 deg nose-down and bears on that edge -- a LINE contact. The spec gives the toe its
    own patch FRAME: centre AT the physical edge [0.120, 0, -0.036], axes
    [[cos(tp), 0, sin(tp)], [0, 1, 0]] (tp = design toe pitch), i.e. the plane tangent to the ground
    when the foot is pitched by tp, and a TINY sagittal half (0.004 m ~ rubber compliance) so the FWP
    cannot invent CoP lever the hardware does not have. (A 0.02 m half -- floated in the request --
    would both overhang the physical toe by 1 cm and claim 4 cm of fictional lever; the library doc's
    0.01 centred at 0.110 leaves the strip's rear corner pretending contact where the pitched sole is
    ~20 mm in the air. Centre-at-the-edge + small half has zero penetration and near-zero fiction.)
 5. WITH ALL FOUR CONTACTS PINNED the arm chain is fully determined by the base pose (5 rows on
    5 dofs per arm; the one soft direction, palm yaw about the normal, is physically free -- E5's
    rule: NO yaw row on a palm, pinning it saturates wrist_roll exactly like the ankle_roll lesson --
    and the W_REG posture row polices it). So the nominal-elbow row and the base-z row are two
    handles on the SAME degrees of freedom: both are calibrated from the SAME keyframe pre-solve and
    cannot fight.

WHY THE KEYFRAME PRE-SOLVE EXISTS (the stroke is measured, not assumed)
The pelvis is neither the shoulders nor the COM: authoring a pelvis-z stroke by hand re-introduces
the base-as-formula defect this method exists to kill. Instead:
  KF0   solve the TOP posture with the hand-x row DROPPED (hands find natural reach under the
        shoulders) -> FREEZE the handholds at the solved palm centres.
  KF-TOP / KF-LOW  re-solve with hands pinned and the elbow PINNED (W_ELB_KF) at e_top / e_low
        -> read off base z, pitch, and the achieved COM.
  TRAJ  C2 (quintic) profiles between the keyframe values drive W_BZ / W_ATT / W_ELB rows; every
        other row is closure. The COM target interpolates the ACHIEVED keyframe COM (biased toward
        the toes by --com-frac at the keyframes), so the COM row never chases a point the geometry
        cannot reach.

LOAD DESIGN (gate 3 as a design input, not a hope)
Contacts are hands (front) and toes (back): moving the COM toward the TOES shifts load to the LEGS,
whose joints have 88-139 N*m budgets, and off the ARMS, whose 25 N*m shoulder_pitch is the binding
actuator (probe: hand force x the wrist->palm lever ~ 0.17-0.23 m is the whole shoulder story).
--com-frac is the fraction of toe->palm distance: 0.5 = the LP-probed 50/50 split; default 0.44
biases toward the toes as far as a mild pike can carry it. Tempo is slow (1.5 s strokes) because the
LP reports the BEST split and controller splits are worse (crawl campaign: 70-100% measured where
the LP said 34%).

THE LUT STARTS AT THE PLANK TOP. Getting down from standing (and back up) is the GET-UP motion's
job (library motion 10), not this ghost's; a BATON sequence composes them. seq=true, hold_end=true.

Usage:
  python ghost_synth_pushup.py projects/policies/ghosts/g1/ghost_pushup_synth_lut.json
  python build_keypoints.py <out> --links
  python ghost_dynamics.py <out> --contacts projects/policies/training/specs/contacts_pushup.json \
      --smooth 7 --stride 4
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ghost_synth as GS       # the frozen origin: DT, WB_JOINTS, quintic, DEF_MJCF, weights
import ghost_close as GCL      # path_joints -- generalizes the contact chain to ARMS

DT = GS.DT
WB_JOINTS = GS.WB_JOINTS
RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEF_SPEC_OUT = os.path.join(RT, "projects/policies/training/specs/contacts_pushup.json")

# weights -- origin: ghost_synth.py (W_FOOT renamed W_CON: the contacts are hands + toes now).
# All the origin's row lessons hold: full-strength rows or deleted rows, never down-weighted ones.
W_CON, W_ORI, W_COM, W_BZ, W_REG = GS.W_FOOT, GS.W_ORI, GS.W_COM, GS.W_BZ, GS.W_REG
W_ATT = 0.5        # nominal-pitch row (E4): regularizer against nullspace wander, never a constraint
W_ELB_TRAJ = 1.0   # nominal-elbow row: the authored stroke channel (consistent with W_BZ by build)
W_ELB_KF = 30.0    # keyframe mode: PIN the elbow, read the resulting geometry off the solution

IDX = {n: i for i, n in enumerate(WB_JOINTS)}
ELB_L, ELB_R = IDX["left_elbow_joint"], IDX["right_elbow_joint"]
KNEE_L, KNEE_R = IDX["left_knee_joint"], IDX["right_knee_joint"]
W_KNEE = 0.5   # leg-extension nominal: without it the solve parks the pelvis close to the toes and
               # FOLDS the legs (knee 1.44 at LOW) -- and the knee VERTEX then dips BELOW the floor,
               # which no contact gate can see (the knee is not a contact). The row makes the solver
               # stretch the plank (slide the pelvis aft, base x is free) instead of folding.


def build_spec(toe_pitch, toe_pitch_top=None, toe_pitch_low=None):
    """The push-up contact set. Every number is measured (see the header, facts 1 and 4).

    `toe_pitch` here is the MID of the rock (see THE FOOT ROCKS below): the spec's corner frame is
    fixed, the ghost's foot pitch rides +/- half the rock around it, and the corner-height error is
    half * sin(rock/2) < 1 mm -- far inside swing_tol. The patch CENTRE is the physical edge, which
    stays exactly on the ground at every rock angle, so [CLOSURE] and hover are exact."""
    c, s = math.cos(toe_pitch), math.sin(toe_pitch)
    return {
        "mu": 1.0,
        "swing_tol": 0.012,
        "terrain": {"type": "flat", "z": 0.0},
        "toe_pitch": float(toe_pitch),
        "toe_pitch_top": (None if toe_pitch_top is None else float(toe_pitch_top)),
        "toe_pitch_low": (None if toe_pitch_low is None else float(toe_pitch_low)),
        "contacts": [
            {"name": "hand_L", "body": "left_wrist_roll_rubber_hand",
             "half": [0.06, 0.03], "patch": [0.1267, -0.0062, -0.0418]},
            {"name": "hand_R", "body": "right_wrist_roll_rubber_hand",
             "half": [0.06, 0.03], "patch": [0.1267, +0.0062, -0.0418]},
            {"name": "toe_L", "body": "left_ankle_roll_link",
             "half": [0.004, 0.03], "patch": [0.120, 0.0, -0.036],
             "axes": [[c, 0.0, s], [0.0, 1.0, 0.0]]},
            {"name": "toe_R", "body": "right_ankle_roll_link",
             "half": [0.004, 0.03], "patch": [0.120, 0.0, -0.036],
             "axes": [[c, 0.0, s], [0.0, 1.0, 0.0]]},
        ],
    }


def build_stroke(reps, top_s, down_s, low_s, up_s):
    """Per-frame stroke phase s in [0, 1] (0 = plank TOP, 1 = LOW), C2 by quintic ramps, plus phase
    tags. One authored channel; base z, nominal pitch, nominal elbow and the COM target all ride it,
    so they are consistent by construction (the stair lesson: never let two channels disagree about
    when the body moves)."""
    phases = [("top", top_s, 0.0, 0.0)]
    for _ in range(reps):
        phases += [("down", down_s, 0.0, 1.0), ("low", low_s, 1.0, 1.0),
                   ("up", up_s, 1.0, 0.0), ("top", top_s, 0.0, 0.0)]
    svals, tags = [], []
    for name, dur, a, b in phases:
        nfr = int(round(dur / DT))
        for j in range(nfr):
            u = (j + 0.5) / nfr
            svals.append(a + (b - a) * GS.quintic(u))
            tags.append(name)
    return np.array(svals), tags


def prep_contacts(m, spec):
    """Origin: the spec-loading loop in ghost_synth.solve. Generalized: path_joints pulls the ARM
    chain for hand bodies automatically (E5); waist_yaw is filtered (sagittal plan, waist stays 0);
    each contact carries its patch-plane NORMAL from the spec axes (E2) so one orientation residual
    serves soles, palms and pitched toe strips alike."""
    for c in spec["contacts"]:
        c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
        if c["bid"] < 0:
            raise SystemExit("ghost_synth_pushup: contact body not in model: %s" % c["body"])
        jids = [j for j in GCL.path_joints(m, c["bid"])
                if mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) in WB_JOINTS
                and mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) != "waist_yaw_joint"]
        c["jwb"] = [IDX[mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j)] for j in jids]
        c["jdof"] = [m.jnt_dofadr[j] for j in jids]
        c["jlo"] = np.array([m.jnt_range[j][0] for j in jids])
        c["jhi"] = np.array([m.jnt_range[j][1] for j in jids])
        ax = c.get("axes")
        c["npatch"] = (np.cross(np.asarray(ax[0], float), np.asarray(ax[1], float))
                       if ax else np.array([0.0, 0.0, 1.0]))
        c["is_hand"] = "hand" in c["name"]


# branch-defining joints: the ones whose sign/region picks the plank family vs the contorted sprawl
# the first build fell into (shoulder_pitch -2.46 + yaw 1.2 + wrist 1.1 closes the same contacts).
PRIOR_JOINTS = [IDX[n] for n in (
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_wrist_roll_joint")]


def solve_frames(m, d, spec, tgts, com_tg, bz, pnom, enom, seed_p, seed_q,
                 w_elb, drop_hand_x=False, iters=120, use_com=True,
                 w_bz=W_BZ, w_att=W_ATT, qprior=None, w_prior=0.0, knee_nom=0.25, toe_nom=None):
    """The LM solve -- copied from ghost_synth.solve (origin: projects/policies/training/
    ghost_synth.py) with the E4/E5 extensions marked [PUSHUP-*]. Everything else, including every
    warning comment that still applies, is the origin's; if you fix a bug here, fix it there too.

    Unknowns per frame: [base x, y, z, PITCH] + 12 leg + 10 arm joints (26).   [PUSHUP-E4/E5]
    Rows: palm pos(3)+normal(2) per hand (NO yaw row -- palm yaw about the normal is free, pinning
    it saturates wrist_roll: the ankle_roll lesson transplanted); toe pos(3)+normal(2)+yaw(1) per
    foot; COM xy (2); base-z profile (1); nominal pitch (1); nominal elbow (2); W_REG posture rows.
    """
    nb = len(bz)
    knee_nom = np.full(nb, float(knee_nom)) if np.isscalar(knee_nom) else np.asarray(knee_nom, float)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
    cols = [0, 1, 2, 4] + sorted({dd for c in spec["contacts"] for dd in c["jdof"]})
    # [PUSHUP-E4] base rotational dof 4 = the free joint's y rotation. Pure-pitch parameterization
    # keeps body y == world y, so this Jacobian column is exact for d/d(pitch).
    PCOL = 3                                    # index of the pitch column in `cols`
    solwb = sorted({j for c in spec["contacts"] for j in c["jwb"]})

    wb = np.zeros((nb, len(WB_JOINTS)))
    root = np.zeros((nb, 4))
    pitch = np.zeros(nb)
    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv)); jcom = np.zeros((3, m.nv))
    q = np.asarray(seed_q, float).copy()
    p = np.asarray(seed_p, float).copy()        # [x, y, z, pitch]
    ferr, cerr, unreach = [], [], []

    def _fk(pp, qq):
        d.qpos[0:3] = pp[0:3]
        d.qpos[3:7] = [math.cos(pp[3] / 2), 0.0, math.sin(pp[3] / 2), 0.0]   # [PUSHUP-E4]
        for k2, n in enumerate(WB_JOINTS):
            d.qpos[qadr[n]] = qq[k2]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)

    def _res(i, pp, qq, qprev, want_J):
        _fk(pp, qq)
        rows, res = [], []
        for c in spec["contacts"]:
            R = d.xmat[c["bid"]].reshape(3, 3)
            sp = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
            # THE FOOT ROCKS ON ITS TOE EDGE: the toe orientation target rides the stroke
            # (toe_nom, ~77 deg at TOP -> ~60 deg heel-drop at LOW). Without it the LOW frame is
            # over-constrained: the shoulder slides 0.16 m aft with the elbow bend, and legs pinned
            # to a FIXED foot pitch can only absorb that by folding the knee THROUGH THE FLOOR
            # (measured: knee vertex -40 mm) or piking. The edge (= patch centre) never moves.
            if c["is_hand"] or toe_nom is None:
                npatch = c["npatch"]
            else:
                npatch = np.array([-math.sin(toe_nom[i]), 0.0, math.cos(toe_nom[i])])
            nnow = R @ npatch
            perr = tgts[c["name"]][i] - sp
            psel = [1, 2] if (drop_hand_x and c["is_hand"]) else [0, 1, 2]
            res.append(W_CON * perr[psel])
            # patch-plane tangency: drive R@npatch onto the world normal (+z on flat ground). For a
            # sole/palm npatch is body +z and this IS the origin's _rot_err_xy; for the pitched toe
            # strip it is the tilted spec normal. Small-angle rows = jacr, exactly as the origin.
            e2 = np.cross(nnow, np.array([0.0, 0.0, 1.0]))[0:2]
            res.append(W_ORI * e2)
            if not c["is_hand"]:
                res.append(W_ORI * np.array([-math.atan2(R[1, 0], R[0, 0])]))   # heading, feet only
            if want_J:
                mj.mj_jac(m, d, jacp, jacr, sp, c["bid"])
                rows.append(W_CON * jacp[psel][:, cols])
                rows.append(W_ORI * jacr[0:2][:, cols])
                if not c["is_hand"]:
                    rows.append(W_ORI * jacr[2:3][:, cols])
        if use_com:
            res.append(W_COM * (com_tg[i] - d.subtree_com[0][0:2]))
        res.append(np.array([w_bz * (bz[i] - pp[2])]))
        res.append(np.array([w_att * (pnom[i] - pp[3])]))                       # [PUSHUP-E4]
        res.append(w_elb * (enom[i] - qq[[ELB_L, ELB_R]]))                      # the STROKE channel
        res.append(W_KNEE * (knee_nom[i] - qq[[KNEE_L, KNEE_R]]))               # leg-shape nominal
        res.append(W_REG * (qprev[solwb] - qq[solwb]))
        if w_prior > 0.0 and qprior is not None:
            res.append(w_prior * (qprior[i][PRIOR_JOINTS] - qq[PRIOR_JOINTS]))
        if want_J:
            if use_com:
                mj.mj_jacSubtreeCom(m, d, jcom, 0)
                rows.append(W_COM * jcom[0:2][:, cols])
            rz = np.zeros((1, len(cols))); rz[0, 2] = w_bz
            rows.append(rz)
            rp = np.zeros((1, len(cols))); rp[0, PCOL] = w_att
            rows.append(rp)
            re = np.zeros((2, len(cols)))
            re[0, cols.index(next(dd for c in spec["contacts"] if c["name"] == "hand_L"
                                  for j, dd in zip(c["jwb"], c["jdof"]) if j == ELB_L))] = w_elb
            re[1, cols.index(next(dd for c in spec["contacts"] if c["name"] == "hand_R"
                                  for j, dd in zip(c["jwb"], c["jdof"]) if j == ELB_R))] = w_elb
            rows.append(re)
            rk = np.zeros((2, len(cols)))
            rk[0, cols.index(next(dd for c in spec["contacts"] if c["name"] == "toe_L"
                                  for j, dd in zip(c["jwb"], c["jdof"]) if j == KNEE_L))] = W_KNEE
            rk[1, cols.index(next(dd for c in spec["contacts"] if c["name"] == "toe_R"
                                  for j, dd in zip(c["jwb"], c["jdof"]) if j == KNEE_R))] = W_KNEE
            rows.append(rk)
            rr = np.zeros((len(solwb), len(cols)))
            for k2, jw in enumerate(solwb):
                dd = next(dd for c in spec["contacts"] for j, dd in zip(c["jwb"], c["jdof"]) if j == jw)
                rr[k2, cols.index(dd)] = W_REG
            rows.append(rr)
            if w_prior > 0.0 and qprior is not None:
                rq = np.zeros((len(PRIOR_JOINTS), len(cols)))
                for k2, jw in enumerate(PRIOR_JOINTS):
                    dd = next(dd for c in spec["contacts"] for j, dd in zip(c["jwb"], c["jdof"]) if j == jw)
                    rq[k2, cols.index(dd)] = w_prior
                rows.append(rq)
            return np.concatenate(res), np.vstack(rows)
        return np.concatenate(res), None

    def _apply(pp, qq, dq):
        p2 = pp.copy(); p2[0:3] += dq[0:3]; p2[3] += dq[PCOL]
        q2 = qq.copy()
        for c in spec["contacts"]:
            sel = [cols.index(dd) for dd in c["jdof"]]
            q2[c["jwb"]] = np.clip(qq[c["jwb"]] + dq[sel], c["jlo"], c["jhi"])
        return p2, q2

    for i in range(nb):
        qprev = q.copy()
        lam = 1e-2
        r, J = _res(i, p, q, qprev, True)
        cost = float(r @ r)
        for _ in range(iters):
            if cost < 1e-14:
                break
            dq = np.linalg.solve(J.T @ J + lam * np.eye(J.shape[1]), J.T @ r)
            nrm_dq = float(np.linalg.norm(dq))
            if nrm_dq > 0.20:                     # origin: trust region, never leap the workspace
                dq *= 0.20 / nrm_dq
            p2, q2 = _apply(p, q, dq)
            r2, _ = _res(i, p2, q2, qprev, False)
            c2 = float(r2 @ r2)
            if c2 < cost:                          # accept, and trust the model more
                p, q, cost = p2, q2, c2
                lam = max(lam * 0.6, 1e-8)
                r, J = _res(i, p, q, qprev, True)
            else:                                  # reject, and trust it less
                lam *= 3.0
                if lam > 1e6:
                    break
        _fk(p, q)
        e = 0.0
        for c in spec["contacts"]:
            R = d.xmat[c["bid"]].reshape(3, 3)
            sp = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
            pe = tgts[c["name"]][i] - sp
            if drop_hand_x and c["is_hand"]:
                pe = pe[1:3]
            e = max(e, float(np.linalg.norm(pe)))
        ferr.append(e)
        # origin: GATE 2's OWN RESIDUAL -- the COM row is the lowest-weight task and pays first
        # whenever a joint limit makes the system inconsistent. Report it, never assume it.
        cerr.append(float(np.linalg.norm(d.subtree_com[0][0:2] - com_tg[i])) if use_com else 0.0)
        if e > 2e-3:
            unreach.append((i, e))
        wb[i] = q
        root[i] = [p[0], p[1], p[2], 0.0]
        pitch[i] = p[3]
    return wb, root, pitch, np.array(ferr), np.array(cerr), unreach, p, q


def solve_keyframe(m, d, spec, hand_t, toe_t, com_des, com_pull, e_pin, pitch_nom, z_est, seeds,
                   drop_hand_x, use_com, knee_nom=0.25, toe_nom=None):
    """One posture, solved as a 1-frame sequence with the elbow PINNED. TWO STAGES:

      A  the trunk is held near the measured plank family (strong z/pitch priors on the ANALYTIC
         estimates, posture prior on the branch-defining joints) so the LM cannot leave the branch
         while it closes the contacts;
      B  released (w_bz = 0: the keyframe's base z is an OUTPUT, exactly the doctrine; weak pitch
         nominal; token posture prior) and re-converged from A.

    Multi-seeded: keyframes have no warm start, and the origin's crouch-seed lesson (a straight
    limb is a singular Jacobian) applies to knees AND elbows here. The first build skipped stage A
    and closed the contacts perfectly in a CONTORTED branch (shoulder_pitch -2.46, yaw +1.2, knees
    folded 1.0, ankle_pitch on its stop) -- closure alone cannot tell the branches apart.

    THE COM TARGET IS CLAMPED TO THE REACHABLE (measured on build 2: asking for 9 cm of COM shift
    the geometry cannot give turns W_COM into a permanent pull that drags the trunk out of the
    branch -- the row must nudge, never fight). Stage A runs WITHOUT the COM row and measures the
    natural COM; stage B asks for at most `com_pull` of pike-bias toward the toes."""
    tg = {k: np.array([v]) for k, v in {**hand_t, **toe_t}.items()}
    best = None
    for p0, q0 in seeds:
        pA, qA = p0.copy(), q0.copy()
        com_t = np.zeros(2)
        # stage B keeps WEAK z/pitch nominals (0.3/0.5): with w_bz = 0 the leg-extension row pikes
        # the pelvis into a downward-dog (measured: pitch 120 deg, base z 0.302) -- the fold-vs-pike
        # trade is arbitrated exactly here, and the analytic z_est is the right arbiter.
        for stage, (wbz, watt, wpr, its, com_on) in enumerate((
                (3.0, 3.0, 0.5, 500, False), (0.3, 0.5, 0.05, 500, use_com))):
            wb, root, pit, fe, ce, un, pA, qA = solve_frames(
                m, d, spec, tg, np.array([com_t]), np.array([z_est]), np.array([pitch_nom]),
                np.array([[e_pin, e_pin]]), pA, qA, w_elb=W_ELB_KF,
                drop_hand_x=drop_hand_x, iters=its, use_com=com_on,
                w_bz=wbz, w_att=watt, qprior=np.array([q0]), w_prior=wpr, knee_nom=knee_nom,
                toe_nom=(None if toe_nom is None else np.array([toe_nom])))
            if stage == 0 and use_com:
                # natural COM after stage A; clamp the bias to what a mild pike can deliver
                com_nat = _fk_com(m, d, wb[0], root[0], pit[0])
                com_t = np.array([max(com_des, com_nat[0] - com_pull), 0.0])
        score = fe[0] + 0.1 * ce[0]
        if best is None or score < best[0]:
            best = (score, wb, root, pit, fe, ce, pA, qA, com_t)
    _, wb, root, pit, fe, ce, p, q, com_t = best
    return wb[0], root[0], pit[0], fe[0], ce[0], p, q, com_t


def _fk_com(m, d, wb, root, pitch):
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
    d.qpos[0:3] = root[0:3]
    d.qpos[3:7] = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    for k, n in enumerate(WB_JOINTS):
        d.qpos[qadr[n]] = wb[k]
    mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
    return d.subtree_com[0].copy()


# measured segment constants (probe 2026-07-09; used ONLY to seed/estimate, never emitted):
#   wrist origin sits 0.0418 above its palm face; elbow->wrist z offset ~0.010 -> forearm height
#   ~0.052 when the palm is flat; |shoulder->elbow| = 0.193; pelvis->shoulder along body z = 0.2918.
ELBOW_Z, UPPER_ARM, TORSO_Z = 0.052, 0.193, 0.2918


def z_estimate(elbow, pitch):
    """Analytic pelvis-z of the flat-palm family: shoulder rides 0.193*cos(e) above the horizontal
    forearm; the pelvis hangs TORSO_Z*cos(p) below the shoulder line. Stage-A prior only."""
    return ELBOW_Z + UPPER_ARM * math.cos(elbow) - TORSO_Z * math.cos(pitch)


def family_seed(pitch, elbow, base_xz, foot_y):
    """The measured flat-palm plank family (header fact 2/3): shoulder_pitch = -(pitch + elbow),
    everything else sagittal. Legs near-straight with a small crouch-analog bend (origin lesson:
    seed away from the extension singularity)."""
    q = np.zeros(len(WB_JOINTS))
    for h, k, an in ((0, 3, 4), (6, 9, 10)):
        q[h], q[k], q[an] = 0.10, 0.30, -0.45
    for sp, sr, el in ((13, 14, 16), (18, 19, 21)):
        q[sp] = -(pitch + elbow)
        q[sr] = 0.10 if sp == 13 else -0.10
        q[el] = elbow
    return np.array([base_xz[0], 0.0, base_xz[1], pitch]), q


def filmstrip(path, m, d, spec, wb, root, pitch, tags, com_frames):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
    bods = ["pelvis", "torso_link",
            "left_shoulder_pitch_link", "left_elbow_link", "left_wrist_roll_rubber_hand",
            "left_hip_pitch_link", "left_knee_link", "left_ankle_roll_link"]
    bid = {n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n) for n in bods}
    # 8 frames: first-top mid, descend 30/70%, low mid, press 30/70%, second-top mid, last
    nb = len(wb)
    def _phase_frames(name, k):
        idx = [i for i in range(nb) if tags[i] == name]
        runs = []
        for i in idx:
            if runs and i == runs[-1][-1] + 1:
                runs[-1].append(i)
            else:
                runs.append([i])
        return runs[k] if k < len(runs) else runs[-1]
    sel = []
    t0 = _phase_frames("top", 0); sel.append(t0[len(t0) // 2])
    dn = _phase_frames("down", 0); sel += [dn[int(0.3 * len(dn))], dn[int(0.7 * len(dn))]]
    lo = _phase_frames("low", 0); sel.append(lo[len(lo) // 2])
    up = _phase_frames("up", 0); sel += [up[int(0.3 * len(up))], up[int(0.7 * len(up))]]
    t1 = _phase_frames("top", 1); sel.append(t1[len(t1) // 2])
    sel.append(nb - 1)

    fig, axes = plt.subplots(2, 4, figsize=(18, 7), sharex=True, sharey=True)
    for ax, i in zip(axes.ravel(), sel):
        d.qpos[0:3] = root[i, 0:3]
        d.qpos[3:7] = [math.cos(pitch[i] / 2), 0, math.sin(pitch[i] / 2), 0]
        for k, n in enumerate(WB_JOINTS):
            d.qpos[qadr[n]] = wb[i, k]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
        P = {n: d.xpos[bid[n]].copy() for n in bods}
        pat = {}
        for c in spec["contacts"]:
            R = d.xmat[c["bid"]].reshape(3, 3)
            ctr = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
            ax0 = np.asarray(c.get("axes", [[1, 0, 0], [0, 1, 0]])[0], float)
            u = R @ ax0 * c["half"][0]
            pat[c["name"]] = (ctr - u, ctr + u, ctr)
        com = d.subtree_com[0].copy()
        ax.axhline(0, color="0.4", lw=1)
        chain = [("pelvis", "torso_link"), ("torso_link", "left_shoulder_pitch_link"),
                 ("left_shoulder_pitch_link", "left_elbow_link"),
                 ("left_elbow_link", "left_wrist_roll_rubber_hand"),
                 ("pelvis", "left_hip_pitch_link"), ("left_hip_pitch_link", "left_knee_link"),
                 ("left_knee_link", "left_ankle_roll_link")]
        for a, b in chain:
            ax.plot([P[a][0], P[b][0]], [P[a][2], P[b][2]], "-o", color="tab:blue", ms=3, lw=2)
        ax.plot(*zip(*[(pat["hand_L"][0][0], pat["hand_L"][0][2]),
                       (pat["hand_L"][1][0], pat["hand_L"][1][2])]), "-", color="tab:red", lw=4)
        ax.plot(*zip(*[(pat["toe_L"][0][0], pat["toe_L"][0][2]),
                       (pat["toe_L"][1][0], pat["toe_L"][1][2])]), "-", color="tab:orange", lw=4)
        # wrist -> palm centre (the hand itself)
        ax.plot([P["left_wrist_roll_rubber_hand"][0], pat["hand_L"][2][0]],
                [P["left_wrist_roll_rubber_hand"][2], pat["hand_L"][2][2]], "-", color="tab:red", lw=2)
        ax.plot([com[0]], [com[2]], "k*", ms=10)
        ax.plot([com[0], com[0]], [0, com[2]], "k:", lw=1)
        ax.set_title("f%d %s  t=%.2fs  elb %.2f  pitch %.0fdeg"
                     % (i, tags[i], i * DT, wb[i, ELB_L], math.degrees(pitch[i])))
        ax.set_aspect("equal")
        ax.set_ylim(-0.05, 0.45)
    fig.suptitle("G1 push-up ghost (side view: left chain, palm/toe patches, COM * + projection)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("  filmstrip -> %s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--top-s", type=float, default=1.0, help="plank TOP hold (s)")
    ap.add_argument("--down-s", type=float, default=1.5, help="descend (s) -- slow: LP-vs-controller")
    ap.add_argument("--low-s", type=float, default=0.8, help="LOW hold (s)")
    ap.add_argument("--up-s", type=float, default=1.5, help="press up (s)")
    ap.add_argument("--e-top", type=float, default=0.10,
                    help="elbow at plank TOP. 0 = 90deg bent = upper arm VERTICAL = the tallest "
                         "flat-palm posture (header fact 3); near-straight (1.45) cannot plant a palm")
    ap.add_argument("--e-low", type=float, default=0.85,
                    help="elbow at the bottom of the stroke. Deeper (1.0) is reachable but the "
                         "extra aft-slide of the shoulder has to come out of knee fold, and the "
                         "knee pad hits the floor before the elbow hits its range.")
    ap.add_argument("--toe-pitch", type=float, default=1.35,
                    help="foot pitch at the plank TOP (rad, sole vs ground); ~77 deg is the "
                         "near-natural plank value, ankle_pitch absorbs the residual")
    ap.add_argument("--toe-pitch-low", type=float, default=1.05,
                    help="foot pitch at the LOW (rad): the foot ROCKS on its planted toe edge, "
                         "heels dropping aft as the shoulder slides aft -- the missing dof that "
                         "lets the legs close the LOW frame without folding the knee into the floor")
    ap.add_argument("--toe-x", type=float, default=0.0, help="world x of the toe edges")
    ap.add_argument("--foot-y", type=float, default=GS.STANCE_Y)
    ap.add_argument("--hand-y", type=float, default=0.13,
                    help="palm centre |y|; ~0.136 is the natural under-shoulder value (the elbow "
                         "sits 4.7 cm outboard of the shoulder at roll 0 -- measured)")
    ap.add_argument("--hand-dx", type=float, default=0.0,
                    help="shift the frozen handholds (m); + moves them ahead of natural reach")
    ap.add_argument("--com-frac", type=float, default=0.44,
                    help="COM target as a fraction of toe->palm distance; <0.5 biases load to the "
                         "LEGS (relieves the 25 N*m arm actuators)")
    ap.add_argument("--com-pull", type=float, default=0.035,
                    help="max COM bias (m) the row may ask for beyond the natural posture's COM -- "
                         "an unreachable COM target is a permanent pull that drags the solve out of "
                         "the plank branch (measured on build 2)")
    ap.add_argument("--pitch0", type=float, default=1.35, help="nominal base pitch seed (rad)")
    ap.add_argument("--knee-top", type=float, default=0.30,
                    help="knee nominal at the plank TOP (rad). MUST be near-straight: in plank the "
                         "knee vertex points DOWN, and a buckled TOP (0.78) parks the knee origin "
                         "0.02 m up with a 0.064 m pad below it. 0.30 stretches the plank (KF0 "
                         "slides the pelvis aft until the legs nearly extend; vertex ~+0.11).")
    ap.add_argument("--knee-low", type=float, default=0.85,
                    help="knee nominal at the LOW (rad). The LOW frame is a three-way trade: FOLD "
                         "(knee 1.44 -> pad 40+ mm under the floor), PIKE (knee 0.25 -> pitch 120 "
                         "deg downward-dog), or this balanced point (moderate fold + slight pike). "
                         "The knee/pitch/z nominal rows arbitrate it.")
    ap.add_argument("--pitch-low", type=float, default=1.70,
                    help="nominal base pitch at the LOW (rad, ~97 deg: the torso dives a little "
                         "past vertical so the pelvis does not have to slide aft with the shoulder)")
    ap.add_argument("--mjcf", default=None)
    ap.add_argument("--contacts-out", default=DEF_SPEC_OUT,
                    help="the contact spec is (re)written here so lut and spec can never drift")
    ap.add_argument("--filmstrip", default=None, help="write an 8-frame side-view PNG")
    a = ap.parse_args()

    toe_mid = 0.5 * (a.toe_pitch + a.toe_pitch_low)
    spec = build_spec(toe_mid, a.toe_pitch, a.toe_pitch_low)
    os.makedirs(os.path.dirname(a.contacts_out), exist_ok=True)
    json.dump({k: v for k, v in spec.items()}, open(a.contacts_out, "w"), indent=1)
    print("GHOST SYNTH PUSH-UP  (hands + toes, solved base pitch, solved arm chains)")
    print("  contact spec -> %s  (toe frame at the mid-rock %.0f deg; foot rocks %.0f..%.0f deg)"
          % (a.contacts_out, math.degrees(toe_mid), math.degrees(a.toe_pitch),
             math.degrees(a.toe_pitch_low)))

    m = mj.MjModel.from_xml_path(a.mjcf or GS.DEF_MJCF)
    d = mj.MjData(m)
    prep_contacts(m, spec)

    toe_t = {"toe_L": np.array([a.toe_x, +a.foot_y, 0.0]),
             "toe_R": np.array([a.toe_x, -a.foot_y, 0.0])}

    # ---- KF0: TOP posture, hand-x row dropped -> the hands find natural reach; FREEZE handholds --
    z_top_est = z_estimate(a.e_top, a.pitch0)
    seeds = [(family_seed(a.pitch0, a.e_top, (a.toe_x + bx, z_top_est), a.foot_y)) for bx in
             (0.74, 0.70, 0.78)]
    hand_guess = {"hand_L": np.array([a.toe_x + 1.25, +a.hand_y, 0.0]),
                  "hand_R": np.array([a.toe_x + 1.25, -a.hand_y, 0.0])}
    wb0, r0, p0, fe0, _, pf, qf, _ = solve_keyframe(m, d, spec, hand_guess, toe_t, 0.0, 0.0,
                                                    a.e_top, a.pitch0, z_top_est, seeds,
                                                    drop_hand_x=True, use_com=False,
                                                    knee_nom=a.knee_top, toe_nom=a.toe_pitch)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
    d.qpos[0:3] = r0[0:3]; d.qpos[3:7] = [math.cos(p0 / 2), 0, math.sin(p0 / 2), 0]
    for k, n in enumerate(WB_JOINTS):
        d.qpos[qadr[n]] = wb0[k]
    mj.mj_kinematics(m, d)
    hx = float(np.mean([(d.xpos[c["bid"]] + d.xmat[c["bid"]].reshape(3, 3)
                         @ np.asarray(c["patch"], float))[0]
                        for c in spec["contacts"] if c["is_hand"]])) + a.hand_dx
    hand_t = {"hand_L": np.array([hx, +a.hand_y, 0.0]), "hand_R": np.array([hx, -a.hand_y, 0.0])}
    print("  KF0 (natural TOP, hand-x free): residual %.2f mm -> handholds FROZEN at x=%.3f (y +/-%.3f)"
          % (1000 * fe0, hx, a.hand_y))

    # ---- KF TOP / KF LOW: hands pinned, elbow pinned, COM biased --------------------------------
    com_des = a.toe_x + a.com_frac * (hx - a.toe_x)
    kf = {}
    # THREE keyframes: top, MID, low. With only top/low, the linearly-interpolated base/pitch
    # profiles are infeasible halfway through the stroke and the KNEE pays the difference
    # (measured: knee-origin z 0.017-0.019 mid-stroke = pad 5 cm into the floor). The mid keyframe
    # lets the base path bow to what the geometry actually wants; profiles are quadratic-Lagrange
    # through the three measured points (still C2 in time: they are smooth functions of the C2 s(t)).
    for tag, epin, pnm, tnm, knm, sds in (
            ("top", a.e_top, a.pitch0, a.toe_pitch, a.knee_top, [(pf, qf)]),
            ("mid", 0.5 * (a.e_top + a.e_low), 0.5 * (a.pitch0 + a.pitch_low),
             0.5 * (a.toe_pitch + a.toe_pitch_low), 0.5 * (a.knee_top + a.knee_low),
             [(pf.copy(), qf.copy())]),
            ("low", a.e_low, a.pitch_low, a.toe_pitch_low, a.knee_low, [(pf.copy(), qf.copy())])):
        wbk, rk, pk, fek, cek, pf2, qf2, com_used = solve_keyframe(
            m, d, spec, hand_t, toe_t, com_des, a.com_pull, epin, pnm,
            z_estimate(epin, pnm), sds, drop_hand_x=False, use_com=True, knee_nom=knm,
            toe_nom=tnm)
        d.qpos[0:3] = rk[0:3]; d.qpos[3:7] = [math.cos(pk / 2), 0, math.sin(pk / 2), 0]
        for k, n in enumerate(WB_JOINTS):
            d.qpos[qadr[n]] = wbk[k]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
        com_ach = d.subtree_com[0][0:2].copy()
        sh_z = 0.5 * (d.xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "left_shoulder_pitch_link")][2]
                      + d.xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "right_shoulder_pitch_link")][2])
        kf[tag] = dict(z=rk[2], x=rk[0], pitch=pk, com=com_ach, ferr=fek, cerr=cek, shz=sh_z,
                       seed=(pf2, qf2), wb=wbk)
        print("  KF %-3s (elbow %.2f): base z %.3f  pitch %.1f deg  shoulder z %.3f  residual %.2f mm"
              % (tag.upper(), epin, rk[2], math.degrees(pk), sh_z, 1000 * fek))
        print("        COM x %.3f (designed %.3f, clamped target %.3f; frac %.2f -> achieved %.2f)"
              "  COM-row residual %.1f mm"
              % (com_ach[0], com_des, com_used[0], a.com_frac,
                 (com_ach[0] - a.toe_x) / (hx - a.toe_x), 1000 * cek))

    # ---- profiles ride the stroke; targets through the ACHIEVED keyframe values -----------------
    s, tags = build_stroke(a.reps, a.top_s, a.down_s, a.low_s, a.up_s)
    nb = len(s)
    # quadratic Lagrange through the three measured keyframes (nodes s = 0, 0.5, 1)
    L0 = 2.0 * (s - 0.5) * (s - 1.0)
    Lm = -4.0 * s * (s - 1.0)
    L1 = 2.0 * s * (s - 0.5)

    def q3(key, sub=None):
        v = [kf[t][key] if sub is None else kf[t][key][sub] for t in ("top", "mid", "low")]
        return v[0] * L0 + v[1] * Lm + v[2] * L1

    bz = q3("z")
    pnom = q3("pitch")
    enom_v = a.e_top + (a.e_low - a.e_top) * s
    enom = np.stack([enom_v, enom_v], axis=1)
    com_tg = np.stack([q3("com", 0), np.zeros(nb)], axis=1)
    tgts = {k: np.tile(v, (nb, 1)) for k, v in {**hand_t, **toe_t}.items()}
    # posture prior rides the stroke through the three MEASURED keyframe postures -- a branch
    # anchor (weight 0.05, two decades below the contact rows), not a constraint.
    qprior = (kf["top"]["wb"][None, :] * L0[:, None] + kf["mid"]["wb"][None, :] * Lm[:, None]
              + kf["low"]["wb"][None, :] * L1[:, None])

    wb, root, pitch, fe, ce, unreach, _, _ = solve_frames(
        m, d, spec, tgts, com_tg, bz, pnom, enom, kf["top"]["seed"][0], kf["top"]["seed"][1],
        w_elb=W_ELB_TRAJ, iters=120, qprior=qprior, w_prior=0.05,
        knee_nom=(a.knee_top + (a.knee_low - a.knee_top) * s),
        toe_nom=(a.toe_pitch + (a.toe_pitch_low - a.toe_pitch) * s))

    dur = nb * DT
    print("  TRAJ  nb=%d (%.2f s): %d reps, stroke base z %.3f..%.3f, pitch %.1f..%.1f deg"
          % (nb, dur, a.reps, bz.min(), bz.max(), math.degrees(pnom.min()), math.degrees(pnom.max())))
    print("  contact-target residual  mean %.3f mm  max %.3f mm   unreachable frames %d/%d"
          % (1000 * fe.mean(), 1000 * fe.max(), len(unreach), nb))
    print("  COM-to-target (gate 2)   mean %.2f mm  max %.2f mm %s"
          % (1000 * ce.mean(), 1000 * ce.max(),
             "<-- the COM row lost to a joint limit" if ce.max() > 5e-3 else ""))
    if unreach:
        i, e = max(unreach, key=lambda t: t[1])
        print("  WORST frame %d short by %.1f mm -- a handhold/foothold the chain cannot reach with the"
              % (i, 1000 * e))
        print("  base where the profiles put it. Retune --hand-dx / --e-low; do not ignore this.")
    elb = wb[:, ELB_L]
    print("  ELBOW stroke achieved %.2f..%.2f rad (%.0f..%.0f deg geometric bend-from-straight: "
          "straight=1.45)" % (elb.min(), elb.max(),
                              math.degrees(1.45 - elb.max()), math.degrees(1.45 - elb.min())))
    print("  SMOOTH worst one-bin joint step %.4f rad" % np.abs(np.diff(wb, axis=0)).max())
    ap_i = IDX["left_ankle_pitch_joint"]
    print("  ANKLE_PITCH range used %.3f..%.3f (limit -0.873..+0.524; the heel-drop rock spends it)"
          % (wb[:, ap_i].min(), wb[:, ap_i].max()))
    # BODY CLEARANCE witness: the gates only see declared contacts; a folded knee or a dipped torso
    # can pass every gate while poking through the floor (caught on build 3's filmstrip: knee -40 mm).
    # pad = how far the body's crawl-URDF collision box extends below its origin in this posture
    # class (knee front face 0.064; elbow box bottom 0.046 -- the horizontal forearm INHERENTLY
    # skims ~5 mm over the floor in the flat-palm family; hip/pelvis/torso boxes ~0.06-0.08).
    clr = {"left_knee_link": 0.064, "right_knee_link": 0.064,
           "left_elbow_link": 0.046, "right_elbow_link": 0.046,
           "torso_link": 0.075, "pelvis": 0.060,
           "left_hip_pitch_link": 0.060, "right_hip_pitch_link": 0.060}
    cb = {n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n) for n in clr}
    zmin = {n: (1e9, -1) for n in clr}
    for i in range(0, nb, 2):
        d.qpos[0:3] = root[i, 0:3]
        d.qpos[3:7] = [math.cos(pitch[i] / 2), 0, math.sin(pitch[i] / 2), 0]
        for k, n in enumerate(WB_JOINTS):
            d.qpos[qadr[n]] = wb[i, k]
        mj.mj_kinematics(m, d)
        for n, b in cb.items():
            z = float(d.xpos[b][2])
            if z < zmin[n][0]:
                zmin[n] = (z, i)
    bad = False
    for n in sorted(clr, key=lambda n: zmin[n][0] - clr[n]):
        z, i = zmin[n]
        marg = z - clr[n]
        if marg < 0.02:
            bad = bad or (marg < 0.0)
            print("  CLEARANCE %-22s origin z %.3f - pad %.3f = %+.3f m margin (frame %d)%s"
                  % (n, z, clr[n], marg, i, "  <-- IN THE FLOOR" if marg < 0 else ""))
    if not bad:
        print("  CLEARANCE all non-contact pads above the floor (worst margin printed above if < 20 mm)")

    sched = [sorted(c["name"] for c in spec["contacts"])] * nb
    out = {
        "nb": nb, "freq": 1.0 / dur, "cycle_s": dur, "vx": 0.0, "seq": True, "hold_end": True,
        "arm_A": 0.0, "shroll": float(np.mean(wb[:, IDX["left_shoulder_roll_joint"]])),
        "leg_lut": [[float(v) for v in r[0:12]] for r in wb],
        "arm_lut": [[float(r[13]), float(r[18])] for r in wb],
        "elbow_lut": [[float(r[ELB_L]), float(r[ELB_R])] for r in wb],
        "att_lut": [[0.0, float(pv)] for pv in pitch],       # E4: solved pitch; ONE channel owns it
        "root_lut": [[float(v) for v in r] for r in root],
        "wb_joints": WB_JOINTS, "wb_lut": [[float(v) for v in r] for r in wb],
        "contact_schedule": sched, "terrain": spec["terrain"],
        "ghost_schema": 2, "ghost_closed": True,
        "pushup": {"phase_tags_rle": _rle(tags), "e_top": a.e_top, "e_low": a.e_low,
                   "toe_pitch": a.toe_pitch, "handhold_x": hx, "toe_x": a.toe_x,
                   "com_frac_designed": a.com_frac,
                   "kf": {t: {"z": float(kf[t]["z"]), "pitch": float(kf[t]["pitch"]),
                              "shoulder_z": float(kf[t]["shz"]), "com_x": float(kf[t]["com"][0])}
                          for t in ("top", "mid", "low")}},
        "source": ("SYNTHESIZED by ghost_synth_pushup.py (plan the contacts, solve everything else): "
                   "%d push-up reps on HANDS+TOES; palms flat at x=%.3f (frozen from the natural TOP "
                   "reach), toe edges tangent at %.0f deg at x=%.3f; per frame the base (x,y,z) + base "
                   "PITCH (E4) + 12 leg + 10 arm joints (E5, path_joints chains) are SOLVED so closure "
                   "holds and the COM rides a designed toe-biased point (frac %.2f of toe->palm); the "
                   "stroke is a C2 elbow/base-z profile between MEASURED keyframes (TOP elbow %.2f "
                   "base z %.3f pitch %.2f; LOW elbow %.2f base z %.3f pitch %.2f). NOTE the G1 has no "
                   "wrist pitch: a flat palm REQUIRES a horizontal forearm, so the TOP is elbow~%.2f "
                   "(0 = 90deg-bent convention), NOT a straight arm -- a straight vertical arm cannot "
                   "plant its palm. Starts AT the plank top: standing->plank is get-up's job, not this "
                   "ghost's. Contact residual max %.2f mm, COM residual max %.2f mm. Gates 1+2 by "
                   "construction; gate 3 is ghost_dynamics' to judge (spec: contacts_pushup.json)."
                   % (a.reps, hx, math.degrees(a.toe_pitch), a.toe_x, a.com_frac,
                      a.e_top, kf["top"]["z"], kf["top"]["pitch"],
                      a.e_low, kf["low"]["z"], kf["low"]["pitch"], a.e_top,
                      1000 * fe.max(), 1000 * ce.max())),
    }
    json.dump(out, open(a.out, "w"))
    print("  wrote %s" % a.out)
    print("  NEXT: build_keypoints.py %s --links ; ghost_dynamics.py %s --contacts %s --smooth 7 --stride 4"
          % (a.out, a.out, a.contacts_out))

    if a.filmstrip:
        filmstrip(a.filmstrip, m, d, spec, wb, root, pitch, tags, None)


def _rle(tags):
    out = []
    for t in tags:
        if out and out[-1][0] == t:
            out[-1][1] += 1
        else:
            out.append([t, 1])
    return out


if __name__ == "__main__":
    main()
