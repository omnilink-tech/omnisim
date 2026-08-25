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

"""ghost_synth_squat.py -- SYNTHESIZE a one-shot DEEP-SQUAT ghost (plan the contacts, solve the rest).

THE METHOD (ghost_synth.py, applied to the simplest possible contact plan)
Plan the CONTACTS. Solve everything else. For a squat the contact plan is trivial -- both soles planted
at the neutral stance (x = 0, y = +/-STANCE_Y) on EVERY frame, schedule = [foot_L, foot_R] throughout --
and everything interesting is in what gets SOLVED:

  * The base (x, y, z) is solved per frame together with the 12 leg joints (Levenberg-Marquardt on the
    real mujoco Jacobians, mj_jac + mj_jacSubtreeCom) so the COM rides the centroid of the two feet:
    x over the ANKLE line (the patch centre minus the 35 mm patch offset -- the 11.7 N*m lesson from
    ghost_synth.support_anchor), y = 0. Nothing is a formula: the pelvis drifts backward as the knees
    fold forward because the SOLVE says so, not because anyone authored a hip-shift profile.
  * base_z follows an explicit C2 PROFILE (quintic in, quintic out -- zero velocity AND acceleration at
    every phase boundary): settle at the ride height, descend, deep hold, ascend, stand hold. This is
    BaseZPolicy mode "profile" in the library-design doc's terms, implemented locally. The profile is a
    W_BZ=1 row against W_FOOT=10 / W_COM=2 rows, so when the requested depth exceeds what the joint
    limits allow, the SOLVED base stops tracking the profile instead of tearing a foot loose -- which is
    exactly how --bottom auto finds the deepest feasible ride height (see probe_depth).
  * The arms are a C2 overlay, not a solve: both shoulders pitch FORWARD proportionally to descent depth
    (humans counterweight a squat the same way). Verified sign: negative shoulder pitch = hand forward =
    COM forward. The overlay shifts the COM ~+20 mm forward at depth (reported per build), which the
    solver absorbs by letting the pelvis sit that much further back -- less ankle dorsiflexion, deeper
    feasible squat. Elbows at 1.2 (the 0 = 90-degrees-bent convention: 1.2 is a natural slight bend).

DEVIATIONS from the library design doc, section "2. SQUAT" (measured, not opined):
  * The doc suggests letting torso PITCH be solved (E4-lite) because "the ankle dorsiflexion stop
    (-0.87) binds before the knee range does". This builder keeps ghost_synth's fixed-upright torso
    (E4 is a shared-solver extension; this task is new-files-only) and instead measures what upright
    costs: the depth probe reports the deepest feasible ride and WHICH limits bind. Measured result:
    ankle_pitch saturates at its -0.8727 stop on BOTH legs, exactly as the doc predicted; with the arm
    counterweight at its measured optimum (gain 4.0 -> shoulders -1.32 rad, COM +25.5 mm forward) the
    upright-torso squat reaches pelvis z = 0.380 m (shipped bottom 0.390 with 10 mm stop margin; knee
    2.32 rad of 2.88, hip -1.45 of -2.53) and the [COM]/[FWP]/[TORQUE] gates all clear. If a future
    campaign needs the last few cm, implement E4 in the shared core.
  * The doc's "small nominal forward pitch profile" is likewise E4 territory -- not authored here,
    because an att_lut the solver did not enforce would be a formula, and formulas are the disease
    this method exists to cure.

KNOWN TRAP (memory 2026-07-01): an old AMP squat reference had hip/ankle SIGNS WRONG; the fix then was
deterministic ratios hip = -0.60*knee, ankle = -(hip+knee). Here every angle comes out of FK-solving
real footholds, so the ratios are not authored -- but they are REPORTED at the bottom frame as a sanity
witness (an upright-torso flat-foot squat must approximately satisfy ankle = -(hip+knee), which is the
closed-chain statement "shank+thigh+torso angles sum to upright").

Usage:
  python ghost_synth_squat.py projects/policies/ghosts/g1/ghost_squat_synth_lut.json
  python build_keypoints.py <out> --links --elbow 1.20 --shroll 0.15     # ruler tables
  python ghost_dynamics.py <out> --smooth 7 --stride 4                   # the verdict
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ghost_synth as GS
import ghost_contacts as GC
from ghost_dynamics import _urdf_limits, DEF_URDF

DT = GS.DT
FOOT_HALF = GS.FOOT_HALF
STANCE_Y = GS.STANCE_Y


# ---------------------------------------------------------------------------------------------------
# THE PLAN. Both feet planted at the neutral stance every frame; base_z is an explicit C2 profile.
# ---------------------------------------------------------------------------------------------------

def plan_squat(settle_s, down_s, hold_s, up_s, stand_s, ride, bottom):
    """Frame count, constant foot targets, all-stance schedule, and the C2 base_z profile.

    The COM target is CONSTANT at the two-foot centroid for the whole routine, so the head and tail
    are trivially at the neutral both-feet midpoint -- the spawn-centred rule from
    ghost_synth.support_anchor._both holds on every frame, not just the ends."""
    nb = int(round((settle_s + down_s + hold_s + up_s + stand_s) / DT))
    tgt = {"L": np.tile([0.0, +STANCE_Y, 0.0], (nb, 1)),
           "R": np.tile([0.0, -STANCE_Y, 0.0], (nb, 1))}
    sched = [["foot_L", "foot_R"] for _ in range(nb)]
    b0 = int(round(settle_s / DT))                 # descend starts
    b1 = b0 + int(round(down_s / DT))              # hold starts
    b2 = b1 + int(round(hold_s / DT))              # ascend starts
    b3 = b2 + int(round(up_s / DT))                # final stand starts
    base_z = np.full(nb, ride)
    for i in range(b0, min(b1, nb)):
        base_z[i] = ride + (bottom - ride) * GS.quintic((i - b0 + 0.5) / max(b1 - b0, 1))
    for i in range(b1, min(b2, nb)):
        base_z[i] = bottom
    for i in range(b2, min(b3, nb)):
        base_z[i] = bottom + (ride - bottom) * GS.quintic((i - b2 + 0.5) / max(b3 - b2, 1))
    return nb, tgt, sched, base_z, (b1, b2)


def arm_pitch_profile(base_z, ride, arm_gain):
    """Both shoulders pitch forward proportionally to descent depth. C2 because base_z is C2.
    Negative shoulder pitch = hand forward = COM forward (FK-verified sign)."""
    return -arm_gain * (ride - base_z)


# ---------------------------------------------------------------------------------------------------
# THE SOLVE -- copied from ghost_synth.solve (origin: projects/policies/training/ghost_synth.py) with
# TWO modifications, both marked below. It could not be reused by parameters: the arm overlay is a
# per-frame formula hardcoded inside the origin's frame loop, and overwriting a formula's output after
# the solve would move the COM the solver just placed.
#   [SQUAT-ARM]  the arms follow the depth-proportional counterweight profile (per-frame array) instead
#                of the walk counter-swing formula. Same overlay structure: set BEFORE the LM iterations
#                of each frame, so the COM row solves the legs+base around the arms' true mass positions.
#   [SQUAT-OERR] the per-frame read-out also records the bearing soles' orientation residual and the
#                base-z tracking error -- the two quantities the depth probe judges feasibility by.
# Everything else, including every origin warning comment, is verbatim -- if you fix a bug here, fix it
# there too. Both feet bear on every frame, so every foot always gets the full STANCE_ROT_ROWS (roll +
# pitch lay the sole flat, yaw pins the heading); there is no swing_s input because nothing ever swings.
# ---------------------------------------------------------------------------------------------------

W_FOOT, W_ORI, W_COM, W_BZ, W_REG = GS.W_FOOT, GS.W_ORI, GS.W_COM, GS.W_BZ, GS.W_REG


def solve_squat(nb, tgt, sched, com_tg, base_z, arm_pitch, elbow, shroll, terr, mjcf=None, iters=120):
    import ghost_close as GCL
    WB_JOINTS = GS.WB_JOINTS
    m = mj.MjModel.from_xml_path(mjcf or GS.DEF_MJCF)
    d = mj.MjData(m)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
    spec = GC.load_spec("feet")
    for c in spec["contacts"]:
        c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
        jids = [j for j in GCL.path_joints(m, c["bid"])
                if mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) in WB_JOINTS]
        c["jwb"] = [WB_JOINTS.index(mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j)) for j in jids]
        c["jdof"] = [m.jnt_dofadr[j] for j in jids]
        c["jlo"] = np.array([m.jnt_range[j][0] for j in jids])
        c["jhi"] = np.array([m.jnt_range[j][1] for j in jids])
    byfoot = {"L": spec["contacts"][0], "R": spec["contacts"][1]}
    cols = [0, 1, 2] + sorted({dd for c in spec["contacts"] for dd in c["jdof"]})

    wb = np.zeros((nb, len(WB_JOINTS)))
    root = np.zeros((nb, 4))
    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv)); jcom = np.zeros((3, m.nv))
    # SEED FROM A CROUCH, NOT FROM ZEROS. With straight legs the foot Jacobian is near-singular in z
    # (the knee has no authority), so the first Levenberg-Marquardt steps are enormous and the base ran
    # to z = -0.57 m. The nominal G1 crouch is well inside the workspace and conditions the whole solve.
    q = np.zeros(len(WB_JOINTS))
    for h, k, an in ((0, 3, 4), (6, 9, 10)):
        q[h], q[k], q[an] = -0.30, 0.60, -0.30
    p = np.array([com_tg[0, 0], com_tg[0, 1], base_z[0]])
    ferr, cerr, oerr, bzerr, unreach = [], [], [], [], []

    def _fk(pp, qq):
        d.qpos[0:3] = pp; d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]       # torso upright: no lean to unlearn
        for k2, n in enumerate(WB_JOINTS):
            d.qpos[qadr[n]] = qq[k2]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)

    legcols = [cols.index(dd) for c in spec["contacts"] for dd in c["jdof"]]
    legwb = [j for c in spec["contacts"] for j in c["jwb"]]

    def _res(i, pp, qq, qprev, want_J):
        _fk(pp, qq)
        rows, res = [], []
        for f in ("L", "R"):
            c = byfoot[f]
            R = d.xmat[c["bid"]].reshape(3, 3)
            sp = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
            nrm = GC.surface_normal(terr, float(tgt[f][i][0]), float(tgt[f][i][1]))
            rr = GS.STANCE_ROT_ROWS                      # both feet bear on every squat frame
            e3 = np.concatenate([GCL._rot_err_xy(R, nrm), [-math.atan2(R[1, 0], R[0, 0])]])
            res.append(W_FOOT * (tgt[f][i] - sp))
            res.append(W_ORI * e3[list(rr)])
            if want_J:
                mj.mj_jac(m, d, jacp, jacr, sp, c["bid"])
                rows.append(W_FOOT * jacp[:, cols]); rows.append(W_ORI * jacr[list(rr)][:, cols])
        res.append(W_COM * (com_tg[i] - d.subtree_com[0][0:2]))
        res.append(np.array([W_BZ * (base_z[i] - pp[2])]))
        # posture task: stay near the previous frame. Cheap, and it removes the nullspace the freed swing
        # ankle would otherwise wander through -- without weakening any constraint that matters.
        res.append(W_REG * (qprev[legwb] - qq[legwb]))
        if want_J:
            mj.mj_jacSubtreeCom(m, d, jcom, 0)
            rows.append(W_COM * jcom[0:2][:, cols])
            rz = np.zeros((1, len(cols))); rz[0, 2] = W_BZ
            rows.append(rz)
            rr2 = np.zeros((len(legwb), len(cols)))
            for k2, cc in enumerate(legcols):
                rr2[k2, cc] = W_REG
            rows.append(rr2)
            return np.concatenate(res), np.vstack(rows)
        return np.concatenate(res), None

    def _apply(pp, qq, dq):
        p2 = pp + dq[0:3]; q2 = qq.copy()
        for c in spec["contacts"]:
            sel = [cols.index(dd) for dd in c["jdof"]]
            q2[c["jwb"]] = np.clip(qq[c["jwb"]] + dq[sel], c["jlo"], c["jhi"])
        return p2, q2

    for i in range(nb):
        # [SQUAT-ARM] the arm overlay: both shoulders follow the depth-proportional counterweight
        # profile; shoulder roll and elbow are constant carry values (elbow 1.2: 0 = 90-deg bent).
        q[13] = q[18] = float(arm_pitch[i])
        q[14], q[19] = shroll, -shroll
        q[16] = q[21] = elbow
        qprev = q.copy()
        lam = 1e-2
        r, J = _res(i, p, q, qprev, True)
        cost = float(r @ r)
        for _ in range(iters):
            if cost < 1e-14:
                break
            dq = np.linalg.solve(J.T @ J + lam * np.eye(J.shape[1]), J.T @ r)
            nrm_dq = float(np.linalg.norm(dq))
            if nrm_dq > 0.20:                                       # trust region: never leap the workspace
                dq *= 0.20 / nrm_dq
            p2, q2 = _apply(p, q, dq)
            r2, _ = _res(i, p2, q2, qprev, False)
            c2 = float(r2 @ r2)
            if c2 < cost:                                           # accept, and trust the model more
                p, q, cost = p2, q2, c2
                lam = max(lam * 0.6, 1e-8)
                r, J = _res(i, p, q, qprev, True)
            else:                                                   # reject, and trust it less
                lam *= 3.0
                if lam > 1e6:
                    break
        _fk(p, q)  # leaves d at the accepted solution for the residual read-out below
        e = max(float(np.linalg.norm((d.xpos[byfoot[f]["bid"]]
                                      + d.xmat[byfoot[f]["bid"]].reshape(3, 3) @ np.asarray(byfoot[f]["patch"], float))
                                     - tgt[f][i])) for f in ("L", "R"))
        ferr.append(e)
        # GATE 2's OWN RESIDUAL. Report it: the foot rows are weighted 10 and the COM row 1, so whenever
        # the system is inconsistent (a joint on its limit) it is the COM that quietly gets sacrificed --
        # and the COM is the whole point.
        cerr.append(float(np.linalg.norm(d.subtree_com[0][0:2] - com_tg[i])))
        # [SQUAT-OERR] sole-flatness + base-z tracking read-outs: the depth probe's feasibility rulers.
        oe = 0.0
        for f in ("L", "R"):
            c = byfoot[f]
            R = d.xmat[c["bid"]].reshape(3, 3)
            nrm = GC.surface_normal(terr, float(tgt[f][i][0]), float(tgt[f][i][1]))
            oe = max(oe, float(np.linalg.norm(GCL._rot_err_xy(R, nrm))))
        oerr.append(oe)
        bzerr.append(abs(float(base_z[i] - p[2])))
        if e > 2e-3:
            unreach.append((i, e))
        wb[i] = q
        root[i] = [p[0], p[1], p[2], 0.0]
    return wb, root, np.array(ferr), np.array(cerr), np.array(oerr), np.array(bzerr), unreach, m, d, spec


# ---------------------------------------------------------------------------------------------------
# DEPTH PROBE. Ask for a slow staircase of ever-deeper base_z targets and watch where the solve stops
# delivering them. Because base_z is the WEAKEST row (W_BZ=1), infeasibility shows up as the solved
# pelvis refusing to follow the profile (bzerr grows) or, past that, as the COM/sole rows losing to a
# clipped joint -- never as a torn-loose foot. "Deepest feasible" = the deepest requested z on which
# every ruler still holds.
# ---------------------------------------------------------------------------------------------------

def probe_depth(ride, floor, step, arm_gain, elbow, shroll, terr, com_tg_xy, mjcf):
    nz = max(2, int(round((ride - floor) / step)) + 1)
    zreq = np.linspace(ride, floor, nz)
    tgt = {"L": np.tile([0.0, +STANCE_Y, 0.0], (nz, 1)),
           "R": np.tile([0.0, -STANCE_Y, 0.0], (nz, 1))}
    sched = [["foot_L", "foot_R"] for _ in range(nz)]
    com_tg = np.tile(com_tg_xy, (nz, 1))
    ap = arm_pitch_profile(zreq, ride, arm_gain)
    wb, root, ferr, cerr, oerr, bzerr, _unr, m, d, spec = solve_squat(
        nz, tgt, sched, com_tg, zreq, ap, elbow, shroll, terr, mjcf)
    ok = (ferr < 1.0e-3) & (cerr < 1.5e-3) & (oerr < 0.010) & (bzerr < 3.0e-3)
    deepest_i = None
    for i in range(nz):
        if ok[i]:
            deepest_i = i
        else:
            break                       # feasibility is monotone in depth; stop at the first failure
    if deepest_i is None:
        raise SystemExit("ghost_synth_squat: even the ride height is infeasible -- check --ride")
    # which joint limits bind just past the deepest feasible depth?
    jlo = {n: m.jnt_range[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)][0] for n in GS.WB_JOINTS}
    jhi = {n: m.jnt_range[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)][1] for n in GS.WB_JOINTS}
    at_limit = []
    qrow = wb[min(deepest_i + 1, nz - 1)]
    for k, n in enumerate(GS.WB_JOINTS):
        if k >= 12:
            continue                    # legs only: the arms are an overlay, not solved
        if qrow[k] - jlo[n] < 2e-3 or jhi[n] - qrow[k] < 2e-3:
            at_limit.append("%s=%+.3f (range %+.3f..%+.3f)" % (n, qrow[k], jlo[n], jhi[n]))
    return float(zreq[deepest_i]), at_limit, (zreq, ferr, cerr, oerr, bzerr)


# ---------------------------------------------------------------------------------------------------
# BOTTOM-FRAME STATIC REPORT: the FWP torque split at the deepest posture (qvel = qacc = 0), computed
# the same way ghost_dynamics does it (mj_inverse floating-base rows -> fwp_check_G -> treq), so the
# number printed here and the [TORQUE] gate are one computation of one physics.
# ---------------------------------------------------------------------------------------------------

def bottom_report(m, spec, root_row, wb_row, mu=1.0):
    m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT) | int(mj.mjtDisableBit.mjDSBL_CONSTRAINT)
    d = mj.MjData(m)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in GS.WB_JOINTS}
    dadr = {n: m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in GS.WB_JOINTS}
    d.qpos[0:3] = root_row[0:3]; d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for k, n in enumerate(GS.WB_JOINTS):
        d.qpos[qadr[n]] = wb_row[k]
    d.qvel[:] = 0.0
    mj.mj_forward(m, d)
    d.qacc[:] = 0.0                     # trap 1: qacc AFTER mj_forward, before mj_inverse
    mj.mj_inverse(m, d)
    qfi = d.qfrc_inverse.copy()
    active = []
    for c in spec["contacts"]:
        bid = c["bid"]
        R = d.xmat[bid].reshape(3, 3)
        cs = GC.corners(d.xpos[bid], R, c["half"], patch=c["patch"])
        active.append(dict(name=c["name"], bid=bid, corners=cs,
                           normals=[np.array([0.0, 0.0, 1.0])] * len(cs)))
    cols, pts, owner, corner_pos, cbid = GC.build_columns(active, mu)
    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
    Jc = []
    for cp, bid in zip(corner_pos, cbid):
        mj.mj_jac(m, d, jacp, jacr, cp, bid)
        Jc.append(jacp.copy())
    Gfull = np.stack([Jc[owner[j]].T @ cols[j] for j in range(len(cols))], axis=1)
    eff, _vel = _urdf_limits(DEF_URDF)
    jnames = [n for n in GS.WB_JOINTS if eff.get(n, 0) > 0]
    jrows = [dadr[n] for n in jnames]
    jlims = [eff[n] for n in jnames]
    ok, t, f = GC.fwp_check_G(Gfull[0:6, :], qfi[0:6].copy(), Gfull[jrows, :], qfi[jrows], jlims,
                              owner, cols, len(corner_pos))
    tau = {}
    if ok:
        tau_c = np.zeros(m.nv)
        for ci in range(len(corner_pos)):
            tau_c += Jc[ci].T @ f[ci]
        treq = qfi - tau_c
        tau = {n: float(treq[dadr[n]]) for n in jnames}
    com = d.subtree_com[0].copy()
    return ok, t, tau, {n: eff[n] for n in jnames}, com


def filmstrip(m, spec, root, wb, out_png, nframes=8):
    """Side-view (x-z) filmstrip: legs, soles, arms, COM dot + its ground projection."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = mj.MjData(m)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in GS.WB_JOINTS}
    bid = {n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n) for n in (
        "pelvis", "left_hip_pitch_link", "left_knee_link", "left_ankle_roll_link",
        "right_hip_pitch_link", "right_knee_link", "right_ankle_roll_link",
        "left_shoulder_pitch_link", "left_elbow_link", "left_wrist_roll_rubber_hand",
        "torso_link")}
    nb = wb.shape[0]
    idx = np.linspace(0, nb - 1, nframes).round().astype(int)
    fig, axes = plt.subplots(2, 4, figsize=(16, 8), sharex=True, sharey=True)
    for ax, i in zip(axes.ravel(), idx):
        d.qpos[0:3] = root[i, 0:3]; d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for k, n in enumerate(GS.WB_JOINTS):
            d.qpos[qadr[n]] = wb[i, k]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
        P = {n: d.xpos[b].copy() for n, b in bid.items()}
        ax.axhline(0.0, color="0.6", lw=1)
        for hip, knee, ank, cst in (("left_hip_pitch_link", "left_knee_link", "left_ankle_roll_link", "tab:blue"),
                                    ("right_hip_pitch_link", "right_knee_link", "right_ankle_roll_link", "tab:cyan")):
            seg = np.array([P["pelvis"], P[hip], P[knee], P[ank]])
            ax.plot(seg[:, 0], seg[:, 2], "-o", color=cst, ms=2.5, lw=1.6)
        for c in spec["contacts"]:                      # soles, drawn from the true patch pose
            R = d.xmat[c["bid"]].reshape(3, 3)
            ctr = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
            toe = ctr + R @ np.array([FOOT_HALF, 0.0, 0.0])
            heel = ctr - R @ np.array([FOOT_HALF, 0.0, 0.0])
            ax.plot([heel[0], toe[0]], [heel[2], toe[2]], "-", color="k", lw=2.5)
        arm = np.array([P["left_shoulder_pitch_link"], P["left_elbow_link"], P["left_wrist_roll_rubber_hand"]])
        ax.plot(arm[:, 0], arm[:, 2], "-o", color="tab:orange", ms=2.5, lw=1.4)
        ax.plot([P["pelvis"][0], P["left_shoulder_pitch_link"][0]],
                [P["pelvis"][2], P["left_shoulder_pitch_link"][2]], "-", color="0.4", lw=1.4)
        com = d.subtree_com[0]
        ax.plot(com[0], com[2], "o", color="red", ms=6)
        ax.plot(com[0], 0.0, "x", color="red", ms=7, mew=2)
        ax.plot([com[0], com[0]], [0.0, com[2]], ":", color="red", lw=0.8)
        ax.set_title("t=%.2fs  z=%.3f" % (i * DT, root[i, 2]), fontsize=9)
        ax.set_aspect("equal"); ax.set_xlim(-0.55, 0.55); ax.set_ylim(-0.05, 1.0)
        ax.grid(alpha=0.25)
    fig.suptitle("ghost_squat_synth: side view -- legs, soles, arm, COM (dot) + ground projection (x)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--ride", type=float, default=0.72, help="standing pelvis height (m)")
    ap.add_argument("--bottom", type=float, default=None,
                    help="pelvis height at the deep hold (m). Default: FOUND BY THE DEPTH PROBE -- the "
                         "deepest requested z the solve still delivers (foot < 1 mm, COM < 1.5 mm, sole "
                         "flat < 0.01 rad, base-z tracking < 3 mm), plus --depth-margin.")
    ap.add_argument("--depth-margin", type=float, default=0.01,
                    help="back-off above the probed kinematic limit (m). At the exact limit the ankle "
                         "sits ON its stop; a reference should not park a joint at zero margin.")
    ap.add_argument("--floor", type=float, default=0.05, help="probe scan floor (m)")
    ap.add_argument("--probe-step", type=float, default=0.005, help="probe scan step (m)")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--down-s", type=float, default=2.0)
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--up-s", type=float, default=2.0)
    ap.add_argument("--stand-s", type=float, default=1.5)
    ap.add_argument("--arm-gain", type=float, default=4.0,
                    help="shoulder forward pitch per metre of descent (rad/m); the COM counterweight. "
                         "MEASURED depth sweep (probe, ride 0.72): gain 0 -> deepest 0.495 m, 1 -> 0.475, "
                         "2 -> 0.450, 3 -> 0.410, 4 -> 0.380 (optimum), 5 -> 0.385, 6 -> 0.405. Past "
                         "~-1.4 rad of shoulder pitch the hands rise instead of advancing, so the "
                         "forward COM shift -- and with it the extra depth -- turns around.")
    ap.add_argument("--elbow", type=float, default=1.2)
    ap.add_argument("--shroll", type=float, default=0.15)
    ap.add_argument("--mjcf", default=None)
    ap.add_argument("--filmstrip", default=None, help="write a side-view PNG here")
    a = ap.parse_args()

    terr = {"type": "flat", "z": 0.0}
    patch_x = float(GC.load_spec("feet")["contacts"][0]["patch"][0])
    com_xy = np.array([0.0 - patch_x, 0.0])        # centroid of the feet, corrected to the ANKLE line

    bottom = a.bottom
    probe_info = None
    if bottom is None:
        deepest, at_limit, probe_info = probe_depth(a.ride, a.floor, a.probe_step, a.arm_gain,
                                                    a.elbow, a.shroll, terr, com_xy, a.mjcf)
        bottom = deepest + a.depth_margin
        print("DEPTH PROBE  deepest feasible ride %.3f m (scan %.3f..%.3f step %.3f)"
              % (deepest, a.ride, a.floor, a.probe_step))
        print("  binding at the first infeasible depth: %s"
              % ("; ".join(at_limit) if at_limit else "(no leg joint pinned -- residual criteria failed first)"))
        print("  -> bottom = %.3f m  (probe limit + %.3f m margin)" % (bottom, a.depth_margin))

    nb, tgt, sched, base_z, (i_bot0, i_bot1) = plan_squat(a.settle_s, a.down_s, a.hold_s, a.up_s,
                                                          a.stand_s, a.ride, bottom)
    com_tg = np.tile(com_xy, (nb, 1))
    ap_prof = arm_pitch_profile(base_z, a.ride, a.arm_gain)
    wb, root, ferr, cerr, oerr, bzerr, unreach, m, d, spec = solve_squat(
        nb, tgt, sched, com_tg, base_z, ap_prof, a.elbow, a.shroll, terr, a.mjcf)

    cyc = nb * DT
    print("GHOST SYNTH SQUAT  one-shot seq, flat ground, both soles planted every frame")
    print("  nb=%d, %.2f s  (settle %.1f + down %.1f + hold %.1f + up %.1f + stand %.1f)"
          % (nb, cyc, a.settle_s, a.down_s, a.hold_s, a.up_s, a.stand_s))
    print("  ride %.3f m -> bottom %.3f m  (depth %.3f m)" % (a.ride, bottom, a.ride - bottom))
    print("  foot-target residual  mean %.3f mm  max %.3f mm   unreachable frames %d/%d"
          % (1000 * ferr.mean(), 1000 * ferr.max(), len(unreach), nb))
    print("  COM-to-target (gate 2)  mean %.2f mm  max %.2f mm %s"
          % (1000 * cerr.mean(), 1000 * cerr.max(),
             "<-- the COM row lost to a joint limit; the support margin pays for it" if cerr.max() > 5e-3 else ""))
    print("  sole flatness  max %.4f rad    base-z tracking  max %.2f mm %s"
          % (oerr.max(), 1000 * bzerr.max(),
             "<-- profile NOT delivered; raise --bottom" if bzerr.max() > 5e-3 else ""))
    print("  base  z %.3f..%.3f   x %+.3f..%+.3f   y %+.3f..%+.3f"
          % (root[:, 2].min(), root[:, 2].max(), root[:, 0].min(), root[:, 0].max(),
             root[:, 1].min(), root[:, 1].max()))

    # ---- bottom-frame report: angles, ratio witness, static FWP torque split ----------------------
    ib = (i_bot0 + i_bot1) // 2                       # middle of the deep hold
    J = GS.WB_JOINTS
    hp, kn, an = (float(wb[ib, J.index(n)]) for n in
                  ("left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"))
    print("  BOTTOM (frame %d, t=%.2f s, pelvis z %.3f):" % (ib, ib * DT, root[ib, 2]))
    print("    hip_pitch %+.3f  knee %+.3f  ankle_pitch %+.3f  (left; right symmetric to %.1e)"
          % (hp, kn, an, float(np.abs(wb[ib, 0:6] - wb[ib, 6:12]).max())))
    print("    RATIO WITNESS vs the 2026-07-01 deterministic fix (hip=-0.60*knee, ankle=-(hip+knee)):")
    print("      hip   %+.3f  vs -0.60*knee   = %+.3f   (ratio hip/knee = %+.3f)"
          % (hp, -0.60 * kn, hp / kn if abs(kn) > 1e-9 else float("nan")))
    print("      ankle %+.3f  vs -(hip+knee)  = %+.3f" % (an, -(hp + kn)))
    ok, t, tau, lims, comb = bottom_report(m, spec, root[ib], wb[ib])
    if ok:
        print("    static FWP at the bottom: FEASIBLE, best-split peak torque ratio t = %.3f" % t)
        for n in ("left_knee_joint", "left_hip_pitch_joint", "left_ankle_pitch_joint"):
            print("      %-24s %6.1f / %5.1f N*m = %.2f" % (n, abs(tau[n]), lims[n], abs(tau[n]) / lims[n]))
    else:
        print("    static FWP at the bottom: INFEASIBLE <-- do not ship; raise --bottom")
    # arm counterweight contribution to COM x: same posture with the shoulders zeroed
    wb0 = wb[ib].copy(); wb0[13] = wb0[18] = 0.0
    _ok0, _t0, _tau0, _l0, comb0 = bottom_report(m, spec, root[ib], wb0)
    print("    arm overlay at bottom: shoulder_pitch %+.3f rad -> COM x shift %+.1f mm (with-arms %+.4f, "
          "hanging %+.4f; target %+.4f)"
          % (float(ap_prof[ib]), 1000 * (comb[0] - comb0[0]), comb[0], comb0[0], com_xy[0]))

    out = {
        "nb": nb, "freq": 1.0 / cyc, "cycle_s": cyc, "vx": 0.0, "seq": True, "hold_end": True,
        "arm_A": 0.0, "shroll": float(a.shroll), "squat_arm_gain": float(a.arm_gain),
        "ride": float(a.ride), "bottom": float(bottom),
        "leg_lut": [[float(v) for v in r[0:12]] for r in wb],
        "arm_lut": [[float(r[13]), float(r[18])] for r in wb],
        "elbow_lut": [[float(a.elbow), float(a.elbow)] for _ in range(nb)],
        "att_lut": [[0.0, 0.0] for _ in range(nb)],
        "root_lut": [[float(v) for v in r] for r in root],
        "wb_joints": GS.WB_JOINTS, "wb_lut": [[float(v) for v in r] for r in wb],
        "contact_schedule": sched, "terrain": terr, "ghost_schema": 2, "ghost_closed": True,
        "source": ("SYNTHESIZED by ghost_synth_squat.py (plan the contacts, solve everything else): "
                   "one-shot deep squat, both soles planted at the neutral stance every frame; base "
                   "(x,y,z) SOLVED per frame with the 12 leg joints so the COM stays over the two-foot "
                   "centroid (ankle line, x=%.3f); base_z follows a C2 quintic profile ride %.3f -> "
                   "bottom %.3f -> ride (depth probed against the joint limits, margin %.3f); arms are "
                   "a C2 depth-proportional forward counterweight (gain %.1f rad/m). Foot residual max "
                   "%.2f mm, COM-to-target max %.2f mm, sole flatness max %.4f rad, base-z tracking max "
                   "%.2f mm. Gates 1+2 by construction; gate 3 is ghost_dynamics' to judge."
                   % (com_xy[0], a.ride, bottom, a.depth_margin, a.arm_gain,
                      1000 * ferr.max(), 1000 * cerr.max(), oerr.max(), 1000 * bzerr.max())),
    }
    json.dump(out, open(a.out, "w"))
    print("  wrote %s" % a.out)
    if a.filmstrip:
        filmstrip(m, spec, root, wb, a.filmstrip)
        print("  filmstrip -> %s" % a.filmstrip)
    print("  NEXT: build_keypoints.py %s --links --elbow %.2f --shroll %.2f ; "
          "ghost_dynamics.py %s --smooth 7 --stride 4"
          % (a.out, a.elbow, a.shroll, a.out))


if __name__ == "__main__":
    main()
