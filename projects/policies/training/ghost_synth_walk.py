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

"""ghost_synth_walk.py -- SYNTHESIZE a CYCLIC, SELF-SUPPORTING straight-line walk ghost.

THE METHOD (ghost_synth.py, generalized to a cycle)
Plan the CONTACTS. Solve everything else. Footholds are world poses on flat ground; every stance frame
targets the SAME world pose; the base (x, y, z) is SOLVED per frame together with the 12 leg joints by
Levenberg-Marquardt on the real mujoco Jacobians (mj_jac + mj_jacSubtreeCom) so the COM rides over the
bearing foot's ANKLE during single support and transfers by a quintic during double support. Nothing is
a formula. The flagship RECORDED walk ghost (ghost_official_full_v3) keeps its COM on the centreline
while its stance foot sits 80-100 mm to the side -- ~21 N*m of crane, permanently. This builder exists
to beat that number with 0.0 N by construction.

WHAT IS NEW HERE vs ghost_synth.py (stairs): the output is a CYCLE, not a one-shot `seq` routine.
ghost_dynamics treats a cyclic lut (no `seq` key) with WRAPPED finite differences: frame nb is frame 0
translated by one cycle of travel. Three things make that true:

  1. STEADY-STATE EXTRACTION. We do not solve one isolated cycle: the LM solve is warm-started frame to
     frame and the support-anchor interpolation clamps at the array ends, so the first cycle after the
     settle carries a transient. Instead we lay out `--n-steps` (default 6) steps, solve the WHOLE
     timeline exactly as the stairs builder does, and cut steps 3+4 -- one full L+R cycle, interior on
     both sides, where the solution is periodic because its inputs are (footholds/anchors for step k+2
     are step k's translated by 2*stride). Measured periodicity of the solve itself is reported
     (wb[i0+nb] vs wb[i0]) and is typically < 1e-4 rad.
  2. THE SEAM IS PLACED AT A QUIET INSTANT: the cycle starts on the first double-support frame, right
     after touchdown, where the swing bump (sin^3, C2 at its ends), the swing quintic and the anchor
     transfer quintic ALL have zero velocity. So wb_lut[-1] -> wb_lut[0] is a one-bin step at
     near-zero joint speed, and the wrapped derivatives see no kink.
  3. Any residual periodicity error is MORPHED OUT: a linear ramp over the cycle (error/nb per frame,
     micrometres) makes frame nb EXACTLY frame 0 + [2*stride, 0, 0, 0]. The ramp moves a stance foot by
     at most the periodicity error itself, which is orders below the 1 mm closure gate.

One cycle = 2 steps (L then R). The contact schedule is DECLARED per frame (double support during the
transfers, the bearing foot alone during a swing); the stance that spans the seam is split by
ghost_dynamics at the array boundary into two runs, each of which holds a world-fixed foothold, so
[CLOSURE] judges it correctly without special cases.

Everything hard-won lives in ghost_synth.py and is IMPORTED, not copied: the crouch seed (straight legs
= singular Jacobian), the sin^3 swing bump (C2 -- no impulsive lift-off), the solved-not-clamped apex,
the dropped swing-foot ROLL row (ankle_roll saturates and the COM row pays), and the posture
regulariser (the freed swing ankle otherwise wanders its nullspace). Read that module's comments before
touching the solve.

GATE 3 IS THE LATERAL TRANSFER'S TO LOSE -- and it took three measured fixes, not one:
  1. cosh_s, not quintic, for the double-support COM transfer: a quintic's lateral acceleration peaks
     while the COM is still over the outgoing foot, where the CoP lever is ~3 cm and the LIPM bound is
     0.43 m/s^2. Every quintic build failed the FWP at the transfer ends at ANY practical ds.
  2. --com-inset: the residual infeasibility was mu-INSENSITIVE -- pure CoP lever geometry. The solved
     pelvis overshoots the foot line, so the catch has no lever; 12 mm of COM-target inset buys it.
  3. --ds-s >= ~1.3 s: the legs' own angular momentum while the pelvis sways scales ~1/T^2 and eats
     the last few N*m below that.
At the defaults below the verdict is 0 infeasible frames, 0.0 N + 0.0 N*m of external help, COM margin
+0.018 m, torque 0.47, speed 0.23 (ghost_dynamics --smooth 7). vx is 0.131 m/s: SELF-SUPPORT ON 6 CM
WIDE FEET IS SLOW. The measured tradeoff (external moment vs vx, quintic vs cosh) is in the module's
campaign notes; the flagship RECORDED walk does 0.45 m/s only because the crane pays ~21 N*m.

Usage:
  python ghost_synth_walk.py projects/policies/controllers/g1_ghost/ghost_walk_synth_lut.json
  python build_keypoints.py <out> --links --elbow 1.20 --shroll 0.15    # ruler tables
  python ghost_dynamics.py <out> --smooth 7 --stride 4                  # the verdict
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

DT = GS.DT
FOOT_HALF = GS.FOOT_HALF
STANCE_Y = GS.STANCE_Y


def plan_walk(stride, n_steps):
    """Alternating straight-line footfalls on flat ground: footfall k at x = k*stride, L odd, R even.
    Both feet start side by side at x = 0. From step 3 on, the pattern is exactly periodic: each foot
    advances 2*stride per own step, footfalls alternate stride apart."""
    start = {"L": np.array([0.0, +STANCE_Y, 0.0]), "R": np.array([0.0, -STANCE_Y, 0.0])}
    steps, cur = [], {k: v.copy() for k, v in start.items()}
    for k in range(1, n_steps + 1):
        f = "L" if k % 2 == 1 else "R"
        nxt = cur[f].copy()
        nxt[0] = k * stride
        steps.append((f, cur[f].copy(), nxt.copy()))
        cur[f] = nxt
    return start, steps


def cosh_s(u, k):
    """Symmetric S-curve on [0,1] built from cosh segments: f(0)=0, f(1)=1, f'(0)=f'(1)=0, and the
    ACCELERATION GROWS WITH THE DISTANCE TRAVELLED (|f''| = k^2 * (f + a0-ish)), instead of peaking
    near the ends the way a quintic does.

    ⛔ WHY (measured, ds/ss sweep 0.4..0.9 s, judged by ghost_dynamics): with a QUINTIC double-support
    transfer every single infeasible frame sits at the START or END of double support, demanding a pure
    ROLL moment (e = [0,0,0 | +-20..41 N*m Mx, ...]). The physics: a quintic's lateral acceleration
    peaks at s=0.28, when the COM is still nearly over the outgoing foot -- where the CoP has only the
    ~3 cm of sole beyond the foot's centreline as a lever. The inverted-pendulum bound there is
    |ay| <= (g/h)*0.03 = 0.43 m/s^2; the quintic demands 5.77*dy/T^2 (1.7 m/s^2 even at ds=0.9 s), and
    the validator rightly says no contact forces can do it. Matching the quintic to the bound needs
    ds ~ 2 s. The cosh profile is the LIPM-native transfer -- CoP parked near the outgoing hull edge,
    COM accelerating away exponentially, mirrored for the catch -- so its demand tracks the available
    lever at every phase, and it crosses in ~1/2 the time a compliant quintic would need.
    k = lambda*T in LIPM terms; keep k <= ~0.92 * omega*T (omega = sqrt(g/h)) or the CoP the profile
    implies walks outside the sole."""
    u = min(max(u, 0.0), 1.0)
    C = 2.0 * (math.cosh(0.5 * k) - 1.0)
    if u <= 0.5:
        return (math.cosh(k * u) - 1.0) / C
    return 1.0 - (math.cosh(k * (1.0 - u)) - 1.0) / C


def walk_anchor(nb, tgt, sched, patch_x, ride, k_ratio=0.92, com_inset=0.010):
    """Support anchor for the cyclic walk -- copied from ghost_synth.support_anchor (origin:
    projects/policies/training/ghost_synth.py) with TWO changes, marked [WALK-COSH] and [WALK-INSET]:
    the LATERAL (y) interpolation across double support uses cosh_s instead of quintic (see cosh_s's
    comment for the measured roll-moment wall the quintic hits), and the single-support COM target sits
    `com_inset` INSIDE the bearing foot's centreline. x and z keep the original quintic. All the
    origin's warnings hold: base_z is never min(zL,zR)+ride, and the COM rides over the ANKLE (patch_x
    backed off), not the sole centre.

    ⛔ WHY THE INSET (measured, mu-probe: the residual infeasibility is mu-INSENSITIVE, so it is CoP
    lever geometry, not friction): with the COM dead on the foot's centreline the SOLVED pelvis
    overshoots to ~+-0.145 (leg mass pulls the COM inward), so at the catch end of every transfer the
    near sole offers only ~2 cm of one-sided lever -- and the legs' own angular momentum while the
    pelvis sways demands a few N*m more than that lever supplies (e = pure Mx of 2..8 N*m on 6-10% of
    frames, at ANY ds). Shifting the target 10 mm toward the centreline buys the catch ~3.5 N*m of
    lever and shortens the transfer, at the price of 10 mm of [COM] margin (0.030 -> 0.020, still
    above the 0.015 gate)."""
    anchor = np.full((nb, 3), np.nan)
    for i in range(nb):
        if len(sched[i]) == 1:
            f = sched[i][0][-1]
            yv = tgt[f][i][1]
            yv -= math.copysign(com_inset, yv)                     # [WALK-INSET]
            anchor[i] = [tgt[f][i][0] - patch_x, yv, tgt[f][i][2] + ride]
    idx = np.where(~np.isnan(anchor[:, 0]))[0]
    if len(idx) == 0:
        raise SystemExit("ghost_synth_walk: no single-support frame -- nothing defines the base path")
    omega = math.sqrt(9.81 / ride)
    out = np.zeros((nb, 3))
    for i in range(nb):
        if not np.isnan(anchor[i, 0]):
            out[i] = anchor[i]
            continue
        prv = idx[idx < i]; nxt = idx[idx > i]
        if len(prv) == 0:
            out[i] = anchor[nxt[0]]
        elif len(nxt) == 0:
            out[i] = anchor[prv[-1]]
        else:
            a, b = prv[-1], nxt[0]
            u = (i - a) / float(b - a)
            out[i] = anchor[a] + (anchor[b] - anchor[a]) * GS.quintic(u)
            # [WALK-COSH] lateral transfer: acceleration must grow with the lever, not lead it
            k = k_ratio * omega * (b - a) * DT
            out[i, 1] = anchor[a, 1] + (anchor[b, 1] - anchor[a, 1]) * cosh_s(u, k)
    return out[:, 0:2], out[:, 2]


def _wroll(sv):
    """Weight of the swing foot's ROLL orientation row, as a function of swing phase.

    ⛔ WHY THIS EXISTS (measured on the first cyclic build, ds=ss=0.5): ghost_synth.solve drops the
    swing foot's roll row BINARILY -- correct on stairs and correct in principle here too (holding the
    sole level while the pelvis rides ~0.26 m across needs ~0.42 rad of ankle_roll against a 0.262 rad
    limit), but the one-frame ON/OFF switch is an impulse generator on a cycle: at touchdown the
    re-enabled roll row yanked ankle_roll 0.28 rad in ONE frame (17.6 rad/s), and because the clipped
    row is inconsistent, the redistribution also stepped the knee/hip by ~0.011 rad at the same frame
    -- the exact moment a reference must be gentlest, and the frame the cyclic SEAM sits on. RAMP the
    row instead: full weight through stance, quintic fade over the first 30% of the swing, quintic
    rise over the last 30%. The joint then rides toward its (clipped) touchdown state smoothly, and
    the mid-swing dead zone keeps ghost_synth's lesson intact: no impossible roll demand mid-swing,
    the COM row never has to pay there."""
    if math.isnan(sv):
        return 1.0
    if sv <= 0.30:
        return GS.quintic((0.30 - sv) / 0.30)
    if sv >= 0.70:
        return GS.quintic((sv - 0.70) / 0.30)
    return 0.0


# ---------------------------------------------------------------------------------------------------
# THE SOLVE -- copied from ghost_synth.solve (origin: projects/policies/training/ghost_synth.py) with
# ONE modification, marked [WALK-RAMP] below: the swing foot's roll orientation row is weighted by
# _wroll(s) instead of being dropped binarily (see _wroll's comment for the measured impulse it kills).
# Everything else, including every warning comment, is verbatim from the origin -- if you fix a bug
# here, fix it there too. It could not be reused by parameters: the row selection is hardcoded inside
# the residual closure.
# ---------------------------------------------------------------------------------------------------

W_FOOT, W_ORI, W_COM, W_BZ, W_REG = GS.W_FOOT, GS.W_ORI, GS.W_COM, GS.W_BZ, GS.W_REG


def solve_walk(nb, tgt, swing_s, sched, com_tg, base_z, terr, arm_A, elbow, shroll, mjcf=None, iters=120):
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
    # ⛔ SEED FROM A CROUCH, NOT FROM ZEROS. With straight legs the foot Jacobian is near-singular in z
    # (the knee has no authority), so the first Levenberg-Marquardt steps are enormous and the base ran
    # to z = -0.57 m. The nominal G1 crouch is well inside the workspace and conditions the whole solve.
    q = np.zeros(len(WB_JOINTS))
    for h, k, an in ((0, 3, 4), (6, 9, 10)):
        q[h], q[k], q[an] = -0.30, 0.60, -0.30
    p = np.array([com_tg[0, 0], com_tg[0, 1], base_z[0]])
    ferr, cerr, unreach = [], [], []

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
            # [WALK-RAMP] the ONLY change vs ghost_synth.solve: the roll row is weighted by _wroll(s)
            # instead of selected binarily via STANCE_ROT_ROWS/SWING_ROT_ROWS. All the origin's row
            # warnings still apply: never keep a full-weight roll row on a mid-swing foot (ankle_roll
            # saturates and the COM row pays), and never down-weight pitch/yaw (nullspace wander).
            wr = _wroll(float(swing_s[f][i]))
            rr = (0, 1, 2) if wr > 1e-6 else (1, 2)
            wvec = np.array(([W_ORI * wr] if wr > 1e-6 else []) + [W_ORI, W_ORI])
            e3 = np.concatenate([GCL._rot_err_xy(R, nrm), [-math.atan2(R[1, 0], R[0, 0])]])
            res.append(W_FOOT * (tgt[f][i] - sp))
            res.append(wvec * e3[list(rr)])
            if want_J:
                mj.mj_jac(m, d, jacp, jacr, sp, c["bid"])
                rows.append(W_FOOT * jacp[:, cols])
                rows.append(wvec[:, None] * jacr[list(rr)][:, cols])
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
        # arms counter-swing the same-side foot's offset from the pelvis; elbow is a constant carry pose
        for f, (ap, ar, el) in (("L", (13, 14, 16)), ("R", (18, 19, 21))):
            q[ap] = -arm_A * (tgt[f][i][0] - p[0])
            q[ar] = shroll if f == "L" else -shroll
            q[el] = elbow
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
        if e > 2e-3:
            unreach.append((i, e))
        wb[i] = q
        root[i] = [p[0], p[1], p[2], 0.0]
    return wb, root, np.array(ferr), np.array(cerr), unreach, m, d, spec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--stride", type=float, default=0.25, help="footfall spacing (m per step); one cycle travels 2x this")
    ap.add_argument("--ride", type=float, default=0.68,
                    help="pelvis height above the bearing sole (m). ⛔ NOT 0.72 like the stairs default: "
                         "at touchdown the landing leg reaches stride+patch ahead AND ~0.26 m across, and "
                         "at 0.72 the knee arrives STRAIGHT, sticks on its extension stop through early "
                         "double support, then pops 0.22 rad in one frame when the pelvis catches up. "
                         "0.68 keeps the knee >= ~0.3 rad bent at touchdown. 0.65 is too low: ankle_pitch "
                         "clips in the deep crouch and the COM row pays 13 mm.")
    ap.add_argument("--ds-s", type=float, default=1.30,
                    help="double-support weight transfer (s). The gate-3 lever: the lateral COM "
                         "transfer is CoP-lever-limited (see cosh_s), and below ~1.2 s frames start "
                         "failing the FWP no matter the profile.")
    ap.add_argument("--ss-s", type=float, default=0.60, help="swing (s)")
    ap.add_argument("--settle-s", type=float, default=0.8, help="lead-in before step 1 (discarded)")
    ap.add_argument("--top-s", type=float, default=1.5, help="tail hold after the last step (discarded)")
    ap.add_argument("--n-steps", type=int, default=6, help="steps laid out; the extracted cycle is steps 3+4")
    ap.add_argument("--clear", type=float, default=0.04, help="nominal swing apex above the ground (m)")
    ap.add_argument("--min-gap", type=float, default=0.020, help="required mid-swing sole-to-ground gap (m)")
    ap.add_argument("--com-inset", type=float, default=0.012,
                    help="single-support COM target sits this far INSIDE the bearing foot's centreline "
                         "(m). See walk_anchor: buys the transfer catch its CoP lever; costs the same "
                         "amount of [COM] margin.")
    ap.add_argument("--k-ratio", type=float, default=0.70,
                    help="cosh transfer curvature as a fraction of the LIPM omega*T (see cosh_s). "
                         "Higher = harder push at the hull edges, less CoP margin there.")
    ap.add_argument("--arm-A", type=float, default=0.35)
    ap.add_argument("--elbow", type=float, default=1.2)
    ap.add_argument("--shroll", type=float, default=0.15)
    ap.add_argument("--mjcf", default=None)
    a = ap.parse_args()
    if a.n_steps < 5:
        raise SystemExit("ghost_synth_walk: need >= 5 steps (extracted cycle is steps 3+4, plus one "
                         "interior step after it for the periodicity witness)")

    terr = {"type": "flat", "z": 0.0}
    start, steps = plan_walk(a.stride, a.n_steps)
    clears = [max(a.clear, GS.needed_apex(terr, frm, to, FOOT_HALF, a.min_gap)) for (_f, frm, to) in steps]
    worst = min(GS.check_arc_clearance(terr, frm, to, c, FOOT_HALF)
                for (_f, frm, to), c in zip(steps, clears))

    nb_full, tgt, swing_s, sched = GS.build_timeline(start, steps, a.settle_s, a.ds_s, a.ss_s,
                                                     a.top_s, terr, clears)
    patch_x = float(GC.load_spec("feet")["contacts"][0]["patch"][0])
    com_tg, base_z = walk_anchor(nb_full, tgt, sched, patch_x, a.ride, a.k_ratio, a.com_inset)
    wb, root, ferr, cerr, unreach, m, d, spec = solve_walk(nb_full, tgt, swing_s, sched, com_tg, base_z,
                                                           terr, a.arm_A, a.elbow, a.shroll, a.mjcf)

    # ---- extract the steady-state cycle: steps 3+4 ------------------------------------------------
    ds_b, ss_b = int(round(a.ds_s / DT)), int(round(a.ss_s / DT))
    step_b = ds_b + ss_b
    settle_b = int(round(a.settle_s / DT))
    i0 = settle_b + 2 * step_b          # first frame of step 3's double support (just after touchdown)
    nbc = 2 * step_b                    # one L+R cycle
    if i0 + nbc >= nb_full:
        raise SystemExit("ghost_synth_walk: timeline too short for interior extraction (internal error)")
    travel = 2.0 * a.stride
    cyc = nbc * DT

    # periodicity of the SOLVE (before any correction): the witness that steady state was reached
    per_wb = float(np.abs(wb[i0 + nbc] - wb[i0]).max())
    per_root = root[i0 + nbc] - root[i0] - np.array([travel, 0.0, 0.0, 0.0])
    per_root_mm = 1000.0 * float(np.abs(per_root[0:3]).max())

    wbc = wb[i0:i0 + nbc].copy()
    rc = root[i0:i0 + nbc].copy()
    rc[:, 0] -= root[i0, 0]
    schedc = [list(s) for s in sched[i0:i0 + nbc]]
    # morph the residual periodicity error out along the cycle: frame nbc == frame 0 + [travel,0,0,0]
    ramp = (np.arange(nbc, dtype=float) / nbc)[:, None]
    wbc += ramp * (wb[i0] - wb[i0 + nbc])[None, :]
    rc += ramp * (-per_root)[None, :]

    # the task-level cyclic checks
    seam_wb = float(np.abs(wbc[0] - wbc[-1]).max())        # one bin apart, at the quiet seam
    fe = ferr[i0:i0 + nbc]
    ce = cerr[i0:i0 + nbc]
    unreach_c = [(i, e) for (i, e) in unreach if i0 <= i < i0 + nbc]

    # ---- 2-cycle FK seam test: the stance foot planted across the seam must not jump --------------
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in GS.WB_JOINTS}

    def _sole(fname, i, xoff):
        c = {"foot_L": spec["contacts"][0], "foot_R": spec["contacts"][1]}[fname]
        d.qpos[0:3] = rc[i, 0:3] + np.array([xoff, 0.0, 0.0])
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for k, n in enumerate(GS.WB_JOINTS):
            d.qpos[qadr[n]] = wbc[i, k]
        mj.mj_kinematics(m, d)
        R = d.xmat[c["bid"]].reshape(3, 3)
        return d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)

    seam_foot = [f for f in ("foot_L", "foot_R") if f in schedc[-1]]
    seam_jump = max(float(np.linalg.norm(_sole(f, 0, travel) - _sole(f, nbc - 1, 0.0)))
                    for f in seam_foot)

    # ---- style channels ----------------------------------------------------------------------------
    hip_amp = 0.5 * (wbc[:, 0].max() - wbc[:, 0].min())
    arm_amp = 0.5 * (wbc[:, 13].max() - wbc[:, 13].min())
    vx = travel / cyc

    print("GHOST SYNTH WALK  cyclic straight-line, flat ground")
    print("  stride %.3f m/step  ds %.3f s (%d bins)  ss %.3f s (%d bins)  ->  cycle %.3f s  vx %.3f m/s"
          % (a.stride, ds_b * DT, ds_b, ss_b * DT, ss_b, cyc, vx))
    print("  laid out %d steps (%d frames); extracted steps 3+4 -> frames [%d, %d), nb=%d"
          % (len(steps), nb_full, i0, i0 + nbc, nbc))
    print("  swing apex %.0f mm; worst sole-to-ground gap along any arc %.1f mm" % (1000 * clears[0], 1000 * worst))
    print("  foot-target residual (cycle)  mean %.3f mm  max %.3f mm   unreachable frames %d/%d"
          % (1000 * fe.mean(), 1000 * fe.max(), len(unreach_c), nbc))
    print("  COM-to-target (gate 2, cycle) mean %.2f mm  max %.2f mm %s"
          % (1000 * ce.mean(), 1000 * ce.max(),
             "<-- the COM row lost to a joint limit" if ce.max() > 5e-3 else ""))
    if unreach_c:
        i, e = max(unreach_c, key=lambda t: t[1])
        print("  WORST frame %d short by %.1f mm -- lower --ride or shorten --stride; do not ignore"
              % (i, 1000 * e))
    print("  PERIODICITY of the solve  joints %.2e rad  base %.3f mm  (morphed out along the cycle)"
          % (per_wb, per_root_mm))
    print("  SEAM  wb_lut[0] vs wb_lut[-1] max |dq| = %.2e rad  %s  (must be < 1e-3)"
          % (seam_wb, "PASS" if seam_wb < 1e-3 else "FAIL"))
    print("  SEAM  2-cycle FK: planted foot (%s) jump across the seam = %.3f mm  %s"
          % ("/".join(seam_foot), 1000 * seam_jump, "PASS" if seam_jump < 1e-3 else "FAIL"))
    # smoothness witness: a reference must not command an impulsive joint step anywhere in the cycle
    stepmax = np.abs(np.diff(np.vstack([wbc, wbc[0][None, :]]), axis=0)).max(axis=0)  # incl. the wrap
    kws = int(np.argmax(stepmax))
    print("  SMOOTH worst one-bin joint step %.4f rad (%.1f rad/s) at %s"
          % (stepmax[kws], stepmax[kws] / DT, GS.WB_JOINTS[kws]))
    print("  base  z %.3f..%.3f   y %+.3f..%+.3f   travel/cycle %.3f m (declared %.3f)"
          % (rc[:, 2].min(), rc[:, 2].max(), rc[:, 1].min(), rc[:, 1].max(),
             rc[-1, 0] - rc[0, 0] + travel / nbc, travel))
    print("  STYLE  hip swing +/-%.1f deg   cadence %.2f steps/s (cycle %.2f Hz)   arm swing +/-%.1f deg (elbow %.2f)"
          % (math.degrees(hip_amp), 2.0 / cyc, 1.0 / cyc, math.degrees(arm_amp), a.elbow))

    out = {
        "nb": nbc, "freq": 1.0 / cyc, "cycle_s": cyc, "vx": vx,
        "arm_A": float(a.arm_A), "shroll": float(a.shroll),
        "stride": float(a.stride), "ds_s": ds_b * DT, "ss_s": ss_b * DT, "ride": float(a.ride),
        "leg_lut": [[float(v) for v in r[0:12]] for r in wbc],
        "arm_lut": [[float(r[13]), float(r[18])] for r in wbc],
        "elbow_lut": [[float(a.elbow), float(a.elbow)] for _ in range(nbc)],
        "att_lut": [[0.0, 0.0] for _ in range(nbc)],
        "root_lut": [[float(v) for v in r] for r in rc],
        "wb_joints": GS.WB_JOINTS, "wb_lut": [[float(v) for v in r] for r in wbc],
        "contact_schedule": [sorted(s) for s in schedc],
        "terrain": terr, "ghost_schema": 2, "ghost_closed": True,
        "source": ("SYNTHESIZED by ghost_synth_walk.py (plan the contacts, solve everything else): "
                   "cyclic 2-step walk, stride %.3f m, ds %.3f s, ss %.3f s, vx %.3f m/s; base (x,y,z) "
                   "SOLVED per frame with the leg joints so the COM rides over the bearing ankle "
                   "(single support) / quintic-transfers (double support); steady-state cycle extracted "
                   "from a %d-step solve (periodicity %.1e rad, morphed exact); foot residual max "
                   "%.2f mm, COM-to-target max %.2f mm, seam jump %.3f mm. Gates 1+2 by construction; "
                   "gate 3 is ghost_dynamics' to judge."
                   % (a.stride, ds_b * DT, ss_b * DT, vx, len(steps), per_wb,
                      1000 * fe.max(), 1000 * ce.max(), 1000 * seam_jump)),
    }
    json.dump(out, open(a.out, "w"))
    print("  wrote %s" % a.out)
    print("  NEXT: build_keypoints.py %s --links --elbow %.2f --shroll %.2f ; ghost_dynamics.py %s --smooth 7 --stride 4"
          % (a.out, a.elbow, a.shroll, a.out))


if __name__ == "__main__":
    main()
