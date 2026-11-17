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

"""ghost_synth.py -- SYNTHESIZE a ghost that passes all three gates BY CONSTRUCTION.

THE METHOD, IN ONE SENTENCE
Plan the CONTACTS. Solve everything else.

Every ghost builder we have ever written does the opposite: it writes the base as a formula in the foot
positions (`pelvis_x = 0.5*(xL+xR)`, `pelvis_z = min(zL,zR) + ride`, `pelvis_y = 0`) and then fills the
joints in with a closed-form leg IK. That over-determines the motion, and the residual comes out as
LEVITATION: on ghost_stair_climb_lut.json the "planted" foot drifts 62 mm per stance, only 55% of the
declared 0.35 m climb is produced by the legs, and the reference is airborne on 638 of 1112 frames.
A tracker reproduces the joints -- which are achievable -- and the base goes wherever physics sends it.

THE THREE GATES (ghost_dynamics.py enforces them; this file makes them true from the start)
  GATE 1  CLOSURE   a planted contact does not move.
  GATE 2  SUPPORT   the COM stays inside the support hull.
  GATE 3  FEASIBLE  the contacts can supply the wrench the motion demands.

HOW EACH ONE IS MADE TRUE HERE
  1. Footholds are world poses. Every stance frame targets the SAME world pose, and the pose is hit by
     numerical IK on the real forward kinematics (mj_jac), not by a 2-link analytic chain that places the
     ankle joint and then assumes a constant drop to the sole. That assumed drop is what broke the stair
     ghost: its true value ranges over 10..64 mm because the G1's contact patch sits 35 mm AHEAD of the
     ankle and swings on a lever. The old builder printed "FK-feasible by construction" while checking
     the analytic chain. Closure is now checked against the thing that actually moves the robot.
  2. The BASE IS AN OUTPUT. Per frame we solve (base x, y, z) together with the leg joints so that the
     COM lands on a target that rides over the bearing foot. Nothing is a formula. Measured: the same
     posture with the pelvis pinned to y=0 -- the shipped rule -- goes from 0.0 N of external help to
     10.7 N*m of crane.
  3. Footholds are placed with NOSING CLEARANCE, and swing arcs are lifted to clear the terrain ALONG
     THE ARC, not just at their endpoints. The old plan put the toe exactly on the first riser's face
     (zero clearance), and sent each foot over an intermediate tread with `--clear` measured against the
     endpoints, so the swing skimmed it within a centimetre.

WHAT THIS DOES NOT PROMISE
Gate 3 depends on TIMING, which is a free parameter here. A plan that is statically fine can still demand
a wrench the contacts cannot supply if it is executed too fast. Synthesize, then run ghost_dynamics; if
[FWP] fails, slow the transfer (--ds-s) rather than move the feet. ghost_topp.py re-times without
changing the path.

Usage:
  python ghost_synth.py out.json --gait stairs --terrain 'stairs:riser=0.07,going=0.26,x0=0.12,n=5' \
      --ride 0.72 --ds-s 0.8 --ss-s 0.7
  python build_keypoints.py out.json --links      # ruler tables
  python ghost_dynamics.py out.json               # the verdict
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ghost_contacts as GC
import ghost_close as GCL

RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEF_MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")

WB_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
]
DT = 0.016                 # sequence bin == trainer tick
FOOT_HALF = 0.085          # sole half-length; the nosing clearance is measured from the toe/heel
STANCE_Y = 0.1185          # lateral foothold offset: where the G1's soles naturally sit


def quintic(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


# ---------------------------------------------------------------------------------------------------
# FOOTSTEP PLAN. A list of (foot, foothold_xyz) in order, plus the timeline that walks through it.
# ---------------------------------------------------------------------------------------------------

def plan_stairs(terr, nose_clear, lead="L"):
    """Step-to gait: both feet visit every tread. Footholds are tread CENTRES, clamped so neither the
    toe nor the heel overhangs a nosing."""
    r, g, x0, n = terr["riser"], terr["going"], terr["x0"], int(terr["n"])

    def foothold_x(k):
        if k <= 0:
            return x0 - FOOT_HALF - nose_clear                     # floor, toe short of the first riser
        lo = x0 + (k - 1) * g + FOOT_HALF + nose_clear             # heel clear of THIS tread's nosing
        # the top tread and the landing beyond it are the same height, so tread n has no forward nosing
        hi = (x0 + k * g - FOOT_HALF - nose_clear) if k < n else 1e9
        if lo > hi:
            raise SystemExit("ghost_synth: tread %.3f m cannot hold a %.3f m foot with %.3f m clearance"
                             % (g, 2 * FOOT_HALF, nose_clear))
        return min(max(x0 + (k - 0.5) * g, lo), hi)                # centre of the tread, clamped

    def fh(foot, k):
        return np.array([foothold_x(k), (+STANCE_Y if foot == "L" else -STANCE_Y), min(k, n) * r])

    start = {"L": fh("L", 0), "R": fh("R", 0)}
    steps = []                                                     # (foot, from, to)
    cur = dict(start)
    ld = lead
    for k in range(1, n + 1):
        other = "R" if ld == "L" else "L"
        for f in (ld, other):
            nxt = fh(f, k)
            steps.append((f, cur[f].copy(), nxt.copy()))
            cur[f] = nxt
        ld = other
    return start, steps


def swing_arc(terr, a, b, s, clear, foot_half):
    """Position of a swinging contact at phase s.

    ⛔ The bump must be C2 at the ends. A `sin(pi*s)**0.5` lift has INFINITE velocity at s=0: the foot
    leaves the ground with an impulsive acceleration, the reference demands an impulsive moment, and the
    FWP rightly refuses it. `sin(pi*s)**3` vanishes to third order at both ends.
    ⛔ And `max(z, terrain)` clamping introduces a kink wherever it binds. Clearance is CHECKED along the
    arc (check_arc_clearance) and fixed by raising the apex, never by clamping the path.
    """
    x = a[0] + (b[0] - a[0]) * quintic(s)
    y = a[1] + (b[1] - a[1]) * quintic(s)
    z = a[2] + (b[2] - a[2]) * quintic(s) + clear * (math.sin(math.pi * min(max(s, 0.0), 1.0)) ** 3)
    return np.array([x, y, z])


def _under(terr, x, y, foot_half):
    """Highest terrain anywhere beneath the SOLE's footprint -- heel, centre and toe, not just centre."""
    return max(GC.surface_z(terr, x + o, y) for o in (-foot_half, 0.0, +foot_half))


def needed_apex(terr, a, b, foot_half, min_gap, n=400):
    """The smallest apex for which the whole sole clears the terrain by `min_gap * bump(s)` everywhere.

    ⛔ The margin must SCALE WITH THE BUMP. Demanding a flat `min_gap` at every phase is nonsense near
    the ends, where the foot is resting on its foothold and bump(s) -> 0: requiring 20 mm of gap at
    s = 0.12, where the bump is 5% of the apex, forced a 456 mm apex and pushed the knee past its speed
    limit. Solve for the apex instead of iterating a clamp:

        z(s) = lin(s) + A * bump(s)   must satisfy   z(s) - under(s) >= min_gap * bump(s)
        =>   A >= max_s [ (under(s) - lin(s)) / bump(s) ] + min_gap

    Where the terrain is below the straight line, the term is negative and imposes nothing. Where the
    toe would catch a nosing, it is exactly the lift that clears it.
    """
    need = 0.0
    for i in range(1, n):
        s = i / float(n)
        bump = math.sin(math.pi * s) ** 3
        if bump < 1e-3:
            continue
        x = a[0] + (b[0] - a[0]) * quintic(s)
        y = a[1] + (b[1] - a[1]) * quintic(s)
        lin = a[2] + (b[2] - a[2]) * quintic(s)
        need = max(need, (_under(terr, x, y, foot_half) - lin) / bump)
    return need + min_gap


def check_arc_clearance(terr, a, b, clear, foot_half, n=256):
    """Smallest gap between the swinging sole and the terrain beneath it, over the whole arc. Reported,
    not enforced: the apex is chosen by needed_apex, and this is the independent witness that it worked."""
    worst = 1e9
    for i in range(1, n):
        s = i / float(n)
        p = swing_arc(terr, a, b, s, clear, foot_half)
        worst = min(worst, float(p[2] - _under(terr, p[0], p[1], foot_half)))
    return worst


def build_timeline(start, steps, settle_s, ds_s, ss_s, top_s, terr, clears):
    """Frames -> (foot targets, contact schedule). Double support transfers weight; single support swings.
    The schedule is DECLARED here, not inferred later from geometry."""
    nb = int(round((settle_s + len(steps) * (ds_s + ss_s) + top_s) / DT))
    tgt = {"L": np.zeros((nb, 3)), "R": np.zeros((nb, 3))}
    swing_s = {"L": np.full(nb, np.nan), "R": np.full(nb, np.nan)}   # swing phase, NaN while bearing
    sched = [[] for _ in range(nb)]
    cur = {k: v.copy() for k, v in start.items()}
    i = 0

    def emit(nfr, swinging=None, frm=None, to=None, clear=0.0):
        nonlocal i
        for j in range(nfr):
            if i >= nb:
                return
            for f in ("L", "R"):
                if f == swinging:
                    s = (j + 0.5) / nfr
                    tgt[f][i] = swing_arc(terr, frm, to, s, clear, FOOT_HALF)
                    swing_s[f][i] = s
                else:
                    tgt[f][i] = cur[f]
                    sched[i].append("foot_" + f)
            i += 1

    emit(int(round(settle_s / DT)))
    # ⛔ PER-STEP apex, not one global maximum. Only the first step -- off the floor, toe 30 mm behind
    # the first nosing -- needs a 169 mm lift. Applying that to all ten steps throws the other nine legs
    # 100 mm higher than they need to go, and the swing leg's momentum is exactly what the stance foot
    # then has to fight (foot-spin torque was the last constraint standing).
    for (f, frm, to), clear in zip(steps, clears):
        emit(int(round(ds_s / DT)))                    # both feet down: shift the weight
        emit(int(round(ss_s / DT)), swinging=f, frm=frm, to=to, clear=clear)
        cur[f] = to.copy()
    emit(nb - i)                                       # top landing hold
    for k in range(nb):
        sched[k] = sorted(sched[k])
    return nb, tgt, swing_s, sched


def support_anchor(nb, tgt, sched, patch_x, ride):
    """The single object every base quantity is derived from: at each frame, the (x, y, z) the support
    demands. Single support -> over the bearing foot. Double support -> a quintic transfer between the
    outgoing and incoming bearing feet, which stays inside the two-foot hull the whole way.

    Returns (com_xy, base_z). Both are interpolations of the SAME anchor, so the base climbs while the
    weight transfers -- during double support, where a rising pelvis is supported by two feet.

    ⛔ Do NOT write base_z = min(zL, zR) + ride. That is the shipped stair ghost's formula, and it makes
    the pelvis step up by a whole riser the instant the swing foot touches down -- an impulsive vertical
    acceleration on a reference that is supposed to be quasi-static. I reintroduced it here by accident
    and the FWP caught it: 27 frames where no contact force distribution could supply the demanded wrench.

    ⛔ The COM rides over the ANKLE, not over the sole's centre. The G1's contact patch sits 35 mm ahead
    of the ankle, so a COM over the patch centre hands the ankle m*g*0.035 = 11.7 N*m of standing torque
    for nothing. ankle_pitch was the first joint to blow its limit (1.12x).
    """
    anchor = np.full((nb, 3), np.nan)
    for i in range(nb):
        if len(sched[i]) == 1:
            f = sched[i][0][-1]
            anchor[i] = [tgt[f][i][0] - patch_x, tgt[f][i][1], tgt[f][i][2] + ride]

    def _both(i):
        """Neutral anchor when both feet bear: between them. This is where a motion must START and
        END. ⛔ Clamping the head/tail to the nearest single-support anchor instead put frame 0's
        pelvis 0.143 m over the first stance foot -- but the trainer SPAWNS the robot centred, so the
        reference would open with an un-tracked lateral lunge before the motion even begins."""
        fx = [tgt[f][i][0] - patch_x for f in ("L", "R")]
        fy = [tgt[f][i][1] for f in ("L", "R")]
        fz = [tgt[f][i][2] for f in ("L", "R")]
        return np.array([np.mean(fx), np.mean(fy), min(fz) + ride])

    idx = np.where(~np.isnan(anchor[:, 0]))[0]
    if len(idx) == 0:
        raise SystemExit("ghost_synth: no single-support frame -- nothing defines the base path")
    i0, i1 = idx[0], idx[-1]
    if i0 > 0:
        anchor[0] = _both(0)                          # settle starts neutral, shifts across the head
    if i1 < nb - 1:
        anchor[nb - 1] = _both(nb - 1)                # final stand returns to neutral
    idx = np.where(~np.isnan(anchor[:, 0]))[0]
    out = np.zeros((nb, 3))
    for i in range(nb):
        if not np.isnan(anchor[i, 0]):
            out[i] = anchor[i]; continue
        prv = idx[idx < i]; nxt = idx[idx > i]
        if len(prv) == 0:
            out[i] = anchor[nxt[0]]
        elif len(nxt) == 0:
            out[i] = anchor[prv[-1]]
        else:
            a, b = prv[-1], nxt[0]
            out[i] = anchor[a] + (anchor[b] - anchor[a]) * quintic((i - a) / float(b - a))
    return out[:, 0:2], out[:, 2]


# ---------------------------------------------------------------------------------------------------
# THE SOLVE. Per frame: base (x,y,z) + 12 leg joints, so both soles hit their targets flat and the COM
# lands on its own. 15 unknowns, 15 residuals. Warm-started, so the solution is continuous.
# ---------------------------------------------------------------------------------------------------

W_FOOT, W_ORI, W_COM, W_BZ, W_REG = 10.0, 4.0, 2.0, 1.0, 0.02

# Which orientation rows a foot gets. Angular rows are about world x (roll), y (pitch), z (yaw).
#   BEARING foot:  all three. Roll and pitch lay the sole flat -- that is what makes the support a
#                  rectangle instead of an edge -- and yaw pins the heading.
#   SWINGING foot: pitch and yaw only.
#
# ⛔ Insisting a SWINGING foot stay level in ROLL saturates ankle_roll (+/-0.262 rad; the G1 needs
# ~0.37 rad to hold a flat sole while its leg reaches 0.26 m across the body). Once a joint clips, the
# least-squares is inconsistent and the lowest-weight row loses -- which was the COM row, the whole
# point of the solve: 28 mm of COM error, and the support margin fell from 30 mm to 3 mm.
# ⛔ The answer is NOT to down-weight the whole orientation task. At weight 0.2 the swing leg's ankle
# roll became a free direction, the solve wandered through its own nullspace, and the foot missed its
# foothold by 37 mm on 240 frames. Delete exactly the row that cannot be satisfied, keep the rest at
# full strength, and add a light posture regulariser (W_REG) toward the previous frame for conditioning.
STANCE_ROT_ROWS = (0, 1, 2)
SWING_ROT_ROWS = (1, 2)


def solve(nb, tgt, swing_s, sched, com_tg, base_z, terr, arm_A, elbow, shroll, mjcf=None, iters=120):
    m = mj.MjModel.from_xml_path(mjcf or DEF_MJCF)
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
            rr = STANCE_ROT_ROWS if math.isnan(float(swing_s[f][i])) else SWING_ROT_ROWS
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
            rr = np.zeros((len(legwb), len(cols)))
            for k2, cc in enumerate(legcols):
                rr[k2, cc] = W_REG
            rows.append(rr)
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
        _fk(p, q)  # noqa: F841  (leaves d at the accepted solution for the residual read-out below)
        e = max(float(np.linalg.norm((d.xpos[byfoot[f]["bid"]]
                                      + d.xmat[byfoot[f]["bid"]].reshape(3, 3) @ np.asarray(byfoot[f]["patch"], float))
                                     - tgt[f][i])) for f in ("L", "R"))
        ferr.append(e)
        # GATE 2's OWN RESIDUAL. Report it: the foot rows are weighted 10 and the COM row 1, so whenever
        # the system is inconsistent (a joint on its limit) it is the COM that quietly gets sacrificed --
        # and the COM is the whole point. Five mid-swing frames sat 27 mm off target and dropped the
        # support margin from 30 mm to 3 mm before this line existed.
        cerr.append(float(np.linalg.norm(d.subtree_com[0][0:2] - com_tg[i])))
        if e > 2e-3:
            unreach.append((i, e))
        wb[i] = q
        root[i] = [p[0], p[1], p[2], 0.0]
    return wb, root, np.array(ferr), np.array(cerr), unreach, m, d, spec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--gait", default="stairs", choices=["stairs"])
    ap.add_argument("--terrain", default="stairs:riser=0.07,going=0.26,x0=0.12,n=5")
    ap.add_argument("--ride", type=float, default=0.72, help="pelvis height above the bearing foot (m)")
    ap.add_argument("--ds-s", type=float, default=0.8, help="double-support weight transfer (s)")
    ap.add_argument("--ss-s", type=float, default=0.7, help="swing (s)")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--top-s", type=float, default=2.0)
    ap.add_argument("--clear", type=float, default=0.06, help="nominal swing apex above the endpoints (m)")
    ap.add_argument("--min-gap", type=float, default=0.020,
                    help="required mid-swing gap between the whole SOLE and the terrain under it (m); "
                         "the apex is raised until this holds on every arc")
    ap.add_argument("--nose-clear", type=float, default=0.030, help="toe/heel margin from a nosing (m)")
    ap.add_argument("--arm-A", type=float, default=0.35)
    ap.add_argument("--elbow", type=float, default=1.2)
    ap.add_argument("--shroll", type=float, default=0.15)
    ap.add_argument("--mjcf", default=None)
    a = ap.parse_args()

    terr = GC.make_terrain(a.terrain)
    start, steps = plan_stairs(terr, a.nose_clear)

    # the swing must clear the terrain under the WHOLE SOLE. Solve for each step's apex; never clamp the
    # arc onto the terrain (a clamp is a kink is an impulse). `worst` is the independent witness.
    clears = [max(a.clear, needed_apex(terr, frm, to, FOOT_HALF, a.min_gap)) for (_f, frm, to) in steps]
    worst = min(check_arc_clearance(terr, frm, to, c, FOOT_HALF) for (_f, frm, to), c in zip(steps, clears))

    nb, tgt, swing_s, sched = build_timeline(start, steps, a.settle_s, a.ds_s, a.ss_s, a.top_s, terr, clears)
    patch_x = float(GC.load_spec("feet")["contacts"][0]["patch"][0])
    com_tg, base_z = support_anchor(nb, tgt, sched, patch_x, a.ride)
    wb, root, ferr, cerr, unreach, m, d, spec = solve(nb, tgt, swing_s, sched, com_tg, base_z, terr,
                                                      a.arm_A, a.elbow, a.shroll, a.mjcf)

    print("GHOST SYNTH  gait=%s  terrain=%s" % (a.gait, json.dumps(terr)))
    print("  %d footsteps, nb=%d, %.2f s   (settle %.1f + %d x (%.1f ds + %.1f ss) + %.1f hold)"
          % (len(steps), nb, nb * DT, a.settle_s, len(steps), a.ds_s, a.ss_s, a.top_s))
    print("  swing apex per step %s mm (asked %.0f); worst sole-to-terrain gap along any arc %.1f mm"
          % ("/".join("%.0f" % (1000 * c) for c in clears), 1000 * a.clear, 1000 * worst))
    print("  foot-target residual  mean %.3f mm  max %.3f mm   unreachable frames %d/%d"
          % (1000 * ferr.mean(), 1000 * ferr.max(), len(unreach), nb))
    print("  COM-to-target (gate 2)  mean %.2f mm  max %.2f mm %s"
          % (1000 * cerr.mean(), 1000 * cerr.max(),
             "<-- the COM row lost to a joint limit; the support margin pays for it" if cerr.max() > 5e-3 else ""))
    if unreach:
        i, e = max(unreach, key=lambda t: t[1])
        print("  WORST frame %d short by %.1f mm -- the plan asks for a foothold the leg cannot reach with"
              % (i, 1000 * e))
        print("  the pelvis where the COM requires. Lower --ride, or shorten the step. Do not ignore this:")
        print("  the old analytic IK 'solved' exactly these frames by clamping and letting the foot float.")
    print("  base  z %.3f..%.3f   y %+.3f..%+.3f   climb %+.3f m   travel %+.3f m"
          % (root[:, 2].min(), root[:, 2].max(), root[:, 1].min(), root[:, 1].max(),
             root[-1, 2] - root[0, 2], root[-1, 0] - root[0, 0]))

    cyc = nb * DT
    out = {
        "nb": nb, "freq": 1.0 / cyc, "cycle_s": cyc, "vx": 0.0, "seq": True, "hold_end": True,
        "arm_A": float(a.arm_A), "shroll": float(a.shroll),
        "leg_lut": [[float(v) for v in r[0:12]] for r in wb],
        "arm_lut": [[float(r[13]), float(r[18])] for r in wb],
        "elbow_lut": [[float(a.elbow), float(a.elbow)] for _ in range(nb)],
        "att_lut": [[0.0, 0.0] for _ in range(nb)],
        "root_lut": [[float(v) for v in r] for r in root],
        "wb_joints": WB_JOINTS, "wb_lut": [[float(v) for v in r] for r in wb],
        "contact_schedule": sched, "terrain": terr, "ghost_schema": 2, "ghost_closed": True,
        "source": ("SYNTHESIZED by ghost_synth.py: footholds planned on the terrain with %.0f mm nosing "
                   "clearance; swing arcs clear the terrain UNDER the arc by %.0f mm; base (x,y,z) SOLVED "
                   "per frame together with the leg joints so the COM rides over the bearing foot; foot "
                   "poses hit by numerical IK on the real forward kinematics (max residual %.2f mm, "
                   "COM-to-target %.2f mm). Gates 1 and 2 hold by construction; gate 3 is "
                   "ghost_dynamics' to judge."
                   % (1000 * a.nose_clear, 1000 * min(clears), 1000 * ferr.max(), 1000 * cerr.max())),
    }
    json.dump(out, open(a.out, "w"))
    print("  wrote %s" % a.out)
    print("  NEXT: build_keypoints.py %s --links --elbow %.2f --shroll %.2f ; ghost_dynamics.py %s"
          % (a.out, a.elbow, a.shroll, a.out))


if __name__ == "__main__":
    main()
