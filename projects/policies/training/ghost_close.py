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

"""ghost_close.py -- CLOSE a ghost's kinematic loop: make its feet actually rest where it says.

THE DEFECT THIS EXISTS TO KILL
Every ghost builder we have follows the same recipe: plan footholds, prescribe a pelvis path, and fill
in the joints with a closed-form leg IK. The IK places the ANKLE JOINT and then adds a constant offset
down to the sole. That offset is not constant. The G1's contact patch sits 35 mm AHEAD of and 36 mm
BELOW `*_ankle_roll_link`, so it swings on a lever as the ankle pitches: measured on the stair ghost,
the true ankle-to-sole drop ranges over 10..64 mm. The plan pins the stance foot; the forward
kinematics does not. Nobody checks, because everyone checks the plan.

Consequences, measured on ghost_stair_climb_lut.json (riser 0.07, 5 steps):
    declared base climb, root_lut ................ +0.3500 m
    upward drift of the STANCE sole ..............  +0.3730 m   (107% of the climb)
    base climb recovered from joints + contacts ... +0.0012 m   (ghost_root, never reads root_lut)
i.e. the reference is a FLAT WALK with a fictional pelvis ramp bolted on. An RL tracker reproduces the
joints -- which are achievable -- and the base goes wherever physics says, which is nowhere. That is
exactly the observed failure: joint match 0.95, distance travelled 0.18, "the feet just shuffle".

THE FIX, AND THE RULE
    A ghost's root_lut must equal the base implied by its own wb_lut and contact schedule.
Nothing else needs to be trusted -- not the IK, not the plan, not the provenance ("recorded", "solved").
The check is one call to ghost_root.reconstruct. When it fails, this module repairs it:

    SNAP    each stance segment's foothold = where the contact FIRST lands, projected onto the terrain
    PIN     hold that world pose for the whole segment
    SOLVE   Gauss-Newton on the joints of the kinematic path base->contact, using the REAL Jacobian,
            until the contact patch sits on its foothold with the sole flat to the surface

The solver cannot cheat. Where the analytic IK silently clamped and levitated the foot, this reports a
residual: the leg is too short, so the ghost as designed is not merely dynamically hard, it is
KINEMATICALLY IMPOSSIBLE. Lower the pelvis or shorten the step -- but do it knowingly.

Footholds are taken from the ghost's own first touch, not from a foothold plan, so this works on any
ghost: walk, stairs, crawl (hands + knees), carry. The contact bodies and the terrain come from the
same spec ghost_dynamics uses.

WHAT CLOSURE DOES *NOT* FIX -- both found on design_stair_climb_ghost.py, both the FOOTHOLD PLAN's fault
  1. ZERO NOSING CLEARANCE. `tread_x(0) = x_start - 0.12` puts the toe exactly on the first riser's
     face, so any placement error at all drives the toe into the step. Measured: 60-70 mm of the foot
     buried inside tread 1. Closure snaps the patch CENTRE onto the surface under it and cannot help a
     foot that straddles a nosing. Plan the footholds with a clearance; ghost_dynamics reports the
     penetration under [TERRAIN].
  2. THE SWING SKIMS THE INTERMEDIATE TREAD. The step-to gait sends each foot up TWO risers per step
     (L: treads 1->3->5), so `--clear 0.07` above the HIGHER tread leaves almost nothing above the one
     in between -- the foot passes within a centimetre of it. Clearance must be measured against the
     terrain along the whole swing arc, not against the endpoints.

AND WHAT CLOSURE CANNOT FIX AT ALL: a base that was never solved. After closure the stair ghost is a
real climb -- feet pinned to 0.2 mm, 101% of its declared climb recovered from its joints alone -- and
still demands 62 N of external help, because `pelvis_y = 0` leaves the COM 88.5 mm outside the stance
foot on every single-support frame. Closure is gate 1. See ghost_dynamics.py's header for gates 2 and 3.

Usage:
  python ghost_close.py <in_lut.json> <out_lut.json> [--contacts feet] [--terrain 'stairs:riser=..,..']
                        [--iters 40] [--report]
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

RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEF_MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def path_joints(model, bid):
    """Every joint on the kinematic path base -> body `bid`. Derived from the model, so this is right
    for feet, and equally right for the crawl ghost's hands and knees."""
    out = []
    b = bid
    while b > 0:
        for j in range(model.body_jntadr[b], model.body_jntadr[b] + model.body_jntnum[b]):
            if model.jnt_type[j] not in (mj.mjtJoint.mjJNT_FREE,):
                out.append(j)
        b = model.body_parentid[b]
    return sorted(out)


def _sole_pose(data, bid, patch):
    R = data.xmat[bid].reshape(3, 3)
    return data.xpos[bid] + R @ np.asarray(patch, float), R


def _rot_err_xy(R_sole, n_surf):
    """Roll/pitch error that would lay the sole flat on the surface. Yaw is left free."""
    n_now = R_sole @ np.array([0.0, 0.0, 1.0])
    e = np.cross(n_now, np.asarray(n_surf, float))     # axis * sin(angle), world frame
    return e[0:2]


def close(lut, spec, terr, mjcf=None, iters=40, v_tol=0.05, h_tol=0.05, damp=1e-3, verbose=False,
          allow_flight=False, targets=None, schedule=None):
    """Return (wb_closed, report). `wb_closed` differs from lut['wb_lut'] only on stance frames.

    Pass `targets` {(contact_name, frame): (world_pos, surface_normal)} and `schedule` to pin GIVEN
    footholds instead of the ones this ghost happens to land on. That is what lets you change the base
    trajectory -- add a lateral weight shift, lower the ride height -- and re-solve the legs against the
    footholds you already committed to, rather than against wherever the moved base drags the feet.
    """
    m = mj.MjModel.from_xml_path(mjcf or DEF_MJCF)
    d = mj.MjData(m)
    NB = int(lut["nb"])
    wb = np.array(lut["wb_lut"], float)
    WBJ = list(lut["wb_joints"])
    root = np.array(lut["root_lut"], float)
    att = np.asarray(lut["att_lut"], float) if "att_lut" in lut else np.zeros((NB, 2))
    bp = float(lut.get("base_pitch", 0.0))
    dt = float(lut.get("cycle_s", 1.0)) / NB

    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ}
    for c in spec["contacts"]:
        c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
        c.setdefault("patch", [0.0, 0.0, 0.0])
        # the joints this contact can actually be moved by, straight out of the model
        jids = [j for j in path_joints(m, c["bid"]) if mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) in WBJ]
        c["jids"] = jids
        c["jwb"] = [WBJ.index(mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j)) for j in jids]
        c["jdof"] = [m.jnt_dofadr[j] for j in jids]
        c["jlo"] = np.array([m.jnt_range[j][0] for j in jids])
        c["jhi"] = np.array([m.jnt_range[j][1] for j in jids])

    def _set(i, q=None):
        d.qpos[0:3] = root[i, 0:3]
        d.qpos[3:7] = _quat(float(att[i, 0]), float(att[i, 1]) + bp, float(root[i, 3]))
        qq = wb[i] if q is None else q
        for k, n in enumerate(WBJ):
            d.qpos[qadr[n]] = qq[k]
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)      # mj_jac reads d.cdof; mj_kinematics alone leaves it stale (J == 0,
                                # the Gauss-Newton step is 0, and the solver silently does nothing)

    # ---- pass 1: where does each contact go, as the ghost is written? -------------------------
    sole = {c["name"]: np.zeros((NB, 3)) for c in spec["contacts"]}
    for i in range(NB):
        _set(i)
        for c in spec["contacts"]:
            sole[c["name"]][i] = _sole_pose(d, c["bid"], c["patch"])[0]

    # ---- pass 2: stance segments. A contact is INTENDED to bear when it is low and slow. --------
    # Deliberately generous: the whole point is that a broken ghost's stance foot floats and creeps, so
    # a tight tolerance would classify the defect away and declare the motion "all flight".
    # ⛔ v_tol must clear the STANCE CREEP (tens of mm/s on a broken ghost) while still rejecting a swing
    # (hundreds of mm/s). At 0.02 m/s the schedule lost 203 frames -- in runs of 0.78 s -- exactly where
    # the levitation was fastest; the base recovery then coasted across the holes and read 151% of the
    # true climb. Empty frames are REPORTED below, never silently tolerated.
    byname = {c["name"]: c for c in spec["contacts"]}
    segs = []           # (name, i0, i1) inclusive
    forced = 0
    if targets is None:
        stance = {}; hgts = {}
        for c in spec["contacts"]:
            z = sole[c["name"]][:, 2]
            vz = np.abs(np.gradient(z, dt))
            hgt = np.array([z[i] - GC.surface_z(terr, float(sole[c["name"]][i, 0]), float(sole[c["name"]][i, 1]))
                            for i in range(NB)])
            hgts[c["name"]] = hgt
            stance[c["name"]] = (vz < v_tol) & (hgt < h_tol)

        if not allow_flight:
            # A walker, climber or crawler is ALWAYS supported. Where the tolerances leave a hole, the
            # lowest contact is the one bearing load -- say so, rather than leave the base unconstrained.
            for i in range(NB):
                if not any(stance[c["name"]][i] for c in spec["contacts"]):
                    lo = min(spec["contacts"], key=lambda c: hgts[c["name"]][i])
                    stance[lo["name"]][i] = True
                    forced += 1

        for c in spec["contacts"]:
            s = stance[c["name"]]; i = 0
            while i < NB:
                if s[i]:
                    j = i
                    while j + 1 < NB and s[j + 1]:
                        j += 1
                    if j - i >= 2:
                        segs.append((c["name"], i, j))
                    i = j + 1
                else:
                    i += 1

        # ---- pass 3: SNAP each segment's foothold to the terrain, PIN it, SOLVE the joints ------
        targets = {}        # (name, frame) -> (pos3, normal3)
        for (nm, i0, i1) in segs:
            p0 = sole[nm][i0].copy()
            p0[2] = GC.surface_z(terr, float(p0[0]), float(p0[1]))     # snap onto what it lands on
            n0 = GC.surface_normal(terr, float(p0[0]), float(p0[1]))
            for i in range(i0, i1 + 1):
                targets[(nm, i)] = (p0, n0)

    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
    err0, err1, reach_short = [], [], []
    dq_at = {c["name"]: {} for c in spec["contacts"]}      # name -> frame -> correction on c["jwb"]
    for i in range(NB):
        act = [nm for nm in byname if (nm, i) in targets]
        if not act:
            continue
        for nm in act:
            c = byname[nm]; tgt, nrm = targets[(nm, i)]
            q = wb[i].copy()
            _set(i, q)
            e0 = float(np.linalg.norm(_sole_pose(d, c["bid"], c["patch"])[0] - tgt))
            for _ in range(iters):
                _set(i, q)
                p, R = _sole_pose(d, c["bid"], c["patch"])
                r = np.concatenate([tgt - p, _rot_err_xy(R, nrm)])
                if np.linalg.norm(r) < 1e-6:
                    break
                mj.mj_jac(m, d, jacp, jacr, p, c["bid"])
                J = np.vstack([jacp[:, c["jdof"]], jacr[0:2][:, c["jdof"]]])
                dq = np.linalg.solve(J.T @ J + damp * np.eye(J.shape[1]), J.T @ r)
                qn = np.clip(q[c["jwb"]] + dq, c["jlo"], c["jhi"])     # joint limits are not negotiable
                if np.linalg.norm(qn - q[c["jwb"]]) < 1e-9:
                    break
                q[c["jwb"]] = qn
            _set(i, q)
            e1 = float(np.linalg.norm(_sole_pose(d, c["bid"], c["patch"])[0] - tgt))
            err0.append(e0); err1.append(e1)
            if e1 > 2e-3:
                reach_short.append((i, nm, e1))
            dq_at[nm][i] = q[c["jwb"]] - wb[i][c["jwb"]]

    # ---- pass 4: TAPER the correction through the swings ----------------------------------------
    # Only stance frames are constrained, so writing the corrected joints back on stance frames alone
    # leaves a step of tens of millimetres at every touchdown and lift-off. A reference with a C0 kink
    # demands an impulsive moment and the FWP test rightly calls it infeasible (we have already paid for
    # that lesson once). Interpolate each joint's correction across the swing; at the knots it is exact.
    wb2 = wb.copy()
    acc = np.zeros_like(wb); cnt = np.zeros(wb.shape[1])
    for c in spec["contacts"]:
        idx = sorted(dq_at[c["name"]])
        if not idx:
            continue
        for k, jw in enumerate(c["jwb"]):
            vals = [dq_at[c["name"]][i][k] for i in idx]
            acc[:, jw] += np.interp(np.arange(NB), idx, vals)     # holds the end values outside [i0,i1]
            cnt[jw] += 1
    for j in range(wb.shape[1]):
        if cnt[j]:
            wb2[:, j] = wb[:, j] + acc[:, j] / cnt[j]             # a joint shared by two contacts averages

    # the schedule this closure was built on -- derived from the terrain, not from "which foot is lower"
    schedule = [sorted(nm for nm in byname if (nm, i) in targets) for i in range(NB)]

    # post-closure penetration: closure snaps the patch CENTRE onto the surface under it, so a foot
    # straddling a stair nosing still buries its toe. That is a FOOTHOLD-PLAN defect, not an IK defect.
    pen = 1e9
    for i in range(NB):
        _set(i, wb2[i])
        for c in spec["contacts"]:
            p, R = _sole_pose(d, c["bid"], c["patch"])
            for q in GC.corners(d.xpos[c["bid"]], R, c["half"], c["patch"], axes=c.get("axes")):
                pen = min(pen, float(q[2]) - GC.surface_z(terr, float(q[0]), float(q[1])))

    # a ghost that never leaves the ground must never have an unsupported frame. If it does, the base
    # is unconstrained there and anything derived from the schedule (base recovery, wrench) is guesswork.
    empty = [i for i in range(NB) if not schedule[i]]
    run = best = 0
    for i in range(NB):
        run = run + 1 if not schedule[i] else 0
        best = max(best, run)

    rep = dict(segments=len(segs), stance_frames=len(err0), schedule=schedule, penetration=pen,
               empty_frames=len(empty), longest_flight=best, flight_s=best * dt, forced=forced,
               err_before_max=(max(err0) if err0 else 0.0), err_before_mean=(float(np.mean(err0)) if err0 else 0.0),
               err_after_max=(max(err1) if err1 else 0.0), err_after_mean=(float(np.mean(err1)) if err1 else 0.0),
               unreachable=len(reach_short),
               worst_unreachable=(max(reach_short, key=lambda t: t[2]) if reach_short else None))
    if verbose:
        for (nm, i0, i1) in segs:
            print("    segment %-7s frames %4d..%4d  foothold (%.3f, %.3f, %.3f)"
                  % (nm, i0, i1, *targets[(nm, i0)][0]))
    return wb2, rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut"); ap.add_argument("out")
    ap.add_argument("--contacts", default="feet")
    ap.add_argument("--terrain", default=None)
    ap.add_argument("--mjcf", default=None)
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--allow-flight", action="store_true",
                    help="permit frames with no contact (hop/jump ghosts). Default: the lowest contact "
                         "is taken as bearing, because a walker is never airborne and an unsupported "
                         "frame leaves the base recovery unconstrained.")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()

    lut = json.load(open(a.lut))
    if "wb_lut" not in lut or "root_lut" not in lut:
        raise SystemExit("ghost_close: needs a schema-2 ghost (wb_lut + root_lut). Run ghost_upgrade.py first.")
    spec = GC.load_spec(a.contacts)
    t = a.terrain
    if t and os.path.exists(t):
        t = json.load(open(t))
    terr = GC.make_terrain(t if t is not None else (lut.get("terrain") or spec.get("terrain")))

    wb2, rep = close(lut, spec, terr, a.mjcf, a.iters, verbose=a.report, allow_flight=a.allow_flight)
    print("GHOST CLOSE: %s   terrain=%s" % (os.path.basename(a.lut), json.dumps(terr)))
    print("  stance segments %d over %d contact-frames" % (rep["segments"], rep["stance_frames"]))
    if rep["forced"]:
        print("  %d frames had no contact within tolerance; the lowest contact was taken as the bearing one"
              % rep["forced"])
        print("  (--allow-flight to keep them unsupported, e.g. for a hop ghost)")
    if rep["empty_frames"]:
        print("  UNSUPPORTED on %d frames (longest run %.2f s). If this ghost is not meant to leave the"
              % (rep["empty_frames"], rep["flight_s"]))
        print("  ground, the schedule is wrong -- and every base recovery across those gaps is a guess.")
    print("  contact-to-foothold error   BEFORE  mean %.4f  max %.4f m" % (rep["err_before_mean"], rep["err_before_max"]))
    print("                              AFTER   mean %.4f  max %.4f m" % (rep["err_after_mean"], rep["err_after_max"]))
    if rep["unreachable"]:
        i, nm, e = rep["worst_unreachable"]
        print("  KINEMATICALLY UNREACHABLE on %d stance frames -- worst frame %d, %s, short by %.1f mm."
              % (rep["unreachable"], i, nm, 1000 * e))
        print("  The leg cannot put the foot where the ghost's own pelvis path requires. This is not a")
        print("  tuning problem: lower the ride height or shorten the step. The old analytic IK hid it")
        print("  by clamping and letting the foot float.")
    else:
        print("  all stance contacts reach their footholds -- the ghost is kinematically CLOSED.")
    if rep["penetration"] < -2e-3:
        print("  REMAINING PENETRATION %.1f mm: a contact patch still cuts into the terrain. Closure snaps"
              % (1000 * rep["penetration"]))
        print("  the patch CENTRE onto the surface under it, so a foot straddling a stair nosing buries its")
        print("  toe. That is the FOOTHOLD PLAN's fault, not the IK's -- give the nosing a clearance.")

    out = dict(lut)
    out["wb_lut"] = [[float(v) for v in r] for r in wb2]
    out["terrain"] = terr
    out["contact_schedule"] = rep["schedule"]
    out["ghost_schema"] = 2
    out["ghost_closed"] = True
    json.dump(out, open(a.out, "w"))
    print("  wrote %s" % a.out)


if __name__ == "__main__":
    main()
