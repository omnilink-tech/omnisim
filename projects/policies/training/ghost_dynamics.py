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

"""ghost_dynamics.py -- is a ghost DYNAMICALLY achievable? (any motion class, any contact set)

ghost_doctor.py answers "does any joint exceed a position/velocity limit?" (T0). It never asks whether
the robot can PRODUCE the motion. This asks the one question that stays meaningful for ANY contact set
at ANY heights, for walk, turn, stairs, crawl or carry alike:

    do contact forces EXIST -- each inside its friction cone, each pressing -- that supply exactly what
    the floating base needs, AND leave joint torques inside their limits?

That is FWP membership (FWP = actuation wrench polytope ∩ contact wrench cone), posed as one LP. For a
single planar foot the CWC decomposes in closed form into friction + CoP-in-support + foot-yaw torque
(Caron, Pham & Nakamura, ICRA 2015, arXiv:1501.04719); the LP subsumes all three and generalises past
them. The torque rows are what turn CWC into FWP (Orsolino RA-L 2018; arXiv:2202.01628).

⛔⛔ "ACHIEVED" IS NOT "ACHIEVABLE" -- the thing this tool exists to catch.
Our recorded ghosts are captured from policies running under the CRANE HARNESS (HARNESS_KZ / HARNESS_FY
/ attitude PD). A crane applies external wrenches, so recording a motion proves only that it is feasible
WITH THE CRANE. Measured on the flagship walk ghost: the feet are 19 cm apart, the stance foot sits
8-10 cm to the side, and the COM never leaves the centreline (+/-15 mm). No contact force distribution
can hold that. Driving the base to quasi-static, the residual external help converges to a pure moment
of ~21 N*m against the geometric prediction m*g*offset = 26.8 N*m -- i.e. the crane. The SOLVED turn
ghost, by contrast, demands 0.0 N. [SUPPORT] reports this debt; zero means free-standing.
So ghost-design Rule 1's "Recorded" provenance class is NOT a feasibility certificate.

⛔ These are CHECKS, not objectives. Our 3-arm controlled experiment (commit 621a9850) showed that
MAXIMISING a stability margin is worse than useless: a centred COM costs ~35% more pelvis excursion,
which raises peak lateral velocity, and it was velocity -- not margin -- that decided whether the policy
could track the ghost. Margin is a restriction sized to a disturbance, never a thing to maximise.

=== THE THREE GATES, IN ORDER. A ghost that fails gate N makes gate N+1 meaningless. ==============
  GATE 1  CLOSURE   a planted contact does not move.        [CLOSURE]   -> repair: ghost_close.py
  GATE 2  SUPPORT   the COM stays inside the support hull.  [COM]       -> repair: SOLVE the base
  GATE 3  FEASIBLE  the contacts can supply the wrench.     [FWP]/[SUPPORT]
Gate 3 is all we had, and on the stair ghost it could not even run: no terrain, and the contact patch
was pinned to the ankle origin instead of the sole. Gate 1 is pure forward kinematics and would have
killed the stair campaign's reference in one second.

⛔⛔ THE BASE IS AN OUTPUT, NOT A DESIGN VARIABLE. Every ghost builder we have writes the base as a
formula in the foot positions -- `pelvis_x = 0.5*(xL+xR)`, `pelvis_z = min(zL,zR) + ride`, `pelvis_y = 0`
-- and then fills the joints in with a closed-form IK. That over-determines the system, and the residual
comes out as levitation: the stair ghost's stance foot drifts 62 mm per stance, so only 55% of its
declared 0.35 m climb is produced by its legs. Worse, `pelvis_y = 0` leaves the COM 88.5 mm outside the
stance foot on EVERY single-support frame -- the exact defect measured on the flagship walk ghost, whose
COM never leaves the centreline while its stance foot sits 80-100 mm to the side.
Measured demand for external help (the crane), same tool, three ghosts:
    SOLVED turn ghost   (base solved so COM stays in support) ....  0.0 N        <- the only one that passes
    RECORDED walk ghost (base recorded under the crane) ........... ~21 N*m
    FORMULA stair ghost (base = 0.5*(xL+xR), min(zL,zR)+ride, y=0)  62 N + 56 N*m
Sweeping the formula's constants (sway toward the stance foot, pelvis over the stance foot, less lean)
moves the COM margin from -0.162 m to -0.065 m and never reaches zero. Parameterising the base does not
work. Solve it -- see build_step_turn_ghost.solve_base_for_com.

⛔ FOUR TRAPS, every one of which produced convincing wrong numbers before being caught:
  1. mj_forward() OVERWRITES d.qacc. Prescribe the trajectory's qacc AFTER it, before mj_inverse, or you
     inverse-dynamics a different motion (torques then read suspiciously exactly at the limits).
  2. Contacts at z=0 make mujoco resolve its OWN contact forces inside mj_inverse, double-counting the
     ground wrench (base residual 16468 N). Contacts/constraints are disabled here.
  3. CoP == ZMP only when all contacts lie in ONE plane (Sardain & Bessonnet 2004). On stairs they do
     not. Non-coplanar support is detected and the CoP number is refused.
  4. The wrench the contacts must supply is qfrc_inverse's FLOATING-BASE ROWS -- the same dynamics that
     produced the joint torques. Assembling it independently from finite-differenced com_ddot / Ldot
     makes the LP target and the torque bookkeeping two computations of one physics; they disagree by
     the derivative error. Invisible on a near-static ghost, ruinous on a dynamic one (the achieved walk
     read 204% of body weight in residual, and "infeasible" for the wrong reason).
Cyclic ghosts wrap their finite differences; one-shot `seq` routines clamp. Smoothing filters the
TRAJECTORY, never the derivatives -- and never wrap-pads the base position, which drifts.

Usage:
  python ghost_dynamics.py <lut.json> [--contacts feet|crawl|<spec.json>] [--smooth K] [--mu 1.0] [--json]
"""
import argparse
import json
import math
import os

import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # runnable from any cwd
import ghost_contacts as GC

# repo root: OMNISIM_HOME if set, else walk up from this file (projects/policies/training/..)
RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEF_MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")
DEF_URDF = os.path.join(RT, "projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf")
G = 9.81


def _urdf_limits(urdf):
    import xml.etree.ElementTree as ET
    eff, vel = {}, {}
    try:
        for j in ET.parse(urdf).getroot().iter("joint"):
            l = j.find("limit")
            if l is None:
                continue
            if l.get("effort"):
                eff[j.get("name")] = float(l.get("effort"))
            if l.get("velocity"):
                vel[j.get("name")] = float(l.get("velocity"))
    except Exception:
        pass
    return eff, vel


def analyze(lut_path, contacts="feet", mjcf=None, urdf=None, mu=None, smooth=0, terrain=None, stride=1):
    gd = json.load(open(lut_path))
    NB = int(gd["nb"])
    cyc = float(gd.get("cycle_s", 1.0 / gd["freq"]))
    dt = cyc / NB
    # ⛔ PREFLIGHT. A dynamics verdict needs the WHOLE-BODY pose and the BASE trajectory. Our luts vary
    # in three independent ways (wb_lut vs leg-only, root_lut present or not, contact schedule never
    # declared), which is precisely why five builders grew five different feasibility notions. Refuse to
    # guess: a validator that invents the missing half will print confident, wrong numbers.
    if "wb_lut" not in gd:
        raise SystemExit(
            "ghost_dynamics: '%s' has no wb_lut (leg-only ghost).\n"
            "  A dynamic verdict needs every link's mass, so the whole-body pose is required.\n"
            "  Fix the GHOST FORMAT (emit wb_lut), not this tool." % os.path.basename(lut_path))
    wb = np.asarray(gd["wb_lut"], float)
    WBJ = gd["wb_joints"]
    synthesized_root = "root_lut" not in gd
    if synthesized_root:
        # CYCLIC ghosts (walk, crawl) carry no root_lut -- only vx and a base height. Synthesize a base
        # that glides forward at vx: enough for contact/COM/wrench analysis, but the base path is
        # ASSUMED, not measured, so treat absolute margins from a cyclic ghost with that in mind.
        vx = float(gd.get("vx", 0.0)); z0 = float(gd.get("ghost_z", 0.72))
        tt = np.arange(NB) * dt
        root = np.stack([vx * tt, np.zeros(NB), np.full(NB, z0), np.zeros(NB)], axis=1)
    else:
        root = np.asarray(gd["root_lut"], float)
    att = np.asarray(gd["att_lut"], float) if "att_lut" in gd else np.zeros((NB, 2))
    base_pitch = float(gd.get("base_pitch", 0.0))
    # A `seq` ghost is a one-shot routine (clamp the ends); anything else is a CYCLE and its finite
    # differences must WRAP, or the first and last frames get invented accelerations.
    cyclic = not bool(gd.get("seq", False))
    # ⛔ [CLOSURE] is judged on the ghost AS WRITTEN. --smooth is a diagnostic for C0 kinks in the
    # DYNAMICS; it necessarily perturbs the kinematics, and it moved a foot pinned to 0.2 mm by 19 mm.
    wb_raw, root_raw, att_raw = wb.copy(), root.copy(), att.copy()
    if smooth > 1:
        # ⛔ Smooth the TRAJECTORY, never the derivatives. Smoothing cdd/Ldot (which build the LP's target
        # wrench) while leaving qacc alone (which builds qfrc_inverse) makes the two disagree: on the
        # near-static turn that hid inside the noise, on a dynamic walk it blew the floating-base residual
        # to 87.6% of body weight. Filter the path, then differentiate it -- then everything is consistent.
        k = np.ones(smooth) / smooth
        def _sm(a, m):
            ap = np.pad(a, ((smooth, smooth), (0, 0)), mode=m)
            return np.stack([np.convolve(ap[:, c], k, mode="same")[smooth:smooth + a.shape[0]]
                             for c in range(a.shape[1])], axis=1)
        # ⛔ joints and attitude are PERIODIC -> wrap-pad. The base POSITION translates over a cycle, so
        # wrap-padding it splices the end back onto the start and injects a metre-sized step at the seam
        # (measured: floating-base residual 1799% of body weight). Edge-pad anything that drifts.
        per = "wrap" if cyclic else "edge"
        wb = _sm(wb, per)
        att = _sm(att, per)
        root[:, 0:3] = _sm(root[:, 0:3], "edge")
        root[:, 3:4] = _sm(root[:, 3:4], "edge")

    spec = GC.load_spec(contacts)
    mu = mu if mu is not None else float(spec.get("mu", 1.0))
    swing_tol = float(spec.get("swing_tol", 0.012))
    # THE WORLD THE GHOST STANDS ON. Precedence: CLI > the ghost's own declaration > the contact spec.
    # A ghost that climbs is only judgeable against the thing it climbs; grading a stair ghost on flat
    # ground marks every raised foot "flight" and then reports a verdict about nothing.
    terr = GC.make_terrain(terrain if terrain is not None else (gd.get("terrain") or spec.get("terrain")))
    for c in spec["contacts"]:
        if "terrain" in c:
            c["_terr"] = GC.make_terrain(c["terrain"])
        elif c.get("surface_z") not in (None, "auto"):
            c["_terr"] = {"type": "flat", "z": float(c["surface_z"])}   # legacy per-contact plane
        else:
            c["_terr"] = terr
        c["patch"] = c.get("patch", [0.0, 0.0, 0.0])

    m = mj.MjModel.from_xml_path(mjcf or DEF_MJCF)
    m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT) | int(mj.mjtDisableBit.mjDSBL_CONSTRAINT)
    d = mj.MjData(m)
    qadr = {n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ}
    dadr = {n: m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ}
    for c in spec["contacts"]:
        c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
        if c["bid"] < 0:
            raise SystemExit("contact body not in model: %s" % c["body"])
    MASS = float(mj.mj_getTotalmass(m))
    NV = m.nv
    eff, vlim = _urdf_limits(urdf or DEF_URDF)

    def _set(i):
        d.qpos[0:3] = root[i, 0:3]
        # base attitude: roll/pitch from att_lut (+ a constant base_pitch, e.g. the crawl lean), yaw from root
        r_, p_, y_ = float(att[i, 0]), float(att[i, 1]) + base_pitch, float(root[i, 3])
        cr, sr = math.cos(r_ / 2), math.sin(r_ / 2)
        cp, sp = math.cos(p_ / 2), math.sin(p_ / 2)
        cy, sy = math.cos(y_ / 2), math.sin(y_ / 2)
        d.qpos[3:7] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                       cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        for k, n in enumerate(WBJ):
            d.qpos[qadr[n]] = wb[i, k]

    # pass 1: kinematics, COM, contact-PATCH poses (the sole plane, not the ankle origin -- see
    # ghost_contacts.corners: the G1's sole sits 36 mm below and 35 mm ahead of *_ankle_roll_link)
    com = np.zeros((NB, 3)); qpos = np.zeros((NB, m.nq))
    cpos = {c["name"]: np.zeros((NB, 3)) for c in spec["contacts"]}      # patch centre, world
    crot = {c["name"]: np.zeros((NB, 3, 3)) for c in spec["contacts"]}

    def _kin():
        for i in range(NB):
            _set(i); mj.mj_forward(m, d)
            com[i] = d.subtree_com[0]; qpos[i] = d.qpos
            for c in spec["contacts"]:
                R = d.xmat[c["bid"]].reshape(3, 3)
                cpos[c["name"]][i] = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
                crot[c["name"]][i] = R

    byname_spec = {c["name"]: c for c in spec["contacts"]}

    def _cornerset(cname, chalf, i):
        # patch centre already applied in cpos; axes = the patch PLANE (a knee bears on its +x face)
        return GC.corners(cpos[cname][i], crot[cname][i], chalf, axes=byname_spec[cname].get("axes"))

    _kin()
    if synthesized_root:
        # A cyclic ghost never says how HIGH its base sits (`ghost_z` is the hologram's display offset,
        # not the pelvis height -- using it floats the robot and every contact reads "flight"). Solve it:
        # drop the base until the lowest contact CORNER rests on the surface beneath it.
        low = min(min(float(q[2]) - GC.surface_z(c["_terr"], float(q[0]), float(q[1]))
                      for i in range(NB) for q in _cornerset(c["name"], c["half"], i))
                  for c in spec["contacts"])
        root[:, 2] -= low
        _kin()

    # finite differences. A CYCLE wraps; a one-shot routine clamps its ends.
    def _nb(i):
        if cyclic:
            return (i + 1) % NB, (i - 1) % NB, 2 * dt
        ip, im = min(i + 1, NB - 1), max(i - 1, 0)
        return ip, im, max((ip - im), 1) * dt

    qvel = np.zeros((NB, NV)); qacc = np.zeros((NB, NV))
    for i in range(NB):
        ip, im, h = _nb(i)
        # ⛔ a cyclic gait TRANSLATES over its cycle, so wrapping the base position would read as a giant
        # backward jump at the seam. Wrap the joints; extrapolate the base across the seam instead.
        if cyclic and (i == 0 or i == NB - 1):
            jp, jm = min(i + 1, NB - 1), max(i - 1, 0)
            qvel[i, 0:3] = (qpos[jp, 0:3] - qpos[jm, 0:3]) / (max(jp - jm, 1) * dt)
        else:
            qvel[i, 0:3] = (qpos[ip, 0:3] - qpos[im, 0:3]) / h
        dy = math.atan2(math.sin(root[ip, 3] - root[im, 3]), math.cos(root[ip, 3] - root[im, 3]))
        qvel[i, 5] = dy / h
        for n in WBJ:
            qvel[i, dadr[n]] = (qpos[ip, qadr[n]] - qpos[im, qadr[n]]) / h
    for i in range(NB):
        ip, im, h = _nb(i)
        qacc[i] = (qvel[ip] - qvel[im]) / h

    Lang = np.zeros((NB, 3))
    for i in range(NB):
        d.qpos[:] = qpos[i]; d.qvel[:] = qvel[i]
        mj.mj_forward(m, d); mj.mj_subtreeVel(m, d)   # trap 1: mj_forward clobbers qacc
        Lang[i] = d.subtree_angmom[0]

    cdd = np.zeros((NB, 3)); Ldot = np.zeros((NB, 3))
    for i in range(NB):
        ip, im, h = _nb(i)
        # angular momentum is PERIODIC -> safe to wrap. The COM TRANSLATES over a cycle, so wrapping it
        # reads as a giant jump at the seam; clamp it and accept two slightly-wrong end frames.
        jp, jm = min(i + 1, NB - 1), max(i - 1, 0)
        if 0 < i < NB - 1:
            cdd[i] = (com[jp] - 2 * com[i] + com[jm]) / (dt * dt)
        Ldot[i] = (Lang[ip] - Lang[im]) / h

    jacp = np.zeros((3, NV)); jacr = np.zeros((3, NV))
    peak_tau = {n: 0.0 for n in WBJ}
    base_res = 0.0; com_margin = 1e9; rob_min = 1e9; torque_ratio = 0.0; ne_mismatch = 0.0; ext_F = 0.0; ext_M = 0.0
    n_support = 0; n_infeasible = 0; noncoplanar = 0; n_lp = 0; infeasible_at = []
    contact_hist = {}
    gap_pen = 1e9        # most negative corner gap anywhere -> penetration into the terrain
    gap_hover = -1e9     # worst "lowest corner still floats this far" over PLANTED frames
    jnames = [n for n in WBJ if eff.get(n, 0) > 0]      # joints with an effort limit -> the FWP rows
    jlims = [eff[n] for n in jnames]

    # [CLOSURE] GATE 1: a planted contact must NOT MOVE. Forward kinematics alone -- no LP, no wrench.
    # The stair ghost's "planted" feet drifted 62 mm per stance and floated 168 mm in total, so a tracker
    # reproducing its joints perfectly still could not climb: the reference's own root_lut was fiction.
    #
    # ⛔ THIS GATE NEEDS THE GHOST'S CONTACT SCHEDULE, and will not fake one. Two heuristics were tried
    # and both lied. "A corner is within swing_tol" is not "planted": on a step-to stair gait each foot
    # climbs TWO risers per step, so its swing SKIMS the intermediate tread inside swing_tol and the run
    # merges a stance with a swing (392 mm of drift on a ghost pinned to 0.2 mm). Breaking runs on corner
    # speed then chops the broken ghost's slow levitating creep into short runs and hides it (9 mm, when
    # the truth is 62 mm). Deciding which frames bear load is precisely what a ghost must DECLARE.
    # ghost_close.py derives one and writes it; ghost_upgrade.py emits it for legacy luts.
    csched = gd.get("contact_schedule")
    slips = []
    for i in range(NB):
        # ⛔ contact is decided PER CORNER against z = f(x,y), not per body against one plane. On stairs a
        # foot can have its toe on tread k and its heel hanging over the nosing; a single patch-level test
        # either invents the two missing corners or throws the two real ones away.
        active = []
        for c in spec["contacts"]:
            T = c["_terr"]
            cs = _cornerset(c["name"], c["half"], i)
            surf = [GC.surface_z(T, float(q[0]), float(q[1])) for q in cs]
            gaps = [float(q[2]) - s for q, s in zip(cs, surf)]
            gap_pen = min(gap_pen, min(gaps))
            on = [j for j in range(len(cs)) if gaps[j] <= swing_tol]
            if on:
                # "hover" only means something for a contact the ghost SAYS is bearing load. A swing foot
                # passes through the swing_tol band on its way up and down; counting that as a hovering
                # planted foot printed "FEET NEVER TOUCH" on a ghost whose soles sit exactly on the tread.
                if (not csched) or (c["name"] in csched[i]):
                    gap_hover = max(gap_hover, min(gaps))
                active.append(dict(name=c["name"], bid=c["bid"],
                                   corners=[cs[j] for j in on], surf=[surf[j] for j in on],
                                   normals=[GC.surface_normal(T, float(cs[j][0]), float(cs[j][1])) for j in on]))
        key = "+".join(sorted(a["name"] for a in active)) or "flight"
        contact_hist[key] = contact_hist.get(key, 0) + 1
        if not active:
            continue
        n_support += 1
        allsurf = [s for a in active for s in a["surf"]]
        if max(allsurf) - min(allsurf) > 1e-3:
            noncoplanar += 1
        if i % stride:
            continue                 # LPs are the expensive part; contact bookkeeping above is not
        n_lp += 1

        hull = GC.hull2d([(p[0], p[1]) for a in active for p in a["corners"]])
        com_margin = min(com_margin, GC.margin_in_hull(com[i][:2], hull))

        d.qpos[:] = qpos[i]; d.qvel[:] = qvel[i]
        mj.mj_forward(m, d); d.qacc[:] = qacc[i]; mj.mj_inverse(m, d)   # trap 1: qacc AFTER forward
        qfi = d.qfrc_inverse.copy()

        # ⛔ TRAP 4. The wrench the contacts must supply is qfrc_inverse's FLOATING-BASE rows -- the same
        # dynamics that produced the joint torques. Building it independently from finite-differenced
        # com_ddot / Ldot makes the LP target and the torque bookkeeping two different computations of the
        # same physics; they then disagree by the derivative error. That is invisible on a near-static
        # ghost (com_ddot ~ 0, so the two agree by accident) and catastrophic on a dynamic one: the
        # achieved walk read 204% of body weight in residual and "infeasible" on 54/64 frames.
        w = qfi[0:6].copy()
        F = MASS * (cdd[i] + np.array([0.0, 0.0, G]))    # kept only as an independent consistency witness
        ne_mismatch = max(ne_mismatch, float(np.linalg.norm(qfi[0:3] - F)))

        cols, pts, owner, corner_pos, cbid = GC.build_columns(active, mu)
        # generalized force of a UNIT weight on each cone edge, at its corner's body. The body comes
        # from build_columns, not from a nearest-corner search: on stairs the two feet can be nearer to
        # each other than a foot's own corners are, and the search then attributes force to the wrong leg.
        Jc = []
        for cp, bid in zip(corner_pos, cbid):
            mj.mj_jac(m, d, jacp, jacr, cp, bid)
            Jc.append(jacp.copy())
        Gfull = np.stack([Jc[owner[j]].T @ cols[j] for j in range(len(cols))], axis=1)   # (NV, ncols)
        jrows = [dadr[n] for n in jnames]
        # equality rows are the BASE generalized-force rows of the very same G, so "the contacts supply
        # exactly what the floating base needs" is enforced in the coordinates it is measured in.
        ok, tratio, f = GC.fwp_check_G(Gfull[0:6, :], w, Gfull[jrows, :], qfi[jrows], jlims,
                                       owner, cols, len(corner_pos))
        if not ok:
            n_infeasible += 1
            infeasible_at.append(i)
            # the contacts cannot do it -- so how much EXTERNAL wrench is the ghost silently leaning on?
            eF, eM = GC.support_demand(Gfull[0:6, :], w, Gfull[jrows, :], qfi[jrows], jlims)
            if np.isfinite(eF):
                ext_F = max(ext_F, eF); ext_M = max(ext_M, eM)
            continue
        torque_ratio = max(torque_ratio, tratio)
        # self-check: with the LP's own force distribution the FLOATING-BASE residual must vanish
        tau_c = np.zeros(NV)
        for ci, cp in enumerate(corner_pos):
            tau_c += Jc[ci].T @ f[ci]
        treq = qfi - tau_c
        base_res = max(base_res, float(np.max(np.abs(treq[0:6]))))
        for n in WBJ:
            peak_tau[n] = max(peak_tau[n], abs(float(treq[dadr[n]])))
        _ok2, _f2, rob = GC.cwc_feasible(active, com[i], w, mu)
        if _ok2:
            rob_min = min(rob_min, rob)

    if csched:
        # net displacement of each declared stance, on the UNSMOOTHED ghost: exactly the quantity
        # ghost_close drives to zero.
        raw = {c["name"]: np.zeros((NB, 3)) for c in spec["contacts"]}
        for i in range(NB):
            d.qpos[0:3] = root_raw[i, 0:3]
            r_, p_, y_ = float(att_raw[i, 0]), float(att_raw[i, 1]) + base_pitch, float(root_raw[i, 3])
            cr, sr = math.cos(r_ / 2), math.sin(r_ / 2)
            cp, sp = math.cos(p_ / 2), math.sin(p_ / 2)
            cy, sy = math.cos(y_ / 2), math.sin(y_ / 2)
            d.qpos[3:7] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                           cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
            for k, n in enumerate(WBJ):
                d.qpos[qadr[n]] = wb_raw[i, k]
            mj.mj_kinematics(m, d)
            for c in spec["contacts"]:
                R = d.xmat[c["bid"]].reshape(3, 3)
                raw[c["name"]][i] = d.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
        for c in spec["contacts"]:
            nm = c["name"]; i = 0
            while i < NB:
                if nm in csched[i]:
                    j = i
                    while j + 1 < NB and nm in csched[j + 1]:
                        j += 1
                    if j - i >= 3:
                        slips.append(float(np.linalg.norm(raw[nm][j] - raw[nm][i])))
                    i = j + 1
                else:
                    i += 1

    tau_worst = sorted(((peak_tau[n] / eff[n], n, peak_tau[n], eff[n]) for n in WBJ if eff.get(n, 0) > 0), reverse=True)
    spd_worst = sorted(((float(np.max(np.abs(qvel[:, dadr[n]]))) / vlim[n], n,
                         float(np.max(np.abs(qvel[:, dadr[n]]))), vlim[n]) for n in WBJ if vlim.get(n, 0) > 0), reverse=True)
    return dict(nb=NB, cycle_s=cyc, mass=MASS, mu=mu, support_frames=n_support,
                synthesized_root=synthesized_root, smooth=smooth, stride=stride, lp_frames=n_lp,
                terrain=terr, gap_penetration=(0.0 if gap_pen > 1e8 else gap_pen),
                gap_hover=(0.0 if gap_hover < -1e8 else gap_hover),
                slip_mean=(float(np.mean(slips)) if slips else 0.0), has_schedule=bool(csched),
                slip_max=(float(np.max(slips)) if slips else 0.0), slip_runs=len(slips),
                infeasible_at=infeasible_at,
                com_margin=com_margin, fwp_infeasible=n_infeasible,
                wrench_robustness_N=(0.0 if rob_min > 1e8 else rob_min),
                noncoplanar_frames=noncoplanar, contact_hist=contact_hist,
                torque_use=torque_ratio,          # BEST achievable peak-torque ratio over valid force splits
                speed_use=(spd_worst[0][0] if spd_worst else 0.0),
                base_residual_N=base_res, ne_mismatch_N=ne_mismatch, ext_force_N=ext_F, ext_torque_Nm=ext_M, body_weight_N=MASS * G, tau_worst=tau_worst[:4], spd_worst=spd_worst[:2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut")
    ap.add_argument("--contacts", default="feet", help="preset (feet|crawl) or path to a contact-spec json")
    ap.add_argument("--mjcf", default=None); ap.add_argument("--urdf", default=None)
    ap.add_argument("--mu", type=float, default=None)
    ap.add_argument("--smooth", type=int, default=0,
                    help="moving-average window on the finite-difference com_ddot / Ldot. A C0 keyframe path "
                         "has impulsive curvature at phase boundaries, which the FWP test rightly calls "
                         "infeasible. Smoothing distinguishes such a kink (failure vanishes) from a genuine "
                         "dynamic demand (failure survives).")
    ap.add_argument("--terrain", default=None,
                    help="what the ghost stands on: flat | 'stairs:riser=0.07,going=0.26,x0=0.12,n=5' | "
                         "'ramp:slope=0.15,x0=1.0' | a json file. Overrides the ghost's own declaration.")
    ap.add_argument("--stride", type=int, default=1, help="run the (expensive) LPs on every Nth support frame")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    terr = a.terrain
    if terr and os.path.exists(terr):
        terr = json.load(open(terr))
    r = analyze(a.lut, a.contacts, a.mjcf, a.urdf, a.mu, a.smooth, terr, max(1, a.stride))
    if a.json:
        print(json.dumps({k: v for k, v in r.items() if k not in ("tau_worst", "spd_worst")}, indent=2, default=str))
        return
    print("GHOST DYNAMICS: %s   contacts=%s" % (os.path.basename(a.lut), a.contacts))
    print("  nb=%d cycle_s=%.2f mass=%.1fkg mu=%.1f  support frames=%d%s%s" % (
        r["nb"], r["cycle_s"], r["mass"], r["mu"], r["support_frames"],
        "  smooth=%d" % r["smooth"] if r["smooth"] else "",
        "  stride=%d (%d LP frames)" % (r["stride"], r["lp_frames"]) if r["stride"] > 1 else ""))
    if r["synthesized_root"]:
        print("  NOTE: cyclic ghost (no root_lut) -- base path SYNTHESIZED from vx/ghost_z; margins are indicative")
    print("  [TERRAIN]  %s" % json.dumps(r["terrain"]))
    print("             penetration %+.4f m (min corner gap)   hover %+.4f m (worst planted-foot float)%s"
          % (r["gap_penetration"], r["gap_hover"],
             "  <-- FEET NEVER TOUCH" if r["gap_hover"] > 0.004 else ""))
    print("  contact phases: %s" % dict(sorted(r["contact_hist"].items(), key=lambda kv: -kv[1])))
    # GATE 1. Pure kinematics -- no LP, no wrench, nothing to tune. A ghost that fails here cannot be
    # tracked into the motion it claims: a policy reproduces the JOINTS, and the base then goes wherever
    # physics sends it. Repair with ghost_close.py before reading any number below.
    if not r["has_schedule"]:
        print("  [CLOSURE]  SKIPPED -- this ghost declares no contact_schedule, so nothing here knows which\n"
              "             frames bear load. Run ghost_close.py to derive one (it also repairs the drift).")
    else:
        print("  [CLOSURE]  net drift of a planted contact over its stance: mean %.1f mm  max %.1f mm  (%d stances) %s"
              % (1000 * r["slip_mean"], 1000 * r["slip_max"], r["slip_runs"],
                 "<-- NOT CLOSED: repair with ghost_close.py before reading anything below"
                 if r["slip_mean"] > 5e-3 else ""))
    feasible_frames = r["support_frames"] - r["fwp_infeasible"]
    if feasible_frames <= 0:
        # the residual is only computed on feasible frames; with none, "0.0 N" would be a vacuous PASS
        print("  self-check: N/A -- no frame produced a valid force distribution, so nothing was verified")
    else:
        ok = r["base_residual_N"] < 0.15 * r["body_weight_N"]
        print("  self-check: floating-base residual %.1f N vs body weight %.0f N (%.1f%%) %s"
              % (r["base_residual_N"], r["body_weight_N"], 100 * r["base_residual_N"] / r["body_weight_N"],
                 "OK" if ok else "<-- UNTRUSTWORTHY"))
    if r["synthesized_root"] and r["com_margin"] < 0:
        # ASCII only: printed output must survive a cp1252 console (a unicode glyph here raised
        # UnicodeEncodeError and killed the run -- a validator must not die formatting its own verdict).
        print("  VERDICT REFUSED: the base trajectory was SYNTHESIZED (no root_lut) and the COM lands\n"
              "     outside the contact hull -- the synthesis, not necessarily the ghost, is at fault.\n"
              "     Give this ghost a root_lut; do NOT read the numbers below as a judgement of the motion.")
    print("  [FWP]      infeasible frames: %d/%d %s" % (r["fwp_infeasible"], r["support_frames"],
                                                        "<-- MOTION NOT REALISABLE (no force split satisfies wrench + torque limits)"
                                                        if r["fwp_infeasible"] else ""))
    if r["fwp_infeasible"]:
        _t = [i * r["cycle_s"] / r["nb"] for i in r["infeasible_at"]]
        print("             infeasible at t = %s s" % ", ".join("%.2f" % v for v in _t[:14]))
        print("  [SUPPORT]  this ghost DEMANDS external help on the infeasible frames:")
        print("             up to %.1f N of force and %.1f N*m of torque that the CONTACTS cannot supply." % (r["ext_force_N"], r["ext_torque_Nm"]))
        print("             A recorded ghost is only evidence of feasibility UNDER ITS HARNESS.")
    print("  [WRENCH]   robustness margin %.1f N of horizontal disturbance" % r["wrench_robustness_N"])
    print("  [COM]      margin inside active-contact hull: %.4f m %s" % (r["com_margin"], "<-- OUTSIDE" if r["com_margin"] < 0 else ""))
    if r["noncoplanar_frames"]:
        print("  [CoP/ZMP]  SKIPPED: %d frames have non-coplanar contacts -- CoP==ZMP does not hold there"
              % r["noncoplanar_frames"])
    print("  [TORQUE]   %.2f of limit  (BEST achievable over valid force splits; with redundant contacts the "
          "split is the controller's choice, not the ghost's)" % r["torque_use"])
    for ratio, n, pk, lm in r["tau_worst"][:3]:
        print("        %-26s %6.1f / %5.1f N*m = %.2f %s" % (n, pk, lm, ratio, "<-- OVER" if ratio > 1 else ""))
    print("  [SPEED]    %.2f of limit" % r["speed_use"])
    for ratio, n, pk, lm in r["spd_worst"][:2]:
        print("        %-26s %6.2f / %5.2f rad/s = %.2f %s" % (n, pk, lm, ratio, "<-- OVER" if ratio > 1 else ""))


if __name__ == "__main__":
    main()
