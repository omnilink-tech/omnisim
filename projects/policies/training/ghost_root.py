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

"""ghost_root.py -- RECOVER a ghost's base trajectory from its joints + contact schedule.

THE IDEA
A ghost lut stores joint angles. Whether the motion is dynamically achievable depends on where the BASE
goes, and most of our luts never say -- walk and crawl carry no root_lut at all. Inventing one (a glide
at vx) is worthless: it floats the robot, every contact reads "flight", and the validator either lies or
refuses (see ghost_dynamics.py).

But the base trajectory is not missing information. It is IMPLIED. A planted contact is stationary in
the world, so between two frames where contact c stays planted:

    p_wc = p_wb(i)   + R_wb(i)   * p_bc(i)          (world contact position, frame i)
         = p_wb(i+1) + R_wb(i+1) * p_bc(i+1)        (unchanged -- it is planted)

    =>  p_wb(i+1) = p_wb(i) + R_wb(i) * p_bc(i) - R_wb(i+1) * p_bc(i+1)

p_bc comes from forward kinematics with the base at the identity, so it depends on the JOINTS ALONE.
This is contact-aided kinematic odometry, the same relation legged robots use to estimate their own
motion. Integrate it and the base pops out of the joint trajectory -- for ANY motion class, including
crawl (contacts are hands and knees) and stairs (the foot simply stays where it landed, which is what
makes the base climb).

WHICH CONTACTS ARE PLANTED
Also derivable without the base. Rotate each contact's base-frame offset into world axes; the planted
ones are those sitting at the bottom, within swing_tol of the lowest. That test needs the base's
ATTITUDE (which the lut has, in att_lut + base_pitch) but not its POSITION -- so there is no circularity.

WHAT THIS ASSUMES, PLAINLY
  - Base yaw is constant (straight-line ghosts: walk, crawl). Turning ghosts already carry a root_lut;
    do not run this on them. A yaw-recovering version needs the contact's world YAW to be constant too,
    which is ill-conditioned through a 90-degree base pitch (crawl) -- so it is deliberately not here.
  - A contact that just landed contributes nothing until the next frame (it defines its own world pose).
  - The absolute height is fixed afterwards, by dropping the base until the lowest planted contact rests
    on its surface.
"""
import math

import numpy as np
import mujoco as mj


def _R(roll, pitch, yaw):
    """ZYX: R = Rz(yaw) Ry(pitch) Rx(roll) -- the convention ghost_dynamics._set builds its quat with."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def base_frame_contacts(model, data, wb, WBJ, spec):
    """Contact-PATCH positions in the BASE frame (base at identity) -- a function of the joints alone.

    ⛔ It must be the patch, not the body origin. What stays still in the world is the sole, and the
    G1's sole is 35 mm ahead of and 36 mm below `*_ankle_roll_link`. A foot pivoting about a planted
    sole swings its ankle origin through a centimetre; treating that origin as the stationary point
    banks a small error at every stance and the recovered climb came out 150% of the truth.
    """
    qadr = {n: model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ}
    NB = wb.shape[0]
    out = {c["name"]: np.zeros((NB, 3)) for c in spec["contacts"]}
    for c in spec["contacts"]:
        c.setdefault("bid", mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, c["body"]))
        c.setdefault("patch", [0.0, 0.0, 0.0])
    for i in range(NB):
        data.qpos[0:3] = 0.0
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for k, n in enumerate(WBJ):
            data.qpos[qadr[n]] = wb[i, k]
        mj.mj_forward(model, data)
        for c in spec["contacts"]:
            R = data.xmat[c["bid"]].reshape(3, 3)
            out[c["name"]][i] = data.xpos[c["bid"]] + R @ np.asarray(c["patch"], float)
    return out


def infer_schedule(p_bc, R_wb, names, swing_tol):
    """Planted = sitting within swing_tol of the LOWEST contact, in world-aligned axes.
    Needs the base ATTITUDE only, never its position -- so this is not circular.

    ⛔ THIS IS A FLAT-GROUND RULE, and it is wrong the moment the contacts are not coplanar. On stairs
    the trailing foot stands a whole riser BELOW the leading one, so it stays "the lowest contact" for
    the first half of its own swing: the recoverer then pins a foot that is climbing through the air and
    concludes the base is descending. Measured on the closed stair ghost, it recovered 0.00 m of climb
    from joints that provably deliver 0.35 m.

    On non-coplanar terrain, pass an explicit `sched` to reconstruct() -- ghost_close derives one from
    the terrain (a contact is planted when it is near the surface BENEATH IT, not near the other foot),
    and any schema-2 ghost carries `contact_schedule`.
    """
    NB = len(R_wb)
    sched = []
    for i in range(NB):
        h = {n: float((R_wb[i] @ p_bc[n][i])[2]) for n in names}
        lo = min(h.values())
        sched.append(sorted(n for n in names if h[n] <= lo + swing_tol))
    return sched


def reconstruct(lut, model, data, spec, wb, WBJ, yaw=0.0, lam=0.15, sched=None, terr=None):
    """SOLVE the base position from contact stationarity (least squares), not integrate it.

    ⛔ Frame-by-frame integration is what you reach for first, and it is wrong. Each contact switch
    injects a small inconsistency (the planted foot of a phase-folded gait does not hold perfectly
    still, and the constraining contact SET changes), and integration accumulates it. Measured on the
    achieved walk: the mean speed came out right (0.447 vs a declared 0.45 m/s) while the per-frame
    base lurched BACKWARD at -0.156 m/s and hit 45 m/s^2 -- 4.6 g. At dt = 0.0125 s a millimetre of
    position noise is metres per second squared, so the wrench built from those accelerations was
    nonsense (floating-base residual 205% of body weight).

    So pose it globally instead. Unknowns are all base positions at once:
        contact rows   for every c planted across step i:   p_{i+1} - p_i = R_i q_i - R_{i+1} q_{i+1}
        smoothness     lam * (p_{i+1} - 2 p_i + p_{i-1}) = 0     -- a bounded-acceleration prior
        gauge          p_0 = 0
    Least squares distributes each switch's inconsistency over the whole cycle instead of banking it.
    `lam` trades contact fidelity against base smoothness; it must stay small enough not to drag the
    mean velocity (check the recovered speed against the lut's own vx -- an independent witness).
    """
    NB = wb.shape[0]
    att = np.asarray(lut["att_lut"], float) if "att_lut" in lut else np.zeros((NB, 2))
    bp = float(lut.get("base_pitch", 0.0))
    R_wb = [_R(float(att[i, 0]), float(att[i, 1]) + bp, yaw) for i in range(NB)]

    names = [c["name"] for c in spec["contacts"]]
    p_bc = base_frame_contacts(model, data, wb, WBJ, spec)
    if sched is None:
        sched = infer_schedule(p_bc, R_wb, names, float(spec.get("swing_tol", 0.012)))

    rows, rhs = [], []

    def _row(coeffs, b):
        for ax in range(3):
            r = np.zeros(3 * NB)
            for idx, w in coeffs:
                r[3 * idx + ax] = w
            rows.append(r); rhs.append(b[ax])

    for i in range(NB - 1):
        for n in [n for n in sched[i] if n in sched[i + 1]]:
            _row([(i + 1, 1.0), (i, -1.0)], R_wb[i] @ p_bc[n][i] - R_wb[i + 1] @ p_bc[n][i + 1])
    for i in range(1, NB - 1):
        _row([(i + 1, lam), (i, -2.0 * lam), (i - 1, lam)], np.zeros(3))
    _row([(0, 1e3)], np.zeros(3))                     # gauge: anchor the first frame

    p = np.linalg.lstsq(np.asarray(rows), np.asarray(rhs), rcond=None)[0].reshape(NB, 3)

    # absolute height: drop the base until the lowest planted contact rests on the surface BENEATH IT.
    # A single scalar surface is a flat world; on stairs the surface under a contact depends on where
    # that contact is, which is why `terr` is threaded through rather than assumed away.
    def _surf(n, i):
        if terr is None:
            return float(next(c for c in spec["contacts"] if c["name"] == n).get("surface_z") or 0.0)
        import ghost_contacts as _GC
        q = p[i] + R_wb[i] @ p_bc[n][i]
        return _GC.surface_z(terr, float(q[0]), float(q[1]))

    lows = [float((p[i] + R_wb[i] @ p_bc[n][i])[2]) - _surf(n, i) for i in range(NB) for n in sched[i]]
    if lows:
        p[:, 2] -= min(lows)

    root = np.zeros((NB, 4))
    root[:, 0:3] = p
    root[:, 3] = yaw
    return root, sched
