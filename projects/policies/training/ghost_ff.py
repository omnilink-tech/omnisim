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

"""ghost_ff.py -- make the ghost DECLARE ITS OWN FEEDFORWARD (ffdq_lut).

THE BOUND THIS CLOSES -- measured 2026-07-10, the corridor-vs-torque law:
A Shadowing policy expresses torque only as position-target offsets through the PD plant,
dq = tau / kp. The corridor (GHOST_RESIDUAL) clamps those offsets to +-0.12..0.20 rad around the
reference. But the quasi-static torque the reference ITSELF requires (the same LP split gate 3 uses)
already costs, at kp=200:

    walk        knee 0.226 rad        -> every walk campaign trained at corridor 0.12: the policy
    stairs 3cm  knee 0.192 rad           could never hold its own stance torque inside the corridor,
    stairs 7cm  knee 0.233 rad           and the CRANE absorbed the deficit. The 7cm climb was
                                         IMPOSSIBLE BY CONSTRUCTION at corridor 0.12; the 3cm climb
                                         worked exactly when the corridor was widened to 0.20 > 0.192.

And the gate-4 funnel (ghost_funnel.py) shows the 7cm climb IS perfectly trackable by plain PD once
the feedforward is supplied (funnel score 1.000 with ff + the training crane, from every phase).

The fix is NOT a wider corridor -- width is the mimicry guarantee. It is to CENTRE the corridor on
the feedforward-shifted target:

    q_cmd = q_ref + ffdq(phase) + a * GHOST_RESIDUAL,      ffdq = tau_ff / kp

so gravity/stance compensation costs the policy nothing, the corridor keeps its narrow meaning, and
gmatch still scores against q_ref (the pose, not the command). tau_ff comes from the identical
quasi-static LP that certifies gate 3 -- the ghost's own feasibility certificate, cashed as control.

This tool computes ffdq_lut (nb x 12, LEG joint order = wb slots 0..11) and writes it into the lut.
Trainer side: GHOST_FF=1 (g1_walk_recipe.py) shifts the corridor centre by it.

Usage:
  python ghost_ff.py <lut.json> [--kp 200] [--stride 4] [--out <path>]   (default: in-place)
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
import ghost_funnel as GF


def compute_ffdq(lut_path, kp=200.0, stride=4, mjcf=None, urdf=None):
    gd = json.load(open(lut_path))
    NB = int(gd["nb"])
    wb = np.asarray(gd["wb_lut"], float); WBJ = gd["wb_joints"]
    root = (np.asarray(gd["root_lut"], float) if "root_lut" in gd else None)
    att = np.asarray(gd["att_lut"], float) if "att_lut" in gd else np.zeros((NB, 2))
    if root is None:
        vx = float(gd.get("vx", 0.0)); tt = np.arange(NB) * float(gd["cycle_s"]) / NB
        root = np.stack([vx * tt, 0 * tt, 0.72 + 0 * tt, 0 * tt], axis=1)
    terr = GC.make_terrain(gd.get("terrain"))
    m = GF.build_model(mjcf or GF.DEF_MJCF, gd.get("terrain"), 1.0)
    d = mj.MjData(m)
    qadr = np.array([m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ])
    dadr = np.array([m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ])
    eff = GF._urdf_eff(urdf or GF.DEF_URDF)
    tlim = np.array([eff.get(n, 100.0) for n in WBJ])
    spec_c = GC.load_spec("feet")
    for c in spec_c["contacts"]:
        c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
    m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT)

    bins = list(range(0, NB, max(1, stride)))
    tab = np.zeros((NB, len(WBJ)))
    misses = 0
    for b in bins:
        d.qpos[:] = 0; d.qvel[:] = 0
        d.qpos[0:3] = root[b, 0:3]
        r_, p_, y_ = float(att[b, 0]), float(att[b, 1]), float(root[b, 3])
        d.qpos[3:7] = GF._quat(r_, p_, y_)
        d.qpos[qadr] = wb[b]
        mj.mj_forward(m, d); d.qacc[:] = 0.0; mj.mj_inverse(m, d)
        qfi = d.qfrc_inverse.copy()
        active = []
        for c in spec_c["contacts"]:
            R = d.xmat[c["bid"]].reshape(3, 3)
            cs = GC.corners(d.xpos[c["bid"]], R, c["half"], c["patch"])
            on = [q for q in cs if float(q[2]) - GC.surface_z(terr, float(q[0]), float(q[1])) <= 0.03]
            if on:
                active.append(dict(name=c["name"], bid=c["bid"], corners=on, normals=[(0, 0, 1)] * len(on)))
        tau = None
        if active:
            cols, pts, owner, cp, cbid = GC.build_columns(active, 1.0)
            Jc = []
            for q_, bid_ in zip(cp, cbid):
                mj.mj_jac(m, d, jacp, jacr, q_, bid_); Jc.append(jacp.copy())
            Gf = np.stack([Jc[owner[j]].T @ cols[j] for j in range(len(cols))], axis=1)
            ok, _t, f = GC.fwp_check_G(Gf[0:6, :], qfi[0:6], Gf[dadr, :], qfi[dadr],
                                       list(tlim), owner, cols, len(cp))
            if ok and f is not None:
                tc = np.zeros(m.nv)
                for ci, q_ in enumerate(cp):
                    mj.mj_jac(m, d, jacp, jacr, q_, cbid[ci]); tc += jacp.T @ f[ci]
                tau = np.clip((qfi - tc)[dadr], -tlim, tlim)
        if tau is None:                                   # flight/LP-miss frame: no static split exists
            tau = np.zeros(len(WBJ)); misses += 1
        tab[b] = tau
    for k in range(len(WBJ)):
        tab[:, k] = np.interp(np.arange(NB), bins, tab[bins, k])
    ffdq = tab[:, 0:12] / kp                              # LEG slots 0..11 -- the corridored joints
    return gd, ffdq, misses, WBJ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut"); ap.add_argument("--out", default=None)
    ap.add_argument("--kp", type=float, default=200.0)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--mjcf", default=None); ap.add_argument("--urdf", default=None)
    a = ap.parse_args()
    gd, ffdq, misses, WBJ = compute_ffdq(a.lut, a.kp, a.stride, a.mjcf, a.urdf)
    gd["ffdq_lut"] = [[float(v) for v in r] for r in ffdq]
    gd["ffdq_kp"] = a.kp
    out = a.out or a.lut
    json.dump(gd, open(out, "w"))
    pk = np.abs(ffdq).max(axis=0)
    worst = np.argsort(-pk)[:4]
    print("GHOST-FF: wrote ffdq_lut (%d x 12) into %s   kp=%.0f  LP-miss frames=%d" %
          (ffdq.shape[0], os.path.basename(out), a.kp, misses))
    print("  peak |ffdq| per joint (rad):")
    for k in worst:
        print("    %-26s %.3f   (vs corridor 0.12 / 0.20)" % (WBJ[k], pk[k]))


if __name__ == "__main__":
    main()
