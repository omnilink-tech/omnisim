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

"""Engine-free PREVIEW of the stair-climb ghost -- the owner sign-off artifact
(ghost-first rule: design + show the reference, get sign-off, THEN train).

FK's the ghost's whole-body lut (mujoco kinematics, no dynamics) at frames across
the climb and draws a side-view (x-z) montage of the G1 skeleton ascending the
shared StairProfile staircase. Confirms, before any GPU time, that the reference
(a) lands its feet ON the treads, (b) rises tread-by-tread, (c) keeps a plausible
climbing posture. Saves a PNG.

  python projects/policies/training/preview_stair_climb_ghost.py \
      [projects/policies/controllers/g1_ghost/ghost_stair_climb_lut.json] \
      [--riser 0.10 --going 0.26 --n-steps 5] [--out preview.png]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.environ.get("OMNISIM_HOME", os.path.abspath(os.path.join(HERE, "..", "..", "..")))
sys.path.insert(0, RT)
sys.path.insert(0, os.path.join(RT, "projects", "policies", "research", "shadowing"))
from hill_terrain import StairProfile  # noqa: E402

# skeleton bodies to draw + the bone connectivity for the stick figure
SPEC = json.load(open(os.path.join(HERE, "specs", "retarget_g1.json")))
B = SPEC["bodies"]
BONES = [("pelvis", "hip_l"), ("hip_l", "knee_l"), ("knee_l", "foot_l"),
         ("pelvis", "hip_r"), ("hip_r", "knee_r"), ("knee_r", "foot_r"),
         ("pelvis", "torso"), ("torso", "shoulder_l"), ("shoulder_l", "elbow_l"),
         ("elbow_l", "hand_l"), ("torso", "shoulder_r"), ("shoulder_r", "elbow_r"),
         ("elbow_r", "hand_r")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut", nargs="?",
                    default=os.path.join(RT, "projects/policies/controllers/g1_ghost/ghost_stair_climb_lut.json"))
    ap.add_argument("--riser", type=float, default=0.10)
    ap.add_argument("--going", type=float, default=0.26)
    ap.add_argument("--n-steps", type=int, default=5)
    ap.add_argument("--frames", type=int, default=9)
    ap.add_argument("--out", default=os.path.join(RT, "_scratch", "stair_climb_ghost_preview.png"))
    a = ap.parse_args()

    d = json.load(open(a.lut))
    wb = np.array(d["wb_lut"], np.float32)
    JN = d["wb_joints"]
    root = np.array(d["root_lut"], np.float32)          # x, y, z, yaw
    att = np.array(d.get("att_lut", np.zeros((len(wb), 2))), np.float32)
    nb = len(wb)

    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(RT, SPEC["model"]))
    dd = mujoco.MjData(m)
    jadr = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): m.jnt_qposadr[j]
            for j in range(m.njnt) if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)}
    bid = {k: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, v) for k, v in B.items()}
    fixed_base = (m.nq == len([x for x in jadr]))       # fk_only urdf is fixed-base

    def fk_world(bin_i):
        dd.qpos[:] = 0.0
        for j, nm in enumerate(JN):
            if nm in jadr:
                dd.qpos[jadr[nm]] = wb[bin_i, j]
        mujoco.mj_kinematics(m, dd)
        # pelvis-relative -> world: pitch about y by att pitch, translate by root (x,z)
        pitch = float(att[bin_i, 1])
        rx, rz = float(root[bin_i, 0]), float(root[bin_i, 2])
        pel = dd.xpos[bid["pelvis"]].copy()
        c, s = math.cos(pitch), math.sin(pitch)
        pts = {}
        for k, b in bid.items():
            p = dd.xpos[b] - pel                        # relative to pelvis
            xw = c * p[0] + s * p[2] + rx               # rotate about y, add root
            zw = -s * p[0] + c * p[2] + rz
            pts[k] = (xw, zw)
        return pts

    sp = StairProfile(riser=a.riser, going=a.going, n_steps=a.n_steps, x_start=0.12)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception as e:
        print("matplotlib unavailable (%r) -- printing numeric foot-vs-tread check instead" % e)
        _numeric_check(fk_world, sp, nb, a)
        return

    fig, ax = plt.subplots(figsize=(11, 6))
    # staircase (filled step blocks)
    for bx in sp._boxes():
        ax.add_patch(Rectangle((bx["cx"] - bx["hx"], 0), 2 * bx["hx"], 2 * bx["hz"],
                               facecolor="0.75", edgecolor="0.45", zorder=1))
    ax.axhline(0, color="0.5", lw=1, zorder=0)
    idxs = np.linspace(0, nb - 1, a.frames).astype(int)
    cmap = plt.cm.viridis(np.linspace(0.05, 0.95, len(idxs)))
    footgap = []
    for fi, bi in enumerate(idxs):
        pts = fk_world(bi)
        for (p, q) in BONES:
            x0, z0 = pts[p]; x1, z1 = pts[q]
            ax.plot([x0, x1], [z0, z1], "-", color=cmap[fi], lw=2.0, alpha=0.9, zorder=3)
        for k in ("foot_l", "foot_r", "hand_l", "hand_r"):
            ax.plot(*pts[k], "o", color=cmap[fi], ms=4, zorder=4)
        for k in ("foot_l", "foot_r"):
            fx, fz = pts[k]
            footgap.append((fx, (fz - SPEC["sole"]) - sp.height(fx)))
    ax.set_aspect("equal"); ax.set_xlabel("x (m)  climb direction ->"); ax.set_ylabel("z (m)")
    ax.set_title("G1 stair-climb GHOST (%d frames): step-to gait, feet on treads, base rises %.2fm"
                 % (a.frames, sp.rise))
    ax.set_xlim(-0.3, sp.x_top + sp.landing + 0.2); ax.set_ylim(-0.05, root[:, 2].max() + 0.35)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.tight_layout(); fig.savefig(a.out, dpi=110)
    fg = np.array([g for _, g in footgap])
    print("PREVIEW -> %s" % a.out)
    print("foot-vs-tread gap over drawn frames: mean=%+.1f mm  min=%+.1f mm  max=%+.1f mm "
          "(stance feet ~0; +swing apex, -penetration)" % (fg.mean()*1000, fg.min()*1000, fg.max()*1000))


def _numeric_check(fk_world, sp, nb, a):
    gaps = []
    for bi in range(0, nb, max(1, nb // 40)):
        pts = fk_world(bi)
        for k in ("foot_l", "foot_r"):
            fx, fz = pts[k]
            gaps.append((fz - SPEC["sole"]) - sp.height(fx))
    g = np.array(gaps)
    print("foot-vs-tread gap: mean=%+.1f mm min=%+.1f mm max=%+.1f mm" %
          (g.mean()*1000, g.min()*1000, g.max()*1000))


if __name__ == "__main__":
    main()
