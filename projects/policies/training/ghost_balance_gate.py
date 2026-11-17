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

"""GATE 1d: BALANCE / DYNAMIC-CONSISTENCY of a sequence ghost -- the missing stage of the
skeleton->ghost transitioning algorithm (owner root-cause call, 2026-07-04).

WHY: the retarget transfers KINEMATICS (where the limbs are) but not DYNAMICS (where the
weight is). A human's mocap trajectory is dynamically consistent for the HUMAN -- their mass
distribution, foot size, ankle authority; the weight shifts are baked invisibly into the
trajectory. Mapped onto a robot with a different COM height and small feet, the same poses
can put the robot's center of mass outside its own support polygon for whole beats -- every
kinematic gate passes (salsa 60_03: skelmatch 96%) while every dynamic probe fails (segment
isolation: only the wide-stance intro masters; the core dies ON-pose at gmatch 0.85+).

WHAT: per bin, using the ROBOT's own mass model (mujoco inertials) and its own foot
geometry: COM trajectory, contact schedule (sole height), support polygon (union of
in-contact foot rectangles), and two margins:
  static margin  -- signed distance of the COM ground projection to the support polygon
                    (quasi-static balance; the binding constraint for holds/slow beats)
  zmp margin     -- same for the Zero-Moment-Point approximation
                    xy_zmp = xy_com - (z_com/g) * a_com_xy  (dynamic balance)
Reports per-16th violation stats so the numbers line up directly with the feasibility map
and the segment-isolation table. Robot-agnostic: model path + foot dims are arguments.

Usage:
  python ghost_balance_gate.py <lut.json> [--model <fk_urdf>] [--foot-lx 0.085 --foot-ly 0.03]
"""
import argparse
import json
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError:
    sys.exit("needs mujoco")

HERE = os.environ.get("OMNISIM_HOME", os.getcwd())


def support_margin(p, rects):
    """Signed distance of point p (xy) to the union of rectangles [(cx,cy,yaw,lx,ly)].
    Positive = inside (distance to nearest edge); negative = outside (distance to union).
    Union handled as max over rects of the per-rect signed distance -- exact for points
    inside any rect; a tight lower bound outside (fine for a margin gate). For two-foot
    support this misses the convex-hull bridge between the feet -- so ALSO check the hull
    of both rects' corners (double support standing between the feet is balanced)."""
    if not rects:
        return -1.0
    best = -1e9
    corners = []
    for cx, cy, yaw, lx, ly in rects:
        c, s = np.cos(-yaw), np.sin(-yaw)
        dx = c * (p[0] - cx) - s * (p[1] - cy)
        dy = s * (p[0] - cx) + c * (p[1] - cy)
        ex, ey = lx - abs(dx), ly - abs(dy)
        d = min(ex, ey) if (ex > 0 and ey > 0) else -np.hypot(max(0, -ex), max(0, -ey))
        best = max(best, d)
        cc, ss = np.cos(yaw), np.sin(yaw)
        for sx in (-lx, lx):
            for sy in (-ly, ly):
                corners.append((cx + cc * sx - ss * sy, cy + ss * sx + cc * sy))
    if len(rects) == 2 and best < 0:
        # convex hull of both feet: cross products against hull edges
        pts = np.array(corners)
        hull = _hull(pts)
        inside = all(np.cross(hull[(i + 1) % len(hull)] - hull[i], np.array(p) - hull[i]) >= 0
                     for i in range(len(hull)))
        if inside:
            best = min(np.abs(np.cross(hull[(i + 1) % len(hull)] - hull[i],
                                       np.array(p) - hull[i]))
                       / (np.linalg.norm(hull[(i + 1) % len(hull)] - hull[i]) + 1e-9)
                       for i in range(len(hull)))
    return float(best)


def _hull(pts):
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and np.cross(lo[-1] - lo[-2], p - lo[-2]) <= 0:
            lo.pop()
        lo.append(p)
    for p in pts[::-1]:
        while len(up) >= 2 and np.cross(up[-1] - up[-2], p - up[-2]) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut")
    ap.add_argument("--model", default=None)
    ap.add_argument("--robot-spec", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                         "specs", "retarget_g1.json"))
    ap.add_argument("--foot-lx", type=float, default=None)
    ap.add_argument("--foot-ly", type=float, default=None)
    ap.add_argument("--sole", type=float, default=None)
    ap.add_argument("--contact-z", type=float, default=0.06, help="sole below this = in contact")
    a = ap.parse_args()

    # robot constants from the retarget spec (any robot = a new spec file); CLI overrides win
    spec = json.load(open(a.robot_spec))
    a.model = a.model or os.path.join(HERE, spec["model"])
    a.foot_lx = a.foot_lx if a.foot_lx is not None else float(spec.get("foot_lx", 0.085))
    a.foot_ly = a.foot_ly if a.foot_ly is not None else float(spec.get("foot_ly", 0.030))
    a.sole = a.sole if a.sole is not None else float(spec.get("sole", 0.035))
    SB = spec["bodies"]

    m = mujoco.MjModel.from_xml_path(a.model)
    d = mujoco.MjData(m)
    bid = lambda b: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
    jadr = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): m.jnt_qposadr[j] for j in range(m.njnt)}
    FT = {"L": bid(SB["foot_l"]), "R": bid(SB["foot_r"])}
    MTOT = float(m.body_mass.sum())

    lut = json.load(open(a.lut))
    W = np.array(lut["wb_lut"], np.float32)
    R = np.array(lut["root_lut"], np.float32)
    nb = len(W)
    dtb = float(lut["cycle_s"]) / nb

    com = np.zeros((nb, 3)); feet = np.zeros((nb, 2, 3)); fyaw = np.zeros((nb, 2))
    for i in range(nb):
        d.qpos[:] = 0
        for jn, v in zip(lut["wb_joints"], W[i]):
            if jn in jadr:
                d.qpos[jadr[jn]] = v
        mujoco.mj_forward(m, d)
        cy, sy = np.cos(R[i, 3]), np.sin(R[i, 3])
        rot = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
        base = np.array([R[i, 0], R[i, 1], R[i, 2]])
        cl = np.sum(m.body_mass[:, None] * d.xipos, 0) / MTOT     # COM, base frame
        com[i] = rot @ cl + base
        for k, s in enumerate("LR"):
            feet[i, k] = rot @ d.xpos[FT[s]] + base
            Rf = (rot @ d.xmat[FT[s]].reshape(3, 3))
            fyaw[i, k] = np.arctan2(Rf[1, 0], Rf[0, 0])

    acc = np.gradient(np.gradient(com[:, :2], dtb, axis=0), dtb, axis=0)
    acc = np.clip(acc, -8, 8)
    g = 9.81
    stat_m = np.zeros(nb); zmp_m = np.zeros(nb); nsup = np.zeros(nb, int)
    for i in range(nb):
        rects = []
        for k in range(2):
            if feet[i, k, 2] - a.sole < a.contact_z - a.sole:     # sole height above ground
                rects.append((feet[i, k, 0], feet[i, k, 1], fyaw[i, k], a.foot_lx, a.foot_ly))
        nsup[i] = len(rects)
        stat_m[i] = support_margin(com[i, :2], rects)
        zmp = com[i, :2] - (com[i, 2] / g) * acc[i]
        zmp_m[i] = support_margin(zmp, rects)

    seg = np.array_split(np.arange(nb), 16)
    print("GATE1d balance: %s  (robot %.1f kg, foot %.2fx%.2f)" % (os.path.basename(a.lut), MTOT, 2 * a.foot_lx, 2 * a.foot_ly))
    print("  static margin: mean %+.3f  p5 %+.3f  violated %.0f%% of bins"
          % (stat_m.mean(), np.percentile(stat_m, 5), (stat_m < 0).mean() * 100))
    print("  zmp margin:    mean %+.3f  p5 %+.3f  violated %.0f%% of bins"
          % (zmp_m.mean(), np.percentile(zmp_m, 5), (zmp_m < 0).mean() * 100))
    print("  single-support: %.0f%% of bins | no-contact: %.0f%%"
          % ((nsup == 1).mean() * 100, (nsup == 0).mean() * 100))
    print("  per-16th %%static-violated: " + " ".join("%2.0f" % ((stat_m[s] < 0).mean() * 100) for s in seg))
    print("  per-16th mean static margin (mm): " + " ".join("%+4.0f" % (stat_m[s].mean() * 1000) for s in seg))


if __name__ == "__main__":
    main()
