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

"""Derive a humanoid's sagittal-leg gait-kinematics constants DIRECTLY from
its URDF kinematic tree -- no MuJoCo, no meshes needed (H1's meshes have a
bad path and Valkyrie ships .dae, so neither URDF loads in MuJoCo).

The constants are exactly those frozen at the top of g1_human_gait.py:
  L1, L2          thigh / shank sagittal lengths (hip_pitch->knee->ankle)
  THIGH_OFF       thigh angle from vertical at q=0 (+ = knee ahead of hip)
  SHANK_OFF       shank angle relative to thigh at q=0
  HIP_SIGN/KNEE_SIGN/ANKLE_AX   joint-sign conventions (from the joint axes)
  HIP_DROP        pelvis origin -> hip_pitch anchor, vertical drop
  SOLE_DROP       ankle_pitch anchor -> foot sole (flat foot)

Sign convention: the gait FK measures the thigh angle as +forward (+x tilt
from vertical). A pitch hinge with axis +y rotates a downward leg toward -x
for +q, so +q tilts the leg BACKWARD -> sign = -1 (the G1's are all -1). For
axis -y it is +1. This is read straight off <axis>, robust to which way the
modeller drew it. The result is cross-checked against the robot's known
flat-foot stand pose (ankle = -(hip+knee)).

Usage:
  python projects/policies/control/gait/tools/urdf_leg_calib.py h1
  python projects/policies/control/gait/tools/urdf_leg_calib.py valkyrie
"""
from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]

# (urdf, [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch], foot_link,
#  stand: (hip, knee, ankle))  -- stand pose from the humanoid_stand_deploy spec.
ROBOTS = {
    "h1": dict(
        urdf="projects/robots/unitree/h1/urdf/h1.urdf",
        chain=["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
               "left_knee_joint", "left_ankle_joint"],
        foot="left_ankle_link",
        stand=(-0.30, 0.60, -0.30),
    ),
    "valkyrie": dict(
        urdf="projects/robots/nasa/valkyrie/urdf/valkyrie.urdf",
        chain=["leftHipYaw", "leftHipRoll", "leftHipPitch",
               "leftKneePitch", "leftAnklePitch"],
        foot="leftFoot",
        stand=(-0.35, 0.70, -0.35),
    ),
}


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin(el):
    o = el.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if o is not None:
        if o.get("xyz"):
            xyz = np.array([float(v) for v in o.get("xyz").split()])
        if o.get("rpy"):
            rpy = np.array([float(v) for v in o.get("rpy").split()])
    return xyz, rpy


def calib(name):
    cfg = ROBOTS[name]
    tree = ET.parse(ROOT / cfg["urdf"])
    root = tree.getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}

    # Accumulate fixed (q=0) transform pelvis -> each joint anchor down the chain.
    T = np.eye(4)
    anch = {}
    axes = {}
    limits = {}
    for jn in cfg["chain"]:
        j = joints[jn]
        xyz, rpy = _origin(j)
        Tj = np.eye(4)
        Tj[:3, :3] = _rpy_to_R(*rpy)
        Tj[:3, 3] = xyz
        T = T @ Tj
        anch[jn] = T[:3, 3].copy()
        ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        axes[jn] = (T[:3, :3] @ ax)        # axis in pelvis frame
        lim = j.find("limit")
        limits[jn] = (float(lim.get("lower")), float(lim.get("upper")),
                      float(lim.get("effort")))

    hp = anch[cfg["chain"][2]]    # hip_pitch
    kn = anch[cfg["chain"][3]]    # knee
    ap = anch[cfg["chain"][4]]    # ankle_pitch
    # Foot sole: ankle anchor -> bottom of the foot collision box (flat foot).
    foot = links[cfg["foot"]]
    col = foot.find("collision")
    box = col.find("geometry").find("box")
    bxyz, _ = _origin(col)
    bsize = np.array([float(v) for v in box.get("size").split()])
    # foot link frame == its parent joint child frame; the ankle_pitch->foot
    # offset is the chain after ankle_pitch (ankle_roll origin, usually 0).
    # Sole z below the ankle_pitch anchor:
    sole_below_ankle = -(bxyz[2] - bsize[2] / 2.0)

    L1 = math.hypot((kn - hp)[0], (kn - hp)[2])
    L2 = math.hypot((ap - kn)[0], (ap - kn)[2])
    thigh_ang = math.atan2((kn - hp)[0], -(kn - hp)[2])      # +x fwd from vertical
    shank_ang = math.atan2((ap - kn)[0], -(ap - kn)[2])
    THIGH_OFF = thigh_ang
    SHANK_OFF = shank_ang - thigh_ang
    HIP_DROP = -hp[2]
    SOLE_DROP = sole_below_ankle

    def pitch_sign(jn):
        # +q about axis +y tilts a downward leg toward -x (backward) -> -1.
        return -1.0 if axes[jn][1] >= 0 else 1.0

    HIP_SIGN = pitch_sign(cfg["chain"][2])
    KNEE_SIGN = pitch_sign(cfg["chain"][3])
    ANKLE_AX = pitch_sign(cfg["chain"][4])

    print(f"# ── frozen {name.upper()} leg constants (from URDF kinematics) ──")
    print(f"L1 = {L1:.5f}          # thigh hip_pitch->knee")
    print(f"L2 = {L2:.5f}          # shank knee->ankle")
    print(f"THIGH_OFF = {THIGH_OFF:+.5f}")
    print(f"SHANK_OFF = {SHANK_OFF:+.5f}")
    print(f"HIP_SIGN = {HIP_SIGN:+.0f}")
    print(f"KNEE_SIGN = {KNEE_SIGN:+.0f}")
    print(f"ANKLE_AX = {ANKLE_AX:+.0f}")
    print(f"HIP_DROP = {HIP_DROP:.5f}")
    print(f"SOLE_DROP = {SOLE_DROP:.5f}")
    reach = HIP_DROP + L1 + L2 + SOLE_DROP
    print(f"# straight-leg reach = {reach:.4f} m  (hip_y offset = {hp[1]:+.4f} m)")
    print(f"# leg-joint limits (lo, hi, effort):")
    for jn in cfg["chain"]:
        lo, hi, eff = limits[jn]
        print(f"#   {jn:24s} [{lo:+.3f}, {hi:+.3f}]  eff {eff:.0f}")

    # Cross-check the signs against the known flat-foot stand: with the gait FK
    # theta_shank = THIGH_OFF + HIP_SIGN*hip + SHANK_OFF + KNEE_SIGN*knee, the
    # flat-foot ankle is q = ANKLE_AX*(0 - theta_shank). It must reproduce the
    # spec stand's ankle = -(hip+knee) (within the small THIGH/SHANK offset).
    sh, sk, sa = cfg["stand"]
    theta_sh = THIGH_OFF + HIP_SIGN * sh + SHANK_OFF + KNEE_SIGN * sk
    qa_flat = ANKLE_AX * (0.0 - theta_sh)
    print(f"# stand cross-check: spec ankle {sa:+.3f} vs flat-foot-rule "
          f"{qa_flat:+.3f}  (Δ {abs(qa_flat - sa):.3f} rad)")
    return dict(L1=L1, L2=L2, THIGH_OFF=THIGH_OFF, SHANK_OFF=SHANK_OFF,
                HIP_SIGN=HIP_SIGN, KNEE_SIGN=KNEE_SIGN, ANKLE_AX=ANKLE_AX,
                HIP_DROP=HIP_DROP, SOLE_DROP=SOLE_DROP, reach=reach,
                hip_y=hp[1])


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "h1"
    calib(which)
