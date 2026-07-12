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

"""SHOW the improved shadows. Renders the legacy shadow vs the three upgrades
(A LIPM, B achieved, C human-3D) so the new LATERAL motion is visible:
  - shadow_curves.png : hip-roll + hip-yaw + ankle-roll over the gait cycle.
  - shadow_front.png  : FRONT-view stick figures (where the lateral lives).
  - shadow_side.png   : SIDE-view (confirms the human stride is preserved).
  - shadow_front.gif  : front-view walk, all four side by side.

Rescued from _scratch into the tracked tree (visualisation tooling for the
G1-improved-shadow work; see docs/developer/g1-improved-shadow.md). Paths are
repo-relative; figures land in <repo>/_scratch by default (override via OUT env).
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, _REPO)
import projects.policies.control.gait.g1_human_gait as g

OUT = os.environ.get("OUT", os.path.join(_REPO, "_scratch"))
os.makedirs(OUT, exist_ok=True)
HW = 0.09          # half hip width (m)
L1, L2, LF = g.L1, g.L2, 0.14

MODES = [
    ("legacy (sway only)", g.GaitParams(style="winter"), "#888888"),
    ("A: LIPM weight transfer", g.GaitParams(style="winter", lateral="lipm"), "#1f77b4"),
    ("B: achieved (our 297m walk)", g.GaitParams(style="achieved"), "#2ca02c"),
    ("C: human 3D kinematics", g.GaitParams(style="winter", lateral="human", yaw="human"), "#d62728"),
]
PHASES = np.linspace(0, 2 * math.pi, 96, endpoint=False)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def leg_3d(q6, side):
    """3D points hip->knee->ankle->toe for one leg. q6 = [hp,hr,hy,kn,ap,ar].
    Frame: x fwd, y left, z up. side=+1 left, -1 right. Roll/yaw signs mirror."""
    hp, hr, hy, kn, ap, ar = q6
    # hip rotation: yaw(z) * roll(x) * pitch(y) acting on the down-thigh vector.
    Rhip = _rz(hy) @ _rx(hr) @ _ry(g.HIP_SIGN * hp)
    thigh = Rhip @ np.array([0, 0, -L1])
    Rknee = Rhip @ _ry(g.KNEE_SIGN * kn)
    shank = Rknee @ np.array([0, 0, -L2])
    # ankle: pitch (about its y) + roll (about its x) for the foot/toe
    Rank = Rknee @ _ry(g.ANKLE_AX * ap) @ _rx(ar)
    foot = Rank @ np.array([LF, 0, 0])
    hip = np.array([0.0, side * HW, 0.0])
    knee = hip + thigh
    ankle = knee + shank
    toe = ankle + foot
    return np.array([hip, knee, ankle, toe])


def pose_3d(p, ph):
    legs, _, _ = g.targets_np(ph, p, t_since_start=1e6)
    L = leg_3d(legs[0:6], +1)
    R = leg_3d(legs[6:12], -1)
    return L, R


# -- 1. Curves: hip-roll, hip-yaw, ankle-roll over the cycle --------------
fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
titles = ["Hip ROLL  (lateral weight transfer)",
          "Hip YAW  (was ZERO -> the 20deg gap)",
          "Ankle ROLL  (flat-foot counter-rotation)"]
jidx = [g._L_HR, g._L_HY, g._L_AR]
for ax, ji, ti in zip(axes, jidx, titles):
    for name, p, col in MODES:
        v = np.degrees([g.targets_np(ph, p)[0][ji] for ph in PHASES])
        ax.plot(np.linspace(0, 100, len(PHASES)), v, lw=2.3, color=col, label=name)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title(ti, fontsize=10); ax.set_xlabel("% gait cycle")
    ax.set_ylabel("deg"); ax.grid(alpha=0.3); ax.set_xlim(0, 100)
axes[0].legend(fontsize=8, loc="upper right")
fig.suptitle("The improved shadow: a PRINCIPLED, non-zero frontal/transverse plane "
             "(legacy was ~flat -> impossible to match without falling)", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(f"{OUT}/shadow_curves.png", dpi=110)
plt.close(fig)
print("saved shadow_curves.png")


# -- 2. FRONT view pose strip (the lateral plane: y left, z up) -----------
def draw_front(ax, p, ph, col):
    L, R = pose_3d(p, ph)
    for P, c in ((L, col), (R, col)):
        ax.plot(P[:, 1], P[:, 2], "-o", color=c, lw=2.3, ms=3)
    ax.plot([-HW, HW], [0, 0], "k-", lw=2)          # pelvis bar
    ax.axhline(-0.78, color="saddlebrown", lw=1.5)  # ground
    ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.85, 0.10)
    ax.set_aspect("equal"); ax.axis("off")


fig, axes = plt.subplots(len(MODES), 8, figsize=(15, 8), sharex=True, sharey=True)
strip = np.linspace(0, 2 * math.pi, 8, endpoint=False)
for r, (name, p, col) in enumerate(MODES):
    for cax, ph in zip(axes[r], strip):
        draw_front(cax, p, ph, col)
    axes[r][0].set_title(name, fontsize=10, loc="left", color=col)
fig.suptitle("FRONT view (looking at the robot) -- one gait cycle. Watch the legs "
             "lean/cross: legacy barely moves; A/C transfer weight cleanly; B "
             "shows the recorded SPLAY.", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{OUT}/shadow_front.png", dpi=110)
plt.close(fig)
print("saved shadow_front.png")


# -- 3. SIDE view (confirms the human sagittal stride is intact) ----------
def draw_side(ax, p, ph, col):
    L, R = pose_3d(p, ph)
    ax.plot(L[:, 0], L[:, 2], "-o", color="#d62728", lw=2.2, ms=3)
    ax.plot(R[:, 0], R[:, 2], "-o", color="#1f77b4", lw=2.2, ms=3)
    ax.plot([0], [0], "ks", ms=6)
    ax.axhline(-0.78, color="saddlebrown", lw=1.5)
    ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.85, 0.10)
    ax.set_aspect("equal"); ax.axis("off")


fig, axes = plt.subplots(len(MODES), 8, figsize=(15, 8), sharex=True, sharey=True)
for r, (name, p, col) in enumerate(MODES):
    for cax, ph in zip(axes[r], strip):
        draw_side(cax, p, ph, col)
    axes[r][0].set_title(name, fontsize=10, loc="left", color=col)
fig.suptitle("SIDE view -- the human stride (knee double-bend + push-off) is "
             "preserved across all variants (red=L, blue=R).", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{OUT}/shadow_side.png", dpi=110)
plt.close(fig)
print("saved shadow_side.png")


# -- 4. Animated front-view GIF: all four side by side --------------------
try:
    from matplotlib.animation import FuncAnimation, PillowWriter
    figA, axesA = plt.subplots(1, 4, figsize=(15, 4.5))
    NF = 48

    def anim_frame(fr):
        ph = 2 * math.pi * fr / NF
        for ax, (name, p, col) in zip(axesA, MODES):
            draw_front(ax, p, ph, col)
            ax.set_title(name, fontsize=9, color=col)
        figA.suptitle("Improved shadows -- FRONT view (lateral weight transfer)",
                      fontsize=12)
    anim = FuncAnimation(figA, anim_frame, frames=NF, interval=60)
    anim.save(f"{OUT}/shadow_front.gif", writer=PillowWriter(fps=16))
    plt.close(figA)
    print("saved shadow_front.gif")
except Exception as e:
    print(f"GIF failed (non-fatal): {e}")
