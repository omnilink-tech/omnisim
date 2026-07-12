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

"""SHOW the three humanoid walking shadows (G1, H1, Valkyrie) side by side, as
render-independent 3D stick figures driven by each robot's own gait model. This
is the "ghost first" preview that does NOT depend on the OmniSim renderer (so
Valkyrie's heavy meshes are irrelevant). Outputs to <repo>/_scratch (or $OUT):

  humanoid_shadow_side.png  : SIDE view pose strip (the natural sagittal stride:
                              knee double-bend + ankle push-off), 3 robots x 8 phases.
  humanoid_shadow_front.png : FRONT view pose strip (the lateral weight transfer
                              / hip-roll that keeps the CoM over the stance foot).
  humanoid_shadow_walk.gif  : all three walking forward, side by side.

Run: python projects/policies/control/gait/tools/show_humanoid_shadows.py
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

import projects.policies.control.gait.g1_human_gait as g1
import projects.policies.control.gait.h1_human_gait as h1
import projects.policies.control.gait.valkyrie_human_gait as valk

OUT = os.environ.get("OUT", os.path.join(_REPO, "_scratch"))
os.makedirs(OUT, exist_ok=True)

# (module, GaitParams, half-hip-width m, hip-height-above-ground m, color, name)
ROBOTS = [
    (g1,   g1.GaitParams(style="winter", lateral="human", yaw="human"),
     0.09, g1.GaitParams().pelvis_height - g1.HIP_DROP, "#1f77b4", "G1 (34 kg)"),
    (h1,   h1.GaitParams(),
     0.203, h1.GaitParams().pelvis_height - h1.HIP_DROP, "#2ca02c", "H1 (47 kg)"),
    (valk, valk.GaitParams(),
     0.102, valk.GaitParams().pelvis_height - valk.HIP_DROP, "#d62728", "Valkyrie (130 kg)"),
]
LF = 0.16   # drawn foot length


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def leg_3d(mod, q6, side, HW):
    """3D points hip->knee->ankle->toe for one leg, in the pelvis frame
    (x fwd, y left, z up). Uses the robot's own L1/L2/sign constants."""
    hp, hr, hy, kn, ap, ar = q6
    Rhip = _rz(hy) @ _rx(hr) @ _ry(mod.HIP_SIGN * hp)
    thigh = Rhip @ np.array([0, 0, -mod.L1])
    Rknee = Rhip @ _ry(mod.KNEE_SIGN * kn)
    shank = Rknee @ np.array([0, 0, -mod.L2])
    Rank = Rknee @ _ry(mod.ANKLE_AX * ap) @ _rx(ar)
    foot = Rank @ np.array([LF, 0, 0])
    hip = np.array([0.0, side * HW, 0.0])
    knee = hip + thigh
    ankle = knee + shank
    toe = ankle + foot
    return np.array([hip, knee, ankle, toe])


def pose(mod, p, ph, HW):
    legs, arms, _ = mod.targets_np(ph, p, t_since_start=1e6)
    return leg_3d(mod, legs[0:6], +1, HW), leg_3d(mod, legs[6:12], -1, HW), arms


def draw_side(ax, mod, p, ph, HW, ground, col):
    L, R, _ = pose(mod, p, ph, HW)
    ax.plot(L[:, 0], L[:, 2], "-o", color="#d62728", lw=2.2, ms=3)
    ax.plot(R[:, 0], R[:, 2], "-o", color="#1f77b4", lw=2.2, ms=3)
    ax.plot([0, 0], [0, 0.28], "k-", lw=3)             # torso stub
    ax.plot([0], [0], "ks", ms=6)
    ax.axhline(ground, color="saddlebrown", lw=1.5)
    ax.set_xlim(-0.55, 0.55); ax.set_ylim(ground - 0.05, 0.40)
    ax.set_aspect("equal"); ax.axis("off")


def draw_front(ax, mod, p, ph, HW, ground, col):
    L, R, _ = pose(mod, p, ph, HW)
    ax.plot(L[:, 1], L[:, 2], "-o", color=col, lw=2.3, ms=3)
    ax.plot(R[:, 1], R[:, 2], "-o", color=col, lw=2.3, ms=3)
    ax.plot([-HW, HW], [0, 0], "k-", lw=2)             # pelvis bar
    ax.plot([0, 0], [0, 0.28], "k-", lw=3)             # torso
    ax.axhline(ground, color="saddlebrown", lw=1.5)
    ax.set_xlim(-0.5, 0.5); ax.set_ylim(ground - 0.05, 0.40)
    ax.set_aspect("equal"); ax.axis("off")


strip = np.linspace(0, 2 * math.pi, 8, endpoint=False)

# -- SIDE view strip ------------------------------------------------------
fig, axes = plt.subplots(len(ROBOTS), 8, figsize=(15, 7), sharex=True, sharey=True)
for r, (mod, p, HW, hh, col, name) in enumerate(ROBOTS):
    for cax, ph in zip(axes[r], strip):
        draw_side(cax, mod, p, ph, HW, -hh, col)
    axes[r][0].set_title(name, fontsize=11, loc="left", color=col)
fig.suptitle("Walking shadows -- SIDE view, one gait cycle (red=L leg, blue=R). "
             "Natural human stride: stance knee EXTENDS (tall), swing knee flexes, "
             "ankle push-off at toe-off.", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{OUT}/humanoid_shadow_side.png", dpi=110)
plt.close(fig)
print("saved humanoid_shadow_side.png")

# -- FRONT view strip -----------------------------------------------------
fig, axes = plt.subplots(len(ROBOTS), 8, figsize=(15, 7), sharex=True, sharey=True)
for r, (mod, p, HW, hh, col, name) in enumerate(ROBOTS):
    for cax, ph in zip(axes[r], strip):
        draw_front(cax, mod, p, ph, HW, -hh, col)
    axes[r][0].set_title(name, fontsize=11, loc="left", color=col)
fig.suptitle("Walking shadows -- FRONT view, one gait cycle. The lateral weight "
             "transfer (hip-roll) rocks the body over each stance foot -- the "
             "principled, achievable frontal plane (not a flat/impossible one).",
             fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{OUT}/humanoid_shadow_front.png", dpi=110)
plt.close(fig)
print("saved humanoid_shadow_front.png")

# -- Walking GIF: all three striding forward, side by side ----------------
try:
    from matplotlib.animation import FuncAnimation, PillowWriter
    figA, axesA = plt.subplots(1, 3, figsize=(15, 5))
    NF = 60

    def frame(fr):
        for ax, (mod, p, HW, hh, col, name) in zip(axesA, ROBOTS):
            ax.clear()
            t = 2.5 * fr / NF                    # seconds of walk
            ph = (mod.DS_PHASE + 2 * math.pi * p.freq * t) % (2 * math.pi)
            x0 = p.vx * t
            L, R, _ = pose(mod, p, ph, HW)
            ax.plot(L[:, 0] + x0, L[:, 2], "-o", color="#d62728", lw=2.4, ms=3)
            ax.plot(R[:, 0] + x0, R[:, 2], "-o", color="#1f77b4", lw=2.4, ms=3)
            ax.plot([x0, x0], [0, 0.30], "k-", lw=3)
            ax.axhline(-hh, color="saddlebrown", lw=1.5)
            ax.set_xlim(x0 - 0.7, x0 + 0.7); ax.set_ylim(-hh - 0.05, 0.42)
            ax.set_aspect("equal"); ax.axis("off")
            ax.set_title(f"{name}  v={p.vx} m/s", fontsize=10, color=col)
        figA.suptitle("The designed walking shadows (kinematic reference for RL)", fontsize=12)

    anim = FuncAnimation(figA, frame, frames=NF, interval=60)
    anim.save(f"{OUT}/humanoid_shadow_walk.gif", writer=PillowWriter(fps=16))
    plt.close(figA)
    print("saved humanoid_shadow_walk.gif")
except Exception as e:
    print(f"GIF failed (non-fatal): {e}")
