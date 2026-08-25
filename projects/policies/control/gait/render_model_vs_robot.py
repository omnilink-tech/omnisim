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

"""Render the gait MODEL command (ghost) against the ROBOT's actual pose,
from a live deploy trace (G1_TRACE_REF csv). Produces a pose strip PNG and
a looping GIF: dashed grey = what the Winter model asked for; solid colour
= what the robot physically did (RL-stabilised)."""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_REPO = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(_REPO))
import projects.policies.control.gait.g1_human_gait as g

OUT = str(_REPO / "_scratch")
rows = np.loadtxt(f"{OUT}/g1_gait_trace.csv", delimiter=",")
# columns: t, bx, phase, joint_q[13], baseline[13], targets[13]
t = rows[:, 0]
qact = rows[:, 3:16]       # actual measured joint angles
qref = rows[:, 16:29]      # the MODEL reference (baseline)

# leg joint indices within the 13-vector
L_HP, L_KN, L_AP = 0, 3, 4
R_HP, R_KN, R_AP = 6, 9, 10


def leg_pts(q, hp, kn, ap):
    """hip->knee->ankle->toe xz from hip-pitch/knee/ankle-pitch angles."""
    theta_t = g.THIGH_OFF + g.HIP_SIGN * q[hp]
    theta_s = theta_t + g.SHANK_OFF + g.KNEE_SIGN * q[kn]
    hip = np.array([0.0, 0.0])
    knee = hip + g.L1 * np.array([math.sin(theta_t), -math.cos(theta_t)])
    ankle = knee + g.L2 * np.array([math.sin(theta_s), -math.cos(theta_s)])
    fp = theta_s - q[ap]
    toe = ankle + 0.14 * np.array([math.cos(fp), -math.sin(fp)])
    return np.array([hip, knee, ankle, toe])


def draw_frame(ax, i):
    ax.clear()
    qa, qr = qact[i], qref[i]
    # model reference: ghost (dashed grey), both legs
    for hp, kn, ap in ((L_HP, L_KN, L_AP), (R_HP, R_KN, R_AP)):
        P = leg_pts(qr, hp, kn, ap)
        ax.plot(P[:, 0], P[:, 1], "--", color="#999", lw=2, zorder=1)
    # robot actual: solid coloured
    for hp, kn, ap, c in ((L_HP, L_KN, L_AP, "#d62728"),
                          (R_HP, R_KN, R_AP, "#1f77b4")):
        P = leg_pts(qa, hp, kn, ap)
        ax.plot(P[:, 0], P[:, 1], "-o", color=c, lw=3, ms=4, zorder=3)
    ax.plot([0], [0], "ks", ms=10)
    ax.plot([0, 0], [0, 0.28], "k-", lw=3)
    ax.axhline(-0.705, color="saddlebrown", lw=2)
    ax.set_xlim(-0.5, 0.5); ax.set_ylim(-0.80, 0.40)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"t={t[i]:.1f}s   dashed = MODEL command,  solid = ROBOT actual",
                 fontsize=11)


# pose strip: 8 frames spanning ~2 gait cycles in the steady walk
start = np.searchsorted(t, t[0] + 6.0)        # after launch ramp
span = int(2.0 / 1.3 / 0.016)                 # ~2 cycles in ticks
idxs = np.linspace(start, start + span, 8).astype(int)
fig, axes = plt.subplots(1, 8, figsize=(16, 3.6), sharey=True)
for ax, i in zip(axes, idxs):
    draw_frame(ax, i)
    ax.set_title(f"{t[i]:.2f}s", fontsize=10)
fig.suptitle("Gait MODEL (dashed grey ghost) vs ROBOT actual pose (solid) -- "
             "live deploy. The robot tracks the human-kinematics model; the RL "
             "layer supplies the balance the model cannot.", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.92])
fig.savefig(f"{OUT}/gait_model_vs_robot_strip.png", dpi=110)
plt.close(fig)
print("saved gait_model_vs_robot_strip.png")

# GIF: ~6 s of the steady walk
from matplotlib.animation import FuncAnimation, PillowWriter
gif_start = np.searchsorted(t, t[0] + 6.0)
gif_idx = range(gif_start, min(gif_start + int(6.0 / 0.016), len(t)), 2)
figA, axA = plt.subplots(figsize=(5.5, 5.5))
anim = FuncAnimation(figA, lambda f: draw_frame(axA, list(gif_idx)[f]),
                     frames=len(list(gif_idx)), interval=32)
anim.save(f"{OUT}/gait_model_vs_robot.gif", writer=PillowWriter(fps=30))
plt.close(figA)
print("saved gait_model_vs_robot.gif")
