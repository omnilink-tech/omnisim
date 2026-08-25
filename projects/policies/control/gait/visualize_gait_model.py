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

"""Visualise the human-gait MODEL: the Winter normative joint-angle curves,
a stick-figure animation driven by them, and the knee before/after.
Outputs PNGs + a GIF to _scratch/."""
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

# ── 1. The MODEL itself: Winter joint-angle curves over the gait cycle ──
pct = np.linspace(0, 100, g._W_N, endpoint=False)
fig, ax = plt.subplots(figsize=(10, 5.5))
ax.plot(pct, g.HIP_TAB, lw=2.5, label="Hip (flexion +)", color="#1f77b4")
ax.plot(pct, g.KNEE_TAB, lw=2.5, label="Knee (flexion +)", color="#d62728")
ax.plot(pct, g.ANKLE_TAB, lw=2.5, label="Ankle (dorsiflexion +)", color="#2ca02c")
ax.axhline(0, color="k", lw=0.5)
# annotate the human signatures
ki = int(0.15 * g._W_N)
ax.annotate("knee weight-acceptance\nbend (~18 deg)", (15, g.KNEE_TAB[ki]),
            xytext=(2, 40), fontsize=9, color="#d62728",
            arrowprops=dict(arrowstyle="->", color="#d62728"))
ks = int(0.72 * g._W_N)
ax.annotate("knee swing flex (~62 deg)", (72, g.KNEE_TAB[ks]),
            xytext=(55, 70), fontsize=9, color="#d62728",
            arrowprops=dict(arrowstyle="->", color="#d62728"))
ap = int(0.63 * g._W_N)
ax.annotate("ANKLE PUSH-OFF\n(plantarflex kick)", (63, g.ANKLE_TAB[ap]),
            xytext=(70, -28), fontsize=9, color="#2ca02c",
            arrowprops=dict(arrowstyle="->", color="#2ca02c"))
ax.axvspan(0, 62, alpha=0.06, color="blue")
ax.axvspan(62, 100, alpha=0.06, color="orange")
ax.text(31, -27, "STANCE (foot on ground, 0-62%)", ha="center", fontsize=9, color="navy")
ax.text(81, -27, "SWING (foot in air)", ha="center", fontsize=9, color="darkorange")
ax.set_xlabel("% of gait cycle  (0 = heel strike)")
ax.set_ylabel("joint angle (degrees)")
ax.set_title("The human-gait MODEL: Winter normative sagittal joint kinematics\n"
             "(measured human walking data -- this is what drives the robot's joints)")
ax.legend(loc="upper center", ncol=3, fontsize=9)
ax.set_xlim(0, 100)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUT}/gait_model_curves.png", dpi=110)
plt.close(fig)
print("saved gait_model_curves.png")


# ── 2. FK: joint positions in the sagittal plane from the model q ──
def leg_points(leg_phi):
    """Return [(hip),(knee),(ankle),(toe)] xz for the model at this leg phase."""
    qh, qk, qa = g._winter_leg_np(leg_phi % 1.0, g.GaitParams(style="winter"), 1.0)
    theta_t = g.THIGH_OFF + g.HIP_SIGN * qh        # thigh from vertical, fwd+
    theta_s = theta_t + g.SHANK_OFF + g.KNEE_SIGN * qk
    hip = np.array([0.0, 0.0])
    knee = hip + g.L1 * np.array([math.sin(theta_t), -math.cos(theta_t)])
    ankle = knee + g.L2 * np.array([math.sin(theta_s), -math.cos(theta_s)])
    fp = theta_s - qa                               # world foot pitch (0=flat)
    toe = ankle + 0.14 * np.array([math.cos(fp), -math.sin(fp)])
    return np.array([hip, knee, ankle, toe])


# ── 3. Pose-strip: 8 snapshots through one cycle (both legs) ──
fig, axes = plt.subplots(1, 8, figsize=(16, 3.4), sharey=True)
phases = np.linspace(0, 1, 8, endpoint=False)
for ax, ph in zip(axes, phases):
    for phi, col, lab in ((ph, "#d62728", "L"), ((ph + 0.5) % 1, "#1f77b4", "R")):
        P = leg_points(phi)
        # pelvis bob: highest at mid-stance -- raise the whole leg origin a touch
        P[:, 1] += 0.0
        ax.plot(P[:, 0], P[:, 1], "-o", color=col, lw=2.2, ms=4)
    ax.plot([0], [0], "ks", ms=7)                   # hip/pelvis
    ax.axhline(P[2, 1] if False else -0.70, color="gray", lw=1)  # ground ref
    ax.set_title(f"{int(ph*100)}%", fontsize=10)
    ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.80, 0.10)
    ax.set_aspect("equal"); ax.axis("off")
fig.suptitle("Stick figure driven by the human-gait model -- one full cycle "
             "(red = left leg, blue = right). Note the knee double-bend + the "
             "ankle push-off at ~62%.", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(f"{OUT}/gait_model_posestrip.png", dpi=110)
plt.close(fig)
print("saved gait_model_posestrip.png")


# ── 4. Before/after: old planner knee vs Winter knee ──
pw = g.GaitParams(style="winter")
pik = g.GaitParams(style="ik")
phi = np.linspace(0, 1, 256, endpoint=False)
# winter knee q over a leg cycle
knee_w = np.array([g._winter_leg_np(p, pw, 1.0)[1] for p in phi])
# ik-style knee: sample targets_np across global phase, read left knee
knee_ik = []
for p in phi:
    legs, _, _ = g.targets_np(p * 2 * math.pi, pik, t_since_start=1e6)
    knee_ik.append(legs[g._L_KN])
knee_ik = np.array(knee_ik)
fig, ax = plt.subplots(figsize=(9, 4.5))
ax.plot(phi * 100, np.degrees(knee_ik - knee_ik.min()), lw=2.5,
        label="v1 planner+IK knee (single smooth bend)", color="#888")
ax.plot(phi * 100, np.degrees(knee_w - knee_w.min()), lw=2.5,
        label="Winter human knee (DOUBLE-bend)", color="#d62728")
ax.set_xlabel("% of gait cycle"); ax.set_ylabel("knee flexion (deg, from min)")
ax.set_title("Why it now looks human: the knee waveform")
ax.legend(); ax.grid(alpha=0.3); ax.set_xlim(0, 100)
fig.tight_layout()
fig.savefig(f"{OUT}/gait_model_knee_compare.png", dpi=110)
plt.close(fig)
print("saved gait_model_knee_compare.png")


# ── 5. Animated GIF: stick figure walking, translating forward ──
try:
    from matplotlib.animation import FuncAnimation, PillowWriter
    figA, axA = plt.subplots(figsize=(6, 5))
    NF = 48
    # estimate forward travel per frame from ankle x excursion
    def draw(frame):
        axA.clear()
        ph = frame / NF
        body_x = 0.9 * ph                            # translate body forward
        for phi, col in ((ph, "#d62728"), ((ph + 0.5) % 1, "#1f77b4")):
            P = leg_points(phi)
            P[:, 0] += body_x
            axA.plot(P[:, 0], P[:, 1], "-o", color=col, lw=3, ms=5)
        axA.plot([body_x], [0], "ks", ms=10)         # pelvis
        axA.plot([body_x, body_x], [0, 0.28], "k-", lw=3)   # torso stub
        axA.axhline(-0.705, color="saddlebrown", lw=2)
        axA.set_xlim(body_x - 0.6, body_x + 0.6)
        axA.set_ylim(-0.80, 0.40); axA.set_aspect("equal"); axA.axis("off")
        axA.set_title("Human-gait model on the G1 leg geometry", fontsize=11)
    anim = FuncAnimation(figA, draw, frames=NF, interval=60)
    anim.save(f"{OUT}/gait_model_animation.gif", writer=PillowWriter(fps=16))
    plt.close(figA)
    print("saved gait_model_animation.gif")
except Exception as e:
    print(f"GIF failed (non-fatal): {e}")
