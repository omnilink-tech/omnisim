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

"""Figures for the BATON paper.

⛔ HONESTY CONTRACT (read before adding a figure)
Every number plotted here must trace to a committed artifact:
  - fig_horizon      <- _scratch/baton_horizon/results.json, produced by
                        scripts/dev/baton_horizon_experiment.sh (an arm x seed sweep
                        of REAL deploy runs). If that file is missing, this script
                        REFUSES to draw the figure. It does not invent a curve, and
                        it does not fall back to a "representative" one.
  - fig_switch       <- the per-switch transient table from baton_metrics.py on the
                        same runs (peak |roll|, |pitch|, min pelvis z in the 2 s after
                        each handover).
  - fig_arch         <- a diagram, no data.
The Shadowing paper carries the same contract, and it is the reason its figures
survived an audit that killed two of them. Keep it.
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DATA = os.path.join(ROOT, "_scratch", "baton_horizon", "results.json")
FIGS = os.path.join(HERE, "figs")
os.makedirs(FIGS, exist_ok=True)

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["DejaVu Serif"], "font.size": 9,
    "axes.titlesize": 10, "axes.labelsize": 9.5, "axes.grid": True,
    "grid.alpha": 0.30, "grid.linewidth": 0.5, "axes.axisbelow": True,
    "figure.dpi": 200, "savefig.dpi": 200, "savefig.bbox": "tight",
    "legend.fontsize": 8, "legend.framealpha": 0.92,
})
C = {"blue": "#0072B2", "orange": "#E69F00", "green": "#009E73", "red": "#D55E00",
     "purple": "#CC79A7", "sky": "#56B4E9", "gray": "#666666", "ink": "#222222"}


def save(fig, name):
    p = os.path.join(FIGS, name)
    fig.savefig(p, facecolor="white")
    plt.close(fig)
    print("wrote", p)


# ---------------------------------------------------------------------------
# Fig 1 -- the handover mechanism (diagram; no data)
# ---------------------------------------------------------------------------
def fig_arch():
    fig, ax = plt.subplots(figsize=(7.6, 2.5))
    ax.set_xlim(0, 100); ax.set_ylim(0, 44); ax.axis("off")
    BY, BH = 8, 24
    yc = BY + BH / 2

    def box(x, w, title, body, fc, ec):
        ax.add_patch(FancyBboxPatch((x, BY), w, BH, boxstyle="round,pad=0.3,rounding_size=1.6",
                                    lw=1.5, edgecolor=ec, facecolor=fc, alpha=0.95,
                                    mutation_aspect=0.5))
        ax.text(x + w / 2, BY + BH - 4.0, title, ha="center", va="center",
                fontsize=10, fontweight="bold", color=ec)
        ax.text(x + w / 2, BY + BH / 2 - 4.5, body, ha="center", va="center",
                fontsize=6.8, color=C["ink"], linespacing=1.35)

    box(2, 20, "Specialist A", "walk\n(LSTM, 120-d obs)\ntrained alone\nagainst its ghost",
        "#E8F1FA", C["blue"])
    box(38, 24, "THE HANDOVER", "morph blend (N ticks)\nphase-gated entry\nrecurrent-state law:\n"
        "stand -> locomotion = COLD", "#FFF4E6", C["orange"])
    box(78, 20, "Specialist B", "turn / stand / carry\ntrained alone\nagainst its ghost",
        "#E9F7F1", C["green"])
    ax.add_patch(FancyArrowPatch((22, yc), (37.5, yc), arrowstyle="-|>", mutation_scale=15,
                                 lw=1.8, color=C["gray"]))
    ax.add_patch(FancyArrowPatch((62.5, yc), (77.5, yc), arrowstyle="-|>", mutation_scale=15,
                                 lw=1.8, color=C["gray"]))
    ax.text(50, 41,
            "BATON: the specialists are independent; the ENGINEERING is in the switch",
            ha="center", va="center", fontsize=10, style="italic", color=C["ink"])
    ax.text(50, 3.5,
            "a warm recurrent state carried out of a STAND locks the incoming walker in the "
            "stand attractor: it marches in place",
            ha="center", va="center", fontsize=6.6, color=C["red"])
    save(fig, "fig_arch.png")


# ---------------------------------------------------------------------------
# Fig 2 -- THE HEADLINE: success rate vs horizon. Data or nothing.
# ---------------------------------------------------------------------------
def fig_horizon():
    if not os.path.exists(DATA):
        print("SKIP fig_horizon: no data at", DATA)
        print("     run:  bash scripts/dev/baton_horizon_experiment.sh 6 5 900")
        print("     (this script will NOT draw a curve it does not have.)")
        return False
    d = json.load(open(DATA))
    n = d["cycles"]
    ks = np.arange(n)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.9),
                                  gridspec_kw=dict(width_ratios=[1.35, 1.0]))

    style = {"engineered": (C["green"], "o", "BATON: engineered handover"),
             "naive": (C["red"], "s", "naive handover (warm state, no morph)")}
    for arm, dd in d["arms"].items():
        col, mk, lab = style.get(arm, (C["gray"], "^", arm))
        ax.plot(ks, dd["success_rate"], marker=mk, color=col, lw=2.0, ms=5, label=lab)
    ax.set_xlabel("horizon (cycle index)")
    ax.set_ylabel("success rate (fraction of seeds alive)")
    ax.set_ylim(-0.05, 1.08)
    ax.set_xticks(ks)
    ax.set_title(f"(a) Success vs horizon  (n={d['seeds']} seeds/arm)", fontsize=9.5)
    ax.legend(loc="lower left", fontsize=6.6)

    arms = list(d["arms"])
    surv = [np.mean(d["arms"][a]["survival"]) for a in arms]
    cols = [style.get(a, (C["gray"],))[0] for a in arms]
    ax2.bar(np.arange(len(arms)), surv, color=cols, edgecolor="black", lw=0.6, width=0.6)
    for i, v in enumerate(surv):
        ax2.text(i, v + 0.08, f"{v:.1f}", ha="center", fontsize=7.5, fontweight="bold")
    ax2.set_xticks(np.arange(len(arms)))
    ax2.set_xticklabels(["engineered", "naive"], fontsize=8)
    ax2.set_ylabel(f"cycles survived (of {n})")
    ax2.set_ylim(0, n + 0.6)
    ax2.set_title("(b) Mean survival", fontsize=9.5)
    save(fig, "fig_horizon.png")
    return True


if __name__ == "__main__":
    fig_arch()
    ok = fig_horizon()
    print("\nALL FIGURES DONE" if ok else
          "\nfig_horizon SKIPPED (no experiment data) -- the paper cannot claim the "
          "horizon result until it exists.")
    sys.exit(0)
