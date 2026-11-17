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

"""Generate all figures for the Shadowing paper from the real ghost .npz data.
Outputs 200-dpi PNGs into ./figs/. Pure-data plots use projects/policies/research/shadowing/ghosts/*.npz."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
GH = os.path.join(HERE, "..", "..", "..", "projects", "policies", "research",
                  "shadowing", "ghosts")
FIGS = os.path.join(HERE, "figs")
os.makedirs(FIGS, exist_ok=True)

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif"],
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9.5,
    "axes.grid": True,
    "grid.alpha": 0.30,
    "grid.linewidth": 0.5,
    "axes.axisbelow": True,
    "figure.dpi": 200,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
    "legend.fontsize": 8,
    "legend.framealpha": 0.92,
})

# colorblind-friendly palette
C = {
    "blue": "#0072B2", "orange": "#E69F00", "green": "#009E73",
    "red": "#D55E00", "purple": "#CC79A7", "sky": "#56B4E9",
    "yellow": "#F0E442", "gray": "#666666", "ink": "#222222",
}

def load(name):
    return np.load(os.path.join(GH, name), allow_pickle=True)

def save(fig, name):
    p = os.path.join(FIGS, name)
    fig.savefig(p, facecolor="white")
    plt.close(fig)
    print("wrote", p)


# ----------------------------------------------------------------------------
# Fig 1 — Architecture / pipeline diagram
# ----------------------------------------------------------------------------
def fig_architecture():
    fig, ax = plt.subplots(figsize=(7.6, 2.45))
    ax.set_xlim(0, 100); ax.set_ylim(0, 42); ax.axis("off")

    BY, BH = 5.5, 25.5          # box bottom, height
    yc = BY + BH / 2            # vertical centre of boxes/arrows

    def box(x, w, num, title, body, fc, ec):
        b = FancyBboxPatch((x, BY), w, BH, boxstyle="round,pad=0.3,rounding_size=1.6",
                           linewidth=1.5, edgecolor=ec, facecolor=fc,
                           alpha=0.95, mutation_aspect=0.5)
        ax.add_patch(b)
        cx = x + w / 2
        ax.text(cx, BY + BH - 3.6, f"{num}  {title}", ha="center", va="center",
                fontsize=10.5, fontweight="bold", color=ec)
        ax.text(cx, BY + BH/2 - 4.0, body, ha="center", va="center",
                fontsize=7.0, color=C["ink"], linespacing=1.4)

    def arrow(x0, x1, label=None):
        ax.add_patch(FancyArrowPatch((x0, yc), (x1, yc), arrowstyle="-|>",
                     mutation_scale=15, linewidth=1.8, color=C["gray"]))
        if label:
            ax.text((x0 + x1) / 2, yc + 3.4, label, ha="center", va="bottom",
                    fontsize=6.6, style="italic", color=C["gray"], linespacing=1.2)

    # input -> 1 -> 2 -> 3 -> output, with wide gaps for arrow labels
    ax.text(6.5, yc, "Intent\n(robot model,\ngoal /\nkeyframes)", ha="center", va="center",
            fontsize=7.3, fontweight="bold", color=C["blue"], linespacing=1.35)
    arrow(12.5, 16)

    box(16, 19, "1", "Ghost Generator",
        "receding-horizon\nMPPI over the full\ncontact dynamics\n(position-target\nactuators)\n→ feasible by\nconstruction", "#E8F1FA", C["blue"])
    arrow(35.5, 43.5, "ghost\nq, base")

    box(44, 19, "2", "Ghost Verifier",
        "per-step contact-\nwrench / ZMP\nfeasibility program\n→ pass/fail score\nBEFORE any\nlearning", "#FFF4E6", C["orange"])
    arrow(63.5, 71.5, "certified\nghost")

    box(72, 19, "3", "Tracker",
        "RL policy shadows\nthe ghost under\ndomain\nrandomization\n→ crosses\nsim-to-deploy", "#E9F7F1", C["green"])

    # output
    ax.add_patch(FancyArrowPatch((81.5, BY), (81.5, BY - 2.6), arrowstyle="-|>",
                 mutation_scale=15, linewidth=1.8, color=C["green"]))
    ax.text(81.5, BY - 4.2, "deployable policy", ha="center", va="center",
            fontsize=8.0, fontweight="bold", color=C["green"])

    ax.text(50, 40.0,
            "Shadowing:  planning describes the problem  —  control learns to solve it",
            ha="center", va="center", fontsize=10, style="italic", color=C["ink"])
    save(fig, "fig_architecture.png")


# ----------------------------------------------------------------------------
# Fig 2 — G1 sit-to-stand: the feasible ghost (pelvis height vs time)
# ----------------------------------------------------------------------------
def fig_sitstand():
    d = load("g1_sitstand_ghost.npz")
    dt = float(d["dt"]); q = d["q"]; com = d["com"]
    N = q.shape[0]; t = np.arange(N) * dt
    pelvis = q[:, 2]; comz = com[:, 2]

    fig, ax = plt.subplots(figsize=(3.45, 2.7))
    ax.plot(t, pelvis, color=C["blue"], lw=2.0, label="pelvis height")
    ax.plot(t, comz, color=C["orange"], lw=1.5, ls="--", label="CoM height")
    # phase shading
    spans = [(0.0, 1.2, "seated /\nlean", "#f2f2f2"),
             (1.2, 3.0, "rise", "#e8f1fa"),
             (3.0, 6.0, "stand + hold", "#e9f7f1")]
    for a, b, lab, col in spans:
        ax.axvspan(a, b, color=col, alpha=0.7, zorder=0)
        ax.text((a+b)/2, 0.60, lab, ha="center", va="center", fontsize=6.6,
                color=C["gray"], linespacing=1.1)
    ax.annotate(f"peak {pelvis.max():.3f} m\n(tilt 2–5°)", xy=(t[pelvis.argmax()], pelvis.max()),
                xytext=(3.5, 0.665), fontsize=6.8, color=C["blue"],
                arrowprops=dict(arrowstyle="->", color=C["blue"], lw=1.0))
    ax.set_xlabel("time (s)"); ax.set_ylabel("height (m)")
    ax.set_title("G1 sit-to-stand ghost (generated, feasible)")
    ax.set_ylim(0.55, 0.80); ax.set_xlim(0, 6)
    # "lower right" sat on top of the "stand + hold" phase label; upper-left is clear.
    ax.legend(loc="upper left", fontsize=7)
    save(fig, "fig_sitstand.png")


# ----------------------------------------------------------------------------
# Fig 3 — The negative result: deepest-stable-crouch wall
# ----------------------------------------------------------------------------
def fig_negative():
    fig, ax = plt.subplots(figsize=(3.7, 2.85))
    # The ghost's peak pelvis height is DERIVED from the ghost itself, not typed
    # in: the old code hard-coded 0.741 while the committed ghost peaks at 0.734.
    ghost_peak = float(load("g1_sitstand_ghost.npz")["q"][:, 2].max())
    labels = ["seated\nstart", "stay-seated\noptimum", "PPO-resid.\n×21 runs",
              "DAgger\ndistill", "MPC\nin-loop", "feasible\nghost"]
    vals = [0.595, 0.595, 0.60, 0.655, 0.74, ghost_peak]
    cols = [C["gray"], C["gray"], C["red"], C["red"], C["green"], C["blue"]]
    x = np.arange(len(vals))
    ax.bar(x, vals, color=cols, edgecolor="black", linewidth=0.6, width=0.72)
    # wall line
    ax.axhline(0.60, color=C["red"], ls=":", lw=1.2)
    ax.text(0.04, 0.583, "deepest stable crouch ≈ 0.60 m  (the wall)",
            transform=ax.get_yaxis_transform(), fontsize=6.4, color=C["red"])
    ax.axhline(ghost_peak, color=C["blue"], ls=":", lw=1.0, alpha=0.7)
    ax.text(0.62, ghost_peak + 0.004, f"full stand {ghost_peak:.2f} m",
            transform=ax.get_yaxis_transform(), fontsize=6.2, color=C["blue"])
    for xi, v in zip(x, vals):
        ax.text(xi, v + 0.004, f"{v:.2f}", ha="center", va="bottom", fontsize=6.6)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=6.0)
    ax.set_ylabel("peak pelvis height (m)")
    ax.set_ylim(0.55, 0.79)
    ax.set_title("Dead-seated launch: reactive tracking stalls")
    # DAgger topple marker
    ax.annotate("then topples\n(tilt 60–117°)", xy=(3, 0.658), xytext=(3.0, 0.715),
                fontsize=5.8, color=C["red"], ha="center",
                arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.8))
    save(fig, "fig_negative.png")


# ----------------------------------------------------------------------------
# Fig 4 — arm toss-to-place: the feasibility frontier  [FIGURE REMOVED]
#
# The former fig_toss() plotted the fixed-base manipulator's toss ghost (swing +
# ballistic release) against the 1.38 m feasibility frontier. Its source ghosts are
# not part of this tree, so the figure cannot be regenerated here and the paper no
# longer includes it. The toss RESULT itself (1.5 cm landing error at 1.3 m; the
# 1.8 m target correctly rejected before deployment) is unchanged and still reported
# in the paper text and in the generality table.
# ----------------------------------------------------------------------------


# ----------------------------------------------------------------------------
# Fig 5 — Hill walk: CoM height vs distance across grades (B2) + peak vs grade
# ----------------------------------------------------------------------------
def fig_hill():
    grades = [("b2_hill4_ghost.npz", 4, C["sky"]),
              ("b2_hill6_ghost.npz", 6, C["blue"]),
              ("b2_hill8_ghost.npz", 8, C["green"]),
              ("b2_hill12_ghost.npz", 12, C["orange"]),
              ("b2_hill_ghost.npz", 15, C["red"])]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.7),
                                  gridspec_kw=dict(width_ratios=[2.1, 1]))
    peaks = []
    for fn, g, col in grades:
        d = load(fn); com = d["com"]
        ax.plot(com[:, 0], com[:, 2], color=col, lw=1.6, label=f"{g}°")
        peaks.append((g, com[:, 2].max(), col))
    ax.set_xlabel("forward distance (m)"); ax.set_ylabel("CoM height (m)")
    ax.set_title("B2 hill-walk ghosts (up–over–down)")
    ax.legend(title="grade", loc="upper left", ncol=2, fontsize=7)
    ax.set_xlim(-0.2, 9.2)

    gg = [p[0] for p in peaks]; pp = [p[1] for p in peaks]
    ax2.plot(gg, pp, "-o", color=C["ink"], lw=1.4, ms=5)
    for g, p, col in peaks:
        ax2.plot([g], [p], "o", color=col, ms=7, mec="black", mew=0.5)
    ax2.set_xlabel("hill grade (deg)"); ax2.set_ylabel("crest CoM height (m)")
    ax2.set_title("crest height vs grade")
    save(fig, "fig_hill.png")


# ----------------------------------------------------------------------------
# Fig 6 — OmniQuad leap  [REMOVED 2026-07-11 — DO NOT RESURRECT WITHOUT SOURCES]
#
# The former fig_jump() plotted the omniquad_jump ghost against a hard-coded
# "deploy ceiling 0.69 m" and a "stance rest 0.56 m". BOTH were unsourceable:
#   1. Neither 0.694 nor 0.563 appears in rl-current-state.md (the canonical RL
#      status) or in any committed deploy log. The paper's accompanying claim
#      ("deploys cleanly but under-jumps, saturating near 0.69 m against a
#      1.05 m ghost") therefore had no traceable artifact behind it.
#   2. Worse, the ghost itself does NOT certify: docs/developer/shadowing.md's
#      certificate table records `omniquad jump -> FAIL (boundary)` — the explosive
#      takeoff is not contact-consistent. Charting it as an exemplar of a
#      pipeline whose selling point is a pre-training feasibility certificate
#      inverted the paper's own thesis.
# The leap row, the leap prose (§5.3), and the §6 "leap height" mention were all
# removed from the paper for the same reason. If the leap is ever revived, it
# needs a certified ghost AND a logged deploy run; do not re-add fabricated
# ceiling lines.
# ----------------------------------------------------------------------------


# ----------------------------------------------------------------------------
# Fig 7 — Get-up rise curves (OmniQuad + B2), MPPI-discovered contact-rich motions
# ----------------------------------------------------------------------------
def fig_getup():
    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    peak = 0.0
    for fn, col, lab in [("omniquad_getup_ghost.npz", C["blue"], "OmniQuad get-up"),
                         ("b2_getup_ghost.npz", C["orange"], "B2 get-up")]:
        d = load(fn); dt = float(d["dt"]); base = d["base"]
        N = base.shape[0]; t = np.arange(N) * dt
        ax.plot(t, base[:, 2], color=col, lw=2.0, label=lab)
        peak = max(peak, float(base[:, 2].max()))
    ax.set_xlabel("time (s)"); ax.set_ylabel("base height (m)")
    ax.set_title("Get-up ghosts: lying → stand (MPPI-discovered)")
    ax.legend(loc="lower right")
    # NOTE: the old hard-coded ylim of 0.62 CLIPPED both curves (OmniQuad peaks at
    # 0.635 m, B2 at 0.646 m) -- the rise looked truncated. Derive from the data.
    ax.set_xlim(0, None); ax.set_ylim(0, peak * 1.12)
    ax.text(0.4, 0.04, "starts collapsed (belly-flat)", fontsize=6.4, color=C["gray"])
    save(fig, "fig_getup.png")


# ----------------------------------------------------------------------------
# Fig 8 — Throughput
#
# HONESTY CONTRACT (2026-07-11): every bar traces to a FIRST-HAND, committed
# measurement. The previous version of this figure charted A100 / H100 / H200
# bars (a "bandwidth collapse at 8k envs", tracker rates of 500-616k). NONE of
# those numbers appears in any committed artifact -- not in rl-current-state.md,
# not in docs/benchmarks/, not in a trainer log -- and OmniSim has no cloud
# training path at all (the Modal/H100 wrappers were REMOVED, ef46a52e). They
# were deleted rather than reconstructed. Do not re-add a datacentre-GPU bar
# without a committed log.
#
# Sources for what remains:
#   * 39k -> 98k env-steps/s, 2.5x end-to-end, full-forward reset was ~60% of
#     train time -- projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py:26-33
#     (RTX 5070 Ti Laptop, full-body kp100)
#   * quadruped 661,636 @ 4096 envs and 162,604 @ 1024 envs, RTX 3060 Laptop,
#     "first-hand" -- docs/benchmarks/performance-comparison.md:333-336 (table 5.1)
#   * OmniQuad model+residual ~80k env-steps/s, RTX 5070 -- performance-comparison.md:191
#   * G1 humanoid stand ~27-62k env-steps/s, RTX 3060 -- performance-comparison.md:195
# ----------------------------------------------------------------------------
def fig_throughput():
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.8, 2.7),
                                  gridspec_kw=dict(width_ratios=[0.72, 1.28]))
    # the right panel's y-tick labels are long (robot + GPU); without extra gap the
    # left panel's bar labels collide with them.
    fig.subplots_adjust(wspace=0.62)

    # left: the reset optimization (the one throughput claim the paper makes)
    names = ["full-physics\nforward reset", "kinematics-only\nreset"]
    vals = [39, 98]
    ax.bar([0, 1], vals, color=[C["gray"], C["green"]], edgecolor="black",
           lw=0.6, width=0.6)
    for x, v in zip([0, 1], vals):
        ax.text(x, v + 1.8, f"{v}k", ha="center", va="bottom", fontsize=7.5,
                fontweight="bold")
    ax.annotate("", xy=(1, 92), xytext=(0, 44),
                arrowprops=dict(arrowstyle="->", color=C["red"], lw=1.3))
    ax.text(0.5, 74, "2.5×", ha="center", fontsize=9, fontweight="bold",
            color=C["red"])
    ax.set_xticks([0, 1]); ax.set_xticklabels(names, fontsize=6.6)
    ax.set_ylabel("env-steps/s (×10³)")
    ax.set_ylim(0, 118)
    ax.set_title("G1 walk trainer: reset cost", fontsize=9)
    ax.text(0.5, 0.03, "RTX 5070 Ti Laptop", transform=ax.transAxes,
            ha="center", fontsize=5.8, color=C["gray"], style="italic")

    # right: measured sustained throughput -- each bar a DIFFERENT robot AND GPU
    rows = [
        ("quadruped walk\n4096 envs · RTX 3060", 662, C["blue"]),
        ("quadruped walk\n1024 envs · RTX 3060", 163, C["sky"]),
        ("G1 walk (23 DOF)\n4096 envs · 5070 Ti", 98, C["green"]),
        ("OmniQuad model+residual\nRTX 5070", 80, C["orange"]),
        ("G1 stand\nRTX 3060", 45, C["purple"]),   # midpoint of the 27-62k range
    ]
    yi = np.arange(len(rows))
    ax2.barh(yi, [r[1] for r in rows], color=[r[2] for r in rows],
             edgecolor="black", lw=0.5)
    labs = ["662k", "163k", "98k", "~80k", "27–62k"]
    for y, r, lab in zip(yi, rows, labs):
        ax2.text(r[1] + 12, y, lab, va="center", fontsize=6.6)
    # show the G1-stand range as an error bar rather than a false point value
    ax2.plot([27, 62], [4, 4], color="black", lw=1.0, zorder=5)
    ax2.plot([27, 62], [4, 4], "|", color="black", ms=4, zorder=5)
    ax2.set_yticks(yi); ax2.set_yticklabels([r[0] for r in rows], fontsize=6.0)
    ax2.invert_yaxis()
    ax2.set_xlabel("env-steps/s (×10³)")
    ax2.set_title("Measured trainer throughput (first-hand)", fontsize=9)
    ax2.set_xlim(0, 790)
    ax2.text(0.985, 0.05,
             "different robot AND different GPU per bar\n— not a scaling curve",
             transform=ax2.transAxes, ha="right", va="bottom", fontsize=5.6,
             color=C["gray"], style="italic")
    save(fig, "fig_throughput.png")


# ----------------------------------------------------------------------------
# Fig 9 — Deploy distances + the certificate/deploy partition
#
# HONESTY CONTRACT (2026-07-11): every bar here MUST trace to a verified deploy
# run in docs/developer/rl-current-state.md ("Table A — Durable walks: VERIFIED,
# 0 falls"). Do NOT add a bar you cannot source there.
#   * The G1 has NO durable free-standing deploy walk -> it gets NO DISTANCE BAR.
#     A finite bout is not a distance result and must not be charted as one, and
#     the durable in-engine G1 gait we do have is BALANCE-HARNESSED (a pelvis
#     crane carrying up to ~2x body weight), which is a tracking result, not a
#     locomotion-distance result. It appears in panel (b) instead, truthfully.
#   * Distances that appeared in earlier drafts and are NOT in Table A were
#     trainer-side or stale numbers that do not reproduce in deploy. They are
#     gone. Bars are added from Table A only.
#   * One robot charted in an early draft has since been removed from the repo
#     entirely; do not restore it or any figure that references it.
# ----------------------------------------------------------------------------
def fig_generality():
    """Two panels.

    (a) The durable deploy results -- quadruped distances (sourced, Table A).
    (b) THE HONEST FRAMING OF THE BIPED. The old figure was quadruped-only with an
        apologetic footnote saying "the G1 has no bar". That under-represented the
        biped work AND buried the paper's actual thesis. Panel (b) instead plots the
        quantity the G1 *does* have: the certificate verdict vs the deploy outcome.
        Every ghost certifies; the deploy column is what splits. That IS the paper's
        central diagnostic claim (cert PASS + deploy FAIL localizes the failure to
        the tracker, exonerating the reference), and it lets the G1 appear truthfully
        -- as a certified ghost that does not deploy free-standing -- rather than
        being represented by a distance it never achieved, or omitted entirely.

    Every cell traces to a committed artifact:
      * distances            -> rl-current-state.md "Table A - Durable walks: VERIFIED"
      * certificate verdicts -> docs/developer/shadowing.md "Certificate - verified
                                across the motion repertoire" table
      * deploy outcomes      -> rl-current-state.md Table A / Table B + skill manifests
    NEVER invent a bar. The G1 gets no distance bar because it has no durable
    free-standing deploy walk.
    """
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.95),
                                  gridspec_kw=dict(width_ratios=[1.0, 1.62]))

    # ---- (a) THE HEAD-TO-HEAD: certified ghost vs the analytic-trot baseline ----
    # HONESTY CONTRACT (2026-07-12). This panel used to plot the OmniQuad/Go2/B2 deploy
    # distances under the title "Durable deploy walks", inside a figure about the
    # generality of Shadowing -- which invited the reading that a certified ghost
    # produced them. It did not: all three track an ANALYTIC foot-space trot (the
    # degenerate case), and the repo's own catalogue had them mislabelled as
    # "shadowing" until it was corrected. Plotting them here was the visual form of
    # that same mislabel.
    #
    # What belongs here is the experiment that actually isolates the method: the same
    # robot, the same world, the same physics, differing only in the reference --
    # analytic trot (the incumbent) vs a certified, recorded ghost. Source: the
    # interleaved 3x240 s live deploy exam, both policies asserted loaded
    # (0.429 vs 0.381 m/s; |y| drift 0.05 vs 0.26 m; zero falls both sides).
    labels = ["mean speed\n(m/s)", "peak lateral\ndrift (m)"]
    base = [0.381, 0.26]      # residual RL on the analytic trot  (gpu_go2_walk_main)
    ghost = [0.429, 0.05]     # Shadowing a certified ghost       (gpu_go2_shadow_main)
    yi = np.arange(len(labels))
    h = 0.36
    ax.barh(yi + h / 2, base, height=h, color=C["gray"],
            edgecolor="black", lw=0.5, label="analytic trot (incumbent)")
    ax.barh(yi - h / 2, ghost, height=h, color=C["red"],
            edgecolor="black", lw=0.5, label="certified ghost (Shadowing)")
    for y, (b, g) in enumerate(zip(base, ghost)):
        ax.text(b + 0.012, y + h / 2, f"{b:.3f}", va="center", fontsize=6.0)
        ax.text(g + 0.012, y - h / 2, f"{g:.3f}", va="center", fontsize=6.0, fontweight="bold")
    ax.set_yticks(yi); ax.set_yticklabels(labels, fontsize=7)
    ax.set_xlim(0, 0.56)
    ax.set_xlabel("Go2, live deploy (3×240 s, interleaved)", fontsize=7.5)
    ax.set_title("(a) Certified ghost vs analytic trot", fontsize=9.5)
    ax.legend(fontsize=5.8, loc="lower right", framealpha=0.9)
    ax.invert_yaxis()

    # ---- (b) certificate (pre-RL) vs deploy outcome: the partition ----
    # (motion, cert label, deployed?, note)
    cert_rows = [
        ("Go2 walk",          "PASS  1.7% mg",  True,  "86.7 m"),
        ("B2 walk",           "PASS  3.1% mg",  True,  "110.7 m"),
        ("Arm toss",          "PASS",           True,  "1.5 cm"),
        ("B2 get-up",         "PASS",           True,  "rises"),
        ("G1 walk",           "PASS  4.6% mg",  False, "topples free-standing"),
        ("G1 sit-stand",      "PASS",           False, "launch not learned"),
        ("Hill walk (B2 / OmniQuad)", "PASS  5.8% mg", False, "tracker stalls"),
        ("G1 stair climb 7 cm", "PASS",         False, "no policy climbs it"),
    ]
    ax2.set_xlim(0, 4.35); ax2.set_ylim(-0.7, len(cert_rows) - 0.3)
    ax2.invert_yaxis()
    ax2.axis("off")
    ax2.text(1.80, -0.62, "ghost certificate\n(before any RL)", ha="center", va="center",
             fontsize=6.8, fontweight="bold", color=C["orange"], linespacing=1.2)
    ax2.text(3.30, -0.62, "deploys durably", ha="center", va="center",
             fontsize=6.8, fontweight="bold", color=C["green"])
    # separator between the two outcome columns
    ax2.plot([2.72, 2.72], [-0.28, len(cert_rows) - 0.45], color=C["gray"],
             lw=0.7, ls="--", alpha=0.6)
    # NB: use plotted markers, NOT unicode check/ballot glyphs -- DejaVu Serif has
    # neither (U+2713 / U+2717) and they render as tofu boxes in the PDF.
    for i, (name, cert, dep, note) in enumerate(cert_rows):
        ax2.text(-0.02, i, name, ha="left", va="center", fontsize=6.5, color=C["ink"])
        ax2.plot([1.40], [i], marker="o", ms=5.0, color=C["green"],
                 mec="black", mew=0.5, zorder=4)
        ax2.text(1.52, i, cert, ha="left", va="center", fontsize=6.3, color=C["green"])
        col = C["green"] if dep else C["red"]
        ax2.plot([2.88], [i], marker="o" if dep else "X", ms=5.4 if dep else 5.8,
                 color=col, mec="black", mew=0.5, zorder=4)
        ax2.text(3.02, i, note, ha="left", va="center", fontsize=6.3, color=col)
    ax2.set_title("(b) Every ghost certifies — the deploy outcome splits", fontsize=9.5)
    ax2.text(2.10, len(cert_rows) - 0.55,
             "cert PASS + deploy FAIL ⇒ the reference is exonerated;\n"
             "the failure is the tracker / deployment pipeline",
             ha="center", va="top", fontsize=5.9, color=C["gray"], style="italic",
             linespacing=1.3,
             bbox=dict(boxstyle="round,pad=0.3", fc="#f7f7f7", ec=C["gray"], lw=0.5))
    save(fig, "fig_generality.png")


# ----------------------------------------------------------------------------
# Fig 10 — Graded learnability: does the certificate score predict deploy outcome?
#
# Data: docs/developer/shadowing_paper/data/e2_rl_learnability.json (committed).
# n=8 Go2 trot references generated at increasing commanded vx; each tracked by an
# identically-trained residual RL policy. Stats independently recomputed here:
#   rho(cert_score, first_fall) = +0.939 (p=0.0006)
#   rho(vx,         first_fall) = -0.881 (p=0.0039)   <- the honest caveat
# HONESTY: half the scores are tied at a saturated 0, and vx ALONE nearly predicts
# the outcome. This is a FEASIBILITY CLIFF SEPARATION, not a graded regressor, and
# the figure must say so. The binary PASS gate is ALSO shown failing (it passes
# vx=1.2/1.6/2.0, all of which fall in ~1.1-1.5 s) -- only the SCORE separates.
# ----------------------------------------------------------------------------
def fig_learnability():
    import json
    p = os.path.join(HERE, "data", "e2_rl_learnability.json")
    d = json.load(open(p))
    vx = np.array([r["vx"] for r in d]); sc = np.array([r["score"] for r in d])
    ff = np.array([r["first_fall"] for r in d]); base = np.array([r["base"] for r in d])
    tau = np.array([r["tau"] for r in d]); cp = [r["cert_pass"] for r in d]

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.75))
    fig.subplots_adjust(wspace=0.46)

    # left: cert score vs first-fall
    good = sc > 0
    ax.scatter(sc[good], ff[good], s=54, c=C["green"], edgecolor="black",
               lw=0.6, zorder=3, label="score > 0")
    ax.scatter(sc[~good], ff[~good], s=54, c=C["red"], edgecolor="black",
               lw=0.6, zorder=3, marker="s", label="score = 0 (saturated)")
    # label only the separated points individually; the four saturated ones are
    # stacked on x=0 and their labels would collide -- annotate them as a cluster.
    for x, y, v in zip(sc[good], ff[good], vx[good]):
        ax.annotate(f"{v}", (x, y), textcoords="offset points", xytext=(5, 4),
                    fontsize=6.0, color=C["gray"])
    ax.annotate("vx = 1.2 – 2.5\n(4 refs, tied at 0)", xy=(0.012, 1.3),
                xytext=(0.10, 3.6), fontsize=6.0, color=C["red"], linespacing=1.25,
                arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.8))
    ax.axhspan(0, 2.5, color="#fdecea", zorder=0)
    ax.text(0.40, 1.55, "collapse:  first fall < 1.6 s", fontsize=6.0, color=C["red"])
    ax.set_xlabel("ghost certificate score (pre-training)")
    ax.set_ylabel("first fall in deploy (s)")
    ax.set_title("ρ = +0.94  (n=8, p=0.0006)", fontsize=9)
    ax.legend(fontsize=6.0, loc="upper left")
    ax.set_xlim(-0.06, 0.78); ax.set_ylim(0, 13.4)
    ax.text(0.985, 0.03, "labels = commanded vx (m/s)", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=5.7, color=C["gray"], style="italic")

    # right: the mechanism + the honest caveat -- where the gate fails and the score works
    ax2.plot(vx, base, "-o", color=C["blue"], lw=1.5, ms=4.5, label="base-wrench residual")
    ax2.axhline(0.08, color=C["blue"], ls=":", lw=1.1)
    ax2.text(0.32, 0.088, "binary PASS threshold (0.08)", fontsize=5.9, color=C["blue"])
    ax2.set_xlabel("commanded vx (m/s)"); ax2.set_ylabel("base residual (× mg)",
                                                         color=C["blue"])
    ax2.tick_params(axis="y", labelcolor=C["blue"])
    ax2.set_ylim(0, 0.30)
    ax3 = ax2.twinx()
    ax3.plot(vx, sc, "-s", color=C["orange"], lw=1.5, ms=4.5, label="certificate score")
    ax3.set_ylabel("certificate score", color=C["orange"])
    ax3.tick_params(axis="y", labelcolor=C["orange"])
    ax3.set_ylim(-0.03, 0.78); ax3.grid(False)
    # mark the cliff
    ax2.axvspan(0.95, 1.25, color="#eeeeee", zorder=0)
    ax2.annotate("score collapses to 0 here\n(τ margin saturates)\nfirst-fall: 6.7 s → 1.1 s",
                 xy=(1.2, 0.042), xytext=(1.30, 0.20), fontsize=5.9, color=C["ink"],
                 arrowprops=dict(arrowstyle="->", color=C["ink"], lw=0.9))
    # the binary gate only rejects the LAST point
    ax2.annotate("binary gate rejects\nonly vx = 2.5", xy=(2.5, 0.268),
                 xytext=(1.62, 0.268), fontsize=5.9, color=C["red"],
                 arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.9))
    ax2.set_title("The gate is too loose; the score separates", fontsize=9)
    save(fig, "fig_learnability.png")


if __name__ == "__main__":
    fig_architecture()
    fig_sitstand()
    fig_negative()
    fig_hill()
    fig_getup()
    fig_throughput()
    fig_generality()
    fig_learnability()
    print("ALL FIGURES DONE")
