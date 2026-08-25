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

"""Render a side-view PREVIEW of an up-over-down HILL-WALK ghost for owner
sign-off (ghost-first). Shows the rounded hill cross-section, the base path over
it, and the robot's stick-figure pose (pitched trunk + legs to the feet on the
surface) at keyframes across the traverse -- a cheap, reliable PNG before the
full OmniSim translucent-ghost world + RL tracker.

  python projects/policies/research/shadowing/preview_hill_ghost.py --robot b2
  python projects/policies/research/shadowing/preview_hill_ghost.py --robot omniquad
"""
import argparse
import importlib
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.hill_terrain import HillProfile  # noqa: E402

GAIT = {"b2": "projects.policies.control.gait.b2_trot_gait",
        "omniquad": "projects.policies.control.gait.omniquad_trot_gait",
        "go2": "projects.policies.control.gait.go2_trot_gait"}


def _rot(phi, bx, bz):
    """body-frame (bx,bz) -> world offset, body pitched up by phi (nose-up +)."""
    c, s = math.cos(phi), math.sin(phi)
    return bx * c - bz * s, bx * s + bz * c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=list(GAIT), default="b2")
    ap.add_argument("--ghost", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    ghost = a.ghost or os.path.join(REPO, "projects/policies/research/shadowing/ghosts", f"{a.robot}_hill_ghost.npz")
    out = a.out or os.path.join(REPO, "_scratch", f"{a.robot}_hill_preview.png")

    g = np.load(ghost, allow_pickle=True)
    base, feet, dt = g["base"], g["feet"], float(g["dt"])
    hp = HillProfile(grade_deg=float(g["hill_grade_deg"]), approach=float(g["hill_approach"]),
                     ramp_run=float(g["hill_ramp_run"]), crest=float(g["hill_crest"]),
                     fillet=float(g["hill_fillet"]))
    stg = importlib.import_module(GAIT[a.robot])
    gp = stg.GaitParams(vx=float(g["vx"]))
    n = base.shape[0]
    pitch = np.array([-2.0 * math.atan2(base[k, 5], base[k, 3]) for k in range(n)])  # nose-up +

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 7.6),
                                  gridspec_kw={"height_ratios": [2.4, 1]})

    # --- hill cross-section ---
    xs = np.linspace(-0.5, hp.x_end + 1.3, 400)
    hs = np.array([hp.height(x) for x in xs])
    ax.fill_between(xs, -0.3, hs, color="#cdbfae", zorder=0)
    ax.plot(xs, hs, color="#7a6a55", lw=2, zorder=1)
    ax.plot(base[:, 0], base[:, 2], color="#1565c0", lw=1.4, alpha=0.8,
            label="ghost base path", zorder=3)

    # --- keyframe stick figures (color = time) ---
    cmap = plt.get_cmap("viridis")
    kfs = np.linspace(0, n - 1, 9).astype(int)
    for j, k in enumerate(kfs):
        col = cmap(j / (len(kfs) - 1))
        bx, bz, phi = base[k, 0], base[k, 2], pitch[k]
        fh = (bx + _rot(phi, gp.x_front, 0)[0], bz + _rot(phi, gp.x_front, 0)[1])
        rh = (bx + _rot(phi, gp.x_rear, 0)[0], bz + _rot(phi, gp.x_rear, 0)[1])
        ax.plot([fh[0], rh[0]], [fh[1], rh[1]], "-", color=col, lw=4, zorder=5,
                solid_capstyle="round")
        for fi in (0, 1):                                   # front feet -> front hip
            ax.plot([fh[0], feet[k, fi, 0]], [fh[1], feet[k, fi, 2]], "-", color=col, lw=1.4, zorder=4)
        for fi in (2, 3):                                   # rear feet -> rear hip
            ax.plot([rh[0], feet[k, fi, 0]], [rh[1], feet[k, fi, 2]], "-", color=col, lw=1.4, zorder=4)
        ax.plot(feet[k, :, 0], feet[k, :, 2], "o", color=col, ms=3.5, zorder=6)
        ax.text(bx, bz + 0.12, f"{k*dt:.0f}s", color=col, fontsize=8, ha="center", weight="bold")

    ax.set_aspect("equal")
    ax.set_xlim(-0.5, hp.x_end + 1.3)
    ax.set_ylim(-0.25, hp.rise + 1.1)
    ax.set_xlabel("forward x (m)")
    ax.set_ylabel("height z (m)")
    ax.set_title(f"{a.robot.upper()} hill-walk GHOST  —  up {hp.grade_deg:.0f}° / over a "
                 f"{hp.rise:.2f} m crest / down {hp.grade_deg:.0f}°, "
                 f"body pitches with the slope (feet on the surface)", fontsize=12)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.15)

    # --- base height + pitch vs time ---
    t = np.arange(n) * dt
    ax2.plot(t, base[:, 2], color="#1565c0", lw=1.8, label="base height z (m)")
    ax2.plot(t, [hp.height(x) for x in base[:, 0]], color="#7a6a55", lw=1.4, ls="--",
             label="ground h(x) under base")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("height (m)", color="#1565c0")
    ax2.grid(alpha=0.15)
    ax2b = ax2.twinx()
    ax2b.plot(t, np.degrees(pitch), color="#e65100", lw=1.6, label="base pitch (deg)")
    ax2b.axhline(hp.grade_deg, color="#e65100", ls=":", lw=0.9, alpha=0.6)
    ax2b.axhline(-hp.grade_deg, color="#e65100", ls=":", lw=0.9, alpha=0.6)
    ax2b.set_ylabel("pitch (deg)", color="#e65100")
    ln1, lb1 = ax2.get_legend_handles_labels()
    ln2, lb2 = ax2b.get_legend_handles_labels()
    ax2.legend(ln1 + ln2, lb1 + lb2, loc="upper right", fontsize=8, ncol=3)

    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"[preview] wrote {os.path.relpath(out, REPO)}")


if __name__ == "__main__":
    main()
