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

"""Render a side-view PREVIEW of the OMNIARM6 toss-to-place ghost for owner sign-off (ghost-first).

Shows the arm at the release instant, the payload's swing path, the ballistic arc into the bin,
the kinematic-reach zone (what a carry/IK controller could reach), and the feasibility frontier
(every distance the swing could hit by releasing at a different instant). Cheap PNG preview
before the full OmniSim translucent-ghost world + RL.

  python projects/policies/research/shadowing/preview_omniarm6_toss.py [--ghost PATH] [--out PATH]
"""
import argparse
import os
import sys

import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
import projects.policies.research.shadowing.generate_omniarm6_toss as TG  # noqa: E402

ARM_BODIES = ["link0", "link1", "link2", "link3", "link4", "link5", "link6", "payload"]


def arm_points(m, d, q):
    """Cartesian (x,z) of each arm body origin at config q -> stick figure."""
    d.qpos[:6] = q[:6]
    mujoco.mj_forward(m, d)
    pts = []
    for b in ARM_BODIES:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        p = d.xpos[bid]
        pts.append((p[0], p[2]))
    return np.array(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ghost", default=os.path.join(REPO, "projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz"))
    ap.add_argument("--out", default=os.path.join(REPO, "_scratch/omniarm6_toss_preview.png"))
    args = ap.parse_args()

    g = np.load(args.ghost, allow_pickle=True)
    q, tcp, tcpv = g["q"], g["tcp"], g["tcpvel"]
    k = int(g["release_k"]); bin_x = float(g["bin_x"]); land = g["land"]
    max_range = float(g["max_range"]); velr = float(g["vel_ratio"])

    m = mujoco.MjModel.from_xml_path(TG.MJCF)
    d = mujoco.MjData(m)
    pts = arm_points(m, d, q[k])

    # ballistic arc from release
    p0, v0 = tcp[k], tcpv[k]
    tf = TG.ballistic_landing(p0, v0)[2]
    ts = np.linspace(0, tf, 60)
    bx = p0[0] + v0[0] * ts
    bz = p0[2] + v0[2] * ts - 0.5 * TG.G * ts ** 2

    # release map = the reachable landing distances (feasibility frontier)
    rmap = TG.release_map({"tcp": tcp, "tcpvel": tcpv})
    lands = sorted(r[1] for r in rmap)

    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.axhline(0, color="#444", lw=2)                                   # ground
    # carry-reach zone (what IK can place into a floor bin ~0.9 m)
    ax.add_patch(Wedge((0, 0), 0.9, 0, 90, width=0.9, color="#4caf50", alpha=0.08))
    ax.add_patch(Wedge((0, 0), TG.KIN_REACH_X, 0, 90, width=0.02, color="#4caf50", alpha=0.5))
    ax.text(0.45, 0.05, "carry / IK reach\n(≤0.9 m)", color="#2e7d32", fontsize=9, ha="center")

    # feasibility frontier band (throw-only zone)
    ax.axvspan(TG.KIN_REACH_X, max_range, color="#2196f3", alpha=0.08)
    ax.axvline(max_range, color="#1565c0", ls="--", lw=1.2)
    ax.text(max_range, 1.5, f"  feasibility frontier\n  max throw {max_range:.2f} m",
            color="#1565c0", fontsize=9, va="top")
    ax.text((TG.KIN_REACH_X + max_range) / 2, 1.62, "THROW-ONLY zone",
            color="#1565c0", fontsize=10, ha="center", weight="bold")

    # payload swing path
    ax.plot(tcp[:, 0], tcp[:, 2], color="#9c27b0", lw=1.3, alpha=0.6, label="payload swing path")
    # STROBOSCOPIC arm poses through the swing (windup -> release): faded grey -> solid blue, so
    # you can read the whole motion, not just the release instant.
    strobe = list(range(0, k + 1, max(1, k // 7)))
    if strobe[-1] != k:
        strobe.append(k)
    for si, idx in enumerate(strobe):
        sp = arm_points(m, d, q[idx])
        f = si / max(1, len(strobe) - 1)
        ax.plot(sp[:, 0], sp[:, 1], "-", color=(0.5 * (1 - f), 0.55 * (1 - f) + 0.3 * f, 0.55 * (1 - f) + 0.9 * f),
                lw=1.6, alpha=0.25 + 0.5 * f, zorder=2)
    # arm stick figure at release (solid)
    ax.plot(pts[:, 0], pts[:, 1], "-o", color="#0d47a1", lw=3, ms=5, label="OMNIARM6 at release", zorder=5)
    ax.plot([pts[0, 0]], [pts[0, 1]], "s", color="#222", ms=12, zorder=6)   # base

    # ballistic arc + landing
    ax.plot(bx, bz, color="#e53935", lw=2.2, label="ballistic flight")
    ax.plot([p0[0]], [p0[2]], "o", color="#e53935", ms=9)
    ax.annotate(f"release\n{np.linalg.norm(v0):.1f} m/s @ {np.degrees(np.arctan2(v0[2], v0[0])):+.0f}°",
                (p0[0], p0[2]), textcoords="offset points", xytext=(-10, 12), fontsize=9, color="#b71c1c")

    # target bin
    bw = 0.18
    ax.add_patch(Rectangle((bin_x - bw / 2, 0), bw, TG.BIN_RIM_Z, fill=False, ec="#e65100", lw=2))
    ax.plot([land[0]], [TG.BIN_RIM_Z], "*", color="#e65100", ms=18, label=f"lands @ {land[0]:.2f} m")
    ax.text(bin_x, -0.13, f"bin @ {bin_x:.1f} m\n(beyond reach)", color="#e65100", fontsize=9, ha="center")

    # reachable-landing ticks (frontier sampling)
    for lx in lands:
        ax.plot([lx], [TG.BIN_RIM_Z], "|", color="#1565c0", ms=8, alpha=0.5)

    ax.set_xlim(-1.0, max(2.0, max_range + 0.4))
    ax.set_ylim(-0.25, 1.75)
    ax.set_aspect("equal")
    ax.set_xlabel("forward distance from base x (m)")
    ax.set_ylabel("height z (m)")
    ax.set_title(f"OMNIARM6 toss-to-place GHOST  —  feasible (velocity ratio {velr:.2f}), "
                 f"throws {land[0]:.2f} m into a bin the arm cannot reach", fontsize=11)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.15)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"[preview] wrote {os.path.relpath(args.out, REPO)}")


if __name__ == "__main__":
    main()
