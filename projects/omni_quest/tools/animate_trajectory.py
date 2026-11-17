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

"""Animate an Omni Quest run into a GIF: the Husky driving its course.

Reads _trajectory.csv and _course.json (waypoints + obstacles) and writes a GIF.
Works for the flat M1 course and the off-road obstacle course.

    python projects/omni_quest/tools/animate_trajectory.py [run.csv] [out.gif]
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

PROJ = Path(__file__).resolve().parents[1]
_DEF_ROUTE = [(20, 0, "east_gate"), (20, 20, "ne_corner"),
              (0, 20, "north_post"), (0, 0, "home")]
N_FRAMES = 220
FPS = 25


def load_course():
    p = PROJ / "_course.json"
    if p.exists():
        c = json.loads(p.read_text(encoding="utf-8"))
        return c.get("route", _DEF_ROUTE), c.get("obstacles", []), c.get("start")
    return _DEF_ROUTE, [], None


def main() -> int:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else PROJ / "_trajectory.csv"
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else PROJ / "docs" / "offroad_run.gif"
    if not csv_path.exists():
        print(f"[anim] no trajectory at {csv_path} — run the world first", file=sys.stderr)
        return 1
    route, obstacles, start = load_course()

    t, te, tn, oe, on, hd, wp = [], [], [], [], [], [], []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            te.append(float(row["true_e"])); tn.append(float(row["true_n"]))
            oe.append(float(row["obs_e"])); on.append(float(row["obs_n"]))
            hd.append(float(row["heading_deg"])); wp.append(int(row["wp_idx"]))
    n = len(t)
    idx = [min(n - 1, round(k * (n - 1) / (N_FRAMES - 1))) for k in range(N_FRAMES)]

    xs = te + [e for e, _, _ in route] + ([start[0]] if start else [])
    ys = tn + [nn for _, nn, _ in route] + ([start[1]] if start else [])
    pad = 4
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.grid(True, ls=":", alpha=0.5)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")

    for j, (x, y, r) in enumerate(obstacles):
        ax.add_patch(plt.Circle((x, y), r, color="forestgreen", alpha=0.55, zorder=2,
                                 label="obstacles (trees)" if j == 0 else None))
    for i, (e, nn, name) in enumerate(route):
        ax.scatter([e], [nn], marker="*", s=240, c="crimson", edgecolors="black",
                   linewidths=0.6, zorder=5, label="GPS waypoints" if i == 0 else None)
        ax.annotate(f"{i}", (e, nn), textcoords="offset points", xytext=(7, 6),
                    fontsize=9, fontweight="bold", zorder=6)
    if start:
        ax.scatter([start[0]], [start[1]], marker="o", s=70, c="white",
                   edgecolors="black", linewidths=1.0, zorder=5, label="start")

    gps = ax.scatter([], [], s=5, c="tab:orange", alpha=0.20, linewidths=0,
                     label="GPS fixes (noisy)")
    (path_line,) = ax.plot([], [], "-", c="tab:blue", lw=1.9, label="true path")
    (bot,) = ax.plot([], [], "o", c="navy", ms=8, zorder=7)
    arrow = ax.annotate("", xytext=(0, 0), xy=(0, 0),
                        arrowprops=dict(arrowstyle="-|>", color="navy", lw=2))
    ax.legend(loc="upper left", fontsize=7, framealpha=0.92)
    title = ax.set_title("")

    def update(frame):
        s = idx[frame]
        gps.set_offsets(list(zip(oe[:s + 1], on[:s + 1])) or [(0, 0)])
        path_line.set_data(te[:s + 1], tn[:s + 1])
        x, y, h = te[s], tn[s], math.radians(hd[s])
        bot.set_data([x], [y])
        arrow.set_position((x, y))
        arrow.xy = (x + 2.0 * math.cos(h), y + 2.0 * math.sin(h))
        name = route[min(wp[s], len(route) - 1)][2]
        title.set_text(f"Omni Quest off-road — heading to {wp[s]}:{name}    t = {t[s]:4.0f} s")
        return gps, path_line, bot, arrow, title

    anim = FuncAnimation(fig, update, frames=N_FRAMES, interval=1000 / FPS, blit=False)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out_path, writer=PillowWriter(fps=FPS))
    print(f"[anim] wrote {out_path}  ({N_FRAMES} frames, "
          f"{out_path.stat().st_size / 1024:.0f} KB, {len(obstacles)} obstacles)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
