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

"""Plot an Omni Quest run: true path vs the noisy GPS, over the course.

Reads _trajectory.csv (the run) and _course.json (waypoints + obstacles, written
by the controller) and writes a PNG. Works for both the flat M1 course and the
off-road obstacle course.

    python projects/omni_quest/tools/plot_trajectory.py [run.csv] [out.png]
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

PROJ = Path(__file__).resolve().parents[1]

# Fallback course (flat M1) if no _course.json is present.
_DEF_ROUTE = [(20, 0, "east_gate"), (20, 20, "ne_corner"),
              (0, 20, "north_post"), (0, 0, "home")]


def load_course():
    p = PROJ / "_course.json"
    if p.exists():
        c = json.loads(p.read_text(encoding="utf-8"))
        return c.get("route", _DEF_ROUTE), c.get("obstacles", []), c.get("start")
    return _DEF_ROUTE, [], None


def main() -> int:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else PROJ / "_trajectory.csv"
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else PROJ / "docs" / "offroad_trajectory.png"
    if not csv_path.exists():
        print(f"[plot] no trajectory at {csv_path} — run the world first", file=sys.stderr)
        return 1
    route, obstacles, start = load_course()

    te, tn, oe, on, t = [], [], [], [], []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            te.append(float(row["true_e"])); tn.append(float(row["true_n"]))
            oe.append(float(row["obs_e"])); on.append(float(row["obs_n"]))
    duration = t[-1] if t else 0.0
    peak_err = max((math.hypot(a - c, b - d) for a, b, c, d in zip(oe, on, te, tn)),
                   default=0.0)

    fig, ax = plt.subplots(figsize=(8.5, 8.5))

    # Obstacle field.
    for j, (x, y, r) in enumerate(obstacles):
        ax.add_patch(plt.Circle((x, y), r, color="forestgreen", alpha=0.55, zorder=2,
                                 label="obstacles (trees)" if j == 0 else None))
    # Noisy GPS + true path.
    ax.scatter(oe, on, s=5, c="tab:orange", alpha=0.20, linewidths=0,
               label="GPS fixes (noisy)", zorder=3)
    ax.plot(te, tn, "-", c="tab:blue", lw=1.9, label="true path driven", zorder=4)

    # Waypoints + start.
    for i, (e, n, name) in enumerate(route):
        ax.scatter([e], [n], marker="*", s=260, c="crimson", edgecolors="black",
                   linewidths=0.6, zorder=6, label="GPS waypoints" if i == 0 else None)
        ax.annotate(f"{i}:{name}", (e, n), textcoords="offset points", xytext=(8, 8),
                    fontsize=8, fontweight="bold", zorder=7)
    if start:
        ax.scatter([start[0]], [start[1]], marker="o", s=90, c="white",
                   edgecolors="black", linewidths=1.0, zorder=6, label="start")

    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.5)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    kind = "camera-based obstacle avoidance" if obstacles else "flat GPS waypoints"
    ax.set_title(f"Omni Quest — {kind}\n"
                 f"{len(route)} GPS waypoints reached in {duration:.0f}s · "
                 f"{len(obstacles)} obstacles avoided · peak GPS error {peak_err:.2f} m")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.92)
    ax.margins(0.08)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    print(f"[plot] wrote {out_path}  ({len(t)} samples, {duration:.1f}s, "
          f"{len(obstacles)} obstacles)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
