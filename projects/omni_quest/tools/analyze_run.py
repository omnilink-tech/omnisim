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

"""Follow an Omni Quest run through its telemetry and report how it performed.

Reads _trajectory.csv (the run) + _course.json (route + obstacles) and prints a
performance breakdown: per-leg timing & path efficiency, closest approach to any
obstacle (near-misses), where the robot slowed, and GPS-error stats.

    python projects/omni_quest/tools/analyze_run.py
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]


def pct(sorted_vals, q):
    if not sorted_vals:
        return 0.0
    i = min(len(sorted_vals) - 1, int(q * len(sorted_vals)))
    return sorted_vals[i]


def main() -> int:
    csv_path = PROJ / "_trajectory.csv"
    course_path = PROJ / "_course.json"
    if not csv_path.exists() or not course_path.exists():
        print("[analyze] need _trajectory.csv and _course.json — run a world first",
              file=sys.stderr)
        return 1
    course = json.loads(course_path.read_text(encoding="utf-8"))
    route = course["route"]
    obstacles = course["obstacles"]
    start = course.get("start")

    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t"]), float(r["true_e"]), float(r["true_n"]),
                         float(r["obs_e"]), float(r["obs_n"]), int(r["wp_idx"])))
    if len(rows) < 2:
        print("[analyze] not enough samples", file=sys.stderr)
        return 1

    n_legs = len(route)
    leg_time = [0.0] * n_legs
    leg_path = [0.0] * n_legs
    leg_minclear = [math.inf] * n_legs
    errs = []
    speeds = []
    slow_time = 0.0
    total_path = 0.0
    total_headturn = 0.0
    prev = rows[0]
    global_minclear = math.inf
    global_minclear_at = None
    close_calls = 0

    for cur in rows[1:]:
        t, te, tn, oe, on, wp = cur
        pt, pe, pn, _, _, pwp = prev
        dt = t - pt
        seg = math.hypot(te - pe, tn - pn)
        total_path += seg
        if 0 <= wp < n_legs:
            leg_path[wp] += seg
            leg_time[wp] += dt
        if dt > 0:
            v = seg / dt
            speeds.append(v)
            if v < 0.4:
                slow_time += dt
        errs.append(math.hypot(oe - te, on - tn))
        # clearance to nearest obstacle edge (disc radius already includes margin)
        for ox, oy, r in obstacles:
            clr = math.hypot(te - ox, tn - oy) - r
            if clr < global_minclear:
                global_minclear = clr
                global_minclear_at = (round(te, 1), round(tn, 1))
            if 0 <= wp < n_legs and clr < leg_minclear[wp]:
                leg_minclear[wp] = clr
            if clr < 0.3:
                close_calls += 1
        prev = cur

    errs.sort()
    speeds_sorted = sorted(speeds)
    total_time = rows[-1][0]

    # straight-line route length (start -> wp0 -> wp1 -> ...)
    pts = ([tuple(start)] if start else [(rows[0][1], rows[0][2])])
    pts += [(e, n) for e, n, _ in route]
    straight_total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(pts, pts[1:]))

    print("=" * 64)
    print(f"OMNI QUEST RUN ANALYSIS  ({len(rows)} samples)")
    print("=" * 64)
    print(f"waypoints reached : {n_legs}/{n_legs}")
    print(f"total time        : {total_time:.1f} s")
    print(f"path driven       : {total_path:.1f} m  (straight-line route "
          f"{straight_total:.1f} m)")
    print(f"path efficiency   : {100*straight_total/total_path:4.1f} %  "
          f"(100% = no detour; lower = more weaving around obstacles)")
    print(f"avg speed         : {total_path/total_time:.2f} m/s   "
          f"max {pct(speeds_sorted,0.99):.2f} m/s")
    print(f"time slowed (<0.4): {100*slow_time/total_time:4.1f} %  "
          f"(speed scaled down near obstacles)")
    print(f"GPS error         : mean {sum(errs)/len(errs):.2f} m   "
          f"p95 {pct(errs,0.95):.2f} m   peak {errs[-1]:.2f} m")
    print(f"closest obstacle  : {global_minclear:.2f} m clearance at "
          f"ENU {global_minclear_at}  "
          f"({'CONTACT!' if global_minclear < 0 else 'OK'})")
    print(f"close calls (<0.3m): {close_calls} samples")
    print("-" * 64)
    print(f"{'leg':<22}{'time':>7}{'path':>8}{'eff':>7}{'minClear':>10}")
    for i, (e, n, name) in enumerate(route):
        straight = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        effv = 100 * straight / leg_path[i] if leg_path[i] > 0 else 0
        mc = leg_minclear[i] if leg_minclear[i] != math.inf else float("nan")
        print(f"{i}:{name:<20}{leg_time[i]:6.1f}s{leg_path[i]:7.1f}m"
              f"{effv:6.0f}%{mc:9.2f}m")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
