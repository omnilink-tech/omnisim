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

"""Numerically verify a world: follow every robot's pose against the obstacles.

Reports each GPS+camera NAVIGATOR (one per _trajectory*.csv + its _last_run*.log
— bare for a single-robot world, suffixed per id for the fleet) and each roaming
robot (_roam_*.csv, logged under OMNI_QUEST_VERIFY=1). For every robot it checks:
moved, stayed upright, stayed in bounds, didn't get stuck, didn't tunnel an
obstacle; navigators also get their waypoint + clearance result.

    python projects/omni_quest/tools/verify_world.py
"""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
PASTURE = (0.0, 29.0, 10.5)        # roamers (populated world) are bounded here


def path_len(xs, ys):
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
               for i in range(1, len(xs)))


def min_clearance(xs, ys, obstacles):
    best, at = math.inf, None
    for x, y in zip(xs, ys):
        for ox, oy, r in obstacles:
            c = math.hypot(x - ox, y - oy) - r
            if c < best:
                best, at = c, (round(x, 1), round(y, 1))
    return best, at


def longest_stuck(ts, xs, ys):
    run = best = 0.0
    for i in range(1, len(xs)):
        if math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) < 0.05:
            run += ts[i] - ts[i - 1]
            best = max(best, run)
        else:
            run = 0.0
    return best


def load_obstacles():
    for p in sorted(PROJ.glob("_course*.json")):
        try:
            return json.loads(p.read_text(encoding="utf-8")).get("obstacles", [])
        except Exception:
            pass
    return []


def read_xy(path, cols):
    xs, ys, zs, ts = [], [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            xs.append(float(row[cols[0]])); ys.append(float(row[cols[1]]))
            ts.append(float(row["t"]))
            zs.append(float(row["z"]) if "z" in row else 0.0)
    return ts, xs, ys, zs


def nav_report(suffix, obstacles):
    name = suffix.lstrip("_") or "husky"
    log = PROJ / f"_last_run{suffix}.log"
    reached, mission = 0, "no log"
    if log.exists():
        txt = log.read_text(encoding="utf-8", errors="replace")
        reached = len(re.findall(r"REACHED wp", txt))
        m = re.search(r"MISSION COMPLETE[^\n]*", txt)
        mission = (m.group(0).split(": ", 1)[-1] if m else "not complete")
    xs, ys, zs, cds, pds = [], [], [], [], []
    tp = PROJ / f"_trajectory{suffix}.csv"
    if tp.exists():
        with open(tp, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                xs.append(float(row["true_e"])); ys.append(float(row["true_n"]))
                zs.append(float(row.get("z") or 0))
                cd = row.get("car_dist")
                if cd not in (None, "", "-1", "-1.0") and float(cd) >= 0:
                    cds.append(float(cd))
                pd = row.get("ped_dist")
                if pd not in (None, "", "-1", "-1.0") and float(pd) >= 0:
                    pds.append(float(pd))
    clr, at = min_clearance(xs, ys, obstacles) if (xs and obstacles) else (float("nan"), None)
    verdict = "OK" if (reached and "COMPLETE" in mission) else "CHECK"
    if obstacles and not clr > -0.4:
        verdict = "CHECK"
    print(f"NAVIGATOR {name}")
    print(f"  waypoints reached : {reached}")
    print(f"  result            : {mission}")
    print(f"  path driven       : {path_len(xs, ys):.1f} m" if xs else "  (no trajectory)")
    if obstacles:
        print(f"  closest obstacle  : {clr:+.2f} m at {at}  "
              f"({'GRAZE' if -0.4 < clr < 0 else 'CONTACT' if clr <= -0.4 else 'clear'})")
    if zs and max(zs) > 0.01:
        zmin, zmax = min(zs), max(zs)
        on_deck = zmin > 0.13           # on the 0.15 m sidewalk deck (ground ~0.07)
        print(f"  height z          : [{zmin:.2f}, {zmax:.2f}] m  "
              f"({'ON the sidewalk' if on_deck else 'SUNK / fell off deck'})")
        if not on_deck:
            verdict = "CHECK"
    if ys:
        print(f"  lateral (north)   : [{min(ys):.1f}, {max(ys):.1f}] m  "
              f"(tight band = stayed on the sidewalk line)")
    if cds:
        cm = min(cds)
        print(f"  closest car       : {cm:.2f} m  "
              f"({'CONTACT — touched a car' if cm < 1.4 else 'clear of traffic'})")
        if cm < 1.4:
            verdict = "CHECK"
    if pds:
        pm = min(pds)
        tag = ("COLLISION — hit a person" if pm < 0.85
               else "close pass" if pm < 1.1 else "clear of people")
        print(f"  closest person    : {pm:.2f} m  ({tag})")
        if pm < 0.85:
            verdict = "CHECK"
    print(f"  VERDICT           : {verdict}")
    print()


def roamer_report(path, obstacles):
    name = path.stem.replace("_roam_", "")
    ts, xs, ys, zs = read_xy(path, ("x", "y"))
    if len(xs) < 2:
        print(f"ROAMER {name}: no data\n"); return
    dist = path_len(xs, ys)
    rng = max(math.hypot(xs[i] - xs[0], ys[i] - ys[0]) for i in range(len(xs)))
    zmin, zmax = min(zs), max(zs)
    clr, _ = min_clearance(xs, ys, obstacles)
    inb = sum(1 for x, y in zip(xs, ys)
              if math.hypot(x - PASTURE[0], y - PASTURE[1]) <= PASTURE[2] + 2.5)
    inb_pct = 100 * inb / len(xs)
    stuck = longest_stuck(ts, xs, ys)
    upright = zmin > -0.25 and zmax < 0.7
    ok = (dist > 5 and upright and inb_pct > 80 and stuck < 12 and clr > -0.4)
    print(f"ROAMER {name}")
    print(f"  moved             : {dist:.1f} m over {ts[-1] - ts[0]:.0f}s  "
          f"(range {rng:.1f} m)  {'MOVING' if dist > 5 else 'STUCK/IDLE'}")
    print(f"  upright (z range) : [{zmin:+.2f}, {zmax:+.2f}]  "
          f"({'upright' if upright else 'TIPPED?'})")
    print(f"  in pasture bounds : {inb_pct:.0f}%   longest stuck {stuck:.1f}s")
    print(f"  VERDICT           : {'OK' if ok else 'CHECK'}")
    print()


def main():
    obstacles = load_obstacles()
    print("=" * 60)
    print(f"OMNI QUEST VERIFICATION  ({len(obstacles)} obstacles)")
    print("=" * 60)
    navs = sorted(PROJ.glob("_trajectory*.csv"))
    if not navs:
        print("(no navigator trajectories found)")
    series = {}
    for traj in navs:
        suffix = traj.stem[len("_trajectory"):]
        nav_report(suffix, obstacles)
        ts, xs, ys, _ = read_xy(traj, ("true_e", "true_n"))
        series[suffix.lstrip("_") or "husky"] = {round(t, 1): (x, y)
                                                 for t, x, y in zip(ts, xs, ys)}

    # Did any two navigators collide while their paths crossed?
    names = list(series)
    if len(names) >= 2:
        print("ROBOT <-> ROBOT closest approach (centre-to-centre):")
        worst = math.inf
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = series[names[i]], series[names[j]]
                shared = set(a) & set(b)
                if not shared:
                    continue
                d = min(math.hypot(a[t][0] - b[t][0], a[t][1] - b[t][1])
                        for t in shared)
                worst = min(worst, d)
                tag = "CONTACT?" if d < 1.0 else ("close pass" if d < 2.0 else "ok")
                print(f"  {names[i]:<6}<-> {names[j]:<6}: {d:5.2f} m  ({tag})")
        print(f"  -> worst pair: {worst:.2f} m  "
              f"({'CONTACT' if worst < 1.0 else 'all robots avoided each other'})")
        print()

    for p in sorted(PROJ.glob("_roam_*.csv")):
        roamer_report(p, obstacles)
    print("=" * 60)


if __name__ == "__main__":
    main()
