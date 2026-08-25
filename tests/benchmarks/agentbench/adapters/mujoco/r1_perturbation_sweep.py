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

"""How big must R1's seeded perturbation be before it actually discriminates?

``tasks/R1_lidar_nav/meta.json`` declares the perturbation as the answer to
memorisation: the agent authors against the published obstacle layout and is
graded against one it has never seen, so "a robot that senses adapts; a
hardcoded path collides". Measured on the MuJoCo arm at the declared
``PERTURBATION_MAX_M`` = 0.6 m, that defence fires **3 times in 20**: a
hardcoded path planned with clearance to spare simply does not care about a
0.6 m nudge. This tool finds the displacement where it does -- if there is one.

A filter has to do two things and the second is the one that is easy to lose:

* **catch the memoriser** -- ``r1_hardcode.py``, which reads
  ``obstacles.json`` and casts zero beams, must FAIL;
* **spare the sensor** -- ``r1_oracle.py``, which reads no layout, must still
  PASS. A displacement that breaks the honest navigator too is not a filter,
  it is noise, and it would publish as the agent's failure.

Both rates are measured over the same seeds at each level, so they are
comparable, and three things that would otherwise inflate the headline are
measured alongside:

1. **the fallback.** ``r1_core.perturbed_spec`` refuses any layout whose
   straight START->GOAL line is no longer blocked and returns the PUBLISHED
   layout instead. That fallback is the memoriser's best case, so every
   fallback is a guaranteed escape and belongs in the denominator;
2. **legality.** Nothing in ``perturbed_spec`` checks that the moved boxes stay
   inside the arena, stay off each other, stay off the start and the goal, or
   leave any route at all. At larger displacements they stop doing so, and a
   run that fails because the layout is impossible is not evidence about the
   agent;
3. **why the memoriser failed** -- a collision (the mechanism the task's prose
   claims) or a failure to arrive (which a harder layout also produces).

Verdicts come from the unmodified ``graders.r1_core``. "Caught" is defined on
the BEHAVIOURAL trio R1.4/R1.5/R1.6, because R1.3 matches the world against
the PUBLISHED spec and therefore fails for everyone on a perturbed layout --
see the note this tool prints about what wiring the perturbation also requires.

Free to run: MuJoCo in-process, no GPU, no network, no model quota.

    python adapters/mujoco/r1_perturbation_sweep.py \\
        --levels 0.6,1.0,1.4,1.8,2.2 --seeds 1-20 --workers 6
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.mujoco import evidence, launcher  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

LANE = launcher.LANE
SCENE = LANE / "r1_oracle.xml"
DURATION_S = 60.0

#: Inner face of the arena walls in r1_oracle.xml (wall centre 5.0, half 0.1).
ARENA_INNER_M = 4.9
#: What bounds the robot's footprint (r1_oracle.ROBOT_RADIUS_M).
ROBOT_RADIUS_M = 0.30
#: Inflation for the independent "is this layout solvable at all" check: the
#: tightest squeeze a real robot of that radius could physically take.
SOLVABLE_INFLATE_M = 0.35
#: Grid for that check -- finer than the oracle's own planner on purpose, so
#: the legality verdict is not an artefact of the oracle's discretisation.
SOLVE_CELL_M = 0.1

DRIVERS = {"oracle": "r1_oracle.py", "hardcode": "r1_hardcode.py"}


# --- layout legality, measured independently of any driver ------------------

def _aabb(o):
    cx, cy = o["position"][0], o["position"][1]
    sx, sy = o["size"][0], o["size"][1]
    return (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


def _covers(o, pt, pad=0.0):
    x0, y0, x1, y1 = _aabb(o)
    return (x0 - pad <= pt[0] <= x1 + pad) and (y0 - pad <= pt[1] <= y1 + pad)


def solvable(moved, inflate_m=SOLVABLE_INFLATE_M, cell=SOLVE_CELL_M):
    """Is there ANY collision-free route from start to goal in this layout?

    A flood fill on a fine grid with the obstacles and the walls inflated by
    ``inflate_m``. Deliberately not the oracle's planner: this asks whether the
    layout is possible, not whether one particular navigator solves it.
    """
    n = int(round(2 * r1_core.ARENA_HALF_M / cell))
    xs = (np.arange(n) + 0.5) * cell - r1_core.ARENA_HALF_M
    gx, gy = np.meshgrid(xs, xs, indexing="ij")
    blocked = ((np.abs(gx) > ARENA_INNER_M - inflate_m)
               | (np.abs(gy) > ARENA_INNER_M - inflate_m))
    for o in moved:
        x0, y0, x1, y1 = _aabb(o)
        blocked |= ((gx > x0 - inflate_m) & (gx < x1 + inflate_m)
                    & (gy > y0 - inflate_m) & (gy < y1 + inflate_m))

    def cell_of(p):
        return (min(max(int((p[0] + r1_core.ARENA_HALF_M) / cell), 0), n - 1),
                min(max(int((p[1] + r1_core.ARENA_HALF_M) / cell), 0), n - 1))

    src, dst = cell_of(r1_core.START_XY), cell_of(r1_core.GOAL_XY)
    if blocked[src] or blocked[dst]:
        return False
    seen = np.zeros_like(blocked)
    seen[src] = True
    stack = [src]
    while stack:
        cx, cy = stack.pop()
        if (cx, cy) == dst:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < n and 0 <= ny < n and not blocked[nx, ny] \
                    and not seen[nx, ny]:
                seen[nx, ny] = True
                stack.append((nx, ny))
    return False


def legality(moved):
    """Every way a perturbed layout can be illegal, named separately."""
    out = {"outside_arena": [], "overlapping": [], "on_start": [],
            "on_goal": []}
    for o in moved:
        x0, y0, x1, y1 = _aabb(o)
        if (min(x0, y0) < -ARENA_INNER_M or max(x1, y1) > ARENA_INNER_M):
            out["outside_arena"].append(o["name"])
        # A box within a robot radius of the start or the goal makes the pose
        # unoccupiable -- the robot spawns inside it, or the goal cannot be
        # stood on.
        if _covers(o, r1_core.START_XY, ROBOT_RADIUS_M):
            out["on_start"].append(o["name"])
        if _covers(o, r1_core.GOAL_XY, ROBOT_RADIUS_M):
            out["on_goal"].append(o["name"])
    for i, a in enumerate(moved):
        ax0, ay0, ax1, ay1 = _aabb(a)
        for b in moved[i + 1:]:
            bx0, by0, bx1, by1 = _aabb(b)
            if ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
                out["overlapping"].append("%s/%s" % (a["name"], b["name"]))
    out["solvable"] = solvable(moved)
    # FATAL vs COSMETIC, and the split is load-bearing. Two interpenetrating
    # static boxes are a compound obstacle: MuJoCo does not collide two geoms
    # that are both welded to the world, the layout stays solvable, and R1.3
    # still matches five boxes by geometry. It changes the scene's CHARACTER
    # (five obstacles become four effective ones) and is worth reporting, but
    # it does not invalidate a run and must not be used to discard one.
    # Fatal is: a box through the arena wall, a box on the start or the goal,
    # or no collision-free route at all -- each of which makes a FAIL evidence
    # about the LAYOUT rather than about the agent.
    out["fused"] = bool(out["overlapping"])
    out["legal"] = (not out["outside_arena"] and not out["on_start"]
                    and not out["on_goal"] and out["solvable"])
    return out


def layout(seed, max_m):
    """``(moved, applied, legality)`` for one (seed, level)."""
    spec = r1_core.obstacle_spec()
    moved, _offsets, applied = r1_core.perturbed_spec(spec, seed, max_m=max_m)
    return moved, bool(applied), legality(moved)


# --- one run ----------------------------------------------------------------

def write_scene(moved, dest_dir):
    text = SCENE.read_text(encoding="utf-8")
    for o in moved:
        text, n = re.subn(
            r'(<geom name="%s" type="box" pos=")[^"]*(")' % o["name"],
            r"\g<1>%g %g %g\g<2>" % tuple(o["position"]), text)
        if n != 1:
            raise RuntimeError("%s: rewrote %d positions" % (o["name"], n))
    dest_dir.mkdir(parents=True, exist_ok=True)
    p = dest_dir / "r1_oracle.xml"
    p.write_text(text, encoding="utf-8")
    return p


def one_cell(job):
    """Run one (driver, seed, level) and reduce it to the facts. Never raises."""
    driver, seed, level = job
    moved, applied, legal = layout(seed, level)
    rec = {"driver": driver, "seed": seed, "level": level,
           "applied": applied, "legality": legal}
    tmp = Path(tempfile.mkdtemp(prefix="r1sweep_"))
    try:
        model = write_scene(moved, tmp)
        out = tmp / "run"
        launcher.launch(model, out, driver=LANE / DRIVERS[driver],
                        duration=DURATION_S, timeout_s=900.0)
        bundle = evidence.build_bundle("R1_lidar_nav",
                                       robot_identity="any_robot",
                                       run_dir=out, artifact=str(model))
        v = r1_core.grade(bundle)
        ok = {a.id: a.ok for a in v.assertions}
        m = {a.id: a.measured for a in v.assertions}
        rec["ok"] = ok
        rec["behavioural_pass"] = bool(ok.get("R1.4") and ok.get("R1.5")
                                       and ok.get("R1.6"))
        rec["distance_to_goal_m"] = (m.get("R1.4") or {}).get(
            "distance to goal (m)")
        rec["path_m"] = (m.get("R1.6") or {}).get("integrated path length (m)")
        hits = (m.get("R1.5") or {}).get("first hits") or []
        rec["hits"] = hits
        rec["collided"] = bool(hits)
        rec["obstacles_matched"] = (m.get("R1.3") or {}).get(
            "specified obstacles found")
        # The RAW contact pairs, straight off the adapter, next to what R1.5
        # made of them. On a perturbed layout these disagree, and the
        # disagreement is the point: match_spec_obstacles matches against the
        # PUBLISHED spec, so a moved obstacle is not in `obstacle_names` and a
        # contact naming it can never be a hit. R1.5 then reports "nothing was
        # hit" for a robot that is visibly stuck against a box.
        try:
            con = json.loads((out / "contacts.json").read_text("utf-8"))
            rec["raw_robot_obstacle_pairs"] = [
                "%s/%s" % (p.get("a"), p.get("b")) for p in con.get("pairs", [])
                if {"rover"} & {p.get("a"), p.get("b")}
                and any("OBSTACLE" in str(x) for x in (p.get("a"), p.get("b")))]
        except (OSError, ValueError):
            rec["raw_robot_obstacle_pairs"] = None
        traj = bundle.trajectory
        if traj is not None and traj.xyz is not None:
            xy = traj.xy(traj.index_of_name("rover"))
            rec["clearance_m"] = round(
                min(_point_to_box(p, o) for o in moved for p in xy)
                - ROBOT_RADIUS_M, 4)
    except Exception as exc:                                  # noqa: BLE001
        rec["error"] = repr(exc)
        rec["behavioural_pass"] = None
    return rec


def _point_to_box(p, o):
    x0, y0, x1, y1 = _aabb(o)
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


# --- the sweep ---------------------------------------------------------------

def summarise(records, level):
    rows = [r for r in records if r["level"] == level]
    out = {"level": level, "n_seeds": len({r["seed"] for r in rows})}
    seeds = sorted({r["seed"] for r in rows})
    lay = {s: [r for r in rows if r["seed"] == s][0] for s in seeds}
    out["fallback_rate"] = sum(0 if lay[s]["applied"] else 1
                               for s in seeds) / float(len(seeds))
    out["illegal_rate"] = sum(0 if lay[s]["legality"]["legal"] else 1
                              for s in seeds) / float(len(seeds))
    out["unsolvable_rate"] = sum(0 if lay[s]["legality"]["solvable"] else 1
                                 for s in seeds) / float(len(seeds))
    for kind in ("outside_arena", "overlapping", "on_start", "on_goal"):
        out["%s_rate" % kind] = sum(
            1 for s in seeds if lay[s]["legality"][kind]) / float(len(seeds))
    out["fused_rate"] = sum(1 for s in seeds
                            if lay[s]["legality"]["fused"]) / float(len(seeds))
    for driver in DRIVERS:
        d = [r for r in rows if r["driver"] == driver]
        legal = [r for r in d if r["legality"]["legal"]]
        out[driver] = {
            "n": len(d),
            "pass": sum(1 for r in d if r["behavioural_pass"]),
            "pass_rate": (sum(1 for r in d if r["behavioural_pass"])
                          / float(len(d))) if d else None,
            "n_legal": len(legal),
            "pass_legal": sum(1 for r in legal if r["behavioural_pass"]),
            "pass_rate_legal": (sum(1 for r in legal
                                    if r["behavioural_pass"])
                                / float(len(legal))) if legal else None,
            "collided": sum(1 for r in d if r.get("collided")),
            "errors": sum(1 for r in d if r.get("error")),
        }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--levels", default="0.6,1.0,1.4,1.8,2.2")
    ap.add_argument("--seeds", default="1-20")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", default="")
    ap.add_argument("--layouts-only", action="store_true",
                    help="skip the runs; report fallback + legality only "
                         "(instant, and enough for a big seed sample)")
    args = ap.parse_args(argv)

    levels = [float(x) for x in args.levels.split(",")]
    if "-" in args.seeds:
        lo, hi = args.seeds.split("-")
        seeds = list(range(int(lo), int(hi) + 1))
    else:
        seeds = [int(x) for x in args.seeds.split(",")]

    if args.layouts_only:
        print("       -- FATAL --------------------------------  "
              "-- cosmetic --")
        print("level  fallback  fatal  unsolvable  outside  on_start  "
              "on_goal | fused   (n=%d seeds)" % len(seeds))
        for lv in levels:
            lays = [layout(s, lv) for s in seeds]
            n = float(len(lays))
            f = sum(0 if a else 1 for _m, a, _l in lays) / n
            g = lambda k: sum(1 for _m, _a, L in lays if L[k]) / n  # noqa: E731
            il = sum(0 if L["legal"] else 1 for _m, _a, L in lays) / n
            un = sum(0 if L["solvable"] else 1 for _m, _a, L in lays) / n
            print("%5.2f  %7.0f%%  %4.0f%%  %9.0f%%  %6.0f%%  %7.0f%%  "
                  "%6.0f%% | %4.0f%%"
                  % (lv, 100 * f, 100 * il, 100 * un, 100 * g("outside_arena"),
                     100 * g("on_start"), 100 * g("on_goal"),
                     100 * g("overlapping")))
        return 0

    jobs = [(d, s, lv) for lv in levels for s in seeds for d in DRIVERS]
    records = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i, rec in enumerate(pool.map(one_cell, jobs), 1):
            records.append(rec)
            print("  [%3d/%3d] %-9s seed %-4d level %.2f  behavioural=%s%s"
                  % (i, len(jobs), rec["driver"], rec["seed"], rec["level"],
                     rec["behavioural_pass"],
                     "" if rec["legality"]["legal"] else "  (ILLEGAL LAYOUT)"),
                  flush=True)

    print("\n%-6s | %-28s | %-28s | %s"
          % ("level", "memoriser (r1_hardcode)", "sensor (r1_oracle)",
             "layout health"))
    print("%-6s | %-28s | %-28s | %s"
          % ("(m)", "caught / n   (legal only)", "passed / n   (legal only)",
             "fallback  fatal  fused"))
    print("-" * 104)
    summaries = []
    for lv in levels:
        s = summarise(records, lv)
        summaries.append(s)
        h, o = s["hardcode"], s["oracle"]
        print("%-6.2f | %2d/%-2d = %3.0f%%  (%2d/%-2d = %3.0f%%)      | "
              "%2d/%-2d = %3.0f%%  (%2d/%-2d = %3.0f%%)      | "
              "%4.0f%%     %4.0f%%   %4.0f%%"
              % (lv,
                 h["n"] - h["pass"], h["n"],
                 100 * (1 - (h["pass_rate"] or 0)),
                 h["n_legal"] - h["pass_legal"], h["n_legal"],
                 100 * (1 - (h["pass_rate_legal"] or 0)) if h["n_legal"] else 0,
                 o["pass"], o["n"], 100 * (o["pass_rate"] or 0),
                 o["pass_legal"], o["n_legal"],
                 100 * (o["pass_rate_legal"] or 0) if o["n_legal"] else 0,
                 100 * s["fallback_rate"], 100 * s["illegal_rate"],
                 100 * s["fused_rate"]))

    if args.out:
        Path(args.out).write_text(json.dumps(
            {"levels": levels, "seeds": seeds, "summaries": summaries,
             "records": records}, indent=1), encoding="utf-8")
        print("\nwrote %s" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
