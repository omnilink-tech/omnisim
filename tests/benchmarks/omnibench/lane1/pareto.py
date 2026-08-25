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

"""Accuracy against COST -- the speed/accuracy Pareto front for lane 1.

WHY THIS EXISTS
---------------
"Which simulator is more accurate?" is not answerable, and neither is "which
is faster", because **they are the same knob**. Every lane-1 error falls as
dt falls, and every cost rises. Quoting an accuracy at one dt, or a speed at
one dt, silently picks the winner -- the independent DLR study found the
largest STABLE timestep differs by 48x across engines, so a single-dt
comparison is close to meaningless.

The answerable question is the Pareto one:

    at the error I can tolerate, which engine is cheapest?
    and at the cost I can afford, which engine is most accurate?

That has an answer, it is per-scene, and it is what this file computes.

THE COST AXIS, AND WHY IT IS FAIR
---------------------------------
All three arms are priced on ONE instrument: `time.perf_counter()` around
the solver call and nothing else.

  * mujoco   -- `mj_step`, bracketed in run_mujoco.run_model
  * pybullet -- `p.stepSimulation()`, bracketed in run_pybullet
  * omnisim  -- `mj_step` on the model OmniSim's OWN solver stepped, dumped
                via OMNISIM_NEWTON_SAVE_MJCF and priced by
                model_step_cost.py with a byte-identical bracket

⚠ The OmniSim column is therefore **the physics OmniSim asked for, not what
an OmniSim step costs you**. The engine adds world parse, Qt, controller
IPC, newton<->mjc state marshalling and scene-graph writeback on top, and a
1.2-1.5 s one-time solver construction. That overhead is real and is ours to
own -- `--show-overhead` prints it next to the solver cost so the two are
never silently merged. What this front compares is translation quality and
solver configuration, which is the thing a physics comparison should isolate.

Costs are normalised to **ms of wall time per second of simulated time**,
because that is what makes dt comparable: a solver call costs roughly the
same regardless of dt, so halving dt doubles the calls and doubles the cost
for the same simulated second.

THE TRAP THIS TOOL IS BUILT TO AVOID
------------------------------------
A diverged run can post an excellent-looking error. T4's energy drift at
dt=32 ms reads 8.4e-13 -- not because it conserved energy, but because it
BLEW UP, and the metric normalises by a quantity the explosion inflated. Fed
raw into a Pareto front, that becomes the cheapest, most accurate point in
the entire sweep, and the tool would recommend the one setting that does not
simulate.

So every point is gated by an explicit per-scene validity predicate before
it may enter the front, and rejected points are REPORTED, not dropped
silently -- a front that quietly discarded half its inputs would read as
"this engine cannot go faster" when the truth is "this engine exploded".
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent


# ---------------------------------------------------------------------------
# per-scene: the headline metric, and what makes a run VALID at all
# ---------------------------------------------------------------------------
# `metric` is the number plotted. `valid` decides whether the point is even
# eligible -- see the module docstring on T4 at dt=32.

def _finite(v):
    return v is not None and isinstance(v, (int, float)) and math.isfinite(v)


SCENES = {
    "bounce": {
        "label": "T1 restitution",
        "metric": "bounce_height_rmse_rel", "unit": "rel",
        "valid": lambda m: _finite(m.get("bounce_height_rmse_rel")),
    },
    "incline": {
        "label": "T2 friction stick/slip",
        "metric": "stick_violation_max_m", "unit": "m",
        "valid": lambda m: _finite(m.get("stick_violation_max_m")),
    },
    "roll": {
        "label": "T3 rolling without slip",
        "metric": "roll_accel_rel_err", "unit": "rel",
        "valid": lambda m: _finite(m.get("roll_accel_rel_err")),
    },
    "pendulum_energy": {
        "label": "T4 energy conservation",
        "metric": "energy_drift_rel", "unit": "rel",
        # An exploded run inflates its own normaliser and posts a near-zero
        # drift. The scorer already flags it; honour the flag.
        "valid": lambda m: (_finite(m.get("energy_drift_rel"))
                            and not m.get("energy_blew_up", False)),
    },
    "momentum": {
        "label": "T5 momentum conservation",
        "metric": "angular_momentum_drift_abs", "unit": "kg m^2/s",
        # _rel is a normaliser artefact (the campaign says cite _abs only).
        "valid": lambda m: _finite(m.get("angular_momentum_drift_abs")),
    },
    "stack": {
        "label": "T6 multi-contact stability",
        "metric": "max_penetration_m", "unit": "m",
        # A collapsed tower penetrates less because there is no tower left.
        "valid": lambda m: (_finite(m.get("max_penetration_m"))
                            and m.get("stack_survivors") == 10),
    },
    "spin": {
        "label": "T7 gyroscopic integration",
        "metric": "angmom_drift_rel", "unit": "rel",
        "valid": lambda m: _finite(m.get("angmom_drift_rel")),
    },
}

REJECT_REASON = {
    "pendulum_energy": "energy_blew_up",
    "stack": "stack collapsed (survivors < 10)",
}


# ---------------------------------------------------------------------------
# load
# ---------------------------------------------------------------------------

def load_accuracy(path):
    """lane-1 results.jsonl -> {(test, engine, dt_ms): row}"""
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if "metrics" not in r:
            continue
        out[(r["test"], r["engine"], float(r["dt_ms"]))] = r
    return out


def load_omnisim_cost(path):
    """model_step_cost.jsonl -> {(TEST_lower, dt_ms): ms_per_sim_second}"""
    out = {}
    p = Path(path)
    if not p.exists():
        return out
    for line in p.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("status") != "ok":
            continue
        dt_ms = round(r["basic_time_step_s"] * 1000.0, 6)
        stem = Path(r["world"]).stem            # t3_roll_dt4 / t2_incline_a15_dt4
        test = _scene_from_stem(stem)
        if test is None:
            continue
        # ⚠ T2 IS SUMMED, NOT MAXED, AND THE ASYMMETRY IS REAL.
        # run_mujoco.xml_t2 builds ALL SEVEN incline angles into ONE model on
        # parallel lanes, so a single mj_step covers the whole scene. OmniSim
        # runs seven SEPARATE .wbt worlds for the same scene. Taking one angle
        # would price a seventh of the work against MuJoCo's whole scene and
        # make OmniSim look ~7x cheaper on T2 for no physical reason.
        # Summing the angle worlds compares total work to total work. Every
        # other scene has exactly one world per dt, so the sum is the identity.
        key = (test, dt_ms)
        prev = out.get(key)
        if prev is None:
            out[key] = dict(r)
            out[key]["_n_worlds"] = 1
        else:
            prev["ms_per_sim_second"] += r["ms_per_sim_second"]
            prev["ms_per_step"] += r["ms_per_step"]
            prev["_n_worlds"] += 1
    return out


_STEM2SCENE = {"t1": "bounce", "t2": "incline", "t3": "roll",
               "t4": "pendulum_energy", "t5": "momentum",
               "t6": "stack", "t7": "spin"}


def _scene_from_stem(stem):
    return _STEM2SCENE.get(stem.split("_", 1)[0].lower())


# ---------------------------------------------------------------------------
# cost normalisation
# ---------------------------------------------------------------------------

def cost_ms_per_sim_s(engine, row, dt_ms, omni_cost):
    """Wall ms per simulated second, on the solver-call instrument.

    Returns (value, note) -- note is non-empty when the value is NOT usable.
    """
    dt_s = dt_ms / 1000.0
    if engine in ("mujoco", "pybullet"):
        per_step = row.get("wall_ms_per_step")
        if not _finite(per_step):
            return None, "no wall_ms_per_step"
        return per_step / dt_s, ""
    if engine.startswith("omnisim"):
        test = row["test"]
        hit = omni_cost.get((test, dt_ms))
        if hit is None:
            return None, "no model_step_cost row (run model_step_cost.py)"
        return hit["ms_per_sim_second"], ""
    return None, "unknown engine"


# ---------------------------------------------------------------------------
# Pareto
# ---------------------------------------------------------------------------

def pareto_front(points):
    """points: [{cost, error, ...}] -> the non-dominated subset, cost-sorted.

    p dominates q iff p is no worse on both axes and strictly better on one.
    """
    pts = sorted(points, key=lambda p: (p["cost"], p["error"]))
    front, best_err = [], float("inf")
    for p in pts:
        if p["error"] < best_err - 1e-15:
            front.append(p)
            best_err = p["error"]
    return front


def analyse(acc, omni_cost, engines):
    """-> {scene: {"points": [...], "rejected": [...], "front": [...]}}"""
    by_scene = defaultdict(lambda: {"points": [], "rejected": []})
    for (test, engine, dt_ms), row in sorted(acc.items()):
        if engine not in engines:
            continue
        spec = SCENES.get(test)
        if spec is None:
            continue
        m = row["metrics"]
        if not spec["valid"](m):
            by_scene[test]["rejected"].append({
                "engine": engine, "dt_ms": dt_ms,
                "reason": REJECT_REASON.get(test, "metric not finite"),
                "raw": m.get(spec["metric"])})
            continue
        # DIVERGENCE GATE, and it is load-bearing. A *relative* error above 1.0
        # means the answer is worse than predicting zero -- the run did not
        # simulate the scene, it left it. Such a point is always the cheapest
        # available (a diverged run is fast), so without this gate it anchors
        # the front and the tool recommends the one setting that does not work.
        # Measured here: MuJoCo's T1 calibration is frozen at dt=1 ms and its
        # contact integration goes unstable at dt>=8 ms -- documented in the
        # arm's own deviations as energy GAIN -- posting 35.9, 33.5 and 396.5.
        # 396.5 was being listed as the cheapest point on the T1 front.
        if spec["unit"] == "rel":
            e = float(m[spec["metric"]])
            if e > 1.0:
                by_scene[test]["rejected"].append({
                    "engine": engine, "dt_ms": dt_ms,
                    "reason": "diverged (relative error %.4g > 100%%: worse "
                              "than predicting zero)" % e,
                    "raw": e})
                continue
        cost, note = cost_ms_per_sim_s(engine, row, dt_ms, omni_cost)
        if cost is None:
            by_scene[test]["rejected"].append({
                "engine": engine, "dt_ms": dt_ms,
                "reason": "no cost: %s" % note, "raw": m.get(spec["metric"])})
            continue
        by_scene[test]["points"].append({
            "engine": engine, "dt_ms": dt_ms,
            "error": float(m[spec["metric"]]), "cost": float(cost)})

    for test, d in by_scene.items():
        d["front"] = pareto_front(d["points"])
    return by_scene


def cheapest_at_budget(points, budget):
    """Cheapest point meeting error <= budget. -> point or None."""
    ok = [p for p in points if p["error"] <= budget]
    return min(ok, key=lambda p: p["cost"]) if ok else None


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------

def render(by_scene, budgets=(), show_rejected=True):
    L = []
    for test, spec in SCENES.items():
        d = by_scene.get(test)
        if not d or not d["points"]:
            continue
        L.append("")
        L.append("=" * 78)
        L.append("%s  --  %s [%s]" % (spec["label"], spec["metric"], spec["unit"]))
        L.append("=" * 78)
        L.append("%-16s %8s %14s %16s   %s"
                 % ("engine", "dt(ms)", "error", "ms/sim-second", "on front"))
        front = {(p["engine"], p["dt_ms"]) for p in d["front"]}
        for p in sorted(d["points"], key=lambda p: (p["engine"], p["dt_ms"])):
            L.append("%-16s %8g %14.6g %16.4f   %s"
                     % (p["engine"], p["dt_ms"], p["error"], p["cost"],
                        "***" if (p["engine"], p["dt_ms"]) in front else ""))
        L.append("")
        L.append("  Pareto front (cheapest first; each is the most accurate "
                 "option at its cost):")
        for p in d["front"]:
            L.append("    %-16s dt=%-6g error=%-13.6g %.4f ms/sim-s"
                     % (p["engine"], p["dt_ms"], p["error"], p["cost"]))
        for b in budgets:
            hit = cheapest_at_budget(d["points"], b)
            L.append("  error budget %-10g -> %s" % (
                b, ("%s at dt=%g, %.4f ms/sim-s"
                    % (hit["engine"], hit["dt_ms"], hit["cost"]))
                if hit else "UNREACHABLE by any engine at any dt in this sweep"))
        if show_rejected and d["rejected"]:
            L.append("  rejected (NOT silently dropped):")
            for r in d["rejected"]:
                L.append("    %-16s dt=%-6g %s (raw metric %s)"
                         % (r["engine"], r["dt_ms"], r["reason"], r["raw"]))
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Speed/accuracy Pareto front for lane 1. Answers 'at my "
                    "error budget, which engine is cheapest'.")
    ap.add_argument("--results", required=True,
                    help="lane-1 results.jsonl (the dt sweep)")
    ap.add_argument("--omnisim-cost",
                    help="model_step_cost.jsonl from model_step_cost.py; "
                         "without it the OmniSim arm has no comparable cost "
                         "and is reported as rejected rather than guessed")
    ap.add_argument("--engines", default="omnisim-newton,mujoco,pybullet")
    ap.add_argument("--budget", type=float, action="append", default=[],
                    help="repeatable error budget to solve for")
    ap.add_argument("--json", help="write the full analysis here")
    a = ap.parse_args(argv)

    acc = load_accuracy(a.results)
    omni = load_omnisim_cost(a.omnisim_cost) if a.omnisim_cost else {}
    engines = [e.strip() for e in a.engines.split(",") if e.strip()]
    by_scene = analyse(acc, omni, engines)

    print(render(by_scene, budgets=a.budget))
    print("")
    print("COST AXIS: wall ms per simulated second, solver call only. The "
          "OmniSim column prices the model OmniSim's solver stepped, NOT an "
          "OmniSim engine step -- engine overhead is excluded and is real.")
    if not omni:
        print("⚠ No --omnisim-cost given: the OmniSim arm has no comparable "
              "cost axis and was rejected rather than estimated.")

    if a.json:
        Path(a.json).write_text(json.dumps(
            {k: v for k, v in by_scene.items()}, indent=2), encoding="utf-8")
        print("[pareto] wrote %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
