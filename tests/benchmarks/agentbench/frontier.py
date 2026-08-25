"""Verified Build Frontier: a claim-safe view over AgenticSimBench rows.

This module does not invent a second score or a second grader.  It turns the
task verdicts already produced by AgenticSimBench into a small set of ordered
build tracks.  A track's frontier is the longest contiguous prefix completed;
passing one spectacular task cannot hide a missing easier prerequisite.

Two frontiers are always reported:

* ``measured`` -- what the selected rows physically passed.  Useful during
  development, but exploratory when a task's publication gates are not green.
* ``claimable`` -- the same prefix with ``readiness.py`` applied per task and
  simulator.  This is the only frontier suitable for external claims.

The distinction is load-bearing.  It prevents a polished chart from laundering
an ungated grader or a stale red-evidence table into marketing evidence.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCHMARKS = HERE.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench import readiness


FRONTIER_ID = "agenticsimbench/verified-build-frontier-v0.1"


@dataclass(frozen=True)
class Track:
    id: str
    title: str
    tasks: tuple[str, ...]
    meaning: str


# Order is a product claim, so it lives in code and is regression-tested.
# Tasks may appear in more than one track when they are genuine prerequisites
# for both (R2 is the actuation floor for the manipulation track).
TRACKS = (
    Track(
        "scene_reasoning",
        "Scene understanding and quantitative iteration",
        ("B1_overlap_audit", "B3_measure_and_report",
         "B2_subject_in_frame"),
        "inspect structure, measure geometry, then change and verify a view",
    ),
    Track(
        "debug_loop",
        "Authoring and physics debugging",
        ("C1_parse_error_fix", "C2_fall_through_floor"),
        "repair syntax first, then diagnose and verify a physical failure",
    ),
    Track(
        "autonomy_scale",
        "Closed-loop autonomy and multi-robot scale",
        ("R1_lidar_nav", "A1_husky_swarm_10"),
        "navigate one sensed robot, then build and verify ten moving robots",
    ),
    Track(
        "manipulation",
        "Manipulation complexity",
        ("R2_arm_reach", "R3_pick_and_place", "R4_mobile_manipulation"),
        "reach, then pick/place, then combine navigation and manipulation",
    ),
)

TASKS = tuple(dict.fromkeys(task for track in TRACKS for task in track.tasks))


class FrontierError(ValueError):
    """The selected rows cannot form one comparable frontier."""


def load_rows(paths):
    rows = []
    for raw in paths:
        path = Path(raw)
        with path.open("r", encoding="utf-8") as stream:
            for lineno, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except ValueError as exc:
                    raise FrontierError(
                        "%s:%d is not valid JSON: %s" % (path, lineno, exc))
                row.setdefault("_source", "%s:%d" % (path, lineno))
                rows.append(row)
    return rows


def _model(row):
    return (row.get("agent") or {}).get("model")


def select_rows(rows, *, sim, condition=None, model=None):
    """Select exactly one row per task and reject mixed experiments."""
    selected = []
    for row in rows:
        if row.get("task") not in TASKS or row.get("sim") != sim:
            continue
        if condition is not None and row.get("condition") != condition:
            continue
        if model is not None and _model(row) != model:
            continue
        selected.append(row)

    by_task = {}
    for row in selected:
        task = row.get("task")
        if task in by_task:
            raise FrontierError(
                "more than one selected row for %s (%s and %s); a frontier "
                "uses one run per task, so narrow --condition/--model or "
                "provide one campaign"
                % (task, by_task[task].get("_source", "unknown"),
                   row.get("_source", "unknown")))
        by_task[task] = row

    suites = {row.get("suite") for row in selected}
    protocols = {
        ((row.get("protocol") or {}).get("id")
         if isinstance(row.get("protocol"), dict) else row.get("protocol"))
        for row in selected
    }
    models = {_model(row) for row in selected}
    conditions = {row.get("condition") for row in selected}
    for name, values in (("suite", suites), ("protocol", protocols),
                         ("model", models), ("condition", conditions)):
        if len(values) > 1:
            raise FrontierError("selected rows mix %s values: %s" % (
                name, sorted(repr(v) for v in values)))
    return by_task


def publication_ready(task_id, sim):
    """Return ``(bool, reasons)`` from the same gates readiness.py prints."""
    _task, rows = readiness.check(task_id)
    gates = next((g for sid, g in rows if sid == sim), None)
    if gates is None:
        return False, ["simulator %s is not an implemented arm" % sim]
    required = ("expressible", "deliverable", "red_evidence",
                "discriminating", "publishable")
    reasons = []
    for name in required:
        state, why = gates[name]
        if state != readiness.OK:
            reasons.append("%s: %s" % (name, why or state))
    return not reasons, reasons


def _task_state(task_id, by_task, sim, readiness_fn):
    row = by_task.get(task_id)
    if row is None:
        return {
            "task": task_id, "outcome": "MISSING", "measured_pass": False,
            "claimable_pass": False, "publication_ready": False,
            "readiness_blockers": ["no selected row"],
        }
    measured = row.get("outcome") == "PASS"
    ready, blockers = readiness_fn(task_id, sim)
    return {
        "task": task_id,
        "outcome": row.get("outcome"),
        "measured_pass": measured,
        "claimable_pass": measured and ready,
        "publication_ready": ready,
        "readiness_blockers": blockers,
        "source": row.get("_source"),
    }


def _prefix(states, key):
    n = 0
    for state in states:
        if not state[key]:
            break
        n += 1
    return n


def build_report(by_task, *, sim, readiness_fn=publication_ready):
    tracks = []
    for track in TRACKS:
        states = [_task_state(t, by_task, sim, readiness_fn)
                  for t in track.tasks]
        measured = _prefix(states, "measured_pass")
        claimable = _prefix(states, "claimable_pass")
        tracks.append({
            "id": track.id,
            "title": track.title,
            "meaning": track.meaning,
            "levels": len(states),
            "measured_frontier": measured,
            "claimable_frontier": claimable,
            "measured_label": (states[measured - 1]["task"]
                               if measured else None),
            "claimable_label": (states[claimable - 1]["task"]
                                if claimable else None),
            "tasks": states,
        })

    selected = list(by_task.values())
    first = selected[0] if selected else {}
    return {
        "frontier": FRONTIER_ID,
        "sim": sim,
        "suite": first.get("suite"),
        "protocol": first.get("protocol"),
        "condition": first.get("condition"),
        "model": _model(first),
        "tracks": tracks,
        "measured_tasks_passed": sum(
            1 for t in TASKS if by_task.get(t, {}).get("outcome") == "PASS"),
        "tasks_total": len(TASKS),
        "claimable": all(t["claimable_frontier"] == t["levels"]
                         for t in tracks),
        "note": ("Only claimable_frontier is suitable for external claims; "
                 "measured_frontier includes exploratory rows."),
    }


def _print(report):
    print("Verified Build Frontier -- %s" % report["sim"])
    print("model=%s condition=%s suite=%s" % (
        report.get("model"), report.get("condition"), report.get("suite")))
    print()
    for track in report["tracks"]:
        print("%-20s measured %d/%d  claimable %d/%d" % (
            track["id"], track["measured_frontier"], track["levels"],
            track["claimable_frontier"], track["levels"]))
        for state in track["tasks"]:
            mark = "PASS" if state["measured_pass"] else state["outcome"]
            suffix = "" if state["publication_ready"] else " (exploratory)"
            print("  %-28s %-8s%s" % (state["task"], mark, suffix))
    print()
    print("FULL ENVELOPE CLAIMABLE: %s" % ("YES" if report["claimable"] else "NO"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("rows", nargs="+", help="one or more rows.jsonl files")
    ap.add_argument("--sim", required=True)
    ap.add_argument("--condition")
    ap.add_argument("--model")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    try:
        rows = load_rows(args.rows)
        selected = select_rows(rows, sim=args.sim,
                               condition=args.condition, model=args.model)
        report = build_report(selected, sim=args.sim)
    except FrontierError as exc:
        ap.error(str(exc))
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        _print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
