"""Run a resumable, one-cell-per-task Codex Verified Build Frontier campaign."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCHMARKS = HERE.parents[1]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench import frontier, sims, tasks  # noqa: E402
from agentbench.codex_lane import run_codex_task  # noqa: E402


def parse_csv(value):
    return tuple(v.strip() for v in value.split(",") if v.strip())


def make_plan(sim_ids, task_ids):
    """Return comparable cells and explicit fixture gaps in stable order."""
    cells, omitted = [], []
    for sim_id in sim_ids:
        sim = sims.get(sim_id)
        for task_id in task_ids:
            tasks.get(task_id)  # fail early on a misspelling
            if sim.expresses(task_id):
                cells.append((sim_id, task_id))
            else:
                omitted.append({
                    "sim": sim_id, "task": task_id,
                    "reason": "fixture/deliverable arm is not implemented",
                })
    return cells, omitted


def _read_row(path):
    lines = [line for line in Path(path).read_text(
        encoding="utf-8").splitlines() if line.strip()]
    if len(lines) != 1:
        raise RuntimeError("%s must contain exactly one row" % path)
    return json.loads(lines[0])


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=str) + "\n",
                          encoding="utf-8")


def run_campaign(*, sim_ids, task_ids, model, root, out_dir, codex=None,
                 layout_seed=20260813):
    if not model:
        raise ValueError("campaign model must be pinned")
    root = Path(root).resolve()
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    executable = codex or run_codex_task.shutil.which("codex")
    if not executable:
        raise RuntimeError("codex executable not found")
    identity = run_codex_task.codex_identity(executable)
    cells, omitted = make_plan(sim_ids, task_ids)
    config = {
        "schema": "agenticsimbench/codex-campaign/v1",
        "frontier": frontier.FRONTIER_ID,
        "suite": run_codex_task.shared.SUITE,
        "condition": run_codex_task.CONDITION,
        "model": model,
        "codex": identity,
        "sims": list(sim_ids),
        "tasks": list(task_ids),
        "layout_seed": layout_seed,
        "cells": [{"sim": s, "task": t} for s, t in cells],
        "omitted": omitted,
    }
    config_path = out_dir / "campaign.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if previous != config:
            raise RuntimeError(
                "campaign configuration differs from the existing record; "
                "use a new --out directory")
    else:
        _write_json(config_path, config)

    rows = []
    for sim_id, task_id in cells:
        cell_dir = out_dir / "cells" / sim_id / task_id
        row_path = cell_dir / "rows.jsonl"
        if row_path.is_file():
            row = _read_row(row_path)
        else:
            if cell_dir.exists():
                raise RuntimeError(
                    "incomplete cell exists at %s; preserve it for audit and "
                    "start a new campaign directory" % cell_dir)
            row = run_codex_task.run_task(
                sim=sim_id, task_id=task_id, root=root, out_dir=cell_dir,
                model=model, codex=identity["path"], layout_seed=layout_seed)
        rows.append(row)
        (out_dir / "rows.jsonl").write_text(
            "".join(json.dumps(r, default=str) + "\n" for r in rows),
            encoding="utf-8")

    reports = {}
    for sim_id in sim_ids:
        selected = frontier.select_rows(
            rows, sim=sim_id, condition=run_codex_task.CONDITION, model=model)
        report = frontier.build_report(selected, sim=sim_id)
        reports[sim_id] = report
        _write_json(out_dir / ("frontier_%s.json" % sim_id), report)
    summary = {"config": config, "rows": len(rows), "frontiers": reports}
    _write_json(out_dir / "summary.json", summary)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sims", default="omnisim,webots",
                    help="comma-separated simulator ids")
    ap.add_argument("--tasks", default="frontier",
                    help="'frontier' or comma-separated task ids")
    ap.add_argument("--model", required=True)
    ap.add_argument("--root", required=True,
                    help="disposable staging root outside the repository")
    ap.add_argument("--out", required=True,
                    help="new or resumable campaign evidence directory")
    ap.add_argument("--codex", help="Codex executable (default: PATH)")
    ap.add_argument("--layout-seed", type=int, default=20260813)
    args = ap.parse_args(argv)
    sim_ids = parse_csv(args.sims)
    task_ids = frontier.TASKS if args.tasks == "frontier" else parse_csv(
        args.tasks)
    result = run_campaign(
        sim_ids=sim_ids, task_ids=task_ids, model=args.model,
        root=args.root, out_dir=args.out, codex=args.codex,
        layout_seed=args.layout_seed)
    for sim_id, report in result["frontiers"].items():
        print("%s: %d/%d measured tasks; full claimable=%s" % (
            sim_id, report["measured_tasks_passed"], report["tasks_total"],
            "yes" if report["claimable"] else "no"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
