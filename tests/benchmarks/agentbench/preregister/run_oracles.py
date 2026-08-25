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

"""Run the Lane B scripted oracles through the REAL runner + graders, both
simulators, both conditions, and record the plan-2.1/2.2 verdicts.

    python tests/benchmarks/agentbench/preregister/run_oracles.py \\
        [--tasks B1,B2,...] [--sims omnisim,webots] [--skip-verdicts]

Per cell (task x sim x condition), SEQUENTIALLY (plan 5.3):

* **OmniSim cells** go through the shipped ``run_agentbench.py`` end to end
  (``--agent llm`` + the scripted backend), so the row, the phase-B
  standalone recorder pass, the grader and the condition attribution are the
  shipped ones. ``tool_calls`` is the runner Ledger's count.
* **Webots cells** run the same scripted agent through the same runner loop
  (``AGENTBENCH_SIM=webots`` selects the bridge tool set for shell+tools),
  then are graded by the same task graders with ``sim="webots"``: the
  Phase W launcher produces the artifact set, ``task_support`` adds the
  prober-measured AABBs (and, for B2, the live camera poses), and the
  sim-neutral cores render the verdict.

Cell records land in ``preregister/runs/cells.json``; the grader- and
runner-produced evidence a WSL-free test can re-check is copied under
``preregister/evidence/``; the three verdicts are computed by
``verdicts.py`` and written to ``preregister/oracle_verdicts.json`` (with a
human-readable ``.md`` beside it).

Engine-launch flake rule: the launcher and the OmniSim standalone adapter
each retry once on a launch that left no evidence; nothing here retries a
verdict.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
sys.path.insert(0, str(AGENTBENCH.parent))

from agentbench import tasks as task_registry            # noqa: E402
from agentbench.adapters.webots import launcher, task_support  # noqa: E402
from agentbench.agents.base import AgentContext          # noqa: E402
from agentbench.common import worldtext                  # noqa: E402
from agentbench.common.paths import REPO                 # noqa: E402
from agentbench.common.trace import Trace                # noqa: E402
from agentbench.runner.config import RunnerConfig        # noqa: E402
from agentbench.runner.loop import run_agent             # noqa: E402
from agentbench.runner.tools import normalize_condition  # noqa: E402
from agentbench.preregister import verdicts as verdicts_mod  # noqa: E402

SCRIPTS = AGENTBENCH / "runner" / "scripts"
RUNS = HERE / "runs"
EVIDENCE = HERE / "evidence"

TASK_IDS = verdicts_mod.TASKS
CONDITIONS = verdicts_mod.CONDITIONS

INITIAL_WEBOTS = "initial_webots"

# The B2 view channel needs the SHIPPED initial world measured too.
B2 = "B2_subject_in_frame"
C1 = "C1_parse_error_fix"

# Tasks whose graders consume world-space AABBs on the webots arm.
NEEDS_AABB = {"B1_overlap_audit", B2, "B3_measure_and_report",
              "C2_fall_through_floor"}


def script_for(task_id, sim):
    short = task_id.split("_")[0].lower()
    return SCRIPTS / ("oracle_%s_%s.json" % (short, sim))


def cond_key(condition):
    return condition.replace("+", "_")


# --- OmniSim cells (through the shipped harness) ----------------------------


def run_omnisim_cell(task_id, condition):
    out_dir = RUNS / "omnisim" / ("%s.%s" % (task_id, cond_key(condition)))
    if out_dir.exists():
        shutil.rmtree(out_dir, ignore_errors=True)
    env = dict(os.environ)
    env["AGENTBENCH_BACKEND"] = "scripted"
    env["AGENTBENCH_SCRIPT"] = str(script_for(task_id, "omnisim"))
    env.pop("AGENTBENCH_SIM", None)
    cmd = [sys.executable, str(AGENTBENCH / "run_agentbench.py"),
           "--tasks", task_id, "--agent", "llm",
           "--condition", condition, "--repeats", "1",
           "--out", str(out_dir)]
    t0 = time.time()
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace",
                          cwd=str(REPO), timeout=3600)
    rows_path = out_dir / "rows.jsonl"
    row = None
    if rows_path.is_file():
        lines = [ln for ln in rows_path.read_text(
            encoding="utf-8").splitlines() if ln.strip()]
        if lines:
            row = json.loads(lines[-1])
    cell = {
        "task": task_id, "sim": "omnisim", "condition": condition,
        "script": script_for(task_id, "omnisim").name,
        "outcome": (row or {}).get("outcome"),
        "tool_calls": ((row or {}).get("metrics") or {}).get("tool_calls"),
        "turns": ((row or {}).get("metrics") or {}).get("turns"),
        "stop_reason": (row or {}).get("stop_reason"),
        "failed_assertion": (row or {}).get("failed_assertion"),
        "assertions": (row or {}).get("assertions"),
        "self_verified": (row or {}).get("self_verified"),
        "run_dir": str(out_dir),
        "row_file": str(rows_path),
        "driver_wall_s": round(time.time() - t0, 1),
        "runner_rc": proc.returncode,
    }
    if row is None:
        cell["error"] = ("run_agentbench produced no row; stdout tail: %s"
                         % proc.stdout[-800:])
    return cell


# --- Webots cells (runner loop + Phase W launcher + neutral cores) ----------


def materialise_webots_scratch(task, scratch: Path):
    scratch.mkdir(parents=True, exist_ok=True)
    src = task.dir / INITIAL_WEBOTS
    written = []
    for p in sorted(src.rglob("*")):
        if p.is_dir():
            continue
        dst = scratch / p.relative_to(src)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dst)
        written.append(str(dst))
    return written


def run_webots_cell(task_id, condition):
    task = task_registry.get(task_id)
    run_dir = RUNS / "webots" / ("%s.%s" % (task_id, cond_key(condition)))
    if run_dir.exists():
        shutil.rmtree(run_dir, ignore_errors=True)
    scratch = run_dir / "scratch"
    scratch.mkdir(parents=True, exist_ok=True)
    initial = materialise_webots_scratch(task, scratch)

    t0 = time.time()
    trace = Trace(run_dir)
    prev_sim = os.environ.get("AGENTBENCH_SIM")
    os.environ["AGENTBENCH_SIM"] = "webots"
    try:
        ctx = AgentContext(task_id=task.id, prompt=task.prompt,
                           scratch_dir=scratch, run_dir=run_dir, repo=REPO,
                           trace=trace,
                           deadline_s=time.time() + (task.timeout_s or 0),
                           seed=0)
        cfg = RunnerConfig.from_env(
            condition=normalize_condition(condition),
            backend="scripted",
            script=str(script_for(task_id, "webots")))
        loop = run_agent(ctx, cfg)
    finally:
        trace.close()
        if prev_sim is None:
            os.environ.pop("AGENTBENCH_SIM", None)
        else:
            os.environ["AGENTBENCH_SIM"] = prev_sim
    t_agent = time.time() - t0

    cell = {
        "task": task_id, "sim": "webots", "condition": condition,
        "script": script_for(task_id, "webots").name,
        "tool_calls": loop.ledger.tool_calls,
        "turns": loop.ledger.turns,
        "stop_reason": loop.stop_reason,
        "tool_set": loop.tool_set,
        "tools_sha256": loop.tools_sha256,
        "manifest_sha256": loop.manifest_sha256,
        "self_verified": loop.self_verified,
        "initial_files": initial,
        "run_dir": str(run_dir),
        "t_agent_s": round(t_agent, 1),
    }
    if loop.stop_reason != "model_stopped":
        cell["outcome"] = "INVALID"
        cell["error"] = ("agent loop ended with %s (%s), not a clean stop"
                        % (loop.stop_reason, loop.stop_detail))
        return cell

    # ---- grade: launcher run + prober AABBs + the sim-neutral cores ------
    artifact = worldtext.pick_artifact(scratch)
    cell["artifact"] = str(artifact) if artifact else None
    sa = task.standalone
    _dir, facts = launcher.launch(
        artifact, run_dir / "webots",
        duration=float(sa.get("duration_s", 10.0)),
        contact_steps=int(sa.get("contact_steps", 1)),
        # Forward the link/solid track requests, or the WEBOTS arm stays blind
        # to arm links and named objects while OmniSim can see them -- an
        # instrument asymmetry that would read as a capability difference.
        links=int(sa.get("links", 0)),
        solids=tuple(sa.get("solids", ())))
    cell["launch"] = {k: facts.get(k) for k in
                      ("exit_code", "timed_out", "wall_s", "attempts_used")}

    aabbs_doc = None
    if task_id in NEEDS_AABB:
        aabbs_doc, pfacts = task_support.probe_world(
            artifact, run_dir / "aabb")
        cell["aabb_probe"] = {"ok": aabbs_doc is not None,
                              "exit_code": pfacts.get("exit_code")}
    run2, merged = task_support.augment_run(run_dir / "webots", aabbs_doc)
    cell["aabb_merged_bodies"] = merged

    kw = {"artifact": artifact, "scratch_dir": scratch, "sim": "webots",
          "run": run2, "self_verified": loop.self_verified}
    if task_id == B2:
        shipped = task.dir / INITIAL_WEBOTS / "frame_the_cylinder.wbt"
        init_doc, _f = task_support.probe_world(shipped,
                                               run_dir / "aabb_initial")
        kw["view"] = task_support.view_evidence(init_doc, aabbs_doc,
                                                artifact=artifact)
    grader = task.grader()
    if task_id in (C1,):
        v = grader.grade(run_dir, **kw)
    else:
        v = grader.grade(run_dir, answer=loop.final_message, **kw)
    vd = v.as_dict()
    (run_dir / "verdict.json").write_text(
        json.dumps(vd, indent=2, default=str), encoding="utf-8")
    cell["outcome"] = vd["outcome"]
    cell["failed_assertion"] = vd.get("failed_assertion")
    cell["assertions"] = {k: a["ok"] for k, a in vd["assertions"].items()}
    cell["verdict_file"] = str(run_dir / "verdict.json")
    print(v.summary())
    return cell


# --- evidence copies (what a WSL-free test can re-check) --------------------

_WEBOTS_EVIDENCE = ("roster.json", "trajectory.json", "contacts.json",
                    "completion.json", "process.json", "stdout.log",
                    "stderr.log", "version.txt")


def copy_evidence(cell):
    tag = "%s.%s" % (cell["task"], cond_key(cell["condition"]))
    dest = EVIDENCE / cell["sim"] / tag
    if dest.exists():
        shutil.rmtree(dest, ignore_errors=True)
    dest.mkdir(parents=True)
    run_dir = Path(cell["run_dir"])
    if cell["sim"] == "webots":
        (dest / "webots").mkdir()
        for name in _WEBOTS_EVIDENCE:
            src = run_dir / "webots" / name
            if src.is_file():
                shutil.copy2(src, dest / "webots" / name)
        for probe in ("aabb", "aabb_initial"):
            src = run_dir / probe / "aabbs.json"
            if src.is_file():
                (dest / probe).mkdir(exist_ok=True)
                shutil.copy2(src, dest / probe / "aabbs.json")
        for name in ("verdict.json", "runner_result.json"):
            src = run_dir / name
            if src.is_file():
                shutil.copy2(src, dest / name)
        art = cell.get("artifact")
        if art and Path(art).is_file():
            shutil.copy2(art, dest / Path(art).name)
        # The agent's final message, for answer-driven graders.
        (dest / "final_message.txt").write_text(
            _final_message(run_dir), encoding="utf-8")
    else:
        for name in ("rows.jsonl",):
            src = run_dir / name
            if src.is_file():
                shutil.copy2(src, dest / name)
        for sub in run_dir.glob("*/verdict.json"):
            shutil.copy2(sub, dest / "verdict.json")
    cell["evidence_dir"] = str(dest)


def _final_message(run_dir):
    try:
        doc = json.loads((run_dir / "runner_result.json").read_text(
            encoding="utf-8"))
        return doc.get("final_message") or ""
    except (OSError, ValueError):
        return ""


# --- verdict rendering ------------------------------------------------------


def render_md(doc):
    lines = [
        "# Lane B oracle verdicts (pre-freeze instrument)",
        "",
        "Generated by `preregister/run_oracles.py`; arithmetic in "
        "`preregister/verdicts.py`; measured cells in "
        "`preregister/runs/cells.json` (evidence copies under "
        "`preregister/evidence/`). Regenerate with:",
        "",
        "    python tests/benchmarks/agentbench/preregister/run_oracles.py "
        "--verdicts-only",
        "",
        "Measured: %s (machine %s)" % (doc["meta"]["utc"],
                                       doc["meta"].get("machine")),
        "",
        "## Measured oracle cells (tool_calls by the runner Ledger)",
        "",
        "| task | sim | condition | outcome | tool_calls | turns |",
        "|---|---|---|---|---|---|",
    ]
    for c in doc["cells"]:
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            c["task"], c["sim"], c["condition"], c.get("outcome"),
            c.get("tool_calls"), c.get("turns")))
    v = doc["verdicts"]
    lines += ["", "## 1. Lane test (plan 2.1)", "",
              "| task | omnisim best | webots best | ratio | membership |",
              "|---|---|---|---|---|"]
    for task, r in v["lane_test"].items():
        lines.append("| %s | %s (%s calls, %s) | %s (%s calls, %s) | %s | "
                     "**%s** |" % (
                         task, r["omnisim_best"]["condition"],
                         r["omnisim_best"]["tool_calls"],
                         r["omnisim_best"]["outcome"],
                         r["webots_best"]["condition"],
                         r["webots_best"]["tool_calls"],
                         r["webots_best"]["outcome"],
                         r["ratio_webots_over_omnisim"],
                         r["lane_b_membership"]))
    lines += ["", "## 2. Granularity guard (plan 2.2 guard 1)", "",
              "| task | webots shell | webots bridge | verdict |",
              "|---|---|---|---|"]
    for task, r in v["granularity_guard"].items():
        lines.append("| %s | %s | %s | %s |" % (
            task, r["webots_shell_tool_calls"],
            r["webots_bridge_tool_calls"], r["verdict"]))
    d = v["distinctness"]
    lines += ["", "## 3. Distinctness (plan 2.2 guard 2)", "",
              "Verdict: **%s** -- %s" % (d["verdict"], d["status"]), ""]
    for task, r in d["per_task"].items():
        lines.append("* %s: saving %s (qualifies: %s)"
                     % (task, r["saving_fraction"], r["qualifies"]))
    lines += ["", "Consequence if NOT DISTINCT: %s"
              % d["consequence_if_not_distinct"], ""]
    for note in doc.get("notes", []):
        lines.append("> %s" % note)
        lines.append("")
    return "\n".join(lines) + "\n"


NOTES = [
    "Broken-world verification, measured on the WSL2 upstream install "
    "(BRINGUP.md): the C1 initial_webots world FAILS to load under upstream "
    "R2025a exactly as the task requires -- 2 ERROR lines ('92:1: error: "
    "Expected field name or }, found DEF' then 'Failed to load due to "
    "syntax error(s)'), after which upstream idles with no world until "
    "killed (rc=124 by timeout, no roster, recorder never starts, 2/2 "
    "attempts identical). Upstream's parser aborts at the unbalanced brace, "
    "so the 'Soild' defect is masked until the brace is fixed -- a "
    "different behaviour from OmniSim's engine, which SKIPS the unknown "
    "'Soild' node with an error line and loads the rest (measured "
    "2026-07-31, c1_core docstring). The C2 initial_webots world loads "
    "cleanly and its crate free-falls through the hologram floor to "
    "z = -121.6 m in 5 s (launcher-measured trajectory), matching the "
    "OmniSim arm's symptom.",
    "Every cell above was produced by replaying the committed oracle script "
    "through the real runner loop (real tool sets, real Ledger) and grading "
    "the produced artifacts with the real graders on real simulator runs; "
    "cells.json cites each cell's run_dir and evidence copy.",
    "The per-simulator oracle script is REPLAYED UNDER BOTH conditions "
    "because the minimal competent path for all five tasks is file-level "
    "(reads and writes of the scene, of the robot model's own geometry "
    "sources, and of the installed simulator's assets, plus shell-run "
    "verification) and the shell tool set is a subset of every shell+tools "
    "set; each script's `minimality` block enumerates the bridge/harness-"
    "idiomatic alternative paths and their strictly-larger call counts "
    "(arithmetic over the same tool surfaces, not measured runs). "
    "Consequence: no bridge verb sits on any oracle's minimal path, the "
    "granularity guard holds by construction, and the distinctness verdict "
    "is NOT DISTINCT unless a reviewer produces a cheaper bridge-idiomatic "
    "path.",
    "B1 fixture disclosure -- RESOLVED 2026-08-01, pre-freeze: both B1 "
    "initial worlds' headers used to name the overlapping pair (and their "
    "titles said one existed), so the one-call oracle priced file-reading, "
    "not measurement. The disclosure was stripped from both arms (comments "
    "and titles only; every pose is untouched, so the committed grader "
    "ground truth is unchanged), and the B1 oracles were regenerated to "
    "MEASURE: read the scene for poses, read the robot model's own "
    "geometry source for the footprint (the Husky URDF; the cached "
    "Pioneer3at PROTO's boundingObject boxes), compute the pairwise "
    "clearances -- 2 calls per arm, lane ratio unchanged at 1.0.",
    "Oracle cells are n=1 by design (the plan's lane test and bridge "
    "guards are defined over the scripted oracle, a deterministic "
    "instrument); they are pre-freeze instrument readings, not campaign "
    "scores, and are quotable only as such.",
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tasks", default=",".join(TASK_IDS))
    ap.add_argument("--sims", default="omnisim,webots")
    ap.add_argument("--conditions", default=",".join(CONDITIONS))
    ap.add_argument("--verdicts-only", action="store_true",
                    help="recompute verdicts from the recorded cells.json "
                         "without running anything")
    args = ap.parse_args(argv)

    cells_path = RUNS / "cells.json"
    if args.verdicts_only:
        cells = json.loads(cells_path.read_text(encoding="utf-8"))["cells"]
        _write_verdicts(cells)
        return 0

    tasks = [t.strip() for t in args.tasks.split(",") if t.strip()]
    sims = [s.strip() for s in args.sims.split(",") if s.strip()]
    conditions = [c.strip() for c in args.conditions.split(",") if c.strip()]

    RUNS.mkdir(parents=True, exist_ok=True)
    cells = []
    if cells_path.is_file():
        cells = json.loads(cells_path.read_text(
            encoding="utf-8")).get("cells", [])

    for task_id in tasks:
        for sim in sims:
            for condition in conditions:
                print("\n=== oracle cell: %s / %s / %s ==="
                      % (task_id, sim, condition))
                if sim == "omnisim":
                    cell = run_omnisim_cell(task_id, condition)
                else:
                    cell = run_webots_cell(task_id, condition)
                copy_evidence(cell)
                cells = [c for c in cells
                         if not (c["task"] == task_id and c["sim"] == sim
                                 and c["condition"] == condition)]
                cells.append(cell)
                print("  -> outcome=%s tool_calls=%s"
                      % (cell.get("outcome"), cell.get("tool_calls")))
                _write_cells(cells)
    _write_verdicts(cells)
    return 0


def _order(c):
    return (c["task"], c["sim"], c["condition"])


def _write_cells(cells):
    cells = sorted(cells, key=_order)
    (RUNS / "cells.json").write_text(
        json.dumps({"cells": cells}, indent=2), encoding="utf-8")


def _write_verdicts(cells):
    cells = sorted(cells, key=_order)
    try:
        from agentbench.common.paths import ob_results
        fp = ob_results.machine_fingerprint() or {}
        machine = ((fp.get("machine") or {}).get("id"),
                   fp.get("build"))
        machine = {"id": machine[0], "engine_build": machine[1]}
    except Exception:                                     # noqa: BLE001
        machine = None
    doc = {
        "meta": {
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "machine": machine,
            "generator": "preregister/run_oracles.py",
            "n_per_cell": 1,
            "status": "pre-freeze oracle instrument (plan 2.1/2.2); "
                      "NOT campaign scores",
        },
        "cells": [{k: c.get(k) for k in
                   ("task", "sim", "condition", "script", "outcome",
                    "tool_calls", "turns", "stop_reason", "failed_assertion",
                    "run_dir", "evidence_dir")} for c in cells],
        "verdicts": verdicts_mod.compute(cells),
        "notes": NOTES,
    }
    (HERE / "oracle_verdicts.json").write_text(
        json.dumps(doc, indent=2), encoding="utf-8")
    (HERE / "oracle_verdicts.md").write_text(render_md(doc),
                                             encoding="utf-8")
    print("\nverdicts -> %s" % (HERE / "oracle_verdicts.json"))
    for task, r in doc["verdicts"]["lane_test"].items():
        print("  lane %s: %s (ratio %s)"
              % (task, r["lane_b_membership"],
                 r["ratio_webots_over_omnisim"]))
    d = doc["verdicts"]["distinctness"]
    print("  distinctness: %s" % d["verdict"])


if __name__ == "__main__":
    sys.exit(main())
