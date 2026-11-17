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

"""Re-adjudicate cc-lane cells poisoned by a leaked foreign harness.

The defect (measured 2026-08-01, campaign ``phasew_cc_v1``): a benchmark
agent in one cell launched ``omnisim_harness.py --port 6789`` anchored at the
REAL repo (``cd /o/omnisim``), escaping the workspace-scoped process sweep.
The harness outlived its cell. Every subsequent agent that probed the
canonical port -- exactly what the staged AGENTS.md teaches -- found a live
harness serving a FOREIGN world, took it for its task world, and did all its
work there. The graded artifact fell back to the staged world unchanged and
the cell recorded a FAIL that measured the contamination, not the agent.

This tool rewrites such rows to INVALID (SPEC 3.3: a broken instrument is
never a PASS and never a FAIL) **from evidence, never from assertion**. A row
is re-adjudicated only when BOTH witnesses hold:

1. **foreign-world witness** -- the session transcript contains a harness
   state/scene response naming a world under the real repo's ``projects/``
   tree, a path no hermetic cell can produce (workspace worlds live under the
   instance dir);
2. **untouched-deliverable witness** -- the collected artifact is
   byte-identical to the task's staged initial world, i.e. the agent's work
   went somewhere the grader could not see.

A PASS row is NEVER rewritten (a correct deliverable is real regardless of
what else the session talked to), and a FAIL row whose artifact differs from
the staged world stands (the agent really did edit the world; that verdict
was earned). Original outcomes are preserved verbatim inside the row's
``grading_recovery`` block and per-cell ``grading_recovery.json`` files.

Usage::

    python readjudicate.py --campaign-dir .../campaigns/phasew_cc_v1 \\
        --task B2_subject_in_frame --sim omnisim [--repeats 0,1,2] \\
        [--context-note "..."] --apply     # without --apply: dry run
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
REPO = AGENTBENCH.parents[2]

#: A harness/scene payload naming a world under the REAL repo's projects
#: tree. Backslash multiplicity in a transcript is escaping-dependent, so any
#: run of slashes/backslashes matches. Bound to "world" plus a drive-rooted
#: (or /o/-rooted) omnisim/projects path so a mere mention of the repo in
#: prose cannot trip it.
_FOREIGN_WORLD_RE = re.compile(
    r'world[^a-zA-Z0-9]{1,10}'
    r'(?:[A-Za-z]:[\\/]+|/o/)omnisim[\\/]+projects\b',
    re.IGNORECASE)


def foreign_world_witness(transcript_path):
    """The transcript lines proving the session talked to a harness serving
    a real-repo world. Returns ``[excerpt, ...]`` (empty = no witness)."""
    p = Path(transcript_path)
    if not p.is_file():
        return []
    hits = []
    with open(p, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = _FOREIGN_WORLD_RE.search(line)
            if m:
                lo = max(0, m.start() - 40)
                hits.append(line[lo:m.end() + 80].strip())
    return hits


#: A session LAUNCHING a harness itself (as opposed to finding one leaked).
#: A foreign world reached through a harness the agent started is the
#: agent's own workspace escape -- a real capability failure, never
#: re-adjudicated.
_SELF_LAUNCH_RE = re.compile(r"omnisim_harness\.py", re.IGNORECASE)


def self_launched_harness(transcript_path):
    """True when the session's own tool calls launched a harness."""
    p = Path(transcript_path)
    if not p.is_file():
        return False
    with open(p, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if _SELF_LAUNCH_RE.search(line) and '"tool_use"' in line:
                return True
    return False


def untouched_deliverable_witness(artifact_path, task_id, *,
                                  tasks_dir=AGENTBENCH / "tasks"):
    """True when the collected artifact is byte-identical to the task's
    staged initial world (None when either file is unreadable)."""
    art = Path(artifact_path)
    if not art.is_file():
        return None
    initial = Path(tasks_dir) / task_id / "initial" / art.name
    if not initial.is_file():
        return None
    try:
        return art.read_bytes() == initial.read_bytes()
    except OSError:
        return None


def cell_dir_for(campaign_dir, task_id, sim, repeat):
    return (Path(campaign_dir) / "cells"
            / ("%s_%s_r%d" % (task_id, sim, repeat)))


def adjudicate_cell(campaign_dir, task_id, sim, repeat, *,
                    tasks_dir=AGENTBENCH / "tasks", answer_task=False):
    """The witnesses for one cell. Returns a dict with the evidence and the
    ruling (``invalidate`` True/False + ``reason``).

    World tasks (default): invalidate only when the foreign-world witness
    holds AND the graded artifact is byte-identical to the staged initial
    world. Answer tasks (``answer_task=True`` -- B1/B3, where the
    deliverable is the agent's measured answer and there is no world
    artifact): invalidate when the foreign-world witness holds AND the
    session did NOT launch the harness itself -- an answer derived from a
    scene a previous cell leaked measures the contamination, but a session
    that escaped its own workspace earned its verdict."""
    cdir = cell_dir_for(campaign_dir, task_id, sim, repeat)
    transcript = cdir / "transcript.jsonl"
    hits = foreign_world_witness(transcript)
    self_launched = self_launched_harness(transcript)
    artifact = None
    if (cdir / "artifact").is_dir():
        for cand in sorted([*(cdir / "artifact").glob("*.omniworld"), *(cdir / "artifact").glob("*.wbt")]):
            artifact = cand
            break
    untouched = (untouched_deliverable_witness(artifact, task_id,
                                               tasks_dir=tasks_dir)
                 if artifact is not None else None)
    if answer_task:
        invalidate = bool(hits) and not self_launched
        if invalidate:
            reason = ("the session's answer was measured against a foreign "
                      "harness world it FOUND, not launched (%d transcript "
                      "witness(es), no self-launch): the verdict measured "
                      "cross-cell contamination, not the agent" % len(hits))
        elif hits:
            reason = ("the session reached a foreign world through a "
                      "harness it launched ITSELF -- a workspace escape is "
                      "the agent's own failure and the verdict stands")
        else:
            reason = "no foreign-world witness in the transcript"
    else:
        invalidate = bool(hits) and untouched is True
        if invalidate:
            reason = ("the session provably worked against a foreign "
                      "harness world (%d transcript witness(es)) and the "
                      "graded artifact is byte-identical to the staged "
                      "initial world: the verdict measured cross-cell "
                      "contamination, not the agent" % len(hits))
        elif hits:
            reason = ("foreign-harness contact seen in the transcript, but "
                      "the artifact differs from the staged world -- the "
                      "deliverable is the agent's own work and the verdict "
                      "stands")
        else:
            reason = "no foreign-world witness in the transcript"
    return {"cell_dir": str(cdir), "transcript": str(transcript),
            "artifact": str(artifact) if artifact else None,
            "foreign_world_witnesses": hits[:5],
            "n_witnesses": len(hits),
            "self_launched_harness": self_launched,
            "artifact_untouched": untouched,
            "invalidate": invalidate, "reason": reason}


def rewrite_rows(campaign_dir, task_id, sim, *, repeats=None,
                 context_note=None, apply=False, out=print,
                 tasks_dir=AGENTBENCH / "tasks", answer_task=False):
    """Re-adjudicate the group's rows. Returns the per-repeat rulings.

    Rewrites ``groups/<task>_<sim>/rows.jsonl`` in place (whole-file, single
    write) and drops a ``grading_recovery.json`` next to each rewritten
    cell's report. Dry-run unless ``apply``.
    """
    campaign_dir = Path(campaign_dir)
    group = campaign_dir / "groups" / ("%s_%s" % (task_id, sim))
    rows_path = group / "rows.jsonl"
    if not rows_path.is_file():
        raise FileNotFoundError(rows_path)
    utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    rulings = {}
    out_lines = []
    changed = 0
    for line in rows_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        rep = row.get("repeat")
        if (row.get("task") != task_id or row.get("sim") != sim
                or (repeats is not None and rep not in repeats)):
            out_lines.append(line)
            continue
        ruling = adjudicate_cell(campaign_dir, task_id, sim, rep,
                                 tasks_dir=tasks_dir,
                                 answer_task=answer_task)
        rulings[rep] = ruling
        original = row.get("outcome")
        if not ruling["invalidate"] or original == "PASS":
            if original == "PASS" and ruling["invalidate"]:
                ruling["invalidate"] = False
                ruling["reason"] += (" -- but the row is a PASS, which is "
                                     "never rewritten")
            out_lines.append(line)
            continue
        recovery = {
            "utc": utc,
            "kind": "contaminated_environment",
            "original_outcome": original,
            "revised_outcome": "INVALID",
            "reason": ruling["reason"],
            "evidence": {
                "foreign_world_witnesses":
                    ruling["foreign_world_witnesses"],
                "n_witnesses": ruling["n_witnesses"],
                "artifact_untouched": ruling["artifact_untouched"],
                "artifact": ruling["artifact"],
                "transcript": ruling["transcript"],
            },
        }
        if context_note:
            recovery["context"] = context_note
        row["grading_recovery"] = recovery
        row["outcome"] = "INVALID"
        row.setdefault("notes", []).append(
            "grading_recovery (%s): outcome rewritten %s -> INVALID; %s. "
            "SPEC 3.3: a contaminated environment measures nothing about "
            "the agent -- never a FAIL, never a PASS."
            % (utc, original, ruling["reason"]))
        out_lines.append(json.dumps(row, default=str))
        changed += 1
        if apply:
            (Path(ruling["cell_dir"]) / "grading_recovery.json").write_text(
                json.dumps(recovery, indent=2), encoding="utf-8")
        out("r%s: %s -> INVALID (%s)" % (rep, original, ruling["reason"]))
    for rep, ruling in rulings.items():
        if not ruling["invalidate"]:
            out("r%s: unchanged (%s)" % (rep, ruling["reason"]))
    if apply and changed:
        rows_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
        out("rewrote %s (%d row(s) revised)" % (rows_path, changed))
    elif changed:
        out("dry run: %d row(s) WOULD be revised (re-run with --apply)"
            % changed)
    else:
        out("nothing to revise")
    return rulings


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--campaign-dir", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("--sim", required=True, choices=("omnisim", "webots"))
    ap.add_argument("--repeats", default=None,
                    help="comma-separated repeat indices (default: all)")
    ap.add_argument("--context-note", default=None,
                    help="forensic context recorded verbatim in "
                         "grading_recovery.context")
    ap.add_argument("--apply", action="store_true",
                    help="write the revised rows (default: dry run)")
    ap.add_argument("--answer-task", action="store_true",
                    help="the task's deliverable is the agent's ANSWER "
                         "(B1/B3): invalidate on a found -- never "
                         "self-launched -- foreign world, without the "
                         "artifact-untouched witness")
    args = ap.parse_args(argv)
    repeats = (None if args.repeats is None
               else {int(r) for r in args.repeats.split(",") if r.strip()})
    rewrite_rows(Path(args.campaign_dir), args.task, args.sim,
                 repeats=repeats, context_note=args.context_note,
                 apply=args.apply, answer_task=args.answer_task)
    return 0


if __name__ == "__main__":
    sys.exit(main())
