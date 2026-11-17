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

"""Where has a running cell got to? Read-only, safe to run mid-cell.

    python tests/benchmarks/ladder/cell/cell_progress.py
    python tests/benchmarks/ladder/cell/cell_progress.py --watch

⚠ **THERE IS NO "PERCENT OF THE TASK DONE" HERE, AND THERE WILL NOT BE.**

An open-ended build has no denominator: nobody -- not the agent, not us --
knows how many steps it will take until it is finished. Any percentage of
"task complete" would be invented, and a number that was invented rather than
measured is precisely the defect class this benchmark keeps tripping over
(a tool returning the value it was asked for instead of the value it measured).

So the three things below are each honestly derived, and each says what it is:

**Budget** is exact. Elapsed against the cap the task declared. It is a
percentage of the BUDGET, never of the work.

**Effort** is a real count of turns and tool calls, shown beside what previous
COMPLETED runs of the same task on the same column actually used. That gives a
sense of "roughly halfway through the usual amount of work" without claiming
to know this run's total -- and a run legitimately doing something harder will
blow past the median, which is information rather than an error.

**Milestones** are facts on disk: does a scene exist yet, a controller, has the
engine actually run, is there a deliverable. They are the closest thing to
real progress that can be checked rather than guessed, and they are what a
human watching over the agent's shoulder would look at.

Plus a staleness line, because "no file has changed and no tool has been called
for eleven minutes" is the single most useful thing to know about a long cell
and is invisible from a wall-clock alone.
"""

from __future__ import annotations

import argparse
import glob
import json
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve()
BENCH = HERE.parents[3]
RESULTS = HERE.parents[1] / "results" / "ladder_cell"

#: Milestones, in the order a cell reaches them. Each is a predicate over the
#: workspace, so each is a FACT rather than a guess about intent.
MILESTONES = (
    ("workspace staged", lambda ws, rd: (ws / "container").is_dir()),
    ("session started", lambda ws, rd: _transcript_for(ws) is not None),
    ("scene authored", lambda ws, rd: _any(ws, ("*.wbt", "*.xml", "*.mjcf"))),
    ("controller authored", lambda ws, rd: _any(ws, ("*.py",), under="controllers")
     or _any(ws, ("drive.py", "*_drive.py", "*control*.py"))),
    ("engine has run", lambda ws, rd: _any(ws, ("*.log", "*log.txt"))
     or (ws / "omnisim_log.txt").is_file()),
    ("deliverable collected", lambda ws, rd: (rd / "deliverable").exists()),
    ("graded", lambda ws, rd: (rd / "verdict.json").is_file()),
    ("row written", lambda ws, rd: (rd / "rows.jsonl").is_file()),
)

_SKIP_DIRS = {"msys64", ".git", "__pycache__", "node_modules", "docs", "src",
              "lib", "resources", "projects", "tests", "scripts", "include",
              "dependencies", "distribution", "omnisim", "agents", "Contents"}


def _any(ws: Path, patterns, under: str | None = None) -> bool:
    """Did the AGENT author a file like this? Staged trees are skipped."""
    root = ws / under if under else ws
    if not root.is_dir():
        root = ws
    for pat in patterns:
        for p in root.rglob(pat):
            parts = set(p.relative_to(ws).parts[:-1])
            if parts & _SKIP_DIRS:
                continue
            return True
    return False


def _transcript_for(ws: Path):
    slug = "".join(c if c.isalnum() else "-" for c in str(ws.resolve()))
    hits = sorted(Path.home().joinpath(".claude", "projects", slug).glob("*.jsonl")
                  if Path.home().joinpath(".claude", "projects", slug).is_dir() else [],
                  key=lambda p: p.stat().st_mtime, reverse=True)
    return hits[0] if hits else None


def _read_transcript(path):
    turns = tools = 0
    last_tool = last_text = ""
    last_ts = None
    if path is None or not path.is_file():
        return turns, tools, last_tool, last_text, last_ts
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        ts = rec.get("timestamp")
        if ts:
            last_ts = ts
        msg = rec.get("message") or {}
        if msg.get("role") == "assistant":
            turns += 1
        content = msg.get("content")
        if not isinstance(content, list):
            continue
        for blk in content:
            if not isinstance(blk, dict):
                continue
            if blk.get("type") == "tool_use":
                tools += 1
                inp = blk.get("input") or {}
                d = (inp.get("command") or inp.get("file_path")
                     or inp.get("pattern") or inp.get("description") or "")
                last_tool = "%s %s" % (blk.get("name"), str(d)[:90].replace("\n", " "))
            elif blk.get("type") == "text" and blk.get("text"):
                last_text = str(blk["text"])[-160:].replace("\n", " ")
    return turns, tools, last_tool, last_text, last_ts


def historical(task_id, column):
    """(median_minutes, median_tools, n) over COMPLETED rows for this cell."""
    mins, tls = [], []
    for rows in RESULTS.glob("*/rows.jsonl"):
        try:
            for line in rows.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                r = json.loads(line)
                if r.get("task") != task_id or r.get("sim") != column:
                    continue
                if (r.get("cell") or {}).get("value") != "achieved":
                    continue
                m = (r.get("metrics") or {})
                if m.get("t_total_s"):
                    mins.append(float(m["t_total_s"]) / 60.0)
                if m.get("tool_calls"):
                    tls.append(int(m["tool_calls"]))
        except (OSError, ValueError):
            continue

    def med(xs):
        return sorted(xs)[len(xs) // 2] if xs else None
    return med(mins), med(tls), len(mins)


def newest_cell():
    dirs = sorted((d for d in RESULTS.glob("*") if d.is_dir()),
                  key=lambda d: d.stat().st_mtime, reverse=True)
    return dirs[0] if dirs else None


def workspace_for(run_dir: Path):
    rep = run_dir / "cell_report.json"
    if rep.is_file():
        try:
            ws = json.loads(rep.read_text(encoding="utf-8")).get("workspace")
            if ws:
                return Path(ws)
        except (OSError, ValueError):
            pass
    # No report yet -- it is written at the END of a cell -- so find the
    # instance the runner made for this run. Instances are created within a few
    # seconds of the run dir, so match on the closest CREATION TIME rather than
    # on a name prefix: several cells a day share a date stamp and the prefix
    # is ambiguous between them.
    root = Path.home() / "AppData" / "Local" / "Temp" / "ladder_cell" / "instances"
    if not root.is_dir():
        return None
    cands = [d for d in root.iterdir() if d.is_dir()]
    if not cands:
        return None
    target = run_dir.stat().st_ctime
    best = min(cands, key=lambda d: abs(d.stat().st_ctime - target))
    # ...and only if it really belongs to this run. A leftover instance from a
    # killed cell must never be reported as this one's progress.
    return best if abs(best.stat().st_ctime - target) < 300 else None


def report(run_dir: Path) -> str:
    rd = run_dir
    ws = workspace_for(rd)
    done = (rd / "rows.jsonl").is_file()

    task_id = column = "?"
    cap_s = None
    rep = rd / "cell_report.json"
    if rep.is_file():
        try:
            j = json.loads(rep.read_text(encoding="utf-8"))
            task_id = j.get("task") or task_id
            column = j.get("column") or column
            cap_s = (j.get("session_cap") or {}).get("cap_s")
        except (OSError, ValueError):
            pass
    if task_id == "?":
        name = rd.name
        for col in ("omnisim", "mujoco", "webots"):
            if col in name:
                column = col
                task_id = name.split(col + "_")[-1]
    cap_s = cap_s or 5400.0

    started = datetime.fromtimestamp(rd.stat().st_ctime)
    elapsed = time.time() - rd.stat().st_ctime

    tx = _transcript_for(ws) if ws else None
    turns, tools, last_tool, last_text, last_ts = _read_transcript(tx)

    med_min, med_tools, n_hist = historical(task_id, column)

    out = []
    out.append("cell      : %s / %s%s" % (task_id, column,
                                          "   [FINISHED]" if done else ""))
    out.append("run dir   : %s" % rd.name)
    out.append("started   : %s" % started.strftime("%H:%M:%S"))

    pct = 100.0 * elapsed / cap_s
    bar_n = int(min(pct, 100) / 5)
    out.append("budget    : %dm %02ds of %dm cap  [%s%s] %.0f%% OF THE BUDGET "
               "(not of the task)"
               % (elapsed // 60, elapsed % 60, cap_s // 60,
                  "#" * bar_n, "." * (20 - bar_n), pct))

    if med_tools:
        out.append("effort    : %d turns, %d tool calls   (past ACHIEVED runs "
                   "of this cell used a median of %d tool calls / %.0f min, n=%d)"
                   % (turns, tools, med_tools, med_min or 0, n_hist))
    else:
        out.append("effort    : %d turns, %d tool calls   (no completed run of "
                   "this cell to compare against yet)" % (turns, tools))

    out.append("milestones:")
    for label, pred in MILESTONES:
        try:
            ok = bool(ws and pred(ws, rd))
        except (OSError, ValueError):
            ok = False
        out.append("   [%s] %s" % ("x" if ok else " ", label))

    if last_ts:
        try:
            t = datetime.fromisoformat(last_ts.replace("Z", "+00:00"))
            quiet = (datetime.now(timezone.utc) - t).total_seconds()
            flag = "  <-- QUIET, check it is not wedged" if quiet > 600 else ""
            out.append("last act  : %.0fs ago%s" % (quiet, flag))
        except ValueError:
            pass
    if last_tool:
        out.append("doing     : %s" % last_tool)
    if last_text:
        out.append("saying    : %s" % last_text)
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run-dir", help="default: the most recent cell")
    ap.add_argument("--watch", action="store_true", help="refresh every 30 s")
    a = ap.parse_args(argv)

    rd = Path(a.run_dir) if a.run_dir else newest_cell()
    if rd is None:
        print("no cell run directories under %s" % RESULTS)
        return 1
    while True:
        text = report(rd)
        sys.stdout.buffer.write((text + "\n").encode("utf-8", "replace"))
        if not a.watch or (rd / "rows.jsonl").is_file():
            return 0
        time.sleep(30)
        print("-" * 72)


if __name__ == "__main__":
    sys.exit(main())
