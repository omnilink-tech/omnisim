# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Provenance, result schema and reporting for the OmniLink agent benchmark.

Deliberately mirrors ``tests/benchmarks/omnibench/common/results.py``: JSON
Lines, one row per (task, engine, repeat), each row carrying a machine
fingerprint so a number can be attributed to the box that produced it.

The honesty rules this module exists to enforce:

* A row is written only for a run that actually happened. There is no
  "expected" or "estimated" field anywhere in the schema.
* ``outcome`` has five values and four of them are not a score:
  ``PASS`` / ``FAIL`` are model results; ``INVALID`` (infrastructure broke,
  or the task's precondition was not established), ``SKIPPED`` (no credential
  for this engine) and ``ERROR`` (the harness itself failed) are *not*, and
  the summary counts them separately instead of folding them into a
  denominator.
* Missing cost is ``None``, never ``0.0``. Zero is a measurement; absence is
  not.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

SUITE = "omnilink-tasks/v1"

# Outcomes that are a statement about the MODEL. Everything else is a
# statement about the harness, the stack, or the account.
GRADED = ("PASS", "FAIL")
UNGRADED = ("INVALID", "SKIPPED", "ERROR")

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[2]

_FINGERPRINT_CACHE: Optional[Dict[str, Any]] = None
_GIT_SHA_CACHE: Optional[str] = None


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def enable_utf8_console() -> None:
    """Make stdout survive a degree sign.

    On Windows the default console encoding is cp1252, so a single '±', '°'
    or '⚠' in a grader's detail line raises UnicodeEncodeError inside
    codecs.charmap_encode and kills the run — scoring a console-encoding
    problem as a harness crash. The sibling suite lost a whole task to
    exactly this. Call from every entry point.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
        except Exception:
            pass


def git_sha(repo_root: Optional[Path] = None) -> Optional[str]:
    """Short git sha of the tree the benchmark ran from, or None.

    Suffixed ``+dirty`` when the working tree has uncommitted changes — a
    result from a dirty tree is not reproducible from the sha alone and the
    row should say so.
    """
    global _GIT_SHA_CACHE
    if _GIT_SHA_CACHE is not None:
        return _GIT_SHA_CACHE or None
    root = str(repo_root or REPO_ROOT)
    try:
        sha = subprocess.run(["git", "-C", root, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=20)
        if sha.returncode != 0:
            _GIT_SHA_CACHE = ""
            return None
        out = sha.stdout.strip()
        dirty = subprocess.run(["git", "-C", root, "status", "--porcelain"],
                               capture_output=True, text=True, timeout=30)
        if dirty.returncode == 0 and dirty.stdout.strip():
            out += "+dirty"
        _GIT_SHA_CACHE = out
        return out
    except Exception:
        _GIT_SHA_CACHE = ""
        return None


def machine_fingerprint() -> Dict[str, Any]:
    """Machine attribution, same source as OmniBench.

    Reuses ``omnibench/common/results.py`` so an OmniLink row and an OmniBench
    row name the same box the same way. Falls back to a small local identity if
    that import fails — a degraded fingerprint is labelled, never silently
    empty.

    ``OMNILINK_BENCH_FAST_FINGERPRINT=1`` skips the OmniBench path (which
    shells out to nvidia-smi and walks site-packages, ~seconds) and uses the
    cheap local identity. Use it for offline harness tests, never for a real
    run.
    """
    global _FINGERPRINT_CACHE
    if _FINGERPRINT_CACHE is not None:
        return _FINGERPRINT_CACHE
    fp: Dict[str, Any]
    if os.environ.get("OMNILINK_BENCH_FAST_FINGERPRINT", "").strip() == "1":
        fp = _local_identity()
        fp["fingerprint_source"] = "local-fast (OMNILINK_BENCH_FAST_FINGERPRINT=1)"
    else:
        try:
            sys.path.insert(0, str(REPO_ROOT / "tests" / "benchmarks" / "omnibench"))
            from common import results as ob_results  # type: ignore
            fp = dict(ob_results.machine_fingerprint())
            fp["fingerprint_source"] = "omnibench/common/results.py"
        except Exception as exc:  # noqa: BLE001
            fp = _local_identity()
            fp["fingerprint_source"] = f"local fallback ({type(exc).__name__}: {exc})"
    fp = json.loads(json.dumps(fp, default=str))
    _FINGERPRINT_CACHE = fp
    return fp


def _local_identity() -> Dict[str, Any]:
    """Cheap, non-personal machine identity. Same hashing convention as
    projects/policies/common/env_fingerprint.py: the hostname is hashed
    because result rows get committed and a hostname can carry a real name."""
    import hashlib
    import platform
    try:
        host = platform.node() or ""
    except Exception:
        host = ""
    try:
        cpu = platform.processor() or platform.machine() or ""
    except Exception:
        cpu = ""
    try:
        osfam = platform.platform(terse=True)
    except Exception:
        osfam = ""
    payload = "|".join(["", cpu, osfam, host])
    mid = hashlib.sha256(payload.encode("utf-8", "replace")).hexdigest()[:12]
    host_id = ("h" + hashlib.sha256(host.encode("utf-8", "replace")).hexdigest()[:10]
               ) if host else "?"
    return {"machine": {"id": mid, "host": host_id, "cpu": cpu or "?",
                        "cores": os.cpu_count(), "gpu": "unmeasured"},
            "os": osfam}


# ── Result rows ──────────────────────────────────────────────────────


def make_row(*, task: str, task_version: str, category: str, objective: bool,
             engine: str, model: Optional[str], repeat: int,
             outcome: str, detail: str, reason: Optional[str] = None,
             metric: Optional[Dict[str, Any]] = None,
             latency_s: Optional[float] = None,
             turns: Optional[int] = None,
             tool_calls: Optional[int] = None,
             cost: Optional[Dict[str, Any]] = None,
             trace: Optional[Dict[str, Any]] = None,
             stack: Optional[Dict[str, Any]] = None,
             machine: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Build one schema-conformant row. Does not write it.

    ``cost`` is passed through verbatim, including ``{"credits_usd": None}``
    — the caller is responsible for never turning an unmeasured cost into 0.
    """
    if outcome not in GRADED + UNGRADED:
        raise ValueError(f"outcome {outcome!r} not in {GRADED + UNGRADED}")
    return {
        "suite": SUITE,
        "task": task,
        "task_version": task_version,
        "category": category,
        "objective": bool(objective),
        "engine": engine,
        "model": model,
        "repeat": int(repeat),
        "outcome": outcome,
        "reason": reason,
        "detail": detail,
        "metric": metric or {},
        "latency_s": (None if latency_s is None else round(float(latency_s), 3)),
        "turns": turns,
        "tool_calls": tool_calls,
        "cost": cost if cost is not None else {"credits_usd": None,
                                               "source": "not sampled"},
        "trace": trace or {},
        "stack": stack or {},
        "git_sha": git_sha(),
        "machine": machine if machine is not None else machine_fingerprint(),
        "utc": utc_now(),
    }


def append_row(path: Path | str, row: Dict[str, Any]) -> Dict[str, Any]:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a", encoding="utf-8") as f:
        f.write(json.dumps(row, default=str) + "\n")
    return row


def read_rows(path: Path | str) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


# ── Aggregation + reporting ──────────────────────────────────────────


def aggregate(rows: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Fold rows into {(engine, task): stats} plus per-engine totals.

    Pass rate is ``passed / graded``, where ``graded`` counts only PASS+FAIL.
    A task that was never graded contributes to ``ungraded`` and to nothing
    else — it must not silently deflate or inflate a rate.
    """
    per_cell: Dict[str, Dict[str, Any]] = {}
    per_engine: Dict[str, Dict[str, Any]] = {}
    for r in rows:
        eng = r.get("engine", "?")
        task = r.get("task", "?")
        key = f"{eng}\t{task}"
        cell = per_cell.setdefault(key, {
            "engine": eng, "task": task, "category": r.get("category", "?"),
            "objective": r.get("objective", True),
            "runs": 0, "passed": 0, "failed": 0, "ungraded": 0,
            "ungraded_reasons": [], "latencies": [], "details": [],
        })
        eng_tot = per_engine.setdefault(eng, {
            "engine": eng, "runs": 0, "passed": 0, "failed": 0, "ungraded": 0,
            "skipped": 0, "latency_s": 0.0, "cost_usd": None,
            "cost_source": None, "ungraded_reasons": [],
        })
        cell["runs"] += 1
        eng_tot["runs"] += 1
        outcome = r.get("outcome")
        if outcome == "PASS":
            cell["passed"] += 1
            eng_tot["passed"] += 1
        elif outcome == "FAIL":
            cell["failed"] += 1
            eng_tot["failed"] += 1
        else:
            cell["ungraded"] += 1
            eng_tot["ungraded"] += 1
            if outcome == "SKIPPED":
                eng_tot["skipped"] += 1
            why = r.get("reason") or r.get("detail") or outcome
            cell["ungraded_reasons"].append(f"{outcome}: {why}")
            eng_tot["ungraded_reasons"].append(f"{outcome}: {why}")
        if r.get("latency_s") is not None:
            cell["latencies"].append(float(r["latency_s"]))
            eng_tot["latency_s"] += float(r["latency_s"])
        if r.get("detail"):
            cell["details"].append(str(r["detail"]))

    for cell in per_cell.values():
        graded = cell["passed"] + cell["failed"]
        cell["graded"] = graded
        cell["pass_rate"] = (cell["passed"] / graded) if graded else None
        cell["latency_mean_s"] = (sum(cell["latencies"]) / len(cell["latencies"])
                                  if cell["latencies"] else None)
    for tot in per_engine.values():
        graded = tot["passed"] + tot["failed"]
        tot["graded"] = graded
        tot["pass_rate"] = (tot["passed"] / graded) if graded else None
    return {"cells": per_cell, "engines": per_engine}


def _rate_cell(cell: Dict[str, Any]) -> str:
    if cell["graded"] == 0:
        return "n/a"
    if cell["runs"] == cell["graded"] == 1:
        return "PASS" if cell["passed"] else "FAIL"
    return f"{cell['passed']}/{cell['graded']}"


def markdown_summary(rows: List[Dict[str, Any]], *, repeats: int,
                     engine_cost: Optional[Dict[str, Dict[str, Any]]] = None,
                     header: str = "") -> str:
    """Markdown report: task x engine pass-rate matrix + per-engine totals.

    Every cell shows ``passed/graded``; a cell with nothing graded shows
    ``n/a`` and the reason is printed underneath rather than being hidden.
    """
    agg = aggregate(rows)
    engines: List[str] = []
    tasks: List[str] = []
    meta: Dict[str, Dict[str, Any]] = {}
    for r in rows:
        if r.get("engine") not in engines:
            engines.append(r.get("engine"))
        if r.get("task") not in tasks:
            tasks.append(r.get("task"))
            meta[r["task"]] = {"category": r.get("category", "?"),
                               "objective": r.get("objective", True)}

    out: List[str] = []
    if header:
        out.append(header.rstrip() + "\n")
    out.append(f"**Suite** `{SUITE}` · **repeats** {repeats} · "
               f"**git** `{git_sha() or 'unknown'}` · **utc** {utc_now()}")
    mach = (machine_fingerprint().get("machine") or {})
    out.append(f"**Machine** `{mach.get('id', '?')}` "
               f"(gpu {mach.get('gpu', '?')}, cpu {mach.get('cpu', '?')})\n")

    out.append("## Pass rate (passed / graded)\n")
    out.append("| task | category | " + " | ".join(engines) + " |")
    out.append("|---|---|" + "---|" * len(engines))
    for t in tasks:
        cells = []
        for e in engines:
            cell = agg["cells"].get(f"{e}\t{t}")
            cells.append(_rate_cell(cell) if cell else "-")
        star = "" if meta[t]["objective"] else " ⚠"
        out.append(f"| `{t}`{star} | {meta[t]['category']} | " + " | ".join(cells) + " |")
    out.append("")
    if any(not meta[t]["objective"] for t in tasks):
        out.append("⚠ = graded partly on the agent's own words (see the task's "
                   "`measures` line and the classifier limits in the README). "
                   "Every other task is graded on measured robot pose or on the "
                   "recorded tool-call trace.\n")

    out.append("## Per engine\n")
    out.append("| engine | passed / graded | ungraded | wall (s) | measured cost |")
    out.append("|---|---|---|---|---|")
    for e in engines:
        tot = agg["engines"][e]
        cost_txt = "not sampled"
        if engine_cost and e in engine_cost:
            c = engine_cost[e]
            usd = c.get("credits_usd")
            cost_txt = (f"${usd:.4f}" if isinstance(usd, (int, float))
                        else f"unmeasured ({c.get('source', '?')})")
        rate = (f"{tot['passed']}/{tot['graded']}" if tot["graded"]
                else "**nothing graded**")
        out.append(f"| {e} | {rate} | {tot['ungraded']} | "
                   f"{tot['latency_s']:.1f} | {cost_txt} |")
    out.append("")

    ungraded_notes: List[str] = []
    for e in engines:
        tot = agg["engines"][e]
        if tot["ungraded"]:
            seen: List[str] = []
            for why in tot["ungraded_reasons"]:
                if why not in seen:
                    seen.append(why)
            ungraded_notes.append(f"- **{e}** — {tot['ungraded']} run(s) not "
                                  f"graded: " + "; ".join(seen[:4]))
    if ungraded_notes:
        out.append("## Not graded\n")
        out.append("These are not model failures. They are missing credentials, "
                   "an unarmed fault, or an unreachable stack.\n")
        out.extend(ungraded_notes)
        out.append("")
    return "\n".join(out) + "\n"
