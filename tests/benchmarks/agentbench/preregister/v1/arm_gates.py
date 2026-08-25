#!/usr/bin/env python3
"""Build the AgenticSimBench v1 per-task/per-simulator publication ledger.

This is intentionally stricter than :mod:`agentbench.readiness`, which belongs
to the older campaign contract and iterates only adapters that already exist.
The v1 claim names five primary simulators.  Every one of their 50 cells must
therefore appear here, including adapters that have not been built yet.

An expressible cell is READY only when all five frozen gates are green.  A
future adapter may instead declare a task explicitly unsupported; that closes
the publication gate as an honest capability gap, but never counts as a solved
cell and therefore stops the contiguous frontier.  Anything else is BLOCKED.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
CONTRACT = HERE / "contract.json"
COVERAGE = HERE / "coverage.json"
OUTPUT = HERE / "arm_gates.json"
LOCAL_RECORDS = HERE / "arm_gate_records"
LEGACY_VERDICTS = HERE.parent / "oracle_verdicts.json"

if str(AGENTBENCH.parent) not in sys.path:
    sys.path.insert(0, str(AGENTBENCH.parent))

from agentbench import sims  # noqa: E402
from agentbench.agents import external as external_agent  # noqa: E402


GREEN = "GREEN"
BLOCKED = "BLOCKED"
NOT_APPLICABLE = "NOT_APPLICABLE"


def _load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _task_ids(contract: dict[str, Any]) -> list[str]:
    return [task for track in contract["tracks"].values() for task in track]


def _red_by_task() -> dict[str, bool]:
    doc = _load(COVERAGE)
    return {
        task: counts["validated"] == counts["total"] and counts["total"] > 0
        for task, counts in doc["summary"]["by_task"].items()
    }


def _gate_records() -> tuple[dict[tuple[str, str], dict[str, str]],
                             dict[tuple[str, str], list[str]]]:
    """Return explicit oracle/null outcomes and their source paths.

    Only records that name both the simulator and the driver role count.  The
    older scripted lane cells have positive oracles but no per-simulator null
    role, so treating them as a complete gate would recreate the exact C2
    failure mode this ledger is meant to prevent.
    """
    outcomes: dict[tuple[str, str], dict[str, str]] = {}
    sources: dict[tuple[str, str], list[str]] = {}

    if LEGACY_VERDICTS.is_file():
        doc = _load(LEGACY_VERDICTS)
        for row in (doc.get("driver_gates") or {}).get("cells") or []:
            role = row.get("agent")
            if role not in {"oracle", "null"}:
                continue
            key = (row.get("task"), row.get("sim"))
            if not all(key):
                continue
            outcomes.setdefault(key, {})[role] = row.get("outcome")
            sources.setdefault(key, []).append(
                str(LEGACY_VERDICTS.relative_to(AGENTBENCH)).replace("\\", "/"))

    if LOCAL_RECORDS.is_dir():
        for path in sorted(LOCAL_RECORDS.glob("*.json")):
            doc = _load(path)
            if doc.get("suite") != "agenticsimbench/v1":
                continue
            if doc.get("schema") != "agenticsimbench/live-arm-gate/v1":
                raise ValueError(f"{path}: unknown live gate schema")
            for relative, expected in (doc.get("source_sha256") or {}).items():
                source = AGENTBENCH / relative
                if not source.is_file() or _sha256(source) != expected:
                    raise ValueError(
                        f"{path}: source hash mismatch for {relative}; "
                        "the live gate must be rerun")
            key = (doc.get("task"), doc.get("sim"))
            if not all(key):
                raise ValueError(f"{path}: gate record needs task and sim")
            for role in ("oracle", "null"):
                if role in (doc.get("outcomes") or {}):
                    outcome = doc["outcomes"][role]
                    expected = "PASS" if role == "oracle" else "FAIL"
                    if outcome != expected:
                        raise ValueError(
                            f"{path}: {role} must be {expected}, got {outcome}")
                    outcomes.setdefault(key, {})[role] = outcome
            sources.setdefault(key, []).append(
                str(path.relative_to(AGENTBENCH)).replace("\\", "/"))

    return outcomes, sources


def _gate(state: str, detail: str, evidence: list[str] | None = None) -> dict[str, Any]:
    return {"state": state, "detail": detail, "evidence": evidence or []}


def _deliverable(task: str, sim_id: str) -> tuple[bool, str]:
    if task in external_agent.ANSWER_TASKS:
        return True, "agent final answer text"
    name = external_agent.artifact_name(task, sim_id)
    return bool(name), name or "no registered deliverable convention"


def build() -> dict[str, Any]:
    contract = _load(CONTRACT)
    primary = list(contract["comparators"]["primary"])
    tasks = _task_ids(contract)
    red = _red_by_task()
    recorded, record_sources = _gate_records()
    rows: list[dict[str, Any]] = []

    for sim_id in primary:
        sim = sims.get(sim_id)
        for task in tasks:
            adapter_ready = sim.implemented
            explicitly_unsupported = adapter_ready and not sim.expresses(task)
            gates: dict[str, dict[str, Any]] = {}

            if not adapter_ready:
                missing = sim.blocked_by or "adapter is not implemented"
                gates["expressible_or_explicitly_unsupported"] = _gate(
                    BLOCKED, "cannot classify task until adapter exists: " + missing)
                gates["deliverable_convention"] = _gate(
                    BLOCKED, "adapter missing; no v1 deliverable can be exercised")
                gates["oracle_PASS"] = _gate(
                    BLOCKED, "adapter missing; oracle has not run")
                gates["null_FAIL"] = _gate(
                    BLOCKED, "adapter missing; null control has not run")
                gates["no_task_scoped_bringup_blocker"] = _gate(BLOCKED, missing)
                state = BLOCKED
            elif explicitly_unsupported:
                gates["expressible_or_explicitly_unsupported"] = _gate(
                    GREEN, "adapter explicitly declares this task unsupported")
                for name in ("deliverable_convention", "oracle_PASS", "null_FAIL"):
                    gates[name] = _gate(
                        NOT_APPLICABLE,
                        "verified unsupported capability gap; no score is permitted")
                blockers = sim.pending_for(task)
                gates["no_task_scoped_bringup_blocker"] = _gate(
                    GREEN if not blockers else BLOCKED,
                    "no task-scoped blocker" if not blockers else "; ".join(
                        item.text() for item in blockers))
                state = "UNSUPPORTED" if not blockers else BLOCKED
            else:
                gates["expressible_or_explicitly_unsupported"] = _gate(
                    GREEN, "adapter declares task expressible")
                deliverable_ok, convention = _deliverable(task, sim_id)
                gates["deliverable_convention"] = _gate(
                    GREEN if deliverable_ok else BLOCKED, convention)
                key = (task, sim_id)
                outcomes = recorded.get(key, {})
                evidence = sorted(set(record_sources.get(key, [])))
                gates["oracle_PASS"] = _gate(
                    GREEN if outcomes.get("oracle") == "PASS" else BLOCKED,
                    "oracle PASS recorded" if outcomes.get("oracle") == "PASS"
                    else "no oracle PASS recorded for this task/simulator",
                    evidence)
                gates["null_FAIL"] = _gate(
                    GREEN if outcomes.get("null") == "FAIL" else BLOCKED,
                    "null FAIL recorded" if outcomes.get("null") == "FAIL"
                    else "no null FAIL recorded for this task/simulator",
                    evidence)
                blockers = sim.pending_for(task)
                gates["no_task_scoped_bringup_blocker"] = _gate(
                    GREEN if not blockers else BLOCKED,
                    "no task-scoped blocker" if not blockers else "; ".join(
                        item.text() for item in blockers))
                state = ("READY" if all(g["state"] == GREEN for g in gates.values())
                         else BLOCKED)

            rows.append({
                "task": task,
                "sim": sim_id,
                "adapter_implemented": adapter_ready,
                "red_evidence": _gate(
                    GREEN if red.get(task) else BLOCKED,
                    "all task assertions have real red evidence" if red.get(task)
                    else "task red evidence is incomplete",
                    [str(COVERAGE.relative_to(AGENTBENCH)).replace("\\", "/")]),
                "gates": gates,
                "publication_state": state,
            })

    states = Counter(row["publication_state"] for row in rows)
    adapters = {sim_id: sims.get(sim_id).implemented for sim_id in primary}
    return {
        "schema": "agenticsimbench/arm-gates/v1",
        "suite": contract["suite"],
        "comparators": primary,
        "tasks": tasks,
        "summary": {
            "cells_total": len(rows),
            "cells_ready": states["READY"],
            "cells_explicitly_unsupported": states["UNSUPPORTED"],
            "cells_blocked": states[BLOCKED],
            "primary_adapters_ready": sum(adapters.values()),
            "primary_adapters_total": len(adapters),
            "red_assertions_validated": _load(COVERAGE)["summary"]["validated"],
            "red_assertions_total": _load(COVERAGE)["summary"]["total"],
            "full_claim_gate": "GREEN" if not states[BLOCKED] else "BLOCKED",
        },
        "adapter_status": adapters,
        "rows": rows,
    }


def _encoded() -> str:
    return json.dumps(build(), indent=2, sort_keys=True) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    encoded = _encoded()
    if args.write:
        OUTPUT.write_text(encoded, encoding="utf-8")
    if args.check and (
        not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != encoded
    ):
        print(f"stale arm-gate ledger: run {Path(__file__).name} --write")
        return 1
    if args.json:
        print(encoded, end="")
    elif not args.write:
        summary = build()["summary"]
        print(
            "v1 arm gates: {cells_ready}/{cells_total} READY; "
            "{cells_explicitly_unsupported} explicitly unsupported; "
            "{cells_blocked} BLOCKED; adapters "
            "{primary_adapters_ready}/{primary_adapters_total}".format(**summary)
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
