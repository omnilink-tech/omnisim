#!/usr/bin/env python3
"""Build the cumulative v1 red-evidence coverage view."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
BASELINE = AGENTBENCH / "phase0_validation" / "coverage.json"
EVIDENCE_DIR = HERE / "red_evidence"
OUTPUT = HERE / "coverage.json"


def _load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def build() -> dict[str, Any]:
    """Layer qualifying v1 observations on the immutable Phase 0 baseline."""
    doc = copy.deepcopy(_load(BASELINE))
    rows = doc["rows"]
    index = {(row["task"], row["assertion"]): row for row in rows}
    evidence_files: list[str] = []

    for path in sorted(EVIDENCE_DIR.glob("*.verdict.json")):
        verdict = _load(path)
        if verdict.get("suite") != "agenticsimbench/v1":
            continue
        fixture = verdict.get("fixture")
        fixture_kind = verdict.get("fixture_kind")
        expected = set(verdict.get("expected_failures", []))
        observed = set(verdict.get("observed_failures", []))
        if (
            not isinstance(fixture, str)
            or not fixture
            or fixture_kind == "null"
            or not verdict.get("qualifies_as_non_null_red_evidence")
            or not expected
            or expected != observed
        ):
            continue

        task = verdict["task"]
        for assertion in sorted(observed):
            key = (task, assertion)
            if key not in index:
                raise ValueError(f"evidence references unknown assertion {task}/{assertion}")
            row = index[key]
            row["validated"] = True
            row["v1_observed_red"] = True
            reference = str(path.relative_to(AGENTBENCH)).replace("\\", "/")
            references = row.setdefault("v1_evidence", [])
            if reference not in references:
                references.append(reference)
        evidence_files.append(str(path.relative_to(AGENTBENCH)).replace("\\", "/"))

    by_task: dict[str, dict[str, int]] = {}
    for row in rows:
        bucket = by_task.setdefault(row["task"], {"validated": 0, "total": 0})
        bucket["total"] += 1
        bucket["validated"] += int(bool(row["validated"]))

    validated = sum(int(bool(row["validated"])) for row in rows)
    doc["suite"] = "agenticsimbench/v1"
    doc["baseline"] = str(BASELINE.relative_to(AGENTBENCH)).replace("\\", "/")
    doc["evidence_files"] = evidence_files
    doc["summary"] = {
        "validated": validated,
        "total": len(rows),
        "open": len(rows) - validated,
        "by_task": by_task,
    }
    return doc


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
        not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != encoded
    ):
        print(f"stale v1 coverage view: run {Path(__file__).name} --write")
        return 1
    if args.json:
        print(encoded, end="")
    elif not args.write:
        summary = build()["summary"]
        print(
            f"v1 red evidence: {summary['validated']}/{summary['total']} validated; "
            f"{summary['open']} open"
        )
        for task, counts in summary["by_task"].items():
            print(f"  {task}: {counts['validated']}/{counts['total']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
