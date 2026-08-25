#!/usr/bin/env python3
"""Exhaustive primary-simulator gate ledger for BuildScale v1."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
CONTRACT = HERE / "contract.json"
SPEC = HERE / "build_scale_spec.json"
RECORDS = HERE / "build_scale_gate_records"
OUTPUT = HERE / "build_scale_gates.json"


def _load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _records():
    found = {}
    for path in sorted(RECORDS.glob("*.json")) if RECORDS.is_dir() else []:
        row = _load(path)
        if row.get("schema") != "agenticsimbench/build-scale-live-gate/v1":
            raise ValueError(f"{path}: unknown BuildScale gate schema")
        if row.get("suite") != "agenticsimbench/v1":
            continue
        for relative, expected in row.get("source_sha256", {}).items():
            source = AGENTBENCH / relative
            if not source.is_file() or _sha(source) != expected:
                raise ValueError(f"{path}: source hash mismatch for {relative}")
        for relative, expected in row.get("run_evidence_sha256", {}).items():
            evidence = AGENTBENCH / relative
            if not evidence.is_file() or _sha(evidence) != expected:
                raise ValueError(f"{path}: run evidence mismatch for {relative}")
        if row.get("outcomes") != {"oracle": "PASS", "null": "FAIL"}:
            raise ValueError(f"{path}: expected oracle PASS and null FAIL")
        if row.get("oracle", {}).get("assertions_passed") != 9:
            raise ValueError(f"{path}: oracle did not pass all 9 assertions")
        if "BS.5" not in row.get("null", {}).get("failed_assertions", []):
            raise ValueError(f"{path}: null did not falsify physical movement")
        key = (row.get("sim"), int(row.get("level", 0)))
        if key in found:
            raise ValueError(f"duplicate BuildScale gate record for {key}")
        found[key] = (row, path)
    return found


def build():
    contract, spec = _load(CONTRACT), _load(SPEC)
    primary = contract["comparators"]["primary"]
    levels = spec["levels"]
    records = _records()
    rows = []
    for sim in primary:
        for level in levels:
            hit = records.get((sim, level))
            rows.append({
                "sim": sim,
                "level": level,
                "state": "READY" if hit else "BLOCKED",
                "detail": ("live oracle PASS and null FAIL"
                           if hit else "no hash-bound live gate record"),
                "evidence": ([] if not hit else [str(
                    hit[1].relative_to(AGENTBENCH)).replace("\\", "/")]),
            })
    ready = sum(row["state"] == "READY" for row in rows)
    return {
        "schema": "agenticsimbench/build-scale-gates/v1",
        "suite": contract["suite"],
        "comparators": primary,
        "levels": levels,
        "summary": {"cells_total": len(rows), "cells_ready": ready,
                    "cells_blocked": len(rows) - ready,
                    "scored_model_rows": 0,
                    "full_gate": "GREEN" if ready == len(rows) else "BLOCKED"},
        "rows": rows,
    }


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()
    encoded = json.dumps(build(), indent=2) + "\n"
    if args.write:
        OUTPUT.write_text(encoded, encoding="utf-8")
        print(f"wrote {OUTPUT}")
        return 0
    if not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != encoded:
        print(f"STALE: regenerate {OUTPUT} with --write")
        return 1
    print("BuildScale gates are current")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
