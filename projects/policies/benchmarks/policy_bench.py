#!/usr/bin/env python3
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

"""Living acceptance runner for the OmniSim Shadowing + BATON policy block.

The simulator emits machine-readable ``BATON-CYCLE`` verdicts and named physical-task
events. This tool turns those logs into an explicit threshold verdict. It never infers
success from an upright final frame and never treats a missing event as success.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

POLICIES = Path(__file__).resolve().parents[1]
REPO = POLICIES.parents[1]
SUITE_PATH = Path(__file__).with_name("suite.json")
sys.path.insert(0, str(POLICIES / "skills"))
sys.path.insert(0, str(POLICIES / "training"))

import baton_metrics as BM  # noqa: E402
import manifest as M  # noqa: E402

QUAD_TELEMETRY_RE = re.compile(
    r"\[t=(\d+)s\] mode=(\w+)\s+u=([\d.]+) sw=(\d+) "
    r"x=([+\-\d.]+) y=([+\-\d.]+) bz=([+\-\d.]+) "
    r"roll=([+\-\d.]+) vx=([+\-\d.]+) gm=([+\-\d.]+)")


def load_quad_telemetry(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = QUAD_TELEMETRY_RE.search(line)
        if match:
            rows.append({
                "t": int(match.group(1)), "mode": match.group(2),
                "u": float(match.group(3)), "switches": int(match.group(4)),
                "x": float(match.group(5)), "y": float(match.group(6)),
                "z": float(match.group(7)), "roll": float(match.group(8)),
                "vx": float(match.group(9)), "gmatch": float(match.group(10)),
            })
    return rows


def load_suite(path: Path = SUITE_PATH) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("benchmark suite must be a JSON object")
    return data


def cases_by_name(suite: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {case["name"]: case for case in suite.get("cases", [])}


def validate_suite(registry: M.Registry | None = None,
                   suite: dict[str, Any] | None = None) -> list[str]:
    suite = suite or load_suite()
    registry = registry or M.Registry.discover()
    issues: list[str] = []
    if suite.get("schema") != 1:
        issues.append(f"benchmark schema must be 1, got {suite.get('schema')!r}")
    seen: set[str] = set()
    for i, case in enumerate(suite.get("cases", [])):
        name = case.get("name", "")
        where = name or f"case[{i}]"
        if not name:
            issues.append(f"{where}: missing name")
        elif name in seen:
            issues.append(f"{where}: duplicate benchmark name")
        seen.add(name)
        sequence = case.get("sequence", "")
        if sequence not in registry.sequences:
            issues.append(f"{where}: unknown sequence {sequence!r}")
        if not case.get("support"):
            issues.append(f"{where}: support disclosure is required")
        thresholds = case.get("thresholds")
        if not isinstance(thresholds, dict) or not thresholds:
            issues.append(f"{where}: non-empty thresholds object is required")
        for event in case.get("required_events", []):
            if not event.get("pattern") or int(event.get("min_count", 0)) < 1:
                issues.append(f"{where}: each required event needs pattern + min_count>=1")
        if case.get("status") == "verified":
            evidence = case.get("reference_evidence")
            if not isinstance(evidence, dict):
                issues.append(f"{where}: verified case requires structured reference_evidence")
            else:
                for key in ("date", "machine_id", "venue", "engine", "result_file", "result"):
                    if not evidence.get(key):
                        issues.append(f"{where}: reference_evidence missing {key!r}")
                result_file = evidence.get("result_file")
                if result_file:
                    result_path = REPO / result_file
                    if not result_path.exists():
                        issues.append(f"{where}: reference result not found: {result_file}")
                    else:
                        try:
                            recorded = json.loads(result_path.read_text(encoding="utf-8"))
                        except Exception as exc:  # noqa: BLE001
                            issues.append(f"{where}: invalid reference result: {exc}")
                        else:
                            if recorded.get("benchmark") != name or recorded.get("passed") is not True:
                                issues.append(f"{where}: reference result does not record a PASS for this case")
                            criteria = recorded.get("criteria") or {}
                            if criteria.get("thresholds") != case.get("thresholds"):
                                issues.append(f"{where}: reference result was scored against different thresholds")
                            if criteria.get("required_events") != case.get("required_events", []):
                                issues.append(f"{where}: reference result was scored against different events")
    return issues


def score(case: dict[str, Any], rl_path: Path, mpc_path: Path | None = None) -> dict[str, Any]:
    if not rl_path.exists():
        raise FileNotFoundError(rl_path)
    mpc_path = mpc_path if mpc_path and mpc_path.exists() else None
    cycles = BM.load_cycles(rl_path)
    switches, telemetry = BM.load(rl_path, mpc_path or rl_path)
    quad = load_quad_telemetry(rl_path)
    text = rl_path.read_text(encoding="utf-8", errors="replace")
    # The MPC side log mirrors controller events with an ``[inengine-mpc]`` prefix. Count events
    # from the RL log only or every physical transition is double-counted.

    complete = [c for c in cycles if c["ok"] and c["segs"] == c["nsegs"]]
    zs = [row[2] for row in telemetry.values()] or [row["z"] for row in quad]
    rolls = [abs(row[3]) for row in telemetry.values()] or [abs(row["roll"]) for row in quad]
    fall_height = float(case.get("fall_height_m", 0.45))
    fall_ticks = sum(z < fall_height for z in zs)
    turn = [row for row in quad if row["mode"] == "turn"]
    post_turn = [row for row in quad if row["mode"] == "walk" and row["switches"] >= 2]

    def displacement(rows: list[dict[str, Any]]) -> float | None:
        if len(rows) < 2:
            return None
        return ((rows[-1]["x"] - rows[0]["x"]) ** 2
                + (rows[-1]["y"] - rows[0]["y"]) ** 2) ** 0.5

    metrics = {
        "cycles_emitted": len(cycles),
        "complete_cycles": len(complete),
        "cycle_min_pelvis_z_m": min((c["minz"] for c in complete), default=None),
        "cycle_max_duration_s": max((c["dur"] for c in complete), default=None),
        "switches": max([len(switches)] + [row["switches"] for row in quad]),
        "telemetry_ticks": len(telemetry) or len(quad),
        "telemetry_min_pelvis_z_m": min(zs) if zs else None,
        "max_abs_roll_rad": max(rolls) if rolls else None,
        "turn_displacement_m": displacement(turn),
        "post_turn_displacement_m": displacement(post_turn),
        "turn_samples": len(turn),
        "fall_height_m": fall_height,
        "fall_ticks": fall_ticks,
    }
    checks: list[dict[str, Any]] = []

    def check(name: str, actual: Any, op: str, expected: Any, passed: bool) -> None:
        checks.append({"name": name, "actual": actual, "operator": op,
                       "expected": expected, "passed": bool(passed)})

    t = case.get("thresholds", {})
    if "min_complete_cycles" in t:
        check("complete_cycles", metrics["complete_cycles"], ">=", t["min_complete_cycles"],
              metrics["complete_cycles"] >= t["min_complete_cycles"])
    if "min_cycle_pelvis_z_m" in t:
        actual = metrics["cycle_min_pelvis_z_m"]
        check("cycle_min_pelvis_z_m", actual, ">=", t["min_cycle_pelvis_z_m"],
              actual is not None and actual >= t["min_cycle_pelvis_z_m"])
    if "max_cycle_duration_s" in t:
        actual = metrics["cycle_max_duration_s"]
        check("cycle_max_duration_s", actual, "<=", t["max_cycle_duration_s"],
              actual is not None and actual <= t["max_cycle_duration_s"])
    if "min_switches" in t:
        check("switches", metrics["switches"], ">=", t["min_switches"],
              metrics["switches"] >= t["min_switches"])
    if "min_telemetry_pelvis_z_m" in t:
        actual = metrics["telemetry_min_pelvis_z_m"]
        check("telemetry_min_pelvis_z_m", actual, ">=", t["min_telemetry_pelvis_z_m"],
              actual is not None and actual >= t["min_telemetry_pelvis_z_m"])
    if "max_abs_roll_rad" in t:
        actual = metrics["max_abs_roll_rad"]
        check("max_abs_roll_rad", actual, "<=", t["max_abs_roll_rad"],
              actual is not None and actual <= t["max_abs_roll_rad"])
    if "max_turn_displacement_m" in t:
        actual = metrics["turn_displacement_m"]
        check("turn_displacement_m", actual, "<=", t["max_turn_displacement_m"],
              actual is not None and actual <= t["max_turn_displacement_m"])
    if "min_post_turn_displacement_m" in t:
        actual = metrics["post_turn_displacement_m"]
        check("post_turn_displacement_m", actual, ">=", t["min_post_turn_displacement_m"],
              actual is not None and actual >= t["min_post_turn_displacement_m"])
    if "min_turn_samples" in t:
        check("turn_samples", metrics["turn_samples"], ">=", t["min_turn_samples"],
              metrics["turn_samples"] >= t["min_turn_samples"])
    if "max_fall_ticks" in t:
        check("fall_ticks", metrics["fall_ticks"], "<=", t["max_fall_ticks"],
              metrics["fall_ticks"] <= t["max_fall_ticks"])
    for event in case.get("required_events", []):
        count = len(re.findall(event["pattern"], text))
        check(f"event:{event['pattern']}", count, ">=", event["min_count"],
              count >= event["min_count"])

    return {
        "schema": 1,
        "benchmark": case["name"],
        "sequence": case["sequence"],
        "support": case["support"],
        "logs": {"rl": str(rl_path), "mpc": str(mpc_path) if mpc_path else None},
        "criteria": {
            "thresholds": case.get("thresholds", {}),
            "required_events": case.get("required_events", []),
        },
        "metrics": metrics,
        "checks": checks,
        "passed": bool(checks) and all(c["passed"] for c in checks),
    }


def _print_result(result: dict[str, Any]) -> None:
    print(f"[{('PASS' if result['passed'] else 'FAIL')}] {result['benchmark']}  "
          f"({result['support']})")
    for item in result["checks"]:
        mark = "ok" if item["passed"] else "FAIL"
        print(f"  [{mark:<4}] {item['name']}: {item['actual']!r} "
              f"{item['operator']} {item['expected']!r}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="OmniSim policy-block acceptance benchmarks")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("list")
    sub.add_parser("validate")
    cp = sub.add_parser("command")
    cp.add_argument("case")
    sp = sub.add_parser("score")
    sp.add_argument("case")
    sp.add_argument("--rl", type=Path, required=True)
    sp.add_argument("--mpc", type=Path)
    sp.add_argument("--json", action="store_true")
    rp = sub.add_parser("run")
    rp.add_argument("case")
    args = ap.parse_args(argv)
    suite = load_suite()
    cases = cases_by_name(suite)

    if args.cmd in (None, "list"):
        for case in cases.values():
            print(f"{case['name']:<28} {case['status']:<12} sequence={case['sequence']}  "
                  f"duration={case['duration_s']}s")
        return 0
    if args.cmd == "validate":
        issues = validate_suite(suite=suite)
        for issue in issues:
            print(f"[ERR ] {issue}")
        print(f"benchmark suite: {len(cases)} case(s), {len(issues)} error(s)")
        return 1 if issues else 0
    case = cases.get(args.case)
    if not case:
        print(f"unknown benchmark {args.case!r}", file=sys.stderr)
        return 2
    command = [sys.executable, str(POLICIES / "skills" / "skill_lib.py"), "sequence",
               case["sequence"], "--duration", str(case["duration_s"]), "--gui", "headless"]
    if args.cmd == "command":
        print(subprocess.list2cmdline(command))
        return 0
    if args.cmd == "run":
        rc = subprocess.call(command, cwd=REPO)
        if rc:
            return rc
        tag = case["sequence"]
        result = score(case, REPO / "_scratch" / "foot_redesign" / f"{tag}_rl.txt",
                       REPO / "_scratch" / "foot_redesign" / f"{tag}_mpc.txt")
        _print_result(result)
        return 0 if result["passed"] else 1
    result = score(case, args.rl, args.mpc)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        _print_result(result)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
