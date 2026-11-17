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

"""Sequential independent cold starts; preserve every child trial and verdict."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

HERE = Path(__file__).resolve().parent
# Run as a script, this folder is already on sys.path; loaded by file path (as
# test_measurement.py does, under the repo's pytest import mode) it is not.
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from run_trial import PORTS, json_safe, pose_errors, score_trial, world_to_map  # noqa: E402
MINIMUM_RUNS = 3


def cleanup_complete(data):
    return (bool(data.get("process_exit")) and all(code == 0 for code in data["process_exit"].values())
            and set(data.get("cleanup_listeners", {})) == {str(port) for port in PORTS}
            and not any(data["cleanup_listeners"].values()))


def summarize(paths, requested_runs):
    records, valid_results, reasons = [], [], []
    ids, directories = set(), set()
    signature = None
    for path in paths:
        path = Path(path).resolve()
        record = dict(result=str(path), overall="INVALID")
        try:
            if path.parent in directories:
                raise ValueError("duplicate trial output directory")
            directories.add(path.parent)
            data = json.loads(path.read_text())
            identity = data["trial_id"]
            if not identity or identity in ids:
                raise ValueError("missing/duplicate independent trial identity")
            ids.add(identity)
            if data.get("schema_version", 0) < 2 or data.get("cold_start") is not True:
                raise ValueError("not an independent new-capability cold-start trial")
            verdict = data["verdicts"]
            record.update(overall=verdict["overall"], trial_id=identity, trial_verdicts=verdict)
            if verdict["overall"] not in ("PASS", "FAIL", "INVALID", "NOT_SCORED"):
                raise ValueError("unknown trial verdict")
            record["observations"] = data.get("stationary_observations", {})
            record["provenance"] = data.get("provenance", {})
            if not cleanup_complete(data):
                raise ValueError("trial cleanup is not established")
            thresholds = data["configuration"]["acceptance_thresholds"]
            observation = data["configuration"]["observation"]
            measured = score_trial(data["endpoints"], data.get("invalid_reasons", []), record["observations"],
                                   thresholds["position_m"], thresholds["absolute_yaw_rad"],
                                   observation["position_jitter_limit_m"], observation["yaw_jitter_limit_rad"])
            if measured["measurement"] != "PASS":
                raise ValueError(f"invalid trial measurement: {measured['reasons']}")
            if any(verdict[key] != measured[key] for key in ("measurement", "localization", "overall")):
                raise ValueError("recorded verdict contradicts measured evidence/thresholds")
            record["endpoints"] = {phase: pose_errors(data["endpoints"][phase]["amcl"],
                                                      world_to_map(data["endpoints"][phase]["ground_truth"][-1]["node"]))
                                   for phase in ("initial", "post_motion")}
            current = dict(commit=data["provenance"]["commit"], sha256=data["provenance"]["sha256"],
                           machine=data["provenance"]["machine"], physics=data["provenance"]["physics_packages"],
                           configuration=data["configuration"])
            if signature is not None and signature != current:
                raise ValueError("source/runtime/environment/configuration differs between trials")
            signature = current
            valid_results.append(record)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            record.update(overall="INVALID", reason=str(exc))
            reasons.append(f"{path}: {exc}")
        records.append(record)
    counts = Counter(record["overall"] for record in records)
    if len(paths) < MINIMUM_RUNS or len(paths) != requested_runs:
        reasons.append("at least three completed independent valid trials are required")
    if reasons or counts["INVALID"]:
        outcome = "INVALID"
    elif counts["NOT_SCORED"]:
        outcome = "NOT_SCORED" if counts["NOT_SCORED"] == len(paths) else "INVALID"
    else:
        outcome = "FAIL" if counts["FAIL"] else "PASS"
    statistics_by_phase = {}
    for phase in ("initial", "post_motion"):
        values = {"position_error_m": [], "absolute_yaw_error_rad": [], "convergence_time_s": []}
        for record in valid_results:
            errors = record["endpoints"][phase]
            values["position_error_m"].append(errors["position_error_m"])
            values["absolute_yaw_error_rad"].append(abs(errors["yaw_error_rad"]))
            t = record["observations"][phase]["convergence"]["convergence_time_s"]
            if t is not None:
                values["convergence_time_s"].append(t)
        statistics_by_phase[phase] = {key: dict(count=len(v), mean=statistics.mean(v) if v else None,
                                                maximum=max(v) if v else None) for key, v in values.items()}
    return dict(schema_version=1, requested_runs=requested_runs, actual_runs=len(paths),
                counts={key: counts[key] for key in ("PASS", "FAIL", "INVALID", "NOT_SCORED")},
                pass_rate=counts["PASS"]/len(paths) if paths else 0., overall=outcome,
                repeatability_verified=outcome == "PASS", reasons=reasons, trials=records,
                statistics=statistics_by_phase, comparison_provenance=signature,
                note="Statistics use valid trials only; null convergence times are excluded, not replaced by zero.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=MINIMUM_RUNS)
    parser.add_argument("--output", type=Path, required=True, help="new repeat directory")
    args, extra = parser.parse_known_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    paths = []
    env = dict(os.environ)
    env.pop("_HUSKY_AMCL_ENV", None)  # Every child must establish its own clean environment.
    try:
        for index in range(args.runs):
            folder = out / f"trial-{index+1:03d}"
            paths.append(folder / "result.json")
            with (out / f"trial-{index+1:03d}.log").open("w") as log:
                subprocess.run([sys.executable, "-B", str(HERE / "run_trial.py"),
                                "--output", str(folder), *extra], stdout=log, stderr=subprocess.STDOUT,
                               env=env, check=False)
            summary = summarize(paths, args.runs)
            (out / "repeat_summary.json").write_text(json.dumps(json_safe(summary), indent=2, allow_nan=False)+"\n")
            print(f"trial {index+1}/{args.runs}: {summary['trials'][-1]['overall']}", flush=True)
            # A runner with incomplete cleanup must not overlap the next cold start.
            try:
                result = json.loads(paths[-1].read_text())
                if not cleanup_complete(result):
                    break
            except (OSError, ValueError, TypeError, AttributeError):
                break
    finally:
        summary = summarize(paths, args.runs)
        (out / "repeat_summary.json").write_text(json.dumps(json_safe(summary), indent=2, allow_nan=False)+"\n")
    print(json.dumps(dict(overall=summary["overall"], counts=summary["counts"], pass_rate=summary["pass_rate"])), flush=True)
    return {"PASS": 0, "NOT_SCORED": 0, "FAIL": 1, "INVALID": 2}[summary["overall"]]


if __name__ == "__main__":
    raise SystemExit(main())
