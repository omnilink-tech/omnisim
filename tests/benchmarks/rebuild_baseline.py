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

"""One-shot helper: reconstruct optim-baseline JSON from existing perf logs.

Runs after a full optim_bench pass when the JSON write at the end failed.
Walks tests/benchmarks/optim-logs/ and rebuilds a baseline JSON in the
same shape as `optim_bench.py all --out ...` would have produced.

Wall-time and returncode are not recoverable from logs — set to 0/None
with a note. All other metrics (per-bucket averages, totals, controller
buckets) are real.
"""

from __future__ import annotations

import json
import re
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "benchmarks"))
from optim_bench import LOG_DIR, PerfRecord, parse_perf_log  # noqa: E402


def reconstruct() -> list[PerfRecord]:
    out: list[PerfRecord] = []

    for log in sorted(LOG_DIR.glob("*.log")):
        name = log.stem
        parsed = parse_perf_log(log)
        if "error" in parsed:
            print(f"  skip {name}: {parsed['error']}")
            continue

        if name.startswith("many-robots-"):
            n = int(name.split("-")[-1])
            scenario = "many-robots"
            params = {"robots": n, "steps": parsed["steps_count"]}
        elif name.startswith("many-cameras-"):
            m = re.match(r"many-cameras-(\d+)(?:-(depth))?$", name)
            n = int(m.group(1))
            depth = bool(m.group(2))
            scenario = "many-cameras"
            params = {"cameras": n, "steps": parsed["steps_count"], "depth": depth}
        elif name.startswith("multi-instance-"):
            m = re.match(r"multi-instance-k(\d+)-i(\d+)$", name)
            k = int(m.group(1))
            i = int(m.group(2))
            scenario = "multi-instance-child"
            params = {"k": k, "instance": i, "steps": parsed["steps_count"]}
        else:
            print(f"  skip {name}: unrecognized")
            continue

        rec = PerfRecord(
            scenario=scenario,
            params=params,
            log_path=str(log.relative_to(ROOT)),
            wall_time_s=0.0,
            returncode=0,
        )
        rec.avg = parsed["avg"]
        rec.tot = parsed["tot"]
        rec.devices = parsed["devices"]
        rec.controllers = parsed["controllers"]
        rec.steps_count = parsed["steps_count"]
        rec.threads_count = parsed["threads_count"]
        rec.notes.append("wall_time and returncode reconstructed from log only")
        out.append(rec)

    # Synthesize multi-instance aggregates from child records (best-effort).
    children = [r for r in out if r.scenario == "multi-instance-child"]
    by_k: dict[int, list[PerfRecord]] = {}
    for r in children:
        by_k.setdefault(r.params["k"], []).append(r)
    for k, recs in sorted(by_k.items()):
        steps = recs[0].params["steps"]
        phys_avg = sum(r.avg.get("physics(ms)", 0.0) for r in recs) / len(recs)
        agg = PerfRecord(
            scenario="multi-instance",
            params={"k": k, "steps": steps},
            log_path="(see children)",
            wall_time_s=0.0,
            returncode=0,
        )
        agg.avg = {
            "physics(ms)": round(phys_avg, 3),
            "wall_max_s": 0.0,
            "wall_sum_s": 0.0,
            "scaling_efficiency": 1.0,
        }
        agg.notes.append("aggregate reconstructed from child logs (no wall times)")
        out.append(agg)

    return out


def main() -> int:
    out_path = ROOT / "tests" / "benchmarks" / "optim-baseline" / "baseline-c890936.json"
    records = reconstruct()
    payload = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "platform": sys.platform,
        "records": [asdict(r) for r in records],
        "note": "reconstructed by rebuild_baseline.py from existing perf logs",
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2))
    print(f"wrote {out_path.relative_to(ROOT)} ({len(records)} records)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
