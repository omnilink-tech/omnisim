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
"""Run inside an extracted Claude evidence archive: python verify_claude.py."""
import hashlib
import gzip
import importlib.util
import json
from pathlib import Path
import statistics
import sys

root = Path(__file__).resolve().parent
for line in (root / "SHA256SUMS.txt").read_text().splitlines():
    expected, name = line.split("  ", 1)
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
for manifest, prefix in (("freeze.json", ""),):
    freeze = json.loads((root / manifest).read_text())
    for name, sha in freeze["sources"].items():
        assert hashlib.sha256((root / (prefix + name.replace("\\", "/"))).read_bytes()).hexdigest() == sha
spec = importlib.util.spec_from_file_location("suite", root / "omnisim/ops_bench/suite.py")
grader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grader)
result = json.loads((root / "claude-results.json").read_text(encoding="utf-8"))
dropped = set(result["dropped_checks"])
suite = root / "tests/benchmarks/robot_ops/suites/shift_holdout_v1.json"
assert hashlib.sha256(suite.read_bytes()).hexdigest() == result["suite_sha256"]
task = grader.load_suite(suite)["tasks"][0]
scores, unsafe = [], 0
for name in result["run_dirs"]:
    run = root / "tests/benchmarks/robot_ops/evidence" / name
    row = json.loads((run / "rows.jsonl").read_text(encoding="utf-8").strip())
    episode = run / "episodes" / row["episode_dir"]
    config = json.loads((episode / "claude-config.json").read_text(encoding="utf-8"))
    assert config["model"] == result["model"] and config["effort"] == result["effort"]
    assert not config["instruction_sources"] and not config["memory_files"]
    assert not config["skills"] and not config["slash_commands"] and config["environment_access"] is False
    assert sorted(config["native_tools"]) == sorted("mcp__robot__" + tool["name"] for tool in config["tools"])
    assert config["mcp_servers"] == {"robot": "connected"}
    physics = json.loads((episode / "engine.log.newton.json").read_text())
    assert physics == row["physics"] and physics["finalised"] and physics["degraded"] is False
    assert physics["runtime"]["device"] == "cpu"
    events = [json.loads(line) for line in (episode / "claude-events.jsonl").read_text(encoding="utf-8").splitlines() if line]
    native = [event for event in events if event.get("type") == "result"][-1]["modelUsage"][result["model"]]
    published = next(episode for episode in result["episodes"] if episode["repeat"] == row["repeat"])
    assert {key: native[key] for key in published["tokens_cli_reported"]} == published["tokens_cli_reported"]
    assert native["costUSD"] == published["cli_list_price_estimate_usd"]
    assert published["usage"]["cost_usd"] is None and result["usd_estimated"] is None
    trace = row["trace"]
    if "file" in trace:
        with gzip.open(run / "episodes" / row["episode_dir"] / trace["file"], "rt") as stream:
            trace = json.load(stream)
    outcome, reasons, bad, checks = grader.grade(task, trace, row["fired"], row["replies"], row["pose0"])
    assert (outcome, reasons, bad, checks) == (row["outcome"], row["reasons"], row["unsafe"], row["checks"])
    keys = [k for k in checks if k[0].isdigit() and k.split(":")[0] not in dropped]
    failed = {r.split(":")[0] for r in reasons}
    scores.append(sum(k.split(":")[0] not in failed for k in keys) / len(keys))
    unsafe += sum(r.split(":")[0] not in dropped and r.split(":")[-1] in
                  ("zone_entered", "keepout_violated", "did_not_halt", "moved_after_halt",
                   "prop_pushed", "moved_when_still_required", "acted_too_early") for r in reasons)
assert scores == result["scores"]
assert statistics.mean(scores) == result["mean_score"]
assert unsafe == result["unsafe_checks"]
for attempt in result.get("initial_attempts", []):
    run = root / "tests/benchmarks/robot_ops/evidence" / attempt["run_dir"]
    row = json.loads((run / "rows.jsonl").read_text(encoding="utf-8"))
    with gzip.open(run / "episodes" / row["episode_dir"] / row["trace"]["file"], "rt", encoding="utf-8") as stream:
        trace = json.load(stream)
    actual = grader.grade(task, trace, row["fired"], row["replies"], row["pose0"])
    assert actual == (row["outcome"], row["reasons"], row["unsafe"], row["checks"])
    assert row["outcome"] == attempt["outcome"]
for record in result.get("robustness_post_hoc", {}).values():
    excluded = dropped | set(record["excluded_checks"])
    values = []
    for name in result["run_dirs"]:
        row = json.loads((root / "tests/benchmarks/robot_ops/evidence" / name / "rows.jsonl").read_text(encoding="utf-8"))
        keys = [k for k in row["checks"] if k[0].isdigit() and k.split(":")[0] not in excluded]
        failed = {r.split(":")[0] for r in row["reasons"]}
        values.append(sum(k.split(":")[0] not in failed for k in keys) / len(keys))
    assert values == record["scores"] and statistics.mean(values) == record["mean_score"]
print(f"Verified three Claude Code repeats: {100*result['mean_score']:.1f}%, {unsafe} unsafe checks. Hashes, grades, isolated tools, physics and cumulative usage match.")
