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
"""Regrade and package the retrospective Codex extension, preserving v1 evidence."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import statistics
import sys
import zipfile

REPO = Path(__file__).resolve().parents[4]
OPS = REPO / "tests/benchmarks/robot_ops"
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(OPS))
import analyze_shift as A
from omnisim.ops_bench.runner import load_trace
from omnisim.ops_bench.suite import grade, load_suite

INITIAL = [f"codex-shift-v1-20260930-r{i}" for i in range(3)]
RUNS = [f"codex-shift-v1-20260930-corrected-r{i}" for i in range(3)]
SUITE = OPS / "suites/shift_holdout_v1.json"
DROP = {"5", "10"}


def build(destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    A.DROPPED.clear()
    A.DROPPED.update(DROP)
    task = load_suite(SUITE)["tasks"][0]
    rows = []
    episodes = []
    for name in RUNS:
        run = OPS / "evidence" / name
        rr = [json.loads(l) for l in (run / "rows.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
        if len(rr) != 1 or rr[0]["outcome"] == "ERROR":
            raise ValueError(f"Incomplete/error repeat: {name}")
        row = rr[0]
        sidecar = json.loads((run / "episodes" / row["episode_dir"] / "engine.log.newton.json").read_text())
        assert sidecar["finalised"] and sidecar["degraded"] is False
        assert sidecar["runtime"]["device"] == "cpu"
        assert sidecar == row["physics"]
        outcome, reasons, unsafe, checks = grade(task, load_trace(row, run), row["fired"],
                                                row["replies"], row["pose0"])
        assert (outcome, reasons, unsafe, checks) == (row["outcome"], row["reasons"], row["unsafe"], row["checks"]), name
        config = json.loads((run / "episodes" / row["episode_dir"] / "codex-config.json").read_text(encoding="utf-8"))
        assert config["model"] == "gpt-6.1-sol" and config["reasoning_effort"] == "high"
        assert not config["instruction_sources"] and config["environment_access"] is False
        rows.append(row)
        assert len([k for k in checks if k[0].isdigit() and k.split(":")[0] not in DROP]) == 33
        episodes.append({"repeat": row["repeat"], "score": A.score(row),
                         "unsafe_checks": A.unsafe_count(row), "unsafe_episode": A.unsafe_count(row) > 0,
                         "worst_real_time_window": A.worst_window({"trace": load_trace(row, run)}),
                         "usage": row.get("codex_usage"), "elapsed_s": row["elapsed_s"]})
    assert sorted(e["repeat"] for e in episodes) == [0, 1, 2]
    scores = [e["score"] for e in sorted(episodes, key=lambda e: e["repeat"])]
    robustness = {}
    for label, extra in (
        ("without_keyword_reply_checks", {str(i) for i,c in enumerate(task["checks"])
          if c["type"] == "reply" and ("any_of" in c or "none_of" in c or c.get("question"))}),
        ("without_any_reply_check", {str(i) for i,c in enumerate(task["checks"]) if c["type"] == "reply"})):
        A.DROPPED.update(extra)
        values = [A.score(row) for row in rows]
        robustness[label] = {"excluded_checks": sorted(extra, key=int), "scores": values,
                             "mean_score": statistics.mean(values), "post_hoc": True}
        A.DROPPED.clear()
        A.DROPPED.update(DROP)
    initial_attempts = []
    for name in INITIAL:
        run = OPS / "evidence" / name
        row = json.loads((run / "rows.jsonl").read_text(encoding="utf-8"))
        assert grade(task, load_trace(row, run), row["fired"], row["replies"], row["pose0"]) == (
            row["outcome"], row["reasons"], row["unsafe"], row["checks"])
        initial_attempts.append({"run_dir": name, "outcome": row["outcome"],
            "partial_check_score": A.score(row), "unsafe_checks": A.unsafe_count(row),
            "errors": {k: v["error"] for k,v in row["replies"].items() if v.get("error")},
            "excluded_from_primary": "Entire initial wave superseded after documented native interruption transport correction."})
    result = {"benchmark": "Warehouse shift v1 — retrospective Codex extension",
              "suite_sha256": hashlib.sha256(SUITE.read_bytes()).hexdigest(),
              "configuration": "codex_full", "runtime": "Codex app-server 0.159.0",
              "model": "gpt-6.1-sol", "reasoning_effort": "high", "repeats": 3,
              "checks_scored": 33, "dropped_checks": sorted(DROP), "scores": scores,
              "mean_score": statistics.mean(scores),
              "unsafe_checks": sum(e["unsafe_checks"] for e in episodes),
              "unsafe_episodes": sum(e["unsafe_episode"] for e in episodes),
              "usd_estimated": None, "episodes": episodes, "robustness_post_hoc": robustness,
              "run_dirs": RUNS, "initial_attempts": initial_attempts,
              "retrospective": True, "simulation_only": True,
              "comparison_scope": "Original seven configurations used Gemini 3.5 Flash on September 27. Codex ran later with a different model, native runtime and three concurrent simulators; original runs used seven. This is a system comparison, not a model-controlled or fresh blind test. No superiority claim is added.",
              "billing": "Signed-in Codex account. Tokens recorded; billed dollar cost unavailable, not zero.",
              "usage_scope": "Model request counts are observed native token-usage notifications. Interrupted calls may lack final usage; these counts are not a billed request total."}
    freeze = json.loads((REPO / "tmp/codex-shift/freeze.json").read_text())
    for path, sha in freeze["sources"].items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == sha, f"Source changed after freeze: {path}"
    (destination / "codex-results.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    public_freeze = {**freeze, "sources": {p.replace("\\", "/"): sha for p, sha in freeze["sources"].items()}}
    files = {"codex-results.json": (destination / "codex-results.json").read_bytes(),
             "freeze.json": json.dumps(public_freeze, indent=2).encode(),
             "machine.txt": (REPO / "tmp/codex-shift/machine.txt").read_bytes(),
             "LICENSE": (REPO / "LICENSE").read_bytes(),
             "freeze-first.json": (REPO / "tmp/codex-shift/freeze-first.json").read_bytes()}
    for rel in freeze["sources"]:
        files[rel.replace("\\", "/")] = (REPO / rel).read_bytes()
    initial_freeze = json.loads(files["freeze-first.json"])
    for rel, sha in initial_freeze["sources"].items():
        data = (REPO / "tmp/codex-shift/initial-source" / rel).read_bytes()
        assert hashlib.sha256(data).hexdigest() == sha, rel
        files["initial-source/" + rel.replace("\\", "/")] = data
    for rel in ("omnisim/control_bench/engine.py", "omnisim/paths.py", "omnisim/ops_bench/competitors.py",
                "omnisim/ops_bench/__init__.py", "tests/benchmarks/robot_ops/analyze_shift.py",
                "tests/benchmarks/robot_ops/analyze_holdout.py"):
        files[rel] = (REPO / rel).read_bytes()
    for name in RUNS + INITIAL:
        run = OPS / "evidence" / name
        for file in run.rglob("*"):
            if not file.is_file():
                continue
            if file.name not in ("manifest.json", "rows.jsonl", "trace.json.gz",
                                 "codex-config.json", "codex-events.jsonl", "engine.log.newton.json"):
                continue
            data = file.read_bytes()
            if file.name == "codex-events.jsonl":
                # Account quota/credit notifications are unrelated to the test.
                events = [json.loads(line) for line in data.decode("utf-8").splitlines() if line]
                events = [event for event in events if not event.get("method", "").startswith(("account/", "auth/"))]
                data = ("\n".join(json.dumps(event, ensure_ascii=False) for event in events) + "\n").encode("utf-8")
            if file.name == "codex-config.json":
                config = json.loads(data)
                config.get("runtime", {}).pop("codexHome", None)
                data = json.dumps(config, indent=2).encode("utf-8")
            # Same explicit environment redaction as the original public bundle.
            if file.name == "rows.jsonl":
                redacted = []
                for line in data.decode().splitlines():
                    row = json.loads(line)
                    if "OMNILINK_REMOTE_USER_KEY" in row.get("env", {}):
                        row["env"]["OMNILINK_REMOTE_USER_KEY"] = "<redacted>"
                    redacted.append(json.dumps(row, ensure_ascii=False))
                data = ("\n".join(redacted) + "\n").encode()
            files[str(file.relative_to(REPO)).replace("\\", "/")] = data
    # Reject accidental credential inclusion against the explicitly supplied key.
    key = Path("O:/omnilink-keys/omni_key.txt").read_bytes().strip()
    assert key and all(key not in content for content in files.values())
    files["verify_codex.py"] = (OPS / "publish/verify_codex.py").read_bytes()
    files["REDACTIONS.txt"] = b"Account quota/credit notifications and codexHome path removed from public Codex events/config. OMNILINK_REMOTE_USER_KEY redacted if present. Generic runner summaries are omitted: Codex billed cost is unavailable, not zero.\n"
    sums = "\n".join(f"{hashlib.sha256(content).hexdigest()}  {name}" for name, content in sorted(files.items())) + "\n"
    with zipfile.ZipFile(destination / "codex-evidence.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name, content in files.items():
            archive.writestr(name, content)
        archive.writestr("SHA256SUMS.txt", sums)
    archive_sha = hashlib.sha256((destination / "codex-evidence.zip").read_bytes()).hexdigest()
    (destination / "CODEX-SHA256SUMS.txt").write_text(f"{archive_sha}  codex-evidence.zip\n")
    print(json.dumps({"mean": result["mean_score"], "episodes": episodes, "archive_sha256": archive_sha}))


if __name__ == "__main__":
    build(sys.argv[1])
