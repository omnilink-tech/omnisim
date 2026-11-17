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
"""Regrade and package the retrospective Claude Code extension (mirrors build_codex_extension.py)."""
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

# Scored repeats; an infrastructure ERROR repeat would be rerun once and listed in INITIAL.
RUNS = [f"claude-shift-v1-20260930-r{i}" for i in range(3)]
INITIAL: list = []
SUITE = OPS / "suites/shift_holdout_v1.json"
DROP = {"5", "10"}
MODEL, EFFORT = "claude-opus-5-5", "high"
FREEZE = REPO / "tmp/claude-shift/freeze.json"


def _public_events(data):
    """Drop account quota notices and local paths; keep every model and tool event."""
    events = [json.loads(line) for line in data.decode("utf-8").splitlines() if line]
    events = [e for e in events if e.get("type") != "rate_limit_event"]
    for e in events:
        for key in ("memory_paths", "messaging_socket_path", "powershell_path", "cwd"):
            e.pop(key, None)
    return ("\n".join(json.dumps(e, ensure_ascii=False) for e in events) + "\n").encode("utf-8")


def build(destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    A.DROPPED.clear()
    A.DROPPED.update(DROP)
    task = load_suite(SUITE)["tasks"][0]
    rows, episodes = [], []
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
        config = json.loads((run / "episodes" / row["episode_dir"] / "claude-config.json").read_text(encoding="utf-8"))
        assert config["model"] == MODEL and config["effort"] == EFFORT
        assert not config["instruction_sources"] and not config["memory_files"]
        assert config["environment_access"] is False and not config["skills"]
        assert sorted(config["native_tools"]) == sorted("mcp__robot__" + t["name"] for t in config["tools"])
        rows.append(row)
        assert len([k for k in checks if k[0].isdigit() and k.split(":")[0] not in DROP]) == 33
        # Claude Code's own cumulative accounting, from the last result event. The
        # adapter's stream count ("usage.tokens") reads partial per-message usage
        # snapshots and undercounts output; it is kept as recorded, not used here.
        events = [json.loads(l) for l in (run / "episodes" / row["episode_dir"] / "claude-events.jsonl")
                  .read_text(encoding="utf-8").splitlines() if l.strip()]
        results = [e for e in events if e.get("type") == "result"]
        model_usage = results[-1]["modelUsage"][MODEL]
        cli_tokens = {k: model_usage[k] for k in ("inputTokens", "cacheCreationInputTokens",
                      "cacheReadInputTokens", "outputTokens", "thinkingTokens")}
        episodes.append({"repeat": row["repeat"], "score": A.score(row),
                         "unsafe_checks": A.unsafe_count(row), "unsafe_episode": A.unsafe_count(row) > 0,
                         "failed_checks": [r for r in row["reasons"] if r.split(":")[0] not in DROP],
                         "worst_real_time_window": A.worst_window({"trace": load_trace(row, run)}),
                         "usage": row.get("claude_usage"), "tokens_cli_reported": cli_tokens,
                         "cli_list_price_estimate_usd": model_usage.get("costUSD"),
                         "native_turns": {s: sum(e.get("subtype") == s for e in results)
                                          for s in sorted({e.get("subtype") for e in results})},
                         "interrupts_sent": sum(e.get("kind") == "interrupt_sent" for e in events),
                         "interrupt_races": sum(e.get("kind") == "interrupt_race" for e in events),
                         "elapsed_s": row["elapsed_s"],
                         "runtime": config["runtime"]["claude_code_version"]})
    assert sorted(e["repeat"] for e in episodes) == [0, 1, 2]
    scores = [e["score"] for e in sorted(episodes, key=lambda e: e["repeat"])]
    robustness = {}
    for label, extra in (
        ("without_keyword_reply_checks", {str(i) for i, c in enumerate(task["checks"])
          if c["type"] == "reply" and ("any_of" in c or "none_of" in c or c.get("question"))}),
        ("without_any_reply_check", {str(i) for i, c in enumerate(task["checks"]) if c["type"] == "reply"})):
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
            "errors": {k: v["error"] for k, v in row["replies"].items() if v.get("error")},
            "excluded_from_primary": "Infrastructure ERROR; rerun once per CLAUDE_EXTENSION.md."})
    codex = json.loads((OPS / "evidence/CODEX_SHIFT_V1_RESULTS.json").read_text(encoding="utf-8"))
    result = {"benchmark": "Warehouse shift v1 — retrospective Claude Code extension",
              "suite_sha256": hashlib.sha256(SUITE.read_bytes()).hexdigest(),
              "configuration": "claude_full",
              "runtime": f"Claude Code {episodes[0]['runtime']} (headless, stream-JSON)",
              "model": MODEL, "effort": EFFORT, "repeats": 3,
              "checks_scored": 33, "dropped_checks": sorted(DROP), "scores": scores,
              "mean_score": statistics.mean(scores),
              "unsafe_checks": sum(e["unsafe_checks"] for e in episodes),
              "unsafe_episodes": sum(e["unsafe_episode"] for e in episodes),
              "usd_estimated": None, "episodes": episodes, "robustness_post_hoc": robustness,
              "run_dirs": RUNS, "initial_attempts": initial_attempts,
              "codex_reference": {"model": codex["model"], "scores": codex["scores"],
                                  "mean_score": codex["mean_score"], "unsafe_checks": codex["unsafe_checks"],
                                  "robustness_post_hoc": {k: v["mean_score"] for k, v in
                                                          codex["robustness_post_hoc"].items()}},
              "retrospective": True, "simulation_only": True,
              "comparison_scope": "Original seven configurations used Gemini 3.5 Flash on September 27. Codex (gpt-6.1-sol) and Claude Code (claude-opus-5-5) ran on September 30, each as three concurrent simulators, on the same machine and engine binary. Codex and Claude differ in model AND native runtime, so this is a system comparison, not a model-controlled or fresh blind test. The adapter developer for the Claude arm was itself a Claude Code session on the same model. No superiority claim is added.",
              "billing": "Signed-in Claude subscription. Tokens recorded; billed dollar cost unavailable, not zero. cli_reported_cost_usd is Claude Code's own list-price estimate, not a bill.",
              "usage_scope": "Model request counts are distinct assistant message ids in the native stream; interrupted calls may lack final usage; not a billed request total. Token totals are tokens_cli_reported (Claude Code's own cumulative modelUsage). The adapter-recorded usage.tokens undercounts output (partial stream snapshots) and is superseded for reporting; the adapter was frozen and not changed."}
    freeze = json.loads(FREEZE.read_text())
    for path, sha in freeze["sources"].items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == sha, f"Source changed after freeze: {path}"
    text = json.dumps(result, indent=2) + "\n"
    (destination / "claude-results.json").write_text(text, encoding="utf-8")
    (OPS / "evidence/CLAUDE_SHIFT_V1_RESULTS.json").write_text(text, encoding="utf-8")
    files = {"claude-results.json": text.encode("utf-8"),
             "freeze.json": json.dumps(freeze, indent=2).encode(),
             "machine.txt": (REPO / "tmp/claude-shift/machine.txt").read_bytes(),
             "LICENSE": (REPO / "LICENSE").read_bytes()}
    for rel in freeze["sources"]:
        files[rel] = (REPO / rel).read_bytes()
    for rel in ("omnisim/control_bench/engine.py", "omnisim/paths.py", "omnisim/ops_bench/__init__.py",
                "tests/benchmarks/robot_ops/analyze_shift.py", "tests/benchmarks/robot_ops/analyze_holdout.py"):
        files[rel] = (REPO / rel).read_bytes()
    for name in RUNS + INITIAL:
        run = OPS / "evidence" / name
        for file in run.rglob("*"):
            if not file.is_file() or file.name not in (
                    "manifest.json", "rows.jsonl", "trace.json.gz", "claude-config.json",
                    "claude-events.jsonl", "engine.log.newton.json"):
                continue
            data = file.read_bytes()
            if file.name == "claude-events.jsonl":
                data = _public_events(data)
            if file.name == "rows.jsonl":
                redacted = []
                for line in data.decode().splitlines():
                    row = json.loads(line)
                    if "OMNILINK_REMOTE_USER_KEY" in row.get("env", {}):
                        row["env"]["OMNILINK_REMOTE_USER_KEY"] = "<redacted>"
                    redacted.append(json.dumps(row, ensure_ascii=False))
                data = ("\n".join(redacted) + "\n").encode()
            files[str(file.relative_to(REPO)).replace("\\", "/")] = data
    # Reject accidental credential or account inclusion.
    key = Path("O:/omnilink-keys/omni_key.txt").read_bytes().strip()
    assert key and all(key not in content for content in files.values())
    assert all(b"@gmail.com" not in content for content in files.values())
    files["verify_claude.py"] = (OPS / "publish/verify_claude.py").read_bytes()
    files["REDACTIONS.txt"] = (b"Rate-limit (account quota) events and local cwd/memory/socket paths removed from "
                               b"public Claude events. Account details were never written to claude-config.json. "
                               b"OMNILINK_REMOTE_USER_KEY redacted if present. Billed cost is unavailable, not zero.\n")
    sums = "\n".join(f"{hashlib.sha256(content).hexdigest()}  {name}" for name, content in sorted(files.items())) + "\n"
    with zipfile.ZipFile(destination / "claude-evidence.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name, content in files.items():
            archive.writestr(name, content)
        archive.writestr("SHA256SUMS.txt", sums)
    archive_sha = hashlib.sha256((destination / "claude-evidence.zip").read_bytes()).hexdigest()
    (destination / "CLAUDE-SHA256SUMS.txt").write_text(f"{archive_sha}  claude-evidence.zip\n")
    print(json.dumps({"mean": result["mean_score"], "scores": scores, "unsafe": result["unsafe_checks"],
                      "robustness": {k: v["mean_score"] for k, v in robustness.items()},
                      "episodes": episodes, "archive_sha256": archive_sha}, indent=1))


if __name__ == "__main__":
    build(sys.argv[1])
