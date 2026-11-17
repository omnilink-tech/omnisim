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

"""Claude Code lane unit tests -- no Claude Code, no simulator, no repo copy.

Everything staging-shaped runs against SYNTHETIC trees under tmp_path; the
junction tests use real junctions on a dummy target, so the teardown-safety
property ("never delete through a junction") is proven on disposable data
before any real workspace exists.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import subprocess
import time
import sys
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[2]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                  # noqa: E402
from agentbench.cc_lane import containment_guard as guard   # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging  # noqa: E402
from agentbench.cc_lane import run_cc_cell as cell          # noqa: E402
from agentbench import tasks as tasks_mod                  # noqa: E402
from agentbench import sims as sims_mod                    # noqa: E402
from agentbench.agents import external as external_agent   # noqa: E402

WINDOWS = os.name == "nt"
needs_windows = pytest.mark.skipif(not WINDOWS,
                                   reason="junctions are a Windows feature")


# --- include/exclude correctness ---------------------------------------------


SYNTH_FILES = [
    "AGENTS.md",
    "CLAUDE.md",
    "README.md",
    "CHANGELOG.md",                                    # excluded by name
    "docs/guide/x.md",
    "docs/developer/agent-edge-validation-plan.md",    # answer key
    "docs/developer/webots-control-baseline.md",       # answer key
    "docs/developer/harness-latency-2026-07-31.md",    # benchmark internals
    "tests/benchmarks/agentbench/SPEC.md",             # answer key (tree)
    "tests/benchmarks/agentbench/graders/c2_core.py",  # answer key (tree)
    "tests/benchmarks/omnibench/SPEC.md",              # NOT excluded
    "scripts/dev/omnisim_dev.py",
    "social/post_x.py",                                # private tree
    "cloud/runpod/README.md",                          # private tree
    "projects/samples/w.wbt",                          # junctioned dir
    "msys64/mingw64/bin/omnisim-bin.exe",              # junctioned dir
    "lib/controller/x.py",                             # junctioned dir
    "resources/icons/x.png",                           # junctioned dir
]


def test_select_omnisim_files_drops_the_answer_key_and_private_trees():
    copied, excluded = staging.select_omnisim_files(SYNTH_FILES)
    assert "AGENTS.md" in copied
    assert "docs/guide/x.md" in copied
    assert "tests/benchmarks/omnibench/SPEC.md" in copied
    assert "scripts/dev/omnisim_dev.py" in copied
    # the answer key and private trees are EXCLUDED (recorded), junctioned
    # dirs are neither copied nor listed as exclusions (they arrive by link)
    for key in ("docs/developer/agent-edge-validation-plan.md",
                "docs/developer/webots-control-baseline.md",
                "docs/developer/harness-latency-2026-07-31.md",
                "CHANGELOG.md",
                "tests/benchmarks/agentbench/SPEC.md",
                "tests/benchmarks/agentbench/graders/c2_core.py",
                "social/post_x.py", "cloud/runpod/README.md"):
        assert key not in copied
        assert key in excluded
    for junctioned in ("projects/samples/w.wbt", "lib/controller/x.py",
                       "msys64/mingw64/bin/omnisim-bin.exe",
                       "resources/icons/x.png"):
        assert junctioned not in copied
        assert junctioned not in excluded


def _synth_repo(tmp_path):
    repo = tmp_path / "repo"
    for rel in SYNTH_FILES:
        p = repo / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("content of %s\n" % rel, encoding="utf-8")
    return repo


def test_build_template_omits_answer_key_files_on_disk(tmp_path):
    repo = _synth_repo(tmp_path)
    root = tmp_path / "root"
    tpl = staging.build_omnisim_template(
        root, repo=repo, file_list=SYNTH_FILES,
        junction_dirs=())                 # no junctions on the synthetic tree
    assert (tpl / "AGENTS.md").is_file()
    assert (tpl / "tests" / "benchmarks" / "omnibench" / "SPEC.md").is_file()
    assert not (tpl / "tests" / "benchmarks" / "agentbench").exists()
    assert not (tpl / "docs" / "developer"
                / "agent-edge-validation-plan.md").exists()
    assert not (tpl / "docs" / "developer"
                / "webots-control-baseline.md").exists()
    assert not (tpl / "CHANGELOG.md").exists()
    assert not (tpl / "social").exists()
    man = json.loads((root / "templates" / "omnisim.manifest.json")
                     .read_text(encoding="utf-8"))
    assert "tests/benchmarks/agentbench/SPEC.md" in man["excluded"]
    # with junction_dirs=() the 4 runtime-dir files are plain copies, so:
    # 18 synthetic files - 8 excluded = 10 copied
    assert man["included_file_count"] == len(SYNTH_FILES) - 8
    assert len(man["excluded"]) == 8
    # the manifest hash matches the copied list, and disclosure is recorded
    assert man["filelist_sha256"]
    assert any("AGENTS.md" in d for d in man["known_disclosures"])


# --- junctions: creation, instantiation, teardown safety -----------------------


@needs_windows
def test_teardown_severs_junctions_and_never_deletes_through_them(tmp_path):
    target = tmp_path / "real_target"
    (target / "sub").mkdir(parents=True)
    keep = target / "sub" / "keep.txt"
    keep.write_text("must survive", encoding="utf-8")

    ws = tmp_path / "ws"
    (ws / "plain").mkdir(parents=True)
    (ws / "plain" / "f.txt").write_text("x", encoding="utf-8")
    staging.make_junction(ws / "junc", target)
    assert (ws / "junc" / "sub" / "keep.txt").is_file()  # visible through it

    severed = staging.teardown_workspace(ws)
    assert not ws.exists()
    assert keep.is_file(), "teardown deleted THROUGH the junction"
    assert any(str(Path(s)).endswith("junc") for s in severed)


@needs_windows
def test_teardown_handles_nested_junctions(tmp_path):
    target = tmp_path / "t2"
    target.mkdir()
    (target / "data.txt").write_text("survive", encoding="utf-8")
    ws = tmp_path / "ws2"
    (ws / "a" / "b").mkdir(parents=True)
    staging.make_junction(ws / "a" / "b" / "deep_junc", target)
    staging.teardown_workspace(ws)
    assert not ws.exists()
    assert (target / "data.txt").is_file()


def test_teardown_refuses_the_repo_and_ancestors():
    with pytest.raises(ValueError):
        staging.teardown_workspace(staging.REPO)
    with pytest.raises(ValueError):
        staging.teardown_workspace(staging.REPO.parent)


@needs_windows
def test_instantiate_recreates_junctions_rather_than_copying(tmp_path):
    # a synthetic template + manifest with one junction spec
    root = tmp_path / "root"
    tpl = root / "templates" / "omnisim"
    (tpl / "docs").mkdir(parents=True)
    (tpl / "docs" / "a.md").write_text("a", encoding="utf-8")
    target = tmp_path / "big_runtime"
    target.mkdir()
    (target / "huge.bin").write_text("pretend 1 GB", encoding="utf-8")
    (root / "templates" / "omnisim.manifest.json").write_text(json.dumps({
        "sim": "omnisim", "filelist_sha256": "x", "known_disclosures": [],
        "junctions": [{"name": "msys64", "target": str(target),
                       "kind": "junction"}]}), encoding="utf-8")

    inst = root / "instances" / "cell0"
    man = staging.instantiate(tpl, inst)
    assert (inst / "docs" / "a.md").is_file()
    assert staging.is_link(inst / "msys64")
    assert (inst / "msys64" / "huge.bin").is_file()
    assert man["links_materialised"][0]["name"] == "msys64"
    # fresh-per-cell: a second instantiation at the same path is refused
    with pytest.raises(FileExistsError):
        staging.instantiate(tpl, inst)
    staging.teardown_workspace(inst)
    assert (target / "huge.bin").is_file()


@needs_windows
def test_copy_tree_refuses_a_template_containing_a_link(tmp_path):
    tpl = tmp_path / "tpl"
    tpl.mkdir()
    target = tmp_path / "t"
    target.mkdir()
    staging.make_junction(tpl / "sneaky", target)
    with pytest.raises(RuntimeError):
        staging._copy_tree_no_links(tpl, tmp_path / "dst")


# --- transcript tool-call counter ----------------------------------------------


def test_count_tool_calls_counts_assistant_tool_use_blocks(tmp_path):
    lines = [
        {"type": "user", "message": {"role": "user", "content": "hi"}},
        {"type": "assistant", "message": {"role": "assistant", "content": [
            {"type": "text", "text": "looking"},
            {"type": "tool_use", "id": "1", "name": "Bash", "input": {}}]}},
        {"type": "user", "message": {"role": "user", "content": [
            {"type": "tool_result", "tool_use_id": "1"}]}},
        {"type": "assistant", "message": {"role": "assistant", "content": [
            {"type": "tool_use", "id": "2", "name": "Edit", "input": {}},
            {"type": "tool_use", "id": "3", "name": "Read", "input": {}}]}},
        {"type": "assistant", "message": {"role": "assistant", "content": [
            {"type": "text", "text": "done"}]}},
    ]
    p = tmp_path / "session.jsonl"
    p.write_text("\n".join(json.dumps(l) for l in lines) + "\nnot json\n",
                 encoding="utf-8")
    n, reason = cell.count_tool_calls(p)
    assert n == 3
    assert reason is None


def test_count_tool_calls_is_null_with_reason_when_unreadable(tmp_path):
    n, reason = cell.count_tool_calls(tmp_path / "absent.jsonl")
    assert n is None and "unreadable" in reason
    empty = tmp_path / "empty.jsonl"
    empty.write_text("garbage\n", encoding="utf-8")
    n, reason = cell.count_tool_calls(empty)
    assert n is None and "no parseable" in reason


# --- env scrub -------------------------------------------------------------------


def test_scrub_env_removes_nesting_and_credential_vars():
    base = {
        "PATH": "keep", "CLAUDE_CONFIG_DIR": "keep-auth",
        "CLAUDECODE": "1", "CLAUDE_CODE_ENTRYPOINT": "cli",
        "CLAUDE_CODE_SSE_PORT": "123",
        "ANTHROPIC_API_KEY": "sk-ant-x", "ANTHROPIC_MODEL": "x",
        "AGENTBENCH_SIM": "webots", "AGENTBENCH_EXTERNAL_ARTIFACT": "x",
        "OMNISIM_LOG_PATH": "x", "OMNISIM_FORCE_ODE": "1",
        "WEBOTS_HOME": "stale",
    }
    env, removed = cell.scrub_env(base)
    assert env["PATH"] == "keep"
    assert env["CLAUDE_CONFIG_DIR"] == "keep-auth", \
        "scrubbing the auth locator breaks login, not isolation"
    for k in ("CLAUDECODE", "CLAUDE_CODE_ENTRYPOINT", "CLAUDE_CODE_SSE_PORT",
              "ANTHROPIC_API_KEY", "ANTHROPIC_MODEL", "AGENTBENCH_SIM",
              "AGENTBENCH_EXTERNAL_ARTIFACT", "OMNISIM_LOG_PATH",
              "OMNISIM_FORCE_ODE", "WEBOTS_HOME"):
        assert k not in env
        assert k in removed


# --- row merge --------------------------------------------------------------------


GRADER_ROW = {
    "task": "C2_fall_through_floor", "sim": "omnisim",
    "condition": "claude_code", "outcome": "PASS",
    "assertions": {"C2.1": True}, "failed_assertion": None,
    "measurements": {"rest_z_m": 0.25},
    "agent": {"model": "scripted:external", "kind": "external"},
    "agent_artifacts": {"external_label": "claude_code"},
    # the external STUB's own numbers -- one copy call, near-zero clock --
    # which must never survive as the agent's
    "metrics": {"t_agent_s": 0.002, "t_total_s": 61.0, "turns": 1,
                "tool_calls": 1, "tokens_in": None, "tokens_out": None,
                "tokens_cache_read": None, "tokens_cache_write": None,
                "usd": None},
}

CC = {
    "num_turns": 24, "tool_calls": 31, "tool_calls_reason": None,
    "total_cost_usd": 1.23, "t_agent_s": 301.5,
    "tokens_in": 1000, "tokens_out": 2000,
    "tokens_cache_read": 50000, "tokens_cache_write": 700,
    "model": "claude-x-9-20260101", "session_id": "abc",
    "claude_code_version": "2.1.179 (Claude Code)",
    "permission_mode": "dangerously-skip-permissions",
    "transcript": "C:/x/abc.jsonl", "cell_wall_s": 320.0,
    "duration_ms": 301500, "duration_api_ms": 250000,
    "is_error": False, "subtype": "success", "cli_command": "claude ...",
}


def test_merge_overwrites_agent_metrics_and_keeps_the_verdict():
    row = cell.merge_cc_metrics(GRADER_ROW, CC)
    m = row["metrics"]
    assert m["turns"] == 24 and m["tool_calls"] == 31
    assert m["usd"] == 1.23 and m["t_agent_s"] == 301.5
    assert m["tokens_cache_write"] == 700
    assert m["t_total_s"] == 61.0, "the grader's own clock is not touched"
    assert row["outcome"] == "PASS"
    assert row["measurements"] == {"rest_z_m": 0.25}
    assert row["agent"]["model"] == "claude-x-9-20260101"
    assert row["agent"]["kind"] == "claude_code"
    art = row["agent_artifacts"]
    assert art["metrics_source"] == "claude_code_headless_json"
    assert art["condition"] == "claude_code"
    assert art["external_label"] == "claude_code"     # original rides along
    assert art["claude_code"]["permission_mode"] \
        == "dangerously-skip-permissions"
    # the input row was not mutated
    assert GRADER_ROW["metrics"]["tool_calls"] == 1


def test_merge_nulls_unmeasured_agent_metrics_instead_of_keeping_stub():
    cc = dict(CC, tool_calls=None,
              tool_calls_reason="transcript not found")
    row = cell.merge_cc_metrics(GRADER_ROW, cc)
    assert row["metrics"]["tool_calls"] is None, \
        "the stub's copy-call count must not impersonate the agent"
    assert row["agent_artifacts"]["claude_code"]["tool_calls_reason"] \
        == "transcript not found"


def test_cc_metrics_from_result_never_invents():
    cc = cell.cc_metrics_from_result(
        {"num_turns": 5, "usage": {}, "modelUsage": {}},
        tool_calls=None, tool_calls_reason="r", version="v",
        permission_mode="m", transcript=None, cell_wall_s=1.0,
        cli_command="c")
    assert cc["num_turns"] == 5
    assert cc["total_cost_usd"] is None
    assert cc["tokens_in"] is None
    assert cc["t_agent_s"] is None
    assert cc["model"] is None


# --- artifact discovery -------------------------------------------------------------


def test_discover_artifact_prefers_session_modified_worlds(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    staged = ws / "fall_through.wbt"
    staged.write_text("broken", encoding="utf-8")
    old = 1_000_000_000.0
    os.utime(staged, (old, old))
    (ws / "docs").mkdir()
    decoy = ws / "docs" / "sample.wbt"
    decoy.write_text("decoy", encoding="utf-8")
    os.utime(decoy, (old, old))

    start = 2_000_000_000.0
    # nothing modified -> fall back to the staged world, honestly labelled
    art, rule = cell.discover_artifact(ws, [str(staged)], start)
    assert art == staged and "falling back" in rule

    fixed = ws / "fixed.wbt"
    fixed.write_text("fixed", encoding="utf-8")
    os.utime(fixed, (start + 10, start + 10))
    injected = ws / "_agentbench_x.wbt"
    injected.write_text("injected", encoding="utf-8")
    os.utime(injected, (start + 99, start + 99))
    art, rule = cell.discover_artifact(ws, [str(staged)], start)
    assert art == fixed, "injected copies are never candidates"

    # the staged deliverable, once the session modified it, outranks NEWER
    # verification scratch (the first webots smoke cell collected the
    # session's own labelled-broken control copy under the newest-first rule)
    os.utime(staged, (start + 5, start + 5))
    control = ws / "verify" / "broken.wbt"
    control.parent.mkdir()
    control.write_text("control copy", encoding="utf-8")
    os.utime(control, (start + 50, start + 50))
    art, rule = cell.discover_artifact(ws, [str(staged)], start)
    assert art == staged, "the modified deliverable outranks scratch: %s" % art
    assert "deliverable" in rule


@needs_windows
def test_discover_artifact_junction_worlds_gated_by_session_mtime(tmp_path):
    # Contract updated 2026-08-01 (A1 r0-r2: agents wrote their deliverable
    # THROUGH the projects/ junction and the link-blind walk lost all
    # three): a .wbt behind a junction is collected as a FALLBACK when the
    # session wrote it (mtime after session start), and stays invisible
    # otherwise -- the mtime gate is what keeps arbitrary repo files out.
    import os as _os
    import time as _time
    target = tmp_path / "repo_projects"
    target.mkdir()
    trap = target / "trap.wbt"
    trap.write_text("x", encoding="utf-8")
    ws = tmp_path / "ws"
    ws.mkdir()
    staging.make_junction(ws / "projects", target)
    start = _time.time() + 5        # session starts AFTER the trap's mtime
    _os.utime(trap, (start - 100, start - 100))
    art, rule = cell.discover_artifact(ws, [], start)
    assert art is None, ("a pre-session .wbt behind a junction must stay "
                         "invisible: %s" % art)
    # ...but one the session wrote through the junction is recovered
    authored = target / "authored.wbt"
    authored.write_text("y", encoding="utf-8")
    _os.utime(authored, (start + 10, start + 10))
    art2, rule2 = cell.discover_artifact(ws, [], start)
    assert art2 is not None and art2.name == "authored.wbt"
    assert "junction" in rule2


# --- answer-key redaction (the 2026-08-01 ruling) ---------------------------------


def test_redaction_removes_task_id_sentences_keeps_capability_docs():
    text = (
        "This is the supported headless contract. **That is ALL a bare PASS "
        "means.** Measured on AgentBench task `C2_fall_through_floor`: a "
        "world whose floor Solid has no boundingObject printed PASS. A bare "
        "PASS cannot certify a physics fix.\n"
        "\n"
        "**When the point of the run is to certify physical behaviour, add "
        "`--fail-on-runaway`:** it samples every top-level dynamic body's "
        "pose and FAILS when one has left the world.\n")
    new, reds = staging.redact_markdown(text)
    flat = " ".join(new.split())          # rewrap-insensitive comparison
    assert "C2_fall_through_floor" not in new
    assert "AgentBench" not in new
    assert "**That is ALL a bare PASS means.**" in flat
    assert "A bare PASS cannot certify a physics fix." in flat
    assert "--fail-on-runaway" in new, "capability docs must survive"
    assert len(reds) == 1 and reds[0]["kind"] == "paragraph"
    assert "C2_fall_through_floor" in reds[0]["before"]
    assert "C2_fall_through_floor" not in reds[0]["after"]


def test_redaction_drops_matching_table_rows_whole():
    text = ("| a | b |\n"
            "|---|---|\n"
            "| **Ask about agents** | **AgentBench** describes the tasks |\n"
            "| **Benchmark physics** | OmniBench stays |\n")
    new, reds = staging.redact_markdown(text)
    assert "AgentBench" not in new
    assert "OmniBench stays" in new
    assert [r["kind"] for r in reds] == ["table_row"]
    assert reds[0]["after"] == ""


def test_redaction_strips_clause_not_sentence_when_possible():
    text = ("A body that fell and landed, one descending steadily, and one "
            "legally mid-air all pass; on the same C2 pair it FAILs the "
            "broken world and PASSes both fixes.\n")
    new, reds = staging.redact_markdown(text)
    assert "C2 pair" not in new
    assert "all pass" in new, "the non-matching clause survives"
    assert len(reds) == 1


def test_redaction_strips_matching_parenthetical_only():
    text = ("Use `--fail-on-runaway` when the claim is about the physics "
            "rather than the load (see 3b: both variants of the C2 "
            "fall-through task passed the log-only lane identically).\n")
    new, reds = staging.redact_markdown(text)
    assert "C2" not in new
    assert "--fail-on-runaway" in new
    assert len(reds) == 1


def test_redaction_never_touches_robot_or_baseline_short_ids():
    text = ("The Unitree B2 walks +95 m under Newton. Baseline item C1 "
            "(the wgpu default flip) is done; C2 (surface sign-off) too. "
            "The A1 root-cause scaffolding stays. agent-benchmarks.md is a "
            "different doc.\n")
    new, reds = staging.redact_markdown(text)
    assert new == text
    assert reds == []


def test_redaction_bullets_are_independent_blocks():
    text = ("- first bullet mentions AgentBench task ids like "
            "B1_overlap_audit everywhere so it goes\n"
            "- second bullet is about the Unitree B2 robot and stays\n")
    new, reds = staging.redact_markdown(text)
    assert "B1_overlap_audit" not in new
    assert "second bullet is about the Unitree B2 robot and stays" in new
    assert len(reds) == 1


# --- concurrency: locks, semaphore, rate-limit recognition -------------------------


def test_file_lock_exclusive_and_release(tmp_path):
    a = concurrency.FileLock(tmp_path / "x.lock", lane="A")
    b = concurrency.FileLock(tmp_path / "x.lock", lane="B")
    assert a.try_acquire()
    assert not b.try_acquire(), "second holder must be refused"
    a.release()
    assert b.try_acquire()
    b.release()


def test_stale_lock_is_reclaimed(tmp_path):
    p = tmp_path / "stale.lock"
    p.write_text(json.dumps({"pid": 999999999, "lane": "dead"}),
                 encoding="utf-8")
    lock = concurrency.FileLock(p, lane="A")
    assert lock.try_acquire(), "a dead pid's lock must be reclaimable"
    lock.release()


def test_engine_slots_semaphore_and_others_active(tmp_path):
    s1 = concurrency.EngineSlots(tmp_path, slots=2, lane="A")
    s2 = concurrency.EngineSlots(tmp_path, slots=2, lane="B")
    s3 = concurrency.EngineSlots(tmp_path, slots=2, lane="C")
    assert s1.try_acquire("one")
    assert not s1.others_active(), "same pid never counts as another lane"
    assert s2.try_acquire("two")
    assert not s3.try_acquire("three"), "slot 3 of 2 must be refused"
    s1.release()
    assert s3.try_acquire("three now")
    s2.release()
    s3.release()


def test_task_lock_same_task_never_overlaps(tmp_path):
    t1 = concurrency.task_lock(tmp_path, "B1_overlap_audit", lane="A")
    t2 = concurrency.task_lock(tmp_path, "B1_overlap_audit", lane="B")
    other = concurrency.task_lock(tmp_path, "C1_parse_error_fix", lane="B")
    assert t1.try_acquire()
    assert not t2.try_acquire(), "same-task cells must be mutually exclusive"
    assert other.try_acquire(), "different tasks are independent"
    t1.release()
    other.release()


def test_rate_limit_reason_matches_limits_only():
    assert concurrency.rate_limit_reason(
        {"is_error": True, "result": "5-hour usage limit reached"}) \
        is not None
    assert concurrency.rate_limit_reason(
        {"is_error": True, "result": "Overloaded_error from API"}) is not None
    assert concurrency.rate_limit_reason(
        None, "HTTP 429 Too Many Requests") is not None
    # a CLEAN result discussing limits is never deferred
    assert concurrency.rate_limit_reason(
        {"is_error": False,
         "result": "I checked the rate limit docs as asked"}) is None
    # an error that is not limit-shaped is a real failure
    assert concurrency.rate_limit_reason(
        {"is_error": True, "result": "world file not found"}) is None
    assert concurrency.rate_limit_reason(None, "") is None


def test_rate_limit_reason_recognises_the_session_limit_refusal():
    # The REAL payloads (phasew_cc_v1 A1:omnisim r8 / phasew_cc_v1_B
    # A1:webots r7, 2026-08-01): is_error true, subtype "success", U+00B7
    # separator. Both were misclassified as artifact-discovery blockers.
    real = ("You've hit your session limit · resets 7:10pm "
            "(Europe/Berlin)")
    assert concurrency.rate_limit_reason(
        {"is_error": True, "subtype": "success", "result": real}) is not None
    # a replacement-char / mojibake copy of the same refusal still matches
    assert concurrency.rate_limit_reason(
        {"is_error": True,
         "result": "You've hit your session limit � resets 7:10pm"}) \
        is not None
    # wording/separator variants
    for text in ("You've hit your usage limit · resets 3am",
                 "SESSION LIMIT ∙ resets at 19:10",
                 "You've hit your session-limit - resets at 7pm",
                 "usage limit; resets at 07:10"):
        assert concurrency.rate_limit_reason(
            {"is_error": True, "result": text}) is not None, text
    # a CLEAN result carrying the exact same words is never deferred
    assert concurrency.rate_limit_reason(
        {"is_error": False, "result": real}) is None
    # non-limit errors stay real failures
    assert concurrency.rate_limit_reason(
        {"is_error": True, "result": "session crashed: world not found"}) \
        is None


def test_resource_guard_returns_measurement_or_says_unmeasurable():
    ok, detail = concurrency.resource_guard(min_free_ram_gb=0.0,
                                            max_cpu_load_pct=100.0)
    assert ok in (True, False)
    assert ("free_ram_gb" in detail) or detail.get("unmeasurable")


# --- per-task deliverable conventions ------------------------------------------------


def test_task_conventions_cover_every_task():
    """Every task must declare a deliverable convention, or a cell will grade
    the wrong file. This list grows as the robotics tier lands -- an addition
    that forgets its convention fails HERE rather than mid-campaign."""
    from agentbench.agents import external as ext
    world_tasks = set(ext.ARTIFACT_NAME)
    answer_tasks = set(ext.ANSWER_TASKS)
    assert answer_tasks == {"B1_overlap_audit", "B3_measure_and_report"}
    assert world_tasks == {"A1_husky_swarm_10", "B2_subject_in_frame",
                           "C1_parse_error_fix", "C2_fall_through_floor",
                           "R1_lidar_nav", "R2_arm_reach",
                           "R3_pick_and_place",
                           # R4 registered its convention in `1f2c0241a` and
                           # this list was not grown with it, so the lane's
                           # own suite was RED before its first cell ran --
                           # which is the test doing its job, late.
                           "R4_mobile_manipulation"}
    assert not (world_tasks & answer_tasks)
    # run_cc_cell's view of the split is the same object, not a copy
    assert cell.ANSWER_TASKS is ext.ANSWER_TASKS
    assert cell.ARTIFACT_NAME is ext.ARTIFACT_NAME
    assert cell.WORLD_PLUS_ANSWER_TASKS == {"B2_subject_in_frame"}


def test_external_registry_has_all_lane_b_entries():
    from agentbench import agents as agents_pkg
    for task in ("A1_husky_swarm_10", "B1_overlap_audit",
                 "B2_subject_in_frame", "B3_measure_and_report",
                 "C1_parse_error_fix", "C2_fall_through_floor"):
        entry = agents_pkg.get(task, "external")
        assert entry["expect_pass"] is None, \
            "external outcomes are unknown by construction"
        assert entry["expect_failures"] is None


# --- resilient teardown (the 2026-08-01 phasew_cc_v1 crash class) ------------------


def test_teardown_resilient_ok_path_matches_plain_teardown(tmp_path):
    ws = tmp_path / "ws"
    (ws / "d").mkdir(parents=True)
    (ws / "d" / "f.txt").write_text("x", encoding="utf-8")
    out = staging.teardown_workspace_resilient(ws, sleep=lambda s: None)
    assert out["ok"] is True and out["pending"] is None
    assert not ws.exists()


def test_teardown_resilient_survives_a_held_open_file(tmp_path):
    """The crash class: a straggler holds a file open, rmtree raises
    WinError 32 -- the resilient teardown must NOT raise, and must mark the
    leftover for the later sweep instead."""
    ws = tmp_path / "instances" / "cell_x"
    ws.mkdir(parents=True)
    locked = ws / "omnisim_log.txt"
    locked.write_text("held", encoding="utf-8")
    sleeps = []
    with open(locked, "r", encoding="utf-8"):
        if not WINDOWS:
            pytest.skip("an open handle only blocks deletion on Windows")
        out = staging.teardown_workspace_resilient(
            ws, tries=3, backoff_s=0.01, sleep=sleeps.append)
    assert out["ok"] is False, "an open handle must fail the delete"
    assert len(sleeps) == 2, "3 tries -> 2 backoffs"
    assert out["error"]
    # marked for the later sweep: renamed *.pending_delete, or (when even
    # the rename is refused, as Windows does with an open file below) a
    # *.pending_delete.marker sibling
    assert out["pending"] is not None
    assert (str(out["pending"]).endswith(staging.PENDING_DELETE_SUFFIX)
            or str(out["pending"]).endswith(staging.PENDING_MARKER_SUFFIX))
    # ...and the handle is closed now, so the sweep reclaims everything
    swept = staging.sweep_pending_deletes(tmp_path)
    assert swept["failed"] == []
    assert not ws.exists()
    leftovers = [p for p in (tmp_path / "instances").iterdir()]
    assert leftovers == [], "sweep must leave nothing behind: %s" % leftovers


def test_teardown_resilient_keeps_the_repo_guard_fatal():
    with pytest.raises(ValueError):
        staging.teardown_workspace_resilient(staging.REPO)


def test_sweep_pending_deletes_reclaims_dirs_and_markers(tmp_path):
    inst = tmp_path / "instances"
    inst.mkdir()
    gone = inst / ("a" + staging.PENDING_DELETE_SUFFIX)
    (gone / "sub").mkdir(parents=True)
    (gone / "sub" / "f").write_text("x", encoding="utf-8")
    stuck = inst / "b"
    stuck.mkdir()
    (inst / ("b" + staging.PENDING_MARKER_SUFFIX)).write_text(
        json.dumps({"path": str(stuck), "utc": "t"}), encoding="utf-8")
    live = inst / "c_live_instance"
    live.mkdir()
    swept = staging.sweep_pending_deletes(tmp_path)
    assert swept["failed"] == []
    assert not gone.exists() and not stuck.exists()
    assert live.is_dir(), "a live instance is never swept"
    assert not (inst / ("b" + staging.PENDING_MARKER_SUFFIX)).exists()


@needs_windows
def test_sweep_pending_deletes_severs_links_never_deletes_through(tmp_path):
    target = tmp_path / "real"
    target.mkdir()
    keep = target / "keep.txt"
    keep.write_text("must survive", encoding="utf-8")
    inst = tmp_path / "instances"
    pend = inst / ("x" + staging.PENDING_DELETE_SUFFIX)
    pend.mkdir(parents=True)
    staging.make_junction(pend / "msys64", target)
    swept = staging.sweep_pending_deletes(tmp_path)
    assert swept["failed"] == []
    assert not pend.exists()
    assert keep.is_file(), "sweep deleted THROUGH the junction"


# --- the process sweep --------------------------------------------------------------


def test_cmdline_references_matches_path_and_name_only():
    inst = (Path("C:/Users/u/AppData/Local/Temp/agentbench_cc/instances"
                 "/20260801_133251_omnisim_B1") if WINDOWS
            else Path("/tmp/agentbench_cc/instances"
                      "/20260801_133251_omnisim_B1"))
    full = str(inst)
    assert staging.cmdline_references(
        "omnisim-bin.exe --batch %s/worlds/x.wbt" % full, inst)
    # both slash flavours of the full path match
    assert staging.cmdline_references(
        "harness.py --root " + full.replace("\\", "/"), inst)
    if WINDOWS:
        assert staging.cmdline_references(
            "harness.py --root " + full.replace("/", "\\"), inst)
        # Windows paths are case-insensitive
        assert staging.cmdline_references("x " + full.upper(), inst)
    # the unique instance NAME alone matches (covers WSL-translated paths)
    assert staging.cmdline_references(
        "/usr/bin/webots /mnt/c/.../instances/20260801_133251_omnisim_B1/w",
        inst)
    # never a bare image-name match, never another instance's
    assert not staging.cmdline_references("omnisim-bin.exe --batch", inst)
    assert not staging.cmdline_references(
        "python x.py 20260801_999999_omnisim_B1", inst)
    assert not staging.cmdline_references("", inst)


def test_sweep_kills_only_processes_referencing_the_instance(tmp_path):
    import subprocess as sp
    import time as _t
    inst = tmp_path / "instances" / "20990101_000000_omnisim_T1"
    inst.mkdir(parents=True)
    ours = sp.Popen([sys.executable, "-c",
                     "import sys,time; time.sleep(120)", str(inst)])
    bystander = sp.Popen([sys.executable, "-c",
                          "import sys,time; time.sleep(120)",
                          "unrelated_argument"])
    try:
        records = staging.sweep_workspace_processes(inst, grace_s=0.5)
        killed_pids = {r["pid"] for r in records
                       if r.get("action") in ("terminated", "killed")}
        assert ours.pid in killed_pids, records
        assert bystander.pid not in killed_pids
        # the straggler is really gone; the bystander really is not
        deadline = _t.monotonic() + 10
        while ours.poll() is None and _t.monotonic() < deadline:
            _t.sleep(0.1)
        assert ours.poll() is not None, "matched process must be dead"
        assert bystander.poll() is None, "bystander must survive"
        rec = next(r for r in records if r["pid"] == ours.pid)
        assert rec["cmdline"], "each kill is logged with its command line"
    finally:
        for p in (ours, bystander):
            if p.poll() is None:
                p.kill()


# --- junction-artifact hygiene (the A1 r1/r3/r6 poisoning class) ---------------------


def _git(repo, *args):
    subprocess.run(["git", "-C", str(repo), "-c", "core.hooksPath=",
                    "-c", "commit.gpgsign=false"] + list(args),
                   check=True, capture_output=True)


def _synth_git_repo(tmp_path):
    """A tiny git repo standing in for the REAL one the junction points at:
    a tracked world, a tracked controller, and a gitignore."""
    repo = tmp_path / "gitrepo"
    (repo / "projects" / "demos" / "worlds").mkdir(parents=True)
    (repo / "projects" / "demos" / "controllers" / "keep").mkdir(parents=True)
    (repo / ".gitignore").write_text("*.log\n", encoding="utf-8")
    (repo / "projects" / "demos" / "worlds" / "tracked.wbt").write_text(
        "tracked world", encoding="utf-8")
    (repo / "projects" / "demos" / "controllers" / "keep" / "keep.py"
     ).write_text("tracked controller", encoding="utf-8")
    _git(repo, "init", "-q")
    _git(repo, "config", "user.email", "cc@test")
    _git(repo, "config", "user.name", "cc")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-qm", "init")
    return repo


def _put(p, text, mtime):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")
    os.utime(p, (mtime, mtime))
    return p


def test_repo_junction_sweep_preserves_then_deletes_session_files(tmp_path):
    repo = _synth_git_repo(tmp_path)
    worlds = repo / "projects" / "demos" / "worlds"
    ctrls = repo / "projects" / "demos" / "controllers"
    t0, t1 = 2_000_000_000.0, 2_000_000_100.0
    inside = t0 + 50

    authored = _put(worlds / "authored.wbt", "session world", inside)
    ctrl = _put(ctrls / "husky_random" / "husky_random.py", "ctrl", inside)
    ctrl_data = _put(ctrls / "husky_random" / "data.csv", "csv", inside)
    stray = _put(ctrls / "keep" / "u.py", "untracked beside tracked", inside)
    pre_existing = _put(worlds / "older.wbt", "old", t0 - 50)
    boundary = _put(worlds / "boundary.wbt", "edge", t0)   # NOT strictly in
    notes = _put(worlds / "notes.txt", "notes", inside)    # ineligible shape
    ignored = _put(worlds / "scratch.log", "ignored", inside)  # gitignored
    tracked = worlds / "tracked.wbt"
    tracked.write_text("tracked world DIRTY", encoding="utf-8")
    os.utime(tracked, (inside, inside))                    # dirty but TRACKED

    dest = tmp_path / "cell" / "repo_artifacts"
    records = staging.sweep_repo_junction_artifacts(
        dest, window=(t0, t1), repo=repo)

    # swept: preserved byte-identical with sha256 recorded, repo copy gone
    for src, text in ((authored, "session world"), (ctrl, "ctrl"),
                      (ctrl_data, "csv"),
                      (stray, "untracked beside tracked")):
        rel = src.relative_to(repo).as_posix()
        assert not src.exists(), "repo copy must be deleted: %s" % rel
        copy = dest / rel
        assert copy.read_text(encoding="utf-8") == text
        rec = next(r for r in records if r.get("rel") == rel)
        assert rec["action"] == "preserved_and_deleted"
        assert rec["sha256"] == hashlib.sha256(
            text.encode("utf-8")).hexdigest()
    # rails: everything else untouched
    assert pre_existing.exists(), "mtime before the window: never touched"
    assert boundary.exists(), "window bounds are STRICT"
    assert notes.exists(), "non-deliverable shapes are never touched"
    assert ignored.exists(), "gitignored build scratch is not evidence"
    assert tracked.read_text(encoding="utf-8") == "tracked world DIRTY", \
        "a TRACKED file must never be touched, however dirty"
    assert (ctrls / "keep" / "keep.py").is_file()
    # the emptied controller dir is pruned; a dir still holding a tracked
    # file survives (rmdir cannot remove a non-empty dir)
    assert not (ctrls / "husky_random").exists()
    assert (ctrls / "keep").is_dir()
    # skips are logged too
    rec = next(r for r in records
               if r.get("rel") == "projects/demos/worlds/boundary.wbt")
    assert rec["action"] == "skipped_outside_window"


def test_repo_junction_sweep_without_window_takes_any_leftover(tmp_path):
    # the pre-session quarantine mode: leftovers from crashed prior cells
    # have arbitrary mtimes, so no window -- but it is a MOVE, so nothing
    # is destroyed and the tracked tree is still never touched
    repo = _synth_git_repo(tmp_path)
    leftover = _put(repo / "projects" / "demos" / "worlds" / "leftover.wbt",
                    "crashed cell leftover", 1_000_000_000.0)
    dest = tmp_path / "quarantine" / "20990101_000000"
    records = staging.sweep_repo_junction_artifacts(dest, window=None,
                                                    repo=repo)
    assert not leftover.exists()
    assert (dest / "projects" / "demos" / "worlds"
            / "leftover.wbt").read_text(encoding="utf-8") \
        == "crashed cell leftover"
    assert (repo / "projects" / "demos" / "worlds" / "tracked.wbt").is_file()
    acted = [r["rel"] for r in records
             if r["action"] == "preserved_and_deleted"]
    assert acted == ["projects/demos/worlds/leftover.wbt"]


@needs_windows
def test_repo_junction_sweep_keeps_evidence_when_delete_is_refused(tmp_path):
    # a file held open (the running-cell case the forensics doc warns about):
    # preserved copy kept, refusal recorded, never raised
    repo = _synth_git_repo(tmp_path)
    held = _put(repo / "projects" / "demos" / "worlds" / "held.wbt",
                "held open", 1_000_000_000.0)
    dest = tmp_path / "d"
    with open(held, "r", encoding="utf-8"):
        records = staging.sweep_repo_junction_artifacts(dest, window=None,
                                                        repo=repo)
    rec = next(r for r in records
               if r.get("rel") == "projects/demos/worlds/held.wbt")
    assert rec["action"] == "preserved_delete_failed"
    assert held.exists()
    assert (dest / "projects" / "demos" / "worlds" / "held.wbt").is_file()


def test_repo_junction_sweep_refuses_a_dest_inside_the_swept_tree(tmp_path):
    repo = _synth_git_repo(tmp_path)
    with pytest.raises(ValueError):
        staging.sweep_repo_junction_artifacts(
            repo / "projects" / "quarantine", window=None, repo=repo)


def test_quarantine_dir_lands_at_the_campaign_root(tmp_path):
    campaign = tmp_path / "campaigns" / "phasew_cc_v9"
    assert cell._quarantine_dir(campaign / "cells" / "A1_x_omnisim_r0") \
        == campaign / "quarantine"
    assert cell._quarantine_dir(
        campaign / "cells_superseded" / "A1_x_omnisim_r0") \
        == campaign / "quarantine"
    standalone = tmp_path / "results" / "cc_lane" / "20260801_x_omnisim"
    assert cell._quarantine_dir(standalone) \
        == tmp_path / "results" / "cc_lane" / "quarantine"


# --- row-before-teardown (the reorder the crash mandated) ----------------------------


def _fake_cc_result():
    return {
        "type": "result", "subtype": "success", "is_error": False,
        "num_turns": 3, "duration_ms": 1500, "duration_api_ms": 1200,
        "result": "yes -- overlap between a and b",
        "session_id": "sess-1", "total_cost_usd": 0.5,
        "usage": {"input_tokens": 10, "output_tokens": 20,
                  "cache_read_input_tokens": 0,
                  "cache_creation_input_tokens": 0},
        "modelUsage": {"model-x": {"outputTokens": 20}},
    }


def _grader_row_stub():
    return {
        "task": "B1_overlap_audit", "sim": "omnisim",
        "condition": "claude_code", "outcome": "PASS",
        "assertions": {"B1.1": True}, "failed_assertion": None,
        "measurements": {}, "agent": {}, "agent_artifacts": {},
        "metrics": {"t_total_s": 9.0},
    }


def test_run_cell_writes_the_row_before_teardown_and_survives_a_lock(
        tmp_path, monkeypatch):
    """Synthetic end-to-end: a file held open during teardown must not lose
    the row. run_cell must (1) call teardown only AFTER rows.jsonl exists,
    (2) not raise when the teardown fails, (3) keep the teardown result out
    of the row and in the report postscript."""
    root = tmp_path / "root"
    run_dir = tmp_path / "out"
    ws_holder = {}
    order = []

    def fake_template(r):
        tpl = Path(r) / "templates" / "omnisim"
        tpl.mkdir(parents=True, exist_ok=True)
        return tpl

    def fake_instantiate(tpl, ws):
        Path(ws).mkdir(parents=True)
        ws_holder["ws"] = Path(ws)
        return {"filelist_sha256": "synth", "known_disclosures": [],
                "redactions": []}

    def fake_stage_task(ws, task_id, sim):
        return "find the overlap", []

    def fake_run_claude(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        # the "session" leaves a file open inside the workspace, like the
        # lingering engine that held omnisim_log.txt
        locked = Path(ws) / "omnisim_log.txt"
        locked.write_text("engine log", encoding="utf-8")
        ws_holder["handle"] = open(locked, "r", encoding="utf-8")
        return _fake_cc_result(), {
            "permission_mode": "dangerously-skip-permissions",
            "cli_command": "claude <prompt>", "rc": 0, "timed_out": False,
            "wall_s": 1.5, "launch_error": None}

    def fake_grade(task_id, artifact, grade_dir, notes, *, answer_path=None,
                   layout_dir=None):
        order.append("grade")
        return _grader_row_stub()

    real_resilient = staging.teardown_workspace_resilient

    def spying_resilient(ws, **kw):
        order.append("teardown")
        assert (run_dir / "rows.jsonl").is_file(), \
            "teardown ran before the row was written"
        assert (run_dir / "cell_report.json").is_file(), \
            "teardown ran before the cell report was written"
        return real_resilient(ws, tries=2, backoff_s=0.01,
                              sleep=lambda s: None)

    monkeypatch.setattr(cell, "preflight", lambda run_dir_, env: {
        "version": "test", "default_model": "model-x", "ok": True,
        "detail": "OK"})
    monkeypatch.setattr(cell.staging, "build_omnisim_template", fake_template)
    monkeypatch.setattr(cell.staging, "instantiate", fake_instantiate)
    monkeypatch.setattr(cell.staging, "stage_task", fake_stage_task)
    monkeypatch.setattr(cell.staging, "sweep_workspace_processes",
                        lambda ws, **kw: [])
    # the repo junction sweep must never touch the REAL repo from a unit test
    monkeypatch.setattr(cell.staging, "sweep_repo_junction_artifacts",
                        lambda dest, **kw: [])
    monkeypatch.setattr(cell.staging, "teardown_workspace_resilient",
                        spying_resilient)
    monkeypatch.setattr(cell, "run_claude_cell", fake_run_claude)
    monkeypatch.setattr(cell, "find_transcript", lambda sid: None)
    monkeypatch.setattr(cell, "grade_omnisim", fake_grade)

    try:
        merged = cell.run_cell("omnisim", "B1_overlap_audit", root=root,
                               out_dir=run_dir, use_locks=False, repeat=0)
    finally:
        ws_holder["handle"].close()

    assert order == ["grade", "teardown"]
    rows = [json.loads(l) for l in
            (run_dir / "rows.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    assert len(rows) == 1
    assert rows[0]["outcome"] == "PASS"
    assert rows[0]["metrics"]["turns"] == 3
    assert rows[0]["metrics"]["usd"] == 0.5
    assert "workspace_teardown" not in rows[0], \
        "the teardown result is a report postscript, never part of the row"
    report = json.loads((run_dir / "cell_report.json")
                        .read_text(encoding="utf-8"))
    if WINDOWS:
        assert report["workspace_teardown"]["ok"] is False, \
            "the held-open file must have failed the teardown on Windows"
        assert report["workspace_teardown"]["pending"], \
            "the leftover must be marked for the later sweep"
    assert merged["outcome"] == "PASS"
    # the answer artifact was collected before any teardown
    assert (run_dir / "artifact" / "answer.txt").read_text(
        encoding="utf-8").startswith("yes")


def test_run_cell_sweeps_stragglers_after_session_and_before_teardown(
        tmp_path, monkeypatch):
    root = tmp_path / "root"
    run_dir = tmp_path / "out"
    sweeps = []

    monkeypatch.setattr(cell, "preflight", lambda run_dir_, env: {
        "version": "test", "default_model": "model-x", "ok": True,
        "detail": "OK"})
    monkeypatch.setattr(cell.staging, "build_omnisim_template",
                        lambda r: Path(r) / "templates" / "omnisim")
    monkeypatch.setattr(
        cell.staging, "instantiate",
        lambda tpl, ws: (Path(ws).mkdir(parents=True),
                         {"filelist_sha256": "synth"})[1])
    monkeypatch.setattr(cell.staging, "stage_task",
                        lambda ws, t, s: ("p", []))
    monkeypatch.setattr(
        cell.staging, "sweep_workspace_processes",
        lambda ws, **kw: sweeps.append(str(ws)) or
        [{"where": "host", "pid": 4242, "name": "omnisim-bin.exe",
          "cmdline": "omnisim-bin.exe %s" % ws, "action": "terminated",
          "detail": None}])
    monkeypatch.setattr(
        cell.staging, "teardown_workspace_resilient",
        lambda ws, **kw: {"ok": True, "severed": [], "attempts": 1,
                          "error": None, "pending": None})
    monkeypatch.setattr(cell, "run_claude_cell",
                        lambda *a, **kw: (_fake_cc_result(), {
                            "permission_mode": "x", "cli_command": "c",
                            "rc": 0, "timed_out": False, "wall_s": 1.0,
                            "launch_error": None}))
    monkeypatch.setattr(cell, "find_transcript", lambda sid: None)
    monkeypatch.setattr(cell, "grade_omnisim",
                        lambda *a, **kw: _grader_row_stub())

    # the port-hygiene reaper must never touch the REAL machine's ports from
    # a unit test -- stub it, and record its schedule
    port_sweeps = []
    monkeypatch.setattr(
        cell.staging, "reap_port_listeners",
        lambda **kw: port_sweeps.append("called") or [])
    # ...and the repo junction sweep must never touch the REAL repo -- stub
    # it, and record its schedule + destinations
    repo_sweeps = []
    monkeypatch.setattr(
        cell.staging, "sweep_repo_junction_artifacts",
        lambda dest, **kw: repo_sweeps.append(
            {"dest": str(dest), "window": kw.get("window")}) or [])

    cell.run_cell("omnisim", "B1_overlap_audit", root=root, out_dir=run_dir,
                  use_locks=False)
    assert len(sweeps) == 2, "post-session AND pre-teardown sweeps"
    report = json.loads((run_dir / "cell_report.json")
                        .read_text(encoding="utf-8"))
    ws_whens = [s["when"] for s in report["process_sweeps"]
                if s.get("kind") != "port"]
    assert ws_whens == ["post_session", "pre_teardown"]
    # the port sweeps run before the session (contamination guard), after it
    # (reap our own leak) and after grading
    port_whens = [s["when"] for s in report["process_sweeps"]
                  if s.get("kind") == "port"]
    assert port_whens == ["pre_session", "post_session", "post_grade"]
    # every kill is logged into the cell report with pid + command line
    first_ws = [s for s in report["process_sweeps"]
                if s.get("kind") != "port"][0]
    assert first_ws["kills"][0]["pid"] == 4242
    assert "omnisim-bin.exe" in first_ws["kills"][0]["cmdline"]
    # ...and the row carries the port-hygiene provenance
    row = json.loads((run_dir / "rows.jsonl")
                     .read_text(encoding="utf-8").strip())
    assert [p["when"] for p in row["agent_artifacts"]["port_hygiene"]] == \
        ["pre_session", "post_session", "post_grade"]
    # the junction-artifact sweeps ran pre-session (quarantine, attributed
    # to no cell) and post-session (evidence into the cell dir, mtime
    # window), and both are logged into the cell report
    assert [s["when"] for s in report["repo_artifact_sweeps"]] == \
        ["pre_session", "post_session"]
    assert len(repo_sweeps) == 2
    assert "quarantine" in repo_sweeps[0]["dest"]
    assert repo_sweeps[0]["window"] is None
    assert repo_sweeps[1]["dest"] == str(run_dir / "repo_artifacts")
    assert repo_sweeps[1]["window"] is not None \
        and repo_sweeps[1]["window"][0] <= repo_sweeps[1]["window"][1]


def test_discover_artifact_reads_the_sweeps_preserved_copies(tmp_path):
    # After the post-session sweep the repo copy is GONE; the preserved
    # copy (mtimes kept by copy2) must feed discovery, mtime-gated the
    # same way as everything else.
    ws = tmp_path / "ws"
    ws.mkdir()
    preserved = tmp_path / "out" / "repo_artifacts"
    start = 2_000_000_000.0
    old = preserved / "projects" / "w" / "pre_existing.wbt"
    old.parent.mkdir(parents=True)
    old.write_text("not the session's", encoding="utf-8")
    os.utime(old, (start - 100, start - 100))
    art, rule = cell.discover_artifact(ws, [], start,
                                       extra_roots=(preserved,))
    assert art is None, "the mtime gate applies to preserved copies too"
    authored = preserved / "projects" / "w" / "authored.wbt"
    authored.write_text("session world", encoding="utf-8")
    os.utime(authored, (start + 10, start + 10))
    art, rule = cell.discover_artifact(ws, [], start,
                                       extra_roots=(preserved,))
    assert art == authored
    assert "PRESERVED" in rule


def test_run_cell_defers_a_session_limit_instead_of_blocking(
        tmp_path, monkeypatch):
    """The r8/r7 misclassification (2026-08-01): a session cut by the
    subscription limit must become a DEFERRED attempt on the existing
    backoff path -- never a blocked/failed cell."""
    root = tmp_path / "root"
    run_dir = tmp_path / "out"
    calls = {"n": 0}
    limit_result = {
        "type": "result", "subtype": "success", "is_error": True,
        "num_turns": 1, "duration_ms": 1628, "duration_api_ms": 0,
        "result": "You've hit your session limit \u00b7 resets 7:10pm "
                  "(Europe/Berlin)",
        "session_id": "limited", "total_cost_usd": 0, "usage": {},
        "modelUsage": {}}

    def fake_run_claude(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        calls["n"] += 1
        meta = {"permission_mode": "x", "cli_command": "c", "rc": 1,
                "timed_out": False, "wall_s": 1.0, "launch_error": None}
        if calls["n"] == 1:
            return dict(limit_result), meta
        return _fake_cc_result(), dict(meta, rc=0)

    monkeypatch.setattr(cell, "preflight", lambda run_dir_, env: {
        "version": "test", "default_model": "model-x", "ok": True,
        "detail": "OK"})
    monkeypatch.setattr(cell.staging, "build_omnisim_template",
                        lambda r: Path(r) / "templates" / "omnisim")
    monkeypatch.setattr(
        cell.staging, "instantiate",
        lambda tpl, ws: (Path(ws).mkdir(parents=True),
                         {"filelist_sha256": "synth"})[1])
    monkeypatch.setattr(cell.staging, "stage_task",
                        lambda ws, t, s: ("p", []))
    monkeypatch.setattr(cell.staging, "sweep_workspace_processes",
                        lambda ws, **kw: [])
    monkeypatch.setattr(cell.staging, "reap_port_listeners",
                        lambda **kw: [])
    monkeypatch.setattr(cell.staging, "sweep_repo_junction_artifacts",
                        lambda dest, **kw: [])
    monkeypatch.setattr(
        cell.staging, "teardown_workspace_resilient",
        lambda ws, **kw: {"ok": True, "severed": [], "attempts": 1,
                          "error": None, "pending": None})
    monkeypatch.setattr(cell, "run_claude_cell", fake_run_claude)
    monkeypatch.setattr(cell, "find_transcript", lambda sid: None)
    monkeypatch.setattr(cell, "grade_omnisim",
                        lambda *a, **kw: _grader_row_stub())

    merged = cell.run_cell("omnisim", "B1_overlap_audit", root=root,
                           out_dir=run_dir, use_locks=False,
                           rate_limit_backoff_s=0.0)
    assert calls["n"] == 2, "the limited attempt retries after backoff"
    assert merged["outcome"] == "PASS"
    defs = merged["agent_artifacts"]["rate_limit_deferrals"]
    assert len(defs) == 1
    assert "session" in defs[0]["marker"] and "limit" in defs[0]["marker"]
    report = json.loads((run_dir / "cell_report.json")
                        .read_text(encoding="utf-8"))
    assert "blocker" not in report, \
        "a recognised limit must never block/burn the cell"
    assert [d["marker"] for d in report["rate_limit_deferrals"]] \
        == [defs[0]["marker"]]


# --- the pinned model actually did the work ----------------------------------
#
# Asking for a model and running it are different facts. Claude Code bills
# small auxiliary calls to a cheaper model, so a cell is never literally
# single-model; these pin the tolerance that separates "auxiliary noise" from
# "a different model did the task".

def test_model_attribution_accepts_measured_auxiliary_haiku():
    """The real measured split from the first v0.3 pilot cell."""
    att = cell.model_attribution(
        {"claude-opus-5": 11175, "claude-haiku-4-5-20251001": 15},
        "claude-opus-5")
    assert att["ok"] is True
    assert att["expected_share"] > 0.99
    assert att["other_models"] == {"claude-haiku-4-5-20251001": 15}


def test_model_attribution_catches_a_wrong_model_doing_the_work():
    att = cell.model_attribution(
        {"claude-opus-4-8": 9000, "claude-opus-5": 10}, "claude-opus-5")
    assert att["ok"] is False, "a different model did the generating"
    assert att["other_models"] == {"claude-opus-4-8": 9000}


def test_model_attribution_is_none_not_true_when_unmeasured():
    """Unmeasured must never read as verified (SPEC 8.1: ground truth,
    never narration)."""
    for usage in (None, {}, "nonsense"):
        att = cell.model_attribution(usage, "claude-opus-5")
        assert att["ok"] is None
        assert "reason" in att


def test_merged_row_carries_the_model_split_and_attribution():
    cc = cell.cc_metrics_from_result(
        {"modelUsage": {"claude-opus-5": {"outputTokens": 500},
                        "claude-haiku-4-5-20251001": {"outputTokens": 3}},
         "usage": {}, "num_turns": 2},
        tool_calls=2, tool_calls_reason=None, version="test",
        permission_mode="x", transcript=None, cell_wall_s=1.0,
        cli_command="claude -p")
    assert cc["model"] == "claude-opus-5"
    assert cc["model_usage"] == {"claude-haiku-4-5-20251001": 3,
                                 "claude-opus-5": 500}
    merged = cell.merge_cc_metrics({"outcome": "PASS"}, cc)
    art = merged["agent_artifacts"]["claude_code"]
    assert art["model_usage"] == cc["model_usage"], \
        "every model that billed is published, not just the argmax"
    assert art["model_attribution"]["ok"] is True


def test_campaign_state_records_the_resolved_model_not_null(monkeypatch,
                                                            tmp_path):
    """state.json must be auditable on its own -- no "model": null.

    Exercises the campaign's OWN argument parser, so a future edit that
    reverts the default to None fails here rather than silently producing an
    unauditable campaign state.
    """
    from agentbench.cc_lane import run_campaign_cc as camp

    captured = {}

    class _Stub(camp.Campaign):
        def __init__(self, args):
            captured["model"] = args.model
            raise SystemExit(0)          # stop before touching the filesystem

    monkeypatch.setattr(camp, "Campaign", _Stub)
    with pytest.raises(SystemExit):
        camp.main(["--campaign-id", "t", "--sim", "omnisim"])
    assert captured["model"] == cell.DEFAULT_MODEL == "claude-opus-5",         "the campaign must pin the resolved id, not None"


# --- budget exhaustion is a RESULT, never a missing row -----------------------
#
# Measured on the first v0.3 pilot: B2/omnisim hit its 720 s cap and run_cell
# raised SystemExit, so the driver marked the cell "blocked" and appended NO
# row. Timed-out cells therefore left the pass@1 DENOMINATOR -- and since the
# cells that time out are exactly the ones the agent was losing, a simulator
# that ran out of time scored BETTER than one that finished and was wrong.
# SPEC 2.4: budget exhausted -> FAIL, and a FAIL is a row.

def _minimal_cell_env(monkeypatch, tmp_path, *, staged, cc_meta,
                      grade_row=None):
    """Wire run_cell up with no Claude, no engine and no real workspace."""
    def fake_template(r):
        tpl = Path(r) / "templates" / "omnisim"
        tpl.mkdir(parents=True, exist_ok=True)
        return tpl

    def fake_instantiate(tpl, ws):
        Path(ws).mkdir(parents=True)
        return {"filelist_sha256": "synth", "known_disclosures": [],
                "redactions": []}

    def fake_stage_task(ws, task_id, sim):
        out = []
        for name in staged:
            p = Path(ws) / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("WorldInfo {\n}\n", encoding="utf-8")
            # Real stage_task copies with shutil.copy2, which PRESERVES the
            # source mtime -- that is exactly what makes discover_artifact's
            # "modified after session start" gate mean anything. A fake that
            # writes fresh files makes an untouched world look session-edited.
            stale = time.time() - 3600
            os.utime(p, (stale, stale))
            out.append(str(p))
        return "do the task", out

    def fake_run_claude(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        # A real session ALWAYS leaves its NDJSON behind, and the containment
        # gate reads it, so the double has to leave one too -- a fake that
        # reports 61 tool calls and writes no stream is a session shape that
        # cannot happen, and testing against it would tune the gate to
        # fiction. Two innocuous product reads: in bounds, nothing excluded.
        Path(run_dir_).mkdir(parents=True, exist_ok=True)
        events = [json.dumps({
            "type": "assistant",
            "message": {"role": "assistant",
                        "content": [{"type": "tool_use", "id": "t",
                                     "name": "Read",
                                     "input": {"file_path": f}}]}})
            for f in ("AGENTS.md", "docs/guide/omnilink-chat-demos.md")]
        (Path(run_dir_) / "cc_stream.jsonl").write_text(
            "\n".join(events) + "\n", encoding="utf-8")
        # ...and by the same argument, a real session whose guard hook is
        # installed leaves a guard-log line per tool call. A double that skips
        # it is the "hook never ran" shape, which the guard gate exists to
        # refuse -- so the double would be testing an instrument failure it is
        # not trying to simulate.
        if kw.get("settings_path"):
            log = Path(kw["settings_path"]).parent / "guard_events.jsonl"
            log.write_text("".join(
                json.dumps({"tool": "Read", "allow": True, "matched": None,
                            "input": f}) + "\n"
                for f in ("AGENTS.md", "docs/guide/omnilink-chat-demos.md")),
                encoding="utf-8")
        return None, dict(cc_meta)          # no result JSON

    monkeypatch.setattr(cell, "preflight", lambda run_dir_, env: {
        "version": "test", "default_model": "cli-default", "ok": True,
        "detail": "OK"})
    monkeypatch.setattr(cell.staging, "build_omnisim_template", fake_template)
    monkeypatch.setattr(cell.staging, "build_webots_template", fake_template)
    monkeypatch.setattr(cell.staging, "build_mujoco_template", fake_template,
                        raising=False)
    monkeypatch.setattr(cell.staging, "instantiate", fake_instantiate)
    monkeypatch.setattr(cell.staging, "stage_task", fake_stage_task)
    monkeypatch.setattr(cell.staging, "sweep_workspace_processes",
                        lambda ws, **kw: [])
    monkeypatch.setattr(cell.staging, "sweep_repo_junction_artifacts",
                        lambda dest, **kw: [])
    monkeypatch.setattr(cell.staging, "teardown_workspace_resilient",
                        lambda ws, **kw: {"ok": True, "pending": None,
                                          "error": None})
    monkeypatch.setattr(cell, "run_claude_cell", fake_run_claude)
    monkeypatch.setattr(cell, "find_transcript", lambda sid: None)
    # The lane's virtual display is a WSL round trip; these cells never touch
    # WSL, so it is stubbed UP -- the failure branch is exercised directly in
    # test_headless_is_forced_on_both_arms.
    monkeypatch.setattr(
        cell, "ensure_virtual_display",
        lambda **kw: {"ok": True,
                      "display": getattr(cell, "WEBOTS_HEADLESS_DISPLAY", ":99"),
                      "state": "started", "detail": "stub"},
        raising=False)
    monkeypatch.setattr(
        cell, "grade_omnisim",
        lambda *a, **kw: dict(grade_row or _grader_row_stub(),
                              outcome="FAIL"))
    monkeypatch.setattr(
        cell, "grade_webots",
        lambda *a, **kw: dict(grade_row or _grader_row_stub(), sim="webots",
                              outcome="FAIL"))
    monkeypatch.setattr(
        cell, "grade_mujoco",
        lambda *a, **kw: dict(grade_row or _grader_row_stub(), sim="mujoco",
                              outcome="FAIL"), raising=False)


def test_timeout_on_an_answer_task_still_lands_a_row(tmp_path, monkeypatch):
    run_dir = tmp_path / "out"
    _minimal_cell_env(
        monkeypatch, tmp_path, staged=["six_huskies.wbt"],
        cc_meta={"permission_mode": "dangerously-skip-permissions",
                 "cli_command": "claude -p", "rc": None, "timed_out": True,
                 "wall_s": 816.5, "launch_error": None})
    merged = cell.run_cell("omnisim", "B1_overlap_audit",
                           root=tmp_path / "root", out_dir=run_dir,
                           use_locks=False, repeat=0)
    assert merged["outcome"] == "FAIL", "budget exhaustion is a FAIL"
    assert merged["agent_artifacts"]["budget_exhausted"] is True
    rows = [json.loads(l) for l in
            (run_dir / "rows.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    assert len(rows) == 1, "a timed-out cell MUST land a row (SPEC 2.4)"
    # no CLI JSON existed, so agent metrics are null -- never invented
    assert rows[0]["metrics"]["tokens_out"] is None
    assert rows[0]["metrics"]["usd"] is None


def test_timeout_on_a_world_task_grades_the_pristine_staged_world(
        tmp_path, monkeypatch):
    """No deliverable + no row would drop the cell; grade what was staged."""
    run_dir = tmp_path / "out"
    _minimal_cell_env(
        monkeypatch, tmp_path, staged=["fall_through.wbt"],
        cc_meta={"permission_mode": "x", "cli_command": "claude -p",
                 "rc": None, "timed_out": True, "wall_s": 900.0,
                 "launch_error": None})
    merged = cell.run_cell("omnisim", "C2_fall_through_floor",
                           root=tmp_path / "root", out_dir=run_dir,
                           use_locks=False, repeat=0)
    assert merged["outcome"] == "FAIL"
    assert merged["agent_artifacts"]["budget_exhausted"] is True
    # discover_artifact already falls back to the UNCHANGED staged world for a
    # task that stages one, so this cell is graded normally (the task's defect
    # is still present -> FAIL). The point is only that the row exists.
    assert merged["agent_artifacts"]["no_artifact"] is False
    assert (run_dir / "rows.jsonl").is_file()


def test_no_deliverable_at_all_is_a_fail_row_not_a_dropped_cell(
        tmp_path, monkeypatch):
    """A1 stages nothing, so a session that writes no world leaves the
    discovery rules with nothing -- and that cell must still be counted."""
    run_dir = tmp_path / "out"
    _minimal_cell_env(
        monkeypatch, tmp_path, staged=[],          # A1: empty initial/
        cc_meta={"permission_mode": "x", "cli_command": "claude -p",
                 "rc": None, "timed_out": True, "wall_s": 900.0,
                 "launch_error": None})
    merged = cell.run_cell("omnisim", "A1_husky_swarm_10",
                           root=tmp_path / "root", out_dir=run_dir,
                           use_locks=False, repeat=0)
    assert merged["outcome"] == "FAIL"
    assert merged["progress"] == 0
    assert merged["failed_assertion"] == "no_artifact"
    assert merged["agent_artifacts"]["no_artifact"] is True
    assert merged["assertions"] == {}, "nothing was measured, nothing claimed"
    rows = [json.loads(l) for l in
            (run_dir / "rows.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    assert len(rows) == 1, "an empty-handed cell MUST stay in the denominator"


def test_a_launch_failure_is_still_a_blocker_not_a_fail(tmp_path, monkeypatch):
    """A broken instrument is NOT the agent's result: no row, still blocks."""
    run_dir = tmp_path / "out"
    _minimal_cell_env(
        monkeypatch, tmp_path, staged=["six_huskies.wbt"],
        cc_meta={"permission_mode": None, "cli_command": "claude -p",
                 "rc": 1, "timed_out": False,
                 "wall_s": 0.3, "launch_error": "claude: command not found"})
    with pytest.raises(SystemExit) as exc:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=run_dir, use_locks=False, repeat=0)
    assert "command not found" in str(exc.value)
    assert not (run_dir / "rows.jsonl").exists(), \
        "instrument breakage must not be scored as an agent FAIL"


# --- relative asset urls survive collection ----------------------------------
#
# Measured on the first v0.3 pilot (A1/omnisim): the agent saved a correct
# 10-Husky world to projects/samples/demos/worlds/showcase/ and referenced
# "../../../../robots/clearpath/.../husky.urdf" -- which resolves exactly
# right from there. The junction-hygiene sweep then preserved the world into
# <run_dir>/repo_artifacts/<same relative path> and deleted the repo copy, so
# the relative url pointed into a mirror with no robots/ sibling. Phase B
# logged "Cannot open URDF file" ten times and graded n_robots: 0. The agent's
# world was right; the collection broke it.

def test_authoring_dir_recovers_the_original_repo_location(tmp_path):
    run_dir = tmp_path / "cell"
    art = (run_dir / "repo_artifacts" / "projects" / "samples" / "demos"
           / "worlds" / "showcase" / "w.wbt")
    art.parent.mkdir(parents=True)
    art.write_text("x", encoding="utf-8")
    assert cell._authoring_dir(art, run_dir) == (
        cell.REPO / "projects/samples/demos/worlds/showcase")


def test_authoring_dir_is_the_parent_for_a_workspace_artifact(tmp_path):
    run_dir = tmp_path / "cell"
    art = tmp_path / "ws" / "world.omniworld"
    art.parent.mkdir(parents=True)
    art.write_text("x", encoding="utf-8")
    assert cell._authoring_dir(art, run_dir) == art.parent


def test_relative_urls_are_rebased_against_where_the_world_was_written(
        tmp_path):
    """The end-to-end property: a url that resolved where the agent wrote it
    still resolves after collection."""
    from agentbench.common.worldtext import rebase_relative_urls
    authored = tmp_path / "worlds" / "showcase"
    authored.mkdir(parents=True)
    asset = tmp_path / "robots" / "husky.urdf"
    asset.parent.mkdir(parents=True)
    asset.write_text("<robot/>", encoding="utf-8")
    world = 'URDFRobot {\n  url "../../robots/husky.urdf"\n}\n'
    new, changes = rebase_relative_urls(world, authored)
    assert len(changes) == 1
    assert Path(changes[0]["to"]).is_file(), \
        "the rebased url must resolve to the real asset"
    # a url that was ALREADY broken where it was authored stays broken, so a
    # genuinely bad reference still fails the task rather than being repaired
    bad, bad_changes = rebase_relative_urls(
        'URDFRobot {\n  url "../nope/missing.urdf"\n}\n', authored)
    assert bad_changes == [] and "../nope/missing.urdf" in bad


# --- the wall-clock cap must actually be a cap --------------------------------
#
# Measured 2026-08-09: a webots A1 cell reached 28.5 minutes against a
# 15-minute cap and was still spawning engines. Cause: subprocess.run(
# capture_output=True, timeout=...) reaps a timed-out child with
# communicate() and NO timeout on Windows, so a grandchild holding the
# inherited pipe blocks the reap for ever. The cost ceiling was not a ceiling.

def test_timeout_fires_even_when_a_grandchild_holds_the_stdio(tmp_path,
                                                              monkeypatch):
    """The exact shape that hung: the session spawns a longer-lived child that
    inherits stdout/stderr. The cap must still fire, promptly."""
    helper = tmp_path / "spawner.py"
    helper.write_text(
        "import subprocess, sys, time\n"
        "subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(45)'])\n"
        "time.sleep(45)\n",
        encoding="utf-8")
    monkeypatch.setattr(cell, "_claude_exe", lambda: sys.executable)

    t0 = time.monotonic()
    with pytest.raises(subprocess.TimeoutExpired) as exc:
        cell._run_claude([str(helper)], cwd=tmp_path, env=dict(os.environ),
                         timeout_s=3,
                         stdout_path=tmp_path / "o.txt",
                         stderr_path=tmp_path / "e.txt")
    waited = time.monotonic() - t0
    assert waited < 30, (
        "the cap did not fire promptly (%.1f s for a 3 s timeout) -- the reap "
        "is blocking on a grandchild again" % waited)
    assert getattr(exc.value, "killed_tree", None) is not None


def test_a_normal_session_still_returns_stdout_and_rc(tmp_path, monkeypatch):
    """The fix must not break the ordinary path."""
    helper = tmp_path / "ok.py"
    helper.write_text("print('hello')\n", encoding="utf-8")
    monkeypatch.setattr(cell, "_claude_exe", lambda: sys.executable)
    proc = cell._run_claude([str(helper)], cwd=tmp_path, env=dict(os.environ),
                            timeout_s=60, stdout_path=tmp_path / "o.txt",
                            stderr_path=tmp_path / "e.txt")
    assert proc.returncode == 0
    assert "hello" in proc.stdout
    assert (tmp_path / "o.txt").is_file(), "stdio is written to files"


# --- the deliverable is a PROJECT, not a file --------------------------------
#
# MEASURED 2026-08-09 (R1/omnisim, results/cc_lane/r1_omnisim_first/): the agent
# wrote its controller to projects/samples/demos/controllers/avoid_obstacles/
# and a world declaring controller "avoid_obstacles". Collection took ONLY the
# .wbt -- artifact/ holds lidar_nav.wbt and nothing else -- so the graded world
# had no controllers/ sibling, the robot got no controller and never moved:
# path length 0.0 m, two assertions failed on OUR bug.

def _r1_obstacles_wbt():
    """The five published obstacles as ``.wbt`` text.

    Generated from the task's own asset rather than typed out, so this fixture
    cannot drift from the spec. An R1 deliverable needs them for a reason that
    has nothing to do with what these tests measure: since grade-time
    placement was wired in (``common/r1_placement.py``), a cell whose
    deliverable has no obstacles to move is BLOCKED rather than graded, and
    these fixtures would otherwise block before reaching the assertions they
    exist for.
    """
    from agentbench.graders import r1_core
    return "".join(
        'DEF %s Solid {\n'
        '  translation %g %g %g\n'
        '  children [ Shape { geometry Box { size %g %g %g } } ]\n'
        '  name "%s"\n'
        '}\n' % ((o["name"],) + tuple(o["position"]) + tuple(o["size"])
                 + (o["name"],))
        for o in r1_core.obstacle_spec())


def _r1_obstacles_mjcf():
    """...and as MJCF geoms. ``size`` is a HALF extent on this arm."""
    from agentbench.graders import r1_core
    return "".join(
        '    <geom name="%s" type="box" pos="%g %g %g" size="%g %g %g"/>\n'
        % ((o["name"],) + tuple(o["position"])
           + tuple(v / 2.0 for v in o["size"]))
        for o in r1_core.obstacle_spec())


R1_PROJECT_WORLD = """#OMNISIM R2025a utf8
WorldInfo {
}
Robot {
  name "rover"
  controller "avoid_obstacles"
}
Robot {
  name "prop"
  # ships with the product, NOT with the agent's project
  controller "void"
}
Robot {
  name "decoration"
  controller "<none>"
}
""" + _r1_obstacles_wbt()


def _session_writing_a_project(cc_meta, world_rel, ctrl_rel, *,
                               world_text=R1_PROJECT_WORLD):
    """A fake ``run_claude_cell`` that leaves a project-shaped deliverable."""
    def fake(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        world = Path(ws) / world_rel
        world.parent.mkdir(parents=True, exist_ok=True)
        world.write_text(world_text, encoding="utf-8")
        ctrl = Path(ws) / ctrl_rel
        ctrl.parent.mkdir(parents=True, exist_ok=True)
        ctrl.write_text("print('drive')\n", encoding="utf-8")
        # discover_artifact is mtime-gated against session start; the clock
        # granularity on Windows is coarse enough to lose a same-tick write.
        later = time.time() + 5
        os.utime(world, (later, later))
        return None, dict(cc_meta)
    return fake


def test_a_world_is_collected_with_the_controller_it_names(tmp_path,
                                                           monkeypatch):
    """The R1 regression: collect the PROJECT, and put it where phase B looks.

    Before this, only the ``.wbt`` was copied and every assertion below failed.
    """
    run_dir = tmp_path / "out"
    cc_meta = {"permission_mode": "x", "cli_command": "claude -p", "rc": None,
               "timed_out": True, "wall_s": 900.0, "launch_error": None}
    _minimal_cell_env(monkeypatch, tmp_path, staged=[], cc_meta=cc_meta)
    monkeypatch.setattr(cell, "run_claude_cell", _session_writing_a_project(
        cc_meta,
        "projects/samples/demos/worlds/showcase/indoor_avoidance.wbt",
        "projects/samples/demos/controllers/avoid_obstacles/"
        "avoid_obstacles.py"))

    merged = cell.run_cell("omnisim", "R1_lidar_nav", root=tmp_path / "root",
                           out_dir=run_dir, use_locks=False, repeat=0)

    # 1. the evidence copy: the cell dir shows the deliverable WHOLE
    assert (run_dir / "artifact" / "lidar_nav.wbt").is_file()
    assert (run_dir / "artifact" / "controllers" / "avoid_obstacles"
            / "avoid_obstacles.py").is_file(), \
        "the controller the world names must be collected beside it"

    # 2. the copy the ENGINE reads, at the project root phase B resolves
    grade_project = run_dir / "grade"
    assert (grade_project / "controllers" / "avoid_obstacles"
            / "avoid_obstacles.py").is_file()

    # 3. ...and that root really is the one the engine computes for the world
    #    run_agentbench will launch:  <grade>/worlds/<cell>/scratch/<world>.
    #    (OmProject::projectPathFromWorldFile -- nearest "worlds" ancestor's
    #    parent. The same walk resolves the webots arm's <grade>/worlds/<w>.)
    staged_world = (grade_project / "worlds" / "R1_lidar_nav.external.r0"
                    / "scratch" / "lidar_nav.wbt")
    assert cell.project_root_for_world(staged_world) \
        == grade_project.resolve()
    assert cell.project_root_for_world(grade_project / "worlds" / "w.wbt") \
        == grade_project.resolve()

    # 4. the row says what came with the world, so a reader never has to assume
    art = merged["agent_artifacts"]
    assert art["collected_controllers"] == ["avoid_obstacles"]
    # a controller the agent did NOT author is named, not copied: it resolves
    # from the product install at phase B exactly as it did for the agent
    assert art["controllers_not_in_project"] == ["void"]
    assert not (grade_project / "controllers" / "void").exists()
    # sentinels name behaviour, not a directory
    assert "<none>" not in cell.controller_names(
        run_dir / "artifact" / "lidar_nav.wbt")


def test_controllers_are_recovered_from_the_hygiene_mirror(tmp_path):
    """The measured shape: the sweep deleted the repo copy and preserved the
    project under ``repo_artifacts/<repo-relative path>``, so the controller
    must be found relative to where the world SITS, not only where it was
    written."""
    run_dir = tmp_path / "cell"
    mirror = run_dir / "repo_artifacts" / "projects" / "samples" / "demos"
    world = mirror / "worlds" / "showcase" / "indoor_avoidance.wbt"
    world.parent.mkdir(parents=True)
    world.write_text(R1_PROJECT_WORLD, encoding="utf-8")
    ctrl = mirror / "controllers" / "avoid_obstacles" / "avoid_obstacles.py"
    ctrl.parent.mkdir(parents=True)
    ctrl.write_text("print('drive')\n", encoding="utf-8")

    authored_in = cell._authoring_dir(world, run_dir)
    assert authored_in == cell.REPO / "projects/samples/demos/worlds/showcase"
    got, missing = cell.collect_controllers(
        world, tmp_path / "art",
        search_roots=(cell.project_root_for_world(world),
                      cell.project_root_for_world(authored_in / world.name)))
    assert [c["name"] for c in got] == ["avoid_obstacles"]
    assert missing == ["void"]
    assert (tmp_path / "art" / "controllers" / "avoid_obstacles"
            / "avoid_obstacles.py").read_text(encoding="utf-8") \
        == "print('drive')\n"


# --- "headless" must actually be headless, on BOTH arms ----------------------
#
# MEASURED 2026-08-09 (webots arm): the agent's own scripts/run_sim.sh called
# `webots --batch --mode=fast` with no xvfb-run. Our launcher's xvfb-run is not
# inherited by anything the AGENT launches, and WSLg supplies DISPLAY=:0, so
# upstream opened real GUI windows on the operator's Windows desktop -- and GUI
# rendering moves the wall clock, which is the leaderboard's second key.

def test_headless_is_forced_on_both_arms(monkeypatch):
    env, removed = cell.scrub_env({"PATH": "keep", "WSLENV": "FOO/p",
                                   "OMNISIM_NO_WINDOW": "stale",
                                   "OMNISIM_LOG_PATH": "x"})
    # the directive survives the OMNISIM_ scrub that would otherwise eat it
    assert "OMNISIM_LOG_PATH" in removed and "OMNISIM_LOG_PATH" not in env
    assert env["OMNISIM_NO_WINDOW"] == "1"
    assert env["DISPLAY"] == cell.WEBOTS_HEADLESS_DISPLAY
    assert env["WSLENV"].split(":") == ["FOO/p", "DISPLAY/u"], \
        "WSLENV is what carries DISPLAY across the Win->WSL boundary, and it " \
        "must not clobber what the operator already shares"

    # omnisim cell: the engine's own windowless knob, and nothing aimed at WSL
    o = dict(env)
    rec_o = cell.enforce_headless(o, "omnisim")
    assert rec_o["enforced"] is True and o["OMNISIM_NO_WINDOW"] == "1"
    assert "DISPLAY" not in o

    # webots cell: DISPLAY forwarded once the lane's virtual display is up
    w = dict(env)
    rec_w = cell.enforce_headless(
        w, "webots",
        ensure=lambda: {"ok": True, "state": "started", "display": ":99"})
    assert rec_w["enforced"] is True
    assert w["DISPLAY"] == cell.WEBOTS_HEADLESS_DISPLAY
    assert "DISPLAY/u" in w["WSLENV"].split(":")
    assert w["OMNISIM_NO_WINDOW"] == "1", \
        "inert on this arm, but the policy is set in one shared place"

    # ...and a display that could NOT be brought up is FLAGGED, never faked:
    # forwarding a dead DISPLAY would stop the agent starting the simulator at
    # all, which biases the comparison far harder than a visible window.
    w2 = dict(env)
    rec_bad = cell.enforce_headless(
        w2, "webots",
        ensure=lambda: {"ok": False, "state": "xvfb_missing", "detail": "no"})
    assert rec_bad["enforced"] is False
    assert "DISPLAY" not in w2
    assert w2["WSLENV"] == "FOO/p", \
        "only OUR entry comes back out; the operator's sharing list stays"
    assert "not comparable" in rec_bad["detail"]
    # nothing else to share -> the variable goes entirely
    w3 = {"WSLENV": "DISPLAY/u", "DISPLAY": ":99"}
    cell.drop_webots_headless(w3)
    assert w3 == {}


def test_both_arms_record_their_headless_enforcement_in_the_row(tmp_path,
                                                                monkeypatch):
    """Whichever arm a cell ran on, the row says whether it was windowless."""
    cc_meta = {"permission_mode": "x", "cli_command": "claude -p", "rc": None,
               "timed_out": True, "wall_s": 900.0, "launch_error": None}
    seen = {}
    for i, sim in enumerate(("omnisim", "webots")):
        run_dir = tmp_path / ("out_%s" % sim)
        mp = monkeypatch.__class__()
        try:
            _minimal_cell_env(mp, tmp_path, staged=["fall_through.wbt"],
                              cc_meta=cc_meta)
            merged = cell.run_cell(sim, "C2_fall_through_floor",
                                   root=tmp_path / ("root_%d" % i),
                                   out_dir=run_dir, use_locks=False, repeat=0)
        finally:
            mp.undo()
        head = merged["agent_artifacts"]["headless"]
        assert head["arm"] == sim
        assert head["enforced"] is True, \
            "%s cells must not silently render a GUI" % sim
        seen[sim] = head["mechanism"]
    assert seen["omnisim"] == "OMNISIM_NO_WINDOW=1"
    assert "DISPLAY" in seen["webots"], \
        "both arms are forced headless -- neither is handicapped"


def test_ensure_virtual_display_never_raises_and_reports_the_reason():
    """A machine with no WSL is a machine where the webots directive cannot be
    enforced; that is a recorded fact, not a crash."""
    def boom(cmd):
        raise FileNotFoundError("wsl.exe not found")
    out = cell.ensure_virtual_display(runner=boom)
    assert out["ok"] is False and out["state"] == "wsl_unavailable"

    class _P:
        returncode, stdout, stderr = 3, "NOXVFB\n", ""
    out = cell.ensure_virtual_display(runner=lambda cmd: _P())
    assert out["ok"] is False and out["state"] == "xvfb_missing"

    class _Q:
        returncode, stdout, stderr = 0, "REUSED\n", ""
    out = cell.ensure_virtual_display(runner=lambda cmd: _Q())
    assert out["ok"] is True and out["state"] == "reused"


def test_controllers_are_found_when_the_world_sits_at_the_workspace_root():
    """The webots layout that lost its controllers.

    The workspace we hand an agent is near-empty, so it invents a layout, and
    writing the world at the ROOT (beside the staged benchmark_assets/) is the
    natural choice. That world has no `worlds` ancestor, so the ENGINE's
    project rule lands on the workspace's PARENT and never sees the sibling
    controllers/ dir. Measured 2026-08-09: a webots R1 world naming
    lidar_avoider and monitor collected NOTHING, which grades a robot that
    cannot move -- understating that arm for an instrument reason and
    flattering ours in the comparison.
    """
    import tempfile
    ws = Path(tempfile.mkdtemp()) / "ws"
    (ws / "benchmark_assets").mkdir(parents=True)
    world = ws / "lidar_nav.wbt"
    world.write_text('Robot { controller "lidar_avoider" }\n'
                     'Robot { controller "monitor" }\n', encoding="utf-8")
    for n in ("lidar_avoider", "monitor"):
        d = ws / "controllers" / n
        d.mkdir(parents=True)
        (d / (n + ".py")).write_text("# ctrl", encoding="utf-8")

    # the engine rule ALONE misses it -- that is the bug, pinned
    assert not (cell.project_root_for_world(world) / "controllers").is_dir()

    dest = Path(tempfile.mkdtemp()) / "artifact"
    got, missing = cell.collect_controllers(
        world, dest, search_roots=cell.candidate_project_roots(world))
    assert missing == [], "a referenced controller was not collected"
    assert sorted(g["name"] for g in got) == ["lidar_avoider", "monitor"]
    assert (dest / "controllers" / "lidar_avoider"
            / "lidar_avoider.py").is_file()


def test_a_cell_is_bounded_in_total_not_just_per_session(tmp_path,
                                                         monkeypatch):
    """The deferral loop must not multiply the budget.

    Measured 2026-08-09: a webots R1 cell ran 58 minutes across at least two
    sessions. The per-session cap bounded ONE session; the deferral loop then
    handed each retry a fresh full-length session plus a backoff, so the real
    worst case was retries x (budget + backoff) -- about NINE HOURS for one
    cell -- while the ceiling claimed cells x budget was the bound.
    """
    seen = []

    def fake_run_claude(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        seen.append(timeout_s)
        # always "rate limited", so the loop would spin for ever if unbounded.
        # The refusal is on the CHILD's stderr, which is where the real one
        # arrives and the only channel `deferral_reason` reads -- our own
        # `launch_error` prose is not evidence (see run_cc_cell.deferral_reason:
        # it quotes rc=4294967295, and "4294967295" contains "429").
        return None, {"permission_mode": "x", "cli_command": "claude -p",
                      "rc": 1, "timed_out": False, "wall_s": 0.1,
                      "stderr_tail": "Claude usage limit reached",
                      "launch_error": "no result event in the session stream "
                                      "(rc=1, 0 events): Claude usage limit "
                                      "reached"}

    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta={})
    monkeypatch.setattr(cell, "run_claude_cell", fake_run_claude)

    # A backoff larger than the remaining wall clock: the retry could not
    # FINISH inside the cell's ceiling, so it must be abandoned rather than
    # started. (No sleep happens -- the guard fires before the backoff.)
    with pytest.raises(SystemExit) as exc:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False,
                      rate_limit_backoff_s=10_000.0, repeat=0)
    assert "wall ceiling" in str(exc.value), \
        "an out-of-room rate limit must abandon the cell, not keep retrying"
    # and every session was offered no more than the task budget
    budget = tasks_mod.get("B1_overlap_audit").timeout_s
    assert seen and all(s <= budget + 1 for s in seen)


# --- the MuJoCo arm: a deliverable that is a PAIR ----------------------------
#
# MJCF is inert data. It declares no controller, starts no process, and a
# scene alone cannot move -- so a MuJoCo deliverable is TWO files: the model
# and the Python program that steps it. Collecting only the `.xml` reproduces
# exactly the defect that zeroed OmniSim's R1 (a world collected without the
# thing that drives it, graded as a robot that never moved -- on OUR bug, not
# the agent's work). `adapters/mujoco/BRINGUP.md` sec. 6 states the
# convention; these pin the wiring of it.

def test_artifact_name_is_per_sim_and_never_guesses():
    """The deliverable's filename is the ARM's property, not only the task's."""
    an = external_agent.artifact_name
    # the two .wbt arms are untouched -- including the default (no sim given)
    for sim in (None, "omnisim", "webots"):
        assert an("R1_lidar_nav", sim) == "lidar_nav.wbt"
        assert an("C2_fall_through_floor", sim) == "fall_through.wbt"
        assert external_agent.artifact_suffixes(sim) == (".wbt",)
    # MuJoCo takes the MJCF model, same stem
    assert an("R1_lidar_nav", "mujoco") == "lidar_nav.xml"
    assert an("A1_husky_swarm_10", "mujoco") == "husky_swarm_10.xml"
    assert external_agent.artifact_suffixes("mujoco") == (".xml",)
    # A task that arm cannot express has NO convention there -- it must never
    # inherit the .wbt name, which would stage a file MuJoCo cannot load and
    # then score the load failure as the agent's.
    assert an("C2_fall_through_floor", "mujoco") is None
    assert an("R3_pick_and_place", "mujoco") is None
    # ...and an unregistered task is still None on the default arm
    assert an("Z9_not_a_task") is None


def test_the_mujoco_filename_registry_matches_what_that_arm_can_express():
    """One authority for "which tasks does MuJoCo cover", not two.

    A name registered for a task the arm cannot express would stage a
    deliverable for a cell ``require_implemented`` refuses; a task the arm
    covers with no registered name would be refused at COLLECTION time
    instead -- after the tokens were spent.
    """
    named = set(external_agent.ARTIFACT_NAME_BY_SIM["mujoco"])
    expressible = set(sims_mod.SIMS["mujoco"].tasks or ())
    assert named == expressible, (
        "sims.SIMS['mujoco'].tasks and ARTIFACT_NAME_BY_SIM['mujoco'] "
        "disagree: %s" % (named ^ expressible))


def test_discover_artifact_finds_the_model_on_the_mujoco_arm(tmp_path):
    """`.xml` is discovered there, `.wbt` here, and neither leaks."""
    ws = tmp_path / "ws"
    ws.mkdir()
    start = time.time() - 10
    model = ws / "arena.xml"
    model.write_text("<mujoco/>", encoding="utf-8")
    world = ws / "arena.wbt"
    world.write_text("WorldInfo {}", encoding="utf-8")
    for p in (model, world):
        os.utime(p, (start + 5, start + 5))

    got, rule = cell.discover_artifact(ws, [], start, suffixes=(".xml",))
    assert got == model, "the MuJoCo arm must collect the MJCF, got %s" % got
    assert ".xml" in rule, rule
    # the default is unchanged, so the other two arms cannot see the .xml
    got2, _ = cell.discover_artifact(ws, [], start)
    assert got2 == world
    # ...and the injected-copy and mtime gates still apply on the new suffix
    stale = ws / "stale.xml"
    stale.write_text("<mujoco/>", encoding="utf-8")
    os.utime(stale, (start - 100, start - 100))
    injected = ws / "_agentbench_x.xml"
    injected.write_text("<mujoco/>", encoding="utf-8")
    os.utime(injected, (start + 99, start + 99))
    got3, _ = cell.discover_artifact(ws, [], start, suffixes=(".xml",))
    assert got3 == model


def test_collect_driver_uses_the_adapters_rule_and_refuses_to_guess(tmp_path):
    """ONE rule for finding the driver -- the adapter's, not a second copy."""
    d = tmp_path / "authored"
    d.mkdir()
    model = d / "arena.xml"
    model.write_text("<mujoco/>", encoding="utf-8")
    (d / "arena.py").write_text("print('step')\n", encoding="utf-8")
    dest = tmp_path / "artifact" / "lidar_nav.xml"
    dest.parent.mkdir()

    rec = cell.collect_driver(model, dest)
    assert rec["to"] == str(dest.with_suffix(".py")), (
        "the driver travels under the COLLECTED model's stem, so the same "
        "discovery rule finds it again at grading time")
    assert Path(rec["to"]).read_text(encoding="utf-8") == "print('step')\n"
    assert "stem" in rec["rule"]

    # Several candidates and none matching the stem: the adapter REFUSES to
    # guess (a wrong pick reads as an agent whose robot did nothing), and the
    # refusal is recorded rather than turned into a silent empty result.
    d2 = tmp_path / "ambiguous"
    d2.mkdir()
    m2 = d2 / "scene.xml"
    m2.write_text("<mujoco/>", encoding="utf-8")
    for n in ("drive.py", "helper.py"):
        (d2 / n).write_text("x\n", encoding="utf-8")
    dest2 = tmp_path / "artifact2" / "lidar_nav.xml"
    dest2.parent.mkdir()
    rec2 = cell.collect_driver(m2, dest2)
    assert rec2["found"] is None and rec2["to"] is None
    assert "refusing to guess" in rec2["rule"]
    assert not dest2.with_suffix(".py").exists(), \
        "nothing was invented on disk either"


def test_a_mujoco_cell_collects_the_model_and_the_program_that_steps_it(
        tmp_path, monkeypatch):
    """The cell-level regression: a MuJoCo cell must not ship an inert scene.

    The session authors ``arena.xml`` + ``arena.py``; collection must produce
    the PAIR under the task's registered stem, and the row must say which
    program came with it and by which rule -- otherwise a cell graded without
    a driver is indistinguishable from a robot that did not move.
    """
    run_dir = tmp_path / "out"
    cc_meta = {"permission_mode": "x", "cli_command": "claude -p", "rc": None,
               "timed_out": True, "wall_s": 900.0, "launch_error": None}

    def fake_session(prompt, ws, env, run_dir_, *, model, timeout_s, **kw):
        m = Path(ws) / "arena.xml"
        m.write_text("<mujoco>\n  <worldbody>\n%s  </worldbody>\n</mujoco>\n"
                     % _r1_obstacles_mjcf(), encoding="utf-8")
        d = Path(ws) / "arena.py"
        d.write_text("print('drive')\n", encoding="utf-8")
        later = time.time() + 5
        os.utime(m, (later, later))
        return None, dict(cc_meta)

    _minimal_cell_env(monkeypatch, tmp_path, staged=[], cc_meta=cc_meta)
    monkeypatch.setattr(cell, "run_claude_cell", fake_session)

    merged = cell.run_cell("mujoco", "R1_lidar_nav", root=tmp_path / "root",
                           out_dir=run_dir, use_locks=False, repeat=0)

    art = run_dir / "artifact"
    assert (art / "lidar_nav.xml").is_file(), "the model was not collected"
    assert (art / "lidar_nav.py").read_text(encoding="utf-8") \
        == "print('drive')\n", (
        "the program that steps the model must travel with it -- an MJCF "
        "scene alone cannot move")
    drv = merged["agent_artifacts"]["collected_driver"]
    assert drv["found"].endswith("arena.py")
    assert drv["to"].endswith("lidar_nav.py")
    assert drv["rule"], "the discovery rule is published on the row"
    # ...and no .wbt machinery fired on this arm
    assert merged["agent_artifacts"]["collected_controllers"] == []
    head = merged["agent_artifacts"]["headless"]
    assert head["arm"] == "mujoco" and head["enforced"] is True
    assert "no GUI" in head["mechanism"]


def test_a_task_mujoco_cannot_express_is_refused_before_a_cell_is_staged(
        tmp_path, monkeypatch):
    """A missing FIXTURE is ours, and must never be scored as their failure.

    SPEC 6.4. The refusal has to happen BEFORE the workspace and before a
    token is spent, and it has to name what the arm does cover.
    """
    run_dir = tmp_path / "out"
    _minimal_cell_env(monkeypatch, tmp_path, staged=["fall_through.wbt"],
                      cc_meta={})
    with pytest.raises(NotImplementedError) as exc:
        cell.run_cell("mujoco", "C2_fall_through_floor",
                      root=tmp_path / "root", out_dir=run_dir,
                      use_locks=False, repeat=0)
    msg = str(exc.value)
    assert "cannot express" in msg and "C2_fall_through_floor" in msg
    assert "R1_lidar_nav" in msg, "the refusal names what the arm DOES cover"
    assert not (run_dir / "rows.jsonl").exists(), \
        "our missing fixture is not a FAIL row attributed to MuJoCo"
    assert not (tmp_path / "root" / "instances").exists(), \
        "refused before any workspace was staged"
    # ...and the arm still runs every task it CAN express
    for tid in sims_mod.SIMS["mujoco"].tasks:
        sims_mod.require_implemented("mujoco", tid)


def test_a_campaign_skips_unexpressible_groups_without_calling_it_blocked(
        tmp_path, monkeypatch):
    """`not_expressible` is a THIRD status, and it must stay third.

    `done` puts a cell in the denominator; `blocked` says the instrument
    broke. A task whose fixture does not exist on an arm is neither -- it is
    OUR gap, and recording it as either would either score MuJoCo for a cell
    it never ran or read as MuJoCo failing.
    """
    from agentbench.cc_lane import run_campaign_cc as camp

    ran = []
    monkeypatch.setattr(camp.cell_mod, "run_cell",
                        lambda sim, task, **kw: ran.append((sim, task)) or
                        dict(_grader_row_stub(), sim=sim, task=task))
    monkeypatch.setattr(camp.staging, "sweep_pending_deletes",
                        lambda root: {"deleted": [], "failed": []})
    monkeypatch.setattr(camp, "publish_run", lambda d: d)

    class _Args:
        campaign_id = "t"
        lane = "A"
        groups = "R1_lidar_nav:mujoco,C2_fall_through_floor:mujoco"
        sim = "mujoco"
        n = 1
        n_a1 = 1
        model = "claude-opus-5"
        root = str(tmp_path / "root")
        lock_root = None
        engine_slots = 1
        timeout_s = 60.0
        rate_limit_backoff_s = 1.0
        max_rate_limit_retries = 0
        out = str(tmp_path / "camp")
        min_free_gb = 0.0
        resource_wait_s = 0.0

    c = camp.Campaign.__new__(camp.Campaign)
    c.args = _Args()
    c.dir = tmp_path / "camp"
    c.dir.mkdir(parents=True)
    c.groups = camp.parse_groups(_Args.groups)
    c.state = {"cells": {}, "published": {}}
    c._save = lambda: None
    c._wait_for_resources = lambda: {"ok": True}
    c.run()

    assert ran == [("mujoco", "R1_lidar_nav")], \
        "only the expressible task may reach the runner: %s" % (ran,)
    statuses = {k: v["status"] for k, v in c.state["cells"].items()}
    skipped = [k for k, s in statuses.items() if s == "not_expressible"]
    assert len(skipped) == 1 and "C2_fall_through_floor" in skipped[0]
    assert "blocked" not in statuses.values(), \
        "our missing fixture is not an instrument failure"
    assert "cannot express" in c.state["cells"][skipped[0]]["reason"]


# --- the budget must be MEASURED, not asserted -------------------------------
#
# THREE separate budget bugs shipped on 2026-08-09, each because the STATED
# guarantee was stronger than the ENFORCED one, and each found by a human
# noticing a clock rather than by a test:
#
#   1. the per-session timeout never fired. `subprocess.run(
#      capture_output=True, timeout=...)` reaps a timed-out child with
#      `communicate()` and NO timeout of its own, so a grandchild still
#      holding the inherited pipe blocks the reap for ever. A webots A1 cell
#      reached 28.5 minutes against a 15-minute cap (fixed 4700dbaf6).
#   2. timed-out cells produced NO ROW and silently left the pass@1
#      denominator -- so a simulator that ran out of time scored BETTER than
#      one that finished and was wrong (fixed cdac451f4).
#   3. the cap bounded ONE SESSION, so the deferral loop multiplied it: 12
#      retries x (30 min + 15 min backoff) is about NINE HOURS for one cell,
#      while the docstring claimed `cells x ceiling` was the worst case
#      (fixed 43c2f3446).
#
# Every existing budget test asserts a mechanism (this branch runs, that
# guard fires). None of them MEASURES the thing the guarantee is about, which
# is why all three shipped. This one drives a real `run_cell` against a
# session that behaves as badly as the real ones did -- it ignores its kill
# signals, leaves a grandchild holding the stdio it inherited, and (in one
# mode) refuses with a rate limit -- and then asserts the only property that
# matters: the cell's TOTAL wall clock is inside the bound the tree declares,
# and the cell ended in a row or a blocker, never in silence.
#
# The budget is scaled DOWN for the test rather than waited out. "Total
# elapsed <= declared bound" is scale-free, so a 6-second ceiling proves the
# same property as a 30-minute one in 1/300th of the time.

#: The task ceiling this test scales the whole budget system down to. Small
#: enough to be fast, large enough that ordinary process startup is not the
#: thing being measured.
SCALED_CEILING_S = 6.0

#: The scaled-down stand-in for ``run_cc_cell.CELL_WALL_ALLOWANCE_S`` (900 s
#: shipped). Kept BELOW the ceiling and well under the 60 s constant in
#: ``can_retry``'s room check, so the `rate_limited` mode still runs out of
#: room and abandons the cell rather than looping through 12 deferrals.
SCALED_ALLOWANCE_S = 4.0

#: Fixed per-cell instrument cost that does NOT scale with the budget:
#: workspace staging, the port/process sweeps, teardown, writing the row.
#: Measured at ~1.3 s on machine 9722d23d12a3; the allowance is generous so
#: the test fails on a broken bound rather than on a slow disk, and it is a
#: CONSTANT -- it never grows with the budget, so it cannot hide a multiplier.
INSTRUMENT_OVERHEAD_S = 5.0

#: How long the grandchild holds the stdio it inherited. Must exceed
#: `bound + INSTRUMENT_OVERHEAD_S`, or bug 1 would fit inside the ceiling and
#: the test would pass against the code that shipped it.
GRANDCHILD_HOLDS_STDIO_S = 40.0

_BADLY_BEHAVED_CLAUDE = '''\
import signal, subprocess, sys, time

# 1. ignore every kill signal we are allowed to ignore
for _name in ("SIGINT", "SIGTERM", "SIGHUP", "SIGBREAK"):
    _sig = getattr(signal, _name, None)
    if _sig is not None:
        try:
            signal.signal(_sig, signal.SIG_IGN)
        except (OSError, ValueError, RuntimeError):
            pass

# 2. leave a GRANDCHILD holding the stdout/stderr we inherited. It outlives
#    us, which is what makes an unbounded communicate() reap block for ever.
subprocess.Popen([sys.executable, "-c",
                  "import time; time.sleep(%(grandchild_s)s)"])

# 3. refuse, or hang, or both
sys.stderr.write(%(stderr)r)
sys.stderr.flush()
time.sleep(%(self_sleep_s)s)
sys.exit(1)
'''


def _install_badly_behaved_claude(tmp_path, *, stderr_text, self_sleep_s,
                                  grandchild_s):
    """A fake ``claude`` EXECUTABLE that misbehaves the way the real one did.

    A real executable, not a monkeypatched function: the reap-blocking bug
    lives in how the parent waits on a process TREE, so a test that stubs the
    call out cannot see it.
    """
    script = tmp_path / "badly_behaved_claude.py"
    script.write_text(_BADLY_BEHAVED_CLAUDE % {
        "grandchild_s": grandchild_s, "stderr": stderr_text,
        "self_sleep_s": self_sleep_s}, encoding="utf-8")
    if os.name == "nt":
        exe = tmp_path / "claude.bat"
        exe.write_text('@echo off\r\n"%s" "%s" %%*\r\n'
                       % (sys.executable, script), encoding="utf-8")
    else:
        exe = tmp_path / "claude"
        exe.write_text('#!/bin/sh\nexec "%s" "%s" "$@"\n'
                       % (sys.executable, script), encoding="utf-8")
        exe.chmod(0o755)
    return exe


@pytest.mark.parametrize("mode", ["hangs_past_its_budget", "rate_limited"])
def test_a_badly_behaved_session_cannot_outlive_the_declared_ceiling(
        mode, tmp_path, monkeypatch):
    """The cell's TOTAL wall clock, measured, against the bound it declares.

    Both modes are things a real session did on the day a budget bug was
    found: one hangs past its budget while a grandchild holds the stdio, the
    other refuses with a usage limit and lets the deferral loop have it.

    The two endings are deliberately different and both are asserted, because
    conflating them is its own defect:

    * **budget exhausted is the AGENT's result** -- it lands a row, FAIL, and
      stays in the pass@1 denominator (bug 2 dropped exactly these cells, and
      they are precisely the ones the agent was losing);
    * **an out-of-room rate limit is the INSTRUMENT's** -- a blocker, no row,
      because scoring it would attribute our own capacity to the agent.

    What must never happen in either case is silence: no row, no blocker, and
    a machine still busy.
    """
    # Scale the whole budget system down. Task.timeout_s clamps to this, so
    # one constant moves the session budget AND the cell ceiling together.
    monkeypatch.setattr(tasks_mod, "TASK_HARD_CEILING_S", SCALED_CEILING_S)
    # The cell wall bound is `task budget + a FIXED allowance` (the allowance
    # pays for staging, preflight, sweeps, placement, grading and teardown,
    # none of which scale with the agent's budget). A fixed term does not
    # scale down with the ceiling, so it is scaled explicitly here -- both
    # halves of the bound have to shrink together or the test measures the
    # allowance instead of the guarantee.
    monkeypatch.setattr(cell, "CELL_WALL_ALLOWANCE_S", SCALED_ALLOWANCE_S)
    task_id = "B1_overlap_audit"
    task_budget = float(tasks_mod.get(task_id).timeout_s)
    assert task_budget == SCALED_CEILING_S, "the scale-down did not take"

    # The bound the TREE declares, read from the tree. Code that declares no
    # bound at all is claiming `cells x ceiling` -- which is exactly what
    # tasks/__init__ says the campaign's worst case is -- so the task budget
    # itself is the honest default.
    bound_f = getattr(cell, "cell_wall_bound_s", lambda b: float(b))
    bound_s = float(bound_f(task_budget))

    if mode == "hangs_past_its_budget":
        stderr_text = "working...\n"
        self_sleep_s = 60.0
        grandchild_s = GRANDCHILD_HOLDS_STDIO_S
    else:
        # Exits at once so the refusal is READ, and still leaves a grandchild
        # behind. The deferral loop then has something to multiply.
        stderr_text = ("Claude usage limit reached. Your limit will reset at "
                       "7:10pm.\n")
        self_sleep_s = 0.0
        grandchild_s = 2.0
    exe = _install_badly_behaved_claude(
        tmp_path, stderr_text=stderr_text, self_sleep_s=self_sleep_s,
        grandchild_s=grandchild_s)

    # Everything EXCEPT the session is stubbed -- and the session is put back,
    # because the real `run_claude_cell` -> `_run_claude` path is the code
    # under test here.
    real_session = cell.run_claude_cell
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta={})
    monkeypatch.setattr(cell, "run_claude_cell", real_session)
    monkeypatch.setattr(cell, "_claude_exe", lambda: str(exe))

    run_dir = tmp_path / "out"
    row, blocker = None, None
    t0 = time.monotonic()
    try:
        row = cell.run_cell("omnisim", task_id, root=tmp_path / "root",
                            out_dir=run_dir, use_locks=False,
                            rate_limit_backoff_s=2.0, repeat=0)
    except SystemExit as exc:
        blocker = str(exc)
    elapsed = time.monotonic() - t0

    # 1. THE property. Everything else in this file asserts a mechanism; this
    #    asserts the guarantee.
    assert elapsed <= bound_s + INSTRUMENT_OVERHEAD_S, (
        "the cell ran %.1f s against a declared ceiling of %.1f s "
        "(the %.1f s task budget + the %.1f s cell wall allowance) + %.1f s "
        "of fixed instrument overhead. A ceiling that a badly-behaved session "
        "can walk through is not a ceiling, and the campaign's cost bound is "
        "fiction."
        % (elapsed, bound_s, task_budget, SCALED_ALLOWANCE_S,
           INSTRUMENT_OVERHEAD_S))

    # 2. ...and the cell ended in something. Never silence.
    report_path = run_dir / "cell_report.json"
    assert report_path.is_file(), "the cell left no report at all"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    rows_path = run_dir / "rows.jsonl"
    assert (row is not None) or (blocker is not None), \
        "the cell neither produced a row nor blocked -- that is silence"

    if mode == "hangs_past_its_budget":
        # The session really did misbehave (guards against a future refactor
        # where the fake silently stops being exercised).
        assert report["cc_meta"]["timed_out"] is True, \
            "the badly-behaved session was not actually run to its cap"
        assert blocker is None, (
            "budget exhaustion is the AGENT's result, not a broken "
            "instrument: %s" % blocker)
        assert row is not None and row["outcome"] == "FAIL"
        assert row["agent_artifacts"]["budget_exhausted"] is True
        lines = [l for l in rows_path.read_text(encoding="utf-8").splitlines()
                 if l.strip()]
        assert len(lines) == 1, (
            "a timed-out cell MUST land a row (SPEC 2.4) -- dropping it "
            "removes the cell from the pass@1 denominator, and the cells "
            "that time out are the ones the agent was losing")
        # nothing was invented from a session that produced no JSON
        assert row["metrics"]["usd"] is None
        assert row["metrics"]["tokens_out"] is None
        # ...and the row says how many times this cell was run. At one run
        # per (task, arm) a reader who does not find `samples: 1` on the row
        # has no way to tell an outcome from a rate, which is the whole
        # hazard the single-run protocol introduces.
        proto = row["protocol"]
        assert proto["samples"] == 1
        assert proto["runs_per_cell"] == 1
        assert proto["variance_measured"] is False
        assert proto["is_rate"] is False
        assert proto["id"] == cell.PROTOCOL_ID
        assert proto["cell_wall_bound_s"] == bound_s
        assert proto["hard_ceiling_s"] == SCALED_CEILING_S
    else:
        assert blocker is not None, (
            "an out-of-room rate limit must abandon the cell, not keep "
            "retrying inside it")
        assert "wall ceiling" in blocker, blocker
        assert not rows_path.exists(), (
            "an instrument limit is NOT an agent FAIL and must not land a "
            "row")
        assert report.get("blocker"), "the blocker was not recorded"


# --- the single-run protocol, and the claim it must never license ------------
#
# The protocol changed on 2026-08-10 from "n repeats per cell, aggregated"
# to "ONE run per (task, arm) under a wall-clock ceiling" (owner's decision;
# repeats were the dominant token cost). Everything below guards the ONE
# property that change puts at risk: a single observation must not be
# readable, by a person or by a downstream aggregator, as a converged rate.
#
# It is not a hypothetical misreading. The suite's own report generator prints
# `pass@1` with a Wilson interval, and its inputs are rows exactly like these;
# the difference between "3/5, CI [0.15, 0.95]" and "passed once" is the
# difference between a measurement and an anecdote, and nothing in a row said
# which it was until the `protocol` block existed.


def test_the_protocol_default_is_one_run_per_cell():
    """The campaign schedules ONE cell per (task, sim) -- both tiers."""
    from agentbench.cc_lane import run_campaign_cc as camp
    assert camp.DEFAULT_N == 1
    assert camp.DEFAULT_N_A1 == 1, (
        "A1's old n = 10 is gone with the rest; the flagship does not get a "
        "different protocol from everything else")
    assert cell.PROTOCOL_RUNS_PER_CELL == 1
    # ...and repeating remains POSSIBLE, so a deliberate variance experiment
    # is not blocked by the default.
    assert camp.n_for("B1_overlap_audit", 5, 1) == 5
    assert camp.n_for("A1_husky_swarm_10", 1, 7) == 7


def test_no_task_declares_a_budget_the_ceiling_would_truncate():
    """`min(3 x par, ceiling)` with the ceiling at 45 min truncates nothing.

    Not a decorative check. `TASK_HARD_CEILING_S` is GLOBAL, so it silently
    rewrites every task's budget at once, and a `meta.json` still carrying a
    stale baked-in truncation would hand its task a budget that the rule says
    it should not have. A1 shipped exactly that: `timeout_s: 1800` was the
    900->1800 ceiling's truncation frozen into the file, and it would have
    kept A1 at 2.5x par under a ceiling that grants 3x.
    """
    for tid, task in sorted(tasks_mod.discover().items()):
        assert task.timeout_s == min(3 * task.par_s,
                                     tasks_mod.TASK_HARD_CEILING_S), tid
        meta_budget = task.meta.get("budget") or {}
        assert meta_budget.get("hard_ceiling_s") == \
            tasks_mod.TASK_HARD_CEILING_S, (
                "%s's meta.json records a ceiling of %s while the code uses "
                "%s -- the recorded budget is then a description of a run "
                "nobody made" % (tid, meta_budget.get("hard_ceiling_s"),
                                 tasks_mod.TASK_HARD_CEILING_S))
        assert meta_budget.get("runs_per_cell") == 1, tid
        assert meta_budget.get("variance_measured") is False, tid


def test_the_cell_wall_bound_is_the_budget_plus_a_fixed_allowance():
    """Additive, not a multiple -- and the difference is the point.

    A multiplier scales the allowance with the AGENT's budget; the things it
    pays for (staging, preflight, sweeps, placement, the grading launch,
    teardown) cost the same either way. The old `x 2.0` carried onto a
    45-minute ceiling would have licensed a 90-minute cell.
    """
    assert cell.CELL_WALL_ALLOWANCE_S > 0
    for budget in (720.0, 1800.0, 2700.0):
        assert cell.cell_wall_bound_s(budget) == \
            budget + cell.CELL_WALL_ALLOWANCE_S
    # the allowance is a constant: doubling the budget must not double it
    small = cell.cell_wall_bound_s(720.0) - 720.0
    large = cell.cell_wall_bound_s(2700.0) - 2700.0
    assert small == large, (
        "the allowance grew with the budget -- that is a multiplier wearing "
        "an additive name")
    # ...and one rate-limit deferral still fits, so a retried cell is not
    # silently handed a shortened session.
    assert cell.CELL_WALL_ALLOWANCE_S >= cell.DEFAULT_RATE_LIMIT_BACKOFF_S


def test_nothing_in_the_lane_calls_a_single_observation_a_rate():
    """No `pass@1` / `n = 5` / `mean over repeats` wording left in cc_lane.

    Wording is not cosmetic here. A comment or a help string that still says
    "the pass@1 denominator" tells the next reader the suite aggregates
    repeats, and the suite no longer does; that reader then writes a report
    header that is false about the evidence. The `protocol` block's own note
    is exempt -- it names those terms precisely to say they are UNDEFINED.
    """
    import re
    # A term is allowed where the text DENIES or historicises it. The check is
    # over a small window, not the matched line, because the denial is often
    # the next clause of the same sentence and a one-line rule would force the
    # prose to be worse than the rule.
    denials = ("undefined", "not a rate", "never measured", "used to be",
               "is gone", "no longer", "superseded")
    lane = Path(cell.__file__).parent
    offenders = []
    for p in sorted(lane.glob("*.py")):
        if p.name == "test_cc_lane.py":
            continue
        lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
        for i, line in enumerate(lines):
            if not re.search(r"pass@1|mean over repeats|n ?= ?5|n ?= ?10",
                             line):
                continue
            window = " ".join(lines[max(0, i - 2):i + 3]).lower()
            if any(d in window for d in denials):
                continue
            offenders.append("%s:%d: %s" % (p.name, i + 1, line.strip()))
    assert not offenders, (
        "these lines still imply the suite aggregates repeats:\n  "
        + "\n  ".join(offenders))


# --- the deliverable is a PROJECT, and OUR OWN published assets are part of it
#
# MEASURED 2026-08-11, R4's first webots cell. The lane collects <world>.wbt
# plus every controllers/<name>/ the world declares, and nothing else -- while
# the agent worked in a workspace where THIS LANE had planted the task's
# published assets at the workspace root. The agent's controller opened
# `../../benchmark_assets/scene.json` at __init__ (the only relative path that
# resolves from a controller directory), which worked in its workspace and
# raised FileNotFoundError in the graded copy: the controller exited 1, its
# robot never moved, and the run recorded a base path length of 0.00 m over
# 150 s on a world whose scene was otherwise exact to 0.000 m. That is a
# property of the LANE and it applies to R1/R2/R3/R4 identically.
#
# These tests pin the fix (`run_cc_cell.stage_task_assets`) AND its safety
# rule: the bytes come from the frozen task tree in the repo, never from the
# agent's workspace, so the channel cannot carry a pre-computed answer.

#: The one line that killed the R4 cell, verbatim in shape.
_ASSET_READING_CONTROLLER = (
    "import json, os\n"
    "here = os.path.dirname(os.path.abspath(__file__))\n"
    "path = os.path.join(here, '..', '..', 'benchmark_assets', 'scene.json')\n"
    "with open(path) as f:\n"
    "    scene = json.load(f)\n"
    "print('OK', len(scene))\n"
)


def _grade_project_with_asset_reading_controller(root, name="warehouse"):
    """A graded project shaped exactly as the lane builds one: the collected
    controller under `controllers/<name>/`, and the world under `worlds/`."""
    proj = Path(root) / "grade"
    cdir = proj / "controllers" / name
    cdir.mkdir(parents=True)
    (cdir / (name + ".py")).write_text(_ASSET_READING_CONTROLLER,
                                       encoding="utf-8")
    (proj / "worlds").mkdir(parents=True)
    return proj, cdir / (name + ".py")


@pytest.mark.parametrize("sim", ["omnisim", "webots"])
def test_a_controller_reading_a_published_asset_starts_in_the_graded_project(
        tmp_path, sim):
    """THE regression. Run the controller's own path arithmetic for real.

    The assertion is not "a file was copied" -- it is that the exact program
    shape that died still runs. And it is proven able to go RED first: with
    the staging step skipped, the same controller raises FileNotFoundError,
    which is the 0.00 m run.
    """
    # ...RED without the fix
    _proj, ctrl = _grade_project_with_asset_reading_controller(
        tmp_path / ("red_" + sim))
    red = subprocess.run([sys.executable, str(ctrl)], capture_output=True,
                         text=True)
    assert red.returncode != 0 and "FileNotFoundError" in red.stderr, (
        "this test cannot detect the defect it exists for -- the unstaged "
        "controller was expected to die exactly as R4's did")

    # ...GREEN with it
    proj, ctrl = _grade_project_with_asset_reading_controller(
        tmp_path / ("green_" + sim))
    rec = cell.stage_task_assets("R4_mobile_manipulation", sim, proj)
    green = subprocess.run([sys.executable, str(ctrl)], capture_output=True,
                           text=True)
    assert green.returncode == 0, (
        "a controller reading a PUBLISHED task asset still cannot start in "
        "the graded project: %s\nstaged: %s"
        % (green.stderr.strip(), [f["path"] for f in rec["files"]]))
    assert green.stdout.startswith("OK")


@pytest.mark.parametrize("sim", ["omnisim", "webots"])
def test_published_task_assets_land_at_their_workspace_relative_paths(
        tmp_path, sim):
    """Same relative path in the graded project as in the workspace.

    That equality is the whole mechanism: the graded project root plays the
    part the workspace root played, so no reference has to be rewritten.
    """
    ws = tmp_path / "ws"
    ws.mkdir()
    _prompt, staged = staging.stage_task(ws, "R4_mobile_manipulation", sim)
    ws_rel = sorted(Path(p).relative_to(ws).as_posix() for p in staged)

    proj = tmp_path / "grade"
    proj.mkdir()
    rec = cell.stage_task_assets("R4_mobile_manipulation", sim, proj)
    assert sorted(f["path"] for f in rec["files"]) == ws_rel != []
    for rel in ws_rel:
        assert (proj / rel).read_bytes() == (ws / rel).read_bytes()


def test_the_graded_asset_copy_comes_from_the_repo_never_the_workspace(
        tmp_path):
    """The SAFETY RULE, pinned: this channel cannot carry agent bytes.

    An agent that overwrites its copy of a published asset -- with a
    pre-computed answer, a doctored scene, anything -- must not see that copy
    reach the grader. The source is the frozen task tree and nothing else, so
    a tampered workspace (and a tampered collected artifact) are simply not
    read.
    """
    ws = tmp_path / "ws"
    ws.mkdir()
    staging.stage_task(ws, "R4_mobile_manipulation", "omnisim")
    tampered = b'{"SMUGGLED": "a pre-computed answer"}'
    (ws / "benchmark_assets" / "scene.json").write_bytes(tampered)
    art = tmp_path / "artifact" / "benchmark_assets"
    art.mkdir(parents=True)
    (art / "scene.json").write_bytes(tampered)

    proj = tmp_path / "grade"
    proj.mkdir()
    cell.stage_task_assets("R4_mobile_manipulation", "omnisim", proj)

    frozen = (tasks_mod.get("R4_mobile_manipulation").dir / "initial"
              / "benchmark_assets" / "scene.json").read_bytes()
    got = (proj / "benchmark_assets" / "scene.json").read_bytes()
    assert got == frozen, "the graded copy is not the published one"
    assert got != tampered, "an agent's bytes reached the graded project"


def test_published_assets_never_shadow_the_deliverable(tmp_path,
                                                       monkeypatch):
    """`controllers/` and `worlds/` belong to the thing being measured.

    A task whose published tree contained either would otherwise overwrite the
    agent's collected code (or the staged world) with ours, which is not a
    packaging fix, it is grading our own file.
    """
    init = tmp_path / "tasks" / "T_shadow" / "initial"
    (init / "controllers" / "warehouse").mkdir(parents=True)
    (init / "controllers" / "warehouse" / "warehouse.py").write_text(
        "# OURS, must not be staged\n", encoding="utf-8")
    (init / "worlds").mkdir(parents=True)
    (init / "worlds" / "w.wbt").write_text("# OURS\n", encoding="utf-8")
    (init / "benchmark_assets").mkdir(parents=True)
    (init / "benchmark_assets" / "scene.json").write_text(
        "{}", encoding="utf-8")

    proj, ctrl = _grade_project_with_asset_reading_controller(tmp_path)
    mine = ctrl.read_bytes()

    files = staging.published_task_files("T_shadow", "omnisim",
                                         tasks_dir=tmp_path / "tasks")
    assert len(files) == 3, "the fixture must exercise both reserved roots"

    monkeypatch.setattr(staging, "published_task_files",
                        lambda t, s: files)
    monkeypatch.setattr(staging, "task_initial_dir", lambda t, s: init)
    rec = cell.stage_task_assets("T_shadow", "omnisim", proj)

    assert [f["path"] for f in rec["files"]] == ["benchmark_assets/scene.json"]
    assert sorted(r["path"] for r in rec["refused"]) == [
        "controllers/warehouse/warehouse.py", "worlds/w.wbt"]
    assert ctrl.read_bytes() == mine, "the deliverable was overwritten"
    assert not (proj / "worlds" / "w.wbt").exists()


def test_every_task_that_publishes_a_runtime_asset_is_covered_on_both_arms():
    """The defect is per-LANE, so the fix must be too.

    R1/R3/R4 all publish a `benchmark_assets/` tree that a controller can
    plausibly read at run time. Whatever the workspace got, the graded project
    must get -- on every `.wbt` arm, or the arms are not comparable.
    """
    for task_id in ("R1_lidar_nav", "R3_pick_and_place",
                    "R4_mobile_manipulation"):
        for sim in ("omnisim", "webots"):
            pub = staging.published_task_files(task_id, sim)
            assets = [rel for rel, _ in pub
                      if rel.startswith("benchmark_assets/")]
            assert assets, ("%s/%s publishes no benchmark asset -- if that is "
                            "intended, this list is stale" % (task_id, sim))


def test_the_webots_run_script_carries_project_assets_into_wsl():
    """Only `worlds/` and `controllers/` used to travel into the WSL workdir.

    Staging the asset into the Windows-side project root is half the fix; the
    upstream arm runs in a copy of the project inside WSL, and a file that
    does not cross is a file the controller cannot open.
    """
    from agentbench.adapters.webots import launcher

    kw = dict(work_dir="/tmp/x", project_src_wsl="/mnt/o/proj",
              injected_world_wsl="/mnt/o/run/_inj.wbt",
              recorder_src_wsl="/mnt/o/rec", run_dir_wsl="/mnt/o/run",
              world_name="w.wbt", perf_steps=10)
    base = launcher.build_run_script(**kw)
    with_assets = launcher.build_run_script(
        extra_project_paths=("benchmark_assets/scene.json",
                             "benchmark_assets/obstacles.json"), **kw)

    assert base == launcher.build_run_script(extra_project_paths=(), **kw), \
        "the default must build a byte-identical script for every old caller"
    for rel in ("benchmark_assets/scene.json",
                "benchmark_assets/obstacles.json"):
        assert rel not in base, "fixture is wrong: the asset was already there"
        assert ('cp -r "/mnt/o/proj/%s" "$WORK/project/%s"' % (rel, rel)
                in with_assets), "%s never crosses into the WSL project" % rel
    assert 'mkdir -p "$WORK/project/benchmark_assets"' in with_assets, \
        "the parent directory must exist before the copy"


def test_the_webots_run_script_refuses_a_traversing_asset_path():
    """This list decides what crosses into the run, so it is validated.

    `..` or an absolute path would let a caller copy something outside the
    graded project into the workdir. Nothing does today; the guard is here so
    nothing can.
    """
    from agentbench.adapters.webots import launcher

    s = launcher.build_run_script(
        work_dir="/tmp/x", project_src_wsl="/mnt/o/proj",
        injected_world_wsl="/i.wbt", recorder_src_wsl="/r",
        run_dir_wsl="/run", world_name="w.wbt", perf_steps=10,
        extra_project_paths=("../secrets", "/etc/passwd", "a/../../b", ""))
    assert "secrets" not in s and "passwd" not in s and "/../" not in s


# --- CONTAINMENT: the workspace filter protects a COPY, not the machine ------
#
# MEASURED 2026-08-11 on the first R4/omnisim cell. `stage_workspaces` removes
# `tests/benchmarks/agentbench/` from the workspace because it holds the
# graders, their thresholds, the oracle fixtures and the task metas. The
# session ran `ls -la /o/omnisim/`, found the source tree at its real path,
# and read `graders/r4_core.py` (twice, the second read offset onto the
# assertion bodies), `graders/r4.py` and the task's `meta.json`. 22 tool calls
# reached the tree. That cell's score is not a measurement of an agent -- and
# before this gate, its report looked exactly like a clean one.
#
# The gate cannot PREVENT it (the lane cannot path-confine `run_shell` on a
# bare host, and the product docs we deliberately stage name the checkout);
# what it does is make it impossible for it to happen silently again.


def _ndjson(path, tool_calls):
    """A Claude Code NDJSON file carrying `tool_calls` as assistant blocks."""
    lines = []
    for name, inp in tool_calls:
        lines.append(json.dumps({
            "type": "assistant",
            "message": {"role": "assistant",
                        "content": [{"type": "tool_use", "id": "t",
                                     "name": name, "input": inp}]}}))
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return Path(path)


def test_containment_catches_a_session_that_read_the_grader(tmp_path):
    """The exact shape that happened, from both path spellings."""
    src = _ndjson(tmp_path / "s.jsonl", [
        ("Bash", {"command": "ls -la /o/omnisim/"}),
        ("Read", {"file_path":
                  "/o/omnisim/tests/benchmarks/agentbench/graders/r4_core.py"}),
        ("Read", {"file_path":
                  "O:\\omnisim\\tests\\benchmarks\\agentbench\\tasks"
                  "\\R4_mobile_manipulation\\meta.json"}),
    ])
    a = cell.audit_containment((src,), tool_calls=3)
    assert a["clean"] is False and a["hit_count"] == 2, a
    assert {h["tool"] for h in a["hits"]} == {"Read"}
    blocked, why = cell.containment_blocks(a)
    assert blocked and "CONTAMINATED" in why and "answer key" in why


def test_containment_passes_a_session_that_stayed_in_the_product(tmp_path):
    """A green here must be a MEASUREMENT, so it has to be reachable.

    Reading the product -- including the OTHER benchmark suites, which are
    documentation an agent is entitled to -- is not contamination. A gate that
    fired on `tests/benchmarks/` wholesale would be unusable and would teach
    the next reader to ignore it.
    """
    src = _ndjson(tmp_path / "s.jsonl", [
        ("Read", {"file_path": "/o/omnisim/AGENTS.md"}),
        ("Bash", {"command": "python -m omnisim doctor"}),
        ("Read", {"file_path":
                  "/o/omnisim/tests/benchmarks/omnibench/lane4/README.md"}),
        ("Read", {"file_path":
                  "/o/omnisim/projects/samples/devices/worlds/lidar.omniworld"}),
    ])
    a = cell.audit_containment((src,), tool_calls=4)
    assert a["clean"] is True, a["hits"]
    assert cell.containment_blocks(a) == (False, None)


def test_containment_needles_come_from_the_staging_exclusion_list():
    """One declaration of "the agent must not see this", two enforcements.

    Staging removes those paths from the workspace; this audit catches a
    session that reached them another way. A second, hand-written list would
    drift, and the drift would be invisible.
    """
    labels = {lab for lab, _ in cell._containment_needles()}
    assert "tests/benchmarks/agentbench/" in labels
    for p in staging.OMNISIM_EXCLUDE_PREFIXES:
        assert p.replace("\\", "/").strip("/").lower() + "/" in labels
    for f in staging.OMNISIM_EXCLUDE_FILES:
        assert f.replace("\\", "/").strip("/").lower() in labels
    # ...and the single-component ones are PATHS, not English words: the gate
    # must not fire on prose, or the next reader learns to ignore it.
    prose = [(lab, pat) for lab, pat in cell._containment_needles()
             if lab == "cloud/"]
    assert prose, "the fixture needs a single-component prefix"
    lab, pat = prose[0]
    assert not pat.search('grep -r "cloud" docs/'),         "the gate fires on the English word, which makes it noise"
    assert pat.search("cd /o/omnisim/cloud/runpod && ls")


def test_an_unreadable_source_is_open_not_clean(tmp_path):
    """"We could not check" must never render as "we checked"."""
    a = cell.audit_containment((tmp_path / "missing.jsonl",), tool_calls=0)
    assert a["clean"] is None and "NOT established" in a["reason"]
    # ...but a session that made NO tool calls could not have read anything,
    # so it is not turned into an instrument failure
    assert cell.containment_blocks(a) == (False, None)


def test_a_session_that_used_tools_we_cannot_see_is_blocked(tmp_path):
    """The dangerous half of `None`: it acted, and the record is gone."""
    a = cell.audit_containment((tmp_path / "missing.jsonl",), tool_calls=57)
    assert a["clean"] is None
    blocked, why = cell.containment_blocks(a)
    assert blocked and "57" in why


@pytest.mark.skipif(
    not (Path(cell.__file__).parents[1] / "results" / "cc_lane"
         / "r4_omnisim_20260811" / "cc_stream.jsonl").is_file(),
    reason="the measured cell is not in this tree")
def test_the_r4_omnisim_cell_that_motivated_this_gate_is_caught():
    """Pinned against the real stream, not a fixture of it.

    A synthetic case proves the matcher; this proves the matcher would have
    caught the run that actually happened, in the shape a live session writes.
    """
    stream = (Path(cell.__file__).parents[1] / "results" / "cc_lane"
              / "r4_omnisim_20260811" / "cc_stream.jsonl")
    a = cell.audit_containment((stream,), tool_calls=1)
    assert a["clean"] is False
    assert a["hit_count"] >= 20, (
        "the measured cell made 22 tool calls into the benchmark tree")
    read = {h["input"] for h in a["hits"] if h["tool"] == "Read"}
    assert any("r4_core.py" in s for s in read), \
        "the grader read is the finding -- it must be in the hits"


# --- the containment GUARD (prevention, not detection) -----------------------
#
# The audit above catches a contaminated cell after the quota is spent. These
# cover the hook that refuses the call in the first place. The decision
# function is pure, so these test exactly what runs in the hook.


def _guard_cfg(tmp_path):
    ws = tmp_path / "instances" / "20260812_omnisim_R4"
    return {
        "workspace": str(ws).replace("\\", "/"),
        "repo": "O:/omnisim",
        "scratch_root": str(tmp_path).replace("\\", "/"),
        "junction_dirs": list(staging.OMNISIM_JUNCTION_DIRS),
        "exclude_prefixes": list(staging.OMNISIM_EXCLUDE_PREFIXES),
        "exclude_files": list(staging.OMNISIM_EXCLUDE_FILES),
        # A run dir OUTSIDE both roots -- the `--out` case. The default run
        # dir lives under the repo's own results tree and is already denied by
        # the exclusion needles, so `protect` is the backstop for the rest.
        "protect": ["D:/cells/r4_cell_1"],
    }


def test_guard_refuses_the_exact_calls_the_measured_cell_made(tmp_path):
    """The 2026-08-11 R4 escape, call for call, now denied before it runs."""
    cfg = _guard_cfg(tmp_path)
    # The refusal TEMPLATE is product language: a session told "you have hit
    # the benchmark's graders" has been told it is being measured, which
    # changes the thing being measured.
    tmpl = guard.DENY_TEMPLATE.lower()
    for word in ("benchmark", "grader", "answer key", "agentbench", "oracle",
                 "score", "contamin"):
        assert word not in tmpl, word
    for tool, ti in [
            ("Bash", {"command": "ls -la /o/omnisim/"}),
            ("Bash", {"command": r"cd O:\omnisim && git log --oneline"}),
            ("Read", {"file_path": "/o/omnisim/tests/benchmarks/agentbench"
                                   "/graders/r4_core.py"}),
            ("Read", {"file_path": r"O:\omnisim\tests\benchmarks\agentbench"
                                   r"\tasks\R4_mobile_manipulation\meta.json"}),
            ("Grep", {"pattern": "threshold",
                      "path": "/mnt/o/omnisim/tests/benchmarks/agentbench"}),
    ]:
        allow, reason, matched = guard.decide(tool, ti, cfg)
        assert allow is False, (tool, ti)
        # The refusal quotes the path back, so those words can appear -- but
        # only ones the AGENT ITSELF typed. It must add none of its own, and
        # it must deny identically for a path that does not exist, so a
        # refusal never confirms that one does.
        low, typed = (reason or "").lower(), guard._norm(json.dumps(ti))
        for word in ("benchmark", "grader", "agentbench", "oracle"):
            assert word not in low or word in typed, (word, reason)
    ghost = "/o/omnisim/tests/benchmarks/agentbench/graders/NOT_A_FILE.py"
    assert guard.decide("Read", {"file_path": ghost}, cfg)[0] is False


def test_guard_admits_the_product_the_user_actually_has(tmp_path):
    """A green here must be reachable, or the guard is just an off switch.

    Engine C++ source is ADMITTED on purpose: OmniSim ships it, a real user
    diagnosing a motor opens it, and it contains no task, threshold, grader or
    oracle. See containment_guard.__doc__ for the full ruling.
    """
    cfg = _guard_cfg(tmp_path)
    ws = cfg["workspace"]
    for tool, ti in [
            ("Read", {"file_path": ws + "/AGENTS.md"}),
            ("Read", {"file_path": ws + "/src/omnisim/nodes/OmBasicJoint.cpp"}),
            ("Read", {"file_path": "src/omnisim/nodes/OmBasicJoint.cpp"}),
            ("Bash", {"command": "python -m omnisim doctor"}),
            ("Bash", {"command": "python -m omnisim run-headless "
                                 "worlds/task.wbt --duration 20"}),
            ("Bash", {"command": 'curl -s -X POST '
                                 'http://127.0.0.1:6789/world/load '
                                 '-d \'{"path":"worlds/task.wbt"}\''}),
            ("Bash", {"command": 'grep -rn "cloud" docs/'}),
            ("Glob", {"pattern": "**/*.wbt"}),
            # the junctioned runtime dirs are the workspace's own bytes
            ("Read", {"file_path": "O:/omnisim/projects/robots/x/y.urdf"}),
    ]:
        allow, reason, matched = guard.decide(tool, ti, cfg)
        assert allow is True, (tool, ti, matched, reason)


def test_guard_denies_other_cells_and_its_own_run_dir(tmp_path):
    """Sibling cells hold predecessors' deliverables; the run dir holds this
    cell's report, its preserved workspace and the guard itself."""
    cfg = _guard_cfg(tmp_path)
    other = str(tmp_path / "instances" / "20260812_omnisim_R1")
    allow, _, matched = guard.decide("Read", {"file_path": other + "/w.wbt"},
                                     cfg)
    assert allow is False and matched == "other_cell_or_template"
    allow, _, matched = guard.decide(
        "Read", {"file_path": "D:/cells/r4_cell_1/cell_report.json"}, cfg)
    assert allow is False and matched == "cell_run_dir"


def test_guard_fails_CLOSED_on_a_broken_config(tmp_path, monkeypatch, capsys):
    """A guard that fails open stops guarding silently, which is the failure
    mode that created this file. Breaking it must break the cell LOUDLY."""
    gdir = tmp_path / "guard"
    gdir.mkdir()
    (gdir / guard.CONFIG_NAME).write_text("{not json", encoding="utf-8")
    monkeypatch.setattr(guard, "load_config",
                        lambda here=None: guard.load_config(gdir))
    monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps(
        {"tool_name": "Read", "tool_input": {"file_path": "README.md"}})))
    assert guard.main() == 0
    out = json.loads(capsys.readouterr().out)
    assert out["hookSpecificOutput"]["permissionDecision"] == "deny"


def test_guard_config_is_derived_from_the_staging_exclusion_list(tmp_path):
    """One declaration, three enforcements: staging removes them, the guard
    refuses them, the audit catches what got through anyway."""
    rec = cell.install_guard(tmp_path / "cell", tmp_path / "ws",
                             scratch_root=tmp_path)
    cfg = json.loads(Path(rec["config"]).read_text(encoding="utf-8"))
    assert cfg["exclude_prefixes"] == list(staging.OMNISIM_EXCLUDE_PREFIXES)
    assert cfg["exclude_files"] == list(staging.OMNISIM_EXCLUDE_FILES)
    assert cfg["junction_dirs"] == list(staging.OMNISIM_JUNCTION_DIRS)
    # the settings file must stay the MINIMAL shape that was verified to work:
    # `claude -p` ignores a settings file that fails validation SILENTLY, so an
    # unvalidated extra key here would turn the guard off without a word.
    st = json.loads(Path(rec["settings"]).read_text(encoding="utf-8"))
    assert list(st) == ["hooks"] and list(st["hooks"]) == ["PreToolUse"]


def test_a_guard_that_never_fired_blocks_the_cell(tmp_path):
    """The instrument check. An empty guard log after a session that used
    tools means the hook never ran -- which looks exactly like a clean cell."""
    rec = cell.install_guard(tmp_path / "cell", tmp_path / "ws",
                             scratch_root=tmp_path)
    cell.read_guard_log(rec)
    assert rec["enforced"] is False
    blocked, why = cell.guard_blocks(rec, tool_calls=42)
    assert blocked and "42" in why and "NOT contained" in why
    # a session that made no tool calls had nothing to guard
    assert cell.guard_blocks(rec, tool_calls=0) == (False, None)
    # ...and a guard that DID see calls is fine
    Path(rec["log"]).write_text(
        json.dumps({"tool": "Read", "allow": True}) + "\n"
        + json.dumps({"tool": "Bash", "allow": False,
                      "matched": "repo_checkout"}) + "\n", encoding="utf-8")
    cell.read_guard_log(rec)
    assert rec["enforced"] is True and rec["denied"] == 1
    assert cell.guard_blocks(rec, tool_calls=42) == (False, None)


def test_a_cached_template_is_checked_against_what_it_claims(tmp_path):
    """MEASURED 2026-08-12: the cached template's manifest recorded 4799 files
    and the directory held 255 -- no AGENTS.md, no README.md, and a
    src/omnisim/nodes/ still on the pre-rename Wb* names. Every cell staged
    from it handed its agent a nearly empty tree missing the product's own
    entry point, and "the directory exists and has a manifest" said it was
    fine.
    """
    tpl = tmp_path / "templates" / "omnisim"
    mpath = tmp_path / "templates" / "omnisim.manifest.json"
    tpl.mkdir(parents=True)
    copied = ["AGENTS.md", "docs/x.md", "src/a.cpp"]
    for rel in copied:
        p = tpl / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("x", encoding="utf-8")
    good = {"utc": "2026-08-01T00:00:00Z",
            "included_file_count": len(copied),
            "filelist_sha256": staging.filelist_sha256(copied),
            "exclude_prefixes": list(staging.OMNISIM_EXCLUDE_PREFIXES),
            "exclude_files": list(staging.OMNISIM_EXCLUDE_FILES)}
    mpath.write_text(json.dumps(good), encoding="utf-8")
    assert staging.omnisim_template_reusable(tpl, mpath, copied) == (True, None)

    # 1. the tree was partly swept after it was built -- THE measured failure
    (tpl / "AGENTS.md").unlink()
    ok, why = staging.omnisim_template_reusable(tpl, mpath, copied)
    assert ok is False and "partly deleted" in why and "2026-08-01" in why
    (tpl / "AGENTS.md").write_text("x", encoding="utf-8")

    # 2. the repo moved on
    ok, why = staging.omnisim_template_reusable(
        tpl, mpath, copied + ["docs/new.md"])
    assert ok is False and "tracked-file selection has changed" in why

    # 3. a needle was added to the answer key -- every template built before
    #    it is now a leak, not merely stale
    m2 = dict(good, exclude_prefixes=["social/"])
    mpath.write_text(json.dumps(m2), encoding="utf-8")
    ok, why = staging.omnisim_template_reusable(tpl, mpath, copied)
    assert ok is False and "exclusion list has changed" in why
