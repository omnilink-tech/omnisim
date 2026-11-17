import json

import pytest

from agentbench.codex_lane import run_codex_task as lane


def test_command_uses_the_documented_noninteractive_safety_shape(tmp_path):
    cmd = lane.codex_command(
        "codex", prompt="build it", workspace=tmp_path,
        model="gpt-test", answer_path=tmp_path / "answer.txt")
    assert cmd[:2] == ["codex", "exec"]
    assert "--ephemeral" in cmd
    assert "--json" in cmd
    assert cmd[cmd.index("--sandbox") + 1] == "workspace-write"
    assert cmd[cmd.index("--ask-for-approval") + 1] == "never"
    assert "--ignore-user-config" in cmd
    assert "--ignore-rules" in cmd
    assert "--skip-git-repo-check" in cmd
    assert cmd[cmd.index("--model") + 1] == "gpt-test"
    assert cmd[-1] == "build it"


def test_command_refuses_an_unpinned_model(tmp_path):
    with pytest.raises(ValueError, match="explicit model"):
        lane.codex_command(
            "codex", prompt="x", workspace=tmp_path, model=None,
            answer_path=tmp_path / "answer.txt")


def test_jsonl_metrics_are_measured_not_invented():
    text = "\n".join(json.dumps(v) for v in [
        {"type": "thread.started", "thread_id": "thread-1"},
        {"type": "item.completed", "item": {
            "type": "command_execution", "command": "dir"}},
        {"type": "item.completed", "item": {
            "type": "agent_message", "text": "done"}},
        {"type": "turn.completed", "usage": {
            "input_tokens": 100, "cached_input_tokens": 20,
            "output_tokens": 30, "reasoning_output_tokens": 4}},
    ])
    got = lane.parse_events(text)
    assert got["thread_id"] == "thread-1"
    assert got["tool_calls"] == 1
    assert got["final_message"] == "done"
    assert got["tokens_in"] == 100
    assert got["tokens_cache_read"] == 20
    assert got["tokens_out"] == 30
    assert got["completed"] is True


def test_codex_identity_records_resolved_executable_and_version(
        tmp_path, monkeypatch):
    exe = tmp_path / "codex.exe"
    exe.write_bytes(b"")

    class Result:
        returncode = 0
        stdout = "codex-cli 1.2.3\n"
        stderr = ""

    monkeypatch.setattr(lane.subprocess, "run", lambda *a, **k: Result())
    got = lane.codex_identity(exe)
    assert got == {"path": str(exe.resolve()), "version": "codex-cli 1.2.3"}


def test_containment_is_unknown_without_events(tmp_path):
    got = lane.containment_audit("", workspace=tmp_path)
    assert got["clean"] is None


def test_containment_blocks_a_direct_read_of_the_real_checkout(tmp_path):
    repo = tmp_path / "real_repo"
    ws = tmp_path / "workspace"
    repo.mkdir()
    ws.mkdir()
    event = json.dumps({
        "type": "item.completed",
        "item": {"type": "command_execution",
                 "command": "type %s\\tests\\benchmarks\\agentbench\\SPEC.md"
                            % repo},
    })
    got = lane.containment_audit(event, workspace=ws, repo=repo)
    assert got["clean"] is False
    assert got["hits"]


def test_omnisim_template_copies_projects_instead_of_junctioning_them(
        tmp_path, monkeypatch):
    seen = {}

    def fake(root, **kwargs):
        seen["root"] = root
        seen.update(kwargs)
        return tmp_path / "template"

    monkeypatch.setattr(lane.staging, "build_omnisim_template", fake)
    lane._build_template("omnisim", tmp_path)
    assert "projects" not in seen["junction_dirs"]
    assert seen["junction_dirs"] == lane.OMNISIM_RUNTIME_JUNCTIONS
    assert seen["root"] == tmp_path / "codex_safe"
