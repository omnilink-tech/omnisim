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

"""The four shell-condition handlers, against a real filesystem and a real
shell. These execute subprocesses, so they are the slowest tests in the runner
suite -- but a mocked shell would not have caught the environment-allowlist
bugs these do.
"""

from __future__ import annotations

import pytest

POSIX = pytest.mark.skipif(False, reason="")


def call(tool_set, name, args, timeout=30.0):
    return tool_set.get(name).handler(args, timeout)


def test_write_then_read_round_trip(shell_tools):
    out = call(shell_tools, "write_file",
               {"path": "sub/dir/a.txt", "content": "line1\nline2\n"})
    assert not out.is_error and out.data["bytes"] == 12
    back = call(shell_tools, "read_file", {"path": "sub/dir/a.txt"})
    assert "line1" in back.text and "line2" in back.text
    assert back.data["total_lines"] == 2


def test_read_file_paging(shell_tools):
    call(shell_tools, "write_file",
         {"path": "big.txt", "content": "\n".join(str(i) for i in range(100))})
    out = call(shell_tools, "read_file",
               {"path": "big.txt", "start_line": 50, "max_lines": 3})
    assert out.data["returned"] == 3
    assert "49" in out.text and "51" in out.text
    assert "10\n" not in out.text


def test_write_refuses_outside_the_scratch_dir(shell_tools, tmp_path):
    out = call(shell_tools, "write_file",
               {"path": str(tmp_path / "nope.txt"), "content": "x"})
    assert out.is_error and "confined" in out.text
    assert not (tmp_path / "nope.txt").exists()


def test_write_refuses_a_traversal_escape(shell_tools):
    out = call(shell_tools, "write_file",
               {"path": "../../../escaped.txt", "content": "x"})
    assert out.is_error


def test_read_refuses_the_benchmark_package(shell_tools):
    from agentbench.common.paths import AGENTBENCH
    out = call(shell_tools, "read_file",
               {"path": str(AGENTBENCH / "graders" / "a1.py")})
    assert out.is_error and "benchmark" in out.text


def test_read_allows_the_repository(shell_tools):
    from agentbench.common.paths import REPO
    out = call(shell_tools, "read_file", {"path": str(REPO / "AGENTS.md")})
    assert not out.is_error


def test_list_dir_reports_entries(shell_tools):
    call(shell_tools, "write_file", {"path": "a.txt", "content": "x"})
    call(shell_tools, "write_file", {"path": "d/b.txt", "content": "x"})
    flat = call(shell_tools, "list_dir", {"path": "."})
    assert "a.txt" in flat.text and "d/" in flat.text
    deep = call(shell_tools, "list_dir", {"path": ".", "recursive": True})
    names = {e["name"].replace("\\", "/") for e in deep.data["entries"]}
    assert "d/b.txt" in names


def test_list_dir_caps_entries(shell_tools):
    for i in range(20):
        call(shell_tools, "write_file", {"path": "f%d.txt" % i,
                                         "content": "x"})
    out = call(shell_tools, "list_dir", {"path": ".", "max_entries": 5})
    assert len(out.data["entries"]) == 5
    assert out.data["total_seen"] == 20


def test_bad_arguments_are_errors_not_crashes(shell_tools):
    assert call(shell_tools, "run_shell", {}).is_error
    assert call(shell_tools, "write_file", {"path": "x"}).is_error
    assert call(shell_tools, "read_file", {"path": "missing.txt"}).is_error
    assert call(shell_tools, "list_dir", {"path": "missing_dir"}).is_error


# -- the shell itself ------------------------------------------------------

def test_run_shell_captures_exit_code_and_output(shell_tools):
    out = call(shell_tools, "run_shell", {"command": "echo hello-agentbench"})
    assert out.data["exit_code"] == 0
    assert "hello-agentbench" in out.data["stdout"]
    assert not out.is_error


def test_run_shell_reports_a_nonzero_exit_as_an_error(shell_tools):
    out = call(shell_tools, "run_shell", {"command": "exit 3"})
    assert out.data["exit_code"] == 3
    assert out.is_error, "a failed command must be visible to the model"


def test_run_shell_runs_in_the_scratch_dir(shell_tools, sandbox):
    call(shell_tools, "write_file", {"path": "marker.txt", "content": "x"})
    out = call(shell_tools, "run_shell", {"command": "ls marker.txt"})
    assert "marker.txt" in out.data["stdout"]
    assert str(sandbox.scratch_dir) == out.data["cwd"]


def test_run_shell_honours_the_timeout(shell_tools):
    out = call(shell_tools, "run_shell",
               {"command": "sleep 5", "timeout_s": 1.0}, timeout=2.0)
    assert out.data["timed_out"] is True
    assert out.is_error
    assert out.data["duration_s"] < 4.0


def test_run_shell_timeout_cannot_exceed_the_budget_slice(shell_tools):
    """The per-call ceiling wins even if the model asks for longer."""
    out = call(shell_tools, "run_shell",
               {"command": "sleep 5", "timeout_s": 600.0}, timeout=1.0)
    assert out.data["timed_out"] is True


def test_the_agent_environment_has_the_sim_but_not_the_secrets(shell_tools,
                                                               monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-should-not-leak")
    monkeypatch.setenv("AGENTBENCH_EXTERNAL_ARTIFACT", "/tmp/answer.wbt")
    out = call(shell_tools, "run_shell", {
        "command": 'echo "HOME=[$OMNISIM_HOME]"; echo "KEY=[$ANTHROPIC_API_KEY]"'
                   '; echo "EXT=[$AGENTBENCH_EXTERNAL_ARTIFACT]"'
                   '; echo "HASHSEED=[$PYTHONHASHSEED]"'})
    stdout = out.data["stdout"]
    assert "HOME=[" in stdout and "omnisim" in stdout.lower()
    assert "KEY=[]" in stdout, "the model API key must never reach the agent"
    assert "EXT=[]" in stdout, "the external artifact path must not leak"
    assert "HASHSEED=[]" in stdout, "PYTHONHASHSEED must stay unset"


def test_long_output_is_clipped_for_the_model_but_kept_in_data(shell_tools):
    from agentbench.runner.tools.shell import MODEL_TEXT_LIMIT
    n = MODEL_TEXT_LIMIT + 5000
    out = call(shell_tools, "run_shell",
               {"command": "python -c \"print('z' * %d)\"" % n})
    assert len(out.text) <= MODEL_TEXT_LIMIT + 200
    assert "characters omitted" in out.text
    assert len(out.data["stdout"]) >= n
