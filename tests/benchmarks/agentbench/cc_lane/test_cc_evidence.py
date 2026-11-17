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

"""Regressions for the four defects that made ``r1_3arm_20260810`` produce
zero valid cells AND destroy the evidence.

Each test names the observation it is pinned to. In order:

1. ``shutil.which("claude")`` returns npm's ``claude.CMD`` on Windows;
   ``CreateProcess`` runs a ``.cmd`` through cmd.exe, which ends the command
   line at the first newline of any argument. Every prompt in this suite is
   multi-line, so the child got line 1 and lost ``--output-format json``,
   ``--model`` and ``--dangerously-skip-permissions`` -- silently, rc=0.
2. The verdict was taken from the session's STDOUT, so a cell whose agent had
   built something was BLOCKED for emitting prose, and its workspace was then
   deleted by the ordinary teardown.
3. A session killed at 8.7% of its task budget published
   ``FAIL / no_artifact / progress 0``, indistinguishable in the grid from an
   agent that worked for half an hour and built nothing.
4. A running cell was a black box: nothing could be said about it until it
   exited.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[2]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import evidence                     # noqa: E402
from agentbench.cc_lane import run_cc_cell as cell          # noqa: E402

WINDOWS = os.name == "nt"


# --- 1. the launcher --------------------------------------------------------


def _npm_layout(tmp_path, *, with_exe=True):
    """npm's Windows install shape: a .CMD shim beside node_modules."""
    binroot = tmp_path / "npm"
    (binroot).mkdir(parents=True, exist_ok=True)
    shim = binroot / "claude.CMD"
    shim.write_text(
        '@ECHO off\r\n"%dp0%\\node_modules\\@anthropic-ai\\claude-code\\bin'
        '\\claude.exe"   %*\r\n', encoding="utf-8")
    exe = binroot / "node_modules" / "@anthropic-ai" / "claude-code" / "bin" \
        / "claude.exe"
    if with_exe:
        exe.parent.mkdir(parents=True, exist_ok=True)
        exe.write_bytes(b"MZ")
    return shim, exe


@pytest.mark.skipif(not WINDOWS, reason=".cmd shims are a Windows thing")
def test_the_cmd_shim_is_resolved_to_the_real_exe(tmp_path):
    """THE root cause. A .cmd launcher must never be what we exec."""
    shim, exe = _npm_layout(tmp_path)
    resolved, how = cell._resolve_cli_launcher(shim)
    assert Path(resolved) == exe, (
        "the npm .cmd shim was not resolved to the real executable; running "
        "the shim sends the command line through cmd.exe, which truncates it "
        "at the first newline of the prompt and silently drops every flag "
        "after it")
    assert "shim" in how


def test_a_real_executable_is_left_alone(tmp_path):
    exe = tmp_path / ("claude.exe" if WINDOWS else "claude")
    exe.write_bytes(b"MZ")
    resolved, _how = cell._resolve_cli_launcher(exe)
    assert Path(resolved) == exe


@pytest.mark.skipif(not WINDOWS, reason=".cmd shims are a Windows thing")
def test_an_unresolvable_shim_is_reported_as_unsafe(tmp_path):
    shim, _exe = _npm_layout(tmp_path, with_exe=False)
    resolved, how = cell._resolve_cli_launcher(shim)
    assert Path(resolved) == Path(shim)
    assert "UNRESOLVED" in how, \
        "an unresolvable shim must SAY it will truncate, not resolve quietly"


@pytest.mark.skipif(not WINDOWS, reason=".cmd shims are a Windows thing")
def test_running_a_multiline_prompt_through_a_cmd_shim_is_REFUSED(
        tmp_path, monkeypatch):
    """The fence. Loud refusal beats a silent rc=0 with an eaten prompt.

    This is the test that would have gone red on 2026-08-11. It must stay
    red-able: if the fence is removed, this fails.
    """
    shim, _exe = _npm_layout(tmp_path, with_exe=False)
    monkeypatch.setattr(cell, "_claude_exe", lambda: str(shim))
    with pytest.raises(RuntimeError) as exc:
        cell._run_claude(["-p", "line one\nline two", "--output-format",
                          "json"],
                         cwd=tmp_path, env=dict(os.environ), timeout_s=10)
    msg = str(exc.value)
    assert "truncat" in msg.lower()
    assert "r1_3arm_20260810" in msg, \
        "the refusal must name the incident it is fencing"


def test_the_preflight_probe_is_multiline():
    """A preflight that cannot fail the way the real call fails is decoration.

    ``"Say OK"`` is the one prompt shape cmd.exe truncation cannot corrupt,
    which is exactly why the instrument's self-check stayed green through a
    campaign in which every real cell was corrupted.
    """
    assert "\n" in cell.PREFLIGHT_PROMPT, (
        "the preflight prompt must contain a newline so that a launcher which "
        "truncates at one fails HERE, before any tokens are spent")


def test_the_session_asks_for_a_streaming_format():
    assert "stream-json" in cell.SESSION_OUTPUT_ARGS
    assert "--verbose" in cell.SESSION_OUTPUT_ARGS, \
        "the CLI refuses stream-json under --print without --verbose"


# --- 2. the stream is metadata, and it survives a kill ----------------------


def _write_stream(path, *, with_result=True, turns=2, tools=3):
    lines = [{"type": "system", "subtype": "init", "session_id": "sess-abc",
              "model": "claude-opus-5", "permissionMode": "bypassPermissions",
              "cwd": "/ws"}]
    for i in range(turns):
        content = [{"type": "text", "text": "thinking %d" % i}]
        for j in range(tools if i == 0 else 0):
            content.append({"type": "tool_use", "name": "Write",
                            "input": {"file_path": "worlds/arena%d.wbt" % j}})
        lines.append({"type": "assistant", "session_id": "sess-abc",
                      "message": {"role": "assistant",
                                  "model": "claude-opus-5",
                                  "content": content}})
    if with_result:
        lines.append({"type": "result", "subtype": "success",
                      "is_error": False, "num_turns": turns,
                      "duration_ms": 1234, "session_id": "sess-abc",
                      "total_cost_usd": 1.5, "result": "done",
                      "usage": {"input_tokens": 1, "output_tokens": 2},
                      "modelUsage": {"claude-opus-5": {"outputTokens": 2}}})
    Path(path).write_text(
        "\n".join(json.dumps(x) for x in lines) + "\n", encoding="utf-8")


def test_a_killed_session_still_yields_its_metadata(tmp_path):
    """No result event -- the exact shape of a session we killed at its budget.

    The old single-blob reader produced ``session_id: null`` and "transcript
    not found for session_id=None" while the transcript sat on disk the whole
    time, findable only BY that id.
    """
    p = tmp_path / "cc_stream.jsonl"
    _write_stream(p, with_result=False, turns=2, tools=3)
    s = evidence.read_stream(p)
    assert s["result"] is None
    assert s["session_id"] == "sess-abc", \
        "the session id arrives in the first event and must survive a kill"
    assert s["assistant_turns"] == 2
    assert s["tool_calls"] == 3
    assert s["model"] == "claude-opus-5"
    assert [c["target"] for c in s["tool_call_log"]][0] == "worlds/arena0.wbt"


def test_a_partial_trailing_line_is_counted_not_fatal(tmp_path):
    """The file is read while the session is still appending to it."""
    p = tmp_path / "cc_stream.jsonl"
    _write_stream(p, with_result=False, turns=1, tools=1)
    with open(p, "a", encoding="utf-8") as fh:
        fh.write('{"type": "assistant", "message": {"role": "assis')
    s = evidence.read_stream(p)
    assert s["assistant_turns"] == 1
    assert s["unparsed_lines"] == 1


# --- 3. liveness ------------------------------------------------------------


def _liveness(**kw):
    base = dict(meta={"wall_s": 900.0, "timed_out": False},
                stream={"events": 40, "assistant_turns": 12,
                        "tool_calls": 30, "session_id": "s1",
                        "model": "claude-opus-5"},
                transcript="C:/x/s1.jsonl", session_budget_s=1800.0,
                task_budget_s=1800.0, pinned_model="claude-opus-5",
                result_json={"session_id": "s1"})
    base.update(kw)
    return cell.assess_liveness(**base)


def test_a_healthy_session_is_live():
    assert _liveness()["ok"] is True


def test_the_starved_webots_cell_would_have_been_refused():
    """THE case. 156.7 s granted against a 1800 s task budget."""
    lv = _liveness(session_budget_s=156.7, task_budget_s=1800.0,
                   meta={"wall_s": 156.8, "timed_out": True},
                   stream={}, transcript=None, result_json=None)
    assert lv["ok"] is False
    assert "budget_honoured" in lv["vetoing"]
    assert "STARVED" in lv["verdict"]


def test_a_session_killed_at_its_FULL_budget_is_live_and_scored():
    """SPEC 2.4: budget exhaustion is the AGENT's result, not a broken
    instrument. This must NOT be confused with the starved case above -- the
    difference is which budget it was killed at."""
    lv = _liveness(meta={"wall_s": 1800.0, "timed_out": True},
                   stream={"events": 0}, transcript=None, result_json=None,
                   session_budget_s=1800.0)
    assert lv["ok"] is True, \
        "a session that held the machine for its whole budget ran; scoring it "\
        "is the point of SPEC 2.4"


def test_a_launch_that_never_started_is_refused():
    lv = _liveness(meta={"wall_s": 0.3, "timed_out": False}, stream={},
                   transcript=None, result_json=None)
    assert lv["ok"] is False
    assert "session_started" in lv["vetoing"]
    assert "NEVER RAN" in lv["verdict"]


def test_a_wrong_model_is_recorded_but_does_not_veto():
    """The 82/82-turns-on-the-default-model fact. It contaminates the
    MEASUREMENT (model_attribution fences that) but it is not evidence the
    session did not run, so it must not silently drop the cell."""
    lv = _liveness(stream={"events": 9, "assistant_turns": 3,
                           "tool_calls": 5, "session_id": "s1",
                           "model": "claude-opus-4-8"})
    assert lv["ok"] is True
    assert "model_pin_applied" in lv["failed"]
    assert "model_pin_applied" not in lv["vetoing"]


def test_a_missing_transcript_alone_does_not_veto():
    lv = _liveness(transcript=None)
    assert lv["ok"] is True
    assert "transcript_present" in lv["failed"]


# --- 4. workspace preservation ----------------------------------------------


def test_the_workspace_is_copied_and_links_are_never_followed(tmp_path):
    ws = tmp_path / "ws"
    (ws / "worlds").mkdir(parents=True)
    (ws / "worlds" / "arena.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
    (ws / "__pycache__").mkdir()
    (ws / "__pycache__" / "x.pyc").write_bytes(b"junk")
    (ws / "notes.pyc").write_bytes(b"junk")
    huge = tmp_path / "outside"
    huge.mkdir()
    (huge / "gigabytes.bin").write_bytes(b"x" * 10)
    linked = False
    try:                                     # junction on Windows, symlink else
        if WINDOWS:
            subprocess.run(["cmd", "/c", "mklink", "/J", str(ws / "projects"),
                            str(huge)], capture_output=True, check=True)
        else:
            os.symlink(huge, ws / "projects")
        linked = True
    except (OSError, subprocess.CalledProcessError):
        pass

    dest = tmp_path / "cell" / "workspace"
    man = evidence.preserve_workspace(ws, dest)

    assert (dest / "worlds" / "arena.wbt").read_text(encoding="utf-8") == \
        "WorldInfo {}\n"
    assert not (dest / "__pycache__").exists()
    assert not (dest / "notes.pyc").exists()
    assert man["files"] == 1 and man["bytes"] > 0
    # Exclusions are ENUMERATED. "never silently truncate" is the rule.
    assert any(x["path"] == "__pycache__" for x in man["excluded_dirs"])
    assert any(x["path"] == "notes.pyc" for x in man["excluded_files"])
    if linked:
        assert not (dest / "projects" / "gigabytes.bin").exists(), \
            "following the junction copies somebody else's tree, not the "\
            "agent's work"
        assert any(x["path"] == "projects" for x in man["excluded_links"])
    assert (dest / "workspace_manifest.json").is_file()


def test_preserving_is_idempotent_and_incremental(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    (ws / "a.txt").write_text("one", encoding="utf-8")
    dest = tmp_path / "cell" / "workspace"
    evidence.preserve_workspace(ws, dest)
    (ws / "b.txt").write_text("two", encoding="utf-8")
    man = evidence.preserve_workspace(ws, dest)
    assert man["files"] == 2
    assert (dest / "b.txt").read_text(encoding="utf-8") == "two"


def test_preserving_a_vanished_workspace_records_it_instead_of_raising(
        tmp_path):
    man = evidence.preserve_workspace(tmp_path / "gone",
                                      tmp_path / "cell" / "workspace")
    assert man["source_present"] is False
    assert "note" in man


def test_the_total_cap_is_recorded_not_silent(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    for i in range(4):
        (ws / ("f%d.bin" % i)).write_bytes(b"x" * 100)
    man = evidence.preserve_workspace(ws, tmp_path / "d", max_total_bytes=150)
    assert man["truncated"] is True
    assert man["truncated_at"], \
        "a truncated copy must name the file it stopped at"


# --- 5. the status view -----------------------------------------------------


def _fake_cell_dir(tmp_path, *, finished=False):
    cd = tmp_path / "cell"
    cd.mkdir(parents=True, exist_ok=True)
    ws = tmp_path / "live_ws"
    (ws / "worlds").mkdir(parents=True, exist_ok=True)
    (ws / "worlds" / "arena.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
    rep = {
        "sim": "webots", "task": "R1_lidar_nav",
        "utc_start": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                   time.gmtime(time.time() - 120)),
        "workspace": str(ws), "pinned_model": "claude-opus-5",
        "budget": {"task_timeout_s": 1800.0, "session_budgets_s": [1800.0],
                   "session_budget_curtailed": False},
    }
    if finished:
        rep["utc_end"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        rep["row"] = {"outcome": "FAIL", "failed_assertion": "R1.3"}
    (cd / "cell_report.json").write_text(json.dumps(rep), encoding="utf-8")
    _write_stream(cd / "cc_stream.jsonl", with_result=finished)
    return cd, ws


def test_status_reports_a_running_cell_from_disk_alone(tmp_path):
    cd, ws = _fake_cell_dir(tmp_path)
    st = evidence.cell_status(cd)
    assert st["state"] == "running"
    assert st["stream"]["session_id"] == "sess-abc"
    assert st["stream"]["tool_calls"] == 3
    assert st["workspace_kind"] == "live"
    assert any(f["path"] == "worlds/arena.wbt" for f in st["recent_files"]), \
        "watching files appear in the workspace IS the answer to 'what is it "\
        "doing right now'"
    text = evidence.render_status(cd)
    assert "R1_lidar_nav" in text and "worlds/arena.wbt" in text
    assert "sess-abc" in text


def test_status_finds_the_live_workspace_before_any_report_exists(tmp_path):
    """The first minutes of a run are exactly when someone asks what it is
    doing -- and the cell report may not be on disk yet. The stream's
    system/init event names the session's cwd, which IS the workspace."""
    cd = tmp_path / "cell"
    cd.mkdir()
    ws = tmp_path / "live_ws"
    (ws / "worlds").mkdir(parents=True)
    (ws / "worlds" / "arena.wbt").write_text("WorldInfo {}\n", encoding="utf-8")
    lines = [{"type": "system", "subtype": "init", "session_id": "s9",
              "model": "claude-opus-5", "cwd": str(ws)},
             {"type": "assistant", "session_id": "s9",
              "message": {"role": "assistant", "content": [
                  {"type": "tool_use", "name": "Write",
                   "input": {"file_path": "worlds/arena.wbt"}}]}}]
    (cd / "cc_stream.jsonl").write_text(
        "\n".join(json.dumps(x) for x in lines) + "\n", encoding="utf-8")
    st = evidence.cell_status(cd)
    assert st["state"] == "running"
    assert st["workspace_kind"] == "live"
    assert Path(st["workspace_read"]) == ws
    assert st["elapsed_s"] is not None, \
        "a live cell with no report must still report an elapsed time"
    assert any(f["path"] == "worlds/arena.wbt" for f in st["recent_files"])


def test_status_falls_back_to_the_preserved_copy_when_the_workspace_is_gone(
        tmp_path):
    cd, ws = _fake_cell_dir(tmp_path, finished=True)
    evidence.preserve_workspace(ws, cd / "workspace")
    import shutil
    shutil.rmtree(ws)
    st = evidence.cell_status(cd)
    assert st["state"] == "finished"
    assert st["workspace_kind"] == "preserved copy"
    assert any(f["path"] == "worlds/arena.wbt" for f in st["recent_files"])


def test_a_finished_cell_reads_the_preserved_copy_not_a_gutted_temp_dir(
        tmp_path):
    """Teardown can hit a locked file, delete most of the tree and leave the
    directory standing. Measured on the verification cell: a status read of
    that path reported "recently written files: (none)" for a cell whose 27
    preserved files were sitting in the cell dir."""
    cd, ws = _fake_cell_dir(tmp_path, finished=True)
    evidence.preserve_workspace(ws, cd / "workspace")
    (ws / "worlds" / "arena.wbt").unlink()          # teardown got this far
    st = evidence.cell_status(cd)
    assert st["workspace_kind"] == "preserved copy"
    assert any(f["path"] == "worlds/arena.wbt" for f in st["recent_files"]), \
        "a finished cell must be described from the record, not from whatever "\
        "teardown happened to leave behind"


def test_status_writes_nothing_into_the_cell_dir(tmp_path):
    """Read-only means read-only: a status call against a LIVE cell must not
    perturb the thing it is describing."""
    cd, _ws = _fake_cell_dir(tmp_path)
    before = {p.name: p.stat().st_mtime_ns for p in cd.rglob("*")}
    evidence.render_status(cd)
    after = {p.name: p.stat().st_mtime_ns for p in cd.rglob("*")}
    assert before == after


# --- 6. end to end: the workspace is the artifact of record -----------------

from agentbench.cc_lane.test_cc_lane import _minimal_cell_env  # noqa: E402


def _prose_session_meta(**kw):
    """What the mujoco/omnisim cells looked like on 2026-08-11: rc 0, a long
    session, real work done -- and no parseable result on stdout."""
    meta = {"permission_mode": "dangerously-skip-permissions",
            "cli_command": "claude -p <prompt> ...", "rc": 0,
            "timed_out": False, "wall_s": 908.0,
            "launch_error": "no result event in the session stream (rc=0)",
            "stream": {"events": 153, "assistant_turns": 82, "tool_calls": 61,
                       "session_id": "4eec650e", "model": "claude-opus-5",
                       "reached_result_event": False}}
    meta.update(kw)
    return meta


def test_a_session_that_emitted_prose_is_GRADED_not_blocked(tmp_path,
                                                            monkeypatch):
    """THE fix. The verdict comes from the deliverable, not from stdout.

    On 2026-08-11 this exact shape was BLOCKED with "claude session produced
    no result JSON: no JSON on stdout (rc=0)" -- and the workspace holding
    whatever the agent built was torn down minutes later.
    """
    run_dir = tmp_path / "out"
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_prose_session_meta())
    row = cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                        out_dir=run_dir, use_locks=False, repeat=0)
    assert row is not None, "a cell with a gradeable deliverable must land a row"
    assert (run_dir / "rows.jsonl").is_file()
    rep = json.loads((run_dir / "cell_report.json").read_text(encoding="utf-8"))
    assert rep.get("blocker") is None
    assert rep["session_incomplete"] is True, \
        "the metadata gap must be RECORDED, not hidden by a successful grade"
    assert rep["liveness"]["ok"] is True
    # ...and the metadata that survived is used rather than nulled.
    assert row["agent_artifacts"]["session_stream"]["assistant_turns"] == 82
    assert row["metrics"]["tool_calls"] == 61
    assert row["agent_artifacts"]["liveness"]["ok"] is True


def test_the_workspace_is_preserved_even_when_the_cell_blocks(tmp_path,
                                                              monkeypatch):
    """Preservation is unconditional. A harness failure must never be able to
    destroy the only record of what the agent did."""
    run_dir = tmp_path / "out"
    _minimal_cell_env(
        monkeypatch, tmp_path, staged=["six_huskies.wbt"],
        cc_meta={"permission_mode": None, "cli_command": "claude -p",
                 "rc": 1, "timed_out": False, "wall_s": 0.3,
                 "launch_error": "claude: command not found"})
    with pytest.raises(SystemExit):
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=run_dir, use_locks=False, repeat=0)
    preserved = run_dir / "workspace"
    assert preserved.is_dir(), \
        "a blocked cell must still leave its workspace next to the report"
    assert (preserved / "six_huskies.wbt").is_file()
    assert (preserved / "workspace_manifest.json").is_file()
    rep = json.loads((run_dir / "cell_report.json").read_text(encoding="utf-8"))
    assert rep["liveness"]["ok"] is False
    assert "session_started" in rep["liveness"]["vetoing"]


def test_status_survives_text_the_console_cannot_encode(tmp_path):
    """MEASURED mid-run: the agent's message contained a Unicode minus
    (U+2212), the Windows console is cp1252, and `print()` raised. A status
    command that dies on the text it is quoting fails precisely when the
    agent is doing something worth quoting."""
    import io
    cd = tmp_path / "cell"
    cd.mkdir()
    lines = [{"type": "system", "subtype": "init", "session_id": "s1",
              "cwd": str(tmp_path)},
             {"type": "assistant", "session_id": "s1",
              "message": {"role": "assistant", "content": [
                  {"type": "text", "text": "clearance −0.35 m — bad"}]}}]
    (cd / "cc_stream.jsonl").write_text(
        "\n".join(json.dumps(x) for x in lines) + "\n", encoding="utf-8")

    class Cp1252Stream(io.StringIO):
        encoding = "cp1252"

        def write(self, s):
            s.encode("cp1252")            # raises exactly as the console does
            return super().write(s)

    out = Cp1252Stream()
    evidence.print_status(cd, out)        # must not raise
    assert "clearance" in out.getvalue()


def test_status_cli_accepts_a_cell_dir(tmp_path, capsys):
    cd, _ws = _fake_cell_dir(tmp_path)
    rc = cell.main(["--status", str(cd)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "R1_lidar_nav" in out
    rc = cell.main(["--status", str(cd), "--json"])
    assert rc == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc["stream"]["session_id"] == "sess-abc"


def test_a_graded_cell_records_a_MEASURED_containment_verdict(tmp_path,
                                                              monkeypatch):
    """Every cell carries the verdict, not only the contaminated ones.

    An absent field reads as "not checked", and a checked-and-clean cell has
    to be distinguishable from one nobody looked at -- the same rule the
    liveness block follows. Measured 2026-08-11: the first R4/omnisim cell
    read the R4 grader and its report was indistinguishable from a clean
    one's, so the contamination took a hand audit of 1.4 MB of NDJSON to find.
    """
    run_dir = tmp_path / "out"
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_prose_session_meta())
    cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                  out_dir=run_dir, use_locks=False, repeat=0)
    rep = json.loads((run_dir / "cell_report.json").read_text(encoding="utf-8"))
    c = rep["containment"]
    assert c["clean"] is True and c["tool_calls_scanned"] == 2
    assert c["hits"] == [] and c["needles"] >= 5
    assert rep.get("blocker") is None
