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

"""Unit tests for the advisory rows `omnisim doctor` grew on 2026-09-02.

Every function under test is pure (paths, strings, dicts in; dicts out) --
no engine, no harness, no network. The one gating row among them is the
vendored-runtime staleness check, pinned in both directions because its
whole point is a trap that a green build cannot see: the engine imports the
BUNDLE, so an edit to src/omnisim/physics/omnisim_newton_runtime.py ships
only after `bundle_newton_runtime.py --mode vendor` (AGENTS.md: measured
2026-08-09, a bundle 23 minutes older than the source and a full validation
pass that proved nothing).
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from omnisim import doctor  # noqa: E402

ENGINE_EXE = doctor.ENGINE_PROCESS_NAMES[0]  # the Windows process name


# --- vendored runtime staleness -------------------------------------------

def _pair(tmp_path: Path, src_text: str, bundle_text: str, source_newer: bool):
    src = tmp_path / "src" / "omnisim_newton_runtime.py"
    bun = tmp_path / "bundle" / "omnisim_newton_runtime.py"
    src.parent.mkdir()
    bun.parent.mkdir()
    src.write_text(src_text)
    bun.write_text(bundle_text)
    now = time.time()
    older, newer = now - 600, now
    if source_newer:
        os.utime(src, (newer, newer))
        os.utime(bun, (older, older))
    else:
        os.utime(src, (older, older))
        os.utime(bun, (newer, newer))
    return src, bun


def test_runtime_bundle_identical_is_ok(tmp_path):
    src, bun = _pair(tmp_path, "x = 1\n", "x = 1\n", source_newer=True)
    info = doctor._runtime_bundle_status(src, bun)
    assert info["status"] == "ok"
    assert info["fix"] is None
    assert info["source_sha256"] == info["bundle_sha256"]


def test_runtime_bundle_stale_when_source_is_newer(tmp_path):
    src, bun = _pair(tmp_path, "x = 2\n", "x = 1\n", source_newer=True)
    info = doctor._runtime_bundle_status(src, bun)
    assert info["status"] == "stale"
    assert info["fix"] == doctor.RUNTIME_VENDOR_FIX
    assert "STALE" in info["detail"]
    # ...and it GATES: the engine would run the old physics code.
    build = {"verdict": "ok", "engine": "e", "controller_lib": "c",
             "engine_ipc_nonce": True, "controller_lib_ipc_nonce": True, "detail": ""}
    c = doctor._coherence(build, None, info)
    assert c["ok"] is False
    assert any(doctor.RUNTIME_VENDOR_FIX in f for f in c["fatal"])


def test_runtime_bundle_differs_but_newer_is_advisory(tmp_path):
    src, bun = _pair(tmp_path, "x = 1\n", "x = 2\n", source_newer=False)
    info = doctor._runtime_bundle_status(src, bun)
    assert info["status"] == "differs"
    assert info["fix"] == doctor.RUNTIME_VENDOR_FIX
    build = {"verdict": "ok", "engine": "e", "controller_lib": "c",
             "engine_ipc_nonce": True, "controller_lib_ipc_nonce": True, "detail": ""}
    c = doctor._coherence(build, None, info)
    assert c["ok"] is True
    assert any("differs" in a for a in c["advisory"])


def test_runtime_bundle_absent_without_a_bundle(tmp_path):
    src = tmp_path / "omnisim_newton_runtime.py"
    src.write_text("x = 1\n")
    assert doctor._runtime_bundle_status(src, None)["status"] == "absent"
    assert doctor._runtime_bundle_status(src, tmp_path / "nope.py")["status"] == "absent"
    assert doctor._runtime_bundle_status(tmp_path / "missing.py", None)["status"] == "absent"


def test_coherence_signature_stays_backwards_compatible():
    # Callers that predate the runtime_bundle argument keep working.
    build = {"verdict": "ok", "engine": "e", "controller_lib": "c",
             "engine_ipc_nonce": True, "controller_lib_ipc_nonce": True, "detail": ""}
    assert doctor._coherence(build)["ok"] is True
    assert doctor._coherence(build, None)["ok"] is True


# --- engine process listing (parsers only) --------------------------------

def test_world_from_cmdline_finds_the_world_argument():
    cmd = (f'"O:\\omnisim\\msys64\\mingw64\\bin\\{ENGINE_EXE}" '
           'O:\\omnisim\\projects\\samples\\demos\\worlds\\showcase\\.harness_x.omniworld '
           '--mode=fast --no-rendering --minimize')
    assert doctor._world_from_cmdline(cmd).endswith(".harness_x.omniworld")
    assert doctor._world_from_cmdline("/opt/omnisim/omnisim-bin /w/legacy.wbt --minimize") == "/w/legacy.wbt"
    assert doctor._world_from_cmdline(f"{ENGINE_EXE} --minimize") is None
    assert doctor._world_from_cmdline("") is None


def test_parse_tasklist_csv_extracts_engine_pids():
    text = (f'"{ENGINE_EXE}","23992","Console","9","65,788 K"\r\n'
            f'"{ENGINE_EXE}","24001","Console","9","70,000 K"\r\n'
            '"python.exe","1","Console","9","1 K"\r\n')
    assert doctor._parse_tasklist_csv(text) == [23992, 24001]
    assert doctor._parse_tasklist_csv("INFO: No tasks are running which match the specified criteria.\r\n") == []
    assert doctor._parse_tasklist_csv("") == []


def test_parse_cim_json_handles_one_or_many_rows():
    one = ('{"ProcessId":23992,"CommandLine":"O:\\\\omnisim\\\\bin\\\\%s '
           'O:\\\\w\\\\a.omniworld --minimize"}' % ENGINE_EXE)
    rows = doctor._parse_cim_json(one)
    assert rows[0]["pid"] == 23992 and rows[0]["world"].endswith("a.omniworld")
    many = "[" + one + "," + one.replace("23992", "5") + "]"
    assert [r["pid"] for r in doctor._parse_cim_json(many)] == [23992, 5]
    assert doctor._parse_cim_json("") == []
    assert doctor._parse_cim_json("not json") == []
    assert doctor._parse_cim_json('{"ProcessId": 7, "CommandLine": null}')[0]["world"] is None


def test_parse_ps_output_filters_to_the_engine():
    text = (f"  100 /opt/omnisim/bin/{doctor.ENGINE_PROCESS_NAMES[1]} /w/x.wbt --minimize\n"
            "  101 python3 scripts/harness/some_other_service.py --port 6789\n"
            f"  102 {doctor.ENGINE_PROCESS_NAMES[1]}\n")
    rows = doctor._parse_ps_output(text)
    assert [r["pid"] for r in rows] == [100, 102]
    assert rows[0]["world"] == "/w/x.wbt" and rows[1]["world"] is None


# --- harness probe (pure half) ---------------------------------------------

def test_harness_world_reads_the_sim_state_shape():
    assert doctor._harness_world({"world": "a.omniworld", "running": True}) == "a.omniworld"
    assert doctor._harness_world({"world": None}) is None
    assert doctor._harness_world({"last_load": {"path": "b.wbt"}}) == "b.wbt"
    assert doctor._harness_world(None) is None
    assert doctor._harness_world({}) is None


# --- git hooks ---------------------------------------------------------------

def test_hooks_path_ok_accepts_relative_and_absolute_forms(tmp_path):
    (tmp_path / ".githooks").mkdir()
    assert doctor._hooks_path_ok(".githooks", tmp_path)
    assert doctor._hooks_path_ok(str(tmp_path / ".githooks"), tmp_path)
    assert doctor._hooks_path_ok(f'"{tmp_path / ".githooks"}"', tmp_path)
    assert not doctor._hooks_path_ok(None, tmp_path)
    assert not doctor._hooks_path_ok("", tmp_path)
    assert not doctor._hooks_path_ok(".git/hooks", tmp_path)


# --- renderer / pillow / ffmpeg rows never raise ---------------------------

def test_wgpu_native_status_reports_absent_beside_a_binary(tmp_path):
    binary = tmp_path / ENGINE_EXE
    binary.write_bytes(b"MZ")
    info = doctor._wgpu_native_status(str(binary))
    if os.name == "nt":
        assert info["status"] == "absent" and info["fix"] == doctor.WGPU_SETUP_FIX
        (tmp_path / doctor.WGPU_NATIVE_DLL).write_bytes(b"MZ")
        assert doctor._wgpu_native_status(str(binary))["status"] == "present"
    else:
        assert info["status"] == "unknown"
    assert doctor._wgpu_native_status(None)["status"] == "unknown"


def test_pillow_and_ffmpeg_rows_have_the_documented_shape():
    p = doctor._pillow_status()
    assert set(p) == {"available", "interpreter", "detail", "fix"}
    assert (p["fix"] is None) == p["available"]
    f = doctor._ffmpeg_status()
    assert set(f) == {"path", "detail"}


# --- hosted CI row (pure classification + every `unknown` path) -------------
#
# The private repo's CI sat RED for a day (2026-09-02) with every local check
# green: the pre-push gate is a Windows smoke and cannot see the Linux
# renderer assertion or the provenance gate. The row is advisory and bounded
# (one gh call, 1.5 s); these pin its wording and that nothing here can raise.

import json  # noqa: E402
import subprocess  # noqa: E402

_HEAD = "a" * 40
_OLD = "b" * 40


def _run(name, conclusion, sha, created, status="completed", rid=1):
    return {"name": name, "status": status, "conclusion": conclusion, "headSha": sha,
            "createdAt": created, "databaseId": rid, "url": f"https://x/runs/{rid}"}


def _finish(runs, behind=lambda sha, head: 0):
    return doctor._ci_finish({"head_sha": _HEAD, "branch": "main"}, runs, behind=behind)


def test_ci_all_green_uses_the_newest_run_per_workflow():
    runs = [
        _run("linux-build", "failure", _OLD, "2026-09-02T13:00:00Z", rid=1),  # superseded
        _run("linux-build", "success", _HEAD, "2026-09-02T15:00:00Z", rid=2),
        _run("Licence and provenance", "success", _HEAD, "2026-09-02T15:00:00Z", rid=3),
        _run("Update SPONSORS.md", "skipped", _OLD, "2026-09-02T10:00:00Z", rid=4),
    ]
    info = _finish(runs)
    assert info["available"] is True and info["green"] is True
    assert (info["green_count"], info["total"]) == (3, 3)
    assert info["newest_sha"] == _HEAD and info["next"] is None and info["non_green"] == []
    assert [w["name"] for w in info["workflows"]] == sorted(
        ["linux-build", "Licence and provenance", "Update SPONSORS.md"])
    assert set(info["workflows"][0]) == {"name", "status", "conclusion", "head_sha",
                                         "created_at", "url", "id"}
    assert doctor._ci_row_lines(info) == ["ci          3/3 workflows green on aaaaaaa"]


def test_ci_one_failure_warns_and_names_the_next_step():
    runs = [
        _run("linux-build", "failure", _HEAD, "2026-09-02T15:00:00Z", rid=7),
        _run("Licence and provenance", "success", _HEAD, "2026-09-02T15:00:00Z", rid=8),
    ]
    info = _finish(runs)
    assert info["green"] is False and (info["green_count"], info["total"]) == (1, 2)
    assert [w["name"] for w in info["non_green"]] == ["linux-build"]
    lines = doctor._ci_row_lines(info)
    assert lines[0] == "ci          WARN: 1/2 workflows green on aaaaaaa (linux-build: failure)"
    assert lines[1] == "            next: gh run view 7 --log-failed  (https://x/runs/7)"


def test_ci_in_flight_run_is_not_green_and_says_watch():
    runs = [_run("launch-race-stress", "", _HEAD, "2026-09-02T17:00:00Z",
                 status="in_progress", rid=9)]
    info = _finish(runs)
    assert info["green"] is False and info["non_green"][0]["label"] == "in_progress"
    lines = doctor._ci_row_lines(info)
    assert lines[0] == ("ci          WARN: 0/1 workflows green on aaaaaaa "
                        "(launch-race-stress: in_progress)")
    assert lines[1] == "            next: gh run watch 9  (https://x/runs/9)"


def test_ci_stale_commit_says_how_far_behind():
    runs = [_run("linux-build", "success", _OLD, "2026-09-02T15:00:00Z", rid=1)]
    info = _finish(runs, behind=lambda sha, head: 3)
    assert info["behind"] == 3 and info["green"] is True
    assert doctor._ci_row_lines(info) == [
        "ci          1/1 workflows green on bbbbbbb, behind by 3 commits"]
    assert doctor._ci_row_lines(_finish(runs, behind=lambda sha, head: 1))[0].endswith(
        "behind by 1 commit")
    assert doctor._ci_row_lines(_finish(runs, behind=lambda sha, head: None)) == [
        "ci          1/1 workflows green on bbbbbbb (not an ancestor of HEAD)"]
    # A failure on the stale commit keeps both facts on one line.
    bad = [_run("linux-build", "failure", _OLD, "2026-09-02T15:00:00Z", rid=1)]
    assert doctor._ci_row_lines(_finish(bad, behind=lambda sha, head: 2))[0] == (
        "ci          WARN: 0/1 workflows green on bbbbbbb, behind by 2 commits "
        "(linux-build: failure)")


def test_ci_behind_is_zero_on_head_and_none_for_an_unknown_sha():
    assert doctor._ci_behind(_HEAD, _HEAD) == 0
    assert doctor._ci_behind(None, _HEAD) is None
    assert doctor._ci_behind("f" * 40, _HEAD) is None  # git: invalid revision range


def test_ci_empty_run_list_is_unknown_no_runs():
    info = _finish([])
    assert info["available"] is True and info["green"] is None and info["total"] == 0
    assert info["reason"] == "no-runs"
    assert doctor._ci_row_lines(info) == ["ci          unknown (no-runs)"]
    # Junk rows are ignored, never fatal.
    assert doctor._ci_classify([None, {}, {"name": ""}, "x"], _HEAD)["total"] == 0


def test_ci_missing_gh_yields_unknown_without_raising(monkeypatch):
    monkeypatch.setattr(doctor.shutil, "which", lambda name: None)
    info = doctor._ci_status("deadbeef", "main")
    assert info["available"] is False and info["reason"] == "no-gh"
    assert info["workflows"] == [] and info["green"] is None and "_proc" not in info
    assert doctor._ci_row_lines(info) == ["ci          unknown (no-gh)"]


def test_ci_non_github_or_missing_remote_is_unknown(monkeypatch):
    monkeypatch.setattr(doctor.shutil, "which", lambda name: "/usr/bin/gh")
    monkeypatch.setattr(doctor, "_git", lambda *a: "https://gitlab.com/o/r.git")
    assert doctor._ci_status("deadbeef", "main")["reason"] == "not-github"
    monkeypatch.setattr(doctor, "_git", lambda *a: None)
    assert doctor._ci_status("deadbeef", "main")["reason"] == "no-remote"
    monkeypatch.setattr(doctor, "_git", lambda *a: "git@github.com:o/r.git")
    assert doctor._ci_status("deadbeef", "HEAD")["reason"] == "detached"


def test_ci_timeout_errors_and_bad_output_are_unknown(monkeypatch):
    class Fake:
        def __init__(self, out=b"", err=b"", rc=0, slow=False):
            self.out, self.err, self.returncode, self.slow = out, err, rc, slow
            self.killed = False

        def communicate(self, timeout=None):
            if self.slow and not self.killed:
                raise subprocess.TimeoutExpired("gh", timeout)
            return self.out, self.err

        def kill(self):
            self.killed = True

    def status(proc):
        monkeypatch.setattr(doctor.shutil, "which", lambda name: "/usr/bin/gh")
        monkeypatch.setattr(doctor, "_git", lambda *a: "git@github.com:o/r.git")
        monkeypatch.setattr(doctor.subprocess, "Popen", lambda *a, **k: proc)
        return doctor._ci_status("deadbeef", "main")

    slow = Fake(slow=True)
    assert status(slow)["reason"] == "timeout" and slow.killed
    assert status(Fake(out=b"<html>"))["reason"] == "not-json"
    assert status(Fake(out=b'{"a": 1}'))["reason"] == "not-json"
    assert status(Fake(err=b"To get started with GitHub CLI, please run:  gh auth login",
                       rc=4))["reason"] == "auth"
    assert status(Fake(err=b"error connecting to api.github.com: dial tcp: no such host",
                       rc=1))["reason"] == "offline"
    assert status(Fake(err=b"failed to get runs: HTTP 404: Not Found", rc=1))["reason"] == "no-repo"
    for reason in ("timeout", "not-json", "auth", "offline"):
        assert doctor._ci_row_lines({"available": False, "reason": reason}) == [
            f"ci          unknown ({reason})"]
    # And the happy path through the same fake: the repo is parsed, runs land.
    good = json.dumps([_run("linux-build", "success", "deadbeef", "2026-09-02T15:00:00Z")])
    info = status(Fake(out=good.encode()))
    assert info["available"] is True and info["repo"] == "o/r" and info["green"] is True


def test_github_repo_from_remote_parses_every_url_form():
    f = doctor._github_repo_from_remote
    assert f("https://github.com/ekko1227/omnisim.git") == "ekko1227/omnisim"
    assert f("https://github.com/ekko1227/omnisim") == "ekko1227/omnisim"
    assert f("git@github.com:omnilink-tech/omnisim.git") == "omnilink-tech/omnisim"
    assert f("ssh://git@github.com/o/r.git") == "o/r"
    assert f("https://x-access-token:TOKEN@github.com/o/r.git") == "o/r"
    assert f("https://gitlab.com/o/r.git") is None
    assert f(None) is None and f("") is None
