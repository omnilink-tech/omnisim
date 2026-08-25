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

"""Regressions for the 2026-08-12 diagnostic round's isolation defects.

Every test here was written RED against a measured incident in
``results/cc_lane/20260812_round1_*`` and names the cell it came from. The
theme is one sentence: **a cell used to be able to destroy another cell, and
to be graded on a file it did not write.** Nothing measured through this lane
was trustworthy until each of these went green.

No Claude Code, no simulator, no repo copy -- synthetic trees and injected
process/listener tables only.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[2]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                  # noqa: E402
from agentbench.cc_lane import containment_guard as guard   # noqa: E402
from agentbench.cc_lane import evidence                     # noqa: E402
from agentbench.cc_lane import run_cc_cell as cell          # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging  # noqa: E402

WINDOWS = os.name == "nt"
needs_windows = pytest.mark.skipif(not WINDOWS,
                                   reason="junctions are a Windows feature")


# --- defect 1 + 2: a cell must not reap another cell's processes --------------
#
# MEASURED, `20260812_round1_r4_omnisim_c1`: its `post_session` PORT sweep
# terminated `omnisim_harness.py --port 6789`, an `omnisim-bin.exe` whose argv
# names instance `20260812_102807_omnisim_A1` (a DIFFERENT cell), and ten
# `husky_random.py` controllers. The victim cell's agent then spent four tool
# calls hunting a capture-service bug that did not exist. The port sweep killed
# any listener on 6789-6792 machine-wide; ownership was never consulted.


def _claim(pid, ws, **kw):
    rec = {"pid": pid, "workspace": str(ws), "lane": kw.get("lane", "l"),
           "sim": kw.get("sim", "omnisim"), "task": kw.get("task", "A1"),
           "started_ts": kw.get("started_ts", time.time())}
    return rec


def test_port_sweep_spares_a_listener_owned_by_another_active_cell(tmp_path):
    """The r4_c1 incident, reduced: our sweep sees a harness on 6789 that
    belongs to a live A1 cell. It must be recorded and left alone."""
    mine = _claim(1000, tmp_path / "ws_mine", task="R4")
    theirs = _claim(2000, tmp_path / "ws_theirs", task="A1")
    killed = []

    recs = staging.reap_port_listeners(
        ports=(6789,),
        listeners=[(6789, 2100)],                     # the victim harness
        processes={2100: ("python.exe",
                          "python.exe scripts/harness/omnisim_harness.py "
                          "--port 6789")},
        ancestry={2100: [2000]},                      # child of THEIR cell
        mine=mine, others=[theirs],
        kill=lambda pid, grace_s=1.5: (killed.append(pid), ("killed", None))[1])

    assert killed == [], "another cell's harness was killed: %r" % killed
    assert [r["action"] for r in recs] == ["skipped_other_cell"]
    assert recs[0]["owner"]["task"] == "A1"


def test_port_sweep_still_reaps_an_unclaimed_leak(tmp_path):
    """The original measured need (phasew_cc_v1 B2 r0-r2) must survive the
    fix: a harness leaked by a FINISHED cell is owned by nobody and is still
    reaped, or every later cell reads a foreign world as its task world."""
    mine = _claim(1000, tmp_path / "ws_mine")
    killed = []
    recs = staging.reap_port_listeners(
        ports=(6789,),
        listeners=[(6789, 3100)],
        processes={3100: ("python.exe", "python.exe omnisim_harness.py")},
        ancestry={3100: [1]},
        mine=mine, others=[],
        kill=lambda pid, grace_s=1.5: (killed.append(pid), ("killed", None))[1])
    assert killed == [3100]
    assert recs[0]["action"] == "killed"


def test_port_sweep_reaps_our_own_engine(tmp_path):
    """...and a straggler of OUR OWN session is still ours to reap."""
    mine = _claim(1000, tmp_path / "ws_mine")
    theirs = _claim(2000, tmp_path / "ws_theirs")
    killed = []
    recs = staging.reap_port_listeners(
        ports=(6791,),
        listeners=[(6791, 1100)],
        processes={1100: ("python.exe", "python.exe omnisim_capture.py")},
        ancestry={1100: [1000]},
        mine=mine, others=[theirs],
        kill=lambda pid, grace_s=1.5: (killed.append(pid), ("killed", None))[1])
    assert killed == [1100]
    assert recs[0]["action"] == "killed"


def test_port_sweep_spares_a_listener_inside_another_cells_workspace(tmp_path):
    """Ownership by PATH as well as by lineage: the engine whose argv names
    `...instances\\20260812_102807_omnisim_A1\\...` is that cell's even when
    our ancestry table cannot see the tree (psutil unavailable)."""
    mine = _claim(1000, tmp_path / "20260812_095153_omnisim_R4")
    theirs = _claim(2000, tmp_path / "20260812_102807_omnisim_A1")
    killed = []
    recs = staging.reap_port_listeners(
        ports=(6790,),
        listeners=[(6790, 4100)],
        processes={4100: ("omnisim-bin.exe",
                          str(tmp_path / "20260812_102807_omnisim_A1"
                              / "msys64" / "bin" / "omnisim-bin.exe"))},
        ancestry={},                                  # no lineage available
        mine=mine, others=[theirs],
        kill=lambda pid, grace_s=1.5: (killed.append(pid), ("killed", None))[1])
    assert killed == []
    assert recs[0]["action"] == "skipped_other_cell"


def test_active_cell_registry_round_trips_and_prunes_dead_cells(tmp_path):
    """The registry the sweeps consult. A cell whose PID is gone must never
    protect anything -- otherwise one crashed cell disarms the leak sweep for
    the rest of the campaign."""
    live = concurrency.register_cell(tmp_path, pid=111, lane="a", sim="omnisim",
                                     task="A1", workspace=tmp_path / "w1",
                                     run_dir=tmp_path / "r1")
    concurrency.register_cell(tmp_path, pid=222, lane="b", sim="omnisim",
                              task="R4", workspace=tmp_path / "w2",
                              run_dir=tmp_path / "r2")
    alive = {111}
    got = concurrency.active_cells(tmp_path, alive=lambda p: p in alive)
    assert [c["pid"] for c in got] == [111]
    got = concurrency.active_cells(tmp_path, exclude_pid=111,
                                   alive=lambda p: p in alive)
    assert got == []
    concurrency.unregister_cell(live)
    assert concurrency.active_cells(tmp_path, alive=lambda p: True) == []


# --- defect 1 (agent side): no machine-wide process reaping from a cell -------
#
# MEASURED, `20260812_round1_a1_omnisim_c1`: the agent's own cleanup matched on
# `Name -eq 'omnisim-bin.exe'` and reported "remaining omnisim-bin: 0" -- after
# reaping a concurrent lane's engine twice. Containment scoped FILES and said
# nothing about PROCESSES.

GUARD_CFG = {"workspace": "C:/ws", "repo": "O:/omnisim",
             "scratch_root": None, "junction_dirs": ["projects"],
             "exclude_prefixes": [], "exclude_files": [], "protect": []}


@pytest.mark.parametrize("command", [
    "taskkill /IM omnisim-bin.exe /F",
    "Get-Process -Name omnisim-bin | Stop-Process -Force",
    "Get-Process omnisim-bin.exe | Stop-Process",
    "Stop-Process -Name python -Force",
    "pkill -f omnisim-bin",
    "killall omnisim-bin",
    "Get-CimInstance Win32_Process | Where-Object { $_.Name -eq "
    "'omnisim-bin.exe' } | ForEach-Object { Stop-Process -Id $_.ProcessId }",
])
def test_guard_denies_machine_wide_process_reaping(command):
    allow, reason, matched = guard.decide("Bash", {"command": command},
                                          GUARD_CFG)
    assert allow is False, "machine-wide kill allowed: %r" % command
    assert matched == "machine_wide_kill"
    # Product language only: a refusal that names the benchmark tells the agent
    # it is being measured.
    for word in ("benchmark", "grader", "answer key", "cell"):
        assert word not in reason.lower()


@pytest.mark.parametrize("command", [
    "taskkill /PID 1234 /T /F",
    "Stop-Process -Id 1234",
    "kill 1234",
    "kill -9 $PID",
    "Get-Process -Name omnisim-bin",           # a READ is not a kill
    "python -m omnisim run-headless w.wbt --duration 10",
])
def test_guard_allows_pid_scoped_and_read_only_process_calls(command):
    allow, _reason, _matched = guard.decide("Bash", {"command": command},
                                            GUARD_CFG)
    assert allow is True, "refused a legitimate call: %r" % command


# --- defect 2 (repo half): the pre-session sweep must not eat a live cell -----
#
# MEASURED, `20260812_round1_r4_omnisim_c1` + `..._a1_omnisim_c3`: the repo
# sweep runs with `window=None` over the REAL `projects/` tree and
# `preserved_and_deleted` `husky_random_10.wbt` -- another lane's LIVE
# deliverable -- into the wrong cell's directory.


def _fake_repo(tmp_path, rels):
    repo = tmp_path / "repo"
    (repo / "projects").mkdir(parents=True)
    for rel in rels:
        p = repo / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("#VRML_SIM\n", encoding="utf-8")
    return repo


def test_repo_sweep_leaves_files_a_live_cell_may_still_be_writing(tmp_path,
                                                                 monkeypatch):
    repo = _fake_repo(tmp_path, ["projects/w/live.wbt", "projects/w/old.wbt"])
    monkeypatch.setattr(staging, "repo_sweep_candidates",
                        lambda repo=None, subdir=None: (
                            {"projects/w/live.wbt", "projects/w/old.wbt"},
                            set()))
    t_now = time.time()
    os.utime(repo / "projects/w/live.wbt", (t_now, t_now))
    os.utime(repo / "projects/w/old.wbt", (t_now - 9000, t_now - 9000))

    recs = staging.sweep_repo_junction_artifacts(
        tmp_path / "quarantine", window=None, repo=repo,
        protect_after_ts=t_now - 600)           # a cell started 10 min ago

    by_rel = {r["rel"]: r for r in recs if r.get("rel")}
    assert (repo / "projects/w/live.wbt").is_file(), \
        "the live cell's deliverable was deleted out from under it"
    assert by_rel["projects/w/live.wbt"]["action"] == \
        "skipped_owned_by_active_cell"
    assert not (repo / "projects/w/old.wbt").is_file()
    assert by_rel["projects/w/old.wbt"]["action"] == "preserved_and_deleted"


# --- defect 3: the cell was graded on the WRONG FILE --------------------------
#
# MEASURED, `20260812_round1_a1_omnisim_c1`: `artifact.found` resolved to
# `repo_artifacts/.../.capture_newton_husky_swarm_drive.wbt` -- the shipped
# 8-Husky demo plus a capture supervisor -- and the cell FAILED every
# assertion though its ten robots drove 5.14-30.27 m. `_HARNESS_SIBLING`
# matches `.harness_*` and not `.capture_*`, and discovery is newest-first.


def test_discover_artifact_never_collects_a_tool_written_sibling(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    art = tmp_path / "repo_artifacts" / "projects" / "worlds"
    art.mkdir(parents=True)
    start = time.time() - 100
    mine = art / "husky_random_swarm_10.wbt"
    mine.write_text("#VRML_SIM R2025a utf8\n", encoding="utf-8")
    os.utime(mine, (start + 10, start + 10))
    for scratch in (".capture_newton_husky_swarm_drive.wbt",
                    ".harness_husky_random_swarm_10.wbt",
                    ".omnisim_runaway_husky_random_swarm_10.wbt"):
        p = art / scratch
        p.write_text("#VRML_SIM R2025a utf8\n", encoding="utf-8")
        os.utime(p, (start + 50, start + 50))       # NEWER than the agent's

    found, rule = cell.discover_artifact(
        ws, [], start, extra_roots=(tmp_path / "repo_artifacts",))

    assert found is not None
    assert Path(found).name == "husky_random_swarm_10.wbt", \
        "collected tool scratch as the deliverable: %s" % found
    assert "sibling" in rule or "scratch" in rule


def test_discover_artifact_skips_siblings_in_the_workspace_scan(tmp_path):
    ws = tmp_path / "ws" / "worlds"
    ws.mkdir(parents=True)
    start = time.time() - 100
    mine = ws / "arena.wbt"
    mine.write_text("x", encoding="utf-8")
    os.utime(mine, (start + 10, start + 10))
    sib = ws / ".capture_arena.wbt"
    sib.write_text("x", encoding="utf-8")
    os.utime(sib, (start + 60, start + 60))
    found, _rule = cell.discover_artifact(tmp_path / "ws", [], start)
    assert Path(found).name == "arena.wbt"


def test_discover_artifact_ranks_the_agents_world_above_scratch(tmp_path):
    """Ranking, not mtime: a world under a `verify/` or `.omnisim_*` scratch
    directory loses to one the agent wrote in a project, even when newer."""
    ws = tmp_path / "ws"
    (ws / "projects" / "worlds").mkdir(parents=True)
    (ws / "verify").mkdir(parents=True)
    start = time.time() - 100
    real = ws / "projects" / "worlds" / "arena.wbt"
    real.write_text("x", encoding="utf-8")
    os.utime(real, (start + 10, start + 10))
    scratch = ws / "verify" / "broken.wbt"
    scratch.write_text("x", encoding="utf-8")
    os.utime(scratch, (start + 60, start + 60))
    found, _rule = cell.discover_artifact(ws, [], start)
    assert Path(found).name == "arena.wbt"


# --- defect 4: a killed cell mis-read as rate-limited, then blocked forever ---
#
# MEASURED, `20260812_round1_a1_omnisim_c2`: the cell recorded
# `rate_limit_deferrals: [{"marker": "429", ...}]`, tore its workspace down and
# slept 900 s holding the task lock -- no artifact, no grade, no row. The
# session was KILLED, not limited: Windows reports a killed process as
# `rc=4294967295`, our own `launch_error` text quotes that number, and the
# bare substring `"429"` is a rate-limit marker. "4294967295" contains "429".


def test_a_killed_session_is_not_mistaken_for_a_rate_limit():
    launch_error = ("no result event in the session stream (rc=4294967295, "
                    "375 events, 130 assistant turns, 64 tool calls): ")
    assert concurrency.rate_limit_reason(None, launch_error) is None


@pytest.mark.parametrize("text,expected", [
    ("HTTP 429 returned by the api", "429"),
    ("api error: status 429", "429"),
    ("rc=4294967295", None),
    ("exit code 429496", None),
    ("upload of 1429 bytes failed", None),
])
def test_429_is_matched_as_a_status_code_not_as_a_substring(text, expected):
    got = concurrency.rate_limit_reason({"is_error": True, "result": text}, "")
    assert got == expected


def test_the_deferral_source_is_the_childs_stderr_not_our_own_summary(tmp_path):
    """Our synthesized `launch_error` must never be scanned for limit markers:
    it quotes numbers and counts we generated, and a marker found there is an
    artefact of our own prose."""
    meta = {"launch_error": "no result event ... (rc=4294967295)",
            "stderr_tail": ""}
    assert cell.deferral_reason(None, meta) is None
    meta = {"launch_error": "no result event ... (rc=1)",
            "stderr_tail": "Claude usage limit reached"}
    assert cell.deferral_reason(None, meta) == "usage limit"


def test_a_backoff_is_visible_to_the_operator(tmp_path):
    """A cell asleep in a rate-limit backoff used to be indistinguishable from
    a hung one (`--status` said `running`/`no stream activity` and nothing
    else). It must say what it is waiting for and until when."""
    (tmp_path / "cell_report.json").write_text(json.dumps({
        "sim": "omnisim", "task": "A1",
        "utc_start": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "deferred_until_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() + 600)),
        "rate_limit_deferrals": [{"marker": "usage limit", "attempt": 0,
                                  "backoff_s": 900.0}],
    }), encoding="utf-8")
    s = evidence.cell_status(tmp_path)
    assert s["state"].startswith("deferred")
    assert "deferred" in evidence.render_status(tmp_path)


# --- defect 5: the preserved workspace did not contain the deliverable -------
#
# MEASURED, 3/3 cells: `excluded_links: lib, msys64, projects, resources`.
# `projects/` is a junction, the agent's world is written through it, so the
# preserved copy -- the artifact of record -- never held the artifact. Only
# `repo_artifacts` recovered it, and that never runs on a killed cell.


@needs_windows
def test_preserved_workspace_keeps_session_writes_made_through_a_junction(
        tmp_path):
    target = tmp_path / "real_projects" / "worlds"
    target.mkdir(parents=True)
    old = target / "shipped_demo.wbt"
    old.write_text("shipped", encoding="utf-8")
    os.utime(old, (time.time() - 9000, time.time() - 9000))

    ws = tmp_path / "ws"
    ws.mkdir()
    (ws / "AGENTS.md").write_text("hi", encoding="utf-8")
    staging.make_junction(ws / "projects", tmp_path / "real_projects")

    start = time.time() - 60
    authored = target / "arena.wbt"
    authored.write_text("#VRML_SIM\n", encoding="utf-8")

    man = evidence.preserve_workspace(ws, tmp_path / "dest",
                                      newer_than=start, link_dirs=("projects",))

    kept = tmp_path / "dest" / "projects" / "worlds" / "arena.wbt"
    assert kept.is_file(), "the deliverable is still missing from the record"
    assert not (tmp_path / "dest" / "projects" / "worlds"
                / "shipped_demo.wbt").exists(), \
        "the whole junction target was copied, not the session window"
    assert any(e["path"] == "projects/worlds/arena.wbt"
               for e in man["link_window_files"])
    # The link is still RECORDED as excluded: nothing is silently followed.
    assert any(e["path"] == "projects" for e in man["excluded_links"])


def test_preserve_workspace_without_a_window_is_byte_identical(tmp_path):
    """The new parameters are opt-in: a caller that passes neither gets the
    old behaviour exactly."""
    ws = tmp_path / "ws"
    ws.mkdir()
    (ws / "a.txt").write_text("a", encoding="utf-8")
    man = evidence.preserve_workspace(ws, tmp_path / "d1")
    assert man["files"] == 1
    assert man.get("link_window_files") == []


# --- defect 8: --status counted the queue as session time --------------------
#
# MEASURED: `--status` reported 27.0/36 min for a cell whose session had used
# 8 -- `utc_start` is stamped before the same-task lock is acquired, and the
# cell then waited 19 min in the queue. The operator kills at plateau, so an
# instrument that overstates elapsed time by 3x directly misleads the kill
# decision. Queue time is already EXCLUDED from the enforced deadline
# (`budget.queue_waits`); only the display disagreed.


def test_status_reports_session_elapsed_not_queue_plus_session(tmp_path):
    now = time.time()
    (tmp_path / "cell_report.json").write_text(json.dumps({
        "sim": "omnisim", "task": "A1",
        "utc_start": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                   time.gmtime(now - 27 * 60)),
        "budget": {"task_timeout_s": 2160.0, "session_budgets_s": [2160.0],
                   "queue_waits": [{"what": "same-task lock",
                                    "waited_s": 19 * 60}]},
    }), encoding="utf-8")
    s = evidence.cell_status(tmp_path)
    assert s["elapsed_s"] == pytest.approx(27 * 60, abs=90)
    assert s["session_elapsed_s"] == pytest.approx(8 * 60, abs=90), \
        "the queue wait is still being counted as session time"
    assert s["queued_s"] == pytest.approx(19 * 60, abs=5)
    text = evidence.render_status(tmp_path)
    assert "queued" in text


def test_status_session_elapsed_prefers_the_recorded_session_start(tmp_path):
    now = time.time()
    (tmp_path / "cell_report.json").write_text(json.dumps({
        "sim": "omnisim", "task": "A1",
        "utc_start": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                   time.gmtime(now - 3600)),
        "session_started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                             time.gmtime(now - 300)),
        "budget": {"session_budgets_s": [2160.0]},
    }), encoding="utf-8")
    s = evidence.cell_status(tmp_path)
    assert s["session_elapsed_s"] == pytest.approx(300, abs=30)


# --- defect 9: the session wrote memory into the operator's real ~/.claude ----


def test_session_home_is_collected_as_evidence(tmp_path):
    home = tmp_path / "claude_home"
    # The real slug convention, copied from the operator's own tree:
    # `C--Users-<user>-AppData-Local-Temp-agentbench-cc-instances-20260801-...`
    slug = home / "projects" / "C--tmp-agentbench-cc-instances-x-omnisim-A1"
    (slug / "memory").mkdir(parents=True)
    (slug / "memory" / "MEMORY.md").write_text("learned a thing",
                                               encoding="utf-8")
    (slug / "abc.jsonl").write_text("{}\n", encoding="utf-8")   # transcript
    other = home / "projects" / "o--omnisim"
    (other / "memory").mkdir(parents=True)
    (other / "memory" / "MEMORY.md").write_text("the operator's own",
                                                encoding="utf-8")

    run_dir = tmp_path / "cell"
    run_dir.mkdir()
    rec = cell.collect_session_home(run_dir,
                                    workspace=Path("C:/tmp/agentbench_cc/"
                                                   "instances/x_omnisim_A1"),
                                    session_id="abc", claude_home=home)

    kept = run_dir / "claude_home" / "memory" / "MEMORY.md"
    assert kept.is_file(), "the session's memory was left outside the evidence"
    assert kept.read_text(encoding="utf-8") == "learned a thing"
    assert rec["files"] >= 1
    assert (other / "memory" / "MEMORY.md").read_text(encoding="utf-8") == \
        "the operator's own", "collected from the operator's own project slug"


def test_session_home_records_the_miss_instead_of_inventing_one(tmp_path):
    run_dir = tmp_path / "cell"
    run_dir.mkdir()
    rec = cell.collect_session_home(run_dir, workspace=tmp_path / "nope",
                                    session_id=None,
                                    claude_home=tmp_path / "empty")
    assert rec["files"] == 0
    assert rec["slug_dir"] is None
    assert rec["reason"]


# --- the WIRING: the units above must actually be reached by run_cell --------
#
# Every defect here was a wiring defect as much as a logic one -- the sweeps
# had no ownership to consult and the mirror had no window to apply. A green
# unit test for a function nothing calls with the right arguments proves
# nothing, so these drive `run_cell` and read what it passed.

from agentbench.cc_lane.test_cc_lane import _minimal_cell_env   # noqa: E402


def _cc_meta(**kw):
    base = {"permission_mode": "x", "cli_command": "claude -p", "rc": 0,
            "timed_out": False, "wall_s": 1.0, "launch_error": None,
            "stderr_tail": ""}
    base.update(kw)
    return base


def test_run_cell_hands_the_port_sweep_every_other_live_cell(tmp_path,
                                                             monkeypatch):
    seen = []
    # AFTER `_minimal_cell_env`: it installs its own doubles on the same
    # module attributes, and a spy set first is simply overwritten (which is
    # how the first draft of this test passed against unwired code).
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_cc_meta())
    monkeypatch.setattr(staging, "reap_port_listeners",
                        lambda **kw: seen.append(kw) or [])
    lock_root = tmp_path / "locks"
    # A neighbouring cell, claimed under a pid that is NOT ours (so
    # `exclude_pid` keeps it) and asserted live (so the test does not depend on
    # a second real process being scheduled).
    concurrency.register_cell(
        lock_root, pid=os.getpid() + 1, lane="other", sim="omnisim", task="R4",
        workspace=tmp_path / "other_ws", run_dir=tmp_path / "other")
    monkeypatch.setattr(concurrency, "_pid_alive", lambda pid: True)

    try:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False,
                      lock_root=lock_root, repeat=0)
    except SystemExit:
        pass

    assert seen, "the port sweep was never called"
    for call in seen:
        assert "mine" in call and "others" in call, \
            "the sweep was called with no ownership at all: %r" % call
        assert any(c.get("task") == "R4" for c in call["others"]), \
            "run_cell did not tell the sweep about the other live cell"


def test_run_cell_protects_a_live_cells_files_from_the_pre_session_sweep(
        tmp_path, monkeypatch):
    seen = []
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_cc_meta())
    monkeypatch.setattr(staging, "sweep_repo_junction_artifacts",
                        lambda dest, **kw: seen.append(kw) or [])
    lock_root = tmp_path / "locks"
    started = time.time() - 300
    concurrency.register_cell(
        lock_root, pid=os.getpid() + 1, lane="other", sim="omnisim", task="R4",
        workspace=tmp_path / "other_ws", run_dir=tmp_path / "other",
        started_ts=started)
    monkeypatch.setattr(concurrency, "_pid_alive", lambda pid: True)

    try:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False,
                      lock_root=lock_root, repeat=0)
    except SystemExit:
        pass

    pre = [c for c in seen if c.get("window") is None]
    assert pre, "the pre-session repo sweep never ran"
    assert pre[0].get("protect_after_ts") == pytest.approx(started, abs=2), \
        ("the unwindowed sweep ran with nothing protected -- this is the call "
         "that deleted a live cell's deliverable")


def test_run_cell_mirrors_the_session_window_through_the_junctions(
        tmp_path, monkeypatch):
    seen = {}
    real = evidence.WorkspaceMirror

    class SpyMirror(real):
        def __init__(self, ws, dest, **kw):
            seen.update(kw)
            super().__init__(ws, dest, **kw)

    monkeypatch.setattr(evidence, "WorkspaceMirror", SpyMirror)
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_cc_meta())
    try:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False, repeat=0)
    except SystemExit:
        pass
    assert seen.get("newer_than"), \
        "the live mirror runs with no session window, so a killed cell's " \
        "deliverable is still lost"
    assert "projects" in (seen.get("link_dirs") or ())


def test_run_cell_records_the_session_start_and_the_session_home(tmp_path,
                                                                 monkeypatch):
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_cc_meta())
    try:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False, repeat=0)
    except SystemExit:
        pass
    rep = json.loads((tmp_path / "out" / "cell_report.json")
                     .read_text(encoding="utf-8"))
    assert rep.get("session_started_utc"), \
        "no session start recorded, so --status must guess it from utc_start"
    assert rep.get("session_start_ts")
    assert "session_home" in rep, \
        "the session's ~/.claude state is not accounted for anywhere"


# --- defect 7: repo-wide search is blind behind the workspace junctions ------
#
# MEASURED, 3/3 cells: `Glob **/controllers/drive_forward/*.py` at the
# workspace root returns *No files found*; the identical Glob at `o:\omnisim`
# finds it. One cell's `find . -type d -name controllers` returned `scripts/`
# and `tests/` and NONE of the 47 shipped controller directories, so its agent
# concluded the product ships no examples. Re-verified while writing this:
# `rg --files` under a junction lists 0 files, and rg skips symlinked FILES
# too -- so this is not fixable by swapping the link type, only by staging a
# real tree. Until then the bias is RECORDED on every cell rather than left to
# distort silently.


@needs_windows
def test_search_visibility_reports_what_the_agents_tools_cannot_see(tmp_path):
    real = tmp_path / "real_projects" / "demos" / "controllers" / "drive_forward"
    real.mkdir(parents=True)
    (real / "drive_forward.py").write_text("x", encoding="utf-8")
    (tmp_path / "real_projects" / "demos" / "worlds").mkdir(parents=True)
    (tmp_path / "real_projects" / "demos" / "worlds" / "a.wbt").write_text(
        "x", encoding="utf-8")

    ws = tmp_path / "ws"
    (ws / "scripts").mkdir(parents=True)
    staging.make_junction(ws / "projects", tmp_path / "real_projects")

    vis = cell.search_visibility(ws, junction_dirs=("projects",))
    assert vis["biased"] is True, \
        "the junction blindness is not being measured at all"
    assert vis["probes"]["controller_dirs"]["visible_to_search"] == 0
    assert vis["probes"]["controller_dirs"]["hidden_behind_links"] >= 1
    assert vis["probes"]["worlds"]["hidden_behind_links"] >= 1


def test_search_visibility_says_unbiased_when_nothing_is_hidden(tmp_path):
    ws = tmp_path / "ws" / "projects" / "demos" / "controllers" / "d"
    ws.mkdir(parents=True)
    (ws / "d.py").write_text("x", encoding="utf-8")
    vis = cell.search_visibility(tmp_path / "ws", junction_dirs=("projects",))
    assert vis["biased"] is False
    assert vis["probes"]["controller_dirs"]["visible_to_search"] == 1


def test_run_cell_records_search_visibility_on_every_cell(tmp_path,
                                                          monkeypatch):
    _minimal_cell_env(monkeypatch, tmp_path, staged=["six_huskies.wbt"],
                      cc_meta=_cc_meta())
    try:
        cell.run_cell("omnisim", "B1_overlap_audit", root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False, repeat=0)
    except SystemExit:
        pass
    rep = json.loads((tmp_path / "out" / "cell_report.json")
                     .read_text(encoding="utf-8"))
    assert "search_visibility" in rep, \
        "nothing on the cell says what the agent's search could not reach"
    assert "probes" in rep["search_visibility"]
