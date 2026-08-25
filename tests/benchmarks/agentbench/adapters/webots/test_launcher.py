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

"""Unit tests for the Webots-lane launcher. **No WSL, no Webots, no network.**

Everything WSL-dependent in :mod:`agentbench.adapters.webots.launcher` is
behind pure functions -- command construction, path translation, world
injection, the artifact-dir contract -- and those are what these tests pin.
The live end-to-end path (WSL2 + upstream R2025a) is exercised by the Phase W
bring-up documented in ``BRINGUP.md``, not here: the control arm's unit tests
must run on a machine that has none of our stack (same rule as
``test_webots_adapter.py``).

    pytest tests/benchmarks/agentbench/adapters/webots -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.webots import launcher, recording  # noqa: E402
from agentbench.common.worldtext import INJECTED_PREFIX  # noqa: E402


# --- path translation --------------------------------------------------------


def test_windows_path_translates_to_mnt():
    assert launcher.win_to_wsl_path(r"O:\omnisim\a b\w.wbt") == \
        "/mnt/o/omnisim/a b/w.wbt"
    assert launcher.win_to_wsl_path("C:/Users/x/run") == "/mnt/c/Users/x/run"


def test_posix_paths_pass_through_unchanged():
    assert launcher.win_to_wsl_path("/tmp/x/y") == "/tmp/x/y"


def test_unc_paths_are_refused_not_mistranslated():
    """A \\\\wsl$ path has no /mnt form; guessing one points the run at a
    directory that does not exist."""
    with pytest.raises(ValueError):
        launcher.win_to_wsl_path(r"\\wsl$\Ubuntu-22.04\tmp\x")


def test_relative_paths_are_refused():
    with pytest.raises(ValueError):
        launcher.win_to_wsl_path("relative/only")


def test_wsl_to_win_roundtrip():
    assert launcher.wsl_to_win_path("/mnt/o/omnisim/x") == r"O:\omnisim\x"
    win = r"O:\omnisim\tests\benchmarks"
    assert launcher.wsl_to_win_path(launcher.win_to_wsl_path(win)) == win


def test_non_mnt_wsl_paths_are_refused():
    with pytest.raises(ValueError):
        launcher.wsl_to_win_path("/tmp/agentbench_webots/x")


# --- port discipline ---------------------------------------------------------


def test_the_default_port_is_inside_the_webots_lane_range():
    lo, hi = launcher.WEBOTS_PORT_RANGE
    assert lo <= launcher.DEFAULT_PORT <= hi
    assert launcher.check_port(launcher.DEFAULT_PORT) == launcher.DEFAULT_PORT


def test_omnisims_scan_range_is_refused():
    """Baseline sec. 6: 1234-1244 is OmniSim's auto-scan range and an
    omnisim-bin has been observed LISTENING on 1234. Never share it."""
    for port in (1234, 1240, 1244):
        with pytest.raises(ValueError):
            launcher.check_port(port)


# --- the invocation ----------------------------------------------------------


def test_invocation_matches_the_baseline_recipe():
    """Every flag of webots-control-baseline.md sec. 3, and nothing renamed."""
    inv = launcher.webots_invocation("/tmp/w/project/worlds/x.wbt", "/tmp/w/out",
                                     perf_steps=625, port=1504, timeout_s=420)
    for flag in ("--batch", "--mode=fast", "--no-rendering", "--minimize",
                 "--stdout", "--stderr", "--port=1504"):
        assert flag in inv, flag
    assert inv.startswith("WEBOTS_HOME=%s " % launcher.WEBOTS_HOME), \
        "WEBOTS_HOME must be a per-process prefix (baseline sec. 2)"
    assert "timeout 420" in inv
    assert "xvfb-run -a" in inv
    assert '--log-performance="/tmp/w/out/perf.txt",625' in inv
    assert inv.endswith('"/tmp/w/project/worlds/x.wbt"')


def test_invocation_refuses_a_colliding_port():
    with pytest.raises(ValueError):
        launcher.webots_invocation("/w.wbt", "/out", perf_steps=1, port=1234)


def test_basic_time_step_is_parsed_with_upstreams_default():
    assert launcher.parse_basic_time_step_ms("WorldInfo {\n  basicTimeStep 16\n}") \
        == pytest.approx(16.0)
    assert launcher.parse_basic_time_step_ms("WorldInfo {\n}") == \
        pytest.approx(32.0)


# --- trap 3: rotations of exactly +/-pi --------------------------------------


def test_pi_rotations_are_flagged():
    text = ("DEF A Foo {\n  rotation 0 0 1 -3.14159\n}\n"
            "DEF B Foo {\n  rotation 0 0 1 2.9\n}\n"
            "DEF C Foo {\n  rotation 0 0 1 3.1415926\n}\n")
    assert launcher.pi_rotation_lines(text) == [2, 8]


def test_the_bringup_world_carries_no_pi_rotation():
    """The world this lane authored must honour the trap it documents."""
    world = launcher.WEBOTS_LANE / "worlds" / "bringup_pioneer.wbt"
    text = world.read_text(encoding="utf-8")
    assert launcher.pi_rotation_lines(text) == []


# --- injection ---------------------------------------------------------------


def test_injection_appends_the_recorder_and_preserves_the_world():
    world = "#VRML_SIM R2025a utf8\nWorldInfo {\n}\nDEF X Robot {\n}\n"
    out = launcher.inject_recorder_text(world, "/tmp/run/out", duration=12.5,
                                        contact_steps=7)
    assert out.startswith(world)
    assert 'controller "%s"' % launcher.RECORDER_NAME in out
    assert "supervisor TRUE" in out
    assert '"--out-dir=/tmp/run/out"' in out
    assert '"--duration=12.5"' in out
    assert '"--contact-steps=7"' in out


def test_injected_worlds_use_the_shared_marker_prefix():
    """common.worldtext.pick_artifact skips this prefix on BOTH arms; a
    different one here would let an injected copy be graded as the agent's
    artifact."""
    assert launcher.INJECTED_PREFIX == INJECTED_PREFIX


def test_the_recorder_controller_dir_exists_and_is_named_for_upstream():
    """Upstream resolves ``controllers/<name>/<name>.py``; a mismatch means
    the recorder silently never starts."""
    d = launcher.RECORDER_DIR
    assert d.is_dir()
    assert (d / ("%s.py" % launcher.RECORDER_NAME)).is_file()


def test_project_root_resolution_walks_to_the_worlds_parent(tmp_path):
    w = tmp_path / "proj" / "worlds" / "sub" / "x.wbt"
    w.parent.mkdir(parents=True)
    w.write_text("#", encoding="utf-8")
    assert launcher.project_root_for_world(w) == (tmp_path / "proj").resolve()
    flat = tmp_path / "scratch" / "x.wbt"
    flat.parent.mkdir()
    flat.write_text("#", encoding="utf-8")
    assert launcher.project_root_for_world(flat) == tmp_path.resolve()


# --- the run script ----------------------------------------------------------


def _script(**kw):
    args = dict(work_dir="/tmp/agentbench_webots/t1",
                project_src_wsl="/mnt/o/omnisim/lane",
                injected_world_wsl="/mnt/c/run/_agentbench_x.wbt",
                recorder_src_wsl="/mnt/o/omnisim/rec/agentbench_webots_recorder",
                run_dir_wsl="/mnt/c/run", world_name="x.wbt", perf_steps=625)
    args.update(kw)
    return launcher.build_run_script(**args)


def test_run_script_copies_the_recorder_last_so_it_wins():
    s = _script()
    project_copy = s.index('cp -r "/mnt/o/omnisim/lane/controllers/."')
    purge = s.index('rm -rf "$WORK/project/controllers/%s"'
                    % launcher.RECORDER_NAME)
    recorder_copy = s.index('cp -r "/mnt/o/omnisim/rec/agentbench_webots_recorder"')
    assert project_copy < purge < recorder_copy, \
        "a project-supplied recorder impostor must be overwritten"


def test_run_script_emits_the_markers_and_copies_back():
    s = _script()
    assert 'echo "RC=$RC"' in s
    assert "WALL=" in s
    assert 'cp -r "$WORK/out/." "/mnt/c/run/"' in s
    assert 'echo "COPIED=1"' in s
    # the version witness for process.json
    assert "version.txt" in s


def test_markers_are_parsed_and_junk_is_ignored():
    got = launcher.parse_markers(
        "noise\nRC=0\nWALL=5.891\nCOPIED=1\nRC=notanint\n")
    assert got["rc"] == 0
    assert got["wall_s"] == pytest.approx(5.891)
    assert got["copied"] is True
    empty = launcher.parse_markers("")
    assert empty == {"rc": None, "wall_s": None, "copied": False}


# --- the artifact-dir contract ----------------------------------------------
#
# The launcher's whole reason to exist is producing what recording.read_run
# parses. These tests pin the contract WITHOUT running anything: the exact
# filenames the launcher writes/copies back must be the ones the reader
# tries.


def test_process_facts_round_trip_through_the_reader(tmp_path):
    facts = launcher.process_facts(
        rc=0, timed_out=False, wall_s=5.89, attempts_used=1,
        perf_log=tmp_path / "perf.txt",
        command="WEBOTS_HOME=/opt/upstream-webots/R2025a timeout 420 ...",
        webots_version="R2025a", world=r"O:\w\x.wbt")
    (tmp_path / "process.json").write_text(json.dumps(facts),
                                           encoding="utf-8")
    (tmp_path / "perf.txt").write_text("TOT: 625 steps\n", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.process["exit_code"] == 0
    assert run.process["timed_out"] is False
    assert run.process["webots_version"] == "R2025a"
    assert run.process["attempts_used"] == 1
    assert run.perf_nonempty is True
    assert run.world == r"O:\w\x.wbt"


def test_every_artifact_the_launcher_writes_is_a_reader_candidate(tmp_path):
    """Filenames the launcher produces vs recording.py's candidate lists."""
    produced = {
        "trajectory.json": '{"dt_s": 0.016, "bodies": [{"name": "a", '
                           '"t": [0.0], "xyz": [[0, 0, 0]]}]}',
        "roster.json": '{"t_s": 0.0, "frozen": true, "synchronization": true,'
                       ' "bodies": []}',
        "contacts.json": '{"supported": true, "steps": 10, "pairs": [],'
                         ' "total_observed": 0, "distinct_named": 0}',
        "completion.json": '{"complete": true, "quit_called": true,'
                           ' "recorded_s": 10.0, "steps": 625, "dt_s": 0.016}',
        "process.json": '{"exit_code": 0, "timed_out": false}',
        "stdout.log": "INFO: bringup_drive: Starting controller: python3\n",
        "stderr.log": "",
        "perf.txt": "TOT\n",
    }
    for name, text in produced.items():
        (tmp_path / name).write_text(text, encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.missing == {}, run.missing
    assert run.errors == {}, run.errors
    assert run.trajectory and run.roster and run.contacts
    assert run.completion and run.process
    assert run.console_text is not None
    assert "Starting controller" in run.console_text
    assert run.perf_nonempty is True


def test_launch_on_an_unreadable_world_reports_not_raises(tmp_path):
    """Adapter rule 1 holds for the launcher too: no exception, a
    process.json recording what went wrong."""
    run_dir, facts = launcher.launch(tmp_path / "missing.wbt",
                                     tmp_path / "run")
    assert facts["exit_code"] is None
    assert "error" in facts
    assert (Path(run_dir) / "process.json").is_file()
    got = json.loads((Path(run_dir) / "process.json")
                     .read_text(encoding="utf-8"))
    assert "world unreadable" in got["error"]


def test_the_stanza_defaults_match_the_omnisim_recorders_shape():
    """Same Robot name as the OmniSim arm's stanza, so a cross-arm reader
    sees ONE grader identity in both worlds."""
    s = launcher.recorder_stanza("/out", duration=10.0)
    assert 'name "agentbench_recorder"' in s
    assert "supervisor TRUE" in s
