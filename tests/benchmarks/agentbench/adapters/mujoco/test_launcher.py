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

"""Unit tests for the MuJoCo-lane launcher. **No MuJoCo, no child process.**

Everything in :mod:`agentbench.adapters.mujoco.launcher` that decides something
-- which program is the deliverable's driver, what the child's argv looks like,
what ``process.json`` says -- is a pure function, and those are what these tests
pin. The live path is exercised by ``test_mujoco_end_to_end.py`` (which skips
when ``mujoco`` is not importable) and documented in ``BRINGUP.md``.

    pytest tests/benchmarks/agentbench/adapters/mujoco -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.mujoco import launcher, recording  # noqa: E402


# --- driver discovery --------------------------------------------------------


def test_driver_is_the_model_stem_sibling(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / "scene.py").write_text("pass", encoding="utf-8")
    (tmp_path / "notes.py").write_text("pass", encoding="utf-8")
    drv, rule = launcher.find_driver(tmp_path / "scene.xml")
    assert drv == tmp_path / "scene.py"
    assert "stem" in rule


def test_driver_falls_back_to_the_only_python_file(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / "drive_it.py").write_text("pass", encoding="utf-8")
    drv, rule = launcher.find_driver(tmp_path / "scene.xml")
    assert drv == tmp_path / "drive_it.py"
    assert "only .py" in rule


def test_several_candidates_are_refused_rather_than_guessed(tmp_path):
    """Picking one would be the GRADER choosing which program the agent meant.

    A wrong pick does not read as a grader error -- it reads as an agent whose
    robot did nothing, which is the most expensive kind of wrong answer this
    suite can produce.
    """
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    for name in ("a.py", "b.py", "c.py"):
        (tmp_path / name).write_text("pass", encoding="utf-8")
    drv, rule = launcher.find_driver(tmp_path / "scene.xml")
    assert drv is None
    assert "refusing to guess" in rule


def test_no_driver_at_all_is_named_as_such(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    drv, rule = launcher.find_driver(tmp_path / "scene.xml")
    assert drv is None
    assert "does not move on its own" in rule


def test_explicit_driver_wins_and_a_missing_one_is_reported(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / "scene.py").write_text("pass", encoding="utf-8")
    other = tmp_path / "other.py"
    other.write_text("pass", encoding="utf-8")
    assert launcher.find_driver(tmp_path / "scene.xml", other)[0] == other
    drv, rule = launcher.find_driver(tmp_path / "scene.xml",
                                     tmp_path / "gone.py")
    assert drv is None and "does not exist" in rule


def test_dunder_init_is_never_the_driver(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / "__init__.py").write_text("", encoding="utf-8")
    assert launcher.find_driver(tmp_path / "scene.xml")[0] is None


# --- the child's command -----------------------------------------------------


def test_runner_command_shape():
    cmd = launcher.runner_command("py", "m.xml", "m.py", "/out",
                                  duration=12.0, wall_limit=300.0)
    assert cmd[0] == "py"
    assert cmd[1] == str(launcher.RUNNER)
    assert "--model" in cmd and "m.xml" in cmd
    assert "--driver" in cmd and "m.py" in cmd
    assert "--out" in cmd and "/out" in cmd
    assert cmd[cmd.index("--duration") + 1] == repr(12.0)
    assert cmd[cmd.index("--wall-limit") + 1] == repr(300.0)


def test_runner_command_omits_a_default_stride_and_an_absent_driver():
    cmd = launcher.runner_command("py", "m.xml", None, "/out",
                                  contact_stride=1)
    assert "--contact-stride" not in cmd
    assert "--driver" not in cmd


def test_runner_command_carries_a_non_default_stride_and_caps():
    cmd = launcher.runner_command("py", "m.xml", "m.py", "/out",
                                  contact_stride=5, body_cap=8, pair_cap=9)
    assert cmd[cmd.index("--contact-stride") + 1] == "5"
    assert cmd[cmd.index("--body-cap") + 1] == "8"
    assert cmd[cmd.index("--pair-cap") + 1] == "9"


def test_runner_script_exists_where_the_command_points():
    assert launcher.RUNNER.is_file()


def test_child_env_does_not_pin_a_gl_backend():
    """Forcing MUJOCO_GL would change what the agent's program does.

    A driver that opens a viewer fails on a headless box, and that is the
    honest outcome of a benchmark that is headless for every arm.
    """
    env = launcher.child_env(base={"PATH": "/usr/bin"})
    assert "MUJOCO_GL" not in env
    assert env["PYTHONUNBUFFERED"] == "1"


# --- process facts -----------------------------------------------------------


def test_process_facts_are_the_shape_recording_reads(tmp_path):
    facts = launcher.process_facts(
        rc=0, timed_out=False, wall_s=1.25, attempts_used=1,
        command=["py", "runner"], python="py", model="m.xml", driver="m.py",
        driver_rule="explicitly supplied",
        probe={"ok": True, "mujoco_version": "3.8.1",
               "mujoco_python_version": "3.8.1"})
    (tmp_path / "process.json").write_text(json.dumps(facts),
                                           encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.process["exit_code"] == 0
    assert run.process["mujoco_version"] == "3.8.1"
    assert run.driver == "m.py"
    assert run.model == "m.xml"
    assert "process" not in run.missing


def test_launch_reports_a_missing_model_without_raising(tmp_path):
    _rd, facts = launcher.launch(tmp_path / "nope.xml", tmp_path / "run")
    assert "does not exist" in facts["error"]
    assert (tmp_path / "run" / "process.json").is_file()


def test_launch_reports_an_interpreter_without_mujoco_as_a_dependency_gap(
        tmp_path, monkeypatch):
    """A missing library must not read as a simulator that failed to simulate.

    Those are different facts, and only one of them is about MuJoCo.
    """
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    monkeypatch.setattr(launcher, "probe_python",
                        lambda *a, **k: {"ok": False, "python": "py",
                                         "mujoco_version": None,
                                         "error": "No module named 'mujoco'"})
    _rd, facts = launcher.launch(tmp_path / "scene.xml", tmp_path / "run")
    assert "MISSING DEPENDENCY" in facts["error"]
    assert facts["interpreter_probe"]["ok"] is False


def test_resolve_python_prefers_the_environment_override(monkeypatch):
    monkeypatch.setenv(launcher.PYTHON_ENV, "/opt/py/bin/python")
    assert launcher.resolve_python() == "/opt/py/bin/python"
    assert launcher.resolve_python("/other") == "/other"
