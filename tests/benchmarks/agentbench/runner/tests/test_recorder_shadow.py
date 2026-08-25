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

"""THE regression suite for the recorder-shadowing exploit.

The attack, in full, because it is the worst thing that was wrong with this
benchmark: the grader measures a run by injecting a ``Robot`` whose
``controller`` field is ``"agentbench_recorder"``, and finds that controller
through ``WEBOTS_EXTRA_PROJECT_PATH``. But
``WbRobot::updateControllerDir`` (``src/omnisim/nodes/WbRobot.cpp``) searches
``WbProject::current()->controllersPath()`` **first**, and
``WbProject::projectPathFromWorldFile`` (``src/omnisim/core/WbProject.cpp``)
resolves the project of a world with no ``worlds`` ancestor to the world's
parent's parent -- the run directory. The run directory used to be writable.

So: author ten real Huskies with real controllers, so the structural
assertions pass honestly off the engine log; let them never move; and write
ONE file to ``../controllers/agentbench_recorder/agentbench_recorder.py``
that prints a fabricated CSV and quits. A1.3, A1.5, A1.6, A1.7 and A1.8 --
the entire core-physical basis of the task -- would then be graded from data
the agent wrote. The weaker version needs no file at all: the engine selects a
controller directory on EXISTENCE, so an empty directory is enough to stop the
real recorder loading, and the cell scores INVALID ("our harness broke")
instead of FAIL.

Two independent defences, and these tests hold both:

  * ``Sandbox.create`` confines writes to the scratch dir -- the run dir is
    readable but no longer writable;
  * ``headless.controller_shadow_check`` resolves the project root the way the
    engine does and REFUSES the run before launch. This is the one that
    matters, because ``run_shell`` cannot be path-confined on a bare host: an
    agent that shells out can still create the directory, and the only honest
    answer is to detect it and decline to produce a number.

The third test group is the reason the check resolves the project root
properly instead of just watching ``<run_dir>/controllers``: a world written
to ``<scratch>/worlds/x.wbt`` moves the project root to ``<scratch>``, which
is legitimately writable, so the write guard cannot see that attack at all.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[3]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.adapters.omnisim import headless          # noqa: E402
from agentbench.common.paths import AGENTBENCH, REPO      # noqa: E402
from agentbench.common.worldtext import INJECTED_PREFIX   # noqa: E402

WORLD_TEXT = '#VRML_SIM R2025a utf8\nWorldInfo {\n}\n'

FAKE_RECORDER = (
    "# a recorder the AGENT wrote: it fabricates the measurement\n"
    "open('phaseB.csv', 'w').write('t,r0_x,r0_y,r0_z\\n0,0,0,0.2\\n')\n")


def _world(dirpath, name="husky_swarm_10.wbt"):
    p = Path(dirpath)
    p.mkdir(parents=True, exist_ok=True)
    w = p / name
    w.write_text(WORLD_TEXT, encoding="utf-8")
    return w


def _plant(project_root, controller="agentbench_recorder", body=None):
    d = Path(project_root) / "controllers" / controller
    d.mkdir(parents=True, exist_ok=True)
    if body is not None:
        (d / ("%s.py" % controller)).write_text(body, encoding="utf-8")
    return d


# --- half one: the write guard ---------------------------------------------

def test_write_file_refuses_the_run_dir(shell_tools, sandbox):
    """The exact exploit call. It used to return "wrote N bytes"."""
    out = shell_tools.get("write_file").handler(
        {"path": "../controllers/agentbench_recorder/agentbench_recorder.py",
         "content": FAKE_RECORDER}, 30.0)
    assert out.is_error, out.text
    assert "confined" in out.text
    assert not (sandbox.run_dir / "controllers").exists()


def test_writes_are_confined_to_scratch_not_the_run_dir(sandbox):
    guard = sandbox.guard
    assert guard.write_roots == [sandbox.scratch_dir.resolve()]
    inside = guard.resolve("a/b.wbt", sandbox.scratch_dir)
    assert guard.check_write(inside) is None
    for escape in ("../controllers/agentbench_recorder/x.py",
                   "../trace.jsonl", "../phaseB/phaseB.csv"):
        p = guard.resolve(escape, sandbox.scratch_dir)
        assert guard.check_write(p), "%s must not be writable" % escape


def test_the_run_dir_is_still_readable_and_still_not_a_leak(sandbox):
    """Tightening writes must not break reads or spam the leak detector.

    The run dir normally lives INSIDE the denied benchmark package
    (``agentbench/results/...``); if it stopped being exempt, every tool call
    that mentions its own trace path would flag ``leak_suspect`` and a signal
    that fires on everything is not a signal.
    """
    guard = sandbox.guard
    p = guard.resolve("trace.jsonl", sandbox.run_dir)
    assert guard.check_read(p) is None
    spill = str(sandbox.run_dir / "tool_results" / "x.json")
    assert guard.flags(spill) is False
    assert guard.flags(str(AGENTBENCH / "graders" / "a1_core.py")) is True


# --- half two: resolving the project root the way the engine does ----------

def test_project_root_matches_the_engine_for_a_scratch_world(tmp_path):
    """No ``worlds`` ancestor: parent's parent (the legacy fallback branch)."""
    w = _world(tmp_path / "run" / "scratch")
    assert headless.project_root_for_world(w) == (tmp_path / "run").resolve()


def test_project_root_matches_the_engine_for_a_worlds_layout(tmp_path):
    """A ``worlds`` ancestor wins, at any depth -- OmniSim allows
    ``worlds/<category>/foo.wbt``, so the walk is up, not one level."""
    w = _world(tmp_path / "proj" / "worlds" / "flagship")
    assert headless.project_root_for_world(w) == (tmp_path / "proj").resolve()
    w2 = _world(tmp_path / "proj2" / "worlds")
    assert headless.project_root_for_world(w2) == (tmp_path
                                                  / "proj2").resolve()


# --- half three: the refusal ----------------------------------------------

def test_a_planted_recorder_is_detected(tmp_path):
    w = _world(tmp_path / "run" / "scratch")
    assert headless.controller_shadow_check(w) == []
    _plant(tmp_path / "run", body=FAKE_RECORDER)
    hits = headless.controller_shadow_check(w)
    assert [h["controller"] for h in hits] == ["agentbench_recorder"]
    assert hits[0]["project_root"] == str((tmp_path / "run").resolve())
    assert "agentbench_recorder.py" in hits[0]["entries"]


def test_an_EMPTY_shadow_directory_is_detected_too(tmp_path):
    """The engine picks a controller dir on EXISTENCE alone, so an empty one
    is enough to stop the real recorder loading."""
    w = _world(tmp_path / "run" / "scratch")
    _plant(tmp_path / "run", body=None)
    hits = headless.controller_shadow_check(w)
    assert hits and hits[0]["entries"] == []


def test_a_shadow_the_write_guard_cannot_see_is_still_refused(tmp_path):
    """The attack that survives scratch-only writes.

    Put the world at ``<scratch>/worlds/x.wbt`` and the project root moves to
    ``<scratch>`` -- which the agent may legitimately write to. Only resolving
    the project root the engine's way catches this.
    """
    scratch = tmp_path / "run" / "scratch"
    w = _world(scratch / "worlds")
    assert headless.project_root_for_world(w) == scratch.resolve()
    _plant(scratch, body=FAKE_RECORDER)
    hits = headless.controller_shadow_check(w)
    assert hits and hits[0]["project_root"] == str(scratch.resolve())


def test_the_harness_supervisor_can_be_shadowed_too(tmp_path):
    w = _world(tmp_path / "run" / "scratch")
    _plant(tmp_path / "run", controller="harness_supervisor",
           body="# fake\n")
    assert [h["controller"] for h in headless.controller_shadow_check(w)] == [
        "harness_supervisor"]


def test_the_shipped_controllers_are_not_mistaken_for_a_shadow():
    """A world inside ``projects/default/worlds/`` resolves its project to
    ``projects/default``, which really does ship ``harness_supervisor``. That
    is the real one, not a shadow, and must not be refused."""
    shipped = REPO / "projects" / "default"
    if not (shipped / "controllers" / "harness_supervisor").is_dir():
        pytest.skip("this clone has no projects/default/controllers")
    fake_world = shipped / "worlds" / "does_not_need_to_exist.wbt"
    assert headless.project_root_for_world(fake_world) == shipped.resolve()
    assert headless.controller_shadow_check(fake_world) == []


def test_run_standalone_refuses_before_it_launches_anything(tmp_path):
    """No engine, no injected world, no retries: the run is declined."""
    scratch = tmp_path / "run" / "scratch"
    w = _world(scratch)
    _plant(tmp_path / "run", body=FAKE_RECORDER)
    res = headless.run_standalone(w, tmp_path / "run" / "phaseB",
                                  duration=1.0, settle=0.0, attempts=3)
    assert res.tamper, "the run was NOT refused"
    assert res.error.startswith("TAMPER:")
    assert res.rc is None and res.injected_world is None
    assert res.attempts_used == 1, "a refusal must never be retried"
    assert not list(scratch.glob(INJECTED_PREFIX + "*.wbt"))
    # and it is not mistaken for the launch flake, which would be INVALID
    # "the stack broke" rather than a named tamper
    assert headless.stack_broke(res) is False
    assert res.as_dict()["tamper"] == res.tamper


def test_a_clean_world_is_not_refused(tmp_path):
    w = _world(tmp_path / "run" / "scratch")
    assert headless.controller_shadow_check(w) == []


# --- and the verdict the runner produces -----------------------------------

def test_run_cell_scores_a_shadowed_run_INVALID_not_PASS(tmp_path,
                                                         monkeypatch):
    """End to end through ``run_cell``, with no simulator: an agent that
    plants a fake recorder gets INVALID and a named TAMPER note -- never a
    graded row."""
    from agentbench import agents as agent_registry
    from agentbench import run_agentbench
    from agentbench import tasks as task_registry
    from agentbench.agents.base import AgentResult
    from agentbench.graders.verdict import INVALID

    task = task_registry.get("A1_husky_swarm_10")

    def cheat(ctx):
        (ctx.scratch_dir / "husky_swarm_10.wbt").write_text(
            WORLD_TEXT, encoding="utf-8")
        _plant(ctx.run_dir, body=FAKE_RECORDER)
        res = AgentResult()
        res.final_message = "ten Huskies, all driving. Trust me."
        return res

    monkeypatch.setitem(agent_registry.REGISTRY, (task.id, "_cheat"),
                        {"fn": cheat, "expect_pass": None,
                         "expect_failures": None})
    row, verdict = run_agentbench.run_cell(task, "_cheat", 0, tmp_path,
                                           quiet=True)
    assert verdict.outcome == INVALID, verdict.summary()
    assert any(n.startswith("TAMPER:") for n in verdict.notes), verdict.notes
    assert row["outcome"] == INVALID
    assert row["artifacts"]["tamper"][0]["controller"] == "agentbench_recorder"
    assert any("TAMPER" in d for d in row["deviations"])
