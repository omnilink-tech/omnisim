#!/usr/bin/env python3
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

"""Grade-time placement INSIDE the cell -- the wiring, not the mechanism.

``common/test_r1_placement.py`` pins what placement does to a world.
This file pins the three things the CAMPAIGN depends on, on every arm:

    1. the artifact the grader is handed carries the DRAWN layout, not the
       published one (``test_the_world_handed_to_the_grader_...``);
    2. the grader RESOLVES the declaration -- the sidecar reaches it through
       whichever channel that arm's grading path uses
       (``test_the_sidecar_reaches_...``);
    3. a placement that fails BLOCKS the cell instead of grading it against
       the layout the agent memorised (``test_a_placement_failure_blocks_...``).

Every test here drives ``run_cc_cell.run_cell``'s real sequence with a STUBBED
agent session: no Claude Code, no model quota, no simulator. What is being
tested is where in that sequence placement sits and what it hands on, which is
exactly the part a stub cannot fake.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.cc_lane import run_cc_cell as cell  # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging  # noqa: E402
from agentbench.common import mjcftext, r1_placement, worldtext  # noqa: E402
from agentbench.graders import r1 as r1_grader  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

AGENTBENCH = Path(__file__).resolve().parents[1]
FIXTURE = {
    "omnisim": (AGENTBENCH / "adapters" / "omnisim" / "omnisim_lane"
                / "worlds" / "r1_null.wbt"),
    "webots": (AGENTBENCH / "adapters" / "omnisim" / "omnisim_lane"
               / "worlds" / "r1_null.wbt"),
    "mujoco": (AGENTBENCH / "adapters" / "mujoco" / "mujoco_lane"
               / "r1_oracle.xml"),
}
SCANNER = {"omnisim": worldtext, "webots": worldtext, "mujoco": mjcftext}

TASK = "R1_lidar_nav"


def _cc_result():
    return {"type": "result", "subtype": "success", "is_error": False,
            "num_turns": 2, "duration_ms": 900, "duration_api_ms": 800,
            "result": "built it", "session_id": "sess-r1",
            "total_cost_usd": 0.1,
            "usage": {"input_tokens": 1, "output_tokens": 1,
                      "cache_read_input_tokens": 0,
                      "cache_creation_input_tokens": 0},
            "modelUsage": {"model-x": {"outputTokens": 1}}}


def _row_stub():
    return {"task": TASK, "sim": "omnisim", "condition": "claude_code",
            "outcome": "PASS", "assertions": {"R1.1": True},
            "failed_assertion": None, "measurements": {}, "agent": {},
            "agent_artifacts": {}, "metrics": {"t_total_s": 1.0}}


def _harness(tmp_path, monkeypatch, sim, *, world_text=None, seen=None):
    """Stub everything the cell needs except the part under test.

    The "session" writes a deliverable into the workspace exactly where
    ``discover_artifact`` looks for one, so step 5 collects it for real and
    placement operates on the collected copy for real.
    """
    seen = seen if seen is not None else {}
    # The unpatched functions, kept so a test can exercise the REAL one after
    # the cell has run with the stub.
    seen["real"] = {"grade_omnisim": cell.grade_omnisim}
    src = FIXTURE[sim]
    text = world_text if world_text is not None else src.read_text(
        encoding="utf-8")
    name = cell.artifact_name(TASK, sim)

    def fake_template(root):
        tpl = Path(root) / "templates" / sim
        tpl.mkdir(parents=True, exist_ok=True)
        return tpl

    def fake_instantiate(tpl, ws):
        Path(ws).mkdir(parents=True, exist_ok=True)
        return {"filelist_sha256": "synth", "known_disclosures": [],
                "redactions": []}

    def fake_stage_task(ws, task_id, sim_):
        return "build a lidar rover", []

    def fake_session(prompt, ws, env, run_dir, *, model, timeout_s, **kw):
        seen["agent_env"] = dict(env)
        # A real session with the containment guard installed leaves a
        # guard-log line per tool call; a double that skips it is the "hook
        # never ran" shape the guard gate exists to refuse.
        if kw.get("settings_path"):
            (Path(kw["settings_path"]).parent / "guard_events.jsonl").write_text(
                json.dumps({"tool": "Read", "allow": True}) + "\n",
                encoding="utf-8")
        seen["workspace"] = Path(ws)
        world = Path(ws) / "worlds" / name
        world.parent.mkdir(parents=True, exist_ok=True)
        world.write_text(text, encoding="utf-8")
        if sim == "mujoco":            # the model's driver travels with it
            world.with_suffix(".py").write_text("# driver\n",
                                                encoding="utf-8")
        # ``discover_artifact`` gates on "written after the session started".
        # A real session takes minutes; this one takes microseconds, and on a
        # coarse clock the write can land on the same tick as the start stamp.
        # Stamped forward so the stub is never the flaky part.
        stamp = time.time() + 5.0
        for p in (world, world.with_suffix(".py")):
            if p.is_file():
                os.utime(p, (stamp, stamp))
        return _cc_result(), {"permission_mode": "dangerously-skip-permissions",
                              "cli_command": "claude", "rc": 0,
                              "timed_out": False, "wall_s": 1.0,
                              "launch_error": None}

    def fake_grade_omnisim(task_id, artifact, grade_dir, notes, *,
                           answer_path=None, layout_dir=None):
        seen["graded_artifact"] = Path(artifact)
        seen["layout_dir"] = layout_dir
        seen["grade_dir"] = Path(grade_dir)
        return _row_stub()

    def fake_grade_webots(task_id, artifact, run_dir, answer,
                          project_assets=()):
        seen["graded_artifact"] = Path(artifact)
        seen["grader_run_dir"] = Path(run_dir)
        # the published task assets that must travel with the project into
        # the WSL workdir (a controller that reads one at run time otherwise
        # dies at startup -- R4's first cell, 0.00 m over 150 s)
        seen["project_assets"] = list(project_assets)
        return _row_stub()

    def fake_grade_mujoco(task_id, artifact, run_dir, answer):
        seen["graded_artifact"] = Path(artifact)
        seen["grader_run_dir"] = Path(run_dir)
        return _row_stub()

    monkeypatch.setattr(cell, "preflight", lambda run_dir, env: {
        "version": "test", "default_model": "model-x", "ok": True,
        "detail": "OK"})
    monkeypatch.setattr(cell.staging, "build_omnisim_template", fake_template)
    monkeypatch.setattr(cell.staging, "build_webots_template", fake_template)
    monkeypatch.setattr(cell.staging, "build_mujoco_template", fake_template)
    monkeypatch.setattr(cell.staging, "instantiate", fake_instantiate)
    monkeypatch.setattr(cell.staging, "stage_task", fake_stage_task)
    monkeypatch.setattr(cell.staging, "sweep_workspace_processes",
                        lambda ws, **kw: [])
    monkeypatch.setattr(cell.staging, "sweep_repo_junction_artifacts",
                        lambda dest, **kw: [])
    monkeypatch.setattr(cell.staging, "teardown_workspace_resilient",
                        lambda ws, **kw: {"ok": True})
    monkeypatch.setattr(cell, "_sweep_ports",
                        lambda report, when, **kw: None)
    monkeypatch.setattr(cell, "ensure_virtual_display",
                        lambda *a, **kw: {"ok": True, "display": ":99",
                                          "state": "started", "detail": ""})
    monkeypatch.setattr(cell, "run_claude_cell", fake_session)
    monkeypatch.setattr(cell, "find_transcript", lambda sid: None)
    monkeypatch.setattr(cell, "grade_omnisim", fake_grade_omnisim)
    monkeypatch.setattr(cell, "grade_webots", fake_grade_webots)
    monkeypatch.setattr(cell, "grade_mujoco", fake_grade_mujoco)
    monkeypatch.delenv("AGENTBENCH_R1_LAYOUT_SEED", raising=False)
    return seen


def _run(tmp_path, monkeypatch, sim, **kw):
    seen = _harness(tmp_path, monkeypatch, sim, **kw)
    row = cell.run_cell(sim, TASK, root=tmp_path / "root",
                        out_dir=tmp_path / "out", use_locks=False, repeat=0,
                        layout_seed="cellspec/r0")
    return row, seen, tmp_path / "out"


# --- 1. the graded world is not the published one ---------------------------

@pytest.mark.parametrize("sim", ["omnisim", "webots", "mujoco"])
def test_the_world_handed_to_the_grader_carries_the_drawn_layout(
        tmp_path, monkeypatch, sim):
    """⭐ The defect this whole change closes, checked per arm.

    Before it, ``run_cell`` collected the agent's deliverable and handed it to
    the grader untouched -- so the graded world's obstacles sat at the
    positions ``benchmark_assets/obstacles.json`` publishes, and a controller
    that read that file and cast no beam passed 6/6 (measured).

    The assertion is on the FILE THE GRADER WAS GIVEN, not on the placer's own
    report: whatever else happened, the poses in that file must be the drawn
    ones and must differ from the published ones.
    """
    row, seen, out = _run(tmp_path, monkeypatch, sim)
    graded = seen["graded_artifact"]
    assert graded.is_file(), graded

    bodies = SCANNER[sim].scan_bodies(graded.read_text(encoding="utf-8"))
    drawn = r1_core.sample_layout("cellspec/r0")
    found, missing = r1_core.match_spec_obstacles(bodies, drawn)
    assert missing == [] and len(found) == r1_core.N_OBSTACLES, (
        "the graded artifact is not the drawn layout: %s" % missing)
    # ...and it is NOT the published one. Both halves, because a placement
    # that moved nothing would satisfy neither and a placement that moved
    # things at random would satisfy only the second.
    _pub_found, pub_missing = r1_core.match_spec_obstacles(
        bodies, r1_core.obstacle_spec())
    assert pub_missing, (
        "every published obstacle is still at its published pose in the "
        "graded artifact -- the memorising agent's best case")

    place = row["agent_artifacts"]["r1_placement"]
    assert place["ok"] is True
    assert place["seed"] == "cellspec/r0"
    assert place["mechanism"] == r1_core.PLACEMENT_MECHANISM
    assert len(place["obstacles"]) == r1_core.N_OBSTACLES


@pytest.mark.parametrize("sim", ["omnisim", "mujoco"])
def test_the_agents_own_copy_is_never_touched(tmp_path, monkeypatch, sim):
    """The deliverable in the WORKSPACE keeps the published layout.

    Placement operates on the collected copy under the results tree. If it
    reached back into the workspace, the layout would exist in a directory the
    session owned -- and the whole "the agent cannot see it" argument would
    rest on the session having already exited rather than on the layout being
    somewhere else.
    """
    _row, seen, _out = _run(tmp_path, monkeypatch, sim)
    ws_copy = seen["workspace"] / "worlds" / cell.artifact_name(TASK, sim)
    bodies = SCANNER[sim].scan_bodies(ws_copy.read_text(encoding="utf-8"))
    _found, missing = r1_core.match_spec_obstacles(bodies,
                                                   r1_core.obstacle_spec())
    assert missing == [], (
        "the agent's own file was rewritten; placement must only touch the "
        "collected copy")


def test_the_drawn_layout_never_enters_the_agents_environment(
        tmp_path, monkeypatch):
    """The seed is not handed to the session, in any variable.

    ``SCRUB_PREFIXES`` drops every ``AGENTBENCH_*`` name from the child env,
    which is what makes ``$AGENTBENCH_R1_LAYOUT_SEED`` safe to support at all.
    Pinned here rather than assumed, because the day someone spells the
    variable differently the scrub stops covering it.
    """
    monkeypatch.setenv("AGENTBENCH_R1_LAYOUT_SEED", "operator/seed")
    seen = _harness(tmp_path, monkeypatch, "omnisim")
    monkeypatch.setenv("AGENTBENCH_R1_LAYOUT_SEED", "operator/seed")
    cell.run_cell("omnisim", TASK, root=tmp_path / "root",
                  out_dir=tmp_path / "out", use_locks=False, repeat=0)
    env = seen["agent_env"]
    assert not [k for k in env if k.startswith("AGENTBENCH_")], sorted(env)
    assert "operator/seed" not in json.dumps(env)
    # ...and the operator's variable WAS the seed that was used, so this is a
    # test of the scrub rather than of a variable nobody reads.
    report = json.loads((tmp_path / "out" / "cell_report.json")
                        .read_text(encoding="utf-8"))
    assert report["layout_seed"] == "operator/seed"
    assert report["layout_seed_source"] == "$AGENTBENCH_R1_LAYOUT_SEED"


# --- 2. the declaration reaches the grader ----------------------------------

@pytest.mark.parametrize("sim", ["webots", "mujoco"])
def test_the_sidecar_reaches_the_in_process_graders(tmp_path, monkeypatch,
                                                    sim):
    """These two arms grade in-process with ``run_dir = <cell>/grade``, which
    is one of the directories ``r1_core.resolve_graded_layout`` searches. The
    check is the grader's own resolution call, not the file's existence."""
    _row, seen, _out = _run(tmp_path, monkeypatch, sim)
    run_dir = seen["grader_run_dir"]
    layout, source = r1_core.resolve_graded_layout(
        directories=(run_dir, run_dir / "phaseB", run_dir / "worlds"))
    assert layout == r1_core.sample_layout("cellspec/r0")
    assert r1_core.LAYOUT_SIDECAR in source


def test_the_sidecar_reaches_the_omnisim_subprocess_grader(tmp_path,
                                                           monkeypatch):
    """The OmniSim arm grades in a ``run_agentbench.py`` SUBPROCESS whose
    per-cell run dir is created (and any existing one deleted) after placement
    has finished, so no directory the placer could write to is on the grader's
    search path. The directory is named in the child's environment instead --
    and this asserts the whole chain: the cell passes it, ``grade_omnisim``
    puts it in the child env, and ``graders/r1`` resolves the sidecar from it.
    """
    row, seen, out = _run(tmp_path, monkeypatch, "omnisim")
    assert seen["layout_dir"] == out, (
        "the cell did not tell grade_omnisim where it declared the layout")

    # ...the real grade_omnisim builds the child env (subprocess stubbed out).
    captured = {}

    class _Proc:
        returncode = 0
        stdout = ""
        stderr = ""

    def fake_run(cmd, env=None, **kw):
        captured["env"] = env
        return _Proc()

    monkeypatch.setattr(cell.subprocess, "run", fake_run)
    with pytest.raises(RuntimeError):     # no rows.jsonl: the stub ran nothing
        seen["real"]["grade_omnisim"](
            TASK, out / "artifact" / "lidar_nav.wbt", tmp_path / "gd",
            "notes", layout_dir=out)
    assert captured["env"][r1_core.LAYOUT_DIR_ENV] == str(out)

    # ...and that is a channel graders/r1 actually reads.
    monkeypatch.setenv(r1_core.LAYOUT_DIR_ENV, str(out))
    got = {}

    class _StubAdapter:
        @staticmethod
        def build_bundle(*a, **kw):
            return object()

    monkeypatch.setattr(r1_grader.adapters, "resolve",
                        lambda sim: _StubAdapter)
    monkeypatch.setattr(r1_grader.r1_core, "grade",
                        lambda bundle, **kw: got.update(kw))
    r1_grader.grade(tmp_path / "elsewhere", sim="omnisim")
    assert got["graded_layout"] == r1_core.sample_layout("cellspec/r0")
    assert r1_core.LAYOUT_SIDECAR in got["layout_source"]


def test_without_the_sidecar_the_grader_says_the_row_proves_nothing(tmp_path):
    """The negative control for the handshake.

    If the declaration does not reach the grader, R1 scores the PUBLISHED
    layout and stamps the verdict with a note saying so. That behaviour is
    what makes a missing sidecar detectable rather than silent -- and it is
    why placement failing must block the cell rather than fall through to it.
    """
    layout, source = r1_core.resolve_graded_layout(
        directories=(tmp_path, tmp_path / "phaseB"))
    assert layout is None and source is None


# --- 3. failure BLOCKS ------------------------------------------------------

def test_a_placement_failure_blocks_the_cell_instead_of_grading_it(
        tmp_path, monkeypatch):
    """⭐ No fallback, at the level that matters: the CELL.

    The deliverable here has four of the five obstacles, so the drawn layout
    cannot be placed into it. The cell must refuse to grade: no row, a blocker
    naming the reason, and -- the part that would be invisible otherwise -- no
    graded artifact handed to any grader.

    Grading it anyway would score the published layout while producing a row
    that is indistinguishable from a placed one. The cost of blocking is that
    an agent which built four boxes gets a BLOCKED cell rather than a FAIL
    row; that is deliberate, and it is the safe direction: a blocker is
    visible and re-runnable, a silently-published layout is neither.
    """
    text = FIXTURE["mujoco"].read_text(encoding="utf-8")
    four = "\n".join(ln for ln in text.splitlines()
                     if 'name="OBSTACLE_5"' not in ln)
    seen = _harness(tmp_path, monkeypatch, "mujoco", world_text=four)

    with pytest.raises(SystemExit) as exc:
        cell.run_cell("mujoco", TASK, root=tmp_path / "root",
                      out_dir=tmp_path / "out", use_locks=False, repeat=0,
                      layout_seed="cellspec/r0")
    assert "placement failed" in str(exc.value)
    assert "OBSTACLE_5" in str(exc.value)
    assert "graded_artifact" not in seen, "the cell graded an unplaced world"

    out = tmp_path / "out"
    assert not (out / "rows.jsonl").exists(), (
        "a blocked cell must land NO row: a row here would score the "
        "published layout")
    report = json.loads((out / "cell_report.json").read_text(encoding="utf-8"))
    assert report["r1_placement"]["ok"] is False
    assert report["r1_placement"]["missing"] == ["OBSTACLE_5"]
    assert "PUBLISHED" in report["blocker"]
    # ...and no declaration was left behind claiming a layout nobody placed.
    assert not (out / r1_core.LAYOUT_SIDECAR).exists()


def test_a_task_without_a_drawn_layout_is_untouched(tmp_path, monkeypatch):
    """Placement is R1's. Another task's cell records ``null`` rather than
    nothing, so a reader can tell "no layout applies" from "the mechanism
    silently did not run"."""
    seen = _harness(tmp_path, monkeypatch, "omnisim")

    def fake_stage_task(ws, task_id, sim_):
        return "fix the world", []

    monkeypatch.setattr(cell.staging, "stage_task", fake_stage_task)
    row = cell.run_cell("omnisim", "C2_fall_through_floor",
                        root=tmp_path / "root", out_dir=tmp_path / "out",
                        use_locks=False, repeat=0)
    assert row["agent_artifacts"]["r1_placement"] is None
    assert row["agent_artifacts"]["layout_seed"] is None
    assert seen["layout_dir"] is None


# --- provenance -------------------------------------------------------------

def test_the_row_records_a_seed_the_layout_can_be_re_derived_from(
        tmp_path, monkeypatch):
    """A row that says "the obstacles were moved" and cannot say WHERE is a
    claim. Re-derived here from the row alone, with the pure sampler."""
    row, seen, _out = _run(tmp_path, monkeypatch, "mujoco")
    art = row["agent_artifacts"]
    assert art["layout_seed"] == "cellspec/r0"
    assert art["r1_placement"]["seed_source"].startswith("argument")
    layout = r1_core.sample_layout(art["layout_seed"])
    assert [o["position"][:2] for o in layout] == \
        [o["to"] for o in art["r1_placement"]["obstacles"]]
    bodies = mjcftext.scan_bodies(
        seen["graded_artifact"].read_text(encoding="utf-8"))
    assert r1_core.match_spec_obstacles(bodies, layout)[1] == []


def test_the_default_seed_is_used_and_recorded_when_none_is_passed(
        tmp_path, monkeypatch):
    seen = _harness(tmp_path, monkeypatch, "mujoco")
    row = cell.run_cell("mujoco", TASK, root=tmp_path / "root",
                        out_dir=tmp_path / "out", use_locks=False, repeat=3)
    assert row["agent_artifacts"]["layout_seed"] == \
        r1_placement.default_seed(TASK, 3)
    assert "default" in row["agent_artifacts"]["r1_placement"]["seed_source"]
    assert seen["graded_artifact"].is_file()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
