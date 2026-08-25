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

"""Grade-time placement, measured on REAL runs through the campaign's own path.

``adapters/mujoco/test_r1_discriminates_mujoco.py`` proves the mechanism works
when a test fixture places the layout itself. This file proves the WIRING
works: the deliverable is staged the way ``cc_lane`` stages one,
``common/r1_placement.place_and_declare`` is the only thing that touches it,
and ``cc_lane.run_cc_cell.grade_mujoco`` -- the function a campaign cell calls
-- launches and grades it. Nothing here passes a layout to the grader by hand;
the grader finds the declaration for itself, exactly as it will in a campaign.

Two drivers, same scene, same pipeline, one variable:

    r1_oracle.py     senses with ``mj_ray``, reads no layout   must PASS
    r1_hardcode.py   plans from benchmark_assets/obstacles.json, casts
                     zero beams                                 must be CAUGHT

The second is the memoriser this mechanism exists for -- and on the PUBLISHED
layout it passes 6/6, which is measured in the sibling file. The first is the
control that keeps the mechanism honest: a filter that also breaks honest
agents is not a filter, so the oracle failing here would mean the placement is
broken, not the oracle.

Campaign-wide rates measured through THIS path are recorded in
``tasks/R1_lidar_nav/meta.json``; the seeds below pin single instances of each
so a regression that quietly restored the published layout cannot pass the
suite.

**Cost.** Real MuJoCo runs in a child interpreter, ~5-10 s each, no GPU, no
network, no model quota. Skipped, not failed, without ``mujoco``.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.mujoco import launcher  # noqa: E402
from agentbench.common import mjcftext, r1_placement  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

pytest.importorskip("mujoco", reason="the MuJoCo arm needs the mujoco package")

from agentbench.cc_lane import run_cc_cell as cell  # noqa: E402

LANE = launcher.LANE
SCENE = LANE / "r1_oracle.xml"
TASK = "R1_lidar_nav"

#: Seeds pinned in the suite. The campaign-wide measurement over many more is
#: in meta.json; these two are the regression pins.
ORACLE_SEEDS = ["wired/oracle/1"]
MEMORISER_SEEDS = ["wired/memo/1"]


#: The collected driver. The lane fixtures are a SET -- ``r1_hardcode`` is
#: ``r1_oracle``'s planner and control law with the map read from a file
#: instead of from beams, so it imports its sibling and reads the task asset
#: by a path relative to its own location. Copying one file out of the lane
#: therefore breaks it (measured: ``ModuleNotFoundError: r1_oracle``), which
#: is a property of the FIXTURE, not of the pipeline. So the collected driver
#: runs the fixture where it lives, with the argv the runner hands it. Every
#: pipeline behaviour under test -- the pair travelling together,
#: ``find_driver``'s stem rule, the recorder, the grader -- is unchanged.
_SHIM = '''"""Collected driver: run the lane fixture %s under the runner's argv."""
import runpy
import sys

FIXTURE = r"%s"
sys.argv = [FIXTURE] + sys.argv[1:]
runpy.run_path(FIXTURE, run_name="__main__")
'''


def staged_cell(tmp_path, driver, seed):
    """One cell's worth of the real path: collect, PLACE, declare, grade.

    Mirrors ``run_cc_cell.run_cell`` steps 5, 6a and 6 exactly -- the
    deliverable is a model plus its driver under the collected name, placement
    writes the sidecar into the directories the cell writes them into, and
    ``grade_mujoco`` does the rest. The agent's session is the only thing
    missing, and it is replaced by staging a known driver.
    """
    art_dir = Path(tmp_path) / "artifact"
    art_dir.mkdir(parents=True, exist_ok=True)
    model = art_dir / "lidar_nav.xml"
    shutil.copyfile(SCENE, model)
    model.with_suffix(".py").write_text(
        _SHIM % (driver, str(LANE / driver)), encoding="utf-8")

    grade_project = Path(tmp_path) / "grade"
    report = r1_placement.place_and_declare(
        model, seed=seed, declare_dirs=(Path(tmp_path), grade_project))
    row = cell.grade_mujoco(TASK, model, grade_project, answer="")
    return report, row


def _assertions(row):
    return row["assertions"]


# --- the honest navigator still passes --------------------------------------

@pytest.mark.parametrize("seed", ORACLE_SEEDS)
def test_the_sensing_oracle_passes_through_the_wired_path(tmp_path, seed):
    """⭐ The control. If this fails, the PLACEMENT is broken, not the oracle.

    The oracle reads no layout: its map starts empty and is filled by 181
    beams per scan during the run. So a layout it has never seen costs it
    nothing -- provided the world it was placed into is coherent and the
    grader is scoring that same world. Both of those are what this measures.
    """
    report, row = staged_cell(tmp_path, "r1_oracle.py", seed)
    assert report["verification"]["matched"] == r1_core.N_OBSTACLES
    assert row["outcome"] == "PASS", (row["failed_assertion"], row["notes"])
    assert all(_assertions(row).values()), _assertions(row)
    # ...and it was scored against the DRAWN layout, not the published one:
    # the grader resolved the declaration by itself.
    scored = row["measurements"]["r1_graded_layout"]
    assert scored["placed"] is True
    assert r1_core.LAYOUT_SIDECAR in scored["source"]
    drawn = r1_core.sample_layout(seed)
    assert [o["position"][:2] for o in scored["obstacles"]] == \
        [[round(float(c), 4) for c in o["position"][:2]] for o in drawn]


# --- the memoriser is caught ------------------------------------------------

@pytest.mark.parametrize("seed", MEMORISER_SEEDS)
def test_the_memoriser_is_caught_through_the_wired_path(tmp_path, seed):
    """⭐ The mechanism, doing the job it was built for, on a real run.

    Same robot, same control law, same planner as the oracle. The only
    difference is where the map comes from: this one reads
    ``benchmark_assets/obstacles.json`` before the wheels turn and casts no
    beam. On the published layout that passes 6/6 (measured, sibling file).
    Through the wired path it drives its memorised route into a box that
    moved.

    The failure must be BEHAVIOURAL -- R1.1-R1.3 still pass -- or the fixture
    is broken rather than the memoriser caught.
    """
    _report, row = staged_cell(tmp_path, "r1_hardcode.py", seed)
    ok = _assertions(row)
    assert row["outcome"] != "PASS", ok
    assert ok["R1.1"] and ok["R1.2"] and ok["R1.3"], (
        "the run must fail on behaviour, not because the scene or the run "
        "broke: %s" % ok)
    assert not (ok["R1.4"] and ok["R1.5"] and ok["R1.6"])


def test_the_row_is_scored_against_the_declared_layout_not_the_published_one(
        tmp_path):
    """The negative control for the handshake, on a real graded run.

    Same run, graded twice: once with the declaration in place, once with the
    sidecar deleted. Without it the grader falls back to the PUBLISHED layout
    and stamps the verdict with the note that says the row proves nothing --
    which is precisely why the cell must block rather than let that happen.
    """
    report, row = staged_cell(tmp_path, "r1_hardcode.py", "wired/memo/1")
    assert row["measurements"]["r1_graded_layout"]["placed"] is True

    for side in report["sidecars"]:
        Path(side).unlink()
    grade_project = Path(tmp_path) / "grade"
    again = cell.grade_mujoco(TASK, Path(tmp_path) / "artifact"
                              / "lidar_nav.xml", grade_project, answer="")
    scored = again["measurements"]["r1_graded_layout"]
    assert scored["placed"] is False
    assert "PUBLISHED" in scored["source"]
    assert any("not evidence of perception" in str(n)
               for n in again["notes"]), again["notes"]


def test_the_placed_model_is_still_a_valid_mjcf(tmp_path):
    """Compiled by MuJoCo itself, so a text rewrite that produced something
    only our own scanner can read would be caught here."""
    mujoco = pytest.importorskip("mujoco")
    model = Path(tmp_path) / "lidar_nav.xml"
    shutil.copyfile(SCENE, model)
    layout = r1_core.sample_layout("wired/compile/1")
    r1_placement.place_artifact(model, layout)
    m = mujoco.MjModel.from_xml_path(str(model))
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(m.ngeom)]
    for o in layout:
        gid = names.index(o["name"])
        assert abs(float(m.geom_pos[gid][0]) - o["position"][0]) < 1e-6
        assert abs(float(m.geom_pos[gid][1]) - o["position"][1]) < 1e-6
    # ...and the scanner and the compiler agree about where the boxes are.
    for b in mjcftext.scan_bodies(model.read_text(encoding="utf-8")):
        if not b.name.startswith("OBSTACLE"):
            continue
        gid = names.index(b.name)
        assert abs(b.centre[0] - float(m.geom_pos[gid][0])) < 1e-6


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
