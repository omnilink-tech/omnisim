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

"""R1 must DISCRIMINATE on the MuJoCo arm -- the SPEC 7.1 gate, run here.

The gate is two halves and it is worth nothing with only one of them: an
ORACLE that performs the known-good solution must PASS, and a NULL that does
nothing must not. Until both have been shown on a given arm, that arm's pass
rate is unfalsifiable -- which is not hypothetical. C2 shipped a world whose
UNFIXED form passed 5/5 for an entire campaign, because nobody had asserted
the task could fail, and every C2 number ever recorded was uninformative as a
result.

Before this file, the MuJoCo arm had the failing half only: ``r1_probe`` (a
deliberately blind rover) FAILS R1.4/R1.5/R1.6, which proves the assertions can
fire but not that anything can satisfy them. ``sims.SIMS["mujoco"].publishable``
was False for exactly that reason.

Four programs, ONE scene (``r1_oracle.xml``), the unmodified
``graders.r1_core.grade``. Holding the world fixed and varying only the program
is the whole design: a difference in the verdict can then only be the program.

    r1_null.py       steps, commands nothing        FAIL (R1.4, R1.5, R1.6)
    r1_probe.py      drives blind at the goal       FAIL (R1.4, R1.5, R1.6)
    r1_hardcode.py   a path read off obstacles.json PASS 6/6   <-- see below
    r1_oracle.py     senses with mj_ray, then goes  PASS 6/6

The third row is not a bug in this file and it is not hidden: R1's behavioural
evidence is satisfiable by a robot that memorised the layout it is graded on --
*when the layout it is graded on is the published one*. That is what
``meta.json``'s ``anti_hardcode`` and ``status`` declare in prose and what this
fixture measures.

The answer is **grade-time placement** (``r1_core.sample_layout``): the count
and the sizes are published, the positions are drawn from the benchmark seed
under legality constraints, and the run is graded against *that* layout. Both
halves of it are exercised here on real runs --
:func:`test_the_memoriser_is_caught_when_the_layout_is_placed_at_grade_time`
(the memoriser drives into a box that moved) and
:func:`test_the_oracle_still_arrives_when_the_layout_is_placed_at_grade_time`
(the sensing navigator, which reads no layout, arrives 6/6 on a layout nobody
has published). The campaign-wide rates are in ``meta.json``: 95/120 caught,
120/120 oracle passes.

**Cost.** These are real MuJoCo runs, in-process-free (each is a child
interpreter under the arm's own launcher), ~2-4 s each and no GPU, no network
and no model quota. Skipped, not failed, without ``mujoco``.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.mujoco import evidence, launcher  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

pytest.importorskip("mujoco", reason="the MuJoCo arm needs the mujoco package")

LANE = launcher.LANE
SCENE = LANE / "r1_oracle.xml"

# The oracle driver is also imported (not only executed) so its declared
# geometry is the ONE source of the robot's bounding radius: a clearance
# assertion written against a magic 0.30 would stop meaning anything the day
# the rover changed size.
sys.path.insert(0, str(LANE))
import r1_oracle  # noqa: E402

#: The task's own standalone window (tasks/R1_lidar_nav/meta.json).
DURATION_S = 60.0

#: R1.5 resolves a contact to the robot's SUBTREE, so this is the name every
#: collision assertion sees.
ROBOT = "rover"


def _grade(run_dir, model=SCENE, layout=None):
    bundle = evidence.build_bundle("R1_lidar_nav", robot_identity="any_robot",
                                   run_dir=run_dir, artifact=str(model))
    return bundle, r1_core.grade(bundle, graded_layout=layout,
                                 layout_source="test fixture")


def _run(tmp_dir, driver, model=SCENE, duration=DURATION_S, layout=None):
    """``(bundle, verdict, run_dir)`` for one real MuJoCo run."""
    out = Path(tmp_dir) / "run"
    launcher.launch(model, out, driver=LANE / driver, duration=duration,
                    timeout_s=900.0)
    bundle, verdict = _grade(out, model, layout=layout)
    return bundle, verdict, out


def _ok(verdict):
    return {a.id: a.ok for a in verdict.assertions}


def _measured(verdict, aid):
    return [a for a in verdict.assertions if a.id == aid][0].measured


# --- the four runs, each done once and shared ------------------------------

@pytest.fixture(scope="module")
def oracle(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_oracle"), "r1_oracle.py")


@pytest.fixture(scope="module")
def null(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_null"), "r1_null.py")


@pytest.fixture(scope="module")
def hardcoded(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_hardcode"), "r1_hardcode.py")


# --- the positive half ------------------------------------------------------

def test_the_oracle_passes_every_assertion(oracle):
    """The half this arm did not have. A benchmark whose tasks cannot be
    passed measures our harness, not the agent."""
    _bundle, v, _rd = oracle
    assert v.outcome == "PASS", v.summary()
    assert all(_ok(v).values()), _ok(v)


def test_the_oracle_arrived_by_driving_and_the_numbers_say_so(oracle):
    """Each behavioural assertion, against its own threshold.

    Pinned individually rather than through the outcome so that a future edit
    which loosens one threshold cannot hide behind the other five.
    """
    _bundle, v, _rd = oracle
    m4 = _measured(v, "R1.4")
    assert m4["distance to goal (m)"] <= r1_core.GOAL_TOL_M

    m5 = _measured(v, "R1.5")
    assert m5["robot-obstacle/wall contacts"] == 0
    assert m5["distance actually travelled (m)"] >= \
        r1_core.MIN_MOTION_FOR_CREDIT_M

    m6 = _measured(v, "R1.6")
    # The floor is derived from the layout being scored -- here the published
    # one, since this run was not placed. A fixed 11.5 m was defect B.
    assert m6["integrated path length (m)"] >= \
        r1_core.min_path_length(r1_core.obstacle_spec())
    assert m6["largest single-sample step (m)"] <= \
        r1_core.MAX_STEP_DISPLACEMENT_M
    assert m6["obstacles the track passes through"] == []
    # A drive, not a hop: the per-sample step should be ~1e-3 m, two orders
    # under the teleport bound. A fixture that started satisfying R1.6 by
    # sampling coarsely would trip this.
    assert m6["largest single-sample step (m)"] < 0.01


def test_the_oracle_did_not_squeak_past_the_obstacles(oracle):
    """An oracle that passes on 2 cm of margin has not shown much.

    Measured independently of the contact channel: the minimum distance from
    the robot's centre to any published obstacle surface, over every recorded
    sample, against the radius that bounds the robot's own footprint.
    """
    bundle, _v, _rd = oracle
    traj = bundle.trajectory
    xy = traj.xy(traj.index_of_name(ROBOT))
    worst = min(_distance_to_box(p, o["position"], o["size"])
                for o in r1_core.obstacle_spec() for p in xy)
    clearance = worst - r1_oracle.ROBOT_RADIUS_M
    assert clearance > 0.10, (
        "the oracle's true clearance beyond its own bounding radius was "
        "%.3f m; that is a pass by luck, not by navigation" % clearance)


def test_the_oracle_consumed_the_lidar_it_declares(oracle):
    """The perception claim, checked against the run's own stdout.

    R1's assertions are deliberately behavioural -- a sensor read-count is
    simulator-specific and trivially gamed -- so this is not an assertion the
    grader makes. It is this FIXTURE's own honesty check: the file claims to
    cast a 181-beam fan, and the run says how many beams it actually cast.
    """
    _bundle, _v, run_dir = oracle
    text = (run_dir / "stdout.log").read_text(encoding="utf-8")
    m = re.search(r"(\d+) scans / (\d+) beams", text)
    assert m, text[-400:]
    scans, beams = int(m.group(1)), int(m.group(2))
    assert scans > 100
    assert beams == scans * r1_oracle.SCAN_BEAMS
    # ...and the fan it declares clears the task's stated LiDAR minima.
    assert r1_oracle.SCAN_BEAMS >= 180
    assert r1_oracle.SCAN_FOV_DEG >= 180.0
    assert r1_oracle.SCAN_RANGE_M >= 5.0


# --- the negative half ------------------------------------------------------

def test_doing_nothing_fails(null):
    """SPEC 1.1 / 7.1: no task may be passable by doing nothing."""
    _bundle, v, _rd = null
    assert v.outcome != "PASS", v.summary()


def test_the_null_fails_exactly_the_assertions_it_should(null):
    """...and it fails for the reason claimed, which is the other half of a
    negative control. R1.1-R1.3 SHOULD pass: the run is clean, the robot is
    drivable and the obstacles are intact. Those are true of an idle agent,
    and a gate that demanded they fail would be asking the null to be a broken
    world rather than an agent that did nothing."""
    _bundle, v, _rd = null
    assert _ok(v) == {"R1.1": True, "R1.2": True, "R1.3": True,
                      "R1.4": False, "R1.5": False, "R1.6": False}


def test_the_parked_robot_is_not_credited_with_being_collision_free(null):
    """The free pass R1.5 refuses to hand out: a robot that never moved hits
    nothing. Without ``MIN_MOTION_FOR_CREDIT_M`` the null would collect this
    assertion for doing least."""
    _bundle, v, _rd = null
    m5 = _measured(v, "R1.5")
    assert m5["robot-obstacle/wall contacts"] == 0
    assert m5["distance actually travelled (m)"] < \
        r1_core.MIN_MOTION_FOR_CREDIT_M


def test_the_null_and_the_oracle_differ_only_in_the_program(null, oracle):
    """The controlled comparison. Same scene file, same recorder, same window,
    same grader -- so the verdict difference is the driver and nothing else."""
    nb, nv, _n = null
    ob, ov, _o = oracle
    assert nb.artifact == ob.artifact
    assert nv.outcome != ov.outcome
    # ...and both runs are real: each compiled the model and stepped it.
    assert nb.process.reached_finalize is True
    assert ob.process.reached_finalize is True


# --- what the gate does NOT establish, measured rather than assumed --------

def test_a_path_read_off_the_published_file_also_passes(hardcoded):
    """R1 alone does not prove perception, and this is the demonstration.

    Same robot, same control law, same planner as the oracle; the ONLY
    difference is that the map is read from ``benchmark_assets/obstacles.json``
    before the wheels turn instead of being built from beams. It passes 6/6.

    That is not news -- ``meta.json``'s ``anti_hardcode`` says it -- but a
    declared hole nobody has ever driven through is a claim, and this is the
    measurement. It is also, since the injection step was built
    (``common/r1_placement.py``, 2026-08-10), the NEGATIVE CONTROL for that
    step: this is what a graded run looks like when no placement happened, and
    the two tests below are the same measurement with the layout placed. The
    memoriser fails there, and it fails through the campaign's own path in
    ``test_r1_placement_end_to_end.py``.
    """
    _bundle, v, _rd = hardcoded
    assert v.outcome == "PASS", v.summary()
    meta_status = _task_meta()["status"]
    assert "MAY NOT BE PUBLISHED" in meta_status.upper(), (
        "this fixture passes because THIS run was graded on the PUBLISHED "
        "layout -- no placement, by construction. R1's numbers stay "
        "unpublishable while either arm still lacks its oracle/null gate; if "
        "that has changed, re-read what this fixture is claiming")


def test_a_moved_obstacle_that_is_STRUCK_is_now_counted_as_a_hit(tmp_path):
    """Defect A, closed and measured -- this replaces the test that pinned it.

    ``grade`` used to match obstacles against the PUBLISHED layout at 0.05 m,
    and R1.5 then decided "was this a hit" by asking whether the other
    participant was among the MATCHED obstacle names. Move the obstacles and
    nothing matched, so the hit set was empty and **a robot-obstacle contact
    could not be a hit**: over 30 seeds at 1.8 m of perturbation the memorising
    driver physically struck an obstacle in **20** runs and R1.5 counted **0**
    (BRINGUP.md §3.5, finding 3). An agent that clipped a box and still
    arrived would have PASSED "nothing was hit".

    Now the grader is handed the layout the world was PLACED with, so the same
    run reports what actually happened. Both facts are asserted, from two
    independent channels: the raw adapter contact pairs (what the simulator
    saw) and R1.5's own count (what the grader made of it).
    """
    seed = 1
    layout, model = _placed_scene(tmp_path, seed)
    out = Path(tmp_path) / "run"
    launcher.launch(model, out, driver=LANE / "r1_hardcode.py",
                    duration=DURATION_S, timeout_s=900.0)

    import json
    pairs = json.loads((out / "contacts.json").read_text("utf-8"))["pairs"]
    struck = ["%s/%s" % (p["a"], p["b"]) for p in pairs
              if ROBOT in (p["a"], p["b"])
              and any("OBSTACLE" in str(x) for x in (p["a"], p["b"]))]
    assert struck, ("this fixture is supposed to drive a memorised path into "
                    "a box that moved; if it no longer does, pick another "
                    "seed rather than deleting the check")

    _bundle, v = _grade(out, model, layout=layout)
    assert _measured(v, "R1.3")["specified obstacles found"] == \
        r1_core.N_OBSTACLES, (
        "the world WAS placed with this layout, so all five must match it")
    assert _measured(v, "R1.5")["robot-obstacle/wall contacts"] >= 1, (
        "the adapter recorded %s and R1.5 must not report the opposite of "
        "what happened" % struck)
    assert v.outcome != "PASS", v.summary()


def test_the_memoriser_is_caught_when_the_layout_is_placed_at_grade_time(
        tmp_path):
    """The mechanism doing its job, on a real run rather than in a table.

    Same driver that PASSES 6/6 on the published layout above -- the oracle's
    planner and control law fed a map read from ``obstacles.json``, zero beams
    cast. Against a layout drawn at grade time it drives its memorised path
    into a box. The campaign-wide rate is 95/120 = 79.2 % (meta.json); this is
    one seed of it, pinned so a regression that quietly restored the old
    behaviour cannot pass the suite.
    """
    seed = 2
    layout, model = _placed_scene(tmp_path, seed)
    _bundle, v, _rd = _run(tmp_path, "r1_hardcode.py", model=model,
                           layout=layout)
    ok = _ok(v)
    assert v.outcome != "PASS", v.summary()
    assert ok["R1.1"] and ok["R1.2"] and ok["R1.3"], (
        "the run must fail on BEHAVIOUR -- a clean run, a drivable robot and "
        "an intact layout -- not because the fixture broke: %s" % v.summary())
    assert not ok["R1.5"], "it drove into a box; that is the mechanism"


def test_the_oracle_is_a_valid_DELIVERABLE_under_the_arms_own_convention(
        tmp_path):
    """Staged the way a collected agent deliverable is, and graded there.

    The gate above runs the pair from the lane directory under its fixture
    names. A deliverable is collected under the name
    ``agents.external.artifact_name(task, "mujoco")`` says -- ``lidar_nav.xml``
    plus the same stem with ``.py`` -- and found by
    ``launcher.find_driver``'s stem rule with no explicit driver argument. If
    the oracle only worked under its own filenames it would be a lane
    curiosity rather than the known-good solution, so this stages it under the
    shipped names, in a directory of its own, and re-grades it.
    """
    from agentbench.agents import external

    stem = Path(external.artifact_name("R1_lidar_nav", "mujoco")).stem
    assert stem, "the mujoco arm declares no R1 deliverable name"
    model = tmp_path / ("%s.xml" % stem)
    model.write_text(SCENE.read_text(encoding="utf-8"), encoding="utf-8")
    (tmp_path / ("%s.py" % stem)).write_text(
        (LANE / "r1_oracle.py").read_text(encoding="utf-8"), encoding="utf-8")

    out = tmp_path / "run"
    _rd, facts = launcher.launch(model, out, duration=DURATION_S,
                                 timeout_s=900.0)
    assert facts["driver_rule"] == "<model stem>.py beside the model"
    _bundle, v = _grade(out, model)
    assert v.outcome == "PASS", v.summary()


@pytest.mark.parametrize("seed", [7, 20260809])
def test_the_oracle_still_arrives_when_the_layout_is_placed_at_grade_time(
        tmp_path, seed):
    """A filter that also breaks honest agents is not a filter.

    The oracle is handed a layout no one has published -- drawn by
    ``r1_core.sample_layout`` from the seed. It reads no layout, so it does
    not care: it arrives, collision-free, on a map it built from beams during
    the run. Measured over 120 such layouts the oracle passes 120/120
    (meta.json); these two seeds pin it.

    All SIX assertions are asserted, R1.3 included, because the grader now
    scores the world against the layout it was placed with -- under the
    superseded perturbation R1.3 had to be expected to fail here, which is
    itself a sign that the grader was scoring the wrong scene.
    """
    layout, model = _placed_scene(tmp_path, seed)
    _bundle, v, _rd = _run(tmp_path, "r1_oracle.py", model=model,
                           layout=layout)
    assert v.outcome == "PASS", v.summary()
    assert _measured(v, "R1.5")["robot-obstacle/wall contacts"] == 0
    assert _measured(v, "R1.6")["obstacles the track passes through"] == []


# --- helpers ----------------------------------------------------------------

def _distance_to_box(p, centre, size):
    """Planar distance from a point to an axis-aligned box, 0 inside it."""
    dx = max(abs(p[0] - centre[0]) - size[0] / 2.0, 0.0)
    dy = max(abs(p[1] - centre[1]) - size[1] / 2.0, 0.0)
    return math.hypot(dx, dy)


def _placed_scene(tmp_path, seed):
    """``(layout, model)`` -- ``r1_oracle.xml`` PLACED with the layout drawn
    for ``seed`` by :func:`r1_core.sample_layout`.

    This stands in for the injection step the harness does not have yet: only
    the obstacle geom positions are rewritten in the text, so the robot, the
    walls, the actuators and the LiDAR mount are identical to the published
    scene and a verdict difference can still only be the layout or the driver.
    MJCF box ``size`` is a HALF extent and the sizes do not change, so no size
    conversion is involved.
    """
    layout = r1_core.sample_layout(seed)
    assert r1_core.layout_legality(layout)["legal"], seed
    text = SCENE.read_text(encoding="utf-8")
    for o in layout:
        text, n = re.subn(
            r'(<geom name="%s" type="box" pos=")[^"]*(")' % o["name"],
            r"\g<1>%g %g %g\g<2>" % tuple(o["position"]), text)
        assert n == 1, "%s: rewrote %d geom positions" % (o["name"], n)
    dest = Path(tmp_path) / "r1_oracle.xml"
    dest.write_text(text, encoding="utf-8")
    return layout, dest


def _task_meta():
    import json
    p = (Path(__file__).resolve().parents[2] / "tasks" / "R1_lidar_nav"
         / "meta.json")
    return json.loads(p.read_text(encoding="utf-8"))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
