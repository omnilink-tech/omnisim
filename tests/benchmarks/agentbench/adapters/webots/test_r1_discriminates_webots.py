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

"""R1 must DISCRIMINATE on the upstream-Webots arm -- SPEC 7.1's gate, run here.

The gate is two halves and is worth nothing with only one of them: an ORACLE
performing the known-good solution must PASS, and a NULL doing nothing must
FAIL. Until both have been shown for a given (task, arm) pair that arm's pass
rate on that task is unfalsifiable -- which is not hypothetical. C2 shipped a
world whose UNFIXED form passed 5/5 for a whole campaign because nobody had
asserted the task could fail, and every C2 number ever recorded was
uninformative as a result.

**This arm is the CONTROL EXPERIMENT** -- same file format, same base engine,
no OmniSim harness -- so any OmniSim/Webots delta is exactly the surface we
added. A weak or broken control manufactures a win for us and destroys the
benchmark's credibility (SPEC 6.2), which is why the world here is built from
upstream's own pieces, mounted where upstream's own lidar-navigation sample
mounts them, and why the third program below exists.

Four programs, ONE scene, the unmodified ``graders.r1_core.grade`` reached
through this arm's real launcher/recorder/evidence path:

    r1_null.py    connects and commands nothing   FAIL (R1.4, R1.5, R1.6)
    r1_blind.py   drives at the goal, never reads
                  the LiDAR                       FAIL -- and R1.5 NAMES the
                                                  obstacle it hit
    r1_oracle.py  senses with a Sick LMS 291,
                  maps, plans, drives             PASS 6/6
    ...the oracle again, on layouts nobody has published, to show the
       pass is navigation and not memorisation.

The third row is the one that had to be earned. Until 2026-08-09 the Webots
recorder queried only Robot nodes for contacts, and upstream never names the
other party -- so a robot/obstacle contact had no second participant, R1.5
counted zero collisions for EVERY run, and the assertion whose whole job is
collision-freedom could not fail on this arm. ``r1_blind`` is what proves it
can now: it strikes OBSTACLE_2 and the verdict says so.

**Cost.** These are real upstream-Webots runs in WSL2, ~15-25 s of wall each,
no GPU and no network (the asset cache is pre-seeded). SKIPPED, not failed,
where WSL or the R2025a install is absent -- every other test in this
directory is WSL-free and stays that way.
"""

from __future__ import annotations

import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.webots import (  # noqa: E402
    evidence, launcher, task_support)
from agentbench.agents import external  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

LANE = launcher.WEBOTS_LANE
WORLD = LANE / "worlds" / "r1_lidar_nav.wbt"

#: The task's own standalone window (tasks/R1_lidar_nav/meta.json).
DURATION_S = 60.0
#: The obstacle names meta.json lists for the per-body track channel. Passed so
#: this gate launches the world exactly as the campaign does.
SOLIDS = ("OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3", "OBSTACLE_4",
          "OBSTACLE_5")
#: -1 == the WHOLE run. R1.5 is phrased over the whole run and the MuJoCo arm
#: scans every step; ``meta.json`` asks this arm for **0** steps, which is the
#: open defect :func:`test_a_zero_contact_window_cannot_credit_R1_5` pins.
WHOLE_RUN = -1

#: How the robot is named in the roster, and therefore in a contact pair.
ROBOT = "Pioneer 3-AT"
#: The Pioneer 3-AT's circumscribed radius; see r1_oracle.ROBOT_RADIUS_M, which
#: is the same number derived from the PROTO's own wheel anchors.
ROBOT_RADIUS_M = 0.34


def _wsl_ready():
    """Is there a WSL2 distro with the pinned upstream install in it?"""
    try:
        p = subprocess.run(
            ["wsl", "-d", launcher.DISTRO, "--", "test", "-x",
             "%s/webots" % launcher.WEBOTS_HOME],
            capture_output=True, timeout=60)
        return p.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


pytestmark = pytest.mark.skipif(
    not _wsl_ready(),
    reason="upstream Webots R2025a is not installed in WSL2 (%s:%s); this "
           "gate needs the real arm" % (launcher.DISTRO, launcher.WEBOTS_HOME))


# --- staging ----------------------------------------------------------------

def stage(root, controller, *, world_name=None, layout=None):
    """A private Webots project holding the world and its controller.

    Staged the way ``cc_lane.run_cc_cell`` stages a COLLECTED deliverable --
    ``<root>/worlds/<name>.wbt`` beside ``<root>/controllers/<name>/`` -- so
    passing here is not a property of the lane directory's layout.
    """
    worlds, ctrls = Path(root) / "worlds", Path(root) / "controllers"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrls.mkdir(parents=True, exist_ok=True)
    src = LANE / "controllers" / controller
    dst = ctrls / controller
    if not dst.exists():
        shutil.copytree(src, dst)
    text = WORLD.read_text(encoding="utf-8")
    if controller != "r1_oracle":
        text = text.replace('controller "r1_oracle"',
                            'controller "%s"' % controller)
    if layout:
        text = with_layout(text, layout)
    world = worlds / (world_name or "r1_lidar_nav.wbt")
    world.write_text(text, encoding="utf-8")
    return world


def with_layout(text, layout):
    """``r1_lidar_nav.wbt`` with the five obstacle translations rewritten.

    Only those five lines change: the robot, the walls, the floor, the LiDAR
    mount and the controller are identical to the published world, so a
    difference in the verdict can only be the layout.
    """
    for o in layout:
        text, n = re.subn(r'(DEF %s Solid \{\n  translation )[^\n]*'
                          % o["name"],
                          r"\g<1>%g %g %g" % tuple(o["position"]), text)
        assert n == 1, "%s: rewrote %d translations" % (o["name"], n)
    return text


def run(root, controller, *, contact_steps=WHOLE_RUN, layout=None,
        world_name=None, graded_layout=None):
    """``(bundle, verdict, run_dir)`` for one real upstream-Webots run.

    The path is the campaign's: ``launcher.launch`` -> ``augment_run`` ->
    ``evidence.build_bundle(scene_inventory=True)`` -> the UNMODIFIED
    ``r1_core.grade``. R1 is not in ``cc_lane.NEEDS_AABB``, so no prober launch
    happens here either.
    """
    world = stage(root, controller, world_name=world_name, layout=layout)
    out = Path(root) / "run"
    launcher.launch(world, out, duration=DURATION_S,
                    contact_steps=contact_steps, timeout_s=900.0,
                    solids=SOLIDS)
    run2, _merged = task_support.augment_run(out, None)
    bundle = evidence.build_bundle("R1_lidar_nav", robot_identity="any_robot",
                                   live_expected=False, artifact=str(world),
                                   run=run2, scene_inventory=True)
    v = r1_core.grade(bundle, graded_layout=graded_layout,
                      layout_source="test fixture" if graded_layout else None)
    return bundle, v, out


def _ok(verdict):
    return {a.id: a.ok for a in verdict.assertions}


def _measured(verdict, aid):
    return [a for a in verdict.assertions if a.id == aid][0].measured


# --- the three runs, each done once and shared -------------------------------

@pytest.fixture(scope="module")
def oracle(tmp_path_factory):
    """Staged under the DELIVERABLE name this arm declares, not its own."""
    name = external.artifact_name("R1_lidar_nav", "webots")
    assert name, "the webots arm declares no R1 deliverable name"
    return run(tmp_path_factory.mktemp("r1_oracle"), "r1_oracle",
               world_name=name)


@pytest.fixture(scope="module")
def null(tmp_path_factory):
    return run(tmp_path_factory.mktemp("r1_null"), "r1_null")


@pytest.fixture(scope="module")
def blind(tmp_path_factory):
    return run(tmp_path_factory.mktemp("r1_blind"), "r1_blind")


# --- the positive half -------------------------------------------------------

def test_the_oracle_passes_every_assertion(oracle):
    """The half this arm did not have. A benchmark whose tasks cannot be
    passed on the control arm measures our harness, not the agent."""
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
    # ...and the collision channel was not merely quiet: the four walls and the
    # five obstacles were all bodies a touch WOULD have counted against.
    assert m5["bodies a touch would count against"] >= 9
    assert not m5["bodies with no bounds to judge"]

    m6 = _measured(v, "R1.6")
    assert m6["obstacles the track passes through"] == []
    assert m6["largest single-sample step (m)"] <= \
        r1_core.MAX_STEP_DISPLACEMENT_M
    # A drive, not a hop: at 0.62 m/s sampled every 16 ms the step is ~0.01 m,
    # an order under the teleport bound. A fixture that started satisfying
    # R1.6 by sampling coarsely would trip this.
    assert m6["largest single-sample step (m)"] < 0.05
    assert m6["distance from the declared start (m)"] < 0.05


def test_the_oracle_did_not_squeak_past_the_obstacles(oracle):
    """An oracle that passes on 2 cm of margin has not shown much.

    Measured independently of the contact channel: the minimum distance from
    the robot's centre to any published obstacle surface, over every recorded
    sample, against the radius that bounds the robot's own footprint.
    """
    bundle, _v, _rd = oracle
    traj = bundle.trajectory
    idx = traj.index_of_name(ROBOT)
    assert idx is not None, list(traj.body_ids)
    xy = traj.xy(idx)
    worst = min(_distance_to_box(p, o["position"], o["size"])
                for o in r1_core.obstacle_spec() for p in xy)
    clearance = worst - ROBOT_RADIUS_M
    assert clearance > 0.10, (
        "the oracle's true clearance beyond its own bounding radius was "
        "%.3f m; that is a pass by luck, not by navigation" % clearance)


def test_the_oracle_consumed_the_lidar_it_declares(oracle):
    """The perception claim, checked against the run's own stdout.

    R1's assertions are deliberately behavioural -- a sensor read-count is
    simulator-specific and trivially gamed -- so this is not an assertion the
    grader makes. It is this FIXTURE's own honesty check: the driver prints
    what the DEVICE reported about itself and how many scans it consumed, and
    both are checked here against the task's stated LiDAR minima.
    """
    _bundle, _v, run_dir = oracle
    text = (run_dir / "stdout.log").read_text(encoding="utf-8",
                                              errors="replace")
    dev = re.search(r"lidar '.*': (\d+) samples over ([\d.]+) deg, "
                    r"max range ([\d.]+) m", text)
    assert dev, text[-600:]
    assert int(dev.group(1)) >= 180            # LIDAR_MIN_SAMPLES
    assert float(dev.group(2)) >= 180.0        # LIDAR_MIN_FOV_DEG
    assert float(dev.group(3)) >= 5.0          # LIDAR_MIN_RANGE_M

    used = re.search(r"(\d+) scans / (\d+) beams / (\d+) occupied cells", text)
    assert used, text[-600:]
    scans, beams, cells = (int(used.group(i)) for i in (1, 2, 3))
    assert scans > 100
    assert beams == scans * int(dev.group(1))
    # The map was BUILT, not supplied: every occupied cell came from a beam.
    assert cells > 50


# --- the negative half -------------------------------------------------------

def test_doing_nothing_fails(null):
    """SPEC 1.1 / 7.1: no task may be passable by doing nothing."""
    _bundle, v, _rd = null
    assert v.outcome != "PASS", v.summary()


def test_the_null_fails_exactly_the_assertions_it_should(null):
    """...and it fails for the reason claimed, which is the other half of a
    negative control. R1.1-R1.3 SHOULD pass: the run is clean, the robot is
    drivable and the obstacles are intact. Those are true of an idle agent, and
    a gate that demanded they fail would be asking the null to be a broken
    world rather than an agent that did nothing."""
    _bundle, v, _rd = null
    assert _ok(v) == {"R1.1": True, "R1.2": True, "R1.3": True,
                      "R1.4": False, "R1.5": False, "R1.6": False}


def test_the_parked_robot_is_not_credited_with_being_collision_free(null):
    """The free pass R1.5 refuses to hand out: a robot that never moved hits
    nothing. Without ``MIN_MOTION_FOR_CREDIT_M`` the null would collect this
    assertion for doing least -- and note the contact channel was LIVE here
    (it saw the robot resting on the floor), so this is a refusal to credit,
    not a failure to measure."""
    bundle, v, _rd = null
    m5 = _measured(v, "R1.5")
    assert m5["robot-obstacle/wall contacts"] == 0
    assert m5["distance actually travelled (m)"] < \
        r1_core.MIN_MOTION_FOR_CREDIT_M
    assert bundle.contacts.supported is True
    assert bundle.contacts.total_observed > 0


def test_the_null_and_the_oracle_differ_only_in_the_program(null, oracle):
    """The controlled comparison. Same scene text but for the controller name,
    same recorder, same window, same grader -- so the verdict difference is the
    driver and nothing else."""
    nb, nv, _n = null
    ob, ov, _o = oracle
    a = Path(nb.artifact).read_text(encoding="utf-8").splitlines()
    b = Path(ob.artifact).read_text(encoding="utf-8").splitlines()
    diff = [(x, y) for x, y in zip(a, b) if x != y]
    assert len(a) == len(b) and diff == [('  controller "r1_null"',
                                          '  controller "r1_oracle"')], diff
    assert nv.outcome != ov.outcome
    # ...and both runs are real: each loaded, stepped and finalized the world.
    assert nb.process.reached_finalize is True
    assert ob.process.reached_finalize is True


def test_a_blind_driver_is_caught_by_the_collision_assertion(blind):
    """R1.5 can FAIL on this arm, and it took a recorder fix to make it so.

    ``r1_blind`` is the oracle's control law with the LiDAR read deleted: same
    robot, same world, same GPS/InertialUnit localisation, same speed limits.
    It drives the straight line the task blocked on purpose, so it strikes
    OBSTACLE_2 and stops there.

    Before 2026-08-09 this run reported **zero** collisions -- the recorder
    queried only Robot nodes and upstream never names the other party, so a
    robot/obstacle contact had no second participant. An agent that ploughed
    through a box and still reached the goal would have PASSED "nothing was
    hit". This is the measurement that says it no longer would.
    """
    _bundle, v, _rd = blind
    ok = _ok(v)
    assert ok["R1.5"] is False, v.summary()
    m5 = _measured(v, "R1.5")
    assert m5["robot-obstacle/wall contacts"] >= 1
    assert any("OBSTACLE" in h for h in m5["first hits"]), m5["first hits"]
    # It was moving when it hit, so the failure is a collision and not the
    # null's "never earned it".
    assert m5["distance actually travelled (m)"] >= \
        r1_core.MIN_MOTION_FOR_CREDIT_M
    assert ok["R1.4"] is False        # and it never arrives


def test_a_zero_contact_window_cannot_credit_R1_5(tmp_path):
    """⚠ OPEN DEFECT, measured rather than argued: as the campaign invokes it,
    R1 requests a contact window of **zero steps** on this arm.

    ``tasks/R1_lidar_nav/meta.json`` declares ``standalone.contact_steps: 0``,
    and every caller forwards it (``cc_lane.run_cc_cell``,
    ``preregister/run_oracles.py``, ``run_agentbench.py``). Zero steps means no
    contact was ever sampled, so an empty pair list says "never looked", not
    "nothing was hit" -- and the recorder now reports ``supported: false`` with
    that reason attached rather than handing R1.5 a free pass on evidence
    nobody collected.

    The consequence is stated plainly because it is the owner's to fix, not
    ours to paper over: **R1 cannot reach 6/6 on upstream Webots until the task
    asks for a contact window.** One number -- ``contact_steps: -1`` (the whole
    run, what the MuJoCo arm already does) -- makes the very same oracle pass,
    which every other test in this file demonstrates. Delete this test when
    that lands.
    """
    _bundle, v, _rd = run(tmp_path, "r1_oracle", contact_steps=0)
    ok = _ok(v)
    assert ok["R1.5"] is False
    assert _measured(v, "R1.5") == {"contact query supported": False}
    # ...and it is the ONLY thing standing between this run and a pass, which
    # is what makes the remedy a one-liner rather than a research project.
    assert [k for k, good in ok.items() if not good] == ["R1.5"], ok


@pytest.mark.parametrize("seed", [1, 20260809])
def test_the_oracle_still_arrives_when_the_obstacles_move(tmp_path, seed):
    """The anti-memorisation half: the driver reads no layout, so it does not
    care which one it is given.

    Every obstacle is redrawn by ``r1_core.sample_layout`` -- the grade-time
    placement mechanism, whose draws satisfy "a route exists" and "the straight
    line is still blocked" by construction -- and the verdict is scored against
    that same layout. A controller that had planned a fixed path from
    ``benchmark_assets/obstacles.json`` would drive into a box that moved.
    """
    layout = r1_core.sample_layout(seed)
    _bundle, v, _rd = run(tmp_path, "r1_oracle", layout=layout,
                          graded_layout=layout)
    assert v.outcome == "PASS", v.summary()
    assert _measured(v, "R1.5")["robot-obstacle/wall contacts"] == 0
    assert _measured(v, "R1.6")["obstacles the track passes through"] == []


# --- helpers ----------------------------------------------------------------

def _distance_to_box(p, centre, size):
    """Planar distance from a point to an axis-aligned box, 0 inside it."""
    dx = max(abs(float(p[0]) - centre[0]) - size[0] / 2.0, 0.0)
    dy = max(abs(float(p[1]) - centre[1]) - size[1] / 2.0, 0.0)
    return math.hypot(dx, dy)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
