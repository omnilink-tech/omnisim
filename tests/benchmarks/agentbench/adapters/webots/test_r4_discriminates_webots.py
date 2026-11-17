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

"""R4 must DISCRIMINATE on the upstream-Webots arm -- SPEC 7.1's gate, run here.

The gate is two halves and is worth nothing with only one of them: an ORACLE
performing the known-good solution must PASS, and a NULL doing nothing must
FAIL. Until both have been shown for a (task, arm) pair that arm's pass rate on
that task is unfalsifiable -- which is not hypothetical. C2 shipped a world
whose UNFIXED form passed 5/5 for a whole campaign because nobody had asserted
the task could fail, and every C2 number ever recorded was uninformative.

**This arm is the CONTROL EXPERIMENT** -- same file format, same base engine, no
OmniSim harness -- so any OmniSim/Webots delta is exactly the surface we added.
A weak or broken control manufactures a win for us and destroys the benchmark's
credibility (SPEC 6.2), which is why the robot in ``r4_mobile_manipulation.wbt``
is authored out of upstream's OWN base nodes -- no PROTO, no EXTERNPROTO, no
network fetch -- and why the third program below exists.

Three programs, ONE scene, the unmodified ``graders.r4_core.grade`` reached
through this arm's real launcher / recorder / evidence path:

    r4_null.py    connects and commands nothing   FAIL (R4.3, R4.5-R4.8)
    r4_blind.py   the oracle with the LiDAR read
                  DELETED: same robot, same
                  planner, same arm              FAIL -- and R4.5 NAMES the
                                                 obstacle it struck
    r4_oracle.py  senses, plans, drives, grasps,
                  carries, places                PASS 9/9

**Cost.** These are real upstream-Webots runs in WSL2 -- 150 s of simulated
time, ~35-55 s of wall each, no GPU and no network (the world fetches no
PROTO). SKIPPED, not failed, where WSL or the R2025a install is absent.
"""

from __future__ import annotations

import math
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.webots import (  # noqa: E402
    evidence, launcher, task_support)
from agentbench.agents import external  # noqa: E402
from agentbench.graders import r4_core  # noqa: E402

LANE = launcher.WEBOTS_LANE
WORLD = LANE / "worlds" / "r4_mobile_manipulation.wbt"

#: The task's own standalone window and track requests
#: (tasks/R4_mobile_manipulation/meta.json phases.standalone). Passed verbatim
#: so this gate launches the world exactly as the campaign does.
DURATION_S = 150.0
SOLIDS = ("payload", "table", "pad")
LINKS = 16
#: -1 == the WHOLE run. R4.5 is phrased over the whole drive; a first-N-steps
#: window can only witness a collision in the first N steps, which for a 150 s
#: run is none of them. R1 lost a week to exactly that.
WHOLE_RUN = -1

ROBOT = "mobile manipulator"


def _wsl_ready():
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

def stage(root, controller, *, world_name=None):
    """A private Webots project holding the world and its controller.

    Staged the way ``cc_lane.run_cc_cell`` stages a COLLECTED deliverable --
    ``<root>/worlds/<name>.wbt`` beside ``<root>/controllers/<name>/`` -- so
    passing here is not a property of the lane directory's layout.
    """
    worlds, ctrls = Path(root) / "worlds", Path(root) / "controllers"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrls.mkdir(parents=True, exist_ok=True)
    dst = ctrls / controller
    if not dst.exists():
        shutil.copytree(LANE / "controllers" / controller, dst)
    text = WORLD.read_text(encoding="utf-8")
    if controller != "r4_oracle":
        text = text.replace('controller "r4_oracle"',
                            'controller "%s"' % controller)
    world = worlds / (world_name or "mobile_manipulation.wbt")
    world.write_text(text, encoding="utf-8")
    return world


def run(root, controller, *, world_name=None):
    """``(bundle, verdict, run_dir)`` for one real upstream-Webots run.

    The path is the campaign's: ``launcher.launch`` -> ``augment_run`` ->
    ``evidence.build_bundle(scene_inventory=True)`` -> the UNMODIFIED
    ``r4_core.grade``. R4 is not in ``cc_lane.NEEDS_AABB``, so no prober launch
    happens here either.
    """
    world = stage(root, controller, world_name=world_name)
    out = Path(root) / "run"
    launcher.launch(world, out, duration=DURATION_S, contact_steps=WHOLE_RUN,
                    timeout_s=900.0, solids=SOLIDS, links=LINKS)
    run2, _merged = task_support.augment_run(out, None)
    bundle = evidence.build_bundle("R4_mobile_manipulation",
                                   robot_identity="any_robot",
                                   live_expected=False, artifact=str(world),
                                   run=run2, scene_inventory=True)
    return bundle, r4_core.grade(bundle), out


def _ok(verdict):
    return {a.id: a.ok for a in verdict.assertions}


def _measured(verdict, aid):
    return [a for a in verdict.assertions if a.id == aid][0].measured


# --- the three runs, each done once and shared -------------------------------

@pytest.fixture(scope="module")
def oracle(tmp_path_factory):
    """Staged under the DELIVERABLE name this arm declares, not its own."""
    name = external.artifact_name("R4_mobile_manipulation", "webots")
    assert name, "the webots arm declares no R4 deliverable name"
    return run(tmp_path_factory.mktemp("r4_oracle"), "r4_oracle",
               world_name=name)


@pytest.fixture(scope="module")
def null(tmp_path_factory):
    return run(tmp_path_factory.mktemp("r4_null"), "r4_null")


@pytest.fixture(scope="module")
def blind(tmp_path_factory):
    return run(tmp_path_factory.mktemp("r4_blind"), "r4_blind")


# --- the positive half -------------------------------------------------------

def test_the_oracle_passes_every_assertion(oracle):
    """A benchmark whose tasks cannot be passed on the control arm measures our
    harness, not the agent."""
    _bundle, v, _rd = oracle
    assert v.outcome == "PASS", v.summary()
    assert all(_ok(v).values()), _ok(v)


def test_the_oracle_navigated_and_touched_nothing(oracle):
    """R4.5's numbers, pinned individually rather than through the outcome, so
    a future edit that loosens one threshold cannot hide behind the others."""
    _bundle, v, _rd = oracle
    m = _measured(v, "R4.5")
    assert m["robot-obstacle/wall contacts"] == 0
    assert m["obstacles the track passes through"] == []
    assert m["distance actually driven (m)"] >= r4_core.MIN_MOTION_FOR_CREDIT_M
    # ...and the collision channel was not merely quiet: the five obstacles and
    # the four walls were all bodies a touch WOULD have counted against.
    assert m["bodies a touch would count against"] >= 9
    assert not m["bodies with no bounds to judge"]
    # the ORDER, not just the visits
    assert m["first sample within 1.5 m of the table (s)"] < \
        m["first later sample within 1.5 m of the pad (s)"]


def test_the_oracle_actually_carried_the_payload(oracle):
    """The grasp proof's own numbers. This is the assertion R4 exists for, and
    it is the one that took the longest to make true on this arm: a friction
    pinch that survives a multi-metre drive is a property of the CONTACT MODEL,
    not of an API."""
    _bundle, v, _rd = oracle
    m = _measured(v, "R4.6")
    assert m["the carry began with the payload over the table"] is True
    assert m["base travel during the carry (m)"] >= r4_core.TRANSPORT_MIN_M
    assert m["payload travel during the carry (m)"] >= r4_core.TRANSPORT_MIN_M
    assert m["spread of |payload - base| during the carry (m)"] <= \
        r4_core.CARRY_RADIUS_TOL_M
    assert m["spread of (payload z - base z) during the carry (m)"] <= \
        r4_core.CARRY_DZ_TOL_M
    # ...and it tracked the GRIPPER, not the deck: the payload stayed within a
    # few centimetres of an ARTICULATED link the whole way.
    assert m["furthest the payload got from it during the carry (m)"] <= \
        r4_core.GRIP_PROXIMITY_M
    assert m["the articulated link the payload stayed nearest"]

    m7 = _measured(v, "R4.7")
    assert m7["separate airborne episodes of >= 0.5 s"] == 1
    assert m7["distance from the pad where the carry ended (m)"] <= \
        r4_core.RELEASE_NEAR_PAD_M


def test_the_oracle_delivered_it_onto_the_pad(oracle):
    """...and left it there, measured against the MEASURED pad."""
    _bundle, v, _rd = oracle
    m = _measured(v, "R4.8")
    assert m["of those, outside the delivery tolerance"] == 0
    assert m["of those, not at the pad rest height"] == 0
    assert m["samples in the last 2.0 s"] >= r4_core.HOLD_MIN_SAMPLES
    x, y, z = m["final payload xyz (m)"]
    assert math.hypot(x - r4_core.PAD_XY[0], y - r4_core.PAD_XY[1]) <= \
        r4_core.DELIVERY_TOL_M
    assert abs(z - r4_core.PAYLOAD_REST_Z_PAD_M) <= r4_core.PLACE_Z_TOL_M


def test_the_oracle_read_the_lidar_it_declares(oracle):
    """The perception claim, checked against the run's own stdout.

    R4's graded assertions are deliberately behavioural -- a sensor read-count
    is simulator-specific and trivially gamed -- so this is not something the
    GRADER asserts. It is this FIXTURE's own honesty check: the driver prints
    what the DEVICE reported about itself and how many beams it consumed, and
    both are checked against the task's stated LiDAR minima.
    """
    import re
    _bundle, _v, run_dir = oracle
    text = (run_dir / "stdout.log").read_text(encoding="utf-8",
                                              errors="replace")
    dev = re.search(r"lidar '.*': (\d+) samples over ([\d.]+) deg, "
                    r"max range ([\d.]+) m", text)
    assert dev, text[-600:]
    assert int(dev.group(1)) >= r4_core.LIDAR_MIN_SAMPLES
    assert float(dev.group(2)) >= r4_core.LIDAR_MIN_FOV_DEG
    assert float(dev.group(3)) >= r4_core.LIDAR_MIN_RANGE_M
    used = re.search(r"(\d+) scans / (\d+) beams / (\d+) occupied cells", text)
    assert used, text[-600:]
    scans, beams, cells = (int(used.group(i)) for i in (1, 2, 3))
    assert scans > 100 and beams == scans * int(dev.group(1))
    assert cells > 50          # the map was BUILT, not supplied


# --- the negative half -------------------------------------------------------

def test_doing_nothing_fails(null):
    """SPEC 1.1 / 7.1: no task may be passable by doing nothing."""
    _bundle, v, _rd = null
    assert v.outcome != "PASS", v.summary()


def test_the_null_fails_exactly_the_assertions_it_should(null):
    """...and for the reason claimed, which is the other half of a negative
    control.

    R4.1, R4.2, R4.4 and R4.9 SHOULD pass: the run is clean, the scene is
    intact, the payload is on the table and nothing was teleported. All four
    are true of an idle agent, and a gate that demanded they fail would be
    asking the null to be a broken world rather than an agent that did nothing.

    R4.3 fails, and that is deliberate rather than incidental: its articulation
    clause is BEHAVIOURAL (link bodies that translate in the base frame), so a
    robot that never moved its arm cannot satisfy it. No honest agent is
    harmed -- one that builds a manipulator and drives it passes, and one that
    builds a manipulator and never runs it fails R4.5-R4.8 anyway.
    """
    _bundle, v, _rd = null
    assert _ok(v) == {"R4.1": True, "R4.2": True, "R4.3": False,
                      "R4.4": True, "R4.5": False, "R4.6": False,
                      "R4.7": False, "R4.8": False, "R4.9": True}


def test_the_parked_robot_is_not_credited_with_being_collision_free(null):
    """The free pass R4.5 refuses to hand out: a robot that never moved hits
    nothing. Note the contact channel was LIVE (it saw the robot resting on the
    floor), so this is a refusal to credit, not a failure to measure."""
    bundle, v, _rd = null
    m = _measured(v, "R4.5")
    assert m["robot-obstacle/wall contacts"] == 0
    assert m["distance actually driven (m)"] < r4_core.MIN_MOTION_FOR_CREDIT_M
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
    assert len(a) == len(b) and diff == [('  controller "r4_null"',
                                          '  controller "r4_oracle"')], diff
    assert nv.outcome != ov.outcome
    # ...and both runs are real: each loaded, stepped and finalized the world.
    assert nb.process.reached_finalize is True
    assert ob.process.reached_finalize is True


def test_a_blind_driver_is_caught_by_the_collision_assertion(blind):
    """R4.5 can FAIL on this arm, and the run says WHAT was struck.

    ``r4_blind`` is the oracle's control law with the LiDAR read deleted: same
    robot, same world, same GPS/InertialUnit localisation, same planner, same
    arm, same speeds. Its occupancy grid stays empty, so it plans the straight
    line the task blocked on purpose and drives into OBSTACLE_1.

    An assertion that cannot go red is not evidence -- R1 shipped one for a
    week (its recorder queried only Robot nodes, so a robot/obstacle contact
    had no second participant and every run reported zero collisions). This is
    the measurement that says R4.5 is not that.
    """
    _bundle, v, _rd = blind
    ok = _ok(v)
    assert ok["R4.5"] is False, v.summary()
    m = _measured(v, "R4.5")
    assert m["robot-obstacle/wall contacts"] >= 1
    assert any("obstacle" in h.lower() for h in m["what it struck"]), \
        m["what it struck"]
    # It was moving when it hit, so the failure is a COLLISION and not the
    # null's "never earned it".
    assert m["distance actually driven (m)"] >= \
        r4_core.MIN_MOTION_FOR_CREDIT_M
    # ...and it never reaches the table, so it never picks anything up.
    assert m["first sample within 1.5 m of the table (s)"] is None
    assert ok["R4.6"] is False and ok["R4.8"] is False
    # But it IS a mobile manipulator and it DID move its arm: R4.3 passes, so
    # this control is controlling for blindness and nothing else.
    assert ok["R4.3"] is True


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
