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

"""The MuJoCo arm, end to end, on a scene these tests build themselves.

**No other simulator is involved and none is installed for this.** A tiny MJCF
-- a ground plane, two named walls, a box that falls, and a two-joint wheeled
body an actuator drives -- is written to a temp directory, run through the real
launcher, and the resulting neutral bundle is asserted: trajectory rows, a
FROZEN t=0 inventory with world-space AABBs, and a contact that appears when
the box lands. Then the sim-neutral R1 core is run over it, to prove the arm is
gradeable rather than merely parseable.

Skipped, not failed, when ``mujoco`` is not importable: this file is about the
adapter's fidelity to MuJoCo, and on a machine without MuJoCo there is nothing
to be faithful to. The mapping rules themselves are pinned without it in
``test_mujoco_adapter.py``.

    pytest tests/benchmarks/agentbench/adapters/mujoco -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import adapters  # noqa: E402
from agentbench.adapters.mujoco import evidence, launcher, recording  # noqa: E402

pytest.importorskip("mujoco", reason="the MuJoCo arm needs the mujoco package")

# A ground plane, two named walls, one two-joint wheeled body an actuator
# drives, and a box that starts in the air. Every channel of the bundle is
# exercised by something in here and nothing in here is decorative.
TINY = """
<mujoco model="agentbench_tiny">
  <option timestep="0.002" integrator="implicitfast"/>
  <compiler autolimits="true"/>
  <worldbody>
    <camera name="eye" pos="-2 -2 1.5" xyaxes="0.7 -0.7 0 0.3 0.3 0.9"
            fovy="50" resolution="640 480"/>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <geom name="wall_north" type="box" pos="0 2.5 0.4" size="2.5 0.1 0.4"/>
    <body name="rover" pos="0 0 0.15">
      <freejoint/>
      <geom name="chassis" type="box" size="0.25 0.16 0.06" mass="6"/>
      <geom name="caster" type="sphere" pos="0.22 0 -0.09" size="0.055"
            mass="0.2" friction="0.02 0.005 0.0001"/>
      <body name="wheel_left" pos="-0.1 0.19 -0.02">
        <joint name="wheel_left_joint" type="hinge" axis="0 1 0"/>
        <geom name="tyre_left" type="cylinder" size="0.13 0.04"
              quat="0.7071068 0.7071068 0 0" mass="0.8"/>
      </body>
      <body name="wheel_right" pos="-0.1 -0.19 -0.02">
        <joint name="wheel_right_joint" type="hinge" axis="0 1 0"/>
        <geom name="tyre_right" type="cylinder" size="0.13 0.04"
              quat="0.7071068 0.7071068 0 0" mass="0.8"/>
      </body>
    </body>
    <body name="dropped_box" pos="1.2 0.8 0.8">
      <freejoint/>
      <geom name="dropped_box_geom" type="box" size="0.15 0.15 0.15"
            mass="2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="left_motor" joint="wheel_left_joint" kv="18"/>
    <velocity name="right_motor" joint="wheel_right_joint" kv="18"/>
  </actuator>
</mujoco>
"""

DRIVER = """
import sys
import mujoco

m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
d.ctrl[0] = 9.0
d.ctrl[1] = 7.0
while True:
    mujoco.mj_step(m, d)
"""

DURATION_S = 3.0


def _scene(tmp_path, *, driver=DRIVER, model=TINY, stem="tiny"):
    (tmp_path / ("%s.xml" % stem)).write_text(model, encoding="utf-8")
    if driver is not None:
        (tmp_path / ("%s.py" % stem)).write_text(driver, encoding="utf-8")
    return tmp_path / ("%s.xml" % stem)


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    """One real run, shared by the assertions below (it takes ~1 s)."""
    work = tmp_path_factory.mktemp("mujoco_tiny")
    model = _scene(work)
    out = work / "run"
    launcher.launch(model, out, duration=DURATION_S, timeout_s=300.0)
    return out


@pytest.fixture(scope="module")
def bundle(run_dir):
    return evidence.build_bundle(
        "R1_lidar_nav", robot_identity="any_robot", run_dir=run_dir,
        artifact=str(Path(run_dir).parent / "tiny.xml"))


# --- the run itself ----------------------------------------------------------


def test_the_run_produced_the_whole_artifact_set(run_dir):
    run = recording.read_run(run_dir)
    assert run.any_evidence
    assert run.missing == {}, run.missing
    assert run.errors == {}, run.errors
    assert run.compiled is True


def test_the_graders_clock_stopped_a_driver_that_never_stops(run_dir):
    """MuJoCo has no --duration; the driver loops forever on purpose."""
    run = recording.read_run(run_dir)
    assert run.completion["stopped_by"] == "duration"
    assert run.completion["complete"] is True
    assert run.completion["hook_intact"] is True
    assert run.completion["tamper"] == []
    assert run.process["exit_code"] == 0
    assert run.process["timed_out"] is False
    assert abs(run.completion["recorded_s"] - DURATION_S) < 0.02


def test_the_bundle_satisfies_the_adapter_contract(bundle):
    assert adapters.check_bundle(bundle) == []


# --- the frozen t=0 inventory ------------------------------------------------


def test_the_t0_inventory_is_frozen_at_zero(bundle):
    assert bundle.t0.frozen is True
    assert bundle.t0.t_s == 0.0


def test_every_scene_object_carries_a_world_space_aabb(bundle):
    for b in bundle.t0.bodies:
        assert b.has_aabb, b.name


def test_the_boxs_aabb_is_its_authored_geometry_in_world_space(bundle):
    box = bundle.t0.by_name("dropped_box")
    lo, hi = box.aabb
    assert [round(v, 6) for v in lo] == [1.05, 0.65, 0.65]
    assert [round(v, 6) for v in hi] == [1.35, 0.95, 0.95]
    assert round(box.height, 6) == 0.3


def test_worldbody_geoms_are_separate_objects_not_one_world_body(bundle):
    names = {b.name for b in bundle.roster.bodies}
    assert {"floor", "wall_north"} <= names
    wall = bundle.roster.by_name("wall_north")
    assert wall.kind == "world_geom" and wall.dynamic is False
    assert [round(v, 6) for v in wall.aabb_min] == [-2.5, 2.4, 0.0]


def test_the_actuated_body_is_the_only_robot_and_its_wheels_are_links(bundle):
    assert [r.name for r in bundle.roster.robots] == ["rover"]
    rover = bundle.roster.by_name("rover")
    assert rover.n_joints == 2          # the floating base is not an articulation
    assert rover.dynamic is True
    assert "drives joint" in rover.identity_evidence
    links = bundle.t0.links_of(rover.body_id)
    assert sorted(x.name for x in links) == ["wheel_left", "wheel_right"]
    assert all(x.robot_class is False for x in links)


def test_a_free_box_is_not_a_robot(bundle):
    box = bundle.t0.by_name("dropped_box")
    assert box.robot_class is False
    assert box.dynamic is True          # it moves; it is just not driven
    assert box.n_joints == 0            # a freejoint is not an articulation


def test_the_plane_publishes_its_infinite_collision_surface(bundle):
    assert any("INFINITE for collision" in n for n in bundle.notes)


# --- the pose series ---------------------------------------------------------


def test_the_trajectory_has_a_row_per_body_at_one_sample_per_step(bundle):
    t = bundle.trajectory
    assert t.n_bodies == 4              # rover, 2 wheels, box
    # One sample per completed step over the recording window -- the +-1 is
    # float accumulation on the window's own clock, not a stride.
    assert abs(t.n_samples - DURATION_S / 0.002) <= 2
    assert abs(t.recorded_s - DURATION_S) < 0.02
    assert abs(t.dt_s - 0.002) < 1e-9
    assert t.complete is True
    assert t.kinds == ["robot", "link", "link", "solid"]
    assert t.parents[1] == t.parents[2] == "body:1"
    assert t.n_top_level == 2


def test_a_body_resolved_through_the_roster_indexes_its_own_row(bundle):
    """The index invariant, checked the way a core actually uses it."""
    rover = bundle.roster.by_name("rover")
    i = bundle.trajectory.index_of(rover.body_id)
    assert i is not None
    assert bundle.trajectory.name_of(i) == "rover"


def test_the_driven_body_moved_and_the_undriven_one_did_not(bundle):
    t = bundle.trajectory
    rover = t.xy(t.index_of_name("rover"))
    crate = t.xy(t.index_of_name("dropped_box"))
    moved = math.hypot(rover[-1][0] - rover[0][0], rover[-1][1] - rover[0][1])
    drift = math.hypot(crate[-1][0] - crate[0][0], crate[-1][1] - crate[0][1])
    assert moved > 1.0
    assert drift < 0.1


def test_the_box_fell_and_came_to_rest_on_the_floor(bundle):
    z = bundle.trajectory.z(bundle.trajectory.index_of_name("dropped_box"))
    assert abs(z[0] - 0.8) < 1e-6
    assert abs(z[-1] - 0.15) < 0.01     # half-extent above a floor at z=0


# --- contacts ----------------------------------------------------------------


def test_a_contact_appears_when_the_box_lands(bundle):
    c = bundle.contacts
    assert c.supported
    pair = [p for p in c.pairs
            if {p.a, p.b} == {"floor", "dropped_box"}]
    assert len(pair) == 1
    # ...and it is not there from the start: the box was in the air.
    assert pair[0].step > 50
    assert pair[0].distinct is True


def test_both_vacuity_counters_are_real_counts(bundle):
    c = bundle.contacts
    assert c.total_observed > 0
    assert c.distinct_named > 0
    assert c.can_name_a_robot_pair is True


def test_a_wheel_contact_is_reported_under_the_ROBOTs_name(bundle):
    """Naming the link would hide a wheel-vs-wall hit from every collision
    assertion in the suite -- and would flatter this arm, not ours."""
    pair = [p for p in bundle.contacts.pairs
            if {p.a, p.b} == {"floor", "rover"}]
    assert len(pair) == 1
    assert pair[0].b_robot is True or pair[0].a_robot is True
    detail = bundle.adapter_measurements["motion"]["contact_pairs"]
    assert any("wheel" in (d.get("b_detail") or "") for d in detail)


# --- attribution and process -------------------------------------------------


def test_the_run_is_attributed_to_mujoco_with_the_solver_it_used(bundle):
    a = bundle.attribution
    assert a.backend == "mujoco"
    assert a.solver.startswith("mjSOL_")
    assert a.extra["integrator"].startswith("mjINT_")
    assert a.extra["timestep_s"] == 0.002
    assert a.extra["mujoco_version"]
    assert "COMPILED MjModel" in a.source


def test_the_process_is_clean_and_reached_a_built_and_stepped_world(bundle):
    p = bundle.process
    assert p.exit_code == 0 and p.error_lines == [] and p.timed_out is False
    assert p.clean is True
    assert p.driver_completed is True
    assert p.reached_finalize is True
    assert p.behaviour_starts == {"tiny.py": 1}
    assert bundle.live_load_ok is True


def test_the_camera_is_read_out_of_mujocos_own_frame(run_dir):
    ev = evidence.build_view_evidence("B2_subject_in_frame", run_dir=run_dir)
    assert ev.final is not None
    assert [round(v, 4) for v in ev.final.position] == [-2.0, -2.0, 1.5]
    # fovy is the FULL VERTICAL angle in degrees
    assert abs(math.degrees(ev.final.fov_v_rad) - 50.0) < 1e-6
    assert abs(ev.final.aspect - 640 / 480.0) < 1e-9
    assert abs(sum(v * v for v in ev.final.forward) - 1.0) < 1e-9


# --- the failure modes, which must be measurements ---------------------------


def test_a_model_that_does_not_compile_is_recorded_not_raised(tmp_path):
    model = _scene(tmp_path, model="<mujoco><worldbody><geom type='nope'/>"
                                   "</worldbody></mujoco>",
                   driver="import sys\nprint('never gets a model')\n")
    out = tmp_path / "run"
    _rd, facts = launcher.launch(model, out, duration=1.0, timeout_s=180.0)
    b = evidence.build_bundle("C2_fall_through_floor", run_dir=out,
                              artifact=str(model))
    assert b.live_load_ok is False
    assert any("MJCF compile error" in ln for ln in b.process.error_lines)
    assert b.attribution is None        # nothing was ever compiled or stepped
    assert facts["exit_code"] == 0      # the DRIVER exited fine; the model did not


def test_a_driver_that_raises_is_recorded_with_its_exception(tmp_path):
    model = _scene(tmp_path, driver="raise ValueError('the agent broke it')\n")
    out = tmp_path / "run"
    _rd, facts = launcher.launch(model, out, duration=1.0, timeout_s=180.0)
    run = recording.read_run(out)
    assert facts["exit_code"] == 1
    assert "ValueError" in run.completion["driver_error"]
    assert run.completion["complete"] is False
    b = evidence.build_bundle("R1_lidar_nav", run_dir=out, artifact=str(model))
    assert b.process.clean is False
    assert b.process.error_lines == []  # the agent's program, not MuJoCo's
    assert b.live_load_ok is True       # ...but the MODEL was fine


def test_a_scene_with_no_driver_is_reported_as_inert_not_as_a_still_robot(
        tmp_path):
    model = _scene(tmp_path, driver=None)
    out = tmp_path / "run"
    launcher.launch(model, out, duration=1.0, timeout_s=180.0)
    run = recording.read_run(out)
    assert "does not move on its own" in run.process["driver_rule"]
    assert run.completion["steps"] == 0
    b = evidence.build_bundle("R1_lidar_nav", run_dir=out, artifact=str(model))
    assert b.trajectory is None or b.trajectory.xyz is None
    assert b.motion_error
    assert b.process.reached_finalize is False
    assert b.process.behaviour_starts == {}


def test_a_diverging_run_still_terminates_and_publishes_its_state_resets(
        tmp_path):
    """``data.time`` is not a monotone clock, and a grader must survive that.

    When the solve diverges MuJoCo raises ``mjWARN_BADQACC`` and calls
    ``mj_resetData``, which sets simulated time back to zero. A recording
    window keyed on ``data.time`` therefore never closes: measured on this
    arm's own bring-up, a badly conditioned scene reached 311,846 steps at a
    reported 0.015 s of "simulated time". The window runs on an accumulated
    clock instead, and every discontinuity is published rather than smoothed.
    """
    unstable = TINY.replace(
        '<option timestep="0.002" integrator="implicitfast"/>',
        '<option timestep="0.05" integrator="Euler"/>').replace(
        'kv="18"', 'kv="4000"')
    model = _scene(tmp_path, model=unstable)
    out = tmp_path / "run"
    launcher.launch(model, out, duration=2.0, wall_limit=45.0,
                    timeout_s=180.0)
    run = recording.read_run(out)
    assert run.completion["stopped_by"] == "duration", run.completion
    assert run.completion["n_state_resets"] > 0
    assert run.completion["warnings"].get("mjWARN_BADQACC")
    b = evidence.build_bundle("R1_lidar_nav", run_dir=out, artifact=str(model))
    assert any("RESET the simulation state" in n for n in b.notes)
    # The time base handed to a core stays monotone, so a path integral is
    # still defined; the discontinuity is in the POSITIONS, where R1.6's
    # teleport bound can see it.
    t = b.trajectory.t
    assert all(t[i] >= t[i - 1] for i in range(1, len(t)))
    assert b.process.error_lines == []      # a warning is not an error line


def test_a_driver_that_unhooks_the_recorder_is_caught(tmp_path):
    """The driver shares this process with the recorder, so it CAN unhook it.

    That is this arm's analogue of a world project shadowing the grader's
    controller, and it is detected and published rather than assumed away.
    """
    model = _scene(tmp_path, driver=(
        "import sys, mujoco\n"
        "m = mujoco.MjModel.from_xml_path(sys.argv[1])\n"
        "d = mujoco.MjData(m)\n"
        "mujoco.mj_step(m, d)\n"
        "mujoco.mj_step = lambda *a, **k: None\n"))
    out = tmp_path / "run"
    launcher.launch(model, out, duration=1.0, timeout_s=180.0)
    run = recording.read_run(out)
    assert run.completion["hook_intact"] is False
    assert run.completion["tamper"]
    b = evidence.build_bundle("R1_lidar_nav", run_dir=out, artifact=str(model))
    assert any(n.startswith("TAMPER:") for n in b.notes)


# --- gradeable, not merely parseable -----------------------------------------


@pytest.fixture(scope="module")
def r1_probe(tmp_path_factory):
    """The R1 arena with the five published obstacles, driven BLIND at the goal.

    The instrument probe, not a solution: a blind rover drives into
    ``OBSTACLE_1``/``OBSTACLE_2``, which is what the straight START->GOAL line
    is blocked by. It exists so the three things R1 needs -- obstacle world
    AABBs, a robot trajectory, and a robot-vs-obstacle contact -- are each
    shown to RESOLVE on this arm rather than assumed to.
    """
    out = tmp_path_factory.mktemp("mujoco_r1") / "run"
    launcher.launch(launcher.LANE / "r1_probe.xml", out, duration=12.0,
                    timeout_s=300.0)
    return evidence.build_bundle(
        "R1_lidar_nav", robot_identity="any_robot", run_dir=out,
        artifact=str(launcher.LANE / "r1_probe.xml"))


def test_all_five_published_obstacles_are_matched_by_GEOMETRY(r1_probe):
    """R1.3's own matcher, run against real measured world AABBs.

    Nothing here is name-keyed: this arm's t=0 scan bounds every body and every
    world geom without being handed a list, so an agent that calls its boxes
    "crate A" is as visible as one that uses the published names. That is the
    instrument gap R1.3 currently reports on the arms whose bounds scan has to
    be told what to look at.
    """
    from agentbench.graders import r1_core
    spec = r1_core.obstacle_spec()
    non_robots = [b for b in r1_probe.roster.bodies if not b.robot_class]
    found, missing = r1_core.match_spec_obstacles(non_robots, spec)
    assert missing == []
    assert len(found) == r1_core.N_OBSTACLES
    # ...and the sensing argument R1.4-R1.6 rest on is re-derived, not trusted.
    assert r1_core.segment_blocked_by(found)


def test_a_robot_vs_obstacle_contact_is_named_so_R1_5_can_fail(r1_probe):
    """"Nothing was hit" is worth nothing until a hit can be reported."""
    from agentbench.graders import r1_core
    v = r1_core.grade(r1_probe)
    by_id = {a.id: a for a in v.assertions}
    assert by_id["R1.5"].ok is False
    hits = by_id["R1.5"].measured[r1_probe.lbl("first_hits", "first hits")]
    assert any("OBSTACLE" in h for h in hits)


def test_the_blind_probe_fails_exactly_the_behavioural_assertions(r1_probe):
    from agentbench.graders import r1_core
    v = r1_core.grade(r1_probe)
    ok = {a.id: a.ok for a in v.assertions}
    assert ok == {"R1.1": True, "R1.2": True, "R1.3": True,
                  "R1.4": False, "R1.5": False, "R1.6": False}


def test_the_sim_neutral_R1_core_grades_this_run_end_to_end(bundle):
    """The point of the whole arm: a neutral core scores it without changes.

    The outcome is a FAIL and correctly so -- this is a bring-up scene with no
    obstacles and no goal, graded against R1's task. What is asserted is that
    every assertion RESOLVED from measured evidence: the run is clean, there is
    exactly one drivable robot, and the collision clause was decided by a real
    contact query rather than reported vacuous.
    """
    from agentbench.graders import r1_core
    v = r1_core.grade(bundle)
    by_id = {a.id: a for a in v.assertions}
    assert set(by_id) == {"R1.1", "R1.2", "R1.3", "R1.4", "R1.5", "R1.6"}
    assert by_id["R1.1"].ok is True     # exit 0, no error lines, completed
    assert by_id["R1.2"].ok is True     # exactly one drivable robot
    assert by_id["R1.4"].ok is False    # no goal in a bring-up scene
    # R1.5's contact clause was DECIDED, not skipped for want of a query.
    assert "contact query supported" not in str(by_id["R1.5"].measured)
    assert by_id["R1.3"].measured       # obstacles were scannable, not blind
