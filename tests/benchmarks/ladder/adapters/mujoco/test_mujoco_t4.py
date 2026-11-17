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

"""Unit tests for the MuJoCo column's **T4** half. Most need no simulator.

Run::

    pytest tests/benchmarks/ladder/adapters/mujoco/test_mujoco_t4.py -q

The split is the T1, T2 and T3 files' and for the same reason: a bug in the
gait produces a robot that falls over and is obvious, while a bug in
``t4_channels`` produces a **verdict** that is wrong and looks fine. So the
mapping from what the sampler wrote to what the sim-neutral core reads is
tested here on a synthetic document, with no MuJoCo involved at all.

Four of these are **tripwires on the findings** in ``BRINGUP_T4.md`` rather
than tests of our code -- that MuJoCo's URDF defaults still lose ``base_link``,
that ``armature`` survives the MJCF round trip, that a plane geom's AABB is
still astronomical, and that a **welded body's equality reaction still totals
its own weight**. If a future MuJoCo changes any of them they fail, which is
the correct way to learn that a docstring has stopped being true.

And three are tripwires on the **recipe** this column rebuilt rather than on
MuJoCo: the recorded ankle-to-sole constant, the recorded hip-pitch sign, and
the reach band the task file states. All three were wrong, all three are
asserted here against the compiled model, and the corrections are reported in
``BRINGUP_T4.md`` §5 rather than folded away.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder import tasks as ladder_tasks                          # noqa: E402
from ladder.adapters.mujoco import (evidence, recording,          # noqa: E402
                                    runner_t4, t4_drive, t4_scene)
from ladder.graders import t4_evidence as t4ev                    # noqa: E402

HERE = Path(__file__).resolve().parent
CONTAINER = (Path(__file__).resolve().parents[2]
             / "tasks/T4_humanoid/container")
STRIDER_URDF = CONTAINER / "strider_description/urdf/strider.urdf"


def _mujoco():
    return pytest.importorskip("mujoco",
                               reason="the mujoco wheel is not installed")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """The scene, built once. Everything below reads it rather than rebuilding."""
    _mujoco()
    ws = tmp_path_factory.mktemp("t4scene")
    res = t4_scene.build_t4_scene(CONTAINER, ws)
    assert res.problems == [], res.problems
    return res


@pytest.fixture(scope="module")
def measured(built):
    """What the driver measures off the compiled model. Read, never typed."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return t4_drive.setup(model, data)


# --- t4_scene: reading the shipped description -------------------------------


def test_the_declared_efforts_are_read_off_the_shipped_description():
    eff = t4_scene.urdf_efforts(STRIDER_URDF)
    assert len(eff) == 12
    assert eff["hip_pitch_l_joint"] == 100.0
    assert eff["knee_l_joint"] == 120.0
    assert eff["ankle_pitch_l_joint"] == 60.0
    assert eff["ankle_roll_l_joint"] == 30.0


def test_the_spawn_height_is_the_commanded_stance_solved_backwards():
    """Not a number somebody liked: 0.03 + 0.055 + 0.02 to the hip pitch axis,
    the commanded 0.50 m hip-to-sole below it, plus 2 mm of settle clearance."""
    assert t4_scene.HIP_PITCH_BELOW_BASE_M == pytest.approx(
        0.03 + 0.055 + 0.02, abs=1e-12)
    assert t4_scene.SPAWN_Z == pytest.approx(
        t4_scene.HIP_PITCH_BELOW_BASE_M + t4_drive.STAND_HEIGHT_M + 0.002,
        abs=1e-12)
    assert t4_scene.STAND_HEIGHT_M == t4_drive.STAND_HEIGHT_M


def test_the_floor_is_long_enough_for_the_tasks_own_clock(built):
    """The slab's LENGTH is derived, and getting it wrong is a fall.

    ``phases.standalone.duration_s`` is 300 s and this gait makes about
    0.24 m/s, so a floor that a T3-sized 45 m slab would give ends the run with
    the robot walking off the edge -- which the fall test reads as a fall, not
    as an arena finding.
    """
    task = ladder_tasks.get("T4_humanoid")
    required = task.meta["arena"]["min_free_run_up_m"]
    duration = task.standalone["duration_s"]
    reach = duration * t4_drive.NOMINAL_SPEED_MPS
    assert t4_scene.GROUND_X[1] > required
    assert t4_scene.GROUND_X[1] > reach, (
        "the task's own %g s window at the gait's nominal %g m/s needs %g m of "
        "floor" % (duration, t4_drive.NOMINAL_SPEED_MPS, reach))
    assert built.ground["kind"] == "box" and built.ground["bounded"] is True


def test_the_compiled_robot_weighs_what_the_description_declares(built):
    assert built.mass_kg_compiled == pytest.approx(built.mass_kg_declared,
                                                   rel=1e-6)
    assert built.mass_kg_declared == pytest.approx(
        ladder_tasks.get("T4_humanoid").meta["robot"]["mass_kg_declared"])


def test_the_scene_keeps_the_body_the_task_grades_by_name(built):
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             "base_link") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             t4_scene.GROUND_BODY) >= 0
    for name in ("foot_l", "foot_r"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0, (
            "the tier counts make-and-break contacts PER CONTACTING BODY, so "
            "an importer that fused the feet away would make T4.3 ungradeable")


def test_the_urdf_defaults_absorb_the_base_into_the_world(tmp_path):
    """TRIPWIRE on BRINGUP_T4 §4.2, not a test of our code.

    ``compiler/fusestatic`` defaults to **true for URDF**, so the jointless
    root link is absorbed into the world body and the compiled model carries no
    ``base_link`` at all. T4 grades the base by name, so a scene built on the
    defaults is one the grader must refuse.
    """
    mujoco = _mujoco()
    raw = mujoco.MjModel.from_xml_path(str(STRIDER_URDF))
    assert mujoco.mj_name2id(raw, mujoco.mjtObj.mjOBJ_BODY, "base_link") < 0
    fixed = t4_scene.prepare_urdf(STRIDER_URDF, tmp_path / "s.urdf")
    ok = mujoco.MjModel.from_xml_path(str(fixed))
    assert mujoco.mj_name2id(ok, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0


def test_the_armature_survives_the_round_trip_to_disk(built):
    """TRIPWIRE on BRINGUP_T4 §4.1, the finding this whole column turns on.

    URDF has no field for rotor inertia; the scene sets it on the MJCF side.
    If it did not survive being written and re-read, the deliverable would be a
    robot that cannot stand and nothing would say so.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    hinge = [i for i in range(model.njnt)
             if int(model.jnt_type[i]) == int(mujoco.mjtJoint.mjJNT_HINGE)]
    assert len(hinge) == 12
    for j in hinge:
        adr = int(model.jnt_dofadr[j])
        assert float(model.dof_armature[adr]) == pytest.approx(
            t4_scene.ARMATURE)
        assert float(model.dof_damping[adr]) == pytest.approx(t4_scene.DAMPING)


def test_a_plane_floor_cannot_answer_the_arena_channel_with_a_readable_number():
    """TRIPWIRE on the reason the floor is a box, inherited from T3."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="ground">'
        '<geom type="plane" size="60 60 0.5"/></body></worldbody></mujoco>')
    assert float(model.geom_aabb[0][3]) > 1e9


def test_the_default_scene_has_no_constraint_that_could_hold_the_robot(built):
    """What makes the applied-wrench total complete rather than partial."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    assert int(model.neq) == 0
    assert int(model.nmocap) == 0
    assert int(model.ntendon) == 0
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    facts = runner_t4.constraint_rig_facts(mujoco, model, base)
    assert facts["actuators_on_the_base_free_joint"] == 0
    assert facts["base_has_a_free_joint"] is True
    assert facts["base_is_kinematic"] is False
    assert runner_t4.support_attestation(facts) == (True, None)


def test_the_weld_rig_build_really_builds_a_constraint(tmp_path):
    """The demonstration rig, which is a demonstration and not a technique."""
    mujoco = _mujoco()
    res = t4_scene.build_t4_scene(CONTAINER, tmp_path, rig="weld")
    assert res.problems == [], res.problems
    model = mujoco.MjModel.from_xml_path(res.scene)
    assert int(model.neq) == 1
    assert int(model.nmocap) == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             t4_scene.RIG_ANCHOR) >= 0
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    facts = runner_t4.constraint_rig_facts(mujoco, model, base)
    assert facts["neq"] == 1
    assert [e["name"] for e in facts["equalities"]] == [t4_scene.RIG_WELD]
    assert sorted(facts["equalities"][0]["objects"]) == ["base_link",
                                                         t4_scene.RIG_ANCHOR]


# --- t4_drive: the gait, most of it with no simulator at all ------------------


def test_exactly_one_foot_is_in_the_air_at_every_instant():
    """Single support: the whole difficulty of this tier, as arithmetic.

    A four-legged robot keeps three feet down and is statically stable. This
    one cannot, and the schedule says so rather than the prose.

    Sampled at half-step offsets on purpose. At the *exact* instant of a phase
    swap the two legs' phases are ``0.5`` and ``0.0`` and double rounding can
    put both on the planted side of the comparison for one tick -- a
    measure-zero floating-point tie, not a double-support phase, and it is
    asserted below rather than hidden by the offset.
    """
    for k in range(400):
        t = t4_drive.SETTLE_S + (k + 0.5) * t4_drive.CYCLE_S / 400.0
        airborne = [leg for leg in t4_drive.LEGS
                    if t4_drive.foot_target(leg, t)[2] == "swing"]
        assert len(airborne) == 1, (t, airborne)


def test_the_phase_swap_instant_is_a_measure_zero_tie_and_is_named_as_one():
    """The knife edge the test above steps around, stated rather than buried.

    ``0.5 - 2**-54`` plus ``0.5`` rounds to exactly ``1.0``, so at the swap
    instant both legs can read ``stance`` for a single sample. The gait is
    driven at a 2 ms step and the swap lands on a sample boundary only when the
    arithmetic is exact, so it costs nothing physically -- but a reader who
    sees it in a trace should know what it is.
    """
    t = t4_drive.SETTLE_S + 0.5 * t4_drive.CYCLE_S
    phases = [t4_drive.foot_target(leg, t)[2] for leg in t4_drive.LEGS]
    assert phases.count("swing") <= 1
    just_after = [t4_drive.foot_target(leg, t + 1e-6)[2]
                  for leg in t4_drive.LEGS]
    assert just_after.count("swing") == 1


def test_the_stance_foot_tracks_backwards_at_the_nominal_speed():
    """A planted foot that does not slip drives the body forward at that speed."""
    dt = 1e-4
    t = t4_drive.SETTLE_S + 0.1 * t4_drive.CYCLE_S      # mid-stance for "l"
    a = t4_drive.foot_target("l", t)
    b = t4_drive.foot_target("l", t + dt)
    assert a[2] == b[2] == "stance"
    assert (b[0] - a[0]) / dt == pytest.approx(-t4_drive.NOMINAL_SPEED_MPS,
                                               rel=1e-6)


def test_the_recipes_hip_sign_walks_this_robot_backwards():
    """TRIPWIRE ON THE RECIPE, BRINGUP_T4 §5.1 -- as arithmetic, no simulator.

    On this robot the hip pitch axis is ``+y``, so a *positive* hip pitch
    swings the foot **backward**. The recorded recipe solved
    ``hip = alpha + beta``; the correct solution is ``hip = beta - alpha``, and
    the two agree exactly at ``x = 0``, which is why a self-check that only
    solves the standing pose cannot see it.
    """
    st = {"L1": 0.28, "L2": 0.28, "ankle_to_sole": 0.06,
          "limits": {"hip_pitch_l_joint": (-2.0, 1.0),
                     "knee_l_joint": (-2.3, -0.05),
                     "ankle_pitch_l_joint": (-0.9, 0.9)}}
    mid = t4_drive.solve_leg(st, 0.0, 0.50)
    fwd = t4_drive.solve_leg(st, 0.09, 0.50)
    back = t4_drive.solve_leg(st, -0.09, 0.50)
    assert None not in (mid, fwd, back)
    # a foot placed FORWARD needs a SMALLER hip pitch than one placed under the
    # hip. The two off-centre solutions are symmetric about their own common
    # knee term and NOT about the centred one -- the leg is shorter straight
    # down than it is reaching out, so the knee differs.
    assert fwd[0] < mid[0] < back[0]
    alpha = math.atan2(0.09, 0.50 - 0.06)
    assert (back[0] - fwd[0]) == pytest.approx(2.0 * alpha, rel=1e-9)
    assert fwd[1] == pytest.approx(back[1], rel=1e-12)
    assert fwd[1] != pytest.approx(mid[1], rel=1e-6)
    # ...and the recipe's own expression is exactly this solution MIRRORED: its
    # answer for a foot in front is this one's answer for a foot behind, which
    # is what walks the robot the other way.
    beta = math.atan2(0.28 * math.sin(-fwd[1]),
                      0.28 + 0.28 * math.cos(-fwd[1]))
    assert (alpha + beta) == pytest.approx(back[0], rel=1e-12)
    assert (beta - alpha) == pytest.approx(fwd[0], rel=1e-12)


def test_the_ankle_keeps_the_foot_level_at_every_solution():
    st = {"L1": 0.28, "L2": 0.28, "ankle_to_sole": 0.06,
          "limits": {"hip_pitch_l_joint": (-2.0, 1.0),
                     "knee_l_joint": (-2.3, -0.05),
                     "ankle_pitch_l_joint": (-0.9, 0.9)}}
    for x in (-0.09, -0.045, 0.0, 0.045, 0.09):
        hip, knee, ankle = t4_drive.solve_leg(st, x, 0.50)
        assert ankle == pytest.approx(-(hip + knee), abs=1e-12)


def test_the_rig_knob_is_read_from_the_environment(monkeypatch):
    """Two cells from one script, and the switch must not be a source edit."""
    monkeypatch.delenv(t4_drive.RIG_ENV, raising=False)
    assert t4_drive.commanded_rig() == t4_drive.RIG_WRENCH
    monkeypatch.setenv(t4_drive.RIG_ENV, "none")
    assert t4_drive.commanded_rig() == t4_drive.RIG_NONE
    monkeypatch.setenv(t4_drive.RIG_ENV, "nonsense")
    assert t4_drive.commanded_rig() == t4_drive.RIG_WRENCH


def test_the_measured_ankle_to_sole_is_not_the_recipes_constant(measured):
    """TRIPWIRE ON THE RECIPE, BRINGUP_T4 §5.2.

    The recorded recipe used 0.04 m, described as *"ankle_roll offset 0.02 +
    foot half height 0.02"* -- which drops the foot box's own ``-0.02 m``
    origin offset. The container's ``PROVENANCE.txt`` agrees with the model:
    0.105 + 0.28 + 0.28 + 0.06 = 0.725 m from base origin to sole with every
    joint at zero, which is exactly the number it states.
    """
    assert measured["ankle_to_sole"] == pytest.approx(0.06, abs=1e-9)
    assert measured["ankle_to_sole"] != pytest.approx(0.04, abs=1e-3)
    zero_pose_sole_below_base = (t4_scene.HIP_PITCH_BELOW_BASE_M
                                 + measured["L1"] + measured["L2"]
                                 + measured["ankle_to_sole"])
    assert zero_pose_sole_below_base == pytest.approx(0.725, abs=1e-9)


def test_the_reach_band_the_task_file_states_is_wrong_and_the_model_is_right(
        measured):
    """TRIPWIRE ON THE TASK FILE, BRINGUP_T4 §5.3 -- the same shape as T3 §5.1.

    ``meta.json`` -> ``robot.standing_geometry`` says *"each leg reaches
    between 0.09 m and 0.60 m from the hip pitch axis to the sole"*. Measured
    off the compiled model against the knee's own ``-2.30 .. -0.05`` limits the
    band is **0.2888 .. 0.6198 m**. The lower end is not close: 0.09 m would
    need the two 0.28 m links folded onto each other, which this knee cannot
    do. The claim's substance -- that the knee limit and not the reach is what
    binds -- survives, and the numbers are corrected here.
    """
    lo, hi = measured["reach_hip_to_sole"]
    assert lo == pytest.approx(0.2888, abs=0.001)
    assert hi == pytest.approx(0.6198, abs=0.001)
    assert t4_drive.STAND_HEIGHT_M > lo and t4_drive.STAND_HEIGHT_M < hi


def test_the_ik_puts_the_sole_where_it_was_asked_to_over_a_whole_cycle(
        measured):
    """The IK's own falsifier, run on every deployment rather than promised."""
    assert measured["ik_residual_m"] < 1e-9
    assert len(measured["ik_checks"]) == 17
    assert measured["ik_targets_this_robot_cannot_reach"] == []
    assert measured["L1"] == pytest.approx(0.28)
    assert measured["L2"] == pytest.approx(0.28)
    assert measured["half_stance"] == pytest.approx(0.085)
    # the foot box extends AHEAD of the ankle, which is why walking forward and
    # walking backward are not mirror images on this robot
    assert measured["foot_centre_ahead_of_ankle"] == pytest.approx(0.03)
    # ...and it reaches the RUN RECORD, not only the caller
    assert t4_drive.describe()["ik_residual_m"] == measured["ik_residual_m"]


def test_the_rig_never_carries_any_weight(built):
    """``fx = fz = 0`` by construction, checked rather than promised.

    ``AGENTS.md``'s disclosure rule binds every sentence about a supported
    humanoid walk, and the one narrowing statement this rig's cell is allowed
    to make -- that the legs carried the robot and the rig only kept it upright
    -- rests entirely on this.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    st = t4_drive.setup(model, data)
    seen_nonzero = False
    for k in range(200):
        t4_drive.control(model, data, data.time)
        mujoco.mj_step(model, data)
        w = t4_drive.support_wrench(model, data, st["base"])
        assert w[0] == 0.0 and w[2] == 0.0, "the rig carried weight at k=%d" % k
        applied = data.xfrc_applied[st["base"]]
        assert float(applied[0]) == 0.0 and float(applied[2]) == 0.0
        seen_nonzero = seen_nonzero or any(abs(float(x)) > 0 for x in applied)
    assert seen_nonzero, "a rig that applied nothing would prove nothing"


# --- runner_t4: the support channel, which is the only new thing on this rung -


def test_a_welded_body_reports_its_whole_weight_as_a_constraint_reaction():
    """TRIPWIRE on the arithmetic the whole attestation rests on.

    A 12 kg body welded to a mocap anchor is held by a constraint and applies
    **nothing** through ``xfrc_applied``. The equality reaction on its free
    joint must total ``m.g`` -- and if MuJoCo ever changes its efc row order or
    its free-joint frame convention, this is what says so.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <option timestep="0.002" integrator="implicitfast"/>
      <worldbody>
        <body name="ground"><geom name="g" type="plane" size="0 0 0.05"/></body>
        <body name="anchor" mocap="true" pos="0 0 1.0"/>
        <body name="base" pos="0 0 1.0" euler="0 0 0.7">
          <freejoint name="fj"/>
          <geom name="b" type="box" size="0.1 0.14 0.22" mass="12"/>
        </body>
      </worldbody>
      <equality><weld name="rig" body1="base" body2="anchor"/></equality>
    </mujoco>""")
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    dofadr, nfree = runner_t4.base_free_dofs(mujoco, model, bid)
    assert nfree == 1 and dofadr == 0
    for _ in range(200):
        mujoco.mj_step(model, data)
    assert float(abs(data.xfrc_applied[bid]).max()) == 0.0, (
        "the whole point: an equality rig applies NO explicit wrench")
    force, torque = runner_t4.equality_reaction(mujoco, model, data, bid,
                                                dofadr)
    assert force[2] == pytest.approx(12.0 * 9.81, rel=1e-3)
    assert abs(force[0]) < 1e-6 and abs(force[1]) < 1e-6
    assert len(torque) == 3


def test_a_scene_with_no_equality_has_a_reaction_of_zero_by_construction(built):
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    dofadr, _n = runner_t4.base_free_dofs(mujoco, model, bid)
    mujoco.mj_step(model, data)
    assert runner_t4.equality_reaction(mujoco, model, data, bid, dofadr) == (
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


def test_a_kinematic_base_is_support_unverified_and_never_unsupported():
    """The other half of the hole, and the honest answer to it.

    A base welded structurally into the world has no DOFs, so no reaction can
    be read off it at all. ``T4-unsupported`` means *"nothing was applied"* and
    ``T4-support-unverified`` means *"nobody knows"*; a robot held by the model
    itself belongs in the second.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="ground">'
        '<geom type="plane" size="0 0 0.05"/></body>'
        '<body name="base_link" pos="0 0 0.6">'
        '<geom type="box" size="0.1 0.14 0.22" mass="12"/>'
        '</body></worldbody></mujoco>')
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    facts = runner_t4.constraint_rig_facts(mujoco, model, bid)
    assert facts["base_has_a_free_joint"] is False
    assert facts["base_is_kinematic"] is True
    attested, error = runner_t4.support_attestation(facts)
    assert attested is None
    assert "KINEMATIC" in error


def test_the_ground_reading_is_structural_and_its_limit_is_the_name_list(built):
    """``other_is_ground`` is "static and not the robot" -- true of a wall too.

    Which is exactly why the task's name list is authoritative: the core takes
    the stricter of the two readings, so an adapter cannot relabel the thing
    holding the robot up as the floor.
    """
    mujoco = _mujoco()
    from ladder.adapters.mujoco import runner as t1_runner
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    robot = t1_runner.robot_subtree(mujoco, model, base)
    from ladder.adapters.mujoco import runner_t3
    ground = runner_t3._ground_bodies(mujoco, model, robot)  # noqa: SLF001
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
             for i in ground}
    assert names == {t4_scene.GROUND_BODY}
    assert len(robot) == 13, "base + 2 legs x 6 links"
    assert not (ground & robot)


def test_the_deliverable_is_two_files_and_the_driver_needs_no_package(tmp_path):
    """A driver that needed the benchmark's tree would not stand alone."""
    _mujoco()
    from ladder.adapters.mujoco import run_t4
    deliverable, res = run_t4.build_deliverable(tmp_path)
    assert res.problems == []
    assert sorted(p.name for p in deliverable.iterdir()) == ["drive.py",
                                                             "scene.xml"]
    text = (deliverable / "drive.py").read_text(encoding="utf-8")
    assert "import ladder" not in text and "from ladder" not in text
    mod = runner_t4.load_driver(deliverable / "drive.py")
    assert callable(mod.control) and callable(mod.setup)


# --- t4_channels: the mapping, on a synthetic document -----------------------


def _t4_doc(*, welded=False):
    """The smallest ``t4.json`` that answers all eight channels."""
    total_f = [[0.0, 3.0, 0.0]] * 3
    total_m = [[1.0, 2.0, 0.5]] * 3
    xfrc_f = [[0.0, 0.0, 0.0]] * 3 if welded else total_f
    xfrc_m = [[0.0, 0.0, 0.0]] * 3 if welded else total_m
    return {
        "t_s": [0.0, 0.01, 0.02],
        "base_pose": {"body": "base_link",
                      "xyz": [[0, 0, 0.6], [0.01, 0, 0.601], [0.02, 0, 0.6]],
                      "rot": [[1, 0, 0, 0, 1, 0, 0, 0, 1]] * 3,
                      "source": "a synthetic fixture"},
        "standing": {"z_m": 0.5987, "body": "base_link",
                     "source": "a synthetic fixture"},
        "gait": {"contacts": [{"robot_body": "foot_l",
                               "other_body": "ground",
                               "other_is_ground": True, "other_is_robot": False,
                               "point": [0, 0, 0], "t_s": 0.01, "step": 5}],
                 "sample_times": [0.0, 0.01, 0.02], "supported": True,
                 "total_observed": 12, "distinct_named": 12, "steps": 10,
                 "window_s": 0.02, "source": "a synthetic fixture"},
        "support": {"attested": True, "t": [0.0, 0.01, 0.02],
                    "force": total_f, "torque": total_m,
                    "equality_reaction_was_live_in_samples": 3 if welded else 0,
                    "peak_equality_rows_observed": 6 if welded else 0,
                    "xfrc_only_identical_to_total": not welded,
                    "source": "a synthetic fixture"},
        "support_xfrc_only": {"attested": True,
                              "t": [0.0, 0.01, 0.02] if welded else [],
                              "force": xfrc_f if welded else [],
                              "torque": xfrc_m if welded else [],
                              "identical_to_the_attested_total": not welded,
                              "source": "a synthetic fixture"},
        "constraint_rig": {"neq": 1 if welded else 0, "equalities": [],
                           "nmocap": 1 if welded else 0, "ntendon": 0,
                           "base_has_a_free_joint": True,
                           "base_is_kinematic": False,
                           "actuators_on_the_base_free_joint": 0,
                           "source": "a synthetic fixture"},
        "arena": {"aabb_min": [-5, -6, -0.1], "aabb_max": [145, 6, 0.0],
                  "boundary_bodies": [], "source": "a synthetic fixture"},
        "base_physics": {"body": "base_link", "mass_kg": 12.0,
                         "dynamic": True, "source": "a synthetic fixture"},
        "world": {"gravity_mps2": 9.81, "gravity_vec_mps2": [0, 0, -9.81],
                  "source": "a synthetic fixture"},
        "controller": {"declared_method": "scripted", "loaded": True,
                       "evidence": "the runner imported drive.py by path",
                       "identity": "drive.py sha256=deadbeef",
                       "source": "a synthetic fixture"},
    }


def _run_with(doc, tmp_path):
    (tmp_path / "t4.json").write_text(json.dumps(doc), encoding="utf-8")
    return recording.read_run(tmp_path)


def test_every_t4_channel_is_mapped_into_the_ladders_own_dataclasses(tmp_path):
    got = evidence.t4_channels(run=_run_with(_t4_doc(), tmp_path))
    assert set(got) == set(evidence.T4_CHANNEL_KEYS)
    assert isinstance(got["base_pose"], t4ev.PoseSeries)
    assert got["base_pose"].has_orientation, "half of T4.2 dies without it"
    assert got["standing"].z_m == pytest.approx(0.5987)
    assert got["gait"].has_sample_times, "a lifted foot is unobservable without"
    assert got["gait"].classified
    assert isinstance(got["support"], t4ev.AppliedSupport)
    assert got["support"].usable and got["support"].has_series
    assert got["arena"].usable
    assert got["base_physics"].mass_kg == pytest.approx(12.0)
    assert got["world"].gravity_mps2 == pytest.approx(9.81)
    assert got["controller"].loaded is True
    assert got["controller"].method == "scripted"
    for ch in got.values():
        assert getattr(ch, "source", ""), (
            "a channel with no provenance is a number somebody typed")


def test_a_run_with_no_t4_file_supplies_nothing_rather_than_empty_channels(
        tmp_path):
    """An empty channel would read as "measured, and there was nothing there"."""
    assert evidence.t4_channels(run=recording.read_run(tmp_path)) == {}
    assert evidence.t4_channels(None) == {}


def test_the_xfrc_only_reading_is_served_only_where_it_actually_differs(
        tmp_path):
    """The hole, as a mapping test: same recording, two support readings.

    On a scene with no equality the two ARE the same series and the run says
    so; on a welded one the xfrc-only reading is zero while the total is not,
    and grading both is what turns the task file's open question into a printed
    pair of cells.
    """
    plain = evidence.t4_channels(run=_run_with(_t4_doc(), tmp_path),
                                 support_reading="xfrc_only")
    assert float(abs(plain["support"].force).max()) == pytest.approx(3.0)

    weld_dir = tmp_path / "weld"
    weld_dir.mkdir()
    run = _run_with(_t4_doc(welded=True), weld_dir)
    total = evidence.t4_channels(run=run)
    naive = evidence.t4_channels(run=run, support_reading="xfrc_only")
    assert float(abs(total["support"].force).max()) == pytest.approx(3.0)
    assert float(abs(total["support"].torque).max()) == pytest.approx(2.0)
    assert float(abs(naive["support"].force).max()) == 0.0
    assert float(abs(naive["support"].torque).max()) == 0.0


def test_the_two_readings_land_in_different_cells_on_a_welded_run(tmp_path):
    """The finding itself, end to end through the real profile, no simulator.

    ``support_profile`` is the function ``t4_core`` uses to choose the cell, so
    this is the same decision the grader makes -- and it makes it two different
    ways on one recording depending only on whether the column counted the
    constraint reaction.
    """
    run = _run_with(_t4_doc(welded=True), tmp_path)
    weight = 12.0 * 9.81
    total = evidence.t4_channels(run=run)["support"]
    naive = evidence.t4_channels(run=run,
                                 support_reading="xfrc_only")["support"]
    a = t4ev.support_profile(total, weight, 0.0, 0.02,
                             force_fraction=0.02, torque_limit_nm=2.0)
    b = t4ev.support_profile(naive, weight, 0.0, 0.02,
                             force_fraction=0.02, torque_limit_nm=2.0)
    assert a.cell == t4ev.CELL_SUPPORTED
    assert b.cell == t4ev.CELL_UNSUPPORTED
    assert "unsupported: peak 0 x body weight, 0 N.m, 0% of window" in \
        b.cell_text()


def test_a_graded_synthetic_run_reaches_the_core_and_answers_every_channel(
        tmp_path):
    """The mapping's end-to-end check, still with no simulator involved."""
    got = evidence.t4_channels(run=_run_with(_t4_doc(), tmp_path))
    ev = t4ev.T4Evidence(
        bundle=None, robot_name="base_link",
        surface=t4ev.WalkingSurface(names=("ground",), source="a fixture"),
        **got)
    assert t4ev.unanswered_channels(ev) == {}
    assert t4ev.check_t4_evidence(ev) == []


# --- the two hooks the grader looks for --------------------------------------


def test_the_column_exposes_a_t4_channel_hook_and_a_t4_phase_b_hook():
    assert callable(getattr(evidence, "t4_channels", None))
    assert callable(getattr(evidence, "t4_run_standalone", None))


def test_the_t4_channel_keys_are_the_t3_ones_because_the_contract_is_shared():
    """T4 replaces an ASSERTION, not an evidence contract."""
    assert evidence.T4_CHANNEL_KEYS == evidence.T3_CHANNEL_KEYS
    assert set(evidence.T4_CHANNEL_KEYS) == set(t4ev.T4_CHANNEL_ASSERTIONS) - {
        "base_orientation", "standing_height", "gait_contacts",
        "applied_support", "base_mass", "gravity", "controller_load"} | {
        "base_pose", "standing", "gait", "support", "base_physics", "world",
        "controller", "arena"}
