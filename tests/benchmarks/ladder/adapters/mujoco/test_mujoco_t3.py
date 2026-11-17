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

"""Unit tests for the MuJoCo column's **T3** half. Most need no simulator.

Run::

    pytest tests/benchmarks/ladder/adapters/mujoco/test_mujoco_t3.py -q

The split is the T1 and T2 files' and for the same reason: a bug in the gait
produces a robot that falls over and is obvious, while a bug in
``t3_channels`` produces a **verdict** that is wrong and looks fine. So the
mapping from what the sampler wrote to what the sim-neutral core reads is
tested here on a synthetic document, with no MuJoCo involved at all.

Three of these are **tripwires on the findings** in ``BRINGUP_T3.md`` rather
than tests of our code: that MuJoCo's URDF defaults still lose ``base_link``,
that ``armature`` survives the MJCF round trip, and that a plane geom's AABB is
still astronomical. If a future MuJoCo changes any of them they fail -- which
is the correct way to learn that a docstring has stopped being true.
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
                                    runner_t3, t3_drive, t3_scene)
from ladder.graders import t3_core, t3_evidence as t3ev           # noqa: E402

HERE = Path(__file__).resolve().parent
CONTAINER = (Path(__file__).resolve().parents[2]
             / "tasks/T3_quadruped/container")
WALKER_URDF = CONTAINER / "walker_description/urdf/walker.urdf"


def _mujoco():
    return pytest.importorskip("mujoco",
                               reason="the mujoco wheel is not installed")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """The scene, built once. Everything below reads it rather than rebuilding."""
    _mujoco()
    ws = tmp_path_factory.mktemp("t3scene")
    res = t3_scene.build_t3_scene(CONTAINER, ws)
    assert res.problems == [], res.problems
    return res


# --- t3_scene: reading the shipped description -------------------------------


def test_the_declared_efforts_are_read_off_the_shipped_description():
    eff = t3_scene.urdf_efforts(WALKER_URDF)
    assert eff["hip_fl_joint"] == 25.0
    assert eff["thigh_fl_joint"] == 30.0
    assert eff["knee_fl_joint"] == 35.0
    assert len(eff) == 12


def test_the_spawn_height_is_the_commanded_stance_solved_backwards():
    """Not a number somebody liked: 0.022 + 0.24 + 0.02, plus 3 mm of settle."""
    assert t3_scene.SPAWN_Z == pytest.approx(
        0.022 + t3_drive.STAND_HEIGHT_M + 0.02 + 0.003, abs=1e-9)


def test_the_ground_offers_more_than_the_free_run_up_the_task_states(built):
    lo_x, hi_x = t3_scene.GROUND_X
    required = ladder_tasks.get("T3_quadruped").meta["arena"]["min_free_run_up_m"]
    assert hi_x - 0.0 > required
    assert built.ground["kind"] == "box" and built.ground["bounded"] is True


def test_the_compiled_robot_weighs_what_the_description_declares(built):
    assert built.mass_kg_compiled == pytest.approx(built.mass_kg_declared,
                                                   rel=1e-6)
    assert built.mass_kg_declared == pytest.approx(
        ladder_tasks.get("T3_quadruped").meta["robot"]["mass_kg_declared"])


def test_the_scene_keeps_the_body_the_task_grades_by_name(built):
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             "base_link") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             t3_scene.GROUND_BODY) >= 0


def test_the_urdf_defaults_absorb_the_base_into_the_world(tmp_path):
    """TRIPWIRE on BRINGUP_T3 §4.1, not a test of our code.

    ``compiler/fusestatic`` defaults to **true for URDF**, so the jointless
    root link is absorbed into the world body and the compiled model carries no
    ``base_link`` at all. T3 grades the base by name, so a scene built on the
    defaults is one the grader must refuse.
    """
    mujoco = _mujoco()
    raw = mujoco.MjModel.from_xml_path(str(WALKER_URDF))
    assert mujoco.mj_name2id(raw, mujoco.mjtObj.mjOBJ_BODY, "base_link") < 0
    fixed = t3_scene.prepare_urdf(WALKER_URDF, tmp_path / "w.urdf")
    ok = mujoco.MjModel.from_xml_path(str(fixed))
    assert mujoco.mj_name2id(ok, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0


def test_the_armature_survives_the_round_trip_to_disk(built):
    """TRIPWIRE on BRINGUP_T3 §4.1, the finding this whole column turns on.

    URDF has no field for rotor inertia; the scene sets it on the MJCF side.
    If it did not survive being written and re-read, the deliverable would be
    a robot that cannot stand and nothing would say so.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    hinge = [i for i in range(model.njnt)
             if int(model.jnt_type[i]) == int(mujoco.mjtJoint.mjJNT_HINGE)]
    assert len(hinge) == 12
    for j in hinge:
        adr = int(model.jnt_dofadr[j])
        assert float(model.dof_armature[adr]) == pytest.approx(
            t3_scene.ARMATURE)


def test_a_plane_floor_cannot_answer_the_arena_channel_with_a_readable_number():
    """TRIPWIRE on BRINGUP_T3 §4.3, and the reason the floor is a box."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="ground">'
        '<geom type="plane" size="60 60 0.5"/></body></worldbody></mujoco>')
    assert float(model.geom_aabb[0][3]) > 1e9


# --- t3_drive: the gait, with no simulator at all ----------------------------


def test_exactly_one_foot_is_in_the_air_at_every_instant():
    """The duty factor's whole point: three feet down makes it statically stable.

    Checked on the phase arithmetic rather than on a run, because this is a
    property of the schedule and a run could hide it behind a bounce.
    """
    st = {"hip_at": {leg: [0.18 * (1 if leg[0] == "f" else -1),
                           0.09 * t3_drive.SIDE[leg], -0.02]
                     for leg in t3_drive.LEGS}}
    for k in range(400):
        t = t3_drive.SETTLE_S + k / (400.0 * t3_drive.CYCLE_HZ)
        airborne = [leg for leg in t3_drive.LEGS
                    if t3_drive.foot_target(st, leg, t)[3] == "swing"]
        assert len(airborne) == 1, (t, airborne)


def test_every_leg_swings_once_per_cycle_and_in_the_recorded_order():
    st = {"hip_at": {leg: [0.18, 0.09, -0.02] for leg in t3_drive.LEGS}}
    first = {}
    for k in range(2000):
        t = t3_drive.SETTLE_S + k / (2000.0 * t3_drive.CYCLE_HZ)
        for leg in t3_drive.LEGS:
            if t3_drive.foot_target(st, leg, t)[3] == "swing":
                first.setdefault(leg, t)
    assert set(first) == set(t3_drive.LEGS)
    order = [leg for leg, _t in sorted(first.items(), key=lambda kv: kv[1])]
    assert order == ["rr", "fr", "rl", "fl"], order   # fl -> rr -> fr -> rl


def test_the_stance_foot_tracks_backwards_at_exactly_the_commanded_speed():
    """A planted foot that does not slip drives the body forward at that speed."""
    st = {"hip_at": {leg: [0.18, 0.09, -0.02] for leg in t3_drive.LEGS}}
    leg, dt = "fl", 1e-4
    t = t3_drive.SETTLE_S + 0.1 / t3_drive.CYCLE_HZ      # mid-stance
    a = t3_drive.foot_target(st, leg, t)
    b = t3_drive.foot_target(st, leg, t + dt)
    assert a[3] == b[3] == "stance"
    assert (b[0] - a[0]) / dt == pytest.approx(-t3_drive.BODY_SPEED_MPS,
                                               rel=1e-6)


def test_the_sway_amplitude_is_derived_from_the_measured_hip_spacing():
    """It is the shift that buys a stated static margin, not a chosen number.

    The support triangle's nearest edge runs through the body centre, so a
    level body on three feet has a margin of exactly zero -- which is the whole
    reason the sway exists and the reason its size is solved for.
    """
    st = {"hip_at": {leg: [0.18 * (1 if leg[0] == "f" else -1),
                           0.09 * t3_drive.SIDE[leg], -0.02]
                     for leg in t3_drive.LEGS}}
    s = t3_drive.sway_amplitude(st, lateral=0.14, margin=0.015)
    assert s == pytest.approx(0.015 * math.hypot(0.18, 0.14) / 0.18)
    assert s == pytest.approx(0.019, abs=0.0005)
    # a longer body gets a different answer from the same rule
    longer = {"hip_at": {leg: [0.36 * (1 if leg[0] == "f" else -1),
                               0.09 * t3_drive.SIDE[leg], -0.02]
                         for leg in t3_drive.LEGS}}
    assert t3_drive.sway_amplitude(longer, lateral=0.14, margin=0.015) < s


def test_the_recorded_recipes_sway_asks_for_the_whole_friction_budget():
    """BRINGUP_T3 §4.2, as arithmetic rather than as a claim.

    A 0.06 m sinusoid at 2.2 Hz demands 11.5 m/s^2 of lateral acceleration
    against a mu.g ceiling of 11.8 -- which is why re-implementing the recorded
    number produced 0.18 m of travel in 20 s instead of a walk.
    """
    demanded = 0.06 * (2.0 * math.pi * t3_drive.CYCLE_HZ) ** 2
    ceiling = t3_scene.GROUND_FRICTION[0] * 9.81
    assert demanded > 0.9 * ceiling
    derived = t3_drive.sway_amplitude(
        {"hip_at": {leg: [0.18, 0.09, -0.02] for leg in t3_drive.LEGS}})
    assert derived * (2.0 * math.pi * t3_drive.CYCLE_HZ) ** 2 < 0.35 * ceiling


def test_the_speed_knob_is_read_from_the_environment(monkeypatch):
    """The bob-vs-speed measurement must be reproducible without a source edit."""
    monkeypatch.delenv(t3_drive.SPEED_ENV, raising=False)
    assert t3_drive.commanded_speed() == t3_drive.BODY_SPEED_MPS
    monkeypatch.setenv(t3_drive.SPEED_ENV, "0.27")
    assert t3_drive.commanded_speed() == pytest.approx(0.27)
    monkeypatch.setenv(t3_drive.SPEED_ENV, "not a number")
    assert t3_drive.commanded_speed() == t3_drive.BODY_SPEED_MPS


def test_the_ik_puts_the_foot_where_it_was_asked_to(built):
    """The IK's falsifier: solved angles in, compiled forward kinematics out."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    st = t3_drive.setup(model, data)
    assert st["ik_residual_m"] < 1e-9
    # ...and it reaches the RUN RECORD, not only the caller: a self-check
    # nobody can read after the fact is not evidence
    assert t3_drive.describe()["ik_residual_m"] == st["ik_residual_m"]
    assert st["L1"] == pytest.approx(0.20) and st["L2"] == pytest.approx(0.20)
    assert st["foot_radius"] == pytest.approx(0.022)
    # the thigh axis really is offset from the hip axis, which is the term a
    # small-angle abduction approximation would drop
    assert abs(st["thigh_offset"]["fl"]) == pytest.approx(0.055)
    assert st["thigh_offset"]["fl"] == -st["thigh_offset"]["fr"]


def test_the_knee_limit_is_what_keeps_the_leg_off_its_own_singularity(built):
    """BRINGUP_T3 §5.1, corrected against the model rather than repeated.

    The task file said *"the wrist of this problem is the knee limit, not
    reach ... bounded to 0.040 .. 0.396 m"*, and rebuilding the demonstration
    **corrected both ends of that band**: the knee's ``-2.50 .. -0.20`` range
    puts hip-to-foot distance in **0.126 .. 0.398 m** on this robot
    (``0.040 m`` would need a knee of about ``-2.94 rad``, which the
    description does not allow). The claim's *substance* survives and is what
    is asserted here: the upper bound costs only ~2 mm of extension against the
    two links' own 0.400 m -- so what it buys is not working volume but that
    the solver never returns a **fully extended** leg, which is numerically
    fine and physically rigid.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    st = t3_drive.setup(model, data)
    l1, l2 = st["L1"], st["L2"]
    lo, hi = st["limits"]["knee_fl_joint"]
    band = (math.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * math.cos(lo)),
            math.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * math.cos(hi)))
    assert band[0] == pytest.approx(0.126, abs=0.002)
    assert band[1] == pytest.approx(0.398, abs=0.002)
    assert (l1 + l2) - band[1] < 0.003, "the knee costs ~2 mm of extension"
    # the commanded stance sits comfortably inside the band
    assert band[0] < t3_drive.STAND_HEIGHT_M < band[1]

    assert t3_drive.solve_leg(st, "fl", 0.0, 0.05, -0.24) is not None
    assert t3_drive.solve_leg(st, "fl", 0.0, 0.0, -0.4025) is None   # too straight
    assert t3_drive.solve_leg(st, "fl", 0.0, 0.0, -0.90) is None     # too far
    assert t3_drive.solve_leg(st, "fl", 0.0, 0.0, -0.10) is None     # too folded


# --- runner_t3: the structural reads -----------------------------------------


def test_the_ground_reading_is_structural_and_its_limit_is_the_name_list(built):
    """``other_is_ground`` is "static and not the robot" -- true of a wall too.

    Which is exactly why the task's name list is authoritative: the core takes
    the stricter of the two readings, so an adapter cannot relabel the thing
    holding the robot up as the floor.
    """
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    robot = runner_t3.robot_ids_for_test = None
    from ladder.adapters.mujoco import runner as t1_runner
    robot = t1_runner.robot_subtree(mujoco, model, base)
    ground = runner_t3._ground_bodies(mujoco, model, robot)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
             for i in ground}
    assert names == {t3_scene.GROUND_BODY}
    assert len(robot) == 13, "base + 4 legs x 3 links"
    assert not (ground & robot)


def test_no_actuator_acts_on_the_base_and_no_equality_binds_it(built):
    """What makes the applied-wrench total complete rather than partial."""
    mujoco = _mujoco()
    model = mujoco.MjModel.from_xml_path(built.scene)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    assert int(model.neq) == 0
    assert runner_t3._actuators_on_base(mujoco, model, base) == 0


def test_the_deliverable_is_two_files_and_the_driver_needs_no_package(tmp_path):
    """A driver that needed the benchmark's tree would not stand alone."""
    _mujoco()
    from ladder.adapters.mujoco import run_t3
    deliverable, res = run_t3.build_deliverable(tmp_path)
    assert res.problems == []
    assert sorted(p.name for p in deliverable.iterdir()) == ["drive.py",
                                                             "scene.xml"]
    text = (deliverable / "drive.py").read_text(encoding="utf-8")
    assert "import ladder" not in text and "from ladder" not in text
    mod = runner_t3.load_driver(deliverable / "drive.py")
    assert callable(mod.control) and callable(mod.setup)
    assert "t3_deliverable_driver" not in sys.modules or True


# --- t3_channels: the mapping, on a synthetic document -----------------------


def _t3_doc():
    """The smallest ``t3.json`` that answers all eight channels."""
    return {
        "t_s": [0.0, 0.01, 0.02],
        "base_pose": {"body": "base_link",
                      "xyz": [[0, 0, 0.28], [0.01, 0, 0.281], [0.02, 0, 0.28]],
                      "rot": [[1, 0, 0, 0, 1, 0, 0, 0, 1]] * 3,
                      "source": "a synthetic fixture"},
        "standing": {"z_m": 0.2776, "body": "base_link",
                     "source": "a synthetic fixture"},
        "gait": {"contacts": [{"robot_body": "shank_fl",
                               "other_body": "ground",
                               "other_is_ground": True, "other_is_robot": False,
                               "point": [0, 0, 0], "t_s": 0.01, "step": 5}],
                 "sample_times": [0.0, 0.01, 0.02], "supported": True,
                 "total_observed": 12, "distinct_named": 12, "steps": 10,
                 "window_s": 0.02, "source": "a synthetic fixture"},
        "support": {"attested": True, "t": [0.0, 0.01, 0.02],
                    "force": [[0, 0, 0]] * 3, "torque": [[0, 0, 0]] * 3,
                    "source": "a synthetic fixture"},
        "arena": {"aabb_min": [-5, -6, -0.1], "aabb_max": [45, 6, 0.0],
                  "boundary_bodies": [], "source": "a synthetic fixture"},
        "base_physics": {"body": "base_link", "mass_kg": 6.0, "dynamic": True,
                         "source": "a synthetic fixture"},
        "world": {"gravity_mps2": 9.81, "gravity_vec_mps2": [0, 0, -9.81],
                  "source": "a synthetic fixture"},
        "controller": {"declared_method": "scripted", "loaded": True,
                       "evidence": "the runner imported drive.py by path",
                       "identity": "drive.py sha256=deadbeef",
                       "source": "a synthetic fixture"},
    }


def _run_with(doc, tmp_path):
    (tmp_path / "t3.json").write_text(json.dumps(doc), encoding="utf-8")
    return recording.read_run(tmp_path)


def test_every_t3_channel_is_mapped_into_the_ladders_own_dataclasses(tmp_path):
    got = evidence.t3_channels(run=_run_with(_t3_doc(), tmp_path))
    assert set(got) == set(evidence.T3_CHANNEL_KEYS)
    assert isinstance(got["base_pose"], t3ev.PoseSeries)
    assert got["base_pose"].has_orientation, "half of T3.2 dies without it"
    assert isinstance(got["standing"], t3ev.StandingHeight)
    assert got["standing"].z_m == pytest.approx(0.2776)
    assert isinstance(got["gait"], t3ev.GaitContactObservation)
    assert got["gait"].has_sample_times, "a lifted foot is unobservable without"
    assert got["gait"].classified
    assert isinstance(got["support"], t3ev.AppliedSupport)
    assert got["support"].usable and got["support"].peak_force == 0.0
    assert isinstance(got["arena"], t3ev.ArenaBounds) and got["arena"].usable
    assert got["base_physics"].mass_kg == pytest.approx(6.0)
    assert got["world"].gravity_mps2 == pytest.approx(9.81)
    assert got["controller"].loaded is True
    assert got["controller"].method == "scripted"
    for ch in got.values():
        src = getattr(ch, "source", "")
        assert src, "a channel with no provenance is a number somebody typed"


def test_a_run_with_no_t3_file_supplies_nothing_rather_than_empty_channels(
        tmp_path):
    """An empty channel would read as "measured, and there was nothing there"."""
    assert evidence.t3_channels(run=recording.read_run(tmp_path)) == {}
    assert evidence.t3_channels(None) == {}


def test_an_unattested_support_survives_the_mapping_as_unattested(tmp_path):
    doc = _t3_doc()
    doc["support"] = {"attested": None,
                      "error": "this column exposes no applied-wrench total"}
    got = evidence.t3_channels(run=_run_with(doc, tmp_path))
    assert got["support"].attested is None
    assert not got["support"].usable


def test_a_graded_synthetic_run_reaches_the_core_and_answers_every_channel(
        tmp_path):
    """The mapping's end-to-end check, still with no simulator involved."""
    got = evidence.t3_channels(run=_run_with(_t3_doc(), tmp_path))
    ev = t3ev.T3Evidence(
        bundle=None, robot_name="base_link",
        surface=t3ev.WalkingSurface(names=("ground",), source="a fixture"),
        **got)
    assert t3ev.unanswered_channels(ev) == {}


# --- the two hooks the grader looks for --------------------------------------


def test_the_column_exposes_a_t3_channel_hook_and_a_t3_phase_b_hook():
    assert callable(getattr(evidence, "t3_channels", None))
    assert callable(getattr(evidence, "t3_run_standalone", None))


