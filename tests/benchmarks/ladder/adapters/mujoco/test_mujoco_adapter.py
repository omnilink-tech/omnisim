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

"""Unit tests for the MuJoCo column. **No MuJoCo needed for most of them.**

Run::

    pytest tests/benchmarks/ladder/adapters/mujoco -q

The tests that need the ``mujoco`` wheel are marked and skipped without it, so
this file still guards the pure logic -- URDF reading, the drive law, the
artifact reader, and every mapping into the neutral schema -- on a machine that
has no simulator at all. That matters because the mapping is where a column
lies: a bug in ``build_scene`` produces a robot that does not move and is
obvious, while a bug in ``_robot_class`` produces a *verdict* that is wrong and
looks fine.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import adapters as ab_adapters              # noqa: E402
from ladder.adapters.mujoco import evidence, model_build, recording, runner  # noqa: E402
from ladder.graders import ladder_evidence as lev           # noqa: E402

mujoco = pytest.importorskip("mujoco",
                             reason="the mujoco wheel is not installed")

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
CONTAINER = (REPO / "tests/benchmarks/ladder/tasks/T1_arrive/container"
             / "husky_description")

URDF_MIN = """<?xml version="1.0"?>
<robot name="tiny">
  <link name="base">
    <inertial><mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><mesh filename="package://tiny/meshes/hull.stl"/>
      </geometry></collision>
  </link>
  <link name="wheel">
    <inertial><mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="w" type="continuous">
    <parent link="base"/><child link="wheel"/>
    <origin xyz="0 0.3 0"/><axis xyz="0 1 0"/>
  </joint>
  <gazebo><plugin name="p" filename="libgazebo_ros_control.so"/></gazebo>
</robot>
"""


# --- model_build: reading a URDF --------------------------------------------


def test_read_urdf_names_links_and_joints(tmp_path):
    p = tmp_path / "tiny.urdf"
    p.write_text(URDF_MIN, encoding="utf-8")
    name, links, joints = model_build.read_urdf(p)
    assert name == "tiny"
    assert links == ["base", "wheel"]
    assert [j["name"] for j in joints] == ["w"]
    assert joints[0]["type"] == "continuous"


def test_root_link_is_the_link_that_is_never_a_child(tmp_path):
    p = tmp_path / "tiny.urdf"
    p.write_text(URDF_MIN, encoding="utf-8")
    _n, links, joints = model_build.read_urdf(p)
    assert model_build.root_link(links, joints) == "base"


def test_mesh_refs_ignores_gazebo_plugin_filenames():
    """A ``<plugin filename="lib*.so">`` uses the same attribute as a mesh.

    Reading the text instead of the XML would report a missing mesh for a
    shared library no simulator was ever asked to load -- and this Husky ships
    three of them.
    """
    refs = model_build.mesh_refs(URDF_MIN)
    assert refs == ["package://tiny/meshes/hull.stl"]
    assert not any(r.endswith(".so") for r in refs)


def test_urdf_declared_mass_sums_the_inertials(tmp_path):
    p = tmp_path / "tiny.urdf"
    p.write_text(URDF_MIN, encoding="utf-8")
    assert model_build.urdf_declared_mass(p) == pytest.approx(12.0)


def test_resolve_mesh_dirs_finds_the_basename_under_the_package(tmp_path):
    pkg = tmp_path / "tiny"
    (pkg / "meshes").mkdir(parents=True)
    (pkg / "meshes" / "hull.stl").write_bytes(b"solid\n")
    (pkg / "urdf").mkdir()
    urdf = pkg / "urdf" / "tiny.urdf"
    urdf.write_text(URDF_MIN, encoding="utf-8")
    dirs, unresolved = model_build.resolve_mesh_dirs(
        urdf, ["package://tiny/meshes/hull.stl"], package_roots=[pkg])
    assert unresolved == []
    assert [d.name for d in dirs] == ["meshes"]


def test_resolve_mesh_dirs_reports_what_it_could_not_find(tmp_path):
    pkg = tmp_path / "tiny"
    (pkg / "urdf").mkdir(parents=True)
    urdf = pkg / "urdf" / "tiny.urdf"
    urdf.write_text(URDF_MIN, encoding="utf-8")
    dirs, unresolved = model_build.resolve_mesh_dirs(
        urdf, ["package://tiny/meshes/hull.stl"], package_roots=[pkg])
    assert dirs == []
    assert unresolved == ["package://tiny/meshes/hull.stl"]


def test_dae_is_not_a_decodable_mesh_extension():
    """MuJoCo 3.8.1 has no COLLADA decoder; the Husky's visuals are all .dae.

    Measured, not assumed: with ``discardvisual="false"`` the load fails with
    ``no decoder found for mesh file '...base_link.dae'``. The URDF default
    hides it, which is exactly why the extension list is asserted here.
    """
    assert ".dae" not in model_build.DECODABLE_MESH_EXT
    assert ".stl" in model_build.DECODABLE_MESH_EXT


# --- the drive law ----------------------------------------------------------


def test_wheel_targets_are_symmetric_for_a_pure_spin():
    left, right = model_build.wheel_targets(0.0, 1.0, 0.2, 0.3)
    assert left == pytest.approx(-right)


def test_wheel_targets_are_equal_for_a_pure_advance():
    left, right = model_build.wheel_targets(1.0, 0.0, 0.5, 0.3)
    assert left == pytest.approx(right) == pytest.approx(2.0)


def test_drive_command_turns_in_place_when_badly_pointed():
    v, w, arrived, turning = runner.drive_command(
        (0.0, 0.0), 0.0, (0.0, 5.0), arrived=False, turning=True)
    assert v == 0.0 and w > 0 and turning and not arrived


def test_drive_command_drives_when_pointed_at_the_goal():
    v, w, arrived, turning = runner.drive_command(
        (0.0, 0.0), 1.5708, (0.0, 5.0), arrived=False, turning=False)
    assert v > 0 and not turning and not arrived


def test_drive_command_hysteresis_keeps_turning_below_the_entry_threshold():
    """Between the two thresholds the answer depends on which state we are in.

    Without this the robot chatters between turning and driving on the
    standing heading error a skid steer always carries, which reads as a
    physics problem and is not one.
    """
    herr_between = 0.5 * (runner.TURN_FIRST_RAD + runner.TURN_EXIT_RAD)
    yaw = 1.5708 - herr_between
    still_turning = runner.drive_command((0.0, 0.0), yaw, (0.0, 5.0),
                                         arrived=False, turning=True)
    now_driving = runner.drive_command((0.0, 0.0), yaw, (0.0, 5.0),
                                       arrived=False, turning=False)
    assert still_turning[3] is True and still_turning[0] == 0.0
    assert now_driving[3] is False and now_driving[0] > 0.0


def test_drive_command_latches_on_arrival_and_never_unlatches():
    v, w, arrived, _ = runner.drive_command((0.0, 4.95), 0.0, (0.0, 5.0),
                                            arrived=False)
    assert arrived and v == 0.0 and w == 0.0
    v2, w2, arrived2, _ = runner.drive_command((0.0, 0.0), 0.0, (0.0, 5.0),
                                               arrived=True)
    assert arrived2 and v2 == 0.0 and w2 == 0.0


def test_yaw_of_reads_the_body_x_axis():
    identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    quarter = [0, -1, 0, 1, 0, 0, 0, 0, 1]      # +90 deg about z
    assert runner.yaw_of(identity) == pytest.approx(0.0)
    assert runner.yaw_of(quarter) == pytest.approx(1.5708, abs=1e-4)


def test_wrap_pi_wraps():
    assert model_build.wrap_pi(3.5) == pytest.approx(3.5 - 2 * 3.14159265,
                                                     abs=1e-5)


# --- recording --------------------------------------------------------------


def test_read_run_records_what_is_missing_rather_than_raising(tmp_path):
    run = recording.read_run(tmp_path)
    assert not run.any_evidence
    assert set(run.missing) >= {"trajectory", "roster", "console",
                                "saved_model"}
    assert run.errors == {}


def test_read_run_records_a_parse_error_without_raising(tmp_path):
    (tmp_path / "roster.json").write_text("{not json", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.roster is None
    assert "roster" in run.errors


def test_to_arrays_truncates_ragged_bodies_and_says_so():
    doc = {"dt_s": 0.01, "bodies": [
        {"name": "a", "t": [0, 0.01, 0.02],
         "xyz": [[0, 0, 0], [1, 0, 0], [2, 0, 0]]},
        {"name": "b", "t": [0, 0.01], "xyz": [[0, 1, 0], [0, 2, 0]]}]}
    arr = recording.to_arrays(doc)
    assert arr.fatal is None
    assert arr.xyz.shape == (2, 2, 3)
    assert any("sample count" in p for p in arr.problems)


def test_to_arrays_reports_a_non_monotone_clock():
    doc = {"bodies": [{"name": "a", "t": [0, 0.02, 0.01],
                       "xyz": [[0, 0, 0], [1, 0, 0], [2, 0, 0]]}]}
    arr = recording.to_arrays(doc)
    assert any("strictly increasing" in p for p in arr.problems)


def test_to_arrays_is_fatal_but_silent_on_an_empty_document():
    arr = recording.to_arrays({"bodies": []})
    assert arr.fatal and arr.xyz is None


# --- the identity rule ------------------------------------------------------


def _rec(**kw):
    base = {"id": 1, "parent_id": 0, "n_joints_subtree": 5,
            "has_free_joint": True}
    base.update(kw)
    return base


def test_robot_class_accepts_an_articulated_child_of_the_world():
    assert evidence._robot_class(_rec(), "any_robot") is True


def test_robot_class_accepts_a_WELDED_articulated_assembly():
    """A welded chassis is a robot that cannot move, not an empty scene.

    This is the regression the ``welded`` negative fixture caught: an identity
    rule keyed on ``mjJNT_FREE`` reported *no robot in the scene* for the exact
    model a raw URDF load produces, which collapses the verdict instead of
    failing T1.1 and T1.3.
    """
    assert evidence._robot_class(
        _rec(has_free_joint=False, n_joints_subtree=4), "any_robot") is True


def test_robot_class_rejects_the_world_body():
    assert evidence._robot_class(_rec(id=0), "any_robot") is False


def test_robot_class_rejects_an_unarticulated_static_body():
    assert evidence._robot_class(
        _rec(id=9, n_joints_subtree=0, has_free_joint=False),
        "any_robot") is False


def test_robot_class_rejects_a_body_nested_inside_the_robot():
    assert evidence._robot_class(_rec(id=4, parent_id=1),
                                 "any_robot") is False


def test_robot_class_is_None_when_unanswerable():
    assert evidence._robot_class({"id": 3}, "any_robot") is None


def test_named_robot_needs_the_higher_joint_floor():
    assert evidence._robot_class(_rec(n_joints_subtree=2), "husky") is False
    assert evidence._robot_class(_rec(n_joints_subtree=5), "husky") is True


# --- the neutral mapping ----------------------------------------------------


def _synthetic_run(tmp_path, *, warnings=None, contacts=True, saved=True):
    """A minimal but complete run directory, written by hand."""
    t = [0.0, 0.01, 0.02]
    (tmp_path / "roster.json").write_text(json.dumps({
        "t_s": 0.0, "frozen": True, "bodies": [
            {"name": "world", "id": 0, "parent_id": 0, "n_joints": 0,
             "n_joints_subtree": 0, "mass_kg": 0.0, "subtree_mass_kg": 0.0,
             "has_free_joint": False},
            {"name": "base_link", "id": 1, "parent_id": 0, "n_joints": 1,
             "n_joints_subtree": 5, "mass_kg": 40.0, "subtree_mass_kg": 56.0,
             "has_free_joint": True, "aabb_min": [-0.5, -0.3, 0.0],
             "aabb_max": [0.5, 0.3, 0.4], "position": [0, 0, 0.13]},
            {"name": "ground", "id": 2, "parent_id": 0, "n_joints": 0,
             "n_joints_subtree": 0, "mass_kg": 0.0, "subtree_mass_kg": 0.0,
             "has_free_joint": False, "aabb_min": [-9, -9, -0.1],
             "aabb_max": [9, 9, 0.0], "position": [0, 0, 0]}]}),
        encoding="utf-8")
    (tmp_path / "trajectory.json").write_text(json.dumps({
        "dt_s": 0.01, "recorded_s": 0.02, "complete": True,
        "record_stride_steps": 1, "physics_timestep_s": 0.01,
        "bodies": [
            {"name": "world", "id": 0, "t": t,
             "xyz": [[0, 0, 0]] * 3},
            {"name": "base_link", "id": 1, "t": t,
             "xyz": [[0, 0, 0.13], [0, 0.1, 0.13], [0, 0.2, 0.13]]},
            {"name": "ground", "id": 2, "t": t, "xyz": [[0, 0, 0]] * 3}]}),
        encoding="utf-8")
    if contacts:
        (tmp_path / "contacts.json").write_text(json.dumps({
            "supported": True, "steps": 3, "window_s": 0.02,
            "total_observed": 12, "distinct_named": 12,
            "emit_stride_steps": 1,
            "pairs": [{"a": "ground", "b": "base_link", "a_robot": False,
                       "b_robot": True, "point": [0, 0.1, 0.0], "step": 2,
                       "t_s": 0.01}]}), encoding="utf-8")
    (tmp_path / "completion.json").write_text(json.dumps({
        "complete": True, "recorded_s": 0.02, "steps": 3, "dt_s": 0.01,
        "ctrl_writes": 3, "sim_time_at_exit_s": 0.03,
        "warnings": warnings or {},
        "scene": str(tmp_path / "scene.xml"),
        "engine": {"library": "mujoco", "version": "3.8.1",
                   "solver": "Newton", "integrator": "Euler",
                   "cone": "pyramidal", "timestep_s": 0.01}}),
        encoding="utf-8")
    (tmp_path / "process.json").write_text(json.dumps({
        "exit_code": 0, "timed_out": False, "wall_s": 1.0,
        "mujoco_version": "3.8.1"}), encoding="utf-8")
    (tmp_path / "stdout.log").write_text("", encoding="utf-8")
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    if saved:
        (tmp_path / "model_saved.xml").write_text("<mujoco model='x'/>",
                                                  encoding="utf-8")
    return tmp_path


def test_build_bundle_satisfies_the_published_adapter_contract(tmp_path):
    """``agentbench.adapters.check_bundle`` is the gate a new column must pass.

    Its own docstring says to run it *"before anyone spends tokens on a
    competitor cell"*, so it is asserted here rather than left to a human.
    """
    run_dir = _synthetic_run(tmp_path)
    bundle = evidence.build_bundle("T1_arrive", run_dir=run_dir)
    assert ab_adapters.check_bundle(bundle) == []


def test_bundle_names_the_simulator_and_the_adapter(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert bundle.sim == "mujoco"
    assert bundle.adapter.endswith("mujoco.evidence")


def test_t0_inventory_is_frozen_and_carries_world_space_aabbs(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert bundle.t0.frozen is True
    assert any(b.has_aabb for b in bundle.t0.bodies)


def test_exactly_one_body_answers_the_identity_predicate(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert [b.name for b in bundle.roster.robots] == ["base_link"]


def test_absent_contacts_stay_None_and_never_become_an_empty_result(tmp_path):
    """``None`` and "the query ran and found nothing" are different claims.

    Reporting the second when the first is true is how ``A1.3`` passed for
    weeks without being able to fail.
    """
    run_dir = _synthetic_run(tmp_path, contacts=False)
    bundle = evidence.build_bundle("T1_arrive", run_dir=run_dir)
    assert bundle.contacts is None


def test_support_observation_names_the_surface_and_times_it(tmp_path):
    run = recording.read_run(_synthetic_run(tmp_path))
    obs = evidence.support_observation(run=run)
    assert obs.supported and obs.timed
    assert obs.surface_bodies == ["ground"]
    assert obs.support_pairs[0].surface_is_robot is False
    assert lev.check_t1_evidence(
        lev.T1Evidence(bundle=evidence.build_bundle("T1_arrive", run=run),
                       support=obs,
                       waypoint=lev.Waypoint(xy=(0.0, 5.0), source="test"))
    ) == []


def test_support_observation_without_a_run_is_an_error_not_an_empty_pass():
    obs = evidence.support_observation(None)
    assert obs.error and not obs.supported
    assert obs.can_name_a_support_pair is False


def test_physics_warnings_become_error_class_lines(tmp_path):
    """MuJoCo's only runtime error channel is a counter, not a line.

    Counting console text alone would make T1.5 unfalsifiable here: a run whose
    contact buffer filled and whose solver truncated would read "clean".
    """
    run_dir = _synthetic_run(tmp_path, warnings={"mjWARN_CONTACTFULL": 4})
    bundle = evidence.build_bundle("T1_arrive", run_dir=run_dir)
    assert len(bundle.process.error_lines) == 1
    assert "mjWARN_CONTACTFULL" in bundle.process.error_lines[0]


def test_rendering_only_warnings_are_not_error_class(tmp_path):
    run_dir = _synthetic_run(tmp_path, warnings={"mjWARN_VGEOMFULL": 9})
    bundle = evidence.build_bundle("T1_arrive", run_dir=run_dir)
    assert bundle.process.error_lines == []


def test_the_saved_model_is_the_preferred_finalize_witness(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert bundle.process.reached_finalize is True
    assert "mj_saveLastXML" in bundle.process.finalize_evidence


def test_without_the_saved_model_the_weaker_witness_is_named_as_weaker(
        tmp_path):
    run_dir = _synthetic_run(tmp_path, saved=False)
    bundle = evidence.build_bundle("T1_arrive", run_dir=run_dir)
    assert bundle.process.reached_finalize is True
    assert "WEAKER" in bundle.process.finalize_evidence


def test_attribution_is_a_per_run_reading_with_a_citation(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert bundle.attribution.backend == "mujoco"
    assert bundle.attribution.solver == "Newton"
    assert "NOT NVIDIA Newton" in bundle.attribution.source


def test_an_empty_directory_produces_no_attribution_and_no_exception(tmp_path):
    bundle = evidence.build_bundle("T1_arrive", run_dir=tmp_path)
    assert bundle.attribution is None
    assert bundle.trajectory is None
    assert any("no MuJoCo run artifacts found" in n for n in bundle.notes)


def test_trajectory_rows_are_aligned_to_the_roster(tmp_path):
    bundle = evidence.build_bundle("T1_arrive",
                                   run_dir=_synthetic_run(tmp_path))
    assert bundle.trajectory.body_ids == ["#0", "#1", "#2"]
    row = bundle.trajectory.index_of(bundle.roster.by_name("base_link").body_id)
    assert bundle.trajectory.xyz[row][-1][1] == pytest.approx(0.2)


# --- with the wheel, end to end ---------------------------------------------


@pytest.mark.skipif(not CONTAINER.is_dir(),
                    reason="the T1 container is not in this checkout")
def test_the_shipped_container_urdf_compiles_and_stands(tmp_path):
    """The whole point of the column, in one test.

    Builds the T1 container's raw URDF into a MuJoCo scene by the documented
    first-party path and asserts the three things a raw load does NOT give
    you: a floating base, actuators, and the mass the description declares.
    """
    res = model_build.build_scene(CONTAINER, tmp_path)
    assert res.problems == [], res.problems
    assert res.scene and Path(res.scene).is_file()
    assert res.mass_kg_compiled == pytest.approx(res.mass_kg_declared,
                                                 rel=1e-6)

    m = mujoco.MjModel.from_xml_path(res.scene)
    assert m.nu == 4, "URDF cannot express an actuator; the build must add them"
    assert any(int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)
               for j in range(m.njnt)), "the base must not be welded"
    d = mujoco.MjData(m)
    for _ in range(400):
        mujoco.mj_step(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    assert 0.05 < float(d.xpos[bid][2]) < 0.5, "it should be resting on the floor"
    assert int(d.ncon) >= 4, "four wheels should be touching"


@pytest.mark.skipif(not CONTAINER.is_dir(),
                    reason="the T1 container is not in this checkout")
def test_a_raw_load_of_the_container_urdf_fails_on_package_uris(tmp_path):
    """The finding this column exists to record, asserted so it cannot rot.

    MuJoCo ships no ROS package resolver, so the shipped description does not
    load as it stands. If a future MuJoCo gains one this test fails, which is
    the correct way to learn that the column's headline changed.
    """
    with pytest.raises((ValueError, RuntimeError)) as exc:
        mujoco.MjModel.from_xml_path(str(CONTAINER / "urdf" / "husky.urdf"))
    assert "package://" in str(exc.value)
