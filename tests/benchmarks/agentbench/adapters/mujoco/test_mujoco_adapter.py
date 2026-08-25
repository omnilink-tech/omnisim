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

"""The MuJoCo -> neutral-bundle mapping. **No MuJoCo needed to run these.**

The competitor's adapter must be auditable by someone who does not have our
stack, and testable on a machine with no simulator at all -- so every mapping
rule is pinned here against artifact fixtures on disk, exactly as the Webots
arm's ``test_webots_adapter.py`` is. What needs a real engine (does the
recorder read MuJoCo correctly?) lives in ``test_mujoco_end_to_end.py`` and
skips when ``mujoco`` is absent.

    pytest tests/benchmarks/agentbench/adapters/mujoco -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import adapters  # noqa: E402
from agentbench.adapters.mujoco import evidence, recording  # noqa: E402


# --- fixtures ----------------------------------------------------------------


def _roster(**over):
    doc = {
        "t_s": 0.0, "frozen": True,
        "robot_roots": [1],
        "actuated_roots": {"1": ["actuator 'ml' drives joint 'jl'"]},
        "forced_roots": {},
        "plane_notes": [],
        "cameras": [],
        "bodies": [
            {"key": "body:1", "name": "rover", "id": 1, "root": 1,
             "top_level": True, "position": [0.0, 0.0, 0.2],
             "aabb_min": [-0.3, -0.2, 0.1], "aabb_max": [0.3, 0.2, 0.3],
             "n_joints": 2, "mass_kg": 6.0, "movable": True,
             "actuated_reasons": ["actuator 'ml' drives joint 'jl'"]},
            {"key": "body:2", "name": "wheel_left", "id": 2, "root": 1,
             "top_level": False, "position": [0.0, 0.25, 0.15],
             "aabb_min": [-0.1, 0.2, 0.05], "aabb_max": [0.1, 0.3, 0.25],
             "mass_kg": 0.8, "movable": True, "parent_key": "body:1"},
            {"key": "body:3", "name": "crate", "id": 3, "root": 3,
             "top_level": True, "position": [2.0, 0.0, 0.2],
             "aabb_min": [1.8, -0.2, 0.0], "aabb_max": [2.2, 0.2, 0.4],
             "n_joints": 0, "mass_kg": 3.0, "movable": True,
             "actuated_reasons": []},
        ],
        "world_geoms": [
            {"key": "geom:0", "name": "floor", "id": 0,
             "geom_type": "mjGEOM_PLANE", "position": [0, 0, 0],
             "aabb_min": [-5.0, -5.0, 0.0], "aabb_max": [5.0, 5.0, 0.0],
             "movable": False},
            {"key": "geom:1", "name": "wall_north", "id": 1,
             "geom_type": "mjGEOM_BOX", "position": [0, 3, 0.5],
             "aabb_min": [-3.0, 2.9, 0.0], "aabb_max": [3.0, 3.1, 1.0],
             "movable": False},
        ],
    }
    doc.update(over)
    return doc


def write_run(d, *, roster=None, contacts=None, trajectory=True,
              completion=None, process=None, model_info=None,
              model_load=None, n=5, dt=0.002):
    d = Path(d)
    d.mkdir(parents=True, exist_ok=True)
    _w(d / "roster.json", _roster() if roster is None else roster)
    _w(d / "contacts.json", {
        "supported": True, "steps": n, "stride": 1, "window_s": dt * (n - 1),
        "total_observed": 12, "distinct_named": 12, "pairs_truncated": False,
        "pairs": [{"a": "floor", "b": "rover", "a_key": "geom:0",
                   "b_key": "body:1", "a_root": 0, "b_root": 1,
                   "a_detail": "floor", "b_detail": "tyre (wheel_left)",
                   "self_pair": False, "point": [0, 0, 0], "step": 1,
                   "t_s": 0.002}],
    } if contacts is None else contacts)
    if trajectory:
        bodies = [{"key": "body:1", "name": "rover", "kind": "robot",
                   "parent": None},
                  {"key": "body:2", "name": "wheel_left", "kind": "link",
                   "parent": "body:1"},
                  {"key": "body:3", "name": "crate", "kind": "solid",
                   "parent": None}]
        _w(d / "trajectory.json", {
            "csv": "trajectory.csv", "n_samples": n, "dt_s": dt,
            "model_timestep_s": 0.002, "recorded_s": dt * (n - 1),
            "complete": True, "bodies": bodies, "bodies_truncated": False,
            "body_cap": 256})
        rows = ["t,a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z"]
        for i in range(n):
            t = i * dt
            rows.append(",".join("%g" % v for v in (
                t, i * 0.1, 0.0, 0.2, i * 0.1, 0.25, 0.15, 2.0, 0.0, 0.2)))
        (d / "trajectory.csv").write_text("\n".join(rows) + "\n",
                                          encoding="utf-8")
    _w(d / "completion.json", {
        "complete": True, "quit_called": True, "stopped_by": "duration",
        "recorded_s": dt * (n - 1), "steps": n, "dt_s": dt,
        "driver": "scene.py", "driver_error": None, "hook_intact": True,
        "tamper": [], "warnings": {}, "notes": [],
    } if completion is None else completion)
    _w(d / "process.json", {
        "exit_code": 0, "timed_out": False, "wall_s": 0.4,
        "attempts_used": 1, "python": "py", "mujoco_version": "3.8.1",
        "model": "scene.xml", "driver": "scene.py",
        "driver_rule": "<model stem>.py beside the model",
    } if process is None else process)
    _w(d / "model_info.json", {
        "mujoco_version": "3.8.1", "mujoco_python_version": "3.8.1",
        "model_name": "scene", "solver": "mjSOL_NEWTON",
        "integrator": "mjINT_IMPLICITFAST", "cone": "mjCONE_PYRAMIDAL",
        "jacobian": "mjJAC_AUTO", "timestep_s": 0.002, "iterations": 100,
        "gravity": [0.0, 0.0, -9.81], "impratio": 1.0, "nbody": 4,
        "ngeom": 5, "njnt": 4, "nu": 2, "observed": True,
    } if model_info is None else model_info)
    _w(d / "model_load.json", {
        "compiled": True, "error": None, "model": "scene.xml",
    } if model_load is None else model_load)
    (d / "stdout.log").write_text("", encoding="utf-8")
    return d


def _w(p, doc):
    Path(p).write_text(json.dumps(doc, indent=1), encoding="utf-8")


def _bundle(d, **kw):
    kw.setdefault("artifact", str(Path(d) / "scene.xml"))
    return evidence.build_bundle("R1_lidar_nav", run_dir=d, **kw)


# --- the contract ------------------------------------------------------------


def test_a_complete_run_satisfies_the_adapter_contract(tmp_path):
    write_run(tmp_path)
    assert adapters.check_bundle(_bundle(tmp_path)) == []


def test_the_adapter_is_registered_and_resolvable():
    mod = adapters.resolve("mujoco")
    assert mod.SIM == "mujoco"
    assert hasattr(mod, "build_bundle")


def test_an_empty_directory_answers_nothing_rather_than_zero(tmp_path):
    b = evidence.build_bundle("R1_lidar_nav", run_dir=tmp_path)
    assert b.contacts is None          # never an empty "we looked" result
    assert b.trajectory is None
    assert b.attribution is None
    assert b.roster.bodies == []
    assert b.roster.error
    assert any("no MuJoCo run artifacts" in n for n in b.notes)


# --- what is a robot ---------------------------------------------------------


def test_only_the_driven_top_level_body_is_robot_class(tmp_path):
    write_run(tmp_path)
    b = _bundle(tmp_path)
    assert [r.name for r in b.roster.robots] == ["rover"]
    assert b.roster.by_name("crate").robot_class is False
    assert b.roster.by_name("floor").robot_class is False


def test_a_link_is_never_a_robot_and_carries_its_membership(tmp_path):
    write_run(tmp_path)
    b = _bundle(tmp_path)
    # The roster is what the cores count robots in; a link is not in it.
    assert b.roster.by_name("wheel_left") is None
    link = b.t0.by_name("wheel_left")
    assert link.robot_class is False
    assert link.member_of == "body:1"
    assert [x.body_id for x in b.t0.links_of("body:1")] == ["body:2"]


def test_a_body_driven_only_by_an_observed_force_is_still_a_robot(tmp_path):
    """The model declared no actuator; the RUN showed it being pushed.

    Reading only the file would call it scenery, which is reading the
    declaration instead of the measurement.
    """
    r = _roster(robot_roots=[3], actuated_roots={},
                forced_roots={"3": ["xfrc_applied was written on a body of "
                                    "this subtree"]})
    write_run(tmp_path, roster=r)
    b = _bundle(tmp_path)
    assert [x.name for x in b.roster.robots] == ["crate"]
    assert any("OBSERVED applied force" in n for n in b.notes)


def test_husky_identity_without_a_declared_analogue_is_a_definite_no(tmp_path):
    write_run(tmp_path)
    (tmp_path / "scene.xml").write_text(
        '<mujoco><worldbody><body name="rover"/></worldbody></mujoco>',
        encoding="utf-8")
    b = evidence.build_bundle("A1_husky_swarm_10", robot_identity="husky",
                              artifact=str(tmp_path / "scene.xml"),
                              run_dir=tmp_path)
    assert b.roster.robots == []
    assert b.identity.declared_count == 0
    assert any("ships a Clearpath Husky" in n for n in b.notes)


def test_a_declared_analogue_is_accepted_and_published(tmp_path):
    r = _roster()
    r["bodies"][0]["n_joints"] = 4          # a four-wheeled UGV
    write_run(tmp_path, roster=r)
    (tmp_path / "scene.xml").write_text(
        '<mujoco><worldbody><body name="rover"/></worldbody></mujoco>',
        encoding="utf-8")
    b = evidence.build_bundle("A1_husky_swarm_10", robot_identity="husky",
                              robot_model="rover",
                              artifact=str(tmp_path / "scene.xml"),
                              run_dir=tmp_path)
    assert [x.name for x in b.roster.robots] == ["rover"]
    assert b.identity.declared_count == 1
    assert "declared analogue" in b.identity.label
    assert "DECLARED ANALOGUE" in b.identity.scene_rule


def test_the_flagship_joint_floor_is_the_same_number_on_every_arm(tmp_path):
    """A two-joint rover is not the four-wheeled UGV A1 asks for.

    ``MIN_JOINTS`` is 4 on all three adapters on purpose: the identity rules
    must differ only in the channel that carries the model's identity, never in
    how much robot the task requires.
    """
    assert evidence.MIN_JOINTS == 4
    write_run(tmp_path)                     # the default rover has 2 joints
    (tmp_path / "scene.xml").write_text(
        '<mujoco><worldbody><body name="rover"/></worldbody></mujoco>',
        encoding="utf-8")
    b = evidence.build_bundle("A1_husky_swarm_10", robot_identity="husky",
                              robot_model="rover",
                              artifact=str(tmp_path / "scene.xml"),
                              run_dir=tmp_path)
    assert b.roster.robots == []


# --- the scene inventory -----------------------------------------------------


def test_worldbody_geoms_are_independent_objects_with_their_own_bounds(
        tmp_path):
    """Five obstacle geoms on <worldbody> must not become one 'world' body."""
    write_run(tmp_path)
    b = _bundle(tmp_path)
    wall = b.roster.by_name("wall_north")
    assert wall.kind == "world_geom"
    assert wall.has_aabb and wall.dynamic is False
    assert wall.aabb_min == (-3.0, 2.9, 0.0)
    assert b.roster.by_name("floor") is not b.roster.by_name("wall_north")


def test_a_plane_publishes_the_infinite_collision_surface_caveat(tmp_path):
    write_run(tmp_path, roster=_roster(plane_notes=[
        "geom 'floor': a MuJoCo plane geom is INFINITE for collision; this "
        "AABB is its DRAWN extent (geom_size 5 x 5 m)"]))
    b = _bundle(tmp_path)
    assert any("INFINITE for collision" in n for n in b.notes)


def test_a_scan_taken_after_t0_is_refused_rather_than_believed(tmp_path):
    write_run(tmp_path, roster=_roster(frozen=True, t_s=1.5))
    b = _bundle(tmp_path)
    assert b.t0.frozen is False
    assert any("claim is refused" in n for n in b.notes)
    # ...and the contract checker then says the t=0 questions are ungradeable.
    assert any("not marked frozen" in p
               for p in adapters.check_bundle(b))


# --- contacts ----------------------------------------------------------------


def test_contacts_carry_both_vacuity_counters(tmp_path):
    write_run(tmp_path)
    c = _bundle(tmp_path).contacts
    assert c.supported and c.total_observed == 12 and c.distinct_named == 12
    assert c.can_name_a_robot_pair is True


def test_a_zero_witness_survives_as_zero_and_a_missing_one_as_none(tmp_path):
    write_run(tmp_path, contacts={"supported": True, "steps": 3,
                                  "total_observed": 0, "pairs": []})
    c = _bundle(tmp_path).contacts
    assert c.total_observed == 0        # a real count
    assert c.distinct_named is None     # never counted -> never 0
    assert c.can_name_a_robot_pair is False


def test_the_robot_side_of_a_pair_is_marked_from_the_subtree_root(tmp_path):
    write_run(tmp_path)
    p = _bundle(tmp_path).contacts.pairs[0]
    assert (p.a, p.b) == ("floor", "rover")
    assert p.a_robot is False and p.b_robot is True
    assert p.distinct is True


def test_a_contact_stride_is_published_as_a_bias_against_detection(tmp_path):
    write_run(tmp_path, contacts={"supported": True, "steps": 3, "stride": 10,
                                  "total_observed": 1, "distinct_named": 1,
                                  "pairs": []})
    b = _bundle(tmp_path)
    assert any("against detecting a hit" in n for n in b.notes)


# --- trajectory --------------------------------------------------------------


def test_trajectory_rows_are_keyed_the_same_way_the_inventory_is(tmp_path):
    write_run(tmp_path, n=6)
    b = _bundle(tmp_path)
    t = b.trajectory
    assert t.body_ids == ["body:1", "body:2", "body:3"]
    assert t.kinds == ["robot", "link", "solid"]
    assert t.parents == [None, "body:1", None]
    rover = b.roster.by_name("rover")
    assert t.index_of(rover.body_id) == 0
    assert t.n_samples == 6 and t.n_top_level == 2


def test_a_column_map_longer_than_the_csv_truncates_and_says_so(tmp_path):
    write_run(tmp_path, n=4)
    doc = json.loads((tmp_path / "trajectory.json").read_text())
    doc["bodies"].append({"key": "body:9", "name": "ghost", "kind": "solid"})
    _w(tmp_path / "trajectory.json", doc)
    b = _bundle(tmp_path)
    assert b.trajectory.n_bodies == 3
    assert any("no row is silently re-labelled" in n for n in b.notes)


def test_a_downsampled_series_is_published_with_the_bias_direction(tmp_path):
    write_run(tmp_path, n=5, dt=0.02)      # 10x the model timestep
    b = _bundle(tmp_path)
    assert any("DOWNSAMPLED" in n for n in b.notes)


def test_a_missing_csv_is_a_motion_error_not_an_empty_trajectory(tmp_path):
    write_run(tmp_path)
    (tmp_path / "trajectory.csv").unlink()
    b = _bundle(tmp_path)
    assert b.trajectory.xyz is None
    assert "CSV is not" in (b.motion_error or "")


# --- process, errors and attribution -----------------------------------------


def test_a_clean_run_reaches_finalize_through_compile_plus_step(tmp_path):
    write_run(tmp_path)
    p = _bundle(tmp_path).process
    assert p.clean and p.driver_completed is True
    assert p.reached_finalize is True
    assert "compiled into an MjModel" in p.finalize_evidence
    assert p.behaviour_starts == {"scene.py": 1}
    assert p.start_source


def test_a_compile_failure_is_an_error_line_and_a_failed_load(tmp_path):
    write_run(tmp_path, model_load={"compiled": False,
                                    "error": "XML Error: invalid keyword"})
    b = _bundle(tmp_path)
    assert b.live_load_ok is False
    assert any("MJCF compile error" in ln for ln in b.process.error_lines)
    assert any("did NOT compile" in n for n in b.notes)


def test_a_divergence_warning_is_recorded_but_never_counted_as_an_error(
        tmp_path):
    """Counting mjWARN_* would make this assertion stricter on MuJoCo.

    Neither the OmniSim nor the Webots adapter counts its engine's divergence
    warnings, and a comparison whose thresholds lean towards our own simulator
    is not evidence of anything.
    """
    write_run(tmp_path, completion={
        "complete": True, "quit_called": True, "stopped_by": "duration",
        "recorded_s": 0.008, "steps": 5, "dt_s": 0.002, "driver": "scene.py",
        "driver_error": None, "hook_intact": True, "tamper": [],
        "warnings": {"mjWARN_BADQACC": 3}, "notes": []})
    b = _bundle(tmp_path)
    assert b.process.error_lines == []
    assert any("RECORDED and NOT counted" in n for n in b.notes)
    assert b.adapter_measurements["motion"]["mujoco_warnings"] == {
        "mjWARN_BADQACC": 3}


def test_a_driver_traceback_is_not_an_engine_error_line(tmp_path):
    write_run(tmp_path, completion={
        "complete": False, "quit_called": False, "stopped_by": "driver_raised",
        "recorded_s": 0.0, "steps": 0, "driver": "scene.py",
        "driver_error": "ZeroDivisionError: division by zero",
        "hook_intact": True, "tamper": [], "warnings": {}, "notes": []},
        process={"exit_code": 1, "timed_out": False, "wall_s": 0.2,
                 "attempts_used": 1, "driver": "scene.py"})
    b = _bundle(tmp_path)
    assert b.process.error_lines == []      # the agent's program, not MuJoCo
    assert b.process.clean is False         # ...and it still fails, on exit 1
    assert b.process.driver_completed is False
    assert "never called mj_step" in b.process.finalize_evidence \
        or "raised" in b.process.finalize_evidence


def test_a_mujoco_fatal_error_IS_an_engine_error_line(tmp_path):
    write_run(tmp_path, completion={
        "complete": False, "quit_called": False, "stopped_by": "driver_raised",
        "recorded_s": 0.0, "steps": 2, "driver": "scene.py",
        "driver_error": "FatalError: mj_stackAlloc: out of memory",
        "hook_intact": True, "tamper": [], "warnings": {}, "notes": []})
    b = _bundle(tmp_path)
    assert any("MuJoCo engine error" in ln for ln in b.process.error_lines)


def test_attribution_names_the_solver_and_cites_the_compiled_model(tmp_path):
    write_run(tmp_path)
    a = _bundle(tmp_path).attribution
    assert a.backend == "mujoco"
    assert a.solver == "mjSOL_NEWTON"
    assert a.extra["integrator"] == "mjINT_IMPLICITFAST"
    assert a.extra["cone"] == "mjCONE_PYRAMIDAL"
    assert "COMPILED MjModel" in a.source


def test_a_run_that_never_compiled_a_model_is_unattributed(tmp_path):
    write_run(tmp_path, model_info={"observed": False},
              completion={"complete": False, "steps": 0, "driver": "scene.py",
                          "warnings": {}, "notes": [], "tamper": []})
    b = _bundle(tmp_path)
    assert b.attribution is None
    assert any("stays unattributed" in n for n in b.notes)


def test_tampering_with_the_step_hook_is_published(tmp_path):
    write_run(tmp_path, completion={
        "complete": True, "quit_called": True, "stopped_by": "duration",
        "recorded_s": 0.008, "steps": 5, "driver": "scene.py",
        "hook_intact": False, "warnings": {}, "notes": [],
        "tamper": ["the driver replaced mujoco.mj_step during the run"]})
    b = _bundle(tmp_path)
    assert any(n.startswith("TAMPER:") for n in b.notes)


# --- frame -------------------------------------------------------------------


def test_a_non_z_gravity_is_stated_loudly_and_never_silently_remapped(
        tmp_path):
    write_run(tmp_path, model_info={
        "solver": "mjSOL_NEWTON", "integrator": "mjINT_EULER",
        "cone": "mjCONE_PYRAMIDAL", "jacobian": "mjJAC_AUTO",
        "timestep_s": 0.002, "iterations": 100, "gravity": [0.0, -9.81, 0.0],
        "impratio": 1.0, "observed": True, "mujoco_version": "3.8.1"})
    b = _bundle(tmp_path)
    assert any("measuring the wrong axis" in n for n in b.notes)


def test_zero_gravity_is_named_as_a_world_where_falling_has_no_meaning(
        tmp_path):
    write_run(tmp_path, model_info={
        "solver": "mjSOL_NEWTON", "gravity": [0.0, 0.0, 0.0],
        "timestep_s": 0.002, "observed": True})
    b = _bundle(tmp_path)
    assert any("ZERO gravity" in n for n in b.notes)


# --- the deliverable ---------------------------------------------------------


def test_the_artifact_is_the_newest_non_injected_xml(tmp_path):
    import os
    import time
    old = tmp_path / "old.xml"
    new = tmp_path / "new.xml"
    injected = tmp_path / "_agentbench_new.xml"
    for p in (old, new, injected):
        p.write_text("<mujoco/>", encoding="utf-8")
    now = time.time()
    os.utime(old, (now - 100, now - 100))
    os.utime(injected, (now + 100, now + 100))
    assert evidence.pick_model(tmp_path) == new


def test_no_xml_anywhere_means_no_artifact(tmp_path):
    assert evidence.pick_model(tmp_path) is None


def test_scan_mjcf_reads_a_malformed_file_rather_than_throwing(tmp_path):
    """Whether the file is valid is answered by the COMPILER, not by a parser
    here -- and the declaration count must still exist when it is not."""
    p = tmp_path / "broken.xml"
    p.write_text('<mujoco><worldbody><body name="a"><geom </worldbody>',
                 encoding="utf-8")
    scanned = evidence.scan_mjcf(p)
    assert scanned["body_names"] == ["a"]


# --- the camera channel ------------------------------------------------------


def test_the_camera_pose_is_converted_out_of_mujocos_frame(tmp_path):
    write_run(tmp_path, roster=_roster(cameras=[
        {"name": "overview", "id": 0, "position": [-3.0, 0.0, 2.0],
         "forward": [1.0, 0.0, 0.0], "up": [0.0, 0.0, 1.0],
         "fovy_deg": 60.0, "resolution": [1280, 720]}]))
    ev = evidence.build_view_evidence("B2_subject_in_frame", run_dir=tmp_path)
    assert ev.final.position == (-3.0, 0.0, 2.0)
    assert ev.final.forward == (1.0, 0.0, 0.0)
    assert abs(ev.final.fov_v_rad - 1.0471975) < 1e-5
    assert abs(ev.final.aspect - 1280 / 720.0) < 1e-9
    # ...and the FULL horizontal angle is DERIVED, with the derivation named.
    _h, _v, how = ev.final.half_angles()
    assert "derived from the aspect ratio" in how


def test_a_model_with_no_camera_reports_no_pose_rather_than_a_default(
        tmp_path):
    write_run(tmp_path)
    ev = evidence.build_view_evidence("B2_subject_in_frame", run_dir=tmp_path)
    assert ev.final is None
    assert "no <camera>" in ev.error


def test_b2_declares_the_missing_initial_fixture_rather_than_inventing_one(
        tmp_path):
    write_run(tmp_path, roster=_roster(cameras=[
        {"name": "c", "id": 0, "position": [0, 0, 1], "forward": [1, 0, 0],
         "up": [0, 0, 1], "fovy_deg": 45.0, "resolution": None}]))
    ev = evidence.build_view_evidence("B2_subject_in_frame", run_dir=tmp_path)
    assert ev.initial is None
    assert "missing FIXTURE, not a missing MuJoCo capability" in ev.error


# --- reading a run off disk ---------------------------------------------------


def test_read_run_reports_what_is_missing_instead_of_raising(tmp_path):
    run = recording.read_run(tmp_path)
    assert run.any_evidence is False
    assert set(recording.CANDIDATES) <= set(run.missing)
    assert run.compiled is None


def test_read_run_records_a_parse_error_rather_than_dropping_the_file(
        tmp_path):
    write_run(tmp_path)
    (tmp_path / "contacts.json").write_text("{not json", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.contacts is None
    assert "not valid JSON" in run.errors["contacts"]
