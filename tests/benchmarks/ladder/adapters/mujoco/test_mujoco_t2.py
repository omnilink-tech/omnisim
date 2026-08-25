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

"""Unit tests for the MuJoCo column's **T2** half. Most need no simulator.

Run::

    pytest tests/benchmarks/ladder/adapters/mujoco/test_mujoco_t2.py -q

The split is the sibling file's and for the sibling file's reason: a bug in the
scene builder produces an arm that does not move and is obvious, while a bug in
``t2_channels`` produces a **verdict** that is wrong and looks fine. So the
mapping from what the sampler wrote to what the sim-neutral core reads is
tested here on synthetic documents, with no MuJoCo involved at all.

Four of these are **tripwires on the findings** rather than tests of our code
(the shape ``test_a_raw_load_of_the_container_urdf_fails_on_package_uris`` has
in the T1 file): if a future MuJoCo changes its URDF defaults, its parent
contact filter or its ``<default/>`` round trip, they fail -- which is the
correct way to learn that ``t2_scene``'s docstring has stopped being true.
"""

from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder import adapters as ladder_adapters                   # noqa: E402
from ladder import tasks as ladder_tasks                         # noqa: E402
from ladder.adapters.mujoco import (evidence, recording,         # noqa: E402
                                    runner_t2, t2_drive, t2_scene)
from ladder.graders import t2 as t2_grader                       # noqa: E402
from ladder.graders import t2_core                               # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
CONTAINER = REPO / "tests/benchmarks/ladder/tasks/T2_transfer/container"
ARM_URDF = CONTAINER / "bench_arm/urdf/bench_arm.urdf"
BLOCK_URDF = CONTAINER / "props/urdf/block.urdf"
BIN_URDF = CONTAINER / "props/urdf/bin.urdf"


def _mujoco():
    return pytest.importorskip("mujoco",
                               reason="the mujoco wheel is not installed")


# --- t2_scene: reading the shipped descriptions ------------------------------


def test_layout_puts_the_object_and_the_container_at_one_radius():
    lay = t2_scene.layout(0.44)
    bx, by = lay["block_xy_m"]
    cx, cy = lay["bin_xy_m"]
    assert math.hypot(bx, by) == pytest.approx(0.44)
    assert math.hypot(cx, cy) == pytest.approx(0.44)
    # 90 degrees apart, so the transfer is a yaw of the first axis -- which is
    # what puts a real rotation inside T2.3's frame-relative hold.
    assert abs(math.atan2(cy, cx) - math.atan2(by, bx)) == pytest.approx(
        math.pi / 2, abs=1e-9)


def test_the_object_half_height_is_read_off_its_own_description():
    assert t2_scene._block_half_height(BLOCK_URDF) == pytest.approx(0.025)


def test_actuator_force_limits_come_from_the_urdfs_own_effort_tags():
    eff = t2_scene._urdf_efforts(ARM_URDF)
    assert eff["joint_1"] == 150.0
    assert eff["joint_3"] == 90.0
    assert eff["joint_6"] == 20.0
    assert eff["finger_joint_left"] == 80.0
    # Every joint the scene actuates must have a declared effort, or an
    # actuator would silently be stronger than the description allows.
    for jn in t2_scene.ARM_JOINTS + t2_scene.FINGER_JOINTS:
        assert jn in eff


def test_prepare_urdf_never_touches_the_shipped_file(tmp_path):
    before = ARM_URDF.read_bytes()
    out = t2_scene.prepare_urdf(ARM_URDF, tmp_path / "arm.urdf")
    assert ARM_URDF.read_bytes() == before
    text = out.read_text(encoding="utf-8")
    assert 'fusestatic="false"' in text
    assert "<mujoco>" in text


def test_prepare_urdf_can_add_a_floating_joint_and_stays_valid_xml(tmp_path):
    out = t2_scene.prepare_urdf(BLOCK_URDF, tmp_path / "block.urdf",
                                floating_base="block")
    root = ET.parse(out).getroot()
    joints = [j for j in root.findall("joint") if j.get("type") == "floating"]
    assert len(joints) == 1
    assert joints[0].find("child").get("link") == "block"


def test_sanitise_mjcf_strips_only_the_nameless_empty_defaults():
    text = ('<mujoco><default><default /><default class="keep" />'
            '<default><geom /></default></default></mujoco>')
    out, n = t2_scene.sanitise_mjcf(text)
    assert n == 1
    assert 'class="keep"' in out
    assert "<geom" in out


def test_sanitise_mjcf_is_a_no_op_when_there_is_nothing_to_strip():
    text = '<mujoco><worldbody><body name="a" /></worldbody></mujoco>'
    out, n = t2_scene.sanitise_mjcf(text)
    assert (out, n) == (text, 0)


# --- t2_drive: the control law, with no simulator ----------------------------

_ST = {"a1": 0.35, "a2": 0.38, "a3": 0.07, "shoulder_z": 0.25,
       "limits": {"joint_1": (-2.9, 2.9), "joint_2": (-2.2, 2.2),
                  "joint_3": (-2.6, 2.6), "joint_4": (-2.9, 2.9),
                  "joint_5": (-2.0, 2.0), "joint_6": (-3.0, 3.0)}}


def _fk(st, q):
    """``(radius, height, tool pitch)`` of the tool origin, in the base frame."""
    p1 = q["joint_2"]
    p2 = p1 + q["joint_3"]
    p3 = p2 + q["joint_5"]
    u = (st["a1"] * math.sin(p1) + st["a2"] * math.sin(p2)
         + st["a3"] * math.sin(p3))
    w = (st["shoulder_z"] + st["a1"] * math.cos(p1) + st["a2"] * math.cos(p2)
         + st["a3"] * math.cos(p3))
    return u, w, p3


def test_ik_lands_on_the_commanded_pose_with_the_tool_pointing_down():
    for radius, height in ((0.44, 0.523), (0.44, 0.648), (0.44, 0.158),
                           (0.30, 0.45)):
        q = t2_drive.solve_ik(_ST, radius, height, 0.7)
        assert q is not None, (radius, height)
        u, w, pitch = _fk(_ST, q)
        assert u == pytest.approx(radius, abs=1e-9)
        assert w == pytest.approx(height, abs=1e-9)
        assert pitch == pytest.approx(math.pi, abs=1e-9)
        assert q["joint_1"] == pytest.approx(0.7)
        assert q["joint_4"] == 0.0 and q["joint_6"] == 0.0


def test_ik_refuses_a_pose_beyond_reach():
    assert t2_drive.solve_ik(_ST, 1.5, 0.5, 0.0) is None


def test_the_wrist_range_binds_before_reach_does():
    """The finding ``t2_scene`` records: a straight-down grasp needs a FOLD.

    ``joint_5`` is limited to +/-2.0 and the three pitches must sum to pi, so
    ``joint_2 + joint_3 >= 1.14 rad``. At 0.50 m and the pre-grasp height this
    driver uses there is no solution -- while the pose is comfortably inside
    the 0.73 m the two long links can reach. That is a property of the shipped
    description and it is why the layout works at 0.44 m.
    """
    reach = _ST["a1"] + _ST["a2"] + _ST["a3"]
    radius, height = 0.50, 0.653
    span = math.hypot(radius, height + _ST["a3"] - _ST["shoulder_z"])
    assert span < reach                       # not a reach problem
    assert t2_drive.solve_ik(_ST, radius, height, 0.0) is None
    assert t2_drive.solve_ik(_ST, 0.44, height, 0.0) is not None


def _drive_state():
    st = dict(_ST)
    st.update({"object_start": [0.44, 0.0, 0.425], "object_half": [0.025] * 3,
               "tool_to_object": 0.098, "grip_open": 0.0, "grip_close": 0.024,
               "table_top_z": 0.40, "bin_floor_top_z": 0.02,
               "object_radius": 0.44, "object_bearing": 0.0,
               "bin_radius": 0.44, "bin_bearing": math.pi / 2})
    st["waypoints"] = t2_drive.waypoints(st)
    return st


def test_the_schedule_starts_and_ends_where_the_waypoints_say():
    st = _drive_state()
    assert t2_drive.target_at(st, 0.0)[:4] == pytest.approx(
        st["waypoints"]["start"])
    end = t2_drive.target_at(st, t2_drive.DURATION_S + 5.0)
    assert end[4] == "done"
    assert end[:4] == pytest.approx(st["waypoints"]["stand"])


def test_the_schedule_is_continuous_across_every_phase_boundary():
    st = _drive_state()
    t = 0.0
    for name, dur in t2_drive.PHASE_S:
        t += dur
        before = np.asarray(t2_drive.target_at(st, t - 1e-6)[:4], dtype=float)
        after = np.asarray(t2_drive.target_at(st, t + 1e-6)[:4], dtype=float)
        assert np.allclose(before, after, atol=1e-4), name


def test_the_jaws_stay_closed_for_longer_than_the_tier_asks():
    """T2.3 asks for 10.0 s. The schedule must not be cutting that fine."""
    st = _drive_state()
    ts = np.arange(0.0, t2_drive.DURATION_S, 0.01)
    closed = np.array([t2_drive.target_at(st, float(t))[3] for t in ts])
    held = ts[closed >= st["grip_close"] - 1e-9]
    assert len(held), "the jaws never reach the closed command"
    assert float(held.max() - held.min()) > t2_core.HOLD_S


def test_the_carry_lifts_the_object_well_clear_of_the_tier_threshold():
    """T2.2 asks for 0.05 m; the plan is not decided by servo sag."""
    st = _drive_state()
    carry_tool_z = st["waypoints"]["steady"][1]
    lowest = carry_tool_z - st["tool_to_object"] - st["object_half"][2]
    assert lowest - st["table_top_z"] > 2.0 * t2_core.LIFT_CLEARANCE_M


def test_the_release_pose_puts_the_object_under_the_container_rim():
    st = _drive_state()
    bin_tool_z = st["waypoints"]["release"][1]
    lowest = bin_tool_z - st["tool_to_object"] - st["object_half"][2]
    assert lowest > st["bin_floor_top_z"]        # not through the floor
    assert lowest < 0.16                          # under the shipped rim


# --- runner_t2: the deliverable contract -------------------------------------


def test_a_deliverable_directory_resolves_to_its_scene_and_its_driver(tmp_path):
    (tmp_path / runner_t2.SCENE_NAME).write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / runner_t2.DRIVER_NAME).write_text("", encoding="utf-8")
    scene, driver = runner_t2.resolve_deliverable(tmp_path)
    assert scene == tmp_path / runner_t2.SCENE_NAME
    assert driver == tmp_path / runner_t2.DRIVER_NAME


def test_a_bare_scene_file_finds_its_sibling_driver(tmp_path):
    scene_p = tmp_path / "world.xml"
    scene_p.write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / runner_t2.DRIVER_NAME).write_text("", encoding="utf-8")
    scene, driver = runner_t2.resolve_deliverable(scene_p)
    assert (scene, driver) == (scene_p, tmp_path / runner_t2.DRIVER_NAME)


def test_a_deliverable_with_no_driver_is_a_measurement_not_an_error(tmp_path):
    (tmp_path / runner_t2.SCENE_NAME).write_text("<mujoco/>", encoding="utf-8")
    scene, driver = runner_t2.resolve_deliverable(tmp_path)
    assert scene is not None and driver is None


def test_an_empty_deliverable_resolves_to_nothing(tmp_path):
    assert runner_t2.resolve_deliverable(tmp_path) == (None, None)
    assert runner_t2.resolve_deliverable(tmp_path / "nope") == (None, None)


def test_the_driver_is_imported_by_path_and_never_put_on_sys_path(tmp_path):
    p = tmp_path / runner_t2.DRIVER_NAME
    p.write_text("MARK = 41\ndef control(m, d, t):\n    return t\n",
                 encoding="utf-8")
    before = list(sys.path)
    mod = runner_t2.load_driver(p)
    assert mod.MARK == 41
    assert sys.path == before
    assert str(tmp_path) not in sys.path
    assert runner_t2.load_driver(None) is None


def test_the_roles_come_from_the_task_file_and_not_from_this_adapter():
    roles = runner_t2._roles_from_task()
    declared = ladder_tasks.get("T2_transfer").roles
    assert roles["object"] == declared["object"] == "block"
    assert roles["end_effector"] == declared["end_effector"] == "tool"
    assert roles["container"] == declared["container"] == "bin"


def test_run_standalone_on_an_empty_deliverable_reports_rather_than_raises(
        tmp_path):
    res = runner_t2.run_standalone(tmp_path / "nothing", tmp_path / "run")
    assert res.rc is None
    assert "nothing to re-run" in res.error
    assert isinstance(res.as_dict(), dict)


# --- evidence.t2_channels: the mapping, on synthetic documents ---------------


def _t2_doc(**over):
    n = 12
    t = [round(0.01 * i, 4) for i in range(n)]
    eye = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    doc = {
        "roles": {"object": "block", "end_effector": "tool",
                  "container": "bin"},
        "t_s": t,
        "object": {"body": "block", "xyz": [[0.4, 0.0, 0.5]] * n,
                   "rot": [eye] * n, "source": "a synthetic sampler"},
        "end_effector": {"body": "tool", "xyz": [[0.4, 0.0, 0.6]] * n,
                         "rot": [eye] * n, "source": "a synthetic sampler"},
        "container": {"body": "bin", "aabb_min": [-0.15, 0.29, 0.0],
                      "aabb_max": [0.15, 0.59, 0.16], "rim_z": 0.16,
                      "rim_rule": runner_t2.RIM_RULE, "t_s": 0.0,
                      "subtree_bodies": ["bin", "bin_wall_x_pos"]},
        "support_surfaces": [
            {"body": "ground", "aabb_min": [-3, -3, -0.1],
             "aabb_max": [3, 3, 0.0], "static": True},
            {"body": "table", "aabb_min": [0.24, -0.22, 0.0],
             "aabb_max": [0.64, 0.22, 0.4], "static": True}],
        "support_surfaces_source": runner_t2.STATIC_SOURCE,
        "object_physics": {"body": "block", "mass_kg": 0.2, "dynamic": True},
        "world": {"gravity_mps2": 9.81, "gravity_vec_mps2": [0, 0, -9.81],
                  "source": "mjModel.opt.gravity"},
        "object_aabb": {"min": [0.375, -0.025, 0.475],
                        "max": [0.425, 0.025, 0.525]},
        "grip": {"mechanism": "friction", "attachment": False,
                 "equality_constraints_in_the_model": 0, "supported": True,
                 "steps": 100, "window_s": 0.11, "total_observed": 40,
                 "distinct_named": 40,
                 "contacts": [{"holder_body": "finger_left",
                               "held_body": "block", "point": [0.4, 0.02, 0.5],
                               "t_s": 0.05, "step": 25}],
                 "source": runner_t2.GRIP_SOURCE},
    }
    doc.update(over)
    return doc


def _run_with(doc, tmp_path):
    (tmp_path / "t2.json").write_text(json.dumps(doc), encoding="utf-8")
    return recording.read_run(tmp_path)


def test_no_run_means_no_channels_so_the_shim_reports_the_gap():
    assert evidence.t2_channels(None) == {}
    assert evidence.t2_channels("/definitely/not/a/run") == {}


def test_every_t2_channel_is_answered_from_the_samplers_own_document(tmp_path):
    ch = evidence.t2_channels(_run_with(_t2_doc(), tmp_path))
    assert set(ch) == set(evidence.T2_CHANNEL_KEYS)
    assert ch["object_pose"].body == "block"
    assert ch["end_effector"].body == "tool"
    assert ch["container"].body == "bin"
    assert ch["object_physics"].mass_kg == 0.2
    assert ch["world"].gravity_mps2 == 9.81
    assert len(ch["support_surfaces"]) == 2
    assert ch["object_aabb"][0] == (0.375, -0.025, 0.475)


def test_the_end_effector_series_carries_a_rotation_matrix_per_sample(tmp_path):
    ch = evidence.t2_channels(_run_with(_t2_doc(), tmp_path))
    ee = ch["end_effector"]
    assert ee.has_orientation
    assert np.asarray(ee.rot).shape == (ee.n_samples, 3, 3)
    # The two series must be on ONE clock or the frame computation is
    # meaningless; the sampler writes both from the same loop.
    _idx, skew = ee.align_to(ch["object_pose"].t)
    assert skew == pytest.approx(0.0)


def test_a_ragged_series_is_truncated_and_says_so(tmp_path):
    doc = _t2_doc()
    doc["object"]["xyz"] = doc["object"]["xyz"][:5]
    ch = evidence.t2_channels(_run_with(doc, tmp_path))
    assert ch["object_pose"].n_samples == 5
    assert len(np.asarray(ch["object_pose"].t)) == 5


def test_an_attested_rim_survives_and_an_absent_one_stays_absent(tmp_path):
    ch = evidence.t2_channels(_run_with(_t2_doc(), tmp_path))
    assert ch["container"].rim_attested is True
    assert ch["container"].rim_height == pytest.approx(0.16)

    doc = _t2_doc()
    doc["container"]["rim_z"] = None
    ch = evidence.t2_channels(_run_with(doc, tmp_path))
    assert ch["container"].rim_attested is False
    # The permissive fallback: the box top, and the row says which was used.
    assert ch["container"].rim_height == pytest.approx(0.16)
    assert "permissive" in ch["container"].rim_source


def test_the_support_surfaces_are_attested_static_and_carry_a_citation(
        tmp_path):
    ch = evidence.t2_channels(_run_with(_t2_doc(), tmp_path))
    for s in ch["support_surfaces"]:
        assert s.static is True
        assert s.usable
        assert s.source


def test_the_mechanism_is_read_from_the_model_not_declared(tmp_path):
    ch = evidence.t2_channels(_run_with(_t2_doc(), tmp_path))
    assert ch["grip"].recorded_mechanism == "friction"
    assert ch["grip"].attachment is False
    assert ch["grip"].holder_bodies == ["finger_left"]

    doc = _t2_doc()
    doc["grip"]["mechanism"] = "attachment"
    doc["grip"]["attachment"] = True
    ch = evidence.t2_channels(_run_with(doc, tmp_path))
    assert ch["grip"].recorded_mechanism == "attachment"
    assert ch["grip"].attachment is True


def test_t2_channels_never_raises_on_a_broken_document(tmp_path):
    for doc in ({}, {"t_s": "nonsense"}, {"object": 4, "t_s": [0.0, 0.1]},
                {"t_s": [0.0, 0.1], "object": {"body": "b", "xyz": [["x"]]}},
                {"t_s": [0.0], "container": {"body": "bin"}}):
        ch = evidence.t2_channels(_run_with(doc, tmp_path))
        assert isinstance(ch, dict)


def test_a_run_with_channels_reaches_the_core_through_the_real_shim(tmp_path):
    """The whole seam: sampler document -> shim -> T2Evidence, unanswered = 0."""
    _run_with(_t2_doc(), tmp_path)
    (tmp_path / "completion.json").write_text(json.dumps(
        {"complete": True, "recorded_s": 0.11, "steps": 100,
         "ctrl_writes": 100, "engine": {"library": "mujoco", "version": "3.8.1",
                                        "solver": "Newton"}}), encoding="utf-8")
    ev = ladder_adapters.build_t2_evidence(
        ladder_tasks.get("T2_transfer"), sim="mujoco", run_dir=tmp_path,
        artifact=str(tmp_path / "scene.xml"))
    gaps = ladder_adapters.t2_unanswered_channels(ev)
    assert gaps == {}, gaps
    assert ev.object_half_extent == pytest.approx((0.025, 0.025, 0.025))


# --- the parts that need MuJoCo ---------------------------------------------


OPEN_BOX_XML = """<mujoco>
  <worldbody>
    <body name="bin">
      <geom name="floor" type="box" size="0.15 0.15 0.01" pos="0 0 0.01"/>
      <body name="w0" pos="0.14 0 0.09">
        <geom type="box" size="0.01 0.15 0.07"/></body>
      <body name="w1" pos="-0.14 0 0.09">
        <geom type="box" size="0.01 0.15 0.07"/></body>
      <body name="w2" pos="0 0.14 0.09">
        <geom type="box" size="0.13 0.01 0.07"/></body>
      <body name="w3" pos="0 -0.14 0.09">
        <geom type="box" size="0.13 0.01 0.07"/></body>
    </body>
  </worldbody>
</mujoco>"""


def _loaded(mujoco, tmp_path, xml, name="c.xml"):
    p = tmp_path / name
    p.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_the_rim_rule_reads_the_wall_tops_of_an_open_box(tmp_path):
    """The floor covers the centre line and cannot decide the rim; the walls
    stand clear of it and do. Both are in the container's own subtree."""
    mujoco = _mujoco()
    model, data = _loaded(mujoco, tmp_path, OPEN_BOX_XML)
    got = runner_t2.container_geometry(mujoco, model, data, "bin")
    assert got["aabb_max"][2] == pytest.approx(0.16)
    assert got["rim_z"] == pytest.approx(0.16)
    assert "floor" not in got["rim_from_geoms"]
    assert len(got["subtree_bodies"]) == 5


def test_the_rim_rule_ignores_a_lid_and_reports_the_opening(tmp_path):
    """The reason the rim is not just the top of the bounding box."""
    mujoco = _mujoco()
    xml = """<mujoco>
      <worldbody>
        <body name="bin">
          <geom name="floor" type="box" size="0.15 0.15 0.01" pos="0 0 0.01"/>
          <body name="w0" pos="0.14 0 0.09">
            <geom type="box" size="0.01 0.15 0.07"/></body>
          <body name="w1" pos="-0.14 0 0.09">
            <geom type="box" size="0.01 0.15 0.07"/></body>
          <body name="lid" pos="0 0 0.17">
            <geom type="box" size="0.15 0.15 0.01"/></body>
        </body>
      </worldbody>
    </mujoco>"""
    p = tmp_path / "lidded.xml"
    p.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    got = runner_t2.container_geometry(mujoco, model, data, "bin")
    assert got["aabb_max"][2] == pytest.approx(0.18)   # the lid's top
    assert got["rim_z"] == pytest.approx(0.16)         # the walls' top
    assert "lid" not in " ".join(got["rim_from_geoms"])


def test_a_solid_slab_has_no_rim_and_says_so(tmp_path):
    mujoco = _mujoco()
    xml = ('<mujoco><worldbody><body name="bin">'
           '<geom type="box" size="0.15 0.15 0.08" pos="0 0 0.08"/>'
           '</body></worldbody></mujoco>')
    p = tmp_path / "slab.xml"
    p.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    got = runner_t2.container_geometry(mujoco, model, data, "bin")
    assert got["rim_z"] is None      # -> the core falls back, permissively


def test_support_surfaces_keep_the_welded_and_drop_the_free(tmp_path):
    mujoco = _mujoco()
    xml = ('<mujoco><worldbody>'
           '<body name="ground"><geom type="box" size="2 2 0.05" '
           'pos="0 0 -0.05"/></body>'
           '<body name="crate" pos="0 0 0.5"><freejoint/>'
           '<geom type="box" size="0.05 0.05 0.05"/></body>'
           '</worldbody></mujoco>')
    p = tmp_path / "s.xml"
    p.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = [s["body"] for s in
             runner_t2.support_surfaces(mujoco, model, data, exclude=set())]
    assert "ground" in names
    assert "crate" not in names
    phys = runner_t2.object_physics(mujoco, model, "crate")
    assert phys["dynamic"] is True and phys["mass_kg"] > 0
    assert runner_t2.object_physics(mujoco, model, "ground")["dynamic"] is False


def test_the_urdf_defaults_lose_the_object_and_the_container():
    """TRIPWIRE on ``t2_scene`` finding 1. If this starts passing, the
    docstring is wrong and ``fusestatic="false"`` may no longer be needed."""
    mujoco = _mujoco()
    for urdf, name in ((BLOCK_URDF, "block"), (BIN_URDF, "bin")):
        model = mujoco.MjSpec.from_file(str(urdf)).compile()
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0, (
            "MuJoCo's URDF defaults now keep %r as a body; t2_scene's finding "
            "1 needs re-measuring" % name)


def test_a_welded_pedestal_jams_the_arms_first_axis(tmp_path):
    """TRIPWIRE on ``t2_scene`` finding 3 -- the loudest one.

    Weld the pedestal to the world and MuJoCo's parent contact filter stops
    applying to the pedestal/shoulder pair that overlaps by design, so the
    first axis cannot turn. Measured here through the real path: full command,
    full torque, and the joint does not move.
    """
    mujoco = _mujoco()
    welded = t2_scene.prepare_urdf(ARM_URDF, tmp_path / "welded.urdf")
    spec = mujoco.MjSpec.from_file(str(welded))
    a = spec.add_actuator()
    a.name = "pos_joint_1"
    a.target = "joint_1"
    a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    a.gaintype = mujoco.mjtGain.mjGAIN_AFFINE
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = 3000.0
    a.biasprm[1] = -3000.0
    a.forcerange = [-150.0, 150.0]
    a.forcelimited = 1
    model = spec.compile()
    data = mujoco.MjData(model)
    adr = int(model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "joint_1")])
    data.ctrl[0] = 1.5
    for _ in range(1500):                     # 3 s at the URDF default dt
        mujoco.mj_step(model, data)
    # Full command, full torque, and 5% of the commanded angle: the joint is
    # being held by its own pedestal.
    assert abs(float(data.actuator_force[0])) == pytest.approx(150.0), (
        "joint_1 is no longer saturating against its pedestal")
    assert abs(float(data.qpos[adr])) < 0.2, (
        "the welded pedestal no longer jams joint_1 (%.4f rad of a commanded "
        "1.5); t2_scene's finding 3 needs re-measuring" % data.qpos[adr])


def test_a_free_pedestal_lets_the_first_axis_turn(tmp_path):
    """The other half of the finding: the fix works, through the same path."""
    mujoco = _mujoco()
    free = t2_scene.prepare_urdf(ARM_URDF, tmp_path / "free.urdf",
                                 floating_base="base")
    spec = mujoco.MjSpec.from_file(str(free))
    ground = spec.worldbody.add_body(name="ground")
    g = ground.add_geom(name="ground_slab")
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.size = [2.0, 2.0, 0.05]
    g.pos = [0.0, 0.0, -0.05]
    a = spec.add_actuator()
    a.name = "pos_joint_1"
    a.target = "joint_1"
    a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    a.gaintype = mujoco.mjtGain.mjGAIN_AFFINE
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = 3000.0
    a.biasprm[1] = -3000.0
    a.biasprm[2] = -120.0
    a.forcerange = [-150.0, 150.0]
    a.forcelimited = 1
    model = spec.compile()
    data = mujoco.MjData(model)
    adr = int(model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "joint_1")])
    data.ctrl[0] = 1.5
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert float(data.qpos[adr]) > 1.4


def test_the_scene_builds_reloads_and_keeps_all_three_declared_names(tmp_path):
    mujoco = _mujoco()
    res = t2_scene.build_t2_scene(CONTAINER, tmp_path / "ws")
    assert res.problems == [], res.problems
    assert res.scene
    model = mujoco.MjModel.from_xml_path(res.scene)
    for name in ("tool", "block", "bin"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0
    # The three descriptions' own declared mass, not the scene furniture's.
    assert res.mass_kg_compiled == pytest.approx(53.9, abs=1e-6)
    assert model.nu == 8
    assert model.neq == 0        # nothing is welded to anything


def test_the_scripted_oracle_completes_the_transfer_and_grades_pass(tmp_path):
    """**The achievability proof, as a regression test.**

    ``tasks/T2_transfer/container/PROVENANCE.txt``: *"The arm has never been
    demonstrated completing this task on any simulator … if it turns out to be
    unachievable everywhere then the grid measured the asset and not the
    agents."* This is the answer, and it runs in a few seconds on one CPU core
    with no agent, no network and no GPU.

    It is NOT a ladder cell and its verdict is not a result -- a human wrote
    the control law knowing the thresholds.
    """
    _mujoco()
    from ladder.adapters.mujoco import run_t2
    task = ladder_tasks.get("T2_transfer")
    deliverable, build = run_t2.build_deliverable(tmp_path, task=task)
    assert deliverable is not None, build.problems
    res = runner_t2.run_standalone(
        deliverable, tmp_path, duration=task.standalone["duration_s"],
        settle=task.standalone["settle_s"],
        stride=task.standalone["contact_stride"], surfaces=task.surfaces)
    assert res.rc == 0, (res.error, (tmp_path / "stderr.log").read_text())

    verdict = t2_grader.grade(tmp_path, task=task, phase_b=res, sim="mujoco",
                              artifact=str(deliverable / "scene.xml"))
    assert verdict.outcome == "PASS", verdict.summary()
    assert not verdict.failed
    assert verdict.measurements["hold_mechanism"] == "friction"
    assert "unanswered_channels" not in verdict.measurements
    # Not one clause may be vacuous: the point of the oracle is that every
    # assertion was actually MEASURED, so a later red is a physical finding.
    assert verdict.vacuous == {}, verdict.vacuous


def test_the_oracle_run_passes_both_contract_checks(tmp_path):
    _mujoco()
    from ladder.adapters.mujoco import run_t2
    task = ladder_tasks.get("T2_transfer")
    deliverable, _build = run_t2.build_deliverable(tmp_path, task=task)
    res = runner_t2.run_standalone(deliverable, tmp_path, duration=40.0,
                                   settle=task.standalone["settle_s"],
                                   stride=task.standalone["contact_stride"],
                                   surfaces=task.surfaces)
    assert res.rc == 0
    ev = ladder_adapters.build_t2_evidence(
        task, sim="mujoco", run_dir=tmp_path, phase_b=res,
        artifact=str(deliverable / "scene.xml"))
    assert ladder_adapters.check_t2_evidence(ev) == []


# --- the driver may be called anything (2026-08-03) -------------------------


def _mk(tmp_path, name, body):
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / name).write_text(body, encoding="utf-8")
    return tmp_path


def test_the_documented_name_is_still_preferred(tmp_path):
    from ladder.adapters.mujoco import runner_t2
    _mk(tmp_path, "drive.py", "def control(m, d, t):\n    pass\n")
    (tmp_path / "other.py").write_text("def control(m, d, t):\n    pass\n",
                                       encoding="utf-8")
    assert runner_t2.find_driver(tmp_path).name == "drive.py"


def test_a_differently_named_driver_is_found(tmp_path):
    """MEASURED: an agent built a working scene, called its controller
    run.py, and phase B stepped 60 s with ctrl_writes=0 -- three assertions
    red for a reason that was neither MuJoCo's nor the agent's."""
    from ladder.adapters.mujoco import runner_t2
    _mk(tmp_path, "run.py", "def control(model, data, t):\n    pass\n")
    assert runner_t2.find_driver(tmp_path).name == "run.py"


def test_a_python_file_with_no_entry_point_is_NOT_taken_as_the_driver(tmp_path):
    """The falsifier. A deliverable full of helper modules must not have one
    picked at random and stepped as though it were the controller."""
    from ladder.adapters.mujoco import runner_t2
    _mk(tmp_path, "helpers.py", "def build_scene():\n    return 1\n")
    (tmp_path / "kinematics.py").write_text("def ik(x):\n    return x\n",
                                            encoding="utf-8")
    assert runner_t2.find_driver(tmp_path) is None


def test_a_deliverable_with_no_python_at_all_reports_none(tmp_path):
    from ladder.adapters.mujoco import runner_t2
    (tmp_path / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
    assert runner_t2.find_driver(tmp_path) is None


def test_discovery_reads_rather_than_imports(tmp_path):
    """A deliverable with a syntax error must not take the grader down: the
    scan reads text and never imports to decide whether to import."""
    from ladder.adapters.mujoco import runner_t2
    _mk(tmp_path, "broken.py", "def control(m, d, t)\n    this is not python\n")
    assert runner_t2.find_driver(tmp_path) is None
