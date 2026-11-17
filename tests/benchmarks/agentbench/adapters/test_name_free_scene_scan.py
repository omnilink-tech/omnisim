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

"""**The NAME-FREE t=0 bounds scan, on BOTH of our arms.**

The gap this closes, measured rather than argued: ``r1_core`` matches the five
benchmark obstacles by GEOMETRY -- world-space AABB centre and footprint -- and
never by name, deliberately, because agents name things freely (one real R1 run
called the boxes ``crate A``..``crate E``, another ``obstacle_1``..
``obstacle_6``, lowercase and six of them). A grader keyed on our published
``OBSTACLE_n`` names would score a correct world zero, which is grading OUR
CONVENTION instead of the task.

But until 2026-08-09 the only bounds channel either of our recorders had was
``--solids=``, **a name list**. So a world using its own names produced no
candidates at all, the geometric matcher had nothing to match, and R1.3 was
UNDECIDABLE on OmniSim and on upstream Webots -- while it passed on the MuJoCo
arm, whose t=0 scan has bounded every body and every world geom with no name
list since the day it was written. That is an instrument gap of ours, wearing a
capability difference's clothes.

Every test below fails against the pre-2026-08-09 recorders, and the last class
passes against BOTH -- which is the other half of the contract: the scan is
additive, the bundle only carries it when a grader asks (``scene_inventory``),
and the six already-frozen tasks measure exactly what they measured before.

Both arms run against a fake scene graph carrying the handful of Supervisor
methods each recorder actually calls. A live check would need ``omnisim-bin``
on one side and upstream Webots in WSL2 on the other; what a fake CAN prove is
the part that was wrong -- which bodies the walk selects, and whether a box
comes out the other end attached to the right body.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest

AGENTBENCH = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(AGENTBENCH.parents[2]))

from agentbench import adapters  # noqa: E402
from agentbench import tasks as task_registry  # noqa: E402
from agentbench.adapters.omnisim import evidence as om_evidence  # noqa: E402
from agentbench.adapters.omnisim import headless  # noqa: E402
from agentbench.adapters.omnisim import (  # noqa: E402
    test_recorder_tracks as om_fake)
from agentbench.adapters.webots import evidence as wb_evidence  # noqa: E402
from agentbench.adapters.webots import launcher  # noqa: E402
from agentbench.adapters.webots import (  # noqa: E402
    test_webots_recorder as wb_fake)
from agentbench.graders import r1_core  # noqa: E402

#: The tasks frozen before the name-free scan existed. Their evidence shape is
#: the backward-compatibility contract.
FROZEN_TASKS = ("A1_husky_swarm_10", "B1_overlap_audit", "B2_subject_in_frame",
                "B3_measure_and_report", "C1_parse_error_fix",
                "C2_fall_through_floor")

OM_REC = om_fake.REC
WB_REC = wb_fake.REC


# --- a bounds helper for the OmniSim fake ------------------------------------
#
# ``geometry.bounds_for_subtree`` is an engine-side import that does not exist
# outside a running OmniSim, so the fake supplies a box the same SHAPE the real
# helper returns. The half-extents live on the fake node, so a test can author
# a 0.6 x 0.6 x 1.2 obstacle and assert the matcher finds it.


def _install_om_geometry(monkeypatch):
    def bounds_for_subtree(node):
        p = node.getPosition()
        h = getattr(node, "half", (0.5, 0.5, 0.5))
        return {"bbox_min": [p[0] - h[0], p[1] - h[1], p[2] - h[2]],
                "bbox_max": [p[0] + h[0], p[1] + h[1], p[2] + h[2]],
                "exact": True}
    stub = types.SimpleNamespace(bounds_for_subtree=bounds_for_subtree)
    monkeypatch.setattr(OM_REC, "_geometry", stub)
    monkeypatch.setattr(OM_REC, "_GEOM_ERR", None)


def _om_solid(name, xyz, size, physics=False, defname=""):
    """One fake OmniSim Solid with an authored footprint."""
    node = om_fake.N("Solid", name=name, defname=defname, physics=physics,
                     position=xyz)
    node.half = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
    return node


def _om_rover(xyz=(-4.0, -4.0, 0.2)):
    """A two-joint wheeled robot: R1.2's "one drivable robot"."""
    wheels = [om_fake.N("Solid", name="wheel_%d" % i, physics=True,
                        position=(xyz[0], xyz[1] + 0.2 * i, 0.1))
              for i in range(2)]
    joints = [om_fake.N("HingeJoint", end_point=w) for w in wheels]
    return om_fake.N("Robot", name="rover", defname="ROVER",
                     controller="nav", physics=True, children=joints,
                     position=xyz)


def _om_run(monkeypatch, tmp_path, root, argv, *, movers=()):
    return om_fake._run_recorder(monkeypatch, tmp_path, root, argv,
                                 movers=movers)


def _om_bundle(tmp_path, rows, meta, phase_a, task="R1_lidar_nav", **kw):
    res = om_fake._phase_b(tmp_path, rows, meta)
    res.phase_a = phase_a
    return om_evidence.build_bundle(task, phase_b=res,
                                    artifact=str(tmp_path / "w.wbt"), **kw)


# --- fake geometry for the Webots arm ----------------------------------------
#
# The Webots recorder computes its own AABBs from real geometry nodes, so the
# fake supplies real geometry: a Shape wrapping a Box, exactly as a world does.


class _GeomField:
    """An SFVec3f / SFVec2f / SFFloat / SFInt32 field on a fake node."""

    def __init__(self, kind, value):
        self.kind, self.value = kind, value

    def getSFVec3f(self):
        return list(self.value)

    def getSFVec2f(self):
        return list(self.value)

    def getSFFloat(self):
        return float(self.value)

    def getSFInt32(self):
        return int(self.value)

    def getSFRotation(self):
        return list(self.value)


def _wb_box(size):
    node = wb_fake.N("Box", translation=False)
    node.fields["size"] = _GeomField("vec3", list(size))
    return node


def _wb_shape(geometry):
    node = wb_fake.N("Shape", translation=False)
    node.fields["geometry"] = wb_fake._Field("sf", geometry)
    return node


def _wb_solid(name, xyz, size, physics=False, defname=None, children=(),
              bounded=True):
    """A fake upstream Solid with a real ``boundingObject`` Box."""
    node = wb_fake.N("Solid", name=name, defname=defname, physics=physics,
                     position=xyz, children=list(children))
    if bounded:
        node.fields["boundingObject"] = wb_fake._Field("sf",
                                                       _wb_shape(_wb_box(size)))
    node.orientation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return node


def _wb_getorientation(self):
    return list(getattr(self, "orientation",
                        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]))


wb_fake.N.getOrientation = _wb_getorientation


def _wb_rover(xyz=(-4.0, -4.0, 0.2)):
    wheels = [_wb_solid("wheel_%d" % i, (xyz[0], xyz[1] + 0.2 * i, 0.1),
                        (0.2, 0.1, 0.2), physics=True) for i in range(2)]
    joints = [wb_fake.N("HingeJoint", end_point=w, translation=False)
              for w in wheels]
    return wb_fake.N("Robot", name="rover", defname="ROVER", controller="nav",
                     physics=True, children=joints, position=xyz)


def _wb_run(monkeypatch, tmp_path, root, argv, *, movers=()):
    return wb_fake._run_recorder(monkeypatch, tmp_path, root, argv,
                                 movers=movers)


def _wb_bundle(run_docs, tmp_path, task="R1_lidar_nav", **kw):
    """A Webots bundle from in-memory recorder documents."""
    for name, doc in run_docs.items():
        if doc is not None:
            (tmp_path / ("%s.json" % name)).write_text(json.dumps(doc),
                                                       encoding="utf-8")
    from agentbench.adapters.webots import recording
    run = recording.read_run(tmp_path)
    return wb_evidence.build_bundle(task, run=run,
                                    artifact=str(tmp_path / "w.wbt"), **kw)


# --- 1. a body the task never named arrives with world-space bounds ----------


def test_omnisim_bounds_a_body_nobody_named(monkeypatch, tmp_path):
    """OmniSim: no ``--solids=``, and the crate still has a world box.

    THE assertion the whole change exists for. Against the pre-2026-08-09
    recorder ``t0_scene`` does not exist and ``bundle.roster`` holds one robot
    and nothing else, so a geometric matcher has zero candidates.
    """
    _install_om_geometry(monkeypatch)
    crate = _om_solid("crate A", (1.0, 2.0, 0.5), (0.6, 0.6, 1.0))
    root = om_fake.N("Group", children=[crate, _om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])

    scan = phase_a["t0_scene"]
    assert scan["found"] == 1 and scan["bounded"] == 1
    assert scan["truncated"] is False

    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)
    body = bundle.roster.by_name("crate A")
    assert body is not None, [b.name for b in bundle.roster.bodies]
    assert body.has_aabb
    assert body.aabb_center == pytest.approx((1.0, 2.0, 0.5))
    assert body.robot_class is False
    # ...and it is in the frozen t=0 inventory too, which is what the
    # interpenetration and rest-height assertions read.
    assert bundle.t0.by_name("crate A") is not None


def test_webots_bounds_a_body_nobody_named(monkeypatch, tmp_path):
    """Upstream: same claim, same shape, computed from ``boundingObject``.

    Upstream has no Supervisor bounds query at all, so the recorder derives the
    box from the body's own geometry and world pose. Against the pre-2026-08-09
    recorder no body in a graded R1 run carries an AABB from any channel: the
    separate prober is only launched for the four tasks in ``NEEDS_AABB``.
    """
    crate = _wb_solid("crate A", (1.0, 2.0, 0.5), (0.6, 0.6, 1.0))
    root = wb_fake.N("Group", children=[wb_fake.N("WorldInfo",
                                                  translation=False),
                                        crate, _wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0"])

    scan = out["roster"]["scene_bodies"]
    assert scan["supported"] is True
    assert scan["found"] == 1 and scan["bounded"] == 1
    rec = scan["bodies"][0]
    assert rec["name"] == "crate A"
    assert rec["bounds_source"] == "boundingObject"
    assert rec["bounds"]["bbox_min"] == pytest.approx([0.7, 1.7, 0.0])
    assert rec["bounds"]["bbox_max"] == pytest.approx([1.3, 2.3, 1.0])

    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)
    body = bundle.roster.by_name("crate A")
    assert body is not None and body.has_aabb
    assert body.aabb_center == pytest.approx((1.0, 2.0, 0.5))
    assert body.robot_class is False


# --- 2. THE POINT: five of five, whatever the agent called them ---------------


def _perturbed_obstacle_scene(builder):
    """The R1 spec's five obstacles, built with names the task never uses."""
    spec = r1_core.obstacle_spec()
    letters = "ABCDE"
    return [builder("crate %s" % letters[i], tuple(o["position"]),
                    tuple(o["size"]))
            for i, o in enumerate(spec)], spec


def test_omnisim_r1_finds_five_of_five_with_arbitrary_names(monkeypatch,
                                                            tmp_path):
    """``match_spec_obstacles`` on a world that uses NONE of our names.

    This is the end-to-end proof, run against the real grader core and the real
    frozen ``obstacles.json``: build the specified geometry, call the boxes
    ``crate A``..``crate E``, and the matcher finds all five and re-derives
    that the straight START->GOAL line is blocked.
    """
    _install_om_geometry(monkeypatch)
    boxes, spec = _perturbed_obstacle_scene(_om_solid)
    root = om_fake.N("Group", children=boxes + [_om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)

    non_robots = [b for b in bundle.roster.bodies if not b.robot_class]
    assert [b for b in non_robots if b.has_aabb], "no candidate had bounds"
    found, missing = r1_core.match_spec_obstacles(non_robots, spec)
    assert missing == []
    assert len(found) == r1_core.N_OBSTACLES
    assert r1_core.segment_blocked_by(found), (
        "the sensing argument rests on the straight line being blocked")
    # ...and the robot is still exactly one robot.
    assert len([b for b in bundle.roster.bodies if b.robot_class]) == 1


def test_webots_r1_finds_five_of_five_with_arbitrary_names(monkeypatch,
                                                           tmp_path):
    """The same claim on the control arm, so the comparison stays a comparison.

    An assertion one arm can pass and the other structurally cannot is not a
    measurement of the simulators; it is a measurement of our instrument.
    """
    boxes, spec = _perturbed_obstacle_scene(_wb_solid)
    root = wb_fake.N("Group",
                     children=[wb_fake.N("WorldInfo", translation=False)]
                     + boxes + [_wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0"])
    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)

    non_robots = [b for b in bundle.roster.bodies if not b.robot_class]
    found, missing = r1_core.match_spec_obstacles(non_robots, spec)
    assert missing == []
    assert len(found) == r1_core.N_OBSTACLES
    assert r1_core.segment_blocked_by(found)
    assert len([b for b in bundle.roster.bodies if b.robot_class]) == 1


# --- 3. robot_class stays honest ---------------------------------------------


def test_omnisim_scanned_scenery_is_never_robot_class(monkeypatch, tmp_path):
    """A scanned box is not a robot, and a robot's wheel is not a body of the
    scene.

    R1.2, R2.2 and R3.3 all count ``roster.robots``. If the scan let scenery
    read as robot-class, R1.2's "exactly one drivable robot" would fail a
    correct world with six; if it descended INTO the robot, the wheels would
    arrive as independent scenery and the same thing would happen.
    """
    _install_om_geometry(monkeypatch)
    boxes, _spec = _perturbed_obstacle_scene(_om_solid)
    root = om_fake.N("Group", children=boxes + [_om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)

    assert [b.name for b in bundle.roster.robots] == ["rover"]
    assert [b.name for b in bundle.t0.robots] == ["rover"]
    scanned = {b.name for b in bundle.roster.bodies if not b.robot_class}
    assert scanned == {"crate %s" % c for c in "ABCDE"}
    assert not any(n.startswith("wheel_") for n in scanned), (
        "the scan entered a Robot subtree; links belong to --links=")


def test_webots_scanned_scenery_is_never_robot_class(monkeypatch, tmp_path):
    boxes, _spec = _perturbed_obstacle_scene(_wb_solid)
    root = wb_fake.N("Group",
                     children=[wb_fake.N("WorldInfo", translation=False)]
                     + boxes + [_wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0"])
    names = {b["name"] for b in out["roster"]["scene_bodies"]["bodies"]}
    assert names == {"crate %s" % c for c in "ABCDE"}
    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)
    assert [b.name for b in bundle.roster.robots] == ["rover"]
    for b in bundle.roster.bodies:
        if b.name != "rover":
            assert b.robot_class is False, b.name


# --- 4. nesting, and the identity a nested body carries ----------------------


def test_omnisim_reaches_a_body_nested_inside_another(monkeypatch, tmp_path):
    """A crate inside a pallet is found, and says whose part it is.

    ``--solids=`` walked the whole tree; a name-free scan has to as well, or a
    world that groups its obstacles is invisible for a reason that has nothing
    to do with the agent.
    """
    _install_om_geometry(monkeypatch)
    inner = _om_solid("widget", (1.0, 1.0, 1.0), (0.2, 0.2, 0.2),
                      physics=True)
    pallet = om_fake.N("Solid", name="pallet", physics=False,
                       children=[inner], position=(1.0, 1.0, 0.0))
    pallet.half = (0.8, 0.8, 0.1)
    root = om_fake.N("Group", children=[pallet, _om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)

    widget = bundle.t0.by_name("widget")
    assert widget is not None and widget.has_aabb
    assert widget.member_of == bundle.t0.by_name("pallet").body_id
    # ``independent`` is what a core asks for "the scene's own objects".
    assert "widget" not in {b.name for b in bundle.t0.independent}
    assert "pallet" in {b.name for b in bundle.t0.independent}


def test_webots_reaches_a_body_nested_inside_another(monkeypatch, tmp_path):
    inner = _wb_solid("widget", (1.0, 1.0, 1.0), (0.2, 0.2, 0.2),
                      physics=True)
    pallet = _wb_solid("pallet", (1.0, 1.0, 0.0), (1.6, 1.6, 0.2),
                       children=[inner])
    root = wb_fake.N("Group", children=[wb_fake.N("WorldInfo",
                                                  translation=False),
                                        pallet, _wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0"])
    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)

    widget = bundle.t0.by_name("widget")
    assert widget is not None and widget.has_aabb
    assert widget.aabb_center == pytest.approx((1.0, 1.0, 1.0))
    assert widget.member_of == bundle.t0.by_name("pallet").body_id
    assert "widget" not in {b.name for b in bundle.t0.independent}


# --- 5. the cost bound, and its truncation ------------------------------------


def test_omnisim_scan_is_capped_and_says_so(monkeypatch, tmp_path):
    """Truncation is REPORTED, never silent.

    A procedurally generated forest could hand the scan thousands of trunks;
    the cap is what stops one world from costing a campaign. A grader that
    fails to find something must be able to tell "it is not there" from "the
    inventory stopped early".
    """
    _install_om_geometry(monkeypatch)
    boxes = [_om_solid("box_%d" % i, (float(i), 0.0, 0.5), (0.4, 0.4, 1.0))
             for i in range(9)]
    root = om_fake.N("Group", children=boxes + [_om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--scan-solids=4"])

    scan = phase_a["t0_scene"]
    assert scan["cap"] == 4
    assert scan["found"] == 9
    assert len(scan["bodies"]) == 4
    assert scan["truncated"] is True
    assert meta["scene_scan_cap"] == 4

    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)
    assert any("TRUNCATED" in n for n in bundle.notes), bundle.notes
    row = bundle.adapter_measurements["roster"]["scene_scan"]
    assert (row["found"], row["cap"], row["truncated"]) == (9, 4, True)


def test_webots_scan_is_capped_and_says_so(monkeypatch, tmp_path):
    boxes = [_wb_solid("box_%d" % i, (float(i), 0.0, 0.5), (0.4, 0.4, 1.0))
             for i in range(9)]
    root = wb_fake.N("Group",
                     children=[wb_fake.N("WorldInfo", translation=False)]
                     + boxes + [_wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0",
                   "--scan-solids=4"])
    scan = out["roster"]["scene_bodies"]
    assert (scan["cap"], scan["found"], scan["truncated"]) == (4, 9, True)
    assert len(scan["bodies"]) == 4

    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)
    assert any("TRUNCATED" in n for n in bundle.notes), bundle.notes


def test_the_scan_adds_no_sampled_row_on_either_arm(monkeypatch, tmp_path):
    """The OTHER half of the cost bound: t=0 only, no per-step column.

    A track costs one sample per body per basic timestep for the whole
    recording -- 2,188 samples per body over R2's 35 s window. The scan costs
    one geometry walk, once. If it ever started adding rows, ``a1_core``'s
    ``n_bodies == 10`` would fail on a correct swarm.
    """
    _install_om_geometry(monkeypatch)
    boxes, _spec = _perturbed_obstacle_scene(_om_solid)
    root = om_fake.N("Group", children=boxes + [_om_rover()])
    rows, meta, _pa = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    assert rows[0] == ["t", "r0_x", "r0_y", "r0_z"]
    assert meta["n_tracks"] == meta["n_robots"] == 1
    assert meta["n_solid_tracks"] == 0

    wb_boxes, _s = _perturbed_obstacle_scene(_wb_solid)
    wroot = wb_fake.N("Group",
                      children=[wb_fake.N("WorldInfo", translation=False)]
                      + wb_boxes + [_wb_rover()])
    out = _wb_run(monkeypatch, tmp_path / "wb", wroot,
                  ["--duration=0.032", "--contact-steps=0"])
    # Upstream's recorder samples top-level bodies that CAN move; the five
    # static crates are in the roster and out of the trajectory, exactly as
    # before the scan existed.
    assert [b["name"] for b in out["trajectory"]["bodies"]] == ["rover"]


# --- 6. absent stays absent ---------------------------------------------------


def test_omnisim_an_unbounded_body_gets_no_invented_box(monkeypatch,
                                                        tmp_path):
    """A body the bounds walk could not measure carries ``None``, not zeros."""
    _install_om_geometry(monkeypatch)

    def failing(node):
        raise RuntimeError("no geometry")
    monkeypatch.setattr(OM_REC._geometry, "bounds_for_subtree", failing)

    crate = _om_solid("crate A", (1.0, 2.0, 0.5), (0.6, 0.6, 1.0))
    root = om_fake.N("Group", children=[crate, _om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    scan = phase_a["t0_scene"]
    assert scan["found"] == 1 and scan["bounded"] == 0
    assert scan["bodies"][0]["bounds"] is None
    assert "no geometry" in scan["bodies"][0]["bounds_error"]

    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)
    body = bundle.roster.by_name("crate A")
    assert body is not None
    assert body.aabb_min is None and body.aabb_max is None
    assert any("no world-space AABB" in n for n in bundle.notes), bundle.notes


def test_webots_a_mesh_bounded_body_is_visibly_unmeasured(monkeypatch,
                                                          tmp_path):
    """The real upstream gap, published rather than approximated.

    A ``Mesh { url ... }`` hull needs the STL/OBJ/PLY/glTF readers OmniSim's
    ``geometry.bounds_for_subtree`` has and a controller under upstream's own
    python3 does not. The honest answer is no box and the reason -- the wrong
    answer would be a guessed one, which is an approximation that quietly
    disadvantages whichever arm it lands on.
    """
    mesh = wb_fake.N("Mesh", translation=False)
    crate = wb_fake.N("Solid", name="statue", position=(1.0, 2.0, 0.5))
    crate.fields["boundingObject"] = wb_fake._Field("sf", _wb_shape(mesh))
    root = wb_fake.N("Group", children=[wb_fake.N("WorldInfo",
                                                  translation=False),
                                        crate, _wb_rover()])
    out = _wb_run(monkeypatch, tmp_path, root,
                  ["--duration=0.032", "--contact-steps=0"])
    rec = out["roster"]["scene_bodies"]["bodies"][0]
    assert rec["bounds"] is None
    assert "Mesh" in rec["skipped_geometry"]
    assert out["roster"]["scene_bodies"]["bounded"] == 0

    bundle = _wb_bundle(out, tmp_path, scene_inventory=True)
    body = bundle.roster.by_name("statue")
    assert body is not None and not body.has_aabb
    assert any("Mesh" in n and "NO " in n for n in bundle.notes), bundle.notes


def test_omnisim_a_recording_without_the_scan_says_instrument_gap(tmp_path):
    """An old recording must not read as "the scene is empty".

    ``r1_core`` distinguishes "no candidates were scanned" (an instrument gap)
    from "the obstacles are not there" (an agent failure), and it can only do
    that if the adapter refuses to turn a missing scan into an empty one.
    """
    res = headless.PhaseBResult(tmp_path)
    res.phase_a = {"t0_robots": [], "robot_robot_contacts": []}
    assert res.scene_scan is None
    bundle = om_evidence.build_bundle("R1_lidar_nav", phase_b=res,
                                      artifact=str(tmp_path / "w.wbt"),
                                      scene_inventory=True)
    assert any("INSTRUMENT GAP" in n for n in bundle.notes), bundle.notes


# --- 7. the six frozen tasks' evidence shape is unchanged ---------------------


class TestTheSixFrozenTasksAreUntouched:
    """The backward-compatibility contract, asserted rather than hoped for."""

    @pytest.mark.parametrize("task_id", FROZEN_TASKS)
    def test_no_frozen_task_asks_for_the_scan(self, task_id):
        """No frozen meta sets ``scan_solids``, so no new controllerArg."""
        assert "scan_solids" not in task_registry.get(task_id).standalone

    @pytest.mark.parametrize("task_id", FROZEN_TASKS)
    def test_the_injected_stanza_is_unchanged(self, task_id, tmp_path):
        """Both arms: no opt-in => byte-identical injected world text.

        The scan runs at the recorder's own default, so its cap never has to
        be written into a world -- which is what keeps every frozen task's
        injected sibling exactly the file it was.
        """
        sa = task_registry.get(task_id).standalone
        world = tmp_path / "w.wbt"
        world.write_text("WorldInfo {\n}\n", encoding="utf-8")
        sib = headless.inject_recorder(
            world, tmp_path / "phaseB.csv",
            duration=float(sa.get("duration_s", 30.0)),
            settle=float(sa.get("settle_s", 1.0)),
            contact_steps=int(sa.get("contact_steps", 10)),
            solids=tuple(sa.get("solids", ())),
            links=int(sa.get("links", 0)),
            solid_tracks=sa.get("solid_tracks"),
            scan_solids=sa.get("scan_solids"))
        assert "--scan-solids=" not in sib.read_text(encoding="utf-8")

        stanza = launcher.recorder_stanza(
            "/out", duration=float(sa.get("duration_s", 10.0)),
            contact_steps=int(sa.get("contact_steps", 1)),
            solids=tuple(sa.get("solids", ())),
            links=int(sa.get("links", 0)),
            solid_tracks=sa.get("solid_tracks"),
            scan_solids=sa.get("scan_solids"))
        assert "--scan-solids=" not in stanza

    def test_omnisim_bundle_ignores_the_scan_unless_asked(self, monkeypatch,
                                                          tmp_path):
        """THE linchpin: a recording WITH a scan grades as if it had none.

        ``a1_core`` publishes ``robots_seen = len(roster.bodies)`` and
        ``c1_core`` publishes "bodies beyond the intended roster" from
        ``t0.names``. Handing either a floor and four walls would rewrite a
        frozen row's measurements without one physical fact changing, so the
        bundle carries the scan only when a grader asks for it.
        """
        _install_om_geometry(monkeypatch)
        floor = _om_solid("floor", (0.0, 0.0, 0.0), (10.0, 10.0, 0.1))
        walls = [_om_solid("wall_%d" % i, (float(i), 5.0, 1.0),
                           (10.0, 0.2, 2.0)) for i in range(2)]
        root = om_fake.N("Group", children=[floor] + walls + [_om_rover()])
        rows, meta, phase_a = _om_run(
            monkeypatch, tmp_path, root,
            ["--duration=0.032", "--settle=0.0", "--phase-a=1",
             "--contact-steps=0"])
        assert phase_a["t0_scene"]["found"] == 3

        off = _om_bundle(tmp_path, rows, meta, phase_a,
                         task="A1_husky_swarm_10")
        assert [b.name for b in off.roster.bodies] == ["rover"]
        assert [b.name for b in off.t0.bodies] == ["rover"]
        assert "scene_scan" not in (
            off.adapter_measurements.get("roster") or {})

        on = _om_bundle(tmp_path, rows, meta, phase_a,
                        task="A1_husky_swarm_10", scene_inventory=True)
        assert len(on.roster.bodies) == 4

    def test_webots_bundle_ignores_the_scan_unless_asked(self, monkeypatch,
                                                         tmp_path):
        """Same on the control arm, where the stakes are the AABB channel.

        B1 / B2 / B3 / C2 are graded on bounds the separate
        ``agentbench_aabb_prober`` measured in its own launch. Supplying a
        second source by default would change what those rows measured.
        """
        floor = _wb_solid("floor", (0.0, 0.0, 0.0), (10.0, 10.0, 0.1))
        root = wb_fake.N("Group", children=[wb_fake.N("WorldInfo",
                                                      translation=False),
                                            floor, _wb_rover()])
        out = _wb_run(monkeypatch, tmp_path, root,
                      ["--duration=0.032", "--contact-steps=0"])
        # The prober's channel is untouched: no AABB on any ``bodies`` record.
        for rec in out["roster"]["bodies"]:
            assert "aabb_min" not in rec and "bounds" not in rec

        off = _wb_bundle(out, tmp_path, task="B1_overlap_audit")
        assert not any(b.has_aabb for b in off.roster.bodies)
        assert len(off.roster.bodies) == 2

        on = _wb_bundle(out, tmp_path, task="B1_overlap_audit",
                        scene_inventory=True)
        assert on.roster.by_name("floor").has_aabb

    def test_webots_prefers_the_probers_numbers_when_both_exist(self,
                                                               monkeypatch,
                                                               tmp_path):
        """A task already graded through the prober keeps the prober's boxes.

        The two channels measure the same body from two launches of the same
        world text. They should agree -- but "should" is not a contract, and a
        frozen row must not silently change which launch its numbers came
        from.
        """
        floor = _wb_solid("floor", (0.0, 0.0, 0.0), (10.0, 10.0, 0.1))
        root = wb_fake.N("Group", children=[wb_fake.N("WorldInfo",
                                                      translation=False),
                                            floor, _wb_rover()])
        out = _wb_run(monkeypatch, tmp_path, root,
                      ["--duration=0.032", "--contact-steps=0"])
        for rec in out["roster"]["bodies"]:
            if rec["name"] == "floor":
                rec["bounds"] = {"bbox_min": [-1.0, -1.0, -1.0],
                                 "bbox_max": [1.0, 1.0, 1.0]}
        bundle = _wb_bundle(out, tmp_path, task="B1_overlap_audit",
                            scene_inventory=True)
        body = bundle.roster.by_name("floor")
        assert body.aabb_min == (-1.0, -1.0, -1.0)
        # ...and the citation is byte-identical to the one a run without the
        # scan carries. ``c2_core`` prints ``identity_evidence`` into its
        # ``attested`` list, so a frozen row must not change wording because a
        # channel it never used now exists.
        off = _wb_bundle(out, tmp_path, task="B1_overlap_audit")
        assert (body.identity_evidence
                == off.roster.by_name("floor").identity_evidence)
        assert "name-free" not in body.identity_evidence


# --- 8. the bundle still satisfies the adapter contract -----------------------


def test_the_scanned_bundle_passes_check_bundle(monkeypatch, tmp_path):
    """Every scanned body has an id, and the contract checker agrees.

    ``check_bundle`` refuses a body with no ``body_id`` -- distinctness needs
    one -- and refuses a t=0 inventory with no AABB anywhere. Both are exactly
    what the scan supplies, so this is the shape check that would catch a
    scanned entry arriving anonymous.
    """
    _install_om_geometry(monkeypatch)
    boxes, _spec = _perturbed_obstacle_scene(_om_solid)
    root = om_fake.N("Group", children=boxes + [_om_rover()])
    rows, meta, phase_a = _om_run(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"])
    bundle = _om_bundle(tmp_path, rows, meta, phase_a, scene_inventory=True)
    assert adapters.check_bundle(bundle) == []


# --- 9. the two Webots AABB implementations agree -----------------------------


def _load_prober():
    """Import the prober outside Webots, the same way the recorders load."""
    path = (AGENTBENCH / "adapters" / "webots" / "webots_lane" / "controllers"
            / "agentbench_aabb_prober" / "agentbench_aabb_prober.py")
    stub = types.ModuleType("controller")
    stub.Supervisor = object
    saved = sys.modules.get("controller")
    sys.modules["controller"] = stub
    try:
        spec = importlib.util.spec_from_file_location(
            "_agentbench_aabb_prober_under_test", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        if saved is None:
            sys.modules.pop("controller", None)
        else:
            sys.modules["controller"] = saved
    return mod


PROBER = _load_prober()


@pytest.mark.parametrize("size", [(0.6, 0.6, 1.0), (2.0, 0.25, 0.5),
                                  (10.0, 10.0, 0.1)])
def test_the_recorders_boxes_match_the_probers(size):
    """Two implementations of "the world AABB of a boundingObject", pinned.

    The prober is frozen: B1 / B2 / B3 / C2 are graded on its numbers, from its
    own launch, and this change deliberately does not touch it. The recorder
    grew its own copy of the walk so a graded run needs no second launch -- so
    the two must agree on the geometry they BOTH handle, or the arm would
    report one box for a task in ``NEEDS_AABB`` and a different one for R1.
    (The recorder's walk is a documented SUPERSET: Cone, ElevationGrid,
    explicit coordinate sets and a visual-geometry fallback, none of which the
    prober attempts.)
    """
    body = _wb_solid("thing", (1.0, -2.0, 0.5), size)
    lo_p, hi_p, _skipped = PROBER._aabb_for(body)
    lo_r, hi_r, _src, _sk = WB_REC._world_aabb(body)
    assert lo_r == pytest.approx(lo_p)
    assert hi_r == pytest.approx(hi_p)


def test_the_recorder_bounds_a_body_the_prober_declines(monkeypatch):
    """The superset, stated: a Cone hull is measured here and skipped there."""
    cone = wb_fake.N("Cone", translation=False)
    cone.fields["bottomRadius"] = _GeomField("float", 0.5)
    cone.fields["height"] = _GeomField("float", 2.0)
    body = wb_fake.N("Solid", name="cone", position=(0.0, 0.0, 1.0))
    body.fields["boundingObject"] = wb_fake._Field("sf", _wb_shape(cone))

    lo_p, hi_p, skipped_p = PROBER._aabb_for(body)
    assert lo_p is None and "Cone" in skipped_p

    lo_r, hi_r, _src, _sk = WB_REC._world_aabb(body)
    assert lo_r == pytest.approx([-0.5, -0.5, 0.0])
    assert hi_r == pytest.approx([0.5, 0.5, 2.0])


def test_the_recorder_falls_back_to_visual_geometry():
    """A Solid with no ``boundingObject`` still gets a box, and says so.

    OmniSim's helper unions visual AND collision geometry, so a body that is
    invisible to one arm's bounds and not the other's would be an instrument
    asymmetry wearing a capability's clothes.
    """
    body = _wb_solid("decor", (0.0, 0.0, 1.0), (1.0, 1.0, 1.0), bounded=False,
                     children=[_wb_shape(_wb_box((1.0, 1.0, 1.0)))])
    lo, hi, source, _sk = WB_REC._world_aabb(body)
    assert source == "children geometry"
    assert lo == pytest.approx([-0.5, -0.5, 0.5])
    assert hi == pytest.approx([0.5, 0.5, 1.5])
