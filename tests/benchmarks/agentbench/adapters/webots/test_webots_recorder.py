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

"""The upstream-Webots recorder's three track kinds -- **the symmetry half**.

The OmniSim arm's twin of this file is
``adapters/omnisim/test_recorder_tracks.py``. Both exist because a
cross-simulator comparison in which one arm can see an end effector and the
other cannot is not a comparison: it prices our own instrument's blind spot as
a property of the other simulator. So every capability added to one recorder is
asserted here for the other, in the same order, against the same shaped scene.

One asymmetry is narrower than it was and the rest of it is still real.
Upstream's Supervisor API has no bounds query (``BRINGUP.md`` §4.1), so the
``bodies`` list still carries no AABB from any channel of this file's -- that is
the separate ``agentbench_aabb_prober``'s, merged in from its own launch, and
``test_bounds_are_declared_not_expressible`` pins it. Since 2026-08-09 the
recorder DOES compute world-space AABBs, in a channel of their own
(``scene_bodies``, the name-free t=0 scan), because the suite's geometric
assertions match by box and never by name and a bounds channel that has to be
handed a name list cannot answer them. What upstream still cannot express is a
``Mesh { url ... }`` hull -- covered, with both arms side by side, in
``adapters/test_name_free_scene_scan.py``.

Everything runs against a fake scene graph with the handful of Supervisor
methods the recorder actually calls: upstream Webots lives in WSL2 and a live
run needs it, but which bodies the walk selects -- the part that was wrong --
does not.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path

AGENTBENCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AGENTBENCH.parents[2]))

from agentbench.adapters.webots import evidence as wb_evidence  # noqa: E402
from agentbench.adapters.webots import launcher, recording  # noqa: E402

RECORDER_PATH = (AGENTBENCH / "adapters" / "webots" / "webots_lane"
                 / "controllers" / "agentbench_webots_recorder"
                 / "agentbench_webots_recorder.py")


def _load_recorder():
    stub = types.ModuleType("controller")
    stub.Supervisor = object
    saved = sys.modules.get("controller")
    sys.modules["controller"] = stub
    try:
        spec = importlib.util.spec_from_file_location(
            "_agentbench_webots_recorder_under_test", RECORDER_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        if saved is None:
            sys.modules.pop("controller", None)
        else:
            sys.modules["controller"] = saved
    return mod


REC = _load_recorder()


# --- a fake scene graph ------------------------------------------------------


class _Field:
    def __init__(self, kind, value):
        self.kind, self.value = kind, value

    def getCount(self):
        if self.kind != "mf":
            raise TypeError("not an MF field")
        return len(self.value)

    def getMFNode(self, i):
        return self.value[i]

    def getSFNode(self):
        if self.kind != "sf":
            raise TypeError("not an SF node field")
        return self.value

    def getSFString(self):
        if self.kind != "string":
            raise TypeError("not an SF string field")
        return self.value


_IDS = [200]


class N:
    def __init__(self, type_name, *, base=None, name=None, defname=None,
                 children=None, end_point=None, physics=None,
                 controller=None, position=(0.0, 0.0, 0.0),
                 velocity=(0.0, 0.0, 0.0), translation=True,
                 contacts=(), schedule=None, contact_error=None):
        _IDS[0] += 1
        self.contacts = list(contacts)
        self.schedule = dict(schedule or {})
        self.contact_error = contact_error
        self.type_name = type_name
        self.base = base or type_name
        self.node_id = _IDS[0]
        self.defname = defname
        self.position = list(position)
        self.velocity = list(velocity)
        self.fields = {}
        if children is not None:
            self.fields["children"] = _Field("mf", list(children))
        if end_point is not None:
            self.fields["endPoint"] = _Field("sf", end_point)
        if name is not None:
            self.fields["name"] = _Field("string", name)
        if physics is not None:
            self.fields["physics"] = _Field(
                "sf", N("Physics") if physics else None)
        if controller is not None:
            self.fields["controller"] = _Field("string", controller)
        if translation:
            self.fields["translation"] = _Field("vec", list(position))

    def getField(self, name):
        return self.fields.get(name)

    def getProto(self):
        return None

    def getTypeName(self):
        return self.type_name

    def getBaseTypeName(self):
        return self.base

    def getDef(self):
        return self.defname

    def getId(self):
        return self.node_id

    def getPosition(self):
        return list(self.position)

    def getContactPoints(self, _include=False):
        if self.contact_error is not None:
            raise self.contact_error
        return [_CP(p) for p in self.contacts]


class _CP:
    """Upstream's ``ContactPoint``: a world point and the descendant it
    belongs to. It never names the OTHER participant, which is the whole
    reason the recorder has to pair across queries."""

    def __init__(self, point, node_id=-1):
        self.point, self.node_id = list(point), node_id

    def getPoint(self):
        return self.point

    def getNodeId(self):
        return self.node_id


class FakeSupervisor:
    def __init__(self, root, dt_ms=16, movers=(), self_node=None,
                 scripted=()):
        self.root = root
        self.dt_ms = dt_ms
        self.movers = list(movers)
        self.scripted = list(scripted)
        self.self_node = self_node or N("Robot", name="agentbench_recorder")
        self.time = 0.0
        self.steps = 0
        self.quit_code = None

    def getRoot(self):
        return self.root

    def getSelf(self):
        return self.self_node

    def getBasicTimeStep(self):
        return self.dt_ms

    def getWorldPath(self):
        return "/fake/world.omniworld"

    def getTime(self):
        return self.time

    def step(self, ms):
        self.time += ms / 1000.0
        self.steps += 1
        for node in self.movers:
            for k in range(3):
                node.position[k] += node.velocity[k]
        # Scripted contacts: a body's contact set for THIS step, so a fixture
        # can stage "the robot strikes the box at step 3" without a simulator.
        for node in self.scripted:
            node.contacts = list(node.schedule.get(self.steps, ()))
        return 0

    def simulationQuit(self, code):
        self.quit_code = code


def _run_recorder(monkeypatch, tmp_path, root, argv, *, movers=(),
                  scripted=()):
    sup = FakeSupervisor(root, movers=movers, scripted=scripted)
    monkeypatch.setattr(REC, "Supervisor", lambda: sup)
    monkeypatch.setattr(
        sys, "argv",
        ["agentbench_webots_recorder", "--out-dir=%s" % tmp_path] + list(argv))
    REC.main()
    read = {}
    for name in ("roster", "trajectory", "contacts", "completion"):
        p = tmp_path / ("%s.json" % name)
        read[name] = (json.loads(p.read_text(encoding="utf-8"))
                      if p.is_file() else None)
    return read


# --- scenes (shaped like the OmniSim arm's, deliberately) --------------------


def _arm_scene():
    tool = N("Solid", name="tool", physics=True, position=(0.4, 0.0, 0.5),
             velocity=(0.001, 0.0, 0.0))
    nameplate = N("Solid", name="nameplate", physics=False)
    forearm = N("Solid", name="forearm", physics=True, children=[tool],
                position=(0.3, 0.0, 0.5), velocity=(0.001, 0.0, 0.0))
    upperarm = N("Solid", name="upperarm", physics=True,
                 children=[N("HingeJoint", end_point=forearm), nameplate],
                 position=(0.1, 0.0, 0.4), velocity=(0.0005, 0.0, 0.0))
    arm = N("Robot", name="arm", defname="ARM", controller="reach",
            physics=None, children=[N("HingeJoint", end_point=upperarm)])
    return arm, {"tool": tool, "forearm": forearm, "upperarm": upperarm}


def _pick_place_scene():
    cube = N("Solid", name="cube", defname="CUBE", physics=True,
             position=(0.45, 0.0, 0.8), velocity=(0.0, 0.001, 0.0))
    table = N("Solid", name="table", defname="TABLE", physics=False,
              position=(0.5, 0.15, 0.0))
    crate = N("Solid", name="crate", physics=False,
              children=[N("Solid", name="widget", physics=True,
                          position=(1.0, 1.0, 1.0),
                          velocity=(0.0, 0.0, 0.001))])
    arm = N("Robot", name="arm", defname="ARM", controller="pick",
            children=[])
    world_info = N("WorldInfo", translation=False)
    root = N("Group", children=[world_info, table, crate, cube, arm])
    return root, {"cube": cube, "table": table, "crate": crate, "arm": arm}


# --- 1. named non-robot Solids ------------------------------------------------


def test_a_nested_named_solid_gets_a_per_step_track(monkeypatch, tmp_path):
    """``--solids=`` reaches a body that is NOT a ``root.children`` entry.

    Upstream's recorder enumerated ``root.children`` only, so a dynamic part
    parented inside a crate had no row at all -- the same class of blind spot
    the OmniSim arm had for Solids generally. The named-solid walk is now
    scene-wide on both arms.
    """
    root, nodes = _pick_place_scene()
    widget = nodes["crate"].fields["children"].value[0]
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.064", "--contact-steps=0",
                         "--solids=widget"],
                        movers=[widget, nodes["cube"]])

    names = [b["name"] for b in out["trajectory"]["bodies"]]
    assert "widget" in names
    row = out["trajectory"]["bodies"][names.index("widget")]
    assert row["kind"] == "solid" and row["parent"] is None
    assert row["xyz"][-1][2] > row["xyz"][0][2]

    named = {r["name"]: r for r in out["roster"]["named_solids"]}
    assert named["widget"]["tracked"] is True
    # ...and the roster's own ``bodies`` list -- what the adapter turns into
    # the roster the cores count robots in -- did NOT grow.
    assert "widget" not in [b["name"] for b in out["roster"]["bodies"]]


def test_a_static_named_solid_is_not_tracked_but_says_why(monkeypatch,
                                                          tmp_path):
    """The cost bound, and why the already-frozen tasks did not move.

    Every named solid B2, C1, C2 and R3 ask for is static; upstream never
    tracked a static top-level body either (``has_physics or Robot``), so the
    ``dynamic`` default leaves both arms exactly where they were.
    """
    root, _n = _pick_place_scene()
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=0",
                         "--solids=table"])
    named = {r["name"]: r for r in out["roster"]["named_solids"]}
    assert named["table"]["tracked"] is False
    assert "no mass model" in named["table"]["track_reason"]
    assert "table" not in [b["name"] for b in out["trajectory"]["bodies"]]
    # ...and it is still in the ROSTER, which is where its bounds get merged.
    assert "table" in [b["name"] for b in out["roster"]["bodies"]]


def test_solid_tracks_all_reaches_a_kinematic_prop(monkeypatch, tmp_path):
    """``all`` is the escape hatch for a body a supervisor drives by hand."""
    root, nodes = _pick_place_scene()
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=0",
                         "--solids=table", "--solid-tracks=all"],
                        movers=[nodes["table"]])
    assert "table" in [b["name"] for b in out["trajectory"]["bodies"]]


def test_solid_tracks_none_restores_the_old_behaviour(monkeypatch, tmp_path):
    root, nodes = _pick_place_scene()
    widget = nodes["crate"].fields["children"].value[0]
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=0",
                         "--solids=widget", "--solid-tracks=none"],
                        movers=[widget])
    assert "widget" not in [b["name"] for b in out["trajectory"]["bodies"]]


def test_a_top_level_dynamic_body_is_still_tracked_once(monkeypatch, tmp_path):
    """Naming a body that upstream ALREADY tracked must not duplicate its row.

    A duplicate would make the same body two entries in the pose series, and
    ``Trajectory``'s index invariant would then be untrue for every row after
    it.
    """
    root, nodes = _pick_place_scene()
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=0",
                         "--solids=cube"],
                        movers=[nodes["cube"]])
    names = [b["name"] for b in out["trajectory"]["bodies"]]
    assert names.count("cube") == 1


# --- 2. links inside a robot --------------------------------------------------


def test_a_link_inside_a_robot_gets_a_per_step_track(monkeypatch, tmp_path):
    """R2's end effector, on the control arm. The symmetry assertion."""
    arm, nodes = _arm_scene()
    root = N("Group", children=[arm])
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.064", "--contact-steps=0", "--links=8"],
                        movers=[nodes["upperarm"], nodes["forearm"],
                                nodes["tool"]])

    bodies = out["trajectory"]["bodies"]
    assert [b["kind"] for b in bodies] == ["robot", "link", "link", "link"]
    # Structural ids, never the agent's naming choice.
    assert [b["name"] for b in bodies[1:]] == ["arm/link0", "arm/link1",
                                               "arm/link2"]
    assert all(b["parent"] == "arm" for b in bodies[1:])
    assert bodies[3]["xyz"][-1][0] > bodies[3]["xyz"][0][0]
    assert bodies[0]["xyz"][-1] == bodies[0]["xyz"][0]   # fixed base

    links = {r["key"]: r for r in out["roster"]["links"]}
    assert links["arm/link0"]["joint_endpoint"] is True
    assert links["arm/link2"]["joint_endpoint"] is False


def test_link_predicate_matches_the_omnisim_arms(monkeypatch, tmp_path):
    """Joint endPoint OR a Physics node; a visual prop is neither."""
    arm, _nodes = _arm_scene()
    names = [REC._sf_string(n, "name") for n, _ in REC._link_bodies(arm)]
    assert names == ["upperarm", "forearm", "tool"]

    kin = N("Robot", name="kin", children=[
        N("HingeJoint", end_point=N("Solid", name="kin_tip", physics=False))])
    assert [REC._sf_string(n, "name")
            for n, _ in REC._link_bodies(kin)] == ["kin_tip"]


def test_link_sampling_does_not_enter_a_nested_robot():
    inner = N("Robot", name="inner", children=[
        N("HingeJoint", end_point=N("Solid", name="inner_link", physics=True))])
    assert REC._link_bodies(N("Robot", name="outer", children=[inner])) == []


def test_the_link_cap_truncates_and_says_so(monkeypatch, tmp_path):
    arm, _nodes = _arm_scene()
    out = _run_recorder(monkeypatch, tmp_path, N("Group", children=[arm]),
                        ["--duration=0.032", "--contact-steps=0", "--links=2"])
    assert len(out["roster"]["links"]) == 2
    assert out["roster"]["links_truncated"] is True


def test_the_link_cap_ceiling_bounds_a_mis_set_task():
    assert REC._args.__defaults__ is None       # keyword-free helper
    saved = sys.argv
    try:
        sys.argv = ["r", "--links=100000"]
        assert REC._args()["links"] == REC._LINK_CAP_CEILING
    finally:
        sys.argv = saved


def test_links_are_off_unless_asked(monkeypatch, tmp_path):
    """The default that keeps every already-recorded task's shape."""
    arm, _nodes = _arm_scene()
    out = _run_recorder(monkeypatch, tmp_path, N("Group", children=[arm]),
                        ["--duration=0.032", "--contact-steps=0"])
    assert len(out["trajectory"]["bodies"]) == 1
    assert out["roster"]["links"] == []


# --- 3. the reader carries the rows through ----------------------------------


def test_to_arrays_carries_kind_and_parent(monkeypatch, tmp_path):
    arm, nodes = _arm_scene()
    out = _run_recorder(monkeypatch, tmp_path, N("Group", children=[arm]),
                        ["--duration=0.064", "--contact-steps=0", "--links=8"],
                        movers=[nodes["tool"]])
    arr = recording.to_arrays(out["trajectory"])
    assert arr.fatal is None
    assert arr.kinds == ["robot", "link", "link", "link"]
    assert arr.parents == [None, "arm", "arm", "arm"]


def test_an_old_trajectory_without_kinds_stays_unclassified():
    """Absence is not a guess: no ``kind`` key => no classification at all.

    Every trajectory.json recorded before 2026-08-09 looks like this, and
    reading its rows as ``"robot"`` would be inventing a witness.
    """
    arr = recording.to_arrays({"dt_s": 0.016, "bodies": [
        {"name": "a", "t": [0.0, 0.016], "xyz": [[0, 0, 0], [1, 0, 0]]}]})
    assert arr.kinds == [] and arr.parents == []


def test_the_bundle_exposes_links_and_keeps_robot_class_honest(monkeypatch,
                                                               tmp_path):
    """A forearm is part of the arm; it is not a robot, and not a body of the
    scene. R2.2 counts ``roster.robots`` and must still see exactly one arm."""
    arm, nodes = _arm_scene()
    out = _run_recorder(monkeypatch, tmp_path, N("Group", children=[arm]),
                        ["--duration=0.064", "--contact-steps=0", "--links=8"],
                        movers=[nodes["tool"]])
    (tmp_path / "process.json").write_text(
        json.dumps({"exit_code": 0, "timed_out": False,
                    "webots_version": "R2025a"}), encoding="utf-8")
    run = recording.read_run(tmp_path)
    bundle = wb_evidence.build_bundle("R2_arm_reach", run=run,
                                      artifact=str(tmp_path / "w.wbt"))
    traj = bundle.trajectory

    assert traj.rows_of_kind("link") == [1, 2, 3]
    assert traj.links_of("arm") == [1, 2, 3]
    assert traj.n_top_level == 1
    assert [b.name for b in bundle.roster.robots] == ["arm"]
    assert len(bundle.roster.bodies) == 1
    for b in bundle.t0.links_of("arm"):
        assert b.robot_class is False
    assert {b.name for b in bundle.t0.links_of("arm")} == {
        "upperarm", "forearm", "tool"}


def test_bounds_are_declared_not_expressible(monkeypatch, tmp_path):
    """The ``bodies`` list stays the prober's channel, byte for byte.

    Upstream has no Supervisor bounds query, so every AABB on this arm is
    COMPUTED. B1 / B2 / B3 / C2 are graded on the ones
    ``agentbench_aabb_prober`` computes in its own launch and merges into
    ``bodies`` (``task_support.augment_run``), and this recorder must not put a
    second source there -- a frozen row must not silently change which launch
    its numbers came from. The recorder's own measurements live in
    ``scene_bodies`` and stay there.
    """
    root, _n = _pick_place_scene()
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=0",
                         "--solids=table"])
    assert out["roster"]["bounds_supported"] is False
    assert "no Supervisor bounds query" in out["roster"]["bounds_note"]
    assert "aabb_prober" in out["roster"]["bounds_note"]
    for rec in out["roster"]["bodies"]:
        assert "aabb_min" not in rec and "bounds" not in rec
    # ...and the name-free channel exists next to it rather than instead of it.
    assert "scene_bodies" in out["roster"]


# --- 4. contacts name BOTH participants, scenery included --------------------
#
# Until 2026-08-09 this recorder queried only Robot nodes, and upstream's
# ContactPoint names the queried subtree and never the other party -- so a
# robot striking an obstacle or a wall produced a contact with ONE participant,
# which no collision assertion can count. R1.5 ("nothing was hit") therefore
# reported zero collisions for every R1 run on this arm, honest or not. These
# fixtures are the WSL-free half of the fix; the live half is
# ``test_r1_discriminates_webots.py::test_a_blind_driver_is_caught_by_the_collision_assertion``.

_HIT = (1.25, -0.5, 0.3)


def _collision_scene(schedule_robot, schedule_box, *, box_error=None):
    """One robot, one static box, and a scripted contact between them."""
    box = N("Solid", name="crate", defname="CRATE", physics=False,
            position=(1.5, -0.5, 0.25), schedule=schedule_box,
            contact_error=box_error)
    rover = N("Robot", name="rover", defname="ROVER", controller="drive",
              physics=True, position=(0.0, 0.0, 0.1),
              schedule=schedule_robot)
    root = N("Group", children=[N("WorldInfo", translation=False), box,
                                rover])
    return root, box, rover


def test_a_robot_scenery_contact_is_named(monkeypatch, tmp_path):
    """The defect, inverted: a robot/box contact now has two named ends."""
    root, box, rover = _collision_scene({3: [_HIT]}, {3: [_HIT]})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    doc = out["contacts"]
    assert doc["supported"] is True
    assert doc["robot_scenery_pairs"] == 1
    pair = [p for p in doc["pairs"] if p["b"] == "CRATE"]
    assert pair == [{"a": "rover", "b": "CRATE", "a_robot": True,
                     "b_robot": False, "point": list(_HIT), "step": 3}]


def test_the_scenery_channel_leaves_the_robot_side_counters_alone(monkeypatch,
                                                                  tmp_path):
    """A1.3 parity: it reads ``robot_robot_pairs`` and ``distinct_named``, and
    a scenery pair must not be able to masquerade as either. ``b_robot: False``
    keeps it out of the first; its own counters keep it out of the second."""
    root, box, rover = _collision_scene({2: [_HIT]}, {2: [_HIT]})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    doc = out["contacts"]
    assert doc["distinct_named"] == 0            # no robot-ROBOT pair existed
    assert doc["total_observed"] == 1            # the robot's own query only
    assert doc["scenery_observed"] == 1
    run = recording.read_run(tmp_path)
    bundle = wb_evidence.build_bundle("A1_husky_swarm_10", run=run,
                                      artifact=str(tmp_path / "w.wbt"))
    assert bundle.contacts.robot_robot_pairs == []
    assert bundle.contacts.can_name_a_robot_pair is False


def test_a_scenery_pair_is_recorded_once_however_long_the_contact_lasts(
        monkeypatch, tmp_path):
    """A robot resting against something for 3,750 steps is ONE fact.

    Without the dedupe a whole-run window would spend the 200-pair cap on the
    first contact it found -- the robot's own wheels on the floor -- and a
    later collision would fall off the end of the list unrecorded.
    """
    every = {k: [_HIT] for k in range(1, 11)}
    root, box, rover = _collision_scene(every, every)
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    assert out["contacts"]["robot_scenery_pairs"] == 1
    assert len([p for p in out["contacts"]["pairs"]
                if p["b"] == "CRATE"]) == 1
    assert out["contacts"]["scenery_observed"] == 10   # still counted per step


def test_only_a_matching_point_pairs(monkeypatch, tmp_path):
    """Both sides must report the SAME world point. A box touched by something
    else entirely is not a contact with this robot, and inferring one would be
    the ``(id, id)`` self-pair bug wearing different clothes."""
    root, box, rover = _collision_scene({3: [(9.0, 9.0, 9.0)]}, {3: [_HIT]})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    assert out["contacts"]["robot_scenery_pairs"] == 0
    assert out["contacts"]["scenery_observed"] == 1    # seen, just not paired


def test_one_body_refusing_the_query_does_not_blind_the_channel(monkeypatch,
                                                                tmp_path):
    """A single prop the engine will not answer for must not take the whole
    collision witness down with it -- that would turn one unmeasurable body
    into "nothing was hit" for the entire scene."""
    root, box, rover = _collision_scene({3: [_HIT]}, {3: [_HIT]},
                                        box_error=RuntimeError("nope"))
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    doc = out["contacts"]
    assert doc["supported"] is True
    assert doc["robot_scenery_pairs"] == 0
    assert "getContactPoints failed" in (doc["error"] or "")
    assert doc["total_observed"] == 1        # the robot side kept working


def test_contact_steps_minus_one_scans_the_whole_run(monkeypatch, tmp_path):
    """R1.5 is phrased over the WHOLE run, and the MuJoCo arm scans every step.
    A first-N-steps window can only ever witness a collision in the first N."""
    root, box, rover = _collision_scene({}, {})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=-1"],
                        scripted=(box, rover))
    doc = out["contacts"]
    assert doc["whole_run"] is True
    assert doc["requested_steps"] == -1
    assert doc["steps"] == out["completion"]["steps"] == 10
    assert doc["window_s"] == 0.16


def test_a_fixed_window_still_stops_where_it_is_told(monkeypatch, tmp_path):
    """The old behaviour is untouched for every task that asks for a window."""
    root, box, rover = _collision_scene({3: [_HIT]}, {3: [_HIT]})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=2"],
                        scripted=(box, rover))
    assert out["contacts"]["steps"] == 2
    assert out["contacts"]["whole_run"] is False
    assert out["contacts"]["robot_scenery_pairs"] == 0   # the hit was later


def test_a_zero_window_is_not_reported_as_supported(monkeypatch, tmp_path):
    """"We did not look" must never read as "nothing was hit".

    ``r1_core`` branches on ``supported`` alone, and a run that sampled zero
    contact steps has no evidence either way -- crediting it is the C2 defect
    in miniature. The reason travels with the flag so a reader can tell
    "this simulator has no contact query" from "this RUN was not asked for
    one".
    """
    root, box, rover = _collision_scene({1: [_HIT]}, {1: [_HIT]})
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.16", "--contact-steps=0"],
                        scripted=(box, rover))
    doc = out["contacts"]
    assert doc["supported"] is False
    assert doc["steps"] == 0
    assert "never looked" in doc["error"]
    assert "--contact-steps=-1" in doc["error"]
    run = recording.read_run(tmp_path)
    bundle = wb_evidence.build_bundle("R1_lidar_nav", run=run,
                                      artifact=str(tmp_path / "w.wbt"))
    assert bundle.contacts.supported is False


def test_the_scenery_participant_list_is_capped_and_says_so(monkeypatch,
                                                            tmp_path):
    """THE COST BOUND: one IPC round-trip per body per contact step, so a
    whole-run window on a crowded scene is bounded by the cap and not by the
    scene. Truncation is reported, never silent."""
    monkeypatch.setattr(REC, "_CONTACT_SCENERY_CAP", 2)
    boxes = [N("Solid", name="crate%d" % i, physics=False,
               position=(float(i), 0.0, 0.25)) for i in range(5)]
    rover = N("Robot", name="rover", controller="drive", physics=True)
    root = N("Group", children=[N("WorldInfo", translation=False)] + boxes
             + [rover])
    out = _run_recorder(monkeypatch, tmp_path, root,
                        ["--duration=0.032", "--contact-steps=-1"])
    assert out["contacts"]["scenery_participants"] == 2
    assert out["contacts"]["scenery_truncated"] is True


# --- 5. the launcher passes the knobs, and only when asked -------------------


def test_the_launcher_forwards_the_new_knobs():
    s = launcher.recorder_stanza("/out", duration=10.0, contact_steps=1,
                                 solids=("table", "bin"), links=8,
                                 solid_tracks="all")
    assert '"--solids=table,bin"' in s
    assert '"--links=8"' in s
    assert '"--solid-tracks=all"' in s


def test_the_launcher_stanza_is_unchanged_when_nothing_is_asked():
    """Backward compatibility on the control arm: no opt-in, no new args."""
    s = launcher.recorder_stanza("/out", duration=10.0, contact_steps=1)
    assert "--solids=" not in s
    assert "--links=" not in s
    assert "--solid-tracks=" not in s
    assert s.count('"--') == 3
