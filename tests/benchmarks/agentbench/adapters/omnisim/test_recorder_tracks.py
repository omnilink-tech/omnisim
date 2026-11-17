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

"""The OmniSim recorder's THREE track kinds, and the promise that adding two of
them changed nothing for the six tasks already frozen.

Until 2026-08-09 the recorder sampled ``Robot`` nodes and nothing else, which
is why R2 had no end effector to measure (an arm authored as one Robot exposes
only its fixed base) and R3 had to author its cube as a ``Robot`` node to get a
trajectory for a body that is plainly not a robot. Every test below fails
against that recorder:

    test_a_named_dynamic_solid_gets_a_per_step_track
    test_a_link_inside_a_robot_gets_a_per_step_track
    ... and the neutral-schema and adapter tests that read those rows.

And every test in ``TestTheSixFrozenTasksAreUntouched`` passes against BOTH,
which is the other half of the contract: the two new kinds are opt-in, they are
appended after the robot rows, and a task that asks for neither produces the
same CSV, the same header and the same ``body_ids`` it did before.

The scene graph here is a fake -- a handful of nodes with the four Supervisor
methods the recorder actually calls. That is deliberate and it is the only way
this file can exist: a live check would need ``omnisim-bin``, and the recorder's
whole job is to run inside it. What a fake CAN prove is the part that was
wrong: which bodies the walk selects, in what order, into which CSV column.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest

AGENTBENCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AGENTBENCH.parents[2]))

from agentbench import adapters  # noqa: E402
from agentbench import tasks as task_registry  # noqa: E402
from agentbench.adapters.omnisim import evidence as om_evidence  # noqa: E402
from agentbench.adapters.omnisim import headless  # noqa: E402

RECORDER_PATH = (AGENTBENCH / "controllers" / "agentbench_recorder"
                 / "agentbench_recorder.py")

#: The tasks frozen before the two new track kinds existed. Their evidence
#: shape is the backward-compatibility contract.
FROZEN_TASKS = ("A1_husky_swarm_10", "B1_overlap_audit", "B2_subject_in_frame",
                "B3_measure_and_report", "C1_parse_error_fix",
                "C2_fall_through_floor")


# --- loading the controller outside the engine -------------------------------


def _load_recorder():
    """Import the recorder module with a stub ``controller`` package.

    The controller imports ``from controller import Supervisor`` at module
    scope, which only resolves inside a running engine. Stubbing it is what
    makes the selection logic testable at all; ``Supervisor`` itself is
    replaced per-test by a fake, so the stub is never called.
    """
    stub = types.ModuleType("controller")
    stub.Supervisor = object
    saved = sys.modules.get("controller")
    sys.modules["controller"] = stub
    try:
        spec = importlib.util.spec_from_file_location(
            "_agentbench_recorder_under_test", RECORDER_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        if saved is None:
            sys.modules.pop("controller", None)
        else:
            sys.modules["controller"] = saved
    # The bounds helper is an engine-side import; without it every scan simply
    # reports no AABB, which is what this file wants (it measures TRACKS).
    mod._geometry = None
    mod._observe = None
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
        if self.kind != "mf":
            raise TypeError("not an MF field")
        return self.value[i]

    def getSFNode(self):
        if self.kind != "sf":
            raise TypeError("not an SF node field")
        return self.value

    def getSFString(self):
        if self.kind != "string":
            raise TypeError("not an SF string field")
        return self.value

    def getSFBool(self):
        if self.kind != "bool":
            raise TypeError("not an SF bool field")
        return self.value


_IDS = [100]


class N:
    """One scene node: exactly the surface the recorder touches."""

    def __init__(self, type_name, *, base=None, name=None, defname="",
                 children=None, end_point=None, physics=None,
                 controller=None, supervisor=None, position=(0.0, 0.0, 0.0),
                 velocity=(0.0, 0.0, 0.0)):
        _IDS[0] += 1
        self.type_name = type_name
        self.base = base or type_name
        self.node_id = _IDS[0]
        self.defname = defname
        self.position = list(position)
        self.velocity = list(velocity)
        self.position_reads = 0
        self.pose_reads = 0
        self.pose_tracking_enables = []
        self.contact_tracking_enables = []
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
        if supervisor is not None:
            self.fields["supervisor"] = _Field("bool", supervisor)

    # -- Supervisor Node API ------------------------------------------------
    def getField(self, name):
        return self.fields.get(name)

    def getTypeName(self):
        return self.type_name

    def getBaseTypeName(self):
        return self.base

    def getDef(self):
        return self.defname

    def getId(self):
        return self.node_id

    def getPosition(self):
        self.position_reads += 1
        return list(self.position)

    def enablePoseTracking(self, sampling_period, from_node=None):
        self.pose_tracking_enables.append((sampling_period, from_node))

    def getPose(self, from_node=None):
        self.pose_reads += 1
        x, y, z = self.position
        return [1.0, 0.0, 0.0, x,
                0.0, 1.0, 0.0, y,
                0.0, 0.0, 1.0, z,
                0.0, 0.0, 0.0, 1.0]

    def enableContactPointsTracking(self, sampling_period,
                                    include_descendants=False):
        self.contact_tracking_enables.append(
            (sampling_period, include_descendants))


def test_proto_derived_robot_keeps_robot_identity_in_recorder_evidence():
    """A custom PROTO name must not erase its declared Robot base type."""
    recorder = _load_recorder()
    node = N("ScaleBot", base="Robot", name="scale_000")
    assert recorder._is_robot(node)

    record = {"id": node.getId(), "name": "scale_000", "type": "ScaleBot",
              "base_type": "Robot", "controller": "drive",
              "num_joints": 0}

    def is_any_robot(kind, _n_joints, base_type=None):
        return kind == "Robot" or base_type == "Robot"

    body = om_evidence._body_from_recorder(record, is_any_robot)
    assert body.robot_class is True
    assert "base_type=Robot" in body.identity_evidence


def test_pose_rows_use_the_step_batched_tracking_cache(monkeypatch, tmp_path):
    """Per-step sampling must not issue one synchronous getPosition per body."""
    root, robots = _swarm_scene(3)
    rows, meta, _phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.048", "--settle=0.0", "--phase-a=0"],
        movers=robots)

    assert meta["pose_sampling"] == {
        "mode": "tracked_pose_step_batch", "sampling_period_ms": 16,
        "requested": 3, "enabled": 3, "fallback": 0,
        "read_fallbacks": 0, "errors": []}
    assert len(rows) == 5  # header + t=0 + three stepped samples
    assert all(r.pose_tracking_enables == [(16, None)] for r in robots)
    assert all(r.pose_reads == 4 for r in robots)
    assert all(r.position_reads == 0 for r in robots)


class FakeSupervisor:
    """A frozen world that advances its movers by a fixed step."""

    def __init__(self, root, dt_ms=16.0, movers=()):
        self.root = root
        self.dt_ms = dt_ms
        self.movers = list(movers)
        self.steps = 0
        self.quit_code = None

    def getRoot(self):
        return self.root

    def getBasicTimeStep(self):
        return self.dt_ms

    def getWorldPath(self):
        return "/fake/world.omniworld"

    def step(self, _ms):
        self.steps += 1
        for node in self.movers:
            for k in range(3):
                node.position[k] += node.velocity[k]
        return 0

    def simulationQuit(self, code):
        self.quit_code = code


def _run_recorder(monkeypatch, tmp_path, root, argv, *, movers=(), dt_ms=16.0):
    """Drive ``REC.main()`` on a fake world -> (csv rows, meta, phase_a)."""
    out = tmp_path / "phaseB.csv"
    sup = FakeSupervisor(root, dt_ms=dt_ms, movers=movers)
    monkeypatch.setattr(REC, "Supervisor", lambda: sup)
    monkeypatch.setattr(sys, "argv",
                        ["agentbench_recorder", "--out=%s" % out] + list(argv))
    REC.main()
    with open(out, newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    meta = json.loads(Path(str(out) + ".meta.json").read_text(encoding="utf-8"))
    pa_path = Path(str(out) + ".phase_a.json")
    phase_a = (json.loads(pa_path.read_text(encoding="utf-8"))
               if pa_path.is_file() else None)
    return rows, meta, phase_a


# --- scenes ------------------------------------------------------------------


def _arm_scene():
    """A fixed-base 2-joint arm with a rigid tool, plus scenery.

    The shape R2 actually gets: ONE ``Robot`` node whose links hang off joints
    inside it. Every link moves; the Robot's own origin does not.
    """
    tool = N("Solid", name="tool", physics=True, position=(0.4, 0.0, 0.5),
             velocity=(0.001, 0.0, 0.0))
    decoration = N("Solid", name="nameplate", physics=False,
                   position=(0.0, 0.0, 0.2))
    link1 = N("Solid", name="forearm", physics=True,
              children=[tool], position=(0.3, 0.0, 0.5),
              velocity=(0.001, 0.0, 0.0))
    joint1 = N("HingeJoint", end_point=link1)
    link0 = N("Solid", name="upperarm", physics=True,
              children=[joint1, decoration], position=(0.1, 0.0, 0.4),
              velocity=(0.0005, 0.0, 0.0))
    joint0 = N("HingeJoint", end_point=link0)
    arm = N("Robot", name="arm", defname="ARM", controller="reach",
            physics=False, children=[joint0], position=(0.0, 0.0, 0.0))
    return arm, {"tool": tool, "forearm": link1, "upperarm": link0,
                 "nameplate": decoration}


def _pick_place_scene():
    """R3's shape once the cube is what it actually is: a dynamic ``Solid``."""
    cube = N("Solid", name="cube", defname="CUBE", physics=True,
             position=(0.45, 0.0, 0.8), velocity=(0.0, 0.001, 0.0))
    table = N("Solid", name="table", defname="TABLE", physics=False,
              position=(0.5, 0.15, 0.0))
    bin_ = N("Solid", name="bin", defname="BIN", physics=False,
             position=(0.3, 0.35, 0.8))
    arm = N("Robot", name="arm", defname="ARM", controller="pick",
            children=[], position=(0.0, 0.0, 0.0))
    root = N("Group", children=[table, bin_, cube, arm])
    return root, {"cube": cube, "table": table, "bin": bin_, "arm": arm}


def _swarm_scene(n=3):
    """A1's shape: N wheeled Robots, each with wheels the walk must NOT add."""
    robots = []
    for i in range(n):
        wheels = [N("Solid", name="wheel_%d_%d" % (i, w), physics=True,
                    position=(float(i), float(w), 0.1))
                  for w in range(4)]
        joints = [N("HingeJoint", end_point=w) for w in wheels]
        robots.append(N("Robot", name="husky_%d" % i, defname="HUSKY_%d" % i,
                        controller="drive", physics=True, children=joints,
                        position=(float(i), 0.0, 0.2),
                        velocity=(0.01, 0.0, 0.0)))
    floor = N("Solid", name="floor", physics=False)
    return N("Group", children=[floor] + robots), robots


# --- 1. a named non-robot Solid gets a per-step track ------------------------


def test_a_named_dynamic_solid_gets_a_per_step_track(monkeypatch, tmp_path):
    """R3's cube: named in ``--solids=``, it now has xyz over time.

    Before this change ``--solids=`` produced a t=0 bounds entry and nothing
    else, so a grader asking "did the cube leave the table, get carried and
    land in the bin" had no series to ask it of. This is the assertion that
    fails against the old recorder.
    """
    root, nodes = _pick_place_scene()
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.048", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=table,bin,cube"],
        movers=[nodes["cube"]])

    kinds = [tr["kind"] for tr in meta["tracks"]]
    assert kinds == ["robot", "solid"], kinds
    solid = meta["tracks"][1]
    assert solid["name"] == "cube"
    assert solid["column"] == "s0"

    # The header keeps the robot columns exactly where they were and appends.
    assert rows[0] == ["t", "r0_x", "r0_y", "r0_z", "s0_x", "s0_y", "s0_z"]

    # ...and the column actually MOVES -- a track, not a repeated t=0 pose.
    ys = [float(r[5]) for r in rows[1:]]
    assert len(ys) >= 4 and ys[-1] > ys[0]

    # The t=0 bounds half of the --solids contract is still there, and every
    # named solid now says whether it got a track and why.
    by_name = {e["name"]: e for e in phase_a["t0_solids"]}
    assert set(by_name) == {"table", "bin", "cube"}
    assert by_name["cube"]["tracked"] is True
    assert by_name["cube"]["track_index"] == 1
    assert by_name["table"]["tracked"] is False
    assert "no mass model" in by_name["table"]["track_reason"]


def test_a_static_named_solid_is_not_tracked_but_says_why(monkeypatch,
                                                          tmp_path):
    """The cost bound, and the reason the six frozen tasks did not move.

    Every named solid B2, C1, C2 and R3 ask for is STATIC, so ``dynamic`` --
    the default -- gives them a bounds entry and no row, exactly as before.
    """
    root, nodes = _pick_place_scene()
    _rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=table,bin"])
    assert meta["n_solid_tracks"] == 0
    assert [tr["kind"] for tr in meta["tracks"]] == ["robot"]


def test_solid_tracks_all_reaches_a_kinematic_prop(monkeypatch, tmp_path):
    """``all`` is the escape hatch for a body a supervisor drives by hand.

    It has no mass model, so ``dynamic`` correctly declines it -- but it does
    move, and a task that knows that can say so.
    """
    root, _nodes = _pick_place_scene()
    _rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=table", "--solid-tracks=all"])
    assert meta["n_solid_tracks"] == 1
    assert meta["tracks"][1]["name"] == "table"


def test_solid_tracks_none_restores_the_bounds_only_behaviour(monkeypatch,
                                                              tmp_path):
    root, nodes = _pick_place_scene()
    _rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=cube", "--solid-tracks=none"],
        movers=[nodes["cube"]])
    assert meta["n_solid_tracks"] == 0
    assert phase_a["t0_solids"][0]["name"] == "cube"
    assert phase_a["t0_solids"][0]["tracked"] is False


def test_a_robot_free_world_with_a_dynamic_solid_still_records(monkeypatch,
                                                               tmp_path):
    """The no-robots early exit used to be unconditional.

    It wrote a header-only CSV and quit, so a scene whose only moving body was
    a Solid produced no motion evidence at all. It now quits only when there is
    genuinely nothing to sample.
    """
    cube = N("Solid", name="cube", physics=True, position=(0.0, 0.0, 1.0),
             velocity=(0.0, 0.0, -0.01))
    root = N("Group", children=[cube])
    rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.064", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=cube"],
        movers=[cube])
    assert meta["n_robots"] == 0 and meta["n_solid_tracks"] == 1
    assert rows[0] == ["t", "s0_x", "s0_y", "s0_z"]
    assert float(rows[-1][3]) < float(rows[1][3])   # it fell
    assert meta["complete"] is True


def test_a_robot_free_world_with_nothing_dynamic_still_quits_clean(monkeypatch,
                                                                   tmp_path):
    """B2's exact shape: five static props, no Robot. Unchanged."""
    root, _n = _pick_place_scene()
    root.fields["children"] = _Field(
        "mf", [c for c in root.fields["children"].value
               if c.getTypeName() != "Robot"])
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=1.0", "--settle=0.0", "--phase-a=1", "--contact-steps=1",
         "--solids=table,bin"])
    assert rows == [["t"]]
    assert meta["complete"] is True and meta["n_tracks"] == 0
    assert len(phase_a["t0_solids"]) == 2


# --- 2. a link inside a robot gets a per-step track --------------------------


def test_a_link_inside_a_robot_gets_a_per_step_track(monkeypatch, tmp_path):
    """R2's end effector: the tip of an arm authored as ONE Robot node.

    This is the assertion R2's ``meta.json`` 'status' block is about. Against
    the old recorder the arm contributes one row -- its stationary base -- and
    ``select_end_effector`` reports ``candidates = 0``.
    """
    arm, nodes = _arm_scene()
    root = N("Group", children=[arm])
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.048", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--links=8"],
        movers=[nodes["upperarm"], nodes["forearm"], nodes["tool"]])

    kinds = [tr["kind"] for tr in meta["tracks"]]
    assert kinds == ["robot", "link", "link", "link"], kinds
    assert [tr["name"] for tr in meta["tracks"][1:]] == [
        "upperarm", "forearm", "tool"]

    # The identity is STRUCTURAL -- "link k of robot X" -- never the name the
    # agent happened to choose (the R1 mistake).
    assert [tr["body_id"] for tr in meta["tracks"][1:]] == [
        "ARM/link0", "ARM/link1", "ARM/link2"]
    assert all(tr["parent_body_id"] == "ARM" for tr in meta["tracks"][1:])

    assert rows[0][:4] == ["t", "r0_x", "r0_y", "r0_z"]
    assert rows[0][4:] == ["l0_x", "l0_y", "l0_z", "l1_x", "l1_y", "l1_z",
                           "l2_x", "l2_y", "l2_z"]
    # The tip moved while the fixed base did not: exactly what R2.4 asks.
    assert float(rows[-1][10]) > float(rows[1][10])
    assert float(rows[-1][1]) == float(rows[1][1])

    by_id = {e["body_id"]: e for e in phase_a["t0_links"]}
    assert by_id["ARM/link0"]["joint_endpoint"] is True
    assert by_id["ARM/link2"]["joint_endpoint"] is False   # rigid tool child


def test_link_sampling_skips_bodies_that_are_neither_jointed_nor_massed():
    """The predicate, stated as a test: what is a link and what is decoration.

    A joint ``endPoint`` counts even with no mass model (a KINEMATIC arm still
    actuates and still reaches the targets, so requiring Physics would report
    'no end effector' for an arm that did the task), and a Physics-carrying
    child counts even with no joint (a rigidly-mounted tool). A visual prop is
    neither and is not sampled.
    """
    arm, nodes = _arm_scene()
    got = REC._link_bodies(arm)
    names = [REC._sf_string(n, "name") for n, _ in got]
    assert names == ["upperarm", "forearm", "tool"]
    assert "nameplate" not in names

    kinematic_tip = N("Solid", name="kin_tip", physics=False)
    kin_arm = N("Robot", name="kin",
                children=[N("HingeJoint", end_point=kinematic_tip)])
    assert [REC._sf_string(n, "name") for n, _ in REC._link_bodies(kin_arm)] \
        == ["kin_tip"]


def test_link_sampling_does_not_enter_a_nested_robot():
    """A nested Robot is tracked as a robot ROW; entering it would double-count
    one body and make the roster-index invariant a lie."""
    inner_link = N("Solid", name="inner_link", physics=True)
    inner = N("Robot", name="inner",
              children=[N("HingeJoint", end_point=inner_link)])
    outer = N("Robot", name="outer", children=[inner])
    assert REC._link_bodies(outer) == []


def test_the_link_cap_truncates_and_says_so(monkeypatch, tmp_path):
    """The declared cost bound. Truncation is reported, never silent."""
    arm, nodes = _arm_scene()
    root = N("Group", children=[arm])
    _rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--links=2"],
        movers=[nodes["tool"]])
    assert meta["n_link_tracks"] == 2
    assert meta["links_truncated"] is True
    assert meta["robots"][0]["num_link_bodies_found"] == 3
    assert meta["robots"][0]["num_link_tracks"] == 2


def test_the_link_cap_ceiling_bounds_a_mis_set_task(monkeypatch, tmp_path):
    arm, _nodes = _arm_scene()
    root = N("Group", children=[arm])
    _rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.016", "--settle=0.0", "--phase-a=0", "--links=100000"])
    assert meta["link_cap"] == REC.LINK_CAP_CEILING


def test_links_are_off_unless_asked(monkeypatch, tmp_path):
    """The default that keeps A1 passing: 3 Huskies, 12 wheels, 3 rows."""
    root, robots = _swarm_scene(3)
    rows, meta, _pa = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0"],
        movers=robots)
    assert meta["n_tracks"] == 3 == meta["n_robots"]
    assert rows[0] == ["t"] + ["r%d_%s" % (i, a) for i in range(3)
                              for a in "xyz"]


# --- 3. the neutral schema and the adapter carry the rows through ------------


def _contract_problems(bundle):
    """``adapters.check_bundle`` minus the one finding this FAKE causes.

    The bounds helper is an engine-side import, so no body in a fake-scene run
    carries a world-space AABB and ``check_bundle`` says so -- correctly, and
    about the harness rather than about the code under test. Every OTHER
    contract rule is asserted in full, including the one that matters most
    here: ``len(body_ids) == xyz.shape[0]``, i.e. no track row is anonymous.
    """
    aabb = "t0 inventory carries no world-space AABB"
    return [p for p in adapters.check_bundle(bundle) if aabb not in p]


def _phase_b(tmp_path, rows, meta):
    """A ``PhaseBResult`` from an already-written recorder output."""
    res = headless.PhaseBResult(tmp_path)
    res.rc = 0
    res.meta = meta
    res.log_text = "[OmNewtonBackend] world finalised (solver=mujoco)\n"
    res.sidecar = {"present": True, "solver": "mujoco", "finalised": True}
    import numpy as np
    data = np.array([[float(v) for v in r] for r in rows[1:]], dtype=float)
    res.t = data[:, 0]
    n = (data.shape[1] - 1) // 3
    res.xyz = np.empty((n, data.shape[0], 3), dtype=float)
    for i in range(n):
        res.xyz[i] = data[:, 1 + 3 * i: 4 + 3 * i]
    return res


def test_the_bundle_lets_a_grader_ask_for_the_cube_by_name(monkeypatch,
                                                           tmp_path):
    """The requirement, end to end: name in, xyz over time out."""
    root, nodes = _pick_place_scene()
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.048", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--solids=table,bin,cube"],
        movers=[nodes["cube"]])
    res = _phase_b(tmp_path, rows, meta)
    res.phase_a = phase_a
    bundle = om_evidence.build_bundle("R3_pick_and_place", phase_b=res,
                                      artifact=str(tmp_path / "w.wbt"))
    traj = bundle.trajectory

    row = traj.index_of_name("cube")
    assert row == 1
    assert traj.kind_of(row) == "solid"
    assert traj.xyz[row].shape[1] == 3 and traj.xyz[row][-1][1] > \
        traj.xyz[row][0][1]

    # ...and the t=0 body and the pose row resolve to the SAME body, which is
    # how r3_core's trajectory_row() crosses the two inventories.
    cube_body = bundle.t0.by_name("cube")
    assert cube_body is not None
    assert traj.index_of(cube_body.body_id) == row

    assert _contract_problems(bundle) == []


def test_a_link_row_is_identified_without_a_name(monkeypatch, tmp_path):
    """``kinds`` + ``parents`` ARE the sim-neutral identity of a link."""
    arm, nodes = _arm_scene()
    root = N("Group", children=[arm])
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.048", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--links=8"],
        movers=[nodes["upperarm"], nodes["forearm"], nodes["tool"]])
    res = _phase_b(tmp_path, rows, meta)
    res.phase_a = phase_a
    bundle = om_evidence.build_bundle("R2_arm_reach", phase_b=res,
                                      artifact=str(tmp_path / "w.wbt"))
    traj = bundle.trajectory

    assert traj.rows_of_kind("link") == [1, 2, 3]
    assert traj.links_of("ARM") == [1, 2, 3]
    assert traj.n_top_level == 1
    assert traj.n_bodies == 4
    assert _contract_problems(bundle) == []


def test_r2_can_now_see_an_end_effector(monkeypatch, tmp_path):
    """The gap, closed, measured with R2's OWN selector.

    ``select_end_effector`` is the function that reported
    ``end_effector_candidates = 0`` on every R2 run; handed a recording with
    link rows it picks the tip -- the body furthest along the chain, chosen by
    geometry and never by name.
    """
    from agentbench.graders import r2_core

    arm, nodes = _arm_scene()
    root = N("Group", children=[arm])
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.16", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--links=8"],
        movers=[nodes["upperarm"], nodes["forearm"], nodes["tool"]])
    res = _phase_b(tmp_path, rows, meta)
    res.phase_a = phase_a
    bundle = om_evidence.build_bundle("R2_arm_reach", phase_b=res,
                                      artifact=str(tmp_path / "w.wbt"))
    traj = bundle.trajectory

    ee_row, report = r2_core.select_end_effector(traj.xyz, 0)
    assert report["candidates"] == 3
    assert ee_row is not None
    # The tool travels furthest per sample, so it is the tip.
    assert traj.name_of(ee_row) == "tool"


def test_robot_class_stays_honest_for_links_and_solids(monkeypatch, tmp_path):
    """A cube is not a robot, and neither is a forearm.

    R1.2, R2.2 and R3.3 all count ``roster.robots``; a link or a prop reading
    as robot-class would turn one arm into four and fail a correct world.
    """
    arm, nodes = _arm_scene()
    cube = N("Solid", name="cube", defname="CUBE", physics=True)
    root = N("Group", children=[arm, cube])
    rows, meta, phase_a = _run_recorder(
        monkeypatch, tmp_path, root,
        ["--duration=0.032", "--settle=0.0", "--phase-a=1",
         "--contact-steps=0", "--links=8", "--solids=cube"],
        movers=[nodes["tool"]])
    res = _phase_b(tmp_path, rows, meta)
    res.phase_a = phase_a
    bundle = om_evidence.build_bundle("R2_arm_reach", phase_b=res,
                                      artifact=str(tmp_path / "w.wbt"))

    assert [b.name for b in bundle.roster.robots] == ["arm"]
    assert [b.name for b in bundle.t0.robots] == ["arm"]
    for name in ("upperarm", "forearm", "tool", "cube"):
        body = bundle.t0.by_name(name)
        assert body is not None and body.robot_class is False, name
    # Membership is carried by member_of, so nothing has to lie to express it.
    assert {b.name for b in bundle.t0.links_of("ARM")} == {
        "upperarm", "forearm", "tool"}
    assert bundle.t0.by_name("cube").member_of is None


# --- 4. the six frozen tasks' evidence shape is unchanged --------------------


class TestTheSixFrozenTasksAreUntouched:
    """The backward-compatibility contract, asserted rather than hoped for."""

    @pytest.mark.parametrize("task_id", FROZEN_TASKS)
    def test_no_frozen_task_opts_in(self, task_id):
        """Neither new key appears in any already-frozen task's meta.

        Both are opt-in precisely because row COUNT is load-bearing: a1_core
        asserts ``n == 10`` and would fail a correct swarm handed forty wheel
        tracks.
        """
        sa = task_registry.get(task_id).standalone
        assert "links" not in sa
        assert "solid_tracks" not in sa

    @pytest.mark.parametrize("task_id", FROZEN_TASKS)
    def test_the_injected_stanza_is_unchanged(self, task_id, tmp_path):
        """No opt-in => no new controllerArgs => the same world text."""
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
            solid_tracks=sa.get("solid_tracks"))
        text = sib.read_text(encoding="utf-8")
        assert "--links=" not in text
        assert "--solid-tracks=" not in text

    def test_a_solids_task_still_gets_robots_only_rows(self, monkeypatch,
                                                       tmp_path):
        """C2's exact request -- one robot, ``--solids=floor`` -- is one row.

        The floor is static, so the default ``dynamic`` mode declines it and
        ``roster.index_of_name(...) -> traj.z(i)``, which c2_core does, still
        lands on the same row it always did.
        """
        floor = N("Solid", name="floor", physics=False)
        bot = N("Robot", name="crate_bot", defname="CRATE_BOT",
                controller="fall", physics=True, children=[],
                position=(0.0, 0.0, 1.0), velocity=(0.0, 0.0, -0.01))
        root = N("Group", children=[floor, bot])
        rows, meta, phase_a = _run_recorder(
            monkeypatch, tmp_path, root,
            ["--duration=0.064", "--settle=0.0", "--phase-a=1",
             "--contact-steps=0", "--solids=floor"],
            movers=[bot])
        assert rows[0] == ["t", "r0_x", "r0_y", "r0_z"]
        assert meta["n_tracks"] == meta["n_robots"] == 1
        assert [tr["kind"] for tr in meta["tracks"]] == ["robot"]
        assert [e["name"] for e in phase_a["t0_solids"]] == ["floor"]

        res = _phase_b(tmp_path, rows, meta)
        res.phase_a = phase_a
        bundle = om_evidence.build_bundle("C2_fall_through_floor",
                                          phase_b=res,
                                          artifact=str(tmp_path / "w.wbt"))
        idx = bundle.roster.index_of_name("crate_bot")
        assert idx == 0
        assert bundle.trajectory.z(idx)[-1] < bundle.trajectory.z(idx)[0]
        assert bundle.trajectory.body_ids == ["CRATE_BOT"]

    def test_robot_body_ids_are_the_same_whichever_channel_supplies_them(
            self, monkeypatch, tmp_path):
        """THE linchpin of the whole backward-compatibility claim.

        The trajectory's ids used to come from the roster; they now come from
        the recorder's track map. If the two conventions ever diverged, every
        already-published row's ``body_ids`` would silently change and
        ``r3_core.trajectory_row`` would stop resolving a body it used to find.
        Checked on DEF-LESS robots, where the id is the node id and there is no
        DEF to make the two agree by accident.
        """
        bots = [N("Robot", name="bot_%d" % i, controller="drive",
                  physics=True, children=[], position=(float(i), 0.0, 0.0))
                for i in range(3)]
        root = N("Group", children=bots)
        _rows, meta, _pa = _run_recorder(
            monkeypatch, tmp_path, root,
            ["--duration=0.032", "--settle=0.0", "--phase-a=1",
             "--contact-steps=0"])
        legacy = [om_evidence._recorder_id(r) for r in meta["robots"]]
        assert [tr["body_id"] for tr in meta["tracks"]] == legacy
        assert all(i.startswith("#") for i in legacy)

    def test_a_recording_with_no_track_map_reads_exactly_as_before(self,
                                                                   tmp_path):
        """A pre-2026-08-09 phaseB: robots only, ids from the roster.

        Every already-published row was produced by that recorder, so the
        adapter has to keep reading it identically or the frozen results stop
        being reproducible.
        """
        import numpy as np
        res = headless.PhaseBResult(tmp_path)
        res.rc = 0
        res.sidecar = {"present": True, "solver": "mujoco"}
        res.meta = {"dt_ms": 16.0, "recorded_s": 0.032, "complete": True,
                    "quit_called": True, "rows": 3,
                    "robots": [{"name": "husky_0", "def": "H0", "id": 7,
                                "type": "Robot", "controller": "drive",
                                "num_joints": 4, "has_physics": True},
                               {"name": "husky_1", "def": "", "id": 9,
                                "type": "Robot", "controller": "drive",
                                "num_joints": 4, "has_physics": True}]}
        res.t = np.array([0.0, 0.016, 0.032])
        res.xyz = np.zeros((2, 3, 3))
        bundle = om_evidence.build_bundle("A1_husky_swarm_10", phase_b=res,
                                          artifact=str(tmp_path / "w.wbt"))
        assert bundle.trajectory.body_ids == ["H0", "#9"]
        assert bundle.trajectory.kinds == []
        assert bundle.trajectory.n_top_level == 2 == bundle.trajectory.n_bodies
        assert res.n_robots == 2


# --- the contact VACUITY WITNESS on a task with no phase-A window ------------
#
# MEASURED on the first R4/omnisim cell (2026-08-11). `graders/r4_core` gates
# R4.5 ("the base drove the route without striking anything") on a falsifier --
# `supported and total_observed > 0`, i.e. "could this channel have reported a
# collision at all" -- and reads that counter off the phase-A window. R4's task
# meta asks for `contact_steps: -1`, which this recorder clamps to a single
# PRE-STEP sample, and a pre-step sample observes nothing because the engine
# has not stepped. So the witness said 0 while the run-long watch was holding
# 25,209 contacts and had NAMED the robot's collision with OBSTACLE_2, and
# R4.5 was un-passable on this arm whatever a robot did. That is the mirror of
# the defect R1 shipped (a collision clause that could not go red).

from agentbench.adapters.omnisim import evidence as om_evidence   # noqa: E402
from agentbench.graders.evidence import EvidenceBundle            # noqa: E402


def _run_doc(**kw):
    doc = {"supported": True, "every_n_steps": 8, "steps_sampled": 2344,
           "total_observed": 25209, "distinct_named": 25165,
           "pairs": [{"a": "warehouse manipulator", "b": "OBSTACLE_2",
                      "b_robot": False, "point": [0, 0, 0], "first_step": 2864,
                      "last_step": 2896, "count": 5}]}
    doc.update(kw)
    return doc


def _bundle():
    return EvidenceBundle(task="T", sim="omnisim", adapter="test")


def _channel_live(con):
    """r4_core's own falsifier, transcribed -- the thing that must go true."""
    return bool(con is not None and con.supported
                and (con.total_observed or 0) > 0)


def test_a_zero_width_phase_a_window_gets_the_run_long_witness():
    b = _bundle()
    om_evidence._apply_run_contacts(b, _run_doc(), phase_a_steps=0)
    assert b.contacts.total_observed == 25209
    assert b.contacts.distinct_named == 25165
    assert _channel_live(b.contacts), \
        "R4.5's falsifier is still false while 25209 contacts were observed"
    # ...and the collision itself is still in the pair list, so the assertion
    # that CAN now pass also still goes red on a real hit.
    assert [p.b for p in b.contacts.pairs] == ["OBSTACLE_2"]


def test_a_real_phase_a_window_keeps_its_own_counters():
    """A1 asks for contact_steps 10, and its rows must stay reproducible.

    The counters are a witness for a question about t=0 there. Overwriting
    them with a whole-run number would re-scope a frozen measurement without
    any physical fact having changed -- the same argument that keeps the
    run-long robot-robot pairs out of the phase-A list.
    """
    b = _bundle()
    b.contacts = om_evidence.ContactObservation(
        pairs=[], steps=10, supported=True, total_observed=0,
        distinct_named=0, source="phase A")
    om_evidence._apply_run_contacts(b, _run_doc(), phase_a_steps=10)
    assert b.contacts.total_observed == 0, "A1's t=0 witness was overwritten"
    assert b.contacts.distinct_named == 0
    assert b.contacts.steps == 10


def test_a_watch_that_never_sampled_leaves_the_witness_alone():
    """No samples is not an observation of zero, and must not read as one."""
    b = _bundle()
    b.contacts = om_evidence.ContactObservation(
        pairs=[], steps=0, supported=True, total_observed=None,
        distinct_named=None, source="phase A")
    om_evidence._apply_run_contacts(
        b, _run_doc(steps_sampled=0, total_observed=None, pairs=[]),
        phase_a_steps=0)
    assert b.contacts.total_observed is None
    assert not _channel_live(b.contacts)
