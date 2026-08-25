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

"""The grader-owned sampler's tier half, with a fake scene instead of an engine.

``ladder_recorder`` only runs inside ``omnisim-bin``, which makes the parts of
it that decide a cell -- what counts as a support route, which mass is
readable, how coarse the pose series may be -- exactly the parts that are
hardest to test. So the module is imported against a **stub ``controller``**
and driven with a fake scene graph.

Every case below is one that was **measured wrong first**:

* ``WorldInfo.physics`` reads as the literal string ``"<none>"`` on a scene
  with no plugin, and treating that as a declared plugin opened a support
  route on a clean T3 probe and turned an otherwise perfect structural
  attestation into ``unverified``;
* ``WorldInfo.gravity`` is an **SFFloat** in this engine and an SFVec3f
  upstream, and reading only the vector form reported gravity absent on a
  world that plainly had it -- which is T3.4's and T2.4's datum;
* a fixed record stride of 5 on a 16 ms world gives a 0.08 s pose interval,
  and T3.1's continuity clause needs 0.05 s or finer, so it went vacuous.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

LADDER = Path(__file__).resolve().parents[2]
RECORDER = LADDER / "controllers" / "ladder_recorder" / "ladder_recorder.py"


def _load_recorder():
    """Import the controller with a stub ``controller`` package in place."""
    if "controller" not in sys.modules:
        stub = types.ModuleType("controller")
        stub.Supervisor = object
        sys.modules["controller"] = stub
    spec = importlib.util.spec_from_file_location("_ladder_recorder_under_test",
                                                  RECORDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rec = _load_recorder()


# --- a fake scene graph -------------------------------------------------------


class Field:
    def __init__(self, value, kind):
        self.value = value
        self.kind = kind

    def _as(self, kind):
        if self.kind != kind:
            raise TypeError("field is %s, not %s" % (self.kind, kind))
        return self.value

    def getSFString(self):
        return self._as("string")

    def getSFBool(self):
        return self._as("bool")

    def getSFFloat(self):
        return self._as("float")

    def getSFVec3f(self):
        return self._as("vec3")

    def getSFNode(self):
        return self._as("node")

    def getCount(self):
        return len(self._as("mfnode"))

    def getMFNode(self, i):
        return self._as("mfnode")[i]


class Node:
    _next = [1]

    def __init__(self, type_name, name="", *, children=(), physics=None,
                 supervisor=None, controller_name=None, endpoint=None,
                 position=(0.0, 0.0, 0.0), extra=None, def_=""):
        self.type_name = type_name
        self.id = Node._next[0]
        Node._next[0] += 1
        self.def_ = def_
        self.position = list(position)
        self.parent = None
        self.fields = {}
        if name:
            self.fields["name"] = Field(name, "string")
        self.fields["children"] = Field(list(children), "mfnode")
        self.fields["physics"] = Field(physics, "node")
        if supervisor is not None:
            self.fields["supervisor"] = Field(supervisor, "bool")
        if controller_name is not None:
            self.fields["controller"] = Field(controller_name, "string")
        if endpoint is not None:
            self.fields["endPoint"] = Field(endpoint, "node")
        self.fields.update(extra or {})
        for c in children:
            c.parent = self
        if endpoint is not None:
            endpoint.parent = self

    # -- the Supervisor Node API the sampler uses ---------------------------
    def getTypeName(self):
        return self.type_name

    def getDef(self):
        return self.def_

    def getId(self):
        return self.id

    def getField(self, name):
        return self.fields.get(name)

    def getPosition(self):
        return list(self.position)

    def getOrientation(self):
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def getParentNode(self):
        return self.parent


def physics(mass):
    return Node("Physics", extra={"mass": Field(mass, "float")})


class Supervisor:
    def __init__(self, root):
        self._root = root

    def getRoot(self):
        return self._root


def scene(*children, world_info=None):
    kids = list(children)
    if world_info is not None:
        kids.insert(0, world_info)
    return Supervisor(Node("Group", children=kids))


# --- mass ---------------------------------------------------------------------


def test_a_stated_mass_is_read_in_kilograms():
    n = Node("Solid", "block", physics=physics(0.2))
    assert rec._own_mass(n) == (0.2, None)              # noqa: SLF001


def test_a_body_with_no_physics_node_has_no_mass_and_says_why():
    m, why = rec._own_mass(Node("Solid", "floor"))      # noqa: SLF001
    assert m is None and "static" in why


def test_a_density_based_mass_is_refused_rather_than_multiplied_out():
    m, why = rec._own_mass(                             # noqa: SLF001
        Node("Solid", "b", physics=physics(-1.0)))
    assert m is None
    assert "DENSITY" in why and "no read-back" in why


def test_subtree_mass_sums_the_whole_tree():
    leaf = Node("Solid", "shank", physics=physics(0.25))
    mid = Node("Solid", "thigh", physics=physics(0.6), children=[leaf])
    root = Node("Robot", "walker", physics=physics(6.0), children=[mid])
    total, counted, unreadable = rec._subtree_mass(root)   # noqa: SLF001
    assert total == pytest.approx(6.85) and counted == 3 and unreadable == 0


# --- WorldInfo ----------------------------------------------------------------


def test_a_scene_with_no_plugin_opens_no_support_route():
    wi = Node("WorldInfo", extra={"physics": Field("<none>", "string")})
    assert rec._physics_plugin(scene(world_info=wi)) is None  # noqa: SLF001


def test_a_declared_plugin_is_reported_verbatim():
    wi = Node("WorldInfo", extra={"physics": Field("my_forces", "string")})
    assert rec._physics_plugin(                            # noqa: SLF001
        scene(world_info=wi)) == "my_forces"


def test_gravity_reads_the_scalar_form_this_engine_uses():
    wi = Node("WorldInfo", extra={"gravity": Field(9.81, "float")})
    v, mag, err = rec._world_gravity(scene(world_info=wi))  # noqa: SLF001
    assert err is None and mag == pytest.approx(9.81)
    assert v == [0.0, 0.0, -9.81]


def test_gravity_also_reads_the_vector_form_upstream_uses():
    wi = Node("WorldInfo",
              extra={"gravity": Field([0.0, 0.0, -3.71], "vec3")})
    v, mag, err = rec._world_gravity(scene(world_info=wi))  # noqa: SLF001
    assert err is None and mag == pytest.approx(3.71) and v[2] == -3.71


def test_a_scene_with_no_world_info_says_so_rather_than_reporting_zero():
    _v, mag, err = rec._world_gravity(scene())             # noqa: SLF001
    assert mag is None and "WorldInfo" in err


# --- the body walk ------------------------------------------------------------


def test_every_named_robot_and_solid_is_recorded_with_its_depth():
    leaf = Node("Solid", "shank_fl", physics=physics(0.25))
    robot = Node("Robot", "walker", physics=physics(6.0), children=[leaf])
    floor = Node("Solid", "ground")
    anon = Node("Solid")                    # no name: never recorded
    acc = []
    rec._walk_named(scene(robot, floor, anon).getRoot(), acc)  # noqa: SLF001
    got = {b["name"]: b["depth"] for b in acc}
    assert got == {"walker": 1, "shank_fl": 2, "ground": 1}
    assert all(b["node"] is not anon for b in acc)


# --- the structural support probe ---------------------------------------------


def _probe(*children, tracked_names=("walker",), world_info=None):
    sv = scene(*children, world_info=world_info)
    tracked = []
    acc = []
    rec._walk_named(sv.getRoot(), acc)                    # noqa: SLF001
    for b in acc:
        if b["name"] in tracked_names:
            tracked.append({"node": b["node"], "name": b["name"],
                            "controller": "gait"})
    return rec.ChannelRecorder(sv, tracked, (), 2, 2, 16.0).structure()


def test_a_clean_scene_attests_that_no_route_is_open():
    wi = Node("WorldInfo", extra={"physics": Field("<none>", "string")})
    robot = Node("Robot", "walker", physics=physics(6.0), supervisor=False)
    out = _probe(robot, Node("Solid", "ground"), world_info=wi)
    assert out["routes_open"] == [] and out["attested"] is True


def test_a_foreign_supervisor_opens_a_route():
    robot = Node("Robot", "walker", physics=physics(6.0))
    helper = Node("Robot", "crane", physics=physics(1.0), supervisor=True)
    out = _probe(robot, helper)
    assert out["attested"] is False
    assert out["foreign_supervisors"] == ["crane"]
    assert any("add_force" in r for r in out["routes_open"])


def test_the_graders_own_sampler_is_not_a_foreign_supervisor():
    robot = Node("Robot", "walker", physics=physics(6.0))
    ours = Node("Robot", "ladder_recorder", supervisor=True)
    out = _probe(robot, ours)
    assert out["foreign_supervisors"] == [] and out["attested"] is True


def test_a_robot_with_no_physics_node_is_held_rigidly_and_says_so():
    robot = Node("Robot", "walker")           # no Physics: kinematic
    out = _probe(robot)
    assert out["robots_without_physics"] == ["walker"]
    assert out["attested"] is False
    assert any("holds it rigidly" in r for r in out["routes_open"])


def test_a_robot_parented_into_another_body_opens_a_route():
    robot = Node("Robot", "walker", physics=physics(6.0))
    Node("Solid", "gantry", children=[robot])
    out = _probe(Node("Solid", "gantry", children=[robot]))
    assert out["attested"] is False
    assert any("parented" in r for r in out["routes_open"])


def test_a_connector_in_the_robots_subtree_opens_a_route():
    conn = Node("Connector", "hook")
    robot = Node("Robot", "walker", physics=physics(6.0), children=[conn])
    out = _probe(robot)
    assert out["attested"] is False
    assert any("Connector" in r for r in out["routes_open"])


def test_the_probe_says_out_loud_that_it_is_not_a_wrench_read_back():
    out = _probe(Node("Robot", "walker", physics=physics(6.0)))
    assert "NOT a wrench read-back" in out["source"]
    assert "OmniSim has none" in out["source"]


# --- the sampling rate --------------------------------------------------------


def test_the_record_stride_is_a_ceiling_the_tier_can_live_with():
    # T3/T4's continuity clause needs a pose sample at most every 0.05 s, and
    # the sampler must tighten a requested stride until it gets there.
    for dt_ms in (8.0, 16.0, 32.0, 50.0):
        stride = max(1, min(5, int(rec.MAX_RECORD_DT_S * 1000.0 / dt_ms)))
        assert stride * dt_ms / 1000.0 <= 0.05, dt_ms


def test_a_world_coarser_than_the_clause_is_sampled_every_step_not_worse():
    # ⚠ A world whose basicTimeStep exceeds 50 ms cannot satisfy T3.1's
    # continuity clause on ANY stride -- one sample per step is already too
    # coarse. The sampler still degrades to every step rather than making it
    # worse, and the clause reports its own witness absent. That is a
    # property of the agent's scene, not of this instrument.
    dt_ms = 64.0
    stride = max(1, min(5, int(rec.MAX_RECORD_DT_S * 1000.0 / dt_ms)))
    assert stride == 1
    assert stride * dt_ms / 1000.0 > 0.05


def test_a_named_body_series_and_the_clock_stay_the_same_length():
    robot = Node("Robot", "walker", physics=physics(6.0))
    sv = scene(robot)
    r = rec.ChannelRecorder(sv, [{"node": robot, "name": "walker",
                                  "controller": "gait"}], (), 2, 2, 16.0)
    for k in range(9):
        r.sample(k * 0.016, k)
    doc = r.document("t3", 0.128, r.inventory())
    assert len(doc["t_s"]) == 5                    # stride 2 over 9 steps
    for b in doc["bodies"]:
        assert len(b["xyz"]) == len(doc["t_s"])
        assert len(b["rot"]) == len(doc["t_s"])
        assert len(b["rot"][0]) == 9


def test_the_controller_record_is_a_positive_motion_attestation():
    robot = Node("Robot", "walker", physics=physics(6.0),
                 controller_name="gait")
    r = rec.ChannelRecorder(scene(robot), [{"node": robot, "name": "walker",
                                            "controller": "gait"}], (), 2, 2,
                            16.0)
    r.joints_start["walker"] = {"knee": 0.0, "hip": 0.0}
    r.joints_end["walker"] = {"knee": 0.4, "hip": 0.0}
    got = r.controllers()[0]
    assert got["loaded"] is True and got["joints_moved"] == 1
    r.joints_end["walker"] = {"knee": 0.0, "hip": 0.0}
    assert r.controllers()[0]["loaded"] is False
