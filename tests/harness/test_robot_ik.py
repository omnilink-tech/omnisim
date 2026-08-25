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

"""Unit tests for POST /robot/<def>/ik (`solve_ik`, internal parity plan, item W2.1).

The supervisor verb is exercised through `dispatch()` against stub nodes (the
same import trick as test_joint_set.py). The engine semantics the stubs model:

  - Node.solveIk is a PURE PREVIEW: the engine runs World.solve_ik against the
    live Newton model and NOTHING moves — so the verb must never step the sim
    and never call setJointPosition.
  - The engine answers joint NODE IDS (unique ids), not names; the verb owns
    the id→name mapping via the same joint_write_index walk the write path
    uses, so the names an agent gets back are, by construction, the names
    POST /robot/<def>/joints/set can apply.
  - residuals are FK-measured metres on exactly the returned angles — the verb
    must pass them through untouched (an unreachable target reports its large
    residual; it is never hidden).

The live-engine closed loop (solve → apply → measure the end effector's real
world position) is verified manually per scripts/harness/README.md; these
tests lock down the contract with no simulator in the loop.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SUPERVISOR_DIR = (REPO_ROOT / "projects" / "default" / "controllers"
                  / "harness_supervisor")
HARNESS_DIR = REPO_ROOT / "scripts" / "harness"
sys.path.insert(0, str(HARNESS_DIR))


def _supervisor_module(monkeypatch):
    """Import the controller module under stock Python with one API stub."""
    monkeypatch.syspath_prepend(str(SUPERVISOR_DIR))
    api = types.ModuleType("omnisim")
    api.Supervisor = object
    monkeypatch.setitem(sys.modules, "omnisim", api)
    spec = importlib.util.spec_from_file_location(
        "robot_ik_supervisor_under_test",
        SUPERVISOR_DIR / "harness_supervisor.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# Stub scene (same field-access shape as test_joint_set.py, plus node ids and
# an effector carrying solveIk)
# ---------------------------------------------------------------------------


class _SFFloat:
    def __init__(self, owner, key):
        self._owner, self._key = owner, key

    def getSFFloat(self):
        return self._owner.values[self._key]


class _SFString:
    def __init__(self, value):
        self._value = value

    def getSFString(self):
        return self._value


class _SFNode:
    def __init__(self, node):
        self._node = node

    def getSFNode(self):
        return self._node


class _MFNode:
    def __init__(self, nodes):
        self._nodes = list(nodes)

    def getCount(self):
        return len(self._nodes)

    def getMFNode(self, i):
        return self._nodes[i]


class _Params:
    def __init__(self, position=0.0, min_stop=0.0, max_stop=0.0):
        self.values = {"position": position, "minStop": min_stop,
                       "maxStop": max_stop}

    def getField(self, name):
        if name in self.values:
            return _SFFloat(self, name)
        return None


class _Motor:
    def __init__(self, name, min_position=0.0, max_position=0.0,
                 type_name="RotationalMotor"):
        self._name = name
        self._type = type_name
        self.values = {"minPosition": min_position, "maxPosition": max_position}

    def getTypeName(self):
        return self._type

    def getField(self, name):
        if name == "name":
            return _SFString(self._name)
        if name in self.values:
            return _SFFloat(self, name)
        return None


class _Joint:
    def __init__(self, node_id, motor=None, params=None,
                 type_name="HingeJoint"):
        self.id = node_id           # what entry["node"].id reads
        self._type = type_name
        self.motor = motor
        self.params = params
        self.set_calls: list[tuple[float, int]] = []

    def getTypeName(self):
        return self._type

    def getField(self, name):
        if name == "jointParameters":
            return _SFNode(self.params)
        if name == "device":
            return _MFNode([self.motor] if self.motor is not None else [])
        return None

    def setJointPosition(self, position, index=1):
        self.set_calls.append((position, index))


class _Effector:
    """End-effector Solid stub carrying the new Node.solveIk."""

    def __init__(self, result):
        self.result = result
        self.calls: list[tuple] = []

    def getTypeName(self):
        return "Solid"

    def solveIk(self, targets, rotations=None, toolOffset=None, iterations=64):
        self.calls.append((targets, rotations, toolOffset, iterations))
        return self.result


class _Robot:
    def __init__(self, joints):
        self._joints = joints

    def getTypeName(self):
        return "Robot"

    def getField(self, name):
        if name == "children":
            return _MFNode(self._joints)
        return None


class _Supervisor:
    def __init__(self, nodes):
        self.nodes = nodes
        self.step_count = 0

    def getFromDef(self, name):
        return self.nodes.get(name)

    def step(self, _basic_step):
        self.step_count += 1
        return 0


def _arm_with_effector(ik_result):
    shoulder = _Joint(101, motor=_Motor("shoulder", -3.14, 3.14),
                      params=_Params(position=0.1, min_stop=-3.14,
                                     max_stop=3.14))
    elbow = _Joint(102, motor=_Motor("elbow", -2.0, 2.0),
                   params=_Params(position=-0.2, min_stop=-2.0, max_stop=2.0))
    robot = _Robot([shoulder, elbow])
    effector = _Effector(ik_result)
    supervisor = _Supervisor({"ARM": robot, "TIP": effector})
    return supervisor, shoulder, elbow, effector


# ---------------------------------------------------------------------------
# Response shape: names, residuals, pure-preview
# ---------------------------------------------------------------------------


def test_solve_ik_maps_slot_angles_to_named_joints(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor, shoulder, elbow, effector = _arm_with_effector({
        "status": 0,
        "joint_node_ids": [101, 102],
        "angles": [[0.5, -1.0], [0.7, -0.9]],
        "residuals": [0.0004, 0.2103],
    })
    result = module.dispatch(supervisor, 16, 0, "solve_ik", {
        "def": "ARM", "effector": "TIP",
        "targets": [[0.4, 0.0, 0.3], [0.9, 0.0, 0.1]],
    })
    assert result["robot"] == "ARM"
    assert result["effector"] == "TIP"
    assert result["solved_joints"] == [
        {"name": "shoulder", "node_id": 101, "appliable": True},
        {"name": "elbow", "node_id": 102, "appliable": True},
    ]
    r0, r1 = result["results"]
    assert r0["target"] == [0.4, 0.0, 0.3]
    assert r0["joints"] == {"shoulder": 0.5, "elbow": -1.0}
    assert r0["residual_m"] == pytest.approx(0.0004)
    # The unreachable target's residual is passed through, never hidden.
    assert r1["residual_m"] == pytest.approx(0.2103)
    assert r1["joints"] == {"shoulder": 0.7, "elbow": -0.9}
    # PURE PREVIEW: no stepping, no joint writes.
    assert supervisor.step_count == 0
    assert shoulder.set_calls == [] and elbow.set_calls == []
    # And it says so where the agent will read it.
    v = result["verification"]
    assert "PURE PREVIEW" in v["semantics"]
    assert "residual" in v["semantics"]
    assert "warmup" in v and "kernel" in v["warmup"]
    assert isinstance(result["solve_ms"], float)
    # The engine saw the arguments it was given.
    assert effector.calls == [([[0.4, 0.0, 0.3], [0.9, 0.0, 0.1]],
                               None, None, 64)]


def test_optional_arguments_are_forwarded(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, effector = _arm_with_effector({
        "status": 0, "joint_node_ids": [101, 102],
        "angles": [[0.0, 0.0]], "residuals": [0.0]})
    module.dispatch(supervisor, 16, 0, "solve_ik", {
        "def": "ARM", "effector": "TIP", "targets": [[0.1, 0.2, 0.3]],
        "rotations": [[0.0, 0.0, 0.0, 1.0]],
        "tool_offset": [0.0, 0.0, 0.05],
        "iterations": 128,
    })
    targets, rotations, tool, iterations = effector.calls[0]
    assert rotations == [[0.0, 0.0, 0.0, 1.0]]
    assert tool == [0.0, 0.0, 0.05]
    assert iterations == 128


def test_foreign_joint_ids_are_reported_not_hidden(monkeypatch):
    # An effector on ANOTHER robot solves joints this robot walk cannot name:
    # the angles come back keyed node_<id> and the mismatch is disclosed.
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, _ = _arm_with_effector({
        "status": 0, "joint_node_ids": [101, 999],
        "angles": [[0.5, 0.1]], "residuals": [0.001]})
    result = module.dispatch(supervisor, 16, 0, "solve_ik", {
        "def": "ARM", "effector": "TIP", "targets": [[0.4, 0.0, 0.3]]})
    assert result["solved_joints"][1] == {
        "name": None, "node_id": 999, "appliable": False}
    assert result["results"][0]["joints"]["node_999"] == 0.1
    assert result["verification"]["unmapped_node_ids"] == [999]
    assert "effector" in result["verification"]["unmapped_note"]


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_unknown_robot_def_matches_the_404_shape(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor = _Supervisor({})
    with pytest.raises(module.CommandError, match="no node with DEF"):
        module.dispatch(supervisor, 16, 0, "solve_ik", {
            "def": "NOPE", "effector": "TIP", "targets": [[0, 0, 0]]})


def test_unknown_effector_def_matches_the_404_shape(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, _ = _arm_with_effector({"status": 0})
    with pytest.raises(module.CommandError,
                       match="no node with DEF 'GONE'"):
        module.dispatch(supervisor, 16, 0, "solve_ik", {
            "def": "ARM", "effector": "GONE", "targets": [[0, 0, 0]]})


def test_bad_targets_are_refused_before_the_engine_is_touched(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, effector = _arm_with_effector({"status": 0})
    for bad in (None, [], "x", [[1, 2]], [[1, 2, "z"]],
                [[float("nan"), 0, 0]]):
        with pytest.raises(module.CommandError):
            module.dispatch(supervisor, 16, 0, "solve_ik", {
                "def": "ARM", "effector": "TIP", "targets": bad})
    assert effector.calls == []


def test_mismatched_rotations_are_refused(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, effector = _arm_with_effector({"status": 0})
    with pytest.raises(module.CommandError, match="pair 1:1"):
        module.dispatch(supervisor, 16, 0, "solve_ik", {
            "def": "ARM", "effector": "TIP",
            "targets": [[0, 0, 0], [1, 1, 1]],
            "rotations": [[0, 0, 0, 1]]})
    assert effector.calls == []


@pytest.mark.parametrize("status,needle", [
    (-1, "IK unavailable"),
    (-2, "no Newton physics body"),
    (-3, "no IK-solvable joints"),
    (-4, "IK solver failed"),
    (-9, "did not answer"),
])
def test_engine_statuses_become_named_refusals(monkeypatch, status, needle):
    module = _supervisor_module(monkeypatch)
    supervisor, _, _, _ = _arm_with_effector({"status": status})
    with pytest.raises(module.CommandError, match=needle):
        module.dispatch(supervisor, 16, 0, "solve_ik", {
            "def": "ARM", "effector": "TIP", "targets": [[0, 0, 0]]})


# ---------------------------------------------------------------------------
# Harness-side declarations
# ---------------------------------------------------------------------------


def test_route_is_declared_in_routes_table():
    import omnisim_harness as h
    route = next((r for r in h.ROUTES
                  if (r["method"], r["path"]) == ("POST", "/robot/<def>/ik")),
                 None)
    assert route is not None
    # The three hard-won semantics must be disclosed where an agent reads
    # them: pure preview, FK-measured residual, and the cold kernel compile.
    assert "PURE" in route["summary"]
    assert "residual" in route["summary"]
    assert "kernel" in route["summary"]
    assert "effector" in route["body"] and "targets" in route["body"]


def test_ik_preview_is_transparently_retryable():
    import omnisim_harness as h
    assert "solve_ik" in h.IDEMPOTENT_SUPERVISOR_COMMANDS
    assert h.is_retryable_supervisor_call("solve_ik") is True


def test_ik_error_codes_are_branchable():
    import omnisim_harness as h
    assert h.classify_supervisor_error(
        "no IK-solvable joints on robot DEF 'ARM': ...") == (422, "IK_NO_JOINTS")
    assert h.classify_supervisor_error(
        "IK unavailable: end effector 'TIP' has no Newton physics body"
    ) == (422, "IK_NO_BODY")
    assert h.classify_supervisor_error(
        "IK unavailable: no Newton physics backend, or the world is not "
        "finalised yet") == (503, "IK_UNAVAILABLE")
    assert h.classify_supervisor_error(
        "IK solver failed inside the engine — see the log"
    ) == (500, "IK_SOLVER_FAILED")
    codes = h.known_request_error_codes()
    for code in ("IK_NO_JOINTS", "IK_NO_BODY", "IK_UNAVAILABLE",
                 "IK_SOLVER_FAILED"):
        assert code in codes


def test_supervisor_publishes_the_verb(monkeypatch):
    module = _supervisor_module(monkeypatch)
    assert "solve_ik" in module.dispatch_commands()
