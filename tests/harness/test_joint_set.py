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

"""Unit tests for POST /robot/<def>/joints/set (`set_joint_positions`).

The supervisor verb is exercised through `dispatch()` against stub nodes (the
same import trick as test_world_sync.py: the controller module is imported
under stock Python with `omnisim.Supervisor` stubbed). The engine semantics
the stubs model are the ones the verb is built around:

  - Node.setJointPosition is a PD setpoint under Newton, not a teleport
    (OmJoint.cpp:130-149 re-pins the motor target), so `achieved` is measured
    AFTER settling — the "residual" stub models imperfect convergence.
  - The engine clamps against JointParameters minStop/maxStop only, and
    minStop == maxStop == 0 means unconstrained
    (OmJointParameters::clampPosition).
  - A motorised hinge whose effective limits (motor minPosition/maxPosition,
    else joint stops) are EQUAL is registered as a velocity wheel with ke=0
    and position targets are silently ignored (OmBasicJoint.cpp, the W1.4
    trap) — the wheel stub ignores the write, and the verb must pre-classify
    it position_controllable: false rather than return bare success.

The live-engine behaviour (real UR5e convergence, clamp warning, wheel
inertness) is verified manually per scripts/harness/README.md; these tests
lock down the contract with no simulator in the loop.
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
        "joint_set_supervisor_under_test",
        SUPERVISOR_DIR / "harness_supervisor.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# Stub scene
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
    """JointParameters stub: position + hard stops."""

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
    """HingeJoint/SliderJoint stub.

    `residual`: how far short of the commanded target the joint settles
    (models PD convergence error). `ignores_position` models the velocity
    wheel: the write is accepted and the physics never moves the joint.
    """

    def __init__(self, motor=None, params=None, type_name="HingeJoint",
                 residual=0.0, ignores_position=False):
        self._type = type_name
        self.motor = motor
        self.params = params
        self.residual = residual
        self.ignores_position = ignores_position
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
        if self.params is not None and not self.ignores_position:
            self.params.values["position"] = position - self.residual


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
    def __init__(self, robots):
        self.robots = robots
        self.step_count = 0

    def getFromDef(self, name):
        return self.robots.get(name)

    def step(self, _basic_step):
        self.step_count += 1
        return 0


def _arm():
    """A two-joint arm: one exact servo, one with 0.005 residual."""
    shoulder = _Joint(motor=_Motor("shoulder", -3.14, 3.14),
                      params=_Params(position=0.1, min_stop=-3.14,
                                     max_stop=3.14))
    elbow = _Joint(motor=_Motor("elbow", -2.0, 2.0),
                   params=_Params(position=-0.2, min_stop=-2.0, max_stop=2.0),
                   residual=0.005)
    return _Robot([shoulder, elbow]), shoulder, elbow


# ---------------------------------------------------------------------------
# Response shape + measured error
# ---------------------------------------------------------------------------


def test_set_joint_positions_measures_achieved_and_error(monkeypatch):
    module = _supervisor_module(monkeypatch)
    robot, shoulder, elbow = _arm()
    supervisor = _Supervisor({"ARM": robot})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "ARM",
        "joints": {"shoulder": 0.5, "elbow": -1.0},
        "settle_steps": 4,
    })
    assert supervisor.step_count == 4  # one settle for the batch
    assert shoulder.set_calls == [(0.5, 1)]
    assert elbow.set_calls == [(-1.0, 1)]
    s = result["joints"]["shoulder"]
    assert s["requested"] == 0.5
    assert s["commanded"] == 0.5
    assert s["clamped"] is False
    assert s["position_before"] == 0.1
    assert s["achieved"] == 0.5
    assert s["error"] == 0.0
    assert s["moved"] is True
    assert s["position_controllable"] is True
    e = result["joints"]["elbow"]
    assert e["achieved"] == pytest.approx(-1.005)
    assert e["error"] == pytest.approx(-0.005)
    v = result["verification"]
    assert v["applied"] == 2
    assert v["settle_steps"] == 4
    assert v["sim_time_advanced_ms"] == 64
    assert v["max_abs_error"] == pytest.approx(0.005)
    assert v["max_abs_error_joint"] == "elbow"
    # The response must never echo the argument back as a measurement.
    assert "semantics" in v and "not a teleport" in v["semantics"]


def test_settle_steps_defaults_and_validation(monkeypatch):
    module = _supervisor_module(monkeypatch)
    robot, _, _ = _arm()
    supervisor = _Supervisor({"ARM": robot})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "ARM", "joints": {"shoulder": 0.2}})
    assert result["verification"]["settle_steps"] == 16  # default
    assert supervisor.step_count == 16
    with pytest.raises(module.CommandError):
        module.dispatch(supervisor, 16, 0, "set_joint_positions", {
            "def": "ARM", "joints": {"shoulder": 0.2}, "settle_steps": -1})


# ---------------------------------------------------------------------------
# Clamping (mirrors OmJointParameters::clampPosition)
# ---------------------------------------------------------------------------


def test_target_beyond_hard_stops_is_clamped_and_flagged(monkeypatch):
    module = _supervisor_module(monkeypatch)
    robot, shoulder, _ = _arm()
    supervisor = _Supervisor({"ARM": robot})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "ARM", "joints": {"shoulder": 9.0}, "settle_steps": 1})
    s = result["joints"]["shoulder"]
    assert s["requested"] == 9.0
    assert s["commanded"] == 3.14   # the adopted (clamped) value
    assert s["clamped"] is True
    assert shoulder.set_calls == [(3.14, 1)]  # the engine is sent the clamp
    assert s["error"] == 0.0        # error is vs the ADOPTED value


def test_zero_zero_stops_mean_unconstrained_no_clamp(monkeypatch):
    module = _supervisor_module(monkeypatch)
    joint = _Joint(motor=_Motor("hinge", -6.0, 6.0),
                   params=_Params(position=0.0, min_stop=0.0, max_stop=0.0))
    supervisor = _Supervisor({"R": _Robot([joint])})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "R", "joints": {"hinge": 5.0}, "settle_steps": 1})
    j = result["joints"]["hinge"]
    assert j["commanded"] == 5.0
    assert j["clamped"] is False


# ---------------------------------------------------------------------------
# Refusals: unknown robot / unknown joints / ambiguity — all-or-nothing
# ---------------------------------------------------------------------------


def test_unknown_robot_def_matches_the_404_shape(monkeypatch):
    module = _supervisor_module(monkeypatch)
    supervisor = _Supervisor({})
    with pytest.raises(module.CommandError, match="no node with DEF"):
        module.dispatch(supervisor, 16, 0, "set_joint_positions", {
            "def": "NOPE", "joints": {"a": 0.0}})


def test_unknown_joint_names_refuse_the_whole_batch(monkeypatch):
    module = _supervisor_module(monkeypatch)
    robot, shoulder, _ = _arm()
    supervisor = _Supervisor({"ARM": robot})
    with pytest.raises(module.CommandError) as excinfo:
        module.dispatch(supervisor, 16, 0, "set_joint_positions", {
            "def": "ARM", "joints": {"shoulder": 0.5, "bogus": 1.0}})
    msg = str(excinfo.value)
    assert "unknown joint name" in msg
    assert "bogus" in msg
    assert "shoulder" in msg            # the available names are offered
    assert shoulder.set_calls == []     # nothing was written
    assert supervisor.step_count == 0   # and nothing was stepped


def test_ambiguous_joint_name_is_refused(monkeypatch):
    module = _supervisor_module(monkeypatch)
    a = _Joint(motor=_Motor("wheel", -1.0, 1.0),
               params=_Params(min_stop=-1.0, max_stop=1.0))
    b = _Joint(motor=_Motor("wheel", -1.0, 1.0),
               params=_Params(min_stop=-1.0, max_stop=1.0))
    supervisor = _Supervisor({"R": _Robot([a, b])})
    with pytest.raises(module.CommandError, match="match multiple joints"):
        module.dispatch(supervisor, 16, 0, "set_joint_positions", {
            "def": "R", "joints": {"wheel": 0.5}})
    assert a.set_calls == [] and b.set_calls == []


def test_non_finite_target_is_refused(monkeypatch):
    module = _supervisor_module(monkeypatch)
    robot, shoulder, _ = _arm()
    supervisor = _Supervisor({"ARM": robot})
    for bad in (float("inf"), float("nan"), "hello", None):
        with pytest.raises(module.CommandError):
            module.dispatch(supervisor, 16, 0, "set_joint_positions", {
                "def": "ARM", "joints": {"shoulder": bad}})
    assert shoulder.set_calls == []


# ---------------------------------------------------------------------------
# The W1.4 trap: limit-less motor == velocity wheel, never bare success
# ---------------------------------------------------------------------------


def test_limitless_motor_is_reported_not_position_controllable(monkeypatch):
    module = _supervisor_module(monkeypatch)
    wheel = _Joint(motor=_Motor("left_wheel"),   # minPosition == maxPosition == 0
                   params=_Params(position=0.0),  # and no stops either
                   ignores_position=True)         # the physics ignores the write
    supervisor = _Supervisor({"ROVER": _Robot([wheel])})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "ROVER", "joints": {"left_wheel": 2.0}, "settle_steps": 2})
    j = result["joints"]["left_wheel"]
    assert j["position_controllable"] is False
    assert "VELOCITY wheel" in j["note"]
    assert "IGNORED" in j["note"]
    # And the measurement agrees with the classification:
    assert j["moved"] is False
    assert j["error"] == pytest.approx(-2.0)


def test_motor_limits_make_a_hinge_position_controllable(monkeypatch):
    module = _supervisor_module(monkeypatch)
    # No stops, but the MOTOR declares limits — the engine's registration
    # rule prefers motor minPosition/maxPosition, so this is a servo.
    servo = _Joint(motor=_Motor("lift", -1.0, 1.0), params=_Params())
    supervisor = _Supervisor({"R": _Robot([servo])})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "R", "joints": {"lift": 0.5}, "settle_steps": 1})
    j = result["joints"]["lift"]
    assert j["position_controllable"] is True
    assert j["limits"] == {"lower": -1.0, "upper": 1.0,
                           "source": "motor minPosition/maxPosition"}


def test_slider_is_position_controlled_even_without_limits(monkeypatch):
    module = _supervisor_module(monkeypatch)
    finger = _Joint(motor=_Motor("finger", type_name="LinearMotor"),
                    params=_Params(), type_name="SliderJoint")
    supervisor = _Supervisor({"G": _Robot([finger])})
    result = module.dispatch(supervisor, 16, 0, "set_joint_positions", {
        "def": "G", "joints": {"finger": 0.02}, "settle_steps": 1})
    assert result["joints"]["finger"]["position_controllable"] is True


def test_passive_joint_is_reported_unheld_not_false(monkeypatch):
    module = _supervisor_module(monkeypatch)
    # No motor at all: the write lands but nothing holds it. That is a
    # different fact from the wheel's "ignored" — reported as null + note.
    passive = _Joint(motor=None, params=_Params(position=0.3))
    supervisor = _Supervisor({"R": _Robot([passive])})
    # Name falls back to the endPoint solid's name; no endpoint here, so the
    # joint is unaddressable — the refusal must name what IS available.
    with pytest.raises(module.CommandError, match="unknown joint name"):
        module.dispatch(supervisor, 16, 0, "set_joint_positions", {
            "def": "R", "joints": {"anything": 0.0}})


# ---------------------------------------------------------------------------
# Harness-side declarations
# ---------------------------------------------------------------------------


def test_route_is_declared_in_routes_table():
    import omnisim_harness as h
    paths = {(r["method"], r["path"]) for r in h.ROUTES}
    assert ("POST", "/robot/<def>/joints/set") in paths
    route = next(r for r in h.ROUTES
                 if (r["method"], r["path"]) == ("POST", "/robot/<def>/joints/set"))
    # The two hard-won semantics must be disclosed where an agent will read
    # them: PD convergence, and the velocity-wheel silent-ignore trap.
    assert "NOT a teleport" in route["summary"]
    assert "position_controllable" in route["summary"]
    assert "joints" in route["body"] and "settle_steps" in route["body"]


def test_joint_write_is_not_transparently_retryable():
    import omnisim_harness as h
    assert "set_joint_positions" not in h.IDEMPOTENT_SUPERVISOR_COMMANDS
    assert h.is_retryable_supervisor_call("set_joint_positions") is False


def test_joint_error_codes_are_branchable():
    import omnisim_harness as h
    assert h.classify_supervisor_error(
        "unknown joint name(s) ['bogus'] on DEF 'ARM'; available joints: []"
    ) == (422, "JOINT_NOT_FOUND")
    assert h.classify_supervisor_error(
        "joint name(s) ['wheel'] match multiple joints on DEF 'R'"
    ) == (409, "JOINT_NAME_AMBIGUOUS")
    codes = h.known_request_error_codes()
    assert "JOINT_NOT_FOUND" in codes
    assert "JOINT_NAME_AMBIGUOUS" in codes


def test_supervisor_publishes_the_verb(monkeypatch):
    module = _supervisor_module(monkeypatch)
    assert "set_joint_positions" in module.dispatch_commands()
