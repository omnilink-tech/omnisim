#!/usr/bin/env python3
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

"""The omnilink_mobile_bridge in-place-rotation REFUSAL contract.

Why this file exists. `MobileBridge._turn_feasibility` refuses a rotation the
base provably cannot finish inside `TURN_MAX_SPIN_S`, and publishes the same
verdict ahead of time as `capabilities.can_rotate_in_place`. Before commit
69b4b024b a wheeled base delivered ~0.7-13% of a commanded pivot, so the
ROSbot XL tripped that refusal for real. 69b4b024b lifted every measured yaw
ceiling by 7-76x, and since 3a3aca456 re-measured the `yaw_rate_gain`
constants NO SHIPPED BASE TRIPS IT ANY MORE -- the slowest, the Jackal, holds
1.552 rad/s and needs 2.0 s for a half turn against a 45 s ceiling.

That makes the mechanism correct and DORMANT, which is the state in which code
rots without anyone noticing. Keeping it is right: it is the honest answer for
a future slow base (or a solver regression that re-starves the servo), and it
is the difference between "this base cannot do that" and a command that runs
for minutes and then reports a timeout. So the SHAPE is pinned here instead,
against a config slow enough to trip the threshold -- deliberately NOT against
any shipped base, so re-measuring a real base can never silently delete this
coverage.

Engine-free: the bridge is imported with a stub Supervisor, so the whole
derivation chain (config -> kinematic ceiling -> measured ceiling ->
capabilities -> refusal) runs without the simulator.

Run from the repo root:

    python -m pytest tests/test_mobile_turn_refusal_contract.py -q
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
CONTROLLER_DIR = (REPO / "projects" / "samples" / "demos" / "controllers"
                  / "omnilink_mobile_bridge")

_BRIDGE_MODULE = None


def _bridge_module():
    """Import omnilink_mobile_bridge under a private name, engine-free.

    The controller dir must be on sys.path for its own sibling imports
    (`_mobile_configs`, `_omnilink_relay`) and lib/controller/python for
    `omnisim`; both are removed again once the module body has run, so no
    other test module inherits a shadowed import path.
    """
    global _BRIDGE_MODULE
    if _BRIDGE_MODULE is not None:
        return _BRIDGE_MODULE
    added = [str(REPO / "lib" / "controller" / "python"), str(CONTROLLER_DIR)]
    for path in added:
        sys.path.insert(0, path)
    try:
        spec = importlib.util.spec_from_file_location(
            "_test_omnilink_mobile_bridge",
            CONTROLLER_DIR / "omnilink_mobile_bridge.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except Exception as exc:  # pragma: no cover - environment problem
        pytest.skip(f"omnilink_mobile_bridge is not importable here: {exc!r}")
    finally:
        for path in added:
            try:
                sys.path.remove(path)
            except ValueError:
                pass
    _BRIDGE_MODULE = module
    return module


class _StubNode:
    """The handful of Supervisor-node reads MobileBridge.__init__ makes."""

    def getPosition(self):
        return [0.0, 0.0, 0.0]

    def getOrientation(self):
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def getField(self, _name):
        return None

    def getProtoField(self, _name):
        return None

    def getDef(self):
        return "STUB"


class _StubSupervisor:
    """A robot that exists only so the bridge can be constructed.

    No devices, no clock, no scene. Every command path this test touches is
    pure arithmetic over the config, which is the point: the refusal contract
    must not depend on a running simulator to be checked.
    """

    def getBasicTimeStep(self):
        return 16.0

    def getDevice(self, _name):
        return None

    def getSelf(self):
        return _StubNode()

    def getFromDef(self, _name):
        return None

    def getTime(self):
        return 0.0

    def step(self, _ms):
        return 0


def _bridge_from(cfg, robot_id="stub"):
    """A MobileBridge over exactly this config."""
    module = _bridge_module()
    return module, module.MobileBridge(_StubSupervisor(), dict(cfg), robot_id)


def _bridge(**overrides):
    """A MobileBridge over a Husky-shaped config with fields overridden."""
    module = _bridge_module()
    cfg = dict(module.MOBILE_CONFIGS["husky"])
    cfg.update(overrides)
    return _bridge_from(cfg, "husky")


def _slow_bridge():
    """A base far too slow to pivot: 0.0023 rad/s, ~679 s for a quarter turn.

    Built by starving the WHEEL SPEED and the measured yaw gain together --
    the two independent ways a base ends up unable to pivot (a genuinely slow
    platform, and a solver that will not deliver the torque a scrub pivot
    needs, which is exactly what 69b4b024b fixed).
    """
    return _bridge(model="Stub Slow Base",
                   max_wheel_speed_radps=0.2,
                   yaw_rate_gain=0.02)


# --------------------------------------------------------------------------- #
# the refusal                                                                  #
# --------------------------------------------------------------------------- #

def test_slow_base_refuses_an_in_place_rotation():
    module, bridge = _slow_bridge()
    refusal = bridge._turn_feasibility(math.pi / 2)

    assert refusal is not None, "a base needing ~679 s for a quarter turn must refuse"
    assert refusal["accepted"] is False
    assert refusal["refused"] == "cannot_rotate_in_place"
    # An unattempted turn reports NOTHING as achieved. A refusal that echoed
    # the command back as `achieved` would read to a caller as a completed
    # turn, which is the failure mode the measured-not-echoed rule exists for.
    assert refusal["achieved"] is None
    assert refusal["error"] is None
    assert refusal["settled"] is False
    assert refusal["timed_out"] is False
    assert refusal["commanded"] == pytest.approx(math.pi / 2)
    assert refusal["unit"] == "rad"


def test_refusal_carries_the_measured_ceiling_and_the_time_it_would_need():
    module, bridge = _slow_bridge()
    want = math.pi
    refusal = bridge._turn_feasibility(want)

    ceiling = refusal["max_angular_rad_s"]
    assert ceiling == pytest.approx(bridge.v_max_angular)
    assert refusal["max_angular_rad_s_kinematic"] == pytest.approx(
        bridge.v_max_angular_kinematic)
    assert ceiling < refusal["max_angular_rad_s_kinematic"], (
        "the measured ceiling must be published BELOW the geometric one, or a "
        "caller plans turns the base cannot make")
    assert refusal["yaw_rate_gain"] == pytest.approx(bridge.yaw_gain)

    # estimated_spin_s is the reason, not decoration: it is the quantity
    # compared against the ceiling, and it must be reproducible from the two
    # numbers alongside it.
    assert refusal["estimated_spin_s"] == pytest.approx(want / ceiling)
    assert refusal["estimated_spin_s"] > module.MobileBridge.TURN_MAX_SPIN_S


def test_refusal_offers_alternatives_a_caller_can_act_on():
    _module, bridge = _slow_bridge()
    alternatives = bridge._turn_feasibility(math.pi / 2)["alternatives"]

    assert isinstance(alternatives, list) and alternatives
    assert all(isinstance(a, str) and a.strip() for a in alternatives)
    # At least one alternative must name a verb this bridge actually publishes
    # -- a refusal that suggests an action the base does not expose costs the
    # caller a turn to discover that.
    published = set(bridge.capabilities["actions"])
    assert any(any(verb in a for verb in published) for a in alternatives), (
        f"no alternative names a published action: {alternatives}")


def test_refusal_message_names_the_base_and_both_rates():
    _module, bridge = _slow_bridge()
    message = bridge._turn_feasibility(math.pi / 2)["message"]

    assert "Stub Slow Base" in message
    assert f"{bridge.v_max_angular:.4f}" in message
    assert f"{bridge.v_max_angular_kinematic:.2f}" in message


def test_capabilities_report_can_rotate_in_place_false_before_any_turn():
    """The refusal is also published AHEAD of the call, not only on refusal.

    An agent that has to issue a turn to find out the base cannot turn has
    already spent the turn; `can_rotate_in_place` is how it plans instead.
    """
    _module, bridge = _slow_bridge()
    assert bridge.capabilities["can_rotate_in_place"] is False
    assert bridge.capabilities["max_angular_rad_s"] == pytest.approx(
        bridge.v_max_angular)
    assert bridge.capabilities["yaw_rate_gain"] == pytest.approx(bridge.yaw_gain)


# --------------------------------------------------------------------------- #
# the boundary, and what must NOT be refused                                   #
# --------------------------------------------------------------------------- #

def test_threshold_is_turn_max_spin_s_and_nothing_else():
    module, bridge = _slow_bridge()
    ceiling = bridge.v_max_angular
    limit = module.MobileBridge.TURN_MAX_SPIN_S

    just_inside = ceiling * limit * 0.99
    just_outside = ceiling * limit * 1.01
    assert bridge._turn_feasibility(just_inside) is None
    assert bridge._turn_feasibility(just_outside) is not None
    # Sign must not matter: a base that cannot pivot left cannot pivot right.
    assert bridge._turn_feasibility(-just_outside) is not None


def test_a_rotation_inside_the_settle_tolerance_is_never_refused():
    module, bridge = _slow_bridge()
    assert bridge._turn_feasibility(
        module.MobileBridge.TURN_TOL_RAD * 0.5) is None


def test_a_kinematic_body_is_never_refused():
    """A supervisor-integrated body has no wheels to scrub -- the refusal is
    about a skid-steer pivot it cannot physically perform, and must not fire
    on a base that writes its own pose."""
    _module, bridge = _bridge(model="Stub Kinematic", kinematic=True,
                              max_wheel_speed_radps=0.2, yaw_rate_gain=0.02)
    assert bridge._turn_feasibility(math.pi) is None
    assert bridge.capabilities["can_rotate_in_place"] is True


def test_the_refusal_is_dormant_on_the_slowest_shipped_base():
    """Pins the fact that made this test necessary.

    The Jackal is the slowest pivoting base in MOBILE_CONFIGS since
    69b4b024b/3a3aca456. If any shipped base starts tripping the refusal
    again, the yaw ceilings have regressed and THIS is the test that says so
    -- the rest of the suite would stay green.
    """
    module = _bridge_module()
    for robot_id, cfg in module.MOBILE_CONFIGS.items():
        _m, bridge = _bridge_from(cfg, robot_id)
        assert bridge.capabilities["can_rotate_in_place"] is True, robot_id
        assert bridge._turn_feasibility(math.pi) is None, (
            f"{robot_id} now refuses a half turn: yaw ceiling "
            f"{bridge.v_max_angular:.4f} rad/s")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
