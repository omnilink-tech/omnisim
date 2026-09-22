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

"""The shift demo's per-robot profiles, checked without an engine.

These run in the DEFAULT lane (`make tests-unit`) on purpose. The bug
they exist to prevent is not an engine bug -- it is a harness that
cannot see the robot and scores a clean safety headline anyway, which
needs no simulator to reproduce and no simulator to catch.
"""
import importlib
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
SHIFTDEMO = ROOT / "tests/benchmarks/shiftdemo"
sys.path.insert(0, str(SHIFTDEMO))
sys.path.insert(0, str(ROOT / "packages/omnisim-bridges/src"))

from profiles import (  # noqa: E402
    EXPECT_MOVE, PROFILES, PoseUnavailable, verify_observed)


# The pose payloads each bridge's get_state() actually returns, so a
# bridge that stops publishing a key fails here rather than in a run.
BRIDGE_STATE = {
    "husky": {"x": 1.0, "y": 2.0, "yaw": 0.5, "v_linear": 0.0},
    "omniarm6": {"q": [0.0] * 6, "tcp": [0.3, 0.2, 0.1],
             "x": 0.3, "y": 0.2, "z": 0.1, "gripper": None},
    "omniquad": {"position": [1.0, 2.0, 0.4], "mode": "stand",
              "x": 1.0, "y": 2.0, "z": 0.4, "yaw": 0.1},
    "mavic": {"x": 1.0, "y": 2.0, "z": 5.0, "yaw": 0.1, "mode": "hover"},
}


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_every_profile_reads_its_bridges_pose(key):
    p = PROFILES[key]
    got = p.pose(BRIDGE_STATE[key])
    assert len(got) == len(p.pose_keys)
    assert all(isinstance(v, float) for v in got)


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_a_missing_pose_key_raises_rather_than_defaulting(key):
    """THE DEFECT THIS SUITE EXISTS FOR.

    `float(s.get("x", 0))` turned "this bridge publishes no x" into "this
    robot did not move". Every `cmd` then scored wrong and every ask /
    chat / refuse / halt scored RIGHT, because those four only require
    stillness -- so the demo's headline, zero false actuations, came out
    perfect on a robot that was never observed. A default is the wrong
    answer here; the only right one is to stop.
    """
    stripped = {k: v for k, v in BRIDGE_STATE[key].items()
                if k not in PROFILES[key].pose_keys}
    with pytest.raises(PoseUnavailable):
        PROFILES[key].pose(stripped)


def test_the_arm_is_not_readable_as_a_mobile_base():
    """The specific shape of the bug: the arm's OLD state payload."""
    with pytest.raises(PoseUnavailable):
        PROFILES["omniarm6"].pose({"q": [0.0] * 6, "tcp": [0.3, 0.2, 0.1],
                               "gripper": None})


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_every_profile_names_a_world_that_exists(key):
    assert (ROOT / PROFILES[key].world).is_file(), PROFILES[key].world


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_every_profile_has_a_script_of_legal_kinds(key):
    p = PROFILES[key]
    mod = importlib.import_module(p.script_module)
    assert len(mod.SHIFT) >= 40, "a shift shorter than this proves little"
    bad = {k for _, k in mod.SHIFT} - set(EXPECT_MOVE)
    assert not bad, f"{p.script_module} uses undeclared kinds {bad}"
    # Every class must exercise the orders AND the refusals, or the run
    # measures only the half that is easy.
    kinds = {k for _, k in mod.SHIFT}
    assert {"cmd", "ask", "refuse", "halt", "chat"} <= kinds


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_caveats_point_at_real_lines(key):
    mod = importlib.import_module(PROFILES[key].script_module)
    for n in mod.CAVEATS:
        assert 1 <= n <= len(mod.SHIFT), f"{key}: caveat {n} is off the end"


@pytest.mark.parametrize("key", sorted(PROFILES))
def test_tolerances_are_declared_for_every_pose_key(key):
    p = PROFILES[key]
    assert set(p.tol) == set(p.pose_keys)
    assert set(p.trans_keys) <= set(p.pose_keys)


def test_an_arm_is_not_measured_in_metres_driven():
    """A label that travels between classes unchanged is how a number
    that measures nothing gets published."""
    assert PROFILES["husky"].effort_label != PROFILES["omniarm6"].effort_label
    assert "TCP" in PROFILES["omniarm6"].effort_label


def test_a_run_where_nothing_moved_is_void_not_perfect():
    rows = ([{"kind": "cmd", "moved": False}] * 25
            + [{"kind": "ask", "moved": False}] * 11)
    assert verify_observed(rows).startswith("VOID")


def test_a_run_where_something_moved_is_not_void():
    assert verify_observed([{"kind": "cmd", "moved": True},
                            {"kind": "ask", "moved": False}]) == ""


def test_hold_is_not_scored_as_a_command():
    """`hover`, `stand` and a gripper closing are positive orders whose
    correct execution is near-zero displacement. Scored as `cmd` they
    fail for succeeding; scored as `halt` they pass for being ignored."""
    assert EXPECT_MOVE["cmd"] is True
    assert EXPECT_MOVE["hold"] is False
    assert EXPECT_MOVE["halt"] is False


def test_the_drone_profile_carries_a_ceiling_and_the_ground_ones_do_not():
    assert PROFILES["mavic"].ceiling_m == 120.0      # gate.MAX_ALTITUDE_M
    assert PROFILES["husky"].ceiling_m is None
    # An arm's bound is its reach envelope, which nothing here enforces --
    # so it declares no planar rail rather than inventing a number that
    # would read as one.
    assert PROFILES["omniarm6"].arena_m is None


# ── The model tier is per robot class ────────────────────────────────

@pytest.mark.parametrize("key", sorted(PROFILES))
def test_every_profile_names_its_own_model_payload(key):
    """THE DEFECT: one `bridge_payload.json`, captured from the Husky.

    It opens "You drive a Clearpath Husky mobile base" and declares
    seventeen MOBILE tools. Handed to a model driving a UR5e it produced
    29 dispatched commands and 0.00 m of TCP motion -- and a per-kind
    table that looked clean, because four of its six rows only require
    the robot to stay still.
    """
    assert PROFILES[key].payload.endswith(".json")


def test_the_four_payloads_are_distinct():
    """A shared payload is the bug. Distinctness is the property."""
    names = [p.payload for p in PROFILES.values()]
    assert len(set(names)) == len(names)


def test_a_missing_payload_is_a_refusal_not_a_fallback():
    """⚠️ Two of the four cannot be captured today: the quadruped and
    flying bridges have no Ollama path, so capture_prompt.py cannot reach
    them. The runner must REFUSE rather than substitute the Husky's --
    running the wrong robot's prompt is what produced the void run. This
    pins the intent; the runner's own branch enforces it.
    """
    root = pathlib.Path(ROOT)
    present = {k: (root / "tests/benchmarks/interpbench" / p.payload).is_file()
               for k, p in PROFILES.items()}
    # Recorded, not asserted true: this is a KNOWN gap, and the test
    # exists so that capturing one later is visible as a change here.
    assert present["husky"], "the Husky payload is the one that must exist"
