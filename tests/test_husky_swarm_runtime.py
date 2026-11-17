# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Unit tests for the model-independent HuskySwarm execution layer."""

from __future__ import annotations

import ast
import importlib.util
import json
import math
import os
import re
import sys
import types
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "agents" / "production" / "husky_swarm" / "swarm_agent.py"
sys.path.insert(0, str(ROOT / "agents" / "production"))

from _lib.supervision import normalize_outcome  # noqa: E402

# The execution layer uses OmniLinkClient only from main(). Unit-test
# environments should not need the separately distributed omnilink package
# merely to exercise local tools.
try:
    import omnilink.client  # type: ignore  # noqa: F401
except ImportError:
    omnilink_package = types.ModuleType("omnilink")
    omnilink_client = types.ModuleType("omnilink.client")
    omnilink_client.OmniLinkClient = object
    omnilink_package.client = omnilink_client
    sys.modules["omnilink"] = omnilink_package
    sys.modules["omnilink.client"] = omnilink_client

SPEC = importlib.util.spec_from_file_location("husky_swarm_runtime_under_test",
                                               MODULE_PATH)
assert SPEC and SPEC.loader
swarm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(swarm)


class FakeBridges:
    """Tiny deterministic bridge model sufficient to test skill contracts."""

    def __init__(self) -> None:
        self.state = {
            "husky_ne": {"x": 3.0, "y": 3.0, "yaw": 0.0, "mode": "idle"},
            "husky_nw": {"x": -3.0, "y": 3.0, "yaw": 0.0, "mode": "idle"},
            "husky_se": {"x": 1.0, "y": -1.0, "yaw": 0.0, "mode": "idle"},
            "husky_sw": {"x": -3.0, "y": -3.0, "yaw": 0.0, "mode": "idle"},
        }
        self.motion = []

    def post(self, husky, path, payload=None, **_):
        payload = payload or {}
        st = self.state[husky]
        if path == "get_robot_state":
            return dict(st)
        if path == "drive_forward":
            distance = float(payload["distance"])
            st["x"] += math.cos(st["yaw"]) * distance
            st["y"] += math.sin(st["yaw"]) * distance
            self.motion.append((husky, "drive", distance))
            return {"accepted": True, "distance": distance}
        if path == "turn":
            angle = float(payload["angle"])
            # Mirror the measured open-loop gain. The production runtime should
            # compensate it and converge to the requested relative heading.
            st["yaw"] = swarm._norm_angle(
                st["yaw"] + angle * swarm.TURN_GAIN)
            self.motion.append((husky, "turn", angle))
            return {"accepted": True, "angle_rad": angle}
        if path == "stop_robot":
            return {"halted_at": 1.0}
        raise AssertionError(f"unexpected bridge path: {path}")


def test_drive_husky_returns_measured_postcondition():
    fake = FakeBridges()
    with patch.object(swarm, "_bridge_post", side_effect=fake.post):
        result = swarm.tool_drive_husky(husky="husky_ne", distance_m=1.25)

    assert result["completed"] is True
    assert result["start_pose"]["x"] == 3.0
    assert result["final_pose"]["x"] == 4.25
    assert result["distance_achieved_m"] == 1.25
    assert result["signed_distance_achieved_m"] == 1.25
    assert result["target_error_m"] == 0.0


def test_drive_husky_does_not_call_a_stall_completed():
    fake = FakeBridges()
    original = fake.post

    def stalled(husky, path, payload=None, **kwargs):
        if path == "drive_forward":
            fake.motion.append((husky, "drive", float(payload["distance"])))
            return {"accepted": True}
        return original(husky, path, payload, **kwargs)

    with patch.object(swarm, "_bridge_post", side_effect=stalled):
        result = swarm.tool_drive_husky(husky="husky_ne", distance_m=1.0)

    assert result["completed"] is False
    assert result["distance_achieved_m"] == 0.0
    assert result["target_error_m"] == 1.0
    assert "partial result" in result["hint"]


def test_turn_then_drive_owns_ordering_and_verification():
    fake = FakeBridges()
    with patch.object(swarm, "_bridge_post", side_effect=fake.post):
        result = swarm.tool_turn_then_drive(
            husky="husky_nw", angle_deg=90, distance_m=1.5)

    assert result["completed"] is True
    assert result["ordered_steps"] == ["turn", "drive"]
    assert [entry[1] for entry in fake.motion] == ["turn", "drive"]
    assert abs(result["turn"]["residual_error_deg"]) <= 5.0
    assert result["final_pose"]["x"] == -3.0
    assert result["final_pose"]["y"] == 4.5


def test_turn_then_drive_never_drives_while_bridge_is_still_turning():
    fake = FakeBridges()
    fake.state["husky_nw"]["mode"] = "turn"

    with (patch.object(swarm, "_bridge_post", side_effect=fake.post),
          patch.object(swarm.time, "sleep", return_value=None),
          patch.object(swarm.time, "time", side_effect=[0.0, 0.0, 31.0])):
        result = swarm.tool_turn_then_drive(
            husky="husky_nw", angle_deg=90, distance_m=1.5)

    assert result["error"] == "turn step failed"
    assert result["drive_skipped"] is True
    assert result["turn"]["error"] == "robot_not_idle"
    assert fake.motion == []


def test_repeat_turn_then_drive_runs_dependent_legs_sequentially():
    fake = FakeBridges()
    with patch.object(swarm, "_bridge_post", side_effect=fake.post):
        result = swarm.tool_repeat_turn_then_drive(
            husky="husky_ne", angle_deg=90, distance_m=1.0, repetitions=4)

    assert result["completed"] is True
    assert result["completed_legs"] == 4
    assert [kind for _, kind, _ in fake.motion] == [
        "turn", "drive", "turn", "drive", "turn", "drive", "turn", "drive"]
    assert math.hypot(result["final_pose"]["x"] - 3.0,
                      result["final_pose"]["y"] - 3.0) < 0.01


class ScriptedPivot:
    """A husky that reports a WRAPPED yaw (as every real bridge does) while
    the test tracks the true, unwrapped rotation separately."""

    def __init__(self, gain: float = None, deliver_fraction: float = 1.0):
        self.yaw = 0.0
        self.true_rotation = 0.0
        self.gain = swarm.TURN_GAIN if gain is None else gain
        self.deliver_fraction = deliver_fraction
        self.commands = []

    def post(self, husky, path, payload=None, **_):
        if path == "get_robot_state":
            return {"x": 0.0, "y": 0.0, "yaw": self.yaw, "mode": "idle"}
        if path == "turn":
            angle = float(payload["angle"])
            self.commands.append(angle)
            delivered = angle * self.gain * self.deliver_fraction
            self.true_rotation += delivered
            self.yaw = swarm._norm_angle(self.yaw + delivered)
            return {"accepted": True}
        if path in ("drive_forward", "stop_robot"):
            return {"accepted": True}
        raise AssertionError(f"unexpected bridge path: {path}")


def test_turn_measurement_is_unwrapped_past_half_a_turn():
    """The wrap blind spot fixed in husky_omnilink_bridge (2e2471b8), twin.

    The old client aimed at wrap_pi(start + angle) and measured
    wrap_pi(end - start). Both wraps destroy the question. Measured on this
    exact double before the fix: a commanded 2*pi moved the husky NOT AT ALL
    and returned achieved 0.00 deg / error -0.00 deg / completed True, and a
    commanded 3.5 rad rotated -159.46 deg -- the OPPOSITE WAY -- and still
    returned error 0.00 deg / completed True.
    """
    for requested_rad in (2 * math.pi, 3.5, -3.5, math.radians(45)):
        pivot = ScriptedPivot()
        with patch.object(swarm, "_bridge_post", side_effect=pivot.post):
            result = swarm.tool_turn_husky(
                husky="husky_ne", angle_deg=math.degrees(requested_rad))

        true_deg = math.degrees(pivot.true_rotation)
        requested_deg = math.degrees(requested_rad)
        # The husky physically performed the whole requested rotation...
        assert abs(true_deg - requested_deg) < 1.0, (
            f"requested {requested_deg:.2f} deg, husky turned {true_deg:.2f}")
        # ...and the number handed to the model is that same unwrapped value.
        assert abs(result["angle_achieved_deg"] - true_deg) < 0.01
        assert abs(result["residual_error_deg"]) < 1.0
        assert result["completed"] is True
        # No single command may exceed half a turn -- beyond that neither the
        # bridge's target nor a pose difference is unambiguous.
        assert all(abs(c) < math.pi for c in pivot.commands), pivot.commands


def test_turn_cannot_claim_a_rotation_that_did_not_happen():
    """A bridge that accepts a turn and delivers nothing must not read as done."""
    pivot = ScriptedPivot(deliver_fraction=0.0)
    with patch.object(swarm, "_bridge_post", side_effect=pivot.post):
        result = swarm.tool_turn_husky(husky="husky_ne", angle_deg=360.0)

    assert pivot.true_rotation == 0.0
    assert abs(result["angle_achieved_deg"]) < 1e-6
    assert abs(result["residual_error_deg"] + 360.0) < 1.0   # UNWRAPPED
    assert result["completed"] is False
    assert normalize_outcome(result).verified is not True

    # And a partial delivery is reported as the partial it is.
    weak = ScriptedPivot(deliver_fraction=0.5)
    with patch.object(swarm, "_bridge_post", side_effect=weak.post):
        partial = swarm.tool_turn_husky(husky="husky_ne", angle_deg=360.0)
    assert abs(partial["angle_achieved_deg"]
               - math.degrees(weak.true_rotation)) < 0.01


def test_compound_turn_skills_inherit_the_unwrapped_measurement():
    pivot = ScriptedPivot()
    with patch.object(swarm, "_bridge_post", side_effect=pivot.post):
        result = swarm.tool_turn_then_drive(
            husky="husky_ne", angle_deg=math.degrees(3.5), distance_m=0.0)

    assert abs(result["turn"]["angle_achieved_deg"]
               - math.degrees(pivot.true_rotation)) < 0.01
    assert abs(result["turn"]["angle_achieved_deg"] - math.degrees(3.5)) < 1.0


def test_execute_parallel_rejects_duplicate_robot_actions():
    result = swarm.tool_execute_parallel(actions=[
        {"tool": "drive_husky", "args": {"husky": "husky_ne",
                                          "distance_m": 1.0}},
        {"tool": "turn_husky", "args": {"husky": "husky_ne",
                                         "angle_deg": 90}},
    ])

    assert result["error"] == "duplicate_robot_action"
    assert result["husky"] == "husky_ne"


def test_find_and_drive_uses_live_selection_and_moves_only_winner():
    fake = FakeBridges()
    with patch.object(swarm, "_bridge_post", side_effect=fake.post):
        result = swarm.tool_find_and_drive(
            criteria="nearest_to_centre", distance_m=1.0)

    assert result["selected_husky"] == "husky_se"
    assert fake.motion == [("husky_se", "drive", 1.0)]
    assert result["drive"]["final_pose"]["x"] == 2.0


def test_visit_and_return_saves_before_both_verified_legs():
    order = []

    def save(**kwargs):
        order.append(("save", kwargs))
        return {"saved": True, "name": kwargs["name"], "x": 3.0, "y": -3.0}

    def outbound(**kwargs):
        order.append(("outbound", kwargs))
        return {"arrived": True, "final_xy": [1.0, -1.0]}

    def returned(**kwargs):
        order.append(("return", kwargs))
        return {"within_tolerance": True, "final_xy": [3.0, -3.0]}

    with (patch.object(swarm, "tool_save_waypoint", side_effect=save),
          patch.object(swarm, "tool_drive_to_xy", side_effect=outbound),
          patch.object(swarm, "tool_drive_to_waypoint",
                       side_effect=returned)):
        result = swarm.tool_visit_and_return(
            husky="husky_se", x=1.0, y=-1.0,
            checkpoint_name="inspection_home")

    assert [step for step, _ in order] == ["save", "outbound", "return"]
    assert result["checkpoint_saved"] is True
    assert result["outbound_completed"] is True
    assert result["return_completed"] is True
    assert result["completed"] is True


def test_drive_with_fallback_runs_fallback_only_after_guard_refusal():
    primary = {"error": "out_of_arena", "arena_bound_m": 7.0}
    fallback = {"arrived": True, "final_xy": [5.2, 3.0]}
    with (patch.object(swarm, "tool_drive_husky", return_value=primary),
          patch.object(swarm, "tool_drive_to_xy",
                       return_value=fallback) as drive_to):
        result = swarm.tool_drive_with_fallback(
            husky="husky_ne", distance_m=2.0,
            fallback_x=5.2, fallback_y=3.0)

    drive_to.assert_called_once()
    assert result["primary_refused"] is True
    assert result["fallback_attempted"] is True
    assert result["completed"] is True


def test_execute_parallel_needs_positive_arrival_evidence():
    """R8. drive_to_xy / drive_radial -- the two motion tools the shipped
    prompt puts INSIDE execute_parallel -- report `arrived`, never `completed`,
    and set no `error` when they fall short. The old negative rule ("a failure
    is an `error` key or completed is False") therefore saw nothing: "fan out
    to the corners" with one husky wedged 3 m short returned
    all_completed: True, failure_indexes: [], and _lib.supervision then
    stamped the batch verified. Four arrivals reported; three happened."""
    arrived = {"husky": "husky_ne", "final_xy": [6.0, 6.0], "error_m": 0.02,
               "arrived": True}
    wedged = {"husky": "husky_nw", "final_xy": [-3.1, 3.4], "error_m": 3.02,
              "arrived": False}
    outcomes = [arrived, wedged]

    with patch.object(swarm, "dispatch",
                      side_effect=lambda t, a, gate=True: outcomes.pop(0)):
        result = swarm.tool_execute_parallel(actions=[
            {"tool": "drive_to_xy",
             "args": {"husky": "husky_ne", "x": 6.0, "y": 6.0}},
            {"tool": "drive_to_xy",
             "args": {"husky": "husky_nw", "x": -6.0, "y": 6.0}},
        ])

    assert result["failure_indexes"] == [1]
    assert result["all_completed"] is False
    # And the shared supervisor must agree -- `all_completed` is one of its
    # completion keys, so a wrong verdict here propagates into every ledger.
    assert normalize_outcome(result).verified is not True


def test_execute_parallel_never_counts_a_missing_postcondition_as_success():
    """A motion tool that reports nothing measurable is unverified, not done."""
    with patch.object(swarm, "dispatch",
                      return_value={"accepted": True}):
        result = swarm.tool_execute_parallel(actions=[
            {"tool": "set_husky_velocity",
             "args": {"husky": "husky_ne", "linear": 0.2}},
        ])

    assert result["failure_indexes"] == []
    assert result["unverified_indexes"] == [0]
    assert result["all_completed"] is False
    assert "not proof of completion" in result["hint"].lower()

    # A non-actuating tool has no postcondition to publish, so a clean read is
    # all it can offer and all that is required of it.
    with patch.object(swarm, "dispatch",
                      return_value={"x": 3.0, "y": 3.0, "mode": "idle"}):
        reads = swarm.tool_execute_parallel(actions=[
            {"tool": "get_husky_status", "args": {"husky": "husky_ne"}},
        ])
    assert reads["all_completed"] is True
    assert reads["unverified_indexes"] == []


def test_drive_to_xy_publishes_its_measured_verdict_as_completed():
    """R8, the other half: the tool owns the verdict its supervisors read."""
    fake = FakeBridges()
    with patch.object(swarm, "_bridge_post", side_effect=fake.post):
        arrived = swarm.tool_drive_to_xy(husky="husky_ne", x=4.0, y=3.0)
    assert arrived["arrived"] is True
    assert arrived["completed"] is True
    assert normalize_outcome(arrived).state == "completed"

    # A husky that cannot reach the target says so in BOTH keys.
    with patch.object(swarm, "_bridge_post", side_effect=fake.post), \
         patch.object(swarm, "_turn_to_heading", return_value=0.0), \
         patch.object(swarm, "_settle", return_value=True):
        stuck = swarm.tool_drive_to_xy(
            husky="husky_sw", x=-3.0, y=3.0, max_iters=0)
    assert stuck["arrived"] is False
    assert stuck["completed"] is False
    assert normalize_outcome(stuck).failed is True


def test_arena_guard_fails_closed_when_telemetry_is_unreadable():
    """R27. `_bridge_post` RETURNS an error dict instead of raising, so a
    timed-out state read fell through to `state.get("x", SPAWN[husky])`: a
    husky actually at (6.5, 3.0) was guarded as if it sat at its (3.0, 3.0)
    spawn. A 3.5 m command projecting to x = 10.0 m passed a 7.0 m bound. For
    set_husky_velocity this guard is the ONLY bound check that exists."""
    timed_out = {"error": "bridge unreachable: timed out", "husky": "husky_ne"}

    with patch.object(swarm, "_bridge_post", return_value=timed_out):
        verdict = swarm._guard_drive_distance("husky_ne", 3.5)
    assert verdict is not None
    assert verdict["error"] == "pose_unavailable"

    # A pose missing any component is just as blind as no pose at all.
    with patch.object(swarm, "_bridge_post", return_value={"x": 6.5, "y": 3.0}):
        assert swarm._guard_drive_distance(
            "husky_ne", 3.5)["error"] == "pose_unavailable"

    # Nothing reaches the bridge -- the refusal is not advisory.
    reached = []

    def blind(husky, path, payload=None, **_):
        if path == "get_robot_state":
            return dict(timed_out)
        reached.append((husky, path))
        return {"accepted": True}

    with patch.object(swarm, "_bridge_post", side_effect=blind):
        assert swarm.tool_set_husky_velocity(
            husky="husky_ne", linear=0.3)["error"] == "pose_unavailable"
        assert swarm.tool_drive_husky(
            husky="husky_ne", distance_m=1.0)["error"] == "pose_unavailable"
    assert reached == []

    # A readable pose still behaves exactly as before: in-bounds passes,
    # out-of-bounds is refused as out_of_arena (not as a telemetry fault).
    with patch.object(swarm, "_bridge_post",
                      return_value={"x": 6.5, "y": 3.0, "yaw": 0.0}):
        assert swarm._guard_drive_distance("husky_ne", 0.2) is None
        assert swarm._guard_drive_distance(
            "husky_ne", 3.5)["error"] == "out_of_arena"


def test_parallel_halt_on_error_contains_the_whole_fleet():
    actions = [
        {"tool": "drive_husky",
         "args": {"husky": "husky_ne", "distance_m": 1.0}},
        {"tool": "drive_husky",
         "args": {"husky": "husky_nw", "distance_m": 1.0}},
    ]
    with (patch.object(swarm, "dispatch",
                       return_value={"error": "estop_engaged"}),
          patch.object(swarm, "tool_halt_all",
                       return_value={"halted": {}}) as halt):
        result = swarm.tool_execute_parallel(actions, halt_on_error=True)

    halt.assert_called_once()
    assert result["failure_indexes"] == [0, 1]
    assert result["all_completed"] is False
    assert result["halted_on_error"] is True


def test_production_profile_reduces_schema_entropy_without_losing_discovery():
    with patch.dict(os.environ, {
        "HUSKY_SWARM_TOOL_TIER": "performance",
        "HUSKY_SWARM_PROMPT_PROFILE": "performance",
    }):
        advertised = swarm._advertised_tools()
        main_task = swarm._main_task_for_runtime("legacy " * 3000)

    assert len(advertised) < len(swarm.TOOLS) / 2
    assert {"get_husky_status", "turn_then_drive", "find_and_drive",
            "visit_and_return", "drive_with_fallback",
            "repeat_turn_then_drive",
            "execute_parallel", "find_tools", "invoke_tool"} <= set(advertised)
    assert "ACTION ROUTER" in main_task
    assert "one drive_radial per robot inside execute_parallel" in main_task
    assert "calculate quadrant-specific turn angles" in main_task
    assert main_task.index("Explicit unit-agent work") < main_task.index(
        '"All", "simultaneously"')
    assert "MEMORY IS HISTORICAL, NEVER LIVE STATE" in main_task
    assert len(main_task) < 2500


def test_extended_profile_remains_an_explicit_compatibility_mode():
    original = "custom operator prompt"
    with patch.dict(os.environ, {
        "HUSKY_SWARM_PROMPT_PROFILE": "extended",
    }):
        assert swarm._main_task_for_runtime(original) == original


# ── prompt <-> registry agreement ──────────────────────────────────────────
#
# Two releases shipped a prompt whose tool count was wrong (28 vs 40, then
# 40 vs 45), and both times the missing entries were the newest compound
# closed-loop skills -- the class this agent's design argues matters most.
# The count is parsed from the SOURCE with ast rather than read from
# swarm.TOOLS so a broken import cannot make these vacuously pass.

def _registered_tool_names():
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "TOOLS":
            return [key.value for key in node.value.keys]
    raise AssertionError("TOOLS registry not found in swarm_agent.py")


def _profile_main_task():
    path = MODULE_PATH.parent / "profile.json"
    return json.loads(path.read_text(encoding="utf-8"))["settings"]["mainTask"]


_COUNT_CLAIM = re.compile(r"\b(\d+)(?= (?:registered )?tools\b| primitives\b)")


def test_shipped_prompt_names_every_registered_tool():
    names = _registered_tool_names()
    assert sorted(names) == sorted(swarm.TOOLS)
    main_task = _profile_main_task()

    missing = [n for n in names if n not in main_task]
    assert missing == [], (
        f"profile.json's layer enumeration omits {missing}. Add them to a "
        f"LAYER block -- the model cannot compose a primitive it is never told "
        f"about.")
    assert swarm.tools_missing_from_prompt(main_task) == []


def test_prompt_tool_count_claims_match_the_registry():
    names = _registered_tool_names()
    main_task = _profile_main_task()
    claims = _COUNT_CLAIM.findall(main_task)
    assert claims, "the prompt no longer states a tool count -- update this test"
    assert set(claims) == {str(len(names))}, (
        f"prompt claims {set(claims)} tools, registry holds {len(names)}")


def test_push_time_count_patch_is_generated_not_a_literal_match():
    """The old push-time fix replaced ONE exact sentence, so the next drift
    sailed straight through. Any numeric claim must now be retargeted."""
    stale = ("You hold 28 tools across seven layers. Budget: 12 calls / 30s. "
             "They are compositions of the 28 primitives.")
    fixed = swarm._retarget_tool_count_claims(stale)
    n = len(swarm.TOOLS)
    assert f"You hold {n} tools" in fixed
    assert f"the {n} primitives" in fixed
    assert "12 calls / 30s" in fixed           # unrelated numbers untouched
    assert swarm._retarget_tool_count_claims(fixed) == fixed


def test_push_appends_any_tool_the_prompt_forgot():
    """Belt and braces: even if profile.json drifts, the LIVE prompt is never
    short of the registry."""
    base = json.loads((MODULE_PATH.parent / "profile.json").read_text(
        encoding="utf-8"))["settings"]
    dropped = base["mainTask"].replace("visit_and_return", "REMOVED_BY_TEST")
    assert swarm.tools_missing_from_prompt(dropped) == ["visit_and_return"]

    with patch.dict(os.environ, {"HUSKY_SWARM_PROMPT_PROFILE": "extended"}):
        built = swarm.build_settings(
            dict(base, mainTask=dropped), "http://127.0.0.1:51520/tool")
    assert "visit_and_return" in built["mainTask"]


def test_readme_tool_layers_match_the_registry():
    """The README's per-layer lists are the human-facing copy of the same
    registry; keeping them asserted is what makes the count hard to drift."""
    readme = (MODULE_PATH.parent / "README.md").read_text(encoding="utf-8")
    section = readme[readme.index("## Tool layers"):readme.index("## Showcase prompts")]
    documented = set(re.findall(r"`([a-z_]+)`", section))
    registered = set(_registered_tool_names())
    assert registered - documented == set(), (
        f"README Tool layers omits {sorted(registered - documented)}")
    assert documented - registered == set(), (
        f"README Tool layers lists tools that no longer exist: "
        f"{sorted(documented - registered)}")
