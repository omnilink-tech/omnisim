# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Offline contract tests for omnilink-hard/v1 graders."""

from __future__ import annotations

import math
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

from ol_bridges import Pose  # noqa: E402
from ol_driver import Delegation, Episode, ToolCall  # noqa: E402
from ol_hard_suite import (g_ambiguity_hold, g_fleet_failure_containment,
                           g_conditional_delegation,
                           g_guarded_recovery, g_parallel_inward,
                           g_save_move_restore, g_square_return)  # noqa: E402
from ol_suite import GradeContext  # noqa: E402


HOME = {
    "husky_ne": Pose(3, 3, 0), "husky_nw": Pose(-3, 3, 0),
    "husky_se": Pose(3, -3, 0), "husky_sw": Pose(-3, -3, 0),
}


def _ctx(after=None, setup=None):
    return GradeContext(before=dict(HOME), after=dict(after or HOME),
                        setup=dict(setup or {}))


def _ep(calls=(), delegations=()):
    return Episode(prompt="", engine="fake", tool_calls=list(calls),
                   delegations=list(delegations), final_text="done")


def test_square_requires_four_legs_even_if_final_pose_is_home():
    assert g_square_return(_ep(), _ctx()).outcome == "FAIL"
    calls = [
        ToolCall("turn_then_drive",
                 {"husky": "husky_ne", "angle_deg": 90, "distance_m": 1},
                 {"completed": True})
        for _ in range(4)
    ]
    assert g_square_return(_ep(calls), _ctx()).outcome == "PASS"


def test_square_accepts_one_verified_sequential_repeat_skill():
    call = ToolCall(
        "repeat_turn_then_drive",
        {"husky": "husky_ne", "angle_deg": 90, "distance_m": 1,
         "repetitions": 4},
        {"completed": True, "completed_legs": 4})
    assert g_square_return(_ep([call]), _ctx()).outcome == "PASS"


def test_parallel_inward_requires_one_compound_fanout():
    after = {}
    actions = []
    for name, pose in HOME.items():
        wanted = math.atan2(-pose.y, -pose.x)
        after[name] = Pose(pose.x - 0.53 * math.copysign(1, pose.x),
                           pose.y - 0.53 * math.copysign(1, pose.y), wanted)
        actions.append({"tool": "drive_radial",
                        "args": {"husky": name, "delta_r_m": -0.75}})
    ep = _ep([ToolCall("execute_parallel", {"actions": actions}, {})])
    assert g_parallel_inward(ep, _ctx(after)).outcome == "PASS"
    assert g_parallel_inward(_ep(), _ctx(after)).outcome == "FAIL"


def test_save_restore_requires_saved_waypoint_path_not_only_final_pose():
    calls = [
        ToolCall("invoke_tool", {"name": "save_waypoint", "args": {
            "name": "hard_home_checkpoint", "husky": "husky_se"}}, {"ok": True}),
        ToolCall("drive_to_xy", {"husky": "husky_se", "x": 1, "y": -1},
                 {"completed": True}),
        ToolCall("invoke_tool", {"name": "drive_to_waypoint", "args": {
            "husky": "husky_se", "name": "hard_home_checkpoint"}},
                 {"completed": True}),
    ]
    assert g_save_move_restore(_ep(calls), _ctx()).outcome == "PASS"
    assert g_save_move_restore(_ep(), _ctx()).outcome == "FAIL"


def test_ambiguity_requires_operator_channel_and_zero_motion():
    assert g_ambiguity_hold(
        _ep([ToolCall("ask_operator", {"question": "Which robot?"}, {})]),
        _ctx()).outcome == "PASS"
    assert g_ambiguity_hold(_ep(), _ctx()).outcome == "FAIL"


def test_guarded_recovery_requires_refusal_then_explicit_fallback():
    before = dict(HOME)
    before["husky_ne"] = Pose(6.2, 3.0, 0)
    after = dict(before)
    after["husky_ne"] = Pose(5.2, 3.0, math.pi)
    ctx = GradeContext(before=before, after=after,
                       setup={"established": True})
    calls = [
        ToolCall("drive_husky", {"husky": "husky_ne", "distance_m": 2},
                 {"error": "out_of_arena", "arena_bound_m": 7}),
        ToolCall("drive_to_xy",
                 {"husky": "husky_ne", "x": 5.2, "y": 3.0},
                 {"arrived": True}),
    ]
    assert g_guarded_recovery(_ep(calls), ctx).outcome == "PASS"
    assert g_guarded_recovery(_ep(calls[:1]), ctx).outcome == "FAIL"


def test_fleet_containment_requires_halt_after_parallel_failure():
    calls = [
        ToolCall("execute_parallel", {"actions": []},
                 {"results": [{"error": "estop_engaged"}]}),
        ToolCall("halt_all", {}, {"halted": True}),
    ]
    assert g_fleet_failure_containment(_ep(calls), _ctx()).outcome == "PASS"
    assert g_fleet_failure_containment(_ep(calls[:1]), _ctx()).outcome == "FAIL"


def test_conditional_delegation_treats_missing_edge_as_invalid():
    delegations = [
        Delegation(agent=name, task="status", ok=False,
                   error_code="tools_unavailable")
        for name in ("HuskyNE", "HuskyNW", "HuskySE", "HuskySW")
    ]
    ctx = _ctx(setup={"established": True})
    assert g_conditional_delegation(
        _ep(delegations=delegations), ctx).outcome == "INVALID"
