# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Tests for the model-independent Husky Maze long-horizon executor."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "agents" / "production"))

# Import through the agent's namespace package, NOT by adding the agent dir to
# sys.path and importing bare `tools`: every production agent ships its own
# `tools` package, so the bare name collides in sys.modules and whichever test
# is collected second raises at COLLECTION TIME -- aborting the whole session.
from husky_maze.tools import husky  # noqa: E402


def test_cell_mission_retries_slower_and_returns_to_measured_start():
    calls = []
    stops = []

    def drive(**kwargs):
        calls.append(kwargs)
        target = (kwargs["to_col"], kwargs["to_row"])
        attempts_for_target = sum(
            1
            for call in calls
            if (call["to_col"], call["to_row"]) == target
        )
        if target == (10, 10) and attempts_for_target == 1:
            return {"arrived": False, "reason": "max_replans_exhausted"}
        return {"arrived": True, "cell": list(target)}

    with (
        patch.object(
            husky,
            "bridge_get",
            return_value={"current_cell": {"col": 0, "row": 10}},
        ),
        patch.object(husky, "_impl_drive_to_cell", side_effect=drive),
        patch.object(
            husky,
            "bridge_post",
            side_effect=lambda _path, body: stops.append(body) or {"halted": True},
        ),
        patch.object(
            husky,
            "_impl_complete_mission",
            return_value={"mission_complete": True, "verified": True},
        ),
    ):
        result = husky._impl_execute_cell_mission(
            waypoints=[[10, 10], [10, 0]],
            rationale="visit objectives and return",
            speed=0.42,
        )

    assert result["completed"] is True
    assert result["verified"] is True
    assert result["waypoints"] == [[10, 10], [10, 0], [0, 10]]
    assert result["ledger"]["completed_steps"] == 3
    assert calls[0]["speed"] == 0.42
    assert calls[1]["speed"] < calls[0]["speed"]
    assert stops == [{"action": "stop"}]


def test_cell_mission_stops_before_downstream_waypoint_on_recovery_failure():
    calls = []

    def never_arrives(**kwargs):
        calls.append((kwargs["to_col"], kwargs["to_row"]))
        return {"arrived": False, "reason": "wedged"}

    with (
        patch.object(
            husky,
            "bridge_get",
            return_value={"current_cell": {"col": 0, "row": 10}},
        ),
        patch.object(husky, "_impl_drive_to_cell", side_effect=never_arrives),
        patch.object(husky, "bridge_post", return_value={"halted": True}),
    ):
        result = husky._impl_execute_cell_mission(
            waypoints=[[10, 10], [10, 0]],
            rationale="visit objectives and return",
            attempts_per_waypoint=2,
        )

    assert result["error"] == "cell_mission_failed"
    assert result["ledger"]["failed_step"] == "waypoint_1"
    assert result["ledger"]["completed_steps"] == 0
    assert calls == [(10, 10), (10, 10)]
