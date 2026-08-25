# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Tests for shared PRO-agent supervision and Mission Captain ledgers."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any, Dict, List
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "agents" / "production"))

from _lib.supervision import (  # noqa: E402
    execute_verified_sequence,
    failure_error,
    find_duplicate_resource_claims,
    normalize_outcome,
)
# Import through the agent's namespace package, NOT by adding the agent dir to
# sys.path and importing bare `tools`: every production agent ships its own
# `tools` package, so the bare name collides in sys.modules and whichever test
# is collected second raises at COLLECTION TIME -- aborting the whole session.
from mission_captain.tools import captain, orchestration  # noqa: E402


def _load_husky_maze_runner():
    """Load the REAL Husky Maze runner module under a private name.

    These tests exist because the captain<->specialist seam was certified green
    by mocks: the old suite patched `_impl_delegate_to_agent` to return
    `mission_complete: True`, a value the real function could not produce, and
    patched `http_get` to return a key `/status` never carried. So the
    specialist side here is the shipped `status_snapshot`, never a literal.
    """
    path = ROOT / "agents" / "production" / "husky_maze" / "husky_maze_agent.py"
    spec = importlib.util.spec_from_file_location("_husky_maze_runner_uut", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HUSKY_MAZE_RUNNER = _load_husky_maze_runner()

# What the maze bridge's /state actually returns once a complete_mission claim
# has been accepted (husky_omnilink_bridge.py, the `mission_complete` field).
SOLVED_BRIDGE_STATE: Dict[str, Any] = {
    "x": 10.0, "y": 0.0, "yaw": 0.0, "mode": "idle", "fault": None,
    "goal_reached": True, "mission_complete": True,
    "current_cell": {"col": 10, "row": 0, "x": 10.0, "y": 0.0, "drift_m": 0.0},
}
FRESH_BRIDGE_STATE: Dict[str, Any] = dict(
    SOLVED_BRIDGE_STATE, x=0.0, goal_reached=False, mission_complete=False,
    current_cell={"col": 0, "row": 10, "x": 0.0, "y": 10.0, "drift_m": 0.0})


def runner_status(bridge_state: Dict[str, Any],
                  activity_log: List[Dict[str, Any]] = ()) -> Dict[str, Any]:
    """The specialist's real /status payload for a given bridge /state."""
    with patch.object(HUSKY_MAZE_RUNNER, "bridge_get", return_value=dict(bridge_state)):
        return HUSKY_MAZE_RUNNER.status_snapshot(list(activity_log))


class FakeOmniLinkClient:
    """A specialist that narrates and keeps calling one tool, forever.

    It NEVER reports completion itself -- completion has to come from the
    bridge, through /status, which is the seam under test.
    """

    def __init__(self, **_: Any) -> None:
        self.chats = 0

    def list_profiles(self):
        return [{
            "name": "Husky Maze",
            "id": "profile-1",
            "settings": {"mainTask": "solve the maze", "availableToolDetails": []},
        }]

    def chat(self, **_: Any):
        self.chats += 1
        return {"text": "Driving toward the goal cell.",
                "toolCalls": [{"name": "get_state", "arguments": {}}]}


def test_numeric_error_is_a_residual_not_a_failure():
    """R20. The maze bridge names its NUMERIC residual `error`.

    A healthy blocking drive returns
    {"commanded": 1.0, "achieved": 0.97, "error": -0.03, ...}. Reading that as
    a failure marked every successful blocking drive/turn failed -- only an
    exactly-0.0 residual escaped -- and made the runner answer status "err"
    over HTTP for work that went right.
    """
    healthy_drive = {
        "commanded": 1.0, "achieved": 0.97, "error": -0.03, "unit": "m",
        "settled": True, "fault": None,
    }
    outcome = normalize_outcome(healthy_drive)
    assert outcome.failed is False
    assert outcome.state != "failed"
    assert outcome.reason is None
    # And a residual of exactly zero is not special-cased into the only pass.
    assert normalize_outcome(dict(healthy_drive, error=0.0)).failed is False

    # Real failures still fail, in every shape a shipped tool emits.
    assert normalize_outcome({"error": "bridge unreachable"}).failed is True
    assert normalize_outcome({"error": {"code": 502}}).failed is True
    assert normalize_outcome({"error": True}).failed is True

    # failure_error is the shared decoder; runner_base derives its HTTP
    # status ("ok" vs "err") from the same call, so the two cannot disagree.
    assert failure_error(healthy_drive) is None
    assert failure_error({"error": "boom"}) == "boom"
    assert failure_error({}) is None


def test_accepted_is_not_completed():
    outcome = normalize_outcome({"accepted": True})

    assert outcome.state == "accepted"
    assert outcome.completed is None
    assert outcome.verified is None
    assert outcome.failed is False


def test_verified_sequence_stops_before_dependent_work_after_failure():
    calls = []

    def execute(step):
        calls.append(step["id"])
        return {"completed": step["id"] != "navigate"}

    ledger = execute_verified_sequence(
        [
            {"id": "inspect"},
            {"id": "navigate"},
            {"id": "report"},
        ],
        execute,
    )

    assert calls == ["inspect", "navigate"]
    assert ledger["completed"] is False
    assert ledger["completed_steps"] == 1
    assert ledger["failed_step"] == "navigate"
    assert ledger["stopped_early"] is True


def test_duplicate_resource_claims_are_mechanical():
    duplicates = find_duplicate_resource_claims(
        [
            {"robot": "a"},
            {"robot": "b"},
            {"robot": "a"},
            {"robot": "a"},
        ],
        lambda item: item["robot"],
    )

    assert duplicates == [
        {"resource": "a", "first_index": 0, "duplicate_index": 2},
        {"resource": "a", "first_index": 0, "duplicate_index": 3},
    ]


def test_specialist_status_publishes_the_bridge_mission_complete_flag():
    """R7. The captain polls the RUNNER's /status; the flag lives on the BRIDGE.

    This assertion is the whole defect: `status_snapshot` never emitted
    `mission_complete`, so `_impl_delegate_to_agent` polled for a key that
    could not appear and every delegation burnt its full turn budget. If the
    runner ever stops forwarding the flag -- or forwards it from the wrong
    endpoint -- this fails.
    """
    status = runner_status(SOLVED_BRIDGE_STATE)
    assert status["mission_complete"] is True
    assert status["mission_complete_source"] == "bridge"

    fresh = runner_status(FRESH_BRIDGE_STATE)
    assert fresh["mission_complete"] is False
    assert fresh["mission_complete_source"] == "bridge"


def test_specialist_status_never_invents_completion_when_the_bridge_is_down():
    """An unreadable bridge must degrade to None + a named source, not False.

    A fabricated False is indistinguishable from a real one, and the captain
    would then blame the robot for an outage in the runner's own telemetry.
    """
    with patch.object(HUSKY_MAZE_RUNNER, "bridge_get",
                      return_value={"error": "bridge unreachable: timed out"}):
        blind = HUSKY_MAZE_RUNNER.status_snapshot([])
    assert blind["mission_complete"] is None
    assert blind["mission_complete_source"] == "unavailable"

    # With prior observed evidence it may answer, but must say it is stale.
    observed = [{"data": {"tool": "get_state",
                          "result": {"mission_complete": True}}}]
    with patch.object(HUSKY_MAZE_RUNNER, "bridge_get",
                      return_value={"error": "bridge unreachable: timed out"}):
        degraded = HUSKY_MAZE_RUNNER.status_snapshot(observed)
    assert degraded["mission_complete"] is True
    assert degraded["mission_complete_source"] == "activity_log"


def test_mission_plan_rejects_stale_specialist_completion():
    # The stale payload is the REAL /status of a runner whose bridge still has
    # a mission_complete claim latched -- not a hand-written dict.
    with patch.object(
        orchestration,
        "http_get",
        return_value=runner_status(SOLVED_BRIDGE_STATE),
    ):
        result = orchestration._impl_execute_mission_plan(
            objective="visit all corners",
            steps=[{"agent": "Husky Maze", "task": "visit all corners"}],
        )

    assert result["error"] == "specialist_not_fresh"


def test_mission_plan_rejects_a_specialist_that_cannot_report_completion():
    """A /status with no boolean flag can never verify a leg -- say so early."""
    flagless = runner_status(SOLVED_BRIDGE_STATE)
    flagless.pop("mission_complete")
    with patch.object(orchestration, "http_get", return_value=flagless):
        result = orchestration._impl_execute_mission_plan(
            objective="visit all corners",
            steps=[{"agent": "Husky Maze", "task": "visit all corners"}],
        )

    assert result["error"] == "specialist_cannot_report_completion"


def _run_one_leg_mission(objective: str, task: str) -> Dict[str, Any]:
    """Drive the WHOLE captain seam with only the network faked.

    Nothing here patches `_impl_delegate_to_agent`: the real sub-chat loop
    runs, dispatches the specialist's tool call, and polls the specialist's
    real `status_snapshot` output for `mission_complete`. The bridge starts
    unsolved and flips once the specialist's tool call lands -- which is what
    a solved mission looks like from the captain's side.
    """
    solved = {"yet": False}

    def fake_http_get(url, timeout=3.0):
        return runner_status(
            SOLVED_BRIDGE_STATE if solved["yet"] else FRESH_BRIDGE_STATE)

    def fake_http_post(url, payload, timeout=45.0):
        solved["yet"] = True
        return {"result": {"ok": True, "mission_complete": True}}

    orchestration._MISSION_PLANS.clear()
    with (
        patch.dict("os.environ", {"OMNI_KEY": "olink_test"}),
        patch.object(orchestration, "OmniLinkClient", FakeOmniLinkClient),
        patch.object(orchestration, "http_get", side_effect=fake_http_get),
        patch.object(orchestration, "http_post", side_effect=fake_http_post),
        patch.object(orchestration.time, "sleep", return_value=None),
    ):
        return orchestration._impl_execute_mission_plan(
            objective=objective,
            steps=[{"id": "tour", "agent": "Husky Maze", "task": task}],
        )


def test_mission_plan_and_captain_claim_share_verified_ledger():
    """R7, end to end. RED before the fix: the plan came back unverified and
    complete_mission answered `mission_plan_not_verified` for a mission the
    specialist had actually solved."""
    plan = _run_one_leg_mission(
        "visit all corners and return", "visit all corners and return to start")

    assert plan["completed"] is True
    assert plan["verified"] is True
    assert plan["ledger"]["completed_steps"] == 1
    # It must not have ground through the whole 30-turn budget to get there.
    delegation = plan["ledger"]["steps"][0]["result"]
    assert delegation["mission_complete"] is True
    assert delegation["turns"] < orchestration.SUB_AGENT_MAX_TURNS

    claim = captain._impl_complete_mission(
        rationale="The specialist completed the verified tour.",
        plan_id=plan["plan_id"],
    )
    assert claim["captain_mission_complete"] is True
    assert claim["claim"]["plan_id"] == plan["plan_id"]


def test_a_verified_plan_backs_exactly_one_completion_claim():
    """R26. `_MISSION_PLANS` is process-lifetime and never pruned, so without a
    consumed marker a stale plan_id kept passing the gate: mission B's claim
    ("found the red cylinder") was accepted against mission A's plan and the
    audit record carried A's objective and A's legs."""
    plan = _run_one_leg_mission(
        "MISSION A: visit all four maze corners", "visit all corners")

    first = captain._impl_complete_mission(
        rationale="Toured all four corners and returned.",
        plan_id=plan["plan_id"])
    assert first["captain_mission_complete"] is True

    second = captain._impl_complete_mission(
        rationale="found the red cylinder", plan_id=plan["plan_id"])
    assert second["error"] == "mission_plan_already_claimed"
    assert second["objective"] == "MISSION A: visit all four maze corners"
    assert second["claimed_by"]["rationale"].startswith("Toured all four")
    assert "captain_mission_complete" not in second


def test_captain_cannot_claim_without_plan_evidence():
    result = captain._impl_complete_mission(rationale="I think it is done.")

    assert result["error"] == "plan_id is required"


def test_current_execution_contract_precedes_legacy_profile_guidance():
    slim = orchestration._slim_settings_for_subchat(
        {
            "mainTask": "Legacy: execute every primitive by hand.",
            "executionContract": "Use the supervised mission primitive.",
        }
    )

    assert slim["mainTask"].startswith("CURRENT EXECUTION CONTRACT")
    assert slim["mainTask"].index("Use the supervised") < slim["mainTask"].index(
        "Legacy:"
    )
