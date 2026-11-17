import pytest

from agentbench import frontier


def row(task, outcome="PASS", **overrides):
    value = {
        "suite": "agenticsimbench/v0.3",
        "protocol": {"id": "single-run-under-ceiling/2026-08-10"},
        "task": task,
        "sim": "omnisim",
        "condition": "codex_full_surface",
        "agent": {"model": "gpt-test"},
        "outcome": outcome,
        "_source": task,
    }
    value.update(overrides)
    return value


def always_ready(_task, _sim):
    return True, []


def test_frontier_is_the_contiguous_prefix_not_the_flashiest_pass():
    rows = {
        "R1_lidar_nav": row("R1_lidar_nav", "FAIL"),
        "A1_husky_swarm_10": row("A1_husky_swarm_10", "PASS"),
    }
    report = frontier.build_report(rows, sim="omnisim",
                                   readiness_fn=always_ready)
    track = next(t for t in report["tracks"]
                 if t["id"] == "autonomy_scale")
    assert track["measured_frontier"] == 0
    assert track["tasks"][1]["measured_pass"] is True


def test_claimable_frontier_stops_at_an_ungated_task():
    rows = {
        "C1_parse_error_fix": row("C1_parse_error_fix"),
        "C2_fall_through_floor": row("C2_fall_through_floor"),
    }

    def readiness_fn(task, _sim):
        return (task == "C1_parse_error_fix",
                [] if task == "C1_parse_error_fix" else ["gate missing"])

    report = frontier.build_report(rows, sim="omnisim",
                                   readiness_fn=readiness_fn)
    track = next(t for t in report["tracks"] if t["id"] == "debug_loop")
    assert track["measured_frontier"] == 2
    assert track["claimable_frontier"] == 1


def test_selection_refuses_duplicate_runs_and_mixed_protocols():
    with pytest.raises(frontier.FrontierError, match="more than one"):
        frontier.select_rows(
            [row("R1_lidar_nav"), row("R1_lidar_nav")],
            sim="omnisim")

    mixed = [
        row("R1_lidar_nav"),
        row("A1_husky_swarm_10", protocol={"id": "different"}),
    ]
    with pytest.raises(frontier.FrontierError, match="protocol"):
        frontier.select_rows(mixed, sim="omnisim")


def test_every_implemented_task_appears_once_in_the_envelope():
    assert len(frontier.TASKS) == 10
    assert set(frontier.TASKS) == {
        "A1_husky_swarm_10", "B1_overlap_audit",
        "B2_subject_in_frame", "B3_measure_and_report",
        "C1_parse_error_fix", "C2_fall_through_floor",
        "R1_lidar_nav", "R2_arm_reach", "R3_pick_and_place",
        "R4_mobile_manipulation",
    }
