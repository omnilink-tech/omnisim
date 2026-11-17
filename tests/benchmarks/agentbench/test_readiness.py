"""Unit tests for the mechanically generated readiness table."""

from agentbench import readiness


def test_answer_tasks_have_a_deliverable_convention():
    for task_id in ("B1_overlap_audit", "B3_measure_and_report"):
        state, detail = readiness._deliverable(task_id, "omnisim")
        assert state == readiness.OK, detail
        assert detail == "final answer text"


def test_world_task_still_reports_its_registered_artifact():
    state, detail = readiness._deliverable(
        "C2_fall_through_floor", "omnisim")
    assert state == readiness.OK
    assert detail == "fall_through.wbt"


def test_red_evidence_is_a_publication_gate():
    state, detail = readiness._red_evidence("B1_overlap_audit")
    assert state == readiness.OK, detail

    state, detail = readiness._red_evidence("R1_lidar_nav")
    assert state == readiness.NO
    assert "R1.1" in detail
