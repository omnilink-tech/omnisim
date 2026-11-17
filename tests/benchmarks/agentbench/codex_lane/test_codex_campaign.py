import json

import pytest

from agentbench.codex_lane import run_codex_campaign as campaign


def test_plan_records_unimplemented_arms_instead_of_scoring_them_as_failures():
    cells, omitted = campaign.make_plan(
        ("omnisim", "mujoco"), ("R1_lidar_nav", "B1_overlap_audit"))
    assert ("omnisim", "R1_lidar_nav") in cells
    assert ("omnisim", "B1_overlap_audit") in cells
    assert ("mujoco", "R1_lidar_nav") in cells
    assert {v["task"] for v in omitted if v["sim"] == "mujoco"} == {
        "B1_overlap_audit"}


def test_campaign_resumes_only_a_complete_matching_cell(tmp_path, monkeypatch):
    root = tmp_path / "stage"
    out = tmp_path / "out"
    calls = []

    def fake_run(**kwargs):
        calls.append((kwargs["sim"], kwargs["task_id"]))
        row = {
            "suite": campaign.run_codex_task.shared.SUITE,
            "task": kwargs["task_id"], "sim": kwargs["sim"],
            "condition": campaign.run_codex_task.CONDITION,
            "agent": {"model": kwargs["model"]},
            "protocol": {"id": campaign.run_codex_task.shared.PROTOCOL_ID},
            "outcome": "FAIL",
        }
        cell = kwargs["out_dir"]
        cell.mkdir(parents=True)
        (cell / "rows.jsonl").write_text(json.dumps(row) + "\n",
                                           encoding="utf-8")
        return row

    monkeypatch.setattr(campaign.run_codex_task, "run_task", fake_run)
    monkeypatch.setattr(campaign.run_codex_task, "codex_identity",
                        lambda _exe: {"path": "codex", "version": "test"})
    monkeypatch.setattr(campaign.run_codex_task.shutil, "which",
                        lambda _exe: "codex")
    monkeypatch.setattr(campaign.frontier, "publication_ready",
                        lambda _task, _sim: (True, []))
    first = campaign.run_campaign(
        sim_ids=("omnisim",), task_ids=("B1_overlap_audit",),
        model="gpt-test", root=root, out_dir=out)
    second = campaign.run_campaign(
        sim_ids=("omnisim",), task_ids=("B1_overlap_audit",),
        model="gpt-test", root=root, out_dir=out)
    assert calls == [("omnisim", "B1_overlap_audit")]
    assert first["rows"] == second["rows"] == 1


def test_campaign_refuses_to_relabel_an_existing_run(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "campaign.json").write_text("{}", encoding="utf-8")
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(campaign.run_codex_task, "codex_identity",
                   lambda _exe: {"path": "codex", "version": "test"})
        mp.setattr(campaign.run_codex_task.shutil, "which",
                   lambda _exe: "codex")
        with pytest.raises(RuntimeError, match="configuration differs"):
            campaign.run_campaign(
                sim_ids=("omnisim",), task_ids=("B1_overlap_audit",),
                model="gpt-test", root=tmp_path / "root", out_dir=out)
