# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at https://www.apache.org/licenses/LICENSE-2.0
"""Engine-free tests of evidence integrity and cinematic renderer routing."""
import contextlib
import io
import json
import sys
from pathlib import Path
from unittest import mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from cinema import cli, replay
from cinema.replay_record import PoseRecorder

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


@pytest.fixture
def production(tmp_path):
    for name in ("scene.blend", "world.omniworld", "controller.py", "result.json"):
        (tmp_path / name).write_text("test input", encoding="utf-8")
    replay.write_json(tmp_path / "run.log.newton.json",
                      {"backend": "newton", "finalised": True, "degraded": False})
    with (tmp_path / "motion.jsonl").open("w", encoding="utf-8") as stream:
        for t in (0, 0.5, 1):
            pose = list(IDENTITY)
            pose[3] = t
            stream.write(json.dumps({"time": t, "poses": {"body": pose}}) + "\n")
    data = replay.template("Test action", "world.omniworld", "test")
    data["fps"] = 4
    data["resolution"] = [64, 36]
    data["scenario"] = {"objective": "Move to the target", "challenge": "Cross the gap",
                        "outcome": "Reached the target", "limitations": []}
    data["shots"][0]["duration_s"] = 1
    path = tmp_path / "film.json"
    replay.write_json(path, data)
    return path


def edit(path, change):
    data = json.loads(path.read_text())
    change(data)
    replay.write_json(path, data)


def fake_proxy(data):
    folder = replay.job_directory(data, "proxy")
    folder.mkdir(parents=True)
    video = folder / "action.mp4"
    video.write_bytes(b"test media receipt fixture")
    report = folder / "bake_report.json"
    replay.write_json(report, {"checks": 4, "max_position_error_m": 0})
    replay.write_json(folder / "receipt.json", {
        "input_key": data["input_key"], "profile": "proxy",
        "shots": [{"id": "action", "video": str(video),
                   "video_sha256": replay.digest(video), "bake_report": str(report),
                   "bake_report_sha256": replay.digest(report)}]})
    return folder


def test_new_defaults_to_replay_and_native_requires_explicit_choice():
    for args, renderer in ((["new"], replay.RENDERER), (["new", "--renderer", "native"], "native")):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            assert cli.main(args) == 0
        assert json.loads(output.getvalue())["renderer"] == renderer


def test_default_render_routes_to_replay_without_contacting_capture(production):
    with mock.patch.object(replay, "render", return_value={}) as render, \
            mock.patch.object(cli.director, "direct") as native:
        assert cli.main(["render", str(production)]) == 0
    render.assert_called_once()
    native.assert_not_called()


def test_old_native_storyboard_does_not_silently_fall_back(production):
    replay.write_json(production, cli.storyboard.template())
    data = json.loads(production.read_text())
    del data["renderer"]
    replay.write_json(production, data)
    with mock.patch.object(cli.director, "direct") as native, pytest.raises(SystemExit) as error:
        cli.main(["render", str(production)])
    assert error.value.code == 2
    native.assert_not_called()


def test_portable_paths_and_source_hash_invalidation(production, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path.parent)
    before = replay.validate(production)
    assert before["recordings"][0]["path"] == str(tmp_path / "motion.jsonl")
    assert before["output"] == str(tmp_path / "renders")
    (tmp_path / "controller.py").write_text("changed controller")
    assert replay.validate(production)["input_key"] != before["input_key"]


@pytest.mark.parametrize("change,match", [
    (lambda d: d["shots"][0].update(source_start_s=0.5), "source range"),
    (lambda d: d["shots"][0].update(speed=2), "source range"),
    (lambda d: d["shots"][0].update(duration_s=0.3), "exact number"),
    (lambda d: d["shots"][0].update(id="../escape"), "Shot ids"),
    (lambda d: d["recordings"][0]["bindings"].update(missing="Other"), "missing/invalid"),
    (lambda d: d["recordings"][0]["bindings"].update(other="test/body"), "multiple"),
    (lambda d: d["scenario"].update(outcome="TODO"), "scenario.outcome"),
    (lambda d: d["shots"][0].update(speed=float("inf")), "finite"),
])
def test_invalid_inputs_fail_before_render(production, change, match):
    data = json.loads(production.read_text())
    change(data)
    # JSON with infinity is deliberately malformed input for the reader.
    production.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        replay.validate(production)


def test_sidecar_initialization_is_required_but_result_can_be_failure(production):
    (production.parent / "result.json").write_text('{"success": false}')
    replay.validate(production)
    replay.write_json(production.parent / "run.log.newton.json",
                      {"backend": "newton", "finalised": False, "degraded": False})
    with pytest.raises(ValueError, match="did not finalise"):
        replay.validate(production)


@pytest.mark.parametrize("poses,times", [
    ([IDENTITY, IDENTITY], [1, 0]),
    ([IDENTITY, IDENTITY], [0, 0]),
    ([IDENTITY, [2] + IDENTITY[1:]], [0, 1]),
    ([IDENTITY, [-1] + IDENTITY[1:]], [0, 1]),
    ([IDENTITY, IDENTITY[:12] + [1, 0, 0, 1]], [0, 1]),
])
def test_invalid_pose_data_is_rejected(tmp_path, poses, times):
    path = tmp_path / "motion.jsonl"
    path.write_text("\n".join(json.dumps({"time": t, "poses": {"body": p}})
                                for t, p in zip(times, poses)))
    with pytest.raises(ValueError):
        replay.read_trace(path, ["body"])


def test_review_requires_complete_current_proxy(production):
    with pytest.raises(ValueError, match="Render proxy first"):
        replay.record_review(production, "Reviewed movement")
    data = replay.validate(production)
    folder = fake_proxy(data)
    review = replay.record_review(production, "Observed the action and inspected contacts")
    assert review["input_key"] == data["input_key"]
    (folder / "action.mp4").write_bytes(b"altered")
    with pytest.raises(ValueError, match="artifact changed"):
        replay.record_review(production, "Review cannot bless a replaced video")


def test_final_requires_review_and_changed_proxy_invalidates_it(production):
    data = replay.validate(production)
    folder = fake_proxy(data)
    with pytest.raises(ValueError, match="Inspect the proxy"):
        replay.render(production, profile="final")
    replay.record_review(production, "Checked visible action")
    receipt = json.loads((folder / "receipt.json").read_text())
    receipt["new_revision"] = True
    replay.write_json(folder / "receipt.json", receipt)
    with pytest.raises(ValueError, match="stale"):
        replay.render(production, profile="final")


def test_recorder_never_commands_nodes_or_overwrites_a_take(tmp_path):
    node = mock.Mock()
    node.getPose.return_value = IDENTITY
    path = tmp_path / "motion.jsonl"
    with PoseRecorder(path, {"body": node}) as recorder:
        recorder.sample(0)
        recorder.sample(0.008)
        with pytest.raises(ValueError, match="new recording"):
            recorder.sample(0)
    assert node.mock_calls == [mock.call.getPose(), mock.call.getPose()]
    assert len(replay.read_trace(path, ["body"])) == 2
    with pytest.raises(FileExistsError):
        PoseRecorder(path, {"body": node})
