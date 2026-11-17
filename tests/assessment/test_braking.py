"""Independent analytic traces exercise the assessor's validity and failure gates."""
from copy import deepcopy
from dataclasses import replace
import json
import math

import pytest

from omnisim.assessment.schema import BrakingSpec
from omnisim.assessment.oracles import grade


def analytic_case(delay=0.0):
    spec = BrakingSpec(brake_delay_s=delay)
    dt, request, horizon = .016, 2.0, 4.512
    rows = []
    x = 0.0
    for i in range(round(horizon / dt)):
        t = round((i + 1) * dt, 9)
        v = .6 if t <= request + delay else 0.0
        x += v * dt
        rows.append({"seq": i, "t_s": t, "x_m": x, "y_m": 0.0, "z_m": .132,
                     "vx_m_s": v, "vy_m_s": 0.0, "vz_m_s": 0.0, "tilt_rad": 0.0,
                     "contacts": 4, "wheel_position_rad": [x / .1651] * 4,
                     "stop_requested": t >= request, "brake_applied": t >= request + delay,
                     "wheel_command_rad_s": .6 / .1651 if t < request + delay else 0.0})
    complete = {"complete": True, "samples": len(rows), "last_t_s": rows[-1]["t_s"]}
    return spec, rows, complete, {"backend": "newton", "finalised": True, "degraded": False,
                                  "runtime": {"device": "cpu"}}


def test_distance_is_measured_from_request_not_brake_application():
    good = grade(*analytic_case())
    bad = grade(*analytic_case(.8))
    assert good["status"] == "PASS"
    assert bad["status"] == "FAIL"
    assert bad["metrics"]["max_forward_excursion_m"] == pytest.approx(.48)
    assert bad["metrics"]["observed_command_delay_s"] == pytest.approx(.8)
    assert bad["reasons"] == ["stopping_distance_exceeded"]


def test_injected_delay_does_not_determine_grade():
    spec, rows, completion, physics = analytic_case(.8)
    # An oracle that just reads the fault knob would incorrectly PASS this.
    assert grade(replace(spec, brake_delay_s=0), rows, completion, physics)["status"] == "FAIL"


@pytest.mark.parametrize("problem,reason", [
    ("frozen", "insufficient_initial_motion"),
    ("no_physics", "physics_not_proven"),
    ("no_contacts", "no_measured_ground_contacts"),
    ("frozen_wheels", "wheel_rotation_not_proven"),
    ("gap", "trace_gap_or_clock_discontinuity"),
    ("short", "post_request_observation_too_short"),
    ("incomplete", "controller_episode_incomplete"),
    ("nan", "nonfinite_trace"),
    ("truncated_count", "trace_count_mismatch"),
])
def test_insufficient_evidence_cannot_pass(problem, reason):
    spec, rows, completion, physics = deepcopy(analytic_case())
    if problem == "frozen":
        for r in rows: r["x_m"] = 0
    elif problem == "no_physics": physics = {}
    elif problem == "no_contacts":
        for r in rows: r["contacts"] = 0
    elif problem == "frozen_wheels":
        for r in rows: r["wheel_position_rad"] = [0] * 4
    elif problem == "gap": rows.pop(20)
    elif problem == "short":
        rows = rows[:-50]
        completion.update(samples=len(rows), last_t_s=rows[-1]["t_s"])
    elif problem == "incomplete": completion["complete"] = False
    elif problem == "nan": rows[4]["vx_m_s"] = math.nan
    elif problem == "truncated_count": completion["samples"] += 1
    result = grade(spec, rows, completion, physics)
    assert result["status"] == "INCONCLUSIVE"
    assert reason in result["reasons"]


def test_never_settles_fails_even_inside_distance_limit():
    spec, rows, completion, physics = analytic_case()
    for r in rows:
        if r["t_s"] > 2.0:
            r["vx_m_s"] = .03
    result = grade(spec, rows, completion, physics)
    assert result["status"] == "FAIL"
    assert result["metrics"]["stop_time_s"] is None
    assert "stopping_deadline_exceeded" in result["reasons"]


@pytest.mark.parametrize("data", [{"robot": "jackal"}, {"target_speed_m_s": math.nan},
    {"brake_delay_s": -1}, {"target_speed_m_s": True}, {"coulombFriction": .5},
    {"observe_after_request_s": .5}, {"target_speed_m_s": 1.2}])
def test_spec_rejects_unsupported_or_invalid_inputs(data):
    with pytest.raises(ValueError): BrakingSpec.from_dict(data)


def test_offline_regrade_detects_changed_evidence(tmp_path):
    from omnisim.assessment.runner import digest, regrade, save
    spec, rows, completion, physics = analytic_case()
    save(tmp_path / "spec.json", spec.to_dict())
    save(tmp_path / "completion.json", completion)
    save(tmp_path / "engine.log.newton.json", physics)
    (tmp_path / "trace.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    save(tmp_path / "manifest.json", {"engine_exit_code": 0, "artifacts_sha256": {
        "trace.jsonl": digest(tmp_path / "trace.jsonl")}})
    assert regrade(tmp_path)["status"] == "PASS"
    with (tmp_path / "trace.jsonl").open("a") as f: f.write("\n")
    verdict = regrade(tmp_path)
    assert verdict["status"] == "ERROR"
    assert "artifact_hash_mismatch:trace.jsonl" in verdict["reasons"]


def test_rerun_refuses_changed_sources_before_launch(tmp_path, monkeypatch):
    from omnisim.assessment import runner
    runner.save(tmp_path / "manifest.json", {"sources_sha256": {"controller": "old"}})
    monkeypatch.setattr(runner, "source_hashes", lambda *args: {"controller": "new"})
    with pytest.raises(ValueError, match="source hashes differ"):
        runner.rerun(tmp_path, tmp_path / "unused")
    assert not (tmp_path / "unused").exists()
