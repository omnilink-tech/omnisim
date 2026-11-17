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

"""Static and log-scoring contracts for the packaged Shadowing + BATON block."""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "projects" / "policies" / "skills"))
sys.path.insert(0, str(REPO / "projects" / "policies" / "benchmarks"))

import manifest as M  # noqa: E402
import policy_bench as BENCH  # noqa: E402
import skill_lib as LIB  # noqa: E402


def test_policy_registry_discovers_unique_valid_manifests():
    reg = M.Registry.discover()
    assert not reg.issues
    assert reg.skills and reg.sequences
    for skill in reg.skills.values():
        assert not [i for i in skill.validate() if not i.startswith("warn:")], skill.name
    for sequence in reg.sequences.values():
        assert not [i for i in sequence.validate(reg) if not i.startswith("warn:")], sequence.name


def test_generated_registry_is_exactly_current():
    reg = M.Registry.discover()
    actual = json.loads((M.SKILLS_DIR / "registry.json").read_text(encoding="utf-8"))
    assert actual == LIB._registry_payload(reg)


def test_every_reproduced_demo_matches_its_manifest():
    assert LIB.cmd_verify_demos(M.Registry.discover()) == 0


def test_benchmark_suite_is_a_valid_catalog_contract():
    assert BENCH.validate_suite(M.Registry.discover()) == []


def _write_benchmark_log(path: Path, *, include_arrest: bool = True) -> None:
    lines = [
        "BATON switch #1 (-> stand) at t=100 phase=4.08",
        "deploy t=100 x=1.0 y=0.0 z=0.720 roll=0.01 pit=0.02 cmd=0.0 vx=0.0",
        "PHYS-GRASP -> place_brake",
        "SUCTION PRESS-PLACE released -> course advances",
    ]
    if include_arrest:
        lines.append("PLACE-ARREST released")
    lines.append("BATON-CYCLE k=0 ok=1 segs=14/14 t=9687 dur=155.0 minz=0.720")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_benchmark_scorer_passes_complete_physical_cycle(tmp_path):
    case = BENCH.cases_by_name(BENCH.load_suite())["g1_box_delivery_e2e"]
    log = tmp_path / "run.txt"
    _write_benchmark_log(log)
    result = BENCH.score(case, log)
    assert result["passed"] is True
    assert result["metrics"]["complete_cycles"] == 1
    assert result["metrics"]["fall_ticks"] == 0


def test_benchmark_scorer_fails_closed_on_missing_physical_event(tmp_path):
    case = BENCH.cases_by_name(BENCH.load_suite())["g1_box_delivery_e2e"]
    log = tmp_path / "run.txt"
    _write_benchmark_log(log, include_arrest=False)
    result = BENCH.score(case, log)
    assert result["passed"] is False
    assert any(c["name"].startswith("event:PLACE-ARREST") and not c["passed"]
               for c in result["checks"])


def test_benchmark_scorer_handles_free_standing_quadruped_telemetry(tmp_path):
    case = BENCH.cases_by_name(BENCH.load_suite())["go2_walk_turn_walk_e2e"]
    log = tmp_path / "go2.txt"
    lines = [
        "ONNX loaded: walk.onnx", "ONNX loaded: turn.onnx",
        "BATON switch #1 (-> turn) at t=625 phase=3.27",
    ]
    for t, x, y in ((10, 4.20, -0.07), (11, 4.18, -0.06), (12, 4.19, -0.08),
                    (13, 4.20, -0.10), (14, 4.22, -0.12)):
        lines.append(f"[t={t}s] mode=turn  u=1.00 sw=1 x={x:+.2f} y={y:+.2f} "
                     "bz=0.28 roll=+0.03 vx=+0.01 gm=0.85")
    lines.append("BATON switch #2 (-> walk) at t=938 phase=3.37")
    for t, x in ((15, 4.20), (20, 1.00), (25, -1.20)):
        lines.append(f"[t={t}s] mode=walk  u=1.00 sw=2 x={x:+.2f} y=+0.00 "
                     "bz=0.27 roll=+0.10 vx=-0.40 gm=0.88")
    log.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = BENCH.score(case, log)
    assert result["passed"] is True
    assert result["metrics"]["switches"] == 2
    assert result["metrics"]["post_turn_displacement_m"] >= 5.0
