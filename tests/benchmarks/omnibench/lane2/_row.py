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

"""Lane-2 results-row writer — thin wrapper over common/results.py.

Historically this module duplicated the SPEC row schema (it predated
common/); it now delegates entirely to tests/benchmarks/omnibench/common/
results.py, keeping the public signatures the lane code was verified
against (fingerprint / make_row / write_row / default_out / repo_root).
"""
from __future__ import annotations

import sys
from pathlib import Path

LANE2 = Path(__file__).resolve().parent
OMNIBENCH = LANE2.parent
REPO = LANE2.parents[3]  # tests/benchmarks/omnibench/lane2 -> repo root
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
if str(OMNIBENCH) not in sys.path:
    sys.path.insert(0, str(OMNIBENCH))

from common import results as _results  # noqa: E402


def fingerprint(engine_log_path=None) -> dict:
    """Machine attribution per SPEC (mandatory on every row). Pass
    engine_log_path for tier C so the Newton backend-verdict sidecar of THAT
    run decides `physics`."""
    return _results.machine_fingerprint(engine_log_path=engine_log_path)


def make_row(*, test: str, engine: str, dt_ms: float, metrics: dict,
             wall_ms_per_step: float, steps: int, sim_seconds: float,
             deviations=None, machine=None) -> dict:
    return _results.make_row(
        test=test, engine=engine, dt_ms=dt_ms, metrics=metrics,
        wall_ms_per_step=wall_ms_per_step, steps=steps,
        sim_seconds=sim_seconds, deviations=deviations or [],
        machine=machine)


def write_row(out_path, row: dict) -> None:
    _results.append_row(out_path, row)


def default_out(name: str) -> Path:
    return LANE2 / "results" / name


def repo_root() -> Path:
    return REPO
