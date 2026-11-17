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

"""The never-stepped verdict reports the MEASURED first-step wait, not the budget.

External report, 2026-08-29: ``--until-finalized --step-wait-timeout 420`` printed
"the run ended 11.5s after finalize, having waited 420.0s for a first step". The
420.0 was ``step_deadline - completion_at`` -- the argument echoed back. (The
other half of that report, the duration ceiling cutting the wait short, was fixed
2026-09-01 in eaaa13077.)
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNNER = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("headless_runner_wait", _RUNNER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


runner = _load_runner()


def test_reported_wait_is_measured_not_the_budget() -> None:
    # The reported case: finalize at t=100, a 420 s budget, the loop stopped at 111.5.
    line = runner.first_step_wait_summary(100.0, 111.5, 100.0 + 420.0)
    assert "waited 11.5s" in line
    assert "420s budget" in line
    assert "420.0s for a first step" not in line
    assert "BEFORE the budget ran out" in line


def test_full_budget_spent_is_not_flagged_as_short() -> None:
    line = runner.first_step_wait_summary(100.0, 110.2, 110.0)
    assert "waited 10.2s" in line
    assert "10s budget" in line
    assert "BEFORE" not in line


def test_no_wait_armed() -> None:
    line = runner.first_step_wait_summary(100.0, 102.0, None)
    assert "no first-step wait was armed" in line
    assert "2.0s after finalize" in line


def test_explicit_step_wait_timeout_overrides_the_device_budget() -> None:
    assert runner.step_wait_budget_s("cpu", False, 420.0) == 420.0
    assert runner.step_wait_budget_s("cuda:0", True, 0.0) == 0.0
    assert runner.step_wait_budget_s("cpu", False, None) == runner.STEP_WAIT_CPU_S
