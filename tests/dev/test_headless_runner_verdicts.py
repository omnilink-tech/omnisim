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

"""Pure-function pins on the headless runner's verdict logic.

`scripts/dev/headless_runner.py` is imported as a module (its directory goes
on sys.path); only the four pure functions are exercised, with synthetic log
text in the engine's actual wording:

* `sim_never_stepped`     -- finalize with no step line and no sidecar is the
                             stale-libController hang (commit 6eea9d76), and
                             must read True; finalize + a step line reads False.
* `controller_start_failures` -- "Starting controller" followed by the
                             engine's "exited with status: 1" / IPC lines.
* `step_wait_budget_s` / `is_gpu_device` -- the first-step wait is 180 s on a
                             GPU device or a warp mention in the log, 10 s on
                             the CPU, and an unreadable sidecar (None) never
                             buys the GPU budget on its own.

No engine is launched; nothing here touches a binary.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

from headless_runner import (  # noqa: E402
    STEP_WAIT_CPU_S,
    STEP_WAIT_GPU_S,
    controller_start_failures,
    is_gpu_device,
    sim_never_stepped,
    step_wait_budget_s,
)

# The engine's own line shapes (OmNewtonBackend.cpp finalize + step counter).
_FINALISED = "INFO: [OmNewtonBackend] world finalised (solver=mujoco, bodies=3, joints=2)\n"
_STEP = "INFO: [OmNewtonBackend] step 1 dt=0.008 b0=(0,0,0.5) b1=(1,0,0.5)\n"
_HEADER = "=== OmniSim Log Started: 2026-09-02 10:00:00 ===\n"


# --- sim_never_stepped --------------------------------------------------------

def test_finalized_without_a_step_and_without_a_sidecar_is_never_stepped():
    assert sim_never_stepped(_HEADER + _FINALISED, sidecar_finalised=False) is True


def test_finalized_via_sidecar_without_a_step_is_never_stepped():
    # The sidecar alone proves finalize even when the log line is absent.
    assert sim_never_stepped(_HEADER, sidecar_finalised=True) is True


def test_finalized_and_stepped_is_not_flagged():
    assert sim_never_stepped(_HEADER + _FINALISED + _STEP, sidecar_finalised=False) is False
    assert sim_never_stepped(_HEADER + _FINALISED + _STEP, sidecar_finalised=True) is False


def test_no_finalize_evidence_never_guesses():
    # A cold load that never reached finalize is NOT a hang verdict.
    assert sim_never_stepped(_HEADER, sidecar_finalised=False) is False
    assert sim_never_stepped("", sidecar_finalised=False) is False


# --- controller_start_failures -------------------------------------------

def test_exited_with_status_after_starting_controller_is_a_start_failure():
    log = (_HEADER
           + "INFO: Starting controller: python.exe -u husky_random.py\n"
           + "WARNING: 'husky_random' controller exited with status: 1.\n")
    failures = controller_start_failures(log)
    assert failures == ["WARNING: 'husky_random' controller exited with status: 1."]


def test_ipc_handshake_and_pairing_lines_are_start_failures():
    log = (_HEADER
           + "INFO: Starting controller: python.exe -u g1_walk.py\n"
           + "ERROR: 'g1_walk' controller: libController did not complete the OmniSim IPC "
             "handshake within 5 seconds: it predates the handshake protocol\n"
           + "ERROR: 'g1_walk' controller never paired with the simulator (zero ticks)\n")
    failures = controller_start_failures(log)
    assert len(failures) == 2
    assert any("OmniSim IPC handshake" in f for f in failures)
    assert any("never paired with the simulator" in f for f in failures)


def test_clean_start_has_no_failures_and_status_zero_is_not_one():
    log = (_HEADER
           + "INFO: Starting controller: python.exe -u husky_random.py\n"
           + "INFO: 'husky_random' controller exited successfully.\n"
           + "INFO: 'husky_random' controller exited with status: 0.\n")
    assert controller_start_failures(log) == []


def test_duplicate_failure_lines_are_reported_once():
    line = "WARNING: 'ctl' controller exited with status: 2."
    assert controller_start_failures(line + "\n" + line + "\n") == [line]


# --- step_wait_budget_s / is_gpu_device ----------------------------------

def test_gpu_device_gets_the_long_budget():
    assert step_wait_budget_s("cuda:0") == 180 == STEP_WAIT_GPU_S
    assert step_wait_budget_s("cuda") == 180


def test_cpu_device_gets_the_short_budget():
    assert step_wait_budget_s("cpu") == 10 == STEP_WAIT_CPU_S
    # An explicit CPU sidecar wins over a warp mention in the log.
    assert step_wait_budget_s("cpu", warp_in_log=True) == 10


def test_unreadable_sidecar_with_warp_in_log_gets_the_long_budget():
    assert step_wait_budget_s(None, warp_in_log=True) == 180
    assert step_wait_budget_s(None, warp_in_log=False) == 10


def test_explicit_override_wins():
    assert step_wait_budget_s("cuda:0", override=0) == 0.0
    assert step_wait_budget_s("cpu", override=42.5) == 42.5
    assert step_wait_budget_s("cpu", override=-1) == 10  # negative = not an override


def test_is_gpu_device_never_treats_unknown_as_gpu():
    assert is_gpu_device("cuda:0") is True
    assert is_gpu_device("CPU") is False
    assert is_gpu_device("cpu") is False
    assert is_gpu_device(None) is False
    assert is_gpu_device("") is False
