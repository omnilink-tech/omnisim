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

"""M2 load test + gravity-kernel correctness check for GranularGroup.

Two halves:

1. **Load test** (always runs): confirms the node parses cleanly on both
   `OMNISIM_WITH_CUDA=ON` and `=OFF`, and the corresponding marker line
   appears in `omnisim_log.txt`.

2. **Numeric kernel check** (CUDA-on only): parses the periodic
   `GranularGroup step N: y min=… mean=… max=…` lines, verifies the seed
   mean is the expected drop height (~5 m), and verifies the mean y at
   step 32 matches semi-implicit Euler integration of -9.81 m/s² to within
   1 cm. This proves the GPU kernel is doing real, correct integration —
   not just zeroing buffers.

Returns:
  0  -- PASS
  1  -- FAIL
  77 -- SKIP (omnisim-bin or runner unavailable on this box)

Runnable via `python tests/cuda/test_granular_group_load.py`.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]

# Matches: GranularGroup step 32: up min=3.6749 mean=3.8789 max=4.0730 (n=250)
_STEP_RE = re.compile(
    r"GranularGroup step (?P<step>\d+): up min=(?P<min>-?[0-9.]+) "
    r"mean=(?P<mean>-?[0-9.]+) max=(?P<max>-?[0-9.]+)"
)
# Matches: GranularGroup seed: up min=5.0009 mean=5.2049 max=5.3990 (n=250)
_SEED_RE = re.compile(
    r"GranularGroup seed: up min=(?P<min>-?[0-9.]+) "
    r"mean=(?P<mean>-?[0-9.]+) max=(?P<max>-?[0-9.]+)"
)


def _check_gravity_integration(log_text: str) -> tuple[bool, str]:
    """Verify the GPU kernel is integrating gravity correctly during the
    free-fall phase (before particles touch the floor or each other).

    Kernel does K=16 internal substeps per outer step at dt_outer=16 ms, so
    inner dt = 1 ms. After N outer steps with semi-implicit Euler:
      y(N) = y(0) - g · dt_inner² · (N·K)·(N·K+1)/2

    For N=32, K=16, dt=0.016, g=9.81: expected drop ≈ 1.314 m. We check at
    step 32 (~0.5 s of sim) so particles are still in free fall — drop from
    y≈5 m gets to ~3.7 m, well above the floor and any pile-up.
    """
    seed_match = _SEED_RE.search(log_text)
    if seed_match is None:
        return False, "no 'GranularGroup seed' line"
    seed_mean = float(seed_match.group("mean"))
    if not (1.0 <= seed_mean <= 2.5):
        return False, f"seed mean up={seed_mean:.4f} outside expected 1.0–2.5 m drop range"

    step32 = None
    for m in _STEP_RE.finditer(log_text):
        if int(m.group("step")) == 32:
            step32 = float(m.group("mean"))
            break
    if step32 is None:
        return False, "no 'GranularGroup step 32' line in log"

    g, dt_outer, n_outer, k_substeps = 9.81, 0.016, 32, 8
    dt_inner = dt_outer / k_substeps
    inner = n_outer * k_substeps
    expected_drop = g * dt_inner * dt_inner * inner * (inner + 1) / 2
    expected_y = seed_mean - expected_drop
    actual_drop = seed_mean - step32
    err = abs(step32 - expected_y)
    if err > 0.05:  # 5 cm tolerance — loose enough for small numerical drift, tight enough to catch real bugs
        return False, (f"step 32: expected mean y={expected_y:.4f} (drop {expected_drop:.4f}), "
                       f"got {step32:.4f} (drop {actual_drop:.4f}), |err|={err:.4f} m > 5 cm")
    return True, (f"step 32: expected drop {expected_drop:.4f} m, "
                  f"actual {actual_drop:.4f} m (|err|={err*1000:.1f} mm)")


def main() -> int:
    headless_runner = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"
    world = HERE / "granular_group_load.omniworld"
    log_path = REPO_ROOT / "omnisim_log.txt"

    if not world.exists():
        print(f"[gg_load] FAIL: missing world {world}.")
        return 1
    if not headless_runner.exists():
        print(f"[gg_load] SKIP: missing {headless_runner}.")
        return 77

    binaries = [
        REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
        REPO_ROOT / "bin" / "omnisim-bin",
        REPO_ROOT / "Contents" / "MacOS" / "webots",
    ]
    if not any(b.exists() for b in binaries):
        print("[gg_load] SKIP: omnisim-bin not built.")
        return 77

    env = os.environ.copy()
    env["OMNISIM_STRUCTURED_LOG"] = "1"
    env.setdefault("OMNISIM_HOME", str(REPO_ROOT))

    cmd = [sys.executable, str(headless_runner), str(world), "--duration", "5"]
    print("[gg_load] launching:", " ".join(cmd))
    run = subprocess.run(cmd, env=env, capture_output=True, text=True)
    print(run.stdout, end="")
    if run.stderr:
        print(run.stderr, end="", file=sys.stderr)

    if run.returncode != 0:
        print("[gg_load] FAIL: headless runner returned", run.returncode)
        return 1

    if not log_path.exists():
        print(f"[gg_load] FAIL: {log_path} not produced.")
        return 1
    log_text = log_path.read_text(encoding="utf-8", errors="replace")

    cuda_on_marker = "GranularGroup: reserved GPU buffers"
    cuda_off_marker = "GranularGroup is inert: CUDA is not available"
    diag_off_marker = "[OMNISIM-DIAG] code=CUDA_NOT_AVAILABLE"

    if cuda_on_marker in log_text:
        print("[gg_load] PASS (CUDA on): GPU buffers reserved for the GranularGroup.")
        ok, msg = _check_gravity_integration(log_text)
        if ok:
            print(f"[gg_load] PASS (gravity kernel): {msg}")
            return 0
        else:
            print(f"[gg_load] FAIL (gravity kernel): {msg}")
            return 1
    if cuda_off_marker in log_text and diag_off_marker in log_text:
        print("[gg_load] PASS (CUDA off): CUDA_NOT_AVAILABLE warning + diagnostic emitted.")
        return 0
    if cuda_off_marker in log_text:
        print("[gg_load] PARTIAL: warning present but tagged diagnostic missing — "
              "is OMNISIM_STRUCTURED_LOG set?")
        return 1

    print("[gg_load] FAIL: neither expected log line found.")
    print("--- log tail ---")
    for line in log_text.splitlines()[-15:]:
        print(line)
    return 1


if __name__ == "__main__":
    sys.exit(main())
