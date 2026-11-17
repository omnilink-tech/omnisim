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

"""A world pinned to newtonSolver "mujoco_warp" runs CPU mj_step when there is no CUDA device.

External report, 2026-08-29 (Windows 11, Intel Arc, no CUDA): warehouse_husky --
the quickstart's first demo -- finalised on the "cpu" device under mujoco_warp and
did not step inside `run-headless --until-finalized`'s budget, because warp
JIT-compiles the mujoco_warp kernels for its CPU device instead of refusing.

Engine-free: the decision is a pure function in the runtime module, extracted by
AST and executed on its own (the module itself imports warp and newton). The
wiring into finalize() is pinned by source checks.
"""

from __future__ import annotations

import ast
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
RUNTIME = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
_SRC = RUNTIME.read_text(encoding="utf-8")


def _load_fn(name: str):
    tree = ast.parse(_SRC, filename=str(RUNTIME))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            ns: dict = {}
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(RUNTIME), "exec"), ns)
            return ns[name]
    raise AssertionError(f"{name} not found at module level in {RUNTIME}")


reason = _load_fn("_mjwarp_cpu_fallback_reason")


def test_no_cuda_device_downgrades_the_world_pin() -> None:
    msg = reason("mujoco_warp", False, 0)
    assert msg and "CPU mj_step" in msg and "OMNISIM_NEWTON_MJWARP=1" in msg


def test_a_cuda_device_keeps_mujoco_warp() -> None:
    assert reason("mujoco_warp", False, 1) is None


def test_explicit_env_request_is_honoured_without_cuda() -> None:
    assert reason("mujoco_warp", True, 0) is None


def test_unknown_cuda_count_changes_nothing() -> None:
    assert reason("mujoco_warp", False, None) is None


def test_other_solver_values_are_untouched() -> None:
    for pref in (None, "", "mujoco", "vbd", "mujoco+vbd"):
        assert reason(pref, False, 0) is None


def test_finalize_decides_before_the_device_pin_and_labels_the_fallback() -> None:
    fin = _SRC[_SRC.index("    def finalize(self):"):]
    decide = fin.index("_mjwarp_cpu_fallback_reason(")
    pin = fin.index('_dev_env = (_devos.environ.get("OMNISIM_NEWTON_MODEL_DEVICE")')
    assert decide < pin, "the fallback must be decided before the model-device pin reads _solver_pref"
    assert 'self._solver_pref = "mujoco"' in fin[decide:pin]
    # The sidecar/log label must say so, and must NOT contain "(mujoco_warp" --
    # OmNewtonBackend.cpp keys mSolverIsMuJoCoWarp on that substring.
    label = "no CUDA device: WorldInfo.newtonSolver mujoco_warp fell back"
    assert label in fin
    assert "(mujoco_warp" not in label
