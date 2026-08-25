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

"""Drift guard: ``projects/policies/research/backends/g1_deploy_runtime.py`` mirrors
the canonical ``src/omnisim/physics/omnisim_newton_runtime.py`` deploy runtime. The
G1 golden-parity test builds the LITERAL deploy model from that extract, so the
proof is only valid while the copy is byte-faithful to the deploy.

This test reads the canonical Python module and asserts it still equals the body
of the policy mirror (normalized for trailing whitespace). It
is PURE stdlib -- no warp / newton / GPU -- so it runs in bare CI.

Run with:
    python -m pytest tests/test_g1_deploy_runtime_sync.py -v
"""
from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
# ⛔ This pointed at `projects/rl/backends/` until 2026-07-13 -- a path deleted by the
# rl -> policies reorg (056b26a8b). The module simply was not there, so all three tests in this
# file had been ERRORING since that commit, and nobody noticed because this test is not in the
# active CI set. A drift guard that cannot find the thing it guards is not guarding anything.
# The correct path is the one this file's own docstring names.
MODULE = REPO / "projects" / "policies" / "research" / "backends" / "g1_deploy_runtime.py"
RUNTIME = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"


def _mirror_body():
    """Read the legacy mirror without importing warp/newton.

    Older mirrors contain the former C++ raw-literal seam; normalize that one
    mechanical artifact while the sync script migrates consumers.
    """
    src = MODULE.read_text(encoding="utf-8")
    guard_mark = "# " + "=" * 75 + "\n# Sync guard"
    first = src.index('"""')
    body = src[src.index('"""', first + 3) + 3:src.index(guard_mark)]
    return body.replace('\n)PY"\nR"PY(', '\n').strip()


def test_deploy_runtime_module_exists():
    assert MODULE.is_file(), f"missing extract: {MODULE}"


@pytest.mark.skipif(not RUNTIME.is_file(),
                    reason="canonical Newton runtime not present")
def test_deploy_runtime_in_sync_with_canonical_module():
    """The policy mirror must remain byte-faithful to the shipped module."""
    assert _mirror_body() == RUNTIME.read_text(encoding="utf-8").strip()


@pytest.mark.skipif(not RUNTIME.is_file(),
                    reason="canonical Newton runtime not present")
def test_block_starts_and_ends_as_expected():
    block = RUNTIME.read_text(encoding="utf-8")
    lines = block.splitlines()
    assert lines[0] == "import warp as wp"
    assert lines[1] == "import newton"
    assert "class World:" in block
    # the deploy reads the OMNISIM_NEWTON_* knobs the golden test exercises.
    for knob in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_TARGET_KE",
                 "OMNISIM_NEWTON_TARGET_KD", "OMNISIM_NEWTON_FORCE_MUJOCO",
                 "OMNISIM_NEWTON_CONTACT_KE"):
        assert knob in block, f"deploy runtime no longer reads {knob}"
