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

"""Drift guard: ``projects/policies/research/backends/g1_deploy_runtime.py`` is a VERBATIM
extract of the embedded ``kNewtonRuntimeSource`` Python block inside
``src/omnisim/physics/WbNewtonBackend.cpp`` (the deploy's physics runtime). The
G1 golden-parity test builds the LITERAL deploy model from that extract, so the
proof is only valid while the copy is byte-faithful to the deploy.

This test re-reads the R"PY(...)PY" block out of the .cpp and asserts it still
equals the body of the extracted module (normalized for trailing whitespace). It
is PURE stdlib -- no warp / newton / GPU -- so it runs in bare CI.

Run with:
    python -m pytest tests/test_g1_deploy_runtime_sync.py -v
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
MODULE = REPO / "projects" / "rl" / "backends" / "g1_deploy_runtime.py"
CPP = REPO / "src" / "omnisim" / "physics" / "WbNewtonBackend.cpp"


def _load_module():
    """Import g1_deploy_runtime by path WITHOUT importing warp/newton.

    The module body is the verbatim deploy source (``import warp``,
    ``import newton`` at the top), so a normal import would require those. We
    only need its sync-guard helpers, which are appended AFTER the verbatim block
    and are pure-stdlib. We therefore exec only the guard section.
    """
    src = MODULE.read_text(encoding="utf-8")
    guard_mark = "# " + "=" * 75 + "\n# Sync guard"
    idx = src.index(guard_mark)
    guard_src = src[idx:]
    ns: dict = {"__file__": str(MODULE)}
    exec(compile(guard_src, str(MODULE), "exec"), ns)
    return ns


def test_deploy_runtime_module_exists():
    assert MODULE.is_file(), f"missing extract: {MODULE}"


@pytest.mark.skipif(not CPP.is_file(),
                    reason="WbNewtonBackend.cpp not present (source-only checkout)")
def test_deploy_runtime_in_sync_with_cpp():
    """The extract must be byte-faithful (normalized) to the .cpp block."""
    ns = _load_module()
    # check_in_sync raises AssertionError with a precise diff on drift.
    assert ns["check_in_sync"]() is True


@pytest.mark.skipif(not CPP.is_file(),
                    reason="WbNewtonBackend.cpp not present (source-only checkout)")
def test_block_starts_and_ends_as_expected():
    ns = _load_module()
    block = ns["deploy_runtime_source_from_cpp"]()
    lines = block.splitlines()
    assert lines[0] == "import warp as wp"
    assert lines[1] == "import newton"
    assert "class World:" in block
    # the deploy reads the OMNISIM_NEWTON_* knobs the golden test exercises.
    for knob in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_TARGET_KE",
                 "OMNISIM_NEWTON_TARGET_KD", "OMNISIM_NEWTON_FORCE_MUJOCO",
                 "OMNISIM_NEWTON_CONTACT_KE"):
        assert knob in block, f"deploy runtime no longer reads {knob}"
