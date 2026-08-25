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

"""Guard: the GPU trainer and the local deploy bundle pin the SAME physics stack.

OmniSim claims bit-exact train == deploy (docs/developer/g1-single-source-of-
truth.md). That guarantee is only real if the GPU trainer and the local
OmniSim-Newton deploy runtime run identical versions of Warp / Newton /
MuJoCo-Warp. This test fails if they ever drift.

- The single source of truth is scripts/packaging/newton_runtime_pins.py
  (consumed by the deploy bundler, scripts/packaging/bundle_newton_runtime.py).
- The standalone GPU trainers pin their deps in
  projects/policies/research/training/requirements-train.txt.

If this test fails, you changed one side without the other: reconcile the
version in projects/policies/research/training/requirements-train.txt with
newton_runtime_pins.PHYSICS_STACK (or vice versa), in the same change.
"""
import importlib.util
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PINS_PY = REPO / "scripts" / "packaging" / "newton_runtime_pins.py"
TRAIN_REQS = REPO / "projects" / "policies" / "research" / "training" / "requirements-train.txt"


def _load_pins():
    spec = importlib.util.spec_from_file_location("newton_runtime_pins", PINS_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _parse_requirements(text: str) -> dict[str, str]:
    """{normalized-name: version} for exact `name==version` pins, ignoring
    comments / blank / unpinned lines. Names are normalized (PEP 503: lowercase,
    runs of -_. collapse to -) so `mujoco_warp` and `mujoco-warp` compare equal."""
    pins: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        m = re.fullmatch(r"([A-Za-z0-9._-]+)==([A-Za-z0-9._-]+)", line)
        if m:
            name = re.sub(r"[-_.]+", "-", m.group(1)).lower()
            pins[name] = m.group(2)
    return pins


def _norm(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def test_pins_file_and_train_requirements_exist():
    assert PINS_PY.exists(), f"missing SSOT: {PINS_PY}"
    assert TRAIN_REQS.exists(), f"missing trainer requirements: {TRAIN_REQS}"


def test_physics_stack_versions_match_train_requirements():
    pins = _load_pins()
    canonical = {_norm(n): v for n, v in pins.physics_specs().items()}
    trainer = _parse_requirements(TRAIN_REQS.read_text(encoding="utf-8"))

    mismatches = []
    for name, want in canonical.items():
        got = trainer.get(name)
        if got is None:
            mismatches.append(
                f"{name}: pinned {want} in newton_runtime_pins.py but NOT pinned "
                f"in projects/policies/research/training/requirements-train.txt")
        elif got != want:
            mismatches.append(
                f"{name}: newton_runtime_pins.py says {want}, "
                f"requirements-train.txt says {got}")

    assert not mismatches, (
        "train/deploy physics-stack version drift — bit-exact parity is at risk:\n"
        + "\n".join("  - " + m for m in mismatches))


def test_bundle_requirements_are_all_exact_pins():
    """The deploy bundler must vendor exact versions, never name-only (which
    would resolve to latest-on-PyPI and desync from the trainer)."""
    pins = _load_pins()
    for spec in pins.bundle_requirements():
        assert re.fullmatch(r"[A-Za-z0-9._-]+==[A-Za-z0-9._-]+", spec), (
            f"bundle requirement {spec!r} is not an exact ==version pin")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
