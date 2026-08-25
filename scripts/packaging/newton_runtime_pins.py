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

"""Single source of truth for the Newton physics-runtime package versions.

Why this file exists
--------------------
OmniSim's headline correctness claim is **bit-exact train == deploy**: the local
GPU trainer and the local OmniSim-Newton deploy runtime run the *same* physics
(docs/developer/g1-single-source-of-truth.md). That only holds if both sides run
the *same versions* of the physics packages they share.

They used to be declared in two places that could drift apart silently:
  - the local deploy bundle (scripts/packaging/bundle_newton_runtime.py) used to
    `pip install --upgrade` the packages **by name**, i.e. it vendored whatever
    was newest on PyPI at bundle time;
  - the standalone GPU trainer (projects/policies/research/training/requirements-train.txt) pins **exact versions**.
A newer Warp or MuJoCo-Warp landing on PyPI would silently desync the deploy
bundle from the trainer the next time anyone re-ran the bundler — breaking
bit-exactness with zero signal.

This module is the one place those versions live. The deploy bundler vendors
exactly these; tests/test_newton_pins_parity.py fails CI if the trainer
requirements ever disagree on the SHARED stack.

The parity contract — who runs what
------------------------------------
The two sides do NOT install identical package sets, so parity is scoped to the
packages that genuinely run on both:

  SHARED_STACK  — imported by BOTH the trainer and the deploy runtime, and
                  determinism-critical. These MUST match across the two sides;
                  they are the parity-checked set.
                    warp-lang    — the GPU kernel runtime + CUDA-graph capture
                    mujoco-warp  — the batched-GPU MuJoCo solver both sides step
                  (The trainer imports `mujoco_warp` directly; the deploy runtime
                  reaches the same solver through Newton's SolverMuJoCo. Same
                  solver, same version → same physics.)

  DEPLOY_STACK  — installed ONLY in the deploy bundle. Determinism-relevant on
                  the deploy side but with no trainer counterpart to match, so
                  pinned for reproducibility, not parity-checked.
                    newton    — the rigid-body / SolverMuJoCo wrapper over warp
                                (the trainer is pure-Python mujoco_warp, no Newton)
                    usd-core  — Newton's USD/asset loader (imported as `pxr`)

To bump the runtime: change it here, re-pin projects/policies/research/training/requirements-train.txt to match
the SHARED_STACK (the test tells you if you forgot), and re-run the bundler.
"""
from __future__ import annotations

# Present on BOTH sides and determinism-critical → parity-checked against the
# standalone trainer requirements. Keys are PyPI names.
SHARED_STACK: dict[str, str] = {
    "warp-lang": "1.16.0",
    "mujoco-warp": "3.11.0",
    # mujoco joined the parity-checked set in the 1.2.0 -> 1.5.0 bump. It was
    # previously pinned NOWHERE except docker/Dockerfile.train and arrived
    # transitively from mujoco-warp -- yet both sides step it, and its
    # `mj_fullM` signature CHANGED between 3.8 and 3.9 (verified empirically:
    # the 3.8 form raises AttributeError on 3.11.0). An unpinned physics
    # library shared by trainer and deploy is exactly what this file exists to
    # prevent.
    "mujoco": "3.11.0",
}

# Deploy-bundle-only. Pinned for a reproducible bundle; NOT parity-checked
# (the trainer installs none of these).
DEPLOY_STACK: dict[str, str] = {
    "newton": "1.5.0",
    "usd-core": "26.5",
    # Newton's codeless USD schema plugin (pure data: plugInfo.json + a
    # generatedSchema.usda, ~90 KB installed, zero deps, Apache-2.0). Newton's
    # `add_usd` importer hard-fails without it (newton/_src/usd/__init__.py
    # raises from require_newton_usd_schemas), so a bundle without it cannot
    # import USD assets. Matches newton 1.5.0's `importers` extra
    # (newton-usd-schemas>=0.4.1).
    "newton-usd-schemas": "0.5.0",
}


def bundle_requirements() -> list[str]:
    """Exact `name==version` specs the local Newton runtime bundle vendors."""
    return [f"{n}=={v}" for n, v in {**SHARED_STACK, **DEPLOY_STACK}.items()]


def physics_specs() -> dict[str, str]:
    """The parity-checked shared stack: {pypi_name: version}. These must be
    version-identical in projects/policies/research/training/requirements-train.txt."""
    return dict(SHARED_STACK)


if __name__ == "__main__":
    # `python scripts/packaging/newton_runtime_pins.py` prints the canonical set.
    for spec in bundle_requirements():
        print(spec)
