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

"""`docs/reference/environment-variables.md` must match the source tree.

That page is GENERATED from every `OMNISIM_*` read in the tracked tree by
`scripts/dev/gen_env_reference.py`. AGENTS.md tells agents to read it before
inventing a new hatch, and calls it the list of "every `OMNISIM_*` variable".

THIS TEST IS DELIBERATELY THE SECOND ONE. `docs/tests/test_env_reference.py`
already pins the page and is the more thorough of the two -- it also checks that
every variable carries a description at its read site. It is NOT redundant with
this file; it is better. But it runs under `make tests-docs`, a target separate
from the `make tests-unit` that AGENTS.md sends agents to, and it needs
pytest+Pillow that the Makefile itself notes may be absent on a working clone.

So the thorough gate existed and nobody ran it. On 2026-09-11 it was RED --
`OMNISIM_NEWTON_VELOCITY_GAIN_CLAMP`, introduced by `69b4b024b` that same day,
had no description at its read site -- and the page had drifted far enough to
still flag `OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO` as undocumented hours after the
same commit documented it. A gate in a lane nobody runs is a gate that fails
quietly, which is the defect this file exists to prevent.

An agent that reads a stale page concludes a hatch does not exist and invents a
second one for the same job. This test repeats only the cheap drift check, with
no extra dependencies, in the lane that actually runs. If you are adding
coverage about env vars, add it to the docs test, not here.

⚠️ If this fails, do NOT hand-edit the page -- it is generated, and an edit will
be overwritten. Run:

    python scripts/dev/gen_env_reference.py

Run with:
    pytest tests/test_env_reference_current.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR = REPO_ROOT / "scripts" / "dev" / "gen_env_reference.py"
REFERENCE = REPO_ROOT / "docs" / "reference" / "environment-variables.md"


def test_generator_and_reference_page_both_exist() -> None:
    """The negative control: this test is meaningless if either file moved."""
    assert GENERATOR.is_file(), f"missing generator: {GENERATOR}"
    assert REFERENCE.is_file(), f"missing reference page: {REFERENCE}"


def test_environment_variable_reference_is_not_stale() -> None:
    """`--check` exits 1 when the committed page differs from a fresh run."""
    proc = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
    )

    if proc.returncode != 0:
        pytest.fail(
            "docs/reference/environment-variables.md is out of date with the "
            "source tree.\n\n"
            "Regenerate it (do NOT hand-edit -- the page is generated):\n"
            "    python scripts/dev/gen_env_reference.py\n\n"
            f"generator exit code: {proc.returncode}\n"
            f"stdout: {proc.stdout.strip()}\n"
            f"stderr: {proc.stderr.strip()}"
        )
