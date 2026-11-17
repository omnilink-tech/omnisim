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

"""Shared plumbing for the harness test suite.

`harness_supervisor.py` does not import under stock pytest (its
`from omnisim import Supervisor` needs the controller runtime), so tests that
exercise its pure functions exec a textual slice of the source instead —
`supervisor_source()` / `exec_supervisor_slice()` centralise that pattern
(test_mutation_verbs.py grew its own copy before this existed).
"""

from __future__ import annotations

import functools
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SUPERVISOR_DIR = REPO_ROOT / "projects" / "default" / "controllers" / "harness_supervisor"
HARNESS_DIR = REPO_ROOT / "scripts" / "harness"


@functools.lru_cache(maxsize=1)
def supervisor_source() -> str:
    return (SUPERVISOR_DIR / "harness_supervisor.py").read_text(encoding="utf-8")


@functools.lru_cache(maxsize=None)
def _compiled_slice(start_anchor: str, end_anchor: str):
    src = supervisor_source()
    text = src[src.index(start_anchor):src.index(end_anchor)]
    return compile(text, f"harness_supervisor[{start_anchor.strip()!r}]", "exec")


def exec_supervisor_slice(start_anchor: str, end_anchor: str, **ns) -> dict:
    """Exec the source between two anchors into a namespace and return it.

    The compile is cached per anchor pair; the exec is repeated so each caller
    gets fresh function objects bound to its own namespace.
    """
    exec(_compiled_slice(start_anchor, end_anchor), ns)  # noqa: S102
    return ns
