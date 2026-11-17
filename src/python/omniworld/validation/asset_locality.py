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

"""Asset-locality validator.

Per the asset-pipeline rules, a generated world must not reference remote
URLs. Local PROTOs are fine; so is the OmniSim-relative scheme
``omnisim://`` which the simulator resolves inside its own distribution.

The check is a cheap line scan — a world that pulls an ``http(s)://`` or
``ftp://`` reference is a bug, no matter where it appears.
"""

from __future__ import annotations

import re
from pathlib import Path

_REMOTE_RE = re.compile(r"\b(?:https?|ftp)://", re.IGNORECASE)

# The longest scheme the pattern accepts (``https``) precedes the ``://``.
_SCHEME_MAX = 5


def _has_remote_ref(text: str) -> bool:
    """Would ``_REMOTE_RE`` match anywhere in ``text``? Same answer, via the
    ``://`` occurrences (``str.find`` is a C scan) instead of a full search."""
    q = text.find("://")
    while q >= 0:
        if _REMOTE_RE.search(text, max(0, q - _SCHEME_MAX), q + 3) is not None:
            return True
        q = text.find("://", q + 3)
    return False


def check_asset_locality(path: Path, text: str):
    from . import CheckResult  # local import breaks the top-level cycle

    del path  # unused; kept in signature for uniformity with other checks
    # The verdict is decided without splitting the text into lines (0.3 s on
    # an 8 MB generated world, and a whole-text ``_REMOTE_RE.search`` costs
    # the same: ``\b`` plus the alternation is tried at every position).
    # Every match contains the literal ``://``, so only the windows around
    # those occurrences can match; ``search(pos, endpos)`` keeps ``\b``
    # looking at the real preceding character. The per-line pass below
    # then runs only to NUMBER the hits, so the messages are unchanged.
    if not _has_remote_ref(text):
        return CheckResult(name="asset_locality", passed=True)
    hits: list[str] = []
    for i, line in enumerate(text.splitlines(), start=1):
        if _REMOTE_RE.search(line):
            hits.append(f"line {i}: {line.strip()}")
    if hits:
        detail = "remote asset reference(s) found: " + "; ".join(hits[:3])
        if len(hits) > 3:
            detail += f" (+{len(hits) - 3} more)"
        return CheckResult(name="asset_locality", passed=False, detail=detail)
    return CheckResult(name="asset_locality", passed=True)
