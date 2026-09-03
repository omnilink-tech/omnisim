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

"""Every diagnostic code carries a concrete next-action hint.

AGENTS.md §5 tells agents to branch on `diagnostics[].code` and the harness
attaches `hint` to every diagnostic it classifies. Until 2026-09-02, 31 of the
51 classifier codes had no hint at all -- every WORLD_PARSE_*, every CUDA_*,
PROTO_NAME_MISMATCH, TEXTURE_READ_FAILED, CONTROLLER_CRASHED... -- so the
agent got a code and no next call. This pins the vocabulary closed: a code
added without a hint fails here.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

import diagnostic_codes as dc  # noqa: E402
import omnisim_harness as h  # noqa: E402

ALL_CODES = dc.all_codes()


def test_vocabulary_is_the_documented_size():
    # 40 rule codes + 9 CUDA + NEWTON_ZERO_DYNAMIC_BODIES + UNKNOWN + 5 harness-synthesized.
    rule_codes = {r[2] for r in dc._RULES}
    assert len(rule_codes) == 40
    assert len(dc.CUDA_CODES) == 9
    assert set(ALL_CODES) == rule_codes | set(dc.CUDA_CODES) | set(dc.SYNTHESIZED_CODES) \
        | set(dc.HARNESS_SYNTHESIZED_CODES) | {"UNKNOWN"}
    assert len(ALL_CODES) == 56


def test_harness_synthesized_codes_agree_with_the_harness():
    assert set(dc.HARNESS_SYNTHESIZED_CODES) == set(h.HARNESS_DIAGNOSTIC_CODES)
    # ...and the harness's own enumeration is exactly this vocabulary.
    assert set(h.known_diagnostic_codes()) == set(ALL_CODES)


@pytest.mark.parametrize("code", ALL_CODES)
def test_every_code_has_a_concrete_hint(code):
    hint = dc.hint_for(code)
    assert isinstance(hint, str) and hint.strip(), f"{code} has no hint"
    assert len(hint) >= 60, f"{code}: a hint is a next action, not a label: {hint!r}"
    # A next action names something to run, read, set, or change.
    assert re.search(r"(python |GET /|POST /|`|reload|run |read |check|set |unset|rename|"
                     r"add |install|fix|replace|remove|delete|give |reduce|update|copy|"
                     r"start |retry|not fixable|nothing to fix|author |compare |break |use )",
                     hint, re.IGNORECASE), \
        f"{code}: hint names no action: {hint!r}"


def test_no_hint_names_a_stale_fact():
    for code, hint in dc.HINTS.items():
        assert "OMNISIM_ALLOW_ODE_FALLBACK=1" not in hint, code
        assert "actuation is unimplemented" not in hint, code  # Ball/Hinge2 work since 2026-08-17


def test_unknown_lines_carry_the_unknown_hint():
    diag = dc.classify_line("WARNING: a line nobody wrote a rule for")
    assert diag["code"] == "UNKNOWN"
    assert diag["hint"] == dc.HINTS["UNKNOWN"]
    assert "raw" in diag


def test_classified_lines_carry_their_hint():
    cases = {
        "ERROR: 'Foo' PROTO identifier does not match filename": "PROTO_NAME_MISMATCH",
        "WARNING: Texture file could not be read: '/x/tex.png'": "TEXTURE_READ_FAILED",
        "WARNING: 'my_ctrl' controller crashed.": "CONTROLLER_CRASHED",
        "WARNING: 'my_ctrl' controller exited with status: 1.": "CONTROLLER_EXITED_NONZERO",
        "ERROR: '/w/a.omniworld': Failed to load due to syntax error(s).": "WORLD_PARSE_SYNTAX_ERROR",
        "ERROR: Error downloading EXTERNPROTO 'Foo': 404": "EXTERNPROTO_DOWNLOAD_FAILED",
        "[OMNISIM-DIAG] code=CUDA_OUT_OF_MEMORY severity=error message=oom": "CUDA_OUT_OF_MEMORY",
    }
    for line, code in cases.items():
        diag = dc.classify_line(line)
        assert diag["code"] == code, (line, diag)
        assert diag["hint"] == dc.HINTS[code]


def test_hint_for_unknown_code_is_none():
    assert dc.hint_for("NOT_A_CODE") is None
