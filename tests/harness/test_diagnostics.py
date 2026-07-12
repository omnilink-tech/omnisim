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

"""Fixture-driven tests for scripts/harness/diagnostic_codes.py.

The mapper is the most load-bearing piece of the M1 harness slice: every
error path that a coding agent might hit needs to land on a stable code,
not a raw English string.

Each rule in the mapper should have at least one positive test here. When
the simulator's user-facing strings change, these tests catch the drift.

Run with:
    pytest tests/harness/test_diagnostics.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

from diagnostic_codes import classify_line, classify_text  # noqa: E402


# ---- Header parsing ----------------------------------------------------------

def test_blank_line_returns_none():
    assert classify_line("") is None
    assert classify_line("   \n") is None


def test_info_line_is_ignored():
    assert classify_line("INFO: 'controller' controller exited successfully.") is None


def test_unprefixed_line_is_ignored():
    assert classify_line("=== OmniSim Log Started: 2026-04-27 14:37:00 ===") is None


def test_unknown_error_line_passes_through():
    diag = classify_line("ERROR: Something nobody planned for.")
    assert diag == {
        "code": "UNKNOWN",
        "severity": "error",
        "message": "Something nobody planned for.",
        "raw": "ERROR: Something nobody planned for.",
    }


def test_severity_is_set_from_header():
    assert classify_line("FATAL: boom.")["severity"] == "fatal"
    assert classify_line("ERROR: boom.")["severity"] == "error"
    assert classify_line("WARNING: boom.")["severity"] == "warning"


# ---- World file gating (WbApplication.cpp) ----------------------------------

def test_world_file_not_found():
    diag = classify_line("ERROR: Could not open file: '/tmp/missing.wbt'.")
    assert diag["code"] == "WORLD_FILE_NOT_FOUND"
    assert diag["source_path"] == "/tmp/missing.wbt"
    assert diag["severity"] == "error"


def test_world_wrong_extension():
    # Order matters: this rule is more specific than WORLD_FILE_NOT_FOUND.
    diag = classify_line("ERROR: Could not open file: '/tmp/x.txt'. The world file extension must be '.wbt'.")
    assert diag["code"] == "WORLD_WRONG_EXTENSION"
    assert diag["source_path"] == "/tmp/x.txt"


def test_world_parse_invalid_tokens():
    diag = classify_line("ERROR: '/tmp/foo.wbt': Failed to load due to invalid token(s).")
    assert diag["code"] == "WORLD_PARSE_INVALID_TOKENS"
    assert diag["source_path"] == "/tmp/foo.wbt"


def test_world_parse_syntax_error():
    diag = classify_line("ERROR: '/tmp/foo.wbt': Failed to load due to syntax error(s).")
    assert diag["code"] == "WORLD_PARSE_SYNTAX_ERROR"
    assert diag["source_path"] == "/tmp/foo.wbt"


# ---- Tokenizer / parser (WbTokenizer.cpp) -----------------------------------

def test_world_file_empty():
    diag = classify_line("ERROR: File is empty: '/tmp/foo.wbt'.")
    assert diag["code"] == "WORLD_FILE_EMPTY"
    assert diag["source_path"] == "/tmp/foo.wbt"


def test_header_missing_error():
    diag = classify_line("ERROR: '/tmp/foo.wbt': error: Missing header.")
    assert diag["code"] == "HEADER_MISSING"
    assert diag["source_path"] == "/tmp/foo.wbt"


def test_header_missing_warning():
    diag = classify_line("WARNING: '/tmp/foo.wbt': Missing header.")
    assert diag["code"] == "HEADER_MISSING"
    assert diag["severity"] == "warning"


def test_header_invalid():
    diag = classify_line("ERROR: '/tmp/foo.wbt': Invalid header.")
    assert diag["code"] == "HEADER_INVALID"
    assert diag["source_path"] == "/tmp/foo.wbt"


def test_parse_error_with_location():
    diag = classify_line("ERROR: '/tmp/foo.wbt':42:7: error: expected '{'.")
    assert diag["code"] == "PARSE_ERROR"
    assert diag["source_path"] == "/tmp/foo.wbt"
    assert diag["line"] == 42
    assert diag["column"] == 7
    assert diag["detail"] == "expected '{'"


# ---- PROTO model (WbProtoModel.cpp) -----------------------------------------

def test_proto_recursive():
    diag = classify_line("ERROR: Recursive definition of PROTO node 'Foo' is not supported")
    assert diag["code"] == "PROTO_RECURSIVE"
    assert diag["node_def"] == "Foo"


def test_proto_base_name_invalid():
    diag = classify_line("ERROR: PROTO node 'Foo' cannot have a base node name")
    assert diag["code"] == "PROTO_BASE_NAME_INVALID"
    assert diag["node_def"] == "Foo"


def test_proto_name_mismatch():
    diag = classify_line("ERROR: 'Foo' PROTO identifier does not match filename")
    assert diag["code"] == "PROTO_NAME_MISMATCH"
    assert diag["node_def"] == "Foo"


def test_proto_param_error():
    diag = classify_line("ERROR: Errors when parsing the PROTO parameters")
    assert diag["code"] == "PROTO_PARAM_ERROR"


# ---- EXTERNPROTO and asset retrieval ----------------------------------------

def test_externproto_download_failed():
    diag = classify_line("ERROR: Error downloading EXTERNPROTO 'Foo': connection timed out")
    assert diag["code"] == "EXTERNPROTO_DOWNLOAD_FAILED"
    assert diag["node_def"] == "Foo"
    assert "connection timed out" in diag["detail"]


def test_asset_download_failed():
    diag = classify_line("ERROR: Cannot download 'https://example.com/x.png', error code: 404: Not Found")
    assert diag["code"] == "ASSET_DOWNLOAD_FAILED"
    assert diag["source_path"] == "https://example.com/x.png"


def test_texture_read_failed_is_warning():
    diag = classify_line("WARNING: Texture file could not be read: '/tmp/x.png'")
    assert diag["code"] == "TEXTURE_READ_FAILED"
    assert diag["severity"] == "warning"
    assert diag["source_path"] == "/tmp/x.png"


def test_mesh_read_failed_is_warning():
    diag = classify_line("WARNING: Mesh file could not be read: '/tmp/x.stl'")
    assert diag["code"] == "MESH_READ_FAILED"


def test_urdf_mesh_unresolved():
    diag = classify_line(
        "WARNING: URDF link 'base_link': visual mesh '/tmp/x.dae' could not be resolved and will be skipped."
    )
    assert diag["code"] == "URDF_MESH_UNRESOLVED"
    assert diag["node_def"] == "base_link"


# ---- Controller lifecycle (WbController.cpp) --------------------------------

def test_controller_crashed():
    diag = classify_line("WARNING: 'my_controller' controller crashed.")
    assert diag["code"] == "CONTROLLER_CRASHED"
    assert diag["node_def"] == "my_controller"


def test_controller_exited_nonzero():
    diag = classify_line("WARNING: 'my_controller' controller exited with status: 137.")
    assert diag["code"] == "CONTROLLER_EXITED_NONZERO"
    assert diag["node_def"] == "my_controller"
    assert diag["detail"] == "137"


# ---- Tagged-line short-circuit (OMNISIM_STRUCTURED_LOG=1) --------------------

def test_tagged_line_simple():
    diag = classify_line('[OMNISIM-DIAG] code=PROTO_NOT_FOUND severity=error message="Foo not found"')
    assert diag["code"] == "PROTO_NOT_FOUND"
    assert diag["severity"] == "error"
    assert diag["message"] == "Foo not found"


def test_tagged_line_with_file_and_line():
    diag = classify_line(
        '[OMNISIM-DIAG] code=PARSE_ERROR severity=error file="/tmp/x.wbt" line=42 message="bad token"'
    )
    assert diag["code"] == "PARSE_ERROR"
    assert diag["source_path"] == "/tmp/x.wbt"
    assert diag["line"] == 42


def test_tagged_line_unknown_severity_defaults_to_error():
    diag = classify_line('[OMNISIM-DIAG] code=FOO severity=potato message="x"')
    assert diag["severity"] == "error"


def test_tagged_line_without_code_falls_through():
    # No `code=` field means it isn't a real diagnostic tag — treat as raw text.
    assert classify_line("[OMNISIM-DIAG] severity=error message=\"x\"") is None


# ---- CUDA codes (tagged-line only — no regex rules) -------------------------

def test_cuda_codes_round_trip_through_tagged_parser():
    # CUDA_* codes are emitted only via WbLog::diagnostic. Verify the tagged
    # parser preserves them verbatim and the published vocabulary list stays
    # in sync with what the parser actually accepts.
    from diagnostic_codes import CUDA_CODES
    for code in CUDA_CODES:
        diag = classify_line(f'[OMNISIM-DIAG] code={code} severity=error message="boom"')
        assert diag is not None
        assert diag["code"] == code
        assert diag["severity"] == "error"


def test_cuda_not_available_is_branchable():
    diag = classify_line('[OMNISIM-DIAG] code=CUDA_NOT_AVAILABLE severity=warning '
                         'message="No CUDA-capable device found"')
    assert diag["code"] == "CUDA_NOT_AVAILABLE"
    assert diag["severity"] == "warning"
    assert "CUDA-capable device" in diag["message"]


# ---- Multi-line text input ---------------------------------------------------

@pytest.fixture
def world_load_log() -> str:
    return (
        "=== OmniSim Log Started: 2026-04-27 14:37:00 ===\n"
        "INFO: Reading world file\n"
        "WARNING: Texture file could not be read: '/tmp/missing.png'\n"
        "ERROR: '/tmp/foo.wbt': Failed to load due to syntax error(s).\n"
        "ERROR: Could not open file: '/tmp/also_missing.wbt'.\n"
    )


def test_classify_text_returns_only_diagnostics(world_load_log):
    diags = classify_text(world_load_log)
    codes = [d["code"] for d in diags]
    assert codes == ["TEXTURE_READ_FAILED", "WORLD_PARSE_SYNTAX_ERROR", "WORLD_FILE_NOT_FOUND"]


def test_classify_text_preserves_severity_distribution(world_load_log):
    diags = classify_text(world_load_log)
    severities = [d["severity"] for d in diags]
    assert severities == ["warning", "error", "error"]


def test_classify_text_drops_info_and_banner(world_load_log):
    # The header banner and INFO: lines must not appear as diagnostics.
    diags = classify_text(world_load_log)
    for diag in diags:
        assert "OmniSim Log Started" not in diag.get("message", "")
        assert diag["severity"] in ("warning", "error", "fatal")
