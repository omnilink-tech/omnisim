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

"""Diagnostic-code mapper for the OmniSim agent-facing validation harness.

Free-text log lines from `omnisim_log.txt` (and the C++ side of the simulator)
are mapped to structured diagnostic dicts that an agent can branch on.

Two input shapes are supported:

1. **Tagged lines** emitted by the C++ side when `OMNISIM_STRUCTURED_LOG=1`:

       [OMNISIM-DIAG] code=PROTO_NOT_FOUND severity=error file=foo.wbt line=42 message=...

   These short-circuit the regex matcher and are the most reliable signal.

2. **Legacy free-text lines** prefixed by `ERROR:` / `WARNING:` / `FATAL:`.
   These are pattern-matched against the rules below; rules are anchored in
   the actual strings emitted by the simulator (grep `WbLog::error` /
   `tr(...)` calls in `src/omnisim/`).

Anything that matches no rule is returned with `code = "UNKNOWN"` and the
full raw line, so nothing is silently dropped.

The mapping is intentionally a flat ordered list — readability and the
ability to add a rule by appending one tuple matter more than performance.
The first matching rule wins.
"""

from __future__ import annotations

import re
from typing import Iterable

# Severity strings used in diagnostic dicts.
FATAL = "fatal"
ERROR = "error"
WARNING = "warning"

# Tagged-line format emitted from C++ when OMNISIM_STRUCTURED_LOG=1.
# Example:
#   [OMNISIM-DIAG] code=WORLD_PARSE_INVALID_TOKENS severity=error message=...
_TAG_PREFIX = "[OMNISIM-DIAG]"
_TAG_KV = re.compile(r'(\w+)=(?:"((?:[^"\\]|\\.)*)"|(\S+))')

# Header-stripping for legacy lines from WbLog.cpp.
_HEADER_PATTERNS: tuple[tuple[str, str], ...] = (
    ("FATAL: ", FATAL),
    ("ERROR: ", ERROR),
    ("WARNING: ", WARNING),
)

# A rule: (severity, compiled regex, code, dict of group name -> field name).
# `severity` of None means "match any severity".
# Groups named in the regex that map to a field name are extracted into the
# diagnostic dict alongside the standard fields.
_Rule = tuple[str | None, re.Pattern[str], str, dict[str, str]]


def _r(severity: str | None, pattern: str, code: str, **groups: str) -> _Rule:
    return (severity, re.compile(pattern), code, groups)


# Codes emitted exclusively via the tagged-line path (`WbLog::diagnostic`),
# so they need no regex rule — the tagged-line parser short-circuits and
# preserves the code field verbatim. Listed here so an agent reading the
# harness vocabulary sees the full set of CUDA_* codes the C++ side may emit.
# Source: src/omnisim/compute/cuda/WbCudaError.hpp.
CUDA_CODES: tuple[str, ...] = (
    "CUDA_NOT_AVAILABLE",
    "CUDA_DEVICE_INIT_FAILED",
    "CUDA_DRIVER_TOO_OLD",
    "CUDA_COMPUTE_CAPABILITY_TOO_OLD",
    "CUDA_OUT_OF_MEMORY",
    "CUDA_KERNEL_LAUNCH_FAILED",
    "CUDA_KERNEL_EXECUTION_ERROR",
    "CUDA_MEMCPY_FAILED",
    "CUDA_GL_INTEROP_NOT_IMPLEMENTED",
)


# Order matters: more specific rules come before generic ones.
# Patterns are anchored in real WbLog::error / WbLog::warning call sites in
# src/omnisim/. Updating the simulator's user-facing strings should be paired
# with updates here; the test in tests/harness/test_diagnostics.py will catch
# rules that no longer match.
_RULES: tuple[_Rule, ...] = (
    # ---- World file gating (WbApplication.cpp) ----
    _r(ERROR, r"^Could not open file: '(?P<file>[^']+)'\. The world file extension must be '\.wbt'\.$",
       "WORLD_WRONG_EXTENSION", file="source_path"),
    _r(ERROR, r"^Could not open file: '(?P<file>[^']+)'\.$",
       "WORLD_FILE_NOT_FOUND", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Failed to load due to invalid token\(s\)\.$",
       "WORLD_PARSE_INVALID_TOKENS", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Failed to load due to syntax error\(s\)\.$",
       "WORLD_PARSE_SYNTAX_ERROR", file="source_path"),

    # ---- Tokenizer / parser (WbTokenizer.cpp) ----
    _r(ERROR, r"^File is empty: '(?P<file>[^']+)'\.$",
       "WORLD_FILE_EMPTY", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': error: Missing header\.$",
       "HEADER_MISSING", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Invalid header\.$",
       "HEADER_INVALID", file="source_path"),
    _r(WARNING, r"^'(?P<file>[^']+)': Missing header\.$",
       "HEADER_MISSING", file="source_path"),
    # Generic parser error with location: 'file':line:col: error: message.
    _r(ERROR, r"^'(?P<file>[^']+)':(?P<line>\d+):(?P<column>\d+): error: (?P<detail>.+?)\.$",
       "PARSE_ERROR", file="source_path", line="line", column="column", detail="detail"),

    # ---- PROTO model (WbProtoModel.cpp) ----
    _r(ERROR, r"^Recursive definition of PROTO node '(?P<proto>[^']+)' is not supported$",
       "PROTO_RECURSIVE", proto="node_def"),
    _r(ERROR, r"^PROTO node '(?P<proto>[^']+)' cannot have a base node name$",
       "PROTO_BASE_NAME_INVALID", proto="node_def"),
    _r(ERROR, r"^'(?P<proto>[^']+)' PROTO identifier does not match filename$",
       "PROTO_NAME_MISMATCH", proto="node_def"),
    _r(ERROR, r"^Errors when parsing the PROTO parameters$",
       "PROTO_PARAM_ERROR"),

    # ---- EXTERNPROTO and asset retrieval ----
    _r(ERROR, r"^Error downloading EXTERNPROTO '(?P<proto>[^']+)': (?P<detail>.+)$",
       "EXTERNPROTO_DOWNLOAD_FAILED", proto="node_def", detail="detail"),
    _r(ERROR, r"^Cannot download '(?P<url>[^']+)', error code: (?P<status>\d+): (?P<detail>.+)$",
       "ASSET_DOWNLOAD_FAILED", url="source_path", status="detail", detail="detail"),
    _r(WARNING, r"^Texture file could not be read: '(?P<file>[^']+)'$",
       "TEXTURE_READ_FAILED", file="source_path"),
    _r(WARNING, r"^Mesh file could not be read: '(?P<file>[^']+)'$",
       "MESH_READ_FAILED", file="source_path"),
    _r(WARNING, r"^URDF link '(?P<link>[^']+)': .* mesh '(?P<mesh>[^']+)' could not be resolved.*$",
       "URDF_MESH_UNRESOLVED", link="node_def", mesh="source_path"),

    # ---- Controller lifecycle (WbController.cpp) ----
    _r(WARNING, r"^'(?P<controller>[^']+)' controller crashed\.$",
       "CONTROLLER_CRASHED", controller="node_def"),
    _r(WARNING, r"^'(?P<controller>[^']+)' controller exited with status: (?P<status>\d+)\.$",
       "CONTROLLER_EXITED_NONZERO", controller="node_def", status="detail"),
)


def _classify_line(severity: str, message: str) -> dict:
    for sev_filter, regex, code, groups in _RULES:
        if sev_filter is not None and sev_filter != severity:
            continue
        match = regex.match(message)
        if match is None:
            continue
        diag = {
            "code": code,
            "severity": severity,
            "message": message,
        }
        for group_name, field in groups.items():
            try:
                value = match.group(group_name)
            except IndexError:
                continue
            if value is None:
                continue
            if field == "line" or field == "column":
                try:
                    value = int(value)
                except ValueError:
                    pass
            diag[field] = value
        return diag
    return {"code": "UNKNOWN", "severity": severity, "message": message}


def _parse_tagged_line(payload: str) -> dict | None:
    fields: dict[str, str] = {}
    for match in _TAG_KV.finditer(payload):
        key = match.group(1)
        quoted = match.group(2)
        bare = match.group(3)
        value = quoted if quoted is not None else bare
        if value is not None:
            fields[key] = value.replace(r"\"", '"').replace(r"\\", "\\")
    if "code" not in fields:
        return None
    severity = fields.pop("severity", ERROR).lower()
    if severity not in (FATAL, ERROR, WARNING):
        severity = ERROR
    diag: dict = {
        "code": fields.pop("code"),
        "severity": severity,
        "message": fields.pop("message", ""),
    }
    if "file" in fields:
        diag["source_path"] = fields.pop("file")
    if "line" in fields:
        try:
            diag["line"] = int(fields.pop("line"))
        except ValueError:
            diag["line"] = fields.pop("line")
    if "node" in fields:
        diag["node_def"] = fields.pop("node")
    diag.update(fields)
    return diag


def classify_line(line: str) -> dict | None:
    """Classify one log line into a diagnostic dict, or None to ignore it."""
    stripped = line.strip()
    if not stripped:
        return None

    if stripped.startswith(_TAG_PREFIX):
        payload = stripped[len(_TAG_PREFIX):].strip()
        diag = _parse_tagged_line(payload)
        if diag is not None:
            diag["raw"] = stripped
            return diag

    for header, severity in _HEADER_PATTERNS:
        if stripped.startswith(header):
            message = stripped[len(header):].strip()
            diag = _classify_line(severity, message)
            diag["raw"] = stripped
            return diag

    return None


def classify_lines(lines: Iterable[str]) -> list[dict]:
    """Classify a stream of log lines, dropping ones that aren't diagnostics."""
    out: list[dict] = []
    for line in lines:
        diag = classify_line(line)
        if diag is not None:
            out.append(diag)
    return out


def classify_text(text: str) -> list[dict]:
    """Classify the contents of an `omnisim_log.txt`-style log file."""
    return classify_lines(text.splitlines())
