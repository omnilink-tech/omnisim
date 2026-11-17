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

"""PROTO header parser.

Extracts everything an agent or validator needs to reason about a PROTO
without executing its JavaScript template body:

* Header metadata (license, documentation URL, keywords, prose description,
  template language).
* ``EXTERNPROTO`` declarations.
* The PROTO name.
* Field declarations: access modifier, VRML type, name, raw + typed default,
  end-of-line hint comment, and any preceding standalone comment lines.

The parser is intentionally permissive: malformed or unrecognized field
shapes are preserved as ``raw_default`` strings rather than rejected, so
the tool surfaces the issue downstream in :mod:`omnisim.protos.validate`
instead of crashing on first-encounter.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field as dc_field
from pathlib import Path
from typing import Any, Iterator


VRML_TYPES = frozenset({
    "SFBool", "SFInt32", "SFFloat", "SFString", "SFColor",
    "SFVec2f", "SFVec3f", "SFRotation", "SFNode", "SFTime",
    "MFBool", "MFInt32", "MFFloat", "MFString", "MFColor",
    "MFVec2f", "MFVec3f", "MFRotation", "MFNode", "MFTime",
})

ACCESS_MODIFIERS = frozenset({
    "field", "hiddenField", "deprecatedField", "vrmlField", "w3dField",
})

_HEADER_TAGS = (
    "license",
    "license url",
    "documentation url",
    "keywords",
    "template language",
    "tags",
)


@dataclass
class ProtoField:
    access: str
    type: str
    name: str
    raw_default: str
    default: Any
    hint: str = ""
    leading_comments: list[str] = dc_field(default_factory=list)
    line: int = 0
    enum: list[Any] | None = None  # OmniSim field-enum restriction, e.g. SFString{"red","blue"}


@dataclass
class ProtoHeader:
    vrml_line: str = ""
    license: str = ""
    license_url: str = ""
    documentation_url: str = ""
    keywords: list[str] = dc_field(default_factory=list)
    template_language: str = ""
    description: str = ""
    tags: list[str] = dc_field(default_factory=list)


@dataclass
class ParsedProto:
    name: str
    path: Path
    header: ProtoHeader
    extern_protos: list[str]
    fields: list[ProtoField]


class ProtoParseError(ValueError):
    """Raised for unrecoverable parse failures (missing PROTO keyword, etc.)."""


# --------------------------------------------------------------------------- #
# Tokenizer
# --------------------------------------------------------------------------- #


_TOKEN_RE = re.compile(
    r"""
    (?P<string>"(?:\\.|[^"\\])*")          # quoted string
    | (?P<lbrack>\[)
    | (?P<rbrack>\])
    | (?P<lbrace>\{)
    | (?P<rbrace>\})
    | (?P<comment>\#[^\n]*)
    | (?P<ws>\s+)
    | (?P<word>[^\s\[\]\{\}\#"]+)
    """,
    re.VERBOSE,
)


def _tokenize(text: str) -> Iterator[tuple[str, str, int]]:
    """Yield ``(kind, value, line)`` tokens. Skips whitespace; keeps comments."""
    line = 1
    for m in _TOKEN_RE.finditer(text):
        kind = m.lastgroup
        val = m.group()
        if kind == "ws":
            line += val.count("\n")
            continue
        yield kind, val, line
        line += val.count("\n")


# --------------------------------------------------------------------------- #
# Default-value typing
# --------------------------------------------------------------------------- #


def _parse_number(tok: str) -> float | int:
    try:
        if any(c in tok for c in ".eE"):
            return float(tok)
        return int(tok)
    except ValueError:
        return float("nan")


def _typed_default(vrml_type: str, raw: str) -> Any:
    """Best-effort typed projection of a raw default value.

    Returns ``None`` if the default is ``NULL`` (for SFNode) or empty MF.
    Returns the ``raw`` string unchanged for shapes we don't recognize so
    downstream consumers can still see something.
    """
    raw = raw.strip()
    if not raw:
        return None
    if vrml_type == "SFBool":
        u = raw.upper()
        if u in ("TRUE", "FALSE"):
            return u == "TRUE"
        return raw
    if vrml_type == "SFInt32":
        try:
            return int(raw)
        except ValueError:
            return raw
    if vrml_type in ("SFFloat", "SFTime"):
        try:
            return float(raw)
        except ValueError:
            return raw
    if vrml_type == "SFString":
        if raw.startswith('"') and raw.endswith('"'):
            return raw[1:-1]
        return raw
    if vrml_type in ("SFVec2f", "SFVec3f", "SFColor", "SFRotation"):
        parts = raw.split()
        nums = [_parse_number(p) for p in parts]
        return nums if nums else raw
    if vrml_type == "SFNode":
        if raw.upper() == "NULL":
            return None
        return raw  # node literal, keep raw
    if vrml_type.startswith("MF"):
        # Strip outer [ ] if present.
        inner = raw.strip()
        if inner.startswith("[") and inner.endswith("]"):
            inner = inner[1:-1].strip()
        if not inner:
            return []
        # For MFString, split on quoted tokens; for MF-numeric, on whitespace.
        if vrml_type == "MFString":
            return re.findall(r'"((?:\\.|[^"\\])*)"', inner)
        if vrml_type in ("MFBool", "MFInt32", "MFFloat", "MFTime",
                        "MFVec2f", "MFVec3f", "MFColor", "MFRotation"):
            parts = inner.split()
            return [_parse_number(p) for p in parts]
        # MFNode → keep raw; nodes are complex.
        return raw
    return raw


# --------------------------------------------------------------------------- #
# Header
# --------------------------------------------------------------------------- #


def _parse_header(lines: list[str]) -> tuple[ProtoHeader, int]:
    """Parse leading ``#`` comments. Returns (header, idx_of_next_line)."""
    header = ProtoHeader()
    desc_lines: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.rstrip()
        if i == 0 and stripped.startswith(("#OMNISIM", "#VRML_SIM")):
            header.vrml_line = stripped
            i += 1
            continue
        if not stripped:
            i += 1
            continue
        if stripped.startswith("#"):
            body = stripped[1:].strip()
            matched = False
            for tag in _HEADER_TAGS:
                pref = f"{tag}:"
                if body.lower().startswith(pref):
                    val = body[len(pref):].strip()
                    if tag == "license":
                        header.license = val
                    elif tag == "license url":
                        header.license_url = val
                    elif tag == "documentation url":
                        header.documentation_url = val
                    elif tag == "keywords":
                        header.keywords = [k.strip() for k in val.split(",") if k.strip()]
                    elif tag == "template language":
                        header.template_language = val
                    elif tag == "tags":
                        header.tags = [t.strip() for t in val.split(",") if t.strip()]
                    matched = True
                    break
            if not matched:
                desc_lines.append(body)
            i += 1
            continue
        break
    header.description = " ".join(desc_lines).strip()
    return header, i


# --------------------------------------------------------------------------- #
# Field block
# --------------------------------------------------------------------------- #


def _read_balanced(tokens: list[tuple[str, str, int]], idx: int,
                   open_kind: str, close_kind: str) -> tuple[str, int]:
    """Consume a balanced [...] or {...} block starting at ``tokens[idx]``.

    Returns (raw_text_including_delimiters, next_idx).
    """
    parts: list[str] = []
    depth = 0
    while idx < len(tokens):
        kind, val, _ = tokens[idx]
        if kind == "comment":
            idx += 1
            continue
        parts.append(val)
        if kind == open_kind:
            depth += 1
        elif kind == close_kind:
            depth -= 1
            if depth == 0:
                return " ".join(parts), idx + 1
        idx += 1
    raise ProtoParseError(f"unbalanced '{open_kind}' block")


def _consume_default(vrml_type: str, tokens: list[tuple[str, str, int]],
                     idx: int) -> tuple[str, str, int]:
    """Read the default-value tokens for a field. Returns (raw_default, hint, next_idx).

    The default ends when we either:
      * Finish a balanced ``[..]`` (MF) or ``{..}`` (SFNode literal) block, or
      * Hit a ``#`` comment (which becomes the hint), or
      * Hit the next access-modifier / ``]`` / EOF on a new logical field.

    For scalar types we read exactly the type's arity; that's the most
    robust way to avoid swallowing the start of the next field.
    """
    parts: list[str] = []
    hint = ""
    arity = _scalar_arity(vrml_type)
    consumed_scalars = 0

    while idx < len(tokens):
        kind, val, _ = tokens[idx]
        if kind == "comment":
            hint = val.lstrip("#").strip()
            idx += 1
            return " ".join(parts), hint, idx
        if kind == "lbrack":
            raw, idx = _read_balanced(tokens, idx, "lbrack", "rbrack")
            parts.append(raw)
            # MF default consumed; look for trailing comment.
            return _attach_trailing_hint(" ".join(parts), tokens, idx)
        if kind == "lbrace":
            raw, idx = _read_balanced(tokens, idx, "lbrace", "rbrace")
            if parts:
                parts[-1] = parts[-1] + " " + raw
            else:
                parts.append(raw)
            return _attach_trailing_hint(" ".join(parts), tokens, idx)
        if kind == "rbrack":
            # End of field block — caller will handle.
            return " ".join(parts), hint, idx
        if kind == "word" or kind == "string":
            parts.append(val)
            if arity > 0:
                consumed_scalars += 1
                if consumed_scalars >= arity:
                    idx += 1
                    return _attach_trailing_hint(" ".join(parts), tokens, idx)
            elif vrml_type == "SFNode" and val.upper() == "NULL":
                idx += 1
                return _attach_trailing_hint("NULL", tokens, idx)
            idx += 1
            continue
        idx += 1
    return " ".join(parts), hint, idx


def _scalar_arity(vrml_type: str) -> int:
    """Number of whitespace-separated scalar tokens a field of this type expects.

    Returns 0 for types whose default shape isn't fixed-arity scalar
    (MF*, SFNode literal, etc.) — the caller handles those via balanced reads.
    """
    return {
        "SFBool": 1,
        "SFInt32": 1,
        "SFFloat": 1,
        "SFString": 1,
        "SFTime": 1,
        "SFVec2f": 2,
        "SFVec3f": 3,
        "SFColor": 3,
        "SFRotation": 4,
    }.get(vrml_type, 0)


def _attach_trailing_hint(raw: str, tokens: list[tuple[str, str, int]],
                          idx: int) -> tuple[str, str, int]:
    """If the next non-whitespace token is a comment, consume it as the hint."""
    if idx < len(tokens) and tokens[idx][0] == "comment":
        hint = tokens[idx][1].lstrip("#").strip()
        return raw, hint, idx + 1
    return raw, "", idx


def _parse_field_block(tokens: list[tuple[str, str, int]],
                       start_idx: int) -> tuple[list[ProtoField], int]:
    """Parse from the opening ``[`` of a PROTO field block to its closing ``]``."""
    assert tokens[start_idx][0] == "lbrack"
    idx = start_idx + 1
    fields: list[ProtoField] = []
    pending_comments: list[str] = []

    while idx < len(tokens):
        kind, val, line = tokens[idx]
        if kind == "rbrack":
            return fields, idx + 1
        if kind == "comment":
            pending_comments.append(val.lstrip("#").strip())
            idx += 1
            continue
        if kind == "word" and val in ACCESS_MODIFIERS:
            access = val
            idx += 1
            # Skip comments between access and type.
            while idx < len(tokens) and tokens[idx][0] == "comment":
                pending_comments.append(tokens[idx][1].lstrip("#").strip())
                idx += 1
            if idx >= len(tokens) or tokens[idx][0] != "word":
                raise ProtoParseError(f"expected VRML type after '{access}' near line {line}")
            vrml_type = tokens[idx][1]
            idx += 1
            # Optional inline enum constraint: ``SFString{"red","blue"}``.
            enum_values: list[Any] | None = None
            if idx < len(tokens) and tokens[idx][0] == "lbrace":
                raw_enum, idx = _read_balanced(tokens, idx, "lbrace", "rbrace")
                inner = raw_enum.strip().lstrip("{").rstrip("}").strip()
                enum_values = _parse_enum_values(vrml_type, inner)
            if idx >= len(tokens) or tokens[idx][0] != "word":
                raise ProtoParseError(f"expected field name after '{vrml_type}' near line {line}")
            name = tokens[idx][1]
            idx += 1
            raw_default, hint, idx = _consume_default(vrml_type, tokens, idx)
            fields.append(ProtoField(
                access=access,
                type=vrml_type,
                name=name,
                raw_default=raw_default.strip(),
                default=_typed_default(vrml_type, raw_default),
                hint=hint,
                leading_comments=list(pending_comments),
                line=line,
                enum=enum_values,
            ))
            pending_comments.clear()
            continue
        # Unknown token inside field block — skip.
        idx += 1
    raise ProtoParseError("PROTO field block not terminated")


# --------------------------------------------------------------------------- #
# Top-level
# --------------------------------------------------------------------------- #


_EXTERN_RE = re.compile(r'EXTERNPROTO\s+"([^"]+)"')
_PROTO_RE = re.compile(r"\bPROTO\s+([\w\-]+)\s*\[")


def _parse_enum_values(vrml_type: str, inner: str) -> list[Any]:
    """Parse the body of an inline enum-restriction ``{...}`` block.

    OmniSim accepts comma-separated literals matching the field's VRML type
    (strings for SFString, integers for SFInt32, etc.). We project them
    through the same typed-default machinery so the sidecar carries
    real values rather than raw strings.
    """
    parts = [p.strip() for p in inner.split(",") if p.strip()]
    return [_typed_default(vrml_type, p) for p in parts]


def parse_proto(text: str, path: Path | None = None) -> ParsedProto:
    """Parse a PROTO file into a :class:`ParsedProto`.

    ``path`` is informational only; it's threaded through into the result
    so error messages can name the source file.
    """
    path = path or Path("<memory>")
    lines = text.splitlines(keepends=True)
    header, hdr_end = _parse_header(lines)

    rest = "".join(lines[hdr_end:])
    extern_protos = _EXTERN_RE.findall(rest)

    pm = _PROTO_RE.search(rest)
    if not pm:
        raise ProtoParseError(f"{path}: no PROTO declaration found")
    name = pm.group(1)

    # Tokenize from the '[' onward.
    bracket_pos = rest.index("[", pm.end() - 1)
    tokens = list(_tokenize(rest[bracket_pos:]))
    fields, _ = _parse_field_block(tokens, 0)

    return ParsedProto(
        name=name,
        path=path,
        header=header,
        extern_protos=extern_protos,
        fields=fields,
    )


def parse_proto_file(path: Path) -> ParsedProto:
    return parse_proto(path.read_text(encoding="utf-8"), path=path)
