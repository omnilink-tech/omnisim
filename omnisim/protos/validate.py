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

"""Validate PROTOs against their ``.proto.yaml`` schema sidecars.

Emits structured findings agents can act on:

.. code-block:: json

    {
      "path": "projects/.../Foo.proto",
      "line": 17,
      "field": "height",
      "code": "EXTRA-MIN",
      "severity": "error",
      "message": "default 0 is below extra.min 0.001",
      "expected": ">= 0.001",
      "got": 0,
      "suggestion": "Raise the default or relax extra.min."
    }

Run with ``python -m omnisim proto validate [paths...]``.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field as dc_field
from pathlib import Path
from typing import Any

from ..paths import REPO_ROOT
from .parser import (
    ACCESS_MODIFIERS,
    VRML_TYPES,
    ParsedProto,
    ProtoField,
    ProtoParseError,
    parse_proto_file,
)
from .schema import build_auto, dump_sidecar, load_sidecar


@dataclass
class Finding:
    path: str
    line: int
    code: str
    severity: str  # "error" | "warning" | "info"
    message: str
    field: str | None = None
    expected: Any = None
    got: Any = None
    suggestion: str | None = None


_ALLOWED_EXTRA_KEYS = frozenset({"units", "min", "max", "enum", "semantic", "notes"})


# --------------------------------------------------------------------------- #
# Structural checks (PROTO alone)
# --------------------------------------------------------------------------- #


def _check_header(parsed: ParsedProto, findings: list[Finding]) -> None:
    h = parsed.header
    rel = str(parsed.path)
    if not h.vrml_line.startswith(("#OMNISIM", "#VRML_SIM")):
        findings.append(Finding(rel, 1, "HEADER-MISSING-VRML",
                                "error", "missing '#OMNISIM ... utf8' first line",
                                suggestion="Add `#OMNISIM R2025a utf8` as line 1 "
                                           "(legacy `#VRML_SIM R2025a utf8` is also accepted)."))
    if not h.license:
        findings.append(Finding(rel, 1, "HEADER-MISSING-LICENSE",
                                "warning", "missing '# license:' header tag",
                                suggestion="Add a `# license: <name>` line before PROTO."))
    if not h.documentation_url:
        findings.append(Finding(rel, 1, "HEADER-MISSING-DOC-URL",
                                "info", "missing '# documentation url:' header tag"))


def _check_types(parsed: ParsedProto, findings: list[Finding]) -> None:
    seen: set[str] = set()
    rel = str(parsed.path)
    for f in parsed.fields:
        if f.access not in ACCESS_MODIFIERS:
            findings.append(Finding(rel, f.line, "FIELD-BAD-ACCESS",
                                    "error",
                                    f"unknown access modifier {f.access!r}",
                                    field=f.name, got=f.access,
                                    expected=sorted(ACCESS_MODIFIERS)))
        if f.type not in VRML_TYPES:
            findings.append(Finding(rel, f.line, "FIELD-BAD-TYPE",
                                    "error",
                                    f"unknown VRML type {f.type!r}",
                                    field=f.name, got=f.type,
                                    expected=sorted(VRML_TYPES)))
        if f.name in seen:
            findings.append(Finding(rel, f.line, "FIELD-DUPLICATE",
                                    "error",
                                    f"duplicate field {f.name!r}",
                                    field=f.name))
        seen.add(f.name)
        _check_default_shape(rel, f, findings)


def _check_default_shape(rel: str, f: ProtoField, findings: list[Finding]) -> None:
    """Default value must structurally match the declared VRML type."""
    d = f.default
    if f.type == "SFBool" and not isinstance(d, bool):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", f"SFBool default must be TRUE or FALSE",
                                field=f.name, got=f.raw_default, expected="TRUE | FALSE"))
    elif f.type == "SFInt32" and not isinstance(d, int):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", f"SFInt32 default must be an integer",
                                field=f.name, got=f.raw_default, expected="integer"))
    elif f.type in ("SFFloat", "SFTime") and not isinstance(d, (int, float)):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", f"{f.type} default must be numeric",
                                field=f.name, got=f.raw_default, expected="number"))
    elif f.type == "SFVec3f" and not (isinstance(d, list) and len(d) == 3):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", "SFVec3f default needs exactly 3 numbers",
                                field=f.name, got=f.raw_default, expected="x y z"))
    elif f.type == "SFVec2f" and not (isinstance(d, list) and len(d) == 2):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", "SFVec2f default needs exactly 2 numbers",
                                field=f.name, got=f.raw_default, expected="x y"))
    elif f.type == "SFColor" and not (isinstance(d, list) and len(d) == 3):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", "SFColor default needs exactly 3 numbers (r g b)",
                                field=f.name, got=f.raw_default, expected="r g b"))
    elif f.type == "SFRotation" and not (isinstance(d, list) and len(d) == 4):
        findings.append(Finding(rel, f.line, "DEFAULT-TYPE-MISMATCH",
                                "error", "SFRotation default needs 4 numbers (axis xyz + angle)",
                                field=f.name, got=f.raw_default, expected="ax ay az angle"))


def _check_enum_default(parsed: ParsedProto, findings: list[Finding]) -> None:
    """If a scalar field carries an inline enum constraint, the default must satisfy it.

    For SFNode/MFNode the inline ``{...}`` is a *type* constraint (allowed
    child node types), not a value enum, so value-equality is skipped.
    Full type-checking node defaults would require parsing the PROTO body,
    which is out of scope.
    """
    rel = str(parsed.path)
    for f in parsed.fields:
        if not f.enum or f.default is None:
            continue
        if f.type in ("SFNode", "MFNode"):
            continue
        if f.default in f.enum:
            continue
        findings.append(Finding(rel, f.line, "DEFAULT-NOT-IN-ENUM",
                                "error",
                                f"default {f.default!r} not in inline enum",
                                field=f.name, got=f.default, expected=f.enum,
                                suggestion="Pick one of the enum values or relax the constraint."))


# --------------------------------------------------------------------------- #
# Sidecar checks
# --------------------------------------------------------------------------- #


def _check_sidecar(parsed: ParsedProto, sidecar_path: Path,
                   strict: bool, findings: list[Finding]) -> dict[str, Any] | None:
    """Compare on-disk sidecar's ``auto`` block with a fresh regeneration."""
    rel = str(parsed.path)
    if not sidecar_path.exists():
        severity = "error" if strict else "warning"
        findings.append(Finding(rel, 1, "SCHEMA-MISSING",
                                severity, f"no sidecar at {sidecar_path}",
                                suggestion="Run `python -m omnisim proto schema`."))
        return None

    existing = load_sidecar(sidecar_path)
    try:
        rel_source = str(parsed.path.relative_to(REPO_ROOT)).replace("\\", "/")
    except ValueError:
        rel_source = str(parsed.path)
    fresh_auto = build_auto(parsed, rel_source)
    fresh_text = dump_sidecar(fresh_auto, existing.get("extra") if isinstance(existing, dict) else None)
    on_disk = sidecar_path.read_text(encoding="utf-8")
    if fresh_text != on_disk:
        findings.append(Finding(rel, 1, "SCHEMA-DRIFT",
                                "error",
                                "sidecar is stale; regenerate or the PROTO has been edited",
                                suggestion="Run `python -m omnisim proto schema`."))
    return existing if isinstance(existing, dict) else None


def _check_extra(parsed: ParsedProto, sidecar: dict[str, Any] | None,
                 findings: list[Finding]) -> None:
    """Enforce the hand-curated semantic constraints in ``extra.fields``."""
    if not sidecar:
        return
    extra = sidecar.get("extra") if isinstance(sidecar.get("extra"), dict) else None
    if not extra:
        return
    extra_fields = extra.get("fields") if isinstance(extra.get("fields"), dict) else None
    if not extra_fields:
        return

    by_name = {f.name: f for f in parsed.fields}
    rel = str(parsed.path)

    for fname, spec in extra_fields.items():
        if fname not in by_name:
            findings.append(Finding(rel, 1, "EXTRA-UNKNOWN-FIELD",
                                    "warning",
                                    f"extra.fields.{fname} has no matching PROTO field",
                                    field=fname,
                                    suggestion="Remove or rename the entry in extra.fields."))
            continue
        if not isinstance(spec, dict):
            continue
        for key in spec:
            if key not in _ALLOWED_EXTRA_KEYS:
                findings.append(Finding(rel, by_name[fname].line, "EXTRA-UNKNOWN-KEY",
                                        "warning",
                                        f"extra.fields.{fname}.{key} is not a recognized key",
                                        field=fname,
                                        expected=sorted(_ALLOWED_EXTRA_KEYS), got=key))
        f = by_name[fname]
        _enforce_min_max(rel, f, spec, findings)
        _enforce_extra_enum(rel, f, spec, findings)


def _as_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _enforce_min_max(rel: str, f: ProtoField, spec: dict[str, Any],
                     findings: list[Finding]) -> None:
    if "min" in spec:
        bound = _as_number(spec["min"])
        actual = _as_number(f.default)
        if bound is not None and actual is not None and actual < bound:
            findings.append(Finding(rel, f.line, "EXTRA-MIN",
                                    "error",
                                    f"default {f.default!r} below extra.min {bound}",
                                    field=f.name, got=f.default,
                                    expected=f">= {bound}",
                                    suggestion="Raise the PROTO default or relax extra.min."))
    if "max" in spec:
        bound = _as_number(spec["max"])
        actual = _as_number(f.default)
        if bound is not None and actual is not None and actual > bound:
            findings.append(Finding(rel, f.line, "EXTRA-MAX",
                                    "error",
                                    f"default {f.default!r} above extra.max {bound}",
                                    field=f.name, got=f.default,
                                    expected=f"<= {bound}",
                                    suggestion="Lower the PROTO default or relax extra.max."))


def _enforce_extra_enum(rel: str, f: ProtoField, spec: dict[str, Any],
                        findings: list[Finding]) -> None:
    if "enum" in spec:
        allowed = spec["enum"]
        if isinstance(allowed, list) and f.default not in allowed and f.default is not None:
            findings.append(Finding(rel, f.line, "EXTRA-ENUM",
                                    "error",
                                    f"default {f.default!r} not in extra.enum",
                                    field=f.name, got=f.default, expected=allowed,
                                    suggestion="Pick an enum value or expand extra.enum."))


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #


def validate_proto(proto_path: Path, strict: bool = False) -> list[Finding]:
    findings: list[Finding] = []
    try:
        parsed = parse_proto_file(proto_path)
    except ProtoParseError as e:
        return [Finding(str(proto_path), 1, "PARSE-ERROR", "error", str(e))]

    sidecar_path = proto_path.with_suffix(".proto.yaml")
    sidecar = _check_sidecar(parsed, sidecar_path, strict, findings)
    _check_header(parsed, findings)
    _check_types(parsed, findings)
    _check_enum_default(parsed, findings)
    _check_extra(parsed, sidecar, findings)
    return findings


def _iter_protos(paths: list[str]) -> list[Path]:
    if not paths:
        roots = [REPO_ROOT / "projects"]
    else:
        roots = [Path(p) if Path(p).is_absolute() else REPO_ROOT / p for p in paths]
    out: list[Path] = []
    for r in roots:
        if r.is_file() and r.suffix == ".proto":
            out.append(r)
        elif r.is_dir():
            out.extend(sorted(r.rglob("*.proto")))
    return sorted(set(out))


def run(args: argparse.Namespace) -> int:
    protos = _iter_protos(args.paths)
    if not protos:
        print("no PROTOs found", file=sys.stderr)
        return 1

    all_findings: list[Finding] = []
    for p in protos:
        all_findings.extend(validate_proto(p, strict=args.strict))

    errors = [f for f in all_findings if f.severity == "error"]
    warnings = [f for f in all_findings if f.severity == "warning"]
    infos = [f for f in all_findings if f.severity == "info"]

    if args.json:
        print(json.dumps([asdict(f) for f in all_findings], indent=2, default=str))
    else:
        if not all_findings:
            print(f"proto validate: {len(protos)} PROTOs — all clean")
        else:
            for f in all_findings:
                tag = f.severity.upper()
                where = f"{f.path}:{f.line}" + (f" [{f.field}]" if f.field else "")
                print(f"{tag} {f.code} {where} — {f.message}")
            print()
            print(f"proto validate: {len(protos)} PROTOs scanned — "
                  f"{len(errors)} errors, {len(warnings)} warnings, {len(infos)} info")

    return 1 if errors else 0
