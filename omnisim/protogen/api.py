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

"""Python authoring API for PROTOs.

The Python source declares a single :func:`emit` call describing the
header, fields, and body. The transpiler (``python -m omnisim proto
build``) imports the source — which sets a module-level
``_LAST_EMITTED`` via :func:`emit` — and renders the result to a
sibling ``.proto`` file.

This module also exposes :func:`render` for unit-test use without
touching the filesystem.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from typing import Any, Iterable

from ..protos.parser import ACCESS_MODIFIERS, VRML_TYPES


@dataclass
class FieldSpec:
    type: str
    name: str
    default: Any
    hint: str = ""
    access: str = "field"
    enum: list[Any] | None = None

    def validate(self) -> None:
        if self.access not in ACCESS_MODIFIERS:
            raise ValueError(f"unknown access modifier: {self.access!r}")
        if self.type not in VRML_TYPES:
            raise ValueError(f"unknown VRML type: {self.type!r}")


@dataclass
class ProtoSpec:
    name: str
    fields: list[FieldSpec]
    body: str
    extern_protos: list[str] = dc_field(default_factory=list)
    license: str = ""
    license_url: str = ""
    documentation_url: str = ""
    keywords: list[str] = dc_field(default_factory=list)
    template_language: str = ""
    description: str = ""
    vrml_version: str = "R2025a"


_LAST_EMITTED: ProtoSpec | None = None


def emit(
    *,
    name: str,
    fields: Iterable[Any],
    body: str,
    extern_protos: Iterable[str] = (),
    license: str = "",
    license_url: str = "",
    documentation_url: str = "",
    keywords: Iterable[str] = (),
    template_language: str = "",
    description: str = "",
    vrml_version: str = "R2025a",
) -> ProtoSpec:
    """Capture a PROTO spec into module-level state for the transpiler.

    ``fields`` accepts either :class:`FieldSpec` instances or
    ``(type, name, default[, hint])`` tuples for brevity.
    """
    normalized: list[FieldSpec] = []
    for item in fields:
        if isinstance(item, FieldSpec):
            normalized.append(item)
        elif isinstance(item, tuple):
            if len(item) == 3:
                t, n, d = item
                normalized.append(FieldSpec(type=t, name=n, default=d))
            elif len(item) == 4:
                t, n, d, h = item
                normalized.append(FieldSpec(type=t, name=n, default=d, hint=h))
            else:
                raise ValueError(f"field tuple needs 3 or 4 elements, got {item!r}")
        else:
            raise TypeError(f"unsupported field spec: {item!r}")
    for f in normalized:
        f.validate()
    spec = ProtoSpec(
        name=name,
        fields=normalized,
        body=body,
        extern_protos=list(extern_protos),
        license=license,
        license_url=license_url,
        documentation_url=documentation_url,
        keywords=list(keywords),
        template_language=template_language,
        description=description,
        vrml_version=vrml_version,
    )
    global _LAST_EMITTED
    _LAST_EMITTED = spec
    return spec


def consume_last() -> ProtoSpec | None:
    """Return and clear the most recent :func:`emit` payload."""
    global _LAST_EMITTED
    spec = _LAST_EMITTED
    _LAST_EMITTED = None
    return spec


# --------------------------------------------------------------------------- #
# Rendering: ProtoSpec -> .proto text
# --------------------------------------------------------------------------- #


def _format_default(vrml_type: str, value: Any) -> str:
    if vrml_type == "SFBool":
        return "TRUE" if value else "FALSE"
    if vrml_type == "SFString":
        return '"' + str(value).replace('"', '\\"') + '"'
    if vrml_type == "SFNode":
        if value is None:
            return "NULL"
        return str(value)
    if vrml_type in ("SFInt32",):
        return str(int(value))
    if vrml_type in ("SFFloat", "SFTime"):
        return _fmt_num(float(value))
    if vrml_type in ("SFVec2f", "SFVec3f", "SFColor", "SFRotation"):
        if not isinstance(value, (list, tuple)):
            return str(value)
        return " ".join(_fmt_num(float(v)) for v in value)
    if vrml_type.startswith("MF"):
        if not value:
            return "[]"
        if isinstance(value, str):
            # Pre-rendered MF default (e.g. multi-line node list).
            return value
        if vrml_type == "MFString":
            return "[ " + " ".join(f'"{str(v)}"' for v in value) + " ]"
        return "[ " + " ".join(_fmt_num(float(v)) for v in value) + " ]"
    return str(value)


def _fmt_num(v: float) -> str:
    if v == int(v):
        return str(int(v))
    return ("%g" % v)


def _format_field_line(f: FieldSpec, col_widths: dict[str, int]) -> str:
    type_col = col_widths["type"]
    name_col = col_widths["name"]
    default_col = col_widths["default"]
    default = _format_default(f.type, f.default)
    enum = ""
    if f.enum:
        enum = "{" + ", ".join(_format_default(f.type, v) for v in f.enum) + "}"
    type_with_enum = f.type + enum
    line = f"  {f.access} {type_with_enum:<{type_col}} {f.name:<{name_col}} {default:<{default_col}}"
    if f.hint:
        line = line.rstrip() + f"  # {f.hint}"
    return line.rstrip()


def render(spec: ProtoSpec) -> str:
    """Render a :class:`ProtoSpec` into the canonical PROTO text."""
    lines: list[str] = []
    lines.append(f"#OMNISIM {spec.vrml_version} utf8")
    if spec.license:
        lines.append(f"# license: {spec.license}")
    if spec.license_url:
        lines.append(f"# license url: {spec.license_url}")
    if spec.documentation_url:
        lines.append(f"# documentation url: {spec.documentation_url}")
    if spec.keywords:
        lines.append(f"# keywords: {', '.join(spec.keywords)}")
    if spec.description:
        # Wrap into a single comment block, preserving paragraph breaks.
        for para_line in spec.description.splitlines() or [""]:
            stripped = para_line.strip()
            if stripped:
                lines.append(f"# {stripped}")
    if spec.template_language:
        lines.append(f"# template language: {spec.template_language}")
    lines.append("")

    for url in spec.extern_protos:
        lines.append(f'EXTERNPROTO "{url}"')
    if spec.extern_protos:
        lines.append("")

    lines.append(f"PROTO {spec.name} [")
    col_widths = _compute_widths(spec.fields)
    for f in spec.fields:
        lines.append(_format_field_line(f, col_widths))
    lines.append("]")
    lines.append("{")
    body = spec.body.rstrip("\n")
    # Dedent the body block uniformly so authors can use triple-quoted strings.
    body = _dedent_body(body)
    for body_line in body.splitlines():
        lines.append("  " + body_line if body_line.strip() else "")
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def _compute_widths(fields: list[FieldSpec]) -> dict[str, int]:
    type_w = max((len(f.type) + (len("{" + ", ".join(map(str, f.enum)) + "}") if f.enum else 0)
                  for f in fields), default=8)
    name_w = max((len(f.name) for f in fields), default=8)
    default_w = max((len(_format_default(f.type, f.default)) for f in fields), default=8)
    return {"type": type_w, "name": name_w, "default": min(default_w, 40)}


def _dedent_body(body: str) -> str:
    import textwrap
    return textwrap.dedent(body)
