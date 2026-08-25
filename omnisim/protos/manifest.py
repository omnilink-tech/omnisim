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

"""Inspect / validate PROTO-set manifests (``omnisim.yaml``).

OmniSim's PROTO-set manifest is ``omnisim.yaml``. The schema extends the
upstream-Webots ``publish: true`` flag with optional metadata so
agents can answer "what is this set, who owns it, what version is it,
what does it depend on" without reading 90+ files by hand. All new
keys are optional — pre-existing manifests stay valid.

Extended schema::

    publish: true
    name: solids                    # stable slug (kebab/snake-case)
    display_name: Solid Primitives  # human-readable title
    description: |-                 # short prose
      Solid-node primitives (box, pipe, rounded box, torus) with
      inline boundingObject geometry for ODE collision.
    version: 0.1.0                  # semver
    license: ...                    # default license for PROTOs in this set
    homepage: https://...           # URL
    maintainers: [...]              # list of names or emails
    requires_webots: R2025a         # min Webots release tag

Subcommands:

* ``manifest check`` — validate every manifest under ``projects/``.
* ``manifest resolve <proto>`` — walk a PROTO's ``EXTERNPROTO`` graph
  and report each dependency's owning set.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass, field as dc_field
from pathlib import Path
from typing import Any

import yaml

from ..paths import REPO_ROOT
from .parser import parse_proto_file

MANIFEST_FILENAME = "omnisim.yaml"


_RECOGNIZED_KEYS = frozenset({
    "publish", "name", "display_name", "description", "version", "license",
    "homepage", "maintainers", "requires_webots", "tags",
})

_SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+(?:[+-][\w.\-]+)?$")
_SLUG_RE = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass
class ManifestSpec:
    path: Path
    publish: bool = True
    name: str = ""
    display_name: str = ""
    description: str = ""
    version: str = ""
    license: str = ""
    homepage: str = ""
    maintainers: list[str] = dc_field(default_factory=list)
    requires_webots: str = ""
    tags: list[str] = dc_field(default_factory=list)
    unknown_keys: list[str] = dc_field(default_factory=list)


def load_manifest(path: Path) -> ManifestSpec:
    raw: dict[str, Any] = {}
    if path.exists():
        with path.open("r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        if isinstance(data, dict):
            raw = data
    spec = ManifestSpec(path=path)
    spec.publish = bool(raw.get("publish", True))
    spec.name = str(raw.get("name", "") or "")
    spec.display_name = str(raw.get("display_name", "") or "")
    spec.description = str(raw.get("description", "") or "")
    spec.version = str(raw.get("version", "") or "")
    spec.license = str(raw.get("license", "") or "")
    spec.homepage = str(raw.get("homepage", "") or "")
    maintainers = raw.get("maintainers") or []
    spec.maintainers = [str(m) for m in maintainers] if isinstance(maintainers, list) else []
    spec.requires_webots = str(raw.get("requires_webots", "") or "")
    tags = raw.get("tags") or []
    spec.tags = [str(t) for t in tags] if isinstance(tags, list) else []
    spec.unknown_keys = sorted(set(raw) - _RECOGNIZED_KEYS)
    return spec


# --------------------------------------------------------------------------- #
# Validation
# --------------------------------------------------------------------------- #


@dataclass
class ManifestFinding:
    path: str
    code: str
    severity: str
    message: str
    key: str | None = None
    suggestion: str | None = None


def validate_manifest(spec: ManifestSpec) -> list[ManifestFinding]:
    findings: list[ManifestFinding] = []
    rel = str(spec.path)
    for key in spec.unknown_keys:
        findings.append(ManifestFinding(
            path=rel, code="MANIFEST-UNKNOWN-KEY", severity="warning",
            message=f"unrecognized key {key!r}", key=key,
            suggestion=f"Allowed keys: {sorted(_RECOGNIZED_KEYS)}",
        ))
    if spec.name and not _SLUG_RE.match(spec.name):
        findings.append(ManifestFinding(
            path=rel, code="MANIFEST-BAD-NAME", severity="error",
            message=f"name {spec.name!r} is not a valid slug (lowercase, underscores)",
            key="name",
        ))
    if spec.version and not _SEMVER_RE.match(spec.version):
        findings.append(ManifestFinding(
            path=rel, code="MANIFEST-BAD-VERSION", severity="error",
            message=f"version {spec.version!r} is not semver (X.Y.Z[-pre][+build])",
            key="version",
        ))
    return findings


def _iter_manifests() -> list[Path]:
    return sorted((REPO_ROOT / "projects").rglob(MANIFEST_FILENAME))


# --------------------------------------------------------------------------- #
# Resolution
# --------------------------------------------------------------------------- #


def find_set_for_proto(proto_path: Path) -> Path | None:
    """Walk up from ``proto_path`` until we find the nearest ``omnisim.yaml``."""
    p = proto_path.resolve().parent
    repo_root = REPO_ROOT.resolve()
    while p != p.parent:
        manifest = p / MANIFEST_FILENAME
        if manifest.exists():
            return manifest
        if p == repo_root:
            return None
        p = p.parent
    return None


def resolve_proto_url(url: str, source_proto: Path) -> Path | None:
    """Resolve a ``omnisim://`` or relative URL into a real PROTO path."""
    if url.startswith("omnisim://"):
        rel = url[len("omnisim://"):]
        candidate = REPO_ROOT / rel
    else:
        candidate = source_proto.parent / url
    return candidate if candidate.exists() else None


def resolve_proto(proto_path: Path) -> dict[str, Any]:
    """Build a dependency report for one PROTO.

    Returns a dict with the owning set, immediate EXTERNPROTOs, and the
    set each one resolves to.
    """
    parsed = parse_proto_file(proto_path)
    own_set = find_set_for_proto(proto_path)
    own_spec = load_manifest(own_set) if own_set else None

    deps: list[dict[str, Any]] = []
    for url in parsed.extern_protos:
        target = resolve_proto_url(url, proto_path)
        target_set = find_set_for_proto(target) if target else None
        target_spec = load_manifest(target_set) if target_set else None
        deps.append({
            "url": url,
            "resolved_path": str(target) if target else None,
            "resolved": target is not None,
            "set": str(target_set) if target_set else None,
            "set_name": target_spec.name if target_spec else None,
            "set_version": target_spec.version if target_spec else None,
        })
    return {
        "proto": parsed.name,
        "path": str(proto_path),
        "set": str(own_set) if own_set else None,
        "set_name": own_spec.name if own_spec else None,
        "set_version": own_spec.version if own_spec else None,
        "dependencies": deps,
        "unresolved": [d for d in deps if not d["resolved"]],
    }


# --------------------------------------------------------------------------- #
# CLI entry
# --------------------------------------------------------------------------- #


def run(args: argparse.Namespace) -> int:
    if args.manifest_command == "check":
        return _run_check(args)
    if args.manifest_command == "resolve":
        return _run_resolve(args)
    print(f"unknown manifest subcommand: {args.manifest_command}", file=sys.stderr)
    return 1


def _run_check(args: argparse.Namespace) -> int:
    manifests = _iter_manifests()
    all_findings: list[ManifestFinding] = []
    specs: list[ManifestSpec] = []
    for m in manifests:
        spec = load_manifest(m)
        specs.append(spec)
        all_findings.extend(validate_manifest(spec))

    errors = [f for f in all_findings if f.severity == "error"]
    warnings = [f for f in all_findings if f.severity == "warning"]

    if args.json:
        print(json.dumps({
            "manifests": len(manifests),
            "errors": len(errors),
            "warnings": len(warnings),
            "findings": [asdict(f) for f in all_findings],
            "sets": [{"path": str(s.path), "name": s.name, "version": s.version,
                      "publish": s.publish} for s in specs],
        }, indent=2))
    else:
        print(f"proto manifest check: {len(manifests)} manifests scanned")
        for f in all_findings:
            print(f"  {f.severity.upper()} {f.code} {f.path}: {f.message}")
        print(f"  errors: {len(errors)}, warnings: {len(warnings)}")

    return 1 if errors else 0


def _run_resolve(args: argparse.Namespace) -> int:
    proto = Path(args.proto)
    if not proto.is_absolute():
        proto = REPO_ROOT / proto
    if not proto.exists():
        print(f"PROTO not found: {proto}", file=sys.stderr)
        return 1
    report = resolve_proto(proto)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"PROTO: {report['proto']} ({report['path']})")
        print(f"set:   {report['set_name'] or '(unnamed)'} @ {report['set_version'] or '?'}  ({report['set']})")
        print(f"dependencies ({len(report['dependencies'])}):")
        for d in report["dependencies"]:
            mark = "OK " if d["resolved"] else "?? "
            print(f"  {mark}{d['url']}")
            if d["resolved"]:
                print(f"        -> {d['set_name'] or '(unnamed)'} @ {d['set_version'] or '?'}  ({d['set']})")
            else:
                print(f"        -> UNRESOLVED")
    return 0 if not report["unresolved"] else 2
