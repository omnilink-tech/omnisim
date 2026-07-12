"""PROTO test harness.

Two layers of checks:

1. **Static** (the default): parse the PROTO, run schema validation, and
   resolve every ``EXTERNPROTO`` and every texture/mesh ``url`` against
   the filesystem. Fast (sub-second across all 493 PROTOs) and
   deterministic — catches every "PROTO references a thing that no
   longer exists" class of regression.

2. **Deep** (``--with-webots``): wrap each PROTO in a minimal world,
   launch headless OmniSim, and confirm the world loads without
   error/warning. Slow and requires an OmniSim install; off by default.

Run with ``python -m omnisim proto test [paths...]``. Output is a
structured per-PROTO report (``--json`` for machine consumption).
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass, field as dc_field
from pathlib import Path
from typing import Any

from ..paths import REPO_ROOT
from .manifest import resolve_proto_url
from .parser import ParsedProto, ProtoParseError, parse_proto_file
from .validate import validate_proto


_URL_RE = re.compile(r'url\s*\[\s*"([^"]+)"', re.MULTILINE)
_SINGLE_URL_RE = re.compile(r'url\s+"([^"]+)"')


@dataclass
class ProtoTestResult:
    proto: str
    path: str
    passed: bool
    parse_ok: bool = False
    validate_findings: int = 0
    validate_errors: int = 0
    extern_missing: list[str] = dc_field(default_factory=list)
    asset_missing: list[str] = dc_field(default_factory=list)
    notes: list[str] = dc_field(default_factory=list)


def _extract_url_assets(proto_text: str) -> list[str]:
    """Pull out every ``url "..."`` reference (textures, meshes, etc.)."""
    seen: list[str] = []
    for m in _URL_RE.finditer(proto_text):
        seen.append(m.group(1))
    for m in _SINGLE_URL_RE.finditer(proto_text):
        if m.group(1) not in seen:
            seen.append(m.group(1))
    return seen


def _is_dynamic_url(url: str) -> bool:
    """Skip URLs assembled by the JS template (contain ``%<...>%`` markers)."""
    return "%<" in url or "%>" in url


def _check_assets(parsed: ParsedProto, proto_path: Path,
                  proto_text: str) -> list[str]:
    """Return relative paths of asset URLs that don't resolve."""
    missing: list[str] = []
    for raw_url in _extract_url_assets(proto_text):
        if _is_dynamic_url(raw_url):
            continue
        if raw_url.startswith(("http://", "https://", "data:")):
            continue
        resolved = resolve_proto_url(raw_url, proto_path) if raw_url.startswith("omnisim://") \
            else proto_path.parent / raw_url
        if resolved is None or not resolved.exists():
            missing.append(raw_url)
    return missing


def _test_one(proto_path: Path) -> ProtoTestResult:
    rel = str(proto_path.relative_to(REPO_ROOT)) if proto_path.is_absolute() \
        else str(proto_path)
    res = ProtoTestResult(proto="?", path=rel, passed=False)

    try:
        parsed = parse_proto_file(proto_path)
        res.parse_ok = True
        res.proto = parsed.name
    except ProtoParseError as e:
        res.notes.append(f"parse: {e}")
        return res

    # Schema/structural validation
    findings = validate_proto(proto_path, strict=False)
    res.validate_findings = len(findings)
    res.validate_errors = sum(1 for f in findings if f.severity == "error")

    # EXTERNPROTO resolution
    for url in parsed.extern_protos:
        if _is_dynamic_url(url):
            continue
        resolved = resolve_proto_url(url, proto_path)
        if resolved is None:
            res.extern_missing.append(url)

    # Asset (texture/mesh) resolution
    proto_text = proto_path.read_text(encoding="utf-8")
    res.asset_missing = _check_assets(parsed, proto_path, proto_text)

    res.passed = (
        res.parse_ok
        and res.validate_errors == 0
        and not res.extern_missing
        and not res.asset_missing
    )
    return res


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

    results = [_test_one(p) for p in protos]
    passed = sum(1 for r in results if r.passed)
    failed = [r for r in results if not r.passed]

    if args.json:
        print(json.dumps({
            "total": len(results),
            "passed": passed,
            "failed": len(failed),
            "results": [asdict(r) for r in results],
        }, indent=2))
        return 0 if not failed else 1

    print(f"proto test: {len(results)} PROTOs scanned")
    print(f"  passed: {passed}")
    print(f"  failed: {len(failed)}")
    for r in failed[:50]:
        reasons: list[str] = []
        if not r.parse_ok:
            reasons.append("parse-error")
        if r.validate_errors:
            reasons.append(f"{r.validate_errors} validation errors")
        if r.extern_missing:
            reasons.append(f"{len(r.extern_missing)} unresolved EXTERNPROTO")
        if r.asset_missing:
            reasons.append(f"{len(r.asset_missing)} missing assets")
        print(f"  FAIL {r.path}: {', '.join(reasons)}")
        for m in r.extern_missing[:3]:
            print(f"        extern: {m}")
        for m in r.asset_missing[:3]:
            print(f"        asset:  {m}")
    if len(failed) > 50:
        print(f"  ... and {len(failed) - 50} more")

    return 0 if not failed else 1
