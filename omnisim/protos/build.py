"""Transpile Python PROTO sources (``Foo.proto.py``) to ``Foo.proto``.

The source file is executed in a clean module namespace; its single
:func:`omnisim.protogen.emit` call captures the spec, which is then
rendered to a sibling ``Foo.proto``. Output is deterministic, so a
re-run against an unchanged source is a no-op.

Run with ``python -m omnisim proto build [paths...]``. With no paths,
walks every ``*.proto.py`` under ``projects/``. ``--check`` reports
drift without writing — useful in CI.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
import uuid
from pathlib import Path

from ..paths import REPO_ROOT
from ..protogen.api import ProtoSpec, consume_last, render


def _load_spec(source: Path) -> ProtoSpec:
    """Execute a ``*.proto.py`` source and return the spec it emitted."""
    mod_name = f"_proto_py_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(mod_name, source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {source}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(mod_name, None)
    proto = consume_last()
    if proto is None:
        raise RuntimeError(f"{source}: source did not call omnisim.protogen.emit()")
    return proto


def _iter_sources(paths: list[str]) -> list[Path]:
    if not paths:
        roots = [REPO_ROOT / "projects"]
    else:
        roots = [Path(p) if Path(p).is_absolute() else REPO_ROOT / p for p in paths]
    out: list[Path] = []
    for r in roots:
        if r.is_file() and r.name.endswith(".proto.py"):
            out.append(r)
        elif r.is_dir():
            out.extend(sorted(r.rglob("*.proto.py")))
    return sorted(set(out))


def run(args: argparse.Namespace) -> int:
    sources = _iter_sources(args.paths)
    if not sources:
        print("no *.proto.py sources found", file=sys.stderr)
        return 0

    written: list[Path] = []
    unchanged: list[Path] = []
    drifted: list[Path] = []
    errors: list[tuple[Path, str]] = []

    for source in sources:
        target = source.with_name(source.name[: -len(".proto.py")] + ".proto")
        try:
            spec = _load_spec(source)
            rendered = render(spec)
        except Exception as e:  # noqa: BLE001
            errors.append((source, f"{type(e).__name__}: {e}"))
            continue
        on_disk = target.read_text(encoding="utf-8") if target.exists() else ""
        if rendered == on_disk:
            unchanged.append(target)
            continue
        if args.check:
            drifted.append(target)
            continue
        target.write_text(rendered, encoding="utf-8")
        written.append(target)

    print(f"proto build: {len(sources)} sources scanned")
    print(f"  written:   {len(written)}")
    print(f"  unchanged: {len(unchanged)}")
    if drifted:
        print(f"  drifted:   {len(drifted)} (run without --check to regenerate)")
        for d in drifted[:10]:
            print(f"    {d}")
    if errors:
        print(f"  errors:    {len(errors)}")
        for p, m in errors[:10]:
            print(f"    {p}: {m}")

    if args.check and drifted:
        return 2
    if errors:
        return 1
    return 0
