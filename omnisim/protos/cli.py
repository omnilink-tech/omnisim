"""``python -m omnisim proto`` subcommand dispatch.

Subcommands are wired here and implemented in sibling modules. Each
handler returns an int exit code so the top-level CLI can propagate it.
"""

from __future__ import annotations

import argparse
from typing import Callable


def _schema(args: argparse.Namespace) -> int:
    from . import schema as _schema_mod
    return _schema_mod.run(args)


def _validate(args: argparse.Namespace) -> int:
    from . import validate as _validate_mod
    return _validate_mod.run(args)


def _build(args: argparse.Namespace) -> int:
    from . import build as _build_mod
    return _build_mod.run(args)


def _reload(args: argparse.Namespace) -> int:
    from . import reload as _reload_mod
    return _reload_mod.run(args)


def _test(args: argparse.Namespace) -> int:
    from . import test as _test_mod
    return _test_mod.run(args)


def _manifest(args: argparse.Namespace) -> int:
    from . import manifest as _manifest_mod
    return _manifest_mod.run(args)


def _add_paths_arg(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "paths",
        nargs="*",
        help=(
            "PROTO files or directories to operate on. "
            "Defaults to all PROTOs under projects/ if omitted."
        ),
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omnisim proto",
        description="OmniSim PROTO tooling (schemas, validation, authoring, hot-reload, tests).",
    )
    sub = parser.add_subparsers(dest="proto_command", required=True, metavar="<subcommand>")

    p = sub.add_parser("schema", help="Generate .proto.yaml schema sidecars from PROTO sources.")
    _add_paths_arg(p)
    p.add_argument("--check", action="store_true",
                   help="Don't write; fail if any sidecar is missing or stale.")
    p.add_argument("--force", action="store_true",
                   help="Overwrite existing sidecars even if hand-edited.")
    p.add_argument("--json", action="store_true", help="Emit JSON summary.")
    p.set_defaults(func=_schema)

    p = sub.add_parser("validate", help="Validate PROTOs against their .proto.yaml schemas.")
    _add_paths_arg(p)
    p.add_argument("--json", action="store_true", help="Emit structured JSON errors.")
    p.add_argument("--strict", action="store_true",
                   help="Fail if a PROTO is missing a sidecar (default: warn).")
    p.set_defaults(func=_validate)

    p = sub.add_parser("build", help="Transpile Python PROTO sources (*.proto.py) to .proto.")
    _add_paths_arg(p)
    p.add_argument("--check", action="store_true",
                   help="Don't write; fail if any .proto is missing or stale.")
    p.set_defaults(func=_build)

    p = sub.add_parser("reload", help="Drive supervisor worldLoad to hot-reload a running world.")
    p.add_argument("--world", help="World file to (re)load. Defaults to the running world.")
    p.add_argument("--watch", action="store_true",
                   help="Watch PROTO sources and reload on change.")
    p.add_argument("--interval", type=float, default=1.0,
                   help="Watch poll interval in seconds (default: 1.0).")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=6789, help="Harness HTTP port.")
    p.set_defaults(func=_reload)

    p = sub.add_parser("test", help="Headlessly instantiate every PROTO and report failures.")
    _add_paths_arg(p)
    p.add_argument("--json", action="store_true")
    p.add_argument("--timeout", type=int, default=20,
                   help="Per-PROTO timeout seconds (default: 20).")
    p.set_defaults(func=_test)

    p = sub.add_parser("manifest", help="Inspect / validate PROTO-set manifests (omnisim.yaml).")
    sub_m = p.add_subparsers(dest="manifest_command", required=True)
    pm = sub_m.add_parser("check", help="Validate every PROTO-set manifest under projects/.")
    pm.add_argument("--json", action="store_true")
    pm = sub_m.add_parser("resolve", help="Resolve a PROTO's manifest chain (deps, version).")
    pm.add_argument("proto", help="PROTO file or DEF name.")
    pm.add_argument("--json", action="store_true")
    p.set_defaults(func=_manifest)

    return parser


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    func: Callable[[argparse.Namespace], int] = args.func
    return func(args)
