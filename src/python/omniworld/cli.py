"""Command-line interface for the omniworld library.

    omniworld --version
    omniworld list-recipes
    omniworld describe <recipe>
    omniworld generate <recipe> --seed N --out PATH [--param k=v ...]
                                [--skip-validate]
    omniworld validate <world.wbt>

The CLI is the validation surface everything else checks against. CI,
hand tests, and the (later-tier) agent tool all call these commands.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence

from . import __version__, get_recipe, list_recipes
from .core.api import generate
from .validation import validate


def _parse_param(raw: str) -> tuple[str, Any]:
    """Parse ``key=value``. ``value`` is JSON-decoded so ``--param size=5``
    gives an int and ``--param spawn_urdf=null`` gives ``None``."""
    if "=" not in raw:
        raise argparse.ArgumentTypeError(
            f"expected key=value for --param, got {raw!r}"
        )
    key, _, value_text = raw.partition("=")
    key = key.strip()
    if not key:
        raise argparse.ArgumentTypeError(f"empty key in --param {raw!r}")
    try:
        value = json.loads(value_text)
    except json.JSONDecodeError:
        # Fall back to raw string — lets users write ``--param name=foo``
        # without quoting, while still enabling ``--param x=3.14``.
        value = value_text
    return key, value


def _cmd_generate(args: argparse.Namespace) -> int:
    params: dict[str, Any] = {}
    for pair in args.param or []:
        key, value = _parse_param(pair)
        params[key] = value

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    result = generate(
        recipe=args.recipe,
        seed=args.seed,
        out=out_path,
        params=params,
    )
    print(f"wrote {result.world_path}")
    print(f"wrote {result.manifest_path}")
    print(f"sha256 {result.manifest.world_sha256}")

    if args.skip_validate:
        return 0

    report = validate(result.world_path, with_sim=args.with_sim)
    print(report.format())
    return 0 if report.ok else 1


def _cmd_validate(args: argparse.Namespace) -> int:
    report = validate(args.world, with_sim=args.with_sim)
    print(report.format())
    return 0 if report.ok else 1


def _cmd_list(args: argparse.Namespace) -> int:
    del args
    for name in list_recipes():
        print(name)
    return 0


def _cmd_describe(args: argparse.Namespace) -> int:
    recipe = get_recipe(args.recipe)
    payload = {
        "name": recipe.name,
        "default_params": recipe.default_params(),
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omniworld",
        description="OmniSim procedural world generator.",
    )
    parser.add_argument("--version", action="version", version=__version__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    p_gen = subparsers.add_parser("generate", help="generate a world")
    p_gen.add_argument("recipe", help="registered recipe name")
    p_gen.add_argument("--seed", type=int, required=True, help="parent seed")
    p_gen.add_argument("--out", required=True, help="output .wbt path")
    p_gen.add_argument(
        "--param",
        action="append",
        metavar="KEY=VALUE",
        help="override a recipe parameter (JSON value; repeatable)",
    )
    p_gen.add_argument(
        "--skip-validate",
        action="store_true",
        help="skip post-generation validation (debugging only)",
    )
    p_gen.add_argument(
        "--with-sim",
        action="store_true",
        help="run the headless-simulator validator (requires OmniSim)",
    )
    p_gen.set_defaults(func=_cmd_generate)

    p_val = subparsers.add_parser("validate", help="validate an existing .wbt")
    p_val.add_argument("world", help="path to the .wbt world file")
    p_val.add_argument(
        "--with-sim",
        action="store_true",
        help="run the headless-simulator validator (requires OmniSim)",
    )
    p_val.set_defaults(func=_cmd_validate)

    p_list = subparsers.add_parser("list-recipes", help="print registered recipes")
    p_list.set_defaults(func=_cmd_list)

    p_desc = subparsers.add_parser(
        "describe", help="print a recipe's default parameter set"
    )
    p_desc.add_argument("recipe", help="registered recipe name")
    p_desc.set_defaults(func=_cmd_describe)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
