"""OmniSim CLI — `python -m omnisim ...`.

The canonical agent-facing entry point. AGENTS.md S0 directs agents here.

Build / test / run / profile / harness subcommands are handled in-process
via `omnisim.dev.commands`. `damage` and `damage-regression` are handled
in-process via `omnisim.damage`. `capture` still forwards to its script
runner pending its own migration. `doctor` is the first-turn ground-truth
command (see `omnisim.doctor`).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from . import __version__
from .dev import commands as dev
from .paths import REPO_ROOT


_TEST_GROUPS = ("api", "cache", "other_api", "physics", "protos", "parser", "rendering")
_CAPTURE_SCRIPT = REPO_ROOT / "scripts" / "capture" / "omnisim_capture.py"
_RUN_AGENT_SCRIPT = REPO_ROOT / "scripts" / "dev" / "omnisim_run_agent.py"


def _forward(script: Path, argv: list[str]) -> int:
    cmd = [sys.executable, str(script), *argv]
    return subprocess.run(cmd, cwd=REPO_ROOT, env=os.environ.copy(), check=False).returncode


def _damage(argv: list[str]) -> int:
    from .damage.headless_test import main as damage_main
    return damage_main(argv)


def _damage_regression(argv: list[str]) -> int:
    from .damage.regression_suite import main as regression_main
    return regression_main(argv)


def _proto(argv: list[str]) -> int:
    from .protos import main as proto_main
    return proto_main(argv)


def _doctor(args: argparse.Namespace) -> int:
    from . import doctor
    extra = []
    if args.json:
        extra.append("--json")
    if getattr(args, "fingerprint", False):
        extra.append("--fingerprint")
    return doctor.run(extra)


def _verify_install(args: argparse.Namespace) -> int:
    from .conformance.cli import run as run_verify
    return run_verify(args)


def _build(args: argparse.Namespace) -> int:
    return dev.build(args.target, mode=args.mode, jobs=args.jobs)


def _test_smoke(args: argparse.Namespace) -> int:
    return dev.test_smoke(nomake=args.nomake)


def _test_group(args: argparse.Namespace) -> int:
    return dev.test_group(args.group, nomake=args.nomake)


def _test_world(args: argparse.Namespace) -> int:
    return dev.test_world(args.world, nomake=args.nomake)


def _profile_world(args: argparse.Namespace) -> int:
    return dev.profile_world(args.world, log_path=args.log)


def _benchmarks(args: argparse.Namespace) -> int:
    return dev.benchmarks(nomake=args.nomake)


def _compile_commands(_: argparse.Namespace) -> int:
    return dev.compile_commands()


def _run_world(args: argparse.Namespace) -> int:
    # First-run conformance gate (Phase 3): mandatory-but-bypassable on the
    # interactive launch path, fails OPEN. Clears instantly once a valid stamp
    # exists for this build. See docs/developer/install-conformance.md §2.
    from .conformance.gate import gate
    if gate(args.extra_args) != 0:
        return 1
    return dev.run_world(args.world, args.extra_args)


def _run_headless(args: argparse.Namespace) -> int:
    return dev.run_headless(
        args.world,
        duration=args.duration,
        fail_on_warning=args.fail_on_warning,
        extra_args=args.extra_args,
    )


def _harness(args: argparse.Namespace) -> int:
    return dev.harness(host=args.host, port=args.port, supervisor_port=args.supervisor_port)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omnisim",
        description="OmniSim - agent-driven robot simulation (built on Webots).",
    )
    parser.add_argument("-V", "--version", action="version", version=f"omnisim {__version__}")
    sub = parser.add_subparsers(dest="command", required=True, metavar="<command>")

    p = sub.add_parser("doctor", help="Print repo / runtime ground truth (AGENTS.md S0).")
    p.add_argument("--json", action="store_true", help="Emit JSON instead of text.")
    p.add_argument(
        "--fingerprint",
        action="store_true",
        help="Also capture the config fingerprint (resolved physics backend, "
        "Newton-runtime presence, GPU, fingerprint_id).",
    )
    p.set_defaults(func=_doctor)

    p = sub.add_parser(
        "verify-install",
        help="Run the install conformance self-test (non-gating reporter).",
    )
    p.add_argument("--fast", action="store_true",
                   help="Run only the fast subset (bit-exact canaries + load liveness).")
    p.add_argument("--deep", action="store_true",
                   help="Opt-in deep lane (Phase 4; never gates an install).")
    p.add_argument("--json", action="store_true",
                   help="Emit JSON (schema omnisim.install_check/v1).")
    p.add_argument("--report", action="store_true",
                   help="Also write a scrubbed {json,md} report bundle under install-reports/.")
    p.add_argument("--strict", action="store_true",
                   help="Promote PASS-WITH-DRIFT to a non-zero exit (for CI lanes).")
    p.add_argument("--fingerprint-only", action="store_true",
                   help="Print just the config fingerprint and exit.")
    p.add_argument("--gate", action="store_true",
                   help="Run the gate: fast lane unless a valid stamp / escape "
                        "condition holds; non-zero exit on FAIL. Used by the launcher.")
    p.add_argument("--post-build", action="store_true",
                   help="Gate semantics but advisory -- never a non-zero exit "
                        "(pre-warms the stamp after a build).")
    p.add_argument("--never-ask", action="store_true",
                   help="Disable the first-run gate for this clone (writes a skip file).")
    p.add_argument("--calibrate", action="store_true",
                   help="Run each SOFT-metric demo N times and write calibration.json "
                        "(within-host band calibration).")
    p.add_argument("--runs", type=int, default=3,
                   help="Calibration runs per demo (default: 3).")
    p.set_defaults(func=_verify_install)

    p = sub.add_parser("build", help="Build a logical target.")
    p.add_argument("target", choices=dev.BUILD_TARGETS)
    p.add_argument("--mode", default="release", choices=("release", "debug", "profile"))
    p.add_argument("--jobs", type=int, default=0)
    p.set_defaults(func=_build)

    p = sub.add_parser("test-smoke", help="Run the fast smoke suite.")
    p.add_argument("--nomake", action="store_true")
    p.set_defaults(func=_test_smoke)

    p = sub.add_parser("test-group", help="Run one existing test-suite group.")
    p.add_argument("group", choices=_TEST_GROUPS)
    p.add_argument("--nomake", action="store_true")
    p.set_defaults(func=_test_group)

    p = sub.add_parser("test-world", help="Run one world through the existing test suite.")
    p.add_argument("world")
    p.add_argument("--nomake", action="store_true")
    p.set_defaults(func=_test_world)

    p = sub.add_parser("run-world", help="Launch the simulator (GUI) on one world.")
    p.add_argument("world")
    p.add_argument("extra_args", nargs=argparse.REMAINDER)
    p.set_defaults(func=_run_world)

    p = sub.add_parser("run-headless", help="Run one world headlessly with monitoring and timeout.")
    p.add_argument("world")
    p.add_argument("--duration", type=int, default=10, help="Seconds to run (default: 10).")
    p.add_argument("--fail-on-warning", action="store_true", help="Treat warnings as failures.")
    p.add_argument("extra_args", nargs=argparse.REMAINDER)
    p.set_defaults(func=_run_headless)

    p = sub.add_parser("profile-world", help="Run one world and collect performance logs.")
    p.add_argument("world")
    p.add_argument("--log", default="")
    p.set_defaults(func=_profile_world)

    p = sub.add_parser("benchmarks", help="Run the benchmark world set.")
    p.add_argument("--nomake", action="store_true")
    p.set_defaults(func=_benchmarks)

    p = sub.add_parser("compile-commands", help="Generate compile_commands.json with bear.")
    p.set_defaults(func=_compile_commands)

    p = sub.add_parser("harness", help="Start the agent-facing validation harness (HTTP, foreground).")
    p.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1).")
    p.add_argument("--port", type=int, default=6789, help="Bind port (default: 6789).")
    p.add_argument(
        "--supervisor-port",
        type=int,
        default=None,
        help=(
            "Supervisor IPC port inside the OmniSim subprocess "
            "(default: --port + 1). Pass to run a second harness "
            "alongside an existing one."
        ),
    )
    p.set_defaults(func=_harness)

    sub.add_parser("damage", help="Run a damage-system scenario headless (--scenario / --list / --world).")
    sub.add_parser("damage-regression", help="Run the damage-system numerical regression suite.")
    sub.add_parser("capture", help="Forward to scripts/capture/omnisim_capture.py.")
    sub.add_parser("cinema", help="Agent-driven cinematic capture pipeline (storyboards, looks, multi-aspect).")
    sub.add_parser("run-agent", help="Launch an OmniSim world + its OmniLink agent runner together.")
    sub.add_parser("proto", help="PROTO tooling: schemas, validation, authoring, hot-reload, tests.")

    return parser


def main() -> int:
    argv = sys.argv[1:]
    # Pre-route subcommands that own their own argparse so `--help` and
    # other flags reach the underlying handler instead of being consumed
    # by the top-level parser here.
    if argv and argv[0] == "damage":
        return _damage(argv[1:])
    if argv and argv[0] == "damage-regression":
        return _damage_regression(argv[1:])
    if argv and argv[0] == "capture":
        return _forward(_CAPTURE_SCRIPT, argv[1:])
    if argv and argv[0] == "cinema":
        # Import lazily so `omnisim --help` doesn't pay for the cinema deps.
        import sys as _sys
        scripts_dir = str(REPO_ROOT / "scripts")
        if scripts_dir not in _sys.path:
            _sys.path.insert(0, scripts_dir)
        from cinema.cli import main as cinema_main
        return cinema_main(argv[1:])
    if argv and argv[0] == "run-agent":
        return _forward(_RUN_AGENT_SCRIPT, argv[1:])
    if argv and argv[0] == "proto":
        return _proto(argv[1:])
    args = _build_parser().parse_args(argv)
    return args.func(args)
