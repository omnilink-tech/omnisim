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


def _policy(argv: list[str]) -> int:
    from .policy.cli import main as policy_main
    return policy_main(argv)


def _agent(argv: list[str]) -> int:
    from .agent.cli import main as agent_main
    return agent_main(argv)


def _key(argv: list[str]) -> int:
    from .key import main as key_main
    return key_main(argv)


def _byok(argv: list[str]) -> int:
    from .provider_key import main as byok_main
    return byok_main(argv)


def _doctor(args: argparse.Namespace) -> int:
    from . import doctor
    extra = []
    if args.json:
        extra.append("--json")
    if getattr(args, "fingerprint", False):
        extra.append("--fingerprint")
    if getattr(args, "strict", False):
        extra.append("--strict")
    return doctor.run(extra)


def _verify_install(args: argparse.Namespace) -> int:
    from .conformance.cli import run as run_verify
    return run_verify(args)


def _build(args: argparse.Namespace) -> int:
    return dev.build(
        args.target,
        mode=args.mode,
        jobs=args.jobs,
        base=args.base,
        link=args.link,
        staged=args.staged,
    )


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


def _validate_worlds(args: argparse.Namespace) -> int:
    """Batch load-check: N worlds through K reused engines.

    Thin passthrough to scripts/dev/batch_validate.py, which owns the logic.
    Exposed on the CLI because the whole point is that any lane or agent can
    reach it -- a tool nobody can find gets used by nobody.
    """
    cmd = [sys.executable, str(REPO_ROOT / "scripts" / "dev" / "batch_validate.py")]
    cmd += list(args.worlds or [])
    if args.from_file:
        cmd += ["--from-file", args.from_file]
    if args.glob:
        cmd += ["--glob", args.glob]
    if args.jobs != 1:
        cmd += ["--jobs", str(args.jobs)]
    if args.heavy:
        cmd.append("--heavy")
    if args.json_out:
        cmd += ["--json", args.json_out]
    return subprocess.run(cmd).returncode


def _harness(args: argparse.Namespace) -> int:
    return dev.harness(host=args.host, port=args.port, supervisor_port=args.supervisor_port,
                       auto_port=getattr(args, "auto_port", False))


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
    p.add_argument(
        "--strict",
        action="store_true",
        help="Tier 0 install-coherence gate: exit non-zero if the install is "
        "incoherent (engine/libController ABI mismatch, missing binary). Preflight "
        "for hooks / CI / launchers; plain doctor is unchanged and always exits 0.",
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
    p.add_argument("--base", default="HEAD",
                   help="Git base for `build changed` (default: working tree vs HEAD).")
    p.add_argument("--link", action="store_true",
                   help="For `build changed`, link after affected objects compile.")
    p.add_argument("--staged", action="store_true",
                   help="Link to omnisim-bin.next.exe instead of replacing the live binary.")
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
    # default None, NOT 10: the absence of --duration is meaningful. The runner
    # reads it as "this is a load check" and stops at Newton finalize instead of
    # sleeping (measured 15.52s -> 6.37s). Hard-coding 10 here made that
    # impossible to express -- the child could never tell a caller who wanted
    # ten seconds from one who just did not say.
    p.add_argument("--duration", type=int, default=None,
                   help="Seconds to run. Omit for a load check (stops at Newton "
                        "finalize); pass a value to observe the sim for that long.")
    p.add_argument("--fail-on-warning", action="store_true", help="Treat warnings as failures.")
    # Everything else is forwarded verbatim to scripts/dev/headless_runner.py,
    # which owns the full flag set. The ones worth knowing about:
    #   --fail-on-runaway   also FAIL when a body has left the world (a floor with
    #                       no boundingObject); a log-only PASS cannot see that.
    #   --port=N            pin the extern-controller port (needed for parallel runs)
    #   --gui / --realtime  watch the run in a real window at 1x
    #   --wait-for-step     start the duration clock at the first physics step
    p.add_argument("extra_args", nargs=argparse.REMAINDER,
                   help="Forwarded to headless_runner.py: --fail-on-runaway, --port=N, "
                        "--gui, --realtime, --wait-for-step, --require-newton, ...")
    p.set_defaults(func=_run_headless)

    p = sub.add_parser("validate-worlds",
                       help="Load-check MANY worlds through reused engines (fast batch).")
    p.add_argument("worlds", nargs="*", help="World paths.")
    p.add_argument("--from-file", help="Read world paths from a file, one per line.")
    p.add_argument("--glob",
                   help="Glob for worlds, e.g. 'projects/**/worlds/*.omniworld'. "
                        "Both .omniworld and the legacy .wbt are accepted.")
    p.add_argument("--jobs", "-j", type=int, default=1,
                   help="Engines to run in parallel (default 1; 4 is the measured "
                        "sweet spot on a 16-core box, 6 is already slower).")
    p.add_argument("--heavy", action="store_true",
                   help="Keep the per-step contact/joint/grip trackers (default: light).")
    p.add_argument("--json", dest="json_out", help="Write per-world results as JSON.")
    p.set_defaults(func=_validate_worlds)

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
    p.add_argument(
        "--auto-port",
        action="store_true",
        help=(
            "If the requested (port, port+1) pair is in use, scan upward for a free "
            "pair and bind there instead of failing. The chosen ports are printed to "
            "stderr so a caller can discover the actual address."
        ),
    )
    p.set_defaults(func=_harness)

    sub.add_parser("damage", help="Run a damage-system scenario headless (--scenario / --list / --world).")
    sub.add_parser("damage-regression", help="Run the damage-system numerical regression suite.")
    sub.add_parser("capture", help="Forward to scripts/capture/omnisim_capture.py.")
    sub.add_parser("cinema", help="Agent-driven cinematic capture pipeline (storyboards, looks, multi-aspect).")
    sub.add_parser("run-agent", help="Launch an OmniSim world + its OmniLink agent runner together.")
    sub.add_parser("proto", help="PROTO tooling: schemas, validation, authoring, hot-reload, tests.")
    sub.add_parser("policy", help="Policy brain: skills, BATON graphs, benchmarks, and promotion.")
    sub.add_parser("agent", help="Create and manage OmniLink agent projects.")
    sub.add_parser("key", help="Get, set and check your OmniLink Omni Key.")
    sub.add_parser("byok", help="Connect the model provider that pays for the tokens.")

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
    if argv and argv[0] == "policy":
        return _policy(argv[1:])
    if argv and argv[0] == "agent":
        return _agent(argv[1:])
    if argv and argv[0] == "key":
        return _key(argv[1:])
    if argv and argv[0] == "byok":
        return _byok(argv[1:])
    args = _build_parser().parse_args(argv)
    return args.func(args)
