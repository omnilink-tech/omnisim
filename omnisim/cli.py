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


_DEMO_CATALOGUE = (
    REPO_ROOT / "projects" / "samples" / "demos" / "controllers"
    / "omnilink_launcher" / "demos.json"
)
# The demo README, BETA and the guided tour all lead with. demos.json is the
# in-sim gallery's manifest and did not carry it, so the front-page demo was
# absent from every catalogue in the tree.
_DEFAULT_DEMO = "omniarm6_real_pick_place"


def _console_safe(text: str) -> str:
    """Drop characters the active console cannot encode.

    demos.json uses typographic characters (a middle dot in every "Chat . X"
    label). A default Windows console is cp1252, and a diagnostic that
    UnicodeEncodeErrors while listing the demos is worse than a plain one.
    """
    enc = getattr(sys.stdout, "encoding", None) or "ascii"
    return text.encode(enc, "replace").decode(enc, "replace")


def _load_demos() -> list[dict]:
    """Flatten demos.json into [{id, name, world, blurb, category}].

    Returns [] rather than raising: a missing or malformed catalogue must
    degrade to "no demos listed", never to a traceback in front of someone
    running their first command.
    """
    import json
    try:
        doc = json.loads(_DEMO_CATALOGUE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    out: list[dict] = []
    for cat in doc.get("categories", []):
        for demo in cat.get("demos", []):
            entry = dict(demo)
            entry["category"] = cat.get("label", cat.get("id", ""))
            out.append(entry)
    return out


def _demos(args: argparse.Namespace) -> int:
    demos = _load_demos()
    if not demos:
        print("No demo catalogue found at %s" % _DEMO_CATALOGUE, file=sys.stderr)
        return 1
    missing = 0
    current = None
    for demo in demos:
        if demo["category"] != current:
            current = demo["category"]
            print("")
            print(_console_safe(current))
        world = REPO_ROOT / demo["world"]
        present = world.exists()
        missing += 0 if present else 1
        print(_console_safe("  %s %-28s %s"
                            % (" " if present else "!", demo["id"], demo.get("name", ""))))
        if args.verbose and demo.get("blurb"):
            print(_console_safe("      %s" % demo["blurb"]))
    print("")
    print("%d demos.  Run one:  python -m omnisim demo <id>" % len(demos))
    if missing:
        print("! %d listed world(s) are not present in this install." % missing)
    return 0


def _demo(args: argparse.Namespace) -> int:
    demos = _load_demos()
    wanted = args.id or _DEFAULT_DEMO
    match = next((d for d in demos if d["id"] == wanted), None)
    if match is None and args.id is None:
        # The default is allowed not to be in the catalogue: resolve it by path
        # so `omnisim demo` works even on a tree whose manifest is behind.
        world = (
            REPO_ROOT / "projects" / "samples" / "demos" / "worlds"
            / "flagship" / (_DEFAULT_DEMO + ".omniworld")
        )
        if world.exists():
            match = {
                "id": _DEFAULT_DEMO,
                "name": "OmniArm 6 real pick and place",
                "world": world.relative_to(REPO_ROOT).as_posix(),
            }
    if match is None:
        print("Unknown demo '%s'. See: python -m omnisim demos" % wanted, file=sys.stderr)
        return 2
    world = REPO_ROOT / match["world"]
    if not world.exists():
        print("Demo '%s' names a world that is not in this install:" % wanted, file=sys.stderr)
        print("  %s" % world, file=sys.stderr)
        return 1
    print(_console_safe("[omnisim] %s" % match.get("name", wanted)))
    print("[omnisim] %s" % match["world"])
    if args.headless:
        return dev.run_headless(str(world), duration=None, fail_on_warning=False,
                                extra_args=["--until-finalized", *args.extra_args])
    return _run_world(argparse.Namespace(world=str(world), extra_args=list(args.extra_args)))


def _run_world(args: argparse.Namespace) -> int:
    # Resolve the world FIRST. The gate below spawns a real engine, so a typo'd
    # path used to cost a silent multi-second launch and only then report
    # "World not found".
    from .dev.runner import require_world
    try:
        world = require_world(args.world)
    except SystemExit as exc:
        print(str(exc), file=sys.stderr)
        print("List what is available:  python -m omnisim demos", file=sys.stderr)
        return 1
    # First-run conformance gate (Phase 3): mandatory-but-bypassable on the
    # interactive launch path, fails OPEN. Clears instantly once a valid stamp
    # exists for this build. It spawns a real engine and printed NOTHING, so a
    # first launch looked like a hang; announce it.
    from .conformance.gate import gate, _env_skip, _argv_skip
    if _env_skip() is None and _argv_skip(args.extra_args) is None:
        print("[omnisim] first launch on this build: verifying the install "
              "(~30 s, once only; skip with OMNISIM_SKIP_CONFORMANCE=1)", flush=True)
    from .conformance.gate import gate
    if gate(args.extra_args) != 0:
        return 1
    return dev.run_world(world, args.extra_args)


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
        description="OmniSim - agent-native robot simulation. Start with: doctor, demo.",
    )
    parser.add_argument("-V", "--version", action="version", version=f"omnisim {__version__}")
    # required=False: `python -m omnisim` with no arguments is the most natural
    # exploratory command there is, and argparse answered it with two lines of
    # parser noise and exit 2. main() renders an orientation instead.
    sub = parser.add_subparsers(dest="command", required=False, metavar="<command>")

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

    p = sub.add_parser("demo", help="Run a named demo (see `omnisim demos`). No id = the flagship demo.")
    p.add_argument("id", nargs="?", help="Demo id from `omnisim demos`.")
    p.add_argument("--headless", action="store_true",
                   help="Load-check it headlessly instead of opening a window.")
    p.add_argument("extra_args", nargs=argparse.REMAINDER,
                   help="Extra flags forwarded to the simulator.")
    p.set_defaults(func=_demo)

    p = sub.add_parser("demos", help="List every runnable demo, grouped by category.")
    p.add_argument("-v", "--verbose", action="store_true", help="Also print each demo's blurb.")
    p.set_defaults(func=_demos)

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
                   help="Forwarded to headless_runner.py: --until-finalized (stop at Newton "
                        "finalize -- the standard load check, and what omitting --duration "
                        "selects), --fail-on-runaway, --port=N, --gui, --realtime, "
                        "--wait-for-step, --require-newton, ...")
    p.set_defaults(func=_run_headless)


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
    sub.add_parser("capture", help="Cinematic capture service (HTTP, port 6791): stills and mp4 sequences.")
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
    if getattr(args, "command", None) is None:
        return _orientation()
    return args.func(args)


def _orientation() -> int:
    """What `python -m omnisim` prints with no arguments.

    Three facts and three commands. It deliberately reports the two things a
    newcomer cannot otherwise discover -- whether there is an engine, and
    whether there is a physics backend -- because Newton is the only backend
    and its absence produces an install where nothing ever moves while every
    command still exits 0.
    """
    from .doctor import _physics_runtime, invocation
    from .paths import resolve_omnisim_binary
    binary = resolve_omnisim_binary()
    physics = _physics_runtime(binary)
    engine = "OK" if binary else "NOT BUILT"
    phys = {"present": "OK", "absent": "MISSING", "unknown": "?"}[physics["status"]]
    print("OmniSim %s   %s" % (__version__, REPO_ROOT))
    print("engine: %s   physics: %s" % (engine, phys))
    print("")
    cli = invocation()
    for verb, blurb in (("demo", "see a robot move"),
                        ("demos", "the full demo catalogue"),
                        ("doctor", "check this install"),
                        ("--help", "every command")):
        print("  %-28s # %s" % (cli + " " + verb, blurb))
    if not binary or physics["status"] == "absent":
        print("")
        print("This install is not ready to run. `doctor` says what to do about it.")
        return 1
    return 0
