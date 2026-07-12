"""Action functions for omnisim build / test / run / profile / harness.

Importable, side-effect-only (run subprocesses, return their exit code).
The CLI in `omnisim.cli` wires argparse to these.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

from ..paths import REPO_ROOT
from .runner import infer_jobs, require_world, run, webots_binary_or_die, webots_env


BUILD_TARGETS = ("all", "core", "gui", "renderer", "controller-libs", "all-controllers", "package")


def build(target: str, mode: str = "release", jobs: int = 0) -> int:
    env = webots_env()
    j = str(jobs or infer_jobs())
    commands = {
        "all":              ["make", f"-j{j}", mode],
        "core":             ["make", f"-j{j}", "webots_target"],
        "gui":              ["make", "-C", "src/omnisim", f"-j{j}", mode],
        "renderer":         ["make", "-C", "src/wren", mode],
        "controller-libs":  ["make", "-C", "src/controller", f"-j{j}", mode, f"OMNISIM_HOME={env['OMNISIM_HOME']}"],
        "all-controllers":  ["make", f"-j{j}", "all-controllers"],
        "package":          ["make", f"-j{j}", "distrib"],
    }
    if target not in commands:
        raise SystemExit(f"Unknown build target: {target}")
    return run(commands[target], env=env)


def test_smoke(nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "smoke" / "run_smoke.py")]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=webots_env())


def test_group(group: str, nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "test_suite.py"), "--group", group]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=webots_env())


def test_world(world: str, nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "test_suite.py")]
    if nomake:
        cmd.append("--nomake")
    cmd.append(require_world(world))
    return run(cmd, env=webots_env())


def profile_world(world: str, log_path: str = "") -> int:
    log = log_path or str(REPO_ROOT / "tests" / "benchmarks" / "last-performance.log")
    cmd = [
        sys.executable,
        str(REPO_ROOT / "tests" / "test_suite.py"),
        "--nomake",
        "--performance-log",
        log,
        require_world(world),
    ]
    return run(cmd, env=webots_env())


def benchmarks(nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "benchmarks" / "run_benchmarks.py")]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=webots_env())


def compile_commands() -> int:
    bear = shutil.which("bear")
    if not bear:
        print(
            "[omnisim] `bear` is not installed; cannot generate compile_commands.json automatically.",
            file=sys.stderr,
        )
        print("[omnisim] Install bear and rerun: python -m omnisim compile-commands", file=sys.stderr)
        return 2
    return run([bear, "--", "make", f"-j{infer_jobs()}", "webots_target"], env=webots_env())


def run_world(world: str, extra_args: list[str] | tuple[str, ...] = ()) -> int:
    cmd = [webots_binary_or_die(), require_world(world), *extra_args]
    return run(cmd, env=webots_env())


def run_headless(
    world: str,
    duration: int = 10,
    fail_on_warning: bool = False,
    extra_args: list[str] | tuple[str, ...] = (),
) -> int:
    cmd = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "dev" / "headless_runner.py"),
        require_world(world),
    ]
    # The CLI's REMAINDER capture may already carry these flags verbatim;
    # only add the parsed values when they aren't being forwarded, so the
    # child never sees "--duration 10 --duration 40".
    if "--duration" not in extra_args:
        cmd += ["--duration", str(duration)]
    if fail_on_warning and "--fail-on-warning" not in extra_args:
        cmd.append("--fail-on-warning")
    cmd.extend(extra_args)
    return run(cmd, env=webots_env())


def harness(host: str = "127.0.0.1", port: int = 6789, supervisor_port: int | None = None) -> int:
    """Start the agent-facing validation harness (foreground)."""
    cmd = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "harness" / "omnisim_harness.py"),
        "--host", host,
        "--port", str(port),
    ]
    if supervisor_port is not None:
        cmd.extend(["--supervisor-port", str(supervisor_port)])
    return run(cmd, env=webots_env())
