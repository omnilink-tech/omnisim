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

"""Action functions for omnisim build / test / run / profile / harness.

Importable, side-effect-only (run subprocesses, return their exit code).
The CLI in `omnisim.cli` wires argparse to these.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

from ..paths import REPO_ROOT
from .runner import infer_jobs, require_world, run, omnisim_binary_or_die, omnisim_env


BUILD_TARGETS = (
    "all", "core", "gui", "changed", "activate-staged", "renderer",
    "controller-libs", "all-controllers", "package",
)


def _activate_staged_binary() -> int:
    binary_dir = REPO_ROOT / "msys64" / "mingw64" / "bin"
    primary = binary_dir / "omnisim-bin.exe"
    staged = binary_dir / "omnisim-bin.next.exe"
    previous = binary_dir / "omnisim-bin.previous.exe"
    if not staged.exists():
        print(f"[omnisim] No staged binary exists at {staged}", file=sys.stderr)
        print("[omnisim] Build one with: python -m omnisim build gui --staged", file=sys.stderr)
        return 2
    try:
        if primary.exists():
            primary.replace(previous)
        try:
            staged.replace(primary)
        except OSError:
            if previous.exists() and not primary.exists():
                previous.replace(primary)
            raise
    except OSError as exc:
        print(f"[omnisim] Could not activate the staged binary: {exc}", file=sys.stderr)
        print("[omnisim] Close running OmniSim processes and retry `build activate-staged`.", file=sys.stderr)
        return 1
    print(f"[omnisim] Activated {primary.name}; previous build kept as {previous.name}")
    return 0


def build(
    target: str,
    mode: str = "release",
    jobs: int = 0,
    base: str = "HEAD",
    link: bool = False,
    staged: bool = False,
) -> int:
    env = omnisim_env()
    j = str(jobs or infer_jobs())
    commands = {
        "all":              ["make", f"-j{j}", mode],
        "core":             ["make", f"-j{j}", "omnisim_target"],
        # Route public targets through the ccache-enabled top-level Makefile.
        "gui":              ["make", f"-j{j}", "sim-gui-staged" if staged else "sim-gui"],
        # NOTE: "renderer" is deliberately absent. The `renderer` make target
        # survives only in the top-level Makefile's .PHONY and goal-filter
        # lists and has NO RECIPE, so `make renderer` printed "Nothing to be
        # done" and exited 0 -- a build subcommand that reported success while
        # building nothing. It named WREN, which was deleted on 2026-08-23
        # (976b9449d); the wgpu renderer lives in src/omnisim/render/ and is
        # rebuilt by `core` and `gui`. Handled explicitly below so the answer
        # is a message rather than a silent no-op.
        "controller-libs":  ["make", f"-j{j}", "controller-libs"],
        "all-controllers":  ["make", f"-j{j}", "all-controllers"],
        "package":          ["make", f"-j{j}", "distrib"],
    }
    if target == "changed":
        from .changed_build import build_changed
        return build_changed(base=base, jobs=int(j), link=link, staged=staged, env=env)
    if target == "activate-staged":
        return _activate_staged_binary()
    if target == "renderer":
        raise SystemExit(chr(10).join([
            'build renderer: there is no separate renderer target any more.',
            '  WREN was deleted on 2026-08-23 (976b9449d) and wgpu-native is the only',
            '  renderer; its source is src/omnisim/render/ and it is built as part of the',
            '  engine. Use `build core` (or `build gui` for the desktop layer).',
            '  This used to run `make renderer`, a target with NO RECIPE -- it printed',
            "  'Nothing to be done' and exited 0, so it reported success while building",
            '  nothing.',
        ]))
    if target not in commands:
        raise SystemExit(f"Unknown build target: {target}")
    return run(commands[target], env=env)


def test_smoke(nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "smoke" / "run_smoke.py")]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=omnisim_env())


def test_group(group: str, nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "test_suite.py"), "--group", group]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=omnisim_env())


def test_world(world: str, nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "test_suite.py")]
    if nomake:
        cmd.append("--nomake")
    cmd.append(require_world(world))
    return run(cmd, env=omnisim_env())


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
    return run(cmd, env=omnisim_env())


def benchmarks(nomake: bool = False) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "tests" / "benchmarks" / "run_benchmarks.py")]
    if nomake:
        cmd.append("--nomake")
    return run(cmd, env=omnisim_env())


def compile_commands() -> int:
    bear = shutil.which("bear")
    if not bear:
        print(
            "[omnisim] `bear` is not installed; cannot generate compile_commands.json automatically.",
            file=sys.stderr,
        )
        print("[omnisim] Install bear and rerun: python -m omnisim compile-commands", file=sys.stderr)
        return 2
    return run([bear, "--", "make", f"-j{infer_jobs()}", "omnisim_target"], env=omnisim_env())


def run_world(world: str, extra_args: list[str] | tuple[str, ...] = ()) -> int:
    cmd = [omnisim_binary_or_die(), require_world(world), *extra_args]
    return run(cmd, env=omnisim_env())


def run_headless(
    world: str,
    duration: int | None = None,
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
    # Forward --duration ONLY when the caller actually chose one. Passing a
    # default here would tell the runner "the caller wants exactly N seconds",
    # which is the opposite of the truth and suppresses its load-check path.
    if duration is not None and "--duration" not in extra_args:
        cmd += ["--duration", str(duration)]
    if fail_on_warning and "--fail-on-warning" not in extra_args:
        cmd.append("--fail-on-warning")
    cmd.extend(extra_args)
    return run(cmd, env=omnisim_env())


def harness(host: str = "127.0.0.1", port: int = 6789, supervisor_port: int | None = None,
            auto_port: bool = False) -> int:
    """Start the agent-facing validation harness (foreground).

    ``auto_port`` forwards ``--auto-port``: if the requested (port, port+1) pair
    is taken, scan upward and print the pair actually bound. Without forwarding it
    here, the documented `python -m omnisim harness --auto-port` invocation could
    not do the thing the flag exists for.
    """
    cmd = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "harness" / "omnisim_harness.py"),
        "--host", host,
        "--port", str(port),
    ]
    if auto_port:
        cmd.append("--auto-port")
    if supervisor_port is not None:
        cmd.extend(["--supervisor-port", str(supervisor_port)])
    return run(cmd, env=omnisim_env())
