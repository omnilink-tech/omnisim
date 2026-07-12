"""Shared subprocess + path helpers for omnisim.dev."""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from pathlib import Path

from ..paths import REPO_ROOT


def infer_jobs() -> int:
    count = os.cpu_count() or 4
    return max(1, (count * 3) // 2)


def shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def run(parts: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> int:
    workdir = cwd or REPO_ROOT
    print(f"[omnisim] {shell_join(parts)}")
    return subprocess.run(parts, cwd=workdir, env=env or os.environ.copy(), check=False).returncode


def require_world(path: str) -> str:
    world = Path(path)
    if not world.is_absolute():
        world = REPO_ROOT / world
    if not world.exists():
        raise SystemExit(f"World not found: {world}")
    return str(world)


def webots_env() -> dict[str, str]:
    """Build the env for `omnisim` CLI subprocesses (harness, build, test, run-*).

    `OMNISIM_HOME` is pinned to this clone's `REPO_ROOT` regardless of any
    pre-existing value in the parent shell. `omnisim` is a dev tool that
    operates on *this* clone, so a user whose shell has a system Webots
    install on `OMNISIM_HOME=C:\\Program Files\\Webots` should still see CLI
    subprocesses use the clone's bundled binary at
    `<repo>/msys64/mingw64/bin/webots.exe`. (Mirrors the working version
    in `scripts/dev/omnisim_run_agent.py:webots_env()`.) On Windows the
    bundled msys2 mingw64 bin is also prepended to `PATH` so Qt6 / mingw
    runtime DLLs resolve without the user editing their shell profile.
    """
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    # Pin the legacy alias too: libController and the simulator dual-read
    # WEBOTS_HOME, and a stale system value from an old Webots install
    # (C:\Program Files\Webots) must not leak into children.
    env["WEBOTS_HOME"] = str(REPO_ROOT)
    if sys.platform == "win32":
        msys_bin = REPO_ROOT / "msys64" / "mingw64" / "bin"
        if msys_bin.exists():
            env["PATH"] = f"{msys_bin}{os.pathsep}{env.get('PATH', '')}"
    return env


def webots_binary_or_die() -> str:
    """Resolve the OmniSim binary or raise SystemExit with a helpful message."""
    from ..paths import resolve_webots_binary
    binary = resolve_webots_binary()
    if not binary:
        raise SystemExit(
            "Could not locate the OmniSim binary. "
            "Set OMNISIM_HOME to a built tree or install location, or build first "
            "(`python -m omnisim build all`)."
        )
    return binary
