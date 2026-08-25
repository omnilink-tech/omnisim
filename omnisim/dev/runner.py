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

"""Shared subprocess + path helpers for omnisim.dev."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

from ..paths import REPO_ROOT


def infer_jobs() -> int:
    configured = os.environ.get("OMNISIM_BUILD_JOBS")
    if configured:
        try:
            return max(1, int(configured))
        except ValueError:
            pass
    count = os.cpu_count() or 4
    # C++ front ends are CPU- and memory-heavy. Avoid oversubscribing logical
    # CPUs; broad rebuilds on the reference 16-thread machine previously
    # launched 24 competing cc1plus processes.
    return max(1, min(count, 12))


def shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def run(parts: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> int:
    workdir = cwd or REPO_ROOT
    print(f"[omnisim] {shell_join(parts)}")
    child_env = env or os.environ.copy()
    resolved = list(parts)
    # MSYS Python's CreateProcess path lookup is inconsistent for executables
    # added to a Windows-style PATH at runtime. Resolve the GNU make front door
    # explicitly; recursive make then works normally inside MSYS.
    if sys.platform == "win32" and resolved and resolved[0] == "make":
        resolved[0] = shutil.which("make", path=child_env.get("PATH")) or "make"
    return subprocess.run(resolved, cwd=workdir, env=child_env, check=False).returncode


def require_world(path: str) -> str:
    world = Path(path)
    if not world.is_absolute():
        world = REPO_ROOT / world
    if not world.exists():
        raise SystemExit(f"World not found: {world}")
    return str(world)


def omnisim_env() -> dict[str, str]:
    """Build the env for `omnisim` CLI subprocesses (harness, build, test, run-*).

    `OMNISIM_HOME` is pinned to this clone's `REPO_ROOT` regardless of any
    pre-existing value in the parent shell. `omnisim` is a dev tool that
    operates on *this* clone, so a user whose shell has a system Webots
    install on `OMNISIM_HOME=C:\\Program Files\\Webots` should still see CLI
    subprocesses use the clone's bundled binary at
    `<repo>/msys64/mingw64/bin/webots.exe`. (Mirrors the working version
    in `scripts/dev/omnisim_run_agent.py:omnisim_env()`.) On Windows the
    bundled msys2 mingw64 bin is also prepended to `PATH` so Qt6 / mingw
    runtime DLLs resolve without the user editing their shell profile.

    PATH ORDER CHOOSES THE CONTROLLER INTERPRETER -- see the long note on the
    newton-runtime entry below before reordering anything here.
    """
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    # Pin the legacy alias too: libController and the simulator dual-read
    # WEBOTS_HOME, and a stale system value from an old Webots install
    # (C:\Program Files\Webots) must not leak into children.
    env["WEBOTS_HOME"] = str(REPO_ROOT)
    # Keep incremental objects and relinks on the same renderer feature set.
    # This checkout-local SDK is where the documented wgpu setup stages its
    # headers/libraries; omitting it after prior wgpu-enabled compiles produces
    # a late link full of unresolved wgpu symbols.
    local_wgpu = REPO_ROOT / "_scratch" / "wgpu-native"
    if "WGPU_NATIVE_HOME" not in env and (local_wgpu / "include" / "webgpu" / "webgpu.h").exists():
        env["WGPU_NATIVE_HOME"] = str(local_wgpu).replace("\\", "/")
    if sys.platform == "win32":
        msys_bin = REPO_ROOT / "msys64" / "mingw64" / "bin"
        msys_toolchain_bin = Path("C:/msys64/mingw64/bin")
        msys_usr_bin = Path("C:/msys64/usr/bin")
        if msys_bin.exists():
            tool_paths = [str(msys_bin)]
            if msys_toolchain_bin.exists() and msys_toolchain_bin != msys_bin:
                tool_paths.append(str(msys_toolchain_bin))
            # The compiler lives in mingw64/bin, while GNU make and the POSIX
            # recipe tools (`date`, `dirname`, `sh`) live in MSYS usr/bin.
            # Include both so `python -m omnisim build` works from PowerShell,
            # not only from an already-configured MSYS terminal.
            if msys_usr_bin.exists():
                tool_paths.append(str(msys_usr_bin))
            env["PATH"] = os.pathsep.join([*tool_paths, env.get("PATH", "")])
        # MSYS make's wildcard probe can be blocked by Windows ACL translation
        # even when the compiler can read the python.org headers. Pin the same
        # conventional installation explicitly for CLI builds.
        if "PYTHON_HOME" not in env:
            python_root = Path(env.get("LOCALAPPDATA", "")) / "Programs" / "Python"
            candidates = sorted(
                path.parent.parent for path in python_root.glob("Python3??/include/Python.h")
            ) if python_root.is_dir() else []
            if candidates:
                env["PYTHON_HOME"] = str(candidates[-1])
                env.setdefault("PYTHON_LIB", f"-l{candidates[-1].name.lower()}")
        # ── newton-runtime goes ON THE TAIL, not the front ────────────────
        # This used to be PREPENDED, which silently degraded every OmniLink
        # demo launched through `python -m omnisim run-world` /
        # `run-headless`:
        #
        #   The engine spawns each Python controller as the BARE COMMAND
        #   "python.exe" (OmLanguageTools::pythonCommand -> QProcess),
        #   resolved from this PATH. Prepending handed every bridge the
        #   bundled physics interpreter, which has no `omnisim_bridges` on
        #   sys.path, so the deferred-intent tool layer and the shared
        #   status/resume intents fell back to their "package absent" stubs
        #   behind a bare `except Exception`. Nothing said so: controller
        #   stdout never reaches the log (OmLog::appendStdout writes to
        #   std::cout and emits a Qt signal but never calls fileLog(), and
        #   omnisim-bin.exe is a GUI-subsystem binary), so the demo came up
        #   quietly missing features. Measured 2026-07-28: with the old
        #   order `python` resolved to
        #   msys64\mingw64\bin\newton-runtime\python.exe on this path, and
        #   `import omnisim_bridges` fails there.
        #
        # This was never what made Newton work, so moving it costs nothing.
        # The engine's embedded interpreter is not resolved through PATH:
        # omnisim-bin.exe loads python312.dll from its OWN directory (the
        # application directory is searched before PATH) and the
        # python312._pth beside that DLL puts sys.path on
        # newton-runtime/{Lib,DLLs,site-packages}. Verified: 22 launches via
        # scripts/dev/headless_runner.py -- which never puts newton-runtime
        # on PATH -- all wrote {"backend":"newton","degraded":false,
        # "finalised":true,"solver":"MuJoCo (cpu/mj_step, ...)"} to
        # <log>.newton.json.
        #
        # Kept on the TAIL for its one real service: a box with no system
        # Python still resolves an interpreter, so controllers start
        # (degraded) instead of every one dying "Python was not found".
        bundled_python = msys_bin / "newton-runtime" / "python.exe"
        if bundled_python.exists():
            env["PATH"] = f"{env['PATH']}{os.pathsep}{bundled_python.parent}"
    return env


def omnisim_binary_or_die() -> str:
    """Resolve the OmniSim binary or raise SystemExit with a helpful message."""
    from ..paths import resolve_omnisim_binary
    binary = resolve_omnisim_binary()
    if not binary:
        raise SystemExit(
            "Could not locate the OmniSim binary. "
            "Set OMNISIM_HOME to a built tree or install location, or build first "
            "(`python -m omnisim build all`)."
        )
    return binary
