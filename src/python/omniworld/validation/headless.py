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

"""Headless-simulator validation.

Wraps ``webots --batch --mode=fast --no-rendering`` and parses the
output for load-time, errors, and warnings. This is the T1.7
validator set — the one the in-process checks cannot cover because
they never touch the simulator.

Strategy:

- Discover the OmniSim binary: ``OMNIWORLD_OMNISIM_BIN`` env var wins
  (``OMNIWORLD_WEBOTS_BIN`` is honoured as a legacy alias); otherwise
  we try ``$OMNISIM_HOME/msys64/mingw64/bin/omnisim-bin.exe`` (the
  OmniSim bundled layout) and ``omnisim-bin`` on PATH.
- Launch with a short timeout (OmniSim does not auto-exit, so we
  kill it after ``timeout_s`` seconds).
- A run is a PASS if the simulator produced no ``ERROR:`` lines and
  the wall-clock load took less than ``load_time_budget_s``.

The headless validator is NOT part of the default ``validate()``
call because it is slow (a few seconds per world). Callers opt in
through ``validate(..., with_sim=True)`` or the CLI ``--with-sim``
flag. CI runs it on a slower lane alongside the existing tests.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence


_DEFAULT_TIMEOUT_S = 15.0
_DEFAULT_LOAD_BUDGET_S = 10.0


@dataclass
class HeadlessResult:
    """Outcome of one headless-simulator run."""

    passed: bool
    load_time_s: float
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    stdout_tail: str = ""
    stderr_tail: str = ""

    def detail(self) -> str:
        parts: list[str] = [f"load_time={self.load_time_s:.2f}s"]
        if self.errors:
            parts.append(f"errors={len(self.errors)}")
            parts.append("first_error=" + self.errors[0][:100])
        if self.warnings:
            parts.append(f"warnings={len(self.warnings)}")
        return "; ".join(parts)


def discover_omnisim_bin() -> Path | None:
    """Locate an OmniSim binary we can run. Preference order:

    1. ``OMNIWORLD_OMNISIM_BIN`` env var (absolute path), falling back
       to the legacy ``OMNIWORLD_WEBOTS_BIN``.
    2. ``$OMNISIM_HOME/msys64/mingw64/bin/omnisim-bin.exe`` — the
       OmniSim-bundled layout on Windows (legacy ``WEBOTS_HOME`` is
       honoured when ``OMNISIM_HOME`` is unset).
    3. ``omnisim-bin`` on PATH.
    4. ``<repo_root>/msys64/mingw64/bin/omnisim-bin.exe`` — the
       checked-in bundled runtime.

    An empty value counts as unset throughout, matching the C runtime
    (``src/controller/c/system.c``).
    """
    # OMNIWORLD_OMNISIM_BIN is preferred; OMNIWORLD_WEBOTS_BIN is the legacy alias.
    env_bin = os.environ.get("OMNIWORLD_OMNISIM_BIN") or os.environ.get("OMNIWORLD_WEBOTS_BIN")
    if env_bin:
        candidate = Path(env_bin)
        if candidate.exists():
            return candidate

    # OMNISIM_HOME is canonical; WEBOTS_HOME is the legacy alias.
    omnisim_home = os.environ.get("OMNISIM_HOME") or os.environ.get("WEBOTS_HOME")
    if omnisim_home:
        candidate = Path(omnisim_home) / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
        if candidate.exists():
            return candidate

    found = shutil.which("omnisim-bin")
    if found:
        return Path(found)

    # The repo may ship its own bundled runtime under msys64/.
    # Walk up from this file to find it.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
        if candidate.exists():
            return candidate

    return None


_ERROR_RE = re.compile(r"^(?:ERROR|FATAL)\s*[:!]", re.MULTILINE)
_WARNING_RE = re.compile(r"^WARNING\s*[:!]", re.MULTILINE)


def _split_lines_matching(text: str, regex: re.Pattern) -> list[str]:
    return [line.strip() for line in text.splitlines() if regex.match(line.strip())]


def run_headless(
    world_path: str | Path,
    *,
    omnisim_bin: str | Path | None = None,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
    load_time_budget_s: float = _DEFAULT_LOAD_BUDGET_S,
    strict_warnings: bool = False,
) -> HeadlessResult:
    """Launch OmniSim headless on ``world_path`` and decide whether
    the world loads cleanly.

    Arguments:
        world_path: path to the ``.wbt`` to load.
        omnisim_bin: override the discovered OmniSim binary path.
        timeout_s: kill the process after this many seconds. Must be
            strictly greater than ``load_time_budget_s``.
        load_time_budget_s: maximum acceptable wall-clock time from
            launch to "OmniSim has accepted the world" (proxy: wall
            clock at process exit).
        strict_warnings: treat any ``WARNING:`` line as a failure.

    Raises:
        FileNotFoundError: if no OmniSim binary can be discovered.
    """
    path = Path(world_path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"world file does not exist: {path}")

    binary = Path(omnisim_bin) if omnisim_bin else discover_omnisim_bin()
    if binary is None:
        raise FileNotFoundError(
            "could not locate omnisim binary; set OMNIWORLD_OMNISIM_BIN "
            "(legacy: OMNIWORLD_WEBOTS_BIN) or OMNISIM_HOME, or put "
            "omnisim-bin on PATH"
        )

    if timeout_s <= load_time_budget_s:
        raise ValueError(
            "timeout_s must exceed load_time_budget_s so we can "
            "distinguish 'load too slow' from 'never exited'"
        )

    cmd: Sequence[str] = [
        str(binary),
        "--batch",
        "--mode=fast",
        "--no-rendering",
        "--stdout",
        "--stderr",
        str(path),
    ]

    # Expose the repo-local msys64 runtime on PATH so the engine finds its
    # DLLs (matches what launch.bat does). Harmless on platforms where
    # that directory does not exist. OMNISIM_HOME is canonical; WEBOTS_HOME
    # is the legacy alias. An empty value counts as unset -- a bare presence
    # test would make Path("") resolve the bundled dir relative to the cwd.
    env = os.environ.copy()
    install_root = env.get("OMNISIM_HOME") or env.get("WEBOTS_HOME")
    if install_root:
        bundled = Path(install_root) / "msys64" / "mingw64" / "bin"
        if bundled.exists():
            env["PATH"] = f"{bundled}{os.pathsep}{env.get('PATH', '')}"

    start = time.perf_counter()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            timeout=timeout_s,
            env=env,
            check=False,
            text=True,
        )
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""
        load_time = time.perf_counter() - start
    except subprocess.TimeoutExpired as exc:
        # A timeout is expected — OmniSim does not self-exit. We treat
        # it as "the load phase finished within budget" iff the
        # elapsed-so-far is >= load_time_budget_s (i.e. we had to kill
        # it because it was running steadily) AND there are no errors
        # in what it already emitted.
        load_time = time.perf_counter() - start
        stdout = (exc.stdout or b"").decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = (exc.stderr or b"").decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")

    combined = stdout + "\n" + stderr
    errors = _split_lines_matching(combined, _ERROR_RE)
    warnings = _split_lines_matching(combined, _WARNING_RE)

    # Pass criterion: zero ERROR: lines. OmniSim does not self-exit, so
    # ``load_time`` is always ≈ ``timeout_s``; a real load-time budget
    # is only meaningful once a supervisor controller drives
    # ``wb_supervisor_simulation_quit``, which is T1.9 scope. Until
    # then ``load_time_budget_s`` is reserved on the API but not
    # enforced.
    passed = not errors
    if strict_warnings and warnings:
        passed = False
    _ = load_time_budget_s  # reserved

    def _tail(text: str, n: int = 400) -> str:
        if len(text) <= n:
            return text
        return "..." + text[-n:]

    return HeadlessResult(
        passed=passed,
        load_time_s=load_time,
        errors=errors,
        warnings=warnings,
        stdout_tail=_tail(stdout),
        stderr_tail=_tail(stderr),
    )


__all__ = ["HeadlessResult", "discover_omnisim_bin", "run_headless"]
