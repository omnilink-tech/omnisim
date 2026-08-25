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

"""Path helpers shared across OmniSim's CLI surface."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEMOS_WORLDS = REPO_ROOT / "projects" / "samples" / "demos" / "worlds"


def infer_omnisim_home() -> str:
    """Return the install root from ``OMNISIM_HOME``, or the repo containing
    this file when the env var is unset."""
    home = os.environ.get("OMNISIM_HOME")
    if home:
        return home
    return str(REPO_ROOT)


def linux_runtime_env(omnisim_home: str | Path, base_env: dict | None = None) -> dict:
    """Return a child-process env with the Linux runtime vars omnisim-bin needs.

    Running ``bin/omnisim-bin`` directly (as the headless runner, the harness
    and the capture service all do) bypasses the ``webots`` launcher shell, so
    on Linux the child would otherwise miss:

    - ``LD_LIBRARY_PATH`` → without ``$OMNISIM_HOME/lib/webots`` the bundled
      Qt-6.5.3 xcb plugin's transitive ``libQt6XcbQpa.so.6`` resolves against
      the system Qt (e.g. 6.10) and aborts with ``version 'Qt_6.10' not
      found`` (RUNPATH does not propagate to dlopen'd plugins' dependencies).
      Prepended to any existing value.
    - ``QT_QPA_PLATFORM=xcb`` — only if not already set.
    - ``WEBOTS_TMPDIR=/tmp`` — unset it produces a benign "not started
      regularly" ERROR line that fails strict log analysis. Only if unset.
    - ``LIBGL_ALWAYS_SOFTWARE=1`` — headless boxes have no GL; harmless where
      they do. Only if not already set, so GUI users on a real GPU can export
      ``LIBGL_ALWAYS_SOFTWARE=0`` to override.

    On non-Linux platforms the env is returned unchanged. ``base_env`` is not
    mutated; pass ``None`` to start from ``os.environ``.
    """
    env = dict(base_env) if base_env is not None else os.environ.copy()
    if not sys.platform.startswith("linux"):
        return env
    lib_dir = str(Path(omnisim_home) / "lib" / "webots")
    existing = env.get("LD_LIBRARY_PATH", "")
    parts = [p for p in existing.split(os.pathsep) if p]
    if lib_dir not in parts:
        env["LD_LIBRARY_PATH"] = os.pathsep.join([lib_dir] + parts)
    env.setdefault("QT_QPA_PLATFORM", "xcb")
    env.setdefault("WEBOTS_TMPDIR", "/tmp")
    env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    return env


def resolve_omnisim_binary() -> str | None:
    """Locate the OmniSim simulator executable, or return None if missing.

    The ``webots.exe`` candidate below is the legacy launcher copy the Windows
    build still produces; it is a real filename on disk, not a stale name.
    """
    omnisim_home = Path(infer_omnisim_home())
    if sys.platform == "win32":
        candidates = [
            omnisim_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            omnisim_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [omnisim_home / "Contents" / "MacOS" / "omnisim-bin"]
    else:
        candidates = [
            omnisim_home / "omnisim-bin",
            omnisim_home / "bin" / "omnisim-bin",
        ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    on_path = shutil.which("omnisim-bin")
    if on_path:
        return on_path
    return None
