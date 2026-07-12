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


def resolve_webots_binary() -> str | None:
    """Locate the OmniSim simulator executable, or return None if missing.

    Function name is kept (``resolve_webots_binary``) for back-compat with
    existing callers; the binary it resolves is ``omnisim-bin``.
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
