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

"""Where the checkout is, and the physics libraries the engine itself uses.

The core must step the model with the SAME MuJoCo build the engine ran, or
"the same model" stops meaning "the same numbers". That build lives in the
engine's runtime bundle, so the bundle goes first on the import path.
"""

from __future__ import annotations

import sys
from pathlib import Path

# packages/omnisim-core/src/omnisim_core/runtime.py -> the checkout
HOME = Path(__file__).resolve().parents[4]
SRC = Path(__file__).resolve().parents[1]

_BUNDLE_CANDIDATES = (
    "msys64/mingw64/bin/newton-runtime/site-packages",
    "bin/newton-runtime/site-packages",
    "lib/omnisim/newton-runtime/site-packages",
)


def bundle_dir() -> Path | None:
    for rel in _BUNDLE_CANDIDATES:
        p = HOME / rel
        if (p / "mujoco").is_dir():
            return p
    return None


def engine_binary() -> Path | None:
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        p = HOME / rel
        if p.is_file():
            return p
    return None


_READY: tuple[bool, str] | None = None


def ensure_runtime() -> tuple[bool, str]:
    """Put the engine's physics libraries on the path. -> (usable, why not)."""
    global _READY
    if _READY is not None:
        return _READY
    b = bundle_dir()
    if b is None:
        _READY = (False, "the engine's physics runtime bundle is not staged (python -m omnisim doctor)")
        return _READY
    if str(b) not in sys.path:
        sys.path.insert(0, str(b))
    try:
        import mujoco  # noqa: F401
        import numpy  # noqa: F401
    except Exception as exc:  # a broken bundle is a reason to use the engine, not to crash
        _READY = (False, f"the physics runtime does not import here: {exc!r}"[:200])
        return _READY
    _READY = (True, "")
    return _READY


def have(module: str) -> bool:
    import importlib.util
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False
