"""Pytest config: put ``src/python`` on sys.path so tests can import omniworld
without requiring an editable install."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PY_SRC = REPO_ROOT / "src" / "python"
if str(PY_SRC) not in sys.path:
    sys.path.insert(0, str(PY_SRC))
