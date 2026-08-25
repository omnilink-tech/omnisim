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

"""Compatibility import for the canonical :mod:`omnisim_bridges` bridge.

The real-robot starter scripts historically carried a byte-for-byte copy of
``BridgeBase``.  Keeping a second implementation allowed security and protocol
fixes to drift.  The starters now execute the package source in this checkout,
which is the same code external users receive.
"""

from __future__ import annotations

import sys
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[2]
_PACKAGE_SRC = _REPO_ROOT / "packages" / "omnisim-bridges" / "src"
if str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from omnisim_bridges.bridge_base import BridgeBase, serve_http  # noqa: E402,F401

__all__ = ["BridgeBase", "serve_http"]
