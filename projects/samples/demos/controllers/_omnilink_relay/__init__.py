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

"""Shared OmniLink relay used by every ``omnilink_*_bridge`` controller.

The installable ``omnisim-bridges`` package is the canonical implementation.
Controllers load its source directly from this checkout, so the simulator
dogfoods the same relay external integrators receive. There is deliberately no
second controller-local copy to drift.
"""

from __future__ import annotations

import sys
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[5]
_PACKAGE_SRC = _REPO_ROOT / "packages" / "omnisim-bridges" / "src"

if not _PACKAGE_SRC.is_dir():
    raise ImportError(f"canonical omnisim-bridges source missing: {_PACKAGE_SRC}")
if str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))
from omnisim_bridges.relay import (  # noqa: F401,E402
    OmniLinkRelay,
    OllamaRelay,
    Tool,
    get_omni_key,
    is_enabled,
    ollama_available,
)
from omnisim_bridges import profile_sync  # noqa: F401,E402
