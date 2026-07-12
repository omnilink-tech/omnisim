#!/usr/bin/env python3
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

"""Backwards-compat shim. Use `python -m omnisim ...` going forward.

Logic now lives in `omnisim.dev.commands` (importable) and
`omnisim.cli` (argparse). This file forwards `python scripts/dev/omnisim_dev.py
<args>` to the package CLI so existing AGENTS.md examples and external
callers keep working.
"""

from __future__ import annotations

import sys
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from omnisim.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
