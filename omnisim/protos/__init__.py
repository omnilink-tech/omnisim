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

"""OmniSim PROTO tooling — schemas, validation, authoring, hot-reload, tests.

This package is the agent-facing tooling layer above OmniSim's PROTO system.
Every subcommand here is a pure tooling-layer operation; nothing in here
requires rebuilding the C++ simulator core. PROTO files remain the
authoritative runtime artifact — these tools generate, validate, and
exercise them.

Entry point: ``python -m omnisim proto <subcommand>``.
"""

from __future__ import annotations

__all__ = ["main"]


def main(argv: list[str]) -> int:
    from .cli import main as _main
    return _main(argv)
