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

"""omnisim-mcp — a Model Context Protocol server over the OmniSim World Harness.

See server.py for the full rationale. The short version: OmniSim already ships a
first-party agent-facing HTTP surface (the harness, PROTOCOL.md §world_harness);
this exposes it to the MCP-standardized agent ecosystem (Claude Desktop, Cursor)
as a thin, dependency-free stdio proxy.
"""
from .server import main, TOOLS

__version__ = "0.1.0"
__all__ = ["main", "TOOLS", "__version__"]
