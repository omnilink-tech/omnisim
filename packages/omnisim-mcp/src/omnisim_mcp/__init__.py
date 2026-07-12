"""omnisim-mcp — a Model Context Protocol server over the OmniSim World Harness.

See server.py for the full rationale. The short version: OmniSim already ships a
first-party agent-facing HTTP surface (the harness, PROTOCOL.md §world_harness);
this exposes it to the MCP-standardized agent ecosystem (Claude Desktop, Cursor)
as a thin, dependency-free stdio proxy.
"""
from .server import main, TOOLS

__version__ = "0.1.0"
__all__ = ["main", "TOOLS", "__version__"]
