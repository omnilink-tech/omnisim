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
