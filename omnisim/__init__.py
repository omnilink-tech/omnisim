"""OmniSim — agent-driven robot simulation built on Webots.

The canonical Python entry point. Run `python -m omnisim --help` for the CLI.
"""

# Tracks the OmniSim release. Keep in sync with omniSimVersionString in
# src/omnisim/core/WbApplicationInfo.cpp — publish_snapshot.sh bumps that one
# but NOT this one, so this must be updated by hand at release time.
__version__ = "5.0.0"
