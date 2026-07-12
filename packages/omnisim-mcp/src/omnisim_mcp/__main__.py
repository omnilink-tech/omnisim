"""Enable `python -m omnisim_mcp` (no install needed, from this package's src/)."""
import sys

from .server import main

if __name__ == "__main__":
    sys.exit(main())
