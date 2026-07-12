#!/usr/bin/env bash
# Incremental relink after the install-dir guard change in WbFileUtil.cpp.
# Skips webots_dependencies (no network) and builds the code libs directly.
# Pins cwd to the repo root (derived from this script's own location) so
# it works from any directory on any machine.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"
# OmniSim binary is a Windows process and reads OMNISIM_HOME as a
# Windows path. Convert the POSIX repo root via cygpath when available;
# otherwise pass POSIX (works on Linux/macOS native builds).
if command -v cygpath >/dev/null 2>&1; then
  export OMNISIM_HOME="$(cygpath -w "$REPO_ROOT")"
else
  export OMNISIM_HOME="$REPO_ROOT"
fi
# Build the distributable CUDA-OFF stub path (byte-equivalent per the Makefile),
# so the link doesn't require the local CUDA toolkit's runtime libs.
export OMNISIM_WITH_CUDA=OFF
# Drop CUDA-enabled objects so they recompile as the no-op stubs.
rm -f src/omnisim/build/release/WbCuda*.o
echo "=== toolchain ==="
gcc --version | head -1
echo "=== ode ===";     make -C src/ode      release
echo "=== glad ===";    make -C src/glad     release
echo "=== wren ===";    make -C src/wren     release
echo "=== omnisim ==="; make -C src/omnisim  release
echo "=== DONE ==="
