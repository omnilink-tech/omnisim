#!/usr/bin/env bash
# OmniSim container entrypoint.
#
# Two jobs, both of which are easy to get wrong by hand and so are done here
# once:
#
#  1. cd to $OMNISIM_HOME. `python3 -m omnisim` only resolves when the install
#     root is on sys.path, and the container deliberately does NOT set a global
#     PYTHONPATH -- that would leak the CLI package into every controller
#     process the engine spawns, where `import omnisim` must resolve to the
#     CONTROLLER API instead. cwd gives the CLI what it needs without that.
#
#  2. Provide an X server. This is not optional: the engine constructs a Qt
#     main window for any world-running invocation, so a Qt/XCB context exists
#     even under --no-rendering. Without a display the engine aborts in Qt's
#     platform-plugin init with a header-only log. (This used to add "while the
#     launcher still exits 0" -- a pre-03e988c58 measurement; run-headless now
#     FAILs on it and names the cause, public issue #6. OMNISIM_NO_WINDOW=1
#     would avoid the window entirely; Xvfb stays because the default
#     --minimize path still realises one.)
set -euo pipefail

cd "${OMNISIM_HOME:-/opt/omnisim}"

# `docker run ... bash` / `sh` should drop you in a shell, still under Xvfb so
# that anything you then run by hand behaves the same as the default path.
case "${1:-}" in
  bash|sh)
    exec xvfb-run -a "$@"
    ;;
  # Escape hatch: run an arbitrary command instead of an `omnisim` subcommand.
  --exec)
    shift
    exec xvfb-run -a "$@"
    ;;
esac

exec xvfb-run -a python3 -m omnisim "$@"
