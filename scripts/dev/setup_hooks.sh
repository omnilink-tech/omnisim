#!/usr/bin/env bash
# One-time per-clone setup: route git hooks through the versioned .githooks/
# directory so post-merge / post-checkout fire after pulls and branch switches.
set -e
ROOT="$(git rev-parse --show-toplevel)"
git -C "$ROOT" config core.hooksPath .githooks
chmod +x "$ROOT/.githooks/"* 2>/dev/null || true
echo "Hooks enabled (core.hooksPath=.githooks)."
echo "Preview cleanups any time with:"
echo "  python scripts/dev/clean_orphans.py --dry-run"
