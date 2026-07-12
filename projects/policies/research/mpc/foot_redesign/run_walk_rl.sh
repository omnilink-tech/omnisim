#!/usr/bin/env bash
# COMPAT SHIM -> the launcher moved to projects/policies/training/run_walk_rl.sh (flagship promotion)
exec bash "$(dirname "${BASH_SOURCE[0]}")/../../../training/run_walk_rl.sh" "$@"
