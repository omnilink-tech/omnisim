#!/usr/bin/env bash
# Compatibility wrapper for the verified, labelled Python encoder.
set -euo pipefail
REAL="${1:?usage: make_side_by_side.sh <real.mp4> <sim_frames_dir> <out.mp4>}"
SIMDIR="${2:?}"
OUT="${3:?}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python "$SCRIPT_DIR/make_video.py" --real "$REAL" --run-dir "$(dirname -- "$SIMDIR")" --output "$OUT"
