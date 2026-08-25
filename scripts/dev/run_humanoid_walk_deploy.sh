#!/usr/bin/env bash
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

# Deterministic WALK-DEPLOY: the PHYSICS humanoid tracks its verified-feasible
# walking SHADOW under OmniSim Newton with a stiff position-PD (no RL). This
# MEASURES the sim-to-deploy gap before any RL compute is spent: how well the
# real Newton articulation follows the shadow, and how long it stays upright.
#
#   bash scripts/dev/run_humanoid_walk_deploy.sh h1 [duration] [--ke 200] [--vx 0.45] [--gui]
#
# The robot argument accepts h1 only. "valkyrie" was removed 2026-08-24: the
# robot package went in be41986f8 and with it ${ROBOT}_walk_deploy.omniworld,
# so the accepted set offered a value that could only ever fail at world-load.
#
# Options: --ke N --kd N --vx V --freq F --settle S --ank-bias B --onnx <path>
#          --pure-rl --act-scale S --obs-history K --res-scale S --extra-log <path> --gui
#
# NOTE: bash sibling of run_humanoid_walk_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

ROBOT="${1:-}"
case "$ROBOT" in h1) shift;; *) echo "usage: run_humanoid_walk_deploy.sh h1 [duration] [options]" >&2; exit 2;; esac

DURATION=20; GUI=0; KE="200"; KD="12"; VX="-1"; FREQ="-1"; SETTLE="0.3"; ANKBIAS="0"
ONNX=""; PURE_RL=0; ACT_SCALE="1"; OBS_HISTORY="1"; RES_SCALE="-1"; EXTRALOG=""
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)    DURATION="$2"; shift 2;;
    --gui)         GUI=1; shift;;
    --ke)          KE="$2"; shift 2;;
    --kd)          KD="$2"; shift 2;;
    --vx)          VX="$2"; shift 2;;
    --freq)        FREQ="$2"; shift 2;;
    --settle)      SETTLE="$2"; shift 2;;
    --ank-bias)    ANKBIAS="$2"; shift 2;;
    --onnx)        ONNX="$2"; shift 2;;
    --pure-rl)     PURE_RL=1; shift;;
    --act-scale)   ACT_SCALE="$2"; shift 2;;
    --obs-history) OBS_HISTORY="$2"; shift 2;;
    --res-scale)   RES_SCALE="$2"; shift 2;;
    --extra-log)   EXTRALOG="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/${ROBOT}_walk_deploy.omniworld"
if [ -n "$EXTRALOG" ]; then LOG="$EXTRALOG"; else LOG="$ROOT/_scratch/walk/${ROBOT}_walk.log"; fi
mkdir -p "$(dirname "$LOG")"
rm -f "$LOG"
if [ ! -f "$WORLD" ]; then echo "no world $WORLD" >&2; exit 1; fi

export OMNISIM_HOME="$ROOT"
export PYTHONUTF8="1"
# --- deploy-faithful Newton solver config (IDENTICAL to the proven stand) ---
export OMNISIM_NEWTON_STATICS="1"
export OMNISIM_NEWTON_SUBSTEPS="4"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_BASE_GUARD="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_SEED_REBUILD="1"
export OMNISIM_NEWTON_TARGET_KE="$KE"
export OMNISIM_NEWTON_TARGET_KD="$KD"
export OMNISIM_NEWTON_GROUND_MU="2.0"
# --- walk controller config ---
export HUMANOID_WALK_ROBOT="$ROBOT"
export HW_SETTLE_S="$SETTLE"
export HW_ANK_BIAS="$ANKBIAS"
if awk "BEGIN{exit !($VX >= 0)}"; then export HW_VX="$VX"; else unset HW_VX; fi
if awk "BEGIN{exit !($FREQ >= 0)}"; then export HW_FREQ="$FREQ"; else unset HW_FREQ; fi
if [ -n "$ONNX" ]; then
  # absolute path (Windows-style under msys bash), like Resolve-Path in the .ps1
  export HUMANOID_WALK_ONNX="$(cd "$(dirname "$ONNX")" && (pwd -W 2>/dev/null || pwd))/$(basename "$ONNX")"
else
  unset HUMANOID_WALK_ONNX
fi
if [ "$PURE_RL" = "1" ]; then
  export HUMANOID_WALK_PURE_RL="1"; export H1_ACT_SCALE="$ACT_SCALE"; export HUMANOID_WALK_OBS_HISTORY="$OBS_HISTORY"
else
  unset HUMANOID_WALK_PURE_RL
fi
if awk "BEGIN{exit !($RES_SCALE >= 0)}"; then export HUMANOID_WALK_RES_SCALE="$RES_SCALE"; else unset HUMANOID_WALK_RES_SCALE; fi
export HUMANOID_WALK_LOG="$LOG"
export OMNISIM_DEPLOY_LOG="$LOG"
export OMNISIM_LOG_PATH="$ROOT/_scratch/walk/omnisim_log_${ROBOT}_walk.txt"

echo "=== $ROBOT WALK-DEPLOY (shadow tracking under Newton, KE=$KE) -- measuring the sim-to-deploy gap ==="
if [ "$GUI" = "1" ]; then
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --realtime --duration "$DURATION"
else
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --duration "$DURATION" 2>&1 | tail -n 6 || true
fi

echo
echo "===== $ROBOT WALK RESULT ====="
if [ -f "$LOG" ]; then
  FALLLINE="$(grep -m1 'FALL@' "$LOG" || true)"
  LAST="$(tail -n 1 "$LOG" || true)"
  if [ -n "$FALLLINE" ]; then echo "  $FALLLINE"; else echo "  no fall logged"; fi
  echo "  last: $LAST"
else
  echo "  (no deploy log)"
fi
