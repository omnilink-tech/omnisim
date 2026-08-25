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

# Deploy the OmniQuad FORWARD LEAP in OmniSim Newton -- the flight-phase shadowing test.
# OmniQuad shadows its MPPI jump ghost (RL tracker) and leaps. The landing impact is the
# worst mujoco_warp->Newton gap; this is where we find the limit.
# Usage: bash scripts/dev/run_omniquad_jump_deploy.sh [duration] [--gui] [--policy <onnx>] [--cpu-engine]
#
# NOTE: bash sibling of run_omniquad_jump_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=12; GUI=0; CPU_ENGINE=0; POLICY=""
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --gui)        GUI=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_jump_deploy.omniworld"
LOG="$ROOT/_scratch/omniquad_jump_deploy.log"
if [ -z "$POLICY" ]; then POLICY="$ROOT/projects/policies/research/training/runs/gpu_omniquad_jump_a/policy.onnx"; fi

export OMNISIM_HOME="$ROOT"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"          # assert Newton engaged -- fail loud, no silent ODE fallback
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
if [ "$CPU_ENGINE" = "1" ]; then unset OMNISIM_NEWTON_MJWARP; else export OMNISIM_NEWTON_MJWARP="1"; fi
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
export OMNISIM_LOG_PATH="$ROOT/_scratch/omnisim_log_omniquad_jump.txt"
# Reuse the generic reference-tracking controller (OMNIQUAD_GETUP_* knobs) with the JUMP ghost.
export OMNIQUAD_GETUP_REF="$ROOT/projects/policies/research/shadowing/ghosts/omniquad_jump_ghost.npz"
export OMNIQUAD_GETUP_LOG="$LOG"
export OMNIQUAD_GETUP_RES_SCALE="0.25"
export OMNIQUAD_SETTLE_S="0.6"
export OMNIQUAD_GETUP_HOLD_FADE_S="0.8"
if [ -f "$POLICY" ]; then export OMNIQUAD_GETUP_POLICY="$POLICY"; echo "RL policy: $POLICY"
else echo "NO policy at $POLICY -- FEEDFORWARD (the leap will likely fail open-loop)"; fi

if [ "$GUI" = "1" ]; then
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --realtime --duration "$DURATION" 2>&1 | tail -n 3 || true
else
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --duration "$DURATION" 2>&1 | tail -n 3 || true
fi

echo
echo "===== OMNIQUAD JUMP RESULT ====="
if [ -f "$LOG" ]; then cat "$LOG"; else echo "  (no log)"; fi
