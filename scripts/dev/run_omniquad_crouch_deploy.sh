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

# Deploy the shadowing pipeline END-TO-END on a NON-gait OmniQuad motion: the OmniQuad
# SHADOWS its MPPI-generated, verifier-certified crouch-recover ghost in OmniSim
# Newton. Feedforward by default (the ghost is feasible by construction); pass
# --policy <onnx> to add the RL residual (Component 3).
#
# Pipeline: generate_omniquad_crouch.py (MPPI, Component 1) -> ghost_verifier (C2,
# certified PASS) -> THIS deploy (Stage D). The crouch ghost lives at
# projects/policies/research/shadowing/ghosts/omniquad_crouch_ghost.npz.
#
# Usage: bash scripts/dev/run_omniquad_crouch_deploy.sh [duration] [--loop] [--gui] [--policy <onnx>] [--cpu-engine]
#
# NOTE: bash sibling of run_omniquad_crouch_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=30; LOOP=0; GUI=0; CPU_ENGINE=0; POLICY=""
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --loop)       LOOP=1; shift;;
    --gui)        GUI=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_crouch_deploy.omniworld"
LOG="$ROOT/_scratch/omniquad_crouch_deploy.log"

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
# Match the ghost's dynamics (the MPPI generator used the deploy-matched kp=500/kv=60).
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
export OMNISIM_LOG_PATH="$ROOT/_scratch/omnisim_log_omniquad_crouch.txt"
export OMNIQUAD_CROUCH_REF="$ROOT/projects/policies/research/shadowing/ghosts/omniquad_crouch_ghost.npz"
export OMNIQUAD_CROUCH_LOG="$LOG"
export OMNIQUAD_SETTLE_S="1.0"
if [ "$LOOP" = "1" ]; then export OMNIQUAD_CROUCH_LOOP="1"; else unset OMNIQUAD_CROUCH_LOOP; fi
if [ -n "$POLICY" ]; then export OMNIQUAD_CROUCH_POLICY="$POLICY"; else unset OMNIQUAD_CROUCH_POLICY; fi

if [ -n "$POLICY" ]; then echo "OmniQuad crouch deploy (RL residual: $POLICY)"
else                      echo "OmniQuad crouch deploy (FEEDFORWARD -- shadowing the certified ghost)"; fi
if [ "$GUI" = "1" ]; then
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --realtime --duration "$DURATION" 2>&1 | tail -n 3 || true
else
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --duration "$DURATION" 2>&1 | tail -n 3 || true
fi

echo
echo "===== OMNIQUAD CROUCH RESULT ====="
if [ -f "$LOG" ]; then
  FALL="$(grep -c "FALL" "$LOG" || true)"
  if [ "${FALL:-0}" -eq 0 ]; then echo "  NO FALL."; else echo "  FELL."; fi
  cat "$LOG"
else
  echo "  (no log produced)"
fi
