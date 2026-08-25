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

# Run the OmniQuad WALK deploy world headless and report forward progress.
#
# THE G1 WALK RECIPE PORTED TO OMNIQUAD (gait-residual RL on the
# stiffness-matched model). The trainer MJCF matches the deploy joint
# drive EXACTLY:
#   C:\tmp\omniquad_newton_fixed2.xml (kp=500/kv=60) <-> OMNISIM_NEWTON_TARGET_KE=500/KD=60
#
# Gait reference: foot-space trot model (projects/policies/control/gait/omniquad_trot_gait.py)
# -- stance feet slide at exactly -vx, quintic swing, duty 0.6 four-foot
# overlap, IK-realized, stride ramps in from the model's own standing pose.
# Bare-model baseline in the trainer: NEVER falls in 40 s (512 envs).
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly.
#
# Usage:  bash scripts/dev/run_omniquad_walk_deploy.sh [duration] [--policy <onnx>] [--bare] [--cpu-engine]
#
# NOTE: bash sibling of run_omniquad_walk_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=240; POLICY=""; BARE=0; CPU_ENGINE=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --bare)       BARE=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_walk_deploy.omniworld"
if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_walk_main/policy.onnx"
fi
LOG="$ROOT/_scratch/omniquad_walk_deploy.log"
rm -f "$LOG"

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
# Joint drive MUST match the trainer MJCF (kp=500/kv=60).
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
if [ "$BARE" = "1" ]; then export OMNIQUAD_POLICY_ONNX="$ROOT/__no_policy__"; else export OMNIQUAD_POLICY_ONNX="$POLICY"; fi
# Gait config -- MUST match the policy's training run.
export OMNIQUAD_GAIT_VX="0.4"
export OMNIQUAD_GAIT_FREQ="1.4"
export OMNIQUAD_GAIT_DUTY="0.6"
export OMNIQUAD_GAIT_STEP_H="0.06"
export OMNIQUAD_GAIT_BODY_H="0.55"
export OMNIQUAD_GAIT_RAMP_S="1.0"
export OMNIQUAD_ACT_SCALE="0.15"
export OMNIQUAD_DEPLOY_LOG="$LOG"
export OMNIQUAD_DEPLOY_TRACE="1"

if [ "$BARE" = "1" ]; then echo "Running omniquad_walk_deploy.omniworld for ${DURATION}s wall (BARE gait model)..."
else                       echo "Running omniquad_walk_deploy.omniworld for ${DURATION}s wall (trot model + RL policy)..."; fi
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --duration "$DURATION" 2>&1 | tail -n 4 || true

echo
echo "===== OMNIQUAD WALK RESULT ====="
if [ -f "$LOG" ]; then
  FALL="$(grep -c "FALL" "$LOG" || true)"
  LAST="$(grep '^\[t=' "$LOG" | tail -n 1 || true)"
  if [ "${FALL:-0}" -eq 0 ]; then
    echo "  NO FALL. last: $LAST"
  else
    echo "  FELL: $(grep -m1 'FALL' "$LOG" || true)"
  fi
else
  echo "  (no deploy log produced)"
fi
