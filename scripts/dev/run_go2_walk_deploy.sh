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

# Run the Unitree Go2 WALK deploy world headless and report forward progress.
#
# THE OMNIQUAD/G1 WALK RECIPE PORTED TO GO2 (gait-residual RL on the
# stiffness-matched model). The trainer MJCF matches the deploy joint drive
# EXACTLY:
#   C:\tmp\go2_newton.xml (kp=250/kv=6) <-> OMNISIM_NEWTON_TARGET_KE=250/KD=6
#
# Gait reference: foot-space trot model (projects/policies/control/gait/go2_trot_gait.py)
# -- stance feet slide at exactly -vx, quintic swing, duty 0.6 four-foot
# overlap, IK-realized, stride ramps in from the model's own standing pose.
# Bare-model baseline (512 envs, KE=250): 100% upright, ~0.22 m/s.
#
# Engine: OMNISIM_NEWTON_MJWARP=1 matches the trainer engine exactly.
#
# Usage:  bash scripts/dev/run_go2_walk_deploy.sh [duration] [--policy <onnx>] [--bare] [--gui] [--realtime] [--cpu-engine]
#
# NOTE: bash sibling of run_go2_walk_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=240; POLICY=""; BARE=0; GUI=0; REALTIME=0; CPU_ENGINE=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --bare)       BARE=1; shift;;
    --gui)        GUI=1; shift;;
    --realtime)   REALTIME=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/go2_walk_deploy.omniworld"
if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_go2_walk_main/policy.onnx"
fi
LOG="$ROOT/_scratch/go2_walk_deploy.log"
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
# Joint drive MUST match the trainer MJCF (kp=250/kv=6).
export OMNISIM_NEWTON_TARGET_KE="250"
export OMNISIM_NEWTON_TARGET_KD="6"
if [ "$BARE" = "1" ]; then export GO2_POLICY_ONNX="$ROOT/__no_policy__"; else export GO2_POLICY_ONNX="$POLICY"; fi
# Gait config -- MUST match the policy's training run.
export GO2_GAIT_VX="0.4"
export GO2_GAIT_FREQ="1.8"
export GO2_GAIT_DUTY="0.6"
export GO2_GAIT_STEP_H="0.05"
export GO2_GAIT_BODY_H="0.30"
export GO2_GAIT_RAMP_S="1.0"
export GO2_ACT_SCALE="0.15"
export GO2_DEPLOY_LOG="$LOG"
export GO2_DEPLOY_TRACE="1"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
if [ "$BARE" = "1" ]; then echo "Running go2_walk_deploy.omniworld for ${DURATION}s wall (BARE gait model)..."
else                       echo "Running go2_walk_deploy.omniworld for ${DURATION}s wall (trot model + RL policy)..."; fi
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== GO2 WALK RESULT ====="
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
