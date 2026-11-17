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

# VELOCITY-CONDITIONED Go2 walk deploy + the time-aware WALK/STOP/WALK
# milestone (the OmniQuad/G1 walk29_vc recipe, ported to Go2).
#
# One policy (gpu_go2_walk_vc_main) trained with a commandable forward speed
# (--vx-cmd-max 0.45) INCLUDING 0 = stand. The deploy appends the normalised
# command to the obs (GO2_VX_CMD_MAX) and drives it from the schedule's stride
# scale _w: full trot at _w=1, stand on four feet at _w=0. Default schedule is
# "walk a bit, stop a bit, repeat" -- walk GO2_WALK_FOR_S, stand
# GO2_STAND_FOR_S, repeating; the SAME policy decelerates + stands, then
# resumes. For a quadruped the stand is statically stable.
#
# Usage:
#   bash scripts/dev/run_go2_walk_vc_deploy.sh [duration] [--walk-for 6] [--stand-for 4] [--blend-s 1] [--policy <onnx>] [--gui] [--realtime] [--cpu-engine]
#   --walk-for 0     ->  no stop, pure continuous walk
#   --gui --realtime ->  open a live OmniSim window paced 1.0x to WATCH it
#
# NOTE: bash sibling of run_go2_walk_vc_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=40; WALKFOR="6"; STANDFOR="4"; BLENDS="1"; POLICY=""; GUI=0; REALTIME=0; CPU_ENGINE=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --walk-for)   WALKFOR="$2"; shift 2;;
    --stand-for)  STANDFOR="$2"; shift 2;;
    --blend-s)    BLENDS="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --gui)        GUI=1; shift;;
    --realtime)   REALTIME=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_go2_walk_vc_main/policy.onnx"
fi
if [ ! -f "$POLICY" ]; then echo "missing $POLICY"; exit 1; fi
WORLD="$ROOT/projects/policies/research/worlds/go2_walk_deploy.omniworld"
LOG="$ROOT/_scratch/go2_walk_vc_deploy.log"
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
export OMNISIM_NEWTON_TARGET_KE="250"
export OMNISIM_NEWTON_TARGET_KD="6"
export GO2_POLICY_ONNX="$POLICY"
# Gait config -- MUST match the policy's training run.
export GO2_GAIT_VX="0.4"; export GO2_GAIT_FREQ="1.8"; export GO2_GAIT_DUTY="0.6"
export GO2_GAIT_STEP_H="0.05"; export GO2_GAIT_BODY_H="0.30"; export GO2_GAIT_RAMP_S="1.0"
export GO2_ACT_SCALE="0.15"; export GO2_SETTLE_S="1.5"
# Velocity conditioning + the time-aware walk<->stop schedule.
export GO2_VX_CMD_MAX="0.45"          # MUST match trainer --vx-cmd-max
export GO2_WALK_FOR_S="$WALKFOR"      # 0 = continuous walk (no stop)
export GO2_STAND_FOR_S="$STANDFOR"
export GO2_MODE_BLEND_S="$BLENDS"
export GO2_DEPLOY_LOG="$LOG"
export GO2_DEPLOY_TRACE="1"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
if awk "BEGIN{exit !($WALKFOR > 0)}"; then
  echo "Running ${DURATION}s: walk ${WALKFOR}s / STAND ${STANDFOR}s, repeating (velocity-conditioned)..."
else
  echo "Running ${DURATION}s continuous walk (vx=0.4, no stop)..."
fi
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== GO2 VC WALK/STOP RESULT ====="
if [ -f "$LOG" ]; then
  FALL="$(grep -c "FALL" "$LOG" || true)"
  if [ "${FALL:-0}" -eq 0 ]; then
    echo "  NO FALL."
  else
    echo "  FELL: $(grep -m1 'FALL' "$LOG" || true)"
  fi
  # show the speed at a couple of walk and stand moments
  grep '^\[t=' "$LOG" | grep -E 't=(2|5|8|12|15)s' | sed 's/^/  /' || true
fi
