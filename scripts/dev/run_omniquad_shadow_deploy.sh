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

# Run the OmniQuad SHADOWING walk deploy world headless and report progress.
#
# The ghost-lut sibling of run_omniquad_walk_deploy.ps1 (which stays unchanged for the
# legacy champion): the controller (omniquad_shadow_deploy) rides the CERTIFIED achieved
# ghost lut instead of the analytic trot, exactly as the SHADOWING trainer
# (quad_walk_recipe.py QUAD_GHOST=...) trained it.
#
# Engine env == run_omniquad_walk_deploy.ps1 byte-for-byte (KE=500/KD=60 -- OmniQuad's PD is
# an order of magnitude stiffer than the Go2's 250/6 -- SUBSTEPS=8, MJWARP,
# GROUND_MU=2.0) and the world is omniquad_walk_deploy.omniworld with ONLY the controller line
# changed, so the legacy-vs-Shadowing head-to-head is same-conditions.
#
# Usage:  bash scripts/dev/run_omniquad_shadow_deploy.sh [duration] [--policy <onnx>]
#             [--ghost <lut.json>] [--act-scale <corridor>] [--ff] [--bare]
#             [--gui] [--realtime] [--cpu-engine]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=240; POLICY=""; BARE=0; GUI=0; REALTIME=0; CPU_ENGINE=0; FF=0
GHOST="$ROOT/projects/policies/ghosts/omniquad/omniquad_shadow_ghost_lut.json"
ACT_SCALE="0.15"
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --ghost)      GHOST="$2"; shift 2;;
    --act-scale)  ACT_SCALE="$2"; shift 2;;
    --ff)         FF=1; shift;;
    --bare)       BARE=1; shift;;
    --gui)        GUI=1; shift;;
    --realtime)   REALTIME=1; shift;;
    --cpu-engine) CPU_ENGINE=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_shadow_deploy.omniworld"
if [ -z "$POLICY" ]; then
  # The SHIPPED champion, not a training-runs path: runs/ is gitignored, so a fresh
  # clone would find nothing there and the controller would refuse to run.
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_shadow_main/policy.onnx"
  if [ ! -f "$POLICY" ]; then
    POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_walk_main/policy.onnx"
  fi
  if [ ! -f "$POLICY" ]; then
    POLICY="$ROOT/projects/policies/training/runs/gpu_omniquad_shadow/policy.onnx"
  fi
fi
# multi-instance safe: honour a caller-supplied log path (parallel pods/runs). Two
# children sharing one log is the same class of bug as two omnisim-bin sharing
# omnisim_log.txt -- last writer wins and the run becomes unreadable.
LOG="${OMNIQUAD_DEPLOY_LOG:-$ROOT/_scratch/omniquad_shadow_deploy.log}"
mkdir -p "$(dirname "$LOG")"; rm -f "$LOG"

export OMNISIM_HOME="$ROOT"
# ---- engine: byte-for-byte run_omniquad_walk_deploy.ps1 (the legacy champion's physics) ----
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"          # fail loud, no silent ODE fallback
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"   # assert the MuJoCo solver, not XPBD
if [ "$CPU_ENGINE" = "1" ]; then unset OMNISIM_NEWTON_MJWARP; else export OMNISIM_NEWTON_MJWARP="1"; fi
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
# Joint drive MUST match the trainer MJCF (kp=500/kv=60) -- NOT the Go2's 250/6.
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
if [ "$BARE" = "1" ]; then export OMNIQUAD_POLICY_ONNX="$ROOT/__no_policy__"; else export OMNIQUAD_POLICY_ONNX="$POLICY"; fi
# ---- gait: MUST match the lut's own gait block + the training run ----
export OMNIQUAD_GAIT_VX="0.4"
export OMNIQUAD_GAIT_FREQ="1.4"
export OMNIQUAD_GAIT_DUTY="0.6"
export OMNIQUAD_GAIT_STEP_H="0.06"
export OMNIQUAD_GAIT_BODY_H="0.55"
export OMNIQUAD_GAIT_RAMP_S="1.0"
export OMNIQUAD_ACT_SCALE="$ACT_SCALE"
export OMNIQUAD_GHOST_LUT="$GHOST"
if [ "$FF" = "1" ]; then export OMNIQUAD_GHOST_FF="1"; fi
export OMNIQUAD_DEPLOY_LOG="$LOG"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
echo "Running omniquad_shadow_deploy.omniworld for ${DURATION}s wall (ghost lut + RL policy; corridor=$ACT_SCALE ff=$FF)..."
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== OMNIQUAD SHADOW WALK RESULT ====="
if [ ! -f "$LOG" ]; then echo "  (no deploy log produced)"; exit 1; fi
# ⛔ NEVER TRUST THE EXIT CODE. A ghost lut replays well enough on its own that a
# zero-residual run LOOKS good (it walks, it does not fall, and it scores a
# near-ceiling gmatch because it IS the ghost). A 2026-07-12 Go2 head-to-head was
# voided exactly this way -- onnxruntime was missing from the CONTROLLER interpreter.
if [ "$BARE" != "1" ] && ! grep -q "ONNX loaded:" "$LOG"; then
  echo "  ⛔ INVALID: the walk policy never loaded -- this run is a bare-ghost replay, not a"
  echo "     policy result. (See the 2026-07-12 voided head-to-head.)"
  grep -m2 "FATAL\|refusing\|not found" "$LOG" || true
  exit 3
fi
FALL="$(grep -c "FALL" "$LOG" || true)"
LAST="$(grep '^\[t=' "$LOG" | tail -n 1 || true)"
GM="$(grep 'GMATCH FINAL' "$LOG" | tail -n 1 || true)"
if [ "${FALL:-0}" -eq 0 ]; then
  echo "  NO FALL. last: $LAST"
else
  echo "  FELL: $(grep -m1 'FALL' "$LOG" || true)"
fi
[ -n "$GM" ] && echo "  $GM"
# gmatch is a POSE metric: it cannot see that the robot is upside down (a flipped Go2
# scored 0.92). Read the FALL line and the x-progress above, never gmatch alone.
exit 0
