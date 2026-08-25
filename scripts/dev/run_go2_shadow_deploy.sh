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

# Run the Unitree Go2 SHADOWING walk deploy world headless and report progress.
#
# The ghost-lut sibling of run_go2_walk_deploy.sh (which stays unchanged for the
# legacy champion): the controller (go2_shadow_deploy) rides the CERTIFIED
# achieved ghost lut (build_go2_shadow_ghost.py; ghost_validator PASS) instead
# of the analytic trot, exactly as the SHADOWING trainer
# (quad_walk_recipe.py QUAD_GHOST=...) trained it.
#
# Engine env == run_go2_walk_deploy.sh byte-for-byte (KE=250/KD=6, SUBSTEPS=8,
# MJWARP, GROUND_MU=2.0) so the head-to-head exam is same-conditions.
#
# Usage:  bash scripts/dev/run_go2_shadow_deploy.sh [duration] [--policy <onnx>]
#             [--ghost <lut.json>] [--act-scale <corridor>] [--ff] [--bare]
#             [--gui] [--realtime] [--cpu-engine]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=240; POLICY=""; BARE=0; GUI=0; REALTIME=0; CPU_ENGINE=0; FF=0
GHOST="$ROOT/projects/policies/ghosts/go2/go2_shadow_ghost_lut.json"
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

# ⛔ The CONTROLLER runs with its OWN cwd, not $ROOT -- a relative --ghost/--policy silently
# becomes "not found", the controller exits 1, the robot never moves, and the summary below still
# says "NO FALL" because there are no FALL lines. Resolve to absolute here.
case "$GHOST" in /*|?:*) ;; *) GHOST="$ROOT/$GHOST";; esac
case "$POLICY" in ""|/*|?:*) ;; *) POLICY="$ROOT/$POLICY";; esac
# World is overridable (GO2_SHADOW_WORLD) so the SAME shadow policy + engine env can run on a
# different scene -- e.g. the rough-terrain track -- without duplicating the recipe. Any override
# must spawn the go2 URDFRobot at the same pose and run the go2_shadow_deploy controller.
WORLD="${GO2_SHADOW_WORLD:-$ROOT/projects/policies/research/worlds/go2_shadow_deploy.omniworld}"
if [ -z "$POLICY" ]; then
  # The SHIPPED champion (shD), not a training-runs path: runs/ is gitignored, so
  # a fresh clone would find nothing there and the controller would refuse to run.
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx"
  if [ ! -f "$POLICY" ]; then
    POLICY="$ROOT/projects/policies/training/runs/gpu_go2_shadow/policy.onnx"
  fi
fi
LOG="$ROOT/_scratch/go2_shadow_deploy.log"
rm -f "$LOG"

export OMNISIM_HOME="$ROOT"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"
if [ "$CPU_ENGINE" = "1" ]; then unset OMNISIM_NEWTON_MJWARP; else export OMNISIM_NEWTON_MJWARP="1"; fi
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
export OMNISIM_NEWTON_TARGET_KE="250"
export OMNISIM_NEWTON_TARGET_KD="6"
if [ "$BARE" = "1" ]; then export GO2_POLICY_ONNX="$ROOT/__no_policy__"; else export GO2_POLICY_ONNX="$POLICY"; fi
# Gait envs -- MUST match the lut's own gait block + the training run.
export GO2_GAIT_VX="0.4"
export GO2_GAIT_FREQ="1.8"
export GO2_GAIT_DUTY="0.6"
export GO2_GAIT_STEP_H="0.05"
export GO2_GAIT_BODY_H="0.30"
export GO2_GAIT_RAMP_S="1.0"
export GO2_ACT_SCALE="$ACT_SCALE"
export GO2_GHOST_LUT="$GHOST"
if [ "$FF" = "1" ]; then export GO2_GHOST_FF="1"; fi
export GO2_DEPLOY_LOG="$LOG"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
echo "Running go2_shadow_deploy.omniworld for ${DURATION}s wall (ghost lut + RL policy; corridor=$ACT_SCALE ff=$FF)..."
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== GO2 SHADOW WALK RESULT ====="
# ⛔ NEVER report a run that never ran. No per-tick trace == the controller died on startup
# (missing lut/policy), and "NO FALL" on an empty log is a LIE.
if [ -f "$LOG" ] && ! grep -q '^\[t=' "$LOG"; then
  echo "  ⛔ INVALID: the controller produced no tick trace -- it never stepped. Cause:"
  head -3 "$LOG" | sed 's/^/     /'
  exit 3
fi
if [ -f "$LOG" ] && [ "$BARE" != "1" ] && ! grep -q 'ONNX loaded:' "$LOG"; then
  echo "  ⛔ INVALID: the policy never loaded -- this would be a BARE-GHOST replay reported as a"
  echo "     policy result (the trap that voided the 2026-07-12 head-to-head)."
  exit 3
fi
if [ -f "$LOG" ]; then
  FALL="$(grep -c "FALL" "$LOG" || true)"
  LAST="$(grep '^\[t=' "$LOG" | tail -n 1 || true)"
  GM="$(grep 'GMATCH FINAL' "$LOG" | tail -n 1 || true)"
  if [ "${FALL:-0}" -eq 0 ]; then
    echo "  NO FALL. last: $LAST"
  else
    echo "  FELL: $(grep -m1 'FALL' "$LOG" || true)"
  fi
  [ -n "$GM" ] && echo "  $GM"
else
  echo "  (no deploy log produced)"
fi
# A FELL run is still a VALID run (the fall is the honest result). Without this,
# the trailing `[ -n "$GM" ] && echo` above made every fallen-early run (no
# 'GMATCH FINAL' line) exit 1 -- an exit-code lie that would abort any caller
# treating rc!=0 as "leg invalid" (the terrain campaign's baseline exam does
# exactly that, and the baseline is SUPPOSED to fall). Invalid runs still exit 3.
exit 0
