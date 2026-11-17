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

# Run the Unitree B2 HILL-WALK deploy world: the PHYSICALLY-SIMULATED B2 (Newton)
# shadows its hill ghost up a 15-deg slope, over the crest, and down -- beside the
# translucent ideal ghost. Same recipe as run_b2_walk_deploy.sh (trot model + RL
# residual on the stiffness-matched MJCF) plus:
#   * OMNISIM_NEWTON_STATICS=1  -> the hill boxes are COLLIDABLE static colliders
#   * B2_HILL_GHOST=<npz>       -> the slope-conditioned obs (target pitch + height)
#   * gait vx=0.35              -> the hill ghost's climb speed
#
# Usage:  bash scripts/dev/run_b2_hill_deploy.sh [duration] [--policy <onnx>] [--gui] [--realtime] [--bare]
#
# NOTE: bash sibling of run_b2_hill_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=60; POLICY=""; BARE=0; GUI=0; REALTIME=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2;;
    --policy)   POLICY="$2"; shift 2;;
    --bare)     BARE=1; shift;;
    --gui)      GUI=1; shift;;
    --realtime) REALTIME=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/b2_hill_deploy.omniworld"
GHOST="$ROOT/projects/policies/research/shadowing/ghosts/b2_hill_ghost.npz"
if [ -z "$POLICY" ]; then POLICY="$ROOT/projects/policies/research/training/runs/gpu_b2_hill_h100_15/policy.onnx"; fi
LOG="$ROOT/_scratch/b2_hill_deploy.log"
rm -f "$LOG"

export OMNISIM_HOME="$ROOT"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"          # assert Newton engaged -- fail loud, no silent ODE fallback
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_NEWTON_STATICS="1"          # collidable hill boxes
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
export OMNISIM_NEWTON_TARGET_KE="1400"     # MUST match the trainer MJCF
export OMNISIM_NEWTON_TARGET_KD="35"
if [ "$BARE" = "1" ]; then export B2_POLICY_ONNX="$ROOT/__no_policy__"; else export B2_POLICY_ONNX="$POLICY"; fi
export B2_HILL_GHOST="$GHOST"              # enables the slope-conditioned obs (+2)
export B2_HILL_LOOKAHEAD="0.4"             # MUST match training --hill-lookahead
# CRAWL gait config -- MUST match the hill training run (statically stable on slopes).
export B2_GAIT_MODULE="projects.policies.control.gait.b2_crawl_gait"
export B2_GAIT_VX="0.35"
export B2_GAIT_FREQ="1.0"
export B2_GAIT_DUTY="0.85"
export B2_GAIT_STEP_H="0.10"
export B2_GAIT_BODY_H="0.50"
export B2_GAIT_RAMP_S="1.0"
export B2_ACT_SCALE="0.18"
export B2_DEPLOY_LOG="$LOG"
# the translucent ghost alongside (kinematic replay)
export HILL_ROBOT="b2"
export HILL_GHOST_NPZ="$GHOST"
export HILL_GHOST_ALPHA="0.5"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
if [ "$BARE" = "1" ]; then DESC="BARE gait"; else DESC="trot + RL hill policy"; fi
echo "Running b2_hill_deploy.omniworld for ${DURATION}s ($DESC)..."
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== B2 HILL-WALK RESULT ====="
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
