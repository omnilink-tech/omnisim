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

# Run the OmniQuad HILL-WALK deploy world: the physically-simulated OmniQuad (Newton)
# shadows its hill ghost up a 15-deg slope, over the crest, and down -- with the
# statically-stable CRAWL gait. Mirrors run_b2_hill_deploy.sh (OmniQuad gains/gait).
#
# Usage:  bash scripts/dev/run_omniquad_hill_deploy.sh [duration] [--policy <onnx>] [--gui] [--realtime] [--bare]
#
# NOTE: bash sibling of run_omniquad_hill_deploy.ps1 (the Windows launcher) --
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

WORLD="$ROOT/projects/policies/research/worlds/omniquad_hill_deploy.omniworld"
GHOST="$ROOT/projects/policies/research/shadowing/ghosts/omniquad_hill_ghost.npz"
if [ -z "$POLICY" ]; then POLICY="$ROOT/projects/policies/research/training/runs/gpu_omniquad_hill_h100_15/policy.onnx"; fi
LOG="$ROOT/_scratch/omniquad_hill_deploy.log"
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
export OMNISIM_NEWTON_TARGET_KE="500"      # MUST match the trainer MJCF (omniquad)
export OMNISIM_NEWTON_TARGET_KD="60"
if [ "$BARE" = "1" ]; then export OMNIQUAD_POLICY_ONNX="$ROOT/__no_policy__"; else export OMNIQUAD_POLICY_ONNX="$POLICY"; fi
export OMNIQUAD_HILL_GHOST="$GHOST"            # slope-conditioned obs (+2)
export OMNIQUAD_HILL_LOOKAHEAD="0.4"           # MUST match training --hill-lookahead
# CRAWL gait config -- MUST match the hill training run.
export OMNIQUAD_GAIT_MODULE="projects.policies.control.gait.omniquad_crawl_gait"
export OMNIQUAD_GAIT_VX="0.35"
export OMNIQUAD_GAIT_FREQ="1.2"
export OMNIQUAD_GAIT_DUTY="0.85"
export OMNIQUAD_GAIT_STEP_H="0.06"
export OMNIQUAD_GAIT_BODY_H="0.55"
export OMNIQUAD_GAIT_RAMP_S="1.0"
export OMNIQUAD_ACT_SCALE="0.18"
export OMNIQUAD_DEPLOY_LOG="$LOG"
export HILL_ROBOT="omniquad"
export HILL_GHOST_NPZ="$GHOST"
export HILL_GHOST_ALPHA="0.5"

RUN_ARGS=("$WORLD" --duration "$DURATION")
if [ "$GUI" = "1" ]; then RUN_ARGS+=(--gui); fi
if [ "$REALTIME" = "1" ]; then RUN_ARGS+=(--realtime); fi
if [ "$BARE" = "1" ]; then DESC="BARE crawl"; else DESC="crawl + RL hill policy"; fi
echo "Running omniquad_hill_deploy.omniworld for ${DURATION}s ($DESC)..."
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== OMNIQUAD HILL-WALK RESULT ====="
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
