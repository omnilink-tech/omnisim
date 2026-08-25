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

# WATCH OmniQuad walk live in a real OmniSim window.
#
# The G1 GUI launcher pattern: headless_runner.py --gui so the full
# Newton/mujoco_warp env config (the engine the policy was trained on)
# propagates reliably while a real 3D window opens. The foot-space trot
# model + RL residual drive the legs; the heading hold keeps it straight.
# Paced at 1.0x wall clock (--realtime); pass --fast to run unpaced.
#
# Config MUST match scripts/dev/run_omniquad_walk_deploy.sh / .ps1.
#
# Usage:  bash scripts/dev/run_omniquad_walk_deploy_gui.sh [duration] [--fast] [--policy <onnx>]
#
# NOTE: bash sibling of run_omniquad_walk_deploy_gui.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=120; FAST=0; POLICY=""
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2;;
    --fast)     FAST=1; shift;;
    --policy)   POLICY="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_walk_deploy.omniworld"
if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_walk_main/policy.onnx"
fi

export OMNISIM_HOME="$ROOT"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"          # assert Newton engaged -- fail loud, no silent ODE fallback
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"   # assert Newton MuJoCo solver (not XPBD) -- the trained engine
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
# Joint drive MUST match the trainer MJCF (kp=500/kv=60).
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
export OMNIQUAD_POLICY_ONNX="$POLICY"
# Gait config -- MUST match the policy's training run.
export OMNIQUAD_GAIT_VX="0.4"
export OMNIQUAD_GAIT_FREQ="1.4"
export OMNIQUAD_GAIT_DUTY="0.6"
export OMNIQUAD_GAIT_STEP_H="0.06"
export OMNIQUAD_GAIT_BODY_H="0.55"
export OMNIQUAD_GAIT_RAMP_S="1.0"
export OMNIQUAD_ACT_SCALE="0.15"

echo "Opening omniquad_walk_deploy.omniworld in the GUI. Watch OmniQuad trot; close the window to stop."
if [ "$FAST" = "1" ]; then
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --duration "$DURATION"
else
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --realtime --duration "$DURATION"
fi
