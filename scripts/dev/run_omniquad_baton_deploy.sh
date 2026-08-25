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

# THE SECOND QUADRUPED BATON DEMO -- walk -> stand -> walk on OmniQuad, driven by
# the SAME policy-switching library that sequences the G1 and the Go2
# (projects/policies/training/baton.py, imported UNCHANGED).
#
# run_go2_baton_deploy.sh proved BATON survives leaving the humanoid. This proves it was not
# then quietly re-fitted to the Go2: OmniQuad is a different quadruped in every constant the host
# touches --
#   * joint names carry NO "_joint" suffix (front_left_hip_x, not FL_hip_joint)
#   * limits: hip_x +-1.50, hip_y [-0.50,+3.13], knee [-1.20,-0.01]
#   * a 2x-taller stance (body_height 0.55 vs 0.30) and a slower clock (1.4 vs 1.8 Hz)
#   * an order-of-magnitude stiffer PD: KE=500/KD=60, vs the Go2's 250/6
# ...and the library still took it with zero changes. The whole robot-specific surface is the
# ~60-line OmniQuadBatonHost.
#
# The `stand` specialist has NO POLICY: a quadruped is statically stable on its nominal stance,
# so the hold is deterministic (zero residual) and needed no training at all. And there is NO
# CRANE -- every G1 BATON demo rides a weight-bearing rig; quads carry none.
#
# ⛔ THE SUPPORT GATE IS NOT OPTIONAL. BATON's naive quadruped default (`always_ok`) was REFUTED
# live on the Go2: a walk:12 switch landed mid-swing and flipped the robot onto its back, which
# it then "walked" while still scoring gmatch 0.92 -- a pose metric cannot see that you are
# upside down. OmniQuad's trot also runs duty=0.6 > 0.5, so it too has four-foot support windows;
# the host gates on them (BATON_DS_GATE=1 + OmniQuadBatonHost.support_gate).
#
# Engine env is byte-for-byte run_omniquad_shadow_deploy.sh's, and the world is
# omniquad_walk_deploy.omniworld with ONLY the controller line changed, so this is the same physics the
# Shadowing head-to-head is scored on.
#
# Usage:  bash scripts/dev/run_omniquad_baton_deploy.sh [duration] [--schedule "walk:12,stand:6,walk:12"]
#             [--policy <onnx>] [--ghost <lut>] [--stand-lut <lut>] [--morph <ticks>]
#             [--gui] [--realtime]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

DURATION=45; POLICY=""; GUI=0; REALTIME=0
SCHEDULE="walk:12,stand:6,walk:12"
MORPH="30"
ACT_SCALE="0.15"
GHOST="$ROOT/projects/policies/ghosts/omniquad/omniquad_shadow_ghost_lut.json"
STAND="$ROOT/projects/policies/ghosts/omniquad/omniquad_stand_ghost_lut.json"
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)   DURATION="$2"; shift 2;;
    --schedule)   SCHEDULE="$2"; shift 2;;
    --policy)     POLICY="$2"; shift 2;;
    --ghost)      GHOST="$2"; shift 2;;
    --stand-lut)  STAND="$2"; shift 2;;
    --act-scale)  ACT_SCALE="$2"; shift 2;;
    --morph)      MORPH="$2"; shift 2;;
    --gui)        GUI=1; shift;;
    --realtime)   REALTIME=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/omniquad_baton_deploy.omniworld"
if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_shadow_main/policy.onnx"
  if [ ! -f "$POLICY" ]; then
    POLICY="$ROOT/projects/policies/research/inference/policies/gpu_omniquad_walk_main/policy.onnx"
  fi
fi
# multi-instance safe: honour a caller-supplied log path (parallel pods/runs)
LOG="${OMNIQUAD_DEPLOY_LOG:-$ROOT/_scratch/omniquad_baton_deploy.log}"
mkdir -p "$(dirname "$LOG")"; rm -f "$LOG"

export OMNISIM_HOME="$ROOT"
# ---- engine: byte-for-byte run_omniquad_shadow_deploy.sh (same physics as the head-to-head) ----
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
# OmniQuad's joint drive: kp=500/kv=60 (the Go2's is 250/6 -- do NOT copy that here).
export OMNISIM_NEWTON_TARGET_KE="500"
export OMNISIM_NEWTON_TARGET_KD="60"
# ---- gait: MUST match the lut's own gait block + the training run ----
export OMNIQUAD_GAIT_VX="0.4"
export OMNIQUAD_GAIT_FREQ="1.4"
export OMNIQUAD_GAIT_DUTY="0.6"
export OMNIQUAD_GAIT_STEP_H="0.06"
export OMNIQUAD_GAIT_BODY_H="0.55"
export OMNIQUAD_GAIT_RAMP_S="1.0"
export OMNIQUAD_ACT_SCALE="$ACT_SCALE"
export OMNIQUAD_GHOST_FF="1"         # the quad corridor law: a ghost recorded under tau_ff is
                                 # untrackable by a pure position PD unless the corridor
                                 # exceeds tau_ff/kp -- replay the feedforward instead.
# ---- BATON (the library reads exactly these, for every robot) ----
export OMNIQUAD_GHOST_LUT="$GHOST"
export OMNIQUAD_STAND_LUT="$STAND"
export OMNIQUAD_POLICY_ONNX="$POLICY"
export BATON_SCHEDULE="$SCHEDULE"
export BATON_MORPH_TICKS="$MORPH"
export BATON_DS_GATE="1"         # ⛔ gate the hand-over on FOUR-FOOT SUPPORT. A live Go2 run
                                 # REFUTED the "a trotter has no forbidden instant" default:
                                 # a walk:12 switch landed mid-swing and flipped the robot.
export OMNIQUAD_SUPPORT_TOL="0.05"   # max per-leg swing weight that still counts as "planted"
export BATON_SPECIALISTS="stand||$STAND|0"   # empty ckpt == a DETERMINISTIC HOLD (no policy)
export OMNIQUAD_DEPLOY_LOG="$LOG"

RUN_ARGS=("$WORLD" --duration "$DURATION")
[ "$GUI" = "1" ] && RUN_ARGS+=(--gui)
[ "$REALTIME" = "1" ] && RUN_ARGS+=(--realtime)
echo "OmniQuad BATON: schedule=$SCHEDULE morph=$MORPH corridor=$ACT_SCALE  (${DURATION}s wall)"
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== OMNIQUAD BATON RESULT ====="
if [ ! -f "$LOG" ]; then echo "  (no deploy log produced)"; exit 1; fi
# ⛔ NEVER TRUST THE EXIT CODE: a bare ghost walks and scores a near-ceiling gmatch, so a run
# whose policy never loaded looks like a GOOD result. Assert the load, loudly.
if ! grep -q "ONNX loaded:" "$LOG"; then
  echo "  ⛔ INVALID: the walk policy never loaded -- this run is a bare-ghost replay, not a"
  echo "     policy result. (See the 2026-07-12 voided head-to-head: the broken run looked"
  echo "     BETTER than the real one.) Fix the CONTROLLER interpreter's deps:"
  echo "     pip install onnxruntime   (the python that runs the controller, not the engine's)"
  grep -m2 "FATAL\|refusing" "$LOG" || true
  exit 3
fi
grep -m1 "armed:" "$LOG" || true
grep "BATON switch" "$LOG" || echo "  ⛔ NO SWITCHES -- the arbiter never handed over."
echo "  --- per-second trace (mode/u/switches):"
grep '^\[t=' "$LOG" | awk 'NR%4==1' | tail -n 12
echo "  --- verdict:"
if grep -q "FALL" "$LOG"; then echo "  FELL: $(grep -m1 FALL "$LOG")"; else echo "  NO FALL."; fi
grep "BATON FINAL" "$LOG" || true
grep "  segment" "$LOG" || true
