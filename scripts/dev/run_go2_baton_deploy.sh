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

# THE QUADRUPED BATON DEMO -- walk -> stand -> walk, driven by the SAME policy-switching
# library that sequences the G1 (projects/policies/training/baton.py, imported UNCHANGED).
#
# This is the falsifier for "BATON is a library, not a humanoid deploy function":
#   * the host is a Webots CONTROLLER (separate process) -- there is no `world`
#   * policies are ONNX sessions -- there is no torch
#   * the channels are (glut, ref, ffdq) -- there is no arm or elbow to blend
#   * there is NO CRANE (every G1 BATON demo rides a weight-bearing rig; quads carry none)
#   * the support gate comes from the Go2 gait model's four-foot-support window
#
# The `stand` specialist has NO POLICY: a quadruped is statically stable on its nominal stance,
# so the hold is deterministic (zero residual) and needed no training at all.
#
# Engine env is byte-for-byte run_go2_shadow_deploy.sh's, so this is the same physics the
# Shadowing head-to-head was scored on.
#
# Usage:  bash scripts/dev/run_go2_baton_deploy.sh [duration] [--schedule "walk:12,stand:6,walk:12"]
#             [--policy <onnx>] [--morph <ticks>] [--gui] [--realtime]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
PY="${OMNISIM_PYTHON:-$(command -v python || command -v python3 || true)}"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }
"$PY" -c "import onnxruntime" >/dev/null 2>&1 || {
  echo "ERROR: controller Python '$PY' has no onnxruntime; refusing a bare-ghost run" >&2
  exit 1
}

DURATION=45; POLICY=""; GUI=0; REALTIME=0
# honour a caller-assembled schedule (skill_lib sequence exports BATON_SCHEDULE);
# --schedule still overrides. Default unchanged for the classic demo.
SCHEDULE="${BATON_SCHEDULE:-walk:12,stand:6,walk:12}"
MORPH="30"
ACT_SCALE="0.15"
GHOST="$ROOT/projects/policies/ghosts/go2/go2_shadow_ghost_lut.json"
STAND="$ROOT/projects/policies/ghosts/go2/go2_stand_ghost_lut.json"
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2;;
    --schedule) SCHEDULE="$2"; shift 2;;
    --policy)   POLICY="$2"; shift 2;;
    --morph)    MORPH="$2"; shift 2;;
    --gui)      GUI=1; shift;;
    --realtime) REALTIME=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

WORLD="$ROOT/projects/policies/research/worlds/go2_baton_deploy.omniworld"
if [ -z "$POLICY" ]; then
  POLICY="$ROOT/projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx"
fi
# multi-instance safe: honour a caller-supplied log path (parallel pods/runs)
LOG="${GO2_DEPLOY_LOG:-$ROOT/_scratch/go2_baton_deploy.log}"
export WARP_CACHE_PATH="${WARP_CACHE_PATH:-$ROOT/_scratch/warp_cache/go2_baton_$$}"
mkdir -p "$ROOT/_scratch" "$WARP_CACHE_PATH"; rm -f "$LOG"

export OMNISIM_HOME="$ROOT"
# ---- engine: byte-for-byte run_go2_shadow_deploy.sh (same physics as the head-to-head) ----
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_REQUIRE_NEWTON="1"
export OMNISIM_REQUIRE_MUJOCO_SOLVER="1"
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_GROUND_MU="2.0"
export OMNISIM_NEWTON_SUBSTEPS="8"
export OMNISIM_NEWTON_TARGET_KE="250"
export OMNISIM_NEWTON_TARGET_KD="6"
# ---- gait: MUST match the lut's own gait block + the training run ----
export GO2_GAIT_VX="0.4"
export GO2_GAIT_FREQ="1.8"
export GO2_GAIT_DUTY="0.6"
export GO2_GAIT_STEP_H="0.05"
export GO2_GAIT_BODY_H="0.30"
export GO2_GAIT_RAMP_S="1.0"
export GO2_ACT_SCALE="$ACT_SCALE"
export GO2_GHOST_FF="1"          # the Go2 corridor law: the ghost is untrackable without it
# ---- BATON (the library reads exactly these, for every robot) ----
export GO2_GHOST_LUT="$GHOST"
export GO2_STAND_LUT="$STAND"
export GO2_POLICY_ONNX="$POLICY"
export BATON_SCHEDULE="$SCHEDULE"
export BATON_MORPH_TICKS="$MORPH"
export BATON_DS_GATE="1"         # ⛔ WAS 0 ("a trotter has no forbidden instant") -- a live run
                                 # REFUTED that: a walk:12 switch landed mid-swing and flipped the
                                 # robot onto its back. The host gates on FOUR-FOOT SUPPORT.
# honour a caller-assembled specialist list (e.g. skill_lib sequence adds `turn` with
# its ONNX). Default = the classic stand-only hold. Empty ckpt == DETERMINISTIC HOLD.
export BATON_SPECIALISTS="${BATON_SPECIALISTS:-stand||$STAND|0}"
export GO2_DEPLOY_LOG="$LOG"

RUN_ARGS=("$WORLD" --duration "$DURATION")
[ "$GUI" = "1" ] && RUN_ARGS+=(--gui)
[ "$REALTIME" = "1" ] && RUN_ARGS+=(--realtime)
echo "Go2 BATON: schedule=$SCHEDULE morph=$MORPH corridor=$ACT_SCALE  (${DURATION}s wall)"
"$PY" -u "$ROOT/scripts/dev/headless_runner.py" "${RUN_ARGS[@]}" 2>&1 | tail -n 4 || true

echo
echo "===== GO2 BATON RESULT ====="
if [ ! -f "$LOG" ]; then echo "  (no deploy log produced)"; exit 1; fi
# ⛔ NEVER TRUST THE EXIT CODE: a bare ghost walks and scores a near-ceiling gmatch.
if ! grep -q "ONNX loaded:" "$LOG"; then
  echo "  ⛔ INVALID: the walk policy never loaded -- this run is a bare-ghost replay, not a"
  echo "     policy result. (See the 2026-07-12 voided head-to-head.)"
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
