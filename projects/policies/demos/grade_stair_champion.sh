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

# GRADE A STAIR CHAMPION on BOTH rulers (motion-legitimacy doctrine, 2026-07-11):
#   1. kinematic  -- certify_stair_human.py  (heading, placement, steps, summit stand)
#   2. legitimacy -- verify_motion_legitimacy.py (L0 upright, L1 crane, L2 contact purity,
#                    L3 feet support), on a fully instrumented live run.
# Usage:
#   bash projects/policies/demos/grade_stair_champion.sh <ckpt.pt> [lam=0.4] [runs=3]
# Exit 0 iff at least one run PASSES BOTH rulers. Each run's verdict is printed.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
CKPT="${1:?usage: grade_stair_champion.sh <ckpt.pt> [lam] [runs]}"
LAM="${2:-0.4}"; RUNS="${3:-3}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
PASS=0
for N in $(seq 1 "$RUNS"); do
  TAG=grade$N; MPC=_scratch/foot_redesign/${TAG}_mpc.txt
  for TRY in 1 2 3; do
    bash projects/policies/training/run_walk_rl.sh 150 "$TAG" deploy headless \
      PYTHONUNBUFFERED=1 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
      OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
      WALK_WORLD=projects/policies/worlds/g1_climb_stairs_demo3.omniworld \
      WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.72 STAND_POSE=unitree WALK_WARM_TICKS=30 WALK_MJW_RESET=0 \
      POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 REF_OBS=1 REF_OBS_K=3 \
      GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_METRIC_JSON=$G/ghost_official_full_v3_lut.json \
      GHOST_RESIDUAL=0.22 GHOST_RESIDUAL_LAT=0.30 ARM_RESIDUAL=0.10 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
      SHRY_TARGET=0.15 SHRY_RESIDUAL=0.10 \
      W_TRACK_LIN=8 W_HEIGHT=0 SWING_H=0.13 W_LINK=0 Z_TGT=0.72 \
      HARNESS_LAM0="$LAM" HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_ATT_GHOST=1 HARNESS_KZ=0 \
      HEADING_TARGET=0 HARNESS_KYAW=80 VX_MAX=0.35 \
      FOOT_LOG=1 FOOT_LOG_EVERY=6 CONTACT_LOG=1 CONTACT_LOG_ALL=1 CONTACT_LOG_EVERY=4 CRANE_LOG=1 CRANE_LOG_EVERY=4 \
      RES_POLICY="$CKPT" >/dev/null 2>&1
    grep -q FOOTLOG "$MPC" 2>/dev/null && break
  done
  echo "========== RUN $N (lam=$LAM) =========="
  KIN=1; LEG=1
  python projects/policies/training/certify_stair_human.py "$MPC" 1.2 && KIN=0
  python projects/policies/training/verify_motion_legitimacy.py "$MPC" 1.0 2.6 6,7,12,13 && LEG=0
  if [ "$KIN" = 0 ] && [ "$LEG" = 0 ]; then
    echo "RUN $N: DOUBLE PASS"; PASS=1
  else
    echo "RUN $N: kinematic=$([ $KIN = 0 ] && echo PASS || echo FAIL) legitimacy=$([ $LEG = 0 ] && echo PASS || echo FAIL)"
  fi
done
[ "$PASS" = 1 ] && { echo "CHAMPION: at least one DOUBLE-PASS run"; exit 0; }
echo "CHAMPION: no double-pass in $RUNS runs"; exit 1
