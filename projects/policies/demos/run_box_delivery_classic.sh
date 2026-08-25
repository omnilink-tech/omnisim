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

# THE CLASSIC BOX-DELIVERY DEMO (owner's working baseline, restored 2026-07-08; renamed
# box_delivery -> box_delivery_classic 2026-07-10 when the corner+hand-track demo took the name).
# The G1 walks to the cart, stands, PICKS the real 1.5 kg box, CARRIES it along its
# natural arc to the second cart, PLACES it (real contact physics), walks on, stands.
# BATON walk<->stand<->carry blend + crane-yaw steering (the documented dead-heading-
# channel workaround) + the real ODE box (harness_rig moves it off the mode file).
# Verified 0 falls. Run: bash projects/policies/demos/run_box_delivery_classic.sh [dur] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
DUR="${1:-220}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
MODEF="${BATON_MODE_FILE:-$ROOT/_scratch/foot_redesign/boxmode.txt}"
mkdir -p "$(dirname "$MODEF")"
echo "parked" > "$MODEF"
bash projects/policies/training/run_walk_rl.sh "$DUR" box_delivery_classic deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=projects/policies/worlds/g1_walk_puppet.omniworld \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 \
  RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 \
  "BATON_SPECIALISTS=carry|$R/wr_carrier.pt|$G/ghost_carry_v1_lut.json|0.45;stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "BATON_COURSE=walkto,4.0,-0.25;stand,3;carryto,8.2,-3.3;stand,5;walkto,9.6,-4.1;stand,0" \
  BATON_COLD_HIDDEN=1 BATON_MORPH_TICKS=30 \
  "BATON_MODE_FILE=$MODEF" \
  STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256
