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

# BOX DELIVERY v3 (2026-07-19): physical suction pick-and-place with rolling clearance arcs.
# The G1 walks to cart A, establishes a sub-centimeter suction seal, and lifts/carries the
# untouched Newton body under finite forces. It presses the box onto cart B, releases it, and
# lets contact physics settle it. The carrier follows a forward arc around cart A; after placing,
# the navigator moves south away from cart B and then stays in forward locomotion through a wide
# open-floor U-arc. Reverse and stationary-turn specialists are intentionally excluded because
# full-course verification showed that they do not preserve stable travel under the live frames.
# Heading changes happen under ordinary footfalls and no pivot is attempted beside either cart.
# Run: bash projects/policies/demos/run_box_delivery.sh [dur] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
DUR="${1:-900}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
MODEF="${BATON_MODE_FILE:-$ROOT/_scratch/foot_redesign/boxmode.txt}"
TAG="${TAG:-box_delivery}"
mkdir -p "$(dirname "$MODEF")"
echo "parked" > "$MODEF"
bash projects/policies/training/run_walk_rl.sh "$DUR" "$TAG" deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0 OMNISIM_NEWTON_DISABLE_ISLAND=1 WALK_WORLD=projects/policies/worlds/g1_box_grasp.omniworld \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 GHOST_OMEGA_SCALE=2.0 \
  FOOT_HEADING_LOCK=1 FOOT_HEADING_TRACK_TARGET=1 FOOT_HEADING_TRIM_SIGN=-1 \
  FOOT_HEADING_KP=0.3 FOOT_HEADING_KD=0.02 FOOT_HEADING_TRIM_CAP=0.06 \
  WALK_DIAG=1 ACT_AUTH=1 \
  RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 \
  "BATON_SPECIALISTS=carry|$R/wr_carrier.pt|$G/ghost_carry_v1_lut.json|0.45;stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "BATON_COURSE=walkto,4.3,-0.98;stand,6;carryto,5.3,-2.1;carryto,7.8,-1.42;carryto,8.55,-1.61;carryto,9.3,-1.62;stand,6;walkto,8.70,-2.70;walkto,11.0,-2.70;walkto,12.0,-1.80;walkto,12.0,0.0;walkto,11.4,1.0;walkto,10.7,1.9;stand,0" \
  "OMNISIM_DONE_PATTERN=BATON-(COURSE DONE|COURSE FELL)" \
  "OMNISIM_SUCCESS_PATTERN=BATON-CYCLE k=.* ok=1" \
  BATON_COLD_HIDDEN=1 BATON_MORPH_TICKS=30 BATON_STAND_PIVOT=0 \
  "BATON_MODE_FILE=$MODEF" \
  HSTAND_WARMUP_RELOAD=0 \
  GHOST_FOLLOW=1 HAND_TRACK=1 \
  PHYS_GRASP=1 GR_PICK_SEG=1 GR_PLACE_SEG=6 \
  GR_SUCTION=1 GR_SUCTION_R=0.010 GR_SUCTION_DROP=0.0 GR_PAD_X=0.10 GR_PAD_Y=0.13 GR_CLOSE_TICKS=100 \
  GR_PREPICK_FIXTURE=1 GR_PREPICK_KP=1200 GR_PREPICK_KD=80 GR_PREPICK_FMAX=40 \
  GR_ENG_PITCH=-0.77 GR_ENG_ELB=1.02 GR_ENG_RETRY=5 \
  GR_DROP_X=9.10 GR_DROP_Y=-1.63 GR_DROP_R=0.14 GR_DROP_DWELL=45 \
  GR_PLACE_DS_TOL=0.20 GR_PLACE_DS_MAX=80 \
  GR_LOWER_TICKS=150 GR_REL_X=9.15 GR_REL_Y=-1.63 GR_REL_RTOL=0.08 GR_REL_VMAX=0.12 \
  GR_REL_PITCH=-0.65 GR_REL_ELB=1.18 GR_PIN_F=30 GR_REL_ZMAX=0.66 \
  GR_CLEAR_R=0.85 GR_CLEAR_TICKS=2000 GR_OPEN_TICKS=100 \
  GR_UNSTICK_PITCH=-0.85 GR_UNSTICK_TICKS=60 GR_REACH_PITCH=-0.72 GR_REACH_ELB=1.08 \
  GR_HOLD_PITCH=-0.90 GR_HOLD_ELB=0.86 GR_RATE=0.005 OMNISIM_NEWTON_BASE_GUARD=0 \
  STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_ATT_HEADING=1 HARNESS_KZ=2000 HARNESS_DZ=150 \
  GR_NAN_WATCH=1 POLICY_ARCH=lstm PPO_HID=256 ${EXTRA:-}
