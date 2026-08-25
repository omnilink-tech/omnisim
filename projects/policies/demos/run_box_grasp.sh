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

# ✅ SUCTION DELIVERY (2026-07-11) -- the G1 picks the box up with a SUCTION GRIPPER and the
# delivery is 100% physics: NO kinematic writes to the box, ever (PHYS_GRASP=1 puts the rig in
# log-only mode). The suction is a contact-free force coupling (GR_SUCTION=1): a clamped
# spring-damper wrench ties the box's top anchor to a cup point hanging GR_SUCTION_DROP below
# the palm centroid, delivered through control.joint_f (+wrench on the box free joint, reaction
# on the pelvis) -- exactly the suction-cup style, no contact impulses, which sidesteps the
# open deploy-stack x contact NaN entirely. Verified full course: walk 3.8 m -> hover-engage
# (SUCTION ON at ~0.17 m gap) -> lift (+8 cm) -> 5 m two-corner suction carry -> PRESS-PLACE
# (lower + stillness/on-target-gated release from the carry-press; the course advances only
# after release) -> box settles ON the delivery stand -> east exit, 40-deg turn, north leg,
# final stand. Zero NaN, zero falls. Key learnings baked in as defaults below:
#   - palms must NEVER touch the box (palm-sphere contact -> the known stack x contact NaN);
#     the hover geometry + GR_SUCTION_DROP=0.10 keep the hanging box 4.5 cm below the palms
#   - the carry-PRESS against the delivery stand is the stable hover; a stand-settle drags the
#     hanging box ~20 cm west (that is why the place fires FROM carry, GR_DROP_DWELL)
#   - release only when the box is over GR_REL_X/Y (RTOL) AND slow (GR_REL_VMAX)
# FRICTION-GRIP EXPERIMENT (parked): squeezing with the palms NaNs on first palm contact --
# the triangulated deploy-hook x contact defect (bisect plan in the git history of this
# header). The kinematic-rig flagship remains run_box_delivery.sh (tall carts, 3/3).
# Run: bash projects/policies/demos/run_box_grasp.sh [dur] [gui|headless] [COURSE=...]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
# default duration = course completion (~490 s) + margin; the placed box is verified stable
# for 175+ s of the live demo, but an idle sim left running for many extra minutes shows a
# slow solver micro-creep (~0.5 mm/s) that can eventually walk the box off the stand -- end
# the run when the demo ends.
DUR="${1:-560}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
MODEF="${BATON_MODE_FILE:-$ROOT/_scratch/foot_redesign/boxmode.txt}"
mkdir -p "$(dirname "$MODEF")"
echo "parked" > "$MODEF"
# the post-pick TURN (owner 2026-07-11): after the grab the robot turns ~40deg AWAY from the
# pick table (it cannot walk backward), then detours NE through open floor before rejoining
# the verified western approach to the delivery stand -- no more squeezing forward past the
# table with the box.
# pre-press waypoint (8.55,-1.75): the final press leg must be SHORT and straight -- a long
# last leg accumulates enough veer to miss the stand's stem entirely (measured y spread 0.5 m)
# dwell budget TRIMMED per the trajectory audit (both turns finish in ~10-14 s; the old
# 25 s / 40 s turn dwells were pure shuffle; place-release completes before the stand)
COURSE="${COURSE:-walkto,4.3,-0.98;stand,6;turn,14,40;stand,2;carryto,5.0,0.2;carryto,7.8,-1.42;carryto,8.55,-1.61;carryto,9.3,-1.41;stand,6;walkto,11.0,-2.6;stand,2;turn,16,40;stand,2;walkto,10.7,1.9;stand,0}"
# TAG override lets parallel verification instances keep separate run logs
TAG="${TAG:-box_grasp}"
bash projects/policies/training/run_walk_rl.sh "$DUR" "$TAG" deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0 WALK_WORLD=projects/policies/worlds/g1_box_grasp.omniworld \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 \
  RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 \
  "BATON_SPECIALISTS=carry|$R/wr_carrier.pt|$G/ghost_carry_v1_lut.json|0.45;stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "BATON_COURSE=$COURSE" \
  BATON_COLD_HIDDEN=1 BATON_MORPH_TICKS=30 BATON_STAND_PIVOT=0 \
  "BATON_MODE_FILE=$MODEF" \
  PHYS_GRASP=1 GR_PICK_SEG=1 GR_PLACE_SEG=8 HAND_TRACK=1 \
  GR_SUCTION=1 GR_SUCTION_R=0.05 GR_SUCTION_DROP=0.01 GR_CLOSE_TICKS=100 \
  GR_ENG_PITCH=-0.77 GR_ENG_ELB=1.02 \
  GR_DROP_X=9.10 GR_DROP_Y=-1.63 GR_DROP_R=0.14 GR_DROP_DWELL=45 \
  GR_LOWER_TICKS=150 \
  GR_REL_X=9.15 GR_REL_Y=-1.63 GR_REL_RTOL=0.08 GR_REL_VMAX=0.12 GR_REL_PITCH=-0.65 GR_REL_ELB=1.18 \
  GR_PIN_F=30 GR_REL_ZMAX=0.66 \
  GR_CLEAR_R=0.85 GR_CLEAR_TICKS=2000 \
  GR_OPEN_TICKS=100 GR_UNSTICK_PITCH=-0.85 GR_UNSTICK_TICKS=60 \
  GR_REACH_PITCH=-0.72 GR_REACH_ELB=1.08 GR_HOLD_PITCH=-0.90 GR_HOLD_ELB=0.86 GR_RATE=0.005 \
  OMNISIM_NEWTON_BASE_GUARD=0 \
  BATON_TURN_CKPT=$R/wr_turn90.pt BATON_TURN_LUT=$G/ghost_turn_90_lut.json BATON_TURN_REF_WB=1 \
  BATON_TURN_SETTLE_TICKS=180 BATON_TURN_EXIT_TICKS=150 TURN_TO_DEG=90 TURN_LOOP=1 TURN_LOOP_MAX=6 TURN_LOOP_ABORT_DEG=35 TURN_HOLD_LOCK=0 TURN_OBS_RELATIVE=1 \
  GHOST_FOLLOW=1 \
  STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_ATT_HEADING=1 HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256 ${EXTRA:-}
