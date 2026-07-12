#!/usr/bin/env bash
# BOX DELIVERY v2 (owner 2026-07-10; renamed 2026-07-10): realistic pick-and-place with a REAL corner.
# The G1 walks to cart A and stops WITH THE BOX AT ITS HANDS, takes it (two-phase lift: straight
# up off the cart, then into the hands -- proximity-gated, it can never levitate across the room),
# carries it down the south corridor to cart B and SETS IT DOWN (two-phase: over the rest point,
# then straight down; real contact settle), walks on past the cart, takes a REAL ~90-deg footwork
# corner (TURN-LOOP, wtz=0), and walks away north, ending in a stand.
#
# ⛔⛔ THE HEADING BAND (why this is NOT the there-and-back shuttle, measured across 10 runs):
# the turn specialist's ghost spans EXACTLY 0->90 deg of absolute heading and the policy never
# saw states beyond it -- every corner whose sweep crossed ~95 deg fell/spun (at 103/106/113/
# ~150 deg; an entry at 93 spun instantly), while this demo's 0->90 corner is 10/10 across
# campaigns. Three deploy-side frame fixes landed on the way (all measured, all armed here):
# heading-frame FY brake source during turns, HARNESS_ATT_HEADING=1 (body-frame attitude
# spring -- the raw spring is cross-wired past ~45 deg), TURN_OBS_RELATIVE=1 (entry-relative
# angular-rate obs; banked 58 deg past the band vs 20-45 without). The full shuttle needs a
# HEADING-RANDOMIZED retrain of the turner (SEQ yaw0 randomization) -- the named next step.
# Also load-bearing: never chain rotations without a walk between (a single turn,60,180 loop
# fell at pass 3-4; two 90s with only a stand between spun to 311 deg -- wound feet).
# The hologram ghost executes the same routine alongside (walk/carry/turn/stand references).
#
# Stack: the classic box_delivery base (BATON walk<->stand<->carry + ODE box rig) + the walk_turn_walk turn
# stack (TURN-LOOP, TURN_HOLD_LOCK=0, stand-sandwich, keep-heading, heading-frame brake,
# OMNISIM_FOOT_TORSION=0.25) + course "turn,SEC,DEG" per-segment angles + the two-phase rig.
# BATON_STAND_PIVOT=0 (corners are footwork, not the crane). Turns are EMPTY-HANDED by design.
# Run: bash projects/policies/demos/run_box_delivery.sh [dur] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
DUR="${1:-170}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/controllers/g1_ghost
MODEF="${BATON_MODE_FILE:-$ROOT/_scratch/foot_redesign/boxmode.txt}"
mkdir -p "$(dirname "$MODEF")"
echo "parked" > "$MODEF"
bash projects/policies/training/run_walk_rl.sh "$DUR" box_delivery deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0.25 WALK_WORLD=projects/policies/worlds/g1_walk_puppet.wbt \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 \
  RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 \
  "BATON_SPECIALISTS=carry|$R/wr_carrier.pt|$G/ghost_carry_v1_lut.json|0.45;stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "BATON_COURSE=walkto,4.1,-1.35;stand,3;carryto,5.3,-2.1;carryto,8.85,-1.85;stand,6;walkto,11.0,-1.9;stand,3;turn,40;stand,3;walkto,10.7,1.9;stand,0" \
  BATON_COLD_HIDDEN=1 BATON_MORPH_TICKS=30 BATON_STAND_PIVOT=0 \
  "BATON_MODE_FILE=$MODEF" \
  BATON_TURN_CKPT=$R/wr_turn90.pt BATON_TURN_LUT=$G/ghost_turn_90_lut.json BATON_TURN_REF_WB=1 \
  BATON_TURN_SETTLE_TICKS=180 BATON_TURN_EXIT_TICKS=150 TURN_TO_DEG=90 TURN_LOOP=1 TURN_LOOP_MAX=6 TURN_LOOP_ABORT_DEG=35 TURN_HOLD_LOCK=0 TURN_OBS_RELATIVE=1 \
  GHOST_FOLLOW=1 HAND_TRACK=1 \
  STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_ATT_HEADING=1 HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256
