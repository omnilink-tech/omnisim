#!/usr/bin/env bash
# TURN SOLO (owner 2026-07-08): show the FOOTWORK TURN policy (wr_stepturn) BY ITSELF -- G1 settles to
# a stand, then turns ~90 deg by REAL footwork (BATON solo-turn context; crane-yaw auto-OFF => wtz=0,
# pure footwork, no rope), decelerates to a stop, holds. No walk legs. The turn is at its most reliable
# from a clean STAND (the in-sequence variance comes from the walk->turn handoff, not the turn itself).
# Uses the box-demo walker (wr_navigator) as the warmup/settle context + wr_stepturn swapped in for the
# turn. OMNISIM_FOOT_TORSION=0.25 gives the feet spin friction (a flat foot otherwise freewheels and the
# turn loses ~2/3 its yaw). TURN_TO_DEG=90 decel-stops at 90.
# Run: bash projects/policies/demos/run_turn_solo.sh [dur] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
DUR="${1:-90}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/controllers/g1_ghost
bash projects/policies/training/run_walk_rl.sh "$DUR" turn_solo deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0.25 WALK_WORLD=projects/policies/worlds/g1_turn_demo.wbt \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 HEADING_TARGET=0.0 \
  BATON_TURN_CKPT=$R/wr_stepturn.pt BATON_TURN_LUT=$G/ghost_turn_180_lut.json BATON_TURN_REF_WB=1 \
  BATON_TURN_SETTLE_TICKS=180 TURN_TO_DEG=90 TURN_DECEL_DEG=20 \
  "WALK_SCHEDULE=walk:4,turn:120" \
  STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256
