#!/usr/bin/env bash
# G1 STAIR-CLIMB demo (owner 2026-07-08): the G1 walks in from the flat and CLIMBS a 5-step
# staircase with REAL foot steps -- each foot lifts and lands ON the next tread, the body rises
# tread-by-tread, no vertical wire (HARNESS_KZ=0 => the LEGS do the climbing; the crane only
# balances laterally/attitude, exactly as required).
#
# ⭐ HOW THIS WORKS (the arc that cracked it, after 7 climb-GHOST approaches all failed with the
# feet just shuffling at the base): FORGET a bespoke climb ghost. Warm-start the EXISTING WALKER
# (wr_navigator, trained on ghost_official_full_v3) and drive it with the WALKING ghost on the
# STAIR TERRAIN -- the robot walks FORWARD and the terrain forces it UP (the proven "blind walk
# over rough terrain" recipe). Trained under a TERRAIN CURRICULUM (shallow risers first) with a
# NaN-guard on the reward (a blown-up stair-contact env used to poison the batched reward and kill
# PPO once the robot climbed high). Policy: wr_stairwalk3d (full 5-step climb, live-verified via
# FOOT_LOG: feet ascend tread 1->5, base z 0.72->0.88).
#
# ⛔ RISER = 3cm: the stock-foot G1, climbing LEGS-ONLY (KZ=0), caps at ~3cm risers in live deploy
# (4cm~=2 steps, 5cm~=0) -- the small foot's propulsion wall. The original 10/7cm staircase needs a
# bigfoot morphology or a vertical assist; this demo ships what the STOCK G1 genuinely can do.
#
# Judge the climb by watching the GUI (or a >=45s FOOT_LOG deploy) -- a short/eval-horizon run
# truncates the climb at ~3 of 5 steps.
#
# Run:  bash projects/policies/demos/run_climb_stairs.sh [dur=60] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
DUR="${1:-60}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/controllers/g1_ghost
bash projects/policies/training/run_walk_rl.sh "$DUR" climb_stairs deploy "$GUI" \
  PYTHONUNBUFFERED=1 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=projects/policies/worlds/g1_climb_stairs_demo3.wbt \
  RES_POLICY=$R/wr_stairwalk3d.pt \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_METRIC_JSON=$G/ghost_official_full_v3_lut.json \
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.72 STAND_POSE=unitree WALK_WARM_TICKS=30 WALK_MJW_RESET=0 \
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 REF_OBS=1 REF_OBS_K=3 \
  GHOST_RESIDUAL=0.15 GHOST_RESIDUAL_LAT=0.30 ARM_RESIDUAL=0.10 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_RESIDUAL=0.10 \
  W_TRACK_LIN=8 W_HEIGHT=0 SWING_H=0.11 W_LINK=0 Z_TGT=0.72 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_ATT_GHOST=1 HARNESS_KZ=0 \
  FOOT_LOG=1 FOOT_LOG_EVERY=6
