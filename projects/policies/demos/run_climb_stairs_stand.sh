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

# G1 STAIR-CLIMB + STAND demo (owner ask 2026-07-10/11: climb the stairs AS A HUMAN WOULD --
# chest forward, real foot steps -- and really STAND at the top): the G1 walks in, CLIMBS the
# full 5-step staircase (3 cm risers, legs-only: HARNESS_KZ=0) with its CHEST HELD FORWARD,
# then a BATON handover swaps in the stand specialist and the robot STANDS MOTIONLESS ON THE
# TOP LANDING. Ghost hologram world via SHOW_WORLD=projects/policies/worlds/g1_climb_stairs_demo3_show.omniworld.
#
# THE CHAMPION (2026-07-11, wr_stairhuman14): the first CHEST-FORWARD stair climber. The
# corridor-torque law was the lock the whole campaign -- a 3 cm step-up needs 0.192 rad of knee
# deviation from the flat-walk reference and every earlier chest-forward run capped the corridor
# at 0.15, so the straight climb was infeasible BY CONSTRUCTION (the old champion's yaw-twist was
# the only climb that fit). GHOST_RESIDUAL=0.22 unlocked it: batched, it climbs all 5 risers at
# max |yaw| 13.7 deg; live, the FOOT_LOG shows every tread taken in sequence with in-climb yaw
# ~3-15 deg. Trained with the crane yaw-trim on BOTH sides (HARNESS_KYAW mirror), mid-staircase
# spawn curriculum (COLLECT_ONSTAIR bank), SWING_H=0.13 riser clearance.
#
# VERIFIED (FOOT_LOG, 2026-07-11, 9 headless runs of THIS composition): 5 clean end-to-end
# passes -- full climb -> catch -> motionless stand on the landing (e.g. base x 3.11-3.22,
# z 0.88, final-8s base x-std 2.0-4.5 mm).
#
# HONEST LIMITS: pass rate ~1 in 2 (retry on a fall -- failures are a mid-climb fall or a
# missed stand-catch; each run self-verifies via FOOT_LOG). A brief launch wobble in the first
# metre (the walk-start transient, up to ~60 deg) recovers before/at the first treads; the climb
# itself is chest-forward. Deploy speed pinned to the trained range (VX_MAX=0.35 -- the 0.7
# default is out-of-distribution).
#
# !! MOTION-LEGITIMACY: FAIL (owner caught it 2026-07-11; verify_motion_legitimacy.py measures
# it). This champion passes the KINEMATIC gates only. Dynamically it leans on the crane's
# attitude springs (|ty| mean 77.5 N*m, >40 N*m on 77% of climb ticks -- the crane carries the
# lean) and its knees touch the stairs on 13.6% of climb ticks. Verify any successor with:
#   CRANE_LOG=1 CONTACT_LOG=1 CONTACT_LOG_ALL=1 on the deploy, then
#   python projects/policies/training/verify_motion_legitimacy.py <tag>_mpc.txt 1.0 2.6 6,7,12,13
# The next champion must pass BOTH rulers. Training fix queued: crane graduation to lam~0
# (HARNESS_GRAD_SURV=2.0 currently means the crane NEVER weans) + knee-contact penalty.
#
# HANDOVER: POSITION-GATED through the schedule's proven catch path (WALK_STAND_AT_X=3.0 skips
# the walk segment when base-x crosses the landing; the catch then decels to WALK_DECEL_FLOOR
# and waits for double support). A raw timer handed over anywhere between mid-staircase and past
# the far edge; a raw BATON_COURSE arrival handed over at full stride and fell 3/3.
#
# Run:  bash projects/policies/demos/run_climb_stairs_stand.sh [dur=480] [gui|headless]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
DUR="${1:-480}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
bash projects/policies/training/run_walk_rl.sh "$DUR" climb_stairs_stand deploy "$GUI" \
  PYTHONUNBUFFERED=1 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=${SHOW_WORLD:-projects/policies/worlds/g1_climb_stairs_demo3.omniworld} \
  RES_POLICY=$R/wr_stairhuman14.pt \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_METRIC_JSON=$G/ghost_official_full_v3_lut.json \
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.72 STAND_POSE=unitree WALK_WARM_TICKS=30 WALK_MJW_RESET=0 \
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 REF_OBS=1 REF_OBS_K=3 \
  GHOST_RESIDUAL=0.22 GHOST_RESIDUAL_LAT=0.30 ARM_RESIDUAL=0.10 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_RESIDUAL=0.10 \
  W_TRACK_LIN=8 W_HEIGHT=0 SWING_H=0.13 W_LINK=0 Z_TGT=0.72 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_ATT_GHOST=1 HARNESS_KZ=0 \
  HEADING_TARGET=0 HARNESS_KYAW=${HARNESS_KYAW:-80} VX_MAX=${VX_MAX:-0.35} \
  "BATON_SPECIALISTS=stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "WALK_SCHEDULE=walk:120,stand:600" WALK_STAND_AT_X=${WALK_STAND_AT_X:-3.0} BATON_COLD_HIDDEN=1 \
  FOOT_LOG=1 FOOT_LOG_EVERY=6
