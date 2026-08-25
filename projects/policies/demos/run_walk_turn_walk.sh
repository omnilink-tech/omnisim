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

# WALK -> TURN -> WALK (owner 2026-07-08, FINALIZED 2026-07-10): the G1 walks ~5 m along +x,
# turns a REAL ~90 deg by FOOTWORK (BATON solo-turn, crane yaw auto-OFF during the turn => wtz=0),
# then walks a CLEAN straight leg along the NEW heading (+y). 0 falls, repeatable.
#
# The stack (each piece verified in telemetry, _scratch/foot_redesign/walk_turn_walk_mpc.txt):
#   - STAND-SANDWICH schedule (walk,stand,turn,stand,walk): a stand on both sides of the turn --
#     clean standstill entry, and the stand specialist catches the settled post-turn pose.
#   - TURN-LOOP (2026-07-10, the piece that finally squared the corner): the 90-deg step-turn
#     ghost played ONCE only banks ~60-65% of its yaw live (foot-slip: the legs track the ghost,
#     the base under-rotates -- gain 0.67 measured, stable across runs). The ghost is a MODULAR
#     staircase of 15-deg feet-together mini-pivots, so the deploy REPLAYS partial passes --
#     restarting at the plateau whose remaining staircase ~= remaining angle / measured gain --
#     until the ACTUAL accumulated heading reaches TURN_TO_DEG. Never stops mid-lut (the robot
#     lags the ghost; both mid-lut arrests measured recoiled -19 deg or spun+fell): every pass
#     plays through the ghost's own final decel into its end-hold, the trained leak-free settle.
#     Measured 3/3: lands the turn at 90.6-95.6 deg actual in 2-3 passes (gain 0.67-0.73).
#   - TURN_HOLD_LOCK=0: the completed-turn pelvis-xy clamp (built for the CONTINUOUS turner's
#     post-decel slide) is OOD for this feet-together turner -- measured, the settled end-hold
#     stays perfect ~1 s then the clamp winds it into a spin+fall. This turner pivots in place
#     (x drift < 0.05 m); the clamp is pure downside here. The STAND-segment anti-slide hold
#     (the pre-turn +4 m slide fix) is unaffected.
#   - keep-heading: at turn exit the walk's heading target pins to the ACHIEVED settled yaw
#     (measured 3/3: 84-90 deg), so leg 2 walks straight along the new bearing.
#   - HARNESS_FY_HEADING=1: the deploy crane's lateral brake rotated into the heading frame
#     (world-frame it fought the +y walk -- the old 40-deg crab). No-op at heading 0.
# Verified (2026-07-10, 3/3 headless): leg 1 5.2 m +x clean; turn to 90.6-95.6 deg in place,
# upright; leg 2 straight at 82-90 deg bearing (best run: x drift 0.3 m over 6.4 m of +y) to the
# arena edge; min z 0.662+, 0 falls. Landing quantum = the ghost's 15-deg mini-pivot.
# Run: bash projects/policies/demos/run_walk_turn_walk.sh [dur] [gui|headless]
# (default 120 s ends leg 2 mid-arena; longer runs walk to the arena wall at y~7 and mark time
#  there, still 0 falls. Headless fast mode covers ~2x sim-seconds per wall-second.)
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
DUR="${1:-120}"; GUI="${2:-gui}"
R=projects/policies/training/runs
G=projects/policies/ghosts/g1
bash projects/policies/training/run_walk_rl.sh "$DUR" walk_turn_walk deploy "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0.25 WALK_WORLD=projects/policies/worlds/g1_walk_puppet.omniworld \
  GHOST_LUT_JSON=$G/ghost_official_full_v3_lut.json GHOST_RESIDUAL=0.100 RES_POLICY=$R/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 HEADING_TARGET=0.0 \
  BATON_TURN_CKPT=$R/wr_turn90.pt BATON_TURN_LUT=$G/ghost_turn_90_lut.json BATON_TURN_REF_WB=1 \
  BATON_TURN_SETTLE_TICKS=180 BATON_TURN_EXIT_TICKS=150 TURN_TO_DEG=90 TURN_LOOP=1 TURN_HOLD_LOCK=0 \
  GHOST_FOLLOW=1 \
  "BATON_SPECIALISTS=stand|$R/wr_stander.pt|$G/ghost_stand_v1_lut.json|0" \
  "WALK_SCHEDULE=walk:32,stand:8,turn:45,stand:10,walk:40" \
  STAND_SEED=1 WALK_WARM_TICKS=30 WALK_RESET_STEPS=180 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256
