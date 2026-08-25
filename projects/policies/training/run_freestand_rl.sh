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

# ⭐ THE FREE-STANDING WALK CAMPAIGN — drive the balance harness to λ=0 (2026-07-13).
#
#   bash projects/policies/training/run_freestand_rl.sh <dur_s> <tag> [KEY=VAL ...]
#
# THE TARGET: a durable G1 walk with NO external pelvis wrench (λ=0, HARNESS_KZ=0), passing
# verify_motion_legitimacy.py. Every shipped G1 demo today runs at λ=0.9 on a ≤700 N weight-bearing
# crane, and no walk champion has ever been shown at λ=0. This is the repo's open problem.
#
# WHY THE FLAGSHIP GHOST CANNOT GET THERE, AND THIS ONE CAN:
# The flagship walks its recorded ghost at vx=0.45 m/s. That reference is NOT a self-supporting
# motion -- it was recorded FROM a crane-driven policy, and the crane was doing the work (measured:
# it demands 21-131 N·m of external moment; "achieved is not achievable", commit 222b27ec0). The
# quasi-static self-support ceiling on the G1's 6 cm sole is vx ≈ 0.131 m/s (commit d7f16d378):
#     vx 0.252 -> 142 N·m | 0.191 -> 77 | 0.156 -> 34 | 0.138 -> 1.7 | 0.131 -> 0.0 N·m
# You cannot wean a crane off a policy imitating a motion that REQUIRES a crane. So we train the
# one walk ghost that is certified self-supporting: ghost_walk_synth (0.0 N, 0.0 N·m, all 3 gates,
# ghost_validator WARN-clean) -- solved by ghost_synth_walk.py with the COM riding over the bearing
# ankle. It is a deep-crouch "Groucho" walk at 0.131 m/s. That crouch is not a flaw: it is what
# self-support on this foot LOOKS like. No policy has ever been trained on it.
#
# THE THREE STRUCTURAL CHANGES THIS RUN CARRIES (all opt-in, all default-off elsewhere):
#   GHOST_FF=1     the corridor CENTRE shifts by ffdq = τ_ff/kp, so the reference's own stance
#                  torque costs the policy nothing. This ghost's ffdq peaks at 0.231 rad while the
#                  flagship corridor is 0.10 -- i.e. every walk campaign to date was ~2.3x too
#                  narrow for the policy to hold its own stance torque, and the crane paid the
#                  difference. Without this, λ→0 is arithmetically impossible (ghost_ff.py).
#   EST_HEAD=1     state-estimator head: the actor regresses base velocity + foot contact from its
#                  OWN observation history (supervised, detached into the policy head). The harness
#                  is a feedback controller on base linear velocity (HARNESS_FY damps it, KZ/DZ
#                  regulate height/vertical velocity) and the actor has never been able to SEE any
#                  of it -- it is critic-only privileged state. Take the crane away and the policy
#                  must replace a velocity damper with foot placement, blind. This is the fix.
#   POLICY_HEAD=mlp post-RNN [256,128] ELU head. The recurrent actor's head is otherwise a bare
#                  Linear: outside the LSTM gates its entire nonlinear capacity is one tanh.
#
# THE LADDER: λ steps down only when the honest eval says the policy EARNED it (HARNESS_GRAD_SURV)
# AND is not leaning on the springs. The anti-lean gate here is HARNESS_GRAD_SUS: the fraction of eval
# ticks on which the crane carried sustained attitude torque (>40 N·m), i.e. verify_motion_legitimacy's
# L1 criterion applied DURING training, so the ladder and the final exam agree by construction.
#
# ⚠️ A GATE IS NOT A GRADIENT -- W_CRANE is what makes the gate reachable. Run fs1 (gate on, W_CRANE=0)
# graduated 0.9→0.4 and then PARKED: survival stayed a flat 1.000 while the lean grew AWAY from the gate
# (sustained 17→27%, |tau| 25→32 N·m, gmatch 0.93→0.87, wobble 0.32→0.56) because PPO was buying speed
# with lean and lean was free. W_CRANE prices it: a dense penalty on the wrench the crane actually
# applies, in the gate's own units. It vanishes to zero exactly when the robot no longer needs the crane.
# (Do NOT use HARNESS_GRAD_ATT here: that posture proxy is only computed when the arm-ghost eval runs,
# and for every other ghost it was never assigned and the gate read it as a default 1.0 -- i.e. it
# passed unconditionally. That silent hole is what run p2e fell through: it leaned on the springs, rode
# the ladder to λ=0.3, and froze at surv 0.478 for 400+ iterations. Fixed 2026-07-13; now fails closed.)
# The last rung is also a cliff: run p2g held surv >0.92 down to λ=0.10 and then collapsed to 0.23 on
# the 0.10→0 step. HARNESS_FINE_BELOW splits that final rung into 0.02-sized ones.
#
# ATTRIBUTION: to run the control (same ghost, same ff, same ladder, OLD net):
#   EST_HEAD=0 POLICY_HEAD=linear bash .../run_freestand_rl.sh 7200 fs_control
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"

DUR="${1:-7200}"; TAG="${2:-freestand}"
shift 2 2>/dev/null || true

GHOST="$ROOT/projects/policies/ghosts/g1/ghost_walk_synth_lut.json"
# Seed the robot IN the reference, not 0.8 rad away from it: the ghost's bin-0 pose is a deep crouch
# (knee +1.10 rad) and the Unitree default (+0.30) would drop the robot outside its own corridor on
# tick 0. Base z 0.68 is the ghost's SOLVED height -- Z_TGT and HARNESS_Z0 must both follow it, or
# the height reward and the bungee fight the reference (Z_TGT defaults to 0.74).
SEED="-0.655,-0.159,0.172,1.100,-0.459,0.234,-0.714,-0.148,0.180,0.356,0.345,0.233"

bash "projects/policies/training/run_walk_rl.sh" "$DUR" "$TAG" train headless \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step \
  WALK_WORLD=projects/policies/worlds/g1_walk_orig.omniworld \
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.70 "STAND_POSE_LEGS=$SEED" \
  WALK_WARM_TICKS=30 WALK_MJW_RESET=0 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 \
  "POLICY_HEAD=${POLICY_HEAD:-mlp}" "EST_HEAD=${EST_HEAD:-1}" "EST_COEF=${EST_COEF:-1.0}" \
  REF_OBS=1 REF_OBS_K=3 REF_OBS_STRIDE=4 \
  "GHOST_LUT_JSON=$GHOST" "G1_GHOST_LUT=$GHOST" \
  GHOST_FF=1 "GHOST_RESIDUAL=${GHOST_RESIDUAL:-0.12}" \
  VX_START=0.131 VX_MAX=0.131 VX_CURR_ITERS=1 \
  Z_TGT=0.68 \
  HARNESS_LAM0="${HARNESS_LAM0:-0.9}" HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 \
  HARNESS_KZ=2000 HARNESS_DZ=150 HARNESS_Z0=0.68 \
  HARNESS_GRAD_SURV="${HARNESS_GRAD_SURV:-0.90}" HARNESS_GRAD_SUS="${HARNESS_GRAD_SUS:-0.15}" \
  "W_CRANE=${W_CRANE:-0.5}" \
  HARNESS_STEP=0.10 HARNESS_FINE_BELOW=0.20 HARNESS_STEP_FINE=0.02 \
  "PPO_ITERS=${PPO_ITERS:-4000}" "EVAL_EVERY=${EVAL_EVERY:-100}" "EVAL_H=${EVAL_H:-1500}" \
  "CKPT_EVERY=${CKPT_EVERY:-200}" \
  "RES_POLICY=$ROOT/projects/policies/training/runs/wr_${TAG}.pt" \
  "$@"
