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

# BATON HORIZON EXPERIMENT -- the success-vs-cycle-index run.
#
# This is the experiment docs/developer/policy-switching.md has called for since it
# was written: a many-cycle task on which BATON's ENGINEERED handover can be scored
# against a NAIVE one, so "switching beats a monolith" stops being a hypothesis.
#
# The task is a closed loop the robot repeats N times:
#     walk out -> turn 90 -> walk back -> turn 90  (a square-ish circuit)
# Every cycle returns the robot to (roughly) where it started, so cycle k is
# comparable to cycle 0 and the ONLY thing that grows is accumulated handover error.
# That is the point: a handover that leaks a little state each switch degrades with
# horizon, and the plot shows it.
#
# ARMS (select with ARM=):
#   engineered  -- BATON as shipped: cold hidden at the stand->locomotion edge,
#                  morph blend over BATON_MORPH_TICKS, phase-gated entry.
#   naive       -- the ablation: warm hidden carried across every switch, no morph
#                  (BATON_MORPH_TICKS=0). Same specialists, same course, same physics.
#                  This is the "naive FSM hierarchy" arm of the 3-way comparison.
#
# Usage:  bash projects/policies/demos/run_baton_horizon.sh [cycles] [arm] [dur_s] [gui] [seed]
#   e.g.  bash projects/policies/demos/run_baton_horizon.sh 6 engineered 600 headless 3
#
# SEEDS: the deploy is otherwise deterministic, so a single run per arm would give a
# survival horizon, not a success RATE -- and a rate is what a horizon plot needs.
# DEPLOY_IC_SEED perturbs the initial leg pose (DEPLOY_IC_NOISE, 0.02 rad), which is
# the same symmetry-breaking the trainer uses. Sweep the seed to get N independent
# runs per arm. Do NOT report a "success rate" from one seed.
#
# Scoring: each cycle emits one machine-readable line (recipe: _wr_course_advance)
#     BATON-CYCLE k=<i> ok=<0|1> segs=<done>/<total> t=<tick> dur=<s> minz=<m>
# Harvest with:  python projects/policies/training/baton_metrics.py --horizon <log>
#
# ⚠️ Runs on the WEIGHT-BEARING balance harness (HARNESS_LAM0=0.9, HARNESS_KZ=2000),
# like every G1 locomotion demo in this repo. The rotation is the robot's own footwork
# (crane yaw torque is zero during the turn), but the robot is being CARRIED. Never
# describe this as a free-standing walk.
set -euo pipefail
cd "$(dirname "$0")/../../.."

CYCLES="${1:-6}"
ARM="${2:-engineered}"
DUR="${3:-600}"
GUI="${4:-headless}"
SEED="${5:-7}"

case "$ARM" in
  engineered) COLD=1; MORPH=30 ;;
  naive)      COLD=0; MORPH=0  ;;
  *) echo "unknown ARM '$ARM' (want: engineered | naive)" >&2; exit 2 ;;
esac

# One cycle = a CLOSED SQUARE: 4 sides, 4 corners, back to the start pose.
#
#   (0,0) -> (2.5,0) -> (2.5,2.5) -> (0,2.5) -> (0,0), turning 90 deg at each corner.
#
# The circuit MUST close, and that is not cosmetic. The first version of this course
# had only two corners, so after one lap the robot stood at the far side facing the
# wrong way and cycle 1 opened by demanding a walk ~90 deg off its heading -- it fell
# at t=4301 and the run scored ok=0 for a reason that had nothing to do with the
# handover under test. A horizon experiment is only meaningful if cycle k begins in
# the same state as cycle 0; otherwise you are measuring your own geometry.
#
# Deliberately locomotion-only: no manipulation, no suction, no props. The ONLY thing
# that accumulates across cycles is handover error, which is the variable under test.
COURSE="walkto,2.5,0.0;stand,2;turn,45,90;stand,2;walkto,2.5,2.5;stand,2;turn,45,90;stand,2;walkto,0.0,2.5;stand,2;turn,45,90;stand,2;walkto,0.0,0.0;stand,2;turn,45,90;stand,2"

echo ">>> BATON HORIZON  arm=$ARM  cycles=$CYCLES  seed=$SEED  cold_hidden=$COLD  morph=$MORPH  dur=${DUR}s"

bash projects/policies/training/run_walk_rl.sh "$DUR" "baton_horizon_${ARM}_s${SEED}" deploy "$GUI" \
  DEPLOY_IC_SEED="$SEED" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  OMNISIM_FOOT_TORSION=0.25 \
  WALK_WORLD=projects/policies/worlds/g1_walk_puppet.omniworld \
  GHOST_LUT_JSON=projects/policies/ghosts/g1/ghost_official_full_v3_lut.json \
  GHOST_RESIDUAL=0.100 \
  RES_POLICY=projects/policies/training/runs/wr_navigator.pt \
  WHOLE_BODY=1 REF_OBS=1 REF_OBS_K=3 \
  BATON_SPECIALISTS="stand|projects/policies/training/runs/wr_stander.pt|projects/policies/ghosts/g1/ghost_stand_v1_lut.json|0" \
  BATON_COURSE="$COURSE" \
  BATON_COURSE_LOOPS="$CYCLES" \
  BATON_COLD_HIDDEN="$COLD" \
  BATON_MORPH_TICKS="$MORPH" \
  BATON_STAND_PIVOT=0 \
  BATON_TURN_CKPT=projects/policies/training/runs/wr_turn90.pt \
  BATON_TURN_LUT=projects/policies/ghosts/g1/ghost_turn_90_lut.json \
  BATON_TURN_REF_WB=1 BATON_TURN_SETTLE_TICKS=180 BATON_TURN_EXIT_TICKS=150 \
  TURN_TO_DEG=90 TURN_LOOP=1 TURN_LOOP_MAX=6 TURN_LOOP_ABORT_DEG=35 \
  TURN_HOLD_LOCK=0 TURN_OBS_RELATIVE=1 \
  GHOST_FOLLOW=1 STAND_SEED=1 WALK_WARM_TICKS=30 \
  ARM_RESIDUAL=0.100 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  HARNESS_KYAW=150 HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 \
  HARNESS_FY=400 HARNESS_FY_HEADING=1 HARNESS_ATT_HEADING=1 \
  HARNESS_KZ=2000 HARNESS_DZ=150 \
  POLICY_ARCH=lstm PPO_HID=256
