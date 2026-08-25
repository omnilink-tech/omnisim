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

# ⭐ FLAGSHIP HUMANOID DEMO — "the decent walker" (owner-designated 2026-07-06).
#
# The Unitree G1 walking the OFFICIAL Unitree gait on the PUPPET rig: a fully
# physical robot carried by a visible overhead harness (lam=0.9) beside its
# GHOST hologram, driven by the LSTM+foresight champion wr_decent_walker.pt
# (WBMATCH4 0.868 on the honest shape-only ruler; exam-verified, K=2048).
# Arms swing naturally through vertical, clear of the thighs (ghost v3).
#
#   bash projects/policies/worlds/run_g1_decent_walker.sh
#
# The demo needs the full deploy env (engine hook + corridors + harness), which a
# bare world launch cannot carry -- always start it via this script (or the
# PowerShell sibling run_g1_decent_walker.ps1 on Windows). Headless-verified
# 2026-07-17 on machine 9722d23d12a3: 0.120 m/s over 15.04 s, zero backward
# samples, max |yaw| 0.148 rad. This is still the weight-bearing lam=0.9 balance
# harness, not free-standing walking.
#
# NOTE: bash sibling of run_g1_decent_walker.ps1 -- keep the two in sync:
# same env vars, same run_walk_rl.sh argument list, same checkpoint.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere

export OMNISIM_HOME="$ROOT"
export OMNISIM_NEWTON_COMPOUND_COLLIDERS="1"   # 4-sphere feet = the ghost recording's plant

bash "projects/policies/training/run_walk_rl.sh" 900 flagshipdemo deploy gui \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=projects/policies/worlds/g1_walk_puppet.omniworld \
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.75 STAND_POSE=unitree \
  WALK_WARM_TICKS=30 WALK_MJW_RESET=0 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 \
  REF_OBS=1 REF_OBS_K=3 REF_OBS_STRIDE=4 \
  "GHOST_LUT_JSON=$ROOT/projects/policies/ghosts/g1/ghost_official_full_v3_lut.json" \
  "G1_GHOST_LUT=$ROOT/projects/policies/ghosts/g1/ghost_official_full_v3_lut.json" \
  GHOST_RESIDUAL=0.10 PHASE_LEAD=0.45 GHOST_OMEGA_SCALE=2.0 \
  FOOT_HEADING_LOCK=1 FOOT_HEADING_TRIM_SIGN=-1 \
  FOOT_HEADING_KP=0.3 FOOT_HEADING_KD=0.02 FOOT_HEADING_TRIM_CAP=0.06 \
  WALK_DIAG=1 ACT_AUTH=1 FOOT_LOG=1 FOOT_LOG_EVERY=8 \
  ARM_RESIDUAL=0.10 ELB_HANG_REF=1.6 ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 \
  SHRY_TARGET=0.15 SHRY_YAW_TARGET=0 SHRY_RESIDUAL=0.10 \
  HARNESS_LAM0=0.9 HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 \
  HARNESS_KZ=2000 HARNESS_DZ=150 HARNESS_Z0=0.70 HARNESS_ATT_GHOST=1 \
  VX_MAX=0.45 \
  "RES_POLICY=$ROOT/projects/policies/training/runs/wr_decent_walker.pt"
