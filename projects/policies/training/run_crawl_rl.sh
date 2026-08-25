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

# G1 COMMANDO-CRAWL training (Shadowing): shadow ghost_crawl_v1 in-engine.
#
# Thin wrapper over the canonical run_walk_rl.sh that bundles the CRAWL recipe:
#   * the recipe trainer (NOT the legacy ES),
#   * the crawl-collider world (g1_crawl_train.omniworld: hands/knees/forearms/torso/
#     pelvis boxes),
#   * MTRACK motion-tracking (keypoint + whole-body imitation, balance emergent),
#   * CRAWL=1 -> the prone hooks in g1_walk_recipe.py (spawn/fall/attitude/harness
#     pulled from g1_crawl_mode.py),
#   * a low prone harness (Z0/gate from g1_crawl_mode; assists the pelvis at the
#     crawl height like the lambda=0.9 puppet does for the walk).
#
# Usage: bash run_crawl_rl.sh <dur> <tag> <mode:train|deploy> [gui] [ENV=val ...]
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && { pwd -W 2>/dev/null || pwd; })"
DUR="${1:-3600}"; TAG="${2:-wr_crawl1}"; MODE="${3:-train}"; GUI="${4:-headless}"; shift 4 2>/dev/null || shift $# 2>/dev/null || true
GHOSTS="$ROOT/projects/policies/ghosts/g1"

bash "$ROOT/projects/policies/training/run_walk_rl.sh" "$DUR" "$TAG" "$MODE" "$GUI" \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step \
  WALK_WORLD=projects/policies/worlds/g1_crawl_train.omniworld \
  CRAWL=1 MTRACK=1 WHOLE_BODY=1 \
  GHOST_LUT_JSON="$GHOSTS/ghost_crawl_v1_lut.json" \
  GHOST_METRIC_JSON="$GHOSTS/ghost_crawl_v1_lut.json" \
  W_KP=6.0 W_WB=3.0 KP_SIG=0.12 WB_SIG=0.5 \
  W_UP=1.0 W_HEIGHT=8.0 Z_TGT=0.30 \
  GHOST_RESIDUAL=0.0 \
  STAND_SEED=1 STAND_Z=0.30 STAND_POSE=unitree \
  HARNESS_LAM0=0.9 HARNESS_KZ=1500 HARNESS_DZ=120 HARNESS_KP=250 HARNESS_KD=40 HARNESS_FY=200 \
  WALK_MJW_RESET=0 WALK_WARM_TICKS=30 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  PPO_NENV="${PPO_NENV:-2048}" PPO_HID="${PPO_HID:-256}" \
  PPO_ITERS="${PPO_ITERS:-600}" EVAL_EVERY="${EVAL_EVERY:-50}" EVAL_H="${EVAL_H:-800}" CKPT_EVERY="${CKPT_EVERY:-50}" \
  RES_POLICY="$ROOT/projects/policies/training/runs/${TAG}.pt" \
  "$@"
