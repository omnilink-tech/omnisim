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

# Launch the DETERMINISTIC quasi-static walk (humanoid_static_walk) on the BIGFOOT G1.
# This is the statically-stable gait (CoM stays over the support polygon) -- the one that
# fell at 1.42s on the small foot; the bigfoot should let it actually walk.
#   headless: bash run_static_walk.sh <dur> <tag>            [WALK_*=.. ENV=..]
#   live GUI: bash run_static_walk.sh <dur> <tag> gui        [WALK_*=.. ENV=..]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."
DUR="${1:-30}"; TAG="${2:-swalk}"; MODE="${3:-headless}"; shift 3 2>/dev/null || shift $# 2>/dev/null || true
ROOT="$(pwd -W 2>/dev/null || pwd)"
export OMNISIM_HOME=$ROOT PYTHONUTF8=1
export WEBOTS_EXTRA_PROJECT_PATH="$ROOT/projects/policies/research"
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1 OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
export OMNISIM_NEWTON_TARGET_KE=400 OMNISIM_NEWTON_TARGET_KD=60 OMNISIM_NEWTON_GROUND_MU=2.0
export HUMANOID_STAND_SPEC="$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json"
export HSTAND_ANK_BIAS=-0.06
export HUMANOID_STAND_LOG="$ROOT/_scratch/foot_redesign/${TAG}.log"
export OMNISIM_DEPLOY_LOG="$HUMANOID_STAND_LOG"
export OMNISIM_LOG_PATH="$ROOT/_scratch/foot_redesign/${TAG}_omnisim.txt"
export HSTAND_WARMUP_RELOAD=1
export OMNISIM_WARMUP_TOKEN="swalk_$$_${RANDOM}"
for kv in "$@"; do export "$kv"; done
WORLD="${SWALK_WORLD:-projects/policies/research/worlds/g1_static_walk_bigfoot.omniworld}"
rm -f "$HUMANOID_STAND_LOG" "$OMNISIM_LOG_PATH"
echo ">>> $TAG mode=$MODE dur=${DUR}s  $(for kv in "$@"; do printf '%s ' "$kv"; done)"
if [ "$MODE" = "gui" ]; then
  python -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR" &
  echo "GUI launched (pid $!). Telemetry -> _scratch/foot_redesign/${TAG}.log"
else
  python -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" \
      > "$ROOT/_scratch/foot_redesign/${TAG}_console.txt" 2>&1
  echo "=== ${TAG} walk telemetry ==="
  grep -E '\[swalk:g1\] (QUASI|settle|t=)' "$HUMANOID_STAND_LOG" 2>/dev/null | tail -16
fi
