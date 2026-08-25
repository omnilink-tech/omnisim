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

# Launch the BIGFOOT G1 stand world + the in-engine MPPI walk driver (g1_walk_mpc).
# Headless:  bash run_g1_walk.sh <dur> <tag>
# Live GUI:  bash run_g1_walk.sh <dur> <tag> gui
# Extra knobs as trailing ENV=val (e.g. GWM_VX=0.10 GWM_STEP_WIDTH=0.18).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."
DUR="${1:-30}"; TAG="${2:-g1walk}"; MODE="${3:-headless}"; shift 3 2>/dev/null || shift $# 2>/dev/null || true
export OMNISIM_HOME="$(pwd -W 2>/dev/null || pwd)"
export WEBOTS_HOME="$OMNISIM_HOME"
export WEBOTS_EXTRA_PROJECT_PATH="$OMNISIM_HOME/projects/policies"
export PYTHONUTF8=1
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1 OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
export OMNISIM_NEWTON_TARGET_KE=400 OMNISIM_NEWTON_TARGET_KD=60 OMNISIM_NEWTON_GROUND_MU=2.0   # g1 spec stiffness
export HUMANOID_STAND_SPEC="$OMNISIM_HOME/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json"
export HSTAND_ANK_BIAS=-0.06 HSTAND_LEAN=1
export HUMANOID_STAND_LOG="$OMNISIM_HOME/_scratch/foot_redesign/${TAG}.log"
export OMNISIM_DEPLOY_LOG="$HUMANOID_STAND_LOG"
export OMNISIM_LOG_PATH="$OMNISIM_HOME/_scratch/foot_redesign/${TAG}_omnisim.txt"
export OMNISIM_INENGINE_MPC_LOG="$OMNISIM_HOME/_scratch/foot_redesign/${TAG}_mpc.txt"
export HSTAND_WARMUP_RELOAD=1
export OMNISIM_WARMUP_TOKEN="g1walk_$$_${RANDOM}"
export OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_walk_mpc:g1_walk_step"
export PATH="$OMNISIM_HOME/msys64/mingw64/bin:$PATH"
for kv in "$@"; do export "$kv"; done
WORLD=projects/policies/worlds/g1_walk_bigfoot.omniworld
rm -f "$HUMANOID_STAND_LOG" "$OMNISIM_INENGINE_MPC_LOG" "$OMNISIM_LOG_PATH"
echo ">>> $TAG  mode=$MODE dur=${DUR}s  $(for kv in "$@"; do printf '%s ' "$kv"; done)"
if [ "$MODE" = "gui" ]; then
  python -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR" &
  echo "GUI launched (pid $!). Watch the window; MPC log -> _scratch/foot_redesign/${TAG}_mpc.txt"
else
  python -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" \
      > "$OMNISIM_HOME/_scratch/foot_redesign/${TAG}_console.txt" 2>&1
  echo "=== ${TAG} walk telemetry (last 14) ==="
  grep -E 'g1walk: (ready|walk start|graph|t=)' "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null | sed 's/np.int32(\([0-9]*\))/\1/g' | tail -14
  echo "=== stand settle / fall ==="
  grep -E '\[hstand:g1\] (settle|t=)|FALL@' "$HUMANOID_STAND_LOG" 2>/dev/null | tail -4
fi
