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

# WARP CONTACT-KERNEL REPRO RUNNER (2026-07-10, the grasp campaign's residue): step each
# minimal physics-only world (projects/policies/worlds/repro/warp_contact_*.wbt) under the
# WALK STACK's exact solver env, on BOTH engines (mujoco_warp and the CPU reference), and
# report DIVERGED/CLEAN per cell. No robots, no policies -- a NaN here is a pure solver
# defect, and the CPU column is the ground truth the warp column must match.
# Detector: the base-divergence guard's console print ("base-divergence guard tripped")
# lands in the per-run OMNISIM_LOG_PATH.
# Usage: bash scripts/dev/run_warp_contact_repro.sh [duration_s]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
DUR="${1:-30}"
OUT=_scratch/foot_redesign/warp_repro
mkdir -p "$OUT"
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1
export OMNISIM_NEWTON_TARGET_KE=200 OMNISIM_NEWTON_TARGET_KD=30 OMNISIM_NEWTON_GROUND_MU=2.0
# tier-2 cases (D/E) carry the G1 running its deterministic stand controller (no pymod, no
# harness): same env the walk launcher gives it
ROOT="$(pwd -W 2>/dev/null || pwd)"
export HUMANOID_STAND_SPEC="$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json"
export HSTAND_ANK_BIAS=-0.06 HSTAND_LEAN=0
export OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
echo "case,engine,verdict,detail"
# Dual-read: the corpus migrated .wbt -> .omniworld, and a .wbt-only glob
# matched nothing, so this loop body never ran -- the script printed its CSV
# header and exited 0, reporting success for zero cases.
REPRO_WORLDS=$(ls projects/policies/worlds/repro/warp_contact_*.omniworld \
                  projects/policies/worlds/repro/warp_contact_*.wbt 2>/dev/null)
if [ -z "$REPRO_WORLDS" ]; then
  echo "run_warp_contact_repro: no warp_contact_* worlds found under projects/policies/worlds/repro/" >&2
  exit 1
fi
for W in $REPRO_WORLDS; do
  NAME=$(basename "$W"); NAME="${NAME%.*}"
  for ENG in warp cpu; do
    LOG="$OUT/${NAME}_${ENG}.txt"
    rm -f "$LOG"
    if [ "$ENG" = "warp" ]; then export OMNISIM_NEWTON_MJWARP=1; else unset OMNISIM_NEWTON_MJWARP; fi
    OMNISIM_LOG_PATH="$LOG" python -m omnisim run-headless "$W" --duration "$DUR" > "$OUT/${NAME}_${ENG}_console.txt" 2>&1
    if grep -q "base-divergence guard tripped" "$LOG" 2>/dev/null; then
      V="DIVERGED"; D=$(grep -m1 "guard tripped" "$LOG" | head -c 80)
    elif grep -q "world finalised" "$LOG" 2>/dev/null; then
      V="CLEAN"; D=$(grep -m1 "world finalised" "$LOG" | sed 's/.*solver=//' | head -c 60)
    else
      V="NO-RUN"; D="load flake or missing log"
    fi
    echo "$NAME,$ENG,$V,$D"
  done
done
