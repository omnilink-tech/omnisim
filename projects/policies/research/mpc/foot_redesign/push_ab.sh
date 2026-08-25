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

# Deterministic in-engine push A/B: one calibrated forward shove (HSTAND_TEST_PUSH,
# no random cubes) on the orig vs bigfoot foot, swept over magnitude. Finds the level
# where the small foot topples but the big foot recovers. Clean (no throw RNG).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."; ROOT="$(pwd -W 2>/dev/null || pwd)"
export OMNISIM_HOME=$ROOT PYTHONUTF8=1 WEBOTS_EXTRA_PROJECT_PATH=$ROOT/projects/policies
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1 OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
export OMNISIM_NEWTON_TARGET_KE=400 OMNISIM_NEWTON_TARGET_KD=60 OMNISIM_NEWTON_GROUND_MU=2.0
export HUMANOID_STAND_SPEC=$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json
export HSTAND_ANK_BIAS=-0.06 HSTAND_LEAN=1 HSTAND_THROW=0
LOGD=_scratch/foot_redesign; : > $LOGD/push_ab.txt
for cfg in "$@"; do
  set -- $cfg; which=$1; mag=$2; ang=${3:-0}
  if [ "$which" = "big" ]; then W=projects/policies/worlds/g1_hstand_cubethrow_bigfoot.omniworld
  else W=projects/policies/worlds/g1_hstand_cubethrow.omniworld; fi
  tel=$ROOT/$LOGD/push_${which}_m${mag}_a${ang}.telemetry
  export HUMANOID_STAND_LOG=$tel OMNISIM_DEPLOY_LOG=$tel OMNISIM_LOG_PATH=$ROOT/$LOGD/push_${which}_m${mag}.enginelog
  : > "$LOGD/push_${which}_m${mag}_a${ang}.telemetry"
  export HSTAND_TEST_PUSH=$mag HSTAND_TEST_PUSH_ANG=$ang HSTAND_TEST_PUSH_T=4.0
  python -u scripts/dev/headless_runner.py "$W" --duration 14 >/dev/null 2>&1
  fall=$(grep -E 'FALL@' "$LOGD/push_${which}_m${mag}_a${ang}.telemetry" | tail -1)
  last=$(grep -E '\[hstand:g1\] t=' "$LOGD/push_${which}_m${mag}_a${ang}.telemetry" | tail -1)
  if [ -n "$fall" ]; then v="FELL"; else v="STOOD"; fi
  pk=$(grep -oE 'peakTilt=[0-9.]+' "$LOGD/push_${which}_m${mag}_a${ang}.telemetry" | sort -t= -k2 -n | tail -1)
  printf '%-16s push=%-4s ang=%-3s -> %-5s  %s  (%s)\n' "$which" "$mag" "$ang" "$v" "$pk" "${last##*OK  }" | tee -a $LOGD/push_ab.txt
done
echo "=== push A/B done ==="; cat $LOGD/push_ab.txt
