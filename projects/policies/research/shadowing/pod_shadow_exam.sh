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

# THE THREE-WAY EXAM for SHADOW ITERATION -- v1 (incumbent) vs TREAT (ghost v2) vs CTRL (trained
# longer on ghost v1). Same pod, same world, same physics, same duration, run in PARALLEL (deploys
# are light -- they barely touch the GPU).
#
# Each policy rides THE GHOST IT WAS TRAINED ON. That is its deploy contract, and it is the same
# rule the round-1 head-to-head used: we are comparing two DEPLOYED SYSTEMS, not two networks in a
# vacuum.
#
# ⛔ `ONNX loaded:` is asserted for every arm. A bare-ghost replay walks, does not fall, and scores
# a near-ceiling gmatch because it IS the ghost -- that trap once voided an entire Go2 exam.
set -uo pipefail
cd /workspace/omnisim
DUR="${1:-240}"

export OMNISIM_HOME=/workspace/omnisim WEBOTS_HOME=/workspace/omnisim
export LD_LIBRARY_PATH=/workspace/omnisim/lib/webots
export QT_QPA_PLATFORM=xcb WEBOTS_TMPDIR=/tmp LIBGL_ALWAYS_SOFTWARE=1
export OMNISIM_REQUIRE_NEWTON=1

G1=/workspace/omnisim/projects/policies/ghosts/go2/go2_shadow_ghost_lut.json
G2=/workspace/omnisim/_scratch/go2_shadow_ghost_v2_lut.json
P_V1=/workspace/omnisim/projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx
P_TREAT=/workspace/champ_treat_s0/policy.onnx
P_CTRL=/workspace/champ_ctrl_s0/policy.onnx

run_arm () {                       # name policy ghost
  local NM=$1 POL=$2 GH=$3
  [ -f "$POL" ] || { echo "$NM: MISSING POLICY $POL"; return 1; }
  OMNISIM_LOG_PATH=/workspace/exam_${NM}_engine.txt \
  GO2_DEPLOY_LOG=/workspace/exam_${NM}.log \
  xvfb-run -a bash scripts/dev/run_go2_shadow_deploy.sh "$DUR" \
      --policy "$POL" --ghost "$GH" --ff > /workspace/exam_${NM}_console.log 2>&1
}

echo "=== running the three arms IN PARALLEL (${DUR}s wall each)"
run_arm v1    "$P_V1"    "$G1" &
run_arm treat "$P_TREAT" "$G2" &
run_arm ctrl  "$P_CTRL"  "$G1" &
wait

echo
echo "########## SHADOW ITERATION -- THE RULER ##########"
printf "%-7s %-9s %-9s %-9s %-8s %-8s %s\n" arm dist_m vx_ms ymax_m falls gmatch policy_loaded
for NM in v1 treat ctrl; do
  L=/workspace/exam_${NM}.log
  [ -f "$L" ] || { printf "%-7s (no log)\n" "$NM"; continue; }
  LOADED=$(grep -c "ONNX loaded:" "$L")
  LAST=$(grep '^\[t=' "$L" | tail -1)
  DIST=$(echo "$LAST" | grep -oE 'x=[-+0-9.]+' | cut -d= -f2)
  # steady speed: mean vx over the last 60% of the per-second samples
  VX=$(grep '^\[t=' "$L" | awk -F'vx=' 'NF>1{split($2,a," ");v[NR]=a[1]} END{s=0;n=0;for(i=int(NR*0.4);i<=NR;i++){s+=v[i];n++}printf "%.3f", (n?s/n:0)}')
  YMAX=$(grep -oE 'y=[-+0-9.]+' "$L" | cut -d= -f2 | awk '{a=($1<0?-$1:$1); if(a>m)m=a} END{printf "%.3f", m}')
  FALLS=$(grep -c FALL "$L")
  GM=$(grep -oE 'GMATCH FINAL mean=[0-9.]+' "$L" | cut -d= -f2)
  printf "%-7s %-9s %-9s %-9s %-8s %-8s %s\n" "$NM" "${DIST:-?}" "${VX:-?}" "${YMAX:-?}" "${FALLS:-?}" "${GM:-?}" "${LOADED}"
done
echo
echo "  (policy_loaded MUST be 1 for every arm -- a 0 means that arm ran the BARE GHOST and its"
echo "   numbers are meaningless, however good they look.)"
