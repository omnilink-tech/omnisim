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

# Verification harnesses for the suction-gripper delivery demo (run_box_grasp.sh).
#
#   bash projects/policies/demos/verify_box_grasp.sh par     # 2x PARALLEL ship gate (~35 min)
#   bash projects/policies/demos/verify_box_grasp.sh place   # short-course place iteration (~10 min)
#
# par:   two isolated instances (per-instance TAG / BOXRIG_LOG / HAND_TRACK_FILE /
#        GHOST_FOLLOW_FILE / BATON_MODE_FILE -- shared telemetry files once interleaved and
#        masqueraded as fresh results). Engine TCP ports auto-scan; ~60% of sequential wall.
# place: pick -> direct carry -> place -> short exit; skips the post-pick turn + NE detour.
#        For PLACE-mechanics iteration only; the full course is the ship gate.
#
# Kills are PID-scoped by CommandLine world match: concurrent sessions run their own sims
# (stair climb, harness) that must never be touched.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
SD=_scratch/foot_redesign
MODE="${1:-par}"

# end-of-demo watcher: the sim/wall speed ratio varies 0.5-2.5x with GPU load, so a fixed
# duration either wastes many idle minutes (fast GPU) or truncates the demo (slow GPU).
# Poll the boxrig telemetry for the robot standing at the final north waypoint and kill
# the sim the moment the demo is complete.
wait_demo_done() {
  local LOGF="$1" DEADLINE=$((SECONDS + 2400))
  while [ $SECONDS -lt $DEADLINE ]; do
    if tail -1 "$LOGF" 2>/dev/null | grep -q 'mode=stand.*robot= *+10\.[78]' &&        tail -1 "$LOGF" 2>/dev/null | grep -qE 'robot= *\+10\.[0-9] *(\+1\.[0-9]|-2\.[3-9])'; then
      return 0
    fi
    sleep 10
  done
  return 1
}
busy() {
  powershell -NoProfile -Command "@(Get-CimInstance Win32_Process | Where-Object { \$_.Name -match 'omnisim' -and \$_.CommandLine -match 'g1_box_grasp' }).Count" | grep -qv '^0'
}
kill_mine() {
  powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { \$_.Name -match 'omnisim' -and \$_.CommandLine -match 'g1_box_grasp' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force -Confirm:\$false -ErrorAction SilentlyContinue }" 2>/dev/null
  sleep 3
}
if busy; then echo "PRECHECK: a g1_box_grasp sim is already running -- aborting"; exit 2; fi

if [ "$MODE" = "place" ]; then
  export COURSE='walkto,4.3,-0.98;stand,6;carryto,5.3,-2.1;carryto,8.55,-1.75;carryto,9.3,-1.55;stand,6;walkto,11.0,-2.6;stand,10'
  export EXTRA="GR_PLACE_SEG=5 GR_NAN_WATCH=1 ${PLACE_EXTRA:-}"
  rm -f "$SD/boxrig.log" "$SD/box_grasp_rl.txt" 2>/dev/null
  ( wait_demo_done "$SD/boxrig.log" && sleep 15 && kill_mine ) &
  W=$!
  bash projects/policies/demos/run_box_grasp.sh 420 headless > /dev/null 2>&1
  kill $W 2>/dev/null
  kill_mine
  echo "== events"
  grep -E 'SUCTION|PHYS-GRASP ->|PHYS-GRASP clearout|GR-NAN|guard tripped' "$SD/box_grasp_rl.txt" 2>/dev/null | head -24
  echo "== final"
  tail -2 "$SD/boxrig.log" 2>/dev/null
  grep -oE 'z=[0-9.]+ roll=[-0-9.]+ pit=[-0-9.]+' "$SD/box_grasp_mpc.txt" 2>/dev/null | tail -2
  echo PLACE_ITER_DONE
  exit 0
fi

run_one() {
  local N=$1
  ( export TAG="box_grasp_v$N" \
           BOXRIG_LOG="$PWD/$SD/boxrig_v$N.log" \
           HAND_TRACK_FILE="$PWD/$SD/rig_hands_v$N.json" \
           GHOST_FOLLOW_FILE="$SD/ghost_follow_v$N.json" \
           BATON_MODE_FILE="$PWD/$SD/boxmode_v$N.txt" \
           EXTRA='GR_NAN_WATCH=1'
    unset COURSE
    local A
    for A in 1 2 3; do   # per-instance launch-flake retry (the sim dies at load ~25% of runs)
      rm -f "$SD/boxrig_v$N.log" "$SD/box_grasp_v${N}_rl.txt" 2>/dev/null
      bash projects/policies/demos/run_box_grasp.sh 600 headless > /dev/null 2>&1
      [ "$(stat -c %s "$SD/boxrig_v$N.log" 2>/dev/null || echo 0)" -gt 5000 ] && break
    done )
}
run_one 1 & P1=$!
sleep 20   # stagger the cold loads
run_one 2 & P2=$!
( wait_demo_done "$SD/boxrig_v1.log" && wait_demo_done "$SD/boxrig_v2.log" && sleep 20 && kill_mine ) &
W=$!
wait $P1 $P2
kill $W 2>/dev/null
kill_mine
for N in 1 2; do
  echo "##### VERIFY RUN $N #####"
  grep -E 'SUCTION|PHYS-GRASP ->|PHYS-GRASP clearout|GR-NAN|guard tripped' "$SD/box_grasp_v${N}_rl.txt" 2>/dev/null | head -24
  echo "== final"
  tail -2 "$SD/boxrig_v$N.log" 2>/dev/null
  grep -oE 'z=[0-9.]+ roll=[-0-9.]+ pit=[-0-9.]+' "$SD/box_grasp_v${N}_mpc.txt" 2>/dev/null | tail -2
done
echo "ALL_VERIFY_DONE"
