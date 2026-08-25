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

# One-off damage-cadence FPS bench (2026-05). Kept for the record of that
# measurement; for current benchmarks use tests/benchmarks/optim_bench.py.
#
# Run damage demo at each cadence, collect FPS samples, summarize.
# For each cadence: 40-second window. Skip first 3 samples (warmup) and
# report median of the remaining.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OMNISIM_HOME="${OMNISIM_HOME:-$REPO_ROOT}"
OMNISIM_BIN="$OMNISIM_HOME/msys64/mingw64/bin/omnisim-bin.exe"
WORLD="$OMNISIM_HOME/projects/robot_combat/worlds/tests/newton_husky_head_on_damage.omniworld"
FPS_LOG="$OMNISIM_HOME/sim_fps.log"
RUN_S=45

# Snapshot pre-existing omnisim-bin PIDs so the per-cadence and final cleanup
# kills only the children *this* sweep spawned. Without this we take down every
# OmniSim on the host — any unrelated agent or harness session running in
# parallel — which is exactly the bug we're trying to avoid.
BASELINE_PIDS=$(powershell.exe -Command "(Get-Process omnisim-bin -ErrorAction SilentlyContinue | ForEach-Object { \$_.Id }) -join ','" 2>/dev/null | tr -d '\r')
kill_our_sim() {
    powershell.exe -Command "\$baseline = @($BASELINE_PIDS); Get-Process omnisim-bin -ErrorAction SilentlyContinue | Where-Object { \$baseline -notcontains \$_.Id } | ForEach-Object { Stop-Process -Id \$_.Id -Force }" >/dev/null 2>&1
}

declare -a CADENCES=(1 2 4 8 16 32)
declare -a RESULTS

for CADENCE in "${CADENCES[@]}"; do
    echo "=== cadence=$CADENCE: starting ===" >&2
    kill_our_sim
    sleep 3
    rm -f "$FPS_LOG"

    OMNISIM_DAMAGE_POLL_EVERY=$CADENCE \
    OMNISIM_FPS_LOG="$FPS_LOG" \
        "$OMNISIM_BIN" --mode=run "$WORLD" > /dev/null 2>&1 &
    sleep $RUN_S

    # Drop first 3 samples (warmup), compute median of speed_window from the rest.
    median=$(tail -n +5 "$FPS_LOG" 2>/dev/null \
        | awk '/speed_window/ { for (i=1;i<=NF;i++) if (match($i,/speed_window=/)) { v=$i; sub("speed_window=","",v); sub("x","",v); print v } }' \
        | sort -n \
        | awk 'BEGIN{c=0} {a[c++]=$1} END{ if(c==0){print "0.00";exit} if(c%2){print a[(c-1)/2]} else { printf "%.2f\n",(a[c/2-1]+a[c/2])/2 } }')

    samples=$(tail -n +5 "$FPS_LOG" 2>/dev/null | wc -l)
    last=$(tail -1 "$FPS_LOG" 2>/dev/null | tr -s ' ')
    echo "cadence=$CADENCE median_speed=${median}x samples=$samples last={$last}" | tee -a "$OMNISIM_HOME/fps_sweep.summary.txt"
    RESULTS+=("cadence=$CADENCE median=${median}x")
done

kill_our_sim

echo
echo "===== SUMMARY ====="
for r in "${RESULTS[@]}"; do echo "  $r"; done
