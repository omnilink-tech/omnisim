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

#
# NOTE (Newton completion plan P4): launched from bash, omnisim-bin's embedded interpreter
# cannot find `warp`, so this script SILENTLY runs on ODE, not Newton. For a Newton-correct
# match runner that sets OMNISIM_DAMAGE_VEL_SMOOTH and reads the scorecard, use the PowerShell
# harness instead:
#     python scripts/dev/combat_match.py <world.omniworld> [timer_s] [vel_smooth] [newton|ode]
# Kept for the legacy ODE husky-hunt flow.
#
# combat_run.sh — run N consecutive husky_hunt combat matches headless,
# summarize victory/impact/detach counts per match. Each match runs
# until either side declares VICTORY or MATCH_TIMEOUT_S elapses.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OMNISIM_HOME="${OMNISIM_HOME:-$REPO_ROOT}"
OMNISIM_BIN="$OMNISIM_HOME/msys64/mingw64/bin/omnisim-bin.exe"
WORLD="$OMNISIM_HOME/projects/robot_combat/worlds/tests/newton_husky_head_on_2.omniworld"
N=${N:-3}
MATCH_TIMEOUT_S=${MATCH_TIMEOUT_S:-180}
POLL_S=4

SUMMARY="$OMNISIM_HOME/combat_matches.summary"
> "$SUMMARY"

# Snapshot pre-existing omnisim-bin PIDs so per-match and final cleanup kill only
# the children *this* run spawned. The old broad kill took down every OmniSim on
# the host, breaking parallel agent / harness sessions.
BASELINE_PIDS=$(powershell.exe -Command "(Get-Process omnisim-bin -ErrorAction SilentlyContinue | ForEach-Object { \$_.Id }) -join ','" 2>/dev/null | tr -d '\r')
kill_our_sim() {
    powershell.exe -Command "\$baseline = @($BASELINE_PIDS); Get-Process omnisim-bin -ErrorAction SilentlyContinue | Where-Object { \$baseline -notcontains \$_.Id } | ForEach-Object { Stop-Process -Id \$_.Id -Force }" >/dev/null 2>&1
}

for match in $(seq 1 "$N"); do
    echo "=== match $match/$N ===" | tee -a "$SUMMARY"
    # Kill any leftover from this script's previous iteration (not other sessions).
    kill_our_sim
    sleep 2
    rm -f "$OMNISIM_HOME/supervisor_stderr.log" \
          "$OMNISIM_HOME/omnisim_log.txt" \
          "$OMNISIM_HOME/launch_run.log" \
          "$OMNISIM_HOME"/husky_hunt_*.log

    OMNISIM_SUPERVISOR_STDERR_LOG="$OMNISIM_HOME/supervisor_stderr.log" \
        "$OMNISIM_BIN" --mode=realtime --no-rendering --batch "$WORLD" \
        > "$OMNISIM_HOME/launch_run.log" 2>&1 &
    sleep 5
    # Pick OUR omnisim-bin (the one not in BASELINE_PIDS), not the first one in
    # tasklist — otherwise a foreign OmniSim from another agent session would
    # be tracked by mistake.
    PID=$(powershell.exe -Command "Get-Process omnisim-bin -ErrorAction SilentlyContinue | Where-Object { @($BASELINE_PIDS) -notcontains \$_.Id } | Select-Object -First 1 -ExpandProperty Id" 2>/dev/null | tr -d '\r ')
    if [ -z "$PID" ]; then
        echo "  ERROR: omnisim-bin did not start" | tee -a "$SUMMARY"
        continue
    fi

    start_t=$(date +%s)
    victory=""
    while true; do
        if ! tasklist 2>/dev/null | grep -q " $PID "; then
            echo "  SIM-EXITED unexpectedly" | tee -a "$SUMMARY"
            break
        fi
        elapsed=$(( $(date +%s) - start_t ))
        # Check for victory
        v_a=$(grep -c "VICTORY" "$OMNISIM_HOME/husky_hunt_husky.log" 2>/dev/null || echo 0)
        v_b=$(grep -c "VICTORY" "$OMNISIM_HOME/husky_hunt_husky_b.log" 2>/dev/null || echo 0)
        if [ "$v_a" -gt 0 ] && [ -z "$victory" ]; then victory="husky"; fi
        if [ "$v_b" -gt 0 ] && [ -z "$victory" ]; then victory="husky_b"; fi
        if [ -n "$victory" ]; then break; fi
        if [ "$elapsed" -ge "$MATCH_TIMEOUT_S" ]; then
            echo "  TIMEOUT at ${elapsed}s" | tee -a "$SUMMARY"
            break
        fi
        sleep "$POLL_S"
    done

    # Gather stats from the run.
    imp_a=$(grep -c "IMPACT" "$OMNISIM_HOME/husky_hunt_husky.log" 2>/dev/null || echo 0)
    imp_b=$(grep -c "IMPACT" "$OMNISIM_HOME/husky_hunt_husky_b.log" 2>/dev/null || echo 0)
    stale_a=$(grep -c "STALEMATE" "$OMNISIM_HOME/husky_hunt_husky.log" 2>/dev/null || echo 0)
    stale_b=$(grep -c "STALEMATE" "$OMNISIM_HOME/husky_hunt_husky_b.log" 2>/dev/null || echo 0)
    detach_total=$(grep -c "detached" "$OMNISIM_HOME/supervisor_stderr.log" 2>/dev/null || echo 0)
    detach_wheel=$(grep "detached wheel_" "$OMNISIM_HOME/supervisor_stderr.log" 2>/dev/null | wc -l)
    sim_t_finish=$(grep "VICTORY" "$OMNISIM_HOME"/husky_hunt_*.log 2>/dev/null | grep -oE "t=[0-9.]+s" | head -1 | sed 's/t=\([0-9.]*\)s/\1/')

    echo "  winner=${victory:-NONE}  sim_finish=${sim_t_finish:-?}s  wall=${elapsed}s" | tee -a "$SUMMARY"
    echo "  impacts: husky=$imp_a husky_b=$imp_b" | tee -a "$SUMMARY"
    echo "  stalemates: husky=$stale_a husky_b=$stale_b" | tee -a "$SUMMARY"
    echo "  detaches: total=$detach_total wheels=$detach_wheel" | tee -a "$SUMMARY"
done

kill_our_sim
echo
echo "===== final summary ====="
cat "$SUMMARY"
