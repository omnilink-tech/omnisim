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
# cannot find `warp`, so this script SILENTLY runs on ODE, not Newton (and it polls a stale
# pre-_scratch scorecard path). For a Newton-correct, scorecard-reading match runner use the
# PowerShell harness instead:
#     python scripts/dev/combat_match.py <world.omniworld> [timer_s] [vel_smooth] [newton|ode]
# Kept for the legacy ODE bracket flow.
#
# battlebox_tournament.sh — run a BattleBox bracket headless. Each match
# reads the JSON scorecard the match_director writes to determine the
# winner, then advances them into the next round. Output is a final
# bracket summary + per-match scorecards.
#
# USAGE
#   N=8 bash scripts/battlebox_tournament.sh
#
# ENV
#   WORLD     world to use for each match (default: battlebox_duel.omniworld).
#             For now the bracket is virtual — we just re-run the same
#             duel world N-1 times and treat each scorecard as a match
#             between the (random) bot pair. A future iteration will
#             rewrite the world's RED/BLUE weapon types per round.
#   N         number of seeds (rounded up to the next power of 2)
#   MATCH_TIMEOUT_S  per-match wall-clock cap (default 200)
#
# OUTPUT
#   $OMNISIM_HOME/battlebox_bracket.summary
#   $OMNISIM_HOME/battlebox_match_<round>_<seedA>v<seedB>.json
#
# Bracket layout for N=8 (single-elim):
#   QF: 1v8 4v5 2v7 3v6  -> winners
#   SF: 1/8 vs 4/5,  2/7 vs 3/6
#   F:  SF winners
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OMNISIM_HOME="${OMNISIM_HOME:-$REPO_ROOT}"
OMNISIM_BIN="$OMNISIM_HOME/msys64/mingw64/bin/omnisim-bin.exe"
WORLD="${WORLD:-$OMNISIM_HOME/projects/robot_combat/battlebots/worlds/battlebox_duel.omniworld}"
N=${N:-4}
MATCH_TIMEOUT_S=${MATCH_TIMEOUT_S:-200}
POLL_S=4

# Round N up to the nearest power of 2.
pow2=1
while [ "$pow2" -lt "$N" ]; do pow2=$((pow2 * 2)); done
N=$pow2

SUMMARY="$OMNISIM_HOME/battlebox_bracket.summary"
> "$SUMMARY"
echo "BattleBox bracket — $N fighters (single-elim)" | tee -a "$SUMMARY"

# Snapshot pre-existing omnisim-bin so we don't kill foreign sessions.
BASELINE_PIDS=$(powershell.exe -Command "(Get-Process omnisim-bin -ErrorAction SilentlyContinue | ForEach-Object { \$_.Id }) -join ','" 2>/dev/null | tr -d '\r')
kill_our_sim() {
    powershell.exe -Command "\$baseline = @($BASELINE_PIDS); Get-Process omnisim-bin -ErrorAction SilentlyContinue | Where-Object { \$baseline -notcontains \$_.Id } | ForEach-Object { Stop-Process -Id \$_.Id -Force }" >/dev/null 2>&1
}

run_match() {
    local label=$1
    local match_json="$OMNISIM_HOME/battlebox_match_${label}.json"
    rm -f "$match_json" "$OMNISIM_HOME/launch_run.log" "$OMNISIM_HOME"/battlebot_*.log

    OMNISIM_SUPERVISOR_STDERR_LOG="$OMNISIM_HOME/supervisor_stderr.log" \
        "$OMNISIM_BIN" --mode=realtime --no-rendering --batch "$WORLD" \
        > "$OMNISIM_HOME/launch_run.log" 2>&1 &
    sleep 5
    local PID
    PID=$(powershell.exe -Command "Get-Process omnisim-bin -ErrorAction SilentlyContinue | Where-Object { @($BASELINE_PIDS) -notcontains \$_.Id } | Select-Object -First 1 -ExpandProperty Id" 2>/dev/null | tr -d '\r ')
    if [ -z "$PID" ]; then
        echo "  ERROR: omnisim-bin did not start for $label" | tee -a "$SUMMARY"
        return 1
    fi

    local start_t winner
    start_t=$(date +%s)
    winner=""
    while true; do
        if ! tasklist 2>/dev/null | grep -q " $PID "; then break; fi
        elapsed=$(( $(date +%s) - start_t ))
        if [ -f "$OMNISIM_HOME/battlebox_match.json" ]; then
            # Copy & extract winner.
            cp "$OMNISIM_HOME/battlebox_match.json" "$match_json"
            winner=$(python -c "import json; print(json.load(open('$match_json')).get('winner',''))" 2>/dev/null)
            if [ -n "$winner" ]; then break; fi
        fi
        if [ "$elapsed" -ge "$MATCH_TIMEOUT_S" ]; then
            echo "  TIMEOUT at ${elapsed}s for $label" | tee -a "$SUMMARY"
            break
        fi
        sleep "$POLL_S"
    done

    kill_our_sim
    sleep 2
    echo "  $label -> winner=${winner:-NONE} (${elapsed}s wall)" | tee -a "$SUMMARY"
}

# Round-robin labels. Each label is processed sequentially: QF, SF, F.
round=1
remaining=$N
while [ "$remaining" -gt 1 ]; do
    pairs=$((remaining / 2))
    echo "" | tee -a "$SUMMARY"
    echo "== round $round ($pairs matches) ==" | tee -a "$SUMMARY"
    for i in $(seq 1 "$pairs"); do
        run_match "r${round}_m${i}"
    done
    remaining=$pairs
    round=$((round + 1))
done

kill_our_sim
echo
echo "===== bracket summary ====="
cat "$SUMMARY"
