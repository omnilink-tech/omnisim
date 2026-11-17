#!/usr/bin/env bash
# Run one ARM of the before/after A/B: 3 cells of A1_husky_swarm_10, strictly
# sequential, 10-minute session budget, engine sha pinned, GPU held under the
# owner's thermal ceiling between cells.
#
#   run_arm.sh <armA|armB> <engine_sha256> [n_cells]
set -u
ARM="$1"; SHA="$2"; N="${3:-3}"
WT="/o/omnisim-ab/$ARM"
ROOT="C:\\Users\\<user>\\AppData\\Local\\Temp\\agentbench_cc_$ARM"
LOCKS="C:\\Users\\<user>\\AppData\\Local\\Temp\\agentbench_cc\\locks"
RES="O:\\omnisim\\tests\\benchmarks\\agentbench\\results\\cc_lane"
TEMPCSV="/o/omnisim/tests/benchmarks/agentbench/results/cc_lane/20260812_beforeafter_gpu_temp.csv"

gpu() { nvidia-smi --query-gpu=temperature.gpu,utilization.gpu --format=csv,noheader 2>/dev/null | head -1; }
cpu() { python -c "import psutil;print(int(psutil.cpu_percent(interval=1.0)))" 2>/dev/null || echo "?"; }
builders() { tasklist /FO CSV /NH 2>/dev/null | grep -icE '^"(make|cc1plus|g\+\+|ld|cc1)\.exe' || echo 0; }
gput() { nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader 2>/dev/null | head -1 | tr -d ' '; }

[ -f "$TEMPCSV" ] || echo "utc,arm,cell,phase,temp_c,util_pct" > "$TEMPCSV"

for i in $(seq 1 "$N"); do
  # --- thermal gate: never start a cell above 70 C (ceiling is 75 C) --------
  while :; do
    T=$(gput); [ -z "$T" ] && T=0
    if [ "$T" -lt 70 ]; then break; fi
    echo "[$ARM c$i] GPU ${T}C >= 70C, cooling..."
    sleep 45
  done
  echo "$(date -u +%FT%TZ),$ARM,c$i,pre,$(gpu),cpu=$(cpu),builders=$(builders)" | tr -d ' ' >> "$TEMPCSV"
  echo "=== $(date -u +%FT%TZ) $ARM cell $i starting (GPU $(gpu)) ==="

  cd "$WT" || exit 1
  python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py \
      --sim omnisim --task A1_husky_swarm_10 \
      --model claude-opus-5 \
      --timeout-s 600 \
      --root "$ROOT" \
      --lock-root "$LOCKS" \
      --lane "ba_${ARM}_c${i}" \
      --out "${RES}\\20260812_ba_${ARM}_c${i}" \
      --pin-engine-sha256 "$SHA" \
      > "/o/omnisim/_scratch/ab_beforeafter/${ARM}_c${i}.runner.log" 2>&1
  RC=$?
  echo "$(date -u +%FT%TZ),$ARM,c$i,post,$(gpu),cpu=$(cpu),builders=$(builders)" | tr -d ' ' >> "$TEMPCSV"
  echo "=== $(date -u +%FT%TZ) $ARM cell $i done rc=$RC (GPU $(gpu)) ==="
  tail -6 "/o/omnisim/_scratch/ab_beforeafter/${ARM}_c${i}.runner.log"
done
echo "ARM $ARM COMPLETE"
