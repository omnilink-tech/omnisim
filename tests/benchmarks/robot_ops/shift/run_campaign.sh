#!/usr/bin/env bash
# Warehouse shift v1 campaign, exactly as FREEZE.md states: three waves, one
# after another, each ONE process running all 7 arms at once. The spend cap is
# shared: each wave gets what the earlier waves left of CAP_USD.
#
#   bash tests/benchmarks/robot_ops/shift/run_campaign.sh <omnikey file> [first wave]
set -u
KEY="$1"; FIRST="${2:-0}"
CAP_USD=45
PY=tests/benchmarks/harness_comparison/.venv/Scripts/python.exe
EVI=tests/benchmarks/robot_ops/evidence
ARMS="omnilink plain_basic langgraph_basic lobster_basic plain_full langgraph_full lobster_full"

spent() {
  python - "$@" <<'PY'
import json, sys, pathlib
tot = 0.0
for d in sys.argv[1:]:
    p = pathlib.Path(d) / "rows.jsonl"
    if p.exists():
        for line in p.read_text(encoding="utf-8").splitlines():
            if line.strip():
                tot += (json.loads(line).get("cost") or {}).get("usd_estimated", 0)
    h = pathlib.Path(d) / "health.jsonl"
    if h.exists():
        for line in h.read_text(encoding="utf-8").splitlines():
            if line.strip():
                tot += json.loads(line).get("usd_estimated", 0)
print(f"{tot:.4f}")
PY
}

for r in 0 1 2; do
  [ "$r" -lt "$FIRST" ] && continue
  # A wave starts only on a machine with no other simulator running.
  until [ "$(tasklist //FI "IMAGENAME eq omnisim-bin.exe" 2>/dev/null | grep -c omnisim-bin)" = "0" ]; do
    echo "waiting: another omnisim-bin is running"; sleep 30
  done
  DONE=$(spent $EVI/shift-holdout-v1-wave0 $EVI/shift-holdout-v1-wave1 $EVI/shift-holdout-v1-wave2)
  LEFT=$(python -c "print(max(0.0, $CAP_USD - $DONE))")
  echo "wave $r: spent so far \$$DONE, cap for this wave \$$LEFT  ($(date +%H:%M))"
  $PY -m omnisim ops-bench run \
    --suite tests/benchmarks/robot_ops/suites/shift_holdout_v1.json \
    --arms $ARMS --parallel 7 --repeat-base $r --repeats 1 \
    --engine g1-engine --model gemini-3.5-flash \
    --cap-usd "$LEFT" --rates 1.5,0.15,9.0 --max-requests 1000 \
    --health-median-s 8 --health-max-wait-h 2 \
    --out $EVI/shift-holdout-v1-wave$r --key-file "$KEY"
  echo "wave $r exit $?  ($(date +%H:%M))"
done
echo "campaign done"
