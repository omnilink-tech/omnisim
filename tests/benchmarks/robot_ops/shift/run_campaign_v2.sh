#!/usr/bin/env bash
# Shift v2 campaign, exactly as FREEZE_V2.md states:
#   1. five waves, one after another, each ONE process with all 7 arms at once;
#   2. one re-run wave of every ERROR episode, together;
#   3. the independent judge over every run directory;
#   4. the preregistered analysis.
# The spend cap is shared across all of it.
#
#   bash tests/benchmarks/robot_ops/shift/run_campaign_v2.sh <OmniKey file> [first wave] [out prefix]
set -u
KEY="$1"; FIRST="${2:-0}"; PREFIX="${3:-tests/benchmarks/robot_ops/evidence/shift-holdout-v2}"
CAP_USD=95
WAVES=5
SUITE=tests/benchmarks/robot_ops/suites/shift_holdout_v2.json
PY=tests/benchmarks/harness_comparison/.venv/Scripts/python.exe
[ -x "$PY" ] || PY=python
ARMS="omnilink plain_basic langgraph_basic lobster_basic plain_full langgraph_full lobster_full"
DROP="${DROP:-}"            # checks dropped before the freeze, e.g. DROP=5,10 (see FREEZE_V2.md)

spent() {  # model spend so far across every directory of this campaign, judge included
  python - "$PREFIX" <<'PY'
import glob, json, sys, pathlib
tot = 0.0
for d in glob.glob(sys.argv[1] + "-*"):
    p = pathlib.Path(d)
    for name, key in (("rows.jsonl", "cost"), ("health.jsonl", None)):
        f = p / name
        if f.exists():
            for line in f.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    rec = json.loads(line)
                    tot += (rec.get("cost") or {}).get("usd_estimated", 0) if key else rec.get("usd_estimated", 0)
print(f"{tot:.4f}")
PY
}

wait_quiet() {
  until [ "$(tasklist //FI "IMAGENAME eq omnisim-bin.exe" 2>/dev/null | grep -c omnisim-bin)" = "0" ]; do
    echo "waiting: another omnisim-bin is running"; sleep 30
  done
}

run_wave() {  # $1 = out dir, $2 = repeat base, rest = arms
  local out="$1" base="$2"; shift 2
  wait_quiet
  local left; left=$(python -c "print(max(0.0, $CAP_USD - $(spent)))")
  echo "$(basename "$out"): cap left \$$left  ($(date +%H:%M))"
  $PY -m omnisim ops-bench run --suite "$SUITE" --arms "$@" --parallel "$#" \
    --repeat-base "$base" --repeats 1 --engine g1-engine --model gemini-3.5-flash \
    --cap-usd "$left" --rates 1.5,0.15,9.0 --max-requests 1000 \
    --health-median-s 8 --health-max-wait-h 2 --out "$out" --key-file "$KEY"
  echo "$(basename "$out") exit $?  ($(date +%H:%M))"
}

for ((r = FIRST; r < WAVES; r++)); do
  run_wave "$PREFIX-wave$r" "$r" $ARMS
done

# Every ERROR episode, re-run once, all together (as the scored episodes ran).
ERR=$(python - "$PREFIX" <<'PY'
import glob, json, sys, pathlib
arms = []
for d in sorted(glob.glob(sys.argv[1] + "-wave*")):
    for line in (pathlib.Path(d) / "rows.jsonl").read_text(encoding="utf-8").splitlines():
        if line.strip() and json.loads(line)["outcome"] == "ERROR":
            arms.append(json.loads(line)["arm"])
print(" ".join(arms))
PY
)
if [ -n "$ERR" ]; then run_wave "$PREFIX-rerun" "$WAVES" $ERR; fi

# The independent judge, then the analysis.
DIRS=$(ls -d "$PREFIX"-wave* "$PREFIX"-rerun 2>/dev/null)
$PY -m omnisim ops-bench judge $DIRS --suite "$SUITE" --engine g1-engine --model gemini-2.5-pro \
  --key-file "$KEY"
python tests/benchmarks/robot_ops/analyze_shift.py $DIRS --rule=mw ${DROP:+--drop=$DROP} \
  > "$PREFIX-analysis.json"
echo "campaign done: spent \$$(spent)  analysis -> $PREFIX-analysis.json"
