#!/usr/bin/env bash
# Contact-model sweep for hydra_vs_gravedigger (2026-09-09 realism pass).
# Five sibling copies of the rewritten world that differ ONLY in WorldInfo
# newtonContactKe / newtonContactKd / newtonSubsteps; each runs a 25 s traced
# match through scripts/dev/combat_match.py, then all are scored against the
# pre-change baseline with scripts/dev/combat_realism_report.py.
set -u
cd /o/omnisim
W=projects/robot_combat/battlebots/worlds
SRC=$W/hydra_vs_gravedigger.omniworld
OUT=_scratch/realism
mkdir -p $OUT
export OMNISIM_REQUIRE_NEWTON=1
TIMER=${TIMER:-25}

gen() {
  local name=$1; shift
  sed "$@" $SRC > $W/.sweep_${name}.omniworld
}
# v0: the new bot specs on the STOCK contact model (20 ms, 4 sub-steps) -- the control
gen v0_stock -e '/newtonContactKe 111000/d' -e '/newtonContactKd 667/d' -e 's/newtonSubsteps 12/newtonSubsteps 4/'
# v1: 5 ms critically damped, 8 sub-steps (0.5 ms)
gen v1_t5    -e 's/newtonContactKe 111000/newtonContactKe 40000/' -e 's/newtonContactKd 667/newtonContactKd 400/' -e 's/newtonSubsteps 12/newtonSubsteps 8/'
# v2: 3 ms critically damped, 12 sub-steps (0.33 ms) -- what the world file carries
cp $SRC $W/.sweep_v2_t3.omniworld
# v3: 3 ms, damping ratio 0.7 (a little restitution), 12 sub-steps
gen v3_t3z07 -e 's/newtonContactKe 111000/newtonContactKe 227000/'
# v4: 2 ms critically damped, 16 sub-steps (0.25 ms)
gen v4_t2    -e 's/newtonContactKe 111000/newtonContactKe 250000/' -e 's/newtonContactKd 667/newtonContactKd 1000/' -e 's/newtonSubsteps 12/newtonSubsteps 16/'

for v in v0_stock v1_t5 v2_t3 v3_t3z07 v4_t2; do
  echo "=== $v ($(date +%T)) ==="
  grep -E 'newtonContactK|newtonSubsteps' $W/.sweep_$v.omniworld | tr -s ' ' | tr '\n' ';'; echo
  export OMNISIM_LOG_PATH=$PWD/$OUT/sweep_$v.log
  export OMNISIM_DAMAGE_TRACE=$PWD/$OUT/sweep_$v.jsonl
  rm -f _scratch/hydra_vs_gravedigger.json
  start=$(date +%s)
  python scripts/dev/combat_match.py $W/.sweep_$v.omniworld $TIMER 2>&1 | tail -6
  echo "wall=$(( $(date +%s) - start ))s"
  cp _scratch/hydra_vs_gravedigger.json $OUT/sweep_${v}_scorecard.json 2>/dev/null || echo "no scorecard"
  ls $OMNISIM_LOG_PATH.newton.json >/dev/null 2>&1 && echo "newton sidecar OK" || echo "NO NEWTON SIDECAR"
  grep -c -E '^WARNING' $OMNISIM_LOG_PATH | sed 's/^/warnings in log: /'
done
echo "=== report ==="
python scripts/dev/combat_realism_report.py $OUT/before.jsonl $OUT/sweep_v0_stock.jsonl $OUT/sweep_v1_t5.jsonl $OUT/sweep_v2_t3.jsonl $OUT/sweep_v3_t3z07.jsonl $OUT/sweep_v4_t2.jsonl \
  --labels before v0_stock v1_t5 v2_t3 v3_t3z07 v4_t2 \
  --scorecard $OUT/before_scorecard.json $OUT/sweep_v0_stock_scorecard.json $OUT/sweep_v1_t5_scorecard.json $OUT/sweep_v2_t3_scorecard.json $OUT/sweep_v3_t3z07_scorecard.json $OUT/sweep_v4_t2_scorecard.json \
  --bar-inertia 4.8 2.5 2.5 2.5 2.5 2.5 --md $OUT/sweep_report.md --json $OUT/sweep_report.json
echo "=== sweep done $(date +%T) ==="
