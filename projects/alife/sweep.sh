#!/usr/bin/env bash
# Serial contact/scale sweep. ONE engine process at a time (thermal limit).
set -u
run() {
  name="$1"; shift
  echo "=== $name ==="
  env "$@" TERRA_OUT="sw_$name" python projects/alife/gen_terrarium.py >/dev/null
  rm -f projects/alife/_probe_result.json
  L="$PWD/projects/alife/_sw_${name}.log"; rm -f "$L" "$L.newton.json"
  OMNISIM_LOG_PATH="$L" PROBE_TICKS=300 timeout 240 \
    python -m omnisim run-headless "projects/alife/worlds/sw_$name.omniworld" \
    --duration 70 >/dev/null 2>&1
  cp projects/alife/_probe_result.json "projects/alife/_res_${name}.json" 2>/dev/null
  PYTHONIOENCODING=utf-8 python -c "
import json,sys
try:
    d=json.load(open('projects/alife/_res_${name}.json'))
    p4=d['p4']; p6=d['p6']
    print('  ticks=%s  engine=%.2f ms/step  best_locomotion=%.3f m'%(
        d['p1'].get('ticks'), p4.get('engine_ms_per_step_median',-1), p6.get('best_m') or 0))
except Exception as e: print('  FAILED:',e)
"
}
run base   TERRA_N=12
run soft   TERRA_N=12 TERRA_KE=2000 TERRA_KD=80 TERRA_CONE=pyramidal TERRA_IMPRATIO=1
run n4     TERRA_N=4
run n24    TERRA_N=24
