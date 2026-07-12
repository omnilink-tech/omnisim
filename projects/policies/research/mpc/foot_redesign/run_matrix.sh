#!/usr/bin/env bash
# Foot-redesign experiment matrix. Runs the offline MPPI humanoid walk on the
# original vs modified-foot model copies, identical control config per group, and
# writes one log per run + a one-line summary to results.txt.
# Usage: bash run_matrix.sh <group>   where group in {g1walk, g1det, h1walk, all}
set -u
cd "$(dirname "$0")/../../../../.."   # -> repo root
EXP=projects/policies/research/mpc/foot_redesign
PY="python -u $EXP/walk_exp.py"
LOG=_scratch/foot_redesign
mkdir -p "$LOG"
RES="$LOG/results.txt"

run () {  # run <tag> <args...>
  local tag="$1"; shift
  echo ">>> $tag : $*"
  timeout 600 $PY "$@" > "$LOG/$tag.log" 2>&1
  local last; last="$(grep -E '^\[walk\] (g1|h1)' "$LOG/$tag.log" | tail -1)"
  local fell; fell="$(grep -E 'FELL @' "$LOG/$tag.log" | tail -1)"
  printf '%-22s | %s | %s\n' "$tag" "${last:-NO-SUMMARY}" "${fell:-no-fall-line}" | tee -a "$RES"
}

G1WALK="--secs 9 --vx 0.20"
# documented H1 upright config (slow, lateral-tuned)
H1UP="--secs 12 --vx 0.12 --freq 0.80 --w-y 75 --w-rate 9 --w-yaw 13 --w-over 40"

case "${1:-all}" in
  g1walk)
    run g1_orig_walk        --robot g1_orig        $G1WALK
    run g1_long_walk        --robot g1_long        $G1WALK
    run g1_long_strong_walk --robot g1_long_strong $G1WALK
    run g1_big_walk         --robot g1_big         $G1WALK
    ;;
  g1det)   # deterministic realtime balance (no MPPI) — the deployable controller
    run g1_orig_det --robot g1_orig --det --secs 9 --vx 0.15 --k-vx 0.4
    run g1_big_det  --robot g1_big  --det --secs 9 --vx 0.15 --k-vx 0.4
    ;;
  g1base)  # bare gait, no balance at all — raw morphological stability
    run g1_orig_base --robot g1_orig --baseline --secs 6 --vx 0.15
    run g1_big_base  --robot g1_big  --baseline --secs 6 --vx 0.15
    ;;
  h1walk)
    run h1_orig_walk    --robot h1_orig    $H1UP
    run h1_wide_walk    --robot h1_wide    $H1UP
    run h1_wide_xl_walk --robot h1_wide_xl $H1UP
    ;;
  g1tuned)   # G1 under the EXACT lateral-tuned config that made H1 durable (apples-to-apples)
    run g1_orig_tuned --robot g1_orig $H1UP
    run g1_big_tuned  --robot g1_big  $H1UP
    ;;
  g1lat)     # bigfoot sagittal wall is gone -> attack the new LATERAL binding axis with stance width
    run g1_big_sw16 --robot g1_big $H1UP --step-width 0.16
    run g1_big_sw20 --robot g1_big $H1UP --step-width 0.20
    run g1_big_sw20_wy --robot g1_big --secs 12 --vx 0.10 --freq 0.80 --w-y 120 --w-rate 12 --w-yaw 16 --w-over 40 --step-width 0.20
    ;;
  stand)   # passive push-recovery basin (no gait, no balance) via stand_basin.py
    sb () { local tag="$1" robot="$2" m="$3"
      echo ">>> basin $tag"
      timeout 400 python -u "$EXP/stand_basin.py" --model "$EXP/models/$m.mjcf.xml" --robot "$robot" \
        > "$LOG/basin_$tag.log" 2>&1
      printf '%-18s | %s\n' "$tag" "$(grep 'BASIN:' "$LOG/basin_$tag.log" | tail -1)" | tee -a "$LOG/basin_results.txt"; }
    : > "$LOG/basin_results.txt"
    sb g1_orig  g1 g1_orig_legs
    sb g1_long  g1 g1_longfoot_legs
    sb g1_lstr  g1 g1_longfoot_strong_legs
    sb g1_big   g1 g1_bigfoot_legs
    sb h1_orig  h1 h1_orig
    sb h1_wide  h1 h1_widefoot
    sb h1_wxl   h1 h1_widefoot_xl
    echo "=== basin results ==="; cat "$LOG/basin_results.txt"; exit 0;;
  standlean)  # push-recovery basin WITH the reactive ankle lean (the real stand mechanism)
    sbl () { local tag="$1" robot="$2" m="$3"; shift 3
      echo ">>> leanbasin $tag"
      timeout 400 python -u "$EXP/stand_basin.py" --model "$EXP/models/$m.mjcf.xml" --robot "$robot" \
        --lean --lean-sign 1 "$@" > "$LOG/lean_$tag.log" 2>&1
      printf '%-18s | %s\n' "$tag" "$(grep 'BASIN:' "$LOG/lean_$tag.log" | tail -1)" | tee -a "$LOG/lean_results.txt"; }
    : > "$LOG/lean_results.txt"
    sbl g1_orig  g1 g1_orig_legs
    sbl g1_long  g1 g1_longfoot_legs
    sbl g1_lstr  g1 g1_longfoot_strong_legs
    sbl g1_big   g1 g1_bigfoot_legs
    sbl h1_orig  h1 h1_orig          --ke 800 --kd 70
    sbl h1_wide  h1 h1_widefoot      --ke 800 --kd 70
    sbl h1_wxl   h1 h1_widefoot_xl   --ke 800 --kd 70
    echo "=== lean basin results ==="; cat "$LOG/lean_results.txt"; exit 0;;
  *) echo "groups: g1walk g1det g1base h1walk g1tuned stand"; exit 1;;
esac
echo "=== results so far ==="; cat "$RES"
