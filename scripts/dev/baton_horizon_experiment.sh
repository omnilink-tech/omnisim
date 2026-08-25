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

# THE BATON HORIZON EXPERIMENT -- the arm x seed matrix, harvested to one dataset.
#
# This is the experiment docs/developer/policy-switching.md has been asking for since
# it was written, and could not run until 2026-07-12 because the deploy emitted no
# per-cycle verdict. Now it does (g1_walk_recipe: _wr_course_advance ->
# "BATON-CYCLE k=.. ok=.. .."), so success-vs-horizon is measurable.
#
# ARMS
#   engineered  BATON as shipped: cold recurrent state at the stand->locomotion edge,
#               morph blend, phase-gated entry.
#   naive       the ablation: warm hidden carried across every switch, no morph. Same
#               specialists, same course, same physics. This is the "naive FSM
#               hierarchy" arm -- the honest control for "does the ENGINEERING matter".
#
# (The third arm of the full comparison -- a fairly-trained MONOLITH -- is a training
# campaign, not a deploy sweep, and is NOT run here. Do not claim
# "switching beats a monolith" from this experiment alone; it can only support the
# narrower, still-useful claim "an engineered handover beats a naive one".)
#
# SEEDS: DEPLOY_IC_SEED perturbs the initial leg pose (the same symmetry-breaking the
# trainer uses). Without a seed sweep the deploy is deterministic and you get a single
# survival horizon, not a success rate.
#
# Usage:  bash scripts/dev/baton_horizon_experiment.sh [cycles] [seeds] [dur_s]
#   e.g.  bash scripts/dev/baton_horizon_experiment.sh 6 5 900
#
# Output:  _scratch/baton_horizon/results.json   (+ per-run logs)
#          then:  python projects/policies/training/baton_metrics.py --horizon <tags...>
set -uo pipefail
cd "$(dirname "$0")/../.."

CYCLES="${1:-6}"
SEEDS="${2:-5}"
DUR="${3:-900}"
OUT=_scratch/baton_horizon
mkdir -p "$OUT"

echo "=========================================================================="
echo " BATON HORIZON EXPERIMENT   cycles=$CYCLES  seeds=1..$SEEDS  dur=${DUR}s/run"
echo " runs: $((2 * SEEDS))   (2 arms x $SEEDS seeds)"
echo "=========================================================================="

for ARM in engineered naive; do
  for S in $(seq 1 "$SEEDS"); do
    TAG="baton_horizon_${ARM}_s${S}"
    echo ""
    echo "--- [$ARM seed=$S] ---"
    bash projects/policies/demos/run_baton_horizon.sh "$CYCLES" "$ARM" "$DUR" headless "$S" \
      > "$OUT/${TAG}.stdout" 2>&1 || echo "    (launcher returned non-zero -- judging by the LOG, not the code)"
    # the verdict lives in the log, never in the exit status
    if [ -f "_scratch/foot_redesign/${TAG}_rl.txt" ]; then
      cp "_scratch/foot_redesign/${TAG}_rl.txt"  "$OUT/${TAG}_rl.txt"  2>/dev/null || true
      cp "_scratch/foot_redesign/${TAG}_mpc.txt" "$OUT/${TAG}_mpc.txt" 2>/dev/null || true
      grep -E 'BATON-CYCLE|BATON-COURSE DONE' "$OUT/${TAG}_rl.txt" | sed 's/^/    /' || echo "    NO CYCLE VERDICT (run produced nothing scoreable)"
    else
      echo "    ** no rl log produced -- run did not start"
    fi
  done
done

echo ""
echo "=========================================================================="
echo " HARVEST"
echo "=========================================================================="
python scripts/dev/baton_harvest.py "$OUT" "$CYCLES" "$SEEDS"
