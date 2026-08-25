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

# GRADE A WALK CHAMPION — the FREE-STANDING exam (2026-07-13).
#
#   bash projects/policies/demos/grade_walk_champion.sh <ckpt.pt> [lam=0] [dur_s=150]
#
# The walk side has never had one of these. grade_stair_champion.sh exists; the G1 WALK champion
# (wr_decent_walker.pt) has NEVER been put on the legitimacy ruler -- and it runs at λ=0.9 with
# HARNESS_KZ=2000, so it would fail L1 catastrophically. This is that missing exam.
#
# DEFAULT λ=0: no harness at all. That is the whole point -- the robot either carries itself or it
# does not. verify_motion_legitimacy then reads:
#   L0 upright        -- did it end on its feet?
#   L1 crane support  -- with λ=0 the wrench is identically zero, asserted tick by tick (the deploy
#                        hook emits a zero CRANELOG so "no crane" is a RECORDED FACT, not an absence
#                        of evidence -- without that the verifier scores an unassisted robot as FAIL)
#   L2 contact purity -- only feet touch the world (no knee/hand/pelvis levering)
#   L3 feet support   -- the feet stay under load; no long airborne gaps
# Pass a nonzero λ to ask the different question "when a crane IS available, does it lean on it?"
#
# The deploy env MUST mirror the training env key-for-key (ghost, GHOST_FF, EST_HEAD, POLICY_HEAD,
# corridors, Z_TGT, seed pose) or the net shape and the corridor centre will not match the
# checkpoint. Keep this in sync with run_freestand_rl.sh.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"

CKPT="${1:?usage: grade_walk_champion.sh <ckpt.pt> [lam] [dur_s]}"
LAM="${2:-0}"; DUR="${3:-150}"
TAG="gradewalk"
MPC="_scratch/foot_redesign/${TAG}_mpc.txt"

GHOST="$ROOT/projects/policies/ghosts/g1/ghost_walk_synth_lut.json"
SEED="-0.655,-0.159,0.172,1.100,-0.459,0.234,-0.714,-0.148,0.180,0.356,0.345,0.233"

bash projects/policies/training/run_walk_rl.sh "$DUR" "$TAG" deploy headless \
  PYTHONUNBUFFERED=1 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=projects/policies/worlds/g1_walk_orig.omniworld \
  WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.70 "STAND_POSE_LEGS=$SEED" \
  WALK_WARM_TICKS=30 WALK_MJW_RESET=0 \
  POLICY_ARCH=lstm POLICY_LAYERS=1 PPO_HID=256 \
  "POLICY_HEAD=${POLICY_HEAD:-mlp}" "EST_HEAD=${EST_HEAD:-1}" \
  REF_OBS=1 REF_OBS_K=3 REF_OBS_STRIDE=4 \
  "GHOST_LUT_JSON=$GHOST" "G1_GHOST_LUT=$GHOST" "GHOST_METRIC_JSON=$GHOST" \
  GHOST_FF=1 "GHOST_RESIDUAL=${GHOST_RESIDUAL:-0.12}" \
  VX_MAX=0.131 Z_TGT=0.68 HEADING_TARGET=0 \
  HARNESS_LAM0="$LAM" HARNESS_KP=600 HARNESS_KD=60 HARNESS_FY=400 \
  HARNESS_KZ=2000 HARNESS_DZ=150 HARNESS_Z0=0.68 HARNESS_KYAW=0 \
  FOOT_LOG=1 FOOT_LOG_EVERY=6 CONTACT_LOG=1 CONTACT_LOG_ALL=1 CONTACT_LOG_EVERY=4 \
  CRANE_LOG=1 CRANE_LOG_EVERY=4 \
  "RES_POLICY=$CKPT" >/dev/null 2>&1

echo "=========== FREE-STANDING EXAM (lam=$LAM, ${DUR}s) : $(basename "$CKPT") ==========="
python - "$MPC" <<'PY'
import re, sys
log = sys.argv[1]
bx = bz = None; t = 0
try:
    for ln in open(log, encoding="utf-8", errors="replace"):
        m = re.search(r"FOOTLOG t=(\d+).*bz=([-\d.]+)", ln)
        if m:
            t = int(m.group(1)); bz = float(m.group(2))
        m = re.search(r"CRANELOG t=(\d+) .*bx=([-\d.]+) bz=([-\d.]+)", ln)
        if m:
            t = int(m.group(1)); bx = float(m.group(2)); bz = float(m.group(3))
except FileNotFoundError:
    print("no deploy log -- the run did not produce %s" % log); sys.exit(2)
print("DURABILITY: last tick t=%d   base x=%s m   base z=%s m   %s"
      % (t, "%.2f" % bx if bx is not None else "?", "%.3f" % bz if bz is not None else "?",
         "UPRIGHT" if (bz or 0) > 0.5 else "FALLEN"))
PY
# L0-L3. x-window 0.5..15 m: skip the launch settle, cover the run.
python projects/policies/training/verify_motion_legitimacy.py "$MPC" 0.5 15.0 6,7,12,13
RC=$?
echo "-----------------------------------------------------------------------"
[ "$RC" = 0 ] && echo "VERDICT: LEGITIMACY PASS at lam=$LAM" || echo "VERDICT: LEGITIMACY FAIL at lam=$LAM"
exit $RC
