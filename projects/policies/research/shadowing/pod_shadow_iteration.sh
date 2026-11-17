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

# ⭐ SHADOW ITERATION -- is Shadowing a FIXED-POINT IMPROVEMENT OPERATOR?
#
# THE QUESTION. Round 1 is already in the repo and verified: take the legacy Go2 champion, roll it
# deterministically, phase-fold its ACHIEVED gait at 64 bins, harmonic-smooth it -> that is the
# ghost -> shadow-train against it (corridor + GHOST-FF, warm-started from the legacy champion) ->
# +12.6% speed, 5x straighter, 0 falls. The fold-and-smooth is a DENOISING PROJECTION onto the
# clean-periodic-gait manifold, and the corridor holds the new policy near it while PPO re-optimises
# the task reward. Nobody has asked the obvious next question: DOES IT COMPOUND?
#
#   ghost_{n+1} = fold(roll(champion_n))          champion_{n+1} = shadow_train(ghost_{n+1})
#
# If that iteration keeps improving, Shadowing is not "imitate a reference" -- it is a
# SELF-IMPROVEMENT OPERATOR on gaits, and the reference is a fixed point it converges to.
#
# ⛔ THE ABLATION IS THE WHOLE EXPERIMENT. A round-2 champion trained for 400 more iterations will
# beat round 1 for the trivial reason that IT TRAINED LONGER. So we run TWO arms that are identical
# in every respect -- same warm start, same iters, same envs, same seed, same corridor, same reward,
# SAME GPU MODEL -- and differ in EXACTLY ONE THING:
#
#   treat : QUAD_GHOST = ghost v2  (rebuilt from the shadow champion's OWN achieved gait)
#   ctrl  : QUAD_GHOST = ghost v1  (the old ghost -- i.e. purely "train 400 iters longer")
#
# Whatever separates them is attributable to the GHOST REFRESH and to nothing else.
#
# Usage (on a pod):  bash pod_shadow_iteration.sh treat|ctrl
set -uo pipefail
cd /workspace/omnisim
ARM="${1:?usage: pod_shadow_iteration.sh treat|ctrl [seed]}"
SEED="${2:-0}"

export OMNISIM_HOME=/workspace/omnisim WEBOTS_HOME=/workspace/omnisim
export LD_LIBRARY_PATH=/workspace/omnisim/lib/webots
export QT_QPA_PLATFORM=xcb WEBOTS_TMPDIR=/tmp LIBGL_ALWAYS_SOFTWARE=1
export OMNISIM_REQUIRE_NEWTON=1
export OMNISIM_LOG_PATH=/workspace/omnisim_${ARM}_s${SEED}.txt

GHOST_V1=/workspace/omnisim/projects/policies/ghosts/go2/go2_shadow_ghost_lut.json
GHOST_V2=/workspace/omnisim/_scratch/go2_shadow_ghost_v2_lut.json
CHAMP_V1_ONNX=/workspace/omnisim/projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx
CHAMP_V1_PT=/workspace/omnisim/projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.pt
OUT=/workspace/champ_${ARM}_s${SEED}
mkdir -p "$OUT" /workspace/omnisim/_scratch

# ── ROUND-2 GHOST: fold the SHADOW champion's own achieved gait ─────────────────────────────
# It must be rolled on the baseline IT RIDES (ghost v1 + its declared feedforward) -- rolling a
# shadow champion on the analytic trot would record it far out of distribution and the "achieved
# gait" would be junk. That is why the builder now takes --baseline/--baseline-lut.
if [ "$ARM" = "treat" ] && [ ! -f "$GHOST_V2" ]; then
  echo "=== [treat] building ghost v2 from the shadow champion's OWN achieved gait"
  python3 projects/policies/research/shadowing/build_go2_shadow_ghost.py \
      --policy "$CHAMP_V1_ONNX" \
      --baseline ghost --baseline-lut "$GHOST_V1" --corridor 0.15 \
      --T 12 --nb 64 --fold-from 3.0 \
      --out /workspace/omnisim/_scratch/go2_shadow_ghost_v2 2>&1 | tail -25
  [ -f "$GHOST_V2" ] || { echo "GHOST V2 BUILD FAILED"; exit 1; }
  echo "=== [treat] validating ghost v2 (fail-closed; an unvalidatable ghost is not a ghost)"
  python3 projects/policies/training/ghost_validator.py "$GHOST_V2" --stamp 2>&1 | tail -12
  GHOST="$GHOST_V2"
elif [ "$ARM" = "treat" ]; then
  echo "=== [treat] reusing ghost v2"; GHOST="$GHOST_V2"
else
  echo "=== [ctrl] SAME training, OLD ghost (this is the 'you just trained longer' control)"
  GHOST="$GHOST_V1"
fi

echo "=== [$ARM] training: warmstart=shadow-champion  ghost=$(basename "$GHOST")"
# every knob below is round 1's own launch_env (tag=shD), so the ONLY difference between the two
# arms is QUAD_GHOST.
xvfb-run -a bash projects/policies/training/run_quad_walk_rl.sh 2400 "shadow_${ARM}_s${SEED}" train headless \
  QUAD_ROBOT=go2 \
  QUAD_GHOST="$GHOST" \
  QUAD_GHOST_FF=1 \
  QUAD_W_GMATCH=2.0 \
  QUAD_RES_SCALE=0.15 \
  QUAD_ITERS=400 \
  QUAD_ENVS=16384 \
  QUAD_FAST_RESET=1 \
  MPC_NJMAX=64 MPC_NCONMAX=64 \
  QUAD_SEED="$SEED" \
  QUAD_WARMSTART="$CHAMP_V1_PT" \
  RES_POLICY="$OUT/policy.pt" 2>&1 | tail -30

echo "=== [$ARM] artifacts:"
ls -la "$OUT"
echo "=== [$ARM] DONE"
