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

# IN-ENGINE quad WALK trainer launcher (go2 | omniquad | b2) -- trains the
# residual-on-trot walk THROUGH omnisim-bin (train == deploy bit-exact, zero
# MJCF-reparse gap), via projects/policies/training/quad_walk_recipe.py.
# NEW / additive-only: the standalone gpu_mjwarp_* trainers are untouched (the
# recipe SUBCLASSES their env and swaps only the model source).
#
#   bash run_quad_walk_rl.sh <dur> <tag> <mode:train> [gui|headless] [ENV=val ...]
#   QUAD_ROBOT=go2|omniquad|b2   (default go2)
#
# Deploy with each robot's existing <robot>_walk_deploy controller + the ONNX this
# writes; benchmark with scripts/dev/run_quad_rough_track.ps1 -Robot <r>.
# Warm-start the flat champion, e.g.:
#   QUAD_WARMSTART=$ROOT/projects/policies/research/inference/policies/gpu_go2_walk_main/warmstart.pt
#
# THROUGHPUT knobs (measured 2026-07-12, RunPod 4090, Go2 flat; baseline 132k
# env-steps/s @ K=4096 -> 784k @ K=16384 with all three; quality-gated):
#   QUAD_FAST_RESET=1   reset-path mjw.forward -> mjw.kinematics. THE big one:
#                       the full-pipeline forward (collision+EPA scratch sized
#                       nconmax*nworld) ran on ALL K worlds on every rollout
#                       step with >=1 fall -- 66-81% of iteration time, and the
#                       1.4 GB EPA alloc was the K=16384 OOM. Micro numerics
#                       change (stale solver warmstart on the reset step), so
#                       default OFF.
#   MPC_NJMAX=64 MPC_NCONMAX=64   slim per-world constraint/contact caps for
#                       the engine rollout buffers (engine default 256/256;
#                       Go2 flat uses ~10 contacts/world). Also required to
#                       fit K=16384 without FAST_RESET. Raise for terrain.
#   QUAD_PROFILE=1      per-phase timing breakdown every 5 iters (sync fences:
#                       absolute fps lower; the value is the split).
#   QUAD_FAST_UPDATE=1 [+QUAD_MINIBATCH=n, QUAD_BF16=1]  faster PPO update
#                       (fused Adam, manual gaussian math, optional minibatch/
#                       bf16). NOT needed on a 4090 (update = ~3% of iter);
#                       default OFF.
# FOLDABILITY knobs (SHADOWING mode, i.e. QUAD_GHOST set):
#   QUAD_W_SATHINGE=<w> (2026-07-17, default 0 = off)  hinge penalty on the
#                       torque-saturation PEAK: prices only the part of
#                       |kp*(cmd-q)|/tau_lim ABOVE QUAD_SATHINGE_FRAC,
#                       quadratically; identically zero below it. Targets PEAK
#                       saturation for ghost-foldability; W_TAU targets mean
#                       effort and does NOT restore foldability (9dbe57c7: OmniQuad's
#                       legacy champion saturates 28% of steps vs the Go2's 3% ->
#                       its fold FAILS closure p95 81.1mm vs the <20mm gate).
#                       kp + tau_lim are read from the LIVE compiled model
#                       (gainprm / jnt_actfrcrange), never re-declared. sat_frac
#                       (fraction of (step,joint) samples above FRAC) is logged
#                       every 5 iters and at eval EVEN when the penalty is off --
#                       the campaign's foldability-progress readout.
#   QUAD_SATHINGE_FRAC=<f> (default 0.85)  the hinge threshold, as a fraction of
#                       each joint's actuator torque limit.
#   QUAD_W_FOOTSLIP=<w> (2026-07-17, default 0 = off)  THE CLOSURE-WALL term:
#                       penalty = mean over the 4 feet of (in_contact * |v_xy|^2),
#                       i.e. squared horizontal foot speed WHILE PLANTED, pushing the
#                       policy toward gaits whose stance feet stay put -> lower FOLDED
#                       closure -> foldable. This is the r2->r3 wall: d6846147 PROVED
#                       it is foot-slip/CLOSURE (fold p95 28.3mm vs the <20mm gate),
#                       NOT saturation (W_SATHINGE cut saturation ~4x with closure
#                       unmoved). Feet come from the ENGINE-derived calf bodies
#                       (robot-correct for go2|omniquad|b2, no hardcoded indices); stance =
#                       foot z below QUAD_FOOTSLIP_VZ (the same z-notion the closure
#                       gate uses). Mean stance-slip SPEED (mm/s) is logged every 5
#                       iters and at eval EVEN when off -- the closure-precursor readout.
#   QUAD_FOOTSLIP_VZ=<m> (default 0.02)  foot world-z (ground-relative on terrain)
#                       below which a foot counts as planted / in-contact.
# TURN knob (SHADOWING mode):
#   QUAD_WZ_FIXED=<rad/s> (2026-07-17, TRAIN_PLAN.md §4, default unset = off)  pin the
#                       wz command (obs slot 48 + the r_yaw target) to a constant so
#                       r_yaw = QUAD_YAW*|wz - wz_cmd| becomes a yaw-TRACKING reward --
#                       the go2_turn upgrade path when the champion under-rotates. Train
#                       e.g. QUAD_WZ_FIXED=0.5895 QUAD_YAW=-1.0; deploy then needs the
#                       matching wz-obs hook in go2_baton_deploy.py (another workstream).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."           # repo root
ROOT="$(pwd -W 2>/dev/null || pwd)"                     # Windows-style (D:/...) for the exe
DUR="${1:-600}"; TAG="${2:-quadwalk}"; MODE="${3:-train}"; GUI="${4:-headless}"; shift 4 2>/dev/null || shift $# 2>/dev/null || true
export OMNISIM_HOME=$ROOT PYTHONUTF8=1
# The embedded newton-runtime python (3.14) reaches the system torch (same ABI,
# warp version pin-matched by tests/test_newton_pins_parity.py) -- REQUIRED for
# in-engine RL training (torch PPO). Derive it BEFORE we prepend msys to PATH.
# Pass it as OMNISIM_TORCH_SITE only. Do NOT put it on PYTHONPATH: the engine's
# embedded interpreter would then import the SYSTEM warp/newton during Newton
# backend init, breaking the FFI smoke (REQUIRE_NEWTON -> FATAL). The pymod adds
# it to sys.path AFTER Newton is up (quad_walk_recipe._ensure_torch_path).
# Respect a pre-set OMNISIM_TORCH_SITE. Otherwise derive it from the
# interpreter the ENGINE BINARY LINKS (ldd), not from `python` on PATH --
# on dual-python cloud images those differ, and handing the embedded 3.10 a
# 3.11 torch dies with "Failed to load PyTorch C extensions" (ABI mismatch).
if [ -z "${OMNISIM_TORCH_SITE:-}" ]; then
  _TORCH_PY=python
  if [ "$(uname -s 2>/dev/null)" != "Windows_NT" ] && [ -x "$ROOT/bin/omnisim-bin" ]; then
    _PYLIB=$(ldd "$ROOT/bin/omnisim-bin" 2>/dev/null | grep -oE 'libpython3\.[0-9]+' | head -1 || true)
    [ -n "$_PYLIB" ] && command -v "python${_PYLIB#libpython}" >/dev/null 2>&1 && _TORCH_PY="python${_PYLIB#libpython}"
  fi
  _TORCH_SP="$($_TORCH_PY -c 'import torch,os;print(os.path.dirname(os.path.dirname(torch.__file__)))' 2>/dev/null || true)"
  [ -n "$_TORCH_SP" ] && export OMNISIM_TORCH_SITE="$_TORCH_SP"
fi
export WEBOTS_EXTRA_PROJECT_PATH="$ROOT/projects/policies"
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=8 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_REQUIRE_NEWTON=1 OMNISIM_REQUIRE_MUJOCO_SOLVER=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE=1 OMNISIM_NEWTON_SEED_POSE=1
export OMNISIM_NEWTON_GROUND_MU=2.0
export OMNISIM_LOG_PATH="$ROOT/_scratch/foot_redesign/${TAG}_omnisim.txt"
export RES_LOG="$ROOT/_scratch/foot_redesign/${TAG}_rl.txt"
export OMNISIM_INENGINE_MPC_LOG="$ROOT/_scratch/foot_redesign/${TAG}_mpc.txt"   # captures engine-side pymod errors
export OMNISIM_INENGINE_PYMOD="projects.policies.training.quad_walk_recipe:quad_walk_recipe_step"
export RES_MODE="$MODE"
export PATH="$ROOT/msys64/mingw64/bin:$PATH"
mkdir -p "$ROOT/projects/policies/training/runs" "$ROOT/_scratch/foot_redesign"
for kv in "$@"; do export "$kv"; done                   # QUAD_ROBOT / QUAD_* / RES_POLICY / ...

# ── per-robot deploy-matched stiffness + host world (train == deploy) ──────
# GROUND TRUTH = the actuator gains baked into the MJCF each champion was trained
# on (projects/policies/research/training/mjcf/<robot>_newton*.xml), verified by
# reading gainprm/biasprm. run_quad_rough_track.ps1 agrees on all three.
# ⛔ go2_walk_deploy.py and b2_walk_deploy.py BOTH document "KE=80 KD=2.0
# (MATCHED)" -- that is WRONG for both (b2's is copy-pasted from go2, and go2's is
# wrong too). Do not trust those docstrings; they cost a mis-trained run.
ROBOT="${QUAD_ROBOT:-go2}"; export QUAD_ROBOT="$ROBOT"
case "$ROBOT" in
  go2)  DEF_KE=250;  DEF_KD=6   ;;   # go2_newton.xml           kp=250 kv=6
  omniquad) DEF_KE=500;  DEF_KD=60  ;;   # omniquad_newton_fixed2.xml   kp=500 kv=60
  b2)   DEF_KE=1400; DEF_KD=35  ;;   # b2_newton.xml            kp=1400 kv=35
  *) echo "ERROR: QUAD_ROBOT must be go2|omniquad|b2 (got '$ROBOT')" >&2; exit 2 ;;
esac
export OMNISIM_NEWTON_TARGET_KE="${OMNISIM_NEWTON_TARGET_KE:-$DEF_KE}"
export OMNISIM_NEWTON_TARGET_KD="${OMNISIM_NEWTON_TARGET_KD:-$DEF_KD}"
export RES_POLICY="${RES_POLICY:-$ROOT/projects/policies/training/runs/gpu_${ROBOT}_inengine/policy.pt}"
# DUAL-READ (AGENTS.md): the tree writes .omniworld and still reads .wbt, so
# resolve whichever is on disk. This line said ".wbt" outright and the extension
# migration renamed the file underneath it, so on a migrated tree the quadruped
# trainer died at "World not found: ..._walk_deploy.wbt" before a single step --
# taking OmniBench lane 2's tier C (the in-engine PPO row the README quotes)
# with it. The g1 runners were migrated; this one was missed.
WORLD="${WALK_WORLD:-}"
if [ -z "$WORLD" ]; then
  for _ext in omniworld wbt; do
    _cand="projects/policies/research/worlds/${ROBOT}_walk_deploy.${_ext}"
    if [ -f "$ROOT/$_cand" ] || [ -f "$_cand" ]; then WORLD="$_cand"; break; fi
  done
  WORLD="${WORLD:-projects/policies/research/worlds/${ROBOT}_walk_deploy.omniworld}"
fi

rm -f "$RES_LOG" "$RES_LOG.status" "$OMNISIM_INENGINE_MPC_LOG"
echo ">>> $TAG robot=$ROBOT mode=$MODE gui=$GUI dur=${DUR}s policy=$RES_POLICY world=$WORLD"
echo "    KE=$OMNISIM_NEWTON_TARGET_KE KD=$OMNISIM_NEWTON_TARGET_KD substeps=$OMNISIM_NEWTON_SUBSTEPS  $(for kv in "$@"; do printf '%s ' "$kv"; done)"
if [ "$GUI" = "gui" ]; then
  python -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR" &
  echo "GUI launched pid $!. RL log -> $RES_LOG"
else
  python -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" \
      > "$ROOT/_scratch/foot_redesign/${TAG}_console.txt" 2>&1 &
  RPID=$!
  # WATCHDOG (== run_walk_rl.sh): the in-engine trainer cannot stop the sim
  # itself, so it writes <RES_LOG>.status {"state":"DONE"} when finished.
  T0=$(date +%s); RC=0
  while kill -0 $RPID 2>/dev/null; do
    sleep 20
    NOW=$(date +%s)
    WINPID=$(cat /proc/$RPID/winpid 2>/dev/null || echo "")
    if [ -f "$RES_LOG.status" ] && grep -q '"state": "DONE"' "$RES_LOG.status" 2>/dev/null; then
      echo ">>> trainer reported DONE -- stopping the simulator (idle wall time saved)"
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      [ -z "$WINPID" ] && { kill -TERM -- -$RPID 2>/dev/null || kill -TERM $RPID 2>/dev/null; }
      break
    fi
    if [ -f "$RES_LOG" ]; then
      AGE=$(( NOW - $(stat -c %Y "$RES_LOG" 2>/dev/null || echo "$NOW") ))
      if [ "$AGE" -gt "${STALL_S:-1800}" ]; then
        echo "ERROR: STALL -- $RES_LOG silent for ${AGE}s (> STALL_S=${STALL_S:-1800}); killing the sim tree" >&2
        [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
        [ -z "$WINPID" ] && { kill -TERM -- -$RPID 2>/dev/null || kill -TERM $RPID 2>/dev/null; }
        RC=3; break
      fi
    elif [ $(( NOW - T0 )) -gt "${STARTUP_S:-900}" ]; then
      echo "ERROR: STARTUP STALL -- no $RES_LOG after $(( NOW - T0 ))s; killing the sim tree" >&2
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      [ -z "$WINPID" ] && { kill -TERM -- -$RPID 2>/dev/null || kill -TERM $RPID 2>/dev/null; }
      RC=3; break
    fi
  done
  wait $RPID 2>/dev/null
  echo "=== ${TAG} RL log (last 20) ==="
  tail -20 "$RES_LOG" 2>/dev/null
  exit $RC
fi
