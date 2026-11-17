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

# In-engine residual-RL WALK: train/deploy/probe the residual on the deterministic walk,
# INSIDE OmniSim's Newton engine (zero sim-to-deploy gap). Bigfoot G1 world.
#   bash run_walk_rl.sh <dur> <tag> <mode:probe|train|deploy> [gui] [ENV=val ...]
#
# !! TRAINER SELECTION -- the #1 silent-failure trap of this launcher (2026-07-04):
#   The DEFAULT OMNISIM_INENGINE_PYMOD below is the LEGACY ES residual trainer
#   (g1_walk_residual_inengine, pop=16, 144 params). The flagship Shadowing/recipe
#   trainer (ghosts, corridors, GHOST_SEQ sequences, WBMATCH, GPU PPO) is a DIFFERENT
#   module and must be selected explicitly on the command line:
#     OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step
#   If you pass recipe-only envs (GHOST_LUT_JSON, GHOST_SEQ, GHOST_RESIDUAL, ...) without
#   that override, the legacy trainer runs and SILENTLY IGNORES them -- the robot
#   typically falls within seconds and a 144-parameter ES grinds on the corpse.
#   Tell-tale in <tag>_rl.txt: "TRAIN start pop=16 ... N_PARAM=144" instead of the
#   "GHOST-SEQ:" / "WALK-GPU it=" banners. The guard below now fails fast on that
#   mismatch (bypass with ALLOW_MISMATCHED_PYMOD=1 if you really mean it).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."          # repo root, wherever the clone lives
ROOT="$(pwd -W 2>/dev/null || pwd)"                    # Windows-style (D:/...) for the engine exe
DUR="${1:-60}"; TAG="${2:-walkrl}"; MODE="${3:-probe}"; GUI="${4:-headless}"; shift 4 2>/dev/null || shift $# 2>/dev/null || true
export OMNISIM_HOME=$ROOT PYTHONUTF8=1
export WEBOTS_EXTRA_PROJECT_PATH="$ROOT/projects/policies"
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1 OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
export OMNISIM_NEWTON_TARGET_KE=200 OMNISIM_NEWTON_TARGET_KD=30 OMNISIM_NEWTON_GROUND_MU=2.0   # walk stiffness
export HUMANOID_STAND_SPEC="$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json"
export HSTAND_ANK_BIAS=-0.06 HSTAND_LEAN=0          # the HOOK does lean+gait+residual; controller = pure pose
export HUMANOID_STAND_LOG="$ROOT/_scratch/foot_redesign/${TAG}.log"
export OMNISIM_DEPLOY_LOG="$HUMANOID_STAND_LOG"
export OMNISIM_LOG_PATH="$ROOT/_scratch/foot_redesign/${TAG}_omnisim.txt"
export OMNISIM_INENGINE_MPC_LOG="$ROOT/_scratch/foot_redesign/${TAG}_mpc.txt"
export RES_LOG="$ROOT/_scratch/foot_redesign/${TAG}_rl.txt"
export HSTAND_WARMUP_RELOAD=1
export OMNISIM_WARMUP_TOKEN="walkrl_$$_${RANDOM}"
export OMNISIM_INENGINE_PYMOD="projects.policies.training.g1_walk_residual_inengine:g1_walk_res_step"
export RES_MODE="$MODE"
export RES_POLICY="${RES_POLICY:-$ROOT/projects/policies/training/runs/g1_walk_res.npz}"
export PATH="$ROOT/msys64/mingw64/bin:$PATH"
mkdir -p "$ROOT/projects/policies/training/runs"
for kv in "$@"; do export "$kv"; done
# A constructed carry ghost is self-describing.  When payload adaptation is launched from an
# older campaign that supplies only CARRY_PAYLOAD_KG, do not silently train the payload at the
# nominal hanging-arm pose: activate the same whole-body carry corridors used by the physical
# suction demo. Explicit caller values still win.
if [ -n "${GHOST_LUT_JSON:-}" ] && [ -f "$GHOST_LUT_JSON" ] && \
   grep -q '"carry"[[:space:]]*:[[:space:]]*true' "$GHOST_LUT_JSON"; then
  export W_ARMGHOST="${W_ARMGHOST:-4}" W_WB="${W_WB:-6}" ARM_RESIDUAL="${ARM_RESIDUAL:-0.15}"
  export ELBOW_TARGET="${ELBOW_TARGET:-0}" ELBOW_RESIDUAL="${ELBOW_RESIDUAL:-0.12}"
  export SHRY_TARGET="${SHRY_TARGET:-0.15}" SHRY_YAW_TARGET="${SHRY_YAW_TARGET:-0}"
  export SHRY_RESIDUAL="${SHRY_RESIDUAL:-0.10}"
fi
export WARP_CACHE_PATH="${WARP_CACHE_PATH:-$ROOT/_scratch/warp_cache/$TAG}"
mkdir -p "$WARP_CACHE_PATH"
# -- fail-fast guard: recipe-only envs + non-recipe trainer = silent no-op (see header) --
if [ "${ALLOW_MISMATCHED_PYMOD:-0}" != "1" ] && [[ "$OMNISIM_INENGINE_PYMOD" != *jobserver* ]]; then
  case "$OMNISIM_INENGINE_PYMOD" in
    *g1_walk_recipe*) : ;;
    *)
      for _renv in GHOST_LUT_JSON GHOST_SEQ GHOST_RESIDUAL GHOST_MORPH_JSON GHOST_METRIC_JSON \
                   ARM_RESIDUAL W_ATTGHOST SEQ_EVAL_MAP WHOLE_BODY PPO_ITERS; do
        if [ -n "$(eval echo "\${$_renv:-}")" ]; then
          echo "ERROR: $_renv is set but OMNISIM_INENGINE_PYMOD=$OMNISIM_INENGINE_PYMOD" >&2
          echo "       Recipe/Shadowing envs are ONLY read by the recipe trainer. Add:" >&2
          echo "       OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step" >&2
          echo "       (or set ALLOW_MISMATCHED_PYMOD=1 to bypass this guard.)" >&2
          exit 2
        fi
      done ;;
  esac
fi
WORLD="${WALK_WORLD:-projects/policies/worlds/g1_walk_bigfoot.omniworld}"   # override with WALK_WORLD=... to swap the model
rm -f "$RES_LOG" "$OMNISIM_INENGINE_MPC_LOG"
echo ">>> $TAG mode=$MODE gui=$GUI dur=${DUR}s policy=$RES_POLICY  $(for kv in "$@"; do printf '%s ' "$kv"; done)"
# REQUIRE_NEWTON on the DEPLOY lane (2026-07-12): every deploy policy reads the
# mujoco_warp state (world.solver.mjw_data), so a silent ODE fallback does not fail --
# it comes up with NO policy at all and the robot just stands there looking broken (a
# Newton FFI-smoke flake did exactly this to a live demo). Assert the backend: the
# engine now exits non-zero instead of degrading. Training keeps the default (it dies
# on its own if mjw is missing, and a hard gate there would break ODE experiments).
RQN=""; [ "$MODE" = "deploy" ] && RQN="--require-newton"
RPORT=""; [ -n "${OMNISIM_PORT:-}" ] && RPORT="--port ${OMNISIM_PORT}"
RDONE=()
if [ -n "${OMNISIM_DONE_PATTERN:-}" ]; then
  RDONE=(--completion-log "${OMNISIM_DONE_LOG:-$OMNISIM_INENGINE_MPC_LOG}"
         --completion-pattern "$OMNISIM_DONE_PATTERN"
         --completion-grace "${OMNISIM_DONE_GRACE:-2}")
fi
# Interpreter resolution (2026-07-17): a bash spawned from a Windows process
# (skill_lib subprocess, machine_conformance, CI) rebuilds PATH with the MSYS
# dirs first, so bare `python` resolves to nothing (or the MSYS python3 -- the
# wrong interpreter for the runner). Callers pass OMNISIM_PYTHON to be exact;
# interactively `python` keeps working as before.
# The runner and the simulator do not have to share an executable.  On Windows the
# bundled Newton interpreter is the correct embedded/controller runtime, but using
# that minimal distribution to spawn/own omnisim-bin from Git Bash can report the
# launcher's early exit while the native child is still starting.  Keep the engine's
# OMNISIM_PYTHON untouched and allow a full host Python to own headless_runner.
PY="${OMNISIM_RUNNER_PYTHON:-${OMNISIM_PYTHON:-python}}"
if ! command -v "$PY" >/dev/null 2>&1; then PY=python3; fi
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "ERROR: no controller/runner Python found (set OMNISIM_PYTHON to python.exe)" >&2
  exit 2
fi
case "$PY" in
  */*) export PATH="${PY%/*}:$PATH" ;;
esac
# -- DEPLOY HOOK HEALTH (2026-08-24) -------------------------------------------
# The in-engine hook CATCHES pymod exceptions and writes them ONLY to the MPC log.
# So a hook that throws on EVERY tick is invisible to every other signal: the engine
# log still reads "0 errors", --require-newton still passes (Newton DID drive the
# world -- it just received no targets), and the launcher still prints PASS with
# exit 0 on a robot lying face-up on the floor.
#
# That is exactly how newton 1.5's removal of `Control.joint_target_pos` killed the
# G1 flagship demo for 10 days (found 2026-08-24): 2210 throws, 100% of ticks, not
# one joint target ever applied, and the ONLY evidence anywhere was this log.
# Assert the hook actually RAN -- not merely that Newton finalised a world.
verify_deploy_hook() {
  why="$1"; nerr=$(grep -c 'pymod error' "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null || true)
  [ -z "$nerr" ] && nerr=0
  if [ "$nerr" -gt 0 ]; then
    echo "ERROR: ${why} the in-engine hook threw on ${nerr} tick(s) -- the policy applied NO" >&2
    echo "       joint targets, so the robot was never actually driven. First error:" >&2
    grep -m1 'pymod error' "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null | sed 's/^/         /' >&2
    return 4
  fi
  if ! grep -q 'WALK-RECIPE DEPLOY ready' "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null; then
    echo "ERROR: ${why} controller never reported ready (checkpoint/setup unproven)" >&2; return 4
  fi
  if ! grep -q 'walk-recipe deploy t=' "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null; then
    echo "ERROR: ${why} controller produced no policy ticks" >&2; return 4
  fi
  return 0
}

if [ "$GUI" = "gui" ]; then
  "$PY" -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR" \
      "${RDONE[@]}" $RQN $RPORT &
  GPID=$!
  echo "GUI launched pid $GPID. RL log -> $RES_LOG"
  # WAIT, and propagate (2026-07-16): this branch used to background the runner and fall
  # off the end of the script, so $? was the echo's -- a GUI demo whose engine died at
  # cold load printed "FAIL: simulator exited early with code 3221225477" and STILL exited
  # 0. That is the same fault bea4417b fixed on the go2 launcher: a launcher must not
  # report a run that never ran. The run is bounded by --duration, exactly like the
  # headless branch below, so waiting on it costs nothing and makes the code honest.
  wait $GPID; RC=$?
  # The GUI branch used to run NO deploy verification at all -- that is why the
  # broken flagship demo printed PASS while the G1 lay on the floor. Same guard.
  if [ "$MODE" = "deploy" ] && [ "$RC" -eq 0 ]; then
    verify_deploy_hook "GUI deploy:" || RC=$?
  fi
  if [ "$MODE" = "deploy" ] && [ "$RC" -eq 0 ] && [ -n "${OMNISIM_SUCCESS_PATTERN:-}" ] && \
     ! grep -Eq "$OMNISIM_SUCCESS_PATTERN" "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null; then
    echo "ERROR: GUI deploy did not emit required success verdict: $OMNISIM_SUCCESS_PATTERN" >&2
    RC=4
  fi
  [ "$RC" -ne 0 ] && echo "ERROR: the GUI run FAILED (rc=$RC) -- the demo did NOT complete" >&2
  exit $RC
else
  "$PY" -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" \
      --wait-for-step --startup-timeout "${STARTUP_S:-900}" $RQN \
      "${RDONE[@]}" $RPORT \
      > "$ROOT/_scratch/foot_redesign/${TAG}_console.txt" 2>&1 &
  RPID=$!
  # If the campaign shell is interrupted, reap the runner instead of leaving a detached
  # omnisim-bin training against (and writing to) the same checkpoint as the retry.  A RunPod
  # correction exposed this: killing only the wrapper orphaned the engine under PID 1, and two
  # trainers silently competed for one .pt file.  headless_runner owns cleanup of its child when
  # it receives TERM, so make that ownership survive wrapper interruption.
  cleanup_runner() {
    # On Linux the simulator is a child of headless_runner.  TERMing only the runner does not
    # currently reap that child, so kill descendants first; on Windows task ownership is handled
    # by the runner/job object and pkill may be absent.
    if command -v pkill >/dev/null 2>&1; then
      pkill -TERM -P "$RPID" 2>/dev/null || true
    fi
    if kill -0 "$RPID" 2>/dev/null; then
      kill -TERM "$RPID" 2>/dev/null || true
      wait "$RPID" 2>/dev/null || true
    fi
  }
  terminate_runner() {
    trap - EXIT INT TERM
    cleanup_runner
    exit 143
  }
  trap cleanup_runner EXIT
  trap terminate_runner INT TERM
  # ── WATCHDOG (structural fix, 2026-07-04): the in-engine trainer cannot stop the
  # simulator itself -- a finished trainer used to IDLE until the wall budget expired,
  # and a stalled sim looked identical to a running one. The recipe trainer writes
  # <RES_LOG>.status ({"state": "TRAINING"|"DONE", ...}) every log interval:
  #   state DONE                -> stop the sim tree NOW (no idle tail), exit 0
  #   RES_LOG silent > STALL_S  -> kill the sim tree, exit 3 LOUDLY (default 900 s)
  #   no RES_LOG > STARTUP_S    -> startup stall, kill, exit 3 (default 900 s)
  T0=$(date +%s); RC=0; TRAINER_DONE=0
  while kill -0 $RPID 2>/dev/null; do
    sleep 20
    NOW=$(date +%s)
    WINPID=$(cat /proc/$RPID/winpid 2>/dev/null || echo "")
    if [ -f "$RES_LOG.status" ] && grep -q '"state": "ERROR"' "$RES_LOG.status" 2>/dev/null; then
      echo "ERROR: in-engine trainer reported ERROR -- stopping the simulator" >&2
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      [ -z "$WINPID" ] && { kill -TERM -- -$RPID 2>/dev/null || kill -TERM $RPID 2>/dev/null; }
      RC=5; break
    fi
    if [ -f "$RES_LOG.status" ] && grep -q '"state": "DONE"' "$RES_LOG.status" 2>/dev/null; then
      echo ">>> trainer reported DONE -- stopping the simulator (idle wall time saved)"
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      [ -z "$WINPID" ] && { kill -TERM -- -$RPID 2>/dev/null || kill -TERM $RPID 2>/dev/null; }
      TRAINER_DONE=1
      break
    fi
    if [ -f "$RES_LOG" ]; then
      AGE=$(( NOW - $(stat -c %Y "$RES_LOG" 2>/dev/null || echo "$NOW") ))
      if [ "$AGE" -gt "${STALL_S:-900}" ]; then
        echo "ERROR: STALL -- $RES_LOG silent for ${AGE}s (> STALL_S=${STALL_S:-900}); killing the sim tree" >&2
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
  if wait $RPID 2>/dev/null; then PRC=0; else PRC=$?; fi
  trap - EXIT INT TERM
  if [ "$RC" -eq 0 ] && [ "$TRAINER_DONE" -eq 0 ] && [ "$PRC" -ne 0 ]; then
    RC=$PRC
  fi
  if [ "$MODE" = "deploy" ] && [ "$RC" -eq 0 ]; then
    SIDECAR="$OMNISIM_LOG_PATH.newton.json"
    if ! grep -Eq '"finalised"[[:space:]]*:[[:space:]]*true' "$SIDECAR" 2>/dev/null; then
      echo "ERROR: deploy has no finalised Newton sidecar: $SIDECAR" >&2; RC=4
    elif ! verify_deploy_hook "deploy:"; then
      RC=4
    elif [ -n "${OMNISIM_SUCCESS_PATTERN:-}" ] && \
         ! grep -Eq "$OMNISIM_SUCCESS_PATTERN" "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null; then
      echo "ERROR: deploy did not emit required success verdict: $OMNISIM_SUCCESS_PATTERN" >&2; RC=4
    fi
  fi
  echo "=== ${TAG} RL log (last 16) ==="
  grep -E 'res:|GEN |walk ready|captured spawn|TRAIN|DEPLOY' "$RES_LOG" "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null | sed 's/np.int32(\([0-9]*\))/\1/g' | tail -16
  exit $RC
fi
