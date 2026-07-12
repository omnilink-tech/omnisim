#!/usr/bin/env bash
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
WORLD="${WALK_WORLD:-projects/policies/worlds/g1_walk_bigfoot.wbt}"   # override with WALK_WORLD=... to swap the model
rm -f "$RES_LOG" "$OMNISIM_INENGINE_MPC_LOG"
echo ">>> $TAG mode=$MODE gui=$GUI dur=${DUR}s policy=$RES_POLICY  $(for kv in "$@"; do printf '%s ' "$kv"; done)"
if [ "$GUI" = "gui" ]; then
  python -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR" &
  echo "GUI launched pid $!. RL log -> $RES_LOG"
else
  python -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" \
      > "$ROOT/_scratch/foot_redesign/${TAG}_console.txt" 2>&1 &
  RPID=$!
  # ── WATCHDOG (structural fix, 2026-07-04): the in-engine trainer cannot stop the
  # simulator itself -- a finished trainer used to IDLE until the wall budget expired,
  # and a stalled sim looked identical to a running one. The recipe trainer writes
  # <RES_LOG>.status ({"state": "TRAINING"|"DONE", ...}) every log interval:
  #   state DONE                -> stop the sim tree NOW (no idle tail), exit 0
  #   RES_LOG silent > STALL_S  -> kill the sim tree, exit 3 LOUDLY (default 900 s)
  #   no RES_LOG > STARTUP_S    -> startup stall, kill, exit 3 (default 900 s)
  T0=$(date +%s); RC=0
  while kill -0 $RPID 2>/dev/null; do
    sleep 20
    NOW=$(date +%s)
    WINPID=$(cat /proc/$RPID/winpid 2>/dev/null || echo "")
    if [ -f "$RES_LOG.status" ] && grep -q '"state": "DONE"' "$RES_LOG.status" 2>/dev/null; then
      echo ">>> trainer reported DONE -- stopping the simulator (idle wall time saved)"
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      break
    fi
    if [ -f "$RES_LOG" ]; then
      AGE=$(( NOW - $(stat -c %Y "$RES_LOG" 2>/dev/null || echo "$NOW") ))
      if [ "$AGE" -gt "${STALL_S:-900}" ]; then
        echo "ERROR: STALL -- $RES_LOG silent for ${AGE}s (> STALL_S=${STALL_S:-900}); killing the sim tree" >&2
        [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
        RC=3; break
      fi
    elif [ $(( NOW - T0 )) -gt "${STARTUP_S:-900}" ]; then
      echo "ERROR: STARTUP STALL -- no $RES_LOG after $(( NOW - T0 ))s; killing the sim tree" >&2
      [ -n "$WINPID" ] && taskkill //PID "$WINPID" //T //F >/dev/null 2>&1
      RC=3; break
    fi
  done
  wait $RPID 2>/dev/null
  echo "=== ${TAG} RL log (last 16) ==="
  grep -E 'res:|GEN |walk ready|captured spawn|TRAIN|DEPLOY' "$RES_LOG" "$OMNISIM_INENGINE_MPC_LOG" 2>/dev/null | sed 's/np.int32(\([0-9]*\))/\1/g' | tail -16
  exit $RC
fi
