#!/usr/bin/env bash
# IN-ENGINE quad WALK trainer launcher (go2 | spot | b2) -- trains the
# residual-on-trot walk THROUGH omnisim-bin (train == deploy bit-exact, zero
# MJCF-reparse gap), via projects/policies/training/quad_walk_recipe.py.
# NEW / additive-only: the standalone gpu_mjwarp_* trainers are untouched (the
# recipe SUBCLASSES their env and swaps only the model source).
#
#   bash run_quad_walk_rl.sh <dur> <tag> <mode:train> [gui|headless] [ENV=val ...]
#   QUAD_ROBOT=go2|spot|b2   (default go2)
#
# Deploy with each robot's existing <robot>_walk_deploy controller + the ONNX this
# writes; benchmark with scripts/dev/run_quad_rough_track.ps1 -Robot <r>.
# Warm-start the flat champion, e.g.:
#   QUAD_WARMSTART=$ROOT/projects/policies/research/inference/policies/gpu_go2_walk_main/warmstart.pt
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
_TORCH_SP="$(python -c 'import torch,os;print(os.path.dirname(os.path.dirname(torch.__file__)))' 2>/dev/null || true)"
[ -n "$_TORCH_SP" ] && export OMNISIM_TORCH_SITE="$_TORCH_SP"
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
  spot) DEF_KE=500;  DEF_KD=60  ;;   # spot_newton_fixed2.xml   kp=500 kv=60
  b2)   DEF_KE=1400; DEF_KD=35  ;;   # b2_newton.xml            kp=1400 kv=35
  *) echo "ERROR: QUAD_ROBOT must be go2|spot|b2 (got '$ROBOT')" >&2; exit 2 ;;
esac
export OMNISIM_NEWTON_TARGET_KE="${OMNISIM_NEWTON_TARGET_KE:-$DEF_KE}"
export OMNISIM_NEWTON_TARGET_KD="${OMNISIM_NEWTON_TARGET_KD:-$DEF_KD}"
export RES_POLICY="${RES_POLICY:-$ROOT/projects/policies/training/runs/gpu_${ROBOT}_inengine/policy.pt}"
WORLD="${WALK_WORLD:-projects/policies/research/worlds/${ROBOT}_walk_deploy.wbt}"

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
      break
    fi
    if [ -f "$RES_LOG" ]; then
      AGE=$(( NOW - $(stat -c %Y "$RES_LOG" 2>/dev/null || echo "$NOW") ))
      if [ "$AGE" -gt "${STALL_S:-1800}" ]; then
        echo "ERROR: STALL -- $RES_LOG silent for ${AGE}s (> STALL_S=${STALL_S:-1800}); killing the sim tree" >&2
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
  echo "=== ${TAG} RL log (last 20) ==="
  tail -20 "$RES_LOG" 2>/dev/null
  exit $RC
fi
