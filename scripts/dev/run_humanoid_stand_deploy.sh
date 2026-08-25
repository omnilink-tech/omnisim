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

# Deterministic PURE-POSE stand deploy for a humanoid, using the EXACT proven
# G1 recipe (run_g1_standwave_pose_deploy.ps1): stiff Newton position hold of a
# statically-stable squat NOMINAL, no RL policy, ankle PD off by default.
# Generic across robots -- the robot is selected by the first argument and
# described by projects/policies/controllers/humanoid_stand_deploy/specs/<robot>.json.
#
# Usage:
#   bash scripts/dev/run_humanoid_stand_deploy.sh h1 [duration]
#   bash scripts/dev/run_humanoid_stand_deploy.sh g1 --ke 800
#
# The robot argument accepts h1 and g1. "valkyrie" was removed 2026-08-24: the
# robot package went in be41986f8 and took specs/valkyrie.json with it, so the
# accepted set offered a value the spec check below could only ever reject.
#
# Options (mirror the .ps1 switches): --duration N --gui --ke N --kd N
#   --ankle-kp N --ankle-kd N --ank-bias B --hip-bias B --bal-clamp C
#   --lean / --no-lean --capture-step --test-push V --test-push-ang A --test-push-t T
#   --wave --throw --rain --throw-speed V --throw-period S
#   --arms-down --one-leg --manip --arm-motion --squat --mpc --wbc --step-mpc --cmpc
#   --cold --extra-log <path>
#
# NOTE: bash sibling of run_humanoid_stand_deploy.ps1 (the Windows launcher) --
# keep the two in sync: same env vars, same values, same launch command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"   # Windows-style (D:/...) under msys bash, POSIX elsewhere
PY="$(command -v python || command -v python3 || true)"
[ -n "$PY" ] || { echo "ERROR: no python/python3 on PATH" >&2; exit 1; }

ROBOT="${1:-}"
case "$ROBOT" in h1|g1) shift;; *) echo "usage: run_humanoid_stand_deploy.sh <h1|g1> [duration] [options]" >&2; exit 2;; esac

DURATION=20; GUI=0
KE="400"; KD="60"; KE_SET=0; KD_SET=0            # Newton position-PD stiffness (raise for heavy robots)
ANKLE_KP="0"; ANKLE_KD="0"                       # ankle balance PD (0 = pure pose)
ANK_BIAS="0"; ANK_BIAS_SET=0                     # fore/aft trim: ankle-pitch bias (neg = lean back)
HIP_BIAS="0"; HIP_BIAS_SET=0                     # fore/aft trim: hip-pitch bias
BAL_CLAMP="0.2"                                  # max ankle balance correction (rad)
LEAN_SET=0; LEAN=0                               # reactive fore/aft ankle lean
CAPTURE_STEP=0; TEST_PUSH="0"; TEST_PUSH_ANG="0"; TEST_PUSH_T="1.5"
WAVE=0; THROW=0; RAIN=0; THROW_SPEED="4.5"; THROW_PERIOD="2"
ARMS_DOWN=0; ONE_LEG=0; MANIP=0; ARM_MOTION=0; SQUAT=0; MPC=0; WBC=0; STEP_MPC=0; CMPC=0
COLD=0; EXTRALOG=""
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION="$1"; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --duration)      DURATION="$2"; shift 2;;
    --gui)           GUI=1; shift;;
    --ke)            KE="$2"; KE_SET=1; shift 2;;
    --kd)            KD="$2"; KD_SET=1; shift 2;;
    --ankle-kp)      ANKLE_KP="$2"; shift 2;;
    --ankle-kd)      ANKLE_KD="$2"; shift 2;;
    --ank-bias)      ANK_BIAS="$2"; ANK_BIAS_SET=1; shift 2;;
    --hip-bias)      HIP_BIAS="$2"; HIP_BIAS_SET=1; shift 2;;
    --bal-clamp)     BAL_CLAMP="$2"; shift 2;;
    --lean)          LEAN=1; LEAN_SET=1; shift;;
    --no-lean)       LEAN=0; LEAN_SET=1; shift;;
    --capture-step)  CAPTURE_STEP=1; shift;;
    --test-push)     TEST_PUSH="$2"; shift 2;;
    --test-push-ang) TEST_PUSH_ANG="$2"; shift 2;;
    --test-push-t)   TEST_PUSH_T="$2"; shift 2;;
    --wave)          WAVE=1; shift;;
    --throw)         THROW=1; shift;;
    --rain)          RAIN=1; shift;;
    --throw-speed)   THROW_SPEED="$2"; shift 2;;
    --throw-period)  THROW_PERIOD="$2"; shift 2;;
    --arms-down)     ARMS_DOWN=1; shift;;
    --one-leg)       ONE_LEG=1; shift;;
    --manip)         MANIP=1; shift;;
    --arm-motion)    ARM_MOTION=1; shift;;
    --squat)         SQUAT=1; shift;;
    --mpc)           MPC=1; shift;;
    --wbc)           WBC=1; shift;;
    --step-mpc)      STEP_MPC=1; shift;;
    --cmpc)          CMPC=1; shift;;
    --cold)          COLD=1; shift;;
    --extra-log)     EXTRALOG="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

# Prefer a dedicated <robot>_hstand_deploy.wbt for this generic harness; fall back
# to <robot>_stand_deploy.wbt. (G1 needs the _hstand_ name so it does NOT collide
# with the pre-existing g1_stand_deploy.omniworld, which runs G1's own ONNX controller.)
# --rain drops cubes from above (reliable); --throw launches them horizontally.
if [ "$MANIP" = "1" ]; then
  # Table-manipulation world (table + DEF GRASP_CUBE* + side-ring DEF CUBE* for
  # composing with --throw). Lives under research/worlds like the other G1 worlds.
  WORLD="$ROOT/projects/policies/research/worlds/g1_hstand_manip.omniworld"
elif [ "$ARM_MOTION" = "1" ]; then
  # Arm-motion world (stand + side-ring DEF CUBE* for composing with --throw).
  WORLD="$ROOT/projects/policies/research/worlds/g1_arm_motion.omniworld"
elif [ "$RAIN" = "1" ]; then
  WORLD="$ROOT/projects/policies/worlds/${ROBOT}_hstand_cuberain.omniworld"
elif [ "$THROW" = "1" ]; then
  WORLD="$ROOT/projects/policies/worlds/${ROBOT}_hstand_cubethrow.omniworld"
else
  # Prefer the full-arm _hstand_deploy world (projects/policies/worlds OR research/worlds)
  # before the legs-only _stand_deploy fallback. G1's lives in research/worlds.
  CANDS=(
    "$ROOT/projects/policies/worlds/${ROBOT}_hstand_deploy.omniworld"
    "$ROOT/projects/policies/research/worlds/${ROBOT}_hstand_deploy.omniworld"
    "$ROOT/projects/policies/worlds/${ROBOT}_stand_deploy.omniworld"
  )
  WORLD="${CANDS[0]}"
  for c in "${CANDS[@]}"; do if [ -f "$c" ]; then WORLD="$c"; break; fi; done
fi
# Some robots' hstand worlds (G1) live under research/worlds; fall back there.
if [ ! -f "$WORLD" ]; then
  ALT="${WORLD/\/projects\/policies\/worlds\//\/projects\/policies\/research\/worlds\/}"
  if [ -f "$ALT" ]; then WORLD="$ALT"; fi
fi
# Spec select: --one-leg (weight-shift onto one leg) or --arms-down (arms at the
# sides) or the default stand. The arms-down spec uses a reactive arm swing; the
# one-leg spec uses the torque-ankle CoP to hold the CoM over the support foot.
if   [ "$WBC" = "1" ];        then SPEC_NAME="${ROBOT}_tsid"
elif [ "$MPC" = "1" ];        then SPEC_NAME="${ROBOT}_mpc"
elif [ "$MANIP" = "1" ];      then SPEC_NAME="${ROBOT}_manip"
elif [ "$ARM_MOTION" = "1" ]; then SPEC_NAME="${ROBOT}_arm_motion"
elif [ "$ONE_LEG" = "1" ];    then SPEC_NAME="${ROBOT}_oneleg"
elif [ "$SQUAT" = "1" ];      then SPEC_NAME="${ROBOT}_squat"
elif [ "$ARMS_DOWN" = "1" ];  then SPEC_NAME="${ROBOT}_armsdown"
else SPEC_NAME="$ROBOT"; fi
SPEC="$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/${SPEC_NAME}.json"
if [ -n "$EXTRALOG" ]; then LOG="$EXTRALOG"; else LOG="$ROOT/_scratch/stand/${ROBOT}_hstand.log"; fi
mkdir -p "$(dirname "$LOG")"
rm -f "$LOG"

if [ ! -f "$WORLD" ]; then echo "no world $WORLD" >&2; exit 1; fi
if [ ! -f "$SPEC" ];  then echo "no spec $SPEC"  >&2; exit 1; fi

# Per-robot deploy stiffness lives in the spec (deploy_ke/deploy_kd). Use it as
# the default unless the caller passed --ke/--kd explicitly. A heavier robot
# needs a stiffer ankle hold than the light default.
spec_get() {
  "$PY" -c 'import json,sys
v = json.load(open(sys.argv[1])).get(sys.argv[2])
print("" if v is None else ("%g" % v if isinstance(v, (int, float)) else v))' "$SPEC" "$1"
}
if [ "$KE_SET" = "0" ]; then V="$(spec_get deploy_ke)"; if [ -n "$V" ]; then KE="$V"; fi; fi
if [ "$KD_SET" = "0" ]; then V="$(spec_get deploy_kd)"; if [ -n "$V" ]; then KD="$V"; fi; fi
# Likewise the fore/aft trim: don't clobber the spec's ank_bias/hip_bias with the
# option default (0) unless the caller passed it explicitly.
if [ "$ANK_BIAS_SET" = "0" ]; then V="$(spec_get ank_bias)"; if [ -n "$V" ]; then ANK_BIAS="$V"; fi; fi
if [ "$HIP_BIAS_SET" = "0" ]; then V="$(spec_get hip_bias)"; if [ -n "$V" ]; then HIP_BIAS="$V"; fi; fi

export OMNISIM_HOME="$ROOT"
export PYTHONUTF8="1"
# The humanoid_stand_deploy controller lives in projects/policies/controllers, but some
# worlds (G1's) are archived under projects/policies/research/worlds, whose project root
# is projects/policies/research -- so OmniSim can't find the controller there. Add the
# rl project as an extra search path so the controller resolves either way.
export WEBOTS_EXTRA_PROJECT_PATH="$ROOT/projects/policies"
# --- deploy-faithful Newton solver config, IDENTICAL to the proven G1 stand ---
export OMNISIM_NEWTON_STATICS="1"
export OMNISIM_NEWTON_SUBSTEPS="4"
export OMNISIM_NEWTON_FORCE_MUJOCO="1"
export OMNISIM_NEWTON_MJWARP="1"
export OMNISIM_URDF_USE_INERTIA="1"
export OMNISIM_NEWTON_BASE_GUARD="1"
export OMNISIM_NEWTON_SEED_POSE="1"
export OMNISIM_NEWTON_SEED_REBUILD="1"
# --- STIFF position PD (proven stand value; --ke raises it for heavy robots) ---
export OMNISIM_NEWTON_TARGET_KE="$KE"
export OMNISIM_NEWTON_TARGET_KD="$KD"
export OMNISIM_NEWTON_GROUND_MU="2.0"
# --- generic stand controller config ---
export HUMANOID_STAND_SPEC="$SPEC"
# Arms-down balances via reactive arm swing + return-to-home integral. That
# integral absorbs the cold-load articulation offset, so it runs robustly COLD
# (verified) -- no warm reload, which in the GUI reloads the world and disrupts
# the live window.
if [ "$ARMS_DOWN" = "1" ]; then export HSTAND_WARMUP_RELOAD="0"; fi
# One-leg weight-shift: enable the one_leg controller; runs cold (CoP feedback is
# robust to the cold-load offset), warm reload off to keep the GUI window stable.
if [ "$ONE_LEG" = "1" ]; then export HSTAND_ONELEG="1"; export HSTAND_WARMUP_RELOAD="0"; fi
# Table manipulation: enable the manip overlay; warm-reload ON so the arm tracks
# crisply for the grasp (the cold-load articulation undershoot otherwise misses
# the cube). For a live GUI demo, pass warm-reload off via the spec if the reload
# disrupts the window; headless is unaffected.
if [ "$MANIP" = "1" ]; then export HSTAND_MANIP="1"; export HSTAND_WARMUP_RELOAD="1"; fi
# Arm-motion exercises: enable the overlay. Runs COLD like arms-down (the
# return-to-home integrals absorb the cold-load offset, so it's robust without a
# warm reload, which keeps the live GUI window stable). No grasp precision needed.
if [ "$ARM_MOTION" = "1" ]; then export HSTAND_ARM_MOTION="1"; export HSTAND_WARMUP_RELOAD="0"; fi
# Squat: enable the squat overlay. Warm reload ON -- the RISE phase (standing back
# up) is the unstable part of the motion and is timing-sensitive, so we want the
# crisp warm articulation (the cold first-load under-tracks and topples the rise).
# For a live GUI demo pass --cold if the reload disrupts the window.
if [ "$SQUAT" = "1" ]; then export HSTAND_SQUAT="1"; export HSTAND_WARMUP_RELOAD="1"; fi
# MPC brain: enable the sampling-MPC planner. Warm reload ON -- the cold-load
# articulation undershoot otherwise biases the very planner state we read.
if [ "$MPC" = "1" ]; then export HSTAND_MPC="1"; export HSTAND_WARMUP_RELOAD="1"; fi
# WBC: in-engine whole-body controller as operational-space feedforward on the
# position servo (servo holds the centered squat; the WBC adds a per-tick CoM +
# torso-orientation balance torque). Tick-rate so the CUDA graph stays on. Stands
# + survives the designed cube-defense throws. Warm reload ON.
if [ "$WBC" = "1" ]; then
  export HSTAND_WARMUP_RELOAD="1"
  export OMNISIM_INENGINE_TSID="1"; export TSID_TICK="1"; export TSID_FF="1"
  export TSID_KP_COM="35"; export TSID_KD_COM="12"; export TSID_KP_ORI="90"; export TSID_KD_ORI="18"
  export TSID_KP_POST="0"; export TSID_KD_POST="0"; export TSID_KD_JOINT="0"
  export TSID_TAUMAX="120"; export TSID_FF_GRAV="0"
fi
# StepMpc: in-engine push-recovery MPPI on the POSITION SERVO. The deterministic
# stand (default g1 spec, lean + arm-balance overlays) settles the robot during the
# warmup window; then g1_step_mpc captures that pose as the nominal and OWNS the servo
# targets = nominal + MPPI-optimized residual (rolled out in mujoco_warp). Servo mode
# (NO torque), per-TICK (the servo holds between ticks -> CUDA graph stays on, fast).
# Warm reload ON so the captured nominal matches a stabilised session.
if [ "$STEP_MPC" = "1" ]; then
  export HSTAND_WARMUP_RELOAD="1"
  export OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_step_mpc:step_mpc"
  unset OMNISIM_INENGINE_PYMOD_SS
  unset OMNISIM_NEWTON_TORQUE_MODE
  # Capture the in-engine MPC diagnostics (_mpc_log only goes to stderr->DEVNULL
  # in headless otherwise). Sits next to the deploy log.
  if [ -n "$EXTRALOG" ]; then
    export OMNISIM_INENGINE_MPC_LOG="${LOG%.*}._mpc.txt"
  else
    export OMNISIM_INENGINE_MPC_LOG="$ROOT/_scratch/stand/smpc_mpc_${ROBOT}.txt"
  fi
  rm -f "$OMNISIM_INENGINE_MPC_LOG"
fi
# Cmpc: centroidal-MPC + WBC balance (the user's G1-Newton-MPC plan, M4). The force QP
# computes per-foot GRFs to drive CoM+torso to reference; the WBC maps them to joint
# feedforward on the ke=400 servo; the arm CAM adds angular-momentum balance. Stands +
# tracks CoM to ~1mm; deploys in-engine (no obs gap). Warm reload ON.
if [ "$CMPC" = "1" ]; then
  export HSTAND_WARMUP_RELOAD="1"
  export OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_centroidal.driver:cmpc_step"
  export CMPC_STAGE="4"; export CMPC_WARM_TICKS="200"; export CMPC_CAM="1"
  export CMPC_KP_COM="80"; export CMPC_KD_COM="18"
  unset OMNISIM_INENGINE_PYMOD_SS
  unset OMNISIM_NEWTON_TORQUE_MODE
  if [ -n "$EXTRALOG" ]; then
    export OMNISIM_INENGINE_MPC_LOG="${LOG%.*}._cmpc.txt"
  else
    export OMNISIM_INENGINE_MPC_LOG="$ROOT/_scratch/stand/cmpc_${ROBOT}.txt"
  fi
  rm -f "$OMNISIM_INENGINE_MPC_LOG"
fi
# --cold overrides any warm reload (fast iteration; the grasp uses the live hand
# link pose so it still attaches, just with a touch more cold-load slop).
if [ "$COLD" = "1" ]; then export HSTAND_WARMUP_RELOAD="0"; fi
export HSTAND_ANKLE_KP="$ANKLE_KP"
export HSTAND_ANKLE_KD="$ANKLE_KD"
export HSTAND_ANK_BIAS="$ANK_BIAS"
export HSTAND_HIP_BIAS="$HIP_BIAS"
export HSTAND_BAL_CLAMP="$BAL_CLAMP"
if [ "$WAVE" = "1" ]; then export HSTAND_WAVE="1"; else export HSTAND_WAVE="0"; fi
# Lean is per-robot: G1's spec enables it (its stand is marginal), H1 leaves it
# off (it holds passively and the lean only destabilises it). Only
# OVERRIDE the spec when --lean/--no-lean is passed explicitly; otherwise let the
# spec decide.
if [ "$LEAN_SET" = "1" ]; then
  if [ "$LEAN" = "1" ]; then export HSTAND_LEAN="1"; else export HSTAND_LEAN="0"; fi
else
  unset HSTAND_LEAN
fi
if [ "$CAPTURE_STEP" = "1" ]; then export HSTAND_CAPTURE_STEP="1"; else export HSTAND_CAPTURE_STEP="0"; fi
export HSTAND_TEST_PUSH="$TEST_PUSH"
export HSTAND_TEST_PUSH_ANG="$TEST_PUSH_ANG"
export HSTAND_TEST_PUSH_T="$TEST_PUSH_T"
if [ "$THROW" = "1" ]; then export HSTAND_THROW="1"; else export HSTAND_THROW="0"; fi
export HSTAND_THROW_SPEED="$THROW_SPEED"
export HSTAND_THROW_PERIOD="$THROW_PERIOD"
export HUMANOID_STAND_LOG="$LOG"
export OMNISIM_DEPLOY_LOG="$LOG"
# Engine log: per-run unique when --extra-log is given (so parallel runs / a concurrent
# session don't lock each other out of the shared default), else the shared default.
if [ -n "$EXTRALOG" ]; then
  export OMNISIM_LOG_PATH="${LOG%.*}._omnisim.txt"
else
  export OMNISIM_LOG_PATH="$ROOT/_scratch/stand/omnisim_log_${ROBOT}_hstand.txt"
fi

echo "=== $ROBOT deterministic stand (ke=$KE kd=$KD ankle_pd=$ANKLE_KP/$ANKLE_KD) ==="
if [ "$GUI" = "1" ]; then
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --gui --realtime --duration "$DURATION"
else
  "$PY" -u "$ROOT/scripts/dev/headless_runner.py" "$WORLD" --duration "$DURATION" 2>&1 | tail -n 10 || true
fi

echo
echo "===== $ROBOT STAND RESULT ====="
if [ -f "$LOG" ]; then
  FALL="$(grep -c "FALL" "$LOG" || true)"
  SETTLE_LINE="$(grep -m1 "settle done" "$LOG" || true)"
  LAST="$(tail -n 1 "$LOG" || true)"
  if [ -n "$SETTLE_LINE" ]; then echo "  $SETTLE_LINE"; fi
  if [ "${FALL:-0}" -eq 0 ]; then echo "  PASS - no fall. last: $LAST"
  else                            echo "  $FALL FALL line(s). last: $LAST"; fi
  if [ "$SQUAT" = "1" ]; then
    DONE="$(grep "SQUAT_DONE" "$LOG" | tail -n 1 || true)"
    if [ -n "$DONE" ]; then echo "  [SQUAT] $DONE"
    else echo "  [SQUAT] no SQUAT_DONE line (reps may not have completed in --duration)"; fi
  fi
  if [ "$MANIP" = "1" ]; then
    GRASP="$(grep -c "MANIP_GRASP attached" "$LOG" || true)"
    MISS="$(grep -c "MANIP_GRASP_MISS" "$LOG" || true)"
    PLACE="$(grep -c "MANIP_PLACE .* OK" "$LOG" || true)"
    SUCC="$(grep "MANIP_SUCCESS" "$LOG" | tail -n 1 || true)"
    echo "  [MANIP] grasped=$GRASP miss=$MISS placed_ok=$PLACE"
    if [ -n "$SUCC" ]; then echo "  [MANIP] $SUCC"; fi
  fi
  if [ "$ARM_MOTION" = "1" ]; then
    ON="$(grep -m1 "ARM_MOTION on:" "$LOG" || true)"
    EX="$(grep -c "ARM_MOTION -> " "$LOG" || true)"
    if [ -n "$ON" ]; then echo "  [ARM_MOTION] $ON"; fi
    echo "  [ARM_MOTION] exercise transitions: $EX"
  fi
else
  echo "  (no deploy log produced)"
fi
