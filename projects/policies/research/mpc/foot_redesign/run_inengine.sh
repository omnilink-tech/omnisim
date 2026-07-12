#!/usr/bin/env bash
# In-engine A/B driver for the foot-redesign demo. Replicates the proven Newton +
# humanoid_stand_deploy env from scripts/dev/run_humanoid_stand_deploy.ps1 and runs the
# deterministic G1 cube-defense stand on either the ORIG or the BIGFOOT world.
#   bash run_inengine.sh <orig|big> <speed> <period> <duration> [gui]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.."
ROOT="$(pwd -W 2>/dev/null || pwd)"
WHICH="${1:-big}"; SPEED="${2:-4.5}"; PERIOD="${3:-2.0}"; DUR="${4:-16}"; GUI="${5:-}"
if [ "$WHICH" = "big" ]; then
  WORLD=projects/policies/worlds/g1_hstand_cubethrow_bigfoot.wbt
else
  WORLD=projects/policies/worlds/g1_hstand_cubethrow.wbt
fi
TAG="inengine_${WHICH}_s${SPEED}_p${PERIOD}"
LOG=_scratch/foot_redesign/$TAG
export OMNISIM_HOME=$ROOT
export PYTHONUTF8=1
export OMNISIM_LOG_PATH=$ROOT/$LOG.enginelog
export WEBOTS_EXTRA_PROJECT_PATH=$ROOT/projects/policies
# deploy-faithful Newton solver config (identical to the proven G1 stand)
export OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1
export OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_BASE_GUARD=1 OMNISIM_NEWTON_SEED_POSE=1 OMNISIM_NEWTON_SEED_REBUILD=1
export OMNISIM_NEWTON_TARGET_KE=400 OMNISIM_NEWTON_TARGET_KD=60 OMNISIM_NEWTON_GROUND_MU=2.0
# stand controller config (g1 spec drives nominal squat + lean + arm_balance)
export HUMANOID_STAND_SPEC=$ROOT/projects/policies/controllers/humanoid_stand_deploy/specs/g1.json
export HSTAND_ANK_BIAS=-0.06 HSTAND_LEAN=1
export HSTAND_THROW=1 HSTAND_THROW_SPEED=$SPEED HSTAND_THROW_PERIOD=$PERIOD
export THROW_SPEED=$SPEED THROW_PERIOD=$PERIOD
export HUMANOID_STAND_LOG=$ROOT/$LOG.telemetry
export OMNISIM_DEPLOY_LOG=$ROOT/$LOG.telemetry
: > "$LOG.telemetry"   # truncate so each run's telemetry is clean (controller opens "a")

echo ">>> $TAG  world=$WHICH speed=$SPEED period=$PERIOD dur=${DUR}s gui=${GUI:-no}"
if [ "$GUI" = "gui" ]; then
  python -u scripts/dev/headless_runner.py "$WORLD" --gui --realtime --duration "$DUR"
else
  python -u scripts/dev/headless_runner.py "$WORLD" --duration "$DUR" > "$LOG.out" 2>&1
  echo "--- engine fall/telemetry lines ---"
  grep -iE 'FALL@|fell|FELL|topple|z=0\.[0-5]|stand.*ok|OK ' "$LOG.telemetry" 2>/dev/null | tail -8
  grep -iE 'error|exception|require.newton|world finalised|FALL' "$LOG.out" 2>/dev/null | tail -8
fi
