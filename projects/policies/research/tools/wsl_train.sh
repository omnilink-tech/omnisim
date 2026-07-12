#!/bin/bash
# Train Spot via MJX inside WSL Ubuntu + deploy ONNX back to Windows OmniSim.
#
# Run from inside WSL Ubuntu (or `wsl -d Ubuntu-22.04 -- bash <this script>`).
# Writes its outputs back into this same checkout, so the trained policy is
# immediately deployable in Windows OmniSim.
#
# Usage — invoke it by its path inside the WSL mount of your checkout, e.g.
# a repo at D:\omnisim is /mnt/d/omnisim from WSL:
#   wsl -d Ubuntu-22.04 -u root -- bash /mnt/d/omnisim/projects/policies/research/tools/wsl_train.sh
#
# The repo root is derived from this script's own location, so no path needs
# editing. Set OMNISIM_HOME to override (must be a WSL-visible path).
#
# Override the defaults with env vars:
#   STEPS=10000000  ENVS=1024  RUN_NAME=spot_mjx_v1  bash wsl_train.sh
set -e

# tools/ -> research/ -> policies/ -> projects/ -> repo root  (4 levels up).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${OMNISIM_HOME:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"

if [ ! -d "$REPO/projects/policies/research/training" ]; then
    echo "[wsl_train] ERROR: repo root '$REPO' does not look like an OmniSim checkout" >&2
    echo "[wsl_train]        (expected projects/policies/research/training/ under it)." >&2
    echo "[wsl_train]        Set OMNISIM_HOME to the WSL path of your checkout." >&2
    exit 1
fi

cd "$REPO"

STEPS=${STEPS:-10000000}
ENVS=${ENVS:-1024}
RUN_NAME=${RUN_NAME:-spot_mjx_gpu_$(date +%Y%m%d_%H%M%S)}

# Sensible PPO knobs for legged locomotion at this env count.
N_STEPS=${N_STEPS:-24}     # rollout length per env per update
BATCH=${BATCH:-6144}        # ENVS * N_STEPS / 4
EPOCHS=${EPOCHS:-5}

# Walk-focused reward weights so the policy doesn't plateau at stand-still.
export SPOT_R_LIN_VEL_WT=${SPOT_R_LIN_VEL_WT:-2.0}
export SPOT_R_ALIVE_BONUS=${SPOT_R_ALIVE_BONUS:-0.02}

# Keep GPU memory polite so cuDNN can initialise its scratch pool.
export XLA_PYTHON_CLIENT_PREALLOCATE=${XLA_PYTHON_CLIENT_PREALLOCATE:-false}
export XLA_PYTHON_CLIENT_MEM_FRACTION=${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.8}

echo "[wsl_train] cwd=$(pwd)"
echo "[wsl_train] python=$(which python3)"
echo "[wsl_train] jax devices:"
python3 -c "import jax; print('  ', jax.devices())"
echo "[wsl_train] starting training: envs=$ENVS steps=$STEPS run=$RUN_NAME"
echo "[wsl_train] reward weights: lin_vel=$SPOT_R_LIN_VEL_WT alive=$SPOT_R_ALIVE_BONUS"
echo ''

python3 projects/policies/research/training/train_robot.py \
    --robot spot \
    --backend mjx \
    --envs "$ENVS" \
    --steps "$STEPS" \
    --n-steps "$N_STEPS" \
    --batch-size "$BATCH" \
    --n-epochs "$EPOCHS" \
    --run-name "$RUN_NAME" \
    --ckpt-every 200000

# Output paths Windows OmniSim can use directly:
POLICY="$REPO/projects/policies/research/inference/policies/$RUN_NAME/policy.onnx"
SPEC="$REPO/projects/policies/research/inference/policies/$RUN_NAME/robot_spec.json"
echo ''
echo "[wsl_train] DONE"
echo "[wsl_train] policy : $POLICY"
echo "[wsl_train] spec   : $SPEC"
echo ''
echo "[wsl_train] Deploy from Windows PowerShell:"
echo "  \$env:OMNISIM_POLICY_ONNX = 'projects\\policies\\research\\inference\\policies\\$RUN_NAME\\policy.onnx'"
echo "  .\\msys64\\mingw64\\bin\\omnisim-bin.exe projects\\policies\\research\\worlds\\rl_deploy.wbt"
