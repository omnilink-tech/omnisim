#!/bin/bash
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

# 50k-step MJX smoke test — run this first on any GPU box to verify the
# JAX stack works, the URDF loads, and ema_return is finite. Should
# complete in ~2-3 minutes on a real GPU.
#
# Usage (from the repo root):
#   bash projects/policies/research/tools/smoke_mjx_train.sh
#
# Override knobs via env vars:
#   ENVS=256 STEPS=100000 bash projects/policies/research/tools/smoke_mjx_train.sh
#
# If JAX can't find your CUDA / cuDNN libs, set LD_LIBRARY_PATH first:
#   export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/path/to/cudnn/lib
set -e

# Polite GPU memory so cuDNN init has scratch space.
export XLA_PYTHON_CLIENT_PREALLOCATE=${XLA_PYTHON_CLIENT_PREALLOCATE:-false}
export XLA_PYTHON_CLIENT_MEM_FRACTION=${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.8}

# Walk-focused shaping (so even a tiny smoke run produces non-zero return).
export OMNIQUAD_R_LIN_VEL_WT=${OMNIQUAD_R_LIN_VEL_WT:-2.0}
export OMNIQUAD_R_ALIVE_BONUS=${OMNIQUAD_R_ALIVE_BONUS:-0.02}

ENVS=${ENVS:-64}
STEPS=${STEPS:-50000}
N_STEPS=${N_STEPS:-24}
BATCH=${BATCH:-$((ENVS * N_STEPS / 4))}
RUN=${RUN_NAME:-omniquad_mjx_smoke}

echo "[smoke] envs=$ENVS steps=$STEPS n_steps=$N_STEPS batch=$BATCH run=$RUN"
echo "[smoke] jax devices:"
python3 -c "import jax; print('  ', jax.devices())"
echo ''

python3 -u projects/policies/research/training/train_robot.py \
    --robot omniquad --backend mjx \
    --envs "$ENVS" --steps "$STEPS" \
    --n-steps "$N_STEPS" --batch-size "$BATCH" \
    --n-epochs 4 --run-name "$RUN" --ckpt-every 100000

echo ''
echo "[smoke] DONE"
echo "[smoke] policy : projects/policies/research/inference/policies/$RUN/policy.onnx"
echo "[smoke] spec   : projects/policies/research/inference/policies/$RUN/robot_spec.json"
