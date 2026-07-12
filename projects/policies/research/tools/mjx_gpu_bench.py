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

"""MJX GPU benchmark with the Spot URDF — measures pure physics
throughput at a range of batch sizes. Useful for sizing the `--envs`
flag for `train_robot.py` on a new GPU box.

Usage (from the repo root):
    python projects/policies/research/tools/mjx_gpu_bench.py

If JAX can't find your CUDA / cuDNN libs, set LD_LIBRARY_PATH first:
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/path/to/cudnn/lib

Read the output for the largest N before OOM — that's your ceiling.
Use 2x or 4x smaller for actual training, to leave headroom for the
PPO update buffers."""
import os
import time
from pathlib import Path
import sys

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.7")

import jax
import jax.numpy as jnp
print("backend:", jax.default_backend(), "devices:", jax.devices())

import mujoco
from mujoco import mjx

# Load Spot URDF through OmniSim's loader (rewrites package:// URIs, adds
# position actuators).
from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
from projects.policies.research.backends.spot_robot_spec import SPOT

print(f"loading Spot URDF: {SPOT.urdf_path}")
m = load_or_convert(SPOT.urdf_path,
                    actuator_joints=list(SPOT.joint_names),
                    joint_limits=list(zip(SPOT.joint_limits_lo, SPOT.joint_limits_hi)))
mx = mjx.put_model(m)
print(f"mujoco model: nq={m.nq} nv={m.nv} nu={m.nu}")


def bench(N, n_steps=200):
    keys = jax.random.split(jax.random.PRNGKey(0), N)
    dxs = jax.vmap(lambda _: mjx.make_data(mx))(keys)
    step = jax.jit(jax.vmap(lambda d: mjx.step(mx, d)))
    # Warmup compile.
    dxs = step(dxs); dxs.qpos.block_until_ready()
    t0 = time.time()
    for _ in range(n_steps):
        dxs = step(dxs)
    dxs.qpos.block_until_ready()
    return N * n_steps / (time.time() - t0)


print("")
print("  N         env-steps/s    per-env steps/s")
for N in [32, 64, 128, 256, 512, 1024, 2048, 4096]:
    try:
        fps = bench(N)
        print(f"  N={N:<6d} -> {fps:>9.0f}     ({fps/N:.1f})", flush=True)
    except Exception as e:
        print(f"  N={N:<6d} -> FAILED: {type(e).__name__}: {str(e)[:80]}", flush=True)
        break
